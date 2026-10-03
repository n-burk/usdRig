// Compile-time validation of scalar connections and weight domains.

#include "rigEvaluatorInternal.h"
#include "rigEvaluatorConstraints.h"
#include "movers/moverRegistry.h"

#include "pxr/usd/usd/attribute.h"
#include "pxr/usd/usd/prim.h"
#include "pxr/usd/usd/relationship.h"
#include "pxr/usd/usdGeom/mesh.h"
#include "pxr/usd/usdGeom/xformable.h"
#include "pxr/usd/usdGeom/pointBased.h"

#include <algorithm>
#include <set>

namespace rigExec {

using namespace evaluatorDetail;

namespace evaluatorDetail {

// Scalar AttributeValue inputs compile to one provider edge. Validate that
// edge recursively so the graph and the independent CPU resolver never choose
// different fallbacks for a malformed, dangling, or cyclic source chain.
bool
_ValidateScalarConnection(
    const UsdStageRefPtr &stage, const UsdAttribute &attribute,
    const SdfValueTypeName &expectedType, std::set<SdfPath> *visiting,
    std::string *error)
{
    if (!attribute) {
        return true;
    }
    if (!visiting->insert(attribute.GetPath()).second) {
        *error = attribute.GetPath().GetString() +
                 ": scalar attribute connection contains a cycle";
        return false;
    }
    struct _EraseOnReturn {
        std::set<SdfPath> *paths;
        SdfPath path;
        ~_EraseOnReturn() { paths->erase(path); }
    } erase{visiting, attribute.GetPath()};

    const SdfPathVector sources = _AuthoredConnections(attribute);
    if (sources.size() > 1) {
        *error = attribute.GetPath().GetString() +
                 ": scalar input must have at most one connection";
        return false;
    }
    if (sources.empty()) {
        return true;
    }
    const UsdAttribute source = stage->GetAttributeAtPath(sources[0]);
    if (!source) {
        *error = attribute.GetPath().GetString() +
                 ": connection targets missing attribute " +
                 sources[0].GetString();
        return false;
    }
    // A float input may read a double source (an avar): the read is cast.
    const bool coerced = expectedType == SdfValueTypeNames->Float &&
                         source.GetTypeName() == SdfValueTypeNames->Double;
    if (source.GetTypeName() != expectedType && !coerced) {
        *error = attribute.GetPath().GetString() +
                 ": connection target " + sources[0].GetString() +
                 " has type " + source.GetTypeName().GetAsToken().GetString() +
                 ", expected " + expectedType.GetAsToken().GetString() +
                 (expectedType == SdfValueTypeNames->Float ? " or double"
                                                           : "");
        return false;
    }
    return _ValidateScalarConnection(
        stage, source, coerced ? SdfValueTypeNames->Double : expectedType,
        visiting, error);
}

// Validates the complete weight-object composition against one mover domain.
// Point domains retain the long-standing PointBased-prim -> .points
// canonicalization. Scalar property and transform domains are exact: there is
// no second value on those targets for the compiler to infer. An atomic
// multi-target mover uses its own prim as a one-element operation domain.
// Every composed input is checked too, so a CombineWeight cannot hide a field
// painted for a different target behind a compatible top-level declaration.
bool
_ValidateWeightObjectDomain(
    const UsdStageRefPtr &stage, const SdfPath &weightPath,
    const SdfPath &moverTarget, bool pointDomain, bool operationDomain,
    size_t logicalCount, std::set<SdfPath> *visiting, std::string *error)
{
    if (!weightPath.IsPrimPath()) {
        *error = "rigExec:weightObject must target a weight-object prim, got " +
                 weightPath.GetString();
        return false;
    }
    const UsdPrim weightPrim = stage->GetPrimAtPath(weightPath);
    if (!weightPrim || !_IsWeightObjectType(weightPrim.GetTypeName())) {
        *error = "rigExec:weightObject targets missing or incompatible prim " +
                 weightPath.GetString();
        return false;
    }
    if (!visiting->insert(weightPath).second) {
        *error = weightPath.GetString() +
                 ": weight object composition contains a cycle";
        return false;
    }
    struct _EraseOnReturn {
        std::set<SdfPath> *paths;
        SdfPath path;
        ~_EraseOnReturn() { paths->erase(path); }
    } erase{visiting, weightPath};

    if (!pointDomain && _IsVolumeWeightType(weightPrim.GetTypeName())) {
        *error = weightPath.GetString() +
                 ": volumetric weights require a point domain";
        return false;
    }

    const TfToken typeName = weightPrim.GetTypeName();
    auto readToken = [&weightPrim](const char *name, const char *fallback) {
        TfToken value(fallback);
        if (const UsdAttribute attr =
                weightPrim.GetAttribute(TfToken(name))) {
            attr.Get(&value, UsdTimeCode::Default());
        }
        return value;
    };
    const TfToken representation = readToken(
        "rigExec:representation",
        (typeName == "RigExecCombineWeight" ||
         _IsVolumeWeightType(typeName)) ? "dense" : "constant");
    const TfToken rangePolicy = readToken(
        "rigExec:rangePolicy",
        (typeName == "RigExecCombineWeight" ||
         _IsVolumeWeightType(typeName)) ? "clamp" : "strict");
    if (rangePolicy != "strict" && rangePolicy != "clamp") {
        *error = weightPath.GetString() +
                 ": unknown rigExec:rangePolicy '" +
                 rangePolicy.GetString() + "'";
        return false;
    }
    if (typeName == "RigExecStaticWeight" ||
        typeName == "RigExecDynamicWeight") {
        if (representation != "constant" && representation != "dense" &&
            representation != "sparse") {
            *error = weightPath.GetString() +
                     ": unknown rigExec:representation '" +
                     representation.GetString() + "'";
            return false;
        }
    } else if (representation != "dense") {
        *error = weightPath.GetString() +
                 ": generated/composed weights require dense "
                 "rigExec:representation";
        return false;
    }

    if (typeName == "RigExecStaticWeight") {
        // Static means the complete descriptor and field are authored once.
        // A time sample or value-producing connection would make Exec consume
        // a changing packet while the CPU oracle and binding epoch treated it
        // as frozen.
        static const TfToken staticFields[] = {
            TfToken("rigExec:values"), TfToken("rigExec:indices"),
            TfToken("rigExec:defaultWeight"),
            TfToken("rigExec:representation"),
            TfToken("rigExec:rangePolicy")};
        for (const TfToken &field : staticFields) {
            const UsdAttribute attr = weightPrim.GetAttribute(field);
            if (attr && (attr.GetNumTimeSamples() > 0 ||
                         attr.HasAuthoredConnections())) {
                *error = weightPath.GetString() + ": static weight field " +
                         field.GetString() +
                         " must not have time samples or connections";
                return false;
            }
        }

        VtFloatArray values;
        VtIntArray indices;
        float defaultWeight = 0.0f;
        if (const UsdAttribute attr = weightPrim.GetAttribute(
                TfToken("rigExec:values"))) {
            attr.Get(&values, UsdTimeCode::Default());
        }
        if (const UsdAttribute attr = weightPrim.GetAttribute(
                TfToken("rigExec:indices"))) {
            attr.Get(&indices, UsdTimeCode::Default());
        }
        if (const UsdAttribute attr = weightPrim.GetAttribute(
                TfToken("rigExec:defaultWeight"))) {
            attr.Get(&defaultWeight, UsdTimeCode::Default());
        }
        // Values and the sparse/constant fallback are value-generation state,
        // not descriptor shape.  Their finite/range policy is enforced while
        // building the current packet so a bad edit fails this application
        // atomically without rebuilding (or invalidating) the whole epoch.
        if (representation == "constant") {
            if (!values.empty() || !indices.empty()) {
                *error = weightPath.GetString() +
                         ": constant weights must not author values or "
                         "indices";
                return false;
            }
        } else if (representation == "dense") {
            if (!indices.empty() || values.size() != logicalCount ||
                defaultWeight != 0.0f) {
                *error = weightPath.GetString() +
                         ": dense weight must have exactly " +
                         std::to_string(logicalCount) +
                         " values, no indices, and canonical "
                         "defaultWeight 0";
                return false;
            }
        } else {
            if (indices.size() != values.size()) {
                *error = weightPath.GetString() +
                         ": sparse index/value size mismatch";
                return false;
            }
            std::set<int> support;
            for (int index : indices) {
                if (index < 0 || static_cast<size_t>(index) >= logicalCount ||
                    !support.insert(index).second) {
                    *error = weightPath.GetString() +
                             ": sparse indices must be unique and within "
                             "the weighted domain";
                    return false;
                }
            }
        }
    }

    if (typeName == "RigExecDynamicWeight") {
        const TfToken operation =
            readToken("rigExec:operation", "multiply");
        if (operation != "multiply") {
            *error = weightPath.GetString() +
                     ": unknown rigExec:operation '" +
                     operation.GetString() + "'";
            return false;
        }
        for (const char *name :
             {"inputs:driver", "inputs:scale", "inputs:bias"}) {
            const UsdAttribute input =
                weightPrim.GetAttribute(TfToken(name));
            std::set<SdfPath> visitingConnections;
            if (!_ValidateScalarConnection(
                    stage, input, SdfValueTypeNames->Float,
                    &visitingConnections, error)) {
                return false;
            }
        }
        SdfPathVector bases;
        if (const UsdRelationship rel = weightPrim.GetRelationship(
                TfToken("rigExec:baseWeight"))) {
            rel.GetTargets(&bases);
        }
        if (bases.size() > 1) {
            *error = weightPath.GetString() +
                     ": rigExec:baseWeight must have at most one target";
            return false;
        }
        if (bases.empty() && representation != "constant") {
            *error = weightPath.GetString() +
                     ": a DynamicWeight without a base must be constant";
            return false;
        }
        if (bases.size() == 1) {
            const UsdPrim base = stage->GetPrimAtPath(bases[0]);
            TfToken actualBaseRepresentation("constant");
            if (base) {
                if (const UsdAttribute attr = base.GetAttribute(
                        TfToken("rigExec:representation"))) {
                    attr.Get(&actualBaseRepresentation,
                             UsdTimeCode::Default());
                }
                if (base.GetTypeName() == "RigExecCombineWeight" ||
                    _IsVolumeWeightType(base.GetTypeName())) {
                    if (!base.GetAttribute(
                            TfToken("rigExec:representation"))) {
                        actualBaseRepresentation = TfToken("dense");
                    }
                }
            }
            if (!base || actualBaseRepresentation != representation) {
                *error = weightPath.GetString() +
                         ": dynamic/base representation mismatch";
                return false;
            }
            if (representation == "sparse") {
                VtIntArray mine, theirs;
                if (const UsdAttribute attr = weightPrim.GetAttribute(
                        TfToken("rigExec:indices"))) {
                    attr.Get(&mine, UsdTimeCode::Default());
                }
                if (const UsdAttribute attr = base.GetAttribute(
                        TfToken("rigExec:indices"))) {
                    attr.Get(&theirs, UsdTimeCode::Default());
                }
                if (!mine.empty() &&
                    std::set<int>(mine.begin(), mine.end()) !=
                        std::set<int>(theirs.begin(), theirs.end())) {
                    *error = weightPath.GetString() +
                             ": dynamic/base sparse support mismatch";
                    return false;
                }
            }
        }
    }

    if (typeName == "RigExecCombineWeight") {
        const TfToken mode = readToken("rigExec:combineMode", "multiply");
        static const std::set<TfToken> modes = {
            TfToken("multiply"), TfToken("add"), TfToken("subtract"),
            TfToken("max"), TfToken("min"), TfToken("average"),
            TfToken("overlay")};
        if (!modes.count(mode)) {
            *error = weightPath.GetString() +
                     ": unknown rigExec:combineMode '" +
                     mode.GetString() + "'";
            return false;
        }
    }
    if (operationDomain) {
        if (representation != "constant") {
            *error = weightPath.GetString() +
                     ": an atomic multi-target mover requires a constant "
                     "one-element weight field";
            return false;
        }
    }

    SdfPathVector declaredTargets;
    if (const UsdRelationship rel =
            weightPrim.GetRelationship(TfToken("rigExec:weightTarget"))) {
        rel.GetTargets(&declaredTargets);
    }
    if (declaredTargets.size() != 1) {
        *error = weightPath.GetString() +
                 ": rigExec:weightTarget must have exactly one target";
        return false;
    }
    const SdfPath declared = pointDomain
        ? _ResolveGeometryInput(stage, declaredTargets[0])
        : declaredTargets[0];
    if (declared != moverTarget) {
        *error = weightPath.GetString() +
                 ": rigExec:weightTarget does not match mover target " +
                 moverTarget.GetString();
        return false;
    }

    for (const char *relName : {"rigExec:inputWeights",
                                "rigExec:baseWeight"}) {
        SdfPathVector inputs;
        if (const UsdRelationship rel =
                weightPrim.GetRelationship(TfToken(relName))) {
            rel.GetTargets(&inputs);
        }
        for (const SdfPath &input : inputs) {
            if (!_ValidateWeightObjectDomain(
                    stage, input, moverTarget, pointDomain, operationDomain,
                    logicalCount, visiting, error)) {
                return false;
            }
        }
    }
    return true;
}

} // namespace evaluatorDetail

bool
RigExecRigEvaluator::_DiscoverMovers(
    std::vector<RigExecMoverRecord> &newMovers,
    std::vector<_SurfaceProjectorRecord> &newSurfaceProjectors,
    size_t &inertMovers,
    std::vector<std::string> *errors, _CompileFailure *failure) const
{
    const auto fail = [failure](const std::string &message,
                                SdfPathVector operations = {}) {
        *failure = {message, std::move(operations)};
        return false;
    };
    // Same channel, different verdict: a notice is reported to the author and
    // the compile CONTINUES. It exists so that "this is not wired up" can be
    // said out loud without being fatal -- an incomplete mover is inert, not a
    // reason to refuse the whole rig. Fatal failures use fail; notices leave compilation running.
    auto reportNotice = [errors](const std::string &message) {
        if (errors) {
            errors->push_back(message);
        }
    };
    // Mover discovery: reverse-sibling post-order walk of the whole composed
    // rig. Carrying rigExec:moves is what makes a prim a mover -- the scope it
    // sits under is an authoring convention, never a requirement (see
    // _GetMoverExecutionOrder). Descendants run before their mover parent;
    // sibling branches run bottom-to-top in usdview (reverse composed child
    // order, spec §4.2).
    /// Mover-bearing prims discovered but skipped because nothing is wired to
    /// their rigExec:moves yet. They are not outputs, but they ARE evidence
    /// that the rig root points somewhere real.
    inertMovers = 0;
    const UsdPrim moverRig = _stage->GetPrimAtPath(_rigPath);
    int ordinal = 0;
    if (moverRig) {
        for (const UsdPrim &prim : _GetMoverExecutionOrder(moverRig)) {
            if (_IsSkippedOperation(prim.GetPath())) {
                continue;
            }
            const UsdRelationship moves = prim.GetRelationship(_movesRel);
            if (!moves) {
                if (_IsFrameConstraintType(prim.GetTypeName())) {
                    return fail(
                        prim.GetTypeName().GetString() + " " +
                        prim.GetPath().GetString() +
                        " has executable constraint semantics but no "
                        "rigExec:moves relationship", {prim.GetPath()});
                }
                // Not a mover: a solver, a control, a joint, a weight, or a
                // grouping scope. Solvers pose joints through the rig-wide
                // solver discovery above; the rest are read, not applied.
                continue;
            }
            if (prim.GetTypeName() == "RigExecSurfaceProjector") {
                // Not a mover record: compile makes it derived targets on
                // the chain it rides, which run on that chain's final
                // points as the normals and extent maintenance do.
                SdfPathVector projected;
                moves.GetTargets(&projected);
                for (const SdfPath &t : projected) {
                    if (!t.IsPropertyPath() || t.GetNameToken() != "points") {
                        return fail(
                            "RigExecSurfaceProjector " +
                            prim.GetPath().GetString() + " target " +
                            t.GetString() + " is not a points property");
                    }
                    _SurfaceProjectorRecord record;
                    record.path = prim.GetPath();
                    record.target = t;
                    newSurfaceProjectors.push_back(std::move(record));
                }
                continue;
            }
            SdfPathVector targets;
            moves.GetTargets(&targets);
            if (targets.empty()) {
                // An unwired mover writes nothing, so it is INERT -- not a
                // reason to fail the rig. The stack is dynamic: disconnecting
                // rigExec:moves is the ordinary interactive edit, and taking
                // every other mover down with it makes a node graph unusable
                // the moment a wire is pulled.
                // This is the same treatment a prim with no rigExec:moves at
                // all already gets just above, with one difference: that case
                // is silent because every scope, control and joint in the rig
                // would otherwise announce itself, while an authored but
                // empty write set is a wire the author meant to connect.
                // So it is skipped and SAID, never skipped silently.
                ++inertMovers;
                reportNotice("Mover has no moves targets: " +
                             prim.GetPath().GetString() +
                             "; it is inert this generation");
                continue;
            }

            RigExecMoverRecord record;
            record.moverPath = prim.GetPath();
            record.schemaType = prim.GetTypeName();
            record.ordinal = ordinal++;

            // Structural/topology properties are never writable move
            // targets (spec §4.2, §7.7).
            static const std::set<TfToken> structuralProperties = {
                TfToken("faceVertexCounts"), TfToken("faceVertexIndices"),
                TfToken("holeIndices"), TfToken("curveVertexCounts"),
                TfToken("cornerIndices"), TfToken("cornerSharpnesses"),
                TfToken("creaseIndices"), TfToken("creaseLengths"),
                TfToken("creaseSharpnesses"), TfToken("subdivisionScheme"),
                TfToken("type"), TfToken("basis"), TfToken("wrap"),
                TfToken("orientation"), TfToken("doubleSided")};
            const SdfPath assetRoot = _rigPath.GetParentPath();

            for (const SdfPath &t : targets) {
                const SdfPath canonical = t;
                const SdfPath primPath = canonical.GetPrimPath();
                // Reject dangling targets (spec §4.2).
                if (!_stage->GetPrimAtPath(primPath)) {
                    return fail("Mover " + prim.GetPath().GetString() +
                                " targets missing prim " +
                                primPath.GetString(), {prim.GetPath()});
                }
                // Cross-rig writes are rejected in v1 (spec §4.2).
                if (!primPath.HasPrefix(assetRoot)) {
                    return fail("Mover " + prim.GetPath().GetString() +
                                " targets outside the rig asset: " +
                                canonical.GetString(), {prim.GetPath()});
                }
                if (canonical.IsPropertyPath()) {
                    if (structuralProperties.count(canonical.GetNameToken())) {
                        return fail(
                            "Mover " + prim.GetPath().GetString() +
                            " targets structural property " +
                            canonical.GetString(), {prim.GetPath()});
                    }
                    // Derived properties are compiler-maintained (spec
                    // §7.6 revised): no authored mover writes them.
                    if (canonical.GetNameToken() == "normals" ||
                        canonical.GetNameToken() == "extent") {
                        return fail(
                            "Mover " + prim.GetPath().GetString() +
                            " targets derived property " +
                            canonical.GetString() +
                            "; normals/extent maintenance is synthesized "
                            "by the compiler", {prim.GetPath()});
                    }
                    if (!_stage->GetAttributeAtPath(canonical)) {
                        return fail(
                            "Mover " + prim.GetPath().GetString() +
                            " targets missing property " +
                            canonical.GetString(), {prim.GetPath()});
                    }
                }
                // Duplicate targets after canonicalization are rejected
                // (spec §6.1).
                if (std::find(record.targets.begin(), record.targets.end(),
                              canonical) != record.targets.end()) {
                    return fail("Mover " + prim.GetPath().GetString() +
                                " has duplicate canonical target " +
                                canonical.GetString(), {prim.GetPath()});
                }
                record.targets.push_back(canonical);
            }

            // FBX-style transform constraints use rigExec:moves as the
            // composition-native replacement for FBX's singleton
            // ConstrainedObject connection. Source constraints therefore
            // write exactly one transform provider. SingleChainIK is the one
            // multi-output exception: its write set is the complete inferred
            // joint chain and is validated during constraint compilation.
            if (_IsSourceFrameConstraintType(record.schemaType)) {
                if (record.targets.size() != 1) {
                    return fail(
                        record.schemaType.GetString() + " " +
                        prim.GetPath().GetString() +
                        " must move exactly one transform-provider prim", {prim.GetPath()});
                }
                const SdfPath &constraintTarget = record.targets[0];
                // <prim>.points names the geometry domain. It is a legal
                // spelling the compiler must recognise, not a malformed
                // transform target; the implementation lands in phase 4.
                if (constraintTarget.IsPropertyPath() &&
                    constraintTarget.GetNameToken() == "points") {
                    const UsdPrim owner =
                        _stage->GetPrimAtPath(constraintTarget.GetPrimPath());
                    if (!owner || !owner.IsA<UsdGeomPointBased>()) {
                        return fail(
                            record.schemaType.GetString() + " " +
                            prim.GetPath().GetString() + " targets " +
                            constraintTarget.GetString() +
                            ", whose owner is not a UsdGeomPointBased prim", {prim.GetPath()});
                    }
                    // Legal: the geometry domain. The prim must still be able
                    // to supply a base frame, because the delta is measured
                    // against it exactly as in the transform domain.
                    if (!UsdGeomXformable(owner)) {
                        return fail(
                            record.schemaType.GetString() + " " +
                            prim.GetPath().GetString() + " targets " +
                            constraintTarget.GetString() +
                            ", whose owner cannot supply a base frame", {prim.GetPath()});
                    }
                } else {
                    // The transform domain: the target must be able to carry
                    // a transform. This is the predicate bindFrameSource
                    // already applies to sources, and it admits any
                    // UsdGeomXformable -- Mesh and BasisCurves included.
                    const UsdPrim targetPrim =
                        constraintTarget.IsPrimPath()
                            ? _stage->GetPrimAtPath(constraintTarget)
                            : UsdPrim();
                    if (!targetPrim || !UsdGeomXformable(targetPrim)) {
                        return fail(
                            record.schemaType.GetString() + " " +
                            prim.GetPath().GetString() + " targets " +
                            constraintTarget.GetString() +
                            ", which is not a transform provider; a "
                            "constraint target must be a UsdGeomXformable", {prim.GetPath()});
                    }
                }
            } else if (record.schemaType ==
                       "RigExecSingleChainIkConstraint") {
                if (record.targets.size() < 2 ||
                    std::any_of(record.targets.begin(), record.targets.end(),
                                [](const SdfPath &p) {
                                    return !p.IsPrimPath();
                                })) {
                    return fail(
                        "RigExecSingleChainIkConstraint " +
                        prim.GetPath().GetString() +
                        " must move every joint in a chain of at least two "
                        "prim targets", {prim.GetPath()});
                }
            } else if (const _ConstraintHandler *unevaluated =
                           _FindConstraintHandler(record.schemaType);
                       unevaluated && !unevaluated->solve &&
                       !unevaluated->dispatchesInline) {
                // A registered operator with neither a solve nor an inline
                // branch has no evaluator at all. Attaching a write set to
                // one would otherwise compile and silently do nothing -- the
                // most dangerous possible behavior. Reading this off the
                // table rather than the type name means a future operator
                // cannot be registered without an evaluator and quietly pass.
                return fail(
                    record.schemaType.GetString() + " " +
                    prim.GetPath().GetString() +
                    " has rigExec:moves but no registered evaluator", {prim.GetPath()});
            }

            // Mover target rules and mover-specific validation, from the
            // mover's own row (see movers/). The generic rules check every
            // target's domain and the single-target cardinality; the row's validate
            // step checks only what is specific to its mover. Unregistered
            // types (constraints, solvers) have no row and are validated by
            // their own checks.
            if (const RigExecMoverHandler *moverHandler =
                    RigExecFindMoverHandler(record.schemaType)) {
                std::string moverError;
                if (!RigExecValidateMoverTargets(
                        _stage, prim, moverHandler, record.targets,
                        &moverError)) {
                    return fail(moverError, {prim.GetPath()});
                }
                if (moverHandler->validate) {
                    const RigExecMoverValidateContext moverCtx{
                        _stage, prim, record.targets, moverHandler};
                    if (!moverHandler->validate(moverCtx, &moverError)) {
                        return fail(moverError, {prim.GetPath()});
                    }
                }
            }

            // Universal MoverAPI envelope. A mover either broadcasts its
            // normalized inputs:defaultWeight or binds one compatible total
            // weight field; the relationship never multiplies the scalar.
            {
                for (const auto &commonInput : {
                         std::make_pair("inputs:defaultWeight",
                                        SdfValueTypeNames->Float),
                         std::make_pair("inputs:enabled",
                                        SdfValueTypeNames->Bool)}) {
                    std::set<SdfPath> visitingConnections;
                    std::string connectionError;
                    if (!_ValidateScalarConnection(
                            _stage,
                            prim.GetAttribute(TfToken(commonInput.first)),
                            commonInput.second, &visitingConnections,
                            &connectionError)) {
                        return fail(
                            record.schemaType.GetString() + " " +
                            prim.GetPath().GetString() + ": " +
                            connectionError, {prim.GetPath()});
                    }
                }
                SdfPathVector weightObjects;
                if (const UsdRelationship rel = prim.GetRelationship(
                        TfToken("rigExec:weightObject"))) {
                    rel.GetTargets(&weightObjects);
                }
                if (weightObjects.size() > 1) {
                    return fail(
                        record.schemaType.GetString() + " " +
                        prim.GetPath().GetString() +
                        " binds more than one rigExec:weightObject", {prim.GetPath()});
                }
                if (!weightObjects.empty()) {
                    // A mover with multiple write targets is one atomic
                    // operation envelope, not an ambiguous spatial field per
                    // target. It therefore binds a constant, one-element
                    // field whose weightTarget is the mover itself; that
                    // scalar broadcasts to every application/joint.
                    const bool operationDomain = record.targets.size() != 1;
                    const SdfPath target = operationDomain
                        ? prim.GetPath()
                        : record.targets[0];
                    const UsdPrim owner = target.IsPropertyPath()
                        ? _stage->GetPrimAtPath(target.GetPrimPath())
                        : UsdPrim();
                    const bool pointDomain =
                        !operationDomain && target.IsPropertyPath() &&
                        target.GetNameToken() == "points" && owner &&
                        owner.IsA<UsdGeomPointBased>();
                    size_t logicalCount = 1;
                    if (pointDomain) {
                        const UsdAttribute pointsAttr =
                            _stage->GetAttributeAtPath(target);
                        VtVec3fArray points;
                        std::vector<double> sampleTimes;
                        if (pointsAttr) {
                            pointsAttr.GetTimeSamples(&sampleTimes);
                        }
                        const bool hasAuthoredDefault =
                            pointsAttr &&
                            pointsAttr.GetResolveInfo(UsdTimeCode::Default())
                                    .GetSource() ==
                                UsdResolveInfoSourceDefault;
                        bool resolvedCardinality = false;
                        if (hasAuthoredDefault &&
                            pointsAttr.Get(
                                &points, UsdTimeCode::Default())) {
                            logicalCount = points.size();
                            resolvedCardinality = true;
                        }
                        for (double sampleTime : sampleTimes) {
                            VtVec3fArray sampled;
                            if (!pointsAttr.Get(
                                    &sampled, UsdTimeCode(sampleTime))) {
                                continue;
                            }
                            if (!resolvedCardinality) {
                                logicalCount = sampled.size();
                                resolvedCardinality = true;
                            } else if (sampled.size() != logicalCount) {
                                return fail(
                                    record.schemaType.GetString() + " " +
                                    prim.GetPath().GetString() +
                                    ": weighted point-domain cardinality "
                                    "changes across the binding epoch at " +
                                    target.GetString(), {prim.GetPath()});
                            }
                        }
                        if (!resolvedCardinality && pointsAttr &&
                            pointsAttr.Get(
                                &points, UsdTimeCode::Default())) {
                            // Schema fallback (usually an empty array) is the
                            // only remaining base when neither a default nor
                            // a sample is authored.
                            logicalCount = points.size();
                            resolvedCardinality = true;
                        }
                        if (!resolvedCardinality) {
                            return fail(
                                record.schemaType.GetString() + " " +
                                prim.GetPath().GetString() +
                                ": cannot resolve the weighted point "
                                "domain's compile-time cardinality at " +
                                target.GetString(), {prim.GetPath()});
                        }
                    }
                    std::set<SdfPath> visitingWeights;
                    std::string weightError;
                    if (!_ValidateWeightObjectDomain(
                            _stage, weightObjects[0], target, pointDomain,
                            operationDomain, logicalCount, &visitingWeights,
                            &weightError)) {
                        return fail(
                            record.schemaType.GetString() + " " +
                            prim.GetPath().GetString() + ": " + weightError, {prim.GetPath()});
                    }
                }
            }


            // Strict migration: this mover's concrete envelope property was
            // replaced by MoverAPI inputs:defaultWeight. Once removed from the
            // schema, an old layer opinion composes as a custom attribute and
            // would otherwise be ignored silently. The predecessor name comes
            // from the mover's own row; movers without one have nothing to reject.
            {
                const RigExecMoverHandler *migrationHandler =
                    RigExecFindMoverHandler(record.schemaType);
                const char *oldName = migrationHandler
                    ? migrationHandler->legacyEnvelopeAttribute
                    : nullptr;
                if (oldName) {
                    const UsdAttribute old =
                        prim.GetAttribute(TfToken(oldName));
                    if (old && old.HasAuthoredValue()) {
                        return fail(
                            record.schemaType.GetString() + " " +
                            prim.GetPath().GetString() + " authors " + oldName +
                            ", which was replaced by inputs:defaultWeight", {prim.GetPath()});
                    }
                }
            }
            // The read-phase attributes were replaced by rigExecReadPhase
            // metadata on the input relationship. An old opinion would now
            // compose as an inert custom attribute, so it is refused.
            {
                static const std::pair<const char *, const char *>
                    kRemovedPhases[] = {
                        {"rigExec:transformReadPhase",
                         "rigExec:transform, rigExec:influences or "
                         "rigExec:driverTransforms"},
                        {"rigExec:cageReadPhase", "rigExec:cage"},
                        {"rigExec:surfaceReadPhase", "rigExec:surface"},
                        {"rigExec:driverCurveReadPhase",
                         "rigExec:driverCurve"},
                        {"rigExec:pointsReadPhase", "rigExec:targetPoints"},
                    };
                for (const auto &[oldName, input] : kRemovedPhases) {
                    const UsdAttribute old =
                        prim.GetAttribute(TfToken(oldName));
                    if (old && old.HasAuthoredValue()) {
                        return fail(record.schemaType.GetString() + " " +
                                        prim.GetPath().GetString() +
                                        " authors " + oldName +
                                        ", which was replaced by "
                                        "rigExecReadPhase metadata on " +
                                        input,
                                    {prim.GetPath()});
                    }
                }
            }
            // Read phases, validated from the AUTHORED stage.
            // Binding resolution parses these too, but it has to be total --
            // it returns a binding, not a verdict -- so an unparseable phase
            // there degrades to `base`. That is the wrong answer delivered
            // silently: the author asked for a specific revision and got the
            // authored value. The parse verdict belongs here, in Phase A,
            // where it can reject the compile before any epoch state moves.
            {
                static const char *const kPhased[] = {
                    "rigExec:transform", "rigExec:influences",
                    "rigExec:driverTransforms", "rigExec:cage",
                    "rigExec:surface", "rigExec:bindCoordinates",
                    "rigExec:driverCurve"};
                for (const char *relName : kPhased) {
                    RigExecReadPhase phase;
                    std::string phaseError;
                    if (!RigExecResolveReadPhase(
                            prim.GetRelationship(TfToken(relName)), &phase,
                            &phaseError)) {
                        return fail(record.schemaType.GetString() + " " +
                                    prim.GetPath().GetString() + ": " +
                                    phaseError, {prim.GetPath()});
                    }
                }
            }
            if (const RigExecMoverHandler *handler =
                    RigExecFindMoverHandler(record.schemaType);
                handler && handler->assembleExternal) {
                for (const UsdRelationship &rel : prim.GetRelationships()) {
                    RigExecReadPhase phase;
                    std::string phaseError;
                    if (!RigExecResolveReadPhase(rel, &phase,
                                                 &phaseError)) {
                        return fail(record.schemaType.GetString() + " " +
                                    prim.GetPath().GetString() + ": " +
                                    phaseError, {prim.GetPath()});
                    }
                }
            }
            // Structure-determining tokens must be static. `uniform` is a
            // convention, not an enforcement: USD permits time samples on a
            // uniform attribute, and these tokens select the compiled
            // operation and read phase. Sampling them per-frame would let the
            // op or binding change under a compiled epoch without changing
            // the binding-epoch digest, so reject samples here rather than
            // resolving them at evaluation time. The same rule
            // the aggregate cardinality attributes already follow.
            std::vector<const char *> structuralTokens = {
                "rigExec:mode", "rigExec:operation"};
            // Which operators carry a rotation order is a table column, not
            // a list of type names repeated at each site that asks.
            const _ConstraintHandler *orderHandler =
                _FindConstraintHandler(record.schemaType);

            // No authored constraint property is ever silently ignored. The
            // family's recurring defect was the opposite: rigExec:rotationOrder
            // on a Position constraint compiled and did nothing, and
            // inputs:affectX meant a different channel on each operator. Every
            // channel property is now checked against what the operator
            // actually honors.
            if (orderHandler) {
                // Spelled only when a message needs it: this runs for every
                // mover, and almost none of them has anything to report.
                const auto who = [&record, &prim]() {
                    return record.schemaType.GetString() + " " +
                           prim.GetPath().GetAsString();
                };
                const auto authored = [&prim](const char *name) {
                    const UsdAttribute a = prim.GetAttribute(TfToken(name));
                    return a && a.HasAuthoredValue();
                };

                // Renamed when the envelope and the per-element weight field
                // collapsed into one concept. The old name is no longer part
                // of the schema, so an authored opinion would compose as a
                // custom property and be ignored.
                if (authored("inputs:weight")) {
                    return fail(who() +
                                " authors inputs:weight, which a constraint no"
                                " longer has; the envelope is now"
                                " inputs:defaultWeight", {prim.GetPath()});
                }

                // Replaced by the group-qualified spelling, because it named
                // a different channel on every operator that had it.
                for (const char *legacyMask :
                     {"inputs:affectX", "inputs:affectY", "inputs:affectZ"}) {
                    if (authored(legacyMask)) {
                        return fail(
                            who() + " authors " + legacyMask +
                            ", which named a different channel on every"
                            " operator; use the group-qualified spelling"
                            " (inputs:affectTranslation*, affectRotation* or"
                            " affectScale*)", {prim.GetPath()});
                    }
                }

                struct _ChannelProperty {
                    const char *name;
                    _ChannelGroup group;
                    bool isMask;
                };
                static const _ChannelProperty kChannelProperties[] = {
                    {"inputs:affectTranslationX", _ChannelGroup::Translation, true},
                    {"inputs:affectTranslationY", _ChannelGroup::Translation, true},
                    {"inputs:affectTranslationZ", _ChannelGroup::Translation, true},
                    {"inputs:affectRotationX", _ChannelGroup::Rotation, true},
                    {"inputs:affectRotationY", _ChannelGroup::Rotation, true},
                    {"inputs:affectRotationZ", _ChannelGroup::Rotation, true},
                    {"inputs:affectScaleX", _ChannelGroup::Scale, true},
                    {"inputs:affectScaleY", _ChannelGroup::Scale, true},
                    {"inputs:affectScaleZ", _ChannelGroup::Scale, true},
                    {"inputs:translationOffset", _ChannelGroup::Translation, false},
                    {"inputs:rotationOffset", _ChannelGroup::Rotation, false},
                    {"inputs:scaleOffset", _ChannelGroup::Scale, false},
                };
                for (const _ChannelProperty &channel : kChannelProperties) {
                    if (!authored(channel.name)) {
                        continue;
                    }
                    const _ChannelGroup honored = channel.isMask
                        ? orderHandler->maskGroup
                        : orderHandler->offsetGroup;
                    if (honored == channel.group ||
                        (channel.isMask && honored == _ChannelGroup::All)) {
                        continue;
                    }
                    return fail(
                        who() + " authors " + channel.name + ", which it does "
                        "not honor; the operator writes a different channel "
                        "group" +
                        (orderHandler->offsetGroup == _ChannelGroup::None &&
                         !channel.isMask
                             ? " and composes per-source offset arrays instead"
                             : ""), {prim.GetPath()});
                }

                if (authored("rigExec:rotationOrder") &&
                    !orderHandler->usesRotationOrder) {
                    return fail(who() +
                                " authors rigExec:rotationOrder, which it does"
                                " not honor; only the operators that compose a"
                                " rotation read it", {prim.GetPath()});
                }

                // rigExec:preserve is the legacy channel mask at opposite
                // polarity: it names the components the solve must leave
                // alone, where inputs:affect* names the ones it writes. Its
                // default ["origin", "scale"] says an aim writes orientation
                // only, which is exactly what the kernel does -- it modifies
                // the decomposed rotation and reconstructs, leaving
                // translation and scale untouched. So the default needs no
                // implementation; it is already the behavior.
                // Any OTHER value does not. ["scale"] alone would ask an aim
                // to move the origin too, which requires writing the
                // translation group that Aim does not write. That is the
                // dangerous case today: accepted and silently ignored.
                if (const UsdAttribute preserve =
                        prim.GetAttribute(TfToken("rigExec:preserve"))) {
                    VtTokenArray value;
                    if (preserve.HasAuthoredValue() && preserve.Get(&value)) {
                        const VtTokenArray expected{TfToken("origin"),
                                                    TfToken("scale")};
                        if (value != expected) {
                            return fail(
                                who() +
                                " authors a non-default rigExec:preserve;"
                                " only [\"origin\", \"scale\"] is"
                                " implemented, which is the orientation-only"
                                " solve the kernel already performs. Use the"
                                " inputs:affect* masks to vary which channels"
                                " are written", {prim.GetPath()});
                        }
                    }
                }
            }
            if (orderHandler && orderHandler->usesRotationOrder) {
                structuralTokens.push_back("rigExec:rotationOrder");
            }
            if (record.schemaType == "RigExecAimConstraint") {
                structuralTokens.insert(
                    structuralTokens.end(),
                    {"rigExec:worldUpType",
                     "rigExec:aimAxis", "rigExec:upPolicy"});
            } else if (record.schemaType ==
                       "RigExecSingleChainIkConstraint") {
                structuralTokens.insert(
                    structuralTokens.end(),
                    {"rigExec:solverMode", "rigExec:poleVectorMode",
                     "rigExec:evaluationMode", "rigExec:orientationMode"});
            }
            for (const char *name : structuralTokens) {
                const UsdAttribute a = prim.GetAttribute(TfToken(name));
                if (a && a.GetNumTimeSamples() > 0) {
                    return fail(record.schemaType.GetString() + " " +
                                prim.GetPath().GetString() + ": " + name +
                                " must not be time-sampled (it selects the "
                                "compiled operation/read phase)", {prim.GetPath()});
                }
            }

            auto validateToken = [&](const char *name,
                                     std::initializer_list<const char *> allowed) {
                const UsdAttribute attr = prim.GetAttribute(TfToken(name));
                if (!attr) {
                    return true;
                }
                TfToken value;
                if (!attr.Get(&value) || !_TokenIsOneOf(value, allowed)) {
                    return fail(record.schemaType.GetString() + " " +
                                prim.GetPath().GetString() + ": " + name +
                                " has unsupported value '" +
                                value.GetString() + "'", {prim.GetPath()});
                }
                return true;
            };
            static const std::initializer_list<const char *> eulerOrders = {
                "XYZ", "XZY", "YXZ", "YZX", "ZXY", "ZYX"};
            if (orderHandler && orderHandler->usesRotationOrder &&
                !validateToken("rigExec:rotationOrder", eulerOrders)) {
                return false;
            }
            if (record.schemaType == "RigExecAimConstraint" &&
                (!validateToken(
                     "rigExec:worldUpType",
                     {"sceneUp", "objectUp", "objectRotationUp", "vector",
                      "none"}) ||
                 !validateToken("rigExec:aimAxis", {"x", "y", "z"}) ||
                 !validateToken(
                     "rigExec:upPolicy", {"preserveInputUp"}))) {
                return false;
            }
            if (record.schemaType == "RigExecSingleChainIkConstraint" &&
                (!validateToken(
                     "rigExec:solverMode", {"rotatePlane", "singleChain"}) ||
                 !validateToken(
                     "rigExec:poleVectorMode", {"vector", "object"}) ||
                 !validateToken(
                     "rigExec:evaluationMode",
                     {"neverTS", "autoDetect", "alwaysTS"}) ||
                 !validateToken("rigExec:orientationMode", {"aimX", "preserve"}))) {
                return false;
            }
            newMovers.push_back(std::move(record));
        }
    }

    return true;
}

} // namespace rigExec
