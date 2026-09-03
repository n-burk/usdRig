//
// RigExec rig evaluator implementation.
//
#include "rigEvaluator.h"

#include "frameExtraction.h"
#include "rigExecMath/envelope.h"
#include "rigExecMath/geometryKernels.h"
#include "rigExecMath/propertyMath.h"
#include "rigExecMath/singleChainIk.h"
#include "rigExecMath/solvers.h"
#include "rigExecMath/weightFields.h"

#include "pxr/base/gf/rotation.h"
#include "pxr/base/ts/spline.h"
#include "pxr/base/tf/diagnostic.h"
#include "pxr/base/tf/stringUtils.h"
#include "pxr/usd/usd/attribute.h"
#include "pxr/usd/usd/prim.h"
#include "pxr/usd/usd/primRange.h"
#include "pxr/usd/usd/relationship.h"
#include "pxr/usd/usdGeom/basisCurves.h"
#include "pxr/usd/usdGeom/curves.h"
#include "pxr/usd/usdGeom/gprim.h"
#include "pxr/usd/usdGeom/imageable.h"
#include "pxr/usd/usdGeom/mesh.h"
#include "pxr/usd/usdGeom/metrics.h"
#include "pxr/usd/usdGeom/xformCache.h"
#include "pxr/usd/usdGeom/xformable.h"
#include "pxr/usd/usdGeom/pointBased.h"
#include "pxr/usd/usdGeom/points.h"

#include <algorithm>
#include <cmath>
#include <optional>
#include <functional>
#include <set>

namespace rigExec {

namespace {

// A static input read that prefers what the current generation already
// resolved.
//
// The evaluator and the packet assemblers both read a mover's authored
// inputs directly off the stage; neither goes through exec, so neither sees
// a value override. Routing both through this is what makes a property
// mover's result reach them -- and, just as importantly, what keeps the
// graph path and the CPU oracle reading the SAME number, so parity stays a
// real check rather than two copies of the same mistake.
template <class T>
T
_ResolvedRead(const RigExecResolvedInputs &resolved, const UsdPrim &prim,
              const char *name, T fallback, UsdTimeCode time)
{
    T value = fallback;
    if (!prim) {
        return value;
    }
    const TfToken token(name);
    if (const UsdAttribute a = prim.GetAttribute(token)) {
        resolved.GetAttribute(a, time, &value);
    }
    return value;
}

}  // namespace

namespace {

const TfToken _computePointFrame("computePointFrame");
const TfToken _computePointFrameArray("computePointFrameArray");
const TfToken _movesRel("rigExec:moves");
const TfToken _enabledAttr("inputs:enabled");
const TfToken _restPointsAttr("rigExec:restPoints");
const TfToken _computeFalloffLut("computeFalloffLut");
const TfToken _falloffProfileAttr("rigExec:falloffProfile");
const TfToken _falloffCurveAttr("rigExec:falloffCurve");
const TfToken _samplePhaseAttr("rigExec:samplePhase");

// usdview presents a prim's composed children from top to bottom. Movers use
// that namespace as a stack, so the bottom branch executes first, each mover
// parent executes after all of its descendants, and the top branch executes
// last. This is therefore post-order within a branch and REVERSE composed
// child order between sibling branches.
//
// Keep this as the one traversal primitive for both the structural digest and
// compilation. If those walks ever disagree, an order edit can retain the old
// epoch digest while executing a different chain.
std::vector<UsdPrim>
_GetMoverExecutionOrder(const UsdPrim &movers)
{
    std::vector<UsdPrim> ordered;
    if (movers) {
        // Reversing an ordinary composed pre-order yields exactly the stack
        // walk: reversed sibling branches, recursively, with each parent
        // after its descendants. Using UsdPrimRange here also preserves its
        // standard traversal predicate and instance behavior.
        for (const UsdPrim &prim : UsdPrimRange(movers)) {
            ordered.push_back(prim);
        }
        std::reverse(ordered.begin(), ordered.end());
    }
    return ordered;
}

/// True for the schema types that GENERATE a weight field from a placed
/// volume, as opposed to storing or modulating one.
bool
_IsVolumeWeightType(const TfToken &typeName)
{
    return typeName == "RigExecSphereWeight" ||
           typeName == "RigExecPlaneWeight" ||
           typeName == "RigExecCurveWeight";
}

/// True for every schema that publishes computeWeightPacket.
///
/// The volumetric types are NOT RigExecWeightObject subclasses -- a typed
/// schema gets exactly one base and they spend it on RigExecXformable, to
/// be placeable (see the RigExecVolumeWeight schema doc) -- so weight-object
/// identity is a type-name question here rather than an IsA one. That is
/// what the rest of this file already does for RigExecDynamicWeight.
bool
_IsWeightObjectType(const TfToken &typeName)
{
    return typeName == "RigExecStaticWeight" ||
           typeName == "RigExecDynamicWeight" ||
           typeName == "RigExecCombineWeight" ||
           _IsVolumeWeightType(typeName);
}

/// Everything a constraint solve needs that is common to every operator. The
/// per-operator reads -- offsets, masks -- happen inside the solve, because
/// that is exactly what differs between operators.
struct _ConstraintSolveContext {
    const RigExecResolvedInputs *resolved = nullptr;
    UsdPrim prim;
    UsdTimeCode time;
    RigExecPointFrame inputFrame;
    const std::vector<RigExecConstraintSource> *sources = nullptr;
    RigExecConstraintAxisMask affect;
    RigExecEulerOrder order = RigExecEulerOrder::XYZ;
    double weight = 1.0;
};

using _ConstraintSolveFn =
    RigExecPointFrame (*)(const _ConstraintSolveContext &);

// Bodies sit next to the dispatch they replaced.
RigExecPointFrame _SolvePositionConstraint(const _ConstraintSolveContext &);
RigExecPointFrame _SolveRotationConstraint(const _ConstraintSolveContext &);
RigExecPointFrame _SolveScaleConstraint(const _ConstraintSolveContext &);
RigExecPointFrame _SolveParentConstraint(const _ConstraintSolveContext &);

/// One row per constraint operator. This table is the single source of truth
/// about which operators exist and what each one honors; adding an operator
/// is a row here plus its solve, not an edit at every dispatch site.
///
/// solve == nullptr with dispatchesInline == true means the operator is
/// evaluated by a bespoke branch in Evaluate() because it consumes evaluator
/// state a uniform context cannot carry: Aim resolves a world-up binding
/// against other providers, SingleChainIk writes an inferred joint chain
/// atomically. They are still registered, so the table remains the complete
/// answer to "which operators exist".
///
/// solve == nullptr with dispatchesInline == false means a registered
/// operator has no evaluator at all and must not silently do nothing.
/// The channel group an operator's per-axis mask addresses. Masks and offsets
/// are spelled by (group, axis) on the base class, so an operator declares
/// which group is "its" channel rather than each inventing a name.
/// Parent writes all three and reads them itself; None means the operator
/// honors no mask at all.
enum class _ChannelGroup { None, Translation, Rotation, Scale, All };

struct _ConstraintHandler {
    const char *schemaType;
    bool sourceFrame;      ///< blends rigExec:sources into one revision
    bool frameConstraint;  ///< compiles to frame wiring at all
    bool usesRotationOrder;
    bool dispatchesInline;
    _ChannelGroup maskGroup;
    /// Which scalar offset the operator honors. Parent is None: it composes
    /// PER-SOURCE offset ARRAYS instead, so the inherited scalar offsets
    /// would be silently ignored on it.
    _ChannelGroup offsetGroup;
    _ConstraintSolveFn solve;
};

const std::vector<_ConstraintHandler> &
_ConstraintHandlers()
{
    static const std::vector<_ConstraintHandler> handlers = {
        {"RigExecAimConstraint", true, true, true, true,
         _ChannelGroup::Rotation, _ChannelGroup::Rotation, nullptr},
        {"RigExecPositionConstraint", true, true, false, false,
         _ChannelGroup::Translation, _ChannelGroup::Translation,
         _SolvePositionConstraint},
        {"RigExecRotationConstraint", true, true, true, false,
         _ChannelGroup::Rotation, _ChannelGroup::Rotation,
         _SolveRotationConstraint},
        {"RigExecScaleConstraint", true, true, false, false,
         _ChannelGroup::Scale, _ChannelGroup::Scale,
         _SolveScaleConstraint},
        {"RigExecParentConstraint", true, true, true, false,
         _ChannelGroup::All, _ChannelGroup::None,
         _SolveParentConstraint},
        {"RigExecSingleChainIkConstraint", false, true, false, true,
         _ChannelGroup::None, _ChannelGroup::None, nullptr},
    };
    return handlers;
}

const _ConstraintHandler *
_FindConstraintHandler(const TfToken &typeName)
{
    for (const _ConstraintHandler &handler : _ConstraintHandlers()) {
        if (typeName == handler.schemaType) {
            return &handler;
        }
    }
    return nullptr;
}

/// Source-blending constraints that revise one transform provider.
bool
_IsSourceFrameConstraintType(const TfToken &typeName)
{
    const _ConstraintHandler *handler = _FindConstraintHandler(typeName);
    return handler && handler->sourceFrame;
}

/// Every built-in constraint with fixed evaluator semantics.
bool
_IsFrameConstraintType(const TfToken &typeName)
{
    const _ConstraintHandler *handler = _FindConstraintHandler(typeName);
    return handler && handler->frameConstraint;
}

RigExecEulerOrder
_ParseConstraintEulerOrder(const TfToken &token)
{
    if (token == "XZY") return RigExecEulerOrder::XZY;
    if (token == "YXZ") return RigExecEulerOrder::YXZ;
    if (token == "YZX") return RigExecEulerOrder::YZX;
    if (token == "ZXY") return RigExecEulerOrder::ZXY;
    if (token == "ZYX") return RigExecEulerOrder::ZYX;
    return RigExecEulerOrder::XYZ;
}

bool
_IsUsableConstraintFrame(const RigExecPointFrame &frame)
{
    if (!frame.IsValid() || frame.IsDegenerate()) {
        return false;
    }
    for (const GfVec3d &point : frame.points) {
        if (!std::isfinite(point[0]) || !std::isfinite(point[1]) ||
            !std::isfinite(point[2])) {
            return false;
        }
    }
    return true;
}

bool
_TokenIsOneOf(const TfToken &value,
              std::initializer_list<const char *> allowed)
{
    return std::any_of(
        allowed.begin(), allowed.end(),
        [&value](const char *candidate) { return value == candidate; });
}

RigExecConstraintAxisMask
_ReadConstraintAxisMask(
    const RigExecResolvedInputs &resolved, const UsdPrim &prim,
    const char *x, const char *y, const char *z, UsdTimeCode time,
    bool fallback = true)
{
    RigExecConstraintAxisMask mask;
    mask.x = _ResolvedRead(resolved, prim, x, fallback, time);
    mask.y = _ResolvedRead(resolved, prim, y, fallback, time);
    mask.z = _ResolvedRead(resolved, prim, z, fallback, time);
    return mask;
}

/// Reads the per-axis mask for an operator's own channel group. The masks
/// are spelled by (group, axis) on the base class, so which triple to read is
/// a property of the operator, not of the call site.
RigExecConstraintAxisMask
_ReadGroupMask(const RigExecResolvedInputs &resolved, const UsdPrim &prim,
               _ChannelGroup group, UsdTimeCode time)
{
    switch (group) {
    case _ChannelGroup::Translation:
        return _ReadConstraintAxisMask(
            resolved, prim, "inputs:affectTranslationX",
            "inputs:affectTranslationY", "inputs:affectTranslationZ", time);
    case _ChannelGroup::Rotation:
        return _ReadConstraintAxisMask(
            resolved, prim, "inputs:affectRotationX", "inputs:affectRotationY",
            "inputs:affectRotationZ", time);
    case _ChannelGroup::Scale:
        return _ReadConstraintAxisMask(
            resolved, prim, "inputs:affectScaleX", "inputs:affectScaleY",
            "inputs:affectScaleZ", time);
    case _ChannelGroup::All:
    case _ChannelGroup::None:
        break;
    }
    // Parent reads all three groups itself; the operators that honor no mask
    // are unmasked. Both want the all-true identity.
    return RigExecConstraintAxisMask();
}

RigExecPointFrame
_SolvePositionConstraint(const _ConstraintSolveContext &c)
{
    RigExecPositionConstraintParams params;
    params.offset = _ResolvedRead(
        *c.resolved, c.prim, "inputs:translationOffset", GfVec3d(0), c.time);
    params.affect = c.affect;
    params.weight = c.weight;
    return RigExecApplyPositionConstraint(c.inputFrame, *c.sources, params);
}

RigExecPointFrame
_SolveRotationConstraint(const _ConstraintSolveContext &c)
{
    RigExecRotationConstraintParams params;
    params.offsetDegrees = _ResolvedRead(
        *c.resolved, c.prim, "inputs:rotationOffset", GfVec3d(0), c.time);
    params.affect = c.affect;
    params.rotationOrder = c.order;
    params.weight = c.weight;
    return RigExecApplyRotationConstraint(c.inputFrame, *c.sources, params);
}

RigExecPointFrame
_SolveScaleConstraint(const _ConstraintSolveContext &c)
{
    RigExecScaleConstraintParams params;
    params.offset = _ResolvedRead(
        *c.resolved, c.prim, "inputs:scaleOffset", GfVec3d(0), c.time);
    params.affect = c.affect;
    params.weight = c.weight;
    return RigExecApplyScaleConstraint(c.inputFrame, *c.sources, params);
}

RigExecPointFrame
_SolveParentConstraint(const _ConstraintSolveContext &c)
{
    RigExecParentConstraintParams params;
    params.translationAxes = _ReadConstraintAxisMask(
        *c.resolved, c.prim, "inputs:affectTranslationX",
        "inputs:affectTranslationY", "inputs:affectTranslationZ", c.time);
    params.rotationAxes = _ReadConstraintAxisMask(
        *c.resolved, c.prim, "inputs:affectRotationX",
        "inputs:affectRotationY", "inputs:affectRotationZ", c.time);
    // FBX disables scale by default; the explicit false fallback is the
    // authored contract, not an oversight (schema.usda:769-771).
    params.scaleAxes = _ReadConstraintAxisMask(
        *c.resolved, c.prim, "inputs:affectScaleX", "inputs:affectScaleY",
        "inputs:affectScaleZ", c.time, false);
    params.rotationOrder = c.order;
    params.weight = c.weight;
    return RigExecApplyParentConstraint(c.inputFrame, *c.sources, params);
}


/// Bakes a volumetric weight's distance-to-weight remap into the lookup
/// table its exec kernel consumes.
///
/// The named profiles bake analytically; `curve` resamples the Ts spline
/// authored on rigExec:falloffCurve. Both land in the same table, so the
/// kernel has one remap path and an author switching between a preset and
/// a hand-drawn curve changes only the numbers.
std::vector<float>
_BakeFalloffLut(const UsdPrim &prim)
{
    TfToken profile("smooth");
    if (UsdAttribute a = prim.GetAttribute(_falloffProfileAttr)) {
        a.Get(&profile);
    }
    if (profile == "linear") {
        return RigExecBuildFalloffLut(RigExecFalloffProfile::Linear);
    }
    if (profile == "smooth") {
        return RigExecBuildFalloffLut(RigExecFalloffProfile::Smooth);
    }
    if (profile == "easeIn") {
        return RigExecBuildFalloffLut(RigExecFalloffProfile::EaseIn);
    }
    if (profile == "easeOut") {
        return RigExecBuildFalloffLut(RigExecFalloffProfile::EaseOut);
    }
    if (profile == "constant") {
        return RigExecBuildFalloffLut(RigExecFalloffProfile::Constant);
    }
    if (profile != "curve") {
        return {};  // unknown token: linear, never coerced to a preset
    }

    const UsdAttribute curve = prim.GetAttribute(_falloffCurveAttr);
    if (!curve || !curve.HasSpline()) {
        // `curve` with nothing drawn is linear, not empty: the profile
        // token is a promise about SHAPE, and an author who selects it
        // before touching the editor should see the identity ramp.
        return RigExecBuildFalloffLut(RigExecFalloffProfile::Linear);
    }
    const TsSpline spline = curve.GetSpline();
    std::vector<float> lut(RigExecFalloffLutSize);
    for (size_t i = 0; i < RigExecFalloffLutSize; ++i) {
        const double x = double(i) / double(RigExecFalloffLutSize - 1);
        float value = 0.0f;
        // Ts extrapolates HELD outside the authored knot range, so a
        // curve drawn over a shorter span still yields a total field.
        if (!spline.Eval(x, &value) || !std::isfinite(value)) {
            value = float(x);
        }
        lut[i] = value;
    }
    return lut;
}

bool
_GetLandmarks(
    const UsdPrim &prim, const TfToken &attrName, UsdTimeCode time,
    std::array<GfVec3d, 4> *out)
{
    VtVec3dArray points;
    const UsdAttribute attr = prim.GetAttribute(attrName);
    if (!attr || !attr.Get(&points, time) || points.size() != 4) {
        return false;
    }
    std::copy(points.begin(), points.end(), out->begin());
    return true;
}

// Resolves a READ-side geometry input: naming a PointBased prim means its
// .points property, because a geometry input has exactly one thing to read.
// Property paths stay exact (spec §4.2).
//
// This rule is deliberately NOT applied to write targets. On the write side a
// bare prim path names the transform domain and <prim>.points names the
// geometry domain -- two different write sets on the same prim -- so inferring
// between them is what made a constraint unable to target a Mesh at all.
SdfPath
_ResolveGeometryInput(const UsdStageRefPtr &stage, const SdfPath &target)
{
    if (target.IsPrimPath()) {
        const UsdPrim prim = stage->GetPrimAtPath(target);
        if (prim && prim.IsA<UsdGeomPointBased>()) {
            return target.AppendProperty(TfToken("points"));
        }
    }
    return target;
}

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

    SdfPathVector sources;
    attribute.GetConnections(&sources);
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
    if (source.GetTypeName() != expectedType) {
        *error = attribute.GetPath().GetString() +
                 ": connection target " + sources[0].GetString() +
                 " has type " + source.GetTypeName().GetAsToken().GetString() +
                 ", expected " + expectedType.GetAsToken().GetString();
        return false;
    }
    return _ValidateScalarConnection(
        stage, source, expectedType, visiting, error);
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

// The write path no longer infers <prim> -> <prim>.points, so a point-domain
// mover handed a bare PointBased prim gets the spelling it needed. Empty for
// any other target, so callers can append unconditionally.
std::string
_PointsTargetHint(const UsdStageRefPtr &stage, const SdfPath &target)
{
    if (!target.IsPrimPath()) {
        return std::string();
    }
    const UsdPrim prim = stage->GetPrimAtPath(target);
    if (!prim || !prim.IsA<UsdGeomPointBased>()) {
        return std::string();
    }
    return ". Did you mean " +
           target.AppendProperty(TfToken("points")).GetString() + "?";
}

// Discovers the rig's joint output set implicitly (spec §4.1: the rig is a
// namespace root, not a manifest). Movers are already found this way -- a
// reverse-sibling post-order walk where carrying rigExec:moves is what makes a
// prim a mover -- and joints now follow the same rule: being a RigExecJoint
// under the rig is what makes a prim a joint output. Returned in namespace
// pre-order, which reproduces the parent-before-child ordering the authored
// lists used and keeps the binding-epoch digest stable against unrelated edits.
//
// Operator-declared joints are unioned in afterwards. Solver rigExec:joints
// targets are validated to be RigExecJoint prims later in Compile, so in a
// valid rig they are already a subset of the namespace walk; including them
// means a rig that is midway through an edit still compiles the joints its
// operators actually drive, instead of failing on a set that disagrees with the
// graph. The schema is codeless (skipCodeGeneration), so type identity is a
// type-name comparison -- the same idiom the imaging registry uses to find the
// rig itself. RigExecJoint has no derived types.
std::vector<SdfPath>
_DiscoverJointOutputs(const UsdStageRefPtr &stage, const SdfPath &rigPath)
{
    static const TfToken kJointType("RigExecJoint");
    static const TfToken kJointsRel("rigExec:joints");

    std::vector<SdfPath> joints;
    std::set<SdfPath> seen;

    const UsdPrim rig = stage->GetPrimAtPath(rigPath);
    if (!rig) {
        return joints;
    }

    for (const UsdPrim &prim : UsdPrimRange(rig)) {
        if (prim.GetTypeName() == kJointType && seen.insert(prim.GetPath()).second) {
            joints.push_back(prim.GetPath());
        }
    }

    // Union in whatever the operators name, in solver namespace order.
    for (const UsdPrim &prim : UsdPrimRange(rig)) {
        const UsdRelationship jointsRel = prim.GetRelationship(kJointsRel);
        if (!jointsRel) {
            continue;
        }
        SdfPathVector targets;
        jointsRel.GetTargets(&targets);
        for (const SdfPath &target : targets) {
            const UsdPrim joint = stage->GetPrimAtPath(target);
            if (joint && joint.GetTypeName() == kJointType &&
                seen.insert(target).second) {
                joints.push_back(target);
            }
        }
    }

    return joints;
}

// Discovers the rig's controls the same implicit way (spec §4.1): being a
// RigExecControl under the rig is what makes a prim a control. Returned in
// namespace pre-order so the discovered order -- and with it the epoch
// digest -- is stable against unrelated edits.
//
// No union pass over operator wiring, unlike the joints. A solver's
// rigExec:controls names inputs it READS, and reading a control does not
// make it one; the type does. And no emptiness rule either: a rig whose
// joints are animated directly has no control prims, which is a legal rig
// that simply draws no control guides.
std::vector<SdfPath>
_DiscoverControls(const UsdStageRefPtr &stage, const SdfPath &rigPath)
{
    static const TfToken kControlType("RigExecControl");

    std::vector<SdfPath> controls;
    const UsdPrim rig = stage->GetPrimAtPath(rigPath);
    if (!rig) {
        return controls;
    }
    for (const UsdPrim &prim : UsdPrimRange(rig)) {
        if (prim.GetTypeName() == kControlType) {
            controls.push_back(prim.GetPath());
        }
    }
    return controls;
}

// Discovers every concrete placed weight volume beneath the rig. A volume is
// a viewport output in its own right: the authored falloff surfaces are useful
// while the rigger is placing the field, before any mover consumes it. Keep
// this namespace-based, like joints and controls, so no manifest or temporary
// weight binding is required merely to make the schema's guide contract work.
std::vector<SdfPath>
_DiscoverVolumeWeights(const UsdStageRefPtr &stage, const SdfPath &rigPath)
{
    std::vector<SdfPath> volumes;
    const UsdPrim rig = stage->GetPrimAtPath(rigPath);
    if (!rig) {
        return volumes;
    }
    for (const UsdPrim &prim : UsdPrimRange(rig)) {
        if (_IsVolumeWeightType(prim.GetTypeName())) {
            volumes.push_back(prim.GetPath());
        }
    }
    return volumes;
}

// Every solver type that publishes computePointFrameArray for view-free
// joint extraction.
bool
_IsAggregateSolverType(const TfToken &typeName)
{
    static const std::set<TfToken> kAggregateSolverTypes = {
        TfToken("RigExecFkChain"), TfToken("RigExecTwoBoneIk"),
        TfToken("RigExecBlendPointFrames"),
        TfToken("RigExecTwistDistribution"), TfToken("RigExecRibbon")};
    return kAggregateSolverTypes.count(typeName) > 0;
}

// Discovers aggregate solvers by TYPE anywhere beneath the rig (spec §4.1:
// no membership lists). The scope a solver sits under is an authoring
// convenience, not identity -- "Solvers" is the convention, not a
// requirement. Returned in namespace pre-order so the discovered order,
// and with it the epoch digest, is stable against unrelated edits.
std::vector<UsdPrim>
_DiscoverAggregateSolvers(
    const UsdStageRefPtr &stage, const SdfPath &rigPath)
{
    std::vector<UsdPrim> solvers;
    const UsdPrim rig = stage->GetPrimAtPath(rigPath);
    if (!rig) {
        return solvers;
    }
    for (const UsdPrim &prim : UsdPrimRange(rig)) {
        if (_IsAggregateSolverType(prim.GetTypeName())) {
            solvers.push_back(prim);
        }
    }
    return solvers;
}

}  // namespace

RigExecRigEvaluator::RigExecRigEvaluator(
    const UsdStageRefPtr &stage, const SdfPath &rigPath)
    : _stage(stage)
    , _rigPath(rigPath)
{
    // The rig evaluates directly against the source stage. There used to be
    // a private derived stage here, holding an anonymous session sublayer
    // for the compiler's generated property applications; the engine authors
    // nothing now, so there is nothing to hold and no stage to derive.
}

RigExecRigEvaluator::~RigExecRigEvaluator()
{
    _guideTaps.reset();
    _taps.reset();
}

size_t
RigExecRigEvaluator::_ComputeStructureDigest() const
{
    // The v0.1 binding-epoch identity: canonical mover paths, schema
    // types, targets, mover execution ordinals, and the structural dependency
    // wiring each operation declares — relationship identities, read
    // phases, weight-descriptor shape, and blend membership/activations
    // (spec §4.2, §6.3). Structural edits change it; numeric values and
    // shape-preserving enables do not.
    std::string digest;

    auto appendRelTargets =
        [this, &digest](const UsdPrim &prim, const char *name,
                        bool sorted) {
        SdfPathVector targets;
        if (UsdRelationship rel = prim.GetRelationship(TfToken(name))) {
            rel.GetTargets(&targets);
        }
        if (sorted) {
            std::sort(targets.begin(), targets.end());
        }
        digest += name;
        digest += '=';
        for (const SdfPath &t : targets) {
            digest += t.GetString();
            digest += ',';
        }
        digest += '|';
        return targets;
    };
    // A read phase decides WHICH revision of an input a mover consumes, which
    // is compiled wiring, not a value -- so editing one has to re-epoch
    // exactly the way retargeting the relationship does. Authored on the
    // property, so it is hashed alongside that property's targets rather than
    // as another prim-level token.
    auto appendPhase = [&digest](const UsdPrim &prim, const char *name) {
        std::string authored;
        if (const UsdRelationship rel = prim.GetRelationship(TfToken(name))) {
            rel.GetMetadata(TfToken(RigExecReadPhaseMetadataName), &authored);
        } else if (const UsdAttribute a = prim.GetAttribute(TfToken(name))) {
            a.GetMetadata(TfToken(RigExecReadPhaseMetadataName), &authored);
        }
        digest += name;
        digest += "@phase=";
        digest += authored;
        digest += '|';
    };
    auto appendToken = [&digest](const UsdPrim &prim, const char *name) {
        TfToken value;
        if (UsdAttribute a = prim.GetAttribute(TfToken(name))) {
            a.Get(&value);
            digest += name;
            digest += "#samples=";
            digest += std::to_string(a.GetNumTimeSamples());
            digest += '|';
        }
        digest += name;
        digest += '=';
        digest += value.GetString();
        digest += '|';
    };
    auto appendAttributeBinding =
        [this, &digest](const UsdPrim &prim, const char *name) {
        const UsdAttribute attr = prim.GetAttribute(TfToken(name));
        digest += name;
        digest += "@sources=";
        std::set<SdfPath> visiting;
        std::function<void(const UsdAttribute &)> append =
            [&](const UsdAttribute &a) {
            if (!a || !visiting.insert(a.GetPath()).second) {
                digest += a ? "cycle:" + a.GetPath().GetString()
                            : std::string("missing");
                digest += ',';
                return;
            }
            digest += a.GetPath().GetString() + ":" +
                      a.GetTypeName().GetAsToken().GetString() + ":samples:" +
                      std::to_string(a.GetNumTimeSamples()) + "->";
            SdfPathVector sources;
            a.GetConnections(&sources);
            for (const SdfPath &source : sources) {
                const UsdAttribute sourceAttr =
                    _stage->GetAttributeAtPath(source);
                if (!sourceAttr) {
                    digest += "missing:" + source.GetString() + ",";
                } else {
                    append(sourceAttr);
                }
            }
            visiting.erase(a.GetPath());
        };
        append(attr);
        digest += '|';
    };
    auto appendFrameBindingIdentity =
        [this, &digest](const UsdPrim &prim, const char *name) {
        SdfPathVector targets;
        if (const UsdRelationship rel =
                prim.GetRelationship(TfToken(name))) {
            rel.GetTargets(&targets);
        }
        digest += name;
        digest += "@bindings=";
        for (const SdfPath &target : targets) {
            const UsdPrim provider =
                _stage->GetPrimAtPath(target.GetPrimPath());
            const TfToken type = provider ? provider.GetTypeName() : TfToken();
            digest += target.GetString();
            digest += ':';
            digest += type.GetString();
            digest += ':';
            if (type == "RigExecControl" || type == "RigExecJoint") {
                digest += "frameTap";
            } else if (provider && UsdGeomXformable(provider)) {
                digest += "nativeXform";
            } else {
                digest += "invalid";
            }
            digest += ',';
        }
        digest += '|';
    };

    // Weight-object descriptor shape is epoch identity (spec §4.1):
    // target, representation, policy, and canonical sparse support.
    //
    // Recursive, because a combine's field shape is its inputs' shapes:
    // an edit inside a composed input has to re-epoch the combine that
    // folds it, or the baked falloff tables replay stale.
    //
    // Cycle-TRACKED rather than depth-limited. A depth cap terminates,
    // but it terminates by silently dropping everything below it, so a
    // legitimately deep composition stops contributing to the epoch
    // identity and edits down there stop triggering a recompile. Marking
    // the path being walked costs the same and is exact; a genuine cycle
    // is caught and reported by the compile-time walk instead.
    std::set<SdfPath> digestVisiting;
    std::function<void(const SdfPath &)> appendWeightObject =
        [&](const SdfPath &weightPath) {
        const UsdPrim w = _stage->GetPrimAtPath(weightPath);
        if (!w || !digestVisiting.insert(weightPath).second) {
            return;
        }
        struct Pop {
            std::set<SdfPath> &s;
            const SdfPath &p;
            ~Pop() { s.erase(p); }
        } pop{digestVisiting, weightPath};
        digest += w.GetTypeName().GetString();
        digest += '|';
        appendRelTargets(w, "rigExec:weightTarget", true);
        appendRelTargets(w, "rigExec:baseWeight", true);
        appendToken(w, "rigExec:representation");
        appendToken(w, "rigExec:rangePolicy");
        appendToken(w, "rigExec:operation");
        VtIntArray indices;
        if (UsdAttribute a = w.GetAttribute(TfToken("rigExec:indices"))) {
            a.Get(&indices);
        }
        std::vector<int> support(indices.begin(), indices.end());
        std::sort(support.begin(), support.end());
        for (int i : support) {
            digest += std::to_string(i);
            digest += ',';
        }
        digest += '|';
        VtFloatArray values;
        if (UsdAttribute a = w.GetAttribute(TfToken("rigExec:values"))) {
            a.Get(&values);
        }
        digest += "rigExec:values#size=" +
                  std::to_string(values.size()) + '|';

        // Attribute connection identity is compiled Exec wiring. A rewire
        // must rebuild the prepared request even when the two sources happen
        // to carry the same value at the current time. Static fields include
        // these markers too so adding an illegal source re-enters Compile and
        // is rejected instead of replaying the old request.
        for (const char *field : {
                 "rigExec:values", "rigExec:indices",
                 "rigExec:defaultWeight", "rigExec:representation",
                 "rigExec:rangePolicy", "rigExec:operation",
                 "inputs:driver", "inputs:scale", "inputs:bias",
                 "inputs:falloffMin", "inputs:falloffMax",
                 "inputs:invert", "inputs:strength", "inputs:scaleX",
                 "inputs:scaleY", "inputs:scaleZ", "inputs:extentU",
                 "inputs:extentV"}) {
            appendAttributeBinding(w, field);
        }

        // Volumetric extension. Only STRUCTURAL properties belong here:
        // the shape family, which axis it measures, where it samples,
        // and the baked remap. inputs:falloffMin/Max, invert, strength,
        // scaleX/Y/Z and extentU/V are deliberately absent -- they are
        // per-frame exec values, and hashing them would recompile every
        // frame an artist scrubs one. rigExec:planeBounds IS here
        // because it selects which field function runs, exactly as
        // rigExec:planeAxis selects which coordinate it measures.
        appendToken(w, "rigExec:falloffProfile");
        appendToken(w, "rigExec:samplePhase");
        appendToken(w, "rigExec:planeAxis");
        appendToken(w, "rigExec:planeBounds");
        appendToken(w, "rigExec:combineMode");
        const SdfPathVector curveTargets =
            appendRelTargets(w, "rigExec:curve", true);
        // A CurveWeight's relationship alone is not the complete structural
        // binding: the target must still resolve to a point3f[] attribute.
        // Hash that resolution so removing/retyping the source, or repairing
        // it in place without changing the relationship path, re-enters
        // Compile and applies the same validation as the original authoring.
        for (const SdfPath &target : curveTargets) {
            const SdfPath pointsPath = target.IsPropertyPath()
                ? target
                : target.AppendProperty(TfToken("points"));
            const UsdAttribute points =
                _stage->GetAttributeAtPath(pointsPath);
            digest += "rigExec:curveSource=";
            digest += pointsPath.GetString();
            digest += ':';
            digest += points
                ? points.GetTypeName().GetAsToken().GetString()
                : std::string("missing");
            digest += '|';
        }
        appendRelTargets(w, "rigExec:sampleSource", true);
        // The falloff curve is structural: it is resampled to a table
        // once per epoch, so an edit to it has to begin a new one.
        //
        // What gets hashed is the BAKED TABLE, not the knots. Hashing
        // knot times and values misses everything else that changes the
        // curve's shape -- interpolation mode, tangent slopes and widths,
        // dual values, extrapolation, loops -- so flipping a knot from
        // curve to held left the digest unchanged, the epoch unrebuilt,
        // and exec replaying a stale LUT while the CPU oracle rebaked the
        // live spline. Hashing the table is exact by construction: it is
        // precisely the bytes exec consumes, so anything that changes
        // them re-epochs and nothing that does not, does.
        //
        // This also folds in rigExec:falloffProfile, which is why that
        // token is not hashed separately.
        if (_IsVolumeWeightType(w.GetTypeName())) {
            const std::vector<float> lut = _BakeFalloffLut(w);
            digest += "lut=";
            digest.append(reinterpret_cast<const char *>(lut.data()),
                          lut.size() * sizeof(float));
            digest += '|';
        }

        // Composition order is semantic for subtract and overlay, so the
        // input list is hashed UNSORTED -- unlike every other
        // relationship here, whose permutation is explicitly not.
        SdfPathVector inputs;
        if (UsdRelationship rel =
                w.GetRelationship(TfToken("rigExec:inputWeights"))) {
            rel.GetTargets(&inputs);
        }
        digest += "rigExec:inputWeights=";
        for (const SdfPath &t : inputs) {
            digest += t.GetString();
            digest += ',';
        }
        digest += '|';
        for (const SdfPath &t : inputs) {
            appendWeightObject(t);
        }
        // A dynamic weight's base is composed the same way.
        SdfPathVector bases;
        if (UsdRelationship rel =
                w.GetRelationship(TfToken("rigExec:baseWeight"))) {
            rel.GetTargets(&bases);
        }
        for (const SdfPath &t : bases) {
            appendWeightObject(t);
        }
    };

    // Rig output set. Discovered rather than authored, so the digest hashes the
    // discovered paths -- adding, removing, or renaming a joint prim changes
    // the epoch exactly as editing the old manifest relationship did.
    // Derived-property maintenance (spec §7.6 revised) is unconditional now, so
    // there is no policy token left to hash: what the compiler synthesizes depends
    // only on which gprims author normals/extent, and that is already epoch
    // identity through the points-chain targets below.
    for (const SdfPath &jointPath : _DiscoverJointOutputs(_stage, _rigPath)) {
        digest += jointPath.GetString();
        digest += ',';
    }
    digest += '|';

    // The discovered control set, for the same reason: it decides which
    // computePointFrame taps the epoch's prepared request carries. A control
    // that no solver reads is otherwise invisible to this digest -- adding
    // one purely to draw a guide would leave the compiled tap set behind and
    // the guide would never appear.
    for (const SdfPath &controlPath : _DiscoverControls(_stage, _rigPath)) {
        digest += controlPath.GetString();
        digest += ',';
    }
    digest += '|';

    // Standalone volume guides carry placement taps and baked falloff state
    // even when no mover consumes their field. Their discovered paths and
    // structural properties therefore belong to the epoch just as standalone
    // controls do; otherwise adding or repairing one would replay a tap set
    // that can never publish it.
    for (const SdfPath &volumePath :
         _DiscoverVolumeWeights(_stage, _rigPath)) {
        digest += volumePath.GetString();
        digest += '|';
        appendWeightObject(volumePath);
    }
    digest += '|';

    // Solver->joint wiring is epoch identity (view-free extraction,
    // user-directed 2026-07-25, replaces RigExecPointFrameView): each
    // solver's ORDERED rigExec:joints list decides which joint
    // self-extracts which aggregate element, so adding, removing, or
    // reordering joints changes what compile Pass 0 synthesizes. Order is
    // semantic (position = element index), so this list is never sorted.
    if (const UsdPrim rig = _stage->GetPrimAtPath(_rigPath)) {
        // Recursive over the composed rig subtree (not GetChildren):
        // solvers live wherever the author put them, so every scope's
        // wiring must contribute to epoch identity
        // (consistent with mover discovery and compile Pass 0).
        for (const UsdPrim &solver : UsdPrimRange(rig)) {
            SdfPathVector joints;
            if (const UsdRelationship rel =
                    solver.GetRelationship(TfToken("rigExec:joints"))) {
                rel.GetTargets(&joints);
            }
            // Emit for joint-bearing prims (their bindings) AND for every
            // aggregate solver even without joints: its cardinality feeds
            // Phase A element checks, possibly indirectly through a Blend
            // input, so a cardinality edit must begin a new epoch.
            const bool isAggregate =
                _IsAggregateSolverType(solver.GetTypeName());
            if (joints.empty() && !isAggregate) {
                continue;
            }
            digest += solver.GetPath().GetString();
            digest += '|';
            digest += solver.GetTypeName().GetString();
            digest += '|';
            for (const SdfPath &j : joints) {
                digest += j.GetString();
                digest += ',';
            }
            // Element remap is structural: it changes which frame each
            // joint self-extracts. Parallel to joints, so not sorted.
            digest += '|';
            const UsdAttribute jeAttr =
                solver.GetAttribute(TfToken("rigExec:jointElements"));
            VtIntArray jointElements;
            if (jeAttr) {
                jeAttr.Get(&jointElements);
            }
            for (int e : jointElements) {
                digest += std::to_string(e);
                digest += ',';
            }
            // jointElements is a static input; record its time-sample count
            // so adding a sample post-compile re-runs Compile()'s rejection
            // rather than silently keeping the captured default (round-6).
            digest += "s" + std::to_string(
                                jeAttr ? jeAttr.GetNumTimeSamples() : 0);
            // Cardinality-determining inputs: an edit that changes how many
            // frames the solver produces must recompile so Phase A
            // re-validates every element binding. Value-only
            // edits that don't change frame count stay value-only.
            digest += "|card=";
            const TfToken stype = solver.GetTypeName();
            if (stype == "RigExecFkChain") {
                appendRelTargets(solver, "rigExec:controls", false);
            } else if (stype == "RigExecTwistDistribution") {
                const UsdAttribute ca =
                    solver.GetAttribute(TfToken("rigExec:count"));
                const UsdAttribute wa =
                    solver.GetAttribute(TfToken("rigExec:weights"));
                int cnt = 1;
                if (ca) {
                    ca.Get(&cnt);
                }
                VtFloatArray w;
                if (wa) {
                    wa.Get(&w);
                }
                // Effective cardinality (weights wins, so an ignored count
                // value does not churn the epoch) plus the
                // time-sample presence of BOTH attrs so that ADDING a
                // sample without changing the default still changes the
                // digest, forcing the recompile that re-runs the pre-pass
                // sample rejection.
                const size_t effective =
                    !w.empty() ? w.size()
                               : static_cast<size_t>(std::max(cnt, 1));
                digest += std::to_string(effective) + "/" +
                          std::to_string(ca ? ca.GetNumTimeSamples() : 0) +
                          "/" +
                          std::to_string(wa ? wa.GetNumTimeSamples() : 0) +
                          ",";
            } else if (stype == "RigExecRibbon") {
                const UsdAttribute a =
                    solver.GetAttribute(TfToken("rigExec:sampleCount"));
                int sc = 5;
                if (a) {
                    a.Get(&sc);
                }
                digest += std::to_string(sc) + "/" +
                          std::to_string(a ? a.GetNumTimeSamples() : 0) + ",";
                // The driver curve lowers (Pass 1.5) to generated
                // resolvedDriverPoints + a bind-time restDriverPoints
                // capture, so rewiring it (or a layer-mute/variant switch
                // that retargets it) is structural.
                appendRelTargets(solver, "rigExec:driverCurve", false);
            } else if (stype == "RigExecBlendPointFrames") {
                appendRelTargets(solver, "rigExec:inputA", false);
                appendRelTargets(solver, "rigExec:inputB", false);
            }
            if (stype == "RigExecTwoBoneIk") {
                // Whether each absolute length is authored decides between
                // the authored value and the rest-implied one, so
                // authoring or clearing one begins a new epoch. The length
                // and offset VALUES stay value-only (read live at
                // Evaluate); only the authored-or-not bit is here.
                const UsdAttribute upper = solver.GetAttribute(
                    TfToken("rigExec:upperLength"));
                const UsdAttribute lower = solver.GetAttribute(
                    TfToken("rigExec:lowerLength"));
                digest += "|lengthAuthored=";
                digest += (upper && upper.HasAuthoredValueOpinion()) ? '1'
                                                                    : '0';
                digest += (lower && lower.HasAuthoredValueOpinion()) ? '1'
                                                                    : '0';
            }
            digest += ';';
        }
    }

    const UsdPrim movers =
        _stage->GetPrimAtPath(_rigPath.AppendChild(TfToken("Movers")));
    if (movers) {
        for (const UsdPrim &prim : _GetMoverExecutionOrder(movers)) {
            const UsdRelationship moves = prim.GetRelationship(_movesRel);
            if (!moves) {
                continue;
            }
            digest += prim.GetPath().GetString();
            digest += '|';
            digest += prim.GetTypeName().GetString();
            digest += '|';
            SdfPathVector targets;
            moves.GetTargets(&targets);
            // Target-list order is non-semantic (spec §4.2): sort before
            // hashing so a permutation does not change the epoch.
            std::sort(targets.begin(), targets.end());
            for (const SdfPath &t : targets) {
                const SdfPath canonical = t;
                digest += canonical.GetString();
                digest += ',';
                // Derived synthesis identity (spec §7.6 revised): whether
                // a written points target's gprim authors the derived
                // properties decides what the compiler synthesizes, so
                // authoring or removing them is a structural edit.
                if (canonical.IsPropertyPath() &&
                    canonical.GetNameToken() == "points") {
                    const SdfPath owner = canonical.GetPrimPath();
                    // Point-domain cardinality is frozen descriptor shape.
                    // Hash every authored cardinality (not the point values)
                    // so a 3 -> 2 target edit rebuilds the epoch and lets the
                    // compile validator reject a now-mismatched dense field.
                    // A set avoids recompiling merely because another sample
                    // with the same frozen cardinality was authored.
                    std::set<size_t> cardinalities;
                    if (const UsdAttribute points =
                            _stage->GetAttributeAtPath(canonical)) {
                        VtVec3fArray value;
                        bool resolved = false;
                        if (points.GetResolveInfo(UsdTimeCode::Default())
                                .GetSource() ==
                                UsdResolveInfoSourceDefault &&
                            points.Get(&value, UsdTimeCode::Default())) {
                            cardinalities.insert(value.size());
                            resolved = true;
                        }
                        std::vector<double> times;
                        points.GetTimeSamples(&times);
                        for (double sampleTime : times) {
                            if (points.Get(
                                    &value, UsdTimeCode(sampleTime))) {
                                cardinalities.insert(value.size());
                                resolved = true;
                            }
                        }
                        if (!resolved &&
                            points.Get(&value, UsdTimeCode::Default())) {
                            cardinalities.insert(value.size());
                        }
                    }
                    digest += "pointCardinalities=";
                    for (size_t cardinality : cardinalities) {
                        digest += std::to_string(cardinality);
                        digest += ',';
                    }
                    digest += '|';
                    auto authored = [this, &owner](const char *name) {
                        const UsdAttribute a = _stage->GetAttributeAtPath(
                            owner.AppendProperty(TfToken(name)));
                        return a && a.HasAuthoredValue();
                    };
                    digest += "derived=";
                    // Owner schema type gates mesh-only normal synthesis.
                    if (const UsdPrim ownerPrim =
                            _stage->GetPrimAtPath(owner)) {
                        digest += ownerPrim.GetTypeName().GetString();
                    }
                    digest += authored("normals") ? 'n' : '-';
                    digest += authored("extent") ? 'e' : '-';
                    digest += authored("widths") ? 'w' : '-';
                    digest += ',';
                }
            }
            digest += ';';

            // Declared dependency wiring and read phases.
            appendRelTargets(prim, "rigExec:transform", true);
            appendToken(prim, "rigExec:transformReadPhase");
            appendToken(prim, "rigExec:operation");
            appendToken(prim, "rigExec:mode");
            for (const char *input : {
                     "inputs:defaultWeight", "inputs:enabled",
                     "inputs:value", "inputs:min", "inputs:max"}) {
                appendAttributeBinding(prim, input);
            }
            // Pose-constraint wiring. Source order is semantic because every
            // source has a parallel weight (and Parent has parallel offsets),
            // so it must never be sorted. aimTarget remains the legacy
            // single-source spelling and is hashed for existing assets.
            appendRelTargets(prim, "rigExec:aimTarget", false);
            appendRelTargets(prim, "rigExec:sources", false);
            appendRelTargets(prim, "rigExec:worldUpObject", false);
            appendRelTargets(prim, "rigExec:firstJoint", false);
            appendRelTargets(prim, "rigExec:endJoint", false);
            appendRelTargets(prim, "rigExec:effector", false);
            appendRelTargets(prim, "rigExec:poleVectorObjects", false);
            if (_IsFrameConstraintType(prim.GetTypeName())) {
                appendFrameBindingIdentity(prim, "rigExec:moves");
                appendFrameBindingIdentity(prim, "rigExec:aimTarget");
                appendFrameBindingIdentity(prim, "rigExec:sources");
                appendFrameBindingIdentity(prim, "rigExec:worldUpObject");
                appendFrameBindingIdentity(prim, "rigExec:firstJoint");
                appendFrameBindingIdentity(prim, "rigExec:endJoint");
                appendFrameBindingIdentity(prim, "rigExec:effector");
                appendFrameBindingIdentity(
                    prim, "rigExec:poleVectorObjects");
                appendToken(prim, "rigExec:rotationOrder");
                appendToken(prim, "rigExec:worldUpType");
                appendToken(prim, "rigExec:aimAxis");
                appendToken(prim, "rigExec:upPolicy");
                appendToken(prim, "rigExec:solverMode");
                appendToken(prim, "rigExec:poleVectorMode");
                appendToken(prim, "rigExec:evaluationMode");
            }
            // Static-input relationships captured at compile into generated
            // resolved*/rest* wiring (lattice cage, surface, curve bind/
            // driver): retargeting any of these must recompile so the
            // captured bind-time values are refreshed. Hashed in AUTHORED
            // order (sorted=false) because the compiler consumes targets[0], so
            // a reorder that changes the selected input must change the
            // digest. An absent rel appends a constant
            // empty marker (harmless, invariant per mover type).
            appendRelTargets(prim, "rigExec:cage", false);
            appendRelTargets(prim, "rigExec:surface", false);
            appendRelTargets(prim, "rigExec:bindCoordinates", false);
            for (const char *phased : {"rigExec:transform", "rigExec:cage",
                                       "rigExec:surface", "rigExec:curvenet",
                                       "rigExec:bindCoordinates",
                                       "rigExec:driverCurve"}) {
                appendPhase(prim, phased);
            }
            appendRelTargets(prim, "rigExec:driverFrames", false);
            appendRelTargets(prim, "rigExec:driverCurve", false);
            // Authored order, not sorted: the binding takes targets[0], so
            // reordering a multi-target relationship changes the wiring and
            // must therefore change the digest.
            appendRelTargets(prim, "rigExec:curvenet", false);
            for (const SdfPath &w :
                 appendRelTargets(prim, "rigExec:weightObject", true)) {
                appendWeightObject(w);
            }
            for (const SdfPath &inputPath :
                 appendRelTargets(prim, "rigExec:blendInputs", true)) {
                const UsdPrim input = _stage->GetPrimAtPath(inputPath);
                if (!input) {
                    continue;
                }
                for (const SdfPath &samplePath :
                     appendRelTargets(input, "rigExec:samples", true)) {
                    const UsdPrim sample =
                        _stage->GetPrimAtPath(samplePath.GetPrimPath());
                    if (!sample) {
                        continue;
                    }
                    appendRelTargets(sample, "rigExec:targetPoints", true);
                    float activation = 1;
                    if (UsdAttribute a = sample.GetAttribute(
                            TfToken("rigExec:activation"))) {
                        a.Get(&activation);
                    }
                    // Activation edits are structural (spec §7.3).
                    digest += std::to_string(activation);
                    digest += '|';
                }
            }
            digest += ';';
        }
    }
    return std::hash<std::string>{}(digest);
}

bool
RigExecRigEvaluator::Compile(std::vector<std::string> *errors)
{
    auto reportError = [errors](const std::string &message) {
        if (errors) {
            errors->push_back(message);
        }
    };

    // Same channel, different verdict: a notice is reported to the author and
    // the compile CONTINUES. It exists so that "this is not wired up" can be
    // said out loud without being fatal -- an incomplete mover is inert, not a
    // reason to refuse the whole rig. Every caller of reportError still
    // returns false; nothing that calls this one does.
    auto reportNotice = [errors](const std::string &message) {
        if (errors) {
            errors->push_back(message);
        }
    };

    if (!_stage) {
        reportError("no stage; nothing to compile");
        return false;
    }

    const UsdPrim rig = _stage->GetPrimAtPath(_rigPath);
    if (!rig) {
        reportError("Rig prim not found: " + _rigPath.GetString());
        return false;
    }
    // Prototype-hosted rigs fail validation (spec §4.1: a rig in a
    // prototype or otherwise unable to deinstance is rejected).
    if (rig.IsInstanceProxy() || rig.IsInPrototype()) {
        reportError("Rig is instance-proxy/prototype hosted: " +
                    _rigPath.GetString());
        return false;
    }

    // Phase A: validation into locals. Nothing below mutates evaluator
    // state until every check passes, so a failed structural edit keeps
    // the previous epoch publishable (spec §4.1 atomic transactions).
    std::vector<SdfPath> newJointPaths =
        _DiscoverJointOutputs(_stage, _rigPath);
    // A rig with no joints is legal.
    //
    // It used to be rejected here, on the reading that a joint is what a rig
    // publishes. That was never true of the evaluator, only of this check: a
    // mover writes an exact target, and a target is a points array, a plain
    // UsdGeomXformable's transform, or a scalar property just as readily as
    // it is a joint frame -- three output domains that all reach a consumer
    // through RigExecRigPose. Requiring a joint forced authors to add a
    // vestigial one to rigs that pose none (10_AimXformTurret says so in a
    // comment), and rejected outright the simplest rig there is: a constraint
    // aiming one Xform at another.
    //
    // What the rig DOES need is at least one output, and that cannot be known
    // until the mover walk below has run. The check moved there.
    // Controls and placed volumes are discovered alongside the joints. Zero
    // of either kind is ordinary; the combined output gate below decides
    // whether the whole rig is genuinely empty.
    std::vector<SdfPath> newControlPaths =
        _DiscoverControls(_stage, _rigPath);
    std::vector<SdfPath> newVolumeWeightPaths =
        _DiscoverVolumeWeights(_stage, _rigPath);

    // Transform-authority validation (host-durability redesign).
    //
    // Neither condition can FAIL a compile, and both are reported rather
    // than fixed: the rig still evaluates exactly right, because the
    // evaluator reads rest:space and the avars and nothing else. What
    // breaks is the BOUNDS -- a provider's computed extent bakes its posed
    // frame into asset-relative space, which is only the whole story while
    // nothing else contributes a transform between the asset root and the
    // provider. Refusing to compile over a framing inaccuracy would be
    // wildly out of proportion; saying nothing would leave an author
    // wondering why one control frames to the wrong place.
    {
        const SdfPath assetRoot = _rigPath.GetParentPath();
        auto warn = [errors](const std::string &message) {
            // Both channels on purpose: TF_WARN is what a host surfaces to
            // the author, and the errors vector is what a test can read.
            // Compile still returns true.
            if (errors) {
                errors->push_back("warning: " + message);
            }
            TF_WARN("%s", message.c_str());
        };
        // Every Boundable provider, aggregate solvers included: they
        // inherit Boundable/Xformable too, so an authored op on one is
        // applied by BBoxCache to an already-baked extent while the guide
        // it draws ignores it entirely.
        std::vector<SdfPath> providers = newJointPaths;
        providers.insert(providers.end(), newControlPaths.begin(),
                         newControlPaths.end());
        providers.insert(providers.end(), newVolumeWeightPaths.begin(),
                         newVolumeWeightPaths.end());
        {
            for (const UsdPrim &solver :
                 _DiscoverAggregateSolvers(_stage, _rigPath)) {
                providers.push_back(solver.GetPath());
            }
        }
        for (const SdfPath &providerPath : providers) {
            const UsdPrim prim = _stage->GetPrimAtPath(providerPath);
            if (!prim) {
                continue;
            }
            // xformOps arrive on every provider now that RigExecXformable
            // inherits UsdGeomBoundable, but they are NOT a transform
            // authority: rest:space plus the avars are the only one (the
            // Ir alignment). An authored op is a second one that nothing
            // reads, so the prim moves in a stock UsdGeom traversal while
            // the rig ignores it entirely.
            if (const UsdGeomXformable xformable = UsdGeomXformable(prim)) {
                bool resetsStack = false;
                if (!xformable.GetOrderedXformOps(&resetsStack).empty()) {
                    warn(prim.GetTypeName().GetString() + " " +
                         providerPath.GetString() +
                         " authors xformOps, which are not a transform "
                         "authority for a RigExec provider (rest:space and "
                         "the avars are); the ops are ignored by evaluation "
                         "and are not in the computed extent");
                }
            }
            // ...and nothing between the provider and the asset root may
            // contribute one either. RigExec's own types are skipped: a
            // joint nested under a joint is the ordinary shape of a rig,
            // and the loop above already polices ops authored on those.
            for (SdfPath ancestorPath = providerPath.GetParentPath();
                 ancestorPath != assetRoot &&
                     !ancestorPath.IsAbsoluteRootPath() &&
                     !ancestorPath.IsEmpty();
                 ancestorPath = ancestorPath.GetParentPath()) {
                const UsdPrim ancestor = _stage->GetPrimAtPath(ancestorPath);
                if (!ancestor) {
                    break;
                }
                if (TfStringStartsWith(ancestor.GetTypeName().GetString(),
                                       "RigExec")) {
                    continue;
                }
                if (UsdGeomXformable(ancestor)) {
                    warn("Xformable " + ancestorPath.GetString() +
                         " sits between the asset root and provider " +
                         providerPath.GetString() +
                         "; its transform is not composed into the "
                         "provider's frames, so the computed extent places "
                         "the guide as if it were identity");
                }
            }

            // A provider's extent covers the guides beneath it, and only
            // those. Authored geometry parented under one is invisible to
            // it -- and to every ancestor, because UsdGeomBBoxCache stops
            // descending at a Boundable -- so the gprim silently drops out
            // of every bounding box in the scene.
            for (const UsdPrim &descendant : UsdPrimRange(prim)) {
                if (descendant == prim) {
                    continue;
                }
                // A provider nested under a provider with a DIFFERENT
                // resolved purpose is dropped from the ancestor's extent
                // on purpose: one extent carries one purpose, and the
                // bounding-box cache files it under the ancestor's. Nobody
                // reading the namespace would guess that, so say it.
                if (descendant.IsA<UsdGeomImageable>()) {
                    const UsdGeomImageable descendantImageable(descendant);
                    const UsdGeomImageable providerImageable(prim);
                    const TfToken descendantPurpose =
                        descendantImageable.ComputePurpose();
                    const TfToken providerPurpose =
                        providerImageable.ComputePurpose();
                    if (!descendantPurpose.IsEmpty() &&
                        !providerPurpose.IsEmpty() &&
                        descendantPurpose != providerPurpose &&
                        TfStringStartsWith(
                            descendant.GetTypeName().GetString(),
                            "RigExec")) {
                        warn(descendant.GetTypeName().GetString() + " " +
                             descendant.GetPath().GetString() +
                             " has purpose '" +
                             descendantPurpose.GetString() +
                             "' but is nested under " +
                             providerPath.GetString() + " whose purpose is '" +
                             providerPurpose.GetString() +
                             "'; one extent carries one purpose, so this "
                             "provider is excluded from its ancestor's "
                             "bounds");
                    }
                }
                if (descendant.IsA<UsdGeomGprim>()) {
                    warn("gprim " + descendant.GetPath().GetString() +
                         " is parented under RigExec provider " +
                         providerPath.GetString() +
                         "; a provider's computed extent covers only the "
                         "guides beneath it, and bounds stop descending at "
                         "a Boundable, so this geometry is absent from "
                         "every bounding box that should contain it");
                }
            }
        }
    }

    std::vector<SdfPath> solverArrayPaths;
    std::map<SdfPath, SdfPath> newRibbonDriverPoints;
    // Every aggregate frame provider, wherever the author placed it: their
    // computePointFrameArray results are published (and drawn as guides by
    // the imaging chain, like OpenExec's IrJointScope guides).
    for (const UsdPrim &child :
         _DiscoverAggregateSolvers(_stage, _rigPath)) {
        solverArrayPaths.push_back(child.GetPath());
        // A ribbon's driver-curve points, resolved to the exact native
        // attribute. This replaces the compiler's last authoring pass:
        // the resolution is compiled state (rewiring the relationship is
        // structural, and the epoch digest already treats it that way),
        // and the values ride in as exec overrides at evaluation time.
        if (child.GetTypeName() == "RigExecRibbon") {
            SdfPathVector curves;
            if (const UsdRelationship rel = child.GetRelationship(
                    TfToken("rigExec:driverCurve"))) {
                rel.GetTargets(&curves);
            }
            if (!curves.empty()) {
                newRibbonDriverPoints[child.GetPath()] =
                    curves[0].IsPrimPath()
                        ? curves[0].AppendProperty(TfToken("points"))
                        : curves[0];
            }
        }
    }

    // Mover discovery: reverse-sibling post-order walk of the composed Movers
    // namespace. Descendants run before their mover parent; sibling branches
    // run bottom-to-top in usdview (reverse composed child order, spec §4.2).
    std::vector<RigExecMoverRecord> newMovers;
    /// Mover-bearing prims discovered but skipped because nothing is wired to
    /// their rigExec:moves yet. They are not outputs, but they ARE evidence
    /// that the rig root points somewhere real.
    size_t inertMovers = 0;
    const UsdPrim movers =
        _stage->GetPrimAtPath(_rigPath.AppendChild(TfToken("Movers")));
    int ordinal = 0;
    if (movers) {
        for (const UsdPrim &prim : _GetMoverExecutionOrder(movers)) {
            const UsdRelationship moves = prim.GetRelationship(_movesRel);
            if (!moves) {
                if (_IsFrameConstraintType(prim.GetTypeName())) {
                    reportError(
                        prim.GetTypeName().GetString() + " " +
                        prim.GetPath().GetString() +
                        " has executable constraint semantics but no "
                        "rigExec:moves relationship");
                    return false;
                }
                // A solver carries no rigExec:moves; it poses joints through
                // the rig-wide solver discovery above, so here it is just
                // skipped like any other grouping scope.
                continue;  // grouping scope
            }
            SdfPathVector targets;
            moves.GetTargets(&targets);
            if (targets.empty()) {
                // An unwired mover writes nothing, so it is INERT -- not a
                // reason to fail the rig. The stack is dynamic: disconnecting
                // rigExec:moves is the ordinary interactive edit, and taking
                // every other mover down with it makes a node graph unusable
                // the moment a wire is pulled.
                //
                // This is the same treatment a prim with no rigExec:moves at
                // all already gets just above (grouping scope), with one
                // difference: that case is silent because every Scope under
                // Movers would otherwise announce itself, while an authored
                // but empty write set is a wire the author meant to connect.
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
                    reportError("Mover " + prim.GetPath().GetString() +
                                " targets missing prim " +
                                primPath.GetString());
                    return false;
                }
                // Cross-rig writes are rejected in v1 (spec §4.2).
                if (!primPath.HasPrefix(assetRoot)) {
                    reportError("Mover " + prim.GetPath().GetString() +
                                " targets outside the rig asset: " +
                                canonical.GetString());
                    return false;
                }
                if (canonical.IsPropertyPath()) {
                    if (structuralProperties.count(canonical.GetNameToken())) {
                        reportError(
                            "Mover " + prim.GetPath().GetString() +
                            " targets structural property " +
                            canonical.GetString());
                        return false;
                    }
                    // Derived properties are compiler-maintained (spec
                    // §7.6 revised): no authored mover writes them.
                    if (canonical.GetNameToken() == "normals" ||
                        canonical.GetNameToken() == "extent") {
                        reportError(
                            "Mover " + prim.GetPath().GetString() +
                            " targets derived property " +
                            canonical.GetString() +
                            "; normals/extent maintenance is synthesized "
                            "by the compiler");
                        return false;
                    }
                    if (!_stage->GetAttributeAtPath(canonical)) {
                        reportError(
                            "Mover " + prim.GetPath().GetString() +
                            " targets missing property " +
                            canonical.GetString());
                        return false;
                    }
                }
                // Duplicate targets after canonicalization are rejected
                // (spec §6.1).
                if (std::find(record.targets.begin(), record.targets.end(),
                              canonical) != record.targets.end()) {
                    reportError("Mover " + prim.GetPath().GetString() +
                                " has duplicate canonical target " +
                                canonical.GetString());
                    return false;
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
                    reportError(
                        record.schemaType.GetString() + " " +
                        prim.GetPath().GetString() +
                        " must move exactly one transform-provider prim");
                    return false;
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
                        reportError(
                            record.schemaType.GetString() + " " +
                            prim.GetPath().GetString() + " targets " +
                            constraintTarget.GetString() +
                            ", whose owner is not a UsdGeomPointBased prim");
                        return false;
                    }
                    // Legal: the geometry domain. The prim must still be able
                    // to supply a base frame, because the delta is measured
                    // against it exactly as in the transform domain.
                    if (!UsdGeomXformable(owner)) {
                        reportError(
                            record.schemaType.GetString() + " " +
                            prim.GetPath().GetString() + " targets " +
                            constraintTarget.GetString() +
                            ", whose owner cannot supply a base frame");
                        return false;
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
                        reportError(
                            record.schemaType.GetString() + " " +
                            prim.GetPath().GetString() + " targets " +
                            constraintTarget.GetString() +
                            ", which is not a transform provider; a "
                            "constraint target must be a UsdGeomXformable");
                        return false;
                    }
                }
            } else if (record.schemaType ==
                       "RigExecSingleChainIkConstraint") {
                if (record.targets.size() < 2 ||
                    std::any_of(record.targets.begin(), record.targets.end(),
                                [](const SdfPath &p) {
                                    return !p.IsPrimPath();
                                })) {
                    reportError(
                        "RigExecSingleChainIkConstraint " +
                        prim.GetPath().GetString() +
                        " must move every joint in a chain of at least two "
                        "prim targets");
                    return false;
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
                reportError(
                    record.schemaType.GetString() + " " +
                    prim.GetPath().GetString() +
                    " has rigExec:moves but no registered evaluator");
                return false;
            }

            // Matrix movers narrow the general rule (spec §4.2): moves and
            // transform each have cardinality one, and the move target is a
            // native PointBased points property. The optional common weight
            // object was validated above with every other mover envelope.
            if (record.schemaType == "RigExecMatrixMover") {
                std::string error;
                if (!_ValidateMatrixMover(prim, record, &error)) {
                    reportError(error);
                    return false;
                }
            }
            // Blend movers: the mover-owned parameter packet derives
            // deltas against one declared base, so v0.1 supports exactly
            // one canonical points target per blend mover. Multi-target
            // fan-out requires per-target parameter specialization and
            // is rejected rather than silently computed wrong.
            if (record.schemaType == "RigExecBlendShapeMover" &&
                record.targets.size() != 1) {
                reportError("BlendShapeMover " +
                            prim.GetPath().GetString() +
                            " must have exactly one canonical target in "
                            "v0.1 (multi-target fan-out unsupported)");
                return false;
            }
            // Typed geometry movers move points: every canonical target
            // must be a native points property — anything else would
            // pass validation yet silently produce no application.
            {
                static const std::set<TfToken> pointsMoverTypes = {
                    TfToken("RigExecSmoothMover"),
                    TfToken("RigExecVolumeCorrectMover"),
                    TfToken("RigExecLatticeMover"),
                    TfToken("RigExecSurfaceMover"),
                    TfToken("RigExecCurveMover"),
                    TfToken("RigExecCurvenetMover"),
                    TfToken("RigExecBlendShapeMover")};
                if (pointsMoverTypes.count(record.schemaType)) {
                    for (const SdfPath &t : record.targets) {
                        const UsdPrim owner = t.IsPropertyPath()
                            ? _stage->GetPrimAtPath(t.GetPrimPath())
                            : UsdPrim();
                        const UsdAttribute attr = t.IsPropertyPath()
                            ? _stage->GetAttributeAtPath(t)
                            : UsdAttribute();
                        if (!t.IsPropertyPath() ||
                            t.GetNameToken() != "points" ||
                            !owner || !owner.IsA<UsdGeomPointBased>() ||
                            !attr ||
                            attr.GetTypeName() !=
                                SdfValueTypeNames->Point3fArray) {
                            reportError(
                                record.schemaType.GetString() + " " +
                                prim.GetPath().GetString() +
                                " target " + t.GetString() +
                                " is not a native UsdGeomPointBased "
                                "point3f[] points attribute" +
                                _PointsTargetHint(_stage, t));
                            return false;
                        }
                    }
                }
            }
            // Smooth/volume/lattice movers own one mover-level parameter
            // packet resolved against their target, so v0.1 supports
            // exactly one canonical points target each (like blend
            // movers); multi-target fan-out would alias the last
            // target's parameters onto every application.
            if ((record.schemaType == "RigExecSmoothMover" ||
                 record.schemaType == "RigExecVolumeCorrectMover" ||
                 record.schemaType == "RigExecLatticeMover" ||
                 record.schemaType == "RigExecCurvenetMover") &&
                record.targets.size() != 1) {
                reportError(record.schemaType.GetString() + " " +
                            prim.GetPath().GetString() +
                            " must have exactly one canonical points "
                            "target in v0.1 (multi-target fan-out would "
                            "alias mover-level parameters)");
                return false;
            }
            // Property-domain movers: exactly one exact scalar target, and
            // its value type must be the one the mover is statically typed
            // for. These are the third output domain -- a mover revises a
            // float, a vector, or a matrix the same way another revises a
            // points array -- so the target rules are the mirror image of the
            // pointsMoverTypes block above.
            {
                const TfToken &type = record.schemaType;
                const bool isProperty =
                    type == "RigExecFloatMathMover" ||
                    type == "RigExecVec3fMathMover" ||
                    type == "RigExecMatrixMathMover";
                if (isProperty) {
                    if (record.targets.size() != 1) {
                        reportError(
                            type.GetString() + " " +
                            prim.GetPath().GetString() +
                            " must have exactly one target (its parameters "
                            "are mover-level, so a fan-out would alias them "
                            "across targets)");
                        return false;
                    }
                    const SdfPath &t = record.targets[0];
                    // A prim path canonicalizes to .points, which is never
                    // what a property mover means; require the exact
                    // property the author wrote.
                    const UsdAttribute attr = t.IsPropertyPath()
                        ? _stage->GetAttributeAtPath(t)
                        : UsdAttribute();
                    if (!attr) {
                        reportError(
                            type.GetString() + " " +
                            prim.GetPath().GetString() + " target " +
                            t.GetString() +
                            " is not an exact property path");
                        return false;
                    }
                    const SdfValueTypeName valueType = attr.GetTypeName();
                    bool typeOk = false;
                    const char *expected = "";
                    if (type == "RigExecFloatMathMover") {
                        expected = "float";
                        typeOk = valueType == SdfValueTypeNames->Float;
                    } else if (type == "RigExecVec3fMathMover") {
                        // Every GfVec3f-backed scalar role, not just float3:
                        // a mover offsetting a vector3f or a color3f is doing
                        // the identical arithmetic, and refusing it would be
                        // a distinction the kernel does not make.
                        expected = "float3/vector3f/point3f/normal3f/color3f";
                        typeOk =
                            valueType == SdfValueTypeNames->Float3 ||
                            valueType == SdfValueTypeNames->Vector3f ||
                            valueType == SdfValueTypeNames->Point3f ||
                            valueType == SdfValueTypeNames->Normal3f ||
                            valueType == SdfValueTypeNames->Color3f;
                    } else {
                        expected = "matrix4d";
                        typeOk = valueType == SdfValueTypeNames->Matrix4d;
                    }
                    if (!typeOk) {
                        reportError(
                            type.GetString() + " " +
                            prim.GetPath().GetString() + " target " +
                            t.GetString() + " has type " +
                            valueType.GetAsToken().GetString() +
                            "; expected " + expected);
                        return false;
                    }
                    // allowedTokens is advisory in USD, and an unparsed
                    // operation would otherwise fall through to a silent
                    // pass-through every frame.
                    TfToken operation;
                    if (const UsdAttribute a = prim.GetAttribute(
                            TfToken("rigExec:operation"))) {
                        a.Get(&operation);
                    }
                    RigExecPropertyOp op;
                    if (!RigExecParsePropertyOp(operation, &op)) {
                        reportError(
                            type.GetString() + " " +
                            prim.GetPath().GetString() +
                            ": unknown rigExec:operation '" +
                            operation.GetString() + "'");
                        return false;
                    }
                    if (type == "RigExecMatrixMathMover" &&
                        op != RigExecPropertyOp::Multiply &&
                        op != RigExecPropertyOp::Blend) {
                        reportError(
                            type.GetString() + " " +
                            prim.GetPath().GetString() +
                            ": rigExec:operation '" + operation.GetString() +
                            "' has no matrix meaning (multiply or blend)");
                        return false;
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
                        reportError(
                            record.schemaType.GetString() + " " +
                            prim.GetPath().GetString() + ": " +
                            connectionError);
                        return false;
                    }
                }
                SdfPathVector weightObjects;
                if (const UsdRelationship rel = prim.GetRelationship(
                        TfToken("rigExec:weightObject"))) {
                    rel.GetTargets(&weightObjects);
                }
                if (weightObjects.size() > 1) {
                    reportError(
                        record.schemaType.GetString() + " " +
                        prim.GetPath().GetString() +
                        " binds more than one rigExec:weightObject");
                    return false;
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
                                reportError(
                                    record.schemaType.GetString() + " " +
                                    prim.GetPath().GetString() +
                                    ": weighted point-domain cardinality "
                                    "changes across the binding epoch at " +
                                    target.GetString());
                                return false;
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
                            reportError(
                                record.schemaType.GetString() + " " +
                                prim.GetPath().GetString() +
                                ": cannot resolve the weighted point "
                                "domain's compile-time cardinality at " +
                                target.GetString());
                            return false;
                        }
                    }
                    std::set<SdfPath> visitingWeights;
                    std::string weightError;
                    if (!_ValidateWeightObjectDomain(
                            _stage, weightObjects[0], target, pointDomain,
                            operationDomain, logicalCount, &visitingWeights,
                            &weightError)) {
                        reportError(
                            record.schemaType.GetString() + " " +
                            prim.GetPath().GetString() + ": " + weightError);
                        return false;
                    }
                }
            }

            // Strict migration: these concrete mover envelope properties were
            // replaced by MoverAPI inputs:defaultWeight. Once removed from the
            // schema, an old layer opinion composes as a custom attribute and
            // would otherwise be ignored silently.
            {
                const TfToken &type = record.schemaType;
                const bool legacyWeight =
                    type == "RigExecFloatMathMover" ||
                    type == "RigExecVec3fMathMover" ||
                    type == "RigExecMatrixMathMover";
                const bool legacyStrength =
                    type == "RigExecSmoothMover" ||
                    type == "RigExecCurvenetMover" ||
                    type == "RigExecVolumeCorrectMover";
                const auto rejectAuthored =
                    [&](const char *oldName) -> bool {
                    const UsdAttribute old =
                        prim.GetAttribute(TfToken(oldName));
                    if (!old || !old.HasAuthoredValue()) {
                        return false;
                    }
                    reportError(
                        type.GetString() + " " +
                        prim.GetPath().GetString() + " authors " + oldName +
                        ", which was replaced by inputs:defaultWeight");
                    return true;
                };
                if ((legacyWeight && rejectAuthored("inputs:weight")) ||
                    (legacyStrength && rejectAuthored("inputs:strength"))) {
                    return false;
                }
            }
            // Read phases, validated from the AUTHORED stage.
            //
            // Binding resolution parses these too, but it has to be total --
            // it returns a binding, not a verdict -- so an unparseable phase
            // there degrades to `base`. That is the wrong answer delivered
            // silently: the author asked for a specific revision and got the
            // authored value. The parse verdict belongs here, in Phase A,
            // where it can reject the compile before any epoch state moves.
            {
                static const std::pair<const char *, const char *> kPhased[] = {
                    {"rigExec:transform", "rigExec:transformReadPhase"},
                    {"rigExec:cage", "rigExec:cageReadPhase"},
                    {"rigExec:surface", "rigExec:surfaceReadPhase"},
                    {"rigExec:curvenet", nullptr},
                    {"rigExec:bindCoordinates", nullptr},
                    {"rigExec:driverCurve", "rigExec:driverCurveReadPhase"},
                };
                for (const auto &[relName, legacyAttr] : kPhased) {
                    RigExecReadPhase phase;
                    std::string phaseError;
                    bool ok = true;
                    if (const UsdRelationship rel =
                            prim.GetRelationship(TfToken(relName))) {
                        ok = RigExecResolveReadPhase(rel, legacyAttr, &phase,
                                                     &phaseError);
                    } else if (legacyAttr) {
                        if (const UsdAttribute a =
                                prim.GetAttribute(TfToken(legacyAttr))) {
                            ok = RigExecResolveReadPhase(a, legacyAttr, &phase,
                                                         &phaseError);
                        }
                    }
                    if (!ok) {
                        reportError(record.schemaType.GetString() + " " +
                                    prim.GetPath().GetString() + ": " +
                                    phaseError);
                        return false;
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
                "rigExec:mode", "rigExec:operation",
                "rigExec:transformReadPhase", "rigExec:cageReadPhase",
                "rigExec:surfaceReadPhase"};
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
                const std::string who = record.schemaType.GetString() + " " +
                                        prim.GetPath().GetString();
                const auto authored = [&prim](const char *name) {
                    const UsdAttribute a = prim.GetAttribute(TfToken(name));
                    return a && a.HasAuthoredValue();
                };

                // Renamed when the envelope and the per-element weight field
                // collapsed into one concept. The old name is no longer part
                // of the schema, so an authored opinion would compose as a
                // custom property and be ignored.
                if (authored("inputs:weight")) {
                    reportError(who +
                                " authors inputs:weight, which a constraint no"
                                " longer has; the envelope is now"
                                " inputs:defaultWeight");
                    return false;
                }

                // Replaced by the group-qualified spelling, because it named
                // a different channel on every operator that had it.
                for (const char *legacyMask :
                     {"inputs:affectX", "inputs:affectY", "inputs:affectZ"}) {
                    if (authored(legacyMask)) {
                        reportError(
                            who + " authors " + legacyMask +
                            ", which named a different channel on every"
                            " operator; use the group-qualified spelling"
                            " (inputs:affectTranslation*, affectRotation* or"
                            " affectScale*)");
                        return false;
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
                    reportError(
                        who + " authors " + channel.name + ", which it does "
                        "not honor; the operator writes a different channel "
                        "group" +
                        (orderHandler->offsetGroup == _ChannelGroup::None &&
                         !channel.isMask
                             ? " and composes per-source offset arrays instead"
                             : ""));
                    return false;
                }

                if (authored("rigExec:rotationOrder") &&
                    !orderHandler->usesRotationOrder) {
                    reportError(who +
                                " authors rigExec:rotationOrder, which it does"
                                " not honor; only the operators that compose a"
                                " rotation read it");
                    return false;
                }

                // rigExec:preserve is the legacy channel mask at opposite
                // polarity: it names the components the solve must leave
                // alone, where inputs:affect* names the ones it writes. Its
                // default ["origin", "scale"] says an aim writes orientation
                // only, which is exactly what the kernel does -- it modifies
                // the decomposed rotation and reconstructs, leaving
                // translation and scale untouched. So the default needs no
                // implementation; it is already the behavior.
                //
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
                            reportError(
                                who +
                                " authors a non-default rigExec:preserve;"
                                " only [\"origin\", \"scale\"] is"
                                " implemented, which is the orientation-only"
                                " solve the kernel already performs. Use the"
                                " inputs:affect* masks to vary which channels"
                                " are written");
                            return false;
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
                     "rigExec:evaluationMode"});
            }
            for (const char *name : structuralTokens) {
                const UsdAttribute a = prim.GetAttribute(TfToken(name));
                if (a && a.GetNumTimeSamples() > 0) {
                    reportError(record.schemaType.GetString() + " " +
                                prim.GetPath().GetString() + ": " + name +
                                " must not be time-sampled (it selects the "
                                "compiled operation/read phase)");
                    return false;
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
                    reportError(record.schemaType.GetString() + " " +
                                prim.GetPath().GetString() + ": " + name +
                                " has unsupported value '" +
                                value.GetString() + "'");
                    return false;
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
                     {"neverTS", "autoDetect", "alwaysTS"}))) {
                return false;
            }
            newMovers.push_back(std::move(record));
        }
    }

    // A rig has to publish SOMETHING (the check the joint requirement used
    // to stand in for).
    //
    // Controls, joints, volumes, and movers are the four ways it can: a control
    // publishes its posed frame for the synthesized viewport guide, a joint
    // publishes a frame whether or not anything moves it, a placed weight
    // volume publishes its falloff guide while it is being authored, and a
    // mover publishes whatever its targets are. Zero of all four is a rig that
    // evaluates to an empty generation every frame, which is far likelier to
    // be an authoring mistake -- a Movers scope whose contents were renamed
    // out from under it, a rig root pointed at the wrong prim -- than an
    // intent.
    // An inert mover is not an output, but it is evidence of intent: the rig
    // root found mover prims, they simply are not wired yet. Failing that is
    // the same mistake as failing the whole rig for one disconnected mover --
    // it makes the last wire you pull take the rig down. The error is for a
    // rig that found NOTHING, which is the misconfiguration it describes.
    if (newControlPaths.empty() && newJointPaths.empty() &&
        newVolumeWeightPaths.empty() &&
        newMovers.empty() && inertMovers == 0) {
        reportError("Rig publishes no outputs: " + _rigPath.GetString() +
                    " has no RigExecControl, RigExecJoint, or placed volume "
                    "weight prims and no movers");
        return false;
    }

    // Multiple writers of one target are an ordinary stack, not an error.
    //
    // Their order is the reverse-sibling post-order walk of the FINAL COMPOSED
    // hierarchy above. UsdPrim::GetChildrenNames() returns the displayed
    // top-to-bottom order with any parent child-order instruction (reorder
    // nameChildren) already folded in; the stack consumes that order in
    // reverse so the bottom branch runs first. A reorder is a convenience for
    // redirecting that order, never a precondition for having one.
    //
    // This deliberately does not reason about HOW the composed order arose --
    // which layer authored a sibling, which arc contributed it, whether a
    // reorder opinion exists. The compiler reads the final stage and nothing
    // else. An earlier revision rejected non-nested same-target writers that
    // lacked an authored reorder covering both branches; it demanded ceremony
    // (examples/13_ReadPhases.usda restated its own file order to satisfy it)
    // while not actually preventing the precedence surprises it cited, which
    // come from arc order and weak-side insertion rather than from a missing
    // reorder opinion.

    // final transform reads are legal only when every writer of that
    // provider precedes the reader in logical order (spec §4.2): a
    // frame mover with a later ordinal than a consuming matrix mover is
    // an unsatisfied final read.
    {
        std::map<SdfPath, int> lastFrameWriterOrdinal;
        for (const RigExecMoverRecord &m : newMovers) {
            if (!_IsFrameConstraintType(m.schemaType)) {
                continue;
            }
            for (const SdfPath &t : m.targets) {
                if (t.IsPrimPath()) {
                    lastFrameWriterOrdinal[t] = std::max(
                        lastFrameWriterOrdinal.count(t)
                            ? lastFrameWriterOrdinal[t] : -1,
                        m.ordinal);
                }
            }
        }
        for (const RigExecMoverRecord &m : newMovers) {
            if (m.schemaType != "RigExecMatrixMover") {
                continue;
            }
            const UsdPrim prim = _stage->GetPrimAtPath(m.moverPath);
            if (!prim) {
                continue;
            }
            TfToken phase("base");
            if (UsdAttribute a = prim.GetAttribute(
                    TfToken("rigExec:transformReadPhase"))) {
                a.Get(&phase);
            }
            if (phase != "final") {
                continue;
            }
            SdfPathVector transforms;
            if (UsdRelationship rel =
                    prim.GetRelationship(TfToken("rigExec:transform"))) {
                rel.GetTargets(&transforms);
            }
            for (const SdfPath &provider : transforms) {
                const auto it = lastFrameWriterOrdinal.find(provider);
                if (it != lastFrameWriterOrdinal.end() &&
                    it->second > m.ordinal) {
                    reportError(
                        "Unsatisfied final read: " +
                        m.moverPath.GetString() + " reads final of " +
                        provider.GetString() +
                        " but a writer with a later ordinal exists "
                        "(spec §4.2)");
                    return false;
                }
            }
        }
    }

    // View-free solver->joint binding validation. This
    // runs in Phase A, BEFORE any epoch teardown, so an invalid binding
    // rejects the compile while the previous epoch stays publishable (the
    // BLOCKER fix: compile Pass 0 must never be the first place a bad
    // binding is discovered, because by then the old layer is gone and
    // restoration would re-hit the same invalid state). Recursive over the
    // composed Solvers subtree. Authored-conflict checks read the SOURCE
    // stage (_stage).
    //
    // joint -> (posing solver, element). The validation below already
    // resolves and bounds-checks exactly this pair; keeping it is what lets
    // Evaluate extract each bound joint's frame from its solver's aggregate
    // and supply it as a value override, so the binding never has to be
    // authored anywhere (it used to become rigExec:frameSource /
    // rigExec:frameElement on the joint, in the derived layer).
    std::map<SdfPath, std::pair<SdfPath, int>> newJointBinding;
    {
        std::map<SdfPath, SdfPath> jointClaim;  // joint -> claiming solver
        {
            const std::vector<UsdPrim> solvers =
                _DiscoverAggregateSolvers(_stage, _rigPath);
            // Cardinality attributes must be static (compile-time
            // structural) for EVERY aggregate solver under the rig, not
            // only joint-bearing ones: a non-joint Twist/Ribbon feeding a
            // joint-bearing Blend still determines that Blend's element
            // count, so a time-sampled cardinality would silently shift a
            // blend-bound joint's frame. `uniform` is only
            // a hint; reject samples explicitly.
            for (const UsdPrim &solver : solvers) {
                const TfToken t = solver.GetTypeName();
                std::vector<const char *> cardinalityAttrs;
                if (t == "RigExecTwistDistribution") {
                    cardinalityAttrs = {"rigExec:count", "rigExec:weights"};
                } else if (t == "RigExecRibbon") {
                    cardinalityAttrs = {"rigExec:sampleCount"};
                }
                for (const char *name : cardinalityAttrs) {
                    const UsdAttribute a = solver.GetAttribute(TfToken(name));
                    if (a && a.GetNumTimeSamples() > 0) {
                        reportError(
                            solver.GetTypeName().GetString() + " " +
                            solver.GetPath().GetString() + ": " + name +
                            " must not be time-sampled (it defines frame "
                            "cardinality)");
                        return false;
                    }
                }
            }
            // A claim binds wherever the solver sits; a non-solver prim
            // carrying rigExec:joints is rejected wherever it sits.
            if (const UsdPrim rig = _stage->GetPrimAtPath(_rigPath)) {
                for (const UsdPrim &solver : UsdPrimRange(rig)) {
                    const UsdRelationship jointsRel =
                        solver.GetRelationship(TfToken("rigExec:joints"));
                    if (!jointsRel) {
                        continue;
                    }
                    SdfPathVector jointTargets;
                    jointsRel.GetTargets(&jointTargets);
                    if (jointTargets.empty()) {
                        continue;
                    }
                    const std::string who = solver.GetTypeName().GetString() +
                                            " " + solver.GetPath().GetString();

                    // The claimant must be a real aggregate solver (it must
                    // publish computePointFrameArray for the joint to extract).
                    // A Scope or arbitrary prim carrying rigExec:joints would
                    // otherwise pass, and the binding would name a prim with no
                    // aggregate computation to extract an element from.
                    if (!_IsAggregateSolverType(solver.GetTypeName())) {
                        reportError(who + " authors rigExec:joints but is not a "
                                          "recognized aggregate solver type");
                        return false;
                    }

                    // jointElements parallel-array shape: empty, or exactly one
                    // entry per joint (no partial remap / silent truncation).
                    VtIntArray jointElements;
                    if (const UsdAttribute a = solver.GetAttribute(
                            TfToken("rigExec:jointElements"))) {
                        std::vector<double> times;
                        if (a.GetTimeSamples(&times) && !times.empty()) {
                            reportError(who + ": rigExec:jointElements must not "
                                              "carry time samples");
                            return false;
                        }
                        a.Get(&jointElements);
                    }
                    if (!jointElements.empty() &&
                        jointElements.size() != jointTargets.size()) {
                        reportError(
                            who + ": rigExec:jointElements length " +
                            std::to_string(jointElements.size()) +
                            " must equal rigExec:joints length " +
                            std::to_string(jointTargets.size()));
                        return false;
                    }

                    // Knowable aggregate element count per solver type
                    // (-1 = not cheaply knowable, e.g. a blend of another
                    // aggregate: element bounds are enforced only at runtime).
                    int knownCount = -1;
                    const TfToken type = solver.GetTypeName();
                    if (type == "RigExecTwoBoneIk") {
                        knownCount = 3;
                    } else if (type == "RigExecFkChain") {
                        SdfPathVector controls;
                        if (const UsdRelationship c = solver.GetRelationship(
                                TfToken("rigExec:controls"))) {
                            c.GetTargets(&controls);
                        }
                        knownCount = static_cast<int>(controls.size());
                    } else if (type == "RigExecTwistDistribution") {
                        // count/weights time samples already rejected above.
                        VtFloatArray weights;
                        if (const UsdAttribute a = solver.GetAttribute(
                                TfToken("rigExec:weights"))) {
                            a.Get(&weights);
                        }
                        if (!weights.empty()) {
                            knownCount = static_cast<int>(weights.size());
                        } else if (const UsdAttribute a = solver.GetAttribute(
                                       TfToken("rigExec:count"))) {
                            int c = 1;
                            a.Get(&c);
                            knownCount = std::max(c, 1);
                        }
                    } else if (type == "RigExecRibbon") {
                        if (const UsdAttribute a = solver.GetAttribute(
                                TfToken("rigExec:sampleCount"))) {
                            int c = 5;
                            a.Get(&c);
                            // The transported-frame ribbon needs >= 2 samples;
                            // below that it produces no frames, so no element
                            // is bindable (matches the runtime cardinality).
                            knownCount = c >= 2 ? c : 0;
                        }
                    }

                    for (size_t i = 0; i < jointTargets.size(); ++i) {
                        const SdfPath &jointPath = jointTargets[i];
                        const int element = i < jointElements.size()
                            ? jointElements[i] : static_cast<int>(i);
                        if (element < 0) {
                            reportError(who + ": negative element index " +
                                        std::to_string(element) + " for " +
                                        jointPath.GetString());
                            return false;
                        }
                        if (knownCount >= 0 && element >= knownCount) {
                            reportError(
                                who + ": element " + std::to_string(element) +
                                " for " + jointPath.GetString() +
                                " is out of range (solver produces " +
                                std::to_string(knownCount) + " frames)");
                            return false;
                        }
                        const UsdPrim jointPrim =
                            _stage->GetPrimAtPath(jointPath);
                        if (!jointPrim) {
                            reportError(who + " rigExec:joints targets missing "
                                              "prim " + jointPath.GetString());
                            return false;
                        }
                        if (jointPrim.GetTypeName() != "RigExecJoint") {
                            reportError(
                                who + " rigExec:joints target " +
                                jointPath.GetString() + " is a " +
                                jointPrim.GetTypeName().GetString() +
                                ", not a RigExecJoint");
                            return false;
                        }
                        // Every joint is tapped individually and a bound one also
                        // carries a per-prim value override, both of which key on
                        // a real prim; only the rig's nearest instanceable
                        // ancestor is deinstanced. Reject an instance-proxy /
                        // prototype-hosted joint up front.
                        if (jointPrim.IsInstanceProxy() ||
                            jointPrim.IsInPrototype()) {
                            reportError(
                                who + " rigExec:joints target " +
                                jointPath.GetString() +
                                " is instance-proxy/prototype hosted and cannot "
                                "receive a solver binding");
                            return false;
                        }
                        // Exclusive ownership: a solver-posed joint must not
                        // also author its own posed:space connection (the solver
                        // pose is supplied as an override and would silently win
                        // over the connection the raw stage shows).
                        //
                        // There is no longer a companion check for a legacy
                        // authored rigExec:frameSource. Nothing reads that name
                        // now -- it is neither a schema property nor a
                        // registered computation input -- so a leftover opinion
                        // from an asset saved against the old schema is inert,
                        // and failing the compile over it would reject a rig
                        // that evaluates correctly.
                        if (const UsdPrim srcJoint =
                                _stage->GetPrimAtPath(jointPath)) {
                            if (const UsdAttribute posed = srcJoint.GetAttribute(
                                    TfToken("posed:space"))) {
                                SdfPathVector conns;
                                posed.GetConnections(&conns);
                                if (!conns.empty()) {
                                    reportError(who + ": joint " +
                                                jointPath.GetString() +
                                                " also connects posed:space");
                                    return false;
                                }
                            }
                        }
                        const auto claimed = jointClaim.find(jointPath);
                        if (claimed != jointClaim.end()) {
                            reportError("joint " + jointPath.GetString() +
                                        " is posed by two solvers (" +
                                        claimed->second.GetString() + " and " +
                                        solver.GetPath().GetString() + ")");
                            return false;
                        }
                        jointClaim[jointPath] = solver.GetPath();
                        newJointBinding[jointPath] = {solver.GetPath(), element};
                    }
                }
            }
        }
    }

    // Solver->solver acyclicity. Unique joint ownership does NOT imply this:
    // two solvers can each uniquely pose their own joints while reading each
    // other's, which is a genuine cycle. Evaluate resolves the
    // solver->joint overrides by iterating to a fixed point, and a cycle has
    // no fixed point to reach -- so reject it here, where the author gets a
    // path, instead of discovering it as a non-converging generation.
    //
    // Edges are derived generically: any relationship on a solver whose target
    // is a joint that another solver poses makes this solver depend on that
    // one. That needs no per-solver-type table of which relationships carry
    // frames, so a new solver type cannot quietly escape the check.
    {
        std::map<SdfPath, std::set<SdfPath>> dependsOn;
        const std::vector<UsdPrim> solvers =
            _DiscoverAggregateSolvers(_stage, _rigPath);
        std::set<SdfPath> solverPaths;
        for (const UsdPrim &solver : solvers) {
            solverPaths.insert(solver.GetPath());
        }
        for (const UsdPrim &solver : solvers) {
            for (const UsdRelationship &rel :
                 solver.GetRelationships()) {
                if (rel.GetName() == "rigExec:joints") {
                    continue;  // what it poses, not what it reads
                }
                SdfPathVector targets;
                rel.GetTargets(&targets);
                for (const SdfPath &target : targets) {
                    const SdfPath targetPrim = target.GetPrimPath();
                    // Indirect: reading a joint that another solver poses.
                    const auto it = newJointBinding.find(targetPrim);
                    if (it != newJointBinding.end() &&
                        it->second.first != solver.GetPath()) {
                        dependsOn[solver.GetPath()].insert(
                            it->second.first);
                    }
                    // Direct: reading another solver's aggregate.
                    // RigExecBlendPointFrames takes rigExec:inputA /
                    // inputB as solver paths, so a cycle can run through
                    // a solver->solver edge without touching a joint at
                    // all; deriving only the joint edges would miss it.
                    // Membership, not a scope prefix, decides: solvers live
                    // wherever the author put them.
                    if (targetPrim != solver.GetPath() &&
                        solverPaths.count(targetPrim)) {
                        dependsOn[solver.GetPath()].insert(targetPrim);
                    }
                }
            }
        }
        // Iterative DFS with a colour map: grey means on the current stack.
        std::map<SdfPath, int> colour;  // 0 unvisited, 1 grey, 2 black
        std::vector<SdfPath> stack;
        std::function<bool(const SdfPath &)> visit =
            [&](const SdfPath &node) -> bool {
            colour[node] = 1;
            stack.push_back(node);
            for (const SdfPath &next : dependsOn[node]) {
                if (colour[next] == 1) {
                    std::string cycle;
                    bool started = false;
                    for (const SdfPath &p : stack) {
                        if (p == next) {
                            started = true;
                        }
                        if (started) {
                            cycle += p.GetString() + " -> ";
                        }
                    }
                    cycle += next.GetString();
                    reportError("solver dependency cycle: " + cycle +
                                " (each solver reads a joint the next one "
                                "poses, so no pose can be resolved)");
                    return false;
                }
                if (colour[next] == 0 && !visit(next)) {
                    return false;
                }
            }
            stack.pop_back();
            colour[node] = 2;
            return true;
        };
        for (const auto &[solverPath, _] : dependsOn) {
            if (colour[solverPath] == 0 && !visit(solverPath)) {
                return false;
            }
        }
    }

    const size_t newDigest = _ComputeStructureDigest();

    // Phase B: epoch replacement. This used to remove and re-author an
    // owned sublayer of generated prims, with a restore path for when the
    // authoring failed. Nothing is authored now, so the whole epoch swap is
    // just tearing down the previous exec system before building the next.
    _guideTaps.reset();
    _taps.reset();
    _restFrameTaps.reset();

    auto restorePreviousEpoch = [&]() {
        // The previous request was torn down with its taps above, so a
        // failed compile leaves the evaluator uncompiled either way.
        _compiled = false;
    };

    // Phase C: build and prepare the new epoch's taps before committing
    // any evaluator state; a request that cannot be built valid must not
    // become a "successful" epoch. Public tap addresses stay canonical
    // (native property or provider prim plus computation/phase);
    // generated chain heads are private resolutions (spec §9.1).
    static const TfToken basePhase("base");
    static const TfToken finalPhase("final");
    auto newTaps = std::make_unique<RigExecTapSet>(_stage);
    std::vector<RigExecTapId> newJointFrameTaps;
    std::vector<RigExecTapId> newJointFinalFrameTaps;
    std::vector<RigExecTapId> newJointFinalMatrixTaps;
    std::map<SdfPath, RigExecTapId> newSolverArrayTaps;
    std::map<SdfPath, RigExecTapId> newJointSolverArrayTaps;

    // Volumetric weight epoch state (spec §4.1 volumetric extension).
    std::vector<RigExecValueOverride> newFalloffLutOverrides;
    std::set<SdfPath> newCurrentPhaseWeights;
    std::map<SdfPath, RigExecTapId> newVolumeWeightMatrixTaps;

    // Walks a weight object and everything it composes, gathering what
    // the volumetric types need beyond their computeWeightPacket tap.
    // Depth-limited for the same reason the structure digest is: a cycle
    // is authoring error, and the bound only has to keep this
    // terminating.
    // Structural authoring errors on a volume weight, collected during
    // the walk below and reported before the epoch commits.
    //
    // These are cardinality rules on the points-bearing relationships,
    // and they exist because the two evaluation paths CANNOT disagree
    // about them safely: the exec kernel receives a relationship's
    // targets as one flattened value stream, so two targets on
    // rigExec:curve silently concatenate into one polyline with a
    // spurious segment joining them, while the CPU oracle reads targets
    // explicitly and rejects the pair. Catching it here means neither
    // path ever sees the ambiguous authoring.
    std::string volumeWeightError;

    // Weight objects currently being visited, for cycle detection. A
    // cycle is an authoring error and must be DIAGNOSED, not survived:
    // the CPU resolver recurses through the same edges with no depth
    // guard of its own, so an undetected cycle exhausts the stack rather
    // than producing a bad answer.
    std::set<SdfPath> visiting;

    // Returns true when this weight object, or anything it composes,
    // samples the in-flight points.
    //
    // The answer has to propagate UP: the graph build loop tests the
    // weight object a mover actually binds, which for a composed field is
    // the combine, not the sphere inside it. Recording only the leaf left
    // exec applying the reference-phase packet while the CPU oracle
    // reached a leaf with no in-flight points and failed.
    std::function<bool(const SdfPath &)> registerVolumeWeights =
        [&](const SdfPath &weightPath) -> bool {
        const UsdPrim w = _stage->GetPrimAtPath(weightPath);
        if (!w) {
            return false;
        }
        if (!visiting.insert(weightPath).second) {
            volumeWeightError =
                weightPath.GetString() +
                ": weight object composition contains a cycle";
            return false;
        }
        struct Pop {
            std::set<SdfPath> &s;
            const SdfPath &p;
            ~Pop() { s.erase(p); }
        } pop{visiting, weightPath};

        if (newVolumeWeightMatrixTaps.count(weightPath) ||
            newCurrentPhaseWeights.count(weightPath)) {
            // Already walked through another consumer; its answer stands.
            return newCurrentPhaseWeights.count(weightPath) != 0;
        }
        bool isCurrent = false;
        if (_IsVolumeWeightType(w.GetTypeName())) {
            auto requireTargets = [&](const char *rel, size_t exact,
                                      const char *what) {
                SdfPathVector targets;
                if (UsdRelationship r = w.GetRelationship(TfToken(rel))) {
                    r.GetTargets(&targets);
                }
                if (targets.size() > exact) {
                    volumeWeightError =
                        weightPath.GetString() + ": " + rel + " must name " +
                        what;
                }
                return targets.size();
            };
            // At most one sampling override; exactly one curve for a
            // curve weight.
            requireTargets("rigExec:sampleSource", 1,
                           "at most one points source");
            if (w.GetTypeName() == "RigExecCurveWeight") {
                SdfPathVector curves;
                if (const UsdRelationship rel = w.GetRelationship(
                        TfToken("rigExec:curve"))) {
                    rel.GetTargets(&curves);
                }
                if (curves.size() != 1) {
                    volumeWeightError =
                        weightPath.GetString() +
                        ": rigExec:curve must name exactly one points source";
                } else {
                    const SdfPath pointsPath = curves[0].IsPropertyPath()
                        ? curves[0]
                        : curves[0].AppendProperty(TfToken("points"));
                    const UsdAttribute points =
                        _stage->GetAttributeAtPath(pointsPath);
                    if (!points ||
                        points.GetTypeName() != SdfValueTypeNames->Point3fArray) {
                        volumeWeightError =
                            weightPath.GetString() +
                            ": rigExec:curve target " +
                            curves[0].GetString() +
                            " must resolve to a point3f[] points source";
                    }
                }
            }
        }
        if (_IsVolumeWeightType(w.GetTypeName())) {
            // computePointFrame, NOT computeMatrix: the latter is the
            // rest->posed map, so an unanimated volume's is the identity
            // and its field would land at the origin however the prim is
            // placed. See _RigidWorldToLocal in moverKernels.cpp.
            newVolumeWeightMatrixTaps[weightPath] =
                newTaps->Add(RigExecValueAddress::Prim(
                    weightPath, _computePointFrame));

            RigExecValueOverride lutOverride;
            lutOverride.prim = weightPath;
            lutOverride.computation = _computeFalloffLut;
            RigExecFalloffLut lut;
            lut.samples = _BakeFalloffLut(w);
            lutOverride.value = VtValue(lut);
            newFalloffLutOverrides.push_back(std::move(lutOverride));

            TfToken phase("reference");
            if (UsdAttribute a = w.GetAttribute(_samplePhaseAttr)) {
                a.Get(&phase);
            }
            if (phase == "current") {
                newCurrentPhaseWeights.insert(weightPath);
                isCurrent = true;
            }
        }
        for (const char *rel :
             {"rigExec:inputWeights", "rigExec:baseWeight"}) {
            SdfPathVector targets;
            if (UsdRelationship r = w.GetRelationship(TfToken(rel))) {
                r.GetTargets(&targets);
            }
            for (const SdfPath &t : targets) {
                // Not short-circuited: every reachable weight object
                // still needs its matrix tap and LUT override, so the
                // walk must complete even once the answer is known.
                if (registerVolumeWeights(t)) {
                    isCurrent = true;
                }
            }
        }
        // A composed field is current-phase if anything inside it is, so
        // that the combine a mover actually binds tests true.
        if (isCurrent) {
            newCurrentPhaseWeights.insert(weightPath);
        }
        return isCurrent;
    };

    // A placed volume is a guide output even before it is bound to a mover.
    // Register every discovered volume first; the recursive consumer walks
    // below naturally deduplicate against this map. This also applies the same
    // CurveWeight cardinality validation to standalone and consumed volumes.
    for (const SdfPath &volumePath : newVolumeWeightPaths) {
        registerVolumeWeights(volumePath);
    }

    // Gather volumetric epoch state for every common mover envelope, not only
    // point-graph revisions. Geometry-domain constraints publish directly
    // after the pose walk and therefore never appear in _graphChains, but a
    // sphere/plane/curve field bound to one still needs the same placement tap
    // and falloff override as a geometry mover.
    for (const RigExecMoverRecord &mover : newMovers) {
        const UsdPrim moverPrim = _stage->GetPrimAtPath(mover.moverPath);
        if (!moverPrim) {
            continue;
        }
        SdfPathVector objects;
        if (const UsdRelationship rel = moverPrim.GetRelationship(
                TfToken("rigExec:weightObject"))) {
            rel.GetTargets(&objects);
        }
        if (objects.size() == 1) {
            registerVolumeWeights(objects[0]);
        }
    }

    // The aggregate frame array of every solver that poses a joint, in its
    // own request so it can be evaluated first: the authoritative request
    // below is computed with each bound joint's frame overridden by an
    // element of these. Solvers read controls, never joints, so there is no
    // cycle between the two requests.
    auto newSolverFrameTaps = std::make_unique<RigExecTapSet>(_stage);
    for (const auto &[jointPath, binding] : newJointBinding) {
        const SdfPath &solverPath = binding.first;
        if (newJointSolverArrayTaps.count(solverPath)) {
            continue;
        }
        newJointSolverArrayTaps[solverPath] = newSolverFrameTaps->Add(
            RigExecValueAddress::Prim(solverPath, _computePointFrameArray));
    }
    if (!newJointSolverArrayTaps.empty() && !newSolverFrameTaps->Prepare()) {
        reportError("failed to build a valid prepared request for the "
                    "solver frame aggregates");
        restorePreviousEpoch();
        return false;
    }

    // Rest-implied TwoBoneIk lengths. A solver whose absolute length
    // carries no authored opinion measures that bone from its bound
    // joints' rest positions at Evaluate time (plus the authored
    // offset); an authored absolute implies nothing. Element order is
    // root/mid/end, honoring any rigExec:jointElements remap via the
    // compiled binding rather than raw list position.
    std::vector<_ImpliedIkLengths> newImplied;
    auto newRestTaps = std::make_unique<RigExecTapSet>(_stage);
    std::map<SdfPath, RigExecTapId> newImpliedRestTaps;
    {
        std::map<SdfPath, std::map<int, SdfPath>> elementsBySolver;
        for (const auto &[jointPath, binding] : newJointBinding) {
            elementsBySolver[binding.first][binding.second] = jointPath;
        }
        for (const UsdPrim &solver :
             _DiscoverAggregateSolvers(_stage, _rigPath)) {
            if (solver.GetTypeName() != "RigExecTwoBoneIk") {
                continue;
            }
            const UsdAttribute upper = solver.GetAttribute(
                TfToken("rigExec:upperLength"));
            const UsdAttribute lower = solver.GetAttribute(
                TfToken("rigExec:lowerLength"));
            _ImpliedIkLengths record;
            record.solver = solver.GetPath();
            record.implyUpper = !upper || !upper.HasAuthoredValueOpinion();
            record.implyLower = !lower || !lower.HasAuthoredValueOpinion();
            if (!record.implyUpper && !record.implyLower) {
                continue;
            }
            const auto found = elementsBySolver.find(solver.GetPath());
            const bool hasBinding = found != elementsBySolver.end();
            for (int element = 0; element < 3 && hasBinding; ++element) {
                const auto joint = found->second.find(element);
                if (joint == found->second.end()) {
                    break;
                }
                record.joints[element] = joint->second;
                if (!newImpliedRestTaps.count(joint->second)) {
                    newImpliedRestTaps[joint->second] = newRestTaps->Add(
                        RigExecValueAddress::Prim(
                            joint->second, TfToken("computeRestFrame")));
                }
            }
            // A solver binding fewer than three elements keeps its schema
            // defaults; Evaluate diagnoses which unauthored length could
            // not be implied. Compile stays open: a solver wired before
            // its joints are finished still compiles, as it always has.
            newImplied.push_back(record);
        }
    }
    if (!newImpliedRestTaps.empty() && !newRestTaps->Prepare()) {
        reportError("failed to build a valid prepared request for the "
                    "implied-length rest frames");
        restorePreviousEpoch();
        return false;
    }

    // Pose-domain constraints, compiled to in-memory structural wiring. Aim,
    // Position, Rotation, Scale, and Parent revise one transform provider;
    // SingleChainIK revises its inferred joint chain atomically. Values stay
    // authored on the mover and are sampled during Evaluate().
    std::vector<_FrameConstraint> newFrameConstraints;
    std::map<SdfPath, std::vector<SdfPath>> newFrameChains;
    std::map<SdfPath, RigExecTapId> newProviderRestFrameTaps;
    std::map<SdfPath, RigExecTapId> newProviderBaseFrameTaps;

    auto getTargets = [](const UsdPrim &prim, const char *name) {
        SdfPathVector paths;
        if (const UsdRelationship rel =
                prim.GetRelationship(TfToken(name))) {
            rel.GetTargets(&paths);
        }
        return paths;
    };

    auto bindFrameSource = [&](const SdfPath &authored,
                               const std::string &role,
                               _FrameSourceBinding *binding) {
        if (!authored.IsPrimPath()) {
            reportError(role + " must target a prim, got " +
                        authored.GetString());
            return false;
        }
        const UsdPrim sourcePrim = _stage->GetPrimAtPath(authored);
        if (!sourcePrim) {
            reportError(role + " targets missing prim " +
                        authored.GetString());
            return false;
        }
        binding->sourcePath = authored;
        const TfToken sourceType = sourcePrim.GetTypeName();
        if (sourceType == "RigExecControl" ||
            sourceType == "RigExecJoint") {
            binding->frameTap = newTaps->Add(
                RigExecValueAddress::Prim(authored, _computePointFrame,
                                          basePhase));
            return true;
        }
        if (UsdGeomXformable(sourcePrim)) {
            binding->xformPath = authored;
            return true;
        }
        reportError(role + " targets " + authored.GetString() +
                    ", which is neither a RigExec transform provider nor "
                    "a UsdGeomXformable");
        return false;
    };

    for (const RigExecMoverRecord &mover : newMovers) {
        if (!_IsFrameConstraintType(mover.schemaType)) {
            continue;
        }
        const UsdPrim moverPrim = _stage->GetPrimAtPath(mover.moverPath);
        if (!moverPrim) {
            continue;
        }
        _FrameConstraint constraint;
        constraint.moverPath = mover.moverPath;
        constraint.schemaType = mover.schemaType;
        const SdfPathVector commonWeights =
            getTargets(moverPrim, "rigExec:weightObject");
        if (!commonWeights.empty()) {
            // Phase-A validation already established at-most-one and the
            // exact domain: the moved prim for a source constraint, or this
            // mover prim for an atomic multi-target SingleChainIK.
            constraint.weightObject = commonWeights[0];
        }

        if (_IsSourceFrameConstraintType(mover.schemaType)) {
            constraint.targets = mover.targets;

            // Domain selection, by the authored spelling alone. A bare prim
            // path is the transform domain; <prim>.points is the geometry
            // domain. Either way the frame key is the PRIM -- the solve is
            // identical and only the publish differs -- so the points
            // property is carried aside and targets[0] is normalized.
            if (!constraint.targets.empty() &&
                constraint.targets[0].IsPropertyPath() &&
                constraint.targets[0].GetNameToken() == "points") {
                constraint.pointsTarget = constraint.targets[0];
                constraint.targets[0] = constraint.targets[0].GetPrimPath();
            }

            SdfPathVector sources = getTargets(moverPrim, "rigExec:sources");
            if (sources.empty() && mover.schemaType ==
                                       "RigExecAimConstraint") {
                // Backward-compatible spelling used by every existing Aim
                // asset. New assets use the ordered FBX-style sources list.
                sources = getTargets(moverPrim, "rigExec:aimTarget");
            }
            if (sources.empty()) {
                reportError(mover.schemaType.GetString() + " " +
                            mover.moverPath.GetString() +
                            " has no constraint sources");
                restorePreviousEpoch();
                return false;
            }
            for (const SdfPath &source : sources) {
                _FrameSourceBinding binding;
                if (!bindFrameSource(
                        source,
                        mover.schemaType.GetString() + " " +
                            mover.moverPath.GetString() + " source",
                        &binding)) {
                    restorePreviousEpoch();
                    return false;
                }
                constraint.sources.push_back(binding);
            }

            if (mover.schemaType == "RigExecAimConstraint") {
                const SdfPathVector upObjects =
                    getTargets(moverPrim, "rigExec:worldUpObject");
                if (upObjects.size() > 1) {
                    reportError("RigExecAimConstraint " +
                                mover.moverPath.GetString() +
                                " has more than one world-up object");
                    restorePreviousEpoch();
                    return false;
                }
                if (!upObjects.empty() &&
                    !bindFrameSource(
                        upObjects[0],
                        "RigExecAimConstraint " +
                            mover.moverPath.GetString() + " world-up object",
                        &constraint.worldUpObject)) {
                    restorePreviousEpoch();
                    return false;
                }
            }
        } else {
            // FBX SingleChainIK names endpoints, not an ordered output list.
            // RigExec joint hierarchy is namespace nesting, so the exact
            // chain is inferred by walking End Joint's ancestors to First
            // Joint. rigExec:moves must declare that complete set.
            const SdfPathVector first =
                getTargets(moverPrim, "rigExec:firstJoint");
            const SdfPathVector end =
                getTargets(moverPrim, "rigExec:endJoint");
            const SdfPathVector effector =
                getTargets(moverPrim, "rigExec:effector");
            if (first.size() != 1 || end.size() != 1 ||
                effector.size() != 1 || !first[0].IsPrimPath() ||
                !end[0].IsPrimPath()) {
                reportError("RigExecSingleChainIkConstraint " +
                            mover.moverPath.GetString() +
                            " requires exactly one firstJoint, endJoint, "
                            "and effector prim");
                restorePreviousEpoch();
                return false;
            }
            SdfPath cursor = end[0];
            while (!cursor.IsEmpty() && cursor != SdfPath::AbsoluteRootPath()) {
                const UsdPrim joint = _stage->GetPrimAtPath(cursor);
                if (!joint || joint.GetTypeName() != "RigExecJoint") {
                    reportError("RigExecSingleChainIkConstraint " +
                                mover.moverPath.GetString() +
                                " endpoint ancestry contains non-joint " +
                                cursor.GetString());
                    restorePreviousEpoch();
                    return false;
                }
                constraint.ikChain.push_back(cursor);
                if (cursor == first[0]) {
                    break;
                }
                cursor = cursor.GetParentPath();
            }
            if (constraint.ikChain.empty() ||
                constraint.ikChain.back() != first[0]) {
                reportError("RigExecSingleChainIkConstraint " +
                            mover.moverPath.GetString() + ": endJoint " +
                            end[0].GetString() +
                            " is not a namespace descendant of firstJoint " +
                            first[0].GetString());
                restorePreviousEpoch();
                return false;
            }
            std::reverse(constraint.ikChain.begin(),
                         constraint.ikChain.end());
            if (constraint.ikChain.size() < 2) {
                reportError("RigExecSingleChainIkConstraint " +
                            mover.moverPath.GetString() +
                            " needs at least two joints");
                restorePreviousEpoch();
                return false;
            }
            std::set<SdfPath> declared(mover.targets.begin(),
                                       mover.targets.end());
            std::set<SdfPath> inferred(constraint.ikChain.begin(),
                                       constraint.ikChain.end());
            if (declared != inferred) {
                reportError("RigExecSingleChainIkConstraint " +
                            mover.moverPath.GetString() +
                            " rigExec:moves must equal the complete inferred "
                            "firstJoint-to-endJoint chain");
                restorePreviousEpoch();
                return false;
            }
            constraint.targets = constraint.ikChain;
            if (!bindFrameSource(
                    effector[0],
                    "RigExecSingleChainIkConstraint " +
                        mover.moverPath.GetString() + " effector",
                    &constraint.effector)) {
                restorePreviousEpoch();
                return false;
            }
            // SingleChain deliberately ignores every pole input. Do not even
            // bind these relationships: an otherwise malformed dormant pole
            // must not make the selected solver mode fail to compile.
            TfToken solverMode("rotatePlane");
            if (const UsdAttribute a = moverPrim.GetAttribute(
                    TfToken("rigExec:solverMode"))) {
                a.Get(&solverMode);
            }
            if (solverMode == "rotatePlane") {
                for (const SdfPath &pole :
                     getTargets(moverPrim, "rigExec:poleVectorObjects")) {
                    _FrameSourceBinding binding;
                    if (!bindFrameSource(
                            pole,
                            "RigExecSingleChainIkConstraint " +
                                mover.moverPath.GetString() +
                                " pole-vector object",
                            &binding)) {
                        restorePreviousEpoch();
                        return false;
                    }
                    constraint.poleObjects.push_back(binding);
                }
            }
        }

        for (const SdfPath &target : constraint.targets) {
            newFrameChains[target].push_back(mover.moverPath);
        }
        newFrameConstraints.push_back(std::move(constraint));
    }
    // A provider carrying pose revisions that is not a joint needs a base
    // frame from somewhere. A Control has computePointFrame like a joint; a
    // plain UsdGeomXformable has no exec computation at all, so its frame
    // comes from its own USD transform and its revised matrix is published
    // back onto the prim for Hydra to inherit.
    std::set<SdfPath> newXformDerivedProviders;
    for (const auto &[provider, revisions] : newFrameChains) {
        if (std::find(newJointPaths.begin(), newJointPaths.end(), provider) !=
            newJointPaths.end()) {
            continue;  // joints are tapped below
        }
        const UsdPrim providerPrim = _stage->GetPrimAtPath(provider);
        const TfToken type = providerPrim ? providerPrim.GetTypeName()
                                          : TfToken();
        if (type == "RigExecControl" || type == "RigExecJoint") {
            newProviderBaseFrameTaps[provider] =
                newTaps->Add(RigExecValueAddress::Prim(
                    provider, _computePointFrame, basePhase));
        } else if (providerPrim && UsdGeomXformable(providerPrim)) {
            newXformDerivedProviders.insert(provider);
        } else {
            reportError(
                "constraint target " + provider.GetString() +
                " is neither a RigExec transform provider nor a "
                "UsdGeomXformable; nothing can carry the revised frame");
                restorePreviousEpoch();
            return false;
        }
    }

    // Rest frames, only for providers that actually publish one. A plain
    // UsdGeomXformable has no computeRestFrame -- requesting it is a hard exec
    // failure, not a missing value -- and needs none: its rest is identity,
    // which is what the published revision is measured against.
    for (const auto &[provider, revisions] : newFrameChains) {
        if (newXformDerivedProviders.count(provider)) {
            continue;
        }
        newProviderRestFrameTaps[provider] =
            newTaps->Add(RigExecValueAddress::Prim(
                provider, TfToken("computeRestFrame")));
    }

    for (const SdfPath &jointPath : newJointPaths) {
        newJointFrameTaps.push_back(newTaps->Add(RigExecValueAddress::Prim(
            jointPath, _computePointFrame, basePhase)));
        // The final-phase taps no longer resolve to a generated prim. When a
        // joint carries aim revisions its final frame is computed in memory
        // and overwrites these; when it does not, final IS base and these
        // resolve to the joint itself, which is what the empty resolution
        // already meant.
        newJointFinalFrameTaps.push_back(newTaps->Add(
            RigExecValueAddress::Prim(
                jointPath, _computePointFrame, finalPhase)));
        newJointFinalMatrixTaps.push_back(newTaps->Add(
            RigExecValueAddress::Prim(
                jointPath, TfToken("computeMatrix"), finalPhase)));
    }

    // Control frames, base phase only: a control is an input, so nothing in
    // the pose domain revises it and its base frame IS its posed frame.
    // Same request as the joints -- a control that cannot produce a frame
    // means the animator's own channel failed to evaluate, which is not a
    // condition to publish a generation under.
    std::vector<RigExecTapId> newControlFrameTaps;
    for (const SdfPath &controlPath : newControlPaths) {
        newControlFrameTaps.push_back(newTaps->Add(RigExecValueAddress::Prim(
            controlPath, _computePointFrame, basePhase)));
    }

    // Property-domain chains, from the same mover execution walk.
    //
    // Nothing to bind and nothing to tap: a math mover's inputs are all
    // authored on itself, and the chain's base is the target attribute's own
    // authored value. That is exactly what makes the chain evaluable BEFORE
    // exec runs, and therefore what lets its result be supplied to exec as a
    // value override -- which is how a clamped weight actually reaches the
    // solver that reads it instead of being reimplemented inside that
    // solver's kernel.
    std::map<SdfPath, std::vector<_PropertyRevision>> newPropertyChains;
    for (const RigExecMoverRecord &mover : newMovers) {
        if (mover.schemaType != "RigExecFloatMathMover" &&
            mover.schemaType != "RigExecVec3fMathMover" &&
            mover.schemaType != "RigExecMatrixMathMover") {
            continue;
        }
        // Validation above guarantees exactly one exact property target of
        // the matching type.
        for (const SdfPath &target : mover.targets) {
            newPropertyChains[target].push_back(
                _PropertyRevision{mover.moverPath, mover.schemaType});
        }
    }

    // A property chain can revise an input consumed by another property
    // chain. Resolve those producers first; namespace/map order is unrelated
    // to dataflow and made `/Consumer` read its authored envelope before a
    // lexically later `/Driver` had produced the revised one.
    std::vector<SdfPath> newPropertyChainOrder;
    {
        std::map<SdfPath, std::set<SdfPath>> dependsOn;
        for (const auto &[target, _] : newPropertyChains) {
            dependsOn[target];
        }

        auto addAttributeDependency = [&](const SdfPath &consumer,
                                          const UsdAttribute &attribute) {
            std::set<SdfPath> visited;
            std::function<void(const UsdAttribute &)> walk =
                [&](const UsdAttribute &a) {
                if (!a || !visited.insert(a.GetPath()).second) {
                    return;
                }
                if (newPropertyChains.count(a.GetPath())) {
                    dependsOn[consumer].insert(a.GetPath());
                }
                SdfPathVector connections;
                a.GetConnections(&connections);
                for (const SdfPath &sourcePath : connections) {
                    walk(_stage->GetAttributeAtPath(sourcePath));
                }
            };
            walk(attribute);
        };
        auto addPrimDependencies = [&](const SdfPath &consumer,
                                       const UsdPrim &prim) {
            if (!prim) {
                return;
            }
            for (const UsdAttribute &attribute : prim.GetAttributes()) {
                addAttributeDependency(consumer, attribute);
            }
        };
        std::function<void(const SdfPath &, const SdfPath &,
                           std::set<SdfPath> *)>
            addWeightDependencies =
                [&](const SdfPath &consumer, const SdfPath &weightPath,
                    std::set<SdfPath> *visited) {
                if (!visited->insert(weightPath).second) {
                    return;
                }
                const UsdPrim weight = _stage->GetPrimAtPath(weightPath);
                addPrimDependencies(consumer, weight);
                for (const char *relationship :
                     {"rigExec:inputWeights", "rigExec:baseWeight"}) {
                    SdfPathVector inputs;
                    if (const UsdRelationship rel =
                            weight.GetRelationship(TfToken(relationship))) {
                        rel.GetTargets(&inputs);
                    }
                    for (const SdfPath &input : inputs) {
                        addWeightDependencies(consumer, input, visited);
                    }
                }
            };

        for (const auto &[target, revisions] : newPropertyChains) {
            for (const _PropertyRevision &revision : revisions) {
                const UsdPrim mover =
                    _stage->GetPrimAtPath(revision.moverPath);
                addPrimDependencies(target, mover);
                SdfPathVector weights;
                if (const UsdRelationship rel = mover.GetRelationship(
                        TfToken("rigExec:weightObject"))) {
                    rel.GetTargets(&weights);
                }
                std::set<SdfPath> visitedWeights;
                for (const SdfPath &weight : weights) {
                    addWeightDependencies(target, weight, &visitedWeights);
                }
            }
        }

        std::map<SdfPath, int> colour;
        std::vector<SdfPath> stack;
        std::function<bool(const SdfPath &)> visit =
            [&](const SdfPath &target) {
            colour[target] = 1;
            stack.push_back(target);
            for (const SdfPath &producer : dependsOn[target]) {
                if (colour[producer] == 1) {
                    std::string cycle;
                    for (const SdfPath &path : stack) {
                        cycle += path.GetString() + " -> ";
                    }
                    cycle += producer.GetString();
                    reportError("property input dependency cycle: " + cycle);
                    return false;
                }
                if (colour[producer] == 0 && !visit(producer)) {
                    return false;
                }
            }
            stack.pop_back();
            colour[target] = 2;
            newPropertyChainOrder.push_back(target);
            return true;
        };
        for (const auto &[target, _] : dependsOn) {
            if (colour[target] == 0 && !visit(target)) {
                restorePreviousEpoch();
                return false;
            }
        }
    }

    // The compiled mover graph, built from the same mover execution walk
    // the generated prims come from. It runs alongside them for now: Evaluate
    // compares the two and diagnoses any disagreement, so the graph can be
    // proven equal before it becomes what publishes (see
    // docs/mover-graph-cutover.md). Bindings resolve off the authored stage
    // and author nothing.
    std::map<SdfPath, std::vector<_GraphRevision>> newGraphChains;
    for (const RigExecMoverRecord &mover : newMovers) {
        const UsdPrim moverPrim = _stage->GetPrimAtPath(mover.moverPath);
        if (!moverPrim) {
            continue;
        }
        TfToken curveMode;
        if (const UsdAttribute a =
                moverPrim.GetAttribute(TfToken("rigExec:mode"))) {
            a.Get(&curveMode);
        }
        const std::optional<RigExecRevisionOp> op =
            RigExecRevisionOpForSchema(mover.schemaType, curveMode);
        if (!op) {
            continue;
        }
        for (const SdfPath &target : mover.targets) {
            if (!target.IsPropertyPath() ||
                target.GetNameToken() != "points") {
                continue;
            }
            _GraphRevision revision;
            revision.moverPath = mover.moverPath;
            revision.target = target;
            revision.op = *op;
            revision.binding =
                RigExecResolveRevisionBinding(moverPrim, target, {});
            TfToken phase("base");
            if (const UsdAttribute a = moverPrim.GetAttribute(
                    TfToken("rigExec:transformReadPhase"))) {
                a.Get(&phase);
            }
            revision.transformFinalPhase = phase == "final";
            if (!revision.binding.transform.IsEmpty()) {
                revision.transformTap = newTaps->Add(
                    RigExecValueAddress::Prim(revision.binding.transform,
                                              TfToken("computeMatrix")));
            }
            if (!revision.binding.weightObject.IsEmpty()) {
                revision.weightTap = newTaps->Add(RigExecValueAddress::Prim(
                    revision.binding.weightObject,
                    TfToken("computeWeightPacket")));
                // Volumetric weights need two things exec cannot supply
                // on its own: a baked falloff table (no spline accessor
                // exists -- see RigExecFalloffLut) and, for the CPU
                // oracle, their resolved placement. Both are gathered
                // once here, following composition into combines.
                registerVolumeWeights(revision.binding.weightObject);
            }
            if (!revision.binding.driverFrames.IsEmpty()) {
                revision.driverFramesTap =
                    newTaps->Add(RigExecValueAddress::Prim(
                        revision.binding.driverFrames,
                        _computePointFrameArray));
            }
            // binding.blendInputs is already in canonical sorted order, which
            // is the order the accumulation is defined in.
            for (const SdfPath &input : revision.binding.blendInputs) {
                revision.blendChannelTaps.push_back(
                    newTaps->Add(RigExecValueAddress::Prim(
                        input, TfToken("computeBlendChannel"))));
            }
            newGraphChains[target].push_back(revision);
        }
    }

    // Derived maintenance (spec §7.6 revised), mirroring Pass 3: for every
    // moved points target whose gprim authors normals or extent, synthesize
    // the recompute revision. There is no authored mover, so the gprim itself
    // stands in as the parameter source -- it supplies the stage for the
    // static topology reads and has no inputs:enabled, so the revision is
    // enabled. Vertex-normal recomputation is mesh-only; the compiler already
    // rejects authored normals on a non-mesh points target.
    std::map<SdfPath, std::vector<_GraphRevision>> newGraphDerivedChains;
    for (const auto &[pointsTarget, revisions] : newGraphChains) {
        const SdfPath ownerPath = pointsTarget.GetPrimPath();
        const UsdPrim owner = _stage->GetPrimAtPath(ownerPath);
        if (!owner) {
            continue;
        }
        for (const bool isNormals : {true, false}) {
            const TfToken property(isNormals ? "normals" : "extent");
            const SdfPath derivedTarget =
                ownerPath.AppendProperty(property);
            const UsdAttribute authored =
                _stage->GetAttributeAtPath(derivedTarget);
            if (!authored || !authored.HasAuthoredValue()) {
                continue;
            }
            // Vertex-normal recomputation is mesh-only. Silently leaving
            // authored normals stale on a moved Points/BasisCurves target
            // would break the automatic-maintenance contract (spec §7.6
            // revised), so reject the configuration rather than skip it. This
            // check used to live in the compiler's Pass 3.
            if (isNormals && !UsdGeomMesh(owner)) {
                reportError(
                    "authored normals on non-mesh points target " +
                    ownerPath.GetString() +
                    " cannot be maintained (vertex-normal recomputation is "
                    "mesh-only, spec §7.6 revised); remove the authored "
                    "normals");
                return false;
            }
            _GraphRevision derived;
            derived.moverPath = ownerPath;
            derived.target = derivedTarget;
            derived.op = isNormals ? RigExecRevisionOp::RecomputeNormals
                                   : RigExecRevisionOp::RecomputeExtent;
            derived.binding.moverPath = ownerPath;
            derived.binding.target = derivedTarget;
            derived.binding.topologyCounts =
                ownerPath.AppendProperty(TfToken("faceVertexCounts"));
            derived.binding.topologyIndices =
                ownerPath.AppendProperty(TfToken("faceVertexIndices"));
            if (!isNormals) {
                // The authoritative winning widths, when authored: they
                // widen the extent bound (Pass 3 wires resolvedWidths).
                const SdfPath widthsPath =
                    ownerPath.AppendProperty(TfToken("widths"));
                if (const UsdAttribute w =
                        _stage->GetAttributeAtPath(widthsPath)) {
                    if (w.HasAuthoredValue()) {
                        derived.binding.widths = widthsPath;
                    }
                }
            }
            newGraphDerivedChains[pointsTarget].push_back(derived);
        }
    }

    if (!newTaps->Prepare()) {
        reportError("failed to build a valid prepared request for the "
                    "new epoch");
        restorePreviousEpoch();
        return false;
    }

    // Observational solver-guide taps prepare separately so a failing or
    // unused aggregate solver never gates the authoritative rig request;
    // preparation failure simply drops solver guide drawing.
    auto newGuideTaps = std::make_unique<RigExecTapSet>(_stage);
    for (const SdfPath &solverPath : solverArrayPaths) {
        newSolverArrayTaps[solverPath] = newGuideTaps->Add(
            RigExecValueAddress::Prim(solverPath, _computePointFrameArray));
    }
    if (newSolverArrayTaps.empty() || !newGuideTaps->Prepare()) {
        newGuideTaps.reset();
        newSolverArrayTaps.clear();
    }

    // Commit the new epoch atomically with respect to evaluator state.
    _movers = std::move(newMovers);
    _jointPaths = std::move(newJointPaths);
    _controlPaths = std::move(newControlPaths);
    _controlFrameTaps = std::move(newControlFrameTaps);
    _frameConstraints = std::move(newFrameConstraints);
    _frameChains = std::move(newFrameChains);
    _providerRestFrameTaps = std::move(newProviderRestFrameTaps);
    _providerBaseFrameTaps = std::move(newProviderBaseFrameTaps);
    _xformDerivedProviders = std::move(newXformDerivedProviders);
    _ribbonDriverPoints = std::move(newRibbonDriverPoints);
    if (!volumeWeightError.empty()) {
        reportError(volumeWeightError);
        restorePreviousEpoch();
        return false;
    }

    _falloffLutOverrides = std::move(newFalloffLutOverrides);
    _currentPhaseWeights = std::move(newCurrentPhaseWeights);
    _volumeWeightMatrixTaps = std::move(newVolumeWeightMatrixTaps);
    _volumeWeightMatrices.clear();
    _structureDigest = newDigest;
    _taps = std::move(newTaps);
    _guideTaps = std::move(newGuideTaps);
    _jointFrameTaps = std::move(newJointFrameTaps);
    _jointFinalFrameTaps = std::move(newJointFinalFrameTaps);
    _jointFinalMatrixTaps = std::move(newJointFinalMatrixTaps);
    _jointSolverBinding = std::move(newJointBinding);
    _jointSolverArrayTaps = std::move(newJointSolverArrayTaps);
    _solverFrameTaps = std::move(newSolverFrameTaps);
    _impliedIkLengths = std::move(newImplied);
    _impliedRestTaps = std::move(newImpliedRestTaps);
    _restFrameTaps = std::move(newRestTaps);
    _solverArrayTaps = std::move(newSolverArrayTaps);
    _graphChains = std::move(newGraphChains);
    _graphDerivedChains = std::move(newGraphDerivedChains);
    _propertyChains = std::move(newPropertyChains);
    _propertyChainOrder = std::move(newPropertyChainOrder);

    // Chain evaluation order.
    //
    // A chain that reads another chain's target at a non-base phase cannot
    // run until that chain has. Collect those edges and sort; a cycle is a
    // compile error, because there is no order that satisfies it and the
    // alternative -- picking one and reading a stale or authored value -- is
    // the silent-wrong-answer failure this engine refuses everywhere else.
    {
        std::map<SdfPath, std::set<SdfPath>> dependsOn;  // target -> producers
        for (const auto &[target, revisions] : _graphChains) {
            dependsOn[target];  // every chain is a node, even with no edges
        }
        for (const auto &chain : _graphChains) {
            const SdfPath &target = chain.first;
            const std::vector<_GraphRevision> &revisions = chain.second;
            for (const _GraphRevision &revision : revisions) {
                auto addEdge = [&](const SdfPath &producer) {
                    if (producer.IsEmpty() || producer == target ||
                        !_graphChains.count(producer)) {
                        return;
                    }
                    dependsOn[target].insert(producer);
                };
                for (const auto &[inputPath, phase] : revision.binding.phases) {
                    addEdge(inputPath);
                }
                // The Profile Mover's implicit dependency: its net's knots
                // are posed by ordinary movers and it must see them posed.
                // Stated as an edge now rather than as a pass ordering, so
                // one mechanism carries both kinds.
                addEdge(revision.binding.curvenetPoints);
            }
        }

        std::vector<SdfPath> order;
        order.reserve(dependsOn.size());
        std::set<SdfPath> emitted;
        // Kahn over a map: deterministic because the map iterates in path
        // order, so an unconstrained pair always comes out the same way.
        bool progress = true;
        while (progress && emitted.size() < dependsOn.size()) {
            progress = false;
            for (const auto &[target, producers] : dependsOn) {
                if (emitted.count(target)) {
                    continue;
                }
                bool ready = true;
                for (const SdfPath &producer : producers) {
                    if (!emitted.count(producer)) {
                        ready = false;
                        break;
                    }
                }
                if (ready) {
                    order.push_back(target);
                    emitted.insert(target);
                    progress = true;
                }
            }
        }
        if (emitted.size() != dependsOn.size()) {
            std::string cycle;
            for (const auto &[target, producers] : dependsOn) {
                if (!emitted.count(target)) {
                    if (!cycle.empty()) {
                        cycle += ", ";
                    }
                    cycle += target.GetString();
                }
            }
            reportError("Cyclic read-phase dependency between chains: " +
                        cycle + " (a phased read cannot be satisfied in any "
                        "evaluation order)");
            restorePreviousEpoch();
            return false;
        }
        _chainOrder = std::move(order);
    }

    // Validate every declared phase, and reduce it to the one revision it
    // names.
    {
        std::map<SdfPath, std::vector<SdfPath>> chainMovers;
        for (const auto &[target, revisions] : _graphChains) {
            for (const _GraphRevision &revision : revisions) {
                chainMovers[target].push_back(revision.moverPath);
            }
        }
        std::map<SdfPath, int> ordinalOf;
        for (const RigExecMoverRecord &m : _movers) {
            ordinalOf[m.moverPath] = m.ordinal;
        }

        _snapshotPoints.clear();
        for (const auto &[target, revisions] : _graphChains) {
            for (const _GraphRevision &revision : revisions) {
                for (const auto &[inputPath, phase] : revision.binding.phases) {
                    const std::string who =
                        revision.moverPath.GetString() + ": read phase '" +
                        phase.GetAsString() + "' on " + inputPath.GetString();

                    // A phase on an input nothing writes is a no-op that
                    // reads as intent. Reject it: the author asked for a
                    // revision of something that has none, and silently
                    // handing back the authored value is how a rig ends up
                    // deforming against the wrong pose with no signal.
                    const auto moversIt = chainMovers.find(inputPath);
                    if (moversIt == chainMovers.end()) {
                        reportError(who + " names a property no mover writes; "
                                          "only `base` is meaningful there");
                        restorePreviousEpoch();
                        return false;
                    }
                    const std::vector<SdfPath> &movers = moversIt->second;

                    if (phase.kind == RigExecReadPhaseKind::Final) {
                        continue;  // the chain's published result
                    }
                    if (phase.kind == RigExecReadPhaseKind::Preceding) {
                        // Only meaningful when the reader is itself in that
                        // chain; otherwise there is no position to precede.
                        const auto at = std::find(movers.begin(), movers.end(),
                                                  revision.moverPath);
                        if (at == movers.end()) {
                            reportError(
                                who + " is `preceding`, but " +
                                revision.moverPath.GetString() +
                                " does not write " + inputPath.GetString() +
                                "; there is no preceding revision to name");
                            restorePreviousEpoch();
                            return false;
                        }
                        if (at != movers.begin()) {
                            _snapshotPoints[inputPath].insert(*(at - 1));
                        }
                        continue;
                    }

                    // AtPrim: the last revision at or beneath the named prim.
                    SdfPath found;
                    for (const SdfPath &mover : movers) {
                        if (mover == phase.prim || mover.HasPrefix(phase.prim)) {
                            found = mover;
                        }
                    }
                    if (found.IsEmpty()) {
                        reportError(who + " names " + phase.prim.GetString() +
                                    ", which writes nothing to " +
                                    inputPath.GetString());
                        restorePreviousEpoch();
                        return false;
                    }
                    // Within one chain the named revision must already have
                    // run when the reader runs. Across chains the topological
                    // order above guarantees it, so only the self-read case
                    // can be unsatisfiable.
                    if (inputPath == target) {
                        const auto namedOrdinal = ordinalOf.find(found);
                        const auto readerOrdinal =
                            ordinalOf.find(revision.moverPath);
                        if (namedOrdinal != ordinalOf.end() &&
                            readerOrdinal != ordinalOf.end() &&
                            namedOrdinal->second >= readerOrdinal->second) {
                            reportError(
                                who + " names " + found.GetString() +
                                ", which runs at or after the reader in the "
                                "composed walk (spec §4.2)");
                            restorePreviousEpoch();
                            return false;
                        }
                    }
                    _snapshotPoints[inputPath].insert(found);
                }
            }
        }

        // The transform provider's phase is answered from the FRAME chains,
        // so it validates against those rather than against chainMovers.
        for (const auto &[target, revisions] : _graphChains) {
            for (const _GraphRevision &revision : revisions) {
                const RigExecReadPhase &phase = revision.binding.transformPhase;
                if (phase.kind != RigExecReadPhaseKind::AtPrim) {
                    continue;
                }
                const std::string who =
                    revision.moverPath.GetString() + ": read phase '" +
                    phase.GetAsString() + "' on rigExec:transform";
                const auto frameIt =
                    _frameChains.find(revision.binding.transform);
                if (frameIt == _frameChains.end()) {
                    reportError(who + " names a point in the pose walk, but " +
                                revision.binding.transform.GetString() +
                                " is revised by no pose mover");
                    restorePreviousEpoch();
                    return false;
                }
                SdfPath found;
                for (const SdfPath &frameMover : frameIt->second) {
                    if (frameMover == phase.prim ||
                        frameMover.HasPrefix(phase.prim)) {
                        found = frameMover;
                    }
                }
                if (found.IsEmpty()) {
                    reportError(who + " names " + phase.prim.GetString() +
                                ", which revises nothing on " +
                                revision.binding.transform.GetString());
                    restorePreviousEpoch();
                    return false;
                }
                _snapshotPoints[revision.binding.transform].insert(found);
            }
        }
    }

    _compiled = true;
    return true;
}

bool
RigExecRigEvaluator::_ValidateMatrixMover(
    const UsdPrim &prim,
    const RigExecMoverRecord &record,
    std::string *error) const
{
    const std::string who = "MatrixMover " + prim.GetPath().GetString();
    if (record.targets.size() != 1 ||
        !record.targets[0].IsPropertyPath() ||
        record.targets[0].GetNameToken() != "points") {
        *error = who + ": moves must resolve to exactly one native "
                       "PointBased points property" +
                 (record.targets.size() == 1
                      ? _PointsTargetHint(_stage, record.targets[0])
                      : std::string());
        return false;
    }
    const UsdPrim owner =
        _stage->GetPrimAtPath(record.targets[0].GetPrimPath());
    if (!owner || !owner.IsA<UsdGeomPointBased>()) {
        *error = who + ": move target owner is not a stock PointBased prim";
        return false;
    }
    // Exact Sdf type/role check: equal C++ element types never infer
    // compatibility (spec §7.2).
    const UsdAttribute targetAttr =
        _stage->GetAttributeAtPath(record.targets[0]);
    if (!targetAttr ||
        targetAttr.GetTypeName() != SdfValueTypeNames->Point3fArray) {
        *error = who + ": move target is not an exact point3f[] property";
        return false;
    }

    SdfPathVector transforms;
    if (UsdRelationship rel =
            prim.GetRelationship(TfToken("rigExec:transform"))) {
        rel.GetTargets(&transforms);
    }
    if (transforms.size() != 1) {
        *error = who + ": rigExec:transform must have exactly one target";
        return false;
    }
    // preceding is legal only for a dependency specialized to one
    // consuming application ordinal (spec §4.2); the v0.1 compiler
    // supports base and acyclic final.
    TfToken phase("base");
    if (UsdAttribute a =
            prim.GetAttribute(TfToken("rigExec:transformReadPhase"))) {
        a.Get(&phase);
    }
    if (phase != "base" && phase != "final") {
        *error = who + ": unsupported transformReadPhase '" +
                 phase.GetString() + "' (v0.1 supports base and final)";
        return false;
    }
    // The transform target must be a catalogued computeMatrix provider.
    const UsdPrim transformPrim = _stage->GetPrimAtPath(transforms[0]);
    static const std::set<TfToken> frameProviderTypes = {
        TfToken("RigExecControl"), TfToken("RigExecJoint")};
    // The applied-API arm is gone with RigExecPointTransformAPI: that was the
    // pre-alignment landmark transform model, superseded by RigExecXformable
    // (matrix rest/posed spaces plus avars) and applied by nothing.
    const bool isProvider =
        transformPrim && frameProviderTypes.count(transformPrim.GetTypeName());
    if (!isProvider) {
        *error = who + ": rigExec:transform target is not a catalogued "
                       "matrix provider";
        return false;
    }
    return true;
}

bool
RigExecRigEvaluator::_ReadTargetPoints(
    const UsdPrim &prim, const char *relationshipName, UsdTimeCode time,
    std::vector<GfVec3f> *points) const
{
    points->clear();
    SdfPathVector targets;
    if (UsdRelationship rel = prim.GetRelationship(TfToken(relationshipName))) {
        rel.GetTargets(&targets);
    }
    if (targets.size() != 1) {
        return false;
    }
    const SdfPath canonical = _ResolveGeometryInput(_stage, targets[0]);
    const UsdAttribute attr = _stage->GetAttributeAtPath(canonical);
    VtVec3fArray value;
    if (!attr || !attr.Get(&value, time)) {
        return false;
    }
    points->assign(value.begin(), value.end());
    return true;
}

bool
RigExecRigEvaluator::_ResolveVolumeWeights(
    const UsdPrim &prim, size_t count, UsdTimeCode time,
    std::vector<float> *weights, std::string *error,
    const std::vector<GfVec3f> *currentPoints) const
{
    const std::string who = prim.GetPath().GetString();
    const TfToken typeName = prim.GetTypeName();

    // The composed field folds its inputs; it measures nothing itself.
    if (typeName == "RigExecCombineWeight") {
        SdfPathVector inputs;
        if (UsdRelationship rel =
                prim.GetRelationship(TfToken("rigExec:inputWeights"))) {
            rel.GetTargets(&inputs);
        }
        TfToken modeName("multiply");
        if (UsdAttribute a =
                prim.GetAttribute(TfToken("rigExec:combineMode"))) {
            a.Get(&modeName, time);
        }
        RigExecWeightCombine mode;
        if (modeName == "multiply") {
            mode = RigExecWeightCombine::Multiply;
        } else if (modeName == "add") {
            mode = RigExecWeightCombine::Add;
        } else if (modeName == "subtract") {
            mode = RigExecWeightCombine::Subtract;
        } else if (modeName == "max") {
            mode = RigExecWeightCombine::Max;
        } else if (modeName == "min") {
            mode = RigExecWeightCombine::Min;
        } else if (modeName == "average") {
            mode = RigExecWeightCombine::Average;
        } else if (modeName == "overlay") {
            mode = RigExecWeightCombine::Overlay;
        } else {
            *error = who + ": unknown rigExec:combineMode " +
                     modeName.GetString();
            return false;
        }

        // Authored order, unsorted: subtract and overlay are order
        // dependent by design (see the schema doc).
        std::vector<std::vector<float>> fields;
        fields.reserve(inputs.size());
        for (const SdfPath &input : inputs) {
            std::vector<float> field;
            if (!_ResolveWeights(input, count, time, &field, error,
                                 currentPoints)) {
                return false;
            }
            fields.push_back(std::move(field));
        }
        if (!RigExecCombineWeightFields(mode, fields, count, weights)) {
            *error = who + ": combine inputs disagree on element count";
            return false;
        }
        const float strength = _ResolvedRead(
            _resolvedInputs, prim, "inputs:strength", 1.0f, time);
        const float invert = _ResolvedRead(
            _resolvedInputs, prim, "inputs:invert", 0.0f, time);
        for (float &w : *weights) {
            w = (w + (1.0f - 2.0f * w) * invert) * strength;
        }
        return true;
    }

    // Placement.
    //
    // Taken from the volume's own exec computeMatrix rather than
    // recomputed here. The oracle exists to check the WEIGHT FIELD math
    // independently, not the xformable frame chain -- that already has
    // its own parity coverage, and a second hand-rolled implementation
    // of posed:space + rest offsets + avars + rotation order is exactly
    // the drift frameExtraction.h was created to prevent.
    const auto matrixIt = _volumeWeightMatrices.find(prim.GetPath());
    if (matrixIt == _volumeWeightMatrices.end()) {
        *error = who + ": no resolved placement for this volume weight";
        return false;
    }
    // Scale and shear are removed so the field matches the rigid guide a
    // viewer draws; inputs:scaleX/Y/Z is the sole authority on
    // anisotropy (see the RigExecVolumeWeight schema doc).
    GfMatrix4d rigid = matrixIt->second.RemoveScaleShear();
    const double det = rigid.GetDeterminant();
    if (!std::isfinite(det) || std::abs(det) < 1e-12) {
        *error = who + ": degenerate volume placement";
        return false;
    }
    GfMatrix4d worldToLocal = rigid.GetInverse();

    // Which points the distance function measures.
    std::vector<GfVec3f> samplePoints;
    TfToken samplePhase("reference");
    if (UsdAttribute a = prim.GetAttribute(_samplePhaseAttr)) {
        a.Get(&samplePhase, time);
    }
    if (samplePhase == "current") {
        if (!currentPoints) {
            *error = who +
                     ": rigExec:samplePhase is `current` but no in-flight "
                     "points were supplied";
            return false;
        }
        samplePoints = *currentPoints;
    } else if (samplePhase == "reference") {
        // An explicit sampleSource wins over the weighted domain, which
        // is how one mesh is weighted by another mesh's shape.
        if (!_ReadTargetPoints(prim, "rigExec:sampleSource", time,
                               &samplePoints) &&
            !_ReadTargetPoints(prim, "rigExec:weightTarget", time,
                               &samplePoints)) {
            *error = who + ": could not read the points to sample";
            return false;
        }
    } else {
        *error = who + ": unknown rigExec:samplePhase " +
                 samplePhase.GetString();
        return false;
    }
    if (samplePoints.size() != count) {
        *error = who + ": sampled point count does not match the target";
        return false;
    }

    RigExecFalloffParams params;
    auto readFloat = [this, &prim, time](const char *name, float fallback) {
        return _ResolvedRead(
            _resolvedInputs, prim, name, fallback, time);
    };
    params.falloffMin = readFloat("inputs:falloffMin", 0.0f);
    params.falloffMax = readFloat("inputs:falloffMax", 1.0f);
    params.invert = readFloat("inputs:invert", 0.0f);
    params.strength = readFloat("inputs:strength", 1.0f);
    params.curve = _BakeFalloffLut(prim);

    if (typeName == "RigExecPlaneWeight") {
        TfToken axis("y");
        if (UsdAttribute a = prim.GetAttribute(TfToken("rigExec:planeAxis"))) {
            a.Get(&axis, time);
        }
        const int axisIndex =
            axis == "x" ? 0 : (axis == "y" ? 1 : (axis == "z" ? 2 : -1));
        if (axisIndex < 0) {
            *error = who + ": unknown rigExec:planeAxis " + axis.GetString();
            return false;
        }
        // Bounded clips the field to the in-plane rectangle. Mirrors
        // _BuildPlaneWeightPacket exactly, including reading the extents
        // only in the bounded arm -- the two paths have to agree value
        // for value or the parity harness fires.
        TfToken boundsMode("unbounded");
        if (UsdAttribute a =
                prim.GetAttribute(TfToken("rigExec:planeBounds"))) {
            a.Get(&boundsMode, time);
        }
        RigExecPlaneBounds extent;
        const RigExecPlaneBounds *extentPtr = nullptr;
        if (boundsMode == "bounded") {
            extent.extentU = readFloat("inputs:extentU", 1.0f);
            extent.extentV = readFloat("inputs:extentV", 1.0f);
            for (const float e : {extent.extentU, extent.extentV}) {
                if (!std::isfinite(e) || e <= 0.0f) {
                    *error = who +
                             ": inputs:extentU/V must be finite and positive "
                             "when rigExec:planeBounds is `bounded`";
                    return false;
                }
            }
            extentPtr = &extent;
        } else if (boundsMode != "unbounded") {
            *error = who + ": unknown rigExec:planeBounds " +
                     boundsMode.GetString();
            return false;
        }
        RigExecPlaneWeightField(
            samplePoints, worldToLocal, axisIndex, params, weights, extentPtr);
        return true;
    }

    // Sphere and curve both take the per-axis divisors, folded into the
    // matrix so the hot loop stays one transform.
    const float sx = readFloat("inputs:scaleX", 1.0f);
    const float sy = readFloat("inputs:scaleY", 1.0f);
    const float sz = readFloat("inputs:scaleZ", 1.0f);
    for (float s : {sx, sy, sz}) {
        if (!std::isfinite(s) || s <= 0.0f) {
            *error = who + ": inputs:scaleX/Y/Z must be finite and positive";
            return false;
        }
    }
    GfMatrix4d divide(1.0);
    divide.SetScale(GfVec3d(1.0 / double(sx), 1.0 / double(sy),
                            1.0 / double(sz)));
    worldToLocal = worldToLocal * divide;

    if (typeName == "RigExecSphereWeight") {
        RigExecSphereWeightField(samplePoints, worldToLocal, params, weights);
        return true;
    }
    if (typeName == "RigExecCurveWeight") {
        std::vector<GfVec3f> curvePoints;
        if (!_ReadTargetPoints(prim, "rigExec:curve", time, &curvePoints) ||
            curvePoints.empty()) {
            *error = who + ": rigExec:curve must name exactly one points source";
            return false;
        }
        RigExecCurveWeightField(
            samplePoints, curvePoints, worldToLocal, params, weights);
        return true;
    }
    *error = who + ": not a volumetric weight object";
    return false;
}

bool
RigExecRigEvaluator::_ResolveWeights(
    const SdfPath &weightPrimPath, size_t count, UsdTimeCode time,
    std::vector<float> *weights, std::string *error,
    const std::vector<GfVec3f> *currentPoints) const
{
    weights->assign(count, 1.0f);
    const UsdPrim prim = _stage->GetPrimAtPath(weightPrimPath);
    if (!prim) {
        *error = "missing weight object " + weightPrimPath.GetString();
        return false;
    }

    // A type this oracle does not understand must FAIL, never fall
    // through to the authored-table path. That path reads no
    // representation and no defaultWeight off a volumetric prim and so
    // returns an all-zero field and `true` -- a silently wrong answer,
    // and the one shape of bug the parity harness cannot catch because
    // both sides would agree on nothing.
    const TfToken typeName = prim.GetTypeName();
    if (!_IsWeightObjectType(typeName)) {
        *error = "unknown weight object type " + typeName.GetString() +
                 " on " + weightPrimPath.GetString();
        return false;
    }
    if (_IsVolumeWeightType(typeName) || typeName == "RigExecCombineWeight") {
        std::vector<float> resolved;
        if (!_ResolveVolumeWeights(prim, count, time, &resolved, error,
                                   currentPoints)) {
            return false;
        }
        TfToken volumePolicy("clamp");
        if (UsdAttribute a =
                prim.GetAttribute(TfToken("rigExec:rangePolicy"))) {
            a.Get(&volumePolicy, time);
        }
        for (float &w : resolved) {
            if (!std::isfinite(w)) {
                *error = "non-finite weight on " + weightPrimPath.GetString();
                return false;
            }
            if (w < 0.0f || w > 1.0f) {
                if (volumePolicy != "clamp") {
                    *error = "strict range violation on " +
                             weightPrimPath.GetString();
                    return false;
                }
                w = std::min(std::max(w, 0.0f), 1.0f);
            }
        }
        *weights = std::move(resolved);
        return true;
    }

    TfToken representation("constant");
    if (UsdAttribute a = prim.GetAttribute(
            TfToken("rigExec:representation"))) {
        a.Get(&representation, time);
    }
    const float defaultWeight = _ResolvedRead(
        _resolvedInputs, prim, "rigExec:defaultWeight", 0.0f, time);
    TfToken rangePolicy("strict");
    if (UsdAttribute a = prim.GetAttribute(TfToken("rigExec:rangePolicy"))) {
        a.Get(&rangePolicy, time);
    }
    if (rangePolicy != "strict" && rangePolicy != "clamp") {
        *error = "unknown rangePolicy on " + weightPrimPath.GetString();
        return false;
    }

    const bool isDynamic = prim.GetTypeName() == "RigExecDynamicWeight";
    if (isDynamic) {
        TfToken operation("multiply");
        if (UsdAttribute a =
                prim.GetAttribute(TfToken("rigExec:operation"))) {
            a.Get(&operation, time);
        }
        if (operation != "multiply") {
            *error = "unknown dynamic-weight operation on " +
                     weightPrimPath.GetString();
            return false;
        }

        // Base field first, then r_i = (b_i * d) * s + a (spec §4.1).
        std::vector<float> base(count, 1.0f);
        SdfPathVector baseTargets;
        if (UsdRelationship rel =
                prim.GetRelationship(TfToken("rigExec:baseWeight"))) {
            rel.GetTargets(&baseTargets);
        }
        if (baseTargets.size() > 1) {
            *error = "rigExec:baseWeight must have at most one target on " +
                     weightPrimPath.GetString();
            return false;
        }
        if (baseTargets.empty()) {
            // Without a base, only constant representation is legal and
            // b_i = 1 everywhere (spec §4.1).
            if (representation != "constant") {
                *error = "no-base dynamic weight must be constant on " +
                         weightPrimPath.GetString();
                return false;
            }
        } else {
            const UsdPrim basePrim = _stage->GetPrimAtPath(baseTargets[0]);
            if (!basePrim) {
                *error = "missing base weight object on " +
                         weightPrimPath.GetString();
                return false;
            }
            // The base descriptor must exactly match the dynamic
            // descriptor: canonical target, representation, and sparse
            // support (spec §4.1).
            auto canonicalWeightTarget =
                [this, &time](const UsdPrim &p) -> SdfPath {
                SdfPathVector t;
                if (UsdRelationship rel = p.GetRelationship(
                        TfToken("rigExec:weightTarget"))) {
                    rel.GetTargets(&t);
                }
                return t.size() == 1 ? _ResolveGeometryInput(_stage, t[0])
                                     : SdfPath();
            };
            if (canonicalWeightTarget(prim) !=
                    canonicalWeightTarget(basePrim) ||
                canonicalWeightTarget(prim).IsEmpty()) {
                *error = "dynamic/base weight target mismatch on " +
                         weightPrimPath.GetString();
                return false;
            }
            TfToken baseRepresentation("constant");
            if (UsdAttribute a = basePrim.GetAttribute(
                    TfToken("rigExec:representation"))) {
                a.Get(&baseRepresentation, time);
            }
            if (baseRepresentation != representation) {
                *error = "dynamic/base representation mismatch on " +
                         weightPrimPath.GetString();
                return false;
            }
            if (representation == "sparse") {
                // The dynamic descriptor's sparse support is inherited
                // from the base; a dynamic prim that authors its own
                // support must match the base exactly (spec §4.1).
                VtIntArray mine;
                if (UsdAttribute a =
                        prim.GetAttribute(TfToken("rigExec:indices"))) {
                    a.Get(&mine, time);
                }
                if (!mine.empty()) {
                    VtIntArray theirs;
                    if (UsdAttribute a = basePrim.GetAttribute(
                            TfToken("rigExec:indices"))) {
                        a.Get(&theirs, time);
                    }
                    const std::set<int> mySupport(mine.begin(), mine.end());
                    const std::set<int> baseSupport(
                        theirs.begin(), theirs.end());
                    if (mySupport != baseSupport) {
                        *error = "dynamic/base sparse support mismatch on " +
                                 weightPrimPath.GetString();
                        return false;
                    }
                }
            }
            // currentPoints is forwarded: a dynamic weight modulating a
            // current-phase volume must still measure against the
            // in-flight points, or the base silently reverts to the
            // reference field.
            if (!_ResolveWeights(baseTargets[0], count, time, &base, error,
                                 currentPoints)) {
                return false;
            }
        }
        const float driver = _ResolvedRead(
            _resolvedInputs, prim, "inputs:driver", 1.0f, time);
        const float scale = _ResolvedRead(
            _resolvedInputs, prim, "inputs:scale", 1.0f, time);
        const float bias = _ResolvedRead(
            _resolvedInputs, prim, "inputs:bias", 0.0f, time);
        for (size_t i = 0; i < count; ++i) {
            float r = (base[i] * driver) * scale + bias;
            if (!std::isfinite(r)) {
                *error = "non-finite dynamic weight on " +
                         weightPrimPath.GetString();
                return false;
            }
            if (r < 0.0f || r > 1.0f) {
                if (rangePolicy == "clamp") {
                    r = std::min(std::max(r, 0.0f), 1.0f);
                } else {
                    *error = "strict range violation on " +
                             weightPrimPath.GetString();
                    return false;
                }
            }
            (*weights)[i] = r;
        }
        return true;
    }

    // Static weights are time-invariant by contract: reject time samples
    // and value connections on every field (spec §4.1).
    static const TfToken staticFields[] = {
        TfToken("rigExec:values"), TfToken("rigExec:indices"),
        TfToken("rigExec:defaultWeight"), TfToken("rigExec:representation"),
        TfToken("rigExec:rangePolicy")};
    for (const TfToken &field : staticFields) {
        const UsdAttribute a = prim.GetAttribute(field);
        if (a && (a.GetNumTimeSamples() > 0 || a.HasAuthoredConnections())) {
            *error = "static weight field " + field.GetString() +
                     " has time samples or connections on " +
                     weightPrimPath.GetString();
            return false;
        }
    }
    VtFloatArray values;
    if (UsdAttribute a = prim.GetAttribute(TfToken("rigExec:values"))) {
        a.Get(&values, time);
    }
    if (representation == "constant") {
        if (!values.empty()) {
            *error = "constant weight must not author values on " +
                     weightPrimPath.GetString();
            return false;
        }
        weights->assign(count, defaultWeight);
    } else if (representation == "dense") {
        if (values.size() != count) {
            *error = "dense weight cardinality mismatch on " +
                     weightPrimPath.GetString();
            return false;
        }
        // Canonical encoding requires dense defaultWeight = 0 (spec §4.1).
        if (defaultWeight != 0.0f) {
            *error = "dense weight requires canonical defaultWeight 0 on " +
                     weightPrimPath.GetString();
            return false;
        }
        weights->assign(values.begin(), values.end());
    } else if (representation == "sparse") {
        VtIntArray indices;
        if (UsdAttribute a = prim.GetAttribute(TfToken("rigExec:indices"))) {
            a.Get(&indices, time);
        }
        if (indices.size() != values.size()) {
            *error = "sparse index/value size mismatch on " +
                     weightPrimPath.GetString();
            return false;
        }
        weights->assign(count, defaultWeight);
        std::set<int> seen;
        for (size_t i = 0; i < indices.size(); ++i) {
            if (indices[i] < 0 || static_cast<size_t>(indices[i]) >= count) {
                *error = "sparse index out of range on " +
                         weightPrimPath.GetString();
                return false;
            }
            // Indices are unique logical element indices; authored pair
            // order is non-semantic (spec §4.1).
            if (!seen.insert(indices[i]).second) {
                *error = "duplicate sparse index on " +
                         weightPrimPath.GetString();
                return false;
            }
            (*weights)[indices[i]] = values[i];
        }
    } else {
        *error = "unknown weight representation on " +
                 weightPrimPath.GetString();
        return false;
    }

    for (float w : *weights) {
        if (!std::isfinite(w) ||
            (rangePolicy == "strict" && (w < 0.0f || w > 1.0f))) {
            *error = "weight range violation on " +
                     weightPrimPath.GetString();
            return false;
        }
    }
    if (rangePolicy == "clamp") {
        for (float &w : *weights) {
            w = std::min(std::max(w, 0.0f), 1.0f);
        }
    }
    return true;
}

namespace {

bool
_IsEnabled(const UsdPrim &mover, UsdTimeCode time)
{
    bool enabled = true;
    if (UsdAttribute a = mover.GetAttribute(_enabledAttr)) {
        a.Get(&enabled, time);
    }
    return enabled;
}

}  // namespace

VtVec3fArray
RigExecRigEvaluator::_EvaluateChain(
    const SdfPath &target,
    const std::vector<const RigExecMoverRecord *> &chain,
    const RigExecRigPose &pose,
    const std::map<SdfPath, GfMatrix4d> &baseProviderMatrices,
    const std::map<SdfPath, GfMatrix4d> &finalProviderMatrices,
    UsdTimeCode time,
    std::vector<std::string> *diagnostics) const
{
    // The oracle resolves a read phase ITSELF, from the authored metadata,
    // and reads the same recorded snapshots. That keeps it independent of
    // RigExecResolveRevisionBinding and RigExecAssembleParameters -- which is
    // what makes parity a real check -- while sharing the authored INTENT,
    // which it must, or the two paths are evaluating different rigs.
    auto phasedPoints = [this, time](
        const UsdPrim &prim, const char *relName, const char *legacyAttr,
        const SdfPath &pointsPath, const SdfPath &readerMover,
        VtVec3fArray *out) {
        RigExecReadPhase phase;
        if (const UsdRelationship r = prim.GetRelationship(TfToken(relName))) {
            RigExecResolveReadPhase(r, legacyAttr, &phase, nullptr);
        }
        if (!phase.IsBase()) {
            if (const VtValue *v = _chainSnapshots.Lookup(
                    pointsPath, phase, readerMover)) {
                if (v->IsHolding<VtVec3fArray>()) {
                    *out = v->UncheckedGet<VtVec3fArray>();
                    return;
                }
            }
        }
        if (const UsdAttribute a = _stage->GetAttributeAtPath(pointsPath)) {
            a.Get(out, time);
        }
    };

    // Base: the stock resolved value of the exact native property
    // (spec §7.2). The base revision is retained: blend-shape deltas
    // derive against base points, not the preceding revision (spec §7.3).
    VtVec3fArray points;
    const UsdAttribute baseAttr = _stage->GetAttributeAtPath(target);
    if (!baseAttr || !baseAttr.Get(&points, time)) {
        diagnostics->push_back("no base value for " + target.GetString());
        return points;
    }
    const VtVec3fArray basePoints = points;

    for (const RigExecMoverRecord *mover : chain) {
        const UsdPrim prim = _stage->GetPrimAtPath(mover->moverPath);
        if (!prim || !_IsEnabled(prim, time)) {
            continue;  // pass-through (spec §4.2)
        }
        const std::string type = mover->schemaType.GetString();

        // Resolve the common envelope against the PRECEDING revision. A
        // current-phase volume therefore measures exactly the points that
        // enter this mover. A bound object supersedes inputs:defaultWeight.
        const VtVec3fArray preceding = points;
        std::vector<float> envelope(points.size(), 1.0f);
        SdfPathVector weightObjects;
        if (const UsdRelationship rel =
                prim.GetRelationship(TfToken("rigExec:weightObject"))) {
            rel.GetTargets(&weightObjects);
        }
        if (!weightObjects.empty()) {
            const std::vector<GfVec3f> currentPoints(
                preceding.begin(), preceding.end());
            std::string error;
            if (!_ResolveWeights(weightObjects[0], points.size(), time,
                                 &envelope, &error, &currentPoints)) {
                diagnostics->push_back(
                    "MoverFailed " + mover->moverPath.GetString() + ": " +
                    error);
                continue;
            }
        } else {
            const float scalar = _ResolvedRead(
                _resolvedInputs, prim, "inputs:defaultWeight", 1.0f, time);
            if (!std::isfinite(scalar) || scalar < 0.0f || scalar > 1.0f) {
                diagnostics->push_back(
                    "MoverFailed " + mover->moverPath.GetString() +
                    ": inputs:defaultWeight must be finite and in [0, 1]");
                continue;
            }
            std::fill(envelope.begin(), envelope.end(), scalar);
        }

        if (type == "RigExecBlendShapeMover") {
            // p'_i = p_i + sum_k alpha_k(w_k) d_{k,i} (spec §7.3).
            SdfPathVector inputs;
            if (UsdRelationship rel =
                    prim.GetRelationship(TfToken("rigExec:blendInputs"))) {
                rel.GetTargets(&inputs);
            }
            // Active inputs accumulate in canonical input-path order
            // (spec §7.3); authored relationship order is non-semantic.
            std::sort(inputs.begin(), inputs.end());

            VtVec3fArray next = points;
            bool failed = false;
            for (const SdfPath &inputPath : inputs) {
                const UsdPrim input = _stage->GetPrimAtPath(inputPath);
                if (!input) {
                    diagnostics->push_back(
                        "MoverFailed " + mover->moverPath.GetString() +
                        ": missing blend input " + inputPath.GetString());
                    failed = true;
                    break;
                }
                float channel = 0;
                if (UsdAttribute a =
                        input.GetAttribute(TfToken("inputs:weight"))) {
                    a.Get(&channel, time);
                }
                if (!std::isfinite(channel)) {
                    diagnostics->push_back(
                        "MoverFailed " + mover->moverPath.GetString() +
                        ": non-finite channel weight on " +
                        inputPath.GetString());
                    failed = true;
                    break;
                }
                SdfPathVector samplePaths;
                if (UsdRelationship rel = input.GetRelationship(
                        TfToken("rigExec:samples"))) {
                    rel.GetTargets(&samplePaths);
                }

                // Collect samples: activation plus target-shape points.
                // Activations must be finite, strictly positive, and
                // unique; samples compile in (activation, canonicalPath)
                // order with an implicit zero-delta sample at activation 0
                // and the channel weight clamped to [0, lastActivation]
                // (spec §7.3).
                struct _Sample {
                    float activation;
                    SdfPath path;
                    VtVec3fArray shape;
                };
                std::vector<_Sample> samples;
                if (samplePaths.empty()) {
                    diagnostics->push_back(
                        "MoverFailed " + mover->moverPath.GetString() +
                        ": blend input has no samples: " +
                        inputPath.GetString());
                    failed = true;
                    break;
                }
                for (const SdfPath &samplePath : samplePaths) {
                    const UsdPrim sample =
                        _stage->GetPrimAtPath(samplePath.GetPrimPath());
                    if (!sample) {
                        diagnostics->push_back(
                            "MoverFailed " + mover->moverPath.GetString() +
                            ": missing blend sample " +
                            samplePath.GetString());
                        failed = true;
                        break;
                    }
                    SdfPathVector shapeTargets;
                    if (UsdRelationship rel = sample.GetRelationship(
                            TfToken("rigExec:targetPoints"))) {
                        rel.GetTargets(&shapeTargets);
                    }
                    if (shapeTargets.size() != 1) {
                        diagnostics->push_back(
                            "MoverFailed " + mover->moverPath.GetString() +
                            ": blend sample must target exactly one native "
                            "points property: " + samplePath.GetString());
                        failed = true;
                        break;
                    }
                    _Sample s;
                    s.path = sample.GetPath();
                    const UsdAttribute shapeAttr =
                        _stage->GetAttributeAtPath(shapeTargets[0]);
                    if (!shapeAttr || !shapeAttr.Get(&s.shape, time) ||
                        s.shape.size() != basePoints.size()) {
                        diagnostics->push_back(
                            "MoverFailed " + mover->moverPath.GetString() +
                            ": sample cardinality mismatch at " +
                            shapeTargets[0].GetString());
                        failed = true;
                        break;
                    }
                    s.activation = 1;
                    if (UsdAttribute a = sample.GetAttribute(
                            TfToken("rigExec:activation"))) {
                        a.Get(&s.activation, time);
                    }
                    if (!std::isfinite(s.activation) || s.activation <= 0) {
                        diagnostics->push_back(
                            "MoverFailed " + mover->moverPath.GetString() +
                            ": non-positive activation at " +
                            s.path.GetString());
                        failed = true;
                        break;
                    }
                    samples.push_back(std::move(s));
                }
                if (failed) {
                    break;
                }
                if (samples.empty()) {
                    continue;
                }
                std::sort(samples.begin(), samples.end(),
                          [](const _Sample &x, const _Sample &y) {
                              return x.activation != y.activation
                                  ? x.activation < y.activation
                                  : x.path < y.path;
                          });
                for (size_t k = 1; k < samples.size(); ++k) {
                    if (samples[k].activation == samples[k - 1].activation) {
                        diagnostics->push_back(
                            "MoverFailed " + mover->moverPath.GetString() +
                            ": duplicate activation at " +
                            samples[k].path.GetString());
                        failed = true;
                        break;
                    }
                }
                if (failed) {
                    break;
                }

                const float w = std::min(
                    std::max(channel, 0.0f), samples.back().activation);
                if (w == 0.0f) {
                    continue;
                }
                // Piecewise-linear interpolation between the bracketing
                // activation samples; the lower bracket may be the
                // implicit zero-delta sample at activation 0. Deltas
                // derive against the destination BASE points (spec §7.3),
                // then accumulate onto the preceding revision.
                size_t hi = 0;
                while (hi < samples.size() &&
                       samples[hi].activation < w) {
                    ++hi;
                }
                if (hi >= samples.size()) {
                    hi = samples.size() - 1;
                }
                const float aHi = samples[hi].activation;
                const float aLo = hi > 0 ? samples[hi - 1].activation : 0.0f;
                const float t = aHi > aLo ? (w - aLo) / (aHi - aLo) : 1.0f;
                const VtVec3fArray *shapeLo =
                    hi > 0 ? &samples[hi - 1].shape : nullptr;
                const VtVec3fArray &shapeHi = samples[hi].shape;
                for (size_t i = 0; i < next.size(); ++i) {
                    const GfVec3f deltaHi = shapeHi[i] - basePoints[i];
                    const GfVec3f deltaLo = shapeLo
                        ? (*shapeLo)[i] - basePoints[i] : GfVec3f(0);
                    const GfVec3f delta =
                        deltaLo + (deltaHi - deltaLo) * t;
                    next[i] += delta;
                }
            }
            if (!failed) {
                points = next;
            }
        } else if (type == "RigExecMatrixMover") {
            // p' = q + w (T q - q) (spec §7.4).
            SdfPathVector transforms;
            if (UsdRelationship rel =
                    prim.GetRelationship(TfToken("rigExec:transform"))) {
                rel.GetTargets(&transforms);
            }
            if (transforms.size() != 1) {
                diagnostics->push_back(
                    "MoverFailed " + mover->moverPath.GetString() +
                    ": transform must have exactly one target");
                continue;
            }
            TfToken phase("base");
            if (const UsdAttribute a = prim.GetAttribute(
                    TfToken("rigExec:transformReadPhase"))) {
                a.Get(&phase);
            }
            const auto &matrices =
                phase == "final" ? finalProviderMatrices
                                 : baseProviderMatrices;
            const auto matrixIt = matrices.find(transforms[0]);
            if (matrixIt == matrices.end()) {
                diagnostics->push_back(
                    "MoverFailed " + mover->moverPath.GetString() +
                    ": no " + phase.GetString() + " matrix provider at " +
                    transforms[0].GetString());
                continue;
            }
            const GfMatrix4d &m = matrixIt->second;
            for (size_t i = 0; i < points.size(); ++i) {
                const GfVec3d moved = RigExecApplyWeightedMatrix(
                    GfVec3d(points[i]), m, 1.0f);
                points[i] = GfVec3f(moved);
            }
        } else if (type == "RigExecCurveMover") {
            TfToken mode("ribbon");
            if (UsdAttribute a = prim.GetAttribute(TfToken("rigExec:mode"))) {
                a.Get(&mode, time);
            }
            // Parity path samples the driver curve directly: rest from
            // the bind-time authored value, posed from the timed value.
            SdfPathVector frameTargets;
            if (UsdRelationship rel = prim.GetRelationship(
                    TfToken("rigExec:driverFrames"))) {
                rel.GetTargets(&frameTargets);
            }
            SdfPath curvePoints;
            int sampleCount = 5;
            if (!frameTargets.empty()) {
                if (const UsdPrim ribbon =
                        _stage->GetPrimAtPath(frameTargets[0])) {
                    SdfPathVector curves;
                    if (UsdRelationship rel = ribbon.GetRelationship(
                            TfToken("rigExec:driverCurve"))) {
                        rel.GetTargets(&curves);
                    }
                    if (!curves.empty()) {
                        curvePoints = curves[0].IsPrimPath()
                            ? curves[0].AppendProperty(TfToken("points"))
                            : curves[0];
                    }
                    if (UsdAttribute a = ribbon.GetAttribute(
                            TfToken("rigExec:sampleCount"))) {
                        a.Get(&sampleCount, time);
                    }
                }
            }
            VtVec3fArray posedCvs, restCvs;
            if (UsdAttribute a = _stage->GetAttributeAtPath(curvePoints)) {
                a.Get(&posedCvs, time);
                a.Get(&restCvs, UsdTimeCode::Default());
            }
            const auto posedSamples = RigExecSampleCurveRMF(
                std::vector<GfVec3f>(posedCvs.begin(), posedCvs.end()),
                sampleCount);
            const auto restSamples = RigExecSampleCurveRMF(
                std::vector<GfVec3f>(restCvs.begin(), restCvs.end()),
                sampleCount);
            if (mode == "emitGuidePoints") {
                if (posedSamples.GetSize() == points.size()) {
                    for (size_t i = 0; i < points.size(); ++i) {
                        points[i] = posedSamples.positions[i];
                    }
                } else {
                    diagnostics->push_back(
                        "MoverFailed " + mover->moverPath.GetString() +
                        ": guide cardinality mismatch");
                }
            } else {
                SdfPathVector binds;
                if (UsdRelationship rel = prim.GetRelationship(
                        TfToken("rigExec:bindCoordinates"))) {
                    rel.GetTargets(&binds);
                }
                VtVec2fArray sts;
                if (!binds.empty()) {
                    if (UsdAttribute a =
                            _stage->GetAttributeAtPath(binds[0])) {
                        a.Get(&sts, time);
                    }
                }
                std::vector<GfVec3f> scratch(points.begin(), points.end());
                RigExecApplyRibbonTransport(
                    &scratch,
                    std::vector<GfVec2f>(sts.begin(), sts.end()),
                    restSamples, posedSamples);
                std::copy(scratch.begin(), scratch.end(), points.begin());
            }
        } else if (type == "RigExecVolumeCorrectMover" ||
                   type == "RigExecSmoothMover") {
            const bool isVolume = type == "RigExecVolumeCorrectMover";
            std::vector<GfVec3f> scratch(points.begin(), points.end());
            if (isVolume) {
                VtVec3fArray base;
                if (UsdAttribute a = _stage->GetAttributeAtPath(target)) {
                    a.Get(&base, time);
                }
                const double reference = RigExecBoundVolume(
                    base.cdata(), base.size());
                RigExecApplyVolumeCorrect(&scratch, reference, 1.0f);
            } else {
                const UsdPrim owner =
                    _stage->GetPrimAtPath(target.GetPrimPath());
                VtIntArray counts, indices;
                if (owner) {
                    owner.GetAttribute(TfToken("faceVertexCounts"))
                        .Get(&counts, time);
                    owner.GetAttribute(TfToken("faceVertexIndices"))
                        .Get(&indices, time);
                }
                RigExecApplyLaplacianSmooth(
                    &scratch,
                    std::vector<int>(counts.begin(), counts.end()),
                    std::vector<int>(indices.begin(), indices.end()),
                    1.0f);
            }
            std::copy(scratch.begin(), scratch.end(), points.begin());
        } else if (type == "RigExecCurvenetMover") {
            // No scalar oracle. Every other mover here is a few lines of
            // arithmetic that can be written twice independently, which is
            // what makes the parity check worth anything; the Profile Mover
            // is a mesh cut plus two sparse solves, and a second
            // "independent" copy of that would be the same code with the
            // same bugs. Say so rather than leave the points untouched and
            // let parity mode report a difference it cannot explain.
            if (diagnostics) {
                diagnostics->push_back(
                    "cpu parity: " + prim.GetPath().GetString() +
                    " is a RigExecCurvenetMover, which has no scalar "
                    "reference kernel; its chain is covered by "
                    "testRigExecCurvenet instead");
            }
            continue;
        } else if (type == "RigExecLatticeMover") {
            // Independent of RigExecAssembleParameters on purpose: this is the
            // parity oracle, so it resolves its own inputs off the stage. A
            // reference that called the assembler would have agreed with the
            // SurfaceProject strength bug instead of catching it.
            //
            // The rest cage is the cage at Default time. That is exactly
            // what the deleted compiler captured into
            // rigExec:restCagePoints -- a Default-time read and nothing
            // more, which is why the authored capture was removable.
            SdfPathVector cages;
            if (UsdRelationship rel =
                    prim.GetRelationship(TfToken("rigExec:cage"))) {
                rel.GetTargets(&cages);
            }
            if (cages.empty()) {
                continue;
            }
            SdfPath cagePoints = cages[0];
            if (cagePoints.IsPrimPath()) {
                cagePoints = cagePoints.AppendProperty(TfToken("points"));
            }
            VtVec3fArray restCage, posedCage, base;
            // Rest is the BIND pose and always the authored value; only the
            // live cage carries a phase.
            if (UsdAttribute a = _stage->GetAttributeAtPath(cagePoints)) {
                a.Get(&restCage, UsdTimeCode::Default());
            }
            phasedPoints(prim, "rigExec:cage", "rigExec:cageReadPhase",
                         cagePoints, mover->moverPath, &posedCage);
            if (UsdAttribute a = _stage->GetAttributeAtPath(target)) {
                a.Get(&base, time);
            }
            GfVec3i divisions(0);
            if (UsdAttribute a =
                    prim.GetAttribute(TfToken("rigExec:divisions"))) {
                a.Get(&divisions, time);
            }
            const size_t cageCount = size_t(divisions[0]) *
                                     size_t(divisions[1]) *
                                     size_t(divisions[2]);
            if (divisions[0] < 2 || divisions[1] < 2 || divisions[2] < 2 ||
                restCage.size() != cageCount ||
                posedCage.size() != cageCount || base.size() != points.size()) {
                diagnostics->push_back(
                    "MoverFailed " + mover->moverPath.GetString() +
                    ": lattice cage/divisions mismatch");
                continue;
            }
            std::vector<GfVec3f> scratch(points.begin(), points.end());
            RigExecApplyLattice(
                &scratch, std::vector<GfVec3f>(base.begin(), base.end()),
                std::vector<GfVec3f>(restCage.begin(), restCage.end()),
                std::vector<GfVec3f>(posedCage.begin(), posedCage.end()),
                divisions);
            std::copy(scratch.begin(), scratch.end(), points.begin());
        } else if (type == "RigExecSurfaceMover") {
            SdfPathVector surfaces;
            if (UsdRelationship rel =
                    prim.GetRelationship(TfToken("rigExec:surface"))) {
                rel.GetTargets(&surfaces);
            }
            if (surfaces.empty()) {
                continue;
            }
            const SdfPath surfacePrim = surfaces[0].GetPrimPath();
            VtVec3fArray surfacePoints;
            VtIntArray counts, indices;
            phasedPoints(prim, "rigExec:surface", "rigExec:surfaceReadPhase",
                         surfacePrim.AppendProperty(TfToken("points")),
                         mover->moverPath, &surfacePoints);
            if (const UsdPrim s = _stage->GetPrimAtPath(surfacePrim)) {
                s.GetAttribute(TfToken("faceVertexCounts")).Get(&counts, time);
                s.GetAttribute(TfToken("faceVertexIndices"))
                    .Get(&indices, time);
            }
            if (surfacePoints.empty() || counts.empty()) {
                diagnostics->push_back(
                    "MoverFailed " + mover->moverPath.GetString() +
                    ": surface has no points/topology");
                continue;
            }
            std::vector<GfVec3f> scratch(points.begin(), points.end());
            // v0.1 attach/project both map fully; the mode token selects no
            // numeric difference yet (_BuildSurfaceMoverParameters pins 1.0).
            RigExecApplySurfaceProject(
                &scratch,
                std::vector<GfVec3f>(surfacePoints.begin(),
                                     surfacePoints.end()),
                std::vector<int>(counts.begin(), counts.end()),
                std::vector<int>(indices.begin(), indices.end()), 1.0f);
            std::copy(scratch.begin(), scratch.end(), points.begin());
        }

        // Every branch above computes the operation's full-strength
        // candidate. The universal envelope is the one and only blend back
        // over the preceding revision.
        if (points.size() != preceding.size() ||
            envelope.size() != points.size()) {
            diagnostics->push_back(
                "MoverFailed " + mover->moverPath.GetString() +
                ": result cardinality changed; revision passed through");
            points = preceding;
            continue;
        }
        for (size_t i = 0; i < points.size(); ++i) {
            points[i] = RigExecBlendEnvelope(
                preceding[i], points[i], envelope[i]);
        }
    }
    return points;
}

namespace {

// Reads the authored inputs of one float/vec3f math mover at \p time.
//
// Every field is read even though the operation uses only some of them: the
// packet is the mover's whole authored state, and branching on the operation
// while reading would put the same switch in two places.
template <class T>
bool
_ReadPropertyMathParams(
    const RigExecResolvedInputs &resolved, const UsdPrim &moverPrim,
    UsdTimeCode time,
    RigExecPropertyMathParams<T> *params)
{
    TfToken operation;
    if (const UsdAttribute a =
            moverPrim.GetAttribute(TfToken("rigExec:operation"))) {
        a.Get(&operation);
    }
    if (!RigExecParsePropertyOp(operation, &params->op)) {
        return false;
    }
    params->value = _ResolvedRead(
        resolved, moverPrim, "inputs:value", params->value, time);
    params->min = _ResolvedRead(
        resolved, moverPrim, "inputs:min", params->min, time);
    params->max = _ResolvedRead(
        resolved, moverPrim, "inputs:max", params->max, time);
    return true;
}

bool
_IsFinite(float v)
{
    return std::isfinite(v);
}

bool
_IsFinite(const GfVec3f &v)
{
    return std::isfinite(v[0]) && std::isfinite(v[1]) && std::isfinite(v[2]);
}

bool
_IsFinite(const GfMatrix4d &m)
{
    for (size_t r = 0; r < 4; ++r) {
        for (size_t c = 0; c < 4; ++c) {
            if (!std::isfinite(m[r][c])) {
                return false;
            }
        }
    }
    return true;
}

}  // namespace

void
RigExecRigEvaluator::_EvaluatePropertyChains(
    UsdTimeCode time,
    std::map<SdfPath, VtValue> *results,
    std::vector<RigExecValueOverride> *overrides,
    std::vector<std::string> *diagnostics)
{
    auto diag = [diagnostics](const std::string &message) {
        if (diagnostics) {
            diagnostics->push_back(message);
        }
    };

    for (const SdfPath &orderedTarget : _propertyChainOrder) {
        const auto chainIt = _propertyChains.find(orderedTarget);
        if (chainIt == _propertyChains.end()) {
            continue;
        }
        const auto &chain = *chainIt;
        // Named locals rather than a structured binding: the lambdas below
        // capture both, and capturing a structured binding is C++20.
        const SdfPath &target = chain.first;
        const std::vector<_PropertyRevision> &revisions = chain.second;
        const UsdAttribute attr = _stage->GetAttributeAtPath(target);
        if (!attr) {
            diag("property chain " + target.GetString() +
                 ": target attribute disappeared; chain skipped");
            continue;
        }
        const SdfValueTypeName valueType = attr.GetTypeName();

        // One shared revision loop over the three value domains. Each
        // iteration reads the mover's own authored state and applies it to
        // the preceding revision -- the base being the target's AUTHORED
        // value, exactly as a point chain's base is the target's authored
        // points.
        auto runChain = [&](auto value, auto apply) {
            using ValueT = decltype(value);
            if (!attr.Get(&value, time)) {
                diag("property chain " + target.GetString() +
                     ": target has no authored value; chain skipped");
                return false;
            }
            if (!_IsFinite(value)) {
                diag("property chain " + target.GetString() +
                     ": authored base is not finite; chain skipped");
                return false;
            }
            for (const _PropertyRevision &revision : revisions) {
                const UsdPrim moverPrim =
                    _stage->GetPrimAtPath(revision.moverPath);
                if (!moverPrim) {
                    continue;
                }
                const bool enabled = _ResolvedRead(
                    _resolvedInputs, moverPrim, "inputs:enabled", true, time);
                if (!enabled) {
                    diag("diag " + revision.moverPath.GetString() +
                         ": disabled; revision passed through");
                    continue;  // ordinary pass-through (spec §6.6)
                }
                float envelope = 1.0f;
                SdfPathVector weightObjects;
                if (const UsdRelationship rel =
                        moverPrim.GetRelationship(
                            TfToken("rigExec:weightObject"))) {
                    rel.GetTargets(&weightObjects);
                }
                if (!weightObjects.empty()) {
                    std::vector<float> weights;
                    std::string error;
                    if (!_ResolveWeights(weightObjects[0], 1, time,
                                         &weights, &error) ||
                        weights.size() != 1) {
                        diag("diag " + revision.moverPath.GetString() +
                             ": " + error + "; revision passed through");
                        continue;
                    }
                    envelope = weights[0];
                } else {
                    envelope = _ResolvedRead(
                        _resolvedInputs, moverPrim, "inputs:defaultWeight",
                        1.0f, time);
                    if (!std::isfinite(envelope) || envelope < 0.0f ||
                        envelope > 1.0f) {
                        diag("diag " + revision.moverPath.GetString() +
                             ": inputs:defaultWeight must be finite and in "
                             "[0, 1]; revision passed through");
                        continue;
                    }
                }
                ValueT next = value;
                if (!apply(moverPrim, value, envelope, &next)) {
                    diag("diag " + revision.moverPath.GetString() +
                         ": inputs unusable; revision passed through");
                    continue;
                }
                if (!_IsFinite(next)) {
                    // A NaN reaching an exec override propagates into every
                    // consumer of the attribute with no way to report it
                    // back, so the mover fails and passes through instead
                    // (spec §6.6).
                    diag("diag " + revision.moverPath.GetString() +
                         ": produced a non-finite value; revision passed "
                         "through");
                    continue;
                }
                value = next;
            }
            if (results) {
                (*results)[target] = VtValue(value);
            }
            // Publish immediately, not after every chain has run. Dependency
            // ordering guarantees that any later chain which consumes this
            // property reads the revised value.
            _resolvedInputs.SetProperty(target, VtValue(value));
            if (overrides) {
                overrides->push_back(RigExecValueOverride{
                    target.GetPrimPath(), TfToken(), target.GetNameToken(),
                    VtValue(value)});
            }
            return true;
        };

        if (valueType == SdfValueTypeNames->Float) {
            runChain(float(0), [&](const UsdPrim &mover, float in,
                                   float envelope, float *out) {
                RigExecPropertyMathParams<float> params;
                if (!_ReadPropertyMathParams(
                        _resolvedInputs, mover, time, &params) ||
                    !_IsFinite(params.value) || !_IsFinite(params.min) ||
                    !_IsFinite(params.max)) {
                    return false;
                }
                params.weight = envelope;
                *out = RigExecApplyFloatMath(in, params);
                return true;
            });
        } else if (valueType == SdfValueTypeNames->Matrix4d) {
            runChain(GfMatrix4d(1.0), [&](const UsdPrim &mover,
                                          const GfMatrix4d &in,
                                          float envelope,
                                          GfMatrix4d *out) {
                TfToken operation;
                if (const UsdAttribute a =
                        mover.GetAttribute(TfToken("rigExec:operation"))) {
                    a.Get(&operation);
                }
                RigExecPropertyOp op;
                if (!RigExecParsePropertyOp(operation, &op)) {
                    return false;
                }
                const GfMatrix4d opValue = _ResolvedRead(
                    _resolvedInputs, mover, "inputs:value",
                    GfMatrix4d(1.0), time);
                if (!_IsFinite(opValue)) {
                    return false;
                }
                return RigExecApplyMatrixMath(
                    in, op, opValue, envelope, out);
            });
        } else {
            // Every remaining type the compiler admits is GfVec3f-backed.
            runChain(GfVec3f(0), [&](const UsdPrim &mover, const GfVec3f &in,
                                     float envelope,
                                     GfVec3f *out) {
                RigExecPropertyMathParams<GfVec3f> params;
                if (!_ReadPropertyMathParams(
                        _resolvedInputs, mover, time, &params) ||
                    !_IsFinite(params.value) || !_IsFinite(params.min) ||
                    !_IsFinite(params.max)) {
                    return false;
                }
                params.weight = envelope;
                *out = RigExecApplyVec3fMath(in, params);
                return true;
            });
        }
    }
}

RigExecRigPose
RigExecRigEvaluator::Evaluate(UsdTimeCode time)
{
    RigExecRigPose pose;
    pose.time = time;
    if (!_compiled && !Compile(nullptr)) {
        return pose;
    }
    // Structural edits begin a new epoch: recompile when the composed
    // mover topology digest changed (spec §4.2, §6.3).
    if (_ComputeStructureDigest() != _structureDigest) {
        if (!Compile(nullptr)) {
            pose.diagnostics.push_back(
                "structural recompilation failed");
            return pose;
        }
        pose.diagnostics.push_back("structural edit: epoch rebuilt");
    }

    // 0. Solver aggregates first, then each solver-posed joint's frame as an
    // override on the authoritative request. This is the whole of the
    // solver->joint binding: the element choice is compiled state, and exec
    // sees the result as if the joint had computed it, so computeMatrix,
    // frame-chain applications, and the NamespaceAncestor fallback that
    // unbound descendants follow all stay correct with nothing authored.
    // Ribbon driver-curve points, live and at bind time. Constant for a given
    // time, so they are built once and prefixed to every override vector --
    // including the guide taps, which evaluate the same ribbon solvers.
    // Bind-time values read at Default(), which is exactly what the deleted
    // compiler pass captured.
    std::vector<RigExecValueOverride> baseOverrides;

    // Property chains resolve FIRST, and their results ride in as attribute
    // overrides on every request below.
    //
    // This is the whole point of evaluating them off the authored stage: a
    // math mover's inputs are all authored on itself, so its chain owes exec
    // nothing and can be computed before exec runs -- which means the value
    // it produces can be handed to exec as the attribute's value. A clamped
    // IK/FK weight then reaches RigExecBlendPointFrames as the weight it
    // reads, instead of that kernel reimplementing the author's clamp
    // internally and the authored mover meaning nothing.
    //
    // No cycle is possible: nothing in a property chain reads a computation.
    std::map<SdfPath, VtValue> propertyResults;
    _resolvedInputs.Clear();
    _chainSnapshots.Clear();
    if (!_propertyChains.empty()) {
        _EvaluatePropertyChains(time, &propertyResults, &baseOverrides,
                                &pose.diagnostics);
        // Two delivery routes for one value, and they must not disagree.
        // baseOverrides carries it to every exec consumer; _resolvedInputs
        // carries it to the static reads exec never touches -- packet
        // assembly and the CPU oracle.
        for (const auto &[path, value] : propertyResults) {
            _resolvedInputs.SetProperty(path, value);
        }
    }

    for (const auto &[ribbonPath, pointsPath] : _ribbonDriverPoints) {
        const UsdAttribute a = _stage->GetAttributeAtPath(pointsPath);
        if (!a) {
            continue;
        }
        VtVec3fArray live, rest;
        a.Get(&live, time);
        a.Get(&rest, UsdTimeCode::Default());
        RigExecPointsPacket livePacket, restPacket;
        livePacket.points.assign(live.begin(), live.end());
        restPacket.points.assign(rest.begin(), rest.end());
        baseOverrides.push_back(RigExecValueOverride{
            ribbonPath, TfToken("rigExec:computeDriverPoints"), TfToken(),
            VtValue(livePacket)});
        baseOverrides.push_back(RigExecValueOverride{
            ribbonPath, TfToken("rigExec:computeRestDriverPoints"), TfToken(),
            VtValue(restPacket)});
    }

    // Rest-implied TwoBoneIk lengths, measured before anything consumes
    // them: each record's joints publish computeRestFrame through the
    // epoch's rest request, and a bone is its rest-origin distance plus
    // the authored offset. The overrides join baseOverrides, so the
    // solver aggregates, the joint-override loop, and the solver guides
    // all read the same implied values -- and an authored absolute
    // (which implies no record) is never touched.
    if (_restFrameTaps && !_impliedIkLengths.empty()) {
        const RigExecSnapshot restSnapshot =
            _restFrameTaps->Evaluate(time, baseOverrides);
        if (!restSnapshot.IsValid() || !restSnapshot.IsComplete()) {
            pose.diagnostics.push_back(
                restSnapshot.IsValid()
                    ? "implied-length rest snapshot incomplete: schema "
                      "defaults stand"
                    : "implied-length rest evaluation failed: schema "
                      "defaults stand");
        } else {
            for (const _ImpliedIkLengths &implied : _impliedIkLengths) {
                RigExecPointFrame rests[3];
                bool usable = true;
                for (int i = 0; i < 3; ++i) {
                    if (implied.joints[i].IsEmpty()) {
                        usable = false;
                        break;
                    }
                    const auto tap =
                        _impliedRestTaps.find(implied.joints[i]);
                    if (tap == _impliedRestTaps.end()) {
                        usable = false;
                        break;
                    }
                    rests[i] =
                        restSnapshot.Get<RigExecPointFrame>(tap->second);
                    if (!rests[i].IsValid()) {
                        usable = false;
                        break;
                    }
                }
                if (!usable) {
                    pose.diagnostics.push_back(
                        "solver " + implied.solver.GetString() +
                        " binds fewer than three elements; unauthored "
                        "lengths keep their schema defaults");
                    continue;
                }
                const UsdPrim solverPrim =
                    _stage->GetPrimAtPath(implied.solver);
                const GfVec3d origins[3] = {
                    rests[0].Origin(), rests[1].Origin(),
                    rests[2].Origin()};
                // A mover-driven length is explicit authoring intent: when
                // a property chain already overrode the attribute, the
                // rest-implied value yields to it rather than pushing a
                // second override for the same key.
                auto drivenByMover = [&](const char *lengthName) {
                    const TfToken name(lengthName);
                    for (const RigExecValueOverride &o : baseOverrides) {
                        if (o.prim == implied.solver &&
                            o.attribute == name) {
                            return true;
                        }
                    }
                    return false;
                };
                auto imply = [&](bool want, const char *lengthName,
                                 const char *offsetName, const GfVec3d &a,
                                 const GfVec3d &b) {
                    if (!want || drivenByMover(lengthName)) {
                        return;
                    }
                    const double measured = (b - a).GetLength();
                    double offset = _ResolvedRead(
                        _resolvedInputs, solverPrim, offsetName, 0.0, time);
                    if (!std::isfinite(offset)) {
                        pose.diagnostics.push_back(
                            "solver " + implied.solver.GetString() + " " +
                            offsetName + " is not finite; treated as 0");
                        offset = 0.0;
                    }
                    if (!std::isfinite(measured)) {
                        pose.diagnostics.push_back(
                            "solver " + implied.solver.GetString() +
                            " rest distance for " + lengthName +
                            " is not finite; schema default stands");
                        return;
                    }
                    const double length = measured + offset;
                    if (length <= 0.0) {
                        pose.diagnostics.push_back(
                            "solver " + implied.solver.GetString() +
                            " implied " + lengthName +
                            " is not positive; the kernel clamps it");
                    }
                    baseOverrides.push_back(RigExecValueOverride{
                        implied.solver, TfToken(), TfToken(lengthName),
                        VtValue(length)});
                    pose.diagnostics.push_back(
                        "solver " + implied.solver.GetString() + " implied " +
                        lengthName + "=" + std::to_string(length) +
                        " (rest " + std::to_string(measured) + " + offset " +
                        std::to_string(offset) + ")");
                };
                imply(
                    implied.implyUpper, "rigExec:upperLength",
                    "rigExec:upperLengthOffset", origins[0], origins[1]);
                imply(
                    implied.implyLower, "rigExec:lowerLength",
                    "rigExec:lowerLengthOffset", origins[1], origins[2]);
            }
        }
    }

    std::vector<RigExecValueOverride> jointOverrides = baseOverrides;
    // Joints whose solver published no element for them (an incomplete
    // solver: its required inputs are unwired, so the kernel returned an
    // empty aggregate). They keep their natural rest-chain frame below, so
    // a rig mid-edit stays visible instead of vanishing.
    std::set<SdfPath> fallbackJoints;
    if (_solverFrameTaps && !_jointSolverBinding.empty()) {
        // Iterated to a fixed point, NOT computed once.
        //
        // Solvers are not downstream-only of controls: RigExecTwistDistribution
        // takes rigExec:start / rigExec:end as computePointFrame inputs, and
        // those endpoints are routinely joints that ANOTHER solver poses
        // (05_TwistRibbonSpine: SpineFK poses Root and Chest, which SpineTwist
        // reads, and SpineTwist in turn poses TwistMid). Evaluating the
        // aggregates once with no overrides would compute such a solver from
        // its endpoints' unposed namespace-parent fallback, and the joint it
        // poses would get an override derived from stale input.
        //
        // Graph/lowered parity cannot see this: both paths consume the same
        // override, so both are wrong together and agree perfectly.
        //
        // Each round feeds the previous round's overrides back in, so a solver
        // that depends on another solver's joints converges one level per
        // round. The binding graph is a DAG (a joint claimed by two solvers is
        // rejected at compile), so depth is bounded by the number of posing
        // solvers; the cap is a backstop, and stability is the real exit.
        const size_t maxRounds = _jointSolverArrayTaps.size() + 1;
        for (size_t round = 0; round < maxRounds; ++round) {
            const RigExecSnapshot solverSnapshot =
                _solverFrameTaps->Evaluate(time, jointOverrides);
            if (!solverSnapshot.IsValid() || !solverSnapshot.IsComplete()) {
                pose.diagnostics.push_back(
                    solverSnapshot.IsValid()
                        ? "solver aggregate snapshot incomplete: missing frames"
                        : "solver aggregate evaluation failed");
                return pose;
            }
            std::map<SdfPath, RigExecPointFrameArray> aggregates;
            for (const auto &[solverPath, tap] : _jointSolverArrayTaps) {
                aggregates[solverPath] =
                    solverSnapshot.Get<RigExecPointFrameArray>(tap);
            }

            // Seeded with the ribbon driver points, which are constant for
            // this time: the loop rebuilds this vector every round, so a bare
            // `next` would drop them after round 0 and the ribbon would
            // sample an empty curve.
            std::vector<RigExecValueOverride> next = baseOverrides;
            next.reserve(baseOverrides.size() + _jointSolverBinding.size());
            for (const auto &[jointPath, binding] : _jointSolverBinding) {
                const auto it = aggregates.find(binding.first);
                if (it == aggregates.end()) {
                    continue;
                }
                // No element for this joint means the solver never ran: its
                // required inputs are unwired and the kernel returned an
                // empty aggregate. Emit no override so the joint keeps its
                // natural rest-chain frame (and stays visible); the
                // diagnostic below names the gap. A PRESENT-but-degenerate
                // element is different -- the solver ran and failed
                // atomically -- and keeps propagating as an override
                // (spec §6.6: no silent substitution of a real failure).
                if (binding.second < 0 ||
                    static_cast<size_t>(binding.second) >=
                        it->second.GetSize()) {
                    fallbackJoints.insert(jointPath);
                    continue;
                }
                next.push_back(RigExecValueOverride{
                    jointPath, _computePointFrame, TfToken(),
                    VtValue(RigExecExtractElementFrame(
                        &it->second, static_cast<size_t>(binding.second)))});
            }

            bool changed = next.size() != jointOverrides.size();
            for (size_t i = 0; !changed && i < next.size(); ++i) {
                changed = next[i].prim != jointOverrides[i].prim ||
                          next[i].value != jointOverrides[i].value;
            }
            jointOverrides.swap(next);
            if (!changed) {
                // Rounds beyond the first that still moved a value are the
                // solver-depends-on-solver-posed-joint case; reporting the
                // count is what makes that dependency visible instead of
                // silently absorbed.
                pose.solverOverrideRounds = round;
                pose.diagnostics.push_back(
                    "solver->joint overrides converged after " +
                    std::to_string(round) + " refinement round(s)");
                break;  // fixed point
            }
            if (round + 1 == maxRounds) {
                // Refusing to publish, not publishing the last iterate.
                // A non-converged override set means some joint is posed
                // from a frame that is still moving, and every consumer
                // downstream -- geometry, matrices, imaging -- would take it
                // as settled. An unevaluated generation is recoverable; a
                // plausible wrong one is not (spec §6.6).
                pose.solverOverrideRounds = maxRounds;
                pose.solverOverridesConverged = false;
                pose.diagnostics.push_back(
                    "solver->joint overrides did not converge in " +
                    std::to_string(maxRounds) +
                    " rounds; refusing to publish this generation");
                return pose;  // pose.valid stays false
            }
        }
    }

    // An incomplete solver is an authoring gap, not a silent one: name every
    // joint that fell back so a rig mid-edit explains itself.
    for (const SdfPath &jointPath : fallbackJoints) {
        const auto binding = _jointSolverBinding.find(jointPath);
        const std::string solver =
            binding != _jointSolverBinding.end()
                ? binding->second.first.GetString()
                : std::string("<unknown>");
        const int element =
            binding != _jointSolverBinding.end() ? binding->second.second
                                                 : -1;
        pose.diagnostics.push_back(
            "solver " + solver + " published no element " +
            std::to_string(element) + " for joint " + jointPath.GetString() +
            "; joint fell back to its rest chain");
    }

    // Baked falloff tables ride along with the joint overrides. They are
    // epoch-constant, so this replays the same values Compile produced
    // until the next epoch -- exec has no accessor for an attribute's
    // spline, and a falloff curve is the whole function rather than one
    // resolved value (see RigExecFalloffLut in types.h).
    jointOverrides.insert(jointOverrides.end(), _falloffLutOverrides.begin(),
                          _falloffLutOverrides.end());

    // 1. Transforms and solvers through OpenExec. An incomplete snapshot
    // means some computation failed to compile or evaluate; refusing to
    // continue prevents default-constructed values from masquerading as
    // results (spec §6.6).
    const RigExecSnapshot snapshot = _taps->Evaluate(time, jointOverrides);
    if (!snapshot.IsValid() || !snapshot.IsComplete()) {
        pose.diagnostics.push_back(
            snapshot.IsValid() ? "snapshot incomplete: missing tap values"
                               : "snapshot evaluation failed");
        return pose;
    }

    // Resolved volume placements, for the CPU oracle and for any
    // `current`-phase field recomputed against the in-flight points.
    _volumeWeightMatrices.clear();
    for (const auto &[weightPath, tap] : _volumeWeightMatrixTaps) {
        // The tap is the ABSOLUTE posed frame; the placement is the map
        // taking the identity landmarks to it.
        const RigExecPointFrame posed = snapshot.Get<RigExecPointFrame>(tap);
        GfMatrix4d placement(1.0);
        if (posed.IsValid() && !posed.IsDegenerate()) {
            RigExecPointsToMatrix(
                RigExecIdentityLandmarks(), posed.points, &placement);
        }
        _volumeWeightMatrices[weightPath] = placement;
    }
    // Published as-is: the imaging bridge draws each volume's falloff
    // iso-surfaces in exactly the space its field was measured in.
    pose.weightFrames = _volumeWeightMatrices;
    // 2. Pose-domain FBX-style constraints, applied in the single composed
    // mover walk. A global walk is essential for SingleChainIK: all joints in
    // its write set must be solved and committed atomically, while ordinary
    // one-provider constraints still chain in exactly the same order as every
    // points/property revision.
    std::map<SdfPath, RigExecPointFrame> baseFrames;
    std::map<SdfPath, RigExecPointFrame> finalFrames;
    std::map<SdfPath, RigExecPointFrame> restFrames;
    std::map<SdfPath, GfMatrix4d> xformDerivedBases;
    std::map<SdfPath, GfMatrix4d> finalMatrices;
    /// Geometry-domain constraint results: the delta each one produced, the
    /// envelope it carries, and its optional per-element weight field.
    /// Produced by the pose walk below and consumed after it, the same
    /// in-memory hand-off finalMatrices performs for a "final" read phase.
    std::map<SdfPath, GfMatrix4d> constraintDeltas;
    std::map<SdfPath, double> constraintEnvelopes;
    std::map<SdfPath, SdfPath> constraintWeightObjects;
    const UsdPrim assetRoot =
        _stage->GetPrimAtPath(_rigPath.GetParentPath());
    UsdGeomXformCache constraintXformCache(time);

    auto frameFromXform = [&](const SdfPath &path,
                              RigExecPointFrame *out,
                              GfMatrix4d *matrix) {
        const UsdPrim prim = _stage->GetPrimAtPath(path);
        if (!prim || !assetRoot || !UsdGeomXformable(prim)) {
            return false;
        }
        bool resetsBelowAsset = false;
        const GfMatrix4d relative =
            constraintXformCache.ComputeRelativeTransform(
                prim, assetRoot, &resetsBelowAsset);
        if (out) {
            *out = RigExecFrameFromMatrix(relative);
        }
        if (matrix) {
            *matrix = relative;
        }
        return true;
    };

    // Resolve every written provider's base/rest state before any revision
    // runs, so a multi-output operation never observes a half-updated chain.
    for (const auto &[provider, revisions] : _frameChains) {
        RigExecPointFrame base;
        if (_xformDerivedProviders.count(provider)) {
            GfMatrix4d matrix(1.0);
            if (!frameFromXform(provider, &base, &matrix)) {
                pose.diagnostics.push_back(
                    "could not resolve constraint target " +
                    provider.GetString() + " relative to the asset root");
                return pose;
            }
            xformDerivedBases[provider] = matrix;
            restFrames[provider] = RigExecFrameFromMatrix(GfMatrix4d(1.0));
        } else if (const auto it = _providerBaseFrameTaps.find(provider);
                   it != _providerBaseFrameTaps.end()) {
            base = snapshot.Get<RigExecPointFrame>(it->second);
        } else {
            const auto joint =
                std::find(_jointPaths.begin(), _jointPaths.end(), provider);
            if (joint == _jointPaths.end()) {
                pose.diagnostics.push_back(
                    "constraint target has no base frame: " +
                    provider.GetString());
                return pose;
            }
            base = snapshot.Get<RigExecPointFrame>(
                _jointFrameTaps[joint - _jointPaths.begin()]);
        }
        if (!_xformDerivedProviders.count(provider)) {
            const auto rest = _providerRestFrameTaps.find(provider);
            if (rest != _providerRestFrameTaps.end()) {
                restFrames[provider] =
                    snapshot.Get<RigExecPointFrame>(rest->second);
            }
        }
        baseFrames[provider] = base;
        finalFrames[provider] = base;
    }

    // Constraints execute after the OpenExec snapshot, so a revised parent
    // joint cannot make exec recompute its namespace descendants this
    // generation. Keep every discovered joint/control in the in-memory pose
    // map so an ancestor revision can carry untouched branches with it and a
    // later constraint can consume that revised provider.
    for (size_t i = 0; i < _jointPaths.size(); ++i) {
        const SdfPath &path = _jointPaths[i];
        if (!finalFrames.count(path)) {
            const RigExecPointFrame frame =
                snapshot.Get<RigExecPointFrame>(_jointFrameTaps[i]);
            baseFrames[path] = frame;
            finalFrames[path] = frame;
        }
    }
    for (size_t i = 0; i < _controlPaths.size(); ++i) {
        const SdfPath &path = _controlPaths[i];
        if (!finalFrames.count(path)) {
            const RigExecPointFrame frame =
                snapshot.Get<RigExecPointFrame>(_controlFrameTaps[i]);
            baseFrames[path] = frame;
            finalFrames[path] = frame;
        }
    }

    auto resolveBinding = [&](const _FrameSourceBinding &binding,
                              RigExecPointFrame *out) {
        // Constraint relationships have implicit `preceding` semantics: the
        // single composed mover walk is the authority, so a later constraint
        // observes every earlier revision of the provider while a reference to
        // a provider written later still sees its current (normally base)
        // value. This is deterministic and cannot create an evaluation cycle.
        if (const auto revised = finalFrames.find(binding.sourcePath);
            revised != finalFrames.end()) {
            *out = revised->second;
            return out->IsValid();
        }
        if (binding.frameTap >= 0) {
            *out = snapshot.Get<RigExecPointFrame>(binding.frameTap);
            return out->IsValid();
        }
        if (!binding.xformPath.IsEmpty()) {
            if (!frameFromXform(binding.xformPath, out, nullptr) ||
                !out->IsValid()) {
                return false;
            }
            // A native source that is not itself a written provider may still
            // sit beneath a constrained transform provider. The closest
            // revised ancestor contains all higher ancestor deltas, so apply
            // it once to the stage-derived source frame.
            SdfPath closest;
            for (const auto &[provider, current] : finalFrames) {
                const auto base = baseFrames.find(provider);
                if (provider == binding.xformPath ||
                    !binding.xformPath.HasPrefix(provider) ||
                    base == baseFrames.end() ||
                    current.points == base->second.points) {
                    continue;
                }
                if (closest.IsEmpty() ||
                    provider.GetPathElementCount() >
                        closest.GetPathElementCount()) {
                    closest = provider;
                }
            }
            if (!closest.IsEmpty()) {
                GfMatrix4d delta(1.0);
                if (!RigExecPointsToMatrix(
                        baseFrames[closest].points,
                        finalFrames[closest].points, &delta)) {
                    return false;
                }
                *out = RigExecMatrixToPoints(out->points, delta);
            }
            return out->IsValid();
        }
        return false;
    };

    auto readWeights = [&](const UsdPrim &prim, const char *name,
                           size_t count, std::vector<double> *weights) {
        VtFloatArray authored;
        if (const UsdAttribute a = prim.GetAttribute(TfToken(name))) {
            a.Get(&authored, time);
        }
        if (!authored.empty() && authored.size() != count) {
            pose.diagnostics.push_back(
                prim.GetPath().GetString() + " " + name + " has " +
                std::to_string(authored.size()) + " entries for " +
                std::to_string(count) + " sources");
            return false;
        }
        weights->assign(count, 1.0);
        for (size_t i = 0; i < authored.size(); ++i) {
            (*weights)[i] = authored[i];
        }
        return true;
    };

    auto readOffsets = [&](const UsdPrim &prim, const char *name,
                           size_t count, std::vector<GfVec3d> *offsets) {
        VtVec3dArray authored;
        if (const UsdAttribute a = prim.GetAttribute(TfToken(name))) {
            a.Get(&authored, time);
        }
        if (!authored.empty() && authored.size() != count) {
            pose.diagnostics.push_back(
                prim.GetPath().GetString() + " " + name + " has " +
                std::to_string(authored.size()) + " entries for " +
                std::to_string(count) + " sources");
            return false;
        }
        offsets->assign(count, GfVec3d(0));
        for (size_t i = 0; i < authored.size(); ++i) {
            (*offsets)[i] = authored[i];
        }
        return true;
    };

    auto buildSources = [&](const _FrameConstraint &constraint,
                            const UsdPrim &prim,
                            std::vector<RigExecConstraintSource> *sources) {
        std::vector<double> weights;
        if (!readWeights(prim, "inputs:sourceWeights",
                         constraint.sources.size(), &weights)) {
            return false;
        }
        std::vector<GfVec3d> translationOffsets, rotationOffsets;
        if (constraint.schemaType == "RigExecParentConstraint") {
            if (!readOffsets(prim, "inputs:translationOffsets",
                             constraint.sources.size(),
                             &translationOffsets) ||
                !readOffsets(prim, "inputs:rotationOffsets",
                             constraint.sources.size(), &rotationOffsets)) {
                return false;
            }
        } else {
            translationOffsets.assign(constraint.sources.size(), GfVec3d(0));
            rotationOffsets.assign(constraint.sources.size(), GfVec3d(0));
        }
        sources->clear();
        sources->reserve(constraint.sources.size());
        for (size_t i = 0; i < constraint.sources.size(); ++i) {
            RigExecPointFrame sourceFrame;
            if (!resolveBinding(constraint.sources[i], &sourceFrame)) {
                pose.diagnostics.push_back(
                    prim.GetPath().GetString() +
                    " could not resolve source " +
                    constraint.sources[i].sourcePath.GetString());
                return false;
            }
            RigExecConstraintSource source;
            source.frame = sourceFrame;
            source.normalizedWeight = weights[i];
            source.translationOffset = translationOffsets[i];
            source.rotationOffsetDegrees = rotationOffsets[i];
            sources->push_back(source);
        }
        return true;
    };

    auto recordFrame = [&](const SdfPath &provider,
                           const SdfPath &afterMover) {
        const auto wanted = _snapshotPoints.find(provider);
        const auto frame = finalFrames.find(provider);
        if (wanted == _snapshotPoints.end() ||
            !wanted->second.count(afterMover) || frame == finalFrames.end() ||
            !frame->second.IsValid()) {
            return;
        }
        const auto rest = restFrames.find(provider);
        const auto &landmarks =
            rest != restFrames.end() && rest->second.IsValid()
                ? rest->second.points
                : RigExecIdentityLandmarks();
        GfMatrix4d matrix(1.0);
        if (RigExecPointsToMatrix(landmarks, frame->second.points, &matrix)) {
            _chainSnapshots.Record(provider, afterMover, VtValue(matrix));
        }
    };

    const std::set<SdfPath> hierarchicalProviders(
        [&]() {
            std::set<SdfPath> result(_jointPaths.begin(), _jointPaths.end());
            result.insert(_controlPaths.begin(), _controlPaths.end());
            return result;
        }());

    // Validate and commit one constraint's complete write bundle. Descendant
    // RigExec providers are updated from the nearest changed ancestor in the
    // same transaction; native Xform descendants ride the published ancestor
    // delta in Hydra and therefore must not be duplicated here.
    auto commitConstraintFrames =
        [&](const SdfPath &moverPath,
            const std::map<SdfPath, RigExecPointFrame> &candidates) {
        for (const auto &[path, frame] : candidates) {
            if (!_IsUsableConstraintFrame(frame)) {
                pose.diagnostics.push_back(
                    moverPath.GetString() +
                    " produced an invalid or degenerate frame for " +
                    path.GetString() + "; constraint passed through");
                return false;
            }
        }

        std::map<SdfPath, RigExecPointFrame> propagated;
        for (const auto &[provider, current] : finalFrames) {
            if (candidates.count(provider) ||
                !hierarchicalProviders.count(provider)) {
                continue;
            }
            SdfPath closest;
            for (const auto &[target, candidate] : candidates) {
                if (provider != target && provider.HasPrefix(target) &&
                    (closest.IsEmpty() ||
                     target.GetPathElementCount() >
                         closest.GetPathElementCount())) {
                    closest = target;
                }
            }
            if (closest.IsEmpty()) {
                continue;
            }
            const auto before = finalFrames.find(closest);
            if (before == finalFrames.end() ||
                !_IsUsableConstraintFrame(current)) {
                pose.diagnostics.push_back(
                    moverPath.GetString() +
                    " could not propagate its pose revision through " +
                    provider.GetString() + "; constraint passed through");
                return false;
            }
            GfMatrix4d delta(1.0);
            if (!RigExecPointsToMatrix(
                    before->second.points, candidates.at(closest).points,
                    &delta)) {
                pose.diagnostics.push_back(
                    moverPath.GetString() +
                    " produced a singular hierarchy delta; constraint passed "
                    "through");
                return false;
            }
            RigExecPointFrame frame =
                RigExecMatrixToPoints(current.points, delta);
            if (!_IsUsableConstraintFrame(frame)) {
                pose.diagnostics.push_back(
                    moverPath.GetString() +
                    " produced an invalid descendant frame for " +
                    provider.GetString() + "; constraint passed through");
                return false;
            }
            propagated[provider] = frame;
        }

        for (const auto &[path, frame] : candidates) {
            finalFrames[path] = frame;
        }
        for (const auto &[path, frame] : propagated) {
            finalFrames[path] = frame;
        }
        return true;
    };

    auto ikUsesAnimatedTs = [&](const std::vector<SdfPath> &chain) {
        for (const SdfPath &path : chain) {
            if (_jointSolverBinding.count(path)) {
                return true;
            }
            const UsdPrim joint = _stage->GetPrimAtPath(path);
            for (const char *name : {
                     "posed:space", "avars:tx", "avars:ty", "avars:tz",
                     "avars:sx", "avars:sy", "avars:sz"}) {
                const UsdAttribute attr = joint.GetAttribute(TfToken(name));
                SdfPathVector connections;
                if (attr &&
                    (attr.GetNumTimeSamples() > 0 ||
                     (attr.GetConnections(&connections) &&
                      !connections.empty()))) {
                    return true;
                }
            }
        }
        return false;
    };

    // `neverTS` retains current rotations/root placement while rebuilding
    // child placement and handle lengths from rest frames. A joint without
    // any authored rest transform has the schema's identity fallback, which
    // is not an actual chain rest layout; use its current static layout in
    // that case. The public math solver can therefore keep measuring its
    // input chain; evaluator-side preparation decides whether those
    // measurements are rest- or animation-derived.
    auto ikWithoutAnimatedTs =
        [&](const std::vector<RigExecPointFrame> &current,
            const std::vector<RigExecPointFrame> &rest,
            std::vector<RigExecPointFrame> *prepared) {
        if (current.size() != rest.size() || current.empty()) {
            return false;
        }
        bool usableRestLayout = true;
        for (size_t i = 1; i < rest.size(); ++i) {
            const double segmentLength =
                (rest[i].Origin() - rest[i - 1].Origin()).GetLength();
            if (!std::isfinite(segmentLength) || segmentLength <= 0.0) {
                usableRestLayout = false;
                break;
            }
        }
        const std::vector<RigExecPointFrame> &lengthReference =
            usableRestLayout ? rest : current;
        prepared->clear();
        prepared->reserve(current.size());
        for (size_t i = 0; i < current.size(); ++i) {
            if (!_IsUsableConstraintFrame(current[i]) ||
                !_IsUsableConstraintFrame(rest[i])) {
                return false;
            }
            GfVec3d origin = current[i].Origin();
            if (i > 0) {
                GfMatrix4d parentRest(1.0), parentPrepared(1.0);
                if (!RigExecPointsToMatrix(
                        RigExecIdentityLandmarks(),
                        lengthReference[i - 1].points,
                        &parentRest) ||
                    !RigExecPointsToMatrix(
                        RigExecIdentityLandmarks(), prepared->back().points,
                        &parentPrepared)) {
                    return false;
                }
                origin = parentPrepared.TransformAffine(
                    parentRest.GetInverse().TransformAffine(
                        lengthReference[i].Origin()));
            }

            RigExecPointFrame frame = current[i];
            frame.points[0] = origin;
            for (size_t axis = 1; axis < frame.points.size(); ++axis) {
                GfVec3d direction =
                    current[i].points[axis] - current[i].Origin();
                const double directionLength = direction.GetLength();
                const double length =
                    (lengthReference[i].points[axis] -
                     lengthReference[i].Origin()).GetLength();
                if (!std::isfinite(length) || length <= 0.0 ||
                    !std::isfinite(directionLength) ||
                    directionLength <= 0.0) {
                    return false;
                }
                direction /= directionLength;
                frame.points[axis] = origin + direction * length;
            }
            if (!_IsUsableConstraintFrame(frame)) {
                return false;
            }
            prepared->push_back(frame);
        }
        return true;
    };

    for (const _FrameConstraint &constraint : _frameConstraints) {
        const UsdPrim prim = _stage->GetPrimAtPath(constraint.moverPath);
        if (!prim) {
            continue;
        }
        const bool enabled = _ResolvedRead(
            _resolvedInputs, prim, "inputs:enabled", true, time);
        if (!enabled) {
            for (const SdfPath &target : constraint.targets) {
                recordFrame(target, constraint.moverPath);
            }
            continue;
        }
        double weight = 1.0;
        if (!constraint.weightObject.IsEmpty() &&
            constraint.pointsTarget.IsEmpty()) {
            std::vector<float> resolvedWeight;
            std::string error;
            if (!_ResolveWeights(constraint.weightObject, 1, time,
                                 &resolvedWeight, &error) ||
                resolvedWeight.size() != 1) {
                pose.diagnostics.push_back(
                    constraint.moverPath.GetString() + ": " + error +
                    "; constraint passed through");
                for (const SdfPath &target : constraint.targets) {
                    recordFrame(target, constraint.moverPath);
                }
                continue;
            }
            weight = resolvedWeight[0];
        } else if (constraint.weightObject.IsEmpty()) {
            weight = _ResolvedRead(
                _resolvedInputs, prim, "inputs:defaultWeight", 1.0f, time);
            if (!std::isfinite(weight) || weight < 0.0 || weight > 1.0) {
                pose.diagnostics.push_back(
                    constraint.moverPath.GetString() +
                    " has inputs:defaultWeight outside finite [0, 1]; "
                    "constraint passed through");
                for (const SdfPath &target : constraint.targets) {
                    recordFrame(target, constraint.moverPath);
                }
                continue;
            }
        }
        // A zero/negative envelope is an exact dormant pass-through. Do this
        // before resolving sources, effectors, or poles so malformed
        // disconnected inputs cannot make a disabled constraint fail.
        //
        // A geometry-domain object is per point and resolves after the solve,
        // so it cannot short-circuit here. A transform object has already
        // resolved its one element and may use the ordinary dormant path.
        if (weight <= 0.0 &&
            (constraint.pointsTarget.IsEmpty() ||
             constraint.weightObject.IsEmpty())) {
            for (const SdfPath &target : constraint.targets) {
                recordFrame(target, constraint.moverPath);
            }
            continue;
        }

        // The envelope is applied exactly ONCE. In the transform domain the
        // kernel's per-channel blend carries it. In the geometry domain the
        // per-point lerp does, so the solve must run UNWEIGHTED and hand back
        // the full-strength delta -- passing the envelope to both would
        // square it, and 0.5 would come out as 0.25 on points.
        const double solveWeight =
            constraint.pointsTarget.IsEmpty() ? weight : 1.0;

        if (constraint.schemaType == "RigExecSingleChainIkConstraint") {
            std::vector<RigExecPointFrame> chain;
            chain.reserve(constraint.ikChain.size());
            bool inputsValid = true;
            for (const SdfPath &joint : constraint.ikChain) {
                const auto frame = finalFrames.find(joint);
                if (frame == finalFrames.end()) {
                    pose.diagnostics.push_back(
                        constraint.moverPath.GetString() +
                        " has no current frame for " + joint.GetString());
                    inputsValid = false;
                    break;
                }
                chain.push_back(frame->second);
            }
            RigExecPointFrame effector;
            if (inputsValid &&
                !resolveBinding(constraint.effector, &effector)) {
                pose.diagnostics.push_back(
                    constraint.moverPath.GetString() +
                    " could not resolve its effector; constraint passed "
                    "through");
                inputsValid = false;
            }

            RigExecSingleChainIkParams params;
            TfToken solverMode("rotatePlane");
            if (const UsdAttribute a =
                    prim.GetAttribute(TfToken("rigExec:solverMode"))) {
                a.Get(&solverMode);
            }
            params.mode = solverMode == "singleChain"
                ? RigExecSingleChainIkMode::SingleChain
                : RigExecSingleChainIkMode::RotatePlane;
            TfToken poleMode("vector");
            if (const UsdAttribute a =
                    prim.GetAttribute(TfToken("rigExec:poleVectorMode"))) {
                a.Get(&poleMode);
            }
            params.weight = weight;
            if (params.mode == RigExecSingleChainIkMode::RotatePlane) {
                params.pole = _ResolvedRead(
                    _resolvedInputs, prim, "inputs:poleVector",
                    GfVec3d(0, 1, 0), time);
                params.twistDegrees = _ResolvedRead(
                    _resolvedInputs, prim, "inputs:twistDegrees", 0.0,
                    time);
            }
            if (inputsValid &&
                params.mode == RigExecSingleChainIkMode::RotatePlane &&
                poleMode == "object") {
                if (constraint.poleObjects.empty()) {
                    pose.diagnostics.push_back(
                        constraint.moverPath.GetString() +
                        " uses object pole mode with no pole-vector objects; "
                        "constraint passed through");
                    inputsValid = false;
                }
                std::vector<double> poleWeights;
                if (inputsValid &&
                    !readWeights(prim, "inputs:poleVectorWeights",
                                 constraint.poleObjects.size(), &poleWeights)) {
                    inputsValid = false;
                }
                GfVec3d polePoint(0);
                double total = 0;
                for (size_t i = 0;
                     inputsValid && i < constraint.poleObjects.size(); ++i) {
                    RigExecPointFrame poleFrame;
                    if (!resolveBinding(constraint.poleObjects[i],
                                        &poleFrame) ||
                        !std::isfinite(poleWeights[i]) ||
                        poleWeights[i] < 0) {
                        pose.diagnostics.push_back(
                            constraint.moverPath.GetString() +
                            " has an invalid pole-vector source or weight; "
                            "constraint passed through");
                        inputsValid = false;
                        break;
                    }
                    polePoint += poleFrame.Origin() * poleWeights[i];
                    total += poleWeights[i];
                }
                if (inputsValid && total <= 0) {
                    pose.diagnostics.push_back(
                        constraint.moverPath.GetString() +
                        " has zero total pole-vector weight; constraint passed "
                        "through");
                    inputsValid = false;
                }
                if (inputsValid) {
                    params.pole = polePoint / total;
                }
            }

            TfToken evaluationMode("neverTS");
            if (const UsdAttribute a =
                    prim.GetAttribute(TfToken("rigExec:evaluationMode"))) {
                a.Get(&evaluationMode);
            }
            std::vector<RigExecPointFrame> solveChain = chain;
            const bool useAnimatedTs =
                evaluationMode == "alwaysTS" ||
                (evaluationMode == "autoDetect" &&
                 ikUsesAnimatedTs(constraint.ikChain));
            if (inputsValid && !useAnimatedTs) {
                std::vector<RigExecPointFrame> rest;
                rest.reserve(constraint.ikChain.size());
                for (const SdfPath &joint : constraint.ikChain) {
                    const auto frame = restFrames.find(joint);
                    if (frame == restFrames.end()) {
                        inputsValid = false;
                        break;
                    }
                    rest.push_back(frame->second);
                }
                if (!inputsValid ||
                    !ikWithoutAnimatedTs(chain, rest, &solveChain)) {
                    pose.diagnostics.push_back(
                        constraint.moverPath.GetString() +
                        " could not prepare rest-derived IK inputs; "
                        "constraint passed through");
                    inputsValid = false;
                }
            }

            std::vector<RigExecPointFrame> solved;
            if (inputsValid) {
                solved = RigExecSolveSingleChainIk(
                    solveChain, effector, params);
            }
            if (inputsValid &&
                (solved.size() != constraint.ikChain.size() ||
                 std::any_of(
                     solved.begin(), solved.end(),
                     [](const RigExecPointFrame &frame) {
                         return !_IsUsableConstraintFrame(frame);
                     }))) {
                pose.diagnostics.push_back(
                    constraint.moverPath.GetString() +
                    " failed to solve its joint chain; constraint passed "
                    "through atomically");
                inputsValid = false;
            }
            if (inputsValid) {
                std::map<SdfPath, RigExecPointFrame> candidates;
                for (size_t i = 0; i < solved.size(); ++i) {
                    candidates[constraint.ikChain[i]] = solved[i];
                }
                commitConstraintFrames(constraint.moverPath, candidates);
            }
            for (size_t i = 0; i < constraint.ikChain.size(); ++i) {
                recordFrame(constraint.ikChain[i], constraint.moverPath);
            }
            continue;
        }

        std::vector<RigExecConstraintSource> sources;
        if (!buildSources(constraint, prim, &sources)) {
            pose.diagnostics.push_back(
                constraint.moverPath.GetString() +
                " has unusable constraint inputs; constraint passed through");
            recordFrame(constraint.targets[0], constraint.moverPath);
            continue;
        }
        const RigExecPointFrame inputFrame =
            finalFrames[constraint.targets[0]];
        RigExecPointFrame candidate = inputFrame;
        bool candidateReady = true;
        // Which mask triple this operator reads is a table property: masks
        // are addressed by (group, axis), so Position reads the translation
        // triple and Rotation and Aim read the rotation one, rather than all
        // three sharing an inputs:affectX that means something different in
        // each.
        const _ConstraintHandler *solveHandler =
            _FindConstraintHandler(constraint.schemaType);
        const RigExecConstraintAxisMask affect = _ReadGroupMask(
            _resolvedInputs, prim,
            solveHandler ? solveHandler->maskGroup : _ChannelGroup::None,
            time);
        TfToken orderToken("XYZ");
        if (const UsdAttribute a =
                prim.GetAttribute(TfToken("rigExec:rotationOrder"))) {
            a.Get(&orderToken);
        }
        const RigExecEulerOrder order =
            _ParseConstraintEulerOrder(orderToken);

        // The kernel-backed operators solve through the registry: one row
        // per operator, so adding an operator is a table entry rather than
        // another arm here. Aim falls through to the inline branch below,
        // which resolves a world-up binding the uniform context cannot carry.
        if (solveHandler && solveHandler->solve) {
            _ConstraintSolveContext solveContext;
            solveContext.resolved = &_resolvedInputs;
            solveContext.prim = prim;
            solveContext.time = time;
            solveContext.inputFrame = inputFrame;
            solveContext.sources = &sources;
            solveContext.affect = affect;
            solveContext.order = order;
            solveContext.weight = solveWeight;
            candidate = solveHandler->solve(solveContext);
        } else {
            // Aim uses the same weighted source set, reduced to the target
            // point specified by FBX's AimAtObjects contract.
            GfVec3d target(0);
            double total = 0;
            for (const RigExecConstraintSource &source : sources) {
                if (!std::isfinite(source.normalizedWeight) ||
                    source.normalizedWeight < 0) {
                    pose.diagnostics.push_back(
                        constraint.moverPath.GetString() +
                        " has an invalid source weight; constraint passed "
                        "through");
                    candidateReady = false;
                    break;
                }
                target += source.frame.Origin() * source.normalizedWeight;
                total += source.normalizedWeight;
            }
            if (candidateReady && total > 0) {
                target /= total;
                RigExecAimConstraintParams params;
                const UsdAttribute aimVectorAttr =
                    prim.GetAttribute(TfToken("inputs:aimVector"));
                params.localAimVector = _ResolvedRead(
                    _resolvedInputs, prim, "inputs:aimVector",
                    GfVec3d(1, 0, 0), time);
                // Existing assets author aimAxis but predate aimVector. Keep
                // that authored meaning until they opt into the vector form.
                if (!aimVectorAttr ||
                    !aimVectorAttr.HasAuthoredValueOpinion()) {
                    TfToken axis("x");
                    if (const UsdAttribute a = prim.GetAttribute(
                            TfToken("rigExec:aimAxis"))) {
                        a.Get(&axis);
                    }
                    params.localAimVector =
                        axis == "y" ? GfVec3d(0, 1, 0)
                                    : axis == "z" ? GfVec3d(0, 0, 1)
                                                  : GfVec3d(1, 0, 0);
                }
                params.localUpVector = _ResolvedRead(
                    _resolvedInputs, prim, "inputs:upVector",
                    GfVec3d(0, 1, 0), time);
                params.rotationOffsetDegrees = _ResolvedRead(
                    _resolvedInputs, prim, "inputs:rotationOffset",
                    GfVec3d(0), time);
                params.affectRotation = affect;
                params.rotationOrder = order;
                params.weight = solveWeight;

                TfToken worldUpType("none");
                if (const UsdAttribute a = prim.GetAttribute(
                        TfToken("rigExec:worldUpType"))) {
                    a.Get(&worldUpType);
                }
                SdfPathVector authoredSources;
                if (const UsdRelationship rel = prim.GetRelationship(
                        TfToken("rigExec:sources"))) {
                    rel.GetTargets(&authoredSources);
                }
                // The legacy aimTarget/aimAxis contract preserves input up.
                // FBX WorldUpType=None is the distinct minimum-swing mode.
                params.preserveInputUp = authoredSources.empty();
                const GfVec3d authoredWorldUp = _ResolvedRead(
                    _resolvedInputs, prim, "inputs:worldUpVector",
                    GfVec3d(0, 1, 0), time);
                if (worldUpType == "sceneUp") {
                    const std::string up =
                        UsdGeomGetStageUpAxis(_stage).GetString();
                    params.worldUpDirection =
                        (up == "Z" || up == "z")
                            ? GfVec3d(0, 0, 1)
                            : GfVec3d(0, 1, 0);
                } else if (worldUpType == "vector") {
                    params.worldUpDirection = authoredWorldUp;
                } else if (worldUpType == "objectUp") {
                    // FBX ObjectUp without a reference object uses the
                    // world origin as the object point.
                    if (constraint.worldUpObject.sourcePath.IsEmpty()) {
                        params.worldUpDirection = -inputFrame.Origin();
                    } else {
                        RigExecPointFrame upObject;
                        if (!resolveBinding(constraint.worldUpObject,
                                            &upObject)) {
                            pose.diagnostics.push_back(
                                constraint.moverPath.GetString() +
                                " could not resolve its world-up object; "
                                "constraint passed through");
                            candidateReady = false;
                        } else {
                            params.worldUpDirection =
                                upObject.Origin() - inputFrame.Origin();
                        }
                    }
                } else if (worldUpType == "objectRotationUp") {
                    // With no object, FBX applies WorldUpVector directly in
                    // world space rather than treating a missing binding as
                    // a failed constraint.
                    if (constraint.worldUpObject.sourcePath.IsEmpty()) {
                        params.worldUpDirection = authoredWorldUp;
                    } else {
                        RigExecPointFrame upObject;
                        if (!resolveBinding(constraint.worldUpObject,
                                            &upObject)) {
                            pose.diagnostics.push_back(
                                constraint.moverPath.GetString() +
                                " could not resolve its world-up object; "
                                "constraint passed through");
                            candidateReady = false;
                        }
                        if (candidateReady) {
                            GfMatrix4d upMatrix(1.0);
                            if (!RigExecPointsToMatrix(
                                    RigExecIdentityLandmarks(),
                                    upObject.points, &upMatrix)) {
                                pose.diagnostics.push_back(
                                    constraint.moverPath.GetString() +
                                    " has a degenerate world-up object");
                                candidateReady = false;
                            } else {
                                params.worldUpDirection =
                                    upMatrix.ExtractRotation().TransformDir(
                                        authoredWorldUp);
                            }
                        }
                    }
                }
                if (candidateReady) {
                    candidate = RigExecApplyAimConstraint(
                        inputFrame, target, params);
                }
            }
        }
        if (candidateReady && !constraint.pointsTarget.IsEmpty()) {
            // The GEOMETRY domain. The solve produced the same full-strength
            // frame the transform domain would publish; the delta against the
            // prim's own base frame is what the points ride.
            //
            //     D = F_solved * F_base^-1
            //
            // Stashed here and consumed after the pose walk, the same
            // in-memory hand-off finalMatrices performs for a "final" read
            // phase. The prim's transform is NOT revised: a geometry-domain
            // constraint writes points and nothing else.
            GfMatrix4d baseMatrix(1.0);
            GfMatrix4d solvedMatrix(1.0);
            if (frameFromXform(constraint.targets[0], nullptr, &baseMatrix) &&
                RigExecPointsToMatrix(RigExecIdentityLandmarks(),
                                      candidate.points, &solvedMatrix)) {
                constraintDeltas[constraint.pointsTarget] =
                    baseMatrix.GetInverse() * solvedMatrix;
                constraintEnvelopes[constraint.pointsTarget] = weight;
                constraintWeightObjects[constraint.pointsTarget] =
                    constraint.weightObject;
            } else {
                pose.diagnostics.push_back(
                    constraint.moverPath.GetString() +
                    " could not measure its delta against " +
                    constraint.targets[0].GetString() +
                    "; constraint passed through");
            }
        } else if (candidateReady) {
            commitConstraintFrames(
                constraint.moverPath,
                {{constraint.targets[0], candidate}});
        }
        if (constraint.pointsTarget.IsEmpty()) {
            recordFrame(constraint.targets[0], constraint.moverPath);
        }
    }

    // Publish final provider matrices after the atomic pose walk.
    for (const auto &[provider, frame] : finalFrames) {
        if (_xformDerivedProviders.count(provider)) {
            if (!_IsUsableConstraintFrame(frame)) {
                pose.diagnostics.push_back(
                    "constraint target " + provider.GetString() +
                    " has an invalid final frame; transform omitted");
                continue;
            }
            GfMatrix4d revised(1.0);
            if (RigExecPointsToMatrix(RigExecIdentityLandmarks(),
                                      frame.points, &revised)) {
                pose.providerXforms[provider] = revised;
                pose.providerBaseXforms[provider] =
                    xformDerivedBases[provider];
            }
        }
        const auto rest = restFrames.find(provider);
        if (rest != restFrames.end() &&
            _IsUsableConstraintFrame(rest->second) &&
            _IsUsableConstraintFrame(frame)) {
            GfMatrix4d matrix(1.0);
            if (RigExecPointsToMatrix(rest->second.points, frame.points,
                                      &matrix)) {
                finalMatrices[provider] = matrix;
                _chainSnapshots.RecordFinal(provider, VtValue(matrix));
            }
        }
    }

    // 3. Base and final transform revisions plus paired matrices. A
    // solver-posed joint's base value was supplied as an override above, so
    // exec published it as that joint's computePointFrame; the final value is
    // the in-memory frame revision when the joint carries one, and otherwise
    // is the base (which is what the final-phase tap already resolves to).
    for (size_t i = 0; i < _jointPaths.size(); ++i) {
        const RigExecPointFrame baseFrame =
            snapshot.Get<RigExecPointFrame>(_jointFrameTaps[i]);
        const auto revisedIt = finalFrames.find(_jointPaths[i]);
        const RigExecPointFrame finalFrame =
            revisedIt != finalFrames.end()
                ? revisedIt->second
                : snapshot.Get<RigExecPointFrame>(_jointFinalFrameTaps[i]);
        pose.jointFramesBase[_jointPaths[i]] = baseFrame;
        pose.jointFramesFinal[_jointPaths[i]] = finalFrame;
        // The point frame is the status bearer; the matrix result carries
        // no status and _ComputeJointMatrix returns identity for a
        // degenerate/invalid frame. Publishing that identity would let a
        // matrix-only consumer deform with a plausible-but-wrong transform.
        // Omit the matrix and diagnose so absence — not a
        // false identity — signals the failure; consumers already handle a
        // missing jointMatricesFinal entry. The degenerate frame is still
        // published so imaging can omit its guide.
        if (finalFrame.IsValid() && !finalFrame.IsDegenerate()) {
            auto revisedMatrix = finalMatrices.find(_jointPaths[i]);
            if (revisedMatrix == finalMatrices.end()) {
                // Untargeted descendants carried by a constrained ancestor do
                // not have a dedicated rest-frame tap. Compose their
                // base->revised delta onto the authoritative pre-constraint
                // rest->base matrix from the snapshot.
                GfMatrix4d delta(1.0);
                if (RigExecPointsToMatrix(
                        baseFrame.points, finalFrame.points, &delta)) {
                    finalMatrices[_jointPaths[i]] =
                        snapshot.Get<GfMatrix4d>(
                            _jointFinalMatrixTaps[i]) * delta;
                    revisedMatrix = finalMatrices.find(_jointPaths[i]);
                }
            }
            pose.jointMatricesFinal[_jointPaths[i]] =
                revisedMatrix != finalMatrices.end()
                    ? revisedMatrix->second
                    : snapshot.Get<GfMatrix4d>(_jointFinalMatrixTaps[i]);
        } else {
            pose.diagnostics.push_back(
                "joint " + _jointPaths[i].GetString() +
                " has a degenerate final frame; matrix omitted");
        }
    }
    // 3a. Control frames. Most are animator-authored inputs and therefore
    // publish their base tap directly; a control explicitly named as a
    // constraint write target publishes the revised frame, matching FBX's
    // ability to constrain any transform object. A degenerate/invalid frame
    // remains the status bearer and lets imaging omit the guide.
    for (size_t i = 0; i < _controlPaths.size(); ++i) {
        const auto revised = finalFrames.find(_controlPaths[i]);
        pose.controlFrames[_controlPaths[i]] =
            revised != finalFrames.end()
                ? revised->second
                : snapshot.Get<RigExecPointFrame>(_controlFrameTaps[i]);
    }

    // Observational solver guides never gate the rig snapshot: an
    // incomplete guide evaluation degrades to a diagnostic.
    if (_guideTaps) {
        const RigExecSnapshot guideSnapshot =
            _guideTaps->Evaluate(time, baseOverrides);
        if (guideSnapshot.IsComplete()) {
            for (const auto &[solverPath, tap] : _solverArrayTaps) {
                pose.solverFrames[solverPath] =
                    guideSnapshot.Get<RigExecPointFrameArray>(tap).frames;
            }
        } else {
            pose.diagnostics.push_back(
                "solver guide taps incomplete: guides omitted this "
                "generation");
        }
    }

    // Property-domain results, computed above and already consumed by exec
    // as overrides. Published in the same map as the point chains: a
    // consumer distinguishes them by the type the VtValue holds, not by
    // which mover domain produced them.
    for (const auto &[target, value] : propertyResults) {
        pose.movedProperties[target] = value;
    }

    // The independent CPU parity path must consume the same declared
    // provider phase as the graph while resolving it independently.  Capture
    // every matrix provider the graph taps (controls as well as joints), then
    // overlay the evaluator-side frame revisions for final-phase reads.
    std::map<SdfPath, GfMatrix4d> baseProviderMatrices;
    for (const auto &[target, revisions] : _graphChains) {
        for (const _GraphRevision &revision : revisions) {
            if (revision.transformTap >= 0 &&
                !revision.binding.transform.IsEmpty() &&
                !baseProviderMatrices.count(revision.binding.transform)) {
                baseProviderMatrices[revision.binding.transform] =
                    snapshot.Get<GfMatrix4d>(revision.transformTap);
            }
        }
    }
    std::map<SdfPath, GfMatrix4d> finalProviderMatrices =
        baseProviderMatrices;
    for (const auto &[provider, matrix] : finalMatrices) {
        finalProviderMatrices[provider] = matrix;
    }

    // 3. Geometry point chains from the generated applications: the chain
    // head's passive outputs:value bridge extracts the exact native
    // VtVec3fArray (spec §7.2). Unsupported v0.1-alpha operations were
    // skipped by the compiler and pass through with diagnostics.
    // 3b. The compiled mover graph, which is the ONLY producer of geometry
    // (spec §7.2): no generated prim and no derived-stage read stands between
    // the authored mover chain and the value written to movedProperties.
    //
    // Every op reachable in a point chain has its provider values: matrix
    // (computeMatrix + computeWeightPacket), blendShape (summed
    // computeBlendChannel), ribbon / emitGuidePoints
    // (computePointFrameArray), and volumeCorrect / smooth / lattice /
    // surfaceProject, whose inputs are static reads through the binding plus
    // the authored base.
    //
    // Correctness is policed by the scalar CPU reference below, which resolves
    // its own inputs off the authored stage and authors nothing. A
    // disagreement rejects the complete pose: grepping a diagnostic string
    // is what let a packet-assembly drift reach usdview once already (see
    // docs/mover-graph-cutover.md), so a known-bad generation cannot publish.
    size_t graphChainsBuilt = 0;
    size_t graphRevisionsBuilt = 0;

    // Chains run in dependency order, computed at compile (_chainOrder).
    //
    // This used to be "curvenets first, then everything else", which was the
    // only cross-chain dependency that existed: a Profile Mover needs its
    // net's own chain already evaluated. A phased read is the same shape of
    // dependency stated in general -- a cage read at `final` needs the cage's
    // chain first, exactly as the net does -- so the special case became one
    // edge in a topological sort and the heuristic went away.
    for (const SdfPath &target : _chainOrder) {
        const auto chainIt = _graphChains.find(target);
        if (chainIt == _graphChains.end()) {
            continue;
        }
        const std::vector<_GraphRevision> &revisions = chainIt->second;
        VtVec3fArray basePoints;
        const UsdAttribute baseAttr = _stage->GetAttributeAtPath(target);
        if (!baseAttr || !baseAttr.Get(&basePoints, time)) {
            continue;
        }

        RigExecMoverGraph graph;
        VdfMaskedOutput head = graph.AddPointSource(target, basePoints);
        bool built = true;
        for (const _GraphRevision &revision : revisions) {
            const UsdPrim moverPrim =
                _stage->GetPrimAtPath(revision.moverPath);
            if (!moverPrim) {
                built = false;
                break;
            }
            RigExecProviderValues values;
            // One overlay per revision: the generation-wide property results,
            // plus whatever THIS revision's declared phases resolve to. The
            // assembler reads inputs by path and never learns a phase exists
            // -- which is what lets a phase apply to any input, including
            // ones added later, without touching the assembler.
            RigExecResolvedInputs revisionInputs = _resolvedInputs;
            for (const auto &[inputPath, phase] : revision.binding.phases) {
                if (const VtValue *v = _chainSnapshots.Lookup(
                        inputPath, phase, revision.moverPath)) {
                    revisionInputs.SetProperty(inputPath, *v);
                } else if (phase.kind != RigExecReadPhaseKind::Preceding) {
                    // Preceding falling through to the stage is correct (the
                    // reader is the chain's first revision, so its preceding
                    // value IS the base). Anything else means the phase named
                    // something that produced nothing.
                    pose.diagnostics.push_back(
                        "diag " + revision.moverPath.GetString() +
                        ": read phase '" + phase.GetAsString() + "' for " +
                        inputPath.GetString() +
                        " resolved to nothing; read the authored base");
                }
            }
            values.resolved = &revisionInputs;
            GfMatrix4d transform(1.0);
            RigExecWeightPacket weights;
            RigExecPointFrameArray driverFrames;
            if (revision.driverFramesTap >= 0) {
                driverFrames = snapshot.Get<RigExecPointFrameArray>(
                    revision.driverFramesTap);
                values.driverFrames = &driverFrames;
            }
            if (revision.transformTap >= 0) {
                transform = snapshot.Get<GfMatrix4d>(revision.transformTap);
                values.transform = &transform;
            }
            // A "final" read phase takes the provider's aim-revised matrix.
            // That used to be expressed by binding the generated frame-chain
            // head as the transform provider; now the binding names the
            // provider itself and the revised value is substituted here.
            if (revision.transformFinalPhase) {
                const auto revisedIt =
                    finalMatrices.find(revision.binding.transform);
                if (revisedIt != finalMatrices.end()) {
                    transform = revisedIt->second;
                    values.transform = &transform;
                }
            } else if (revision.binding.transformPhase.kind ==
                       RigExecReadPhaseKind::AtPrim) {
                // The provider's frame as of a named point in the pose walk,
                // rather than its base or its final. Same store the point
                // chains use; the value here is a matrix instead of an array.
                if (const VtValue *v = _chainSnapshots.Lookup(
                        revision.binding.transform,
                        revision.binding.transformPhase,
                        revision.moverPath)) {
                    if (v->IsHolding<GfMatrix4d>()) {
                        transform = v->UncheckedGet<GfMatrix4d>();
                        values.transform = &transform;
                    }
                }
            }
            if (revision.weightTap >= 0) {
                weights =
                    snapshot.Get<RigExecWeightPacket>(revision.weightTap);
                values.weights = &weights;

                // rigExec:samplePhase = "current": the field is measured
                // against the points AS THEY STAND HERE, not the
                // authored base, so the volume grabs whatever is inside
                // it right now.
                //
                // It cannot come from exec. The revision node's only
                // inputs are its parameters, its status, and the
                // read-write point buffer, and the parameters are baked
                // as a VDF constant when the graph is built -- nothing
                // in the packet can depend on a value the graph has not
                // computed yet. What CAN be done is evaluate the chain
                // built SO FAR (RigExecMoverGraph::Evaluate is const and
                // takes any masked output), measure against that, and
                // bake the result into this revision's parameters. One
                // extra graph evaluation per current-phase revision,
                // paid only by rigs that ask for it.
                if (_currentPhaseWeights.count(
                        revision.binding.weightObject)) {
                    const VtVec3fArray inFlight = graph.Evaluate(head);
                    const std::vector<GfVec3f> currentPoints(
                        inFlight.begin(), inFlight.end());
                    std::vector<float> field;
                    std::string weightError;
                    if (_ResolveWeights(revision.binding.weightObject,
                                        currentPoints.size(), time, &field,
                                        &weightError, &currentPoints)) {
                        weights.representation = TfToken("dense");
                        weights.values = std::move(field);
                        weights.indices.clear();
                        weights.defaultWeight = 0.0f;
                        weights.valid = true;
                    } else {
                        // An invalid packet is the kernel's atomic
                        // MoverFailed pass-through, which is the right
                        // answer here: publishing the reference-phase
                        // field instead would silently be a different
                        // deformation.
                        weights = RigExecWeightPacket();
                        pose.diagnostics.push_back(
                            "current-phase weight failed: " + weightError);
                    }
                }
            }
            // Publish the field an authoring tool paints as an influence
            // overlay. Taken from the packet the mover is about to
            // consume, so what a rigger sees is exactly what deformed
            // the geometry -- not a re-derivation that could drift.
            if (revision.weightTap >= 0 && weights.valid) {
                RigExecResolvedWeightField &field =
                    pose.weightFields[revision.binding.weightObject];
                SdfPathVector declaredTargets;
                if (const UsdPrim weightPrim = _stage->GetPrimAtPath(
                        revision.binding.weightObject)) {
                    if (const UsdRelationship rel =
                            weightPrim.GetRelationship(
                                TfToken("rigExec:weightTarget"))) {
                        rel.GetTargets(&declaredTargets);
                    }
                }
                const bool operationDomain =
                    declaredTargets.size() == 1 &&
                    declaredTargets[0] == revision.moverPath;
                field.target = operationDomain ? revision.moverPath : target;
                const size_t logicalCount =
                    operationDomain ? size_t(1) : basePoints.size();
                field.weights.assign(logicalCount, 0.0f);
                for (size_t i = 0; i < logicalCount; ++i) {
                    const float w = weights.Resolve(i, logicalCount);
                    field.weights[i] = w < 0.0f ? 0.0f : w;
                }
            }
            values.basePoints.assign(basePoints.begin(), basePoints.end());
            if (revision.op == RigExecRevisionOp::Curvenet) {
                values.curvenetCache = &_curvenetBindings;
                // The curvenet's own chain result if it has one; pass 0 above
                // guarantees it has already been computed.
                const auto posed =
                    pose.movedProperties.find(revision.binding.curvenetPoints);
                if (posed != pose.movedProperties.end() &&
                    posed->second.IsHolding<VtVec3fArray>()) {
                    const VtVec3fArray &net =
                        posed->second.UncheckedGet<VtVec3fArray>();
                    values.curvenetPoints.assign(net.begin(), net.end());
                }
            }
            if (!revision.blendChannelTaps.empty()) {
                std::vector<RigExecBlendChannel> channels;
                channels.reserve(revision.blendChannelTaps.size());
                for (const RigExecTapId tap : revision.blendChannelTaps) {
                    channels.push_back(
                        snapshot.Get<RigExecBlendChannel>(tap));
                }
                // A structural failure leaves blendDeltas empty, which is
                // what makes the assembled packet invalid -- the same atomic
                // MoverFailed pass-through the kernel produces.
                if (!RigExecSumBlendChannels(channels, values.basePoints,
                                             &values.blendDeltas)) {
                    values.blendDeltas.clear();
                }
            }

            const RigExecMoverParameters parameters =
                RigExecAssembleParameters(moverPrim, revision.op,
                                          revision.binding, values, time);
            if (parameters.enabled && !parameters.valid &&
                revision.binding.weightObject.IsEmpty()) {
                const float scalar = _ResolvedRead(
                    _resolvedInputs, moverPrim, "inputs:defaultWeight",
                    1.0f, time);
                if (!std::isfinite(scalar) || scalar < 0.0f ||
                    scalar > 1.0f) {
                    pose.diagnostics.push_back(
                        "MoverFailed " + revision.moverPath.GetString() +
                        ": inputs:defaultWeight must be finite and in "
                        "[0, 1]; revision passed through");
                }
            } else if (parameters.enabled && !parameters.valid &&
                       !revision.binding.weightObject.IsEmpty() &&
                       (!values.weights || !values.weights->valid)) {
                pose.diagnostics.push_back(
                    "MoverFailed " + revision.moverPath.GetString() +
                    ": rigExec:weightObject produced an invalid common "
                    "envelope; revision passed through");
            }
            head = graph.AddRevision(
                revision.op, head, parameters,
                RigExecStatusForParameters(parameters, revision.moverPath));
            ++graphRevisionsBuilt;

            // Snapshot only where a phased read named this revision. The
            // compile pass reduced every phase to one revision, so this is
            // the whole cost of the feature for a rig that uses it, and
            // nothing at all for one that does not.
            const auto wanted = _snapshotPoints.find(target);
            if (wanted != _snapshotPoints.end() &&
                wanted->second.count(revision.moverPath)) {
                _chainSnapshots.Record(target, revision.moverPath,
                                       VtValue(graph.Evaluate(head)));
            }
        }
        if (!built) {
            continue;
        }

        const VtVec3fArray graphPoints = graph.Evaluate(head);
        pose.movedProperties[target] = VtValue(graphPoints);
        // `final` costs nothing extra: this is the value the chain publishes.
        _chainSnapshots.RecordFinal(target, VtValue(graphPoints));
        ++graphChainsBuilt;

        // Derived maintenance reads this chain's final points, which is why it
        // runs here rather than as another entry in _graphChains.
        const auto derivedIt = _graphDerivedChains.find(target);
        if (derivedIt == _graphDerivedChains.end()) {
            continue;
        }
        for (const _GraphRevision &derived : derivedIt->second) {
            VtVec3fArray derivedBase;
            const UsdAttribute derivedAttr =
                _stage->GetAttributeAtPath(derived.target);
            if (!derivedAttr || !derivedAttr.Get(&derivedBase, time)) {
                continue;
            }
            RigExecProviderValues values;
            values.resolved = &_resolvedInputs;
            values.basePoints.assign(graphPoints.begin(), graphPoints.end());

            RigExecMoverGraph derivedGraph;
            const VdfMaskedOutput derivedSource =
                derivedGraph.AddPointSource(derived.target, derivedBase);
            const RigExecMoverParameters parameters =
                RigExecAssembleParameters(
                    _stage->GetPrimAtPath(derived.moverPath), derived.op,
                    derived.binding, values, time);
            const VdfMaskedOutput derivedHead = derivedGraph.AddRevision(
                derived.op, derivedSource, parameters,
                RigExecStatusForParameters(parameters, derived.moverPath));
            ++graphRevisionsBuilt;

            const VtVec3fArray derivedResult =
                derivedGraph.Evaluate(derivedHead);
            pose.movedProperties[derived.target] = VtValue(derivedResult);
            ++graphChainsBuilt;
        }
    }
    // The same chains against the scalar CPU reference (spec §7.4). This is
    // the oracle that OUTLIVES the generated-prim chains: it resolves every
    // input off the authored stage itself and authors nothing, so it survives
    // the compiler's deletion, and it is a genuinely independent
    // implementation -- it does not call RigExecAssembleParameters, which is
    // why it can catch a packet-assembly drift rather than share one.
    size_t parityAgreements = 0;
    {
        std::map<SdfPath, std::vector<const RigExecMoverRecord *>> chains;
        for (const RigExecMoverRecord &mover : _movers) {
            for (const SdfPath &target : mover.targets) {
                if (target.IsPropertyPath() &&
                    target.GetNameToken() == "points") {
                    chains[target].push_back(&mover);
                }
            }
        }
        for (const auto &[target, chain] : chains) {
            const auto graphIt = pose.movedProperties.find(target);
            if (graphIt == pose.movedProperties.end() ||
                !graphIt->second.IsHolding<VtVec3fArray>()) {
                continue;
            }
            // A Profile Mover has no scalar oracle (see _EvaluateChain), so
            // comparing against one reports a "mismatch" that means only
            // "the reference does not implement this". Skipped and said,
            // rather than counted as a defect.
            bool hasCurvenet = false;
            for (const RigExecMoverRecord *mover : chain) {
                if (mover->schemaType == "RigExecCurvenetMover") {
                    hasCurvenet = true;
                    break;
                }
            }
            if (hasCurvenet) {
                pose.diagnostics.push_back(
                    "cpu reference parity: " + target.GetString() +
                    " skipped, its chain contains a RigExecCurvenetMover");
                continue;
            }
            std::vector<std::string> quiet;
            const VtVec3fArray reference =
                _EvaluateChain(
                    target, chain, pose, baseProviderMatrices,
                    finalProviderMatrices, time, &quiet);
            const VtVec3fArray &graphValue =
                graphIt->second.UncheckedGet<VtVec3fArray>();
            if (reference.size() != graphValue.size()) {
                pose.diagnostics.push_back(
                    "cpu reference parity: size mismatch on " +
                    target.GetString());
                ++pose.moverGraphParityMismatches;
                continue;
            }
            bool agrees = true;
            for (size_t i = 0; i < reference.size(); ++i) {
                if (!GfIsClose(reference[i], graphValue[i], 1e-4)) {
                    pose.diagnostics.push_back(
                        "cpu reference parity: value mismatch on " +
                        target.GetString() + " at element " +
                        std::to_string(i));
                    ++pose.moverGraphParityMismatches;
                    agrees = false;
                    break;
                }
            }
            if (agrees) {
                ++parityAgreements;
            }
        }
    }

    // Whatever the Profile Mover binds reported. Emitted here rather than
    // from inside packet assembly because that has no diagnostic channel,
    // and drained so a cached bind stays silent on every later frame.
    for (std::string &message : _curvenetBindings.TakeDiagnostics()) {
        pose.diagnostics.push_back(std::move(message));
    }

    // Report actual reference comparisons, not every graph/derived chain that
    // happened to build.  Derived normals/extents are not part of this scalar
    // point-chain oracle and a mismatched point chain is never an agreement.
    pose.moverGraphParityAgreements = parityAgreements;
    pose.diagnostics.push_back(
        "mover graph parity: " + std::to_string(parityAgreements) +
        " point chain(s) agreed, " +
        std::to_string(pose.moverGraphParityMismatches) +
        " mismatched; " + std::to_string(graphChainsBuilt) +
        " graph/derived chain(s) built over " +
        std::to_string(graphRevisionsBuilt) + " revision(s)");
    if (pose.moverGraphParityMismatches != 0) {
        pose.diagnostics.push_back(
            "mover graph parity failed; refusing to publish generation");
        return pose;  // pose.valid remains false
    }
    // Optional CPU reference-kernel parity for the lowered chains
    // (scalar-reference goldens, spec §7.4).
    if (cpuParityMode) {
        std::map<SdfPath, std::vector<const RigExecMoverRecord *>> chains;
        for (const RigExecMoverRecord &mover : _movers) {
            for (const SdfPath &target : mover.targets) {
                if (target.IsPropertyPath() &&
                    target.GetNameToken() == "points") {
                    chains[target].push_back(&mover);
                }
            }
        }
        for (const auto &[target, chain] : chains) {
            // A chain containing a Profile Mover has no scalar oracle (see
            // _EvaluateChain), so publishing a CPU value for it would report
            // a "mismatch" that means only "the reference does not implement
            // this". Skip the chain and say so instead.
            bool hasCurvenet = false;
            for (const RigExecMoverRecord *mover : chain) {
                if (mover->schemaType == "RigExecCurvenetMover") {
                    hasCurvenet = true;
                    break;
                }
            }
            if (hasCurvenet) {
                pose.diagnostics.push_back(
                    "cpu parity: " + target.GetString() +
                    " skipped, its chain contains a RigExecCurvenetMover");
                continue;
            }
            if (constraintDeltas.count(target)) {
                // A geometry-domain constraint is not a graph chain; it is
                // published below from its own delta, and its parity twin
                // runs there.
                continue;
            }
            pose.movedPropertiesCpu[target] = VtValue(_EvaluateChain(
                target, chain, pose, baseProviderMatrices,
                finalProviderMatrices, time, &pose.diagnostics));
        }
    }

    // Geometry-domain constraints. The delta is the same one the transform
    // domain would have published; here it rides the points instead, under
    // the constraint's per-element weight field.
    //
    //     P'[i] = lerp(P[i], D.TransformAffine(P[i]), w[i])
    //
    // w[i] comes from a bound rigExec:weightObject, or from a constant
    // synthesized from inputs:defaultWeight when none is bound. The envelope
    // is applied HERE and only here: the solve above ran unweighted.
    for (const auto &[pointsTarget, delta] : constraintDeltas) {
        const UsdAttribute attr = _stage->GetAttributeAtPath(pointsTarget);
        VtVec3fArray authored;
        if (!attr || !attr.Get(&authored, time)) {
            pose.diagnostics.push_back(
                pointsTarget.GetString() +
                " has no readable points; constraint passed through");
            continue;
        }
        const double envelope = constraintEnvelopes[pointsTarget];
        const SdfPath weightObject = constraintWeightObjects[pointsTarget];

        std::vector<float> weights(authored.size(),
                                   static_cast<float>(envelope));
        if (!weightObject.IsEmpty()) {
            // A bound field SUPERSEDES the envelope: it is the weight, not a
            // multiplier on it. Its cardinality must match the point set --
            // a mismatch is MoverFailed for the whole target, never a
            // silently truncated deformation.
            std::string error;
            std::vector<GfVec3f> currentPoints(authored.begin(),
                                               authored.end());
            if (!_ResolveWeights(weightObject, authored.size(), time,
                                 &weights, &error, &currentPoints)) {
                pose.diagnostics.push_back(
                    pointsTarget.GetString() + ": " + error +
                    "; constraint passed through");
                continue;
            }
        }

        VtVec3fArray moved(authored.size());
        for (size_t i = 0; i < authored.size(); ++i) {
            const float w = weights[i];
            if (!std::isfinite(w)) {
                moved = VtVec3fArray();
                break;
            }
            moved[i] = GfVec3f(RigExecApplyWeightedMatrix(
                GfVec3d(authored[i]), delta, w));
        }
        if (moved.empty() && !authored.empty()) {
            pose.diagnostics.push_back(
                pointsTarget.GetString() +
                " has a non-finite constraint weight; constraint passed "
                "through");
            continue;
        }
        pose.movedProperties[pointsTarget] = VtValue(moved);
        _chainSnapshots.RecordFinal(pointsTarget, VtValue(moved));

        // NOT covered by the scalar point-chain oracle, and deliberately not
        // given a fake twin.
        //
        // That oracle earns its keep by re-deriving a graph/exec result along
        // an independent CPU path; a mismatch means one of the two is wrong.
        // This publish has no graph path to disagree with -- it is a single
        // direct computation -- so a "twin" recomputing it with the same
        // inputs and the same kernel would agree unconditionally and prove
        // nothing. Populating movedPropertiesCpu here would report an
        // agreement that was never tested.
        //
        // Routing a geometry-domain constraint through the mover graph as a
        // RigExecRevisionOp::Matrix revision is what would restore real
        // coverage: it needs an input edge for the solved delta instead of
        // the MatrixMover transform-provider edge. The common envelope now
        // already has the same packet semantics on both operations.
        pose.diagnostics.push_back(
            "cpu reference parity: " + pointsTarget.GetString() +
            " not covered; a geometry-domain constraint publishes directly "
            "rather than through the mover graph");
    }

    pose.valid = true;
    return pose;
}

size_t
RigExecConstraintHandlerCount(const TfToken &schemaType)
{
    return _FindConstraintHandler(schemaType) ? 1 : 0;
}

size_t
RigExecConstraintHandlerTotal()
{
    return _ConstraintHandlers().size();
}

bool
RigExecConstraintUsesRotationOrder(const TfToken &schemaType)
{
    const _ConstraintHandler *handler = _FindConstraintHandler(schemaType);
    return handler && handler->usesRotationOrder;
}

}  // namespace rigExec
