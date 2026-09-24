//
// RigExec mover registry storage and shared stage-reading helpers.
//

#include "moverRegistry.h"
#include "moverExecCommon.h"

#include "pxr/usd/usdGeom/pointBased.h"
#include "pxr/usd/usdSkel/blendShape.h"

#include <cmath>

PXR_NAMESPACE_USING_DIRECTIVE

namespace rigExec {

TF_DEFINE_PUBLIC_TOKENS(RigExecMoverExecTokens, RIGEXEC_MOVER_EXEC_TOKENS);

namespace {

std::vector<RigExecMoverHandler> &
_Rows()
{
    // Function-local so registration during library load cannot run
    // before it exists, whatever order the mover TUs initialize in.
    // After load the table is only read; see the header.
    static std::vector<RigExecMoverHandler> rows;
    return rows;
}

}  // namespace

void
RigExecRegisterMoverHandler(RigExecMoverHandler handler)
{
    _Rows().push_back(std::move(handler));
}

const RigExecMoverHandler *
RigExecFindMoverHandler(const TfToken &schemaType)
{
    for (const RigExecMoverHandler &handler : _Rows()) {
        if (schemaType == handler.schemaType) {
            return &handler;
        }
    }
    return nullptr;
}

const std::vector<RigExecMoverHandler> &
RigExecMoverHandlers()
{
    return _Rows();
}

SdfPathVector
RigExecRelationshipTargets(const UsdPrim &prim, const char *rel)
{
    SdfPathVector targets;
    if (const UsdRelationship r = prim.GetRelationship(TfToken(rel))) {
        r.GetTargets(&targets);
    }
    return targets;
}

RigExecReadPhase
RigExecPhaseForInput(
    const UsdPrim &moverPrim, const char *rel, const char *legacyAttr)
{
    RigExecReadPhase phase;
    if (const UsdRelationship r = moverPrim.GetRelationship(TfToken(rel))) {
        RigExecResolveReadPhase(r, legacyAttr, &phase, nullptr);
    } else if (legacyAttr) {
        // No relationship to hang metadata on, but the legacy attribute
        // may still be authored.
        if (const UsdAttribute a =
                moverPrim.GetAttribute(TfToken(legacyAttr))) {
            RigExecResolveReadPhase(a, legacyAttr, &phase, nullptr);
        }
    }
    return phase;
}

void
RigExecReadPhasedPoints(
    const UsdStageRefPtr &stage,
    const RigExecChainSnapshots &snapshots,
    UsdTimeCode time,
    const UsdPrim &prim,
    const char *relName,
    const char *legacyAttr,
    const SdfPath &pointsPath,
    const SdfPath &readerMover,
    VtVec3fArray *out)
{
    RigExecReadPhase phase;
    if (const UsdRelationship r = prim.GetRelationship(TfToken(relName))) {
        RigExecResolveReadPhase(r, legacyAttr, &phase, nullptr);
    }
    if (!phase.IsBase()) {
        if (const VtValue *v = snapshots.Lookup(
                pointsPath, phase, readerMover)) {
            if (v->IsHolding<VtVec3fArray>()) {
                *out = v->UncheckedGet<VtVec3fArray>();
                return;
            }
        }
    }
    if (const UsdAttribute a = stage->GetAttributeAtPath(pointsPath)) {
        a.Get(out, time);
    }
}

std::string
RigExecPointsTargetHint(
    const UsdStageRefPtr &stage, const SdfPath &target)
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

bool
RigExecValidateMoverTargets(
    const UsdStageRefPtr &stage,
    const UsdPrim &prim,
    const RigExecMoverHandler *handler,
    const std::vector<SdfPath> &targets,
    std::string *error)
{
    if (!handler || handler->customTargetValidation) {
        return true;
    }
    const std::string who =
        std::string(handler->schemaType) + " " + prim.GetPath().GetString();
    if (handler->domain == RigExecMoverDomain::Points) {
        // Typed geometry movers move points: every canonical target
        // must be a native points property -- anything else would
        // pass validation yet silently produce no application.
        for (const SdfPath &t : targets) {
            const UsdPrim owner = t.IsPropertyPath()
                ? stage->GetPrimAtPath(t.GetPrimPath())
                : UsdPrim();
            const UsdAttribute attr = t.IsPropertyPath()
                ? stage->GetAttributeAtPath(t)
                : UsdAttribute();
            if (!t.IsPropertyPath() ||
                t.GetNameToken() != "points" ||
                !owner || !owner.IsA<UsdGeomPointBased>() ||
                !attr ||
                attr.GetTypeName() != SdfValueTypeNames->Point3fArray) {
                *error = who + " target " + t.GetString() +
                         " is not a native UsdGeomPointBased "
                         "point3f[] points attribute" +
                         RigExecPointsTargetHint(stage, t);
                return false;
            }
        }
        if (handler->singleTarget && targets.size() != 1) {
            *error = who + " must have exactly one canonical points "
                           "target in v0.1 (multi-target fan-out would "
                           "alias mover-level parameters)";
            return false;
        }
        return true;
    }

    // Property-domain movers: exactly one exact scalar target, of the
    // value type the mover is statically typed for.
    if (targets.size() != 1) {
        *error = who + " must have exactly one target (its parameters "
                       "are mover-level, so a fan-out would alias them "
                       "across targets)";
        return false;
    }
    const SdfPath &t = targets[0];
    // A prim path canonicalizes to .points, which is never what a
    // property mover means; require the exact property the author wrote.
    const UsdAttribute attr = t.IsPropertyPath()
        ? stage->GetAttributeAtPath(t)
        : UsdAttribute();
    if (!attr) {
        *error = who + " target " + t.GetString() +
                 " is not an exact property path";
        return false;
    }
    const SdfValueTypeName valueType = attr.GetTypeName();
    bool typeOk = false;
    const char *expected = "";
    if (handler->domain == RigExecMoverDomain::PropertyFloat) {
        // A double target is a control's avar: the chain computes in
        // float and publishes the double, so a hidden control's
        // channels can be driven by keys.
        expected = "float/double";
        typeOk = valueType == SdfValueTypeNames->Float ||
                 valueType == SdfValueTypeNames->Double;
    } else if (handler->domain == RigExecMoverDomain::PropertyVec3f) {
        // Every GfVec3f-backed scalar role, not just float3: a mover
        // offsetting a vector3f or a color3f is doing the identical
        // arithmetic, and refusing it would be a distinction the
        // kernel does not make.
        expected = "float3/vector3f/point3f/normal3f/color3f";
        typeOk = valueType == SdfValueTypeNames->Float3 ||
                 valueType == SdfValueTypeNames->Vector3f ||
                 valueType == SdfValueTypeNames->Point3f ||
                 valueType == SdfValueTypeNames->Normal3f ||
                 valueType == SdfValueTypeNames->Color3f;
    } else {
        expected = "matrix4d";
        typeOk = valueType == SdfValueTypeNames->Matrix4d;
    }
    if (!typeOk) {
        *error = who + " target " + t.GetString() + " has type " +
                 valueType.GetAsToken().GetString() + "; expected " +
                 expected;
        return false;
    }
    return true;
}

bool
RigExecResolveBlendSampleLayout(
    const UsdStageRefPtr &stage,
    const SdfPath &blendShapePath,
    size_t pointCount,
    RigExecBlendSampleLayout *layout)
{
    layout->pointCount = pointCount;
    layout->valid = false;

    const UsdSkelBlendShape shape(
        stage->GetPrimAtPath(blendShapePath.GetPrimPath()));
    if (!shape) {
        return true;   // cache the miss; it cannot become a hit this epoch
    }
    const UsdAttribute offsetsAttr = shape.GetOffsetsAttr();
    const UsdAttribute indicesAttr = shape.GetPointIndicesAttr();

    // The admission rule. `uniform` already forbids time samples, so a
    // connection is the only route by which these can change while the epoch
    // stands -- and a connected value may itself be animated, which is
    // exactly what the skin layout refuses for.
    //
    // Noted and carried, NOT returned early: a refusal means "read this every
    // frame", so the read still has to happen. Returning here would hand the
    // caller an empty layout, the sample would fail validation, and a
    // perfectly good connected blend shape would come out as MoverFailed
    // instead of merely uncached.
    const bool cacheable = !offsetsAttr.HasAuthoredConnections() &&
                           !indicesAttr.HasAuthoredConnections();

    VtVec3fArray offsets;
    VtIntArray indices;
    offsetsAttr.Get(&offsets);
    indicesAttr.Get(&indices);

    // An empty pointIndices means the offsets are dense and parallel to the
    // base points -- UsdSkelBlendShape's own convention, kept rather than
    // invented so an authored blend shape from anywhere else reads correctly.
    if (!indices.empty()) {
        if (indices.size() != offsets.size()) {
            return cacheable;   // stable and wrong; cached as invalid
        }
        for (int index : indices) {
            if (index < 0 || size_t(index) >= pointCount) {
                return cacheable;
            }
        }
    } else if (!offsets.empty() && offsets.size() != pointCount) {
        return cacheable;
    }
    for (const GfVec3f &offset : offsets) {
        if (!std::isfinite(offset[0]) || !std::isfinite(offset[1]) ||
            !std::isfinite(offset[2])) {
            return cacheable;
        }
    }

    layout->offsets.assign(offsets.begin(), offsets.end());
    layout->indices.assign(indices.begin(), indices.end());
    layout->valid = true;
    return cacheable;
}

}  // namespace rigExec
