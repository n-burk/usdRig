//
// RigExec rig evaluator implementation.
//
#include "rigEvaluator.h"
#include "curvenetWeightComputations.h"
#include "curvenetAdjuster.h"
#include "parallel.h"

#include "frameExtraction.h"
#include "rigExecMath/dualQuat.h"
#include "rigExecMath/envelope.h"
#include "rigExecMath/geometryKernels.h"
#include "rigExecMath/propertyMath.h"
#include "rigExecMath/rbf.h"
#include "solverKernels.h"
#include "rigExecMath/singleChainIk.h"
#include "rigExecMath/solvers.h"
#include "rigExecMath/weightFields.h"

#include "pxr/base/gf/dualQuatd.h"
#include "pxr/base/gf/rotation.h"
#include "pxr/base/work/dispatcher.h"
#include "pxr/base/work/loops.h"
#include "pxr/base/work/withScopedParallelism.h"
#include "pxr/base/ts/spline.h"
#include "pxr/base/tf/diagnostic.h"
#include "pxr/base/tf/getenv.h"
#include "pxr/base/tf/notice.h"
#include "pxr/base/tf/pyLock.h"
#include "pxr/base/tf/stringUtils.h"
#include "pxr/usd/sdf/changeBlock.h"
#include "pxr/usd/sdf/layer.h"
#include "pxr/usd/usd/attribute.h"
#include "pxr/usd/usd/attributeQuery.h"
#include "pxr/usd/usd/editContext.h"
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
#include "pxr/usd/usdSkel/blendShape.h"

#include <algorithm>
#include <cmath>
#include <functional>
#include <limits>
#include <mutex>
#include <optional>
#include <set>
#include <unordered_map>
#include <unordered_set>

namespace rigExec {

namespace {

// Composed connections for an attribute, skipping the composition entirely
// when the attribute carries no authored connection opinion.
//
// UsdAttribute::GetConnections builds a Pcp property index and then a target
// index for the attribute on every call, and PcpBuildTargetIndex derives its
// targets from authored ConnectionPaths opinions alone -- so on an attribute
// that has no such opinion anywhere in its composition the whole index build
// can only ever come back empty. Compile and evaluate ask nearly every
// attribute on the rig for its connections while only a small minority are
// connected at all, so the cheap authored-metadata test comes first.
SdfPathVector
_AuthoredConnections(const UsdAttribute &attribute)
{
    SdfPathVector sources;
    if (attribute && attribute.HasAuthoredConnections()) {
        attribute.GetConnections(&sources);
    }
    return sources;
}

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

// The composed AUTHORED value of one attribute, or \p fallback.
//
// The compile-time twin of _ResolvedRead above, and deliberately not the same
// function: a solve that happens once per epoch has no generation's resolved
// inputs to consult and no time code that means anything, so it reads the
// stage at Default. Everything read this way is structure -- it is hashed
// into the binding-epoch digest, so an edit to one recompiles rather than
// being picked up mid-epoch.
template <class T>
T
_ReadAttribute(const UsdPrim &prim, const char *name, T fallback)
{
    T value = fallback;
    if (!prim) {
        return value;
    }
    if (const UsdAttribute a = prim.GetAttribute(TfToken(name))) {
        a.Get(&value);
    }
    return value;
}

}  // namespace

namespace {

const TfToken _computePointFrame("computePointFrame");
const TfToken _computePointFrameArray("computePointFrameArray");
// The joint rest a solver measures from. Overridden per solver batch when an
// earlier pose step wrote the joint (spec §4.2, "the incoming frame is the
// solver's rest reference").
const TfToken _computeRestFrame("computeRestFrame");
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
_GetPoseStackOrder(const UsdPrim &root)
{
    std::vector<UsdPrim> ordered;
    if (root) {
        // Reversing an ordinary composed pre-order yields exactly the stack
        // walk: reversed sibling branches, recursively, with each parent
        // after its descendants. Using UsdPrimRange here also preserves its
        // standard traversal predicate and instance behavior.
        for (const UsdPrim &prim : UsdPrimRange(root)) {
            ordered.push_back(prim);
        }
        std::reverse(ordered.begin(), ordered.end());
    }
    return ordered;
}

/// The same walk restricted to the Movers subtree, which is what numbers the
/// mover stack. Taken over the RIG ROOT instead, the identical walk numbers
/// the UNIFIED POSE STACK -- joint-writing aggregate solvers and pose-domain
/// frame constraints in one order (spec §4.2) -- and the mover order is a
/// restriction of it.
std::vector<UsdPrim>
_GetMoverExecutionOrder(const UsdPrim &movers)
{
    return _GetPoseStackOrder(movers);
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
           typeName == "RigExecCurvenetWeight" ||
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
    bool masksStatic = false;
    RigExecConstraintAxisMask precompTranslation;
    RigExecConstraintAxisMask precompRotation;
    RigExecConstraintAxisMask precompScale;
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
    if (c.masksStatic) {
        params.translationAxes = c.precompTranslation;
        params.rotationAxes = c.precompRotation;
        params.scaleAxes = c.precompScale;
    } else {
        params.translationAxes = _ReadConstraintAxisMask(
            *c.resolved, c.prim, "inputs:affectTranslationX",
            "inputs:affectTranslationY", "inputs:affectTranslationZ", c.time);
        params.rotationAxes = _ReadConstraintAxisMask(
            *c.resolved, c.prim, "inputs:affectRotationX",
            "inputs:affectRotationY", "inputs:affectRotationZ", c.time);
        // FBX disables scale by default; the explicit false fallback is the
        // authored contract, not an oversight (schema.usda:769-771).
        params.scaleAxes = _ReadConstraintAxisMask(
            *c.resolved, c.prim, "inputs:affectScaleX",
            "inputs:affectScaleY", "inputs:affectScaleZ", c.time, false);
    }
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
        (typeName == "RigExecCombineWeight" || typeName == "RigExecCurvenetWeight" ||
         _IsVolumeWeightType(typeName)) ? "dense" : "constant");
    const TfToken rangePolicy = readToken(
        "rigExec:rangePolicy",
        (typeName == "RigExecCombineWeight" || typeName == "RigExecCurvenetWeight" ||
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

    if (typeName == "RigExecCurvenetWeight") {
        if (!pointDomain) { *error = "curvenet weights require a mesh point domain"; return false; }
        const UsdPrim mesh = stage->GetPrimAtPath(moverTarget.GetPrimPath());
        if (!mesh.IsA<UsdGeomMesh>()) {
            *error = weightPath.GetString() + ": curvenet weights require a native mesh target";
            return false;
        }
        const std::pair<const char *, SdfValueTypeName> inputs[] = {
            {"rigExec:weightTarget", SdfValueTypeNames->Point3fArray},
            {"rigExec:curvenetPoints", SdfValueTypeNames->Point3fArray},
            {"rigExec:curvenetSplineIndices", SdfValueTypeNames->IntArray},
            {"rigExec:meshFaceCounts", SdfValueTypeNames->IntArray},
            {"rigExec:meshFaceIndices", SdfValueTypeNames->IntArray}};
        for (const auto &[name, type] : inputs) {
            SdfPathVector targets;
            weightPrim.GetRelationship(TfToken(name)).GetTargets(&targets);
            const UsdAttribute attr = targets.size() == 1 ?
                weightPrim.GetStage()->GetAttributeAtPath(targets[0]) : UsdAttribute();
            if (!attr || attr.GetTypeName() != type) {
                *error = weightPath.GetString() + ": " + name + " must name one native property of the expected type";
                return false;
            }
        }
        auto targetOf = [&](const char *name) {
            SdfPathVector targets;
            weightPrim.GetRelationship(TfToken(name)).GetTargets(&targets);
            return targets.front();
        };
        const SdfPath netPoints = targetOf("rigExec:curvenetPoints");
        const UsdPrim net = stage->GetPrimAtPath(netPoints.GetPrimPath());
        if (targetOf("rigExec:weightTarget") != moverTarget ||
            targetOf("rigExec:meshFaceCounts") != mesh.GetPath().AppendProperty(TfToken("faceVertexCounts")) ||
            targetOf("rigExec:meshFaceIndices") != mesh.GetPath().AppendProperty(TfToken("faceVertexIndices")) ||
            net.GetTypeName() != "RigExecCurvenet" || netPoints.GetNameToken() != TfToken("points") ||
            targetOf("rigExec:curvenetSplineIndices") != net.GetPath().AppendProperty(TfToken("rigExec:splineIndices"))) {
            *error = weightPath.GetString() + ": parametrization inputs must name the matching native mesh and curvenet properties";
            return false;
        }
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

// Discovers the rig's pose interpolators the same implicit way: being a
// RigExecPoseInterpolator under the rig is what makes a prim one.
//
// The interpolators live at <rig>/PoseInterpolators/<name>, OUTSIDE /Movers
// by design -- an interpolator writes no transform and no points, so it has
// no place in a mover ordering -- which is exactly why discovery is
// type-based here rather than a walk of the mover namespace.
std::vector<SdfPath>
_DiscoverPoseInterpolators(const UsdStageRefPtr &stage, const SdfPath &rigPath)
{
    static const TfToken kInterpolatorType("RigExecPoseInterpolator");

    std::vector<SdfPath> found;
    const UsdPrim rig = stage->GetPrimAtPath(rigPath);
    if (!rig) {
        return found;
    }
    for (const UsdPrim &prim : UsdPrimRange(rig)) {
        if (prim.GetTypeName() == kInterpolatorType) {
            found.push_back(prim.GetPath());
        }
    }
    return found;
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
        TfToken("RigExecTwistDistribution"), TfToken("RigExecRibbon"),
        TfToken("RigExecSplineIk")};
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

// Exact, iterative connection closure used by both the solver cache's
// authored-input index and its topology digest. Keep missing sources and
// cycles in the identity without reading any values or time-sample counts.
static std::set<SdfPath>
_CollectAttributeConnectionInputs(const UsdPrim &prim)
{
    std::set<SdfPath> inputs;
    if (!prim) {
        return inputs;
    }
    std::vector<SdfPath> pending;
    for (const UsdAttribute &attribute : prim.GetAttributes()) {
        pending.push_back(attribute.GetPath());
    }
    while (!pending.empty()) {
        const SdfPath path = pending.back();
        pending.pop_back();
        if (!inputs.insert(path).second) {
            continue;
        }
        const UsdAttribute attribute = prim.GetStage()->GetAttributeAtPath(path);
        if (attribute) {
            const SdfPathVector sources = _AuthoredConnections(attribute);
            pending.insert(pending.end(), sources.begin(), sources.end());
        }
    }
    return inputs;
}

// Matrix expressions can expose a posed ancestor through a connection to
// another provider's parent:space, including through default-space fallbacks.
// Trace the registered inputs conservatively: value-only identity toggles do
// not change this graph. Only an evaluator-owned joint override cuts it.
struct _PoseInputInfo {
    std::set<SdfPath> providers;
    std::set<SdfPath> attributes;
    bool connectedPose = false;
};

static UsdPrim
_NamespaceFrameProvider(UsdPrim prim)
{
    for (prim = prim.GetParent(); prim; prim = prim.GetParent()) {
        if (prim.GetTypeName() == "RigExecJoint" ||
            prim.GetTypeName() == "RigExecControl" ||
            _IsVolumeWeightType(prim.GetTypeName())) return prim;
    }
    return UsdPrim();
}

// Adjustment frames are outputs of the point graph, later than pose taps.
// Reject reads of those computed frames until the two domains share one DAG.
// Authored scalar channels remain ordinary legal attribute dependencies.
static bool
_ValidateAdjustmentPoseConsumers(const UsdStageRefPtr &stage,
    const UsdPrim &rig, std::string *error)
{
    // The closure below exists to report exactly one thing: a consumer that
    // reads a RigExecCurvenetAdjustment frame. With no Adjustment prim on the
    // stage at all there is nothing for it to find, so the whole walk -- every
    // attribute of every rig prim, every connection it reaches, every
    // default-space fallback -- is skipped without changing the answer.
    //
    // The test scans the whole stage, not the rig subtree, because the walk
    // follows connections and relationship targets that may leave the rig; and
    // it uses the all-prims predicate plus instance proxies so that an
    // Adjustment that is inactive, undefined, or behind an instance still
    // takes the slow path, matching every prim GetPrimAtPath can hand back.
    const bool stageHasAdjustment = [&stage]() {
        for (const UsdPrim &prim : UsdPrimRange::Stage(
                 stage, UsdTraverseInstanceProxies(UsdPrimAllPrimsPredicate))) {
            if (prim.GetTypeName() == "RigExecCurvenetAdjustment") {
                return true;
            }
        }
        return false;
    }();
    if (!stageHasAdjustment) {
        return true;
    }
    const auto isFrame = [](const UsdPrim &prim) {
        return prim && prim.GetAttribute(TfToken("rest:space")) &&
            prim.GetAttribute(TfToken("avars:tx"));
    };
    const auto parentFrame = [&](UsdPrim prim) {
        for (prim = prim.GetParent(); prim; prim = prim.GetParent())
            if (isFrame(prim)) return prim;
        return UsdPrim();
    };
    static const std::set<TfToken> spaces{
        TfToken("rest:space"), TfToken("default:space"), TfToken("avars:defaultSpace"),
        TfToken("posed:defaultSpace"), TfToken("parent:defaultSpace"),
        TfToken("parent:space"), TfToken("posed:space")};
    std::vector<std::pair<SdfPath, SdfPath>> pending;
    for (const UsdPrim &prim : UsdPrimRange(rig)) {
        const auto type = prim.GetTypeName();
        if (isFrame(prim) && type != "RigExecCurvenetAdjustment")
            pending.emplace_back(prim.GetPath(), prim.GetPath());
        // Connections are followed as exact attributes, never promoted to
        // provider reads solely because their owner is an Adjustment.
        for (const auto &attr : prim.GetAttributes()) {
            const SdfPathVector sources = _AuthoredConnections(attr);
            for (const auto &source : sources) pending.emplace_back(source, prim.GetPath());
        }
        std::vector<const char *> frameRelationships;
        if (_IsAggregateSolverType(type)) frameRelationships = {
            "rigExec:controls", "rigExec:rootControl", "rigExec:effectorControl",
            "rigExec:poleControl", "rigExec:inputA", "rigExec:inputB",
            "rigExec:start", "rigExec:end", "rigExec:startFrame", "rigExec:endFrame",
            "rigExec:twistFrames", "rigExec:midControl", "rigExec:endControl"};
        if (_IsFrameConstraintType(type)) frameRelationships = {
            "rigExec:moves", "rigExec:sources", "rigExec:aimTarget", "rigExec:worldUpObject",
            "rigExec:firstJoint", "rigExec:endJoint", "rigExec:effector", "rigExec:poleVectorObjects"};
        if (type == "RigExecMatrixMover") frameRelationships = {"rigExec:transform", "rigExec:transformSpace"};
        if (type == "RigExecSkinMover") frameRelationships = {"rigExec:influences"};
        if (type == "RigExecCurveMover") frameRelationships = {
            "rigExec:driverTransforms", "rigExec:driverTransformSpaces",
            "rigExec:driverBaseTransforms",
            "rigExec:driverBaseTransformSpaces"};
        for (const char *name : frameRelationships) {
            SdfPathVector targets;
            if (const auto rel = prim.GetRelationship(TfToken(name))) rel.GetTargets(&targets);
            for (const auto &target : targets) if (target.IsPrimPath())
                pending.emplace_back(target, prim.GetPath());
        }
    }
    std::set<SdfPath> visited;
    while (!pending.empty()) {
        const auto [path, consumer] = pending.back(); pending.pop_back();
        if (!visited.insert(path).second) continue;
        const UsdPrim prim = stage->GetPrimAtPath(path.GetPrimPath());
        if (!prim) continue;
        if (prim.GetTypeName() == "RigExecCurvenetAdjustment" &&
            (path.IsPrimPath() || spaces.count(path.GetNameToken()))) {
            if (error) *error = consumer.GetString() + " reads curvenet Adjustment frame " +
                path.GetString() + "; Adjustment frames are point-graph outputs and cannot "
                "feed pose solvers, constraints, matrix providers or computed spaces";
            return false;
        }
        const auto add = [&](const UsdPrim &owner, const char *name) {
            if (owner) pending.emplace_back(owner.GetPath().AppendProperty(TfToken(name)), consumer);
        };
        if (path.IsPrimPath()) {
            for (const auto &attr : prim.GetAttributes()) pending.emplace_back(attr.GetPath(), consumer);
            if (isFrame(prim)) {
                const auto parent = parentFrame(prim);
                if (parent) pending.emplace_back(parent.GetPath(), consumer);
            }
            continue;
        }
        const auto attr = stage->GetAttributeAtPath(path);
        if (!attr) continue;
        const SdfPathVector sources = _AuthoredConnections(attr);
        for (const auto &source : sources) pending.emplace_back(source, consumer);
        if (!isFrame(prim)) continue;
        const auto name = attr.GetName();
        if (name == "parent:space") {
            const auto parent = parentFrame(prim);
            if (parent) pending.emplace_back(parent.GetPath(), consumer);
        } else if (name == "parent:defaultSpace") {
            add(parentFrame(prim), "default:space");
        } else if (name == "posed:defaultSpace") {
            add(prim, "avars:defaultSpace");
        } else if (name == "avars:defaultSpace") {
            add(prim, "default:space");
        } else if (name == "default:space") {
            add(prim, "parent:defaultSpace");
            for (const char *input : {"rest:space", "rest:tx", "rest:ty", "rest:tz",
                 "rest:rx", "rest:ry", "rest:rz", "default:tx", "default:ty", "default:tz",
                 "default:rx", "default:ry", "default:rz"}) add(prim, input);
        }
    }
    return true;
}

static _PoseInputInfo
_CollectPoseInputInfo(const UsdPrim &prim)
{
    _PoseInputInfo info;
    if (!prim) return info;
    std::vector<std::pair<SdfPath, bool>> pending;
    std::set<std::pair<SdfPath, bool>> visited;
    for (const UsdAttribute &attribute : prim.GetAttributes()) {
        pending.emplace_back(attribute.GetPath(), false);
    }
    while (!pending.empty()) {
        const auto entry = pending.back();
        pending.pop_back();
        if (!visited.insert(entry).second) continue;
        const auto &[path, connected] = entry;
        info.attributes.insert(path);
        const UsdAttribute attribute = prim.GetStage()->GetAttributeAtPath(path);
        if (!attribute) continue;
        const SdfPathVector sources = _AuthoredConnections(attribute);
        for (const SdfPath &source : sources) pending.emplace_back(source, true);
        const UsdPrim provider = attribute.GetPrim();
        if (provider.GetTypeName() != "RigExecJoint" &&
            provider.GetTypeName() != "RigExecControl" &&
            !_IsVolumeWeightType(provider.GetTypeName())) continue;
        const TfToken name = attribute.GetName();
        const auto add = [&](const UsdPrim &owner, const char *input) {
            if (owner) pending.emplace_back(owner.GetPath().AppendProperty(
                TfToken(input)), connected);
        };
        if (name == "parent:space") {
            const UsdPrim parent = _NamespaceFrameProvider(provider);
            if (parent) {
                info.providers.insert(parent.GetPath());
                info.connectedPose = info.connectedPose || connected;
            }
        } else if (name == "parent:defaultSpace") {
            add(_NamespaceFrameProvider(provider), "default:space");
        } else if (name == "posed:defaultSpace") {
            add(provider, "avars:defaultSpace");
        } else if (name == "avars:defaultSpace") {
            add(provider, "default:space");
        } else if (name == "default:space") {
            add(provider, "parent:defaultSpace");
            for (const char *input : {"default:tx", "default:ty", "default:tz",
                 "default:rx", "default:ry", "default:rz", "rest:space",
                 "rest:tx", "rest:ty", "rest:tz", "rest:rx", "rest:ry", "rest:rz"}) {
                add(provider, input);
            }
            const UsdPrim parent = _NamespaceFrameProvider(provider);
            for (const char *input : {"rest:space", "rest:tx", "rest:ty",
                 "rest:tz", "rest:rx", "rest:ry", "rest:rz"}) add(parent, input);
        }
    }
    return info;
}

}  // namespace

namespace {

// What RIGEXEC_EVALUATION_MODE asked this process for, if anything.
//
// Not a code path: it selects the initial value of a setting callers can set
// themselves, so nothing here behaves differently for having been reached
// through the environment. It exists so an EXISTING suite can be re-run under
// the baked mode without every test in it learning about the mode -- which is
// the only way to check the program against the several hundred rigs those
// suites already build. Unset means nothing was asked and the rig's own
// rigExec:baked gets to answer instead.
//
// `authored` is the half the mode alone cannot carry, and it is about
// PRECEDENCE rather than about the value: an unset variable and
// RIGEXEC_EVALUATION_MODE=dynamic both mean Dynamic, and only the second is
// an instruction -- a suite that sets it is saying "every stage this process
// opens runs dynamically", including one whose attribute asks for the
// program. An unrecognised value counts as authored for the same reason: a
// typo must not silently hand the decision back to the stage, so it warns,
// means dynamic, and still outranks the attribute.
//
// Read ONCE per process, at the construction of the first evaluator, and
// fixed from then on: the function-local static below is initialised on its
// first call and never re-reads the environment. Changing the variable after
// that -- with setenv, or between two tests in one binary -- changes nothing;
// SetEvaluationMode is the only way to move an evaluator afterwards, and it
// moves that evaluator alone.
struct _EnvironmentMode {
    bool authored = false;
    RigExecEvaluationMode mode = RigExecEvaluationMode::Dynamic;
};

const _EnvironmentMode &
_EnvironmentEvaluationMode()
{
    static const _EnvironmentMode requested = [] {
        _EnvironmentMode result;
        const std::string value = TfGetenv("RIGEXEC_EVALUATION_MODE", "");
        result.authored = !value.empty();
        if (value == "baked") {
            result.mode = RigExecEvaluationMode::Baked;
        } else if (value == "parity") {
            result.mode = RigExecEvaluationMode::BakedWithParityCheck;
        } else if (result.authored && value != "dynamic") {
            TF_WARN("rigExec: RIGEXEC_EVALUATION_MODE=%s is not one of "
                    "dynamic, baked, parity; using dynamic",
                    value.c_str());
        }
        return result;
    }();
    return requested;
}

// The attribute a rig asks for the baked program with (schema.usda,
// RigExecRoot). Uniform and epoch-level: it decides which path answers the
// rig, not what any frame of it is.
const TfToken &
_BakedAttributeName()
{
    static const TfToken name("rigExec:baked");
    return name;
}

// Whether a fallback to the dynamic path is a FAILURE.
//
// Not a code path either: it changes no evaluated value and no dispatch,
// only whether a generation that ran dynamically while the mode asked for
// the program says so on the pose it publishes. It exists because a suite
// whose fixtures all decline the bake reports zero parity mismatches and
// goes green having compared nothing -- which is the one way a parity run
// can lie, and the way it lies about exactly the rigs a new operator was
// supposed to make bakeable.
//
// Read ONCE per process, in a function-local static for the same reason the
// mode above is: a tool sets it before the first evaluator exists, and
// nothing may change the answer between two tests in one binary.
bool
_BakeRequired()
{
    static const bool required =
        TfGetenvBool("RIGEXEC_BAKE_REQUIRED", false);
    return required;
}

// Every authored input computeRestFrame reads.
//
// Exactly the seven AttributeValue inputs of the computation
// (computations.cpp, RIGEXEC_REGISTER_XFORMABLE) -- rest:space and the six
// rest avars. Its eighth input is the NamespaceAncestor's own
// computeRestFrame, which reads these same seven on the ancestor, so the
// closure over a provider and its RigExec ancestors is the closure over this
// list. Nothing else can move a rest frame, which is what makes both the
// epoch-constancy test and the override test below exact rather than
// approximate.
const std::vector<TfToken> &
_RestInputNames()
{
    static const std::vector<TfToken> names = {
        TfToken("rest:space"), TfToken("rest:tx"), TfToken("rest:ty"),
        TfToken("rest:tz"),    TfToken("rest:rx"), TfToken("rest:ry"),
        TfToken("rest:rz")};
    return names;
}

bool
_IsRestInputName(const TfToken &name)
{
    const std::vector<TfToken> &names = _RestInputNames();
    return std::find(names.begin(), names.end(), name) != names.end();
}

// Whether any override in \p overrides could reach a cached blend sample
// shape.
//
// A shape is a function of exactly `offsets` and `pointIndices` on the
// UsdSkelBlendShape a sparse sample names, so an avar drag -- which is what
// every interactive override is -- cannot move one, and dropping 169
// resolved correctives per mouse sample would re-read 26,276-point arrays
// for nothing. A computation override names a computation this cannot
// inspect, so it counts, which is the same conservative reading the skin
// layout predicate makes.
bool
_OverridesReachBlendShapes(const std::vector<RigExecValueOverride> &overrides)
{
    static const TfToken offsets("offsets"), pointIndices("pointIndices");
    for (const RigExecValueOverride &o : overrides) {
        if (o.attribute.IsEmpty() || o.attribute == offsets ||
            o.attribute == pointIndices) {
            return true;
        }
    }
    return false;
}

// Whether any rest channel of \p provider can move within an epoch.
//
// Three ways it can, and the epoch-constant rest path is refused for all
// three: an authored connection (which can reach anything, including an
// animated avar, so it counts without being followed); time samples anywhere
// in the composition -- BOTH tests, because ValueMightBeTimeVarying() is
// false for exactly one sample of a non-composable type while Default and a
// numeric read still disagree about it; and a property chain writing the
// attribute, which recomputes it every generation (the same guard the skin
// layout already applies to its own three attributes).
bool
_ProviderRestMightVary(const UsdStageRefPtr &stage, const SdfPath &provider,
                       const std::set<SdfPath> &chainTargets)
{
    const UsdPrim prim = stage ? stage->GetPrimAtPath(provider) : UsdPrim();
    if (!prim) {
        return true;
    }
    for (const TfToken &name : _RestInputNames()) {
        if (chainTargets.count(provider.AppendProperty(name))) {
            return true;
        }
        const UsdAttribute attribute = prim.GetAttribute(name);
        if (!attribute) {
            continue;
        }
        if (attribute.HasAuthoredConnections() ||
            attribute.ValueMightBeTimeVarying() ||
            attribute.GetNumTimeSamples() > 0) {
            return true;
        }
    }
    return false;
}

}  // namespace

RigExecRigEvaluator::RigExecRigEvaluator(
    const UsdStageRefPtr &stage, const SdfPath &rigPath)
    : _stage(stage)
    , _rigPath(rigPath)
    , _evaluationMode(_EnvironmentEvaluationMode().mode)
    , _evaluationModeSource(_EnvironmentEvaluationMode().authored
                                ? RigExecEvaluationModeSource::Environment
                                : RigExecEvaluationModeSource::Default)
{
    // The static-input cache lives here and is consulted through the
    // resolved-input lookup every read already goes through.
    _resolvedInputs.SetStaticCache(&_staticInputs);
    if (_stage) {
        _noticeKey = TfNotice::Register(
            TfCreateWeakPtr(this), &RigExecRigEvaluator::_OnObjectsChanged,
            UsdStageWeakPtr(_stage));
    }
    // The rig evaluates directly against the source stage. There used to be
    // a private derived stage here, holding an anonymous session sublayer
    // for the compiler's generated property applications; the engine authors
    // nothing now, so there is nothing to hold and no stage to derive.
}

RigExecRigEvaluator::~RigExecRigEvaluator()
{
    TfNotice::Revoke(_noticeKey);
    _guideTaps.reset();
    _taps.reset();
}

namespace {

// Whether \p notice is provably nothing but new VALUES on the numeric avar
// channels (see _OnObjectsChanged): the gate for the in-place patch path.
// Shared by the notice handler and ClassifyNoticeDisposition, so the two
// can never disagree about which branch a notice takes.
bool
_NoticeIsAvarValuesOnly(const UsdNotice::ObjectsChanged &notice)
{
    if (!notice.GetResolvedAssetPathsResyncedPaths().empty()) {
        return false;
    }
    static const TfToken kDefault("default");
    static const TfToken kTimeSamples("timeSamples");
    static const TfToken kSpline("spline");
    static const TfToken kTypeName("typeName");
    // The numeric channels only. avars:defaultSpace and
    // avars:rotationOrder are tokens that choose how a frame is
    // composed, which is structure, so they are not in this list.
    static const TfToken kChannels[] = {
        TfToken("avars:tx"), TfToken("avars:ty"), TfToken("avars:tz"),
        TfToken("avars:sx"), TfToken("avars:sy"), TfToken("avars:sz"),
        TfToken("avars:rx"), TfToken("avars:ry"), TfToken("avars:rz"),
        TfToken("avars:rspin"), TfToken("avars:unitScaleFactor")};
    const auto isChannel = [](const TfToken &name) {
        for (const TfToken &avar : kChannels) {
            if (name == avar) {
                return true;
            }
        }
        return false;
    };
    bool sawAvar = false;
    for (const SdfPath &path : notice.GetResyncedPaths()) {
        if (!path.IsPropertyPath() || !isChannel(path.GetNameToken())) {
            return false;
        }
        for (const TfToken &field : notice.GetChangedFields(path)) {
            if (field != kTypeName && field != kDefault &&
                field != kTimeSamples && field != kSpline) {
                return false;
            }
        }
        sawAvar = true;
    }
    for (const SdfPath &path : notice.GetChangedInfoOnlyPaths()) {
        if (path.IsPrimPath()) {
            continue;  // ancestor info around the edit
        }
        if (!isChannel(path.GetNameToken())) {
            return false;
        }
        for (const TfToken &field : notice.GetChangedFields(path)) {
            if (field != kDefault && field != kTimeSamples &&
                field != kSpline) {
                return false;
            }
        }
        sawAvar = true;
    }
    return sawAvar;
}

}  // namespace

RigExecNoticeDisposition
RigExecRigEvaluator::ClassifyNoticeDisposition(
    const UsdNotice::ObjectsChanged &notice,
    std::vector<SdfPath> *patchedPaths) const
{
    if (patchedPaths) {
        patchedPaths->clear();
    }
    if (!_bakedProgram) {
        return RigExecNoticeDisposition::None;
    }
    std::vector<SdfPath> paths;
    if (_NoticeIsAvarValuesOnly(notice) &&
        _bakedProgram->DryRunAvarValueEdits(notice, &paths)) {
        if (patchedPaths) {
            *patchedPaths = std::move(paths);
        }
        return RigExecNoticeDisposition::Patched;
    }
    if (_bakedProgram->IsInvalidatedBy(notice)) {
        return RigExecNoticeDisposition::Stale;
    }
    return RigExecNoticeDisposition::StampBumped;
}

void
RigExecRigEvaluator::_OnObjectsChanged(
    const UsdNotice::ObjectsChanged &notice, const UsdStageWeakPtr &)
{
    ++_stageEditSerial;
    // Never evaluate in a notice callback: ExecUsd must finish invalidating
    // its own caches before the next pull. External inputs can live anywhere
    // on the stage, so retain conservative structural checks after edits.
    //
    // EXCEPT for a notice that is provably nothing but new VALUES on the
    // numeric avar channels: an Avar Editor slider tick, a typed value, a
    // key moved, a released gizmo. The structure digest never reads an
    // avar's value (only rigExec:* attributes, and the time-sample COUNT of
    // a handful of those), and the rest frames the epoch refresh on an edit
    // are rest:*, not avars -- so re-deriving both answered "nothing
    // changed" at ~115 ms per slider tick on the biped. Anything else in the
    // notice -- a resync, a non-avar property, a field other than a value --
    // keeps the conservative path.
    //
    // A RESYNC on one of those channels still qualifies when it is on the
    // property alone. The first value a layer holds for an avar creates its
    // property spec, which USD reports as a property resync carrying only
    // typeName, and undoing it removes the spec, a resync carrying nothing.
    // Every released gizmo drag and its undo is exactly that pair, and
    // treating it as structure rebaked the program for ~250 ms per release.
    // A prim resync is never a value edit and keeps the conservative path.
    const bool avarValuesOnly = _NoticeIsAvarValuesOnly(notice);
    if (!avarValuesOnly) {
        _structureDirty = true;
    }
    // Authored values may have moved anywhere on the stage, and the static
    // cache holds authored values: the whole of it is dropped, on every
    // notice, for the same reason the structure is re-checked on every one.
    //
    // This one is dropped even for an avar-only notice: the cache holds the
    // edited avar's OLD authored value, and it is the one cache below that
    // can.
    _staticInputs.Clear();
    // Everything else below is skipped for a notice that is only avar
    // VALUES, because none of it can hold one. Measured on the biped, the
    // re-reads they force cost ~15 ms of a ~23 ms Avar Editor tick:
    //   * property chains are float-typed with type-strict connections
    //     (_ValidateScalarConnection), so no binding can reach a double avar;
    //   * skin layouts, blend sample shapes and the base points a live graph
    //     pushes are jointIndices/weights, shape offsets and mesh points --
    //     none of them avars, and a notice that named any of them would not
    //     be avar-only.
    if (!avarValuesOnly) {
        // The property chains' pinned queries are the same kind of thing one
        // step further in: a query holds where a value comes FROM, which only
        // a stage edit can move, and the prims and relationship targets beside
        // them are structure. Dropped whole, on every notice, and rebound by
        // the next frame -- the conservative answer, and the only one that
        // cannot be wrong.
        _propertyChainBindings.reset();
        // Epoch-scoped geometry caches. A weight-paint edit and a points edit
        // are both VALUE edits: the digest does not change, so no new epoch
        // begins, and nothing else here would ever let go of the arrays they
        // replaced. Dropping them on every notice is the conservative answer,
        // and the only one that cannot be wrong -- rebuilding costs one read
        // per skinned mesh.
        _skinTopologies.Clear();
        _blendSampleShapes.Clear();
        // Which properties can REACH a layout is read off the same stage as
        // the layouts themselves -- a connection authored on
        // rigExec:jointWeights moves no epoch digest and recompiles nothing --
        // so the answer is dropped exactly where the layouts are.
        _skinLayoutInputsValid = false;
        for (auto &[target, live] : _liveGraphs) {
            if (live) live->basePointsPushed = false;
        }
    }
    // The baked program captured values, and the epoch digest is deliberately
    // blind to values, so the digest cannot say whether one of them moved.
    // The program's own index of what the bake read can: a notice that hits
    // it asks for a rebuild, and one that misses it -- a value on an input
    // read per frame, anything on a prim the bake never looked at -- leaves
    // the program standing, which is what keeps an edit elsewhere in the
    // scene from degrading the rig to the dynamic path.
    if (_bakedProgram) {
        // Classified first, through the same query the registry's notice
        // adapter consumes: the branch below and the adapter's
        // retire/re-resolve decision read one verdict.
        _lastNoticeDisposition =
            ClassifyNoticeDisposition(notice, &_lastNoticePatchedPaths);
        if (_lastNoticeDisposition == RigExecNoticeDisposition::Patched) {
            // Patched in place: the program's avar table already holds the
            // new constants, and its by-value slot comparison re-runs only
            // their cone. No rebuild, and no stamp bump -- the bump would
            // mark the whole program dirty for one run to find what the
            // comparison already finds. The dry run inside the
            // classification already passed, so this applies; a refusal
            // (impossible on this thread, but fail-closed) falls back to
            // the stamp bump.
            if (!_bakedProgram->ApplyAvarValueEdits(notice)) {
                _bakedProgram->BumpProgramStamp();
            }
        } else if (_lastNoticeDisposition ==
                   RigExecNoticeDisposition::Stale) {
            _bakedProgramStale = true;
            // The rebuild is allowed to refuse where the standing program did
            // not, and a refusal remembered from before this notice would
            // otherwise answer for a stage that has since changed.
            _bakeRefused = false;
            _bakeRefusalReasons.clear();
        } else {
            // The other half of the same index, and the reason the program
            // may skip work at all. A notice that misses the index is a
            // value edit on something the frame path re-reads -- a keyframe
            // moved, a weight repainted -- so the program is still right
            // about its structure and wrong about every value it cached from
            // the last frame. Saying so here is what makes the next
            // generation run everything once; the program compares its own
            // sources by value from then on.
            _bakedProgram->BumpProgramStamp();
        }
    } else {
        _lastNoticeDisposition = RigExecNoticeDisposition::None;
        _lastNoticePatchedPaths.clear();
    }
    // rigExec:baked is a VALUE on the rig root, so the epoch digest is blind
    // to it and _SettleEpoch will not recompile for it: this is the only
    // place a flip can be seen. Nothing is BUILT here -- never evaluate in a
    // notice callback -- and nothing needs to be. Dropping the program is
    // the whole of true -> false, and false -> true is built by the lazy
    // build in Evaluate, which is the same path SetEvaluationMode leaves
    // behind when it is asked on an epoch that has not settled.
    if (_NoticeNamesTheBakedAttribute(notice) &&
        _RefreshAttributeEvaluationMode() &&
        _evaluationMode == RigExecEvaluationMode::Dynamic) {
        _bakedProgram.reset();
        _bakedProgramPublished = false;
        _bakedProgramStale = false;
    }
    // The seed, connected, and guide requests read authored values straight
    // off the stage; an edit that leaves the override tuple unchanged (a
    // rest attribute, a weight, a goal transform) still changes what they
    // compute. Any stage edit therefore retires their cached snapshots, the
    // same way it retires the affected solver batches below.
    //
    // Retire means CLEAR, not just flagging: the seed and batch caches
    // are time-keyed LRUs, and the dirty flag only forces the FIRST
    // post-edit call to recompute. Once it clears, the other times would
    // hit pre-edit entries whose override tuple still matches -- the edit
    // changed the stage beneath an identical key.
    _firstFramePoseDirty = true;
    _firstFramePoseCache.Clear();
    _authSnapshotDirty = true;
    _authSnapTimeKeyed.clear();
    _guideDirty = true;
    _connectedPoseCache.clear();
    for (auto &[target, derived] : _derivedCache) {
        derived.cached = false;
    }
    const auto dirty = [this](const std::set<size_t> &batches) {
        for (size_t index : batches) {
            _solverBatches[index].dirty = true;
            // Beside the flag: the per-batch cache is a time-keyed LRU,
            // and the flag alone only forces the first post-edit call to
            // recompute -- the other times would hit pre-edit entries.
            _solverBatches[index].cache.Clear();
        }
    };
    for (const SdfPath &property : notice.GetChangedInfoOnlyPaths()) {
        const auto input = _solverInputBatches.find(property.GetPrimPath());
        if (input != _solverInputBatches.end()) {
            dirty(input->second);
        }
    }
    const auto dirtySubtree = [this, &dirty](const SdfPath &path) {
        const SdfPath primPath = path.GetPrimPath();
        for (auto input = _solverInputBatches.lower_bound(primPath);
             input != _solverInputBatches.end() &&
             input->first.HasPrefix(primPath); ++input) {
            dirty(input->second);
        }
    };
    for (const SdfPath &path : notice.GetResyncedPaths()) {
        dirtySubtree(path);
    }
    for (const SdfPath &path : notice.GetResolvedAssetPathsResyncedPaths()) {
        dirtySubtree(path);
    }
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

    const bool profileDigest = _profiler.IsEnabled();
    uint64_t digestRegionStart =
        profileDigest ? RigExecProfiler::NowUs() : 0;
    auto stampDigestRegion = [&](const char *name) {
        if (!profileDigest) {
            return;
        }
        const uint64_t now = RigExecProfiler::NowUs();
        _profiler.Record(name, "compile", digestRegionStart, now);
        digestRegionStart = now;
    };

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
    // Any scalar attribute as text. VtValue's stream operator rather than a
    // type switch: what the digest needs is that two different authored
    // values produce two different strings, not that the string is pretty.
    auto appendScalar = [&digest](const UsdPrim &prim, const char *name) {
        digest += name;
        digest += '=';
        VtValue value;
        if (const UsdAttribute a = prim.GetAttribute(TfToken(name))) {
            a.Get(&value);
        }
        digest += TfStringify(value);
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
            const SdfPathVector sources = _AuthoredConnections(a);
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
        for (const char *input : {"rigExec:curvenetPoints", "rigExec:curvenetSplineIndices",
                "rigExec:meshFaceCounts", "rigExec:meshFaceIndices"})
            appendRelTargets(w, input, true);
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
                 "inputs:extentV", "inputs:weights", "rigExec:autoSmooth",
                 "rigExec:basis", "rigExec:samplesPerSpline", "rigExec:unreachedValue"}) {
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

    // Pose interpolators. The RBF solve is a compile-time CONSTANT -- the
    // inverted matrix of every pose's kernel value at every other pose -- so
    // everything that constant is a function of is epoch identity: the
    // driver, every pose's rotation, translation, type and radii, the kernel,
    // the twist axis, the regularization and which channels are enabled.
    // Numeric though most of those are, an edit to one has to reach a solve
    // that has already happened, and nothing else in this digest even names a
    // prim outside /Movers.
    //
    // inputs:enabled on a POSE is hashed and inputs:enabled on the
    // INTERPOLATOR is not, which is the same distinction the schema draws: a
    // disabled pose is left out of the solve entirely (leaving it in would
    // keep it in every other pose's matrix row, so switching one off would
    // quietly change all the others), while a disabled interpolator just
    // publishes zeros and needs no recompile to do it.
    for (const SdfPath &interpolatorPath :
         _DiscoverPoseInterpolators(_stage, _rigPath)) {
        const UsdPrim interpolator = _stage->GetPrimAtPath(interpolatorPath);
        digest += interpolatorPath.GetString();
        digest += '|';
        appendRelTargets(interpolator, "rigExec:driver", false);
        for (const char *name : {"rigExec:kernel", "rigExec:twistAxis",
                                 "rigExec:regularization",
                                 "rigExec:normalize",
                                 "rigExec:enableRotation",
                                 "rigExec:enableTranslation",
                                 "rigExec:allowNegativeWeights"}) {
            appendScalar(interpolator, name);
        }
        for (const UsdPrim &pose : interpolator.GetChildren()) {
            digest += pose.GetName().GetString();
            digest += '=';
            for (const char *name : {"rigExec:poseType", "rigExec:rotation",
                                     "rigExec:translation",
                                     "rigExec:rotationRadius",
                                     "rigExec:translationRadius",
                                     "inputs:enabled"}) {
                appendScalar(pose, name);
            }
            digest += ';';
        }
        digest += '|';
    }
    digest += '|';

    stampDigestRegion("Digest.OutputSets");
    // Solver->joint wiring is epoch identity (view-free extraction,
    // user-directed 2026-07-25, replaces RigExecPointFrameView): each
    // solver's ORDERED rigExec:joints list decides which joint
    // self-extracts which aggregate element, so adding, removing, or
    // reordering joints changes what compile Pass 0 synthesizes. Order is
    // semantic (position = element index), so this list is never sorted.
    //
    // The block a prim contributes is a pure function of (prim, composed
    // stage) -- it reads connections and namespace-frame providers and
    // nothing else -- and the stage cannot change underneath a const digest
    // computation, so memoizing it for the duration of THIS digest emits
    // exactly the bytes the uncached walk emits.
    //
    // MEASURED 2026-09-13, biped (24 solvers): uncached, this emission was
    // 1.15 MILLION UsdAttribute::GetConnections calls per digest -- 2.33 s --
    // and one digest is computed on EVERY evaluate that follows ANY stage
    // edit, which is what made letting go of a gizmo take 2.4 s. It runs once
    // per ancestor, per relationship target, per relationship, per aggregate
    // solver, and each run re-walked the same pose-input closure from
    // scratch, so a few hundred prims were re-closed tens of thousands of
    // times: O(n^2) in solver count, the 455 + 51n + 6.2n^2 ms measured.
    //
    // The caches are function-local: nothing survives the call, so a stage
    // edit between two digests is still seen. They are deliberately NOT
    // evaluator members -- a member cache would have to be invalidated by
    // _OnObjectsChanged, and the whole point of the digest is to be the
    // thing that does not trust incremental invalidation.
    std::unordered_map<SdfPath, std::string, SdfPath::Hash> solverInputTokens;
    // One attribute can sit in many blocks (every joint's parent:space chain
    // republishes its ancestors' attributes), so resolve each path's
    // "path:type->sources|" text once too.
    std::unordered_map<SdfPath, std::string, SdfPath::Hash> connectionText;
    // Each provider's transitive pose-input closure, as one token, shared by
    // every closure that reaches it.
    std::unordered_map<SdfPath, std::string, SdfPath::Hash>
        providerClosureTokens;
    struct _DigestHop {
        std::vector<SdfPath> own;     // this prim's attributes, sorted
        std::set<SdfPath> depends;    // prims this one reads
    };
    std::unordered_map<SdfPath, _DigestHop, SdfPath::Hash> digestHops;
    const auto appendSolverInputConnections =
        [this, &digest, &solverInputTokens, &connectionText,
         &providerClosureTokens, &digestHops](const UsdPrim &prim) {
        const SdfPath primPath = prim ? prim.GetPath() : SdfPath();
        const auto cached = solverInputTokens.find(primPath);
        if (cached != solverInputTokens.end()) {
            digest += cached->second;
            return;
        }
        // THE TRANSITIVE POSE-INPUT CLOSURE, AS A MERKLE TOKEN.
        //
        // This used to flatten a prim's whole provider closure into one set
        // of attribute paths and hash the text of all of them -- rebuilt
        // from scratch per prim, because every prim's closure is a
        // different set and so the per-prim cache below could not share
        // anything between them. On a nested chain a prim at depth d reads
        // d providers, so its block was O(d) entries of O(d)-long paths,
        // and the digest over N such prims was CUBIC. Measured on one
        // RigExecFkChain over a nested chain: 2.0 s at 100 joints, 191 s at
        // 400, all of it in Digest.Solvers.
        //
        // Now each provider's closure is a token computed once: the text of
        // its OWN attributes plus the tokens of the providers it reads, in
        // path order. A provider is shared by every closure that reaches
        // it, so the work is linear in providers. Identity is at least as
        // strong as the flattened set: any change to any attribute, type or
        // connection anywhere in a closure changes that provider's token
        // and therefore every token that folds it in -- and it also
        // distinguishes WHICH provider an attribute arrived through, which
        // the flat set merged.
        //
        // Iterative post-order, not recursion: closures run hundreds deep.
        // A cycle cannot be closed over, so the edge that closes one
        // contributes a marker naming the path instead; Compile reports the
        // cycle itself, and the marker still makes introducing or removing
        // one change the digest.
        const auto attributeText = [this, &connectionText](
                                       const SdfPath &path)
            -> const std::string & {
            auto text = connectionText.find(path);
            if (text == connectionText.end()) {
                const UsdAttribute attribute = _stage->GetAttributeAtPath(path);
                std::string entry = path.GetString();
                entry += ':';
                entry += attribute
                    ? attribute.GetTypeName().GetAsToken().GetString()
                    : std::string("missing");
                entry += "->";
                for (const SdfPath &source : _AuthoredConnections(attribute)) {
                    entry += source.GetString();
                    entry += ',';
                }
                entry += '|';
                text = connectionText.emplace(path, std::move(entry)).first;
            }
            return text->second;
        };
        // ONE HOP of a prim's pose inputs: every attribute it owns, and the
        // prims those attributes lead to. The same rules as
        // _CollectPoseInputInfo -- connections, parent:space to the
        // namespace frame provider, the default-space fallback -- but it
        // stops at the first foreign prim instead of following it, because
        // that prim's own token already covers everything past it.
        //
        // _CollectPoseInputInfo follows the default-space fallback to the
        // root, so a prim at depth d returns ~13 attributes per ancestor in
        // a std::set whose SdfPath comparisons also walk depth. Built for
        // every prim, that stayed super-quadratic after the token work
        // above (10.4 s of 11 at 400 joints). The pose schedule still uses
        // it, unchanged; only the digest reads this instead.
        const auto hopFor =[this, &digestHops](const UsdPrim &provider)
            -> const _DigestHop & {
            const SdfPath key = provider.GetPath();
            auto found = digestHops.find(key);
            if (found != digestHops.end()) {
                return found->second;
            }
            _DigestHop hop;
            const TfToken type = provider.GetTypeName();
            const bool isProvider = type == "RigExecJoint" ||
                                    type == "RigExecControl" ||
                                    _IsVolumeWeightType(type);
            const UsdPrim frameParent =
                isProvider ? _NamespaceFrameProvider(provider) : UsdPrim();
            for (const UsdAttribute &attribute : provider.GetAttributes()) {
                hop.own.push_back(attribute.GetPath());
                for (const SdfPath &source : _AuthoredConnections(attribute)) {
                    if (source.GetPrimPath() != key) {
                        hop.depends.insert(source.GetPrimPath());
                    }
                }
                if (!isProvider || !frameParent) {
                    continue;
                }
                // The fallback rules that leave this prim: each of them
                // reads the namespace frame provider, whose token carries
                // its own continuation of the chain.
                const TfToken &name = attribute.GetName();
                if (name == "parent:space" || name == "parent:defaultSpace" ||
                    name == "default:space") {
                    hop.depends.insert(frameParent.GetPath());
                }
            }
            std::sort(hop.own.begin(), hop.own.end());
            return digestHops.emplace(key, std::move(hop)).first->second;
        };
        const auto tokenOf = [](const std::string &kind,
                                const std::string &text) {
            return kind + "#" + std::to_string(std::hash<std::string>{}(text)) +
                   ":" + std::to_string(text.size()) + "|";
        };

        std::string providerToken;
        if (prim) {
            // 1 = on the walk, 2 = token ready (in providerClosureTokens).
            std::unordered_map<SdfPath, int, SdfPath::Hash> state;
            std::vector<std::pair<SdfPath, bool>> walk{{prim.GetPath(), false}};
            while (!walk.empty()) {
                const auto [path, expanded] = walk.back();
                walk.pop_back();
                if (providerClosureTokens.count(path)) {
                    continue;
                }
                const UsdPrim provider = _stage->GetPrimAtPath(path);
                if (!provider) {
                    providerClosureTokens.emplace(
                        path, "missingProvider:" + path.GetString() + "|");
                    continue;
                }
                const _DigestHop &hop = hopFor(provider);
                // WHAT THIS TOKEN FOLDS IN: this prim's own attributes by
                // text, and every prim it reads one hop away by token. A
                // foreign attribute is never written out here -- its owner's
                // token already hashes all of that owner's own attributes,
                // so any edit to it changes this token too. That is at least
                // as strong an identity as the flattened set this replaced,
                // and marginally stronger: a structural edit to another
                // attribute of a connection-source prim now also re-epochs,
                // which is a recompile, never a missed one.
                const std::set<SdfPath> &depends = hop.depends;
                if (!expanded) {
                    if (state[path] == 1) {
                        continue;  // already scheduled on this walk
                    }
                    state[path] = 1;
                    walk.push_back({path, true});
                    for (const SdfPath &input : depends) {
                        if (!providerClosureTokens.count(input) &&
                            state[input] != 1) {
                            walk.push_back({input, false});
                        }
                    }
                    continue;
                }
                // std::sets: already in path order and unique, which is
                // what makes the token independent of the order anything
                // was discovered in.
                std::string text = path.GetString();
                text += '{';
                for (const SdfPath &attribute : hop.own) {
                    text += attributeText(attribute);
                }
                for (const SdfPath &input : depends) {
                    const auto ready = providerClosureTokens.find(input);
                    if (ready != providerClosureTokens.end()) {
                        text += ready->second;
                    } else {
                        // Still on the walk: this edge closes a cycle.
                        text += "cycle@" + input.GetString() + "|";
                    }
                }
                text += '}';
                state[path] = 2;
                providerClosureTokens.emplace(path, tokenOf("provider", text));
            }
            providerToken = providerClosureTokens[prim.GetPath()];
        }

        // The prim's own authored connection inputs sit beside its provider
        // closure, exactly as the flattened set used to add them.
        std::string block;
        for (const SdfPath &path : _CollectAttributeConnectionInputs(prim)) {
            block += attributeText(path);
        }
        block += providerToken;
        // What lands in the digest is a TOKEN for this prim's closure, not
        // the closure text. The same prim's block is emitted once per
        // ancestor per target per relationship per solver, so appending the
        // text leaves the digest STRING quadratic in rig size even with the
        // walking cached away -- tens of MB of std::string concatenation per
        // evaluate. The digest as a whole is already collapsed to one size_t
        // by std::hash<std::string> on the way out, so folding a fixed-size
        // hash of the block in here is the same kind of identity it already
        // was: different closures give different tokens, and the surrounding
        // structure (which prim, which relationship, which ancestor) stays in
        // plain text around it. The length goes in beside the hash, so two
        // blocks have to collide in both to be confused.
        std::string token = "closure#" +
            std::to_string(std::hash<std::string>{}(block)) + ":" +
            std::to_string(block.size()) + "|";
        digest += token;
        solverInputTokens.emplace(primPath, std::move(token));
    };
    // THE ANCESTOR CHAIN OF A RELATIONSHIP TARGET, AS ONE TOKEN PER PATH.
    //
    // Every aggregate-solver target contributes its whole ancestor chain --
    // each ancestor's path, type and input closure -- because a rewire
    // anywhere above a joint changes the frame it resolves against. That
    // used to be emitted inline: per target, walk every ancestor and append
    // its full path string. On a nested chain that is N targets x N
    // ancestors x a path N components long, and it was cubic. Measured on
    // one RigExecFkChain over a nested chain: 287 ms at 50 joints, 2.0 s at
    // 100, 188 s at 400 -- and 224 of the biped's 372 ms compile, the
    // largest single cost left after the pose-schedule work.
    //
    // Siblings share every ancestor above them, and a chain's joints share
    // all of theirs, so each path's chain is computed once and folds in its
    // parent's token. The identity is the same kind the closure tokens above
    // already use: a hash of the exact text plus its length, so any change
    // to any ancestor's path, type or closure still changes every token
    // below it. Function-local for the same reason as the caches above.
    //
    // Iterative, not recursive: a chain can be hundreds of joints deep.
    std::unordered_map<SdfPath, std::string, SdfPath::Hash> ancestorChainTokens;
    const auto ancestorChainToken =
        [this, &digest, &ancestorChainTokens,
         &appendSolverInputConnections](const SdfPath &start)
            -> const std::string & {
        static const std::string kRoot;
        // Walk up to the first path already known (or the root), noting the
        // ones that are not; then build them top-down so each finds its
        // parent's token ready.
        std::vector<SdfPath> missing;
        for (SdfPath path = start;
             !path.IsEmpty() && path != SdfPath::AbsoluteRootPath();
             path = path.GetParentPath()) {
            if (ancestorChainTokens.count(path)) {
                break;
            }
            missing.push_back(path);
        }
        for (auto it = missing.rbegin(); it != missing.rend(); ++it) {
            const SdfPath &path = *it;
            const UsdPrim ancestor = _stage->GetPrimAtPath(path);
            std::string block = path.GetString();
            block += ':';
            block += ancestor ? ancestor.GetTypeName().GetString()
                              : std::string("missing");
            block += ',';
            // The input closure's token, captured rather than appended: the
            // helper writes into the digest, and here it belongs inside this
            // path's block instead.
            const size_t mark = digest.size();
            appendSolverInputConnections(ancestor);
            block.append(digest, mark, std::string::npos);
            digest.resize(mark);
            const SdfPath parent = path.GetParentPath();
            const auto above = ancestorChainTokens.find(parent);
            if (above != ancestorChainTokens.end()) {
                block += above->second;
            }
            ancestorChainTokens.emplace(
                path, "chain#" +
                    std::to_string(std::hash<std::string>{}(block)) + ":" +
                    std::to_string(block.size()) + "|");
        }
        const auto found = ancestorChainTokens.find(start);
        return found != ancestorChainTokens.end() ? found->second : kRoot;
    };
    if (const UsdPrim rig = _stage->GetPrimAtPath(_rigPath)) {
        // Recursive over the composed rig subtree (not GetChildren):
        // solvers live wherever the author put them, so every scope's
        // wiring must contribute to epoch identity
        // (consistent with mover discovery and compile Pass 0).
        //
        // This walk's ORDER is now evaluation semantics, not only identity:
        // the solver stack ordinal is the reverse of exactly this composed
        // pre-order, so two solvers writing one joint commit in the order
        // this loop visits them, reversed. Each segment leads with the
        // solver's full path, so a sibling reorder (or a layer-strength
        // change that composes a different order) changes the concatenation
        // and starts a new epoch -- which is the only reason a reordered
        // stack cannot keep a stale schedule. Do not "optimize" this into a
        // sorted set or a path-keyed map.
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
            // The compiled solver DAG depends on input wiring as well as
            // output bindings. A Twist endpoint or IK control rewire must
            // replace the schedule even when cardinality is unchanged.
            if (isAggregate) {
                appendSolverInputConnections(solver);
                for (const UsdRelationship &rel : solver.GetRelationships()) {
                    const std::string name = rel.GetName().GetString();
                    const SdfPathVector targets =
                        appendRelTargets(solver, name.c_str(), false);
                    appendFrameBindingIdentity(solver, name.c_str());
                    for (const SdfPath &target : targets) {
                        digest += ancestorChainToken(target.GetPrimPath());
                    }
                }
            }
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
                // The start-frame inference switch: flipping it rewrites
                // the derived session targets the rel loop above hashed,
                // so the token joins the digest or no recompile follows.
                appendToken(solver, "rigExec:startFramePolicy");
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
            } else if (stype == "RigExecSplineIk") {
                // Cardinality is the joints list, hashed above. The
                // per-joint volume weights are a static parallel array
                // whose length Compile() validates; hash the length and
                // the sample presence so an edit that breaks the parallel
                // shape (or samples the attribute) re-runs that check.
                const UsdAttribute wa =
                    solver.GetAttribute(TfToken("rigExec:volumeWeights"));
                VtFloatArray w;
                if (wa) {
                    wa.Get(&w);
                }
                digest += std::to_string(w.size()) + "/" +
                          std::to_string(wa ? wa.GetNumTimeSamples() : 0) +
                          ",";
            }
            digest += ';';
        }
    }

    stampDigestRegion("Digest.Solvers");
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
            appendRelTargets(prim, "rigExec:transformSpace", true);
            appendRelTargets(prim, "rigExec:driverTransforms", false);
            appendRelTargets(prim, "rigExec:driverTransformSpaces", false);
            appendRelTargets(prim, "rigExec:driverBaseTransforms", false);
            appendRelTargets(prim, "rigExec:driverBaseTransformSpaces", false);
            // Influence order is semantic: jointIndices index into it.
            appendRelTargets(prim, "rigExec:influences", false);
            appendToken(prim, "rigExec:transformReadPhase");
            appendToken(prim, "rigExec:skinningMethod");
            appendToken(prim, "rigExec:operation");
            appendToken(prim, "rigExec:mode");
            appendToken(prim, "rigExec:deltaSpace");
            for (const char *input : {
                     "inputs:defaultWeight", "inputs:enabled",
                     "inputs:value", "inputs:min", "inputs:max",
                     "inputs:keys"}) {
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
                    appendPhase(sample, "rigExec:targetPoints");
                    appendToken(sample, "rigExec:pointsReadPhase");
                    // WHICH blend shape a sparse sample names is structure,
                    // so it belongs in the epoch digest. What the shape
                    // CONTAINS deliberately does not: offsets and
                    // pointIndices are uniform, so they cannot be time
                    // samples, and RigExecBlendSampleCache is cleared by
                    // every notice -- a sculpt edit is caught by the cache's
                    // array compare on the next frame without forcing a
                    // recompile of the whole rig.
                    appendRelTargets(sample, "rigExec:blendShape", true);
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
    stampDigestRegion("Digest.Movers");
    return std::hash<std::string>{}(digest);
}

// ---------------------------------------------------------------------------
// Pose interpolators (the conventional poseInterpolator)
// ---------------------------------------------------------------------------

bool
RigExecRigEvaluator::_CompilePoseInterpolators(
    const std::vector<SdfPath> &joints,
    const std::vector<SdfPath> &controls,
    std::vector<_PoseInterpolator> *out,
    std::vector<std::string> *notes,
    std::string *error) const
{
    static const TfToken kPoseType("RigExecPose");
    static const TfToken kDriver("rigExec:driver");
    static const TfToken kWeight("outputs:weight");

    out->clear();
    const std::vector<SdfPath> interpolators =
        _DiscoverPoseInterpolators(_stage, _rigPath);
    if (interpolators.empty()) {
        return true;
    }

    // A driver has to be something that PUBLISHES A FRAME, because a frame is
    // the only thing this phase can read. A RigExecControl qualifies exactly
    // as a RigExecJoint does -- the conventional drivers are hidden joints, but a
    // control has a local rotation just as a joint does -- so the set is both.
    std::set<SdfPath> providers(joints.begin(), joints.end());
    providers.insert(controls.begin(), controls.end());

    for (const SdfPath &path : interpolators) {
        const UsdPrim prim = _stage->GetPrimAtPath(path);
        _PoseInterpolator record;
        record.prim = path;

        SdfPathVector driverTargets;
        if (const UsdRelationship rel = prim.GetRelationship(kDriver)) {
            rel.GetTargets(&driverTargets);
        }
        if (driverTargets.size() != 1) {
            *error = "pose interpolator " + path.GetString() + " names " +
                     std::to_string(driverTargets.size()) +
                     " rigExec:driver targets; exactly one is required";
            return false;
        }
        record.driver = driverTargets[0].GetPrimPath();
        if (!providers.count(record.driver)) {
            *error = "pose interpolator " + path.GetString() +
                     " names a rigExec:driver, " + record.driver.GetString() +
                     ", that is not a RigExecJoint or RigExecControl of this "
                     "rig and therefore publishes no frame to measure";
            return false;
        }
        // The driver's local rotation is measured against its nearest
        // frame-publishing ancestor. That is the immediate namespace parent on
        // every rig this has been run on; anything else is REPORTED rather
        // than silently accepted, because it means the delta is being measured
        // across a prim that may carry a transform of its own.
        for (SdfPath walk = record.driver.GetParentPath();
             !walk.IsEmpty() && !walk.IsAbsoluteRootPath() &&
                 walk != _rigPath.GetParentPath();
             walk = walk.GetParentPath()) {
            if (providers.count(walk)) {
                record.driverParent = walk;
                break;
            }
        }
        if (notes && !record.driverParent.IsEmpty() &&
            record.driverParent != record.driver.GetParentPath()) {
            notes->push_back(
                "warning: pose interpolator " + path.GetString() +
                " measures its driver against " +
                record.driverParent.GetString() +
                ", which is not the driver's immediate namespace parent");
        }

        record.allowNegativeWeights =
            _ReadAttribute(prim, "rigExec:allowNegativeWeights", true);

        RigExecRbfSolverDesc desc;
        desc.kernel =
            _ReadAttribute(prim, "rigExec:kernel", TfToken("gaussian")) ==
                    TfToken("linear")
                ? RigExecRbfKernel::Linear
                : RigExecRbfKernel::Gaussian;
        desc.regularization =
            _ReadAttribute(prim, "rigExec:regularization", 0.0f);
        desc.normalize = _ReadAttribute(prim, "rigExec:normalize", true);
        desc.enableRotation =
            _ReadAttribute(prim, "rigExec:enableRotation", true);
        desc.enableTranslation =
            _ReadAttribute(prim, "rigExec:enableTranslation", false);
        const TfToken axis =
            _ReadAttribute(prim, "rigExec:twistAxis", TfToken("X"));
        desc.twistAxis = axis == TfToken("Y")   ? GfVec3d(0.0, 1.0, 0.0)
                         : axis == TfToken("Z") ? GfVec3d(0.0, 0.0, 1.0)
                                                : GfVec3d(1.0, 0.0, 0.0);
        if (desc.enableTranslation && notes) {
            // Said once, plainly, rather than measured wrongly. This phase
            // reads the driver's ROTATION; no interpolator of the shipped
            // biped enables the translation channel (0 of 29), so the
            // arithmetic that would measure it has never been run against
            // anything and is not being guessed at here.
            notes->push_back(
                "warning: pose interpolator " + path.GetString() +
                " sets rigExec:enableTranslation, which the evaluation phase "
                "does not measure; its poses are judged on rotation alone");
        }

        std::vector<double> radii;
        std::vector<double> translationRadii;
        size_t poseChildren = 0;
        for (const UsdPrim &child : prim.GetChildren()) {
            if (child.GetTypeName() != kPoseType) {
                *error = "pose interpolator " + path.GetString() +
                         " has a child, " + child.GetPath().GetString() +
                         ", that is not a RigExecPose but a " +
                         child.GetTypeName().GetString();
                return false;
            }
            ++poseChildren;
            const SdfPath weightPath = child.GetPath().AppendProperty(kWeight);
            if (const UsdAttribute weight = child.GetAttribute(kWeight)) {
                // outputs:weight is a SOURCE. An authored outbound connection
                // on it would say it takes its value from somewhere else,
                // which is the opposite of what this phase does to it.
                if (weight.HasAuthoredConnections()) {
                    *error = "pose " + child.GetPath().GetString() +
                             " carries an authored connection on "
                             "outputs:weight, which its interpolator writes";
                    return false;
                }
            }
            if (!_ReadAttribute(child, "inputs:enabled", true)) {
                record.disabledPoseWeights.push_back(weightPath);
                continue;
            }

            const GfQuatf rotation =
                _ReadAttribute(child, "rigExec:rotation", GfQuatf(1.0f));
            desc.poses.push_back(RigExecRbfEulerFromQuaternion(
                GfQuatd(rotation.GetReal(),
                        GfVec3d(rotation.GetImaginary()))));
            // The schema stores centimetres, the solver takes metres.
            const GfVec3f translation =
                _ReadAttribute(child, "rigExec:translation", GfVec3f(0.0f));
            desc.translations.push_back(GfVec3d(translation[0] / 100.0,
                                                translation[1] / 100.0,
                                                translation[2] / 100.0));
            const TfToken kind =
                _ReadAttribute(child, "rigExec:poseType", TfToken("swing"));
            desc.poseTypes.push_back(
                kind == TfToken("twist")   ? RigExecRbfPoseType::Twist
                : kind == TfToken("whole") ? RigExecRbfPoseType::Whole
                                           : RigExecRbfPoseType::Swing);

            const double radius =
                _ReadAttribute(child, "rigExec:rotationRadius", 0.0f);
            const double translationRadius =
                _ReadAttribute(child, "rigExec:translationRadius", 0.0f);
            if (!std::isfinite(radius) || radius < 0.0 ||
                !std::isfinite(translationRadius) ||
                translationRadius < 0.0) {
                *error = "pose " + child.GetPath().GetString() +
                         " has a radius that is negative or not finite";
                return false;
            }
            radii.push_back(radius);
            // Centimetres here too, and NOT the rotation radius: a brow's
            // poses sit millimetres apart and a rotation width would swallow
            // every one of them.
            translationRadii.push_back(translationRadius / 100.0);
            record.poseWeights.push_back(weightPath);
        }
        if (poseChildren == 0) {
            *error = "pose interpolator " + path.GetString() +
                     " has no RigExecPose children";
            return false;
        }
        if (record.poseWeights.empty()) {
            // Every pose disabled is not an error -- it is a shape-preserving
            // enable applied to all of them -- and there is nothing to solve.
            out->push_back(std::move(record));
            continue;
        }
        if (!desc.enableTranslation) {
            desc.translations.clear();
            translationRadii.clear();
        }

        // The shared widths stay at zero and the PER-POSE ones are adopted: a
        // shipped table's widths carry a painted poseFalloff that no falloff
        // vector reproduces, so they are data, not something to re-derive
        // (rbf.h SetSolvedTable). The inverted matrix is the one thing that IS
        // re-derived, because it is a pure function of everything above and
        // storing it would be a second copy to keep in step (schema
        // RigExecPoseInterpolator).
        desc.radius = 0.0;
        desc.translationRadius = 0.0;
        RigExecRbfSolver solver(desc);
        solver.SetSolvedTable(radii, translationRadii, {});
        if (!solver.Solve()) {
            *error = "pose interpolator " + path.GetString() +
                     " could not be solved";
            return false;
        }
        if (notes && solver.Degenerate()) {
            notes->push_back(
                "warning: pose interpolator " + path.GetString() +
                " has poses that are coincident under the channels it has "
                "enabled; its weights will sit at 1/n");
        }
        record.solver = std::move(solver);
        out->push_back(std::move(record));
    }
    return true;
}

void
RigExecRigEvaluator::_EvaluatePoseInterpolators(
    UsdTimeCode time,
    const std::vector<RigExecPointFrame> &restFrames,
    const std::vector<char> &restLive,
    const std::vector<RigExecPointFrame> &finalFrames,
    const std::vector<char> &finalLive,
    RigExecRigPose *pose)
{
    if (_poseInterpolators.empty()) {
        return;
    }
    // Its own scope and its own category. A third of the evaluate was
    // invisible to the profiler until 2026-09-13, because nobody had scoped
    // the publish loops; this phase is not going to be the next one.
    RIGEXEC_PROFILE_SCOPE_CAT(_profiler, "PoseInterpolators", "psd");

    // ORDERING, ASSERTED RATHER THAN TRUSTED (first half; the second is at
    // the head of the geometry chains).
    //
    // WHAT THIS PHASE MUST RUN AFTER: the complete pose walk. Not the pose
    // SEED -- the full pose, every constraint included, and the driver
    // constraints in particular. the conventional pose drivers are hidden joints
    // orient-constrained to the bone that carries everything, so that the
    // parent subtracts the twist back out and the driver's local rotation is
    // the swing alone; run this before that constraint and every driver reads
    // its seed, which is its rest, and every neutral weighs 1.000000 forever
    // however the rig is posed -- a failure that looks exactly like success.
    // The frame lookup below is what says so out loud: a driver with no
    // published FINAL frame has not been posed, and its interpolator
    // diagnoses and publishes zeros rather than quietly measuring a rest.
    const auto rotationOf =
        [&](const SdfPath &path, GfQuatd *out) {
            const auto fi = _providerIndex.find(path);
            if (fi == _providerIndex.end() || !restLive[fi->second]) {
                return false;
            }
            return RigExecFrameRotation(restFrames[fi->second], out);
        };
    const auto rotationOfFinal =
        [&](const SdfPath &path, GfQuatd *out) {
            const auto fi = _providerIndex.find(path);
            if (fi == _providerIndex.end() || !finalLive[fi->second]) {
                return false;
            }
            return RigExecFrameRotation(finalFrames[fi->second], out);
        };

    std::vector<double> weights;
    for (const _PoseInterpolator &interpolator : _poseInterpolators) {
        const UsdPrim prim = _stage->GetPrimAtPath(interpolator.prim);
        const bool enabled =
            _ResolvedRead(_resolvedInputs, prim, "inputs:enabled", true, time);

        // Publishes into BOTH: _resolvedInputs is what a consumer's read
        // resolves through (a RigExecBlendInput's inputs:weight follows its
        // single authored connection to <pose>.outputs:weight and finds the
        // value at that path), and movedProperties is what makes the number
        // observable to a host, a test and the picker.
        const auto publish = [&](const SdfPath &path, float value) {
            _resolvedInputs.SetProperty(path, VtValue(value));
            pose->movedProperties[path] = VtValue(value);
        };
        const auto publishAllZero = [&]() {
            for (const SdfPath &path : interpolator.poseWeights) {
                publish(path, 0.0f);
            }
        };
        // A disabled POSE publishes a hard zero whatever else happens: it was
        // left out of the solve, so it has no weight to be told.
        for (const SdfPath &path : interpolator.disabledPoseWeights) {
            publish(path, 0.0f);
        }
        if (!enabled) {
            // Shape-preserving enable: a corrective that is off has to be
            // off, not frozen at its last value.
            publishAllZero();
            continue;
        }

        GfQuatd driverFinal(1.0), driverRest(1.0);
        GfQuatd parentFinal(1.0), parentRest(1.0);
        if (!rotationOfFinal(interpolator.driver, &driverFinal) ||
            !rotationOf(interpolator.driver, &driverRest) ||
            (!interpolator.driverParent.IsEmpty() &&
             (!rotationOfFinal(interpolator.driverParent,
                          &parentFinal) ||
              !rotationOf(interpolator.driverParent,
                          &parentRest)))) {
            pose->diagnostics.push_back(
                "pose interpolator " + interpolator.prim.GetString() +
                " has no usable frame for its driver " +
                interpolator.driver.GetString() +
                " after the pose walk; its weights are zero this generation");
            publishAllZero();
            continue;
        }

        // The driver's LOCAL rotation relative to its own REST, which is what
        // every authored pose is measured from and why a rig standing still
        // reads its neutral at 1.000000.
        //
        //   local = parent^-1 * world       (row-vector; see RigExecFrameRotation)
        //   delta = restLocal^-1 * local
        //
        // Exactly the `neutral^-1 * pose` the authored quaternions were
        // rebased by (schema RigExecPose.rigExec:rotation), and therefore
        // directly comparable to them. The rest local is taken from the
        // EVALUATED rest frames rather than reconstructed from the driver's
        // authored rest:space: the two agree on a rig whose rest avars are
        // default, and the evaluated pair also carries the rest avars and any
        // intervening plain Xform, which the authored matrix alone does not.
        const GfQuatd local = parentFinal.GetInverse() * driverFinal;
        const GfQuatd restLocal = parentRest.GetInverse() * driverRest;
        const GfQuatd delta = (restLocal.GetInverse() * local).GetNormalized();

        // Through the euler, not around it: rbf_evaluate -- which is what
        // tools/biped/verify_psd.py computes its expected weights with --
        // takes an euler and converts it back inside Evaluate. Taking the same
        // route makes the gate's numbers and the engine's the same
        // floating-point values and not merely the same rotation.
        interpolator.solver.Evaluate(
            RigExecRbfEulerFromQuaternion(delta), nullptr, &weights,
            interpolator.allowNegativeWeights);
        if (weights.size() != interpolator.poseWeights.size()) {
            pose->diagnostics.push_back(
                "pose interpolator " + interpolator.prim.GetString() +
                " solved " + std::to_string(weights.size()) +
                " weights for " +
                std::to_string(interpolator.poseWeights.size()) + " poses");
            publishAllZero();
            continue;
        }
        for (size_t i = 0; i < weights.size(); ++i) {
            // float, not double, and that is load-bearing: a consumer reads
            // inputs:weight as a float, and RigExecResolvedInputs::Get answers
            // only the type the VtValue actually holds -- a double here would
            // miss, fall on through the connection walk, and be answered with
            // the authored zero with nothing reported anywhere.
            publish(interpolator.poseWeights[i],
                    static_cast<float>(weights[i]));
        }
    }
}

std::vector<std::string>
RigExecRigEvaluator::_ApplyDerivedStartFrames()
{
    // Process-wide: two evaluators compiling concurrently (two characters
    // sharing one stage) must not interleave session-layer writes, and USD
    // layers are not thread-safe. Microseconds per compile.
    static std::mutex derivedOpinionsMutex;
    std::lock_guard<std::mutex> lock(derivedOpinionsMutex);

    static const TfToken startFrameRel("rigExec:startFrame");
    static const TfToken startFramePolicy("rigExec:startFramePolicy");
    static const TfToken jointsRel("rigExec:joints");
    static const TfToken jointType("RigExecJoint");
    static const TfToken controlType("RigExecControl");
    static const TfToken policyNone("none");
    static const TfToken policyParent("parent");

    SdfLayerHandle session = _stage->GetSessionLayer();
    // Collected, not emitted: nothing that can Send -- TF_WARN included --
    // may run inside the notice block below, so the caller emits these
    // after this returns (both blocks then closed).
    std::vector<std::string> warnings;
    auto warn = [&warnings](const std::string &message) {
        warnings.push_back(message);
    };

    // NO NOTICE may leave this function. It runs inside Compile, and the
    // imaging registry registers its ObjectsChanged listener BEFORE
    // Compile and holds a non-recursive mutex across the whole call, so a
    // notice fired here re-enters the registry on its own held mutex
    // (measured: an access violation in _OnObjectsChanged on the first
    // usdview activation of a policy-carrying rig). TfNotice::Block
    // swallows the send while the ChangeBlock still batches the Sdf-side
    // work. Sound because the in-flight compile is the only reader of
    // these opinions: the structure digest runs after this returns, and
    // every Compile rebuilds the epoch (fresh taps) rather than
    // invalidating the old one, so no cached exec value can strand on the
    // swallowed send. Block is thread-scoped, and this runs
    // single-threaded, before the digest dispatch.
    TfNotice::Block noticeBlock;
    SdfChangeBlock block;
    // Retract this evaluator's previous opinions first, surgically: only
    // OUR tracked provider leaves each session list, so a hand-authored
    // session opinion on the same relationship survives. Anything still
    // composed afterwards is not ours, which is exactly the "authored
    // wins" test the derivation below applies.
    for (const auto &[solverPath, ourProvider] : _derivedStartFrames) {
        SdfRelationshipSpecHandle spec = session->GetRelationshipAtPath(
            solverPath.AppendProperty(startFrameRel));
        if (!spec) {
            continue;
        }
        bool present = false;
        // Explicit items only: our writes are SetTargets (explicit), and
        // a user's prepended/appended opinions are not ours to inspect.
        const auto explicitItems =
            spec->GetTargetPathList().GetExplicitItems();
        for (size_t i = 0, n = explicitItems.size(); i < n; ++i) {
            if (explicitItems[i] == ourProvider) {
                present = true;
                break;
            }
        }
        if (!present) {
            continue;
        }
        spec->RemoveTargetPath(ourProvider);
        if (!spec->HasTargetPathList()) {
            if (SdfPrimSpecHandle primSpec =
                    session->GetPrimAtPath(solverPath)) {
                primSpec->RemoveProperty(spec);
            }
        }
    }
    _derivedStartFrames.clear();

    UsdPrim rig = _stage->GetPrimAtPath(_rigPath);
    if (rig) {
        UsdEditContext sessionCtx(_stage, session);
        for (const UsdPrim &prim : UsdPrimRange(rig)) {
            if (prim.GetTypeName() != "RigExecFkChain") {
                continue;
            }
            TfToken policy;
            prim.GetAttribute(startFramePolicy).Get(&policy);
            if (policy.IsEmpty()) {
                policy = policyNone;
            }
            if (policy == policyNone) {
                continue;
            }
            const SdfPath solverPath = prim.GetPath();
            if (policy != policyParent) {
                warn(solverPath.GetString() +
                     " has rigExec:startFramePolicy '" +
                     policy.GetString() +
                     "', expected 'none' or 'parent'; solving absolute.");
                continue;
            }
            SdfPathVector composed;
            prim.GetRelationship(startFrameRel).GetTargets(&composed);
            if (!composed.empty()) {
                continue;  // Authored (asset or session) always wins.
            }
            SdfPathVector joints;
            prim.GetRelationship(jointsRel).GetTargets(&joints);
            if (joints.empty()) {
                continue;  // A jointless chain hangs from nothing.
            }
            // The inference: nearest namespace ancestor of the chain's
            // joints that is a joint or control. Structural ancestry
            // only -- GetParentPath, never a name -- so a reparented
            // chain follows its new parent with no authoring change.
            UsdPrim first =
                _stage->GetPrimAtPath(joints[0].GetPrimPath());
            SdfPath provider;
            for (SdfPath a = first ? first.GetPath().GetParentPath()
                                   : SdfPath::EmptyPath();
                 !a.IsEmpty() && a != SdfPath::AbsoluteRootPath();
                 a = a.GetParentPath()) {
                if (a == _rigPath) {
                    break;
                }
                const UsdPrim ancestor = _stage->GetPrimAtPath(a);
                if (!ancestor) {
                    continue;
                }
                const TfToken type = ancestor.GetTypeName();
                if (type == jointType || type == controlType) {
                    provider = a;
                    break;
                }
            }
            if (provider.IsEmpty()) {
                warn(solverPath.GetString() +
                     " has rigExec:startFramePolicy 'parent' but no "
                     "RigExecJoint/RigExecControl ancestor; solving "
                     "absolute.");
                continue;
            }
            bool shared = true;
            for (const SdfPath &j : joints) {
                const SdfPath jp = j.GetPrimPath();
                if (jp == provider || !jp.HasPrefix(provider)) {
                    shared = false;
                    break;
                }
            }
            if (!shared) {
                warn(solverPath.GetString() +
                     " has rigExec:startFramePolicy 'parent' but its "
                     "joints span providers; solving absolute.");
                continue;
            }
            prim.GetRelationship(startFrameRel)
                .SetTargets(SdfPathVector{provider});
            _derivedStartFrames[solverPath] = provider;
        }
    }
    return warnings;
}

bool
RigExecRigEvaluator::Compile(std::vector<std::string> *errors)
{
    RIGEXEC_PROFILE_SCOPE_CAT(_profiler, "Compile", "compile");
    // DROP THE GIL FOR THE WHOLE CALL. Compile dispatches exec work to
    // TBB workers, and the first ExecUsdSystem::PrepareRequest in a
    // process lazily loads the exec definition plugins -- PlugPlugin::
    // Load -> TfDlopen -> TfScriptModuleLoader -> TfPyLock -- which
    // needs the GIL. A Python caller (usdview, the bindings) holds the
    // GIL across this call, so a worker that WINS the race to that lazy
    // load blocks on the GIL while this thread blocks in
    // WorkDispatcher::Wait on the worker, and every other TBB worker
    // piles up behind the plugin-load mutex the stalled one is holding.
    // MEASURED: 5 hangs in 12 runs of testRigExecStageEdits, 0 in 15
    // with RIGEXEC_ENABLE_PARALLEL_EVAL=0. Intermittent because it is a
    // race for who reaches the registry first.
    //
    // Scoped to the whole function, not to the Wait() calls: a
    // WorkDispatcher also waits in its DESTRUCTOR, and Compile has
    // early returns between the warm-up Run() and its Wait(), so a
    // narrower guard would leave those joins exposed. Safe because
    // libs/rigExec calls no Python at all, and the macro compiles to
    // nothing when Python is not initialized.
    TF_PY_ALLOW_THREADS_IN_SCOPE();
    // Sequential region stamps: Compile is flat code with early returns,
    // so RAII scopes cannot span its phases; each stamp closes the
    // previous region and opens the next. One branch when disabled.
    //
    // The clock starts on the FIRST line of the body rather than beside the
    // first stamped phase, where it used to start. Everything above that
    // point could only ever measure as zero, and the trace had 6.1 ms sitting
    // between the Compile scope opening and DiscoverValidate.Validate
    // starting that no row claimed. Starting here hands that gap to
    // Compile.Prologue below instead of losing it in the parent scope.
    const bool profileCompile = _profiler.IsEnabled();
    uint64_t compileRegionStart =
        profileCompile ? RigExecProfiler::NowUs() : 0;
    auto stampCompileRegion = [&](const char *name) {
        if (!profileCompile) {
            return;
        }
        const uint64_t now = RigExecProfiler::NowUs();
        _profiler.Record(name, "compile", compileRegionStart, now);
        compileRegionStart = now;
    };
    // The program describes the epoch this call is about to replace, so it
    // stops being the rig's program here: a Compile that fails and restores
    // the previous epoch returns before the rebuild at the tail, this local
    // goes out of scope, and the rig is left dynamic -- slower and never
    // wrong. Retired rather than destroyed, because its persistent GEOMETRY
    // state (which node ran with which packet) is not about the epoch: the
    // dynamic path keeps its _liveGraphs across a recompile and reconnects
    // whichever nodes survive, and the rebuild below does the same.
    std::unique_ptr<RigExecBakedProgram> retiringBakedProgram =
        std::move(_bakedProgram);
    _bakedProgramStale = false;
    // A new epoch is a new question: whatever refused the last one said
    // nothing about this one.
    _bakeRefused = false;
    _bakeRefusalReasons.clear();
    stampCompileRegion("Compile.Prologue");
    // Derived start-frame targets first, single-threaded, before the digest
    // dispatch and every parallel stage read below: the inference is a pure
    // function of the asset state, so the digest then covers it like any
    // other composed opinion, and the scheduler, exec, and the bake all see
    // it as authored. Returned warnings are emitted here, past the
    // derivation's notice block: both channels on purpose, like the
    // transform-authority pass -- TF_WARN is what a host surfaces to the
    // author, the errors vector is what a test can read.
    for (const std::string &message : _ApplyDerivedStartFrames()) {
        if (errors) {
            errors->push_back("warning: " + message);
        }
        TF_WARN("%s", message.c_str());
    }
    stampCompileRegion("Compile.DerivedStartFrames");

    // The structure digest is a pure read of the composed stage that boils it
    // down to one number, and that number is not consulted until the epoch is
    // committed far below: nothing in between reads it, and compile authors
    // nothing to the stage for it to miss past the derived opinions above
    // (which the digest reads, deterministically, as composed targets). So
    // it runs beside the WHOLE of
    // compile rather than beside only its tail.
    //
    // MEASURED (biped_stack_anim, 201.3 ms compile): dispatched at the old
    // site -- after DiscoverValidate, at t=35.7 ms -- the digest's 107 ms
    // landed at t=143.1 ms, while the main thread reached the join below at
    // t=122.3 ms and then sat idle for 23.4 ms. Compile was paying for the
    // digest after all, in waiting rather than in work. Dispatched here it
    // lands around t=107 ms, comfortably ahead of the join.
    //
    // Safe to hoist past DiscoverValidate because that phase AUTHORS NOTHING:
    // it reads the composed stage and reports, so there is no edit for the
    // digest to race. And a rig malformed enough for DiscoverValidate to
    // reject is still one _ComputeStructureDigest reads without complaint --
    // every lookup in it goes through GetPrimAtPath/GetAttribute/Get, which
    // hand back invalid objects that the code already tests for, rather than
    // throwing. The single precondition it cannot survive is a null _stage,
    // which the old site sat downstream of; hence the guard in the lambda.
    // The 0 that guard leaves behind is never read -- Compile returns false
    // on a null stage long before the commit below.
    //
    // newDigest is declared before the dispatcher so it outlives it, and
    // WorkDispatcher's destructor waits -- which is what covers the early
    // returns between here and the join, now the whole of compile rather
    // than just its tail. Declared AFTER retiringBakedProgram so it is still
    // destroyed first: the digest thread is joined before the retired
    // program is torn down underneath it.
    size_t newDigest = 0;
    WorkDispatcher digestDispatcher;
    const auto computeDigest = [this, &newDigest]() {
        newDigest = _stage ? _ComputeStructureDigest() : 0;
    };
    if (RigExecParallelEvaluationEnabled()) {
        digestDispatcher.Run(computeDigest);
    } else {
        computeDigest();
    }
    // Zero-width when the digest is dispatched, which is the point: it marks
    // WHERE the digest was launched so the trace can be read against the
    // worker row. With RIGEXEC_ENABLE_PARALLEL_EVAL=0 the call above ran
    // inline and this stamp measures the whole of it, exactly as it did at
    // the old site.
    stampCompileRegion("Compile.StructureDigest");

    // Whether to leave the dynamic-only requests unprepared; see
    // _execPrepDeferred for which three those are and why the others are
    // not among them. Read once, here, so that every site below agrees --
    // half a deferred epoch would be a request nothing prepares and nothing
    // knows to.
    const bool deferExecPrep =
        _PeekEvaluationMode() == RigExecEvaluationMode::Baked;
    // The parts WITHIN those regions, for the same reason and on the same
    // clock: a phase that takes a fifth of the compile says nothing about
    // which of its passes to go and look at. Strictly nested inside the
    // stamps above, so the trace shows them as children rather than as a
    // second, competing set of rows. Closed at each phase boundary so no
    // block spans two phases, and on destruction so an early return
    // cannot leave one open.
    RigExecProfilePhases compileBlocks(&_profiler, "compile");

    auto reportError = [errors](const std::string &message) {
        if (errors) {
            errors->push_back(message);
        }
    };

    compileBlocks.Next("DiscoverValidate.Validate");
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
    std::string adjustmentReadError;
    if (!_ValidateAdjustmentPoseConsumers(_stage, rig, &adjustmentReadError)) {
        reportError(adjustmentReadError);
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

    // Pose interpolators, discovered and SOLVED here. Solving is the whole
    // reason this is compile-time work: inverting the matrix of every pose's
    // kernel value at every other pose is a constant of the authored data,
    // and doing it per frame would be inverting the same matrix 326 times a
    // second to be handed the same answer.
    std::vector<_PoseInterpolator> newPoseInterpolators;
    {
        std::string interpolatorError;
        if (!_CompilePoseInterpolators(newJointPaths, newControlPaths,
                                       &newPoseInterpolators, errors,
                                       &interpolatorError)) {
            reportError(interpolatorError);
            return false;
        }
    }

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
        // Resolving a prim's purpose walks up the namespace to the first
        // authored opinion, and this pass asks for it once per provider and
        // then again for every descendant of every provider -- so a joint
        // deep in a chain is resolved once per ancestor provider. Purpose is
        // a pure function of the composed stage, which does not change while
        // a compile runs, so resolve each prim once.
        std::unordered_map<SdfPath, TfToken, SdfPath::Hash> purposeCache;
        const auto resolvedPurpose = [&purposeCache](const UsdPrim &prim) {
            auto it = purposeCache.find(prim.GetPath());
            if (it == purposeCache.end()) {
                it = purposeCache.emplace(
                    prim.GetPath(),
                    UsdGeomImageable(prim).ComputePurpose()).first;
            }
            return it->second;
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
            // An Xformable BETWEEN the asset root and the provider used to
            // warn here, because its transform was dropped. It is now
            // composed at evaluation, by _ComposeInterveningXforms, so there
            // is nothing left to report: placing a rig -- or one leg of an
            // assembly -- under an Xform inside the asset is a supported
            // shape, and warning ten times per compile about a configuration
            // that works is noise nobody can act on.
            //
            // The check above it stays. An op authored on the PROVIDER is
            // still not a transform authority, which is a different claim
            // and still true.

            // Hoisted out of the walk: the provider's own purpose is the
            // same for every one of its descendants.
            const TfToken providerPurpose = resolvedPurpose(prim);

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
                    const TfToken descendantPurpose =
                        resolvedPurpose(descendant);
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
    compileBlocks.Next("DiscoverValidate.AggregateProviders");
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

    compileBlocks.Next("DiscoverValidate.WarmupDispatch");
    // Warm the shared exec network, off the critical path.
    //
    // Every request this compile prepares -- fourteen solver batches, the
    // first-frame pose, the main epoch request, the guides -- compiles into the SAME
    // exec network for this stage, and whichever request asks for a provider
    // first pays to compile it. Asking for all of them at once, here, builds
    // that shared network in one wide parallel round rather than in a series
    // of narrow ones, and it runs beside the scheduling work below that has
    // to happen anyway. It is pure preparation: no value is read from it, and
    // a request that cannot be built valid simply leaves the real
    // preparations to compile what they need, and to report their own
    // failure.
    //
    // The tap set is constructed here and destroyed here, on the compiling
    // thread: its constructor and destructor mutate the tap context's client
    // set, which is not guarded. Only Prepare() runs on the task, and nothing
    // between the Run() below and the Wait() before the first real
    // preparation enters exec at all.
    auto warmupTaps = std::make_unique<RigExecTapSet>(_stage);
    for (const std::vector<SdfPath> *providers :
         {&newJointPaths, &newControlPaths, &newVolumeWeightPaths}) {
        for (const SdfPath &path : *providers) {
            warmupTaps->Add(
                RigExecValueAddress::Prim(path, _computePointFrame));
            warmupTaps->Add(RigExecValueAddress::Prim(
                path, TfToken("computeRestFrame")));
        }
    }
    for (const SdfPath &path : solverArrayPaths) {
        warmupTaps->Add(
            RigExecValueAddress::Prim(path, _computePointFrameArray));
    }
    WorkDispatcher warmupDispatcher;
    const auto prepareWarmupTaps = [this, &warmupTaps]() {
        RIGEXEC_PROFILE_SCOPE_CAT(_profiler, "TapPrepare warmup", "compile");
        warmupTaps->Prepare();
    };
    // The kill switch runs the same work on this thread instead of skipping
    // it: the warm-up is what the real preparations below are cheap because
    // of, so dropping it would change what is being measured.
    if (RigExecParallelEvaluationEnabled()) {
        warmupDispatcher.Run(prepareWarmupTaps);
    } else {
        prepareWarmupTaps();
    }

    compileBlocks.Next("DiscoverValidate.MoverDiscovery");
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
            if (record.schemaType == "RigExecSkinMover") {
                std::string error;
                if (!_ValidateSkinMover(prim, record, &error)) {
                    reportError(error);
                    return false;
                }
            }
            // Blend packets are specialized per application/target below,
            // so each fan-out target gets its own authored-base deltas.
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
                    TfToken("RigExecCurvenetAdjusterMover"),
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
                 record.schemaType == "RigExecCurvenetMover" ||
                 record.schemaType == "RigExecCurvenetAdjusterMover") &&
                record.targets.size() != 1) {
                reportError(record.schemaType.GetString() + " " +
                            prim.GetPath().GetString() +
                            " must have exactly one canonical points "
                            "target in v0.1 (multi-target fan-out would "
                            "alias mover-level parameters)");
                return false;
            }
            if (record.schemaType == "RigExecCurvenetAdjusterMover") {
                std::string error;
                if (!RigExecValidateCurvenetAdjuster(prim, record.targets[0], &error)) {
                    reportError(prim.GetPath().GetString() + ": " + error);
                    return false;
                }
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
                        // A double target is a control's avar: the chain
                        // computes in float and publishes the double, so a
                        // hidden control's channels can be driven by keys.
                        expected = "float/double";
                        typeOk = valueType == SdfValueTypeNames->Float ||
                                 valueType == SdfValueTypeNames->Double;
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
                    if (op == RigExecPropertyOp::Curve) {
                        VtArray<GfVec2f> keys;
                        const UsdAttribute keysAttr =
                            prim.GetAttribute(TfToken("inputs:keys"));
                        if (type != "RigExecFloatMathMover") {
                            reportError(
                                type.GetString() + " " +
                                prim.GetPath().GetString() +
                                ": rigExec:operation 'curve' is defined only "
                                "for RigExecFloatMathMover");
                            return false;
                        }
                        VtArray<GfVec2f> tangents;
                        if (const UsdAttribute t = prim.GetAttribute(
                                TfToken("inputs:tangents"))) {
                            t.Get(&tangents);
                        }
                        if (!keysAttr || !keysAttr.Get(&keys) ||
                            keys.empty() ||
                            !RigExecValidateLinearKeys(
                                keys.cdata(), keys.size()) ||
                            (!tangents.empty() &&
                             tangents.size() != keys.size())) {
                            reportError(
                                type.GetString() + " " +
                                prim.GetPath().GetString() +
                                ": rigExec:operation 'curve' needs "
                                "inputs:keys with finite keys strictly "
                                "increasing in input, and inputs:tangents "
                                "empty or one per key");
                            return false;
                        }
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

            if (record.schemaType == "RigExecBlendShapeMover") {
                TfToken deltaSpace("target");
                prim.GetAttribute(TfToken("rigExec:deltaSpace")).Get(&deltaSpace);
                if (deltaSpace != "target" && deltaSpace != "surfaceFrame") {
                    reportError(prim.GetPath().GetString() + ": invalid blend deltaSpace");
                    return false;
                }
                if (deltaSpace == "surfaceFrame") {
                    SdfPathVector targets;
                    prim.GetRelationship(TfToken("rigExec:moves")).GetTargets(&targets);
                    for (const auto &target : targets) {
                        if (!_stage->GetPrimAtPath(target.GetPrimPath()).IsA<UsdGeomMesh>()) {
                            reportError(prim.GetPath().GetString() + ": surfaceFrame blend requires mesh targets");
                            return false;
                        }
                    }
                }
                SdfPathVector inputs;
                prim.GetRelationship(TfToken("rigExec:blendInputs"))
                    .GetTargets(&inputs);
                for (const SdfPath &inputPath : inputs) {
                    const UsdPrim input = _stage->GetPrimAtPath(inputPath);
                    if (!input) continue;
                    SdfPathVector samples;
                    input.GetRelationship(TfToken("rigExec:samples"))
                        .GetTargets(&samples);
                    for (const SdfPath &samplePath : samples) {
                        const UsdPrim sample = _stage->GetPrimAtPath(samplePath);
                        if (!sample) continue;
                        // A sample states its shape exactly one way. Both
                        // authored is refused rather than resolved by
                        // precedence: two shapes that disagree with a silent
                        // winner is worse than either shape being wrong.
                        SdfPathVector densePoints, sparseShape;
                        if (UsdRelationship rel = sample.GetRelationship(
                                TfToken("rigExec:targetPoints"))) {
                            rel.GetTargets(&densePoints);
                        }
                        if (UsdRelationship rel = sample.GetRelationship(
                                TfToken("rigExec:blendShape"))) {
                            rel.GetTargets(&sparseShape);
                        }
                        if (!densePoints.empty() && !sparseShape.empty()) {
                            reportError(samplePath.GetString() +
                                ": blend sample authors both rigExec:targetPoints"
                                " and rigExec:blendShape; exactly one is allowed");
                            return false;
                        }
                        if (!sparseShape.empty()) {
                            if (sparseShape.size() != 1 ||
                                !_stage->GetPrimAtPath(
                                    sparseShape[0].GetPrimPath())
                                     .IsA<UsdSkelBlendShape>()) {
                                reportError(samplePath.GetString() +
                                    ": rigExec:blendShape must name exactly one"
                                    " UsdSkelBlendShape prim");
                                return false;
                            }
                            // No phased read to validate: the offsets are
                            // authored data, not a chain revision, so there
                            // is no preceding or final version of them.
                            continue;
                        }
                        RigExecReadPhase phase;
                        std::string error;
                        const UsdAttribute legacy = sample.GetAttribute(
                            TfToken("rigExec:pointsReadPhase"));
                        if (!RigExecResolveReadPhase(
                                sample.GetRelationship(TfToken("rigExec:targetPoints")),
                                "rigExec:pointsReadPhase", &phase, &error) ||
                            (legacy && legacy.GetNumTimeSamples() != 0)) {
                            reportError(samplePath.GetString() +
                                ": blend sample points read phase must be a valid static phase" +
                                (error.empty() ? std::string() : ": " + error));
                            return false;
                        }
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

    compileBlocks.Next("DiscoverValidate.OutputCheck");
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
            const bool isSkin = m.schemaType == "RigExecSkinMover";
            if (m.schemaType != "RigExecMatrixMover" && !isSkin) {
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
            if (UsdRelationship rel = prim.GetRelationship(TfToken(
                    isSkin ? "rigExec:influences" : "rigExec:transform"))) {
                rel.GetTargets(&transforms);
            }
            if (!isSkin) {
                SdfPathVector spaces;
                if (UsdRelationship rel = prim.GetRelationship(
                        TfToken("rigExec:transformSpace"))) {
                    rel.GetTargets(&spaces);
                }
                transforms.insert(transforms.end(), spaces.begin(),
                                  spaces.end());
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

    compileBlocks.Next("DiscoverValidate.SolverJointBinding");
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
    //
    // rigExec:joints is an ordered WRITE, not an exclusive claim (spec §4.2,
    // "Solvers stack"), so the value is the ordered stack of writers rather
    // than one owner. Index 0 writes first; the last entry supplies the
    // joint's base frame. The order is settled below, once the POSE graph is
    // complete -- which is later than the solver DAG, because a pair of
    // writers can be ordered by a solver -> constraint -> solver path and by
    // nothing else: data flow decides any pair it orders, and the solver
    // stack ordinal breaks every remaining tie.
    //
    // The ordered solver walk is hoisted here because four passes need it --
    // this validation, the consumed-solver relaxation, the solver DAG and the
    // stack ordinal -- and it is a full UsdPrimRange over the rig each time
    // (critique: "_DiscoverAggregateSolvers is already walked three times").
    const std::vector<UsdPrim> orderedSolvers =
        _DiscoverAggregateSolvers(_stage, _rigPath);
    // Solver -> its position in the SOLVER STACK ORDINAL: the reverse of the
    // composed pre-order of the whole rig, which is _GetMoverExecutionOrder's
    // rule (see the comment at its definition) applied to the solver set
    // instead of the Movers subtree. Bottom composed sibling first, a parent
    // after its descendants -- so "the bottom one executes first" reads the
    // same whichever kind of node a rigger is looking at, and a reorder in
    // usdview reorders the stack.
    //
    // It is a pre-order of the WHOLE rig, not a sibling order: two solvers in
    // different scopes are ordered by where their scopes sit, and a nested
    // solver comes before its ancestor. This is deliberately the same walk
    // the epoch digest uses (see the aggregate-solver segment of the digest
    // below), so an order edit can never retain the old digest while
    // executing a different stack.
    std::map<SdfPath, int> solverStackOrdinal;
    {
        int ordinal = 0;
        for (auto it = orderedSolvers.rbegin(); it != orderedSolvers.rend();
             ++it) {
            solverStackOrdinal[it->GetPath()] = ordinal++;
        }
    }
    std::map<SdfPath, std::vector<std::pair<SdfPath, int>>> newJointBinding;
    {
        {
            const std::vector<UsdPrim> &solvers = orderedSolvers;
            // Cardinality attributes must be static (compile-time
            // structural) for EVERY aggregate solver under the rig, not
            // only joint-bearing ones: a non-joint Twist/Ribbon feeding a
            // joint-bearing Blend still determines that Blend's element
            // count, so a time-sampled cardinality would silently shift a
            // blend-bound joint's frame. `uniform` is only
            // a hint; reject samples explicitly.
            for (const UsdPrim &solver : solvers) {
                const TfToken t = solver.GetTypeName();
                // rigExec:upperLength/rigExec:lowerLength no longer exist
                // in the schema -- bone lengths are measured from the bound
                // joints' rests. An asset saved against the old schema can
                // still carry one as a custom property, where it would be
                // silently inert; that used to freeze the bone, so a rig
                // relying on it would change shape with no explanation.
                // Fail loudly instead and name the knob that replaced it.
                if (t == "RigExecTwoBoneIk") {
                    static const char *const lengthAttrs[2] = {
                        "rigExec:upperLength", "rigExec:lowerLength"};
                    for (const char *name : lengthAttrs) {
                        const UsdAttribute a =
                            solver.GetAttribute(TfToken(name));
                        if (a && a.HasAuthoredValueOpinion()) {
                            reportError(
                                solver.GetPath().GetString() + ": " + name +
                                " was removed from the schema; bone lengths "
                                "are measured from the bound joints' rest "
                                "positions. Remove this opinion and author " +
                                name + "Offset to adjust the measured bone");
                            return false;
                        }
                    }
                }
                std::vector<const char *> cardinalityAttrs;
                if (t == "RigExecTwistDistribution") {
                    cardinalityAttrs = {"rigExec:count", "rigExec:weights"};
                } else if (t == "RigExecRibbon") {
                    cardinalityAttrs = {"rigExec:sampleCount"};
                } else if (t == "RigExecSplineIk") {
                    // Not cardinality (the joints list is), but a static
                    // parallel array: a sampled one would let the
                    // per-joint weight silently detach from the chain.
                    cardinalityAttrs = {"rigExec:volumeWeights"};
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
            // Solvers whose aggregate is READ by another solver. Such a
            // solver does not pose anything itself -- the consumer that
            // reads it is what writes to the joints -- so its
            // rigExec:joints is a rest reference, not an output claim.
            //
            // That is what lets an IK feeding an IK/FK blend name the
            // chain it solves for: the blend still claims those joints
            // exclusively, while the IK gets the joint REST frames it
            // needs to measure its bone lengths from. Without this the
            // IK could not name them at all ("posed by two solvers") and
            // had no bones to measure.
            std::set<SdfPath> consumedSolvers;
            std::set<SdfPath> posedByUnconsumed;
            {
                const std::vector<UsdPrim> &allSolvers = orderedSolvers;
                std::set<SdfPath> solverPaths;
                for (const UsdPrim &solver : allSolvers) {
                    solverPaths.insert(solver.GetPath());
                }
                for (const UsdPrim &solver : allSolvers) {
                    for (const UsdRelationship &rel :
                         solver.GetRelationships()) {
                        if (rel.GetName() == "rigExec:joints") {
                            continue;
                        }
                        SdfPathVector targets;
                        rel.GetTargets(&targets);
                        for (const SdfPath &target : targets) {
                            // GetPrimPath(), not the raw target: the DAG pass
                            // below resolves a property spelling
                            // (</Rig/Solvers/IK.rigExec:aggregate>) to its
                            // prim, and a read that creates a dependency edge
                            // but does not mark the target consumed would
                            // leave the IK writing as well as feeding the
                            // blend -- a stack whose meaning then depended on
                            // namespace order.
                            const SdfPath targetPrim = target.GetPrimPath();
                            if (targetPrim != solver.GetPath() &&
                                solverPaths.count(targetPrim)) {
                                consumedSolvers.insert(targetPrim);
                            }
                        }
                    }
                }
                // "Consumed" only means anything in a DAG: in a cycle every
                // solver reads another, nothing is unconsumed, and the
                // relaxation below would collapse. The full dependency
                // check downstream also folds in joint-binding edges and so
                // cannot run until claims are known -- but a cycle among
                // solver-to-solver edges alone is already decidable here,
                // and reporting it now keeps a cyclic rig from being
                // diagnosed as a bogus double claim.
                {
                    std::map<SdfPath, size_t> pending;
                    std::map<SdfPath, std::vector<SdfPath>> consumers;
                    std::vector<SdfPath> ready;
                    for (const UsdPrim &solver : allSolvers) {
                        std::set<SdfPath> deps;
                        for (const UsdRelationship &rel :
                             solver.GetRelationships()) {
                            if (rel.GetName() == "rigExec:joints") {
                                continue;
                            }
                            SdfPathVector targets;
                            rel.GetTargets(&targets);
                            for (const SdfPath &target : targets) {
                                const SdfPath prim = target.GetPrimPath();
                                if (prim != solver.GetPath() &&
                                    solverPaths.count(prim)) {
                                    deps.insert(prim);
                                }
                            }
                        }
                        pending[solver.GetPath()] = deps.size();
                        if (deps.empty()) {
                            ready.push_back(solver.GetPath());
                        }
                        for (const SdfPath &dep : deps) {
                            consumers[dep].push_back(solver.GetPath());
                        }
                    }
                    size_t scheduled = 0;
                    while (!ready.empty()) {
                        scheduled += ready.size();
                        std::vector<SdfPath> next;
                        for (const SdfPath &solver : ready) {
                            for (const SdfPath &consumer : consumers[solver]) {
                                if (--pending[consumer] == 0) {
                                    next.push_back(consumer);
                                }
                            }
                        }
                        ready.swap(next);
                    }
                    if (scheduled != pending.size()) {
                        std::string paths;
                        for (const auto &[solver, count] : pending) {
                            if (count) {
                                paths += " " + solver.GetString();
                            }
                        }
                        reportError("solver dependency cycle among:" + paths);
                        return false;
                    }
                }

                // Joints that a solver nobody reads writes to. Only these
                // are already spoken for; a consumed solver still POSES
                // any joint no such solver names, so feeding a blend does
                // not silently stop it driving its own extra outputs.
                for (const UsdPrim &solver : allSolvers) {
                    if (consumedSolvers.count(solver.GetPath())) {
                        continue;
                    }
                    if (const UsdRelationship rel = solver.GetRelationship(
                            TfToken("rigExec:joints"))) {
                        SdfPathVector targets;
                        rel.GetTargets(&targets);
                        posedByUnconsumed.insert(targets.begin(),
                                                 targets.end());
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
                    } else if (type == "RigExecSplineIk") {
                        // The chain IS the cardinality: one frame per
                        // joints entry. A remap must then be a permutation
                        // of the chain slots, and the per-joint volume
                        // weights must be parallel to it.
                        knownCount = static_cast<int>(jointTargets.size());
                        if (!jointElements.empty()) {
                            std::vector<bool> filled(jointTargets.size(), false);
                            for (int e : jointElements) {
                                if (e >= 0 && e < knownCount) {
                                    if (filled[e]) {
                                        reportError(
                                            who + ": rigExec:jointElements "
                                            "fills chain slot " +
                                            std::to_string(e) + " twice");
                                        return false;
                                    }
                                    filled[e] = true;
                                }
                            }
                        }
                        if (const UsdAttribute a = solver.GetAttribute(
                                TfToken("rigExec:volumeWeights"))) {
                            VtFloatArray weights;
                            a.Get(&weights);
                            if (!weights.empty() &&
                                weights.size() != jointTargets.size()) {
                                reportError(
                                    who + ": rigExec:volumeWeights length " +
                                    std::to_string(weights.size()) +
                                    " must equal rigExec:joints length " +
                                    std::to_string(jointTargets.size()) +
                                    " (or be empty)");
                                return false;
                            }
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
                                const SdfPathVector conns =
                                    _AuthoredConnections(posed);
                                if (!conns.empty()) {
                                    reportError(who + ": joint " +
                                                jointPath.GetString() +
                                                " also connects posed:space");
                                    return false;
                                }
                            }
                        }
                        // A consumed solver naming a joint that a solver
                        // nobody reads already writes to is referencing it
                        // for its REST, not claiming it: the consumer is
                        // what poses it. Any other joint it names it still
                        // poses itself.
                        if (consumedSolvers.count(solver.GetPath()) &&
                            posedByUnconsumed.count(jointPath)) {
                            continue;
                        }
                        // Two SOLVERS writing one joint is legal and stacks.
                        // One solver naming one joint TWICE is not: the two
                        // entries would collapse at runtime (candidates[] is
                        // a map, and the baked commit's slots are uniqued),
                        // and a stack edge between them would be a self-edge
                        // that Kahn reports as an unexplained pose cycle. Say
                        // what actually happened instead.
                        std::vector<std::pair<SdfPath, int>> &writers =
                            newJointBinding[jointPath];
                        for (const auto &[writer, writerElement] : writers) {
                            if (writer == solver.GetPath()) {
                                reportError(
                                    who + ": rigExec:joints names " +
                                    jointPath.GetString() +
                                    " more than once");
                                return false;
                            }
                        }
                        writers.emplace_back(solver.GetPath(), element);
                    }
                }
            }
        }
    }

    // The binding loop appends in UsdPrimRange pre-order, which is the exact
    // REVERSE of the stack, so every writer list is put into stack-ordinal
    // order here. This is the AUTHORED order, and it is not decoration: the
    // "reads the version standing before its own commit" edges built while
    // the solver DAG is assembled are read off these lists. The pass that
    // settles the stack against the finished pose graph replaces the order
    // with the one data flow and the schedule actually produce, and the two
    // agree whenever nothing orders a pair.
    for (auto &[joint, writers] : newJointBinding) {
        if (writers.size() < 2) {
            continue;  // a one-element stack has no order to get wrong
        }
        // find(), never operator[]: mutating a captured map from inside a
        // sort comparator would insert a silent 0 for any writer the ordinal
        // sweep did not see and make the comparator inconsistent. An unknown
        // writer sorts last instead.
        const auto ordinalOf = [&solverStackOrdinal](const SdfPath &path) {
            const auto it = solverStackOrdinal.find(path);
            return it == solverStackOrdinal.end()
                       ? std::numeric_limits<int>::max()
                       : it->second;
        };
        std::stable_sort(
            writers.begin(), writers.end(),
            [&ordinalOf](const std::pair<SdfPath, int> &a,
                         const std::pair<SdfPath, int> &b) {
                return ordinalOf(a.first) < ordinalOf(b.first);
            });
    }

    compileBlocks.Next("DiscoverValidate.SolverDag");
    // Compile the solver DAG, including frame inputs carried by a posed
    // namespace ancestor. Rest inputs are independent of posed outputs and
    // therefore never introduce a feedback edge (IK -> blend is legal).
    // PRODUCERS: the solvers with no position in the pose stack at all
    // (spec §4.2). Two kinds, and both are scheduled by DATA FLOW alone:
    //
    //   * a solver whose AGGREGATE another solver reads. It is already
    //     special -- the consumed-solver relaxation makes its rigExec:joints
    //     a rest reference wherever the consumer claims the same joint -- and
    //     its consumer cannot read an "earlier version" of an aggregate, so
    //     the aggregate edge decides the pair and the namespace says nothing
    //     about it. This is what keeps every IK/FK blend in the repo
    //     schedulable: reversed sibling order puts the blend BEFORE the
    //     solvers it blends, because a blend is authored last.
    //
    //     (The relaxation is per JOINT, not per solver, so a consumed solver
    //     that also writes a joint its consumer does not name still writes --
    //     `docs/examples/blend_point_frames.usda` is exactly that shape. It
    //     is still a producer: what takes it out of the stack is being read,
    //     not being joint-less.)
    //
    //   * a solver that writes no joint at all -- one whose aggregate only a
    //     geometry mover reads.
    //
    // Every consumer of the ordinal map must therefore test membership rather
    // than use operator[].
    std::set<SdfPath> aggregateProducers;
    {
        std::set<SdfPath> solverPaths;
        for (const UsdPrim &solver : orderedSolvers) {
            solverPaths.insert(solver.GetPath());
        }
        for (const UsdPrim &solver : orderedSolvers) {
            for (const UsdRelationship &rel : solver.GetRelationships()) {
                if (rel.GetName() == "rigExec:joints") continue;
                SdfPathVector targets;
                rel.GetTargets(&targets);
                for (const SdfPath &target : targets) {
                    const SdfPath prim = target.GetPrimPath();
                    if (prim != solver.GetPath() && solverPaths.count(prim)) {
                        aggregateProducers.insert(prim);
                    }
                }
            }
        }
    }
    std::set<SdfPath> jointWritingSolvers;
    for (const auto &[joint, writers] : newJointBinding) {
        for (const auto &[writer, element] : writers) {
            if (aggregateProducers.count(writer)) continue;
            jointWritingSolvers.insert(writer);
        }
    }
    // Strictly before, in the stack restricted to solvers. solverStackOrdinal
    // is the reverse composed pre-order of the whole rig, so its restriction
    // to the solvers IS the unified pose stack restricted to them; the
    // constraint half only interleaves between them and cannot reorder a
    // solver pair.
    //
    // Only a READER that is itself a stack step consults it. A PRODUCER has no
    // position for the comparison to be about, so it keeps the unconditional
    // "wait for every writer of what I read" edge it always had -- which is
    // its data-flow meaning, and which leaves a genuine loop among producers
    // (two of them reading each other's joints) a cycle, reported as one.
    const auto stackBefore = [&solverStackOrdinal](const SdfPath &a,
                                                   const SdfPath &b) {
        const auto ia = solverStackOrdinal.find(a);
        const auto ib = solverStackOrdinal.find(b);
        if (ia == solverStackOrdinal.end() ||
            ib == solverStackOrdinal.end()) {
            return false;
        }
        return ia->second < ib->second;
    };
    // "A writer waits on an EARLIER READER." The new edge class the unified
    // pose stack needs (spec §4.2): a step that reads a provider from BELOW
    // its writer reads the version standing before that write, and without an
    // edge saying so Kahn may schedule the two either way round within a level
    // and the version the reader sees becomes undefined -- a nondeterministic
    // failure, which is the worst kind here. Collected wherever a read is
    // resolved and applied once poseDependencies exists, because the two
    // halves are found on opposite sides of the constraint pass.
    //
    // (waiter, waited-on): poseDependencies[first].insert(second).
    std::vector<std::pair<SdfPath, SdfPath>> poseReverseEdges;
    std::map<SdfPath, std::set<SdfPath>> newSolverDependencies;
    // The AGGREGATE half of those edges, kept apart because the two classes
    // answer to different rules under the unified pose stack (spec §4.2): a
    // frame read is POSITIONAL -- it reads whatever stands at the reader's own
    // place in the stack -- while an aggregate read is ABSOLUTE and a
    // hierarchy that contradicts it is a compile error (checked below, once
    // the ordinal is known).
    std::map<SdfPath, std::set<SdfPath>> newSolverAggregateReads;
    {
        const std::vector<UsdPrim> &solvers = orderedSolvers;
        std::set<SdfPath> solverPaths;
        for (const UsdPrim &solver : solvers) {
            solverPaths.insert(solver.GetPath());
            newSolverDependencies[solver.GetPath()];
        }
        for (const UsdPrim &solver : solvers) {
            for (const UsdRelationship &rel : solver.GetRelationships()) {
                if (rel.GetName() == "rigExec:joints") {
                    continue;
                }
                SdfPathVector targets;
                rel.GetTargets(&targets);
                for (const SdfPath &target : targets) {
                    const SdfPath targetPrim = target.GetPrimPath();
                    if (solverPaths.count(targetPrim)) {
                        newSolverDependencies[solver.GetPath()].insert(targetPrim);
                        newSolverAggregateReads[solver.GetPath()].insert(
                            targetPrim);
                        continue;
                    }
                    const UsdPrim provider = _stage->GetPrimAtPath(targetPrim);
                    const bool frameProvider = provider &&
                        (provider.GetTypeName() == "RigExecJoint" ||
                         provider.GetTypeName() == "RigExecControl" ||
                         _IsVolumeWeightType(provider.GetTypeName()));
                    for (SdfPath path = targetPrim; !path.IsEmpty() &&
                         path != SdfPath::AbsoluteRootPath();
                         path = frameProvider ? path.GetParentPath() : SdfPath()) {
                        const auto binding = newJointBinding.find(path);
                        if (binding != newJointBinding.end()) {
                            // Every writer of this joint that precedes this
                            // solver in the stack, and NEVER this solver
                            // itself: a solver that reads a joint it also
                            // writes reads the version standing before its
                            // own commit, so it depends on the writers below
                            // it and on nothing above. (Before stacking this
                            // was a self-edge and died as a bogus "solver
                            // dependency cycle".)
                            for (const auto &[writer, writerElement] :
                                 binding->second) {
                                if (writer == solver.GetPath()) {
                                    continue;
                                }
                                if (jointWritingSolvers.count(
                                        solver.GetPath()) &&
                                    stackBefore(solver.GetPath(), writer)) {
                                    // The writer stands ABOVE this solver, so
                                    // the solver reads the version before that
                                    // write -- and the writer has to wait, or
                                    // which version it read is undefined.
                                    poseReverseEdges.emplace_back(
                                        writer, solver.GetPath());
                                    continue;
                                }
                                newSolverDependencies[solver.GetPath()].insert(
                                    writer);
                            }
                            // A solver override replaces the whole joint
                            // callback, so that joint never reads its own
                            // namespace-parent posed frame.
                            break;
                        }
                    }
                }
            }
        }
        // Kahn levels avoid recursion on long dependency chains and give
        // Evaluate a complete schedule instead of a fixed-point round cap.
        std::map<SdfPath, size_t> pending;
        std::map<SdfPath, std::vector<SdfPath>> consumers;
        std::vector<SdfPath> ready;
        for (const auto &[solver, dependencies] : newSolverDependencies) {
            pending[solver] = dependencies.size();
            if (dependencies.empty()) {
                ready.push_back(solver);
            }
            for (const SdfPath &dependency : dependencies) {
                consumers[dependency].push_back(solver);
            }
        }
        size_t scheduled = 0;
        while (!ready.empty()) {
            scheduled += ready.size();
            std::vector<SdfPath> next;
            for (const SdfPath &solver : ready) {
                for (const SdfPath &consumer : consumers[solver]) {
                    if (--pending[consumer] == 0) {
                        next.push_back(consumer);
                    }
                }
            }
            ready.swap(next);
        }
        if (scheduled != newSolverDependencies.size()) {
            std::string paths;
            for (const auto &[solver, count] : pending) {
                if (count) {
                    paths += " " + solver.GetString();
                }
            }
            reportError("solver dependency cycle among:" + paths);
            return false;
        }
    }

    compileBlocks.Close();
    stampCompileRegion("Compile.DiscoverValidate");

    compileBlocks.Next("SolverSchedule.ReplacementRequests");
    // Prepare replacement requests while retaining the previous requests and
    // their shared compiler context. The stock network handles changed USD
    // bindings incrementally; no whole execution-system replacement is needed.

    auto restorePreviousEpoch = [&]() {
        // Retain network checkpoints but require a valid binding plan before
        // another generation can be published.
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

    compileBlocks.Next("SolverSchedule.VolumeWeights");
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


    compileBlocks.Next("SolverSchedule.PoseConstraints");
    // Pose-domain constraints, compiled to in-memory structural wiring. Aim,
    // Position, Rotation, Scale, and Parent revise one transform provider;
    // SingleChainIK revises its inferred joint chain atomically. Values stay
    // authored on the mover and are sampled during Evaluate().
    std::vector<_FrameConstraint> newFrameConstraints;
    std::map<SdfPath, std::vector<SdfPath>> newFrameChains;
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

    // Every property-domain target a math mover revises. A mask attribute in
    // this set is not static: its live read must still see the chain's
    // result, so a constraint whose mask lands here keeps masksStatic false.
    std::set<SdfPath> propertyRevisedTargets;
    for (const RigExecMoverRecord &mover : newMovers) {
        if (mover.schemaType != "RigExecFloatMathMover" &&
            mover.schemaType != "RigExecVec3fMathMover" &&
            mover.schemaType != "RigExecMatrixMathMover") {
            continue;
        }
        for (const SdfPath &target : mover.targets) {
            propertyRevisedTargets.insert(target);
        }
    }
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
        // Precompute the axis masks when provably static for this mover: no
        // property chain revises a mask attribute, none is connected, and each
        // is a single authored opinion. Fallbacks mirror the live read:
        // translation/rotation default on, scale defaults off (FBX).
        {
            const char *kMaskNames[3][3] = {
                {"inputs:affectTranslationX", "inputs:affectTranslationY",
                 "inputs:affectTranslationZ"},
                {"inputs:affectRotationX", "inputs:affectRotationY",
                 "inputs:affectRotationZ"},
                {"inputs:affectScaleX", "inputs:affectScaleY",
                 "inputs:affectScaleZ"}};
            const bool kMaskFallback[3] = {true, true, false};
            auto groupStatic = [&](int group) {
                for (int axis = 0; axis < 3; ++axis) {
                    const char *name = kMaskNames[group][axis];
                    // A property path, not a child path: the names carry the
                    // inputs: namespace, which is not path syntax, and the
                    // revised targets are property paths. AppendPath here
                    // builds an invalid path that can never match, silently
                    // freezing every chain-driven mask at its epoch value.
                    if (propertyRevisedTargets.count(
                            constraint.moverPath.AppendProperty(
                                TfToken(name)))) {
                        return false;
                    }
                    const UsdAttribute a = moverPrim.GetAttribute(TfToken(name));
                    if (a) {
                        if (!_AuthoredConnections(a).empty()) {
                            return false;  // connected -> driven
                        }
                        std::vector<double> timeSamples;
                        // GetTimeSamples answers success, not a count: the
                        // comparison must be against the samples it filled.
                        if (a.GetTimeSamples(&timeSamples) &&
                            timeSamples.size() > 1) {
                            return false;  // animated
                        }
                    }
                    // absent -> schema default -> static
                }
                return true;
            };
            constraint.masksStatic =
                groupStatic(0) && groupStatic(1) && groupStatic(2);
            if (constraint.masksStatic) {
                auto readGroup = [&](int group) {
                    RigExecConstraintAxisMask m;
                    bool *out = &m.x;
                    for (int axis = 0; axis < 3; ++axis, ++out) {
                        bool v = kMaskFallback[group];
                        const UsdAttribute a =
                            moverPrim.GetAttribute(TfToken(kMaskNames[group][axis]));
                        if (a) {
                            a.Get(&v);
                        }
                        *out = v;
                    }
                    return m;
                };
                constraint.precompTranslation = readGroup(0);
                constraint.precompRotation = readGroup(1);
                constraint.precompScale = readGroup(2);
            }
        }

        for (const SdfPath &target : constraint.targets) {
            newFrameChains[target].push_back(mover.moverPath);
        }
        newFrameConstraints.push_back(std::move(constraint));
    }
    // ---- the UNIFIED POSE STACK ORDINAL (spec §4.2) ----------------------
    //
    // One order over two kinds of step that used to live in two phases:
    //
    //   * an aggregate solver that WRITES at least one joint, and
    //   * a pose-domain frame constraint (one that moves a transform provider
    //     rather than points).
    //
    // The order is the reverse of the composed pre-order of the WHOLE RIG --
    // the bottom composed sibling first, a parent after its descendants, the
    // rule _GetMoverExecutionOrder already gives movers -- and NOTHING else
    // breaks a tie. A constraint below a solver therefore runs BEFORE it and
    // feeds it (the incoming frame becomes that solver's rest reference); a
    // constraint above it revises its output, which is what every shipped rig
    // authors and why they are unchanged.
    //
    // A PRODUCER is deliberately ABSENT from this map (see
    // jointWritingSolvers above for what makes one), so every consumer of the
    // map must test membership rather than use operator[].
    std::map<SdfPath, int> poseStackOrdinal;
    {
        std::set<SdfPath> stackSteps = jointWritingSolvers;
        for (const _FrameConstraint &constraint : newFrameConstraints) {
            if (constraint.pointsTarget.IsEmpty()) {
                stackSteps.insert(constraint.moverPath);
            }
        }
        if (!stackSteps.empty()) {
            int ordinal = 0;
            for (const UsdPrim &prim :
                 _GetPoseStackOrder(_stage->GetPrimAtPath(_rigPath))) {
                if (stackSteps.count(prim.GetPath())) {
                    poseStackOrdinal[prim.GetPath()] = ordinal++;
                }
            }
        }
    }
    // Ordinal or "no position": a producer sorts before every stack step and
    // is deterministic about it, which is all the emission order needs.
    const auto stackOrdinalOf = [&poseStackOrdinal](const SdfPath &path) {
        const auto it = poseStackOrdinal.find(path);
        return it == poseStackOrdinal.end() ? -1 : it->second;
    };
    // An AGGREGATE read cannot be resolved positionally -- an aggregate is a
    // dataflow value, not a stacked per-joint frame, so there is no "earlier
    // version" of it to read and the consumer must run after the producer.
    // When the hierarchy says otherwise and BOTH are stack steps, that is a
    // contradiction the author has to resolve, and it is named rather than
    // left to surface as a generic Kahn loop.
    //
    // Unreachable while the consumed-solver relaxation stands (a solver whose
    // aggregate another solver reads writes no joint, so it is a producer and
    // has no position). This is the forward guard for the day that changes.
    for (const auto &[solver, dependencies] : newSolverAggregateReads) {
        const auto consumer = poseStackOrdinal.find(solver);
        if (consumer == poseStackOrdinal.end()) continue;
        for (const SdfPath &producer : dependencies) {
            const auto it = poseStackOrdinal.find(producer);
            if (it == poseStackOrdinal.end() || it->second < consumer->second) {
                continue;
            }
            reportError(
                solver.GetString() + " reads the aggregate of " +
                producer.GetString() + " but executes before it in the "
                "composed hierarchy (the bottom sibling executes first). "
                "Move " + producer.GetString() + " below it, or reorder "
                "nameChildren on its parent (spec §4.2)");
            restorePreviousEpoch();
            return false;
        }
    }
    // A startFrame read is positional (spec §4.2): a chain ordered before
    // its provider's writer reads the pre-write frame, and for a rest
    // provider that is the rest pose -- the chain then hangs from nothing
    // and stays frozen with no other diagnostic. Measured on the biped
    // hand: finger chains below the arm blend read the rest wrist (0.0000
    // on every joint). Authored and derived targets alike -- the read
    // does not know which -- so this runs over composed targets.
    for (const UsdPrim &solver : orderedSolvers) {
        if (solver.GetTypeName() != "RigExecFkChain") {
            continue;
        }
        static const TfToken startFrameRelTok("rigExec:startFrame");
        SdfPathVector starts;
        solver.GetRelationship(startFrameRelTok).GetTargets(&starts);
        if (starts.empty()) {
            continue;
        }
        const int here = stackOrdinalOf(solver.GetPath());
        if (here < 0) {
            continue;  // Guide-only: exec re-derives the order.
        }
        const SdfPath provider = starts[0].GetPrimPath();
        SdfPath culprit;
        const auto owned = newJointBinding.find(provider);
        if (owned != newJointBinding.end()) {
            for (const auto &entry : owned->second) {
                if (entry.first != solver.GetPath() &&
                    stackOrdinalOf(entry.first) > here) {
                    culprit = entry.first;
                    break;
                }
            }
        }
        if (culprit.IsEmpty()) {
            const auto revised = newFrameChains.find(provider);
            if (revised != newFrameChains.end()) {
                for (const SdfPath &writer : revised->second) {
                    if (writer != solver.GetPath() &&
                        stackOrdinalOf(writer) > here) {
                        culprit = writer;
                        break;
                    }
                }
            }
        }
        if (culprit.IsEmpty()) {
            continue;
        }
        const std::string message =
            solver.GetPath().GetString() + " reads startFrame " +
            provider.GetString() + " but executes before " +
            culprit.GetString() +
            ", which poses it, so it reads the pre-write frame. Reorder "
            "so the chain executes after the writer -- earlier in the "
            "file, since siblings execute bottom-first (spec §4.2).";
        if (errors) {
            errors->push_back("warning: " + message);
        }
        TF_WARN("%s", message.c_str());
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

    compileBlocks.Next("SolverSchedule.ControlFrames");
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

    compileBlocks.Next("SolverSchedule.PropertyChains");
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
                const SdfPathVector connections = _AuthoredConnections(a);
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

    compileBlocks.Next("SolverSchedule.PointGraph");
    // Bind the persistent point graph from the composed execution walk.
    // Geometry constraints contribute solved matrix packets to this same
    // chain. Compilation resolves bindings without authoring scene data.
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
        std::optional<RigExecRevisionOp> op =
            RigExecRevisionOpForSchema(mover.schemaType, curveMode);
        if (!op && _IsSourceFrameConstraintType(mover.schemaType)) {
            op = RigExecRevisionOp::Matrix;
        }
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
            if (!revision.binding.transformSpace.IsEmpty()) {
                revision.transformSpaceTap = newTaps->Add(
                    RigExecValueAddress::Prim(revision.binding.transformSpace,
                                              TfToken("computeMatrix")));
            }
            for (const SdfPath &influence : revision.binding.influences) {
                revision.influenceTaps.push_back(newTaps->Add(
                    RigExecValueAddress::Prim(influence,
                                              TfToken("computeMatrix"))));
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
            // No computeBlendChannel tap here, deliberately.
            //
            // There used to be one per blend input, pushed into
            // revision.blendChannelTaps -- and NOTHING ever read that vector.
            // The packet is assembled from a direct stage read in
            // _EvaluateDynamic instead, so every dense sample's full points
            // array was pulled twice per frame: once through exec to fill a
            // value that was discarded, once again for real.
            //
            // MEASURED (tools/biped/spikes/blend_cost.py, 64 dense samples on
            // a 26,276-point body, every channel weight 0): the taps cost
            // 7.72 ms/frame of AuthoritativeSnapshot, about 30% of the whole
            // per-target blend cost, for nothing. At N=0 AuthoritativeSnapshot
            // does not reach the profile's top eight at all, which is what
            // identified it.
            if (revision.op == RigExecRevisionOp::Skin) {
                // Whether the per-point layout can change WITHIN this epoch
                // is a question about the stage, so it is answered here once
                // rather than guessed at per frame. The three ways it can:
                // an authored time sample (the arrays differ per time code),
                // an authored connection (the value comes from somewhere
                // else, which may itself be animated), and a property chain
                // writing the attribute (the evaluator computes it per
                // generation). Anything else is epoch-constant by the same
                // definition the digest uses, so the layout is resolved once
                // and shared instead of re-read, re-copied and re-validated
                // on every frame.
                revision.skinTopologyFixed = true;
                for (const char *name : {"rigExec:jointIndices",
                                         "rigExec:jointWeights",
                                         "rigExec:elementSize"}) {
                    const SdfPath propertyPath =
                        mover.moverPath.AppendProperty(TfToken(name));
                    if (newPropertyChains.count(propertyPath)) {
                        revision.skinTopologyFixed = false;
                        break;
                    }
                    const UsdAttribute a =
                        moverPrim.GetAttribute(TfToken(name));
                    if (a && (a.ValueMightBeTimeVarying() ||
                              a.HasAuthoredConnections())) {
                        revision.skinTopologyFixed = false;
                        break;
                    }
                }
            }
            newGraphChains[target].push_back(revision);
        }
    }

    compileBlocks.Next("SolverSchedule.SampleReads");
    // Resolve sample-relative preceding reads against the global mover walk.
    // Samples can belong to a different chain, or multiple blend applications;
    // each application gets its own checkpoint, independent of sample identity.
    std::map<SdfPath, int> applicationOrdinals;
    for (const auto &mover : newMovers) applicationOrdinals[mover.moverPath] = mover.ordinal;
    for (auto &[target, revisions] : newGraphChains) {
        for (auto &revision : revisions) {
            for (auto &[input, samples] : revision.binding.blendSamples) {
                for (auto &sample : samples) {
                    const auto producers = newGraphChains.find(sample.points);
                    if (producers == newGraphChains.end()) {
                        sample.phase = RigExecReadPhase();
                    } else if (sample.phase.kind == RigExecReadPhaseKind::Preceding) {
                        SdfPath checkpoint;
                        for (const auto &producer : producers->second) {
                            if (applicationOrdinals[producer.moverPath] <
                                applicationOrdinals[revision.moverPath])
                                checkpoint = producer.moverPath;
                        }
                        sample.phase = checkpoint.IsEmpty() ? RigExecReadPhase() :
                            RigExecReadPhase{RigExecReadPhaseKind::AtPrim, checkpoint};
                    }
                }
            }
        }
    }

    compileBlocks.Next("SolverSchedule.DerivedMaintenance");
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

    compileBlocks.Next("SolverSchedule.PoseDag");
    // Compile one pose DAG. Constraints retain authored preceding order;
    // solvers read the final constrained state of each declared frame input.
    // Bound joints replace their namespace callback, cutting ancestor edges.
    auto isFrameProvider = [&](const SdfPath &path) {
        const UsdPrim prim = _stage->GetPrimAtPath(path);
        return prim && (prim.GetTypeName() == "RigExecJoint" ||
                        prim.GetTypeName() == "RigExecControl" ||
                        _IsVolumeWeightType(prim.GetTypeName()));
    };
    std::map<SdfPath, _PoseInputInfo> newPoseInputInfo;
    // Two stage closures that the schedule and request passes below ask for
    // over and over about the same prims: a prim's connection-input set, and
    // the pose-provider closure of one input path. The same joint is an input
    // to many solvers, many constraints and many batches, and each ask
    // re-walks the whole chain to the rig root.
    //
    // Both are pure functions of the composed stage and of the joint binding
    // this compile has already decided above; neither changes while a compile
    // runs, so a memoized answer is the answer a recomputation would give.
    // These are compile-local, deliberately separate from the digest's own
    // caches: the digest must stay a self-contained recomputation.
    std::unordered_map<SdfPath, std::set<SdfPath>, SdfPath::Hash>
        attributeInputCache;
    std::unordered_map<SdfPath, std::set<SdfPath>, SdfPath::Hash>
        poseClosureCache;
    // The per-prim half of the pose closure, read ahead of the walks that
    // need it and off this thread. _CollectPoseInputInfo is a pure read of
    // the composed stage -- a free function over one UsdPrim, local state
    // only -- so a prim's answer does not depend on when or where it is
    // computed, which is what lets the walk below take one out of here
    // instead of paying for it in line.
    //
    // Deliberately NOT newPoseInputInfo itself. That map is iterated as a
    // RESULT further down (the connected-pose taps, newPoseProviderInputs),
    // so it has to hold exactly the prims a closure walk actually reached
    // and nothing more. This one is free to overshoot -- a prefetched prim
    // no walk asks about costs a worker's time and nothing else -- which is
    // what makes seeding it generously safe.
    std::unordered_map<SdfPath, _PoseInputInfo, SdfPath::Hash>
        poseInfoPrefetch;
    auto collectAttributeInputs =
        [&](const SdfPath &path) -> const std::set<SdfPath> & {
        auto it = attributeInputCache.find(path);
        if (it == attributeInputCache.end()) {
            it = attributeInputCache.emplace(
                path,
                _CollectAttributeConnectionInputs(
                    _stage->GetPrimAtPath(path))).first;
        }
        return it->second;
    };
    auto computePoseProviderClosure = [&](const SdfPath &input) {
        std::set<SdfPath> closure;
        std::vector<SdfPath> pending{input};
        while (!pending.empty()) {
            const SdfPath path = pending.back();
            pending.pop_back();
            if (path.IsEmpty() || !closure.insert(path).second) continue;
            if (newJointBinding.count(path)) {
                newPoseInputInfo[path];
                continue;
            }
            auto it = newPoseInputInfo.find(path);
            if (it == newPoseInputInfo.end()) {
                // Same value either way: the prefetch ran the same function
                // on the same prim of the same stage. Only newPoseInputInfo
                // records that this path was REACHED.
                // MOVED out, not copied: a path lands in newPoseInputInfo
                // at most once (this branch is the find() miss), so nothing
                // reads the prefetch entry again, and leaving the sets
                // behind would hold every closure's inputs on the heap
                // TWICE for the rest of compile -- which measured as ~13 ms
                // added to the batch pass that runs after it, purely in
                // allocator and locality cost.
                const auto warm = poseInfoPrefetch.find(path);
                it = newPoseInputInfo.emplace(path,
                    warm != poseInfoPrefetch.end()
                        ? std::move(warm->second)
                        : _CollectPoseInputInfo(
                              _stage->GetPrimAtPath(path))).first;
            }
            pending.insert(pending.end(), it->second.providers.begin(),
                           it->second.providers.end());
            if (!isFrameProvider(path)) {
                const UsdPrim parent = _NamespaceFrameProvider(_stage->GetPrimAtPath(path));
                if (parent) pending.push_back(parent.GetPath());
            }
        }
        return closure;
    };
    auto poseProviderClosure =
        [&](const SdfPath &input) -> const std::set<SdfPath> & {
        auto it = poseClosureCache.find(input);
        if (it == poseClosureCache.end()) {
            it = poseClosureCache.emplace(
                input, computePoseProviderClosure(input)).first;
        }
        return it->second;
    };
    std::map<SdfPath, std::set<SdfPath>> solverFrameInputs;
    for (const auto &[solver, dependencies] : newSolverDependencies) {
        const UsdPrim prim = _stage->GetPrimAtPath(solver);
        for (const UsdRelationship &rel : prim.GetRelationships()) {
            if (rel.GetName() == "rigExec:joints") {
                continue;
            }
            SdfPathVector targets;
            rel.GetTargets(&targets);
            for (const SdfPath &target : targets) {
                if (isFrameProvider(target.GetPrimPath())) {
                    solverFrameInputs[solver].insert(target.GetPrimPath());
                }
            }
        }
    }
    // MEASURED (biped_stack_anim): the pose closure was the largest single
    // cost left on this thread -- 13.0 ms inside the constraint pass and
    // 21.9 ms again in PrepareRequests.ProviderClosure, both of them one
    // prim's _CollectPoseInputInfo at a time. The walk that spends it is
    // inherently serial (each prim's answer says which prim to ask about
    // next), but the ASKING is not: a level of the frontier is a set of
    // independent stage reads.
    //
    // So the closure is walked breadth-first here, one level at a time, with
    // each level's reads spread across the pool and merged in on this thread
    // after. The walks below then find their prims already read and do set
    // arithmetic only. Concurrent reads of a UsdStage are what the digest
    // thread, the tap warm-up and the two bake bind loops already do.
    //
    // Seeded with every path a closure will be asked for -- the solvers'
    // frame inputs, every constraint's inputs, and the joints and controls
    // that PrepareRequests seeds providers from -- plus each one's
    // frame-provider ancestors, because those are what newFirstFramePoseFrames
    // holds by the time ProviderClosure asks. Overshooting is free; missing a
    // prim only means the walk reads it in line, exactly as it used to.
    {
        RIGEXEC_PROFILE_SCOPE_CAT(_profiler, "PoseInfoPrefetch", "compile");
        std::vector<SdfPath> frontier;
        const auto seed = [&](const SdfPath &path) {
            if (path.IsEmpty()) return;
            frontier.push_back(path);
            // Only the FRAME-PROVIDER ancestors, because those are the ones
            // seedProvider puts in newFirstFramePoseFrames and ProviderClosure
            // then asks about. Pushing the whole namespace chain instead
            // reads a large part of the stage that no walk ever asks for,
            // and the allocation that costs lands on every pass after it.
            for (SdfPath a = path.GetParentPath();
                 !a.IsEmpty() && a != SdfPath::AbsoluteRootPath();
                 a = a.GetParentPath()) {
                if (isFrameProvider(a)) frontier.push_back(a);
            }
        };
        for (const auto &[solver, inputs] : solverFrameInputs) {
            for (const SdfPath &input : inputs) seed(input);
        }
        for (const _FrameConstraint &constraint : newFrameConstraints) {
            for (const SdfPath &target : constraint.targets) seed(target);
            for (const auto &source : constraint.sources) {
                seed(source.sourcePath);
            }
            seed(constraint.worldUpObject.sourcePath);
            seed(constraint.effector.sourcePath);
            seed(constraint.weightObject);
            for (const auto &pole : constraint.poleObjects) {
                seed(pole.sourcePath);
            }
        }
        for (const SdfPath &path : newJointPaths) seed(path);
        for (const SdfPath &path : newControlPaths) seed(path);

        std::set<SdfPath> queued;
        while (!frontier.empty()) {
            // A joint-bound path stops the real walk without being read, so
            // it is not worth reading here either.
            std::vector<SdfPath> level;
            for (const SdfPath &path : frontier) {
                if (path.IsEmpty() || newJointBinding.count(path)) continue;
                if (!queued.insert(path).second) continue;
                level.push_back(path);
            }
            frontier.clear();
            if (level.empty()) break;
            std::vector<_PoseInputInfo> infos(level.size());
            std::vector<SdfPath> parents(level.size());
            const auto readLevel = [&](size_t begin, size_t end) {
                for (size_t i = begin; i < end; ++i) {
                    const UsdPrim prim = _stage->GetPrimAtPath(level[i]);
                    infos[i] = _CollectPoseInputInfo(prim);
                    if (!isFrameProvider(level[i])) {
                        const UsdPrim parent = _NamespaceFrameProvider(prim);
                        if (parent) parents[i] = parent.GetPath();
                    }
                }
            };
            if (RigExecParallelEvaluationEnabled() && level.size() > 1) {
                WorkParallelForN(level.size(), readLevel);
            } else {
                readLevel(0, level.size());
            }
            for (size_t i = 0; i < level.size(); ++i) {
                frontier.insert(frontier.end(), infos[i].providers.begin(),
                                infos[i].providers.end());
                if (!parents[i].IsEmpty()) frontier.push_back(parents[i]);
                poseInfoPrefetch.emplace(level[i], std::move(infos[i]));
            }
        }
    }
    std::map<SdfPath, std::set<SdfPath>> solverPoseReads;
    for (const auto &[solver, inputs] : solverFrameInputs) {
        for (const SdfPath &input : inputs) {
            const std::set<SdfPath> &closure = poseProviderClosure(input);
            solverPoseReads[solver].insert(closure.begin(), closure.end());
            for (const SdfPath &provider : closure) {
                const auto owner = newJointBinding.find(provider);
                if (owner != newJointBinding.end()) {
                    // Every writer BELOW this solver in the stack, never this
                    // solver itself -- the same rule the direct relationship
                    // walk applies above, and the edge class that catches an
                    // indirect read (an IK control parented under a joint, a
                    // startFrameObject reached through a namespace parent).
                    // A writer ABOVE it gets the mirror edge instead: the
                    // solver read the version standing before that write, so
                    // the write has to wait.
                    for (const auto &[writer, writerElement] :
                         owner->second) {
                        if (writer == solver) {
                            continue;
                        }
                        if (jointWritingSolvers.count(solver) &&
                            stackBefore(solver, writer)) {
                            poseReverseEdges.emplace_back(writer, solver);
                            continue;
                        }
                        newSolverDependencies[solver].insert(writer);
                    }
                }
            }
        }
    }
    // The solver DAG is complete here, but the POSE graph is not: the
    // constraint pass and the frame-inheritance walk below still add
    // solver -> constraint -> solver edges, and a pair those order is a pair
    // the stack must not order differently. So the stack order is settled
    // ONCE, against the finished graph, just before the schedule is built --
    // see "the stack order" block below SolverSchedule.FrameInheritance.
    std::map<SdfPath, std::vector<std::pair<SdfPath, int>>> newSolverJoints;
    std::set<SdfPath> requiredSolvers;
    for (const auto &[joint, writers] : newJointBinding) {
        for (const auto &[solver, element] : writers) {
            newSolverJoints[solver].emplace_back(joint, element);
            requiredSolvers.insert(solver);
        }
    }
    // What used to be one SolverSchedule.AggregateConsumers block, and the
    // largest single region left on the main thread. Split five ways because
    // a guess about which of its passes held the time was already wrong once:
    // the cross product the walk below replaces was predicted to be 20 of
    // those milliseconds and measured, after inversion, to be worth none of
    // them. The passes are quite different work -- set closure, constraint
    // reads through the pose closure, an ancestor walk, a counting pass, and
    // the batch BFS that talks to exec -- so they are timed apart before
    // anything else here is touched.
    compileBlocks.Next("SolverSchedule.RequiredSolvers");
    // Geometry aggregate consumers are authoritative even without joints.
    for (const auto &[target, revisions] : newGraphChains) {
        for (const _GraphRevision &revision : revisions) {
            if (newSolverDependencies.count(revision.binding.driverFrames)) {
                requiredSolvers.insert(revision.binding.driverFrames);
            }
        }
    }
    std::vector<SdfPath> pendingSolvers(requiredSolvers.begin(), requiredSolvers.end());
    while (!pendingSolvers.empty()) {
        const SdfPath solver = pendingSolvers.back();
        pendingSolvers.pop_back();
        for (const SdfPath &dependency : newSolverDependencies[solver]) {
            if (requiredSolvers.insert(dependency).second) {
                pendingSolvers.push_back(dependency);
            }
        }
    }
    std::map<SdfPath, std::set<SdfPath>> poseDependencies;
    std::map<SdfPath, size_t> constraintIndices;
    for (const SdfPath &solver : requiredSolvers) {
        poseDependencies[solver] = newSolverDependencies[solver];
    }
    compileBlocks.Next("SolverSchedule.ConstraintDeps");
    // Collected in the constraint loop, consumed by the upward walk just
    // below it. See the comment there for why the cross product this
    // replaces had to be turned inside out.
    std::map<SdfPath, std::vector<SdfPath>> constraintsByTarget;
    for (size_t i = 0; i < newFrameConstraints.size(); ++i) {
        const _FrameConstraint &constraint = newFrameConstraints[i];
        const SdfPath &path = constraint.moverPath;
        constraintIndices[path] = i;
        auto &dependencies = poseDependencies[path];
        if (i > 0) {
            dependencies.insert(newFrameConstraints[i - 1].moverPath);
        }
        // Every writer of the frame, not just the last one -- and now
        // DIRECTIONALLY (spec §4.2). A frame read is POSITIONAL: the
        // constraint reads the version standing at its own place in the
        // unified pose stack. A writer BELOW it must therefore run first, and
        // a writer ABOVE it must wait, or which version was read is undefined
        // and the schedule decides it by accident. A missed edge here does not
        // fail loudly; it silently reads the wrong version.
        //
        // A geometry-domain constraint carries no stack position, so it keeps
        // the unconditional edge it always had.
        const bool positional = poseStackOrdinal.count(path) > 0;
        const int here = stackOrdinalOf(path);
        auto dependOnFrame = [&](const SdfPath &input) {
            for (const SdfPath &provider : poseProviderClosure(input)) {
                const auto owner = newJointBinding.find(provider);
                if (owner != newJointBinding.end()) {
                    for (const auto &[writer, element] : owner->second) {
                        if (positional && stackOrdinalOf(writer) > here) {
                            poseReverseEdges.emplace_back(writer, path);
                            continue;
                        }
                        dependencies.insert(writer);
                    }
                }
            }
        };
        for (const SdfPath &target : constraint.targets) dependOnFrame(target);
        for (const auto &source : constraint.sources) dependOnFrame(source.sourcePath);
        dependOnFrame(constraint.worldUpObject.sourcePath);
        dependOnFrame(constraint.effector.sourcePath);
        dependOnFrame(constraint.weightObject);
        for (const auto &pole : constraint.poleObjects) dependOnFrame(pole.sourcePath);
        // A geometry-domain constraint revises points, not the frame a solver
        // reads, so no solver can inherit one: this `continue` is the same
        // exclusion the cross product below used to get from skipping the
        // block it guarded, and it has to stay exactly that strict.
        if (!constraint.pointsTarget.IsEmpty()) {
            continue;
        }
        for (const SdfPath &target : constraint.targets) {
            constraintsByTarget[target].push_back(path);
        }
    }
    compileBlocks.Next("SolverSchedule.FrameInheritance");
    // Which solvers must wait on which frame constraints. Asked the other way
    // round: for every pose a solver reads, walk UP to the rig root and pick
    // up the constraints that target each ancestor on the way.
    //
    // This used to be the cross product -- constraints x requiredSolvers x
    // that solver's pose reads x that constraint's targets, with a call to an
    // `inheritsFrame` predicate per tuple, each call re-walking an ancestor
    // chain. That shape was PREDICTED to be 20 of the 23.4 ms this region
    // cost, and the prediction was wrong: inverted, the region measured
    // 25.7 -> 26.4 ms across four runs a side, which is inside the noise.
    // The time is somewhere else in the five blocks this one is now split
    // into. The inversion is kept because it is the better code -- linear in
    // the ancestor chain rather than quartic in the cross product, so it
    // cannot become the problem later -- not because it bought anything.
    // Inverted, each (solver, input) chain
    // is walked ONCE for all constraints instead of once per constraint,
    // because the map lookup answers "which constraints target this ancestor"
    // in one step. The predicate is gone with its only caller.
    //
    // The walk reproduces that predicate exactly, half-open range included.
    // inheritsFrame(input, target) was `input.HasPrefix(target)` AND no
    // newJointBinding entry on any path in [input, target) -- its loop ran
    // `for (path = input; path != target; ...)`, so the TARGET ITSELF was
    // never tested for a binding, and input == target was vacuously true with
    // no test at all. Hence the binding check below comes AFTER recording p's
    // own constraints and not before: a joint binding sitting ON a constraint
    // target does not disqualify that target, only bindings strictly below
    // it do. Reversing those two statements silently drops dependencies and
    // the schedule runs a solver before the constraint it reads.
    //
    // Targets are prim paths (see _FrameConstraint::targets), so "ancestor
    // chain" and HasPrefix agree; a target that is somehow not on the chain
    // is simply never found, which is what the predicate answered for it too.
    //
    // poseDependencies[solver] is a std::set, so the different insertion
    // ORDER this produces cannot change its contents, and the contents are
    // all pendingPose/poseConsumers below are built from -- same sets, same
    // schedule. The `empty()` guard keeps even the default-insertion
    // behaviour of solverPoseReads[solver] identical to the old code, which
    // only reached that operator[] when at least one constraint survived the
    // pointsTarget test.
    if (!constraintsByTarget.empty()) {
        for (const SdfPath &solver : requiredSolvers) {
            for (const SdfPath &input : solverPoseReads[solver]) {
                for (SdfPath p = input;
                     !p.IsEmpty() && p != SdfPath::AbsoluteRootPath();
                     p = p.GetParentPath()) {
                    const auto it = constraintsByTarget.find(p);
                    if (it != constraintsByTarget.end()) {
                        // Directional for the same reason dependOnFrame is:
                        // a constraint ABOVE this solver revises what the
                        // solver already read, so the solver must NOT wait
                        // on it -- the constraint waits on the solver
                        // instead. (Producers have no position and keep the
                        // unconditional edge.)
                        const bool positional =
                            poseStackOrdinal.count(solver) > 0;
                        const int here = stackOrdinalOf(solver);
                        for (const SdfPath &constraint : it->second) {
                            if (positional &&
                                stackOrdinalOf(constraint) > here) {
                                poseReverseEdges.emplace_back(constraint,
                                                              solver);
                                continue;
                            }
                            poseDependencies[solver].insert(constraint);
                        }
                    }
                    if (newJointBinding.count(p)) {
                        break;
                    }
                }
            }
        }
    }
    // ---- the UNIFIED POSE STACK: one order over solvers and constraints ---
    //
    // poseDependencies is complete here and nowhere earlier: the solver DAG
    // was finished above, but the constraint pass and the frame-inheritance
    // walk just added the solver <-> constraint edges.
    //
    // The rule (spec §4.2) is now the NAMESPACE and nothing else. Every step
    // of the pose phase -- a solver that writes a joint, a constraint that
    // moves one -- carries a poseStackOrdinal taken from the reverse composed
    // pre-order of the whole rig, and that ordinal IS the order. Data flow no
    // longer bends it: a frame read below its writer reads the earlier
    // version (the directional edges above), and an aggregate read that
    // contradicts it was rejected by name at compile time.
    //
    // So this block does three things and no searching:
    //
    //   1. put every joint's writer list into ordinal order;
    //   2. build that joint's INTERLEAVED writer chain -- its solvers and the
    //      constraints that move it, in one ordinal order;
    //   3. insert one edge per adjacent pair of that chain, so the schedule
    //      runs them in it.
    //
    // A joint's chain is a restriction of one total order, so no two joints
    // can order the same pair in opposite directions and no edge inserted
    // here can close a loop.
    //
    // WIDTH. The hierarchical order is never turned into a global serial
    // chain: an edge is only ever inserted between two steps that write the
    // SAME joint, and the only other edges in poseDependencies are the real
    // data-flow ones (a reader and a writer of what it reads) and the mover
    // chain that has always serialized the constraints. Two limbs that share
    // no joint and no data flow therefore share a Kahn level and evaluate
    // concurrently, and the baked cone schedule -- built from this same graph
    // -- keeps its width. testRigExecSolverStacking's
    // TestUnrelatedLimbsShareALevel pins it, and the biped measures WIDER
    // than before this change (24 solvers over 6 dependency levels rather
    // than 17, because the frame-inheritance edges that used to make a solver
    // wait on a constraint above it are now directional).
    std::map<SdfPath, std::vector<SdfPath>> jointWriterChain;
    /// solver -> (joint -> the step that wrote the joint just before it), the
    /// compile-side half of "the incoming frame is the solver's rest".
    std::map<SdfPath, std::map<SdfPath, SdfPath>> solverRestPredecessor;
    {
        for (auto &[joint, writers] : newJointBinding) {
            if (writers.size() < 2) continue;
            std::stable_sort(
                writers.begin(), writers.end(),
                [&stackOrdinalOf](const std::pair<SdfPath, int> &a,
                                  const std::pair<SdfPath, int> &b) {
                    return stackOrdinalOf(a.first) < stackOrdinalOf(b.first);
                });
        }
        // Which pose-domain constraints move each provider. Geometry-domain
        // constraints are excluded: they revise points, carry no stack
        // position, and are not writers of a frame.
        std::map<SdfPath, std::vector<SdfPath>> frameWritingConstraints;
        for (const _FrameConstraint &constraint : newFrameConstraints) {
            if (!constraint.pointsTarget.IsEmpty()) continue;
            if (!poseStackOrdinal.count(constraint.moverPath)) continue;
            for (const SdfPath &target : constraint.targets) {
                frameWritingConstraints[target].push_back(
                    constraint.moverPath);
            }
        }
        std::set<SdfPath> written;
        for (const auto &[joint, writers] : newJointBinding) {
            written.insert(joint);
        }
        for (const auto &[target, constraints] : frameWritingConstraints) {
            written.insert(target);
        }
        for (const SdfPath &joint : written) {
            std::vector<std::pair<int, SdfPath>> chain;
            const auto solvers = newJointBinding.find(joint);
            if (solvers != newJointBinding.end()) {
                for (const auto &[writer, element] : solvers->second) {
                    chain.emplace_back(stackOrdinalOf(writer), writer);
                }
            }
            const auto constraints = frameWritingConstraints.find(joint);
            if (constraints != frameWritingConstraints.end()) {
                for (const SdfPath &constraint : constraints->second) {
                    chain.emplace_back(stackOrdinalOf(constraint), constraint);
                }
            }
            std::sort(chain.begin(), chain.end());
            std::vector<SdfPath> &ordered = jointWriterChain[joint];
            for (const auto &[ordinal, step] : chain) {
                ordered.push_back(step);
            }
            for (size_t i = 1; i < ordered.size(); ++i) {
                if (ordered[i] == ordered[i - 1]) continue;
                poseDependencies[ordered[i]].insert(ordered[i - 1]);
            }
            // "The incoming frame replaces the authored rest." For each
            // solver in the chain, the step immediately before it that wrote
            // this joint -- which is the frame that solver measures from.
            for (size_t i = 1; i < ordered.size(); ++i) {
                if (!jointWritingSolvers.count(ordered[i])) continue;
                const auto owner = newJointBinding.find(joint);
                bool writesIt = false;
                if (owner != newJointBinding.end()) {
                    for (const auto &[writer, element] : owner->second) {
                        writesIt = writesIt || writer == ordered[i];
                    }
                }
                if (!writesIt) continue;
                solverRestPredecessor[ordered[i]][joint] = ordered[i - 1];
            }
        }
    }
    // The single-chain IK constraint is the one CONSTRAINT that measures
    // from joint rests, so the same substitution reaches it: a joint a step
    // below it wrote hands it that step's frame as the rest reference.
    for (_FrameConstraint &constraint : newFrameConstraints) {
        if (constraint.ikChain.empty()) continue;
        constraint.ikRestLive.assign(constraint.ikChain.size(), 0);
        for (size_t i = 0; i < constraint.ikChain.size(); ++i) {
            const auto it = jointWriterChain.find(constraint.ikChain[i]);
            if (it == jointWriterChain.end()) continue;
            for (size_t k = 0; k < it->second.size(); ++k) {
                if (it->second[k] == constraint.moverPath) {
                    constraint.ikRestLive[i] = k > 0 ? 1 : 0;
                    break;
                }
            }
        }
    }
    // The reverse edges -- "a writer waits on a reader standing below it" --
    // collected wherever a read was resolved. Applied here, once, because
    // poseDependencies is only complete now and because an edge inserted into
    // it earlier would have been read back as a dependency by the passes
    // above.
    for (const auto &[waiter, waitedOn] : poseReverseEdges) {
        if (waiter == waitedOn) continue;
        if (!poseDependencies.count(waiter)) continue;
        if (!poseDependencies.count(waitedOn)) continue;
        poseDependencies[waiter].insert(waitedOn);
    }

    compileBlocks.Next("SolverSchedule.PoseReady");
    std::vector<_PoseStep> newPoseSteps;
    std::vector<_SolverBatch> newSolverBatches;
    std::map<SdfPath, std::set<size_t>> newSolverInputBatches;
    std::map<SdfPath, size_t> pendingPose;
    std::map<SdfPath, std::vector<SdfPath>> poseConsumers;
    std::vector<SdfPath> readyPose;
    for (const auto &[path, dependencies] : poseDependencies) {
        pendingPose[path] = dependencies.size();
        if (dependencies.empty()) readyPose.push_back(path);
        for (const SdfPath &dependency : dependencies) {
            poseConsumers[dependency].push_back(path);
        }
    }
    compileBlocks.Next("SolverSchedule.SolverBatches");
    {
        // Join the warm-up: everything below this point talks to exec.
        RIGEXEC_PROFILE_SCOPE_CAT(_profiler, "Compile.WarmupJoin", "compile");
        warmupDispatcher.Wait();
    }
    size_t scheduledPose = 0;
    size_t poseLevel = 0;
    while (!readyPose.empty()) {
        // THE INTERLEAVE (spec §4.2). A solver batch and a constraint are one
        // kind of step in one stack, so a ready level is emitted in POSE STACK
        // ORDINAL order rather than "every solver first, then every
        // constraint". That is the whole structural difference between the two
        // phases this change collapses, and it is this one sort.
        //
        // A producer carries no ordinal and sorts first: it publishes an
        // aggregate and writes no joint, so nothing can observe where in the
        // level it landed.
        std::sort(readyPose.begin(), readyPose.end(),
                  [&stackOrdinalOf](const SdfPath &a, const SdfPath &b) {
                      const int oa = stackOrdinalOf(a);
                      const int ob = stackOrdinalOf(b);
                      return oa != ob ? oa < ob : a < b;
                  });
        for (const SdfPath &path : readyPose) {
            const auto constraintStep = constraintIndices.find(path);
            if (constraintStep != constraintIndices.end()) {
                newPoseSteps.push_back({false, constraintStep->second});
                continue;
            }
            if (!requiredSolvers.count(path)) continue;
            _SolverBatch batch;
            batch.level = poseLevel;
            batch.taps = std::make_unique<RigExecTapSet>(_stage);
            batch.solvers[path] = batch.taps->Add(
                RigExecValueAddress::Prim(path, _computePointFrameArray));
            const auto &dependencies = newSolverDependencies[path];
            batch.dependencies.insert(dependencies.begin(), dependencies.end());
            const auto &frames = solverFrameInputs[path];
            batch.frameInputs.insert(frames.begin(), frames.end());
            // "The incoming frame replaces the authored rest" (spec §4.2).
            // Where a step before this solver wrote one of the joints the
            // solver names, the solver measures from THAT frame instead of
            // the joint's authored rest; a joint with no earlier writer still
            // hands it the authored rest, which is why a rig whose
            // constraints all sit above its solvers is bit-identical to what
            // it was before this rule existed.
            //
            // EVERY named joint is listed, not only the live ones, and that
            // is not belt and braces: computeRestFrame reads its NAMESPACE
            // ANCESTOR's computeRestFrame, so an override on the hip would
            // otherwise shift the knee's and the ankle's rests too -- which
            // the baked path, whose rests are per-slot, does not do. Pinning
            // the authored value on the joints with no predecessor is what
            // makes the two paths compute the same description.
            //
            // The map stays EMPTY unless at least one joint is live, so a rig
            // with no constraint below a solver pushes no override at all and
            // exec resolves computeRestFrame from the stage exactly as it
            // always has. That is the parity guarantee, and it is structural
            // rather than argued.
            {
                const auto predecessors = solverRestPredecessor.find(path);
                if (predecessors != solverRestPredecessor.end() &&
                    !predecessors->second.empty()) {
                    // Every joint the solver NAMES, not only the ones it
                    // writes: the relaxation can leave a named joint unwritten
                    // (it is then a pure rest reference), and such a joint
                    // still needs its authored rest pinned or it would inherit
                    // an overridden ancestor's.
                    const UsdPrim solverPrim = _stage->GetPrimAtPath(path);
                    SdfPathVector named;
                    if (solverPrim) {
                        if (const UsdRelationship rel =
                                solverPrim.GetRelationship(
                                    TfToken("rigExec:joints"))) {
                            rel.GetTargets(&named);
                        }
                    }
                    for (const SdfPath &joint : named) {
                        const auto prev =
                            predecessors->second.find(joint.GetPrimPath());
                        batch.restInputs[joint.GetPrimPath()] =
                            prev == predecessors->second.end()
                                ? SdfPath()
                                : prev->second;
                    }
                }
            }
            const bool batchPrepared = deferExecPrep ? true : [&]() {
                RIGEXEC_PROFILE_SCOPE_CAT(
                    _profiler, "TapPrepare solverBatch", "compile");
                return batch.taps->Prepare();
            }();
            if (!batchPrepared) {
                reportError("failed to prepare solver dependency level");
                restorePreviousEpoch();
                return false;
            }
            const size_t batchIndex = newSolverBatches.size();
            auto registerInput = [&](const SdfPath &inputPath, bool followParents) {
                for (SdfPath path = inputPath.GetPrimPath();
                     !path.IsEmpty() && path != SdfPath::AbsoluteRootPath();
                     path = followParents ? path.GetParentPath() : SdfPath()) {
                    newSolverInputBatches[path].insert(batchIndex);
                    for (const SdfPath &attribute :
                         collectAttributeInputs(path)) {
                        newSolverInputBatches[attribute.GetPrimPath()].insert(batchIndex);
                    }
                    if (newJointBinding.count(path)) break;
                }
            };
            for (const auto &[solver, tap] : batch.solvers) {
                registerInput(solver, false);
                for (const SdfPath &provider : solverPoseReads[solver]) {
                    registerInput(provider, false);
                    const auto info = newPoseInputInfo.find(provider);
                    if (info != newPoseInputInfo.end()) {
                        for (const SdfPath &attribute : info->second.attributes) {
                            newSolverInputBatches[attribute.GetPrimPath()].insert(batchIndex);
                        }
                    }
                }
                const UsdPrim prim = _stage->GetPrimAtPath(solver);
                for (const UsdRelationship &rel : prim.GetRelationships()) {
                    SdfPathVector targets;
                    rel.GetTargets(&targets);
                    for (const SdfPath &target : targets) {
                        if (newSolverDependencies.count(target.GetPrimPath())) continue;
                        registerInput(target, isFrameProvider(target.GetPrimPath()));
                    }
                }
            }
            newPoseSteps.push_back({true, batchIndex});
            newSolverBatches.push_back(std::move(batch));
        }
        scheduledPose += readyPose.size();
        std::vector<SdfPath> next;
        for (const SdfPath &path : readyPose) {
            for (const SdfPath &consumer : poseConsumers[path]) {
                if (--pendingPose[consumer] == 0) next.push_back(consumer);
            }
        }
        readyPose.swap(next);
        ++poseLevel;
    }
    if (scheduledPose != poseDependencies.size()) {
        // Report the LOOP, not everything waiting on it. What Kahn leaves
        // behind is every step downstream of the cycle -- on the biped that
        // was sixty constraints and four solvers for a loop of four -- and a
        // list like that reads as "everything depends on everything", which
        // sent two investigations after the wrong edge. A depth-first walk
        // over the unscheduled steps, following only edges into other
        // unscheduled steps (a scheduled dependency cannot be on the loop),
        // finds one elementary cycle; the rest are counted as a hint of the
        // blast radius.
        const auto unscheduled = [&pendingPose](const SdfPath &path) {
            const auto it = pendingPose.find(path);
            return it != pendingPose.end() && it->second != 0;
        };
        struct _Frame {
            SdfPath node;
            std::vector<SdfPath> dependencies;
            size_t next = 0;
        };
        std::vector<SdfPath> loop;
        std::map<SdfPath, int> color;  // 0 unvisited, 1 on the stack, 2 done
        for (const auto &[start, pending] : pendingPose) {
            if (!pending || color[start] != 0 || !loop.empty()) continue;
            std::vector<_Frame> stack;
            const auto push = [&](const SdfPath &node) {
                color[node] = 1;
                const auto &deps = poseDependencies[node];
                stack.push_back({node, {deps.begin(), deps.end()}, 0});
            };
            push(start);
            while (!stack.empty() && loop.empty()) {
                if (stack.back().next >= stack.back().dependencies.size()) {
                    color[stack.back().node] = 2;
                    stack.pop_back();
                    continue;
                }
                const SdfPath dependency =
                    stack.back().dependencies[stack.back().next++];
                if (!unscheduled(dependency)) continue;
                const int c = color[dependency];
                if (c == 1) {
                    for (const _Frame &frame : stack) {
                        if (!loop.empty() || frame.node == dependency) {
                            loop.push_back(frame.node);
                        }
                    }
                } else if (c == 0) {
                    push(dependency);
                }
            }
        }
        std::string message = "pose dependency cycle among:";
        if (loop.empty()) {
            // Cannot happen -- an unschedulable DAG has a cycle by
            // definition -- but a report is still owed if it somehow does.
            for (const auto &[path, pending] : pendingPose) {
                if (pending) message += " " + path.GetString();
            }
        } else {
            // In dependency order: each step waits on the next, the last on
            // the first. A constraint's wait on its predecessor in the
            // Movers stack shows up here as an ordinary edge, which is how
            // "this follower is authored below what it needs" reads.
            size_t waiting = 0;
            for (const auto &[path, pending] : pendingPose) {
                if (pending) ++waiting;
            }
            waiting -= loop.size();
            for (const SdfPath &step : loop) message += " " + step.GetString() + " ->";
            message += " " + loop.front().GetString() +
                       " (each step waits on the next";
            if (waiting) {
                message += "; " + std::to_string(waiting) +
                           (waiting == 1 ? " further pose step waits"
                                         : " further pose steps wait") +
                           " on the loop";
            }
            message += ")";
        }
        reportError(message);
        restorePreviousEpoch();
        return false;
    }

    // ---- the pose stack: the walk order and the per-joint chains ----------
    //
    // Everything above settles which steps exist and what must precede what;
    // the SCHEDULE is what finally puts them in a line. Read that line back
    // here, so that "the last writer supplies the joint's base frame" and
    // "AtPrim resolves against the order that actually runs" are facts rather
    // than hopes.
    //
    // The chain is INTERLEAVED -- solvers and the constraints that move the
    // same joint, in one hierarchical order -- and it is built for every
    // written joint, because the constraint half of a chain is not
    // always-after.
    //
    // Nothing here is REPORTED. Stacking is ordinary authoring under the
    // unified pose stack, not a shape worth a diagnostic, so the compile is
    // silent about it and GetFrameChains() is what a tool or a test reads to
    // see the order. Only real errors reach the caller's message vector.
    if (!newJointBinding.empty()) {
        std::map<SdfPath, std::vector<SdfPath>> walkChains;
        std::map<SdfPath, std::vector<std::pair<SdfPath, int>>> walkWriters;
        for (const _PoseStep &step : newPoseSteps) {
            if (step.solverBatch) {
                for (const auto &[solver, tap] :
                     newSolverBatches[step.index].solvers) {
                    const auto joints = newSolverJoints.find(solver);
                    if (joints == newSolverJoints.end()) {
                        continue;
                    }
                    for (const auto &[joint, element] : joints->second) {
                        walkChains[joint].push_back(solver);
                        walkWriters[joint].emplace_back(solver, element);
                    }
                }
                continue;
            }
            const _FrameConstraint &constraint =
                newFrameConstraints[step.index];
            if (!constraint.pointsTarget.IsEmpty()) {
                continue;  // geometry domain: not a frame writer
            }
            for (const SdfPath &target : constraint.targets) {
                // Only joints a solver also writes get a rebuilt chain; a
                // provider only constraints revise already carries exactly
                // this chain from the constraint pass, so leaving it alone
                // keeps every rig without a solver bit-identical.
                if (!newJointBinding.count(target)) continue;
                walkChains[target].push_back(constraint.moverPath);
            }
        }
        for (auto &[joint, writers] : walkWriters) {
            newJointBinding[joint] = std::move(writers);
        }
        for (auto &[joint, chain] : walkChains) {
            newFrameChains[joint] = chain;
        }
    }

    compileBlocks.Close();
    stampCompileRegion("Compile.SolverSchedule");
    auto newFirstFramePoseTaps = std::make_unique<RigExecTapSet>(_stage);
    std::map<SdfPath, RigExecTapId> newFirstFramePoseFrames, newFirstFramePoseRests;
    // What used to be one PrepareRequests.ProviderTaps block, 22.5 ms with
    // nothing inside it to say which of its three quite different passes was
    // spending them: the seed sweep over joints, controls, volume weights,
    // solver frame inputs and constraints; the pose-provider closure warmed
    // over the seeded set; and the per-provider connected-tap prepare. Split
    // here rather than nested one level deeper, because these run one after
    // another in a single block -- the case RigExecProfilePhases exists for --
    // and because a nested scope could not span the early return in the third
    // pass, whereas compileBlocks closes itself on destruction.
    compileBlocks.Next("PrepareRequests.ProviderSeed");
    // The rest taps are added below, once the provider set is closed, because
    // whether they belong in the per-frame request or in the epoch request
    // depends on the whole set (see newRestsMightVary).
    //
    // Already-seeded is a STOP, not a skip. A path only gets into
    // newFirstFramePoseFrames by way of this loop, which has no early exit of
    // its own and climbs from wherever it started all the way to the root --
    // so by the time any call returns, every frame-provider ancestor of every
    // path it seeded is in the map too, and nothing in this pass ever erases
    // from it. Meeting a seeded path therefore means the whole chain above it
    // is already done, and continuing merely re-runs isFrameProvider on
    // ancestors to reach a conclusion already reached. A non-provider still
    // has to `continue`: it says nothing about its ancestors, which may be
    // providers that no earlier call reached.
    //
    // (Within a single call the guard cannot fire on a path this call itself
    // seeded: the chain strictly ascends, so no path is visited twice.)
    auto seedProvider = [&](SdfPath path) {
        for (; !path.IsEmpty() && path != SdfPath::AbsoluteRootPath();
             path = path.GetParentPath()) {
            if (newFirstFramePoseFrames.count(path)) break;
            if (!isFrameProvider(path)) continue;
            newFirstFramePoseFrames[path] = newFirstFramePoseTaps->Add(
                RigExecValueAddress::Prim(path, _computePointFrame));
        }
    };
    for (const SdfPath &path : newJointPaths) seedProvider(path);
    for (const SdfPath &path : newControlPaths) seedProvider(path);
    for (const auto &[path, tap] : newVolumeWeightMatrixTaps) seedProvider(path);
    for (const auto &[solver, frames] : solverFrameInputs) {
        for (const SdfPath &path : frames) seedProvider(path);
    }
    for (const _FrameConstraint &constraint : newFrameConstraints) {
        for (const SdfPath &target : constraint.targets) seedProvider(target);
        for (const auto &source : constraint.sources) seedProvider(source.sourcePath);
        seedProvider(constraint.worldUpObject.sourcePath);
        seedProvider(constraint.effector.sourcePath);
        for (const auto &pole : constraint.poleObjects) seedProvider(pole.sourcePath);
    }
    compileBlocks.Next("PrepareRequests.ProviderClosure");
    // Warming the closure over the seeded set is what grows newPoseInputInfo,
    // which the pass after this one iterates -- so the two are timed apart:
    // this one is stage walking, that one is exec prepares.
    std::vector<SdfPath> seedPaths;
    for (const auto &[provider, tap] : newFirstFramePoseFrames) seedPaths.push_back(provider);
    for (const SdfPath &provider : seedPaths) poseProviderClosure(provider);
    compileBlocks.Next("PrepareRequests.ConnectedPoseTaps");
    std::map<SdfPath, std::set<SdfPath>> newPoseProviderInputs;
    std::map<SdfPath, std::unique_ptr<RigExecTapSet>> newConnectedPoseTaps;
    for (const auto &[provider, info] : newPoseInputInfo) {
        seedProvider(provider);
        newPoseProviderInputs[provider] = info.providers;
        if (info.connectedPose && !newJointBinding.count(provider)) {
            auto taps = std::make_unique<RigExecTapSet>(_stage);
            taps->Add(RigExecValueAddress::Prim(provider, _computePointFrame));
            const bool connectedPrepared = [&]() {
                RIGEXEC_PROFILE_SCOPE_CAT(
                    _profiler, "TapPrepare connected", "compile");
                return taps->Prepare();
            }();
            if (!connectedPrepared) {
                reportError("failed to prepare connected pose provider " + provider.GetString());
                restorePreviousEpoch();
                return false;
            }
            newConnectedPoseTaps[provider] = std::move(taps);
        }
    }
    compileBlocks.Next("PrepareRequests.RestTaps");
    // Rest frames: one request for the whole epoch, or per-frame taps when
    // some provider's rest channels can move with time.
    //
    // computeRestFrame reads rest:space and the six rest avars of the
    // provider and of every RigExec ancestor, and nothing else. When none of
    // those can change within the epoch, every frame's answer is the same
    // answer, so it is pulled once here instead of 326 times per second. A
    // rest channel that is connected, that carries time samples anywhere in
    // its composition, or that a property chain writes keeps the old
    // per-frame taps: the frozen value would be wrong for it.
    //
    // Compile is not the last word on this. The epoch digest hashes no rest
    // channel, so an edit that ANIMATES one later does not recompile by
    // itself; _SettleEpoch re-asks the same question on every notice and
    // rebuilds the epoch when the answer has changed.
    bool newRestsMightVary = false;
    auto newRestTaps = std::make_unique<RigExecTapSet>(_stage);
    std::map<SdfPath, RigExecTapId> newRestTapIds;
    {
        RIGEXEC_PROFILE_SCOPE_CAT(_profiler, "RestTimeVarying", "compile");
        std::set<SdfPath> chainTargets;
        for (const auto &[target, revisions] : newPropertyChains) {
            chainTargets.insert(target);
        }
        for (const auto &[provider, tap] : newFirstFramePoseFrames) {
            if (_ProviderRestMightVary(_stage, provider, chainTargets)) {
                newRestsMightVary = true;
                break;
            }
        }
    }
    for (const auto &[provider, tap] : newFirstFramePoseFrames) {
        const RigExecValueAddress address =
            RigExecValueAddress::Prim(provider, TfToken("computeRestFrame"));
        if (newRestsMightVary) {
            newFirstFramePoseRests[provider] = newFirstFramePoseTaps->Add(address);
        } else {
            newRestTapIds[provider] = newRestTaps->Add(address);
        }
    }

    compileBlocks.Next("PrepareRequests.ExecPrepare");
    if (newFirstFramePoseFrames.empty()) {
        newFirstFramePoseTaps.reset();
    } else {
        const bool seedPrepared = deferExecPrep ? true : [&]() {
            RIGEXEC_PROFILE_SCOPE_CAT(
                _profiler, "TapPrepare firstFramePose", "compile");
            return newFirstFramePoseTaps->Prepare();
        }();
        if (!seedPrepared) {
            reportError("failed to prepare pose provider inputs");
            restorePreviousEpoch();
            return false;
        }
    }

    const bool mainPrepared = deferExecPrep ? true : [&]() {
        RIGEXEC_PROFILE_SCOPE_CAT(
            _profiler, "TapPrepare main", "compile");
        return newTaps->Prepare();
    }();
    if (!mainPrepared) {
        reportError("failed to build a valid prepared request for the "
                    "new epoch");
        restorePreviousEpoch();
        return false;
    }

    compileBlocks.Next("PrepareRequests.GuideTaps");
    // Observational solver-guide taps prepare separately so a failing or
    // unused aggregate solver never gates the authoritative rig request;
    // preparation failure simply drops solver guide drawing.
    auto newGuideTaps = std::make_unique<RigExecTapSet>(_stage);
    for (const SdfPath &solverPath : solverArrayPaths) {
        newSolverArrayTaps[solverPath] = newGuideTaps->Add(
            RigExecValueAddress::Prim(solverPath, _computePointFrameArray));
    }
    const bool guidesPrepared =
        newSolverArrayTaps.empty() ? false : [&]() {
            RIGEXEC_PROFILE_SCOPE_CAT(
                _profiler, "TapPrepare guides", "compile");
            return newGuideTaps->Prepare();
        }();
    if (!guidesPrepared) {
        newGuideTaps.reset();
        newSolverArrayTaps.clear();
    }
    compileBlocks.Next("PrepareRequests.RestPull");
    // The epoch's rest frames, pulled once, at the stage's start time -- the
    // frame a session opens on, and, with no time-varying rest channel in
    // the epoch, the same frames every other time code would give.
    //
    // A real time code and never Default: a Default pull is a different mode
    // for exec, not a different instant. Values it computes into the shared
    // executor are time-independent by construction and a later ChangeTime
    // to a real frame does not invalidate them, so a Default pull here would
    // hand the first frame values that ignore its time samples
    // (testRigExecConstraints' autoDetect IK case catches exactly that).
    const UsdTimeCode restTime =
        UsdTimeCode(_stage ? _stage->GetStartTimeCode() : 0.0);

    compileBlocks.Next("PrepareRequests.WarmCompute");
    // Pay the first frame's warm compute here.
    //
    // Every Evaluate warms the shared executor before its override-bearing
    // pull (see FirstFramePose), and the first warm of a session computes the whole
    // seed network from an empty cache -- which is most of what makes the
    // first frame cost several times the frames after it. None of that work
    // depends on which frame is asked for first, so it is done once here, at
    // the time a session opens on. It is the same call the first Evaluate
    // would make; nothing is read from it and no value is published.
    //
    // Before the rest pull below, not after: a provider's rest frame is an
    // input to its point frame, so the warm computes the rests too and the
    // pull becomes a copy-out of values that are already there.
    // Skipped with the preparations it belongs to. Warm() PREPARES a request
    // it finds unprepared, so leaving this in a deferred epoch would do the
    // deferred work here under another name -- which is exactly what the
    // first attempt at this did, and the trace said so: the warm went from
    // 6.8 ms to 22.5 ms and the compile barely moved.
    if (newFirstFramePoseTaps && !deferExecPrep) {
        RIGEXEC_PROFILE_SCOPE_CAT(_profiler, "Compile.WarmFirstFramePose", "compile");
        newFirstFramePoseTaps->Warm(restTime);
    }

    std::map<SdfPath, RigExecPointFrame> newEpochRestFrames;
    if (!newRestTapIds.empty()) {
        const bool restsPrepared = [&]() {
            RIGEXEC_PROFILE_SCOPE_CAT(
                _profiler, "TapPrepare restFrames", "compile");
            return newRestTaps->Prepare();
        }();
        RigExecSnapshot restSnapshot;
        if (restsPrepared) {
            RIGEXEC_PROFILE_SCOPE_CAT(
                _profiler, "Compile.RestFrames", "compile");
            restSnapshot = newRestTaps->Evaluate(restTime);
        }
        if (!restsPrepared || !restSnapshot.IsValid() ||
            !restSnapshot.IsComplete()) {
            reportError("failed to evaluate the rig's rest frames");
            restorePreviousEpoch();
            return false;
        }
        for (const auto &[provider, tap] : newRestTapIds) {
            newEpochRestFrames.emplace_hint(
                newEpochRestFrames.end(), provider,
                restSnapshot.Get<RigExecPointFrame>(tap));
        }
    }

    // Every real request is prepared; the warm-up has nothing left to hold
    // open. Destroyed here, on the compiling thread, for the reason its
    // construction is here.
    warmupTaps.reset();

    compileBlocks.Close();
    stampCompileRegion("Compile.PrepareRequests");
    // Commit the new epoch atomically with respect to evaluator state.
    _movers = std::move(newMovers);
    _jointPaths = std::move(newJointPaths);
    _controlPaths = std::move(newControlPaths);
    _poseInterpolators = std::move(newPoseInterpolators);
    _poseWeightProperties.clear();
    for (const _PoseInterpolator &interpolator : _poseInterpolators) {
        _poseWeightProperties.insert(_poseWeightProperties.end(),
                                     interpolator.disabledPoseWeights.begin(),
                                     interpolator.disabledPoseWeights.end());
        _poseWeightProperties.insert(_poseWeightProperties.end(),
                                     interpolator.poseWeights.begin(),
                                     interpolator.poseWeights.end());
    }
    _controlFrameTaps = std::move(newControlFrameTaps);
    _frameConstraints = std::move(newFrameConstraints);
    _frameChains = std::move(newFrameChains);
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
    {
        // Join the digest task: from here on its result is read.
        RIGEXEC_PROFILE_SCOPE_CAT(_profiler, "Compile.DigestJoin", "compile");
        digestDispatcher.Wait();
    }
    _structureDigest = newDigest;
    // Set with the requests it describes, and only once they are the
    // evaluator's: a compile that turned back before here left the previous
    // epoch standing, and its flag with it.
    _execPrepDeferred = deferExecPrep;
    _taps = std::move(newTaps);
    _guideTaps = std::move(newGuideTaps);
    _guideInputs.clear();
    _guideTime = UsdTimeCode::Default();
    _guideSnapshot = RigExecSnapshot();
    _guideDirty = true;
    _jointFrameTaps = std::move(newJointFrameTaps);
    _jointFinalFrameTaps = std::move(newJointFinalFrameTaps);
    _jointFinalMatrixTaps = std::move(newJointFinalMatrixTaps);
    _jointSolverBinding = std::move(newJointBinding);
    _poseProviderInputs = std::move(newPoseProviderInputs);
    _connectedPoseTaps = std::move(newConnectedPoseTaps);
    _connectedPoseCache.clear();
    _namespaceInheritsCache.clear();
    _nearestBlockingCache.clear();
    _poseSteps = std::move(newPoseSteps);
    _firstFramePoseTaps = std::move(newFirstFramePoseTaps);
    _firstFramePoseCache.Clear();
    _firstFramePoseDirty = true;
    _authSnapshotCache.Clear();
    _authSnapTimeKeyed.clear();
    _authSnapshotDirty = true;
    _firstFramePoseFrames = std::move(newFirstFramePoseFrames);
    _hierarchicalProviderSet.clear();
    _hierarchicalProviderSet.reserve(_firstFramePoseFrames.size());
    for (const auto &[provider, tap] : _firstFramePoseFrames)
        _hierarchicalProviderSet.insert(provider);
    _firstFramePoseRests = std::move(newFirstFramePoseRests);
    _restTaps = std::move(newRestTaps);
    _restTapIds = std::move(newRestTapIds);
    _epochRestFrames = std::move(newEpochRestFrames);
    _restTime = restTime;
    // Anchors and intervening-Xform candidates for the new epoch. Both are
    // pure namespace topology; recomputing them per frame cost an xform-cache
    // query per provider only to be told "identity" on every rig that has no
    // such Xform, which is nearly all of them.
    _poseProviderAnchors.clear();
    _interveningXformProviders.clear();
    {
        const SdfPath assetRootPath = _rigPath.GetParentPath();
        for (const auto &[provider, tap] : _firstFramePoseFrames) {
            SdfPath anchorPath;
            for (SdfPath walk = provider.GetParentPath();
                 !walk.IsEmpty() && !walk.IsAbsoluteRootPath() &&
                     walk != assetRootPath;
                 walk = walk.GetParentPath()) {
                if (_firstFramePoseFrames.count(walk)) {
                    anchorPath = walk;
                    break;
                }
            }
            _poseProviderAnchors[provider] = anchorPath;
            if (provider.GetParentPath() !=
                (anchorPath.IsEmpty() ? assetRootPath : anchorPath)) {
                _interveningXformProviders.push_back(provider);
            }
        }
    }
    _solverBatches = std::move(newSolverBatches);
    _solverJoints = std::move(newSolverJoints);
    _solverDependencies = std::move(newSolverDependencies);
    _solverInputBatches = std::move(newSolverInputBatches);
    _solverArrayTaps = std::move(newSolverArrayTaps);
    _graphChains = std::move(newGraphChains);
    _graphDerivedChains = std::move(newGraphDerivedChains);
    // Both caches below are keyed by nothing but a path, so the epoch they
    // belong to has to be stated by emptying them when it ends: the
    // influence table a layout was range-checked against and the base points
    // a source holds are both things a recompile can have changed.
    _skinTopologies.Clear();
    _skinLayoutInputsValid = false;
    _blendSampleShapes.Clear();
    for (auto &[target, live] : _liveGraphs) {
        if (live) live->basePointsPushed = false;
    }
    // Derived results are keyed by target like the live graphs, and the
    // walk may read them from several tasks at once: every entry exists
    // before a generation starts, so no task ever inserts into the map.
    _derivedCache.clear();
    for (const auto &[chainTarget, revisions] : _graphDerivedChains) {
        for (const _GraphRevision &derived : revisions) {
            _derivedCache[derived.target];
        }
    }
    _propertyChains = std::move(newPropertyChains);
    _propertyChainOrder = std::move(newPropertyChainOrder);
    // The bindings describe the chains entry for entry, so a recompile that
    // replaced them has replaced what the bindings are about.
    _propertyChainBindings.reset();

    stampCompileRegion("Compile.Commit");
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
                for (const auto &[inputPath, phase] : revision.binding.GetPhasedInputs()) {
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
        // A ready queue avoids quadratic rescans when path ordering is the
        // reverse of a long dependency chain. The set preserves determinism.
        std::map<SdfPath, size_t> pending;
        std::map<SdfPath, std::vector<SdfPath>> consumers;
        std::set<SdfPath> ready;
        for (const auto &[target, producers] : dependsOn) {
            pending[target] = producers.size();
            if (producers.empty()) ready.insert(target);
            for (const SdfPath &producer : producers) {
                consumers[producer].push_back(target);
            }
        }
        while (!ready.empty()) {
            const SdfPath target = *ready.begin();
            ready.erase(ready.begin());
            emitted.insert(target);
            order.push_back(target);
            for (const SdfPath &consumer : consumers[target]) {
                if (--pending[consumer] == 0) ready.insert(consumer);
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

        // Dependency levels over that order: a new level begins where a chain
        // reads one already in the current level. Greedy over _chainOrder
        // rather than "longest path from a root", so every level is a
        // CONTIGUOUS run of the order the walk takes anyway -- which is what
        // lets a level be spread over tasks without moving anything the walk
        // publishes.
        _chainLevels.clear();
        std::set<SdfPath> inCurrentLevel;
        for (const SdfPath &target : _chainOrder) {
            bool readsCurrentLevel = false;
            const auto producers = dependsOn.find(target);
            if (producers != dependsOn.end()) {
                for (const SdfPath &producer : producers->second) {
                    if (inCurrentLevel.count(producer)) {
                        readsCurrentLevel = true;
                        break;
                    }
                }
            }
            if (_chainLevels.empty() || readsCurrentLevel) {
                _chainLevels.emplace_back();
                inCurrentLevel.clear();
            }
            _chainLevels.back().targets.push_back(target);
            inCurrentLevel.insert(target);
        }
        for (_ChainLevel &level : _chainLevels) {
            level.parallel = _IsChainLevelParallelSafe(level.targets);
        }
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
                for (const auto &[inputPath, phase] : revision.binding.GetPhasedInputs()) {
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
                        if (inputPath == target) {
                            reportError(who + " creates a self dependency on the final output");
                            restorePreviousEpoch();
                            return false;
                        }
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

    stampCompileRegion("Compile.ChainOrderValidate");
    _compiled = true;
    // Build the dense provider universe once per compile epoch. _providerPaths
    // is SdfPath-sorted, so ascending live-index iteration reproduces the
    // std::map key order the frame-domain publish loops previously relied on.
    {
        std::set<SdfPath> universe;
        auto addU = [&](const SdfPath &p) { if (!p.IsEmpty()) universe.insert(p); };
        for (const auto &kv : _firstFramePoseFrames) addU(kv.first);
        for (const auto &p : _xformDerivedProviders) addU(p);
        for (const auto &p : _interveningXformProviders) addU(p);
        for (const _FrameConstraint &fc : _frameConstraints) {
            addU(fc.moverPath);
            for (const auto &t : fc.targets) addU(t);
            for (const auto &s : fc.sources) addU(s.sourcePath);
            addU(fc.worldUpObject.sourcePath);
            addU(fc.effector.sourcePath);
            for (const auto &po : fc.poleObjects) addU(po.sourcePath);
            for (const auto &ik : fc.ikChain) addU(ik);
            addU(fc.pointsTarget);
            addU(fc.weightObject);
        }
        for (const auto &kv : _solverJoints) {
            addU(kv.first);
            for (const auto &p : kv.second) addU(p.first);
        }
        for (const _SolverBatch &b : _solverBatches) {
            for (const auto &d : b.dependencies) addU(d);
            for (const auto &d : b.frameInputs) addU(d);
            for (const auto &kv : b.restInputs) { addU(kv.first); addU(kv.second); }
            for (const auto &kv : b.solvers) addU(kv.first);
        }
        for (const auto &kv : _poseProviderInputs) {
            addU(kv.first);
            for (const auto &v : kv.second) addU(v);
        }
        for (const auto &kv : _snapshotPoints) {
            addU(kv.first);
            for (const auto &v : kv.second) addU(v);
        }
        _providerPaths.assign(universe.begin(), universe.end());
        _providerIndex.clear();
        _providerIndex.reserve(_providerPaths.size());
        for (std::size_t i = 0; i < _providerPaths.size(); ++i)
            _providerIndex.emplace(_providerPaths[i], static_cast<int>(i));
        _hierDescendants.assign(_providerPaths.size(), {});
        for (std::size_t i = 0; i < _providerPaths.size(); ++i) {
            if (_firstFramePoseFrames.count(_providerPaths[i]) == 0) continue;
            for (std::size_t j = i + 1;
                 j < _providerPaths.size() &&
                 _providerPaths[j].HasPrefix(_providerPaths[i]); ++j)
                if (_firstFramePoseFrames.count(_providerPaths[j]))
                    _hierDescendants[i].push_back(static_cast<int>(j));
        }
    }
    // Remove targets that no longer publish an output. Existing target graphs
    // survive a new binding epoch and are spliced lazily on the next pull.
    std::set<SdfPath> retainedTargets;
    for (const auto &[target, revisions] : _graphChains) retainedTargets.insert(target);
    for (const auto &[target, revisions] : _graphDerivedChains) {
        for (const auto &revision : revisions) retainedTargets.insert(revision.target);
    }
    for (auto it = _liveGraphs.begin(); it != _liveGraphs.end();) {
        if (!retainedTargets.count(it->first)) it = _liveGraphs.erase(it);
        else ++it;
    }
    // Every node the walk will index, created now rather than on the frame
    // that first needs it: chains in one level run concurrently, and a map
    // insertion under a concurrent read is a race. The mapped graph stays
    // null until its chain first runs, which is what it already meant.
    for (const SdfPath &target : retainedTargets) {
        _liveGraphs[target];
    }
    _structureDirty = false;
    // The baked program is epoch state like every other compiled table, so
    // it is built here rather than lazily on the first frame -- which would
    // charge one interactive frame for the whole bake.
    // An epoch the program cannot express is not a compile error: the rig
    // evaluates dynamically and IsBakeable says why.
    //
    // The rig's own request is re-read first, and here rather than at the
    // head of Compile: rigExec:baked is composed, so a reference swap or a
    // muted layer can change the answer with nothing else on the stage
    // moving, and the rebuild below has to be told which path this new epoch
    // is for. It is ignored where a tool or the environment already chose --
    // that is the whole of the precedence rule.
    _RefreshAttributeEvaluationMode();
    _RebuildBakedProgram(std::move(retiringBakedProgram));
    return true;
}

std::vector<SdfPath>
RigExecRigEvaluator::GetChainLevelTargets(size_t level) const
{
    return level < _chainLevels.size() ? _chainLevels[level].targets
                                       : std::vector<SdfPath>();
}

bool
RigExecRigEvaluator::IsChainLevelParallel(size_t level) const
{
    return level < _chainLevels.size() && _chainLevels[level].parallel;
}

bool
RigExecRigEvaluator::_IsChainLevelParallelSafe(
    const std::vector<SdfPath> &targets) const
{
    // Two chains are not enough work to pay for the dispatch and the join,
    // and a rig with two deformed meshes is far more common than one with
    // twenty.
    if (targets.size() < 3) {
        return false;
    }
    std::set<SdfPath> levelWeightObjects;
    for (const SdfPath &target : targets) {
        const auto chain = _graphChains.find(target);
        if (chain == _graphChains.end()) {
            return false;
        }
        std::set<SdfPath> chainWeightObjects;
        for (const _GraphRevision &revision : chain->second) {
            // A Profile Mover reads its curvenet's posed points out of the
            // generation being built and binds through a cut/factorization
            // cache every profile chain shares. Both are things a serial walk
            // has finished with before the next chain asks.
            if (revision.op == RigExecRevisionOp::Curvenet) {
                return false;
            }
            // Defence in depth against a future edge type, not a hazard the
            // dependency graph can currently produce: a phased read is an
            // edge addEdge already records, so a phased reader and the chain
            // that produces what it reads land in different levels, and a
            // phase read WITHIN a chain is served from that task's own
            // snapshots. No level the partition builds today holds a phased
            // read across its own chains. It stays because the cost is one
            // level's parallelism on a rig that has any phased read at all,
            // and the alternative is that a new edge kind -- one addEdge does
            // not know to record -- would make a level silently read the
            // chain-snapshot store mid-level, where what this level's chains
            // have recorded has not arrived yet.
            if (!revision.binding.phases.empty()) {
                return false;
            }
            for (const auto &blendInput : revision.binding.blendSamples) {
                for (const RigExecBlendSampleBinding &sample :
                         blendInput.second) {
                    if (!sample.phase.IsBase()) {
                        return false;
                    }
                }
            }
            if (revision.binding.weightObject.IsEmpty()) {
                continue;
            }
            // A `current` sample phase measures the field against the points
            // as they stand mid-chain, which re-enters weight resolution --
            // the one part of assembling a packet that is not a pure read of
            // the stage and this generation's results.
            if (_currentPhaseWeights.count(revision.binding.weightObject)) {
                return false;
            }
            chainWeightObjects.insert(revision.binding.weightObject);
        }
        // One weight object driving two chains in the level: they publish the
        // same pose.weightFields entry, so which field a rigger is shown
        // would become a question about the walk rather than about the rig.
        // Schema validation reaches that case first today -- a weight
        // object's target has to be the mover's own -- so this is the walk
        // declining to depend on a rule enforced somewhere else.
        for (const SdfPath &weightObject : chainWeightObjects) {
            if (!levelWeightObjects.insert(weightObject).second) {
                return false;
            }
        }
    }
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
    SdfPathVector spaces;
    if (UsdRelationship rel =
            prim.GetRelationship(TfToken("rigExec:transformSpace"))) {
        rel.GetTargets(&spaces);
    }
    if (spaces.size() > 1) {
        *error = who + ": rigExec:transformSpace takes at most one target";
        return false;
    }
    if (!spaces.empty()) {
        const UsdPrim spacePrim = _stage->GetPrimAtPath(spaces[0]);
        if (!spacePrim || !frameProviderTypes.count(spacePrim.GetTypeName())) {
            *error = who + ": rigExec:transformSpace target is not a "
                           "catalogued matrix provider";
            return false;
        }
    }
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
RigExecRigEvaluator::_ValidateSkinMover(
    const UsdPrim &prim,
    const RigExecMoverRecord &record,
    std::string *error) const
{
    const std::string who = "SkinMover " + prim.GetPath().GetString();
    // The target rules are the matrix mover's: one native points property,
    // because the jointIndices/jointWeights layout is written against one
    // point count and a fan-out would alias it across targets.
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
    const UsdAttribute targetAttr =
        _stage->GetAttributeAtPath(record.targets[0]);
    if (!targetAttr ||
        targetAttr.GetTypeName() != SdfValueTypeNames->Point3fArray) {
        *error = who + ": move target is not an exact point3f[] property";
        return false;
    }

    SdfPathVector influences;
    if (UsdRelationship rel =
            prim.GetRelationship(TfToken("rigExec:influences"))) {
        rel.GetTargets(&influences);
    }
    if (influences.empty()) {
        *error = who + ": rigExec:influences must name at least one matrix "
                       "provider";
        return false;
    }
    static const std::set<TfToken> frameProviderTypes = {
        TfToken("RigExecControl"), TfToken("RigExecJoint")};
    for (const SdfPath &influence : influences) {
        const UsdPrim provider = _stage->GetPrimAtPath(influence);
        if (!provider || !frameProviderTypes.count(provider.GetTypeName())) {
            *error = who + ": rigExec:influences target " +
                     influence.GetString() +
                     " is not a catalogued matrix provider";
            return false;
        }
    }
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

    // Strict about the method: a declared token the kernel cannot honour is
    // a compile error, never a silent fallback to different maths.
    TfToken method("classicLinear");
    if (UsdAttribute a = prim.GetAttribute(TfToken("rigExec:skinningMethod"))) {
        a.Get(&method);
    }
    if (method != "classicLinear" && method != "dualQuaternion") {
        *error = who + ": unknown rigExec:skinningMethod '" +
                 method.GetString() + "'";
        return false;
    }

    // The layout is a value, re-validated by the assembler at every
    // evaluation; checking the authored default here is what turns a
    // mis-sized export into a compile diagnostic instead of a silently
    // passed-through mesh.
    int elementSize = 1;
    if (UsdAttribute a = prim.GetAttribute(TfToken("rigExec:elementSize"))) {
        a.Get(&elementSize);
    }
    if (elementSize < 1) {
        *error = who + ": rigExec:elementSize must be at least 1";
        return false;
    }
    VtIntArray indices;
    VtFloatArray weights;
    if (UsdAttribute a = prim.GetAttribute(TfToken("rigExec:jointIndices"))) {
        a.Get(&indices);
    }
    if (UsdAttribute a = prim.GetAttribute(TfToken("rigExec:jointWeights"))) {
        a.Get(&weights);
    }
    if (indices.size() != weights.size()) {
        *error = who + ": rigExec:jointIndices length " +
                 std::to_string(indices.size()) +
                 " must equal rigExec:jointWeights length " +
                 std::to_string(weights.size());
        return false;
    }
    VtVec3fArray points;
    if (targetAttr.Get(&points) && !points.empty() &&
        indices.size() != points.size() * size_t(elementSize)) {
        *error = who + ": rigExec:jointIndices length " +
                 std::to_string(indices.size()) + " must equal " +
                 std::to_string(points.size()) + " points * elementSize " +
                 std::to_string(elementSize);
        return false;
    }
    for (size_t i = 0; i < indices.size(); ++i) {
        if (indices[i] < 0 || size_t(indices[i]) >= influences.size()) {
            *error = who + ": rigExec:jointIndices[" + std::to_string(i) +
                     "] = " + std::to_string(indices[i]) +
                     " is outside the " + std::to_string(influences.size()) +
                     " influences";
            return false;
        }
        if (!std::isfinite(weights[i]) || weights[i] < 0.0f) {
            *error = who + ": rigExec:jointWeights[" + std::to_string(i) +
                     "] must be finite and non-negative";
            return false;
        }
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
RigExecRigEvaluator::_ResolveBlendSampleLayout(
    const SdfPath &blendShapePath, size_t pointCount,
    RigExecBlendSampleLayout *layout) const
{
    layout->pointCount = pointCount;
    layout->valid = false;

    const UsdSkelBlendShape shape(
        _stage->GetPrimAtPath(blendShapePath.GetPrimPath()));
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
    if (typeName == "RigExecCurvenetWeight") {
        auto array = [&](const char *relationship, auto *out) {
            SdfPathVector paths;
            prim.GetRelationship(TfToken(relationship)).GetTargets(&paths);
            return paths.size() == 1 && _resolvedInputs.GetAttribute(
                _stage->GetAttributeAtPath(paths[0]), time, out);
        };
        VtVec3fArray mesh, net;
        VtIntArray counts, indices, splines, smooth;
        VtFloatArray authored;
        if (!array("rigExec:weightTarget", &mesh) || !array("rigExec:curvenetPoints", &net) ||
            !array("rigExec:meshFaceCounts", &counts) || !array("rigExec:meshFaceIndices", &indices) ||
            !array("rigExec:curvenetSplineIndices", &splines)) {
            *error = "unresolved curvenet weight geometry"; return false;
        }
        _resolvedInputs.GetAttribute(prim.GetAttribute(TfToken("inputs:weights")), time, &authored);
        _resolvedInputs.GetAttribute(prim.GetAttribute(TfToken("rigExec:autoSmooth")), time, &smooth);
        const auto packet = RigExecComputeCurvenetWeightPacket(
            {mesh.begin(),mesh.end()}, {counts.begin(),counts.end()}, {indices.begin(),indices.end()},
            {net.begin(),net.end()}, {splines.begin(),splines.end()},
            _ResolvedRead(_resolvedInputs,prim,"rigExec:basis",TfToken("catmullRom"),time),
            _ResolvedRead(_resolvedInputs,prim,"rigExec:samplesPerSpline",5,time),
            {smooth.begin(),smooth.end()}, {authored.begin(),authored.end()},
            _ResolvedRead(_resolvedInputs,prim,"rigExec:rangePolicy",TfToken("clamp"),time),
            _ResolvedRead(_resolvedInputs,prim,"rigExec:unreachedValue",0.0f,time),error);
        return packet.ResolveAll(count, weights);
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
    const std::unordered_map<SdfPath, GfMatrix4d, SdfPath::Hash> &baseProviderMatrices,
    const std::unordered_map<SdfPath, GfMatrix4d, SdfPath::Hash> &finalProviderMatrices,
    UsdTimeCode time,
    std::vector<std::string> *diagnostics,
    const std::unordered_map<SdfPath, GfMatrix4d, SdfPath::Hash> &geometryConstraintDeltas) const
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

        if (_IsSourceFrameConstraintType(prim.GetTypeName())) {
            const auto delta = geometryConstraintDeltas.find(mover->moverPath);
            if (delta == geometryConstraintDeltas.end()) continue;
            for (auto &point : points)
                point = GfVec3f(delta->second.TransformAffine(GfVec3d(point)));
        } else if (type == "RigExecBlendShapeMover") {
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
                // Through the resolver, NOT straight off the stage.
                //
                // An authored `inputs:weight` and a DRIVEN one are the same
                // attribute; a plain Get() sees only the first. The moment
                // anything connects a weight -- which is the entire point of
                // a pose-space rig, where an interpolator drives every
                // corrective -- this oracle read 0 while the real path read
                // the driven value, and then reported the disagreement as a
                // parity mismatch in the blend kernel. The packet assembly
                // has always used _resolvedInputs here; this is the oracle
                // catching up to it.
                _resolvedInputs.GetAttribute(
                    input.GetAttribute(TfToken("inputs:weight")), time,
                    &channel);
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
                    SdfPathVector shapeTargets, sparseTargets;
                    if (UsdRelationship rel = sample.GetRelationship(
                            TfToken("rigExec:targetPoints"))) {
                        rel.GetTargets(&shapeTargets);
                    }
                    if (UsdRelationship rel = sample.GetRelationship(
                            TfToken("rigExec:blendShape"))) {
                        rel.GetTargets(&sparseTargets);
                    }
                    if (shapeTargets.size() + sparseTargets.size() != 1) {
                        diagnostics->push_back(
                            "MoverFailed " + mover->moverPath.GetString() +
                            ": blend sample must name exactly one of "
                            "rigExec:targetPoints or rigExec:blendShape: " +
                            samplePath.GetString());
                        failed = true;
                        break;
                    }
                    _Sample s;
                    s.path = sample.GetPath();
                    if (sparseTargets.size() == 1) {
                        // The oracle is a SCALAR REFERENCE, so it reads the
                        // blend shape straight off the stage and expands it
                        // to a full moved-points array -- deliberately not
                        // through RigExecBlendSampleCache. An oracle that
                        // shared the fast path's cache would agree with it
                        // about a stale layout, which is precisely the class
                        // of bug this exists to catch.
                        RigExecBlendSampleLayout layout;
                        _ResolveBlendSampleLayout(sparseTargets[0],
                                                  basePoints.size(), &layout);
                        if (!layout.valid) {
                            diagnostics->push_back(
                                "MoverFailed " + mover->moverPath.GetString() +
                                ": unusable blend shape at " +
                                sparseTargets[0].GetString());
                            failed = true;
                            break;
                        }
                        s.shape = VtVec3fArray(basePoints.begin(),
                                               basePoints.end());
                        if (layout.indices.empty()) {
                            for (size_t i = 0; i < layout.offsets.size(); ++i) {
                                s.shape[i] += layout.offsets[i];
                            }
                        } else {
                            for (size_t k = 0; k < layout.indices.size(); ++k) {
                                s.shape[size_t(layout.indices[k])] +=
                                    layout.offsets[k];
                            }
                        }
                        s.activation = 1;
                        if (UsdAttribute a = sample.GetAttribute(
                                TfToken("rigExec:activation"))) {
                            a.Get(&s.activation, time);
                        }
                        if (!std::isfinite(s.activation) ||
                            s.activation <= 0) {
                            diagnostics->push_back(
                                "MoverFailed " + mover->moverPath.GetString() +
                                ": non-positive activation at " +
                                s.path.GetString());
                            failed = true;
                            break;
                        }
                        samples.push_back(std::move(s));
                        continue;
                    }
                    const UsdAttribute shapeAttr =
                        _stage->GetAttributeAtPath(shapeTargets[0]);
                    bool gotShape = shapeAttr && shapeAttr.Get(&s.shape, time);
                    const auto compiledChain = _graphChains.find(target);
                    if (compiledChain != _graphChains.end()) {
                        for (const auto &revision : compiledChain->second) {
                            if (revision.moverPath != mover->moverPath) continue;
                            const auto channel = revision.binding.blendSamples.find(inputPath);
                            if (channel == revision.binding.blendSamples.end()) continue;
                            for (const auto &binding : channel->second) {
                                if (binding.sample != samplePath) continue;
                                const VtValue *value = _chainSnapshots.Lookup(
                                    binding.points, binding.phase, mover->moverPath);
                                if (value && value->IsHolding<VtVec3fArray>()) {
                                    s.shape = value->UncheckedGet<VtVec3fArray>();
                                    gotShape = true;
                                }
                            }
                        }
                    }
                    if (!gotShape ||
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
                TfToken space("target");
                prim.GetAttribute(TfToken("rigExec:deltaSpace")).Get(&space);
                if (space == "surfaceFrame") {
                    VtIntArray counts, indices;
                    _stage->GetPrimAtPath(target.GetPrimPath()).GetAttribute(
                        TfToken("faceVertexCounts")).Get(&counts, time);
                    _stage->GetPrimAtPath(target.GetPrimPath()).GetAttribute(
                        TfToken("faceVertexIndices")).Get(&indices, time);
                    std::vector<GfVec3f> deltas(next.size()), transported;
                    for (size_t i = 0; i < next.size(); ++i) deltas[i] = next[i] - points[i];
                    if (RigExecTransportSurfaceOffsets(
                            std::vector<GfVec3f>(basePoints.begin(), basePoints.end()),
                            std::vector<GfVec3f>(points.begin(), points.end()),
                            std::vector<int>(counts.begin(), counts.end()),
                            std::vector<int>(indices.begin(), indices.end()), deltas, &transported)) {
                        for (size_t i = 0; i < next.size(); ++i) next[i] = points[i] + transported[i];
                    } else {
                        failed = true;
                        diagnostics->push_back("MoverFailed " + mover->moverPath.GetString() +
                            ": degenerate blend surface frame");
                    }
                }
                if (!failed) points = next;
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
            GfMatrix4d m = matrixIt->second;
            SdfPathVector spaces;
            if (UsdRelationship rel = prim.GetRelationship(
                    TfToken("rigExec:transformSpace"))) {
                rel.GetTargets(&spaces);
            }
            if (!spaces.empty()) {
                const auto spaceIt = matrices.find(spaces[0]);
                if (spaceIt == matrices.end()) {
                    diagnostics->push_back(
                        "MoverFailed " + mover->moverPath.GetString() +
                        ": no " + phase.GetString() +
                        " matrix provider at " + spaces[0].GetString());
                    continue;
                }
                m = RigExecMeasureInSpace(m, spaceIt->second);
            }
            for (size_t i = 0; i < points.size(); ++i) {
                const GfVec3d moved = RigExecApplyWeightedMatrix(
                    GfVec3d(points[i]), m, 1.0f);
                points[i] = GfVec3f(moved);
            }
        } else if (type == "RigExecSkinMover") {
            // p' = (1 - sum_k w_k) p + sum_k w_k T_k p per point, in double,
            // read straight off the stage: independent of
            // RigExecAssembleSkinParameters and of the SIMD kernel on
            // purpose, so parity is a real check.
            SdfPathVector influences;
            if (UsdRelationship rel =
                    prim.GetRelationship(TfToken("rigExec:influences"))) {
                rel.GetTargets(&influences);
            }
            TfToken phase("base");
            if (const UsdAttribute a = prim.GetAttribute(
                    TfToken("rigExec:transformReadPhase"))) {
                a.Get(&phase);
            }
            const auto &matrices =
                phase == "final" ? finalProviderMatrices
                                 : baseProviderMatrices;
            std::vector<GfMatrix4d> transforms;
            bool failed = false;
            for (const SdfPath &provider : influences) {
                const auto matrixIt = matrices.find(provider);
                if (matrixIt == matrices.end()) {
                    diagnostics->push_back(
                        "MoverFailed " + mover->moverPath.GetString() +
                        ": no " + phase.GetString() + " matrix provider at " +
                        provider.GetString());
                    failed = true;
                    break;
                }
                transforms.push_back(matrixIt->second);
            }
            if (failed) {
                continue;
            }
            VtIntArray indices;
            VtFloatArray weights;
            int elementSize = 1;
            TfToken method("classicLinear");
            if (const UsdAttribute a = prim.GetAttribute(
                    TfToken("rigExec:jointIndices"))) {
                if (!_resolvedInputs.Get(a.GetPath(), &indices)) {
                    a.Get(&indices, time);
                }
            }
            if (const UsdAttribute a = prim.GetAttribute(
                    TfToken("rigExec:jointWeights"))) {
                if (!_resolvedInputs.Get(a.GetPath(), &weights)) {
                    a.Get(&weights, time);
                }
            }
            if (const UsdAttribute a = prim.GetAttribute(
                    TfToken("rigExec:elementSize"))) {
                a.Get(&elementSize, time);
            }
            if (const UsdAttribute a = prim.GetAttribute(
                    TfToken("rigExec:skinningMethod"))) {
                a.Get(&method, time);
            }
            if (method != "classicLinear" && method != "dualQuaternion") {
                diagnostics->push_back(
                    "MoverFailed " + mover->moverPath.GetString() +
                    ": skinning method '" + method.GetString() +
                    "' has no scalar reference kernel");
                continue;
            }
            if (elementSize < 1 || transforms.empty() ||
                indices.size() != weights.size() ||
                indices.size() != points.size() * size_t(elementSize)) {
                diagnostics->push_back(
                    "MoverFailed " + mover->moverPath.GetString() +
                    ": jointIndices/jointWeights layout does not match "
                    "the target's point count");
                continue;
            }
            VtVec3fArray next = points;
            bool degenerate = false;
            if (method == "dualQuaternion") {
                // Independent DQS reference, written from the rule rather
                // than taken from the kernel. Every influence is split into
                // a pre-rotation stretch S_j and a rigid motion by the
                // library's polar split (the one piece shared with the
                // kernel: Gf's Factor is a Gram-Schmidt split that
                // legitimately disagrees with it under shear). Per point:
                // the pivot is the largest-weight slot; every influence
                // whose rotation opposes the pivot's is negated; the
                // complement 1 - sum w enters as the identity; the stretch
                // is sum w_j S_j + (1 - sum w) I; the sum is normalised
                // once; p' = (p S) rotated and translated. Accumulation,
                // normalisation and the point transform are Pixar's
                // GfDualQuatd, whose formulas differ from dualQuat.cpp's.
                std::vector<GfDualQuatd> rigid;
                std::vector<GfMatrix3d> stretch;
                for (const GfMatrix4d &t : transforms) {
                    const rigExec::RigExecScaledDualQuat sdq =
                        rigExec::RigExecScaledDualQuatFromMatrix(t);
                    rigid.emplace_back(sdq.rigid.real, sdq.rigid.dual);
                    stretch.push_back(sdq.stretch);
                }
                for (size_t i = 0; i < points.size() && !failed; ++i) {
                    int pivot = -1;
                    float pivotWeight = -1.0f;
                    for (int k = 0; k < elementSize; ++k) {
                        const size_t slot =
                            i * size_t(elementSize) + size_t(k);
                        const int j = indices[slot];
                        const float w = weights[slot];
                        if (j < 0 || size_t(j) >= transforms.size() ||
                            !std::isfinite(w) || w < 0.0f) {
                            failed = true;
                            break;
                        }
                        if (pivotWeight < w) {
                            pivotWeight = w;
                            pivot = j;
                        }
                    }
                    if (failed) {
                        break;
                    }
                    const GfQuatd pivotReal = rigid[pivot].GetReal();
                    GfDualQuatd sum = GfDualQuatd::GetZero();
                    GfMatrix3d s(0.0);
                    double total = 0.0;
                    for (int k = 0; k < elementSize; ++k) {
                        const size_t slot =
                            i * size_t(elementSize) + size_t(k);
                        const int j = indices[slot];
                        const double w = weights[slot];
                        if (w == 0.0) {
                            continue;
                        }
                        const double signedW =
                            GfDot(rigid[j].GetReal(), pivotReal) < 0.0 ? -w
                                                                       : w;
                        sum += rigid[j] * signedW;
                        s += stretch[j] * w;
                        total += w;
                    }
                    const double complement = 1.0 - total;
                    if (complement != 0.0) {
                        const GfDualQuatd identity =
                            GfDualQuatd::GetIdentity();
                        const double signedW =
                            GfDot(identity.GetReal(), pivotReal) < 0.0
                                ? -complement
                                : complement;
                        sum += identity * signedW;
                        s += GfMatrix3d(1.0) * complement;
                    }
                    if (sum.Normalize().first < 1e-9) {
                        degenerate = true;
                        break;
                    }
                    next[i] = GfVec3f(sum.Transform(GfVec3d(points[i]) * s));
                }
            } else {
                for (size_t i = 0; i < points.size() && !failed; ++i) {
                    const GfVec3d p(points[i]);
                    GfVec3d sum(0.0);
                    double total = 0.0;
                    for (int k = 0; k < elementSize; ++k) {
                        const size_t slot =
                            i * size_t(elementSize) + size_t(k);
                        const int j = indices[slot];
                        const float w = weights[slot];
                        if (j < 0 || size_t(j) >= transforms.size() ||
                            !std::isfinite(w) || w < 0.0f) {
                            failed = true;
                            break;
                        }
                        if (w == 0.0f) {
                            continue;
                        }
                        sum += transforms[j].TransformAffine(p) * double(w);
                        total += w;
                    }
                    next[i] = GfVec3f(p * (1.0 - total) + sum);
                }
            }
            if (failed) {
                diagnostics->push_back(
                    "MoverFailed " + mover->moverPath.GetString() +
                    ": jointIndices out of range or jointWeights not finite "
                    "and non-negative");
                continue;
            }
            if (degenerate) {
                diagnostics->push_back(
                    "MoverFailed " + mover->moverPath.GetString() +
                    ": degenerate dual-quaternion blend (over-driven "
                    "weights cancelled the rotation)");
                continue;
            }
            points = next;
        } else if (type == "RigExecCurveMover") {
            TfToken mode("ribbon");
            if (UsdAttribute a = prim.GetAttribute(TfToken("rigExec:mode"))) {
                a.Get(&mode, time);
            }
            if (mode == "wire") {
                // The wire reads its NURBS driver directly: posed control
                // points at the declared phase, rest control points, order
                // and knots as authored, and the authored bind coordinates.
                SdfPathVector curves, binds;
                if (UsdRelationship rel = prim.GetRelationship(
                        TfToken("rigExec:driverCurve"))) {
                    rel.GetTargets(&curves);
                }
                if (UsdRelationship rel = prim.GetRelationship(
                        TfToken("rigExec:bindCoordinates"))) {
                    rel.GetTargets(&binds);
                }
                if (curves.empty() || binds.empty()) {
                    diagnostics->push_back(
                        "MoverFailed " + mover->moverPath.GetString() +
                        ": wire needs rigExec:driverCurve and "
                        "rigExec:bindCoordinates");
                    continue;
                }
                const SdfPath curvePrim = curves[0].GetPrimPath();
                VtVec3fArray posedCvs, restCvs;
                SdfPathVector driverTransforms, driverSpaces;
                if (UsdRelationship rel = prim.GetRelationship(
                        TfToken("rigExec:driverTransforms"))) {
                    rel.GetTargets(&driverTransforms);
                }
                if (UsdRelationship rel = prim.GetRelationship(
                        TfToken("rigExec:driverTransformSpaces"))) {
                    rel.GetTargets(&driverSpaces);
                }
                if (driverTransforms.empty()) {
                    phasedPoints(prim, "rigExec:driverCurve",
                                 "rigExec:driverCurveReadPhase",
                                 curvePrim.AppendProperty(TfToken("points")),
                                 mover->moverPath, &posedCvs);
                }
                VtIntArray order;
                VtDoubleArray knots;
                VtVec2fArray sts;
                float dropoff = 0.0f;
                if (const UsdPrim curve = _stage->GetPrimAtPath(curvePrim)) {
                    curve.GetAttribute(TfToken("points"))
                        .Get(&restCvs, UsdTimeCode::Default());
                    curve.GetAttribute(TfToken("order"))
                        .Get(&order, UsdTimeCode::Default());
                    curve.GetAttribute(TfToken("knots"))
                        .Get(&knots, UsdTimeCode::Default());
                }
                if (UsdAttribute a = _stage->GetAttributeAtPath(binds[0])) {
                    a.Get(&sts, time);
                }
                if (UsdAttribute a =
                        prim.GetAttribute(TfToken("inputs:dropoffDistance"))) {
                    a.Get(&dropoff, time);
                }
                if (!driverTransforms.empty()) {
                    // Independently of the assembler: the providers' own
                    // matrices, measured and weighted per control point.
                    TfToken phase("base");
                    if (const UsdAttribute a = prim.GetAttribute(
                            TfToken("rigExec:transformReadPhase"))) {
                        a.Get(&phase);
                    }
                    const auto &matrices = phase == "final"
                                               ? finalProviderMatrices
                                               : baseProviderMatrices;
                    VtFloatArray weights, baseWeights;
                    if (const UsdAttribute a = prim.GetAttribute(
                            TfToken("inputs:driverWeights"))) {
                        a.Get(&weights, time);
                    }
                    if (const UsdAttribute a = prim.GetAttribute(
                            TfToken("inputs:driverBaseWeights"))) {
                        a.Get(&baseWeights, time);
                    }
                    SdfPathVector baseTransforms, baseSpaces;
                    if (UsdRelationship rel = prim.GetRelationship(
                            TfToken("rigExec:driverBaseTransforms"))) {
                        rel.GetTargets(&baseTransforms);
                    }
                    if (UsdRelationship rel = prim.GetRelationship(
                            TfToken("rigExec:driverBaseTransformSpaces"))) {
                        rel.GetTargets(&baseSpaces);
                    }
                    const auto pick = [](size_t count, size_t j) {
                        return count <= 1 ? size_t(0) : j % count;
                    };
                    bool missing = false;
                    const auto measured = [&](const SdfPathVector &ts,
                                              const SdfPathVector &ss,
                                              size_t j) {
                        GfMatrix4d m(1.0);
                        const auto t = matrices.find(ts[pick(ts.size(), j)]);
                        if (t == matrices.end()) {
                            missing = true;
                            return m;
                        }
                        m = t->second;
                        if (!ss.empty()) {
                            const auto sp =
                                matrices.find(ss[pick(ss.size(), j)]);
                            if (sp == matrices.end()) {
                                missing = true;
                                return m;
                            }
                            m = RigExecMeasureInSpace(m, sp->second);
                        }
                        return m;
                    };
                    posedCvs = restCvs;
                    for (size_t j = 0; j < restCvs.size() && !missing; ++j) {
                        if (!baseTransforms.empty()) {
                            const GfMatrix4d b =
                                measured(baseTransforms, baseSpaces, j);
                            const float wb = baseWeights.empty()
                                ? 1.0f
                                : baseWeights[pick(baseWeights.size(), j)];
                            const GfVec3f moved(
                                b.TransformAffine(GfVec3d(restCvs[j])));
                            restCvs[j] = restCvs[j] + (moved - restCvs[j]) * wb;
                        }
                        const GfMatrix4d m =
                            measured(driverTransforms, driverSpaces, j);
                        const float w = weights.empty()
                            ? 1.0f : weights[pick(weights.size(), j)];
                        const GfVec3f moved(
                            m.TransformAffine(GfVec3d(restCvs[j])));
                        posedCvs[j] = restCvs[j] + (moved - restCvs[j]) * w;
                    }
                    if (missing) {
                        diagnostics->push_back(
                            "MoverFailed " + mover->moverPath.GetString() +
                            ": no matrix for a wire driver transform");
                        continue;
                    }
                }
                const std::vector<GfVec3f> rest(restCvs.begin(), restCvs.end());
                const std::vector<GfVec3f> posed(posedCvs.begin(),
                                                 posedCvs.end());
                const std::vector<double> knotVec(knots.begin(), knots.end());
                const int curveOrder = order.empty() ? 0 : order[0];
                std::vector<GfVec3f> scratch(points.begin(), points.end());
                // A sparse bind table is parallel to the weight object's
                // indices; spread over the whole mesh here, where the
                // envelope below zeroes every point it does not name.
                std::vector<GfVec2f> bindAll(sts.begin(), sts.end());
                if (sts.size() != scratch.size()) {
                    SdfPathVector weightTargets;
                    if (UsdRelationship rel = prim.GetRelationship(
                            TfToken("rigExec:weightObject"))) {
                        rel.GetTargets(&weightTargets);
                    }
                    VtIntArray indices;
                    if (!weightTargets.empty()) {
                        if (const UsdPrim w =
                                _stage->GetPrimAtPath(weightTargets[0])) {
                            w.GetAttribute(TfToken("rigExec:indices"))
                                .Get(&indices);
                        }
                    }
                    if (indices.size() == sts.size()) {
                        bindAll.assign(scratch.size(), GfVec2f(0.0f));
                        for (size_t k = 0; k < indices.size(); ++k) {
                            if (indices[k] >= 0 &&
                                size_t(indices[k]) < bindAll.size()) {
                                bindAll[size_t(indices[k])] = sts[k];
                            }
                        }
                    }
                }
                if (!RigExecApplyWire(
                        &scratch, RigExecNurbsCurve{&rest, curveOrder, &knotVec},
                        RigExecNurbsCurve{&posed, curveOrder, &knotVec},
                        bindAll.data(), bindAll.size(), dropoff,
                        0, scratch.size())) {
                    diagnostics->push_back(
                        "MoverFailed " + mover->moverPath.GetString() +
                        ": wire driver curve or bind coordinates do not "
                        "match the deformed points");
                    continue;
                }
                std::copy(scratch.begin(), scratch.end(), points.begin());
                goto envelope;
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
        } else if (type == "RigExecCurvenetAdjusterMover") {
            const auto p = RigExecAssembleCurvenetAdjusterParameters(
                prim, target, nullptr, time, &_resolvedInputs);
            std::vector<GfVec3f> scratch(points.begin(), points.end());
            std::string error;
            if (!p.valid || !RigExecApplyCurvenetAdjustments(&scratch,
                    p.restPoints, p.topologyIndices, p.curvenetAdjustmentBasis,
                    p.curvenetAdjustments, nullptr, &error)) {
                if (diagnostics) diagnostics->push_back(
                    "MoverFailed " + mover->moverPath.GetString() + ": " + error);
                continue;
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

    envelope:
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

// ---------------------------------------------------------------------------
// What a property chain re-reads every frame, bound once.
//
// The chains are the prologue, on BOTH paths: [P22] makes the baked program
// call this very routine so the two agree line for line, so what is spent
// here is spent twice over. Measured at 119-128us of a 662us biped frame,
// and it is USD value resolution rather than arithmetic -- about 180 reads
// at ~0.43us each, of which 0.38us is the resolution itself.
//
// A UsdAttributeQuery is the answer USD already has for that: it holds the
// resolved value source, so a read at a new time skips the composition
// lookup and goes straight to the layer. Nothing else about the routine
// moves -- every diag() line, in the same order, off the same values -- so
// the dynamic dumps are the proof that this changed no answer.
//
// The structural lookups come with it, because they are the same kind of
// thing: the target attribute, its value type, the mover prim and the
// weight-object relationship's targets are all things only a stage edit can
// move, and a stage edit drops this whole cache.
// ---------------------------------------------------------------------------

struct RigExecPropertyChainBindings
{
    /// One input the revision loop reads by value.
    struct Input {
        UsdAttribute attribute;
        UsdAttributeQuery query;
        SdfPath path;
        /// Authored connections make the read a WALK over the connection
        /// chain, which only RigExecResolvedInputs::GetAttribute knows how
        /// to perform; such an input is handed back to it unchanged.
        bool connected = false;
        /// Whether the attribute cannot change until the stage does -- no
        /// authored connections, no time samples, no time-varying opinion --
        /// which is the admission test RigExecStaticInputCache applies, and
        /// then `constantValue` is what it read once. Exactly as safe as
        /// that cache and for the same reasons: a standing property override
        /// is consulted FIRST and outranks it, and any stage edit drops the
        /// whole binding.
        bool constant = false;
        VtValue constantValue;
        explicit operator bool() const { return bool(attribute); }
    };

    /// One revision of one chain, in the chain's own order.
    struct Revision {
        UsdPrim moverPrim;
        SdfPathVector weightObjects;
        Input enabled;
        Input defaultWeight;
        Input operation;
        Input value;
        Input minimum;
        Input maximum;
        Input keys;
        Input tangents;
    };

    /// One target, in _propertyChainOrder's order. An entry whose target
    /// attribute did not resolve carries an invalid `target`, which is the
    /// same thing the per-frame lookup used to report.
    struct Chain {
        SdfPath targetPath;
        UsdAttribute target;
        UsdAttributeQuery targetQuery;
        SdfValueTypeName valueType;
        std::vector<Revision> revisions;

        // What the chain's answer can depend on, so a frame in which none of
        // it moved republishes the last answer instead of recomputing it.
        //
        //  * `watch`: every input attribute, and every attribute along an
        //    input's connection chain -- where an interactive override can
        //    stand;
        //  * `upstream`: the chains whose targets are among those
        //    attributes;
        //  * `varying`: whether any of them, or the target's own authored
        //    base, can change with time;
        //  * `alwaysDirty`: a weight object is read through its own
        //    resolution, which this does not follow.
        std::vector<SdfPath> watch;
        std::vector<size_t> upstream;
        bool varying = false;
        bool alwaysDirty = false;

        // The last run: whether it published, what, and the diagnostics it
        // pushed, all replayed verbatim when the chain is clean.
        bool cached = false;
        bool published = false;
        VtValue lastValue;
        std::vector<std::string> lastDiagnostics;
        bool changedThisRun = false;
    };

    std::vector<Chain> chains;

    // The interactive overrides and the time the last run saw.
    bool haveLast = false;
    UsdTimeCode lastTime;
    std::unordered_map<SdfPath, VtValue, SdfPath::Hash> lastOverrides;
};

namespace {

/// Binds one input of \p prim, or leaves the binding empty when there is
/// none -- which is what `prim.GetAttribute(...)` returning invalid used to
/// mean at the call site.
RigExecPropertyChainBindings::Input
_BindInput(const UsdPrim &prim, const char *name, UsdTimeCode time)
{
    RigExecPropertyChainBindings::Input input;
    if (!prim) {
        return input;
    }
    if (const UsdAttribute a = prim.GetAttribute(TfToken(name))) {
        input.attribute = a;
        input.path = a.GetPath();
        input.connected = a.HasAuthoredConnections();
        input.query = UsdAttributeQuery(a);
        input.constant = !input.connected && !a.ValueMightBeTimeVarying() &&
                         a.GetNumTimeSamples() == 0;
        if (input.constant) {
            // At THIS time, which is what the static cache fills an entry
            // with on its first read. The attribute does not vary, so the
            // time chooses nothing; saying which one was used is what makes
            // that claim checkable.
            a.Get(&input.constantValue, time);
        }
    }
    return input;
}

/// _ResolvedRead through a pinned query.
///
/// Equivalent to it by construction, arm for arm:
///
///  * no attribute -> the fallback, as `prim.GetAttribute()` returning
///    invalid gives;
///  * a connected attribute -> handed to RigExecResolvedInputs::GetAttribute,
///    which is the only code that follows a connection chain;
///  * an in-memory value of the right type for this exact property -> that
///    value, which is the `Get(a.GetPath(), out)` at the head of the walk
///    (and a value of the WRONG type falls through to the stage there too);
///  * otherwise the attribute's own value, which is what the tail of the
///    walk reads and what the query resolves.
///
/// The static-input cache is not consulted on this path. It only ever
/// answers for an attribute with no connections, no time samples and no
/// time-varying opinion, so the value it would hand back is the value the
/// query resolves; and its hit/refusal counters reach no pose.
template <class T>
T
_PinnedRead(const RigExecResolvedInputs &resolved,
            const RigExecPropertyChainBindings::Input &input, T fallback,
            UsdTimeCode time)
{
    T value = fallback;
    if (!input.attribute) {
        return value;
    }
    if (input.connected) {
        resolved.GetAttribute(input.attribute, time, &value);
        return value;
    }
    if (const VtValue *const standing = resolved.Find(input.path)) {
        if (standing->IsHolding<T>()) {
            return standing->UncheckedGet<T>();
        }
    }
    if (input.constant && input.constantValue.IsHolding<T>()) {
        return input.constantValue.UncheckedGet<T>();
    }
    T resolvedValue;
    if (input.query.Get(&resolvedValue, time)) {
        value = resolvedValue;
    }
    return value;
}

/// rigExec:operation as `a.Get(&operation)` read it: at Default, off the
/// stage, with no resolved input consulted -- the operation names the
/// arithmetic and not a value.
void
_ReadOperation(const RigExecPropertyChainBindings::Input &input,
               TfToken *operation)
{
    if (input.constant) {
        if (input.constantValue.IsHolding<TfToken>()) {
            *operation = input.constantValue.UncheckedGet<TfToken>();
        }
        return;
    }
    input.query.Get(operation);
}

// Reads the authored inputs of one float/vec3f math mover at \p time.
//
// Every field is read even though the operation uses only some of them: the
// packet is the mover's whole authored state, and branching on the operation
// while reading would put the same switch in two places.
//
// rigExec:operation is read at Default with no resolved inputs consulted,
// which is what the unpinned form did: the operation names the arithmetic,
// not a value, and a chain whose arithmetic an override could change is not
// a chain this routine is allowed to be wrong about quietly.
template <class T>
bool
_ReadPinnedPropertyMathParams(
    const RigExecResolvedInputs &resolved,
    const RigExecPropertyChainBindings::Revision &bound, UsdTimeCode time,
    RigExecPropertyMathParams<T> *params)
{
    TfToken operation;
    if (bound.operation) {
        _ReadOperation(bound.operation, &operation);
    }
    if (!RigExecParsePropertyOp(operation, &params->op)) {
        return false;
    }
    params->value =
        _PinnedRead(resolved, bound.value, params->value, time);
    params->min = _PinnedRead(resolved, bound.minimum, params->min, time);
    params->max = _PinnedRead(resolved, bound.maximum, params->max, time);
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

    // The first frame of an epoch pays for the bindings; every frame after it
    // reads through them. Built here rather than in Compile because the
    // chains are also rebuilt by a notice that recompiles nothing, and
    // "whatever the stage now says" is the only state this has to describe.
    if (!_propertyChainBindings) {
        _propertyChainBindings =
            std::make_unique<RigExecPropertyChainBindings>();
        for (const SdfPath &orderedTarget : _propertyChainOrder) {
            const auto chainIt = _propertyChains.find(orderedTarget);
            if (chainIt == _propertyChains.end()) {
                continue;
            }
            RigExecPropertyChainBindings::Chain bound;
            bound.targetPath = chainIt->first;
            bound.target = _stage->GetAttributeAtPath(bound.targetPath);
            if (bound.target) {
                bound.targetQuery = UsdAttributeQuery(bound.target);
                bound.valueType = bound.target.GetTypeName();
            }
            for (const _PropertyRevision &revision : chainIt->second) {
                RigExecPropertyChainBindings::Revision boundRevision;
                boundRevision.moverPrim =
                    _stage->GetPrimAtPath(revision.moverPath);
                const UsdPrim &mover = boundRevision.moverPrim;
                if (mover) {
                    if (const UsdRelationship rel = mover.GetRelationship(
                            TfToken("rigExec:weightObject"))) {
                        rel.GetTargets(&boundRevision.weightObjects);
                    }
                }
                boundRevision.enabled =
                    _BindInput(mover, "inputs:enabled", time);
                boundRevision.defaultWeight =
                    _BindInput(mover, "inputs:defaultWeight", time);
                boundRevision.operation =
                    _BindInput(mover, "rigExec:operation", time);
                boundRevision.value = _BindInput(mover, "inputs:value", time);
                boundRevision.minimum = _BindInput(mover, "inputs:min", time);
                boundRevision.maximum = _BindInput(mover, "inputs:max", time);
                boundRevision.keys = _BindInput(mover, "inputs:keys", time);
                boundRevision.tangents =
                    _BindInput(mover, "inputs:tangents", time);
                bound.alwaysDirty =
                    bound.alwaysDirty || !boundRevision.weightObjects.empty();
                for (const RigExecPropertyChainBindings::Input *input :
                         {&boundRevision.enabled,
                          &boundRevision.defaultWeight,
                          &boundRevision.operation, &boundRevision.value,
                          &boundRevision.minimum, &boundRevision.maximum,
                          &boundRevision.keys, &boundRevision.tangents}) {
                    if (!input->attribute) {
                        continue;
                    }
                    bound.watch.push_back(input->path);
                    if (input->constant) {
                        continue;
                    }
                    if (!input->connected) {
                        bound.varying = true;  // animated in place
                        continue;
                    }
                    // Follow the connection chain the read will walk.
                    UsdAttribute a = input->attribute;
                    std::set<SdfPath> seen;
                    while (a && seen.insert(a.GetPath()).second) {
                        bound.watch.push_back(a.GetPath());
                        if (a.ValueMightBeTimeVarying() ||
                            a.GetNumTimeSamples() > 0) {
                            bound.varying = true;
                        }
                        SdfPathVector sources;
                        a.GetConnections(&sources);
                        if (sources.size() != 1) {
                            break;
                        }
                        a = _stage->GetAttributeAtPath(sources[0]);
                    }
                }
                bound.revisions.push_back(std::move(boundRevision));
            }
            if (bound.target && (bound.target.ValueMightBeTimeVarying() ||
                                 bound.target.GetNumTimeSamples() > 0)) {
                bound.varying = true;
            }
            _propertyChainBindings->chains.push_back(std::move(bound));
        }
        std::unordered_map<SdfPath, size_t, SdfPath::Hash> chainOf;
        auto &chains = _propertyChainBindings->chains;
        for (size_t i = 0; i < chains.size(); ++i) {
            chainOf[chains[i].targetPath] = i;
        }
        for (size_t i = 0; i < chains.size(); ++i) {
            for (const SdfPath &path : chains[i].watch) {
                const auto it = chainOf.find(path);
                if (it != chainOf.end() && it->second != i) {
                    chains[i].upstream.push_back(it->second);
                }
            }
        }
    }

    // What moved since the last run: the time, and each interactive override
    // placed, lifted or changed in value.
    RigExecPropertyChainBindings &bindings = *_propertyChainBindings;
    const bool timeMoved = !bindings.haveLast || time != bindings.lastTime;
    std::unordered_map<SdfPath, VtValue, SdfPath::Hash> nowOverrides;
    for (const RigExecValueOverride &o : _interactiveOverrides) {
        if (!o.attribute.IsEmpty()) {
            nowOverrides[o.prim.AppendProperty(o.attribute)] = o.value;
        }
    }
    std::unordered_set<SdfPath, SdfPath::Hash> overrideMoved;
    for (const auto &[path, value] : nowOverrides) {
        const auto it = bindings.lastOverrides.find(path);
        if (it == bindings.lastOverrides.end() || it->second != value) {
            overrideMoved.insert(path);
        }
    }
    for (const auto &[path, value] : bindings.lastOverrides) {
        if (!nowOverrides.count(path)) {
            overrideMoved.insert(path);
        }
    }
    bindings.haveLast = true;
    bindings.lastTime = time;
    bindings.lastOverrides = std::move(nowOverrides);

    for (RigExecPropertyChainBindings::Chain &chain :
             _propertyChainBindings->chains) {
        const SdfPath &target = chain.targetPath;
        const std::vector<RigExecPropertyChainBindings::Revision> &revisions =
            chain.revisions;
        if (!chain.target) {
            diag("property chain " + target.GetString() +
                 ": target attribute disappeared; chain skipped");
            continue;
        }
        // Clean: nothing the chain reads moved, so the last answer stands.
        // Published and reported exactly as a run would publish and report
        // it, so no consumer can tell the difference.
        bool dirty = !chain.cached || chain.alwaysDirty ||
                     (chain.varying && timeMoved);
        for (size_t k = 0; !dirty && k < chain.watch.size(); ++k) {
            dirty = overrideMoved.count(chain.watch[k]) != 0;
        }
        for (size_t k = 0; !dirty && k < chain.upstream.size(); ++k) {
            dirty = bindings.chains[chain.upstream[k]].changedThisRun;
        }
        if (!dirty) {
            chain.changedThisRun = false;
            for (const std::string &line : chain.lastDiagnostics) {
                diag(line);
            }
            if (chain.published) {
                if (results) {
                    (*results)[target] = chain.lastValue;
                }
                _resolvedInputs.SetProperty(target, chain.lastValue);
                if (overrides) {
                    overrides->push_back(RigExecValueOverride{
                        target.GetPrimPath(), TfToken(),
                        target.GetNameToken(), chain.lastValue});
                }
            }
            continue;
        }
        const size_t diagnosticsBefore =
            diagnostics ? diagnostics->size() : 0;
        _resolvedInputs.ClearProperty(target);
        struct _Remember {
            RigExecPropertyChainBindings::Chain &chain;
            RigExecResolvedInputs &resolved;
            std::vector<std::string> *diagnostics;
            size_t before;
            ~_Remember() {
                const VtValue *now = resolved.Find(chain.targetPath);
                const bool published = now != nullptr;
                chain.changedThisRun =
                    !chain.cached || published != chain.published ||
                    (published && *now != chain.lastValue);
                chain.published = published;
                chain.lastValue = published ? *now : VtValue();
                chain.lastDiagnostics.clear();
                if (diagnostics) {
                    chain.lastDiagnostics.assign(
                        diagnostics->begin() + long(before),
                        diagnostics->end());
                }
                chain.cached = true;
            }
        } remember{chain, _resolvedInputs, diagnostics, diagnosticsBefore};
        RIGEXEC_PROFILE_SCOPE_CAT(
            _profiler, "PropertyChain " + target.GetString(), "property");
        const SdfValueTypeName &valueType = chain.valueType;

        // One shared revision loop over the three value domains. Each
        // iteration reads the mover's own authored state and applies it to
        // the preceding revision -- the base being the target's AUTHORED
        // value, exactly as a point chain's base is the target's authored
        // points.
        auto runChain = [&](auto value, auto apply) {
            using ValueT = decltype(value);
            if (!chain.targetQuery.Get(&value, time)) {
                diag("property chain " + target.GetString() +
                     ": target has no authored value; chain skipped");
                return false;
            }
            if (!_IsFinite(value)) {
                diag("property chain " + target.GetString() +
                     ": authored base is not finite; chain skipped");
                return false;
            }
            for (const RigExecPropertyChainBindings::Revision &revision :
                     revisions) {
                const UsdPrim &moverPrim = revision.moverPrim;
                if (!moverPrim) {
                    continue;
                }
                const SdfPath moverPath = moverPrim.GetPath();
                const bool enabled = _PinnedRead(
                    _resolvedInputs, revision.enabled, true, time);
                if (!enabled) {
                    diag("diag " + moverPath.GetString() +
                         ": disabled; revision passed through");
                    continue;  // ordinary pass-through (spec §6.6)
                }
                float envelope = 1.0f;
                const SdfPathVector &weightObjects = revision.weightObjects;
                if (!weightObjects.empty()) {
                    std::vector<float> weights;
                    std::string error;
                    if (!_ResolveWeights(weightObjects[0], 1, time,
                                         &weights, &error) ||
                        weights.size() != 1) {
                        diag("diag " + moverPath.GetString() +
                             ": " + error + "; revision passed through");
                        continue;
                    }
                    envelope = weights[0];
                } else {
                    envelope = _PinnedRead(
                        _resolvedInputs, revision.defaultWeight, 1.0f, time);
                    if (!std::isfinite(envelope) || envelope < 0.0f ||
                        envelope > 1.0f) {
                        diag("diag " + moverPath.GetString() +
                             ": inputs:defaultWeight must be finite and in "
                             "[0, 1]; revision passed through");
                        continue;
                    }
                }
                ValueT next = value;
                if (!apply(revision, value, envelope, &next)) {
                    diag("diag " + moverPath.GetString() +
                         ": inputs unusable; revision passed through");
                    continue;
                }
                if (!_IsFinite(next)) {
                    // A NaN reaching an exec override propagates into every
                    // consumer of the attribute with no way to report it
                    // back, so the mover fails and passes through instead
                    // (spec §6.6).
                    diag("diag " + moverPath.GetString() +
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

        using BoundRevision = RigExecPropertyChainBindings::Revision;
        const auto applyFloat = [&](const BoundRevision &mover, float in,
                                    float envelope, float *out) {
                RigExecPropertyMathParams<float> params;
                if (!_ReadPinnedPropertyMathParams(
                        _resolvedInputs, mover, time, &params) ||
                    !_IsFinite(params.value) || !_IsFinite(params.min) ||
                    !_IsFinite(params.max)) {
                    return false;
                }
                // Held here so the borrowed key pointer outlives the apply.
                VtArray<GfVec2f> keys;
                VtArray<GfVec2f> tangents;
                if (params.op == RigExecPropertyOp::Curve) {
                    keys = _PinnedRead(
                        _resolvedInputs, mover.keys, keys, time);
                    if (keys.empty() || !RigExecValidateLinearKeys(
                                            keys.cdata(), keys.size())) {
                        return false;
                    }
                    params.keys = keys.cdata();
                    params.keyCount = keys.size();
                    if (mover.tangents) {
                        tangents = _PinnedRead(
                            _resolvedInputs, mover.tangents, tangents, time);
                    }
                    if (!tangents.empty()) {
                        if (tangents.size() != keys.size()) {
                            return false;
                        }
                        params.tangents = tangents.cdata();
                        params.tangentCount = tangents.size();
                    }
                }
                params.weight = envelope;
                *out = RigExecApplyFloatMath(in, params);
                return true;
        };
        if (valueType == SdfValueTypeNames->Float) {
            runChain(float(0), applyFloat);
        } else if (valueType == SdfValueTypeNames->Double) {
            runChain(double(0), [&](const BoundRevision &mover, double in,
                                    float envelope, double *out) {
                float result = 0.0f;
                if (!applyFloat(mover, float(in), envelope, &result)) {
                    return false;
                }
                *out = double(result);
                return true;
            });
        } else if (valueType == SdfValueTypeNames->Matrix4d) {
            runChain(GfMatrix4d(1.0), [&](const BoundRevision &mover,
                                          const GfMatrix4d &in,
                                          float envelope,
                                          GfMatrix4d *out) {
                TfToken operation;
                if (mover.operation) {
                    _ReadOperation(mover.operation, &operation);
                }
                RigExecPropertyOp op;
                if (!RigExecParsePropertyOp(operation, &op)) {
                    return false;
                }
                const GfMatrix4d opValue = _PinnedRead(
                    _resolvedInputs, mover.value, GfMatrix4d(1.0), time);
                if (!_IsFinite(opValue)) {
                    return false;
                }
                return RigExecApplyMatrixMath(
                    in, op, opValue, envelope, out);
            });
        } else {
            // Every remaining type the compiler admits is GfVec3f-backed.
            runChain(GfVec3f(0), [&](const BoundRevision &mover,
                                     const GfVec3f &in, float envelope,
                                     GfVec3f *out) {
                RigExecPropertyMathParams<GfVec3f> params;
                if (!_ReadPinnedPropertyMathParams(
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

// Whether the epoch's rest frames may no longer be epoch constants.
//
// Asked once per notice that did not recompile, never per frame: the answer
// can only change when the stage does, and a notice is the only way it does.
bool
RigExecRigEvaluator::_EpochRestsMightVary() const
{
    std::set<SdfPath> chainTargets;
    for (const auto &[target, revisions] : _propertyChains) {
        chainTargets.insert(target);
    }
    for (const auto &[provider, frame] : _epochRestFrames) {
        if (_ProviderRestMightVary(_stage, provider, chainTargets)) {
            return true;
        }
    }
    return false;
}

bool
RigExecRigEvaluator::_RefreshEpochRestFrames()
{
    if (!_restTaps || _restTapIds.empty()) {
        return true;
    }
    RIGEXEC_PROFILE_SCOPE_CAT(_profiler, "RestFramesRefresh", "evaluate");
    const RigExecSnapshot rests = _restTaps->Evaluate(_restTime);
    if (!rests.IsValid() || !rests.IsComplete()) {
        return false;
    }
    for (const auto &[provider, tap] : _restTapIds) {
        _epochRestFrames[provider] = rests.Get<RigExecPointFrame>(tap);
    }
    return true;
}

bool
RigExecRigEvaluator::_ComposeInterveningXforms(
    const UsdPrim &assetRoot,
    UsdGeomXformCache *xformCache,
    std::vector<RigExecPointFrame> *restFrames,
    std::vector<char> *restLive,
    std::map<SdfPath, RigExecPointFrame> *baseFrames,
    std::vector<RigExecPointFrame> *finalFrames,
    std::vector<char> *finalLive,
    RigExecRigPose *pose) const
{
    if (!assetRoot || !xformCache) {
        return true;
    }
    // No provider has anything standing between it and its anchor, so there
    // is no transform to compose and nothing to ask the xform cache.
    if (_interveningXformProviders.empty()) {
        return true;
    }

    // X(P) per provider, and which provider (if any) anchors it. The anchor
    // is the nearest RigExec ancestor -- exec has already folded that one's
    // rest:space and avars in -- and the asset root otherwise. Only a
    // provider with an intervening prim can have a non-identity X(P); the
    // rest are anchored to their own parent.
    std::map<SdfPath, GfMatrix4d> intervening;
    const std::map<SdfPath, SdfPath> &anchorOf = _poseProviderAnchors;
    bool anyIntervening = false;
    for (const auto &[provider, tap] : _firstFramePoseFrames) {
        intervening[provider] = GfMatrix4d(1.0);
    }
    for (const SdfPath &provider : _interveningXformProviders) {
        const SdfPath &anchorPath = anchorOf.at(provider);
        const UsdPrim anchor = anchorPath.IsEmpty()
            ? assetRoot : _stage->GetPrimAtPath(anchorPath);
        const UsdPrim parent = _stage->GetPrimAtPath(provider.GetParentPath());
        GfMatrix4d x(1.0);
        if (parent && anchor && parent != anchor) {
            bool resetsBelowAnchor = false;
            x = xformCache->ComputeRelativeTransform(
                parent, anchor, &resetsBelowAnchor);
            if (resetsBelowAnchor) {
                // !resetXformStack! detaches the provider from the anchor
                // entirely, so "relative to the anchor" is not a quantity
                // that exists. Reported rather than composed: guessing here
                // would place the provider somewhere nobody asked for.
                pose->diagnostics.push_back(
                    "resetXformStack between " + anchor.GetPath().GetString() +
                    " and " + provider.GetString() +
                    "; the intervening transform is not composed");
                x = GfMatrix4d(1.0);
            }
        }
        intervening[provider] = x;
        anyIntervening = anyIntervening || x != GfMatrix4d(1.0);
    }
    // The overwhelmingly common rig has no such Xform anywhere, and must not
    // pay a frame rebuild for the ones that do.
    if (!anyIntervening) {
        return true;
    }

    // Parents before children, so an anchor is already corrected when the
    // providers under it are reached. Ordered by path element COUNT rather
    // than by SdfPath's own ordering, which makes no parent-first promise.
    std::vector<SdfPath> ordered;
    ordered.reserve(intervening.size());
    for (const auto &[provider, x] : intervening) {
        ordered.push_back(provider);
    }
    std::stable_sort(ordered.begin(), ordered.end(),
                     [](const SdfPath &a, const SdfPath &b) {
                         return a.GetPathElementCount() <
                                b.GetPathElementCount();
                     });

    // The uncorrected matrices, captured before anything is overwritten. The
    // correction divides a provider by its anchor's OLD value to recover its
    // own local factor, so reading the anchor after correcting it would
    // divide by the answer instead of by the question.
    auto toMatrix = [](const RigExecPointFrame &frame, GfMatrix4d *out) {
        if (!frame.IsValid() || frame.IsDegenerate()) {
            return false;
        }
        return RigExecPointsToMatrix(
            RigExecIdentityLandmarks(), frame.points, out);
    };
    std::map<SdfPath, GfMatrix4d> execRest, execBase;
    for (const SdfPath &provider : ordered) {
        GfMatrix4d m(1.0);
        {
            const auto ri = _providerIndex.find(provider);
            if (ri != _providerIndex.end() && (*restLive)[ri->second] &&
                toMatrix((*restFrames)[ri->second], &m))
                execRest[provider] = m;
        }
        m = GfMatrix4d(1.0);
        if (toMatrix(baseFrames->at(provider), &m)) execBase[provider] = m;
    }

    auto correctDense = [&](std::vector<RigExecPointFrame> *frames,
                            std::vector<char> *live,
                            const std::map<SdfPath, GfMatrix4d> &exec,
                            const SdfPath &provider) {
        const auto own = exec.find(provider);
        if (own == exec.end()) {
            return;
        }
        const int fi = _providerIndex.at(provider);
        const SdfPath &anchorPath = anchorOf.at(provider);
        GfMatrix4d anchorExec(1.0), anchorTrue(1.0);
        if (!anchorPath.IsEmpty()) {
            const auto execIt = exec.find(anchorPath);
            if (execIt != exec.end()) {
                anchorExec = execIt->second;
            }
            const auto ai = _providerIndex.find(anchorPath);
            GfMatrix4d corrected(1.0);
            if (ai != _providerIndex.end() && (*live)[ai->second] &&
                toMatrix((*frames)[ai->second], &corrected)) {
                anchorTrue = corrected;
            }
        }
        (*frames)[fi] = RigExecFrameFromMatrix(
            own->second * anchorExec.GetInverse() *
            intervening.at(provider) * anchorTrue);
        (*live)[fi] = 1;
    };
    auto correct = [&](std::map<SdfPath, RigExecPointFrame> *frames,
                       const std::map<SdfPath, GfMatrix4d> &exec,
                       const SdfPath &provider) {
        const auto own = exec.find(provider);
        if (own == exec.end()) {
            return;
        }
        const SdfPath &anchorPath = anchorOf.at(provider);
        GfMatrix4d anchorExec(1.0), anchorTrue(1.0);
        if (!anchorPath.IsEmpty()) {
            const auto execIt = exec.find(anchorPath);
            if (execIt != exec.end()) {
                anchorExec = execIt->second;
            }
            GfMatrix4d corrected(1.0);
            if (toMatrix(frames->at(anchorPath), &corrected)) {
                anchorTrue = corrected;
            }
        }
        (*frames)[provider] = RigExecFrameFromMatrix(
            own->second * anchorExec.GetInverse() *
            intervening.at(provider) * anchorTrue);
    };

    for (const SdfPath &provider : ordered) {
        correctDense(restFrames, restLive, execRest, provider);
        correct(baseFrames, execBase, provider);
        // base and final are the same frame at seeding time; final is
        // reassigned rather than corrected again so the two cannot drift.
        const int fi = _providerIndex.at(provider);
        (*finalFrames)[fi] = baseFrames->at(provider);
        (*finalLive)[fi] = 1;
    }
    return true;
}

void
RigExecRigEvaluator::_ResolveSkinLayoutInputs() const
{
    _skinLayoutInputs.clear();
    _skinLayoutInputsValid = true;
    // The four the layout is assembled from: the two arrays and the element
    // size RigExecResolveSkinTopology reads, and the method
    // RigExecAssembleSkinParameters reads beside them. inputs:enabled and
    // inputs:defaultWeight are deliberately NOT here -- they are read per
    // frame, never cached, so an override on one reaches the next generation
    // without anything being dropped.
    static const TfToken layoutAttributes[] = {
        TfToken("rigExec:jointIndices"), TfToken("rigExec:jointWeights"),
        TfToken("rigExec:elementSize"), TfToken("rigExec:skinningMethod")};
    for (const RigExecMoverRecord &record : _movers) {
        if (record.schemaType != "RigExecSkinMover") {
            continue;
        }
        const UsdPrim prim = _stage->GetPrimAtPath(record.moverPath);
        if (!prim) {
            continue;
        }
        for (const TfToken &name : layoutAttributes) {
            // The SAME walk the value is read through
            // (RigExecResolvedInputs::GetAttribute): a single authored
            // connection per hop, the resolved map consulted at every hop,
            // cycles refused. Every path along it is a path an override can
            // stand on and be seen by the read, so every path along it
            // belongs in this set -- an override one hop upstream of a
            // connected rigExec:jointIndices is the case that makes the
            // difference between a re-read and a silently stale deformation.
            UsdAttribute attribute = prim.GetAttribute(name);
            if (!attribute) {
                // Not authored and not in the schema: an override could
                // still create the opinion the read would find, so the
                // property itself is named even where the attribute is not.
                _skinLayoutInputs.insert(
                    record.moverPath.AppendProperty(name));
                continue;
            }
            while (attribute &&
                   _skinLayoutInputs.insert(attribute.GetPath()).second) {
                SdfPathVector connections;
                if (attribute.HasAuthoredConnections()) {
                    attribute.GetConnections(&connections);
                }
                if (connections.size() != 1) {
                    break;
                }
                attribute = _stage->GetAttributeAtPath(connections[0]);
            }
        }
    }
}

bool
RigExecRigEvaluator::_OverridesReachSkinLayout(
    const std::vector<RigExecValueOverride> &overrides) const
{
    if (overrides.empty()) {
        return false;
    }
    if (!_skinLayoutInputsValid) {
        _ResolveSkinLayoutInputs();
    }
    for (const RigExecValueOverride &o : overrides) {
        // A computation override names no property, so there is nothing to
        // compare it against: it is taken to reach everything.
        if (o.attribute.IsEmpty()) {
            return true;
        }
        if (_skinLayoutInputs.count(o.prim.AppendProperty(o.attribute))) {
            return true;
        }
    }
    return false;
}

size_t
RigExecRigEvaluator::GetSkinTopologyCacheSize() const
{
    return _skinTopologies.GetSize();
}

void
RigExecRigEvaluator::SetInteractiveOverrides(
    std::vector<RigExecValueOverride> overrides)
{
    // Asked of BOTH sets before either is dropped: an override being lifted
    // off a layout attribute moves the value the layout was read with just
    // as much as one being placed on it.
    const bool touchesLayout =
        _OverridesReachSkinLayout(_interactiveOverrides) ||
        _OverridesReachSkinLayout(overrides);
    const bool touchesShapes =
        _OverridesReachBlendShapes(_interactiveOverrides) ||
        _OverridesReachBlendShapes(overrides);
    _interactiveOverrides = std::move(overrides);
    // The static-input cache is NOT dropped here. It used to be, as defence
    // in depth, on the reasoning that this "costs one map clear per drag
    // start" -- but a manipulator calls this on EVERY MOUSE SAMPLE, not once
    // per drag, so every drag frame began with a cold cache and re-read
    // every static input from the stage. Measured on the biped, a brow drag
    // spent 0.94 ms re-reading 161 blend-sample activations that are
    // authored constants, the largest single item in its frame.
    //
    // It was never a correctness requirement, and the three ways it could
    // matter are each closed by construction:
    //   * an overridden attribute is written into the resolved inputs before
    //     anything reads, and GetAttribute consults the resolved map FIRST,
    //     so an override never reaches the cache -- and is never put in it;
    //   * an override on a connection SOURCE cannot leave a stale reader,
    //     because the cache refuses any attribute with an authored
    //     connection and that reader goes the long way every time;
    //   * lifting an override is safe, because the cache only ever holds the
    //     authored value it read from the stage.
    // Authored edits still clear it: every notice does.
    // _propertyChainBindings is NOT dropped here, and the asymmetry with the
    // cache above is deliberate. It folds constants for the same class of
    // attribute, but _PinnedRead consults the resolved inputs FIRST -- and
    // _ApplyInteractiveOverridesToResolved writes every override into those
    // before a chain runs, on both paths -- so an overridden attribute can
    // never reach a folded constant to begin with. Dropping the bindings per
    // drag would rebind every chain input of the rig on both halves of every
    // drag, which is the whole of what binding once per compile bought. The
    // two invalidations it does have are the ones that make a binding WRONG
    // rather than outranked: a stage edit (_OnObjectsChanged) and the
    // recompile that replaces the chains the bindings describe.
    // An override is a value the static reads must prefer over the stage,
    // and the skin layout is read through exactly that route -- so a layout
    // resolved before the override set changed was resolved against a
    // different answer. But only for an override that can actually reach
    // one: dropping every layout costs ~400us of a biped drag frame (a
    // third of it) re-reading and re-comparing 105k elements that no
    // manipulator touched, and an animator's drag names a control avar.
    // The predicate follows the same connection walk the layout is read
    // through and answers yes wherever it is unsure -- see
    // _OverridesReachSkinLayout. The dynamic path shares this cache, so
    // both paths get the same answer either way: the cache hands back the
    // pointer it held for arrays that compare equal, so what is at stake is
    // the re-read and not the deformation.
    if (touchesLayout) {
        _skinTopologies.Clear();
    }
    // The same rule for the blend sample shapes, which are the other cache a
    // drag must not pay to rebuild: see _OverridesReachBlendShapes.
    if (touchesShapes) {
        _blendSampleShapes.Clear();
    }
    // Nothing to invalidate: the program is asked to PLACE these at the top
    // of every generation (Evaluate), and it runs only for a set it can place
    // exactly. An override it cannot place -- one standing on a value folded
    // into bake state, or a computation only exec can answer -- makes that
    // generation dynamic instead, which is the same answer more slowly.
}

void
RigExecRigEvaluator::ClearInteractiveOverrides()
{
    const bool touchesLayout =
        _OverridesReachSkinLayout(_interactiveOverrides);
    const bool touchesShapes =
        _OverridesReachBlendShapes(_interactiveOverrides);
    _interactiveOverrides.clear();
    // Both halves of a drag treat the caches the same way. The static-input
    // cache is kept on the way out for the same reasons it is kept on the
    // way in (see SetInteractiveOverrides): it never holds an override, so
    // there is nothing of the drag's in it to forget.
    if (touchesLayout) {
        _skinTopologies.Clear();
    }
    // The same rule for the blend sample shapes, which are the other cache a
    // drag must not pay to rebuild: see _OverridesReachBlendShapes.
    if (touchesShapes) {
        _blendSampleShapes.Clear();
    }
}

// Appends the interactive overrides to \p overrides, replacing any entry
// already standing on the same key, and mirrors the attribute ones into
// _resolvedInputs.
//
// Replacing rather than appending is not a tidiness preference: exec is given
// a vector of key/value pairs and which of two entries on one key wins is not
// a promise anything here should rely on. Removing the loser makes the answer
// a property of this function.
static void
_ApplyInteractiveOverrides(
    const std::vector<RigExecValueOverride> &interactive,
    std::vector<RigExecValueOverride> *overrides,
    RigExecResolvedInputs *resolved,
    std::map<SdfPath, VtValue> *publishedProperties = nullptr)
{
    for (const RigExecValueOverride &o : interactive) {
        if (overrides) {
            overrides->erase(
                std::remove_if(
                    overrides->begin(), overrides->end(),
                    [&o](const RigExecValueOverride &existing) {
                        return existing.prim == o.prim &&
                               existing.attribute == o.attribute &&
                               existing.computation == o.computation;
                    }),
                overrides->end());
            overrides->push_back(o);
        }
        // Only an ATTRIBUTE override has a property path to resolve; a
        // computation override names no property and the static readers never
        // look for one.
        if (resolved && !o.attribute.IsEmpty()) {
            resolved->SetProperty(o.prim.AppendProperty(o.attribute), o.value);
        }
        // A property a chain WRITES is also PUBLISHED, and the generation
        // Hydra draws has to carry the same value exec was given -- otherwise
        // the viewport shows the chain's arithmetic while every exec consumer
        // sees the held one, which is the disagreement between the two
        // delivery routes that this function exists to prevent.
        //
        // Only an entry that is already there is replaced. Inventing one would
        // publish an avar as a moved property of the generation, and an avar
        // is an input, not a result.
        if (publishedProperties && !o.attribute.IsEmpty()) {
            const auto it = publishedProperties->find(
                o.prim.AppendProperty(o.attribute));
            if (it != publishedProperties->end()) {
                it->second = o.value;
            }
        }
    }
}

void
RigExecRigEvaluator::_ApplyInteractiveOverridesToResolved(
    RigExecResolvedInputs *resolved,
    std::map<SdfPath, VtValue> *published) const
{
    _ApplyInteractiveOverrides(_interactiveOverrides, /* overrides = */
                               nullptr, resolved, published);
}

// One per-frame array of constraint source parameters, read RAW.
//
// Straight off the attribute at the frame's time: no connection walk, no
// resolved-input lookup, no interactive override. A source weight is an
// input of the constraint operator, not of the rig, and the evaluator and
// the program have to read it the same way -- so both read it here, and the
// cardinality diagnostic has one wording rather than one per caller.
//
// An absent or empty array is not a failure: it means the neutral value on
// every source, which is what an unauthored blend has always meant.
bool
RigExecRigEvaluator::_ReadConstraintSourceWeights(
    const UsdPrim &prim,
    const char *name,
    size_t count,
    UsdTimeCode time,
    std::vector<std::string> *diagnostics,
    std::vector<double> *weights)
{
    VtFloatArray authored;
    if (const UsdAttribute a = prim.GetAttribute(TfToken(name))) {
        a.Get(&authored, time);
    }
    if (!authored.empty() && authored.size() != count) {
        diagnostics->push_back(
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
}

// The same read for a per-source offset array, whose neutral value is zero.
bool
RigExecRigEvaluator::_ReadConstraintSourceOffsets(
    const UsdPrim &prim,
    const char *name,
    size_t count,
    UsdTimeCode time,
    std::vector<std::string> *diagnostics,
    std::vector<GfVec3d> *offsets)
{
    VtVec3dArray authored;
    if (const UsdAttribute a = prim.GetAttribute(TfToken(name))) {
        a.Get(&authored, time);
    }
    if (!authored.empty() && authored.size() != count) {
        diagnostics->push_back(
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
}

// `neverTS` retains current rotations/root placement while rebuilding
// child placement and handle lengths from rest frames. A joint without
// any authored rest transform has the schema's identity fallback, which
// is not an actual chain rest layout; use its current static layout in
// that case. The public math solver can therefore keep measuring its
// input chain; evaluator-side preparation decides whether those
// measurements are rest- or animation-derived.
bool
RigExecPrepareRestDerivedIkChain(
    const std::vector<RigExecPointFrame> &current,
    const std::vector<RigExecPointFrame> &rest,
    std::vector<RigExecPointFrame> *prepared)
{
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
}

// ---------------------------------------------------------------------------
// The pieces of the pose walk that are not the walk: frames read off the
// stage, the deltas a native source rides, the placements a commit
// republishes. Each one is called from the dynamic walk below and is written
// to be callable from the baked program over its dense slots, because a
// second implementation of any of them is a second answer.
// ---------------------------------------------------------------------------

bool
RigExecRigEvaluator::_FrameFromXformRelativeToAsset(
    const UsdPrim &assetRoot,
    UsdGeomXformCache *xformCache,
    const SdfPath &path,
    RigExecPointFrame *outFrame,
    GfMatrix4d *outMatrix) const
{
    const UsdPrim prim = _stage->GetPrimAtPath(path);
    if (!prim || !assetRoot || !UsdGeomXformable(prim)) {
        return false;
    }
    bool resetsBelowAsset = false;
    const GfMatrix4d relative =
        xformCache->ComputeRelativeTransform(prim, assetRoot,
                                             &resetsBelowAsset);
    if (outFrame) {
        *outFrame = RigExecFrameFromMatrix(relative);
    }
    if (outMatrix) {
        *outMatrix = relative;
    }
    return true;
}

bool
RigExecApplyRevisedAncestorDelta(
    const SdfPath &xformPath,
    const RigExecPoseFrameEnumerator &providers,
    RigExecPointFrame *frame)
{
    // A native source that is not itself a written provider may still
    // sit beneath a constrained transform provider. The closest
    // revised ancestor contains all higher ancestor deltas, so apply
    // it once to the stage-derived source frame.
    //
    // The comparison is over POINTS and not whole frames: a provider whose
    // flags differ from its base while its points do not has not moved, and
    // comparing the frames would make it the closest revised ancestor and
    // ride the source on an identity that is not one.
    SdfPath closest;
    RigExecPointFrame closestBase, closestCurrent;
    auto select = [&](const SdfPath &provider,
                      const RigExecPointFrame &base,
                      const RigExecPointFrame &current) {
        if (provider == xformPath || !xformPath.HasPrefix(provider) ||
            current.points == base.points) {
            return;
        }
        if (closest.IsEmpty() ||
            provider.GetPathElementCount() >
                closest.GetPathElementCount()) {
            closest = provider;
            closestBase = base;
            closestCurrent = current;
        }
    };
    providers(select);
    if (!closest.IsEmpty()) {
        GfMatrix4d delta(1.0);
        if (!RigExecPointsToMatrix(
                closestBase.points, closestCurrent.points, &delta)) {
            return false;
        }
        *frame = RigExecMatrixToPoints(frame->points, delta);
    }
    return frame->IsValid();
}

bool
RigExecRigEvaluator::_ResolveNativeXformSource(
    const UsdPrim &assetRoot,
    UsdGeomXformCache *xformCache,
    const SdfPath &xformPath,
    const RigExecPoseFrameEnumerator &providers,
    RigExecPointFrame *out) const
{
    if (!_FrameFromXformRelativeToAsset(assetRoot, xformCache, xformPath, out,
                                        nullptr) ||
        !out->IsValid()) {
        return false;
    }
    return RigExecApplyRevisedAncestorDelta(xformPath, providers, out);
}

void
RigExecRigEvaluator::_UpdateVolumePlacements(
    const RigExecPoseFrameLookup &finalFrameOf,
    RigExecRigPose *pose)
{
    _volumeWeightMatrices.clear();
    for (const auto &[path, tap] : _volumeWeightMatrixTaps) {
        RigExecPointFrame frame;
        GfMatrix4d placement(1.0);
        if (finalFrameOf(path, &frame) && _IsUsableConstraintFrame(frame)) {
            RigExecPointsToMatrix(RigExecIdentityLandmarks(),
                                  frame.points, &placement);
        }
        _volumeWeightMatrices[path] = placement;
    }
    pose->weightFrames = _volumeWeightMatrices;
}

bool
RigExecRigEvaluator::_IkUsesAnimatedTs(const std::vector<SdfPath> &chain) const
{
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
            // HasAuthoredConnections first: see _AuthoredConnections.
            if (attr &&
                (attr.GetNumTimeSamples() > 0 ||
                 (attr.HasAuthoredConnections() &&
                  attr.GetConnections(&connections) &&
                  !connections.empty()))) {
                return true;
            }
        }
    }
    return false;
}

// ---------------------------------------------------------------------------
// The evaluation-mode dispatch. The dynamic generation below is unchanged by
// it: Baked is a request that reaches _EvaluateDynamic whenever there is no
// program to run, and the parity mode runs this same function as its
// reference.
// ---------------------------------------------------------------------------

RigExecRigPose
RigExecRigEvaluator::Evaluate(UsdTimeCode time)
{
    RIGEXEC_PROFILE_SCOPE_CAT(
        _profiler,
        time.IsDefault()
            ? std::string("Evaluate@default")
            : "Evaluate@" + TfStringPrintf("%g", time.GetValue()),
        "evaluate");
    // Settle the epoch first: choosing a path before knowing whether the rig
    // still compiles to the same one is choosing it blind, and it is also
    // what would otherwise make the first frame of every session dynamic.
    std::vector<std::string> settled;
    bool settledOk = false;
    {
        RIGEXEC_PROFILE_SCOPE_CAT(_profiler, "Evaluate.Settle", "evaluate");
        settledOk = _SettleEpoch(&settled);
    }
    if (!settledOk) {
        RigExecRigPose failed;
        failed.time = time;
        failed.diagnostics = std::move(settled);
        return failed;
    }
    // A notice hit the program's capture index, so what it folded in is no
    // longer what the stage says. A structural edit would already have
    // rebuilt it through Compile just now; this covers the edit that moved a
    // value and nothing else, which is the case the digest cannot see.
    if (_bakedProgramStale) {
        _bakedProgramStale = false;
        // Handed over rather than dropped: the op list is what the edit
        // invalidated, not the geometry state around it, and starting that
        // over re-runs every per-point kernel and reports nodes as built
        // that were never rebuilt.
        _RebuildBakedProgram(std::move(_bakedProgram));
    }
    // The mode was asked for while the epoch was dirty -- a scene edit
    // between the compile and the request, which is every request made by a
    // UI after the artist has touched anything. SetEvaluationMode could not
    // build then, because the epoch it would have baked was about to be
    // re-settled; now it has been, so build here. Once per epoch: a rig the
    // program cannot express refuses for reasons the epoch fixes, and
    // re-asking every frame pays for the refusal every frame.
    if (_evaluationMode != RigExecEvaluationMode::Dynamic && _compiled &&
        !_bakedProgram && !_bakeRefused) {
        _RebuildBakedProgram();
    }
    // An override the program cannot place would make it answer a question
    // nobody asked; that generation runs dynamically instead.
    bool overridesPlaceable = true;
    {
        RIGEXEC_PROFILE_SCOPE_CAT(_profiler, "Evaluate.PlaceOverrides", "evaluate");
        overridesPlaceable = !_bakedProgram ||
            _bakedProgram->SetOverrides(_interactiveOverrides);
        if (_bakedProgram) {
            _bakedProgram->SetPublishWeightFields(_publishWeightFields);
        }
    }
    // cpuParityMode publishes an independent scalar oracle for the geometry
    // chains. The program is not that oracle -- it shares the kernels -- so
    // asking for the oracle asks for the dynamic path.
    const bool runBaked = _bakedProgram && !cpuParityMode &&
        overridesPlaceable &&
        _evaluationMode != RigExecEvaluationMode::Dynamic;
    if (!runBaked) {
        RigExecRigPose dynamic = _EvaluateDynamic(time, std::move(settled));
        // cpuParityMode is not a fallback: it ASKS for the dynamic path, so
        // a rig that bakes perfectly still runs here and has nothing to
        // report. The other two ways in do: either no program was built, or
        // one was and could not answer the question the drag asks.
        if (!cpuParityMode && _FallbackIsWorthAnnouncing()) {
            // One reason, two audiences: the harness reads the first line
            // and an artist reads the second, and a generation that fell
            // back for one reason must not be able to name two.
            const std::string why =
                overridesPlaceable
                    ? (_bakeRefusalReasons.empty()
                           ? std::string("no baked program")
                           : _bakeRefusalReasons.front())
                    : std::string("interactive overrides are not placeable");
            _ReportBakeRequired(why, &dynamic);
            _ReportAttributeBakeFallback(why, &dynamic);
        }
        return dynamic;
    }
    RigExecRigPose baked;
    baked.time = time;
    baked.diagnostics = settled;
    bool ranBaked = false;
    {
        RIGEXEC_PROFILE_SCOPE_CAT(_profiler, "Evaluate.Run", "evaluate");
        ranBaked = _bakedProgram->Run(time, &baked);
    }
    if (!ranBaked) {
        // The program handed the generation back mid-flight, so its per-frame
        // caches no longer describe a completed frame. Drop it rather than
        // reuse it, and answer from the path that cannot decline.
        _bakedProgram.reset();
        _bakedProgramPublished = false;
        RigExecRigPose dynamic = _EvaluateDynamic(time, std::move(settled));
        if (_FallbackIsWorthAnnouncing()) {
            _ReportBakeRequired("program run failed", &dynamic);
            _ReportAttributeBakeFallback("program run failed", &dynamic);
        }
        return dynamic;
    }
    ++_bakedGenerations;
    _bakedProgramPublished = true;
    if (_evaluationMode == RigExecEvaluationMode::Baked) {
        return baked;
    }
    // BakedWithParityCheck publishes the DYNAMIC generation: it is the
    // reference, so a disagreement must not also change what consumers see.
    RigExecRigPose reference = _EvaluateDynamic(time, std::move(settled));
    RigExecComparePoses(reference, baked, &reference);
    if (reference.bakedParityMismatches) {
        // The mode exists to be believed or disbelieved, and a count that
        // only a caller who thought to read it can see is neither. One line
        // on stderr is also what lets an existing suite be re-run under the
        // mode and FAIL on a disagreement it never looks for itself.
        TF_WARN("rigExec: %zu baked parity mismatch(es) on %s at %s",
                reference.bakedParityMismatches, _rigPath.GetText(),
                time.IsDefault()
                    ? "default"
                    : TfStringPrintf("%g", time.GetValue()).c_str());
    }
    return reference;
}

void
RigExecRigEvaluator::_ReportBakeRequired(const std::string &detail,
                                         RigExecRigPose *pose) const
{
    if (!_BakeRequired() ||
        _evaluationMode == RigExecEvaluationMode::Dynamic) {
        return;
    }
    // The same prefix a real disagreement carries, and deliberately so: to a
    // suite asking "did the program answer this generation", falling back
    // and answering differently are the same failure, and one regex should
    // catch both. Nothing published moves -- the pose is the dynamic path's,
    // which is the reference the mode compares against anyway.
    const std::string message =
        "baked parity mismatch: bake required, evaluated dynamically: " +
        detail;
    pose->diagnostics.push_back(message);
    ++pose->bakedParityMismatches;
    // Same one line on stderr a real disagreement gets, for the same
    // reason: a suite that never reads pose.diagnostics is exactly the
    // suite this variable exists to re-run, and it can only fail on what
    // it prints.
    TF_WARN("rigExec: %s on %s", message.c_str(), _rigPath.GetText());
}

void
RigExecRigEvaluator::_ReportAttributeBakeFallback(const std::string &detail,
                                                  RigExecRigPose *pose) const
{
    if (_evaluationModeSource != RigExecEvaluationModeSource::Attribute ||
        _evaluationMode == RigExecEvaluationMode::Dynamic) {
        return;
    }
    // Deliberately NOT the prefix above, and deliberately uncounted. That
    // prefix is a test harness's failure signal and bakedParityMismatches is
    // what it counts; an attribute somebody authored on the asset is a
    // REQUEST, and a rig that asks for the program and is answered by the
    // dynamic path has been answered CORRECTLY, only slowly. Turning that
    // into a suite failure would make authoring the attribute the dangerous
    // choice, which is the opposite of what it is for.
    //
    // No TF_WARN either: the fallback is a property of the epoch, so the
    // line would repeat on every frame of a session for as long as the
    // epoch stands. It goes on the pose, where a consumer reads it once per
    // generation and a tool prints it beside the rig's other diagnostics.
    pose->diagnostics.push_back(
        "rigExec:baked is set on " + _rigPath.GetString() +
        " but this generation was evaluated dynamically: " + detail);
}

bool
RigExecRigEvaluator::_FallbackIsWorthAnnouncing() const
{
    // A mode nobody asked to be baked cannot fall back to anything: the
    // dynamic path is the answer, not a substitute for one.
    if (_evaluationMode == RigExecEvaluationMode::Dynamic) {
        return false;
    }
    return _BakeRequired() ||
        _evaluationModeSource == RigExecEvaluationModeSource::Attribute;
}

bool
RigExecRigEvaluator::_WantsBakeRefusalReasons() const
{
    return _BakeRequired() ||
        _evaluationModeSource == RigExecEvaluationModeSource::Attribute;
}

RigExecEvaluationMode
RigExecRigEvaluator::_PeekEvaluationMode() const
{
    // The same three-way precedence _RefreshAttributeEvaluationMode applies,
    // read-only. Compile has to decide whether to defer the dynamic-only
    // preparations in its PrepareRequests phase, and the refresh runs at the
    // TAIL of Compile on purpose -- rigExec:baked is composed, so a
    // reference swap can change it with nothing else moving, and the rebuild
    // wants the latest answer. Asking here rather than moving that call
    // keeps the documented ordering.
    //
    // If the two ever disagreed -- composition changing mid-compile -- the
    // cost is a dynamic session whose first frame prepares its own requests.
    // Slower once, never wrong.
    if (_evaluationModeSource == RigExecEvaluationModeSource::Explicit ||
        _evaluationModeSource == RigExecEvaluationModeSource::Environment) {
        return _evaluationMode;
    }
    if (_stage) {
        if (const UsdPrim rig = _stage->GetPrimAtPath(_rigPath)) {
            const UsdAttribute attribute =
                rig.GetAttribute(_BakedAttributeName());
            bool baked = false;
            if (attribute && attribute.HasAuthoredValue() &&
                attribute.Get(&baked) && baked) {
                return RigExecEvaluationMode::Baked;
            }
        }
    }
    return RigExecEvaluationMode::Dynamic;
}

bool
RigExecRigEvaluator::_RefreshAttributeEvaluationMode()
{
    // The attribute is the weakest of the three requests, so this is a
    // no-op the moment a stronger one has been made: SetEvaluationMode is a
    // caller that chose knowing more than the asset does, and
    // RIGEXEC_EVALUATION_MODE is a whole session's answer that the parity
    // suites depend on being able to force onto any stage they open.
    if (_evaluationModeSource == RigExecEvaluationModeSource::Explicit ||
        _evaluationModeSource == RigExecEvaluationModeSource::Environment) {
        return false;
    }
    bool authored = false;
    bool baked = false;
    if (_stage) {
        if (const UsdPrim rig = _stage->GetPrimAtPath(_rigPath)) {
            const UsdAttribute attribute =
                rig.GetAttribute(_BakedAttributeName());
            // AUTHORED, not merely readable: the schema answers false on
            // every RigExecRoot ever written, so a value alone cannot say
            // whether anybody asked. An authored false is still somebody
            // asking -- it is how an asset says "not this one" over a
            // reference that says otherwise -- so it keeps the source and
            // only the MODE goes back to Dynamic.
            if (attribute && attribute.HasAuthoredValue()) {
                authored = attribute.Get(&baked);
            }
        }
    }
    _evaluationModeSource = authored
        ? RigExecEvaluationModeSource::Attribute
        : RigExecEvaluationModeSource::Default;
    const RigExecEvaluationMode mode = authored && baked
        ? RigExecEvaluationMode::Baked
        : RigExecEvaluationMode::Dynamic;
    if (mode == _evaluationMode) {
        return false;
    }
    _evaluationMode = mode;
    // Same reasoning SetEvaluationMode states: the question is being asked
    // again, and a refusal remembered from the last answer says nothing
    // about this one.
    _bakeRefused = false;
    _bakeRefusalReasons.clear();
    return true;
}

bool
RigExecRigEvaluator::_NoticeNamesTheBakedAttribute(
    const UsdNotice::ObjectsChanged &notice) const
{
    const SdfPath baked = _rigPath.AppendProperty(_BakedAttributeName());
    for (const SdfPath &path : notice.GetChangedInfoOnlyPaths()) {
        if (path == baked) {
            return true;
        }
    }
    // A resync names a prim and everything under it went with it, which is
    // how the attribute arrives on a reference arc or leaves with a muted
    // layer -- neither of which reports a changed-info path for it.
    for (const SdfPath &path : notice.GetResyncedPaths()) {
        if (baked.HasPrefix(path)) {
            return true;
        }
    }
    return false;
}

void
RigExecRigEvaluator::SetEvaluationMode(RigExecEvaluationMode mode)
{
    // Before the early return, because what this call settles is WHO
    // decides and not only what was decided: a tool that asks for the mode
    // it is already in has still taken the decision away from the rig's
    // rigExec:baked, and a later notice on that attribute must not take it
    // back.
    _evaluationModeSource = RigExecEvaluationModeSource::Explicit;
    if (mode == _evaluationMode) {
        return;
    }
    _evaluationMode = mode;
    _bakedProgramStale = false;
    // An explicit request is a new question even where the last one was
    // refused, so it does not inherit the epoch's refusal.
    _bakeRefused = false;
    _bakeRefusalReasons.clear();
    if (_compiled && !_structureDirty) {
        // Baked <-> parity keeps the geometry state: the program is rebuilt
        // because the DISPATCH changed, and nothing about the rig did.
        _RebuildBakedProgram(std::move(_bakedProgram));
    } else {
        // Dynamic, or an epoch that has not settled. Either way this
        // evaluator has no program until Evaluate builds one.
        _bakedProgram.reset();
        _bakedProgramPublished = false;
    }
    // A dirty epoch is not a refusal: Evaluate builds the program once the
    // epoch has settled, which is where it can know what it would be baking.
}

void
RigExecRigEvaluator::_RebuildBakedProgram(
    std::unique_ptr<RigExecBakedProgram> outgoing)
{
    // Read before anything can replace the program it describes, and cleared
    // here because from this line on no program of this evaluator has
    // published anything.
    const bool outgoingPublished = _bakedProgramPublished;
    _bakedProgramPublished = false;
    if (_evaluationMode == RigExecEvaluationMode::Dynamic) {
        _bakedProgram.reset();
        return;
    }
    RIGEXEC_PROFILE_SCOPE_CAT(_profiler, "Compile.Bake", "compile");
    ++_bakedProgramBuildAttempts;
    // A refusal is only reportable if somebody kept it: Build is the one
    // thing that knows why, and it hands the reasons back only when it is
    // given a vector. Production passes nullptr, which saves no walk -- the
    // bakeability check assembles its refusals to answer at all -- it drops
    // them, because the dispatch needs the yes/no alone. The two callers
    // that do read them are RIGEXEC_BAKE_REQUIRED and a rig that asked for
    // the program through its own attribute, which is owed the reason it
    // did not get one; see _WantsBakeRefusalReasons.
    std::vector<std::string> reasons;
    _bakedProgram = RigExecBakedProgram::Build(
        this, _WantsBakeRefusalReasons() ? &reasons : nullptr);
    if (_bakedProgram) {
        ++_bakedProgramBuilds;
        if (outgoing && outgoingPublished) {
            // The replacement inherits the geometry nodes that survive,
            // matched the way the dynamic walk matches its VdfNetwork nodes.
            // Without it a rebuilt program re-runs every per-point kernel and
            // publishes "created"/"schedule(s) built" counters, and the mover
            // graph diagnostic, for nodes nothing rebuilt -- while the
            // dynamic path, whose graphs stood through the same edit, reports
            // none of it.
            //
            // Only from a program that PUBLISHED a generation, which is what
            // makes the sentence above true: the whole of what an unrun
            // program carries here is `created = false` on nodes whose
            // creation no consumer has been told about, so adopting it makes
            // the replacement under-report work the dynamic path -- whose
            // graphs are still cold -- goes on to report. The case is reached
            // by building at Compile and changing the mode afterwards, which
            // is what a rig carrying rigExec:baked does whenever a tool then
            // asks for the parity check.
            _bakedProgram->AdoptGeometryStateFrom(*outgoing);
        }
    }
    // Refusing is a property of the epoch, not of the moment: remember it so
    // the lazy build in Evaluate asks once rather than once per frame. The
    // reasons ride with it: the fallback that reports them happens per
    // frame, long after the one build that could say why.
    _bakeRefused = !_bakedProgram;
    _bakeRefusalReasons = std::move(reasons);
}

bool
RigExecRigEvaluator::IsBakeable(std::vector<std::string> *reasons) const
{
    return RigExecBakedProgram::IsBakeable(*this, reasons);
}

bool
RigExecRigEvaluator::_SettleEpoch(std::vector<std::string> *diagnostics)
{
    if (!_compiled && !Compile(diagnostics)) {
        return false;
    }
    // Structural edits begin a new epoch: recompile when the composed
    // mover topology digest changed (spec §4.2, §6.3).
    const bool edited = _structureDirty;
    bool recompiled = false;
    if (_structureDirty && _ComputeStructureDigest() != _structureDigest) {
        if (!Compile(diagnostics)) {
            diagnostics->push_back("structural recompilation failed");
            return false;
        }
        recompiled = true;
        diagnostics->push_back("structural edit: epoch rebuilt");
    }
    _structureDirty = false;
    if (edited && !recompiled) {
        // An edit that did not change the digest can still have changed the
        // KIND of a rest channel -- authored the first time sample on one,
        // connected it, unmuted a layer that animates it. The epoch-constant
        // rest frames then stop being a legal simplification of the
        // per-frame ones, and the digest is blind to that as well, so the
        // classification is re-asked here and answered by recompiling. This
        // is the only place the rest taps can be moved back into the
        // per-frame first-frame-pose request, which is what the fallback is.
        if (!_restTapIds.empty() && _EpochRestsMightVary()) {
            if (!Compile(diagnostics)) {
                diagnostics->push_back("structural recompilation failed");
                return false;
            }
            _structureDirty = false;
            diagnostics->push_back(
                "rest channel became time-varying: epoch rebuilt");
            return true;
        }
        // Otherwise the kind is unchanged and only the VALUES can have
        // moved. A recompile has just pulled fresh rests; anything else that
        // edited the stage has to.
        if (!_RefreshEpochRestFrames()) {
            diagnostics->push_back("rest frame evaluation incomplete");
            return false;
        }
    }
    return true;
}

bool
RigExecRigEvaluator::_RealizeDeferredExecPrep(RigExecRigPose *pose)
{
    RIGEXEC_PROFILE_SCOPE_CAT(_profiler, "DeferredExecPrep", "compile");
    // DROP THE GIL, for the reason Compile states at length and with the
    // same measurement behind it: the first ExecUsdSystem::PrepareRequest
    // in a process lazily loads the exec definition plugins through
    // TfScriptModuleLoader, which needs the GIL, and a Python caller holds
    // it across this call. Deferring the preparation is precisely what
    // moves that first PrepareRequest OUT of Compile's guarded scope, so
    // the guard has to come with it -- without this, a deferred epoch
    // reintroduces the hang Compile was given its guard to fix, and does it
    // only when a Python caller races a worker to the registry.
    TF_PY_ALLOW_THREADS_IN_SCOPE();
    // Cleared first, and whatever happens: a request that will not prepare
    // will not prepare on the next frame either, and retrying it every
    // generation would turn one compile's cost into every frame's.
    _execPrepDeferred = false;

    if (_firstFramePoseTaps && !_firstFramePoseTaps->Prepare()) {
        pose->diagnostics.push_back(
            "failed to prepare pose provider inputs");
        return false;
    }
    if (_taps && !_taps->Prepare()) {
        pose->diagnostics.push_back(
            "failed to build a valid prepared request for the epoch");
        return false;
    }
    for (_SolverBatch &batch : _solverBatches) {
        if (batch.taps && !batch.taps->Prepare()) {
            pose->diagnostics.push_back(
                "failed to prepare solver dependency level");
            return false;
        }
    }
    // The warm Compile does for an eagerly prepared epoch, done here for a
    // deferred one: the first override-bearing pull would otherwise compute
    // the whole seed network from an empty cache behind every override.
    if (_firstFramePoseTaps) {
        RIGEXEC_PROFILE_SCOPE_CAT(_profiler, "DeferredExecPrep.Warm",
                                  "compile");
        _firstFramePoseTaps->Warm(_restTime);
    }
    return true;
}

RigExecRigPose
RigExecRigEvaluator::_EvaluateDynamic(UsdTimeCode time,
                                      std::vector<std::string> diagnostics)
{
    RigExecRigPose pose;
    pose.time = time;
    pose.diagnostics = std::move(diagnostics);
    if (!_SettleEpoch(&pose.diagnostics)) {
        return pose;
    }
    // This is the one path that pulls the deferred requests, and the first
    // line of it that could: everything below reads a snapshot from one of
    // them. After _SettleEpoch, because settling may compile a new epoch --
    // and it is that epoch's requests, not the retired one's, that are owed
    // preparing.
    if (_execPrepDeferred && !_RealizeDeferredExecPrep(&pose)) {
        return pose;
    }

    // Region stamps for the stretches that cannot take an RAII scope,
    // because a scope needs a block and the block would scope out the
    // lambdas the rest of the walk calls.
    //
    // MEASURED 2026-09-13, biped: 3.3-3.9 ms/frame of this function sat
    // inside no profiler scope at all -- 24% of a no-change evaluate, more
    // than AuthoritativeSnapshot. The publish loops turned out to be only
    // 1.2 ms of that; the rest is here, in the per-provider frame maps that
    // are rebuilt from scratch every generation. Instrumenting first is what
    // kept a design from being written against the wrong 2.5 ms.
    const bool profileRegions = _profiler.IsEnabled();
    uint64_t regionStart = profileRegions ? RigExecProfiler::NowUs() : 0;
    auto stampRegion = [&](const char *name) {
        if (!profileRegions) {
            return;
        }
        const uint64_t now = RigExecProfiler::NowUs();
        _profiler.Record(name, "evaluate", regionStart, now);
        regionStart = now;
    };

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
    _resolvedInputs.Clear();
    _chainSnapshots.Clear();

    // Interactive overrides are applied on BOTH sides of the property chains,
    // because an override can be either end of one and the two ends want
    // opposite orderings.
    //
    // Here, before the chains: an override on a value a chain READS -- a
    // control avar feeding a math mover -- has to be the value the chain
    // computes from, or dragging that control would move everything except
    // what the mover drives. _resolvedInputs is the route those reads take,
    // and it was cleared one line ago, so this has to come after the clear.
    //
    // Again after them: an override on a property a chain WRITES has to beat
    // the chain's own result. Which of the two situations a given override is
    // in is not knowable here, and applying it twice means it does not have
    // to be.
    if (!_interactiveOverrides.empty()) {
        _ApplyInteractiveOverrides(
            _interactiveOverrides, &baseOverrides, &_resolvedInputs);
    }
    if (!_propertyChains.empty()) {
        RIGEXEC_PROFILE_SCOPE_CAT(_profiler, "PropertyChains", "property");
        _EvaluatePropertyChains(time, &pose.movedProperties, &baseOverrides,
                                &pose.diagnostics);
        // Two delivery routes for one value, and they must not disagree:
        // baseOverrides carries it to every exec consumer; _resolvedInputs
        // carries it to the static reads exec never touches (packet
        // assembly, CPU oracle). _EvaluatePropertyChains writes both routes
        // for every published chain, so no re-sync loop is needed here.
    }

    // The second of the two applications described above: after the chains,
    // before any copy of baseOverrides. A held drag outranks what the rig
    // would have computed for the property it is holding, and nothing is
    // authored either way -- see SetInteractiveOverrides.
    if (!_interactiveOverrides.empty()) {
        _ApplyInteractiveOverrides(
            _interactiveOverrides, &baseOverrides, &_resolvedInputs,
            &pose.movedProperties);
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


    {
        static bool measured = false;
        if (!measured) {
            measured = true;
            std::map<std::string, size_t> typeCount;
            size_t vecBytes = 0;
            for (const auto &o : baseOverrides) {
                std::string tn = o.value.GetTypeName();
                typeCount[tn]++;
            }
            std::fprintf(stderr,
                "RIGEXEC_MEASURE baseOverrides=%zu solverBatches=%zu connectedPoseTaps=%zu firstFramePose=%zu\n",
                baseOverrides.size(), _solverBatches.size(),
                _connectedPoseTaps.size(), _firstFramePoseFrames.size());
            for (const auto &[tn, c] : typeCount)
                std::fprintf(stderr, "  type %-28s count=%zu\n", tn.c_str(), c);
        }
    }
    // Joints whose solver published no element for them (an incomplete
    // solver: its required inputs are unwired, so the kernel returned an
    // empty aggregate). They keep their natural rest-chain frame below, so
    // a rig mid-edit stays visible instead of vanishing.
    //
    // (joint, (solver, element)) in POSE-WALK order, because under a stack
    // the joint is not enough: several solvers may write it and the one that
    // failed is not necessarily the one a joint->solver lookup would name.
    // The verdict beside it is the other half -- a joint another writer DID
    // publish kept that writer's frame and fell back to nothing at all, so
    // the rest-chain sentence would simply be false. The baked path builds
    // both the same way, from the same walk order, and sorts them by the
    // same key, because the two diagnostic streams are compared verbatim.
    std::vector<std::pair<SdfPath, std::pair<SdfPath, int>>> fallbackJoints;
    std::map<SdfPath, SdfPath> lastPublishingWriter;
    std::map<SdfPath, RigExecPointFrameArray> solvedAggregates;
    RigExecSnapshot seedSnapshot;
    if (_firstFramePoseTaps) {
        RIGEXEC_PROFILE_SCOPE_CAT(_profiler, "FirstFramePose", "pose");
        // Warm the shared executor before anything else: every override
        // pull in this evaluator runs in a throwaway sub-executor seeded
        // from the main one, and a cold main cache makes each such pull
        // recompute the whole upstream network behind it.
        _firstFramePoseTaps->Warm(time);
        // The tap-level dirty flag is drained, never consulted: exec's
        // time/value callbacks fire across sibling frames sharing this
        // system and would spuriously veto fresh entries. Genuine stage
        // edits set _firstFramePoseDirty through the notice handler.
        _firstFramePoseTaps->ConsumeDirty();
        _SnapshotCache::Entry *seed =
            !_firstFramePoseDirty
                ? _firstFramePoseCache.Find(baseOverrides, time)
                : nullptr;
        if (seed != nullptr) {
            seedSnapshot = seed->snapshot;
        } else {
            seedSnapshot = _firstFramePoseTaps->Evaluate(time, baseOverrides);
            if (!seedSnapshot.IsValid() || !seedSnapshot.IsComplete()) {
                _firstFramePoseDirty = true;
                pose.diagnostics.push_back("pose provider input evaluation incomplete");
                return pose;
            }
            _firstFramePoseCache.Store(baseOverrides, time, seedSnapshot);
            _firstFramePoseDirty = false;
        }
    }

    // 2. Pose-domain FBX-style constraints, applied in the single composed
    // mover walk. A global walk is essential for SingleChainIK: all joints in
    // its write set must be solved and committed atomically, while ordinary
    // one-provider constraints still chain in exactly the same order as every
    // points/property revision.
    std::map<SdfPath, RigExecPointFrame> baseFrames;
    std::vector<RigExecPointFrame> finalFrames(_providerPaths.size());
    std::vector<char> finalLive(_providerPaths.size(), 0);
    std::vector<RigExecPointFrame> restFrames(_providerPaths.size());
    std::vector<char> restLive(_providerPaths.size(), 0);
    std::unordered_map<SdfPath, GfMatrix4d, SdfPath::Hash> xformDerivedBases;
    std::unordered_map<SdfPath, GfMatrix4d, SdfPath::Hash> finalMatrices;
    /// Geometry-domain constraint results: the delta each one produced, the
    /// envelope it carries, and its optional per-element weight field.
    /// Produced by the pose walk below and consumed after it, the same
    /// in-memory hand-off finalMatrices performs for a "final" read phase.
    std::unordered_map<SdfPath, GfMatrix4d, SdfPath::Hash> constraintDeltas;
    const UsdPrim assetRoot =
        _stage->GetPrimAtPath(_rigPath.GetParentPath());
    UsdGeomXformCache constraintXformCache(time);

    auto frameFromXform = [&](const SdfPath &path,
                              RigExecPointFrame *out,
                              GfMatrix4d *matrix) {
        return _FrameFromXformRelativeToAsset(
            assetRoot, &constraintXformCache, path, out, matrix);
    };

    // Seed every reachable RigExec frame provider before any pose operation.
    // Solvers will replace their owned joint frames when their inputs are ready.
    // The three maps are empty here and _firstFramePoseFrames hands its keys over
    // already ordered, so each insertion goes straight to the end instead of
    // searching the tree it is building.
    // The rests are the epoch's, pulled once at Compile -- unless this epoch
    // has a rest channel that can move with time, in which case they rode in
    // on the seed snapshot with the frames.
    if (_firstFramePoseRests.empty()) {
        // ... or unless a drag is standing on a rest channel. An interactive
        // override is a value that stands in for an authored one, so it has
        // to move a rest frame exactly as authoring it would: the drag and
        // the commit of that same drag must agree, and jointMatricesFinal is
        // the rest->pose map, so a pose that moved against a rest that did
        // not is not a pose of this rig at all. The epoch pull carries no
        // overrides, so for as long as one stands the rests are pulled
        // per frame with them, through the same request and at the frame's
        // own time code -- which is exactly what the per-frame rest taps
        // used to do.
        //
        // Only a rest input can do it: computeRestFrame reads the seven
        // names below on the provider and on its RigExec ancestors and
        // nothing else (see _RestInputNames), and this epoch has no rest
        // channel with an authored connection -- _ProviderRestMightVary
        // refuses the epoch path outright when one does -- so no override on
        // any other attribute can reach a rest frame. A computation override
        // names a computation this cannot inspect, so it counts.
        const bool restOverridden = [this]() {
            for (const RigExecValueOverride &o : _interactiveOverrides) {
                if (o.attribute.IsEmpty() || _IsRestInputName(o.attribute)) {
                    return true;
                }
            }
            return false;
        }();
        if (restOverridden && _restTaps && !_restTapIds.empty()) {
            RIGEXEC_PROFILE_SCOPE_CAT(
                _profiler, "RestFramesOverridden", "pose");
            const RigExecSnapshot rests =
                _restTaps->Evaluate(time, baseOverrides);
            if (!rests.IsValid() || !rests.IsComplete()) {
                pose.diagnostics.push_back(
                    "rest frame evaluation incomplete under an override");
                return pose;
            }
            for (const auto &[provider, tap] : _restTapIds) {
                const int fi = _providerIndex.at(provider);
                restFrames[fi] = rests.Get<RigExecPointFrame>(tap);
                restLive[fi] = 1;
            }
        } else {
            for (const auto &kv : _epochRestFrames) {
                const int fi = _providerIndex.at(kv.first);
                restFrames[fi] = kv.second;
                restLive[fi] = 1;
            }
        }
    }
    // Everything from here to the pose walk rebuilds the per-provider frame
    // maps from scratch, and it is the largest cost in this function that
    // carried no profiler scope: 1.7 ms with nothing changed, 3.2 ms on a
    // root drag. Stamped rather than scoped because the lambdas the pose
    // walk calls are declared in the middle of it, and a block would scope
    // them out.
    regionStart = profileRegions ? RigExecProfiler::NowUs() : 0;
    for (const auto &[provider, tap] : _firstFramePoseFrames) {
        const RigExecPointFrame frame = seedSnapshot.Get<RigExecPointFrame>(tap);
        baseFrames.emplace_hint(baseFrames.end(), provider, frame);
        const int fi = _providerIndex.at(provider);
        finalFrames[fi] = frame;
        finalLive[fi] = 1;
        if (!_firstFramePoseRests.empty()) {
            restFrames[fi] = seedSnapshot.Get<RigExecPointFrame>(
                _firstFramePoseRests.at(provider));
            restLive[fi] = 1;
        }
    }
    // Compose the transform of any plain Xformable lying between the asset
    // root and a provider, which exec resolves as identity and therefore
    // drops (docs/superpowers/specs/2026-09-09-intervening-xform-design.md).
    //
    // At evaluation, from the stage, into the frames in memory. Nothing is
    // authored: the rig follows the Xform the author wrote, wherever they
    // wrote it, and no layer is rewritten.
    if (!_ComposeInterveningXforms(assetRoot, &constraintXformCache,
                                   &restFrames, &restLive, &baseFrames,
                                   &finalFrames, &finalLive, &pose)) {
        return pose;
    }
    for (const SdfPath &provider : _xformDerivedProviders) {
        RigExecPointFrame base;
        GfMatrix4d matrix(1.0);
        if (!frameFromXform(provider, &base, &matrix)) {
            pose.diagnostics.push_back("could not resolve constraint target " +
                provider.GetString() + " relative to the asset root");
            return pose;
        }
        xformDerivedBases[provider] = matrix;
        const int fi = _providerIndex.at(provider);
        restFrames[fi] = RigExecFrameFromMatrix(GfMatrix4d(1.0));
        restLive[fi] = 1;
        baseFrames[provider] = base;
        finalFrames[fi] = base;
        finalLive[fi] = 1;
    }
    stampRegion("FrameSeed");
    // The walk's frame store, in the two shapes the routines it shares with
    // the baked program ask for it: one provider by path, and every provider
    // it holds a base frame for. Both answer straight out of the maps as the
    // walk has left them -- the store is never copied into a third shape in
    // order to be read.
    auto finalFrameOf = [&](const SdfPath &path, RigExecPointFrame *frame) {
        const auto fi = _providerIndex.find(path);
        if (fi == _providerIndex.end() || !finalLive[fi->second]) {
            return false;
        }
        *frame = finalFrames[fi->second];
        return true;
    };
    auto enumerateProviderFrames = [&](const RigExecPoseFrameVisitor &visit) {
        for (std::size_t i = 0; i < _providerPaths.size(); ++i) {
            if (!finalLive[i]) continue;
            const SdfPath &provider = _providerPaths[i];
            const auto base = baseFrames.find(provider);
            if (base == baseFrames.end()) {
                continue;
            }
            visit(provider, base->second, finalFrames[i]);
        }
    };
    auto updateVolumePlacements = [&]() {
        _UpdateVolumePlacements(finalFrameOf, &pose);
    };
    updateVolumePlacements();

    std::function<bool(const SdfPath &)> refreshPoseProvider;
    auto resolveBinding = [&](const _FrameSourceBinding &binding,
                              RigExecPointFrame *out) {
        if (!refreshPoseProvider(binding.sourcePath)) return false;
        // Constraint relationships have implicit `preceding` semantics: the
        // single composed mover walk is the authority, so a later constraint
        // observes every earlier revision of the provider while a reference to
        // a provider written later still sees its current (normally base)
        // value. This is deterministic and cannot create an evaluation cycle.
        if (const auto revised = _providerIndex.find(binding.sourcePath);
            revised != _providerIndex.end() && finalLive[revised->second]) {
            *out = finalFrames[revised->second];
            return out->IsValid();
        }
        if (!binding.xformPath.IsEmpty()) {
            return _ResolveNativeXformSource(
                assetRoot, &constraintXformCache, binding.xformPath,
                enumerateProviderFrames, out);
        }
        return false;
    };

    auto readWeights = [&](const UsdPrim &prim, const char *name,
                           size_t count, std::vector<double> *weights) {
        return _ReadConstraintSourceWeights(prim, name, count, time,
                                            &pose.diagnostics, weights);
    };

    auto readOffsets = [&](const UsdPrim &prim, const char *name,
                           size_t count, std::vector<GfVec3d> *offsets) {
        return _ReadConstraintSourceOffsets(prim, name, count, time,
                                            &pose.diagnostics, offsets);
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
        const auto fi = _providerIndex.find(provider);
        const bool haveFrame =
            fi != _providerIndex.end() && finalLive[fi->second];
        if (wanted == _snapshotPoints.end() ||
            !wanted->second.count(afterMover) || !haveFrame ||
            !finalFrames[fi->second].IsValid()) {
            return;
        }
        const auto &restFrame = restFrames[fi->second];
        const auto &landmarks =
            restLive[fi->second] && restFrame.IsValid()
                ? restFrame.points
                : RigExecIdentityLandmarks();
        GfMatrix4d matrix(1.0);
        if (RigExecPointsToMatrix(landmarks, finalFrames[fi->second].points, &matrix)) {
            _chainSnapshots.Record(provider, afterMover, VtValue(matrix));
        }
    };

    const std::unordered_set<SdfPath, SdfPath::Hash> &hierarchicalProviders =
        _hierarchicalProviderSet;

    // MEASURED 2026-09-13, biped: 3003 calls per frame, 8.1 us each -- 24 ms
    // of a 49 ms evaluate -- because commitConstraintFrames asks this once per
    // descendant joint per constraint, and the same handful of joints are
    // walked again for every constraint in the rig.
    //
    // The predicate reads parent:space AT A TIME, and parent:space may carry
    // time samples, so a compile-time answer would be wrong on any frame but
    // the one it was baked at -- and this predicate decides whether a
    // constraint's pose propagates through a joint, so a wrong answer moves
    // joints by centimetres (see verify_spine.py). Within one Evaluate,
    // `time` is fixed and the stage cannot change, so a memo is byte-identical
    // to recomputing.
    //
    // Two tiers: _namespaceInheritsCache is a persistent member, cleared on
    // epoch change, that records only stage-constant answers (parent:space
    // with no time samples). Time-sampled attributes stay in the per-Evaluate
    // namespacePoseCache and are re-read every frame.
    std::unordered_map<SdfPath, bool, SdfPath::Hash> namespacePoseCache;
    const auto inheritsNamespacePose = [&](const SdfPath &path) {
        const auto persistCached = _namespaceInheritsCache.find(path);
        if (persistCached != _namespaceInheritsCache.end()) {
            return persistCached->second;
        }
        const auto cached = namespacePoseCache.find(path);
        if (cached != namespacePoseCache.end()) {
            return cached->second;
        }
        const UsdPrim prim = _stage->GetPrimAtPath(path);
        bool inherits = true;
        bool stageConstant = true;
        for (const char *name : {"parent:space"}) {
            const UsdAttribute attribute = prim.GetAttribute(TfToken(name));
            SdfPathVector connections;
            // HasAuthoredConnections first: see _AuthoredConnections.
            // `inherits = false; break;` rather than an early return, so the
            // answer still reaches the memo below.
            if (attribute && attribute.HasAuthoredConnections() &&
                attribute.GetConnections(&connections) &&
                !connections.empty()) { inherits = false; break; }
            GfMatrix4d authored(1.0);
            if (attribute) {
                if (attribute.GetNumTimeSamples() > 0) {
                    stageConstant = false;
                }
                if (attribute.Get(&authored, time) &&
                    authored != GfMatrix4d(1.0)) { inherits = false; break; }
            }
        }
        namespacePoseCache.emplace(path, inherits);
        if (stageConstant) {
            _namespaceInheritsCache.emplace(path, inherits);
        }
        return inherits;
    };

    // Nearest pose-owning ancestor-or-self of a path, memoized for this
    // evaluation.
    //
    // Namespace propagation stops at a path that owns its own pose -- a joint
    // a solver writes, or a provider whose parent:space is authored rather
    // than inherited. Both walks below need that answer for a provider
    // relative to some ancestor, and both used to rediscover it by climbing
    // the namespace and re-reading parent:space off the stage at every step,
    // once per propagated descendant: quadratic along a joint chain, and the
    // reason a long spine costs more at its root than at its tip. The nearest
    // owner depends only on the path, so it is computed once and shared.
    //
    // The climb closes over every path element, not only the known providers:
    // a provider's parent need not itself be a provider.
    const auto ownsItsPose = [&](const SdfPath &path) {
        return _jointSolverBinding.count(path) ||
            (hierarchicalProviders.count(path) && !inheritsNamespacePose(path));
    };
    const auto nearestBlocking = [&](const SdfPath &path) {
        std::vector<SdfPath> pending;
        SdfPath walk = path;
        SdfPath owner;
        for (; !walk.IsEmpty(); walk = walk.GetParentPath()) {
            const auto cached = _nearestBlockingCache.find(walk);
            if (cached != _nearestBlockingCache.end()) {
                owner = cached->second;
                break;
            }
            if (ownsItsPose(walk)) {
                _nearestBlockingCache[walk] = walk;
                owner = walk;
                break;
            }
            pending.push_back(walk);
        }
        for (const SdfPath &seen : pending) {
            _nearestBlockingCache[seen] = owner;
        }
        return owner;
    };

    std::set<SdfPath> constrainedProviders;
    // Validate and commit one constraint's complete write bundle. Descendant
    // RigExec providers are updated from the nearest changed ancestor in the
    // same transaction; native Xform descendants ride the published ancestor
    // delta in Hydra and therefore must not be duplicated here.
    auto commitConstraintFrames =
        [&](const SdfPath &moverPath,
            const std::map<SdfPath, RigExecPointFrame> &candidates,
            bool solverOutput = false, bool derivedRefresh = false) {
        for (const auto &[path, frame] : candidates) {
            if (!solverOutput && !_IsUsableConstraintFrame(frame)) {
                pose.diagnostics.push_back(
                    moverPath.GetString() +
                    " produced an invalid or degenerate frame for " +
                    path.GetString() + "; constraint passed through");
                return false;
            }
        }

        std::vector<std::pair<int, RigExecPointFrame>> propagated;
        // The hierarchy delta for a descendant depends only on its nearest
        // candidate ancestor ("closest"): that ancestor's old (before) and new
        // (candidate) frames are shared by every descendant under it, so the
        // delta is computed once per closest and reused. It is ALWAYS
        // applied, even when before and candidate are exactly equal: the
        // baked program applies unconditionally, and skipping the multiply
        // differs from applying an almost-identity by rounding dust --
        // which is exactly what the parity comparisons forbid.
        std::map<SdfPath, GfMatrix4d> closestDelta;
        // Only descendants can change. Enumerate disjoint changed subtrees,
        // rather than scanning every provider for each solver dependency level.
        std::vector<int> descendants;
        SdfPath coveredRoot;
        {
            RIGEXEC_PROFILE_SCOPE_CAT(_profiler, "ccfDesc", "pose");
        for (const auto &[target, candidate] : candidates) {
            if (!coveredRoot.IsEmpty() && target.HasPrefix(coveredRoot)) continue;
            coveredRoot = target;
            const auto ti = _providerIndex.find(target);
            if (ti == _providerIndex.end()) continue;
            if (!_hierDescendants[ti->second].empty()) {
                for (int di : _hierDescendants[ti->second]) {
                    if (!candidates.count(_providerPaths[di]))
                        descendants.push_back(di);
                }
            } else {
                for (std::size_t j = static_cast<std::size_t>(ti->second) + 1;
                     j < _providerPaths.size() &&
                     _providerPaths[j].HasPrefix(target); ++j) {
                    if (candidates.count(_providerPaths[j])) continue;
                    if (!hierarchicalProviders.count(_providerPaths[j])) continue;
                    descendants.push_back(static_cast<int>(j));
                }
            }
        }
        }
        if (candidates.size() == 1) {
            const SdfPath &closest = candidates.begin()->first;
            const auto beforeIt = _providerIndex.find(closest);
            const bool haveBefore = beforeIt != _providerIndex.end() &&
                                    finalLive[beforeIt->second];
            const RigExecPointFrame &candFrame = candidates.begin()->second;
            // Single candidate: every descendant maps through the same
            // closest, so one shared hierarchy delta serves them all.
            GfMatrix4d delta(1.0);
            bool sharedSingular = false;
            if (haveBefore &&
                !RigExecPointsToMatrix(finalFrames[beforeIt->second].points,
                                       candFrame.points, &delta)) {
                sharedSingular = true;
            }
            {
                RIGEXEC_PROFILE_SCOPE_CAT(_profiler, "ccfSingle", "pose");
            for (int di : descendants) {
                const SdfPath &provider = _providerPaths[di];
                const RigExecPointFrame &current = finalFrames[di];
                // An independently solved joint is an absolute posed
                // override; namespace propagation cannot pass through it.
                const SdfPath blocker = nearestBlocking(provider);
                if (!blocker.IsEmpty() && blocker != closest &&
                    blocker.HasPrefix(closest)) {
                    continue;
                }
                if (solverOutput && (!haveBefore ||
                    !_IsUsableConstraintFrame(current) ||
                    !_IsUsableConstraintFrame(finalFrames[beforeIt->second]) ||
                    !_IsUsableConstraintFrame(candFrame))) continue;
                if (!haveBefore ||
                    !_IsUsableConstraintFrame(current)) {
                    pose.diagnostics.push_back(
                        moverPath.GetString() +
                        " could not propagate its pose revision through " +
                        provider.GetString() + "; constraint passed through");
                    return false;
                }
                if (sharedSingular) {
                    pose.diagnostics.push_back(
                        moverPath.GetString() +
                        " produced a singular hierarchy delta; constraint "
                        "passed through");
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
                propagated.emplace_back(di, frame);
            }
            }
        } else {
            for (int di : descendants) {
                const SdfPath &provider = _providerPaths[di];
                const RigExecPointFrame &current = finalFrames[di];
                SdfPath closest = provider.GetParentPath();
                for (; !closest.IsEmpty() && !candidates.count(closest);
                     closest = closest.GetParentPath()) {}
                if (closest.IsEmpty()) {
                    continue;
                }
                const SdfPath blocker = nearestBlocking(provider);
                if (!blocker.IsEmpty() && blocker != closest &&
                    blocker.HasPrefix(closest)) {
                    continue;
                }
                const auto beforeIt = _providerIndex.find(closest);
                const bool haveBefore = beforeIt != _providerIndex.end() &&
                                        finalLive[beforeIt->second];
                if (solverOutput && (!haveBefore ||
                    !_IsUsableConstraintFrame(current) ||
                    !_IsUsableConstraintFrame(finalFrames[beforeIt->second]) ||
                    !_IsUsableConstraintFrame(candidates.at(closest)))) continue;
                if (!haveBefore ||
                    !_IsUsableConstraintFrame(current)) {
                    pose.diagnostics.push_back(
                        moverPath.GetString() +
                        " could not propagate its pose revision through " +
                        provider.GetString() + "; constraint passed through");
                    return false;
                }
                auto cdIt = closestDelta.find(closest);
                if (cdIt == closestDelta.end()) {
                    GfMatrix4d delta(1.0);
                    if (!RigExecPointsToMatrix(finalFrames[beforeIt->second].points,
                                               candidates.at(closest).points,
                                               &delta)) {
                        pose.diagnostics.push_back(
                            moverPath.GetString() +
                            " produced a singular hierarchy delta; constraint "
                            "passed through");
                        return false;
                    }
                    cdIt = closestDelta.emplace(closest, delta).first;
                }
                RigExecPointFrame frame =
                    RigExecMatrixToPoints(current.points, cdIt->second);
                if (!_IsUsableConstraintFrame(frame)) {
                    pose.diagnostics.push_back(
                        moverPath.GetString() +
                        " produced an invalid descendant frame for " +
                        provider.GetString() + "; constraint passed through");
                    return false;
                }
                propagated.emplace_back(di, frame);
            }
        }

        {
            RIGEXEC_PROFILE_SCOPE_CAT(_profiler, "ccfFinal", "pose");
        for (const auto &[path, frame] : candidates) {
            const int fi = _providerIndex.at(path);
            finalFrames[fi] = frame;
            finalLive[fi] = 1;
            if (!solverOutput && !derivedRefresh) constrainedProviders.insert(path);
            if (solverOutput) baseFrames[path] = frame;
        }
        for (const auto &[idx, frame] : propagated) {
            finalFrames[idx] = frame;
            finalLive[idx] = 1;
            if (solverOutput) baseFrames[_providerPaths[idx]] = frame;
        }
        updateVolumePlacements();
        }
        return true;
    };

    // Connected matrix expressions cannot be updated by a namespace delta:
    // their posed input may live in another subtree. Refresh only those
    // providers, with explicit current/base input phases, before a consumer
    // reads them. Within a generation an unchanged input tuple reuses the
    // previous refresh, even when many operators consume the same control.
    std::map<SdfPath, std::vector<RigExecValueOverride>> connectedInputCache;
    const auto sameOverrides = [](const auto &a, const auto &b) {
        return a.size() == b.size() && std::equal(a.begin(), a.end(), b.begin(),
            [](const RigExecValueOverride &x, const RigExecValueOverride &y) {
                return x.prim == y.prim && x.computation == y.computation &&
                    x.attribute == y.attribute && x.value == y.value;
            });
    };
    refreshPoseProvider = [&](const SdfPath &requested) {
        // Nothing to refresh when no provider has a connected tap: the walk
        // below would visit the whole pose DAG to discover that at every
        // constraint target of every frame. Its only other effect is the
        // cycle diagnostic, and Compile already rejects a cyclic pose DAG.
        if (_connectedPoseTaps.empty()) {
            return true;
        }
        std::vector<std::pair<SdfPath, bool>> pending{{requested, false}};
        std::set<SdfPath> active, complete;
        while (!pending.empty()) {
            const auto [path, ready] = pending.back();
            pending.pop_back();
            if (path.IsEmpty() || complete.count(path) ||
                _jointSolverBinding.count(path) || constrainedProviders.count(path)) continue;
            const auto dependencies = _poseProviderInputs.find(path);
            if (!ready) {
                if (!active.insert(path).second) {
                    pose.diagnostics.push_back("connected pose provider cycle at " + path.GetString());
                    return false;
                }
                pending.emplace_back(path, true);
                if (dependencies != _poseProviderInputs.end()) {
                    for (const SdfPath &input : dependencies->second) {
                        pending.emplace_back(input, false);
                    }
                }
                continue;
            }
            active.erase(path);
            complete.insert(path);
            const auto taps = _connectedPoseTaps.find(path);
            if (taps == _connectedPoseTaps.end()) continue;
            std::vector<RigExecValueOverride> baseInputs = baseOverrides;
            std::vector<RigExecValueOverride> finalInputs = baseOverrides;
            if (dependencies != _poseProviderInputs.end()) {
                for (const SdfPath &input : dependencies->second) {
                    const auto base = baseFrames.find(input);
                    const auto currentIt = _providerIndex.find(input);
                    if (base == baseFrames.end() || currentIt == _providerIndex.end() ||
                        !finalLive[currentIt->second]) continue;
                    baseInputs.push_back({input, _computePointFrame, TfToken(), VtValue(base->second)});
                    finalInputs.push_back({input, _computePointFrame, TfToken(), VtValue(finalFrames[currentIt->second])});
                }
            }
            std::vector<RigExecValueOverride> identity = baseInputs;
            identity.insert(identity.end(), finalInputs.begin(), finalInputs.end());
            const auto cached = connectedInputCache.find(path);
            if (cached != connectedInputCache.end() && sameOverrides(cached->second, identity)) continue;
            auto &stored = _connectedPoseCache[path];
            const bool reuseStored =
                stored.cached && stored.time == time &&
                !taps->second->ConsumeDirty() &&
                sameOverrides(stored.inputs, identity);
            RigExecPointFrame raw;
            RigExecPointFrame current;
            if (reuseStored) {
                raw = stored.base;
                current = stored.current;
            } else {
                const RigExecSnapshot base = taps->second->Evaluate(time, baseInputs);
                if (!base.IsValid() || !base.IsComplete()) {
                    stored.cached = false;
                    pose.diagnostics.push_back("connected base pose input incomplete: " + path.GetString());
                    return false;
                }
                raw = base.Get<RigExecPointFrame>(0);
                current = raw;
                if (!sameOverrides(baseInputs, finalInputs)) {
                    const RigExecSnapshot revised = taps->second->Evaluate(time, finalInputs);
                    if (!revised.IsValid() || !revised.IsComplete()) {
                        stored.cached = false;
                        pose.diagnostics.push_back("connected final pose input incomplete: " + path.GetString());
                        return false;
                    }
                    current = revised.Get<RigExecPointFrame>(0);
                }
            }
            // Maintain the base phase for namespace descendants independently
            // from the current constraint phase. Solver-provided sources are
            // already present in baseFrames; constraint revisions are not.
            const auto previous = baseFrames.find(path);
            GfMatrix4d delta(1.0);
            if (previous != baseFrames.end() && _IsUsableConstraintFrame(raw) &&
                RigExecPointsToMatrix(previous->second.points, raw.points, &delta)) {
                for (auto it = baseFrames.upper_bound(path);
                     it != baseFrames.end() && it->first.HasPrefix(path); ++it) {
                    if (!hierarchicalProviders.count(it->first)) continue;
                    const SdfPath blocker = nearestBlocking(it->first);
                    const bool blocked = !blocker.IsEmpty() &&
                        blocker != path && blocker.HasPrefix(path);
                    if (!blocked) it->second = RigExecMatrixToPoints(it->second.points, delta);
                }
            }
            baseFrames[path] = raw;
            if (!commitConstraintFrames(path, {{path, current}}, false, true)) {
                stored.cached = false;
                return false;
            }
            stored.inputs = identity;
            stored.time = time;
            stored.base = raw;
            stored.current = current;
            stored.cached = true;
            connectedInputCache[path] = std::move(identity);
        }
        return true;
    };

    std::set<size_t> visitedSolverLevels;
    // 2026-09-13: this stamp was briefly believed to be load-bearing -- moving
    // it one line earlier appeared to turn testRigExecNoAuthoring into an
    // access violation and testRigExecWeightOverlay into 0xC0000409. It is
    // not. The crashes were an ABI mismatch: rigExecImaging/bridge.h includes
    // rigEvaluator.h, hence types.h, and types.h was being edited in another
    // window (RigExecBlendSampleData gained a member, so its size changed).
    // A partial rebuild left rigExec.dll and rigExecImaging.dll disagreeing
    // about that layout, which is exactly an access violation in one imaging
    // test and a stack-cookie failure in another -- and why BOTH failures
    // were imaging tests, the clue the "statement placement" theory never
    // explained. Every bisect step rebuilt a different subset, so the
    // pass/fail pattern tracked which DLLs happened to resynchronise, not the
    // edit under test. Placement is irrelevant; verified green in both
    // positions at /O2 and /Od once the tree settled.
    //
    // Left as a note because the methodological error is worth more than the
    // finding was: a single non-reproduced observation was treated as a
    // controlled experiment. Re-run the failing state before believing any
    // bisect taken while another agent is writing shared headers.
    stampRegion("PoseWalkSetup");
    for (const _PoseStep &step : _poseSteps) {
        if (step.solverBatch) {
            _SolverBatch &batch = _solverBatches[step.index];
            RIGEXEC_PROFILE_SCOPE_CAT(
                _profiler,
                "SolverBatch L" + std::to_string(batch.level), "pose");
            std::map<SdfPath, RigExecPointFrame> candidates;
            // Only direct prerequisites enter this request. Copying all previous
            // joint overrides into every level would itself be quadratic for a
            // deep chain, even if the kernels each executed only once.
            // The batch-specific overrides only. baseOverrides is a pure
            // function of `time` -- and (time, tail) uniquely determines the
            // full exec input -- only while no interactive overrides are
            // held: a held drag rides into baseOverrides beside the chains,
            // so a tail hit under a drag would answer with another override
            // set's base. The Find and the Store below both stand down then;
            // the repeat-pass frames the cache exists for carry no overrides.
            std::vector<RigExecValueOverride> tail;
            {
                RIGEXEC_PROFILE_SCOPE_CAT(_profiler, "SolverBatch.Inputs", "pose");
                tail.reserve(batch.dependencies.size() + batch.frameInputs.size() +
                             batch.restInputs.size());
            }
            for (const SdfPath &dependency : batch.dependencies) {
                const auto aggregate = solvedAggregates.find(dependency);
                if (aggregate != solvedAggregates.end()) {
                    tail.push_back({dependency, _computePointFrameArray, TfToken(),
                                    VtValue(aggregate->second)});
                }
            }
            for (const SdfPath &input : batch.frameInputs) {
                if (!refreshPoseProvider(input)) return pose;
                const auto fi = _providerIndex.find(input);
                if (fi != _providerIndex.end() && finalLive[fi->second]) {
                    tail.push_back({input, _computePointFrame, TfToken(),
                                    VtValue(finalFrames[fi->second])});
                }
            }
            // "The incoming frame replaces the authored rest" (spec §4.2).
            //
            // The joint prim publishes computePointFrame AND computeRestFrame,
            // and RigExecTwoBoneIk / RigExecSplineIk request the latter by
            // name off rigExec:joints -- so the whole of the dynamic-side
            // change is one more override loop on a DIFFERENT computation.
            // rigExec:joints stays skipped in solverFrameInputs: overriding a
            // solver's own output joints as computePointFrame would feed the
            // solver back into itself, which is a different thing entirely.
            //
            // A live entry takes the frame the preceding step left; an entry
            // with no predecessor pins the AUTHORED rest, because
            // computeRestFrame reads its namespace ancestor's and an override
            // on the hip would otherwise move the knee's rest too. The map is
            // empty unless something is live, so a rig with no pose step below
            // a solver pushes nothing here at all.
            for (const auto &[joint, predecessor] : batch.restInputs) {
                const bool live = !predecessor.IsEmpty();
                RigExecPointFrame rest;
                bool haveRest = false;
                if (live) {
                    const auto fi = _providerIndex.find(joint);
                    if (fi != _providerIndex.end() && finalLive[fi->second]) {
                        rest = finalFrames[fi->second];
                        haveRest = true;
                    }
                } else {
                    const auto fi = _providerIndex.find(joint);
                    if (fi != _providerIndex.end() && restLive[fi->second]) {
                        rest = restFrames[fi->second];
                        haveRest = true;
                    }
                }
                if (haveRest) {
                    // A solver with a basis of its own -- an FK chain
                    // composing control deltas -- switches to the joint's
                    // rest reference only where this flag says a step below
                    // it really wrote the joint. Everywhere else it keeps the
                    // basis it always had, which is what makes the rule free
                    // on every rig that does not stack.
                    if (live) rest.flags |= RigExecPointFrameLiveRest;
                    tail.push_back({joint, _computeRestFrame, TfToken(),
                                    VtValue(rest)});
                }
            }
            // A batch snapshot is a pure function of (time, inputs). Inputs
            // are themselves assembled from the seed snapshot, prior-level
            // aggregates, and rest frames -- all deterministic for a given
            // frame -- so a recently-computed entry is reusable. The tap-level
            // dirty flag is drained, not consulted (it fires across sibling
            // frames sharing the system); genuine edits ride batch.dirty.
            batch.taps->ConsumeDirty();
            _SnapshotCache::Entry *hit = nullptr;
            {
                RIGEXEC_PROFILE_SCOPE_CAT(_profiler, "SolverBatch.Find", "pose");
                // No cache under a held drag: the key is (tail, time) but
                // the result depends on baseOverrides too, which the drag
                // is part of. See the note where the tail is assembled.
                hit = (!batch.dirty && _interactiveOverrides.empty())
                    ? batch.cache.Find(tail, time)
                    : nullptr;
            }
            if (hit != nullptr) {
                batch.snapshot = hit->snapshot;
            } else {
                RIGEXEC_PROFILE_SCOPE_CAT(
                    _profiler, "ExecEvaluate", "exec");
                std::vector<RigExecValueOverride> inputs;
                inputs.reserve(baseOverrides.size() + tail.size());
                inputs.insert(inputs.end(), baseOverrides.begin(), baseOverrides.end());
                inputs.insert(inputs.end(), tail.begin(), tail.end());
                const RigExecSnapshot refreshed =
                    batch.taps->Evaluate(time, inputs);
                if (!refreshed.IsValid() || !refreshed.IsComplete()) {
                    batch.dirty = true;
                    pose.solverOverridesConverged = false;
                    pose.diagnostics.push_back(
                        "solver dependency level evaluation incomplete");
                    return pose;
                }
                if (_interactiveOverrides.empty()) {
                    batch.cache.Store(tail, time, refreshed);
                    // Cleared only beside the store: an edit that dirtied
                    // the batch ahead of a drag must still force a
                    // re-resolve once the drag releases.
                    batch.dirty = false;
                }
                batch.snapshot = refreshed;
                // ChangeTime/compilation can notify while Evaluate runs. The
                // successful snapshot already includes those invalidations.
                batch.taps->ConsumeDirty();
                // Count the exec pulls actually performed, not the batches
                // the schedule visited: a cache hit reuses the stored
                // snapshot without evaluating, and the counter is how the
                // suite proves it (an unchanged pull evaluates nothing).
                pose.solverEvaluations += batch.solvers.size();
            }
            const RigExecSnapshot &values = batch.snapshot;
            if (visitedSolverLevels.insert(batch.level).second) {
                ++pose.solverOverrideRounds;
            }
            for (const auto &[solver, tap] : batch.solvers) {
                const RigExecPointFrameArray aggregate =
                    values.Get<RigExecPointFrameArray>(tap);
                solvedAggregates[solver] = aggregate;
                const auto joints = _solverJoints.find(solver);
                if (joints == _solverJoints.end()) {
                    continue;
                }
                for (const auto &[joint, element] : joints->second) {
                    if (element < 0 || size_t(element) >= aggregate.GetSize()) {
                        fallbackJoints.push_back({joint, {solver, element}});
                        continue;
                    }
                    const RigExecPointFrame frame =
                        RigExecExtractElementFrame(&aggregate, size_t(element));
                    candidates[joint] = frame;
                    lastPublishingWriter[joint] = solver;
                }
            }
            if (!candidates.empty()) {
                commitConstraintFrames(SdfPath(), candidates, true);
            }
            if (!candidates.empty() && !_snapshotPoints.empty()) {
                // A solver checkpoint: the joint as THIS writer left it, for
                // a reader whose read phase names it. Skipped whole unless
                // SOMETHING on the rig asked for a checkpoint -- recordFrame
                // early-outs per pair anyway, but that is still two map
                // lookups per bound joint per frame on a rig that never names
                // a solver, which is every rig that does not stack.
                for (const auto &[solver, tap] : batch.solvers) {
                    const auto joints = _solverJoints.find(solver);
                    if (joints == _solverJoints.end()) {
                        continue;
                    }
                    for (const auto &[joint, element] : joints->second) {
                        if (candidates.count(joint)) {
                            recordFrame(joint, solver);
                        }
                    }
                }
            }
            continue;
        }
        const _FrameConstraint &constraint = _frameConstraints[step.index];
        RIGEXEC_PROFILE_SCOPE_CAT(
            _profiler,
            constraint.schemaType.GetString() + " " +
                constraint.moverPath.GetName(),
            "pose");
        for (const SdfPath &target : constraint.targets) {
            if (!refreshPoseProvider(target)) return pose;
        }
        if (!refreshPoseProvider(constraint.weightObject)) return pose;
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
            if (!constraint.pointsTarget.IsEmpty())
                constraintDeltas[constraint.moverPath] = GfMatrix4d(1.0);
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
                const auto fi = _providerIndex.find(joint);
                if (fi == _providerIndex.end() || !finalLive[fi->second]) {
                    pose.diagnostics.push_back(
                        constraint.moverPath.GetString() +
                        " has no current frame for " + joint.GetString());
                    inputsValid = false;
                    break;
                }
                chain.push_back(finalFrames[fi->second]);
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
                 _IkUsesAnimatedTs(constraint.ikChain));
            if (inputsValid && !useAnimatedTs) {
                std::vector<RigExecPointFrame> rest;
                rest.reserve(constraint.ikChain.size());
                for (size_t i = 0; i < constraint.ikChain.size(); ++i) {
                    const SdfPath &joint = constraint.ikChain[i];
                    // The incoming frame IS the rest reference where a step
                    // below this constraint wrote the joint; the authored
                    // rest everywhere else.
                    if (i < constraint.ikRestLive.size() &&
                        constraint.ikRestLive[i] && i < chain.size()) {
                        rest.push_back(chain[i]);
                        continue;
                    }
                    const auto fi = _providerIndex.find(joint);
                    if (fi == _providerIndex.end() || !restLive[fi->second]) {
                        inputsValid = false;
                        break;
                    }
                    rest.push_back(restFrames[fi->second]);
                }
                if (!inputsValid ||
                    !RigExecPrepareRestDerivedIkChain(
                        chain, rest, &solveChain)) {
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
        const auto inputIt = _providerIndex.find(constraint.targets[0]);
        const RigExecPointFrame inputFrame =
            (inputIt != _providerIndex.end() && finalLive[inputIt->second])
                ? finalFrames[inputIt->second]
                : RigExecPointFrame();
        RigExecPointFrame candidate = inputFrame;
        bool candidateReady = true;
        // Which mask triple this operator reads is a table property: masks
        // are addressed by (group, axis), so Position reads the translation
        // triple and Rotation and Aim read the rotation one, rather than all
        // three sharing an inputs:affectX that means something different in
        // each.
        const _ConstraintHandler *solveHandler =
            _FindConstraintHandler(constraint.schemaType);
        RigExecConstraintAxisMask affect;
        if (constraint.masksStatic) {
            switch (solveHandler ? solveHandler->maskGroup
                                 : _ChannelGroup::None) {
            case _ChannelGroup::Translation:
                affect = constraint.precompTranslation;
                break;
            case _ChannelGroup::Rotation:
                affect = constraint.precompRotation;
                break;
            case _ChannelGroup::Scale:
                affect = constraint.precompScale;
                break;
            case _ChannelGroup::All:
            case _ChannelGroup::None:
            default:
                affect = RigExecConstraintAxisMask();
                break;
            }
        } else {
            affect = _ReadGroupMask(
                _resolvedInputs, prim,
                solveHandler ? solveHandler->maskGroup : _ChannelGroup::None,
                time);
        }
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
            solveContext.masksStatic = constraint.masksStatic;
            solveContext.precompTranslation = constraint.precompTranslation;
            solveContext.precompRotation = constraint.precompRotation;
            solveContext.precompScale = constraint.precompScale;
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
            if (_IsUsableConstraintFrame(candidate) &&
                frameFromXform(constraint.targets[0], nullptr, &baseMatrix) &&
                std::isfinite(baseMatrix.GetDeterminant()) &&
                baseMatrix.GetDeterminant() != 0.0 &&
                RigExecPointsToMatrix(RigExecIdentityLandmarks(),
                                      candidate.points, &solvedMatrix)) {
                constraintDeltas[constraint.moverPath] =
                    solvedMatrix * baseMatrix.GetInverse();
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

    for (const auto &[provider, tap] : _firstFramePoseFrames) {
        if (!refreshPoseProvider(provider)) return pose;
    }

    // An incomplete solver is an authoring gap, not a silent one: name every
    // writer that published nothing so a rig mid-edit explains itself.
    // Accumulated in walk order, stable-sorted by joint path, so the lines
    // are in the same order the baked epilogue produces them in.
    std::stable_sort(fallbackJoints.begin(), fallbackJoints.end(),
                     [](const std::pair<SdfPath, std::pair<SdfPath, int>> &a,
                        const std::pair<SdfPath, std::pair<SdfPath, int>> &b) {
                         return a.first < b.first;
                     });
    for (const auto &[jointPath, writer] : fallbackJoints) {
        const auto kept = lastPublishingWriter.find(jointPath);
        pose.diagnostics.push_back(
            "solver " + writer.first.GetString() + " published no element " +
            std::to_string(writer.second) + " for joint " +
            jointPath.GetString() + "; " +
            (kept == lastPublishingWriter.end()
                 ? std::string("joint fell back to its rest chain")
                 : "the joint keeps the frame " + kept->second.GetString() +
                       " left"));
    }

    // 1. Transforms and solvers through OpenExec. An incomplete snapshot
    // means some computation failed to compile or evaluate; refusing to
    // continue prevents default-constructed values from masquerading as
    // results (spec §6.6).
    RigExecSnapshot snapshot;
    {
        RIGEXEC_PROFILE_SCOPE_CAT(_profiler, "AuthoritativeSnapshot", "exec");
        _taps->ConsumeDirty();
        // jointOverrides is a pure function of (epoch, time) -- the solver
        // aggregates, base frames, and falloff LUTs are all deterministic
        // given the time -- while no interactive overrides are held. A held
        // drag rides into baseOverrides beside the chains, so a time-keyed
        // hit under a drag would answer with another override set's
        // snapshot. The find and the store below both stand down then.
        // Genuine stage edits set _authSnapshotDirty; the tap-level dirty
        // flag is drained, not consulted (it fires across sibling frames
        // sharing the system).
        bool authResolved = false;
        if (!_authSnapshotDirty && _interactiveOverrides.empty()) {
            const auto tk = _authSnapTimeKeyed.find(time);
            if (tk != _authSnapTimeKeyed.end()) {
                snapshot = tk->second;
                authResolved = true;
            }
        }
        if (!authResolved) {
            std::vector<RigExecValueOverride> jointOverrides = baseOverrides;
            for (const auto &[solver, aggregate] : solvedAggregates) {
                jointOverrides.push_back({solver, _computePointFrameArray,
                                          TfToken(), VtValue(aggregate)});
            }
            for (const auto &[provider, tap] : _firstFramePoseFrames) {
                jointOverrides.push_back({provider, _computePointFrame,
                                          TfToken(),
                                          VtValue(baseFrames.at(provider))});
            }
            jointOverrides.insert(jointOverrides.end(),
                                  _falloffLutOverrides.begin(),
                                  _falloffLutOverrides.end());
            snapshot = _taps->Evaluate(time, jointOverrides);
            if (snapshot.IsValid() && snapshot.IsComplete() &&
                _interactiveOverrides.empty()) {
                if (_authSnapTimeKeyed.size() >= 4)
                    _authSnapTimeKeyed.erase(_authSnapTimeKeyed.begin());
                _authSnapTimeKeyed.emplace(time, snapshot);
                _authSnapshotDirty = false;
            }
        }
    }
    if (!snapshot.IsValid() || !snapshot.IsComplete()) {
        pose.diagnostics.push_back(
            snapshot.IsValid() ? "snapshot incomplete: missing tap values"
                               : "snapshot evaluation failed");
        return pose;
    }

    // Publish final provider matrices after the atomic pose walk.
    //
    // MEASURED 2026-09-13, biped: the three publish loops below cost 3.3-3.9
    // ms/frame -- 24% of a no-change evaluate, more than AuthoritativeSnapshot
    // -- and until these scopes existed NONE of it appeared in the profile.
    // Summing the profiler's rows accounted for only 76% of the floor and
    // nobody had asked where the rest went. The cost is 252 x 3 std::map
    // inserts into jointFramesBase/Final/MatricesFinal, done unconditionally
    // for joints that did not move, plus this provider walk over every
    // finalFrames entry. Scoped separately from the exec snapshot above it
    // because the two want opposite fixes: that one is exec, this one is a
    // container choice (see docs/superpowers/specs/
    // 2026-09-13-sparse-evaluation-design.md, Gate 3).
    {
    RIGEXEC_PROFILE_SCOPE_CAT(_profiler, "PublishProviders", "publish");
    for (std::size_t i = 0; i < _providerPaths.size(); ++i) {
        if (!finalLive[i]) continue;
        const SdfPath &provider = _providerPaths[i];
        const RigExecPointFrame &frame = finalFrames[i];
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
        if (restLive[i] &&
            _IsUsableConstraintFrame(restFrames[i]) &&
            _IsUsableConstraintFrame(frame)) {
            GfMatrix4d matrix(1.0);
            if (RigExecPointsToMatrix(restFrames[i].points, frame.points,
                                      &matrix)) {
                finalMatrices[provider] = matrix;
                _chainSnapshots.RecordFinal(provider, VtValue(matrix));
            }
        }
    }
    }

    // 3. Base and final transform revisions plus paired matrices. A
    // solver-posed joint's base value was supplied as an override above, so
    // exec published it as that joint's computePointFrame; the final value is
    // the in-memory frame revision when the joint carries one, and otherwise
    // is the base (which is what the final-phase tap already resolves to).
    {
    RIGEXEC_PROFILE_SCOPE_CAT(_profiler, "PublishJoints", "publish");
    for (size_t i = 0; i < _jointPaths.size(); ++i) {
        const RigExecPointFrame baseFrame =
            snapshot.Get<RigExecPointFrame>(_jointFrameTaps[i]);
        const auto revisedIt = _providerIndex.find(_jointPaths[i]);
        const bool haveRevised = revisedIt != _providerIndex.end() &&
                                 finalLive[revisedIt->second];
        const RigExecPointFrame finalFrame =
            haveRevised
                ? finalFrames[revisedIt->second]
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
    }
    // 3a. Control frames. Most are animator-authored inputs and therefore
    // publish their base tap directly; a control explicitly named as a
    // constraint write target publishes the revised frame, matching FBX's
    // ability to constrain any transform object. A degenerate/invalid frame
    // remains the status bearer and lets imaging omit the guide.
    {
    RIGEXEC_PROFILE_SCOPE_CAT(_profiler, "PublishControls", "publish");
    for (size_t i = 0; i < _controlPaths.size(); ++i) {
        const auto revised = _providerIndex.find(_controlPaths[i]);
        const bool haveRevised = revised != _providerIndex.end() &&
                                 finalLive[revised->second];
        pose.controlFrames[_controlPaths[i]] =
            haveRevised
                ? finalFrames[revised->second]
                : snapshot.Get<RigExecPointFrame>(_controlFrameTaps[i]);
    }
    }

    // Observational solver guides never gate the rig snapshot: an
    // incomplete guide evaluation degrades to a diagnostic. A consumer that
    // never reads pose.solverFrames disables them outright (see
    // SetSolverGuidesEnabled) and skips the request entirely.
    if (_guideTaps && _solverGuidesEnabled) {
        std::vector<RigExecValueOverride> guideOverrides = baseOverrides;
        guideOverrides.insert(guideOverrides.end(), _falloffLutOverrides.begin(),
                              _falloffLutOverrides.end());
        for (const auto &[solver, aggregate] : solvedAggregates) {
            guideOverrides.push_back({solver, _computePointFrameArray, TfToken(),
                                      VtValue(aggregate)});
        }
        for (const auto &[provider, tap] : _firstFramePoseFrames) {
            guideOverrides.push_back({provider, _computePointFrame, TfToken(),
                                      VtValue(finalFrames[_providerIndex.at(provider)])});
        }
        _guideDirty = _guideTaps->ConsumeDirty() || _guideDirty;
        const bool sameGuideInputs =
            !_guideDirty && _guideTime == time &&
            _guideSnapshot.IsValid() && _guideSnapshot.IsComplete() &&
            guideOverrides.size() == _guideInputs.size() &&
            std::equal(guideOverrides.begin(), guideOverrides.end(),
                       _guideInputs.begin(),
                       [](const RigExecValueOverride &a,
                          const RigExecValueOverride &b) {
                           return a.prim == b.prim &&
                                  a.computation == b.computation &&
                                  a.attribute == b.attribute &&
                                  a.value == b.value;
                       });
        RigExecSnapshot guideSnapshot;
        if (sameGuideInputs) {
            guideSnapshot = _guideSnapshot;
        } else {
            guideSnapshot = [&]() {
                RIGEXEC_PROFILE_SCOPE_CAT(_profiler, "SolverGuides", "exec");
                return _guideTaps->Evaluate(time, guideOverrides);
            }();
            if (guideSnapshot.IsComplete()) {
                _guideSnapshot = guideSnapshot;
                _guideInputs = guideOverrides;
                _guideTime = time;
                _guideDirty = false;
                _guideTaps->ConsumeDirty();
            } else {
                _guideDirty = true;
            }
        }
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

    // Property-domain results are published directly into
    // pose.movedProperties by _EvaluatePropertyChains (and amended by the
    // post-chain interactive-override pass), so no separate copy is needed.
    // They share the map with the point chains below; a consumer distinguishes
    // them by the type the VtValue holds, not by which mover domain produced
    // them.

    // 3c. THE POSE-INTERPOLATOR PHASE.
    //
    // Here and nowhere else. It reads the FINAL pose -- so it runs after the
    // whole pose walk, every constraint included and the driver constraints
    // in particular -- and it writes floats that the geometry chains below
    // consume, so it runs before them. It is not a mover and cannot be one:
    // a mover's inputs are resolved by the property chains, which run before
    // exec does and therefore cannot see the pose at all.
    _EvaluatePoseInterpolators(time, restFrames, restLive, finalFrames, finalLive, &pose);

    // The independent CPU parity path must consume the same declared
    // provider phase as the graph while resolving it independently.  Capture
    // every matrix provider the graph taps (controls as well as joints), then
    // overlay the evaluator-side frame revisions for final-phase reads.
    std::unordered_map<SdfPath, GfMatrix4d, SdfPath::Hash> baseProviderMatrices;
    std::unordered_map<SdfPath, GfMatrix4d, SdfPath::Hash> finalProviderMatrices;
    // Consumed only by the cpuParityMode _EvaluateChain oracle, so skip
    // populating them on the hot dynamic path.
    if (cpuParityMode) {
        for (const auto &[target, revisions] : _graphChains) {
            for (const _GraphRevision &revision : revisions) {
                if (revision.transformTap >= 0 &&
                    !revision.binding.transform.IsEmpty() &&
                    !baseProviderMatrices.count(revision.binding.transform)) {
                    baseProviderMatrices[revision.binding.transform] =
                        snapshot.Get<GfMatrix4d>(revision.transformTap);
                }
                if (revision.transformSpaceTap >= 0 &&
                    !baseProviderMatrices.count(
                        revision.binding.transformSpace)) {
                    baseProviderMatrices[revision.binding.transformSpace] =
                        snapshot.Get<GfMatrix4d>(revision.transformSpaceTap);
                }
                for (size_t k = 0; k < revision.influenceTaps.size() &&
                                   k < revision.binding.influences.size(); ++k) {
                    const SdfPath &provider = revision.binding.influences[k];
                    if (!baseProviderMatrices.count(provider)) {
                        baseProviderMatrices[provider] =
                            snapshot.Get<GfMatrix4d>(revision.influenceTaps[k]);
                    }
                }
            }
        }
        finalProviderMatrices = baseProviderMatrices;
        for (const auto &[provider, matrix] : finalMatrices) {
            finalProviderMatrices[provider] = matrix;
        }
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
    // Keep topology and computed checkpoints across pulls. Updating a source
    // or packet invalidates only its downstream revisions. The independent
    // CPU reference is available in cpuParityMode for validation.
    // ORDERING, ASSERTED RATHER THAN TRUSTED (second half).
    //
    // WHAT MUST RUN AFTER THE POSE-INTERPOLATOR PHASE: these chains. A
    // RigExecBlendInput's inputs:weight carries a single authored connection
    // to <pose>.outputs:weight, and RigExecResolvedInputs::GetAttribute
    // follows it into the in-memory map -- so a phase that had not run yet
    // would be read as the attribute's AUTHORED zero, with no error raised
    // anywhere and every corrective silently off. An assertion of exactly
    // this kind caught a real inversion on 2026-09-13, where the shapes chain
    // landed after the skin.
    //
    // One map probe per published weight: 121 on the biped, and the whole
    // check measures below the profiler's resolution.
    if (!_poseWeightProperties.empty()) {
        size_t unpublished = 0;
        for (const SdfPath &weight : _poseWeightProperties) {
            if (!_resolvedInputs.Find(weight)) {
                ++unpublished;
            }
        }
        if (!TF_VERIFY(unpublished == 0,
                       "%zu of %zu pose weights were not published before the "
                       "geometry chains; the pose-interpolator phase is out "
                       "of order",
                       unpublished, _poseWeightProperties.size())) {
            pose.diagnostics.push_back(
                std::to_string(unpublished) + " of " +
                std::to_string(_poseWeightProperties.size()) +
                " pose weights were not published before the geometry "
                "chains read them");
        }
    }

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
    // What one chain produced, held apart from the pose until the walk folds
    // it in.
    //
    // Independent chains can run at the same time, and two of them appending
    // to one diagnostics vector or inserting into one map would be a data
    // race -- and, worse, whichever finished first would decide the order the
    // rig reports things in. So a chain writes only into its own buffer and
    // the walk merges the buffers in chain order: what a serial walk wrote,
    // in the order it wrote it, however the tasks were scheduled.
    struct _ChainWork {
        std::vector<std::string> diagnostics;
        std::vector<std::pair<SdfPath, VtValue>> movedProperties;
        std::map<SdfPath, RigExecResolvedWeightField> weightFields;
        std::vector<std::pair<SdfPath, RigExecPointFrame>> controlFrames;
        RigExecChainSnapshots snapshots;
        size_t revisionsCreated = 0;
        size_t revisionsExecuted = 0;
        size_t schedulesBuilt = 0;
        size_t chainsBuilt = 0;
        size_t revisionsBuilt = 0;
    };
    const auto mergeChainWork = [&](_ChainWork &work) {
        for (std::string &message : work.diagnostics) {
            pose.diagnostics.push_back(std::move(message));
        }
        for (auto &[path, value] : work.movedProperties) {
            pose.movedProperties[path] = std::move(value);
        }
        for (auto &[path, field] : work.weightFields) {
            pose.weightFields[path] = std::move(field);
        }
        for (const auto &[path, frame] : work.controlFrames) {
            pose.controlFrames[path] = frame;
        }
        _chainSnapshots.Merge(std::move(work.snapshots));
        pose.moverGraphRevisionsCreated += work.revisionsCreated;
        pose.moverGraphRevisionsExecuted += work.revisionsExecuted;
        pose.moverGraphSchedulesBuilt += work.schedulesBuilt;
        graphChainsBuilt += work.chainsBuilt;
        graphRevisionsBuilt += work.revisionsBuilt;
    };

    // One chain, start to finish. Everything it touches outside `work` is
    // read-only for the duration: the compiled bindings, the exec snapshot
    // this generation extracted, the pose as the constraint walk left it, and
    // its own live graph, which no other chain can reach.
    const auto runChain = [&](const SdfPath &target, _ChainWork &work,
                              UsdGeomXformCache &chainXformCache) {
        // This chain's own records answer first. A phase that names a mover
        // in THIS chain is answered by what the chain has recorded so far,
        // which is in this task's buffer and does not reach the evaluator's
        // store until the walk merges it; everything else -- a provider's
        // frame from the pose walk, another chain from an earlier level --
        // is already there and unchanging while this runs.
        const auto lookupChainSnapshot =
            [&](const SdfPath &path, const RigExecReadPhase &phase,
                const SdfPath &reader) -> const VtValue * {
            if (const VtValue *recorded =
                    work.snapshots.Lookup(path, phase, reader)) {
                return recorded;
            }
            return _chainSnapshots.Lookup(path, phase, reader);
        };
        const auto chainIt = _graphChains.find(target);
        if (chainIt == _graphChains.end()) {
            return;
        }
        const std::vector<_GraphRevision> &revisions = chainIt->second;
        VtVec3fArray basePoints;
        // Compile created a node for every chain and derived target, so this
        // never inserts. An insertion here would be a write into a map the
        // other chains of this level are reading at the same time.
        auto &live = _liveGraphs[target];
        if (live && live->basePointsPushed && live->basePointsStatic) {
            // Already in the source, and an authored base that is not
            // time-varying cannot have moved since: this skips the attribute
            // read and the element-by-element compare inside
            // UpdatePointSource, both of which can only conclude "unchanged".
            // The array itself is still needed below (blend deltas, painted
            // weight fields), and handing it over is a refcount.
            basePoints = live->basePoints;
        } else {
            const UsdAttribute baseAttr = _stage->GetAttributeAtPath(target);
            if (!baseAttr || !baseAttr.Get(&basePoints, time)) {
                return;
            }
            if (live &&
                !live->graph.UpdatePointSource(live->source, basePoints)) {
                // A time-varying point count changes VDF element masks.
                // Replace this target's graph only; independent targets keep
                // their caches.
                live.reset();
            }
            if (!live) {
                live = std::make_unique<_LiveGraph>();
                live->source = live->graph.AddPointSource(target, basePoints);
            }
            live->basePoints = basePoints;
            live->basePointsPushed = true;
            live->basePointsStatic = !baseAttr.ValueMightBeTimeVarying();
        }
        // Match stable operation identities, reconnect surviving nodes, and
        // delete removed nodes. A rebind only updates packets; insertion,
        // removal and reordering rebuild this target's schedule, not its nodes
        // or other targets' checkpoints.
        std::vector<std::pair<SdfPath, RigExecRevisionOp>> identities;
        for (const auto &revision : revisions)
            identities.emplace_back(revision.moverPath, revision.op);
        if (identities != live->identities) {
            std::map<std::pair<SdfPath, RigExecRevisionOp>, VdfMaskedOutput> retained;
            for (size_t i = 0; i < live->identities.size(); ++i)
                retained.emplace(live->identities[i], live->revisions[i]);
            std::vector<VdfMaskedOutput> outputs;
            VdfMaskedOutput previous = live->source;
            for (const auto &identity : identities) {
                const auto found = retained.find(identity);
                VdfMaskedOutput output;
                if (found != retained.end()) {
                    output = found->second;
                    live->graph.ReconnectRevision(output, previous);
                    retained.erase(found);
                } else {
                    output = live->graph.AddRevision(identity.second, previous,
                        RigExecMoverParameters(), RigExecMoverStatus());
                    ++work.revisionsCreated;
                }
                outputs.push_back(output);
                previous = output;
            }
            for (const auto &[identity, output] : retained)
                live->graph.RemoveRevision(output);
            live->identities = std::move(identities);
            live->revisions = std::move(outputs);
        }
        RigExecMoverGraph &graph = live->graph;
        const size_t executionsBefore = graph.GetRevisionExecutionCount();
        const size_t schedulesBefore = graph.GetScheduleBuildCount();
        VdfMaskedOutput head = live->source;
        size_t revisionIndex = 0;
        bool built = true;
        for (const _GraphRevision &revision : revisions) {
            const UsdPrim moverPrim = [&]() {
                RIGEXEC_PROFILE_SCOPE_CAT(_profiler, "GetPrim", "geometry");
                return _stage->GetPrimAtPath(revision.moverPath);
            }();
            if (!moverPrim) {
                built = false;
                break;
            }
            // One overlay per revision: the generation-wide property results,
            // plus whatever THIS revision's declared phases resolve to. The
            // assembler reads inputs by path and never learns a phase exists
            // -- which is what lets a phase apply to any input, including
            // ones added later, without touching the assembler.
            //
            // MEASURED 2026-09-13: declared phases are RARE -- almost every
            // revision has none -- and the copy this used to make
            // unconditionally was then bit-identical to _resolvedInputs, one
            // whole std::map<SdfPath, VtValue> per revision per frame. Copy
            // only when there is actually something to overlay.
            const RigExecResolvedInputs *resolved = &_resolvedInputs;
            std::optional<RigExecResolvedInputs> revisionInputs;
            if (!revision.binding.phases.empty()) {
                revisionInputs = _resolvedInputs;
                resolved = &*revisionInputs;
            }
            for (const auto &[inputPath, phase] : revision.binding.phases) {
                if (const VtValue *v = lookupChainSnapshot(
                        inputPath, phase, revision.moverPath)) {
                    revisionInputs->SetProperty(inputPath, *v);
                } else if (phase.kind != RigExecReadPhaseKind::Preceding) {
                    // Preceding falling through to the stage is correct (the
                    // reader is the chain's first revision, so its preceding
                    // value IS the base). Anything else means the phase named
                    // something that produced nothing.
                    work.diagnostics.push_back(
                        "diag " + revision.moverPath.GetString() +
                        ": read phase '" + phase.GetAsString() + "' for " +
                        inputPath.GetString() +
                        " resolved to nothing; read the authored base");
                }
            }
            RigExecProviderValues values;
            values.resolved = resolved;
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
            if (const auto delta = constraintDeltas.find(revision.moverPath);
                delta != constraintDeltas.end()) {
                transform = delta->second;
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
                if (const VtValue *v = lookupChainSnapshot(
                        revision.binding.transform,
                        revision.binding.transformPhase,
                        revision.moverPath)) {
                    if (v->IsHolding<GfMatrix4d>()) {
                        transform = v->UncheckedGet<GfMatrix4d>();
                        values.transform = &transform;
                    }
                }
            }
            // A space provider: the transform measured against it, read at
            // the same phase, so the points take only the handle's motion
            // inside the space.
            if (values.transform && revision.transformSpaceTap >= 0) {
                GfMatrix4d space =
                    snapshot.Get<GfMatrix4d>(revision.transformSpaceTap);
                if (revision.transformFinalPhase) {
                    const auto revisedIt =
                        finalMatrices.find(revision.binding.transformSpace);
                    if (revisedIt != finalMatrices.end()) {
                        space = revisedIt->second;
                    }
                }
                transform = RigExecMeasureInSpace(transform, space);
            }
            // Skin influences: the phase rules of the single transform
            // above, applied to every rigExec:influences entry in order.
            std::vector<GfMatrix4d> influenceTransforms;
            if (!revision.binding.influences.empty()) {
                influenceTransforms.reserve(revision.binding.influences.size());
                for (size_t k = 0; k < revision.binding.influences.size(); ++k) {
                    const SdfPath &provider = revision.binding.influences[k];
                    GfMatrix4d m(1.0);
                    if (k < revision.influenceTaps.size() &&
                        revision.influenceTaps[k] >= 0) {
                        m = snapshot.Get<GfMatrix4d>(revision.influenceTaps[k]);
                    }
                    if (revision.transformFinalPhase) {
                        const auto revisedIt = finalMatrices.find(provider);
                        if (revisedIt != finalMatrices.end()) {
                            m = revisedIt->second;
                        }
                    } else if (revision.binding.transformPhase.kind ==
                               RigExecReadPhaseKind::AtPrim) {
                        if (const VtValue *v = lookupChainSnapshot(
                                provider, revision.binding.transformPhase,
                                revision.moverPath)) {
                            if (v->IsHolding<GfMatrix4d>()) {
                                m = v->UncheckedGet<GfMatrix4d>();
                            }
                        }
                    }
                    influenceTransforms.push_back(m);
                }
                values.influenceTransforms = &influenceTransforms;
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
                    RIGEXEC_PROFILE_SCOPE_CAT(
                        _profiler, "CurrentPhaseEvaluate", "geometry");
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
                        work.diagnostics.push_back(
                            "current-phase weight failed: " + weightError);
                    }
                }
                if (weights.valid && _publishWeightFields) {
                    RigExecResolvedWeightField &field =
                        work.weightFields[revision.binding.weightObject];
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
                    // ResolveAll matches the kernel: O(n+m) scatter for
                    // sparse packets, range-policy checks, atomic on
                    // failure -- the same values the mover consumed.
                    if (!weights.ResolveAll(logicalCount, &field.weights)) {
                        field.weights.clear();
                    }
                }
            }
            // basePoints is the chain-wide authored base, invariant across
            // revisions. Only the ops whose assemble path reads
            // values.basePoints (and the blend-input path below) need it;
            // every other revision would pay a full base copy for nothing.
            const bool needsBasePoints =
                !revision.binding.blendInputs.empty() ||
                revision.op == RigExecRevisionOp::BlendShape ||
                revision.op == RigExecRevisionOp::VolumeCorrect ||
                revision.op == RigExecRevisionOp::Lattice ||
                revision.op == RigExecRevisionOp::RecomputeNormals ||
                revision.op == RigExecRevisionOp::RecomputeExtent;
            if (needsBasePoints) {
                values.basePoints.assign(basePoints.begin(), basePoints.end());
            }
            if (revision.op == RigExecRevisionOp::Skin &&
                revision.skinTopologyFixed) {
                values.skinTopologyCache = &_skinTopologies;
            }
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
            if (!revision.binding.blendInputs.empty()) {
                std::vector<RigExecBlendChannel> channels;
                for (const SdfPath &input : revision.binding.blendInputs) {
                    RigExecBlendChannel channel;
                    const UsdPrim inputPrim = _stage->GetPrimAtPath(input);
                    _resolvedInputs.GetAttribute(inputPrim.GetAttribute(TfToken("inputs:weight")),
                                                 time, &channel.weight);
                    const auto sampleBindings = revision.binding.blendSamples.find(input);
                    if (sampleBindings != revision.binding.blendSamples.end()) {
                        for (const auto &binding : sampleBindings->second) {
                            RigExecBlendSampleData sample;
                            const UsdPrim samplePrim = _stage->GetPrimAtPath(binding.sample);
                            _resolvedInputs.GetAttribute(samplePrim.GetAttribute(TfToken("rigExec:activation")),
                                                         time, &sample.activation);
                            if (!binding.blendShape.IsEmpty()) {
                                // Sparse: the shape is epoch-constant, so it
                                // is resolved once and shared by pointer.
                                // This is the whole reason the relationship
                                // exists -- the dense branch below reads and
                                // copies 26,276 points per sample per frame
                                // whether the channel is at 0 or at 1.
                                sample.layout = _blendSampleShapes.Resolve(
                                    binding.sample,
                                    [&](RigExecBlendSampleLayout *layout) {
                                        return _ResolveBlendSampleLayout(
                                            binding.blendShape,
                                            values.basePoints.size(),
                                            layout);
                                    });
                                if (!sample.layout) {
                                    // Refused the cache: something about the
                                    // shape can move inside this epoch, so it
                                    // is read per frame instead.
                                    auto perFrame = std::make_shared<
                                        RigExecBlendSampleLayout>();
                                    _ResolveBlendSampleLayout(
                                        binding.blendShape,
                                        values.basePoints.size(),
                                        perFrame.get());
                                    sample.layout = perFrame;
                                }
                                channel.samples.push_back(std::move(sample));
                                continue;
                            }
                            VtVec3fArray points;
                            const VtValue *phased = lookupChainSnapshot(
                                binding.points, binding.phase, revision.moverPath);
                            if (phased && phased->IsHolding<VtVec3fArray>()) {
                                points = phased->UncheckedGet<VtVec3fArray>();
                            } else {
                                _resolvedInputs.GetAttribute(_stage->GetAttributeAtPath(binding.points),
                                                             time, &points);
                            }
                            sample.points.assign(points.begin(), points.end());
                            channel.samples.push_back(std::move(sample));
                        }
                    }
                    std::stable_sort(channel.samples.begin(), channel.samples.end(),
                        [](const auto &a, const auto &b) { return a.activation < b.activation; });
                    channels.push_back(std::move(channel));
                }
                // A structural failure leaves blendDeltas empty, which is
                // what makes the assembled packet invalid -- the same atomic
                // MoverFailed pass-through the kernel produces.
                if (!RigExecSumBlendChannels(channels, values.basePoints,
                                             &values.blendDeltas)) {
                    values.blendDeltas.clear();
                }
            }

            const RigExecMoverParameters parameters = [&]() {
                RIGEXEC_PROFILE_SCOPE_CAT(
                    _profiler,
                    "Assemble " + revision.moverPath.GetName(), "geometry");
                return RigExecAssembleParameters(moverPrim, revision.op,
                                                 revision.binding, values,
                                                 time);
            }();
            if (parameters.enabled && !parameters.valid &&
                revision.binding.weightObject.IsEmpty()) {
                const float scalar = _ResolvedRead(
                    _resolvedInputs, moverPrim, "inputs:defaultWeight",
                    1.0f, time);
                if (!std::isfinite(scalar) || scalar < 0.0f ||
                    scalar > 1.0f) {
                    work.diagnostics.push_back(
                        "MoverFailed " + revision.moverPath.GetString() +
                        ": inputs:defaultWeight must be finite and in "
                        "[0, 1]; revision passed through");
                }
            } else if (parameters.enabled && !parameters.valid &&
                       !revision.binding.weightObject.IsEmpty() &&
                       (!values.weights || !values.weights->valid)) {
                work.diagnostics.push_back(
                    "MoverFailed " + revision.moverPath.GetString() +
                    ": rigExec:weightObject produced an invalid common "
                    "envelope; revision passed through");
            }
            head = live->revisions[revisionIndex++];
            {
                RIGEXEC_PROFILE_SCOPE_CAT(_profiler, "UpdateRev", "geometry");
                graph.UpdateRevision(
                    head, parameters,
                    RigExecStatusForParameters(parameters, revision.moverPath));
            }
            ++work.revisionsBuilt;

            // Snapshot only where a phased read named this revision. The
            // compile pass reduced every phase to one revision, so this is
            // the whole cost of the feature for a rig that uses it, and
            // nothing at all for one that does not.
            const auto wanted = _snapshotPoints.find(target);
            if (wanted != _snapshotPoints.end() &&
                wanted->second.count(revision.moverPath)) {
                RIGEXEC_PROFILE_SCOPE_CAT(_profiler, "SnapshotEvaluate", "geometry");
                work.snapshots.Record(target, revision.moverPath,
                                       VtValue(graph.Evaluate(head)));
            }
        }
        if (!built) {
            return;
        }

        const VtVec3fArray graphPoints = [&]() {
            RIGEXEC_PROFILE_SCOPE_CAT(
                _profiler, "GraphEvaluate", "geometry");
            return graph.Evaluate(head);
        }();
        for (size_t i = 0; i < live->revisions.size(); ++i) {
            if (graph.GetRevisionStatus(live->revisions[i]).state == "moverFailed") {
                work.diagnostics.push_back("MoverFailed " + revisions[i].moverPath.GetString() +
                    ": execution rejected its inputs; revision passed through");
            }
            if (revisions[i].op == RigExecRevisionOp::CurvenetAdjuster &&
                graph.GetRevisionStatus(live->revisions[i]).AllowsApply()) {
                const auto frames = graph.GetRevisionControlFrames(live->revisions[i]);
                const auto paths = RigExecCurvenetAdjustmentPaths(
                    _stage->GetPrimAtPath(revisions[i].moverPath));
                // Kernels operate on net-local points. Control outputs use
                // the same asset-space contract as ordinary rig controls.
                GfMatrix4d netToAsset(1.0);
                for (UsdPrim prim = _stage->GetPrimAtPath(target.GetPrimPath());
                     prim && !prim.IsPseudoRoot() && prim != assetRoot;
                     prim = prim.GetParent()) {
                    const UsdGeomXformable xform(prim);
                    if (!xform) continue;
                    GfMatrix4d local(1.0);
                    bool reset = false;
                    xform.GetLocalTransformation(&local, &reset, time);
                    const auto driven = pose.providerXforms.find(prim.GetPath());
                    if (driven != pose.providerXforms.end()) local = driven->second;
                    netToAsset = netToAsset * local;
                    if (reset) {
                        netToAsset = netToAsset *
                            chainXformCache.GetLocalToWorldTransform(assetRoot).GetInverse();
                        break;
                    }
                }
                for (size_t j = 0; j < std::min(frames.size(), paths.size()); ++j) {
                    work.controlFrames.emplace_back(
                        paths[j],
                        RigExecFrameFromMatrix(frames[j] * netToAsset));
                }
            }
        }
        work.revisionsExecuted +=
            graph.GetRevisionExecutionCount() - executionsBefore;
        work.schedulesBuilt +=
            graph.GetScheduleBuildCount() - schedulesBefore;
        work.movedProperties.emplace_back(target, VtValue(graphPoints));
        // `final` costs nothing extra: this is the value the chain publishes.
        work.snapshots.RecordFinal(target, VtValue(graphPoints));
        ++work.chainsBuilt;

        // Derived maintenance reads this chain's final points, which is why it
        // runs here rather than as another entry in _graphChains.
        const auto derivedIt = _graphDerivedChains.find(target);
        if (derivedIt == _graphDerivedChains.end()) {
            return;
        }
        RIGEXEC_PROFILE_SCOPE_CAT(
            _profiler, "Derived " + target.GetString(), "geometry");
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

            const RigExecMoverParameters parameters = [&]() {
                RIGEXEC_PROFILE_SCOPE_CAT(
                    _profiler,
                    "AssembleDerived " + derived.target.GetName(),
                    "geometry");
                return RigExecAssembleParameters(
                    _stage->GetPrimAtPath(derived.moverPath), derived.op,
                    derived.binding, values, time);
            }();
            // The recompute is a pure function of the assembled inputs: an
            // unchanged tuple republishes the stored result and defers the
            // derived graph entirely.
            auto &deferred = _derivedCache[derived.target];
            if (deferred.cached &&
                parameters.auxPoints == deferred.points &&
                derivedBase == deferred.base &&
                parameters.topologyCounts == deferred.topologyCounts &&
                parameters.topologyIndices == deferred.topologyIndices &&
                parameters.widths == deferred.widths) {
                work.movedProperties.emplace_back(
                    derived.target, VtValue(deferred.result));
                // A republished revision is still a revision the chain
                // holds; the count is what the walk reports.
                ++work.revisionsBuilt;
                ++work.chainsBuilt;
                continue;
            }
            // Pre-created at Compile with every other chain node.
            auto &derivedLive = _liveGraphs[derived.target];
            if (derivedLive && !derivedLive->graph.UpdatePointSource(
                    derivedLive->source, derivedBase)) {
                derivedLive.reset();
            }
            if (!derivedLive) {
                derivedLive = std::make_unique<_LiveGraph>();
                derivedLive->source = derivedLive->graph.AddPointSource(
                    derived.target, derivedBase);
                derivedLive->revisions.push_back(
                    derivedLive->graph.AddRevision(
                        derived.op, derivedLive->source, parameters,
                        RigExecStatusForParameters(parameters,
                                                   derived.moverPath)));
                ++work.revisionsCreated;
            }
            RigExecMoverGraph &derivedGraph = derivedLive->graph;
            const size_t derivedExecutions =
                derivedGraph.GetRevisionExecutionCount();
            const size_t derivedSchedules = derivedGraph.GetScheduleBuildCount();
            const VdfMaskedOutput &derivedHead = derivedLive->revisions.front();
            derivedGraph.UpdateRevision(
                derivedHead, parameters,
                RigExecStatusForParameters(parameters, derived.moverPath));
            ++work.revisionsBuilt;

            const VtVec3fArray derivedResult = [&]() {
                RIGEXEC_PROFILE_SCOPE_CAT(
                    _profiler, "DerivedEvaluate", "geometry");
                return derivedGraph.Evaluate(derivedHead);
            }();
            if (derivedGraph.GetRevisionStatus(derivedHead).state == "moverFailed") {
                deferred.cached = false;
                work.diagnostics.push_back("MoverFailed " + derived.target.GetString() +
                    ": derived geometry input/cardinality validation failed");
            } else {
                deferred.points = parameters.auxPoints;
                deferred.base = derivedBase;
                deferred.topologyCounts = parameters.topologyCounts;
                deferred.topologyIndices = parameters.topologyIndices;
                deferred.widths = parameters.widths;
                deferred.result = derivedResult;
                deferred.cached = true;
            }
            work.revisionsExecuted +=
                derivedGraph.GetRevisionExecutionCount() - derivedExecutions;
            work.schedulesBuilt +=
                derivedGraph.GetScheduleBuildCount() - derivedSchedules;
            work.movedProperties.emplace_back(derived.target, VtValue(derivedResult));
            ++work.chainsBuilt;
        }
    };

    // Level by level, and inside a level one task per chain wherever Compile
    // classified that as safe. A level that is not -- and every level when
    // the kill switch is off -- is walked in order on this thread, which is
    // what the whole walk was before levels existed.
    const auto runChainHere = [&](const SdfPath &target) {
        _ChainWork work;
        runChain(target, work, constraintXformCache);
        mergeChainWork(work);
    };
    if (_chainLevels.empty()) {
        // No partition at all -- a rig with no point chains. The chain order
        // is then the whole walk, and it is empty too.
        for (const SdfPath &target : _chainOrder) {
            runChainHere(target);
        }
    }
    for (const _ChainLevel &level : _chainLevels) {
        if (!level.parallel || !RigExecParallelEvaluationEnabled()) {
            for (const SdfPath &target : level.targets) {
                runChainHere(target);
            }
            continue;
        }
        RIGEXEC_PROFILE_SCOPE_CAT(_profiler, "ChainLevel", "geometry");
        std::vector<_ChainWork> levelWork(level.targets.size());
        WorkWithScopedParallelism([&]() {
            WorkDispatcher dispatcher;
            for (size_t i = 0; i < level.targets.size(); ++i) {
                dispatcher.Run([&, i]() {
                    // Its own transform cache: the cache memoizes, so one
                    // shared between tasks would be a shared mutable map, and
                    // what it memoizes is a stage read each task can make for
                    // itself.
                    UsdGeomXformCache taskXformCache(time);
                    runChain(level.targets[i], levelWork[i], taskXformCache);
                });
            }
            // The dispatcher joins on the way out of this scope, so nothing
            // below reads a buffer a task is still writing.
        });
        for (_ChainWork &work : levelWork) {
            mergeChainWork(work);
        }
    }
    // The same chains against the scalar CPU reference (spec §7.4). This is
    // the oracle that OUTLIVES the generated-prim chains: it resolves every
    // input off the authored stage itself and authors nothing, so it survives
    // the compiler's deletion, and it is a genuinely independent
    // implementation -- it does not call RigExecAssembleParameters, which is
    // why it can catch a packet-assembly drift rather than share one.
    size_t parityAgreements = 0;
    if (cpuParityMode) {
        RIGEXEC_PROFILE_SCOPE_CAT(_profiler, "Parity", "parity");
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
            RIGEXEC_PROFILE_SCOPE_CAT(
                _profiler, "ParityChain " + target.GetString(), "parity");
            std::vector<std::string> quiet;
            const VtVec3fArray reference =
                _EvaluateChain(
                    target, chain, pose, baseProviderMatrices,
                    finalProviderMatrices, time, &quiet, constraintDeltas);
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
    if (cpuParityMode) {
        pose.diagnostics.push_back(
            "mover graph parity: " + std::to_string(parityAgreements) +
            " point chain(s) agreed, " +
            std::to_string(pose.moverGraphParityMismatches) + " mismatched");
    }
    pose.diagnostics.push_back(
        "mover graph: " + std::to_string(graphChainsBuilt) +
        " chain(s), " + std::to_string(graphRevisionsBuilt) +
        " revision(s); " + std::to_string(pose.moverGraphRevisionsCreated) +
        " created, " + std::to_string(pose.moverGraphRevisionsExecuted) +
        " executed, " + std::to_string(pose.moverGraphSchedulesBuilt) +
        " schedule(s) built");
    if (pose.moverGraphParityMismatches != 0) {
        pose.diagnostics.push_back(
            "mover graph parity failed; refusing to publish generation");
        return pose;  // pose.valid remains false
    }
    // Optional CPU reference-kernel parity for the lowered chains
    // (scalar-reference goldens, spec §7.4).
    if (cpuParityMode) {
        RIGEXEC_PROFILE_SCOPE_CAT(_profiler, "ParityPublish", "parity");
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
            pose.movedPropertiesCpu[target] = VtValue(_EvaluateChain(
                target, chain, pose, baseProviderMatrices,
                finalProviderMatrices, time, &pose.diagnostics, constraintDeltas));
        }
    }

    pose.valid = true;
    return pose;
}

size_t
RigExecRigEvaluator::GetBakedClusterCount() const
{
    return _bakedProgram ? _bakedProgram->GetClusterCount() : 0;
}

size_t
RigExecRigEvaluator::GetBakedClustersRunLastGeneration() const
{
    return _bakedProgram ? _bakedProgram->GetClustersRunLastGeneration() : 0;
}

const std::vector<TfToken> &
RigExecRigEvaluator::GetConstraintOperatorTypeNames()
{
    // Built from the table itself rather than written out again, so an
    // operator added there is named here without anyone remembering to.
    static const std::vector<TfToken> names = [] {
        std::vector<TfToken> types;
        types.reserve(_ConstraintHandlers().size());
        for (const _ConstraintHandler &handler : _ConstraintHandlers()) {
            types.push_back(TfToken(handler.schemaType));
        }
        return types;
    }();
    return names;
}

bool
RigExecRigEvaluator::IsConstraintOperatorType(const TfToken &schemaType)
{
    return _FindConstraintHandler(schemaType) != nullptr;
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
