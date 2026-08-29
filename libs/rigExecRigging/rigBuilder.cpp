//
// RigExec rigging API implementation. See rigBuilder.h for the contract.
//
#include "rigBuilder.h"

#include "pxr/base/gf/vec3d.h"
#include "pxr/base/gf/vec3i.h"
#include "pxr/base/vt/array.h"
#include "pxr/usd/sdf/layer.h"
#include "pxr/usd/sdf/schema.h"
#include "pxr/usd/usd/attribute.h"
#include "pxr/usd/usd/relationship.h"
#include "pxr/usd/usd/timeCode.h"

#include <algorithm>
#include <stdexcept>

PXR_NAMESPACE_USING_DIRECTIVE

namespace rigExec {

namespace {

const TfToken _kControlApi("RigExecControlAPI");
const TfToken _kMoverApi("RigExecMoverAPI");
const TfToken _kNodeGraphApi("NodeGraphNodeAPI");

/// Applies a named API best-effort: the schema type must be registered (the
/// rigExecSchema resource plugin loaded) for ApplyAPI to succeed, and an
/// unregistered name would log a TF error. The engine detects movers by prim
/// type name plus relationships, so skip silently when not yet registered.
void _ApplyApiBestEffort(const UsdPrim &prim, const TfToken &apiName) {
    const TfType type = TfType::FindByName(apiName);
    if (!type.IsUnknown()) {
        prim.ApplyAPI(type);
    }
}

/// Author (or overwrite) one attribute on \p prim with an EXPLICIT Sdf type
/// name, matching the codeless schema declaration exactly. The engine reads
/// every property through typed UsdAttribute::Get calls, so authoring a
/// `token` as a string or a `float[]` as a double array would still compose
// but would not round-trip cleanly through the schema's declared types.
UsdAttribute
_AuthorAttr(
    const UsdPrim &prim, const char *name, const SdfValueTypeName &typeName,
    VtValue value)
{
    if (!prim.IsValid()) {
        throw std::runtime_error(std::string("invalid prim for attribute ") + name);
    }
    UsdAttribute attr = prim.GetAttribute(TfToken(name));
    if (!attr) {
        attr = prim.CreateAttribute(
            TfToken(name), typeName, false /* uniform */);
    }
    if (!attr) {
        throw std::runtime_error(std::string("failed to create attribute ") + name);
    }
    // UsdAttribute::Set returns false when the payload does not match the
    // declared type (e.g. a raw std::vector where a VtArray is required).
    // Swallowing that would author nothing and fail silently downstream.
    if (!attr.Set(value)) {
        throw std::runtime_error(
            std::string("failed to set attribute ") + name +
            " (value type mismatch for declared type " +
            typeName.GetAsToken().GetString() + ")");
    }
    return attr;
}

/// Append \p target to an ordered relationship, preserving existing targets.
void
_AppendRelTarget(const UsdPrim &prim, const char *name, const SdfPath &target)
{
    if (!prim.IsValid()) {
        throw std::runtime_error(std::string("invalid prim for relationship ") + name);
    }
    UsdRelationship rel = prim.CreateRelationship(TfToken(name));
    if (!rel) {
        throw std::runtime_error(
            std::string("failed to create relationship ") + name);
    }
    SdfPathVector targets;
    rel.GetTargets(&targets);
    targets.push_back(target);
    rel.SetTargets(targets);
}

double
_Clamp01(double x) { return std::min(1.0, std::max(0.0, x)); }

/// Resolve an explicit Sdf type name string ("float", "token", ...) to the
/// stage's schema value-type-name object. Throws on unknown names so a typo in
/// a SetAttr call fails loudly instead of authoring the wrong type.
SdfValueTypeName
_ResolveType(const UsdStageRefPtr &stage, const TfToken &name)
{
    if (!stage || !stage->GetRootLayer()) {
        throw std::runtime_error("no stage for type name resolution");
    }
    SdfValueTypeName t = stage->GetRootLayer()->GetSchema().FindType(name);
    if (t.GetType().IsUnknown()) {
        throw std::runtime_error("unknown Sdf type name: " + name.GetString());
    }
    return t;
}

}  // namespace

// ---------------------------------------------------------------------------
// Handle base
// ---------------------------------------------------------------------------

void
RigExecHandleBase::SetAttr(
    const char *name, const TfToken &typeName, VtValue value)
{
    _AuthorAttr(GetPrim(), name, _ResolveType(_stage, typeName), std::move(value));
}

void
RigExecHandleBase::SetRel(
    const char *name, const std::vector<SdfPath> &targets)
{
    if (!IsValid()) {
        throw std::runtime_error(std::string("invalid handle for relationship ") + name);
    }
    UsdRelationship rel = GetPrim().CreateRelationship(TfToken(name));
    if (!rel) {
        throw std::runtime_error(
            std::string("failed to create relationship ") + name);
    }
    rel.SetTargets(targets);
}

void
RigExecHandleBase::ApplyApi(const TfToken &apiSchemaName)
{
    if (!IsValid()) {
        throw std::runtime_error("invalid handle for API application");
    }
    // Applying a named API requires the schema type to be registered, which
    // happens when the rigExecSchema resource plugin is loaded. The engine
    // detects movers by prim type name plus rigExec:moves, so this is
    // best-effort: skip silently when the plugin is not loaded yet.
    const TfType type = TfType::FindByName(apiSchemaName);
    if (!type.IsUnknown()) {
        GetPrim().ApplyAPI(type);
    }
}

// ---------------------------------------------------------------------------
// Xformable-backed handles (control, joint, volume weights)
// ---------------------------------------------------------------------------

namespace {

void
_SetRestSpace(RigExecHandleBase *self, const GfMatrix4d &m)
{
    _AuthorAttr(
        self->GetPrim(), "rest:space", SdfValueTypeNames->Matrix4d, VtValue(m));
}

void
_SetAvarTranslation(RigExecHandleBase *self, double tx, double ty, double tz)
{
    _AuthorAttr(self->GetPrim(), "avars:tx", SdfValueTypeNames->Double, VtValue(tx));
    _AuthorAttr(self->GetPrim(), "avars:ty", SdfValueTypeNames->Double, VtValue(ty));
    _AuthorAttr(self->GetPrim(), "avars:tz", SdfValueTypeNames->Double, VtValue(tz));
}

void
_SetAvarRotation(
    RigExecHandleBase *self, double rx, double ry, double rz,
    const TfToken &order)
{
    _AuthorAttr(self->GetPrim(), "avars:rx", SdfValueTypeNames->Double, VtValue(rx));
    _AuthorAttr(self->GetPrim(), "avars:ry", SdfValueTypeNames->Double, VtValue(ry));
    _AuthorAttr(self->GetPrim(), "avars:rz", SdfValueTypeNames->Double, VtValue(rz));
    if (!order.IsEmpty()) {
        _AuthorAttr(
            self->GetPrim(), "avars:rotationOrder", SdfValueTypeNames->Token,
            VtValue(order));
    }
}

}  // namespace

void RigExecControlHandle::SetRestSpace(const GfMatrix4d &m)
{ _SetRestSpace(this, m); }
void RigExecControlHandle::SetAvarTranslation(double tx, double ty, double tz)
{ _SetAvarTranslation(this, tx, ty, tz); }
void RigExecControlHandle::SetAvarRotation(
    double rx, double ry, double rz, const TfToken &order)
{ _SetAvarRotation(this, rx, ry, rz, order); }

void
RigExecControlHandle::SetChannelRole(const TfToken &role)
{
    _AuthorAttr(
        GetPrim(), "rigExec:channelRole", SdfValueTypeNames->Token, VtValue(role));
}

void RigExecJointHandle::SetRestSpace(const GfMatrix4d &m)
{ _SetRestSpace(this, m); }
void RigExecJointHandle::SetAvarTranslation(double tx, double ty, double tz)
{ _SetAvarTranslation(this, tx, ty, tz); }
void RigExecJointHandle::SetAvarRotation(
    double rx, double ry, double rz, const TfToken &order)
{ _SetAvarRotation(this, rx, ry, rz, order); }

// ---------------------------------------------------------------------------
// Solvers
// ---------------------------------------------------------------------------

void
RigExecSolverHandle::SetJoints(const std::vector<RigExecJointHandle> &joints)
{
    std::vector<SdfPath> paths;
    paths.reserve(joints.size());
    for (const auto &j : joints) {
        if (!j.IsValid()) {
            throw std::invalid_argument("SetJoints: invalid joint handle");
        }
        paths.push_back(j.GetPath());
    }
    SetRel("rigExec:joints", paths);
}

void
RigExecSolverHandle::SetJoints(const std::vector<SdfPath> &paths)
{
    SetRel("rigExec:joints", paths);
}

void
RigExecSolverHandle::_SetSingleRel(const char *name, const SdfPath &target)
{
    if (target.IsEmpty()) {
        return;  // optional wiring: leave untouched
    }
    SetRel(name, { target });
}

void
RigExecFkChainHandle::SetControls(
    const std::vector<RigExecControlHandle> &controls)
{
    std::vector<SdfPath> paths;
    paths.reserve(controls.size());
    for (const auto &c : controls) {
        if (!c.IsValid()) {
            throw std::invalid_argument("SetControls: invalid control handle");
        }
        paths.push_back(c.GetPath());
    }
    SetRel("rigExec:controls", paths);
}

void
RigExecFkChainHandle::SetControls(const std::vector<SdfPath> &paths)
{
    SetRel("rigExec:controls", paths);
}

void RigExecTwoBoneIkHandle::SetRootControl(const SdfPath &path)
{ _SetSingleRel("rigExec:rootControl", path); }
void RigExecTwoBoneIkHandle::SetEffectorControl(const SdfPath &path)
{ _SetSingleRel("rigExec:effectorControl", path); }
void RigExecTwoBoneIkHandle::SetPoleControl(const SdfPath &path)
{ _SetSingleRel("rigExec:poleControl", path); }

void
RigExecTwoBoneIkHandle::SetStretchPolicy(const TfToken &policy)
{
    _AuthorAttr(
        GetPrim(), "rigExec:stretchPolicy", SdfValueTypeNames->Token, VtValue(policy));
}

void
RigExecTwoBoneIkHandle::SetUnreachablePolicy(const TfToken &policy)
{
    _AuthorAttr(
        GetPrim(), "rigExec:unreachablePolicy", SdfValueTypeNames->Token,
        VtValue(policy));
}

void RigExecBlendPointFramesHandle::SetInputA(const SdfPath &path)
{ _SetSingleRel("rigExec:inputA", path); }
void RigExecBlendPointFramesHandle::SetInputB(const SdfPath &path)
{ _SetSingleRel("rigExec:inputB", path); }

void
RigExecBlendPointFramesHandle::SetWeight(float weight)
{
    _AuthorAttr(
        GetPrim(), "inputs:weight", SdfValueTypeNames->Float, VtValue(weight));
}

void
RigExecBlendPointFramesHandle::SetRotationBlend(const TfToken &mode)
{
    _AuthorAttr(
        GetPrim(), "rigExec:rotationBlend", SdfValueTypeNames->Token, VtValue(mode));
}

void
RigExecBlendPointFramesHandle::SetScaleBlend(const TfToken &mode)
{
    _AuthorAttr(
        GetPrim(), "rigExec:scaleBlend", SdfValueTypeNames->Token, VtValue(mode));
}

void RigExecTwistDistributionHandle::SetStart(const SdfPath &path)
{ _SetSingleRel("rigExec:start", path); }
void RigExecTwistDistributionHandle::SetEnd(const SdfPath &path)
{ _SetSingleRel("rigExec:end", path); }

void
RigExecTwistDistributionHandle::SetCount(int count)
{
    _AuthorAttr(
        GetPrim(), "rigExec:count", SdfValueTypeNames->Int, VtValue(count));
}

void
RigExecTwistDistributionHandle::SetWeights(const std::vector<float> &weights)
{
    _AuthorAttr(
        GetPrim(), "rigExec:weights", SdfValueTypeNames->FloatArray,
        VtValue(VtFloatArray(weights.begin(), weights.end())));
}

void
RigExecTwistDistributionHandle::SetDistribution(const TfToken &mode)
{
    _AuthorAttr(
        GetPrim(), "rigExec:distribution", SdfValueTypeNames->Token, VtValue(mode));
}

void
RigExecTwistDistributionHandle::SetJointElements(const std::vector<int> &elements)
{
    // Explicit VtIntArray payload: a raw std::vector is not accepted by this
    // USD build's VtValue for array attributes.
    _AuthorAttr(
        GetPrim(), "rigExec:jointElements", SdfValueTypeNames->IntArray,
        VtValue(VtIntArray(elements.begin(), elements.end())));
}

void RigExecRibbonHandle::SetDriverCurve(const SdfPath &path)
{ _SetSingleRel("rigExec:driverCurve", path); }
void RigExecRibbonHandle::SetStartFrame(const SdfPath &path)
{ _SetSingleRel("rigExec:startFrame", path); }
void RigExecRibbonHandle::SetEndFrame(const SdfPath &path)
{ _SetSingleRel("rigExec:endFrame", path); }

void
RigExecRibbonHandle::SetTwistFrames(const std::vector<SdfPath> &paths)
{
    SetRel("rigExec:twistFrames", paths);
}

void
RigExecRibbonHandle::SetSampleCount(int count)
{
    _AuthorAttr(
        GetPrim(), "rigExec:sampleCount", SdfValueTypeNames->Int, VtValue(count));
}

void
RigExecRibbonHandle::SetParameterization(const TfToken &mode)
{
    _AuthorAttr(
        GetPrim(), "rigExec:parameterization", SdfValueTypeNames->Token,
        VtValue(mode));
}

void
RigExecRibbonHandle::SetDriverCurveReadPhase(const TfToken &phase)
{
    _AuthorAttr(
        GetPrim(), "rigExec:driverCurveReadPhase", SdfValueTypeNames->Token,
        VtValue(phase));
}

void
RigExecRibbonHandle::SetSurfaceReadPhase(const TfToken &phase)
{
    _AuthorAttr(
        GetPrim(), "rigExec:surfaceReadPhase", SdfValueTypeNames->Token,
        VtValue(phase));
}

void
RigExecRibbonHandle::SetJointElements(const std::vector<int> &elements)
{
    // Explicit VtIntArray payload (see TwistDistribution above).
    _AuthorAttr(
        GetPrim(), "rigExec:jointElements", SdfValueTypeNames->IntArray,
        VtValue(VtIntArray(elements.begin(), elements.end())));
}

// ---------------------------------------------------------------------------
// Constraints
// ---------------------------------------------------------------------------

void
RigExecConstraintHandle::SetTarget(const SdfPath &target)
{
    if (target.IsEmpty()) {
        throw std::invalid_argument("constraint target must not be empty");
    }
    SetRel("rigExec:moves", { target });
}

void
RigExecConstraintHandle::SetDefaultWeight(float weight)
{
    _AuthorAttr(
        GetPrim(), "inputs:defaultWeight", SdfValueTypeNames->Float, VtValue(weight));
}

void
RigExecConstraintHandle::SetWeightObject(const SdfPath &path)
{
    if (path.IsEmpty()) {
        return;
    }
    SetRel("rigExec:weightObject", { path });
}

void
RigExecConstraintHandle::SetTranslationOffset(double x, double y, double z)
{
    _AuthorAttr(
        GetPrim(), "inputs:translationOffset", SdfValueTypeNames->Double3,
        VtValue(GfVec3d(x, y, z)));
}

void
RigExecConstraintHandle::SetRotationOffset(double x, double y, double z)
{
    _AuthorAttr(
        GetPrim(), "inputs:rotationOffset", SdfValueTypeNames->Double3,
        VtValue(GfVec3d(x, y, z)));
}

void
RigExecConstraintHandle::SetScaleOffset(double x, double y, double z)
{
    _AuthorAttr(
        GetPrim(), "inputs:scaleOffset", SdfValueTypeNames->Double3,
        VtValue(GfVec3d(x, y, z)));
}

// The parent constraint is the one operator that reads PER-SOURCE offset
// arrays (double3[] inputs:translationOffsets / rotationOffsets, parallel to
// rigExec:sources). The inherited scalar setters above author different
// attributes and have no effect on a parent constraint.
void
RigExecParentConstraintHandle::SetTranslationOffsets(
    const std::vector<GfVec3d> &offsets)
{
    // The schema declares double3[]; author the exact declared type.
    _AuthorAttr(
        GetPrim(), "inputs:translationOffsets", SdfValueTypeNames->Double3Array,
        VtValue(VtArray<GfVec3d>(offsets.begin(), offsets.end())));
}

void
RigExecParentConstraintHandle::SetRotationOffsets(
    const std::vector<GfVec3d> &degrees)
{
    _AuthorAttr(
        GetPrim(), "inputs:rotationOffsets", SdfValueTypeNames->Double3Array,
        VtValue(VtArray<GfVec3d>(degrees.begin(), degrees.end())));
}

void
RigExecConstraintHandle::_SetSingleRel(const char *name, const SdfPath &target)
{
    if (target.IsEmpty()) {
        return;
    }
    SetRel(name, { target });
}

void
RigExecSourceConstraintHandle::SetSources(const std::vector<SdfPath> &paths)
{
    SetRel("rigExec:sources", paths);
}

void
RigExecSourceConstraintHandle::SetSources(
    const std::vector<SdfPath> &paths, const std::vector<float> &weights)
{
    if (!weights.empty() && weights.size() != paths.size()) {
        throw std::invalid_argument(
            "sourceWeights must be empty or exactly one entry per source");
    }
    SetRel("rigExec:sources", paths);
    if (!weights.empty()) {
        _AuthorAttr(
            GetPrim(), "inputs:sourceWeights", SdfValueTypeNames->FloatArray,
            VtValue(VtFloatArray(weights.begin(), weights.end())));
    }
}

void
RigExecAimConstraintHandle::SetAimVector(double x, double y, double z)
{
    _AuthorAttr(
        GetPrim(), "inputs:aimVector", SdfValueTypeNames->Double3,
        VtValue(GfVec3d(x, y, z)));
}

void
RigExecAimConstraintHandle::SetUpVector(double x, double y, double z)
{
    _AuthorAttr(
        GetPrim(), "inputs:upVector", SdfValueTypeNames->Double3,
        VtValue(GfVec3d(x, y, z)));
}

void
RigExecAimConstraintHandle::SetAimTarget(const SdfPath &path)
{
    _SetSingleRel("rigExec:aimTarget", path);
}

void
RigExecAimConstraintHandle::SetWorldUpObject(const SdfPath &path)
{
    _SetSingleRel("rigExec:worldUpObject", path);
}

void
RigExecAimConstraintHandle::SetWorldUpType(const TfToken &type)
{
    _AuthorAttr(
        GetPrim(), "rigExec:worldUpType", SdfValueTypeNames->Token, VtValue(type));
}

void
RigExecSingleChainIkConstraintHandle::SetFirstJoint(const SdfPath &path)
{
    _SetSingleRel("rigExec:firstJoint", path);
}

void
RigExecSingleChainIkConstraintHandle::SetEndJoint(const SdfPath &path)
{
    _SetSingleRel("rigExec:endJoint", path);
}

void
RigExecSingleChainIkConstraintHandle::SetEffector(const SdfPath &path)
{
    _SetSingleRel("rigExec:effector", path);
}

void
RigExecSingleChainIkConstraintHandle::SetMoves(
    const std::vector<SdfPath> &paths)
{
    if (paths.empty()) {
        throw std::invalid_argument("IK moves chain must not be empty");
    }
    SetRel("rigExec:moves", paths);
}

void
RigExecSingleChainIkConstraintHandle::SetPoleVectorObjects(
    const std::vector<SdfPath> &paths)
{
    SetRel("rigExec:poleVectorObjects", paths);
}

void
RigExecSingleChainIkConstraintHandle::SetSolverMode(const TfToken &mode)
{
    _AuthorAttr(
        GetPrim(), "rigExec:solverMode", SdfValueTypeNames->Token, VtValue(mode));
}

void
RigExecSingleChainIkConstraintHandle::SetPoleVectorMode(const TfToken &mode)
{
    _AuthorAttr(
        GetPrim(), "rigExec:poleVectorMode", SdfValueTypeNames->Token, VtValue(mode));
}

// ---------------------------------------------------------------------------
// Weight objects
// ---------------------------------------------------------------------------

void
RigExecWeightHandle::SetTarget(const SdfPath &target)
{
    if (target.IsEmpty()) {
        throw std::invalid_argument("weight target must not be empty");
    }
    SetRel("rigExec:weightTarget", { target });
}

void
RigExecWeightHandle::SetRepresentation(const TfToken &rep)
{
    _AuthorAttr(
        GetPrim(), "rigExec:representation", SdfValueTypeNames->Token, VtValue(rep));
}

void
RigExecWeightHandle::SetRangePolicy(const TfToken &policy)
{
    _AuthorAttr(
        GetPrim(), "rigExec:rangePolicy", SdfValueTypeNames->Token, VtValue(policy));
}

void
RigExecStaticWeightHandle::SetValues(const std::vector<float> &values)
{
    _AuthorAttr(
        GetPrim(), "rigExec:values", SdfValueTypeNames->FloatArray,
        VtValue(VtFloatArray(values.begin(), values.end())));
}

void
RigExecStaticWeightHandle::SetIndices(const std::vector<int> &indices)
{
    _AuthorAttr(
        GetPrim(), "rigExec:indices", SdfValueTypeNames->IntArray,
        VtValue(VtIntArray(indices.begin(), indices.end())));
}

void
RigExecStaticWeightHandle::SetDefaultWeight(float weight)
{
    _AuthorAttr(
        GetPrim(), "rigExec:defaultWeight", SdfValueTypeNames->Float, VtValue(weight));
}

void
RigExecDynamicWeightHandle::SetBaseWeight(const SdfPath &path)
{
    if (path.IsEmpty()) {
        return;
    }
    SetRel("rigExec:baseWeight", { path });
}

void
RigExecDynamicWeightHandle::SetDriver(float driver)
{
    _AuthorAttr(
        GetPrim(), "inputs:driver", SdfValueTypeNames->Float, VtValue(driver));
}

void
RigExecDynamicWeightHandle::SetScale(float scale)
{
    _AuthorAttr(
        GetPrim(), "inputs:scale", SdfValueTypeNames->Float, VtValue(scale));
}

void
RigExecDynamicWeightHandle::SetBias(float bias)
{
    _AuthorAttr(
        GetPrim(), "inputs:bias", SdfValueTypeNames->Float, VtValue(bias));
}

// ---- Placed volumes -------------------------------------------------------

void RigExecVolumeWeightHandle::SetRestSpace(const GfMatrix4d &m)
{ _SetRestSpace(this, m); }
void RigExecVolumeWeightHandle::SetAvarTranslation(double tx, double ty, double tz)
{ _SetAvarTranslation(this, tx, ty, tz); }
void RigExecVolumeWeightHandle::SetAvarRotation(
    double rx, double ry, double rz, const TfToken &order)
{ _SetAvarRotation(this, rx, ry, rz, order); }

void
RigExecVolumeWeightHandle::SetFalloff(float falloffMin, float falloffMax)
{
    _AuthorAttr(
        GetPrim(), "inputs:falloffMin", SdfValueTypeNames->Float, VtValue(falloffMin));
    _AuthorAttr(
        GetPrim(), "inputs:falloffMax", SdfValueTypeNames->Float, VtValue(falloffMax));
}

void
RigExecVolumeWeightHandle::SetInvert(float invert)
{
    _AuthorAttr(
        GetPrim(), "inputs:invert", SdfValueTypeNames->Float, VtValue(invert));
}

void
RigExecVolumeWeightHandle::SetStrength(float strength)
{
    _AuthorAttr(
        GetPrim(), "inputs:strength", SdfValueTypeNames->Float, VtValue(strength));
}

void
RigExecVolumeWeightHandle::SetFalloffProfile(const TfToken &profile)
{
    _AuthorAttr(
        GetPrim(), "rigExec:falloffProfile", SdfValueTypeNames->Token,
        VtValue(profile));
}

void
RigExecVolumeWeightHandle::SetFalloffCurve(
    const std::vector<std::pair<double, double>> &knots)
{
    if (knots.size() < 2) {
        throw std::invalid_argument("falloff curve needs at least two knots");
    }
    UsdAttribute attr = GetPrim().GetAttribute(TfToken("rigExec:falloffCurve"));
    if (!attr) {
        // The schema declares this as a plain float; the engine reads it with
        // UsdAttribute::GetSpline(), so the curve IS its time samples.
        attr = GetPrim().CreateAttribute(
            TfToken("rigExec:falloffCurve"), SdfValueTypeNames->Float,
            false /* uniform */);
    }
    if (!attr) {
        throw std::runtime_error("failed to create rigExec:falloffCurve");
    }
    for (const auto &knot : knots) {
        attr.Set(float(knot.second), UsdTimeCode(_Clamp01(knot.first)));
    }
}

void
RigExecVolumeWeightHandle::SetSamplePhase(const TfToken &phase)
{
    _AuthorAttr(
        GetPrim(), "rigExec:samplePhase", SdfValueTypeNames->Token, VtValue(phase));
}

void
RigExecVolumeWeightHandle::SetSampleSource(const SdfPath &path)
{
    if (path.IsEmpty()) {
        return;
    }
    SetRel("rigExec:sampleSource", { path });
}

void
RigExecSphereWeightHandle::SetScales(float sx, float sy, float sz)
{
    _AuthorAttr(GetPrim(), "inputs:scaleX", SdfValueTypeNames->Float, VtValue(sx));
    _AuthorAttr(GetPrim(), "inputs:scaleY", SdfValueTypeNames->Float, VtValue(sy));
    _AuthorAttr(GetPrim(), "inputs:scaleZ", SdfValueTypeNames->Float, VtValue(sz));
}

void
RigExecPlaneWeightHandle::SetAxis(const TfToken &axis)
{
    _AuthorAttr(
        GetPrim(), "rigExec:planeAxis", SdfValueTypeNames->Token, VtValue(axis));
}

void
RigExecPlaneWeightHandle::SetBounds(const TfToken &bounds)
{
    _AuthorAttr(
        GetPrim(), "rigExec:planeBounds", SdfValueTypeNames->Token, VtValue(bounds));
}

void
RigExecPlaneWeightHandle::SetExtents(float extentU, float extentV)
{
    _AuthorAttr(GetPrim(), "inputs:extentU", SdfValueTypeNames->Float, VtValue(extentU));
    _AuthorAttr(GetPrim(), "inputs:extentV", SdfValueTypeNames->Float, VtValue(extentV));
}

void
RigExecCurveWeightHandle::SetCurve(const SdfPath &path)
{
    if (path.IsEmpty()) {
        throw std::invalid_argument("curve weight needs a curve source");
    }
    SetRel("rigExec:curve", { path });
}

void
RigExecCurveWeightHandle::SetScales(float sx, float sy, float sz)
{
    _AuthorAttr(GetPrim(), "inputs:scaleX", SdfValueTypeNames->Float, VtValue(sx));
    _AuthorAttr(GetPrim(), "inputs:scaleY", SdfValueTypeNames->Float, VtValue(sy));
    _AuthorAttr(GetPrim(), "inputs:scaleZ", SdfValueTypeNames->Float, VtValue(sz));
}

void
RigExecCombineWeightHandle::SetInputWeights(const std::vector<SdfPath> &paths)
{
    SetRel("rigExec:inputWeights", paths);
}

void
RigExecCombineWeightHandle::SetCombineMode(const TfToken &mode)
{
    _AuthorAttr(
        GetPrim(), "rigExec:combineMode", SdfValueTypeNames->Token, VtValue(mode));
}

void
RigExecCombineWeightHandle::SetStrength(float strength)
{
    _AuthorAttr(
        GetPrim(), "inputs:strength", SdfValueTypeNames->Float, VtValue(strength));
}

void
RigExecCombineWeightHandle::SetInvert(float invert)
{
    _AuthorAttr(
        GetPrim(), "inputs:invert", SdfValueTypeNames->Float, VtValue(invert));
}

// ---------------------------------------------------------------------------
// Blend inputs / samples
// ---------------------------------------------------------------------------

void
RigExecBlendInputHandle::SetWeight(float weight)
{
    _AuthorAttr(
        GetPrim(), "inputs:weight", SdfValueTypeNames->Float, VtValue(weight));
}

RigExecBlendSampleHandle
RigExecBlendInputHandle::AddSample(const std::string &name, float activation)
{
    const SdfPath samplePath = _path.AppendChild(TfToken(name));
    UsdPrim existing = _stage->GetPrimAtPath(samplePath);
    if (existing) {
        throw std::invalid_argument(
            "blend sample already exists: " + samplePath.GetString());
    }
    UsdPrim prim(_stage->DefinePrim(samplePath, TfToken("RigExecBlendSample")));
    _AuthorAttr(
        prim, "rigExec:activation", SdfValueTypeNames->Float, VtValue(activation));
    _AppendRelTarget(GetPrim(), "rigExec:samples", samplePath);
    return RigExecBlendSampleHandle(_stage, samplePath);
}

void
RigExecBlendSampleHandle::SetActivation(float activation)
{
    _AuthorAttr(
        GetPrim(), "rigExec:activation", SdfValueTypeNames->Float, VtValue(activation));
}

void
RigExecBlendSampleHandle::SetTargetPoints(const SdfPath &path)
{
    if (path.IsEmpty()) {
        throw std::invalid_argument("blend sample needs a target points property");
    }
    SetRel("rigExec:targetPoints", { path });
}

void
RigExecBlendSampleHandle::SetReadPhase(const TfToken &phase)
{
    _AuthorAttr(
        GetPrim(), "rigExec:pointsReadPhase", SdfValueTypeNames->Token, VtValue(phase));
}

// ---------------------------------------------------------------------------
// Curvenet
// ---------------------------------------------------------------------------

void
RigExecCurvenetHandle::SetPoints(const std::vector<GfVec3f> &points)
{
    _AuthorAttr(
        GetPrim(), "points", SdfValueTypeNames->Point3fArray,
        VtValue(VtArray<GfVec3f>(points.begin(), points.end())));
}

void
RigExecCurvenetHandle::AddSpline(int p0, int h0, int h1, int p1)
{
    UsdAttribute attr = GetPrim().GetAttribute(TfToken("rigExec:splineIndices"));
    if (!attr) {
        attr = GetPrim().CreateAttribute(
            TfToken("rigExec:splineIndices"), SdfValueTypeNames->IntArray,
            false /* uniform */);
    }
    VtIntArray indices;
    attr.Get(&indices);
    indices.push_back(p0);
    indices.push_back(h0);
    indices.push_back(h1);
    indices.push_back(p1);
    attr.Set(VtValue(indices));
}

void
RigExecCurvenetHandle::SetBasis(const TfToken &basis)
{
    _AuthorAttr(
        GetPrim(), "rigExec:basis", SdfValueTypeNames->Token, VtValue(basis));
}

void
RigExecCurvenetHandle::SetSamplesPerSpline(int count)
{
    _AuthorAttr(
        GetPrim(), "rigExec:samplesPerSpline", SdfValueTypeNames->Int, VtValue(count));
}

// ---------------------------------------------------------------------------
// Mover handles
// ---------------------------------------------------------------------------

void
RigExecMatrixMoverHandle::SetTransformProvider(const SdfPath &path)
{
    if (path.IsEmpty()) {
        throw std::invalid_argument("matrix mover needs a transform provider");
    }
    SetRel("rigExec:transform", { path });
}

void
RigExecMatrixMoverHandle::SetWeightObject(const SdfPath &path)
{
    if (path.IsEmpty()) {
        throw std::invalid_argument("matrix mover needs a weight object");
    }
    SetRel("rigExec:weightObject", { path });
}

void
RigExecMatrixMoverHandle::SetReadPhase(const TfToken &phase)
{
    _AuthorAttr(
        GetPrim(), "rigExec:transformReadPhase", SdfValueTypeNames->Token,
        VtValue(phase));
}

void
RigExecLatticeMoverHandle::SetCage(const SdfPath &path)
{
    if (path.IsEmpty()) {
        throw std::invalid_argument("lattice mover needs a cage prim");
    }
    SetRel("rigExec:cage", { path });
}

void
RigExecLatticeMoverHandle::SetBasis(const TfToken &basis)
{
    _AuthorAttr(
        GetPrim(), "rigExec:basis", SdfValueTypeNames->Token, VtValue(basis));
}

void
RigExecLatticeMoverHandle::SetDivisions(int x, int y, int z)
{
    _AuthorAttr(
        GetPrim(), "rigExec:divisions", SdfValueTypeNames->Int3,
        VtValue(GfVec3i(x, y, z)));
}

void
RigExecLatticeMoverHandle::SetReadPhase(const TfToken &phase)
{
    _AuthorAttr(
        GetPrim(), "rigExec:cageReadPhase", SdfValueTypeNames->Token, VtValue(phase));
}

RigExecBlendInputHandle
RigExecBlendShapeMoverHandle::AddBlendInput(const std::string &name, float weight)
{
    const SdfPath inputPath = _path.AppendChild(TfToken(name));
    UsdPrim existing = _stage->GetPrimAtPath(inputPath);
    if (existing) {
        throw std::invalid_argument(
            "blend input already exists: " + inputPath.GetString());
    }
    UsdPrim prim(_stage->DefinePrim(inputPath, TfToken("RigExecBlendInput")));
    _AuthorAttr(prim, "inputs:weight", SdfValueTypeNames->Float, VtValue(weight));
    _AppendRelTarget(GetPrim(), "rigExec:blendInputs", inputPath);
    return RigExecBlendInputHandle(_stage, inputPath);
}

void
RigExecBlendShapeMoverHandle::SetWeightObject(const SdfPath &path)
{
    if (path.IsEmpty()) {
        return;  // optional per-point weight
    }
    SetRel("rigExec:weightObject", { path });
}

void
RigExecCurveMoverHandle::SetDriverCurve(const SdfPath &path)
{
    if (path.IsEmpty()) {
        throw std::invalid_argument("curve mover needs a driver curve");
    }
    SetRel("rigExec:driverCurve", { path });
}

void
RigExecCurveMoverHandle::SetDriverFrames(const std::vector<SdfPath> &paths)
{
    if (paths.empty()) {
        return;
    }
    SetRel("rigExec:driverFrames", paths);
}

void
RigExecCurveMoverHandle::SetBindCoordinates(const SdfPath &path)
{
    if (path.IsEmpty()) {
        return;
    }
    SetRel("rigExec:bindCoordinates", { path });
}

void
RigExecCurveMoverHandle::SetMode(const TfToken &mode)
{
    _AuthorAttr(
        GetPrim(), "rigExec:mode", SdfValueTypeNames->Token, VtValue(mode));
}

void
RigExecCurveMoverHandle::SetReadPhase(const TfToken &phase)
{
    _AuthorAttr(
        GetPrim(), "rigExec:driverCurveReadPhase", SdfValueTypeNames->Token,
        VtValue(phase));
}

void
RigExecSurfaceMoverHandle::SetSurface(const SdfPath &path)
{
    if (path.IsEmpty()) {
        throw std::invalid_argument("surface mover needs a surface prim");
    }
    SetRel("rigExec:surface", { path });
}

void
RigExecSurfaceMoverHandle::SetMode(const TfToken &mode)
{
    _AuthorAttr(
        GetPrim(), "rigExec:mode", SdfValueTypeNames->Token, VtValue(mode));
}

void
RigExecSurfaceMoverHandle::SetReadPhase(const TfToken &phase)
{
    _AuthorAttr(
        GetPrim(), "rigExec:surfaceReadPhase", SdfValueTypeNames->Token,
        VtValue(phase));
}

void
RigExecSmoothMoverHandle::SetStrength(float strength)
{
    _AuthorAttr(
        GetPrim(), "inputs:strength", SdfValueTypeNames->Float, VtValue(strength));
}

void
RigExecVolumeCorrectMoverHandle::SetStrength(float strength)
{
    _AuthorAttr(
        GetPrim(), "inputs:strength", SdfValueTypeNames->Float, VtValue(strength));
}

void
RigExecCurvenetMoverHandle::SetCurvenet(const SdfPath &path)
{
    if (path.IsEmpty()) {
        throw std::invalid_argument("curvenet mover needs a curvenet prim");
    }
    SetRel("rigExec:curvenet", { path });
}

void
RigExecCurvenetMoverHandle::SetStrength(float strength)
{
    _AuthorAttr(
        GetPrim(), "inputs:strength", SdfValueTypeNames->Float, VtValue(strength));
}

void
RigExecFloatMathMoverHandle::SetOperation(const TfToken &op)
{
    _AuthorAttr(
        GetPrim(), "rigExec:operation", SdfValueTypeNames->Token, VtValue(op));
}

void
RigExecFloatMathMoverHandle::SetValue(float value)
{
    _AuthorAttr(
        GetPrim(), "inputs:value", SdfValueTypeNames->Float, VtValue(value));
}

void
RigExecFloatMathMoverHandle::SetBounds(float min, float max)
{
    _AuthorAttr(GetPrim(), "inputs:min", SdfValueTypeNames->Float, VtValue(min));
    _AuthorAttr(GetPrim(), "inputs:max", SdfValueTypeNames->Float, VtValue(max));
}

void
RigExecFloatMathMoverHandle::SetWeight(float weight)
{
    _AuthorAttr(
        GetPrim(), "inputs:weight", SdfValueTypeNames->Float, VtValue(weight));
}

void
RigExecVec3fMathMoverHandle::SetOperation(const TfToken &op)
{
    _AuthorAttr(
        GetPrim(), "rigExec:operation", SdfValueTypeNames->Token, VtValue(op));
}

void
RigExecVec3fMathMoverHandle::SetValue(GfVec3f value)
{
    _AuthorAttr(
        GetPrim(), "inputs:value", SdfValueTypeNames->Float3, VtValue(value));
}

void
RigExecVec3fMathMoverHandle::SetBounds(GfVec3f min, GfVec3f max)
{
    _AuthorAttr(GetPrim(), "inputs:min", SdfValueTypeNames->Float3, VtValue(min));
    _AuthorAttr(GetPrim(), "inputs:max", SdfValueTypeNames->Float3, VtValue(max));
}

void
RigExecVec3fMathMoverHandle::SetWeight(float weight)
{
    _AuthorAttr(
        GetPrim(), "inputs:weight", SdfValueTypeNames->Float, VtValue(weight));
}

void
RigExecMatrixMathMoverHandle::SetOperation(const TfToken &op)
{
    _AuthorAttr(
        GetPrim(), "rigExec:operation", SdfValueTypeNames->Token, VtValue(op));
}

void
RigExecMatrixMathMoverHandle::SetValue(GfMatrix4d value)
{
    _AuthorAttr(
        GetPrim(), "inputs:value", SdfValueTypeNames->Matrix4d, VtValue(value));
}

void
RigExecMatrixMathMoverHandle::SetWeight(float weight)
{
    _AuthorAttr(
        GetPrim(), "inputs:weight", SdfValueTypeNames->Float, VtValue(weight));
}

// ---------------------------------------------------------------------------
// Mover chains
// ---------------------------------------------------------------------------

SdfPath
RigExecMoverChain::_AddMoverPrim(
    const std::string &typeName, const std::string &name, const SdfPath &target)
{
    if (name.empty()) {
        throw std::invalid_argument("mover name must not be empty");
    }
    const SdfPath path = _scope.AppendChild(TfToken(name));
    UsdPrim existing = _stage->GetPrimAtPath(path);
    if (existing) {
        throw std::invalid_argument(
            "prim already exists: " + path.GetString());
    }
    UsdPrim prim(_stage->DefinePrim(path, TfToken(typeName)));
    if (!prim.IsValid()) {
        throw std::runtime_error("failed to define mover at " + path.GetString());
    }
    // RigExecMoverAPI carries rigExec:moves and inputs:enabled; the node-graph
    // API keeps hand-authored and built rigs visually identical in usdview.
    _ApplyApiBestEffort(prim, _kMoverApi);
    _ApplyApiBestEffort(prim, _kNodeGraphApi);
    // An empty operation target reuses the chain's default target (set at
    // construction); with no default either, the mover is left unwired and
    // the engine treats it as inert.
    const SdfPath effectiveTarget =
        target.IsEmpty() ? _defaultTarget : target;
    if (!effectiveTarget.IsEmpty()) {
        _AppendRelTarget(prim, "rigExec:moves", effectiveTarget);
    }
    return path;
}

RigExecMatrixMoverHandle
RigExecMoverChain::AddMatrixMover(
    const std::string &name, const SdfPath &transformProvider,
    const SdfPath &weightObject, const SdfPath &target, const TfToken &readPhase)
{
    RigExecMatrixMoverHandle handle(_stage, _AddMoverPrim("RigExecMatrixMover", name, target));
    handle.SetTransformProvider(transformProvider);
    handle.SetWeightObject(weightObject);
    if (!readPhase.IsEmpty()) {
        handle.SetReadPhase(readPhase);
    }
    return handle;
}

RigExecLatticeMoverHandle
RigExecMoverChain::AddLatticeMover(
    const std::string &name, const SdfPath &cagePrim, int divX, int divY,
    int divZ, const TfToken &basis, const SdfPath &target,
    const TfToken &readPhase)
{
    RigExecLatticeMoverHandle handle(_stage, _AddMoverPrim("RigExecLatticeMover", name, target));
    handle.SetCage(cagePrim);
    if (!basis.IsEmpty()) {
        handle.SetBasis(basis);
    }
    handle.SetDivisions(divX, divY, divZ);
    if (!readPhase.IsEmpty()) {
        handle.SetReadPhase(readPhase);
    }
    return handle;
}

RigExecBlendShapeMoverHandle
RigExecMoverChain::AddBlendShapeMover(
    const std::string &name, const SdfPath &weightObject, const SdfPath &target)
{
    RigExecBlendShapeMoverHandle handle(_stage, _AddMoverPrim("RigExecBlendShapeMover", name, target));
    if (!weightObject.IsEmpty()) {
        handle.SetWeightObject(weightObject);
    }
    return handle;
}

RigExecCurveMoverHandle
RigExecMoverChain::AddCurveMover(
    const std::string &name, const SdfPath &driverCurve,
    const SdfPath &driverFrames, const SdfPath &bindCoordinates,
    const TfToken &mode, const SdfPath &target, const TfToken &readPhase)
{
    RigExecCurveMoverHandle handle(_stage, _AddMoverPrim("RigExecCurveMover", name, target));
    handle.SetDriverCurve(driverCurve);
    if (!driverFrames.IsEmpty()) {
        handle.SetDriverFrames({ driverFrames });
    }
    if (!bindCoordinates.IsEmpty()) {
        handle.SetBindCoordinates(bindCoordinates);
    }
    if (!mode.IsEmpty()) {
        handle.SetMode(mode);
    }
    if (!readPhase.IsEmpty()) {
        handle.SetReadPhase(readPhase);
    }
    return handle;
}

RigExecSurfaceMoverHandle
RigExecMoverChain::AddSurfaceMover(
    const std::string &name, const SdfPath &surfacePrim, const TfToken &mode,
    const SdfPath &target, const TfToken &readPhase)
{
    RigExecSurfaceMoverHandle handle(_stage, _AddMoverPrim("RigExecSurfaceMover", name, target));
    handle.SetSurface(surfacePrim);
    if (!mode.IsEmpty()) {
        handle.SetMode(mode);
    }
    if (!readPhase.IsEmpty()) {
        handle.SetReadPhase(readPhase);
    }
    return handle;
}

RigExecSmoothMoverHandle
RigExecMoverChain::AddSmoothMover(
    const std::string &name, float strength, const SdfPath &target)
{
    RigExecSmoothMoverHandle handle(_stage, _AddMoverPrim("RigExecSmoothMover", name, target));
    handle.SetStrength(strength);
    return handle;
}

RigExecVolumeCorrectMoverHandle
RigExecMoverChain::AddVolumeCorrectMover(
    const std::string &name, float strength, const SdfPath &target)
{
    RigExecVolumeCorrectMoverHandle handle(_stage, _AddMoverPrim("RigExecVolumeCorrectMover", name, target));
    handle.SetStrength(strength);
    return handle;
}

RigExecCurvenetMoverHandle
RigExecMoverChain::AddCurvenetMover(
    const std::string &name, const SdfPath &curvenetPrim, float strength,
    const SdfPath &target)
{
    RigExecCurvenetMoverHandle handle(_stage, _AddMoverPrim("RigExecCurvenetMover", name, target));
    handle.SetCurvenet(curvenetPrim);
    handle.SetStrength(strength);
    return handle;
}

RigExecFloatMathMoverHandle
RigExecMoverChain::AddFloatMathMover(
    const std::string &name, const TfToken &operation, float value,
    const SdfPath &target, float weight)
{
    RigExecFloatMathMoverHandle handle(_stage, _AddMoverPrim("RigExecFloatMathMover", name, target));
    if (!operation.IsEmpty()) {
        handle.SetOperation(operation);
    }
    handle.SetValue(value);
    handle.SetWeight(weight);
    return handle;
}

RigExecVec3fMathMoverHandle
RigExecMoverChain::AddVec3fMathMover(
    const std::string &name, const TfToken &operation, GfVec3f value,
    const SdfPath &target, float weight)
{
    RigExecVec3fMathMoverHandle handle(_stage, _AddMoverPrim("RigExecVec3fMathMover", name, target));
    if (!operation.IsEmpty()) {
        handle.SetOperation(operation);
    }
    handle.SetValue(value);
    handle.SetWeight(weight);
    return handle;
}

RigExecMatrixMathMoverHandle
RigExecMoverChain::AddMatrixMathMover(
    const std::string &name, const TfToken &operation, GfMatrix4d value,
    const SdfPath &target, float weight)
{
    RigExecMatrixMathMoverHandle handle(_stage, _AddMoverPrim("RigExecMatrixMathMover", name, target));
    if (!operation.IsEmpty()) {
        handle.SetOperation(operation);
    }
    handle.SetValue(value);
    handle.SetWeight(weight);
    return handle;
}

// ---- Constraints as movers -------------------------------------------------
//
// Constraints are movers: the chain applies RigExecMoverAPI and authors the
// exact target on rigExec:moves, so a constraint participates in composed
// post-order application like any other operation.

RigExecAimConstraintHandle
RigExecMoverChain::AddAimConstraint(const std::string &name, const SdfPath &target)
{
    return RigExecAimConstraintHandle(
        _stage, _AddMoverPrim("RigExecAimConstraint", name, target));
}

RigExecPositionConstraintHandle
RigExecMoverChain::AddPositionConstraint(const std::string &name, const SdfPath &target)
{
    return RigExecPositionConstraintHandle(
        _stage, _AddMoverPrim("RigExecPositionConstraint", name, target));
}

RigExecRotationConstraintHandle
RigExecMoverChain::AddRotationConstraint(const std::string &name, const SdfPath &target)
{
    return RigExecRotationConstraintHandle(
        _stage, _AddMoverPrim("RigExecRotationConstraint", name, target));
}

RigExecScaleConstraintHandle
RigExecMoverChain::AddScaleConstraint(const std::string &name, const SdfPath &target)
{
    return RigExecScaleConstraintHandle(
        _stage, _AddMoverPrim("RigExecScaleConstraint", name, target));
}

RigExecParentConstraintHandle
RigExecMoverChain::AddParentConstraint(const std::string &name, const SdfPath &target)
{
    return RigExecParentConstraintHandle(
        _stage, _AddMoverPrim("RigExecParentConstraint", name, target));
}

RigExecSingleChainIkConstraintHandle
RigExecMoverChain::AddSingleChainIkConstraint(const std::string &name, const SdfPath &target)
{
    return RigExecSingleChainIkConstraintHandle(
        _stage, _AddMoverPrim("RigExecSingleChainIkConstraint", name, target));
}

// ---------------------------------------------------------------------------
// Top-level builder
// ---------------------------------------------------------------------------

RigExecRigBuilder
RigExecRigBuilder::Create(
    UsdStageRefPtr stage, const SdfPath &rigRoot, const TfToken &partition)
{
    if (!stage) {
        throw std::invalid_argument("RigExecRigBuilder::Create needs a valid stage");
    }
    if (rigRoot.IsEmpty()) {
        throw std::invalid_argument("rig root path must not be empty");
    }

    UsdPrim root = stage->GetPrimAtPath(rigRoot);
    if (root) {
        if (root.GetTypeName() != TfToken("RigExecRoot")) {
            throw std::invalid_argument(
                "prim at " + rigRoot.GetString() + " is a " +
                root.GetTypeName().GetString() + ", not a RigExecRoot");
        }
    } else {
        root = UsdPrim(stage->DefinePrim(rigRoot, TfToken("RigExecRoot")));
        if (!root.IsValid()) {
            throw std::runtime_error(
                "failed to define RigExecRoot at " + rigRoot.GetString());
        }
    }

    RigExecRigBuilder builder(std::move(stage), rigRoot);
    if (!partition.IsEmpty()) {
        _AuthorAttr(
            root, "rigExec:partition", SdfValueTypeNames->Token, VtValue(partition));
    }
    return builder;
}

SdfPath
RigExecRigBuilder::_EnsureScope(const char *scopeName)
{
    const SdfPath path = _root.AppendChild(TfToken(scopeName));
    UsdPrim scope = _stage->GetPrimAtPath(path);
    if (!scope) {
        scope = UsdPrim(_stage->DefinePrim(path, TfToken("Scope")));
        if (!scope.IsValid()) {
            throw std::runtime_error(
                "failed to define scope at " + path.GetString());
        }
    }
    return path;
}

UsdPrim
RigExecRigBuilder::_DefineTyped(
    const SdfPath &parent, const std::string &typeName, const std::string &name)
{
    if (name.empty()) {
        throw std::invalid_argument("prim name must not be empty");
    }
    const SdfPath path = parent.AppendChild(TfToken(name));
    UsdPrim existing = _stage->GetPrimAtPath(path);
    if (existing) {
        if (existing.GetTypeName() == TfToken(typeName)) {
            return existing;  // idempotent re-open of the same rig object
        }
        throw std::invalid_argument(
            "prim at " + path.GetString() + " is a " +
            existing.GetTypeName().GetString() + ", not a " + typeName);
    }
    UsdPrim prim(_stage->DefinePrim(path, TfToken(typeName)));
    if (!prim.IsValid()) {
        throw std::runtime_error(
            "failed to define " + typeName + " at " + path.GetString());
    }
    return prim;
}

RigExecControlHandle
RigExecRigBuilder::AddControl(const std::string &name, const GfMatrix4d &restSpace)
{
    const SdfPath scope = _EnsureScope("Controls");
    UsdPrim prim = _DefineTyped(scope, "RigExecControl", name);
    _ApplyApiBestEffort(prim, _kControlApi);
    _ApplyApiBestEffort(prim, _kNodeGraphApi);
    RigExecControlHandle handle(_stage, prim.GetPath());
    if (restSpace != GfMatrix4d()) {
        handle.SetRestSpace(restSpace);
    }
    return handle;
}

RigExecJointHandle
RigExecRigBuilder::AddJoint(
    const std::string &name, const GfMatrix4d &restSpace,
    const RigExecJointHandle *parentJoint)
{
    SdfPath parent = _EnsureScope("Joints");
    if (parentJoint && parentJoint->IsValid()) {
        parent = parentJoint->GetPath();
    }
    UsdPrim prim = _DefineTyped(parent, "RigExecJoint", name);
    _ApplyApiBestEffort(prim, _kNodeGraphApi);
    RigExecJointHandle handle(_stage, prim.GetPath());
    if (restSpace != GfMatrix4d()) {
        handle.SetRestSpace(restSpace);
    }
    return handle;
}

RigExecFkChainHandle
RigExecRigBuilder::AddFkChain(const std::string &name)
{
    const SdfPath scope = _EnsureScope("Solvers");
    UsdPrim prim = _DefineTyped(scope, "RigExecFkChain", name);
    _ApplyApiBestEffort(prim, _kNodeGraphApi);
    return RigExecFkChainHandle(_stage, prim.GetPath());
}

RigExecTwoBoneIkHandle
RigExecRigBuilder::AddTwoBoneIk(
    const std::string &name, const SdfPath &rootControl,
    const SdfPath &effectorControl, const SdfPath &poleControl)
{
    const SdfPath scope = _EnsureScope("Solvers");
    UsdPrim prim = _DefineTyped(scope, "RigExecTwoBoneIk", name);
    _ApplyApiBestEffort(prim, _kNodeGraphApi);
    RigExecTwoBoneIkHandle handle(_stage, prim.GetPath());
    if (!rootControl.IsEmpty()) {
        handle.SetRootControl(rootControl);
    }
    if (!effectorControl.IsEmpty()) {
        handle.SetEffectorControl(effectorControl);
    }
    if (!poleControl.IsEmpty()) {
        handle.SetPoleControl(poleControl);
    }
    return handle;
}

RigExecBlendPointFramesHandle
RigExecRigBuilder::AddBlendPointFrames(
    const std::string &name, const SdfPath &inputA, const SdfPath &inputB,
    float weight)
{
    const SdfPath scope = _EnsureScope("Solvers");
    UsdPrim prim = _DefineTyped(scope, "RigExecBlendPointFrames", name);
    _ApplyApiBestEffort(prim, _kNodeGraphApi);
    RigExecBlendPointFramesHandle handle(_stage, prim.GetPath());
    if (!inputA.IsEmpty()) {
        handle.SetInputA(inputA);
    }
    if (!inputB.IsEmpty()) {
        handle.SetInputB(inputB);
    }
    handle.SetWeight(weight);
    return handle;
}

RigExecTwistDistributionHandle
RigExecRigBuilder::AddTwistDistribution(
    const std::string &name, const SdfPath &start, const SdfPath &end, int count)
{
    const SdfPath scope = _EnsureScope("Solvers");
    UsdPrim prim = _DefineTyped(scope, "RigExecTwistDistribution", name);
    _ApplyApiBestEffort(prim, _kNodeGraphApi);
    RigExecTwistDistributionHandle handle(_stage, prim.GetPath());
    if (!start.IsEmpty()) {
        handle.SetStart(start);
    }
    if (!end.IsEmpty()) {
        handle.SetEnd(end);
    }
    handle.SetCount(count);
    return handle;
}

RigExecRibbonHandle
RigExecRigBuilder::AddRibbon(
    const std::string &name, const SdfPath &driverCurve, int sampleCount)
{
    const SdfPath scope = _EnsureScope("Solvers");
    UsdPrim prim = _DefineTyped(scope, "RigExecRibbon", name);
    _ApplyApiBestEffort(prim, _kNodeGraphApi);
    RigExecRibbonHandle handle(_stage, prim.GetPath());
    if (!driverCurve.IsEmpty()) {
        handle.SetDriverCurve(driverCurve);
    }
    handle.SetSampleCount(sampleCount);
    return handle;
}

// ---- Weight objects ---------------------------------------------------------

RigExecStaticWeightHandle
RigExecRigBuilder::AddStaticWeight(
    const std::string &name, const SdfPath &target,
    const std::vector<float> &values, const std::vector<int> &indices,
    float defaultWeight)
{
    const SdfPath scope = _EnsureScope("Weights");
    UsdPrim prim = _DefineTyped(scope, "RigExecStaticWeight", name);
    _ApplyApiBestEffort(prim, _kNodeGraphApi);
    RigExecStaticWeightHandle handle(_stage, prim.GetPath());
    if (!target.IsEmpty()) {
        handle.SetTarget(target);
    }
    if (!values.empty()) {
        handle.SetValues(values);
    }
    if (!indices.empty()) {
        handle.SetIndices(indices);
        handle.SetRepresentation(TfToken("sparse"));
    } else if (!values.empty()) {
        handle.SetRepresentation(TfToken("dense"));
    }
    if (defaultWeight != 0.f) {
        handle.SetDefaultWeight(defaultWeight);
    }
    return handle;
}

RigExecDynamicWeightHandle
RigExecRigBuilder::AddDynamicWeight(
    const std::string &name, const SdfPath &target, const SdfPath &baseWeight)
{
    const SdfPath scope = _EnsureScope("Weights");
    UsdPrim prim = _DefineTyped(scope, "RigExecDynamicWeight", name);
    _ApplyApiBestEffort(prim, _kNodeGraphApi);
    RigExecDynamicWeightHandle handle(_stage, prim.GetPath());
    if (!target.IsEmpty()) {
        handle.SetTarget(target);
    }
    if (!baseWeight.IsEmpty()) {
        handle.SetBaseWeight(baseWeight);
    }
    return handle;
}

RigExecSphereWeightHandle
RigExecRigBuilder::AddSphereWeight(
    const std::string &name, const SdfPath &target, float falloffMin,
    float falloffMax)
{
    const SdfPath scope = _EnsureScope("Weights");
    UsdPrim prim = _DefineTyped(scope, "RigExecSphereWeight", name);
    _ApplyApiBestEffort(prim, _kNodeGraphApi);
    RigExecSphereWeightHandle handle(_stage, prim.GetPath());
    if (!target.IsEmpty()) {
        handle.SetTarget(target);
    }
    handle.SetFalloff(falloffMin, falloffMax);
    return handle;
}

RigExecPlaneWeightHandle
RigExecRigBuilder::AddPlaneWeight(
    const std::string &name, const SdfPath &target, float falloffMin,
    float falloffMax)
{
    const SdfPath scope = _EnsureScope("Weights");
    UsdPrim prim = _DefineTyped(scope, "RigExecPlaneWeight", name);
    _ApplyApiBestEffort(prim, _kNodeGraphApi);
    RigExecPlaneWeightHandle handle(_stage, prim.GetPath());
    if (!target.IsEmpty()) {
        handle.SetTarget(target);
    }
    handle.SetFalloff(falloffMin, falloffMax);
    return handle;
}

RigExecCurveWeightHandle
RigExecRigBuilder::AddCurveWeight(
    const std::string &name, const SdfPath &target, const SdfPath &curve,
    float falloffMin, float falloffMax)
{
    const SdfPath scope = _EnsureScope("Weights");
    UsdPrim prim = _DefineTyped(scope, "RigExecCurveWeight", name);
    _ApplyApiBestEffort(prim, _kNodeGraphApi);
    RigExecCurveWeightHandle handle(_stage, prim.GetPath());
    if (!target.IsEmpty()) {
        handle.SetTarget(target);
    }
    if (!curve.IsEmpty()) {
        handle.SetCurve(curve);
    }
    handle.SetFalloff(falloffMin, falloffMax);
    return handle;
}

RigExecCombineWeightHandle
RigExecRigBuilder::AddCombineWeight(
    const std::string &name, const SdfPath &target,
    const std::vector<SdfPath> &inputWeights, const TfToken &mode)
{
    const SdfPath scope = _EnsureScope("Weights");
    UsdPrim prim = _DefineTyped(scope, "RigExecCombineWeight", name);
    _ApplyApiBestEffort(prim, _kNodeGraphApi);
    RigExecCombineWeightHandle handle(_stage, prim.GetPath());
    if (!target.IsEmpty()) {
        handle.SetTarget(target);
    }
    if (!inputWeights.empty()) {
        handle.SetInputWeights(inputWeights);
    }
    if (!mode.IsEmpty()) {
        handle.SetCombineMode(mode);
    }
    return handle;
}

RigExecCurvenetHandle
RigExecRigBuilder::AddCurvenet(
    const std::string &name, const std::vector<GfVec3f> &points)
{
    const SdfPath scope = _EnsureScope("Curvenets");
    UsdPrim prim = _DefineTyped(scope, "RigExecCurvenet", name);
    _ApplyApiBestEffort(prim, _kNodeGraphApi);
    RigExecCurvenetHandle handle(_stage, prim.GetPath());
    if (!points.empty()) {
        handle.SetPoints(points);
    }
    return handle;
}

RigExecMoverChain
RigExecRigBuilder::NewMoverChain(const std::string &name, const SdfPath &defaultTarget)
{
    const SdfPath scope = _EnsureScope("Movers");
    const SdfPath chainPath = scope.AppendChild(TfToken(name));
    UsdPrim chain = _stage->GetPrimAtPath(chainPath);
    if (!chain) {
        chain = UsdPrim(_stage->DefinePrim(chainPath, TfToken("Scope")));
        if (!chain.IsValid()) {
            throw std::runtime_error(
                "failed to define mover chain at " + chainPath.GetString());
        }
    }
    return RigExecMoverChain(_stage, chainPath, defaultTarget);
}

}  // namespace rigExec
