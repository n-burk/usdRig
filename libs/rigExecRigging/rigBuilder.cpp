//
// RigExec rigging API implementation. See rigBuilder.h for the contract.
//
#include "rigBuilder.h"
#include "schemaAuthoring.h"

#include "rigExecMath/avarScale.h"

#include "pxr/base/gf/vec3d.h"
#include "pxr/base/gf/vec3i.h"
#include "pxr/base/tf/weakPtr.h"
#include "pxr/base/vt/array.h"
#include "pxr/usd/sdf/layer.h"
#include "pxr/usd/sdf/schema.h"
#include "pxr/usd/usd/attribute.h"
#include "pxr/usd/usd/primDefinition.h"
#include "pxr/usd/usd/relationship.h"
#include "pxr/usd/usd/timeCode.h"

#include <algorithm>
#include <cmath>
#include <stdexcept>

PXR_NAMESPACE_USING_DIRECTIVE

namespace rigExec {

namespace {

const TfToken _kControlApi("RigExecControlAPI");
const TfToken _kMoverApi("RigExecMoverAPI");
const TfToken _kNodeGraphApi("NodeGraphNodeAPI");

UsdStageRefPtr
_GetStageRef(const UsdPrim &prim)
{
    if (!prim) {
        throw std::runtime_error("invalid prim for schema-backed authoring");
    }
    UsdStageRefPtr stage =
        TfCreateRefPtrFromProtectedWeakPtr(prim.GetStage());
    if (!stage) {
        throw std::runtime_error(
            "prim has no live stage: " + prim.GetPath().GetString());
    }
    return stage;
}

RigExecSchemaPrim
_Schema(const UsdPrim &prim)
{
    if (!prim || prim.GetTypeName().IsEmpty()) {
        throw std::runtime_error("invalid or untyped prim for schema authoring");
    }
    return RigExecSchemaPrim::Get(
        _GetStageRef(prim), prim.GetPath(), prim.GetTypeName());
}

void
_ApplyApiRequired(const UsdPrim &prim, const TfToken &apiName)
{
    _Schema(prim).ApplyAPI(apiName);
}

void
_RequireAttrDefinition(
    const UsdPrim &prim, const char *name,
    const SdfValueTypeName &typeName)
{
    if (!name || !*name) {
        throw std::invalid_argument("attribute name must not be empty");
    }
    const TfToken attrName(name);
    const UsdPrimDefinition::Attribute definition =
        prim.GetPrimDefinition().GetAttributeDefinition(attrName);
    if (!definition) {
        throw std::invalid_argument(
            "attribute '" + attrName.GetString() +
            "' is not declared at " + prim.GetPath().GetString());
    }
    if (definition.GetTypeName() != typeName) {
        throw std::invalid_argument(
            "attribute '" + attrName.GetString() + "' at " +
            prim.GetPath().GetString() + " is declared as " +
            definition.GetTypeName().GetAsToken().GetString() +
            ", not " + typeName.GetAsToken().GetString());
    }
}

/// Author (or overwrite) one attribute on \p prim with an EXPLICIT Sdf type
/// name, matching the codeless schema declaration exactly. The engine reads
/// every property through typed UsdAttribute::Get calls, so authoring a
/// `token` as a string or a `float[]` as a double array would still compose
// but would not round-trip cleanly through the schema's declared types.
void
_AuthorAttr(
    const UsdPrim &prim, const char *name, const SdfValueTypeName &typeName,
    VtValue value)
{
    _RequireAttrDefinition(prim, name, typeName);
    _Schema(prim).SetAttribute(TfToken(name), value);
}

void
_ClearAttr(const UsdPrim &prim, const char *name)
{
    _Schema(prim).ClearAttribute(TfToken(name));
}

void _RequireTargetPath(const SdfPath &path, const char *what);
void _ClearJointElementsIfTargetsChange(
    const UsdPrim &prim, const std::vector<SdfPath> &newTargets);

void
_SetRel(
    const UsdPrim &prim, const char *name,
    const std::vector<SdfPath> &targets)
{
    if (!name || !*name) {
        throw std::invalid_argument("relationship name must not be empty");
    }
    for (const SdfPath &target : targets) {
        _RequireTargetPath(
            target, (std::string("relationship ") + name + " target").c_str());
    }
    _Schema(prim).SetRelationship(TfToken(name), targets);
}

/// Append \p target to an ordered relationship, preserving existing targets.
void
_AppendRelTarget(const UsdPrim &prim, const char *name, const SdfPath &target)
{
    if (target.IsEmpty()) {
        throw std::invalid_argument(
            std::string("cannot append an empty relationship target to ") + name);
    }
    const UsdRelationship rel = prim.GetRelationship(TfToken(name));
    SdfPathVector targets;
    if (!rel || !rel.GetTargets(&targets)) {
        // The strict setter below produces the precise undeclared/wrong-kind
        // diagnostic without ever inventing a custom relationship.
        _SetRel(prim, name, { target });
        return;
    }
    targets.push_back(target);
    _SetRel(prim, name, targets);
}

double
_Clamp01(double x) { return std::min(1.0, std::max(0.0, x)); }

void
_RequireNormalizedMoverWeight(float weight)
{
    if (!std::isfinite(weight) || weight < 0.0f || weight > 1.0f) {
        throw std::invalid_argument(
            "mover defaultWeight must be finite and in [0, 1]");
    }
}

void
_RequireName(const std::string &name, const char *what)
{
    if (name.empty() || !SdfPath::IsValidIdentifier(name)) {
        throw std::invalid_argument(
            std::string(what) + " must be a valid USD identifier");
    }
}

void
_RequireTargetPath(const SdfPath &path, const char *what)
{
    if (path.IsEmpty() || !path.IsAbsolutePath() ||
        (!path.IsPrimPath() && !path.IsPropertyPath())) {
        throw std::invalid_argument(
            std::string(what) + " must be an absolute prim/property path");
    }
}

void
_RequirePrimPath(const SdfPath &path, const char *what)
{
    if (path.IsEmpty() || !path.IsAbsolutePath() || !path.IsPrimPath() ||
        path == SdfPath::AbsoluteRootPath()) {
        throw std::invalid_argument(
            std::string(what) + " must be an absolute prim path");
    }
}

void
_RequireTypedPrim(
    const UsdStageRefPtr &stage, const SdfPath &path,
    const TfToken &expectedType, const char *what)
{
    _RequirePrimPath(path, what);
    const UsdPrim prim = stage ? stage->GetPrimAtPath(path) : UsdPrim();
    if (!prim) {
        throw std::invalid_argument(
            std::string(what) + " does not exist: " + path.GetString());
    }
    if (prim.GetTypeName() != expectedType ||
        prim.GetPrimTypeInfo().GetSchemaTypeName() != expectedType) {
        throw std::invalid_argument(
            std::string(what) + " must be a " + expectedType.GetString() +
            ": " + path.GetString());
    }
}

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
    _SetRel(GetPrim(), name, targets);
}

void
RigExecHandleBase::ApplyApi(const TfToken &apiSchemaName)
{
    if (!IsValid()) {
        throw std::runtime_error("invalid handle for API application");
    }
    _ApplyApiRequired(GetPrim(), apiSchemaName);
}

void
RigExecMoverHandle::SetEnabled(bool enabled)
{
    _AuthorAttr(
        GetPrim(), "inputs:enabled", SdfValueTypeNames->Bool,
        VtValue(enabled));
}

void
RigExecMoverHandle::SetDefaultWeight(float weight)
{
    _RequireNormalizedMoverWeight(weight);
    _AuthorAttr(
        GetPrim(), "inputs:defaultWeight", SdfValueTypeNames->Float,
        VtValue(weight));
}

void
RigExecMoverHandle::SetWeightObject(const SdfPath &path)
{
    SetRel("rigExec:weightObject", path.IsEmpty() ? std::vector<SdfPath>()
                                                   : std::vector<SdfPath>{ path });
}

void
RigExecMoverHandle::SetMoves(const std::vector<SdfPath> &targets)
{
    SetRel("rigExec:moves", targets);
}

void
RigExecMoverHandle::SetReadPhase(
    const TfToken &propertyName, const std::string &phase)
{
    _Schema(GetPrim()).SetReadPhase(propertyName, phase);
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

void
_SetAvarScale(RigExecHandleBase *self, double sx, double sy, double sz)
{
    const UsdPrim prim = self->GetPrim();
    // Validate the complete triplet before authoring its first component.
    // This keeps a stale/mismatched schema from leaving a partial scale
    // opinion when, for example, only two of the three names are declared.
    static const char *names[3] = {
        "avars:sx", "avars:sy", "avars:sz"};
    const double requested[3] = {sx, sy, sz};
    double normalized[3];
    for (int axis = 0; axis < 3; ++axis) {
        _RequireAttrDefinition(
            prim, names[axis], SdfValueTypeNames->Double);
        if (!RigExecIsFiniteAvarScale(requested[axis])) {
            throw std::invalid_argument(
                std::string(names[axis]) + " must be finite at " +
                prim.GetPath().GetString());
        }
        normalized[axis] = RigExecNormalizeAvarScale(requested[axis]);
    }
    for (int axis = 0; axis < 3; ++axis) {
        _AuthorAttr(
            prim, names[axis], SdfValueTypeNames->Double,
            VtValue(normalized[axis]));
    }
}

void
_SetAvarSpin(RigExecHandleBase *self, double degrees)
{
    _AuthorAttr(
        self->GetPrim(), "avars:rspin", SdfValueTypeNames->Double,
        VtValue(degrees));
}

}  // namespace

void RigExecControlHandle::SetRestSpace(const GfMatrix4d &m)
{ _SetRestSpace(this, m); }
void RigExecControlHandle::SetAvarTranslation(double tx, double ty, double tz)
{ _SetAvarTranslation(this, tx, ty, tz); }
void RigExecControlHandle::SetAvarRotation(
    double rx, double ry, double rz, const TfToken &order)
{ _SetAvarRotation(this, rx, ry, rz, order); }
void RigExecControlHandle::SetAvarScale(double sx, double sy, double sz)
{ _SetAvarScale(this, sx, sy, sz); }
void RigExecControlHandle::SetAvarSpin(double degrees)
{ _SetAvarSpin(this, degrees); }

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
void RigExecJointHandle::SetAvarScale(double sx, double sy, double sz)
{ _SetAvarScale(this, sx, sy, sz); }
void RigExecJointHandle::SetAvarSpin(double degrees)
{ _SetAvarSpin(this, degrees); }

// ---------------------------------------------------------------------------
// Solvers
// ---------------------------------------------------------------------------

void
RigExecSolverHandle::SetJoints(const std::vector<RigExecJointHandle> &joints)
{
    std::vector<SdfPath> paths;
    paths.reserve(joints.size());
    for (const auto &j : joints) {
        if (!j.IsValid() || j.GetStage() != _stage ||
            j.GetSchemaTypeName() != TfToken("RigExecJoint")) {
            throw std::invalid_argument(
                "SetJoints requires RigExecJoint handles from this stage");
        }
        paths.push_back(j.GetPath());
    }
    _ClearJointElementsIfTargetsChange(GetPrim(), paths);
    SetRel("rigExec:joints", paths);
}

void
RigExecSolverHandle::SetJoints(const std::vector<SdfPath> &paths)
{
    for (const SdfPath &path : paths) {
        _RequireTypedPrim(
            _stage, path, TfToken("RigExecJoint"), "solver joint");
    }
    _ClearJointElementsIfTargetsChange(GetPrim(), paths);
    SetRel("rigExec:joints", paths);
}

void
RigExecSolverHandle::_SetSingleRel(const char *name, const SdfPath &target)
{
    SetRel(name, target.IsEmpty() ? std::vector<SdfPath>()
                                  : std::vector<SdfPath>{ target });
}

void
RigExecFkChainHandle::SetControls(
    const std::vector<RigExecControlHandle> &controls)
{
    std::vector<SdfPath> paths;
    paths.reserve(controls.size());
    for (const auto &c : controls) {
        if (!c.IsValid() || c.GetStage() != _stage ||
            c.GetSchemaTypeName() != TfToken("RigExecControl")) {
            throw std::invalid_argument(
                "SetControls requires RigExecControl handles from this stage");
        }
        paths.push_back(c.GetPath());
    }
    SetRel("rigExec:controls", paths);
}

void
RigExecFkChainHandle::SetControls(const std::vector<SdfPath> &paths)
{
    for (const SdfPath &path : paths) {
        _RequireTypedPrim(
            _stage, path, TfToken("RigExecControl"), "FK control");
    }
    SetRel("rigExec:controls", paths);
}

void RigExecTwoBoneIkHandle::SetRootControl(const SdfPath &path)
{
    _RequireTypedPrim(
        _stage, path, TfToken("RigExecControl"), "TwoBoneIK root control");
    _SetSingleRel("rigExec:rootControl", path);
}
void RigExecTwoBoneIkHandle::SetEffectorControl(const SdfPath &path)
{
    _RequireTypedPrim(
        _stage, path, TfToken("RigExecControl"),
        "TwoBoneIK effector control");
    _SetSingleRel("rigExec:effectorControl", path);
}
void RigExecTwoBoneIkHandle::SetPoleControl(const SdfPath &path)
{
    if (!path.IsEmpty()) {
        _RequireTypedPrim(
            _stage, path, TfToken("RigExecControl"),
            "TwoBoneIK pole control");
    }
    _SetSingleRel("rigExec:poleControl", path);
}

void RigExecTwoBoneIkHandle::SetUpperLength(double length)
{ _AuthorAttr(GetPrim(), "rigExec:upperLength", SdfValueTypeNames->Double, VtValue(length)); }
void RigExecTwoBoneIkHandle::SetLowerLength(double length)
{ _AuthorAttr(GetPrim(), "rigExec:lowerLength", SdfValueTypeNames->Double, VtValue(length)); }
void RigExecTwoBoneIkHandle::SetUpperLengthOffset(double offset)
{ _AuthorAttr(GetPrim(), "rigExec:upperLengthOffset", SdfValueTypeNames->Double, VtValue(offset)); }
void RigExecTwoBoneIkHandle::SetLowerLengthOffset(double offset)
{ _AuthorAttr(GetPrim(), "rigExec:lowerLengthOffset", SdfValueTypeNames->Double, VtValue(offset)); }
void RigExecTwoBoneIkHandle::SetPreferredBendRadians(double radians)
{ _AuthorAttr(GetPrim(), "rigExec:preferredBendRadians", SdfValueTypeNames->Double, VtValue(radians)); }
void RigExecTwoBoneIkHandle::SetStretch(float stretch)
{ _AuthorAttr(GetPrim(), "inputs:stretch", SdfValueTypeNames->Float, VtValue(stretch)); }
void RigExecTwoBoneIkHandle::SetSoftness(float softness)
{ _AuthorAttr(GetPrim(), "inputs:softness", SdfValueTypeNames->Float, VtValue(softness)); }

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
RigExecConstraintHandle::SetLocked(bool locked)
{
    _AuthorAttr(
        GetPrim(), "rigExec:locked", SdfValueTypeNames->Bool,
        VtValue(locked));
}

namespace {

void
_SetConstraintMask(
    RigExecHandleBase *self, const char *group, bool x, bool y, bool z)
{
    const std::string prefix = std::string("inputs:affect") + group;
    _AuthorAttr(self->GetPrim(), (prefix + "X").c_str(),
                SdfValueTypeNames->Bool, VtValue(x));
    _AuthorAttr(self->GetPrim(), (prefix + "Y").c_str(),
                SdfValueTypeNames->Bool, VtValue(y));
    _AuthorAttr(self->GetPrim(), (prefix + "Z").c_str(),
                SdfValueTypeNames->Bool, VtValue(z));
}

void
_SetConstraintOffset(
    RigExecHandleBase *self, const char *name, double x, double y, double z)
{
    _AuthorAttr(
        self->GetPrim(), name, SdfValueTypeNames->Double3,
        VtValue(GfVec3d(x, y, z)));
}

void
_SetConstraintRotationOrder(
    RigExecHandleBase *self, const TfToken &order)
{
    _AuthorAttr(
        self->GetPrim(), "rigExec:rotationOrder", SdfValueTypeNames->Token,
        VtValue(order));
}

size_t
_RelationshipTargetCount(const UsdPrim &prim, const char *name)
{
    SdfPathVector targets;
    const UsdRelationship relationship = prim.GetRelationship(TfToken(name));
    if (!relationship || !relationship.GetTargets(&targets)) {
        return 0;
    }
    return targets.size();
}

void
_ClearSourceOffsetsIfTargetsChange(
    const UsdPrim &prim, const std::vector<SdfPath> &newTargets)
{
    SdfPathVector oldTargets;
    const UsdRelationship sources =
        prim.GetRelationship(TfToken("rigExec:sources"));
    if (sources && sources.GetTargets(&oldTargets) &&
        oldTargets == newTargets) {
        return;
    }

    // Parent-constraint offsets are parallel to rigExec:sources. Other source
    // constraints do not declare these properties, so consult the composed
    // definition before clearing instead of manufacturing an attribute.
    const UsdPrimDefinition &definition = prim.GetPrimDefinition();
    for (const char *name : {
             "inputs:translationOffsets", "inputs:rotationOffsets"}) {
        const TfToken token(name);
        if (definition.GetAttributeDefinition(token)) {
            _ClearAttr(prim, name);
        }
    }
}

void
_ClearJointElementsIfTargetsChange(
    const UsdPrim &prim, const std::vector<SdfPath> &newTargets)
{
    SdfPathVector oldTargets;
    const UsdRelationship joints =
        prim.GetRelationship(TfToken("rigExec:joints"));
    if (joints && joints.GetTargets(&oldTargets) && oldTargets == newTargets) {
        return;
    }
    const TfToken elements("rigExec:jointElements");
    if (prim.GetPrimDefinition().GetAttributeDefinition(elements)) {
        _ClearAttr(prim, elements.GetText());
    }
}

}  // namespace

void
RigExecConstraintHandle::_SetSingleRel(const char *name, const SdfPath &target)
{
    SetRel(name, target.IsEmpty() ? std::vector<SdfPath>()
                                  : std::vector<SdfPath>{ target });
}

void
RigExecSourceConstraintHandle::SetSources(const std::vector<SdfPath> &paths)
{
    for (const SdfPath &path : paths) {
        _RequireTargetPath(path, "constraint source");
    }
    // Changing the source list invalidates every parallel payload. Clear old
    // values so they cannot silently acquire a new source ordering.
    _ClearSourceOffsetsIfTargetsChange(GetPrim(), paths);
    _ClearAttr(GetPrim(), "inputs:sourceWeights");
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
    for (const SdfPath &path : paths) {
        _RequireTargetPath(path, "constraint source");
    }
    _ClearSourceOffsetsIfTargetsChange(GetPrim(), paths);
    SetRel("rigExec:sources", paths);
    if (weights.empty()) {
        _ClearAttr(GetPrim(), "inputs:sourceWeights");
    } else {
        SetSourceWeights(weights);
    }
}

void
RigExecSourceConstraintHandle::SetSourceWeights(
    const std::vector<float> &weights)
{
    const size_t sourceCount =
        _RelationshipTargetCount(GetPrim(), "rigExec:sources");
    if (!weights.empty() && weights.size() != sourceCount) {
        throw std::invalid_argument(
            "sourceWeights must be empty or exactly one entry per current source");
    }
    if (weights.empty()) {
        _ClearAttr(GetPrim(), "inputs:sourceWeights");
        return;
    }
    _AuthorAttr(
        GetPrim(), "inputs:sourceWeights", SdfValueTypeNames->FloatArray,
        VtValue(VtFloatArray(weights.begin(), weights.end())));
}

void RigExecAimConstraintHandle::SetAffectRotation(bool x, bool y, bool z)
{ _SetConstraintMask(this, "Rotation", x, y, z); }
void RigExecAimConstraintHandle::SetRotationOffset(double x, double y, double z)
{ _SetConstraintOffset(this, "inputs:rotationOffset", x, y, z); }
void RigExecAimConstraintHandle::SetRotationOrder(const TfToken &order)
{ _SetConstraintRotationOrder(this, order); }

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
RigExecAimConstraintHandle::SetWorldUpVector(double x, double y, double z)
{
    _AuthorAttr(
        GetPrim(), "inputs:worldUpVector", SdfValueTypeNames->Double3,
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

void RigExecPositionConstraintHandle::SetAffectTranslation(bool x, bool y, bool z)
{ _SetConstraintMask(this, "Translation", x, y, z); }
void RigExecPositionConstraintHandle::SetTranslationOffset(double x, double y, double z)
{ _SetConstraintOffset(this, "inputs:translationOffset", x, y, z); }

void RigExecRotationConstraintHandle::SetAffectRotation(bool x, bool y, bool z)
{ _SetConstraintMask(this, "Rotation", x, y, z); }
void RigExecRotationConstraintHandle::SetRotationOffset(double x, double y, double z)
{ _SetConstraintOffset(this, "inputs:rotationOffset", x, y, z); }
void RigExecRotationConstraintHandle::SetRotationOrder(const TfToken &order)
{ _SetConstraintRotationOrder(this, order); }

void RigExecScaleConstraintHandle::SetAffectScale(bool x, bool y, bool z)
{ _SetConstraintMask(this, "Scale", x, y, z); }
void RigExecScaleConstraintHandle::SetScaleOffset(double x, double y, double z)
{ _SetConstraintOffset(this, "inputs:scaleOffset", x, y, z); }

void RigExecParentConstraintHandle::SetAffectTranslation(bool x, bool y, bool z)
{ _SetConstraintMask(this, "Translation", x, y, z); }
void RigExecParentConstraintHandle::SetAffectRotation(bool x, bool y, bool z)
{ _SetConstraintMask(this, "Rotation", x, y, z); }
void RigExecParentConstraintHandle::SetAffectScale(bool x, bool y, bool z)
{ _SetConstraintMask(this, "Scale", x, y, z); }
void RigExecParentConstraintHandle::SetRotationOrder(const TfToken &order)
{ _SetConstraintRotationOrder(this, order); }

void
RigExecParentConstraintHandle::SetTranslationOffsets(
    const std::vector<GfVec3d> &offsets)
{
    const size_t sourceCount =
        _RelationshipTargetCount(GetPrim(), "rigExec:sources");
    if (!offsets.empty() && offsets.size() != sourceCount) {
        throw std::invalid_argument(
            "translationOffsets must be empty or parallel to current sources");
    }
    _AuthorAttr(
        GetPrim(), "inputs:translationOffsets", SdfValueTypeNames->Double3Array,
        VtValue(VtArray<GfVec3d>(offsets.begin(), offsets.end())));
}

void
RigExecParentConstraintHandle::SetRotationOffsets(
    const std::vector<GfVec3d> &degrees)
{
    const size_t sourceCount =
        _RelationshipTargetCount(GetPrim(), "rigExec:sources");
    if (!degrees.empty() && degrees.size() != sourceCount) {
        throw std::invalid_argument(
            "rotationOffsets must be empty or parallel to current sources");
    }
    _AuthorAttr(
        GetPrim(), "inputs:rotationOffsets", SdfValueTypeNames->Double3Array,
        VtValue(VtArray<GfVec3d>(degrees.begin(), degrees.end())));
}

void
RigExecSingleChainIkConstraintHandle::SetFirstJoint(const SdfPath &path)
{
    _RequireTypedPrim(
        _stage, path, TfToken("RigExecJoint"), "IK firstJoint");
    _SetSingleRel("rigExec:firstJoint", path);
}

void
RigExecSingleChainIkConstraintHandle::SetEndJoint(const SdfPath &path)
{
    _RequireTypedPrim(
        _stage, path, TfToken("RigExecJoint"), "IK endJoint");
    _SetSingleRel("rigExec:endJoint", path);
}

void
RigExecSingleChainIkConstraintHandle::SetEffector(const SdfPath &path)
{
    if (path.IsEmpty()) {
        throw std::invalid_argument("effector must not be empty");
    }
    _SetSingleRel("rigExec:effector", path);
}

void
RigExecSingleChainIkConstraintHandle::SetMoves(
    const std::vector<SdfPath> &paths)
{
    if (paths.empty()) {
        throw std::invalid_argument("IK moves chain must not be empty");
    }
    for (const SdfPath &path : paths) {
        _RequireTypedPrim(
            _stage, path, TfToken("RigExecJoint"), "IK moves joint");
    }
    SetRel("rigExec:moves", paths);
}

void
RigExecSingleChainIkConstraintHandle::SetPoleVectorObjects(
    const std::vector<SdfPath> &paths)
{
    for (const SdfPath &path : paths) {
        _RequireTargetPath(path, "pole-vector object");
    }
    _ClearAttr(GetPrim(), "inputs:poleVectorWeights");
    SetRel("rigExec:poleVectorObjects", paths);
}

void
RigExecSingleChainIkConstraintHandle::SetPoleVectorWeights(
    const std::vector<float> &weights)
{
    const size_t objectCount =
        _RelationshipTargetCount(GetPrim(), "rigExec:poleVectorObjects");
    if (!weights.empty() && weights.size() != objectCount) {
        throw std::invalid_argument(
            "poleVectorWeights must be empty or parallel to poleVectorObjects");
    }
    if (weights.empty()) {
        _ClearAttr(GetPrim(), "inputs:poleVectorWeights");
        return;
    }
    _AuthorAttr(
        GetPrim(), "inputs:poleVectorWeights", SdfValueTypeNames->FloatArray,
        VtValue(VtFloatArray(weights.begin(), weights.end())));
}

void
RigExecSingleChainIkConstraintHandle::SetPoleVector(double x, double y, double z)
{
    _AuthorAttr(
        GetPrim(), "inputs:poleVector", SdfValueTypeNames->Double3,
        VtValue(GfVec3d(x, y, z)));
}

void
RigExecSingleChainIkConstraintHandle::SetTwistDegrees(double degrees)
{
    _AuthorAttr(
        GetPrim(), "inputs:twistDegrees", SdfValueTypeNames->Double,
        VtValue(degrees));
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

void
RigExecSingleChainIkConstraintHandle::SetEvaluationMode(const TfToken &mode)
{
    _AuthorAttr(
        GetPrim(), "rigExec:evaluationMode", SdfValueTypeNames->Token,
        VtValue(mode));
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
    VtIntArray oldIndices;
    GetPrim().GetAttribute(TfToken("rigExec:indices")).Get(&oldIndices);
    if (!oldIndices.empty() && oldIndices.size() != values.size()) {
        _ClearAttr(GetPrim(), "rigExec:indices");
    }
    _AuthorAttr(
        GetPrim(), "rigExec:values", SdfValueTypeNames->FloatArray,
        VtValue(VtFloatArray(values.begin(), values.end())));
    if (oldIndices.empty() || oldIndices.size() != values.size()) {
        SetRepresentation(
            values.empty() ? TfToken("constant") : TfToken("dense"));
    }
}

void
RigExecStaticWeightHandle::SetIndices(const std::vector<int> &indices)
{
    for (const int index : indices) {
        if (index < 0) {
            throw std::invalid_argument(
                "static sparse weight indices must be non-negative");
        }
    }
    VtFloatArray values;
    GetPrim().GetAttribute(TfToken("rigExec:values")).Get(&values);
    if (!indices.empty() && indices.size() != values.size()) {
        throw std::invalid_argument(
            "static sparse indices must be parallel to current values");
    }
    if (indices.empty()) {
        _ClearAttr(GetPrim(), "rigExec:indices");
        SetRepresentation(
            values.empty() ? TfToken("constant") : TfToken("dense"));
        return;
    }
    _AuthorAttr(
        GetPrim(), "rigExec:indices", SdfValueTypeNames->IntArray,
        VtValue(VtIntArray(indices.begin(), indices.end())));
    SetRepresentation(TfToken("sparse"));
}

void
RigExecStaticWeightHandle::SetSparseValues(
    const std::vector<float> &values, const std::vector<int> &indices)
{
    if (indices.empty() || indices.size() != values.size()) {
        throw std::invalid_argument(
            "sparse values need one non-empty index per value");
    }
    for (const int index : indices) {
        if (index < 0) {
            throw std::invalid_argument(
                "static sparse weight indices must be non-negative");
        }
    }
    _AuthorAttr(
        GetPrim(), "rigExec:values", SdfValueTypeNames->FloatArray,
        VtValue(VtFloatArray(values.begin(), values.end())));
    _AuthorAttr(
        GetPrim(), "rigExec:indices", SdfValueTypeNames->IntArray,
        VtValue(VtIntArray(indices.begin(), indices.end())));
    SetRepresentation(TfToken("sparse"));
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
    SetRel("rigExec:baseWeight", path.IsEmpty() ? std::vector<SdfPath>()
                                                 : std::vector<SdfPath>{ path });
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
void RigExecVolumeWeightHandle::SetAvarSpin(double degrees)
{ _SetAvarSpin(this, degrees); }

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
    for (const auto &knot : knots) {
        if (!std::isfinite(knot.first) || !std::isfinite(knot.second)) {
            throw std::invalid_argument(
                "falloff curve knots must contain only finite values");
        }
    }
    const RigExecSchemaPrim schema = _Schema(GetPrim());
    // Replacing a curve must also remove knots from the previous curve.
    schema.ClearAttribute(TfToken("rigExec:falloffCurve"));
    for (const auto &knot : knots) {
        schema.SetAttribute(
            TfToken("rigExec:falloffCurve"), VtValue(float(knot.second)),
            UsdTimeCode(_Clamp01(knot.first)));
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
    SetRel("rigExec:sampleSource", path.IsEmpty() ? std::vector<SdfPath>()
                                                   : std::vector<SdfPath>{ path });
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
    if (name.empty() || !SdfPath::IsValidIdentifier(name)) {
        throw std::invalid_argument(
            "blend sample name must be a valid USD identifier");
    }
    const SdfPath samplePath = _path.AppendChild(TfToken(name));
    UsdPrim existing = _stage->GetPrimAtPath(samplePath);
    if (existing) {
        throw std::invalid_argument(
            "blend sample already exists: " + samplePath.GetString());
    }
    UsdPrim prim = RigExecSchemaPrim::Define(
        _stage, samplePath, TfToken("RigExecBlendSample")).GetPrim();
    _ApplyApiRequired(prim, _kNodeGraphApi);
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
    if (phase.IsEmpty()) {
        throw std::invalid_argument("blend sample read phase must not be empty");
    }
    _Schema(GetPrim()).SetReadPhase(
        TfToken("rigExec:targetPoints"), phase.GetString());
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
    const int requested[] = { p0, h0, h1, p1 };
    VtArray<GfVec3f> points;
    const UsdAttribute pointsAttr = GetPrim().GetAttribute(TfToken("points"));
    if (!pointsAttr || !pointsAttr.Get(&points)) {
        throw std::runtime_error("curvenet points are unavailable");
    }
    for (const int index : requested) {
        if (index < 0 || static_cast<size_t>(index) >= points.size()) {
            throw std::invalid_argument(
                "spline index is outside the current curvenet point pool");
        }
    }
    VtIntArray indices;
    const UsdAttribute attr =
        GetPrim().GetAttribute(TfToken("rigExec:splineIndices"));
    attr.Get(&indices);
    indices.push_back(p0);
    indices.push_back(h0);
    indices.push_back(h1);
    indices.push_back(p1);
    _AuthorAttr(
        GetPrim(), "rigExec:splineIndices", SdfValueTypeNames->IntArray,
        VtValue(indices));
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
RigExecMatrixMoverHandle::SetReadPhase(const TfToken &phase)
{
    if (phase.IsEmpty()) {
        throw std::invalid_argument("matrix mover read phase must not be empty");
    }
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
    if (phase.IsEmpty()) {
        throw std::invalid_argument("lattice mover read phase must not be empty");
    }
    _AuthorAttr(
        GetPrim(), "rigExec:cageReadPhase", SdfValueTypeNames->Token,
        VtValue(phase));
}

RigExecBlendInputHandle
RigExecBlendShapeMoverHandle::AddBlendInput(const std::string &name, float weight)
{
    if (name.empty() || !SdfPath::IsValidIdentifier(name)) {
        throw std::invalid_argument(
            "blend input name must be a valid USD identifier");
    }
    const SdfPath inputPath = _path.AppendChild(TfToken(name));
    UsdPrim existing = _stage->GetPrimAtPath(inputPath);
    if (existing) {
        throw std::invalid_argument(
            "blend input already exists: " + inputPath.GetString());
    }
    UsdPrim prim = RigExecSchemaPrim::Define(
        _stage, inputPath, TfToken("RigExecBlendInput")).GetPrim();
    _ApplyApiRequired(prim, _kNodeGraphApi);
    _AuthorAttr(prim, "inputs:weight", SdfValueTypeNames->Float, VtValue(weight));
    _AppendRelTarget(GetPrim(), "rigExec:blendInputs", inputPath);
    return RigExecBlendInputHandle(_stage, inputPath);
}

void
RigExecBlendShapeMoverHandle::SetBlendInputs(
    const std::vector<SdfPath> &paths)
{
    for (const SdfPath &path : paths) {
        _RequireTypedPrim(
            _stage, path, TfToken("RigExecBlendInput"), "blend input");
    }
    SetRel("rigExec:blendInputs", paths);
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
    SetRel("rigExec:driverFrames", paths);
}

void
RigExecCurveMoverHandle::SetBindCoordinates(const SdfPath &path)
{
    SetRel("rigExec:bindCoordinates", path.IsEmpty() ? std::vector<SdfPath>()
                                                      : std::vector<SdfPath>{ path });
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
    if (phase.IsEmpty()) {
        throw std::invalid_argument("curve mover read phase must not be empty");
    }
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
    if (phase.IsEmpty()) {
        throw std::invalid_argument("surface mover read phase must not be empty");
    }
    _AuthorAttr(
        GetPrim(), "rigExec:surfaceReadPhase", SdfValueTypeNames->Token,
        VtValue(phase));
}

void
RigExecSmoothMoverHandle::SetStrength(float strength)
{
    SetDefaultWeight(strength);
}

void
RigExecVolumeCorrectMoverHandle::SetStrength(float strength)
{
    SetDefaultWeight(strength);
}

void
RigExecCurvenetMoverHandle::SetCurvenet(const SdfPath &path)
{
    _RequireTypedPrim(
        _stage, path, TfToken("RigExecCurvenet"), "curvenet mover input");
    SetRel("rigExec:curvenet", { path });
}

void
RigExecCurvenetMoverHandle::SetStrength(float strength)
{
    SetDefaultWeight(strength);
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
    SetDefaultWeight(weight);
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
    SetDefaultWeight(weight);
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
    SetDefaultWeight(weight);
}

// ---------------------------------------------------------------------------
// Mover chains
// ---------------------------------------------------------------------------

SdfPath
RigExecMoverChain::_AddMoverPrim(
    const std::string &typeName, const std::string &name, const SdfPath &target)
{
    _RequireName(name, "mover name");
    const SdfPath effectiveTarget =
        target.IsEmpty() ? _defaultTarget : target;
    _RequireTargetPath(effectiveTarget, "mover target");
    if (!_stage || !_stage->GetPrimAtPath(_scope)) {
        throw std::runtime_error(
            "mover chain scope no longer exists: " + _scope.GetString());
    }
    const SdfPath path = _scope.AppendChild(TfToken(name));
    UsdPrim existing = _stage->GetPrimAtPath(path);
    if (existing) {
        throw std::invalid_argument(
            "prim already exists: " + path.GetString());
    }
    UsdPrim prim = RigExecSchemaPrim::Define(
        _stage, path, TfToken(typeName)).GetPrim();
    try {
        // RigExecMoverAPI carries the write set, enable, and common envelope
        // (inputs:defaultWeight / rigExec:weightObject); the node-graph API
        // keeps hand-authored and built rigs visually identical in usdview. A
        // failed add leaves no half-authored mover behind.
        _ApplyApiRequired(prim, _kMoverApi);
        _ApplyApiRequired(prim, _kNodeGraphApi);
        _SetRel(prim, "rigExec:moves", { effectiveTarget });
    } catch (...) {
        _stage->RemovePrim(path);
        throw;
    }
    return path;
}

RigExecMatrixMoverHandle
RigExecMoverChain::AddMatrixMover(
    const std::string &name, const SdfPath &transformProvider,
    const SdfPath &weightObject, const SdfPath &target, const TfToken &readPhase)
{
    _RequireTargetPath(transformProvider, "matrix mover transform provider");
    if (!weightObject.IsEmpty()) {
        _RequireTargetPath(weightObject, "matrix mover weight object");
    }
    RigExecMatrixMoverHandle handle(_stage, _AddMoverPrim("RigExecMatrixMover", name, target));
    handle.SetTransformProvider(transformProvider);
    if (!weightObject.IsEmpty()) {
        handle.SetWeightObject(weightObject);
    }
    if (!readPhase.IsEmpty()) {
        handle.RigExecMoverHandle::SetReadPhase(
            TfToken("rigExec:transform"), readPhase.GetString());
    }
    return handle;
}

RigExecLatticeMoverHandle
RigExecMoverChain::AddLatticeMover(
    const std::string &name, const SdfPath &cagePrim, int divX, int divY,
    int divZ, const TfToken &basis, const SdfPath &target,
    const TfToken &readPhase)
{
    _RequireTargetPath(cagePrim, "lattice mover cage");
    if (divX < 2 || divY < 2 || divZ < 2) {
        throw std::invalid_argument(
            "lattice mover divisions must be at least 2 on every axis");
    }
    RigExecLatticeMoverHandle handle(_stage, _AddMoverPrim("RigExecLatticeMover", name, target));
    handle.SetCage(cagePrim);
    if (!basis.IsEmpty()) {
        handle.SetBasis(basis);
    }
    handle.SetDivisions(divX, divY, divZ);
    if (!readPhase.IsEmpty()) {
        handle.RigExecMoverHandle::SetReadPhase(
            TfToken("rigExec:cage"), readPhase.GetString());
    }
    return handle;
}

RigExecBlendShapeMoverHandle
RigExecMoverChain::AddBlendShapeMover(
    const std::string &name, const SdfPath &weightObject, const SdfPath &target)
{
    if (!weightObject.IsEmpty()) {
        _RequireTargetPath(weightObject, "blend-shape weight object");
    }
    RigExecBlendShapeMoverHandle handle(_stage, _AddMoverPrim("RigExecBlendShapeMover", name, target));
    if (!weightObject.IsEmpty()) {
        handle.SetWeightObject(weightObject);
    }
    return handle;
}

RigExecCurveMoverHandle
RigExecMoverChain::AddCurveMover(
    const std::string &name, const SdfPath &driverCurve,
    const std::vector<SdfPath> &driverFrames, const SdfPath &bindCoordinates,
    const TfToken &mode, const SdfPath &target, const TfToken &readPhase)
{
    _RequireTargetPath(driverCurve, "curve mover driver curve");
    for (const SdfPath &driverFrame : driverFrames) {
        _RequireTargetPath(driverFrame, "curve mover driver frame");
    }
    if (!bindCoordinates.IsEmpty()) {
        _RequireTargetPath(bindCoordinates, "curve mover bind coordinates");
    }
    RigExecCurveMoverHandle handle(_stage, _AddMoverPrim("RigExecCurveMover", name, target));
    handle.SetDriverCurve(driverCurve);
    handle.SetDriverFrames(driverFrames);
    if (!bindCoordinates.IsEmpty()) {
        handle.SetBindCoordinates(bindCoordinates);
    }
    if (!mode.IsEmpty()) {
        handle.SetMode(mode);
    }
    if (!readPhase.IsEmpty()) {
        handle.RigExecMoverHandle::SetReadPhase(
            TfToken("rigExec:driverCurve"), readPhase.GetString());
    }
    return handle;
}

RigExecCurveMoverHandle
RigExecMoverChain::AddCurveMover(
    const std::string &name, const SdfPath &driverCurve,
    const SdfPath &driverFrame, const SdfPath &bindCoordinates,
    const TfToken &mode, const SdfPath &target, const TfToken &readPhase)
{
    return AddCurveMover(
        name, driverCurve,
        driverFrame.IsEmpty() ? std::vector<SdfPath>()
                              : std::vector<SdfPath>{ driverFrame },
        bindCoordinates, mode, target, readPhase);
}

RigExecSurfaceMoverHandle
RigExecMoverChain::AddSurfaceMover(
    const std::string &name, const SdfPath &surfacePrim, const TfToken &mode,
    const SdfPath &target, const TfToken &readPhase)
{
    _RequireTargetPath(surfacePrim, "surface mover surface");
    RigExecSurfaceMoverHandle handle(_stage, _AddMoverPrim("RigExecSurfaceMover", name, target));
    handle.SetSurface(surfacePrim);
    if (!mode.IsEmpty()) {
        handle.SetMode(mode);
    }
    if (!readPhase.IsEmpty()) {
        handle.RigExecMoverHandle::SetReadPhase(
            TfToken("rigExec:surface"), readPhase.GetString());
    }
    return handle;
}

RigExecSmoothMoverHandle
RigExecMoverChain::AddSmoothMover(
    const std::string &name, float defaultWeight, const SdfPath &target)
{
    _RequireNormalizedMoverWeight(defaultWeight);
    RigExecSmoothMoverHandle handle(_stage, _AddMoverPrim("RigExecSmoothMover", name, target));
    handle.SetDefaultWeight(defaultWeight);
    return handle;
}

RigExecVolumeCorrectMoverHandle
RigExecMoverChain::AddVolumeCorrectMover(
    const std::string &name, float defaultWeight, const SdfPath &target)
{
    _RequireNormalizedMoverWeight(defaultWeight);
    RigExecVolumeCorrectMoverHandle handle(_stage, _AddMoverPrim("RigExecVolumeCorrectMover", name, target));
    handle.SetDefaultWeight(defaultWeight);
    return handle;
}

RigExecCurvenetMoverHandle
RigExecMoverChain::AddCurvenetMover(
    const std::string &name, const SdfPath &curvenetPrim, float defaultWeight,
    const SdfPath &target)
{
    _RequireNormalizedMoverWeight(defaultWeight);
    _RequireTypedPrim(
        _stage, curvenetPrim, TfToken("RigExecCurvenet"),
        "curvenet mover input");
    RigExecCurvenetMoverHandle handle(_stage, _AddMoverPrim("RigExecCurvenetMover", name, target));
    handle.SetCurvenet(curvenetPrim);
    handle.SetDefaultWeight(defaultWeight);
    return handle;
}

RigExecFloatMathMoverHandle
RigExecMoverChain::AddFloatMathMover(
    const std::string &name, const TfToken &operation, float value,
    const SdfPath &target, float defaultWeight)
{
    _RequireNormalizedMoverWeight(defaultWeight);
    RigExecFloatMathMoverHandle handle(_stage, _AddMoverPrim("RigExecFloatMathMover", name, target));
    if (!operation.IsEmpty()) {
        handle.SetOperation(operation);
    }
    handle.SetValue(value);
    handle.SetDefaultWeight(defaultWeight);
    return handle;
}

RigExecVec3fMathMoverHandle
RigExecMoverChain::AddVec3fMathMover(
    const std::string &name, const TfToken &operation, GfVec3f value,
    const SdfPath &target, float defaultWeight)
{
    _RequireNormalizedMoverWeight(defaultWeight);
    RigExecVec3fMathMoverHandle handle(_stage, _AddMoverPrim("RigExecVec3fMathMover", name, target));
    if (!operation.IsEmpty()) {
        handle.SetOperation(operation);
    }
    handle.SetValue(value);
    handle.SetDefaultWeight(defaultWeight);
    return handle;
}

RigExecMatrixMathMoverHandle
RigExecMoverChain::AddMatrixMathMover(
    const std::string &name, const TfToken &operation, GfMatrix4d value,
    const SdfPath &target, float defaultWeight)
{
    _RequireNormalizedMoverWeight(defaultWeight);
    RigExecMatrixMathMoverHandle handle(_stage, _AddMoverPrim("RigExecMatrixMathMover", name, target));
    if (!operation.IsEmpty()) {
        handle.SetOperation(operation);
    }
    handle.SetValue(value);
    handle.SetDefaultWeight(defaultWeight);
    return handle;
}

// ---- Constraints as movers -------------------------------------------------
//
// Constraints are movers: the chain applies RigExecMoverAPI and authors the
// exact target on rigExec:moves, so a constraint participates in composed
// post-order application like any other operation.

namespace {

std::vector<SdfPath>
_InferSingleChainIkJoints(
    const UsdStageRefPtr &stage, const SdfPath &firstJoint,
    const SdfPath &endJoint)
{
    if (!stage) {
        throw std::invalid_argument("SingleChainIK needs a valid stage");
    }
    _RequirePrimPath(firstJoint, "SingleChainIK firstJoint");
    _RequirePrimPath(endJoint, "SingleChainIK endJoint");

    std::vector<SdfPath> reversed;
    SdfPath cursor = endJoint;
    while (cursor != SdfPath::AbsoluteRootPath()) {
        const UsdPrim joint = stage->GetPrimAtPath(cursor);
        if (!joint || joint.GetTypeName() != TfToken("RigExecJoint") ||
            joint.GetPrimTypeInfo().GetSchemaTypeName() !=
                TfToken("RigExecJoint")) {
            throw std::invalid_argument(
                "SingleChainIK ancestry contains non-RigExecJoint " +
                cursor.GetString());
        }
        reversed.push_back(cursor);
        if (cursor == firstJoint) {
            break;
        }
        cursor = cursor.GetParentPath();
    }
    if (reversed.empty() || reversed.back() != firstJoint) {
        throw std::invalid_argument(
            "SingleChainIK endJoint must be a namespace descendant of firstJoint");
    }
    if (reversed.size() < 2) {
        throw std::invalid_argument("SingleChainIK needs at least two joints");
    }
    std::reverse(reversed.begin(), reversed.end());
    return reversed;
}

void
_PreflightExistingPrim(
    const UsdStageRefPtr &stage, const SdfPath &path, const char *what)
{
    _RequirePrimPath(path, what);
    if (!stage || !stage->GetPrimAtPath(path)) {
        throw std::invalid_argument(
            std::string(what) + " does not exist: " + path.GetString());
    }
}

}  // namespace

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
RigExecMoverChain::AddSingleChainIkConstraint(
    const std::string &name, const SdfPath &firstJoint,
    const SdfPath &endJoint, const SdfPath &effector,
    const std::vector<SdfPath> &poleVectorObjects)
{
    const std::vector<SdfPath> moves =
        _InferSingleChainIkJoints(_stage, firstJoint, endJoint);
    return AddSingleChainIkConstraint(
        name, moves, firstJoint, endJoint, effector, poleVectorObjects);
}

RigExecSingleChainIkConstraintHandle
RigExecMoverChain::AddSingleChainIkConstraint(
    const std::string &name, const std::vector<SdfPath> &moves,
    const SdfPath &firstJoint, const SdfPath &endJoint,
    const SdfPath &effector,
    const std::vector<SdfPath> &poleVectorObjects)
{
    _RequireName(name, "SingleChainIK name");
    const std::vector<SdfPath> inferred =
        _InferSingleChainIkJoints(_stage, firstJoint, endJoint);
    if (moves != inferred) {
        throw std::invalid_argument(
            "SingleChainIK moves must exactly equal the ordered inferred chain");
    }
    _PreflightExistingPrim(_stage, effector, "SingleChainIK effector");
    for (const SdfPath &pole : poleVectorObjects) {
        _PreflightExistingPrim(
            _stage, pole, "SingleChainIK pole-vector object");
    }

    RigExecSingleChainIkConstraintHandle handle(
        _stage, _AddMoverPrim(
            "RigExecSingleChainIkConstraint", name, inferred.front()));
    handle.SetMoves(inferred);
    handle.SetFirstJoint(firstJoint);
    handle.SetEndJoint(endJoint);
    handle.SetEffector(effector);
    handle.SetPoleVectorObjects(poleVectorObjects);
    return handle;
}

RigExecMoverChain
RigExecMoverChain::Under(
    const RigExecHandleBase &mover, const SdfPath &defaultTarget) const
{
    if (!mover.IsValid() || mover.GetStage() != _stage) {
        throw std::invalid_argument(
            "Under needs a valid mover handle from this stage");
    }
    const SdfPath moverPath = mover.GetPath();
    if (moverPath == _scope || !moverPath.HasPrefix(_scope)) {
        throw std::invalid_argument(
            "Under mover must be a descendant of this mover chain");
    }
    const RigExecSchemaPrim schema = _Schema(mover.GetPrim());
    if (!schema.HasAPI(_kMoverApi)) {
        throw std::invalid_argument(
            "Under parent does not carry RigExecMoverAPI: " +
            moverPath.GetString());
    }
    const SdfPath effectiveDefault =
        defaultTarget.IsEmpty() ? _defaultTarget : defaultTarget;
    if (!effectiveDefault.IsEmpty()) {
        _RequireTargetPath(effectiveDefault, "nested mover default target");
    }
    return RigExecMoverChain(_stage, moverPath, effectiveDefault);
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
    _RequirePrimPath(rigRoot, "rig root");

    UsdPrim root = RigExecSchemaPrim::Define(
        stage, rigRoot, TfToken("RigExecRoot")).GetPrim();

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
    _RequireName(scopeName ? std::string(scopeName) : std::string(),
                 "scope name");
    const SdfPath path = _root.AppendChild(TfToken(scopeName));
    RigExecSchemaPrim::Define(_stage, path, TfToken("Scope"));
    return path;
}

UsdPrim
RigExecRigBuilder::_DefineTyped(
    const SdfPath &parent, const std::string &typeName, const std::string &name)
{
    _RequireName(name, "prim name");
    const SdfPath path = parent.AppendChild(TfToken(name));
    if (_stage->GetPrimAtPath(path)) {
        throw std::invalid_argument(
            "rig object already exists: " + path.GetString());
    }
    UsdPrim prim = _DefineTypedAt(path, typeName);
    try {
        // Every object created by an Add* call participates in the node graph.
        // Controls and movers additionally carry their semantic applied API.
        if (typeName == "RigExecControl") {
            _ApplyApiRequired(prim, _kControlApi);
        }
        if (typeName.size() >= 5 &&
            (typeName.compare(typeName.size() - 5, 5, "Mover") == 0 ||
             (typeName.size() >= 10 &&
              typeName.compare(
                  typeName.size() - 10, 10, "Constraint") == 0))) {
            _ApplyApiRequired(prim, _kMoverApi);
        }
        _ApplyApiRequired(prim, _kNodeGraphApi);
    } catch (...) {
        _stage->RemovePrim(path);
        throw;
    }
    return prim;
}

UsdPrim
RigExecRigBuilder::_DefineTypedAt(
    const SdfPath &path, const std::string &typeName)
{
    _RequirePrimPath(path, "rig object path");
    if (path == _root || !path.HasPrefix(_root)) {
        throw std::invalid_argument(
            "rig object path must be a proper descendant of " +
            _root.GetString());
    }
    const SdfPath parent = path.GetParentPath();
    if (!_stage->GetPrimAtPath(parent)) {
        throw std::invalid_argument(
            "rig object parent must already exist: " + parent.GetString());
    }
    return RigExecSchemaPrim::Define(
        _stage, path, TfToken(typeName)).GetPrim();
}

RigExecControlHandle
RigExecRigBuilder::AddControl(const std::string &name, const GfMatrix4d &restSpace)
{
    _RequireName(name, "control name");
    const SdfPath scope = _EnsureScope("Controls");
    UsdPrim prim = _DefineTyped(scope, "RigExecControl", name);
    _ApplyApiRequired(prim, _kControlApi);
    _ApplyApiRequired(prim, _kNodeGraphApi);
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
    _RequireName(name, "joint name");
    SdfPath parent;
    if (parentJoint) {
        if (!parentJoint->IsValid() || parentJoint->GetStage() != _stage ||
            parentJoint->GetSchemaTypeName() != TfToken("RigExecJoint")) {
            throw std::invalid_argument(
                "parentJoint must be a valid RigExecJoint on this stage");
        }
        parent = parentJoint->GetPath();
        const SdfPath jointsRoot =
            _root.AppendChild(TfToken("Joints"));
        if (parent == jointsRoot || !parent.HasPrefix(jointsRoot)) {
            throw std::invalid_argument(
                "parentJoint must belong to this builder's Joints hierarchy");
        }
    } else {
        parent = _EnsureScope("Joints");
    }
    UsdPrim prim = _DefineTyped(parent, "RigExecJoint", name);
    _ApplyApiRequired(prim, _kNodeGraphApi);
    RigExecJointHandle handle(_stage, prim.GetPath());
    if (restSpace != GfMatrix4d()) {
        handle.SetRestSpace(restSpace);
    }
    return handle;
}

RigExecFkChainHandle
RigExecRigBuilder::AddFkChain(const std::string &name)
{
    _RequireName(name, "FK chain name");
    const SdfPath scope = _EnsureScope("Solvers");
    UsdPrim prim = _DefineTyped(scope, "RigExecFkChain", name);
    _ApplyApiRequired(prim, _kNodeGraphApi);
    return RigExecFkChainHandle(_stage, prim.GetPath());
}

RigExecTwoBoneIkHandle
RigExecRigBuilder::AddTwoBoneIk(
    const std::string &name, const SdfPath &rootControl,
    const SdfPath &effectorControl, const SdfPath &poleControl)
{
    _RequireName(name, "TwoBoneIK name");
    _RequireTypedPrim(
        _stage, rootControl, TfToken("RigExecControl"),
        "TwoBoneIK root control");
    _RequireTypedPrim(
        _stage, effectorControl, TfToken("RigExecControl"),
        "TwoBoneIK effector control");
    if (!poleControl.IsEmpty()) {
        _RequireTypedPrim(
            _stage, poleControl, TfToken("RigExecControl"),
            "TwoBoneIK pole control");
    }
    const SdfPath scope = _EnsureScope("Solvers");
    UsdPrim prim = _DefineTyped(scope, "RigExecTwoBoneIk", name);
    _ApplyApiRequired(prim, _kNodeGraphApi);
    RigExecTwoBoneIkHandle handle(_stage, prim.GetPath());
    handle.SetRootControl(rootControl);
    handle.SetEffectorControl(effectorControl);
    handle.SetPoleControl(poleControl);
    return handle;
}

RigExecBlendPointFramesHandle
RigExecRigBuilder::AddBlendPointFrames(
    const std::string &name, const SdfPath &inputA, const SdfPath &inputB,
    float weight)
{
    _RequireName(name, "frame blend name");
    _RequireTargetPath(inputA, "frame blend inputA");
    _RequireTargetPath(inputB, "frame blend inputB");
    const SdfPath scope = _EnsureScope("Solvers");
    UsdPrim prim = _DefineTyped(scope, "RigExecBlendPointFrames", name);
    _ApplyApiRequired(prim, _kNodeGraphApi);
    RigExecBlendPointFramesHandle handle(_stage, prim.GetPath());
    handle.SetInputA(inputA);
    handle.SetInputB(inputB);
    handle.SetWeight(weight);
    return handle;
}

RigExecTwistDistributionHandle
RigExecRigBuilder::AddTwistDistribution(
    const std::string &name, const SdfPath &start, const SdfPath &end, int count)
{
    _RequireName(name, "twist distribution name");
    _RequireTargetPath(start, "twist distribution start");
    _RequireTargetPath(end, "twist distribution end");
    if (count < 1) {
        throw std::invalid_argument("twist distribution count must be positive");
    }
    const SdfPath scope = _EnsureScope("Solvers");
    UsdPrim prim = _DefineTyped(scope, "RigExecTwistDistribution", name);
    _ApplyApiRequired(prim, _kNodeGraphApi);
    RigExecTwistDistributionHandle handle(_stage, prim.GetPath());
    handle.SetStart(start);
    handle.SetEnd(end);
    handle.SetCount(count);
    return handle;
}

RigExecRibbonHandle
RigExecRigBuilder::AddRibbon(
    const std::string &name, const SdfPath &driverCurve, int sampleCount)
{
    _RequireName(name, "ribbon name");
    _RequireTargetPath(driverCurve, "ribbon driver curve");
    if (sampleCount < 1) {
        throw std::invalid_argument("ribbon sample count must be positive");
    }
    const SdfPath scope = _EnsureScope("Solvers");
    UsdPrim prim = _DefineTyped(scope, "RigExecRibbon", name);
    _ApplyApiRequired(prim, _kNodeGraphApi);
    RigExecRibbonHandle handle(_stage, prim.GetPath());
    handle.SetDriverCurve(driverCurve);
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
    _RequireName(name, "static weight name");
    _RequireTargetPath(target, "static weight target");
    if (!indices.empty() && indices.size() != values.size()) {
        throw std::invalid_argument(
            "static sparse indices must be parallel to values");
    }
    for (const int index : indices) {
        if (index < 0) {
            throw std::invalid_argument(
                "static sparse weight indices must be non-negative");
        }
    }
    const SdfPath scope = _EnsureScope("Weights");
    UsdPrim prim = _DefineTyped(scope, "RigExecStaticWeight", name);
    _ApplyApiRequired(prim, _kNodeGraphApi);
    RigExecStaticWeightHandle handle(_stage, prim.GetPath());
    handle.SetTarget(target);
    // Always replace both parallel arrays so reopening an existing builder
    // object cannot retain stale values or indices from its previous mode.
    handle.SetValues(values);
    handle.SetIndices(indices);
    handle.SetRepresentation(
        !indices.empty() ? TfToken("sparse")
                         : (!values.empty() ? TfToken("dense")
                                            : TfToken("constant")));
    handle.SetDefaultWeight(defaultWeight);
    return handle;
}

RigExecDynamicWeightHandle
RigExecRigBuilder::AddDynamicWeight(
    const std::string &name, const SdfPath &target, const SdfPath &baseWeight)
{
    _RequireName(name, "dynamic weight name");
    _RequireTargetPath(target, "dynamic weight target");
    if (!baseWeight.IsEmpty()) {
        _RequireTargetPath(baseWeight, "dynamic base weight");
    }
    const SdfPath scope = _EnsureScope("Weights");
    UsdPrim prim = _DefineTyped(scope, "RigExecDynamicWeight", name);
    _ApplyApiRequired(prim, _kNodeGraphApi);
    RigExecDynamicWeightHandle handle(_stage, prim.GetPath());
    handle.SetTarget(target);
    handle.SetBaseWeight(baseWeight);
    return handle;
}

RigExecSphereWeightHandle
RigExecRigBuilder::AddSphereWeight(
    const std::string &name, const SdfPath &target, float falloffMin,
    float falloffMax)
{
    _RequireName(name, "sphere weight name");
    _RequireTargetPath(target, "sphere weight target");
    const SdfPath scope = _EnsureScope("Weights");
    return DefineSphereWeight(
        scope.AppendChild(TfToken(name)), target, falloffMin, falloffMax);
}

RigExecPlaneWeightHandle
RigExecRigBuilder::AddPlaneWeight(
    const std::string &name, const SdfPath &target, float falloffMin,
    float falloffMax)
{
    _RequireName(name, "plane weight name");
    _RequireTargetPath(target, "plane weight target");
    const SdfPath scope = _EnsureScope("Weights");
    return DefinePlaneWeight(
        scope.AppendChild(TfToken(name)), target, falloffMin, falloffMax);
}

RigExecCurveWeightHandle
RigExecRigBuilder::AddCurveWeight(
    const std::string &name, const SdfPath &target, const SdfPath &curve,
    float falloffMin, float falloffMax)
{
    _RequireName(name, "curve weight name");
    _RequireTargetPath(target, "curve weight target");
    _RequireTargetPath(curve, "curve weight source");
    const SdfPath scope = _EnsureScope("Weights");
    return DefineCurveWeight(
        scope.AppendChild(TfToken(name)), target, curve,
        falloffMin, falloffMax);
}

RigExecSphereWeightHandle
RigExecRigBuilder::DefineSphereWeight(
    const SdfPath &path, const SdfPath &target, float falloffMin,
    float falloffMax)
{
    _RequireTargetPath(target, "sphere weight target");
    UsdPrim prim = _DefineTypedAt(path, "RigExecSphereWeight");
    _ApplyApiRequired(prim, _kNodeGraphApi);
    RigExecSphereWeightHandle handle(_stage, path);
    handle.SetTarget(target);
    handle.SetFalloff(falloffMin, falloffMax);
    return handle;
}

RigExecPlaneWeightHandle
RigExecRigBuilder::DefinePlaneWeight(
    const SdfPath &path, const SdfPath &target, float falloffMin,
    float falloffMax)
{
    _RequireTargetPath(target, "plane weight target");
    UsdPrim prim = _DefineTypedAt(path, "RigExecPlaneWeight");
    _ApplyApiRequired(prim, _kNodeGraphApi);
    RigExecPlaneWeightHandle handle(_stage, path);
    handle.SetTarget(target);
    handle.SetFalloff(falloffMin, falloffMax);
    return handle;
}

RigExecCurveWeightHandle
RigExecRigBuilder::DefineCurveWeight(
    const SdfPath &path, const SdfPath &target, const SdfPath &curve,
    float falloffMin, float falloffMax)
{
    _RequireTargetPath(target, "curve weight target");
    _RequireTargetPath(curve, "curve weight source");
    UsdPrim prim = _DefineTypedAt(path, "RigExecCurveWeight");
    _ApplyApiRequired(prim, _kNodeGraphApi);
    RigExecCurveWeightHandle handle(_stage, path);
    handle.SetTarget(target);
    handle.SetCurve(curve);
    handle.SetFalloff(falloffMin, falloffMax);
    return handle;
}

RigExecCombineWeightHandle
RigExecRigBuilder::AddCombineWeight(
    const std::string &name, const SdfPath &target,
    const std::vector<SdfPath> &inputWeights, const TfToken &mode)
{
    _RequireName(name, "combine weight name");
    _RequireTargetPath(target, "combine weight target");
    if (inputWeights.empty()) {
        throw std::invalid_argument(
            "combine weight needs at least one input weight");
    }
    for (const SdfPath &input : inputWeights) {
        _RequireTargetPath(input, "combine input weight");
    }
    const SdfPath scope = _EnsureScope("Weights");
    UsdPrim prim = _DefineTyped(scope, "RigExecCombineWeight", name);
    _ApplyApiRequired(prim, _kNodeGraphApi);
    RigExecCombineWeightHandle handle(_stage, prim.GetPath());
    handle.SetTarget(target);
    handle.SetInputWeights(inputWeights);
    if (!mode.IsEmpty()) {
        handle.SetCombineMode(mode);
    }
    return handle;
}

RigExecBlendInputHandle
RigExecRigBuilder::AddBlendInput(const std::string &name, float weight)
{
    _RequireName(name, "blend input name");
    const SdfPath scope = _EnsureScope("BlendInputs");
    UsdPrim prim = _DefineTyped(scope, "RigExecBlendInput", name);
    _ApplyApiRequired(prim, _kNodeGraphApi);
    RigExecBlendInputHandle handle(_stage, prim.GetPath());
    handle.SetWeight(weight);
    return handle;
}

RigExecCurvenetHandle
RigExecRigBuilder::AddCurvenet(
    const std::string &name, const std::vector<GfVec3f> &points)
{
    _RequireName(name, "curvenet name");
    const SdfPath scope = _EnsureScope("Curvenets");
    UsdPrim prim = _DefineTyped(scope, "RigExecCurvenet", name);
    _ApplyApiRequired(prim, _kNodeGraphApi);
    RigExecCurvenetHandle handle(_stage, prim.GetPath());
    handle.SetPoints(points);
    return handle;
}

RigExecMoverChain
RigExecRigBuilder::NewMoverChain(const std::string &name, const SdfPath &defaultTarget)
{
    _RequireName(name, "mover chain name");
    if (!defaultTarget.IsEmpty()) {
        _RequireTargetPath(defaultTarget, "mover chain default target");
    }
    const SdfPath scope = _EnsureScope("Movers");
    const SdfPath chainPath = scope.AppendChild(TfToken(name));
    RigExecSchemaPrim::Define(_stage, chainPath, TfToken("Scope"));
    return RigExecMoverChain(_stage, chainPath, defaultTarget);
}

}  // namespace rigExec
