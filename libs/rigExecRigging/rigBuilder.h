//
// RigExec rigging API (spec section 4 authoring surface).
//
// A programmatic, fluent way to create and wire the prims a RigExec rig is
// made of -- controls, joints, solvers, constraints, weight objects, and
// mover chains -- directly on a UsdStage. The engine itself authors nothing;
// this library is the authoring side of that contract: it writes exactly the
// typed prims, attributes, and relationships the RigExecRigEvaluator
// discovers (type-name based, codeless schema), and nothing else.
//
// Conventions mirrored from the examples/ assets:
//   <rig>/Controls/<name>    RigExecControl prims (flat)
//   <rig>/Joints/<name>      RigExecJoint prims (nestable; rest:space is an
//                            asset-space bind transform when solver-posed)
//   <rig>/Solvers/<name>     aggregate solvers (FK, IK, blend, twist, ribbon)
//   <rig>/Weights/<name>     weight objects
//   <rig>/Curvenets/<name>   RigExecCurvenet data prims
//   <rig>/Movers/<chain>     mover chains; top-level operations are siblings
//                            and may themselves own child movers. The evaluator
//                            executes them in REVERSE composed child order
//                            (bottom-to-top stack walk) -- so ADD ORDER IS
//                            REVERSE APPLICATION ORDER: the last added runs
//                            first. Add outermost passes first.
//
#ifndef RIGEXEC_RIGGING_BUILDER_H
#define RIGEXEC_RIGGING_BUILDER_H

#include "pxr/pxr.h"
#include "pxr/base/gf/matrix4d.h"
#include "pxr/base/gf/quatf.h"
#include "pxr/base/gf/vec3f.h"
#include "pxr/usd/sdf/path.h"
#include "pxr/base/tf/token.h"
#include "pxr/base/vt/value.h"
#include "pxr/usd/sdf/types.h"
#include "pxr/usd/usd/prim.h"
#include "pxr/usd/usd/stage.h"

#include <memory>
#include <string>
#include <utility>
#include <vector>

PXR_NAMESPACE_USING_DIRECTIVE

namespace rigExec {

/// Base of every handle the builder returns. A handle is a cheap value: it
/// keeps the stage alive and names one schema-typed prim. All setters author
/// into the stage's current edit target immediately (no deferred commit).
class RigExecHandleBase {
public:
    RigExecHandleBase() = default;
    RigExecHandleBase(UsdStageRefPtr stage, SdfPath path)
        : _stage(std::move(stage)), _path(std::move(path)) {
        const UsdPrim prim =
            _stage && !_path.IsEmpty() ? _stage->GetPrimAtPath(_path) : UsdPrim();
        if (prim) {
            _schemaType = prim.GetTypeName();
        }
    }

    bool IsValid() const {
        if (!_stage || _path.IsEmpty() || _schemaType.IsEmpty()) {
            return false;
        }
        const UsdPrim prim = _stage->GetPrimAtPath(_path);
        return prim && prim.GetTypeName() == _schemaType;
    }
    UsdPrim GetPrim() const {
        return IsValid() ? _stage->GetPrimAtPath(_path) : UsdPrim();
    }
    UsdStageRefPtr GetStage() const { return _stage; }
    const SdfPath &GetPath() const { return _path; }
    const TfToken &GetSchemaTypeName() const { return _schemaType; }
    std::string GetName() const { return _path.GetName(); }

    /// Set a schema-declared attribute, retaining the explicit Sdf type-name
    /// argument for source compatibility. The requested type must exactly
    /// equal the active composed prim definition; undeclared/custom
    /// attributes are rejected before any property spec is authored.
    void SetAttr(const char *name, const TfToken &typeName, VtValue value);

protected:
    /// Author an ordered relationship target list (order is semantic for
    /// rigExec:sources / controls / joints / inputWeights / samples).
    void SetRel(const char *name, const std::vector<SdfPath> &targets);
    /// Apply a registered single-apply API schema identifier. This is strict:
    /// unavailable/inapplicable schemas and failed application throw.
    void ApplyApi(const TfToken &apiSchemaName);

    UsdStageRefPtr _stage;
    SdfPath _path;
    TfToken _schemaType;
};

/// Common authored contract of every operation carrying RigExecMoverAPI.
class RigExecMoverHandle : public RigExecHandleBase {
public:
    using RigExecHandleBase::RigExecHandleBase;

    /// Shape-preserving enable/disable (inputs:enabled).
    void SetEnabled(bool enabled);
    /// Common normalized envelope. Zero passes the preceding value through;
    /// one applies the mover in full. Used whenever no weight object is bound.
    void SetDefaultWeight(float weight);
    /// Optional target-compatible weight field. A bound object supersedes
    /// inputs:defaultWeight; an empty path clears the binding.
    void SetWeightObject(const SdfPath &path);
    /// Replace the exact authored write set (rigExec:moves).
    void SetMoves(const std::vector<SdfPath> &targets);
    /// Author canonical rigExecReadPhase metadata on a declared input
    /// relationship or attribute of this mover.
    void SetReadPhase(
        const TfToken &propertyName, const std::string &phase);
};

/// A RigExecControl: animator-facing xformable (spec section 4.1).
class RigExecControlHandle : public RigExecHandleBase {
public:
    using RigExecHandleBase::RigExecHandleBase;

    /// Local-to-world bind transform (rest:space, orthonormalized by the
    /// engine).
    void SetRestSpace(const GfMatrix4d &m);
    /// Author avars:tx/ty/tz (local translation, applied after rest).
    void SetAvarTranslation(double tx, double ty, double tz);
    /// Author avars:rx/ry/rz in degrees plus avars:rotationOrder.
    void SetAvarRotation(
        double rx, double ry, double rz,
        const TfToken &order = TfToken("XYZ"));
    /// Author finite avars:sx/sy/sz (local scale; identity is 1,1,1).
    /// Magnitudes below 1e-4 are raised to that floor with sign preserved.
    void SetAvarScale(double sx, double sy, double sz);
    void SetAvarSpin(double degrees);
    /// RigExecControlAPI rigExec:channelRole (pose | switch | tweak).
    void SetChannelRole(const TfToken &role);
};

/// A curvenet knot control, with local avars in a deformation-relative frame.
class RigExecCurvenetAdjustmentHandle : public RigExecControlHandle {
public:
    using RigExecControlHandle::RigExecControlHandle;
    void SetCurvenet(const SdfPath &path);
    void SetKnotIndex(int index);
    void SetIncludeTangents(bool include);
    RigExecCurvenetAdjustmentHandle AddTangent(const std::string &name, int index);
};

/// A RigExecJoint: a solver-posed output xformable. When posed by a solver
/// its rest:space is the asset-space bind transform; an unposed joint follows
/// its namespace parent's posed space with local rest offsets and avars.
class RigExecJointHandle : public RigExecHandleBase {
public:
    using RigExecHandleBase::RigExecHandleBase;

    void SetRestSpace(const GfMatrix4d &m);
    void SetAvarTranslation(double tx, double ty, double tz);
    void SetAvarRotation(
        double rx, double ry, double rz,
        const TfToken &order = TfToken("XYZ"));
    /// Same signed 1e-4 local-scale floor as RigExecControlHandle.
    void SetAvarScale(double sx, double sy, double sz);
    void SetAvarSpin(double degrees);
};

/// One aggregate solver's handle. Solvers publish computePointFrameArray and
/// pose the ordered joints named on rigExec:joints (list position is the
/// element index).
class RigExecSolverHandle : public RigExecHandleBase {
public:
    using RigExecHandleBase::RigExecHandleBase;

    /// Ordered output joints posed by this solver.
    void SetJoints(const std::vector<RigExecJointHandle> &joints);
    void SetJoints(const std::vector<SdfPath> &paths);

protected:
    void _SetSingleRel(const char *name, const SdfPath &target);
};

/// RigExecFkChain: composes ordered controls down the hierarchy.
class RigExecFkChainHandle : public RigExecSolverHandle {
public:
    using RigExecSolverHandle::RigExecSolverHandle;

    /// Ordered driving controls (rigExec:controls).
    void SetControls(const std::vector<RigExecControlHandle> &controls);
    void SetControls(const std::vector<SdfPath> &paths);
    /// world | parentRelative (rigExec:controlSpace): whether each control's
    /// posed frame already carries the motion of the control before it.
    /// Use parentRelative for controls nested one under the next (AddControl
    /// with a parent) so they travel with their parent, the conventional tool FK style,
    /// without the solver applying that motion twice. The joints pose the
    /// same either way. Any other token is rejected.
    void SetControlSpace(const TfToken &space);
    /// Optional base frame (rigExec:startFrame): the joint or control the
    /// chain hangs from. The solver composes every element onto that
    /// provider's rest-to-pose delta, so the chain rides it. Unauthored
    /// (or an empty path, which clears it) leaves the historical absolute
    /// behaviour untouched. Use it when the chain's joints are namespace
    /// children of something ANOTHER solver poses -- the fingers under a
    /// wrist driven by the arm's IK/FK blend -- where nesting the first
    /// control cannot work: a solver-posed joint is an absolute override
    /// and namespace pose does not propagate through it.
    void SetStartFrame(const SdfPath &path);
};

/// RigExecTwoBoneIk.
class RigExecTwoBoneIkHandle : public RigExecSolverHandle {
public:
    using RigExecSolverHandle::RigExecSolverHandle;

    void SetRootControl(const SdfPath &path);
    void SetEffectorControl(const SdfPath &path);
    void SetPoleControl(const SdfPath &path);
    /// Deltas added to the measured bone lengths. Bone lengths themselves
    /// are computed from the bound joints' rest positions and cannot be
    /// authored, so these offsets are the only length controls.
    void SetUpperLengthOffset(double offset);
    void SetLowerLengthOffset(double offset);
    void SetPreferredBendRadians(double radians);
    void SetStretch(float stretch);
    void SetSoftness(float softness);
    /// uniformSegments | ... (schema allowedTokens).
    void SetStretchPolicy(const TfToken &policy);
    void SetUnreachablePolicy(const TfToken &policy);
};

/// RigExecBlendPointFrames: blends two aggregate providers; weight 0 = A, 1 = B.
class RigExecBlendPointFramesHandle : public RigExecSolverHandle {
public:
    using RigExecSolverHandle::RigExecSolverHandle;

    void SetInputA(const SdfPath &path);
    void SetInputB(const SdfPath &path);
    void SetWeight(float weight);
    void SetRotationBlend(const TfToken &mode);  // shortestArc
    void SetScaleBlend(const TfToken &mode);      // log | linear
};

/// RigExecTwistDistribution: distributes twist over N frames between two providers.
class RigExecTwistDistributionHandle : public RigExecSolverHandle {
public:
    using RigExecSolverHandle::RigExecSolverHandle;

    void SetStart(const SdfPath &path);
    void SetEnd(const SdfPath &path);
    void SetCount(int count);
    /// Optional per-frame weights (parallel to the distributed frames).
    void SetWeights(const std::vector<float> &weights);
    /// Additional signed revolutions around the aim axis, including fractions.
    void SetTwistTurns(double turns);
    void SetDistribution(const TfToken &mode);  // minimumEnergy
    /// Optional element index per rigExec:joints entry.
    void SetJointElements(const std::vector<int> &elements);
};

/// RigExecRibbon: samples a driver curve into transported frames.
class RigExecRibbonHandle : public RigExecSolverHandle {
public:
    using RigExecSolverHandle::RigExecSolverHandle;

    void SetDriverCurve(const SdfPath &path);
    void SetStartFrame(const SdfPath &path);
    void SetEndFrame(const SdfPath &path);
    void SetTwistFrames(const std::vector<SdfPath> &paths);
    void SetSampleCount(int count);
    void SetParameterization(const TfToken &mode);  // arcLength | parametric
    void SetDriverCurveReadPhase(const TfToken &phase);
    void SetSurfaceReadPhase(const TfToken &phase);
    void SetJointElements(const std::vector<int> &elements);
};

/// RigExecSplineIk: control-driven spline IK (root/mid/end controls shape a
/// degree-2 B-spline; the ordered rigExec:joints chain is laid along it).
class RigExecSplineIkHandle : public RigExecSolverHandle {
public:
    using RigExecSolverHandle::RigExecSolverHandle;

    void SetRootControl(const SdfPath &path);
    void SetMidControl(const SdfPath &path);
    void SetEndControl(const SdfPath &path);
    /// Per-joint squash/stretch weights, parallel to rigExec:joints (in
    /// chain-slot order). Empty means no thinning.
    void SetVolumeWeights(const std::vector<float> &weights);
    /// curve | chain: what the stretch ratio is measured against.
    void SetRestLength(const TfToken &mode);
    /// Strength of the linear volume preservation, 0..1.
    void SetPreserveVolume(double amount);
    /// Mid control follow point: 0 follows the root, 1 the end.
    void SetMidFollowWeight(double weight);
    /// Additional roll / twist in degrees (the conventional tool ikHandle roll / twist).
    void SetRoll(double degrees);
    void SetTwist(double degrees);
    /// Length floor as a fraction of the rest root->end chord, 0 = off
    /// (inputs:minLengthRatio).
    void SetMinLengthRatio(double ratio);
    /// rigid | aim: how the root tangent CV is posed (rigExec:rootTangent).
    void SetRootTangent(const TfToken &mode);
    /// Optional chain slot per rigExec:joints entry (a permutation).
    void SetJointElements(const std::vector<int> &elements);
};

/// Common contract of the FBX-style pose constraints (spec section 4.1). A
/// constraint that revises a transform is ALSO a mover: it carries
/// RigExecMoverAPI and its exact target on rigExec:moves, so it participates
/// in composed post-order application like any other operation.
class RigExecConstraintHandle : public RigExecMoverHandle {
public:
    using RigExecMoverHandle::RigExecMoverHandle;

    /// The exact prim or property the constraint revises (rigExec:moves).
    void SetTarget(const SdfPath &target);
    /// Durable authoring lock metadata; it does not disable evaluation.
    void SetLocked(bool locked);

protected:
    void _SetSingleRel(const char *name, const SdfPath &target);
};

/// Base for constraints that blend an ordered set of transform sources.
class RigExecSourceConstraintHandle : public RigExecConstraintHandle {
public:
    using RigExecConstraintHandle::RigExecConstraintHandle;

    /// Ordered sources (rigExec:sources). Order is semantic and identifies
    /// entries in the parallel source weights.
    void SetSources(const std::vector<SdfPath> &paths);
    void SetSources(
        const std::vector<SdfPath> &paths, const std::vector<float> &weights);
    /// Replace only the parallel weight array. An empty array restores equal
    /// full weights; otherwise the current source count must match exactly.
    void SetSourceWeights(const std::vector<float> &weights);
};

/// RigExecAimConstraint: rotates the target so its aim vector points at the
/// weighted source position.
class RigExecAimConstraintHandle : public RigExecSourceConstraintHandle {
public:
    using RigExecSourceConstraintHandle::RigExecSourceConstraintHandle;

    void SetAffectRotation(bool x, bool y, bool z);
    void SetRotationOffset(double x, double y, double z);
    void SetRotationOrder(const TfToken &order);
    void SetAimVector(double x, double y, double z);  // local space
    void SetUpVector(double x, double y, double z);   // local space
    void SetWorldUpVector(double x, double y, double z);  // world space
    /// Legacy single-source spelling (rigExec:aimTarget).
    void SetAimTarget(const SdfPath &path);
    void SetWorldUpObject(const SdfPath &path);
    void SetWorldUpType(const TfToken &type);  // none | ...
};

/// RigExecPositionConstraint.
class RigExecPositionConstraintHandle : public RigExecSourceConstraintHandle {
public:
    using RigExecSourceConstraintHandle::RigExecSourceConstraintHandle;

    void SetAffectTranslation(bool x, bool y, bool z);
    void SetTranslationOffset(double x, double y, double z);
};

/// RigExecRotationConstraint.
class RigExecRotationConstraintHandle : public RigExecSourceConstraintHandle {
public:
    using RigExecSourceConstraintHandle::RigExecSourceConstraintHandle;

    void SetAffectRotation(bool x, bool y, bool z);
    void SetRotationOffset(double x, double y, double z);
    void SetRotationOrder(const TfToken &order);
};

/// RigExecScaleConstraint.
class RigExecScaleConstraintHandle : public RigExecSourceConstraintHandle {
public:
    using RigExecSourceConstraintHandle::RigExecSourceConstraintHandle;

    void SetAffectScale(bool x, bool y, bool z);
    void SetScaleOffset(double x, double y, double z);
};

/// RigExecParentConstraint (FBX parent: scale off by default).
class RigExecParentConstraintHandle : public RigExecSourceConstraintHandle {
public:
    using RigExecSourceConstraintHandle::RigExecSourceConstraintHandle;

    void SetAffectTranslation(bool x, bool y, bool z);
    void SetAffectRotation(bool x, bool y, bool z);
    void SetAffectScale(bool x, bool y, bool z);
    void SetRotationOrder(const TfToken &order);
    /// Per-source additive offsets, parallel to rigExec:sources. The parent
    /// constraint is the one operator that reads these double3[] arrays
    /// (inputs:translationOffsets / rotationOffsets [degrees]).
    void SetTranslationOffsets(const std::vector<GfVec3d> &offsets);
    void SetRotationOffsets(const std::vector<GfVec3d> &degrees);
};

/// RigExecSingleChainIkConstraint: solves an inferred namespace chain.
class RigExecSingleChainIkConstraintHandle : public RigExecConstraintHandle {
public:
    using RigExecConstraintHandle::RigExecConstraintHandle;

    void SetFirstJoint(const SdfPath &path);
    void SetEndJoint(const SdfPath &path);
    void SetEffector(const SdfPath &path);
    /// Complete firstJoint-to-endJoint joint chain on rigExec:moves. The
    /// evaluator requires this relationship to equal the inferred namespace
    /// chain, so every joint in order is required for a valid constraint;
    /// the inherited single-target SetTarget cannot express it.
    void SetMoves(const std::vector<SdfPath> &paths);
    void SetPoleVectorObjects(const std::vector<SdfPath> &paths);
    void SetPoleVectorWeights(const std::vector<float> &weights);
    void SetPoleVector(double x, double y, double z);
    void SetTwistDegrees(double degrees);
    void SetSolverMode(const TfToken &mode);       // rotatePlane | ...
    void SetPoleVectorMode(const TfToken &mode);   // vector | ...
    void SetEvaluationMode(const TfToken &mode);   // neverTS | autoDetect | alwaysTS
};

/// Common contract of weight objects: a total scalar field over the logical
/// elements of one exact target (spec section 4.1).
class RigExecWeightHandle : public RigExecHandleBase {
public:
    using RigExecHandleBase::RigExecHandleBase;

    /// Canonical prim or exact property carrying the weighted domain.
    void SetTarget(const SdfPath &target);
    /// constant | dense | sparse (authored objects).
    void SetRepresentation(const TfToken &rep);
    /// strict | clamp.
    void SetRangePolicy(const TfToken &policy);
};

/// RigExecStaticWeight: time-invariant authored field.
class RigExecStaticWeightHandle : public RigExecWeightHandle {
public:
    using RigExecWeightHandle::RigExecWeightHandle;

    /// Dense values (one per element) or sparse values paired with indices.
    void SetValues(const std::vector<float> &values);
    /// Sparse element indices, parallel to a sparse value list.
    void SetIndices(const std::vector<int> &indices);
    /// Atomically validate and replace a sparse value/index pair.
    void SetSparseValues(
        const std::vector<float> &values,
        const std::vector<int> &indices);
    /// Value for elements no entry covers.
    void SetDefaultWeight(float weight);
};

/// RigExecDynamicWeight: recomputed from static inputs, optionally modulating
/// a base weight object: r_i = (b_i * driver) * scale + bias.
class RigExecDynamicWeightHandle : public RigExecWeightHandle {
public:
    using RigExecWeightHandle::RigExecWeightHandle;

    void SetBaseWeight(const SdfPath &path);
    void SetDriver(float driver);
    void SetScale(float scale);
    void SetBias(float bias);
};

/// Common contract of placed volumetric weight objects (sphere, plane,
/// curve): a distance function generates the field. The volume is an
/// xformable -- place it with rest:space/avars like any control or joint;
/// shape comes from the falloff band and scale inputs, not the transform.
class RigExecVolumeWeightHandle : public RigExecWeightHandle {
public:
    using RigExecWeightHandle::RigExecWeightHandle;

    /// Placement (asset-space bind transform of the volume's local frame).
    void SetRestSpace(const GfMatrix4d &m);
    void SetAvarTranslation(double tx, double ty, double tz);
    void SetAvarRotation(
        double rx, double ry, double rz,
        const TfToken &order = TfToken("XYZ"));
    void SetAvarSpin(double degrees);

    /// Distance at which the field is fully ON / OFF (local units).
    void SetFalloff(float falloffMin, float falloffMax);
    /// 0..1 lerp between the two ramps.
    void SetInvert(float invert);
    /// Final multiplier on the remapped weight.
    void SetStrength(float strength);
    /// linear | smooth | easeIn | easeOut | constant | curve.
    void SetFalloffProfile(const TfToken &profile);
    /// Ts spline over x in [0,1], read when the profile is `curve`. The
    /// schema declares rigExec:falloffCurve as a float; the engine reads it
    /// with UsdAttribute::GetSpline(), so the curve IS its time samples --
    /// each (x, y) knot lands at time x. Knots are clamped to [0,1].
    void SetFalloffCurve(const std::vector<std::pair<double, double>> &knots);
    /// reference (static base points) | current (in-flight stack points).
    void SetSamplePhase(const TfToken &phase);
    /// Optional explicit static sampling source.
    void SetSampleSource(const SdfPath &path);
};

/// RigExecSphereWeight: radial falloff about the placed origin; per-axis
/// scales make the iso-surfaces ellipsoidal.
class RigExecSphereWeightHandle : public RigExecVolumeWeightHandle {
public:
    using RigExecVolumeWeightHandle::RigExecVolumeWeightHandle;

    void SetScales(float sx, float sy, float sz);
};

/// RigExecPlaneWeight: signed gradient across a plane.
class RigExecPlaneWeightHandle : public RigExecVolumeWeightHandle {
public:
    using RigExecVolumeWeightHandle::RigExecVolumeWeightHandle;

    void SetAxis(const TfToken &axis);  // x | y | z
    /// unbounded (half-space) | bounded (clipped to extentU x extentV).
    void SetBounds(const TfToken &bounds);
    void SetExtents(float extentU, float extentV);
};

/// RigExecCurveWeight: tubular falloff about a curve's control polygon.
class RigExecCurveWeightHandle : public RigExecVolumeWeightHandle {
public:
    using RigExecVolumeWeightHandle::RigExecVolumeWeightHandle;

    /// Exactly one native points source (BasisCurves prim or point3f[] property).
    void SetCurve(const SdfPath &path);
    void SetScales(float sx, float sy, float sz);
};

/// RigExecCombineWeight: folds an ordered list of weight objects into one.
class RigExecCombineWeightHandle : public RigExecWeightHandle {
public:
    using RigExecWeightHandle::RigExecWeightHandle;

    /// Ordered inputs (rigExec:inputWeights). All must resolve to the same
    /// element count as this combine's own target.
    void SetInputWeights(const std::vector<SdfPath> &paths);
    /// multiply | add | subtract | max | min | average | overlay.
    void SetCombineMode(const TfToken &mode);
    void SetStrength(float strength);
    void SetInvert(float invert);
};

/// A RigExecBlendInput: one independently composable blend channel.
class RigExecBlendSampleHandle;  // fwd (AddSample return)

class RigExecBlendInputHandle : public RigExecHandleBase {
public:
    using RigExecHandleBase::RigExecHandleBase;

    /// Channel weight (inputs:weight).
    void SetWeight(float weight);
    /// Drive inputs:weight from a RigExecPose's outputs:weight instead of an
    /// authored number. The connection is authored on the CONSUMER, which is
    /// what lets a layer retarget one corrective's channel without touching
    /// the interpolator that drives it. An empty path removes it.
    void ConnectWeight(const SdfPath &output);
    /// Add a sample to this input's ordered rigExec:samples.
    RigExecBlendSampleHandle AddSample(const std::string &name, float activation = 1.f);
};

/// A RigExecBlendSample: one target or in-between points property.
class RigExecBlendSampleHandle : public RigExecHandleBase {
public:
    using RigExecHandleBase::RigExecHandleBase;

    /// Activation of this sample within its input (rigExec:activation).
    void SetActivation(float activation);
    /// Exact native points property of the target shape.
    void SetTargetPoints(const SdfPath &path);
    /// A UsdSkelBlendShape carrying the same shape as sparse offsets, as the
    /// alternative to SetTargetPoints. Mutually exclusive with it: a dense
    /// sample's full points array is read per sample per frame regardless of
    /// its channel weight, which is ~65 ms/frame for 169 correctives on a
    /// 26,276-point body standing at rest.
    void SetBlendShape(const SdfPath &path);
    /// base | preceding | final.
    void SetReadPhase(const TfToken &phase);
};

/// A RigExecPose: one authored pose of an interpolator, and the weight it
/// publishes.
class RigExecPoseHandle : public RigExecHandleBase {
public:
    using RigExecHandleBase::RigExecHandleBase;

    /// swing | twist | whole -- which PART of the driver's rotation this
    /// pose is measured against. poseType, and a property of the POSE
    /// rather than of the interpolator: one driver carries both at once and a
    /// neck that has twisted has not bent.
    void SetPoseType(const TfToken &poseType);
    /// Where the driver stands, as its local rotation relative to its own
    /// rest -- REBASED, with the neutral pose taken out
    ///.
    void SetRotation(const GfQuatf &rotation);
    /// The driver's translation in this pose, in CENTIMETRES, in the driver's
    /// own frame. Read only when the interpolator enables translation.
    void SetTranslation(const GfVec3f &translation);
    /// The pose's own falloff widths: radians and centimetres. Zero means
    /// "measure one from the poses" -- see RigExecRbfFitWidth for why these
    /// are fitted rather than read out of the conventional tool.
    void SetRadii(float rotationRadius, float translationRadius = 0.f);
    /// poseFalloff, as provenance: the share painted on top of the
    /// fitted width, 0.3 being the conventional default. The radii already carry it.
    void SetFalloff(float falloff);
    /// The exact control PROPERTIES that put the rig into this pose and their
    /// values, in OUR units and from OUR zero (degrees, centimetres). The two
    /// vectors must be the same length; authoring data only.
    void SetPoseControls(const std::vector<SdfPath> &properties,
                         const std::vector<double> &values);
    void SetEnabled(bool enabled);
    /// The `outputs:weight` property path, which is what a blend input's
    /// inputs:weight connects to. Valid whether or not anything reads it yet.
    SdfPath GetWeightOutput() const;
};

/// A RigExecPoseInterpolator: the conventional poseInterpolator, one driver in and one
/// weight per pose out.
class RigExecPoseInterpolatorHandle : public RigExecHandleBase {
public:
    using RigExecHandleBase::RigExecHandleBase;

    /// The joint whose LOCAL rotation every pose is measured against.
    void SetDriver(const SdfPath &path);
    /// gaussian | linear. `interpolation`: 0 linear, 1 gaussian.
    void SetKernel(const TfToken &kernel);
    /// enableRotation / enableTranslation. Honoured rather than
    /// assumed: read as a rotation, a translation interpolator's poses are
    /// all identity and it solves degenerate.
    void SetChannels(bool enableRotation, bool enableTranslation);
    void SetAllowNegativeWeights(bool allow);
    void SetNormalize(bool normalize);
    void SetRegularization(float regularization);
    /// X | Y | Z, in the driver's own frame. driverTwistAxis.
    void SetTwistAxis(const TfToken &axis);
    void SetEnabled(bool enabled);
    /// Add a pose as a child of this interpolator. Poses are prims rather
    /// than array entries so each composes independently.
    RigExecPoseHandle AddPose(const std::string &name);
};

/// A RigExecCurvenet: a net of cubic splines profiling a surface (2022 paper).
class RigExecCurvenetHandle : public RigExecHandleBase {
public:
    using RigExecHandleBase::RigExecHandleBase;

    /// The shared control-point pool (knots AND tangent handles), in the
    /// projection pose. UsdGeomPoints `points` attribute.
    void SetPoints(const std::vector<GfVec3f> &points);
    /// Append one cubic spline: four pool indices p0, h0, h1, p1 (bezier).
    void AddSpline(int p0, int h0, int h1, int p1);
    /// bezier | catmullRom.
    void SetBasis(const TfToken &basis);
    void SetSamplesPerSpline(int count);
};

// ---------------------------------------------------------------------------
// Mover handles (point-domain and property-domain operations)
// ---------------------------------------------------------------------------

/// RigExecMatrixMover: p' = q + w (T q - q). Moves points through one
/// provider's matrix, blended by a per-point weight field.
class RigExecMatrixMoverHandle : public RigExecMoverHandle {
public:
    using RigExecMoverHandle::RigExecMoverHandle;
    using RigExecMoverHandle::SetReadPhase;

    /// The GfMatrix4d provider (computeMatrix) -- joint, control, or xform.
    void SetTransformProvider(const SdfPath &path);
    /// base | preceding | final.
    void SetReadPhase(const TfToken &phase);
};

/// RigExecSkinMover: multi-influence skinning in one pass, UsdSkel's
/// jointIndices / jointWeights layout over an ordered influence list.
/// classicLinear: p' = (1 - sum w) p + sum_i w_i (T_i p).
class RigExecSkinMoverHandle : public RigExecMoverHandle {
public:
    using RigExecMoverHandle::RigExecMoverHandle;
    using RigExecMoverHandle::SetReadPhase;

    /// Ordered GfMatrix4d providers (joints or controls). The order is
    /// semantic: jointIndices index this list.
    void SetInfluences(const std::vector<SdfPath> &providers);
    /// Per-point layout: elementSize slots per point in point order, each an
    /// index into the influence list with a parallel weight. Lengths must
    /// agree and be a multiple of elementSize; indices non-negative; weights
    /// finite and non-negative. The point count is checked at compile.
    void SetJointInfluences(
        const std::vector<int> &indices, const std::vector<float> &weights,
        int elementSize);
    /// classicLinear | dualQuaternion (scale-aware dual-quaternion
    /// skinning: rotation blended on the shortest arc, joint scale and
    /// shear blended linearly in the pre-rotation frame).
    void SetSkinningMethod(const TfToken &method);
    /// base | preceding | final, for every influence.
    void SetReadPhase(const TfToken &phase);
};

/// RigExecLatticeMover: tensor-product lattice deformation through a cage.
class RigExecLatticeMoverHandle : public RigExecMoverHandle {
public:
    using RigExecMoverHandle::RigExecMoverHandle;
    using RigExecMoverHandle::SetReadPhase;

    /// Native mesh/points prim supplying the cage control points.
    void SetCage(const SdfPath &path);
    /// bspline | bernstein.
    void SetBasis(const TfToken &basis);
    void SetDivisions(int x, int y, int z);
    /// base | preceding | final.
    void SetReadPhase(const TfToken &phase);
};

/// RigExecBlendShapeMover: applies independently composed blend inputs to a
/// points property.
class RigExecBlendShapeMoverHandle : public RigExecMoverHandle {
public:
    using RigExecMoverHandle::RigExecMoverHandle;

    /// Add a blend channel (rigExec:blendInputs entry) under this mover.
    RigExecBlendInputHandle AddBlendInput(const std::string &name, float weight = 0.f);
    /// Replace the complete relationship to independently authored inputs.
    void SetBlendInputs(const std::vector<SdfPath> &paths);
};

/// RigExecCurveMover: curve-driven movement (wire / spline-IK / ribbon).
class RigExecCurveMoverHandle : public RigExecMoverHandle {
public:
    using RigExecMoverHandle::RigExecMoverHandle;
    using RigExecMoverHandle::SetReadPhase;

    void SetDriverCurve(const SdfPath &path);
    void SetDriverFrames(const std::vector<SdfPath> &paths);
    void SetBindCoordinates(const SdfPath &path);
    /// ribbon | emitGuidePoints.
    void SetMode(const TfToken &mode);
    /// base | preceding | final.
    void SetReadPhase(const TfToken &phase);
};

/// RigExecSurfaceMover: surface attachment / projection.
class RigExecSurfaceMoverHandle : public RigExecMoverHandle {
public:
    using RigExecMoverHandle::RigExecMoverHandle;
    using RigExecMoverHandle::SetReadPhase;

    /// Native mesh prim supplying the driver surface.
    void SetSurface(const SdfPath &path);
    /// attach | project.
    void SetMode(const TfToken &mode);
    /// base | preceding | final.
    void SetReadPhase(const TfToken &phase);
};

/// RigExecSmoothMover: uniform-weight Laplacian smoothing.
class RigExecSmoothMoverHandle : public RigExecMoverHandle {
public:
    using RigExecMoverHandle::RigExecMoverHandle;

    /// Source-compatible alias for SetDefaultWeight.
    void SetStrength(float strength);
};

/// RigExecVolumeCorrectMover: centroid scaling toward the rest bound volume.
class RigExecVolumeCorrectMoverHandle : public RigExecMoverHandle {
public:
    using RigExecMoverHandle::RigExecMoverHandle;

    /// Source-compatible alias for SetDefaultWeight.
    void SetStrength(float strength);
};

/// RigExecCurvenetMover (Profile Mover): propagates a rigged curvenet's
/// articulation over the target points.
class RigExecCurvenetMoverHandle : public RigExecMoverHandle {
public:
    using RigExecMoverHandle::RigExecMoverHandle;

    /// Exactly one RigExecCurvenet supplying the posed control points.
    void SetCurvenet(const SdfPath &path);
    /// Source-compatible alias for SetDefaultWeight.
    void SetStrength(float strength);
};

class RigExecCurvenetAdjusterMoverHandle : public RigExecMoverHandle {
public:
    using RigExecMoverHandle::RigExecMoverHandle;
    void SetAdjustments(const std::vector<SdfPath> &paths);
};

/// RigExecFloatMathMover: add | multiply | clamp | remap | blend over an
/// exact float property. The common MoverAPI envelope mixes the candidate
/// result back over the incoming value.
class RigExecFloatMathMoverHandle : public RigExecMoverHandle {
public:
    using RigExecMoverHandle::RigExecMoverHandle;

    void SetOperation(const TfToken &op);  // add | multiply | clamp | remap | blend
    void SetValue(float value);
    /// Bounds for clamp / remap.
    void SetBounds(float min, float max);
    /// Source-compatible alias for SetDefaultWeight.
    void SetWeight(float weight);
};

/// RigExecVec3fMathMover: component-wise operations over a float3 property.
class RigExecVec3fMathMoverHandle : public RigExecMoverHandle {
public:
    using RigExecMoverHandle::RigExecMoverHandle;

    void SetOperation(const TfToken &op);
    void SetValue(GfVec3f value);
    void SetBounds(GfVec3f min, GfVec3f max);
    /// Source-compatible alias for SetDefaultWeight.
    void SetWeight(float weight);
};

/// RigExecMatrixMathMover: multiply | blend over an exact matrix4d property.
class RigExecMatrixMathMoverHandle : public RigExecMoverHandle {
public:
    using RigExecMoverHandle::RigExecMoverHandle;

    /// multiply (post-multiply, row-vector convention) | blend (replace).
    void SetOperation(const TfToken &op);
    void SetValue(GfMatrix4d value);
    /// Source-compatible alias for SetDefaultWeight.
    void SetWeight(float weight);
};

// ---------------------------------------------------------------------------
// Mover chains
// ---------------------------------------------------------------------------

/// One mover chain: a scope at <rig>/Movers/<name> whose children are the
/// chain's operations as SIBLINGS. The evaluator executes Movers in
/// reverse-sibling post-order (spec section 4.2): descendants run before
/// their mover parent, and sibling branches run in REVERSE composed child
/// order -- so ADD ORDER IS REVERSE APPLICATION ORDER: the last operation
/// added runs first, each earlier one wraps its result. Sibling chains
/// apply in reverse creation order as well. Add outermost passes first.
///
/// (Hand-authored assets may instead nest movers -- deepest applies first --
/// which is the same contract read from the other end of the tree.)
class RigExecMoverChain {
public:
    /// Every Add* method takes the exact target property (or prim) the
    /// operation writes; pass an empty path to reuse the chain's default
    /// target set at construction.

    // ---- Point-domain movers -------------------------------------------

    /// Moves points through a provider's matrix. weightObject is optional;
    /// without one, the MoverAPI defaultWeight envelope is used.
    RigExecMatrixMoverHandle AddMatrixMover(
        const std::string &name,
        const SdfPath &transformProvider,
        const SdfPath &weightObject = {},
        const SdfPath &target = {},
        const TfToken &readPhase = TfToken("base"));

    /// Multi-influence skinning over an ordered influence list. The
    /// per-point layout is authored on the handle (SetJointInfluences).
    /// weightObject is optional; without one, the MoverAPI defaultWeight
    /// envelope is used.
    RigExecSkinMoverHandle AddSkinMover(
        const std::string &name,
        const std::vector<SdfPath> &influences,
        const SdfPath &weightObject = {},
        const SdfPath &target = {},
        const TfToken &readPhase = TfToken("base"));

    /// Tensor-product lattice deformation through a native cage prim.
    RigExecLatticeMoverHandle AddLatticeMover(
        const std::string &name,
        const SdfPath &cagePrim,
        int divX, int divY, int divZ,
        const TfToken &basis = TfToken("bspline"),
        const SdfPath &target = {},
        const TfToken &readPhase = TfToken("base"));

    /// Blend-shape application over independently composed inputs.
    RigExecBlendShapeMoverHandle AddBlendShapeMover(
        const std::string &name,
        const SdfPath &weightObject = {},
        const SdfPath &target = {});

    /// Curve-driven movement (wire / spline-IK / ribbon modes).
    RigExecCurveMoverHandle AddCurveMover(
        const std::string &name,
        const SdfPath &driverCurve,
        const std::vector<SdfPath> &driverFrames,
        const SdfPath &bindCoordinates /* may be empty */,
        const TfToken &mode = TfToken("ribbon"),
        const SdfPath &target = {},
        const TfToken &readPhase = TfToken("base"));
    /// Source-compatible spelling for zero or one driver-frame provider.
    RigExecCurveMoverHandle AddCurveMover(
        const std::string &name,
        const SdfPath &driverCurve,
        const SdfPath &driverFrame /* may be empty */,
        const SdfPath &bindCoordinates /* may be empty */,
        const TfToken &mode = TfToken("ribbon"),
        const SdfPath &target = {},
        const TfToken &readPhase = TfToken("base"));

    /// Surface attachment / projection.
    RigExecSurfaceMoverHandle AddSurfaceMover(
        const std::string &name,
        const SdfPath &surfacePrim,
        const TfToken &mode = TfToken("attach"),
        const SdfPath &target = {},
        const TfToken &readPhase = TfToken("base"));

    /// Uniform-weight Laplacian smoothing with the common mover envelope.
    RigExecSmoothMoverHandle AddSmoothMover(
        const std::string &name, float defaultWeight = 1.0f,
        const SdfPath &target = {});

    /// Centroid scaling toward the authored rest bound volume.
    RigExecVolumeCorrectMoverHandle AddVolumeCorrectMover(
        const std::string &name, float defaultWeight = 1.0f,
        const SdfPath &target = {});

    /// Profile-mover: propagates a rigged curvenet's articulation.
    RigExecCurvenetMoverHandle AddCurvenetMover(
        const std::string &name,
        const SdfPath &curvenetPrim,
        float defaultWeight = 1.0f,
        const SdfPath &target = {});
    RigExecCurvenetAdjusterMoverHandle AddCurvenetAdjusterMover(
        const std::string &name, const std::vector<SdfPath> &adjustments,
        float defaultWeight = 1.0f, const SdfPath &target = {});

    // ---- Property-domain movers ----------------------------------------

    /// add | multiply | clamp | remap | blend over an exact float property.
    RigExecFloatMathMoverHandle AddFloatMathMover(
        const std::string &name,
        const TfToken &operation,
        float value,
        const SdfPath &target = {},
        float defaultWeight = 1.f);

    /// Component-wise add | multiply | clamp | remap | blend over a float3 property.
    RigExecVec3fMathMoverHandle AddVec3fMathMover(
        const std::string &name,
        const TfToken &operation,
        GfVec3f value,
        const SdfPath &target = {},
        float defaultWeight = 1.f);

    /// multiply | blend over an exact matrix4d property.
    RigExecMatrixMathMoverHandle AddMatrixMathMover(
        const std::string &name,
        const TfToken &operation,
        GfMatrix4d value,
        const SdfPath &target = {},
        float defaultWeight = 1.f);

    // ---- Pose constraints (movers too) ---------------------------------

    RigExecAimConstraintHandle AddAimConstraint(
        const std::string &name, const SdfPath &target = {});
    RigExecPositionConstraintHandle AddPositionConstraint(
        const std::string &name, const SdfPath &target = {});
    RigExecRotationConstraintHandle AddRotationConstraint(
        const std::string &name, const SdfPath &target = {});
    RigExecScaleConstraintHandle AddScaleConstraint(
        const std::string &name, const SdfPath &target = {});
    RigExecParentConstraintHandle AddParentConstraint(
        const std::string &name, const SdfPath &target = {});
    /// Infer and author the complete inclusive namespace chain. Every prim
    /// from endJoint through firstJoint must already be a RigExecJoint.
    RigExecSingleChainIkConstraintHandle AddSingleChainIkConstraint(
        const std::string &name,
        const SdfPath &firstJoint,
        const SdfPath &endJoint,
        const SdfPath &effector,
        const std::vector<SdfPath> &poleVectorObjects = {});
    /// Explicit form: the ordered moves list must exactly equal the inferred
    /// first-to-end namespace chain.
    RigExecSingleChainIkConstraintHandle AddSingleChainIkConstraint(
        const std::string &name,
        const std::vector<SdfPath> &moves,
        const SdfPath &firstJoint,
        const SdfPath &endJoint,
        const SdfPath &effector,
        const std::vector<SdfPath> &poleVectorObjects = {});

    /// Return a child chain whose next movers are authored directly beneath
    /// an existing mover. Descendants execute before their mover parent. An
    /// empty override inherits this chain's default target.
    RigExecMoverChain Under(
        const RigExecHandleBase &mover,
        const SdfPath &defaultTarget = {}) const;

    /// The chain anchor: a Scope for a top-level chain or a mover for Under().
    const SdfPath &GetScopePath() const { return _scope; }
    UsdStageRefPtr GetStage() const { return _stage; }

private:
    friend class RigExecRigBuilder;
    RigExecMoverChain(
        UsdStageRefPtr stage, SdfPath scope, SdfPath defaultTarget)
        : _stage(std::move(stage)),
          _scope(std::move(scope)),
          _defaultTarget(std::move(defaultTarget)) {}

    /// Define a typed mover prim as the next child under the chain anchor,
    /// apply RigExecMoverAPI, and set its target when given (an empty target
    /// falls back to the chain's default target; no effective target throws).
    /// Siblings execute in reverse
    /// composed child order -- so appending makes the new mover run FIRST.
    SdfPath _AddMoverPrim(
        const std::string &typeName,
        const std::string &name,
        const SdfPath &target);

    UsdStageRefPtr _stage;
    SdfPath _scope;
    /// Target reused by operations added without an explicit one.
    SdfPath _defaultTarget;
};

// ---------------------------------------------------------------------------
// Top-level builder
// ---------------------------------------------------------------------------

/// The top-level builder: one per (stage, rig root).
class RigExecRigBuilder {
public:
    /// Open or create the rig. If \p rigRoot does not exist it is defined as
    /// a RigExecRoot prim; if it exists with another type this throws
    /// std::invalid_argument. An existing RigExecRoot is reused (idempotent).
    static RigExecRigBuilder Create(
        UsdStageRefPtr stage,
        const SdfPath &rigRoot = SdfPath("/Rig"),
        const TfToken &partition = TfToken());

    UsdStageRefPtr GetStage() const { return _stage; }
    const SdfPath &GetRootPath() const { return _root; }

    // ---- Transform providers -------------------------------------------

    /// Create <rig>/Controls/<name> (or nested under \p parentControl) as a
    /// RigExecControl. restSpace is asset space for a top-level control and
    /// relative to the parent control's rest when nested: a nested
    /// control's rest and posed frames compose through its namespace
    /// ancestor, so it travels with the parent (an FK chain in
    /// parentRelative controlSpace is built this way).
    RigExecControlHandle AddControl(
        const std::string &name, const GfMatrix4d &restSpace = GfMatrix4d(),
        const RigExecControlHandle *parentControl = nullptr);

    /// Create <rig>/Joints/<name> (or nested under \p parentJoint) as a
    /// RigExecJoint. restSpace is the asset-space bind transform for
    /// solver-posed joints.
    RigExecJointHandle AddJoint(
        const std::string &name,
        const GfMatrix4d &restSpace = GfMatrix4d(),
        const RigExecJointHandle *parentJoint = nullptr);

    // ---- Solvers (created under <rig>/Solvers) --------------------------

    RigExecFkChainHandle AddFkChain(const std::string &name);
    RigExecTwoBoneIkHandle AddTwoBoneIk(
        const std::string &name,
        const SdfPath &rootControl,
        const SdfPath &effectorControl,
        const SdfPath &poleControl /* may be empty */);
    RigExecBlendPointFramesHandle AddBlendPointFrames(
        const std::string &name,
        const SdfPath &inputA,
        const SdfPath &inputB,
        float weight = 0.f);
    RigExecTwistDistributionHandle AddTwistDistribution(
        const std::string &name,
        const SdfPath &start,
        const SdfPath &end,
        int count = 1);
    RigExecRibbonHandle AddRibbon(
        const std::string &name,
        const SdfPath &driverCurve,
        int sampleCount = 5);
    RigExecSplineIkHandle AddSplineIk(
        const std::string &name,
        const SdfPath &rootControl,
        const SdfPath &midControl,
        const SdfPath &endControl);

    // ---- Weight objects (created under <rig>/Weights) -------------------

    /// Dense or sparse authored field over \p target.
    RigExecStaticWeightHandle AddStaticWeight(
        const std::string &name,
        const SdfPath &target,
        const std::vector<float> &values = {},
        const std::vector<int> &indices = {},
        float defaultWeight = 0.f);

    /// Driven modulation of an optional base weight object.
    RigExecDynamicWeightHandle AddDynamicWeight(
        const std::string &name,
        const SdfPath &target,
        const SdfPath &baseWeight /* may be empty */);

    /// Placed sphere / plane / curve volumes generating a field over \p target.
    RigExecSphereWeightHandle AddSphereWeight(
        const std::string &name, const SdfPath &target, float falloffMin = 0.f,
        float falloffMax = 1.f);
    RigExecPlaneWeightHandle AddPlaneWeight(
        const std::string &name, const SdfPath &target, float falloffMin = 0.f,
        float falloffMax = 1.f);
    RigExecCurveWeightHandle AddCurveWeight(
        const std::string &name, const SdfPath &target, const SdfPath &curve,
        float falloffMin = 0.f, float falloffMax = 1.f);

    /// Define a placed volume at an explicit prim path beneath this rig. The
    /// parent must already exist; unlike Add* these do not impose /Weights.
    RigExecSphereWeightHandle DefineSphereWeight(
        const SdfPath &path, const SdfPath &target, float falloffMin = 0.f,
        float falloffMax = 1.f);
    RigExecPlaneWeightHandle DefinePlaneWeight(
        const SdfPath &path, const SdfPath &target, float falloffMin = 0.f,
        float falloffMax = 1.f);
    RigExecCurveWeightHandle DefineCurveWeight(
        const SdfPath &path, const SdfPath &target, const SdfPath &curve,
        float falloffMin = 0.f, float falloffMax = 1.f);

    /// Ordered fold of weight objects (multiply | add | max | min | ...).
    RigExecCombineWeightHandle AddCombineWeight(
        const std::string &name,
        const SdfPath &target,
        const std::vector<SdfPath> &inputWeights,
        const TfToken &mode = TfToken("multiply"));

    // ---- Independently composable blend channels ------------------------

    /// Create <rig>/BlendInputs/<name>. Link it to any blend-shape mover with
    /// RigExecBlendShapeMoverHandle::SetBlendInputs.
    RigExecBlendInputHandle AddBlendInput(
        const std::string &name, float weight = 0.f);

    // ---- Pose interpolators (created under <rig>/PoseInterpolators) ------

    /// Create <rig>/PoseInterpolators/<name> reading \p driver.
    ///
    /// Its own scope and not /Movers: an interpolator writes no transform and
    /// no points, so it has no place in an order that exists to say which
    /// write lands on top of which. It publishes a float per pose and the
    /// blend inputs connect to those.
    RigExecPoseInterpolatorHandle AddPoseInterpolator(
        const std::string &name, const SdfPath &driver);

    // ---- Curvenets (created under <rig>/Curvenets) ----------------------

    RigExecCurvenetHandle AddCurvenet(
        const std::string &name, const std::vector<GfVec3f> &points);
    RigExecCurvenetAdjustmentHandle AddCurvenetAdjustment(
        const std::string &name, const SdfPath &curvenet, int knotIndex);

    // ---- Mover chains ----------------------------------------------------

    /// Start a new chain at <rig>/Movers/<chainName>. Operations added to the
    /// returned chain apply in REVERSE add order (last added runs first);
    /// chains themselves also apply in reverse creation order. Add outermost
    /// passes first so the pipeline applies deepest-first.
    RigExecMoverChain NewMoverChain(
        const std::string &name, const SdfPath &defaultTarget = {});

private:
    RigExecRigBuilder(UsdStageRefPtr stage, SdfPath root)
        : _stage(std::move(stage)), _root(std::move(root)) {}

    /// Ensure a (Scope-typed) child namespace exists under the rig and return
    /// its path.
    SdfPath _EnsureScope(const char *scopeName);
    /// Define a new typed prim at \p parent/<name>, rejecting every existing
    /// prim and applying the standard authoring APIs transactionally.
    UsdPrim _DefineTyped(
        const SdfPath &parent, const std::string &typeName,
        const std::string &name);
    /// Strict typed definition at an explicit path beneath this rig.
    UsdPrim _DefineTypedAt(
        const SdfPath &path, const std::string &typeName);

    UsdStageRefPtr _stage;
    SdfPath _root;
};

}  // namespace rigExec

#endif  // RIGEXEC_RIGGING_BUILDER_H
