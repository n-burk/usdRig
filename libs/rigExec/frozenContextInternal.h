// Private frozenContext implementation contracts.

#ifndef RIGEXEC_FROZEN_CONTEXT_INTERNAL_H
#define RIGEXEC_FROZEN_CONTEXT_INTERNAL_H

#include "frozenContext.h"
#include "bakedProgramImpl.h"
#include "tapSet.h"

#include "pxr/base/gf/matrix4d.h"
#include "pxr/base/gf/vec3d.h"
#include "pxr/base/gf/vec3f.h"
#include "pxr/base/tf/staticTokens.h"
#include "pxr/base/tf/token.h"
#include "pxr/usd/sdf/types.h"
#include "pxr/usd/usd/attribute.h"
#include "pxr/usd/usd/attributeQuery.h"
#include <map>

namespace rigExec {

namespace frozenDetail {

// A context that asks for more slots than this is corrupt, not large: the
// largest measured arena (biped, full frame) is ~2.7 MiB, so a gigabyte of
// doubles is three orders of magnitude past anything a real job sizes.
inline constexpr size_t kMaxFrozenArenaSlots = size_t(1) << 27;

// Shared scalar schema for sampling and worker reconstruction. The stage
// sampler and worker must agree on every iterative deformer input.
template <class Fn>
void
_VisitIterativeMoverScalars(RigExecRevisionOp op,
                           RigExecMoverParameters &params, Fn &&fn)
{
    if (op == RigExecRevisionOp::DeltaMush) {
        fn("inputs:iterations", 10, params.mushIterations);
        fn("inputs:step", 0.5f, params.mushStep);
        fn("inputs:pinBorders", true, params.mushPinBorders);
        fn("inputs:distanceWeight", 0.0f, params.mushDistanceWeight);
        fn("inputs:displacement", 1.0f, params.mushDisplacement);
    } else if (op == RigExecRevisionOp::Wrinkle) {
        auto &settings = params.wrinkleSettings;
        fn("inputs:iterations", 80, settings.iterations);
        fn("inputs:neighborDistance", 2, settings.neighborDistance);
        fn("inputs:restLengthScale", 1.0f, settings.restLengthScale);
        fn("inputs:stretchStiffness", 1.0f, settings.stretchStiffness);
        fn("inputs:compressionStiffness", 1.0f, settings.compressionStiffness);
        fn("inputs:bendStiffness", 0.1f, settings.bendStiffness);
        fn("inputs:maxDisplacement", 0.2f, settings.maxDisplacement);
        fn("inputs:pinBorders", true, settings.pinBorders);
        fn("inputs:tangentPlaneCollisions", true,
           settings.tangentPlaneCollisions);
        fn("inputs:tangentPlaneInset", 0.0f, settings.tangentPlaneInset);
        fn("inputs:wrinkleScale", 1.0f, settings.wrinkleScale);
        fn("inputs:smoothingIterations", 0, settings.smoothingIterations);
    }
}

// One ordered field list per binding type. Sampling, burst-site discovery,
// snapshot capture, and worker patching all use these visitors.
template <class Obj, class Fn>
void
_VisitLadderInputs(Obj &ladder, Fn &&fn)
{
    fn(ladder.restSpace);
    fn(ladder.defaultSpace);
    fn(ladder.posedSpace);
    for (int i = 0; i < 6; ++i) {
        fn(ladder.restAvars[i]);
        fn(ladder.defaultAvars[i]);
    }
    fn(ladder.rotationOrder);
}

template <class Obj, class Fn>
void
_VisitSolverInputs(Obj &solver, Fn &&fn)
{
    fn(solver.bend);
    fn(solver.upperOffset);
    fn(solver.lowerOffset);
    fn(solver.stretch);
    fn(solver.softness);
    fn(solver.pin);
    fn(solver.upperScale);
    fn(solver.lowerScale);
    fn(solver.softDistance);
    fn(solver.limbTwist);
    fn(solver.blendWeight);
    fn(solver.preserveVolume);
    fn(solver.midFollowWeight);
    fn(solver.roll);
    fn(solver.twist);
    fn(solver.minLengthRatio);
    fn(solver.twistTurns);
    fn(solver.ribbonSampleCount);
    fn(solver.ikSpace);
}

template <class Obj, class Fn>
void
_VisitConstraintInputs(
    Obj &constraint, Fn &&fn)
{
    fn(constraint.enabled);
    fn(constraint.defaultWeight);
    fn(constraint.offset);
    fn(constraint.affectX);
    fn(constraint.affectY);
    fn(constraint.affectZ);
    fn(constraint.tX);
    fn(constraint.tY);
    fn(constraint.tZ);
    fn(constraint.rX);
    fn(constraint.rY);
    fn(constraint.rZ);
    fn(constraint.sX);
    fn(constraint.sY);
    fn(constraint.sZ);
    fn(constraint.aimVector);
    fn(constraint.upVector);
    fn(constraint.rotationOffset);
    fn(constraint.worldUpVector);
    fn(constraint.poleVector);
    fn(constraint.twistDegrees);
    fn(constraint.ikStretch);
}

template <class Obj, class Fn>
void
_VisitInterpolatorInputs(
    Obj &interp, Fn &&fn)
{
    fn(interp.enabled);
    // A numeric driver's dials, in rigExec:driverAttributes order.
    for (auto &value : interp.valueInputs) {
        fn(value);
    }
}

template <class Obj, class Fn>
void
_VisitSpaceSwitchInputs(Obj &spaceSwitch, Fn &&fn)
{
    fn(spaceSwitch.activeInput);
}

template <class Obj, class Fn>
void
_VisitWeightInputs(Obj &object, Fn &&fn)
{
    fn(object.defaultWeight);
    fn(object.driver);
    fn(object.scale);
    fn(object.bias);
    fn(object.strength);
    fn(object.invert);
    fn(object.falloffMin);
    fn(object.falloffMax);
    fn(object.scaleXPos);
    fn(object.scaleYPos);
    fn(object.scaleZPos);
    fn(object.scaleXNeg);
    fn(object.scaleYNeg);
    fn(object.scaleZNeg);
    fn(object.scaleX);
    fn(object.scaleY);
    fn(object.scaleZ);
    fn(object.extentU);
    fn(object.extentV);
}

// The weight-object schema types the frozen packet build mirrors
// (bakedWeights.cpp names the same set).
TF_DEFINE_PRIVATE_TOKENS(
    _frozenWeightTokens,
    ((staticWeight, "RigExecStaticWeight"))
    ((dynamicWeight, "RigExecDynamicWeight"))
    ((combineWeight, "RigExecCombineWeight"))
    ((sphereWeight, "RigExecSphereWeight"))
    ((planeWeight, "RigExecPlaneWeight"))
    ((curveWeight, "RigExecCurveWeight"))
);

SdfPath
_FrozenWeightArrayKey(const SdfPath &objectPath, const char *role);

SdfPath
_FrozenWireInputKey(const SdfPath &moverPath, const char *role);

SdfPath
_FrozenBlendInputKey(const SdfPath &samplePath, const char *role);

bool
_FrozenPlaceOverrides(const RigExecBakedProgramImpl &B,
                      const std::vector<RigExecValueOverride> &overrides,
                      std::vector<char> *flags);

SdfPath
_FrozenSurfaceInputKey(const SdfPath &moverPath, const char *role);

SdfPath
_FrozenLatticeInputKey(const SdfPath &moverPath, const char *role);

SdfPath
_FrozenRibbonInputKey(const SdfPath &moverPath, const char *role);

bool
_ChainIsFinite(float v);

bool
_ChainIsFinite(const GfVec3f &v);

bool
_ChainIsFinite(const GfMatrix4d &m);

struct _ChainMoverDesc {
    SdfPath moverPath;
    TfToken schemaType;
    SdfPath target;
};

bool
_DiscoverChainMovers(const RigExecRigEvaluator &evaluator,
                     std::vector<_ChainMoverDesc> *out, std::string *error);

bool
_OrderDiscoveredChains(
    const UsdStageRefPtr &stage,
    const std::map<SdfPath, std::vector<_ChainMoverDesc>> &chains,
    std::vector<SdfPath> *order, std::string *error);

RigExecChainSampleInput
_BindChainInput(const UsdPrim &prim, const char *name);

uint64_t
_MixWord(uint64_t hash, uint64_t word);

uint64_t
_HashBytes(uint64_t hash, const void *data, size_t size);

uint64_t
_HashString(uint64_t hash, const char *text);

bool
_HashVtValue(uint64_t *hash, const VtValue &value);

extern thread_local std::shared_ptr<const void> _lastFrozenSlots;

extern thread_local size_t _lastFrozenSlotBytes;

// Visits each auto clavicle's IK/FK blend and dial, which the compose reads
// beside the avars (space switches have their own visitor above). They are
// read live (RigExecIsLiveAvarName), so they must key the frame cache and be
// patched into a frozen run like any other varying input.
template <class Impl, class Fn>
void
_VisitComposeInputs(Impl &B, Fn &&fn)
{
    for (auto &ac : B.autoClavicles) {
        fn(ac.ikBlendInput);
        fn(ac.ikBlendFloat);
        fn(ac.amountInput);
        fn(ac.amountFloat);
    }
}

// Visits every patchable input in one fixed order. The freeze uses it to
// capture head paths (UI thread, handles valid there) and the worker uses
// it to patch constants (side-table keys, no handle dereference); sharing
// the walker is what keeps the two in lockstep.
template <class Impl, class Fn>
void
_ForEachPatchableInput(Impl &B, Fn &&fn)
{
    for (auto &binding : B.avarBindings) {
        fn(binding.input);
    }
    for (auto &binding : B.avarConstantBindings) {
        fn(binding.input);
    }
    for (auto &ladder : B.ladders) {
        _VisitLadderInputs(ladder, fn);
    }
    for (auto &spaceSwitch : B.spaceSwitches) {
        _VisitSpaceSwitchInputs(spaceSwitch, fn);
    }
    for (auto &interpolator : B.poseInterpolators) {
        _VisitInterpolatorInputs(interpolator, fn);
    }
    for (auto &solver : B.solvers) {
        _VisitSolverInputs(solver, fn);
    }
    for (auto &constraint : B.constraints) {
        _VisitConstraintInputs(constraint, fn);
    }
    for (auto &object : B.weightObjects) {
        _VisitWeightInputs(object, fn);
    }
    _VisitComposeInputs(B, fn);
}

void
_CloneImpl(const RigExecBakedProgramImpl &src, RigExecBakedProgramImpl *dst);

// Worker-side state and input patching.

// One job's private working state: the snapshot cloned onto the worker plus
// the live-state stand-ins the reused code dereferences (resolved inputs,
// snapshot stores, profiler, guide taps, guides flag). Nothing in here is
// shared between jobs; nothing in here names the stage or the evaluator.
struct _FrozenWorker {
    RigExecBakedProgramImpl B;
    RigExecResolvedInputs resolved;
    RigExecChainSnapshots chainSnapshots;
    RigExecProfiler profiler;
    bool guidesEnabled = false;
    std::unique_ptr<RigExecTapSet> nullTaps;
    // The worker's own placement map: the snapshot nulls the evaluator's
    // volumeWeightMatrices pointer, so the frozen VolumePlacements body
    // writes here, and B.volumeWeightMatrices points at it for the packet
    // build and pose publish to read.
    std::map<SdfPath, GfMatrix4d> volumeWeightMatrices;
};

template <class T>
bool
_SampleHolds(const VtValue &held, T *out)
{
    if (held.IsHolding<T>()) {
        *out = held.UncheckedGet<T>();
        return true;
    }
    return false;
}

// The one sanctioned cross-type read: a float input over a double source,
// which the frame path coerces at consumption (GetAttribute's float arm and
// _CoerceFromDouble). Every other pair must hold exactly.
template <>
inline bool
_SampleHolds<float>(const VtValue &held, float *out)
{
    if (held.IsHolding<float>()) {
        *out = held.UncheckedGet<float>();
        return true;
    }
    if (held.IsHolding<double>()) {
        *out = float(held.UncheckedGet<double>());
        return true;
    }
    return false;
}

const TfToken &
_FrozenKindToken(RigExecRevisionOp op);

bool
_FrozenStepBody(_FrozenWorker *worker, RigExecBakedStep *step,
               const std::map<SdfPath, size_t> &index,
               const RigExecFrameInputs &inputs, UsdTimeCode time);

bool
_RunFrozen(const RigExecFrozenEvalContext &context,
           const RigExecFrameInputs &inputs, RigExecRigPose *pose);

} // namespace frozenDetail

} // namespace rigExec

#endif
