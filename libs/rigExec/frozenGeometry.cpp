// Worker-side weight and geometry assembly from sampled data.

#include "frozenContextInternal.h"
#include "weightPackets.h"
#include "movers/moverRegistry.h"
#include "rigExecMath/geometryKernels.h"
#include <algorithm>
#include <cmath>
#include <set>

namespace rigExec {

using namespace frozenDetail;

namespace {

// Assembles a normals/extent packet on the worker from the region's points
// plus the sampled topology. Replicates moverGraph.cpp's derived arm
// (2820-2836) line for line: synthesized enable and unit weights, topology
// and widths replayed from their samples (_Array's arms: memory, here the
// sampled override-or-stage value, else empty), and the same validity rule.
bool
_AssembleDerivedPacket(const RigExecBakedProgramImpl::GeomRevision &revision,
                       const GfVec3f *basePoints, size_t baseCount,
                       const std::map<SdfPath, size_t> &index,
                       const RigExecFrameInputs &inputs,
                       RigExecMoverParameters *parameters)
{
    RigExecMoverParameters params;
    params.kind = _FrozenKindToken(revision.op);
    params.enabled = true;
    params.weights = RigExecWeightPacket::Constant(1.0f);
    if (!params.weights.valid) {
        *parameters = params;
        return true;
    }
    params.auxPoints.assign(basePoints, basePoints + baseCount);
    const auto readInts = [&index, &inputs](const SdfPath &path,
                                            std::vector<int> *out) -> bool {
        if (path.IsEmpty()) {
            return true;
        }
        const auto found = index.find(path);
        if (found == index.end()) {
            return true;
        }
        const RigExecSampledInput &sample = inputs.values[found->second];
        if (!sample.hasValue) {
            return true;
        }
        if (!sample.value.IsHolding<VtIntArray>()) {
            return false;
        }
        const VtIntArray &held = sample.value.UncheckedGet<VtIntArray>();
        out->assign(held.begin(), held.end());
        return true;
    };
    if (!readInts(revision.binding.topologyCounts, &params.topologyCounts) ||
        !readInts(revision.binding.topologyIndices,
                  &params.topologyIndices)) {
        return false;
    }
    if (revision.op == RigExecRevisionOp::RecomputeExtent &&
        !revision.binding.widths.IsEmpty()) {
        const auto found = index.find(revision.binding.widths);
        if (found != index.end()) {
            const RigExecSampledInput &sample =
                inputs.values[found->second];
            if (sample.hasValue) {
                if (!sample.value.IsHolding<VtFloatArray>()) {
                    return false;
                }
                const VtFloatArray &held =
                    sample.value.UncheckedGet<VtFloatArray>();
                params.widths.assign(held.begin(), held.end());
            }
        }
    }
    params.valid =
        !params.auxPoints.empty() &&
        (revision.op == RigExecRevisionOp::RecomputeExtent ||
         !params.topologyCounts.empty());
    *parameters = params;
    return true;
}

// The frozen weight-object step: RigExecBakedWeightPacket restated over
// patched scalar inputs (the _ForEachPatchableInput walker covers them)
// and sampled point arrays (the synthetic keys _SampleWeightArrays
// writes). The builders are the shared pure kernels; only the reads
// differ, because the worker cannot touch the stage or the oracle.
// Volume placements go through RigExecVolumePlacement and write the
// worker's own program table, as the live VolumePlacements step does.
bool
_FrozenWeightStep(_FrozenWorker *worker, RigExecBakedStep *step,
                  const std::map<SdfPath, size_t> &index,
                  const RigExecFrameInputs &inputs, UsdTimeCode time)
{
    RigExecBakedProgramImpl &B = worker->B;
    if (step->kind == RigExecBakedStepKind::VolumePlacements) {
        // This step's no-scale provider, placed or identity, from the frame
        // the walk ended with.
        const size_t slot = size_t(step->object);
        if (slot >= B.noScaleAvars.size() || !B.noScaleAvars[slot]) {
            return false;
        }
        B.volumePlacement[slot] =
            RigExecVolumePlacement(B.fin[size_t(B.finLast[slot])]);
        return true;
    }
    const int id = step->object;
    if (id < 0 || size_t(id) >= B.weightObjects.size() ||
        size_t(id) >= B.weightPackets.size()) {
        return false;
    }
    RigExecBakedProgramImpl::WeightObject &object = B.weightObjects[size_t(id)];
    // The worker prologue sampled every leaf from the patched constants.
    const auto rd = [&B](const auto &input) {
        return RigExecBakedLeafRead(B, input);
    };
    const auto findSample = [&](const SdfPath &path) {
        const auto found = index.find(path);
        if (found == index.end()) {
            return static_cast<const RigExecSampledInput *>(nullptr);
        }
        return &inputs.values[found->second];
    };
    const auto samplePoints = [&](const char *role,
                                  std::vector<GfVec3f> *out) {
        if (const RigExecSampledInput *sample =
                findSample(_FrozenWeightArrayKey(object.path, role))) {
            if (sample->hasValue &&
                sample->value.IsHolding<VtVec3fArray>()) {
                const VtVec3fArray &held =
                    sample->value.UncheckedGet<VtVec3fArray>();
                out->assign(held.begin(), held.end());
            }
        }
    };
    if (object.type == _frozenWeightTokens->staticWeight) {
        RigExecStaticWeightInputs packetInputs;
        packetInputs.representation = object.representation;
        packetInputs.rangePolicy = object.rangePolicy;
        packetInputs.values = object.values;
        packetInputs.indices = object.indices;
        packetInputs.defaultWeight = rd(object.defaultWeight);
        B.weightPackets[size_t(id)] =
            RigExecBuildStaticWeightPacket(packetInputs);
        return true;
    }
    if (object.type == _frozenWeightTokens->dynamicWeight) {
        RigExecDynamicWeightInputs packetInputs;
        packetInputs.representation = object.representation;
        packetInputs.rangePolicy = object.rangePolicy;
        packetInputs.driver = rd(object.driver);
        packetInputs.scale = rd(object.scale);
        packetInputs.bias = rd(object.bias);
        const RigExecWeightPacket *base =
            object.base >= 0 ? &B.weightPackets[size_t(object.base)]
                             : nullptr;
        B.weightPackets[size_t(id)] =
            RigExecBuildDynamicWeightPacket(packetInputs, base);
        return true;
    }
    if (object.type == _frozenWeightTokens->combineWeight) {
        std::vector<RigExecWeightPacket> packetInputs;
        packetInputs.reserve(object.inputs.size());
        for (const int input : object.inputs) {
            packetInputs.push_back(B.weightPackets[size_t(input)]);
        }
        size_t targetCount = 0;
        if (const RigExecSampledInput *sample = findSample(
                _FrozenWeightArrayKey(object.path, "combineTargetCount"))) {
            if (sample->hasValue && sample->value.IsHolding<int>()) {
                targetCount =
                    size_t(sample->value.UncheckedGet<int>());
            }
        }
        B.weightPackets[size_t(id)] = RigExecBuildCombineWeightPacket(
            object.representation, object.rangePolicy, object.combineMode,
            packetInputs, targetCount, rd(object.strength),
            rd(object.invert));
        return true;
    }
    if (object.type == _frozenWeightTokens->sphereWeight ||
        object.type == _frozenWeightTokens->planeWeight ||
        object.type == _frozenWeightTokens->curveWeight) {
        RigExecVolumeWeightInputs packetInputs;
        packetInputs.representation = object.representation;
        packetInputs.rangePolicy = object.rangePolicy;
        if (object.providerSlot >= 0) {
            packetInputs.placement =
                B.base[size_t(B.baseLast[size_t(object.providerSlot)])];
            packetInputs.hasPlacement = true;
        }
        packetInputs.params.falloffMin = rd(object.falloffMin);
        packetInputs.params.falloffMax = rd(object.falloffMax);
        packetInputs.params.invert = rd(object.invert);
        packetInputs.params.strength = rd(object.strength);
        packetInputs.params.curve = object.falloffCurve;
        if (object.type != _frozenWeightTokens->planeWeight) {
            packetInputs.positiveScales = GfVec3f(
                rd(object.scaleXPos),
                rd(object.scaleYPos),
                rd(object.scaleZPos));
            packetInputs.negativeScales = GfVec3f(
                rd(object.scaleXNeg),
                rd(object.scaleYNeg),
                rd(object.scaleZNeg));
            packetInputs.scales = GfVec3f(rd(object.scaleX), rd(object.scaleY),
                                          rd(object.scaleZ));
        } else {
            packetInputs.planeAxis = object.planeAxis;
            packetInputs.planeBounds = object.planeBounds;
            if (object.planeBounds == "bounded") {
                packetInputs.extentU = rd(object.extentU);
                packetInputs.extentV = rd(object.extentV);
            }
        }
        if (RigExecVolumeWeightCanBuild(object.type, packetInputs)) {
            samplePoints("targetPoints", &packetInputs.targetPoints);
            samplePoints("samplePoints", &packetInputs.samplePoints);
            if (object.type == _frozenWeightTokens->curveWeight) {
                samplePoints("curvePoints", &packetInputs.curvePoints);
            }
            B.weightPackets[size_t(id)] =
                RigExecBuildVolumeWeightPacket(object.type, packetInputs);
        } else {
            // Live builds nothing here either: the packet stays whatever
            // the step left, which for a fresh run is default (invalid).
            B.weightPackets[size_t(id)] = RigExecWeightPacket();
        }
        return true;
    }
    B.weightPackets[size_t(id)] = RigExecWeightPacket();
    return true;
}

// The common revision envelope (moverGraph.cpp): the bound weight packet,
// or the constant packet synthesized from inputs:defaultWeight. Every
// non-derived op carries exactly this; an invalid envelope fails the
// revision before any op-specific assembly runs.
bool
_FrozenCommonEnvelope(const RigExecBakedProgramImpl &B,
                      const RigExecBakedProgramImpl::GeomRevision &revision,
                      float defaultWeight, RigExecMoverParameters *params)
{
    if (revision.weightObject >= 0 &&
        size_t(revision.weightObject) < B.weightPackets.size()) {
        params->weights = B.weightPackets[size_t(revision.weightObject)];
    } else {
        params->weights = RigExecWeightPacket::Constant(defaultWeight);
    }
    return params->weights.valid;
}

bool
_FrozenSampledEnabled(const std::map<SdfPath, size_t> &index,
                      const RigExecFrameInputs &inputs, const SdfPath &moverPath)
{
    const SdfPath key =
        moverPath.AppendProperty(TfToken("inputs:enabled"));
    const auto found = index.find(key);
    if (found == index.end()) {
        return true;
    }
    const RigExecSampledInput &sample = inputs.values[found->second];
    bool enabled = true;
    if (sample.hasValue && !_SampleHolds(sample.value, &enabled)) {
        return true;
    }
    return enabled;
}

// The phased overlay for one side-input read: when the revision phases
// the input's binding path and this run resolved it, the snapshot value
// shadows the sample, exactly as values.resolved shadows the stage read
// on live. Null otherwise, and the sample answers. Only phased paths
// consult the overlay, so a revision that declares none never touches
// its (stale) revisionInputs.
const VtValue *
_FrozenPhasedValue(
    const RigExecBakedProgramImpl::GeomRevision &revision,
    const SdfPath &bindingPath)
{
    if (revision.binding.phases.find(bindingPath) ==
        revision.binding.phases.end()) {
        return nullptr;
    }
    return revision.revisionInputs.Find(bindingPath);
}

// One phased side-input array: the overlay first, the sample when the
// input is unphased, unresolved, or holding another type (a mistyped
// overlay holding reads as a miss, as live, and the sample answers).
template <class T>
bool
_FrozenSideArray(
    const RigExecBakedProgramImpl::GeomRevision &revision,
    const SdfPath &bindingPath, const SdfPath &sampleKey,
    const std::map<SdfPath, size_t> &index,
    const RigExecFrameInputs &inputs, std::vector<T> *out)
{
    out->clear();
    if (const VtValue *phased = _FrozenPhasedValue(revision, bindingPath)) {
        if (phased->IsHolding<VtArray<T>>()) {
            const VtArray<T> &held = phased->UncheckedGet<VtArray<T>>();
            out->assign(held.begin(), held.end());
            return true;
        }
    }
    if (sampleKey.IsEmpty()) {
        return true;
    }
    const auto found = index.find(sampleKey);
    if (found == index.end()) {
        return true;
    }
    const RigExecSampledInput &sample = inputs.values[found->second];
    if (!sample.hasValue) {
        return true;
    }
    if (!sample.value.IsHolding<VtArray<T>>()) {
        return false;
    }
    const VtArray<T> &held = sample.value.UncheckedGet<VtArray<T>>();
    out->assign(held.begin(), held.end());
    return true;
}

bool
_FrozenAssembleIterativeMover(
    const RigExecBakedProgramImpl &B,
    const RigExecBakedProgramImpl::GeomRevision &revision, float defaultWeight,
    const VtVec3fArray &basePoints, const std::map<SdfPath, size_t> &index,
    const RigExecFrameInputs &inputs, RigExecMoverParameters *params)
{
    *params = RigExecMoverParameters();
    params->kind = RigExecRevisionKindToken(revision.op);
    params->enabled = _FrozenSampledEnabled(index, inputs, revision.moverPath);
    if (!params->enabled) {
        params->valid = true;
        return true;
    }
    if (!_FrozenCommonEnvelope(B, revision, defaultWeight, params)) {
        return true;
    }
    const SdfPath rest = revision.moverPath.AppendProperty(
        TfToken("inputs:restPoints"));
    if (!_FrozenSideArray(revision, rest, rest, index, inputs,
                          &params->restPoints) ||
        !_FrozenSideArray(revision, revision.binding.topologyCounts,
                          revision.binding.topologyCounts, index, inputs,
                          &params->topologyCounts) ||
        !_FrozenSideArray(revision, revision.binding.topologyIndices,
                          revision.binding.topologyIndices, index, inputs,
                          &params->topologyIndices)) {
        return false;
    }
    if (params->restPoints.empty()) {
        params->restPoints.assign(basePoints.begin(), basePoints.end());
    }
    bool validSamples = true;
    const auto scalar = [&](const char *name, auto fallback, auto &value) {
        value = fallback;
        const SdfPath path = revision.moverPath.AppendProperty(TfToken(name));
        if (const VtValue *phased = _FrozenPhasedValue(revision, path)) {
            if (_SampleHolds(*phased, &value)) {
                return;
            }
        }
        const auto found = index.find(path);
        if (found != index.end()) {
            const RigExecSampledInput &sample = inputs.values[found->second];
            if (sample.hasValue && !_SampleHolds(sample.value, &value)) {
                validSamples = false;
            }
        }
    };
    _VisitIterativeMoverScalars(revision.op, *params, scalar);
    if (revision.op == RigExecRevisionOp::Wrinkle) {
        TfToken topology;
        scalar("inputs:topology", TfToken("cloth"), topology);
        if (topology != "cloth" && topology != "surfaceStruts") {
            return validSamples;
        }
        params->wrinkleSettings.topology = topology == "cloth"
            ? RigExecWrinkleTopology::Cloth
            : RigExecWrinkleTopology::SurfaceStruts;
        const SdfPath pins = revision.moverPath.AppendProperty(
            TfToken("inputs:pinPoints"));
        if (!_FrozenSideArray(revision, pins, pins, index, inputs,
                              &params->wrinkleSettings.pinPoints)) {
            return false;
        }
    }
    params->valid = !params->restPoints.empty() && !params->topologyCounts.empty();
    return validSamples;
}

// The wire assembly arm (moverGraph.cpp) over frozen state: rest/order/
// knots/dropoff/bind samples plus the fold's influence table, or the posed
// driver curve when no driver transforms bind. Table math (pick/measured,
// base motion, per-point blend) is the live arm restated.
bool
_FrozenAssembleWire(
    const RigExecBakedProgramImpl &B,
    const RigExecBakedProgramImpl::GeomRevision &revision, float defaultWeight,
    const std::map<SdfPath, size_t> &index, const RigExecFrameInputs &inputs,
    RigExecMoverParameters *params)
{
    const SdfPath &moverPath = revision.moverPath;
    const RigExecRevisionBinding &binding = revision.binding;
    params->kind = RigExecRevisionKindToken(RigExecRevisionOp::Wire);
    params->enabled =
        _FrozenSampledEnabled(index, inputs, moverPath);
    if (!params->enabled) {
        params->valid = true;  // disabled is an ordinary pass-through
        return true;
    }
    if (!_FrozenCommonEnvelope(B, revision, defaultWeight, params)) {
        return true;  // MoverFailed (kind+enabled stay set, as live)
    }
    const auto findSample = [&](const char *role) {
        const auto found =
            index.find(_FrozenWireInputKey(moverPath, role));
        if (found == index.end()) {
            return static_cast<const RigExecSampledInput *>(nullptr);
        }
        return &inputs.values[found->second];
    };
    if (const RigExecSampledInput *sample = findSample("restPoints")) {
        if (sample->hasValue &&
            sample->value.IsHolding<VtVec3fArray>()) {
            const VtVec3fArray &held =
                sample->value.UncheckedGet<VtVec3fArray>();
            params->restPoints.assign(held.begin(), held.end());
        }
    }
    if (const RigExecSampledInput *sample = findSample("curveOrder")) {
        if (sample->hasValue && sample->value.IsHolding<int>()) {
            params->curveOrder = sample->value.UncheckedGet<int>();
        }
    }
    if (const RigExecSampledInput *sample = findSample("curveKnots")) {
        if (sample->hasValue &&
            sample->value.IsHolding<VtDoubleArray>()) {
            const VtDoubleArray &held =
                sample->value.UncheckedGet<VtDoubleArray>();
            params->curveKnots.assign(held.begin(), held.end());
        }
    }
    if (const RigExecSampledInput *sample = findSample("dropoffDistance")) {
        if (sample->hasValue && sample->value.IsHolding<float>()) {
            params->dropoffDistance =
                double(sample->value.UncheckedGet<float>());
        }
    }
    if (const VtValue *phased =
            _FrozenPhasedValue(revision, binding.bindCoords)) {
        if (phased->IsHolding<VtArray<GfVec2f>>()) {
            params->wireBindCoords =
                phased->UncheckedGet<VtArray<GfVec2f>>();
        } else if (const RigExecSampledInput *sample =
                       findSample("bindCoords")) {
            if (sample->hasValue &&
                sample->value.IsHolding<VtArray<GfVec2f>>()) {
                params->wireBindCoords =
                    sample->value.UncheckedGet<VtArray<GfVec2f>>();
            }
        }
    } else if (const RigExecSampledInput *sample =
                   findSample("bindCoords")) {
        if (sample->hasValue &&
            sample->value.IsHolding<VtArray<GfVec2f>>()) {
            params->wireBindCoords =
                sample->value.UncheckedGet<VtArray<GfVec2f>>();
        }
    }
    if (binding.driverTransformCount > 0) {
        const size_t t = size_t(binding.driverTransformCount);
        const size_t s = size_t(binding.driverSpaceCount);
        const size_t bt = size_t(binding.driverBaseTransformCount);
        const std::vector<GfMatrix4d> *table = &revision.influences;
        if (table->size() < t + s + bt) {
            return true;  // MoverFailed (valid stays false)
        }
        VtFloatArray driverWeights, driverBaseWeights;
        if (const RigExecSampledInput *sample =
                findSample("driverWeights")) {
            if (sample->hasValue &&
                sample->value.IsHolding<VtFloatArray>()) {
                driverWeights =
                    sample->value.UncheckedGet<VtFloatArray>();
            }
        }
        if (const RigExecSampledInput *sample =
                findSample("driverBaseWeights")) {
            if (sample->hasValue &&
                sample->value.IsHolding<VtFloatArray>()) {
                driverBaseWeights =
                    sample->value.UncheckedGet<VtFloatArray>();
            }
        }
        // The wire's two frame tokens, off the recorded reads as live
        // records them, and the fold's carry: the live arm's inputs.
        const auto sampledToken = [&](const char *name,
                                      const TfToken &fallback) {
            const auto found = index.find(
                moverPath.AppendProperty(TfToken(name)));
            TfToken value = fallback;
            if (found != index.end()) {
                const RigExecSampledInput &sample =
                    inputs.values[found->second];
                TfToken held;
                if (sample.hasValue && _SampleHolds(sample.value, &held)) {
                    value = held;
                }
            }
            return value;
        };
        RigExecWireDriverFrame frame;
        frame.posedPoints =
            sampledToken("rigExec:pointFrame", TfToken("rest")) == "posed";
        frame.posedDelta =
            sampledToken("rigExec:driverDeltaFrame", TfToken("local")) ==
            "posed";
        frame.carry = revision.haveCarry ? &revision.carry : nullptr;
        RigExecPoseWireDrivers(*table, t, s, bt, driverWeights,
                               driverBaseWeights, frame, &params->restPoints,
                               &params->auxPoints);
    } else if (const VtValue *phased =
                   _FrozenPhasedValue(revision,
                                       binding.driverCurvePoints)) {
        if (phased->IsHolding<VtVec3fArray>()) {
            const VtVec3fArray &held =
                phased->UncheckedGet<VtVec3fArray>();
            params->auxPoints.assign(held.begin(), held.end());
        } else if (const RigExecSampledInput *sample =
                       findSample("driverCurvePoints")) {
            if (sample->hasValue &&
                sample->value.IsHolding<VtVec3fArray>()) {
                const VtVec3fArray &held =
                    sample->value.UncheckedGet<VtVec3fArray>();
                params->auxPoints.assign(held.begin(), held.end());
            }
        }
    } else if (const RigExecSampledInput *sample =
                   findSample("driverCurvePoints")) {
        if (sample->hasValue &&
            sample->value.IsHolding<VtVec3fArray>()) {
            const VtVec3fArray &held =
                sample->value.UncheckedGet<VtVec3fArray>();
            params->auxPoints.assign(held.begin(), held.end());
        }
    }
    const RigExecNurbsCurve rest{&params->restPoints, params->curveOrder,
                                 &params->curveKnots};
    params->valid = rest.IsValid() &&
                    params->auxPoints.size() == params->restPoints.size() &&
                    !params->wireBindCoords.empty();
    return true;
}

// Reads one _Array arm's sample: missing or valueless is the empty array,
// as live; a mistyped holding fails closed (the sampler only ever stores
// the arm's own type, so anything else is a corrupt vector).
template <class T>
bool
_FrozenSampledArray(const std::map<SdfPath, size_t> &index,
                    const RigExecFrameInputs &inputs, const SdfPath &key,
                    std::vector<T> *out)
{
    out->clear();
    if (key.IsEmpty()) {
        return true;
    }
    const auto found = index.find(key);
    if (found == index.end()) {
        return true;
    }
    const RigExecSampledInput &sample = inputs.values[found->second];
    if (!sample.hasValue) {
        return true;
    }
    if (!sample.value.IsHolding<VtArray<T>>()) {
        return false;
    }
    const VtArray<T> &held = sample.value.UncheckedGet<VtArray<T>>();
    out->assign(held.begin(), held.end());
    return true;
}

// The Lattice arm over frozen state: kind, enabled, the shared envelope,
// then rest points plus the rest/live cage pair and the divisions, in live
// order. auxPoints is the BIND-TIME cage and auxPointsB the live one --
// reversing them inverts the deformation as soon as the cage moves.
bool
_FrozenAssembleLattice(
    const RigExecBakedProgramImpl &B,
    const RigExecBakedProgramImpl::GeomRevision &revision,
    float defaultWeight, const VtVec3fArray &lastBase,
    const std::map<SdfPath, size_t> &index,
    const RigExecFrameInputs &inputs, RigExecMoverParameters *params)
{
    const SdfPath &moverPath = revision.moverPath;
    const RigExecRevisionBinding &latticeBinding = revision.binding;
    params->kind = RigExecRevisionKindToken(RigExecRevisionOp::Lattice);
    params->enabled =
        _FrozenSampledEnabled(index, inputs, moverPath);
    if (!params->enabled) {
        params->valid = true;  // disabled is an ordinary pass-through
        return true;
    }
    if (!_FrozenCommonEnvelope(B, revision, defaultWeight, params)) {
        return true;  // MoverFailed (kind+enabled stay set, as live)
    }
    params->restPoints.assign(lastBase.begin(), lastBase.end());
    if (!_FrozenSampledArray(
            index, inputs, _FrozenLatticeInputKey(moverPath, "cageRest"),
            &params->auxPoints) ||
        !_FrozenSideArray(
            revision, latticeBinding.cagePoints,
            _FrozenLatticeInputKey(moverPath, "cageLive"), index, inputs,
            &params->auxPointsB)) {
        return false;
    }
    params->divisions = GfVec3i(0, 0, 0);
    const auto found = index.find(
        moverPath.AppendProperty(TfToken("rigExec:divisions")));
    if (found != index.end()) {
        const RigExecSampledInput &sample = inputs.values[found->second];
        if (sample.hasValue &&
            !_SampleHolds(sample.value, &params->divisions)) {
            return false;
        }
    }
    const size_t cageCount = size_t(params->divisions[0]) *
                             size_t(params->divisions[1]) *
                             size_t(params->divisions[2]);
    params->valid = params->divisions[0] >= 2 &&
                    params->divisions[1] >= 2 &&
                    params->divisions[2] >= 2 &&
                    params->auxPoints.size() == cageCount &&
                    params->auxPointsB.size() == cageCount &&
                    !params->restPoints.empty();
    return true;
}

// The frozen RevisionStatic: the RevisionStatic arm of
// RigExecBakedRunGeometryStep (bakedGeometry.cpp), with the mover-prim
// defaultWeight read replaced by its sample. A revision the path leaves
// assemble is assembled by the live function itself from the job's leaves
// (RigExecBakedAssembleFromLeaves; a skin from its transported packet), the
// rest by the worker-side arms above. Everything else -- status, dirty
// compare, publication sizing, layout and envelope decisions -- is the same
// code shape over the same fields.
bool
_FrozenRevisionStatic(_FrozenWorker *worker, RigExecBakedStep *step,
                      const std::map<SdfPath, size_t> &index,
                      const RigExecFrameInputs &inputs)
{
    RigExecBakedProgramImpl &B = worker->B;
    const auto &[chainIndex, revisionIndex] =
        B.revisionIndex[size_t(step->object)];
    RigExecBakedProgramImpl::GeomChain &chain = B.chains[size_t(chainIndex)];
    if (!chain.haveBase) {
        return true;
    }
    RigExecBakedProgramImpl::GeomRevision &revision =
        chain.revisions[size_t(revisionIndex)];
    if (revision.weightCurrentPhase) {
        return false;
    }
    revision.defaultWeight = 1.0f;
    {
        const SdfPath key = revision.moverPath.AppendProperty(
            TfToken("inputs:defaultWeight"));
        const auto found = index.find(key);
        if (found != index.end()) {
            const RigExecSampledInput &sample =
                inputs.values[found->second];
            if (sample.hasValue) {
                if (!_SampleHolds(sample.value, &revision.defaultWeight)) {
                    return false;
                }
            }
        }
    }
    const bool fromLeaves = revision.leaves.decl.assembles &&
                            revision.op != RigExecRevisionOp::Skin;
    // One overlay per revision that declares phases: the worker's resolved
    // inputs plus whatever the phases' bindings resolve to over the
    // worker's own chains -- the overlay AssembleRevision builds, by the
    // same function. The side-input reads below consult it by binding path
    // before their samples, exactly where live consults values.resolved.
    // The leaf assembly builds its own.
    if (!revision.binding.phases.empty()) {
        if (!B.resolvedInputs) {
            return false;
        }
        if (!fromLeaves) {
            RigExecBakedOverlayPointReads(&B, &revision, *B.resolvedInputs,
                                          &step->diagnostics);
        }
    }
    if (fromLeaves) {
        // What a consistent job always holds, checked so a stale vector
        // declines rather than indexes past the worker's tables: the
        // driver's aggregate, the weight packet, the pose weight slots, a
        // layout for every sparse blend sample, and base-phase samples
        // (freeze refuses the rest).
        if (revision.driverFramesSolver >= 0 &&
            size_t(revision.driverFramesSolver) >= B.aggregates.size()) {
            return false;
        }
        if (revision.weightObject >= 0 &&
            size_t(revision.weightObject) >= B.weightPackets.size()) {
            return false;
        }
        for (const auto &channel : revision.blendChannels) {
            if (channel.poseWeight >= 0 &&
                size_t(channel.poseWeight) >= B.poseWeights.size()) {
                return false;
            }
            for (const auto &sample : channel.samples) {
                if (!sample.phase.IsBase() ||
                    (!sample.blendShape.IsEmpty() && !sample.layout)) {
                    return false;
                }
            }
        }
        revision.parameters = RigExecBakedAssembleFromLeaves(
            &B, &revision, chain.lastBase.cdata(), chain.lastBase.size(),
            &step->diagnostics);
    } else if (revision.op == RigExecRevisionOp::Skin) {
        if (size_t(step->object) >= inputs.revisionPackets.size()) {
            return false;
        }
        revision.parameters = inputs.revisionPackets[size_t(step->object)];
        // Transported skin packets are assembled envelopeless; a bound
        // weight object overrides with its packet, as the live assemble
        // does. Envelopeless skins keep the transported packet untouched.
        if (revision.weightObject >= 0) {
            if (!_FrozenCommonEnvelope(B, revision, revision.defaultWeight,
                                       &revision.parameters)) {
                revision.parameters.valid = false;
            }
        }
    } else if (revision.op == RigExecRevisionOp::Wire) {
        if (!_FrozenAssembleWire(B, revision, revision.defaultWeight,
                                 index, inputs, &revision.parameters)) {
            return false;
        }
    } else if (revision.op == RigExecRevisionOp::DeltaMush ||
               revision.op == RigExecRevisionOp::Wrinkle) {
        if (!_FrozenAssembleIterativeMover(B, revision, revision.defaultWeight,
                                          chain.lastBase, index, inputs,
                                          &revision.parameters)) {
            return false;
        }
    } else if (revision.op == RigExecRevisionOp::Lattice) {
        if (!_FrozenAssembleLattice(B, revision, revision.defaultWeight,
                                    chain.lastBase, index, inputs,
                                    &revision.parameters)) {
            return false;
        }
    } else {
        return false;
    }
    revision.status =
        RigExecStatusForParameters(revision.parameters, revision.moverPath);
    step->counters.revisionsBuilt = 1;
    revision.staticDirty =
        !revision.ran || revision.parameters != revision.lastParameters ||
        revision.status != revision.lastStatus ||
        revision.defaultWeight != revision.lastDefaultWeight;
    revision.lastDefaultWeight = revision.defaultWeight;
    revision.weightFieldPublished = false;
    if (revision.weightObject >= 0 && B.publishWeightFields) {
        const RigExecWeightPacket &packet =
            B.weightPackets[size_t(revision.weightObject)];
        if (packet.valid) {
            const size_t logicalCount = revision.weightOperationDomain
                ? size_t(1)
                : chain.lastBase.size();
            if (!packet.ResolveAll(logicalCount, &revision.weightField)) {
                revision.weightField.assign(logicalCount, 0.0f);
                for (size_t i = 0; i < logicalCount; ++i) {
                    const float w = packet.Resolve(i, logicalCount);
                    revision.weightField[i] = w < 0.0f ? 0.0f : w;
                }
            }
            revision.weightFieldPublished = true;
        }
    }
    const size_t count = chain.lastBase.size();
    if (revision.output.size() != count) {
        revision.output.resize(count);
        revision.staticDirty = true;
    }
    revision.precedingCount = count;
    revision.layoutUsable = false;
    revision.envelopeOk = true;
    revision.fullStrength = true;
    revision.partitionStale = false;
    if (revision.op != RigExecRevisionOp::Skin) {
        return true;
    }
    revision.layoutUsable =
        RigExecSkinLayoutIsUsable(revision.parameters, count);
    revision.fullStrength =
        RigExecEnvelopeIsFullStrength(revision.parameters.weights);
    if (!revision.fullStrength) {
        revision.envelopeOk = revision.parameters.weights.ResolveAll(
            count, &revision.envelope);
    }
    if (revision.chunked) {
        const RigExecSkinTopology *const topology =
            revision.parameters.skinTopology.get();
        const size_t indexCount =
            topology ? topology->indices.size()
                     : revision.parameters.skinIndices.size();
        const int elementSize =
            topology ? topology->elementSize
                     : revision.parameters.skinElementSize;
        revision.partitionStale =
            !revision.parameters.skinTopology ||
            revision.parameters.skinTopology != revision.partitionTopology ||
            indexCount != revision.partitionIndexCount ||
            elementSize != revision.partitionElementSize ||
            count != revision.partitionPointCount;
    }
    return true;
}

// The frozen Derived step: bakedGeometry.cpp:2035's body with the same
// packet substitution (the base value here is the chain's published result,
// not its last base).
bool
_FrozenDerived(_FrozenWorker *worker, RigExecBakedStep *step,
              const std::map<SdfPath, size_t> &index,
              const RigExecFrameInputs &inputs)
{
    RigExecBakedProgramImpl &B = worker->B;
    const auto &[chainIndex, derivedIndex] =
        B.derivedIndex[size_t(step->object)];
    RigExecBakedProgramImpl::GeomChain &chain = B.chains[size_t(chainIndex)];
    RigExecBakedProgramImpl::GeomChain::Derived &derived =
        chain.derived[size_t(derivedIndex)];
    RigExecBakedProgramImpl::GeomRevision &revision = derived.revision;
    if (!chain.haveResult || !derived.haveBase) {
        return true;
    }
    if (derived.matrixTarget) {
        // RigExecReadProjectorTarget's reads, replayed from their samples,
        // then the same RigExecRunProjectorTarget the live paths run.
        const auto sampleOf = [&](const SdfPath &path)
            -> const RigExecSampledInput * {
            const auto found = index.find(path);
            if (found == index.end() || !inputs.values[found->second].hasValue) {
                return nullptr;
            }
            return &inputs.values[found->second];
        };
        const SdfPath &moverPath = revision.moverPath;
        RigExecProjectorReads reads;
        if (revision.op == RigExecRevisionOp::ShaderDials) {
            for (const SdfPath &dial : revision.binding.shaderDials) {
                const RigExecSampledInput *sample = sampleOf(dial);
                reads.dials.push_back(
                    sample && sample->value.IsHolding<double>()
                        ? sample->value.UncheckedGet<double>()
                        : 0.0);
            }
        } else {
            const auto vec = [&](const char *name, GfVec3d *out) {
                if (const RigExecSampledInput *sample = sampleOf(
                        moverPath.AppendProperty(TfToken(name)))) {
                    if (sample->value.IsHolding<GfVec3d>()) {
                        *out = sample->value.UncheckedGet<GfVec3d>();
                    }
                }
            };
            vec("rigExec:rayOrigin", &reads.rayOrigin);
            vec("rigExec:rayDirection", &reads.rayDirection);
            vec("rigExec:rayUp", &reads.rayUp);
            if (const RigExecSampledInput *sample = sampleOf(
                    moverPath.AppendProperty(
                        TfToken("rigExec:shaderOffset")))) {
                if (sample->value.IsHolding<GfMatrix4d>()) {
                    reads.shaderOffset =
                        sample->value.UncheckedGet<GfMatrix4d>();
                }
            }
            if (const RigExecSampledInput *sample = sampleOf(
                    moverPath.AppendProperty(
                        TfToken("rigExec:projectionMode")))) {
                TfToken mode;
                if (_SampleHolds(sample->value, &mode)) {
                    reads.reproject = mode == "reproject";
                }
            }
            const auto ints = [&](const SdfPath &path, std::vector<int> *out) {
                if (const RigExecSampledInput *sample = sampleOf(path)) {
                    if (sample->value.IsHolding<VtIntArray>()) {
                        const VtIntArray &held =
                            sample->value.UncheckedGet<VtIntArray>();
                        out->assign(held.begin(), held.end());
                    }
                }
            };
            ints(revision.binding.topologyCounts, &reads.faceVertexCounts);
            ints(revision.binding.topologyIndices, &reads.faceVertexIndices);
        }
        step->counters.revisionsBuilt = 1;
        derived.haveMatrix = RigExecRunProjectorTarget(
            revision.op, revision.binding,
            RigExecBakedProjectorFrames(B, revision), reads,
            std::vector<GfVec3f>(chain.lastBase.begin(), chain.lastBase.end()),
            std::vector<GfVec3f>(chain.result.begin(), chain.result.end()),
            &derived.matrix, &step->diagnostics);
        derived.haveResult = true;
        step->counters.chainsBuilt = 1;
        return true;
    }
    RigExecMoverParameters parameters;
    if (!_AssembleDerivedPacket(revision, chain.result.cdata(),
                                chain.result.size(), index, inputs,
                                &parameters)) {
        return false;
    }
    const RigExecMoverStatus status =
        RigExecStatusForParameters(parameters, revision.moverPath);
    step->counters.revisionsBuilt = 1;
    std::vector<GfVec3f> aux;
    aux.swap(parameters.auxPoints);
    const bool moved = chain.result != revision.lastAuxPoints ||
                       parameters != revision.lastParameters;
    parameters.auxPoints.swap(aux);
    if (derived.baseDirty || !revision.ran || moved ||
        status != revision.lastStatus) {
        step->counters.revisionsExecuted = 1;
        std::vector<GfVec3f> values(derived.lastBase.begin(),
                                    derived.lastBase.end());
        const bool applied =
            status.AllowsApply() &&
            RigExecRunRevisionKernel(revision.op, parameters, &values);
        revision.resultStatus = status.state;
        if (!applied) {
            values.assign(derived.lastBase.begin(), derived.lastBase.end());
            if (status.AllowsApply()) {
                static const TfToken moverFailed("moverFailed");
                revision.resultStatus = moverFailed;
            }
        }
        revision.output = std::move(values);
        parameters.auxPoints.clear();
        parameters.auxPoints.shrink_to_fit();
        revision.lastParameters = std::move(parameters);
        revision.lastAuxPoints = chain.result;
        revision.lastStatus = status;
        revision.ran = true;
    }
    if (revision.resultStatus == "moverFailed") {
        step->diagnostics.push_back(
            "MoverFailed " + derived.target.GetString() +
            ": derived geometry input/cardinality validation failed");
    }
    derived.spare.resize(revision.output.size());
    std::copy(revision.output.begin(), revision.output.end(),
              derived.spare.data());
    derived.result.swap(derived.spare);
    derived.haveResult = true;
    step->counters.chainsBuilt = 1;
    return true;
}

} // namespace

namespace frozenDetail {

// One step's body, with the two stage-reading bodies substituted. Otherwise
// the same dispatch RunStepBody performs (bakedSchedule.cpp:1524),
// including BeginRun's clearing promise and the weight-step refusal.
bool
_FrozenStepBody(_FrozenWorker *worker, RigExecBakedStep *step,
               const std::map<SdfPath, size_t> &index,
               const RigExecFrameInputs &inputs, UsdTimeCode time)
{
    RigExecBakedProgramImpl &B = worker->B;
    step->BeginRun();
    if (step->kind == RigExecBakedStepKind::WeightPacket ||
        step->kind == RigExecBakedStepKind::VolumePlacements) {
        return _FrozenWeightStep(worker, step, index, inputs, time);
    }
    if (step->kind == RigExecBakedStepKind::RevisionStatic) {
        return _FrozenRevisionStatic(worker, step, index, inputs);
    }
    if (step->kind == RigExecBakedStepKind::Derived) {
        return _FrozenDerived(worker, step, index, inputs);
    }
    if (RigExecBakedIsGeometryStep(step->kind)) {
        RigExecBakedRunGeometryStep(&B, step, time);
    } else {
        RigExecBakedRunPoseStep(&B, step, time);
    }
    return true;
}

} // namespace frozenDetail

} // namespace rigExec
