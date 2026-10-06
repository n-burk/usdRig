// Worker-side weight and geometry assembly from sampled data.

#include "frozenContextInternal.h"
#include "weightPackets.h"
#include "movers/moverRegistry.h"
#include "rigExecMath/geometryKernels.h"

#include "pxr/base/tf/staticTokens.h"

#include <algorithm>
#include <cmath>
#include <set>

// Names the frozen bodies need as tokens, constructed on the owning thread
// by Build (RigExecFrozenGeometryTouchTokens) and never on a worker.
TF_DEFINE_PRIVATE_TOKENS(
    _frozenBodyTokens,
    ((defaultWeight, "inputs:defaultWeight"))
    ((moverFailed, "moverFailed"))
);

namespace rigExec {

void
RigExecFrozenGeometryTouchTokens()
{
    (void)_frozenBodyTokens.Get();
    (void)frozenDetail::_frozenWeightTokens.Get();
}

using namespace frozenDetail;

namespace {

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
    const auto samplePoints = [&](const TfToken &role,
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
                _FrozenWeightArrayKey(
                    object.path, _frozenWeightTokens->combineTargetCountKey))) {
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
            samplePoints(_frozenWeightTokens->targetPointsKey,
                         &packetInputs.targetPoints);
            samplePoints(_frozenWeightTokens->samplePointsKey,
                         &packetInputs.samplePoints);
            if (object.type == _frozenWeightTokens->curveWeight) {
                samplePoints(_frozenWeightTokens->curvePointsKey,
                             &packetInputs.curvePoints);
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

// The frozen RevisionStatic: the RevisionStatic arm of
// RigExecBakedRunGeometryStep (bakedGeometry.cpp), with the mover-prim
// defaultWeight read replaced by its sample. Every revision is assembled by
// the live function itself from the job's leaves
// (RigExecBakedAssembleFromLeaves), a skin with the layout its SkinTopology
// op built in the prologue.
// Everything else -- status, dirty compare, publication sizing, layout and
// envelope decisions -- is the same code shape over the same fields.
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
    const int weightLeaf = revision.defaultWeightLeaf;
    if (weightLeaf >= 0 && size_t(weightLeaf) < revision.leaves.walks.size() &&
        revision.leaves.walks[size_t(weightLeaf)] >= 0) {
        // A chain or record can answer it: the leaf the prologue resolved
        // from the head tier, which is what live's RevisionStatic reads.
        revision.defaultWeight =
            revision.leaves.Value<float>(weightLeaf, 1.0f);
    } else {
        const SdfPath key = revision.moverPath.AppendProperty(
            _frozenBodyTokens->defaultWeight);
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
    // An external mover's plugin is never handed to a worker (freeze
    // refuses the rig).
    if (revision.op == RigExecRevisionOp::External) {
        return false;
    }
    const bool fromLeaves = revision.leaves.decl.assembles;
    // One overlay per revision that declares phases: the worker's resolved
    // inputs plus whatever the phases' bindings resolve to over the
    // worker's own chains -- the overlay AssembleRevision builds, by the
    // same function. The leaf assembly builds its own.
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

// The frozen Derived step: the live Derived body over the job's leaves (the
// packet's base value is the chain's published result, not its last base).
bool
_FrozenDerived(_FrozenWorker *worker, RigExecBakedStep *step)
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
        // The live Derived step's projector run, over the job's leaves.
        step->counters.revisionsBuilt = 1;
        derived.haveMatrix = RigExecBakedRunProjectorTarget(
            B, chain, revision, &derived.matrix, &step->diagnostics);
        derived.haveResult = true;
        step->counters.chainsBuilt = 1;
        return true;
    }
    RigExecMoverParameters parameters = RigExecBakedAssembleFromLeaves(
        &B, &revision, chain.result.cdata(), chain.result.size(),
        &step->diagnostics);
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
            RigExecRunRevisionKernel(revision.op, parameters, &values,
                                     B.useSimd);
        revision.resultStatus = status.state;
        if (!applied) {
            values.assign(derived.lastBase.begin(), derived.lastBase.end());
            if (status.AllowsApply()) {
                revision.resultStatus = _frozenBodyTokens->moverFailed;
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
    const RigExecOpBodyScope body(
        B.purityAudit ? &B.purityViolations.count : nullptr);
    step->BeginRun();
    if (step->kind == RigExecBakedStepKind::WeightPacket ||
        step->kind == RigExecBakedStepKind::VolumePlacements) {
        return _FrozenWeightStep(worker, step, index, inputs, time);
    }
    if (step->kind == RigExecBakedStepKind::RevisionStatic) {
        return _FrozenRevisionStatic(worker, step, index, inputs);
    }
    if (step->kind == RigExecBakedStepKind::Derived) {
        return _FrozenDerived(worker, step);
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
