// Frozen shared-graph execution and result publication.

#include "frozenContextInternal.h"
#include "inputReplay.h"
#include "bakedSchedule.h"
#include "bakedOpGraph.h"
#include "generation.h"
#include "frameCacheSparsity.h"
#include "weightPackets.h"
#include <algorithm>
#include <atomic>
#include <cmath>
#include <cstring>
#include <set>

// Source sampling happens before dispatch. Bodies consume copied leaves and
// typed outputs through the same graph executor as live evaluation; the serial
// scope keeps nested numerical kernels on this background execution lane.

namespace rigExec {

using namespace frozenDetail;

struct RigExecFrozenWorkspace::Impl {
    std::shared_ptr<const RigExecFrozenProgram> snapshot;
    _FrozenWorker worker;
    std::atomic<bool> busy{false};
    bool bound = false;
    uint64_t epochDigest = 0;
    uint64_t programDigest = 0;
};

RigExecFrozenWorkspace::RigExecFrozenWorkspace(
    std::shared_ptr<const RigExecFrozenProgram> snapshot)
    : _impl(new Impl)
{
    _impl->snapshot = std::move(snapshot);
    _impl->worker.snapshot = _impl->snapshot.get();
    _CloneImpl(_impl->snapshot->program, &_impl->worker.B);
    _impl->worker.B.opAdapter.parallel = false;
    // _CloneImpl adopts only completed owned output/signature state.
}

RigExecFrozenWorkspace::~RigExecFrozenWorkspace() = default;

std::unique_ptr<RigExecFrozenWorkspace>
RigExecCreateFrozenWorkspace(std::shared_ptr<const RigExecFrozenProgram> snapshot)
{
    RigExecInputReplayComparisonScope replayComparison("RigExecCreateFrozenWorkspace");
    if (!snapshot) return {};
    return std::unique_ptr<RigExecFrozenWorkspace>(
        new RigExecFrozenWorkspace(std::move(snapshot)));
}

struct RigExecFrozenWorkspaceAccess {
    static _FrozenWorker *Acquire(const RigExecFrozenEvalContext &context)
    {
        auto &state = *context.workspace->_impl;
        bool idle = false;
        if (!state.busy.compare_exchange_strong(idle, true)) return nullptr;
        if (state.snapshot.get() != context.frozen ||
            (state.bound && (state.epochDigest != context.epochDigest ||
                             state.programDigest != context.programDigest))) {
            state.busy.store(false);
            return nullptr;
        }
        state.bound = true;
        state.epochDigest = context.epochDigest;
        state.programDigest = context.programDigest;
        return &state.worker;
    }
    static void Release(RigExecFrozenWorkspace *workspace, bool complete)
    {
        if (!workspace) return;
        auto &state = *workspace->_impl;
        // A declined body may have published only part of the generation.
        // The next accepted job must rebuild every output before cutoff.
        if (!complete) state.worker.B.opAdapter.everRan = false;
        state.busy.store(false);
    }
};

namespace {

// Patches every patchable input's constant from its head-keyed sample and
// nulls its handles, so the reused step bodies answer the patched constant
// on every arm of RigExecBakedRead, which the prologue's leaf sample reads.
// An input with a reader walk has no sample: the leaf sample resolves its
// walk from the head-leaf samples (RigExecBakedReadWalked), so only its
// handles are nulled. A varying input with no sample, or a sample holding a
// type the typed read cannot consume, retains its authored fallback.
// Missing declarations decline a job sampled from a different program.
bool
_PatchInputs(RigExecBakedProgramImpl &B,
             const RigExecFrozenProgram &snapshot,
             const std::map<SdfPath, size_t> &index,
             const RigExecFrameInputs &inputs)
{
    size_t walked = 0;
    bool ok = true;
    _ForEachPatchableInput(B, [&](auto &input) {
        if (!ok) {
            return;
        }
        if (walked >= snapshot.inputHeadPaths.size() ||
            walked >= snapshot.inputConstants.size()) {
            ok = false;
            return;
        }
        const SdfPath &key = snapshot.inputHeadPaths[walked];
        if (!_SampleHolds(snapshot.inputConstants[walked], &input.constant)) {
            ok = false;
            return;
        }
        ++walked;
        if (input.walk >= 0) {
            input.head = UsdAttribute();
            input.query = UsdAttributeQuery();
            input.resolvedAttr = UsdAttribute();
            return;
        }
        if (key.IsEmpty()) {
            // Invalid head: the sampler emits nothing, and live answers the
            // constant on every arm. A varying input the sampler cannot key
            // is a shape it cannot reproduce.
            if (input.varying || input.sourceBacked) {
                ok = false;
            }
            return;
        }
        const auto found = index.find(key);
        if (found == index.end()) {
            // No sample: sound only when the input is not varying (live
            // reads the constant). A varying input with no sample means the
            // vector was sampled from a different program than the snapshot
            // (stale snapshot) -- decline.
            if (input.varying || input.sourceBacked) {
                ok = false;
            }
            return;
        }
        const RigExecSampledInput &sample = inputs.values[found->second];
        if (sample.hasValue && !_SampleHolds(sample.value, &input.constant)) {
            ok = false; return;
        }
        // A valueless sample is a stage read that failed at sample time;
        // live's typed read fails the same way and keeps the constant.
        input.head = UsdAttribute();
        input.query = UsdAttributeQuery();
        input.resolvedAttr = UsdAttribute();
    }, /*includeIntervening=*/false);
    return ok && walked == snapshot.inputHeadPaths.size();
}

// Writes every head leaf the sampler keyed (RigExecForEachHeadLeaf) from
// the job's vector -- a constant leaf from the shared table, a varying one
// from its sample -- setting its `changed` byte against the value the
// snapshot last held, so the common graph selects ops whose inputs
// differ from the snapshot's. Every other leaf holds no value and is
// unchanged. A missing sample or table entry, or one of another type,
// declines: the vector was sampled for another program. No stage, no path
// construction: the keys were built at Build.
bool
_PatchHeadLeaves(RigExecBakedProgramImpl &B,
                 const std::map<SdfPath, size_t> &index,
                 const RigExecFrameInputs &inputs)
{
    for (RigExecBakedHeadLeaf &leaf : B.headLeaves) {
        leaf.changed = 0;
        leaf.mustSample = 0;
    }
    const RigExecHeadLeafConstants *constants =
        inputs.headLeafConstants.get();
    bool ok = true;
    size_t j = 0;
    RigExecForEachHeadLeaf(B, [&](RigExecBakedHeadLeaf &leaf) {
        if (!ok) {
            return;
        }
        const size_t at = j++;
        if (!constants || at >= constants->keys.size() ||
            constants->keys[at] != leaf.frozenKey) {
            ok = false;
            return;
        }
        const VtValue *held = nullptr;
        // A constant leaf at a path the job's upstream layer holds rides a
        // sample too (rule 2's upstream arm), which wins over the table.
        const auto found = index.find(leaf.frozenKey);
        if (!constants->varying[at] && found == index.end()) {
            held = &constants->values[at];
            if (held->IsEmpty()) {
                held = nullptr;
            }
        } else {
            if (found == index.end()) {
                ok = false;
                return;
            }
            const RigExecSampledInput &sample = inputs.values[found->second];
            if (sample.hasValue) {
                held = &sample.value;
            }
        }
        VtValue value;
        if (held) {
            if (!RigExecBakedHeadLeafHolds(leaf, *held)) {
                ok = false;
                return;
            }
            value = *held;
        }
        leaf.changed = RigExecBakedHeadValueSame(value, leaf.value) ? 0 : 1;
        leaf.value = std::move(value);
    });
    return ok;
}

// One revision's or derived target's path leaves from the job's vector,
// answered where the live prologue's reads go through the worker's own
// source facts. Walk and property resolution belongs to the consuming body.
void
_PatchPathLeaves(RigExecBakedProgramImpl *B,
                 RigExecBakedPathLeaves *leaves,
                 const std::vector<VtValue> &values)
{
    for (size_t k = 0; k < values.size(); ++k) {
        RigExecBakedSetPathLeaf(leaves, k, values[k], B->pathLeafRun);
    }
}

// Places sampled source facts in private leaf tables. Derived validation,
// arithmetic, and property resolution happen in their declared graph bodies.
bool
_FrozenPrologue(_FrozenWorker *worker, const RigExecFrozenProgram &snapshot,
               const RigExecFrameInputs &inputs,
               const std::map<SdfPath, size_t> &index, UsdTimeCode time,
               RigExecRigPose *pose)
{
    RigExecBakedProgramImpl &B = worker->B;
    const auto findSample = [&index, &inputs](const SdfPath &key) {
        const auto found = index.find(key);
        return found == index.end() ? nullptr : &inputs.values[found->second];
    };

    B.oraclePublications=inputs.oraclePublications;
    B.oracleWeightInputs=inputs.oracleWeightInputs;
    B.propertyResults.clear();
    B.resolvedInputs->Clear();
    // The override placement, replicated from
    // _ApplyInteractiveOverridesToResolved: every attribute override stands
    // in the resolved inputs, and a chain result at a dragged target takes
    // its place there, the drag being the base it was computed from. The
    // resolved inputs are unread by the patched bindings, but the placement
    // keeps the private state shape-identical. The paths were built on the
    // UI thread (RigExecFrameInputs::overridePaths).
    if (inputs.overridePaths.size() != inputs.overrides.size()) {
        return false;
    }
    for (size_t i = 0; i < inputs.overrides.size(); ++i) {
        const RigExecValueOverride &o = inputs.overrides[i];
        if (o.attribute.IsEmpty()) {
            continue;
        }
        B.resolvedInputs->SetProperty(inputs.overridePaths[i], o.value);
    }
    // Place property source leaves after explicit overrides. Operations and
    // publication remain in the common graph and its joined epilogue.
    for (auto &object : B.weightObjects) {
        auto &leaves = object.oracleLeaves;
        for (size_t k=0;k<leaves.decl.keys.size();++k) {
            const auto found = index.find(object.oracleFrozenKeys[k]);
            VtValue value = found != index.end() && inputs.values[found->second].hasValue
                ? inputs.values[found->second].value : VtValue();
            leaves.changed[k] =
                RigExecBakedSetPathLeaf(&leaves, k, std::move(value), B.pathLeafRun);
        }
    }
    if (B.hasPropertyChains || !B.weightFields.empty() || !B.headLeaves.empty()) {
        if (!_PatchHeadLeaves(B, index, inputs)) {
            return false;
        }

    }

    // Place every sampled leaf. Bound walks stay unresolved until their body runs.
    if (inputs.stageSeeds.intervening.size() != B.interveningSlots.size() ||
        inputs.stageSeeds.interveningReset.size() != B.interveningSlots.size()) return false;
    for(size_t k=0;k<B.providerFrozenKeys.size();++k) {
        const auto found=index.find(B.providerFrozenKeys[k]);
        const VtValue value=found!=index.end() && inputs.values[found->second].hasValue
            ? inputs.values[found->second].value : VtValue();
        const bool blocked=found!=index.end() && inputs.values[found->second].valueBlocked;
        B.providerLeaves.changed[k]=!RigExecBakedHeadValueSame(value,B.providerLeaves.values[k]) ||
            bool(B.providerLeafBlocked[k])!=blocked;
        if(B.providerLeaves.changed[k]) RigExecBakedNoteSpaceLeafSampled(&B,k);
        B.providerLeaves.values[k]=value;
        B.providerLeafBlocked[k]=blocked;
    }
    RigExecBakedSampleLeaves(&B, time, /* all = */ true);
    // This source has no attribute head. Publish the dedicated stage seed
    // into its numbered pool after ordinary sampling resets changed flags.
    for (size_t k = 0; k < B.interveningSlots.size(); ++k) {
        auto &ladder = B.ladders[size_t(B.interveningSlots[k])];
        auto &input = ladder.interveningSpace;
        auto &pool = B.leaves.Of<GfMatrix4d>();
        if (input.leaf < 0 || size_t(input.leaf) >= pool.value.size() ||
            size_t(input.leaf) >= pool.changed.size()) return false;
        const size_t leaf = size_t(input.leaf);
        input.constant = inputs.stageSeeds.intervening[k];
        pool.changed[leaf] = std::memcmp(pool.value[leaf].GetArray(),
            input.constant.GetArray(), 16 * sizeof(double)) != 0;
        pool.value[leaf] = input.constant;
        ladder.interveningReset = inputs.stageSeeds.interveningReset[k];
    }


    // RunSolverSources (bakedPose.cpp:2156): ribbon points replay from the
    // sampled attribute, with the same swap-and-compare.
    for (size_t si = 0; si < B.solvers.size(); ++si) {
        RigExecBakedProgramImpl::Solver &solver = B.solvers[si];
        if (!solver.ribbonPointsVarying ||
            si >= snapshot.ribbonQueryValid.size() ||
            !snapshot.ribbonQueryValid[si]) {
            continue;
        }
        const RigExecSampledInput *sample =
            findSample(solver.ribbonPointsPath);
        if (!sample) {
            return false;
        }
        VtVec3fArray live;
        if (sample->hasValue) {
            if (!sample->value.IsHolding<VtVec3fArray>()) {
                return false;
            }
            live = sample->value.UncheckedGet<VtVec3fArray>();
        }
        solver.lastRibbonPoints.swap(solver.ribbonPoints);
        solver.ribbonPoints.assign(live.begin(), live.end());
        solver.ribbonPointsDirty =
            solver.ribbonPoints != solver.lastRibbonPoints;
    }

    // The stage-frame prologue (bakedProgram.cpp: Run's stageFrames): the
    // sampler read the same slots and paths at the job's time through the
    // program's hook, and the worker publishes the seeds where the live
    // prologue publishes its stage reads -- the bases and the slots' FIRST
    // versions for xform slots, the ok/matrix pairs for delta bases, the
    // ok/frame pairs for native sources. The vectors parallel the program
    // tables; a size mismatch is a vector from another epoch and declines.
    // The cone compares below are the shared ones, against the frozen
    // lasts, so a seed that moved since freeze dirties what live would
    // dirty against the run before -- possibly more, never less -- and the
    // steps are pure functions of these inputs, so freeze-time memos serve
    // wherever the seeds still match freeze.
    if (inputs.stageSeeds.xformBase.size() != B.xformSlots.size() ||
        inputs.stageSeeds.xformFrames.size() != B.xformSlots.size() ||
        inputs.stageSeeds.deltaOk.size() != B.deltaBasePaths.size() ||
        inputs.stageSeeds.deltaBase.size() != B.deltaBasePaths.size() ||
        inputs.stageSeeds.nativeOk.size() != B.nativeSources.size() ||
        inputs.stageSeeds.nativeFrames.size() != B.nativeSources.size()) {
        return false;
    }
    const auto &admission = inputs.stageSeeds.requiredStageFramesAdmission;
    if ((admission.admitted && admission.firstBadTarget != -1) ||
        (!admission.admitted && (admission.firstBadTarget < 0 ||
          size_t(admission.firstBadTarget) >= B.xformSlots.size()))) return false;
    B.requiredStageFramesAdmission = admission;
    for (size_t k = 0; k < B.xformSlots.size(); ++k) {
        if (!admission.admitted && k >= size_t(admission.firstBadTarget)) break;
        const size_t slot = size_t(B.xformSlots[k]);
        B.xformBase[k] = inputs.stageSeeds.xformBase[k];
        B.base[slot] = inputs.stageSeeds.xformFrames[k];
        B.fin[slot] = inputs.stageSeeds.xformFrames[k];
    }
    for (size_t k = 0; admission.admitted && k < B.deltaBasePaths.size(); ++k) {
        B.deltaBaseOk[k] = inputs.stageSeeds.deltaOk[k];
        B.deltaBaseMatrix[k] = inputs.stageSeeds.deltaBase[k];
    }
    for (size_t k = 0; admission.admitted && k < B.nativeSources.size(); ++k) {
        B.nativeFrameOk[k] = inputs.stageSeeds.nativeOk[k];
        B.nativeFrames[k] = inputs.stageSeeds.nativeFrames[k];
    }

    // Raw operator arrays are sampled sources. The owning Constraint body
    // validates cardinalities and expands neutral defaults when it runs.
    if (snapshot.arrayKeys.size() < B.constraintArrays.size() * 4) return false;
    for (size_t row = 0; admission.admitted && row < B.constraintArrays.size(); ++row) {
        for (size_t channel = 0; channel < 4; ++channel) {
            const SdfPath &key = snapshot.arrayKeys[row * 4 + channel];
            // An empty key denotes an unbound channel, not a source named
            // by an empty path. Keep its captured raw opinion unchanged.
            if (key.IsEmpty()) continue;
            const auto *sample = findSample(key);
            B.constraintArrays[row].raw[channel] = sample && sample->hasValue
                ? sample->value : VtValue();
        }
    }

    // Copy only admitted geometry source samples, matching live prologue;
    // ChainInputs and Derived own adoption. Layout-source prep stays independent.
    for (size_t ci = 0; admission.admitted && ci < B.chains.size(); ++ci) {
        RigExecBakedProgramImpl::GeomChain &chain = B.chains[ci];
        if (ci >= snapshot.chainBaseQueryValid.size()) {
            return false;
        }
        VtVec3fArray basePoints;
        bool haveBase = false;
        if (snapshot.chainBaseQueryValid[ci]) {
            const RigExecSampledInput *sample = findSample(chain.target);
            if (!sample) {
                return false;
            }
            if (sample->hasValue && sample->value.IsHolding<VtVec3fArray>()) {
                basePoints = sample->value.UncheckedGet<VtVec3fArray>();
                haveBase = true;
            }
        }
        chain.sampledHaveBase = haveBase;
        chain.sampledBase = basePoints;
        for (RigExecBakedProgramImpl::GeomChain::Derived &derived :
             chain.derived) {
            if (derived.matrixTarget) {
                // As the baked prologue: no base, evaluated every run.
                derived.sampledHaveBase = true;
                continue;
            }
            // Derived bases key by target like chain bases; validity rides
            // the derivedIndex-parallel side-table.
            const RigExecSampledInput *sample = findSample(derived.target);
            VtVec3fArray derivedBase;
            bool haveDerivedBase = false;
            // The side-table index: position of (ci, di) in derivedIndex.
            size_t derivedKey = B.derivedIndex.size();
            for (size_t dk = 0; dk < B.derivedIndex.size(); ++dk) {
                if (B.derivedIndex[dk].first == int(ci) &&
                    &B.chains[size_t(B.derivedIndex[dk].first)]
                             .derived[size_t(B.derivedIndex[dk].second)] ==
                        &derived) {
                    derivedKey = dk;
                    break;
                }
            }
            if (derivedKey >= snapshot.derivedBaseQueryValid.size()) {
                return false;
            }
            if (snapshot.derivedBaseQueryValid[derivedKey]) {
                if (!sample) {
                    return false;
                }
                if (sample->hasValue && sample->value.IsHolding<VtVec3fArray>()) {
                    derivedBase =
                        sample->value.UncheckedGet<VtVec3fArray>();
                    haveDerivedBase = true;
                }
            }
            derived.sampledHaveBase = haveDerivedBase;
            derived.sampledBase = derivedBase;
        }
    }
    // Copy all SkinTopology source leaves. The owning operation prepares
    // the current layout when its exact inputs change.
    const size_t layouts = B.revisionIndex.size() + B.derivedIndex.size();
    if (inputs.layoutSourcePaths.size() != inputs.layoutLeaves.size())
        return false;
    size_t auxiliary = layouts;
    for (size_t r = 0; r < layouts; ++r) {
        const auto &revision = *RigExecBakedLayoutRevision(&B, r);
        const auto &owners = r < B.revisionIndex.size() ?
            inputs.revisionLeaves : inputs.derivedLeaves;
        const size_t owner = r < B.revisionIndex.size() ? r :
            r - B.revisionIndex.size();
        size_t k = 0;
        for (const auto &channel : revision.blendChannels)
            for (const auto &sample : channel.samples) {
                if (sample.blendShape.IsEmpty()) continue;
                const int ids[] = {sample.offsetsLeaf, sample.indicesLeaf};
                const RigExecRevisionLeafType types[] = {
                    RigExecRevisionLeafType::Vec3fArray,
                    RigExecRevisionLeafType::IntArray};
                for (size_t i = 0; i < 2; ++i, ++k) {
                    if (ids[i] < 0 || size_t(ids[i]) >= revision.leaves.decl.keys.size() ||
                        owner >= owners.size() || size_t(ids[i]) >= owners[owner].size() ||
                        auxiliary >= inputs.layoutLeaves.size() ||
                        k >= inputs.layoutLeaves[auxiliary].size() ||
                        k >= inputs.layoutSourcePaths[auxiliary].size()) return false;
                    const auto &key = revision.leaves.decl.keys[size_t(ids[i])];
                    if (key.flavour != RigExecRevisionLeafFlavour::Raw ||
                        key.time != RigExecRevisionLeafTime::AtDefault ||
                        key.type != types[i] || key.path.IsEmpty() ||
                        inputs.layoutSourcePaths[auxiliary][k] != key.path ||
                        !RigExecBakedHeadValueSame(inputs.layoutLeaves[auxiliary][k],
                                                  owners[owner][size_t(ids[i])])) return false;
                }
            }
        if (k) {
            if (inputs.layoutLeaves[auxiliary].size() != k ||
                inputs.layoutSourcePaths[auxiliary].size() != k) return false;
            ++auxiliary;
        }
    }
    if (inputs.layoutLeaves.size() != auxiliary) return false;
    for (size_t r = 0; r < layouts; ++r) {
        RigExecBakedProgramImpl::GeomRevision &revision =
            *RigExecBakedLayoutRevision(&B, r);
        if (revision.op != RigExecRevisionOp::Skin) {
            continue;
        }
        const std::vector<VtValue> &values = inputs.layoutLeaves[r];
        RigExecBakedPathLeaves &leaves = revision.layoutLeaves;
        revision.layoutFixedChanged = false;
        std::fill(leaves.changed.begin(), leaves.changed.end(), 0);
        if (values.size() != leaves.values.size() ||
            values.size() != leaves.changed.size()) {
            return false;
        }
        for (size_t k = 0; k < values.size(); ++k) {
            leaves.changed[k] =
                RigExecBakedSetPathLeaf(&leaves, k, values[k], B.pathLeafRun) ? 1 : 0;
        }
    }
    // Raw revision leaves are copied into the worker's private pools.
    // Sparse layout normalization belongs to the selected assembly body.
    if (inputs.revisionLeaves.size() != B.revisionIndex.size()) {
        return false;
    }
    for (size_t r = 0; r < B.revisionIndex.size(); ++r) {
        const auto &[chainIndex, revisionIndex] = B.revisionIndex[r];
        RigExecBakedProgramImpl::GeomRevision &revision =
            B.chains[size_t(chainIndex)].revisions[size_t(revisionIndex)];
        if (!revision.leaves.decl.assembles) {
            continue;
        }
        const std::vector<VtValue> &values = inputs.revisionLeaves[r];
        if (values.size() != revision.leaves.decl.keys.size()) {
            return false;
        }
        _PatchPathLeaves(&B, &revision.leaves, values);
    }
    // Every derived target's leaves (normals, extent, a projector's matrix
    // targets), which hold no Present key.
    if (inputs.derivedLeaves.size() != B.derivedIndex.size()) {
        return false;
    }
    for (size_t d = 0; d < B.derivedIndex.size(); ++d) {
        const auto &[chainIndex, derivedIndex] = B.derivedIndex[d];
        RigExecBakedProgramImpl::GeomRevision &revision =
            B.chains[size_t(chainIndex)]
                .derived[size_t(derivedIndex)]
                .revision;
        if (!revision.leaves.decl.assembles) {
            continue;
        }
        if (inputs.derivedLeaves[d].size() !=
            revision.leaves.decl.keys.size()) {
            return false;
        }
        _PatchPathLeaves(&B, &revision.leaves, inputs.derivedLeaves[d]);
    }
    return true;
}

// The frozen backend uses the same operation graph and cutoff loop.
bool
_FrozenExecuteGraph(_FrozenWorker *worker, UsdTimeCode time)
{
    return RigExecBakedExecuteOpGraph(&worker->B,time,false);
}

// The frozen geometry epilogue follows RigExecBakedPublishGeometry's order:
// step diagnostics, weight fields, then moved properties under haveBase gates.
void
_FrozenPublishGeometry(RigExecBakedProgramImpl &B, RigExecRigPose *pose)
{
    RigExecBakedEnsureEpilogueIndex(&B);
    RigExecBakedAppendStepLines(&B, RigExecBakedStepLines::Geometry,
                                &pose->diagnostics);
    for (const uint32_t index : B.epilogue.geometrySteps) {
        const RigExecBakedStep &step = B.steps[index];
        if (step.kind == RigExecBakedStepKind::RevisionStatic) {
            const auto &[chainIndex, revisionIndex] =
                B.revisionIndex[size_t(step.object)];
            const RigExecBakedProgramImpl::GeomChain &chain =
                B.chains[size_t(chainIndex)];
            const RigExecBakedProgramImpl::GeomRevision &revision =
                chain.revisions[size_t(revisionIndex)];
            if (B.publishWeightFields && chain.haveBase && revision.weightFieldPublished &&
                revision.weightObject >= 0 &&
                size_t(revision.weightObject) < B.weightObjects.size()) {
                RigExecResolvedWeightField &field =
                    pose->weightFields[
                        B.weightObjects[size_t(revision.weightObject)].path];
                field.target = revision.weightFieldTarget;
                // Shared, never copied, as live publishes it.
                field.weights = revision.publishedWeightValues;
            }
        } else if (step.kind == RigExecBakedStepKind::ChainStatus) {
            RigExecBakedProgramImpl::GeomChain &chain =
                B.chains[size_t(step.object)];
            if (chain.haveBase) {
                pose->movedProperties[chain.target] = VtValue(chain.result);
            }
        } else if (step.kind == RigExecBakedStepKind::Derived) {
            const auto &[chainIndex, derivedIndex] =
                B.derivedIndex[size_t(step.object)];
            const RigExecBakedProgramImpl::GeomChain &chain =
                B.chains[size_t(chainIndex)];
            const RigExecBakedProgramImpl::GeomChain::Derived &derived =
                chain.derived[size_t(derivedIndex)];
            if (chain.haveBase && derived.haveBase) {
                if (!derived.matrixTarget) {
                    pose->movedProperties[derived.target] =
                        VtValue(derived.result);
                } else if (derived.haveMatrix) {
                    pose->movedProperties[derived.target] =
                        VtValue(derived.matrix);
                }
            }
        }
    }
}

} // namespace

namespace frozenDetail {

void
_FrozenPlaceUpstream(RigExecBakedProgramImpl *program,
                     const RigExecFrameInputs &inputs)
{
    RigExecBakedProgramImpl &B = *program;
    B.upstream.clear();
    for (const RigExecUpstreamValue &value : inputs.upstream) {
        B.upstream[value.path] = value.value;
    }
    RigExecBakedPlaceUpstream(&B, /*placeOracle=*/false);
}

// Defined below, beside the executor.
bool _RunFrozen(const RigExecFrozenEvalContext &context,
                const RigExecFrameInputs &inputs, RigExecRigPose *pose);

// Runs one frozen frame: acquire, sample placement, common graph, publication.
// False denotes incompatible transport or fatal execution; a declined run
// leaves the pose invalid and publishes nothing.
bool
_RunFrozen(const RigExecFrozenEvalContext &context,
           const RigExecFrameInputs &inputs, RigExecRigPose *pose)
{
    if (!pose || !context.frozen) {
        return false;
    }
    const RigExecFrozenProgram &snapshot = *context.frozen;
    if (inputs.HasChainResolvedInputs()) {
        return false;
    }
    _FrozenWorker ephemeral;
    _FrozenWorker *lane = nullptr;
    if (context.workspace) {
        lane = RigExecFrozenWorkspaceAccess::Acquire(context);
        if (!lane) return false;
    } else {
        ephemeral.snapshot = &snapshot;
        _CloneImpl(snapshot.program, &ephemeral.B);
        ephemeral.B.opAdapter.parallel = false;
        // The same completed-state guard applies to ephemeral workers.
        lane = &ephemeral;
    }
    struct LaneRelease {
        RigExecFrozenWorkspace *workspace;
        bool complete = false;
        ~LaneRelease() { RigExecFrozenWorkspaceAccess::Release(workspace, complete); }
    } release{context.workspace};
    _FrozenWorker &worker = *lane;
    RigExecBakedProgramImpl &B = worker.B;
    B.resolvedInputs = &worker.resolved;
    B.profiler = &worker.profiler;
    B.interactiveOverrides = &inputs.overrides;
    B.jointSolverBinding = &snapshot.jointSolverBinding;
    B.solverGuidesEnabled = &worker.guidesEnabled;
    B.publishWeightFields =
        (context.flags & kRigExecFrozenPublishWeightFields) != 0;
    worker.guidesEnabled =
        (context.flags & kRigExecFrozenSolverGuidesEnabled) != 0;

    // Override flags for the job's overrides, exactly as SetOverrides
    // computes them; unplaceable declines incompatible transport.
    std::vector<char> flags;
    if (!_FrozenPlaceOverrides(B, inputs.overrides, inputs.overridePaths,
                               &flags)) {
        return false;
    }
    if (flags.size() != B.overridden.size()) {
        return false;
    }
    B.overridden = flags;
    B.anyOverridden = false;
    for (char flag : B.overridden) {
        if (flag) {
            B.anyOverridden = true;
            break;
        }
    }
    // The job's upstream table against the snapshot's (rule 2's upstream
    // arm): the clone's slots and outputs hold the snapshot's values.
    _FrozenPlaceUpstream(&B, inputs);

    std::map<SdfPath, size_t> index;
    for (size_t i = 0; i < inputs.values.size(); ++i) {
        index.emplace(inputs.values[i].path, i);
    }
    if (!_PatchInputs(B, snapshot, index, inputs)) {
        return false;
    }

    const UsdTimeCode time = inputs.time;
    RigExecRigPose working;
    RigExecBakedClearRunStamps(&B);
    if (!_FrozenPrologue(&worker, snapshot, inputs, index, time, &working)) {
        return false;
    }
    if (!_FrozenExecuteGraph(&worker, time)) {
        return false;
    }

    if (!B.requiredStageFramesAdmission.admitted) {
        RigExecBakedPublishStageFramesRefusal(&B, &working);
        working.time = time;
        *pose = std::move(working);
        return false;
    }
    // Publish only after the shared graph has joined.
    if (!RigExecBakedPublishPose(&B, &working)) {
        return false;
    }
    if (snapshot.solverGuidesPresent && worker.guidesEnabled) {
        // The guide block PublishPose skipped for the null taps
        // (bakedPose.cpp:3673), replayed from the aggregates.
        for (const int solverIndex : B.solverPublishOrder) {
            const auto &[solverPath, si] = B.solverArrays[size_t(solverIndex)];
            RigExecBakedEmplace(&working.solverFrames, B.solverArraysAscending,
                                solverPath, B.aggregates[size_t(si)].frames);
        }
    }
    // Live's key set (placedVolumes). A placement step the closure skipped
    // kept the clone's placement; its inputs are unchanged.
    RigExecBakedPublishVolumePlacements(B, &working.weightFrames);
    _FrozenPublishGeometry(B, &working);
    RigExecBakedAppendCycleDiagnostics(B,&working);
    const bool scalarReferenceValid=RigExecBakedRunScalarReference(&B,time,&working);
    working.executedOpCount=B.opExecution.executed;
    working.valid = scalarReferenceValid;
    working.time = time;
    *pose = std::move(working);
    if (RigExecFrozenRunReport *report = _frozenRunReport) {
        report->region = RigExecBakedLastRunTrace(B);
        report->ran = true;
        report->sourceKeysBuilt = B.sourceKeysBuilt;
        report->sourceKeyMismatches = B.sourceKeyMismatches;
    }
    release.complete = true;
    return true;
}

} // namespace frozenDetail

} // namespace rigExec
