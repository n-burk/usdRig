// Worker execution, result publication, and partial-cone restoration.

#include "frozenContextInternal.h"
#include "bakedSchedule.h"
#include "generation.h"
#include "frameCacheSparsity.h"
#include "weightPackets.h"
#include <algorithm>
#include <cmath>
#include <set>

// The frozen executor.
// ISOLATION AUDIT (D3). A frozen run executes on a worker thread while the
// host may author on the UI thread, so no frozen code path may reach a
// UsdStage, UsdAttribute, UsdAttributeQuery, UsdPrim, the evaluator, its
// caches, or any live lock. The argument, per unit the run touches:
//  * The snapshot (RigExecFrozenProgram) is cloned on the UI thread. Its
//    program copy keeps the live program's USD handles COPIED but DEAD: the
//    worker copies them again (refcount operations, thread-safe) and never
//    dereferences one -- not even IsValid or GetPath, which reach composed
//    specs and prim data. Every handle identity the run needs (head paths,
//    query validity, attribute presence) is captured into the snapshot's
//    plain-data side-tables at freeze time.
//  * Constant patching (below) writes every varying input's sampled value
//    into its `constant` and nulls its head/query/resolvedAttr handles, so
//    RigExecBakedRead -- the only input route the reused step bodies take
//    (bakedPose.cpp's rd(), bakedProgramImpl.h:312) -- answers the patched
//    constant on every arm: the overridden arm's GetAttribute on a nulled
//    head is a safe no-op returning false with the value untouched, and the
//    query/resolved arms are skipped for invalid handles.
//  * Pose step bodies (RigExecBakedRunPoseStep, bakedPose.cpp:2524-3490)
//    touch no live state besides B.resolvedInputs (reference formation only
//    -- rd() never reaches it once patched) and B.resolveWeights, which is
//    called only for a constraint binding a weight object; the freeze gate
//    refuses any such constraint, and the snapshot nulls the function, so a
//    call would throw into the scheduler's fail-closed catch, never into
//    the evaluator.
//  * Geometry chunk/fuse/status bodies (bakedGeometry.cpp:2370-2718, minus
//    RevisionStatic/Derived) are pure functions of program slots plus the
//    shared kernels. RevisionStatic and Derived call the static,
//    stage-reading AssembleRevision, so the frozen run does NOT call them:
//    chain-revision packets are assembled on the UI thread by the REAL
//    RigExecAssembleSkinParameters and travel with the job, and the small
//    pure remainder of each body is replicated line-for-line below
//    (_FrozenRevisionStatic, _FrozenDerived, _AssembleDerivedPacket).
//  * RigExecBakedComputeClosure (bakedSchedule.cpp:1223) and
//    RigExecBakedSkipGeometryStep are pure program-state functions and run
//    unmodified. The region loop itself is a serial reimplementation of
//    RunStepsSerial (which is static and dispatches parallel by
//    environment); it never touches the parallel executor or its atomics.
//  * RigExecBakedPublishPose is stage-clean (audited: jointSolverBinding,
//    profiler, guide taps, resolved inputs only). The frozen run points the
//    first at the snapshot's copy, the profiler at a private disabled one
//    (its scopes take only that profiler's own uncontended mutex), the
//    guide taps at a null held privately (the bool test then skips), and
//    replays the guide publication itself from the aggregates.
//  * RigExecBakedPublishGeometry reads the stage UNCONDITIONALLY
//    (bakedGeometry.cpp:2733, B.stage->GetPrimAtPath for the adjuster
//    ladder), so it is replicated below (_FrozenPublishGeometry) for the
//    gated subset (no curvenet adjusters).
//  * Shared kernels (skin, derived, solvers, constraints) are pure per-point
//    math over worker-owned buffers, and their five WorkParallelForN launch
//    sites in moverGraph.cpp (the blend-channel sum among them, which the
//    frozen blend-shape assembler below calls) take the serial variant
//    inside a frozen run (the RigExecFrozenSerialActive hook, D4), so the
//    only thread a frozen frame ever runs on is its own.
//  * TfToken/SdfPath/VtValue copies on the worker touch only their own
//    atomic refcounts and the process-global immutable-after-load tables
//    under brief internal locks -- the same operations the live path
//    performs per frame, never the stage, and never held across evaluation.
// What the frozen run therefore reads: the immutable snapshot, the job's
// sampled vector, and its own working state. What it writes: its own
// working state and the output pose. Everything else declines.

namespace rigExec {

using namespace frozenDetail;

namespace {

// Patches every patchable input's constant from its head-keyed sample and
// nulls its handles, so the reused step bodies answer the patched constant
// on every arm of RigExecBakedRead. A varying input with no sample, or a
// sample holding a type the typed read cannot consume, declines the job:
// the snapshot and the vector describe different programs.
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
        if (walked >= snapshot.inputHeadPaths.size()) {
            ok = false;
            return;
        }
        const SdfPath &key = snapshot.inputHeadPaths[walked++];
        if (key.IsEmpty()) {
            // Invalid head: the sampler emits nothing, and live answers the
            // constant on every arm. A varying input the sampler cannot key
            // is a shape it cannot reproduce.
            if (input.varying) {
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
            if (input.varying) {
                ok = false;
            }
            return;
        }
        const RigExecSampledInput &sample = inputs.values[found->second];
        if (sample.viaChain) {
            // Stale chain output: the hook declined this rig, so the runner
            // never evaluates one.
            ok = false;
            return;
        }
        if (sample.hasValue && !_SampleHolds(sample.value, &input.constant)) {
            ok = false;
            return;
        }
        // A valueless sample is a stage read that failed at sample time;
        // live's typed read fails the same way and keeps the constant.
        input.head = UsdAttribute();
        input.query = UsdAttributeQuery();
        input.resolvedAttr = UsdAttribute();
    });
    return ok && walked == snapshot.inputHeadPaths.size();
}

// The frozen prologue: RigExecBakedProgram::Run's prologue (bakedProgram.cpp)
// plus RunInputs, RunSolverSources, the constraint-array sweep, and the
// geometry prologue, with every stage or live-state read replaced by its
// sampled value. Reads the snapshot, the job's vector, and the worker's own
// program copy; writes only the copy (and the pose's prologue counters).
// Anything it cannot reproduce declines the job.
bool
_FrozenPrologue(_FrozenWorker *worker, const RigExecFrozenProgram &snapshot,
               const RigExecFrameInputs &inputs,
               const std::map<SdfPath, size_t> &index, UsdTimeCode time,
               RigExecRigPose *pose)
{
    RigExecBakedProgramImpl &B = worker->B;
    const RigExecResolvedInputs &R = *B.resolvedInputs;
    const auto findSample = [&index, &inputs](const SdfPath &key) {
        const auto found = index.find(key);
        return found == index.end() ? nullptr : &inputs.values[found->second];
    };

    // Chains, between the two override placements, as on the live path:
    // the sampler evaluated the hook for the job's time on the UI thread,
    // and the per-target results travel with the vector. The worker
    // publishes them where the live prologue publishes the chains it ran.
    // A chainless snapshot with transported results is a stale vector from
    // another epoch and declines.
    if (!B.hasPropertyChains && !inputs.chainResults.empty()) {
        return false;
    }
    B.propertyResults.clear();
    B.resolvedInputs->Clear();
    B.runSnapshots.Clear();
    B.chainSnapshots->Clear();
    // Pre- and post-chain override placement, replicated from
    // _ApplyInteractiveOverridesToResolved: every attribute override stands
    // in the resolved inputs, and one standing on a chain result replaces
    // it. The resolved inputs are unread on the worker (patched constants),
    // but the placement keeps the private state shape-identical.
    for (const RigExecValueOverride &o : inputs.overrides) {
        if (o.attribute.IsEmpty()) {
            continue;
        }
        B.resolvedInputs->SetProperty(o.prim.AppendProperty(o.attribute),
                                      o.value);
    }
    for (const auto &[target, value] : inputs.chainResults) {
        B.propertyResults[target] = value;
        B.resolvedInputs->SetProperty(target, value);
    }
    for (const RigExecValueOverride &o : inputs.overrides) {
        if (o.attribute.IsEmpty()) {
            continue;
        }
        const SdfPath path = o.prim.AppendProperty(o.attribute);
        B.resolvedInputs->SetProperty(path, o.value);
        const auto found = B.propertyResults.find(path);
        if (found != B.propertyResults.end()) {
            found->second = o.value;
        }
    }

    // RunInputs (bakedPose.cpp:2099): ladders recompose from stage reads, so
    // a recompute declines; everything else replays samples through the real
    // RigExecBakedRead, which answers the patched constants.
    bool ladderDragged = false;
    if (B.anyOverridden) {
        for (const int ladderIndex : B.ladderOverrides) {
            if (B.overridden[size_t(ladderIndex)]) {
                ladderDragged = true;
                break;
            }
        }
    }
    B.ladderRecomputed =
        B.ladderVarying || ladderDragged || B.ladderDisturbed;
    if (B.ladderRecomputed) {
        return false;
    }
    if (!B.ladderMovedSlots.empty()) {
        B.ladderMovedSlots.clear();
    }
    for (const auto &binding : B.avarBindings) {
        B.avars[binding.slot] =
            RigExecBakedRead(binding.input, R, time, &B.overridden);
    }
    for (const size_t promoted : B.promotedAvars) {
        if (promoted >= B.avarConstantBindings.size()) {
            return false;
        }
        const auto &binding = B.avarConstantBindings[promoted];
        B.avars[binding.slot] =
            RigExecBakedRead(binding.input, R, time, &B.overridden);
    }
    if (B.anyOverridden || B.avarsDisturbed) {
        for (const auto &binding : B.avarConstantBindings) {
            B.avars[binding.slot] =
                (B.overridden[size_t(binding.input.overrideIndex)] ||
                 binding.input.varying)
                    ? RigExecBakedRead(binding.input, R, time, &B.overridden)
                    : binding.input.constant;
        }
        B.avarsDisturbed = B.anyOverridden;
    }
    for (RigExecBakedProgramImpl::PoseInterpolator &interpolator :
         B.poseInterpolators) {
        interpolator.enabledValue =
            RigExecBakedRead(interpolator.enabled, R, time, &B.overridden);
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
    for (size_t k = 0; k < B.xformSlots.size(); ++k) {
        const size_t slot = size_t(B.xformSlots[k]);
        B.xformBase[k] = inputs.stageSeeds.xformBase[k];
        B.base[slot] = inputs.stageSeeds.xformFrames[k];
        B.fin[slot] = inputs.stageSeeds.xformFrames[k];
    }
    for (size_t k = 0; k < B.deltaBasePaths.size(); ++k) {
        B.deltaBaseOk[k] = inputs.stageSeeds.deltaOk[k];
        B.deltaBaseMatrix[k] = inputs.stageSeeds.deltaBase[k];
    }
    for (size_t k = 0; k < B.nativeSources.size(); ++k) {
        B.nativeFrameOk[k] = inputs.stageSeeds.nativeOk[k];
        B.nativeFrames[k] = inputs.stageSeeds.nativeFrames[k];
    }

    // Constraint operator arrays (bakedProgram.cpp:2467): replayed from the
    // sampled raw attributes with the same cardinality validation, the same
    // neutral fills, and the same diagnostic lines as
    // _ReadConstraintSourceWeights/_ReadConstraintSourceOffsets. A missing
    // or valueless sample is an absent or unreadable attribute -- neutral,
    // exactly as live -- while a mistyped holding declines.
    for (size_t k = 0; k < B.constraintArrays.size(); ++k) {
        RigExecBakedProgramImpl::ConstraintArrays &arrays =
            B.constraintArrays[k];
        if (snapshot.arrayKeys.size() < k * 4 + 4) {
            return false;
        }
        arrays.diagnostics.clear();
        const SdfPath &weightsKey = snapshot.arrayKeys[k * 4];
        VtFloatArray authoredWeights;
        if (!weightsKey.IsEmpty()) {
            if (const RigExecSampledInput *sample = findSample(weightsKey)) {
                if (sample->hasValue) {
                    if (!sample->value.IsHolding<VtFloatArray>()) {
                        return false;
                    }
                    authoredWeights =
                        sample->value.UncheckedGet<VtFloatArray>();
                }
            }
        }
        arrays.ok = true;
        if (!authoredWeights.empty() &&
            authoredWeights.size() != arrays.sourceCount) {
            arrays.diagnostics.push_back(
                weightsKey.GetPrimPath().GetString() +
                " inputs:sourceWeights has " +
                std::to_string(authoredWeights.size()) + " entries for " +
                std::to_string(arrays.sourceCount) + " sources");
            arrays.ok = false;
        } else {
            arrays.weights.assign(arrays.sourceCount, 1.0);
            for (size_t i = 0;
                 i < authoredWeights.size() && i < arrays.weights.size();
                 ++i) {
                arrays.weights[i] = double(authoredWeights[i]);
            }
        }
        bool mistyped = false;
        const auto readOffsets =
            [&](const SdfPath &key, const char *name,
                std::vector<GfVec3d> *offsets) -> bool {
            VtVec3dArray authored;
            if (!key.IsEmpty()) {
                if (const RigExecSampledInput *sample = findSample(key)) {
                    if (sample->hasValue) {
                        if (!sample->value.IsHolding<VtVec3dArray>()) {
                            mistyped = true;
                            return false;
                        }
                        authored =
                            sample->value.UncheckedGet<VtVec3dArray>();
                    }
                }
            }
            if (!authored.empty() &&
                authored.size() != arrays.sourceCount) {
                arrays.diagnostics.push_back(
                    key.GetPrimPath().GetString() + " " + name + " has " +
                    std::to_string(authored.size()) + " entries for " +
                    std::to_string(arrays.sourceCount) + " sources");
                return false;
            }
            offsets->assign(arrays.sourceCount, GfVec3d(0));
            for (size_t i = 0;
                 i < authored.size() && i < offsets->size(); ++i) {
                (*offsets)[i] = authored[i];
            }
            return true;
        };
        if (arrays.parentOffsets) {
            arrays.ok =
                arrays.ok &&
                readOffsets(snapshot.arrayKeys[k * 4 + 1],
                            "inputs:translationOffsets",
                            &arrays.translationOffsets) &&
                readOffsets(snapshot.arrayKeys[k * 4 + 2],
                            "inputs:rotationOffsets", &arrays.rotationOffsets);
            if (mistyped) {
                // A mistyped holding, not a cardinality line: decline rather
                // than serve a half-validated table.
                return false;
            }
        } else {
            arrays.translationOffsets.assign(arrays.sourceCount, GfVec3d(0));
            arrays.rotationOffsets.assign(arrays.sourceCount, GfVec3d(0));
        }
        if (arrays.readPole) {
            arrays.poleDiagnostics.clear();
            const SdfPath &poleKey = snapshot.arrayKeys[k * 4 + 3];
            VtFloatArray authoredPole;
            if (!poleKey.IsEmpty()) {
                if (const RigExecSampledInput *sample = findSample(poleKey)) {
                    if (sample->hasValue) {
                        if (!sample->value.IsHolding<VtFloatArray>()) {
                            return false;
                        }
                        authoredPole =
                            sample->value.UncheckedGet<VtFloatArray>();
                    }
                }
            }
            arrays.poleOk = true;
            if (!authoredPole.empty() &&
                authoredPole.size() != arrays.poleCount) {
                arrays.poleDiagnostics.push_back(
                    poleKey.GetPrimPath().GetString() +
                    " inputs:poleVectorWeights has " +
                    std::to_string(authoredPole.size()) + " entries for " +
                    std::to_string(arrays.poleCount) + " sources");
                arrays.poleOk = false;
            } else {
                arrays.poleWeights.assign(arrays.poleCount, 1.0);
                for (size_t i = 0;
                     i < authoredPole.size() && i < arrays.poleWeights.size();
                     ++i) {
                    arrays.poleWeights[i] = double(authoredPole[i]);
                }
            }
        }
    }

    // Geometry prologue (bakedGeometry.cpp:1759): base reads replay from the
    // sampled queries, with the same reset/count/swap/compare sequence; the
    // topology resolve replays the transported packet's layout with the same
    // re-cut rule; blend and curvenet resolves are refused at freeze.
    const auto resetRevision =
        [](RigExecBakedProgramImpl::GeomRevision *revision) {
        revision->created = true;
        revision->ran = false;
        revision->output.clear();
        revision->currentSource = -1;
        revision->lastParameters = RigExecMoverParameters();
        revision->lastAuxPoints = VtVec3fArray();
        revision->lastStatus = RigExecMoverStatus();
    };
    for (size_t ci = 0; ci < B.chains.size(); ++ci) {
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
            if (sample->hasValue) {
                if (!sample->value.IsHolding<VtVec3fArray>()) {
                    return false;
                }
                basePoints = sample->value.UncheckedGet<VtVec3fArray>();
                haveBase = true;
            }
        }
        chain.haveBase = haveBase;
        if (!chain.haveBase) {
            for (RigExecBakedProgramImpl::GeomChain::Derived &derived :
                 chain.derived) {
                derived.haveBase = false;
            }
            continue;
        }
        if (chain.haveResult && basePoints.size() != chain.lastBase.size()) {
            chain.haveResult = false;
            chain.result = VtVec3fArray();
            chain.scheduleDirty = true;
            for (RigExecBakedProgramImpl::GeomRevision &revision :
                 chain.revisions) {
                resetRevision(&revision);
            }
        }
        for (RigExecBakedProgramImpl::GeomRevision &revision :
             chain.revisions) {
            if (revision.created) {
                ++pose->moverGraphRevisionsCreated;
                revision.created = false;
            }
        }
        if (chain.scheduleDirty && !chain.revisions.empty()) {
            ++pose->moverGraphSchedulesBuilt;
        }
        chain.scheduleDirty = false;
        chain.baseDirty = !chain.haveResult || basePoints != chain.lastBase;
        chain.lastBase = basePoints;
        for (RigExecBakedProgramImpl::GeomRevision &revision :
             chain.revisions) {
            if (revision.op == RigExecRevisionOp::CurvenetAdjuster) {
                return false;
            }
        }
        for (RigExecBakedProgramImpl::GeomChain::Derived &derived :
             chain.derived) {
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
                if (sample->hasValue) {
                    if (!sample->value.IsHolding<VtVec3fArray>()) {
                        return false;
                    }
                    derivedBase =
                        sample->value.UncheckedGet<VtVec3fArray>();
                    haveDerivedBase = true;
                }
            }
            derived.haveBase = haveDerivedBase;
            if (!derived.haveBase) {
                continue;
            }
            if (derived.haveResult &&
                derivedBase.size() != derived.lastBase.size()) {
                derived.haveResult = false;
                derived.result = VtVec3fArray();
                resetRevision(&derived.revision);
            }
            if (derived.revision.created) {
                ++pose->moverGraphRevisionsCreated;
                ++pose->moverGraphSchedulesBuilt;
                derived.revision.created = false;
            }
            derived.baseDirty =
                !derived.haveResult || derivedBase != derived.lastBase;
            derived.lastBase = derivedBase;
        }
    }
    // The topology resolve, after the bases (it reads the point count):
    // the transported packet's layout, with the prologue's re-cut rule.
    if (inputs.revisionPackets.size() != B.revisionIndex.size()) {
        return false;
    }
    for (size_t r = 0; r < B.revisionIndex.size(); ++r) {
        const auto &[chainIndex, revisionIndex] = B.revisionIndex[r];
        RigExecBakedProgramImpl::GeomRevision &revision =
            B.chains[size_t(chainIndex)].revisions[size_t(revisionIndex)];
        if (!revision.skinTopologyFixed) {
            continue;
        }
        revision.topology = inputs.revisionPackets[r].skinTopology;
        revision.topologyResolved = true;
        if (!revision.chunked ||
            revision.topology == revision.partitionTopology) {
            continue;
        }
        if (!revision.topology) {
            continue;
        }
        RigExecBakedPartitionRevision(
            &revision, revision.topology->indices.data(),
            revision.topology->indices.size(), revision.topology->elementSize,
            int(revision.chunks.size()));
        revision.partitionTopology = revision.topology;
    }
    return true;
}

// The frozen region: RigExecBakedRunSteps (bakedSchedule.cpp:1833) with the
// serial executor INLINED rather than dispatched by environment -- sources
// in program order, the real closure, skips, then the closed set serially.
// No calibration, no step timing, no trace intervals: the worker shares the
// timing statics with nothing and reports no trace. It skips by CLUSTER, not
// by the live executors' step closure: every step of a cluster that holds a
// closed step runs, which is a superset of the closure and so the same
// answer -- a clean step re-run reads what it read last run.
bool
_FrozenRunSteps(_FrozenWorker *worker,
               const std::map<SdfPath, size_t> &index,
               const RigExecFrameInputs &inputs, UsdTimeCode time)
{
    RigExecBakedProgramImpl &B = worker->B;
    for (RigExecBakedStep &step : B.steps) {
        step.startUs = step.endUs = 0;
    }
    for (RigExecBakedStep &step : B.steps) {
        if (step.isSource) {
            if (!_FrozenStepBody(worker, &step, index, inputs, time)) {
                return false;
            }
            if (!step.snapshots.IsEmpty()) {
                B.runSnapshots.Merge(std::move(step.snapshots));
            }
            if (step.bail) {
                return false;
            }
        }
    }
    RigExecBakedComputeClosure(&B, time, /*force=*/false);
    for (RigExecBakedStep &step : B.steps) {
        if (step.isSource || B.closed.Test(step.cluster)) {
            continue;
        }
        step.MarkSkipped();
        if (RigExecBakedIsGeometryStep(step.kind)) {
            RigExecBakedSkipGeometryStep(&B, &step);
        }
    }
    B.clustering.lastRunTimed = false;
    for (RigExecBakedStep &step : B.steps) {
        if (step.isSource || !B.closed.Test(step.cluster)) {
            continue;
        }
        if (!_FrozenStepBody(worker, &step, index, inputs, time)) {
            return false;
        }
        if (!step.snapshots.IsEmpty()) {
            B.runSnapshots.Merge(std::move(step.snapshots));
        }
        if (step.bail) {
            return false;
        }
    }
    return true;
}

// The production cluster runner (plan 2.0): binds the worker's rebound
// clone -- the per-job program the caller cloned and restored retained
// slots into -- plus the retained handle it was rebound from, into the
// rebind context RigExecRunSparsePlan executes. One cluster runs through
// _FrozenStepBody, the exact per-step body _FrozenRunSteps dispatches,
// with the same snapshot merge and bail handling; the program parameter
// must name the bound clone (a foreign program declines rather than run
// against state the runner does not own). File-local beside the dispatch
// it shares: _FrozenStepBody takes the worker, so no header surface can
// name this factory -- the public production surface is
// RigExecRunPartialCone, which builds its context here.
RigExecClusterRebindContext
_MakeProductionClusterRunner(
    _FrozenWorker *worker, const std::map<SdfPath, size_t> &index,
    const RigExecFrameInputs &inputs, UsdTimeCode time,
    const std::shared_ptr<const void> &retained)
{
    RigExecClusterRebindContext rebind;
    if (!worker) {
        return rebind;
    }
    rebind.program = std::shared_ptr<RigExecBakedProgramImpl>(
        &worker->B, [](RigExecBakedProgramImpl *) {});
    rebind.retained = retained;
    rebind.runCluster =
        [worker, &index, &inputs, time](RigExecBakedProgramImpl &prog,
                                       int cluster) {
            if (!worker || &prog != &worker->B) {
                return false;
            }
            RigExecBakedProgramImpl &owned = worker->B;
            for (RigExecBakedStep &step : owned.steps) {
                if (step.cluster != cluster) {
                    continue;
                }
                if (!_FrozenStepBody(worker, &step, index, inputs, time)) {
                    return false;
                }
                if (!step.snapshots.IsEmpty()) {
                    owned.runSnapshots.Merge(std::move(step.snapshots));
                }
                if (step.bail) {
                    return false;
                }
            }
            return true;
        };
    return rebind;
}

// The frozen geometry epilogue: RigExecBakedPublishGeometry
// (bakedGeometry.cpp:2719) minus the adjuster ladder, which reads the stage
// (B.stage->GetPrimAtPath, unconditionally). No adjuster revision can reach
// here (the freeze refuses the op), so the sweep -- step diagnostics in
// program order, weight fields last-writer-wins per RevisionStatic step,
// movedProperties per ChainStatus/Derived step under the haveBase gates --
// is the whole function.
void
_FrozenPublishGeometry(RigExecBakedProgramImpl &B, RigExecRigPose *pose)
{
    for (const RigExecBakedStep &step : B.steps) {
        if (!RigExecBakedIsGeometryStep(step.kind)) {
            continue;
        }
        for (const std::string &diagnostic : step.diagnostics) {
            pose->diagnostics.push_back(diagnostic);
        }
        if (step.kind == RigExecBakedStepKind::RevisionStatic) {
            const auto &[chainIndex, revisionIndex] =
                B.revisionIndex[size_t(step.object)];
            const RigExecBakedProgramImpl::GeomChain &chain =
                B.chains[size_t(chainIndex)];
            const RigExecBakedProgramImpl::GeomRevision &revision =
                chain.revisions[size_t(revisionIndex)];
            if (chain.haveBase && revision.weightFieldPublished &&
                revision.weightObject >= 0 &&
                size_t(revision.weightObject) < B.weightObjects.size()) {
                RigExecResolvedWeightField &field =
                    pose->weightFields[
                        B.weightObjects[size_t(revision.weightObject)].path];
                field.target = revision.weightFieldTarget;
                field.weights = revision.weightField;
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
                pose->movedProperties[derived.target] =
                    VtValue(derived.result);
            }
        }
    }
}

} // namespace

namespace frozenDetail {

// Defined below, beside the executor.
bool _RunFrozen(const RigExecFrozenEvalContext &context,
                const RigExecFrameInputs &inputs, RigExecRigPose *pose);

// Worker-built tokens, constructed once: TfToken(const char*) takes the
// token registry's spin lock per construction, so the worker hoists them
// the way the assemblers do rather than rebuilding them per read.
const TfToken &
_FrozenKindToken(RigExecRevisionOp op)
{
    static const TfToken normals("recomputeNormals");
    static const TfToken extent("recomputeExtent");
    return op == RigExecRevisionOp::RecomputeExtent ? extent : normals;
}

// Runs one frozen frame: clone, patch, prologue, region, epilogue. Returns
// false to decline (the caller hands the generation back); a declined run
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
    _FrozenWorker worker;
    _CloneImpl(snapshot.program, &worker.B);
    RigExecBakedProgramImpl &B = worker.B;
    B.resolvedInputs = &worker.resolved;
    B.volumeWeightMatrices = &worker.volumeWeightMatrices;
    B.chainSnapshots = &worker.chainSnapshots;
    B.profiler = &worker.profiler;
    B.interactiveOverrides = &inputs.overrides;
    B.jointSolverBinding = &snapshot.jointSolverBinding;
    B.guideTaps = &worker.nullTaps;
    B.solverGuidesEnabled = &worker.guidesEnabled;
    B.publishWeightFields =
        (context.flags & kRigExecFrozenPublishWeightFields) != 0;
    worker.guidesEnabled =
        (context.flags & kRigExecFrozenSolverGuidesEnabled) != 0;

    // Override flags for the job's overrides, exactly as SetOverrides
    // computes them; unplaceable declines (live runs dynamically).
    std::vector<char> flags;
    if (!_FrozenPlaceOverrides(B, inputs.overrides, &flags)) {
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

    std::map<SdfPath, size_t> index;
    for (size_t i = 0; i < inputs.values.size(); ++i) {
        index.emplace(inputs.values[i].path, i);
    }
    if (!_PatchInputs(B, snapshot, index, inputs)) {
        return false;
    }

    const UsdTimeCode time = inputs.time;
    RigExecRigPose working;
    if (!_FrozenPrologue(&worker, snapshot, inputs, index, time, &working)) {
        return false;
    }
    if (!_FrozenRunSteps(&worker, index, inputs, time)) {
        return false;
    }

    // Epilogue (bakedProgram.cpp:2644): chain lines first (live runs the
    // chains before everything), then the pose and geometry publication, the
    // work counters, the curvenet drain, and the summary line, in live's
    // order. No timing replay, calibration, or cone verification: the worker
    // shares those statics with nothing and reports no trace.
    for (const std::string &line : inputs.chainDiagnostics) {
        working.diagnostics.push_back(line);
    }
    if (!RigExecBakedPublishPose(&B, &working)) {
        return false;
    }
    if (snapshot.guideTapsPresent && worker.guidesEnabled) {
        // The guide block PublishPose skipped for the null taps
        // (bakedPose.cpp:3673), replayed from the aggregates.
        for (const int solverIndex : B.solverPublishOrder) {
            const auto &[solverPath, si] = B.solverArrays[size_t(solverIndex)];
            RigExecBakedEmplace(&working.solverFrames, B.solverArraysAscending,
                                solverPath, B.aggregates[size_t(si)].frames);
        }
    }
    if (B.volumeWeightMatrices) {
        working.weightFrames = *B.volumeWeightMatrices;
    }
    _FrozenPublishGeometry(B, &working);
    working.solverOverrideRounds += B.solverOverrideRounds;
    working.solverEvaluations += B.solverEvaluations;
    size_t chainsBuilt = 0, revisionsBuilt = 0;
    for (const RigExecBakedStep &step : B.steps) {
        working.moverGraphRevisionsExecuted += step.counters.revisionsExecuted;
        working.moverGraphRevisionsCreated += step.counters.revisionsCreated;
        working.moverGraphSchedulesBuilt += step.counters.schedulesBuilt;
        chainsBuilt += step.counters.chainsBuilt;
        revisionsBuilt += step.counters.revisionsBuilt;
    }
    for (std::string message : B.curvenetBindings.TakeDiagnostics()) {
        working.diagnostics.push_back(std::move(message));
    }
    working.diagnostics.push_back(
        "mover graph: " + std::to_string(chainsBuilt) + " chain(s), " +
        std::to_string(revisionsBuilt) + " revision(s); " +
        std::to_string(working.moverGraphRevisionsCreated) + " created, " +
        std::to_string(working.moverGraphRevisionsExecuted) + " executed, " +
        std::to_string(working.moverGraphSchedulesBuilt) +
        " schedule(s) built");
    working.valid = true;
    working.time = time;
    *pose = working;
    RigExecCapturePartialSlots(B, &_lastFrozenSlots, &_lastFrozenSlotBytes);
    return true;
}

} // namespace frozenDetail

void
RigExecPartialSlots::Capture(const RigExecBakedProgramImpl &program)
{
    poseWeights = program.poseWeights;
    weightPackets = program.weightPackets;
    posedM = program.posedM;
    finalMatrix = program.finalMatrix;
    baseMatrix = program.baseMatrix;
    base = program.base;
    fin = program.fin;
    aggregates = program.aggregates;
    deltaValues = program.deltaValues;
    deltaPresent = program.deltaPresent;
    solvers.resize(program.solvers.size());
    for (size_t s = 0; s < program.solvers.size(); ++s) {
        solvers[s].outFrames = program.solvers[s].outFrames;
        solvers[s].outPresent = program.solvers[s].outPresent;
        solvers[s].fallbackJoints = program.solvers[s].fallbackJoints;
    }
    commits.resize(program.commits.size());
    for (size_t c = 0; c < program.commits.size(); ++c) {
        commits[c].present = program.commits[c].present;
        commits[c].deltaOk = program.commits[c].deltaOk;
        commits[c].frames = program.commits[c].frames;
        commits[c].staged = program.commits[c].staged;
        commits[c].deltas = program.commits[c].deltas;
        commits[c].outcome = program.commits[c].outcome;
        commits[c].sources = program.commits[c].sources;
        commits[c].abandoned = program.commits[c].abandoned;
    }
    steps.resize(program.steps.size());
    for (size_t k = 0; k < program.steps.size(); ++k) {
        steps[k].diagnostics = program.steps[k].diagnostics;
        steps[k].counters = program.steps[k].counters;
        steps[k].bail = program.steps[k].bail;
    }
    volumeWeightMatrices.clear();
    if (program.volumeWeightMatrices) {
        volumeWeightMatrices = *program.volumeWeightMatrices;
    }
}

bool
RigExecPartialSlots::Restore(RigExecBakedProgramImpl *program) const
{
    if (!program) {
        return false;
    }
    RigExecBakedProgramImpl &B = *program;
    if (solvers.size() != B.solvers.size() ||
        commits.size() != B.commits.size() ||
        steps.size() != B.steps.size()) {
        return false;
    }
    B.poseWeights = poseWeights;
    B.weightPackets = weightPackets;
    B.posedM = posedM;
    B.finalMatrix = finalMatrix;
    B.baseMatrix = baseMatrix;
    B.base = base;
    B.fin = fin;
    B.aggregates = aggregates;
    B.deltaValues = deltaValues;
    B.deltaPresent = deltaPresent;
    for (size_t s = 0; s < B.solvers.size(); ++s) {
        B.solvers[s].outFrames = solvers[s].outFrames;
        B.solvers[s].outPresent = solvers[s].outPresent;
        B.solvers[s].fallbackJoints = solvers[s].fallbackJoints;
    }
    for (size_t c = 0; c < B.commits.size(); ++c) {
        B.commits[c].present = commits[c].present;
        B.commits[c].deltaOk = commits[c].deltaOk;
        B.commits[c].frames = commits[c].frames;
        B.commits[c].staged = commits[c].staged;
        B.commits[c].deltas = commits[c].deltas;
        B.commits[c].outcome = commits[c].outcome;
        B.commits[c].sources = commits[c].sources;
        B.commits[c].abandoned = commits[c].abandoned;
    }
    for (size_t k = 0; k < B.steps.size(); ++k) {
        B.steps[k].diagnostics = steps[k].diagnostics;
        B.steps[k].counters = steps[k].counters;
        B.steps[k].bail = steps[k].bail;
    }
    B.runSnapshots.Clear();
    if (B.volumeWeightMatrices) {
        *B.volumeWeightMatrices = volumeWeightMatrices;
    } else if (!volumeWeightMatrices.empty()) {
        return false;
    }
    return true;
}

size_t
RigExecPartialSlots::Bytes() const
{
    size_t total = sizeof(RigExecPartialSlots);
    total += poseWeights.size() * sizeof(float);
    for (const RigExecWeightPacket &packet : weightPackets) {
        total += sizeof(RigExecWeightPacket);
        total += packet.values.size() * sizeof(float);
        total += packet.indices.size() * sizeof(int);
    }
    total += posedM.size() * sizeof(GfMatrix4d);
    total += finalMatrix.size() * sizeof(GfMatrix4d);
    total += baseMatrix.size() * sizeof(GfMatrix4d);
    total += base.size() * sizeof(RigExecPointFrame);
    total += fin.size() * sizeof(RigExecPointFrame);
    total += aggregates.size() * sizeof(RigExecPointFrameArray);
    total += deltaValues.size() * sizeof(GfMatrix4d);
    total += deltaPresent.size() * sizeof(char);
    for (const SolverSlots &solver : solvers) {
        total += solver.outFrames.size() * sizeof(RigExecPointFrame);
        total += solver.outPresent.size() * sizeof(char);
        total += solver.fallbackJoints.size() * sizeof(SdfPath);
    }
    for (const CommitSlots &commit : commits) {
        total += commit.present.size() * sizeof(char);
        total += commit.deltaOk.size() * sizeof(char);
        total += commit.frames.size() * sizeof(RigExecPointFrame);
        total += commit.staged.size() * sizeof(RigExecPointFrame);
        total += commit.deltas.size() * sizeof(GfMatrix4d);
        total += commit.outcome.size() * sizeof(uint8_t);
        total += commit.sources.size() * sizeof(RigExecConstraintSource);
    }
    for (const StepSlots &step : steps) {
        for (const std::string &line : step.diagnostics) {
            total += line.size();
        }
        total += sizeof(RigExecBakedStepCounters) + sizeof(bool);
    }
    total += volumeWeightMatrices.size() *
             (sizeof(SdfPath) + sizeof(GfMatrix4d));
    return total;
}

bool
RigExecCapturePartialSlots(
    const RigExecBakedProgramImpl &program,
    std::shared_ptr<const void> *slotsOut, size_t *bytesOut)
{
    if (!slotsOut || !bytesOut) {
        return false;
    }
    auto slots = std::make_shared<RigExecPartialSlots>();
    slots->Capture(program);
    *bytesOut = slots->Bytes();
    *slotsOut = std::move(slots);
    return true;
}

bool
RigExecTakeLastFrozenSlots(std::shared_ptr<const void> *slotsOut,
                         size_t *bytesOut)
{
    if (!slotsOut || !bytesOut) {
        return false;
    }
    if (!_lastFrozenSlots) {
        return false;
    }
    *slotsOut = std::move(_lastFrozenSlots);
    *bytesOut = _lastFrozenSlotBytes;
    _lastFrozenSlots.reset();
    _lastFrozenSlotBytes = 0;
    return true;
}

RigExecPartialRunResult
RigExecRunPartialCone(
    const RigExecFrozenProgram &snapshot,
    const RigExecFrameInputs &freshInputs,
    const std::shared_ptr<const void> &baseSlots,
    const RigExecRigPose &basePose,
    const RigExecBakedClusterSet &planClusters, uint32_t contextFlags)
{
    RigExecPartialRunResult result;
    result.pose.time = freshInputs.time;
    result.pose.valid = false;
    if (!baseSlots) {
        return result;
    }
    const RigExecPartialSlots *slots =
        static_cast<const RigExecPartialSlots *>(baseSlots.get());
    if (!slots) {
        return result;
    }
    if (freshInputs.HasChainResolvedInputs()) {
        return result;
    }
    const size_t clusterCount =
        snapshot.program.clustering.clusters.size();
    const size_t planBits = planClusters.words.size() * 64;
    for (size_t c = clusterCount; c < planBits; ++c) {
        if ((planClusters.words[c / 64] >> (c % 64)) & 1ull) {
            return result;
        }
    }
    _FrozenWorker worker;
    _CloneImpl(snapshot.program, &worker.B);
    RigExecBakedProgramImpl &B = worker.B;
    B.resolvedInputs = &worker.resolved;
    B.volumeWeightMatrices = &worker.volumeWeightMatrices;
    B.chainSnapshots = &worker.chainSnapshots;
    B.profiler = &worker.profiler;
    B.interactiveOverrides = &freshInputs.overrides;
    B.jointSolverBinding = &snapshot.jointSolverBinding;
    B.guideTaps = &worker.nullTaps;
    B.solverGuidesEnabled = &worker.guidesEnabled;
    B.publishWeightFields =
        (contextFlags & kRigExecFrozenPublishWeightFields) != 0;
    worker.guidesEnabled =
        (contextFlags & kRigExecFrozenSolverGuidesEnabled) != 0;
    if (!slots->Restore(&B)) {
        return result;
    }
    std::vector<char> flags;
    if (!_FrozenPlaceOverrides(B, freshInputs.overrides, &flags)) {
        return result;
    }
    if (flags.size() != B.overridden.size()) {
        return result;
    }
    B.overridden = flags;
    B.anyOverridden = false;
    for (char flag : B.overridden) {
        if (flag) {
            B.anyOverridden = true;
            break;
        }
    }
    std::map<SdfPath, size_t> index;
    for (size_t i = 0; i < freshInputs.values.size(); ++i) {
        index.emplace(freshInputs.values[i].path, i);
    }
    if (!_PatchInputs(B, snapshot, index, freshInputs)) {
        return result;
    }
    const UsdTimeCode time = freshInputs.time;
    RigExecRigPose working;
    if (!_FrozenPrologue(&worker, snapshot, freshInputs, index, time,
                         &working)) {
        return result;
    }
    for (RigExecBakedStep &step : B.steps) {
        step.startUs = step.endUs = 0;
    }
    for (RigExecBakedStep &step : B.steps) {
        if (step.cluster < 0 || planClusters.Test(step.cluster)) {
            continue;
        }
        step.MarkSkipped();
        if (RigExecBakedIsGeometryStep(step.kind)) {
            RigExecBakedSkipGeometryStep(&B, &step);
        }
    }
    B.clustering.lastRunTimed = false;
    RigExecClusterRebindContext rebind = _MakeProductionClusterRunner(
        &worker, index, freshInputs, time, baseSlots);
    RigExecSparsePlan sparsePlan;
    sparsePlan.verdict = RigExecSparseVerdict::Partial;
    sparsePlan.clusters = planClusters;
    const RigExecSparseExecution execution = RigExecRunSparsePlan(
        sparsePlan, B.clustering.topologicalOrder,
        RigExecMakeClusterRunner(rebind));
    if (!execution.completed) {
        return result;
    }
    result.executedClusters = execution.executedClusters;
    for (const std::string &line : freshInputs.chainDiagnostics) {
        working.diagnostics.push_back(line);
    }
    if (!RigExecBakedPublishPose(&B, &working)) {
        return result;
    }
    if (snapshot.guideTapsPresent && worker.guidesEnabled) {
        for (const int solverIndex : B.solverPublishOrder) {
            const auto &[solverPath, si] = B.solverArrays[size_t(solverIndex)];
            RigExecBakedEmplace(&working.solverFrames, B.solverArraysAscending,
                                solverPath, B.aggregates[size_t(si)].frames);
        }
    }
    if (B.volumeWeightMatrices) {
        working.weightFrames = *B.volumeWeightMatrices;
    }
    _FrozenPublishGeometry(B, &working);
    for (const RigExecBakedStep &step : B.steps) {
        if (!RigExecBakedIsGeometryStep(step.kind)) {
            continue;
        }
        if (step.cluster >= 0 && planClusters.Test(step.cluster)) {
            continue;
        }
        if (step.kind == RigExecBakedStepKind::ChainStatus) {
            const RigExecBakedProgramImpl::GeomChain &chain =
                B.chains[size_t(step.object)];
            const auto found = basePose.movedProperties.find(chain.target);
            if (found != basePose.movedProperties.end()) {
                working.movedProperties[chain.target] = found->second;
            } else {
                working.movedProperties.erase(chain.target);
            }
        } else if (step.kind == RigExecBakedStepKind::Derived) {
            const auto &[chainIndex, derivedIndex] =
                B.derivedIndex[size_t(step.object)];
            const RigExecBakedProgramImpl::GeomChain::Derived &derived =
                B.chains[size_t(chainIndex)].derived[size_t(derivedIndex)];
            const auto found =
                basePose.movedProperties.find(derived.target);
            if (found != basePose.movedProperties.end()) {
                working.movedProperties[derived.target] = found->second;
            } else {
                working.movedProperties.erase(derived.target);
            }
        } else if (step.kind == RigExecBakedStepKind::RevisionStatic) {
            const auto &[chainIndex, revisionIndex] =
                B.revisionIndex[size_t(step.object)];
            (void)chainIndex;
            (void)revisionIndex;
            const RigExecBakedProgramImpl::GeomChain &chain =
                B.chains[size_t(chainIndex)];
            const RigExecBakedProgramImpl::GeomRevision &revision =
                chain.revisions[size_t(revisionIndex)];
            if (revision.weightObject >= 0 &&
                size_t(revision.weightObject) < B.weightObjects.size()) {
                const SdfPath &path =
                    B.weightObjects[size_t(revision.weightObject)].path;
                const auto found = basePose.weightFields.find(path);
                if (found != basePose.weightFields.end()) {
                    working.weightFields[path] = found->second;
                } else {
                    working.weightFields.erase(path);
                }
            }
        }
    }
    working.solverOverrideRounds += B.solverOverrideRounds;
    working.solverEvaluations += B.solverEvaluations;
    size_t chainsBuilt = 0, revisionsBuilt = 0;
    for (const RigExecBakedStep &step : B.steps) {
        working.moverGraphRevisionsExecuted += step.counters.revisionsExecuted;
        working.moverGraphRevisionsCreated += step.counters.revisionsCreated;
        working.moverGraphSchedulesBuilt += step.counters.schedulesBuilt;
        chainsBuilt += step.counters.chainsBuilt;
        revisionsBuilt += step.counters.revisionsBuilt;
    }
    for (std::string message : B.curvenetBindings.TakeDiagnostics()) {
        working.diagnostics.push_back(std::move(message));
    }
    working.diagnostics.push_back(
        "mover graph: " + std::to_string(chainsBuilt) + " chain(s), " +
        std::to_string(revisionsBuilt) + " revision(s); " +
        std::to_string(working.moverGraphRevisionsCreated) + " created, " +
        std::to_string(working.moverGraphRevisionsExecuted) + " executed, " +
        std::to_string(working.moverGraphSchedulesBuilt) +
        " schedule(s) built");
    working.valid = true;
    working.time = time;
    result.pose = working;
    if (!RigExecCapturePartialSlots(B, &result.slots, &result.slotBytes)) {
        result.pose.valid = false;
        result.slots.reset();
        result.slotBytes = 0;
        return result;
    }
    result.completed = true;
    return result;
}

} // namespace rigExec
