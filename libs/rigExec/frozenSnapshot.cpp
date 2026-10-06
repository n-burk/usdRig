// Stage-side program snapshot construction and constant patching.

#include "frozenContextInternal.h"
#include <algorithm>
#include <cmath>
#include <set>

namespace rigExec {

using namespace frozenDetail;

namespace frozenDetail {

// Memberwise program clone. RigExecBakedProgramImpl is movable but not
// copyable (clusterCounters' unique_ptr), so the copy is explicit, field by
// field, in struct order. Live pointers are nulled -- the worker repoints
// the ones it uses at worker-owned state -- and every other field is
// copied, including the per-frame working state (the history the frozen
// run branches from) and the USD handles (copied but dead; see the audit).
// When a field is added to RigExecBakedProgramImpl, it must be added here:
// a missing epoch field silently changes the frozen run's answers, and a
// missing per-frame field silently changes its history. The bit-identity
// test is the backstop, not the discipline.
void
_CloneImpl(const RigExecBakedProgramImpl &src, RigExecBakedProgramImpl *dst)
{
    RigExecBakedProgramImpl &D = *dst;
    D.evaluator = nullptr;
    D.stage = UsdStageRefPtr();
    D.assetRootPath = src.assetRootPath;
    D.resolvedInputs = nullptr;
    D.chainSnapshots = nullptr;
    D.skinTopologies = nullptr;
    D.blendSampleShapes = nullptr;
    D.resolveBlendSample = {};
    D.profiler = nullptr;
    D.interactiveOverrides = nullptr;
    D.jointSolverBinding = nullptr;
    D.guideTaps = nullptr;
    D.solverGuidesEnabled = nullptr;
    D.hasPropertyChains = src.hasPropertyChains;
    D.paths = src.paths;
    D.index = src.index;
    D.slotKind = src.slotKind;
    D.parent = src.parent;
    D.propParent = src.propParent;
    D.xformSlots = src.xformSlots;
    D.xformPrimsBySlot = src.xformPrimsBySlot;
    D.assetRoot = src.assetRoot;
    D.xformBase = src.xformBase;
    D.lastXformBase = src.lastXformBase;
    D.constraintArrays = src.constraintArrays;
    D.nativeSources = src.nativeSources;
    D.nativeFrames = src.nativeFrames;
    D.lastNativeFrames = src.lastNativeFrames;
    D.nativeFrameOk = src.nativeFrameOk;
    D.lastNativeFrameOk = src.lastNativeFrameOk;
    D.ladders = src.ladders;
    D.ladderVarying = src.ladderVarying;
    D.ladderOverrides = src.ladderOverrides;
    D.ladderDisturbed = src.ladderDisturbed;
    D.ladderMovedSlots = src.ladderMovedSlots;
    D.ladderWatched = src.ladderWatched;
    D.ladderRecomputed = src.ladderRecomputed;
    D.restChainVaries = src.restChainVaries;
    D.posedAuthored = src.posedAuthored;
    D.posedAuthoredM = src.posedAuthoredM;
    D.restM = src.restM;
    D.restPts = src.restPts;
    D.restFrames = src.restFrames;
    D.selfD = src.selfD;
    D.parentDinv = src.parentDinv;
    D.rotOrder = src.rotOrder;
    D.restRoundTrip = src.restRoundTrip;
    D.defaultRoundTrip = src.defaultRoundTrip;
    D.lastRestM = src.lastRestM;
    D.lastSelfD = src.lastSelfD;
    D.lastParentDinv = src.lastParentDinv;
    D.lastPosedAuthoredM = src.lastPosedAuthoredM;
    D.lastPosedAuthored = src.lastPosedAuthored;
    D.lastRotOrder = src.lastRotOrder;
    D.noScaleAvars = src.noScaleAvars;
    D.rotationSign = src.rotationSign;
    D.poseInterpolators = src.poseInterpolators;
    D.poseWeightPaths = src.poseWeightPaths;
    D.poseWeights = src.poseWeights;
    D.poseWeightIndex = src.poseWeightIndex;
    D.avarConstants = src.avarConstants;
    D.avarBindings = src.avarBindings;
    D.avarConstantBindings = src.avarConstantBindings;
    D.patchableAvars = src.patchableAvars;
    D.promotedAvars = src.promotedAvars;
    D.boundInputs = src.boundInputs;
    D.varyingInputs = src.varyingInputs;
    D.avars = src.avars;
    D.posedM = src.posedM;
    D.base = src.base;
    D.fin = src.fin;
    D.finLast = src.finLast;
    D.baseLast = src.baseLast;
    D.propertyResults = src.propertyResults;
    D.frameRecords = src.frameRecords;
    D.frameMatrix = src.frameMatrix;
    D.frameMatrixValid = src.frameMatrixValid;
    D.finalMatrix = src.finalMatrix;
    D.baseMatrix = src.baseMatrix;
    D.needFinal = src.needFinal;
    D.needBase = src.needBase;
    D.solvers = src.solvers;
    D.guideSolvers = src.guideSolvers;
    D.solverIndex = src.solverIndex;
    D.aggregates = src.aggregates;
    D.constraints = src.constraints;
    D.walkSteps = src.walkSteps;
    D.steps = src.steps;
    D.composeGroups = src.composeGroups;
    // The space switches, and the per-slot index the compose step
    // asks before it takes the switched branch. Left out, the
    // frozen compose falls straight through to the namespace
    // parent -- which moves every switched control and everything
    // under it, and reads as a parity mismatch on 1509 control
    // frames of a rig that is standing in the same place.
    D.spaceSwitches = src.spaceSwitches;
    D.spaceSwitchBySlot = src.spaceSwitchBySlot;
    D.autoClavicles = src.autoClavicles;
    D.autoClavicleBySlot = src.autoClavicleBySlot;
    D.commits = src.commits;
    D.revisionIndex = src.revisionIndex;
    D.derivedIndex = src.derivedIndex;
    D.chainRevisionBegin = src.chainRevisionBegin;
    D.chainRevisionEnd = src.chainRevisionEnd;
    D.revisionChunkBase = src.revisionChunkBase;
    D.revisionChunkCount = src.revisionChunkCount;
    D.chainChunkBegin = src.chainChunkBegin;
    D.chainChunkEnd = src.chainChunkEnd;
    D.revisionFuseStep = src.revisionFuseStep;
    D.clustering = src.clustering;
    D.clusterCounters.reset();
    D.runSeqCounter.next.store(0, std::memory_order_relaxed);
    D.cones = src.cones;
    D.closedSteps = src.closedSteps;
    D.closed = src.closed;
    D.lastAvars = src.lastAvars;
    D.lastPropertyResults = src.lastPropertyResults;
    D.lastOverridden = src.lastOverridden;
    D.lastHaveBase = src.lastHaveBase;
    D.lastTime = src.lastTime;
    D.everRan = src.everRan;
    D.programStamp = src.programStamp;
    D.lastProgramStamp = src.lastProgramStamp;
    // Pending value edits travel with the stamps: a clone first run at the
    // source's lastTime -- a rewarm after a generation the live program did
    // not answer -- is the run that owes them, and time alone would not
    // dirty the steps they reached.
    D.edited = src.edited;
    D.anyEdited = src.anyEdited;
    D.valueEditSerial = src.valueEditSerial;
    D.editSerial = src.editSerial;
    D.lastClosedClusters = src.lastClosedClusters;
    D.lastClosedSteps = src.lastClosedSteps;
    D.solverOverrideRounds = src.solverOverrideRounds;
    D.solverEvaluations = src.solverEvaluations;
    D.jointSlots = src.jointSlots;
    D.jointPaths = src.jointPaths;
    D.controlSlots = src.controlSlots;
    D.controlPaths = src.controlPaths;
    D.solverArrays = src.solverArrays;
    D.jointPublishOrder = src.jointPublishOrder;
    D.controlPublishOrder = src.controlPublishOrder;
    D.solverPublishOrder = src.solverPublishOrder;
    D.jointPathsAscending = src.jointPathsAscending;
    D.controlPathsAscending = src.controlPathsAscending;
    D.solverArraysAscending = src.solverArraysAscending;
    D.timedPrologueUs = 0;
    D.timedRegionUs = 0;
    D.timedEpilogueUs = 0;
    D.timedFrames = 0;
    D.measurementSuspended = false;
    D.jointMatrixPublished = src.jointMatrixPublished;
    D.chains = src.chains;
    D.deltaValues = src.deltaValues;
    D.deltaPresent = src.deltaPresent;
    D.deltaBasePaths = src.deltaBasePaths;
    D.deltaBaseMatrix = src.deltaBaseMatrix;
    D.lastDeltaBaseMatrix = src.lastDeltaBaseMatrix;
    D.deltaBaseOk = src.deltaBaseOk;
    D.lastDeltaBaseOk = src.lastDeltaBaseOk;
    D.weightObjects = src.weightObjects;
    D.weightIndex = src.weightIndex;
    D.resolveWeights = {};
    D.volumePlacement = src.volumePlacement;
    D.placedVolumes = src.placedVolumes;
    D.currentPhaseWeights = src.currentPhaseWeights;
    D.falloffLuts = src.falloffLuts;
    // Sized but empty: every packet is rebuilt by its step during the run
    // (reads always follow writes in program order, as live), so copying
    // the previous run's dense fields per job would be pure waste.
    D.weightPackets.clear();
    D.weightPackets.resize(src.weightPackets.size());
    D.rebuild = src.rebuild;
    D.named = src.named;
    D.prims = src.prims;
    D.xformPrims = src.xformPrims;
    D.overridden = src.overridden;
    D.overridableInputs = src.overridableInputs;
    // The leaves, which the worker re-samples whole on every job.
    D.leaves = src.leaves;
    D.leafRefs = src.leafRefs;
    D.leafOfOverride = src.leafOfOverride;
    D.leafByPath = src.leafByPath;
    D.routedOverrides = src.routedOverrides;
    D.lastRoutedOverrides = src.lastRoutedOverrides;
    D.leafSamples = src.leafSamples;
    // The path leaves' bookkeeping. Their values ride the chains and weight
    // objects above; a frozen job takes its revisions' and derived targets'
    // leaves from the job's vector (RigExecFrameInputs::revisionLeaves,
    // derivedLeaves) and samples none itself.
    D.pathLeafRefs = src.pathLeafRefs;
    D.pathLeafChainResults = src.pathLeafChainResults;
    D.pathLeafChainSerial = src.pathLeafChainSerial;
    D.pathLeafSamples = src.pathLeafSamples;
    D.resolvedRoutedPrims = src.resolvedRoutedPrims;
    D.avarsDisturbed = src.avarsDisturbed;
    D.folded = src.folded;
    D.anyOverridden = src.anyOverridden;
    D.publishWeightFields = src.publishWeightFields;
}

} // namespace frozenDetail

bool
RigExecCanFreezeProgram(const RigExecRigEvaluator &evaluator,
                        std::string *error)
{
    const auto fail = [&error](const std::string &why) {
        if (error) {
            *error = why;
        }
        return false;
    };
    if (evaluator.cpuParityMode) {
        return fail("CPU parity mode runs the dynamic path, which no "
                    "snapshot can reproduce");
    }
    const RigExecBakedProgram *program = evaluator.GetBakedProgram();
    if (!program) {
        return fail("no baked program: dynamic/refusal rigs take the D7 "
                    "UI-thread memo path, never a background job");
    }
    const RigExecBakedProgramImpl &B = program->GetStepGraph();
    // Property chains are supported through the sampling hook -- except a
    // chain binding a weight object, whose envelope resolves through the
    // evaluator's live oracle. The discovery must also agree with the
    // program: a mismatch means the mover order and the epoch disagree, and
    // no snapshot is taken from a confused epoch.
    {
        RigExecChainSampleBindings bound;
        std::string bindError;
        if (!RigExecBindChainSampleInputs(evaluator, &bound, &bindError)) {
            return fail("cannot name the property chains: " + bindError);
        }
        if (bound.chains.empty() == B.hasPropertyChains) {
            return fail(
                "chain discovery disagrees with the program about whether "
                "chains exist");
        }
        for (const RigExecChainSampleChain &chain : bound.chains) {
            for (const RigExecChainSampleRevision &revision :
                 chain.revisions) {
                if (!revision.weightObjects.empty()) {
                    return fail(
                        "diag " + revision.moverPath.GetString() +
                        " binds a weight object, whose envelope the frozen "
                        "executor cannot reproduce");
                }
            }
        }
    }
    // No gate on xform slots, native sources, delta bases, or
    // geometry-domain constraints: the seeds sample per frame through the
    // program's hook, and the constraint steps are the shared bodies, so a
    // frozen run reproduces them from the transported seeds.
    // Weight objects and their steps run frozen now (sampled arrays +
    // patched scalars into the shared packet kernels); the remaining
    // oracle-dependent refusals are pose-domain constraints binding weight
    // objects, current-phase revisions, and property-chain diagnostics
    // binding weight objects, each gated where they are found. A
    // geometry-domain constraint never resolves its object -- weight stays
    // 1.0 and the revision's own weight packet scales per point
    // (bakedPose.cpp, the dynamic walk's gate alike) -- so the refusal
    // names exactly the step's oracle-call condition.
    for (const RigExecBakedProgramImpl::Constraint &constraint :
         B.constraints) {
        if (!constraint.weightObject.IsEmpty() &&
            constraint.pointsTarget.IsEmpty()) {
            return fail("constraint " + constraint.path.GetString() +
                        " binds a weight object, whose oracle resolves from "
                        "the live stage");
        }
    }
    if (B.ladderVarying) {
        return fail("a time-varying provider ladder recomposes from stage "
                    "reads the frozen executor cannot reproduce");
    }
    for (const RigExecBakedProgramImpl::GeomChain &chain : B.chains) {
        for (const RigExecBakedProgramImpl::GeomRevision &revision :
             chain.revisions) {
            if (revision.op != RigExecRevisionOp::Skin &&
                revision.op != RigExecRevisionOp::RecomputeNormals &&
                revision.op != RigExecRevisionOp::RecomputeExtent &&
                revision.op != RigExecRevisionOp::Matrix &&
                revision.op != RigExecRevisionOp::Wire &&
                revision.op != RigExecRevisionOp::BlendShape &&
                revision.op != RigExecRevisionOp::VolumeCorrect &&
                revision.op != RigExecRevisionOp::Smooth &&
                revision.op != RigExecRevisionOp::DeltaMush &&
                revision.op != RigExecRevisionOp::Wrinkle &&
                revision.op != RigExecRevisionOp::Lattice &&
                revision.op != RigExecRevisionOp::SurfaceProject &&
                revision.op != RigExecRevisionOp::Ribbon &&
                revision.op != RigExecRevisionOp::EmitGuidePoints) {
                return fail("revision " +
                            revision.moverPath.GetString() + " runs op '" +
                            RigExecRevisionKindToken(revision.op)
                                .GetString() +
                            "', which the frozen executor does not "
                            "implement");
            }
            for (const RigExecBakedProgramImpl::GeomBlendChannel &channel :
                 revision.blendChannels) {
                for (const RigExecBakedProgramImpl::GeomBlendChannel::Sample
                         &sample : channel.samples) {
                    if (!sample.phase.IsBase()) {
                        return fail(
                            "revision " +
                            revision.moverPath.GetString() +
                            " reads a blend sample through a run snapshot, "
                            "which the frozen executor does not implement");
                    }
                }
            }
            // Read phases, snapshot-recording revisions and snapshot readers
            // all run frozen: a point phase resolves through its binding
            // over the worker's own chains, an AtPrim transform phase out
            // of the worker's own FrameMatrix records, and the static
            // assembly builds the same per-revision overlay live builds.
            // (Blend-sample phases keep their own refusal above.)
            if (revision.weightCurrentPhase) {
                return fail("revision " +
                            revision.moverPath.GetString() +
                            " measures its weight field against the current "
                            "phase, which resolves through the live oracle");
            }
            // Driver frames and the geometry-delta hand-off use the
            // worker's own Solve-step aggregate, read in program order,
            // and the delta through the shared constraint step and
            // the shared fold -- the seeds patch the delta bases the
            // constraint step measures against, the step stashes the delta,
            // and FoldInfluences reads the stash, all shared bodies in
            // program order.
            if (revision.op == RigExecRevisionOp::Skin &&
                !revision.skinTopologyFixed) {
                return fail("revision " +
                            revision.moverPath.GetString() +
                            " has an unfixed skin layout, which the packet "
                            "assembly reads per frame off the stage");
            }
        }
        for (const RigExecBakedProgramImpl::GeomChain::Derived &derived :
             chain.derived) {
            if (derived.revision.op != RigExecRevisionOp::RecomputeNormals &&
                derived.revision.op != RigExecRevisionOp::RecomputeExtent &&
                !RigExecIsDerivedMatrixOp(derived.revision.op)) {
                return fail("derived target " +
                            derived.target.GetString() +
                            " runs an op the frozen executor does not "
                            "implement");
            }
            if (!derived.revision.binding.phases.empty()) {
                return fail("derived target " +
                            derived.target.GetString() +
                            " declares a read phase, which the frozen "
                            "executor does not implement");
            }
        }
    }

    return true;
}

bool
RigExecFreezeProgram(const RigExecRigEvaluator &evaluator,
                     std::shared_ptr<const RigExecFrozenProgram> *frozen,
                     std::string *error)
{
    if (!frozen) {
        if (error) {
            *error = "no snapshot to freeze into";
        }
        return false;
    }
    if (!RigExecCanFreezeProgram(evaluator, error)) {
        return false;
    }
    const RigExecBakedProgramImpl &B =
        evaluator.GetBakedProgram()->GetStepGraph();
    auto snapshot = std::make_shared<RigExecFrozenProgram>();
    _CloneImpl(B, &snapshot->program);
    if (B.jointSolverBinding) {
        snapshot->jointSolverBinding = *B.jointSolverBinding;
    }
    snapshot->guideTapsPresent =
        B.guideTaps != nullptr && B.guideTaps->get() != nullptr;
    // Handle identities, captured here (UI thread) as plain data: the
    // worker must not even ask a handle whether it is valid.
    _ForEachPatchableInput(
        B, [&snapshot](const auto &input) {
            snapshot->inputHeadPaths.push_back(
                input.head ? input.head.GetPath() : SdfPath());
        });
    for (const RigExecBakedProgramImpl::ConstraintArrays &arrays :
         B.constraintArrays) {
        const SdfPath primPath =
            arrays.prim ? arrays.prim.GetPath() : SdfPath();
        snapshot->arrayKeys.push_back(
            primPath.IsEmpty()
                ? SdfPath()
                : primPath.AppendProperty(TfToken("inputs:sourceWeights")));
        snapshot->arrayKeys.push_back(
            (primPath.IsEmpty() || !arrays.parentOffsets)
                ? SdfPath()
                : primPath.AppendProperty(
                      TfToken("inputs:translationOffsets")));
        snapshot->arrayKeys.push_back(
            (primPath.IsEmpty() || !arrays.parentOffsets)
                ? SdfPath()
                : primPath.AppendProperty(TfToken("inputs:rotationOffsets")));
        snapshot->arrayKeys.push_back(
            (primPath.IsEmpty() || !arrays.readPole)
                ? SdfPath()
                : primPath.AppendProperty(
                      TfToken("inputs:poleVectorWeights")));
    }
    for (const RigExecBakedProgramImpl::GeomChain &chain : B.chains) {
        snapshot->chainBaseQueryValid.push_back(
            chain.baseQuery.IsValid() ? 1 : 0);
    }
    for (const auto &[chainIndex, derivedIndex] : B.derivedIndex) {
        const RigExecBakedProgramImpl::GeomChain::Derived &derived =
            B.chains[size_t(chainIndex)].derived[size_t(derivedIndex)];
        snapshot->derivedBaseQueryValid.push_back(
            derived.baseQuery.IsValid() ? 1 : 0);
    }
    for (const RigExecBakedProgramImpl::Solver &solver : B.solvers) {
        snapshot->ribbonQueryValid.push_back(
            solver.ribbonPointsQuery.IsValid() ? 1 : 0);
    }
    for (const auto &[chainIndex, revisionIndex] : B.revisionIndex) {
        const RigExecBakedProgramImpl::GeomRevision &revision =
            B.chains[size_t(chainIndex)].revisions[size_t(revisionIndex)];
        const UsdPrim &moverPrim = revision.moverPrim;
        snapshot->moverHasEnabled.push_back(
            moverPrim && moverPrim.GetAttribute(TfToken("inputs:enabled"))
                ? 1
                : 0);
        snapshot->moverHasDefaultWeight.push_back(
            moverPrim &&
                    moverPrim.GetAttribute(TfToken("inputs:defaultWeight"))
                ? 1
                : 0);
        snapshot->moverHasMethod.push_back(
            moverPrim &&
                    moverPrim.GetAttribute(TfToken("rigExec:skinningMethod"))
                ? 1
                : 0);
    }
    *frozen = std::move(snapshot);
    return true;
}

bool
RigExecFrozenSnapshotOwesLiveEdits(const RigExecFrozenProgram &base,
                                   const RigExecBakedProgram &live)
{
    const RigExecBakedProgramImpl &L = live.GetStepGraph();
    return L.valueEditSerial != base.program.valueEditSerial ||
           L.programStamp != base.program.programStamp;
}

uint64_t
RigExecFrozenAvarRegionDigest(const RigExecBakedProgram &program)
{
    // Exactly the fields RigExecProgramAvarPatch writes, plus the table it
    // keeps beside them: per-binding constants and varying flags, the
    // promoted set, the varying count, and the constant table. Working
    // state (avars, lastAvars) is history, not region, and is excluded --
    // it converges on the next run whatever the snapshot holds.
    const RigExecBakedProgramImpl &B = program.GetStepGraph();
    uint64_t h = 1469598103934665603ULL;
    h = _HashBytes(h, &B.varyingInputs, sizeof(B.varyingInputs));
    for (double constant : B.avarConstants) {
        h = _HashBytes(h, &constant, sizeof(constant));
    }
    for (const RigExecBakedProgramImpl::AvarBinding &binding :
         B.avarConstantBindings) {
        h = _HashBytes(h, &binding.input.constant,
                       sizeof(binding.input.constant));
        const unsigned char varying = binding.input.varying ? 1 : 0;
        h = _HashBytes(h, &varying, sizeof(varying));
    }
    for (size_t promoted : B.promotedAvars) {
        h = _HashBytes(h, &promoted, sizeof(promoted));
    }
    return h;
}

} // namespace rigExec
