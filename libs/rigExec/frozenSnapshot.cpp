// Stage-side program snapshot construction and constant patching.

#include "frozenContextInternal.h"
#include "inputReplay.h"
#include "bakedOpValues.h"
#include <algorithm>
#include <cmath>
#include <set>

namespace rigExec {

using namespace frozenDetail;

namespace frozenDetail {

// Memberwise program clone. The copy is explicit, field by field, in struct
// order. Live pointers are nulled -- the worker repoints
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
    D.profiler = nullptr;
    D.recordOpTimings = false;
    D.interactiveOverrides = nullptr;
    D.jointSolverBinding = nullptr;
    D.ownedJointSolverBinding = src.ownedJointSolverBinding;
    D.cycleExclusionProof = src.cycleExclusionProof;
    D.cycleSkipReasons = src.cycleSkipReasons;
    D.solverGuidesEnabled = nullptr;
    D.hasPropertyChains = src.hasPropertyChains;
    D.paths = src.paths;
    D.pathTexts = src.pathTexts;
    D.index = src.index;
    D.slotKind = src.slotKind;
    D.providerActive = src.providerActive;
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
    D.interveningSlots = src.interveningSlots;
    D.interveningAnchors = src.interveningAnchors;
    D.ladderVarying = src.ladderVarying;
    D.ladderOverrides = src.ladderOverrides;
    D.restChainVaries = src.restChainVaries;
    D.posedAuthored = src.posedAuthored;
    D.posedAuthoredM = src.posedAuthoredM;
    D.restM = src.restM;
    D.restPts = src.restPts;
    D.restFrames = src.restFrames;
    D.selfD = src.selfD;
    D.posedD = src.posedD;
    D.parentSpaceM = src.parentSpaceM;
    D.parentSpaceAuthored = src.parentSpaceAuthored;
    D.lastPosedD = src.lastPosedD;
    D.lastParentSpaceM = src.lastParentSpaceM;
    D.lastParentSpaceAuthored = src.lastParentSpaceAuthored;
    D.lastRotationSign = src.lastRotationSign;
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
    D.restChanged = src.restChanged;
    D.ladderChanged = src.ladderChanged;
    D.restMoved = src.restMoved;
    D.ladderMoved = src.ladderMoved;
    D.xyzToken = src.xyzToken;
    D.noScaleAvars = src.noScaleAvars;
    D.rotationSign = src.rotationSign;
    D.poseInterpolators = src.poseInterpolators;
    D.poseWeightPaths = src.poseWeightPaths;
    D.poseWeights = src.poseWeights;
    D.poseWeightIndex = src.poseWeightIndex;
    D.avarConstants = src.avarConstants;
    D.avarBindings = src.avarBindings;
    D.avarConstantBindings = src.avarConstantBindings;
    D.avarBindingBegin = src.avarBindingBegin;
    D.avarConstantBindingBegin = src.avarConstantBindingBegin;
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
    // The steps' output travels with them, so the set of the steps holding
    // any does too; the clone counts its own verification mismatches.
    D.epilogue = src.epilogue;
    D.epilogue.mismatches = 0;
    D.excludedSteps = src.excludedSteps;
    D.opGraph = src.opGraph;
    D.opAdapter = src.opAdapter;
    D.verifyChainVersions = src.verifyChainVersions;
    D.chainContentKeys = src.chainContentKeys;
    D.chainVersionMismatches = 0;
    D.composeGroups = src.composeGroups;
    // The space switches, and the per-slot index the compose step
    // asks before it takes the switched branch. Left out, the
    // frozen compose falls straight through to the namespace
    // parent -- which moves every switched control and everything
    // under it, and reads as a parity mismatch on 1509 control
    // frames of a rig that is standing in the same place.
    D.spaceSwitches = src.spaceSwitches;
    D.autoClavicles = src.autoClavicles;
    D.autoClavicleBySlot = src.autoClavicleBySlot;
    D.switchFrameContexts = src.switchFrameContexts;
    D.switchFrames = src.switchFrames;
    D.spaceSwitchBySlot = src.spaceSwitchBySlot;
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
    D.skinTopologyLayouts = src.skinTopologyLayouts;
    D.clustering = src.clustering;
    // Build's settings: the clone's partition settings and the worker's
    // kernels are the live program's choices, and its bodies count into
    // their own audit counter.
    D.chunkVertexTarget = src.chunkVertexTarget;
    D.chunkCap = src.chunkCap;
    D.useSimd = src.useSimd;
    D.purityAudit = src.purityAudit;
    D.purityViolations.count.store(0, std::memory_order_relaxed);
    D.cones = src.cones;
    D.closedSteps = src.closedSteps;
    D.closed = src.closed;
    D.lastAvars = src.lastAvars;
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
    D.coldPrologueUs = D.coldRegionUs = D.coldEpilogueUs = 0;
    D.coldFrames = 0;
    D.measureColdRuns = src.measureColdRuns;
    D.coldRunExcluded = false;
    D.measurementSuspended = false;
    // The steps above carry the source's stamps; the clone clears them too.
    D.stampedSteps = src.stampedSteps;
    D.jointMatrixPublished = src.jointMatrixPublished;
    D.chains = src.chains;
    // Per-consumer overlays borrow the live source layer only during a body.
    // A frozen clone rebases them on its own source layer when consumed.
    for (auto &chain : D.chains) {
        for (auto &revision : chain.revisions) revision.revisionInputs.Clear();
        for (auto &derived : chain.derived) derived.revision.revisionInputs.Clear();
    }
    D.deltaValues = src.deltaValues;
    D.deltaPresent = src.deltaPresent;
    D.deltaBasePaths = src.deltaBasePaths;
    D.deltaBaseMatrix = src.deltaBaseMatrix;
    D.lastDeltaBaseMatrix = src.lastDeltaBaseMatrix;
    D.deltaBaseOk = src.deltaBaseOk;
    D.lastDeltaBaseOk = src.lastDeltaBaseOk;
    D.weightObjects = src.weightObjects;
    D.weightProgram = src.weightProgram;
    D.weightCycleBlocked = src.weightCycleBlocked;
    D.weightFields = src.weightFields;
    D.volumePlacementBase = src.volumePlacementBase;
    D.weightIndex = src.weightIndex;
    D.volumePlacement = src.volumePlacement;
    D.placedVolumes = src.placedVolumes;
    D.currentPhaseWeights = src.currentPhaseWeights;
    D.falloffLuts = src.falloffLuts;
    // Sized but empty: every packet is rebuilt by its step during the run
    // (reads always follow writes in program order, as live), so copying
    // the previous run's dense fields per job would be pure waste.
    // Completed signatures require their owned packet payloads.
    D.weightPackets = src.weightPackets;
    D.rebuild = src.rebuild;
    D.sourceBackedPaths = src.sourceBackedPaths;
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
    // The upstream layer as the source's last run placed it: its avar
    // slots, leaves and outputs hold those values, so a job diffs its own
    // table against `lastUpstream` (rule 8) before it runs.
    D.upstreamAdmissible = src.upstreamAdmissible;
    D.upstreamOracle = src.upstreamOracle;
    D.upstreamSetsBuilt = src.upstreamSetsBuilt;
    D.upstream = src.upstream;
    D.lastUpstream = src.lastUpstream;
    D.upstreamOn = src.upstreamOn;
    D.upstreamChanged = src.upstreamChanged;
    D.upstreamMovedThisRun = src.upstreamMovedThisRun;
    D.leafSamples = src.leafSamples;
    // The path leaves' bookkeeping. Their values ride the chains and weight
    // objects above; a frozen job takes its revisions' and derived targets'
    // leaves from the job's vector (RigExecFrameInputs::revisionLeaves,
    // derivedLeaves) and samples none itself.
    D.pathLeafRefs = src.pathLeafRefs;
    D.pathLeafChainResults = src.pathLeafChainResults;
    D.pathLeafChainSerial = src.pathLeafChainSerial;
    D.pathLeafSamples = src.pathLeafSamples;
    // The run the copied leaves' observed values and versions refer to.
    D.pathLeafRun = src.pathLeafRun;
    D.resolvedRoutedPrims = src.resolvedRoutedPrims;
    D.avarsDisturbed = src.avarsDisturbed;
    D.folded = src.folded;
    D.anyOverridden = src.anyOverridden;
    D.publishWeightFields = src.publishWeightFields;
    // Retained property values and bindings. Frozen jobs sample source facts
    // and execute the same compiled graph with no attribute reads in workers.
    D.propertyChains = src.propertyChains;
    D.oraclePublications = src.oraclePublications;
    D.oracleWeightInputs = src.oracleWeightInputs;
    D.propertyRecords = src.propertyRecords;
    D.propertyRecordById = src.propertyRecordById;
    D.poseDependencyPaths = src.poseDependencyPaths;
    D.providerProgram = src.providerProgram;
    D.providerValues = src.providerValues;
    D.providerLeaves = src.providerLeaves;
    D.providerLeafBlocked = src.providerLeafBlocked;
    // The leaf keys in opAdapter answer for these samples only together
    // with the reasons publication has not yet consumed.
    D.spaceLeafIndex = src.spaceLeafIndex;
    D.spaceLeafRekey = src.spaceLeafRekey;
    D.spaceLeafKeys = src.spaceLeafKeys;
    D.providerLeafValues = src.providerLeafValues;
    D.providerLeafChains = src.providerLeafChains;
    D.providerRoutedReads = src.providerRoutedReads;
    D.providerFrozenKeys = src.providerFrozenKeys;
    D.providerExternalSlots = src.providerExternalSlots;
    D.providerFrameInputs = src.providerFrameInputs;
    D.poseProviderInputs = src.poseProviderInputs;
    D.connectedPoseProviders = src.connectedPoseProviders;
    D.providerParentRawLeaves = src.providerParentRawLeaves;
    D.providerRefreshes = src.providerRefreshes;
    D.providerRefreshBefore = src.providerRefreshBefore;
    D.propertyVersionCount = src.propertyVersionCount;
    D.headLeaves = src.headLeaves;
    D.headOverrideSlots = src.headOverrideSlots;
    D.headOverrideSlotsByName = src.headOverrideSlotsByName;
    D.propertyValues = src.propertyValues;
    D.propertyVersionValid = src.propertyVersionValid;
    D.propertyChanged = src.propertyChanged;
    D.chainValid = src.chainValid;
    D.chainFinal = src.chainFinal;
    D.recordValues = src.recordValues;
    D.recordStoodAside = src.recordStoodAside;
    D.headOverrides = src.headOverrides;
    D.lastHeadOverrides = src.lastHeadOverrides;
    D.headOverrideMoved = src.headOverrideMoved;
    D.headLeavesSampled = src.headLeavesSampled;
    D.headLeafTime = src.headLeafTime;
    D.headLeafStamp = src.headLeafStamp;
    D.headEverRan = src.headEverRan;
    D.headStamp = src.headStamp;
    D.headLeafSamples = src.headLeafSamples;
    D.headOpsRun = src.headOpsRun;
    // Reader bindings resolve captured source facts and current typed outputs.
    D.readerWalks = src.readerWalks;
    D.crossDomainReads = src.crossDomainReads;
    D.crossDomainErrors = src.crossDomainErrors;
    D.crossDomainOrdinals = src.crossDomainOrdinals;
    D.readerWalkMoved = src.readerWalkMoved;
    D.readerWalkChanged = src.readerWalkChanged;
    D.avarHeadReads = src.avarHeadReads;
    // Adopt only a completed snapshot whose owned outputs still match the
    // signatures copied from that completion. A patched or poisoned payload
    // must run cold rather than borrowing a signature for a different value.
    bool retainedComplete = src.everRan && src.opAdapter.compiled &&
        src.opAdapter.everRan && src.opAdapter.inputKeys.size()==src.steps.size() &&
        src.opAdapter.sourceKeys.size()==src.steps.size() &&
        src.opAdapter.inputExact.size()==src.steps.size();
    for(const auto &op:src.opGraph.ops)for(const auto id:op.descriptor.writes) {
        if(id>=D.opAdapter.values.size()) { retainedComplete=false; continue; }
        const auto &value=D.opAdapter.values[size_t(id)];
        const auto domain=RigExecBakedSlotDomain(value.domain);
        if(!value.initialized || !RigExecBakedOpValueKeyIsExact(D,domain,value.slot)) {
            retainedComplete=false; continue;
        }
        if(!RigExecBakedOpValueKeyStands(D,domain,value.slot,value.key))
            retainedComplete=false;
    }
    D.opAdapter.everRan=retainedComplete;
    if(!retainedComplete)D.opAdapter.retainedFirst.clear();
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
    const RigExecBakedProgram *program = evaluator.GetBakedProgram();
    if (!program) {
        return fail("no compiled program is available to freeze");
    }
    const RigExecBakedProgramImpl &B = program->GetStepGraph();
    // Property chains and weight envelopes run as declared graph operations.
    // Source discovery must match the compiled epoch and every envelope
    // must name its compiled field producer.
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
        for (const RigExecBakedPropertyChain &chain : B.propertyChains) {
            for (const RigExecBakedPropertyChain::Revision &revision :
                 chain.revisions) {
                if (!revision.weightObject.IsEmpty() && revision.weightField < 0) {
                    return fail(
                        "diag " + revision.mover.GetString() +
                        " has no compiled weight field for its property envelope");
                }
            }
        }
    }
    // No gate on xform slots, native sources, delta bases, or
    // geometry-domain constraints: the seeds sample per frame through the
    // program's hook, and the constraint steps are the shared bodies, so a
    // frozen run reproduces them from the transported seeds.
    // Pose constraint envelopes require compiled field bindings. Geometry
    // constraints consume the revision's per-point packet instead.
    for (const RigExecBakedProgramImpl::Constraint &constraint :
         B.constraints) {
        if (!constraint.weightObject.IsEmpty() &&
            constraint.pointsTarget.IsEmpty() && constraint.weightField < 0) {
            return fail("constraint " + constraint.path.GetString() +
                        " has no compiled weight field for its envelope");
        }
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
                revision.op != RigExecRevisionOp::EmitGuidePoints &&
                revision.op != RigExecRevisionOp::External) {
                return fail("revision " +
                            revision.moverPath.GetString() + " runs op '" +
                            RigExecRevisionKindToken(revision.op)
                                .GetString() +
                            "', which the frozen executor does not "
                            "implement");
            }
            if (revision.op == RigExecRevisionOp::External &&
                (!revision.binding.handler || !revision.leaves.decl.assembles))
                return fail("external revision " + revision.moverPath.GetString() +
                            " has no compiled stage-free handler declaration");
            // Phase reads consume the worker's current typed point/frame
            // versions. Revision assembly uses captured source facts.
            // Dense blend samples use those bindings; sparse blend-shape
            // offsets are authored layout data, without point-chain phases.
            if (revision.weightCurrentPhase && revision.weightField < 0) {
                return fail("revision " +
                            revision.moverPath.GetString() +
                            " has no compiled field for its current-phase weight");
            }
            // Driver frames and the geometry-delta hand-off use the
            // worker's own Solve-step aggregate, read in program order,
            // and the delta through the shared constraint step and
            // the shared fold -- the seeds patch the delta bases the
            // constraint step measures against, the step stashes the delta,
            // and FoldInfluences reads the stash, all shared bodies in
            // program order.
            // Skin packet walks resolve the worker's current property
            // versions through the same declared reads as live assembly.
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
            // Derived phases are copied typed point/frame bindings; their
            // shared body resolves them after the declared producers join.
        }
    }

    return true;
}

bool
RigExecFreezeProgram(const RigExecRigEvaluator &evaluator,
                     std::shared_ptr<const RigExecFrozenProgram> *frozen,
                     std::string *error)
{
    RigExecInputReplayComparisonScope replayComparison("RigExecFreezeProgram");
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
    // Workers clone this snapshot and index its spelled slot paths unchecked.
    if (!TF_VERIFY(snapshot->program.pathTexts &&
                   snapshot->program.pathTexts->size() ==
                       snapshot->program.paths.size())) {
        if (error) {
            *error = "slot path texts do not match the slot table";
        }
        return false;
    }
    if (evaluator.cpuReference) {
        // Freeze is an owning-thread source boundary. Enabling the judge
        // after Evaluate must capture its independent inputs here, rather
        // than require another production generation or retain old facts.
        auto &reference = snapshot->program;
        struct SourceCaptureScope {
            RigExecBakedProgramImpl &program;
            ~SourceCaptureScope() {
                program.stage = UsdStageRefPtr();
                program.resolvedInputs = nullptr;
            }
        } sourceScope{reference};
        reference.stage = B.stage;
        reference.resolvedInputs = B.resolvedInputs;
        reference.oraclePublications.emplace();
        RigExecBakedBeginOracleReference(&reference, 0, B.lastTime);
    }
    if (B.jointSolverBinding) {
        snapshot->jointSolverBinding = *B.jointSolverBinding;
    }
    snapshot->solverGuidesPresent = !B.solverArrays.empty();
    // Handle identities, captured here (UI thread) as plain data: the
    // worker must not even ask a handle whether it is valid.
    _ForEachPatchableInput(
        B, [&snapshot](const auto &input) {
            snapshot->inputHeadPaths.push_back(
                input.head ? input.head.GetPath() : SdfPath());
            snapshot->inputConstants.push_back(VtValue(
                input.sourceBacked ? input.sourceFallback : input.constant));
        }, /*includeIntervening=*/false);
    // The samplers' own keys, empty where they read nothing: one source of
    // keys for the sampler, the worker and the snapshot.
    for (const RigExecBakedProgramImpl::ConstraintArrays &arrays :
         B.constraintArrays) {
        const bool valid = arrays.prim.IsValid();
        for (size_t k = 0; k < arrays.keys.size(); ++k) {
            snapshot->arrayKeys.push_back(
                valid && arrays.Sampled(k) ? arrays.keys[k] : SdfPath());
        }
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
        // The sampler's key for the mover's defaultWeight sample.
        snapshot->moverDefaultWeightKeys.push_back(
            revision.moverPath.AppendProperty(
                TfToken("inputs:defaultWeight")));
    }
    // The sampler's synthetic weight-array keys, by the same function.
    for (const RigExecBakedProgramImpl::WeightObject &object :
         B.weightObjects) {
        for (const TfToken &role :
             {_frozenWeightTokens->targetPointsKey,
              _frozenWeightTokens->samplePointsKey,
              _frozenWeightTokens->curvePointsKey,
              _frozenWeightTokens->combineTargetCountKey}) {
            snapshot->weightArrayKeys.push_back(
                _FrozenWeightArrayKey(object.path, role));
        }
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
