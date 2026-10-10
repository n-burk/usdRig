// Binding epochs: validation, graph construction, and request preparation.

#include "rigEvaluatorInternal.h"
#include "crossDomainInputs.h"
#include "inputReplay.h"
#include "moverGraph.h"
#include "rigExecScene/poseSceneLowering.h"
#include "sceneDispatch.h"
#include "rigEvaluatorDependencies.h"
#include "rigEvaluatorPropertyBindings.h"
#include "rigEvaluatorConstraints.h"
#include "parallel.h"
#include "movers/moverRegistry.h"
#include "frameExtraction.h"
#include "solverKernels.h"
#include "rigExecMath/singleChainIk.h"
#include "rigExecMath/solvers.h"

#include "pxr/base/work/dispatcher.h"
#include "pxr/base/work/loops.h"
#include "pxr/base/tf/diagnostic.h"
#include "pxr/base/tf/pyLock.h"
#include "pxr/base/tf/scoped.h"
#include "pxr/base/tf/stringUtils.h"
#include "pxr/usd/usd/attribute.h"
#include "pxr/usd/usd/prim.h"
#include "pxr/usd/usd/primRange.h"
#include "pxr/usd/usd/relationship.h"
#include "pxr/usd/usdGeom/xformCache.h"
#include "pxr/usd/usdGeom/gprim.h"
#include "pxr/usd/usdGeom/imageable.h"
#include "pxr/usd/usdGeom/mesh.h"
#include "pxr/usd/usdGeom/xformable.h"

#include <algorithm>
#include <atomic>
#include <cassert>
#include <cstdio>
#include <functional>
#include <limits>
#include <optional>
#include <set>
#include <unordered_map>

namespace rigExec {

using namespace evaluatorDetail;

namespace {

// Removed attributes the compile refuses, interned once at load: interning
// takes the token registry's lock.
const TfToken _removedSamplePhase("rigExec:samplePhase");
const TfToken _removedRibbonPhases[] = {
    TfToken("rigExec:driverCurveReadPhase"),
    TfToken("rigExec:surfaceReadPhase")};

} // namespace

bool
RigExecRigEvaluator::Compile(std::vector<std::string> *errors)
{
    RigExecInputReplayCompileCall compileCall;
    if (_inputReplayObserver) {
        _inputReplayObserver->Options(_solverGuidesEnabled, _publishWeightFields);
        compileCall = _inputReplayObserver->Compile(errors);
    }
    RigExecInputReplayImplementationScope replayImplementation{bool(_inputReplayObserver)};
    // A caller asking for a compile is asking the stage again, whatever the
    // settle path last concluded about it: nothing the memo remembers may
    // answer for this call, and whatever this call concludes is the caller's
    // to read, not the settle path's to replay.
    _failedCompile = _FailedCompileMemo();
    // Element selection is an authored input contract, not a recoverable
    // operation cycle. Validate it before operation set-aside retries.
    const UsdPrim inputRoot = _stage ? _stage->GetPrimAtPath(_rigPath) : UsdPrim();
    if (inputRoot) for (const auto &prim : UsdPrimRange(inputRoot)) {
        for (const auto &attribute : prim.GetAttributes()) {
            if (!attribute.HasAuthoredMetadata(TfToken(RigExecInputElementMetadataName))) continue;
            VtValue element;
            attribute.GetMetadata(TfToken(RigExecInputElementMetadataName), &element);
            if (!element.IsHolding<int>() || element.UncheckedGet<int>() < 0 ||
                attribute.GetTypeName().GetType() != TfType::Find<GfVec3f>()) {
                if (errors) errors->push_back(attribute.GetPath().GetString() +
                    ": rigExecInputElement requires a nonnegative int on a Vec3f input");
                if (_inputReplayObserver)
                    _inputReplayObserver->CompileResult(compileCall, false, errors);
                return false;
            }
        }
    }
    const bool compiled = _CompileEpoch(errors);
    // The shadow is asked every question this evaluator is, in the same
    // order, so that the one thing that differs between them is how their
    // notices clear the value caches.
    _EnsureScopedClearShadow();
    if (_scopedClearShadow) {
        _scopedClearShadow->Compile();
    }
    if (_inputReplayObserver)
        _inputReplayObserver->CompileResult(compileCall, compiled, errors);
    return compiled;
}

// Retry without failed operations so incomplete rigs still publish valid
// outputs. Structural edits retry all operations; a failure with no operation
// owner rejects the rig as a whole.
bool
RigExecRigEvaluator::_CompileEpoch(std::vector<std::string> *errors)
{
    // Bounds the retries on a rig that is broken everywhere at once; past it
    // the rig fails as a whole, as it always did.
    static constexpr size_t kMaxSkippedOperations = 64;

    const std::map<SdfPath, std::string> committedSkips = _skippedOperations;
    _skippedOperations.clear();
    std::vector<std::string> attempt;
    for (;;) {
        attempt.clear();
        _CompileFailure failure;
        if (_CompileEpochAttempt(&attempt, &failure)) {
            break;
        }
        // The operation the error is about -- or, for a dependency cycle,
        // every operation in it: a cycle is no one member's fault, and
        // setting aside whichever happens to be named first would leave the
        // rest running on an order nobody chose.
        const auto &culprits = failure.operations;
        const bool repeat = std::any_of(
            culprits.begin(), culprits.end(),
            [this](const SdfPath &path) { return _IsSkippedOperation(path); });
        if (culprits.empty() || repeat ||
            _skippedOperations.size() >= kMaxSkippedOperations) {
            // Not one operation's fault. The rig fails as a whole, and the
            // program it had -- with the operations IT had set aside --
            // keeps running. What was set aside on the way is said first:
            // a rig whose only operation is broken fails with "publishes
            // no outputs" once that operation is gone, and that line alone
            // would hide the reason.
            if (errors) {
                // Once per reason: a cycle sets aside every member
                // for the one error.
                std::set<std::string> said;
                for (const auto &[path, reason] : _skippedOperations) {
                    if (said.insert(reason).second) {
                        errors->push_back(reason);
                    }
                }
                errors->insert(errors->end(), attempt.begin(), attempt.end());
            }
            _skippedOperations = committedSkips;
            return false;
        }
        for (const SdfPath &culprit : culprits) {
            _skippedOperations.emplace(culprit, failure.message);
        }
    }
    for (const auto &[path, reason] : _skippedOperations) {
        const std::string line =
            "Skipped operation " + path.GetString() + " -- " + reason;
        TF_WARN("RigExec %s: %s", _rigPath.GetText(), line.c_str());
        if (errors) {
            errors->push_back("warning: " + line);
        }
    }
    if (errors) {
        errors->insert(errors->end(), attempt.begin(), attempt.end());
    }
    return true;
}

bool
RigExecRigEvaluator::_CompileEpochAttempt(std::vector<std::string> *errors,
                                         _CompileFailure *failure)
{
    RIGEXEC_PROFILE_SCOPE_CAT(_profiler, "Compile", "compile");
    // Exec preparation may load Python plugins on a worker. Release the GIL
    // through dispatcher destruction too, since early returns still join tasks.
    TF_PY_ALLOW_THREADS_IN_SCOPE();
    // Sequential region stamps: Compile is flat code with early returns,
    // so RAII scopes cannot span its phases; each stamp closes the
    // previous region and opens the next. One branch when disabled.
    // The clock starts on the FIRST line of the body rather than beside the
    // first stamped phase, where it used to start. Everything above that
    // point could only ever measure as zero, and the trace had 6.1 ms sitting
    // between the Compile scope opening and DiscoverValidate.Validate
    // starting that no row claimed. Starting here hands that gap to
    // Compile.Prologue below instead of losing it in the parent scope.
    const bool profileCompile = _profiler.IsEnabled();
    uint64_t compileRegionStart =
        profileCompile ? RigExecProfiler::NowUs() : 0;
    auto stampCompileRegion = [&](const char *name) {
        if (!profileCompile) {
            return;
        }
        const uint64_t now = RigExecProfiler::NowUs();
        _profiler.Record(name, "compile", compileRegionStart, now);
        compileRegionStart = now;
    };
    // The program describes the epoch this call is about to replace, so it
    // stops being the rig's program here: a Compile that fails and restores
    // the previous epoch returns before the rebuild at the tail, this local
    // goes out of scope, and the rig is left dynamic -- slower and never
    // wrong. Retired rather than destroyed, because its persistent GEOMETRY
    // state (which node ran with which packet) is not about the epoch: the
    // dynamic path keeps its _liveGraphs across a recompile and reconnects
    // whichever nodes survive, and the rebuild below does the same.
    std::unique_ptr<RigExecBakedProgram> retiringBakedProgram =
        std::move(_bakedProgram);
    _bakedProgramStale = false;
    // A new epoch is a new question: whatever refused the last one said
    // nothing about this one, and whatever made its program bail neither.
    _bakeRefused = false;
    _bakeRefusalReasons.clear();

    // The committed footprint answers for the committed digest, and this
    // call is about to replace both: until its own digest is joined, every
    // notice is suspect, including the ones its derived start frames raise.
    // The candidates noted against it go with it: this compile reads the
    // stage they describe.
    _digestGate = _DigestGate();
    _certainCandidates.clear();
    _certainCandidatesOverflowed = false;
    stampCompileRegion("Compile.Prologue");
    // Derived start-frame targets first, single-threaded, before the digest
    // dispatch and every parallel stage read below: the inference is a pure
    // function of the asset state, so the digest then covers it like any
    // other composed opinion, and the scheduler, exec, and the bake all see
    // it as authored. Returned warnings are emitted here, past the
    // derivation's notice block: both channels on purpose, like the
    // transform-authority pass -- TF_WARN is what a host surfaces to the
    // author, the errors vector is what a test can read.
    for (const std::string &message : _ApplyDerivedStartFrames()) {
        if (errors) {
            errors->push_back("warning: " + message);
        }
        TF_WARN("%s", message.c_str());
    }
    stampCompileRegion("Compile.DerivedStartFrames");

    // The structure digest is a pure read of the composed stage that boils it
    // down to one number, and that number is not consulted until the epoch is
    // committed far below: nothing in between reads it, and compile authors
    // nothing to the stage for it to miss past the derived opinions above
    // (which the digest reads, deterministically, as composed targets). So
    // it runs beside the WHOLE of
    // compile rather than beside only its tail.
    // MEASURED (biped_stack_anim, 201.3 ms compile): dispatched at the old
    // site -- after DiscoverValidate, at t=35.7 ms -- the digest's 107 ms
    // landed at t=143.1 ms, while the main thread reached the join below at
    // t=122.3 ms and then sat idle for 23.4 ms. Compile was paying for the
    // digest after all, in waiting rather than in work. Dispatched here it
    // lands around t=107 ms, comfortably ahead of the join.
    // Safe to hoist past DiscoverValidate because that phase AUTHORS NOTHING:
    // it reads the composed stage and reports, so there is no edit for the
    // digest to race. And a rig malformed enough for DiscoverValidate to
    // reject is still one _ComputeStructureDigest reads without complaint --
    // every lookup in it goes through GetPrimAtPath/GetAttribute/Get, which
    // hand back invalid objects that the code already tests for, rather than
    // throwing. The single precondition it cannot survive is a null _stage,
    // which the old site sat downstream of; hence the guard in the lambda.
    // The 0 that guard leaves behind is never read -- Compile returns false
    // on a null stage long before the commit below.
    // It runs as three tasks, one per segment (_DigestSegment), each
    // writing only its own string; the join below concatenates them in
    // segment order and hashes the result, which is the digest one serial
    // walk produces. The Solvers segment is most of the work, so it is
    // dispatched first.
    // digestParts is declared before the dispatcher so it outlives it, and
    // WorkDispatcher's destructor waits -- which is what covers the early
    // returns between here and the join, now the whole of compile rather
    // than just its tail. Declared AFTER retiringBakedProgram so it is still
    // destroyed first: the digest tasks are joined before the retired
    // program is torn down underneath them. Anything declared after the
    // dispatcher dies before that wait, so a task must not reference it:
    // each task holds its own copy of computeDigestPart, which carries only
    // `this`, &digestParts and &digestFootprints, all of which outlive the
    // dispatcher. Each task writes its segment's text and the footprint it
    // recorded (the prims outside the rig it read) into its own slots.
    std::string digestParts[_DigestSegmentCount];
    _DigestFootprint digestFootprints[_DigestSegmentCount];
    WorkDispatcher digestDispatcher;
    const auto computeDigestPart = [this, &digestParts,
                                    &digestFootprints](size_t i) {
        if (_stage) {
            digestParts[i] = _ComputeStructureDigest(
                (1u << i) | _DigestBesideCompile, &digestFootprints[i]);
        }
    };
    for (const size_t i : {size_t(1), size_t(2), size_t(0)}) {
        if (RigExecParallelEvaluationEnabled()) {
            digestDispatcher.Run([computeDigestPart, i]() {
                computeDigestPart(i);
            });
        } else {
            computeDigestPart(i);
        }
    }
    // A compile that returns early discards the digest unread, but the
    // split verifier still checks it there: the rigs Compile rejects
    // include the ones whose pose inputs close a cycle, where the Solvers
    // segment's bytes depend on its walk order. Declared after the
    // dispatcher, so it runs first and waits for the tasks itself.
    bool digestJoined = false;
    const TfScoped<> verifyDigestOnEarlyReturn(
        [this, &digestJoined, &digestDispatcher, &digestParts]() {
            if (!digestJoined && _stage && _DigestSplitVerifyRequested()) {
                digestDispatcher.Wait();
                _JoinStructureDigest(digestParts);
            }
        });
    // Zero-width when the digest is dispatched, which is the point: it marks
    // WHERE the digest was launched so the trace can be read against the
    // worker row. With RIGEXEC_ENABLE_PARALLEL_EVAL=0 the call above ran
    // inline and this stamp measures the whole of it, exactly as it did at
    // the old site.
    stampCompileRegion("Compile.StructureDigest");

    // The parts WITHIN those regions, for the same reason and on the same
    // clock: a phase that takes a fifth of the compile says nothing about
    // which of its passes to go and look at. Strictly nested inside the
    // stamps above, so the trace shows them as children rather than as a
    // second, competing set of rows. Closed at each phase boundary so no
    // block spans two phases, and on destruction so an early return
    // cannot leave one open.
    RigExecProfilePhases compileBlocks(&_profiler, "compile");

    std::function<void()> invalidateEpoch;
    auto fail = [errors, failure, &invalidateEpoch](const std::string &message,
                                       SdfPathVector operations = {}) {
        *failure = {message, std::move(operations)};
        if (errors) {
            errors->push_back(message);
        }
        if (invalidateEpoch) invalidateEpoch();
        return false;
    };

    compileBlocks.Next("DiscoverValidate.Validate");

    if (!_stage) {
        return fail("no stage; nothing to compile");
    }

    const UsdPrim rig = _stage->GetPrimAtPath(_rigPath);
    if (!rig) {
        return fail("Rig prim not found: " + _rigPath.GetString());
    }
    // Prototype-hosted rigs fail validation (spec §4.1: a rig in a
    // prototype or otherwise unable to deinstance is rejected).
    if (rig.IsInstanceProxy() || rig.IsInPrototype()) {
        return fail("Rig is instance-proxy/prototype hosted: " +
                    _rigPath.GetString());
    }
    // Phase A: validation into locals. Nothing below mutates evaluator
    // state until every check passes, so a failed structural edit keeps
    // the previous epoch publishable (spec §4.1 atomic transactions).
    std::vector<SdfPath> newJointPaths =
        _DiscoverJointOutputs(_stage, _rigPath, _skippedOperations);
    // A rig with no joints is legal.
    // It used to be rejected here, on the reading that a joint is what a rig
    // publishes. That was never true of the evaluator, only of this check: a
    // mover writes an exact target, and a target is a points array, a plain
    // UsdGeomXformable's transform, or a scalar property just as readily as
    // it is a joint frame -- three output domains that all reach a consumer
    // through RigExecRigPose. Requiring a joint forced authors to add a
    // vestigial one to rigs that pose none (10_AimXformTurret says so in a
    // comment), and rejected outright the simplest rig there is: a constraint
    // aiming one Xform at another.
    // What the rig DOES need is at least one output, and that cannot be known
    // until the mover walk below has run. The check moved there.
    // Controls and placed volumes are discovered alongside the joints. Zero
    // of either kind is ordinary; the combined output gate below decides
    // whether the whole rig is genuinely empty.
    std::vector<SdfPath> newControlPaths =
        _DiscoverControls(_stage, _rigPath);
    std::vector<SdfPath> newVolumeWeightPaths =
        _DiscoverVolumeWeights(_stage, _rigPath);
    // Every aggregate frame provider, wherever the author placed it: their
    // computePointFrameArray results are published (and drawn as guides by
    // the imaging chain, like OpenExec's IrJointScope guides). Discovered
    // here, with the other providers, because it is the last thing the exec
    // lane below needs before it can start.
    std::vector<UsdPrim> aggregateSolvers =
        _DiscoverAggregateSolvers(_stage, _rigPath);
    aggregateSolvers.erase(
        std::remove_if(aggregateSolvers.begin(), aggregateSolvers.end(),
                       [this](const UsdPrim &solver) {
                           return _IsSkippedOperation(solver.GetPath());
                       }),
        aggregateSolvers.end());
    std::vector<SdfPath> solverArrayPaths;
    solverArrayPaths.reserve(aggregateSolvers.size());
    for (const UsdPrim &solver : aggregateSolvers) {
        solverArrayPaths.push_back(solver.GetPath());
    }

    compileBlocks.Next("DiscoverValidate.WarmupDispatch");
    // Dispatched as early as its inputs allow: straight after the providers
    // are discovered, ahead of the pose-interpolator solve and the
    // transform-authority pass, which read the stage and author nothing. The
    // lane task is compile's critical path in both kinds of epoch -- the
    // warm-up and the solver-request preparations behind it in an eager
    // one, the guides in a deferred one -- so every millisecond it starts
    // earlier is one off the compile. MEASURED on puppetA: those two passes
    // are ~1.9 ms that used to sit in front of the dispatch.
    // Warm the shared exec network, off the critical path, in an epoch that
    // prepares its requests here.
    // Every request an eager epoch prepares -- the solver batches, the
    // first-frame pose, the main epoch request, the guides -- compiles into
    // the SAME exec network for this stage, and whichever request asks for a
    // provider first pays to compile it. Asking for all of them at once,
    // here, builds that shared network in one wide parallel round rather
    // than in a series of narrow ones, and it runs beside the scheduling
    // work below that has to happen anyway. It is pure preparation: no value
    // is read from it, and a request that cannot be built valid simply
    // leaves the real preparations to compile what they need, and to report
    // their own failure.
    compileBlocks.Next("DiscoverValidate.Providers");

    // Pose interpolators, discovered and SOLVED here. Solving is the whole
    // reason this is compile-time work: inverting the matrix of every pose's
    // kernel value at every other pose is a constant of the authored data,
    // and doing it per frame would be inverting the same matrix 326 times a
    // second to be handed the same answer.
    std::unique_ptr<RigExecSceneDescriptors> compileScene;
    const UsdStageWeakPtr compileSceneStage(_stage);
    uint64_t compileSceneSerial = 0;
    bool compileSceneReusable = false;
    std::vector<_PoseInterpolator> newPoseInterpolators;
    {
        RIGEXEC_PROFILE_SCOPE_CAT(_profiler, "Compile.Providers.PoseInterpolators", "compile");
        compileScene = std::make_unique<RigExecSceneDescriptors>();
        const uint64_t before = GetStageEditSerial();
        std::string interpolatorError;
        SdfPath interpolatorOperation;
        if (!_CompilePoseInterpolators(newJointPaths, newControlPaths,
                                       &newPoseInterpolators, errors,
                                       &interpolatorError, &interpolatorOperation,
                                       compileScene.get(),&_profiler)) {
            return fail(interpolatorError, {interpolatorOperation});
        }
        // No interpolators means no capture. All existing lowering reads
        // still use the same fresh Default snapshot at their original site.
        if (compileScene->rigRoot.IsEmpty()) {
            compileScene.reset();
        } else {
            compileSceneSerial = before;
            compileSceneReusable = before == GetStageEditSerial();
        }
    }

    // Transform-authority validation (host-durability redesign).
    // Neither condition can FAIL a compile, and both are reported rather
    // than fixed: the rig still evaluates exactly right, because the
    // evaluator reads rest:space and the avars and nothing else. What
    // breaks is the BOUNDS -- a provider's computed extent bakes its posed
    // frame into asset-relative space, which is only the whole story while
    // nothing else contributes a transform between the asset root and the
    // provider. Refusing to compile over a framing inaccuracy would be
    // wildly out of proportion; saying nothing would leave an author
    // wondering why one control frames to the wrong place.
    {
        RIGEXEC_PROFILE_SCOPE_CAT(_profiler, "Compile.Providers.BoundsValidation", "compile");
        const SdfPath assetRoot = _rigPath.GetParentPath();
        auto warn = [errors](const std::string &message) {
            // Both channels on purpose: TF_WARN is what a host surfaces to
            // the author, and the errors vector is what a test can read.
            // Compile still returns true.
            if (errors) {
                errors->push_back("warning: " + message);
            }
            TF_WARN("%s", message.c_str());
        };
        // Resolving a prim's purpose walks up the namespace to the first
        // authored opinion, and this pass asks for it once per provider and
        // then again for every descendant of every provider -- so a joint
        // deep in a chain is resolved once per ancestor provider. Purpose is
        // a pure function of the composed stage, which does not change while
        // a compile runs, so resolve each prim once.
        std::unordered_map<SdfPath, TfToken, SdfPath::Hash> purposeCache;
        const auto resolvedPurpose = [&purposeCache](const UsdPrim &prim) {
            auto it = purposeCache.find(prim.GetPath());
            if (it == purposeCache.end()) {
                it = purposeCache.emplace(
                    prim.GetPath(),
                    UsdGeomImageable(prim).ComputePurpose()).first;
            }
            return it->second;
        };
        // Every Boundable provider, aggregate solvers included: they
        // inherit Boundable/Xformable too, so an authored op on one is
        // applied by BBoxCache to an already-baked extent while the guide
        // it draws ignores it entirely.
        std::vector<SdfPath> providers = newJointPaths;
        providers.insert(providers.end(), newControlPaths.begin(),
                         newControlPaths.end());
        providers.insert(providers.end(), newVolumeWeightPaths.begin(),
                         newVolumeWeightPaths.end());
        providers.insert(providers.end(), solverArrayPaths.begin(),
                         solverArrayPaths.end());
        for (const SdfPath &providerPath : providers) {
            const UsdPrim prim = _stage->GetPrimAtPath(providerPath);
            if (!prim) {
                continue;
            }
            // xformOps arrive on every provider now that RigExecXformable
            // inherits UsdGeomBoundable, but they are NOT a transform
            // authority: rest:space plus the avars are the only one (the
            // Ir alignment). An authored op is a second one that nothing
            // reads, so the prim moves in a stock UsdGeom traversal while
            // the rig ignores it entirely.
            if (const UsdGeomXformable xformable = UsdGeomXformable(prim)) {
                bool resetsStack = false;
                if (!xformable.GetOrderedXformOps(&resetsStack).empty()) {
                    warn(prim.GetTypeName().GetString() + " " +
                         providerPath.GetString() +
                         " authors xformOps, which are not a transform "
                         "authority for a RigExec provider (rest:space and "
                         "the avars are); the ops are ignored by evaluation "
                         "and are not in the computed extent");
                }
            }
            // An Xformable BETWEEN the asset root and the provider used to
            // warn here, because its transform was dropped. It is now
            // composed at evaluation, by _ComposeInterveningXforms, so there
            // is nothing left to report: placing a rig -- or one leg of an
            // assembly -- under an Xform inside the asset is a supported
            // shape, and warning ten times per compile about a configuration
            // that works is noise nobody can act on.
            // The check above it stays. An op authored on the PROVIDER is
            // still not a transform authority, which is a different claim
            // and still true.

            // Hoisted out of the walk: the provider's own purpose is the
            // same for every one of its descendants.
            const TfToken providerPurpose = resolvedPurpose(prim);

            // A provider's extent covers the guides beneath it, and only
            // those. Authored geometry parented under one is invisible to
            // it -- and to every ancestor, because UsdGeomBBoxCache stops
            // descending at a Boundable -- so the gprim silently drops out
            // of every bounding box in the scene.
            for (const UsdPrim &descendant : UsdPrimRange(prim)) {
                if (descendant == prim) {
                    continue;
                }
                // A provider nested under a provider with a DIFFERENT
                // resolved purpose is dropped from the ancestor's extent
                // on purpose: one extent carries one purpose, and the
                // bounding-box cache files it under the ancestor's. Nobody
                // reading the namespace would guess that, so say it.
                if (descendant.IsA<UsdGeomImageable>()) {
                    const TfToken descendantPurpose =
                        resolvedPurpose(descendant);
                    if (!descendantPurpose.IsEmpty() &&
                        !providerPurpose.IsEmpty() &&
                        descendantPurpose != providerPurpose &&
                        TfStringStartsWith(
                            descendant.GetTypeName().GetString(),
                            "RigExec")) {
                        warn(descendant.GetTypeName().GetString() + " " +
                             descendant.GetPath().GetString() +
                             " has purpose '" +
                             descendantPurpose.GetString() +
                             "' but is nested under " +
                             providerPath.GetString() + " whose purpose is '" +
                             providerPurpose.GetString() +
                             "'; one extent carries one purpose, so this "
                             "provider is excluded from its ancestor's "
                             "bounds");
                    }
                }
                if (descendant.IsA<UsdGeomGprim>()) {
                    warn("gprim " + descendant.GetPath().GetString() +
                         " is parented under RigExec provider " +
                         providerPath.GetString() +
                         "; a provider's computed extent covers only the "
                         "guides beneath it, and bounds stop descending at "
                         "a Boundable, so this geometry is absent from "
                         "every bounding box that should contain it");
                }
            }
        }
    }

    std::map<SdfPath, SdfPath> newRibbonDriverPoints;
    compileBlocks.Next("DiscoverValidate.AggregateProviders");
    for (const UsdPrim &child : aggregateSolvers) {
        // A ribbon's driver-curve points, resolved to the exact native
        // attribute. This replaces the compiler's last authoring pass:
        // the resolution is compiled state (rewiring the relationship is
        // structural, and the epoch digest already treats it that way),
        // and the values ride in as exec overrides at evaluation time.
        if (child.GetTypeName() == "RigExecRibbon") {
            SdfPathVector curves;
            if (const UsdRelationship rel = child.GetRelationship(
                    TfToken("rigExec:driverCurve"))) {
                rel.GetTargets(&curves);
            }
            if (!curves.empty()) {
                newRibbonDriverPoints[child.GetPath()] =
                    curves[0].IsPrimPath()
                        ? curves[0].AppendProperty(TfToken("points"))
                        : curves[0];
            }
        }
    }

    compileBlocks.Next("DiscoverValidate.MoverDiscovery");
    std::vector<RigExecMoverRecord> newMovers;
    SdfPathVector inertMovers;
    std::vector<_SurfaceProjectorRecord> newSurfaceProjectors;
    if (!_DiscoverMovers(newMovers, newSurfaceProjectors, inertMovers,
                         errors, failure)) {
        return fail(failure->message, failure->operations);
    }

    compileBlocks.Next("DiscoverValidate.OutputCheck");
    // A rig has to publish SOMETHING (the check the joint requirement used
    // to stand in for).
    // Controls, joints, volumes, and movers are the four ways it can: a control
    // publishes its posed frame for the synthesized viewport guide, a joint
    // publishes a frame whether or not anything moves it, a placed weight
    // volume publishes its falloff guide while it is being authored, and a
    // mover publishes whatever its targets are. Zero of all four is a rig that
    // evaluates to an empty generation every frame, which is far likelier to
    // be an authoring mistake -- a rig root pointed at the wrong prim -- than
    // an intent.
    // An inert mover is not an output, but it is evidence of intent: the rig
    // root found mover prims, they simply are not wired yet. Failing that is
    // the same mistake as failing the whole rig for one disconnected mover --
    // it makes the last wire you pull take the rig down. The error is for a
    // rig that found NOTHING, which is the misconfiguration it describes.
    if (newControlPaths.empty() && newJointPaths.empty() &&
        newVolumeWeightPaths.empty() &&
        newMovers.empty() && inertMovers.empty()) {
        return fail("Rig publishes no outputs: " + _rigPath.GetString() +
                    " has no RigExecControl, RigExecJoint, or placed volume "
                    "weight prims and no movers");
    }

    // Multiple writers of one target are an ordinary stack, not an error.
    // Their order is the reverse-sibling post-order walk of the FINAL COMPOSED
    // hierarchy above. UsdPrim::GetChildrenNames() returns the displayed
    // top-to-bottom order with any parent child-order instruction (reorder
    // nameChildren) already folded in; the stack consumes that order in
    // reverse so the bottom branch runs first. A reorder is a convenience for
    // redirecting that order, never a precondition for having one.
    // This deliberately does not reason about HOW the composed order arose --
    // which layer authored a sibling, which arc contributed it, whether a
    // reorder opinion exists. The compiler reads the final stage and nothing
    // else. An earlier revision rejected non-nested same-target writers that
    // lacked an authored reorder covering both branches; it demanded ceremony
    // (examples/13_ReadPhases.usda restated its own file order to satisfy it)
    // while not actually preventing the precedence surprises it cited, which
    // come from arc order and weak-side insertion rather than from a missing
    // reorder opinion.

    // final transform reads are legal only when every writer of that
    // provider precedes the reader in logical order (spec §4.2): a
    // frame mover with a later ordinal than a consuming matrix mover is
    // an unsatisfied final read.
    {
        std::map<SdfPath, int> lastFrameWriterOrdinal;
        for (const RigExecMoverRecord &m : newMovers) {
            if (!_IsFrameConstraintType(m.schemaType)) {
                continue;
            }
            for (const SdfPath &t : m.targets) {
                if (t.IsPrimPath()) {
                    lastFrameWriterOrdinal[t] = std::max(
                        lastFrameWriterOrdinal.count(t)
                            ? lastFrameWriterOrdinal[t] : -1,
                        m.ordinal);
                }
            }
        }
        for (const RigExecMoverRecord &m : newMovers) {
            const RigExecMoverHandler *transformHandler =
                RigExecFindMoverHandler(m.schemaType);
            const char *transformRel = transformHandler
                ? transformHandler->transformRelationship
                : nullptr;
            if (!transformRel) {
                continue;
            }
            const UsdPrim prim = _stage->GetPrimAtPath(m.moverPath);
            if (!prim) {
                continue;
            }
            if (RigExecPhaseForInput(prim, transformRel).kind !=
                RigExecReadPhaseKind::Final) {
                continue;
            }
            SdfPathVector transforms;
            if (UsdRelationship rel = prim.GetRelationship(TfToken(
                    transformRel))) {
                rel.GetTargets(&transforms);
            }
            if (transformHandler->spaceRelationship) {
                // Both spaces: the measuring one the handler names, and
                // rigExec:space, the rig's carry. They are read at the SAME
                // phase as the transform, so a final read of either carries
                // the same ordering obligation.
                for (const char *name : {transformHandler->spaceRelationship,
                                         "rigExec:space"}) {
                    SdfPathVector spaces;
                    if (UsdRelationship rel =
                            prim.GetRelationship(TfToken(name))) {
                        rel.GetTargets(&spaces);
                    }
                    transforms.insert(transforms.end(), spaces.begin(),
                                      spaces.end());
                }
            }
            for (const SdfPath &provider : transforms) {
                const auto it = lastFrameWriterOrdinal.find(provider);
                if (it != lastFrameWriterOrdinal.end() &&
                    it->second > m.ordinal) {
                    return fail(
                        "Unsatisfied final read: " +
                        m.moverPath.GetString() + " reads final of " +
                        provider.GetString() +
                        " but a writer with a later ordinal exists "
                        "(spec §4.2)", {m.moverPath});
                }
            }
        }
    }

    compileBlocks.Next("DiscoverValidate.SolverJointBinding");
    // View-free solver->joint binding validation. This
    // runs in Phase A, BEFORE any epoch teardown, so an invalid binding
    // rejects the compile while the previous epoch stays publishable (the
    // BLOCKER fix: compile Pass 0 must never be the first place a bad
    // binding is discovered, because by then the old layer is gone and
    // restoration would re-hit the same invalid state). Recursive over the
    // composed Solvers subtree. Authored-conflict checks read the SOURCE
    // stage (_stage).
    // joint -> (posing solver, element). The validation below already
    // resolves and bounds-checks exactly this pair; keeping it is what lets
    // Evaluate extract each bound joint's frame from its solver's aggregate
    // and supply it as a value override, so the binding never has to be
    // authored anywhere (it used to become rigExec:frameSource /
    // rigExec:frameElement on the joint, in the derived layer).
    // rigExec:joints is an ordered WRITE, not an exclusive claim (spec §4.2,
    // "Solvers stack"), so the value is the ordered stack of writers rather
    // than one owner. Index 0 writes first; the last entry supplies the
    // joint's base frame. The order is settled below, once the POSE graph is
    // complete -- which is later than the solver DAG, because a pair of
    // writers can be ordered by a solver -> constraint -> solver path and by
    // nothing else: data flow decides any pair it orders, and the solver
    // stack ordinal breaks every remaining tie.
    // The ordered solver walk is hoisted here because four passes need it --
    // this validation, the consumed-solver relaxation, the solver DAG and the
    // stack ordinal -- and it is a full UsdPrimRange over the rig each time
    // (critique: "_DiscoverAggregateSolvers is already walked three times").
    // It is now walked once, with the providers, ahead of the exec lane's
    // dispatch; nothing compile does authors to the stage after the derived
    // start frames, so that walk is still the stage's answer here.
    const std::vector<UsdPrim> &orderedSolvers = aggregateSolvers;
    // Solver -> its position in the SOLVER STACK ORDINAL: the reverse of the
    // composed pre-order of the whole rig, which is _GetMoverExecutionOrder's
    // rule (see the comment at its definition) applied to the solver set
    // instead of the movers. Bottom composed sibling first, a parent
    // after its descendants -- so "the bottom one executes first" reads the
    // same whichever kind of node a rigger is looking at, and a reorder in
    // usdview reorders the stack.
    // It is a pre-order of the WHOLE rig, not a sibling order: two solvers in
    // different scopes are ordered by where their scopes sit, and a nested
    // solver comes before its ancestor. This is deliberately the same walk
    // the epoch digest uses (see the aggregate-solver segment of the digest
    // below), so an order edit can never retain the old digest while
    // executing a different stack.
    std::map<SdfPath, int> solverStackOrdinal;
    {
        int ordinal = 0;
        for (auto it = orderedSolvers.rbegin(); it != orderedSolvers.rend();
             ++it) {
            solverStackOrdinal[it->GetPath()] = ordinal++;
        }
    }
    std::map<SdfPath, std::vector<std::pair<SdfPath, int>>> newJointBinding;
    {
        {
            const std::vector<UsdPrim> &solvers = orderedSolvers;
            // Cardinality attributes must be static (compile-time
            // structural) for EVERY aggregate solver under the rig, not
            // only joint-bearing ones: a non-joint Twist/Ribbon feeding a
            // joint-bearing Blend still determines that Blend's element
            // count, so a time-sampled cardinality would silently shift a
            // blend-bound joint's frame. `uniform` is only
            // a hint; reject samples explicitly.
            for (const UsdPrim &solver : solvers) {
                const TfToken t = solver.GetTypeName();
                // rigExec:upperLength/rigExec:lowerLength no longer exist
                // in the schema -- bone lengths are measured from the bound
                // joints' rests. An asset saved against the old schema can
                // still carry one as a custom property, where it would be
                // silently inert; that used to freeze the bone, so a rig
                // relying on it would change shape with no explanation.
                // Fail loudly instead and name the knob that replaced it.
                if (t == "RigExecTwoBoneIk") {
                    static const char *const lengthAttrs[2] = {
                        "rigExec:upperLength", "rigExec:lowerLength"};
                    for (const char *name : lengthAttrs) {
                        const UsdAttribute a =
                            solver.GetAttribute(TfToken(name));
                        if (a && a.HasAuthoredValueOpinion()) {
                            return fail(
                                solver.GetPath().GetString() + ": " + name +
                                " was removed from the schema; bone lengths "
                                "are measured from the bound joints' rest "
                                "positions. Remove this opinion and author " +
                                name + "Offset to adjust the measured bone", {solver.GetPath()});
                        }
                    }
                }
                std::vector<const char *> cardinalityAttrs;
                if (t == "RigExecTwistDistribution") {
                    cardinalityAttrs = {"rigExec:count", "rigExec:weights"};
                } else if (t == "RigExecRibbon") {
                    cardinalityAttrs = {"rigExec:sampleCount"};
                } else if (t == "RigExecSplineIk") {
                    // Not cardinality (the joints list is), but a static
                    // parallel array: a sampled one would let the
                    // per-joint weight silently detach from the chain.
                    cardinalityAttrs = {"rigExec:volumeWeights"};
                }
                for (const char *name : cardinalityAttrs) {
                    const UsdAttribute a = solver.GetAttribute(TfToken(name));
                    if (a && a.GetNumTimeSamples() > 0) {
                        return fail(
                            solver.GetTypeName().GetString() + " " +
                            solver.GetPath().GetString() + ": " + name +
                            " must not be time-sampled (it defines frame "
                            "cardinality)", {solver.GetPath()});
                    }
                }
            }
            // Solvers whose aggregate is READ by another solver. Such a
            // solver does not pose anything itself -- the consumer that
            // reads it is what writes to the joints -- so its
            // rigExec:joints is a rest reference, not an output claim.
            // That is what lets an IK feeding an IK/FK blend name the
            // chain it solves for: the blend still claims those joints
            // exclusively, while the IK gets the joint REST frames it
            // needs to measure its bone lengths from. Without this the
            // IK could not name them at all ("posed by two solvers") and
            // had no bones to measure.
            std::set<SdfPath> consumedSolvers;
            std::set<SdfPath> posedByUnconsumed;
            {
                const std::vector<UsdPrim> &allSolvers = orderedSolvers;
                std::set<SdfPath> solverPaths;
                for (const UsdPrim &solver : allSolvers) {
                    solverPaths.insert(solver.GetPath());
                }
                for (const UsdPrim &solver : allSolvers) {
                    for (const UsdRelationship &rel :
                         solver.GetRelationships()) {
                        if (rel.GetName() == "rigExec:joints") {
                            continue;
                        }
                        SdfPathVector targets;
                        rel.GetTargets(&targets);
                        for (const SdfPath &target : targets) {
                            // GetPrimPath(), not the raw target: the DAG pass
                            // below resolves a property spelling
                            // (</Rig/Solvers/IK.rigExec:aggregate>) to its
                            // prim, and a read that creates a dependency edge
                            // but does not mark the target consumed would
                            // leave the IK writing as well as feeding the
                            // blend -- a stack whose meaning then depended on
                            // namespace order.
                            const SdfPath targetPrim = target.GetPrimPath();
                            if (_IsSkippedOperation(targetPrim)) {
                                return fail(
                                    solver.GetPath().GetString() +
                                    " reads skipped operation " + targetPrim.GetString(),
                                    {solver.GetPath()});
                            }
                            if (targetPrim != solver.GetPath() &&
                                solverPaths.count(targetPrim)) {
                                consumedSolvers.insert(targetPrim);
                            }
                        }
                    }
                }
                // Joints that a solver nobody reads writes to. Only these
                // are already spoken for; a consumed solver still POSES
                // any joint no such solver names, so feeding a blend does
                // not silently stop it driving its own extra outputs.
                for (const UsdPrim &solver : allSolvers) {
                    if (consumedSolvers.count(solver.GetPath())) {
                        continue;
                    }
                    if (const UsdRelationship rel = solver.GetRelationship(
                            TfToken("rigExec:joints"))) {
                        SdfPathVector targets;
                        rel.GetTargets(&targets);
                        posedByUnconsumed.insert(targets.begin(),
                                                 targets.end());
                    }
                }
            }

            // A claim binds wherever the solver sits; a non-solver prim
            // carrying rigExec:joints is rejected wherever it sits.
            if (const UsdPrim rig = _stage->GetPrimAtPath(_rigPath)) {
                for (const UsdPrim &solver : UsdPrimRange(rig)) {
                    if (_IsSkippedOperation(solver.GetPath())) {
                        continue;
                    }
                    const UsdRelationship jointsRel =
                        solver.GetRelationship(TfToken("rigExec:joints"));
                    if (!jointsRel) {
                        continue;
                    }
                    SdfPathVector jointTargets;
                    jointsRel.GetTargets(&jointTargets);
                    if (jointTargets.empty()) {
                        continue;
                    }
                    const std::string who = solver.GetTypeName().GetString() +
                                            " " + solver.GetPath().GetString();

                    // The claimant must be a real aggregate solver (it must
                    // publish computePointFrameArray for the joint to extract).
                    // A Scope or arbitrary prim carrying rigExec:joints would
                    // otherwise pass, and the binding would name a prim with no
                    // aggregate computation to extract an element from.
                    if (!_IsAggregateSolverType(solver.GetTypeName())) {
                        return fail(who + " authors rigExec:joints but is not a "
                                          "recognized aggregate solver type", {solver.GetPath()});
                    }

                    // jointElements parallel-array shape: empty, or exactly one
                    // entry per joint (no partial remap / silent truncation).
                    VtIntArray jointElements;
                    if (const UsdAttribute a = solver.GetAttribute(
                            TfToken("rigExec:jointElements"))) {
                        std::vector<double> times;
                        if (a.GetTimeSamples(&times) && !times.empty()) {
                            return fail(who + ": rigExec:jointElements must not "
                                              "carry time samples", {solver.GetPath()});
                        }
                        a.Get(&jointElements);
                    }
                    if (!jointElements.empty() &&
                        jointElements.size() != jointTargets.size()) {
                        return fail(
                            who + ": rigExec:jointElements length " +
                            std::to_string(jointElements.size()) +
                            " must equal rigExec:joints length " +
                            std::to_string(jointTargets.size()), {solver.GetPath()});
                    }

                    // Knowable aggregate element count per solver type
                    // (-1 = not cheaply knowable, e.g. a blend of another
                    // aggregate: element bounds are enforced only at runtime).
                    int knownCount = -1;
                    const TfToken type = solver.GetTypeName();
                    if (type == "RigExecTwoBoneIk") {
                        knownCount = 3;
                    } else if (type == "RigExecFkChain") {
                        SdfPathVector controls;
                        if (const UsdRelationship c = solver.GetRelationship(
                                TfToken("rigExec:controls"))) {
                            c.GetTargets(&controls);
                        }
                        knownCount = static_cast<int>(controls.size());
                    } else if (type == "RigExecTwistDistribution") {
                        // count/weights time samples already rejected above.
                        VtFloatArray weights;
                        if (const UsdAttribute a = solver.GetAttribute(
                                TfToken("rigExec:weights"))) {
                            a.Get(&weights);
                        }
                        if (!weights.empty()) {
                            knownCount = static_cast<int>(weights.size());
                        } else if (const UsdAttribute a = solver.GetAttribute(
                                       TfToken("rigExec:count"))) {
                            int c = 1;
                            a.Get(&c);
                            knownCount = std::max(c, 1);
                        }
                    } else if (type == "RigExecRibbon") {
                        if (const UsdAttribute a = solver.GetAttribute(
                                TfToken("rigExec:sampleCount"))) {
                            int c = 5;
                            a.Get(&c);
                            // The transported-frame ribbon needs >= 2 samples;
                            // below that it produces no frames, so no element
                            // is bindable (matches the runtime cardinality).
                            knownCount = c >= 2 ? c : 0;
                        }
                    } else if (type == "RigExecSplineIk") {
                        // The chain IS the cardinality: one frame per
                        // joints entry. A remap must then be a permutation
                        // of the chain slots, and the per-joint volume
                        // weights must be parallel to it.
                        knownCount = static_cast<int>(jointTargets.size());
                        if (!jointElements.empty()) {
                            std::vector<bool> filled(jointTargets.size(), false);
                            for (int e : jointElements) {
                                if (e >= 0 && e < knownCount) {
                                    if (filled[e]) {
                                        return fail(
                                            who + ": rigExec:jointElements "
                                            "fills chain slot " +
                                            std::to_string(e) + " twice", {solver.GetPath()});
                                    }
                                    filled[e] = true;
                                }
                            }
                        }
                        if (const UsdAttribute a = solver.GetAttribute(
                                TfToken("rigExec:volumeWeights"))) {
                            VtFloatArray weights;
                            a.Get(&weights);
                            if (!weights.empty() &&
                                weights.size() != jointTargets.size()) {
                                return fail(
                                    who + ": rigExec:volumeWeights length " +
                                    std::to_string(weights.size()) +
                                    " must equal rigExec:joints length " +
                                    std::to_string(jointTargets.size()) +
                                    " (or be empty)", {solver.GetPath()});
                            }
                        }
                    }

                    for (size_t i = 0; i < jointTargets.size(); ++i) {
                        const SdfPath &jointPath = jointTargets[i];
                        const int element = i < jointElements.size()
                            ? jointElements[i] : static_cast<int>(i);
                        if (element < 0) {
                            return fail(who + ": negative element index " +
                                        std::to_string(element) + " for " +
                                        jointPath.GetString(), {solver.GetPath()});
                        }
                        if (knownCount >= 0 && element >= knownCount) {
                            return fail(
                                who + ": element " + std::to_string(element) +
                                " for " + jointPath.GetString() +
                                " is out of range (solver produces " +
                                std::to_string(knownCount) + " frames)", {solver.GetPath()});
                        }
                        const UsdPrim jointPrim =
                            _stage->GetPrimAtPath(jointPath);
                        if (!jointPrim) {
                            return fail(who + " rigExec:joints targets missing "
                                              "prim " + jointPath.GetString(), {solver.GetPath()});
                        }
                        if (jointPrim.GetTypeName() != "RigExecJoint") {
                            return fail(
                                who + " rigExec:joints target " +
                                jointPath.GetString() + " is a " +
                                jointPrim.GetTypeName().GetString() +
                                ", not a RigExecJoint", {solver.GetPath()});
                        }
                        // Every joint is tapped individually and a bound one also
                        // carries a per-prim value override, both of which key on
                        // a real prim; only the rig's nearest instanceable
                        // ancestor is deinstanced. Reject an instance-proxy /
                        // prototype-hosted joint up front.
                        if (jointPrim.IsInstanceProxy() ||
                            jointPrim.IsInPrototype()) {
                            return fail(
                                who + " rigExec:joints target " +
                                jointPath.GetString() +
                                " is instance-proxy/prototype hosted and cannot "
                                "receive a solver binding", {solver.GetPath()});
                        }
                        // Exclusive ownership: a solver-posed joint must not
                        // also author its own posed:space connection (the solver
                        // pose is supplied as an override and would silently win
                        // over the connection the raw stage shows).
                        // There is no longer a companion check for a legacy
                        // authored rigExec:frameSource. Nothing reads that name
                        // now -- it is neither a schema property nor a
                        // registered computation input -- so a leftover opinion
                        // from an asset saved against the old schema is inert,
                        // and failing the compile over it would reject a rig
                        // that evaluates correctly.
                        if (const UsdPrim srcJoint =
                                _stage->GetPrimAtPath(jointPath)) {
                            if (const UsdAttribute posed = srcJoint.GetAttribute(
                                    TfToken("posed:space"))) {
                                const SdfPathVector conns =
                                    _AuthoredConnections(posed);
                                if (!conns.empty()) {
                                    return fail(who + ": joint " +
                                                jointPath.GetString() +
                                                " also connects posed:space", {solver.GetPath()});
                                }
                            }
                        }
                        // A consumed solver naming a joint that a solver
                        // nobody reads already writes to is referencing it
                        // for its REST, not claiming it: the consumer is
                        // what poses it. Any other joint it names it still
                        // poses itself.
                        if (consumedSolvers.count(solver.GetPath()) &&
                            posedByUnconsumed.count(jointPath)) {
                            continue;
                        }
                        // Two SOLVERS writing one joint is legal and stacks.
                        // One solver naming one joint TWICE is not: the two
                        // entries would collapse at runtime (candidates[] is
                        // a map, and the baked commit's slots are uniqued),
                        // and a stack edge between them would be a self-edge
                        // that Kahn reports as an unexplained pose cycle. Say
                        // what actually happened instead.
                        std::vector<std::pair<SdfPath, int>> &writers =
                            newJointBinding[jointPath];
                        for (const auto &[writer, writerElement] : writers) {
                            if (writer == solver.GetPath()) {
                                return fail(
                                    who + ": rigExec:joints names " +
                                    jointPath.GetString() +
                                    " more than once", {solver.GetPath()});
                            }
                        }
                        writers.emplace_back(solver.GetPath(), element);
                    }
                }
            }
        }
    }

    // The binding loop appends in UsdPrimRange pre-order, which is the exact
    // REVERSE of the stack, so every writer list is put into stack-ordinal
    // order here. This is the AUTHORED order, and it is not decoration: the
    // "reads the version standing before its own commit" edges built while
    // the solver DAG is assembled are read off these lists. The pass that
    // settles the stack against the finished pose graph replaces the order
    // with the one data flow and the schedule actually produce, and the two
    // agree whenever nothing orders a pair.
    for (auto &[joint, writers] : newJointBinding) {
        if (writers.size() < 2) {
            continue;  // a one-element stack has no order to get wrong
        }
        // find(), never operator[]: mutating a captured map from inside a
        // sort comparator would insert a silent 0 for any writer the ordinal
        // sweep did not see and make the comparator inconsistent. An unknown
        // writer sorts last instead.
        const auto ordinalOf = [&solverStackOrdinal](const SdfPath &path) {
            const auto it = solverStackOrdinal.find(path);
            return it == solverStackOrdinal.end()
                       ? std::numeric_limits<int>::max()
                       : it->second;
        };
        std::stable_sort(
            writers.begin(), writers.end(),
            [&ordinalOf](const std::pair<SdfPath, int> &a,
                         const std::pair<SdfPath, int> &b) {
                return ordinalOf(a.first) < ordinalOf(b.first);
            });
    }

    compileBlocks.Next("DiscoverValidate.SolverDag");
    // Compile the solver DAG, including frame inputs carried by a posed
    // namespace ancestor. Rest inputs are independent of posed outputs and
    // therefore never introduce a feedback edge (IK -> blend is legal).
    // PRODUCERS: the solvers with no position in the pose stack at all
    // (spec §4.2). Two kinds, and both are scheduled by DATA FLOW alone:
    //   * a solver whose AGGREGATE another solver reads. It is already
    //     special -- the consumed-solver relaxation makes its rigExec:joints
    //     a rest reference wherever the consumer claims the same joint -- and
    //     its consumer cannot read an "earlier version" of an aggregate, so
    //     the aggregate edge decides the pair and the namespace says nothing
    //     about it. This is what keeps every IK/FK blend in the repo
    //     schedulable: reversed sibling order puts the blend BEFORE the
    //     solvers it blends, because a blend is authored last.
    //     (The relaxation is per JOINT, not per solver, so a consumed solver
    //     that also writes a joint its consumer does not name still writes --
    //     `docs/examples/blend_point_frames.usda` is exactly that shape. It
    //     is still a producer: what takes it out of the stack is being read,
    //     not being joint-less.)
    //   * a solver that writes no joint at all -- one whose aggregate only a
    //     geometry mover reads.
    // Every consumer of the ordinal map must therefore test membership rather
    // than use operator[].
    std::set<SdfPath> aggregateProducers;
    {
        std::set<SdfPath> solverPaths;
        for (const UsdPrim &solver : orderedSolvers) {
            solverPaths.insert(solver.GetPath());
        }
        for (const UsdPrim &solver : orderedSolvers) {
            for (const UsdRelationship &rel : solver.GetRelationships()) {
                if (rel.GetName() == "rigExec:joints") continue;
                SdfPathVector targets;
                rel.GetTargets(&targets);
                for (const SdfPath &target : targets) {
                    const SdfPath prim = target.GetPrimPath();
                    if (prim != solver.GetPath() && solverPaths.count(prim)) {
                        aggregateProducers.insert(prim);
                    }
                }
            }
        }
    }
    std::set<SdfPath> jointWritingSolvers;
    for (const auto &[joint, writers] : newJointBinding) {
        for (const auto &[writer, element] : writers) {
            if (aggregateProducers.count(writer)) continue;
            jointWritingSolvers.insert(writer);
        }
    }
    // Strictly before, in the stack restricted to solvers. solverStackOrdinal
    // is the reverse composed pre-order of the whole rig, so its restriction
    // to the solvers IS the unified pose stack restricted to them; the
    // constraint half only interleaves between them and cannot reorder a
    // solver pair.
    // Only a READER that is itself a stack step consults it. A PRODUCER has no
    // position for the comparison to be about, so it keeps the unconditional
    // "wait for every writer of what I read" edge it always had -- which is
    // its data-flow meaning, and which leaves a genuine loop among producers
    // (two of them reading each other's joints) a cycle, reported as one.
    const auto stackBefore = [&solverStackOrdinal](const SdfPath &a,
                                                   const SdfPath &b) {
        const auto ia = solverStackOrdinal.find(a);
        const auto ib = solverStackOrdinal.find(b);
        if (ia == solverStackOrdinal.end() ||
            ib == solverStackOrdinal.end()) {
            return false;
        }
        return ia->second < ib->second;
    };
    // "A writer waits on an EARLIER READER." The new edge class the unified
    // pose stack needs (spec §4.2): a step that reads a provider from BELOW
    // its writer reads the version standing before that write, and without an
    // edge saying so Kahn may schedule the two either way round within a level
    // and the version the reader sees becomes undefined -- a nondeterministic
    // failure, which is the worst kind here. Collected wherever a read is
    // resolved and applied once poseDependencies exists, because the two
    // halves are found on opposite sides of the constraint pass.
    // (waiter, waited-on): poseDependencies[first].insert(second).
    std::vector<std::pair<SdfPath, SdfPath>> poseReverseEdges;
    std::map<SdfPath, std::set<SdfPath>> newSolverDependencies;
    // Aggregate reads bind a producer's same-frame output. Pose frame reads
    // select authored checkpoints; the common graph schedules both classes.
    std::map<SdfPath, std::set<SdfPath>> newSolverAggregateReads;
    {
        const std::vector<UsdPrim> &solvers = orderedSolvers;
        std::set<SdfPath> solverPaths;
        for (const UsdPrim &solver : solvers) {
            solverPaths.insert(solver.GetPath());
            newSolverDependencies[solver.GetPath()];
        }
        for (const UsdPrim &solver : solvers) {
            for (const UsdRelationship &rel : solver.GetRelationships()) {
                if (rel.GetName() == "rigExec:joints") {
                    continue;
                }
                SdfPathVector targets;
                rel.GetTargets(&targets);
                for (const SdfPath &target : targets) {
                    const SdfPath targetPrim = target.GetPrimPath();
                    if (solverPaths.count(targetPrim)) {
                        newSolverDependencies[solver.GetPath()].insert(targetPrim);
                        newSolverAggregateReads[solver.GetPath()].insert(
                            targetPrim);
                        continue;
                    }
                    const UsdPrim provider = _stage->GetPrimAtPath(targetPrim);
                    const bool frameProvider = _IsFrameProvider(provider);
                    for (SdfPath path = targetPrim; !path.IsEmpty() &&
                         path != SdfPath::AbsoluteRootPath();
                         path = frameProvider ? path.GetParentPath() : SdfPath()) {
                        const auto binding = newJointBinding.find(path);
                        if (binding != newJointBinding.end()) {
                            // Every writer of this joint that precedes this
                            // solver in the stack, and NEVER this solver
                            // itself: a solver that reads a joint it also
                            // writes reads the version standing before its
                            // own commit, so it depends on the writers below
                            // it and on nothing above. (Before stacking this
                            // was a self-edge and died as a bogus "solver
                            // dependency cycle".)
                            for (const auto &[writer, writerElement] :
                                 binding->second) {
                                if (writer == solver.GetPath()) {
                                    continue;
                                }
                                if (jointWritingSolvers.count(
                                        solver.GetPath()) &&
                                    stackBefore(solver.GetPath(), writer)) {
                                    // The writer stands ABOVE this solver, so
                                    // the solver reads the version before that
                                    // write -- and the writer has to wait, or
                                    // which version it read is undefined.
                                    poseReverseEdges.emplace_back(
                                        writer, solver.GetPath());
                                    continue;
                                }
                                newSolverDependencies[solver.GetPath()].insert(
                                    writer);
                            }
                            // A solver override replaces the whole joint
                            // callback, so that joint never reads its own
                            // namespace-parent posed frame.
                            break;
                        }
                    }
                }
            }
        }

    }

    compileBlocks.Close();
    stampCompileRegion("Compile.DiscoverValidate");

    compileBlocks.Next("SolverSchedule.ReplacementRequests");
    // Prepare replacement requests while retaining the previous requests and
    // their shared compiler context. The stock network handles changed USD
    // bindings incrementally; no whole execution-system replacement is needed.

    invalidateEpoch = [&]() { _compiled = false; };

    compileBlocks.Next("SolverSchedule.VolumeWeights");
    // Volumetric weight epoch state (spec §4.1 volumetric extension).
    std::vector<RigExecValueOverride> newFalloffLutOverrides;
    std::set<SdfPath> newCurrentPhaseWeights;
    std::set<SdfPath> newVolumeWeightProviders;

    // Walks a weight object and everything it composes, gathering what
    // the volumetric types need beyond their computeWeightPacket tap.
    // Depth-limited for the same reason the structure digest is: a cycle
    // is authoring error, and the bound only has to keep this
    // terminating.
    // Structural authoring errors on a volume weight, collected during
    // the walk below and reported before the epoch commits.
    // These are cardinality rules on the points-bearing relationships,
    // and they exist because the two evaluation paths CANNOT disagree
    // about them safely: the exec kernel receives a relationship's
    // targets as one flattened value stream, so two targets on
    // rigExec:curve silently concatenate into one polyline with a
    // spurious segment joining them, while the CPU oracle reads targets
    // explicitly and rejects the pair. Catching it here means neither
    // path ever sees the ambiguous authoring.
    std::string volumeWeightError;

    // Descriptor discovery terminates recursive composition here. The common
    // operation graph retains producer edges and diagnoses actual cycles.
    std::set<SdfPath> visiting;

    // Returns true when this weight object, or anything it composes,
    // samples the in-flight points.
    // The answer has to propagate UP: the graph build loop tests the
    // weight object a mover actually binds, which for a composed field is
    // the combine, not the sphere inside it. Recording only the leaf left
    // exec applying the reference-phase packet while the CPU oracle
    // reached a leaf with no in-flight points and failed.
    std::function<bool(const SdfPath &)> registerVolumeWeights =
        [&](const SdfPath &weightPath) -> bool {
        const UsdPrim w = _stage->GetPrimAtPath(weightPath);
        if (!w) {
            return false;
        }
        if (!visiting.insert(weightPath).second) return false;
        struct Pop {
            std::set<SdfPath> &s;
            const SdfPath &p;
            ~Pop() { s.erase(p); }
        } pop{visiting, weightPath};

        if (newVolumeWeightProviders.count(weightPath) ||
            newCurrentPhaseWeights.count(weightPath)) {
            // Already walked through another consumer; its answer stands.
            return newCurrentPhaseWeights.count(weightPath) != 0;
        }
        bool isCurrent = false;
        if (_IsVolumeWeightType(w.GetTypeName())) {
            auto requireTargets = [&](const char *rel, size_t exact,
                                      const char *what) {
                SdfPathVector targets;
                if (UsdRelationship r = w.GetRelationship(TfToken(rel))) {
                    r.GetTargets(&targets);
                }
                if (targets.size() > exact) {
                    volumeWeightError =
                        weightPath.GetString() + ": " + rel + " must name " +
                        what;
                }
                return targets.size();
            };
            // At most one sampling override; exactly one curve for a
            // curve weight.
            requireTargets("rigExec:sampleSource", 1,
                           "at most one points source");
            if (w.GetTypeName() == "RigExecCurveWeight") {
                SdfPathVector curves;
                if (const UsdRelationship rel = w.GetRelationship(
                        TfToken("rigExec:curve"))) {
                    rel.GetTargets(&curves);
                }
                if (curves.size() != 1) {
                    volumeWeightError =
                        weightPath.GetString() +
                        ": rigExec:curve must name exactly one points source";
                } else {
                    const SdfPath pointsPath = curves[0].IsPropertyPath()
                        ? curves[0]
                        : curves[0].AppendProperty(TfToken("points"));
                    const UsdAttribute points =
                        _stage->GetAttributeAtPath(pointsPath);
                    if (!points ||
                        points.GetTypeName() != SdfValueTypeNames->Point3fArray) {
                        volumeWeightError =
                            weightPath.GetString() +
                            ": rigExec:curve target " +
                            curves[0].GetString() +
                            " must resolve to a point3f[] points source";
                    }
                }
            }
        }
        if (_IsVolumeWeightType(w.GetTypeName())) {
            // computePointFrame, NOT computeMatrix: the latter is the
            // rest->posed map, so an unanimated volume's is the identity
            // and its field would land at the origin however the prim is
            // placed. See _RigidWorldToLocal in moverKernels.cpp.
            newVolumeWeightProviders.insert(weightPath);

            RigExecValueOverride lutOverride;
            lutOverride.prim = weightPath;
            lutOverride.computation = _computeFalloffLut;
            RigExecFalloffLut lut;
            lut.samples = _BakeFalloffLut(w);
            lutOverride.value = VtValue(lut);
            newFalloffLutOverrides.push_back(std::move(lutOverride));

            // Which points the field measures is the read phase declared
            // on rigExec:weightTarget; the old attribute is refused rather
            // than composing as an inert custom attribute.
            bool inFlight = false;
            std::string phaseError;
            const UsdAttribute old = w.GetAttribute(_removedSamplePhase);
            if (old && old.HasAuthoredValue()) {
                volumeWeightError =
                    weightPath.GetString() +
                    " authors rigExec:samplePhase, which was replaced by "
                    "rigExecReadPhase metadata on rigExec:weightTarget "
                    "(reference is base, current is preceding)";
            } else if (!_VolumeWeightSamplesInFlight(w, &inFlight,
                                                     &phaseError)) {
                volumeWeightError = phaseError;
            } else if (inFlight) {
                newCurrentPhaseWeights.insert(weightPath);
                isCurrent = true;
            }
        }
        for (const char *rel :
             {"rigExec:inputWeights", "rigExec:baseWeight"}) {
            SdfPathVector targets;
            if (UsdRelationship r = w.GetRelationship(TfToken(rel))) {
                r.GetTargets(&targets);
            }
            for (const SdfPath &t : targets) {
                // Not short-circuited: every reachable weight object
                // still needs its matrix tap and LUT override, so the
                // walk must complete even once the answer is known.
                if (registerVolumeWeights(t)) {
                    isCurrent = true;
                }
            }
        }
        // A composed field is current-phase if anything inside it is, so
        // that the combine a mover actually binds tests true.
        if (isCurrent) {
            newCurrentPhaseWeights.insert(weightPath);
        }
        return isCurrent;
    };

    // A placed volume is a guide output even before it is bound to a mover.
    // Register every discovered volume first; the recursive consumer walks
    // below naturally deduplicate against this map. This also applies the same
    // CurveWeight cardinality validation to standalone and consumed volumes.
    for (const SdfPath &volumePath : newVolumeWeightPaths) {
        registerVolumeWeights(volumePath);
    }

    // Gather volumetric epoch state for every common mover envelope, not only
    // point-graph revisions. Geometry-domain constraints publish directly
    // after the pose walk and therefore never appear in _graphChains, but a
    // sphere/plane/curve field bound to one still needs the same placement tap
    // and falloff override as a geometry mover.
    for (const RigExecMoverRecord &mover : newMovers) {
        const UsdPrim moverPrim = _stage->GetPrimAtPath(mover.moverPath);
        if (!moverPrim) {
            continue;
        }
        SdfPathVector objects;
        if (const UsdRelationship rel = moverPrim.GetRelationship(
                TfToken("rigExec:weightObject"))) {
            rel.GetTargets(&objects);
        }
        if (objects.size() == 1) {
            registerVolumeWeights(objects[0]);
        }
    }


    compileBlocks.Next("SolverSchedule.PoseConstraints");
    // Pose-domain constraints, compiled to in-memory structural wiring. Aim,
    // Position, Rotation, Scale, and Parent revise one transform provider;
    // SingleChainIK revises its inferred joint chain atomically. Values stay
    // authored on the mover and are sampled during Evaluate().
    std::vector<_FrameConstraint> newFrameConstraints;
    std::map<SdfPath, std::vector<SdfPath>> newFrameChains;

    auto getTargets = [](const UsdPrim &prim, const char *name) {
        SdfPathVector paths;
        if (const UsdRelationship rel =
                prim.GetRelationship(TfToken(name))) {
            rel.GetTargets(&paths);
        }
        return paths;
    };

    auto bindFrameSource = [&](const SdfPath &operation,
                               const SdfPath &authored,
                               const std::string &role,
                               _FrameSourceBinding *binding) {
        if (!authored.IsPrimPath()) {
            return fail(role + " must target a prim, got " +
                        authored.GetString(), {operation});
        }
        const UsdPrim sourcePrim = _stage->GetPrimAtPath(authored);
        if (!sourcePrim) {
            return fail(role + " targets missing prim " +
                        authored.GetString(), {operation});
        }
        binding->sourcePath = authored;
        const TfToken sourceType = sourcePrim.GetTypeName();
        if (sourceType == "RigExecControl" || sourceType == "RigExecJoint") {
            return true;
        }
        if (UsdGeomXformable(sourcePrim)) {
            binding->xformPath = authored;
            return true;
        }
        return fail(role + " targets " + authored.GetString() +
                    ", which is neither a RigExec transform provider nor "
                    "a UsdGeomXformable", {operation});
    };

    // Every property-domain target a math mover revises. A mask attribute in
    // this set is not static: its live read must still see the chain's
    // result, so a constraint whose mask lands here keeps masksStatic false.
    std::set<SdfPath> propertyRevisedTargets;
    for (const RigExecMoverRecord &mover : newMovers) {
        if (!RigExecIsPropertyMover(mover.schemaType)) {
            continue;
        }
        for (const SdfPath &target : mover.targets) {
            propertyRevisedTargets.insert(target);
        }
    }
    for (const RigExecMoverRecord &mover : newMovers) {
        // A matrix mover in the TRANSFORM domain is frame work. It deforms a
        // frame, and a frame is what the pose walk orders, what later movers
        // observe, and what gets published -- none of which the point graph
        // does. Its geometry-domain twin is unaffected and stays there.
        const bool transformDomainMatrix =
            mover.schemaType == "RigExecMatrixMover" &&
            mover.targets.size() == 1 && mover.targets[0].IsPrimPath() &&
            !rigExec::RigExecIsTransformDomainAmbiguous(_stage, mover.targets[0]);
        if (!_IsFrameConstraintType(mover.schemaType) &&
            !transformDomainMatrix) {
            continue;
        }
        const UsdPrim moverPrim = _stage->GetPrimAtPath(mover.moverPath);
        if (!moverPrim) {
            continue;
        }
        _FrameConstraint constraint;
        constraint.moverPath = mover.moverPath;
        constraint.schemaType = mover.schemaType;
        const SdfPathVector commonWeights =
            getTargets(moverPrim, "rigExec:weightObject");
        if (!commonWeights.empty()) {
            // Phase-A validation already established at-most-one and the
            // exact domain: the moved prim for a source constraint, or this
            // mover prim for an atomic multi-target SingleChainIK.
            constraint.weightObject = commonWeights[0];
        }

        if (transformDomainMatrix) {
            // T = M(transform) * inverse(M(transformSpace)), the same pair
            // the geometry domain reads, carried here as ordered sources so
            // the existing binding and phase machinery resolves them.
            constraint.targets = mover.targets;
            {
                TfToken blend("linear");
                if (const UsdAttribute a = moverPrim.GetAttribute(
                        TfToken("rigExec:weightBlend"))) {
                    a.Get(&blend);
                }
                constraint.radialBlend = blend == "radial";
            }
            const SdfPathVector xf = getTargets(moverPrim, "rigExec:transform");
            if (xf.size() != 1) {
                return fail(mover.moverPath.GetString() +
                            ": a transform-domain MatrixMover needs exactly "
                            "one rigExec:transform", {mover.moverPath});
            }
            const SdfPathVector space =
                getTargets(moverPrim, "rigExec:transformSpace");
            for (const SdfPath &one : {xf[0]}) {
                _FrameSourceBinding binding;
                if (!bindFrameSource(mover.moverPath, one,
                                     mover.moverPath.GetString() +
                                         " rigExec:transform", &binding)) {
                    return false;
                }
                constraint.sources.push_back(binding);
            }
            for (const SdfPath &one : space) {
                _FrameSourceBinding binding;
                if (!bindFrameSource(mover.moverPath, one,
                                     mover.moverPath.GetString() +
                                         " rigExec:transformSpace",
                                     &binding)) {
                    return false;
                }
                constraint.sources.push_back(binding);
                break;
            }
        } else if (_IsSourceFrameConstraintType(mover.schemaType)) {
            constraint.targets = mover.targets;

            // Domain selection, by the authored spelling alone. A bare prim
            // path is the transform domain; <prim>.points is the geometry
            // domain. Either way the frame key is the PRIM -- the solve is
            // identical and only the publish differs -- so the points
            // property is carried aside and targets[0] is normalized.
            if (!constraint.targets.empty() &&
                constraint.targets[0].IsPropertyPath() &&
                constraint.targets[0].GetNameToken() == "points") {
                constraint.pointsTarget = constraint.targets[0];
                constraint.targets[0] = constraint.targets[0].GetPrimPath();
            }

            SdfPathVector sources = getTargets(moverPrim, "rigExec:sources");
            if (sources.empty() && mover.schemaType ==
                                       "RigExecAimConstraint") {
                // Backward-compatible spelling used by every existing Aim
                // asset. New assets use the ordered FBX-style sources list.
                sources = getTargets(moverPrim, "rigExec:aimTarget");
            }
            if (sources.empty()) {
                return fail(mover.schemaType.GetString() + " " +
                            mover.moverPath.GetString() +
                            " has no constraint sources", {mover.moverPath});
            }
            for (const SdfPath &source : sources) {
                _FrameSourceBinding binding;
                if (!bindFrameSource(mover.moverPath,
                        source,
                        mover.schemaType.GetString() + " " +
                            mover.moverPath.GetString() + " source",
                        &binding)) {
                    return false;
                }
                constraint.sources.push_back(binding);
            }

            // The two operators that read Euler components of their
            // sources -- a mask on one, a weighted mean over several --
            // and so the two whose answer turns with an outer rotation.
            if (mover.schemaType == "RigExecRotationConstraint" ||
                mover.schemaType == "RigExecParentConstraint") {
                const SdfPathVector space =
                    getTargets(moverPrim, "rigExec:space");
                if (space.size() > 1) {
                    return fail(mover.schemaType.GetString() + " " +
                                mover.moverPath.GetString() +
                                " has more than one rigExec:space",
                                {mover.moverPath});
                }
                // A space that is not a frame provider is dropped rather
                // than reported, exactly as the space switch drops it: "no
                // space" and "a space that cannot supply a frame" mean the
                // same thing here, and both leave the mask running as it
                // did before the masters.
                if (!space.empty()) {
                    const UsdPrim spacePrim = _stage->GetPrimAtPath(space[0]);
                    const TfToken spaceType =
                        spacePrim ? spacePrim.GetTypeName() : TfToken();
                    if (spaceType == "RigExecControl" ||
                        spaceType == "RigExecJoint") {
                        constraint.spacePath = space[0];
                    }
                }
            }

            // Opt-in behaviours that change an operator's arithmetic. Both
            // are uniform, so they are compiled once and hashed by the digest.
            const auto readUniformBool = [&moverPrim](const char *name) {
                bool value = false;
                if (const UsdAttribute a =
                        moverPrim.GetAttribute(TfToken(name))) {
                    a.Get(&value);
                }
                return value;
            };
            if (mover.schemaType == "RigExecScaleConstraint" ||
                mover.schemaType == "RigExecParentConstraint") {
                constraint.blendShear = readUniformBool("rigExec:blendShear");
            }
            if (mover.schemaType == "RigExecAimConstraint") {
                constraint.worldUpRotationOnly =
                    readUniformBool("rigExec:worldUpRotationOnly");
            }

            if (mover.schemaType == "RigExecAimConstraint") {
                const SdfPathVector upObjects =
                    getTargets(moverPrim, "rigExec:worldUpObject");
                if (upObjects.size() > 1) {
                    return fail("RigExecAimConstraint " +
                                mover.moverPath.GetString() +
                                " has more than one world-up object", {mover.moverPath});
                }
                if (!upObjects.empty() &&
                    !bindFrameSource(mover.moverPath,
                        upObjects[0],
                        "RigExecAimConstraint " +
                            mover.moverPath.GetString() + " world-up object",
                        &constraint.worldUpObject)) {
                    return false;
                }
            }
        } else {
            // FBX SingleChainIK names endpoints, not an ordered output list.
            // RigExec joint hierarchy is namespace nesting, so the exact
            // chain is inferred by walking End Joint's ancestors to First
            // Joint. rigExec:moves must declare that complete set.
            const SdfPathVector first =
                getTargets(moverPrim, "rigExec:firstJoint");
            const SdfPathVector end =
                getTargets(moverPrim, "rigExec:endJoint");
            const SdfPathVector effector =
                getTargets(moverPrim, "rigExec:effector");
            if (first.size() != 1 || end.size() != 1 ||
                effector.size() != 1 || !first[0].IsPrimPath() ||
                !end[0].IsPrimPath()) {
                return fail("RigExecSingleChainIkConstraint " +
                            mover.moverPath.GetString() +
                            " requires exactly one firstJoint, endJoint, "
                            "and effector prim", {mover.moverPath});
            }
            SdfPath cursor = end[0];
            while (!cursor.IsEmpty() && cursor != SdfPath::AbsoluteRootPath()) {
                const UsdPrim joint = _stage->GetPrimAtPath(cursor);
                if (!joint || joint.GetTypeName() != "RigExecJoint") {
                    return fail("RigExecSingleChainIkConstraint " +
                                mover.moverPath.GetString() +
                                " endpoint ancestry contains non-joint " +
                                cursor.GetString(), {mover.moverPath});
                }
                constraint.ikChain.push_back(cursor);
                if (cursor == first[0]) {
                    break;
                }
                cursor = cursor.GetParentPath();
            }
            if (constraint.ikChain.empty() ||
                constraint.ikChain.back() != first[0]) {
                return fail("RigExecSingleChainIkConstraint " +
                            mover.moverPath.GetString() + ": endJoint " +
                            end[0].GetString() +
                            " is not a namespace descendant of firstJoint " +
                            first[0].GetString(), {mover.moverPath});
            }
            std::reverse(constraint.ikChain.begin(),
                         constraint.ikChain.end());
            if (constraint.ikChain.size() < 2) {
                return fail("RigExecSingleChainIkConstraint " +
                            mover.moverPath.GetString() +
                            " needs at least two joints", {mover.moverPath});
            }
            std::set<SdfPath> declared(mover.targets.begin(),
                                       mover.targets.end());
            std::set<SdfPath> inferred(constraint.ikChain.begin(),
                                       constraint.ikChain.end());
            if (declared != inferred) {
                return fail("RigExecSingleChainIkConstraint " +
                            mover.moverPath.GetString() +
                            " rigExec:moves must equal the complete inferred "
                            "firstJoint-to-endJoint chain", {mover.moverPath});
            }
            constraint.targets = constraint.ikChain;
            if (!bindFrameSource(mover.moverPath,
                    effector[0],
                    "RigExecSingleChainIkConstraint " +
                        mover.moverPath.GetString() + " effector",
                    &constraint.effector)) {
                return false;
            }
            // SingleChain deliberately ignores every pole input. Do not even
            // bind these relationships: an otherwise malformed dormant pole
            // must not make the selected solver mode fail to compile.
            TfToken solverMode("rotatePlane");
            if (const UsdAttribute a = moverPrim.GetAttribute(
                    TfToken("rigExec:solverMode"))) {
                a.Get(&solverMode);
            }
            if (solverMode == "rotatePlane") {
                for (const SdfPath &pole :
                     getTargets(moverPrim, "rigExec:poleVectorObjects")) {
                    _FrameSourceBinding binding;
                    if (!bindFrameSource(mover.moverPath,
                            pole,
                            "RigExecSingleChainIkConstraint " +
                                mover.moverPath.GetString() +
                                " pole-vector object",
                            &binding)) {
                        return false;
                    }
                    constraint.poleObjects.push_back(binding);
                }
            }
        }
        // Precompute the axis masks when provably static for this mover: no
        // property chain revises a mask attribute, none is connected, and each
        // is a single authored opinion. Fallbacks mirror the live read:
        // translation/rotation default on, scale defaults off (FBX).
        {
            const char *kMaskNames[3][3] = {
                {"inputs:affectTranslationX", "inputs:affectTranslationY",
                 "inputs:affectTranslationZ"},
                {"inputs:affectRotationX", "inputs:affectRotationY",
                 "inputs:affectRotationZ"},
                {"inputs:affectScaleX", "inputs:affectScaleY",
                 "inputs:affectScaleZ"}};
            const bool kMaskFallback[3] = {true, true, false};
            auto groupStatic = [&](int group) {
                for (int axis = 0; axis < 3; ++axis) {
                    const char *name = kMaskNames[group][axis];
                    // A property path, not a child path: the names carry the
                    // inputs: namespace, which is not path syntax, and the
                    // revised targets are property paths. AppendPath here
                    // builds an invalid path that can never match, silently
                    // freezing every chain-driven mask at its epoch value.
                    if (propertyRevisedTargets.count(
                            constraint.moverPath.AppendProperty(
                                TfToken(name)))) {
                        return false;
                    }
                    const UsdAttribute a = moverPrim.GetAttribute(TfToken(name));
                    if (a) {
                        if (!_AuthoredConnections(a).empty()) {
                            return false;  // connected -> driven
                        }
                        std::vector<double> timeSamples;
                        // GetTimeSamples answers success, not a count: the
                        // comparison must be against the samples it filled.
                        if (a.GetTimeSamples(&timeSamples) &&
                            timeSamples.size() > 1) {
                            return false;  // animated
                        }
                    }
                    // absent -> schema default -> static
                }
                return true;
            };
            constraint.masksStatic =
                groupStatic(0) && groupStatic(1) && groupStatic(2);
            if (constraint.masksStatic) {
                auto readGroup = [&](int group) {
                    RigExecConstraintAxisMask m;
                    bool *out = &m.x;
                    for (int axis = 0; axis < 3; ++axis, ++out) {
                        bool v = kMaskFallback[group];
                        const UsdAttribute a =
                            moverPrim.GetAttribute(TfToken(kMaskNames[group][axis]));
                        if (a) {
                            a.Get(&v);
                        }
                        *out = v;
                    }
                    return m;
                };
                constraint.precompTranslation = readGroup(0);
                constraint.precompRotation = readGroup(1);
                constraint.precompScale = readGroup(2);
            }
        }

        for (const SdfPath &target : constraint.targets) {
            newFrameChains[target].push_back(mover.moverPath);
        }
        newFrameConstraints.push_back(std::move(constraint));
    }
    // One order over two kinds of step that used to live in two phases:
    //   * an aggregate solver that WRITES at least one joint, and
    //   * a pose-domain frame constraint (one that moves a transform provider
    //     rather than points).
    // The order is the reverse of the composed pre-order of the WHOLE RIG --
    // the bottom composed sibling first, a parent after its descendants, the
    // rule _GetMoverExecutionOrder already gives movers -- and NOTHING else
    // breaks a tie. A constraint below a solver therefore runs BEFORE it and
    // feeds it (the incoming frame becomes that solver's rest reference); a
    // constraint above it revises its output, which is what every shipped rig
    // authors and why they are unchanged.
    // A PRODUCER is deliberately ABSENT from this map (see
    // jointWritingSolvers above for what makes one), so every consumer of the
    // map must test membership rather than use operator[].
    std::map<SdfPath, int> poseStackOrdinal;
    {
        std::set<SdfPath> stackSteps = jointWritingSolvers;
        for (const _FrameConstraint &constraint : newFrameConstraints) {
            if (constraint.pointsTarget.IsEmpty()) {
                stackSteps.insert(constraint.moverPath);
            }
        }
        if (!stackSteps.empty()) {
            int ordinal = 0;
            for (const UsdPrim &prim :
                 _GetPoseStackOrder(_stage->GetPrimAtPath(_rigPath))) {
                if (stackSteps.count(prim.GetPath())) {
                    poseStackOrdinal[prim.GetPath()] = ordinal++;
                }
            }
        }
    }
    // Ordinal or "no position": a producer sorts before every stack step and
    // is deterministic about it, which is all the emission order needs.
    const auto stackOrdinalOf = [&poseStackOrdinal](const SdfPath &path) {
        const auto it = poseStackOrdinal.find(path);
        return it == poseStackOrdinal.end() ? -1 : it->second;
    };
    // A startFrame read is positional (spec §4.2): a chain ordered before
    // its provider's writer reads the pre-write frame, and for a rest
    // provider that is the rest pose -- the chain then hangs from nothing
    // and stays frozen with no other diagnostic. Measured on the biped
    // hand: finger chains below the arm blend read the rest wrist (0.0000
    // on every joint). Authored and derived targets alike -- the read
    // does not know which -- so this runs over composed targets.
    for (const UsdPrim &solver : orderedSolvers) {
        if (solver.GetTypeName() != "RigExecFkChain") {
            continue;
        }
        static const TfToken startFrameRelTok("rigExec:startFrame");
        SdfPathVector starts;
        solver.GetRelationship(startFrameRelTok).GetTargets(&starts);
        if (starts.empty()) {
            continue;
        }
        const int here = stackOrdinalOf(solver.GetPath());
        if (here < 0) {
            continue;  // Guide-only: exec re-derives the order.
        }
        const SdfPath provider = starts[0].GetPrimPath();
        SdfPath culprit;
        const auto owned = newJointBinding.find(provider);
        if (owned != newJointBinding.end()) {
            for (const auto &entry : owned->second) {
                if (entry.first != solver.GetPath() &&
                    stackOrdinalOf(entry.first) > here) {
                    culprit = entry.first;
                    break;
                }
            }
        }
        if (culprit.IsEmpty()) {
            const auto revised = newFrameChains.find(provider);
            if (revised != newFrameChains.end()) {
                for (const SdfPath &writer : revised->second) {
                    if (writer != solver.GetPath() &&
                        stackOrdinalOf(writer) > here) {
                        culprit = writer;
                        break;
                    }
                }
            }
        }
        if (culprit.IsEmpty()) {
            continue;
        }
        const std::string message =
            solver.GetPath().GetString() + " reads startFrame " +
            provider.GetString() + " but executes before " +
            culprit.GetString() +
            ", which poses it, so it reads the pre-write frame. Reorder "
            "so the chain executes after the writer -- earlier in the "
            "file, since siblings execute bottom-first (spec §4.2).";
        if (errors) {
            errors->push_back("warning: " + message);
        }
        TF_WARN("%s", message.c_str());
    }
    // A provider carrying pose revisions that is not a joint needs a base
    // frame from somewhere. A Control has computePointFrame like a joint; a
    // plain UsdGeomXformable has no exec computation at all, so its frame
    // comes from its own USD transform and its revised matrix is published
    // back onto the prim for Hydra to inherit.
    std::set<SdfPath> newXformDerivedProviders;
    for (const auto &[provider, revisions] : newFrameChains) {
        if (std::find(newJointPaths.begin(), newJointPaths.end(), provider) !=
            newJointPaths.end()) {
            continue;  // joints are tapped below
        }
        const UsdPrim providerPrim = _stage->GetPrimAtPath(provider);
        const TfToken type = providerPrim ? providerPrim.GetTypeName()
                                          : TfToken();
        if (type == "RigExecControl" || type == "RigExecJoint" || _IsVolumeWeightType(type)) {
        } else if (providerPrim && UsdGeomXformable(providerPrim)) {
            newXformDerivedProviders.insert(provider);
        } else {
            return fail(
                "constraint target " + provider.GetString() +
                " is neither a RigExec transform provider nor a "
                "UsdGeomXformable; nothing can carry the revised frame", revisions);
        }
    }

    compileBlocks.Next("SolverSchedule.PropertyChains");
    // Property-domain chains, from the same mover execution walk.
    // Nothing to bind and nothing to tap: a math mover's inputs are all
    // authored on itself, and the chain's base is the target attribute's own
    // authored value. That is exactly what makes the chain evaluable BEFORE
    // exec runs, and therefore what lets its result be supplied to exec as a
    // value override -- which is how a clamped weight actually reaches the
    // solver that reads it instead of being reimplemented inside that
    // solver's kernel.
    std::map<SdfPath, std::vector<_PropertyRevision>> newPropertyChains;
    std::vector<SdfPath> newPropertyChainOrder;
    std::vector<RigExecPhasedConnection> newPhasedConnections;
    if (!_CompilePropertyChains(newMovers, newPropertyChains,
                                newPropertyChainOrder, newPhasedConnections,
                                orderedSolvers, inertMovers, failure)) {
        return fail(failure->message, failure->operations);
    }

    compileBlocks.Next("SolverSchedule.PointGraph");
    // Bind the persistent point graph from the composed execution walk.
    // Geometry constraints contribute solved matrix packets to this same
    // chain. Compilation resolves bindings without authoring scene data.
    std::map<SdfPath, std::vector<_GraphRevision>> newGraphChains;
    for (const RigExecMoverRecord &mover : newMovers) {
        const UsdPrim moverPrim = _stage->GetPrimAtPath(mover.moverPath);
        if (!moverPrim) {
            continue;
        }
        TfToken curveMode;
        if (const UsdAttribute a =
                moverPrim.GetAttribute(TfToken("rigExec:mode"))) {
            a.Get(&curveMode);
        }
        std::optional<RigExecRevisionOp> op =
            RigExecRevisionOpForSchema(mover.schemaType, curveMode);
        if (!op && _IsSourceFrameConstraintType(mover.schemaType)) {
            op = RigExecRevisionOp::Matrix;
        }
        if (!op) {
            continue;
        }
        for (const SdfPath &target : mover.targets) {
            if (!target.IsPropertyPath() ||
                target.GetNameToken() != "points") {
                continue;
            }
            _GraphRevision revision;
            revision.moverPath = mover.moverPath;
            revision.target = target;
            revision.op = *op;
            revision.binding =
                RigExecResolveRevisionBinding(moverPrim, target, {});
            // The phase each mover's own handler resolved from the input
            // that names its providers: rigExec:transform, rigExec:influences
            // or rigExec:driverTransforms.
            revision.transformFinalPhase =
                revision.binding.transformPhase.kind ==
                RigExecReadPhaseKind::Final;
            {
                TfToken pointFrame;
                if (const UsdAttribute a = moverPrim.GetAttribute(
                        TfToken("rigExec:pointFrame"))) {
                    a.Get(&pointFrame);
                }
                revision.transformPosedPoints = pointFrame == "posed";
            }
            if (!revision.binding.weightObject.IsEmpty()) {
                registerVolumeWeights(revision.binding.weightObject);
            }
            // Declared leaves capture blend samples. A computeBlendChannel
            // request here would duplicate that sampling.
            if (revision.op == RigExecRevisionOp::Skin) {
                // Whether the per-point layout can change WITHIN this epoch
                // is a question about the stage, so it is answered here once
                // rather than guessed at per frame. The three ways it can:
                // an authored time sample (the arrays differ per time code),
                // an authored connection (the value comes from somewhere
                // else, which may itself be animated), and a property chain
                // writing the attribute (the evaluator computes it per
                // generation). Anything else is epoch-constant by the same
                // definition the digest uses, so the layout is resolved once
                // and shared instead of re-read, re-copied and re-validated
                // on every frame.
                revision.skinTopologyFixed = true;
                for (const char *name : {"rigExec:jointIndices",
                                         "rigExec:jointWeights",
                                         "rigExec:elementSize"}) {
                    const SdfPath propertyPath =
                        mover.moverPath.AppendProperty(TfToken(name));
                    if (newPropertyChains.count(propertyPath)) {
                        revision.skinTopologyFixed = false;
                        break;
                    }
                    const UsdAttribute a =
                        moverPrim.GetAttribute(TfToken(name));
                    if (a && (a.ValueMightBeTimeVarying() ||
                              a.HasAuthoredConnections())) {
                        revision.skinTopologyFixed = false;
                        break;
                    }
                }
            }
            newGraphChains[target].push_back(revision);
        }
    }

    compileBlocks.Next("SolverSchedule.SampleReads");
    // Resolve sample-relative preceding reads against the global mover walk.
    // Samples can belong to a different chain, or multiple blend applications;
    // each application gets its own checkpoint, independent of sample identity.
    std::map<SdfPath, int> applicationOrdinals;
    for (const auto &mover : newMovers) applicationOrdinals[mover.moverPath] = mover.ordinal;
    for (auto &[target, revisions] : newGraphChains) {
        for (auto &revision : revisions) {
            for (auto &[input, samples] : revision.binding.blendSamples) {
                for (auto &sample : samples) {
                    const auto producers = newGraphChains.find(sample.points);
                    if (producers == newGraphChains.end()) {
                        sample.phase = RigExecReadPhase();
                    } else if (sample.phase.kind == RigExecReadPhaseKind::Preceding) {
                        SdfPath checkpoint;
                        for (const auto &producer : producers->second) {
                            if (applicationOrdinals[producer.moverPath] <
                                applicationOrdinals[revision.moverPath])
                                checkpoint = producer.moverPath;
                        }
                        sample.phase = checkpoint.IsEmpty() ? RigExecReadPhase() :
                            RigExecReadPhase{RigExecReadPhaseKind::AtPrim, checkpoint};
                    }
                }
            }
        }
    }

    compileBlocks.Next("SolverSchedule.DerivedMaintenance");
    // Derived maintenance (spec §7.6 revised), mirroring Pass 3: for every
    // moved points target whose gprim authors normals or extent, synthesize
    // the recompute revision. There is no authored mover, so the gprim itself
    // stands in as the parameter source -- it supplies the stage for the
    // static topology reads and has no inputs:enabled, so the revision is
    // enabled. Vertex-normal recomputation is mesh-only; the compiler already
    // rejects authored normals on a non-mesh points target.
    std::map<SdfPath, std::vector<_GraphRevision>> newGraphDerivedChains;
    for (const auto &[pointsTarget, revisions] : newGraphChains) {
        const SdfPath ownerPath = pointsTarget.GetPrimPath();
        const UsdPrim owner = _stage->GetPrimAtPath(ownerPath);
        if (!owner) {
            continue;
        }
        for (const bool isNormals : {true, false}) {
            const TfToken property(isNormals ? "normals" : "extent");
            const SdfPath derivedTarget =
                ownerPath.AppendProperty(property);
            const UsdAttribute authored =
                _stage->GetAttributeAtPath(derivedTarget);
            if (!authored || !authored.HasAuthoredValue()) {
                continue;
            }
            // Vertex-normal recomputation is mesh-only. Silently leaving
            // authored normals stale on a moved Points/BasisCurves target
            // would break the automatic-maintenance contract (spec §7.6
            // revised), so reject the configuration rather than skip it. This
            // check used to live in the compiler's Pass 3.
            if (isNormals && !UsdGeomMesh(owner)) {
                return fail(
                    "authored normals on non-mesh points target " +
                    ownerPath.GetString() +
                    " cannot be maintained (vertex-normal recomputation is "
                    "mesh-only, spec §7.6 revised); remove the authored "
                    "normals");
            }
            _GraphRevision derived;
            derived.moverPath = ownerPath;
            derived.target = derivedTarget;
            derived.op = isNormals ? RigExecRevisionOp::RecomputeNormals
                                   : RigExecRevisionOp::RecomputeExtent;
            derived.binding.moverPath = ownerPath;
            derived.binding.target = derivedTarget;
            derived.binding.topologyCounts =
                ownerPath.AppendProperty(TfToken("faceVertexCounts"));
            derived.binding.topologyIndices =
                ownerPath.AppendProperty(TfToken("faceVertexIndices"));
            if (!isNormals) {
                // The authoritative winning widths, when authored: they
                // widen the extent bound (Pass 3 wires resolvedWidths).
                const SdfPath widthsPath =
                    ownerPath.AppendProperty(TfToken("widths"));
                if (const UsdAttribute w =
                        _stage->GetAttributeAtPath(widthsPath)) {
                    if (w.HasAuthoredValue()) {
                        derived.binding.widths = widthsPath;
                    }
                }
            }
            newGraphDerivedChains[pointsTarget].push_back(derived);
        }
    }
    // Surface projectors (RigExecSurfaceProjector): derived MATRIX targets
    // on the chain they ride, measured from that chain's final points the
    // way normals and extent are. The shader matrix and the packed dials
    // are each one target, published as a primvar of the surface and
    // never authored back.
    for (const _SurfaceProjectorRecord &projector : newSurfaceProjectors) {
        const UsdPrim prim = _stage->GetPrimAtPath(projector.path);
        const SdfPath meshPath = projector.target.GetPrimPath();
        const UsdPrim mesh = _stage->GetPrimAtPath(meshPath);
        if (!prim || !mesh || !UsdGeomMesh(mesh)) {
            return fail(projector.path.GetString() +
                        ": a surface projector must move one mesh's "
                        "points", {projector.path});
        }
        if (!newGraphChains.count(projector.target)) {
            TF_WARN("%s projects onto %s, which no mover deforms; it "
                    "publishes nothing", projector.path.GetText(),
                    meshPath.GetText());
            continue;
        }
        // The mesh's own transform places the ray when the source names no
        // sibling space. It is a stage transform the rig does not own, so
        // it is captured here and must not vary over time.
        for (UsdPrim p = mesh; p && !p.IsPseudoRoot(); p = p.GetParent()) {
            if (const UsdGeomXformable xformable{p}) {
                if (xformable.TransformMightBeTimeVarying()) {
                    return fail(projector.path.GetString() +
                                " projects onto " + meshPath.GetString() +
                                ", whose transform is animated; a surface "
                                "projector needs a static surface "
                                "transform", {projector.path});
                }
            }
        }
        UsdGeomXformCache meshXforms(UsdTimeCode::Default());
        const UsdPrim assetRoot =
            _stage->GetPrimAtPath(_rigPath.GetParentPath());
        const GfMatrix4d assetWorld = assetRoot && !assetRoot.IsPseudoRoot()
            ? meshXforms.GetLocalToWorldTransform(assetRoot)
            : GfMatrix4d(1.0);
        const GfMatrix4d assetToMesh = assetWorld *
            meshXforms.GetLocalToWorldTransform(mesh).GetInverse();
        // Source, its sibling space and the rig's space, each at most one
        // frame provider: bound as the transform, transformSpace and carry
        // providers, which every evaluator already reads per phase.
        const auto oneProvider = [&](const char *name, SdfPath *out) {
            const SdfPathVector targets = getTargets(prim, name);
            if (targets.size() > 1) {
                return fail(projector.path.GetString() + " has more than "
                            "one " + std::string(name), {projector.path});
            }
            if (!targets.empty()) {
                if (!_IsFrameProvider(_stage->GetPrimAtPath(targets[0].GetPrimPath()))) {
                    return fail(projector.path.GetString() + " " +
                                std::string(name) + " target " +
                                targets[0].GetString() +
                                " is not a RigExec frame provider",
                                {projector.path});
                }
                *out = targets[0].GetPrimPath();
            }
            return true;
        };
        _GraphRevision base;
        base.moverPath = projector.path;
        base.binding.moverPath = projector.path;
        base.binding.base = projector.target;
        base.binding.topologyCounts =
            meshPath.AppendProperty(TfToken("faceVertexCounts"));
        base.binding.topologyIndices =
            meshPath.AppendProperty(TfToken("faceVertexIndices"));
        base.binding.meshWorldInverse = assetToMesh;
        if (!oneProvider("rigExec:sources", &base.binding.transform) ||
            !oneProvider("rigExec:sourceSpace",
                         &base.binding.transformSpace) ||
            !oneProvider("rigExec:space", &base.binding.carrySpace)) {
            return false;
        }
        const auto primvarTarget = [&](const char *attr,
                                       const TfToken &fallback) {
            TfToken name = fallback;
            if (const UsdAttribute a = prim.GetAttribute(TfToken(attr))) {
                a.Get(&name);
            }
            return name.IsEmpty()
                       ? SdfPath()
                       : meshPath.AppendProperty(
                             TfToken("primvars:" + name.GetString()));
        };
        const SdfPath shaderTarget =
            primvarTarget("rigExec:shaderPrimvar", TfToken("eyeProjector"));
        if (!shaderTarget.IsEmpty()) {
            _GraphRevision derived = base;
            derived.target = shaderTarget;
            derived.binding.target = shaderTarget;
            derived.op = RigExecRevisionOp::SurfaceProjector;
            newGraphDerivedChains[projector.target].push_back(derived);
        }
        const SdfPath dialTarget =
            primvarTarget("rigExec:shaderDialPrimvar", TfToken());
        if (!dialTarget.IsEmpty()) {
            _GraphRevision derived = base;
            derived.target = dialTarget;
            derived.binding.target = dialTarget;
            // Packs dial values only: no frames and no surface, so no mesh
            // topology for the frame cache to sample with every frame.
            derived.binding.transform = SdfPath();
            derived.binding.transformSpace = SdfPath();
            derived.binding.carrySpace = SdfPath();
            derived.binding.topologyCounts = SdfPath();
            derived.binding.topologyIndices = SdfPath();
            derived.op = RigExecRevisionOp::ShaderDials;
            for (const SdfPath &dial :
                 getTargets(prim, "rigExec:shaderDialSources")) {
                if (!dial.IsPropertyPath()) {
                    return fail(projector.path.GetString() +
                                " rigExec:shaderDialSources names " +
                                dial.GetString() + ", not a property",
                                {projector.path});
                }
                if (derived.binding.shaderDials.size() == 16) {
                    TF_WARN("%s names more than sixteen shader dials; the "
                            "rest do not fit the primvar",
                            projector.path.GetText());
                    break;
                }
                derived.binding.shaderDials.push_back(dial);
            }
            newGraphDerivedChains[projector.target].push_back(derived);
        }
    }

    // One output cannot be both a spatial frame and packed scalar dials.
    // Same-semantic overwrites retain their existing ordered behavior.
    std::map<SdfPath, std::pair<RigExecRevisionOp, SdfPath>> matrixOutputs;
    for (const auto &[pointsTarget, revisions] : newGraphDerivedChains) {
        for (const _GraphRevision &revision : revisions) {
            if (revision.op != RigExecRevisionOp::SurfaceProjector &&
                revision.op != RigExecRevisionOp::ShaderDials) {
                continue;
            }
            const auto [found, inserted] = matrixOutputs.emplace(
                revision.target, std::make_pair(revision.op, revision.moverPath));
            if (!inserted && found->second.first != revision.op) {
                const SdfPath spatial =
                    revision.op == RigExecRevisionOp::SurfaceProjector
                        ? revision.moverPath : found->second.second;
                const SdfPath dials =
                    revision.op == RigExecRevisionOp::ShaderDials
                        ? revision.moverPath : found->second.second;
                return fail("shader matrix output " + revision.target.GetString() +
                            " is a spatial SurfaceProjector frame from " +
                            spatial.GetString() + " and packed ShaderDials from " +
                            dials.GetString() + "; choose different "
                            "rigExec:shaderPrimvar and "
                            "rigExec:shaderDialPrimvar names", {spatial, dials});
            }
        }
    }

    compileBlocks.Next("SolverSchedule.PoseDag");
    // Compile one pose DAG. Constraints retain authored preceding order;
    // solvers read the final constrained state of each declared frame input.
    // Bound joints replace their namespace callback, cutting ancestor edges.
    auto isFrameProvider = [&](const SdfPath &path) {
        const UsdPrim prim = _stage->GetPrimAtPath(path);
        return _IsFrameProvider(prim);
    };
    std::map<SdfPath, _PoseInputInfo> newPoseInputInfo;
    // A stage closure that the schedule and request passes below ask for
    // over and over about the same prims: the pose-provider closure of one
    // input path. The same joint is an input to many solvers, many
    // constraints and many batches, and each ask re-walks the whole chain to
    // the rig root.
    // It is a pure function of the composed stage and of the joint binding
    // this compile has already decided above; neither changes while a compile
    // runs, so a memoized answer is the answer a recomputation would give.
    // It is compile-local, deliberately separate from the digest's own
    // caches: the digest must stay a self-contained recomputation. (Its
    // sibling, a prim's connection-input set, went with the solver-input
    // index to _BuildSolverInputIndex, whose pool reads each prim once.)
    std::unordered_map<SdfPath, std::set<SdfPath>, SdfPath::Hash>
        poseClosureCache;
    // The per-prim half of the pose closure, computed ahead of the walks
    // that need it by the pose-input graph, for every prim those walks will
    // ask about (see the prefetch below). A prim's closure is a pure read of
    // the composed stage, so its answer does not depend on when it is
    // computed, which is what lets the walk below take one out of here
    // instead of paying for it in line.
    // Deliberately NOT newPoseInputInfo itself. That map is iterated as a
    // RESULT further down (the connected-pose taps, newPoseProviderInputs),
    // so it has to hold exactly the prims a closure walk actually reached
    // and nothing more. This one is free to overshoot -- a prefetched prim
    // no walk asks about costs a worker's time and nothing else -- which is
    // what makes seeding it generously safe.
    std::unordered_map<SdfPath, _PoseInputInfo, SdfPath::Hash>
        poseInfoPrefetch;
    // Shared, not local: a deferred epoch hands it to
    // _RealizeDeferredExecPrep, which materializes the attribute sets the
    // solver-input index reads from it.
    const auto poseInputGraph = std::make_shared<_PoseInputGraph>();
    const std::function<bool(const SdfPath &)> isBoundJoint =
        [&](const SdfPath &path) { return newJointBinding.count(path) != 0; };
    auto computePoseProviderClosure = [&](const SdfPath &input) {
        std::set<SdfPath> closure;
        std::vector<SdfPath> pending{input};
        while (!pending.empty()) {
            const SdfPath path = pending.back();
            pending.pop_back();
            if (path.IsEmpty() || !closure.insert(path).second) continue;
            if (newJointBinding.count(path)) {
                newPoseInputInfo[path];
                continue;
            }
            auto it = newPoseInputInfo.find(path);
            if (it == newPoseInputInfo.end()) {
                // Same value either way: the prefetch ran the same function
                // on the same prim of the same stage. Only newPoseInputInfo
                // records that this path was REACHED.
                // MOVED out, not copied: a path lands in newPoseInputInfo
                // at most once (this branch is the find() miss), so nothing
                // reads the prefetch entry again, and leaving the sets
                // behind would hold every closure's inputs on the heap
                // TWICE for the rest of compile -- which measured as ~13 ms
                // added to the batch pass that runs after it, purely in
                // allocator and locality cost.
                // A prim the prefetch did not ask about is asked about now,
                // through the same graph, so every closure of this compile
                // materializes its attributes the same way.
                auto warm = poseInfoPrefetch.find(path);
                if (warm == poseInfoPrefetch.end()) {
                    poseInputGraph->Extend(
                        _stage, {path}, isBoundJoint,
                        RigExecParallelEvaluationEnabled(), _profiler,
                        &poseInfoPrefetch);
                    warm = poseInfoPrefetch.find(path);
                }
                it = newPoseInputInfo.emplace(path,
                    warm != poseInfoPrefetch.end()
                        ? std::move(warm->second)
                        : _CollectPoseInputInfo(
                              _stage->GetPrimAtPath(path))).first;
            }
            pending.insert(pending.end(), it->second.providers.begin(),
                           it->second.providers.end());
            if (!isFrameProvider(path)) {
                const UsdPrim parent = _NamespaceFrameProvider(_stage->GetPrimAtPath(path));
                if (parent) pending.push_back(parent.GetPath());
            }
        }
        return closure;
    };
    auto poseProviderClosure =
        [&](const SdfPath &input) -> const std::set<SdfPath> & {
        auto it = poseClosureCache.find(input);
        if (it == poseClosureCache.end()) {
            it = poseClosureCache.emplace(
                input, computePoseProviderClosure(input)).first;
        }
        return it->second;
    };
    std::map<SdfPath, std::set<SdfPath>> solverFrameInputs;
    // The read phase a solver declares on an input relationship, through the
    // same optional rigExecReadPhase metadata a mover input uses. Without
    // one the hierarchy decides (spec §4.2): a constraint ABOVE the solver
    // revises what the solver already read. "final", or a prim path at or
    // after that constraint, asks for the frame as the constraint left it,
    // and the solver then waits on it (the frame-inheritance walk below).
    // Base and preceding are the hierarchy rule and are not stored.
    std::map<SdfPath, std::map<SdfPath, std::vector<RigExecReadPhase>>>
        solverInputPhases;
    for (const auto &[solver, dependencies] : newSolverDependencies) {
        const UsdPrim prim = _stage->GetPrimAtPath(solver);
        // The ribbon's read-phase attributes were replaced by metadata and
        // would now compose as inert custom attributes.
        for (const TfToken &removed : _removedRibbonPhases) {
            const UsdAttribute old = prim.GetAttribute(removed);
            if (old && old.HasAuthoredValue()) {
                return fail(solver.GetString() + " authors " +
                                removed.GetString() +
                                ", which was replaced by rigExecReadPhase "
                                "metadata on the input relationship",
                            {solver});
            }
        }
        for (const UsdRelationship &rel : prim.GetRelationships()) {
            if (rel.GetName() == "rigExec:joints") {
                continue;
            }
            RigExecReadPhase phase;
            std::string phaseError;
            if (!RigExecResolveReadPhase(rel, &phase, &phaseError)) {
                return fail(solver.GetString() + " " +
                                rel.GetName().GetString() + ": " + phaseError,
                            {solver});
            }
            const bool phased = phase.kind == RigExecReadPhaseKind::Final ||
                                phase.kind == RigExecReadPhaseKind::AtPrim;
            SdfPathVector targets;
            rel.GetTargets(&targets);
            for (const SdfPath &target : targets) {
                // Ribbon driverCurve is a geometry value, not a frame
                // relationship. Its exact Base/Preceding/AtPrim/Final points
                // producer is bound by FinalizeCrossDomainReads.
                if (prim.GetTypeName() == "RigExecRibbon" &&
                    rel.GetName() == "rigExec:driverCurve") {
                    const SdfPath points = target.IsPrimPath()
                        ? target.AppendProperty(TfToken("points")) : target;
                    const UsdAttribute input = _stage->GetAttributeAtPath(points);
                    if (phased && (!input || input.GetTypeName().GetType() !=
                                              TfType::Find<VtVec3fArray>())) {
                        return fail(solver.GetString() + " rigExec:driverCurve declares rigExecReadPhase '" +
                                    phase.GetAsString() + "' on " + target.GetString() +
                                    ", which is not a Vec3f array points source", {solver});
                    }
                    continue;
                }
                if (isFrameProvider(target.GetPrimPath())) {
                    solverFrameInputs[solver].insert(target.GetPrimPath());
                    if (phased) {
                        solverInputPhases[solver][target.GetPrimPath()]
                            .push_back(phase);
                    }
                } else if (phased) {
                    // A solver reads a phase only through a provider's
                    // frame chain; anything else would read base silently.
                    return fail(solver.GetString() + " " +
                                    rel.GetName().GetString() +
                                    " declares rigExecReadPhase '" +
                                    phase.GetAsString() + "' on " +
                                    target.GetString() +
                                    ", which is not a frame provider; a "
                                    "solver reads a phase only from a "
                                    "control or joint",
                                {solver});
                }
            }
        }
    }
    // MEASURED (biped_stack_anim): the pose closure was the largest single
    // cost left on this thread -- 13.0 ms inside the constraint pass and
    // 21.9 ms again in PrepareRequests.ProviderClosure, both of them one
    // prim's _CollectPoseInputInfo at a time. The walk that spends it is
    // inherently serial (each prim's answer says which prim to ask about
    // next), but the ASKING is not: a level of the frontier is a set of
    // independent stage reads.
    // So the closures are computed here, all at once, by _PoseInputGraph:
    // its reads are spread across the pool a level at a time, and each
    // attribute any closure reaches is read once rather than once per prim
    // that reaches it. The walks below then find their prims already
    // computed and do set arithmetic only. Concurrent reads of a UsdStage
    // are what the digest thread, the tap warm-up and the two bake bind
    // loops already do.
    // Seeded with every path a closure will be asked for -- the solvers'
    // frame inputs, every constraint's inputs, and the joints and controls
    // that PrepareRequests seeds providers from -- plus each one's
    // frame-provider ancestors, because those are what newNativeProviderPaths
    // holds by the time ProviderClosure asks. Overshooting is free; missing a
    // prim only means the walk asks the graph for it in line.
    {
        RIGEXEC_PROFILE_SCOPE_CAT(_profiler, "PoseInfoPrefetch", "compile");
        std::vector<SdfPath> seeds;
        const auto seed = [&](const SdfPath &path) {
            if (path.IsEmpty()) return;
            seeds.push_back(path);
            // Only the FRAME-PROVIDER ancestors, because those are the ones
            // seedProvider puts in newNativeProviderPaths and ProviderClosure
            // then asks about. Pushing the whole namespace chain instead
            // reads a large part of the stage that no walk ever asks for,
            // and the allocation that costs lands on every pass after it.
            for (SdfPath a = path.GetParentPath();
                 !a.IsEmpty() && a != SdfPath::AbsoluteRootPath();
                 a = a.GetParentPath()) {
                if (isFrameProvider(a)) seeds.push_back(a);
            }
        };
        for (const auto &[solver, inputs] : solverFrameInputs) {
            for (const SdfPath &input : inputs) seed(input);
        }
        for (const _FrameConstraint &constraint : newFrameConstraints) {
            for (const SdfPath &target : constraint.targets) seed(target);
            for (const auto &source : constraint.sources) {
                seed(source.sourcePath);
            }
            seed(constraint.worldUpObject.sourcePath);
            seed(constraint.spacePath);
            seed(constraint.effector.sourcePath);
            seed(constraint.weightObject);
            for (const auto &pole : constraint.poleObjects) {
                seed(pole.sourcePath);
            }
        }
        for (const SdfPath &path : newJointPaths) seed(path);
        for (const SdfPath &path : newControlPaths) seed(path);
        // A joint-bound path stops the real walk without being read, so the
        // graph does not ask about it either.
        poseInputGraph->Extend(_stage, seeds, isBoundJoint,
                               RigExecParallelEvaluationEnabled(), _profiler,
                               &poseInfoPrefetch);
    }
    std::map<SdfPath, std::set<SdfPath>> solverPoseReads;
    // solverInputPhases spread over each input's pose closure, so the walk
    // that visits closure members finds the phase of the input that reached
    // them.
    std::map<SdfPath, std::map<SdfPath, std::vector<RigExecReadPhase>>>
        solverClosurePhases;
    for (const auto &[solver, inputs] : solverFrameInputs) {
        const auto declared = solverInputPhases.find(solver);
        for (const SdfPath &input : inputs) {
            const std::set<SdfPath> &closure = poseProviderClosure(input);
            solverPoseReads[solver].insert(closure.begin(), closure.end());
            if (declared != solverInputPhases.end()) {
                const auto phases = declared->second.find(input);
                if (phases != declared->second.end()) {
                    for (const SdfPath &member : closure) {
                        std::vector<RigExecReadPhase> &into =
                            solverClosurePhases[solver][member];
                        into.insert(into.end(), phases->second.begin(),
                                    phases->second.end());
                    }
                }
            }
            for (const SdfPath &provider : closure) {
                const auto owner = newJointBinding.find(provider);
                if (owner != newJointBinding.end()) {
                    // Every writer BELOW this solver in the stack, never this
                    // solver itself -- the same rule the direct relationship
                    // walk applies above, and the edge class that catches an
                    // indirect read (an IK control parented under a joint, a
                    // startFrameObject reached through a namespace parent).
                    // A writer ABOVE it gets the mirror edge instead: the
                    // solver read the version standing before that write, so
                    // the write has to wait.
                    for (const auto &[writer, writerElement] :
                         owner->second) {
                        if (writer == solver) {
                            continue;
                        }
                        if (jointWritingSolvers.count(solver) &&
                            stackBefore(solver, writer)) {
                            poseReverseEdges.emplace_back(writer, solver);
                            continue;
                        }
                        newSolverDependencies[solver].insert(writer);
                    }
                }
            }
        }
    }
    // The solver DAG is complete here, but the POSE graph is not: the
    // constraint pass and the frame-inheritance walk below still add
    // solver -> constraint -> solver edges, and a pair those order is a pair
    // the stack must not order differently. So the stack order is settled
    // ONCE, against the finished graph, just before the schedule is built --
    // see "the stack order" block below SolverSchedule.FrameInheritance.
    std::map<SdfPath, std::vector<std::pair<SdfPath, int>>> newSolverJoints;
    std::set<SdfPath> requiredSolvers;
    for (const auto &[joint, writers] : newJointBinding) {
        for (const auto &[solver, element] : writers) {
            newSolverJoints[solver].emplace_back(joint, element);
            requiredSolvers.insert(solver);
        }
    }
    // What used to be one SolverSchedule.AggregateConsumers block, and the
    // largest single region left on the main thread. Split five ways because
    // a guess about which of its passes held the time was already wrong once:
    // the cross product the walk below replaces was predicted to be 20 of
    // those milliseconds and measured, after inversion, to be worth none of
    // them. The passes are quite different work -- set closure, constraint
    // reads through the pose closure, an ancestor walk, a counting pass, and
    // the batch BFS that talks to exec -- so they are timed apart before
    // anything else here is touched.
    compileBlocks.Next("SolverSchedule.RequiredSolvers");
    // Geometry aggregate consumers are authoritative even without joints.
    for (const auto &[target, revisions] : newGraphChains) {
        for (const _GraphRevision &revision : revisions) {
            if (newSolverDependencies.count(revision.binding.driverFrames)) {
                requiredSolvers.insert(revision.binding.driverFrames);
            }
        }
    }
    for (const auto &[target, revisions] : newGraphDerivedChains) {
        for (const _GraphRevision &revision : revisions) {
            if (newSolverDependencies.count(revision.binding.driverFrames))
                requiredSolvers.insert(revision.binding.driverFrames);
        }
    }
    std::vector<SdfPath> pendingSolvers(requiredSolvers.begin(), requiredSolvers.end());
    while (!pendingSolvers.empty()) {
        const SdfPath solver = pendingSolvers.back();
        pendingSolvers.pop_back();
        for (const SdfPath &dependency : newSolverDependencies[solver]) {
            if (requiredSolvers.insert(dependency).second) {
                pendingSolvers.push_back(dependency);
            }
        }
    }
    std::map<SdfPath, std::set<SdfPath>> poseDependencies;
    std::map<SdfPath, size_t> constraintIndices;
    for (const SdfPath &solver : requiredSolvers) {
        poseDependencies[solver] = newSolverDependencies[solver];
    }
    compileBlocks.Next("SolverSchedule.ConstraintDeps");
    // Collected in the constraint loop, consumed by the upward walk just
    // below it. See the comment there for why the cross product this
    // replaces had to be turned inside out.
    std::map<SdfPath, std::vector<SdfPath>> constraintsByTarget;
    // Constraint -> constraint frame edges. A frame constraint writes its
    // target's final frame and, through commitConstraintFrames, every
    // namespace descendant up to an independently solved joint; a native
    // Xform source rides its closest revised provider ancestor with no such
    // stop. Pose-domain writers by target, as mover indices.
    std::map<SdfPath, std::vector<size_t>> constraintFrameWriters;
    for (size_t i = 0; i < newFrameConstraints.size(); ++i) {
        const _FrameConstraint &constraint = newFrameConstraints[i];
        if (!constraint.pointsTarget.IsEmpty()) continue;
        for (const SdfPath &target : constraint.targets) {
            constraintFrameWriters[target].push_back(i);
        }
    }
    // The constraints whose commit can reach \p path: writers of the path
    // and of each namespace ancestor, the walk ending AFTER the first
    // solver-bound joint (its own writers count; nothing above it propagates
    // through it). \p throughJoints drops that stop, for a native source.
    // Authored parent:space also stops propagation, but per frame, so the
    // compile keeps the edge. Memoized; a pure function of the binding.
    std::map<std::pair<SdfPath, bool>, std::vector<size_t>>
        constraintWritersReaching;
    const auto writersReaching =
        [&](const SdfPath &path,
            bool throughJoints) -> const std::vector<size_t> & {
        const auto key = std::make_pair(path, throughJoints);
        const auto found = constraintWritersReaching.find(key);
        if (found != constraintWritersReaching.end()) {
            return found->second;
        }
        std::vector<size_t> writers;
        for (SdfPath p = path;
             !p.IsEmpty() && p != SdfPath::AbsoluteRootPath();
             p = p.GetParentPath()) {
            const auto it = constraintFrameWriters.find(p);
            if (it != constraintFrameWriters.end()) {
                writers.insert(writers.end(), it->second.begin(),
                               it->second.end());
            }
            if (!throughJoints && newJointBinding.count(p)) break;
        }
        return constraintWritersReaching.emplace(key, std::move(writers))
            .first->second;
    };
    for (size_t i = 0; i < newFrameConstraints.size(); ++i) {
        const _FrameConstraint &constraint = newFrameConstraints[i];
        const SdfPath &path = constraint.moverPath;
        constraintIndices[path] = i;
        auto &dependencies = poseDependencies[path];
        // A constraint is ordered only by the frames it reads and writes:
        // the edges below, plus the per-joint writer chain built after the
        // frame-inheritance walk. Mover order adds no edge of its own, so
        // constraints that share no frame share a Kahn level and may run
        // concurrently.
        // Every writer of the frame, not just the last one, DIRECTIONALLY
        // (spec §4.2). A frame read is POSITIONAL: the
        // constraint reads the version standing at its own place in the
        // unified pose stack. A writer BELOW it must therefore run first, and
        // a writer ABOVE it must wait, or which version was read is undefined
        // and the schedule decides it by accident. A missed edge here does not
        // fail loudly; it silently reads the wrong version.
        const bool positional = poseStackOrdinal.count(path) > 0;
        const int here = stackOrdinalOf(path);
        // Constraint writers of what this one reads, each ordered once, by
        // the same positional rule as a solver writer. A geometry-domain
        // constraint has no ordinal and keeps the place its mover index gives
        // it: after every earlier frame writer of what it reads, before every
        // later one.
        std::set<size_t> orderedWriters;
        const auto orderAgainst = [&](const std::vector<size_t> &writers) {
            for (const size_t w : writers) {
                if (w == i || !orderedWriters.insert(w).second) continue;
                const SdfPath &writer = newFrameConstraints[w].moverPath;
                const bool above = positional
                    ? stackOrdinalOf(writer) > here
                    : w > i;
                if (above) {
                    poseReverseEdges.emplace_back(writer, path);
                } else {
                    dependencies.insert(writer);
                }
            }
        };
        auto dependOnFrame = [&](const SdfPath &input,
                                 bool nativeSource = false) {
            for (const SdfPath &provider : poseProviderClosure(input)) {
                const auto owner = newJointBinding.find(provider);
                if (owner != newJointBinding.end()) {
                    for (const auto &[writer, element] : owner->second) {
                        if (positional && stackOrdinalOf(writer) > here) {
                            poseReverseEdges.emplace_back(writer, path);
                            continue;
                        }
                        dependencies.insert(writer);
                    }
                }
                orderAgainst(writersReaching(provider, false));
            }
            if (nativeSource) {
                orderAgainst(writersReaching(input, true));
            }
        };
        const auto dependOnSource = [&](const _FrameSourceBinding &source) {
            dependOnFrame(source.sourcePath, !source.xformPath.IsEmpty());
        };
        for (const SdfPath &target : constraint.targets) dependOnFrame(target);
        for (const auto &source : constraint.sources) dependOnSource(source);
        dependOnSource(constraint.worldUpObject);
        // rigExec:space is read from the seed (compose) table, never from a
        // version the walk writes, so it orders nothing.
        dependOnSource(constraint.effector);
        dependOnFrame(constraint.weightObject);
        for (const auto &pole : constraint.poleObjects) dependOnSource(pole);
        // Geometry-domain constraints write only their own per-mover delta,
        // which the revision chains consume after the walk in mover order,
        // so two on one points target need no edge between them.
        // A geometry-domain constraint revises points, not the frame a solver
        // reads, so no solver can inherit one: this `continue` is the same
        // exclusion the cross product below used to get from skipping the
        // block it guarded, and it has to stay exactly that strict.
        if (!constraint.pointsTarget.IsEmpty()) {
            continue;
        }
        for (const SdfPath &target : constraint.targets) {
            constraintsByTarget[target].push_back(path);
        }
    }
    compileBlocks.Next("SolverSchedule.FrameInheritance");
    // Which solvers must wait on which frame constraints. Asked the other way
    // round: for every pose a solver reads, walk UP to the rig root and pick
    // up the constraints that target each ancestor on the way.
    // This used to be the cross product -- constraints x requiredSolvers x
    // that solver's pose reads x that constraint's targets, with a call to an
    // `inheritsFrame` predicate per tuple, each call re-walking an ancestor
    // chain. That shape was PREDICTED to be 20 of the 23.4 ms this region
    // cost, and the prediction was wrong: inverted, the region measured
    // 25.7 -> 26.4 ms across four runs a side, which is inside the noise.
    // The time is somewhere else in the five blocks this one is now split
    // into. The inversion is kept because it is the better code -- linear in
    // the ancestor chain rather than quartic in the cross product, so it
    // cannot become the problem later -- not because it bought anything.
    // Inverted, each (solver, input) chain
    // is walked ONCE for all constraints instead of once per constraint,
    // because the map lookup answers "which constraints target this ancestor"
    // in one step. The predicate is gone with its only caller.
    // The walk reproduces that predicate exactly, half-open range included.
    // inheritsFrame(input, target) was `input.HasPrefix(target)` AND no
    // newJointBinding entry on any path in [input, target) -- its loop ran
    // `for (path = input; path != target; ...)`, so the TARGET ITSELF was
    // never tested for a binding, and input == target was vacuously true with
    // no test at all. Hence the binding check below comes AFTER recording p's
    // own constraints and not before: a joint binding sitting ON a constraint
    // target does not disqualify that target, only bindings strictly below
    // it do. Reversing those two statements silently drops dependencies and
    // the schedule runs a solver before the constraint it reads.
    // Targets are prim paths (see _FrameConstraint::targets), so "ancestor
    // chain" and HasPrefix agree; a target that is somehow not on the chain
    // is simply never found, which is what the predicate answered for it too.
    // poseDependencies[solver] is a std::set, so the different insertion
    // ORDER this produces cannot change its contents, and the contents are
    // all pendingPose/poseConsumers below are built from -- same sets, same
    // schedule. The `empty()` guard keeps even the default-insertion
    // behaviour of solverPoseReads[solver] identical to the old code, which
    // only reached that operator[] when at least one constraint survived the
    // pointsTarget test.
    // The last stack ordinal at or beneath a prim: the moment an AtPrim read
    // phase names, since the stack runs a scope's contents before the scope.
    // -1 when nothing in the stack is there, which no constraint precedes.
    std::map<SdfPath, int> phasePrimOrdinals;
    const auto phaseOrdinal = [&](const SdfPath &prim) {
        const auto found = phasePrimOrdinals.find(prim);
        if (found != phasePrimOrdinals.end()) {
            return found->second;
        }
        int last = -1;
        for (const auto &[step, ordinal] : poseStackOrdinal) {
            if (step.HasPrefix(prim)) {
                last = std::max(last, ordinal);
            }
        }
        phasePrimOrdinals.emplace(prim, last);
        return last;
    };
    // Whether \p solver declared that it reads \p input as of \p constraint
    // or later: a "final" phase, or an AtPrim phase at or after it.
    const auto readsAfter = [&](const SdfPath &solver, const SdfPath &input,
                                const SdfPath &constraint) {
        const auto bySolver = solverClosurePhases.find(solver);
        if (bySolver == solverClosurePhases.end()) {
            return false;
        }
        const auto phases = bySolver->second.find(input);
        if (phases == bySolver->second.end()) {
            return false;
        }
        const int at = stackOrdinalOf(constraint);
        for (const RigExecReadPhase &phase : phases->second) {
            if (phase.kind == RigExecReadPhaseKind::Final ||
                (phase.kind == RigExecReadPhaseKind::AtPrim &&
                 phaseOrdinal(phase.prim) >= at)) {
                return true;
            }
        }
        return false;
    };
    if (!constraintsByTarget.empty()) {
        for (const SdfPath &solver : requiredSolvers) {
            for (const SdfPath &input : solverPoseReads[solver]) {
                for (SdfPath p = input;
                     !p.IsEmpty() && p != SdfPath::AbsoluteRootPath();
                     p = p.GetParentPath()) {
                    const auto it = constraintsByTarget.find(p);
                    if (it != constraintsByTarget.end()) {
                        // Directional for the same reason dependOnFrame is:
                        // a constraint ABOVE this solver revises what the
                        // solver already read, so the solver must NOT wait
                        // on it -- the constraint waits on the solver
                        // instead. (Producers have no position and keep the
                        // unconditional edge.) A solver input that declares
                        // a later read phase is the exception, and the only
                        // one: it reads the frame as the constraint left it.
                        const bool positional =
                            poseStackOrdinal.count(solver) > 0;
                        const int here = stackOrdinalOf(solver);
                        for (const SdfPath &constraint : it->second) {
                            const bool above =
                                positional && stackOrdinalOf(constraint) > here;
                            const bool declared =
                                above && readsAfter(solver, input, constraint);
                            if (declared) {
                                // The solver also writes the joint this
                                // constraint moves, and the stack orders the
                                // two writers by position: a declared read of
                                // the later one would be a cycle.
                                const auto writers = newJointBinding.find(p);
                                if (writers != newJointBinding.end()) {
                                    for (const auto &[writer, element] :
                                         writers->second) {
                                        if (writer != solver) continue;
                                        return fail(
                                            solver.GetString() +
                                                " declares a read phase that "
                                                "reads " + p.GetString() +
                                                " after " +
                                                constraint.GetString() +
                                                ", but it also writes that "
                                                "joint and executes before "
                                                "the constraint; move the "
                                                "constraint below the solver "
                                                "instead (spec §4.2)",
                                            {solver});
                                    }
                                }
                            }
                            if (above && !declared) {
                                poseReverseEdges.emplace_back(constraint,
                                                              solver);
                                continue;
                            }
                            poseDependencies[solver].insert(constraint);
                        }
                    }
                    if (newJointBinding.count(p)) {
                        break;
                    }
                }
            }
        }
    }
    // poseDependencies is complete here and nowhere earlier: the solver DAG
    // was finished above, but the constraint pass and the frame-inheritance
    // walk just added the solver <-> constraint edges.
    // The rule (spec §4.2) is now the NAMESPACE and nothing else. Every step
    // of the pose phase -- a solver that writes a joint, a constraint that
    // moves one -- carries a poseStackOrdinal taken from the reverse composed
    // pre-order of the whole rig, and that ordinal IS the order. Data flow no
    // longer bends it: a frame read below its writer reads the earlier
    // version (the directional edges above), and an aggregate read that
    // contradicts it was rejected by name at compile time.
    // So this block does three things and no searching:
    //   1. put every joint's writer list into ordinal order;
    //   2. build that joint's INTERLEAVED writer chain -- its solvers and the
    //      constraints that move it, in one ordinal order;
    //   3. insert one edge per adjacent pair of that chain, so the schedule
    //      runs them in it.
    // A joint's chain is a restriction of one total order, so no two joints
    // can order the same pair in opposite directions and no edge inserted
    // here can close a loop.
    // WIDTH. The hierarchical order is never turned into a global serial
    // chain: an edge is only ever inserted between two steps that write the
    // SAME joint, and every other edge in poseDependencies is a real data-flow
    // one (a reader and a writer of what it reads). Steps -- solvers and
    // constraints alike -- that share no joint and no data flow therefore
    // share a Kahn level and may evaluate concurrently.
    // testRigExecSolverStacking's TestUnrelatedLimbsShareALevel and
    // TestIndependentConstraintsShareALevel pin it; the biped's 24 solvers
    // span 6 dependency levels.
    std::map<SdfPath, std::vector<SdfPath>> jointWriterChain;
    /// solver -> (joint -> the step that wrote the joint just before it), the
    /// compile-side half of "the incoming frame is the solver's rest".
    std::map<SdfPath, std::map<SdfPath, SdfPath>> solverRestPredecessor;
    {
        for (auto &[joint, writers] : newJointBinding) {
            if (writers.size() < 2) continue;
            std::stable_sort(
                writers.begin(), writers.end(),
                [&stackOrdinalOf](const std::pair<SdfPath, int> &a,
                                  const std::pair<SdfPath, int> &b) {
                    return stackOrdinalOf(a.first) < stackOrdinalOf(b.first);
                });
        }
        // Which pose-domain constraints move each provider. Geometry-domain
        // constraints are excluded: they revise points, carry no stack
        // position, and are not writers of a frame.
        std::map<SdfPath, std::vector<SdfPath>> frameWritingConstraints;
        for (const _FrameConstraint &constraint : newFrameConstraints) {
            if (!constraint.pointsTarget.IsEmpty()) continue;
            if (!poseStackOrdinal.count(constraint.moverPath)) continue;
            for (const SdfPath &target : constraint.targets) {
                frameWritingConstraints[target].push_back(
                    constraint.moverPath);
            }
        }
        std::set<SdfPath> written;
        for (const auto &[joint, writers] : newJointBinding) {
            written.insert(joint);
        }
        for (const auto &[target, constraints] : frameWritingConstraints) {
            written.insert(target);
        }
        for (const SdfPath &joint : written) {
            std::vector<std::pair<int, SdfPath>> chain;
            const auto solvers = newJointBinding.find(joint);
            if (solvers != newJointBinding.end()) {
                for (const auto &[writer, element] : solvers->second) {
                    chain.emplace_back(stackOrdinalOf(writer), writer);
                }
            }
            const auto constraints = frameWritingConstraints.find(joint);
            if (constraints != frameWritingConstraints.end()) {
                for (const SdfPath &constraint : constraints->second) {
                    chain.emplace_back(stackOrdinalOf(constraint), constraint);
                }
            }
            std::sort(chain.begin(), chain.end());
            std::vector<SdfPath> &ordered = jointWriterChain[joint];
            for (const auto &[ordinal, step] : chain) {
                ordered.push_back(step);
            }
            for (size_t i = 1; i < ordered.size(); ++i) {
                if (ordered[i] == ordered[i - 1]) continue;
                poseDependencies[ordered[i]].insert(ordered[i - 1]);
            }
            // "The incoming frame replaces the authored rest." For each
            // solver in the chain, the step immediately before it that wrote
            // this joint -- which is the frame that solver measures from.
            for (size_t i = 1; i < ordered.size(); ++i) {
                if (!jointWritingSolvers.count(ordered[i])) continue;
                const auto owner = newJointBinding.find(joint);
                bool writesIt = false;
                if (owner != newJointBinding.end()) {
                    for (const auto &[writer, element] : owner->second) {
                        writesIt = writesIt || writer == ordered[i];
                    }
                }
                if (!writesIt) continue;
                solverRestPredecessor[ordered[i]][joint] = ordered[i - 1];
            }
        }
    }
    // The single-chain IK constraint is the one CONSTRAINT that measures
    // from joint rests, so the same substitution reaches it: a joint a step
    // below it wrote hands it that step's frame as the rest reference.
    for (_FrameConstraint &constraint : newFrameConstraints) {
        if (constraint.ikChain.empty()) continue;
        constraint.ikRestLive.assign(constraint.ikChain.size(), 0);
        for (size_t i = 0; i < constraint.ikChain.size(); ++i) {
            const auto it = jointWriterChain.find(constraint.ikChain[i]);
            if (it == jointWriterChain.end()) continue;
            for (size_t k = 0; k < it->second.size(); ++k) {
                if (it->second[k] == constraint.moverPath) {
                    constraint.ikRestLive[i] = k > 0 ? 1 : 0;
                    break;
                }
            }
        }
    }
    // The reverse edges -- "a writer waits on a reader standing below it" --
    // collected wherever a read was resolved. Applied here, once, because
    // poseDependencies is only complete now and because an edge inserted into
    // it earlier would have been read back as a dependency by the passes
    // above.
    for (const auto &[waiter, waitedOn] : poseReverseEdges) {
        if (waiter == waitedOn) continue;
        if (!poseDependencies.count(waiter)) continue;
        if (!poseDependencies.count(waitedOn)) continue;
        poseDependencies[waiter].insert(waitedOn);
    }

    compileBlocks.Next("SolverSchedule.PoseReady");
    struct NativePoseOwner { bool solverBatch; size_t index; };
    std::vector<NativePoseOwner> nativePoseOwners;
    std::vector<_SolverBatch> newSolverBatches;
    // Provider identities include every required namespace ancestor.
    std::set<SdfPath> newNativeProviderPaths;
    std::set<SdfPath> newRestEpochProviders;
    const auto seedRestEpochProvider = [&](SdfPath path) {
        for (; !path.IsEmpty() && path != SdfPath::AbsoluteRootPath();
             path = path.GetParentPath()) {
            if (newRestEpochProviders.count(path)) break;
            if (!isFrameProvider(path)) continue;
            newRestEpochProviders.insert(path);
        }
    };
    // Preserve the already-reached connected-read keys, without seeding their
    // namespace ancestors until the original late connected-input pass.
    std::set<SdfPath> restLatePoseInputs;
    for (const auto &entry : newPoseInputInfo) restLatePoseInputs.insert(entry.first);
    auto seedProvider = [&](SdfPath path, bool restEpoch = true) {
        if (restEpoch) seedRestEpochProvider(path);
        for (; !path.IsEmpty() && path != SdfPath::AbsoluteRootPath();
             path = path.GetParentPath()) {
            if (newNativeProviderPaths.count(path)) break;
            if (!isFrameProvider(path)) continue;
            newNativeProviderPaths.insert(path);
        }
    };
    for (const SdfPath &path : newJointPaths) seedProvider(path);
    for (const SdfPath &path : newControlPaths) seedProvider(path);
    for (const SdfPath &path : newVolumeWeightProviders) seedProvider(path);
    const auto seedGeometryProviders = [&](const auto &chains) {
        for (const auto &[target, revisions] : chains)
            for (const _GraphRevision &revision : revisions) {
                seedProvider(revision.binding.transform, false);
                seedProvider(revision.binding.transformSpace, false);
                seedProvider(revision.binding.carrySpace, false);
                for (const SdfPath &provider : revision.binding.influences)
                    seedProvider(provider, false);
            }
    };
    seedGeometryProviders(newGraphChains);
    seedGeometryProviders(newGraphDerivedChains);
    for (const auto &[solver, frames] : solverFrameInputs) {
        for (const SdfPath &path : frames) seedProvider(path);
    }
    for (const _FrameConstraint &constraint : newFrameConstraints) {
        for (const SdfPath &target : constraint.targets) seedProvider(target);
        for (const auto &source : constraint.sources) seedProvider(source.sourcePath);
        seedProvider(constraint.worldUpObject.sourcePath);
        seedProvider(constraint.spacePath);
        seedProvider(constraint.effector.sourcePath);
        for (const auto &pole : constraint.poleObjects) seedProvider(pole.sourcePath);
    }
    compileBlocks.Next("PrepareRequests.SpaceSwitches");
    // RigExecSpaceSwitch: a labelled parent-space list for one xformable.
    //
    // Detached switch records provide structure; every current frame read is
    // bound by the common graph after provider closure and SSA assignment.
    std::vector<_SpaceSwitch> newSpaceSwitches;
    if(const UsdPrim rig=_stage->GetPrimAtPath(_rigPath)) {
        std::set<SdfPath> targets;
        for(const UsdPrim &prim:UsdPrimRange(rig)) {
            if(prim.GetTypeName()!="RigExecSpaceSwitch")continue;
            const SdfPath path=prim.GetPath();
            const bool defaultCapture = compileScene &&
                compileScene->rigRoot == _rigPath &&
                compileScene->identities.size() == 1 &&
                compileScene->identities[0] == UsdTimeCode::Default();
            if (!defaultCapture || !compileSceneReusable ||
                compileSceneStage != UsdStageWeakPtr(_stage) ||
                compileSceneSerial != GetStageEditSerial()) {
                compileScene=std::make_unique<RigExecSceneDescriptors>();
                const uint64_t before=GetStageEditSerial();
                std::string error;
                if(!RigExecDispatchCaptureSceneDescriptors(_stage,_rigPath,{UsdTimeCode::Default()},compileScene.get(),&error))return fail(error);
                // A notice during capture makes the snapshot ineligible for
                // later reuse; its original switch lowering still runs here.
                compileSceneSerial=before;
                compileSceneReusable=before==GetStageEditSerial();
            }
            auto captured=std::make_shared<RigExecSceneSpaceSwitchDescriptor>();std::string error;
            if(!RigExecDispatchLowerSceneSpaceSwitch(*compileScene,path,captured.get(),&error))return fail(error);
            if(!targets.insert(captured->target).second)
                return fail(captured->target.GetString()+" is the target of more than one space switch");
            _SpaceSwitch record;record.descriptor=captured;record.switchPath=path;record.target=captured->target;
            record.spacePath=captured->space;
            seedProvider(record.target);if(!record.spacePath.IsEmpty())seedProvider(record.spacePath);
            for(const auto &path:captured->sources) {
                _SpaceSwitch::Source source;source.path=path;
                if(!path.IsEmpty())seedProvider(path);
                record.sources.push_back(std::move(source));
            }
            newSpaceSwitches.push_back(std::move(record));
        }
    }

    compileBlocks.Next("PrepareRequests.ProviderClosure");
    // Warming the closure over the seeded set is what grows newPoseInputInfo,
    // which the pass after this one iterates -- so the two are timed apart:
    // this one is stage walking, that one is exec prepares.
    std::vector<SdfPath> seedPaths;
    for (const SdfPath &provider : newNativeProviderPaths) seedPaths.push_back(provider);
    for (const SdfPath &provider : seedPaths)
        for (const SdfPath &member : poseProviderClosure(provider))
            seedProvider(member, false);
    // Close exactly the original first-frame/rest provider roots, separately
    // from native geometry-only source roots. This adds no operation or tap.
    const std::vector<SdfPath> restSeedPaths(newRestEpochProviders.begin(),
                                           newRestEpochProviders.end());
    for (const SdfPath &provider : restSeedPaths) {
        const auto &closure = poseProviderClosure(provider);
        restLatePoseInputs.insert(closure.begin(), closure.end());
    }
    // ORIGINAL seeds the reached connected inputs and their namespace
    // ancestors after the explicit-root closure; it does not close those
    // newly introduced namespace roots a second time.
    for (const SdfPath &input : restLatePoseInputs) seedRestEpochProvider(input);
    // The first-frame/rest request admits only valid USD providers, as the
    // original request preparation did. Geometry-only inputs are not roots
    // of this request and retain their independent availability policy.
    for (const SdfPath &path : newRestEpochProviders) {
        const UsdPrim provider = _stage->GetPrimAtPath(path);
        if (!provider || !UsdPrimDefaultPredicate(provider)) {
            return fail("failed to prepare pose provider inputs");
        }
    }
    compileBlocks.Next("SolverSchedule.SolverBatches");
    // Capture native domain descriptors in authored stack order. Scheduling
    // belongs to the single graph built after every domain is lowered.
    std::vector<SdfPath> authoredPose;
    for(const auto &entry:poseDependencies) authoredPose.push_back(entry.first);
    std::sort(authoredPose.begin(),authoredPose.end(),[&](const SdfPath &a,const SdfPath &b) {
        const int oa=stackOrdinalOf(a),ob=stackOrdinalOf(b);
        return oa!=ob?oa<ob:a<b;
    });
    for(const auto &path:authoredPose) {
        const auto constraint=constraintIndices.find(path);
        if(constraint!=constraintIndices.end()) {
            nativePoseOwners.push_back({false,constraint->second});
            continue;
        }
        if(!requiredSolvers.count(path)) continue;
        _SolverBatch batch;
        const auto &dependencies=newSolverDependencies[path];
        batch.dependencies.insert(dependencies.begin(),dependencies.end());
        const auto &frames=solverFrameInputs[path];
        batch.frameInputs.insert(frames.begin(),frames.end());
        SdfPathVector named;
        if(const auto prim=_stage->GetPrimAtPath(path))
            if(const auto relationship=prim.GetRelationship(TfToken("rigExec:joints"))) relationship.GetTargets(&named);
        const auto predecessors=solverRestPredecessor.find(path);
        if(predecessors!=solverRestPredecessor.end() && !predecessors->second.empty())
            for(const auto &joint:named) {
                const auto previous=predecessors->second.find(joint.GetPrimPath());
                batch.restInputs[joint.GetPrimPath()]=previous==predecessors->second.end()?SdfPath():previous->second;
            }
        const size_t index=newSolverBatches.size();
        batch.solvers.insert(path);
        // D4 concerns the blend's numerical aggregate inputs. A guide-only
        // relationship naming a solver is not an aggregate read, and an
        // unpositioned producer (-1) cannot have run in a later stack batch.
        const auto consumerPrim=_stage->GetPrimAtPath(path);
        const bool readsAggregates=consumerPrim &&
            consumerPrim.GetTypeName()==TfToken("RigExecBlendPointFrames");
        std::set<SdfPath> numericalAggregates;
        if(readsAggregates) for(const char *name:{"rigExec:inputA","rigExec:inputB"}) {
            SdfPathVector inputs;
            const auto relationship=consumerPrim.GetRelationship(TfToken(name));
            if(relationship)relationship.GetTargets(&inputs);
            if(!inputs.empty())numericalAggregates.insert(inputs.front().GetPrimPath());
        }
        const auto aggregates=newSolverAggregateReads.find(path);
        if(readsAggregates && poseStackOrdinal.count(path) &&
           aggregates!=newSolverAggregateReads.end()) for(const auto &source:aggregates->second)
            if(numericalAggregates.count(source) && poseStackOrdinal.count(source) &&
               stackOrdinalOf(source)>=stackOrdinalOf(path)) {
                const std::string warning=path.GetString()+" formerly read the loop-carried aggregate of "+
                    source.GetString()+"; the native graph waits for its same-frame producer";
                if(errors) errors->push_back("warning: "+warning);
                TF_WARN("%s",warning.c_str());
            }
        nativePoseOwners.push_back({true,index});
        newSolverBatches.push_back(std::move(batch));
    }

    // A driver-frames revision requires an actually admitted aggregate
    // batch, not merely a prim of solver type. Refuse only its authored
    // owner so the existing compile retry retains independent revisions.
    std::set<SdfPath> admittedAggregates;
    for (const _SolverBatch &batch : newSolverBatches)
        admittedAggregates.insert(batch.solvers.begin(), batch.solvers.end());
    const auto admitDriverFrames = [&](const auto &chains) {
        for (const auto &[target, revisions] : chains)
            for (const _GraphRevision &revision : revisions) {
                const SdfPath &driver = revision.binding.driverFrames;
                if (!driver.IsEmpty() && !admittedAggregates.count(driver))
                    return fail("driver frames " + driver.GetString() +
                        " is not a solver of this rig; mover set aside",
                        {revision.moverPath});
            }
        return true;
    };
    if (!admitDriverFrames(newGraphChains) ||
        !admitDriverFrames(newGraphDerivedChains)) return false;

    // Authored stack order defines the provider value checkpoints. The common
    // graph later schedules the operations that produce these identities.
    if (!newJointBinding.empty()) {
        std::map<SdfPath, std::vector<SdfPath>> walkChains;
        std::map<SdfPath, std::vector<std::pair<SdfPath, int>>> walkWriters;
        for (const NativePoseOwner &step : nativePoseOwners) {
            if (step.solverBatch) {
                for (const SdfPath &solver :
                     newSolverBatches[step.index].solvers) {
                    const auto joints = newSolverJoints.find(solver);
                    if (joints == newSolverJoints.end()) {
                        continue;
                    }
                    for (const auto &[joint, element] : joints->second) {
                        walkChains[joint].push_back(solver);
                        walkWriters[joint].emplace_back(solver, element);
                    }
                }
                continue;
            }
            const _FrameConstraint &constraint =
                newFrameConstraints[step.index];
            if (!constraint.pointsTarget.IsEmpty()) {
                continue;  // geometry domain: not a frame writer
            }
            for (const SdfPath &target : constraint.targets) {
                // Only joints a solver also writes get a rebuilt chain; a
                // provider only constraints revise already carries exactly
                // this chain from the constraint pass, so leaving it alone
                // keeps every rig without a solver bit-identical.
                if (!newJointBinding.count(target)) continue;
                walkChains[target].push_back(constraint.moverPath);
            }
        }
        for (auto &[joint, writers] : walkWriters) {
            newJointBinding[joint] = std::move(writers);
        }
        for (auto &[joint, chain] : walkChains) {
            newFrameChains[joint] = chain;
        }
    }

    compileBlocks.Close();
    stampCompileRegion("Compile.SolverSchedule");
    // Close the provider universe from connected-expression facts. Values
    // are composed by native domain operations after all producers exist.
    for (const auto &[provider, info] : newPoseInputInfo) seedProvider(provider, false);
    bool newEpochRestsConstant = true;
    std::set<SdfPath> restChainTargets;
    for (const auto &entry : newPropertyChains) restChainTargets.insert(entry.first);
    for (const SdfPath &provider : newRestEpochProviders) {
        if (_ProviderRestMightVary(_stage, _restInputNames, provider,
                                  restChainTargets)) {
            newEpochRestsConstant = false;
            break;
        }
    }

    compileBlocks.Close();
    stampCompileRegion("Compile.PrepareRequests");
    std::map<SdfPath,std::set<SdfPath>> newPhaseCheckpoints;
    _CompileFailure chainFailure;
    if (!_ValidateNativePhases(newGraphChains, newFrameChains, newMovers,
                           &newPhaseCheckpoints, &chainFailure)) {
        return fail(chainFailure.message, std::move(chainFailure.operations));
    }
    if (!volumeWeightError.empty()) {
        return fail(volumeWeightError);
    }
    stampCompileRegion("Compile.ChainOrderValidate");

    // Publish descriptor facts atomically before native graph lowering.
    _movers = std::move(newMovers);
    _jointPaths = std::move(newJointPaths);
    _controlPaths = std::move(newControlPaths);
    _poseInterpolators = std::move(newPoseInterpolators);

    _poseWeightProperties.clear();
    for (const _PoseInterpolator &interpolator : _poseInterpolators) {
        _poseWeightProperties.insert(_poseWeightProperties.end(),
                                     interpolator.disabledPoseWeights.begin(),
                                     interpolator.disabledPoseWeights.end());
        _poseWeightProperties.insert(_poseWeightProperties.end(),
                                     interpolator.poseWeights.begin(),
                                     interpolator.poseWeights.end());
    }
    _frameConstraints = std::move(newFrameConstraints);
    _spaceSwitches = std::move(newSpaceSwitches);
    _frameChains = std::move(newFrameChains);
    _xformDerivedProviders = std::move(newXformDerivedProviders);
    _ribbonDriverPoints = std::move(newRibbonDriverPoints);

    _falloffLutOverrides = std::move(newFalloffLutOverrides);
    _currentPhaseWeights = std::move(newCurrentPhaseWeights);
    _nativeVolumeProviders = std::move(newVolumeWeightProviders);
    _nativeGuideSolvers.clear();
    _nativeGuideSolvers.insert(solverArrayPaths.begin(),solverArrayPaths.end());

    _nativePoseDependencies = std::move(poseDependencies);
    _nativePoseOrdinals = std::move(poseStackOrdinal);

    _nativeProviderPaths = std::move(newNativeProviderPaths);
    _restEpochProviderPaths = std::move(newRestEpochProviders);
    _epochRestsConstant = newEpochRestsConstant;
    _hierarchicalProviderSet.clear();
    _hierarchicalProviderSet.reserve(_nativeProviderPaths.size());
    for (const SdfPath &provider : _nativeProviderPaths)
        _hierarchicalProviderSet.insert(provider);
    // Anchors and intervening-Xform candidates for the new epoch. Both are
    // pure namespace topology; recomputing them per frame cost an xform-cache
    // query per provider only to be told "identity" on every rig that has no
    // such Xform, which is nearly all of them.
    _poseProviderAnchors.clear();
    _interveningXformProviders.clear();
    {
        const SdfPath assetRootPath = _rigPath.GetParentPath();
        for (const SdfPath &provider : _nativeProviderPaths) {
            SdfPath anchorPath;
            for (SdfPath walk = provider.GetParentPath();
                 !walk.IsEmpty() && !walk.IsAbsoluteRootPath() &&
                     walk != assetRootPath;
                 walk = walk.GetParentPath()) {
                if (_nativeProviderPaths.count(walk)) {
                    anchorPath = walk;
                    break;
                }
            }
            _poseProviderAnchors[provider] = anchorPath;
            if (provider.GetParentPath() !=
                (anchorPath.IsEmpty() ? assetRootPath : anchorPath)) {
                _interveningXformProviders.push_back(provider);
            }
        }
    }
    _solverBatches = std::move(newSolverBatches);
    _solverJoints = std::move(newSolverJoints);
    _jointSolverBinding = std::move(newJointBinding);
    _solverDependencies = std::move(newSolverDependencies);
    _nativePhaseCheckpoints = std::move(newPhaseCheckpoints);
    _graphChains = std::move(newGraphChains);
    _graphDerivedChains = std::move(newGraphDerivedChains);
    // Both caches below are keyed by nothing but a path, so the epoch they
    // belong to has to be stated by emptying them when it ends: the
    // influence table a layout was range-checked against and the base points
    // a source holds are both things a recompile can have changed.
    _skinTopologies.Clear();
    _skinLayoutInputsValid = false;
    _blendSampleShapes.Clear();
    _blendSampleSourcesValid = false;
    // Derived results are keyed by target like the live graphs, and the
    // walk may read them from several tasks at once: every entry exists
    // before a generation starts, so no task ever inserts into the map.
    _derivedCache.clear();
    for (const auto &[chainTarget, revisions] : _graphDerivedChains) {
        for (const _GraphRevision &derived : revisions) {
            _derivedCache[derived.target];
        }
    }
    _propertyChains = std::move(newPropertyChains);
    _propertyChainOrder = std::move(newPropertyChainOrder);
    _phasedConnections = std::move(newPhasedConnections);
    // Every attribute a chain READS through a connection.
    //
    // The avar-only notice path below skips dropping the chain bindings, on
    // the reasoning that a float-typed chain input cannot reach a double
    // avar. It can: a connection from a mover's float inputs:value to a
    // control's double avars:ty resolves and is what every "distance from
    // its rest, from the control's own translate" chain in this rig is built
    // out of -- cheeks.py's mouth corner, squetch.py's head wire. Without
    // this set such a chain answered once and then held that answer for the
    // life of the stage, so the cheeks stopped following the mouth corner
    // the moment an avar was AUTHORED rather than dragged.
    _propertyChainInputs.clear();
    for (const auto &[target, revisions] : _propertyChains) {
        for (const _PropertyRevision &revision : revisions) {
            const UsdPrim mover = _stage->GetPrimAtPath(revision.moverPath);
            if (!mover) continue;
            for (const char *name : {"inputs:value", "inputs:min",
                                     "inputs:max"}) {
                const UsdAttribute a = mover.GetAttribute(TfToken(name));
                if (!a) continue;
                for (const SdfPath &source : _AuthoredConnections(a)) {
                    _propertyChainInputs.insert(source);
                }
            }
        }
    }
    // The bindings describe the chains entry for entry, so a recompile that
    // replaced them has replaced what the bindings are about.
    _propertyChainBindings.reset();

    stampCompileRegion("Compile.Commit");
    _compiled = true;

    // Build the dense provider universe once per compile epoch. _providerPaths
    // is SdfPath-sorted, so ascending live-index iteration reproduces the
    // std::map key order the frame-domain publish loops previously relied on.
    {
        std::set<SdfPath> universe;
        auto addU = [&](const SdfPath &p) { if (!p.IsEmpty()) universe.insert(p); };
        for (const auto &path : _nativeProviderPaths) addU(path);
        for (const auto &p : _xformDerivedProviders) addU(p);
        for (const auto &p : _interveningXformProviders) addU(p);
        for (const _FrameConstraint &fc : _frameConstraints) {
            addU(fc.moverPath);
            for (const auto &t : fc.targets) addU(t);
            for (const auto &s : fc.sources) addU(s.sourcePath);
            addU(fc.worldUpObject.sourcePath);
            addU(fc.effector.sourcePath);
            for (const auto &po : fc.poleObjects) addU(po.sourcePath);
            for (const auto &ik : fc.ikChain) addU(ik);
            addU(fc.pointsTarget);
            addU(fc.weightObject);
        }
        for (const auto &kv : _solverJoints) {
            addU(kv.first);
            for (const auto &p : kv.second) addU(p.first);
        }
        for (const _SolverBatch &b : _solverBatches) {
            for (const auto &d : b.dependencies) addU(d);
            for (const auto &d : b.frameInputs) addU(d);
            for (const auto &kv : b.restInputs) { addU(kv.first); addU(kv.second); }
            for (const auto &path : b.solvers) addU(path);
        }
        for (const auto &kv : _nativePhaseCheckpoints) {
            addU(kv.first);
            for (const auto &v : kv.second) addU(v);
        }
        _providerPaths.assign(universe.begin(), universe.end());
        _providerIndex.clear();
        _providerIndex.reserve(_providerPaths.size());
        for (std::size_t i = 0; i < _providerPaths.size(); ++i)
            _providerIndex.emplace(_providerPaths[i], static_cast<int>(i));
        _hierDescendants.assign(_providerPaths.size(), {});
        for (std::size_t i = 0; i < _providerPaths.size(); ++i) {
            if (_nativeProviderPaths.count(_providerPaths[i]) == 0) continue;
            for (std::size_t j = i + 1;
                 j < _providerPaths.size() &&
                 _providerPaths[j].HasPrefix(_providerPaths[i]); ++j)
                if (_nativeProviderPaths.count(_providerPaths[j]))
                    _hierDescendants[i].push_back(static_cast<int>(j));
        }
    }
    _structureDirty = false;
    // Compile the complete native domain graph for this epoch.
    _RebuildBakedProgram(std::move(retiringBakedProgram),
                         compileSceneReusable?compileScene.get():nullptr,
                         compileSceneStage,compileSceneSerial);
    _restEditedProviders.clear();
    {
        // Join the digest task: from here on its result is read. Last, and
        // after the bake rather than ahead of the commit where it used to
        // sit: _ComputeStructureDigest reads only the stage and the
        // profiler, nothing this compile commits, so nothing in between
        // needs its answer -- and joined here the bake no longer queues
        // behind it on the rigs where the digest is the longer lane.
        RIGEXEC_PROFILE_SCOPE_CAT(_profiler, "Compile.DigestJoin", "compile");
        digestDispatcher.Wait();
        _structureDigest = _stage ? _JoinStructureDigest(digestParts) : 0;
        digestJoined = true;
    }
    if (_stage) {
        RIGEXEC_PROFILE_SCOPE_CAT(_profiler, "Compile.DigestGate", "compile");
        _digestGate = _MakeDigestGate(digestFootprints);
    }
    return true;
}

bool
RigExecRigEvaluator::_FailedCompileMemoMatches() const
{
    return _failedCompile.valid &&
        _failedCompile.stageEditSerial == _stageEditSerial;
}

bool
RigExecRigEvaluator::_CompileUnlessKnownBroken(
    std::vector<std::string> *diagnostics)
{
    // A compile that failed fails again for as long as nothing it read has
    // moved, and what it reads is the stage and the mode. So a failure is
    // answered from the memo while the key still matches: the same errors,
    // in the same order, where the compile would have put them, at the cost
    // of a key compare instead of a full compile (plus, at the structural
    // site, the digest ahead of it) on every frame of a scrub over a stage
    // somebody left broken.
    // The documented difference is what the compile says OUTSIDE the vector:
    // its TF_WARNs -- a derived start-frame note, the transform-authority
    // pass -- go to the host once, on the compile that failed, rather than
    // once per frame; and the program's build and attempt counters stand
    // still, because nothing is built or attempted.
    if (_FailedCompileMemoMatches()) {
        RIGEXEC_PROFILE_SCOPE_CAT(_profiler, "Settle.KnownBroken",
                                  "evaluate");
        if (diagnostics) {
            diagnostics->insert(diagnostics->end(),
                                _failedCompile.errors.begin(),
                                _failedCompile.errors.end());
        }
        return false;
    }
    // Compile's own lines, kept apart so that the memo holds exactly what a
    // compile says and never what the caller says around it ("structural
    // recompilation failed" is the settle's, and it adds that itself on a
    // replay as it does here).
    std::vector<std::string> errors;
    const bool ok = _CompileEpoch(&errors);
    if (diagnostics) {
        diagnostics->insert(diagnostics->end(), errors.begin(), errors.end());
    }
    if (ok) {
        _failedCompile = _FailedCompileMemo();
        return true;
    }
    // Keyed AFTER the compile, on what the next frame will compare against:
    // the compile authors its derived start frames under a notice block, so
    // the serial it leaves is the stage it read, and the mode is read the
    // same way the next settle reads it.
    _failedCompile.valid = true;
    _failedCompile.stageEditSerial = _stageEditSerial;


    _failedCompile.errors = std::move(errors);
    return false;
}

bool
RigExecRigEvaluator::_SettleEpoch(std::vector<std::string> *diagnostics)
{
    if (!_compiled && !_CompileUnlessKnownBroken(diagnostics)) {
        return false;
    }
    // Structural edits begin a new epoch: recompile when the composed
    // mover topology digest changed (spec §4.2, §6.3).
    // A standing failure memo answers ahead of the digest. It can only match
    // here when the failure it holds was this same structural recompile:
    // no notice has arrived since (the serial is the key), so the digest
    // would come out as it did then -- different from the committed one,
    // which a failed compile never replaces -- and the compile it leads to
    // would be answered from the memo anyway.
    // A digest that comes out equal commits the footprint it recorded: the
    // stage it read is the one the next notice will be judged against. One
    // that differs leaves that to the compile it leads to.
    // An edit that is CERTAINLY structural skips the digest here (rule T-e):
    // a prim the committed digest listed for its type, or a list it wrote,
    // has changed, so the digest has moved and would only be computed to
    // say so -- the compile computes it again anyway, off its critical path.
    // The rebuild line then waits for the compile's digest, and says what
    // it says on the digest path: that the digest moved. An edit that came
    // out equal was a false positive, which RIGEXEC_VERIFY_CERTAIN_STRUCTURAL
    // counts.
    bool recompiled = false;
    bool digestMoved = false;
    bool certain = false;
    bool replayed = false;
    if (_structureDirty) {
        replayed = _FailedCompileMemoMatches();
        digestMoved = replayed;
        if (!digestMoved) {
            {
                RIGEXEC_PROFILE_SCOPE_CAT(_profiler, "Settle.CertainStructural",
                                          "evaluate");
                certain = _EditIsCertainlyStructural();
            }
            if (certain) {
                digestMoved = true;
            } else {
                _DigestGate gate;
                digestMoved =
                    _SettleStructureDigest(&gate) != _structureDigest;
                if (!digestMoved) {
                    _digestGate = std::move(gate);
                }
            }
        }
    }
    if (digestMoved) {
        const size_t digestBefore = _structureDigest;
        _CertainStructuralCounts *const counts =
            _CertainStructuralVerifyCounts();
        if (counts && !replayed) {
            ++(certain ? counts->tagged : counts->digestFound);
        }
        if (!_CompileUnlessKnownBroken(diagnostics)) {
            return false;
        }
        recompiled = true;
        if (!certain || _structureDigest != digestBefore) {
            diagnostics->push_back("structural edit: epoch rebuilt");
        } else if (counts) {
            ++counts->falsePositives;
            std::fprintf(stderr,
                         "RIGEXEC_VERIFY_CERTAIN_STRUCTURAL: false positive "
                         "on <%s>: the edit taken for certainly structural "
                         "left the structure digest where it was\n",
                         _rigPath.GetText());
            std::fflush(stderr);
        }
    }
    _structureDirty = false;
    _certainCandidates.clear();
    _certainCandidatesOverflowed = false;
    _restEditedProviders.clear();
    return true;
}

} // namespace rigExec
