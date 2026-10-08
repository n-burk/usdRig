// Lifecycle, compiled program ownership, and generation dispatch.

#include "rigEvaluatorInternal.h"
#include "rigEvaluatorDependencies.h"
#include "rigEvaluatorPropertyBindings.h"
#include "parallel.h"
#include "goldenSuite.h"
#include "inputReplay.h"

#include "pxr/base/work/detachedTask.h"
#include "pxr/base/tf/diagnostic.h"
#include "pxr/base/tf/getenv.h"
#include "pxr/base/tf/notice.h"
#include "pxr/base/tf/pyLock.h"
#include "pxr/base/tf/stringUtils.h"
#include "pxr/usd/usd/attribute.h"
#include "pxr/usd/usd/prim.h"

#include <algorithm>

namespace rigExec {

using namespace evaluatorDetail;

namespace {

// RIGEXEC_VERIFY_SCOPED_CLEARS=1: every evaluator builds a shadow of itself
// whose notices drop the value caches whole, evaluates it beside every
// generation, and reports each way the two poses differ as a baked parity
// mismatch (spec rule S6). The cone verifier cannot stand in for this: it
// captures the program after the prologue, which is where a stale cache
// would already have been read. Read once per process.
bool
_ScopedClearShadowRequested()
{
    static const bool requested =
        TfGetenvBool("RIGEXEC_VERIFY_SCOPED_CLEARS", false);
    return requested;
}

/// Destroys a program the evaluator has replaced, off this thread.
///
/// A retiring program is several milliseconds of frees on the biped -- every
/// step, table, query and geometry buffer of an epoch -- and nothing waits on
/// them: once the replacement exists, no frame, notice or tool can reach the
/// old one again. So the destroy is detached rather than paid on the thread
/// that is recompiling.
///
/// Two rules make that safe. The stage references go FIRST, here, so the
/// detached delete can never be the one that drops a stage's last reference
/// and tears it down on a worker; what remains (prims, attributes, queries,
/// shared layouts) holds atomic refcounts and owns nothing of the stage. And
/// the destroy touches nothing but the program: the evaluator pointers it
/// carries are never followed by its destructor, so the evaluator may itself
/// be gone by the time it runs. It owns no exec object either -- the guide
/// taps it reads are the evaluator's -- so the single exec lane is not
/// entered.
///
/// Inline where a task is not allowed: with parallel evaluation off, and
/// inside a frozen run, whose thread must dispatch nothing.
void
_RetireBakedProgram(std::unique_ptr<RigExecBakedProgram> retiring)
{
    if (!retiring || !RigExecParallelEvaluationEnabled() ||
        RigExecFrozenSerialActive()) {
        return;
    }
    retiring->ReleaseStageReferences();
    // A raw pointer, because a detached task is invoked through a const call
    // operator and so cannot move out of a capture it owns.
    WorkRunDetachedTask([program = retiring.release()] { delete program; });
}

} // namespace

RigExecRigEvaluator::RigExecRigEvaluator(
    const UsdStageRefPtr &stage, const SdfPath &rigPath)
    : _stage(stage), _rigPath(rigPath)
    , _restInputNames(evaluatorDetail::_MakeRestInputNames())
{
    _inputReplayObserver = RigExecInputReplayObserver::Create(stage, rigPath);
    _goldenSuiteObserver = RigExecGoldenSuiteObserver::Create(rigPath);
    // The static-input cache lives here and is consulted through the
    // resolved-input lookup every read already goes through.
    _resolvedInputs.SetStaticCache(&_staticInputs);
    if (_stage) {
        _noticeKey = TfNotice::Register(
            TfCreateWeakPtr(this), &RigExecRigEvaluator::_OnObjectsChanged,
            UsdStageWeakPtr(_stage));
    }
    // The rig evaluates directly against the source stage. There used to be
    // a private derived stage here, holding an anonymous session sublayer
    // for the compiler's generated property applications; the engine authors
    // nothing now, so there is nothing to hold and no stage to derive.
}

RigExecRigEvaluator::~RigExecRigEvaluator()
{
    _inputReplayObserver.reset();
    TfNotice::Revoke(_noticeKey);
}

void
RigExecRigEvaluator::_EnsureScopedClearShadow()
{
    if (_scopedClearShadow || _wholesaleValueClears ||
        !_ScopedClearShadowRequested()) {
        return;
    }
    // Built before this evaluator compiles or evaluates anything the shadow
    // has not: Compile and Evaluate call this first. Whatever a caller set
    // before then is handed over here, in the order it would have been set
    // on a fresh evaluator; everything after is mirrored as it happens.
    RigExecInputReplaySuppression suppressInputReplay;
    auto shadow = std::make_unique<RigExecRigEvaluator>(
        _stage, _rigPath);
    shadow->_wholesaleValueClears = true;
    shadow->SetPublishWeightFields(_publishWeightFields);
    shadow->SetSolverGuidesEnabled(_solverGuidesEnabled);
    if (!_interactiveOverrides.empty()) {
        shadow->SetInteractiveOverrides(_interactiveOverrides);
    }
    if (!_upstreamRequested.empty()) {
        shadow->SetUpstreamInputs(_upstreamRequested);
    }
    _scopedClearShadow = std::move(shadow);
}

void
RigExecRigEvaluator::_VerifyScopedClears(UsdTimeCode time,
                                         RigExecRigPose *pose)
{
    // A public field rather than a setter, so it is handed over per call.
    _scopedClearShadow->cpuReference = cpuReference;
    const RigExecRigPose shadow = _scopedClearShadow->Evaluate(time);
    // Every domain --pose-out writes. RigExecComparePoses covers the maps,
    // the ordered diagnostics and the work counters; the scalars it leaves
    // out on purpose, because the two paths it was written for disagree
    // about them, are the same path here and must agree too.
    RigExecRigPose differences;
    RigExecComparePoses(shadow, *pose, &differences);
    const auto scalar = [&differences](size_t wholesale, size_t scoped,
                                       const char *what) {
        if (wholesale != scoped) {
            differences.diagnostics.push_back(
                std::string("baked parity: ") + what + " differs (" +
                std::to_string(wholesale) + " vs " + std::to_string(scoped) +
                ")");
            ++differences.comparisonMismatches;
        }
    };
    scalar(shadow.valid, pose->valid, "valid");
    scalar(shadow.comparisonMismatches, pose->comparisonMismatches,
           "baked parity mismatches");
    scalar(shadow.referenceMismatches,
           pose->referenceMismatches, "mover graph parity mismatches");
    scalar(shadow.referenceAgreements,
           pose->referenceAgreements, "mover graph parity agreements");
    if (shadow.movedPropertiesCpu != pose->movedPropertiesCpu) {
        differences.diagnostics.push_back(
            "baked parity: CPU moved properties differ");
        ++differences.comparisonMismatches;
    }
    if (differences.comparisonMismatches == 0) {
        return;
    }
    for (const std::string &line : differences.diagnostics) {
        pose->diagnostics.push_back("scoped cache clears: " + line);
    }
    pose->comparisonMismatches += differences.comparisonMismatches;
    // The wording a parity disagreement uses, so one expression catches it.
    TF_WARN("rigExec: baked parity mismatch: %zu difference(s) between the "
            "scoped cache clears and a shadow evaluator that clears them "
            "whole, at time %s: %s",
            differences.comparisonMismatches, TfStringify(time).c_str(),
            TfStringJoin(differences.diagnostics, "; ").c_str());
}

RigExecRigPose
RigExecRigEvaluator::Evaluate(UsdTimeCode time)
{
    if (_inputReplayObserver) {
        _inputReplayObserver->Options(_solverGuidesEnabled, _publishWeightFields);
        _inputReplayObserver->BeginEvaluate(time);
    }
    struct ReplayEvaluationScope {
        RigExecInputReplayObserver *observer;
        ~ReplayEvaluationScope() { if (observer) observer->EndEvaluate(); }
    } replayScope{_inputReplayObserver.get()};
    _EnsureScopedClearShadow();
    RigExecRigPose pose = _EvaluateGeneration(time);
    if (_scopedClearShadow) {
        _VerifyScopedClears(time, &pose);
    }
    if (_goldenSuiteObserver) _goldenSuiteObserver->Record(pose);
    return pose;
}

void
RigExecRigEvaluator::SetPublishWeightFields(bool publish)
{
    if (_inputReplayObserver)
        _inputReplayObserver->Options(_solverGuidesEnabled, publish);
    _publishWeightFields = publish;
    if (_scopedClearShadow) _scopedClearShadow->SetPublishWeightFields(publish);
}

void
RigExecRigEvaluator::SetSolverGuidesEnabled(bool enabled)
{
    if (_inputReplayObserver)
        _inputReplayObserver->Options(enabled, _publishWeightFields);
    _solverGuidesEnabled = enabled;
    if (_scopedClearShadow) _scopedClearShadow->SetSolverGuidesEnabled(enabled);
}

RigExecRigPose
RigExecRigEvaluator::_EvaluateGeneration(UsdTimeCode time)
{
    TF_PY_ALLOW_THREADS_IN_SCOPE();
    _lastGenerationRanProgram=false;
    RIGEXEC_PROFILE_SCOPE_CAT(_profiler,time.IsDefault()?std::string("Evaluate@default"):
        "Evaluate@"+TfStringPrintf("%g",time.GetValue()),"evaluate");
    RigExecRigPose pose;pose.time=time;
    // Preserve the observable rest-classification transition at this owner
    // boundary. The standing program supplies the epoch's chain authority;
    // the existing rebuild below remains the sole binding rebuild.
    const size_t buildsBeforeSettle = _bakedProgramBuilds;
    bool restBecameVarying = false;
    if (_compiled && _epochRestsConstant && _bakedProgramStale &&
        !_restEditedProviders.empty()) {
        std::set<SdfPath> chainTargets;
        for (const auto &entry : _propertyChains) chainTargets.insert(entry.first);
        for (const SdfPath &provider : _restEditedProviders) {
            if (_ProviderRestMightVary(_stage, _restInputNames, provider,
                                      chainTargets)) {
                restBecameVarying = true;
                break;
            }
        }
    }
    {
        RIGEXEC_PROFILE_SCOPE_CAT(_profiler,"Evaluate.Settle","evaluate");
        if(!_SettleEpoch(&pose.diagnostics))return pose;
    }
    if(_bakedProgramStale) {
        // A full structural recompile already published its own epoch reason.
        const bool restTransition = restBecameVarying &&
            _bakedProgramBuilds == buildsBeforeSettle;
        _bakedProgramStale=false;
        _RebuildBakedProgram(std::move(_bakedProgram));
        if (restTransition && _bakedProgram) {
            // ORIGINAL moves every rest channel to per-frame classification
            // after this first successful varying epoch transition.
            _epochRestsConstant = false;
            pose.diagnostics.push_back(
                "rest channel became time-varying: epoch rebuilt");
        }
    }
    if(_compiled && !_bakedProgram && !_bakeRefused)_RebuildBakedProgram();
    if(!_upstreamRequested.empty() || !_upstreamAdmitted.empty()) {
        _AdmitUpstreamInputs(time);
        pose.diagnostics.insert(pose.diagnostics.end(),_upstreamDropLines.begin(),_upstreamDropLines.end());
    }
    if(!_bakedProgram) {
        pose.diagnostics.push_back("native program unavailable for compiled epoch");
        pose.diagnostics.insert(pose.diagnostics.end(),_bakeRefusalReasons.begin(),_bakeRefusalReasons.end());
        return pose;
    }
    {
        RIGEXEC_PROFILE_SCOPE_CAT(_profiler,"Evaluate.PlaceOverrides","evaluate");
        _bakedProgram->SetUpstreamInputs(_upstreamAdmitted);
        if(!_bakedProgram->SetOverrides(_interactiveOverrides)) {
            pose.diagnostics.push_back("native program refused interactive inputs");
            return pose;
        }
        _bakedProgram->SetPublishWeightFields(_publishWeightFields);
    }
    bool completed=false;
    {
        RIGEXEC_PROFILE_SCOPE_CAT(_profiler,"Evaluate.Run","evaluate");
        completed=_bakedProgram->Run(time,&pose);
    }
    if(!completed) {
        pose.valid=false;
        pose.diagnostics.push_back("native program did not complete generation");
        return pose;
    }
    _bakedProgram->AdoptBlendSampleLayouts(&_blendSampleShapes);
    _skinTopologyObservationPending = false;
    ++_bakedGenerations;
    _bakedProgramPublished=true;
    _lastGenerationRanProgram=true;
    return pose;
}

void
RigExecRigEvaluator::_RebuildBakedProgram(
    std::unique_ptr<RigExecBakedProgram> outgoing,
    const RigExecSceneDescriptors *scene, const UsdStageWeakPtr &sceneStage,
    uint64_t sceneSerial)
{
    // Read before anything can replace the program it describes, and cleared
    // here because from this line on no program of this evaluator has
    // published anything.
    const bool outgoingPublished = _bakedProgramPublished;
    _bakedProgramPublished = false;
    // The replacement has answered no generation, so the op trace and op
    // graph accessors describe none until it runs one.
    _lastGenerationRanProgram = false;
    RIGEXEC_PROFILE_SCOPE_CAT(_profiler, "Compile.Bake", "compile");
    ++_bakedProgramBuildAttempts;
    // Preserve concrete compile-admission reasons for an invalid generation.
    std::vector<std::string> reasons;
    _bakedProgram = RigExecBakedProgram::_BuildWithSceneCapture(
        this, &reasons, scene, sceneStage, sceneSerial);
    if (_bakedProgram) {
        ++_bakedProgramBuilds;
        std::map<SdfPath, SdfPath> switchOwners;
        for (const auto &sw : _spaceSwitches)
            switchOwners.emplace(sw.target, sw.switchPath);
        for (const auto &[owner, reason] : _bakedProgram->_GetCycleSkips(switchOwners))
            _skippedOperations.emplace(owner, reason);
        if (outgoing && outgoingPublished) {
            // The replacement inherits the geometry nodes that survive,
            // matched the way the dynamic walk matches its VdfNetwork nodes.
            // Without it a rebuilt program re-runs every per-point kernel and
            // publishes "created"/"schedule(s) built" counters, and the mover
            // graph diagnostic, for nodes nothing rebuilt -- while the
            // dynamic path, whose graphs stood through the same edit, reports
            // none of it.
            // Only from a program that PUBLISHED a generation, which is what
            // makes the sentence above true: the whole of what an unrun
            // program carries here is `created = false` on nodes whose
            // creation no consumer has been told about, so adopting it makes
            // the replacement under-report work the dynamic path -- whose
            // graphs are still cold -- goes on to report. The case is reached
            // by building at Compile and changing the mode afterwards, which
            // is what a rig carrying rigExec:baked does whenever a tool then
            // asks for the parity check.
            _bakedProgram->AdoptGeometryStateFrom(*outgoing);
        }
    }
    // Refusing is a property of the epoch, not of the moment: remember it so
    // the lazy build in Evaluate asks once rather than once per frame. The
    // reasons ride with it: the fallback that reports them happens per
    // frame, long after the one build that could say why.
    _bakeRefused = !_bakedProgram;
    _bakeRefusalReasons = std::move(reasons);
    // Last, once the replacement has taken what it adopts from it.
    _RetireBakedProgram(std::move(outgoing));
}

bool
RigExecRigEvaluator::IsBakeable(std::vector<std::string> *reasons) const
{
    return RigExecBakedProgram::IsBakeable(*this, reasons);
}

size_t
RigExecRigEvaluator::GetBakedClusterCount() const
{
    return _bakedProgram ? _bakedProgram->GetClusterCount() : 0;
}

size_t
RigExecRigEvaluator::GetBakedClustersRunLastGeneration() const
{
    return _bakedProgram ? _bakedProgram->GetClustersRunLastGeneration() : 0;
}

} // namespace rigExec
