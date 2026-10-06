// Lifecycle, evaluation-mode selection, and generation dispatch.

#include "rigEvaluatorInternal.h"
#include "rigEvaluatorDependencies.h"
#include "rigEvaluatorPropertyBindings.h"
#include "parallel.h"

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

// What RIGEXEC_EVALUATION_MODE asked this process for, if anything.
// Not a code path: it selects the initial value of a setting callers can set
// themselves, so nothing here behaves differently for having been reached
// through the environment. It exists so an EXISTING suite can be re-run under
// the baked mode without every test in it learning about the mode -- which is
// the only way to check the program against the several hundred rigs those
// suites already build. Unset means nothing was asked and the rig's own
// rigExec:baked gets to answer instead.
// `reference` pins ExecReference, the exec-authoritative oracle, so that a
// suite written against the oracle can be re-run against it by name rather
// than against whatever Dynamic runs.
// `authored` is the half the mode alone cannot carry, and it is about
// PRECEDENCE rather than about the value: an unset variable and
// RIGEXEC_EVALUATION_MODE=dynamic both mean Dynamic, and only the second is
// an instruction -- a suite that sets it is saying "every stage this process
// opens runs dynamically", including one whose attribute asks for the
// program. An unrecognised value counts as authored for the same reason: a
// typo must not silently hand the decision back to the stage, so it warns,
// means dynamic, and still outranks the attribute.
// Read ONCE per process, at the construction of the first evaluator, and
// fixed from then on: the function-local static below is initialised on its
// first call and never re-reads the environment. Changing the variable after
// that -- with setenv, or between two tests in one binary -- changes nothing;
// SetEvaluationMode is the only way to move an evaluator afterwards, and it
// moves that evaluator alone.
struct _EnvironmentMode {
    bool authored = false;
    RigExecEvaluationMode mode = RigExecEvaluationMode::Dynamic;
};

const _EnvironmentMode &
_EnvironmentEvaluationMode()
{
    static const _EnvironmentMode requested = [] {
        _EnvironmentMode result;
        const std::string value = TfGetenv("RIGEXEC_EVALUATION_MODE", "");
        result.authored = !value.empty();
        if (value == "baked") {
            result.mode = RigExecEvaluationMode::Baked;
        } else if (value == "parity") {
            result.mode = RigExecEvaluationMode::BakedWithParityCheck;
        } else if (value == "reference") {
            result.mode = RigExecEvaluationMode::ExecReference;
        } else if (result.authored && value != "dynamic") {
            TF_WARN("rigExec: RIGEXEC_EVALUATION_MODE=%s is not one of "
                    "dynamic, baked, parity, reference; using dynamic",
                    value.c_str());
        }
        return result;
    }();
    return requested;
}

// The attribute a rig asks for the baked program with (schema.usda,
// RigExecRoot). Uniform and epoch-level: it decides which path answers the
// rig, not what any frame of it is.
const TfToken &
_BakedAttributeName()
{
    static const TfToken name("rigExec:baked");
    return name;
}

// Whether a fallback to the dynamic path is a FAILURE.
// Not a code path either: it changes no evaluated value and no dispatch,
// only whether a generation that ran dynamically while the mode asked for
// the program says so on the pose it publishes. It exists because a suite
// whose fixtures all decline the bake reports zero parity mismatches and
// goes green having compared nothing -- which is the one way a parity run
// can lie, and the way it lies about exactly the rigs a new operator was
// supposed to make bakeable.
// Read ONCE per process, in a function-local static for the same reason the
// mode above is: a tool sets it before the first evaluator exists, and
// nothing may change the answer between two tests in one binary.
bool
_BakeRequired()
{
    static const bool required =
        TfGetenvBool("RIGEXEC_BAKE_REQUIRED", false);
    return required;
}

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

bool
RigExecDynamicRunsProgram()
{
    // A function-local static for the reason _EnvironmentEvaluationMode
    // gives: one answer per process, fixed at the first evaluator.
    static const bool runs =
        TfGetenvBool("RIGEXEC_DYNAMIC_RUNS_PROGRAM", false);
    return runs;
}

bool
RigExecEvaluationModeRunsProgram(RigExecEvaluationMode mode,
                                 RigExecEvaluationModeSource source)
{
    switch (mode) {
    case RigExecEvaluationMode::Baked:
    case RigExecEvaluationMode::BakedWithParityCheck:
        return true;
    case RigExecEvaluationMode::ExecReference:
        return false;
    case RigExecEvaluationMode::Dynamic:
        break;
    }
    // Each way Dynamic can be chosen, named, so that the day one of them
    // stops following the switch (an authored rigExec:baked = false coming
    // to mean ExecReference is the open one) is a change to its own line.
    switch (source) {
    case RigExecEvaluationModeSource::Default:
    case RigExecEvaluationModeSource::Attribute:
    case RigExecEvaluationModeSource::Environment:
    case RigExecEvaluationModeSource::Explicit:
        return RigExecDynamicRunsProgram();
    }
    return false;
}

RigExecRigEvaluator::RigExecRigEvaluator(
    const UsdStageRefPtr &stage, const SdfPath &rigPath)
    : RigExecRigEvaluator(stage, rigPath, false)
{
}

RigExecRigEvaluator::RigExecRigEvaluator(
    const UsdStageRefPtr &stage, const SdfPath &rigPath, bool preferProgram)
    : _stage(stage)
    , _rigPath(rigPath)
    , _restInputNames(evaluatorDetail::_MakeRestInputNames())
    , _preferProgram(preferProgram)
    , _evaluationMode(_EnvironmentEvaluationMode().mode)
    , _evaluationModeSource(_EnvironmentEvaluationMode().authored
                                ? RigExecEvaluationModeSource::Environment
                                : RigExecEvaluationModeSource::Default)
{
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
    TfNotice::Revoke(_noticeKey);
    _guideTaps.reset();
    _taps.reset();
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
    auto shadow = std::make_unique<RigExecRigEvaluator>(
        _stage, _rigPath, _preferProgram);
    shadow->_wholesaleValueClears = true;
    if (_evaluationModeSource == RigExecEvaluationModeSource::Explicit) {
        shadow->SetEvaluationMode(_evaluationMode);
    }
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
    _scopedClearShadow->cpuParityMode = cpuParityMode;
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
            ++differences.bakedParityMismatches;
        }
    };
    scalar(shadow.valid, pose->valid, "valid");
    scalar(shadow.solverEvaluations, pose->solverEvaluations,
           "solver evaluations");
    scalar(shadow.bakedParityMismatches, pose->bakedParityMismatches,
           "baked parity mismatches");
    scalar(shadow.moverGraphParityMismatches,
           pose->moverGraphParityMismatches, "mover graph parity mismatches");
    scalar(shadow.moverGraphParityAgreements,
           pose->moverGraphParityAgreements, "mover graph parity agreements");
    if (shadow.movedPropertiesCpu != pose->movedPropertiesCpu) {
        differences.diagnostics.push_back(
            "baked parity: CPU moved properties differ");
        ++differences.bakedParityMismatches;
    }
    if (differences.bakedParityMismatches == 0) {
        return;
    }
    for (const std::string &line : differences.diagnostics) {
        pose->diagnostics.push_back("scoped cache clears: " + line);
    }
    pose->bakedParityMismatches += differences.bakedParityMismatches;
    // The wording a parity disagreement uses, so one expression catches it.
    TF_WARN("rigExec: baked parity mismatch: %zu difference(s) between the "
            "scoped cache clears and a shadow evaluator that clears them "
            "whole, at time %s: %s",
            differences.bakedParityMismatches, TfStringify(time).c_str(),
            TfStringJoin(differences.diagnostics, "; ").c_str());
}

RigExecRigPose
RigExecRigEvaluator::Evaluate(UsdTimeCode time)
{
    _EnsureScopedClearShadow();
    RigExecRigPose pose = _EvaluateGeneration(time);
    if (_scopedClearShadow) {
        _VerifyScopedClears(time, &pose);
    }
    return pose;
}

RigExecRigPose
RigExecRigEvaluator::_EvaluateGeneration(UsdTimeCode time)
{
    // The GIL is given up for the whole generation, before anything below
    // can dispatch. Every path through here joins TBB workers -- the settle's
    // digest, the program's clusters, the pose-provider pulls -- and a worker
    // that is the first in the process to reach an exec definition loads its
    // plugin through TfScriptModuleLoader, which takes the GIL. A caller that
    // holds it while this thread waits on that worker deadlocks both. The
    // Python binding releases it itself, but a C++ host need not: Hydra's
    // Render is entered from Python with the GIL held and reaches here through
    // the imaging bridge. Compile keeps its own guard, being public too; the
    // ones in the deferred exec prep and the dynamic walk sit under this one,
    // where a guard finds the GIL already released and does nothing.
    TF_PY_ALLOW_THREADS_IN_SCOPE();
    _lastGenerationRanProgram = false;
    RIGEXEC_PROFILE_SCOPE_CAT(
        _profiler,
        time.IsDefault()
            ? std::string("Evaluate@default")
            : "Evaluate@" + TfStringPrintf("%g", time.GetValue()),
        "evaluate");
    // Settle the epoch first: choosing a path before knowing whether the rig
    // still compiles to the same one is choosing it blind, and it is also
    // what would otherwise make the first frame of every session dynamic.
    std::vector<std::string> settled;
    bool settledOk = false;
    {
        RIGEXEC_PROFILE_SCOPE_CAT(_profiler, "Evaluate.Settle", "evaluate");
        settledOk = _SettleEpoch(&settled);
    }
    if (!settledOk) {
        RigExecRigPose failed;
        failed.time = time;
        failed.diagnostics = std::move(settled);
        return failed;
    }
    // A notice hit the program's capture index, so what it folded in is no
    // longer what the stage says. A structural edit would already have
    // rebuilt it through Compile just now; this covers the edit that moved a
    // value and nothing else, which is the case the digest cannot see.
    if (_bakedProgramStale) {
        _bakedProgramStale = false;
        // Handed over rather than dropped: the op list is what the edit
        // invalidated, not the geometry state around it, and starting that
        // over re-runs every per-point kernel and reports nodes as built
        // that were never rebuilt.
        _RebuildBakedProgram(std::move(_bakedProgram));
    }
    // The mode was asked for while the epoch was dirty -- a scene edit
    // between the compile and the request, which is every request made by a
    // UI after the artist has touched anything. SetEvaluationMode could not
    // build then, because the epoch it would have baked was about to be
    // re-settled; now it has been, so build here. Once per epoch: a rig the
    // program cannot express refuses for reasons the epoch fixes, and
    // re-asking every frame pays for the refusal every frame. A program that
    // bailed is not rebuilt while its memo stands, for the same reason.
    if (_ModeRunsProgram() && _compiled && !_bakedProgram && !_bakeRefused &&
        !_BakeBailMemoMatches()) {
        _RebuildBakedProgram();
    }
    // Upstream values are admitted against the epoch and program that now
    // stand, and every generation, either path, reports the keys it drops.
    if (!_upstreamRequested.empty() || !_upstreamAdmitted.empty()) {
        _AdmitUpstreamInputs();
        settled.insert(settled.end(), _upstreamDropLines.begin(),
                       _upstreamDropLines.end());
    }
    // An override the program cannot place would make it answer a question
    // nobody asked; that generation runs dynamically instead.
    bool overridesPlaceable = true;
    {
        RIGEXEC_PROFILE_SCOPE_CAT(_profiler, "Evaluate.PlaceOverrides", "evaluate");
        if (_bakedProgram) {
            _bakedProgram->SetUpstreamInputs(_upstreamAdmitted);
        }
        overridesPlaceable = !_bakedProgram ||
            _bakedProgram->SetOverrides(_interactiveOverrides);
        if (_bakedProgram) {
            _bakedProgram->SetPublishWeightFields(_publishWeightFields);
        }
    }
    // cpuParityMode publishes an independent scalar oracle for the geometry
    // chains. The program is not that oracle -- it shares the kernels -- so
    // asking for the oracle asks for the dynamic path.
    const bool runBaked = _bakedProgram && !cpuParityMode &&
        overridesPlaceable && _ModeRunsProgram();
    if (!runBaked) {
        // Read before the walk, which is free to move the stage serial: this
        // is the memo that decided there would be no program.
        const bool bailMemoized = _BakeBailMemoMatches();
        RigExecRigPose dynamic = _EvaluateDynamic(time, std::move(settled));
        // cpuParityMode is not a fallback: it ASKS for the dynamic path, so
        // a rig that bakes perfectly still runs here and has nothing to
        // report. The other two ways in do: either no program was built, or
        // one was and could not answer the question the drag asks.
        if (!cpuParityMode && _FallbackIsWorthAnnouncing()) {
            // One reason, two audiences: the harness reads the first line
            // and an artist reads the second, and a generation that fell
            // back for one reason must not be able to name two.
            // A memoized bail names the failure the program last gave, which
            // is what the rebuild it stands in for would have said again.
            const std::string why =
                !overridesPlaceable
                    ? std::string("interactive overrides are not placeable")
                : bailMemoized
                    ? std::string("program run failed")
                : _bakeRefusalReasons.empty()
                    ? std::string("no baked program")
                    : _bakeRefusalReasons.front();
            _ReportBakeRequired(why, &dynamic);
            _ReportAttributeBakeFallback(why, &dynamic);
        }
        return dynamic;
    }
    RigExecRigPose baked;
    baked.time = time;
    baked.diagnostics = settled;
    bool ranBaked = false;
    {
        RIGEXEC_PROFILE_SCOPE_CAT(_profiler, "Evaluate.Run", "evaluate");
        ranBaked = _bakedProgram->Run(time, &baked);
    }
    // An unresolvable constraint target is the one bail the program answers
    // itself: the walk gives the generation back at the same point, so the
    // invalid pose the run left is the walk's answer, and the program --
    // whose region never ran -- stays (rule D3).
    const RigExecBakedBail bail =
        ranBaked ? RigExecBakedBail::None : _bakedProgram->GetLastBail();
    if (!ranBaked && bail != RigExecBakedBail::StageFrames) {
        // The program handed the generation back mid-flight, so its per-frame
        // caches no longer describe a completed frame. Drop it rather than
        // reuse it, and answer from the path that cannot decline -- at those
        // points the walk still returns a valid pose.
        _bakedProgram.reset();
        _bakedProgramPublished = false;
        _lastGenerationRanProgram = false;
        RigExecRigPose dynamic = _EvaluateDynamic(time, std::move(settled));
        // Keyed AFTER the walk, on the serial the next frame compares
        // against, as the failed-compile memo is.
        _bakeBail.valid = true;
        _bakeBail.stageEditSerial = _stageEditSerial;
        if (_FallbackIsWorthAnnouncing()) {
            _ReportBakeRequired("program run failed", &dynamic);
            _ReportAttributeBakeFallback("program run failed", &dynamic);
        }
        return dynamic;
    }
    ++_bakedGenerations;
    // A generation given back at the stage frames ran no geometry, so it
    // reported nothing a rebuild could inherit.
    if (ranBaked) {
        _bakedProgramPublished = true;
        _lastGenerationRanProgram = true;
    }
    // Baked, and Dynamic running the program, publish what it answered.
    if (_evaluationMode != RigExecEvaluationMode::BakedWithParityCheck) {
        return baked;
    }
    // BakedWithParityCheck publishes the DYNAMIC generation: it is the
    // reference, so a disagreement must not also change what consumers see.
    RigExecRigPose reference = _EvaluateDynamic(time, std::move(settled));
    RigExecComparePoses(reference, baked, &reference);
    if (reference.bakedParityMismatches) {
        // The mode exists to be believed or disbelieved, and a count that
        // only a caller who thought to read it can see is neither. One line
        // on stderr is also what lets an existing suite be re-run under the
        // mode and FAIL on a disagreement it never looks for itself.
        TF_WARN("rigExec: %zu baked parity mismatch(es) on %s at %s",
                reference.bakedParityMismatches, _rigPath.GetText(),
                time.IsDefault()
                    ? "default"
                    : TfStringPrintf("%g", time.GetValue()).c_str());
    }
    return reference;
}

void
RigExecRigEvaluator::_ReportBakeRequired(const std::string &detail,
                                         RigExecRigPose *pose) const
{
    if (!_BakeRequired() ||
        !RigExecEvaluationModeWantsProgram(_evaluationMode)) {
        return;
    }
    // The same prefix a real disagreement carries, and deliberately so: to a
    // suite asking "did the program answer this generation", falling back
    // and answering differently are the same failure, and one regex should
    // catch both. Nothing published moves -- the pose is the dynamic path's,
    // which is the reference the mode compares against anyway.
    const std::string message =
        "baked parity mismatch: bake required, evaluated dynamically: " +
        detail;
    pose->diagnostics.push_back(message);
    ++pose->bakedParityMismatches;
    // Same one line on stderr a real disagreement gets, for the same
    // reason: a suite that never reads pose.diagnostics is exactly the
    // suite this variable exists to re-run, and it can only fail on what
    // it prints.
    TF_WARN("rigExec: %s on %s", message.c_str(), _rigPath.GetText());
}

void
RigExecRigEvaluator::_ReportAttributeBakeFallback(const std::string &detail,
                                                  RigExecRigPose *pose) const
{
    if (_evaluationModeSource != RigExecEvaluationModeSource::Attribute ||
        !RigExecEvaluationModeWantsProgram(_evaluationMode)) {
        return;
    }
    // Deliberately NOT the prefix above, and deliberately uncounted. That
    // prefix is a test harness's failure signal and bakedParityMismatches is
    // what it counts; an attribute somebody authored on the asset is a
    // REQUEST, and a rig that asks for the program and is answered by the
    // dynamic path has been answered CORRECTLY, only slowly. Turning that
    // into a suite failure would make authoring the attribute the dangerous
    // choice, which is the opposite of what it is for.
    // No TF_WARN either: the fallback is a property of the epoch, so the
    // line would repeat on every frame of a session for as long as the
    // epoch stands. It goes on the pose, where a consumer reads it once per
    // generation and a tool prints it beside the rig's other diagnostics.
    pose->diagnostics.push_back(
        "rigExec:baked is set on " + _rigPath.GetString() +
        " but this generation was evaluated dynamically: " + detail);
}

bool
RigExecRigEvaluator::_FallbackIsWorthAnnouncing() const
{
    // A mode nobody asked to be baked cannot fall back to anything: the
    // dynamic path is the answer, not a substitute for one.
    if (!RigExecEvaluationModeWantsProgram(_evaluationMode)) {
        return false;
    }
    return _BakeRequired() ||
        _evaluationModeSource == RigExecEvaluationModeSource::Attribute;
}

bool
RigExecRigEvaluator::_WantsBakeRefusalReasons() const
{
    return _BakeRequired() ||
        _evaluationModeSource == RigExecEvaluationModeSource::Attribute;
}

RigExecEvaluationMode
RigExecRigEvaluator::_PeekEvaluationMode() const
{
    // The same three-way precedence _RefreshAttributeEvaluationMode applies,
    // read-only. Compile has to decide whether to defer the dynamic-only
    // preparations in its PrepareRequests phase, and the refresh runs at the
    // TAIL of Compile on purpose -- rigExec:baked is composed, so a
    // reference swap can change it with nothing else moving, and the rebuild
    // wants the latest answer. Asking here rather than moving that call
    // keeps the documented ordering.
    // If the two ever disagreed -- composition changing mid-compile -- the
    // cost is a dynamic session whose first frame prepares its own requests.
    // Slower once, never wrong.
    if (_evaluationModeSource == RigExecEvaluationModeSource::Explicit ||
        _evaluationModeSource == RigExecEvaluationModeSource::Environment) {
        return _evaluationMode;
    }
    if (_stage) {
        if (const UsdPrim rig = _stage->GetPrimAtPath(_rigPath)) {
            const UsdAttribute attribute =
                rig.GetAttribute(_BakedAttributeName());
            bool baked = false;
            if (attribute && attribute.HasAuthoredValue() &&
                attribute.Get(&baked) && baked) {
                return RigExecEvaluationMode::Baked;
            }
        }
    }
    return RigExecEvaluationMode::Dynamic;
}

bool
RigExecRigEvaluator::_RefreshAttributeEvaluationMode()
{
    // The attribute is the weakest of the three requests, so this is a
    // no-op the moment a stronger one has been made: SetEvaluationMode is a
    // caller that chose knowing more than the asset does, and
    // RIGEXEC_EVALUATION_MODE is a whole session's answer that the parity
    // suites depend on being able to force onto any stage they open.
    // The failed-compile memo goes first, whatever the answer: this is the
    // mode being asked again, and a failure remembered under the last
    // answer is not evidence about the next compile.
    _failedCompile = _FailedCompileMemo();
    if (_evaluationModeSource == RigExecEvaluationModeSource::Explicit ||
        _evaluationModeSource == RigExecEvaluationModeSource::Environment) {
        return false;
    }
    bool authored = false;
    bool baked = false;
    if (_stage) {
        if (const UsdPrim rig = _stage->GetPrimAtPath(_rigPath)) {
            const UsdAttribute attribute =
                rig.GetAttribute(_BakedAttributeName());
            // AUTHORED, not merely readable: the schema answers false on
            // every RigExecRoot ever written, so a value alone cannot say
            // whether anybody asked. An authored false is still somebody
            // asking -- it is how an asset says "not this one" over a
            // reference that says otherwise -- so it keeps the source and
            // only the MODE goes back to Dynamic.
            if (attribute && attribute.HasAuthoredValue()) {
                authored = attribute.Get(&baked);
            }
        }
    }
    const bool ranProgram = _ModeRunsProgram();
    _evaluationModeSource = authored
        ? RigExecEvaluationModeSource::Attribute
        : RigExecEvaluationModeSource::Default;
    const RigExecEvaluationMode mode = authored && baked
        ? RigExecEvaluationMode::Baked
        : RigExecEvaluationMode::Dynamic;
    if (mode == _evaluationMode && ranProgram == _ModeRunsProgram(mode)) {
        return false;
    }
    _evaluationMode = mode;
    // Same reasoning SetEvaluationMode states: the question is being asked
    // again, and a refusal remembered from the last answer says nothing
    // about this one.
    _bakeRefused = false;
    _bakeRefusalReasons.clear();
    _bakeBail = _BakeBailMemo();
    return true;
}

bool
RigExecRigEvaluator::_NoticeNamesTheBakedAttribute(
    const UsdNotice::ObjectsChanged &notice) const
{
    const SdfPath baked = _rigPath.AppendProperty(_BakedAttributeName());
    for (const SdfPath &path : notice.GetChangedInfoOnlyPaths()) {
        if (path == baked) {
            return true;
        }
    }
    // A resync names a prim and everything under it went with it, which is
    // how the attribute arrives on a reference arc or leaves with a muted
    // layer -- neither of which reports a changed-info path for it.
    for (const SdfPath &path : notice.GetResyncedPaths()) {
        if (baked.HasPrefix(path)) {
            return true;
        }
    }
    return false;
}

void
RigExecRigEvaluator::SetEvaluationMode(RigExecEvaluationMode mode)
{
    if (_scopedClearShadow) {
        _scopedClearShadow->SetEvaluationMode(mode);
    }
    // Before the early return, because what this call settles is WHO
    // decides and not only what was decided: a tool that asks for the mode
    // it is already in has still taken the decision away from the rig's
    // rigExec:baked, and a later notice on that attribute must not take it
    // back.
    const bool ranProgram = _ModeRunsProgram();
    _evaluationModeSource = RigExecEvaluationModeSource::Explicit;
    // A stage left broken is compiled again on the next frame rather than
    // answered from the failure memo: the mode decides which preparations a
    // compile runs and so which of them can fail (deferExecPrep in Compile),
    // and this call moves the mode without a notice to say so.
    _failedCompile = _FailedCompileMemo();
    if (mode == _evaluationMode && ranProgram == _ModeRunsProgram(mode)) {
        return;
    }
    _evaluationMode = mode;
    _bakedProgramStale = false;
    // An explicit request is a new question even where the last one was
    // refused, so it does not inherit the epoch's refusal or its bail.
    _bakeRefused = false;
    _bakeRefusalReasons.clear();
    _bakeBail = _BakeBailMemo();
    if (_compiled && !_structureDirty) {
        // Baked <-> parity keeps the geometry state: the program is rebuilt
        // because the DISPATCH changed, and nothing about the rig did.
        _RebuildBakedProgram(std::move(_bakedProgram));
    } else {
        // An epoch that has not settled: this evaluator has no program until
        // Evaluate builds one, if the new mode runs one at all. (A clean
        // epoch went through the rebuild above, which drops the program for
        // a mode that does not run one.)
        _bakedProgram.reset();
        _bakedProgramPublished = false;
        _lastGenerationRanProgram = false;
    }
    // A dirty epoch is not a refusal: Evaluate builds the program once the
    // epoch has settled, which is where it can know what it would be baking.
}

void
RigExecRigEvaluator::_RebuildBakedProgram(
    std::unique_ptr<RigExecBakedProgram> outgoing)
{
    // Read before anything can replace the program it describes, and cleared
    // here because from this line on no program of this evaluator has
    // published anything.
    const bool outgoingPublished = _bakedProgramPublished;
    _bakedProgramPublished = false;
    // The replacement has answered no generation, so the op trace and op
    // graph accessors describe none until it runs one.
    _lastGenerationRanProgram = false;
    if (!_ModeRunsProgram()) {
        _bakedProgram.reset();
        _RetireBakedProgram(std::move(outgoing));
        return;
    }
    RIGEXEC_PROFILE_SCOPE_CAT(_profiler, "Compile.Bake", "compile");
    ++_bakedProgramBuildAttempts;
    // A refusal is only reportable if somebody kept it: Build is the one
    // thing that knows why, and it hands the reasons back only when it is
    // given a vector. Production passes nullptr, which saves no walk -- the
    // bakeability check assembles its refusals to answer at all -- it drops
    // them, because the dispatch needs the yes/no alone. The two callers
    // that do read them are RIGEXEC_BAKE_REQUIRED and a rig that asked for
    // the program through its own attribute, which is owed the reason it
    // did not get one; see _WantsBakeRefusalReasons.
    std::vector<std::string> reasons;
    _bakedProgram = RigExecBakedProgram::Build(
        this, _WantsBakeRefusalReasons() ? &reasons : nullptr);
    if (_bakedProgram) {
        ++_bakedProgramBuilds;
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
    bool bakeable = RigExecBakedProgram::IsBakeable(*this, reasons);
    // Here and not in the program's own check, which Build asks: a standing
    // upstream value must not refuse the program, only a bake, until the
    // exporter lists such values as inputs.
    if (HasUpstreamInputs()) {
        if (reasons) {
            reasons->push_back("upstream inputs standing (the exporter does "
                               "not expose them yet)");
        }
        bakeable = false;
    }
    return bakeable;
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
