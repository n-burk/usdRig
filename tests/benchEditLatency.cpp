//
// benchEditLatency -- what one edit costs the next evaluate, tier by tier.
//
// The sibling of benchCommitLag, with the same shape: a stage edit at a held
// playhead, then the re-evaluation that answers it. Where benchCommitLag
// times the release flush around the imaging registry, this drives the
// evaluator directly, because the edit tiers are defined on it (unified-
// program spec section 3): the registry's frame cache and publication would
// add costs that belong to neither the settle nor the program, and the
// tiers' baselines were measured without them.
//
// Every scenario runs in a child process of its own (the bench re-runs
// itself with --child), on a freshly opened stage and a fresh evaluator,
// compiled in the requested mode, profiled, and warmed with three evaluates
// at the held time before anything is timed. A process is the unit because
// compile and settle costs drift upward with the number of epochs a process
// has compiled -- measured on the biped, thirty compiled-and-dropped
// evaluators took the volumeWeights rebuild from 52 to 78 ms -- and a
// scenario's number must not depend on its place in the list. A scenario
// that needs a spec to exist before its value is edited (a schema fallback
// has none in the root layer, and a property's first spec arrives as a
// resync rather than as a value change) authors it in its setup and warms
// again.
//
// One round is: clear the profile, apply the edit (timed as "edit": the
// authoring plus the synchronous notice handling), Evaluate (timed as
// "wall"), then split that evaluate by its profile scopes:
//   settle   Evaluate.Settle -- the notice's digest and any recompile;
//   compile  Compile, which nests inside settle when an edit recompiles;
//   bake     Compile.Bake, inside compile on a recompile and after settle
//            when a captured value only made the program stale;
//   run      Evaluate.Run, the program;
//   other    the rest of Evaluate@: placement, and the dynamic walk when the
//            generation did not run the program.
// Plus how many clusters and steps the program ran (sources always run),
// and in how many rounds the program answered at all. Every column is a
// median over the rounds on its own, so a row need not add up.
//
// Prints numbers and asserts nothing, so it is built but deliberately NOT
// registered with ctest, like the other benches.
//
//   benchEditLatency [examplesDir] [--mode baked|dynamic|parity]
//                    [--rounds N] [--only ID[,ID...]]
//                    [--stage FILE] [--rig PATH]
//
// --child runs the selected scenarios in this process and prints only
// their rows; it is how the bench calls itself.
//
// The stage defaults to <examplesDir>/biped/Biped_anim.usda, whose targets
// the scenarios name first; on another stage each target falls back to the
// first prim of the right kind, and a scenario with no target says so.
// Environment is read and reported, never set: RIGEXEC_ENABLE_PARALLEL_EVAL,
// RIGEXEC_BAKED_SCHEDULE, RIGEXEC_FRAME_CACHE and
// RIGEXEC_DYNAMIC_RUNS_PROGRAM (which makes --mode dynamic run the program).
//

#include "rigExec/bakedProgram.h"
#include "rigExec/bakedProgramImpl.h"
#include "rigExec/parallel.h"
#include "rigExec/profiler.h"
#include "rigExec/rigEvaluator.h"
#include "rigExec/tapSet.h"

#include "pxr/base/plug/registry.h"
#include "pxr/base/tf/pathUtils.h"
#include "pxr/base/tf/stringUtils.h"
#include "pxr/base/tf/token.h"
#include "pxr/usd/sdf/path.h"
#include "pxr/usd/sdf/types.h"
#include "pxr/usd/usd/attribute.h"
#include "pxr/usd/usd/prim.h"
#include "pxr/usd/usd/primRange.h"
#include "pxr/usd/usd/relationship.h"
#include "pxr/usd/usd/stage.h"
#include "pxr/usd/usd/timeCode.h"

#include <algorithm>
#include <chrono>
#include <cstdio>
#include <cstdlib>
#include <functional>
#include <set>
#include <string>
#include <utility>
#include <vector>

using namespace rigExec;

PXR_NAMESPACE_USING_DIRECTIVE

namespace {

const char *
GetEnv(const char *name)
{
#ifdef _WIN32
    // One shared buffer: every caller here prints the value before it asks
    // for the next one.
    static char buffer[1024];
    size_t needed = 0;
    if (getenv_s(&needed, buffer, sizeof(buffer), name) != 0 ||
        needed == 0) {
        return nullptr;
    }
    return buffer;
#else
    return getenv(name);
#endif
}

double
NowUs()
{
    return double(std::chrono::duration_cast<std::chrono::microseconds>(
                      std::chrono::steady_clock::now().time_since_epoch())
                      .count());
}

double
Median(std::vector<double> samples)
{
    if (samples.empty()) {
        return 0.0;
    }
    std::sort(samples.begin(), samples.end());
    return samples[samples.size() / 2];
}

std::string
SchemaResourceDir()
{
#ifdef RIGEXEC_SCHEMA_RESOURCE_DIR
    return TfAbsPath(RIGEXEC_SCHEMA_RESOURCE_DIR);
#else
    return std::string();
#endif
}

SdfPath
FindRig(const UsdStageRefPtr &stage)
{
    for (const UsdPrim &prim : stage->TraverseAll()) {
        if (prim.GetTypeName() == TfToken("RigExecRoot")) {
            return prim.GetPath();
        }
    }
    return SdfPath();
}

bool
IsControl(const UsdPrim &prim)
{
    // Validity first: an invalid prim throws when asked for its type, and
    // every biped prim the targets name first is invalid on another stage.
    return prim && prim.GetTypeName() == TfToken("RigExecControl");
}

bool
IsLeafControl(const UsdPrim &prim)
{
    if (!IsControl(prim)) {
        return false;
    }
    for (const UsdPrim &child : prim.GetChildren()) {
        if (IsControl(child)) {
            return false;
        }
    }
    return true;
}

// ---------------------------------------------------------------------------
// Targets. The biped's named prims come first -- they are the ones the tier
// baselines were measured on -- and each falls back to the first prim of the
// same kind, so the bench still says something on another rig.
// ---------------------------------------------------------------------------

struct Targets {
    SdfPath rig;
    /// The root control: the hips drag, the avar, guide and rest edits
    /// land on it, and the structural edit adds a sibling beside it.
    SdfPath hips;
    /// A finger tip: the smallest cone a drag has.
    SdfPath leaf;
    /// A double avar that already has time samples.
    UsdAttribute keyedAvar;
    /// An unconnected, authored float mover input.
    UsdAttribute moverInput;
    /// A prim neither under nor above the rig.
    SdfPath outside;
    /// A relationship that lists the joints a solver poses.
    UsdRelationship joints;
    /// An aggregate solver's rigExec:jointElements, and how many joints it
    /// lists: a folded value the structure digest reads.
    UsdAttribute jointElements;
    size_t jointCount = 0;
    /// An authored folded float array the digest does not read.
    UsdAttribute foldedWeights;
};

UsdAttribute
FindKeyedAvar(const UsdStageRefPtr &stage, const Targets &t)
{
    if (!t.hips.IsEmpty()) {
        const UsdAttribute rz = stage->GetPrimAtPath(t.hips).GetAttribute(
            TfToken("avars:rz"));
        if (rz && rz.GetNumTimeSamples() > 1) {
            return rz;
        }
    }
    for (const UsdPrim &prim :
         UsdPrimRange(stage->GetPrimAtPath(t.rig))) {
        for (const UsdAttribute &attr : prim.GetAttributes()) {
            if (TfStringStartsWith(attr.GetName().GetString(), "avars:") &&
                attr.GetTypeName() == SdfValueTypeNames->Double &&
                attr.GetNumTimeSamples() > 1) {
                return attr;
            }
        }
    }
    return UsdAttribute();
}

UsdAttribute
FindMoverInput(const UsdStageRefPtr &stage, const SdfPath &rig)
{
    // A float-math mover's weight first, the input the tier names; any
    // mover's unconnected authored weight otherwise.
    UsdAttribute fallback;
    for (const UsdPrim &prim : UsdPrimRange(stage->GetPrimAtPath(rig))) {
        const UsdAttribute weight =
            prim.GetAttribute(TfToken("inputs:defaultWeight"));
        if (!weight || weight.HasAuthoredConnections() ||
            !weight.HasAuthoredValue() ||
            weight.GetTypeName() != SdfValueTypeNames->Float) {
            continue;
        }
        if (prim.GetTypeName() == TfToken("RigExecFloatMathMover")) {
            return weight;
        }
        if (!fallback) {
            fallback = weight;
        }
    }
    return fallback;
}

Targets
FindTargets(const UsdStageRefPtr &stage, const SdfPath &rig)
{
    Targets t;
    t.rig = rig;
    const UsdPrim root = stage->GetPrimAtPath(rig);
    const SdfPath hips = rig.AppendPath(SdfPath("Controls/hips_ctl"));
    if (IsControl(stage->GetPrimAtPath(hips))) {
        t.hips = hips;
    }
    SdfPath anyLeaf;
    for (const UsdPrim &prim : UsdPrimRange(root)) {
        if (t.hips.IsEmpty() && IsControl(prim)) {
            t.hips = prim.GetPath();
        }
        if (IsLeafControl(prim)) {
            anyLeaf = prim.GetPath();
            if (TfStringContains(prim.GetName().GetString(), "index")) {
                t.leaf = prim.GetPath();
            }
        }
    }
    if (t.leaf.IsEmpty()) {
        t.leaf = anyLeaf;
    }
    t.keyedAvar = FindKeyedAvar(stage, t);
    t.moverInput = FindMoverInput(stage, rig);
    const SdfPath cornea("/Biped/Materials/cornea_mat");
    if (stage->GetPrimAtPath(cornea)) {
        t.outside = cornea;
    } else {
        for (const UsdPrim &prim : stage->Traverse()) {
            const SdfPath &path = prim.GetPath();
            if (!path.HasPrefix(rig) && !rig.HasPrefix(path)) {
                t.outside = path;
                break;
            }
        }
    }
    const UsdPrim fkSolver =
        stage->GetPrimAtPath(rig.AppendPath(SdfPath("Solvers/fk_index_l")));
    const UsdRelationship fk = fkSolver
        ? fkSolver.GetRelationship(TfToken("rigExec:joints"))
        : UsdRelationship();
    SdfPathVector targets;
    if (fk && fk.GetTargets(&targets) && !targets.empty()) {
        t.joints = fk;
    } else {
        for (const UsdPrim &prim : UsdPrimRange(root)) {
            const UsdRelationship rel =
                prim.GetRelationship(TfToken("rigExec:joints"));
            if (rel && rel.GetTargets(&targets) && !targets.empty()) {
                t.joints = rel;
                break;
            }
        }
    }
    // The folded values: the biped's spine spline IK first, any aggregate
    // solver (the types that declare rigExec:jointElements) otherwise.
    std::vector<UsdPrim> solvers;
    const UsdPrim spine = stage->GetPrimAtPath(
        rig.AppendPath(SdfPath("Solvers/spine_splineIk")));
    if (spine) {
        solvers.push_back(spine);
    }
    for (const UsdPrim &prim : UsdPrimRange(root)) {
        solvers.push_back(prim);
    }
    for (const UsdPrim &prim : solvers) {
        const UsdAttribute elements =
            prim.GetAttribute(TfToken("rigExec:jointElements"));
        const UsdRelationship joints =
            prim.GetRelationship(TfToken("rigExec:joints"));
        if (!t.jointElements && elements &&
            elements.GetTypeName() == SdfValueTypeNames->IntArray &&
            joints && joints.GetTargets(&targets) && targets.size() >= 2) {
            t.jointElements = elements;
            t.jointCount = targets.size();
        }
        const UsdAttribute weights =
            prim.GetAttribute(TfToken("rigExec:volumeWeights"));
        VtFloatArray values;
        if (!t.foldedWeights && weights && weights.HasAuthoredValue() &&
            weights.Get(&values) && !values.empty()) {
            t.foldedWeights = weights;
        }
    }
    return t;
}

// ---------------------------------------------------------------------------
// One measured round, and the scopes that split it.
// ---------------------------------------------------------------------------

struct Round {
    double editMs = 0, wallMs = 0;
    double settleMs = 0, compileMs = 0, bakeMs = 0, runMs = 0, otherMs = 0;
    double clustersRun = 0, clusters = 0, stepsRun = 0, steps = 0;
    bool baked = false;
    std::string firstDiagnostic;
};

void
SplitByScopes(const std::vector<RigExecProfileEvent> &events, Round *r)
{
    // Containment is by time, not by thread: a recompile's bake may run on a
    // worker while the settle that asked for it waits.
    std::vector<std::pair<uint64_t, uint64_t>> settles;
    double evaluateUs = 0, settleUs = 0, compileUs = 0, runUs = 0;
    for (const RigExecProfileEvent &e : events) {
        if (e.kind != RigExecProfileEventKind::Complete) {
            continue;
        }
        if (e.name == "Evaluate.Settle") {
            settles.emplace_back(e.startUs, e.startUs + e.durationUs);
            settleUs += double(e.durationUs);
        } else if (e.name == "Compile") {
            compileUs += double(e.durationUs);
        } else if (e.name == "Evaluate.Run") {
            runUs += double(e.durationUs);
        } else if (TfStringStartsWith(e.name, "Evaluate@")) {
            evaluateUs += double(e.durationUs);
        }
    }
    double bakeUs = 0, bakeOutsideSettleUs = 0;
    for (const RigExecProfileEvent &e : events) {
        if (e.kind != RigExecProfileEventKind::Complete ||
            e.name != "Compile.Bake") {
            continue;
        }
        bakeUs += double(e.durationUs);
        const uint64_t end = e.startUs + e.durationUs;
        const bool inSettle = std::any_of(
            settles.begin(), settles.end(),
            [&](const std::pair<uint64_t, uint64_t> &s) {
                return e.startUs >= s.first && end <= s.second;
            });
        if (!inSettle) {
            bakeOutsideSettleUs += double(e.durationUs);
        }
    }
    r->settleMs = settleUs / 1000.0;
    r->compileMs = compileUs / 1000.0;
    r->bakeMs = bakeUs / 1000.0;
    r->runMs = runUs / 1000.0;
    r->otherMs =
        std::max(0.0, evaluateUs - settleUs - runUs - bakeOutsideSettleUs) /
        1000.0;
}

void
CountProgramWork(const RigExecRigEvaluator &evaluator, Round *r)
{
    const RigExecBakedProgram *program = evaluator.GetBakedProgram();
    if (!program || !r->baked) {
        return;
    }
    r->clusters = double(program->GetClusterCount());
    r->clustersRun = double(program->GetClustersRunLastGeneration());
    // A step ran when it is a source (sources run every generation) or when
    // it is in the step closure the run decided on.
    const RigExecBakedProgramImpl &impl = program->GetStepGraph();
    const size_t closedBits = impl.closedSteps.words.size() * 64;
    size_t steps = 0, ran = 0;
    for (const RigExecBakedStep &step : impl.steps) {
        const size_t index = steps++;
        if (step.isSource ||
            (index < closedBits && impl.closedSteps.Test(int(index)))) {
            ++ran;
        }
    }
    r->steps = double(steps);
    r->stepsRun = double(ran);
}

/// Applies the edit, evaluates at \p time, and measures both. Returns
/// false when the edit could not be authored.
bool
MeasureRound(RigExecRigEvaluator &evaluator,
             const std::function<bool()> &edit, UsdTimeCode time,
             Round *r)
{
    evaluator.ClearProfile();
    const double editStart = NowUs();
    if (!edit()) {
        return false;
    }
    const double evalStart = NowUs();
    const size_t generationsBefore = evaluator.GetBakedGenerationCount();
    const RigExecRigPose pose = evaluator.Evaluate(time);
    const double evalEnd = NowUs();
    r->editMs = (evalStart - editStart) / 1000.0;
    r->wallMs = (evalEnd - evalStart) / 1000.0;
    r->baked = evaluator.GetBakedGenerationCount() > generationsBefore;
    // The first line that is not a compile warning: a warning rides along
    // on every generation, and the line worth printing is the failure.
    for (const std::string &line : pose.diagnostics) {
        if (r->firstDiagnostic.empty() ||
            TfStringStartsWith(r->firstDiagnostic, "warning:")) {
            r->firstDiagnostic = line;
        }
    }
    SplitByScopes(evaluator.GetProfiler().GetEvents(), r);
    CountProgramWork(evaluator, r);
    return true;
}

// ---------------------------------------------------------------------------
// Reporting.
// ---------------------------------------------------------------------------

void
PrintHeader()
{
    std::printf("%-10s %3s %7s %8s %17s %8s %8s %7s %7s %7s %9s %9s %5s\n",
                "id", "n", "edit", "wall", "[min..max]", "settle",
                "compile", "bake", "run", "other", "clusters", "steps",
                "baked");
}

template <class Get>
double
MedianOf(const std::vector<Round> &rounds, Get get)
{
    std::vector<double> values;
    values.reserve(rounds.size());
    for (const Round &r : rounds) {
        values.push_back(get(r));
    }
    return Median(std::move(values));
}

void
PrintRow(const std::string &id, const std::vector<Round> &rounds)
{
    if (rounds.empty()) {
        return;
    }
    double lo = rounds.front().wallMs, hi = lo;
    size_t baked = 0;
    for (const Round &r : rounds) {
        lo = std::min(lo, r.wallMs);
        hi = std::max(hi, r.wallMs);
        baked += r.baked ? 1 : 0;
    }
    const double clusters = MedianOf(rounds, [](const Round &r) {
        return r.clusters;
    });
    const double steps = MedianOf(rounds, [](const Round &r) {
        return r.steps;
    });
    const std::string clusterText =
        clusters > 0
            ? TfStringPrintf("%.0f/%.0f",
                             MedianOf(rounds, [](const Round &r) {
                                 return r.clustersRun;
                             }),
                             clusters)
            : std::string("-");
    const std::string stepText =
        steps > 0 ? TfStringPrintf("%.0f/%.0f",
                                   MedianOf(rounds, [](const Round &r) {
                                       return r.stepsRun;
                                   }),
                                   steps)
                  : std::string("-");
    const std::string range = TfStringPrintf("[%.2f..%.2f]", lo, hi);
    std::printf(
        "%-10s %3zu %7.2f %8.2f %17s %8.2f %8.2f %7.2f %7.2f %7.2f %9s %9s "
        "%2zu/%-2zu\n",
        id.c_str(), rounds.size(),
        MedianOf(rounds, [](const Round &r) { return r.editMs; }),
        MedianOf(rounds, [](const Round &r) { return r.wallMs; }),
        range.c_str(),
        MedianOf(rounds, [](const Round &r) { return r.settleMs; }),
        MedianOf(rounds, [](const Round &r) { return r.compileMs; }),
        MedianOf(rounds, [](const Round &r) { return r.bakeMs; }),
        MedianOf(rounds, [](const Round &r) { return r.runMs; }),
        MedianOf(rounds, [](const Round &r) { return r.otherMs; }),
        clusterText.c_str(), stepText.c_str(), baked, rounds.size());
}

// ---------------------------------------------------------------------------
// Scenarios.
// ---------------------------------------------------------------------------

struct Scenario {
    std::string id;
    std::string what;
    /// Empty when the stage has what the scenario edits; otherwise why not.
    std::string missing;
    /// Authors whatever must exist before the timed edits; may be null.
    std::function<bool(RigExecRigEvaluator &)> setup;
    /// The edit of one round; null for a round that only evaluates.
    std::function<bool(RigExecRigEvaluator &, size_t)> edit;
    /// The time of one round's evaluate; the held time when null.
    std::function<double(size_t)> time;
    /// Labels for the even and odd rounds, when they edit differently.
    std::string evenLabel, oddLabel;
    /// Rounds are rounded up to an even count (alternating edits).
    bool evenRounds = false;
    /// Measured once after the rounds, reported as its own row; may be null.
    std::function<bool(RigExecRigEvaluator &, Round *)> after;
    std::string afterId;
};

bool
SetOverride(RigExecRigEvaluator &evaluator, const SdfPath &prim,
            const char *attr, double value)
{
    evaluator.SetInteractiveOverrides({RigExecValueOverride{
        prim, TfToken(), TfToken(attr), VtValue(value)}});
    return true;
}

/// The attribute's value at the default time, or \p fallback.
template <class T>
T
DefaultValue(const UsdAttribute &attr, T fallback)
{
    T value = fallback;
    attr.Get(&value, UsdTimeCode::Default());
    return value;
}

std::vector<Scenario>
BuildScenarios(const UsdStageRefPtr &stage, const Targets &t, double held)
{
    std::vector<Scenario> out;
    const UsdPrim hips = stage->GetPrimAtPath(t.hips);
    auto hipsAttr = [stage, t](const char *name) {
        return stage->GetPrimAtPath(t.hips).GetAttribute(TfToken(name));
    };
    const std::string noHips = hips ? "" : "no RigExecControl under the rig";

    {
        Scenario s;
        s.id = "ref-steady";
        s.what = "no edit, held time (reference)";
        out.push_back(std::move(s));
    }
    {
        Scenario s;
        s.id = "ref-time";
        s.what = "no edit, a new time each round (reference)";
        s.time = [held](size_t i) { return held + 10.0 + double(i); };
        out.push_back(std::move(s));
    }
    {
        Scenario s;
        s.id = "T0-leaf";
        s.what = "override drag " + t.leaf.GetString() + ".avars:rx";
        s.missing = t.leaf.IsEmpty() ? "no leaf RigExecControl" : "";
        const SdfPath leaf = t.leaf;
        s.edit = [leaf](RigExecRigEvaluator &e, size_t i) {
            return SetOverride(e, leaf, "avars:rx", 5.0 + double(i));
        };
        out.push_back(std::move(s));
    }
    {
        Scenario s;
        s.id = "T0-hips";
        s.what = "override drag " + t.hips.GetString() + ".avars:rx";
        s.missing = noHips;
        const SdfPath path = t.hips;
        s.edit = [path](RigExecRigEvaluator &e, size_t i) {
            return SetOverride(e, path, "avars:rx", 5.0 + double(i));
        };
        out.push_back(std::move(s));
    }
    {
        Scenario s;
        s.id = "T1a";
        s.what = "avar default " + t.hips.GetString() + ".avars:rx";
        s.missing = noHips;
        if (hips) {
            const UsdAttribute a = hipsAttr("avars:rx");
            const double v0 = DefaultValue(a, 0.0);
            s.setup = [a, v0](RigExecRigEvaluator &) { return a.Set(v0); };
            s.edit = [a](RigExecRigEvaluator &, size_t i) {
                return a.Set(3.0 + double(i));
            };
        }
        out.push_back(std::move(s));
    }
    {
        Scenario s;
        s.id = "T1b";
        const UsdAttribute a = t.keyedAvar;
        s.what = "avar timeSample at the held time " +
                 (a ? a.GetPath().GetString() : std::string());
        s.missing = a ? "" : "no keyed double avar";
        s.edit = [a, held](RigExecRigEvaluator &, size_t i) {
            return a.Set(7.0 + double(i), UsdTimeCode(held));
        };
        out.push_back(std::move(s));
    }
    {
        Scenario s;
        s.id = "T1c-guide";
        s.what = "non-avar value in the rig " + t.hips.GetString() +
                 ".guide:scaleX";
        s.missing = noHips;
        if (hips) {
            const UsdAttribute a = hipsAttr("guide:scaleX");
            const double v0 = DefaultValue(a, 1.0);
            s.setup = [a, v0](RigExecRigEvaluator &) { return a.Set(v0); };
            s.edit = [a, v0](RigExecRigEvaluator &, size_t i) {
                return a.Set(v0 + 0.01 * double(i + 1));
            };
        }
        out.push_back(std::move(s));
    }
    {
        Scenario s;
        s.id = "T1c-mover";
        const UsdAttribute a = t.moverInput;
        s.what = "non-avar value in the rig " +
                 (a ? a.GetPath().GetString() : std::string());
        s.missing = a ? "" : "no unconnected authored float mover weight";
        if (a) {
            // A weight lives in [0, 1], so the rounds step away from the
            // nearer bound; out of range, the mover passes through instead.
            const float v0 = DefaultValue(a, 1.0f);
            const float step = v0 >= 0.5f ? -0.001f : 0.001f;
            s.edit = [a, v0, step](RigExecRigEvaluator &, size_t i) {
                return a.Set(v0 + step * float(i + 1));
            };
        }
        out.push_back(std::move(s));
    }
    {
        Scenario s;
        s.id = "T1d";
        s.what = "value outside the rig " + t.outside.GetString() + ".foo";
        s.missing = t.outside.IsEmpty() ? "no prim outside the rig" : "";
        const SdfPath outside = t.outside;
        s.setup = [stage, outside](RigExecRigEvaluator &) {
            return stage->GetPrimAtPath(outside)
                .CreateAttribute(TfToken("foo"), SdfValueTypeNames->Float)
                .Set(0.0f);
        };
        s.edit = [stage, outside](RigExecRigEvaluator &, size_t i) {
            return stage->GetPrimAtPath(outside)
                .GetAttribute(TfToken("foo"))
                .Set(float(i + 1));
        };
        out.push_back(std::move(s));
    }
    {
        Scenario s;
        s.id = "T2";
        s.what = "captured rest value " + t.hips.GetString() + ".rest:rx";
        s.missing = noHips;
        if (hips) {
            const UsdAttribute a = hipsAttr("rest:rx");
            const double v0 = DefaultValue(a, 0.0);
            s.setup = [a, v0](RigExecRigEvaluator &) { return a.Set(v0); };
            s.edit = [a, v0](RigExecRigEvaluator &, size_t i) {
                return a.Set(v0 + 0.01 * double(i + 1));
            };
        }
        out.push_back(std::move(s));
    }
    {
        // A folded value moves only through a Build. parent:space is the
        // other one the tier names, but authoring it replaces the ladder,
        // which the program refuses ("authored parent:space on provider"),
        // so there is no baked edit of it to time. Setup authors the list
        // position explicitly -- what an empty list already means -- and the
        // rounds swap the last two elements in and out.
        Scenario s;
        s.id = "T3";
        const UsdAttribute a = t.jointElements;
        s.what = "folded value (digest-read) " +
                 (a ? a.GetPath().GetString() : std::string()) +
                 ", last two elements swapped in and out";
        s.missing = a ? "" : "no aggregate solver with two joints";
        VtIntArray identity(t.jointCount);
        for (size_t i = 0; i < identity.size(); ++i) {
            identity[i] = int(i);
        }
        VtIntArray swapped = identity;
        if (swapped.size() >= 2) {
            std::swap(swapped[swapped.size() - 1],
                      swapped[swapped.size() - 2]);
        }
        s.setup = [a, identity](RigExecRigEvaluator &) {
            return a.Set(identity);
        };
        s.edit = [a, identity, swapped](RigExecRigEvaluator &, size_t i) {
            return a.Set(i % 2 == 0 ? swapped : identity);
        };
        out.push_back(std::move(s));
    }
    {
        Scenario s;
        s.id = "T3-weights";
        const UsdAttribute a = t.foldedWeights;
        s.what = "folded value (not digest-read) " +
                 (a ? a.GetPath().GetString() : std::string()) + "[0]";
        s.missing = a ? "" : "no authored rigExec:volumeWeights";
        VtFloatArray v0;
        if (a) {
            a.Get(&v0);
        }
        s.edit = [a, v0](RigExecRigEvaluator &, size_t i) {
            VtFloatArray v = v0;
            v[0] += 0.001f * float(i + 1);
            return a.Set(v);
        };
        out.push_back(std::move(s));
    }
    {
        Scenario s;
        s.id = "T4";
        const SdfPath added = t.hips.IsEmpty()
                                  ? SdfPath()
                                  : t.hips.GetParentPath().AppendChild(
                                        TfToken("zz_new_ctl"));
        s.what = "structural: define / remove RigExecControl " +
                 added.GetString();
        s.missing = noHips;
        s.evenLabel = "T4-add";
        s.oddLabel = "T4-remove";
        s.evenRounds = true;
        s.edit = [stage, added](RigExecRigEvaluator &, size_t i) {
            if (i % 2 == 0) {
                return bool(stage->DefinePrim(added,
                                              TfToken("RigExecControl")));
            }
            return stage->RemovePrim(added);
        };
        out.push_back(std::move(s));
    }
    {
        // The stage stays broken for every round: round 0 authors the bad
        // target, and each later round is one more frame of a scrub over
        // the broken stage. The repair is its own row.
        Scenario s;
        s.id = "T5";
        const UsdRelationship rel = t.joints;
        s.what = "broken: " +
                 (rel ? rel.GetPath().GetString() : std::string()) +
                 " += a missing target, one frame per round";
        s.missing = rel ? "" : "no rigExec:joints relationship";
        SdfPathVector original;
        if (rel) {
            rel.GetTargets(&original);
        }
        const SdfPath missing = t.rig.GetParentPath().IsEmpty()
                                    ? SdfPath("/BenchEditLatencyNope")
                                    : t.rig.GetParentPath().AppendChild(
                                          TfToken("BenchEditLatencyNope"));
        s.edit = [rel, original, missing](RigExecRigEvaluator &, size_t i) {
            if (i != 0) {
                return true;
            }
            SdfPathVector broken = original;
            broken.push_back(missing);
            return rel.SetTargets(broken);
        };
        s.time = [held](size_t i) { return held + double(i); };
        s.afterId = "T5-repair";
        s.after = [rel, original, held](RigExecRigEvaluator &e, Round *r) {
            return MeasureRound(
                e, [&]() { return rel.SetTargets(original); },
                UsdTimeCode(held), r);
        };
        out.push_back(std::move(s));
    }
    return out;
}

const char *
ModeName(RigExecEvaluationMode mode)
{
    switch (mode) {
    case RigExecEvaluationMode::Baked: return "baked";
    case RigExecEvaluationMode::BakedWithParityCheck: return "parity";
    case RigExecEvaluationMode::Dynamic: return "dynamic";
    }
    return "?";
}

bool
ParseMode(const std::string &name, RigExecEvaluationMode *mode)
{
    if (name == "baked") {
        *mode = RigExecEvaluationMode::Baked;
    } else if (name == "dynamic") {
        *mode = RigExecEvaluationMode::Dynamic;
    } else if (name == "parity") {
        *mode = RigExecEvaluationMode::BakedWithParityCheck;
    } else {
        return false;
    }
    return true;
}

int
Usage()
{
    std::printf("usage: benchEditLatency [examplesDir] "
                "[--mode baked|dynamic|parity] [--rounds N] "
                "[--only ID[,ID...]] [--stage FILE] [--rig PATH]\n");
    return 2;
}

/// Runs one scenario on \p stage and prints its rows. Returns false on a
/// failure that makes the rest of the run meaningless (no compile).
bool
RunScenario(const Scenario &scenario, const UsdStageRefPtr &stage,
            const SdfPath &rigPath, RigExecEvaluationMode mode,
            size_t rounds, double held)
{
    if (!scenario.missing.empty()) {
        std::printf("%-10s n/a: %s\n", scenario.id.c_str(),
                    scenario.missing.c_str());
        return true;
    }
    RigExecRigEvaluator evaluator(stage, rigPath);
    evaluator.SetEvaluationMode(mode);
    evaluator.SetProfilingEnabled(true);
    std::vector<std::string> errors;
    if (!evaluator.Compile(&errors)) {
        std::printf("FATAL: %s did not compile\n", rigPath.GetText());
        for (const std::string &error : errors) {
            std::printf("  %s\n", error.c_str());
        }
        return false;
    }
    auto warm = [&]() {
        for (int i = 0; i < 3; ++i) {
            evaluator.Evaluate(UsdTimeCode(held));
        }
    };
    warm();
    bool ok = true;
    if (scenario.setup) {
        ok = scenario.setup(evaluator);
        warm();
    }
    const size_t count = scenario.evenRounds ? rounds + (rounds % 2) : rounds;
    std::vector<Round> all, even, odd;
    std::string diagnostic;
    for (size_t i = 0; ok && i < count; ++i) {
        Round r;
        const double time = scenario.time ? scenario.time(i) : held;
        const std::function<bool()> edit =
            scenario.edit
                ? std::function<bool()>(
                      [&]() { return scenario.edit(evaluator, i); })
                : std::function<bool()>([]() { return true; });
        ok = MeasureRound(evaluator, edit, UsdTimeCode(time), &r);
        if (!ok) {
            break;
        }
        // Why the program did not answer, when it was asked to and did not;
        // a round it answered carries only its routine report lines.
        if (diagnostic.empty() && !r.baked &&
            mode != RigExecEvaluationMode::Dynamic) {
            diagnostic = r.firstDiagnostic;
        }
        all.push_back(r);
        (i % 2 == 0 ? even : odd).push_back(r);
    }
    if (!ok) {
        std::printf("%-10s FAILED: could not author the edit\n",
                    scenario.id.c_str());
    } else {
        PrintRow(scenario.id, all);
        if (!scenario.evenLabel.empty()) {
            PrintRow(scenario.evenLabel, even);
            PrintRow(scenario.oddLabel, odd);
        }
        if (scenario.after) {
            Round r;
            if (scenario.after(evaluator, &r)) {
                PrintRow(scenario.afterId, {r});
            } else {
                std::printf("%-10s FAILED: could not author the edit\n",
                            scenario.afterId.c_str());
            }
        }
    }
    std::printf("%-10s   %s\n", "", scenario.what.c_str());
    if (!diagnostic.empty()) {
        std::printf("%-10s   first diagnostic: %s\n", "", diagnostic.c_str());
    }
    return true;
}

std::string
Quoted(const std::string &text)
{
    return "\"" + text + "\"";
}

/// Runs \p commandLine and copies its standard output to ours. Returns the
/// child's exit status, or -1 when it could not be started.
int
RunChild(const std::string &commandLine)
{
    std::fflush(stdout);
#ifdef _WIN32
    // _popen hands the line to cmd /c, which strips one pair of outer
    // quotes before it parses; without the extra pair, a quoted program
    // path followed by quoted arguments loses its own quotes.
    FILE *pipe = _popen(Quoted(commandLine).c_str(), "r");
#else
    FILE *pipe = popen(commandLine.c_str(), "r");
#endif
    if (!pipe) {
        return -1;
    }
    char line[4096];
    while (std::fgets(line, sizeof(line), pipe)) {
        std::fputs(line, stdout);
    }
    std::fflush(stdout);
#ifdef _WIN32
    return _pclose(pipe);
#else
    return pclose(pipe);
#endif
}

}  // namespace

int
main(int argc, char **argv)
{
    std::string examplesDir = "examples";
    std::string stagePath;
    SdfPath rigPath;
    RigExecEvaluationMode mode = RigExecEvaluationMode::Baked;
    size_t rounds = 8;
    std::set<std::string> only;
    bool child = false;
    bool sawExamples = false;
    // What a child needs to see the same stage the same way: everything but
    // the scenario selection.
    std::string forwarded;
    for (int i = 1; i < argc; ++i) {
        const std::string arg = argv[i];
        const bool hasValue = i + 1 < argc;
        if (arg == "--mode" && hasValue) {
            if (!ParseMode(argv[++i], &mode)) {
                return Usage();
            }
            forwarded += " --mode " + std::string(argv[i]);
        } else if (arg == "--rounds" && hasValue) {
            rounds = size_t(std::max(1, std::atoi(argv[++i])));
            forwarded += " --rounds " + std::to_string(rounds);
        } else if (arg == "--only" && hasValue) {
            for (const std::string &id : TfStringSplit(argv[++i], ",")) {
                only.insert(id);
            }
        } else if (arg == "--stage" && hasValue) {
            stagePath = argv[++i];
            forwarded += " --stage " + Quoted(stagePath);
        } else if (arg == "--rig" && hasValue) {
            rigPath = SdfPath(argv[++i]);
            forwarded += " --rig " + Quoted(rigPath.GetString());
        } else if (arg == "--child") {
            child = true;
        } else if (!TfStringStartsWith(arg, "--") && !sawExamples) {
            examplesDir = arg;
            sawExamples = true;
            forwarded += " " + Quoted(examplesDir);
        } else {
            return Usage();
        }
    }
    if (stagePath.empty()) {
        stagePath = examplesDir + "/biped/Biped_anim.usda";
    }

    const std::string resources = SchemaResourceDir();
    if (!resources.empty() &&
        PlugRegistry::GetInstance().RegisterPlugins(resources).empty()) {
        std::printf("FATAL: no schema plugin found at %s\n",
                    resources.c_str());
        return 2;
    }
    if (!child) {
        for (const char *name : {"RIGEXEC_ENABLE_PARALLEL_EVAL",
                                 "RIGEXEC_BAKED_SCHEDULE",
                                 "RIGEXEC_FRAME_CACHE",
                                 "RIGEXEC_DYNAMIC_RUNS_PROGRAM"}) {
            const char *value = GetEnv(name);
            std::printf("%s=%s\n", name, value ? value : "(unset)");
        }
        std::printf("parallel eval: %s\n",
                    RigExecParallelEvaluationEnabled() ? "on"
                                                       : "off (serial)");
    }

    const UsdStageRefPtr stage = UsdStage::Open(stagePath);
    if (!stage) {
        std::printf("FATAL: cannot open %s\n", stagePath.c_str());
        return 1;
    }
    if (rigPath.IsEmpty()) {
        rigPath = FindRig(stage);
    }
    if (rigPath.IsEmpty() || !stage->GetPrimAtPath(rigPath)) {
        std::printf("FATAL: no RigExecRoot on %s\n", stagePath.c_str());
        return 1;
    }
    // The held playhead of the tier baselines: frame 10, inside the biped's
    // keyed range, so an avar with samples is really read there.
    const double held = 10.0;
    const Targets targets = FindTargets(stage, rigPath);
    const std::vector<Scenario> scenarios =
        BuildScenarios(stage, targets, held);

    if (child) {
        for (const Scenario &scenario : scenarios) {
            if ((only.empty() || only.count(scenario.id)) &&
                !RunScenario(scenario, stage, rigPath, mode, rounds, held)) {
                return 1;
            }
        }
        return 0;
    }

    std::printf("[stage] %s rig=%s mode=%s held=%g rounds=%zu\n",
                stagePath.c_str(), rigPath.GetText(), ModeName(mode), held,
                rounds);
    std::printf("columns: medians over the rounds, ms; settle nests "
                "compile, and compile nests bake on a recompile\n");
    PrintHeader();
    int status = 0;
    for (const std::string &id : only) {
        if (std::none_of(scenarios.begin(), scenarios.end(),
                         [&](const Scenario &s) { return s.id == id; })) {
            std::printf("%-10s n/a: no such scenario\n", id.c_str());
        }
    }
    for (const Scenario &scenario : scenarios) {
        if (!only.empty() && !only.count(scenario.id)) {
            continue;
        }
        if (!scenario.missing.empty()) {
            std::printf("%-10s n/a: %s\n", scenario.id.c_str(),
                        scenario.missing.c_str());
            continue;
        }
        const int exit = RunChild(Quoted(argv[0]) + forwarded + " --only " +
                                  scenario.id + " --child");
        if (exit != 0) {
            std::printf("%-10s FAILED: the child exited with %d\n",
                        scenario.id.c_str(), exit);
            status = 1;
        }
    }
    return status;
}
