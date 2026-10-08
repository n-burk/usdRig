// Upstream inputs through the evaluator API, with no scene index. An upstream
// value is authored-level: with one standing, the dynamic walk and the baked
// program each publish what a fresh evaluator of a stage that authors the
// value publishes, and once it is lifted, what the authored stage publishes.
// Admission keeps a value only on an unconnected attribute with a stage
// value, of its own input-slot type, that a read a bake lists as an input
// reaches; every other key is reported and ignored by both paths.
// Each value case also runs frozen jobs, which carry the
// values in their sampled vector: a job equals live under the same values,
// whichever values the snapshot it clones held, and runs only what moved.
// Registered plain and under the parity entries, where every baked
// generation is also compared with the dynamic walk and, with
// RIGEXEC_BAKED_VERIFY_CONES, the cone run with a forced run of everything.
// A .rigexec playback session of a bake made with no upstream value admits
// the same keys, reports the same drops and publishes live's outputs.
// argv[1] = path to the examples directory.
#include "rigExec/inputReplay.h"
#include "rigExec/bakedProgram.h"
#include "rigExec/bakedProgramImpl.h"
#include "rigExec/bakedTrace.h"
#include "rigExec/frameCache.h"
#include "rigExec/frozenContext.h"
#include "rigExec/rigEvaluator.h"
#include "rigExecBake/bake.h"
#include "rigExecBake/staticReport.h"
#include "rigExecBake/computedCapture.h"
#include "rigExecBake/pathTable.h"
#include "rigExecImaging/playback.h"
#include "rigExecRuntime/stageArrayInputs.h"
#include "rigExecRuntimeDrive.h"

#include "pxr/base/gf/matrix4d.h"
#include "pxr/base/gf/vec3d.h"
#include "pxr/base/gf/vec3f.h"
#include "pxr/base/plug/registry.h"
#include "pxr/base/tf/getenv.h"
#include "pxr/base/tf/pathUtils.h"
#include "pxr/base/tf/setenv.h"
#include "pxr/base/tf/stringUtils.h"
#include "pxr/base/tf/token.h"
#include "pxr/base/vt/types.h"
#include "pxr/base/vt/value.h"
#include "pxr/usd/sdf/layer.h"
#include "pxr/usd/sdf/types.h"
#include "pxr/usd/usd/attribute.h"
#include "pxr/usd/usd/editContext.h"
#include "pxr/usd/usd/prim.h"
#include "pxr/usd/usd/primRange.h"
#include "pxr/usd/usd/stage.h"

#include <algorithm>
#include <cctype>
#include <chrono>
#include <cstdio>
#include <filesystem>
#include <fstream>
#include <functional>
#include <map>
#include <set>
#include <string>
#include <utility>
#include <vector>

using namespace rigExec;

PXR_NAMESPACE_USING_DIRECTIVE

static int failures = 0;

#define CHECK(cond)                                                        \
    do {                                                                   \
        if (!(cond)) {                                                     \
            ++failures;                                                    \
            std::printf("FAIL %s:%d: %s\n", __FILE__, __LINE__, #cond);    \
        }                                                                  \
    } while (0)

namespace {

// Head entries are part of the ordinary execution trace.
static std::vector<RigExecOpTraceEntry>
ExecutedHeads(const RigExecBakedProgramImpl &B)
{
    auto trace = RigExecBakedLastRunTrace(B);
    trace.erase(std::remove_if(trace.begin(),trace.end(),
        [&B](const auto &entry) { return !B.steps[entry.step].isHead; }),trace.end());
    return trace;
}

std::string g_examples;

std::string
SchemaResourceDir()
{
#ifdef RIGEXEC_SCHEMA_RESOURCE_DIR
    return TfAbsPath(RIGEXEC_SCHEMA_RESOURCE_DIR);
#else
    return TfAbsPath(g_examples + "/../plugin/rigExecSchema/resources");
#endif
}

std::string
Example(const std::string &name)
{
    return g_examples + "/" + name;
}

std::string
Fixture(const std::string &name)
{
    return g_examples + "/../tests/fixtures/" + name;
}

bool
ConeVerify()
{
    return TfGetenvBool("RIGEXEC_BAKED_VERIFY_CONES", false);
}

RigExecValueOverride
Up(const SdfPath &attribute, const VtValue &value)
{
    return RigExecValueOverride{attribute.GetPrimPath(), TfToken(),
                                attribute.GetNameToken(), value};
}

GfMatrix4d
Translate(double x, double y, double z)
{
    GfMatrix4d m(1.0);
    m.SetTranslateOnly(GfVec3d(x, y, z));
    return m;
}

// The first RigExecRoot under the pseudo-root.
SdfPath
FindRig(const UsdStageRefPtr &stage)
{
    for (const UsdPrim &prim : stage->Traverse()) {
        if (prim.GetTypeName() == TfToken("RigExecRoot")) {
            return prim.GetPath();
        }
    }
    return SdfPath();
}

std::unique_ptr<RigExecRigEvaluator>
Make(const UsdStageRefPtr &stage, const SdfPath &rig)
{
    auto evaluator = std::make_unique<RigExecRigEvaluator>(stage, rig);
    std::vector<std::string> errors;
    const bool compiled = evaluator->Compile(&errors);
    CHECK(compiled);
    if (!compiled) {
        for (const std::string &e : errors) {
            std::printf("    compile: %s\n", e.c_str());
        }
    }
    return evaluator;
}

// A value the reference stage authors in its session layer: a default, or
// time samples.
struct Authored {
    SdfPath path;
    VtValue value;
    std::map<double, VtValue> samples;
};

UsdStageRefPtr
OpenAuthored(const std::string &stagePath, const std::vector<Authored> &edits)
{
    UsdStageRefPtr stage = UsdStage::Open(stagePath);
    CHECK(stage);
    if (!stage) {
        return stage;
    }
    UsdEditContext context(stage, stage->GetSessionLayer());
    for (const Authored &edit : edits) {
        const UsdAttribute a = stage->GetAttributeAtPath(edit.path);
        CHECK(a);
        if (!a) {
            continue;
        }
        if (!edit.samples.empty()) {
            for (const auto &[t, v] : edit.samples) {
                CHECK(a.Set(v, UsdTimeCode(t)));
            }
        } else {
            CHECK(a.Set(edit.value));
        }
    }
    return stage;
}

// What a fresh evaluator publishes at \p time on \p stagePath
// with \p edits authored.
RigExecRigPose
Reference(const std::string &stagePath, const SdfPath &rig, const std::vector<Authored> &edits,
          UsdTimeCode time)
{
    const UsdStageRefPtr stage = OpenAuthored(stagePath, edits);
    if (!stage) {
        return RigExecRigPose();
    }
    auto evaluator = Make(stage, rig);
    return evaluator->Evaluate(time);
}

// The pose minus what a generation BUILT (a fresh program builds every
// node), and minus the upstream drop lines, which the callers check apart.
void
KeepPosed(RigExecRigPose *pose)
{

    std::vector<std::string> kept;
    for (std::string &line : pose->diagnostics) {
        if (line.rfind("mover graph:", 0) != 0 &&
            line.rfind("structural edit:", 0) != 0 &&
            line.rfind("upstream input ", 0) != 0) {
            kept.push_back(std::move(line));
        }
    }
    pose->diagnostics = std::move(kept);
}

size_t
Differences(const RigExecRigPose &reference, const RigExecRigPose &pose,
            std::vector<std::string> *lines = nullptr)
{
    RigExecRigPose a = reference, b = pose;
    KeepPosed(&a);
    KeepPosed(&b);
    RigExecRigPose diff;
    RigExecComparePoses(a, b, &diff);
    if (lines) {
        *lines = diff.diagnostics;
    }
    return diff.comparisonMismatches;
}

void
CheckSamePose(const std::string &what, const RigExecRigPose &reference,
              const RigExecRigPose &pose)
{
    CHECK(pose.valid);
    CHECK(reference.valid);
    std::vector<std::string> lines;
    const size_t n = Differences(reference, pose, &lines);
    if (n != 0) {
        std::printf("FAIL %s: %zu mismatch(es) against the reference:\n",
                    what.c_str(), n);
        for (size_t i = 0; i < lines.size() && i < 12; ++i) {
            std::printf("    %s\n", lines[i].c_str());
        }
    }
    CHECK(n == 0);
}

bool
HasLine(const RigExecRigPose &pose, const std::string &prefix)
{
    for (const std::string &line : pose.diagnostics) {
        if (line.rfind(prefix, 0) == 0) {
            return true;
        }
    }
    return false;
}

bool
HasReason(const std::vector<std::string> &reasons, const std::string &text)
{
    for (const std::string &reason : reasons) {
        if (reason.find(text) != std::string::npos) {
            return true;
        }
    }
    return false;
}

std::vector<SdfPath>
PathsOf(const std::vector<RigExecValueOverride> &inputs)
{
    std::set<SdfPath> paths;
    for (const RigExecValueOverride &o : inputs) {
        paths.insert(o.prim.AppendProperty(o.attribute));
    }
    return std::vector<SdfPath>(paths.begin(), paths.end());
}

// --- Frozen jobs ------------------------------------------------------------

std::shared_ptr<const RigExecFrozenProgram>
Freeze(const RigExecRigEvaluator &evaluator, const std::string &what)
{
    std::shared_ptr<const RigExecFrozenProgram> frozen;
    std::string error;
    if (!RigExecFreezeProgram(evaluator, &frozen, &error)) {
        std::printf("FAIL %s: freeze refused: %s\n", what.c_str(),
                    error.c_str());
        ++failures;
    }
    return frozen;
}

struct FrozenJob {
    RigExecFrameInputs inputs;
    RigExecFrozenRunReport report;
    RigExecRigPose pose;
};

FrozenJob RunSampled(const RigExecRigEvaluator &evaluator,
                     const std::shared_ptr<const RigExecFrozenProgram> &snapshot,
                     RigExecFrameInputs inputs, const std::string &what);

// One warming job at \p time from \p snapshot, sampled from \p evaluator's
// program with \p upstream as the frame's upstream values, through the
// production runner.
FrozenJob
RunJob(const RigExecRigEvaluator &evaluator,
       const std::shared_ptr<const RigExecFrozenProgram> &snapshot,
       UsdTimeCode time, const std::vector<RigExecValueOverride> &upstream,
       const std::string &what)
{
    RigExecFrameInputs inputs;
    std::string error;
    if (!snapshot ||
        !RigExecSampleFrameInputs(evaluator, time, {},
                                  RigExecUpstreamValuesOf(upstream),
                                  &inputs, &error)) {
        std::printf("FAIL %s: no job sampled: %s\n", what.c_str(),
                    error.c_str());
        ++failures;
        return FrozenJob();
    }
    return RunSampled(evaluator, snapshot, std::move(inputs), what);
}

// One job over an already sampled vector, through the production runner.
FrozenJob
RunSampled(const RigExecRigEvaluator &evaluator,
           const std::shared_ptr<const RigExecFrozenProgram> &snapshot,
           RigExecFrameInputs inputs, const std::string &what)
{
    FrozenJob job;
    job.inputs = std::move(inputs);
    if (!snapshot) {
        std::printf("FAIL %s: no snapshot\n", what.c_str());
        ++failures;
        return job;
    }
    RigExecFrozenEvalContext context;
    context.epochDigest = evaluator.GetBindingEpochDigest();
    context.slotCount = evaluator.GetBakedProgram()->GetProviderCount();
    context.varyingInputCount = job.inputs.values.size();
    if (evaluator.GetPublishWeightFields()) {
        context.flags |= kRigExecFrozenPublishWeightFields;
    }
    if (evaluator.GetSolverGuidesEnabled()) {
        context.flags |= kRigExecFrozenSolverGuidesEnabled;
    }
    context.frozen = snapshot.get();
    job.pose = RigExecEvaluateFrozen(context, job.inputs,
                                     RigExecMakeProductionStepRunner(),
                                     nullptr, SdfPath(), &job.report);
    // Accepted: the worker ran it from samples alone.
    CHECK(!job.inputs.HasChainResolvedInputs());
    CHECK(job.pose.valid);
    CHECK(job.report.ran);
    if (!job.pose.valid) {
        std::printf("FAIL %s: the job declined\n", what.c_str());
    }
    return job;
}

// The steps of \p job outside what live ran over the same history: region
// steps (sources aside) by cluster, as the worker runs whole clusters,
// against the clusters of \p liveRegion and the always-dirty steps; head
// steps against \p liveHead.
size_t
StepsOutsideLive(const RigExecBakedProgramImpl &B, const FrozenJob &job,
                 const std::vector<RigExecOpTraceEntry> &liveRegion,
                 const std::vector<RigExecOpTraceEntry> &liveHead)
{
    std::set<int> clusters;
    for (const RigExecOpTraceEntry &entry : liveRegion) {
        clusters.insert(entry.cluster);
    }
    std::set<size_t> head;
    for (const RigExecOpTraceEntry &entry : liveHead) {
        head.insert(entry.step);
    }
    size_t outside = 0;
    for (const RigExecOpTraceEntry &entry : job.report.region) {
        if (B.steps[entry.step].isHead) continue;
        if (entry.step < B.steps.size() && B.steps[entry.step].isSource) {
            continue;
        }
        if (!clusters.count(entry.cluster) &&
            !B.cones.alwaysSteps.Test(int(entry.step))) {
            ++outside;
        }
    }
    for (const RigExecOpTraceEntry &entry : job.report.region) {
        if (!B.steps[entry.step].isHead) continue;
        if (!head.count(entry.step)) {
            ++outside;
        }
    }
    return outside;
}

// The region steps a job ran beyond its sources and the always-dirty steps.
std::set<size_t>
WorkSteps(const RigExecBakedProgramImpl &B, const FrozenJob &job)
{
    std::set<size_t> work;
    for (const RigExecOpTraceEntry &entry : job.report.region) {
        if (entry.step < B.steps.size() && !B.steps[entry.step].isHead && !B.steps[entry.step].isSource &&
            !B.cones.alwaysSteps.Test(int(entry.step))) {
            work.insert(entry.step);
        }
    }
    return work;
}

size_t
NonSourceWork(const RigExecBakedProgramImpl &B, const FrozenJob &job)
{
    return WorkSteps(B, job).size();
}

// The frozen legs of a value case, on \p evaluator with \p inputs standing
// and its last generation at \p time: \p authoredSnapshot was frozen before
// the values were placed, and the live traces are those of the generation
// that placed them.
//  - A job carrying the values from the authored snapshot equals live with
//    them standing, and runs no step outside live's cone of that change.
//  - A job carrying them from a snapshot frozen while they stand runs only
//    what the authored job runs against the authored snapshot: nothing
//    moved against its history.
//  - A job with none from that snapshot lifts them: it equals the authored
//    pose and its key is the authored job's.
void
RunFrozenLegs(const std::string &what, const RigExecRigEvaluator &evaluator,
              const std::shared_ptr<const RigExecFrozenProgram>
                  &authoredSnapshot,
              const FrozenJob &authoredJob,
              const std::vector<RigExecValueOverride> &inputs,
              UsdTimeCode time, const RigExecRigPose &authored,
              const RigExecRigPose &standing,
              const std::vector<RigExecOpTraceEntry> &liveRegion,
              const std::vector<RigExecOpTraceEntry> &liveHead)
{
    const RigExecBakedProgram *program = evaluator.GetBakedProgram();
    CHECK(program);
    if (!program || !authoredSnapshot) {
        return;
    }
    const RigExecBakedProgramImpl &B = program->GetStepGraph();
    const FrozenJob placed =
        RunJob(evaluator, authoredSnapshot, time, inputs, what + ": placed");
    CheckSamePose(what + ": frozen, placed", standing, placed.pose);
    CHECK(placed.inputs.upstream == RigExecUpstreamValuesOf(inputs));
    CHECK(!placed.report.region.empty());
    const size_t outside = StepsOutsideLive(B, placed, liveRegion, liveHead);
    if (outside != 0) {
        std::printf("FAIL %s: frozen job ran %zu step(s) outside live's "
                    "cone\n", what.c_str(), outside);
    }
    CHECK(outside == 0);
    CHECK(RigExecControlStateDigest(placed.inputs) !=
          RigExecControlStateDigest(authoredJob.inputs));

    const std::shared_ptr<const RigExecFrozenProgram> standingSnapshot =
        Freeze(evaluator, what + ": standing snapshot");
    if (!standingSnapshot) {
        return;
    }
    CHECK(!standingSnapshot->program.lastUpstream.empty());
    const FrozenJob again = RunJob(evaluator, standingSnapshot, time, inputs,
                                   what + ": standing");
    CheckSamePose(what + ": frozen, standing", standing, again.pose);
    // The work a job does with nothing moved against its history: what the
    // authored job did against the authored snapshot (its time-varying
    // reads), and no more.
    CHECK(NonSourceWork(B, again) == NonSourceWork(B, authoredJob));
    CHECK(WorkSteps(B, again) == WorkSteps(B, authoredJob));
    CHECK(std::count_if(again.report.region.begin(),again.report.region.end(),
        [&B](const auto &entry) { return B.steps[entry.step].isHead; }) ==
          std::count_if(authoredJob.report.region.begin(),authoredJob.report.region.end(),
        [&B](const auto &entry) { return B.steps[entry.step].isHead; }));

    const FrozenJob lifted =
        RunJob(evaluator, standingSnapshot, time, {}, what + ": lifted");
    CheckSamePose(what + ": frozen, lifted", authored, lifted.pose);
    CHECK(lifted.inputs.upstream.empty());
    CHECK(RigExecControlStateDigest(lifted.inputs) ==
          RigExecControlStateDigest(authoredJob.inputs));
}

// One value case: the authored pose, then the pose with
// \p inputs standing against a stage authoring \p edits, then the pose
// after the lift against the authored stage again. The
// program answers every generation, and frozen jobs follow
// (RunFrozenLegs).
void
RunCase(const std::string &name, const std::string &stagePath,
        const SdfPath &rig, double t,
        const std::vector<RigExecValueOverride> &inputs,
        const std::vector<Authored> &edits)
{
    std::printf("case: %s\n", name.c_str());
    const UsdTimeCode time(t);
    {
        const std::string what = name + " (" + "graph" + ")";
        const UsdStageRefPtr stage = UsdStage::Open(stagePath);
        CHECK(stage);
        if (!stage) {
            return;
        }
        auto evaluator = Make(stage, rig);
        const RigExecRigPose authored =
            Reference(stagePath, rig, {}, time);
        const RigExecRigPose before = evaluator->Evaluate(time);
        CheckSamePose(what + ": before", authored, before);
        const bool frozen = true;
        CHECK(RigExecCanFreezeProgram(*evaluator));
        std::shared_ptr<const RigExecFrozenProgram> authoredSnapshot;
        FrozenJob authoredJob;
        if (frozen) {
            authoredSnapshot = Freeze(*evaluator, what + ": authored");
            authoredJob = RunJob(*evaluator, authoredSnapshot, time, {},
                                 what + ": authored");
            CheckSamePose(what + ": frozen, authored", before,
                          authoredJob.pose);
        }

        evaluator->SetUpstreamInputs(inputs);
        CHECK(evaluator->HasUpstreamInputs());
        CHECK(!evaluator->HasInteractiveOverrides());
        const RigExecRigPose standing = evaluator->Evaluate(time);
        CHECK(evaluator->GetUpstreamInputPaths() == PathsOf(inputs));
        CHECK(!HasLine(standing, "upstream input "));
        const RigExecRigPose expected =
            Reference(stagePath, rig, edits, time);
        CheckSamePose(what + ": standing", expected, standing);
        // The value moved something, so the case tests a move.
        CHECK(Differences(authored, standing) != 0);
        if (frozen && evaluator->GetBakedProgram()) {
            RunFrozenLegs(what, *evaluator, authoredSnapshot, authoredJob,
                          inputs, time, authored, standing,
                          evaluator->GetLastOpTrace(),
                          ExecutedHeads(
                              evaluator->GetBakedProgram()->GetStepGraph()));
        }
        // A second generation with the same value standing.
        CheckSamePose(what + ": standing again", expected,
                      evaluator->Evaluate(time));

        evaluator->SetUpstreamInputs({});
        CHECK(!evaluator->HasUpstreamInputs());
        CHECK(evaluator->GetUpstreamInputPaths().empty());
        CheckSamePose(what + ": lifted", authored, evaluator->Evaluate(time));
    }
}

// --- Cases ----------------------------------------------------------------

const SdfPath kPropRig("/PropMathAsset/Rig");
const SdfPath kLimbsRig("/LimbsAsset/Rig");
const SdfPath kA0Rz("/LimbsAsset/Rig/Controls/A0.avars:rz");
const SdfPath kA1Rz("/LimbsAsset/Rig/Controls/A0/A1.avars:rz");
const SdfPath kUpstreamSpace("/LimbsAsset/Upstream.inputs:space");
const SdfPath kBRootRestSpace("/LimbsAsset/Rig/Controls/BRoot.rest:space");

// A chain target: the value is the chain's base, which its mover clamps.
void
TestAChainTarget()
{
    const SdfPath gain("/PropMathAsset/Rig/Channels/Dials.rigExec:gain");
    RunCase("chain target", Example("09_PropertyMathMovers.usda"), kPropRig,
            1012, {Up(gain, VtValue(0.5f))}, {{gain, VtValue(0.5f), {}}});
}

// A chain mover's own input, which no override number covers.
void
TestAChainMoverInput()
{
    const SdfPath value("/PropMathAsset/Rig/Movers/OffsetLift.inputs:value");
    RunCase("chain mover inputs:value", Example("09_PropertyMathMovers.usda"),
            kPropRig, 1012, {Up(value, VtValue(GfVec3f(0, 7, 0)))},
            {{value, VtValue(GfVec3f(0, 7, 0)), {}}});
}

// A rest channel of the epoch-rest rig: the dynamic walk re-pulls its
// rests while the value stands, the program recomposes the ladder.
void
TestARestChannel()
{
    const SdfPath tx("/TailAsset/Rig/Joints/Seg1/Seg2.rest:tx");
    RunCase("rest:tx", Example("01_FkChainTail.usda"),
            SdfPath("/TailAsset/Rig"), 1012, {Up(tx, VtValue(0.5))},
            {{tx, VtValue(0.5), {}}});
}

// A skin mover's envelope, a revision leaf.
void
TestASkinMoverDefaultWeight()
{
    const SdfPath weight("/Biped/Rig/Movers/skin_body_geo/body_geo_skin."
                         "inputs:defaultWeight");
    RunCase("skin mover inputs:defaultWeight", Example("biped/Biped.usda"),
            SdfPath("/Biped/Rig"), 1, {Up(weight, VtValue(0.5f))},
            {{weight, VtValue(0.5f), {}}});
}

// The source hop of a connected ladder binding the bake folds to a
// constant: the walk falls back to it, so the upstream matrix is BRoot's
// rest.
void
TestAPinnedHop()
{
    const GfMatrix4d moved = Translate(1, 0, 10);
    RunCase("Upstream.inputs:space", Fixture("upstream_inputs.usda"),
            kLimbsRig, 1, {Up(kUpstreamSpace, VtValue(moved))},
            {{kUpstreamSpace, VtValue(moved), {}}});
}

// A0's unkeyed avar: a constant avar binding.
void
TestAConstantAvar()
{
    RunCase("A0 avars:rz", Fixture("upstream_inputs.usda"), kLimbsRig, 1,
            {Up(kA0Rz, VtValue(30.0))}, {{kA0Rz, VtValue(30.0), {}}});
}

// A solver's own scalar input: the Solve step is seeded only in the run the
// value is placed or lifted (`upstreamChanged`), since nothing else it
// reads moves at a held time.
void
TestASolverInput()
{
    const SdfPath offset(
        "/LimbsAsset/Rig/Solvers/LimbBIK.rigExec:upperLengthOffset");
    RunCase("solver rigExec:upperLengthOffset", Fixture("upstream_inputs.usda"),
            kLimbsRig, 5, {Up(offset, VtValue(0.5))},
            {{offset, VtValue(0.5), {}}});
}

// An upstream value on a constant avar stands while an interactive drag on
// another avar starts and stops: the constant-avar pass writes its leaf.
void
TestUpstreamThroughADrag()
{
    std::printf("case: upstream through a drag\n");
    const std::string stagePath = Fixture("upstream_inputs.usda");
    const UsdTimeCode time(3);
    {
        const std::string what =
            std::string("upstream through a drag (") + "graph" + ")";
        const UsdStageRefPtr stage = UsdStage::Open(stagePath);
        auto evaluator = Make(stage, kLimbsRig);
        evaluator->Evaluate(time);
        evaluator->SetUpstreamInputs({Up(kA0Rz, VtValue(30.0))});
        evaluator->Evaluate(time);
        evaluator->SetInteractiveOverrides({Up(kA1Rz, VtValue(20.0))});
        CHECK(evaluator->HasInteractiveOverrides());
        CheckSamePose(what + ": dragging",
                      Reference(stagePath, kLimbsRig,
                                {{kA0Rz, VtValue(30.0), {}},
                                 {kA1Rz, VtValue(20.0), {}}},
                                time),
                      evaluator->Evaluate(time));
        evaluator->ClearInteractiveOverrides();
        const RigExecRigPose expected = Reference(
            stagePath, kLimbsRig, {{kA0Rz, VtValue(30.0), {}}}, time);
        CheckSamePose(what + ": released", expected,
                      evaluator->Evaluate(time));
        CheckSamePose(what + ": released again", expected,
                      evaluator->Evaluate(time));
        // An interactive value on the same key wins while it stands.
        evaluator->SetInteractiveOverrides({Up(kA0Rz, VtValue(-10.0))});
        CheckSamePose(what + ": dragging the same key",
                      Reference(stagePath, kLimbsRig,
                                {{kA0Rz, VtValue(-10.0), {}}}, time),
                      evaluator->Evaluate(time));
        evaluator->ClearInteractiveOverrides();
        CheckSamePose(what + ": that drag released", expected,
                      evaluator->Evaluate(time));
    }
}

// A value set per frame, as a time-varying source is pulled: each frame
// equals a stage that authors the same time samples.
void
TestATimeVaryingValue()
{
    std::printf("case: A0 avars:rz per frame\n");
    const std::string stagePath = Fixture("upstream_inputs.usda");
    Authored samples{kA0Rz, VtValue(), {}};
    for (int f = 1; f <= 5; ++f) {
        samples.samples[double(f)] = VtValue(5.0 * f);
    }
    {
        const UsdStageRefPtr stage = UsdStage::Open(stagePath);
        auto evaluator = Make(stage, kLimbsRig);
        const UsdStageRefPtr authoredStage =
            OpenAuthored(stagePath, {samples});
        auto reference = Make(authoredStage, kLimbsRig);
        std::shared_ptr<const RigExecFrozenProgram> snapshot;
        std::set<uint64_t> digests;
        for (int f = 1; f <= 5; ++f) {
            const std::string what = std::string("A0 per frame (") +
                                     "graph" + ") at " +
                                     std::to_string(f);
            const std::vector<RigExecValueOverride> inputs = {
                Up(kA0Rz, VtValue(5.0 * f))};
            evaluator->SetUpstreamInputs(inputs);
            const RigExecRigPose expected = reference->Evaluate(UsdTimeCode(f));
            const RigExecRigPose live = evaluator->Evaluate(UsdTimeCode(f));
            CheckSamePose(what, expected, live);
            // Each frame's job from the snapshot taken at frame 1: its own
            // frame's value, whatever the snapshot's.
            if (!snapshot) {
                snapshot = Freeze(*evaluator, what);
            }
            const FrozenJob job =
                RunJob(*evaluator, snapshot, UsdTimeCode(f), inputs, what);
            CheckSamePose(what + " (frozen)", expected, job.pose);
            digests.insert(RigExecControlStateDigest(job.inputs));
            // The upstream block alone separates the frame's key from the
            // same vector under another value.
            FrozenJob other = job;
            other.inputs.upstream =
                RigExecUpstreamValuesOf({Up(kA0Rz, VtValue(5.0 * f + 1.0))});
            CHECK(RigExecControlStateDigest(other.inputs) !=
                  RigExecControlStateDigest(job.inputs));
        }
        {
            CHECK(digests.size() == 5);
        }
    }
}

// A lift reaches a job whose snapshot was frozen while the value stood: the
// clone's slots, leaves and outputs hold the value, and the job, carrying
// none, diffs its empty table against the clone's. Its pose and its key are
// the authored ones at another frame. A constant avar (a slot only the
// constant-avar pass rewrites), a chain mover's constant inputs:value (a
// head leaf) and a skin mover's inputs:defaultWeight (a revision leaf).
void
TestALiftReachesAnOlderSnapshot()
{
    std::printf("case: a lift reaches an older snapshot\n");
    struct Lift {
        std::string name;
        std::string stagePath;
        SdfPath rig;
        RigExecValueOverride input;
        double placedAt, warmedAt;
    };
    const std::vector<Lift> lifts = {
        {"A0 avars:rz", Fixture("upstream_inputs.usda"), kLimbsRig,
         Up(kA0Rz, VtValue(30.0)), 1, 4},
        {"chain mover inputs:value", Example("09_PropertyMathMovers.usda"),
         kPropRig,
         Up(SdfPath("/PropMathAsset/Rig/Movers/OffsetLift.inputs:value"),
            VtValue(GfVec3f(0, 7, 0))),
         1012, 1013},
        {"skin mover inputs:defaultWeight", Example("biped/Biped.usda"),
         SdfPath("/Biped/Rig"),
         Up(SdfPath("/Biped/Rig/Movers/skin_body_geo/body_geo_skin."
                    "inputs:defaultWeight"),
            VtValue(0.5f)),
         1, 4},
    };
    for (const Lift &lift : lifts) {
        const std::string what = "lift " + lift.name;
        const UsdStageRefPtr stage = UsdStage::Open(lift.stagePath);
        auto evaluator = Make(stage, lift.rig);
        const UsdTimeCode placedAt(lift.placedAt), warmedAt(lift.warmedAt);
        evaluator->Evaluate(placedAt);
        const std::shared_ptr<const RigExecFrozenProgram> before =
            Freeze(*evaluator, what + ": before");
        // The authored key at the warmed frame.
        const FrozenJob authoredJob =
            RunJob(*evaluator, before, warmedAt, {}, what + ": authored");
        evaluator->SetUpstreamInputs({lift.input});
        evaluator->Evaluate(placedAt);
        const std::shared_ptr<const RigExecFrozenProgram> standing =
            Freeze(*evaluator, what + ": standing");
        if (!standing) {
            continue;
        }
        CHECK(!standing->program.lastUpstream.empty());
        // The value stands at the warmed frame too.
        const FrozenJob held =
            RunJob(*evaluator, standing, warmedAt, {lift.input},
                   what + ": held");
        CheckSamePose(what + ": held",
                      Reference(lift.stagePath, lift.rig,
                                {{lift.input.prim.AppendProperty(
                                      lift.input.attribute),
                                  lift.input.value,
                                  {}}},
                                warmedAt),
                      held.pose);
        // Lifted live; the snapshot taken while it stood is kept.
        evaluator->SetUpstreamInputs({});
        evaluator->Evaluate(placedAt);
        const FrozenJob job =
            RunJob(*evaluator, standing, warmedAt, {}, what + ": lifted");
        CheckSamePose(what + ": lifted",
                      Reference(lift.stagePath, lift.rig, {},
                                warmedAt),
                      job.pose);
        CheckSamePose(what + ": lifted against the authored job",
                      authoredJob.pose, job.pose);
        CHECK(RigExecControlStateDigest(job.inputs) ==
              RigExecControlStateDigest(authoredJob.inputs));
    }
}

// A warming burst prepared under upstream values samples every frame as the
// plain sampler does under them, sample for sample and digest for digest,
// and its jobs equal a stage authoring the value. A burst refuses frames
// under another list. Cases: a constant avar, the source hop of a pinned
// ladder binding, a chain mover's inputs:value (a head leaf, chains bound)
// and a skin mover's inputs:defaultWeight (a memoized mover scalar).
struct Burst {
    std::string name;
    std::string stagePath;
    SdfPath rig;
    RigExecValueOverride input;
    double first;
};

void CheckBurst(const Burst &burst);

void
TestABurstCarriesUpstream()
{
    std::printf("case: a burst carries upstream values\n");
    const SdfPath space("/LimbsAsset/Upstream.inputs:space");
    const std::vector<Burst> bursts = {
        {"A0 avars:rz", Fixture("upstream_inputs.usda"), kLimbsRig,
         Up(kA0Rz, VtValue(30.0)), 1},
        {"Upstream inputs:space", Fixture("upstream_inputs.usda"), kLimbsRig,
         Up(space, VtValue(Translate(1, 0, 10))), 1},
        {"chain mover inputs:value", Example("09_PropertyMathMovers.usda"),
         kPropRig,
         Up(SdfPath("/PropMathAsset/Rig/Movers/OffsetLift.inputs:value"),
            VtValue(GfVec3f(0, 7, 0))),
         1012},
        {"skin mover inputs:defaultWeight", Example("biped/Biped.usda"),
         SdfPath("/Biped/Rig"),
         Up(SdfPath("/Biped/Rig/Movers/skin_body_geo/body_geo_skin."
                    "inputs:defaultWeight"),
            VtValue(0.5f)),
         1},
    };
    for (const Burst &burst : bursts) {
        CheckBurst(burst);
    }
}

// One burst case (TestABurstCarriesUpstream).
void
CheckBurst(const Burst &burst)
{
    {
        const std::string what = "burst " + burst.name;
        const UsdStageRefPtr stage = UsdStage::Open(burst.stagePath);
        auto evaluator = Make(stage, burst.rig);
        const UsdTimeCode first(burst.first);
        evaluator->Evaluate(first);
        evaluator->SetUpstreamInputs({burst.input});
        evaluator->Evaluate(first);
        const RigExecBakedProgram *program = evaluator->GetBakedProgram();
        CHECK(program);
        const std::shared_ptr<const RigExecFrozenProgram> snapshot =
            Freeze(*evaluator, what);
        if (!program || !snapshot) {
            return;
        }
        RigExecChainSampleBindings pinned;
        std::string error;
        CHECK(RigExecBindChainSampleInputs(*evaluator, &pinned, &error));
        const std::vector<RigExecUpstreamValue> upstream =
            RigExecUpstreamValuesOf({burst.input});
        const std::vector<RigExecValueOverride> none;
        RigExecBurstSampleCache cache;
        CHECK(RigExecBuildBurstSampleCache(
            *program, pinned, none, upstream,
            RigExecFrameCacheEpochDigest(*evaluator), &cache, &error));
        CHECK(cache.usable);
        CHECK(cache.upstreamAdmitted == upstream);
        if (!cache.usable) {
            std::printf("FAIL %s: burst unusable: %s\n", what.c_str(),
                        error.c_str());
            return;
        }
        for (int k = 0; k < 3; ++k) {
            const UsdTimeCode time(burst.first + k);
            const std::string at = what + " at " +
                                   std::to_string(int(time.GetValue()));
            RigExecFrameInputs plain, burstInputs;
            CHECK(RigExecSampleFrameInputsWithChainBindings(
                *evaluator, time, none, upstream, pinned, &plain, &error));
            CHECK(RigExecSampleFrameInputsWithBurstCache(
                *evaluator, time, none, upstream, &cache, &burstInputs,
                &error));
            CHECK(plain.upstream == upstream);
            CHECK(burstInputs.upstream == plain.upstream);
            CHECK(plain.values.size() == burstInputs.values.size());
            for (size_t i = 0; i < plain.values.size() &&
                               i < burstInputs.values.size();
                 ++i) {
                CHECK(plain.values[i].path == burstInputs.values[i].path);
                CHECK(plain.values[i].hasValue ==
                      burstInputs.values[i].hasValue);
                CHECK(plain.values[i].value == burstInputs.values[i].value);
            }
            CHECK(RigExecFrozenControlDigest(plain) ==
                  RigExecFrozenControlDigest(burstInputs));
            CHECK(RigExecControlStateDigest(plain, none) ==
                  RigExecControlStateDigestWithBurstCache(burstInputs, none,
                                                          &cache));
            const FrozenJob job = RunSampled(*evaluator, snapshot,
                                             std::move(burstInputs), at);
            CheckSamePose(at + " (frozen)",
                          Reference(burst.stagePath, burst.rig,
                                    {{burst.input.prim.AppendProperty(
                                          burst.input.attribute),
                                      burst.input.value,
                                      {}}},
                                    time),
                          job.pose);
        }
        // A burst is pinned to its values, as to its overrides.
        RigExecFrameInputs other;
        error.clear();
        CHECK(!RigExecSampleFrameInputsWithBurstCache(
            *evaluator, first, none, {}, &cache, &other, &error));
        CHECK(error.find("upstream") != std::string::npos);
    }
}

// Admission judges each entry before the last-wins rule, in live and in the
// sampler alike: a dropped entry given last for a path leaves the earlier
// admitted one standing.
void
TestTheSamplerAdmitsAsLiveDoes()
{
    std::printf("case: the sampler admits as live does\n");
    const std::string stagePath = Fixture("upstream_inputs.usda");
    const UsdStageRefPtr stage = UsdStage::Open(stagePath);
    auto evaluator = Make(stage, kLimbsRig);
    const UsdTimeCode time(1);
    evaluator->Evaluate(time);
    const std::shared_ptr<const RigExecFrozenProgram> snapshot =
        Freeze(*evaluator, "admission");
    const RigExecRigPose expected = Reference(
        stagePath, kLimbsRig, {{kA0Rz, VtValue(30.0), {}}}, time);
    const RigExecValueOverride good = Up(kA0Rz, VtValue(30.0));
    const RigExecValueOverride wrongType = Up(kA0Rz, VtValue(1.0f));
    for (int order = 0; order < 2; ++order) {
        const std::vector<RigExecValueOverride> given =
            order == 0 ? std::vector<RigExecValueOverride>{good, wrongType}
                       : std::vector<RigExecValueOverride>{wrongType, good};
        const std::string what =
            std::string("admission, ") + (order == 0 ? "bad last" : "bad first");
        evaluator->SetUpstreamInputs(given);
        CHECK(evaluator->GetUpstreamInputPaths() ==
              std::vector<SdfPath>{kA0Rz});
        CheckSamePose(what + " (live)", expected, evaluator->Evaluate(time));
        std::vector<RigExecUpstreamValue> raw;
        for (const RigExecValueOverride &o : given) {
            raw.push_back(RigExecUpstreamValue{
                o.prim.AppendProperty(o.attribute), o.value, 0});
        }
        RigExecFrameInputs inputs;
        std::string error;
        CHECK(RigExecSampleFrameInputs(*evaluator, time, {}, raw, &inputs,
                                       &error));
        CHECK(inputs.upstream == RigExecUpstreamValuesOf({good}));
        const FrozenJob job =
            RunSampled(*evaluator, snapshot, std::move(inputs), what);
        CheckSamePose(what + " (frozen)", expected, job.pose);
        evaluator->SetUpstreamInputs({});
        evaluator->Evaluate(time);
    }
}

// A standing value that did not move runs nothing: a repeated generation
// at the same time closes no cluster.
void
TestAStandingValueCostsNothing()
{
    std::printf("case: a standing value costs nothing\n");
    const std::string fixture = Fixture("upstream_inputs.usda");
    const UsdStageRefPtr stage = UsdStage::Open(fixture);
    auto evaluator = Make(stage, kLimbsRig);
    const RigExecBakedProgram *program = evaluator->GetBakedProgram();
    CHECK(program);
    if (!program) return;
    const auto &B = program->GetStepGraph();
    CHECK(std::all_of(B.cones.alwaysSteps.words.begin(),
                      B.cones.alwaysSteps.words.end(),
                      [](uint64_t w) { return w == 0; }));
    const UsdTimeCode time(1);
    const VtValue rotation(30.0), space(Translate(1, 0, 10));
    const std::vector<RigExecValueOverride> inputs =
        {Up(kA0Rz, rotation), Up(kUpstreamSpace, space)};
    const auto authored = evaluator->Evaluate(time);
    CheckSamePose("D2 authored", Reference(fixture, kLimbsRig, {}, time), authored);
    const auto scalar = Reference(fixture, kLimbsRig, {{kA0Rz, rotation, {}}}, time);
    const auto matrix = Reference(fixture, kLimbsRig, {{kUpstreamSpace, space, {}}}, time);
    const auto expected = Reference(fixture, kLimbsRig,
        {{kA0Rz, rotation, {}}, {kUpstreamSpace, space, {}}}, time);
    CHECK(Differences(authored, scalar) > 0);
    CHECK(Differences(authored, matrix) > 0);
    CHECK(Differences(expected, scalar) > 0);
    CHECK(Differences(expected, matrix) > 0);

    // Count actual bodies, including memo heads; cluster count alone does
    // not establish that the head pass cost nothing. Sources may still run.
    const auto work = [&B](const std::vector<RigExecOpTraceEntry> &trace) {
        std::pair<size_t, size_t> count{0, 0};
        for (const auto &entry : trace) {
            CHECK(entry.step < B.steps.size());
            if (entry.step >= B.steps.size()) continue;
            if (B.steps[entry.step].isHead) ++count.first;
            else if (!B.steps[entry.step].isSource) ++count.second;
        }
        return count;
    };
    evaluator->SetUpstreamInputs(inputs);
    const auto standing = evaluator->Evaluate(time);
    CHECK(evaluator->GetUpstreamInputPaths() == PathsOf(inputs));
    CHECK(!HasLine(standing, "upstream input "));
    CheckSamePose("D2 standing", expected, standing);
    CHECK(Differences(authored, standing) > 0);
    CHECK(evaluator->GetLastOpTrace().size() > 0);
    CHECK(work(evaluator->GetLastOpTrace()).second > 0);
    const auto snapshot = Freeze(*evaluator, "D2 standing snapshot");
    CHECK(snapshot);
    if (!snapshot) return;
    const auto snapshotUpstream = snapshot->program.lastUpstream;

    evaluator->SetUpstreamInputs(inputs);
    CheckSamePose("D2 repeated live standing", standing,
                  evaluator->Evaluate(time));
    CHECK(evaluator->GetLastOpTrace().size() == 0);
    const auto liveWork = work(evaluator->GetLastOpTrace());
    CHECK(liveWork.first == 0 && liveWork.second == 0);
    for (int repeat = 0; repeat < 2; ++repeat) {
        const auto held = RunJob(*evaluator, snapshot, time, inputs,
                                "D2 frozen standing");
        CHECK(held.report.ran && held.pose.valid);
        CheckSamePose("D2 frozen standing", standing, held.pose);
        const auto heldWork = work(held.report.region);
        CHECK(heldWork.first == 0 && heldWork.second == 0);
        CHECK(snapshot->program.lastUpstream == snapshotUpstream);
    }
    const auto lifted = RunJob(*evaluator, snapshot, time, {},
                              "D2 frozen lift");
    CHECK(lifted.report.ran && lifted.pose.valid);
    CheckSamePose("D2 frozen lift", authored, lifted.pose);
    CHECK(work(lifted.report.region).second > 0);
    CHECK(snapshot->program.lastUpstream == snapshotUpstream);
    evaluator->SetUpstreamInputs({});
    CheckSamePose("D2 live lift", authored, evaluator->Evaluate(time));
    CHECK(work(evaluator->GetLastOpTrace()).second > 0);
}

// An oracle-resolved weight object's scalar input: a constraint envelope the
// oracle computes from the generation's resolved inputs. Baked follows it
// through the oracle placement.
void
TestAnOracleReadInput()
{
    const std::string stagePath = Fixture("computed_weights.usda");
    const SdfPath scale("/Asset/Rig/Weights/Driven.inputs:scale");
    {
        const UsdStageRefPtr stage = UsdStage::Open(stagePath);
        auto evaluator =
            Make(stage, SdfPath("/Asset/Rig"));
        const RigExecBakedProgram *program = evaluator->GetBakedProgram();
        CHECK(program && program->GetUpstreamOracle().count(scale));
    }
    // At frame 5, where the driver has left 0 and the scale matters.
    RunCase("oracle input inputs:scale", stagePath, SdfPath("/Asset/Rig"), 5,
            {Up(scale, VtValue(0.25f))}, {{scale, VtValue(0.25f), {}}});

}

// Keys admission drops, in both paths: the pose is the authored one and
// each generation names the key and why.
void
TestDroppedKeys()
{
    std::printf("case: dropped keys\n");
    struct Drop {
        std::string stagePath;
        SdfPath rig;
        RigExecValueOverride input;
        std::string reason;
    };
    const SdfPath driver("/Asset/Rig/Weights/Driven.inputs:driver");
    const std::vector<Drop> drops = {
        {Fixture("upstream_inputs.usda"), kLimbsRig,
         Up(kBRootRestSpace, VtValue(Translate(1, 0, 10))),
         "the attribute is connected"},
        {Fixture("upstream_inputs.usda"), kLimbsRig,
         Up(kA0Rz, VtValue(30.0f)), "a float value on a double attribute"},
        {Fixture("computed_weights.usda"), SdfPath("/Asset/Rig"),
         Up(driver, VtValue(0.5f)), "the attribute is connected"},
    };
    for (const Drop &drop : drops) {
        const SdfPath path =
            drop.input.prim.AppendProperty(drop.input.attribute);
        {
            const UsdStageRefPtr stage = UsdStage::Open(drop.stagePath);
            auto evaluator = Make(stage, drop.rig);
            evaluator->SetUpstreamInputs({drop.input});
            const RigExecRigPose pose = evaluator->Evaluate(UsdTimeCode(1));
            CHECK(evaluator->GetUpstreamInputPaths().empty());
            const std::string line = "upstream input " + path.GetString() +
                                     ": " + drop.reason + "; ignored";
            if (!HasLine(pose, line)) {
                std::printf("FAIL: no line '%s' (%s)\n", line.c_str(),
                            "graph");
                for (const std::string &l : pose.diagnostics) {
                    std::printf("    %s\n", l.c_str());
                }
            }
            CHECK(HasLine(pose, line));
            CheckSamePose(path.GetString() + " dropped (" + "graph" +
                              ")",
                          Reference(drop.stagePath, drop.rig, {},
                                    UsdTimeCode(1)),
                          pose);
        }
    }
    // A key no listed read reaches drops against the program; with no
    // program (dynamic mode) the overlay it would ride is read by nothing.
    const SdfPath scheme("/LimbsAsset/Geom/MeshA.subdivisionScheme");
    const UsdStageRefPtr stage =
        UsdStage::Open(Fixture("upstream_inputs.usda"));
    auto evaluator = Make(stage, kLimbsRig);
    evaluator->SetUpstreamInputs(
        {Up(scheme, VtValue(TfToken("catmullClark")))});
    const RigExecRigPose pose = evaluator->Evaluate(UsdTimeCode(1));
    CHECK(HasLine(pose, "upstream input " + scheme.GetString() +
                            ": no listed read reaches it; ignored"));
    CHECK(evaluator->GetUpstreamInputPaths().empty());
}

// --- Arrays (array admission forced on) -------------------------------------

struct ArrayAdmission {
    bool previous = RigExecUpstreamArrayAdmission();
    ArrayAdmission() { RigExecSetUpstreamArrayAdmissionForTesting(true); }
    ~ArrayAdmission() { RigExecSetUpstreamArrayAdmissionForTesting(previous); }
};

// An environment variable set for a scope, restored after.
struct ScopedEnv {
    std::string name, old;
    ScopedEnv(const std::string &n, const std::string &value)
        : name(n), old(TfGetenv(n))
    {
        TfSetenv(name, value);
    }
    ~ScopedEnv()
    {
        if (old.empty()) {
            TfUnsetenv(name);
        } else {
            TfSetenv(name, old);
        }
    }
};

const SdfPath kTailRig("/TailAsset/Rig");
const SdfPath kSeg1Values("/TailAsset/Rig/Weights/Seg1W.rigExec:values");
const SdfPath kMeshAPoints("/LimbsAsset/Geom/MeshA.points");
const SdfPath kMeshASkin("/LimbsAsset/Rig/Movers/MeshASkin");
const SdfPath kJointIndices("/LimbsAsset/Rig/Movers/MeshASkin."
                            "rigExec:jointIndices");
const SdfPath kJointWeights("/LimbsAsset/Rig/Movers/MeshASkin."
                            "rigExec:jointWeights");
const SdfPath kVolumeRig("/VolumeAsset/Rig");
const SdfPath kTipCurvePoints("/VolumeAsset/Drivers/TipCurve.points");
const SdfPath kStripPoints("/VolumeAsset/Geom/Strip.points");

// The authored default of \p path on \p stagePath.
template <class A>
A
AuthoredArray(const std::string &stagePath, const SdfPath &path)
{
    A value;
    const UsdStageRefPtr stage = UsdStage::Open(stagePath);
    const UsdAttribute a = stage ? stage->GetAttributeAtPath(path)
                                 : UsdAttribute();
    CHECK(a && a.Get(&value));
    return value;
}

// MeshASkin's revision in \p evaluator's program.
const RigExecBakedProgramImpl::GeomRevision *
SkinRevision(const RigExecRigEvaluator &evaluator)
{
    const RigExecBakedProgram *program = evaluator.GetBakedProgram();
    if (!program) {
        return nullptr;
    }
    for (const auto &chain : program->GetStepGraph().chains) {
        for (const auto &revision : chain.revisions) {
            if (revision.moverPath == kMeshASkin) {
                return &revision;
            }
        }
    }
    return nullptr;
}

// Upstream \p input is dropped in each of \p modes with \p reason: the
// generation names it, admits nothing, and publishes the authored pose; in
// the sampler admits nothing either.
void
CheckDropped(const std::string &stagePath, const SdfPath &rig, double t,
             const RigExecValueOverride &input, const std::string &reason)
{
    const SdfPath path = input.prim.AppendProperty(input.attribute);
    const UsdTimeCode time(t);
    {
        const std::string what = path.GetString() + " dropped (" +
                                 "graph" + ")";
        const UsdStageRefPtr stage = UsdStage::Open(stagePath);
        auto evaluator = Make(stage, rig);
        evaluator->SetUpstreamInputs({input});
        const RigExecRigPose pose = evaluator->Evaluate(time);
        CHECK(evaluator->GetUpstreamInputPaths().empty());
        const std::string line =
            "upstream input " + path.GetString() + ": " + reason + "; ignored";
        if (!HasLine(pose, line)) {
            ++failures;
            std::printf("FAIL %s: no line '%s'\n", what.c_str(), line.c_str());
            for (const std::string &l : pose.diagnostics) {
                std::printf("    %s\n", l.c_str());
            }
        }
        CheckSamePose(what, Reference(stagePath, rig, {}, time), pose);
        if (true &&
            evaluator->GetBakedProgram()) {
            RigExecFrameInputs inputs;
            std::string error;
            CHECK(RigExecSampleFrameInputs(*evaluator, time, {},
                                           RigExecUpstreamValuesOf({input}),
                                           &inputs, &error));
            CHECK(inputs.upstream.empty());
        }
    }
}

// The array rows a bake lists: chain bases, revision and layout arrays,
// weight-object points; never the structural ones. They do not depend on
// the admission hook.
void
TestTheArrayRows()
{
    std::printf("case: array rows\n");
    const auto rowsOf = [](const std::string &stagePath, const SdfPath &rig) {
        const UsdStageRefPtr stage = UsdStage::Open(stagePath);
        auto evaluator = Make(stage, rig);
        std::map<SdfPath, RigExecUpstreamArrayRow> rows;
        for (const RigExecUpstreamArrayRow &row :
             RigExecBakedUpstreamAdmissibleArrays(*evaluator)) {
            CHECK(RigExecUpstreamArraySlotType(
                stage->GetAttributeAtPath(row.path).GetTypeName()));
            CHECK(!rows.count(row.path));
            rows[row.path] = row;
        }
        return rows;
    };
    using Time = RigExecUpstreamArrayRow::Time;
    {
        const auto rows = rowsOf(Fixture("upstream_inputs.usda"), kLimbsRig);
        const auto has = [&rows](const SdfPath &path, const TfType &type,
                                 Time time, const std::string &consumer) {
            const auto it = rows.find(path);
            const bool ok = it != rows.end() && it->second.type == type &&
                            it->second.time == time &&
                            it->second.consumer == consumer;
            if (!ok) {
                std::printf("FAIL array row %s (%s)\n", path.GetText(),
                            consumer.c_str());
            }
            CHECK(ok);
        };
        has(kMeshAPoints, TfType::Find<VtVec3fArray>(), Time::AtTime,
            "chain base");
        has(kJointIndices, TfType::Find<VtIntArray>(), Time::AtTime,
            "skin layout");
        has(kJointWeights, TfType::Find<VtFloatArray>(), Time::AtTime,
            "skin layout");
        CHECK(!rows.count(SdfPath("/LimbsAsset/Geom/MeshA.faceVertexCounts")));
    }
    {
        const auto rows = rowsOf(Example("01_FkChainTail.usda"), kTailRig);
        // Structural: the painted values (exec takes them per element) and
        // a derived target's base.
        CHECK(!rows.count(kSeg1Values));
        CHECK(!rows.count(SdfPath("/TailAsset/Geom/TailStrip.normals")));
        // The derived normals read the mesh topology.
        CHECK(rows.count(SdfPath("/TailAsset/Geom/TailStrip.faceVertexCounts")));
        CHECK(rows.count(SdfPath("/TailAsset/Geom/TailStrip.points")));
    }
    {
        // One attribute read at Default (the rest cage) and at the time
        // (the live cage): one row, both kinds.
        const auto rows = rowsOf(Example("06_LatticeBulge.usda"),
                                 SdfPath("/LatticeAsset/Rig"));
        const auto it = rows.find(SdfPath("/LatticeAsset/Geom/Cage.points"));
        CHECK(it != rows.end() && it->second.time == Time::Both &&
              it->second.type == TfType::Find<VtVec3fArray>());
    }
    {
        // A SplineIk's volume weights fold at Build.
        const auto rows = rowsOf(Fixture("computed_ik_space.usda"),
                                 SdfPath("/IkSpaceAsset/Rig"));
        CHECK(!rows.count(SdfPath("/IkSpaceAsset/Rig/Solvers/TailIK."
                                  "rigExec:volumeWeights")));
    }
    {
        // Points a mover-bound weight object reads are exec's per-element
        // reads in the dynamic walk: structural, even as a chain base.
        const auto rows = rowsOf(Example("11_VolumeWeights.usda"),
                                 kVolumeRig);
        CHECK(!rows.count(kTipCurvePoints));
        CHECK(!rows.count(kStripPoints));
    }
}

// The painted values of a weight object (Seg1W, read by the Seg1Skin matrix
// mover). This case gated the array list: with the value placed as an exec
// value override, the dynamic walk refused it ("Expected override of value
// key '...Seg1W.rigExec:values [__computeValue]' to have type 'float'; got
// 'VtArray<float>'": exec takes the painted table as a vectorized input) and
// did not follow. So painted values and indices are structural in every
// backend: dropped as no listed read.
void
TestAPaintedWeightIsDropped()
{
    std::printf("case: painted values dropped\n");
    const ArrayAdmission arrays;
    CheckDropped(Example("01_FkChainTail.usda"), kTailRig, 1012,
                 Up(kSeg1Values,
                    VtValue(VtFloatArray{0.5f, 0.5f, 0.5f, 0.5f, 0.5f, 0.9f,
                                         0.1f, 0.4f, 0.3f, 0.2f})),
                 "no listed read reaches it");
}

// The chain base points of MeshA's chain, every point moved by (0, 1, 0):
// the base every revision skins, read from the upstream layer and never the
// interactive overlay, in each backend.
void
TestUpstreamChainBasePoints()
{
    const ArrayAdmission arrays;
    const std::string stagePath = Fixture("upstream_inputs.usda");
    const VtVec3fArray authored =
        AuthoredArray<VtVec3fArray>(stagePath, kMeshAPoints);
    const auto moved = [&authored](float dy) {
        VtVec3fArray points = authored;
        for (GfVec3f &p : points) {
            p += GfVec3f(0.0f, dy, 0.0f);
        }
        return points;
    };
    RunCase("chain base points", stagePath, kLimbsRig, 5,
            {Up(kMeshAPoints, VtValue(moved(1.0f)))},
            {{kMeshAPoints, VtValue(moved(1.0f)), {}}});
    const UsdTimeCode time(5);
    {
        const std::string what =
            std::string("chain base points (") + "graph" + ")";
        const UsdStageRefPtr stage = UsdStage::Open(stagePath);
        auto evaluator = Make(stage, kLimbsRig);
        evaluator->Evaluate(time);
        evaluator->SetUpstreamInputs({Up(kMeshAPoints, VtValue(moved(1.0f)))});
        const RigExecRigPose first = evaluator->Evaluate(time);
        const RigExecRigPose expected = Reference(
            stagePath, kLimbsRig,
            {{kMeshAPoints, VtValue(moved(1.0f)), {}}}, time);
        CheckSamePose(what + ": first", expected, first);
        // The same value again: nothing re-reads or moves.
        evaluator->SetUpstreamInputs({Up(kMeshAPoints, VtValue(moved(1.0f)))});
        const RigExecRigPose same = evaluator->Evaluate(time);
        CheckSamePose(what + ": the same again", expected, same);
        CHECK(evaluator->GetLastOpTrace().empty());
        // Another value: the base is read again.
        evaluator->SetUpstreamInputs({Up(kMeshAPoints, VtValue(moved(2.0f)))});
        CheckSamePose(what + ": another value",
                      Reference(stagePath, kLimbsRig,
                                {{kMeshAPoints, VtValue(moved(2.0f)), {}}},
                                time),
                      evaluator->Evaluate(time));
        // An interactive points value does not replace the upstream chain base.
        evaluator->SetUpstreamInputs({Up(kMeshAPoints, VtValue(moved(1.0f)))});
        CheckSamePose(what + ": back to the first value", expected,
                      evaluator->Evaluate(time));
        {
            evaluator->SetInteractiveOverrides(
                {Up(kMeshAPoints, VtValue(moved(3.0f)))});
            CheckSamePose(what + ": interactive points ignored", expected,
                          evaluator->Evaluate(time));
            evaluator->ClearInteractiveOverrides();
            CheckSamePose(what + ": interactive lifted", expected,
                          evaluator->Evaluate(time));
        }
        evaluator->SetUpstreamInputs({});
        CheckSamePose(what + ": lifted",
                      Reference(stagePath, kLimbsRig, {}, time),
                      evaluator->Evaluate(time));
    }
}

// MeshASkin's painted skin weights: the SkinTopology op builds a handle from
// the moved leaf, its indices are the partition's, so the chunks adopt it
// and keep running.
void
TestUpstreamJointWeights()
{
    const ArrayAdmission arrays;
    const ScopedEnv always("RIGEXEC_BAKED_CHUNK_ALWAYS", "1");
    const ScopedEnv verts("RIGEXEC_BAKED_CHUNK_VERTS", "5");
    const std::string stagePath = Fixture("upstream_inputs.usda");
    VtFloatArray weights = AuthoredArray<VtFloatArray>(stagePath,
                                                       kJointWeights);
    CHECK(weights.size() == 20);
    // Vertex 0 and vertex 3 shared between the two joints.
    weights[0] = 0.5f;
    weights[1] = 0.5f;
    weights[6] = 0.25f;
    weights[7] = 0.75f;
    RunCase("skin jointWeights", stagePath, kLimbsRig, 5,
            {Up(kJointWeights, VtValue(weights))},
            {{kJointWeights, VtValue(weights), {}}});

    const UsdStageRefPtr stage = UsdStage::Open(stagePath);
    auto evaluator = Make(stage, kLimbsRig);
    const UsdTimeCode time(5);
    evaluator->Evaluate(time);
    const RigExecBakedProgramImpl::GeomRevision *revision =
        SkinRevision(*evaluator);
    CHECK(revision && revision->chunked && revision->chunks.size() >= 2);
    if (!revision || !revision->chunked) {
        return;
    }
    const auto before = revision->topology;
    evaluator->SetUpstreamInputs({Up(kJointWeights, VtValue(weights))});
    evaluator->Evaluate(time);
    revision = SkinRevision(*evaluator);
    CHECK(revision != nullptr);
    if (!revision) {
        return;
    }
    // A new handle, adopted: the chunks ran with it.
    CHECK(revision->topology && revision->topology != before);
    CHECK(revision->partitionTopology == revision->topology);
    CHECK(!revision->partitionStale);
    for (const RigExecBakedProgramImpl::GeomChunk &chunk : revision->chunks) {
        CHECK(chunk.ok);
    }
    std::printf("  jointWeights: adopted, %zu chunk(s) ran\n",
                revision->chunks.size());
}

// An upstream jointIndices on a chunked revision that binds a first-half
// vertex to LimbA1: the partition is stale while it stands, so the revision
// runs whole and no chunk reads a joint its key never declared, even when
// only LimbA1 moves. Lifted, the next handle matches the partition and the
// chunks run again.
void
TestUpstreamChunkedJointIndices()
{
    std::printf("case: chunked jointIndices\n");
    const ArrayAdmission arrays;
    const ScopedEnv always("RIGEXEC_BAKED_CHUNK_ALWAYS", "1");
    const ScopedEnv verts("RIGEXEC_BAKED_CHUNK_VERTS", "5");
    const std::string stagePath = Fixture("upstream_inputs_chunked.usda");
    VtIntArray indices = AuthoredArray<VtIntArray>(stagePath, kJointIndices);
    CHECK(indices.size() == 10);
    indices[0] = 1;
    const RigExecValueOverride input = Up(kJointIndices, VtValue(indices));
    const UsdTimeCode time(1);
    const std::vector<RigExecValueOverride> drag = {Up(kA1Rz, VtValue(40.0))};
    const Authored dragged{kA1Rz, VtValue(40.0), {}};
    const Authored repainted{kJointIndices, VtValue(indices), {}};

    const UsdStageRefPtr stage = UsdStage::Open(stagePath);
    auto evaluator = Make(stage, kLimbsRig);
    CheckSamePose("chunked: authored",
                  Reference(stagePath, kLimbsRig, {}, time),
                  evaluator->Evaluate(time));
    const RigExecBakedProgramImpl::GeomRevision *revision =
        SkinRevision(*evaluator);
    CHECK(revision && revision->chunked && revision->chunks.size() == 2);
    if (!revision || !revision->chunked || revision->chunks.size() != 2) {
        return;
    }
    CHECK(revision->chunks[0].key == std::vector<int>{0});
    CHECK(revision->chunks[1].key == std::vector<int>{1});
    CHECK(!revision->partitionStale);

    evaluator->SetUpstreamInputs({input});
    CheckSamePose("chunked: standing",
                  Reference(stagePath, kLimbsRig,
                            {repainted}, time),
                  evaluator->Evaluate(time));
    // Only LimbA1 moves.
    evaluator->SetInteractiveOverrides(drag);
    const RigExecRigPose moved = evaluator->Evaluate(time);
    CheckSamePose("chunked: standing, LimbA1 moved (authored-stage reference)",
                  Reference(stagePath, kLimbsRig,
                            {repainted, dragged}, time),
                  moved);
    CheckSamePose("chunked: standing, LimbA1 moved (baked reference)",
                  Reference(stagePath, kLimbsRig,
                            {repainted, dragged}, time),
                  moved);
    revision = SkinRevision(*evaluator);
    CHECK(revision != nullptr);
    if (!revision) {
        return;
    }
    CHECK(revision->chunks[0].key == std::vector<int>{0});
    CHECK(revision->chunks[1].key == std::vector<int>{1});
    CHECK(revision->partitionStale);
    CHECK(revision->partitionTopology != revision->topology);
    for (const RigExecBakedProgramImpl::GeomChunk &chunk : revision->chunks) {
        CHECK(!chunk.ok);
    }
    // A frozen job under the same value and drag agrees.
    {
        const std::shared_ptr<const RigExecFrozenProgram> snapshot =
            Freeze(*evaluator, "chunked");
        RigExecFrameInputs inputs;
        std::string error;
        CHECK(RigExecSampleFrameInputs(*evaluator, time, drag,
                                       RigExecUpstreamValuesOf({input}),
                                       &inputs, &error));
        CHECK(inputs.upstream == RigExecUpstreamValuesOf({input}));
        const FrozenJob job = RunSampled(*evaluator, snapshot,
                                         std::move(inputs), "chunked job");
        CheckSamePose("chunked: frozen, standing, LimbA1 moved", moved,
                      job.pose);
    }
    // Lifted: the handle matches the partition again.
    evaluator->SetUpstreamInputs({});
    CheckSamePose("chunked: lifted",
                  Reference(stagePath, kLimbsRig, {dragged},
                            time),
                  evaluator->Evaluate(time));
    revision = SkinRevision(*evaluator);
    CHECK(revision != nullptr);
    if (!revision) {
        return;
    }
    CHECK(!revision->partitionStale);
    CHECK(revision->partitionTopology == revision->topology);
    for (const RigExecBakedProgramImpl::GeomChunk &chunk : revision->chunks) {
        CHECK(chunk.ok);
    }
    evaluator->ClearInteractiveOverrides();
    CheckSamePose("chunked: drag lifted",
                  Reference(stagePath, kLimbsRig, {}, time),
                  evaluator->Evaluate(time));
}

// An upstream jointIndices naming an influence that does not exist: the
// layout does not validate and the skin fails, passing its base through, as
// the same array authored as a value edit fails it. (Compile refuses an
// invalid authored default outright, so the reference edits an evaluator
// already compiled.) Dynamic, baked and a frozen job agree; the lift
// restores the authored skin.
void
TestUpstreamInvalidLayout()
{
    std::printf("case: invalid jointIndices\n");
    const ArrayAdmission arrays;
    const std::string stagePath = Fixture("upstream_inputs.usda");
    VtIntArray indices = AuthoredArray<VtIntArray>(stagePath, kJointIndices);
    indices[3] = 2;  // MeshASkin binds two influences
    const std::vector<RigExecValueOverride> inputs = {
        Up(kJointIndices, VtValue(indices))};
    const UsdTimeCode time(5);
    const auto skinFailed = [](const RigExecRigPose &pose) {
        for (const std::string &line : pose.diagnostics) {
            if (line.rfind("MoverFailed", 0) == 0 &&
                line.find("MeshASkin") != std::string::npos) {
                return true;
            }
        }
        return false;
    };
    {
        const std::string what =
            std::string("invalid jointIndices (") + "graph" + ")";
        const UsdStageRefPtr referenceStage = UsdStage::Open(stagePath);
        auto reference = Make(referenceStage, kLimbsRig);
        reference->Evaluate(time);
        {
            UsdEditContext context(referenceStage,
                                   referenceStage->GetSessionLayer());
            CHECK(referenceStage->GetAttributeAtPath(kJointIndices)
                      .Set(indices));
        }
        const RigExecRigPose expected = reference->Evaluate(time);
        CHECK(skinFailed(expected));

        const UsdStageRefPtr stage = UsdStage::Open(stagePath);
        auto evaluator = Make(stage, kLimbsRig);
        const RigExecRigPose authored = evaluator->Evaluate(time);
        std::shared_ptr<const RigExecFrozenProgram> snapshot;
        {
            snapshot = Freeze(*evaluator, what);
        }
        evaluator->SetUpstreamInputs(inputs);
        const RigExecRigPose standing = evaluator->Evaluate(time);
        CHECK(evaluator->GetUpstreamInputPaths() == PathsOf(inputs));
        CheckSamePose(what + ": standing", expected, standing);
        CHECK(skinFailed(standing));
        if (snapshot) {
            const FrozenJob job =
                RunJob(*evaluator, snapshot, time, inputs, what);
            CheckSamePose(what + ": frozen", standing, job.pose);
        }
        evaluator->SetUpstreamInputs({});
        const RigExecRigPose lifted = evaluator->Evaluate(time);
        CheckSamePose(what + ": lifted", authored, lifted);
        CHECK(!skinFailed(lifted));
    }
}

// Painted skin weights set per frame, as a time-varying source is pulled:
// live and a frozen job at each frame equal a stage authoring that frame's
// array, and the jobs key apart.
void
TestUpstreamTimeVaryingJointWeights()
{
    std::printf("case: jointWeights per frame\n");
    const ArrayAdmission arrays;
    const std::string stagePath = Fixture("upstream_inputs.usda");
    const VtFloatArray authored =
        AuthoredArray<VtFloatArray>(stagePath, kJointWeights);
    const auto at = [&authored](int f) {
        VtFloatArray w = authored;
        w[2] = 0.75f - 0.1f * float(f);
        w[3] = 0.25f + 0.1f * float(f);
        return w;
    };
    {
        const UsdStageRefPtr stage = UsdStage::Open(stagePath);
        auto evaluator = Make(stage, kLimbsRig);
        std::shared_ptr<const RigExecFrozenProgram> snapshot;
        std::set<uint64_t> digests;
        for (int f = 1; f <= 5; ++f) {
            const std::string what = std::string("jointWeights per frame (") +
                                     "graph" + ") at " +
                                     std::to_string(f);
            const std::vector<RigExecValueOverride> inputs = {
                Up(kJointWeights, VtValue(at(f)))};
            evaluator->SetUpstreamInputs(inputs);
            const RigExecRigPose expected =
                Reference(stagePath, kLimbsRig,
                          {{kJointWeights, VtValue(at(f)), {}}},
                          UsdTimeCode(f));
            const RigExecRigPose live = evaluator->Evaluate(UsdTimeCode(f));
            CheckSamePose(what, expected, live);
            if (!snapshot) {
                snapshot = Freeze(*evaluator, what);
            }
            const FrozenJob job =
                RunJob(*evaluator, snapshot, UsdTimeCode(f), inputs, what);
            CheckSamePose(what + " (frozen)", expected, job.pose);
            CHECK(job.inputs.upstream.size() == 1 &&
                  job.inputs.upstream[0].foldHash ==
                      RigExecUpstreamFoldHash(VtValue(at(f))));
            digests.insert(RigExecControlStateDigest(job.inputs));
        }
        {
            CHECK(digests.size() == 5);
        }
    }
}

// A lattice cage: the rest cage (read at Default) and the live cage (at the
// time) read one attribute, so an upstream value, authored-level at every
// time, makes them equal and the deformation the identity, as a stage
// whose default overrides the cage's samples does. The stage cage varies,
// so admission reads its count at each time.
void
TestUpstreamLatticeCage()
{
    const ArrayAdmission arrays;
    const std::string stagePath = Example("06_LatticeBulge.usda");
    const SdfPath cage("/LatticeAsset/Geom/Cage.points");
    VtVec3fArray bulged;
    {
        const UsdStageRefPtr stage = UsdStage::Open(stagePath);
        CHECK(stage->GetAttributeAtPath(cage).Get(&bulged,
                                                  UsdTimeCode(1024)));
    }
    RunCase("lattice cage (identity while it stands)", stagePath,
            SdfPath("/LatticeAsset/Rig"), 1024, {Up(cage, VtValue(bulged))},
            {{cage, VtValue(bulged), {}}});
}

// Bursts under array values sample as the plain route does.
void
TestABurstCarriesUpstreamArrays()
{
    std::printf("case: a burst carries upstream arrays\n");
    const ArrayAdmission arrays;
    const std::string stagePath = Fixture("upstream_inputs.usda");
    VtVec3fArray points = AuthoredArray<VtVec3fArray>(stagePath, kMeshAPoints);
    for (GfVec3f &p : points) {
        p += GfVec3f(0.0f, 1.0f, 0.0f);
    }
    VtFloatArray weights = AuthoredArray<VtFloatArray>(stagePath,
                                                       kJointWeights);
    weights[0] = 0.5f;
    weights[1] = 0.5f;
    CheckBurst({"chain base points", stagePath, kLimbsRig,
                Up(kMeshAPoints, VtValue(points)), 1});
    CheckBurst({"skin jointWeights", stagePath, kLimbsRig,
                Up(kJointWeights, VtValue(weights)), 1});
}

// A burst judges admission once, so a listed array over a stage array
// that varies (condition 4 is per time) fails the build, whether or not it
// is admitted at the burst's judging time: such frames take the plain
// route.
void
TestABurstRefusesAnArrayOverAVaryingStage()
{
    std::printf("case: a burst refuses an array over a varying stage\n");
    const ArrayAdmission arrays;
    const std::string stagePath = Example("06_LatticeBulge.usda");
    const SdfPath rig("/LatticeAsset/Rig");
    const SdfPath cage("/LatticeAsset/Geom/Cage.points");
    const UsdStageRefPtr stage = UsdStage::Open(stagePath);
    VtVec3fArray bulged;
    CHECK(stage->GetAttributeAtPath(cage).Get(&bulged, UsdTimeCode(1024)));
    VtVec3fArray shorter = bulged;
    shorter.pop_back();
    auto evaluator = Make(stage, rig);
    evaluator->Evaluate(UsdTimeCode(1024));
    const RigExecBakedProgram *program = evaluator->GetBakedProgram();
    CHECK(program);
    if (!program) {
        return;
    }
    RigExecChainSampleBindings pinned;
    std::string error;
    CHECK(RigExecBindChainSampleInputs(*evaluator, &pinned, &error));
    for (const VtVec3fArray &value : {bulged, shorter}) {
        RigExecBurstSampleCache cache;
        error.clear();
        CHECK(!RigExecBuildBurstSampleCache(
            *program, pinned, {},
            RigExecUpstreamValuesOf({Up(cage, VtValue(value))}),
            RigExecFrameCacheEpochDigest(*evaluator), &cache, &error));
        CHECK(!cache.usable);
        CHECK(error.find("time-varying") != std::string::npos);
    }
}

// Condition 4 against an attribute whose default and one time sample
// differ in count: the memo, first filled at Default, does not answer the
// numeric time with the default's count.
void
TestTheCountMemoReadsEachOpinion()
{
    std::printf("case: the count memo reads each opinion\n");
    const ArrayAdmission arrays;
    const UsdStageRefPtr stage = UsdStage::CreateInMemory();
    const UsdPrim prim = stage->DefinePrim(SdfPath("/P"));
    const UsdAttribute a =
        prim.CreateAttribute(TfToken("w"), SdfValueTypeNames->FloatArray);
    CHECK(a.Set(VtFloatArray{1, 2}));
    CHECK(a.Set(VtFloatArray{1, 2, 3}, UsdTimeCode(5)));
    const std::map<SdfPath, TfType> listed = {
        {a.GetPath(), TfType::Find<VtFloatArray>()}};
    RigExecUpstreamCountMemo memo;
    CHECK(RigExecUpstreamDropReason(stage, nullptr, a.GetPath(),
                                    VtValue(VtFloatArray{4, 5}),
                                    UsdTimeCode::Default(), &memo, &listed)
              .empty());
    CHECK(RigExecUpstreamDropReason(stage, nullptr, a.GetPath(),
                                    VtValue(VtFloatArray{4, 5, 6}),
                                    UsdTimeCode(5), &memo, &listed)
              .empty());
    CHECK(RigExecUpstreamDropReason(stage, nullptr, a.GetPath(),
                                    VtValue(VtFloatArray{4, 5}),
                                    UsdTimeCode(5), &memo, &listed) ==
          "its element count 2 differs from the stage value's 3");
}

// Condition 4's memo holds the stage count until a notice reaches the
// attribute: a stage edit that changes the count drops the standing value
// at the next generation, with the count in the line.
void
TestTheCountMemoFollowsEdits()
{
    std::printf("case: the count memo follows edits\n");
    const ArrayAdmission arrays;
    const std::string stagePath = Fixture("upstream_inputs.usda");
    const VtFloatArray weights =
        AuthoredArray<VtFloatArray>(stagePath, kJointWeights);
    VtFloatArray painted = weights;
    painted[0] = 0.5f;
    painted[1] = 0.5f;
    {
        const std::string what =
            std::string("count memo (") + "graph" + ")";
        const UsdStageRefPtr stage = UsdStage::Open(stagePath);
        auto evaluator = Make(stage, kLimbsRig);
        evaluator->SetUpstreamInputs({Up(kJointWeights, VtValue(painted))});
        evaluator->Evaluate(UsdTimeCode(5));
        CHECK(evaluator->GetUpstreamInputPaths() ==
              std::vector<SdfPath>{kJointWeights});
        // The stage's weights grow to 22 (and the indices with them): the
        // standing 20 no longer match.
        {
            UsdEditContext context(stage, stage->GetSessionLayer());
            VtFloatArray longer = weights;
            longer.push_back(0.0f);
            longer.push_back(0.0f);
            VtIntArray indices =
                AuthoredArray<VtIntArray>(stagePath, kJointIndices);
            indices.push_back(0);
            indices.push_back(0);
            CHECK(stage->GetAttributeAtPath(kJointWeights).Set(longer));
            CHECK(stage->GetAttributeAtPath(kJointIndices).Set(indices));
        }
        const RigExecRigPose pose = evaluator->Evaluate(UsdTimeCode(5));
        CHECK(evaluator->GetUpstreamInputPaths().empty());
        const std::string line = "upstream input " +
                                 kJointWeights.GetString() +
                                 ": its element count 20 differs from the "
                                 "stage value's 22; ignored";
        if (!HasLine(pose, line)) {
            ++failures;
            std::printf("FAIL %s: no line '%s'\n", what.c_str(),
                        line.c_str());
        }
    }
}

// Array keys admission drops: a count that differs from the stage's, the
// structural arrays, and, with the hook off, every array.
void
TestDroppedArrayKeys()
{
    std::printf("case: dropped array keys\n");
    const std::string limbs = Fixture("upstream_inputs.usda");
    VtFloatArray shortWeights =
        AuthoredArray<VtFloatArray>(limbs, kJointWeights);
    shortWeights.pop_back();
    {
        const ArrayAdmission arrays;
        CheckDropped(limbs, kLimbsRig, 5,
                     Up(kJointWeights, VtValue(shortWeights)),
                     "its element count 19 differs from the stage value's "
                     "20");
        CheckDropped(Fixture("computed_ik_space.usda"),
                     SdfPath("/IkSpaceAsset/Rig"), 5,
                     Up(SdfPath("/IkSpaceAsset/Rig/Solvers/TailIK."
                                "rigExec:volumeWeights"),
                        VtValue(VtFloatArray{1, 1, 1, 1, 1})),
                     "no listed read reaches it");
        VtVec3fArray normals(10, GfVec3f(0.0f, 1.0f, 0.0f));
        CheckDropped(Example("01_FkChainTail.usda"), kTailRig, 1012,
                     Up(SdfPath("/TailAsset/Geom/TailStrip.normals"),
                        VtValue(normals)),
                     "no listed read reaches it");
        // Points a mover-bound weight object reads: exec would refuse the
        // value per element and the dynamic walk keep the stage's, so no
        // backend admits them (a curve weight's curve, and a chain base the
        // volumes measure).
        const std::string volumes = Example("11_VolumeWeights.usda");
        for (const SdfPath &path : {kTipCurvePoints, kStripPoints}) {
            VtVec3fArray points = AuthoredArray<VtVec3fArray>(volumes, path);
            for (GfVec3f &p : points) {
                p += GfVec3f(0.5f, 0.0f, 0.0f);
            }
            CheckDropped(volumes, kVolumeRig, 1024, Up(path, VtValue(points)),
                         "no listed read reaches it");
        }
    }
    // With the hook off, the keys the cases above admit.
    const bool arraysWereAdmitted = RigExecUpstreamArrayAdmission();
    RigExecSetUpstreamArrayAdmissionForTesting(false);
    VtVec3fArray points = AuthoredArray<VtVec3fArray>(limbs, kMeshAPoints);
    points[0] += GfVec3f(0.0f, 1.0f, 0.0f);
    VtFloatArray weights = AuthoredArray<VtFloatArray>(limbs, kJointWeights);
    weights[0] = 0.5f;
    weights[1] = 0.5f;
    VtIntArray indices = AuthoredArray<VtIntArray>(limbs, kJointIndices);
    indices[0] = 1;
    indices[1] = 0;
    for (const RigExecValueOverride &input :
         {Up(kMeshAPoints, VtValue(points)),
          Up(kJointWeights, VtValue(weights)),
          Up(kJointIndices, VtValue(indices))}) {
        CheckDropped(limbs, kLimbsRig, 5, input,
                     "array values are not admitted");
    }
    RigExecSetUpstreamArrayAdmissionForTesting(arraysWereAdmitted);
}

// Connected posed-space inputs are compiled and frozen by the graph.
void
TestThePosedVariant()
{
    const std::string stagePath = Fixture("upstream_inputs_posed.usda");
    const GfMatrix4d moved = Translate(1, 0, 10);
    const std::vector<RigExecValueOverride> inputs = {
        Up(kA0Rz, VtValue(30.0)), Up(kUpstreamSpace, VtValue(moved))};
    const std::vector<Authored> edits = {{kA0Rz, VtValue(30.0), {}},
                                         {kUpstreamSpace, VtValue(moved), {}}};
    auto evaluator = Make(UsdStage::Open(stagePath), kLimbsRig);
    evaluator->SetUpstreamInputs(inputs);
    const RigExecRigPose pose = evaluator->Evaluate(UsdTimeCode(1));
    CHECK(evaluator->GetUpstreamInputPaths() == PathsOf(inputs));
    CheckSamePose("posed variant authored reference",
                  Reference(stagePath, kLimbsRig, edits, UsdTimeCode(1)), pose);
    CHECK(Differences(Reference(stagePath, kLimbsRig, {}, UsdTimeCode(1)), pose) != 0);
    auto snapshot = Freeze(*evaluator, "connected posed space");
    if (snapshot) {
        const FrozenJob job = RunJob(*evaluator, snapshot, UsdTimeCode(1), inputs,
                                    "connected posed space");
        CheckSamePose("posed variant frozen", pose, job.pose);
    }
}

// Suspension restores upstream values while preserving frozen source coverage.
void
TestTheBakeGuard()
{
    std::printf("case: bake guard\n");
    const std::string reason =
        "upstream inputs standing (the exporter does not expose them yet)";
    const UsdStageRefPtr stage =
        UsdStage::Open(Fixture("upstream_inputs.usda"));
    auto evaluator = Make(stage, kLimbsRig);
    std::vector<std::string> reasons;
    CHECK(!HasReason(reasons, reason));
    evaluator->Evaluate(UsdTimeCode(1));
    std::string error;
    CHECK(RigExecCanFreezeProgram(*evaluator, &error));
    const std::vector<RigExecValueOverride> inputs = {
        Up(kA0Rz, VtValue(30.0))};
    evaluator->SetUpstreamInputs(inputs);
    error.clear();
    CHECK(RigExecCanFreezeProgram(*evaluator, &error));
    const RigExecRigPose standing = evaluator->Evaluate(UsdTimeCode(1));
    CHECK(evaluator->GetBakedProgram() != nullptr);
    reasons.clear();
    CHECK(!HasReason(reasons, reason));
    {
        RigExecScopedUpstreamSuspension suspension(*evaluator);
        CHECK(!evaluator->HasUpstreamInputs());
        reasons.clear();
        CHECK(!HasReason(reasons, reason));
        error.clear();
        CHECK(RigExecCanFreezeProgram(*evaluator, &error));
        CheckSamePose("suspended",
                      Reference(Fixture("upstream_inputs.usda"), kLimbsRig, {}, UsdTimeCode(1)),
                      evaluator->Evaluate(UsdTimeCode(1)));
        CHECK(RigExecCanFreezeProgram(*evaluator, &error));
    }
    CHECK(evaluator->GetUpstreamInputs() == inputs);
    reasons.clear();
    CHECK(!HasReason(reasons, reason));
    CHECK(RigExecCanFreezeProgram(*evaluator, &error));
    CheckSamePose("restored", standing, evaluator->Evaluate(UsdTimeCode(1)));
}

// --- .rigexec playback ----------------------------------------------------

// Where the playback cases write the binaries they open.
std::string g_scratch;

// The upstream list a host hands live and playback alike, in the order
// given (each backend judges each entry before the last-wins rule).
std::vector<RigExecUpstreamValue>
UpstreamList(const std::vector<RigExecValueOverride> &inputs)
{
    std::vector<RigExecUpstreamValue> values;
    for (const RigExecValueOverride &o : inputs) {
        values.push_back(RigExecUpstreamValue{
            o.prim.AppendProperty(o.attribute), o.value, 0});
    }
    return values;
}

std::vector<std::string>
DropLines(const RigExecRigPose &pose)
{
    std::vector<std::string> lines;
    for (const std::string &line : pose.diagnostics) {
        if (line.rfind("upstream input ", 0) == 0) {
            lines.push_back(line);
        }
    }
    return lines;
}

// \p evaluator's authored epoch baked at \p t and written as \p name.
std::string
WriteBake(RigExecRigEvaluator &evaluator, double t, const std::string &name)
{
    std::vector<uint8_t> bytes;
    std::string error;
    const bool baked = RigExecTestBakeAt(evaluator, t, &bytes, &error);
    if (!baked) {
        std::printf("FAIL bake %s: %s\n", name.c_str(), error.c_str());
    }
    CHECK(baked);
    const std::string path = g_scratch + "/" + name + ".rigexec";
    std::ofstream stream(path, std::ios::binary);
    stream.write(reinterpret_cast<const char *>(bytes.data()),
                 std::streamsize(bytes.size()));
    return path;
}

// \p reader's last run against \p pose, output domain by output domain and
// the property chains' values, bit for bit.
void
CheckSameRun(const std::string &what, const RigExecRigPose &pose,
             const RigExecRuntimeReader *reader)
{
    CHECK(reader);
    if (!reader) {
        return;
    }
    std::vector<std::string> diffs;
    const bool outputs = RigExecCompareRuntimeOutputs(pose, *reader, &diffs);
    const bool properties =
        RigExecCompareRuntimeProperties(pose, *reader, &diffs);
    if (!outputs || !properties) {
        std::printf("FAIL %s: playback differs from live:\n", what.c_str());
        for (const std::string &line : diffs) {
            std::printf("    %s\n", line.c_str());
        }
    }
    CHECK(outputs);
    CHECK(properties);
}

// Playback parity (R3-M5): \p inputs on a playback session of a bake of the
// rig at \p t made with no upstream values, over the stage with \p edits
// authored. The session admits the keys live admits, reports the keys live
// drops with live's lines, in live's order, and publishes live's outputs
// while they stand, at \p t and then at each of \p later (Apply samples the
// Animated inputs there, and the keys on them are set again), and after the
// lift at the last of those times (an Animated key re-reads the stage
// there). The admitted values move live's pose at \p t. Returns the admitted
// keys that sit on Animated inputs of the file.
std::set<SdfPath>
CheckPlaybackParity(const std::string &what, const std::string &stagePath,
                    const SdfPath &rig, double t,
                    const std::vector<RigExecValueOverride> &inputs,
                    const std::vector<Authored> &edits = {},
                    const std::vector<double> &later = {})
{
    std::printf("case: playback parity: %s\n", what.c_str());
    const UsdTimeCode time(t);
    std::set<SdfPath> animated;
    const UsdStageRefPtr stage = OpenAuthored(stagePath, edits);
    if (!stage) {
        return animated;
    }
    auto evaluator = Make(stage, rig);
    const RigExecRigPose authored = evaluator->Evaluate(time);
    std::string name = what;
    for (char &c : name) {
        c = std::isalnum(static_cast<unsigned char>(c)) ? c : '_';
    }
    const std::string file = WriteBake(*evaluator, t, name);

    auto store = std::make_shared<RigExecSnapshotStore>();
    RigExecBakedPlayback play(stage, rig, store);
    std::string error;
    const bool opened = play.Open(file, &error);
    if (!opened) {
        std::printf("FAIL %s: open: %s\n", what.c_str(), error.c_str());
    }
    CHECK(opened);
    if (!opened) {
        return animated;
    }
    CHECK(play.EvaluateAndPublishResult(time).ok);
    CheckSameRun(what + ": authored", authored, play.GetReaderForTesting());

    evaluator->SetUpstreamInputs(inputs);
    const RigExecRigPose live = evaluator->Evaluate(time);
    const std::vector<SdfPath> admitted = evaluator->GetUpstreamInputPaths();
    const std::vector<std::string> drops = DropLines(live);
    play.SetUpstreamInputs(UpstreamList(inputs));
    CHECK(play.GetUpstreamInputPaths() == admitted);
    CHECK(play.EvaluateAndPublishResult(time).ok);
    if (play.GetUpstreamInputPaths() != admitted ||
        play.GetUpstreamDropLines() != drops) {
        std::printf("FAIL %s: admission differs\n", what.c_str());
        for (const SdfPath &path : admitted) {
            std::printf("    live admits %s\n", path.GetText());
        }
        for (const SdfPath &path : play.GetUpstreamInputPaths()) {
            std::printf("    playback admits %s\n", path.GetText());
        }
        for (const std::string &line : drops) {
            std::printf("    live: %s\n", line.c_str());
        }
        for (const std::string &line : play.GetUpstreamDropLines()) {
            std::printf("    playback: %s\n", line.c_str());
        }
    }
    CHECK(play.GetUpstreamInputPaths() == admitted);
    CHECK(play.GetUpstreamDropLines() == drops);
    for (const SdfPath &path : admitted) {
        const RigExecRuntimeReader *reader = play.GetReaderForTesting();
        size_t index = 0;
        if (reader->FindInput(path.GetString(), &index) &&
            reader->GetInputInfo(index).animated) {
            animated.insert(path);
        }
    }
    CheckSameRun(what + ": standing", live, play.GetReaderForTesting());
    UsdTimeCode last = time;
    for (const double at : later) {
        last = UsdTimeCode(at);
        const RigExecRigPose moved = evaluator->Evaluate(last);
        CHECK(play.EvaluateAndPublishResult(last).ok);
        CHECK(play.GetUpstreamInputPaths() == admitted);
        CHECK(play.GetUpstreamDropLines() == DropLines(moved));
        CheckSameRun(what + ": standing at " + TfStringify(at), moved,
                     play.GetReaderForTesting());
    }

    evaluator->SetUpstreamInputs({});
    const RigExecRigPose lifted = evaluator->Evaluate(last);
    play.SetUpstreamInputs({});
    CHECK(play.EvaluateAndPublishResult(last).ok);
    CHECK(play.GetUpstreamInputPaths().empty());
    CHECK(play.GetUpstreamDropLines().empty());
    CheckSameRun(what + ": lifted", lifted, play.GetReaderForTesting());
    return animated;
}

// The scalar cases, the dropped keys and a hook-off array, each on a
// playback session against live.
void
TestPlaybackParity()
{
    const std::string limbs = Fixture("upstream_inputs.usda");
    const SdfPath offset(
        "/LimbsAsset/Rig/Solvers/LimbBIK.rigExec:upperLengthOffset");
    const SdfPath scheme("/LimbsAsset/Geom/MeshA.subdivisionScheme");
    VtFloatArray weights = AuthoredArray<VtFloatArray>(limbs, kJointWeights);
    weights[0] = 0.5f;
    weights[1] = 0.5f;
    // Admitted, dropped (connected, another type after an admitted entry
    // for the same path, no listed read, no attribute, an array with the
    // admission hook off), in one list. The D2 keys sit on non-Animated
    // inputs.
    const std::set<SdfPath> limbsAnimated = CheckPlaybackParity(
        "limbs", limbs, kLimbsRig, 1,
        {Up(kA0Rz, VtValue(30.0)),
         Up(kUpstreamSpace, VtValue(Translate(1, 0, 10))),
         Up(kBRootRestSpace, VtValue(Translate(1, 0, 10))),
         Up(kA0Rz, VtValue(30.0f)), Up(offset, VtValue(0.5)),
         Up(scheme, VtValue(TfToken("catmullClark"))),
         Up(SdfPath("/LimbsAsset/Rig.noSuchInput"), VtValue(1.0))},
        {}, {4, 7.5});
    CHECK(limbsAnimated.empty());
    // A key on an Animated input (A1's keyed avar): set again after each
    // Apply that sampled, and on the lift read from the stage at the time
    // without a sample.
    CHECK(CheckPlaybackParity("animated input", limbs, kLimbsRig, 3,
                              {Up(kA1Rz, VtValue(20.0))}, {}, {5, 8}) ==
          std::set<SdfPath>{kA1Rz});
    // The solver input where it moves the pose.
    CheckPlaybackParity("solver input", limbs, kLimbsRig, 5,
                        {Up(offset, VtValue(0.5))});
    // A layout scalar of a fixed skin layout rebuilds in both backends.
    CheckPlaybackParity(
        "skin elementSize", limbs, kLimbsRig, 1,
        {Up(SdfPath("/LimbsAsset/Rig/Movers/MeshASkin.rigExec:elementSize"),
            VtValue(1))});
    CheckPlaybackParity(
        "chain target and chain mover input",
        Example("09_PropertyMathMovers.usda"), kPropRig, 1012,
        {Up(SdfPath("/PropMathAsset/Rig/Channels/Dials.rigExec:gain"),
            VtValue(0.5f)),
         Up(SdfPath("/PropMathAsset/Rig/Movers/OffsetLift.inputs:value"),
            VtValue(GfVec3f(0, 7, 0)))});
    CheckPlaybackParity(
        "rest:tx", Example("01_FkChainTail.usda"), kTailRig, 1012,
        {Up(SdfPath("/TailAsset/Rig/Joints/Seg1/Seg2.rest:tx"),
            VtValue(0.5))});
    CheckPlaybackParity(
        "skin mover inputs:defaultWeight", Example("biped/Biped.usda"),
        SdfPath("/Biped/Rig"), 1,
        {Up(SdfPath("/Biped/Rig/Movers/skin_body_geo/body_geo_skin."
                    "inputs:defaultWeight"),
            VtValue(0.5f))});
    // The oracle-read scalar (the runtime's oracle reads it as an input on
    // every call) and a connected listed hop.
    CheckPlaybackParity(
        "oracle input", Fixture("computed_weights.usda"),
        SdfPath("/Asset/Rig"), 5,
        {Up(SdfPath("/Asset/Rig/Weights/Driven.inputs:scale"),
            VtValue(0.25f)),
         Up(SdfPath("/Asset/Rig/Weights/Driven.inputs:driver"),
            VtValue(0.5f))});
}

// A Token key goes through SetInputToken, once with text the file holds
// (another skin mover authors it, so the bake interned it) and once with
// text it does not (the reader interns it): live and playback agree.
void
TestAPlaybackTokenKey()
{
    const std::string limbs = Fixture("upstream_inputs.usda");
    const SdfPath method(
        "/LimbsAsset/Rig/Movers/MeshASkin.rigExec:skinningMethod");
    const SdfPath other(
        "/LimbsAsset/Rig/Movers/MeshBSkin.rigExec:skinningMethod");
    const TfToken dq("dualQuaternion");
    // Whether a token input of the file defaults to \p text.
    const auto holds = [](const RigExecRuntimeReader &reader,
                          const std::string &text) {
        for (size_t i = 0; i < reader.GetInputCount(); ++i) {
            const RigExecRuntimeInputInfo &info = reader.GetInputInfo(i);
            if (info.type == RrInputTag::Token &&
                reader.GetTokenText(info.defaultValue.token) == text) {
                return true;
            }
        }
        return false;
    };
    for (const bool held : {false, true}) {
        const std::string what =
            held ? "token the file holds" : "token the file does not hold";
        const std::vector<Authored> edits =
            held ? std::vector<Authored>{{other, VtValue(dq), {}}}
                 : std::vector<Authored>{};
        // At frame 6, where A1 bends LimbA1 against LimbA0, so dual
        // quaternions and linear blending part.
        CheckPlaybackParity(what, limbs, kLimbsRig, 6,
                            {Up(method, VtValue(dq))}, edits, {9});
        // The reader side: the input holds the text, by the file's id when
        // the file holds it.
        const UsdStageRefPtr stage = OpenAuthored(limbs, edits);
        auto evaluator = Make(stage, kLimbsRig);
        evaluator->Evaluate(UsdTimeCode(6));
        const std::string file =
            WriteBake(*evaluator, 6, held ? "token_held" : "token_new");
        RigExecBakedPlayback play(stage, kLimbsRig,
                                  std::make_shared<RigExecSnapshotStore>());
        std::string error;
        CHECK(play.Open(file, &error));
        const RigExecRuntimeReader *reader = play.GetReaderForTesting();
        if (!reader) {
            continue;
        }
        CHECK(holds(*reader, dq.GetString()) == held);
        size_t index = 0;
        CHECK(reader->FindInput(method.GetString(), &index));
        CHECK(reader->GetInputInfo(index).type == RrInputTag::Token);
        play.SetUpstreamInputs(UpstreamList({Up(method, VtValue(dq))}));
        CHECK(play.EvaluateAndPublishResult(UsdTimeCode(6)).ok);
        CHECK(reader->GetTokenText(reader->GetInputValue(index).token) ==
              dq.GetString());
        size_t otherIndex = 0;
        CHECK(reader->FindInput(other.GetString(), &otherIndex));
        CHECK((reader->GetInputValue(index).token ==
               reader->GetInputInfo(otherIndex).defaultValue.token) == held);
    }
}

// Double arrays currently have a real Default-only leaf: wire knots.
// Numeric animation marks its slot Animated but must not make it sampled.
void
TestDefaultOnlyKnotsLift()
{
    const auto stage = UsdStage::CreateInMemory();
    CHECK(rigExec::RigExecInputReplayImportFromString(stage->GetRootLayer(), R"USD(#usda 1.0
 def Xform "Asset" {
  def RigExecRoot "Rig" {
   uniform token rigExec:partition = "Asset"
   def Scope "Movers" {
    def RigExecCurveMover "Wire" (prepend apiSchemas = ["RigExecMoverAPI"]) {
     uniform token rigExec:mode = "wire"
     rel rigExec:moves = </Asset/Mesh.points>
     rel rigExec:driverCurve = </Asset/Curve>
     rel rigExec:bindCoordinates = </Asset/Bind.st>
    }
   }
  }
  def Mesh "Mesh" {
   point3f[] points = [(0.25, 0, 0), (0.75, 0, 0)]
  }
  def Scope "Bind" {
   float2[] st = [(0.25, 0), (0.75, 0)]
  }
  def NurbsCurves "Curve" {
   int[] curveVertexCounts = [2]
   int[] order = [2]
   double[] knots = [0, 0, 1, 1]
   double[] knots.timeSamples = {1: [0, 0, 1, 1], 3: [0, 0, 3, 3]}
   point3f[] points = [(0, 0, 0), (1, 0, 0)]
   point3f[] points.timeSamples = {
    1: [(0, 0, 0), (1, 0, 0)],
    3: [(0, 0, 0), (1, 1, 0)]
   }
  }
 }
)USD"));
    const SdfPath rig("/Asset/Rig"), path("/Asset/Curve.knots");
    auto evaluator = Make(stage, rig);
    evaluator->Evaluate(UsdTimeCode(1));
    RigExecBakedPlayback play(stage, rig,
                              std::make_shared<RigExecSnapshotStore>());
    std::string error;
    CHECK(play.Open(WriteBake(*evaluator, 1, "default_only_knots"), &error));
    const auto reader = play.GetReaderForTesting();
    CHECK(reader);
    if (!reader) return;
    size_t index = 0;
    const bool listed = reader->FindInput(path.GetString(), &index);
    CHECK(listed);
    if (!listed) return;
    CHECK(reader->GetInputInfo(index).type == RrInputTag::DoubleArray &&
          reader->GetInputInfo(index).animated);
    CHECK(!RigExecRuntimeStageArrayInputs::CanSample(*reader, index));
    const VtDoubleArray initial{0, 0, 1, 1};
    const VtDoubleArray standing{0, 0, 2, 2};
    std::vector<uint8_t> bridgeBytes;
    CHECK(RigExecTestBakeAt(*evaluator, 1, &bridgeBytes, &error));
    auto bridgeReader = RigExecRuntimeReader::Open(
        bridgeBytes.data(), bridgeBytes.size(), &error);
    CHECK(bridgeReader);
    if (!bridgeReader) return;
    const std::string bridgeRefusal = "no sampled array at slot " +
                                      std::to_string(index);
    CHECK(!RigExecRuntimeStageArrayInputs::SetSample(
        *bridgeReader, index,
        RigExecTestArrayView(VtValue(standing)), &error));
    CHECK(error == bridgeRefusal);
    error.clear();
    CHECK(!RigExecRuntimeStageArrayInputs::ClearSample(
        *bridgeReader, index, &error));
    CHECK(error == bridgeRefusal);
    const auto checkValue = [&](const VtDoubleArray &expected) {
        RigExecRuntimeArray value;
        CHECK(reader->GetInputArrayAt(index, &value));
        CHECK(value.count == expected.size());
        CHECK(value.count == expected.size() &&
              std::memcmp(value.data, expected.cdata(),
                          sizeof(double) * expected.size()) == 0);
    };
    for (const bool move : {false, true}) {
        const auto input = std::vector<RigExecValueOverride>{
            Up(path, VtValue(standing))};
        evaluator->SetUpstreamInputs(input);
        play.SetUpstreamInputs(UpstreamList(input));
        CHECK(play.EvaluateAndPublishResult(UsdTimeCode(1)).ok);
        checkValue(standing);
        CheckSameRun("Default-only knots standing", evaluator->Evaluate(UsdTimeCode(1)), reader);
        evaluator->SetUpstreamInputs({});
        play.SetUpstreamInputs(UpstreamList({}));
        const double frame = move ? 3 : 1;
        CHECK(play.EvaluateAndPublishResult(UsdTimeCode(frame)).ok);
        checkValue(initial);
        CheckSameRun("Default-only knots lift", evaluator->Evaluate(UsdTimeCode(frame)), reader);
    }
    std::vector<RigExecBakeStaticEntry> entries;
    CHECK(RigExecBakeStaticReport(*evaluator, &entries, &error));
    CHECK(std::any_of(entries.begin(), entries.end(), [&](const auto &entry) {
        return entry.source == path.GetString();
    }));
}

// Real array playback sessions compare with standing upstream values.
void
TestTheArrayPlaybackLegs()
{
    TestDefaultOnlyKnotsLift();
    const std::string limbs = Fixture("upstream_inputs.usda");
    VtVec3fArray points = AuthoredArray<VtVec3fArray>(limbs, kMeshAPoints);
    for (GfVec3f &p : points) { p[1] += 1.0f; }
    CheckPlaybackParity("array chain base", limbs, kLimbsRig, 5,
                        {Up(kMeshAPoints, VtValue(points))});
    VtFloatArray weights = AuthoredArray<VtFloatArray>(limbs, kJointWeights);
    weights[0] = weights[1] = 0.5f;
    CheckPlaybackParity("array skin weights", limbs, kLimbsRig, 5,
                        {Up(kJointWeights, VtValue(weights))});
    VtIntArray indices = AuthoredArray<VtIntArray>(limbs, kJointIndices);
    indices[0] = 1;
    indices[1] = 0;
    CheckPlaybackParity("array skin indices", limbs, kLimbsRig, 5,
                        {Up(kJointIndices, VtValue(indices))});
    weights.pop_back();
    CheckPlaybackParity("array count refused", limbs, kLimbsRig, 5,
                        {Up(kJointWeights, VtValue(weights))});
    indices[3] = 2;
    CheckPlaybackParity("array invalid skin layout", limbs, kLimbsRig, 5,
                        {Up(kJointIndices, VtValue(indices))});
    const std::string chunked = Fixture("upstream_inputs_chunked.usda");
    VtIntArray chunkIndices =
        AuthoredArray<VtIntArray>(chunked, kJointIndices);
    chunkIndices[0] = 1;
    CheckPlaybackParity("array chunked indices", chunked, kLimbsRig, 5,
                        {Up(kJointIndices, VtValue(chunkIndices))});

    // A genuinely Animated unweighted chain supports changing stage counts;
    // animated skin layouts remain an explicit original-export refusal.
    const SdfPath sampledRig("/SampleAsset/Rig");
    const SdfPath sampledPath("/SampleAsset/Geom/P.points");
    const auto varyingStage = UsdStage::CreateInMemory();
    varyingStage->DefinePrim(sampledRig, TfToken("RigExecRoot"));
    const auto pointsPrim = varyingStage->DefinePrim(
        sampledPath.GetPrimPath(), TfToken("Points"));
    const auto attr = pointsPrim.CreateAttribute(
        TfToken("points"), SdfValueTypeNames->Point3fArray);
    const VtVec3fArray initial{GfVec3f(0, 0, 0), GfVec3f(1, 0, 0),
                             GfVec3f(0, 1, 0)};
    CHECK(attr.Set(initial));
    CHECK(attr.Set(initial, UsdTimeCode(3)));
    VtVec3fArray stageMoved = initial;
    for (auto &point : stageMoved) point[0] += 0.25f;
    CHECK(attr.Set(stageMoved, UsdTimeCode(7)));
    CHECK(attr.Set(initial, UsdTimeCode(9)));
    VtVec3fArray shortStage = initial;
    shortStage.pop_back();
    CHECK(attr.Set(shortStage, UsdTimeCode(10)));
    const auto joint = varyingStage->DefinePrim(
        sampledRig.AppendPath(SdfPath("Joints/J")), TfToken("RigExecJoint"));
    CHECK(joint.CreateAttribute(TfToken("avars:ty"), SdfValueTypeNames->Double)
              .Set(2.0));
    const auto mover = varyingStage->DefinePrim(
        sampledRig.AppendPath(SdfPath("Movers/Move")), TfToken("RigExecMatrixMover"));
    CHECK(mover.ApplyAPI(TfToken("RigExecMoverAPI")));
    CHECK(mover.CreateRelationship(TfToken("rigExec:moves"))
              .SetTargets({sampledPath}));
    CHECK(mover.CreateRelationship(TfToken("rigExec:transform"))
              .SetTargets({joint.GetPath()}));
    auto evaluator = Make(varyingStage, sampledRig);
    evaluator->Evaluate(UsdTimeCode(3));
    RigExecBakedPlayback play(varyingStage, sampledRig,
                              std::make_shared<RigExecSnapshotStore>());
    std::string error;
    const bool opened = play.Open(
        WriteBake(*evaluator, 3, "array_varying_source"), &error);
    CHECK(opened && play.GetReaderForTesting());
    if (!opened || !play.GetReaderForTesting()) return;
    size_t slot = 0;
    const bool listed = play.GetReaderForTesting()->FindInput(
        sampledPath.GetString(), &slot);
    CHECK(listed);
    if (!listed) return;
    CHECK(play.GetReaderForTesting()->GetInputInfo(slot).animated);
    const auto set = [&](const std::vector<RigExecValueOverride> &inputs) {
        evaluator->SetUpstreamInputs(inputs);
        play.SetUpstreamInputs(UpstreamList(inputs));
    };
    const auto compare = [&](const char *what, double frame) {
        CHECK(play.EvaluateAndPublishResult(UsdTimeCode(frame)).ok);
        CheckSameRun(std::string(what) + " native",
                     evaluator->Evaluate(UsdTimeCode(frame)), play.GetReaderForTesting());
    };
    VtVec3fArray standing = initial;
    standing[0][0] = 0.6f;
    const std::vector<RigExecValueOverride> input{Up(sampledPath, VtValue(standing))};
    set(input);
    for (const double frame : {3., 5., 6., 7.}) {
        compare("array unchanged standing source", frame);
        CHECK(evaluator->GetUpstreamInputPaths() == PathsOf(input));
        CHECK(play.GetUpstreamInputPaths() == PathsOf(input));
        if (frame == 5) {
            auto unedited = Make(varyingStage, sampledRig);
            std::vector<std::string> diffs;
            CHECK(!RigExecCompareRuntimeOutputs(unedited->Evaluate(UsdTimeCode(frame)),
                                                 *play.GetReaderForTesting(), &diffs));
        }
        CHECK(play.EvaluateAndPublishResult(UsdTimeCode(frame)).ok);
        CHECK(play.GetReaderForTesting()->GetCounters().executedOpCount == 0);
    }
    const std::vector<RigExecValueOverride> bad{Up(sampledPath, VtValue(shortStage))};
    set(bad);
    compare("array same-time admission refusal lifts standing key", 7);
    CHECK(play.GetUpstreamInputPaths().empty());
    CHECK(play.GetUpstreamDropLines() == DropLines(evaluator->Evaluate(UsdTimeCode(7))));
    set(input);
    compare("array reinstall standing key", 7);
    set({});
    compare("array moving-time lift", 8);
    set(bad);
    compare("array first admission refusal", 9);
    // Live admits this frame's two points; authored runtime sets keep three.
    set(bad);
    compare("array runtime first-refusal restores short stage sample", 10);
    CHECK(evaluator->GetUpstreamInputPaths() == PathsOf(bad));
    CHECK(play.GetUpstreamInputPaths().empty());
    CHECK(play.GetUpstreamDropLines() == std::vector<std::string>{
        "upstream input " + sampledPath.GetString() + ": " +
        sampledPath.GetString() + " holds 3 elements; an array set keeps that count, not 2; ignored"});

    const std::string lattice = Example("06_LatticeBulge.usda");
    const SdfPath cage("/LatticeAsset/Geom/Cage.points");
    const UsdStageRefPtr stage = UsdStage::Open(lattice);
    VtVec3fArray bulged;
    CHECK(stage->GetAttributeAtPath(cage).Get(&bulged, UsdTimeCode(1024)));
    CheckPlaybackParity("array lattice Both identity", lattice,
                        SdfPath("/LatticeAsset/Rig"), 1024,
                        {Up(cage, VtValue(bulged))});
}

// A standing upstream list is reported, never printed into file defaults.
void
CheckStandingBake(const std::string &what, const std::string &stagePath,
                  const SdfPath &rig, double time,
                  const std::vector<RigExecValueOverride> &requested)
{
    const auto stage = UsdStage::Open(stagePath);
    auto evaluator = Make(stage, rig);
    const RigExecRigPose authored = evaluator->Evaluate(UsdTimeCode(time));
    RigExecBakeOpts opts;
    opts.time = time;
    RigExecBakeResult baseline;
    std::string error;
    CHECK(RigExecBakeToBinary(*evaluator, opts, &baseline, &error));
    CHECK(baseline.upstreamInputs.empty());
    evaluator->SetUpstreamInputs(requested);
    const RigExecRigPose standing = evaluator->Evaluate(UsdTimeCode(time));
    const auto admitted = evaluator->GetUpstreamInputPaths();
    CHECK(!admitted.empty());
    CHECK(Differences(authored, standing) != 0);
    for (const auto &entry : requested) {
        const auto path = entry.prim.AppendProperty(entry.attribute);
        if (stage->GetAttributeAtPath(path)) {
            CHECK(std::binary_search(admitted.begin(), admitted.end(), path));
        }
    }
    std::vector<std::string> names;
    for (const auto &path : admitted) {
        names.push_back(path.GetString());
    }
    std::sort(names.begin(), names.end());
    RigExecBakeResult result;
    const bool baked = RigExecBakeToBinary(*evaluator, opts, &result, &error);
    if (!baked) {
        std::printf("FAIL %s bake: %s\n", what.c_str(), error.c_str());
    }
    CHECK(baked);
    CHECK(result.bytes == baseline.bytes);
    CHECK(result.upstreamInputs == names);
    CHECK(std::is_sorted(result.upstreamInputs.begin(),
                         result.upstreamInputs.end()));
    CHECK(evaluator->GetUpstreamInputs() == requested);
    CheckSamePose(what + " restored", standing,
                  evaluator->Evaluate(UsdTimeCode(time)));
    if (!baked) {
        return;
    }
    auto reader = RigExecRuntimeReader::Open(result.bytes.data(),
                                             result.bytes.size(), &error);
    CHECK(reader);
    if (!reader) {
        return;
    }
    CHECK(reader->Execute(&error));
    CheckSameRun(what + " authored defaults", authored, reader.get());
    for (const auto &entry : requested) {
        const auto path = entry.prim.AppendProperty(entry.attribute);
        if (!std::binary_search(admitted.begin(), admitted.end(), path)) {
            continue;
        }
        size_t index = 0;
        CHECK(reader->FindInput(path.GetString(), &index));
        const auto tag = reader->GetInputInfo(index).type;
        if (RrInputTagIsArray(tag)) {
            RigExecRuntimeArray array;
            CHECK(RigExecInputArrayFrom(entry.value, tag, &array));
            CHECK(reader->GetInputInfo(index).defaultCount == array.count);
            CHECK(reader->SetInputArrayAt(index, array, &error));
        } else if (tag == RrInputTag::Token) {
            CHECK(reader->SetInputToken(path.GetString(),
                  entry.value.UncheckedGet<TfToken>().GetString(), &error));
        } else {
            RrInputValue value;
            CHECK(RigExecInputValueFrom(entry.value, tag, &value));
            CHECK(reader->SetSampledInputAt(index, value, &error));
        }
    }
    CHECK(reader->Execute(&error));
    CheckSameRun(what + " reapplied", standing, reader.get());
    reader->ResetInputs();
    CHECK(reader->Execute(&error));
    CheckSameRun(what + " runtime lifted", authored, reader.get());

    // A failure after suspension must restore even ignored requested keys,
    // without publishing any partial result.
    RigExecBakeResult untouched;
    untouched.bytes = {42};
    untouched.upstreamInputs = {"sentinel"};
    untouched.pathReadsWritten = 17;
    untouched.pathReadsEnumerated = 19;
    opts.presentation = {0, 1, 2};
    CHECK(!RigExecBakeToBinary(*evaluator, opts, &untouched, &error));
    CHECK(untouched.bytes == std::vector<uint8_t>{42});
    CHECK(untouched.upstreamInputs == std::vector<std::string>{"sentinel"});
    CHECK(untouched.pathReadsWritten == 17 &&
          untouched.pathReadsEnumerated == 19);
    CHECK(evaluator->GetUpstreamInputs() == requested);
    CheckSamePose(what + " failed bake restored", standing,
                  evaluator->Evaluate(UsdTimeCode(time)));
}

void
TestStandingBakes()
{
    const std::string limbs = Fixture("upstream_inputs.usda");
    VtVec3fArray points = AuthoredArray<VtVec3fArray>(limbs, kMeshAPoints);
    points[0] += GfVec3f(0, 1, 0);
    VtFloatArray weights = AuthoredArray<VtFloatArray>(limbs, kJointWeights);
    weights[0] = 0.25f;
    weights[1] = 0.75f;
    CheckStandingBake("mixed upstream bake", limbs, kLimbsRig, 5,
        {Up(kA0Rz, VtValue(30.0)),
         Up(kUpstreamSpace, VtValue(Translate(1, 0, 10))),
         Up(kMeshAPoints, VtValue(points)),
         Up(kJointWeights, VtValue(weights)),
         Up(SdfPath("/LimbsAsset/Rig.noSuchInput"), VtValue(1.0))});
    CheckStandingBake("rest constants bake", Example("01_FkChainTail.usda"),
        kTailRig, 1012,
        {Up(SdfPath("/TailAsset/Rig/Joints/Seg1.rest:space"),
            VtValue(Translate(0, 0.5, 0)))});
    const std::string lattice = Example("06_LatticeBulge.usda");
    const SdfPath cage("/LatticeAsset/Geom/Cage.points");
    const auto stage = UsdStage::Open(lattice);
    VtVec3fArray bulged;
    CHECK(stage->GetAttributeAtPath(cage).Get(&bulged, UsdTimeCode(1024)));
    CheckStandingBake("Both cage bake", lattice,
        SdfPath("/LatticeAsset/Rig"), 1024, {Up(cage, VtValue(bulged))});
}

void
TestAnOracleBakeListsTheAttribute()
{
    CheckStandingBake("oracle standing bake", Fixture("computed_weights.usda"),
        SdfPath("/Asset/Rig"), 5,
        {Up(SdfPath("/Asset/Rig/Weights/Driven.inputs:scale"),
            VtValue(0.25f))});
}

// --- The admission sets against the exporter -------------------------------

TfType
TagType(fb::InputTag tag)
{
    switch (tag) {
    case fb::InputTag::Double: return TfType::Find<double>();
    case fb::InputTag::Float: return TfType::Find<float>();
    case fb::InputTag::Bool: return TfType::Find<bool>();
    case fb::InputTag::Int: return TfType::Find<int>();
    case fb::InputTag::Token: return TfType::Find<TfToken>();
    case fb::InputTag::Matrix4d: return TfType::Find<GfMatrix4d>();
    case fb::InputTag::Vec3d: return TfType::Find<GfVec3d>();
    case fb::InputTag::Vec3f: return TfType::Find<GfVec3f>();
    case fb::InputTag::Vec3fArray: return TfType::Find<VtVec3fArray>();
    case fb::InputTag::Vec2fArray: return TfType::Find<VtVec2fArray>();
    case fb::InputTag::DoubleArray: return TfType::Find<VtDoubleArray>();
    case fb::InputTag::FloatArray: return TfType::Find<VtFloatArray>();
    case fb::InputTag::IntArray: return TfType::Find<VtIntArray>();
    }
    return TfType();
}

// Unconnected, with a stage value: the attributes admission can keep.
bool
Settable(const UsdStageRefPtr &stage, const SdfPath &path)
{
    const UsdAttribute a = stage->GetAttributeAtPath(path);
    if (!a || !a.HasValue()) {
        return false;
    }
    SdfPathVector connections;
    return !(a.HasAuthoredConnections() && a.GetConnections(&connections) &&
             !connections.empty());
}

void
CheckSetsAgainstTheExporter(const std::string &stagePath, double t)
{
    const UsdStageRefPtr stage = UsdStage::Open(stagePath);
    CHECK(stage);
    if (!stage) {
        return;
    }
    const SdfPath rig = FindRig(stage);
    auto evaluator =
        Make(stage, rig);
    const RigExecBakedProgram *program = evaluator->GetBakedProgram();
    CHECK(program);
    if (!program) {
        std::printf("FAIL %s: no program\n", stagePath.c_str());
        return;
    }
    RigExecBakePathTable paths;
    std::string error;
    RigExecBakeComputedCapture capture(*evaluator, t, &paths, &error);
    CHECK(capture.Valid());
    if (!capture.Valid()) {
        std::printf("FAIL %s: capture: %s\n", stagePath.c_str(),
                    error.c_str());
        return;
    }
    const RigExecBakeInputs &C = capture.GetInputs();
    const std::vector<std::string> &names = capture.GetListedInputNames();
    CHECK(names.size() == C.listedInputs);

    // Listed inputs, restricted, against the admissible set, restricted.
    std::map<SdfPath, TfType> listed, admissible;
    for (size_t i = 0; i < names.size() && i < C.inputs.size(); ++i) {
        const SdfPath path(names[i]);
        if (Settable(stage, path)) {
            listed[path] = TagType(C.inputs[i].type());
        }
    }
    std::set<TfType> slotTypes;
    for (const fb::InputTag tag :
         {fb::InputTag::Double, fb::InputTag::Float, fb::InputTag::Bool,
          fb::InputTag::Int, fb::InputTag::Token, fb::InputTag::Matrix4d,
          fb::InputTag::Vec3d, fb::InputTag::Vec3f}) {
        slotTypes.insert(TagType(tag));
    }
    for (const auto &[path, type] : program->GetUpstreamAdmissible()) {
        // The scalar admission set uses the existing scalar input tags.
        CHECK(slotTypes.count(type));
        if (Settable(stage, path)) {
            admissible[path] = type;
        }
    }
    std::map<SdfPath, TfType> publicArrays;
    for (const RigExecUpstreamArrayRow &row :
         RigExecBakedUpstreamAdmissibleArrays(*evaluator)) {
        publicArrays[row.path] = row.type;
        if (Settable(stage, row.path)) {
            admissible[row.path] = row.type;
        }
    }
    std::map<SdfPath, TfType> exportedArrays;
    for (size_t i = 0; i < C.listedInputs; ++i) {
        if (RigExecFormatIsArrayTag(C.inputs[i].type())) {
            exportedArrays[SdfPath(names[i])] = TagType(C.inputs[i].type());
        }
    }
    CHECK(exportedArrays == publicArrays);
    if (listed != admissible) {
        std::printf("FAIL %s: admissible (%zu) != listed (%zu)\n",
                    stagePath.c_str(), admissible.size(), listed.size());
        size_t shown = 0;
        for (const auto &[path, type] : listed) {
            const auto found = admissible.find(path);
            if ((found == admissible.end() || found->second != type) &&
                shown++ < 10) {
                std::printf("    listed only: %s %s\n", path.GetText(),
                            type.GetTypeName().c_str());
            }
        }
        shown = 0;
        for (const auto &[path, type] : admissible) {
            const auto found = listed.find(path);
            if ((found == listed.end() || found->second != type) &&
                shown++ < 10) {
                std::printf("    admissible only: %s %s\n", path.GetText(),
                            type.GetTypeName().c_str());
            }
        }
    }
    CHECK(listed == admissible);

    // The oracle set: every listed slot on a read of an object the
    // exporter's descending pass marks resolved.
    const RigExecBakedProgramImpl &B = program->GetStepGraph();
    std::vector<char> resolved(C.weightObjects.size(), 0);
    const auto mark = [&resolved](int index) {
        if (index >= 0 && size_t(index) < resolved.size()) {
            resolved[size_t(index)] = 1;
        }
    };
    for (const int32_t index : C.constraintWeightObjectIndex) {
        mark(index);
    }
    for (const auto &chain : C.propertyChains) {
        for (const auto &revision : chain.revisions) {
            mark(revision.envelope);
        }
    }
    for (const auto &chain : B.chains) {
        for (const auto &revision : chain.revisions) {
            if (revision.weightCurrentPhase) {
                mark(revision.weightObject);
            }
        }
    }
    std::set<SdfPath> oracle;
    for (size_t i = C.weightObjects.size(); i-- > 0;) {
        if (!resolved[i]) {
            continue;
        }
        const fb::RigExecWireWeightObject &w = C.weightObjects[i];
        for (const auto *read :
             {&w.defaultWeight, &w.driver,    &w.scale,     &w.bias,
              &w.strength,      &w.invert,    &w.falloffMin,
              &w.falloffMax,    &w.scaleXPos, &w.scaleYPos,
              &w.scaleZPos,     &w.scaleXNeg, &w.scaleYNeg,
              &w.scaleZNeg,     &w.scaleX,    &w.scaleY,
              &w.scaleZ,        &w.extentU,   &w.extentV}) {
            if (*read) {
                for (const uint32_t slot : (*read)->walk) {
                    if (slot < names.size()) {
                        oracle.insert(SdfPath(names[slot]));
                    }
                }
            }
        }
        mark(w.base);
        for (const int32_t input : w.inputs) {
            mark(input);
        }
    }
    if (oracle != program->GetUpstreamOracle()) {
        std::printf("FAIL %s: oracle set (%zu) != exporter's (%zu)\n",
                    stagePath.c_str(), program->GetUpstreamOracle().size(),
                    oracle.size());
    }
    CHECK(oracle == program->GetUpstreamOracle());
    std::printf("  %s: %zu admissible, %zu oracle\n", stagePath.c_str(),
                admissible.size(), oracle.size());
}

void
TestEveryAdmissiblePathIsAnInput()
{
    std::printf("case: every admissible path is an input\n");
    // Every baking example, at its first listed frame.
    std::ifstream table(g_examples + "/../tests/exampleFixtures.cmake");
    std::string line;
    size_t examples = 0;
    while (std::getline(table, line)) {
        const size_t open = line.find('"');
        const size_t close = line.rfind('"');
        if (open == std::string::npos || close <= open) {
            continue;
        }
        const std::vector<std::string> fields = TfStringSplit(
            line.substr(open + 1, close - open - 1), "|");
        if (fields.size() < 7 || fields[6] != "YES") {
            continue;
        }
        const std::vector<std::string> frames = TfStringSplit(fields[1], ",");
        CheckSetsAgainstTheExporter(Example(fields[0]),
                                    frames.empty() ? 1.0
                                                   : std::stod(frames[0]));
        ++examples;
    }
    CHECK(examples > 0);
    CheckSetsAgainstTheExporter(Example("biped/Biped_stack_anim.usda"), 1);
    for (const char *name :
         {"computed_weights.usda", "computed_chains.usda",
          "computed_ik_space.usda", "upstream_inputs.usda"}) {
        CheckSetsAgainstTheExporter(Fixture(name), 1);
    }
    CheckSetsAgainstTheExporter(Fixture("computed_path_reads.usda"), 10);
}

}  // namespace

int
main(int argc, char **argv)
{
    if (argc < 2) {
        std::printf("usage: testRigExecUpstreamInputs <examplesDir>\n");
        return 2;
    }
    g_examples = argv[1];
    const std::string resources = SchemaResourceDir();
    if (PlugRegistry::GetInstance().RegisterPlugins(resources).empty()) {
        std::printf("FATAL: no schema plugin found at %s\n",
                    resources.c_str());
        return 2;
    }
    std::setvbuf(stdout, nullptr, _IONBF, 0);
    const std::filesystem::path scratch =
        std::filesystem::temp_directory_path() /
        ("rigexec-upstream-" +
         std::to_string(
             std::chrono::steady_clock::now().time_since_epoch().count()));
    std::filesystem::create_directories(scratch);
    g_scratch = scratch.generic_string();
    try {
        TestAChainTarget();
        TestAChainMoverInput();
        TestARestChannel();
        TestASkinMoverDefaultWeight();
        TestAPinnedHop();
        TestAConstantAvar();
        TestASolverInput();
        TestUpstreamThroughADrag();
        TestATimeVaryingValue();
        TestALiftReachesAnOlderSnapshot();
        TestABurstCarriesUpstream();
        TestTheSamplerAdmitsAsLiveDoes();
        TestAStandingValueCostsNothing();
        TestAnOracleReadInput();
        TestDroppedKeys();
        TestThePosedVariant();
        TestTheBakeGuard();
        TestPlaybackParity();
        TestAPlaybackTokenKey();
        TestAnOracleBakeListsTheAttribute();
        TestStandingBakes();
        // Structural painted arrays, followed by admitted array paths.
        TestAPaintedWeightIsDropped();
        TestTheArrayRows();
        TestUpstreamChainBasePoints();
        TestUpstreamJointWeights();
        TestUpstreamChunkedJointIndices();
        TestUpstreamInvalidLayout();
        TestUpstreamTimeVaryingJointWeights();
        TestUpstreamLatticeCage();
        TestABurstCarriesUpstreamArrays();
        TestABurstRefusesAnArrayOverAVaryingStage();
        TestTheCountMemoReadsEachOpinion();
        TestTheCountMemoFollowsEdits();
        TestDroppedArrayKeys();
        TestTheArrayPlaybackLegs();
        // Structural: the same answer in every mode, so once, unverified.
        if (TfGetenv("RIGEXEC_EVALUATION_MODE").empty() && !ConeVerify()) {
            TestEveryAdmissiblePathIsAnInput();
        }
    } catch (const std::exception &error) {
        std::printf("FAIL: threw: %s\n", error.what());
        return 1;
    }
    std::error_code removed;
    std::filesystem::remove_all(scratch, removed);
    if (failures) {
        std::printf("%d FAILURE(S)\n", failures);
        return 1;
    }
    std::printf("all tests passed\n");
    return 0;
}
