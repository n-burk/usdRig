// Upstream inputs through the evaluator API, with no scene index. An upstream
// value is authored-level: with one standing, the dynamic walk and the baked
// program each publish what a fresh evaluator of a stage that authors the
// value publishes, and once it is lifted, what the authored stage publishes.
// Admission keeps a value only on an unconnected attribute with a stage
// value, of its own input-slot type, that a read a bake lists as an input
// reaches; every other key is reported and ignored by both paths.
// In baked mode each value case also runs frozen jobs, which carry the
// values in their sampled vector: a job equals live under the same values,
// whichever values the snapshot it clones held, and runs only what moved.
// Registered plain and under the parity entries, where every baked
// generation is also compared with the dynamic walk and, with
// RIGEXEC_BAKED_VERIFY_CONES, the cone run with a forced run of everything.
// argv[1] = path to the examples directory.
#include "rigExec/bakedProgram.h"
#include "rigExec/bakedProgramImpl.h"
#include "rigExec/bakedTrace.h"
#include "rigExec/frameCache.h"
#include "rigExec/frozenContext.h"
#include "rigExec/rigEvaluator.h"
#include "rigExecBake/computedCapture.h"
#include "rigExecBake/pathTable.h"

#include "pxr/base/gf/matrix4d.h"
#include "pxr/base/gf/vec3d.h"
#include "pxr/base/gf/vec3f.h"
#include "pxr/base/plug/registry.h"
#include "pxr/base/tf/getenv.h"
#include "pxr/base/tf/pathUtils.h"
#include "pxr/base/tf/stringUtils.h"
#include "pxr/base/tf/token.h"
#include "pxr/base/vt/value.h"
#include "pxr/usd/sdf/layer.h"
#include "pxr/usd/usd/attribute.h"
#include "pxr/usd/usd/editContext.h"
#include "pxr/usd/usd/prim.h"
#include "pxr/usd/usd/primRange.h"
#include "pxr/usd/usd/stage.h"

#include <algorithm>
#include <cstdio>
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

// Baked, or the checked mode under the parity entries, where every baked
// generation is compared with the dynamic walk too.
RigExecEvaluationMode
BakedMode()
{
    return TfGetenv("RIGEXEC_EVALUATION_MODE") == "parity"
               ? RigExecEvaluationMode::BakedWithParityCheck
               : RigExecEvaluationMode::Baked;
}

const char *
ModeName(RigExecEvaluationMode mode)
{
    return mode == RigExecEvaluationMode::Dynamic ? "dynamic" : "baked";
}

bool
BakeRequired()
{
    return TfGetenvBool("RIGEXEC_BAKE_REQUIRED", false);
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
Make(const UsdStageRefPtr &stage, const SdfPath &rig,
     RigExecEvaluationMode mode)
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
    evaluator->SetEvaluationMode(mode);
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

// What a fresh evaluator in \p mode publishes at \p time on \p stagePath
// with \p edits authored.
RigExecRigPose
Reference(const std::string &stagePath, const SdfPath &rig,
          RigExecEvaluationMode mode, const std::vector<Authored> &edits,
          UsdTimeCode time)
{
    const UsdStageRefPtr stage = OpenAuthored(stagePath, edits);
    if (!stage) {
        return RigExecRigPose();
    }
    auto evaluator = Make(stage, rig, mode);
    return evaluator->Evaluate(time);
}

// The pose minus what a generation BUILT (a fresh program builds every
// node), and minus the upstream drop lines, which the callers check apart.
void
KeepPosed(RigExecRigPose *pose, const RigExecRigPose &like)
{
    pose->moverGraphRevisionsCreated = like.moverGraphRevisionsCreated;
    pose->moverGraphRevisionsExecuted = like.moverGraphRevisionsExecuted;
    pose->moverGraphSchedulesBuilt = like.moverGraphSchedulesBuilt;
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
    KeepPosed(&a, a);
    KeepPosed(&b, a);
    RigExecRigPose diff;
    RigExecComparePoses(a, b, &diff);
    if (lines) {
        *lines = diff.diagnostics;
    }
    return diff.bakedParityMismatches;
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
        if (entry.step < B.steps.size() && B.steps[entry.step].isSource) {
            continue;
        }
        if (!clusters.count(entry.cluster) &&
            !B.cones.alwaysSteps.Test(int(entry.step))) {
            ++outside;
        }
    }
    for (const RigExecOpTraceEntry &entry : job.report.head) {
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
        if (entry.step < B.steps.size() && !B.steps[entry.step].isSource &&
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
    CHECK(again.report.head.size() == authoredJob.report.head.size());

    const FrozenJob lifted =
        RunJob(evaluator, standingSnapshot, time, {}, what + ": lifted");
    CheckSamePose(what + ": frozen, lifted", authored, lifted.pose);
    CHECK(lifted.inputs.upstream.empty());
    CHECK(RigExecControlStateDigest(lifted.inputs) ==
          RigExecControlStateDigest(authoredJob.inputs));
}

// One value case: in each mode, the authored pose, then the pose with
// \p inputs standing against a stage authoring \p edits, then the pose
// after the lift against the authored stage again. In baked mode the
// program answers every generation, and frozen jobs follow
// (RunFrozenLegs), unless \p freezes is false: then the rig must refuse the
// freeze.
void
RunCase(const std::string &name, const std::string &stagePath,
        const SdfPath &rig, double t,
        const std::vector<RigExecValueOverride> &inputs,
        const std::vector<Authored> &edits, bool freezes = true)
{
    std::printf("case: %s\n", name.c_str());
    const UsdTimeCode time(t);
    for (const RigExecEvaluationMode mode :
         {RigExecEvaluationMode::Dynamic, BakedMode()}) {
        const std::string what = name + " (" + ModeName(mode) + ")";
        const UsdStageRefPtr stage = UsdStage::Open(stagePath);
        CHECK(stage);
        if (!stage) {
            return;
        }
        auto evaluator = Make(stage, rig, mode);
        const RigExecRigPose authored =
            Reference(stagePath, rig, mode, {}, time);
        const RigExecRigPose before = evaluator->Evaluate(time);
        CheckSamePose(what + ": before", authored, before);
        const size_t generations = evaluator->GetBakedGenerationCount();
        const bool frozen = mode != RigExecEvaluationMode::Dynamic && freezes;
        if (mode != RigExecEvaluationMode::Dynamic && !freezes) {
            CHECK(!RigExecCanFreezeProgram(*evaluator));
        }
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
            Reference(stagePath, rig, mode, edits, time);
        CheckSamePose(what + ": standing", expected, standing);
        // The value moved something, so the case tests a move.
        CHECK(Differences(authored, standing) != 0);
        if (frozen && evaluator->GetBakedProgram()) {
            RunFrozenLegs(what, *evaluator, authoredSnapshot, authoredJob,
                          inputs, time, authored, standing,
                          evaluator->GetLastOpTrace(),
                          RigExecBakedLastHeadTrace(
                              evaluator->GetBakedProgram()->GetStepGraph()));
        }
        // A second generation with the same value standing.
        CheckSamePose(what + ": standing again", expected,
                      evaluator->Evaluate(time));

        evaluator->SetUpstreamInputs({});
        CHECK(!evaluator->HasUpstreamInputs());
        CHECK(evaluator->GetUpstreamInputPaths().empty());
        CheckSamePose(what + ": lifted", authored, evaluator->Evaluate(time));
        if (mode != RigExecEvaluationMode::Dynamic) {
            CHECK(evaluator->GetBakedGenerationCount() == generations + 3);
        }
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
    for (const RigExecEvaluationMode mode :
         {RigExecEvaluationMode::Dynamic, BakedMode()}) {
        const std::string what =
            std::string("upstream through a drag (") + ModeName(mode) + ")";
        const UsdStageRefPtr stage = UsdStage::Open(stagePath);
        auto evaluator = Make(stage, kLimbsRig, mode);
        evaluator->Evaluate(time);
        evaluator->SetUpstreamInputs({Up(kA0Rz, VtValue(30.0))});
        evaluator->Evaluate(time);
        evaluator->SetInteractiveOverrides({Up(kA1Rz, VtValue(20.0))});
        CHECK(evaluator->HasInteractiveOverrides());
        CheckSamePose(what + ": dragging",
                      Reference(stagePath, kLimbsRig, mode,
                                {{kA0Rz, VtValue(30.0), {}},
                                 {kA1Rz, VtValue(20.0), {}}},
                                time),
                      evaluator->Evaluate(time));
        evaluator->ClearInteractiveOverrides();
        const RigExecRigPose expected = Reference(
            stagePath, kLimbsRig, mode, {{kA0Rz, VtValue(30.0), {}}}, time);
        CheckSamePose(what + ": released", expected,
                      evaluator->Evaluate(time));
        CheckSamePose(what + ": released again", expected,
                      evaluator->Evaluate(time));
        // An interactive value on the same key wins while it stands.
        evaluator->SetInteractiveOverrides({Up(kA0Rz, VtValue(-10.0))});
        CheckSamePose(what + ": dragging the same key",
                      Reference(stagePath, kLimbsRig, mode,
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
    for (const RigExecEvaluationMode mode :
         {RigExecEvaluationMode::Dynamic, BakedMode()}) {
        const UsdStageRefPtr stage = UsdStage::Open(stagePath);
        auto evaluator = Make(stage, kLimbsRig, mode);
        const UsdStageRefPtr authoredStage =
            OpenAuthored(stagePath, {samples});
        auto reference = Make(authoredStage, kLimbsRig, mode);
        std::shared_ptr<const RigExecFrozenProgram> snapshot;
        std::set<uint64_t> digests;
        for (int f = 1; f <= 5; ++f) {
            const std::string what = std::string("A0 per frame (") +
                                     ModeName(mode) + ") at " +
                                     std::to_string(f);
            const std::vector<RigExecValueOverride> inputs = {
                Up(kA0Rz, VtValue(5.0 * f))};
            evaluator->SetUpstreamInputs(inputs);
            const RigExecRigPose expected = reference->Evaluate(UsdTimeCode(f));
            const RigExecRigPose live = evaluator->Evaluate(UsdTimeCode(f));
            CheckSamePose(what, expected, live);
            if (mode == RigExecEvaluationMode::Dynamic) {
                continue;
            }
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
        if (mode != RigExecEvaluationMode::Dynamic) {
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
        auto evaluator = Make(stage, lift.rig, BakedMode());
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
                      Reference(lift.stagePath, lift.rig, BakedMode(),
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
                      Reference(lift.stagePath, lift.rig, BakedMode(), {},
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
void
TestABurstCarriesUpstream()
{
    std::printf("case: a burst carries upstream values\n");
    struct Burst {
        std::string name;
        std::string stagePath;
        SdfPath rig;
        RigExecValueOverride input;
        double first;
    };
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
        const std::string what = "burst " + burst.name;
        const UsdStageRefPtr stage = UsdStage::Open(burst.stagePath);
        auto evaluator = Make(stage, burst.rig, BakedMode());
        const UsdTimeCode first(burst.first);
        evaluator->Evaluate(first);
        evaluator->SetUpstreamInputs({burst.input});
        evaluator->Evaluate(first);
        const RigExecBakedProgram *program = evaluator->GetBakedProgram();
        CHECK(program);
        const std::shared_ptr<const RigExecFrozenProgram> snapshot =
            Freeze(*evaluator, what);
        if (!program || !snapshot) {
            continue;
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
            continue;
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
                          Reference(burst.stagePath, burst.rig, BakedMode(),
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
    auto evaluator = Make(stage, kLimbsRig, BakedMode());
    const UsdTimeCode time(1);
    evaluator->Evaluate(time);
    const std::shared_ptr<const RigExecFrozenProgram> snapshot =
        Freeze(*evaluator, "admission");
    const RigExecRigPose expected = Reference(
        stagePath, kLimbsRig, BakedMode(), {{kA0Rz, VtValue(30.0), {}}}, time);
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
    const UsdStageRefPtr stage =
        UsdStage::Open(Fixture("upstream_inputs.usda"));
    auto evaluator = Make(stage, kLimbsRig, BakedMode());
    const RigExecBakedProgram *program = evaluator->GetBakedProgram();
    CHECK(program);
    if (!program) {
        return;
    }
    // The fixture keeps no always-dirty step (no Derived extent).
    const RigExecBakedClusterSet &always =
        program->GetStepGraph().cones.alwaysSteps;
    CHECK(std::all_of(always.words.begin(), always.words.end(),
                      [](uint64_t w) { return w == 0; }));
    const UsdTimeCode time(1);
    evaluator->Evaluate(time);
    evaluator->SetUpstreamInputs({Up(kA0Rz, VtValue(30.0)),
                                  Up(kUpstreamSpace,
                                     VtValue(Translate(1, 0, 10)))});
    evaluator->Evaluate(time);
    CHECK(evaluator->GetBakedClustersRunLastGeneration() > 0);
    evaluator->SetUpstreamInputs({Up(kA0Rz, VtValue(30.0)),
                                  Up(kUpstreamSpace,
                                     VtValue(Translate(1, 0, 10)))});
    evaluator->Evaluate(time);
    CHECK(evaluator->GetBakedClustersRunLastGeneration() == 0);
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
            Make(stage, SdfPath("/Asset/Rig"), RigExecEvaluationMode::Baked);
        const RigExecBakedProgram *program = evaluator->GetBakedProgram();
        CHECK(program && program->GetUpstreamOracle().count(scale));
    }
    // At frame 5, where the driver has left 0 and the scale matters.
    RunCase("oracle input inputs:scale", stagePath, SdfPath("/Asset/Rig"), 5,
            {Up(scale, VtValue(0.25f))}, {{scale, VtValue(0.25f), {}}},
            /*freezes=*/false);
    // No frozen job computes an oracle-resolved object: the freeze refuses
    // the rig, or a job declines, as with no value standing.
    {
        const UsdStageRefPtr stage = UsdStage::Open(stagePath);
        auto evaluator = Make(stage, SdfPath("/Asset/Rig"), BakedMode());
        evaluator->SetUpstreamInputs({Up(scale, VtValue(0.25f))});
        const RigExecRigPose standing = evaluator->Evaluate(UsdTimeCode(5));
        std::shared_ptr<const RigExecFrozenProgram> frozen;
        std::string error;
        if (RigExecFreezeProgram(*evaluator, &frozen, &error)) {
            FrozenJob job;
            CHECK(RigExecSampleFrameInputs(
                *evaluator, UsdTimeCode(5), {},
                RigExecUpstreamValuesOf({Up(scale, VtValue(0.25f))}),
                &job.inputs, &error));
            RigExecFrozenEvalContext context;
            context.slotCount = evaluator->GetBakedProgram()->GetProviderCount();
            context.varyingInputCount = job.inputs.values.size();
            context.frozen = frozen.get();
            job.pose = RigExecEvaluateFrozen(context, job.inputs,
                                             RigExecMakeProductionStepRunner(),
                                             nullptr, SdfPath(), &job.report);
            std::printf("  oracle: froze; the job %s\n",
                        job.pose.valid ? "ran" : "declined");
            if (job.pose.valid) {
                CheckSamePose("oracle input (frozen)", standing, job.pose);
            }
        } else {
            std::printf("  oracle: freeze refused: %s\n", error.c_str());
            CHECK(error.find("upstream") == std::string::npos);
        }
    }
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
        for (const RigExecEvaluationMode mode :
             {RigExecEvaluationMode::Dynamic, BakedMode()}) {
            const UsdStageRefPtr stage = UsdStage::Open(drop.stagePath);
            auto evaluator = Make(stage, drop.rig, mode);
            evaluator->SetUpstreamInputs({drop.input});
            const RigExecRigPose pose = evaluator->Evaluate(UsdTimeCode(1));
            CHECK(evaluator->GetUpstreamInputPaths().empty());
            const std::string line = "upstream input " + path.GetString() +
                                     ": " + drop.reason + "; ignored";
            if (!HasLine(pose, line)) {
                std::printf("FAIL: no line '%s' (%s)\n", line.c_str(),
                            ModeName(mode));
                for (const std::string &l : pose.diagnostics) {
                    std::printf("    %s\n", l.c_str());
                }
            }
            CHECK(HasLine(pose, line));
            CheckSamePose(path.GetString() + " dropped (" + ModeName(mode) +
                              ")",
                          Reference(drop.stagePath, drop.rig, mode, {},
                                    UsdTimeCode(1)),
                          pose);
        }
    }
    // A key no listed read reaches drops against the program; with no
    // program (dynamic mode) the overlay it would ride is read by nothing.
    const SdfPath scheme("/LimbsAsset/Geom/MeshA.subdivisionScheme");
    const UsdStageRefPtr stage =
        UsdStage::Open(Fixture("upstream_inputs.usda"));
    auto evaluator = Make(stage, kLimbsRig, BakedMode());
    evaluator->SetUpstreamInputs(
        {Up(scheme, VtValue(TfToken("catmullClark")))});
    const RigExecRigPose pose = evaluator->Evaluate(UsdTimeCode(1));
    CHECK(HasLine(pose, "upstream input " + scheme.GetString() +
                            ": no listed read reaches it; ignored"));
    CHECK(evaluator->GetUpstreamInputPaths().empty());
}

// The posed variant: the dynamic walk follows; the program refuses the
// connected posed space, and so does the freeze.
void
TestThePosedVariant()
{
    std::printf("case: posed variant\n");
    const std::string stagePath = Fixture("upstream_inputs_posed.usda");
    const GfMatrix4d moved = Translate(1, 0, 10);
    const std::vector<RigExecValueOverride> inputs = {
        Up(kA0Rz, VtValue(30.0)), Up(kUpstreamSpace, VtValue(moved))};
    const std::vector<Authored> edits = {{kA0Rz, VtValue(30.0), {}},
                                         {kUpstreamSpace, VtValue(moved), {}}};
    {
        const UsdStageRefPtr stage = UsdStage::Open(stagePath);
        auto evaluator =
            Make(stage, kLimbsRig, RigExecEvaluationMode::Dynamic);
        evaluator->SetUpstreamInputs(inputs);
        const RigExecRigPose pose = evaluator->Evaluate(UsdTimeCode(1));
        CHECK(evaluator->GetUpstreamInputPaths() == PathsOf(inputs));
        CheckSamePose("posed variant (dynamic)",
                      Reference(stagePath, kLimbsRig,
                                RigExecEvaluationMode::Dynamic, edits,
                                UsdTimeCode(1)),
                      pose);
        CHECK(Differences(Reference(stagePath, kLimbsRig,
                                    RigExecEvaluationMode::Dynamic, {},
                                    UsdTimeCode(1)),
                          pose) != 0);
    }
    {
        const UsdStageRefPtr stage = UsdStage::Open(stagePath);
        auto evaluator = Make(stage, kLimbsRig, RigExecEvaluationMode::Baked);
        evaluator->SetUpstreamInputs(inputs);
        CHECK(evaluator->GetBakedProgram() == nullptr);
        std::vector<std::string> reasons;
        CHECK(!evaluator->IsBakeable(&reasons));
        CHECK(HasReason(reasons, "connected posed:space on provider"));
        std::shared_ptr<const RigExecFrozenProgram> frozen;
        std::string error;
        CHECK(!RigExecFreezeProgram(*evaluator, &frozen, &error));
        // The generation falls back to the walk, which follows. Under
        // RIGEXEC_BAKE_REQUIRED that fallback is the failure it reports.
        if (!BakeRequired()) {
            CheckSamePose("posed variant (baked mode, walked)",
                          Reference(stagePath, kLimbsRig,
                                    RigExecEvaluationMode::Dynamic, edits,
                                    UsdTimeCode(1)),
                          evaluator->Evaluate(UsdTimeCode(1)));
            CHECK(evaluator->GetBakedGenerationCount() == 0);
        }
    }
}

// The interim bake guard: a bake is refused while values stand, not while
// they are suspended, and the program itself is never refused for them.
// A freeze is not: frozen jobs carry upstream values.
void
TestTheBakeGuard()
{
    std::printf("case: bake guard\n");
    const std::string reason =
        "upstream inputs standing (the exporter does not expose them yet)";
    const UsdStageRefPtr stage =
        UsdStage::Open(Fixture("upstream_inputs.usda"));
    auto evaluator = Make(stage, kLimbsRig, BakedMode());
    std::vector<std::string> reasons;
    CHECK(evaluator->IsBakeable(&reasons));
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
    CHECK(!evaluator->IsBakeable(&reasons));
    CHECK(HasReason(reasons, reason));
    CHECK(reasons.size() == 1);
    {
        RigExecScopedUpstreamSuspension suspension(*evaluator);
        CHECK(!evaluator->HasUpstreamInputs());
        reasons.clear();
        CHECK(evaluator->IsBakeable(&reasons));
        CHECK(!HasReason(reasons, reason));
        error.clear();
        CHECK(RigExecCanFreezeProgram(*evaluator, &error));
        CheckSamePose("suspended",
                      Reference(Fixture("upstream_inputs.usda"), kLimbsRig,
                                BakedMode(), {}, UsdTimeCode(1)),
                      evaluator->Evaluate(UsdTimeCode(1)));
        CHECK(RigExecCanFreezeProgram(*evaluator, &error));
    }
    CHECK(evaluator->GetUpstreamInputs() == inputs);
    reasons.clear();
    CHECK(!evaluator->IsBakeable(&reasons));
    CHECK(HasReason(reasons, reason));
    CHECK(RigExecCanFreezeProgram(*evaluator, &error));
    CheckSamePose("restored", standing, evaluator->Evaluate(UsdTimeCode(1)));
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
        Make(stage, rig, RigExecEvaluationMode::Baked);
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
        // The array part is empty on both sides until the array input kind
        // lands: every admissible type is a scalar slot's.
        CHECK(slotTypes.count(type));
        if (Settable(stage, path)) {
            admissible[path] = type;
        }
    }
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
        // Structural: the same answer in every mode, so once, unverified.
        if (TfGetenv("RIGEXEC_EVALUATION_MODE").empty() && !ConeVerify()) {
            TestEveryAdmissiblePathIsAnInput();
        }
    } catch (const std::exception &error) {
        std::printf("FAIL: threw: %s\n", error.what());
        return 1;
    }
    if (failures) {
        std::printf("%d FAILURE(S)\n", failures);
        return 1;
    }
    std::printf("all tests passed\n");
    return 0;
}
