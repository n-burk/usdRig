//
// Stage VALUE edits routed to the per-frame inputs they reach (unified-program
// spec rules S2, S3): what the program re-runs after one, and that what it
// then publishes is what a program built fresh on the edited stage publishes.
//
// A value edit the capture index misses used to bump the program stamp, and
// the next generation ran everything. Now an edit on an input a step reads
// every frame marks that input edited and the next run re-runs its cone, the
// way the run after a drag is lifted does; an edit on something a
// value-compared source reads (a property chain's input, a stage transform)
// is left to the source's own comparison; and an edit on something nothing
// reads does nothing. Each case below edits once, evaluates, and compares the
// pose with a second evaluator compiled on the edited stage, which ran the
// whole program on its first generation and so cannot have skipped anything.
//
// Registered plain (baked) and under the parity entries, where every
// generation is also compared with the dynamic walk and, with
// RIGEXEC_BAKED_VERIFY_CONES, the cone run with a forced run of everything:
// the edited-vs-forced check this routing has to pass.
//
// argv[1] = path to the examples directory.
//
#include "rigExec/bakedProgram.h"
#include "rigExec/bakedProgramImpl.h"
#include "rigExec/rigEvaluator.h"

#include "pxr/base/plug/registry.h"
#include "pxr/base/tf/getenv.h"
#include "pxr/base/tf/notice.h"
#include "pxr/base/tf/weakBase.h"
#include "pxr/base/tf/pathUtils.h"
#include "pxr/base/gf/vec3f.h"
#include "pxr/base/vt/array.h"
#include "pxr/usd/sdf/types.h"
#include "pxr/usd/usd/attribute.h"
#include "pxr/usd/usd/notice.h"
#include "pxr/usd/usd/prim.h"
#include "pxr/usd/usd/stage.h"

#include <algorithm>
#include <cstdio>
#include <functional>
#include <string>
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

std::string
SchemaResourceDir(const std::string &examplesDir)
{
#ifdef RIGEXEC_SCHEMA_RESOURCE_DIR
    (void)examplesDir;
    return TfAbsPath(RIGEXEC_SCHEMA_RESOURCE_DIR);
#else
    return TfAbsPath(examplesDir + "/../plugin/rigExecSchema/resources");
#endif
}

// Baked, unless the parity entries asked for the checked mode: under it every
// generation is compared with the dynamic walk as well.
RigExecEvaluationMode
Mode()
{
    return TfGetenv("RIGEXEC_EVALUATION_MODE") == "parity"
               ? RigExecEvaluationMode::BakedWithParityCheck
               : RigExecEvaluationMode::Baked;
}

bool
Parity()
{
    return Mode() == RigExecEvaluationMode::BakedWithParityCheck;
}

const char *
DispositionName(RigExecNoticeDisposition d)
{
    switch (d) {
    case RigExecNoticeDisposition::None: return "None";
    case RigExecNoticeDisposition::Patched: return "Patched";
    case RigExecNoticeDisposition::StampBumped: return "StampBumped";
    case RigExecNoticeDisposition::Edited: return "Edited";
    case RigExecNoticeDisposition::Stale: return "Stale";
    }
    return "?";
}

// The pose a program built fresh on \p stage publishes at \p time: its first
// generation runs everything, so it is the answer the edited program's cone
// has to reproduce.
RigExecRigPose
FreshPose(const UsdStageRefPtr &stage, const SdfPath &rig, UsdTimeCode time)
{
    RigExecRigEvaluator fresh(stage, rig);
    std::vector<std::string> errors;
    CHECK(fresh.Compile(&errors));
    fresh.SetEvaluationMode(Mode());
    const RigExecRigPose pose = fresh.Evaluate(time);
    CHECK(fresh.GetBakedGenerationCount() == 1);
    return pose;
}

// Equal to \p reference in everything a pose publishes. The mover-graph
// counters are left out: they count what a generation BUILT, and a fresh
// program's first generation builds every node the edited one kept.
void
CheckSamePose(const std::string &what, const RigExecRigPose &reference,
              RigExecRigPose pose)
{
    CHECK(pose.valid);
    CHECK(reference.valid);
    RigExecRigPose fresh = reference;
    pose.moverGraphRevisionsCreated = fresh.moverGraphRevisionsCreated;
    pose.moverGraphRevisionsExecuted = fresh.moverGraphRevisionsExecuted;
    pose.moverGraphSchedulesBuilt = fresh.moverGraphSchedulesBuilt;
    // And the lines that report them, with the one a recompile adds: what a
    // generation built, not what it posed.
    const auto keepPosed = [](std::vector<std::string> *lines) {
        std::vector<std::string> kept;
        for (std::string &line : *lines) {
            if (line.rfind("mover graph:", 0) != 0 &&
                line.rfind("structural edit:", 0) != 0) {
                kept.push_back(std::move(line));
            }
        }
        *lines = std::move(kept);
    };
    keepPosed(&fresh.diagnostics);
    keepPosed(&pose.diagnostics);
    RigExecRigPose diff;
    RigExecComparePoses(fresh, pose, &diff);
    if (diff.bakedParityMismatches != 0) {
        std::printf("FAIL %s: %zu mismatch(es) against a fresh program:\n",
                    what.c_str(), diff.bakedParityMismatches);
        for (size_t i = 0; i < diff.diagnostics.size() && i < 12; ++i) {
            std::printf("    %s\n", diff.diagnostics[i].c_str());
        }
        for (const std::string &line : fresh.diagnostics) {
            std::printf("    fresh:  %s\n", line.c_str());
        }
        for (const std::string &line : pose.diagnostics) {
            std::printf("    edited: %s\n", line.c_str());
        }
    }
    CHECK(diff.bakedParityMismatches == 0);
}

struct Case {
    std::string name;
    std::string stagePath;
    SdfPath rig;
    double time = 0.0;
    /// Authored before the evaluator compiles, so the edit below is a
    /// changed-info notice on an existing spec rather than a spec's first
    /// appearance (which is a resync, and still bumps the stamp).
    std::function<void(const UsdStageRefPtr &)> setup;
    std::function<void(const UsdStageRefPtr &)> edit;
    RigExecNoticeDisposition expect = RigExecNoticeDisposition::Edited;
    /// Whether the generation after the edit must run less than the whole
    /// program: the point of routing it.
    bool expectCone = true;
    /// Whether the disposition names no path at all (read by nothing).
    bool expectNoPaths = false;
    /// Whether the generation after the edit must run the whole program:
    /// the stamp bump an unroutable edit falls back to.
    bool expectWhole = false;
    /// A property path the disposition must name (read by something).
    SdfPath expectPath;
};

void
RunCase(const Case &c)
{
    UsdStageRefPtr stage = UsdStage::Open(c.stagePath);
    CHECK(stage);
    if (!stage) {
        return;
    }
    if (c.setup) {
        c.setup(stage);
    }
    RigExecRigEvaluator evaluator(stage, c.rig);
    std::vector<std::string> errors;
    CHECK(evaluator.Compile(&errors));
    evaluator.SetEvaluationMode(Mode());
    const UsdTimeCode time(c.time);
    CHECK(evaluator.Evaluate(time).valid);
    CHECK(evaluator.Evaluate(time).valid);
    const size_t generations = evaluator.GetBakedGenerationCount();
    const size_t builds = evaluator.GetBakedProgramBuildCount();
    CHECK(generations == 2);

    c.edit(stage);
    const RigExecNoticeDisposition got = evaluator.GetLastNoticeDisposition();
    if (got != c.expect) {
        std::printf("FAIL %s: disposition %s, expected %s\n", c.name.c_str(),
                    DispositionName(got), DispositionName(c.expect));
    }
    CHECK(got == c.expect);
    if (c.expectNoPaths) {
        CHECK(evaluator.GetLastNoticePatchedPaths().empty());
    }
    if (!c.expectPath.IsEmpty()) {
        const std::vector<SdfPath> &paths =
            evaluator.GetLastNoticePatchedPaths();
        const bool named = std::find(paths.begin(), paths.end(),
                                     c.expectPath) != paths.end();
        if (!named) {
            std::printf("FAIL %s: %s is not among the %zu read path(s)\n",
                        c.name.c_str(), c.expectPath.GetText(),
                        paths.size());
        }
        CHECK(named);
    }

    const RigExecRigPose edited = evaluator.Evaluate(time);
    CHECK(edited.bakedParityMismatches == 0);
    CHECK(evaluator.GetBakedGenerationCount() == generations + 1);
    if (c.expect != RigExecNoticeDisposition::Stale) {
        CHECK(evaluator.GetBakedProgramBuildCount() == builds);
    }
    // Steps rather than clusters: a small rig packs into one cluster, and a
    // cluster runs only the closed steps it holds.
    const RigExecBakedProgramImpl &B =
        evaluator.GetBakedProgram()->GetStepGraph();
    const size_t ran = B.lastClosedSteps;
    const size_t total = B.steps.size();
    std::printf("  %s: %s, %zu of %zu step(s) ran\n", c.name.c_str(),
                DispositionName(got), ran, total);
    if (c.expectCone) {
        CHECK(ran < total);
    }
    if (c.expectWhole) {
        CHECK(evaluator.GetBakedClustersRunLastGeneration() ==
              evaluator.GetBakedClusterCount());
    }
    CheckSamePose(c.name + " (edited frame)", FreshPose(stage, c.rig, time),
                  edited);

    // And the frames after it: the edit was consumed once, and the program
    // goes on agreeing with a fresh one at the held time and at a new one.
    const RigExecRigPose held = evaluator.Evaluate(time);
    CHECK(held.bakedParityMismatches == 0);
    CheckSamePose(c.name + " (held frame)", FreshPose(stage, c.rig, time),
                  held);
    const UsdTimeCode next(c.time + 1.0);
    const RigExecRigPose moved = evaluator.Evaluate(next);
    CHECK(moved.bakedParityMismatches == 0);
    CheckSamePose(c.name + " (next frame)", FreshPose(stage, c.rig, next),
                  moved);
}

// ---------------------------------------------------------------------------
// The edit-latency bench's tiers, on the biped.
// ---------------------------------------------------------------------------

const SdfPath kBiped("/Biped/Rig");
const SdfPath kHips("/Biped/Rig/Controls/hips_ctl");
const SdfPath kMover(
    "/Biped/Rig/Movers/ballRoll_l_down_w/ballRoll_l_down_w_4_multiply");
const SdfPath kCornea("/Biped/Materials/cornea_mat");
constexpr double kHeld = 10.0;

std::string
Biped(const std::string &examplesDir)
{
    return examplesDir + "/biped/Biped_anim.usda";
}

UsdAttribute
Attr(const UsdStageRefPtr &stage, const SdfPath &prim, const char *name)
{
    return stage->GetPrimAtPath(prim).GetAttribute(TfToken(name));
}

// A time sample on a keyed avar at the held time: a varying avar binding,
// which the prologue re-reads and compares by value.
void
TestAKeyedAvarSampleRunsItsCone(const std::string &examplesDir)
{
    Case c;
    c.name = "keyed avar sample";
    c.stagePath = Biped(examplesDir);
    c.rig = kBiped;
    c.time = kHeld;
    c.edit = [](const UsdStageRefPtr &stage) {
        const UsdAttribute a = Attr(stage, kHips, "avars:rz");
        double v = 0.0;
        a.Get(&v, UsdTimeCode(kHeld));
        CHECK(a.Set(v + 7.5, UsdTimeCode(kHeld)));
    };
    RunCase(c);
}

// A guide value: read by nothing in the program, so nothing re-runs but the
// sources.
void
TestAGuideValueRunsNothing(const std::string &examplesDir)
{
    Case c;
    c.name = "guide value";
    c.stagePath = Biped(examplesDir);
    c.rig = kBiped;
    c.time = kHeld;
    c.setup = [](const UsdStageRefPtr &stage) {
        const UsdAttribute a = Attr(stage, kHips, "guide:scaleX");
        double v = 1.0;
        a.Get(&v);
        CHECK(a.Set(v));
    };
    c.edit = [](const UsdStageRefPtr &stage) {
        const UsdAttribute a = Attr(stage, kHips, "guide:scaleX");
        double v = 1.0;
        a.Get(&v);
        CHECK(a.Set(v + 0.25));
    };
    RunCase(c);
}

// A property-chain mover's own input: the chain is re-evaluated every run
// and its result compared by value, so the edit is the chain's to find.
void
TestAChainMoverInputReachesThroughTheChain(const std::string &examplesDir)
{
    Case c;
    c.name = "chain mover input";
    c.stagePath = Biped(examplesDir);
    c.rig = kBiped;
    c.time = kHeld;
    // The value lives in a layer below the root, so the root layer is given
    // its own spec first: the edit is then a value change, not the spec's
    // arrival (a resync, which bumps the stamp).
    c.setup = [](const UsdStageRefPtr &stage) {
        const UsdAttribute a = Attr(stage, kMover, "inputs:defaultWeight");
        CHECK(a);
        float v = 1.0f;
        a.Get(&v);
        CHECK(a.Set(v));
    };
    c.edit = [](const UsdStageRefPtr &stage) {
        const UsdAttribute a = Attr(stage, kMover, "inputs:defaultWeight");
        CHECK(a);
        float v = 1.0f;
        a.Get(&v);
        CHECK(a.Set(v >= 0.5f ? v - 0.25f : v + 0.25f));
    };
    RunCase(c);
}

// A value on a prim outside the rig that nothing reads: routed to nothing,
// and naming no path a frame cache would have to retire for.
void
TestAnUnreadValueOutsideTheRigRunsNothing(const std::string &examplesDir)
{
    Case c;
    c.name = "unread value outside the rig";
    c.stagePath = Biped(examplesDir);
    c.rig = kBiped;
    c.time = kHeld;
    c.setup = [](const UsdStageRefPtr &stage) {
        CHECK(stage->GetPrimAtPath(kCornea)
                  .CreateAttribute(TfToken("foo"), SdfValueTypeNames->Float)
                  .Set(0.0f));
    };
    c.edit = [](const UsdStageRefPtr &stage) {
        CHECK(Attr(stage, kCornea, "foo").Set(1.0f));
    };
    c.expectNoPaths = true;
    RunCase(c);
}

// ---------------------------------------------------------------------------
// A constraint's per-frame input, on a small rig.
// ---------------------------------------------------------------------------

const SdfPath kAimRig("/World/RigRoot");
const SdfPath kAim("/World/RigRoot/Movers/RigExecAimConstraint1");

std::string
AimStage(const std::string &examplesDir)
{
    return examplesDir + "/aimtest.usda";
}

// inputs:defaultWeight keyed across the shot, so the bake binds it as a
// per-frame query that only the constraint step reads.
void
KeyAimWeight(const UsdStageRefPtr &stage)
{
    const UsdAttribute w =
        stage->GetPrimAtPath(kAim).CreateAttribute(
            TfToken("inputs:defaultWeight"), SdfValueTypeNames->Float);
    CHECK(w.Set(1.0f, UsdTimeCode(1.0)));
    CHECK(w.Set(0.5f, UsdTimeCode(100.0)));
}

void
TestAConstraintInputSampleRunsItsCone(const std::string &examplesDir)
{
    Case c;
    c.name = "constraint input sample";
    c.stagePath = AimStage(examplesDir);
    c.rig = kAimRig;
    c.time = 50.0;
    c.setup = KeyAimWeight;
    c.edit = [](const UsdStageRefPtr &stage) {
        CHECK(Attr(stage, kAim, "inputs:defaultWeight")
                  .Set(0.25f, UsdTimeCode(100.0)));
    };
    RunCase(c);
    // The same edit is the step's alone: nothing else of the program is
    // told, so the steps it re-ran are the constraint's cone.
    UsdStageRefPtr stage = UsdStage::Open(AimStage(examplesDir));
    KeyAimWeight(stage);
    RigExecRigEvaluator evaluator(stage, kAimRig);
    CHECK(evaluator.Compile());
    evaluator.SetEvaluationMode(Mode());
    CHECK(evaluator.Evaluate(UsdTimeCode(50.0)).valid);
    CHECK(evaluator.Evaluate(UsdTimeCode(50.0)).valid);
    Attr(stage, kAim, "inputs:defaultWeight").Set(0.75f, UsdTimeCode(100.0));
    const std::vector<SdfPath> &paths = evaluator.GetLastNoticePatchedPaths();
    CHECK(paths.size() == 1 &&
          paths[0] == kAim.AppendProperty(TfToken("inputs:defaultWeight")));
    const RigExecBakedProgramImpl &B =
        evaluator.GetBakedProgram()->GetStepGraph();
    CHECK(B.anyEdited);
}

// Asks the standing program, from inside each notice, whether the notice
// invalidates it: the program's own answer, before a settle that could reach
// the same rebuild another way (the structure digest).
struct _InvalidationProbe : public TfWeakBase {
    const RigExecRigEvaluator *evaluator = nullptr;
    bool invalidated = false;
    void Handle(const UsdNotice::ObjectsChanged &notice,
                const UsdStageWeakPtr &)
    {
        const RigExecBakedProgram *program = evaluator->GetBakedProgram();
        invalidated = program && program->IsInvalidatedBy(notice);
    }
};

// A connection retargeted on an input the bake named rebuilds the program:
// the walk it recorded -- and with it which upstream hops a later value edit
// is routed through -- is not the walk the frame takes any more. The value
// edit on the new upstream hop that follows then reaches the constraint,
// where a program that had kept the old index would have routed it nowhere.
void
TestARetargetedWalkRebuildsBeforeItsUpstreamEdit(
    const std::string &examplesDir)
{
    const SdfPath drivers = kAimRig.AppendChild(TfToken("Drivers"));
    const SdfPath driverA = drivers.AppendProperty(TfToken("drv:a"));
    const SdfPath driverB = drivers.AppendProperty(TfToken("drv:b"));
    const auto setup = [drivers](const UsdStageRefPtr &stage) {
        const UsdPrim prim =
            stage->DefinePrim(drivers, TfToken("Scope"));
        const UsdAttribute a =
            prim.CreateAttribute(TfToken("drv:a"), SdfValueTypeNames->Float);
        CHECK(a.Set(0.9f, UsdTimeCode(1.0)));
        CHECK(a.Set(0.3f, UsdTimeCode(100.0)));
        const UsdAttribute b =
            prim.CreateAttribute(TfToken("drv:b"), SdfValueTypeNames->Float);
        CHECK(b.Set(0.8f, UsdTimeCode(1.0)));
        CHECK(b.Set(0.2f, UsdTimeCode(100.0)));
        // A clamp on each: both are property-chain targets, so a walk that
        // reaches either resolves through the chain every frame (a
        // `resolvedAttr` walk) and its hops are the input's override paths.
        const SdfPath movers = kAimRig.AppendChild(TfToken("Movers"));
        for (const char *name : {"drv:a", "drv:b"}) {
            const UsdPrim clamp = stage->DefinePrim(
                movers.AppendChild(TfToken(std::string("Clamp_") +
                                           (name[4] == 'a' ? "A" : "B"))),
                TfToken("RigExecFloatMathMover"));
            clamp.ApplyAPI(TfToken("RigExecMoverAPI"));
            clamp.CreateAttribute(TfToken("rigExec:operation"),
                                  SdfValueTypeNames->Token)
                .Set(TfToken("clamp"));
            clamp.CreateAttribute(TfToken("inputs:min"),
                                  SdfValueTypeNames->Float).Set(0.0f);
            clamp.CreateAttribute(TfToken("inputs:max"),
                                  SdfValueTypeNames->Float).Set(1.0f);
            clamp.CreateRelationship(TfToken("rigExec:moves"))
                .SetTargets({drivers.AppendProperty(TfToken(name))});
        }
        const UsdAttribute w =
            stage->GetPrimAtPath(kAim).CreateAttribute(
                TfToken("inputs:defaultWeight"), SdfValueTypeNames->Float);
        CHECK(w.SetConnections({drivers.AppendProperty(TfToken("drv:a"))}));
    };
    UsdStageRefPtr stage = UsdStage::Open(AimStage(examplesDir));
    setup(stage);
    RigExecRigEvaluator evaluator(stage, kAimRig);
    CHECK(evaluator.Compile());
    evaluator.SetEvaluationMode(Mode());
    const UsdTimeCode time(50.0);
    CHECK(evaluator.Evaluate(time).valid);
    CHECK(evaluator.Evaluate(time).valid);
    const size_t builds = evaluator.GetBakedProgramBuildCount();

    _InvalidationProbe probe;
    probe.evaluator = &evaluator;
    TfNotice::Key key = TfNotice::Register(TfCreateWeakPtr(&probe),
                                           &_InvalidationProbe::Handle,
                                           UsdStageWeakPtr(stage));
    CHECK(Attr(stage, kAim, "inputs:defaultWeight").SetConnections({driverB}));
    TfNotice::Revoke(key);
    // The walk resolved through a chain every frame, so nothing about its
    // value was captured -- and still the retarget is the program's own
    // reason to rebuild.
    CHECK(probe.invalidated);
    CHECK(evaluator.GetLastNoticeDisposition() ==
          RigExecNoticeDisposition::Stale);
    const RigExecRigPose retargeted = evaluator.Evaluate(time);
    CHECK(retargeted.bakedParityMismatches == 0);
    CHECK(evaluator.GetBakedProgramBuildCount() == builds + 1);
    CheckSamePose("retargeted walk", FreshPose(stage, kAimRig, time),
                  retargeted);
    CHECK(evaluator.Evaluate(time).valid);

    // The new upstream hop's value, at the held time.
    CHECK(stage->GetAttributeAtPath(driverB).Set(0.05f, UsdTimeCode(100.0)));
    const RigExecNoticeDisposition got = evaluator.GetLastNoticeDisposition();
    std::printf("  upstream edit after the retarget: %s\n",
                DispositionName(got));
    CHECK(got == RigExecNoticeDisposition::Edited);
    const RigExecRigPose edited = evaluator.Evaluate(time);
    CHECK(edited.bakedParityMismatches == 0);
    CheckSamePose("upstream edit after the retarget",
                  FreshPose(stage, kAimRig, time), edited);
    // And the old hop is read by nothing now: its edit moves no pose.
    CHECK(stage->GetAttributeAtPath(driverA).Set(0.6f, UsdTimeCode(100.0)));
    const RigExecRigPose stale = evaluator.Evaluate(time);
    CHECK(stale.bakedParityMismatches == 0);
    CheckSamePose("old hop after the retarget",
                  FreshPose(stage, kAimRig, time), stale);
}

// Layer metadata is read by every time conversion and indexed nowhere. The
// `/` path is in the program's rebuild set, so IsInvalidatedBy answers it
// (Stale) before the value-edit routing is asked, and the next generation
// is a new program's first, which runs whole.
void
TestLayerMetadataRebuilds(const std::string &examplesDir)
{
    UsdStageRefPtr stage = UsdStage::Open(AimStage(examplesDir));
    RigExecRigEvaluator evaluator(stage, kAimRig);
    CHECK(evaluator.Compile());
    evaluator.SetEvaluationMode(Mode());
    CHECK(evaluator.Evaluate(UsdTimeCode(50.0)).valid);
    stage->SetStartTimeCode(stage->GetStartTimeCode() - 1.0);
    std::printf("  layer metadata: %s\n",
                DispositionName(evaluator.GetLastNoticeDisposition()));
    CHECK(evaluator.GetLastNoticeDisposition() ==
          RigExecNoticeDisposition::Stale);
    const RigExecRigPose pose = evaluator.Evaluate(UsdTimeCode(50.0));
    CHECK(pose.valid);
    CHECK(pose.bakedParityMismatches == 0);
    CHECK(evaluator.GetBakedClustersRunLastGeneration() ==
          evaluator.GetBakedClusterCount());
}


// A property the bake asked about by name that no step declares as an input
// -- a blend shape's target points, read whole by the geometry prologue's
// shape assembly -- has a reader the routing cannot name, so the edit falls
// back to the stamp: the next generation runs everything, and agrees with a
// fresh program.
void
TestANamedInputNoStepDeclaresBumpsTheStamp(const std::string &examplesDir)
{
    const SdfPath smile("/FaceAsset/Targets/SmileHalf");
    Case c;
    c.name = "named target points";
    c.stagePath = examplesDir + "/04_BlendShapeFace.usda";
    c.rig = SdfPath("/FaceAsset/Rig");
    c.time = 1024.0;
    c.edit = [smile](const UsdStageRefPtr &stage) {
        const UsdAttribute a = Attr(stage, smile, "points");
        VtVec3fArray points;
        CHECK(a.Get(&points));
        for (GfVec3f &p : points) {
            p[2] += 0.25f;
        }
        CHECK(a.Set(points));
    };
    c.expect = RigExecNoticeDisposition::StampBumped;
    c.expectCone = false;
    c.expectWhole = true;
    RunCase(c);
}

const SdfPath kPropMathRig("/PropMathAsset/Rig");
const SdfPath kClampGain("/PropMathAsset/Rig/Movers/ClampGain");
const SdfPath kSettings("/PropMathAsset/Rig/Settings");

// A settings prim nothing in the rig reads by name, with a clamp's maximum
// connected to one of its two values.
void
ConnectClampToSettings(const UsdStageRefPtr &stage)
{
    const UsdPrim settings = stage->DefinePrim(kSettings, TfToken("Scope"));
    const UsdAttribute gain = settings.CreateAttribute(
        TfToken("maxGain"), SdfValueTypeNames->Float);
    CHECK(gain.Set(1.0f));
    CHECK(settings.CreateAttribute(TfToken("otherGain"),
                                   SdfValueTypeNames->Float)
              .Set(1.0f));
    CHECK(Attr(stage, kClampGain, "inputs:max")
              .SetConnections({gain.GetPath()}));
}

// A property-chain mover's input connected to a property on a prim the bake
// never recorded. The chain reads it through the resolved inputs every run
// and compares its result, so the program owes the edit nothing -- but it IS
// read, and the disposition names it for the frame cache, which would
// otherwise keep frames the edit moved. Its unconnected sibling is read by
// nothing. A retarget onto the sibling is a connection edit, which the
// routing refuses, and the sibling is the one named after it.
void
TestAConnectedChainSourceIsReported(const std::string &examplesDir)
{
    const SdfPath maxGain = kSettings.AppendProperty(TfToken("maxGain"));
    const SdfPath otherGain = kSettings.AppendProperty(TfToken("otherGain"));
    Case c;
    c.name = "connected chain source";
    c.stagePath = examplesDir + "/09_PropertyMathMovers.usda";
    c.rig = kPropMathRig;
    c.time = 1001.0;
    c.setup = ConnectClampToSettings;
    c.edit = [maxGain](const UsdStageRefPtr &stage) {
        CHECK(stage->GetAttributeAtPath(maxGain).Set(0.25f));
    };
    c.expectPath = maxGain;
    RunCase(c);

    UsdStageRefPtr stage = UsdStage::Open(c.stagePath);
    ConnectClampToSettings(stage);
    RigExecRigEvaluator evaluator(stage, kPropMathRig);
    CHECK(evaluator.Compile());
    evaluator.SetEvaluationMode(Mode());
    const UsdTimeCode time(c.time);
    const RigExecRigPose before = evaluator.Evaluate(time);
    CHECK(before.valid);
    CHECK(evaluator.Evaluate(time).valid);

    // The sibling nothing connects to: routed to nothing.
    CHECK(stage->GetAttributeAtPath(otherGain).Set(0.5f));
    CHECK(evaluator.GetLastNoticeDisposition() ==
          RigExecNoticeDisposition::Edited);
    CHECK(evaluator.GetLastNoticePatchedPaths().empty());

    // The connected one: named, and it moves the pose.
    CHECK(stage->GetAttributeAtPath(maxGain).Set(0.25f));
    CHECK(evaluator.GetLastNoticeDisposition() ==
          RigExecNoticeDisposition::Edited);
    CHECK(evaluator.GetLastNoticePatchedPaths() ==
          std::vector<SdfPath>{maxGain});
    const RigExecRigPose edited = evaluator.Evaluate(time);
    CheckSamePose("connected chain source (second evaluator)",
                  FreshPose(stage, kPropMathRig, time), edited);
    RigExecRigPose moved;
    RigExecComparePoses(before, edited, &moved);
    CHECK(moved.bakedParityMismatches != 0);

    // The retarget onto the sibling: a connection field, never routed.
    CHECK(Attr(stage, kClampGain, "inputs:max").SetConnections({otherGain}));
    std::printf("  chain source retarget: %s\n",
                DispositionName(evaluator.GetLastNoticeDisposition()));
    CHECK(evaluator.GetLastNoticeDisposition() !=
          RigExecNoticeDisposition::Edited);
    CheckSamePose("chain source retarget",
                  FreshPose(stage, kPropMathRig, time),
                  evaluator.Evaluate(time));

    // Now the sibling is the source, and the old one is read by nothing.
    CHECK(stage->GetAttributeAtPath(otherGain).Set(0.125f));
    CHECK(evaluator.GetLastNoticeDisposition() ==
          RigExecNoticeDisposition::Edited);
    CHECK(evaluator.GetLastNoticePatchedPaths() ==
          std::vector<SdfPath>{otherGain});
    CheckSamePose("retargeted chain source edit",
                  FreshPose(stage, kPropMathRig, time),
                  evaluator.Evaluate(time));
    CHECK(stage->GetAttributeAtPath(maxGain).Set(0.75f));
    CHECK(evaluator.GetLastNoticeDisposition() ==
          RigExecNoticeDisposition::Edited);
    CHECK(evaluator.GetLastNoticePatchedPaths().empty());
    CheckSamePose("old chain source edit",
                  FreshPose(stage, kPropMathRig, time),
                  evaluator.Evaluate(time));
}

}  // namespace

int
main(int argc, char **argv)
{
    if (argc < 2) {
        std::printf("usage: testRigExecValueEdits <examplesDir>\n");
        return 2;
    }
    const std::string examplesDir = argv[1];
    const std::string resources = SchemaResourceDir(examplesDir);
    if (PlugRegistry::GetInstance().RegisterPlugins(resources).empty()) {
        std::printf("FATAL: no schema plugin found at %s\n",
                    resources.c_str());
        return 2;
    }
    std::printf("mode: %s\n", Parity() ? "parity" : "baked");

    TestAKeyedAvarSampleRunsItsCone(examplesDir);
    TestAGuideValueRunsNothing(examplesDir);
    TestAChainMoverInputReachesThroughTheChain(examplesDir);
    TestAnUnreadValueOutsideTheRigRunsNothing(examplesDir);
    TestAConstraintInputSampleRunsItsCone(examplesDir);
    TestARetargetedWalkRebuildsBeforeItsUpstreamEdit(examplesDir);
    TestLayerMetadataRebuilds(examplesDir);
    TestANamedInputNoStepDeclaresBumpsTheStamp(examplesDir);
    TestAConnectedChainSourceIsReported(examplesDir);

    if (failures) {
        std::printf("%d FAILURE(S)\n", failures);
        return 1;
    }
    std::printf("testRigExecValueEdits: all tests passed\n");
    return 0;
}
