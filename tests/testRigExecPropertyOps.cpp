// Property revisions as head-tier ops.
// The baked prologue runs the property chains as its own ops, not through
// the evaluator's _EvaluatePropertyChains: one head op per revision part
// runs from leaves sampled before it, in an
// order derived from what it declares, and only when something it reads
// moved; a publication pass refills the results and the overlay every run.
// These cases hold the ops to the evaluator's own function, run detached
// after every baked generation (results bit for bit, lines line for line),
// under no override, a drag on a mover input, on a chain target, on a phased
// hop, and the drags lifted; pin which parts re-run for a drag on one
// revision's input and on the target; rebind a target removed or retyped
// in place; pin the line order to the chain order
// where the declarations allow another; pin that a stage-frames bail still
// publishes the chains, that a clean tier still publishes, and that a bake
// after an evaluation exports what a fresh one does; and hold a chain
// input connected to a pose interpolator's weight to the authored value.
// The region's readers declare the versions their walks can meet: only the
// readers of a moved version re-run, a stand-aside reader re-runs on its
// hop's drag, every walk's versions are declared and a missing declaration
// refuses the program, a record of another type than the read shadows
// nothing, a chain drag on the biped costs less than a forced run, and the
// exported always-dirty set keeps its recorded digests.
// Registered plain and under the parity entries; under
// RIGEXEC_BAKED_VERIFY_CONES the head tier is also checked against a forced
// run of itself.
// argv[1] = path to the examples directory.
#include "rigExec/backgroundScheduler.h"
#include "rigExec/bakedProgram.h"
#include "rigExec/bakedProgramImpl.h"
#include "rigExec/bakedTrace.h"
#include "rigExec/frozenContext.h"
#include "rigExec/rigEvaluator.h"
#include "rigExec/rigEvaluatorPropertyBindings.h"
#include "rigExecBake/bake.h"
#include "rigExecMath/propertyMath.h"
#include "rigExecPromotionCases.h"

#include "pxr/base/gf/math.h"
#include "pxr/base/gf/quatf.h"
#include "pxr/base/gf/rotation.h"
#include "pxr/base/gf/vec3f.h"
#include "pxr/base/plug/registry.h"
#include "pxr/base/tf/fileUtils.h"
#include "pxr/base/tf/pathUtils.h"
#include "pxr/base/tf/stringUtils.h"
#include "pxr/base/tf/type.h"
#include "pxr/base/vt/array.h"
#include "pxr/usd/sdf/layer.h"
#include "pxr/usd/sdf/types.h"
#include "pxr/usd/usd/attribute.h"
#include "pxr/usd/usd/prim.h"
#include "pxr/usd/usd/relationship.h"
#include "pxr/usd/usd/stage.h"

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <map>
#include <memory>
#include <set>
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

// Head entries are part of the ordinary execution trace.
static std::vector<RigExecOpTraceEntry>
ExecutedHeads(const RigExecBakedProgramImpl &B)
{
    auto trace = RigExecBakedLastRunTrace(B);
    trace.erase(std::remove_if(trace.begin(),trace.end(),
        [&B](const auto &entry) { return !B.steps[entry.step].isHead; }),trace.end());
    return trace;
}


struct Fixture {
    const char *name;
    std::string stage;
    SdfPath rig;
};

std::vector<Fixture>
Fixtures(const std::string &examples)
{
    return {
        {"03", examples + "/03_IkFkBlendClamp.usda",
         SdfPath("/BlendArmAsset/Rig")},
        {"09", examples + "/09_PropertyMathMovers.usda",
         SdfPath("/PropMathAsset/Rig")},
        {"16", examples + "/16_ConnectionReadPhases.usda",
         SdfPath("/PhaseConnectAsset/Rig")},
        {"computed_chains",
         examples + "/../tests/fixtures/computed_chains.usda",
         SdfPath("/Asset/Rig")},
        {"biped", examples + "/biped/Biped_anim.usda", SdfPath("/Biped/Rig")},
    };
}

const Fixture &
FixtureNamed(const std::vector<Fixture> &fixtures, const std::string &name)
{
    for (const Fixture &f : fixtures) {
        if (name == f.name) {
            return f;
        }
    }
    return fixtures.front();
}

std::unique_ptr<RigExecRigEvaluator>
MakeEvaluator(const UsdStageRefPtr &stage, const SdfPath &rig,
              RigExecEvaluationMode mode)
{
    auto evaluator = std::make_unique<RigExecRigEvaluator>(stage, rig);
    std::vector<std::string> errors;
    CHECK(evaluator->Compile(&errors));
    for (const std::string &e : errors) {
        if (e.rfind("warning:", 0) != 0) {
            std::printf("    compile: %s\n", e.c_str());
        }
    }
    evaluator->SetEvaluationMode(mode);
    return evaluator;
}

const RigExecBakedProgramImpl *
Program(const RigExecRigEvaluator &evaluator)
{
    const RigExecBakedProgram *program = evaluator.GetBakedProgram();
    return program ? &program->GetStepGraph() : nullptr;
}

UsdStageRefPtr
StageFrom(const std::string &text)
{
    UsdStageRefPtr stage = UsdStage::CreateInMemory();
    CHECK(stage->GetRootLayer()->ImportFromString(text));
    return stage;
}

// How many head steps are property revisions: the tier also holds the
// rest and ladder composes.
size_t
PropertySteps(const RigExecBakedProgramImpl &B)
{
    return size_t(std::count_if(
        B.steps.begin(), B.steps.end(),
        [](const RigExecBakedStep &step) {
            return step.kind == RigExecBakedStepKind::PropertyRevision;
        }));
}

// The head tier's lines, in head order: what the run put first in the
// generation's diagnostics.
std::vector<std::string>
HeadLines(const RigExecBakedProgramImpl &B)
{
    std::vector<std::string> lines;
    for (const uint32_t index : RigExecBakedHeadIndices(B)) {
        const RigExecBakedStep &step = B.steps[index];
        lines.insert(lines.end(), step.lines.begin(), step.lines.end());
    }
    return lines;
}

std::string
Text(const VtValue &value)
{
    if (value.IsHolding<float>()) {
        char text[64];
        std::snprintf(text, sizeof(text), "%.9g",
                      double(value.UncheckedGet<float>()));
        return text;
    }
    if (value.IsHolding<double>()) {
        char text[64];
        std::snprintf(text, sizeof(text), "%.17g",
                      value.UncheckedGet<double>());
        return text;
    }
    return TfStringify(value);
}

// \p got against \p want, key for key and bit for bit.
size_t
ResultMismatches(const std::map<SdfPath, VtValue> &want,
                 const std::map<SdfPath, VtValue> &got,
                 const std::string &what)
{
    size_t mismatches = 0;
    for (const auto &[path, value] : want) {
        const auto found = got.find(path);
        if (found == got.end()) {
            std::printf("FAIL %s: no result at %s\n", what.c_str(),
                        path.GetText());
            ++mismatches;
        } else if (!RigExecBakedHeadValueSame(value, found->second)) {
            std::printf("FAIL %s: %s is %s, the evaluator's %s\n",
                        what.c_str(), path.GetText(),
                        Text(found->second).c_str(), Text(value).c_str());
            ++mismatches;
        }
    }
    for (const auto &[path, value] : got) {
        if (!want.count(path)) {
            std::printf("FAIL %s: unexpected result at %s\n", what.c_str(),
                        path.GetText());
            ++mismatches;
        }
    }
    return mismatches;
}

size_t
LineMismatches(const std::vector<std::string> &want,
               const std::vector<std::string> &got, const std::string &what)
{
    if (want == got) {
        return 0;
    }
    std::printf("FAIL %s: %zu line(s), the evaluator's %zu\n", what.c_str(),
                got.size(), want.size());
    for (size_t i = 0; i < want.size() || i < got.size(); ++i) {
        const std::string a = i < want.size() ? want[i] : "(none)";
        const std::string b = i < got.size() ? got[i] : "(none)";
        if (a != b) {
            std::printf("    %zu: \"%s\" against \"%s\"\n", i, b.c_str(),
                        a.c_str());
            break;
        }
    }
    return 1;
}

// The last baked run's chain results and lines against the evaluator's own
// function run detached at \p time, which must leave the overlay and the
// memo as it found them.
size_t
CheckAgainstTheEvaluator(RigExecRigEvaluator *evaluator, UsdTimeCode time,
                         const RigExecRigPose &pose, const std::string &what)
{
    const RigExecBakedProgramImpl *program = Program(*evaluator);
    CHECK(program);
    if (!program) {
        return 1;
    }
    const RigExecBakedProgramImpl &B = *program;
    const RigExecResolvedInputs overlay = *B.resolvedInputs;
    const void *memo = RigExecBakedProgramTesting::ChainMemo(*evaluator);
    std::map<SdfPath, VtValue> results;
    std::vector<std::string> lines;
    CHECK(RigExecTestEvaluateChainsDetached(evaluator, time, &results,
                                            &lines));
    CHECK(overlay.HasSameValues(*B.resolvedInputs));
    CHECK(RigExecBakedProgramTesting::ChainMemo(*evaluator) == memo);
    size_t mismatches = ResultMismatches(results, B.propertyResults, what);
    mismatches += LineMismatches(lines, HeadLines(B), what);
    // What the pose publishes at the chains' paths is the results.
    for (const auto &[path, value] : B.propertyResults) {
        const auto found = pose.movedProperties.find(path);
        if (found == pose.movedProperties.end() ||
            !RigExecBakedHeadValueSame(value, found->second)) {
            std::printf("FAIL %s: the pose does not publish %s\n",
                        what.c_str(), path.GetText());
            ++mismatches;
        }
    }
    return mismatches;
}

// A drag on \p property, \p delta from its value at \p time in its own type.
std::vector<RigExecValueOverride>
DragBy(const UsdStageRefPtr &stage, const SdfPath &property, UsdTimeCode time,
       float delta)
{
    const UsdAttribute attribute = stage->GetAttributeAtPath(property);
    if (!attribute) {
        return {};
    }
    const TfType type = attribute.GetTypeName().GetType();
    VtValue value;
    if (type == TfType::Find<float>()) {
        float v = 0.0f;
        attribute.Get(&v, time);
        value = VtValue(v + delta);
    } else if (type == TfType::Find<double>()) {
        double v = 0.0;
        attribute.Get(&v, time);
        value = VtValue(v + double(delta));
    } else if (type == TfType::Find<GfVec3f>()) {
        GfVec3f v(0.0f);
        attribute.Get(&v, time);
        value = VtValue(v + GfVec3f(0.0f, delta, 0.0f));
    } else {
        return {};
    }
    return {RigExecValueOverride{property.GetPrimPath(), TfToken(),
                                 property.GetNameToken(), value}};
}

// The first unconnected input of a revision a drag can stand on.
SdfPath
MoverInput(const RigExecBakedProgramImpl &B)
{
    for (const RigExecBakedPropertyChain &chain : B.propertyChains) {
        for (const RigExecBakedPropertyChain::Revision &r : chain.revisions) {
            for (const RigExecBakedWalk *walk : {&r.value, &r.defaultWeight}) {
                if (walk->flavour == RigExecBakedWalk::Flavour::Pinned &&
                    (walk->type == RigExecBakedHeadValueType::Float ||
                     walk->type == RigExecBakedHeadValueType::Vec3f)) {
                    return walk->hops.front().path;
                }
            }
        }
    }
    return SdfPath();
}

SdfPath
ChainTarget(const RigExecBakedProgramImpl &B)
{
    for (const RigExecBakedPropertyChain &chain : B.propertyChains) {
        if (chain.targetExists &&
            (chain.arm == RigExecBakedPropertyChain::Arm::Float ||
             chain.arm == RigExecBakedPropertyChain::Arm::Double)) {
            return chain.target;
        }
    }
    return SdfPath();
}

SdfPath
PhasedHop(const RigExecBakedProgramImpl &B)
{
    return B.propertyRecords.empty() ? SdfPath()
                                     : B.propertyRecords.front().consumer;
}

// Every fixture at three frames: no override, a drag on a mover input, on a
// chain target and on a phased consumer, then the drags lifted at the held
// frame. After each generation the head tier's results and lines are the
// evaluator's own, and an evaluator that compares baked with dynamic on
// every generation agrees, with its chain memo left as it was by the hook.
void
TestPropertyOpsEqualTheEvaluatorChains(const std::string &examples)
{
    for (const Fixture &f : Fixtures(examples)) {
        UsdStageRefPtr stage = UsdStage::Open(f.stage);
        CHECK(stage);
        if (!stage) {
            continue;
        }
        auto evaluator =
            MakeEvaluator(stage, f.rig, RigExecEvaluationMode::Baked);
        const double start = stage->GetStartTimeCode();
        CHECK(evaluator->Evaluate(UsdTimeCode(start)).valid);
        const RigExecBakedProgramImpl *program = Program(*evaluator);
        CHECK(program && !program->propertyChains.empty());
        if (!program) {
            continue;
        }
        const SdfPath input = MoverInput(*program);
        const SdfPath target = ChainTarget(*program);
        const SdfPath hop = PhasedHop(*program);
        size_t compared = 0, dynamic = 0;
        for (const double offset : {0.0, 2.0, 6.0}) {
            const UsdTimeCode t(start + offset);
            const std::vector<std::pair<std::string, SdfPath>> cases = {
                {"no override", SdfPath()},
                {"mover input", input},
                {"chain target", target},
                {"phased hop", hop},
                {"lifted", SdfPath()}};
            for (const auto &[name, path] : cases) {
                std::vector<RigExecValueOverride> overrides;
                if (!path.IsEmpty()) {
                    overrides = DragBy(stage, path, t, 0.25f);
                    if (overrides.empty()) {
                        continue;
                    }
                } else if (name != "no override" && name != "lifted") {
                    continue;
                }
                const std::string what = std::string(f.name) + " " + name +
                                         " at " +
                                         std::to_string(int(t.GetValue()));
                evaluator->SetInteractiveOverrides(overrides);
                const size_t generations =
                    evaluator->GetBakedGenerationCount();
                const RigExecRigPose pose = evaluator->Evaluate(t);
                CHECK(pose.valid);
                CHECK(pose.bakedParityMismatches == 0);
                if (evaluator->GetBakedGenerationCount() == generations) {
                    // An override the program cannot place answers
                    // dynamically: nothing baked to compare.
                    CHECK(name == "phased hop");
                    ++dynamic;
                    continue;
                }
                CHECK(CheckAgainstTheEvaluator(evaluator.get(), t, pose,
                                               what) == 0);
                ++compared;
            }
        }
        std::printf("equal %s: %zu chain(s), %zu record(s), %zu head "
                    "step(s), %zu generation(s) compared, %zu dynamic; "
                    "drags on %s, %s, %s\n",
                    f.name, program->propertyChains.size(),
                    program->propertyRecords.size(),
                    PropertySteps(*program), compared, dynamic,
                    input.GetText(), target.GetText(), hop.GetText());

        // A generation that runs both paths binds the dynamic chain memo;
        // the hook leaves it as it found it, field for field.
        auto both = MakeEvaluator(stage, f.rig,
                                  RigExecEvaluationMode::BakedWithParityCheck);
        const RigExecRigPose pose = both->Evaluate(UsdTimeCode(start + 2.0));
        CHECK(pose.valid);
        CHECK(pose.bakedParityMismatches == 0);
        const auto *memo = static_cast<const RigExecPropertyChainBindings *>(
            RigExecBakedProgramTesting::ChainMemo(*both));
        CHECK(memo);
        if (!memo) {
            continue;
        }
        const RigExecPropertyChainBindings before = *memo;
        CHECK(CheckAgainstTheEvaluator(both.get(), UsdTimeCode(start + 2.0),
                                       pose,
                                       std::string(f.name) + " parity") == 0);
        CHECK(RigExecBakedProgramTesting::ChainMemo(*both) == memo);
        CHECK(memo->haveLast == before.haveLast);
        CHECK(memo->lastTime == before.lastTime);
        CHECK(memo->lastOverrides == before.lastOverrides);
        CHECK(memo->chains.size() == before.chains.size());
        for (size_t c = 0; c < memo->chains.size() &&
                           c < before.chains.size();
             ++c) {
            const auto &a = memo->chains[c];
            const auto &b = before.chains[c];
            CHECK(a.cached == b.cached && a.published == b.published &&
                  a.changedThisRun == b.changedThisRun &&
                  a.lastValue == b.lastValue &&
                  a.lastPhased == b.lastPhased &&
                  a.lastDiagnostics == b.lastDiagnostics);
        }
    }
}

// The rig the memo cases run on: a three-revision chain on Dial (add, then
// multiply, then add; a chain applies its movers last-declared first), a
// reader of its final value and a reader of it after its first revision.
const char *const kThreeRevisions = R"usda(#usda 1.0
(
    endTimeCode = 10
    startTimeCode = 1
    timeCodesPerSecond = 24
    upAxis = "Y"
)

def Xform "Asset"
{
    def RigExecRoot "Rig"
    {
        def Scope "Controls"
        {
            def RigExecControl "RootCtl" (
                prepend apiSchemas = ["RigExecControlAPI"]
            )
            {
                double avars:ry = 0
                matrix4d rest:space = ( (1, 0, 0, 0), (0, 1, 0, 0), (0, 0, 1, 0), (0, 0, 0, 1) )
            }
        }

        def Scope "Solvers"
        {
            def RigExecFkChain "Chain"
            {
                rel rigExec:controls = </Asset/Rig/Controls/RootCtl>
                rel rigExec:joints = </Asset/Rig/Joints/Root>
            }
        }

        def Scope "Joints"
        {
            def RigExecJoint "Root"
            {
                matrix4d rest:space = ( (1, 0, 0, 0), (0, 1, 0, 0), (0, 0, 1, 0), (0, 0, 0, 1) )
            }
        }

        def Scope "Channels"
        {
            def Scope "Dial"
            {
                float rigExec:amount = 0.5
            }

            def Scope "Out"
            {
                float rigExec:final = 0
                float rigExec:early = 0
            }
        }

        def Scope "Movers"
        {
            def Scope "Dial"
            {
                def RigExecFloatMathMover "Third" (
                    prepend apiSchemas = ["RigExecMoverAPI"]
                )
                {
                    uniform token rigExec:operation = "add"
                    float inputs:value = 0.125
                    rel rigExec:moves = </Asset/Rig/Channels/Dial.rigExec:amount>
                }

                def RigExecFloatMathMover "Second" (
                    prepend apiSchemas = ["RigExecMoverAPI"]
                )
                {
                    uniform token rigExec:operation = "multiply"
                    float inputs:value = 2
                    rel rigExec:moves = </Asset/Rig/Channels/Dial.rigExec:amount>
                }

                def RigExecFloatMathMover "First" (
                    prepend apiSchemas = ["RigExecMoverAPI"]
                )
                {
                    uniform token rigExec:operation = "add"
                    float inputs:value = 0.25
                    rel rigExec:moves = </Asset/Rig/Channels/Dial.rigExec:amount>
                }
            }

            def RigExecFloatMathMover "FinalReader" (
                prepend apiSchemas = ["RigExecMoverAPI"]
            )
            {
                uniform token rigExec:operation = "add"
                float inputs:value (
                    rigExecReadPhase = "final"
                )
                float inputs:value.connect = </Asset/Rig/Channels/Dial.rigExec:amount>
                rel rigExec:moves = </Asset/Rig/Channels/Out.rigExec:final>
            }

            def RigExecFloatMathMover "EarlyReader" (
                prepend apiSchemas = ["RigExecMoverAPI"]
            )
            {
                uniform token rigExec:operation = "add"
                float inputs:value (
                    rigExecReadPhase = "/Asset/Rig/Movers/Dial/First"
                )
                float inputs:value.connect = </Asset/Rig/Channels/Dial.rigExec:amount>
                rel rigExec:moves = </Asset/Rig/Channels/Out.rigExec:early>
            }
        }
    }
}
)usda";

// The head steps the last run executed, as (chain target, part).
std::set<std::pair<SdfPath, int>>
Ran(const RigExecBakedProgramImpl &B)
{
    std::set<std::pair<SdfPath, int>> ran;
    for (const RigExecOpTraceEntry &entry : ExecutedHeads(B)) {
        const RigExecBakedStep &step = B.steps[entry.step];
        if (step.kind == RigExecBakedStepKind::PropertyRevision) {
            ran.emplace(B.propertyChains[size_t(step.object)].target,
                        step.part);
        }
    }
    return ran;
}

std::string
RanText(const std::set<std::pair<SdfPath, int>> &ran)
{
    std::string text;
    for (const auto &[target, part] : ran) {
        text += " " + target.GetPrimPath().GetName() + "." +
                target.GetNameToken().GetString() +
                "#" + std::to_string(part);
    }
    return text.empty() ? " (none)" : text;
}

// A drag on revision 2's inputs:value re-runs parts 2 and 3 of the chain
// and the final reader, and replays parts 0 and 1 and the reader after
// revision 1; a drag on the target re-runs every part and both readers.
// Each run is the evaluator's answer, so an op the memo skipped while its
// input moved would publish a stale value and fail that equality.
void
TestPerVersionMemo()
{
    UsdStageRefPtr stage = StageFrom(kThreeRevisions);
    const SdfPath rig("/Asset/Rig");
    auto evaluator = MakeEvaluator(stage, rig, RigExecEvaluationMode::Baked);
    const UsdTimeCode t(1.0);
    RigExecRigPose pose = evaluator->Evaluate(t);
    CHECK(pose.valid);
    const RigExecBakedProgramImpl *program = Program(*evaluator);
    CHECK(program && program->propertyChains.size() == 3);
    if (!program || program->propertyChains.size() != 3) {
        return;
    }
    const RigExecBakedProgramImpl &B = *program;
    const SdfPath dial("/Asset/Rig/Channels/Dial.rigExec:amount");
    const SdfPath finalOut("/Asset/Rig/Channels/Out.rigExec:final");
    const SdfPath early("/Asset/Rig/Channels/Out.rigExec:early");
    CHECK(B.propertyChains[0].target == dial);
    CHECK(B.propertyRecords.size() == 1);
    CHECK(Ran(B).size() == PropertySteps(B));
    CHECK(CheckAgainstTheEvaluator(evaluator.get(), t, pose, "memo first") ==
          0);

    pose = evaluator->Evaluate(t);
    CHECK(Ran(B).empty());
    CHECK(CheckAgainstTheEvaluator(evaluator.get(), t, pose, "memo again") ==
          0);

    using Ran_ = std::set<std::pair<SdfPath, int>>;
    const auto expect = [&](const Ran_ &want, const std::string &what) {
        const Ran_ got = Ran(B);
        if (got != want) {
            ++failures;
            std::printf("FAIL %s: ran%s, expected%s\n", what.c_str(),
                        RanText(got).c_str(), RanText(want).c_str());
        } else {
            std::printf("memo %s: ran%s\n", what.c_str(),
                        RanText(got).c_str());
        }
    };
    evaluator->SetInteractiveOverrides(
        DragBy(stage, SdfPath("/Asset/Rig/Movers/Dial/Second.inputs:value"), t,
               1.0f));
    pose = evaluator->Evaluate(t);
    expect({{dial, 2}, {dial, 3}, {finalOut, 1}}, "revision 2 dragged");
    CHECK(CheckAgainstTheEvaluator(evaluator.get(), t, pose,
                                   "revision 2 dragged") == 0);

    evaluator->SetInteractiveOverrides({});
    pose = evaluator->Evaluate(t);
    expect({{dial, 2}, {dial, 3}, {finalOut, 1}}, "revision 2 lifted");
    CHECK(CheckAgainstTheEvaluator(evaluator.get(), t, pose,
                                   "revision 2 lifted") == 0);

    evaluator->SetInteractiveOverrides(DragBy(stage, dial, t, 0.5f));
    pose = evaluator->Evaluate(t);
    expect({{dial, 0}, {dial, 1}, {dial, 2}, {dial, 3}, {finalOut, 1},
            {early, 1}},
           "target dragged");
    CHECK(CheckAgainstTheEvaluator(evaluator.get(), t, pose,
                                   "target dragged") == 0);
}

// A chain target removed, then authored again as a double, with nothing
// else edited: the epoch digest names the target's path but not its type, so
// the epoch stands, and the ops must answer from the target as it now is --
// the evaluator rebinds its chain on any edit that reaches the target.
void
TestATargetRetypedInPlaceRebinds()
{
    UsdStageRefPtr stage = StageFrom(kThreeRevisions);
    const SdfPath rig("/Asset/Rig");
    const SdfPath early("/Asset/Rig/Channels/Out.rigExec:early");
    auto evaluator = MakeEvaluator(stage, rig, RigExecEvaluationMode::Baked);
    const UsdTimeCode t(1.0);
    RigExecRigPose pose = evaluator->Evaluate(t);
    CHECK(pose.valid);
    CHECK(CheckAgainstTheEvaluator(evaluator.get(), t, pose,
                                   "target, before") == 0);
    UsdPrim out = stage->GetPrimAtPath(early.GetPrimPath());
    const auto baked = [&](const char *what) {
        const size_t generations = evaluator->GetBakedGenerationCount();
        pose = evaluator->Evaluate(t);
        CHECK(pose.valid);
        CHECK(evaluator->GetBakedGenerationCount() == generations + 1);
        CHECK(CheckAgainstTheEvaluator(evaluator.get(), t, pose, what) == 0);
    };
    CHECK(out.RemoveProperty(early.GetNameToken()));
    baked("target removed");
    CHECK(pose.movedProperties.count(early) == 0);
    CHECK(out.CreateAttribute(early.GetNameToken(), SdfValueTypeNames->Double)
              .Set(1.5));
    baked("target retyped");
    const auto found = pose.movedProperties.find(early);
    CHECK(found != pose.movedProperties.end() &&
          found->second.IsHolding<double>());
}

// Independent chains whose movers sort the other way round from their
// targets: nothing orders one before another but the chain order, which
// _propertyChainOrder takes from the targets. Their lines come out in that
// order, as the evaluator's do.
const char *const kTwoChains = R"usda(#usda 1.0
(
    endTimeCode = 10
    startTimeCode = 1
    timeCodesPerSecond = 24
    upAxis = "Y"
)

def Xform "Asset"
{
    def RigExecRoot "Rig"
    {
        def Scope "Controls"
        {
            def RigExecControl "RootCtl" (
                prepend apiSchemas = ["RigExecControlAPI"]
            )
            {
                double avars:ry = 0
                matrix4d rest:space = ( (1, 0, 0, 0), (0, 1, 0, 0), (0, 0, 1, 0), (0, 0, 0, 1) )
            }
        }

        def Scope "Solvers"
        {
            def RigExecFkChain "Chain"
            {
                rel rigExec:controls = </Asset/Rig/Controls/RootCtl>
                rel rigExec:joints = </Asset/Rig/Joints/Root>
            }
        }

        def Scope "Joints"
        {
            def RigExecJoint "Root"
            {
                matrix4d rest:space = ( (1, 0, 0, 0), (0, 1, 0, 0), (0, 0, 1, 0), (0, 0, 0, 1) )
            }
        }

        def Scope "Channels"
        {
            def Scope "A"
            {
                float rigExec:x = 1
            }

            def Scope "B"
            {
                float rigExec:x = 2
            }

            def Scope "C"
            {
                double rigExec:x = 1e300
            }
        }

        def Scope "Movers"
        {
            def RigExecFloatMathMover "Zed" (
                prepend apiSchemas = ["RigExecMoverAPI"]
            )
            {
                uniform token rigExec:operation = "add"
                bool inputs:enabled = 0
                float inputs:value = 1
                rel rigExec:moves = </Asset/Rig/Channels/A.rigExec:x>
            }

            def RigExecFloatMathMover "Alpha" (
                prepend apiSchemas = ["RigExecMoverAPI"]
            )
            {
                uniform token rigExec:operation = "add"
                bool inputs:enabled = 0
                float inputs:value = 1
                rel rigExec:moves = </Asset/Rig/Channels/B.rigExec:x>
            }

            def RigExecFloatMathMover "Keep" (
                prepend apiSchemas = ["RigExecMoverAPI"]
            )
            {
                uniform token rigExec:operation = "multiply"
                float inputs:value = 1
                rel rigExec:moves = </Asset/Rig/Channels/C.rigExec:x>
            }

            def RigExecFloatMathMover "Yon" (
                prepend apiSchemas = ["RigExecMoverAPI"]
            )
            {
                uniform token rigExec:operation = "add"
                float inputs:defaultWeight = 2
                float inputs:value = 1
                rel rigExec:moves = </Asset/Rig/Channels/A.rigExec:x>
            }
        }
    }
}
)usda";

std::vector<std::string>
ChainLines(const RigExecRigPose &pose)
{
    std::vector<std::string> lines;
    for (const std::string &line : pose.diagnostics) {
        if (line.rfind("diag /Asset/Rig/Movers/", 0) == 0 ||
            line.rfind("property chain ", 0) == 0) {
            lines.push_back(line);
        }
    }
    return lines;
}

void
TestHeadOrderIsChainOrder()
{
    UsdStageRefPtr stage = StageFrom(kTwoChains);
    const SdfPath rig("/Asset/Rig");
    const UsdTimeCode t(1.0);
    auto evaluator = MakeEvaluator(stage, rig, RigExecEvaluationMode::Baked);
    const RigExecRigPose pose = evaluator->Evaluate(t);
    CHECK(pose.valid);
    const RigExecBakedProgramImpl *program = Program(*evaluator);
    CHECK(program && program->propertyChains.size() == 3);
    if (!program || program->propertyChains.size() != 3) {
        return;
    }
    const RigExecBakedProgramImpl &B = *program;
    CHECK(B.propertyChains[0].target ==
          SdfPath("/Asset/Rig/Channels/A.rigExec:x"));
    // The order holds every step, chain by chain, part by part.
    int lastChain = -1, lastPart = -1;
    bool ordered = true;
    for (const uint32_t index : RigExecBakedHeadIndices(B)) {
        const RigExecBakedStep &step = B.steps[index];
        if (step.kind != RigExecBakedStepKind::PropertyRevision) {
            continue;
        }
        ordered = ordered && (step.object > lastChain ||
                              (step.object == lastChain &&
                               step.part == lastPart + 1));
        lastChain = step.object;
        lastPart = step.part;
    }
    CHECK(ordered);
    // No step of one chain reads the other's.
    for (const RigExecBakedStep &step : B.steps) {
        if (step.kind != RigExecBakedStepKind::PropertyRevision) {
            continue;
        }
        for (const uint32_t pred : step.preds) {
            CHECK(B.steps[pred].object == step.object);
        }
    }
    const std::vector<std::string> want = {
        "diag /Asset/Rig/Movers/Yon: inputs:defaultWeight must be finite "
        "and in [0, 1]; revision passed through",
        "diag /Asset/Rig/Movers/Zed: disabled; revision passed through",
        "diag /Asset/Rig/Movers/Alpha: disabled; revision passed through",
        // A double base past float's range: the evaluator tests a double
        // chain's values as floats.
        "property chain /Asset/Rig/Channels/C.rigExec:x: authored base is "
        "not finite; chain skipped"};
    CHECK(LineMismatches(want, ChainLines(pose), "two chains, baked") == 0);
    auto reference =
        MakeEvaluator(stage, rig, RigExecEvaluationMode::ExecReference);
    CHECK(LineMismatches(ChainLines(reference->Evaluate(t)), ChainLines(pose),
                         "two chains, against the reference") == 0);
    CHECK(CheckAgainstTheEvaluator(evaluator.get(), t, pose, "two chains") ==
          0);
}

SdfPath
FindRig(const UsdStageRefPtr &stage)
{
    for (const UsdPrim &prim : stage->Traverse()) {
        if (prim.GetTypeName() == "RigExecRoot") {
            return prim.GetPath();
        }
    }
    return SdfPath();
}

// A constraint target that stops being a transform hands the generation
// back in the prologue, after the chains: the pose it gives back still
// carries the chain results and lines, as the dynamic walk's does.
void
TestTheStageFramesBailStillPublishesChains(const std::string &examples)
{
    const UsdStageRefPtr stage =
        UsdStage::Open(examples + "/05_TwistRibbonSpine.usda");
    CHECK(stage);
    if (!stage) {
        return;
    }
    const SdfPath rigPath = FindRig(stage);
    const SdfPath target("/SpineAsset/Geom/Probe");
    const UsdPrim targetPrim = stage->DefinePrim(target, TfToken("Xform"));
    const UsdPrim aim = stage->DefinePrim(
        rigPath.AppendPath(SdfPath("Movers/ProbeAim")),
        TfToken("RigExecAimConstraint"));
    CHECK(aim.CreateAttribute(TfToken("rigExec:aimAxis"),
                              SdfValueTypeNames->Token, false,
                              SdfVariabilityUniform)
              .Set(TfToken("z")));
    CHECK(aim.CreateRelationship(TfToken("rigExec:aimTarget"))
              .SetTargets({rigPath.AppendPath(SdfPath("Joints/Root/Chest"))}));
    CHECK(aim.CreateRelationship(TfToken("rigExec:moves"))
              .SetTargets({target}));
    // A chain: one revision that adds, one that is off and says so.
    const SdfPath dial = rigPath.AppendPath(SdfPath("BailChannels"))
                             .AppendProperty(TfToken("rigExec:dial"));
    const UsdPrim channels =
        stage->DefinePrim(dial.GetPrimPath(), TfToken("Scope"));
    CHECK(channels.CreateAttribute(TfToken("rigExec:dial"),
                                   SdfValueTypeNames->Float)
              .Set(0.5f));
    const auto mover = [&](const char *name, bool enabled) {
        const UsdPrim prim = stage->DefinePrim(
            rigPath.AppendPath(SdfPath(std::string("Movers/") + name)),
            TfToken("RigExecFloatMathMover"));
        prim.ApplyAPI(TfToken("RigExecMoverAPI"));
        prim.GetRelationship(TfToken("rigExec:moves")).SetTargets({dial});
        prim.GetAttribute(TfToken("rigExec:operation")).Set(TfToken("add"));
        prim.GetAttribute(TfToken("inputs:value")).Set(0.25f);
        prim.GetAttribute(TfToken("inputs:enabled")).Set(enabled);
    };
    mover("BailDial", true);
    mover("BailOff", false);

    RigExecRigEvaluator rig(stage, rigPath);
    rig.SetEvaluationMode(RigExecEvaluationMode::Baked);
    std::vector<std::string> errors;
    CHECK(rig.Compile(&errors));
    for (const std::string &e : errors) {
        std::printf("    compile: %s\n", e.c_str());
    }
    const std::unique_ptr<RigExecBakedProgram> kept =
        RigExecBakedProgram::Build(&rig, nullptr);
    CHECK(kept);
    if (!kept) {
        return;
    }
    RigExecRigPose first;
    CHECK(kept->Run(UsdTimeCode(1001.0), &first));

    CHECK(targetPrim.SetTypeName(TfToken("Scope")));
    const UsdTimeCode bent(1024.0);
    RigExecRigPose given;
    CHECK(!kept->Run(bent, &given));
    CHECK(kept->GetLastBail() == RigExecBakedBail::StageFrames);
    CHECK(!given.valid);
    std::map<SdfPath, VtValue> results;
    std::vector<std::string> lines;
    CHECK(RigExecTestEvaluateChainsDetached(&rig, bent, &results, &lines));
    CHECK(results.count(dial) == 1);
    CHECK(ResultMismatches(results, given.movedProperties, "bail") == 0);
    // The chains' lines first, the target's last.
    CHECK(!lines.empty());
    CHECK(given.diagnostics.size() >= lines.size() + 1);
    if (given.diagnostics.size() >= lines.size() + 1) {
        const std::vector<std::string> head(
            given.diagnostics.begin(),
            given.diagnostics.begin() + long(lines.size()));
        CHECK(LineMismatches(lines, head, "bail lines") == 0);
        CHECK(given.diagnostics.back() ==
              "could not resolve constraint target " + target.GetString() +
                  " relative to the asset root");
    }
    std::printf("bail: %zu result(s), %zu chain line(s)\n", results.size(),
                lines.size());
}

// The same frame twice with nothing moved: the second run executes no head
// op but the volatile ones (a weight-object envelope) and re-reads no head
// leaf, yet publishes the same results, overlay entries and pose values bit
// for bit -- the publication pass, not an op, fills them.
void
TestACleanHeadTierStillPublishes(const std::string &examples)
{
    const std::vector<Fixture> fixtures = Fixtures(examples);
    for (const char *name : {"09", "computed_chains"}) {
        const Fixture &f = FixtureNamed(fixtures, name);
        UsdStageRefPtr stage = UsdStage::Open(f.stage);
        CHECK(stage);
        if (!stage) {
            continue;
        }
        auto evaluator =
            MakeEvaluator(stage, f.rig, RigExecEvaluationMode::Baked);
        const UsdTimeCode t(stage->GetStartTimeCode() + 2.0);
        const RigExecRigPose first = evaluator->Evaluate(t);
        CHECK(first.valid);
        const RigExecBakedProgramImpl *program = Program(*evaluator);
        CHECK(program && !program->propertyResults.empty());
        if (!program) {
            continue;
        }
        const RigExecBakedProgramImpl &B = *program;
        const std::map<SdfPath, VtValue> results = B.propertyResults;
        std::map<SdfPath, VtValue> overlay;
        for (const auto &[path, value] : results) {
            const VtValue *standing = B.resolvedInputs->Find(path);
            overlay[path] = standing ? *standing : VtValue();
        }
        const uint64_t samples = B.headLeafSamples;
        const RigExecRigPose again = evaluator->Evaluate(t);
        CHECK(again.valid);
        size_t ran = 0, volatileRan = 0;
        for (const RigExecOpTraceEntry &entry : ExecutedHeads(B)) {
            if (B.steps[entry.step].alwaysRuns) {
                ++volatileRan;
            } else {
                ++ran;
                std::printf("FAIL %s: %s ran with nothing moved\n", name,
                            entry.label.c_str());
            }
        }
        CHECK(ran == 0);
        CHECK(B.headLeafSamples == samples);
        CHECK(ResultMismatches(results, B.propertyResults,
                               std::string(name) + " clean") == 0);
        for (const auto &[path, value] : overlay) {
            const VtValue *standing = B.resolvedInputs->Find(path);
            CHECK(standing && RigExecBakedHeadValueSame(value, *standing));
        }
        for (const auto &[path, value] : results) {
            const auto a = first.movedProperties.find(path);
            const auto b = again.movedProperties.find(path);
            CHECK(a != first.movedProperties.end() &&
                  b != again.movedProperties.end() &&
                  RigExecBakedHeadValueSame(a->second, b->second));
        }
        std::printf("clean %s: %zu result(s), %zu volatile op(s) ran\n", name,
                    results.size(), volatileRan);
    }
}

uint64_t
Fnv(const std::vector<uint8_t> &bytes)
{
    uint64_t h = 1469598103934665603ull;
    for (const uint8_t b : bytes) {
        h ^= b;
        h *= 1099511628211ull;
    }
    return h;
}

// A baked evaluation at the bake time, then RequestFullRun at that time
// (the bake's request, which the bake's own run may not reach on the same
// program: it compiles again first), then the bake: a forced run executes
// every head op, and the file is the one an evaluator that never evaluated
// bakes (its path reads come from the run's chain results, its blend sample
// points from the overlay). Byte identity with earlier trees is the binary
// hash guard's.
void
TestABakeAfterAnEvaluationExportsTheChains(const std::string &examples)
{
    const std::vector<Fixture> fixtures = Fixtures(examples);
    for (const char *name : {"09", "computed_chains"}) {
        const Fixture &f = FixtureNamed(fixtures, name);
        UsdStageRefPtr stage = UsdStage::Open(f.stage);
        CHECK(stage);
        if (!stage) {
            continue;
        }
        auto evaluator =
            MakeEvaluator(stage, f.rig, RigExecEvaluationMode::Baked);
        const double t = stage->GetStartTimeCode();
        CHECK(evaluator->Evaluate(UsdTimeCode(t)).valid);
        // The request the bake makes, on the standing program: a run at the
        // same time with nothing moved still executes every head op.
        if (const RigExecBakedProgram *standing =
                evaluator->GetBakedProgram()) {
            standing->RequestFullRun();
            CHECK(evaluator->Evaluate(UsdTimeCode(t)).valid);
            CHECK(evaluator->GetBakedProgram() == standing);
            CHECK(ExecutedHeads(standing->GetStepGraph()).size() ==
                  RigExecBakedHeadIndices(standing->GetStepGraph()).size());
        }
        RigExecBakeOpts opts;
        opts.time = t;
        const auto bake = [&](RigExecRigEvaluator &from,
                              RigExecBakeResult *result) {
            std::string error;
            const bool baked = RigExecBakeToBinary(from, opts, result, &error);
            if (!baked) {
                std::printf("FAIL %s: bake refused: %s\n", name,
                            error.c_str());
            }
            CHECK(baked);
        };
        RigExecBakeResult result;
        bake(*evaluator, &result);
        const RigExecBakedProgramImpl *program = Program(*evaluator);
        CHECK(program);
        if (program) {
            CHECK(ExecutedHeads(*program).size() ==
                  RigExecBakedHeadIndices(*program).size());
        }
        auto fresh = MakeEvaluator(stage, f.rig, RigExecEvaluationMode::Baked);
        RigExecBakeResult reference;
        bake(*fresh, &reference);
        if (result.bytes != reference.bytes) {
            std::printf("FAIL %s: bake %016llx, a fresh evaluator's %016llx\n",
                        name,
                        static_cast<unsigned long long>(Fnv(result.bytes)),
                        static_cast<unsigned long long>(
                            Fnv(reference.bytes)));
        }
        CHECK(!result.bytes.empty() && result.bytes == reference.bytes);
        std::printf("bake %s: %016llx, %zu path read(s)\n", name,
                    static_cast<unsigned long long>(Fnv(result.bytes)),
                    result.pathReadsWritten);
    }
}

// A chain input connected to a pose interpolator's outputs:weight, which
// carries an authored value: the chains run before the interpolator
// publishes, so both paths compute the chain from the authored value, bit
// for bit, while the interpolator publishes another weight later in the
// generation.
void
TestAChainReadsAnInterpolatorWeightAsAuthored()
{
    UsdStageRefPtr stage = UsdStage::CreateInMemory();
    const SdfPath rig("/Asset/Rig");
    const SdfPath driver("/Asset/Rig/Controls/Shoulder/Driver");
    const SdfPath interpolator("/Asset/Rig/PoseInterpolators/Swing");
    stage->DefinePrim(SdfPath("/Asset"), TfToken("Scope"));
    stage->DefinePrim(rig, TfToken("RigExecRoot"));
    stage->DefinePrim(SdfPath("/Asset/Rig/Controls"), TfToken("Scope"));
    stage->DefinePrim(driver.GetParentPath(), TfToken("RigExecControl"));
    stage->DefinePrim(driver, TfToken("RigExecControl"));
    stage->DefinePrim(SdfPath("/Asset/Rig/PoseInterpolators"),
                      TfToken("Scope"));
    const UsdPrim swing =
        stage->DefinePrim(interpolator, TfToken("RigExecPoseInterpolator"));
    swing.CreateRelationship(TfToken("rigExec:driver")).SetTargets({driver});
    swing.GetAttribute(TfToken("rigExec:kernel")).Set(TfToken("gaussian"));
    swing.GetAttribute(TfToken("rigExec:twistAxis")).Set(TfToken("Z"));
    const auto addPose = [&](const char *name, double degrees) {
        const UsdPrim pose = stage->DefinePrim(
            interpolator.AppendChild(TfToken(name)), TfToken("RigExecPose"));
        pose.GetAttribute(TfToken("rigExec:poseType")).Set(TfToken("whole"));
        const double half = GfDegreesToRadians(degrees) * 0.5;
        pose.GetAttribute(TfToken("rigExec:rotation"))
            .Set(GfQuatf(float(std::cos(half)),
                         GfVec3f(0.0f, 0.0f, float(std::sin(half)))));
        pose.GetAttribute(TfToken("rigExec:rotationRadius"))
            .Set(float(0.78539816339744830961));
        return pose;
    };
    addPose("neutral", 0.0);
    const UsdPrim forward = addPose("Forward", 45.0);
    addPose("Back", -45.0);
    const SdfPath weight =
        forward.GetPath().AppendProperty(TfToken("outputs:weight"));
    CHECK(forward.GetAttribute(TfToken("outputs:weight")).Set(0.25f));

    const SdfPath gain("/Asset/Rig/Channels/Dial.rigExec:gain");
    stage->DefinePrim(gain.GetPrimPath(), TfToken("Scope"))
        .CreateAttribute(TfToken("rigExec:gain"), SdfValueTypeNames->Float)
        .Set(0.125f);
    stage->DefinePrim(SdfPath("/Asset/Rig/Movers"), TfToken("Scope"));
    const UsdPrim mover = stage->DefinePrim(
        SdfPath("/Asset/Rig/Movers/Gain"), TfToken("RigExecFloatMathMover"));
    mover.ApplyAPI(TfToken("RigExecMoverAPI"));
    mover.GetRelationship(TfToken("rigExec:moves")).SetTargets({gain});
    mover.GetAttribute(TfToken("rigExec:operation")).Set(TfToken("add"));
    UsdAttribute value = mover.GetAttribute(TfToken("inputs:value"));
    value.Set(0.0f);
    value.AddConnection(weight);
    // The driver on Forward, so the interpolator publishes 1 there.
    stage->GetPrimAtPath(driver).GetAttribute(TfToken("avars:rz")).Set(45.0);

    auto reference =
        MakeEvaluator(stage, rig, RigExecEvaluationMode::ExecReference);
    auto baked = MakeEvaluator(stage, rig, RigExecEvaluationMode::Baked);
    const UsdTimeCode t = UsdTimeCode::Default();
    const RigExecRigPose want = reference->Evaluate(t);
    const size_t generations = baked->GetBakedGenerationCount();
    const RigExecRigPose got = baked->Evaluate(t);
    CHECK(want.valid && got.valid);
    CHECK(baked->GetBakedGenerationCount() == generations + 1);
    RigExecPropertyMathParams<float> params;
    params.op = RigExecPropertyOp::Add;
    params.value = 0.25f;
    params.weight = 1.0f;
    const float authored = RigExecApplyFloatMath(0.125f, params);
    for (const RigExecRigPose *pose : {&want, &got}) {
        const auto found = pose->movedProperties.find(gain);
        CHECK(found != pose->movedProperties.end() &&
              RigExecBakedHeadValueSame(found->second, VtValue(authored)));
        const auto published = pose->movedProperties.find(weight);
        CHECK(published != pose->movedProperties.end() &&
              published->second.IsHolding<float>() &&
              std::abs(published->second.UncheckedGet<float>() - 1.0f) <
                  1e-5f);
    }
    if (Program(*baked)) {
        CHECK(CheckAgainstTheEvaluator(baked.get(), t, got,
                                       "interpolator weight") == 0);
    }
}

// A double chain of three revisions on a control's avars:rz (First adds,
// Second multiplies, Third adds), read by three position constraints'
// float inputs:defaultWeight: Follow at `final`, FollowBase through an
// undeclared connection (the base), and FollowViaHop at `final` through
// FollowBase's input, a hop of its record.
const char *const kVersionReaders = R"usda(#usda 1.0
(
    endTimeCode = 10
    startTimeCode = 1
    timeCodesPerSecond = 24
    upAxis = "Y"
)

def Xform "Asset"
{
    def Xform "Source"
    {
        matrix4d xformOp:transform = ( (1, 0, 0, 0), (0, 1, 0, 0), (0, 0, 1, 0), (10, 0, 0, 1) )
        uniform token[] xformOpOrder = ["xformOp:transform"]
    }

    def Xform "Target"
    {
        matrix4d xformOp:transform = ( (1, 0, 0, 0), (0, 1, 0, 0), (0, 0, 1, 0), (0, 0, 0, 1) )
        uniform token[] xformOpOrder = ["xformOp:transform"]
    }

    def Xform "TargetBase"
    {
        matrix4d xformOp:transform = ( (1, 0, 0, 0), (0, 1, 0, 0), (0, 0, 1, 0), (0, 0, 0, 1) )
        uniform token[] xformOpOrder = ["xformOp:transform"]
    }

    def Xform "TargetHop"
    {
        matrix4d xformOp:transform = ( (1, 0, 0, 0), (0, 1, 0, 0), (0, 0, 1, 0), (0, 0, 0, 1) )
        uniform token[] xformOpOrder = ["xformOp:transform"]
    }

    def RigExecRoot "Rig"
    {
        def Scope "Controls"
        {
            def RigExecControl "RootCtl" (
                prepend apiSchemas = ["RigExecControlAPI"]
            )
            {
                double avars:rz = 0.2
                matrix4d rest:space = ( (1, 0, 0, 0), (0, 1, 0, 0), (0, 0, 1, 0), (0, 0, 0, 1) )
            }
        }

        def Scope "Solvers"
        {
            def RigExecFkChain "Chain"
            {
                rel rigExec:controls = </Asset/Rig/Controls/RootCtl>
                rel rigExec:joints = </Asset/Rig/Joints/Root>
            }
        }

        def Scope "Joints"
        {
            def RigExecJoint "Root"
            {
                matrix4d rest:space = ( (1, 0, 0, 0), (0, 1, 0, 0), (0, 0, 1, 0), (0, 0, 0, 1) )
            }
        }

        def Scope "Movers"
        {
            def Scope "Rz"
            {
                def RigExecFloatMathMover "Third" (
                    prepend apiSchemas = ["RigExecMoverAPI"]
                )
                {
                    uniform token rigExec:operation = "add"
                    float inputs:value = 0.125
                    rel rigExec:moves = </Asset/Rig/Controls/RootCtl.avars:rz>
                }

                def RigExecFloatMathMover "Second" (
                    prepend apiSchemas = ["RigExecMoverAPI"]
                )
                {
                    uniform token rigExec:operation = "multiply"
                    float inputs:value = 2
                    rel rigExec:moves = </Asset/Rig/Controls/RootCtl.avars:rz>
                }

                def RigExecFloatMathMover "First" (
                    prepend apiSchemas = ["RigExecMoverAPI"]
                )
                {
                    uniform token rigExec:operation = "add"
                    float inputs:value = 0.05
                    rel rigExec:moves = </Asset/Rig/Controls/RootCtl.avars:rz>
                }
            }

            def RigExecPositionConstraint "Follow" (
                prepend apiSchemas = ["RigExecMoverAPI"]
            )
            {
                float inputs:defaultWeight (
                    rigExecReadPhase = "final"
                )
                prepend float inputs:defaultWeight.connect = </Asset/Rig/Controls/RootCtl.avars:rz>
                rel rigExec:moves = </Asset/Target>
                rel rigExec:sources = </Asset/Source>
            }

            def RigExecPositionConstraint "FollowBase" (
                prepend apiSchemas = ["RigExecMoverAPI"]
            )
            {
                prepend float inputs:defaultWeight.connect = </Asset/Rig/Controls/RootCtl.avars:rz>
                rel rigExec:moves = </Asset/TargetBase>
                rel rigExec:sources = </Asset/Source>
            }

            def RigExecPositionConstraint "FollowViaHop" (
                prepend apiSchemas = ["RigExecMoverAPI"]
            )
            {
                float inputs:defaultWeight (
                    rigExecReadPhase = "final"
                )
                prepend float inputs:defaultWeight.connect = </Asset/Rig/Movers/FollowBase.inputs:defaultWeight>
                rel rigExec:moves = </Asset/TargetHop>
                rel rigExec:sources = </Asset/Source>
            }
        }
    }
}
)usda";

const SdfPath kRz("/Asset/Rig/Controls/RootCtl.avars:rz");
const SdfPath kFollow("/Asset/Rig/Movers/Follow");
const SdfPath kFollowBase("/Asset/Rig/Movers/FollowBase");
const SdfPath kFollowViaHop("/Asset/Rig/Movers/FollowViaHop");

// The region step that commits the constraint at \p path, or -1.
int
ConstraintStep(const RigExecBakedProgramImpl &B, const SdfPath &path)
{
    for (size_t i = 0; i < B.steps.size(); ++i) {
        const RigExecBakedStep &step = B.steps[i];
        if (step.kind != RigExecBakedStepKind::Constraint) {
            continue;
        }
        const RigExecBakedProgramImpl::WalkStep &walk =
            B.walkSteps[size_t(step.object)];
        if (!walk.solverBatch && walk.index >= 0 &&
            B.constraints[size_t(walk.index)].path == path) {
            return int(i);
        }
    }
    return -1;
}

// The constraint at \p path's inputs:defaultWeight leaf.
float
ConstraintWeight(const RigExecBakedProgramImpl &B, const SdfPath &path)
{
    for (const RigExecBakedProgramImpl::Constraint &constraint :
         B.constraints) {
        if (constraint.path == path) {
            return RigExecBakedLeafRead(B, constraint.defaultWeight);
        }
    }
    return -1.0f;
}

// The region steps the last run executed.
std::set<size_t>
StepsRan(const RigExecBakedProgramImpl &B)
{
    std::set<size_t> ran;
    for (const RigExecOpTraceEntry &entry : RigExecBakedLastRunTrace(B)) {
        ran.insert(entry.step);
    }
    return ran;
}

// What two poses publish: moved properties bit for bit, control and joint
// frames, and the lines (less the mover graph's work line).
bool
SameReadings(const RigExecRigPose &a, const RigExecRigPose &b,
             const std::string &what)
{
    std::string why;
    if (!a.valid || !b.valid) {
        why = "an invalid pose";
    } else if (a.movedProperties.size() != b.movedProperties.size()) {
        why = "moved property count";
    } else if (a.controlFrames != b.controlFrames) {
        why = "control frames";
    } else if (a.jointFramesFinal != b.jointFramesFinal) {
        why = "joint frames";
    } else {
        for (auto i = a.movedProperties.begin(),
                  j = b.movedProperties.begin();
             i != a.movedProperties.end(); ++i, ++j) {
            if (i->first != j->first ||
                !RigExecBakedHeadValueSame(i->second, j->second)) {
                why = "moved property " + i->first.GetString();
                break;
            }
        }
        const auto lines = [](const RigExecRigPose &pose) {
            std::vector<std::string> out;
            for (const std::string &line : pose.diagnostics) {
                if (line.rfind("mover graph: ", 0) != 0) {
                    out.push_back(line);
                }
            }
            return out;
        };
        if (why.empty() && lines(a) != lines(b)) {
            why = "diagnostics";
        }
    }
    if (!why.empty()) {
        ++failures;
        std::printf("FAIL %s: the poses differ in %s\n", what.c_str(),
                    why.c_str());
        return false;
    }
    return true;
}

// A program built fresh on \p stage under \p overrides: its first run
// executes everything.
RigExecRigPose
FreshPose(const UsdStageRefPtr &stage, const SdfPath &rig, UsdTimeCode time,
          const std::vector<RigExecValueOverride> &overrides)
{
    auto fresh = MakeEvaluator(stage, rig, RigExecEvaluationMode::Baked);
    fresh->SetInteractiveOverrides(overrides);
    return fresh->Evaluate(time);
}

// A frozen job at \p time under \p overrides, sampled and warmed against a
// snapshot of \p evaluator's program: accepted, and equal to live.
void
CheckFrozenJob(RigExecRigEvaluator *evaluator, const SdfPath &rig,
               UsdTimeCode time,
               const std::vector<RigExecValueOverride> &overrides,
               const RigExecRigPose &live, const std::string &what)
{
    std::shared_ptr<const RigExecFrozenProgram> frozen;
    std::string error;
    CHECK(RigExecFreezeProgram(*evaluator, &frozen, &error));
    if (!frozen) {
        std::printf("FAIL %s: freeze refused: %s\n", what.c_str(),
                    error.c_str());
        return;
    }
    RigExecFrameInputs inputs;
    CHECK(RigExecSampleFrameInputs(*evaluator, time, overrides, &inputs,
                                   &error));
    CHECK(!inputs.HasChainResolvedInputs());
    RigExecBackgroundScheduler scheduler;
    RigExecFrozenEvalContext context;
    context.epochDigest = evaluator->GetBindingEpochDigest();
    context.generation = scheduler.CurrentGeneration(rig);
    context.slotCount = evaluator->GetBakedProgram()->GetProviderCount();
    context.varyingInputCount = inputs.values.size();
    context.frozen = frozen.get();
    const RigExecRigPose warmed = RigExecEvaluateFrozen(
        context, inputs, RigExecMakeProductionStepRunner(), &scheduler, rig);
    if (!warmed.valid) {
        ++failures;
        std::printf("FAIL %s: the frozen job was declined\n", what.c_str());
        return;
    }
    SameReadings(live, warmed, what + " (frozen)");
}

// A drag on revision 2's input moves the chain's versions 2 and 3, so it
// re-runs head parts 2-3 and the steps that declare the final version --
// Follow, and FollowViaHop through its hop -- and not FollowBase, whose
// walk declares the base record alone; a rule re-running every
// chain-reading step on any moved chain result fails here. The lift
// re-runs the same set. Each pose equals a fresh program's, and a
// frozen job under the same drag is accepted and equals live (the worker
// reports no trace, so its executed set is not observed).
void
TestOnlyReadersOfTheChangedVersionRerun()
{
    UsdStageRefPtr stage = StageFrom(kVersionReaders);
    const SdfPath rig("/Asset/Rig");
    auto evaluator = MakeEvaluator(stage, rig, RigExecEvaluationMode::Baked);
    const UsdTimeCode t(1.0);
    CHECK(evaluator->Evaluate(t).valid);
    CHECK(evaluator->Evaluate(t).valid);
    const RigExecBakedProgramImpl *program = Program(*evaluator);
    CHECK(program && program->propertyChains.size() == 1);
    if (!program || program->propertyChains.size() != 1) {
        return;
    }
    const RigExecBakedProgramImpl &B = *program;
    const int follow = ConstraintStep(B, kFollow);
    const int base = ConstraintStep(B, kFollowBase);
    const int viaHop = ConstraintStep(B, kFollowViaHop);
    CHECK(follow >= 0 && base >= 0 && viaHop >= 0);
    if (follow < 0 || base < 0 || viaHop < 0) {
        return;
    }
    // Each reads a walk, and declares its version.
    for (const int index : {follow, base, viaHop}) {
        CHECK(!B.steps[size_t(index)].readerWalks.empty());
        CHECK(!B.steps[size_t(index)].reads.empty());
    }
    const auto check = [&](const std::vector<RigExecValueOverride> &drag,
                           const std::string &what) {
        evaluator->SetInteractiveOverrides(drag);
        const RigExecRigPose pose = evaluator->Evaluate(t);
        CHECK(pose.valid);
        const std::set<size_t> ran = StepsRan(B);
        const bool ok = ran.count(size_t(follow)) &&
                        ran.count(size_t(viaHop)) && !ran.count(size_t(base));
        if (!ok) {
            ++failures;
        }
        std::printf("%s %s: Follow %s, FollowViaHop %s, FollowBase %s; "
                    "head ran%s\n",
                    ok ? "readers" : "FAIL readers", what.c_str(),
                    ran.count(size_t(follow)) ? "ran" : "skipped",
                    ran.count(size_t(viaHop)) ? "ran" : "skipped",
                    ran.count(size_t(base)) ? "ran" : "skipped",
                    RanText(Ran(B)).c_str());
        CHECK((Ran(B) == std::set<std::pair<SdfPath, int>>{{kRz, 2},
                                                           {kRz, 3}}));
        SameReadings(FreshPose(stage, rig, t, drag), pose, what);
        CheckFrozenJob(evaluator.get(), rig, t, drag, pose, what);
    };
    check(DragBy(stage,
                 SdfPath("/Asset/Rig/Movers/Rz/Second.inputs:value"), t,
                 1.0f),
          "revision 2 dragged");
    check({}, "revision 2 lifted");
}

// A drag on FollowBase's input, a hop of FollowViaHop's record, stands that
// record aside: FollowViaHop re-runs and reads the hop's override, and so
// does FollowBase, whose own input it is. Lifting it re-runs both, back on
// the chain's final version and base. Follow, which reads neither, stays.
void
TestAStandAsideReaderDeclaresItsHops()
{
    UsdStageRefPtr stage = StageFrom(kVersionReaders);
    const SdfPath rig("/Asset/Rig");
    auto evaluator = MakeEvaluator(stage, rig, RigExecEvaluationMode::Baked);
    const UsdTimeCode t(1.0);
    CHECK(evaluator->Evaluate(t).valid);
    CHECK(evaluator->Evaluate(t).valid);
    const RigExecBakedProgramImpl *program = Program(*evaluator);
    CHECK(program != nullptr);
    if (!program) {
        return;
    }
    const RigExecBakedProgramImpl &B = *program;
    const int follow = ConstraintStep(B, kFollow);
    const int base = ConstraintStep(B, kFollowBase);
    const int viaHop = ConstraintStep(B, kFollowViaHop);
    CHECK(follow >= 0 && base >= 0 && viaHop >= 0);
    if (follow < 0 || base < 0 || viaHop < 0) {
        return;
    }
    const float finalWeight = ConstraintWeight(B, kFollowViaHop);
    const float baseWeight = ConstraintWeight(B, kFollowBase);
    CHECK(finalWeight == ConstraintWeight(B, kFollow));
    CHECK(finalWeight != baseWeight);

    const std::vector<RigExecValueOverride> drag = {
        RigExecValueOverride{kFollowBase, TfToken(),
                             TfToken("inputs:defaultWeight"), VtValue(0.3f)}};
    evaluator->SetInteractiveOverrides(drag);
    RigExecRigPose pose = evaluator->Evaluate(t);
    CHECK(pose.valid);
    std::set<size_t> ran = StepsRan(B);
    CHECK(ran.count(size_t(viaHop)) && ran.count(size_t(base)));
    CHECK(!ran.count(size_t(follow)));
    CHECK(ConstraintWeight(B, kFollowViaHop) == 0.3f);
    CHECK(ConstraintWeight(B, kFollowBase) == 0.3f);
    CHECK(ConstraintWeight(B, kFollow) == finalWeight);
    SameReadings(FreshPose(stage, rig, t, drag), pose, "hop dragged");
    CheckFrozenJob(evaluator.get(), rig, t, drag, pose, "hop dragged");

    evaluator->SetInteractiveOverrides({});
    pose = evaluator->Evaluate(t);
    CHECK(pose.valid);
    ran = StepsRan(B);
    CHECK(ran.count(size_t(viaHop)) && ran.count(size_t(base)));
    CHECK(!ran.count(size_t(follow)));
    CHECK(ConstraintWeight(B, kFollowViaHop) == finalWeight);
    CHECK(ConstraintWeight(B, kFollowBase) == baseWeight);
    SameReadings(FreshPose(stage, rig, t, {}), pose, "hop lifted");
}

// A double avar connected through FollowBase's float input to the double
// target is a base record of its own, met first. FollowBase's float record
// further along answers no double read: the overlay answers only its exact
// type, so with the avar's record stood aside the walk passes it by and
// reads the target's final version. So the final is shadowed by the avar's
// own record and never by FollowBase's. A drag on revision 2's input leaves
// the avar on the base; a drag on FollowBase's input stands both records
// aside and the avar reads the final, also once revision 2 moves it.
void
TestARecordOfAnotherTypeShadowsNothing()
{
    UsdStageRefPtr stage = StageFrom(kVersionReaders);
    const SdfPath rig("/Asset/Rig");
    const SdfPath probe("/Asset/Rig/Controls/ProbeCtl");
    const SdfPath probeRz = probe.AppendProperty(TfToken("avars:rz"));
    {
        const UsdPrim ctl =
            stage->DefinePrim(probe, TfToken("RigExecControl"));
        ctl.AddAppliedSchema(TfToken("RigExecControlAPI"));
        UsdAttribute rz = ctl.CreateAttribute(TfToken("avars:rz"),
                                              SdfValueTypeNames->Double);
        rz.Set(0.0);
        rz.AddConnection(
            kFollowBase.AppendProperty(TfToken("inputs:defaultWeight")));
        ctl.CreateAttribute(TfToken("rest:space"),
                            SdfValueTypeNames->Matrix4d)
            .Set(GfMatrix4d(1.0));
        const UsdPrim joint = stage->DefinePrim(
            SdfPath("/Asset/Rig/Joints/Probe"), TfToken("RigExecJoint"));
        joint.CreateAttribute(TfToken("rest:space"),
                              SdfValueTypeNames->Matrix4d)
            .Set(GfMatrix4d(1.0));
        const UsdPrim chain = stage->DefinePrim(
            SdfPath("/Asset/Rig/Solvers/ProbeChain"),
            TfToken("RigExecFkChain"));
        chain.CreateRelationship(TfToken("rigExec:controls"))
            .AddTarget(probe);
        chain.CreateRelationship(TfToken("rigExec:joints"))
            .AddTarget(joint.GetPath());
    }
    auto evaluator = MakeEvaluator(stage, rig, RigExecEvaluationMode::Baked);
    const UsdTimeCode t(1.0);
    CHECK(evaluator->Evaluate(t).valid);
    CHECK(evaluator->Evaluate(t).valid);
    const RigExecBakedProgramImpl *program = Program(*evaluator);
    CHECK(program && program->propertyChains.size() == 1);
    if (!program || program->propertyChains.size() != 1) {
        return;
    }
    const RigExecBakedProgramImpl &B = *program;
    const RigExecBakedInput<double> *input = nullptr;
    for (const RigExecBakedProgramImpl::AvarBinding &binding :
         B.avarBindings) {
        if (binding.input.head && binding.input.head.GetPath() == probeRz) {
            input = &binding.input;
        }
    }
    CHECK(input && input->walk >= 0);
    if (!input || input->walk < 0) {
        return;
    }
    // The walk declares the final, and only the avar's own record shadows
    // it.
    const RigExecBakedReaderWalk &walk = B.readerWalks[size_t(input->walk)];
    CHECK(!walk.shadowed.empty());
    for (const auto &[version, record] : walk.shadowed) {
        CHECK(B.propertyRecords[record].consumer == probeRz);
    }
    const auto finalValue = [&B]() {
        const VtValue &v = B.chainFinal.front();
        return v.IsHolding<double>() ? v.UncheckedGet<double>() : -1.0;
    };
    const double base = RigExecBakedLeafRead(B, *input);
    CHECK(base != finalValue());
    const std::vector<RigExecValueOverride> revision2 = DragBy(
        stage, SdfPath("/Asset/Rig/Movers/Rz/Second.inputs:value"), t, 1.0f);
    const std::vector<RigExecValueOverride> hop = {RigExecValueOverride{
        kFollowBase, TfToken(), TfToken("inputs:defaultWeight"),
        VtValue(0.3f)}};
    std::vector<RigExecValueOverride> both = hop;
    both.insert(both.end(), revision2.begin(), revision2.end());
    struct Case {
        const char *what;
        std::vector<RigExecValueOverride> overrides;
        bool readsFinal;
    };
    for (const Case &c : {Case{"revision 2 dragged", revision2, false},
                          Case{"FollowBase's input dragged", hop, true},
                          Case{"both dragged", both, true},
                          Case{"lifted", {}, false}}) {
        const std::string what =
            std::string("a record of another type: ") + c.what;
        evaluator->SetInteractiveOverrides(c.overrides);
        const RigExecRigPose pose = evaluator->Evaluate(t);
        CHECK(pose.valid);
        const double want = c.readsFinal ? finalValue() : base;
        const double got = RigExecBakedLeafRead(B, *input);
        if (got != want) {
            ++failures;
            std::printf("FAIL %s: the avar reads %.9g, expected %.9g\n",
                        what.c_str(), got, want);
        }
        SameReadings(FreshPose(stage, rig, t, c.overrides), pose, what);
        CheckFrozenJob(evaluator.get(), rig, t, c.overrides, pose, what);
    }
}

// Every step that reads a reader walk declares each version the walk can
// meet, on the in-memory readers, computed_chains and the biped; the avar
// slots likewise. With one declaration taken out, the program's region
// validation refuses it and names the walk.
void
TestReaderWalksDeclareTheirVersions(const std::string &examples)
{
    struct Rig {
        std::string name;
        UsdStageRefPtr stage;
        SdfPath rig;
    };
    const std::vector<Fixture> fixtures = Fixtures(examples);
    std::vector<Rig> rigs = {
        {"version readers", StageFrom(kVersionReaders), SdfPath("/Asset/Rig")},
    };
    for (const char *name : {"computed_chains", "biped"}) {
        const Fixture &f = FixtureNamed(fixtures, name);
        rigs.push_back({name, UsdStage::Open(f.stage), f.rig});
    }
    size_t declaring = 0;
    for (const Rig &r : rigs) {
        auto evaluator =
            MakeEvaluator(r.stage, r.rig, RigExecEvaluationMode::Baked);
        const RigExecBakedProgramImpl *program = Program(*evaluator);
        CHECK(program != nullptr);
        if (!program) {
            continue;
        }
        RigExecBakedProgramImpl &B =
            const_cast<RigExecBakedProgramImpl &>(*program);
        std::string error;
        CHECK(RigExecBakedValidateHeadReads(B, &error));
        size_t steps = 0, walks = 0;
        int first = -1;
        for (size_t i = 0; i < B.steps.size(); ++i) {
            const RigExecBakedStep &step = B.steps[i];
            if (step.readerWalks.empty()) {
                continue;
            }
            ++steps;
            for (const int walk : step.readerWalks) {
                ++walks;
                for (const uint32_t id :
                     B.readerWalks[size_t(walk)].versions) {
                    bool declared = false;
                    for (const RigExecBakedSlotRange &range : step.reads) {
                        declared = declared ||
                                   (range.domain == RigExecBakedSlotDomain::PropertyResult &&
                                    range.begin <= id && id < range.end);
                    }
                    CHECK(declared);
                }
            }
            if (first < 0 &&
                (step.kind == RigExecBakedStepKind::Solve ||
                 step.kind == RigExecBakedStepKind::Constraint)) {
                first = int(i);
            }
        }
        std::printf("reader walks %s: %zu step(s) read %zu walk(s), %zu "
                    "reader walk(s) in all\n",
                    r.name.c_str(), steps, walks, B.readerWalks.size());
        declaring += steps;
        if (first < 0) {
            continue;
        }
        // Remove only property declarations, retaining required Rest/Ladder
        // inputs so the refusal still names the undeclared reader walk.
        RigExecBakedStep &step = B.steps[size_t(first)];
        const std::vector<RigExecBakedSlotRange> saved = step.reads;
        step.reads.erase(std::remove_if(step.reads.begin(), step.reads.end(),
            [](const auto &range) {
                return range.domain == RigExecBakedSlotDomain::PropertyResult;
            }), step.reads.end());
        CHECK(step.reads.size() < saved.size());
        error.clear();
        CHECK(!RigExecBakedValidateHeadReads(B, &error));
        CHECK(error.rfind("walk ", 0) == 0 &&
              error.find(" without declaring it") != std::string::npos);
        std::printf("reader walks %s: undeclared: %s\n", r.name.c_str(),
                    error.c_str());
        step.reads = saved;
        CHECK(RigExecBakedValidateHeadReads(B, &error));
    }
    CHECK(declaring > 0);
}

// A drag on a chain target a few steps read, at a held frame on the biped:
// the cone run executes strictly fewer non-source region steps than a
// forced run of the same frame and drag, and skips some of the steps that
// read the generation's resolved inputs (what a rule keyed on any moved
// chain result would dirty); both publish the same pose.
void
TestAChainDragIsNoLongerAWholeRigCost(const std::string &examples)
{
    const std::vector<Fixture> fixtures = Fixtures(examples);
    const Fixture &f = FixtureNamed(fixtures, "biped");
    UsdStageRefPtr stage = UsdStage::Open(f.stage);
    auto evaluator = MakeEvaluator(stage, f.rig, RigExecEvaluationMode::Baked);
    const UsdTimeCode t(stage->GetStartTimeCode() + 2.0);
    CHECK(evaluator->Evaluate(t).valid);
    CHECK(evaluator->Evaluate(t).valid);
    const RigExecBakedProgramImpl *program = Program(*evaluator);
    CHECK(program != nullptr);
    if (!program) {
        return;
    }
    const RigExecBakedProgramImpl &B = *program;
    // The chain whose final version the fewest steps read, among those read.
    SdfPath target;
    size_t readers = 0;
    for (const RigExecBakedPropertyChain &chain : B.propertyChains) {
        const uint32_t last =
            chain.versionBase + uint32_t(chain.revisions.size());
        const size_t n = last < B.cones.headReaders.size()
                             ? B.cones.headReaders[last].size()
                             : 0;
        if (n > 0 && chain.targetExists &&
            (chain.arm == RigExecBakedPropertyChain::Arm::Float ||
             chain.arm == RigExecBakedPropertyChain::Arm::Double) &&
            (target.IsEmpty() || n < readers)) {
            target = chain.target;
            readers = n;
        }
    }
    CHECK(!target.IsEmpty());
    if (target.IsEmpty()) {
        return;
    }
    const auto resolved = RigExecBakedResolvedReaders(B);
    const size_t resolvedReaders = std::count(resolved.begin(), resolved.end(), char(1));
    const std::vector<RigExecValueOverride> drag =
        DragBy(stage, target, t, 0.25f);
    CHECK(!drag.empty());
    const auto nonSource = [&B]() {
        size_t n = 0;
        for (const size_t step : StepsRan(B)) {
            n += B.steps[step].isSource ? 0 : 1;
        }
        return n;
    };
    evaluator->SetInteractiveOverrides(drag);
    const RigExecRigPose cone = evaluator->Evaluate(t);
    const size_t coneSteps = nonSource();
    // The steps the old rule re-ran on any moved chain result.
    size_t skippedChainReaders = 0;
    {
        const std::set<size_t> ran = StepsRan(B);
        for (const int index : B.cones.varyingSteps) {
            const RigExecBakedStep &step = B.steps[size_t(index)];
            if (resolved[size_t(index)] && !step.isSource &&
                !step.externalReads && !ran.count(size_t(index))) {
                ++skippedChainReaders;
            }
        }
    }
    evaluator->GetBakedProgram()->RequestFullRun();
    const RigExecRigPose forced = evaluator->Evaluate(t);
    const size_t forcedSteps = nonSource();
    std::printf("chain drag on %s (%zu declared reader(s)): the cone ran "
                "%zu non-source step(s), a forced run %zu; %zu of %zu "
                "step(s) reading resolved inputs skipped\n",
                target.GetText(), readers, coneSteps, forcedSteps,
                skippedChainReaders, resolvedReaders);
    CHECK(coneSteps < forcedSteps);
    CHECK(skippedChainReaders > 0);
    SameReadings(forced, cone, "chain drag: cone against forced");
}

// Exact build-only binding associations; geometry path walks alone must
// not broaden the historical time-seed classification.
void TestResolvedReadersAreBuildOnlyBindings()
{
    auto stage = UsdStage::CreateInMemory();
    const auto prim = stage->DefinePrim(SdfPath("/Inputs"));
    const auto scalar = prim.CreateAttribute(TfToken("value"),SdfValueTypeNames->Double);
    const auto points = prim.CreateAttribute(TfToken("points"),SdfValueTypeNames->Point3fArray);
    RigExecBakedProgramImpl B;
    B.solvers.resize(2);
    B.solvers[0].upperOffset.resolvedAttr = scalar;
    B.solvers[1].upperOffset.varying = true;
    B.weightObjects.resize(1);
    B.weightObjects[0].samplePoints.push_back(points);
    const auto step = [&](RigExecBakedStepKind kind,int object) {
        RigExecBakedStep s; s.kind=kind; s.object=object; s.cluster=0;
        B.steps.push_back(std::move(s));
    };
    step(RigExecBakedStepKind::Solve,0);
    step(RigExecBakedStepKind::RevisionStatic,-1);
    B.steps.back().readerWalks={0};
    step(RigExecBakedStepKind::WeightPacket,0);
    step(RigExecBakedStepKind::Solve,1);
    B.steps.back().varyingInputs=true;
    CHECK((RigExecBakedResolvedReaders(B)==std::vector<char>{1,0,1,0}));
    B.steps[0].isHead=true;
    CHECK((RigExecBakedResolvedReaders(B)==std::vector<char>{0,0,1,0}));
    // Classification is read-only: it does not mutate declarations.
    CHECK(!B.steps[0].varyingInputs && B.steps[3].varyingInputs);
    CHECK((B.steps[1].readerWalks==std::vector<int>{0}));
}

// Pin semantic identities and region-relative indices across the promotion,
// including all source/always members. Expected values come from the native
// graph before promotion, with only numeric interpolator/Derived removals.
void
TestTheSourceAndAlwaysSetsSurviveThePromotion(const std::string &examples)
{
    size_t checked=0, totalRegion=0, totalSources=0, totalAlways=0, totalVarying=0;
    constexpr uint64_t initial=14695981039346656037ull;
    for (const auto &expected : rigExecTest::rigExecPromotionCases) {
        auto stage=UsdStage::Open(examples+"/../"+expected.fixture);
        CHECK(stage); if (!stage) continue;
        RigExecRigEvaluator evaluator(stage,SdfPath(expected.rig));
        std::vector<std::string> errors;
        CHECK(evaluator.Compile(&errors));
        evaluator.SetEvaluationMode(RigExecEvaluationMode::Baked);
        const auto *program=Program(evaluator);
        CHECK(program); if (!program) continue;
        const auto &B=*program;
        size_t H=0;
        while (H<B.steps.size() && B.steps[H].isHead) ++H;
        size_t sources=0, always=0;
        uint64_t headIdentity=initial, regionIdentity=initial,
            sourceIdentity=initial, alwaysIdentity=initial, varyingIdentity=initial;
        const auto hash=[](uint64_t *value,size_t index,const RigExecBakedStep &step) {
            const std::string text=std::to_string(index)+"|"+
                RigExecBakedStepKindName(step.kind)+"|"+std::to_string(step.object)+
                "|"+std::to_string(step.part)+"|"+step.label+"\n";
            for (const unsigned char c:text) *value=(*value^uint64_t(c))*1099511628211ull;
        };
        for (size_t i=0;i<B.steps.size();++i) {
            const auto &step=B.steps[i];
            CHECK(step.isHead==(i<H));
            CHECK(B.cones.alwaysSteps.Test(int(i))==step.externalReads);
            if (i<H) {
                CHECK(!step.isSource && !step.externalReads);
                hash(&headIdentity,i,step);
            } else {
                hash(&regionIdentity,i-H,step);
                if (step.isSource) { ++sources; hash(&sourceIdentity,i-H,step); }
                if (step.externalReads) { ++always; hash(&alwaysIdentity,i-H,step); }
            }
        }
        std::set<int> varyingSeen;
        for (const int index : B.cones.varyingSteps) {
            CHECK(index >= 0 && size_t(index) < B.steps.size());
            if (index < 0 || size_t(index) >= B.steps.size()) continue;
            CHECK(size_t(index) >= H && !B.steps[size_t(index)].isHead);
            CHECK(varyingSeen.insert(index).second);
            if (size_t(index) >= H) hash(&varyingIdentity,size_t(index)-H,B.steps[size_t(index)]);
        }
        CHECK(B.cones.varyingSteps.size()==expected.varying);
        CHECK(varyingIdentity==expected.varyingIdentity);
        totalVarying+=B.cones.varyingSteps.size();
        const bool same=H==expected.heads && B.steps.size()-H==expected.region &&
            sources==expected.sources && always==expected.always &&
            headIdentity==expected.headIdentity && regionIdentity==expected.regionIdentity &&
            sourceIdentity==expected.sourceIdentity && alwaysIdentity==expected.alwaysIdentity;
        CHECK(same);
        if (!same) std::printf("promotion %s %s: H%zu region%zu source%zu always%zu hashes %016llx %016llx %016llx %016llx\n",
            expected.fixture,expected.rig,H,B.steps.size()-H,sources,always,
            static_cast<unsigned long long>(headIdentity),static_cast<unsigned long long>(regionIdentity),
            static_cast<unsigned long long>(sourceIdentity),static_cast<unsigned long long>(alwaysIdentity));
        ++checked; totalRegion+=B.steps.size()-H; totalSources+=sources; totalAlways+=always;
    }
    CHECK(checked==47);
    CHECK(totalRegion==7154 && totalSources==345 && totalAlways==375);
    CHECK(totalVarying==140);
}

}  // namespace

int
main(int argc, char **argv)
{
    if (argc < 2) {
        std::printf("usage: testRigExecPropertyOps <examples dir>\n");
        return 2;
    }
    PlugRegistry::GetInstance().RegisterPlugins(
        TfAbsPath(RIGEXEC_SCHEMA_RESOURCE_DIR));
    const std::string examples = argv[1];
    TestPropertyOpsEqualTheEvaluatorChains(examples);
    TestPerVersionMemo();
    TestATargetRetypedInPlaceRebinds();
    TestHeadOrderIsChainOrder();
    TestTheStageFramesBailStillPublishesChains(examples);
    TestACleanHeadTierStillPublishes(examples);
    TestABakeAfterAnEvaluationExportsTheChains(examples);
    TestAChainReadsAnInterpolatorWeightAsAuthored();
    TestOnlyReadersOfTheChangedVersionRerun();
    TestAStandAsideReaderDeclaresItsHops();
    TestARecordOfAnotherTypeShadowsNothing();
    TestReaderWalksDeclareTheirVersions(examples);
    TestAChainDragIsNoLongerAWholeRigCost(examples);
    TestResolvedReadersAreBuildOnlyBindings();
    TestTheSourceAndAlwaysSetsSurviveThePromotion(examples);
    std::printf("testRigExecPropertyOps: %d failure(s)\n", failures);
    return failures ? 1 : 0;
}
