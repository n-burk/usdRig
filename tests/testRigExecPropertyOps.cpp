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
// Registered plain and under the parity entries; under
// RIGEXEC_BAKED_VERIFY_CONES the head tier is also checked against a forced
// run of itself.
// argv[1] = path to the examples directory.
#include "rigExec/bakedProgram.h"
#include "rigExec/bakedProgramImpl.h"
#include "rigExec/bakedTrace.h"
#include "rigExec/rigEvaluator.h"
#include "rigExec/rigEvaluatorPropertyBindings.h"
#include "rigExecBake/bake.h"
#include "rigExecMath/propertyMath.h"

#include "pxr/base/gf/math.h"
#include "pxr/base/gf/quatf.h"
#include "pxr/base/gf/rotation.h"
#include "pxr/base/gf/vec3f.h"
#include "pxr/base/plug/registry.h"
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

// The head tier's lines, in head order: what the run put first in the
// generation's diagnostics.
std::vector<std::string>
HeadLines(const RigExecBakedProgramImpl &B)
{
    std::vector<std::string> lines;
    for (const uint32_t index : B.headOrder) {
        const RigExecBakedHeadStep &step = B.headSteps[index];
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
                    program->headSteps.size(), compared, dynamic,
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
    for (const RigExecOpTraceEntry &entry : RigExecBakedLastHeadTrace(B)) {
        const RigExecBakedHeadStep &step = B.headSteps[entry.step];
        ran.emplace(B.propertyChains[size_t(step.object)].target, step.part);
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
    CHECK(Ran(B).size() == B.headSteps.size());
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
    bool ordered = B.headOrder.size() == B.headSteps.size();
    for (const uint32_t index : B.headOrder) {
        const RigExecBakedHeadStep &step = B.headSteps[index];
        ordered = ordered && (step.object > lastChain ||
                              (step.object == lastChain &&
                               step.part == lastPart + 1));
        lastChain = step.object;
        lastPart = step.part;
    }
    CHECK(ordered);
    // No step of one chain reads the other's.
    for (const RigExecBakedHeadStep &step : B.headSteps) {
        for (const uint32_t pred : step.preds) {
            CHECK(B.headSteps[pred].object == step.object);
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
        for (const RigExecOpTraceEntry &entry : RigExecBakedLastHeadTrace(B)) {
            if (B.headSteps[entry.step].alwaysRuns) {
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
            CHECK(RigExecBakedLastHeadTrace(standing->GetStepGraph()).size() ==
                  standing->GetStepGraph().headSteps.size());
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
            CHECK(RigExecBakedLastHeadTrace(*program).size() ==
                  program->headSteps.size());
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
    std::printf("testRigExecPropertyOps: %d failure(s)\n", failures);
    return failures ? 1 : 0;
}
