// Source leaves are sampled on the owning thread. Produced opinions are
// consumed from their declared versions when the owning graph operation runs.
// These cases check exact leaf numbering, types, source hops and override
// admission under edits, lifts and held frames, alongside numerical and
// body-purity checks. Every registered example participates.
// argv[1] = path to the examples directory.
#include "rigExec/inputReplay.h"
#include "rigExec/bakedOpValues.h"
#include "rigExec/bakedProgram.h"
#include "rigExec/bakedProgramImpl.h"
#include "rigExec/bodyPurity.h"
#include "rigExec/frozenContextInternal.h"
#include "rigExec/rigEvaluator.h"
#include "rigExecBake/bake.h"
#include "rigExecBake/revisionReads.h"
#include "rigExecBinary/format.h"
#include "rigExecBinary/generated/rigexec_generated.h"
#include "rigExecRigging/rigBuilder.h"
#include "rigExecExampleFixtures.h"

#include "pxr/base/arch/env.h"
#include "pxr/base/gf/matrix4d.h"
#include "pxr/base/gf/vec2f.h"
#include "pxr/base/gf/vec3d.h"
#include "pxr/base/gf/vec3f.h"
#include "pxr/base/plug/registry.h"
#include "pxr/base/tf/errorMark.h"
#include "pxr/base/tf/pathUtils.h"
#include "pxr/base/tf/stringUtils.h"
#include "pxr/base/tf/type.h"
#include "pxr/base/ts/knot.h"
#include "pxr/base/ts/spline.h"
#include "pxr/base/vt/array.h"
#include "pxr/usd/sdf/attributeSpec.h"
#include "pxr/usd/sdf/changeBlock.h"
#include "pxr/usd/sdf/layer.h"
#include "pxr/usd/usd/attribute.h"
#include "pxr/usd/usd/prim.h"
#include "pxr/usd/usd/primRange.h"
#include "pxr/usd/usd/stage.h"
#include "pxr/usd/usd/variantSets.h"

#include <algorithm>
#include <atomic>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <functional>
#include <map>
#include <memory>
#include <set>
#include <string>
#include <thread>
#include <type_traits>
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
        {"11", examples + "/11_VolumeWeights.usda",
         SdfPath("/VolumeAsset/Rig")},
        {"16", examples + "/16_ConnectionReadPhases.usda",
         SdfPath("/PhaseConnectAsset/Rig")},
        {"biped", examples + "/biped/Biped_anim.usda", SdfPath("/Biped/Rig")},
        {"computed_chains",
         examples + "/../tests/fixtures/computed_chains.usda",
         SdfPath("/Asset/Rig")},
    };
}

void TestLayeredSourceOverlay() {
    const SdfPath a("/Source.a"), b("/Source.b");
    RigExecResolvedInputs source, consumer;
    source.SetProperty(a,VtValue(1)); source.SetProperty(b,VtValue(VtFloatArray{2.0f}));
    consumer.SetChainedBase(&source);
    CHECK(consumer.GetSize()==2);
    int value=0;CHECK(consumer.Get(a,&value) && value==1);
    consumer.SetProperty(a,VtValue(3));
    CHECK(consumer.Get(a,&value) && value==3);
    CHECK(source.Get(a,&value) && value==1);
    consumer.ClearProperty(b);CHECK(!consumer.Find(b));
    const RigExecResolvedInputs detached=consumer.DetachedCopy();
    source.Clear();consumer.Clear();
    CHECK(detached.Get(a,&value) && value==3);
    CHECK(!detached.Find(b));CHECK(detached.GetSize()==1);
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

// Bit for bit, as the sampler's `changed` byte compares.
template <class T>
bool
Same(const T &a, const T &b)
{
    if constexpr (std::is_trivially_copyable_v<T>) {
        return std::memcmp(&a, &b, sizeof(T)) == 0;
    } else {
        return a == b;
    }
}

template <class T>
std::string
Text(const T &value)
{
    if constexpr (std::is_same_v<T, double> || std::is_same_v<T, float>) {
        char text[64];
        std::snprintf(text, sizeof(text), "%.17g", double(value));
        return text;
    } else if constexpr (std::is_same_v<T, int> || std::is_same_v<T, bool>) {
        return std::to_string(int(value));
    } else if constexpr (std::is_same_v<T, TfToken>) {
        return value.GetString();
    } else {
        return "(value)";
    }
}

const RigExecBakedProgramImpl *
Program(const RigExecRigEvaluator &evaluator)
{
    const RigExecBakedProgram *program = evaluator.GetBakedProgram();
    return program ? &program->GetStepGraph() : nullptr;
}

// Every visited binding's leaf against RigExecBakedRead evaluated again now,
// over the generation's resolved inputs as the prologue left them. The
// epilogue publishes the pose weights into those inputs after the region,
// so their paths are put back to what the prologue placed there: the chain
// result, else the override, else nothing.
size_t
LeafMismatches(const RigExecRigEvaluator &evaluator,
               const std::vector<RigExecValueOverride> &overrides,
               UsdTimeCode time, const std::string &what)
{
    const RigExecBakedProgramImpl *program = Program(evaluator);
    CHECK(program);
    if (!program) {
        return 1;
    }
    const RigExecBakedProgramImpl &B = *program;
    RigExecResolvedInputs R = *B.resolvedInputs;
    for (const SdfPath &path : B.poseWeightPaths) {
        R.ClearProperty(path);
        for (const RigExecValueOverride &o : overrides) {
            if (!o.attribute.IsEmpty() &&
                o.prim.AppendProperty(o.attribute) == path) {
                R.SetProperty(path, o.value);
            }
        }
        const auto found = B.propertyResults.find(path);
        if (found != B.propertyResults.end()) {
            R.SetProperty(path, found->second);
        }
    }
    size_t mismatches = 0;
    frozenDetail::_ForEachPatchableInput(B, [&](const auto &input) {
        using T = std::decay_t<decltype(input.constant)>;
        if (input.leaf < 0) {
            ++mismatches;
            std::printf("FAIL %s: a visited binding at %s has no leaf\n",
                        what.c_str(),
                        input.head ? input.head.GetPath().GetText() : "-");
            return;
        }
        // Walked bindings consume their declared current typed versions;
        // their pooled leaf remains the independently captured source value.
        const T leaf = RigExecBakedLeafRead(B, input);
        const T read = RigExecBakedRead(input, R, time, &B.overridden);
        if (!Same(leaf, read)) {
            if (mismatches < 6) {
                std::printf("FAIL %s: leaf %s, read %s at %s\n", what.c_str(),
                            Text(leaf).c_str(), Text(read).c_str(),
                            input.head ? input.head.GetPath().GetText()
                                       : "-");
            }
            ++mismatches;
        }
    });
    return mismatches;
}

std::unique_ptr<RigExecRigEvaluator>
MakeEvaluator(const UsdStageRefPtr &stage, const SdfPath &rig)
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

// A freshly compiled epoch at \p time under \p overrides.
RigExecRigPose
FreshOverridePose(const UsdStageRefPtr &stage, const SdfPath &rig, UsdTimeCode time,
          const std::vector<RigExecValueOverride> &overrides)
{
    auto reference =
        MakeEvaluator(stage, rig);
    reference->SetInteractiveOverrides(overrides);
    return reference->Evaluate(time);
}

// A program built fresh on \p stage: its first generation runs everything.
RigExecRigPose
FreshPose(const UsdStageRefPtr &stage, const SdfPath &rig, UsdTimeCode time)
{
    auto fresh = MakeEvaluator(stage, rig);
    const RigExecRigPose pose = fresh->Evaluate(time);
    CHECK(fresh->GetBakedGenerationCount() == 1);
    return pose;
}

size_t
PoseMismatches(const RigExecRigPose &reference, RigExecRigPose pose,
               const std::string &what, bool quiet = false)
{
    if (!reference.valid || !pose.valid) {
        if (!quiet) {
            std::printf("FAIL %s: an invalid pose\n", what.c_str());
        }
        return 1;
    }
    RigExecRigPose fresh = reference;
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
    if (diff.comparisonMismatches != 0 && !quiet) {
        std::printf("FAIL %s: %zu mismatch(es):\n", what.c_str(),
                    diff.comparisonMismatches);
        for (size_t i = 0; i < diff.diagnostics.size() && i < 8; ++i) {
            std::printf("    %s\n", diff.diagnostics[i].c_str());
        }
    }
    return diff.comparisonMismatches;
}

// One evaluation of the program under \p overrides at \p time, checked:
// a valid generation from the program, the cone verifier (when asked for)
// silent, and every leaf equal to the read it replaced.
RigExecRigPose
RunChecked(RigExecRigEvaluator *evaluator,
           const std::vector<RigExecValueOverride> &overrides,
           UsdTimeCode time, const std::string &what)
{
    evaluator->SetInteractiveOverrides(overrides);
    const size_t generations = evaluator->GetBakedGenerationCount();
    const RigExecRigPose pose = evaluator->Evaluate(time);
    CHECK(pose.valid);
    CHECK(pose.comparisonMismatches == 0);
    CHECK(evaluator->GetBakedGenerationCount() == generations + 1);
    const size_t mismatches = LeafMismatches(*evaluator, overrides, time,
                                             what);
    if (mismatches) {
        std::printf("FAIL %s: %zu leaf/read mismatch(es)\n", what.c_str(),
                    mismatches);
    }
    CHECK(mismatches == 0);
    return pose;
}

template <class T>
RigExecValueOverride
DragOf(const SdfPath &property, T value)
{
    return RigExecValueOverride{property.GetPrimPath(), TfToken(),
                                property.GetNameToken(), VtValue(value)};
}

// A drag on \p property by \p delta from its value at \p time, typed as the
// attribute is; empty unless it is a float or a double.
std::vector<RigExecValueOverride>
DragBy(const UsdStageRefPtr &stage, const SdfPath &property, UsdTimeCode time,
       double delta)
{
    const UsdAttribute attribute = stage->GetAttributeAtPath(property);
    if (!attribute) {
        return {};
    }
    const TfType type = attribute.GetTypeName().GetType();
    if (type == TfType::Find<double>()) {
        double v = 0.0;
        attribute.Get(&v, time);
        return {DragOf(property, v + delta)};
    }
    if (type == TfType::Find<float>()) {
        float v = 0.0f;
        attribute.Get(&v, time);
        return {DragOf(property, float(v + float(delta)))};
    }
    return {};
}

// Solver and constraint inputs a drag can stand on: a scalar head of its
// own type, with no chain on the walk.
std::vector<SdfPath>
SolverInputs(const RigExecBakedProgramImpl &B)
{
    std::vector<SdfPath> out;
    const auto consider = [&out](const auto &input) {
        using T = std::decay_t<decltype(input.constant)>;
        if constexpr (std::is_same_v<T, double> || std::is_same_v<T, float>) {
            if (input.head && !input.resolvedAttr &&
                input.overrideIndex >= 0 &&
                input.head.GetTypeName().GetType() == TfType::Find<T>()) {
                out.push_back(input.head.GetPath());
            }
        }
    };
    for (const auto &solver : B.solvers) {
        frozenDetail::_VisitSolverInputs(solver, consider);
    }
    for (const auto &constraint : B.constraints) {
        frozenDetail::_VisitConstraintInputs(constraint, consider);
    }
    return out;
}

// The hop of a chain-routed binding's walk that a chain writes: a drag on
// it is the chain's base, and it moves the reader through the chain.
std::vector<SdfPath>
ChainHops(const RigExecBakedProgramImpl &B)
{
    std::vector<SdfPath> out;
    frozenDetail::_ForEachPatchableInput(B, [&](const auto &input) {
        using T = std::decay_t<decltype(input.constant)>;
        if (!input.head || !input.resolvedAttr) {
            return;
        }
        bool viaChain = false, varying = false;
        UsdAttribute selected;
        SdfPathVector walk;
        RigExecBakedClassifyInput<T>(input.head, UsdTimeCode::Default(),
                                     B.chainTargets, &viaChain, &varying,
                                     &selected, &walk);
        for (const SdfPath &hop : walk) {
            if (B.chainTargets.count(hop)) {
                out.push_back(hop);
            }
        }
    });
    return out;
}

// The first input of each other visited family a drag can stand on: a
// scalar head of the binding's own type, numbered, not folded, with no chain
// on the walk. Varying and constant avars read through the input fill, the
// ladder through the compose, the rest through step bodies.
std::vector<std::pair<std::string, SdfPath>>
FamilyInputs(const RigExecBakedProgramImpl &B)
{
    std::vector<std::pair<std::string, SdfPath>> out;
    const auto first = [&](const char *kind, const auto &visit) {
        SdfPath found;
        visit([&](const auto &input) {
            using T = std::decay_t<decltype(input.constant)>;
            if constexpr (std::is_same_v<T, double> ||
                          std::is_same_v<T, float>) {
                if (found.IsEmpty() && input.head && !input.resolvedAttr &&
                    input.overrideIndex >= 0 &&
                    input.head.GetTypeName().GetType() ==
                        TfType::Find<T>() &&
                    !B.folded.count(input.head.GetPath())) {
                    found = input.head.GetPath();
                }
            }
        });
        if (!found.IsEmpty()) {
            out.emplace_back(kind, found);
        }
    };
    first("varying avar", [&](const auto &fn) {
        for (const auto &binding : B.avarBindings) {
            fn(binding.input);
        }
    });
    first("constant avar", [&](const auto &fn) {
        for (const auto &binding : B.avarConstantBindings) {
            fn(binding.input);
        }
    });
    first("ladder channel", [&](const auto &fn) {
        for (const auto &ladder : B.ladders) {
            frozenDetail::_VisitLadderInputs(ladder, fn);
        }
    });
    first("space switch", [&](const auto &fn) {
        for (const auto &spaceSwitch : B.spaceSwitches) {
            frozenDetail::_VisitSpaceSwitchInputs(spaceSwitch, fn);
        }
    });
    first("interpolator", [&](const auto &fn) {
        for (const auto &interpolator : B.poseInterpolators) {
            frozenDetail::_VisitInterpolatorInputs(interpolator, fn);
        }
    });
    first("weight object", [&](const auto &fn) {
        for (const auto &object : B.weightObjects) {
            frozenDetail::_VisitWeightInputs(object, fn);
        }
    });
    return out;
}

std::vector<UsdTimeCode>
Frames(const UsdStageRefPtr &stage)
{
    const double start = stage->GetStartTimeCode();
    return {UsdTimeCode(start), UsdTimeCode(start + 2.0),
            UsdTimeCode(start + 6.0)};
}

// A double chain on a control's avars:rz read by a float constraint weight
// at `final`: the reader's walk is a float read of a double hop, which
// RigExecResolvedInputs::GetAttribute answers as the double read, cast.
const char *const kFloatAtDouble = R"usda(#usda 1.0
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

    def RigExecRoot "Rig"
    {
        def Scope "Controls"
        {
            def RigExecControl "RootCtl" (
                prepend apiSchemas = ["RigExecControlAPI"]
            )
            {
                double avars:rz.timeSamples = {
                    1: 0.2,
                    10: 0.4,
                }
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
            def RigExecFloatMathMover "Gain" (
                prepend apiSchemas = ["RigExecMoverAPI"]
            )
            {
                uniform token rigExec:operation = "multiply"
                float inputs:value = 2
                rel rigExec:moves = </Asset/Rig/Controls/RootCtl.avars:rz>
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
        }
    }
}
)usda";

template <class T>
size_t
ReaderWalkMismatch(const RigExecBakedProgramImpl &B,
                   const RigExecResolvedInputs &R, const UsdAttribute &head,
                   int walk, UsdTimeCode time, const std::string &what)
{
    T walked{}, read{};
    const bool a = RigExecBakedResolveReaderWalk(B, walk, &walked);
    const bool b = R.GetAttribute(head, time, &read);
    if (a == b && (!a || Same(walked, read))) {
        return 0;
    }
    std::printf("FAIL %s: walk %s answers %s%s, GetAttribute %s%s\n",
                what.c_str(), head.GetPath().GetText(), a ? "" : "nothing ",
                a ? Text(walked).c_str() : "", b ? "" : "nothing ",
                b ? Text(read).c_str() : "");
    return 1;
}

// Every reader walk against RigExecResolvedInputs::GetAttribute from its
// head over the overlay as the prologue left it (see LeafMismatches): the
// answer and the value, bit for bit.
size_t
ReaderWalkMismatches(const RigExecRigEvaluator &evaluator,
                     const std::vector<RigExecValueOverride> &overrides,
                     UsdTimeCode time, const std::string &what,
                     size_t *checked)
{
    const RigExecBakedProgramImpl *program = Program(evaluator);
    if (!program) {
        return 1;
    }
    const RigExecBakedProgramImpl &B = *program;
    RigExecResolvedInputs R = *B.resolvedInputs;
    for (const SdfPath &path : B.poseWeightPaths) {
        R.ClearProperty(path);
        for (const RigExecValueOverride &o : overrides) {
            if (!o.attribute.IsEmpty() &&
                o.prim.AppendProperty(o.attribute) == path) {
                R.SetProperty(path, o.value);
            }
        }
        const auto found = B.propertyResults.find(path);
        if (found != B.propertyResults.end()) {
            R.SetProperty(path, found->second);
        }
    }
    size_t mismatches = 0;
    for (size_t w = 0; w < B.readerWalks.size(); ++w) {
        const RigExecBakedReaderWalk &reader = B.readerWalks[w];
        const UsdAttribute head = B.stage->GetAttributeAtPath(reader.head);
        const int walk = int(w);
        using Type = RigExecBakedHeadValueType;
        switch (reader.walk.type) {
        case Type::Double:
            mismatches +=
                ReaderWalkMismatch<double>(B, R, head, walk, time, what);
            break;
        case Type::Float:
            mismatches +=
                ReaderWalkMismatch<float>(B, R, head, walk, time, what);
            break;
        case Type::Int:
            mismatches += ReaderWalkMismatch<int>(B, R, head, walk, time, what);
            break;
        case Type::Bool:
            mismatches +=
                ReaderWalkMismatch<bool>(B, R, head, walk, time, what);
            break;
        case Type::Token:
            mismatches +=
                ReaderWalkMismatch<TfToken>(B, R, head, walk, time, what);
            break;
        case Type::Matrix4d:
            mismatches +=
                ReaderWalkMismatch<GfMatrix4d>(B, R, head, walk, time, what);
            break;
        case Type::Vec3d:
            mismatches +=
                ReaderWalkMismatch<GfVec3d>(B, R, head, walk, time, what);
            break;
        case Type::Vec3f:
            mismatches +=
                ReaderWalkMismatch<GfVec3f>(B, R, head, walk, time, what);
            break;
        case Type::Vec2fArray:
            ++mismatches;
            std::printf("FAIL %s: reader walk %s reads an array\n",
                        what.c_str(), reader.head.GetText());
            break;
        }
        ++*checked;
    }
    return mismatches;
}

// Provider conversion retains identity for unavailable finals, including
// valid-flag NaNs. A usable final with an unavailable rest still takes the
// existing base/delta fallback, and a later usable final recovers normally.
void
TestProviderMatrixFallbackAdmission()
{
    RigExecBakedProgramImpl program;
    const RigExecPointFrame identity;
    const auto landmarks = identity.points;
    program.restFrames = {identity};
    program.restPts = {landmarks};
    program.base = {identity};
    program.fin = {identity};
    program.baseLast = {0};
    program.finLast = {0};
    program.baseMatrix = {GfMatrix4d(1.0)};
    program.finalMatrix = {GfMatrix4d(1.0)};
    program.jointSlots = {0};
    RigExecBakedStep step;
    step.kind = RigExecBakedStepKind::ProviderMatrix;
    step.object = 0;
    step.part = 1;
    const auto run = [&](const RigExecPointFrame &final) {
        program.fin[0] = final;
        // Poison the previous output to check publication, including recovery.
        program.finalMatrix[0] = GfMatrix4d(7.0);
        RigExecBakedRunPoseStep(&program, &step, UsdTimeCode(1.0));
        return program.finalMatrix[0];
    };
    RigExecPointFrame translated = identity;
    for (auto &point : translated.points) point += GfVec3d(2.0, 0.0, 0.0);
    GfMatrix4d expected(1.0);
    expected.SetTranslateOnly(GfVec3d(2.0, 0.0, 0.0));
    CHECK(run(translated) == expected);
    RigExecPointFrame unavailable = translated;
    unavailable.flags = 0;
    CHECK(run(unavailable) == GfMatrix4d(1.0));
    unavailable = translated;
    unavailable.flags |= RigExecPointFrameDegenerate;
    CHECK(run(unavailable) == GfMatrix4d(1.0));
    for (const double bad : {std::nan(""), HUGE_VAL}) {
        unavailable = translated;
        unavailable.points[0][0] = bad;
        CHECK(run(unavailable) == GfMatrix4d(1.0));
    }
    // A degenerate rest does not reject a still-usable final fallback. Keep
    // the captured reference basis and Base, as the seeded-provider path does.
    program.restFrames[0].flags |= RigExecPointFrameDegenerate;
    CHECK(run(translated) == expected);
    program.restFrames[0] = identity;
    CHECK(run(translated) == expected);
}

// The walk resolver against the read it restates, over every reader walk
// (chain-routed bindings and path leaves read through the resolved inputs)
// of every fixture, at three frames: with no override, under a drag on a
// chain target the walks meet and on a NaN there (which skips the chain,
// leaving the override standing at its target), and lifted. The in-memory
// rig adds a float read of a double chain target.
void
TestWalkResolverEqualsGetAttribute(const std::string &examples)
{
    std::vector<std::pair<std::string, UsdStageRefPtr>> stages;
    std::vector<SdfPath> rigs;
    for (const Fixture &f : Fixtures(examples)) {
        stages.emplace_back(f.name, UsdStage::Open(f.stage));
        rigs.push_back(f.rig);
    }
    {
        UsdStageRefPtr stage = UsdStage::CreateInMemory();
        CHECK(rigExec::RigExecInputReplayImportFromString(stage->GetRootLayer(), kFloatAtDouble));
        stages.emplace_back("float at double", stage);
        rigs.push_back(SdfPath("/Asset/Rig"));
    }
    size_t checked = 0, doubleTails = 0, skipped = 0;
    for (size_t i = 0; i < stages.size(); ++i) {
        const std::string &name = stages[i].first;
        const UsdStageRefPtr &stage = stages[i].second;
        auto evaluator =
            MakeEvaluator(stage, rigs[i]);
        const RigExecBakedProgramImpl *program = Program(*evaluator);
        CHECK(program != nullptr);
        if (!program) {
            continue;
        }
        // A chain target some walk meets.
        SdfPath target;
        for (const RigExecBakedReaderWalk &reader : program->readerWalks) {
            doubleTails += reader.walk.doubleHops.empty() ? 0 : 1;
            for (const auto *hops :
                 {&reader.walk.hops, &reader.walk.doubleHops}) {
                for (const RigExecBakedWalkHop &hop : *hops) {
                    if (hop.chain >= 0 && target.IsEmpty()) {
                        target = program->propertyChains[size_t(hop.chain)]
                                     .target;
                    }
                }
            }
        }
        for (const UsdTimeCode t : Frames(stage)) {
            std::vector<std::vector<RigExecValueOverride>> cases = {{}};
            if (!target.IsEmpty()) {
                cases.push_back(DragBy(stage, target, t, 0.25));
                const UsdAttribute a = stage->GetAttributeAtPath(target);
                if (a.GetTypeName() == SdfValueTypeNames->Double) {
                    cases.push_back({DragOf(target, std::nan(""))});
                } else if (a.GetTypeName() == SdfValueTypeNames->Float) {
                    cases.push_back({DragOf(target, std::nanf(""))});
                }
                cases.push_back({});
            }
            for (size_t c = 0; c < cases.size(); ++c) {
                evaluator->SetInteractiveOverrides(cases[c]);
                const RigExecRigPose pose = evaluator->Evaluate(t);
                CHECK(pose.valid);
                for (const char valid : program->chainValid) {
                    skipped += valid ? 0 : 1;
                }
                const std::string what = "walk resolver " + name + " at " +
                                         Text(t.GetValue()) + " case " +
                                         std::to_string(c);
                CHECK(ReaderWalkMismatches(*evaluator, cases[c], t, what,
                                           &checked) == 0);
            }
        }
    }
    std::printf("walk resolver: %zu walk read(s) checked, %zu float walk(s) "
                "of a double hop, %zu skipped chain(s) seen\n",
                checked, doubleTails, skipped);
    CHECK(checked > 0);
    CHECK(doubleTails > 0);
    CHECK(skipped > 0);
}

// For every visited binding, after every run, the leaf is the read: with no
// override; under a drag on a solver input, then lifted at the held frame;
// under a drag on a chain hop, then lifted; and the same for the first
// draggable input of every other visited family. The lifted run is the one
// that reads its leaf only because the override stood on the run before.
void
CheckBodyLeavesEqualTheFunnel(const Fixture &f, const UsdStageRefPtr &stage,
                             RigExecRigEvaluator *evaluator)
{
    const std::vector<UsdTimeCode> frames = Frames(stage);
    const std::string name = f.name;
    for (size_t frame = 1; frame < frames.size(); ++frame) {
        const UsdTimeCode t = frames[frame];
        RunChecked(evaluator, {}, t,
                   name + " no override at " + Text(t.GetValue()));
    }
    const RigExecBakedProgramImpl *B = Program(*evaluator);
    CHECK(B);
    if (!B) {
        return;
    }
    const std::vector<SdfPath> solverInputs = SolverInputs(*B);
    const std::vector<SdfPath> hops = ChainHops(*B);
    size_t cases = 0;
    const auto dragAndLift = [&](const SdfPath &path, const char *kind) {
        for (const UsdTimeCode t : frames) {
            const std::vector<RigExecValueOverride> drag =
                DragBy(stage, path, t, 0.25);
            if (drag.empty()) {
                return;
            }
            RunChecked(evaluator, drag, t,
                       name + " " + kind + " " + path.GetString() +
                           " at " + Text(t.GetValue()));
        }
        RunChecked(evaluator, {}, frames.back(),
                   name + " lifted " + kind + " " + path.GetString());
        ++cases;
    };
    if (!solverInputs.empty()) {
        dragAndLift(solverInputs.front(), "solver input");
    }
    if (!hops.empty()) {
        dragAndLift(hops.front(), "chain hop");
    }
    std::string families;
    for (const auto &[kind, path] : FamilyInputs(*B)) {
        dragAndLift(path, kind.c_str());
        families += " " + kind + ",";
    }
    std::printf("funnel %s: %zu solver input(s), %zu chain hop(s),%s "
                "%zu drag case(s)\n",
                f.name, solverInputs.size(), hops.size(),
                families.c_str(), cases);
}

// The first solver or constraint input of \p f whose two drag values pose
// the exec reference differently at \p time.
SdfPath
MovingSolverInput(const UsdStageRefPtr &stage, const SdfPath &rig,
                  UsdTimeCode time, const RigExecBakedProgramImpl &B)
{
    for (const SdfPath &path : SolverInputs(B)) {
        const auto a = DragBy(stage, path, time, 0.25);
        const auto b = DragBy(stage, path, time, 0.5);
        if (a.empty() || b.empty()) {
            continue;
        }
        if (PoseMismatches(FreshOverridePose(stage, rig, time, a),
                           FreshOverridePose(stage, rig, time, b), "probe",
                           /* quiet = */ true) > 0) {
            return path;
        }
    }
    return SdfPath();
}

// Two drag values on one solver input at one frame, one after the other:
// both override flags stand for the second, so only "while standing" -- not
// "when the flag changed" -- re-reads it. The solved pose follows each.
void
TestADragScrubbedAtAHeldFrameReachesTheBody(const std::string &examples)
{
    const Fixture f = FixtureNamed(Fixtures(examples), "03");
    UsdStageRefPtr stage = UsdStage::Open(f.stage);
    CHECK(stage);
    if (!stage) {
        return;
    }
    // Past the IK/FK blend's ramp, where the IK solve is the pose.
    const UsdTimeCode t(stage->GetStartTimeCode() + 34.0);
    auto evaluator = MakeEvaluator(stage, f.rig);
    RunChecked(evaluator.get(), {}, t, "held frame, no drag");
    const RigExecBakedProgramImpl *B = Program(*evaluator);
    CHECK(B);
    if (!B) {
        return;
    }
    const SdfPath input = MovingSolverInput(stage, f.rig, t, *B);
    CHECK(!input.IsEmpty());
    if (input.IsEmpty()) {
        return;
    }
    for (const double delta : {0.25, 0.5}) {
        const auto drag = DragBy(stage, input, t, delta);
        const std::string what =
            "held frame, drag " + Text(delta) + " on " + input.GetString();
        const RigExecRigPose pose = RunChecked(evaluator.get(), drag, t, what);
        CHECK(PoseMismatches(FreshOverridePose(stage, f.rig, t, drag), pose, what) ==
              0);
    }
    std::printf("scrubbed drag: %s\n", input.GetText());
}

// On computed_chains the constraint Follow reads its defaultWeight through
// the Dial chain's target. A drag on one of that chain's movers is on no hop
// of the reader's walk, so no flag of the reader's moves: only the chain
// result does, and the reader's leaf follows it.
void
TestAnotherChainsDragReachesAChainRoutedReader(const std::string &examples)
{
    const Fixture f = FixtureNamed(Fixtures(examples), "computed_chains");
    UsdStageRefPtr stage = UsdStage::Open(f.stage);
    CHECK(stage);
    if (!stage) {
        return;
    }
    const UsdTimeCode t(stage->GetStartTimeCode() + 2.0);
    SdfPath mover;
    std::vector<RigExecValueOverride> drag;
    const UsdPrim movers =
        stage->GetPrimAtPath(f.rig.AppendPath(SdfPath("Movers/Dial")));
    CHECK(movers);
    for (const UsdPrim &prim : UsdPrimRange(movers)) {
        const UsdAttribute value = prim.GetAttribute(TfToken("inputs:value"));
        if (!value || value.HasAuthoredConnections() ||
            value.GetTypeName() != SdfValueTypeNames->Float) {
            continue;
        }
        const auto candidate = DragBy(stage, value.GetPath(), t, 0.2);
        if (PoseMismatches(FreshOverridePose(stage, f.rig, t, {}),
                           FreshOverridePose(stage, f.rig, t, candidate), "probe",
                           /* quiet = */ true) > 0) {
            mover = value.GetPath();
            drag = candidate;
            break;
        }
    }
    CHECK(!mover.IsEmpty());
    if (mover.IsEmpty()) {
        return;
    }
    auto evaluator = MakeEvaluator(stage, f.rig);
    const RigExecRigPose before = RunChecked(evaluator.get(), {}, t, "chain drag, before");
    const auto *beforeProgram = Program(*evaluator);
    CHECK(beforeProgram);
    if (!beforeProgram) return;
    int follow = -1;
    for (size_t i = 0; i < beforeProgram->constraints.size(); ++i)
        if (beforeProgram->constraints[i].path.GetName() == "Follow") follow = int(i);
    CHECK(follow >= 0);
    if (follow < 0) return;
    const float originalWeight = RigExecBakedLeafRead(*beforeProgram,
        beforeProgram->constraints[size_t(follow)].defaultWeight);
    const std::string what = "chain drag on " + mover.GetString();
    const RigExecRigPose pose = RunChecked(evaluator.get(), drag, t, what);
    CHECK(PoseMismatches(FreshOverridePose(stage, f.rig, t, drag), pose, what) == 0);
    // The raw sample stays raw. The consumer resolves the changed typed
    // producer through its declared walk at execution time.
    const RigExecBakedProgramImpl *B = Program(*evaluator);
    const auto &input = B->constraints[size_t(follow)].defaultWeight;
    CHECK(input.walk >= 0);
    const float draggedWeight = RigExecBakedLeafRead(*B, input);
    CHECK(draggedWeight != originalWeight);
    bool declaredRoute = false, consumerRan = false;
    for (size_t i = 0; i < B->steps.size(); ++i) {
        const auto &step = B->steps[i];
        if (std::find(step.readerWalks.begin(), step.readerWalks.end(), input.walk) ==
            step.readerWalks.end()) continue;
        if (step.label.find(B->constraints[size_t(follow)].path.GetString()) ==
            std::string::npos) continue;
        for (const auto &read : step.reads)
            declaredRoute = declaredRoute ||
                (read.domain == RigExecBakedSlotDomain::PropertyResult && read.end > read.begin);
        consumerRan = consumerRan ||
            (i < B->opExecution.ran.size() && B->opExecution.ran[i]);
    }
    CHECK(declaredRoute && consumerRan);
    CHECK(B->overridableInputs.count(mover) == 0);
    std::printf("another chain's drag: %s changed effective Follow weight %.9g -> %.9g\n",
                mover.GetText(), double(originalWeight), double(draggedWeight));
    const RigExecRigPose released = RunChecked(evaluator.get(), {}, t, "chain drag, lifted");
    CHECK(PoseMismatches(FreshOverridePose(stage, f.rig, t, {}), released,
                         "chain drag, lifted") == 0);
    const auto *releasedProgram = Program(*evaluator);
    CHECK(RigExecBakedLeafRead(*releasedProgram,
        releasedProgram->constraints[size_t(follow)].defaultWeight) == originalWeight);

}

// Values that carry no override number: a drag, its release and then an
// authored edit, each held to the reference (the drag) or to a program
// built fresh on the edited stage (the edit).
void
RoutedCase(const Fixture &f, const SdfPath &property, const VtValue &dragged,
           const char *what)
{
    UsdStageRefPtr stage = UsdStage::Open(f.stage);
    CHECK(stage);
    if (!stage) {
        return;
    }
    UsdAttribute attribute = stage->GetAttributeAtPath(property);
    CHECK(attribute);
    if (!attribute) {
        return;
    }
    const UsdTimeCode t(stage->GetStartTimeCode() + 2.0);
    auto evaluator = MakeEvaluator(stage, f.rig);
    const std::string name = std::string(f.name) + " " + what;
    const auto before = RunChecked(evaluator.get(), {}, t, name + ", before");
    const std::vector<RigExecValueOverride> drag = {RigExecValueOverride{
        property.GetPrimPath(), TfToken(), property.GetNameToken(), dragged}};
    RigExecRigPose pose = RunChecked(evaluator.get(), drag, t, name + ", drag");
    CHECK(PoseMismatches(FreshOverridePose(stage, f.rig, t, drag), pose,
                         name + ", drag") == 0);
    pose = RunChecked(evaluator.get(), {}, t, name + ", released");
    CHECK(PoseMismatches(FreshOverridePose(stage, f.rig, t, {}), pose,
                         name + ", released") == 0);
    const size_t builds = evaluator->GetBakedProgramBuildCount();
    CHECK(attribute.Set(dragged));
    pose = RunChecked(evaluator.get(), {}, t, name + ", edited");
    if (std::string(what)=="wire dropoff")
        CHECK(before.movedProperties.at(SdfPath("/Asset/WireMesh.points")) !=
              pose.movedProperties.at(SdfPath("/Asset/WireMesh.points")));
    CHECK(evaluator->GetBakedProgramBuildCount() == builds);
    CHECK(PoseMismatches(FreshPose(stage, f.rig, t), pose, name + ", edited") ==
          0);
    std::printf("routed %s: disposition %d\n", name.c_str(),
                int(evaluator->GetLastNoticeDisposition()));
}

void RoutedPathLeafCases(const std::string &examples);
void RoutedSmoothScrubCase(const std::string &examples);
void RoutedDeformerCases(const std::string &examples);

// The property revisions' own inputs, which the head ops read as head
// leaves: a curve's keys edited and then dragged, a chain target
// given time samples where it had a default, and a mover input given time
// samples. Each edit is held to a program built fresh on the edited stage,
// and the drag to the reference.
void
RoutedChainCases(const std::string &examples)
{
    const std::vector<Fixture> fixtures = Fixtures(examples);
    {
        const Fixture &f = FixtureNamed(fixtures, "computed_chains");
        UsdStageRefPtr stage = UsdStage::Open(f.stage);
        CHECK(stage);
        if (!stage) {
            return;
        }
        const SdfPath keys("/Asset/Rig/Movers/Dial/Shape.inputs:keys");
        UsdAttribute attribute = stage->GetAttributeAtPath(keys);
        CHECK(attribute);
        const UsdTimeCode t(stage->GetStartTimeCode() + 2.0);
        auto evaluator =
            MakeEvaluator(stage, f.rig);
        RunChecked(evaluator.get(), {}, t, "curve keys, before");
        const size_t builds = evaluator->GetBakedProgramBuildCount();
        CHECK(attribute.Set(VtArray<GfVec2f>{GfVec2f(0.0f, 0.0f),
                                             GfVec2f(0.5f, 0.4f),
                                             GfVec2f(2.0f, 1.6f)}));
        RigExecRigPose pose =
            RunChecked(evaluator.get(), {}, t, "curve keys, edited");
        CHECK(evaluator->GetBakedProgramBuildCount() == builds);
        CHECK(PoseMismatches(FreshPose(stage, f.rig, t), pose,
                             "curve keys, edited") == 0);
        const std::vector<RigExecValueOverride> drag = {RigExecValueOverride{
            keys.GetPrimPath(), TfToken(), keys.GetNameToken(),
            VtValue(VtArray<GfVec2f>{GfVec2f(0.0f, 0.0f),
                                     GfVec2f(0.5f, 1.0f),
                                     GfVec2f(2.0f, 1.1f)})}};
        pose = RunChecked(evaluator.get(), drag, t, "curve keys, dragged");
        CHECK(PoseMismatches(FreshOverridePose(stage, f.rig, t, drag), pose,
                             "curve keys, dragged") == 0);
        pose = RunChecked(evaluator.get(), {}, t, "curve keys, released");
        CHECK(PoseMismatches(FreshOverridePose(stage, f.rig, t, {}), pose,
                             "curve keys, released") == 0);
    }
    const Fixture &f = FixtureNamed(fixtures, "09");
    const auto animate = [&](const SdfPath &property, const VtValue &first,
                             const VtValue &third, const char *what) {
        UsdStageRefPtr stage = UsdStage::Open(f.stage);
        CHECK(stage);
        if (!stage) {
            return;
        }
        UsdAttribute attribute = stage->GetAttributeAtPath(property);
        CHECK(attribute);
        const UsdTimeCode t1(stage->GetStartTimeCode());
        const UsdTimeCode t3(stage->GetStartTimeCode() + 2.0);
        auto evaluator =
            MakeEvaluator(stage, f.rig);
        const std::string name = std::string("animated ") + what;
        RunChecked(evaluator.get(), {}, t1, name + ", before");
        const size_t builds = evaluator->GetBakedProgramBuildCount();
        {
            // One notice: the property goes from a default to two samples.
            SdfChangeBlock block;
            CHECK(attribute.Set(first, t1));
            CHECK(attribute.Set(third, t3));
        }
        const RigExecRigPose one =
            RunChecked(evaluator.get(), {}, t1, name + " at 1");
        CHECK(PoseMismatches(FreshPose(stage, f.rig, t1), one,
                             name + " at 1") == 0);
        const RigExecRigPose three =
            RunChecked(evaluator.get(), {}, t3, name + " at 3");
        const RigExecRigPose freshThree = FreshPose(stage, f.rig, t3);
        CHECK(PoseMismatches(freshThree, three, name + " at 3") == 0);
        CHECK(PoseMismatches(freshThree, three,
                             name + " at 3, reference") == 0);
        // The time now moves the chain.
        CHECK(PoseMismatches(one, three, name + " 1 against 3",
                             /*quiet=*/true) != 0);
        std::printf("routed %s: %zu rebuild(s)\n", name.c_str(),
                    evaluator->GetBakedProgramBuildCount() - builds);
    };
    // The target's own value is no epoch input, so this one re-binds its
    // head leaf as varying in the standing program.
    animate(SdfPath("/PropMathAsset/Rig/Channels/Dials.rigExec:gain"),
            VtValue(0.25f), VtValue(0.75f), "chain target");
    animate(SdfPath("/PropMathAsset/Rig/Movers/OffsetLift.inputs:value"),
            VtValue(GfVec3f(0.0f, 4.0f, 0.0f)),
            VtValue(GfVec3f(0.0f, 8.0f, 0.0f)), "chain mover input");
}

void
TestRoutedValuesReachTheirLeaves(const std::string &examples)
{
    const Fixture f = FixtureNamed(Fixtures(examples), "09");
    // A chain mover's constant input, and a chain target's own default.
    RoutedCase(f,
               SdfPath("/PropMathAsset/Rig/Movers/OffsetLift.inputs:value"),
               VtValue(GfVec3f(0.0f, 2.5f, 0.0f)), "chain mover input");
    RoutedCase(f,
               SdfPath("/PropMathAsset/Rig/Channels/Dials.rigExec:gain"),
               VtValue(0.5f), "chain target default");
    // A chain mover's envelope, edited at the held frame.
    RoutedCase(
        f, SdfPath("/PropMathAsset/Rig/Movers/OffsetSpace.inputs:defaultWeight"),
        VtValue(0.5f), "chain mover defaultWeight");
    RoutedChainCases(examples);

    // An avar edited through RigExecProgramAvarPatch, the animation input
    // route: the binding is patched in place and its leaf re-read.
    UsdStageRefPtr stage = UsdStage::Open(f.stage);
    const UsdTimeCode t(stage->GetStartTimeCode() + 2.0);
    auto evaluator = MakeEvaluator(stage, f.rig);
    RunChecked(evaluator.get(), {}, t, "avar patch, before");
    const RigExecBakedProgramImpl *B = Program(*evaluator);
    CHECK(B && !B->patchableAvars.empty());
    if (!B || B->patchableAvars.empty()) {
        return;
    }
    const SdfPath path = B->patchableAvars.begin()->first;
    UsdAttribute attribute = stage->GetAttributeAtPath(path);
    double value = 0.0;
    attribute.Get(&value, UsdTimeCode::Default());
    CHECK(attribute.Set(value + 0.5));
    CHECK(evaluator->GetLastNoticeDisposition() ==
          RigExecNoticeDisposition::Patched);
    const RigExecRigPose pose =
        RunChecked(evaluator.get(), {}, t, "avar patch, edited");
    CHECK(PoseMismatches(FreshPose(stage, f.rig, t), pose,
                         "avar patch, edited") == 0);

    // The path leaves': a skin mover's defaultWeight and a weight object's
    // painted values, then a smooth mover's scalar scrubbed at a held frame.
    RoutedPathLeafCases(examples);
    RoutedSmoothScrubCase(examples);
    // A deltaMush scalar, a lattice's bind-time cage and a wire's dropoff.
    RoutedDeformerCases(examples);
}

// Frame 1 twice, with nothing standing, lifted, edited or routed between:
// the second run re-reads no leaf.
void
CheckAHeldLeafIsNotResampled(const Fixture &f, RigExecRigEvaluator *evaluator,
                           UsdTimeCode t)
{
    const RigExecBakedProgramImpl *B = Program(*evaluator);
    CHECK(B);
    if (!B) {
        return;
    }
    const uint64_t first = B->leafSamples;
    CHECK(first >= B->leafRefs.size());
    RunChecked(evaluator, {}, t, std::string(f.name) + " again");
    const uint64_t again = B->leafSamples - first;
    if (again != 0) {
        std::printf("FAIL %s: %llu leaf re-read(s) with nothing moved\n",
                    f.name, static_cast<unsigned long long>(again));
    }
    CHECK(again == 0);
}

// Leaf ids are dense and follow the visitor; each pool index follows it
// within its pool; every hop of every registered binding's walk is filed
// under that binding's leaf; the filed paths are exactly the override
// table's; and the visitor's count is the parent's.
void
CheckLeafNumberingFollowsThePatchableOrder(const Fixture &f,
                                        RigExecRigEvaluator *evaluator)
{
    const RigExecBakedProgramImpl *program = Program(*evaluator);
    CHECK(program);
    if (!program) {
        return;
    }
    const RigExecBakedProgramImpl &B = *program;
    uint32_t id = 0;
    std::map<RigExecBakedLeafType, int> next;
    size_t broken = 0, unfiled = 0;
    frozenDetail::_ForEachPatchableInput(B, [&](const auto &input) {
        using T = std::decay_t<decltype(input.constant)>;
        const RigExecBakedLeafType type = RigExecBakedLeafTraits<T>::type;
        const auto &pool = B.leaves.template Of<T>();
        const bool numbered =
            input.leaf == next[type] && id < B.leafRefs.size() &&
            B.leafRefs[id].type == type &&
            B.leafRefs[id].index == uint32_t(input.leaf) &&
            size_t(input.leaf) < pool.id.size() &&
            pool.id[size_t(input.leaf)] == id;
        if (!numbered) {
            ++broken;
        }
        ++next[type];
        if (input.head && input.overrideIndex >= 0) {
            bool viaChain = false, varying = false;
            UsdAttribute selected;
            SdfPathVector walk;
            RigExecBakedClassifyInput<T>(input.head, UsdTimeCode::Default(),
                                         B.chainTargets, &viaChain,
                                         &varying, &selected, &walk);
            if (walk.empty()) {
                walk.push_back(input.head.GetPath());
            }
            for (const SdfPath &hop : walk) {
                const auto found = B.leafByPath.find(hop);
                if (found == B.leafByPath.end() ||
                    std::find(found->second.begin(), found->second.end(),
                              id) == found->second.end()) {
                    if (unfiled < 4) {
                        std::printf("FAIL %s: %s not filed under leaf "
                                    "%u\n",
                                    f.name, hop.GetText(), unsigned(id));
                    }
                    ++unfiled;
                }
            }
        }
        ++id;
    });
    CHECK(broken == 0);
    CHECK(unfiled == 0);
    CHECK(id == B.leafRefs.size());
    // The binding leaves' paths are the override table's; the path
    // leaves after them are filed under every hop of their reads.
    std::set<SdfPath> filed, overridable;
    for (const auto &entry : B.leafByPath) {
        for (const uint32_t leaf : entry.second) {
            if (leaf < B.leafRefs.size()) {
                filed.insert(entry.first);
            }
        }
    }
    size_t pathUnfiled = 0;
    for (size_t i = 0; i < B.pathLeafRefs.size(); ++i) {
        const RigExecBakedPathLeafRef &ref = B.pathLeafRefs[i];
        const RigExecBakedPathLeaves *leaves =
            RigExecBakedPathLeavesOf(B, ref);
        CHECK(leaves && ref.key < leaves->hops.size());
        if (!leaves || ref.key >= leaves->hops.size()) {
            continue;
        }
        const uint32_t leafId = uint32_t(B.leafRefs.size() + i);
        for (const SdfPath &hop : leaves->hops[ref.key]) {
            const auto found = B.leafByPath.find(hop);
            if (found == B.leafByPath.end() ||
                std::find(found->second.begin(), found->second.end(),
                          leafId) == found->second.end()) {
                ++pathUnfiled;
            }
        }
    }
    CHECK(pathUnfiled == 0);
    for (const auto &entry : B.overridableInputs) {
        overridable.insert(entry.first);
    }
    CHECK(filed == overridable);
    // The actual visitor and typed pools define the binding census;
    // every visited binding has exactly one sequential id above.
    CHECK(id > 0);
    std::printf("numbering %s: %u leaf(s), %zu path(s) filed\n", f.name,
                unsigned(id), filed.size());
}

// These checks share exactly the authored, empty-override cold state.
// Numbering reads it without mutation; the held run precedes all drags.
void
TestBindingFixtureInputs(const std::string &examples)
{
    for (const Fixture &f : Fixtures(examples)) {
        UsdStageRefPtr stage = UsdStage::Open(f.stage);
        CHECK(stage);
        if (!stage) continue;
        auto evaluator = MakeEvaluator(stage, f.rig);
        const UsdTimeCode t(stage->GetStartTimeCode());
        const RigExecRigPose cold = RunChecked(
            evaluator.get(), {}, t, std::string(f.name) + " first");
        CHECK(cold.valid);
        CheckLeafNumberingFollowsThePatchableOrder(f, evaluator.get());
        CheckAHeldLeafIsNotResampled(f, evaluator.get(), t);
        CheckBodyLeavesEqualTheFunnel(f, stage, evaluator.get());
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

// A constant avar authored, then keyed (promoted to a per-frame read), then
// un-keyed, each through RigExecProgramAvarPatch. After each the binding
// holds exactly what the patch writes, the leaf is the read, and the leaf
// holds the value the stage gives. The bake after the edits prints its
// hash: RigExecBakeToBinary compiles the epoch again first, so it exports a
// program built on the edited stage, and the hash must equal the parent's.
void
TestAnAvarPatchExportsThePatchedBinding(const std::string &examples)
{
    const std::vector<Fixture> fixtures = Fixtures(examples);
    for (const char *name : {"09", "biped"}) {
        const Fixture &f = FixtureNamed(fixtures, name);
        UsdStageRefPtr stage = UsdStage::Open(f.stage);
        CHECK(stage);
        if (!stage) {
            continue;
        }
        const UsdTimeCode t(stage->GetStartTimeCode() + 2.0);
        SdfPath path;
        {
            auto probe =
                MakeEvaluator(stage, f.rig);
            CHECK(probe->Evaluate(t).valid);
            const RigExecBakedProgramImpl *P = Program(*probe);
            CHECK(P && !P->patchableAvars.empty());
            if (!P || P->patchableAvars.empty()) {
                continue;
            }
            path = P->patchableAvars.begin()->first;
        }
        UsdAttribute attribute = stage->GetAttributeAtPath(path);
        double value = 0.0;
        attribute.Get(&value, UsdTimeCode::Default());
        // The root layer's own spec first, so each edit below is a
        // changed-info notice rather than a spec's arrival.
        CHECK(attribute.Set(value));
        auto evaluator =
            MakeEvaluator(stage, f.rig);
        RunChecked(evaluator.get(), {}, t, std::string(name) + " patch");
        const RigExecBakedProgram *program = evaluator->GetBakedProgram();
        const RigExecBakedProgramImpl &B = program->GetStepGraph();
        const size_t index = B.patchableAvars.at(path);
        const RigExecBakedProgramImpl::AvarBinding &binding =
            B.avarConstantBindings[index];
        const auto expectPlain = [&](double expected, const char *step) {
            CHECK(evaluator->GetLastNoticeDisposition() ==
                  RigExecNoticeDisposition::Patched);
            RunChecked(evaluator.get(), {}, t,
                       std::string(name) + " patch " + step);
            CHECK(evaluator->GetBakedProgram() == program);
            CHECK(!binding.input.varying);
            CHECK(!binding.input.resolvedAttr);
            CHECK(binding.input.constant == expected);
            CHECK(B.avarConstants[binding.slot] == expected);
            CHECK(RigExecBakedLeaf(B, binding.input) == expected);
        };
        CHECK(attribute.Set(value + 0.5));
        expectPlain(value + 0.5, "edited");

        TsSpline spline = attribute.GetSpline();
        TsKnot knot;
        knot.SetTime(t.GetValue());
        knot.SetValue(value + 1.5);
        knot.SetNextInterpolation(TsInterpCurve);
        spline.SetKnot(knot);
        CHECK(attribute.SetSpline(spline));
        CHECK(evaluator->GetLastNoticeDisposition() ==
              RigExecNoticeDisposition::Patched);
        RunChecked(evaluator.get(), {}, t, std::string(name) + " patch keyed");
        CHECK(evaluator->GetBakedProgram() == program);
        CHECK(binding.input.varying);
        CHECK(binding.input.resolvedAttr == binding.input.head);
        CHECK(!binding.input.query.IsValid());
        CHECK(RigExecBakedLeaf(B, binding.input) == value + 1.5);

        const SdfAttributeSpecHandle spec =
            stage->GetRootLayer()->GetAttributeAtPath(path);
        CHECK(spec);
        if (spec) {
            spec->ClearSpline();
        }
        expectPlain(value + 0.5, "un-keyed");

        RigExecBakeOpts opts;
        opts.time = t.GetValue();
        RigExecBakeResult result;
        std::string error;
        const bool baked =
            RigExecBakeToBinary(*evaluator, opts, &result, &error);
        if (!baked) {
            std::printf("FAIL %s: bake refused: %s\n", name, error.c_str());
        }
        CHECK(baked);
        std::printf("patch export %s %s: %016llx\n", name, path.GetText(),
                    static_cast<unsigned long long>(Fnv(result.bytes)));
    }
}

// ---------------------------------------------------------------------------
// Path leaves (revision packets and weight-object gathers).

// The resolved inputs as the prologue placed them, from the program's after
// a run: the epilogue publishes the pose weights into them, so those paths
// go back to the chain result, else the override, else nothing.
RigExecResolvedInputs
PrologueInputs(const RigExecBakedProgramImpl &B,
               const std::vector<RigExecValueOverride> &overrides)
{
    RigExecResolvedInputs R = *B.resolvedInputs;
    for (const SdfPath &path : B.poseWeightPaths) {
        R.ClearProperty(path);
        for (const RigExecValueOverride &o : overrides) {
            if (!o.attribute.IsEmpty() &&
                o.prim.AppendProperty(o.attribute) == path) {
                R.SetProperty(path, o.value);
            }
        }
        const auto found = B.propertyResults.find(path);
        if (found != B.propertyResults.end()) {
            R.SetProperty(path, found->second);
        }
    }
    return R;
}

SdfPath
RootOf(const UsdStageRefPtr &stage)
{
    for (const UsdPrim &prim : stage->Traverse()) {
        if (prim.GetTypeName() == TfToken("RigExecRoot")) {
            return prim.GetPath();
        }
    }
    return SdfPath();
}

struct GeometryFixture {
    const char *name;
    std::string stage;
    /// Builds the stage in memory instead, when set.
    std::function<UsdStageRefPtr()> make;
};

UsdStageRefPtr
Open(const GeometryFixture &f)
{
    return f.make ? f.make() : UsdStage::Open(f.stage);
}

// A wrinkle mover over a 5x5 grid (no example ships one): its iterations
// keyed, its amplitude connected to a keyed driver, its topology token and
// pin points authored, so every kind of read it makes is exercised.
UsdStageRefPtr
MakeWrinkleStage()
{
    const UsdStageRefPtr stage = UsdStage::CreateInMemory();
    stage->SetStartTimeCode(1.0);
    stage->SetEndTimeCode(10.0);
    const SdfPath rig("/Rig"), target("/Rig/Mesh.points");
    auto builder = RigExecRigBuilder::Create(stage, rig);
    const UsdPrim mesh =
        stage->DefinePrim(target.GetPrimPath(), TfToken("Mesh"));
    VtVec3fArray rest, posed;
    VtIntArray counts, indices;
    for (int y = 0; y < 5; ++y) {
        for (int x = 0; x < 5; ++x) {
            rest.push_back(GfVec3f(0.25f * x, 0.25f * y, 0.0f));
            posed.push_back(GfVec3f(0.15f * x, 0.25f * y,
                                    (x == 2 && y == 2) ? 0.2f : 0.0f));
        }
    }
    for (int y = 0; y < 4; ++y) {
        for (int x = 0; x < 4; ++x) {
            const int a = y * 5 + x;
            counts.push_back(4);
            for (const int index : {a, a + 1, a + 6, a + 5}) {
                indices.push_back(index);
            }
        }
    }
    mesh.CreateAttribute(TfToken("points"), SdfValueTypeNames->Point3fArray)
        .Set(posed);
    mesh.CreateAttribute(TfToken("faceVertexCounts"),
                         SdfValueTypeNames->IntArray)
        .Set(counts);
    mesh.CreateAttribute(TfToken("faceVertexIndices"),
                         SdfValueTypeNames->IntArray)
        .Set(indices);
    const UsdPrim mover = builder.NewMoverChain("Deform", target)
                              .AddWrinkleMover("Wrinkle")
                              .GetPrim();
    mover.GetAttribute(TfToken("inputs:restPoints")).Set(rest);
    mover.GetAttribute(TfToken("inputs:pinBorders")).Set(false);
    const UsdAttribute iterations =
        mover.GetAttribute(TfToken("inputs:iterations"));
    iterations.Set(4, UsdTimeCode(1.0));
    iterations.Set(12, UsdTimeCode(5.0));
    const UsdAttribute driver = mesh.CreateAttribute(
        TfToken("deformAmount"), SdfValueTypeNames->Float);
    driver.Set(0.0f, UsdTimeCode(1.0));
    driver.Set(1.0f, UsdTimeCode(5.0));
    mover.GetAttribute(TfToken("inputs:wrinkleScale"))
        .SetConnections({driver.GetPath()});
    mover.GetAttribute(TfToken("inputs:topology"))
        .Set(TfToken("surfaceStruts"));
    mover.GetAttribute(TfToken("inputs:pinPoints")).Set(VtIntArray{0, 4});
    return stage;
}

std::vector<GeometryFixture>
GeometryFixtures(const std::string &examples)
{
    const std::string fixtures = examples + "/../tests/fixtures/";
    return {
        {"01", examples + "/01_FkChainTail.usda"},
        {"04", examples + "/04_BlendShapeFace.usda"},
        {"05", examples + "/05_TwistRibbonSpine.usda"},
        {"06", examples + "/06_LatticeBulge.usda"},
        {"07", examples + "/07_SurfaceDrape.usda"},
        {"11", examples + "/11_VolumeWeights.usda"},
        {"13", examples + "/13_ReadPhases.usda"},
        {"14", examples + "/14_VolumeConstrainedSweep.usda"},
        {"15", examples + "/15_TransformMatrixMover.usda"},
        {"biped", examples + "/biped/Biped_anim.usda"},
        {"computed_path_reads", fixtures + "computed_path_reads.usda"},
        {"oneloop_cross_domain", fixtures + "oneloop_cross_domain.usda"},
        {"phased_blend_samples", fixtures + "phased_blend_samples.usda"},
        {"preceding_own_chain", fixtures + "preceding_own_chain.usda"},
        {"bust", examples + "/2d/bust/bust_anim.usda"},
        {"bust_dd_a", examples + "/2d/bust_dd_a/bust_dd_a_anim.usda"},
        // The whole biped: its wires and its eyes' projector targets.
        {"biped_stack", examples + "/biped/Biped_stack_anim.usda"},
        {"projector_spaces", fixtures + "projector_spaces.usda"},
        {"wrinkle", std::string(), &MakeWrinkleStage},
    };
}

// The shadow assembly after the last run, held empty.
size_t
ShadowChecked(const RigExecRigEvaluator &evaluator,
              const std::vector<RigExecValueOverride> &overrides,
              UsdTimeCode time, const std::string &what)
{
    const RigExecBakedProgram *program = evaluator.GetBakedProgram();
    CHECK(program);
    if (!program) {
        return 0;
    }
    size_t compared = 0;
    const std::vector<std::string> report =
        RigExecBakedProgramTesting::ShadowAssembly(
            *program, PrologueInputs(program->GetStepGraph(), overrides),
            time, {}, &compared);
    for (size_t i = 0; i < report.size() && i < 6; ++i) {
        std::printf("FAIL %s: %s\n", what.c_str(), report[i].c_str());
    }
    CHECK(report.empty());
    return compared;
}

// A float drag on \p property, typed as the attribute is.
std::vector<RigExecValueOverride>
DragTo(const UsdStageRefPtr &stage, const SdfPath &property, double value)
{
    const UsdAttribute attribute = stage->GetAttributeAtPath(property);
    if (!attribute) {
        return {};
    }
    if (attribute.GetTypeName().GetType() == TfType::Find<double>()) {
        return {DragOf(property, value)};
    }
    if (attribute.GetTypeName().GetType() == TfType::Find<float>()) {
        return {DragOf(property, float(value))};
    }
    return {};
}

// The first revision the leaves assemble that consults its own
// `inputs:defaultWeight` (no weight object, an authored mover input).
SdfPath
LeafDefaultWeight(const UsdStageRefPtr &stage, const RigExecBakedProgramImpl &B)
{
    for (const auto &chain : B.chains) {
        for (const auto &revision : chain.revisions) {
            if (!revision.leaves.decl.assembles || revision.weightObject >= 0 ||
                revision.leaves.decl.Role(
                    RigExecRevisionLeafRole::DefaultWeight) < 0) {
                continue;
            }
            const SdfPath path = revision.moverPath.AppendProperty(
                TfToken("inputs:defaultWeight"));
            if (stage->GetAttributeAtPath(path)) {
                return path;
            }
        }
    }
    return SdfPath();
}

// The first hop past a path leaf's own attribute that carries an override
// number: a numbered input the leaf reads through a connection.
SdfPath
UpstreamNumberedHop(const RigExecBakedProgramImpl &B)
{
    for (const auto &chain : B.chains) {
        for (const auto &revision : chain.revisions) {
            const RigExecBakedPathLeaves &leaves = revision.leaves;
            for (size_t k = 0; k < leaves.hops.size(); ++k) {
                for (size_t h = 1; h < leaves.hops[k].size(); ++h) {
                    if (B.overridableInputs.count(leaves.hops[k][h])) {
                        return leaves.hops[k][h];
                    }
                }
            }
        }
    }
    return SdfPath();
}

// For every covered revision of every geometry fixture, after every run,
// the packet assembled from the leaves the prologue sampled equals the one
// RigExecAssembleParameters assembles off the stage through the same
// resolved inputs, packet, status and lines: with no override at frames 1,
// 4 and 9, under a drag on a control avar, and under an override on a mover
// input (a defaultWeight, and a blend channel's weight where there is one).
void
CheckPureAssemblyEqualsTheStageAssembler(const GeometryFixture &f,
                                       const UsdStageRefPtr &stage,
                                       const SdfPath &rig,
                                       RigExecRigEvaluator *evaluator,
                                       size_t coldCompared,
                                       std::set<RigExecRevisionOp> &reached)
{
    const double start = stage->GetStartTimeCode();
    const std::string name = f.name;
    size_t compared = coldCompared;
    for (const double offset : {3.0, 8.0}) {
        const UsdTimeCode t(start + offset);
        const std::string what = name + " at " + Text(t.GetValue());
        RunChecked(evaluator, {}, t, what);
        compared = std::max(compared,
                            ShadowChecked(*evaluator, {}, t, what));
    }
    const RigExecBakedProgramImpl *B = Program(*evaluator);
    CHECK(B);
    if (!B) {
        return;
    }
    const UsdTimeCode t(start + 3.0);
    // Each case runs under its overrides, then lifts them unless the
    // next case continues it at the held frame.
    struct Case {
        std::string label;
        std::vector<RigExecValueOverride> overrides;
        bool lift = true;
    };
    std::vector<Case> cases;
    if (!B->avarBindings.empty() && B->avarBindings.front().input.head) {
        const SdfPath avar = B->avarBindings.front().input.head.GetPath();
        cases.push_back(
            {"drag " + avar.GetString(), DragBy(stage, avar, t, 0.25)});
    }
    // A drag on a numbered input a leaf's walk reaches through a
    // connection (a blend weight connected to a control's avar): the
    // leaf has no number of its own, so only the override rule re-reads
    // it, at two values on the held frame and once after it lifts.
    const SdfPath hop = UpstreamNumberedHop(*B);
    if (!hop.IsEmpty()) {
        cases.push_back({"upstream drag " + hop.GetString(),
                         DragBy(stage, hop, t, 0.25), false});
        cases.push_back({"upstream drag again " + hop.GetString(),
                         DragBy(stage, hop, t, 0.5)});
    }
    const SdfPath weight = LeafDefaultWeight(stage, *B);
    if (!weight.IsEmpty()) {
        cases.push_back({"override " + weight.GetString(),
                         DragTo(stage, weight, 0.37)});
    }
    for (const auto &chain : B->chains) {
        for (const auto &revision : chain.revisions) {
            if (!revision.blendChannels.empty() &&
                cases.size() < 6) {
                const SdfPath channel =
                    revision.blendChannels.front().weightPath;
                cases.push_back({"override " + channel.GetString(),
                                 DragTo(stage, channel, 0.42)});
            }
        }
    }
    for (const Case &c : cases) {
        if (c.overrides.empty()) {
            continue;
        }
        const std::string what = name + " " + c.label;
        const RigExecRigPose pose =
            RunChecked(evaluator, c.overrides, t, what);
        ShadowChecked(*evaluator, c.overrides, t, what);
        CHECK(PoseMismatches(FreshOverridePose(stage, rig, t, c.overrides), pose,
                             what) == 0);
        if (c.lift) {
            RunChecked(evaluator, {}, t, what + ", lifted");
            ShadowChecked(*evaluator, {}, t, what + ", lifted");
        }
    }
    std::printf("shadow %s: %zu covered revision(s), %zu override "
                "case(s), upstream hop %s\n",
                f.name, compared, cases.size(),
                hop.IsEmpty() ? "-" : hop.GetText());
    // The operations the shadow compared on this fixture.
    for (const auto &chain : Program(*evaluator)->chains) {
        if (!chain.haveBase) {
            continue;
        }
        for (const auto &revision : chain.revisions) {
            if (revision.leaves.decl.assembles) {
                reached.insert(revision.op);
            }
        }
        for (const auto &derived : chain.derived) {
            if (derived.revision.leaves.decl.assembles &&
                (derived.matrixTarget || derived.haveBase)) {
                reached.insert(derived.revision.op);
            }
        }
    }
}

// A mover's `inputs:defaultWeight` connected to a control's avar on a rig
// with no property chain: a drag on the avar, at two values on a held frame
// and then lifted, reaches the leaf only through the override rule, since
// the leaf carries no number and no chain result moves.
void
TestADragUpstreamOfAMoverInputReachesItsLeaf(const std::string &examples)
{
    for (const GeometryFixture &f : GeometryFixtures(examples)) {
        UsdStageRefPtr stage = Open(f);
        if (!stage) {
            continue;
        }
        const SdfPath rig = RootOf(stage);
        SdfPath input, avar;
        {
            auto probe = MakeEvaluator(stage, rig);
            if (!probe->Evaluate(UsdTimeCode(stage->GetStartTimeCode()))
                     .valid) {
                continue;
            }
            const RigExecBakedProgramImpl *B = Program(*probe);
            if (!B || B->hasPropertyChains || B->avarBindings.empty() ||
                !B->avarBindings.front().input.head) {
                continue;
            }
            input = LeafDefaultWeight(stage, *B);
            avar = B->avarBindings.front().input.head.GetPath();
        }
        if (input.IsEmpty()) {
            continue;
        }
        CHECK(stage->GetAttributeAtPath(input).SetConnections({avar}));
        const UsdTimeCode t(stage->GetStartTimeCode() + 3.0);
        auto evaluator = MakeEvaluator(stage, rig);
        RunChecked(evaluator.get(), {}, t, "upstream, before");
        CHECK(!Program(*evaluator)->hasPropertyChains);
        for (const double delta : {0.25, 0.5}) {
            const auto drag = DragBy(stage, avar, t, delta);
            const std::string what = "upstream " + Text(delta);
            const RigExecRigPose pose =
                RunChecked(evaluator.get(), drag, t, what);
            CHECK(ShadowChecked(*evaluator, drag, t, what) > 0);
            CHECK(PoseMismatches(FreshOverridePose(stage, rig, t, drag), pose, what) ==
                  0);
        }
        RunChecked(evaluator.get(), {}, t, "upstream, lifted");
        ShadowChecked(*evaluator, {}, t, "upstream, lifted");
        std::printf("upstream %s: %s connected to %s\n", f.name,
                    input.GetText(), avar.GetText());
        return;
    }
    std::printf("FAIL no chainless fixture with a mover input and an avar\n");
    CHECK(false);
}

// A declaration with `inputs:defaultWeight` left out: the shadow names the
// read the leaf assembly could not answer, and the packet it then built.
void
TestAnUndeclaredReadIsNamed(const std::string &examples)
{
    UsdStageRefPtr stage = UsdStage::Open(examples + "/06_LatticeBulge.usda");
    CHECK(stage);
    if (!stage) {
        return;
    }
    const SdfPath rig = RootOf(stage);
    auto evaluator = MakeEvaluator(stage, rig);
    const UsdTimeCode t(stage->GetStartTimeCode() + 3.0);
    RunChecked(evaluator.get(), {}, t, "undeclared, before");
    const SdfPath weight = LeafDefaultWeight(stage, *Program(*evaluator));
    CHECK(!weight.IsEmpty());
    if (weight.IsEmpty()) {
        return;
    }
    const std::vector<RigExecValueOverride> drag = DragTo(stage, weight, 0.37);
    RunChecked(evaluator.get(), drag, t, "undeclared, override");
    const SdfPath mover = weight.GetPrimPath();
    size_t compared = 0;
    const std::vector<std::string> report =
        RigExecBakedProgramTesting::ShadowAssembly(
            *evaluator->GetBakedProgram(),
            PrologueInputs(*Program(*evaluator), drag), t,
            [&mover](const SdfPath &path, RigExecRevisionLeafDecl *decl) {
                if (path == mover) {
                    decl->roles[size_t(
                        RigExecRevisionLeafRole::DefaultWeight)] = -1;
                }
            },
            &compared);
    bool named = false, differs = false;
    for (const std::string &line : report) {
        std::printf("undeclared: %s\n", line.c_str());
        named = named || (line.find(mover.GetString()) != std::string::npos &&
                          line.find("undeclared read inputs:defaultWeight") !=
                              std::string::npos);
        differs = differs || line.find("packets differ (weights") !=
                                 std::string::npos;
    }
    CHECK(named);
    CHECK(differs);
}

// Every key the exporter's enumeration emits for a covered revision is a
// declared leaf of it, at the same time policy unless the attribute is
// missing (the enumeration keys a missing one at rest); every key of a
// weight object's gathers is one of its point leaves. Every Resolved
// scalar leaf on an attribute is a listed input of a bake of the rig. After
// the runs each dense blend sample's lastPoints is the points its leaf
// gave the gather, and each weight object's point lists are a fresh
// Build's.
void
TestRevisionLeavesCoverTheExporterEnumeration(const std::string &examples)
{
    std::map<RigExecRevisionOp, size_t> ops;
    // Enumeration read-site representatives; full asset conformance stays
    // in its registered suites, and geometry shadow keeps its all-op floor.
    const std::set<std::string> representatives = {
        "01", "04", "05", "06", "07", "11", "computed_path_reads",
        "oneloop_cross_domain", "phased_blend_samples",
        "preceding_own_chain", "compact_wire", "wrinkle"};
    size_t selectedFixtures = 0;
    std::vector<GeometryFixture> fixtures = GeometryFixtures(examples);
    fixtures.push_back({"compact_wire", examples + "/../tests/fixtures/leaves_routed_geometry.usda"});
    for (const GeometryFixture &f : fixtures) {
        if (!representatives.count(f.name)) continue;
        ++selectedFixtures;
        UsdStageRefPtr stage = Open(f);
        CHECK(stage);
        if (!stage) {
            continue;
        }
        const SdfPath rig = RootOf(stage);
        auto evaluator = MakeEvaluator(stage, rig);
        const double start = stage->GetStartTimeCode();
        size_t uncovered = 0, keys = 0, mistimed = 0;
        for (const double offset : {0.0, 3.0}) {
            const UsdTimeCode t(start + offset);
            RunChecked(evaluator.get(), {}, t,
                       std::string(f.name) + " enumeration");
            const RigExecBakedProgramImpl &B = *Program(*evaluator);
            const auto check = [&](const RigExecBakedPathLeaves &leaves,
                                   const RigExecBakeRevisionRead &read) {
                ++keys;
                bool found = false, timed = false;
                for (const RigExecRevisionLeafKey &key : leaves.decl.keys) {
                    if (key.path != read.path) {
                        continue;
                    }
                    found = true;
                    const bool rest =
                        key.time == RigExecRevisionLeafTime::AtDefault;
                    const bool missing =
                        read.value.IsEmpty() && !read.resolved &&
                        !stage->GetAttributeAtPath(read.path);
                    timed = timed || rest == read.rest || missing;
                }
                if (!found || !timed) {
                    if (uncovered + mistimed < 6) {
                        std::printf("FAIL %s: %s %s (rest %d)\n", f.name,
                                    found ? "mistimed" : "undeclared",
                                    read.path.GetText(), int(read.rest));
                    }
                    (found ? mistimed : uncovered) += 1;
                }
            };
            for (const auto &chain : B.chains) {
                const auto revisionKeys =
                    [&](const RigExecBakedProgramImpl::GeomRevision &revision) {
                        if (!revision.leaves.decl.assembles) {
                            return;
                        }
                        ++ops[revision.op];
                        RigExecBakeEnumerateRevisionReads(
                            B, revision, t.GetValue(),
                            [&](RigExecBakeRevisionRead &&read) {
                                check(revision.leaves, read);
                            });
                    };
                for (const auto &revision : chain.revisions) {
                    revisionKeys(revision);
                }
                for (const auto &derived : chain.derived) {
                    revisionKeys(derived.revision);
                }
            }
            for (size_t w = 0; w < B.weightObjects.size(); ++w) {
                RigExecBakeEnumerateWeightReads(
                    B, w, t.GetValue(), [&](RigExecBakeRevisionRead &&read) {
                        check(B.weightObjects[w].pointLeaves, read);
                    });
            }
        }
        CHECK(uncovered == 0);
        CHECK(mistimed == 0);

        // lastPoints, and the weight objects' point lists.
        const RigExecBakedProgramImpl &B = *Program(*evaluator);
        size_t lastPoints = 0;
        for (const auto &chain : B.chains) {
            if (!chain.haveBase) {
                continue;
            }
            for (const auto &revision : chain.revisions) {
                for (const auto &channel : revision.blendChannels) {
                    for (const auto &sample : channel.samples) {
                        if (sample.pointsLeaf < 0 ||
                            sample.pointBinding.id >= 0) {
                            continue;
                        }
                        const VtVec3fArray leaf =
                            revision.leaves.Value<VtVec3fArray>(
                                sample.pointsLeaf, VtVec3fArray());
                        CHECK(std::vector<GfVec3f>(leaf.begin(), leaf.end()) ==
                              sample.lastPoints);
                        ++lastPoints;
                    }
                }
            }
        }
        auto fresh = MakeEvaluator(stage, rig);
        CHECK(fresh->Evaluate(UsdTimeCode(start)).valid);
        const RigExecBakedProgramImpl &F = *Program(*fresh);
        CHECK(F.weightObjects.size() == B.weightObjects.size());
        const auto paths = [](const std::vector<UsdAttribute> &attributes) {
            std::vector<SdfPath> out;
            for (const UsdAttribute &a : attributes) {
                out.push_back(a.GetPath());
            }
            return out;
        };
        for (size_t w = 0;
             w < F.weightObjects.size() && w < B.weightObjects.size(); ++w) {
            const auto &a = B.weightObjects[w];
            const auto &b = F.weightObjects[w];
            CHECK(paths(a.targetPoints) == paths(b.targetPoints));
            CHECK(paths(a.samplePoints) == paths(b.samplePoints));
            CHECK(paths(a.curvePoints) == paths(b.curvePoints));
            CHECK(paths(a.combineTargetPoints) ==
                  paths(b.combineTargetPoints));
        }

        // The Resolved scalar leaves against a bake's listed inputs.
        RigExecBakeOpts opts;
        opts.time = start;
        RigExecBakeResult result;
        std::string error;
        if (!RigExecBakeToBinary(*evaluator, opts, &result, &error)) {
            std::printf("enumeration %s: %zu key(s), bake refused (%s)\n",
                        f.name, keys, error.c_str());
            continue;
        }
        std::unique_ptr<fb::RigExecWireFile> file;
        CHECK(RigExecFormatOpen(result.bytes.data(), result.bytes.size(),
                                &file, &error));
        if (!file) {
            continue;
        }
        std::set<SdfPath> listed;
        for (const fb::InputSlot &slot : file->inputs) {
            if (slot.flags() & uint8_t(fb::InputSlotFlags::Listed)) {
                listed.insert(
                    SdfPath(RigExecFormatPathText(*file, slot.name())));
            }
        }
        size_t scalars = 0, unlisted = 0;
        const auto listedCheck = [&](const RigExecBakedPathLeaves &leaves) {
            for (size_t k = 0; k < leaves.decl.keys.size(); ++k) {
                const RigExecRevisionLeafKey &key = leaves.decl.keys[k];
                const bool resolved =
                    key.flavour == RigExecRevisionLeafFlavour::Resolved ||
                    key.flavour == RigExecRevisionLeafFlavour::ResolvedOnly;
                const bool scalar =
                    key.type == RigExecRevisionLeafType::Bool ||
                    key.type == RigExecRevisionLeafType::Int ||
                    key.type == RigExecRevisionLeafType::Float ||
                    key.type == RigExecRevisionLeafType::Token;
                // An authored rest value (the wrinkle's topology token) is
                // read at Default, and the exporter lists no slot for one.
                const bool atTime =
                    key.time == RigExecRevisionLeafTime::AtTime;
                if (!resolved || !scalar || !atTime ||
                    !stage->GetAttributeAtPath(key.path)) {
                    continue;
                }
                ++scalars;
                if (!listed.count(key.path)) {
                    if (unlisted < 6) {
                        std::printf("FAIL %s: %s is no listed input\n",
                                    f.name, key.path.GetText());
                    }
                    ++unlisted;
                }
            }
        };
        const RigExecBakedProgramImpl &E = *Program(*evaluator);
        for (const auto &chain : E.chains) {
            for (const auto &revision : chain.revisions) {
                if (revision.leaves.decl.assembles) {
                    listedCheck(revision.leaves);
                }
            }
            for (const auto &derived : chain.derived) {
                listedCheck(derived.revision.leaves);
            }
        }
        CHECK(unlisted == 0);
        std::printf("enumeration %s: %zu key(s), %zu lastPoints, %zu resolved "
                    "scalar(s) listed of %zu input(s)\n",
                    f.name, keys, lastPoints, scalars, listed.size());
    }
    CHECK(selectedFixtures == representatives.size());
    for (const auto &[op, count] : ops) {
        std::printf("enumeration op %d: %zu revision(s)\n", int(op), count);
    }
}

// A held-frame scrub of a smooth mover's AtTime scalar: two drag values, one
// after the other, each the reference's pose.
void
RoutedSmoothScrubCase(const std::string &examples)
{
    UsdStageRefPtr stage = UsdStage::Open(examples + "/06_LatticeBulge.usda");
    CHECK(stage);
    if (!stage) {
        return;
    }
    const SdfPath rig = RootOf(stage);
    const SdfPath input(
        "/LatticeAsset/Rig/Movers/Geometry/VolumeCorrect/Smooth"
        ".inputs:defaultWeight");
    CHECK(stage->GetAttributeAtPath(input));
    const UsdTimeCode t(stage->GetStartTimeCode() + 12.0);
    auto evaluator = MakeEvaluator(stage, rig);
    RunChecked(evaluator.get(), {}, t, "smooth scrub, before");
    for (const double value : {0.25, 0.9}) {
        const auto drag = DragTo(stage, input, value);
        const std::string what = "smooth scrub " + Text(value);
        const RigExecRigPose pose = RunChecked(evaluator.get(), drag, t, what);
        ShadowChecked(*evaluator, drag, t, what);
        CHECK(PoseMismatches(FreshOverridePose(stage, rig, t, drag), pose, what) == 0);
    }
}

// A skin mover's unanimated defaultWeight, dragged, released and then
// authored (its spec made first, so the edit is a value notice), on the
// compact moving skin; and a weight object's painted values authored, which the program
// folds and so rebuilds for. Each against the reference or a fresh program.
void
RoutedPathLeafCases(const std::string &examples)
{
    {
        const std::string path = examples + "/../tests/fixtures/leaves_routed_geometry.usda";
        UsdStageRefPtr stage = UsdStage::Open(path);
        CHECK(stage);
        const SdfPath rig = RootOf(stage);
        SdfPath property;
        {
            auto probe = MakeEvaluator(stage, rig);
            CHECK(probe->Evaluate(UsdTimeCode(stage->GetStartTimeCode())).valid);
            for (const auto &chain : Program(*probe)->chains) {
                for (const auto &revision : chain.revisions) {
                    const SdfPath candidate = revision.moverPath.AppendProperty(
                        TfToken("inputs:defaultWeight"));
                    const UsdAttribute a = stage->GetAttributeAtPath(candidate);
                    if (property.IsEmpty() &&
                        revision.op == RigExecRevisionOp::Skin &&
                        revision.weightObject < 0 && a &&
                        !a.ValueMightBeTimeVarying()) {
                        property = candidate;
                    }
                }
            }
        }
        CHECK(!property.IsEmpty());
        if (!property.IsEmpty()) {
            UsdAttribute attribute = stage->GetAttributeAtPath(property);
            float value = 1.0f;
            attribute.Get(&value);
            CHECK(attribute.Set(value));
            const UsdTimeCode t(stage->GetStartTimeCode() + 2.0);
            auto evaluator =
                MakeEvaluator(stage, rig);
            const auto before = RunChecked(evaluator.get(), {}, t, "skin weight, before");
            const auto drag = DragTo(stage, property, 0.5);
            RigExecRigPose pose =
                RunChecked(evaluator.get(), drag, t, "skin weight, drag");
            ShadowChecked(*evaluator, drag, t, "skin weight, drag");
            CHECK(before.movedProperties.at(SdfPath("/Asset/SkinMesh.points")) !=
                  pose.movedProperties.at(SdfPath("/Asset/SkinMesh.points")));
            CHECK(PoseMismatches(FreshOverridePose(stage, rig, t, drag), pose,
                                 "skin weight, drag") == 0);
            pose = RunChecked(evaluator.get(), {}, t, "skin weight, released");
            CHECK(PoseMismatches(FreshOverridePose(stage, rig, t, {}), pose,
                                 "skin weight, released") == 0);
            const size_t builds = evaluator->GetBakedProgramBuildCount();
            CHECK(attribute.Set(0.5f));
            pose = RunChecked(evaluator.get(), {}, t, "skin weight, edited");
            ShadowChecked(*evaluator, {}, t, "skin weight, edited");
            CHECK(evaluator->GetBakedProgramBuildCount() == builds);
            CHECK(PoseMismatches(FreshPose(stage, rig, t), pose,
                                 "skin weight, edited") == 0);
            std::printf("routed skin weight %s: disposition %d\n",
                        property.GetText(),
                        int(evaluator->GetLastNoticeDisposition()));
        }
    }
    {
        UsdStageRefPtr stage =
            UsdStage::Open(examples + "/01_FkChainTail.usda");
        CHECK(stage);
        const SdfPath rig = RootOf(stage);
        const UsdTimeCode t(stage->GetStartTimeCode() + 12.0);
        auto evaluator = MakeEvaluator(stage, rig);
        RunChecked(evaluator.get(), {}, t, "painted, before");
        UsdAttribute values = stage->GetAttributeAtPath(
            SdfPath("/TailAsset/Rig/Weights/Seg2W.rigExec:values"));
        CHECK(values);
        VtFloatArray painted;
        values.Get(&painted);
        for (float &w : painted) {
            w = 1.0f - w;
        }
        CHECK(values.Set(painted));
        const RigExecRigPose pose =
            RunChecked(evaluator.get(), {}, t, "painted, edited");
        ShadowChecked(*evaluator, {}, t, "painted, edited");
        CHECK(PoseMismatches(FreshPose(stage, rig, t), pose,
                             "painted, edited") == 0);
        std::printf("routed painted weights: disposition %d\n",
                    int(evaluator->GetLastNoticeDisposition()));
    }
}

// The first revision of \p op in \p stage's program, by a probe evaluation.
const RigExecBakedProgramImpl::GeomRevision *
FirstRevisionOf(const RigExecRigEvaluator &probe, RigExecRevisionOp op)
{
    for (const auto &chain : Program(probe)->chains) {
        for (const auto &revision : chain.revisions) {
            if (revision.op == op) {
                return &revision;
            }
        }
    }
    return nullptr;
}

// Routed values of the deltaMush, wire, projector and lattice reads, held to
// the reference (a drag and its release) or to a fresh program (an authored
// edit): a deltaMush's constant `inputs:iterations` (read at the time) and
// its `inputs:restPoints` (at Default), a wire's `inputs:dropoffDistance`
// (read only where it stands), shader dials, one on a prim nothing else
// reads, and a lattice's bind-time cage (at Default, past the resolved
// inputs), edited only: a drag on the cage stands on a prim no baked reader
// names, so the baked program never takes it.
void
RoutedDeformerCases(const std::string &examples)
{
    {
        const Fixture f{"computed_path_reads",
                        examples + "/../tests/fixtures/computed_path_reads.usda",
                        SdfPath("/PathReadAsset/Rig")};
        RoutedCase(f, SdfPath("/PathReadAsset/Rig/Movers/Mush.inputs:iterations"),
                   VtValue(5), "deltaMush iterations");
        RoutedCase(f, SdfPath("/PathReadAsset/Rig/Movers/Mush.inputs:restPoints"),
                   VtValue(VtVec3fArray{GfVec3f(1.1f, 0, 0), GfVec3f(0, 1.3f, 0),
                                        GfVec3f(-1.1f, 0, 0), GfVec3f(0, -1, 0),
                                        GfVec3f(0, 0, 1.2f), GfVec3f(0, 0, -1)}),
                   "deltaMush rest points");
        // A shader dial on a prim nothing else in the program reads: only
        // the projector's leaf files its path, and an edit reaches it.
        UsdStageRefPtr stage = UsdStage::Open(f.stage);
        CHECK(stage);
        stage->SetEditTarget(stage->GetSessionLayer());
        const UsdAttribute dial =
            stage->DefinePrim(SdfPath("/PathReadAsset/Extra"))
                .CreateAttribute(TfToken("dial"), SdfValueTypeNames->Double);
        CHECK(dial.Set(0.25));
        CHECK(stage
                  ->GetPrimAtPath(SdfPath("/PathReadAsset/Rig/Movers/Projector"))
                  .GetRelationship(TfToken("rigExec:shaderDialSources"))
                  .AddTarget(dial.GetPath()));
        const UsdTimeCode t(stage->GetStartTimeCode() + 2.0);
        auto evaluator = MakeEvaluator(stage, f.rig);
        RunChecked(evaluator.get(), {}, t, "unread dial, before");
        const size_t builds = evaluator->GetBakedProgramBuildCount();
        CHECK(dial.Set(0.75));
        const RigExecRigPose pose =
            RunChecked(evaluator.get(), {}, t, "unread dial, edited");
        ShadowChecked(*evaluator, {}, t, "unread dial, edited");
        CHECK(evaluator->GetBakedProgramBuildCount() == builds);
        CHECK(PoseMismatches(FreshPose(stage, f.rig, t), pose,
                             "unread dial, edited") == 0);
        std::printf("routed unread dial: disposition %d\n",
                    int(evaluator->GetLastNoticeDisposition()));
    }
    {
        // A shader dial that is also a constant avar the pose reads: its
        // edit is patched in place (ApplyAvarValueEdits), and the dial's
        // path leaf must re-read it as the binding's leaf does.
        const std::string path =
            examples + "/../tests/fixtures/projector_spaces.usda";
        UsdStageRefPtr stage = UsdStage::Open(path);
        CHECK(stage);
        const SdfPath rig("/ProjectorAsset/Rig");
        const UsdAttribute avar = stage->GetAttributeAtPath(
            SdfPath("/ProjectorAsset/Rig/Controls/Space.avars:tx"));
        const UsdPrim projector =
            stage->GetPrimAtPath(SdfPath("/ProjectorAsset/Rig/Movers/InSpace"));
        CHECK(avar && projector);
        if (avar && projector) {
            projector
                .CreateAttribute(TfToken("rigExec:shaderDialPrimvar"),
                                 SdfValueTypeNames->Token, false,
                                 SdfVariabilityUniform)
                .Set(TfToken("inSpaceDials"));
            projector.CreateRelationship(TfToken("rigExec:shaderDialSources"))
                .SetTargets({avar.GetPath()});
            const UsdTimeCode t(stage->GetStartTimeCode() + 2.0);
            auto evaluator =
                MakeEvaluator(stage, rig);
            RunChecked(evaluator.get(), {}, t, "avar dial, before");
            const size_t builds = evaluator->GetBakedProgramBuildCount();
            CHECK(avar.Set(0.45));
            const RigExecRigPose pose =
                RunChecked(evaluator.get(), {}, t, "avar dial, edited");
            ShadowChecked(*evaluator, {}, t, "avar dial, edited");
            CHECK(evaluator->GetBakedProgramBuildCount() == builds);
            CHECK(PoseMismatches(FreshPose(stage, rig, t), pose,
                                 "avar dial, edited") == 0);
            std::printf("routed avar dial: disposition %d\n",
                        int(evaluator->GetLastNoticeDisposition()));
        }
    }
    {
        const std::string path = examples + "/../tests/fixtures/leaves_routed_geometry.usda";
        UsdStageRefPtr stage = UsdStage::Open(path);
        CHECK(stage);
        const SdfPath rig = RootOf(stage);
        SdfPath dropoff;
        {
            auto probe = MakeEvaluator(stage, rig);
            CHECK(probe->Evaluate(UsdTimeCode(stage->GetStartTimeCode())).valid);
            for (const auto &chain : Program(*probe)->chains) {
                for (const auto &revision : chain.revisions) {
                    const SdfPath candidate = revision.moverPath.AppendProperty(
                        TfToken("inputs:dropoffDistance"));
                    if (dropoff.IsEmpty() &&
                        revision.op == RigExecRevisionOp::Wire &&
                        stage->GetAttributeAtPath(candidate)) {
                        dropoff = candidate;
                    }
                }
            }
        }
        CHECK(!dropoff.IsEmpty());
        if (!dropoff.IsEmpty()) {
            const Fixture f{"compact_wire", path, rig};
            RoutedCase(f, dropoff, VtValue(37.0f), "wire dropoff");
        }
        // A shader dial's authored value (a control's avar), edited.
        // Keep the independent owner and original reference/history assertions.
        stage = UsdStage::Open(examples + "/../tests/fixtures/projector_spaces.usda");
        CHECK(stage);
        const SdfPath dialRig("/ProjectorAsset/Rig");
        const auto dialProjector=stage->GetPrimAtPath(SdfPath("/ProjectorAsset/Rig/Movers/InSpace"));
        CHECK(dialProjector);
        CHECK(dialProjector.CreateAttribute(TfToken("rigExec:shaderDialPrimvar"),
            SdfValueTypeNames->Token,false,SdfVariabilityUniform).Set(TfToken("routedDials")));
        CHECK(dialProjector.CreateRelationship(TfToken("rigExec:shaderDialSources")).SetTargets(
            {SdfPath("/ProjectorAsset/Rig/Controls/Space.avars:tx")}));
        auto evaluator = MakeEvaluator(stage, dialRig);
        const UsdTimeCode t(stage->GetStartTimeCode() + 2.0);
        const auto before = RunChecked(evaluator.get(), {}, t, "shader dial, before");
        UsdAttribute dial;
        for (const auto &chain : Program(*evaluator)->chains) {
            for (const auto &derived : chain.derived) {
                for (const SdfPath &source :
                     derived.revision.binding.shaderDials) {
                    const UsdAttribute a = stage->GetAttributeAtPath(source);
                    if (!dial && a && !a.ValueMightBeTimeVarying()) {
                        dial = a;
                    }
                }
            }
        }
        CHECK(dial);
        if (dial) {
            const size_t builds = evaluator->GetBakedProgramBuildCount();
            const VtValue edited =
                dial.GetTypeName() == SdfValueTypeNames->Float
                    ? VtValue(0.625f)
                    : VtValue(0.625);
            CHECK(dial.Set(edited));
            const RigExecRigPose pose =
                RunChecked(evaluator.get(), {}, t, "shader dial, edited");
            ShadowChecked(*evaluator, {}, t, "shader dial, edited");
            CHECK(before.movedProperties != pose.movedProperties);
            CHECK(evaluator->GetBakedProgramBuildCount() == builds);
            CHECK(PoseMismatches(FreshPose(stage, dialRig, t), pose,
                                 "shader dial, edited") == 0);
            std::printf("routed shader dial %s: disposition %d\n",
                        dial.GetPath().GetText(),
                        int(evaluator->GetLastNoticeDisposition()));
        }
    }
    UsdStageRefPtr stage = UsdStage::Open(examples + "/06_LatticeBulge.usda");
    CHECK(stage);
    if (!stage) {
        return;
    }
    const SdfPath rig = RootOf(stage);
    const UsdTimeCode t(stage->GetStartTimeCode() + 12.0);
    auto evaluator = MakeEvaluator(stage, rig);
    RunChecked(evaluator.get(), {}, t, "lattice cage, before");
    const RigExecBakedProgramImpl::GeomRevision *lattice =
        FirstRevisionOf(*evaluator, RigExecRevisionOp::Lattice);
    CHECK(lattice && !lattice->binding.cagePoints.IsEmpty());
    if (!lattice || lattice->binding.cagePoints.IsEmpty()) {
        return;
    }
    const SdfPath cage = lattice->binding.cagePoints;
    UsdAttribute attribute = stage->GetAttributeAtPath(cage);
    VtVec3fArray rest;
    CHECK(attribute.Get(&rest, UsdTimeCode::Default()) && !rest.empty());
    VtVec3fArray moved = rest;
    for (GfVec3f &p : moved) {
        p *= 1.1f;
    }
    const size_t builds = evaluator->GetBakedProgramBuildCount();
    CHECK(attribute.Set(moved));
    const RigExecRigPose pose =
        RunChecked(evaluator.get(), {}, t, "lattice cage, edited");
    ShadowChecked(*evaluator, {}, t, "lattice cage, edited");
    CHECK(evaluator->GetBakedProgramBuildCount() == builds);
    CHECK(PoseMismatches(FreshPose(stage, rig, t), pose,
                         "lattice cage, edited") == 0);
    std::printf("routed lattice cage %s: disposition %d\n", cage.GetText(),
                int(evaluator->GetLastNoticeDisposition()));
}

// Frame 1 twice with nothing moved: no path leaf is re-read either.
void
CheckAHeldPathLeafIsNotResampled(const GeometryFixture &f,
                               RigExecRigEvaluator *evaluator,
                               UsdTimeCode t)
{
    const RigExecBakedProgramImpl *B = Program(*evaluator);
    CHECK(B);
    if (!B) {
        return;
    }
    const uint64_t first = B->pathLeafSamples;
    // The first run read some, so the second run's zero is a decision.
    CHECK(B->pathLeafRefs.empty() || first > 0);
    RunChecked(evaluator, {}, t, std::string(f.name) + " again");
    const uint64_t again = B->pathLeafSamples - first;
    if (again != 0) {
        std::printf("FAIL %s: %llu path leaf re-read(s) with nothing "
                    "moved\n",
                    f.name, static_cast<unsigned long long>(again));
    }
    CHECK(again == 0);
    std::printf("path leaves %s: %zu, %llu read on the first run\n",
                f.name, B->pathLeafRefs.size(),
                static_cast<unsigned long long>(first));
}

// Preserve the cold assembly check before measuring the held sample decision.
void
TestGeometryFixtureInputs(const std::string &examples)
{
    std::set<RigExecRevisionOp> reached;
    // Finite assembly/read-site representatives. Each selected fixture keeps
    // its complete cold, held, sampled and override/fresh-reference history.
    const std::set<std::string> representatives = {
        "01", "04", "05", "06", "07", "11", "computed_path_reads",
        "oneloop_cross_domain", "phased_blend_samples",
        "preceding_own_chain", "projector_spaces", "bust", "wrinkle",
        "compact_wire"};
    size_t selectedFixtures = 0;
    std::vector<GeometryFixture> fixtures = GeometryFixtures(examples);
    fixtures.push_back({"compact_wire", examples + "/../tests/fixtures/leaves_routed_geometry.usda"});
    for (const GeometryFixture &f : fixtures) {
        if (!representatives.count(f.name)) continue;
        ++selectedFixtures;
        UsdStageRefPtr stage = Open(f);
        CHECK(stage);
        if (!stage) continue;
        const SdfPath rig = RootOf(stage);
        CHECK(!rig.IsEmpty());
        auto evaluator = MakeEvaluator(stage, rig);
        const UsdTimeCode t(stage->GetStartTimeCode());
        RunChecked(evaluator.get(), {}, t, std::string(f.name) + " first");
        const size_t coldCompared = ShadowChecked(
            *evaluator, {}, t, std::string(f.name) + " at " + Text(t.GetValue()));
        CheckAHeldPathLeafIsNotResampled(f, evaluator.get(), t);
        CheckPureAssemblyEqualsTheStageAssembler(
            f, stage, rig, evaluator.get(), coldCompared, reached);
    }
    CHECK(selectedFixtures == representatives.size());
    // Every operation the leaves assemble is held to the stage assembler on
    // some fixture (an external mover's payload in testRigExecExternalMovers).
    for (const RigExecRevisionOp op :
         {RigExecRevisionOp::Matrix, RigExecRevisionOp::Skin,
          RigExecRevisionOp::BlendShape, RigExecRevisionOp::VolumeCorrect,
          RigExecRevisionOp::Smooth, RigExecRevisionOp::Lattice,
          RigExecRevisionOp::SurfaceProject, RigExecRevisionOp::Ribbon,
          RigExecRevisionOp::Wire, RigExecRevisionOp::EmitGuidePoints,
          RigExecRevisionOp::RecomputeNormals,
          RigExecRevisionOp::RecomputeExtent, RigExecRevisionOp::DeltaMush,
          RigExecRevisionOp::Wrinkle, RigExecRevisionOp::SurfaceProjector,
          RigExecRevisionOp::ShaderDials}) {
        if (!reached.count(op)) {
            std::printf("FAIL shadow: no fixture reached op %d\n", int(op));
        }
        CHECK(reached.count(op) == 1);
    }
}

// Build's settings reach a frozen clone (frozenDetail::_CloneImpl): the
// clone's partition settings and the worker's kernels are the live
// program's choices, and its bodies count into their own audit counter.
// The clone starts from other values, so a field the clone skips fails
// here.
void
TestAFrozenCloneKeepsTheBuildSettings(const std::string &examples)
{
    const RigExecExampleFixture *row = nullptr;
    for (const RigExecExampleFixture &candidate : kRigExecExampleFixtures) {
        row = &candidate;
        break;
    }
    CHECK(row);
    if (!row) {
        return;
    }
    ArchSetEnv("RIGEXEC_BAKED_CHUNK_VERTS", "7", /*overwrite=*/true);
    ArchSetEnv("RIGEXEC_BAKED_MAX_CHUNKS", "3", /*overwrite=*/true);
    ArchSetEnv("RIGEXEC_PURITY_AUDIT", "1", /*overwrite=*/true);
    UsdStageRefPtr stage = UsdStage::Open(examples + "/" + row->stage);
    CHECK(stage);
    std::unique_ptr<RigExecRigEvaluator> evaluator;
    if (stage) {
        evaluator = MakeEvaluator(stage, RootOf(stage));
        evaluator->Evaluate(UsdTimeCode(stage->GetStartTimeCode()));
    }
    ArchRemoveEnv("RIGEXEC_BAKED_CHUNK_VERTS");
    ArchRemoveEnv("RIGEXEC_BAKED_MAX_CHUNKS");
    ArchRemoveEnv("RIGEXEC_PURITY_AUDIT");
    const RigExecBakedProgramImpl *B = evaluator ? Program(*evaluator) : nullptr;
    CHECK(B);
    if (!B) {
        return;
    }
    CHECK(B->chunkVertexTarget == 7);
    CHECK(B->chunkCap == 3);
    CHECK(B->purityAudit);
    auto clone = std::make_unique<RigExecBakedProgramImpl>();
    clone->chunkVertexTarget = 1;
    clone->chunkCap = 1;
    clone->useSimd = !B->useSimd;
    clone->purityAudit = false;
    clone->purityViolations.count.store(5);
    frozenDetail::_CloneImpl(*B, clone.get());
    CHECK(clone->chunkVertexTarget == B->chunkVertexTarget);
    CHECK(clone->chunkCap == B->chunkCap);
    CHECK(clone->useSimd == B->useSimd);
    CHECK(clone->purityAudit);
    CHECK(clone->purityViolations.count.load() == 0);
}

// Body purity. The thread mark itself: a read under RigExecOpBodyScope is
// counted into the scope's counter, a nested scope restores the outer mark,
// RigExecVolatileRead lifts it, and no read outside a scope counts.
void
TestTheBodyMarkCountsReadsUnderIt()
{
    const RigExecResolvedInputs R;
    float value = 0.0f;
    std::atomic<uint64_t> outer{0}, inner{0};
    R.GetAttribute(UsdAttribute(), UsdTimeCode::Default(), &value);
    CHECK(!RigExecInOpBody());
    {
        const RigExecOpBodyScope body(&outer);
        CHECK(RigExecInOpBody());
        R.GetAttribute(UsdAttribute(), UsdTimeCode::Default(), &value);
        {
            const RigExecOpBodyScope nested(&inner);
            R.GetAttribute(UsdAttribute(), UsdTimeCode::Default(), &value);
        }
        CHECK(RigExecInOpBody());
        {
            const RigExecVolatileRead volatileRead;
            CHECK(!RigExecInOpBody());
            R.GetAttribute(UsdAttribute(), UsdTimeCode::Default(), &value);
        }
        R.GetAttribute(UsdAttribute(), UsdTimeCode::Default(), &value);
    }
    CHECK(!RigExecInOpBody());
    R.GetAttribute(UsdAttribute(), UsdTimeCode::Default(), &value);
    CHECK(outer.load() == 2);
    CHECK(inner.load() == 1);
}

// The volatile reads a program's bodies make, by name: the weight oracle
// from a Constraint body and from a current-phase RevisionStatic, and the
// stage assembly of a plugin revision bound to region values.
std::vector<std::string>
VolatileReaders(const RigExecBakedProgramImpl &B)
{
    std::vector<std::string> out;
    for (const auto &c : B.constraints) {
        if (!c.weightObject.IsEmpty() && c.pointsTarget.IsEmpty()) {
            out.push_back("oracle (constraint) " + c.path.GetString());
        }
    }
    for (const auto &chain : B.chains) {
        for (const auto &revision : chain.revisions) {
            if (revision.weightCurrentPhase && revision.weightObject >= 0) {
                out.push_back("oracle (current phase) " +
                              revision.moverPath.GetString());
            }
            if (!revision.leaves.decl.assembles) {
                out.push_back("stage assembly " +
                              revision.moverPath.GetString());
            }
        }
        for (const auto &derived : chain.derived) {
            if (!derived.matrixTarget &&
                !derived.revision.leaves.decl.assembles) {
                out.push_back("stage assembly (derived) " +
                              derived.revision.moverPath.GetString());
            }
        }
    }
    return out;
}

struct PurityFixture {
    std::string name;
    std::string stage;
    std::function<UsdStageRefPtr()> make;
    /// The example table's frames, or empty for the stage's first frames.
    std::string frames;
    /// The example table's two drags, when it names them.
    std::string controlPrim, controlAvar, operatorPrim, operatorInput;
};

// Representative body/read-source/edge classes for the unit purity invariant.
// Full registered example numerical conformance remains in the final gates.
std::vector<PurityFixture>
PurityFixtures(const std::string &examples)
{
    std::vector<PurityFixture> out;
    std::set<std::string> seen;
    const auto key = [](const std::string &path) {
        return TfNormPath(TfAbsPath(path));
    };
    // Unit purity covers body/read-funnel branches; unchanged final gates
    // retain every asset's numerical and composition histories.
    const std::set<std::string> bodyExamples = {
        "04_BlendShapeFace.usda",
        "05_TwistRibbonSpine.usda",
        "06_LatticeBulge.usda",
        "07_SurfaceDrape.usda",
        "09_PropertyMathMovers.usda",
        "11_VolumeWeights.usda",
        "13_ReadPhases.usda",
        "14_VolumeConstrainedSweep.usda",
        "16_ConnectionReadPhases.usda",
        "aimtest_points.usda",
        "par_rot_aim_redorder.usd",
    };
    for (const RigExecExampleFixture &row : kRigExecExampleFixtures) {
        if (!bodyExamples.count(row.stage)) continue;
        PurityFixture f;
        f.name = row.stage;
        f.stage = examples + "/" + row.stage;
        f.frames = row.frames;
        f.controlPrim = row.controlPrim;
        f.controlAvar = row.controlAvar;
        f.operatorPrim = row.operatorPrim;
        f.operatorInput = row.operatorInput;
        seen.insert(key(f.stage));
        out.push_back(std::move(f));
    }
    for (const Fixture &leaf : Fixtures(examples)) {
        if (std::string(leaf.name)!="computed_chains") continue;
        if (seen.insert(key(leaf.stage)).second) {
            PurityFixture f;
            f.name = leaf.name;
            f.stage = leaf.stage;
            out.push_back(std::move(f));
        }
    }
    const std::set<std::string> bodyGeometry = {
        "04",
        "05",
        "06",
        "07",
        "11",
        "13",
        "14",
        "computed_path_reads",
        "oneloop_cross_domain",
        "phased_blend_samples",
        "preceding_own_chain",
        "bust_dd_a",
        "biped_stack",
        "projector_spaces",
        "wrinkle",
    };
    for (const GeometryFixture &geometry : GeometryFixtures(examples)) {
        if (!bodyGeometry.count(geometry.name)) continue;
        if (geometry.make || seen.insert(key(geometry.stage)).second) {
            PurityFixture f;
            f.name = geometry.name;
            f.stage = geometry.stage;
            f.make = geometry.make;
            out.push_back(std::move(f));
        }
    }
    return out;
}

std::vector<UsdTimeCode>
PurityFrames(const PurityFixture &f, const UsdStageRefPtr &stage)
{
    if (f.frames.empty()) {
        return Frames(stage);
    }
    std::vector<UsdTimeCode> frames;
    for (const std::string &piece : TfStringSplit(f.frames, ",")) {
        if (!piece.empty()) {
            frames.push_back(UsdTimeCode(TfStringToDouble(piece)));
        }
    }
    return frames;
}

// No step body reads the stage or the resolved-input overlay outside a
// listed volatile read. Every fixture runs Baked with RIGEXEC_PURITY_AUDIT
// set (read at Build), at its frames with no override, under the example
// table's control and operator drags, under a drag on a solver input and on
// a chain hop, and once after each drag is lifted; after every run the
// program's purityViolations is read on this thread and must be 0. The
// schedule is the entry's (RIGEXEC_BAKED_SCHEDULE): testRigExecLeaves is
// registered serial and parallel.
void
TestNoBodyReadsTheStage(const std::string &examples)
{
    ArchSetEnv("RIGEXEC_PURITY_AUDIT", "1", /*overwrite=*/true);
    size_t fixtures = 0, runs = 0, baked = 0;
    std::vector<std::string> volatileRigs;
    for (const PurityFixture &f : PurityFixtures(examples)) {
        UsdStageRefPtr stage =
            f.make ? f.make() : UsdStage::Open(f.stage);
        CHECK(stage);
        if (!stage) {
            continue;
        }
        const SdfPath rig = RootOf(stage);
        CHECK(!rig.IsEmpty());
        if (rig.IsEmpty()) {
            continue;
        }
        auto evaluator =
            MakeEvaluator(stage, rig);
        const std::vector<UsdTimeCode> frames = PurityFrames(f, stage);
        size_t fixtureRuns = 0, fixtureBaked = 0;
        bool reported = false;
        const auto run = [&](const std::vector<RigExecValueOverride> &drag,
                             UsdTimeCode t, const std::string &what) {
            evaluator->SetInteractiveOverrides(drag);
            const size_t generations = evaluator->GetBakedGenerationCount();
            const RigExecRigPose pose = evaluator->Evaluate(t);
            CHECK(pose.valid);
            ++fixtureRuns;
            if (evaluator->GetBakedGenerationCount() == generations + 1) {
                ++fixtureBaked;
            }
            const RigExecBakedProgramImpl *B = Program(*evaluator);
            if (!B) {
                return;
            }
            CHECK(B->purityAudit);
            const uint64_t violations =
                B->purityViolations.count.load(std::memory_order_relaxed);
            if (violations != 0 && !reported) {
                reported = true;
                std::printf("FAIL %s %s at %s: %llu body read(s) of the "
                            "stage or the overlay\n",
                            f.name.c_str(), what.c_str(),
                            Text(t.GetValue()).c_str(),
                            static_cast<unsigned long long>(violations));
            }
            CHECK(violations == 0);
        };
        for (const UsdTimeCode t : frames) {
            run({}, t, "no override");
        }
        const auto dragAndLift = [&](const SdfPath &path, const char *kind) {
            bool dragged = false;
            for (const UsdTimeCode t : frames) {
                const std::vector<RigExecValueOverride> drag =
                    DragBy(stage, path, t, 0.25);
                if (drag.empty()) {
                    return;
                }
                run(drag, t, std::string(kind) + " " + path.GetString());
                dragged = true;
            }
            if (dragged) {
                run({}, frames.back(),
                    std::string("lifted ") + kind + " " + path.GetString());
            }
        };
        if (!f.controlPrim.empty()) {
            dragAndLift(SdfPath(f.controlPrim)
                            .AppendProperty(TfToken(f.controlAvar)),
                        "control");
        }
        if (!f.operatorPrim.empty()) {
            dragAndLift(SdfPath(f.operatorPrim)
                            .AppendProperty(TfToken(f.operatorInput)),
                        "operator input");
        }
        if (const RigExecBakedProgramImpl *B = Program(*evaluator)) {
            const std::vector<SdfPath> solverInputs = SolverInputs(*B);
            const std::vector<SdfPath> hops = ChainHops(*B);
            if (!solverInputs.empty()) {
                dragAndLift(solverInputs.front(), "solver input");
            }
            if (!hops.empty()) {
                dragAndLift(hops.front(), "chain hop");
            }
            for (const std::string &reader : VolatileReaders(*B)) {
                volatileRigs.push_back(f.name + ": " + reader);
            }
        }
        evaluator->ClearInteractiveOverrides();
        // Require actual graph execution rather than an empty purity sweep.
        CHECK(fixtureBaked > 0);
        std::printf("purity %s: %zu run(s), %zu baked\n", f.name.c_str(),
                    fixtureRuns, fixtureBaked);
        ++fixtures;
        runs += fixtureRuns;
        baked += fixtureBaked;
    }
    std::printf("purity: %zu fixture(s), %zu run(s), %zu baked; volatile "
                "readers (inside RigExecVolatileRead):\n",
                fixtures, runs, baked);
    for (const std::string &reader : volatileRigs) {
        std::printf("    %s\n", reader.c_str());
    }
    // The volatile scope is exercised: some fixture reaches the oracle.
    CHECK(!volatileRigs.empty());
}

// A cubic wire over ten points whose envelope is sparse with a zero
// default, so the kernel evaluates only the named points from a basis.
RigExecMoverParameters
SparseWirePacket()
{
    RigExecMoverParameters p;
    p.kind = TfToken("wire");
    p.valid = true;
    p.curveOrder = 4;
    p.curveKnots = {0.0, 0.0, 0.0, 0.0, 1.0, 1.0, 1.0, 1.0};
    for (int j = 0; j < 4; ++j) {
        p.restPoints.emplace_back(float(j), 0.0f, 0.0f);
        p.auxPoints.emplace_back(float(j), 0.25f * float(j * j), 0.5f);
    }
    for (int i = 0; i < 10; ++i) {
        p.wireBindCoords.push_back(GfVec2f(0.1f * float(i), 0.05f * float(i)));
    }
    p.dropoffDistance = 2.0;
    p.weights.representation = TfToken("sparse");
    p.weights.rangePolicy = TfToken("strict");
    p.weights.indices = {1, 3, 5, 7};
    p.weights.values = {1.0f, 0.5f, 0.25f, 1.0f};
    p.weights.defaultWeight = 0.0f;
    p.weights.valid = true;
    return p;
}

std::vector<GfVec3f>
WireMesh()
{
    std::vector<GfVec3f> mesh;
    for (int i = 0; i < 10; ++i) {
        mesh.emplace_back(0.3f * float(i), 0.1f, -0.2f * float(i));
    }
    return mesh;
}

// The wire-basis memo is owned, not shared (RigExecWireBasisCache): through
// a cache the kernel answers bit-for-bit what it answers building the basis
// for one call; a repeat builds nothing; a moved bind table rebuilds and
// answers the fresh build; a copy, as a frozen clone takes, shares what was
// built; the map stays under its cap; and caches on separate threads, which
// share nothing, agree with the reference.
void
TestTheWireBasisMemoIsOwned()
{
    const RigExecMoverParameters p = SparseWirePacket();
    CHECK(RigExecWireTakesSparseEnvelope(p.weights));
    std::vector<GfVec3f> reference = WireMesh();
    CHECK(RigExecRunRevisionKernel(RigExecRevisionOp::Wire, p, &reference,
                                   true, nullptr));
    CHECK(reference != WireMesh());

    RigExecWireBasisCache cache;
    for (int run = 0; run < 3; ++run) {
        std::vector<GfVec3f> points = WireMesh();
        CHECK(RigExecRunRevisionKernel(RigExecRevisionOp::Wire, p, &points,
                                       true, &cache));
        CHECK(points == reference);
    }
    CHECK(cache.BuildCount() == 1);
    CHECK(cache.Size() == 1);

    RigExecWireBasisCache copy = cache;
    {
        std::vector<GfVec3f> points = WireMesh();
        CHECK(RigExecRunRevisionKernel(RigExecRevisionOp::Wire, p, &points,
                                       true, &copy));
        CHECK(points == reference);
        CHECK(copy.BuildCount() == 1);
    }

    RigExecMoverParameters moved = p;
    moved.wireBindCoords[3] = GfVec2f(0.6f, 0.05f);
    std::vector<GfVec3f> movedReference = WireMesh();
    CHECK(RigExecRunRevisionKernel(RigExecRevisionOp::Wire, moved,
                                   &movedReference, true, nullptr));
    CHECK(movedReference != reference);
    {
        std::vector<GfVec3f> points = WireMesh();
        CHECK(RigExecRunRevisionKernel(RigExecRevisionOp::Wire, moved,
                                       &points, true, &cache));
        CHECK(points == movedReference);
        CHECK(cache.BuildCount() == 2);
    }
    // The copy still holds only what it shared: the original's rebuild did
    // not reach it.
    CHECK(copy.Size() == 1);

    for (size_t k = 0; k < RigExecWireBasisCache::kCapacity + 5; ++k) {
        RigExecMoverParameters edited = p;
        edited.dropoffDistance = 2.0 + 0.01 * double(k + 1);
        std::vector<GfVec3f> points = WireMesh();
        CHECK(RigExecRunRevisionKernel(RigExecRevisionOp::Wire, edited,
                                       &points, true, &cache));
        CHECK(cache.Size() <= RigExecWireBasisCache::kCapacity);
    }
    {
        std::vector<GfVec3f> points = WireMesh();
        CHECK(RigExecRunRevisionKernel(RigExecRevisionOp::Wire, p, &points,
                                       true, &cache));
        CHECK(points == reference);
    }

    std::atomic<int> mismatches(0);
    std::vector<std::thread> workers;
    for (int t = 0; t < 4; ++t) {
        workers.emplace_back([&]() {
            RigExecWireBasisCache own;
            for (int run = 0; run < 50; ++run) {
                std::vector<GfVec3f> points = WireMesh();
                if (!RigExecRunRevisionKernel(RigExecRevisionOp::Wire, p,
                                              &points, true, &own) ||
                    points != reference) {
                    ++mismatches;
                }
            }
            if (own.BuildCount() != 1) {
                ++mismatches;
            }
        });
    }
    for (std::thread &worker : workers) {
        worker.join();
    }
    CHECK(mismatches.load() == 0);
}

}  // namespace

// Provider leaves a run will skip, whose key a full publication would
// change now: every leaf no kRigExecSpaceLeaf* reason marks must still hold
// the key it would be given, or the next run keeps a stale one.
size_t
StaleProviderLeaves(const RigExecBakedProgramImpl &B, const std::string &what)
{
    const RigExecBakedSpaceLeafIndex *index = B.spaceLeafIndex.get();
    if (!index) {
        return 1;
    }
    size_t stale = 0;
    std::string key;
    for (size_t k = 0; k < B.spaceLeafRekey.size(); ++k) {
        if (B.spaceLeafRekey[k]) {
            continue;
        }
        const RigExecOpValueState &value =
            B.opAdapter.values[size_t(index->first + k)];
        const bool exact = RigExecBakedSpaceLeafKey(B, uint32_t(k), &key);
        if (!value.initialized || !exact || key != value.key) {
            if (stale < 4) {
                std::printf("FAIL %s: provider leaf %s kept a stale key\n",
                            what.c_str(),
                            B.providerProgram.sampled[k].attribute.GetText());
            }
            ++stale;
        }
    }
    return stale;
}

// Sparse provider-leaf publication over every route that moves a provider
// leaf's key: time, a drag on a provider input and its release, an upstream
// value, its move and its lift, and a value edit at a held frame. After each
// run the skipped leaves' keys are what a full publication would give; under
// RIGEXEC_VERIFY_SPARSE_LEAVES each run also re-keys the leaves it skips and
// verifies them unchanged. A held frame re-keys only the leaves that must.
void
TestSparseProviderLeaves(const std::string &examples)
{
    const Fixture f = FixtureNamed(Fixtures(examples), "biped");
    const UsdStageRefPtr stage = UsdStage::Open(f.stage);
    CHECK(stage);
    if (!stage) {
        return;
    }
    ArchSetEnv("RIGEXEC_VERIFY_SPARSE_LEAVES", "1", /*overwrite=*/true);
    auto evaluator = MakeEvaluator(stage, f.rig);
    ArchRemoveEnv("RIGEXEC_VERIFY_SPARSE_LEAVES");
    const RigExecBakedProgramImpl *B = Program(*evaluator);
    CHECK(B && B->spaceLeafIndex && B->spaceLeafIndex->verify);
    if (!B || !B->spaceLeafIndex) {
        return;
    }
    const auto avar = [&](const char *prim, const char *name) {
        for (const UsdPrim &p : stage->Traverse()) {
            if (p.GetName() == prim) {
                return p.GetPath().AppendProperty(TfToken(name));
            }
        }
        return SdfPath();
    };
    const SdfPath body = avar("M_Body", "avars:ry");
    const SdfPath shoulder = avar("L_Shldr", "avars:rz");
    CHECK(!body.IsEmpty() && !shoulder.IsEmpty());
    const size_t all = B->spaceLeafRekey.size();
    TfErrorMark mark;
    const auto step = [&](double time, const std::string &what) {
        const RigExecRigPose pose = evaluator->Evaluate(UsdTimeCode(time));
        CHECK(pose.valid);
        const RigExecBakedProgramImpl *P = Program(*evaluator);
        CHECK(P);
        CHECK(P && StaleProviderLeaves(*P, what) == 0);
        return P ? P->spaceLeafKeys : 0;
    };
    CHECK(step(1, "first") == all);
    const size_t held = step(1, "held");
    CHECK(held < all);
    step(2, "time");
    CHECK(step(2, "held after time") == held);
    evaluator->SetInteractiveOverrides({DragOf(body, 20.0)});
    const size_t drag = step(2, "drag");
    CHECK(drag > held && drag < all);
    evaluator->SetInteractiveOverrides({DragOf(body, 25.0)});
    CHECK(step(2, "drag moved") == drag);
    evaluator->ClearInteractiveOverrides();
    CHECK(step(2, "drag released") == drag);
    CHECK(step(2, "held after release") == held);
    evaluator->SetUpstreamInputs({DragOf(shoulder, 30.0)});
    const size_t upstream = step(2, "upstream");
    CHECK(upstream > held && upstream < all);
    evaluator->SetUpstreamInputs({DragOf(shoulder, 35.0)});
    CHECK(step(2, "upstream moved") == upstream);
    evaluator->SetUpstreamInputs({});
    CHECK(step(2, "upstream lifted") == upstream);
    CHECK(step(2, "held after upstream") == held);
    std::printf("sparse provider leaves: %zu, re-keyed held %zu, dragged %zu, "
                "upstream %zu\n", all, held, drag, upstream);
    // An authored value edit, patched in place at the held frame.
    CHECK(!B->patchableAvars.empty());
    if (!B->patchableAvars.empty()) {
        UsdAttribute attribute =
            stage->GetAttributeAtPath(B->patchableAvars.begin()->first);
        double value = 0.0;
        attribute.Get(&value, UsdTimeCode::Default());
        CHECK(attribute.Set(value + 0.5));
        step(2, "value edit");
    }
    step(3, "time after the edits");
    CHECK(mark.IsClean());
}

// Three constraints whose operator arrays the prologue reads raw: a
// PositionConstraint's animated source weights, a ParentConstraint's static
// translation offsets (its rotation offsets authored only by a variant),
// and a SingleChainIK in object pole mode, which reads its pole weights at
// their schema fallback until something authors them.
const char *const kConstraintArrays = R"usda(#usda 1.0
(
    endTimeCode = 10
    startTimeCode = 1
    timeCodesPerSecond = 24
    upAxis = "Y"
)

def Xform "Asset"
{
    matrix4d xformOp:transform = ( (1, 0, 0, 0), (0, 1, 0, 0), (0, 0, 1, 0), (0, 0, 0, 1) )
    uniform token[] xformOpOrder = ["xformOp:transform"]

    def "Sources"
    {
        def Xform "A"
        {
            matrix4d xformOp:transform = ( (1, 0, 0, 0), (0, 1, 0, 0), (0, 0, 1, 0), (10, 0, 0, 1) )
            uniform token[] xformOpOrder = ["xformOp:transform"]
        }

        def Xform "B"
        {
            matrix4d xformOp:transform = ( (1, 0, 0, 0), (0, 1, 0, 0), (0, 0, 1, 0), (0, 10, 0, 1) )
            uniform token[] xformOpOrder = ["xformOp:transform"]
        }

        def Xform "Effector"
        {
            matrix4d xformOp:transform = ( (1, 0, 0, 0), (0, 1, 0, 0), (0, 0, 1, 0), (3, 2, 0, 1) )
            uniform token[] xformOpOrder = ["xformOp:transform"]
        }

        def Xform "PoleA"
        {
            matrix4d xformOp:transform = ( (1, 0, 0, 0), (0, 1, 0, 0), (0, 0, 1, 0), (2, 0, 8, 1) )
            uniform token[] xformOpOrder = ["xformOp:transform"]
        }

        def Xform "PoleB"
        {
            matrix4d xformOp:transform = ( (1, 0, 0, 0), (0, 1, 0, 0), (0, 0, 1, 0), (2, 8, 0, 1) )
            uniform token[] xformOpOrder = ["xformOp:transform"]
        }
    }

    def "Targets"
    {
        def Xform "Position"
        {
            matrix4d xformOp:transform = ( (1, 0, 0, 0), (0, 1, 0, 0), (0, 0, 1, 0), (1, 1, 5, 1) )
            uniform token[] xformOpOrder = ["xformOp:transform"]
        }

        def Xform "Parent"
        {
            matrix4d xformOp:transform = ( (1, 0, 0, 0), (0, 1, 0, 0), (0, 0, 1, 0), (0, 0, 0, 1) )
            uniform token[] xformOpOrder = ["xformOp:transform"]
        }
    }

    def RigExecRoot "Rig"
    {
        def "Joints"
        {
            def RigExecJoint "Root"
            {
                matrix4d posed:space = ( (1, 0, 0, 0), (0, 1, 0, 0), (0, 0, 1, 0), (0, 0, 0, 1) )

                def RigExecJoint "Mid"
                {
                    matrix4d posed:space = ( (1, 0, 0, 0), (0, 1, 0, 0), (0, 0, 1, 0), (2, 0, 0, 1) )

                    def RigExecJoint "End"
                    {
                        matrix4d posed:space = ( (1, 0, 0, 0), (0, 1, 0, 0), (0, 0, 1, 0), (4, 0, 0, 1) )
                    }
                }
            }
        }

        def Scope "Movers"
        {
            def RigExecPositionConstraint "Position" (
                prepend apiSchemas = ["RigExecMoverAPI"]
            )
            {
                rel rigExec:moves = </Asset/Targets/Position>
                rel rigExec:sources = [</Asset/Sources/A>, </Asset/Sources/B>]
                float[] inputs:sourceWeights.timeSamples = {
                    1: [1, 0],
                    10: [0, 1],
                }
            }

            def RigExecParentConstraint "Parent" (
                prepend apiSchemas = ["RigExecMoverAPI"]
                variants = {
                    string turn = "none"
                }
                prepend variantSets = "turn"
            )
            {
                rel rigExec:moves = </Asset/Targets/Parent>
                rel rigExec:sources = </Asset/Sources/A>
                double3[] inputs:translationOffsets = [(1, 0, 0)]

                variantSet "turn" = {
                    "none" {
                    }
                    "quarter" {
                        double3[] inputs:rotationOffsets = [(0, 0, 90)]
                    }
                }
            }

            def RigExecSingleChainIkConstraint "IK" (
                prepend apiSchemas = ["RigExecMoverAPI"]
            )
            {
                rel rigExec:moves = [</Asset/Rig/Joints/Root>, </Asset/Rig/Joints/Root/Mid>, </Asset/Rig/Joints/Root/Mid/End>]
                rel rigExec:firstJoint = </Asset/Rig/Joints/Root>
                rel rigExec:endJoint = </Asset/Rig/Joints/Root/Mid/End>
                rel rigExec:effector = </Asset/Sources/Effector>
                double3 inputs:poleVector = (0, 0, 1)
                uniform token rigExec:poleVectorMode = "object"
                rel rigExec:poleVectorObjects = [</Asset/Sources/PoleA>, </Asset/Sources/PoleB>]
            }
        }
    }
}
)usda";

// The IK's pole weights, from a sublayer.
const char *const kConstraintArraysSublayer = R"usda(#usda 1.0

over "Asset"
{
    over "Rig"
    {
        over "Movers"
        {
            over "IK"
            {
                float[] inputs:poleVectorWeights = [0, 1]
            }
        }
    }
}
)usda";

// The constraint operator arrays are epoch state (ConstraintArrays::
// variance): the prologue reads a channel again only when the time moves
// and its read can move with it, or the time moves to or from Default, and
// every channel after any stage notice; the frozen plain sampler serves the
// fixed channels from its memo on the same terms. Every route that can move
// a raw read is taken at a held frame -- a property first authored on a
// channel no bake folds, a value edit, time samples added and removed, a
// connection made and cleared, a sublayer added, muted, unmuted and
// removed, a variant switched -- and so are the two that cannot, an
// interactive override and an upstream value. After every generation each
// raw channel equals a read made now, the pose equals a freshly compiled
// program's, and a program that stood through a notice read every channel.
void
TestConstraintArraysAreEpochState()
{
    using Arrays = RigExecBakedProgramImpl::ConstraintArrays;
    const SdfPath rig("/Asset/Rig");
    const SdfPath position("/Asset/Rig/Movers/Position");
    const SdfPath parent("/Asset/Rig/Movers/Parent");
    const SdfPath ik("/Asset/Rig/Movers/IK");
    const SdfPath mid("/Asset/Rig/Joints/Root/Mid");
    const UsdStageRefPtr stage = UsdStage::CreateInMemory();
    CHECK(rigExec::RigExecInputReplayImportFromString(stage->GetRootLayer(),
                                                      kConstraintArrays));
    auto evaluator = MakeEvaluator(stage, rig);
    const uint64_t kRebuilt = ~uint64_t(0);
    // The channels of the standing program.
    const auto channels = [&]() -> uint64_t {
        const RigExecBakedProgramImpl *B = Program(*evaluator);
        return B ? 4 * uint64_t(B->constraintArrays.size()) : 0;
    };
    // Every raw channel of the standing program against a read made now,
    // as the prologue made it before the channels were epoch state.
    const auto rawMismatches = [&](UsdTimeCode time, const std::string &what) {
        const RigExecBakedProgramImpl *B = Program(*evaluator);
        if (!B) {
            return size_t(1);
        }
        size_t mismatches = 0;
        for (const Arrays &arrays : B->constraintArrays) {
            for (size_t channel = 0; channel < 4; ++channel) {
                const TfToken &name = Arrays::Names()[channel];
                VtValue value;
                if (const UsdAttribute attribute =
                        arrays.prim.GetAttribute(name)) {
                    attribute.Get(&value, time);
                }
                if (!(value == arrays.raw[channel])) {
                    std::printf("FAIL %s: a stale raw read of %s\n",
                                what.c_str(),
                                arrays.path.AppendProperty(name).GetText());
                    ++mismatches;
                }
            }
        }
        return mismatches;
    };
    // A freshly compiled program's generation under the same inputs.
    const auto reference =
        [&](UsdTimeCode time, const std::vector<RigExecValueOverride> &overrides,
            const std::vector<RigExecValueOverride> &upstream) {
            auto fresh = MakeEvaluator(stage, rig);
            if (!overrides.empty()) {
                fresh->SetInteractiveOverrides(overrides);
            }
            if (!upstream.empty()) {
                fresh->SetUpstreamInputs(upstream);
            }
            return fresh->Evaluate(time);
        };
    // One generation at \p time, checked; the channels the standing program
    // read for it, or kRebuilt when another program answered. With an
    // override or an upstream value standing the generation may come from
    // elsewhere, so only its values are checked.
    RigExecRigPose last;
    const auto step =
        [&](UsdTimeCode time, const std::string &what,
            const std::vector<RigExecValueOverride> &overrides = {},
            const std::vector<RigExecValueOverride> &upstream = {}) {
            const RigExecBakedProgramImpl *before = Program(*evaluator);
            const uint64_t was = before ? before->constraintArrayReads : 0;
            const size_t builds = evaluator->GetBakedProgramBuildCount();
            const size_t generations = evaluator->GetBakedGenerationCount();
            last = evaluator->Evaluate(time);
            CHECK(last.valid);
            if (overrides.empty() && upstream.empty()) {
                CHECK(evaluator->GetBakedGenerationCount() == generations + 1);
            }
            CHECK(PoseMismatches(reference(time, overrides, upstream), last,
                                 what) == 0);
            CHECK(rawMismatches(time, what) == 0);
            const RigExecBakedProgramImpl *after = Program(*evaluator);
            if (!after || after != before ||
                evaluator->GetBakedProgramBuildCount() != builds) {
                return kRebuilt;
            }
            return after->constraintArrayReads - was;
        };
    // A notice re-reads every channel of a program that stands through it.
    const auto reread = [&](uint64_t read) {
        return read == kRebuilt || read == channels();
    };
    const auto midOf = [&mid](const RigExecRigPose &pose) {
        const auto found = pose.jointFramesFinal.find(mid);
        return found != pose.jointFramesFinal.end() ? found->second.Origin()
                                                    : GfVec3d(0);
    };
    // The frozen plain sampler's array samples against reads made now:
    // present exactly where the attribute is, with the value, hasValue and
    // blocked bits a read gives.
    const auto checkFrozen = [&](UsdTimeCode time, const std::string &what) {
        RigExecFrameInputs inputs;
        std::string error;
        const bool sampled =
            RigExecSampleFrameInputs(*evaluator, time, {}, &inputs, &error);
        if (!sampled) {
            std::printf("FAIL %s: frozen sampling: %s\n", what.c_str(),
                        error.c_str());
        }
        CHECK(sampled);
        const RigExecBakedProgramImpl *B = Program(*evaluator);
        if (!sampled || !B) {
            return;
        }
        size_t checked = 0;
        for (const Arrays &arrays : B->constraintArrays) {
            for (size_t channel = 0; channel < 4; ++channel) {
                if (!arrays.Sampled(channel)) {
                    continue;
                }
                const SdfPath key =
                    arrays.path.AppendProperty(Arrays::Names()[channel]);
                const RigExecSampledInput *sample = nullptr;
                for (const RigExecSampledInput &s : inputs.values) {
                    if (s.path == key) {
                        sample = &s;
                        break;
                    }
                }
                const UsdAttribute attribute = stage->GetAttributeAtPath(key);
                if (!attribute) {
                    CHECK(!sample);
                    continue;
                }
                CHECK(sample);
                if (!sample) {
                    continue;
                }
                VtValue value;
                const bool hasValue = attribute.Get(&value, time);
                const bool blocked =
                    attribute.GetResolveInfo(time).ValueIsBlocked();
                if (sample->hasValue != hasValue ||
                    sample->valueBlocked != blocked ||
                    !(sample->value == value)) {
                    std::printf("FAIL %s: the frozen sample of %s differs "
                                "from a read\n", what.c_str(), key.GetText());
                    ++failures;
                }
                ++checked;
            }
        }
        CHECK(checked > 0);
    };

    // Held, moved, and to and from Default: one animated channel.
    step(UsdTimeCode(1), "first");
    CHECK(step(UsdTimeCode(1), "held") == 0);
    CHECK(step(UsdTimeCode(2), "time") == 1);
    CHECK(step(UsdTimeCode(2), "held after time") == 0);
    CHECK(step(UsdTimeCode::Default(), "to Default") == channels());
    CHECK(step(UsdTimeCode(3), "from Default") == channels());
    CHECK(step(UsdTimeCode(4), "time again") == 1);
    checkFrozen(UsdTimeCode(4), "frozen");
    checkFrozen(UsdTimeCode(5), "frozen, time");
    if (const RigExecBakedProgramImpl *B = Program(*evaluator)) {
        CHECK(B->frozenArraySamples.size() == B->constraintArrays.size());
        for (size_t row = 0; row < B->constraintArrays.size() &&
                             row < B->frozenArraySamples.size(); ++row) {
            const auto &memo = B->frozenArraySamples[row];
            if (B->constraintArrays[row].path == parent) {
                CHECK(memo[1].variance == Arrays::kFixed && memo[1].present);
            } else if (B->constraintArrays[row].path == position) {
                CHECK(memo[0].variance == Arrays::kVaries);
            }
        }
    }
    checkFrozen(UsdTimeCode::Default(), "frozen, Default");

    // A property first authored on a channel the program does not fold:
    // a PositionConstraint consumes no offsets, but the prologue reads them.
    const SdfPath unread = position.AppendProperty(
        TfToken("inputs:translationOffsets"));
    CHECK(stage->GetPrimAtPath(position)
              .CreateAttribute(unread.GetNameToken(),
                               SdfValueTypeNames->Double3Array)
              .Set(VtVec3dArray{GfVec3d(1, 2, 3)}));
    uint64_t read = step(UsdTimeCode(4), "unfolded channel authored");
    CHECK(reread(read));
    std::printf("constraint arrays: an unfolded channel authored %s the "
                "program (disposition %d)\n",
                read == kRebuilt ? "rebuilt" : "kept",
                int(evaluator->GetLastNoticeDisposition()));
    CHECK(step(UsdTimeCode(4), "held after authoring") == 0);

    // A value edit.
    const SdfPath offsets =
        parent.AppendProperty(TfToken("inputs:translationOffsets"));
    CHECK(stage->GetAttributeAtPath(offsets).Set(
        VtVec3dArray{GfVec3d(0, 2, 0)}));
    CHECK(reread(step(UsdTimeCode(4), "value edit")));
    checkFrozen(UsdTimeCode(4), "frozen, value edit");

    // Time samples added, then removed.
    CHECK(stage->GetAttributeAtPath(offsets).Set(
        VtVec3dArray{GfVec3d(3, 0, 0)}, UsdTimeCode(6)));
    CHECK(reread(step(UsdTimeCode(4), "time sample added")));
    step(UsdTimeCode(6), "time sample added, at it");
    step(UsdTimeCode(7), "time sample added, past it");
    checkFrozen(UsdTimeCode(7), "frozen, time sample added");
    CHECK(stage->GetAttributeAtPath(offsets).ClearAtTime(UsdTimeCode(6)));
    CHECK(reread(step(UsdTimeCode(7), "time sample removed")));
    checkFrozen(UsdTimeCode(7), "frozen, time sample removed");

    // A connection made and cleared: the raw read never follows one.
    CHECK(stage->GetAttributeAtPath(unread).SetConnections({offsets}));
    CHECK(reread(step(UsdTimeCode(7), "connection made")));
    CHECK(stage->GetAttributeAtPath(unread).ClearConnections());
    CHECK(reread(step(UsdTimeCode(7), "connection cleared")));

    // A sublayer authoring the pole weights: added, muted, unmuted and
    // removed. The pole moves the chain's middle joint.
    const GfVec3d unweighted = midOf(last);
    const SdfLayerRefPtr sublayer =
        SdfLayer::CreateAnonymous("constraintArrays");
    CHECK(rigExec::RigExecInputReplayImportFromString(
        sublayer, kConstraintArraysSublayer));
    stage->GetRootLayer()->InsertSubLayerPath(sublayer->GetIdentifier());
    CHECK(reread(step(UsdTimeCode(7), "sublayer added")));
    CHECK((midOf(last) - unweighted).GetLength() > 1e-6);
    checkFrozen(UsdTimeCode(7), "frozen, sublayer added");
    stage->MuteLayer(sublayer->GetIdentifier());
    CHECK(reread(step(UsdTimeCode(7), "sublayer muted")));
    CHECK((midOf(last) - unweighted).GetLength() <= 1e-6);
    stage->UnmuteLayer(sublayer->GetIdentifier());
    CHECK(reread(step(UsdTimeCode(7), "sublayer unmuted")));
    stage->GetRootLayer()->RemoveSubLayerPath(0);
    CHECK(reread(step(UsdTimeCode(7), "sublayer removed")));
    checkFrozen(UsdTimeCode(7), "frozen, sublayer removed");

    // A variant switched, and back.
    CHECK(stage->GetPrimAtPath(parent).GetVariantSet("turn")
              .SetVariantSelection("quarter"));
    CHECK(reread(step(UsdTimeCode(7), "variant switched")));
    checkFrozen(UsdTimeCode(7), "frozen, variant switched");
    CHECK(stage->GetPrimAtPath(parent).GetVariantSet("turn")
              .SetVariantSelection("none"));
    CHECK(reread(step(UsdTimeCode(7), "variant switched back")));

    // An interactive override and an upstream value: neither is a stage
    // edit, and the raw read honours neither (an override on an array
    // itself is refused outright), so nothing is re-read.
    const std::vector<RigExecValueOverride> drag = {
        DragOf(ik.AppendProperty(TfToken("inputs:twistDegrees")), 30.0)};
    evaluator->SetInteractiveOverrides(drag);
    read = step(UsdTimeCode(7), "override", drag);
    CHECK(read == 0 || read == kRebuilt);
    evaluator->ClearInteractiveOverrides();
    read = step(UsdTimeCode(7), "override lifted");
    CHECK(read == 0 || read == kRebuilt);
    const std::vector<RigExecValueOverride> upstream = {
        DragOf(offsets, VtVec3dArray{GfVec3d(0, 5, 0)})};
    evaluator->SetUpstreamInputs(upstream);
    read = step(UsdTimeCode(7), "upstream", {}, upstream);
    CHECK(read == 0 || read == kRebuilt);
    evaluator->SetUpstreamInputs({});
    read = step(UsdTimeCode(7), "upstream lifted");
    CHECK(read == 0 || read == kRebuilt);

    step(UsdTimeCode(8), "time after the routes");
    checkFrozen(UsdTimeCode(8), "frozen, time after the routes");
}

int
main(int argc, char **argv)
{
    if (argc < 2) {
        std::printf("usage: testRigExecLeaves <examples dir>\n");
        return 2;
    }
    PlugRegistry::GetInstance().RegisterPlugins(
        TfAbsPath(RIGEXEC_SCHEMA_RESOURCE_DIR));
    const std::string examples = argv[1];
    TestLayeredSourceOverlay();
    TestBindingFixtureInputs(examples);
    TestADragScrubbedAtAHeldFrameReachesTheBody(examples);
    TestAnotherChainsDragReachesAChainRoutedReader(examples);
    TestProviderMatrixFallbackAdmission();
    TestWalkResolverEqualsGetAttribute(examples);
    TestRoutedValuesReachTheirLeaves(examples);
    TestAnAvarPatchExportsThePatchedBinding(examples);
    TestGeometryFixtureInputs(examples);
    TestADragUpstreamOfAMoverInputReachesItsLeaf(examples);
    TestAnUndeclaredReadIsNamed(examples);
    TestRevisionLeavesCoverTheExporterEnumeration(examples);
    TestTheBodyMarkCountsReadsUnderIt();
    TestAFrozenCloneKeepsTheBuildSettings(examples);
    TestTheWireBasisMemoIsOwned();
    TestNoBodyReadsTheStage(examples);
    TestSparseProviderLeaves(examples);
    TestConstraintArraysAreEpochState();
    std::printf("testRigExecLeaves: %d failure(s)\n", failures);
    return failures ? 1 : 0;
}
