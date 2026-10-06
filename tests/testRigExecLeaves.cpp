// Sampled leaves: every binding frozenDetail::_ForEachPatchableInput visits is
// read once per run, in the prologue, on the owning thread, and every step
// body and the input fill read that leaf instead of resolving the binding
// themselves. The path leaves -- the reads a revision's packet assembly and
// a weight object's gathers make -- are read there too, and the packet is
// held to the stage assembler's, their keys to the exporter's enumeration.
// These cases hold the leaf to the read it replaced
// (RigExecBakedRead evaluated again after the run, bit for bit; a reader
// walk to RigExecResolvedInputs::GetAttribute from its head) under drags,
// lifted drags, drags scrubbed at a held frame, another chain's drag and
// stage edits; hold the poses to the exec reference or to a program built
// fresh after the edit; pin that nothing is re-read when nothing moved; pin
// the numbering to the visitor's order and count; and pin what
// RigExecProgramAvarPatch leaves in a binding, which the exporter reads.
// Registered plain and under the parity entries; under
// RIGEXEC_BAKED_VERIFY_CONES every run is also checked against a forced run
// of the whole program. The evaluators here run Baked whatever the entry:
// the leaf check re-reads through the generation's resolved inputs, which
// a parity generation's dynamic walk would overwrite.
// The purity cases hold every step body away from the stage and the
// resolved-input overlay (RIGEXEC_PURITY_AUDIT, bodyPurity.h) over every
// bakeable example and every fixture above.
// argv[1] = path to the examples directory.
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
    /// _ForEachPatchableInput's count at the parent of the commit that
    /// numbered the leaves. No later commit adds a binding to the visitor:
    /// the exporter refuses a bake whose count differs from its tables.
    size_t visited;
};

std::vector<Fixture>
Fixtures(const std::string &examples)
{
    return {
        {"03", examples + "/03_IkFkBlendClamp.usda",
         SdfPath("/BlendArmAsset/Rig"), 315},
        {"09", examples + "/09_PropertyMathMovers.usda",
         SdfPath("/PropMathAsset/Rig"), 87},
        {"11", examples + "/11_VolumeWeights.usda",
         SdfPath("/VolumeAsset/Rig"), 299},
        {"16", examples + "/16_ConnectionReadPhases.usda",
         SdfPath("/PhaseConnectAsset/Rig"), 68},
        {"biped", examples + "/biped/Biped_anim.usda", SdfPath("/Biped/Rig"),
         13197},
        {"computed_chains",
         examples + "/../tests/fixtures/computed_chains.usda",
         SdfPath("/Asset/Rig"), 64},
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
        const T leaf = RigExecBakedLeaf(B, input);
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
MakeEvaluator(const UsdStageRefPtr &stage, const SdfPath &rig,
              RigExecEvaluationMode mode)
{
    auto evaluator = std::make_unique<RigExecRigEvaluator>(stage, rig);
    std::vector<std::string> errors;
    CHECK(evaluator->Compile(&errors));
    for (const std::string &e : errors) {
        std::printf("    compile: %s\n", e.c_str());
    }
    evaluator->SetEvaluationMode(mode);
    return evaluator;
}

// The exec reference's pose at \p time under \p overrides.
RigExecRigPose
Reference(const UsdStageRefPtr &stage, const SdfPath &rig, UsdTimeCode time,
          const std::vector<RigExecValueOverride> &overrides)
{
    auto reference =
        MakeEvaluator(stage, rig, RigExecEvaluationMode::ExecReference);
    reference->SetInteractiveOverrides(overrides);
    return reference->Evaluate(time);
}

// A program built fresh on \p stage: its first generation runs everything.
RigExecRigPose
FreshPose(const UsdStageRefPtr &stage, const SdfPath &rig, UsdTimeCode time)
{
    auto fresh = MakeEvaluator(stage, rig, RigExecEvaluationMode::Baked);
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
    pose.moverGraphRevisionsCreated = fresh.moverGraphRevisionsCreated;
    pose.moverGraphRevisionsExecuted = fresh.moverGraphRevisionsExecuted;
    pose.moverGraphSchedulesBuilt = fresh.moverGraphSchedulesBuilt;
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
    if (diff.bakedParityMismatches != 0 && !quiet) {
        std::printf("FAIL %s: %zu mismatch(es):\n", what.c_str(),
                    diff.bakedParityMismatches);
        for (size_t i = 0; i < diff.diagnostics.size() && i < 8; ++i) {
            std::printf("    %s\n", diff.diagnostics[i].c_str());
        }
    }
    return diff.bakedParityMismatches;
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
    CHECK(pose.bakedParityMismatches == 0);
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
        CHECK(stage->GetRootLayer()->ImportFromString(kFloatAtDouble));
        stages.emplace_back("float at double", stage);
        rigs.push_back(SdfPath("/Asset/Rig"));
    }
    size_t checked = 0, doubleTails = 0, skipped = 0;
    for (size_t i = 0; i < stages.size(); ++i) {
        const std::string &name = stages[i].first;
        const UsdStageRefPtr &stage = stages[i].second;
        auto evaluator =
            MakeEvaluator(stage, rigs[i], RigExecEvaluationMode::Baked);
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
TestBodyLeavesEqualTheFunnel(const std::string &examples)
{
    for (const Fixture &f : Fixtures(examples)) {
        UsdStageRefPtr stage = UsdStage::Open(f.stage);
        CHECK(stage);
        if (!stage) {
            continue;
        }
        auto evaluator =
            MakeEvaluator(stage, f.rig, RigExecEvaluationMode::Baked);
        const std::vector<UsdTimeCode> frames = Frames(stage);
        const std::string name = f.name;
        for (const UsdTimeCode t : frames) {
            RunChecked(evaluator.get(), {}, t,
                       name + " no override at " + Text(t.GetValue()));
        }
        const RigExecBakedProgramImpl *B = Program(*evaluator);
        CHECK(B);
        if (!B) {
            continue;
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
                RunChecked(evaluator.get(), drag, t,
                           name + " " + kind + " " + path.GetString() +
                               " at " + Text(t.GetValue()));
            }
            RunChecked(evaluator.get(), {}, frames.back(),
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
        if (PoseMismatches(Reference(stage, rig, time, a),
                           Reference(stage, rig, time, b), "probe",
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
    auto evaluator = MakeEvaluator(stage, f.rig, RigExecEvaluationMode::Baked);
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
        CHECK(PoseMismatches(Reference(stage, f.rig, t, drag), pose, what) ==
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
        if (PoseMismatches(Reference(stage, f.rig, t, {}),
                           Reference(stage, f.rig, t, candidate), "probe",
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
    auto evaluator = MakeEvaluator(stage, f.rig, RigExecEvaluationMode::Baked);
    RunChecked(evaluator.get(), {}, t, "chain drag, before");
    const std::string what = "chain drag on " + mover.GetString();
    const RigExecRigPose pose = RunChecked(evaluator.get(), drag, t, what);
    CHECK(PoseMismatches(Reference(stage, f.rig, t, drag), pose, what) == 0);
    // The reader's leaf is what moved: some chain-routed leaf changed.
    const RigExecBakedProgramImpl *B = Program(*evaluator);
    size_t routedChanged = 0;
    frozenDetail::_ForEachPatchableInput(*B, [&](const auto &input) {
        using T = std::decay_t<decltype(input.constant)>;
        if (input.resolvedAttr && input.leaf >= 0 &&
            B->leaves.template Of<T>().changed[size_t(input.leaf)]) {
            ++routedChanged;
        }
    });
    CHECK(routedChanged > 0);
    CHECK(B->overridableInputs.count(mover) == 0);
    std::printf("another chain's drag: %s moved %zu chain-routed leaf(s)\n",
                mover.GetText(), routedChanged);
    RunChecked(evaluator.get(), {}, t, "chain drag, lifted");
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
    auto evaluator = MakeEvaluator(stage, f.rig, RigExecEvaluationMode::Baked);
    const std::string name = std::string(f.name) + " " + what;
    RunChecked(evaluator.get(), {}, t, name + ", before");
    const std::vector<RigExecValueOverride> drag = {RigExecValueOverride{
        property.GetPrimPath(), TfToken(), property.GetNameToken(), dragged}};
    RigExecRigPose pose = RunChecked(evaluator.get(), drag, t, name + ", drag");
    CHECK(PoseMismatches(Reference(stage, f.rig, t, drag), pose,
                         name + ", drag") == 0);
    pose = RunChecked(evaluator.get(), {}, t, name + ", released");
    CHECK(PoseMismatches(Reference(stage, f.rig, t, {}), pose,
                         name + ", released") == 0);
    const size_t builds = evaluator->GetBakedProgramBuildCount();
    CHECK(attribute.Set(dragged));
    pose = RunChecked(evaluator.get(), {}, t, name + ", edited");
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
            MakeEvaluator(stage, f.rig, RigExecEvaluationMode::Baked);
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
        CHECK(PoseMismatches(Reference(stage, f.rig, t, drag), pose,
                             "curve keys, dragged") == 0);
        pose = RunChecked(evaluator.get(), {}, t, "curve keys, released");
        CHECK(PoseMismatches(Reference(stage, f.rig, t, {}), pose,
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
            MakeEvaluator(stage, f.rig, RigExecEvaluationMode::Baked);
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
        CHECK(PoseMismatches(FreshPose(stage, f.rig, t3), three,
                             name + " at 3") == 0);
        CHECK(PoseMismatches(Reference(stage, f.rig, t3, {}), three,
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

    // An avar edited through RigExecProgramAvarPatch, the Animation-mode
    // route: the binding is patched in place and its leaf re-read.
    UsdStageRefPtr stage = UsdStage::Open(f.stage);
    const UsdTimeCode t(stage->GetStartTimeCode() + 2.0);
    auto evaluator = MakeEvaluator(stage, f.rig, RigExecEvaluationMode::Baked);
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
TestALeafIsNotResampledWhenNothingMoved(const std::string &examples)
{
    for (const Fixture &f : Fixtures(examples)) {
        UsdStageRefPtr stage = UsdStage::Open(f.stage);
        CHECK(stage);
        if (!stage) {
            continue;
        }
        auto evaluator =
            MakeEvaluator(stage, f.rig, RigExecEvaluationMode::Baked);
        const UsdTimeCode t(stage->GetStartTimeCode());
        RunChecked(evaluator.get(), {}, t, std::string(f.name) + " first");
        const RigExecBakedProgramImpl *B = Program(*evaluator);
        CHECK(B);
        if (!B) {
            continue;
        }
        const uint64_t first = B->leafSamples;
        CHECK(first >= B->leafRefs.size());
        RunChecked(evaluator.get(), {}, t, std::string(f.name) + " again");
        const uint64_t again = B->leafSamples - first;
        if (again != 0) {
            std::printf("FAIL %s: %llu leaf re-read(s) with nothing moved\n",
                        f.name, static_cast<unsigned long long>(again));
        }
        CHECK(again == 0);
    }
}

// Leaf ids are dense and follow the visitor; each pool index follows it
// within its pool; every hop of every registered binding's walk is filed
// under that binding's leaf; the filed paths are exactly the override
// table's; and the visitor's count is the parent's.
void
TestLeafNumberingFollowsThePatchableOrder(const std::string &examples)
{
    for (const Fixture &f : Fixtures(examples)) {
        UsdStageRefPtr stage = UsdStage::Open(f.stage);
        CHECK(stage);
        if (!stage) {
            continue;
        }
        auto evaluator =
            MakeEvaluator(stage, f.rig, RigExecEvaluationMode::Baked);
        CHECK(evaluator->Evaluate(UsdTimeCode(stage->GetStartTimeCode()))
                  .valid);
        const RigExecBakedProgramImpl *program = Program(*evaluator);
        CHECK(program);
        if (!program) {
            continue;
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
        if (id != f.visited) {
            std::printf("FAIL %s: the visitor visits %u binding(s), the "
                        "parent visited %zu\n",
                        f.name, unsigned(id), f.visited);
        }
        CHECK(id == f.visited);
        std::printf("numbering %s: %u leaf(s), %zu path(s) filed\n", f.name,
                    unsigned(id), filed.size());
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
                MakeEvaluator(stage, f.rig, RigExecEvaluationMode::Baked);
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
            MakeEvaluator(stage, f.rig, RigExecEvaluationMode::Baked);
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
TestPureAssemblyEqualsTheStageAssembler(const std::string &examples)
{
    std::set<RigExecRevisionOp> reached;
    for (const GeometryFixture &f : GeometryFixtures(examples)) {
        UsdStageRefPtr stage = Open(f);
        CHECK(stage);
        if (!stage) {
            continue;
        }
        const SdfPath rig = RootOf(stage);
        CHECK(!rig.IsEmpty());
        auto evaluator = MakeEvaluator(stage, rig, RigExecEvaluationMode::Baked);
        const double start = stage->GetStartTimeCode();
        const std::string name = f.name;
        size_t compared = 0;
        for (const double offset : {0.0, 3.0, 8.0}) {
            const UsdTimeCode t(start + offset);
            const std::string what = name + " at " + Text(t.GetValue());
            RunChecked(evaluator.get(), {}, t, what);
            compared = std::max(compared,
                                ShadowChecked(*evaluator, {}, t, what));
        }
        const RigExecBakedProgramImpl *B = Program(*evaluator);
        CHECK(B);
        if (!B) {
            continue;
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
                RunChecked(evaluator.get(), c.overrides, t, what);
            ShadowChecked(*evaluator, c.overrides, t, what);
            CHECK(PoseMismatches(Reference(stage, rig, t, c.overrides), pose,
                                 what) == 0);
            if (c.lift) {
                RunChecked(evaluator.get(), {}, t, what + ", lifted");
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
            auto probe = MakeEvaluator(stage, rig, RigExecEvaluationMode::Baked);
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
        auto evaluator = MakeEvaluator(stage, rig, RigExecEvaluationMode::Baked);
        RunChecked(evaluator.get(), {}, t, "upstream, before");
        CHECK(!Program(*evaluator)->hasPropertyChains);
        for (const double delta : {0.25, 0.5}) {
            const auto drag = DragBy(stage, avar, t, delta);
            const std::string what = "upstream " + Text(delta);
            const RigExecRigPose pose =
                RunChecked(evaluator.get(), drag, t, what);
            CHECK(ShadowChecked(*evaluator, drag, t, what) > 0);
            CHECK(PoseMismatches(Reference(stage, rig, t, drag), pose, what) ==
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
    auto evaluator = MakeEvaluator(stage, rig, RigExecEvaluationMode::Baked);
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
    for (const GeometryFixture &f : GeometryFixtures(examples)) {
        UsdStageRefPtr stage = Open(f);
        CHECK(stage);
        if (!stage) {
            continue;
        }
        const SdfPath rig = RootOf(stage);
        auto evaluator = MakeEvaluator(stage, rig, RigExecEvaluationMode::Baked);
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
        auto fresh = MakeEvaluator(stage, rig, RigExecEvaluationMode::Baked);
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
    auto evaluator = MakeEvaluator(stage, rig, RigExecEvaluationMode::Baked);
    RunChecked(evaluator.get(), {}, t, "smooth scrub, before");
    for (const double value : {0.25, 0.9}) {
        const auto drag = DragTo(stage, input, value);
        const std::string what = "smooth scrub " + Text(value);
        const RigExecRigPose pose = RunChecked(evaluator.get(), drag, t, what);
        ShadowChecked(*evaluator, drag, t, what);
        CHECK(PoseMismatches(Reference(stage, rig, t, drag), pose, what) == 0);
    }
}

// A skin mover's unanimated defaultWeight, dragged, released and then
// authored (its spec made first, so the edit is a value notice), on the
// biped; and a weight object's painted values authored, which the program
// folds and so rebuilds for. Each against the reference or a fresh program.
void
RoutedPathLeafCases(const std::string &examples)
{
    {
        const std::string path = examples + "/biped/Biped_anim.usda";
        UsdStageRefPtr stage = UsdStage::Open(path);
        CHECK(stage);
        const SdfPath rig = RootOf(stage);
        SdfPath property;
        {
            auto probe = MakeEvaluator(stage, rig, RigExecEvaluationMode::Baked);
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
                MakeEvaluator(stage, rig, RigExecEvaluationMode::Baked);
            RunChecked(evaluator.get(), {}, t, "skin weight, before");
            const auto drag = DragTo(stage, property, 0.5);
            RigExecRigPose pose =
                RunChecked(evaluator.get(), drag, t, "skin weight, drag");
            ShadowChecked(*evaluator, drag, t, "skin weight, drag");
            CHECK(PoseMismatches(Reference(stage, rig, t, drag), pose,
                                 "skin weight, drag") == 0);
            pose = RunChecked(evaluator.get(), {}, t, "skin weight, released");
            CHECK(PoseMismatches(Reference(stage, rig, t, {}), pose,
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
        auto evaluator = MakeEvaluator(stage, rig, RigExecEvaluationMode::Baked);
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
                        SdfPath("/PathReadAsset/Rig"), 0};
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
        auto evaluator = MakeEvaluator(stage, f.rig, RigExecEvaluationMode::Baked);
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
                MakeEvaluator(stage, rig, RigExecEvaluationMode::Baked);
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
        const std::string path = examples + "/biped/Biped_stack_anim.usda";
        UsdStageRefPtr stage = UsdStage::Open(path);
        CHECK(stage);
        const SdfPath rig = RootOf(stage);
        SdfPath dropoff;
        {
            auto probe = MakeEvaluator(stage, rig, RigExecEvaluationMode::Baked);
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
            const Fixture f{"biped_stack", path, rig, 0};
            RoutedCase(f, dropoff, VtValue(37.0f), "wire dropoff");
        }
        // A shader dial's authored value (a control's avar), edited.
        auto evaluator = MakeEvaluator(stage, rig, RigExecEvaluationMode::Baked);
        const UsdTimeCode t(stage->GetStartTimeCode() + 2.0);
        RunChecked(evaluator.get(), {}, t, "shader dial, before");
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
            CHECK(evaluator->GetBakedProgramBuildCount() == builds);
            CHECK(PoseMismatches(FreshPose(stage, rig, t), pose,
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
    auto evaluator = MakeEvaluator(stage, rig, RigExecEvaluationMode::Baked);
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
TestAPathLeafIsNotResampledWhenNothingMoved(const std::string &examples)
{
    for (const GeometryFixture &f : GeometryFixtures(examples)) {
        UsdStageRefPtr stage = Open(f);
        CHECK(stage);
        if (!stage) {
            continue;
        }
        const SdfPath rig = RootOf(stage);
        auto evaluator = MakeEvaluator(stage, rig, RigExecEvaluationMode::Baked);
        const UsdTimeCode t(stage->GetStartTimeCode());
        RunChecked(evaluator.get(), {}, t, std::string(f.name) + " first");
        const RigExecBakedProgramImpl *B = Program(*evaluator);
        const uint64_t first = B->pathLeafSamples;
        // The first run read some, so the second run's zero is a decision.
        CHECK(B->pathLeafRefs.empty() || first > 0);
        RunChecked(evaluator.get(), {}, t, std::string(f.name) + " again");
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
        if (candidate.bakesToday) {
            row = &candidate;
            break;
        }
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
        evaluator = MakeEvaluator(stage, RootOf(stage),
                                  RigExecEvaluationMode::Baked);
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

// Every bakeable example in tests/exampleFixtures.cmake, then every leaf and
// geometry fixture of this file that the table does not already name.
std::vector<PurityFixture>
PurityFixtures(const std::string &examples)
{
    std::vector<PurityFixture> out;
    std::set<std::string> seen;
    const auto key = [](const std::string &path) {
        return TfNormPath(TfAbsPath(path));
    };
    for (const RigExecExampleFixture &row : kRigExecExampleFixtures) {
        if (!row.bakesToday) {
            continue;
        }
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
        if (seen.insert(key(leaf.stage)).second) {
            PurityFixture f;
            f.name = leaf.name;
            f.stage = leaf.stage;
            out.push_back(std::move(f));
        }
    }
    for (const GeometryFixture &geometry : GeometryFixtures(examples)) {
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
            MakeEvaluator(stage, rig, RigExecEvaluationMode::Baked);
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
        // A fixture whose every run fell back to the dynamic path ran no
        // body, and would pass vacuously.
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

}  // namespace

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
    TestLeafNumberingFollowsThePatchableOrder(examples);
    TestALeafIsNotResampledWhenNothingMoved(examples);
    TestBodyLeavesEqualTheFunnel(examples);
    TestADragScrubbedAtAHeldFrameReachesTheBody(examples);
    TestAnotherChainsDragReachesAChainRoutedReader(examples);
    TestWalkResolverEqualsGetAttribute(examples);
    TestRoutedValuesReachTheirLeaves(examples);
    TestAnAvarPatchExportsThePatchedBinding(examples);
    TestAPathLeafIsNotResampledWhenNothingMoved(examples);
    TestPureAssemblyEqualsTheStageAssembler(examples);
    TestADragUpstreamOfAMoverInputReachesItsLeaf(examples);
    TestAnUndeclaredReadIsNamed(examples);
    TestRevisionLeavesCoverTheExporterEnumeration(examples);
    TestTheBodyMarkCountsReadsUnderIt();
    TestAFrozenCloneKeepsTheBuildSettings(examples);
    TestNoBodyReadsTheStage(examples);
    std::printf("testRigExecLeaves: %d failure(s)\n", failures);
    return failures ? 1 : 0;
}
