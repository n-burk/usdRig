// rigExecRuntime inputs against authored values. An input set on a binary
// baked at time T plays what the evaluators publish at T with the same
// value authored on the attribute in the session layer, in the dynamic and
// the baked mode, every output and property value bit for bit, and the
// set moves the outputs off the defaults. The cases: example inputs the
// rig reads in different ways (a volume weight's falloff, a control avar a
// math chain reads, a chain target whose clamp holds the set value at 1);
// the space switches of tests/fixtures/space_switch_nested.usda, whose
// keyed indices are set and held over two runs at T, each alone and both
// together; a control's rest matrix on tests/fixtures/
// space_switch_carry.usda that a switch's recompose reads (a ladder
// input); the geometry assembly's connection-following reads of
// tests/fixtures/computed_path_reads.usda in their own types (int, bool,
// double3, matrix4d); and a delta mush disabled at T that a set enables,
// whose rest points and topology no run at T read.
#include "rigExec/rigEvaluator.h"
#include "rigExecBake/bake.h"
#include "rigExecRuntime/runtime.h"

#include "pxr/base/gf/rotation.h"
#include "pxr/base/plug/registry.h"
#include "pxr/usd/sdf/types.h"
#include "pxr/usd/usd/editContext.h"
#include "pxr/usd/usd/primRange.h"
#include "pxr/usd/usd/stage.h"

#include <cstdio>
#include <cstring>
#include <functional>
#include <limits>
#include <string>
#include <vector>

using namespace rigExec;
PXR_NAMESPACE_USING_DIRECTIVE

static int failures = 0;

#define CHECK(cond)                                                     \
    do {                                                                \
        if (!(cond)) {                                                  \
            ++failures;                                                 \
            std::printf("FAIL %s:%d: %s\n", __FILE__, __LINE__, #cond);  \
        }                                                               \
    } while (0)

#include "rigExecRuntimeDrive.h"

static SdfPath
_FindRig(const UsdStageRefPtr &stage)
{
    for (const UsdPrim &prim : stage->TraverseAll()) {
        if (prim.GetTypeName() == TfToken("RigExecRoot")) {
            return prim.GetPath();
        }
    }
    return SdfPath();
}

// One input set: a double held over later runs, or \p typed set once,
// authored as \p authored on the reference side.
struct _Set {
    std::string name;
    double value = 0.0;
    bool isTyped = false;
    RrInputValue typed;
    VtValue authored;
};

static _Set
_Double(const std::string &name, double value)
{
    _Set set;
    set.name = name;
    set.value = value;
    return set;
}

static _Set
_Typed(const std::string &name, const RrInputValue &typed,
       const VtValue &authored)
{
    _Set set;
    set.name = name;
    set.isTyped = true;
    set.typed = typed;
    set.authored = authored;
    return set;
}

static _Set
_Matrix(const std::string &name, const GfMatrix4d &matrix)
{
    RrInputValue value;
    value.tag = RrInputTag::Matrix4d;
    for (int r = 0; r < 4; ++r) {
        for (int c = 0; c < 4; ++c) {
            value.matrix[size_t(r)][size_t(c)] = matrix[r][c];
        }
    }
    return _Typed(name, value, VtValue(matrix));
}

static _Set
_Int(const std::string &name, int value)
{
    RrInputValue typed;
    typed.tag = RrInputTag::Int;
    typed.i32 = int32_t(value);
    return _Typed(name, typed, VtValue(value));
}

static _Set
_Bool(const std::string &name, bool value)
{
    RrInputValue typed;
    typed.tag = RrInputTag::Bool;
    typed.boolean = value;
    return _Typed(name, typed, VtValue(value));
}

static _Set
_Vec3d(const std::string &name, const GfVec3d &value)
{
    RrInputValue typed;
    typed.tag = RrInputTag::Vec3d;
    typed.vec = RrVec3d(value[0], value[1], value[2]);
    return _Typed(name, typed, VtValue(value));
}

static int matchedCases = 0;
static int matchedRuns = 0;

// Bakes \p stage at \p time T (NaN: the probe time) and sets \p sets on
// the binary, then runs it \p runs times at T: each run must equal,
// outputs and property values bit for bit, a fresh dynamic and a fresh
// baked evaluator at T with the same values authored in the session
// layer, and must differ from the defaults. \p check, when set, sees the
// reader after the last run.
static void
_CheckSets(const std::string &label, const UsdStageRefPtr &stage,
           double time, const std::vector<_Set> &sets, int runs,
           const std::function<void(const RigExecRuntimeReader &)> &check =
               {})
{
    const SdfPath rigPath = _FindRig(stage);
    CHECK(!rigPath.IsEmpty());
    if (rigPath.IsEmpty()) {
        return;
    }
    std::vector<uint8_t> bytes;
    std::string error;
    {
        RigExecRigEvaluator evaluator(stage, rigPath);
        evaluator.SetEvaluationMode(RigExecEvaluationMode::Baked);
        if (!RigExecTestBakeAt(evaluator, time, &bytes, &error)) {
            std::printf("%s: bake: %s\n", label.c_str(), error.c_str());
            CHECK(false);
            return;
        }
    }
    RigExecTestPlayer player;
    if (!player.Open(bytes, stage, &error)) {
        std::printf("%s: open: %s\n", label.c_str(), error.c_str());
        CHECK(false);
        return;
    }
    const double t = player->GetBakeTime();

    std::vector<RigExecTestEdit> edits;
    for (const _Set &set : sets) {
        size_t index = 0;
        CHECK(player->FindInput(set.name, &index));
        const UsdAttribute attribute =
            stage->GetAttributeAtPath(SdfPath(set.name));
        CHECK(attribute);
        if (set.isTyped) {
            CHECK(player->GetInputInfo(index).type == set.typed.tag);
            CHECK(player->SetInput(set.name, set.typed, &error));
            edits.push_back({SdfPath(set.name), set.authored});
        } else {
            CHECK(player.Hold(set.name, set.value, &error));
            edits.push_back({SdfPath(set.name),
                             RigExecTestTypedValue(attribute, set.value)});
        }
    }

    std::vector<RigExecRigPose> defaults;
    CHECK(RigExecTestEditedPoses(stage, rigPath, RigExecEvaluationMode::Baked,
                                 {}, {t}, &defaults, &error));
    struct Reference {
        const char *mode;
        RigExecRigPose pose;
    };
    std::vector<Reference> references;
    for (const auto &[mode, text] :
         {std::make_pair(RigExecEvaluationMode::Dynamic, "dynamic"),
          std::make_pair(RigExecEvaluationMode::Baked, "baked")}) {
        std::vector<RigExecRigPose> poses;
        const bool ok =
            RigExecTestEditedPoses(stage, rigPath, mode, edits, {t}, &poses,
                                   &error);
        CHECK(ok && poses.size() == 1);
        if (!ok || poses.size() != 1) {
            std::printf("%s, %s: %s\n", label.c_str(), text, error.c_str());
            return;
        }
        references.push_back({text, std::move(poses[0])});
    }

    bool same = true;
    for (int run = 0; run < runs; ++run) {
        if (!player.Play(t, &error)) {
            std::printf("%s run %d: %s\n", label.c_str(), run,
                        error.c_str());
            CHECK(false);
            return;
        }
        for (const Reference &reference : references) {
            std::vector<std::string> diffs;
            const bool outputs = RigExecCompareRuntimeOutputs(
                reference.pose, player.Reader(), &diffs);
            const bool properties = RigExecCompareRuntimeProperties(
                reference.pose, player.Reader(), &diffs);
            CHECK(outputs && properties);
            if (!outputs || !properties) {
                same = false;
                std::printf("%s run %d, %s: MISMATCH (%zu differences)\n",
                            label.c_str(), run, reference.mode,
                            diffs.size());
                for (const std::string &line : diffs) {
                    std::printf("    %s\n", line.c_str());
                }
            } else {
                ++matchedRuns;
            }
        }
    }
    // Not vacuous: the values set move the outputs.
    std::vector<std::string> ignored;
    const bool moved =
        defaults.size() == 1 &&
        (!RigExecCompareRuntimeOutputs(defaults[0], player.Reader(),
                                       &ignored) ||
         !RigExecCompareRuntimeProperties(defaults[0], player.Reader(),
                                          &ignored));
    CHECK(moved);
    if (check) {
        check(player.Reader());
    }
    if (same && moved) {
        ++matchedCases;
    }
    std::printf("%s at t=%g: %d run(s) == session edit in the dynamic and "
                "baked evaluators%s (%zu joints, %zu moved, %zu weight "
                "fields, %zu property values)\n",
                label.c_str(), t, runs, moved ? ", moved" : ", NOT moved",
                player->GetJointMatrices().size(), player->GetPoints().size(),
                player->GetWeightFields().size(),
                player->GetPropertyValues().size());
}

static UsdStageRefPtr
_Open(const std::string &path)
{
    const UsdStageRefPtr stage = UsdStage::Open(path);
    CHECK(stage);
    if (!stage) {
        std::printf("cannot open %s\n", path.c_str());
    }
    return stage;
}

int
main(int argc, char **argv)
{
    PlugRegistry::GetInstance().RegisterPlugins(
        RIGEXEC_SCHEMA_RESOURCE_DIR);
    if (argc != 3) {
        std::printf("usage: testRigExecRuntimeInputs <examples> <fixtures>\n");
        return 1;
    }
    const std::string examples = argv[1];
    const std::string fixtures = argv[2];
    const double probe = std::numeric_limits<double>::quiet_NaN();
    int cases = 0;

    // A volume weight's falloff: the shoulder field widens.
    if (const UsdStageRefPtr stage =
            _Open(examples + "/11_VolumeWeights.usda")) {
        ++cases;
        _CheckSets(
            "11 ShoulderVolume.inputs:falloffMax", stage, probe,
            {_Double("/VolumeAsset/Rig/Joints/Shoulder/ShoulderVolume."
                     "inputs:falloffMax",
                     4.5)},
            1);
    }
    // A control avar the property math movers read through connections.
    if (const UsdStageRefPtr stage =
            _Open(examples + "/09_PropertyMathMovers.usda")) {
        ++cases;
        _CheckSets("09 RootCtl.avars:ry", stage, probe,
                   {_Double("/PropMathAsset/Rig/Controls/RootCtl.avars:ry",
                            30.0)},
                   1);
    }
    // A chain target: the set value is the chain's base, and the clamp
    // mover holds the published weight at 1.
    if (const UsdStageRefPtr stage =
            _Open(examples + "/03_IkFkBlendClamp.usda")) {
        ++cases;
        const std::string weight =
            "/BlendArmAsset/Rig/Solvers/IKFKBlend.inputs:weight";
        _CheckSets("03 IKFKBlend.inputs:weight = 1.3", stage, probe,
                   {_Double(weight, 1.3)}, 1,
                   [&](const RigExecRuntimeReader &reader) {
                       bool found = false;
                       for (const RigExecRuntimePropertyValue &value :
                            reader.GetPropertyValues()) {
                           if (value.path == weight) {
                               found = true;
                               CHECK(value.value.tag ==
                                         RrPropertyValue::Tag::Float &&
                                     value.value.f32 == 1.0f);
                           }
                       }
                       CHECK(found);
                   });
    }
    // The nested space switches' keyed indices, held over two runs: S's
    // alone, P's alone, then both. Baked at frame 2, where the controls
    // have moved off their rests (at frame 0 every control rests, and a
    // switch holds its target in place whatever its index).
    if (const UsdStageRefPtr stage =
            _Open(fixtures + "/space_switch_nested.usda")) {
        const std::string s = "/Rig/Movers/sSpaces.inputs:activeSpace";
        const std::string p = "/Rig/Movers/pSpaces.inputs:activeSpace";
        cases += 3;
        _CheckSets("space_switch_nested sSpaces held", stage, 2.0,
                   {_Double(s, 0.25)}, 2);
        _CheckSets("space_switch_nested pSpaces held", stage, 2.0,
                   {_Double(p, 0.75)}, 2);
        _CheckSets("space_switch_nested both held", stage, 2.0,
                   {_Double(s, 0.25), _Double(p, 0.75)}, 2);
    }
    // A ladder input a recomposing step reads: P's rest matrix, which S's
    // switch recomposes P from.
    if (const UsdStageRefPtr stage =
            _Open(fixtures + "/space_switch_carry.usda")) {
        ++cases;
        GfMatrix4d rest(1.0);
        rest.SetRotate(GfRotation(GfVec3d(0, 0, 1), 35.0));
        rest.SetTranslateOnly(GfVec3d(6.0, 3.0, 0.0));
        _CheckSets("space_switch_carry P.rest:space", stage, 2.0,
                   {_Matrix("/Rig/Controls/G/P.rest:space", rest)}, 2);
    }

    // The geometry assembly's connection-following reads, each in its own
    // type: the mush's int iterations and bool enable, the projector's
    // double3 ray origin and matrix4d shader offset. The mush cases run at
    // frame 10, where the keyed points have left the mush's rest points (a
    // mush of its own rest returns it unchanged).
    if (const UsdStageRefPtr stage =
            _Open(fixtures + "/computed_path_reads.usda")) {
        const std::string mush = "/PathReadAsset/Rig/Movers/Mush.";
        const std::string projector =
            "/PathReadAsset/Rig/Movers/Projector.";
        GfMatrix4d offset(1.0);
        offset.SetScale(1.5);
        offset.SetTranslateOnly(GfVec3d(0.0, 0.25, -0.1));
        cases += 4;
        _CheckSets("computed_path_reads Mush.inputs:iterations = 6", stage,
                   10.0, {_Int(mush + "inputs:iterations", 6)}, 1);
        _CheckSets("computed_path_reads Mush.inputs:enabled = false", stage,
                   10.0, {_Bool(mush + "inputs:enabled", false)}, 1);
        _CheckSets("computed_path_reads Projector.rigExec:rayOrigin", stage,
                   1.0,
                   {_Vec3d(projector + "rigExec:rayOrigin",
                           GfVec3d(0.2, -0.1, 0.0))},
                   1);
        _CheckSets("computed_path_reads Projector.rigExec:shaderOffset",
                   stage, 1.0,
                   {_Matrix(projector + "rigExec:shaderOffset", offset)}, 1);
    }
    // A delta mush disabled at T reads nothing past its enable there, so
    // the run at T reaches neither its rest points nor its topology; the
    // binary holds them anyway, and enabling the mover deforms as the
    // session edit does.
    if (const UsdStageRefPtr stage = _Open(
            examples + "/../docs/examples/delta_mush_mover.usda")) {
        const std::string enabled = "/Rig/Movers/Deform/Detail.inputs:enabled";
        bool authored = false;
        {
            UsdEditContext context(stage, stage->GetSessionLayer());
            const UsdPrim mover =
                stage->GetPrimAtPath(SdfPath("/Rig/Movers/Deform/Detail"));
            authored = mover &&
                       mover.CreateAttribute(TfToken("inputs:enabled"),
                                             SdfValueTypeNames->Bool, false)
                           .Set(false);
        }
        CHECK(authored);
        ++cases;
        _CheckSets("delta_mush_mover Detail.inputs:enabled = true", stage,
                   24.0, {_Bool(enabled, true)}, 2);
    }

    std::printf("inputs: %d of %d case(s) == session edit and moved, %d "
                "run(s) compared\n",
                matchedCases, cases, matchedRuns);
    CHECK(matchedCases == cases);
    if (failures == 0) {
        std::printf("testRigExecRuntimeInputs: all tests passed\n");
        return 0;
    }
    std::printf("testRigExecRuntimeInputs: %d failures\n", failures);
    return 1;
}
