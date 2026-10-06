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
// whose rest points and topology no run at T read. Then the array inputs
// the export lists: the input API on a lattice cage, authored sets (which
// every read takes), sampled sets (which only the reads at the time take)
// and a sampled set of another count; a Default-time read alone; a chain
// base, reset and re-run at another count; a fixed skin layout rebuilt
// from its arrays and from its element size, and returned to the Open
// layout by a reset; painted weights, a repeated sparse index among them;
// and a mesh's topology, a ribbon's bind coordinates, a blend sample's
// points and a volume's gathered curve. Then array sets live baked takes as
// interactive overrides, against which the work counters agree too, and a
// lattice whose live cage read is bound where its read phase answers
// nothing.
#include "rigExec/rigEvaluator.h"
#include "rigExecBake/bake.h"
#include "rigExecRuntime/runtime.h"

#include "pxr/base/gf/rotation.h"
#include "pxr/base/plug/registry.h"
#include "pxr/usd/sdf/types.h"
#include "pxr/usd/usd/editContext.h"
#include "pxr/usd/usd/primRange.h"
#include "pxr/usd/usd/stage.h"

#include <algorithm>
#include <cmath>
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

#include "rigExecFileEdit.h"
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

// ----------------------------------------------------------- array inputs
//
// Each case first checks what the export listed: a slot per attribute, its
// default, and the reads of that attribute bound to it.

static int arrayCases = 0;
static int arrayMatched = 0;

// \p stage's bake at \p time into \p bytes.
static bool
_BakeArrays(const std::string &label, const UsdStageRefPtr &stage,
            const SdfPath &rigPath, double time, std::vector<uint8_t> *bytes)
{
    RigExecRigEvaluator evaluator(stage, rigPath);
    evaluator.SetEvaluationMode(RigExecEvaluationMode::Baked);
    std::string error;
    if (!RigExecTestBakeAt(evaluator, time, bytes, &error)) {
        std::printf("%s: bake: %s\n", label.c_str(), error.c_str());
        CHECK(false);
        return false;
    }
    return true;
}

static fb::RigExecWirePathRead *
_Row(fb::RigExecWireFile *file, const std::string &text, bool rest)
{
    return RigExecTestPathReadRow(file, text, rest);
}

// Whether \p row reads array slot \p slot alone, as the export binds a
// raw read: at rest beside a static value, live without one.
static bool
_BoundRaw(const fb::RigExecWirePathRead *row, int64_t slot)
{
    return row && row->read && slot >= 0 &&
           row->read->mode == fb::ReadMode::Raw &&
           row->read->walk == std::vector<uint32_t>{uint32_t(slot)} &&
           bool(row->value) == row->rest;
}

// The default of input slot \p slot of \p file.
static const fb::RigExecWireValue &
_DefaultOf(const fb::RigExecWireFile &file, int64_t slot)
{
    return file.values[file.inputs[size_t(slot)].value()];
}

// Open's refusal of \p bytes changed by \p edit; empty when it opens.
static std::string
_OpenRefusal(const std::vector<uint8_t> &bytes,
             const std::function<void(fb::RigExecWireFile *)> &edit)
{
    const std::vector<uint8_t> edited = RigExecTestEdited(bytes, edit);
    std::string error;
    const std::unique_ptr<RigExecRuntimeReader> reader =
        RigExecRuntimeReader::Open(edited.data(), edited.size(), &error);
    return reader ? std::string() : error;
}

static uint8_t
_SlotFlags(bool animated)
{
    return uint8_t(uint8_t(fb::InputSlotFlags::HasValue) |
                   (animated ? uint8_t(fb::InputSlotFlags::Animated) : 0));
}

// A fresh reader of \p bytes after its first run, at its defaults.
static std::unique_ptr<RigExecRuntimeReader>
_OpenRun(const std::string &label, const std::vector<uint8_t> &bytes)
{
    std::string error;
    std::unique_ptr<RigExecRuntimeReader> reader =
        RigExecRuntimeReader::Open(bytes.data(), bytes.size(), &error);
    if (!reader || !reader->Execute(&error)) {
        std::printf("%s: open or first run: %s\n", label.c_str(),
                    error.c_str());
        CHECK(false);
        return nullptr;
    }
    return reader;
}

// Whether two readers' last runs published the same outputs: moved points,
// joint matrices, weight fields and property values, bit for bit.
static bool
_SameOutputs(const RigExecRuntimeReader &a, const RigExecRuntimeReader &b)
{
    const auto samePoints = [](const std::vector<RigExecRuntimePoints> &x,
                               const std::vector<RigExecRuntimePoints> &y) {
        if (x.size() != y.size()) {
            return false;
        }
        for (size_t i = 0; i < x.size(); ++i) {
            if (x[i].path != y[i].path ||
                x[i].points.size() != y[i].points.size() ||
                (!x[i].points.empty() &&
                 std::memcmp(x[i].points.data(), y[i].points.data(),
                             sizeof(RrVec3f) * x[i].points.size()) != 0)) {
                return false;
            }
        }
        return true;
    };
    const auto sameJoints = [](const RigExecRuntimeReader &x,
                               const RigExecRuntimeReader &y) {
        const auto &p = x.GetJointMatrices();
        const auto &q = y.GetJointMatrices();
        if (p.size() != q.size()) {
            return false;
        }
        for (size_t i = 0; i < p.size(); ++i) {
            if (p[i].path != q[i].path || !(p[i].matrix == q[i].matrix)) {
                return false;
            }
        }
        return true;
    };
    const auto sameFields = [](const RigExecRuntimeReader &x,
                               const RigExecRuntimeReader &y) {
        const auto &p = x.GetWeightFields();
        const auto &q = y.GetWeightFields();
        if (p.size() != q.size()) {
            return false;
        }
        for (size_t i = 0; i < p.size(); ++i) {
            if (p[i].path != q[i].path || p[i].target != q[i].target ||
                p[i].weights.size() != q[i].weights.size() ||
                (!p[i].weights.empty() &&
                 std::memcmp(p[i].weights.data(), q[i].weights.data(),
                             sizeof(float) * p[i].weights.size()) != 0)) {
                return false;
            }
        }
        return true;
    };
    const auto sameProperties = [](const RigExecRuntimeReader &x,
                                   const RigExecRuntimeReader &y) {
        const auto p = x.GetPropertyValues();
        const auto q = y.GetPropertyValues();
        if (p.size() != q.size()) {
            return false;
        }
        for (size_t i = 0; i < p.size(); ++i) {
            if (p[i].path != q[i].path || p[i].value != q[i].value) {
                return false;
            }
        }
        return true;
    };
    return samePoints(a.GetPoints(), b.GetPoints()) && sameJoints(a, b) &&
           sameFields(a, b) && sameProperties(a, b);
}

// Whether two readers' last runs published the same outputs, diagnostics
// and work counters: two runs of the same kind (each a first run, or each
// a later one).
static bool
_SameRuns(const RigExecRuntimeReader &a, const RigExecRuntimeReader &b)
{
    const RigExecRuntimeCounters ca = a.GetCounters();
    const RigExecRuntimeCounters cb = b.GetCounters();
    return _SameOutputs(a, b) &&
           RigExecTestWithoutSummary(a.GetDiagnostics()) ==
               RigExecTestWithoutSummary(b.GetDiagnostics()) &&
           ca.revisionsExecuted == cb.revisionsExecuted &&
           ca.revisionsCreated == cb.revisionsCreated &&
           ca.schedulesBuilt == cb.schedulesBuilt;
}

// What a case compares its run with.
enum class _Reference {
    // The evaluators with the sets authored: outputs, property values and
    // diagnostics.
    Edited,
    // The same, outputs and property values only: the evaluators take the
    // edit for a structural one their compile refuses, where the program
    // fails it at run time; \p check checks the binary's lines.
    EditedOutputs,
    // The evaluators with nothing authored: a set no read of the program
    // takes, which the evaluators cannot be given (a time sample on a
    // static weight's values, which their compile refuses).
    Unedited,
};

// One case: a fresh reader of \p bytes runs at its defaults, takes
// \p sets, and runs again; that run must equal a fresh dynamic and a fresh
// baked evaluator at the bake time with the same values authored in the
// session layer (RigExecTestArraySet), as \p reference says, and must
// differ from the defaults exactly when \p moves. \p check sees the reader
// after the second run.
static void
_CheckArraySets(const std::string &label, const UsdStageRefPtr &stage,
                const SdfPath &rigPath, const std::vector<uint8_t> &bytes,
                const std::vector<RigExecTestArraySet> &sets, bool moves,
                const std::function<void(RigExecRuntimeReader &)> &check =
                    {},
                _Reference reference = _Reference::Edited)
{
    ++arrayCases;
    std::unique_ptr<RigExecRuntimeReader> reader = _OpenRun(label, bytes);
    std::unique_ptr<RigExecRuntimeReader> defaults = _OpenRun(label, bytes);
    if (!reader || !defaults) {
        return;
    }
    const double t = reader->GetBakeTime();
    std::string error;
    if (!RigExecTestApplyArraySets(reader.get(), sets, &error) ||
        !reader->Execute(&error)) {
        std::printf("%s: sets: %s\n", label.c_str(), error.c_str());
        CHECK(false);
        return;
    }
    bool same = true;
    for (const auto &[mode, text] :
         {std::make_pair(RigExecEvaluationMode::Dynamic, "dynamic"),
          std::make_pair(RigExecEvaluationMode::Baked, "baked")}) {
        RigExecRigPose pose;
        if (!RigExecTestArrayReference(
                stage, rigPath, mode,
                reference == _Reference::Unedited
                    ? std::vector<RigExecTestArraySet>()
                    : sets,
                t, &pose, &error)) {
            std::printf("%s, %s: %s\n", label.c_str(), text, error.c_str());
            CHECK(false);
            return;
        }
        std::vector<std::string> diffs;
        const bool equal = RigExecCompareRuntimeRun(
            pose, *reader, &diffs,
            reference == _Reference::EditedOutputs);
        CHECK(equal);
        if (!equal) {
            same = false;
            std::printf("%s, %s: MISMATCH\n", label.c_str(), text);
            for (const std::string &line : diffs) {
                std::printf("    %s\n", line.c_str());
            }
        }
    }
    const bool moved = !_SameOutputs(*reader, *defaults);
    CHECK(moved == moves);
    if (check) {
        check(*reader);
    }
    if (same && moved == moves) {
        ++arrayMatched;
    }
    std::printf("%s at t=%g: == %s in the dynamic and baked evaluators, "
                "%s\n",
                label.c_str(), t,
                reference == _Reference::Edited ? "session edit"
                : reference == _Reference::EditedOutputs
                    ? "session edit's outputs"
                    : "no edit",
                moved ? "moved" : "unmoved");
}

// One step of a drive against live baked: the authored array values that
// stand after it, which replace the ones before (none: every input set
// before is reset).
struct _OverrideStep {
    std::string what;
    std::vector<RigExecTestArraySet> sets;
};

static int overrideRuns = 0;
static int overrideMatched = 0;

// Plays \p steps at the bake time on a fresh reader of \p bytes and on a
// live baked evaluator compiled on \p stage, which takes each step's values
// as interactive overrides and places them in its program, with no
// rebuild. Every run, the first at the defaults included, must agree in
// outputs, property values and every diagnostic line, the work counters'
// summary among them; each live run must come from the program. \p check
// sees the reader after step \p k.
static void
_CheckAgainstOverrides(
    const std::string &label, const UsdStageRefPtr &stage,
    const SdfPath &rigPath, const std::vector<uint8_t> &bytes,
    const std::vector<_OverrideStep> &steps,
    const std::function<void(size_t, RigExecRuntimeReader &)> &check = {})
{
    std::string error;
    const std::unique_ptr<RigExecRuntimeReader> reader =
        RigExecRuntimeReader::Open(bytes.data(), bytes.size(), &error);
    RigExecRigEvaluator evaluator(stage, rigPath);
    evaluator.SetEvaluationMode(RigExecEvaluationMode::Baked);
    std::vector<std::string> notices;
    if (!reader || !evaluator.Compile(&notices)) {
        std::printf("%s: open or compile: %s\n", label.c_str(),
                    error.c_str());
        CHECK(false);
        return;
    }
    const double t = reader->GetBakeTime();
    const auto compare = [&](const std::string &what) {
        ++overrideRuns;
        const size_t generations = evaluator.GetBakedGenerationCount();
        const RigExecRigPose pose = evaluator.Evaluate(UsdTimeCode(t));
        std::vector<std::string> diffs;
        bool same = pose.valid && pose.bakedParityMismatches == 0 &&
                    evaluator.GetBakedGenerationCount() == generations + 1;
        if (!same) {
            diffs.push_back("the live run is invalid or not the program's");
        }
        same = RigExecCompareRuntimeOutputs(pose, *reader, &diffs) && same;
        same = RigExecCompareRuntimeProperties(pose, *reader, &diffs) && same;
        const std::vector<std::string> &got = reader->GetDiagnostics();
        if (pose.diagnostics != got) {
            same = false;
            for (size_t i = 0;
                 i < std::max(pose.diagnostics.size(), got.size()); ++i) {
                const std::string a =
                    i < pose.diagnostics.size() ? pose.diagnostics[i] : "";
                const std::string b = i < got.size() ? got[i] : "";
                if (a != b) {
                    diffs.push_back("baked [" + a.substr(0, 150) +
                                    "] binary [" + b.substr(0, 150) + "]");
                }
            }
        }
        CHECK(same);
        if (same) {
            ++overrideMatched;
        } else {
            std::printf("%s, %s: MISMATCH against live baked\n",
                        label.c_str(), what.c_str());
            for (const std::string &line : diffs) {
                std::printf("    %s\n", line.c_str());
            }
        }
    };
    if (!reader->Execute(&error)) {
        std::printf("%s: first run: %s\n", label.c_str(), error.c_str());
        CHECK(false);
        return;
    }
    compare("defaults");
    std::vector<std::string> standing;
    for (size_t k = 0; k < steps.size(); ++k) {
        const _OverrideStep &step = steps[k];
        bool ok = true;
        for (const std::string &name : standing) {
            ok = reader->ResetInput(name, &error) && ok;
        }
        standing.clear();
        std::vector<RigExecValueOverride> overrides;
        for (const RigExecTestArraySet &set : step.sets) {
            const SdfPath path(set.name);
            overrides.push_back(RigExecValueOverride{
                path.GetPrimPath(), TfToken(), path.GetNameToken(),
                set.value});
            standing.push_back(set.name);
        }
        ok = ok && RigExecTestApplyArraySets(reader.get(), step.sets, &error);
        if (overrides.empty()) {
            evaluator.ClearInteractiveOverrides();
        } else {
            evaluator.SetInteractiveOverrides(overrides);
        }
        if (!ok || !reader->Execute(&error)) {
            std::printf("%s, %s: %s\n", label.c_str(), step.what.c_str(),
                        error.c_str());
            CHECK(false);
            return;
        }
        compare(step.what);
        if (check) {
            check(k, *reader);
        }
    }
    std::printf("%s at t=%g: %zu run(s) against live baked's interactive "
                "overrides\n",
                label.c_str(), t, steps.size() + 1);
}

// Array sets on a mover's own attributes, which live reads through the
// interactive overlay, so that live baked takes the same values without a
// rebuild: here the work counters agree too. MeshASkin's weights (tests/
// fixtures/oneloop_two_limbs.usda) set, then set to the defaults with one
// +0 weight written -0, then reset: the last two compare equal by value to
// the layout standing, which stays, as live's layout op keeps it; and the
// delta mush's rest points (tests/fixtures/computed_path_reads.usda).
static void
_TestArraysAgainstOverrides(const std::string &fixtures)
{
    if (const UsdStageRefPtr stage =
            _Open(fixtures + "/oneloop_two_limbs.usda")) {
        const std::string label = "oneloop_two_limbs MeshASkin weights";
        const std::string mover = "/LimbsAsset/Rig/Movers/MeshASkin";
        const std::string weightsName = mover + ".rigExec:jointWeights";
        const SdfPath rigPath = _FindRig(stage);
        std::vector<uint8_t> bytes;
        VtFloatArray weights;
        CHECK(stage->GetAttributeAtPath(SdfPath(weightsName)).Get(&weights));
        size_t zero = weights.size();
        for (size_t i = 0; i < weights.size() && zero == weights.size(); ++i) {
            if (weights[i] == 0.0f && !std::signbit(weights[i])) {
                zero = i;
            }
        }
        CHECK(zero < weights.size());
        if (zero < weights.size() &&
            _BakeArrays(label, stage, rigPath, 6.0, &bytes)) {
            VtFloatArray swapped = weights;
            for (size_t i = 0; i + 1 < swapped.size(); i += 2) {
                std::swap(swapped[i], swapped[i + 1]);
            }
            VtFloatArray signed0 = weights;
            signed0[zero] = -0.0f;
            const VtValue a(swapped), b(signed0);
            _CheckAgainstOverrides(
                label, stage, rigPath, bytes,
                {{"swapped", {{weightsName, a, false}}},
                 {"-0 for +0", {{weightsName, b, false}}},
                 {"reset", {}},
                 {"-0 for +0 again", {{weightsName, b, false}}},
                 {"reset again", {}}},
                [&](size_t, RigExecRuntimeReader &played) {
                    // Past the first set the Open layout never returns:
                    // the -0 layout stands through both resets.
                    CHECK(!played.GetSkinLayoutIsOpenForTesting(mover));
                });
        }
    }
    if (const UsdStageRefPtr stage =
            _Open(fixtures + "/computed_path_reads.usda")) {
        const std::string label = "computed_path_reads Mush.inputs:restPoints";
        const std::string rest =
            "/PathReadAsset/Rig/Movers/Mush.inputs:restPoints";
        const SdfPath rigPath = _FindRig(stage);
        std::vector<uint8_t> bytes;
        VtVec3fArray points;
        CHECK(stage->GetAttributeAtPath(SdfPath(rest)).Get(&points));
        for (GfVec3f &p : points) {
            p = p * 0.9f + GfVec3f(0.05f, -0.1f, 0.0f);
        }
        if (_BakeArrays(label, stage, rigPath, 10.0, &bytes)) {
            _CheckAgainstOverrides(label, stage, rigPath, bytes,
                                   {{"authored", {{rest, VtValue(points)}}},
                                    {"reset", {}}});
        }
    }
}

// The lattice of examples/13_ReadPhases.usda reads its live cage through
// a `final` read phase on the cage's chain. Where the phase answers, live
// takes the chain's points there and never the cage, and the export binds
// that read to nothing. With the cage blocked at the bake time the chain
// reads no base, the phase answers nothing, and live reads the cage
// itself: the export binds that read to the cage's input beside the rest
// read, the binary plays the bake time as live does, and a sampled set
// reaches the lattice through it.
static void
_TestUnansweredPhaseArray(const std::string &examples)
{
    const std::string label = "13 Cage.points";
    const std::string cage = "/ReadPhaseAsset/Geom/Cage.points";
    const UsdStageRefPtr stage = _Open(examples + "/13_ReadPhases.usda");
    if (!stage) {
        return;
    }
    const SdfPath rigPath = _FindRig(stage);
    const auto rows = [&](const std::vector<uint8_t> &bytes, bool *live,
                          bool *rest) {
        *live = *rest = false;
        const std::unique_ptr<fb::RigExecWireFile> file =
            RigExecTestUnpack(bytes);
        if (!file) {
            return false;
        }
        const int64_t slot = RigExecTestSlotOf(*file, cage);
        const fb::RigExecWirePathRead *liveRow = _Row(file.get(), cage, false);
        *live = liveRow != nullptr && _BoundRaw(liveRow, slot);
        *rest = _BoundRaw(_Row(file.get(), cage, true), slot);
        return slot >= 0 && (!liveRow || *live);
    };
    std::vector<uint8_t> answered;
    bool live = false, rest = false;
    CHECK(_BakeArrays(label, stage, rigPath, 1002.0, &answered) &&
          rows(answered, &live, &rest) && !live && rest);

    VtVec3fArray authored;
    const UsdAttribute attribute = stage->GetAttributeAtPath(SdfPath(cage));
    CHECK(attribute.Get(&authored));
    {
        UsdEditContext context(stage, stage->GetSessionLayer());
        CHECK(attribute.Set(SdfValueBlock(), UsdTimeCode(1002.0)));
    }
    std::vector<uint8_t> bytes;
    const bool listed = _BakeArrays(label, stage, rigPath, 1002.0, &bytes) &&
                        rows(bytes, &live, &rest) && live && rest;
    CHECK(listed);
    std::unique_ptr<RigExecRuntimeReader> reader =
        listed ? _OpenRun(label, bytes) : nullptr;
    bool same = reader != nullptr;
    for (const auto &[mode, text] :
         {std::make_pair(RigExecEvaluationMode::Dynamic, "dynamic"),
          std::make_pair(RigExecEvaluationMode::Baked, "baked")}) {
        RigExecRigPose pose;
        std::string error;
        std::vector<std::string> diffs;
        if (!reader ||
            !RigExecTestArrayReference(stage, rigPath, mode, {}, 1002.0, &pose,
                                       &error) ||
            !RigExecCompareRuntimeRun(pose, *reader, &diffs)) {
            same = false;
            std::printf("%s blocked, %s: MISMATCH %s\n", label.c_str(), text,
                        error.c_str());
            for (const std::string &line : diffs) {
                std::printf("    %s\n", line.c_str());
            }
        }
    }
    CHECK(same);
    bool moved = false;
    if (reader) {
        std::unique_ptr<RigExecRuntimeReader> played = _OpenRun(label, bytes);
        VtVec3fArray scaled = authored;
        for (GfVec3f &p : scaled) {
            p *= 1.1f;
        }
        std::string error;
        CHECK(played &&
              RigExecTestApplyArraySets(played.get(),
                                        {{cage, VtValue(scaled), true}},
                                        &error) &&
              played->Execute(&error));
        moved = played && !_SameOutputs(*reader, *played);
    }
    CHECK(moved);
    stage->GetSessionLayer()->Clear();
    std::printf("%s: live read bound only where its read phase answers "
                "nothing; blocked at t=1002 %s live, a sampled set %s\n",
                label.c_str(), same ? "==" : "differs from",
                moved ? "moves the lattice" : "does not move it");
}

// The lattice cage of examples/06_LatticeBulge.usda, an Animated float3[]
// that the rest read (Default) and the live read (the time) share: the
// input API on it, then an authored set, which both reads take, a sampled
// one, which only the live read takes, a sampled one of another count,
// which fails the lattice as live fails it, and a reset.
static void
_TestLatticeArray(const UsdStageRefPtr &stage)
{
    const std::string label = "06 Cage.points";
    const std::string cage = "/LatticeAsset/Geom/Cage.points";
    const SdfPath rigPath = _FindRig(stage);
    std::vector<uint8_t> bytes;
    if (!_BakeArrays(label, stage, rigPath, 1024.0, &bytes)) {
        return;
    }
    // One Animated input, which both cage reads take.
    bool exported = false;
    if (const std::unique_ptr<fb::RigExecWireFile> file =
            RigExecTestUnpack(bytes)) {
        const int64_t slot = RigExecTestSlotOf(*file, cage);
        exported = slot >= 0 &&
                   file->inputs[size_t(slot)].flags() ==
                       uint8_t(_SlotFlags(true) |
                               uint8_t(fb::InputSlotFlags::Listed)) &&
                   _BoundRaw(_Row(file.get(), cage, true), slot) &&
                   _BoundRaw(_Row(file.get(), cage, false), slot);
    }
    CHECK(exported);
    if (!exported) {
        return;
    }
    const std::vector<uint8_t> &listed = bytes;
    // A second reader at its defaults, which nothing is set on.
    std::unique_ptr<RigExecRuntimeReader> plain = _OpenRun(label, bytes);
    std::unique_ptr<RigExecRuntimeReader> reader = _OpenRun(label, listed);
    if (!plain || !reader) {
        return;
    }

    // The input API.
    size_t index = 0;
    CHECK(reader->FindInput(cage, &index));
    const RigExecRuntimeInputInfo &info = reader->GetInputInfo(index);
    CHECK(info.type == RrInputTag::Vec3fArray && info.animated &&
          info.defaultCount == 12);
    CHECK(reader->GetInputValue(index).tag == RrInputTag::Vec3fArray);
    RigExecRuntimeArray held;
    CHECK(reader->GetInputArrayAt(index, &held) &&
          held.tag == RrInputTag::Vec3fArray && held.count == 12 &&
          held.data != nullptr);
    const void *const defaultData = held.data;
    VtVec3fArray stageCage;
    CHECK(stage->GetAttributeAtPath(SdfPath(cage))
              .Get(&stageCage, UsdTimeCode(1024.0)));
    CHECK(stageCage.size() == 12 &&
          std::memcmp(stageCage.cdata(), held.data,
                      sizeof(GfVec3f) * 12) == 0);
    std::string error;
    CHECK(!reader->SetInput(cage, 1.0, &error) &&
          error == cage + " is a float3[] input: set it with SetInputArray");
    CHECK(!reader->ClearInputAt(index, &error) &&
          error == cage + " is an array input; ResetInput restores it");
    CHECK(!reader->SetInputToken(cage, "x", &error) &&
          error == cage + " is a float3[] input, not a token");
    const VtFloatArray floats(12, 1.0f);
    CHECK(!reader->SetInputArray(cage, RigExecTestArrayView(VtValue(floats)),
                                 &error) &&
          error == cage + " is a float3[] input, not a float[]");
    VtVec3fArray shorter = stageCage;
    shorter.pop_back();
    CHECK(!reader->SetInputArray(cage, RigExecTestArrayView(VtValue(shorter)),
                                 &error) &&
          error == cage + " holds 12 elements; an array set keeps that "
                          "count, not 11");
    RigExecRuntimeArray none;
    none.tag = RrInputTag::Vec3fArray;
    none.count = 3;
    CHECK(!reader->SetInputArrayAt(index, none, &error) &&
          error == cage + ": no elements to copy");
    CHECK(!reader->SetInputArrayAt(9999, none, &error));
    size_t scalar = 0;
    while (scalar < reader->GetInputCount() &&
           RrInputTagIsArray(reader->GetInputInfo(scalar).type)) {
        ++scalar;
    }
    if (scalar < reader->GetInputCount()) {
        CHECK(!reader->SetInputArrayAt(scalar,
                                       RigExecTestArrayView(VtValue(shorter)),
                                       &error) &&
              error.find("input, not an array") != std::string::npos);
    }
    // A refused set changes nothing: the same view, and the run a reader
    // with nothing set runs.
    CHECK(reader->GetInputArrayAt(index, &held) &&
          held.data == defaultData && held.count == 12);
    const auto idle = [&] {
        std::string why;
        CHECK(plain->Execute(&why) && reader->Execute(&why));
        CHECK(reader->GetLastRunTraceForTesting() ==
                  plain->GetLastRunTraceForTesting() &&
              _SameRuns(*plain, *reader));
    };
    idle();
    // A sampled set of the default's elements holds nothing and flags
    // nothing. (An authored one holds nothing either, but the rest read
    // takes it, which the comparison below shows.)
    CHECK(reader->SetSampledInputArrayAt(
        index, RigExecTestArrayView(VtValue(stageCage)), &error));
    CHECK(reader->GetInputArrayAt(index, &held) &&
          held.data == defaultData);
    idle();
    // The reader copies: the caller's buffer may change after a set.
    VtVec3fArray scaled = stageCage;
    for (GfVec3f &p : scaled) {
        p *= 1.1f;
    }
    {
        VtVec3fArray buffer = scaled;
        CHECK(reader->SetInputArrayAt(index,
                                      RigExecTestArrayView(VtValue(buffer)),
                                      &error));
        for (GfVec3f &p : buffer) {
            p = GfVec3f(0.0f);
        }
    }
    CHECK(reader->GetInputArrayAt(index, &held) && held.count == 12 &&
          held.data != defaultData &&
          std::memcmp(held.data, scaled.cdata(), sizeof(GfVec3f) * 12) == 0);
    const void *const setData = held.data;
    CHECK(reader->Execute(&error));
    CHECK(reader->GetInputArrayAt(index, &held) && held.data == setData);
    reader->ResetInputs();
    CHECK(reader->GetInputArrayAt(index, &held) &&
          held.data == defaultData);
    CHECK(reader->Execute(&error) && _SameOutputs(*plain, *reader));

    // Against the evaluators. The posed cage authored: the rest read takes
    // it too, so the lattice holds the slab at rest.
    _CheckArraySets(label + " authored as posed", stage, rigPath, listed,
                    {{cage, VtValue(stageCage), false}}, true,
                    [&](RigExecRuntimeReader &played) {
                        RigExecRuntimeArray view;
                        CHECK(played.GetInputArrayAt(index, &view) &&
                              view.count == 12);
                    });
    _CheckArraySets(label + " authored x1.1", stage, rigPath, listed,
                    {{cage, VtValue(scaled), false}}, true);
    _CheckArraySets(label + " sampled x1.1", stage, rigPath, listed,
                    {{cage, VtValue(scaled), true}}, true);
    _CheckArraySets(label + " sampled, 11 points", stage, rigPath, listed,
                    {{cage, VtValue(shorter), true}}, true);
    // The authored and the sampled set differ: the rest read takes only
    // the authored one.
    {
        std::unique_ptr<RigExecRuntimeReader> a = _OpenRun(label, listed);
        std::unique_ptr<RigExecRuntimeReader> s = _OpenRun(label, listed);
        if (a && s) {
            CHECK(a->SetInputArrayAt(index,
                                     RigExecTestArrayView(VtValue(scaled)),
                                     &error) &&
                  a->Execute(&error));
            CHECK(s->SetSampledInputArrayAt(
                      index, RigExecTestArrayView(VtValue(scaled)),
                      &error) &&
                  s->Execute(&error));
            CHECK(!_SameOutputs(*a, *s));
            CHECK(a->ResetInput(cage, &error) && a->Execute(&error) &&
                  _SameOutputs(*plain, *a));
        }
    }

    // The sampler leaves an array input alone: no warning, no sample, and
    // the default view after a sample at another time.
    {
        RigExecTestPlayer player;
        CHECK(player.Open(listed, stage, &error));
        size_t animatedScalars = 0;
        for (size_t i = 0; i < player->GetInputCount(); ++i) {
            const RigExecRuntimeInputInfo &input = player->GetInputInfo(i);
            animatedScalars +=
                input.animated && !RrInputTagIsArray(input.type) ? 1 : 0;
        }
        CHECK(player.Sampler().GetWarnings().empty() &&
              player.Sampler().GetAnimatedCount() == animatedScalars);
        CHECK(player.Play(1036.0, &error));
        RigExecRuntimeArray sampled;
        CHECK(player->GetInputArrayAt(index, &sampled) &&
              sampled.count == 12);
        RigExecRuntimeArray atOpen;
        std::unique_ptr<RigExecRuntimeReader> fresh = _OpenRun(label, listed);
        CHECK(fresh && fresh->GetInputArrayAt(index, &atOpen) &&
              std::memcmp(sampled.data, atOpen.data,
                          sizeof(GfVec3f) * 12) == 0);
        std::printf("%s: the sampler samples %zu input(s) of %zu, the "
                    "array left at its default\n",
                    label.c_str(), player.Sampler().GetAnimatedCount(),
                    player->GetInputCount());
    }
}

// A Default-time read alone: the delta mush's inputs:restPoints in
// tests/fixtures/computed_path_reads.usda at frame 10. An authored set
// reaches it; a sampled one does not, as a time sample does not.
static void
_TestRestPointsArray(const UsdStageRefPtr &stage)
{
    const std::string label = "computed_path_reads Mush.inputs:restPoints";
    const std::string rest =
        "/PathReadAsset/Rig/Movers/Mush.inputs:restPoints";
    const SdfPath rigPath = _FindRig(stage);
    std::vector<uint8_t> bytes;
    if (!_BakeArrays(label, stage, rigPath, 10.0, &bytes)) {
        return;
    }
    // Read at Default alone: the input's default is the rest row's value.
    bool exported = false;
    if (const std::unique_ptr<fb::RigExecWireFile> file =
            RigExecTestUnpack(bytes)) {
        const int64_t slot = RigExecTestSlotOf(*file, rest);
        const fb::RigExecWirePathRead *row = _Row(file.get(), rest, true);
        exported = _BoundRaw(row, slot) && !_Row(file.get(), rest, false) &&
                   row->value->tag == fb::PathTag::Vec3fArray &&
                   _DefaultOf(*file, slot).array == row->value->array;
    }
    CHECK(exported);
    if (!exported) {
        return;
    }
    const std::vector<uint8_t> &listed = bytes;
    VtVec3fArray points;
    CHECK(stage->GetAttributeAtPath(SdfPath(rest)).Get(&points));
    for (GfVec3f &p : points) {
        p = p * 0.9f + GfVec3f(0.05f, -0.1f, 0.0f);
    }
    _CheckArraySets(label + " authored", stage, rigPath, listed,
                    {{rest, VtValue(points), false}}, true);
    _CheckArraySets(label + " sampled", stage, rigPath, listed,
                    {{rest, VtValue(points), true}}, false);
}

// The chain base of tests/fixtures/oneloop_two_limbs.usda, MeshA.points:
// an authored set of its count moves the base, a reset returns the file's
// base, and a sampled set of two fewer points resets the chain and re-runs
// it at that count, as live's count branch does.
static void
_TestChainBaseArray(const UsdStageRefPtr &stage)
{
    const std::string label = "oneloop_two_limbs MeshA.points";
    const std::string target = "/LimbsAsset/Geom/MeshA.points";
    const SdfPath rigPath = _FindRig(stage);
    std::vector<uint8_t> bytes;
    if (!_BakeArrays(label, stage, rigPath, 6.0, &bytes)) {
        return;
    }
    // The target's points are the chain's base input, its default the
    // base the file stores.
    bool exported = false;
    if (const std::unique_ptr<fb::RigExecWireFile> file =
            RigExecTestUnpack(bytes)) {
        const int64_t slot = RigExecTestSlotOf(*file, target);
        for (const fb::RigExecWireChain &c : file->geometry->chains) {
            if (RigExecFormatPathText(*file, c.target) == target) {
                exported = slot >= 0 && c.haveBase && c.baseSlot == slot &&
                           _DefaultOf(*file, slot).array == c.base &&
                           !_Row(file.get(), target, false) &&
                           !_Row(file.get(), target, true);
            }
        }
    }
    CHECK(exported);
    if (!exported) {
        return;
    }
    const std::vector<uint8_t> &listed = bytes;
    std::unique_ptr<RigExecRuntimeReader> plain = _OpenRun(label, bytes);
    // Open refuses a base slot whose default is another pool entry.
    {
        std::string want;
        const std::string got =
            _OpenRefusal(listed, [&](fb::RigExecWireFile *file) {
                auto &chains = file->geometry->chains;
                for (size_t c = 0; c < chains.size(); ++c) {
                    if (chains[c].baseSlot < 0 ||
                        RigExecFormatPathText(*file, chains[c].target) !=
                            target) {
                        continue;
                    }
                    const uint32_t slot = uint32_t(chains[c].baseSlot);
                    file->values[file->inputs[slot].value()].array = 0;
                    want = "invalid .rigexec: geometry.chains[" +
                           std::to_string(c) + "].base_slot: slot " +
                           std::to_string(slot) +
                           "'s default is not the chain's base";
                }
            });
        CHECK(!want.empty() && got == want);
        std::printf("%s: refused, %s\n", label.c_str(), got.c_str());
    }
    VtVec3fArray base;
    CHECK(stage->GetAttributeAtPath(SdfPath(target)).Get(&base));
    VtVec3fArray displaced = base;
    for (size_t i = 0; i < displaced.size(); ++i) {
        displaced[i] += GfVec3f(0.0f, 0.1f * float(i % 3), 0.05f);
    }
    _CheckArraySets(label + " authored", stage, rigPath, listed,
                    {{target, VtValue(displaced), false}}, true,
                    [&](RigExecRuntimeReader &played) {
                        std::string error;
                        CHECK(played.ResetInput(target, &error) &&
                              played.Execute(&error));
                        CHECK(plain && _SameOutputs(*plain, played));
                    });
    VtVec3fArray fewer(base.begin(), base.end() - 2);
    _CheckArraySets(label + " sampled, 8 points", stage, rigPath, listed,
                    {{target, VtValue(fewer), true}}, true);
}

// The fixed skin layout of MeshASkin in tests/fixtures/oneloop_two_limbs.
// usda, listed as its two arrays, whose defaults are the stored layout:
// an index past the influences fails the mover, a valid weights set
// rebuilds the layout, a reset returns the Open layout itself, a repeated
// set rebuilds nothing, and a set element size rebuilds the layout too.
static void
_TestLayoutArrays(const UsdStageRefPtr &stage)
{
    const std::string label = "oneloop_two_limbs MeshASkin layout";
    const std::string mover = "/LimbsAsset/Rig/Movers/MeshASkin";
    const std::string indicesName = mover + ".rigExec:jointIndices";
    const std::string weightsName = mover + ".rigExec:jointWeights";
    const SdfPath rigPath = _FindRig(stage);
    std::vector<uint8_t> bytes;
    if (!_BakeArrays(label, stage, rigPath, 6.0, &bytes)) {
        return;
    }
    // The revision's two layout inputs, defaulting to its stored layout.
    bool exported = false;
    if (const std::unique_ptr<fb::RigExecWireFile> file =
            RigExecTestUnpack(bytes)) {
        const auto &geometry = *file->geometry;
        const int64_t indices = RigExecTestSlotOf(*file, indicesName);
        const int64_t weights = RigExecTestSlotOf(*file, weightsName);
        for (size_t r = 0; r < geometry.revisionIndex.size(); ++r) {
            const auto &at = geometry.revisionIndex[r];
            const fb::RigExecWireRevision &revision =
                geometry.chains[size_t(at.first)]
                    .revisions[size_t(at.second)];
            if (RigExecFormatPathText(*file, revision.moverPath) != mover) {
                continue;
            }
            exported = indices >= 0 && weights >= 0 &&
                       revision.topologyResolved && revision.topology &&
                       revision.jointIndicesSlot == indices &&
                       revision.jointWeightsSlot == weights &&
                       _DefaultOf(*file, indices).arraySource ==
                           fb::ArraySource::SkinIndices &&
                       _DefaultOf(*file, weights).arraySource ==
                           fb::ArraySource::SkinWeights &&
                       _DefaultOf(*file, indices).array == r &&
                       _DefaultOf(*file, weights).array == r;
        }
    }
    CHECK(exported);
    if (!exported) {
        return;
    }
    const std::vector<uint8_t> &listed = bytes;
    std::unique_ptr<RigExecRuntimeReader> plain = _OpenRun(label, bytes);
    std::unique_ptr<RigExecRuntimeReader> reader = _OpenRun(label, listed);
    CHECK(plain && reader);
    if (!reader) {
        return;
    }
    size_t index = 0;
    CHECK(reader->FindInput(indicesName, &index) &&
          reader->GetInputInfo(index).defaultCount == 20 &&
          reader->GetInputInfo(index).type == RrInputTag::IntArray);
    CHECK(reader->GetSkinLayoutIsOpenForTesting(mover));
    // Open refuses a layout default naming another revision's layout.
    {
        std::string want;
        const std::string got =
            _OpenRefusal(listed, [&](fb::RigExecWireFile *file) {
                const auto &geometry = *file->geometry;
                int64_t other = -1;
                std::string row;
                uint32_t slot = 0;
                for (size_t r = 0; r < geometry.revisionIndex.size(); ++r) {
                    const auto &at = geometry.revisionIndex[r];
                    const fb::RigExecWireRevision &revision =
                        geometry.chains[size_t(at.first)]
                            .revisions[size_t(at.second)];
                    if (RigExecFormatPathText(*file, revision.moverPath) ==
                            mover &&
                        revision.jointIndicesSlot >= 0) {
                        row = "geometry.chains[" + std::to_string(at.first) +
                              "].revisions[" + std::to_string(at.second) +
                              "]";
                        slot = uint32_t(revision.jointIndicesSlot);
                    } else if (revision.skinTopologyFixed &&
                               revision.topologyResolved &&
                               revision.topology) {
                        other = int64_t(r);
                    }
                }
                if (other < 0 || row.empty()) {
                    return;
                }
                file->values[file->inputs[slot].value()].array =
                    uint32_t(other);
                want = "invalid .rigexec: " + row +
                       ".joint_indices_slot: its default does not name "
                       "this revision's layout";
            });
        CHECK(!want.empty() && got == want);
        std::printf("%s: refused, %s\n", label.c_str(), got.c_str());
    }
    VtIntArray indices;
    VtFloatArray weights;
    const UsdPrim prim = stage->GetPrimAtPath(SdfPath(mover));
    CHECK(prim.GetAttribute(TfToken("rigExec:jointIndices")).Get(&indices));
    CHECK(prim.GetAttribute(TfToken("rigExec:jointWeights")).Get(&weights));

    VtIntArray outOfRange = indices;
    outOfRange[3] = 2;
    _CheckArraySets(label + ": an index past the influences", stage, rigPath,
                    listed, {{indicesName, VtValue(outOfRange), false}}, true,
                    [&](RigExecRuntimeReader &played) {
                        CHECK(!played.GetSkinLayoutIsOpenForTesting(mover));
                    });
    VtFloatArray swapped = weights;
    for (size_t i = 0; i + 1 < swapped.size(); i += 2) {
        std::swap(swapped[i], swapped[i + 1]);
    }
    _CheckArraySets(
        label + ": weights swapped", stage, rigPath, listed,
        {{weightsName, VtValue(swapped), false}}, true,
        [&](RigExecRuntimeReader &played) {
            CHECK(!played.GetSkinLayoutIsOpenForTesting(mover));
            std::string error;
            // The same set again rebuilds nothing: the run equals a run
            // with nothing set.
            CHECK(played.Execute(&error));
            const std::vector<int32_t> idle =
                played.GetLastRunTraceForTesting();
            const RigExecRuntimeCounters idleCounters = played.GetCounters();
            size_t at = 0;
            CHECK(played.FindInput(weightsName, &at) &&
                  played.SetInputArrayAt(
                      at, RigExecTestArrayView(VtValue(swapped)), &error) &&
                  played.Execute(&error));
            CHECK(played.GetLastRunTraceForTesting() == idle &&
                  played.GetCounters().revisionsExecuted ==
                      idleCounters.revisionsExecuted);
            // The reset returns the Open layout itself, and its outputs.
            CHECK(played.ResetInput(weightsName, &error) &&
                  played.Execute(&error));
            CHECK(played.GetSkinLayoutIsOpenForTesting(mover));
            CHECK(plain && _SameOutputs(*plain, played));
        });
    // The element size is a scalar input: a set of it rebuilds the layout
    // as live's layout op does, and 1 makes the twenty entries twenty rows,
    // which the mesh's ten points cannot use.
    const std::string elementSize = mover + ".rigExec:elementSize";
    size_t sizeIndex = 0;
    if (reader->FindInput(elementSize, &sizeIndex)) {
        _CheckArraySets(label + ": element size 1", stage, rigPath, listed,
                        {{elementSize, VtValue(1), false}}, true,
                        [&](RigExecRuntimeReader &played) {
                            CHECK(!played.GetSkinLayoutIsOpenForTesting(
                                mover));
                        });
    } else {
        std::printf("%s: %s is not an input\n", label.c_str(),
                    elementSize.c_str());
        CHECK(false);
    }
}

// Painted weights of examples/01_FkChainTail.usda: Seg1W's dense values
// and Seg4W's sparse values and indices, listed as inputs whose defaults
// are the arrays the file stored. An authored set reaches the packet; a
// sampled one does not, as a time sample does not; a repeated sparse index
// fails the consumers as live fails them.
static void
_TestPaintedArrays(const UsdStageRefPtr &stage)
{
    const std::string label = "01 painted weights";
    const std::string weights = "/TailAsset/Rig/Weights/";
    const std::string seg1 = weights + "Seg1W.rigExec:values";
    const std::string seg4 = weights + "Seg4W.rigExec:values";
    const std::string seg4Indices = weights + "Seg4W.rigExec:indices";
    const SdfPath rigPath = _FindRig(stage);
    std::vector<uint8_t> bytes;
    if (!_BakeArrays(label, stage, rigPath, 1024.0, &bytes)) {
        return;
    }
    // Each weight object names its painted inputs, their defaults the
    // arrays at Default.
    size_t listedObjects = 0;
    if (const std::unique_ptr<fb::RigExecWireFile> file =
            RigExecTestUnpack(bytes)) {
        for (const fb::RigExecWireWeightObject &object :
             file->geometry->weightObjects) {
            const std::string path = RigExecFormatPathText(*file, object.path);
            const std::string values = path + ".rigExec:values";
            const std::string indices = path + ".rigExec:indices";
            if (values != seg1 && values != seg4) {
                continue;
            }
            VtFloatArray authored;
            VtIntArray authoredIndices;
            stage->GetAttributeAtPath(SdfPath(values)).Get(&authored);
            stage->GetAttributeAtPath(SdfPath(indices))
                .Get(&authoredIndices);
            const int64_t v = RigExecTestSlotOf(*file, values);
            const int64_t i = RigExecTestSlotOf(*file, indices);
            if (v < 0 || i < 0) {
                continue;
            }
            const std::vector<float> &held =
                file->floatArrays[_DefaultOf(*file, v).array].v;
            const std::vector<int32_t> &heldIndices =
                file->intArrays[_DefaultOf(*file, i).array].v;
            if (object.valuesSlot == v && object.indicesSlot == i &&
                held.size() == authored.size() &&
                std::equal(held.begin(), held.end(), authored.cbegin()) &&
                heldIndices.size() == authoredIndices.size() &&
                std::equal(heldIndices.begin(), heldIndices.end(),
                           authoredIndices.cbegin())) {
                ++listedObjects;
            }
        }
    }
    CHECK(listedObjects == 2);
    if (listedObjects != 2) {
        return;
    }
    const std::vector<uint8_t> &listed = bytes;
    VtFloatArray values;
    CHECK(stage->GetAttributeAtPath(SdfPath(seg1)).Get(&values));
    for (float &w : values) {
        w *= 0.5f;
    }
    _CheckArraySets(label + ": Seg1W values authored", stage, rigPath,
                    listed, {{seg1, VtValue(values), false}}, true);
    // The packet reads the values at Default, so a sampled set leaves it
    // at its default.
    _CheckArraySets(label + ": Seg1W values sampled", stage, rigPath, listed,
                    {{seg1, VtValue(values), true}}, false, {},
                    _Reference::Unedited);
    // The packet build refuses a repeated index, so each consumer fails
    // with the invalid envelope, as the program's packet path fails it;
    // the evaluators' compile refuses the authored edit itself, and skips
    // the consumer with the same pass-through.
    VtIntArray repeated;
    CHECK(stage->GetAttributeAtPath(SdfPath(seg4Indices)).Get(&repeated));
    if (repeated.size() >= 2) {
        repeated[1] = repeated[0];
    }
    const std::string seg4Skin =
        "/TailAsset/Rig/Movers/Geometry/Seg1Skin/Seg2Skin/Seg3Skin/Seg4Skin";
    _CheckArraySets(
        label + ": Seg4W indices repeated", stage, rigPath, listed,
        {{seg4Indices, VtValue(repeated), false}}, true,
        [&](RigExecRuntimeReader &played) {
            const std::vector<std::string> lines =
                RigExecTestWithoutSummary(played.GetDiagnostics());
            CHECK(lines ==
                  std::vector<std::string>(
                      {"MoverFailed " + seg4Skin +
                           ": rigExec:weightObject produced an invalid "
                           "common envelope; revision passed through",
                       "MoverFailed " + seg4Skin +
                           ": execution rejected its inputs; revision "
                           "passed through"}));
        },
        _Reference::EditedOutputs);
}

// The array inputs of example rigs no case above reaches, each listed by
// the export with every read of it bound, and an authored set against the
// same value authored in the session layer: a mesh's topology (01, two
// faces of the tail strip wound the other way, which turns their
// normals), a ribbon's bind coordinates (05), a dense blend sample's
// points (04, where the brow raise is fully on) and a curve weight's
// gathered curve (11).
static void
_TestExampleArrays(const std::string &examples)
{
    struct _Case {
        const char *stage;
        double time;
        const char *attribute;
        fb::InputTag tag;
        // The authored value, from the stage's own.
        std::function<VtValue(const VtValue &)> edit;
    };
    const std::vector<_Case> cases = {
        {"01_FkChainTail.usda", 1024.0,
         "/TailAsset/Geom/TailStrip.faceVertexIndices", fb::InputTag::IntArray,
         [](const VtValue &held) {
             VtIntArray indices = held.Get<VtIntArray>();
             for (size_t face = 0; face < 2 && indices.size() >= 8; ++face) {
                 std::reverse(indices.begin() + std::ptrdiff_t(face * 4),
                              indices.begin() + std::ptrdiff_t(face * 4 + 4));
             }
             return VtValue(indices);
         }},
        {"05_TwistRibbonSpine.usda", 1024.0,
         "/SpineAsset/Geom/SpineStrip.primvars:st", fb::InputTag::Vec2fArray,
         [](const VtValue &held) {
             VtVec2fArray st = held.Get<VtVec2fArray>();
             for (GfVec2f &uv : st) {
                 uv[0] = uv[0] * 0.8f + 0.1f;
             }
             return VtValue(st);
         }},
        {"04_BlendShapeFace.usda", 1044.0,
         "/FaceAsset/Targets/BrowRaise.points", fb::InputTag::Vec3fArray,
         [](const VtValue &held) {
             VtVec3fArray points = held.Get<VtVec3fArray>();
             for (GfVec3f &p : points) {
                 p[1] += 0.2f;
             }
             return VtValue(points);
         }},
        {"11_VolumeWeights.usda", 1024.0,
         "/VolumeAsset/Drivers/TipCurve.points", fb::InputTag::Vec3fArray,
         [](const VtValue &held) {
             VtVec3fArray points = held.Get<VtVec3fArray>();
             for (GfVec3f &p : points) {
                 p[0] += 0.3f;
             }
             return VtValue(points);
         }},
    };
    for (const _Case &c : cases) {
        const UsdStageRefPtr stage = _Open(examples + "/" + c.stage);
        if (!stage) {
            continue;
        }
        const std::string label =
            std::string(c.stage).substr(0, 2) + " " + c.attribute;
        const SdfPath rigPath = _FindRig(stage);
        std::vector<uint8_t> bytes;
        if (!_BakeArrays(label, stage, rigPath, c.time, &bytes)) {
            continue;
        }
        // One input of the attribute's type, and every row of the
        // attribute reads it.
        bool exported = false;
        if (const std::unique_ptr<fb::RigExecWireFile> file =
                RigExecTestUnpack(bytes)) {
            const int64_t slot = RigExecTestSlotOf(*file, c.attribute);
            exported = slot >= 0 && file->inputs[size_t(slot)].type() == c.tag;
            for (const fb::RigExecWirePathRead &row :
                 file->geometry->pathReads) {
                if (RigExecFormatPathText(*file, row.path) == c.attribute) {
                    exported = exported && row.read && slot >= 0 &&
                               row.read->walk.front() == uint32_t(slot);
                }
            }
        }
        CHECK(exported);
        VtValue held;
        stage->GetAttributeAtPath(SdfPath(c.attribute))
            .Get(&held, UsdTimeCode(c.time));
        if (!exported || held.IsEmpty()) {
            std::printf("%s: not listed\n", label.c_str());
            continue;
        }
        _CheckArraySets(label + " authored", stage, rigPath, bytes,
                        {{c.attribute, c.edit(held), false}}, true);
    }
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

    // The array inputs the export lists.
    if (const UsdStageRefPtr stage =
            _Open(examples + "/06_LatticeBulge.usda")) {
        _TestLatticeArray(stage);
    }
    if (const UsdStageRefPtr stage =
            _Open(fixtures + "/computed_path_reads.usda")) {
        _TestRestPointsArray(stage);
    }
    if (const UsdStageRefPtr stage =
            _Open(fixtures + "/oneloop_two_limbs.usda")) {
        _TestChainBaseArray(stage);
        _TestLayoutArrays(stage);
    }
    if (const UsdStageRefPtr stage =
            _Open(examples + "/01_FkChainTail.usda")) {
        _TestPaintedArrays(stage);
    }
    _TestExampleArrays(examples);
    std::printf("array inputs: %d of %d case(s) == session edit\n",
                arrayMatched, arrayCases);
    CHECK(arrayMatched == arrayCases && arrayCases == 18);
    _TestArraysAgainstOverrides(fixtures);
    std::printf("array inputs: %d of %d run(s) == live baked's interactive "
                "overrides, counters included\n",
                overrideMatched, overrideRuns);
    CHECK(overrideMatched == overrideRuns && overrideRuns == 9);
    _TestUnansweredPhaseArray(examples);
    if (failures == 0) {
        std::printf("testRigExecRuntimeInputs: all tests passed\n");
        return 0;
    }
    std::printf("testRigExecRuntimeInputs: %d failures\n", failures);
    return 1;
}
