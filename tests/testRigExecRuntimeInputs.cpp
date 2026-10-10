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
// layout by a reset, whose indices, topology, refuse a sampled set of
// another count; painted weights, a repeated sparse index among them;
// and a mesh's topology, a ribbon's bind coordinates, a blend sample's
// points and a volume's gathered curve. Then array sets live baked takes as
// interactive overrides, against which the work counters agree too, and a
// lattice whose live cage read is bound where its read phase answers
// nothing. Last, every input write route re-keys the leaves keyed from
// the slot it writes, and a run after no write re-keys none, nor one after
// a write that moves no keyed field; and a sampler that skips static
// provider inputs plays what one reading every input plays, and follows a
// Default edit or new keys after NoteStageChanged.
#include "rigExec/rigEvaluator.h"
#include "rigExecBake/bake.h"
#include "rigExecBake/staticReport.h"
#include "rigExecRuntime/runtime.h"
#include "rigExecRuntime/stageArrayInputs.h"

#include "pxr/base/gf/rotation.h"
#include "pxr/base/plug/registry.h"
#include "pxr/base/tf/getenv.h"
#include "pxr/base/tf/setenv.h"
#include "pxr/usd/sdf/types.h"
#include "pxr/usd/usd/editContext.h"
#include "pxr/usd/usd/primRange.h"
#include "pxr/usd/usd/stage.h"

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <cstdlib>
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
    CHECK(RigExecTestEditedPoses(stage, rigPath,
                                 {}, {t}, &defaults, &error));
    struct Reference {
        const char *mode;
        RigExecRigPose pose;
    };
    std::vector<Reference> references;
    {
        const char *text = "native";
        std::vector<RigExecRigPose> poses;
        const bool ok =
            RigExecTestEditedPoses(stage, rigPath, edits, {t}, &poses,
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
           a.GetDiagnostics() ==
               b.GetDiagnostics() &&
           ca.executedOpCount == cb.executedOpCount;
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
    bool crafted = false;
    for (const RigExecTestArraySet &set : sets) {
        size_t index = 0;
        if (!reader->FindInput(set.name, &index)) {
            std::string refusal;
            CHECK(!reader->SetInputArray(set.name,
                                        RigExecTestArrayView(set.value),
                                        &refusal));
            crafted = true;
        }
    }
    if (crafted) {
        reader = _OpenRun(label + ": crafted public storage",
                         RigExecTestListPrivateArraySlots(bytes));
        CHECK(reader);
        if (!reader) {
            return;
        }
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
    {
        const char *text = "native";
        RigExecRigPose pose;
        if (!RigExecTestArrayReference(stage, rigPath,
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
// included; each live run must come from the program. \p check
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
        bool same = pose.valid && pose.comparisonMismatches == 0 &&
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
// +0 weight written -0, then reset: the signed-zero arrays keep distinct
// layouts, while reset restores the original Open layout; and the
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
                [&](size_t step, RigExecRuntimeReader &played) {
                    const bool reset = step == 2 || step == 4;
                    CHECK(played.GetSkinLayoutIsOpenForTesting(mover) == reset);
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
// takes the chain's points there and the bound raw fallback is unused. With the cage blocked at the bake time the chain
// reads no base, the phase answers nothing, and live reads the cage
// itself: the export binds that read to the cage's input beside the rest
// read, the binary plays the bake time as live does, and a sampled set
// reaches the lattice through it.

// Current source presence must retire a retained Final result, then recover
// on the same imported graph; a blocked source is not a successful empty array.
static void
_TestRetainedPointFinalAvailability(const std::string &examples)
{
    const auto stage=_Open(examples+"/13_ReadPhases.usda");
    if(!stage)return;
    const auto rig=_FindRig(stage);
    const SdfPath cage("/ReadPhaseAsset/Geom/Cage.points");
    const SdfPath slab("/ReadPhaseAsset/Geom/Slab.points");
    stage->SetEditTarget(stage->GetSessionLayer());
    const auto attribute=stage->GetAttributeAtPath(cage);
    VtVec3fArray authored,rawSlab;
    CHECK(attribute.Get(&authored));
    CHECK(stage->GetAttributeAtPath(slab).Get(&rawSlab));
    CHECK(attribute.Set(authored,UsdTimeCode(1001)));
    CHECK(attribute.Set(SdfValueBlock(),UsdTimeCode(1002)));
    CHECK(attribute.Set(authored,UsdTimeCode(1003)));
    std::string rootBefore,sessionBefore;
    CHECK(stage->GetRootLayer()->ExportToString(&rootBefore));
    CHECK(stage->GetSessionLayer()->ExportToString(&sessionBefore));
    std::vector<uint8_t> bytes;
    CHECK(_BakeArrays("retained point Final availability",stage,rig,1001,&bytes));
    std::string error;
    auto reader=RigExecRuntimeReader::Open(bytes.data(),bytes.size(),&error);
    CHECK(reader);if(!reader)return;
    RigExecInputSampler sampler;
    CHECK(sampler.Bind(stage,*reader,&error));
    CHECK(sampler.GetWarnings().empty());
    RigExecRigEvaluator evaluator(stage,rig);
    std::vector<std::string> errors;
    CHECK(evaluator.Compile(&errors));
    const auto epoch=evaluator.GetBindingEpochDigest();
    const std::string fallback="diag /ReadPhaseAsset/Rig/Movers/Geometry/SlabLattice: read phase 'final' for "+cage.GetString()+" resolved to nothing; read the authored base";
    VtVec3fArray first,recovered;
    for(double frame:{1001.0,1002.0,1003.0}) {
        const bool blocked=frame==1002.0;
        CHECK(RigExecTestDrive(reader.get(),&sampler,frame,&error));
        const auto pose=evaluator.Evaluate(UsdTimeCode(frame));
        CHECK(pose.valid && evaluator.GetBindingEpochDigest()==epoch);
        std::vector<std::string> diffs;
        const bool same=RigExecCompareRuntimeRun(pose,*reader,&diffs);
        if(!same)for(const auto &line:diffs)std::printf("retained point availability frame%g: %s\n",frame,line.c_str());
        CHECK(same);
        CHECK((std::find(reader->GetDiagnostics().begin(),reader->GetDiagnostics().end(),fallback)!=reader->GetDiagnostics().end())==blocked);
        CHECK((pose.movedProperties.count(cage)!=0)==!blocked);
        const auto result=pose.movedProperties.find(slab);
        CHECK(result!=pose.movedProperties.end() && result->second.IsHolding<VtVec3fArray>());
        if(result!=pose.movedProperties.end() && result->second.IsHolding<VtVec3fArray>()) {
            const auto &points=result->second.UncheckedGet<VtVec3fArray>();
            if(blocked)CHECK(points==rawSlab);
            else if(frame==1001.0)first=points;
            else recovered=points;
        }
        const auto diagnostics=reader->GetDiagnostics();
        CHECK(reader->Execute(&error));
        CHECK(reader->GetCounters().executedOpCount==0);
        CHECK(reader->GetDiagnostics()==diagnostics);
        diffs.clear();CHECK(RigExecCompareRuntimeRun(pose,*reader,&diffs));
    }
    std::string rootAfter,sessionAfter;
    CHECK(stage->GetRootLayer()->ExportToString(&rootAfter));
    CHECK(stage->GetSessionLayer()->ExportToString(&sessionAfter));
    CHECK(rootAfter==rootBefore && sessionAfter==sessionBefore);
    CHECK(!first.empty() && !recovered.empty() && first!=recovered);
}

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
          rows(answered, &live, &rest) && live && rest);

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
    {
        const char *text = "native";
        RigExecRigPose pose;
        std::string error;
        std::vector<std::string> diffs;
        if (!reader ||
            !RigExecTestArrayReference(stage, rigPath, {}, 1002.0, &pose,
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
    std::printf("%s: raw fallback bound beside the phase; blocked at "
                "t=1002 %s live, a sampled set %s\n",
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
            // Identical bytes still change the Default read when the set
            // switches between sampled and authored. Compare both directions
            // with independently executed readers holding each kind.
            std::unique_ptr<RigExecRuntimeReader> authored =
                _OpenRun(label, listed);
            CHECK(authored && authored->SetInputArrayAt(
                      index, RigExecTestArrayView(VtValue(scaled)), &error) &&
                  authored->Execute(&error));
            CHECK(a->SetSampledInputArrayAt(
                      index, RigExecTestArrayView(VtValue(scaled)), &error) &&
                  a->Execute(&error) && _SameOutputs(*a, *s));
            CHECK(a->SetInputArrayAt(
                      index, RigExecTestArrayView(VtValue(scaled)), &error) &&
                  a->Execute(&error) && authored &&
                  _SameOutputs(*a, *authored) && !_SameOutputs(*a, *s));
            CHECK(s->SetInputArrayAt(
                      index, RigExecTestArrayView(VtValue(scaled)), &error) &&
                  s->Execute(&error) && authored &&
                  _SameOutputs(*s, *authored));
            CHECK(s->SetSampledInputArrayAt(
                      index, RigExecTestArrayView(VtValue(scaled)), &error) &&
                  s->Execute(&error) && !_SameOutputs(*s, *a));
            CHECK(a->ResetInput(cage, &error) && a->Execute(&error) &&
                  _SameOutputs(*plain, *a));
        }
    }

    // Both cage reads share a slot: stage sampling changes only its AtTime
    // side; the bind cage keeps its captured Default value.
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
              player.Sampler().GetAnimatedCount() == animatedScalars +
                  RigExecRuntimeStageArrayInputs::Enumerate(*player.operator->()).size());
        CHECK(player.Play(1036.0, &error));
        RigExecRuntimeArray sampled;
        CHECK(player->GetInputArrayAt(index, &sampled) &&
              sampled.count == 12);
        RigExecRuntimeArray atOpen;
        std::unique_ptr<RigExecRuntimeReader> fresh = _OpenRun(label, listed);
        VtVec3fArray atTime;
        CHECK(stage->GetAttributeAtPath(SdfPath(cage))
                  .Get(&atTime, UsdTimeCode(1036.0)));
        CHECK(atTime.size() == sampled.count &&
              std::memcmp(sampled.data, atTime.cdata(),
                          sizeof(GfVec3f) * sampled.count) == 0);
        CHECK(fresh && fresh->GetInputArrayAt(index, &atOpen) &&
              std::memcmp(sampled.data, atOpen.data,
                          sizeof(GfVec3f) * 12) != 0);
        {
            RigExecRigPose pose;
            CHECK(RigExecTestArrayReference(stage, rigPath, {}, 1036.0,
                                             &pose, &error));
            std::vector<std::string> diffs;
            CHECK(RigExecCompareRuntimeRun(pose, *player.operator->(), &diffs));
        }
        std::printf("%s: the sampler samples %zu input(s) of %zu, the "
                    "array sampled at playback time\n",
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

// MeshA.points' ChainInput moves its content version at publication,
// against the base it last published, as the native PublishLeaves does: a
// base sampled away and back between two publications moves nothing, so
// that run is the run of a reader that never sampled it; a base that stays
// moved still publishes moved.
static void
_TestChainInputPublishes(const UsdStageRefPtr &stage)
{
    const std::string label = "oneloop_two_limbs MeshA.points publication";
    const std::string target = "/LimbsAsset/Geom/MeshA.points";
    const SdfPath rigPath = _FindRig(stage);
    std::vector<uint8_t> bytes;
    if (!_BakeArrays(label, stage, rigPath, 6.0, &bytes)) {
        return;
    }
    std::unique_ptr<RigExecRuntimeReader> plain = _OpenRun(label, bytes);
    std::unique_ptr<RigExecRuntimeReader> sampled = _OpenRun(label, bytes);
    if (!plain || !sampled) {
        return;
    }
    VtVec3fArray base;
    CHECK(stage->GetAttributeAtPath(SdfPath(target)).Get(&base));
    CHECK(!base.empty());
    VtVec3fArray displaced = base;
    for (GfVec3f &p : displaced) {
        p += GfVec3f(0.0f, 0.25f, 0.0f);
    }
    std::string error;
    // The same input calls on both; only `sampled` runs a prologue on the
    // displaced base before the reset.
    for (RigExecRuntimeReader *reader : {plain.get(), sampled.get()}) {
        CHECK(reader->SetInputArray(
            target, RigExecTestArrayView(VtValue(displaced)), &error));
        if (reader == sampled.get()) {
            CHECK(reader->SampleGeometryForTesting(&error));
        }
        CHECK(reader->ResetInput(target, &error));
        CHECK(reader->Execute(&error));
    }
    CHECK(sampled->GetLastRunTraceForTesting() ==
              plain->GetLastRunTraceForTesting() &&
          _SameRuns(*plain, *sampled));
    // A base that stays displaced publishes moved points.
    CHECK(sampled->SetInputArray(
              target, RigExecTestArrayView(VtValue(displaced)), &error) &&
          sampled->Execute(&error));
    CHECK(plain->Execute(&error));
    CHECK(!_SameOutputs(*plain, *sampled));
    std::printf("%s: a base sampled and reset between publications keeps "
                "its version\n", label.c_str());
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
            CHECK(idle.empty() && idleCounters.executedOpCount == 0);
            size_t at = 0;
            CHECK(played.FindInput(weightsName, &at) &&
                  played.SetInputArrayAt(
                      at, RigExecTestArrayView(VtValue(swapped)), &error) &&
                  played.Execute(&error));
            CHECK(played.GetLastRunTraceForTesting() == idle &&
                  played.GetCounters().executedOpCount ==
                      idleCounters.executedOpCount);
            // The reset returns the Open layout itself, and its outputs.
            CHECK(played.ResetInput(weightsName, &error) &&
                  played.Execute(&error));
            CHECK(played.GetSkinLayoutIsOpenForTesting(mover));
            CHECK(plain && _SameOutputs(*plain, played));
        });
    // The indices are topology, fixed for the reader: a sampled set of
    // another count is refused with its reason and changes nothing, while
    // one of their own count is taken. The weights are values: a sampled set
    // of theirs of another count is taken, and the mover fails as live does.
    {
        std::unique_ptr<RigExecRuntimeReader> played = _OpenRun(label, listed);
        size_t at = 0, painted = 0;
        CHECK(played && played->FindInput(indicesName, &at) &&
              played->FindInput(weightsName, &painted));
        if (played) {
            std::string why;
            const VtIntArray shorter(indices.begin(), indices.end() - 2);
            CHECK(!played->SetSampledInputArrayAt(
                at, RigExecTestArrayView(VtValue(shorter)), &why));
            CHECK(why == indicesName + " is topology, fixed at " +
                             std::to_string(indices.size()) +
                             " elements for this reader; a sampled set of " +
                             std::to_string(shorter.size()) + " is refused");
            RigExecRuntimeArray kept;
            CHECK(played->GetInputArrayAt(at, &kept) &&
                  kept.count == indices.size());
            CHECK(played->Execute(&why) && plain &&
                  _SameOutputs(*plain, *played));
            CHECK(played->GetSkinLayoutIsOpenForTesting(mover));
            VtIntArray reordered = indices;
            std::swap(reordered[0], reordered[1]);
            CHECK(played->SetSampledInputArrayAt(
                at, RigExecTestArrayView(VtValue(reordered)), &why));
            const VtFloatArray fewer(weights.begin(), weights.end() - 2);
            CHECK(played->SetSampledInputArrayAt(
                painted, RigExecTestArrayView(VtValue(fewer)), &why));
            CHECK(played->Execute(&why));
            CHECK(!played->GetSkinLayoutIsOpenForTesting(mover));
        }
    }
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

// Painted arrays remain private storage with the authored defaults. A
// crafted file exposes them to exercise the runtime API independently of
// the exporter's admission contract. An authored set reaches the packet; a
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
    std::unique_ptr<RigExecRuntimeReader> privateReader =
        _OpenRun(label + ": private defaults", bytes);
    CHECK(privateReader);
    if (!privateReader) {
        return;
    }
    for (const std::string &input : {seg1, seg4, seg4Indices}) {
        size_t index = 0;
        CHECK(!privateReader->FindInput(input, &index));
        const auto file = RigExecTestUnpack(bytes);
        CHECK(file);
        const int64_t slot = file ? RigExecTestSlotOf(*file, input) : -1;
        CHECK(slot >= 0 && uint64_t(slot) >= file->listedInputs);
        RigExecRuntimeArray value;
        CHECK(!privateReader->GetInputArrayAt(size_t(slot), &value));
    }
    const std::vector<uint8_t> listed =
        RigExecTestListPrivateArraySlots(bytes);
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
                played.GetDiagnostics();
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

// Real numeric-time samples, not public setter simulations. A second reader
// holds only this array at its bake value at the same frame, proving that the
// array (rather than an unrelated animated scalar) changes the published pose.
// For \p topology, whose count is fixed for the reader, the \p invalid
// sample of another count is refused with its reason instead, and the
// reader keeps the elements it held.
static void
_CheckStageArraySamples(const std::string &label, const UsdStageRefPtr &stage,
                        const SdfPath &rigPath, const std::string &name,
                        double time, const VtValue &initial,
                        const VtValue &changed, const VtValue &invalid,
                        const VtValue &extra = VtValue(),
                        bool topology = false)
{
    const UsdAttribute attribute = stage->GetAttributeAtPath(SdfPath(name));
    CHECK(attribute);
    if (!attribute) return;
    {
        UsdEditContext context(stage, stage->GetSessionLayer());
        // A single key marks the original file Animated. The owning layout
        // operation consumes later samples, including count loss and recovery.
        CHECK(attribute.Set(initial, UsdTimeCode(time)));
    }
    std::vector<uint8_t> bytes;
    if (!_BakeArrays(label, stage, rigPath, time, &bytes)) return;
    auto reader = _OpenRun(label, bytes);
    auto held = _OpenRun(label + " held", bytes);
    CHECK(reader && held);
    if (!reader || !held) return;
    const auto slots = RigExecRuntimeStageArrayInputs::Enumerate(*reader);
    const auto found = std::find_if(slots.begin(), slots.end(),
        [&](const auto &info) { return info.name == name; });
    CHECK(found != slots.end());
    if (found == slots.end()) return;
    const auto rejectSlot = [&](size_t slot) {
        std::string why;
        const std::string expected = "no sampled array at slot " +
                                     std::to_string(slot);
        CHECK(!RigExecRuntimeStageArrayInputs::CanSample(*reader, slot));
        CHECK(!RigExecRuntimeStageArrayInputs::SetSample(
            *reader, slot, RigExecTestArrayView(initial), &why));
        CHECK(why == expected);
        why.clear();
        CHECK(!RigExecRuntimeStageArrayInputs::ClearSample(*reader, slot, &why));
        CHECK(why == expected);
    };
    const auto file = RigExecTestUnpack(bytes);
    CHECK(file);
    if (file) rejectSlot(file->inputs.size());
    bool scalarChecked = false;
    for (size_t slot = 0; slot < reader->GetInputCount(); ++slot) {
        if (!RrInputTagIsArray(reader->GetInputInfo(slot).type)) {
            rejectSlot(slot);
            scalarChecked = true;
            break;
        }
    }
    CHECK(scalarChecked);
    std::string bridgeError;
    CHECK(reader->Execute(&bridgeError));
    CHECK(_SameOutputs(*reader, *held));
    {
        RigExecRigEvaluator evaluator(stage, rigPath);
        evaluator.Evaluate(UsdTimeCode(time));
        std::vector<RigExecBakeStaticEntry> entries;
        std::string why;
        CHECK(RigExecBakeStaticReport(evaluator, &entries, &why));
        CHECK(std::none_of(entries.begin(), entries.end(),
            [&](const auto &entry) { return entry.source == name; }));
    }
    CHECK(file && found->slot < file->inputs.size() &&
          (file->inputs[found->slot].flags() &
           uint8_t(fb::InputSlotFlags::Animated)) != 0);
    {
        UsdEditContext context(stage, stage->GetSessionLayer());
        CHECK(attribute.Set(changed, UsdTimeCode(time + 1)));
        CHECK(attribute.Set(invalid, UsdTimeCode(time + 2)));
        CHECK(attribute.Set(initial, UsdTimeCode(time + 3)));
        if (!extra.IsEmpty()) {
            CHECK(attribute.Set(extra, UsdTimeCode(time + 4)));
            CHECK(attribute.Set(initial, UsdTimeCode(time + 5)));
        }
    }
    RigExecInputSampler sampler, heldSampler;
    std::string error;
    CHECK(sampler.Bind(stage, *reader, &error));
    CHECK(heldSampler.Bind(stage, *held, &error));
    CHECK(sampler.GetWarnings().empty());
    for (int step = 1; step <= (extra.IsEmpty() ? 3 : 5); ++step) {
        const double frame = time + step;
        if (topology && step == 2) {
            const size_t count = RigExecTestArrayView(invalid).count;
            CHECK(count != RigExecTestArrayView(initial).count);
            CHECK(!RigExecTestDrive(reader.get(), &sampler, frame, &error));
            CHECK(error == name + " is topology, fixed at " +
                               std::to_string(
                                   RigExecTestArrayView(initial).count) +
                               " elements for this reader; a sampled set "
                               "of " + std::to_string(count) +
                               " is refused");
            RigExecRuntimeArray kept;
            if (found->slot < reader->GetInputCount()) {
                CHECK(reader->GetInputArrayAt(found->slot, &kept) &&
                      kept.count == RigExecTestArrayView(changed).count);
            }
            std::printf("%s frame %g: %s\n", label.c_str(), frame,
                        error.c_str());
            continue;
        }
        CHECK(RigExecTestDrive(reader.get(), &sampler, frame, &error));
        {
            RigExecRigPose pose;
            CHECK(RigExecTestArrayReference(stage, rigPath, {}, frame,
                                             &pose, &error));
            std::vector<std::string> diffs;
            const bool equal = RigExecCompareRuntimeRun(pose, *reader, &diffs);
            CHECK(equal);
            for (const auto &line : diffs)
                std::printf("%s frame %g: %s\n", label.c_str(), frame,
                            line.c_str());
        }
        if (step == 1) {
            CHECK(RigExecTestDrive(held.get(), &heldSampler, frame, &error));
            CHECK(RigExecRuntimeStageArrayInputs::SetSample(
                *held, found->slot, RigExecTestArrayView(initial), &error));
            CHECK(held->Execute(&error));
            CHECK(!_SameOutputs(*reader, *held));
        }
        bool sampled = true;
        CHECK(sampler.Apply(UsdTimeCode(frame), reader.get(), &error, &sampled));
        CHECK(!sampled);
    }
    // Sampling arbitrary counts never relaxes the authored count contract.
    CHECK(!reader->SetInputArray(name, RigExecTestArrayView(invalid), &error));
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
        VtValue invalid;
        switch (c.tag) {
        case fb::InputTag::IntArray: {
            const auto &indices = held.Get<VtIntArray>();
            CHECK(indices.size() >= 6);
            // One complete quad followed by an incomplete second face.
            invalid = VtValue(VtIntArray(indices.begin(), indices.begin() + 6));
            break;
        }
        case fb::InputTag::Vec2fArray: invalid = VtValue(VtVec2fArray()); break;
        case fb::InputTag::Vec3fArray: invalid = VtValue(VtVec3fArray()); break;
        default: CHECK(false); break;
        }
        if (std::string(c.stage) == "05_TwistRibbonSpine.usda") {
            // RibbonIk's structural driver is outside the sampled leaf table.
            // Hold it in a stronger Default while testing the real UV leaf.
            const auto driver = stage->GetAttributeAtPath(
                SdfPath("/SpineAsset/Geom/SpineCurve.points"));
            VtVec3fArray points;
            CHECK(driver.Get(&points, UsdTimeCode(c.time)));
            UsdEditContext context(stage, stage->GetSessionLayer());
            CHECK(driver.Set(points));
        }
        // A mesh's face indices are topology: their invalid sample, of
        // another count, is refused.
        const bool topology =
            std::string(c.attribute).find(".faceVertexIndices") !=
            std::string::npos;
        _CheckStageArraySamples(label + " stage samples", stage, rigPath,
                                c.attribute, c.time, held, c.edit(held),
                                invalid, VtValue(), topology);
    }
}

// Sparse leaf publication: a leaf keyed from input slots alone (a provider
// leaf, a constraint's arrays) is re-keyed only after a write to a slot it
// reads. Under RIGEXEC_VERIFY_SPARSE_LEAVES every Execute also re-keys the
// leaves it skipped and fails if one moved, so each write route below has
// to mark its slot: the sampler over time, a sampled set, a blocked flag, a
// sampled clear, an authored set, a clear, a reset. A run after no write
// re-keys none, nor does one after a repeat that moves no keyed field, and
// one write re-keys fewer than all.
static void
_TestSparseSlotLeaves(const std::string &examples)
{
    const UsdStageRefPtr stage = _Open(examples + "/biped/Biped_anim.usda");
    if (!stage) return;
    std::vector<uint8_t> bytes;
    std::string error;
    {
        RigExecRigEvaluator evaluator(stage, _FindRig(stage));
        if (!RigExecTestBakeAt(evaluator, 1.0, &bytes, &error)) {
            std::printf("sparse leaves: bake: %s\n", error.c_str());
            CHECK(false);
            return;
        }
    }
    // The runtime reads its knobs through the C runtime's environment.
    const auto setKnob = [](const char *value) {
#if defined(_WIN32)
        _putenv_s("RIGEXEC_VERIFY_SPARSE_LEAVES", value);
#else
        if (*value) setenv("RIGEXEC_VERIFY_SPARSE_LEAVES", value, 1);
        else unsetenv("RIGEXEC_VERIFY_SPARSE_LEAVES");
#endif
    };
    RigExecTestPlayer player;
    setKnob("1");
    const bool opened = player.Open(bytes, stage, &error);
    setKnob("");
    if (!opened) {
        std::printf("sparse leaves: open: %s\n", error.c_str());
        CHECK(false);
        return;
    }
    RigExecRuntimeReader &reader = player.Reader();
    const auto run = [&](const char *what) {
        error.clear();
        const bool ok = reader.Execute(&error);
        if (!ok) std::printf("sparse leaves, %s: %s\n", what, error.c_str());
        CHECK(ok);
        return reader.GetSlotLeafKeysForTesting();
    };
    CHECK(player.Play(1.0, &error));
    const size_t all = reader.GetSlotLeafKeysForTesting();
    CHECK(all > 0);
    CHECK(run("held") == 0);
    CHECK(player.Play(2.0, &error));
    const size_t sampled = reader.GetSlotLeafKeysForTesting();
    CHECK(run("held after time") == 0);

    // A provider leaf's own source slot, scalar and sampleable: the first
    // Double slot a slot-keyed leaf reads, else the first such Matrix4d
    // slot. Pruning drops the provider leaves no step reads (the animated
    // avars' among them), so the slot can be one EnumerateProviderValues
    // does not list: a non-animated stage-sampled raw source.
    size_t slot = reader.GetInputCount();
    RrInputTag tag = RrInputTag::Double;
    for (const RrInputTag want : {RrInputTag::Double, RrInputTag::Matrix4d}) {
        for (size_t s = 0; s < reader.GetInputCount(); ++s) {
            if (reader.GetInputInfo(s).type == want &&
                RigExecRuntimeStageArrayInputs::CanSampleProviderValue(reader, s) &&
                reader.GetSlotLeafCountForTesting(s) > 0) {
                slot = s;
                tag = want;
                break;
            }
        }
        if (slot < reader.GetInputCount()) break;
    }
    CHECK(slot < reader.GetInputCount());
    if (slot >= reader.GetInputCount()) return;
    // The slot's value: the number, or a translation by it.
    const auto valueOf = [tag](double x) {
        RrInputValue v;
        v.tag = tag;
        v.f64 = x;
        v.matrix = RrMat4d(1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, x, 0, 0, 1);
        return v;
    };
    RrInputValue value = valueOf(0.375);
    CHECK(RigExecRuntimeStageArrayInputs::SetScalarSample(reader, slot, value, &error));
    const size_t one = run("sampled set");
    CHECK(one > 0 && one < all);
    std::printf("sparse slot leaves: %zu, re-keyed after a time step %zu, "
                "after one input %zu (%s, %s)\n", all, sampled, one,
                reader.GetInputInfo(slot).name.c_str(),
                tag == RrInputTag::Double ? "Double" : "Matrix4d");
    CHECK(RigExecRuntimeStageArrayInputs::SetScalarSample(reader, slot, value, &error));
    CHECK(run("repeated sampled set") == 0);
    CHECK(RigExecRuntimeStageArrayInputs::SetSampleBlocked(reader, slot, true, &error));
    CHECK(run("blocked") == one);
    CHECK(RigExecRuntimeStageArrayInputs::SetSampleBlocked(reader, slot, false, &error));
    CHECK(run("unblocked") == one);
    CHECK(RigExecRuntimeStageArrayInputs::ClearScalarSample(reader, slot, &error));
    CHECK(run("sampled clear") == one);
    value = valueOf(-1.25);
    CHECK(reader.SetInputAt(slot, value, &error));
    CHECK(run("authored set") == one);
    CHECK(reader.ClearInputAt(slot, &error));
    CHECK(run("clear") == one);
    CHECK(reader.SetInputAt(slot, value, &error));
    run("authored set again");
    CHECK(reader.ResetInput(reader.GetInputInfo(slot).name, &error));
    CHECK(run("reset") == one);
    CHECK(run("held after reset") == 0);
    reader.ResetInputs();
    run("reset all");
    CHECK(player.Play(3.0, &error));
    CHECK(run("held at the end") == 0);
}

// Sparse source memos: a step's source memo reads input slots and Open-time
// tables only, so a run rebuilds just the memos of steps reading a slot
// some call wrote. Under RIGEXEC_VERIFY_SOURCE_KEYS every Execute also
// rebuilds the memos it keeps and fails if one moved. A run after no write
// builds none; an avar written alone builds some but not all, its move and
// its reset build the same ones, and its repeat none.
static void
_TestSparseSourceMemos(const std::string &examples)
{
    const UsdStageRefPtr stage = _Open(examples + "/biped/Biped_anim.usda");
    if (!stage) return;
    std::vector<uint8_t> bytes;
    std::string error;
    {
        RigExecRigEvaluator evaluator(stage, _FindRig(stage));
        if (!RigExecTestBakeAt(evaluator, 1.0, &bytes, &error)) {
            std::printf("sparse source memos: bake: %s\n", error.c_str());
            CHECK(false);
            return;
        }
    }
    // The runtime reads its knobs through the C runtime's environment.
    const auto setKnob = [](const char *value) {
#if defined(_WIN32)
        _putenv_s("RIGEXEC_VERIFY_SOURCE_KEYS", value);
#else
        if (*value) setenv("RIGEXEC_VERIFY_SOURCE_KEYS", value, 1);
        else unsetenv("RIGEXEC_VERIFY_SOURCE_KEYS");
#endif
    };
    RigExecTestPlayer player;
    setKnob("1");
    const bool opened = player.Open(bytes, stage, &error);
    setKnob("");
    if (!opened) {
        std::printf("sparse source memos: open: %s\n", error.c_str());
        CHECK(false);
        return;
    }
    RigExecRuntimeReader &reader = player.Reader();
    const auto run = [&](const char *what) {
        error.clear();
        const bool ok = reader.Execute(&error);
        if (!ok) std::printf("sparse source memos, %s: %s\n", what, error.c_str());
        CHECK(ok);
        return reader.GetSourceKeysBuiltForTesting();
    };
    CHECK(player.Play(1.0, &error));
    const size_t all = reader.GetSourceKeysBuiltForTesting();
    CHECK(all > 0);
    CHECK(run("held") == 0);
    CHECK(player.Play(2.0, &error));
    CHECK(run("held after time") == 0);

    // A static avar's own slot, which its provider's AvarInputs memo reads.
    const size_t count = reader.GetInputCount();
    size_t slot = count, one = 0;
    RrInputValue value;
    value.tag = RrInputTag::Double;
    value.f64 = 0.375;
    for (size_t s = 0; s < count && slot == count; ++s) {
        const RigExecRuntimeInputInfo &info = reader.GetInputInfo(s);
        if (info.type != RrInputTag::Double || info.animated ||
            info.name.find(".avars:") == std::string::npos) {
            continue;
        }
        error.clear();
        if (!reader.SetInputAt(s, value, &error)) continue;
        const size_t built = run("avar set");
        if (built > 0) {
            slot = s;
            one = built;
        }
    }
    CHECK(slot < count);
    if (slot >= count) return;
    CHECK(one < all);
    CHECK(reader.SetInputAt(slot, value, &error));
    CHECK(run("repeated set") == 0);
    value.f64 = -1.25;
    CHECK(reader.SetInputAt(slot, value, &error));
    CHECK(run("moved set") == one);
    CHECK(run("held after the set") == 0);
    CHECK(reader.ResetInput(reader.GetInputInfo(slot).name, &error));
    CHECK(run("reset") == one);
    CHECK(run("held after reset") == 0);
    std::printf("sparse source memos: %zu keyed, after one avar %zu\n", all, one);
    CHECK(player.Play(3.0, &error));
    CHECK(run("held at the end") == 0);
}

// A runtime knob for the readers opened while it stands: the runtime reads
// its knobs at Open through the C runtime's environment. Unset at the end
// of its scope.
class _RtInputsKnob
{
public:
    _RtInputsKnob(const char *name, const char *value) : _name(name)
    {
        _Put(_name, value);
    }
    ~_RtInputsKnob() { _Put(_name, ""); }
    _RtInputsKnob(const _RtInputsKnob &) = delete;
    _RtInputsKnob &operator=(const _RtInputsKnob &) = delete;

private:
    static void _Put(const char *name, const char *value)
    {
#if defined(_WIN32)
        _putenv_s(name, value);
#else
        if (*value) setenv(name, value, 1);
        else unsetenv(name);
#endif
    }
    const char *_name;
};

// Written slots: a set that leaves a slot's value bits, HasValue, authored
// mark and blocked flag where its readers were last keyed writes nothing a
// run consumes, so that run re-keys no leaf and builds no source memo;
// another value re-keys what the first set did. Under the sparse leaf and
// source key judges every run passes. A reader opened with
// RIGEXEC_RUNTIME_WRITTEN_FILTER=0 plays the same outputs and re-keys on
// every repeat.
static void
_TestARepeatedSetWritesNothing(const std::vector<uint8_t> &bytes)
{
    std::string error;
    std::unique_ptr<RigExecRuntimeReader> filtered, recorded;
    {
        const _RtInputsKnob leaves("RIGEXEC_VERIFY_SPARSE_LEAVES", "1");
        const _RtInputsKnob sources("RIGEXEC_VERIFY_SOURCE_KEYS", "1");
        filtered =
            RigExecRuntimeReader::Open(bytes.data(), bytes.size(), &error);
        const _RtInputsKnob off("RIGEXEC_RUNTIME_WRITTEN_FILTER", "0");
        recorded =
            RigExecRuntimeReader::Open(bytes.data(), bytes.size(), &error);
    }
    CHECK(filtered && recorded);
    if (!filtered || !recorded) {
        std::printf("repeated set: open: %s\n", error.c_str());
        return;
    }
    // The leaves a reader's last run re-keyed and the source memos it built.
    using _Keys = std::pair<size_t, size_t>;
    const _Keys none(0, 0);
    const auto run = [&](RigExecRuntimeReader &reader, const char *what) {
        error.clear();
        const bool ok = reader.Execute(&error);
        if (!ok) std::printf("repeated set, %s: %s\n", what, error.c_str());
        CHECK(ok);
        return _Keys(reader.GetSlotLeafKeysForTesting(),
                     reader.GetSourceKeysBuiltForTesting());
    };
    // Both readers take \p value at input \p slot and run, and play the
    // same outputs: the filtered reader's keys, the other's in `unfiltered`.
    _Keys unfiltered = none;
    const auto setBoth = [&](size_t slot, double value, const char *what) {
        RrInputValue typed;
        typed.tag = RrInputTag::Double;
        typed.f64 = value;
        error.clear();
        CHECK(filtered->SetSampledInputAt(slot, typed, &error));
        CHECK(recorded->SetSampledInputAt(slot, typed, &error));
        const _Keys keys = run(*filtered, what);
        unfiltered = run(*recorded, what);
        CHECK(_SameOutputs(*filtered, *recorded));
        return keys;
    };
    run(*filtered, "first");
    run(*recorded, "first");
    CHECK(run(*filtered, "held") == none);
    CHECK(run(*recorded, "held") == none);

    // A listed Double provider value that some leaf or memo reads.
    const size_t count = filtered->GetInputCount();
    size_t slot = count, tried = 0;
    _Keys keyed = none;
    for (const RigExecStageArrayInputInfo &info :
         RigExecRuntimeStageArrayInputs::EnumerateProviderValues(*filtered)) {
        if (info.tag != RrInputTag::Double || info.slot >= count ||
            tried == 16) {
            continue;
        }
        ++tried;
        keyed = setBoth(info.slot, 0.375, "set");
        if (keyed != none) {
            slot = info.slot;
            break;
        }
    }
    CHECK(slot < count);
    if (slot >= count) {
        return;
    }
    CHECK(unfiltered == keyed);
    CHECK(setBoth(slot, 0.375, "repeated set") == none);
    CHECK(unfiltered == keyed);
    CHECK(setBoth(slot, -1.25, "moved set") == keyed);
    CHECK(unfiltered == keyed);
    CHECK(setBoth(slot, -1.25, "repeated moved set") == none);
    CHECK(unfiltered == keyed);
    CHECK(run(*filtered, "held") == none);
    std::printf("written filter: a set of %s re-keys %zu leaves and builds "
                "%zu memos, its repeat none (%zu without the filter)\n",
                filtered->GetInputInfo(slot).name.c_str(), keyed.first,
                keyed.second, unfiltered.first + unfiltered.second);
}

// Static provider inputs: a sampler told SetStaticInputSkip after Bind, as
// playback tells it, reads a bound input the file does not mark Animated,
// on an attribute that cannot vary with time, only when an Apply refreshes
// (its first sampling one, one after NoteStageChanged, one that changes
// Default-ness), and plays bit for bit what a sampler reading every input
// plays. A Default edit of such an input plays after NoteStageChanged as a
// sampler bound after the edit plays it; a skip sampler never told keeps
// what it read. Keys authored after Bind are followed after the next
// NoteStageChanged.
static void
_TestStaticProviderInputsAreReadOnce(const UsdStageRefPtr &stage,
                                     const std::vector<uint8_t> &bytes)
{
    struct _Played {
        std::unique_ptr<RigExecRuntimeReader> reader;
        RigExecInputSampler sampler;
    };
    std::string error;
    // A reader of the bake with a sampler bound to the stage as it is now.
    const auto open = [&](_Played *played) {
        error.clear();
        played->reader =
            RigExecRuntimeReader::Open(bytes.data(), bytes.size(), &error);
        const bool ok = played->reader &&
                        played->sampler.Bind(stage, *played->reader, &error) &&
                        played->sampler.GetWarnings().empty();
        if (!ok) std::printf("static inputs: open: %s\n", error.c_str());
        CHECK(ok);
        return ok;
    };
    const auto play = [&](_Played *played, UsdTimeCode time) {
        error.clear();
        const bool ok =
            played->sampler.Apply(time, played->reader.get(), &error) &&
            played->reader->Execute(&error);
        if (!ok) std::printf("static inputs: play: %s\n", error.c_str());
        CHECK(ok);
    };
    // A reader played at \p frame over the stage as it is now; null when it
    // does not open.
    const auto fresh = [&](double frame) {
        auto played = std::make_unique<_Played>();
        if (open(played.get())) {
            play(played.get(), UsdTimeCode(frame));
        } else {
            played.reset();
        }
        return played;
    };
    _Played skip, plain, unnoted;
    if (!open(&skip) || !open(&plain) || !open(&unnoted)) {
        return;
    }
    // Set after Bind, as playback sets it.
    skip.sampler.SetStaticInputSkip(true);
    unnoted.sampler.SetStaticInputSkip(true);
    // The bound inputs the file does not mark Animated: provider values
    // read from their source, static at the bake.
    std::vector<RigExecStageArrayInputInfo> statics;
    for (const RigExecStageArrayInputInfo &info :
         RigExecRuntimeStageArrayInputs::EnumerateProviderValues(
             *skip.reader)) {
        if (!info.animated) {
            statics.push_back(info);
        }
    }
    CHECK(!statics.empty());
    if (statics.empty()) {
        return;
    }
    const size_t animated = skip.sampler.GetAnimatedCount();
    const size_t bound = animated + statics.size();
    // Frame 1 is the bake time, which makes no call; frame 2 refreshes and
    // reads every bound input; frames 3 and 4 read the Animated ones alone.
    for (int frame = 1; frame <= 4; ++frame) {
        for (_Played *played : {&skip, &plain, &unnoted}) {
            play(played, UsdTimeCode(double(frame)));
        }
        CHECK(_SameOutputs(*skip.reader, *plain.reader));
        CHECK(_SameOutputs(*unnoted.reader, *plain.reader));
        if (frame >= 2) {
            CHECK(plain.sampler.GetLastReadCount() == bound);
            CHECK(skip.sampler.GetLastReadCount() ==
                  (frame == 2 ? bound : animated));
        }
    }

    // A static Double input whose Default edit moves what the rig plays:
    // the reader that reads every input takes each candidate edit at frame
    // 4, where the skip reader still holds the unedited values.
    const std::unique_ptr<_Played> old5 = fresh(5.0);
    if (!old5) {
        return;
    }
    UsdAttribute edited;
    double original = 0.0;
    size_t tried = 0;
    for (const RigExecStageArrayInputInfo &info : statics) {
        if (info.tag != RrInputTag::Double || tried == 48) {
            continue;
        }
        const UsdAttribute attribute =
            stage->GetAttributeAtPath(SdfPath(info.name));
        double value = 0.0;
        if (!attribute || !attribute.Get(&value, UsdTimeCode(4.0)) ||
            !std::isfinite(value)) {
            continue;
        }
        ++tried;
        {
            UsdEditContext context(stage, stage->GetSessionLayer());
            CHECK(attribute.Set(value - 1.0));
        }
        plain.sampler.Invalidate();
        play(&plain, UsdTimeCode(4.0));
        if (!_SameOutputs(*plain.reader, *skip.reader)) {
            edited = attribute;
            original = value;
            break;
        }
        UsdEditContext context(stage, stage->GetSessionLayer());
        CHECK(attribute.Clear());
    }
    std::printf("static inputs: %zu of %zu bound inputs, %zu tried for an "
                "edit that moves the rig\n",
                statics.size(), bound, tried);
    CHECK(edited);
    if (!edited) {
        return;
    }

    // Frame 5: the reader that reads every input and the skip reader told
    // NoteStageChanged read the edit, as a reader bound after it does; the
    // skip reader never told keeps what it read (the documented semantics).
    const std::unique_ptr<_Played> edited5 = fresh(5.0);
    if (!edited5) {
        return;
    }
    CHECK(!_SameOutputs(*edited5->reader, *old5->reader));
    skip.sampler.NoteStageChanged();
    for (_Played *played : {&skip, &plain, &unnoted}) {
        play(played, UsdTimeCode(5.0));
    }
    CHECK(skip.sampler.GetLastReadCount() == bound);
    CHECK(unnoted.sampler.GetLastReadCount() == animated);
    CHECK(_SameOutputs(*skip.reader, *edited5->reader));
    CHECK(_SameOutputs(*plain.reader, *edited5->reader));
    CHECK(_SameOutputs(*unnoted.reader, *old5->reader));
    for (_Played *played : {&skip, &plain}) {
        play(played, UsdTimeCode(6.0));
    }
    CHECK(skip.sampler.GetLastReadCount() == animated);
    CHECK(_SameOutputs(*skip.reader, *plain.reader));

    // Keys authored after Bind: the refresh the next notice asks for finds
    // the input varying, and every later Apply reads it. Frame 8's key is
    // the edit's value, which the never-told reader does not hold.
    {
        UsdEditContext context(stage, stage->GetSessionLayer());
        CHECK(edited.Set(original, UsdTimeCode(7.0)));
        CHECK(edited.Set(original - 1.0, UsdTimeCode(8.0)));
    }
    skip.sampler.NoteStageChanged();
    for (const double frame : {7.0, 8.0}) {
        for (_Played *played : {&skip, &plain, &unnoted}) {
            play(played, UsdTimeCode(frame));
        }
        const std::unique_ptr<_Played> reference = fresh(frame);
        CHECK(reference && _SameOutputs(*skip.reader, *reference->reader));
        CHECK(_SameOutputs(*plain.reader, *skip.reader));
        CHECK(skip.sampler.GetLastReadCount() ==
              (frame == 7.0 ? bound : animated + 1));
    }
    CHECK(!_SameOutputs(*unnoted.reader, *skip.reader));

    // Default after numeric times refreshes and reads every bound input,
    // and so does the numeric time after it.
    for (_Played *played : {&skip, &plain}) {
        error.clear();
        CHECK(played->sampler.Apply(UsdTimeCode::Default(),
                                    played->reader.get(), &error));
    }
    CHECK(skip.sampler.GetLastReadCount() == bound);
    std::string skipWhy, plainWhy;
    const bool skipRan = skip.reader->Execute(&skipWhy);
    const bool plainRan = plain.reader->Execute(&plainWhy);
    CHECK(skipRan == plainRan && skipWhy == plainWhy);
    if (skipRan && plainRan) {
        CHECK(_SameOutputs(*skip.reader, *plain.reader));
    }
    for (const double frame : {9.0, 10.0}) {
        for (_Played *played : {&skip, &plain}) {
            play(played, UsdTimeCode(frame));
        }
        CHECK(_SameOutputs(*skip.reader, *plain.reader));
        CHECK(skip.sampler.GetLastReadCount() ==
              (frame == 9.0 ? bound : animated + 1));
    }
}

// RIGEXEC_PROVIDER_PRUNE set to \p on for the scope's lifetime, then
// restored; Build reads it.
struct _PruneKnob {
    explicit _PruneKnob(bool on) : saved(TfGetenv("RIGEXEC_PROVIDER_PRUNE"))
    {
        TfSetenv("RIGEXEC_PROVIDER_PRUNE", on ? "1" : "0");
    }
    ~_PruneKnob()
    {
        if (saved.empty()) {
            TfUnsetenv("RIGEXEC_PROVIDER_PRUNE");
        } else {
            TfSetenv("RIGEXEC_PROVIDER_PRUNE", saved);
        }
    }
    std::string saved;
};

// A pruned bake of Biped_anim whose first animated control avar is
// value-blocked at frame 2 (in the session layer, keyed at 1 and 3 with its
// own values, so it stays animated) plays frames 1-3 through the stage
// sampler as the evaluator publishes them, bit for bit: a slot whose
// provider leaf was pruned still reaches its readers. The provider-source
// slots only the unpruned bake lists are printed, the static ones marked
// (decision D7-1).
static void
_TestAPrunedBakeKeepsABlockedAnimatedAvar(const std::string &examples)
{
    const UsdStageRefPtr stage = _Open(examples + "/biped/Biped_anim.usda");
    if (!stage) {
        return;
    }
    const SdfPath rig = _FindRig(stage);
    UsdAttribute avar;
    for (const UsdPrim &prim : stage->Traverse()) {
        if (avar) {
            break;
        }
        if (!prim.GetPath().HasPrefix(rig) ||
            prim.GetTypeName() != TfToken("RigExecControl")) {
            continue;
        }
        for (const UsdAttribute &attribute : prim.GetAttributes()) {
            if (attribute.GetName().GetString().rfind("avars:", 0) == 0 &&
                attribute.ValueMightBeTimeVarying()) {
                avar = attribute;
                break;
            }
        }
    }
    CHECK(avar);
    if (!avar) {
        return;
    }
    VtValue first, last;
    CHECK(avar.Get(&first, UsdTimeCode(1.0)) &&
          avar.Get(&last, UsdTimeCode(3.0)));
    {
        UsdEditContext context(stage, stage->GetSessionLayer());
        CHECK(avar.Set(first, UsdTimeCode(1.0)));
        CHECK(avar.Set(SdfValueBlock(), UsdTimeCode(2.0)));
        CHECK(avar.Set(last, UsdTimeCode(3.0)));
    }
    CHECK(avar.ValueMightBeTimeVarying());
    // A blocked time sample resolves to no value (ValueIsBlocked describes
    // only a blocked default).
    VtValue blocked;
    CHECK(!avar.Get(&blocked, UsdTimeCode(2.0)));
    const std::string label =
        "pruned bake, " + avar.GetPath().GetString() + " blocked at 2";
    std::vector<uint8_t> bytes, unprunedBytes;
    bool baked = false, unprunedBaked = false;
    {
        const _PruneKnob knob(true);
        baked = _BakeArrays(label, stage, rig, 1.0, &bytes);
    }
    {
        const _PruneKnob knob(false);
        unprunedBaked = _BakeArrays(label + " (unpruned)", stage, rig, 1.0,
                                    &unprunedBytes);
    }
    if (!baked || !unprunedBaked) {
        return;
    }
    std::string error;
    std::unique_ptr<RigExecRuntimeReader> reader =
        RigExecRuntimeReader::Open(bytes.data(), bytes.size(), &error);
    CHECK(reader);
    if (!reader) {
        std::printf("%s: open: %s\n", label.c_str(), error.c_str());
        return;
    }
    RigExecInputSampler sampler;
    CHECK(sampler.Bind(stage, *reader, &error));
    CHECK(sampler.GetWarnings().empty());
    for (const std::string &warning : sampler.GetWarnings()) {
        std::printf("%s: sampler: %s\n", label.c_str(), warning.c_str());
    }
    // Compiled by its first Evaluate, so its poses carry the compile
    // warnings the bake recorded, as the reader's diagnostics do.
    RigExecRigEvaluator evaluator(stage, rig);
    for (const double frame : {1.0, 2.0, 3.0}) {
        CHECK(RigExecTestDrive(reader.get(), &sampler, frame, &error));
        const RigExecRigPose pose = evaluator.Evaluate(UsdTimeCode(frame));
        CHECK(pose.valid);
        std::vector<std::string> diffs;
        const bool same = RigExecCompareRuntimeRun(pose, *reader, &diffs);
        for (const std::string &line : diffs) {
            std::printf("%s frame %g: %s\n", label.c_str(), frame,
                        line.c_str());
        }
        CHECK(same);
    }
    // D7-1 evidence: the provider-source slots pruning took off the list.
    std::unique_ptr<RigExecRuntimeReader> unpruned = RigExecRuntimeReader::Open(
        unprunedBytes.data(), unprunedBytes.size(), &error);
    CHECK(unpruned);
    if (!unpruned) {
        return;
    }
    const auto listedPruned =
        RigExecRuntimeStageArrayInputs::EnumerateProviderValues(*reader);
    size_t lost = 0, lostStatic = 0;
    for (const RigExecStageArrayInputInfo &info :
         RigExecRuntimeStageArrayInputs::EnumerateProviderValues(*unpruned)) {
        const bool kept = std::any_of(
            listedPruned.begin(), listedPruned.end(),
            [&](const RigExecStageArrayInputInfo &other) {
                return other.name == info.name;
            });
        if (kept) {
            continue;
        }
        ++lost;
        lostStatic += info.animated ? 0 : 1;
        std::printf("  provider-source slot only the unpruned bake lists: "
                    "%s%s\n",
                    info.name.c_str(), info.animated ? "" : " (static)");
    }
    std::printf("%s: %zu of %zu provider-source slot(s) unlisted by pruning, "
                "%zu static\n",
                label.c_str(), lost, listedPruned.size() + lost, lostStatic);
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
        _TestChainInputPublishes(stage);
        _TestLayoutArrays(stage);
    }
    if (const UsdStageRefPtr stage =
            _Open(examples + "/01_FkChainTail.usda")) {
        _TestPaintedArrays(stage);
    }
    _TestExampleArrays(examples);
    // Extent widths accept one width, ignore malformed counts, and recover.
    // FrameGuides has three points; its extent is published as a property.
    if (const auto stage = _Open(examples + "/../docs/examples/curve_mover.usda")) {
        const std::string name = "/CurveAsset/Geom/FrameGuides.widths";
        VtFloatArray initial;
        CHECK(stage->GetAttributeAtPath(SdfPath(name)).Get(&initial));
        CHECK(initial.size() == 3);
        _CheckStageArraySamples("extent malformed widths", stage, _FindRig(stage),
            name, 1001.0, VtValue(initial), VtValue(VtFloatArray{1.2f}),
            VtValue(VtFloatArray{0.4f, 0.8f}), VtValue(VtFloatArray()));
    }
    // Delta mush's topology samples: a reordering of the same count is
    // followed with exact live parity; a short sample is another count of
    // topology, which the reader refuses with its reason.
    if (const auto stage = _Open(fixtures + "/computed_path_reads.usda")) {
        const std::string name = "/PathReadAsset/Geom/Ball.faceVertexIndices";
        VtIntArray initial;
        CHECK(stage->GetAttributeAtPath(SdfPath(name)).Get(&initial));
        CHECK(initial.size() >= 6);
        VtIntArray changed = initial;
        std::reverse(changed.begin(), changed.begin() + 3);
        _CheckStageArraySamples("smooth short topology", stage, _FindRig(stage),
            name, 1.0, VtValue(initial), VtValue(changed),
            VtValue(VtIntArray(initial.begin(), initial.begin() + 4)),
            VtValue(), /*topology=*/true);
    }
    if (const UsdStageRefPtr stage =
            _Open(fixtures + "/oneloop_two_limbs.usda")) {
        const std::string name =
            "/LimbsAsset/Rig/Movers/MeshASkin.rigExec:jointWeights";
        VtFloatArray initial;
        CHECK(stage->GetAttributeAtPath(SdfPath(name)).Get(&initial));
        {
            UsdEditContext context(stage, stage->GetSessionLayer());
            CHECK(stage->GetAttributeAtPath(SdfPath(name))
                      .Set(initial, UsdTimeCode(5)));
        }
        RigExecRigEvaluator evaluator(stage, _FindRig(stage));
        std::vector<std::string> notices;
        CHECK(evaluator.Compile(&notices));
        std::vector<std::string> reasons;
        CHECK(evaluator.IsBakeable(&reasons));
        CHECK(reasons.empty());
        VtFloatArray changed = initial;
        std::reverse(changed.begin(), changed.end());
        _CheckStageArraySamples("animated skin layout admitted", stage,
            _FindRig(stage), name, 5.0, VtValue(initial), VtValue(changed),
            VtValue(VtFloatArray{}));
    }

    std::printf("array inputs: %d of %d case(s) == session edit\n",
                arrayMatched, arrayCases);
    CHECK(arrayMatched == arrayCases && arrayCases == 18);
    _TestArraysAgainstOverrides(fixtures);
    std::printf("array inputs: %d of %d run(s) == live baked's interactive "
                "overrides, counters included\n",
                overrideMatched, overrideRuns);
    CHECK(overrideMatched == overrideRuns && overrideRuns == 9);
    _TestUnansweredPhaseArray(examples);
    _TestRetainedPointFinalAvailability(examples);
    _TestSparseSlotLeaves(examples);
    _TestSparseSourceMemos(examples);
    // Written slots and static provider inputs, over one bake.
    if (const UsdStageRefPtr stage =
            _Open(examples + "/biped/Biped_anim.usda")) {
        std::vector<uint8_t> bytes;
        if (_BakeArrays("static inputs", stage, _FindRig(stage), 1.0,
                        &bytes)) {
            _TestARepeatedSetWritesNothing(bytes);
            _TestStaticProviderInputsAreReadOnce(stage, bytes);
        }
    }
    _TestAPrunedBakeKeepsABlockedAnimatedAvar(examples);
    if (failures == 0) {
        std::printf("testRigExecRuntimeInputs: all tests passed\n");
        return 0;
    }
    std::printf("testRigExecRuntimeInputs: %d failures\n", failures);
    return 1;
}
