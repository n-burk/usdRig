// rigExecRuntime weight-family parity (M2): baked program vs runtime over
// every baking fixture, comparing weight packets bit for bit, plus
// synthetic builder checks against the real baked builders.
// Fixture strategy per fixture: bake in-process at the first frame, then
// run a fresh evaluator (Evaluate) and a fresh reader played through its
// inputs (the input sampler hands it the stage's animated inputs) frame by
// frame in order, comparing every packet, the weight frames and the
// diagnostics. A `static` row plays the bake time alone. Volume placements:
// per-volume steps and the retired whole-map form on the synthetic file, a
// skipped step's kept placement white-box, and drags of one volume's control
// on tests/fixtures/volume_placements.usda against live baked.
#include "rigExecBake/bake.h"
#include "rigExec/bakedProgram.h"
#include "rigExec/bakedProgramImpl.h"
#include "rigExec/rigEvaluator.h"
#include "rigExec/types.h"
#include "rigExec/weightPackets.h"
#include "rigExecBinary/format.h"
#include "rigExecMath/pointFrame.h"
#include "rigExecMath/weightFields.h"
#include "rigExecRuntime/runtime.h"
#include "rigExecRuntime/store.h"
#include "rigExecExampleFixtures.h"

#include "pxr/base/plug/registry.h"
#include "pxr/base/tf/getenv.h"
#include "pxr/usd/usd/primRange.h"
#include "pxr/usd/usd/stage.h"

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdio>
#include <cstring>
#include <map>
#include <memory>
#include <string>
#include <tuple>
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

static std::vector<double>
_ParseFrames(const std::string &text)
{
    std::vector<double> frames;
    size_t begin = 0;
    while (begin <= text.size()) {
        const size_t end = text.find(",", begin);
        const std::string piece =
            text.substr(begin, end == std::string::npos
                        ? std::string::npos : end - begin);
        if (!piece.empty()) {
            frames.push_back(std::stod(piece));
        }
        if (end == std::string::npos) {
            break;
        }
        begin = end + 1;
    }
    return frames;
}

// The file's text of path id \p id (a token's text for a token id).
static std::string
_WireString(const fb::RigExecWireFile &file, uint32_t id)
{
    return RigExecFormatPathText(file, id);
}

// The step-backed weight objects of \p file: the leading entries the
// WeightPacket steps build, before the envelope-only ones.
static size_t
_StepBackedObjects(const fb::RigExecWireFile &file)
{
    const std::vector<fb::RigExecWireWeightObject> &objects =
        file.geometry->weightObjects;
    size_t count = 0;
    while (count < objects.size() && !objects[count].envelopeOnly) {
        ++count;
    }
    return count;
}

static bool
_ComparePacket(const fb::RigExecWireFile &file,
               const fb::RigExecWireWeightObject &wire,
               const RigExecWeightPacket &baked,
               const RrWeightPacket &actual, double frame)
{
    const std::string path = _WireString(file, wire.path);
    bool ok = true;
    const auto note = [&](const char *field) {
        std::printf("  packet %s @ %g: %s differs\n", path.c_str(),
                    frame, field);
        ok = false;
    };
    if (_WireString(file, actual.representation) !=
        baked.representation.GetString()) {
        note("representation");
    }
    if (_WireString(file, actual.rangePolicy) !=
        baked.rangePolicy.GetString()) {
        note("rangePolicy");
    }
    if (actual.values != baked.values) {
        note("values");
    }
    if (actual.indices.size() != baked.indices.size()) {
        note("indices");
    } else {
        for (size_t i = 0; i < baked.indices.size(); ++i) {
            if (actual.indices[i] != int32_t(baked.indices[i])) {
                note("indices");
                break;
            }
        }
    }
    if (actual.defaultWeight != baked.defaultWeight) {
        note("defaultWeight");
    }
    if (actual.valid != baked.valid) {
        note("valid");
    }
    return ok;
}

static bool
_CompareWeightFrames(const std::vector<RigExecRuntimeWeightFrame> &actual,
                     const std::map<SdfPath, GfMatrix4d> &baked,
                     double frame)
{
    bool ok = true;
    if (actual.size() != baked.size()) {
        std::printf("  weightFrames @ %g: %zu entries vs %zu baked\n",
                    frame, actual.size(), baked.size());
        return false;
    }
    size_t i = 0;
    for (const auto &entry : baked) {
        const RigExecRuntimeWeightFrame &got = actual[i++];
        if (got.path != entry.first.GetString()) {
            std::printf("  weightFrames @ %g: path %s vs %s\n", frame,
                        got.path.c_str(),
                        entry.first.GetString().c_str());
            ok = false;
            continue;
        }
        for (size_t r = 0; r < 4; ++r) {
            for (size_t c = 0; c < 4; ++c) {
                if (got.matrix[r][c] != entry.second[r][c]) {
                    std::printf("  weightFrames %s @ %g: "
                                "matrix differs\n",
                                got.path.c_str(), frame);
                    ok = false;
                    r = c = 4;
                }
            }
        }
    }
    return ok;
}

static bool
_CompareDiagnostics(const std::vector<std::string> &actual,
                    const std::vector<std::string> &baked, double frame)
{
    if (actual == baked) {
        return true;
    }
    std::printf("  diagnostics @ %g: %zu lines vs %zu baked\n", frame,
                actual.size(), baked.size());
    const size_t common = std::min(actual.size(), baked.size());
    for (size_t i = 0; i < common; ++i) {
        if (actual[i] != baked[i]) {
            std::printf("    line %zu:\n      runtime: %s\n      baked:   "
                        "%s\n",
                        i, actual[i].c_str(), baked[i].c_str());
        }
    }
    for (size_t i = common; i < actual.size(); ++i) {
        std::printf("    runtime only %zu: %s\n", i, actual[i].c_str());
    }
    for (size_t i = common; i < baked.size(); ++i) {
        std::printf("    baked only %zu: %s\n", i, baked[i].c_str());
    }
    return false;
}

// Plays one fixture's bake through its inputs with a fresh evaluator and
// reader, \p frames in order: every weight packet, the weight frames and
// the diagnostics, bit for bit and verbatim.
static bool
_RunFixture(const UsdStageRefPtr &stage, const std::vector<double> &frames,
            const std::vector<uint8_t> &bytes,
            const fb::RigExecWireFile &file, size_t *compared)
{
    const size_t objects = _StepBackedObjects(file);
    RigExecRigEvaluator evaluator(stage, _FindRig(stage));
    evaluator.SetEvaluationMode(RigExecEvaluationMode::Baked);
    std::string error;
    RigExecTestPlayer runtime;
    if (!runtime.Open(bytes, stage, &error)) {
        std::printf("  open diagnostic: %s\n", error.c_str());
        return false;
    }
    bool ok = true;
    for (double frame : frames) {
        const RigExecRigPose pose = evaluator.Evaluate(
            UsdTimeCode(frame));
        const RigExecBakedProgram *program =
            evaluator.GetBakedProgram();
        if (!program) {
            std::printf("  no baked program @ %g\n", frame);
            return false;
        }
        if (!runtime.Play(frame, &error)) {
            std::printf("  Execute @ %g: %s\n", frame, error.c_str());
            return false;
        }
        const std::vector<RigExecWeightPacket> &bakedPackets =
            program->GetStepGraph().weightPackets;
        const std::vector<RrWeightPacket> &actualPackets =
            runtime->GetWeightPackets();
        if (bakedPackets.size() != objects ||
            actualPackets.size() != objects) {
            std::printf("  packet storage @ %g: baked %zu runtime %zu "
                        "file %zu\n",
                        frame, bakedPackets.size(),
                        actualPackets.size(), objects);
            return false;
        }
        for (size_t i = 0; i < objects; ++i) {
            ++(*compared);
            if (!_ComparePacket(file, file.geometry->weightObjects[i],
                                bakedPackets[i], actualPackets[i],
                                frame)) {
                ok = false;
            }
        }
        if (!_CompareWeightFrames(runtime->GetWeightFrames(),
                                  pose.weightFrames, frame)) {
            ok = false;
        }
        if (!_CompareDiagnostics(runtime->GetDiagnostics(),
                                 pose.diagnostics, frame)) {
            ok = false;
        }
    }
    return ok;
}

static void
_TestFixture(const std::string &name, const std::string &stagePath,
             const std::vector<double> &frames, bool staticClass)
{
    const UsdStageRefPtr stage = UsdStage::Open(stagePath);
    CHECK(stage);
    if (!stage) {
        return;
    }
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
        CHECK(RigExecTestBakeAt(evaluator, frames.front(), &bytes, &error));
        if (!error.empty()) {
            std::printf("bake diagnostic: %s\n", error.c_str());
        }
    }
    CHECK(!bytes.empty());
    if (bytes.empty()) {
        return;
    }
    const std::unique_ptr<fb::RigExecWireFile> file = RigExecTestUnpack(bytes);
    if (!file) {
        return;
    }
    // A `static` row holds an animated source in static data at the bake
    // time: it plays that time alone.
    const std::vector<double> times =
        staticClass ? std::vector<double>{frames.front()} : frames;
    size_t compared = 0;
    if (_RunFixture(stage, times, bytes, *file, &compared)) {
        std::printf("%s: %zu frame(s) of a bake at %g%s, packets=%zu "
                    "weightFrames=yes diagnostics=yes OK\n",
                    name.c_str(), times.size(), frames.front(),
                    staticClass ? " (static)" : "", compared);
        return;
    }
    ++failures;
    std::printf("%s: FAILED\n", name.c_str());
}

// Synthetic builder checks: a hand-built file exercising every builder arm
// against the real baked builders, including the path-read and chain-base
// gather layers no fixture populates for weight attributes, and the
// invalid-packet shapes.

// The path table and value pool of a hand-built file: prims and properties
// as tree nodes, parents first, tokens as Token nodes, floats pooled by
// their bits; ids in insertion order.
class _Synth {
public:
    explicit _Synth(fb::RigExecWireFile *file) : _file(file)
    {
        file->names = {std::string()};
        file->paths = {fb::PathNode(0, 0, fb::PathKind::None)};
        file->values.clear();
        file->values.emplace_back();
        file->intArrays.emplace_back();
        file->floatArrays.emplace_back();
        file->doubleArrays.emplace_back();
        file->vec2fArrays.emplace_back();
        file->vec3fArrays.emplace_back();
    }

    // "/A/B" as a prim node, "/A/B.c" as a property node of "/A/B".
    uint32_t Path(const std::string &text)
    {
        const size_t slash = text.rfind('/');
        const size_t dot = text.find('.', slash);
        const std::string prim =
            dot == std::string::npos ? text : text.substr(0, dot);
        uint32_t at = 0;
        size_t begin = 1;
        while (begin <= prim.size()) {
            size_t end = prim.find('/', begin);
            if (end == std::string::npos) {
                end = prim.size();
            }
            at = _Node(at, prim.substr(begin, end - begin),
                       fb::PathKind::Prim);
            begin = end + 1;
        }
        if (dot != std::string::npos) {
            at = _Node(at, text.substr(dot + 1), fb::PathKind::Property);
        }
        return at;
    }

    uint32_t Token(const std::string &text)
    {
        return _Node(0, text, fb::PathKind::Token);
    }

    uint32_t Float(float f)
    {
        uint32_t bits = 0;
        std::memcpy(&bits, &f, sizeof(bits));
        const auto found =
            _floats.emplace(bits, uint32_t(_file->values.size()));
        if (found.second) {
            fb::RigExecWireValue value;
            value.tag = fb::InputTag::Float;
            value.bits = bits;
            _file->values.push_back(std::move(value));
        }
        return found.first->second;
    }

    uint32_t Value(fb::RigExecWireValue value)
    {
        _file->values.push_back(std::move(value));
        return uint32_t(_file->values.size() - 1);
    }

    uint32_t Points(const std::vector<GfVec3f> &points)
    {
        fb::RigExecWireVec3fArray array;
        for (const GfVec3f &p : points) {
            array.v.push_back(RigExecWireVec3f{p[0], p[1], p[2]});
        }
        _file->vec3fArrays.push_back(std::move(array));
        return uint32_t(_file->vec3fArrays.size() - 1);
    }

private:
    uint32_t _Node(uint32_t parent, const std::string &name,
                   fb::PathKind kind)
    {
        const auto named =
            _names.emplace(name, uint32_t(_file->names.size()));
        if (named.second) {
            _file->names.push_back(name);
        }
        const auto found = _nodes.emplace(
            std::make_tuple(parent, named.first->second, uint8_t(kind)),
            uint32_t(_file->paths.size()));
        if (found.second) {
            _file->paths.emplace_back(parent, named.first->second, kind);
        }
        return found.first->second;
    }

    fb::RigExecWireFile *_file;
    std::map<std::string, uint32_t> _names;
    std::map<std::tuple<uint32_t, uint32_t, uint8_t>, uint32_t> _nodes;
    std::map<uint32_t, uint32_t> _floats;
};

// The painted arrays of weight object \p object of \p file, as the
// export lists them: its rigExec:values (float[]) and, with \p indices,
// its rigExec:indices (int[]), each a listed input at its place in the path
// order whose default is a pool entry holding the array. Call it once the
// file's other names and paths are interned.
static void
_PaintInputs(fb::RigExecWireFile *file, size_t object,
             const std::vector<float> &values,
             const std::vector<int32_t> *indices)
{
    const std::string path = RigExecFormatPathText(
        *file, file->geometry->weightObjects[object].path);
    const uint8_t has = uint8_t(fb::InputSlotFlags::HasValue);
    fb::RigExecWireFloatArray floats;
    floats.v = values;
    file->floatArrays.push_back(std::move(floats));
    const uint32_t v = RigExecTestAddListedSlot(
        file, path + ".rigExec:values", fb::InputTag::FloatArray,
        RigExecTestAddValue(
            file, RigExecTestArrayValue(
                      fb::InputTag::FloatArray,
                      uint32_t(file->floatArrays.size() - 1))),
        has);
    file->geometry->weightObjects[object].valuesSlot = int32_t(v);
    if (!indices) {
        return;
    }
    fb::RigExecWireIntArray ints;
    ints.v = *indices;
    file->intArrays.push_back(std::move(ints));
    const uint32_t i = RigExecTestAddListedSlot(
        file, path + ".rigExec:indices", fb::InputTag::IntArray,
        RigExecTestAddValue(
            file, RigExecTestArrayValue(fb::InputTag::IntArray,
                                        uint32_t(file->intArrays.size() - 1))),
        has);
    file->geometry->weightObjects[object].indicesSlot = int32_t(i);
}

static std::unique_ptr<fb::RigExecWireInput>
_SynthOwn(const fb::RigExecWireInput &input)
{
    return std::make_unique<fb::RigExecWireInput>(input);
}

// A Baked Float read of the table constant \p value.
static std::unique_ptr<fb::RigExecWireInput>
_SynthFloat(_Synth *synth, float value)
{
    fb::RigExecWireInput input;
    input.tag = fb::InputTag::Float;
    input.mode = fb::ReadMode::Baked;
    input.constant = synth->Float(value);
    return _SynthOwn(input);
}

// A Baked Float read made per run through a listed, Animated slot of its
// own ("/Synth/<k>.value") whose default is \p slotDefault; \p constant is
// the read's table constant.
static std::unique_ptr<fb::RigExecWireInput>
_SynthVaryingFloat(_Synth *synth, fb::RigExecWireFile *file, float constant,
                   float slotDefault)
{
    const uint32_t slot = uint32_t(file->inputs.size());
    file->inputs.emplace_back(
        synth->Path("/Synth/" + std::to_string(slot) + ".value"),
        synth->Float(slotDefault), -1, -1, fb::InputTag::Float,
        uint8_t(fb::InputSlotFlags::Listed) |
            uint8_t(fb::InputSlotFlags::Animated) |
            uint8_t(fb::InputSlotFlags::HasValue));
    file->listedInputs = uint32_t(file->inputs.size());
    fb::RigExecWireInput input;
    input.tag = fb::InputTag::Float;
    input.mode = fb::ReadMode::Baked;
    input.flags = uint8_t(fb::InputReadFlags::Varying);
    input.constant = synth->Float(constant);
    input.walk = {slot};
    input.selected = 0;
    return _SynthOwn(input);
}

static fb::RigExecWireWeightObject
_SynthObject(_Synth *synth, const char *path, const char *type,
             const char *representation, const char *rangePolicy)
{
    fb::RigExecWireWeightObject object;
    object.path = synth->Path(path);
    object.type = synth->Token(type);
    object.representation = synth->Token(representation);
    object.rangePolicy = synth->Token(rangePolicy);
    object.defaultWeight = _SynthFloat(synth, 0.0f);
    object.driver = _SynthFloat(synth, 1.0f);
    object.scale = _SynthFloat(synth, 1.0f);
    object.bias = _SynthFloat(synth, 0.0f);
    object.strength = _SynthFloat(synth, 1.0f);
    object.invert = _SynthFloat(synth, 0.0f);
    object.falloffMin = _SynthFloat(synth, 0.0f);
    object.falloffMax = _SynthFloat(synth, 1.0f);
    for (std::unique_ptr<fb::RigExecWireInput> *scale :
         {&object.scaleXPos, &object.scaleYPos, &object.scaleZPos,
          &object.scaleXNeg, &object.scaleYNeg, &object.scaleZNeg}) {
        *scale = _SynthFloat(synth, 1.0f);
    }
    object.scaleX = _SynthFloat(synth, 1.0f);
    object.scaleY = _SynthFloat(synth, 1.0f);
    object.scaleZ = _SynthFloat(synth, 1.0f);
    object.extentU = _SynthFloat(synth, 1.0f);
    object.extentV = _SynthFloat(synth, 1.0f);
    return object;
}

static void
_SynthAttr(_Synth *synth, const char *path, std::vector<uint32_t> *paths,
           std::vector<uint8_t> *valid)
{
    paths->push_back(synth->Path(path));
    valid->push_back(1);
}

// A static path read of \p points at \p path, at the evaluation time.
static fb::RigExecWirePathRead
_SynthPointsRead(_Synth *synth, const char *path,
                 const std::vector<GfVec3f> &points)
{
    fb::RigExecWirePathRead read;
    read.path = synth->Path(path);
    read.value = std::make_unique<fb::RigExecWirePathValue>();
    read.value->tag = fb::PathTag::Vec3fArray;
    read.value->array = synth->Points(points);
    return read;
}

static bool
_SynthCompare(const char *name, const RigExecWeightPacket &expected,
              const RrWeightPacket &actual, const fb::RigExecWireFile &file)
{
    bool ok = true;
    const auto note = [&](const char *field) {
        std::printf("  synthetic %s: %s differs\n", name, field);
        ok = false;
    };
    if (_WireString(file, actual.representation) !=
        expected.representation.GetString()) {
        note("representation");
    }
    if (_WireString(file, actual.rangePolicy) !=
        expected.rangePolicy.GetString()) {
        note("rangePolicy");
    }
    if (actual.values != expected.values) {
        note("values");
        for (size_t i = 0;
             i < std::min(actual.values.size(),
                          expected.values.size());
             ++i) {
            if (actual.values[i] != expected.values[i]) {
                std::printf("    value %zu: %g vs %g\n", i,
                            double(actual.values[i]),
                            double(expected.values[i]));
                break;
            }
        }
        std::printf("    sizes %zu vs %zu\n", actual.values.size(),
                    expected.values.size());
    }
    if (actual.indices.size() != expected.indices.size()) {
        note("indices");
    } else {
        for (size_t i = 0; i < expected.indices.size(); ++i) {
            if (actual.indices[i] != int32_t(expected.indices[i])) {
                note("indices");
                break;
            }
        }
    }
    if (actual.defaultWeight != expected.defaultWeight) {
        note("defaultWeight");
    }
    if (actual.valid != expected.valid) {
        note("valid");
    }
    return ok;
}

static void
_TestSyntheticBuilders()
{
    fb::RigExecWireFile file;
    _Synth synth(&file);
    _Synth *const writer = &synth;
    file.formatVersion = RigExecFormatVersion;
    file.bakeTime = 1.0;
    file.slotMeta = std::make_unique<fb::RigExecWireSlotMeta>();
    file.constants = std::make_unique<fb::RigExecWireConstants>();
    file.pose = std::make_unique<fb::RigExecWireDomainPose>();
    file.geometry = std::make_unique<fb::RigExecWireDomainGeometry>();
    fb::RigExecWireSlotMeta &slotMeta = *file.slotMeta;
    fb::RigExecWireConstants &constants = *file.constants;
    fb::RigExecWireDomainGeometry &geometry = *file.geometry;

    // Two volume slots: the first carries the volumes, the second only its
    // own placement. The pools hold default frames, so placements are
    // exactly the identity-landmark map.
    file.rig = synth.Path("/Volume");
    const char *const volumeSlots[] = {"/Volume", "/Volume2"};
    const size_t slotCount = 2;
    for (const char *path : volumeSlots) {
        slotMeta.paths.push_back(synth.Path(path));
    }
    slotMeta.slotKind.assign(slotCount, fb::SlotKind::FirstFramePose);
    slotMeta.parent.assign(slotCount, -1);
    slotMeta.propParent.assign(slotCount, -1);
    slotMeta.needFinal.assign(slotCount, 0);
    slotMeta.needBase.assign(slotCount, 0);
    constants.avarConstants.assign(11 * slotCount, 0.0);
    RigExecWireMatrix4d identity{};
    identity[0] = identity[5] = identity[10] = identity[15] = 1.0;
    for (size_t slot = 0; slot < slotCount; ++slot) {
        constants.noScaleAvars.push_back(1);
        constants.rotationSign.push_back(0);
        constants.restM.push_back(identity);
        constants.selfD.push_back(identity);
        constants.parentDinv.push_back(identity);
        constants.restRoundTrip.push_back(identity);
        constants.defaultRoundTrip.push_back(identity);
        constants.posedAuthoredM.push_back(identity);
        std::array<RigExecWireVec3d, 4> landmarks{};
        landmarks[1] = {1.0, 0.0, 0.0};
        landmarks[2] = {0.0, 1.0, 0.0};
        landmarks[3] = {0.0, 0.0, 1.0};
        constants.restPts.push_back(landmarks);
        RigExecWireFrame restFrame;
        restFrame.points = landmarks;
        restFrame.flags = 1;
        constants.restFrames.push_back(restFrame);
        constants.rotOrder.push_back(0);
        constants.posedAuthored.push_back(0);
    }
    // Each slot's ladder: every field a fixed read of its constant.
    {
        fb::RigExecWireValue matrix;
        matrix.tag = fb::InputTag::Matrix4d;
        matrix.matrix = std::make_unique<RigExecWireMatrix4d>(identity);
        const uint32_t identityValue = synth.Value(std::move(matrix));
        fb::RigExecWireValue token;
        token.tag = fb::InputTag::Token;
        const uint32_t tokenValue = synth.Value(std::move(token));
        const auto fixed = [](fb::InputTag tag, uint32_t constant) {
            fb::RigExecWireInput input;
            input.tag = tag;
            input.mode = fb::ReadMode::Baked;
            input.constant = constant;
            return input;
        };
        for (size_t slot = 0; slot < slotCount; ++slot) {
            fb::RigExecWireLadder ladder;
            ladder.restSpace =
                _SynthOwn(fixed(fb::InputTag::Matrix4d, identityValue));
            ladder.defaultSpace =
                _SynthOwn(fixed(fb::InputTag::Matrix4d, identityValue));
            ladder.posedSpace =
                _SynthOwn(fixed(fb::InputTag::Matrix4d, identityValue));
            ladder.rotationOrder =
                _SynthOwn(fixed(fb::InputTag::Token, tokenValue));
            for (int k = 0; k < 6; ++k) {
                ladder.restAvars.push_back(fixed(fb::InputTag::Double, 0));
                ladder.defaultAvars.push_back(
                    fixed(fb::InputTag::Double, 0));
            }
            file.pose->ladders.push_back(std::move(ladder));
        }
    }

    std::vector<RigExecWeightPacket> expected;
    // The painted arrays, listed as inputs once every object is in.
    struct _Painted {
        size_t object;
        std::vector<float> values;
        std::vector<int32_t> indices;
        bool sparse;
    };
    std::vector<_Painted> painted;

    // 0: static dense, valid.
    {
        fb::RigExecWireWeightObject object = _SynthObject(
            writer, "/w/dense", "RigExecStaticWeight", "dense",
            "strict");
        painted.push_back(
            {geometry.weightObjects.size(), {0.25f, 0.5f, 0.75f}, {}, false});
        geometry.weightObjects.push_back(std::move(object));
        RigExecStaticWeightInputs in;
        in.representation = TfToken("dense");
        in.rangePolicy = TfToken("strict");
        in.values = {0.25f, 0.5f, 0.75f};
        in.defaultWeight = 0.0f;
        expected.push_back(RigExecBuildStaticWeightPacket(in));
    }
    // 1: static sparse, unsorted pairs canonicalize.
    {
        fb::RigExecWireWeightObject object = _SynthObject(
            writer, "/w/sparse", "RigExecStaticWeight", "sparse",
            "strict");
        painted.push_back(
            {geometry.weightObjects.size(), {0.9f, 0.1f}, {3, 1}, true});
        object.defaultWeight = _SynthFloat(writer, 0.5f);
        geometry.weightObjects.push_back(std::move(object));
        RigExecStaticWeightInputs in;
        in.representation = TfToken("sparse");
        in.rangePolicy = TfToken("strict");
        in.values = {0.9f, 0.1f};
        in.indices = {3, 1};
        in.defaultWeight = 0.5f;
        expected.push_back(RigExecBuildStaticWeightPacket(in));
    }
    // 2: static constant over a VARYING default (its slot reads 0.3).
    {
        fb::RigExecWireWeightObject object = _SynthObject(
            writer, "/w/varying", "RigExecStaticWeight", "constant",
            "strict");
        object.defaultWeight =
            _SynthVaryingFloat(writer, &file, 0.7f, 0.3f);
        geometry.weightObjects.push_back(std::move(object));
        RigExecStaticWeightInputs in;
        in.representation = TfToken("constant");
        in.rangePolicy = TfToken("strict");
        in.defaultWeight = 0.3f;
        expected.push_back(RigExecBuildStaticWeightPacket(in));
    }
    // 3: static dense with a nonzero default: invalid, tokens kept.
    {
        fb::RigExecWireWeightObject object = _SynthObject(
            writer, "/w/denseBadDefault", "RigExecStaticWeight",
            "dense", "strict");
        painted.push_back(
            {geometry.weightObjects.size(), {0.5f}, {}, false});
        object.defaultWeight = _SynthFloat(writer, 0.5f);
        geometry.weightObjects.push_back(std::move(object));
        RigExecStaticWeightInputs in;
        in.representation = TfToken("dense");
        in.rangePolicy = TfToken("strict");
        in.values = {0.5f};
        in.defaultWeight = 0.5f;
        expected.push_back(RigExecBuildStaticWeightPacket(in));
    }
    // 4: static sparse with duplicate indices: invalid, tokens kept.
    {
        fb::RigExecWireWeightObject object = _SynthObject(
            writer, "/w/sparseDup", "RigExecStaticWeight", "sparse",
            "strict");
        painted.push_back(
            {geometry.weightObjects.size(), {0.5f, 0.5f}, {2, 2}, true});
        geometry.weightObjects.push_back(std::move(object));
        RigExecStaticWeightInputs in;
        in.representation = TfToken("sparse");
        in.rangePolicy = TfToken("strict");
        in.values = {0.5f, 0.5f};
        in.indices = {2, 2};
        expected.push_back(RigExecBuildStaticWeightPacket(in));
    }
    // 5: static with an unknown representation: invalid, tokens kept.
    {
        fb::RigExecWireWeightObject object = _SynthObject(
            writer, "/w/unknownRep", "RigExecStaticWeight",
            "exponential", "strict");
        painted.push_back(
            {geometry.weightObjects.size(), {0.5f}, {}, false});
        geometry.weightObjects.push_back(std::move(object));
        RigExecStaticWeightInputs in;
        in.representation = TfToken("exponential");
        in.rangePolicy = TfToken("strict");
        in.values = {0.5f};
        expected.push_back(RigExecBuildStaticWeightPacket(in));
    }
    // 6: dynamic over no base, clamp policy: 1.1 clamps to 1.
    {
        fb::RigExecWireWeightObject object = _SynthObject(
            writer, "/w/dynFree", "RigExecDynamicWeight", "constant",
            "clamp");
        object.driver = _SynthFloat(writer, 0.5f);
        object.scale = _SynthFloat(writer, 2.0f);
        object.bias = _SynthFloat(writer, 0.1f);
        geometry.weightObjects.push_back(std::move(object));
        RigExecDynamicWeightInputs in;
        in.representation = TfToken("constant");
        in.rangePolicy = TfToken("clamp");
        in.driver = 0.5f;
        in.scale = 2.0f;
        in.bias = 0.1f;
        expected.push_back(
            RigExecBuildDynamicWeightPacket(in, nullptr));
    }
    // 7: dynamic remapping object 0's dense field.
    {
        fb::RigExecWireWeightObject object = _SynthObject(
            writer, "/w/dynDense", "RigExecDynamicWeight", "dense",
            "strict");
        object.base = 0;
        object.scale = _SynthFloat(writer, 0.5f);
        geometry.weightObjects.push_back(std::move(object));
        RigExecDynamicWeightInputs in;
        in.representation = TfToken("dense");
        in.rangePolicy = TfToken("strict");
        in.scale = 0.5f;
        expected.push_back(
            RigExecBuildDynamicWeightPacket(in, &expected[0]));
    }
    // 8: dynamic over object 0 with a mismatched representation.
    {
        fb::RigExecWireWeightObject object = _SynthObject(
            writer, "/w/dynMismatch", "RigExecDynamicWeight",
            "sparse", "strict");
        object.base = 0;
        geometry.weightObjects.push_back(std::move(object));
        RigExecDynamicWeightInputs in;
        in.representation = TfToken("sparse");
        in.rangePolicy = TfToken("strict");
        expected.push_back(
            RigExecBuildDynamicWeightPacket(in, &expected[0]));
    }
    // 9: combine over two dense fields (subtract, authored order).
    {
        fb::RigExecWireWeightObject object = _SynthObject(
            writer, "/w/combine", "RigExecCombineWeight", "dense",
            "strict");
        object.combineMode = writer->Token("subtract");
        object.inputs = {0, 7};
        geometry.weightObjects.push_back(std::move(object));
        expected.push_back(RigExecBuildCombineWeightPacket(
            TfToken("dense"), TfToken("strict"),
            TfToken("subtract"),
            {expected[0], expected[7]}, 0, 1.0f, 0.0f));
    }
    // 10: combine over two constants, sized by its weight target.
    {
        fb::RigExecWireWeightObject object = _SynthObject(
            writer, "/w/combineConst", "RigExecCombineWeight",
            "dense", "strict");
        object.combineMode = writer->Token("add");
        object.inputs = {2, 6};
        object.strength = _SynthFloat(writer, 0.5f);
        _SynthAttr(writer, "/mesh.points",
                   &object.combineTargetPoints,
                   &object.combineTargetValid);
        geometry.weightObjects.push_back(std::move(object));
        expected.push_back(RigExecBuildCombineWeightPacket(
            TfToken("dense"), TfToken("strict"), TfToken("add"),
            {expected[2], expected[6]}, 3, 0.5f, 0.0f));
    }
    // Shared volume points and placements for objects 11-16.
    const std::vector<GfVec3f> meshPoints = {
        GfVec3f(0.0f), GfVec3f(1.0f, 0.0f, 0.0f),
        GfVec3f(0.0f, 2.0f, 0.0f)};
    const std::vector<GfVec3f> samplePoints = {
        GfVec3f(0.0f), GfVec3f(0.0f, 1.0f, 0.0f)};
    const std::vector<GfVec3f> curvePoints = {
        GfVec3f(0.0f, 0.0f, 1.0f), GfVec3f(0.0f, 0.0f, 2.0f)};
    const RigExecPointFrame placement;
    const auto volumeInputs = [&](const char *planeAxis,
                                  const char *planeBounds) {
        RigExecVolumeWeightInputs in;
        in.representation = TfToken("dense");
        in.rangePolicy = TfToken("clamp");
        in.placement = placement;
        in.hasPlacement = true;
        in.params.falloffMin = 0.0f;
        in.params.falloffMax = 2.0f;
        in.targetPoints = meshPoints;
        in.scales = GfVec3f(1.0f);
        in.planeAxis = TfToken(planeAxis);
        in.planeBounds = TfToken(planeBounds);
        return in;
    };
    const std::vector<GfVec3f> signedSamples = {GfVec3f(1, 0, 0), GfVec3f(0, 1, 0), GfVec3f(0, 0, -2)};
    // 11: sphere over pathReads points, smooth remap LUT, and varying
    // signed/legacy scales whose uid order must match capture.
    {
        fb::RigExecWireWeightObject object = _SynthObject(
            writer, "/w/sphere", "RigExecSphereWeight", "dense",
            "clamp");
        object.providerSlot = 0;
        object.falloffMax = _SynthFloat(writer, 2.0f);
        _SynthAttr(writer, "/signed.points", &object.samplePoints, &object.sampleValid);
        object.scaleXPos = _SynthVaryingFloat(writer, &file, 9.0f, 2.0f);
        object.scaleY = _SynthVaryingFloat(writer, &file, 9.0f, 0.5f);
        object.scaleYPos = _SynthFloat(writer, 3.0f);
        object.scaleZNeg = _SynthFloat(writer, 4.0f);
        object.falloffCurve =
            RigExecBuildFalloffLut(RigExecFalloffProfile::Smooth);
        _SynthAttr(writer, "/mesh.points", &object.targetPoints,
                   &object.targetValid);
        geometry.weightObjects.push_back(std::move(object));
        RigExecVolumeWeightInputs sphereIn =
            volumeInputs("y", "unbounded");
        sphereIn.samplePoints = signedSamples;
        sphereIn.scales = GfVec3f(1.0f, 0.5f, 1.0f);
        sphereIn.positiveScales = GfVec3f(2.0f, 3.0f, 1.0f);
        sphereIn.negativeScales = GfVec3f(1.0f, 1.0f, 4.0f);
        sphereIn.params.curve =
            RigExecBuildFalloffLut(RigExecFalloffProfile::Smooth);
        expected.push_back(RigExecBuildVolumeWeightPacket(
            TfToken("RigExecSphereWeight"), sphereIn));
    }
    // 12: bounded plane over the same points, axis x.
    {
        fb::RigExecWireWeightObject object = _SynthObject(
            writer, "/w/plane", "RigExecPlaneWeight", "dense",
            "clamp");
        object.providerSlot = 0;
        object.falloffMax = _SynthFloat(writer, 2.0f);
        object.planeAxis = writer->Token("x");
        object.planeBounds = writer->Token("bounded");
        _SynthAttr(writer, "/mesh.points", &object.targetPoints,
                   &object.targetValid);
        geometry.weightObjects.push_back(std::move(object));
        RigExecVolumeWeightInputs in =
            volumeInputs("x", "bounded");
        in.extentU = 1.0f;
        in.extentV = 1.0f;
        expected.push_back(RigExecBuildVolumeWeightPacket(
            TfToken("RigExecPlaneWeight"), in));
    }
    // 13: curve weight with pathReads curve points.
    {
        fb::RigExecWireWeightObject object = _SynthObject(
            writer, "/w/curve", "RigExecCurveWeight", "dense",
            "clamp");
        object.providerSlot = 0;
        object.falloffMax = _SynthFloat(writer, 2.0f);
        _SynthAttr(writer, "/mesh.points", &object.targetPoints,
                   &object.targetValid);
        _SynthAttr(writer, "/curve.points", &object.curvePoints,
                   &object.curveValid);
        geometry.weightObjects.push_back(std::move(object));
        RigExecVolumeWeightInputs in =
            volumeInputs("y", "unbounded");
        in.curvePoints = curvePoints;
        expected.push_back(RigExecBuildVolumeWeightPacket(
            TfToken("RigExecCurveWeight"), in));
    }
    // 14: sphere with a mistyped sample override: cardinality
    // mismatch is invalid, tokens kept.
    {
        fb::RigExecWireWeightObject object = _SynthObject(
            writer, "/w/sampleBad", "RigExecSphereWeight", "dense",
            "clamp");
        object.providerSlot = 0;
        object.falloffMax = _SynthFloat(writer, 2.0f);
        _SynthAttr(writer, "/mesh.points", &object.targetPoints,
                   &object.targetValid);
        _SynthAttr(writer, "/sample.points", &object.samplePoints,
                   &object.sampleValid);
        geometry.weightObjects.push_back(std::move(object));
        RigExecVolumeWeightInputs in =
            volumeInputs("y", "unbounded");
        in.samplePoints = samplePoints;
        expected.push_back(RigExecBuildVolumeWeightPacket(
            TfToken("RigExecSphereWeight"), in));
    }
    // 15: sphere over a CHAIN base (no pathReads entry).
    {
        fb::RigExecWireWeightObject object = _SynthObject(
            writer, "/w/chainSphere", "RigExecSphereWeight", "dense",
            "clamp");
        object.providerSlot = 0;
        object.falloffMax = _SynthFloat(writer, 2.0f);
        _SynthAttr(writer, "/chainmesh.points",
                   &object.targetPoints, &object.targetValid);
        geometry.weightObjects.push_back(std::move(object));
        RigExecVolumeWeightInputs in =
            volumeInputs("y", "unbounded");
        in.targetPoints = samplePoints;
        expected.push_back(RigExecBuildVolumeWeightPacket(
            TfToken("RigExecSphereWeight"), in));
    }
    // 16: sphere over an unreadable target: invalid, tokens kept.
    {
        fb::RigExecWireWeightObject object = _SynthObject(
            writer, "/w/missing", "RigExecSphereWeight", "dense",
            "clamp");
        object.providerSlot = 0;
        object.falloffMax = _SynthFloat(writer, 2.0f);
        _SynthAttr(writer, "/missing.points",
                   &object.targetPoints, &object.targetValid);
        geometry.weightObjects.push_back(std::move(object));
        // The baked gather reads no points, so the prologue passes
        // but the target check fails: tokens kept, invalid.
        RigExecVolumeWeightInputs missingIn =
            volumeInputs("y", "unbounded");
        missingIn.targetPoints.clear();
        expected.push_back(RigExecBuildVolumeWeightPacket(
            TfToken("RigExecSphereWeight"), missingIn));
    }
    // 17: unknown weight type: bare invalid packet.
    {
        fb::RigExecWireWeightObject object = _SynthObject(
            writer, "/w/magic", "RigExecMagicWeight", "dense",
            "strict");
        geometry.weightObjects.push_back(std::move(object));
        expected.push_back(RigExecWeightPacket());
    }
    // 18-22: the remaining combine modes over objects 0 and 7.
    for (const char *mode :
         {"multiply", "max", "min", "average", "overlay"}) {
        fb::RigExecWireWeightObject object = _SynthObject(
            writer,
            (std::string("/w/combine-") + mode).c_str(),
            "RigExecCombineWeight", "dense", "strict");
        object.combineMode = writer->Token(mode);
        object.inputs = {0, 7};
        geometry.weightObjects.push_back(std::move(object));
        expected.push_back(RigExecBuildCombineWeightPacket(
            TfToken("dense"), TfToken("strict"), TfToken(mode),
            {expected[0], expected[7]}, 0, 1.0f, 0.0f));
    }
    // 23: dynamic remapping object 1's sparse field (the default
    // remaps; dense would pin zero).
    {
        fb::RigExecWireWeightObject object = _SynthObject(
            writer, "/w/dynSparse", "RigExecDynamicWeight",
            "sparse", "strict");
        object.base = 1;
        object.driver = _SynthFloat(writer, 0.5f);
        geometry.weightObjects.push_back(std::move(object));
        RigExecDynamicWeightInputs in;
        in.representation = TfToken("sparse");
        in.rangePolicy = TfToken("strict");
        in.driver = 0.5f;
        expected.push_back(
            RigExecBuildDynamicWeightPacket(in, &expected[1]));
    }
    // 24: static constant with an out-of-range default: invalid.
    {
        fb::RigExecWireWeightObject object = _SynthObject(
            writer, "/w/constBad", "RigExecStaticWeight",
            "constant", "strict");
        object.defaultWeight = _SynthFloat(writer, 2.0f);
        geometry.weightObjects.push_back(std::move(object));
        RigExecStaticWeightInputs in;
        in.representation = TfToken("constant");
        in.rangePolicy = TfToken("strict");
        in.defaultWeight = 2.0f;
        expected.push_back(RigExecBuildStaticWeightPacket(in));
    }
    // 25: constant-only combine whose epilogue violates strict: the
    // BARE invalid packet, not the token-carrying one.
    {
        fb::RigExecWireWeightObject object = _SynthObject(
            writer, "/w/combineBare", "RigExecCombineWeight",
            "dense", "strict");
        object.combineMode = writer->Token("add");
        object.inputs = {2, 6};
        _SynthAttr(writer, "/mesh.points",
                   &object.combineTargetPoints,
                   &object.combineTargetValid);
        geometry.weightObjects.push_back(std::move(object));
        expected.push_back(RigExecBuildCombineWeightPacket(
            TfToken("dense"), TfToken("strict"), TfToken("add"),
            {expected[2], expected[6]}, 3, 1.0f, 0.0f));
    }
    // 26: sphere over a degenerate band: a hard step at distance 1.
    {
        fb::RigExecWireWeightObject object = _SynthObject(
            writer, "/w/degenerate", "RigExecSphereWeight",
            "dense", "clamp");
        object.providerSlot = 0;
        object.falloffMin = _SynthFloat(writer, 1.0f);
        object.falloffMax = _SynthFloat(writer, 1.0f);
        _SynthAttr(writer, "/mesh.points", &object.targetPoints,
                   &object.targetValid);
        geometry.weightObjects.push_back(std::move(object));
        RigExecVolumeWeightInputs in =
            volumeInputs("y", "unbounded");
        in.params.falloffMin = 1.0f;
        in.params.falloffMax = 1.0f;
        expected.push_back(RigExecBuildVolumeWeightPacket(
            TfToken("RigExecSphereWeight"), in));
    }
    // 27: unbounded plane with bad extents: valid, the extents are
    // not even read off the unbounded arm.
    {
        fb::RigExecWireWeightObject object = _SynthObject(
            writer, "/w/planeUnbounded", "RigExecPlaneWeight",
            "dense", "clamp");
        object.providerSlot = 0;
        object.falloffMax = _SynthFloat(writer, 2.0f);
        object.planeAxis = writer->Token("y");
        object.planeBounds = writer->Token("unbounded");
        object.extentU = _SynthFloat(writer, 0.0f);
        object.extentV = _SynthFloat(writer, -1.0f);
        _SynthAttr(writer, "/mesh.points", &object.targetPoints,
                   &object.targetValid);
        geometry.weightObjects.push_back(std::move(object));
        expected.push_back(RigExecBuildVolumeWeightPacket(
            TfToken("RigExecPlaneWeight"),
            volumeInputs("y", "unbounded")));
    }
    // 28: plane with an unknown axis: invalid, tokens kept.
    {
        fb::RigExecWireWeightObject object = _SynthObject(
            writer, "/w/planeBadAxis", "RigExecPlaneWeight",
            "dense", "clamp");
        object.providerSlot = 0;
        object.falloffMax = _SynthFloat(writer, 2.0f);
        object.planeAxis = writer->Token("w");
        object.planeBounds = writer->Token("unbounded");
        _SynthAttr(writer, "/mesh.points", &object.targetPoints,
                   &object.targetValid);
        geometry.weightObjects.push_back(std::move(object));
        expected.push_back(RigExecBuildVolumeWeightPacket(
            TfToken("RigExecPlaneWeight"),
            volumeInputs("w", "unbounded")));
    }
    // 29: strict sphere driven out of range by strength: the BARE
    // invalid packet.
    {
        fb::RigExecWireWeightObject object = _SynthObject(
            writer, "/w/strictBare", "RigExecSphereWeight",
            "dense", "strict");
        object.providerSlot = 0;
        object.falloffMax = _SynthFloat(writer, 2.0f);
        object.strength = _SynthFloat(writer, 10.0f);
        _SynthAttr(writer, "/mesh.points", &object.targetPoints,
                   &object.targetValid);
        geometry.weightObjects.push_back(std::move(object));
        RigExecVolumeWeightInputs in =
            volumeInputs("y", "unbounded");
        in.rangePolicy = TfToken("strict");
        in.params.strength = 10.0f;
        expected.push_back(RigExecBuildVolumeWeightPacket(
            TfToken("RigExecSphereWeight"), in));
    }
    // 30: sphere with a matching sample override: the samples, not
    // the targets, are measured (2 elements).
    {
        fb::RigExecWireWeightObject object = _SynthObject(
            writer, "/w/sampleOk", "RigExecSphereWeight", "dense",
            "clamp");
        object.providerSlot = 0;
        object.falloffMax = _SynthFloat(writer, 2.0f);
        _SynthAttr(writer, "/mesh2.points", &object.targetPoints,
                   &object.targetValid);
        _SynthAttr(writer, "/sample.points", &object.samplePoints,
                   &object.sampleValid);
        geometry.weightObjects.push_back(std::move(object));
        RigExecVolumeWeightInputs in =
            volumeInputs("y", "unbounded");
        in.targetPoints = samplePoints;
        in.samplePoints = samplePoints;
        expected.push_back(RigExecBuildVolumeWeightPacket(
            TfToken("RigExecSphereWeight"), in));
    }

    // One chain feeding object 15's target, its base held.
    {
        fb::RigExecWireChain chain;
        chain.target = synth.Path("/chainmesh.points");
        chain.haveBase = true;
        chain.base = synth.Points(samplePoints);
        geometry.chains.push_back(std::move(chain));
        geometry.chainRevisionBegin = {0};
        geometry.chainRevisionEnd = {0};
        geometry.chainChunkBegin = {0};
        geometry.chainChunkEnd = {0};
    }
    // The static points the gathers read, sorted by (path, rest).
    {
        std::vector<fb::RigExecWirePathRead> reads;
        reads.push_back(
            _SynthPointsRead(writer, "/signed.points", signedSamples));
        reads.push_back(_SynthPointsRead(writer, "/mesh.points", meshPoints));
        reads.push_back(
            _SynthPointsRead(writer, "/curve.points", curvePoints));
        reads.push_back(
            _SynthPointsRead(writer, "/sample.points", samplePoints));
        reads.push_back(
            _SynthPointsRead(writer, "/mesh2.points", samplePoints));
        std::sort(reads.begin(), reads.end(),
                  [](const fb::RigExecWirePathRead &a,
                     const fb::RigExecWirePathRead &b) {
                      return a.path < b.path;
                  });
        geometry.pathReads = std::move(reads);
    }

    for (size_t i = 0; i < geometry.weightObjects.size(); ++i) {
        fb::RigExecWireStep step;
        step.kind = fb::StepKind::WeightPacket;
        step.object = int32_t(i);
        step.cluster = 0;
        step.isSource = true;
        file.steps.push_back(std::move(step));
    }
    // Each volume's own step: part 1 places the one slot it names.
    for (size_t slot = 0; slot < slotCount; ++slot) {
        fb::RigExecWireStep step;
        step.kind = fb::StepKind::VolumePlacements;
        step.object = int32_t(slot);
        step.part = 1;
        step.writes = {fb::SlotRange(fb::SlotDomain::WeightFrames,
                                     uint32_t(slot), uint32_t(slot + 1))};
        step.cluster = 0;
        step.isSource = true;
        file.steps.push_back(std::move(step));
    }

    file.clustering = std::make_unique<fb::RigExecWireClustering>();
    {
        fb::RigExecWireCluster cluster;
        for (size_t i = 0; i < file.steps.size(); ++i) {
            cluster.members.push_back(int32_t(i));
        }
        file.clustering->clusters.push_back(std::move(cluster));
        file.clustering->clusterOf.assign(file.steps.size(), 0);
    }
    file.cones = std::make_unique<fb::RigExecWireCones>();
    {
        fb::RigExecWireClusterSet all;
        all.clusters = 1;
        all.words = {1};
        fb::RigExecWireClusterSet none;
        none.clusters = 1;
        none.words = {0};
        file.cones->cone.push_back(all);
        file.cones->always = std::make_unique<fb::RigExecWireClusterSet>(all);
        file.cones->poseClusters =
            std::make_unique<fb::RigExecWireClusterSet>(none);
        file.cones->avarCluster.assign(slotCount, 0);
        file.cones->chainBaseClusters.emplace_back();
    }

    // Every weight object field is read as a bake writes it: three
    // through slots of their own, each holding the value the capture
    // visits it with (object 2's default, then object 11's signed scale
    // before its legacy scale; distinct values expose a swapped slot), the
    // rest their table constants; and the painted arrays through inputs.
    for (const _Painted &entry : painted) {
        _PaintInputs(&file, entry.object, entry.values,
                     entry.sparse ? &entry.indices : nullptr);
    }
    CHECK(file.inputs.size() == 3 + 7);
    std::vector<uint8_t> bytes;
    std::string error;
    CHECK(RigExecFormatWrite(file, &bytes, &error));
    if (bytes.empty()) {
        std::printf("synthetic write diagnostic: %s\n", error.c_str());
        return;
    }
    std::unique_ptr<RigExecRuntimeReader> runtime =
        RigExecRuntimeReader::Open(bytes.data(), bytes.size(), &error);
    CHECK(runtime);
    if (!runtime) {
        std::printf("synthetic open diagnostic: %s\n", error.c_str());
        return;
    }
    // Pose and geometry prologues are siblings' work; the weight walk
    // runs under its own bit. A fresh reader plays the slots' defaults.
    runtime->SetRunMaskForTesting(0x2u);
    CHECK(runtime->Execute(&error));
    if (!error.empty()) {
        std::printf("synthetic execute diagnostic: %s\n",
                    error.c_str());
    }
    const std::vector<RrWeightPacket> &actual =
        runtime->GetWeightPackets();
    CHECK(actual.size() == expected.size());
    if (actual.size() != expected.size()) {
        return;
    }
    size_t ok = 0;
    for (size_t i = 0; i < expected.size(); ++i) {
        const std::string path =
            _WireString(file, geometry.weightObjects[i].path);
        if (_SynthCompare(path.c_str(), expected[i], actual[i], file)) {
            ++ok;
        } else {
            ++failures;
        }
    }
    // The placement over the default frame is the identity-landmark
    // map; the real kernel says exactly what that is.
    GfMatrix4d expectedPlacement(1.0);
    RigExecPointsToMatrix(RigExecPointFrame().points,
                          RigExecPointFrame().points,
                          &expectedPlacement);
    const std::vector<RigExecRuntimeWeightFrame> &frames =
        runtime->GetWeightFrames();
    CHECK(frames.size() == slotCount);
    if (frames.size() == slotCount) {
        for (size_t slot = 0; slot < slotCount; ++slot) {
            CHECK(frames[slot].path == volumeSlots[slot]);
            for (size_t r = 0; r < 4; ++r) {
                for (size_t c = 0; c < 4; ++c) {
                    CHECK(frames[slot].matrix[r][c] ==
                          expectedPlacement[r][c]);
                }
            }
        }
    }
    std::printf("synthetic: %zu/%zu packets match, %zu weight frame(s)\n",
                ok, expected.size(), frames.size());

    // The whole-map form (part -1) is retired: Open refuses it by name.
    if (const std::unique_ptr<fb::RigExecWireFile> wholeMap =
            RigExecTestUnpack(bytes)) {
        const size_t place = wholeMap->steps.size() - 1;
        CHECK(wholeMap->steps[place].kind == fb::StepKind::VolumePlacements);
        wholeMap->steps[place].part = -1;
        const std::vector<uint8_t> packed =
            RigExecTestPackUnchecked(*wholeMap);
        std::string why;
        const std::unique_ptr<RigExecRuntimeReader> refused =
            RigExecRuntimeReader::Open(packed.data(), packed.size(), &why);
        const std::string want =
            "invalid .rigexec: step " + std::to_string(place) +
            " (VolumePlacements every volume weight) is the retired "
            "whole-map placement (part -1)";
        CHECK(!refused && why == want);
        if (refused || why != want) {
            std::printf("  whole-map open said '%s', expected '%s'\n",
                        refused ? "(opened)" : why.c_str(), want.c_str());
        }
        std::printf("synthetic, whole-map VolumePlacements refused: %s\n",
                    why.c_str());
    }

    // Each slot set as an input: the next run builds every packet that
    // reads one from the new value -- object 2's default (0.3 -> 0.6),
    // the combine over it (object 10), and object 11's signed and legacy
    // scales (2 -> 1.5, 0.5 -> 0.75) -- and leaves the others as they were.
    // The other seven inputs are the painted arrays.
    CHECK(runtime->GetInputCount() == 3 + 7);
    CHECK(runtime->SetInput("/Synth/0.value", 0.6, &error));
    CHECK(runtime->SetInput("/Synth/1.value", 1.5, &error));
    CHECK(runtime->SetInput("/Synth/2.value", 0.75, &error));
    CHECK(runtime->Execute(&error));
    std::vector<RigExecWeightPacket> moved = expected;
    {
        RigExecStaticWeightInputs in;
        in.representation = TfToken("constant");
        in.rangePolicy = TfToken("strict");
        in.defaultWeight = 0.6f;
        moved[2] = RigExecBuildStaticWeightPacket(in);
    }
    moved[10] = RigExecBuildCombineWeightPacket(
        TfToken("dense"), TfToken("strict"), TfToken("add"),
        {moved[2], moved[6]}, 3, 0.5f, 0.0f);
    moved[25] = RigExecBuildCombineWeightPacket(
        TfToken("dense"), TfToken("strict"), TfToken("add"),
        {moved[2], moved[6]}, 3, 1.0f, 0.0f);
    {
        RigExecVolumeWeightInputs sphereIn = volumeInputs("y", "unbounded");
        sphereIn.samplePoints = signedSamples;
        sphereIn.scales = GfVec3f(1.0f, 0.75f, 1.0f);
        sphereIn.positiveScales = GfVec3f(1.5f, 3.0f, 1.0f);
        sphereIn.negativeScales = GfVec3f(1.0f, 1.0f, 4.0f);
        sphereIn.params.curve =
            RigExecBuildFalloffLut(RigExecFalloffProfile::Smooth);
        moved[11] = RigExecBuildVolumeWeightPacket(
            TfToken("RigExecSphereWeight"), sphereIn);
    }
    for (const size_t i : {size_t(2), size_t(10), size_t(11)}) {
        CHECK(moved[i].values != expected[i].values ||
              moved[i].defaultWeight != expected[i].defaultWeight);
    }
    const std::vector<RrWeightPacket> &set = runtime->GetWeightPackets();
    CHECK(set.size() == moved.size());
    size_t setOk = 0;
    for (size_t i = 0; i < moved.size() && i < set.size(); ++i) {
        const std::string path =
            _WireString(file, geometry.weightObjects[i].path);
        if (_SynthCompare(path.c_str(), moved[i], set[i], file)) {
            ++setOk;
        } else {
            ++failures;
        }
    }
    std::printf("synthetic, slots set as inputs: %zu/%zu packets match\n",
                setOk, moved.size());
}

// The read evaluator and the weight oracle over a hand-built file
// (inputs.cpp, weights.cpp): each read mode's arms, the property-chain
// overlay, and the oracle's arithmetic and error text on static, dynamic,
// combine and volume paths the baked fixtures do not reach. Expected fields
// repeat the oracle's float expressions in its order, with operands for
// which a reordering would change the bits.
static void
_TestComputedReadsAndOracle()
{
    using InputTag = fb::InputTag;
    using ReadMode = fb::ReadMode;
    fb::RigExecWireFile file;
    _Synth synth(&file);
    file.pose = std::make_unique<fb::RigExecWireDomainPose>();
    file.geometry = std::make_unique<fb::RigExecWireDomainGeometry>();
    std::vector<uint32_t> paths;
    for (int i = 0; i <= 16; ++i) {
        paths.push_back(synth.Path("/W" + std::to_string(i)));
    }
    std::vector<uint32_t> slotNames;
    for (int i = 0; i < 5; ++i) {
        slotNames.push_back(synth.Path("/S" + std::to_string(i) + ".v"));
    }
    const uint32_t tokStatic = synth.Token("RigExecStaticWeight");
    const uint32_t tokDynamic = synth.Token("RigExecDynamicWeight");
    const uint32_t tokCombine = synth.Token("RigExecCombineWeight");
    const uint32_t tokSphere = synth.Token("RigExecSphereWeight");
    const uint32_t tokConstant = synth.Token("constant");
    const uint32_t tokDense = synth.Token("dense");
    const uint32_t tokSparse = synth.Token("sparse");
    const uint32_t tokStrict = synth.Token("strict");
    const uint32_t tokClamp = synth.Token("clamp");
    const uint32_t tokAdd = synth.Token("add");
    const uint32_t tokMax = synth.Token("max");
    const uint32_t tokMultiply = synth.Token("multiply");
    const uint32_t tokSubtract = synth.Token("subtract");
    const uint32_t tokMin = synth.Token("min");
    const uint32_t tokAverage = synth.Token("average");
    const uint32_t tokOverlay = synth.Token("overlay");
    std::string error;

    const auto value = [&](InputTag tag, uint64_t bits) {
        fb::RigExecWireValue v;
        v.tag = tag;
        v.bits = bits;
        return synth.Value(std::move(v));
    };
    // Float constants pooled by their bits, so a read built after Open
    // finds its constant in the pool Open took.
    const auto floatValue = [&](float f) { return synth.Float(f); };
    const auto doubleValue = [&](double d) {
        uint64_t bits = 0;
        std::memcpy(&bits, &d, sizeof(bits));
        return value(InputTag::Double, bits);
    };
    // Slots: s0 float 0.25, s1 double 0.7, s2 float with no value, s3 int,
    // s4 double with no value.
    const InputTag slotTypes[] = {InputTag::Float, InputTag::Double,
                                  InputTag::Float, InputTag::Int,
                                  InputTag::Double};
    const uint32_t slotValues[] = {floatValue(0.25f), doubleValue(0.7),
                                   floatValue(0.0f), value(InputTag::Int, 3),
                                   doubleValue(0.0)};
    const bool slotHas[] = {true, true, false, true, false};
    for (size_t s = 0; s < 5; ++s) {
        file.inputs.emplace_back(
            slotNames[s], slotValues[s], -1, -1, slotTypes[s],
            uint8_t(fb::InputSlotFlags::Listed) |
                (slotHas[s] ? uint8_t(fb::InputSlotFlags::HasValue)
                            : uint8_t(0)));
    }
    file.listedInputs = 5;

    const auto read = [&](float constant, ReadMode mode, uint8_t flags,
                          std::vector<uint32_t> walk, int16_t selected = -1,
                          int32_t overrideIndex = -1) {
        fb::RigExecWireInput input;
        input.tag = InputTag::Float;
        input.mode = mode;
        input.flags = flags;
        input.constant = floatValue(constant);
        input.walk = std::move(walk);
        input.selected = selected;
        input.overrideIndex = overrideIndex;
        return input;
    };
    // A read whose one-slot walk holds \p v: the oracle reads a read's
    // walk, and falls back to its own read-site constant, never the
    // read's, when the walk yields nothing.
    const auto fixed = [&](float v) {
        file.inputs.emplace_back(0, floatValue(v), -1, -1, InputTag::Float,
                                 uint8_t(fb::InputSlotFlags::Listed) |
                                     uint8_t(fb::InputSlotFlags::HasValue));
        file.listedInputs = uint32_t(file.inputs.size());
        return _SynthOwn(read(v, ReadMode::Resolved, 0,
                              {uint32_t(file.inputs.size() - 1)}));
    };
    const auto object = [&](size_t index, uint32_t type,
                            uint32_t representation, uint32_t rangePolicy) {
        fb::RigExecWireWeightObject o;
        o.path = paths[index];
        o.type = type;
        o.representation = representation;
        o.rangePolicy = rangePolicy;
        for (std::unique_ptr<fb::RigExecWireInput> *input :
             {&o.defaultWeight, &o.driver, &o.scale, &o.bias, &o.strength,
              &o.invert, &o.falloffMin, &o.falloffMax, &o.scaleXPos,
              &o.scaleYPos, &o.scaleZPos, &o.scaleXNeg, &o.scaleYNeg,
              &o.scaleZNeg, &o.scaleX, &o.scaleY, &o.scaleZ, &o.extentU,
              &o.extentV}) {
            *input = fixed(1.0f);
        }
        o.defaultWeight = fixed(0.0f);
        o.bias = fixed(0.0f);
        o.invert = fixed(0.0f);
        o.falloffMin = fixed(0.0f);
        return o;
    };
    std::vector<fb::RigExecWireWeightObject> &objects =
        file.geometry->weightObjects;
    objects.push_back(object(0, tokStatic, tokConstant, tokStrict));
    objects[0].defaultWeight = fixed(0.5f);
    objects.push_back(object(1, tokStatic, tokDense, tokClamp));
    objects.push_back(object(2, tokStatic, tokSparse, tokStrict));
    objects.push_back(object(3, tokStatic, tokSparse, tokStrict));
    objects.push_back(object(4, tokDynamic, tokSparse, tokStrict));
    objects[4].base = 3;
    objects[4].driver = _SynthOwn(read(1.0f, ReadMode::Resolved, 0, {1}));
    objects[4].scale = fixed(0.85f);
    objects.push_back(object(5, tokCombine, tokDense, tokStrict));
    objects[5].combineMode = tokAdd;
    objects[5].inputs = {3, 4};
    objects[5].invert = fixed(0.25f);
    objects[5].strength = fixed(0.75f);
    objects.push_back(object(6, tokStatic, tokConstant, tokStrict));
    objects[6].oracleStaticError = "unknown rangePolicy on /W6";
    objects.push_back(object(7, tokCombine, tokDense, tokStrict));
    objects[7].combineMode = tokMax;
    objects[7].inputs = {6};
    objects.push_back(object(8, tokSphere, tokDense, tokClamp));
    objects[8].samplesInFlight = true;
    objects.push_back(object(9, tokDynamic, tokConstant, tokStrict));
    objects[9].driver = fixed(1.5f);
    objects.push_back(object(10, tokStatic, tokConstant, tokStrict));
    objects[10].defaultWeight = fixed(std::nanf(""));
    // The combine modes no stage case folds two fields with, over a
    // constant 0.75 and object 3's sparse field (order matters for
    // subtract and overlay).
    objects.push_back(object(11, tokStatic, tokConstant, tokStrict));
    objects[11].defaultWeight = fixed(0.75f);
    const uint32_t modes[] = {tokSubtract, tokMin, tokAverage, tokOverlay,
                              tokMultiply};
    for (size_t m = 0; m < 5; ++m) {
        objects.push_back(object(12 + m, tokCombine, tokDense, tokClamp));
        objects.back().combineMode = modes[m];
        objects.back().inputs = modes[m] == tokOverlay
                                    ? std::vector<int32_t>{3, 11}
                                    : std::vector<int32_t>{11, 3};
    }

    // The constant every read below falls back to.
    floatValue(9.0f);
    // The painted arrays, as inputs.
    {
        const std::vector<int32_t> repeated = {1, 1}, reversed = {2, 0};
        _PaintInputs(&file, 1, {0.25f, 1.5f}, nullptr);
        _PaintInputs(&file, 2, {0.1f, 0.2f}, &repeated);
        _PaintInputs(&file, 3, {0.2f, 0.1f}, &reversed);
    }
    // One volume slot, /W8's, so the oracle finds that object's placement
    // at slot 0 once it is placed.
    fb::RigExecWireSlotMeta slotMeta;
    slotMeta.paths = {paths[8]};
    fb::RigExecWireConstants constants;
    constants.noScaleAvars = {1};
    RrProgram program;
    program.poses = file.pose.get();
    program.geometry = file.geometry.get();
    program.slotMeta = &slotMeta;
    program.constants = &constants;
    program.stepWeightObjects = objects.size();
    for (uint32_t id = 0; id < file.paths.size(); ++id) {
        program.nodeText.push_back(RigExecFormatPathText(file, id));
    }
    CHECK(RrInputsOpen(&program, &file, &error));
    CHECK(RrPropertySizeScratch(&program, &error));
    CHECK(RrWeightSizeScratch(&program, &error));
    program.store.overridden.assign(4, 0);
    program.store.volumePlacement.assign(1, RrMat4d(1.0));
    program.store.volumePlaced.assign(1, 0);

    // Reads, mode by mode.
    const auto f = [&](const fb::RigExecWireInput &input) {
        return RrWireValueFloat(RrReadInput(&program, input));
    };
    const float narrowed = static_cast<float>(0.7);
    const uint8_t varying = uint8_t(fb::InputReadFlags::Varying);
    const uint8_t longWay = uint8_t(fb::InputReadFlags::LongWay);
    // Resolved: a double hop reads a double walk and casts it; else the
    // most upstream readable hop of the read's type; else the constant.
    CHECK(f(read(9.0f, ReadMode::Resolved, 0, {0, 1})) == narrowed);
    CHECK(f(read(9.0f, ReadMode::Resolved, 0, {0, 2})) == 0.25f);
    CHECK(f(read(9.0f, ReadMode::Resolved, 0, {2})) == 9.0f);
    CHECK(f(read(9.0f, ReadMode::Resolved, 0, {0, 4})) == 9.0f);
    CHECK(f(read(9.0f, ReadMode::Resolved, 0, {3})) == 9.0f);
    CHECK(RrReadResolvedFloat(&program,
                              read(9.0f, ReadMode::Baked, 0, {0, 1}),
                              5.0f) == narrowed);
    // The oracle's fallback, not the read's constant (a step-backed read's
    // folded value), when the walk yields nothing: here a float head whose
    // connection reaches a double with no value fails the double walk.
    CHECK(RrReadResolvedFloat(&program,
                              read(9.0f, ReadMode::Baked, 0, {0, 4}),
                              5.0f) == 5.0f);
    CHECK(RrReadResolvedFloat(&program, read(9.0f, ReadMode::Baked, 0, {}),
                              5.0f) == 5.0f);
    CHECK(RrReadResolvedFloat(&program,
                              read(9.0f, ReadMode::Baked, 0, {0, 2}),
                              5.0f) == 0.25f);
    // Baked: RigExecBakedRead's four arms.
    CHECK(f(read(9.0f, ReadMode::Baked, 0, {0})) == 9.0f);
    CHECK(f(read(9.0f, ReadMode::Baked, varying, {0}, 0)) == 0.25f);
    CHECK(f(read(9.0f, ReadMode::Baked, varying, {2}, 0)) == 9.0f);
    CHECK(f(read(9.0f, ReadMode::Baked, varying, {0, 1}, 1)) == 9.0f);
    CHECK(f(read(9.0f, ReadMode::Baked, varying | longWay, {0, 1})) ==
          narrowed);
    CHECK(f(read(9.0f, ReadMode::Baked, 0, {0, 1}, -1, 2)) == 9.0f);
    program.store.overridden[2] = 1;
    CHECK(f(read(9.0f, ReadMode::Baked, 0, {0, 1}, -1, 2)) == narrowed);
    program.store.overridden[2] = 0;
    // Pinned and Raw.
    CHECK(f(read(9.0f, ReadMode::Pinned, 0, {0})) == 0.25f);
    CHECK(f(read(9.0f, ReadMode::Pinned, 0, {0, 1})) == narrowed);
    CHECK(f(read(9.0f, ReadMode::Pinned, 0, {})) == 9.0f);
    CHECK(f(read(9.0f, ReadMode::Raw, 0, {1})) == 9.0f);
    CHECK(f(read(9.0f, ReadMode::Raw, 0, {0})) == 0.25f);

    // The overlay: a property-chain result published at a hop answers
    // there when it holds exactly the walk's type (GetAttribute's Get<T>).
    std::map<uint32_t, RrPropertyValue> &overlay =
        program.store.propertyResults;
    program.store.propertyVersions.resize(file.inputs.size());
    program.store.propertyVersionValid.assign(file.inputs.size(), 0);
    const auto clearPublication = [&]() {
        overlay.clear();
        std::fill(program.store.propertyPublishedVersions.begin(),
                  program.store.propertyPublishedVersions.end(), -1);
    };
    const auto publish = [&](size_t slot, RrPropertyValue::Tag tag,
                             double v) {
        RrPropertyValue held;
        held.tag = tag;
        held.f32 = float(v);
        held.f64 = v;
        overlay[slotNames[slot]] = held;
        program.store.propertyVersions[slot] = held;
        program.store.propertyVersionValid[slot] = 1;
        program.store.propertyPublishedVersions[slot] = int32_t(slot);
    };
    publish(0, RrPropertyValue::Tag::Float, 0.125);
    CHECK(f(read(9.0f, ReadMode::Resolved, 0, {0})) == 0.125f);
    // Before the double hop downstream of it.
    CHECK(f(read(9.0f, ReadMode::Resolved, 0, {0, 1})) == 0.125f);
    CHECK(f(read(9.0f, ReadMode::Pinned, 0, {0})) == 0.125f);
    CHECK(f(read(9.0f, ReadMode::Raw, 0, {0})) == 0.25f);
    CHECK(f(read(9.0f, ReadMode::Baked, 0, {0})) == 9.0f);
    CHECK(f(read(9.0f, ReadMode::Baked, varying | longWay, {0, 1})) ==
          0.125f);
    // A result of another type is no answer.
    publish(0, RrPropertyValue::Tag::Double, 0.125);
    CHECK(f(read(9.0f, ReadMode::Resolved, 0, {0, 2})) == 0.25f);
    clearPublication();
    // A double hop's own double result, narrowed.
    publish(1, RrPropertyValue::Tag::Double, 0.3);
    CHECK(f(read(9.0f, ReadMode::Resolved, 0, {0, 1})) ==
          static_cast<float>(0.3));
    clearPublication();
    // GetAttribute<float> tests its own head for a double before the
    // overlay, so a float result at a double head is not read.
    publish(1, RrPropertyValue::Tag::Float, 0.3);
    CHECK(f(read(9.0f, ReadMode::Resolved, 0, {1})) == narrowed);
    clearPublication();

    // The oracle.
    const auto resolve = [&](size_t index, size_t count,
                             const std::vector<RrVec3f> *current,
                             std::vector<float> *weights) {
        error.clear();
        return RrResolveWeightOracle(&program, index, count, current,
                                     weights, &error);
    };
    std::vector<float> w;
    CHECK(resolve(0, 1, nullptr, &w) && w == std::vector<float>{0.5f});
    CHECK(resolve(1, 2, nullptr, &w) &&
          w == (std::vector<float>{0.25f, 1.0f}));
    CHECK(!resolve(1, 3, nullptr, &w) &&
          error == "dense weight cardinality mismatch on /W1");
    CHECK(!resolve(2, 3, nullptr, &w) &&
          error == "duplicate sparse index on /W2");
    const std::vector<float> sparse = {0.1f, 0.0f, 0.2f};
    CHECK(resolve(3, 3, nullptr, &w) && w == sparse);
    CHECK(!resolve(3, 2, nullptr, &w) &&
          error == "sparse index out of range on /W3");
    std::vector<float> dynamic(3);
    size_t reassociated = 0;
    for (size_t i = 0; i < 3; ++i) {
        dynamic[i] = (sparse[i] * narrowed) * 0.85f + 0.0f;
        const float other = sparse[i] * (narrowed * 0.85f) + 0.0f;
        reassociated += std::memcmp(&other, &dynamic[i], sizeof(float)) != 0;
    }
    CHECK(reassociated == 2);
    CHECK(resolve(4, 3, nullptr, &w) && w == dynamic);
    // The driver through a double result published at its hop.
    publish(1, RrPropertyValue::Tag::Double, 0.5);
    std::vector<float> driven(3);
    for (size_t i = 0; i < 3; ++i) {
        driven[i] = (sparse[i] * 0.5f) * 0.85f + 0.0f;
    }
    CHECK(resolve(4, 3, nullptr, &w) && w == driven);
    clearPublication();
    std::vector<float> combined(3);
    for (size_t i = 0; i < 3; ++i) {
        float acc = (0.0f + sparse[i]) + dynamic[i];
        combined[i] = (acc + (1.0f - 2.0f * acc) * 0.25f) * 0.75f;
    }
    CHECK(resolve(5, 3, nullptr, &w) && w == combined);
    {
        std::vector<float> subtract(3), minimum(3), average(3), over(3),
            product(3);
        for (size_t i = 0; i < 3; ++i) {
            subtract[i] = 0.75f - sparse[i];
            minimum[i] = std::min(std::min(1.0f, 0.75f), sparse[i]);
            average[i] = ((0.0f + 0.75f) + sparse[i]) * (1.0f / 2.0f);
            over[i] = sparse[i] < 0.5f
                          ? 2.0f * sparse[i] * 0.75f
                          : 1.0f - 2.0f * (1.0f - sparse[i]) * (1.0f - 0.75f);
            product[i] = (1.0f * 0.75f) * sparse[i];
        }
        const std::vector<float> *expected[] = {&subtract, &minimum, &average,
                                                &over, &product};
        for (size_t m = 0; m < 5; ++m) {
            CHECK(resolve(12 + m, 3, nullptr, &w) && w == *expected[m]);
        }
    }
    CHECK(!resolve(6, 1, nullptr, &w) &&
          error == "unknown rangePolicy on /W6");
    CHECK(!resolve(7, 1, nullptr, &w) &&
          error == "unknown rangePolicy on /W6");
    CHECK(!resolve(8, 3, nullptr, &w) &&
          error == "/W8: no resolved placement for this volume weight");
    program.store.volumePlacement[0] = RrMat4d(1.0);
    program.store.volumePlaced[0] = 1;
    CHECK(!resolve(8, 3, nullptr, &w) &&
          error == "/W8: rigExec:weightTarget reads `preceding` but no "
                   "in-flight points were supplied");
    const std::vector<RrVec3f> two(2, RrVec3f(0.0f));
    CHECK(!resolve(8, 3, &two, &w) &&
          error == "/W8: sampled point count does not match the target");
    const std::vector<RrVec3f> three = {RrVec3f(0.0f), RrVec3f(0.5f, 0, 0),
                                        RrVec3f(3.0f, 0, 0)};
    CHECK(resolve(8, 3, &three, &w) && w.size() == 3);
    CHECK(!resolve(9, 1, nullptr, &w) &&
          error == "strict range violation on /W9");
    CHECK(!resolve(10, 1, nullptr, &w) &&
          error == "weight range violation on /W10");
    CHECK(!resolve(objects.size(), 1, nullptr, &w) &&
          error == "the file holds no weight object " +
                       std::to_string(objects.size()));
    std::printf("computed reads and oracle: checked\n");
}

// A VolumePlacements step the closure skips keeps its slot's placement and
// placed byte bit for bit, the oracle reads that kept placement, and only
// the slot's own step moves it. White-box over a hand-assembled program:
// at the default grain every VolumePlacements step of a baked rig shares
// one cluster, so playback runs them together.
static void
_TestVolumePlacementSkip()
{
    fb::RigExecWireFile file;
    _Synth synth(&file);
    file.pose = std::make_unique<fb::RigExecWireDomainPose>();
    file.geometry = std::make_unique<fb::RigExecWireDomainGeometry>();
    const char *const volumes[] = {"/Volume", "/Volume2"};
    fb::RigExecWireSlotMeta slotMeta;
    fb::RigExecWireConstants constants;
    for (uint32_t slot = 0; slot < 2; ++slot) {
        slotMeta.paths.push_back(synth.Path(volumes[slot]));
        constants.noScaleAvars.push_back(1);
        // A sphere riding the volume, measuring the in-flight points.
        fb::RigExecWireWeightObject object = _SynthObject(
            &synth, volumes[slot], "RigExecSphereWeight", "dense", "clamp");
        object.providerSlot = int32_t(slot);
        object.samplesInFlight = true;
        file.geometry->weightObjects.push_back(std::move(object));
        fb::RigExecWireStep step;
        step.kind = fb::StepKind::VolumePlacements;
        step.object = int32_t(slot);
        step.part = 1;
        step.reads = {fb::SlotRange(fb::SlotDomain::PoseFin, slot, slot + 1)};
        step.writes = {
            fb::SlotRange(fb::SlotDomain::WeightFrames, slot, slot + 1)};
        file.steps.push_back(std::move(step));
    }
    RrProgram program;
    program.steps = &file.steps;
    program.slotMeta = &slotMeta;
    program.constants = &constants;
    program.poses = file.pose.get();
    program.geometry = file.geometry.get();
    program.stepWeightObjects = file.geometry->weightObjects.size();
    for (uint32_t id = 0; id < file.paths.size(); ++id) {
        program.nodeText.push_back(RigExecFormatPathText(file, id));
    }
    std::string error;
    CHECK(RrInputsOpen(&program, &file, &error));
    CHECK(RrPropertySizeScratch(&program, &error));
    CHECK(RrWeightSizeScratch(&program, &error));
    RrStore &store = program.store;
    store.volumePlacement.assign(2, RrMat4d(1.0));
    store.volumePlaced.assign(2, 0);

    // Each slot's last pose version: distinct usable frames.
    const auto translated = [](double x, double y, double z) {
        RigExecPointFrame frame;
        for (GfVec3d &point : frame.points) {
            point += GfVec3d(x, y, z);
        }
        return frame;
    };
    const auto runtimeFrame = [](const RigExecPointFrame &frame) {
        RrPointFrame out;
        for (size_t i = 0; i < 4; ++i) {
            out.points[i] = RrVec3d(frame.points[i][0], frame.points[i][1],
                                    frame.points[i][2]);
        }
        out.flags = frame.flags;
        return out;
    };
    const RigExecPointFrame first = translated(0.5, 0.0, 0.0);
    const RigExecPointFrame second = translated(0.0, 0.25, 0.0);
    const RigExecPointFrame moved = translated(0.0, -0.25, 0.0);
    store.fin = {runtimeFrame(first), runtimeFrame(second)};
    store.finLast = {0, 1};

    // Slot \p slot holds the shared RigExecVolumePlacement of \p frame, bit
    // for bit.
    const auto placedAt = [&](size_t slot, const RigExecPointFrame &frame) {
        const GfMatrix4d expected = RigExecVolumePlacement(frame);
        const RrMat4d &actual = store.volumePlacement[slot];
        for (size_t r = 0; r < 4; ++r) {
            for (size_t c = 0; c < 4; ++c) {
                if (std::memcmp(&actual[r][c], &expected[int(r)][int(c)],
                                sizeof(double)) != 0) {
                    return false;
                }
            }
        }
        return true;
    };
    const auto run = [&](size_t step) {
        error.clear();
        const bool ok = RrRunWeightStep(&program, step, &error);
        CHECK(ok);
        if (!ok) {
            std::printf("  volume step %zu: %s\n", step, error.c_str());
        }
    };
    const std::vector<RrVec3f> samples = {RrVec3f(0.0f),
                                          RrVec3f(0.0f, 0.5f, 0.0f),
                                          RrVec3f(0.25f, 0.0f, 0.0f)};
    const auto resolve = [&](std::vector<float> *weights) {
        error.clear();
        return RrResolveWeightOracle(&program, 1, samples.size(), &samples,
                                     weights, &error);
    };

    // No step has run: no placement.
    std::vector<float> weights;
    CHECK(!resolve(&weights) &&
          error == "/Volume2: no resolved placement for this volume weight");
    run(0);
    run(1);
    CHECK(store.volumePlaced == std::vector<char>({1, 1}));
    CHECK(placedAt(0, first) && placedAt(1, second));
    std::vector<float> placedWeights;
    CHECK(resolve(&placedWeights) && placedWeights.size() == samples.size());
    const RrMat4d kept = store.volumePlacement[1];
    const RrMat4d other = store.volumePlacement[0];

    // Slot 1's pose moves and only slot 0's step runs: slot 1 keeps its
    // placement and placed byte, and the oracle reads the kept placement.
    store.fin[1] = runtimeFrame(moved);
    run(0);
    CHECK(store.volumePlaced[1] == 1);
    CHECK(std::memcmp(&store.volumePlacement[1], &kept, sizeof(RrMat4d)) ==
          0);
    CHECK(store.volumePlaced[0] == 1);
    CHECK(std::memcmp(&store.volumePlacement[0], &other, sizeof(RrMat4d)) ==
          0);
    std::vector<float> keptWeights;
    CHECK(resolve(&keptWeights) && keptWeights == placedWeights);

    // Its own step moves it.
    run(1);
    CHECK(placedAt(1, moved) && placedAt(0, first));
    CHECK(std::memcmp(&store.volumePlacement[1], &kept, sizeof(RrMat4d)) !=
          0);
    std::vector<float> movedWeights;
    CHECK(resolve(&movedWeights) && movedWeights != placedWeights);

    // A step naming no volume slot fails by name and changes nothing.
    const RrMat4d before = store.volumePlacement[1];
    constants.noScaleAvars[1] = 0;
    error.clear();
    CHECK(!RrRunWeightStep(&program, 1, &error) &&
          error == "weight step 1 places no volume slot");
    constants.noScaleAvars[1] = 1;
    CHECK(std::memcmp(&store.volumePlacement[1], &before, sizeof(RrMat4d)) ==
          0);
    std::printf("volume placement skip: a skipped step kept slot 1's "
                "placement bit for bit and the oracle read it; its own step "
                "moved it\n");
}

// tests/fixtures/volume_placements.usda, baked at frame 1 and held at frame
// 5 while one volume's control is dragged, against live baked with the same
// value authored in the session layer: every output, bit for bit, and the
// other volume's weight frame unmoved. Baked with every step in its own
// cluster (RIGEXEC_BAKED_GRAIN_US=0), a drag reruns its own volume's
// VolumePlacements step and not the other's, which the run reports, so the
// other volume's frame is the placement its skipped step kept. At the
// default grain both steps share a cluster and the drag checks values
// alone; _TestVolumePlacementSkip pins the kept placement there.
static void
_TestVolumePlacementDrags(const std::string &stagePath)
{
    const char *const name = "volume_placements drags";
    const UsdStageRefPtr stage = UsdStage::Open(stagePath);
    CHECK(stage);
    if (!stage) {
        return;
    }
    const SdfPath rigPath = _FindRig(stage);
    std::vector<uint8_t> bytes;
    std::string error;
    {
        RigExecRigEvaluator evaluator(stage, rigPath);
        evaluator.SetEvaluationMode(RigExecEvaluationMode::Baked);
        CHECK(RigExecTestBakeAt(evaluator, 1.0, &bytes, &error));
    }
    RigExecTestPlayer player;
    if (bytes.empty() || !player.Open(bytes, stage, &error)) {
        std::printf("%s: open: %s\n", name, error.c_str());
        CHECK(false);
        return;
    }
    const double frame = 5.0;
    std::vector<RigExecRigPose> plain;
    CHECK(RigExecTestEditedPoses(stage, rigPath,
                                 RigExecEvaluationMode::Baked, {},
                                 {1.0, frame}, &plain, &error));
    if (plain.size() != 2) {
        std::printf("%s: reference: %s\n", name, error.c_str());
        return;
    }
    const auto play = [&](const std::string &what, double time,
                          const RigExecRigPose &pose) {
        if (!player.Play(time, &error)) {
            std::printf("%s, %s: %s\n", name, what.c_str(), error.c_str());
            return false;
        }
        std::vector<std::string> diffs;
        if (RigExecCompareRuntimeOutputs(pose, player.Reader(), &diffs)) {
            return true;
        }
        std::printf("%s, %s:\n", name, what.c_str());
        for (const std::string &line : diffs) {
            std::printf("  %s\n", line.c_str());
        }
        return false;
    };
    // The published weight frame at \p path, or null.
    const auto frameAt = [&](const std::string &path) {
        const RigExecRuntimeWeightFrame *found = nullptr;
        for (const RigExecRuntimeWeightFrame &entry :
             player->GetWeightFrames()) {
            if (entry.path == path) {
                found = &entry;
            }
        }
        return found;
    };
    CHECK(play("frame 1", 1.0, plain[0]));
    CHECK(play("frame 5", frame, plain[1]));
    const std::string joints = "/PlacementAsset/Rig/Joints/";
    const std::string volumes[] = {joints + "A/SphereA", joints + "B/SphereB"};
    RrMat4d undragged[2];
    for (size_t v = 0; v < 2; ++v) {
        const RigExecRuntimeWeightFrame *entry = frameAt(volumes[v]);
        CHECK(entry);
        undragged[v] = entry ? entry->matrix : RrMat4d(1.0);
    }
    // Each volume's VolumePlacements step, by its label.
    int64_t placer[2] = {-1, -1};
    {
        std::unique_ptr<fb::RigExecWireFile> file;
        CHECK(RigExecFormatOpen(bytes.data(), bytes.size(), &file, &error));
        for (size_t s = 0; file && s < file->steps.size(); ++s) {
            if (file->steps[s].kind != fb::StepKind::VolumePlacements) {
                continue;
            }
            const std::string label = RigExecFormatStepLabel(*file, s);
            for (size_t v = 0; v < 2; ++v) {
                if (label == "VolumePlacements " + volumes[v]) {
                    placer[v] = int64_t(s);
                }
            }
        }
    }
    CHECK(placer[0] >= 0 && placer[1] >= 0);
    const bool fine = TfGetenv("RIGEXEC_BAKED_GRAIN_US", "") == "0";
    size_t skipped = 0;
    const std::string controls = "/PlacementAsset/Rig/Controls/";
    const std::string drags[] = {controls + "ACtl.avars:rz",
                                 controls + "BCtl.avars:rz"};
    const double value = 20.0;
    size_t matched = 0;
    for (size_t d = 0; d < 2; ++d) {
        std::vector<RigExecRigPose> dragged;
        CHECK(RigExecTestEditedPoses(
            stage, rigPath, RigExecEvaluationMode::Baked,
            {{SdfPath(drags[d]), VtValue(value)}}, {frame}, &dragged,
            &error));
        CHECK(player.Hold(drags[d], value, &error));
        bool ok = dragged.size() == 1 &&
                  play(drags[d] + " held", frame, dragged[0]);
        CHECK(ok);
        // The dragged volume moved; the other is its undragged placement,
        // bit for bit.
        const RigExecRuntimeWeightFrame *moved = frameAt(volumes[d]);
        const RigExecRuntimeWeightFrame *kept = frameAt(volumes[1 - d]);
        CHECK(player->GetWeightFrames().size() == 2 && moved && kept);
        if (moved && kept) {
            CHECK(!(moved->matrix == undragged[d]));
            CHECK(std::memcmp(&kept->matrix, &undragged[1 - d],
                              sizeof(RrMat4d)) == 0);
            ok = ok && !(moved->matrix == undragged[d]) &&
                 std::memcmp(&kept->matrix, &undragged[1 - d],
                             sizeof(RrMat4d)) == 0;
        }
        // One step per cluster: the dragged volume's step ran and the
        // other's did not.
        if (fine && placer[0] >= 0 && placer[1] >= 0) {
            const bool own = player->GetStepRanForTesting(size_t(placer[d]));
            const bool other =
                player->GetStepRanForTesting(size_t(placer[1 - d]));
            CHECK(own && !other);
            if (!own || other) {
                std::printf("%s, %s held: step %lld %s, step %lld %s\n",
                            name, drags[d].c_str(), (long long)placer[d],
                            own ? "ran" : "did not run",
                            (long long)placer[1 - d],
                            other ? "ran" : "did not run");
            }
            ok = ok && own && !other;
            skipped += own && !other ? 1 : 0;
        }
        player.ReleaseAll();
        const bool released = play(drags[d] + " released", frame, plain[1]);
        CHECK(released);
        matched += ok && released ? 1 : 0;
    }
    if (fine) {
        CHECK(skipped == 2);
    }
    const std::string steps =
        fine ? std::to_string(skipped) +
                   " of 2 skipped the other volume's step"
             : std::string("both steps share a cluster at this grain");
    std::printf("%s: %zu of 2 drag(s) match live baked, the other volume's "
                "frame kept; %s\n",
                name, matched, steps.c_str());
}

// A constraint's envelope painted as a dense static weight of one value,
// which the oracle reads at the evaluation time: the export lists the
// value as an input of the envelope entry, and an authored set of it
// moves the constrained joint as the same value authored in the session
// layer does, in the dynamic and the baked evaluator.
static void
_TestPaintedEnvelopeSet()
{
    const char *const name = "painted envelope";
    const UsdStageRefPtr stage = UsdStage::CreateInMemory();
    const auto place = [](const UsdPrim &prim, double tx) {
        const char *const names[3] = {"avars:tx", "avars:ty", "avars:tz"};
        for (int k = 0; k < 3; ++k) {
            prim.CreateAttribute(TfToken(names[k]), SdfValueTypeNames->Double)
                .Set(k == 0 ? tx : 0.0);
        }
    };
    const UsdPrim source =
        stage->DefinePrim(SdfPath("/Asset/Source"), TfToken("Xform"));
    source.CreateAttribute(TfToken("xformOp:translate"),
                           SdfValueTypeNames->Double3)
        .Set(GfVec3d(10.0, 0.0, 0.0));
    source.CreateAttribute(TfToken("xformOpOrder"),
                           SdfValueTypeNames->TokenArray)
        .Set(VtTokenArray{TfToken("xformOp:translate")});
    stage->DefinePrim(SdfPath("/Asset/Rig"), TfToken("RigExecRoot"));
    const UsdPrim joint = stage->DefinePrim(SdfPath("/Asset/Rig/Joints/J"),
                                            TfToken("RigExecJoint"));
    place(joint, 0.0);
    const std::string values = "/Asset/Rig/Weights/Painted.rigExec:values";
    const UsdPrim painted = stage->DefinePrim(
        SdfPath("/Asset/Rig/Weights/Painted"), TfToken("RigExecStaticWeight"));
    painted.CreateRelationship(TfToken("rigExec:weightTarget"))
        .SetTargets({joint.GetPath()});
    painted.CreateAttribute(TfToken("rigExec:representation"),
                            SdfValueTypeNames->Token)
        .Set(TfToken("dense"));
    painted.CreateAttribute(TfToken("rigExec:defaultWeight"),
                            SdfValueTypeNames->Float)
        .Set(0.0f);
    painted.CreateAttribute(TfToken("rigExec:values"),
                            SdfValueTypeNames->FloatArray)
        .Set(VtFloatArray{0.5f});
    const UsdPrim constraint = stage->DefinePrim(
        SdfPath("/Asset/Rig/Movers/Pull"),
        TfToken("RigExecPositionConstraint"));
    CHECK(constraint.ApplyAPI(TfToken("RigExecMoverAPI")));
    constraint.CreateRelationship(TfToken("rigExec:moves"))
        .SetTargets({joint.GetPath()});
    constraint.CreateRelationship(TfToken("rigExec:sources"))
        .SetTargets({source.GetPath()});
    constraint.CreateRelationship(TfToken("rigExec:weightObject"))
        .SetTargets({painted.GetPath()});
    const SdfPath rigPath("/Asset/Rig");
    std::vector<uint8_t> bytes;
    std::string error;
    {
        RigExecRigEvaluator evaluator(stage, rigPath);
        evaluator.SetEvaluationMode(RigExecEvaluationMode::Baked);
        CHECK(RigExecTestBakeAt(evaluator, 1.0, &bytes, &error));
    }
    // The envelope entry names its painted input.
    bool listed = false;
    if (const std::unique_ptr<fb::RigExecWireFile> file =
            RigExecTestUnpack(bytes)) {
        for (const fb::RigExecWireConstraint &c : file->pose->constraints) {
            if (c.weightObjectIndex < 0) {
                continue;
            }
            const fb::RigExecWireWeightObject &object =
                file->geometry->weightObjects[size_t(c.weightObjectIndex)];
            listed = object.envelopeOnly &&
                     object.valuesSlot == RigExecTestSlotOf(*file, values) &&
                     object.valuesSlot >= 0;
        }
    }
    CHECK(listed);
    std::unique_ptr<RigExecRuntimeReader> reader =
        RigExecRuntimeReader::Open(bytes.data(), bytes.size(), &error);
    if (!listed || !reader || !reader->Execute(&error)) {
        std::printf("%s: FAILED (%s)\n", name, error.c_str());
        CHECK(false);
        return;
    }
    const std::vector<RigExecRuntimeJointMatrix> before =
        reader->GetJointMatrices();
    const std::vector<RigExecTestArraySet> sets = {
        {values, VtValue(VtFloatArray{0.25f}), false}};
    CHECK(RigExecTestApplyArraySets(reader.get(), sets, &error) &&
          reader->Execute(&error));
    bool same = true;
    for (const auto mode :
         {RigExecEvaluationMode::Dynamic, RigExecEvaluationMode::Baked}) {
        RigExecRigPose pose;
        CHECK(RigExecTestArrayReference(stage, rigPath, mode, sets, 1.0,
                                        &pose, &error));
        std::vector<std::string> diffs;
        if (!RigExecCompareRuntimeRun(pose, *reader, &diffs)) {
            same = false;
            for (const std::string &diff : diffs) {
                std::printf("  %s: %s\n", name, diff.c_str());
            }
        }
    }
    CHECK(same);
    const std::vector<RigExecRuntimeJointMatrix> &after =
        reader->GetJointMatrices();
    bool moved = before.size() != after.size();
    for (size_t i = 0; !moved && i < before.size(); ++i) {
        moved = !(before[i].matrix == after[i].matrix);
    }
    CHECK(moved);
    std::printf("%s: %s set to 0.25 %s the session edit and moved the "
                "joint\n",
                name, values.c_str(), same ? "==" : "differs from");
}

int
main(int argc, char **argv)
{
    PlugRegistry::GetInstance().RegisterPlugins(
        RIGEXEC_SCHEMA_RESOURCE_DIR);
    _TestSyntheticBuilders();
    _TestComputedReadsAndOracle();
    _TestPaintedEnvelopeSet();
    _TestVolumePlacementSkip();
    _TestVolumePlacementDrags(std::string(RIGEXEC_TEST_FIXTURES_DIR) +
                              "/volume_placements.usda");

    std::string examplesDir = RIGEXEC_EXAMPLES_DIR;
    if (argc > 1) {
        examplesDir = argv[1];
    }
    bool sawBaking = false;
    for (const RigExecExampleFixture &fixture :
         kRigExecExampleFixtures) {
        if (!fixture.bakesToday) {
            continue;
        }
        sawBaking = true;
        const std::vector<double> frames =
            _ParseFrames(fixture.frames);
        CHECK(!frames.empty());
        if (frames.empty()) {
            continue;
        }
        _TestFixture(fixture.stage, examplesDir + "/" + fixture.stage,
                     frames, std::string(fixture.animation) == "static");
    }
    CHECK(sawBaking);

    if (failures == 0) {
        std::printf("testRigExecRuntimeWeights: all tests passed\n");
        return 0;
    }
    std::printf("testRigExecRuntimeWeights: %d failures\n", failures);
    return 1;
}
