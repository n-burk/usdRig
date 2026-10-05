// rigExecRuntime weight-family parity (M2): baked program vs runtime over
// every baking fixture, comparing weight packets bit for bit, plus
// synthetic builder checks against the real baked builders.
// Fixture strategy per fixture: bake in-process at the first frame, then
// run a fresh evaluator (Evaluate) and a fresh reader played through its
// inputs (the input sampler hands it the stage's animated inputs) frame by
// frame in order, comparing every packet, the weight frames and the
// diagnostics. A `static` row plays the bake time alone.
#include "rigExecBake/bake.h"
#include "rigExec/bakedProgram.h"
#include "rigExec/bakedProgramImpl.h"
#include "rigExec/rigEvaluator.h"
#include "rigExec/types.h"
#include "rigExec/weightPackets.h"
#include "rigExecBinary/container.h"
#include "rigExecBinary/geometry.h"
#include "rigExecBinary/inputTable.h"
#include "rigExecBinary/program.h"
#include "rigExecMath/pointFrame.h"
#include "rigExecMath/weightFields.h"
#include "rigExecRuntime/runtime.h"
#include "rigExecExampleFixtures.h"

#include "pxr/base/plug/registry.h"
#include "pxr/usd/usd/primRange.h"
#include "pxr/usd/usd/stage.h"

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <cstring>
#include <map>
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

static std::string
_WireString(RigExecBinaryReader *reader, uint32_t id)
{
    std::string out;
    reader->GetString(id, &out);
    return out;
}

// Decodes the geometry domain and input table straight from the baked
// bytes, for test-side introspection (object types for the deferral
// analysis, token text for packet comparison).
static bool
_DecodeWeightWire(const std::vector<uint8_t> &bytes,
                  std::unique_ptr<RigExecBinaryReader> *reader,
                  RigExecWireDomainGeometry *geometry,
                  std::string *error)
{
    *reader = RigExecBinaryReader::Open(bytes.data(), bytes.size(),
                                        error);
    if (!*reader) {
        return false;
    }
    const uint8_t *data = nullptr;
    size_t size = 0;
    if (!(*reader)->FindSection(RigExecBinarySection::DomainGeometry,
                                &data, &size)) {
        if (error) {
            *error = "test cannot find the geometry section";
        }
        return false;
    }
    RigExecWireReader cursor(data, size);
    return RigExecWireDecodeDomainGeometry(&cursor, geometry, error);
}

static bool
_ComparePacket(RigExecBinaryReader *reader,
               const RigExecWireWeightObject &wire,
               const RigExecWeightPacket &baked,
               const RrWeightPacket &actual, double frame)
{
    const std::string path = _WireString(reader, wire.path);
    bool ok = true;
    const auto note = [&](const char *field) {
        std::printf("  packet %s @ %g: %s differs\n", path.c_str(),
                    frame, field);
        ok = false;
    };
    if (_WireString(reader, actual.representation) !=
        baked.representation.GetString()) {
        note("representation");
    }
    if (_WireString(reader, actual.rangePolicy) !=
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
            const std::vector<uint8_t> &bytes, RigExecBinaryReader *reader,
            const RigExecWireDomainGeometry &geometry, size_t *compared)
{
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
        if (bakedPackets.size() != geometry.weightObjects.size() ||
            actualPackets.size() != geometry.weightObjects.size()) {
            std::printf("  packet storage @ %g: baked %zu runtime %zu "
                        "wire %zu\n",
                        frame, bakedPackets.size(),
                        actualPackets.size(),
                        geometry.weightObjects.size());
            return false;
        }
        for (size_t i = 0; i < geometry.weightObjects.size(); ++i) {
            ++(*compared);
            if (!_ComparePacket(reader, geometry.weightObjects[i],
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
    std::unique_ptr<RigExecBinaryReader> reader;
    RigExecWireDomainGeometry geometry;
    CHECK(_DecodeWeightWire(bytes, &reader, &geometry, &error));
    if (!reader) {
        std::printf("wire decode diagnostic: %s\n", error.c_str());
        return;
    }
    // A `static` row holds an animated source in static data at the bake
    // time: it plays that time alone.
    const std::vector<double> times =
        staticClass ? std::vector<double>{frames.front()} : frames;
    size_t compared = 0;
    if (_RunFixture(stage, times, bytes, reader.get(), geometry,
                    &compared)) {
        std::printf("%s: %zu frame(s) of a bake at %g%s, packets=%zu "
                    "weightFrames=yes diagnostics=yes OK\n",
                    name.c_str(), times.size(), frames.front(),
                    staticClass ? " (static)" : "", compared);
        return;
    }
    ++failures;
    std::printf("%s: FAILED\n", name.c_str());
}

// Synthetic builder checks: a hand-built wire program exercising every
// builder arm against the real baked builders, including the pathReads
// and chain-base gather layers no fixture populates for weight
// attributes, and the invalid-packet shapes.

static RigExecWireInput
_SynthFloat(float value)
{
    RigExecWireInput input;
    input.tag = RigExecWireInput::Tag::Float;
    input.f32 = value;
    return input;
}

static RigExecWireInput
_SynthVaryingFloat(float constant)
{
    RigExecWireInput input = _SynthFloat(constant);
    input.varying = true;
    input.bound = true;
    return input;
}

static RigExecWireWeightObject
_SynthObject(RigExecBinaryWriter *writer, const char *path,
             const char *type, const char *representation,
             const char *rangePolicy)
{
    RigExecWireWeightObject object;
    object.path = writer->AddString(path);
    object.type = writer->AddString(type);
    object.representation = writer->AddString(representation);
    object.rangePolicy = writer->AddString(rangePolicy);
    object.defaultWeight = _SynthFloat(0.0f);
    object.driver = _SynthFloat(1.0f);
    object.scale = _SynthFloat(1.0f);
    object.bias = _SynthFloat(0.0f);
    object.strength = _SynthFloat(1.0f);
    object.invert = _SynthFloat(0.0f);
    object.falloffMin = _SynthFloat(0.0f);
    object.falloffMax = _SynthFloat(1.0f);
    object.scaleX = _SynthFloat(1.0f);
    object.scaleY = _SynthFloat(1.0f);
    object.scaleZ = _SynthFloat(1.0f);
    object.extentU = _SynthFloat(1.0f);
    object.extentV = _SynthFloat(1.0f);
    return object;
}

static void
_SynthAttr(RigExecBinaryWriter *writer, const char *path,
           std::vector<uint32_t> *paths, std::vector<uint8_t> *valid)
{
    paths->push_back(writer->AddString(path));
    valid->push_back(1);
}

static RigExecWirePathRead
_SynthPointsRead(RigExecBinaryWriter *writer, const char *path,
                 const std::vector<GfVec3f> &points)
{
    RigExecWirePathRead read;
    read.path = writer->AddString(path);
    read.value.tag = RigExecWirePathValue::Tag::Vec3fArray;
    for (const GfVec3f &p : points) {
        read.value.vec3s.push_back(
            RigExecWireVec3f{p[0], p[1], p[2]});
    }
    return read;
}

// The Computed section a bake writes beside the synthetic tables: one
// registered read per weight object field, each input the directory gives
// a uid (in the capture's order) read per run through a slot named as that
// entry's head and holding each frame's recorded value, every other field
// its table constant; and one step-backed weight object per table entry,
// whose reads only the weight oracle makes.
static std::vector<uint8_t>
_SynthComputed(RigExecBinaryWriter *writer,
               const RigExecWireDomainGeometry &geometry,
               RigExecWireInputTable *inputs)
{
    using v4::InputTag;
    RigExecWireComputed computed;
    computed.bakeTime =
        inputs->frames.empty() ? 0.0 : inputs->frames[0].frame;
    computed.values.push_back(v4::RigExecWireValue());
    computed.vec3fArrays.emplace_back();
    std::map<uint32_t, uint32_t> floats;
    const auto intern = [&](float f) {
        uint32_t bits = 0;
        std::memcpy(&bits, &f, sizeof(bits));
        const auto found =
            floats.emplace(bits, uint32_t(computed.values.size()));
        if (found.second) {
            v4::RigExecWireValue value;
            value.tag = InputTag::Float;
            value.bits = bits;
            computed.values.push_back(value);
        }
        return found.first->second;
    };
    // The uid of each slot, in slot order.
    std::vector<uint32_t> slotUids;
    uint32_t nextUid = 0;
    for (size_t i = 0; i < geometry.weightObjects.size(); ++i) {
        const RigExecWireWeightObject &wire = geometry.weightObjects[i];
        const RigExecWireInput *const fields[RrWeightFieldCount] = {
            &wire.defaultWeight, &wire.driver,    &wire.scale,
            &wire.bias,          &wire.strength,  &wire.invert,
            &wire.falloffMin,    &wire.falloffMax, &wire.scaleXPos,
            &wire.scaleYPos,     &wire.scaleZPos, &wire.scaleXNeg,
            &wire.scaleYNeg,     &wire.scaleZNeg, &wire.scaleX,
            &wire.scaleY,        &wire.scaleZ,    &wire.extentU,
            &wire.extentV};
        v4::RigExecWireWeightObject object;
        object.path = wire.path;
        object.type = wire.type;
        object.representation = wire.representation;
        object.rangePolicy = wire.rangePolicy;
        v4::RigExecWireInput *const reads[RrWeightFieldCount] = {
            &object.defaultWeight, &object.driver,    &object.scale,
            &object.bias,          &object.strength,  &object.invert,
            &object.falloffMin,    &object.falloffMax, &object.scaleXPos,
            &object.scaleYPos,     &object.scaleZPos, &object.scaleXNeg,
            &object.scaleYNeg,     &object.scaleZNeg, &object.scaleX,
            &object.scaleY,        &object.scaleZ,    &object.extentU,
            &object.extentV};
        for (size_t f = 0; f < size_t(RrWeightFieldCount); ++f) {
            const RigExecWireInput &input = *fields[f];
            reads[f]->tag = InputTag::Float;
            reads[f]->mode = v4::ReadMode::Resolved;
            reads[f]->constant = intern(input.f32);
            RigExecWireRegisteredRead entry;
            entry.family = RigExecWireRegisteredFamily::WeightObject;
            entry.object = uint32_t(i);
            entry.field = uint32_t(f);
            entry.read.tag = InputTag::Float;
            entry.read.mode = v4::ReadMode::Baked;
            entry.read.overrideIndex = input.overrideIndex;
            entry.read.constant = intern(input.f32);
            if (input.varying) {
                entry.read.flags |= uint8_t(v4::InputReadFlags::Varying);
            }
            if (input.varying && input.bound) {
                const uint32_t slot = uint32_t(computed.inputs.size());
                v4::InputSlot attribute;
                attribute.name = writer->AddString(
                    "/Synth/" + std::to_string(slot) + ".value");
                attribute.type = InputTag::Float;
                attribute.flags =
                    uint8_t(v4::InputSlotFlags::Listed) |
                    uint8_t(v4::InputSlotFlags::Animated) |
                    uint8_t(v4::InputSlotFlags::HasValue);
                attribute.value = entry.read.constant;
                computed.inputs.push_back(attribute);
                CHECK(nextUid < inputs->directory.size());
                if (nextUid < inputs->directory.size()) {
                    inputs->directory[nextUid].head = attribute.name;
                }
                entry.uid = int32_t(nextUid);
                entry.read.walk = {slot};
                entry.read.selected = 0;
                slotUids.push_back(nextUid++);
            }
            computed.registeredReads.push_back(entry);
        }
        computed.weightObjects.push_back(std::move(object));
    }
    computed.listedInputs = uint32_t(computed.inputs.size());
    for (const RigExecWireFrameInputs &record : inputs->frames) {
        RigExecWireComputedFrame frame;
        frame.frame = record.frame;
        for (const uint32_t uid : slotUids) {
            float value = 0.0f;
            for (size_t k = 0; k < record.uids.size(); ++k) {
                if (record.uids[k] == uid) {
                    value = record.values[k].f32;
                }
            }
            frame.values.push_back(intern(value));
            frame.hasValue.push_back(1);
        }
        computed.frames.push_back(std::move(frame));
    }
    for (size_t s = 0; s < computed.inputs.size() &&
                       !computed.frames.empty();
         ++s) {
        computed.inputs[s].value = computed.frames[0].values[s];
    }
    std::vector<uint8_t> payload;
    std::string error;
    CHECK(RigExecWireEncodeComputed(computed, &payload, &error));
    if (!error.empty()) {
        std::printf("synthetic computed section: %s\n", error.c_str());
    }
    return payload;
}

static bool
_SynthCompare(const char *name, const RigExecWeightPacket &expected,
              const RrWeightPacket &actual,
              RigExecBinaryReader *reader)
{
    bool ok = true;
    const auto note = [&](const char *field) {
        std::printf("  synthetic %s: %s differs\n", name, field);
        ok = false;
    };
    if (_WireString(reader, actual.representation) !=
        expected.representation.GetString()) {
        note("representation");
    }
    if (_WireString(reader, actual.rangePolicy) !=
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
    RigExecBinaryWriter writer;
    RigExecWireDomainGeometry geometry;
    RigExecWireInputTable inputs;
    RigExecWireSlotMeta slotMeta;
    RigExecWireConstants constants;
    std::vector<RigExecWireStep> steps;

    // One provider slot carrying the volumes; the pools hold default
    // frames, so placements are exactly the identity-landmark map.
    slotMeta.paths.push_back(writer.AddString("/Volume"));
    constants.avarConstants.assign(11, 0.0);
    constants.noScaleAvars.push_back(1);
    {
        RigExecWireMatrix4d identity{};
        identity[0] = identity[5] = identity[10] = identity[15] =
            1.0;
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

    std::vector<RigExecWeightPacket> expected;

    // 0: static dense, valid.
    {
        RigExecWireWeightObject object = _SynthObject(
            &writer, "/w/dense", "RigExecStaticWeight", "dense",
            "strict");
        object.values = {0.25f, 0.5f, 0.75f};
        geometry.weightObjects.push_back(object);
        RigExecStaticWeightInputs in;
        in.representation = TfToken("dense");
        in.rangePolicy = TfToken("strict");
        in.values = {0.25f, 0.5f, 0.75f};
        in.defaultWeight = 0.0f;
        expected.push_back(RigExecBuildStaticWeightPacket(in));
    }
    // 1: static sparse, unsorted pairs canonicalize.
    {
        RigExecWireWeightObject object = _SynthObject(
            &writer, "/w/sparse", "RigExecStaticWeight", "sparse",
            "strict");
        object.values = {0.9f, 0.1f};
        object.indices = {3, 1};
        object.defaultWeight = _SynthFloat(0.5f);
        geometry.weightObjects.push_back(object);
        RigExecStaticWeightInputs in;
        in.representation = TfToken("sparse");
        in.rangePolicy = TfToken("strict");
        in.values = {0.9f, 0.1f};
        in.indices = {3, 1};
        in.defaultWeight = 0.5f;
        expected.push_back(RigExecBuildStaticWeightPacket(in));
    }
    // 2: static constant over a VARYING default (uid 0 reads 0.3).
    {
        RigExecWireWeightObject object = _SynthObject(
            &writer, "/w/varying", "RigExecStaticWeight", "constant",
            "strict");
        object.defaultWeight = _SynthVaryingFloat(0.7f);
        geometry.weightObjects.push_back(object);
        RigExecStaticWeightInputs in;
        in.representation = TfToken("constant");
        in.rangePolicy = TfToken("strict");
        in.defaultWeight = 0.3f;
        expected.push_back(RigExecBuildStaticWeightPacket(in));
    }
    // 3: static dense with a nonzero default: invalid, tokens kept.
    {
        RigExecWireWeightObject object = _SynthObject(
            &writer, "/w/denseBadDefault", "RigExecStaticWeight",
            "dense", "strict");
        object.values = {0.5f};
        object.defaultWeight = _SynthFloat(0.5f);
        geometry.weightObjects.push_back(object);
        RigExecStaticWeightInputs in;
        in.representation = TfToken("dense");
        in.rangePolicy = TfToken("strict");
        in.values = {0.5f};
        in.defaultWeight = 0.5f;
        expected.push_back(RigExecBuildStaticWeightPacket(in));
    }
    // 4: static sparse with duplicate indices: invalid, tokens kept.
    {
        RigExecWireWeightObject object = _SynthObject(
            &writer, "/w/sparseDup", "RigExecStaticWeight", "sparse",
            "strict");
        object.values = {0.5f, 0.5f};
        object.indices = {2, 2};
        geometry.weightObjects.push_back(object);
        RigExecStaticWeightInputs in;
        in.representation = TfToken("sparse");
        in.rangePolicy = TfToken("strict");
        in.values = {0.5f, 0.5f};
        in.indices = {2, 2};
        expected.push_back(RigExecBuildStaticWeightPacket(in));
    }
    // 5: static with an unknown representation: invalid, tokens kept.
    {
        RigExecWireWeightObject object = _SynthObject(
            &writer, "/w/unknownRep", "RigExecStaticWeight",
            "exponential", "strict");
        object.values = {0.5f};
        geometry.weightObjects.push_back(object);
        RigExecStaticWeightInputs in;
        in.representation = TfToken("exponential");
        in.rangePolicy = TfToken("strict");
        in.values = {0.5f};
        expected.push_back(RigExecBuildStaticWeightPacket(in));
    }
    // 6: dynamic over no base, clamp policy: 1.1 clamps to 1.
    {
        RigExecWireWeightObject object = _SynthObject(
            &writer, "/w/dynFree", "RigExecDynamicWeight", "constant",
            "clamp");
        object.driver = _SynthFloat(0.5f);
        object.scale = _SynthFloat(2.0f);
        object.bias = _SynthFloat(0.1f);
        geometry.weightObjects.push_back(object);
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
        RigExecWireWeightObject object = _SynthObject(
            &writer, "/w/dynDense", "RigExecDynamicWeight", "dense",
            "strict");
        object.base = 0;
        object.scale = _SynthFloat(0.5f);
        geometry.weightObjects.push_back(object);
        RigExecDynamicWeightInputs in;
        in.representation = TfToken("dense");
        in.rangePolicy = TfToken("strict");
        in.scale = 0.5f;
        expected.push_back(
            RigExecBuildDynamicWeightPacket(in, &expected[0]));
    }
    // 8: dynamic over object 0 with a mismatched representation.
    {
        RigExecWireWeightObject object = _SynthObject(
            &writer, "/w/dynMismatch", "RigExecDynamicWeight",
            "sparse", "strict");
        object.base = 0;
        geometry.weightObjects.push_back(object);
        RigExecDynamicWeightInputs in;
        in.representation = TfToken("sparse");
        in.rangePolicy = TfToken("strict");
        expected.push_back(
            RigExecBuildDynamicWeightPacket(in, &expected[0]));
    }
    // 9: combine over two dense fields (subtract, authored order).
    {
        RigExecWireWeightObject object = _SynthObject(
            &writer, "/w/combine", "RigExecCombineWeight", "dense",
            "strict");
        object.combineMode = writer.AddString("subtract");
        object.inputs = {0, 7};
        geometry.weightObjects.push_back(object);
        expected.push_back(RigExecBuildCombineWeightPacket(
            TfToken("dense"), TfToken("strict"),
            TfToken("subtract"),
            {expected[0], expected[7]}, 0, 1.0f, 0.0f));
    }
    // 10: combine over two constants, sized by its weight target.
    {
        RigExecWireWeightObject object = _SynthObject(
            &writer, "/w/combineConst", "RigExecCombineWeight",
            "dense", "strict");
        object.combineMode = writer.AddString("add");
        object.inputs = {2, 6};
        object.strength = _SynthFloat(0.5f);
        _SynthAttr(&writer, "/mesh.points",
                   &object.combineTargetPoints,
                   &object.combineTargetValid);
        geometry.weightObjects.push_back(object);
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
        RigExecWireWeightObject object = _SynthObject(
            &writer, "/w/sphere", "RigExecSphereWeight", "dense",
            "clamp");
        object.providerSlot = 0;
        object.falloffMax = _SynthFloat(2.0f);
        _SynthAttr(&writer, "/signed.points", &object.samplePoints, &object.sampleValid);
        object.scaleXPos = _SynthVaryingFloat(9.0f);
        object.scaleY = _SynthVaryingFloat(9.0f);
        object.scaleYPos = _SynthFloat(3.0f);
        object.scaleZNeg = _SynthFloat(4.0f);
        object.falloffCurve =
            RigExecBuildFalloffLut(RigExecFalloffProfile::Smooth);
        _SynthAttr(&writer, "/mesh.points", &object.targetPoints,
                   &object.targetValid);
        geometry.weightObjects.push_back(object);
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
        RigExecWireWeightObject object = _SynthObject(
            &writer, "/w/plane", "RigExecPlaneWeight", "dense",
            "clamp");
        object.providerSlot = 0;
        object.falloffMax = _SynthFloat(2.0f);
        object.planeAxis = writer.AddString("x");
        object.planeBounds = writer.AddString("bounded");
        _SynthAttr(&writer, "/mesh.points", &object.targetPoints,
                   &object.targetValid);
        geometry.weightObjects.push_back(object);
        RigExecVolumeWeightInputs in =
            volumeInputs("x", "bounded");
        in.extentU = 1.0f;
        in.extentV = 1.0f;
        expected.push_back(RigExecBuildVolumeWeightPacket(
            TfToken("RigExecPlaneWeight"), in));
    }
    // 13: curve weight with pathReads curve points.
    {
        RigExecWireWeightObject object = _SynthObject(
            &writer, "/w/curve", "RigExecCurveWeight", "dense",
            "clamp");
        object.providerSlot = 0;
        object.falloffMax = _SynthFloat(2.0f);
        _SynthAttr(&writer, "/mesh.points", &object.targetPoints,
                   &object.targetValid);
        _SynthAttr(&writer, "/curve.points", &object.curvePoints,
                   &object.curveValid);
        geometry.weightObjects.push_back(object);
        RigExecVolumeWeightInputs in =
            volumeInputs("y", "unbounded");
        in.curvePoints = curvePoints;
        expected.push_back(RigExecBuildVolumeWeightPacket(
            TfToken("RigExecCurveWeight"), in));
    }
    // 14: sphere with a mistyped sample override: cardinality
    // mismatch is invalid, tokens kept.
    {
        RigExecWireWeightObject object = _SynthObject(
            &writer, "/w/sampleBad", "RigExecSphereWeight", "dense",
            "clamp");
        object.providerSlot = 0;
        object.falloffMax = _SynthFloat(2.0f);
        _SynthAttr(&writer, "/mesh.points", &object.targetPoints,
                   &object.targetValid);
        _SynthAttr(&writer, "/sample.points", &object.samplePoints,
                   &object.sampleValid);
        geometry.weightObjects.push_back(object);
        RigExecVolumeWeightInputs in =
            volumeInputs("y", "unbounded");
        in.samplePoints = samplePoints;
        expected.push_back(RigExecBuildVolumeWeightPacket(
            TfToken("RigExecSphereWeight"), in));
    }
    // 15: sphere over a CHAIN base (no pathReads entry).
    {
        RigExecWireWeightObject object = _SynthObject(
            &writer, "/w/chainSphere", "RigExecSphereWeight", "dense",
            "clamp");
        object.providerSlot = 0;
        object.falloffMax = _SynthFloat(2.0f);
        _SynthAttr(&writer, "/chainmesh.points",
                   &object.targetPoints, &object.targetValid);
        geometry.weightObjects.push_back(object);
        RigExecVolumeWeightInputs in =
            volumeInputs("y", "unbounded");
        in.targetPoints = samplePoints;
        expected.push_back(RigExecBuildVolumeWeightPacket(
            TfToken("RigExecSphereWeight"), in));
    }
    // 16: sphere over an unreadable target: invalid, tokens kept.
    {
        RigExecWireWeightObject object = _SynthObject(
            &writer, "/w/missing", "RigExecSphereWeight", "dense",
            "clamp");
        object.providerSlot = 0;
        object.falloffMax = _SynthFloat(2.0f);
        _SynthAttr(&writer, "/missing.points",
                   &object.targetPoints, &object.targetValid);
        geometry.weightObjects.push_back(object);
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
        RigExecWireWeightObject object = _SynthObject(
            &writer, "/w/magic", "RigExecMagicWeight", "dense",
            "strict");
        geometry.weightObjects.push_back(object);
        expected.push_back(RigExecWeightPacket());
    }
    // 18-22: the remaining combine modes over objects 0 and 7.
    for (const char *mode :
         {"multiply", "max", "min", "average", "overlay"}) {
        RigExecWireWeightObject object = _SynthObject(
            &writer,
            (std::string("/w/combine-") + mode).c_str(),
            "RigExecCombineWeight", "dense", "strict");
        object.combineMode = writer.AddString(mode);
        object.inputs = {0, 7};
        geometry.weightObjects.push_back(object);
        expected.push_back(RigExecBuildCombineWeightPacket(
            TfToken("dense"), TfToken("strict"), TfToken(mode),
            {expected[0], expected[7]}, 0, 1.0f, 0.0f));
    }
    // 23: dynamic remapping object 1's sparse field (the default
    // remaps; dense would pin zero).
    {
        RigExecWireWeightObject object = _SynthObject(
            &writer, "/w/dynSparse", "RigExecDynamicWeight",
            "sparse", "strict");
        object.base = 1;
        object.driver = _SynthFloat(0.5f);
        geometry.weightObjects.push_back(object);
        RigExecDynamicWeightInputs in;
        in.representation = TfToken("sparse");
        in.rangePolicy = TfToken("strict");
        in.driver = 0.5f;
        expected.push_back(
            RigExecBuildDynamicWeightPacket(in, &expected[1]));
    }
    // 24: static constant with an out-of-range default: invalid.
    {
        RigExecWireWeightObject object = _SynthObject(
            &writer, "/w/constBad", "RigExecStaticWeight",
            "constant", "strict");
        object.defaultWeight = _SynthFloat(2.0f);
        geometry.weightObjects.push_back(object);
        RigExecStaticWeightInputs in;
        in.representation = TfToken("constant");
        in.rangePolicy = TfToken("strict");
        in.defaultWeight = 2.0f;
        expected.push_back(RigExecBuildStaticWeightPacket(in));
    }
    // 25: constant-only combine whose epilogue violates strict: the
    // BARE invalid packet, not the token-carrying one.
    {
        RigExecWireWeightObject object = _SynthObject(
            &writer, "/w/combineBare", "RigExecCombineWeight",
            "dense", "strict");
        object.combineMode = writer.AddString("add");
        object.inputs = {2, 6};
        _SynthAttr(&writer, "/mesh.points",
                   &object.combineTargetPoints,
                   &object.combineTargetValid);
        geometry.weightObjects.push_back(object);
        expected.push_back(RigExecBuildCombineWeightPacket(
            TfToken("dense"), TfToken("strict"), TfToken("add"),
            {expected[2], expected[6]}, 3, 1.0f, 0.0f));
    }
    // 26: sphere over a degenerate band: a hard step at distance 1.
    {
        RigExecWireWeightObject object = _SynthObject(
            &writer, "/w/degenerate", "RigExecSphereWeight",
            "dense", "clamp");
        object.providerSlot = 0;
        object.falloffMin = _SynthFloat(1.0f);
        object.falloffMax = _SynthFloat(1.0f);
        _SynthAttr(&writer, "/mesh.points", &object.targetPoints,
                   &object.targetValid);
        geometry.weightObjects.push_back(object);
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
        RigExecWireWeightObject object = _SynthObject(
            &writer, "/w/planeUnbounded", "RigExecPlaneWeight",
            "dense", "clamp");
        object.providerSlot = 0;
        object.falloffMax = _SynthFloat(2.0f);
        object.planeAxis = writer.AddString("y");
        object.planeBounds = writer.AddString("unbounded");
        object.extentU = _SynthFloat(0.0f);
        object.extentV = _SynthFloat(-1.0f);
        _SynthAttr(&writer, "/mesh.points", &object.targetPoints,
                   &object.targetValid);
        geometry.weightObjects.push_back(object);
        expected.push_back(RigExecBuildVolumeWeightPacket(
            TfToken("RigExecPlaneWeight"),
            volumeInputs("y", "unbounded")));
    }
    // 28: plane with an unknown axis: invalid, tokens kept.
    {
        RigExecWireWeightObject object = _SynthObject(
            &writer, "/w/planeBadAxis", "RigExecPlaneWeight",
            "dense", "clamp");
        object.providerSlot = 0;
        object.falloffMax = _SynthFloat(2.0f);
        object.planeAxis = writer.AddString("w");
        object.planeBounds = writer.AddString("unbounded");
        _SynthAttr(&writer, "/mesh.points", &object.targetPoints,
                   &object.targetValid);
        geometry.weightObjects.push_back(object);
        expected.push_back(RigExecBuildVolumeWeightPacket(
            TfToken("RigExecPlaneWeight"),
            volumeInputs("w", "unbounded")));
    }
    // 29: strict sphere driven out of range by strength: the BARE
    // invalid packet.
    {
        RigExecWireWeightObject object = _SynthObject(
            &writer, "/w/strictBare", "RigExecSphereWeight",
            "dense", "strict");
        object.providerSlot = 0;
        object.falloffMax = _SynthFloat(2.0f);
        object.strength = _SynthFloat(10.0f);
        _SynthAttr(&writer, "/mesh.points", &object.targetPoints,
                   &object.targetValid);
        geometry.weightObjects.push_back(object);
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
        RigExecWireWeightObject object = _SynthObject(
            &writer, "/w/sampleOk", "RigExecSphereWeight", "dense",
            "clamp");
        object.providerSlot = 0;
        object.falloffMax = _SynthFloat(2.0f);
        _SynthAttr(&writer, "/mesh2.points", &object.targetPoints,
                   &object.targetValid);
        _SynthAttr(&writer, "/sample.points", &object.samplePoints,
                   &object.sampleValid);
        geometry.weightObjects.push_back(object);
        RigExecVolumeWeightInputs in =
            volumeInputs("y", "unbounded");
        in.targetPoints = samplePoints;
        in.samplePoints = samplePoints;
        expected.push_back(RigExecBuildVolumeWeightPacket(
            TfToken("RigExecSphereWeight"), in));
    }

    // One chain feeding object 15's target.
    {
        RigExecWireChain chain;
        chain.target = writer.AddString("/chainmesh.points");
        geometry.chains.push_back(chain);
    }

    for (size_t i = 0; i < geometry.weightObjects.size(); ++i) {
        RigExecWireStep step;
        step.kind = RigExecWireStepKind::WeightPacket;
        step.object = int32_t(i);
        step.cluster = 0;
        step.isSource = true;
        char label[32];
        std::snprintf(label, sizeof(label), "weight %zu", i);
        step.label = writer.AddString(label);
        steps.push_back(step);
    }
    {
        RigExecWireStep step;
        step.kind = RigExecWireStepKind::VolumePlacements;
        step.object = 0;
        step.cluster = 0;
        step.isSource = true;
        step.label = writer.AddString("placements");
        steps.push_back(step);
    }

    RigExecWireClustering clustering;
    {
        RigExecWireCluster cluster;
        for (size_t i = 0; i < steps.size(); ++i) {
            cluster.members.push_back(int32_t(i));
        }
        clustering.clusters.push_back(cluster);
        clustering.clusterOf.assign(steps.size(), 0);
    }
    RigExecWireCones cones;
    {
        RigExecWireClusterSet all;
        all.clusters = 1;
        all.words = {1};
        RigExecWireClusterSet none;
        none.clusters = 1;
        none.words = {0};
        cones.cone.push_back(all);
        cones.always = all;
        cones.poseClusters = none;
        cones.avarCluster = {0};
        cones.chainBaseClusters = {{}};
    }

    // Capture visits object 2's default, then object 11's signed scale
    // before its legacy scale. Distinct axes expose a swapped uid mapping.
    inputs.directory.resize(3);
    for (auto &entry : inputs.directory) {
        entry.tag = RigExecWireInput::Tag::Float;
    }
    {
        RigExecWireFrameInputs record;
        record.frame = 1.0;
        record.uids = {0, 1, 2};
        for (float input : {0.3f, 2.0f, 0.5f}) {
            RigExecWireValue value;
            value.tag = RigExecWireInput::Tag::Float;
            value.f32 = input;
            record.values.push_back(value);
        }
        record.pathReads.push_back(
            _SynthPointsRead(&writer, "/signed.points", signedSamples));
        record.pathReads.push_back(
            _SynthPointsRead(&writer, "/mesh.points", meshPoints));
        record.pathReads.push_back(
            _SynthPointsRead(&writer, "/curve.points", curvePoints));
        record.pathReads.push_back(
            _SynthPointsRead(&writer, "/sample.points",
                             samplePoints));
        record.pathReads.push_back(
            _SynthPointsRead(&writer, "/mesh2.points",
                             samplePoints));
        record.chainHaveBase = {1};
        std::vector<RigExecWireVec3f> base;
        for (const GfVec3f &p : samplePoints) {
            base.push_back(RigExecWireVec3f{p[0], p[1], p[2]});
        }
        record.chainBases.push_back(base);
        inputs.frames.push_back(record);
    }

    const std::vector<uint8_t> computed =
        _SynthComputed(&writer, geometry, &inputs);
    writer.AddSection(RigExecBinarySection::Computed, computed);
    std::vector<uint8_t> payload;
    CHECK(RigExecWireEncodeSteps(steps, &payload));
    writer.AddSection(RigExecBinarySection::Steps, payload);
    payload.clear();
    CHECK(RigExecWireEncodeClustering(clustering, &payload));
    writer.AddSection(RigExecBinarySection::Clusters, payload);
    payload.clear();
    CHECK(RigExecWireEncodeCones(cones, &payload));
    writer.AddSection(RigExecBinarySection::Cones, payload);
    payload.clear();
    CHECK(RigExecWireEncodeSlotMeta(slotMeta, &payload));
    writer.AddSection(RigExecBinarySection::SlotMeta, payload);
    payload.clear();
    CHECK(RigExecWireEncodeConstants(constants, &payload));
    writer.AddSection(RigExecBinarySection::Constants, payload);
    payload.clear();
    CHECK(RigExecWireEncodeDomainPose(RigExecWireDomainPose{},
                                      &payload));
    writer.AddSection(RigExecBinarySection::DomainPose, payload);
    payload.clear();
    CHECK(RigExecWireEncodeDomainGeometry(geometry, &payload));
    writer.AddSection(RigExecBinarySection::DomainGeometry, payload);
    payload.clear();
    CHECK(RigExecWireEncodeInputTable(inputs, &payload));
    writer.AddSection(RigExecBinarySection::InputTable, payload);
    const std::vector<uint8_t> bytes = writer.Finish();

    std::string error;
    std::unique_ptr<RigExecRuntimeReader> runtime =
        RigExecRuntimeReader::Open(bytes.data(), bytes.size(), &error);
    CHECK(runtime);
    if (!runtime) {
        std::printf("synthetic open diagnostic: %s\n", error.c_str());
        return;
    }
    // Pose and geometry prologues are siblings' work; the weight walk
    // runs under its own bit. Every weight object field is read through
    // the Computed section, as a bake writes it: a fresh reader plays the
    // slots' defaults.
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
    std::unique_ptr<RigExecBinaryReader> reader =
        RigExecBinaryReader::Open(bytes.data(), bytes.size(), &error);
    CHECK(reader);
    if (!reader) {
        return;
    }
    size_t ok = 0;
    for (size_t i = 0; i < expected.size(); ++i) {
        const std::string path = _WireString(
            reader.get(), geometry.weightObjects[i].path);
        if (_SynthCompare(path.c_str(), expected[i], actual[i],
                          reader.get())) {
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
    CHECK(frames.size() == 1);
    if (frames.size() == 1) {
        CHECK(frames[0].path == "/Volume");
        for (size_t r = 0; r < 4; ++r) {
            for (size_t c = 0; c < 4; ++c) {
                CHECK(frames[0].matrix[r][c] ==
                      expectedPlacement[r][c]);
            }
        }
    }
    std::printf("synthetic: %zu/%zu packets match\n", ok,
                expected.size());

    // Each slot set as an input: the next run builds every packet that
    // reads one from the new value -- object 2's default (0.3 -> 0.6),
    // the combine over it (object 10), and object 11's signed and legacy
    // scales (2 -> 1.5, 0.5 -> 0.75) -- and leaves the others as they were.
    CHECK(runtime->GetInputCount() == 3);
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
            _WireString(reader.get(), geometry.weightObjects[i].path);
        if (_SynthCompare(path.c_str(), moved[i], set[i], reader.get())) {
            ++setOk;
        } else {
            ++failures;
        }
    }
    std::printf("synthetic, slots set as inputs: %zu/%zu packets match\n",
                setOk, moved.size());
}

// The read evaluator and the weight oracle over a hand-built Computed
// section (inputs.cpp, weights.cpp): each read mode's arms, the
// property-chain overlay, and the oracle's arithmetic and error text on
// static, dynamic, combine and volume paths the baked fixtures do not
// reach. Expected fields repeat the oracle's float expressions in its
// order, with operands for which a reordering would change the bits.
static void
_TestComputedReadsAndOracle()
{
    using v4::InputTag;
    using v4::ReadMode;
    RigExecBinaryWriter writer;
    std::vector<uint32_t> paths;
    for (int i = 0; i <= 16; ++i) {
        paths.push_back(writer.AddString("/W" + std::to_string(i)));
    }
    std::vector<uint32_t> slotNames;
    for (int i = 0; i < 5; ++i) {
        slotNames.push_back(writer.AddString("/S" + std::to_string(i) + ".v"));
    }
    const uint32_t tokStatic = writer.AddString("RigExecStaticWeight");
    const uint32_t tokDynamic = writer.AddString("RigExecDynamicWeight");
    const uint32_t tokCombine = writer.AddString("RigExecCombineWeight");
    const uint32_t tokSphere = writer.AddString("RigExecSphereWeight");
    const uint32_t tokConstant = writer.AddString("constant");
    const uint32_t tokDense = writer.AddString("dense");
    const uint32_t tokSparse = writer.AddString("sparse");
    const uint32_t tokStrict = writer.AddString("strict");
    const uint32_t tokClamp = writer.AddString("clamp");
    const uint32_t tokAdd = writer.AddString("add");
    const uint32_t tokMax = writer.AddString("max");
    const uint32_t tokMultiply = writer.AddString("multiply");
    const uint32_t tokSubtract = writer.AddString("subtract");
    const uint32_t tokMin = writer.AddString("min");
    const uint32_t tokAverage = writer.AddString("average");
    const uint32_t tokOverlay = writer.AddString("overlay");
    const std::vector<uint8_t> bytes = writer.Finish();
    std::string error;
    const std::unique_ptr<RigExecBinaryReader> strings =
        RigExecBinaryReader::Open(bytes.data(), bytes.size(), &error);
    CHECK(strings);
    if (!strings) {
        return;
    }

    RigExecWireComputed computed;
    const auto value = [&](InputTag tag, uint64_t bits) {
        v4::RigExecWireValue v;
        v.tag = tag;
        v.bits = bits;
        computed.values.push_back(v);
        return uint32_t(computed.values.size() - 1);
    };
    const auto floatValue = [&](float f) {
        uint32_t bits = 0;
        std::memcpy(&bits, &f, sizeof(bits));
        return value(InputTag::Float, bits);
    };
    const auto doubleValue = [&](double d) {
        uint64_t bits = 0;
        std::memcpy(&bits, &d, sizeof(bits));
        return value(InputTag::Double, bits);
    };
    doubleValue(0.0);
    // Slots: s0 float 0.25, s1 double 0.7, s2 float with no value, s3 int,
    // s4 double with no value.
    const InputTag slotTypes[] = {InputTag::Float, InputTag::Double,
                                  InputTag::Float, InputTag::Int,
                                  InputTag::Double};
    RigExecWireComputedFrame frame;
    frame.values = {floatValue(0.25f), doubleValue(0.7), floatValue(0.0f),
                    value(InputTag::Int, 3), doubleValue(0.0)};
    frame.hasValue = {1, 1, 0, 1, 0};
    for (size_t s = 0; s < 5; ++s) {
        v4::InputSlot slot;
        slot.name = slotNames[s];
        slot.type = slotTypes[s];
        slot.value = frame.values[s];
        slot.flags = uint8_t(v4::InputSlotFlags::Listed) |
                     (frame.hasValue[s]
                          ? uint8_t(v4::InputSlotFlags::HasValue)
                          : uint8_t(0));
        computed.inputs.push_back(slot);
    }
    computed.listedInputs = 5;
    computed.frames.push_back(frame);
    computed.vec3fArrays.emplace_back();

    const auto read = [&](float constant, ReadMode mode, uint8_t flags,
                          std::vector<uint32_t> walk, int16_t selected = -1,
                          int32_t overrideIndex = -1) {
        v4::RigExecWireInput input;
        input.tag = InputTag::Float;
        input.mode = mode;
        input.flags = flags;
        input.constant = floatValue(constant);
        input.walk = std::move(walk);
        input.selected = selected;
        input.overrideIndex = overrideIndex;
        return input;
    };
    // A read whose one-slot walk holds \p v at the frame: the oracle reads
    // a read's walk, and falls back to its own read-site constant, never
    // the read's, when the walk yields nothing.
    const auto fixed = [&](float v) {
        v4::InputSlot slot;
        slot.type = InputTag::Float;
        slot.value = floatValue(v);
        slot.flags = uint8_t(v4::InputSlotFlags::Listed) |
                     uint8_t(v4::InputSlotFlags::HasValue);
        computed.inputs.push_back(slot);
        computed.listedInputs = uint32_t(computed.inputs.size());
        computed.frames[0].values.push_back(slot.value);
        computed.frames[0].hasValue.push_back(1);
        return read(v, ReadMode::Resolved, 0,
                    {uint32_t(computed.inputs.size() - 1)});
    };
    const auto object = [&](size_t index, uint32_t type,
                            uint32_t representation, uint32_t rangePolicy) {
        v4::RigExecWireWeightObject o;
        o.path = paths[index];
        o.type = type;
        o.representation = representation;
        o.rangePolicy = rangePolicy;
        for (v4::RigExecWireInput *input :
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
    std::vector<v4::RigExecWireWeightObject> &objects =
        computed.weightObjects;
    objects.push_back(object(0, tokStatic, tokConstant, tokStrict));
    objects[0].defaultWeight = fixed(0.5f);
    objects.push_back(object(1, tokStatic, tokDense, tokClamp));
    objects[1].values = {0.25f, 1.5f};
    objects.push_back(object(2, tokStatic, tokSparse, tokStrict));
    objects[2].indices = {1, 1};
    objects[2].values = {0.1f, 0.2f};
    objects.push_back(object(3, tokStatic, tokSparse, tokStrict));
    objects[3].indices = {2, 0};
    objects[3].values = {0.2f, 0.1f};
    objects.push_back(object(4, tokDynamic, tokSparse, tokStrict));
    objects[4].base = 3;
    objects[4].driver = read(1.0f, ReadMode::Resolved, 0, {1});
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

    RigExecWireDomainPose poses;
    RigExecWireDomainGeometry geometry;
    RrProgram program;
    program.poses = &poses;
    program.geometry = &geometry;
    program.strings = strings.get();
    CHECK(RrInputsOpen(&program, &computed, &error));
    CHECK(RrWeightSizeScratch(&program, &error));
    program.store.overridden.assign(4, 0);

    // Reads, mode by mode.
    const auto f = [&](const v4::RigExecWireInput &input) {
        return RrWireValueFloat(RrReadInput(&program, input));
    };
    const float narrowed = static_cast<float>(0.7);
    const uint8_t varying = uint8_t(v4::InputReadFlags::Varying);
    const uint8_t longWay = uint8_t(v4::InputReadFlags::LongWay);
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
    const auto publish = [&](size_t slot, RrPropertyValue::Tag tag,
                             double v) {
        RrPropertyValue held;
        held.tag = tag;
        held.f32 = float(v);
        held.f64 = v;
        overlay[slotNames[slot]] = held;
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
    overlay.clear();
    // A double hop's own double result, narrowed.
    publish(1, RrPropertyValue::Tag::Double, 0.3);
    CHECK(f(read(9.0f, ReadMode::Resolved, 0, {0, 1})) ==
          static_cast<float>(0.3));
    overlay.clear();
    // GetAttribute<float> tests its own head for a double before the
    // overlay, so a float result at a double head is not read.
    publish(1, RrPropertyValue::Tag::Float, 0.3);
    CHECK(f(read(9.0f, ReadMode::Resolved, 0, {1})) == narrowed);
    overlay.clear();

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
    overlay.clear();
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
    program.store.weightFrames[paths[8]] = RrMat4d(1.0);
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
          error == "the computed section holds no weight object " +
                       std::to_string(objects.size()));
    std::printf("computed reads and oracle: checked\n");
}

int
main(int argc, char **argv)
{
    PlugRegistry::GetInstance().RegisterPlugins(
        RIGEXEC_SCHEMA_RESOURCE_DIR);
    _TestSyntheticBuilders();
    _TestComputedReadsAndOracle();

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
