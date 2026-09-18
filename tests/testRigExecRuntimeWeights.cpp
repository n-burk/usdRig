//
// rigExecRuntime weight-family parity (M2): baked program vs runtime over
// every baking fixture, comparing weight packets bit for bit, plus
// synthetic builder checks against the real baked builders.
//
// Fixture strategy per fixture: bake in-process, then run a fresh
// evaluator (Evaluate) and a fresh reader (SetFrame + Execute) frame by
// frame in order. Mask 0x7 is tried first; when Execute fails with "not
// implemented yet" (pose or geometry still landing) the fixture falls
// back to mask 0x2 and compares pose-independent packets only: volume
// packets read the pose base frames, so volumetric objects -- and
// anything composed over them -- are deferred, with weightFrames and
// diagnostics, until pose lands. In 0x7 mode weightFrames and
// diagnostics compare verbatim too.
//

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

#include <cstdio>
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
_IsVolumeType(const std::string &type)
{
    return type == "RigExecSphereWeight" || type == "RigExecPlaneWeight" ||
           type == "RigExecCurveWeight";
}

// Objects whose packets read pose state and so cannot compare while
// pose is masked out: volumetric objects, and anything composed over
// a deferred object. The table is in dependency order, so one forward
// pass settles it.
static std::vector<char>
_DeferredWithoutPose(
    RigExecBinaryReader *reader,
    const std::vector<RigExecWireWeightObject> &objects)
{
    std::vector<char> deferred(objects.size(), 0);
    for (size_t i = 0; i < objects.size(); ++i) {
        const RigExecWireWeightObject &wire = objects[i];
        if (_IsVolumeType(_WireString(reader, wire.type))) {
            deferred[i] = 1;
            continue;
        }
        if (wire.base >= 0 && size_t(wire.base) < deferred.size() &&
            deferred[size_t(wire.base)]) {
            deferred[i] = 1;
            continue;
        }
        for (int32_t input : wire.inputs) {
            if (input >= 0 && size_t(input) < deferred.size() &&
                deferred[size_t(input)]) {
                deferred[i] = 1;
                break;
            }
        }
    }
    return deferred;
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

// Runs one fixture at one mask with a fresh evaluator and reader,
// frames in order. Weight steps emit no diagnostics of their own, so a
// diagnostics mismatch in 0x7 mode is sibling drift, reported as such.
static bool
_RunFixtureAtMask(const std::string &stagePath,
                  const std::vector<double> &frames,
                  const std::vector<uint8_t> &bytes,
                  RigExecBinaryReader *reader,
                  const RigExecWireDomainGeometry &geometry,
                  unsigned mask, bool compareFrames,
                  size_t *compared, size_t *deferredCount)
{
    const UsdStageRefPtr stage = UsdStage::Open(stagePath);
    if (!stage) {
        return false;
    }
    RigExecRigEvaluator evaluator(stage, _FindRig(stage));
    evaluator.SetEvaluationMode(RigExecEvaluationMode::Baked);
    std::string error;
    std::unique_ptr<RigExecRuntimeReader> runtime =
        RigExecRuntimeReader::Open(bytes.data(), bytes.size(), &error);
    if (!runtime) {
        std::printf("  open diagnostic: %s\n", error.c_str());
        return false;
    }
    runtime->SetRunMaskForTesting(mask);
    const std::vector<char> deferred =
        mask == 0x7u ? std::vector<char>(geometry.weightObjects.size(),
                                         0)
                     : _DeferredWithoutPose(
                           reader, geometry.weightObjects);
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
        if (!runtime->SetFrame(frame, &error)) {
            std::printf("  SetFrame @ %g: %s\n", frame,
                        error.c_str());
            return false;
        }
        if (!runtime->Execute(&error)) {
            std::printf("  Execute @ %g mask=0x%x: %s\n", frame, mask,
                        error.c_str());
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
            if (deferred[i]) {
                continue;
            }
            ++(*compared);
            if (!_ComparePacket(reader, geometry.weightObjects[i],
                                bakedPackets[i], actualPackets[i],
                                frame)) {
                ok = false;
            }
        }
        if (compareFrames) {
            if (!_CompareWeightFrames(runtime->GetWeightFrames(),
                                      pose.weightFrames, frame)) {
                ok = false;
            }
            if (!_CompareDiagnostics(runtime->GetDiagnostics(),
                                     pose.diagnostics, frame)) {
                ok = false;
            }
        }
    }
    for (char d : deferred) {
        *deferredCount += d ? frames.size() : 0;
    }
    return ok;
}

static void
_TestFixture(const std::string &name, const std::string &stagePath,
             const std::vector<double> &frames)
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
    RigExecRigEvaluator evaluator(stage, rigPath);
    evaluator.SetEvaluationMode(RigExecEvaluationMode::Baked);
    RigExecBakeOpts opts;
    opts.frames = frames;
    RigExecBakeResult result;
    std::string error;
    CHECK(RigExecBakeToBinary(evaluator, opts, &result, &error));
    if (!error.empty()) {
        std::printf("bake diagnostic: %s\n", error.c_str());
    }
    CHECK(!result.bytes.empty());
    if (result.bytes.empty()) {
        return;
    }
    std::unique_ptr<RigExecBinaryReader> reader;
    RigExecWireDomainGeometry geometry;
    CHECK(_DecodeWeightWire(result.bytes, &reader, &geometry, &error));
    if (!reader) {
        std::printf("wire decode diagnostic: %s\n", error.c_str());
        return;
    }

    // Full mask first: weightFrames and diagnostics compare only when
    // every family runs.
    size_t compared = 0, deferredCount = 0;
    if (_RunFixtureAtMask(stagePath, frames, result.bytes, reader.get(),
                          geometry, 0x7u, true, &compared,
                          &deferredCount)) {
        std::printf("%s: mask=0x7 compared=%zu deferred=%zu "
                    "weightFrames=yes diagnostics=yes OK\n",
                    name.c_str(), compared, deferredCount);
        return;
    }
    // Fall back only across an unlanded sibling: a weight failure of
    // our own must fail the fixture, not hide behind the mask. The
    // probe walks every frame, so a partially landed sibling still
    // reads as a gap rather than a weight failure.
    compared = deferredCount = 0;
    std::unique_ptr<RigExecRuntimeReader> probe =
        RigExecRuntimeReader::Open(result.bytes.data(),
                                   result.bytes.size(), &error);
    bool siblingMissing = false;
    if (probe) {
        probe->SetRunMaskForTesting(0x7u);
        for (double frame : frames) {
            if (!probe->SetFrame(frame, &error) ||
                probe->Execute(&error)) {
                continue;
            }
            if (error.find("not implemented yet") !=
                std::string::npos) {
                siblingMissing = true;
            }
            break;
        }
    }
    if (!siblingMissing) {
        ++failures;
        std::printf("%s: mask=0x7 FAILED (not a sibling gap)\n",
                    name.c_str());
        return;
    }
    if (_RunFixtureAtMask(stagePath, frames, result.bytes, reader.get(),
                          geometry, 0x2u, false, &compared,
                          &deferredCount)) {
        std::printf("%s: mask=0x2 compared=%zu deferred=%zu "
                    "weightFrames=deferred diagnostics=deferred OK\n",
                    name.c_str(), compared, deferredCount);
        return;
    }
    ++failures;
    std::printf("%s: mask=0x2 FAILED\n", name.c_str());
}

// ---------------------------------------------------------------------------
// Synthetic builder checks: a hand-built wire program exercising every
// builder arm against the real baked builders, including the pathReads
// and chain-base gather layers no fixture populates for weight
// attributes, and the invalid-packet shapes.
// ---------------------------------------------------------------------------

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
    object.curvenetSamples = _SynthFloat(0.0f);
    object.curvenetSamples.tag = RigExecWireInput::Tag::Int;
    object.curvenetSamples.i32 = 5;
    object.curvenetUnreached = _SynthFloat(0.0f);
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
    // 11: sphere over pathReads points, smooth remap LUT.
    {
        RigExecWireWeightObject object = _SynthObject(
            &writer, "/w/sphere", "RigExecSphereWeight", "dense",
            "clamp");
        object.providerSlot = 0;
        object.falloffMax = _SynthFloat(2.0f);
        object.falloffCurve =
            RigExecBuildFalloffLut(RigExecFalloffProfile::Smooth);
        _SynthAttr(&writer, "/mesh.points", &object.targetPoints,
                   &object.targetValid);
        geometry.weightObjects.push_back(object);
        RigExecVolumeWeightInputs sphereIn =
            volumeInputs("y", "unbounded");
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
    // 17: curvenet with an invalid basis: invalid, no bind attempted.
    {
        RigExecWireWeightObject object = _SynthObject(
            &writer, "/w/curvenetBadBasis", "RigExecCurvenetWeight",
            "dense", "strict");
        object.curvenetBasis = writer.AddString("linear");
        geometry.weightObjects.push_back(object);
        RigExecWeightPacket packet;
        packet.representation = TfToken("dense");
        packet.rangePolicy = TfToken("strict");
        expected.push_back(packet);
    }
    // 18: unknown weight type: bare invalid packet.
    {
        RigExecWireWeightObject object = _SynthObject(
            &writer, "/w/magic", "RigExecMagicWeight", "dense",
            "strict");
        geometry.weightObjects.push_back(object);
        expected.push_back(RigExecWeightPacket());
    }
    // 19-23: the remaining combine modes over objects 0 and 7.
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
    // 24: dynamic remapping object 1's sparse field (the default
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
    // 25: static constant with an out-of-range default: invalid.
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
    // 26: constant-only combine whose epilogue violates strict: the
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
    // 27: sphere over a degenerate band: a hard step at distance 1.
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
    // 28: unbounded plane with bad extents: valid, the extents are
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
    // 29: plane with an unknown axis: invalid, tokens kept.
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
    // 30: strict sphere driven out of range by strength: the BARE
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
    // 31: sphere with a matching sample override: the samples, not
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

    // The uid directory holds the one varying input (object 2's
    // default): ladders, interpolators, solvers and constraints are
    // absent, so the weight walk assigns uid 0.
    inputs.directory.resize(1);
    inputs.directory[0].tag = RigExecWireInput::Tag::Float;
    {
        RigExecWireFrameInputs record;
        record.frame = 1.0;
        record.uids = {0};
        RigExecWireValue value;
        value.tag = RigExecWireInput::Tag::Float;
        value.f32 = 0.3f;
        record.values.push_back(value);
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
    // runs under its own bit.
    runtime->SetRunMaskForTesting(0x2u);
    CHECK(runtime->SetFrame(1.0, &error));
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
}

int
main(int argc, char **argv)
{
    PlugRegistry::GetInstance().RegisterPlugins(
        RIGEXEC_SCHEMA_RESOURCE_DIR);
    _TestSyntheticBuilders();

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
                     frames);
    }
    CHECK(sawBaking);

    if (failures == 0) {
        std::printf("testRigExecRuntimeWeights: all tests passed\n");
        return 0;
    }
    std::printf("testRigExecRuntimeWeights: %d failures\n", failures);
    return 1;
}
