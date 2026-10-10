#include "rigExecMath/geometryKernels.h"
#include "rigExec/rigEvaluator.h"
#include "rigExecRigging/rigBuilder.h"
#include "rigExecBake/bake.h"
#include "rigExecRuntime/runtime.h"
#include "pxr/base/plug/registry.h"
#include "pxr/usd/sdf/layer.h"
#include "pxr/usd/usdGeom/mesh.h"
#include "pxr/usd/usdGeom/points.h"

#include <algorithm>
#include <cmath>
#include <cstring>
#include <fstream>
#include <iostream>
#include <numeric>
#include <random>
#include <stdexcept>
#include <string>
#include <vector>

using namespace rigExec;
PXR_NAMESPACE_USING_DIRECTIVE

#define CHECK(x) do { if (!(x)) throw std::runtime_error( \
    std::string(__FILE__) + ":" + std::to_string(__LINE__) + ": " #x); } while (false)

#include "rigExecRuntimeDrive.h"

namespace {

using Points = std::vector<GfVec3f>;
const SdfPath kRig("/Rig"), kTarget("/Rig/Body.points");

struct Grid {
    Points rest;
    std::vector<int> counts, indices;
    Grid() {
        constexpr int nx = 13, ny = 7;
        for (int y = 0; y < ny; ++y)
            for (int x = 0; x < nx; ++x)
                rest.emplace_back(0.125f * x, 0.2f * y, 0.0f);
        for (int y = 0; y + 1 < ny; ++y) {
            for (int x = 0; x + 1 < nx; ++x) {
                const int a = y * nx + x;
                counts.push_back(4);
                indices.insert(indices.end(), {a, a + 1, a + 1 + nx, a + nx});
            }
        }
    }
    Points Compressed(float scale = 0.65f) const {
        auto points = rest;
        for (auto &p : points) p[0] *= scale;
        return points;
    }
};

bool Near(const Points &a, const Points &b, double tolerance = 1e-5) {
    if (a.size() != b.size()) return false;
    for (size_t i = 0; i < a.size(); ++i)
        if (!((a[i] - b[i]).GetLength() <= tolerance)) return false;
    return true;
}

bool SameBits(const Points &a, const Points &b) {
    if (a.size() != b.size()) return false;
    for (size_t i = 0; i < a.size(); ++i)
        if (std::memcmp(a[i].data(), b[i].data(), 3 * sizeof(float))) return false;
    return true;
}

std::vector<size_t> PlaybackOrder(size_t count) {
    std::vector<size_t> forward(count);
    std::iota(forward.begin(), forward.end(), size_t(0));
    auto order = forward;
    order.insert(order.end(), forward.rbegin(), forward.rend());
    std::mt19937 random(617);
    std::shuffle(forward.begin(), forward.end(), random);
    order.insert(order.end(), forward.begin(), forward.end());
    return order;
}

bool Contains(const std::vector<std::string> &values, const std::string &text) {
    for (const auto &value : values)
        if (value.find(text) != std::string::npos) return true;
    return false;
}

std::string LayerText(const SdfLayerHandle &layer) {
    std::string text;
    CHECK(layer->ExportToString(&text));
    return text;
}

Points Published(const RigExecRigPose &pose) {
    CHECK(pose.valid);
    CHECK(pose.comparisonMismatches == 0);
    CHECK(pose.referenceMismatches == 0);
    CHECK(pose.movedProperties.count(kTarget) == 1);
    const auto &value = pose.movedProperties.at(kTarget).Get<VtVec3fArray>();
    return {value.begin(), value.end()};
}

void Compile(RigExecRigEvaluator &evaluator) {
    std::vector<std::string> diagnostics;
    if (!evaluator.Compile(&diagnostics)) {
        for (const auto &message : diagnostics) std::cerr << message << '\n';
        CHECK(false);
    }
}

UsdGeomMesh AddMesh(const UsdStageRefPtr &stage, const Grid &grid,
                    const Points &points) {
    auto mesh = UsdGeomMesh::Define(stage, kTarget.GetPrimPath());
    CHECK(mesh.CreatePointsAttr().Set(VtVec3fArray(points.begin(), points.end())));
    CHECK(mesh.CreateFaceVertexCountsAttr().Set(
        VtIntArray(grid.counts.begin(), grid.counts.end())));
    CHECK(mesh.CreateFaceVertexIndicesAttr().Set(
        VtIntArray(grid.indices.begin(), grid.indices.end())));
    return mesh;
}

void CheckRuntime(const RigExecRuntimeReader &reader, const Points &expected) {
    bool found = false;
    for (const auto &actual : reader.GetPoints()) {
        if (actual.path != kTarget.GetString()) continue;
        found = true;
        CHECK(actual.points.size() == expected.size());
        for (size_t i = 0; i < expected.size(); ++i)
            CHECK(std::memcmp(actual.points[i].data(), expected[i].data(),
                              3 * sizeof(float)) == 0);
    }
    CHECK(found);
}

// Bakes \p frames' first and plays the file through its inputs in
// PlaybackOrder (forward, backward, shuffled; a time played twice in a row
// samples nothing the second time), each run bit for bit with the
// evaluator's points at that time.
std::vector<uint8_t> CheckBinaryParity(RigExecRigEvaluator &evaluator,
                                      const UsdStageRefPtr &stage,
                                      const std::vector<double> &frames) {
    std::vector<Points> expected;
    for (double frame : frames)
        expected.push_back(Published(evaluator.Evaluate(UsdTimeCode(frame))));
    RigExecBakeOpts options;
    options.time = frames.front();
    RigExecBakeResult result;
    std::string error;
    if (!RigExecBakeToBinary(evaluator, options, &result, &error))
        throw std::runtime_error(error);
    CHECK(!result.bytes.empty());
    RigExecTestPlayer player;
    if (!player.Open(result.bytes, stage, &error))
        throw std::runtime_error(error);
    CHECK(player->GetBakeTime() == frames.front());
    for (size_t index : PlaybackOrder(frames.size())) {
        CHECK(SameBits(Published(evaluator.Evaluate(UsdTimeCode(frames[index]))),
                       expected[index]));
        if (!player.Play(frames[index], &error)) throw std::runtime_error(error);
        CheckRuntime(player.Reader(), expected[index]);
    }
    return result.bytes;
}

Points Solve(const Grid &grid, const Points &posed,
             const RigExecWrinkleSettings &settings = {}) {
    auto result = posed;
    CHECK(RigExecApplyWrinkle(&result, grid.rest, grid.counts, grid.indices, settings));
    return result;
}

void SetSettings(RigExecWrinkleMoverHandle &wrinkle,
                 const RigExecWrinkleSettings &settings) {
    wrinkle.SetIterations(settings.iterations);
    wrinkle.SetTopology(TfToken(settings.topology == RigExecWrinkleTopology::Cloth
        ? "cloth" : "surfaceStruts"));
    wrinkle.SetNeighborDistance(settings.neighborDistance);
    wrinkle.SetRestLengthScale(settings.restLengthScale);
    wrinkle.SetStretchStiffness(settings.stretchStiffness);
    wrinkle.SetCompressionStiffness(settings.compressionStiffness);
    wrinkle.SetBendStiffness(settings.bendStiffness);
    wrinkle.SetMaxDisplacement(settings.maxDisplacement);
    wrinkle.SetPinBorders(settings.pinBorders);
    wrinkle.SetPinPoints(settings.pinPoints);
    wrinkle.SetTangentPlaneCollisions(settings.tangentPlaneCollisions);
    wrinkle.SetTangentPlaneInset(settings.tangentPlaneInset);
    wrinkle.SetWrinkleScale(settings.wrinkleScale);
    wrinkle.SetSmoothingIterations(settings.smoothingIterations);
}

void TestEvaluationAndInvalidation(const Grid &grid) {
    const auto stage = UsdStage::CreateInMemory();
    auto builder = RigExecRigBuilder::Create(stage, kRig);
    const auto posed = grid.Compressed();
    const auto mesh = AddMesh(stage, grid, posed);
    auto wrinkle = builder.NewMoverChain("Deform", kTarget).AddWrinkleMover("Wrinkle");
    wrinkle.SetRestPoints(grid.rest);
    CHECK(wrinkle.GetPrim().GetTypeName() == TfToken("RigExecWrinkleMover"));
    const auto full = Solve(grid, posed);
    CHECK(!Near(full, posed, 0.001));

    const auto rootBefore = LayerText(stage->GetRootLayer());
    const auto sessionBefore = LayerText(stage->GetSessionLayer());
    RigExecRigEvaluator evaluator(stage, kRig);
    Compile(evaluator);
    std::vector<std::string> diagnostics;
    CHECK(evaluator.IsBakeable(&diagnostics));
    auto evaluate = [&]() { return Published(evaluator.Evaluate(UsdTimeCode::Default())); };
    CHECK(Near(evaluate(), full));
    CHECK(evaluator.GetBakedProgram() != nullptr);
    CHECK(Near(evaluate(), full));
    CHECK(LayerText(stage->GetRootLayer()) == rootBefore);
    CHECK(LayerText(stage->GetSessionLayer()) == sessionBefore);

    {
        RigExecRigEvaluator reference(stage, kRig);
        reference.cpuReference = true;
        Compile(reference);
        const auto pose = reference.Evaluate(UsdTimeCode::Default());
        CHECK(Near(Published(pose), full));
        CHECK(pose.referenceAgreements > 0);
    }

    wrinkle.SetDefaultWeight(0.5f);
    auto halfway = posed;
    for (size_t i = 0; i < posed.size(); ++i)
        halfway[i] = posed[i] + 0.5f * (full[i] - posed[i]);
    CHECK(Near(evaluate(), halfway));
    wrinkle.SetDefaultWeight(0.0f);
    CHECK(Near(evaluate(), posed));
    wrinkle.SetDefaultWeight(1.0f);
    wrinkle.SetEnabled(false);
    CHECK(Near(evaluate(), posed));
    wrinkle.SetEnabled(true);
    wrinkle.SetWrinkleScale(0.0f);
    CHECK(Near(evaluate(), posed));
    wrinkle.SetWrinkleScale(1.0f);
    wrinkle.SetIterations(0);
    CHECK(Near(evaluate(), posed));
    wrinkle.SetIterations(80);
    wrinkle.SetMaxDisplacement(0.0f);
    CHECK(Near(evaluate(), posed));
    wrinkle.SetMaxDisplacement(0.2f);
    CHECK(Near(evaluate(), full));

    std::vector<float> mask(posed.size());
    for (size_t i = 0; i < mask.size(); ++i) mask[i] = float(i % 3) * 0.5f;
    const auto weights = builder.AddStaticWeight("Mask", kTarget, mask);
    wrinkle.SetWeightObject(weights.GetPath());
    auto masked = posed;
    for (size_t i = 0; i < posed.size(); ++i)
        masked[i] += mask[i] * (full[i] - posed[i]);
    CHECK(Near(evaluate(), masked));
    wrinkle.SetWeightObject({});

    auto driver = stage->GetPrimAtPath(kRig).CreateAttribute(
        TfToken("wrinkleScale"), SdfValueTypeNames->Float);
    CHECK(driver.Set(0.4f));
    CHECK(wrinkle.GetPrim().GetAttribute(TfToken("inputs:wrinkleScale"))
        .SetConnections({driver.GetPath()}));
    RigExecWrinkleSettings connected;
    connected.wrinkleScale = 0.4f;
    CHECK(Near(evaluate(), Solve(grid, posed, connected)));
    CHECK(driver.Set(1.2f));
    connected.wrinkleScale = 1.2f;
    CHECK(Near(evaluate(), Solve(grid, posed, connected)));
    CHECK(wrinkle.GetPrim().GetAttribute(TfToken("inputs:wrinkleScale")).ClearConnections());

    RigExecWrinkleSettings settings;
    settings.iterations = 50;
    settings.neighborDistance = 3;
    settings.stretchStiffness = 0.8f;
    settings.compressionStiffness = 0.7f;
    settings.bendStiffness = 0.05f;
    settings.maxDisplacement = 0.12f;
    settings.pinPoints = {45};
    settings.tangentPlaneInset = 0.03f;
    settings.wrinkleScale = 0.8f;
    settings.smoothingIterations = 2;
    SetSettings(wrinkle, settings);
    const auto tuned = Solve(grid, posed, settings);
    CHECK(Near(evaluate(), tuned));
    CHECK(!Near(tuned, full));
    CHECK(tuned[45] == posed[45]);

    settings.topology = RigExecWrinkleTopology::SurfaceStruts;
    wrinkle.SetTopology(TfToken("surfaceStruts"));
    CHECK(Near(evaluate(), Solve(grid, posed, settings)));
    settings.topology = RigExecWrinkleTopology::Cloth;
    wrinkle.SetTopology(TfToken("cloth"));
    CHECK(Near(evaluate(), tuned));

    settings.pinPoints = {32, 45, 58};
    wrinkle.SetPinPoints(settings.pinPoints);
    const auto pinned = Solve(grid, posed, settings);
    CHECK(Near(evaluate(), pinned));
    for (const int index : settings.pinPoints) CHECK(pinned[index] == posed[index]);

    // Changing the live target points and static rest both invalidate the kernel inputs.
    auto lessCompressed = grid.Compressed(0.8f);
    CHECK(mesh.GetPointsAttr().Set(VtVec3fArray(lessCompressed.begin(), lessCompressed.end())));
    const auto revised = Solve(grid, lessCompressed, settings);
    CHECK(Near(evaluate(), revised));
    wrinkle.SetRestPoints(lessCompressed);
    CHECK(Near(evaluate(), lessCompressed));
    wrinkle.SetRestPoints(grid.rest);
    CHECK(Near(evaluate(), revised));

    const auto text = LayerText(stage->GetRootLayer());
    const auto layer = SdfLayer::CreateAnonymous("wrinkle_roundtrip.usda");
    CHECK(layer->ImportFromString(text));
    const auto reopened = UsdStage::Open(layer);
    RigExecRigEvaluator reloaded(reopened, kRig);
    Compile(reloaded);
    CHECK(Near(Published(reloaded.Evaluate(UsdTimeCode::Default())), revised));

    // Malformed data leaves the entire preceding revision intact with a diagnostic.
    CHECK(wrinkle.GetPrim().GetAttribute(TfToken("inputs:iterations")).Set(-1));
    auto invalid = evaluator.Evaluate(UsdTimeCode::Default());
    CHECK(Near(Published(invalid), lessCompressed));
    CHECK(Contains(invalid.diagnostics, "MoverFailed"));
    wrinkle.SetIterations(settings.iterations);
    wrinkle.SetRestPoints({grid.rest.front()});
    invalid = evaluator.Evaluate(UsdTimeCode::Default());
    CHECK(Near(Published(invalid), lessCompressed));
    CHECK(Contains(invalid.diagnostics, "MoverFailed"));
    wrinkle.SetRestPoints(grid.rest);
    auto badIndices = grid.indices;
    badIndices.back() = int(posed.size());
    CHECK(mesh.GetFaceVertexIndicesAttr().Set(VtIntArray(badIndices.begin(), badIndices.end())));
    invalid = evaluator.Evaluate(UsdTimeCode::Default());
    CHECK(Near(Published(invalid), lessCompressed));
    CHECK(Contains(invalid.diagnostics, "MoverFailed"));
    CHECK(mesh.GetFaceVertexIndicesAttr().Set(VtIntArray(grid.indices.begin(), grid.indices.end())));
    CHECK(Near(evaluate(), revised));

    // A valid connectivity edit changes the cloth constraints without changing points.
    Grid triangulated = grid;
    triangulated.counts.clear();
    triangulated.indices.clear();
    for (size_t i = 0; i < grid.indices.size(); i += 4) {
        const int a = grid.indices[i], b = grid.indices[i + 1];
        const int c = grid.indices[i + 2], d = grid.indices[i + 3];
        triangulated.counts.insert(triangulated.counts.end(), {3, 3});
        triangulated.indices.insert(triangulated.indices.end(), {a, b, c, a, c, d});
    }
    CHECK(mesh.GetFaceVertexCountsAttr().Set(
        VtIntArray(triangulated.counts.begin(), triangulated.counts.end())));
    CHECK(mesh.GetFaceVertexIndicesAttr().Set(
        VtIntArray(triangulated.indices.begin(), triangulated.indices.end())));
    CHECK(Near(evaluate(), Solve(triangulated, lessCompressed, settings)));
}

void TestStaticInputValidation(const Grid &grid) {
    for (const char *name : {"restPoints", "pinPoints", "topology"}) {
        for (bool connection : {false, true}) {
            const auto stage = UsdStage::CreateInMemory();
            auto builder = RigExecRigBuilder::Create(stage, kRig);
            AddMesh(stage, grid, grid.Compressed());
            auto wrinkle = builder.NewMoverChain("Deform", kTarget).AddWrinkleMover("Wrinkle");
            const auto attr = wrinkle.GetPrim().GetAttribute(
                TfToken(std::string("inputs:") + name));
            VtValue value;
            if (std::string(name) == "restPoints")
                value = VtValue(VtVec3fArray(grid.rest.begin(), grid.rest.end()));
            else if (std::string(name) == "pinPoints") value = VtValue(VtIntArray{45});
            else value = VtValue(TfToken("cloth"));
            if (connection) {
                const auto source = stage->GetPrimAtPath(kRig).CreateAttribute(
                    TfToken("staticInputDriver"), attr.GetTypeName());
                CHECK(source.Set(value));
                CHECK(attr.SetConnections({source.GetPath()}));
            } else CHECK(attr.Set(value, UsdTimeCode(1)));
            RigExecRigEvaluator evaluator(stage, kRig);
            std::vector<std::string> diagnostics;
            CHECK(!evaluator.Compile(&diagnostics));
            CHECK(Contains(diagnostics, name));
            CHECK(Contains(diagnostics, "static"));
        }
    }
    const auto stage = UsdStage::CreateInMemory();
    auto builder = RigExecRigBuilder::Create(stage, kRig);
    auto points = UsdGeomPoints::Define(stage, kTarget.GetPrimPath());
    CHECK(points.CreatePointsAttr().Set(VtVec3fArray(grid.rest.begin(), grid.rest.end())));
    builder.NewMoverChain("Deform", kTarget).AddWrinkleMover("Wrinkle");
    RigExecRigEvaluator evaluator(stage, kRig);
    std::vector<std::string> diagnostics;
    CHECK(!evaluator.Compile(&diagnostics));
    CHECK(Contains(diagnostics, "mesh"));
}

void TestAnimatedBinary(const Grid &grid) {
    const auto stage = UsdStage::CreateInMemory();
    auto builder = RigExecRigBuilder::Create(stage, kRig);
    AddMesh(stage, grid, grid.Compressed());
    auto wrinkle = builder.NewMoverChain("Deform", kTarget).AddWrinkleMover("Wrinkle");
    wrinkle.SetRestPoints(grid.rest);
    wrinkle.SetPinPoints({45});
    const auto prim = wrinkle.GetPrim();
    auto sample = [&](const char *name, auto value, int frame) {
        CHECK(prim.GetAttribute(TfToken(name)).Set(value, UsdTimeCode(frame)));
    };
    sample("inputs:iterations", 40, 1); sample("inputs:iterations", 0, 2);
    sample("inputs:iterations", 65, 3);
    sample("inputs:neighborDistance", 1, 1); sample("inputs:neighborDistance", 3, 3);
    sample("inputs:restLengthScale", 1.0f, 1); sample("inputs:restLengthScale", 1.08f, 3);
    sample("inputs:stretchStiffness", 0.75f, 1); sample("inputs:stretchStiffness", 0.9f, 3);
    sample("inputs:compressionStiffness", 0.8f, 1); sample("inputs:compressionStiffness", 0.6f, 3);
    sample("inputs:bendStiffness", 0.03f, 1); sample("inputs:bendStiffness", 0.15f, 3);
    sample("inputs:maxDisplacement", 0.12f, 1); sample("inputs:maxDisplacement", 0.25f, 3);
    sample("inputs:pinBorders", true, 1); sample("inputs:pinBorders", false, 3);
    sample("inputs:tangentPlaneCollisions", true, 1);
    sample("inputs:tangentPlaneCollisions", false, 3);
    sample("inputs:tangentPlaneInset", 0.01f, 1); sample("inputs:tangentPlaneInset", 0.04f, 3);
    sample("inputs:wrinkleScale", 0.6f, 1); sample("inputs:wrinkleScale", 1.2f, 3);
    sample("inputs:smoothingIterations", 0, 1); sample("inputs:smoothingIterations", 2, 3);
    sample("inputs:defaultWeight", 0.6f, 1); sample("inputs:defaultWeight", 1.0f, 3);
    sample("inputs:enabled", true, 1); sample("inputs:enabled", false, 4);
    sample("inputs:enabled", true, 5);
    RigExecRigEvaluator evaluator(stage, kRig);
    Compile(evaluator);
    CheckBinaryParity(evaluator, stage, {1, 2, 3, 4, 5});

    auto driver = stage->GetPrimAtPath(kRig).CreateAttribute(
        TfToken("wrinkleScale"), SdfValueTypeNames->Float);
    CHECK(driver.Set(0.4f, UsdTimeCode(1)));
    CHECK(driver.Set(0.9f, UsdTimeCode(3)));
    CHECK(prim.GetAttribute(TfToken("inputs:wrinkleScale")).SetConnections({driver.GetPath()}));
    std::vector<float> mask(grid.rest.size());
    for (size_t i = 0; i < mask.size(); ++i) mask[i] = float(i % 3) * 0.5f;
    const auto weights = builder.AddStaticWeight("Mask", kTarget, mask);
    wrinkle.SetWeightObject(weights.GetPath());
    CheckBinaryParity(evaluator, stage, {1, 2, 3, 4, 5});
    const auto beforeMovedScale = Published(evaluator.Evaluate(UsdTimeCode(1)));
    auto scaleMover = builder.NewMoverChain("Scale", driver.GetPath())
        .AddFloatMathMover("Blend", TfToken("blend"), 0.2f);
    CHECK(scaleMover.GetPrim().GetAttribute(TfToken("inputs:value"))
        .Set(0.2f, UsdTimeCode(1)));
    CHECK(scaleMover.GetPrim().GetAttribute(TfToken("inputs:value"))
        .Set(0.8f, UsdTimeCode(3)));
    wrinkle.SetReadPhase(TfToken("inputs:wrinkleScale"), "final");
    CHECK(!Near(Published(evaluator.Evaluate(UsdTimeCode(1))), beforeMovedScale));
    CheckBinaryParity(evaluator, stage, {1, 2, 3, 4, 5});

    // A directly moved setting is read from property results rather than captured paths.
    const auto scaleAttr = prim.GetAttribute(TfToken("inputs:wrinkleScale"));
    CHECK(scaleAttr.ClearConnections());
    auto directScale = builder.NewMoverChain("DirectScale", scaleAttr.GetPath())
        .AddFloatMathMover("Blend", TfToken("blend"), 0.3f);
    CHECK(directScale.GetPrim().GetAttribute(TfToken("inputs:value"))
        .Set(0.3f, UsdTimeCode(1)));
    CHECK(directScale.GetPrim().GetAttribute(TfToken("inputs:value"))
        .Set(0.75f, UsdTimeCode(3)));
    CheckBinaryParity(evaluator, stage, {1, 3});
    wrinkle.SetTopology(TfToken("surfaceStruts"));
    CheckBinaryParity(evaluator, stage, {1, 3});
    wrinkle.SetPinPoints({32, 45, 58});
    CheckBinaryParity(evaluator, stage, {1, 3});
    sample("inputs:iterations", -1, 6);
    CheckBinaryParity(evaluator, stage, {6});
}

void TestUpstreamCompression(const Grid &grid, const char *binaryPath) {
    const auto stage = UsdStage::CreateInMemory();
    auto builder = RigExecRigBuilder::Create(stage, kRig);
    const auto mesh = AddMesh(stage, grid, grid.rest);
    auto control = builder.AddControl("Compression", GfMatrix4d(1));
    auto chain = builder.NewMoverChain("Deform", kTarget);
    auto wrinkle = chain.AddWrinkleMover("Wrinkle");
    // Add order is reverse application order; the empty rest uses authored points.
    chain.AddMatrixMover("Compress", control.GetPath());
    RigExecRigEvaluator evaluator(stage, kRig);
    Compile(evaluator);
    RigExecBakeOpts options;
    options.time = 1;
    RigExecBakeResult result;
    std::string error;
    if (!RigExecBakeToBinary(evaluator, options, &result, &error))
        throw std::runtime_error(error);
    auto reader = RigExecRuntimeReader::Open(result.bytes.data(), result.bytes.size(), &error);
    if (!reader) throw std::runtime_error(error);
    // The control's scale authored on the stage and set as the binary's
    // input: both rerun the matrix mover and the wrinkle after it.
    for (const double scale : {1.0, 0.65, 0.8, 1.1, 1.0}) {
        control.SetAvarScale(scale, 1, 1);
        CHECK(reader->SetInput(control.GetPath().AppendProperty(TfToken("avars:sx"))
            .GetString(), scale, &error));
        const auto actual = Published(evaluator.Evaluate(UsdTimeCode(1)));
        auto expected = grid.rest;
        for (auto &point : expected) point[0] = float(double(point[0]) * scale);
        const auto preceding = expected;
        CHECK(RigExecApplyWrinkle(&expected, grid.rest, grid.counts, grid.indices));
        CHECK(Near(actual, expected));
        if (scale < 1) CHECK(!Near(actual, preceding, 0.001));
        if (!reader->Execute(&error)) throw std::runtime_error(error);
        CheckRuntime(*reader, actual);
    }
    reader->ResetInputs();
    CHECK(reader->Execute(&error));
    CheckRuntime(*reader, grid.rest);

    // An empty rest input follows edits to the authored base, never the preceding mover.
    Grid revisedGrid = grid;
    for (auto &point : revisedGrid.rest) point[1] *= 1.1f;
    CHECK(mesh.GetPointsAttr().Set(
        VtVec3fArray(revisedGrid.rest.begin(), revisedGrid.rest.end())));
    control.SetAvarScale(0.7, 1, 1);
    auto revisedPose = revisedGrid.rest;
    for (auto &point : revisedPose) point[0] = float(double(point[0]) * 0.7);
    CHECK(Near(Published(evaluator.Evaluate(UsdTimeCode(1))),
        Solve(revisedGrid, revisedPose)));
    CheckBinaryParity(evaluator, stage, {1});
    CHECK(mesh.GetPointsAttr().Set(VtVec3fArray(grid.rest.begin(), grid.rest.end())));
    control.SetAvarScale(1, 1, 1);

    const auto scaleAttr = control.GetPrim().GetAttribute(TfToken("avars:sx"));
    CHECK(scaleAttr.Set(1.0, UsdTimeCode(1)));
    CHECK(scaleAttr.Set(0.65, UsdTimeCode(24)));
    CHECK(scaleAttr.Set(1.0, UsdTimeCode(48)));
    std::vector<double> frames;
    std::vector<Points> expected;
    for (int step = 0; step <= 470; ++step) {
        frames.push_back(1 + step / 10.0);
        expected.push_back(Published(evaluator.Evaluate(UsdTimeCode(frames.back()))));
    }
    const auto incomingAt = [&](double frame) {
        const double scale = frame <= 24 ? 1 - 0.35 * (frame - 1) / 23 :
                                          0.65 + 0.35 * (frame - 24) / 24;
        auto incoming = grid.rest;
        for (auto &point : incoming) point[0] = float(double(point[0]) * scale);
        return incoming;
    };
    double maxOffsetStep = 0;
    for (size_t index = 1; index < frames.size(); ++index) {
        const auto previousIncoming = incomingAt(frames[index - 1]);
        const auto incoming = incomingAt(frames[index]);
        for (size_t point = 0; point < grid.rest.size(); ++point) {
            const auto previousOffset = expected[index - 1][point] - previousIncoming[point];
            const auto offset = expected[index][point] - incoming[point];
            const double stepLength = (offset - previousOffset).GetLength();
            CHECK(std::isfinite(stepLength));
            maxOffsetStep = std::max(maxOffsetStep, stepLength);
        }
    }
    // A tenth-frame change in compression must not relocate a fold by a
    // substantial fraction of this fixture's 0.125-unit structural edge.
    CHECK(maxOffsetStep < 0.01);
    const auto rootBefore = LayerText(stage->GetRootLayer());
    const auto sessionBefore = LayerText(stage->GetSessionLayer());
    {
        RigExecRigEvaluator reference(stage, kRig);
        reference.cpuReference = true;
        Compile(reference);
        const auto order = PlaybackOrder(frames.size());
        // Start at a compressed subframe without evaluating preceding poses.
        CHECK(SameBits(Published(reference.Evaluate(UsdTimeCode(frames[163]))), expected[163]));
        for (size_t index : order)
            CHECK(SameBits(Published(reference.Evaluate(UsdTimeCode(frames[index]))),
                           expected[index]));
    }
    const auto bytes = CheckBinaryParity(evaluator, stage, frames);
    CHECK(LayerText(stage->GetRootLayer()) == rootBefore);
    CHECK(LayerText(stage->GetSessionLayer()) == sessionBefore);
    if (binaryPath) {
        std::ofstream file(binaryPath, std::ios::binary);
        CHECK(file);
        file.write(reinterpret_cast<const char *>(bytes.data()), std::streamsize(bytes.size()));
        CHECK(file.good());
    }
}

} // namespace

int main(int argc, char **argv) {
    try {
        CHECK(argc == 2 || argc == 3);
        PlugRegistry::GetInstance().RegisterPlugins(argv[1]);
        const Grid grid;
        TestEvaluationAndInvalidation(grid);
        TestStaticInputValidation(grid);
        TestAnimatedBinary(grid);
        TestUpstreamCompression(grid, argc == 3 ? argv[2] : nullptr);
        std::cout << "Wrinkle: evaluator parity, invalidation, envelopes, validation, "
                     "non-authoring, reload and binary playback passed\n";
    } catch (const std::exception &error) {
        std::cerr << error.what() << '\n';
        return 1;
    }
}
