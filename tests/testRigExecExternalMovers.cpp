#include "rigExec/movers/moverRegistry.h"
#include "rigExec/rigEvaluator.h"
#include "rigExec/frozenContext.h"
#include "rigExecBake/bake.h"
#include "rigExecRigging/rigBuilder.h"

#include "pxr/base/plug/plugin.h"
#include "pxr/base/plug/registry.h"
#include "pxr/usd/sdf/layer.h"
#include "pxr/usd/usdGeom/points.h"

#include <cmath>
#include <chrono>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <stdexcept>

PXR_NAMESPACE_USING_DIRECTIVE
using namespace rigExec;

#define CHECK(expression) do { if (!(expression)) throw std::runtime_error( \
    std::string(__FILE__) + ":" + std::to_string(__LINE__) + ": " #expression); } while (false)

namespace {

const SdfPath kRig("/Rig"), kTarget("/Rig/Body.points");
const VtVec3fArray kBase{GfVec3f(-2, 0, 0), GfVec3f(1, 2, 3), GfVec3f(0)};

void Compile(RigExecRigEvaluator *evaluator)
{
    std::vector<std::string> errors;
    if (!evaluator->Compile(&errors)) {
        for (const auto &error : errors) std::cerr << error << '\n';
        CHECK(false);
    }
}

void CheckPose(const RigExecRigPose &pose, float gain, float envelope,
               const char *phase = "evaluation")
{
    CHECK(pose.valid);
    if (pose.bakedParityMismatches != 0) {
        for (const auto &message : pose.diagnostics) {
            std::cerr << message << '\n';
        }
        throw std::runtime_error(std::string(phase) + ": baked parity mismatch for gain=" +
            std::to_string(gain) + ", envelope=" + std::to_string(envelope));
    }
    CHECK(pose.moverGraphParityMismatches == 0);
    CHECK(pose.movedProperties.count(kTarget));
    const VtVec3fArray &points =
        pose.movedProperties.at(kTarget).Get<VtVec3fArray>();
    CHECK(points.size() == kBase.size());
    for (size_t i = 0; i < points.size(); ++i) {
        GfVec3f expected = kBase[i];
        expected[2] += envelope * gain * expected[0] * expected[0];
        CHECK((points[i] - expected).GetLength() < 1e-6f);
    }
}

void TestRegistration(const char *pluginDirectory)
{
    const auto *builtIn = RigExecFindMoverHandler(TfToken("RigExecMatrixMover"));
    CHECK(builtIn);
    PlugRegistry::GetInstance().RegisterPlugins(pluginDirectory);
    const auto *external = RigExecFindMoverHandler(TfToken("ExternalQuadraticMover"));
    CHECK(external);
    CHECK(external->resolveOp(TfToken()) == RigExecRevisionOp::External);
    CHECK(external->assembleExternal && external->applyExternal);
    CHECK(PlugRegistry::GetInstance().GetPluginWithName("testExternalMover")->IsLoaded());
    CHECK(builtIn == RigExecFindMoverHandler(TfToken("RigExecMatrixMover")));

    std::vector<std::string> diagnostics;
    CHECK(RigExecLoadMoverPlugins(&diagnostics));
    CHECK(diagnostics.empty());
    std::string error;
    CHECK(!RigExecRegisterMoverHandler(*external, &error));
    CHECK(error.find("already registered") != std::string::npos);
    auto incomplete = *external;
    incomplete.schemaType = "IncompleteExternalMover";
    incomplete.applyExternal = nullptr;
    CHECK(!RigExecRegisterMoverHandler(incomplete, &error));
    CHECK(error.find("both") != std::string::npos);

    // Additional libraries and caller-owned names cannot invalidate lookups.
    for (int i = 0; i < 40; ++i) {
        const std::string name = "AdditionalExternalMover" + std::to_string(i);
        auto handler = *external;
        handler.schemaType = name.c_str();
        CHECK(RigExecRegisterMoverHandler(handler, &error));
    }
    CHECK(external == RigExecFindMoverHandler(TfToken("ExternalQuadraticMover")));
    CHECK(RigExecFindMoverHandler(TfToken("AdditionalExternalMover39")));
}

void TestEvaluation()
{
    const auto stage = UsdStage::CreateInMemory();
    RigExecRigBuilder::Create(stage, kRig);
    const auto body = UsdGeomPoints::Define(stage, kTarget.GetPrimPath());
    CHECK(body.CreatePointsAttr().Set(kBase));
    // A plugin may bring a generated USD schema, but is not required to do so.
    const UsdPrim mover = stage->DefinePrim(SdfPath("/Rig/External"),
                                            TfToken("ExternalQuadraticMover"));
    CHECK(mover.AddAppliedSchema(TfToken("RigExecMoverAPI")));
    CHECK(mover.CreateRelationship(TfToken("rigExec:moves"), false)
               .SetTargets({kTarget}));
    const UsdAttribute gain = mover.CreateAttribute(TfToken("inputs:gain"),
                                                    SdfValueTypeNames->Float);
    CHECK(gain.Set(0.5f, UsdTimeCode(1)));
    CHECK(gain.Set(1.5f, UsdTimeCode(3)));
    const UsdAttribute weight = mover.CreateAttribute(TfToken("inputs:defaultWeight"),
                                                      SdfValueTypeNames->Float, false);
    CHECK(weight.Set(0.5f));
    const UsdAttribute enabled = mover.CreateAttribute(TfToken("inputs:enabled"),
                                                       SdfValueTypeNames->Bool, false);
    CHECK(enabled.Set(true));

    std::string before;
    CHECK(stage->GetRootLayer()->ExportToString(&before));
    RigExecRigEvaluator dynamic(stage, kRig);
    dynamic.SetEvaluationMode(RigExecEvaluationMode::Dynamic);
    dynamic.cpuParityMode = true;
    Compile(&dynamic);
    RigExecRigEvaluator baked(stage, kRig);
    baked.SetEvaluationMode(RigExecEvaluationMode::BakedWithParityCheck);
    Compile(&baked);
    CHECK(baked.IsBakeable());
    for (int frame : {3, 1, 2, 3}) {
        const auto reference = dynamic.Evaluate(UsdTimeCode(frame));
        CheckPose(reference, 0.5f * frame, 0.5f);
        CHECK(reference.moverGraphParityAgreements > 0);
        CheckPose(baked.Evaluate(UsdTimeCode(frame)), 0.5f * frame, 0.5f);
    }
    CHECK(baked.GetBakedGenerationCount() > 0);
    std::string after;
    CHECK(stage->GetRootLayer()->ExportToString(&after));
    CHECK(before == after);

    CHECK(gain.Set(2.0f, UsdTimeCode(1)));
    CheckPose(dynamic.Evaluate(UsdTimeCode(1)), 2.0f, 0.5f);
    CheckPose(baked.Evaluate(UsdTimeCode(1)), 2.0f, 0.5f);

    const auto source = stage->GetPrimAtPath(kRig).CreateAttribute(
        TfToken("externalGain"), SdfValueTypeNames->Float);
    CHECK(source.Set(3.0f));
    CHECK(gain.SetConnections({source.GetPath()}));
    CheckPose(dynamic.Evaluate(UsdTimeCode(1)), 3.0f, 0.5f);
    CheckPose(baked.Evaluate(UsdTimeCode(1)), 3.0f, 0.5f);

    for (auto *evaluator : {&dynamic, &baked}) {
        evaluator->SetInteractiveOverrides({RigExecValueOverride{
            kRig, TfToken(), TfToken("externalGain"), VtValue(4.0f)}});
        CheckPose(evaluator->Evaluate(UsdTimeCode(1)), 4.0f, 0.5f, "override set");
        evaluator->ClearInteractiveOverrides();
        CheckPose(evaluator->Evaluate(UsdTimeCode(1)), 3.0f, 0.5f, "override cleared");
    }

    CHECK(enabled.Set(false));
    CheckPose(dynamic.Evaluate(UsdTimeCode(1)), 0.0f, 1.0f);
    CheckPose(baked.Evaluate(UsdTimeCode(1)), 0.0f, 1.0f);
    CHECK(enabled.Set(true));
    for (float invalid : {-1.0f, -2.0f, -3.0f}) {
        CHECK(source.Set(invalid));
        CheckPose(dynamic.Evaluate(UsdTimeCode(1)), 0.0f, 1.0f);
        CheckPose(baked.Evaluate(UsdTimeCode(1)), 0.0f, 1.0f);
    }
    CHECK(source.Set(1.0f));
    CheckPose(baked.Evaluate(UsdTimeCode(1)), 1.0f, 0.5f);

    std::string error;
    CHECK(!RigExecCanFreezeProgram(baked, &error));
    CHECK(!error.empty());
    RigExecBakeOpts options;
    options.frames = {1.0};
    RigExecBakeResult output;
    error.clear();
    CHECK(!RigExecBakeToBinary(baked, options, &output, &error));
    CHECK(!error.empty());
}

void TestIncompatiblePlugin()
{
    const auto directory = std::filesystem::temp_directory_path() /
        ("rigexec-plugin-version-" + std::to_string(
            std::chrono::steady_clock::now().time_since_epoch().count()));
    std::filesystem::create_directories(directory);
    struct Cleanup {
        std::filesystem::path directory;
        ~Cleanup() {
            std::error_code ignored;
            std::filesystem::remove_all(directory, ignored);
        }
    } cleanup{directory};
    {
        std::ofstream metadata(directory / "plugInfo.json");
        metadata << R"({"Plugins": [{"Type": "library", "Name": "incompatibleExternalMover", "Root": ".", "ResourcePath": ".", "LibraryPath": "never-loaded", "Info": {"RigExecMoverPlugin": 999}}]})";
        CHECK(metadata.good());
    }
    // The loader must inspect metadata registered after its initial discovery
    // and reject the version before attempting to load the library.
    PlugRegistry::GetInstance().RegisterPlugins(directory.generic_string());
    const auto plugin = PlugRegistry::GetInstance().GetPluginWithName(
        "incompatibleExternalMover");
    CHECK(plugin && !plugin->IsLoaded());
    std::vector<std::string> diagnostics;
    CHECK(!RigExecLoadMoverPlugins(&diagnostics));
    CHECK(diagnostics.size() == 1);
    CHECK(diagnostics.front().find("incompatible") != std::string::npos);
    CHECK(diagnostics.front().find("version") != std::string::npos);
    CHECK(!plugin->IsLoaded());
}

}  // namespace

int main(int argc, char **argv)
{
    try {
        CHECK(argc == 3);
        PlugRegistry::GetInstance().RegisterPlugins(argv[1]);
        TestRegistration(argv[2]);
        TestEvaluation();
        TestIncompatiblePlugin();
        std::cout << "External mover loading, registration, dynamic/baked parity, "
                     "invalidation, envelopes and export guards passed\n";
    } catch (const std::exception &error) {
        std::cerr << error.what() << '\n';
        return 1;
    }
}
