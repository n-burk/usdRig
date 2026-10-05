#include "rigExec/movers/moverRegistry.h"
#include "rigExec/rigEvaluator.h"
#include "rigExec/frozenContext.h"
#include "rigExecBake/bake.h"
#include "rigExecRigging/rigBuilder.h"
#include "rigExecRuntime/runtime.h"

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
const SdfPath kReference("/Rig/Reference.points"), kControl("/Rig/Ctl");
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
    options.time = 1.0;
    RigExecBakeResult output;
    error.clear();
    CHECK(RigExecBakeToBinary(baked, options, &output, &error));
    CHECK(!output.bytes.empty());
    // The plugin records its gain read, a key no core assembly reads: the
    // bake counts it rather than refusing it.
    CHECK(output.pathReadsUnenumerated == 1);
}

// A plugin mover on /Rig/Body that also reads /Rig/Reference at `final`,
// after a matrix mover lifted it by /Rig/Ctl's animated avars:ty.
UsdStageRefPtr MakeExportRig(const char *moverType)
{
    const auto stage = UsdStage::CreateInMemory();
    stage->SetStartTimeCode(1);
    stage->SetEndTimeCode(3);
    RigExecRigBuilder::Create(stage, kRig);
    CHECK(UsdGeomPoints::Define(stage, kTarget.GetPrimPath())
              .CreatePointsAttr().Set(kBase));
    CHECK(UsdGeomPoints::Define(stage, kReference.GetPrimPath())
              .CreatePointsAttr().Set(VtVec3fArray{GfVec3f(0)}));
    const UsdPrim control =
        stage->DefinePrim(kControl, TfToken("RigExecControl"));
    const UsdAttribute ty = control.GetAttribute(TfToken("avars:ty"));
    CHECK(ty.Set(0.0, UsdTimeCode(1)) && ty.Set(2.0, UsdTimeCode(3)));
    const UsdPrim lift = stage->DefinePrim(SdfPath("/Rig/Lift"),
                                           TfToken("RigExecMatrixMover"));
    CHECK(lift.AddAppliedSchema(TfToken("RigExecMoverAPI")));
    CHECK(lift.CreateRelationship(TfToken("rigExec:moves"), false)
              .SetTargets({kReference}));
    CHECK(lift.CreateRelationship(TfToken("rigExec:transform"), false)
              .SetTargets({kControl}));
    const UsdPrim mover =
        stage->DefinePrim(SdfPath("/Rig/External"), TfToken(moverType));
    CHECK(mover.AddAppliedSchema(TfToken("RigExecMoverAPI")));
    CHECK(mover.CreateRelationship(TfToken("rigExec:moves"), false)
              .SetTargets({kTarget}));
    const UsdRelationship reference =
        mover.CreateRelationship(TfToken("rigExec:reference"), false);
    CHECK(reference.SetTargets({kReference}));
    CHECK(reference.SetMetadata(TfToken(RigExecReadPhaseMetadataName),
                                std::string("final")));
    const UsdAttribute gain = mover.CreateAttribute(
        TfToken("inputs:gain"), SdfValueTypeNames->Float);
    CHECK(gain.Set(0.5f, UsdTimeCode(1)) && gain.Set(1.5f, UsdTimeCode(3)));
    CHECK(mover.CreateAttribute(TfToken("inputs:defaultWeight"),
                                SdfValueTypeNames->Float, false)
              .Set(0.5f));
    return stage;
}

const std::vector<RrVec3f> &Points(const RigExecRuntimeReader &reader,
                                   const SdfPath &path)
{
    for (const RigExecRuntimePoints &moved : reader.GetPoints()) {
        if (moved.path == path.GetString()) return moved.points;
    }
    throw std::runtime_error("playback published no " + path.GetString());
}

// Playback is held to the baked program bit for bit, as every runtime
// family is.
void CheckSame(const std::vector<RrVec3f> &played, const VtVec3fArray &baked,
               const std::string &what)
{
    if (played.size() != baked.size()) {
        throw std::runtime_error(what + ": point count differs");
    }
    for (size_t i = 0; i < played.size(); ++i) {
        for (size_t a = 0; a < 3; ++a) {
            if (played[i][a] != baked[i][a]) {
                throw std::runtime_error(
                    what + ": point " + std::to_string(i) + " is " +
                    std::to_string(played[i][a]) + ", baked " +
                    std::to_string(baked[i][a]));
            }
        }
    }
}

// The fixture's arithmetic at envelope 0.5: y rises by half the reference's
// lift and z by half of gain * x^2.
void CheckDeformed(const std::vector<RrVec3f> &played, float lift, float gain,
                   const std::string &what)
{
    if (played.size() != kBase.size()) {
        throw std::runtime_error(what + ": point count differs");
    }
    for (size_t i = 0; i < played.size(); ++i) {
        const GfVec3f &base = kBase[i];
        const GfVec3f expected(base[0], base[1] + 0.5f * lift,
                               base[2] + 0.5f * gain * base[0] * base[0]);
        for (size_t a = 0; a < 3; ++a) {
            if (std::abs(played[i][a] - expected[a]) > 1e-5f) {
                throw std::runtime_error(what + ": point " +
                                         std::to_string(i) + " is off");
            }
        }
    }
}

bool Mentions(const std::vector<std::string> &lines, const std::string &text)
{
    for (const std::string &line : lines) {
        if (line.find(text) != std::string::npos) return true;
    }
    return false;
}

void TestExport()
{
    const std::string type = "ExternalQuadraticMover";
    const auto stage = MakeExportRig(type.c_str());
    RigExecRigEvaluator baked(stage, kRig);
    baked.SetEvaluationMode(RigExecEvaluationMode::Baked);
    // Baked at frame 1. The plugin mover's payload holds its parameters
    // (gain among them) as the bake time read them, so playback is held to
    // the program at that time; inputs the rest of the rig reads move.
    RigExecBakeOpts options;
    options.time = 1.0;
    RigExecBakeResult result;
    std::string error;
    if (!RigExecBakeToBinary(baked, options, &result, &error)) {
        throw std::runtime_error("export failed: " + error);
    }
    CHECK(result.pathReadsUnenumerated == 1);

    // Played through the plugin's kernel: what the baked program evaluated.
    auto reader = RigExecRuntimeReader::Open(result.bytes.data(),
                                             result.bytes.size(), &error);
    if (!reader) throw std::runtime_error("open failed: " + error);
    CHECK(reader->GetExternalMoverTypes() == std::vector<std::string>{type});
    CHECK(reader->GetMissingExternalKernels() ==
          std::vector<std::string>{type});
    const RigExecMoverHandler *handler =
        RigExecFindMoverHandler(TfToken(type));
    CHECK(handler && handler->runtimeKernel.IsSet());
    CHECK(reader->SetExternalKernel(type, handler->runtimeKernel, &error));
    CHECK(reader->GetMissingExternalKernels().empty());
    const double frame = 1.0;
    if (!reader->Execute(&error)) {
        throw std::runtime_error("playback failed: " + error);
    }
    CHECK(!Mentions(reader->GetDiagnostics(), "no kernel"));
    const RigExecRigPose pose = baked.Evaluate(UsdTimeCode(frame));
    CHECK(pose.valid);
    for (const SdfPath &path : {kTarget, kReference}) {
        CheckSame(Points(*reader, path),
                  pose.movedProperties.at(path).Get<VtVec3fArray>(),
                  path.GetString() + " at frame " + std::to_string(frame));
    }
    // Both sides agreeing is not enough: they must agree on the
    // deformation, reference included (ty is 0 and gain 0.5 at frame 1).
    CheckDeformed(Points(*reader, kTarget), 0.0f, 0.5f,
                  "playback arithmetic");

    // Posed: the phased reference is playback's own evaluation, so the
    // control's avar set as an input moves the plugin mover with it. An
    // unconnected avar's authored value and an interactive override on it
    // are the same value, so the program's reference is the override.
    CHECK(reader->SetInput(kControl.GetString() + ".avars:ty", 5.0, &error));
    CHECK(reader->Execute(&error));
    baked.SetInteractiveOverrides({RigExecValueOverride{
        kControl, TfToken(), TfToken("avars:ty"), VtValue(5.0)}});
    const RigExecRigPose posed = baked.Evaluate(UsdTimeCode(frame));
    baked.ClearInteractiveOverrides();
    CHECK(posed.valid);
    CheckSame(Points(*reader, kTarget),
              posed.movedProperties.at(kTarget).Get<VtVec3fArray>(),
              "posed playback");
    // The payload was baked with the unposed lift of 0; the kernel must
    // have used playback's lift of 5 (and the gain of 0.5 the payload
    // holds).
    CheckDeformed(Points(*reader, kTarget), 5.0f, 0.5f, "posed arithmetic");
    reader->ResetInputs();

    // Without the kernel the mover is a no-op, and every frame says so.
    auto bare = RigExecRuntimeReader::Open(result.bytes.data(),
                                           result.bytes.size(), &error);
    CHECK(bare);
    CHECK(bare->Execute(&error));
    CHECK(Mentions(bare->GetDiagnostics(),
                   "warning: /Rig/External is a " + type +
                       ", which this runtime has no kernel for"));
    const std::vector<RrVec3f> &passed = Points(*bare, kTarget);
    CHECK(passed.size() == kBase.size());
    for (size_t i = 0; i < passed.size(); ++i) {
        for (size_t a = 0; a < 3; ++a) {
            CHECK(passed[i][a] == kBase[i][a]);
        }
    }
    CheckSame(Points(*bare, kReference),
              baked.Evaluate(UsdTimeCode(frame))
                  .movedProperties.at(kReference)
                  .Get<VtVec3fArray>(),
              "reference beside a missing kernel");

    // A kernel that cannot read the epoch bytes leaves it passing through.
    RigExecExternalKernel refusing = handler->runtimeKernel;
    refusing.prepare = [](const uint8_t *, size_t, std::string *why)
        -> std::shared_ptr<const void> {
        if (why) *why = "refused";
        return nullptr;
    };
    CHECK(!bare->SetExternalKernel(type, refusing, &error));
    CHECK(error.find("refused") != std::string::npos);
    CHECK(bare->GetMissingExternalKernels() ==
          std::vector<std::string>{type});
    CHECK(!bare->SetExternalKernel("NoSuchMover", handler->runtimeKernel,
                                   &error));

    // A plugin that never adopted export refuses it, naming the mover.
    const auto unexportable = MakeExportRig("ExternalUnexportableMover");
    RigExecRigEvaluator refused(unexportable, kRig);
    refused.SetEvaluationMode(RigExecEvaluationMode::Baked);
    error.clear();
    CHECK(!RigExecBakeToBinary(refused, options, &result, &error));
    CHECK(error.find("/Rig/External") != std::string::npos &&
          error.find("no .rigexec encoding") != std::string::npos);
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
        TestExport();
        TestIncompatiblePlugin();
        std::cout << "External mover loading, registration, dynamic/baked parity, "
                     "invalidation, envelopes, export and playback passed\n";
    } catch (const std::exception &error) {
        std::cerr << error.what() << '\n';
        return 1;
    }
}
