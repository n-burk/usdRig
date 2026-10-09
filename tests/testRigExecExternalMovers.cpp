#include "rigExec/bakedProgramImpl.h"
#include "rigExec/movers/moverRegistry.h"
#include "rigExec/rigEvaluator.h"
#include "rigExec/frameCache.h"
#include "rigExec/frozenContext.h"
#include "rigExecBake/bake.h"
#include "rigExecBinary/format.h"
#include "rigExecRigging/rigBuilder.h"
#include "rigExecRuntime/runtime.h"
#include "rigExecRuntime/inputs.h"
#include "rigExecRuntime/store.h"
#include "rigExecRuntime/stageArrayInputs.h"
#include "rigExecGraph/usdSceneAccess.h"
#include "rigExecGraph/sceneProgramLowering.h"

#include "pxr/base/plug/plugin.h"
#include "pxr/base/plug/registry.h"
#include "pxr/usd/sdf/layer.h"
#include "pxr/usd/usdGeom/points.h"

#include <algorithm>
#include <cmath>
#include <cstring>
#include <chrono>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <limits>
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
    if (!pose.valid) {
        std::cerr << phase << ": invalid external mover pose\n";
        for (const auto &line : pose.diagnostics) std::cerr << "    " << line << '\n';
    }
    CHECK(pose.valid);

    CHECK(pose.referenceMismatches == 0);
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
    CHECK(external->declareExternalInputs && external->compileScene && external->assembleExternal && external->applyExternal);
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
    CHECK(error.find("declareExternalInputs") != std::string::npos);

    auto undeclared = *external;
    undeclared.schemaType = "UndeclaredExternalMover";
    undeclared.declareExternalInputs = nullptr;
    CHECK(!RigExecRegisterMoverHandler(undeclared, &error));
    CHECK(error.find("declareExternalInputs") != std::string::npos);

    auto detachedMissing=*external;
    detachedMissing.schemaType="MissingDetachedExternalMover";
    detachedMissing.compileScene=nullptr;
    CHECK(!RigExecRegisterMoverHandler(detachedMissing,&error));
    CHECK(error.find("compileScene")!=std::string::npos);

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


void CensusInputs(const SdfPath &path,
                  std::vector<RigExecRevisionLeafKey> *inputs)
{
    using Type=RigExecRevisionLeafType;using Time=RigExecRevisionLeafTime;
    using Flavour=RigExecRevisionLeafFlavour;
    inputs->push_back({path.AppendProperty(TfToken("inputs:gain")),Type::Float,Time::AtTime,Flavour::Resolved,VtValue(1.0f)});
    inputs->push_back({path.AppendProperty(TfToken("inputs:route")),Type::Float,Time::AtTime,Flavour::ResolvedOnly,VtValue(0.0f)});
    inputs->push_back({path.AppendProperty(TfToken("inputs:raw")),Type::Double,Time::AtTime,Flavour::Raw,VtValue(0.0)});
    inputs->push_back({path.AppendProperty(TfToken("inputs:default")),Type::Double,Time::AtDefault,Flavour::Resolved,VtValue(0.0)});
    inputs->push_back({path.AppendProperty(TfToken("inputs:rawDefault")),Type::Double,Time::AtDefault,Flavour::Raw,VtValue(0.0)});
    inputs->push_back({path.AppendProperty(TfToken("inputs:present")),Type::Bool,Time::AtTime,Flavour::Present,VtValue(false)});
    inputs->push_back({path.AppendProperty(TfToken("inputs:missing")),Type::Double,Time::AtTime,Flavour::Resolved,VtValue(0.0)});
}

void DeclareCensusInputs(const RigExecMoverBindContext &ctx,
                        std::vector<RigExecRevisionLeafKey> *inputs)
{
    CensusInputs(ctx.moverPrim.GetPath(),inputs);
}

bool CompileCensusScene(const RigExecSceneDescriptors &,const SdfPath &mover,
                       const SdfPath &,RigExecRevisionBinding *binding,std::string *)
{
    if(!binding)return false;
    binding->externalInputs.clear();
    CensusInputs(mover,&binding->externalInputs);
    return true;
}

void TestDeclaredScalarPublicCensus()
{
    auto handler=*RigExecFindMoverHandler(TfToken("ExternalQuadraticMover"));
    handler.schemaType="ExternalCensusMover";handler.declareExternalInputs=&DeclareCensusInputs;
    handler.compileScene=&CompileCensusScene;
    std::string error;CHECK(RigExecRegisterMoverHandler(handler,&error));
    const auto stage=UsdStage::CreateInMemory();RigExecRigBuilder::Create(stage,kRig);
    CHECK(UsdGeomPoints::Define(stage,kTarget.GetPrimPath()).CreatePointsAttr().Set(kBase));
    const auto mover=stage->DefinePrim(SdfPath("/Rig/Census"),TfToken("ExternalCensusMover"));
    CHECK(mover.AddAppliedSchema(TfToken("RigExecMoverAPI")));
    CHECK(mover.CreateRelationship(TfToken("rigExec:moves"),false).SetTargets({kTarget}));
    CHECK(mover.CreateAttribute(TfToken("inputs:gain"),SdfValueTypeNames->Float).Set(0.5f));
    const auto channels=stage->DefinePrim(SdfPath("/Rig/Channels"));
    const auto source=channels.CreateAttribute(TfToken("source"),SdfValueTypeNames->Float);
    CHECK(source.Set(0.75f));
    CHECK(mover.CreateAttribute(TfToken("inputs:route"),SdfValueTypeNames->Float).SetConnections({source.GetPath()}));
    for(const char*name:{"inputs:raw","inputs:default","inputs:rawDefault"})
        CHECK(mover.CreateAttribute(TfToken(name),SdfValueTypeNames->Double).Set(0.25));
    CHECK(mover.GetAttribute(TfToken("inputs:raw")).Set(1.0,UsdTimeCode(1)));
    CHECK(mover.GetAttribute(TfToken("inputs:raw")).Set(2.0,UsdTimeCode(2)));
    CHECK(mover.CreateAttribute(TfToken("inputs:present"),SdfValueTypeNames->Bool).Set(true));
    mover.CreateAttribute(TfToken("inputs:missing"),SdfValueTypeNames->Double);
    CHECK(mover.CreateAttribute(TfToken("inputs:undeclared"),SdfValueTypeNames->Double).Set(0.75));
    RigExecRigEvaluator evaluator(stage,kRig);Compile(&evaluator);
    CheckPose(evaluator.Evaluate(UsdTimeCode(1)),0.5f,1.0f);
    const auto &admitted=evaluator.GetBakedProgram()->GetUpstreamAdmissible();
    CHECK(admitted.count(mover.GetPath().AppendProperty(TfToken("inputs:gain")))==1);
    CHECK(admitted.count(source.GetPath())==1);
    for(const char*name:{"inputs:route","inputs:raw","inputs:default","inputs:rawDefault","inputs:present","inputs:missing","inputs:undeclared"})
        CHECK(admitted.count(mover.GetPath().AppendProperty(TfToken(name)))==0);
    evaluator.SetUpstreamInputs({{channels.GetPath(),TfToken(),TfToken("source"),VtValue(0.25f)}});
    CHECK(evaluator.Evaluate(UsdTimeCode(1)).valid);
    CHECK(evaluator.GetUpstreamInputPaths()==std::vector<SdfPath>{source.GetPath()});
    evaluator.SetUpstreamInputs({});
    RigExecBakeOpts opts;opts.time=1;RigExecBakeResult result;
    CHECK(RigExecBakeToBinary(evaluator,opts,&result,&error));
    auto reader=RigExecRuntimeReader::Open(result.bytes.data(),result.bytes.size(),&error);CHECK(reader);
    size_t index=0;CHECK(reader->FindInput(source.GetPath().GetString(),&index));
    CHECK(reader->GetInputValue(index).tag==RrInputTag::Float);
    CHECK(reader->GetInputValue(index).f32==0.75f);
    CHECK(reader->SetInput(source.GetPath().GetString(),0.25,&error));
    CHECK(reader->GetInputValue(index).f32==0.25f);
    CHECK(reader->ResetInput(source.GetPath().GetString(),&error));
    CHECK(reader->GetInputValue(index).f32==0.75f);
    for(const char*name:{"inputs:route","inputs:raw","inputs:default","inputs:rawDefault","inputs:present","inputs:missing","inputs:undeclared"})
        CHECK(!reader->FindInput(mover.GetPath().AppendProperty(TfToken(name)).GetString(),&index));
    const auto sampled=RigExecRuntimeStageArrayInputs::EnumerateProviderValues(*reader);
    const auto rawPath=mover.GetPath().AppendProperty(TfToken("inputs:raw")).GetString();
    const auto raw=std::find_if(sampled.begin(),sampled.end(),[&](const auto&row){return row.name==rawPath;});
    CHECK(raw!=sampled.end());
    if(raw!=sampled.end()) CHECK(raw->tag==RrInputTag::Double && raw->animated);
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
    RigExecRigEvaluator scalarChecked(stage, kRig);

    scalarChecked.cpuReference = true;
    Compile(&scalarChecked);
    RigExecRigEvaluator baked(stage, kRig);

    Compile(&baked);
    CHECK(baked.IsBakeable());
    for (int frame : {3, 1, 2, 3}) {
        const auto reference = scalarChecked.Evaluate(UsdTimeCode(frame));
        CheckPose(reference, 0.5f * frame, 0.5f);
        CHECK(reference.referenceAgreements > 0);
        CheckPose(baked.Evaluate(UsdTimeCode(frame)), 0.5f * frame, 0.5f);
    }
    CHECK(baked.GetBakedGenerationCount() > 0);
    std::string after;
    CHECK(stage->GetRootLayer()->ExportToString(&after));
    CHECK(before == after);

    CHECK(gain.Set(2.0f, UsdTimeCode(1)));
    CheckPose(scalarChecked.Evaluate(UsdTimeCode(1)), 2.0f, 0.5f);
    CheckPose(baked.Evaluate(UsdTimeCode(1)), 2.0f, 0.5f);

    const auto source = stage->GetPrimAtPath(kRig).CreateAttribute(
        TfToken("externalGain"), SdfValueTypeNames->Float);
    CHECK(source.Set(3.0f));
    CHECK(gain.SetConnections({source.GetPath()}));
    CheckPose(scalarChecked.Evaluate(UsdTimeCode(1)), 3.0f, 0.5f);
    CheckPose(baked.Evaluate(UsdTimeCode(1)), 3.0f, 0.5f);

    for (auto *evaluator : {&scalarChecked, &baked}) {
        evaluator->SetInteractiveOverrides({RigExecValueOverride{
            kRig, TfToken(), TfToken("externalGain"), VtValue(4.0f)}});
        CheckPose(evaluator->Evaluate(UsdTimeCode(1)), 4.0f, 0.5f, "override set");
        evaluator->ClearInteractiveOverrides();
        CheckPose(evaluator->Evaluate(UsdTimeCode(1)), 3.0f, 0.5f, "override cleared");
    }

    CHECK(enabled.Set(false));
    CheckPose(scalarChecked.Evaluate(UsdTimeCode(1)), 0.0f, 1.0f);
    CheckPose(baked.Evaluate(UsdTimeCode(1)), 0.0f, 1.0f);
    CHECK(enabled.Set(true));
    for (float invalid : {-1.0f, -2.0f, -3.0f}) {
        CHECK(source.Set(invalid));
        CheckPose(scalarChecked.Evaluate(UsdTimeCode(1)), 0.0f, 1.0f);
        CheckPose(baked.Evaluate(UsdTimeCode(1)), 0.0f, 1.0f);
    }
    CHECK(source.Set(1.0f));
    CheckPose(baked.Evaluate(UsdTimeCode(1)), 1.0f, 0.5f);

    std::string error;
    CHECK(RigExecCanFreezeProgram(baked, &error));
    std::shared_ptr<const RigExecFrozenProgram> frozen;
    CHECK(RigExecFreezeProgram(baked,&frozen,&error));
    CHECK(frozen);
    RigExecBakeOpts options;
    options.time = 1.0;
    RigExecBakeResult output;
    error.clear();
    CHECK(RigExecBakeToBinary(baked, options, &output, &error));
    CHECK(!output.bytes.empty());
    // The plugin's own reads stay inside its payload: the file holds its
    // one entry, with the epoch and the frame bytes of the bake's run.
    {
        std::unique_ptr<fb::RigExecWireFile> file;
        CHECK(RigExecFormatOpen(output.bytes.data(), output.bytes.size(),
                                &file, &error));
        CHECK(file && file->externalMovers.size() == 1);
        if (file && file->externalMovers.size() == 1) {
            const fb::RigExecWireExternalMover &mover =
                file->externalMovers.front();
            CHECK(mover.v2FrameValid);
            CHECK(!mover.epoch.empty() && !mover.v2Frame.empty());
        }
    }
}

// The frame-cache key over an external mover. Its declared inputs reach the
// worker as revision leaves, which no sample in `values` covers, so with
// every control static a time-sampled inputs:gain must key each frame apart
// (one shared key would serve one frame's points at another), and the same
// input held static must leave the key standing across frames. A static
// gain connected in session to an animated source is an edit the program
// cannot route: until its next run asks the leaf's variance again, the key
// must already count the gain as varying, and an unroutable edit that
// leaves the gain static must not. The plain and burst digests agree.
void TestTimeVaryingInputsKeyFramesApart()
{
    enum class Gain { Animated, Static, ConnectedInSession };
    for (const Gain mode :
         {Gain::Animated, Gain::Static, Gain::ConnectedInSession}) {
        const auto stage = UsdStage::CreateInMemory();
        RigExecRigBuilder::Create(stage, kRig);
        CHECK(UsdGeomPoints::Define(stage, kTarget.GetPrimPath())
                  .CreatePointsAttr().Set(kBase));
        const UsdPrim mover = stage->DefinePrim(
            SdfPath("/Rig/External"), TfToken("ExternalQuadraticMover"));
        CHECK(mover.AddAppliedSchema(TfToken("RigExecMoverAPI")));
        CHECK(mover.CreateRelationship(TfToken("rigExec:moves"), false)
                  .SetTargets({kTarget}));
        const UsdAttribute gain = mover.CreateAttribute(
            TfToken("inputs:gain"), SdfValueTypeNames->Float);
        const UsdAttribute source =
            stage->DefinePrim(SdfPath("/Src"))
                .CreateAttribute(TfToken("value"), SdfValueTypeNames->Float);
        CHECK(source.Set(0.5f, UsdTimeCode(1)) &&
              source.Set(1.5f, UsdTimeCode(3)));
        if (mode == Gain::Animated) {
            CHECK(gain.Set(0.5f, UsdTimeCode(1)) &&
                  gain.Set(1.5f, UsdTimeCode(3)));
        } else {
            CHECK(gain.Set(0.5f));
        }
        RigExecRigEvaluator evaluator(stage, kRig);
        Compile(&evaluator);
        CheckPose(evaluator.Evaluate(UsdTimeCode(1)), 0.5f, 1.0f);
        if (mode == Gain::ConnectedInSession) {
            CHECK(gain.SetConnections({source.GetPath()}));
            CHECK(evaluator.GetLastNoticeDisposition() ==
                  RigExecNoticeDisposition::StampBumped);
        } else if (mode == Gain::Static) {
            // Unroutable too, but the gain stays static when asked again.
            CHECK(mover.SetDocumentation("a static gain"));
            CHECK(evaluator.GetLastNoticeDisposition() ==
                  RigExecNoticeDisposition::StampBumped);
        }
        std::string error;
        CHECK(RigExecCanFreezeProgram(evaluator, &error));
        const std::vector<RigExecValueOverride> none;
        const bool varies = mode != Gain::Static;
        // Sampled before any later run, then again after one.
        for (const bool afterRun : {false, true}) {
            if (afterRun) {
                CheckPose(evaluator.Evaluate(UsdTimeCode(3)),
                          mode == Gain::Static ? 0.5f : 1.5f, 1.0f);
            }
            RigExecChainSampleBindings pinned;
            CHECK(RigExecBindChainSampleInputs(evaluator, &pinned, &error));
            RigExecBurstSampleCache burst;
            CHECK(evaluator.GetBakedProgram() &&
                  RigExecBuildBurstSampleCache(
                      *evaluator.GetBakedProgram(), pinned, none,
                      RigExecFrameCacheEpochDigest(evaluator), &burst,
                      &error));
            std::vector<uint64_t> digests;
            for (int frame : {1, 2, 3}) {
                RigExecFrameInputs inputs, burstInputs;
                CHECK(RigExecSampleFrameInputs(evaluator, UsdTimeCode(frame),
                                               none, &inputs, &error));
                CHECK(RigExecSampleFrameInputsWithBurstCache(
                    evaluator, UsdTimeCode(frame), none, &burst,
                    &burstInputs, &error));
                CHECK(RigExecControlStateDigestible(inputs, none));
                const uint64_t digest = RigExecControlStateDigest(inputs, none);
                CHECK(RigExecControlStateDigestWithBurstCache(
                          burstInputs, none, &burst) == digest);
                digests.push_back(digest);
                // Only a gain that can vary is listed; a static one is not.
                CHECK(inputs.varyingRevisionLeaves.size() == (varies ? 1u : 0u));
                CHECK(burstInputs.varyingRevisionLeaves ==
                      inputs.varyingRevisionLeaves);
                CHECK(inputs.varyingLayoutRows.empty());
            }
            if (varies) {
                CHECK(digests[0] != digests[1] && digests[0] != digests[2] &&
                      digests[1] != digests[2]);
            } else {
                CHECK(digests[0] == digests[1] && digests[1] == digests[2]);
            }
        }
    }
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

void TestDetachedExternalProgram()
{
    // An explicit recompile must retain the real original SCC authority after
    // its writer bindings were rebuilt into an otherwise acyclic program.
    RigExecSceneProgram cyclic;
    cyclic.layout.valueKeys={"raw","solve","field","commit"};cyclic.layout.leaves={0};
    std::string retainedError;
    CHECK(cyclic.Append("Solve",RigExecSceneCopyOp{3,1},&retainedError));
    CHECK(cyclic.Append("Field",RigExecSceneCopyOp{1,2},&retainedError));
    CHECK(cyclic.Append("Commit",RigExecSceneCopyOp{2,3},&retainedError));
    CHECK(cyclic.Compile(RigExecCyclePolicy::SetAside,&retainedError));
    RigExecOpExclusionProof authority;
    authority.keys={"Solve","Field"};authority.removedKeys={"Commit"};
    authority.cycles=cyclic.graph.cycles;authority.cycleMembers=cyclic.graph.cycleMembers;
    RigExecSceneProgram normalized;
    normalized.layout.valueKeys=cyclic.layout.valueKeys;normalized.layout.leaves={0};
    CHECK(normalized.Append("Solve",RigExecSceneCopyOp{0,1},&retainedError));
    CHECK(normalized.Append("Field",RigExecSceneCopyOp{1,2},&retainedError));
    CHECK(normalized.Compile(RigExecCyclePolicy::SetAside,&authority,&retainedError));
    CHECK(normalized.graph.ops.empty());
    CHECK(normalized.Compile(RigExecCyclePolicy::SetAside,nullptr));
    CHECK(normalized.graph.ops.empty() && normalized.graph.cycleMembers==authority.cycleMembers);
    const auto normalizedIdentity=normalized.identity;
    authority.keys.push_back("Field");
    CHECK(!normalized.Compile(RigExecCyclePolicy::SetAside,&authority,&retainedError));
    CHECK(normalized.identity==normalizedIdentity && normalized.graph.ops.empty());
    auto stage=MakeExportRig("ExternalQuadraticMover");
    RigExecUsdSceneAccess source(stage);RigExecSceneDescriptors scene;std::string error;
    CHECK(RigExecCaptureSceneDescriptors(source,kRig,{UsdTimeCode::Default(),UsdTimeCode(1),UsdTimeCode(3)},&scene,&error));
    RigExecSceneProgram program;CHECK(RigExecLowerSceneProgram(scene,&program,&error));
    CHECK(program.Compile(RigExecCyclePolicy::SetAside,&error));
    const auto preservedIdentity=program.identity;
    const auto preservedPublic=program.publicValues;
    const auto preservedMembers=program.graph.cycleMembers;
    auto invalidScene=scene;invalidScene.identities.clear();
    error.clear();
    CHECK(!RigExecLowerSceneProgram(invalidScene,&program,&error));
    CHECK(!error.empty());
    CHECK(program.identity==preservedIdentity && program.publicValues==preservedPublic &&
          program.graph.cycleMembers==preservedMembers);
    RigExecValueId output=UINT64_MAX;
    CHECK(program.layout.routes.Resolve(scene,{kTarget,RigExecSceneValueDomain::Points},
        SdfPath("/Rig/External"),TfToken("final"),&output,&error));
    RigExecSceneProgramRuntime runtime;CHECK(runtime.Prepare(program,&error));
    for(size_t identity:{size_t(2),size_t(1),size_t(2)}) {
        CHECK(runtime.Evaluate(program,identity,{},&error));
        VtValue boxed;CHECK(RigExecReadSceneGraphValue(runtime.values,output,&boxed));
        CHECK(boxed.IsHolding<VtVec3fArray>());const auto &points=boxed.UncheckedGet<VtVec3fArray>();
        CHECK(points.size()==kBase.size());
        const float gain=identity==2?1.5f:0.5f,lift=identity==2?2.0f:0.0f;
        for(size_t i=0;i<points.size();++i) {
            const auto &base=kBase[i];const GfVec3f expected(base[0],base[1]+0.5f*lift,
                base[2]+0.5f*gain*base[0]*base[0]);
            CHECK((points[i]-expected).GetLength()<1e-6f);
        }
    }
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
    // The file holds the plugin mover's entry: its type, its epoch and the
    // frame bytes of the bake's run. The plugin's own reads stay inside
    // its payload; no path read stands for them.
    {
        std::unique_ptr<fb::RigExecWireFile> file;
        CHECK(RigExecFormatOpen(result.bytes.data(), result.bytes.size(),
                                &file, &error));
        CHECK(file->externalMovers.size() == 1);
        const fb::RigExecWireExternalMover &mover =
            file->externalMovers.front();
        CHECK(RigExecFormatPathText(*file, mover.type) == type);
        CHECK(mover.v2FrameValid);
        CHECK(!mover.epoch.empty() && !mover.v2Frame.empty());
    }

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

    error.clear();
    CHECK(!RigExecBakeToBinary(refused, options, &result, &error));
    CHECK(error.find("/Rig/External") != std::string::npos &&
          error.find("no .rigexec encoding") != std::string::npos);
}

// The plugin mover's `final` reference is a phased point read like any
// other: bound at Build to the reference chain's published points, which
// are the explicitly selected compiled points versions. Playback takes
// the same answer through its own overlay (TestExport), so the runtime's
// "have" flag for it is this binding's first candidate.
void TestPhasedReferenceIsBound()
{
    const auto stage = MakeExportRig("ExternalQuadraticMover");
    RigExecRigEvaluator baked(stage, kRig);

    Compile(&baked);
    for (double frame : {1.0, 2.0, 3.0}) {
        bool captured = false;
        if (const RigExecBakedProgram *program = baked.GetBakedProgram()) {
            captured = RigExecBakedProgramTesting::CapturePointReads(*program);
        }
        const size_t generations = baked.GetBakedGenerationCount();
        const RigExecRigPose pose = baked.Evaluate(UsdTimeCode(frame));
        CHECK(pose.valid);
        CHECK(baked.GetBakedGenerationCount() == generations + 1);
        const RigExecBakedProgram *program = baked.GetBakedProgram();
        CHECK(program);
        const RigExecBakedProgramImpl &B = program->GetStepGraph();
        const RigExecBakedPointsBinding *binding = nullptr;
        for (const auto &chain : B.chains) {
            for (const auto &revision : chain.revisions) {
                if (revision.moverPath == SdfPath("/Rig/External")) {
                    CHECK(revision.pointBindings.size() == 1);
                    binding = &revision.pointBindings.front();
                }
            }
        }
        CHECK(binding && binding->input == kReference &&
              binding->finalRead && binding->candidates.size() == 1);
        const GfVec3f *points = nullptr;
        size_t count = 0;
        CHECK(RigExecBakedResolvePoints(B, *binding, &points, &count));
        CHECK(VtVec3fArray(points, points + count) ==
              pose.movedProperties.at(kReference).Get<VtVec3fArray>());
        const auto &sourceChain=B.chains.at(size_t(binding->candidates.front().chain));
        CHECK(sourceChain.haveResult && sourceChain.target==binding->input &&
              sourceChain.result==VtVec3fArray(points,points+count));
        if (captured) {
            const RigExecBakedPointCapture &capture =
                B.pointCaptures.at(size_t(binding->id));
            CHECK(capture.read && capture.bindingAnswered &&
                  capture.bound == VtVec3fArray(points, points + count));
        }
    }
}

// The plugin mover's packet holds what assembleExternal answers at the run's
// time. Declared input leaves are sampled by the prologue; assembly always
// runs stage-free in RevisionStatic, reading the exact phase overlay. Either way,
// after baked runs at frames 1 and 3 and after the bake's run, the packet's
// payload equals a fresh call's over the same provider values, and the two
// encode to the same bytes -- after the bake, the bytes the file holds.
void TestTheExternalPayloadIsTheAssemblersAnswer()
{
    const std::string type = "ExternalQuadraticMover";
    const RigExecMoverHandler *handler = RigExecFindMoverHandler(TfToken(type));
    CHECK(handler && handler->encodeExternal);
    for (const bool phased : {false, true}) {
        const auto stage = MakeExportRig(type.c_str());
        if (!phased) {
            CHECK(stage->GetPrimAtPath(SdfPath("/Rig/External"))
                      .GetRelationship(TfToken("rigExec:reference"))
                      .ClearTargets(true));
        }
        RigExecRigEvaluator baked(stage, kRig);

        Compile(&baked);
        const std::string label = phased ? "phased" : "prologue";
        // The frame bytes of the packet's payload, after checking it.
        const auto check = [&](double time, const std::string &what) {
            const RigExecBakedProgram *program = baked.GetBakedProgram();
            CHECK(program);
            const RigExecBakedProgramImpl &B = program->GetStepGraph();
            const RigExecBakedProgramImpl::GeomChain *chain = nullptr;
            const RigExecBakedProgramImpl::GeomRevision *revision = nullptr;
            for (const auto &c : B.chains) {
                for (const auto &r : c.revisions) {
                    if (r.moverPath == SdfPath("/Rig/External")) {
                        chain = &c;
                        revision = &r;
                    }
                }
            }
            CHECK(chain && revision);
            CHECK(revision->leaves.decl.assembles);
            CHECK(revision->binding.handler == handler);
            CHECK(revision->leaves.decl.externalBegin >= 0);
            CHECK(revision->binding.externalInputs.size() == (phased ? 2u : 1u));
            CHECK(revision->binding.phases.empty() == !phased);
            RigExecProviderValues values;
            values.basePoints.assign(chain->lastBase.begin(),
                                     chain->lastBase.end());
            values.resolved =
                phased ? &revision->revisionInputs : B.resolvedInputs;
            RigExecExternalPayload fresh;
            RigExecAssembleExternalPayload(revision->moverPrim,
                                           revision->binding, values,
                                           UsdTimeCode(time), &fresh);
            const RigExecMoverParameters &packet = revision->parameters;
            if (!(fresh.valid && packet.valid &&
                  packet.externalSchema == fresh.schema &&
                  packet.externalData == fresh.data)) {
                throw std::runtime_error(label + " " + what +
                                         ": the packet's payload is not the "
                                         "assembler's answer");
            }
            std::vector<uint8_t> epochA, frameA, epochB, frameB;
            CHECK(handler->encodeExternal(packet.externalData,
                                          revision->binding, &epochA,
                                          &frameA));
            CHECK(handler->encodeExternal(fresh.data, revision->binding,
                                          &epochB, &frameB));
            CHECK(epochA == epochB && frameA == frameB);
            return frameA;
        };
        for (const double frame : {1.0, 3.0}) {
            CHECK(baked.Evaluate(UsdTimeCode(frame)).valid);
            check(frame, "frame " + std::to_string(int(frame)));
        }
        RigExecBakeOpts options;
        options.time = 1.0;
        RigExecBakeResult result;
        std::string error;
        if (!RigExecBakeToBinary(baked, options, &result, &error)) {
            throw std::runtime_error(label + " export failed: " + error);
        }
        const std::vector<uint8_t> bytes = check(1.0, "bake");
        std::unique_ptr<fb::RigExecWireFile> file;
        CHECK(RigExecFormatOpen(result.bytes.data(), result.bytes.size(),
                                &file, &error));
        CHECK(file && file->externalMovers.size() == 1);
        if (file && file->externalMovers.size() == 1) {
            const fb::RigExecWireExternalMover &mover =
                file->externalMovers.front();
            CHECK(std::vector<uint8_t>(mover.v2Frame.begin(),
                                       mover.v2Frame.end()) == bytes);
        }
    }
}

void TestExternalFailureAtomicAndStagingRecovery()
{
    RigExecMoverHandler handler("TestDiscardableExternal", nullptr, RigExecMoverDomain::Points);
    handler.applyExternal=[](const VtValue &mode,std::vector<GfVec3f> *points) {
        const int kind=mode.UncheckedGet<int>();
        (*points)[0][0]+=2.0f;
        if(kind==0)return false;
        if(kind==1){points->resize(points->size()+1);return true;}
        if(kind==2){(*points)[0][0]=std::numeric_limits<float>::quiet_NaN();return true;}
        return true;
    };
    RigExecMoverParameters parameters;
    parameters.kind=TfToken("external");parameters.enabled=true;parameters.valid=true;
    parameters.externalHandler=&handler;
    const std::vector<GfVec3f> entering(kBase.begin(),kBase.end());
    for(float strength:{1.0f,.5f})for(int mode=0;mode<3;++mode) {
        parameters.weights=RigExecWeightPacket::Constant(strength);
        parameters.externalData=VtValue(mode);
        auto points=entering;
        CHECK(!RigExecApplyRevisionKernel(RigExecRevisionOp::External,parameters,&points,false,nullptr));
        CHECK(points==entering);
        CHECK(!RigExecRunRevisionKernel(RigExecRevisionOp::External,parameters,&points,false,nullptr));
        CHECK(points==entering);
    }
    RigExecBakedProgramImpl program;
    RigExecResolvedInputs resolved;program.resolvedInputs=&resolved;
    program.chains.resize(1);auto &chain=program.chains[0];
    chain.haveBase=true;chain.lastBase=kBase;chain.revisions.resize(1);
    program.revisionIndex.emplace_back(0,0);auto &revision=chain.revisions[0];
    revision.op=RigExecRevisionOp::External;revision.moverPath=SdfPath("/External");
    revision.chunks.resize(1);revision.precedingCount=kBase.size();
    RigExecBakedStep chunk;chunk.kind=RigExecBakedStepKind::RevisionChunk;chunk.object=0;chunk.part=0;
    RigExecBakedStep fuse;fuse.kind=RigExecBakedStepKind::RevisionFuse;fuse.object=0;
    RigExecBakedStep publish;publish.kind=RigExecBakedStepKind::ChainStatus;publish.object=0;
    for(float strength:{1.0f,.5f})for(int mode:{0,3,1,3,2,3}) {
        parameters.weights=RigExecWeightPacket::Constant(strength);parameters.externalData=VtValue(mode);
        revision.parameters=parameters;revision.status=RigExecStatusForParameters(parameters,revision.moverPath);
        // RevisionStatic's decision, which the fuse selects by.
        revision.acceptance=RigExecRevisionKernelAcceptance(revision.op,parameters,kBase.size());
        RigExecBakedRunGeometryStep(&program,&chunk,UsdTimeCode::Default());
        RigExecBakedRunGeometryStep(&program,&fuse,UsdTimeCode::Default());
        RigExecBakedRunGeometryStep(&program,&publish,UsdTimeCode::Default());
        CHECK(revision.chunks[0].ok==(mode==3));
        CHECK(chain.haveResult && chain.result.size()==kBase.size());
        auto expected=kBase;if(mode==3)expected[0][0]+=2.0f*strength;
        CHECK(chain.result==expected);
        CHECK(revision.currentSource==(mode==3?0:-1));
    }
}

void TestExactDoubleLeafRoutes()
{
    RigExecBakedProgramImpl native;
    native.propertyChains.resize(1);native.propertyChains[0].arm=RigExecBakedPropertyChain::Arm::Float;
    native.propertyVersionValid={1};native.propertyValues.resize(1);native.propertyValues[0].f=.5f;
    native.headLeaves.resize(1);native.headLeaves[0].value=VtValue(.75);
    native.readerWalks.resize(1);auto &walk=native.readerWalks[0].walk;
    walk.flavour=RigExecBakedWalk::Flavour::Connected;walk.type=RigExecBakedHeadValueType::Double;
    walk.hops.emplace_back();walk.hops[0].leaf=0;
    RigExecBakedPathLeaves leaves;
    leaves.decl.Add({SdfPath("/Read.value"),RigExecRevisionLeafType::Double,
        RigExecRevisionLeafTime::AtTime,RigExecRevisionLeafFlavour::ResolvedOnly,VtValue(.25)});
    leaves.values={VtValue(.1)};leaves.exactVersions={0};leaves.exactRecordIndices={-1};
    leaves.exactValueTypes={int(RigExecBakedPropertyChain::Arm::Float)};leaves.walks={0};
    CHECK(RigExecBakedResolvePathLeaf(native,leaves,0).UncheckedGet<double>()==.75);
    native.propertyRecords.resize(1);native.propertyRecords[0].chain=0;
    native.chainValid={1};walk.hops[0].record=0;
    native.recordStoodAside={0};native.recordValues={VtValue(.5f)};leaves.exactRecordIndices[0]=0;
    CHECK(RigExecBakedResolvePathLeaf(native,leaves,0).UncheckedGet<double>()==.75);
    leaves.decl.keys[0].type=RigExecRevisionLeafType::Dial;
    walk.type=RigExecBakedHeadValueType::Float;
    CHECK(RigExecBakedResolvePathLeaf(native,leaves,0).UncheckedGet<double>()==.5);

    RigExecWireFile file;file.inputs.emplace_back(0,0,-1,-1,RigExecWireInputTag::Double,0);
    RrProgram runtime;runtime.inputState.file=&file;runtime.store.propertyVersions.resize(1);
    runtime.store.propertyVersions[0].tag=RrPropertyValue::Tag::Float;
    runtime.store.propertyVersions[0].f32=.5f;runtime.store.propertyVersionValid={1};
    runtime.store.propertyPublishedVersions={0};
    runtime.store.propertyRecordStoodAside={0};
    RrWireValue fallback;fallback.tag=RigExecWireInputTag::Double;const double quarter=.25,raw=.75;
    std::memcpy(&fallback.bits,&quarter,sizeof(quarter));runtime.inputState.values={fallback};
    RrWireValue current=fallback;std::memcpy(&current.bits,&raw,sizeof(raw));
    runtime.inputState.slotCurrent={current};runtime.inputState.slotHasValue={1};
    runtime.inputState.slotAuthored={0};runtime.inputState.slotDefaultHasValue={1};
    RigExecWireExternalDeclaredInput site;site.exactVersion=0;site.exactRecord=0;site.fallbackHasValue=true;
    site.flavour=fb::ExternalInputFlavour::ResolvedOnly;site.read=std::make_unique<RigExecWireInput>();
    site.read->tag=RigExecWireInputTag::Double;site.read->mode=RigExecWireReadMode::Resolved;
    site.bodyWalk=std::make_unique<RigExecWireInput>(*site.read);
    RigExecWirePropertyInputCandidate hop;hop.slot=0;hop.raw=true;
    hop.kind=uint8_t(fb::PropertyCandidateKind::PhasedRecord);hop.version=0;
    site.bodyWalk->propertyCandidates.push_back(hop);
    RrWireValue result;double answer=0;
    CHECK(RrReadExternalScalar(&runtime,site,&result));std::memcpy(&answer,&result.bits,sizeof(answer));CHECK(answer==raw);
    site.allowFloatToDouble=true;
    CHECK(RrReadExternalScalar(&runtime,site,&result));std::memcpy(&answer,&result.bits,sizeof(answer));CHECK(answer==.5);
    site.allowFloatToDouble=false;runtime.store.propertyVersionValid[0]=0;
    CHECK(RrReadExternalScalar(&runtime,site,&result));std::memcpy(&answer,&result.bits,sizeof(answer));CHECK(answer==quarter);
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
        metadata << R"({"Plugins": [{"Type": "library", "Name": "incompatibleExternalMover", "Root": ".", "ResourcePath": ".", "LibraryPath": "never-loaded", "Info": {"RigExecMoverPlugin": 3}}]})";
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
    CHECK(diagnostics.front().find("expected 4") != std::string::npos);
    CHECK(!plugin->IsLoaded());
}

void TestCapturedMetadataOwnership()
{
    RigExecScenePrim heldPrim;RigExecSceneAttribute heldAttribute;
    RigExecSceneRelationship heldRelationship;
    VtDictionary expectedPrim,expectedAttribute,expectedRelationship;
    {
        const auto stage=UsdStage::CreateInMemory();
        const auto prim=UsdGeomPoints::Define(stage,SdfPath("/Metadata")).GetPrim();
        VtDictionary nested;nested["owned"]=VtValue(VtVec3fArray{GfVec3f(1,2,3)});
        prim.SetCustomData(nested);CHECK(prim.SetDocumentation("metadata ownership"));
        const auto attr=prim.CreateAttribute(TfToken("value"),SdfValueTypeNames->Double);
        CHECK(attr.Set(0.25));CHECK(attr.Set(0.5,UsdTimeCode(1)));
        attr.SetCustomData(nested);CHECK(attr.SetConnections({SdfPath("/Other.value")}));
        const auto rel=prim.CreateRelationship(TfToken("targets"));
        rel.SetCustomData(nested);CHECK(rel.SetTargets({SdfPath("/Other")}));
        const auto reference=[](const UsdObject &object) {
            VtDictionary result;
            for(const auto &[key,value]:object.GetAllMetadata())
                if(key!=TfToken("default")&&key!=TfToken("timeSamples")&&key!=TfToken("spline")&&
                   key!=TfToken("connectionPaths")&&key!=TfToken("targetPaths"))result[key.GetString()]=value;
            return result;
        };
        expectedPrim=reference(prim);expectedAttribute=reference(attr);expectedRelationship=reference(rel);
        CHECK(!expectedPrim.empty()&&!expectedAttribute.empty()&&!expectedRelationship.empty());
        const RigExecUsdSceneAccess source(stage);
        CHECK(source.Prim(prim.GetPath(),&heldPrim));CHECK(source.Attribute(attr.GetPath(),&heldAttribute));
        CHECK(source.Relationship(rel.GetPath(),&heldRelationship));
        CHECK(heldPrim.metadata==expectedPrim&&heldAttribute.metadata==expectedAttribute);
        CHECK(heldRelationship.metadata==expectedRelationship);
        // Built-in fallback metadata must also survive, even without an authored value.
        const auto fallback=prim.GetAttribute(TfToken("visibility"));
        RigExecSceneAttribute fallbackFacts;CHECK(source.Attribute(fallback.GetPath(),&fallbackFacts));
        CHECK(fallbackFacts.metadata==reference(fallback));
        CHECK(heldAttribute.connections==SdfPathVector{SdfPath("/Other.value")});
        CHECK(heldRelationship.targets==SdfPathVector{SdfPath("/Other")});
        VtDictionary changed;changed["owned"]=VtValue(VtVec3fArray{GfVec3f(7,8,9)});
        prim.SetCustomData(changed);attr.SetCustomData(changed);rel.SetCustomData(changed);
        RigExecSceneAttribute newer;CHECK(source.Attribute(attr.GetPath(),&newer));
        CHECK(newer.metadata!=heldAttribute.metadata);
    }
    CHECK(heldPrim.metadata==expectedPrim&&heldAttribute.metadata==expectedAttribute);
    CHECK(heldRelationship.metadata==expectedRelationship);
}

class GenericUsdCapture final : public RigExecSceneAccess {
public:
    explicit GenericUsdCapture(const RigExecUsdSceneAccess &source):source(source){}
    const RigExecUsdSceneAccess &source;
    SdfPathVector Prims(const SdfPath &p) const override{return source.Prims(p);}
    bool Prim(const SdfPath &p,RigExecScenePrim *v) const override{return source.Prim(p,v);}
    SdfPathVector Attributes(const SdfPath &p) const override{return source.Attributes(p);}
    SdfPathVector Relationships(const SdfPath &p) const override{return source.Relationships(p);}
    bool Attribute(const SdfPath &p,RigExecSceneAttribute *v) const override{return source.Attribute(p,v);}
    bool Relationship(const SdfPath &p,RigExecSceneRelationship *v) const override{return source.Relationship(p,v);}
    bool Resolve(const SdfPath &p,UsdTimeCode t,VtValue *v) const override{return source.Resolve(p,t,v);}
    bool ValueBlocked(const SdfPath &p,UsdTimeCode t) const override{return source.ValueBlocked(p,t);}
    bool HasIdentity(UsdTimeCode t) const override{return source.HasIdentity(t);}
    double TimeCodesPerSecond() const override{return source.TimeCodesPerSecond();}
    double FramesPerSecond() const override{return source.FramesPerSecond();}
    TfToken Interpolation() const override{return source.Interpolation();}
    TfToken UpAxis() const override{return source.UpAxis();}
};
void CheckSameAttribute(const RigExecSceneAttribute &a,const RigExecSceneAttribute &b)
{
    CHECK(a.path==b.path&&a.type==b.type&&a.metadata==b.metadata&&a.connections==b.connections);
    CHECK(a.hasValue==b.hasValue&&a.hasAuthoredValue==b.hasAuthoredValue);
    CHECK(a.hasAuthoredReadableValue==b.hasAuthoredReadableValue&&a.hasAuthoredConnections==b.hasAuthoredConnections);
    CHECK(a.mightBeTimeVarying==b.mightBeTimeVarying&&a.hasAuthoredDefault==b.hasAuthoredDefault);
    CHECK(a.defaultBlocked==b.defaultBlocked&&a.sampleTimes==b.sampleTimes&&a.variability==b.variability&&a.spline==b.spline);
}
void TestBoundUsdAttributeCapture()
{
    RigExecSceneDescriptors held;
    {
        const auto stage=UsdStage::CreateInMemory();RigExecRigBuilder::Create(stage,kRig);
        const auto prim=stage->DefinePrim(SdfPath("/Rig/Data"));
        const auto value=prim.CreateAttribute(TfToken("value"),SdfValueTypeNames->Double);
        CHECK(value.Set(0.25));CHECK(value.Set(0.5,UsdTimeCode(1)));
        const auto connected=prim.CreateAttribute(TfToken("connected"),SdfValueTypeNames->Double);
        CHECK(connected.Set(0.125));CHECK(connected.SetConnections({value.GetPath()}));
        const auto blocked=prim.CreateAttribute(TfToken("blocked"),SdfValueTypeNames->Double);blocked.Block();
        const auto inactive=stage->DefinePrim(SdfPath("/Rig/Inactive"));
        const auto inactiveValue=inactive.CreateAttribute(TfToken("value"),SdfValueTypeNames->Double);
        CHECK(inactiveValue.Set(0.75));CHECK(inactive.SetActive(false));
        CHECK(prim.CreateRelationship(TfToken("targets")).SetTargets({value.GetPath(),inactive.GetPath()}));
        const RigExecUsdSceneAccess typed(stage);const GenericUsdCapture generic(typed);
        const std::vector<UsdTimeCode> identities={UsdTimeCode::Default(),UsdTimeCode(1)};
        RigExecSceneDescriptors old;std::string typedError,genericError;
        CHECK(RigExecCaptureSceneDescriptors(typed,kRig,identities,&held,&typedError));
        CHECK(RigExecCaptureSceneDescriptors(generic,kRig,identities,&old,&genericError));
        CHECK(typedError==genericError&&held.identities==old.identities&&held.compilerTargets==old.compilerTargets);
        CHECK(held.rigRoot==old.rigRoot&&held.timeCodesPerSecond==old.timeCodesPerSecond&&held.framesPerSecond==old.framesPerSecond);
        CHECK(held.interpolation==old.interpolation&&held.upAxis==old.upAxis);
        CHECK(held.nodes.size()==old.nodes.size()&&held.attributes.size()==old.attributes.size());
        CHECK(held.relationships.size()==old.relationships.size());
        for(const auto &entry:held.relationships) {
            const auto &a=entry.second;const auto &b=old.relationships.at(entry.first);
            CHECK(a.fact.path==b.fact.path&&a.fact.metadata==b.fact.metadata&&a.fact.targets==b.fact.targets);
            CHECK(a.readPhase==b.readPhase&&a.forwardedTargets==b.forwardedTargets&&a.targetExists==b.targetExists);
        }
        CHECK(held.applications.empty()&&old.applications.empty()&&held.jointBindings.empty()&&old.jointBindings.empty());
        for(const auto &entry:held.nodes) {
            const auto &a=entry.second;const auto &b=old.nodes.at(entry.first);
            CHECK(a.fact.path==b.fact.path&&a.fact.type==b.fact.type&&a.fact.appliedSchemas==b.fact.appliedSchemas);
            CHECK(a.fact.metadata==b.fact.metadata&&a.fact.active==b.fact.active&&a.fact.children==b.fact.children);
            CHECK(a.attributes==b.attributes&&a.relationships==b.relationships&&a.stackOrdinal==b.stackOrdinal);
            CHECK(a.domain==b.domain&&a.transformProvider==b.transformProvider&&a.pointBased==b.pointBased);
        }
        for(const auto &entry:held.attributes) {
            const auto &a=entry.second;const auto &b=old.attributes.at(entry.first);CheckSameAttribute(a.fact,b.fact);
            CHECK(a.readPhase==b.readPhase&&a.inputs.size()==b.inputs.size());
            for(size_t i=0;i<a.inputs.size();++i) {
                const auto &x=a.inputs[i];const auto &y=b.inputs[i];
                CHECK(x.raw==y.raw&&x.resolved==y.resolved&&x.rawBlocked==y.rawBlocked&&x.resolvedBlocked==y.resolvedBlocked);
                CHECK(x.state==y.state&&x.source==y.source&&x.hops==y.hops);
            }
        }
        CHECK(held.attributes.at(blocked.GetPath()).inputs[0].rawBlocked);
        CHECK(held.attributes.at(inactiveValue.GetPath()).inputs[0].raw.IsEmpty());
        CHECK(value.Set(0.875));RigExecSceneDescriptors newer;
        CHECK(RigExecCaptureSceneDescriptors(typed,kRig,identities,&newer,&typedError));
        CHECK(newer.attributes.at(value.GetPath()).inputs[0].raw==VtValue(0.875));
        CHECK(held.attributes.at(value.GetPath()).inputs[0].raw==VtValue(0.25));
        CHECK(value.SetMetadata(TfToken("rigExecReadPhase"),VtValue(TfToken("invalid-phase"))));
        typedError.clear();genericError.clear();
        CHECK(!RigExecCaptureSceneDescriptors(typed,kRig,identities,&newer,&typedError));
        CHECK(!RigExecCaptureSceneDescriptors(generic,kRig,identities,&newer,&genericError));
        CHECK(!typedError.empty()&&typedError==genericError);
        const auto handle=typed.BindAttribute(value.GetPath());VtValue result(1.0);
        CHECK(!typed.Resolve(handle,UsdTimeCode(std::numeric_limits<double>::infinity()),&result)&&result.IsEmpty());
        CHECK(!typed.Resolve(SdfPath("/Missing.value"),UsdTimeCode::Default(),&result));
    }
    CHECK(held.attributes.at(SdfPath("/Rig/Data.value")).inputs[0].raw==VtValue(0.25));
}

}  // namespace

int main(int argc, char **argv)
{
    try {
        CHECK(argc == 3);
        PlugRegistry::GetInstance().RegisterPlugins(argv[1]);
        TestRegistration(argv[2]);
        TestCapturedMetadataOwnership();
        TestBoundUsdAttributeCapture();
        TestExactDoubleLeafRoutes();
        TestExternalFailureAtomicAndStagingRecovery();
        TestDeclaredScalarPublicCensus();
        TestEvaluation();
        TestTimeVaryingInputsKeyFramesApart();
        TestDetachedExternalProgram();
        TestExport();
        TestPhasedReferenceIsBound();
        TestTheExternalPayloadIsTheAssemblersAnswer();
        TestIncompatiblePlugin();
        std::cout << "External mover loading, registration, scalar reference and canonical consistency, "
                     "invalidation, envelopes, export and playback passed\n";
    } catch (const std::exception &error) {
        std::cerr << error.what() << '\n';
        return 1;
    }
}
