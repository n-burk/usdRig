// S9 newly admitted singleton envelopes; the old walk refused this domain.
#include "rigExec/bakedProgram.h"
#include "rigExec/bakedProgramImpl.h"
#include "rigExec/rigEvaluator.h"
#include "pxr/base/plug/registry.h"
#include "pxr/base/tf/pathUtils.h"
#include "pxr/usd/sdf/types.h"
#include "pxr/usd/usd/stage.h"
#include <cmath>
#include <cstdio>
#include <string>

PXR_NAMESPACE_USING_DIRECTIVE
using namespace rigExec;
namespace {
int failures=0;
#define CHECK(value) do {if(!(value)){++failures; std::printf("FAIL %s:%d: %s\n",__FILE__,__LINE__,#value); std::fflush(stdout);}} while(0)
const SdfPath rig("/EnvelopeAsset/Rig");
bool Compile(RigExecRigEvaluator &evaluator) {
    std::vector<std::string> errors;
    const bool ok=evaluator.Compile(&errors);
    for(const auto &error:errors)std::printf("    %s\n",error.c_str());
    return ok;
}
float Scalar(const RigExecRigPose &pose,const char *name) {
    const auto found=pose.movedProperties.find(rig.AppendPath(SdfPath("Channels")).AppendProperty(TfToken(name)));
    CHECK(found!=pose.movedProperties.end());
    if(found==pose.movedProperties.end())return -999;
    CHECK(found->second.IsHolding<float>());
    return found->second.IsHolding<float>()?found->second.UncheckedGet<float>():-999;
}
double JointX(const RigExecRigPose &pose,const char *name) {
    const auto found=pose.jointMatricesFinal.find(rig.AppendPath(SdfPath("Joints")).AppendChild(TfToken(name)));
    CHECK(found!=pose.jointMatricesFinal.end());
    return found==pose.jointMatricesFinal.end()?-999:found->second.ExtractTranslation()[0];
}
bool CountFailure(const RigExecRigPose &pose) {
    for(const auto &line:pose.diagnostics)
        if(line.find("sampled point count does not match the target")!=std::string::npos)return true;
    return false;
}
void PlacementGeometryAndRecovery(const std::string &fixture) {
    auto stage=UsdStage::Open(fixture);CHECK(stage);if(!stage)return;
    RigExecRigEvaluator evaluator(stage,rig);
    const bool compiled=Compile(evaluator);CHECK(compiled);if(!compiled)return;
    const auto first=evaluator.Evaluate(UsdTimeCode(1));CHECK(first.valid);
    CHECK(evaluator.GetBakedProgram());
    if(!first.valid || !evaluator.GetBakedProgram()) {
        for(const auto &line:first.diagnostics)std::printf("    %s\n",line.c_str());
        return;
    }
    CHECK(Scalar(first,"base")==10);CHECK(Scalar(first,"final")==0);
    CHECK(Scalar(first,"geometry")==5);
    CHECK(JointX(first,"Base")==6);CHECK(JointX(first,"Final")==0);
    const auto &B=evaluator.GetBakedProgram()->GetStepGraph();
    size_t property=0,constraint=0,base=0,final=0;
    for(const auto &field:B.weightFields) {
        using F=RigExecBakedProgramImpl::WeightField;
        if(field.volumes.empty() || field.form==F::Form::Revision)continue;
        property+=field.form==F::Form::EnvelopeProperty;
        constraint+=field.form==F::Form::EnvelopeConstraint;
        base+=field.placementPhase==F::PlacementPhase::Base;
        final+=field.placementPhase==F::PlacementPhase::Final;
    }
    CHECK(property==3 && constraint==2 && base==3 && final==2);
    bool geometryEdge=false;
    for(const auto &step:B.steps)if(step.kind==RigExecBakedStepKind::WeightField)
        for(const auto &read:step.reads)if(read.domain==RigExecBakedSlotDomain::ChainPoints)geometryEdge=true;
    CHECK(geometryEdge); // Final sample points are a real same-frame producer.
    const auto builds=evaluator.GetBakedProgramBuildCount();
    const auto shortField=evaluator.Evaluate(UsdTimeCode(2));CHECK(shortField.valid);
    CHECK(Scalar(shortField,"base")==0 && JointX(shortField,"Base")==0);
    CHECK(Scalar(shortField,"geometry")==5);CHECK(CountFailure(shortField));
    const auto recovered=evaluator.Evaluate(UsdTimeCode(3));CHECK(recovered.valid);
    CHECK(Scalar(recovered,"base")==10 && JointX(recovered,"Base")==6);
    CHECK(!CountFailure(recovered));CHECK(evaluator.GetBakedProgramBuildCount()==builds);
    const auto empty=evaluator.Evaluate(UsdTimeCode(4));CHECK(empty.valid);
    CHECK(Scalar(empty,"base")==0 && JointX(empty,"Base")==0);CHECK(CountFailure(empty));
    const auto blocked=evaluator.Evaluate(UsdTimeCode(5));CHECK(blocked.valid);
    CHECK(Scalar(blocked,"base")==0 && JointX(blocked,"Base")==0);
    bool unavailable=false;
    for(const auto &line:blocked.diagnostics)
        unavailable|=line.find("could not read the points to sample")!=std::string::npos;
    CHECK(unavailable);CHECK(Scalar(blocked,"geometry")==5);
    const auto restored=evaluator.Evaluate(UsdTimeCode(6));CHECK(restored.valid);
    CHECK(Scalar(restored,"base")==10 && JointX(restored,"Base")==6);
    CHECK(!CountFailure(restored));CHECK(evaluator.GetBakedProgramBuildCount()==builds);
}
void FinalPlacementCycle(const std::string &fixture) {
    auto stage=UsdStage::Open(fixture);CHECK(stage);if(!stage)return;
    const auto sphere=stage->DefinePrim(rig.AppendPath(SdfPath("Weights/Self")),TfToken("RigExecSphereWeight"));
    CHECK(sphere.CreateRelationship(TfToken("rigExec:weightTarget")).SetTargets({sphere.GetPath()}));
    CHECK(sphere.CreateRelationship(TfToken("rigExec:sampleSource")).SetTargets({SdfPath("/EnvelopeAsset/Geom/Samples.points")}));
    const auto mover=stage->DefinePrim(rig.AppendPath(SdfPath("Constraints/Self")),TfToken("RigExecPositionConstraint"));
    CHECK(mover.ApplyAPI(TfToken("RigExecMoverAPI")));
    CHECK(mover.CreateRelationship(TfToken("rigExec:moves")).SetTargets({sphere.GetPath()}));
    CHECK(mover.CreateRelationship(TfToken("rigExec:sources")).SetTargets({rig.AppendPath(SdfPath("Controls/VolumeMove"))}));
    const auto weight=mover.CreateRelationship(TfToken("rigExec:weightObject"));
    CHECK(weight.SetTargets({sphere.GetPath()}));CHECK(weight.SetMetadata(TfToken("rigExecReadPhase"),VtValue(std::string("final"))));
    RigExecReadPhase selectedPhase;std::string phaseError;
    CHECK(RigExecResolveReadPhase(weight,&selectedPhase,&phaseError));
    CHECK(selectedPhase.kind==RigExecReadPhaseKind::Final);
    RigExecRigEvaluator evaluator(stage,rig);
    const bool compiled=Compile(evaluator);CHECK(compiled);if(!compiled)return;
    const auto first=evaluator.Evaluate(UsdTimeCode(1));CHECK(first.valid);
    CHECK(evaluator.GetBakedProgram());
    if(!first.valid || !evaluator.GetBakedProgram()) {
        for(const auto &line:first.diagnostics)std::printf("    %s\n",line.c_str());
        return;
    }
    const auto &B=evaluator.GetBakedProgram()->GetStepGraph();
    bool cycle=false;
    for(const auto &loop:B.opGraph.cycles) {
        bool field=false,move=false;
        for(const auto &key:loop) {
            field|=key.find("Weights/Self")!=std::string::npos;
            move|=key.find("Constraints/Self")!=std::string::npos;
        }
        cycle|=field && move;
    }
    if (!cycle) {
        for (const auto &step : B.steps) {
            if (step.label.find("Self") == std::string::npos &&
                step.kind != RigExecBakedStepKind::VolumePlacements) continue;
            std::printf("cycle descriptor %s object=%d\n", step.label.c_str(), step.object);
            for (const auto &read : step.reads)
                std::printf("  read domain=%u [%u,%u)\n", unsigned(read.domain), read.begin, read.end);
            for (const auto &write : step.writes)
                std::printf("  write domain=%u [%u,%u)\n", unsigned(write.domain), write.begin, write.end);
        }
    }
    CHECK(cycle);CHECK(!B.opAdapter.excludedValues.empty());
    const auto slot=B.index.find(sphere.GetPath());CHECK(slot!=B.index.end());
    bool excludedPlacement=false;
    if(slot!=B.index.end())for(const auto id:B.opAdapter.excludedValues) {
        const auto &value=B.opAdapter.values[size_t(id)];
        excludedPlacement|=value.domain==uint32_t(RigExecBakedSlotDomain::WeightFrames) &&
                           value.slot==uint32_t(slot->second);
    }
    CHECK(excludedPlacement);
    const auto invalidPublication=[&](const RigExecRigPose &pose) {
        if(slot!=B.index.end()) {
            const size_t provider=size_t(slot->second);
            const auto &surviving=B.fin[size_t(B.finLast[provider])];
            const auto &baseFrame=B.base[size_t(B.baseLast[provider])];
            CHECK(surviving.IsValid() && surviving==baseFrame);
            CHECK(!std::isfinite(B.volumePlacement[provider][3][0]));
        }
        const auto frame=pose.weightFrames.find(sphere.GetPath());
        CHECK(frame!=pose.weightFrames.end());
        if(frame!=pose.weightFrames.end())CHECK(!std::isfinite(frame->second[3][0]));
    };
    CHECK(Scalar(first,"base")==10);
    invalidPublication(first);
    const auto later=evaluator.Evaluate(UsdTimeCode(3));CHECK(later.valid);CHECK(Scalar(later,"base")==10);
    invalidPublication(later);
}
}
int main(int argc,char **argv) {
    if(argc<2)return 2;
#ifdef RIGEXEC_SCHEMA_RESOURCE_DIR
    const auto resources=TfAbsPath(RIGEXEC_SCHEMA_RESOURCE_DIR);
#else
    const auto resources=TfAbsPath(std::string(argv[1])+"/../../plugin/rigExecSchema/resources");
#endif
    if(PlugRegistry::GetInstance().RegisterPlugins(resources).empty())return 2;
    const std::string fixture=std::string(argv[1])+"/oneloop_s9_envelope_volumes.usda";
    PlacementGeometryAndRecovery(fixture);FinalPlacementCycle(fixture);
    std::printf("testRigExecEnvelopeVolumes: %d failures\n",failures);return failures?1:0;
}
