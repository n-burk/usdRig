// S9 new capabilities. Existing scalar-reference golden rigs are unchanged.
#include "rigExec/bakedProgram.h"
#include "rigExec/bakedProgramImpl.h"
#include "rigExec/rigEvaluator.h"
#include "pxr/base/plug/registry.h"
#include "pxr/base/tf/pathUtils.h"
#include "pxr/usd/usd/stage.h"
#include "pxr/usd/sdf/types.h"
#include "pxr/usd/sdf/layer.h"
#include <algorithm>
#include <atomic>
#include <cstdio>
#include <memory>

PXR_NAMESPACE_USING_DIRECTIVE
using namespace rigExec;
namespace {
int failures = 0;
#define CHECK(condition) do { if (!(condition)) { ++failures; \
    std::printf("FAIL %s:%d: %s\n",__FILE__,__LINE__,#condition); } } while (0)
const SdfPath rigPath("/S9Asset/Rig");
const SdfPath rawValue("/S9Asset/Rig/Channels.rawSelected");
const SdfPath downstream("/S9Asset/Rig/Channels.downstream");

bool Compile(RigExecRigEvaluator &evaluator)
{
    std::vector<std::string> errors;
    if (evaluator.Compile(&errors)) return true;
    for (const auto &error : errors) std::printf("    %s\n",error.c_str());
    return false;
}
bool Ready(const RigExecRigPose &pose)
{
    CHECK(pose.valid);
    if (!pose.valid) for (const auto &line:pose.diagnostics)
        std::printf("    %s\n",line.c_str());
    return pose.valid;
}
GfVec3f Vector(const RigExecRigPose &pose,const SdfPath &path)
{
    const auto found = pose.movedProperties.find(path);
    CHECK(found != pose.movedProperties.end());
    if (found == pose.movedProperties.end()) return GfVec3f(-999);
    CHECK(found->second.IsHolding<GfVec3f>());
    return found->second.IsHolding<GfVec3f>() ? found->second.UncheckedGet<GfVec3f>() : GfVec3f(-999);
}
bool Unavailable(const RigExecRigPose &pose)
{
    for (const auto &line : pose.diagnostics)
        if (line.find("/S9Asset/Rig/Properties/RawSelected.inputs:value") != std::string::npos &&
            line.find("element 1 is unavailable") != std::string::npos) return true;
    return false;
}

void ReadVersions(const std::string &fixture)
{
    auto stage = UsdStage::Open(fixture); CHECK(stage); if (!stage) return;
    RigExecRigEvaluator evaluator(stage,rigPath);
    const bool compiled=Compile(evaluator); CHECK(compiled); if(!compiled)return;
    const auto pose = evaluator.Evaluate(UsdTimeCode(1)); if(!Ready(pose))return;
    CHECK(evaluator.GetBakedProgram()); if(!evaluator.GetBakedProgram())return;
    CHECK(Vector(pose,SdfPath("/S9Asset/Rig/Channels.selected")) == GfVec3f(1,2,0));
    CHECK(Vector(pose,SdfPath("/S9Asset/Rig/Channels.base")) == GfVec3f(1,0,0));
    CHECK(Vector(pose,SdfPath("/S9Asset/Rig/Channels.checkpoint")) == GfVec3f(1,2,0));
    CHECK(Vector(pose,SdfPath("/S9Asset/Rig/Channels.preceding")) == GfVec3f(1,0,0));
    const auto matrix = pose.movedProperties.find(SdfPath("/S9Asset/Rig/Channels.pose"));
    CHECK(matrix != pose.movedProperties.end());
    if (matrix != pose.movedProperties.end()) {
        CHECK(matrix->second.IsHolding<GfMatrix4d>());
        if (matrix->second.IsHolding<GfMatrix4d>())
            CHECK(matrix->second.UncheckedGet<GfMatrix4d>().ExtractTranslation() == GfVec3d(0,2,0));
    }
    const auto frames = pose.solverFrames.find(SdfPath("/S9Asset/Rig/Solvers/Ribbon"));
    CHECK(frames != pose.solverFrames.end());
    if (frames != pose.solverFrames.end()) {
        CHECK(frames->second.size() == 2);
        if (!frames->second.empty()) CHECK(frames->second.front().Origin()[1] == 2);
    }
    // The same fixture contains the forward property-pose-geometry-weight-
    // geometry acceptance chain. Both rigs use the common compiler/executor.
    RigExecRigEvaluator mixed(stage,SdfPath("/CrossAsset/Rig"));
    const bool mixedCompiled=Compile(mixed); CHECK(mixedCompiled); if(!mixedCompiled)return;
    const auto forward = mixed.Evaluate(UsdTimeCode(6)); if(!Ready(forward))return;
    CHECK(forward.movedProperties.count(SdfPath("/CrossAsset/Geom/Arm.points")) == 1);
    CHECK(forward.weightFields.count(SdfPath("/CrossAsset/Rig/Joints/Finger/FingerVolume")) == 1);
}

void CountRecoveryAndCutoff(const std::string &fixture)
{
    auto stage = UsdStage::Open(fixture); CHECK(stage); if (!stage) return;
    RigExecRigEvaluator evaluator(stage,rigPath);
    const bool compiled=Compile(evaluator); CHECK(compiled); if(!compiled)return;
    const auto first = evaluator.Evaluate(UsdTimeCode(1)); if(!Ready(first))return;
    CHECK(evaluator.GetBakedProgram()); if(!evaluator.GetBakedProgram())return;
    CHECK(Vector(first,rawValue) == GfVec3f(6,0,0));
    CHECK(Vector(first,downstream) == GfVec3f(6,0,0));
    const size_t builds = evaluator.GetBakedProgramBuildCount();
    const auto &B = evaluator.GetBakedProgram()->GetStepGraph();
    std::atomic<unsigned> runs{0};
    RigExecBakedProgramTesting::SetOpObservers(*evaluator.GetBakedProgram(),
        [&](uint32_t op) {
            const auto &step = B.steps[op];
            if (step.kind == RigExecBakedStepKind::PropertyRevision && step.part > 0 &&
                B.propertyChains[size_t(step.object)].target == downstream) ++runs;
        },{});
    const auto unchanged = evaluator.Evaluate(UsdTimeCode(2)); CHECK(unchanged.valid);
    CHECK(Vector(unchanged,rawValue) == GfVec3f(6,0,0));
    CHECK(Vector(unchanged,downstream) == GfVec3f(6,0,0));
    CHECK(runs.load() == 0); // Changes outside element 1 stop at exact equality.
    const auto changed = evaluator.Evaluate(UsdTimeCode(3)); CHECK(changed.valid);
    CHECK(Vector(changed,rawValue) == GfVec3f(8,0,0));
    CHECK(Vector(changed,downstream) == GfVec3f(8,0,0));
    CHECK(runs.load() > 0);
    RigExecBakedProgramTesting::SetOpObservers(*evaluator.GetBakedProgram(),{},{});
    const auto shortArray = evaluator.Evaluate(UsdTimeCode(4)); CHECK(shortArray.valid);
    CHECK(Vector(shortArray,rawValue) == GfVec3f(99)); CHECK(Unavailable(shortArray));
    const auto recovered = evaluator.Evaluate(UsdTimeCode(5)); CHECK(recovered.valid);
    CHECK(Vector(recovered,rawValue) == GfVec3f(9,0,0)); CHECK(!Unavailable(recovered));
    CHECK(evaluator.GetBakedProgramBuildCount() == builds);
}


// Actual authored property bindings: phase authority is independent of lexical
// target discovery, while only a connected Final own-read is a true cycle.
void PropertyPhaseAuthority()
{
    const SdfPath root("/Phase/Rig");
    auto stage=UsdStage::CreateInMemory();
    stage->DefinePrim(SdfPath("/Phase"),TfToken("Scope"));
    stage->DefinePrim(root,TfToken("RigExecRoot"));
    auto channels=stage->DefinePrim(SdfPath("/Phase/Channels"),TfToken("Scope"));
    const auto own=channels.CreateAttribute(TfToken("own"),SdfValueTypeNames->Double,true);
    CHECK(own.Set(3.0));
    const auto base=channels.CreateAttribute(TfToken("base"),SdfValueTypeNames->Double,true);
    const auto checkpoint=channels.CreateAttribute(TfToken("checkpoint"),SdfValueTypeNames->Double,true);
    CHECK(base.Set(0.0));CHECK(checkpoint.Set(0.0));
    const auto math=[&](const char *name,const SdfPath &target,float argument) {
        auto prim=stage->DefinePrim(root.AppendChild(TfToken(name)),TfToken("RigExecFloatMathMover"));
        CHECK(prim.ApplyAPI(TfToken("RigExecMoverAPI")));
        CHECK(prim.GetRelationship(TfToken("rigExec:moves")).SetTargets({target}));
        CHECK(prim.GetAttribute(TfToken("rigExec:operation")).Set(TfToken("add")));
        CHECK(prim.GetAttribute(TfToken("inputs:value")).Set(argument));
        return prim;
    };
    const auto bind=[&](const UsdPrim &prim,const UsdAttribute &source,const std::string &phase) {
        auto input=prim.GetAttribute(TfToken("inputs:value"));
        CHECK(input.SetConnections({source.GetPath()}));
        CHECK(input.SetMetadata(TfToken(RigExecReadPhaseMetadataName),phase));
    };
    const auto first=math("AFirst",own.GetPath(),99.0f);
    bind(first,own,"base");
    const auto second=math("BSecond",own.GetPath(),99.0f);
    bind(second,own,first.GetPath().GetString());
    const auto baseReader=math("ReadBase",base.GetPath(),99.0f);
    bind(baseReader,own,"base");
    const auto checkpointReader=math("ReadCheckpoint",checkpoint.GetPath(),99.0f);
    bind(checkpointReader,own,first.GetPath().GetString());
    const TfTokenVector initialOrder{
        TfToken("ReadCheckpoint"),TfToken("ReadBase"),TfToken("BSecond"),TfToken("AFirst")};
    stage->GetPrimAtPath(root).SetChildrenReorder(initialOrder);
    CHECK(stage->GetPrimAtPath(root).GetChildrenReorder()==initialOrder);
    // Float input reading a Double target crosses the early Double-tail path.
    // The converted Base/AtPrim record must dominate that target's Final edge.
    const auto number=[&](const RigExecRigPose &pose,const UsdAttribute &attr,double expected) {
        const auto found=pose.movedProperties.find(attr.GetPath());
        CHECK(found!=pose.movedProperties.end());
        if(found==pose.movedProperties.end())return;
        CHECK(found->second.IsHolding<double>());
        if(found->second.IsHolding<double>())CHECK(found->second.UncheckedGet<double>()==expected);
    };
    RigExecRigEvaluator evaluator(stage,root);
    CHECK(Compile(evaluator));
    const auto initial=evaluator.Evaluate(UsdTimeCode(1));if(!Ready(initial))return;
    number(initial,own,12.0);number(initial,base,3.0);number(initial,checkpoint,6.0);
    CHECK(evaluator.GetSkippedOperations().empty());
    auto cycle=math("CLaterCycle",own.GetPath(),99.0f);
    bind(cycle,own,"final");
    const TfTokenVector cycleOrder{
        TfToken("ReadCheckpoint"),TfToken("ReadBase"),TfToken("CLaterCycle"),TfToken("BSecond"),TfToken("AFirst")};
    stage->GetPrimAtPath(root).SetChildrenReorder(cycleOrder);
    CHECK(stage->GetPrimAtPath(root).GetChildrenReorder()==cycleOrder);
    std::string authored;CHECK(stage->GetRootLayer()->ExportToString(&authored));
    const auto excluded=evaluator.Evaluate(UsdTimeCode(1));if(!Ready(excluded))return;
    CHECK(evaluator.GetSkippedOperations().size()==1);
    CHECK(evaluator.GetSkippedOperations().count(cycle.GetPath())==1);
    CHECK(excluded.movedProperties.count(own.GetPath())==0);
    number(excluded,base,3.0);number(excluded,checkpoint,6.0);
    const auto program=evaluator.GetBakedProgram();CHECK(program);if(!program)return;
    const auto &B=program->GetStepGraph();
    const auto found=std::find_if(B.propertyChains.begin(),B.propertyChains.end(),
        [&](const auto &chain){return chain.target==own.GetPath();});
    CHECK(found!=B.propertyChains.end());
    if(found!=B.propertyChains.end()) {
        const size_t start=found->versionBase;
        const bool haveVersions=start<=B.propertyVersionValid.size() &&
            B.propertyVersionValid.size()-start>=4;
        CHECK(haveVersions);
        if(haveVersions) {
            CHECK(B.propertyVersionValid[start]);CHECK(B.propertyVersionValid[start+1]);
            CHECK(B.propertyVersionValid[start+2]);CHECK(!B.propertyVersionValid[start+3]);
        }
    }
    const auto held=evaluator.Evaluate(UsdTimeCode(1));if(!Ready(held))return;
    number(held,base,3.0);number(held,checkpoint,6.0);
    CHECK(held.movedProperties.count(own.GetPath())==0);
    CHECK(evaluator.GetLastOpTrace().empty());
    std::string after;CHECK(stage->GetRootLayer()->ExportToString(&after));CHECK(after==authored);

    // The unconnected consumer head can itself be a chain target. That read
    // is its incoming/raw value, rather than a synthetic self-Final edge.
    auto incomingStage=UsdStage::CreateInMemory();
    incomingStage->DefinePrim(SdfPath("/Incoming"),TfToken("Scope"));
    incomingStage->DefinePrim(SdfPath("/Incoming/Rig"),TfToken("RigExecRoot"));
    auto incoming=incomingStage->DefinePrim(SdfPath("/Incoming/Rig/Add"),TfToken("RigExecFloatMathMover"));
    CHECK(incoming.ApplyAPI(TfToken("RigExecMoverAPI")));
    CHECK(incoming.GetAttribute(TfToken("rigExec:operation")).Set(TfToken("add")));
    auto argument=incoming.GetAttribute(TfToken("inputs:value"));CHECK(argument.Set(4.0f));
    CHECK(incoming.GetRelationship(TfToken("rigExec:moves")).SetTargets({argument.GetPath()}));
    RigExecRigEvaluator incomingEvaluator(incomingStage,SdfPath("/Incoming/Rig"));
    CHECK(Compile(incomingEvaluator));
    const auto incomingPose=incomingEvaluator.Evaluate(UsdTimeCode(1));if(!Ready(incomingPose))return;
    CHECK(incomingEvaluator.GetSkippedOperations().empty());
    const auto answer=incomingPose.movedProperties.find(argument.GetPath());
    CHECK(answer!=incomingPose.movedProperties.end());
    if(answer!=incomingPose.movedProperties.end()) {
        CHECK(answer->second.IsHolding<float>());
        if(answer->second.IsHolding<float>())CHECK(answer->second.UncheckedGet<float>()==8.0f);
    }
}

void ForwardScalarAndOverride()
{
    auto stage=UsdStage::CreateInMemory();
    stage->DefinePrim(SdfPath("/Forward"),TfToken("Scope"));
    const SdfPath root("/Forward/Rig");stage->DefinePrim(root,TfToken("RigExecRoot"));
    auto channels=stage->DefinePrim(SdfPath("/Forward/Channels"),TfToken("Scope"));
    auto early=channels.CreateAttribute(TfToken("aEarly"),SdfValueTypeNames->Double,true);CHECK(early.Set(0.0));
    auto later=channels.CreateAttribute(TfToken("zLater"),SdfValueTypeNames->Float,true);CHECK(later.Set(4.0f));
    auto make=[&](const char *name,const UsdAttribute &target,float value) {
        auto prim=stage->DefinePrim(root.AppendChild(TfToken(name)),TfToken("RigExecFloatMathMover"));
        CHECK(prim.ApplyAPI(TfToken("RigExecMoverAPI")));
        CHECK(prim.GetRelationship(TfToken("rigExec:moves")).SetTargets({target.GetPath()}));
        CHECK(prim.GetAttribute(TfToken("rigExec:operation")).Set(TfToken("add")));
        CHECK(prim.GetAttribute(TfToken("inputs:value")).Set(value));return prim;
    };
    make("Producer",later,2.0f);
    auto consumer=make("Consumer",early,99.0f);
    CHECK(consumer.GetAttribute(TfToken("inputs:value")).SetConnections({later.GetPath()}));
    CHECK(consumer.GetAttribute(TfToken("inputs:value")).SetMetadata(
        TfToken(RigExecReadPhaseMetadataName),std::string("final")));
    RigExecRigEvaluator evaluator(stage,root);CHECK(Compile(evaluator));
    const auto read=[&](double expected) {
        const auto pose=evaluator.Evaluate(UsdTimeCode(1));if(!Ready(pose))return;
        const auto found=pose.movedProperties.find(early.GetPath());CHECK(found!=pose.movedProperties.end());
        if(found!=pose.movedProperties.end()) {
            CHECK(found->second.IsHolding<double>());
            if(found->second.IsHolding<double>())CHECK(found->second.UncheckedGet<double>()==expected);
        }
        CHECK(evaluator.GetSkippedOperations().empty());
    };
    read(6.0);
    // A consumer-hop override stands its phase record aside.
    evaluator.SetInteractiveOverrides({RigExecValueOverride{consumer.GetPath(),TfToken(),TfToken("inputs:value"),VtValue(10.0f)}});
    read(10.0);
    evaluator.ClearInteractiveOverrides();read(6.0);
    // A target override instead becomes the producer base before its add.
    const std::vector<RigExecValueOverride> targetDrag{
        RigExecValueOverride{later.GetPrimPath(),TfToken(),later.GetName(),VtValue(10.0f)}};
    evaluator.SetInteractiveOverrides(targetDrag);
    read(12.0);
    const auto FreshPose=[&](const std::vector<RigExecValueOverride> &overrides) {
        RigExecRigEvaluator fresh(stage,root);
        const bool compiled=Compile(fresh);CHECK(compiled);
        if(!compiled)return RigExecRigPose();
        fresh.SetInteractiveOverrides(overrides);
        return fresh.Evaluate(UsdTimeCode(1));
    };
    const auto targetReference=FreshPose(targetDrag);
    const auto targetHeld=evaluator.Evaluate(UsdTimeCode(1));
    CHECK(Ready(targetReference));CHECK(Ready(targetHeld));
    CHECK(targetReference.movedProperties==targetHeld.movedProperties);
    CHECK(targetReference.diagnostics==targetHeld.diagnostics);
    evaluator.ClearInteractiveOverrides();read(6.0);
    const auto releasedReference=FreshPose({});
    const auto releasedHeld=evaluator.Evaluate(UsdTimeCode(1));
    CHECK(Ready(releasedReference));CHECK(Ready(releasedHeld));
    CHECK(releasedReference.movedProperties==releasedHeld.movedProperties);
    CHECK(releasedReference.diagnostics==releasedHeld.diagnostics);
}

void WholePointsAvailability() {
    RigExecBakedProgramImpl B;B.chains.resize(1);
    auto &chain=B.chains[0];chain.haveResult=true;chain.result={GfVec3f(4,5,6)};
    RigExecCrossDomainRead read;read.kind=RigExecCrossDomainRead::Kind::Points;read.finalPoints=true;read.points={{0,1}};
    B.crossDomainReads.push_back(read);B.headLeaves.resize(2);
    B.headLeaves[0].value=VtValue(VtVec3fArray{GfVec3f(99)});
    B.headLeaves[1].value=VtValue(VtVec3fArray{GfVec3f(42)});
    RigExecBakedReaderWalk reader;reader.walk.flavour=RigExecBakedWalk::Flavour::Connected;
    reader.walk.type=RigExecBakedHeadValueType::Vec3fArray;
    RigExecBakedWalkHop consumer,source;consumer.leaf=1;source.leaf=0;source.crossDomain=0;
    reader.walk.hops={consumer,source};reader.rawLeaf=1;B.readerWalks.push_back(reader);
    RigExecRevisionLeafKey key;key.type=RigExecRevisionLeafType::Vec3fArray;key.flavour=RigExecRevisionLeafFlavour::ResolvedOnly;
    auto value=RigExecBakedSampleWalkedPathLeaf(B,key,0);
    CHECK(value.IsHolding<VtVec3fArray>() && value.UncheckedGet<VtVec3fArray>()[0]==GfVec3f(4,5,6));
    chain.result.clear();value=RigExecBakedSampleWalkedPathLeaf(B,key,0);
    CHECK(value.IsHolding<VtVec3fArray>() && value.UncheckedGet<VtVec3fArray>().empty());
    chain.haveResult=false;value=RigExecBakedSampleWalkedPathLeaf(B,key,0);
    CHECK(value.IsHolding<VtVec3fArray>() && value.UncheckedGet<VtVec3fArray>()[0]==GfVec3f(42));
    B.headLeaves[1].value=VtValue();value=RigExecBakedSampleWalkedPathLeaf(B,key,0);CHECK(value.IsEmpty());
    chain.haveResult=true;chain.result={GfVec3f(7,8,9)};value=RigExecBakedSampleWalkedPathLeaf(B,key,0);
    CHECK(value.IsHolding<VtVec3fArray>() && value.UncheckedGet<VtVec3fArray>()[0]==GfVec3f(7,8,9));
}

void ExplicitSelection(const std::string &fixture)
{
    auto stage = UsdStage::Open(fixture); CHECK(stage); if (!stage) return;
    const auto input = stage->GetAttributeAtPath(SdfPath("/S9Asset/Rig/Properties/RawSelected.inputs:value"));
    CHECK(input.ClearMetadata(TfToken(RigExecInputElementMetadataName)));
    RigExecRigEvaluator absent(stage,rigPath);
    const bool compiled=Compile(absent); CHECK(compiled); if(!compiled)return;
    const auto pose = absent.Evaluate(UsdTimeCode(1)); if(!Ready(pose))return;
    CHECK(Vector(pose,rawValue) == GfVec3f(99)); // No implicit array conversion.

    CHECK(input.SetMetadata(TfToken(RigExecInputElementMetadataName),VtValue(-1)));
    RigExecRigEvaluator negative(stage,rigPath);
    std::vector<std::string> errors; CHECK(!negative.Compile(&errors));
    bool rejected = false;
    for (const auto &error : errors)
        rejected |= error.find("rigExecInputElement requires a nonnegative int") != std::string::npos;
    CHECK(rejected);

    CHECK(input.ClearMetadata(TfToken(RigExecInputElementMetadataName)));
    const auto matrixInput = stage->GetAttributeAtPath(SdfPath("/S9Asset/Rig/Properties/Pose.inputs:value"));
    CHECK(matrixInput.SetMetadata(TfToken(RigExecInputElementMetadataName),VtValue(1)));
    RigExecRigEvaluator wrongType(stage,rigPath);
    errors.clear(); CHECK(!wrongType.Compile(&errors));
    rejected = false;
    for (const auto &error : errors)
        rejected |= error.find("rigExecInputElement requires a nonnegative int on a Vec3f input") != std::string::npos;
    CHECK(rejected);
}
}
int main(int argc,char **argv)
{
    if (argc < 2) return 2;
    const std::string fixtures = argv[1];
#ifdef RIGEXEC_SCHEMA_RESOURCE_DIR
    const std::string resources = TfAbsPath(RIGEXEC_SCHEMA_RESOURCE_DIR);
#else
    const std::string resources = TfAbsPath(fixtures+"/../../plugin/rigExecSchema/resources");
#endif
    if (PlugRegistry::GetInstance().RegisterPlugins(resources).empty()) return 2;
    const std::string fixture = fixtures+"/oneloop_s9_reads.usda";
    ReadVersions(fixture); CountRecoveryAndCutoff(fixture); ExplicitSelection(fixture); WholePointsAvailability();
    PropertyPhaseAuthority();ForwardScalarAndOverride();
    std::printf("testRigExecCrossDomainInputs: %d failure(s)\n",failures);
    return failures ? 1 : 0;
}
