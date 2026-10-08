#include "rigExecGraph/solverGraphBinding.h"
#include "rigExec/frameExtraction.h"
#include "rigExecGraph/constraintGraphBinding.h"
#include <cstdio>
#include <cstring>
#include "pxr/usd/sdf/types.h"
using namespace rigExec;
PXR_NAMESPACE_USING_DIRECTIVE
#define CHECK(x) do {if(!(x)){std::printf("FAIL %s:%d: %s\n",__FILE__,__LINE__,#x);return 1;}}while(0)
namespace {
RigExecPointFrame Translated(double x) {
    RigExecPointFrame frame;
    frame.points=RigExecIdentityLandmarks();
    for(auto &point:frame.points)point[0]+=x;
    return frame;
}
RigExecSceneBoundInput Raw(const char *path) {
    RigExecSceneBoundInput input;input.consumer=input.source=SdfPath(path);
    input.route=RigExecSceneReadRoute::Raw;input.readPhase=TfToken("base");return input;
}
}
int main() {
    {
        // Exact identity selection copies facts and whole input outcomes;
        // duplicate Default rows are not an AtTime fallback.
        RigExecSceneDescriptors source;
        source.rigRoot=SdfPath("/Rig");
        source.identities={UsdTimeCode::Default(),UsdTimeCode(3)};
        const SdfPath input("/Rig/C.value"), relation("/Rig/S.rigExec:sources");
        auto &node=source.nodes[SdfPath("/Rig/C")];
        node.fact.path=SdfPath("/Rig/C");node.fact.type=TfToken("RigExecControl");
        node.fact.active=false;node.stackOrdinal=17;
        auto &attr=source.attributes[input];
        attr.fact.path=input;attr.fact.type=SdfValueTypeNames->Double;
        attr.fact.hasValue=true;attr.fact.hasAuthoredValue=true;
        attr.fact.defaultBlocked=true;attr.fact.hasAuthoredDefault=true;
        attr.fact.sampleTimes={3,7};attr.fact.connections={SdfPath("/Other.value")};
        attr.fact.metadata["rigExecReadPhase"]=VtValue(TfToken("final"));
        attr.readPhase=TfToken("final");attr.inputs.resize(2);
        attr.inputs[0].rawBlocked=true;attr.inputs[0].resolvedBlocked=true;
        attr.inputs[0].state=RigExecSceneInputState::Empty;
        uint64_t bits=UINT64_C(0x7ff8000000000123);double nan;
        std::memcpy(&nan,&bits,sizeof(nan));
        attr.inputs[1].raw=VtValue(nan);attr.inputs[1].resolved=VtValue(-0.0);
        attr.inputs[1].state=RigExecSceneInputState::Computed;
        attr.inputs[1].source=SdfPath("/Other.value");
        attr.inputs[1].hops={input,SdfPath("/Other.value")};
        auto &rel=source.relationships[relation];
        rel.fact.path=relation;rel.fact.targets={SdfPath("/Absent")};
        rel.readPhase=TfToken("base");rel.forwardedTargets=rel.fact.targets;
        rel.targetExists={0};
        source.compilerTargets={SdfPath("/Outside")};
        source.timeCodesPerSecond=30;source.framesPerSecond=25;
        source.interpolation=TfToken("held");source.upAxis=TfToken("Z");
        RigExecSceneDescriptors selected;std::string error;
        CHECK(RigExecSelectSceneDescriptorIdentities(source,
              {UsdTimeCode(3),UsdTimeCode::Default(),UsdTimeCode::Default()},&selected,&error));
        CHECK(selected.identities.size()==3);
        const auto &captured=selected.attributes.at(input);
        const double &got=captured.inputs[0].raw.UncheckedGet<double>();
        CHECK(std::memcmp(&got,&nan,sizeof(double))==0);
        const double zero=captured.inputs[0].resolved.UncheckedGet<double>();
        const double minusZero=-0.0;
        CHECK(std::memcmp(&zero,&minusZero,sizeof(double))==0);
        CHECK(captured.inputs[0].state==RigExecSceneInputState::Computed);
        CHECK(captured.inputs[0].source==attr.inputs[1].source);
        CHECK(captured.inputs[0].hops==attr.inputs[1].hops);
        CHECK(captured.inputs[1].raw.IsEmpty() && captured.inputs[1].resolved.IsEmpty());
        CHECK(captured.inputs[1].rawBlocked && captured.inputs[1].resolvedBlocked);
        CHECK(captured.inputs[2].rawBlocked && captured.inputs[2].resolvedBlocked);
        CHECK(captured.fact.type==attr.fact.type && captured.readPhase==attr.readPhase);
        CHECK(captured.fact.metadata==attr.fact.metadata);
        CHECK(captured.fact.sampleTimes==attr.fact.sampleTimes);
        CHECK(captured.fact.connections==attr.fact.connections);
        CHECK(captured.fact.hasValue && captured.fact.hasAuthoredValue);
        CHECK(captured.fact.defaultBlocked && captured.fact.hasAuthoredDefault);
        CHECK(!selected.nodes.at(node.fact.path).fact.active);
        CHECK(selected.nodes.at(node.fact.path).stackOrdinal==17);
        CHECK(selected.relationships.at(relation).forwardedTargets==rel.forwardedTargets);
        CHECK(selected.relationships.at(relation).targetExists==rel.targetExists);
        CHECK(selected.compilerTargets==source.compilerTargets);
        CHECK(selected.timeCodesPerSecond==30 && selected.framesPerSecond==25);
        CHECK(selected.interpolation==TfToken("held") && selected.upAxis==TfToken("Z"));
        const auto savedIdentities=selected.identities;
        CHECK(!RigExecSelectSceneDescriptorIdentities(source,{},&selected,&error));
        CHECK(selected.identities==savedIdentities);
        RigExecSceneDescriptors aliased=source;
        CHECK(RigExecSelectSceneDescriptorIdentities(aliased,
              {UsdTimeCode(3),UsdTimeCode::Default()},&aliased,&error));
        CHECK(aliased.identities==std::vector<UsdTimeCode>({UsdTimeCode(3),UsdTimeCode::Default()}));
        CHECK(aliased.attributes.at(input).inputs[0].state==RigExecSceneInputState::Computed);
        CHECK(aliased.attributes.at(input).inputs[1].rawBlocked);
        CHECK(source.identities==std::vector<UsdTimeCode>({UsdTimeCode::Default(),UsdTimeCode(3)}));
        CHECK(!RigExecSelectSceneDescriptorIdentities(source,{UsdTimeCode(5)},&selected,&error));
        CHECK(selected.identities==savedIdentities);
        attr.inputs.pop_back();
        CHECK(!RigExecSelectSceneDescriptorIdentities(source,{UsdTimeCode::Default()},&selected,&error));
        CHECK(selected.identities==savedIdentities);
    }
    RigExecTypedValueStore values(8);
    RigExecPointFrameArray a,b;a.frames={Translated(1)};b.frames={Translated(5)};
    a.rests={RigExecIdentityLandmarks()};b.rests=a.rests;
    values.Publish(0,VtValue(a));values.Publish(1,VtValue(b));values.Publish(2,0.5f);
    RigExecSceneSolverDescriptor descriptor;
    descriptor.path=SdfPath("/Blend");descriptor.record.kind=RigExecSolverKind::BlendPointFrames;
    descriptor.blendA=SdfPath("/A");descriptor.blendB=SdfPath("/B");
    descriptor.inputs["inputs:weight"]=Raw("/Blend.inputs:weight");
    descriptor.frameBindings["rigExec:inputA"].phase=TfToken("final");
    descriptor.frameBindings["rigExec:inputB"].phase=TfToken("final");
    size_t aggregateReads=0;
    RigExecSceneGraphBindingContext context;
    context.resolve=[&](const RigExecSceneGraphReadRequest &request,RigExecValueId *id,std::string *) {
        if(request.domain==RigExecSceneValueDomain::SolverAggregate) {
            if(request.phase!="final")return false;
            ++aggregateReads;*id=request.source==SdfPath("/A")?0:1;return true;
        }
        if(!request.raw || request.source!=SdfPath("/Blend.inputs:weight"))return false;
        *id=2;return true;
    };
    RigExecSolverGraphBinding solver;std::string error;
    CHECK(RigExecBindSolverGraph(descriptor,context,3,&solver,&error));
    CHECK(aggregateReads==2 && solver.reads==std::vector<RigExecValueId>({0,1,2}));
    RigExecSolverWorkspace workspace;
    CHECK(RigExecRunSolverGraph(solver,&values,&workspace,&error));
    auto result=values.Read<VtValue>(3);
    CHECK(result && result->IsHolding<RigExecPointFrameArray>());
    CHECK(result->UncheckedGet<RigExecPointFrameArray>().frames[0].Origin()==GfVec3d(3,0,0));
    values.Publish(2,1.0f);
    CHECK(RigExecRunSolverGraph(solver,&values,&workspace,&error));
    CHECK(values.Read<VtValue>(3)->UncheckedGet<RigExecPointFrameArray>().frames[0].Origin()==GfVec3d(5,0,0));
    values.values[0].initialized=false;
    CHECK(!RigExecRunSolverGraph(solver,&values,&workspace,&error));
    CHECK(values.values[3].blocked && !values.Read<VtValue>(3));
    CHECK(values.values[3].count==0 && std::get<VtValue>(values.values[3].value).UncheckedGet<RigExecPointFrameArray>().frames.empty());

    values.Publish(0,Translated(2));values.Publish(1,Translated(8));values.PublishSource(2,VtValue(false));values.Publish(3,1.0f);
    RigExecSceneConstraintDescriptor constraint;
    constraint.path=SdfPath("/Position");constraint.record.kind=RigExecConstraintKind::Position;
    constraint.sources={SdfPath("/Source")};
    constraint.inputs["inputs:enabled"]=Raw("/Position.inputs:enabled");
    constraint.inputs["inputs:defaultWeight"]=Raw("/Position.inputs:defaultWeight");
    context.resolve=[](const RigExecSceneGraphReadRequest &request,RigExecValueId *id,std::string *) {
        if(request.domain==RigExecSceneValueDomain::Pose) {
            if(request.consumer!=SdfPath("/Position.rigExec:sources"))return false;
            *id=1;return true;
        }
        *id=request.source==SdfPath("/Position.inputs:enabled")?2:3;return request.raw;
    };
    RigExecSceneDescriptors scene;
    RigExecConstraintGraphTarget target;target.incoming=0;target.output=4;
    RigExecConstraintGraphBinding binding;
    CHECK(RigExecBindConstraintGraph(scene,constraint,context,target,&binding,&error));
    CHECK(binding.reads==std::vector<RigExecValueId>({0,1,2,3}));
    values.values[1].initialized=false;
    CHECK(RigExecRunConstraintGraph(binding,&values,&error));
    CHECK(values.Read<RigExecPointFrame>(4)->Origin()==GfVec3d(2,0,0));
    values.PublishSource(2,VtValue(true));
    CHECK(!RigExecRunConstraintGraph(binding,&values,&error));
    CHECK(values.Read<RigExecPointFrame>(4)->Origin()==GfVec3d(2,0,0));
    CHECK(!values.values[4].blocked && !values.values[4].error.empty());
    values.Publish(1,Translated(8));
    CHECK(RigExecRunConstraintGraph(binding,&values,&error));
    CHECK(values.Read<RigExecPointFrame>(4)->Origin()==GfVec3d(8,0,0));
    CHECK(values.values[4].error.empty());
    return 0;
}
