#include "sceneProgram.h"
#include <algorithm>
#include <type_traits>
namespace rigExec {
namespace { bool Fail(std::string *error,const std::string &message){if(error)*error=message;return false;} }
bool RigExecSceneProgram::Append(const std::string &key,RigExecSceneOperation operation,std::string *error) {
    if(key.empty())return Fail(error,"empty scene operation key");
    RigExecOpDescriptor descriptor;descriptor.key=key;descriptor.kind=101+uint32_t(operation.index());
    std::visit([&](const auto &op){using T=std::decay_t<decltype(op)>;
        if constexpr(std::is_same_v<T,RigExecSceneUnavailableFrameOp>){descriptor.writes={op.output};}
        else if constexpr(std::is_same_v<T,RigExecSceneCopyOp>){descriptor.reads={op.input};descriptor.writes={op.output};}
        else if constexpr(std::is_same_v<T,RigExecPoseCommitBinding>){descriptor.reads=op.reads;descriptor.writes=op.writes;}
        else if constexpr(std::is_same_v<T,RigExecSceneExtractOp>){descriptor.reads={op.aggregate,op.fallback};descriptor.writes={op.output};}
        else if constexpr(std::is_same_v<T,RigExecSceneFieldOp>){descriptor.reads=op.binding.fieldReads;if(op.points!=UINT64_MAX)descriptor.reads.push_back(op.points);for(const auto &hops:{op.defaultWeight.hops,op.defaultWeight.doubleHops})for(const auto &hop:hops)for(const auto id:{hop.raw,hop.overlay})if(id!=UINT64_MAX)descriptor.reads.push_back(id);descriptor.writes={op.output};}
        else if constexpr(std::is_same_v<T,RigExecSceneWeightOp>){descriptor.reads=op.binding.objects.at(size_t(op.object)).packetReads;descriptor.writes={op.output};}
        else if constexpr(std::is_same_v<T,RigExecSceneBlendChannelOp>){descriptor.reads=op.binding.reads;descriptor.writes={op.output};}
        else if constexpr(std::is_same_v<T,RigExecSceneGeometryOp>){descriptor.reads=op.binding.reads;descriptor.reads.push_back(op.incoming);descriptor.reads.push_back(op.pointsBase);descriptor.reads.push_back(op.pointsFinal);if(op.field!=UINT64_MAX)descriptor.reads.push_back(op.field);descriptor.writes={op.output};}
        else if constexpr(std::is_same_v<T,RigExecSceneSwitchOp>){descriptor.reads=op.binding.reads;descriptor.writes={op.output};}
        else if constexpr(std::is_same_v<T,RigExecConstraintGraphBinding>){descriptor.reads=op.reads;descriptor.writes={op.target.output};descriptor.writes.insert(descriptor.writes.end(),op.target.chainOutputs.begin(),op.target.chainOutputs.end());}
        else if constexpr(std::is_same_v<T,RigExecBoundPoseInterpolator>){descriptor.reads=op.reads;descriptor.writes=op.weights;descriptor.writes.insert(descriptor.writes.end(),op.disabledWeights.begin(),op.disabledWeights.end());}
        else {descriptor.reads=op.reads;descriptor.writes={op.output};}
    },operation);
    for(auto id:descriptor.writes)if(id==UINT64_MAX || id>=layout.valueKeys.size())return Fail(error,"invalid scene operation output: "+key);
    descriptor.reads.erase(std::remove(descriptor.reads.begin(),descriptor.reads.end(),UINT64_MAX),descriptor.reads.end());
    layout.descriptors.push_back(std::move(descriptor));operations.push_back(std::move(operation));return true;
}
bool RigExecSceneProgram::Compile(RigExecCyclePolicy policy,std::string *error) {
    return Compile(policy,&retainedExclusions,error);
}
bool RigExecSceneProgram::Compile(RigExecCyclePolicy policy,const RigExecOpExclusionProof *proof,std::string *error) {
    const auto *authority=proof?proof:&retainedExclusions;
    if(!RigExecCompileOpGraph(layout.descriptors,layout.leaves,policy,authority,&graph,error))return false;
    retainedExclusions=*authority;
    identity=std::make_shared<const char>(0);return true;
}
bool RigExecSceneProgramRuntime::Prepare(const RigExecSceneProgram &program,std::string *error) {
    if(program.layout.descriptors.size()!=program.layout.initialProviderOps+program.layout.baseSelections.size()+program.operations.size())
        return Fail(error,"scene operation layout mismatch");
    if(!program.identity)return Fail(error,"scene program has not been compiled");
    _preparedProgram=&program;_identity=program.identity;_excluded.clear();
    values=RigExecTypedValueStore(program.layout.valueKeys.size());_operations=program.operations;
    _workspaces.resize(_operations.size());_ran=false;
    for(size_t i=0;i<_operations.size();++i)if(const auto *op=std::get_if<RigExecSceneWeightOp>(&_operations[i]))
        RigExecPrepareWeightGraphWorkspace(op->binding,&_workspaces[i].weight);
    for(size_t i=0;i<_operations.size();++i)if(auto *op=std::get_if<RigExecSceneFieldOp>(&_operations[i])) {
        RigExecBindWeightGraphCycles(program.layout.descriptors,program.graph,&op->binding);
        RigExecPrepareWeightGraphWorkspace(op->binding,&_workspaces[i].weight);
    }
    for(size_t i=0;i<program.graph.canonicalIndex.size();++i)if(program.graph.canonicalIndex[i]<0)
        for(auto id:program.layout.descriptors[i].writes){_excluded.push_back(id);values.PublishSource(id,VtValue(),true,false);}
    return true;
}
bool RigExecSceneProgramRuntime::Run(const RigExecSceneProgram &program,uint32_t original,std::string *error) {
    const auto providerCount=program.layout.initialProviderOps,baseCount=program.layout.baseSelections.size();
    if(original<providerCount)return RigExecRunProviderOp(program.layout.providers,original,&values,error);
    if(original<providerCount+baseCount)return program.layout.RunBaseSelection(original-providerCount,&values);
    const auto index=size_t(original)-providerCount-baseCount;
    if(index>=_operations.size())return Fail(error,"scene operation index out of range");
    auto &workspace=_workspaces[index];
    return std::visit([&](auto &op)->bool {using T=std::decay_t<decltype(op)>;
        if constexpr(std::is_same_v<T,RigExecSceneBlendChannelOp>) {
            if(!RigExecReadGeometryGraphLeaves(op.binding,values,&workspace.geometryInputs.leaves,error) ||
               !RigExecGatherSceneGeometryBlendChannels(op.binding.descriptor,workspace.geometryInputs.leaves,0,&workspace.blendChannels,error,true))return false;
            if(workspace.blendChannels.empty())values.PublishSource(op.output,VtValue(RigExecBlendChannel()),false,false);
            else values.PublishSource(op.output,VtValue(workspace.blendChannels.front()),false,false);
            return true;
        }
        else if constexpr(std::is_same_v<T,RigExecSceneUnavailableFrameOp>){
            RigExecPointFrame invalid;
            invalid.flags=0; values.Publish(op.output,invalid); return true;
        }
        else if constexpr(std::is_same_v<T,RigExecSceneCopyOp>){values.Copy(op.output,op.input,op.authoritative);return true;}
        else if constexpr(std::is_same_v<T,RigExecSceneEffectiveReadOp>) {
            VtValue selected;std::string diagnostic;
            const bool available=RigExecReadGraphTypedRead(op.read,values,&selected,&diagnostic);
            values.PublishSource(op.output,selected,!available,true,diagnostic);return true;
        }
        else if constexpr(std::is_same_v<T,RigExecBoundAutoClavicle>)return RigExecRunAutoClavicle(op,&values,error);
        else if constexpr(std::is_same_v<T,RigExecSceneProviderOp>)return RigExecRunProviderOp(program.layout.providers,op.originalIndex,&values,error);
        else if constexpr(std::is_same_v<T,RigExecPoseCommitBinding>)return RigExecRunPoseCommit(op,&values,&workspace.poseCommit,error);
        else if constexpr(std::is_same_v<T,RigExecPropertyGraphBinding>)return RigExecRunBoundProperty(op,&values,error);
        else if constexpr(std::is_same_v<T,RigExecSolverGraphBinding>){RigExecRunSolverGraph(op,&values,&workspace.solver,error);return true;}
        else if constexpr(std::is_same_v<T,RigExecConstraintGraphBinding>){RigExecRunConstraintGraph(op,&values,error);return true;}
        else if constexpr(std::is_same_v<T,RigExecSceneExtractOp>) {
            VtValue aggregate;
            if(RigExecReadSceneGraphValue(values,op.aggregate,&aggregate) && aggregate.IsHolding<RigExecPointFrameArray>() && op.element<aggregate.UncheckedGet<RigExecPointFrameArray>().frames.size())
                values.Publish(op.output,aggregate.UncheckedGet<RigExecPointFrameArray>().frames[op.element]);
            else values.Copy(op.output,op.fallback,false);
            return true;
        } else if constexpr(std::is_same_v<T,RigExecSceneFieldOp>) {
            size_t count=op.count;VtValue points;
            if(!op.staticError.empty()){RigExecPublishWeightField(&values,op.output,{},count,false,op.staticError);return true;}
            if(op.points!=UINT64_MAX) {
                if(!RigExecReadSceneGraphValue(values,op.points,&points) || !points.IsHolding<VtVec3fArray>()){RigExecPublishWeightField(&values,op.output,{},0,false,"entering points unavailable");return true;}
                count=points.UncheckedGet<VtVec3fArray>().size();
            }
            bool valid;std::string diagnostic;
            if(op.binding.root>=0)valid=RigExecRunBoundWeightField(op.binding,values,count,op.points,&workspace.weight,&diagnostic);
            else {
                VtValue weight;float fallback=1;
                if(RigExecReadGraphTypedRead(op.defaultWeight,values,&weight,nullptr) && weight.IsHolding<float>())fallback=weight.UncheckedGet<float>();
                valid=RigExecRunEffectiveWeightField({},-1,{},count,nullptr,fallback,&workspace.weight.result,&diagnostic);
            }
            RigExecPublishWeightField(&values,op.output,workspace.weight.result,count,valid,diagnostic);return true;
        } else if constexpr(std::is_same_v<T,RigExecSceneWeightOp>) {
            RigExecWeightPacket packet;
            RigExecRunBoundWeightPacket(op.binding,op.object,values,&workspace.weight,&packet,error);
            RigExecPublishWeightPacket(&values,op.output,packet);return true;
        } else if constexpr(std::is_same_v<T,RigExecSceneSwitchOp>){RigExecRunBoundSpaceSwitch(&op.binding,&values,op.output,error);return true;}
        else if constexpr(std::is_same_v<T,RigExecBoundPoseInterpolator>){RigExecRunBoundPoseInterpolator(&op,&values,error);return true;}
        else {
            const auto &descriptor=op.binding.descriptor;
            auto readPoints=[&](RigExecValueId id,std::vector<GfVec3f> *out) {
                VtValue value;
                if(!RigExecReadSceneGraphValue(values,id,&value) || !value.IsHolding<VtVec3fArray>()) {out->clear();return false;}
                const auto &array=value.UncheckedGet<VtVec3fArray>();out->assign(array.begin(),array.end());return true;
            };
            auto failed=[&](const std::string &reason) {
                if(descriptor.matrixOutput)values.Publish(op.output,VtValue(),true,true,0,reason);
                else if(op.incoming<values.values.size() && values.values[size_t(op.incoming)].initialized) {
                    const auto &source=values.values[size_t(op.incoming)];
                    std::visit([&](const auto &value) {values.Publish(op.output,value,source.blocked,true,source.count,reason);},source.value);
                    values.values[size_t(op.output)].raw=source.raw;
                } else values.Publish(op.output,VtValue(),true,true,0,reason);
                return true;
            };
            if(op.pointsBase!=UINT64_MAX && !readPoints(op.pointsBase,&workspace.basePoints))return failed("geometry base points unavailable");
            if(op.pointsFinal!=UINT64_MAX && !readPoints(op.pointsFinal,&workspace.finalPoints))return failed("geometry final points unavailable");
            if(descriptor.matrixOutput) {
                RigExecSurfaceProjectorFrames frames;RigExecProjectorReads reads;GfMatrix4d matrix;
                workspace.geometryDiagnostics.clear();
                if(!RigExecReadGeometryGraphProjector(op.binding,values,&workspace.geometryInputs,&frames,&reads,error) ||
                   !RigExecRunGeometryMatrix(descriptor.record,frames,reads,workspace.basePoints,workspace.finalPoints,
                       &workspace.geometry,&matrix,&workspace.geometryDiagnostics))
                    return failed(workspace.geometryDiagnostics.empty()?"projector inputs unavailable":workspace.geometryDiagnostics.front());
                values.PublishSource(op.output,VtValue(matrix),false,true);return true;
            }
            if(!readPoints(op.incoming,&workspace.points))return failed("geometry entering points unavailable");
            RigExecGeometryInputs inputs;
            if(descriptor.derived)inputs.basePoints=workspace.finalPoints;
            else if(op.pointsBase!=UINT64_MAX)inputs.basePoints=workspace.basePoints;
            else inputs.basePoints=workspace.points;
            if(!RigExecReadGeometryGraphInputs(op.binding,values,op.posedPoints,&workspace.geometryInputs,&inputs,error))
                return failed("geometry provider inputs unavailable");
            if(!descriptor.blendChannels.empty()) {
                if(!RigExecGatherSceneGeometryBlendChannels(descriptor,workspace.geometryInputs.leaves,inputs.basePoints.size(),&workspace.blendChannels,error) ||
                   !RigExecGeometryBlendDeltas(workspace.blendChannels,inputs.basePoints,&inputs.blendDeltas))
                    return failed("blend shape input/cardinality validation failed");
            }
            if(op.field!=UINT64_MAX) {
                VtValue field;
                if(op.field>=values.values.size() || values.values[size_t(op.field)].blocked ||
                   !RigExecReadSceneGraphValue(values,op.field,&field) || !field.IsHolding<VtFloatArray>() ||
                   field.UncheckedGet<VtFloatArray>().size()!=workspace.points.size())return failed("geometry common envelope unavailable");
                const auto &weights=field.UncheckedGet<VtFloatArray>();
                op.fieldPacket.values.assign(weights.begin(),weights.end());op.fieldPacket.valid=true;inputs.weights=&op.fieldPacket;
            }
            const auto params=RigExecAssembleGeometry(descriptor.record,inputs);
            if(!RigExecRunGeometry(descriptor.record,params,&workspace.geometry,&workspace.points,op.useSimd))
                return failed("geometry input/cardinality validation failed");
            values.PublishSource(op.output,VtValue(VtVec3fArray(workspace.points.begin(),workspace.points.end())),false,true);return true;
        }
    },_operations[index]);
}
bool RigExecSceneProgramRuntime::Evaluate(const RigExecSceneProgram &program,size_t identity,
    const std::map<SdfPath,VtValue> &overlays,std::string *error) {
    if(_preparedProgram!=&program || !program.identity || _identity!=program.identity)return Fail(error,"scene runtime prepared artifact mismatch");
    values.ResetChanges();for(auto id:_excluded)values.PublishSource(id,VtValue(),true,false);
    if(!program.layout.Sample(identity,overlays,&values,&_changed,error)){_ran=false;return false;}
    RigExecOpCallbacks callbacks;callbacks.run=[&](uint32_t canonical){return canonical<program.graph.ops.size() && Run(program,program.graph.ops[canonical].originalIndex,error);};
    callbacks.skip=[](uint32_t){};
    callbacks.changed=[&](RigExecValueId id){return id<values.values.size() && values.values[size_t(id)].changed;};
    if(!RigExecExecuteOpGraph(program.graph,_changed,{},!_ran,callbacks,&execution,error,&_ready)){_ran=false;return false;}
    _ran=true;return true;
}
}
