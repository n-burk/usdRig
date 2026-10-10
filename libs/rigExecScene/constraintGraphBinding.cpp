#include "constraintGraphBinding.h"
#include "poseProgram.h"
#include <algorithm>
#include <cmath>
#include "pxr/base/vt/types.h"
namespace rigExec {
namespace {
bool Fail(std::string *error,const std::string &message) {if(error)*error=message;return false;}
void Add(std::vector<RigExecValueId> *reads,RigExecValueId id) {if(id!=UINT64_MAX)reads->push_back(id);}
bool Frame(const RigExecTypedValueStore &store,RigExecValueId id,RigExecPointFrame *out) {
    VtValue value;
    if(!RigExecReadSceneGraphValue(store,id,&value) || !value.IsHolding<RigExecPointFrame>())return false;
    *out=value.UncheckedGet<RigExecPointFrame>();return RigExecConstraintFrameUsable(*out);
}
}
bool RigExecBindConstraintGraph(const RigExecSceneDescriptors &scene,
    const RigExecSceneConstraintDescriptor &source,const RigExecSceneGraphBindingContext &context,
    const RigExecConstraintGraphTarget &target,RigExecConstraintGraphBinding *destination,std::string *error) {
    if(!destination || !context.resolve || target.incoming==UINT64_MAX || target.output==UINT64_MAX)
        return Fail(error,"invalid constraint graph target");
    RigExecConstraintGraphBinding bound;bound.structural=source;
    bound.structural.inputs.clear();bound.structural.typedReads.clear();bound.target=target;
    if(source.useAnimatedTs) {bound.target.chainRests.clear();bound.target.restLive.clear();}
    if(!RigExecBindGraphInputs(source.inputs,source.typedReads,source.path,context,&bound.inputs,&bound.reads,error))return false;
    Add(&bound.reads,target.incoming);Add(&bound.reads,target.carry);Add(&bound.reads,target.weight);
    for(auto id:bound.target.chainRests)Add(&bound.reads,id);
    const RigExecSceneCompileInputs facts(scene);
    const auto bind=[&](const SdfPath &path,const char *name,RigExecValueId *id) {
        if(path.IsEmpty())return true;
        RigExecSceneGraphReadRequest request;
        request.consumer=source.path.AppendProperty(TfToken(name));request.source=path;request.reader=source.path;
        request.domain=RigExecSceneValueDomain::Pose;
        const auto *relation=facts.Relationship(request.consumer);
        request.phase=relation?relation->readPhase:TfToken("base");
        if(!context.resolve(request,id,error))return false;
        if(*id==UINT64_MAX)return Fail(error,"constraint resolver returned no value ID");
        Add(&bound.reads,*id);return true;
    };
    const auto many=[&](const SdfPathVector &paths,const char *name,std::vector<RigExecValueId> *ids) {
        for(const auto &path:paths) {ids->push_back(UINT64_MAX);if(!bind(path,name,&ids->back()))return false;}return true;
    };
    if(!many(source.sources,source.record.kind==RigExecConstraintKind::Aim && source.preserveInputUp?"rigExec:aimTarget":"rigExec:sources",&bound.sources) ||
       !many(source.poleObjects,"rigExec:poleVectorObjects",&bound.poles) ||
       !bind(source.effector,"rigExec:effector",&bound.effector) ||
       !bind(source.worldUpObject,"rigExec:worldUpObject",&bound.worldUp))return false;
    for(const auto &joint:source.ikChain) {
        RigExecSceneGraphReadRequest request;
        request.consumer=source.path;request.source=joint;request.reader=source.path;
        request.domain=RigExecSceneValueDomain::Pose;request.phase=TfToken("preceding");
        bound.chain.push_back(UINT64_MAX);
        if(!context.resolve(request,&bound.chain.back(),error))return false;
        if(bound.chain.back()==UINT64_MAX)return Fail(error,"constraint chain resolver returned no value ID");
        Add(&bound.reads,bound.chain.back());
    }
    if(source.record.kind==RigExecConstraintKind::SingleChainIk &&
        ((!source.useAnimatedTs && target.chainRests.size()!=bound.chain.size()) ||
         target.chainOutputs.size()!=bound.chain.size() ||
         (!target.restLive.empty() && target.restLive.size()!=bound.chain.size())))
        return Fail(error,"single-chain IK graph version cardinality mismatch");
    for(auto id:bound.target.chainOutputs)if(id==UINT64_MAX)
        return Fail(error,"constraint chain output has no value ID");
    if(!source.useAnimatedTs)for(auto id:bound.target.chainRests)if(id==UINT64_MAX)
        return Fail(error,"constraint chain rest has no value ID");
    std::sort(bound.reads.begin(),bound.reads.end());bound.reads.erase(std::unique(bound.reads.begin(),bound.reads.end()),bound.reads.end());
    *destination=std::move(bound);return true;
}
bool RigExecRunConstraintGraph(const RigExecConstraintGraphBinding &bound,RigExecTypedValueStore *store,std::string *error) {
    if(!store || bound.target.output>=store->values.size())return Fail(error,"invalid constraint graph storage");
    auto descriptor=bound.structural;RigExecConstraintInputs input;
    std::vector<char> chainAvailable;
    const auto publish=[&](const RigExecPointFrame &frame,bool blocked,const std::string &reason) {
        for(size_t i=0;i<input.chain.size();++i)
            if(bound.target.chainOutputs[i]!=bound.target.output)
                store->Publish(bound.target.chainOutputs[i],input.chain[i],!chainAvailable[i],false,1,reason);
        store->Publish(bound.target.output,frame,blocked,false,1,reason);
    };
    if(!Frame(*store,bound.target.incoming,&input.incoming)) {
        RigExecPointFrame invalid;invalid.flags=0;publish(invalid,true,"constraint incoming producer unavailable");
        for(auto output:bound.target.chainOutputs)if(output<store->values.size())
            if(output!=bound.target.output)store->Publish(output,invalid,true,false,1,"constraint incoming producer unavailable");
        return Fail(error,"constraint incoming producer unavailable");
    }
    // Stage current entering values and publish each output once, even when
    // disabled, zero-weight, or an unavailable source passes the body through.
    input.chain.resize(bound.chain.size());
    for(size_t i=0;i<bound.chain.size();++i) {
        RigExecPointFrame current;current.flags=0;
        const bool available=Frame(*store,bound.chain[i],&current);
        input.chain[i]=current;
        if(i>=bound.target.chainOutputs.size() || bound.target.chainOutputs[i]>=store->values.size())
            return Fail(error,"invalid constraint chain output storage");
        chainAvailable.push_back(available);
    }
    const auto failure=[&](const std::string &reason) {publish(input.incoming,false,reason);return Fail(error,reason);};
    const auto read=[&](const char *name,VtValue *value) {
        if(std::string(name)=="inputs:defaultWeight" && bound.target.weight!=UINT64_MAX) {
            VtValue field;
            if(!RigExecReadSceneGraphValue(*store,bound.target.weight,&field) ||
                !field.IsHolding<VtFloatArray>() || field.UncheckedGet<VtFloatArray>().size()!=1)
                return Fail(error,"constraint weight field producer unavailable");
            *value=VtValue(field.UncheckedGet<VtFloatArray>()[0]);return true;
        }
        return RigExecReadBoundGraphInput(bound.inputs,name,*store,value,error);
    };
    // Enabled and zero weight gate precede source reads, preserving dormant
    // pass-through even when a disconnected source is malformed.
    VtValue enabledValue,weightValue;
    if(!read("inputs:enabled",&enabledValue))return failure(error?*error:"constraint enabled input unavailable");
    const bool enabled=!enabledValue.IsHolding<bool>() || enabledValue.UncheckedGet<bool>();
    if(!enabled) {publish(input.incoming,false,{});return true;}
    if(!read("inputs:defaultWeight",&weightValue))return failure(error?*error:"constraint weight unavailable");
    const float weight=weightValue.IsHolding<float>()?weightValue.UncheckedGet<float>():1.0f;
    if(!std::isfinite(weight) || weight<0 || weight>1)return failure("constraint weight outside finite [0,1]");
    if(weight<=0) {publish(input.incoming,false,{});return true;}
    if(bound.target.carry!=UINT64_MAX) {
        VtValue matrix;
        if(!RigExecReadSceneGraphValue(*store,bound.target.carry,&matrix,true) || !matrix.IsHolding<GfMatrix4d>())
            return failure("constraint carry producer unavailable");
        input.carry=matrix.UncheckedGet<GfMatrix4d>();
    }
    input.sources.resize(bound.sources.size());
    for(size_t i=0;i<bound.sources.size();++i)if(!Frame(*store,bound.sources[i],&input.sources[i].frame))return failure("constraint source producer unavailable");
    input.poleObjects.resize(bound.poles.size());
    for(size_t i=0;i<bound.poles.size();++i)if(!Frame(*store,bound.poles[i],&input.poleObjects[i]))return failure("constraint pole producer unavailable");
    if(bound.worldUp!=UINT64_MAX) {RigExecPointFrame up;if(!Frame(*store,bound.worldUp,&up))return failure("constraint world-up producer unavailable");input.worldUp=up;}
    if(descriptor.record.kind==RigExecConstraintKind::SingleChainIk) {
        if(!Frame(*store,bound.effector,&input.effector))return failure("single-chain IK effector producer unavailable");
        for(const auto &frame:input.chain)if(!RigExecConstraintFrameUsable(frame))return failure("single-chain IK chain producer unavailable");
        if(!descriptor.useAnimatedTs) {
            std::vector<RigExecPointFrame> rests(bound.chain.size()),prepared;
            for(size_t i=0;i<rests.size();++i) {
                if(i<bound.target.restLive.size() && bound.target.restLive[i])rests[i]=input.chain[i];
                else if(!Frame(*store,bound.target.chainRests[i],&rests[i]))return failure("single-chain IK rest producer unavailable");
            }
            if(!RigExecPrepareRestDerivedIkChain(input.chain,rests,&prepared))return failure("single-chain IK rest-derived preparation failed");
            input.chain=std::move(prepared);
        }
    }
    bool refreshedEnabled=true;float refreshedWeight=1;
    if(!RigExecRefreshConstraintParameters(&descriptor,read,&input,&refreshedEnabled,&refreshedWeight,error))
        return failure(error?*error:"constraint parameter refresh failed");
    RigExecConstraintResult result;
    if(!RigExecRunConstraint(descriptor.record,input,&result,error))return failure(error?*error:"constraint kernel failed");
    if(descriptor.record.kind==RigExecConstraintKind::SingleChainIk) {
        if(result.chain.size()!=bound.target.chainOutputs.size())return failure("single-chain IK result cardinality mismatch");
        for(size_t i=0;i<result.chain.size();++i) {
            input.chain[i]=result.chain[i];chainAvailable[i]=true;
            if(bound.target.chainOutputs[i]==bound.target.output)result.frame=result.chain[i];
        }
    }
    publish(result.frame,false,{});return true;
}
}
