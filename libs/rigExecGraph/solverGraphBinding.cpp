#include "solverGraphBinding.h"
#include <algorithm>
#include "pxr/base/vt/types.h"
namespace rigExec {
namespace {
void Add(std::vector<RigExecValueId> *reads,RigExecValueId id) {
    if(id!=UINT64_MAX)reads->push_back(id);
}
bool Fail(std::string *error,const char *message) {if(error)*error=message;return false;}
bool Frame(const RigExecTypedValueStore &store,RigExecValueId id,RigExecPointFrame *out) {
    if(id==UINT64_MAX)return true;
    VtValue value;
    if(!RigExecReadSceneGraphValue(store,id,&value) || !value.IsHolding<RigExecPointFrame>())return false;
    *out=value.UncheckedGet<RigExecPointFrame>();return true;
}
const RigExecPointFrameArray *Aggregate(const RigExecTypedValueStore &store,RigExecValueId id) {
    if(id>=store.values.size() || store.values[size_t(id)].blocked)return nullptr;
    const auto *value=store.Read<VtValue>(id);
    return value && value->IsHolding<RigExecPointFrameArray>() ? &value->UncheckedGet<RigExecPointFrameArray>() : nullptr;
}
}
bool RigExecBindGraphInputs(const std::map<std::string,RigExecSceneBoundInput> &inputs,
    const std::map<std::string,RigExecSceneTypedRead> &typed,const SdfPath &reader,
    const RigExecSceneGraphBindingContext &context,RigExecBoundGraphInputs *out,
    std::vector<RigExecValueId> *reads,std::string *error) {
    if(!out || !reads || !context.resolve)return Fail(error,"invalid graph input binding context");
    out->clear();
    for(const auto &[name,input]:inputs) {
        RigExecBoundGraphInput bound;
        const auto traversal=typed.find(name);
        if(traversal!=typed.end()) {
            bound.typed=true;
            if(!RigExecBindGraphTypedRead(traversal->second,context,&bound.traversal,error))return false;
            for(const auto *hops:{&bound.traversal.hops,&bound.traversal.doubleHops})
                for(const auto &hop:*hops) {Add(reads,hop.raw);Add(reads,hop.overlay);}
        } else {
            RigExecSceneGraphReadRequest request;
            request.consumer=input.consumer;request.source=input.source;request.reader=reader;
            request.phase=input.readPhase;request.raw=input.route==RigExecSceneReadRoute::Raw;
            if(!context.resolve(request,&bound.direct,error))return false;
            if(bound.direct==UINT64_MAX)return Fail(error,"graph input resolver returned no value ID");
            Add(reads,bound.direct);
        }
        out->emplace(name,std::move(bound));
    }
    return true;
}
bool RigExecReadBoundGraphInput(const RigExecBoundGraphInputs &inputs,const char *name,
    const RigExecTypedValueStore &store,VtValue *out,std::string *error) {
    if(!out)return Fail(error,"null graph input output");*out=VtValue();
    const auto found=inputs.find(name);if(found==inputs.end())return true;
    if(!found->second.typed) {RigExecReadSceneGraphValue(store,found->second.direct,out);return true;}
    std::string unavailable;
    if(RigExecReadGraphTypedRead(found->second.traversal,store,out,&unavailable))return true;
    if(unavailable.empty())return true;
    if(error)*error=unavailable;return false;
}
bool RigExecBindSolverGraph(const RigExecSceneSolverDescriptor &source,
    const RigExecSceneGraphBindingContext &context,RigExecValueId output,
    RigExecSolverGraphBinding *destination,std::string *error) {
    if(!destination || !context.resolve || output==UINT64_MAX)return Fail(error,"invalid solver graph destination");
    RigExecSolverGraphBinding bound;bound.record=source.record;bound.output=output;
    bound.hasSpace=!source.space.IsEmpty();bound.spaceRest=source.spaceRest;
    if(!RigExecBindGraphInputs(source.inputs,source.typedReads,source.path,context,&bound.inputs,&bound.reads,error))return false;
    const auto bind=[&](const SdfPath &path,const char *name,RigExecValueId *id,bool aggregate=false,bool *required=nullptr) {
        if(path.IsEmpty() || source.record.degenerate)return true;
        RigExecSceneGraphReadRequest request;
        request.consumer=source.path.AppendProperty(TfToken(name));request.source=path;request.reader=source.path;
        request.domain=aggregate?RigExecSceneValueDomain::SolverAggregate:RigExecSceneValueDomain::Pose;
        const auto relation=source.frameBindings.find(name);
        request.phase=relation==source.frameBindings.end()?TfToken("base"):relation->second.phase;
        if(required)*required=request.phase!="base";
        if(!context.resolve(request,id,error))return false;
        if(*id==UINT64_MAX)return Fail(error,"solver resolver returned no value ID");
        Add(&bound.reads,*id);return true;
    };
    for(const auto &control:source.controls) {
        bound.controls.push_back(UINT64_MAX);
        if(!bind(control,"rigExec:controls",&bound.controls.back()))return false;
    }
    const bool twist=source.record.kind==RigExecSolverKind::TwistDistribution;
    const bool ik=source.record.kind==RigExecSolverKind::TwoBoneIk;
    if(!bind(source.start,"rigExec:startFrame",&bound.start) ||
       !bind(source.root,twist?"rigExec:start":"rigExec:rootControl",&bound.root) ||
       !bind(source.mid,"rigExec:midControl",&bound.mid) ||
       !bind(source.end,twist?"rigExec:end":ik?"rigExec:effectorControl":"rigExec:endControl",&bound.end) ||
       !bind(source.pole,"rigExec:poleControl",&bound.pole) || !bind(source.space,"rigExec:space",&bound.space) ||
       !bind(source.blendA,"rigExec:inputA",&bound.blendA,true,&bound.blendARequired) ||
       !bind(source.blendB,"rigExec:inputB",&bound.blendB,true,&bound.blendBRequired))return false;
    const auto rest=[&](const SdfPath &path,RigExecValueId *id,bool live=false,bool *liveRest=nullptr) {
        if(path.IsEmpty() || source.record.degenerate)return true;
        RigExecSceneGraphReadRequest request;
        request.consumer=source.path;request.source=path;request.reader=source.path;
        request.domain=RigExecSceneValueDomain::Pose;request.computation="computeRestFrame";
        request.phase=TfToken(live?"preceding":"base");request.liveRest=liveRest;
        if(!context.resolve(request,id,error))return false;
        if(*id==UINT64_MAX)return Fail(error,"solver resolver returned no value ID");
        Add(&bound.reads,*id);return true;
    };
    for(const auto &control:source.controls) {
        bound.controlRests.push_back(UINT64_MAX);
        if(!rest(control,&bound.controlRests.back()))return false;
    }
    for(const auto &joint:source.joints) {
        bound.jointRests.push_back(UINT64_MAX);
        bool liveRest=false;
        if(!rest(joint,&bound.jointRests.back(),true,&liveRest))return false;
        bound.jointRestLive.push_back(liveRest);
    }
    bound.jointElements=source.jointElements;
    if(!rest(source.start,&bound.startRest) || !rest(source.root,&bound.rootRest) ||
       !rest(source.mid,&bound.midRest) || !rest(source.end,&bound.endRest) ||
       !rest(source.space,&bound.spaceRestId))return false;
    if(source.record.kind==RigExecSolverKind::Ribbon && !source.ribbonPoints.IsEmpty()) {
        RigExecSceneGraphReadRequest request;
        request.consumer=source.path;request.source=source.ribbonPoints;request.reader=source.path;
        request.domain=RigExecSceneValueDomain::Points;request.raw=true;request.atDefault=true;
        if(!context.resolve(request,&bound.ribbonRestPoints,error))return false;
        if(bound.ribbonRestPoints==UINT64_MAX)return Fail(error,"ribbon rest resolver returned no value ID");
        Add(&bound.reads,bound.ribbonRestPoints);
    }
    std::sort(bound.reads.begin(),bound.reads.end());bound.reads.erase(std::unique(bound.reads.begin(),bound.reads.end()),bound.reads.end());
    *destination=std::move(bound);return true;
}
bool RigExecRunSolverGraph(const RigExecSolverGraphBinding &bound,RigExecTypedValueStore *store,
    RigExecSolverWorkspace *workspace,std::string *error) {
    if(!store || !workspace || bound.output>=store->values.size())return Fail(error,"invalid solver graph storage");
    RigExecSolverInputs input;RigExecPointFrameArray output;auto record=bound.record;
    const auto fail=[&](const std::string &message) {store->Publish(bound.output,VtValue(output),true,false,0,message);if(error)*error=message;return false;};
    if(!bound.record.degenerate) {
        record.controlRests.resize(bound.controlRests.size());
        for(size_t i=0;i<bound.controlRests.size();++i) {
            RigExecPointFrame frame;
            if(!Frame(*store,bound.controlRests[i],&frame))return fail("solver control rest producer unavailable");
            record.controlRests[i]=frame.points;
        }
        record.jointRests.resize(bound.jointRests.size());record.restIsLive.assign(bound.jointRests.size(),false);
        std::vector<RigExecPointFrame> jointFrames(bound.jointRests.size());
        for(size_t i=0;i<bound.jointRests.size();++i) {
            if(!Frame(*store,bound.jointRests[i],&jointFrames[i]))return fail("solver joint rest producer unavailable");
            record.jointRests[i]=jointFrames[i].points;
            record.restIsLive[i]=i<bound.jointRestLive.size() && bound.jointRestLive[i];
        }
        RigExecPointFrame startRest,rootRest,midRest,endRest,spaceRest;
        if(!Frame(*store,bound.startRest,&startRest) || !Frame(*store,bound.rootRest,&rootRest) ||
           !Frame(*store,bound.midRest,&midRest) || !Frame(*store,bound.endRest,&endRest) ||
           !Frame(*store,bound.spaceRestId,&spaceRest))return fail("solver endpoint rest producer unavailable");
        if(bound.startRest!=UINT64_MAX)record.startRest=startRest.points;
        if(bound.rootRest!=UINT64_MAX) {record.twistStartRest=rootRest.points;record.splineRootRest=rootRest;}
        if(bound.midRest!=UINT64_MAX)record.splineMidRest=midRest;
        if(bound.endRest!=UINT64_MAX) {record.twistEndRest=endRest.points;record.splineEndRest=endRest;}
        if(record.kind==RigExecSolverKind::TwoBoneIk)
            for(size_t i=0;i<jointFrames.size();++i) {
                if(i>=bound.jointElements.size() || bound.jointElements[i]<0 || bound.jointElements[i]>=3)
                    return fail("two-bone IK rest element binding invalid");
                record.ikRests[size_t(bound.jointElements[i])]=jointFrames[i].points;
            }
        if(record.kind==RigExecSolverKind::SplineIk) {
            record.splineRestFrames.resize(record.splineCount);record.splineJointRests.resize(record.splineCount);
            for(size_t i=0;i<jointFrames.size();++i) {
                if(i>=bound.jointElements.size() || bound.jointElements[i]<0 || size_t(bound.jointElements[i])>=record.splineCount)
                    return fail("spline IK rest element binding invalid");
                const auto element=size_t(bound.jointElements[i]);
                record.splineRestFrames[element]=jointFrames[i];record.splineJointRests[element]=jointFrames[i].points;
            }
            record.splineRest=RigExecSplineIkMakeRest(record.splineRestFrames,record.splineRootRest,record.splineMidRest,
                record.splineEndRest,record.splineRestWeights,record.splineRestMode);
        }
        if(record.kind==RigExecSolverKind::Ribbon && bound.ribbonRestPoints!=UINT64_MAX) {
            VtValue restPoints;RigExecReadSceneGraphValue(*store,bound.ribbonRestPoints,&restPoints);
            record.ribbonRestPoints.clear();
            if(restPoints.IsHolding<VtVec3fArray>()) {
                const auto &points=restPoints.UncheckedGet<VtVec3fArray>();
                record.ribbonRestPoints.assign(points.begin(),points.end());
            }
        }
        input.controls.resize(bound.controls.size());
        for(size_t i=0;i<bound.controls.size();++i)if(!Frame(*store,bound.controls[i],&input.controls[i]))return fail("solver control producer unavailable");
        if(!Frame(*store,bound.start,&input.start) || !Frame(*store,bound.root,&input.root) ||
           !Frame(*store,bound.mid,&input.mid) || !Frame(*store,bound.end,&input.end) ||
           !Frame(*store,bound.pole,&input.pole) || !Frame(*store,bound.space,&input.space))return fail("solver frame producer unavailable");
        input.hasSpace=bound.hasSpace;input.spaceRest=bound.spaceRestId==UINT64_MAX?bound.spaceRest:spaceRest.points;
        input.blendA=Aggregate(*store,bound.blendA);input.blendB=Aggregate(*store,bound.blendB);
        if((bound.blendARequired && !input.blendA) || (bound.blendBRequired && !input.blendB))return fail("solver aggregate producer unavailable");
        const auto read=[&](const char *name,VtValue *value) {return RigExecReadBoundGraphInput(bound.inputs,name,*store,value,error);};
        if(!RigExecRefreshSolverParameters(bound.record.kind,read,&input,error)) {return fail(error && !error->empty()?*error:std::string("solver parameter refresh failed"));}
    }
    const bool valid=RigExecRunSolver(record,input,workspace,&output,error);
    store->Publish(bound.output,VtValue(output),!valid,false,output.frames.size(),valid?std::string():error?*error:"solver kernel failed");return valid;
}
}
