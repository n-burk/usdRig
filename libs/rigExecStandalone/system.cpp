#include "system.h"
#include "sceneRuntime.h"
#include "rigExec/inputReplayValues.h"
#include "pxr/base/vt/types.h"
#include "rigExec/movers/moverRegistry.h"
#include <algorithm>
#include <cstring>
#include <cmath>
namespace rigExec {
namespace {
bool Fail(std::string *error,const std::string &message) {if(error)*error=message;return false;}
bool Same(const VtValue &a,const VtValue &b) {
    std::string x,y,error;return RigExecEncodeInputValue(a,&x,&error) && RigExecEncodeInputValue(b,&y,&error) && x==y;
}
bool Time(const std::string &key,UsdTimeCode *out) {
    if(key=="default"){*out=UsdTimeCode::Default();return true;}
    const size_t prefix=key.rfind("pre:",0)==0?4:key.rfind("exact:",0)==0?6:0;
    if(!prefix || key.size()!=prefix+16)return false;
    uint64_t bits=0;
    for(size_t i=prefix;i<key.size();++i){char c=key[i];if(!((c>='0'&&c<='9')||(c>='a'&&c<='f')))return false;bits=bits*16+(c<='9'?c-'0':c-'a'+10);}
    double value;std::memcpy(&value,&bits,sizeof(value));
    *out=prefix==4?UsdTimeCode::PreTime(value):UsdTimeCode(value);
    return RigExecStandaloneTimeKey(*out)==key;
}
// These schema fields are captured structural tables rather than run-time
// leaves. Ordinary numeric channels and painted values refresh sampled rows.
bool ProjectorCapturedDefault(const RigExecSceneDb &database,const RigExecSceneProgram &program,const SdfPath &path) {
    if(path.GetName().rfind("xformOp:",0)!=0)return false;
    for(const auto &operation:program.operations) {
        const auto *geometry=std::get_if<RigExecSceneGeometryOp>(&operation);
        if(!geometry || !geometry->binding.descriptor.matrixOutput)continue;
        const auto mesh=geometry->binding.descriptor.pointsTarget.GetPrimPath();
        for(auto owner=mesh;!owner.IsEmpty() && owner!=SdfPath::AbsoluteRootPath();owner=owner.GetParentPath()) {
            const auto order=database.attributes.find(owner.AppendProperty(TfToken("xformOpOrder")));
            if(order==database.attributes.end())continue;
            const auto value=order->second.resolved.find("default");
            if(value==order->second.resolved.end() || !value->second.IsHolding<VtTokenArray>())continue;
            bool reset=false;
            for(const auto &token:value->second.UncheckedGet<VtTokenArray>()) {
                auto name=token.GetString();
                if(name=="!resetXformStack!"){reset=true;continue;}
                if(name.rfind("!invert!",0)==0)name.erase(0,8);
                if(owner==path.GetPrimPath() && name==path.GetName())return true;
            }
            if(reset)break;
        }
    }
    return false;
}
bool StructuralValue(const RigExecSceneDb &database,const SdfPath &path,const RigExecSceneProgram *program=nullptr) {
    if(program && ProjectorCapturedDefault(database,*program,path))return true;
    const auto node=database.prims.find(path.GetPrimPath());
    const auto type=node==database.prims.end()?TfToken():node->second.type;
    if(type=="RigExecTwistDistribution" && (path.GetName()=="rigExec:weights" || path.GetName()=="rigExec:count"))return true;
    const auto typeName=type.GetString();
    if(typeName.size()>=10 && typeName.compare(typeName.size()-10,10,"Constraint")==0 && path.GetName()=="rigExec:rotationOrder")return true;
    if(type=="RigExecPose")return true;
    if(type=="RigExecPoseInterpolator" && path.GetName().rfind("rigExec:",0)==0)return true;
    if(type=="RigExecSpaceSwitch" && (path.GetName().rfind("inputs:affect",0)==0 || path.GetName()=="inputs:sourceWeights"))return true;
    for(const auto &handler:RigExecMoverHandlers())if(handler.resolveOp(TfToken())==RigExecRevisionOp::External)
        for(const auto &schema:handler.sceneDataSchemas)if(type==schema)return true;
    static const std::set<std::string> names={"rigExec:jointElements","rigExec:controlSpace",
        "rigExec:scaleBlend","rigExec:rotationBlend",
        "rigExec:volumeWeights","rigExec:restLength","rigExec:rootTangent",
        "rigExec:operation",
        "rigExec:method","rigExec:skinMethod","rigExec:channels","rigExec:driverChannels",
        "rigExec:inbetweenWeights","rigExec:evaluationMode",
        "rigExec:orientationMode","rigExec:poleVectorMode","rigExec:solverMode",
        "rigExec:aimAxis","rigExec:worldUpType","rigExec:spaceLabels",
        "rigExec:rotationFilters","rigExec:twistAxis","rigExec:blendShear",
        "rigExec:worldUpRotationOnly","rigExec:representation","rigExec:rangePolicy",
        "rigExec:combineMode","rigExec:falloffProfile","xformOpOrder"};
    return names.count(path.GetName())!=0;
}
}
struct RigExecStandaloneSystem::_Impl {
    RigExecSceneDb database;
    RigExecStandaloneSceneRuntime runtime;
    std::vector<RigExecValueAddress> addresses;
    std::vector<UsdTimeCode> times;
    std::vector<std::string> timeKeys;
    std::vector<RigExecValueId> taps;
    uint64_t generation=0;size_t invalidations=0;
    bool compiled=false;
    explicit _Impl(const RigExecSceneDb &db):database(db){}
    RigExecValueId PhaseId(const SdfPath &path,RigExecSceneValueDomain domain,const TfToken &phase) const {
        const auto found=runtime.GetProgram().layout.routes.chains.find({path,domain});
        if(found==runtime.GetProgram().layout.routes.chains.end())return UINT64_MAX;
        const auto &chain=found->second;
        return phase=="base" || chain.versions.empty()?(domain==RigExecSceneValueDomain::Pose?chain.poseBase:chain.base):chain.versions.back().value;
    }
    RigExecValueId AddressId(const RigExecValueAddress &address) const {
        const auto &layout=runtime.GetProgram().layout;
        const auto &name=address.publicComputation;
        if(address.target.IsPropertyPath()) {
            if(name=="computeResolvedValue") {const auto found=layout.sampleIndex.find(address.target);return found==layout.sampleIndex.end()?UINT64_MAX:layout.samples[found->second].raw;}
            if(!name.IsEmpty() && name!="computeValue")return UINT64_MAX;
            const auto domain=address.target.GetName()=="points"?RigExecSceneValueDomain::Points:RigExecSceneValueDomain::Property;
            const auto chain=layout.routes.chains.find({address.target,domain});
            if(address.phase=="final" && chain!=layout.routes.chains.end() && !chain->second.versions.empty())return chain->second.versions.back().value;
            return runtime.PublicValueId(address.target);
        }
        if(name=="computePointFrame" || name=="computeMatrix")return PhaseId(address.target,RigExecSceneValueDomain::Pose,address.phase);
        if(name=="computePointFrameArray")return PhaseId(address.target,RigExecSceneValueDomain::SolverAggregate,address.phase);
        if(name=="computeWeightPacket")return PhaseId(address.target,RigExecSceneValueDomain::Weight,address.phase);
        return layout.providers.FindValue(RigExecProviderValueKey(address.target,name.GetString()));
    }
    bool Prepare(std::string *error) {
        for(const auto &address:addresses) {
            if(!database.HasObject(address.target))return Fail(error,"tap provider missing or inactive: "+address.target.GetString());
            if(address.phase!="base" && address.phase!="final")return Fail(error,"standalone tap phase must be base or final");
            if(address.target.IsPropertyPath() && !database.attributes.count(address.target))return Fail(error,"tap property is not an attribute: "+address.target.GetString());
        }
        if(compiled)return true;
        if(!database.Validate(error))return false;
        times.clear();timeKeys.clear();
        for(const auto &key:database.identities){UsdTimeCode time;if(!Time(key,&time))return Fail(error,"invalid standalone identity");times.push_back(time);timeKeys.push_back(key);}
        SdfPath root=SdfPath::AbsoluteRootPath();size_t roots=0;
        for(const auto &[path,prim]:database.prims)if(prim.type=="RigExecRoot" && database.IsActive(path)){root=path;++roots;}
        if(roots>1)root=SdfPath::AbsoluteRootPath();
        SdfPathVector publicAttributes;
        for(const auto &address:addresses)publicAttributes.push_back(address.target);
        if(!runtime.Prepare(database,root,times,error,publicAttributes))return false;
        taps.clear();
        for(const auto &address:addresses){auto id=AddressId(address);if(id==UINT64_MAX)return Fail(error,"standalone computation is unavailable: "+address.target.GetString()+" "+address.publicComputation.GetString());taps.push_back(id);}
        compiled=true;return true;
    }
    void Changed(const SdfPath &path) {
        if(!compiled)return;
        const auto &program=runtime.GetProgram();const auto found=program.layout.sampleIndex.find(path);
        if(found==program.layout.sampleIndex.end())return;
        const auto &sample=program.layout.samples[found->second];
        std::set<RigExecValueId> reached;std::vector<RigExecValueId> pending;
        for(auto id:{sample.raw,sample.rawDefault,sample.providerRaw})if(id!=UINT64_MAX && reached.insert(id).second)pending.push_back(id);
        for(size_t i=0;i<pending.size();++i){const auto readers=program.graph.readers.find(pending[i]);if(readers==program.graph.readers.end())continue;for(auto op:readers->second)for(auto out:program.graph.ops[op].descriptor.writes)if(reached.insert(out).second)pending.push_back(out);}
        for(auto tap:taps)if(reached.count(tap))++invalidations;
    }
    RigExecStandaloneResult Evaluate(UsdTimeCode time) {
        RigExecStandaloneResult result;result.time=time;
        const auto key=RigExecStandaloneTimeKey(time);
        if(key.empty() || !database.identities.count(key)){result.diagnostics.push_back("unexported standalone evaluation identity");return result;}
        std::string error;if(!Prepare(&error)){result.diagnostics.push_back(error);return result;}
        // Refresh all declared raw and Default rows without replacing the graph.
        std::vector<UsdTimeCode> currentTimes;std::vector<std::string> currentKeys;
        for(const auto &identity:database.identities){UsdTimeCode identityTime;if(!Time(identity,&identityTime)){result.diagnostics.push_back("invalid standalone identity");return result;}currentTimes.push_back(identityTime);currentKeys.push_back(identity);}
        if(!runtime.RefreshSamples(database,currentTimes,&error)){result.diagnostics.push_back(error);return result;}
        const auto index=std::find(currentKeys.begin(),currentKeys.end(),key)-currentKeys.begin();
        if(!runtime.Evaluate(size_t(index),{},&error)){result.diagnostics.push_back(error);return result;}
        result.valid=true;result.values.reserve(addresses.size());
        for(size_t i=0;i<addresses.size();++i) {
            const auto &address=addresses[i];VtValue value;
            const bool found=RigExecReadSceneGraphValue(runtime.GetRuntime().values,taps[i],&value,address.target.IsPrimPath() && address.publicComputation=="computeMatrix");
            result.values.push_back(std::move(value));
            if(!found || result.values.back().IsEmpty()){result.valid=false;result.diagnostics.push_back("standalone tap produced no value: "+address.target.GetString()+" "+address.publicComputation.GetString());}
            if(taps[i]<runtime.GetRuntime().values.values.size()) {const auto &diagnostic=runtime.GetRuntime().values.values[size_t(taps[i])].error;if(!diagnostic.empty())result.diagnostics.push_back(diagnostic);}
        }
        for(const auto &[owner,reason]:runtime.GetProgram().skippedOperations)
            result.diagnostics.push_back(owner.GetString()+": "+reason);
        for(const auto &cycle:runtime.GetProgram().graph.cycles){std::string diagnostic="cyclic graph:";for(const auto &member:cycle)diagnostic+=" "+member;result.diagnostics.push_back(std::move(diagnostic));}
        if(result.valid)result.generation=++generation;return result;
    }
};
RigExecStandaloneSystem::RigExecStandaloneSystem(const RigExecSceneDb &db):_impl(std::make_unique<_Impl>(db)){}
RigExecStandaloneSystem::~RigExecStandaloneSystem()=default;
int RigExecStandaloneSystem::AddTap(const RigExecValueAddress &address){_impl->addresses.push_back(address);_impl->compiled=false;return int(_impl->addresses.size()-1);}
bool RigExecStandaloneSystem::Prepare(std::string *error){return _impl->Prepare(error);}
RigExecStandaloneResult RigExecStandaloneSystem::Evaluate(UsdTimeCode time){return _impl->Evaluate(time);}
RigExecStandaloneResult RigExecStandaloneSystem::EvaluateResolved(UsdTimeCode time,const std::map<SdfPath,VtValue> &states) {
    std::map<SdfPath,RigExecStandaloneResolvedState> typed;
    const auto identity=RigExecStandaloneTimeKey(time);
    for(const auto &[path,value]:states) {
        const auto attribute=_impl->database.attributes.find(path);
        const bool blocked=value.IsEmpty() && attribute!=_impl->database.attributes.end() && attribute->second.blockedIdentities.count(identity);
        typed.emplace(path,RigExecStandaloneResolvedState{value,blocked});
    }
    return EvaluateResolved(time,typed);
}
RigExecStandaloneResult RigExecStandaloneSystem::EvaluateResolved(UsdTimeCode time,
    const std::map<SdfPath,RigExecStandaloneResolvedState> &states) {
    RigExecStandaloneResult failure;failure.time=time;const auto identity=RigExecStandaloneTimeKey(time);
    if(identity.empty() || states.size()!=_impl->database.attributes.size()){failure.diagnostics.push_back("transient evaluation requires a complete resolved-state set and finite time");return failure;}
    for(const auto &[path,attr]:_impl->database.attributes) {
        auto state=states.find(path);
        if(state==states.end() || (state->second.blocked && !state->second.value.IsEmpty()) ||
           (!state->second.value.IsEmpty() && state->second.value.GetType()!=attr.type.GetType())) {
            failure.diagnostics.push_back("invalid transient resolved state for "+path.GetString());return failure;
        }
    }
    std::string prepareError;
    if(!_impl->Prepare(&prepareError)){failure.diagnostics.push_back(prepareError);return failure;}
    for(const auto &[path,attr]:_impl->database.attributes) {
        const auto state=states.find(path);
        if(StructuralValue(_impl->database,path,&_impl->runtime.GetProgram())) {
            const auto captured=attr.resolved.find("default");
            const VtValue expected=captured==attr.resolved.end()?VtValue():captured->second;
            if(!Same(state->second.value,expected) || state->second.blocked!=(attr.blockedIdentities.count("default")!=0)) {
                failure.diagnostics.push_back("transient structural state differs from compiled Default: "+path.GetString());return failure;
            }
        }
    }
    const bool retained=_impl->database.identities.count(identity)!=0;
    std::map<SdfPath,VtValue> previous;std::set<SdfPath> blocked;
    for(const auto &[path,attr]:_impl->database.attributes){if(retained)previous[path]=attr.resolved.at(identity);if(attr.blockedIdentities.count(identity))blocked.insert(path);}
    struct Restore {
        RigExecSceneDb &database;std::string identity;bool retained;
        const std::map<SdfPath,VtValue> &previous;const std::set<SdfPath> &blocked;
        ~Restore() {
            for(auto &[path,attr]:database.attributes) {
                if(retained)attr.resolved[identity]=previous.at(path);else attr.resolved.erase(identity);
                if(blocked.count(path))attr.blockedIdentities.insert(identity);else attr.blockedIdentities.erase(identity);
            }
            if(!retained)database.identities.erase(identity);
        }
    } restore{_impl->database,identity,retained,previous,blocked};
    for(auto &[path,attr]:_impl->database.attributes) {
        attr.resolved[identity]=states.at(path).value;
        if(states.at(path).blocked)attr.blockedIdentities.insert(identity);else attr.blockedIdentities.erase(identity);
        _impl->Changed(path);
    }
    _impl->database.identities.insert(identity);
    return _impl->Evaluate(time);
}
bool RigExecStandaloneSystem::SetValue(const SdfPath &path,UsdTimeCode time,const VtValue &value,std::string *error) {
    auto attr=_impl->database.attributes.find(path);const auto identity=RigExecStandaloneTimeKey(time);
    if(attr==_impl->database.attributes.end() || !_impl->database.identities.count(identity))return Fail(error,"attribute or exported identity not found");
    if(!value.IsEmpty() && value.GetType()!=attr->second.type.GetType())return Fail(error,"value does not match exact native attribute type");
    if(Same(attr->second.resolved[identity],value))return true;
    _impl->Changed(path);attr->second.resolved[identity]=value;if(!value.IsEmpty())attr->second.blockedIdentities.erase(identity);
    if(StructuralValue(_impl->database,path,time.IsDefault()?&_impl->runtime.GetProgram():nullptr))_impl->compiled=false;return true;
}
bool RigExecStandaloneSystem::SetConnections(const SdfPath &path,const SdfPathVector &sources,std::string *error) {
    auto attr=_impl->database.attributes.find(path);if(attr==_impl->database.attributes.end())return Fail(error,"attribute not found");
    if(_impl->database.prims.at(path.GetPrimPath()).type.GetString().rfind("RigExec",0)==0 && !attr->second.type.IsArray() && sources.size()>1)return Fail(error,"RigExec scalar attribute requires at most one connection");
    for(const auto &source:sources){auto input=_impl->database.attributes.find(source);if(input==_impl->database.attributes.end())return Fail(error,"connection needs a retained source attribute");}
    if(attr->second.connections==sources)return true;attr->second.connections=sources;attr->second.hasAuthoredConnections=true;_impl->compiled=false;return true;
}
bool RigExecStandaloneSystem::SetTargets(const SdfPath &path,const SdfPathVector &targets,std::string *error) {
    auto relation=_impl->database.relationships.find(path);if(relation==_impl->database.relationships.end())return Fail(error,"relationship not found");
    for(const auto &target:targets)if(!_impl->database.HasObject(target))return Fail(error,"relationship target not found");
    if(relation->second.targets==targets)return true;relation->second.targets=targets;_impl->compiled=false;return true;
}
bool RigExecStandaloneSystem::SetPrimActive(const SdfPath &path,bool active,std::string *error) {
    auto prim=_impl->database.prims.find(path);if(prim==_impl->database.prims.end() || path==SdfPath::AbsoluteRootPath())return Fail(error,"editable prim not found");
    if(prim->second.active==active)return true;prim->second.active=active;_impl->compiled=false;return true;
}
const void *RigExecStandaloneSystem::GetCompilerIdentity() const {return _impl->runtime.GetProgram().identity.get();}
size_t RigExecStandaloneSystem::GetValueInvalidationCount() const {return _impl->invalidations;}
} // namespace rigExec
