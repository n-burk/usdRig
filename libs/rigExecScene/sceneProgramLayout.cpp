#include "sceneProgramLayout.h"
#include "sceneCompileInputs.h"
#include <algorithm>
namespace rigExec {
namespace { bool Fail(std::string *error,const std::string &message){if(error)*error=message;return false;} }
RigExecValueId RigExecSceneProgramLayout::Allocate(const std::string &key) {
    const auto id=RigExecValueId(valueKeys.size());valueKeys.push_back(key);return id;
}
bool RigExecSceneProgramLayout::RunBaseSelection(size_t index,RigExecTypedValueStore *store) const {
    if(!store || index>=baseSelections.size())return false;
    const auto &binding=baseSelections[index];const auto &overlay=store->values[size_t(binding.overlay)];
    return store->Copy(binding.output,overlay.initialized && overlay.authoritative?binding.overlay:binding.raw,overlay.initialized && overlay.authoritative),true;
}
bool RigExecSceneProgramLayout::Prepare(const RigExecSceneDescriptors &scene,std::string *error) {
    *this=RigExecSceneProgramLayout();
    if(!RigExecBuildProviderProgram(scene,true,&providers,error))return false;
    initialProviderOps=providers.ops.size();identityCount=scene.identities.size();
    valueKeys=providers.valueKeys;descriptors=providers.descriptors;leaves=providers.leaves;
    for(const auto &[path,node]:scene.nodes) if(!node.fact.active &&
        (node.domain==RigExecSceneDomain::Provider || node.transformProvider)) {
        for(const char *name:{"computePointFrame","computeRestFrame"}) {
            const auto key=RigExecProviderValueKey(path,name);
            const auto id=Allocate(key);providers.valueIds[key]=id;
            unavailableFrames.push_back(id);
        }
    }
    providers.valueKeys=valueKeys;
    RigExecSceneCompileInputs source(scene);
    for(const auto &[path,attribute]:scene.attributes) {
        RigExecSceneSampleSlot slot;slot.path=path;
        const auto existing=providers.rawInputs.find(path);
        slot.providerRaw=existing==providers.rawInputs.end()?UINT64_MAX:existing->second;
        slot.raw=Allocate("sourceRaw:"+path.GetString());slot.type=std::type_index(attribute.fact.type.GetType().GetTypeid());
        slot.rawDefault=Allocate("rawDefault:"+path.GetString());
        slot.overlay=Allocate("overlay:"+path.GetString());
        RigExecSceneBoundInput raw;
        if(!source.Bind(path,RigExecSceneReadRoute::Raw,&raw,error))return false;
        slot.values=raw.values;slot.available=raw.available;slot.blocked=raw.blocked;
        bool blocked=false;
        slot.defaultAvailable=source.Read(raw,UsdTimeCode::Default(),&slot.defaultValue,&blocked,nullptr);
        slot.defaultBlocked=blocked;
        sampleIndex[path]=samples.size();samples.push_back(std::move(slot));
        for(const auto id:{samples.back().raw,samples.back().rawDefault,samples.back().overlay})
            if(std::find(leaves.begin(),leaves.end(),id)==leaves.end())leaves.push_back(id);
        const auto domain=path.GetNameToken()=="points"?RigExecSceneValueDomain::Points:RigExecSceneValueDomain::Property;
        const auto base=Allocate("base:"+path.GetString());
        baseSelections.push_back({samples.back().raw,samples.back().overlay,base});
        RigExecOpDescriptor select;select.key="selectBase:"+path.GetString();select.kind=100;
        select.reads={samples.back().raw,samples.back().overlay};select.writes={base};descriptors.push_back(std::move(select));
        const auto selectedDefault=Allocate("selectedDefault:"+path.GetString());samples.back().selectedDefault=selectedDefault;
        baseSelections.push_back({samples.back().rawDefault,samples.back().overlay,selectedDefault});
        RigExecOpDescriptor defaultSelect;defaultSelect.key="selectDefault:"+path.GetString();defaultSelect.kind=100;
        defaultSelect.reads={samples.back().rawDefault,samples.back().overlay};defaultSelect.writes={selectedDefault};descriptors.push_back(std::move(defaultSelect));
        if(!routes.RegisterBase({path,domain},base,error))return false;
    }
    for(const auto &[path,node]:scene.nodes) {
        {
            auto id=providers.FindValue(RigExecProviderValueKey(path,"computePointFrame"));
            if(id!=UINT64_MAX && !routes.RegisterBase({path,RigExecSceneValueDomain::Pose},id,error))return false;
        }
        if(node.domain==RigExecSceneDomain::Solver) {
            const auto id=Allocate("solverAggregate:"+path.GetString());solverAggregates[path]=id;
            if(!routes.RegisterBase({path,RigExecSceneValueDomain::SolverAggregate},id,error))return false;
        }
        if(node.domain==RigExecSceneDomain::Weight) {
            const auto id=Allocate("weightPacket:"+path.GetString());weightPackets[path]=id;
            if(!routes.RegisterBase({path,RigExecSceneValueDomain::Weight},id,error))return false;
        }
    }
    return true;
}
bool RigExecSceneProgramLayout::Resolve(const RigExecSceneDescriptors &scene,
    const RigExecSceneGraphReadRequest &request,RigExecValueId *out,std::string *error) const {
    if(!out)return Fail(error,"null scene layout read destination");*out=UINT64_MAX;
    if(request.liveRest)*request.liveRest=false;
    if(request.computation=="computeRestFrame" && request.phase=="preceding") {
        const auto chain=routes.chains.find({request.source.GetPrimPath(),RigExecSceneValueDomain::Pose});
        const auto reader=scene.nodes.find(request.reader.GetPrimPath());
        if(chain!=routes.chains.end() && reader!=scene.nodes.end())for(const auto &version:chain->second.versions) {
            if(version.ordinal>=reader->second.stackOrdinal)break;*out=version.value;
        }
        if(*out!=UINT64_MAX){if(request.liveRest)*request.liveRest=true;return true;}
    }
    if(!request.computation.empty()) {
        *out=providers.FindValue(RigExecProviderValueKey(request.source.GetPrimPath(),request.computation));
        return *out!=UINT64_MAX || Fail(error,"missing provider computation: "+request.source.GetString()+":"+request.computation);
    }
    if(request.raw) {
        const auto found=sampleIndex.find(request.source);
        if(found==sampleIndex.end())return Fail(error,"missing raw scene input: "+request.source.GetString());
        const auto &sample=samples[found->second];*out=request.atDefault?sample.rawDefault:sample.raw;return true;
    }
    // Explicit computed attributes are production arithmetic values, never raw
    // schema defaults or ordinary user-overlay leaves.
    const auto attribute=providers.attributeValues.find(request.source);
    const auto captured=scene.attributes.find(request.source);
    bool computed=false;
    if(captured!=scene.attributes.end())for(const auto &input:captured->second.inputs)
        computed|=input.state==RigExecSceneInputState::Computed;
    if((computed || request.domain==RigExecSceneValueDomain::ProviderSpace) && attribute!=providers.attributeValues.end()){*out=attribute->second;return true;}
    if(request.atDefault) {
        const auto sampled=sampleIndex.find(request.source);
        if(sampled!=sampleIndex.end()){*out=samples[sampled->second].selectedDefault;return true;}
    }
    return routes.Resolve(scene,{request.source,request.domain},request.reader,
        request.phase.IsEmpty()?TfToken("base"):request.phase,out,error);
}
RigExecSceneGraphBindingContext RigExecSceneProgramLayout::BindingContext(const RigExecSceneDescriptors &scene) const {
    RigExecSceneGraphBindingContext context;
    context.resolve=[this,&scene](const auto &request,auto *out,auto *error){return Resolve(scene,request,out,error);};
    context.weightPacket=[this](const SdfPath &path,RigExecValueId *out,std::string *error){
        const auto found=weightPackets.find(path);
        if(!out || found==weightPackets.end())return Fail(error,"missing scene weight packet: "+path.GetString());
        *out=found->second;return true;
    };return context;
}
bool RigExecSceneProgramLayout::Sample(size_t identity,const std::map<SdfPath,VtValue> &overlays,
    RigExecTypedValueStore *store,std::vector<RigExecValueId> *changed,std::string *error) const {
    if(!store || !changed || store->values.size()!=valueKeys.size())return Fail(error,"scene store layout mismatch");
    if(identity>=identityCount)return Fail(error,"scene sample identity is unavailable");
    for(const auto &[path,value]:overlays) {
        const auto found=sampleIndex.find(path);
        if(found==sampleIndex.end() || (!value.IsEmpty() && std::type_index(value.GetTypeid())!=samples[found->second].type))return Fail(error,"scene overlay native type mismatch: "+path.GetString());
    }
    changed->clear();
    for(const auto &[id,value]:constants)if(store->PublishSource(id,value,false,false))changed->push_back(id);
    for(const auto &sample:samples) {
        if(identity>=sample.values.size())return Fail(error,"scene sample identity is unavailable");
        const auto overlay=overlays.find(sample.path);
        const bool have=identity<sample.available.size() && sample.available[identity];
        const bool block=identity<sample.blocked.size() && sample.blocked[identity];
        if(store->PublishSource(sample.raw,have?sample.values[identity]:VtValue(),block,false))changed->push_back(sample.raw);
        if(sample.providerRaw!=UINT64_MAX) {
            const bool providerOverride=overlay!=overlays.end();
            if(store->PublishSource(sample.providerRaw,providerOverride?overlay->second:(have?sample.values[identity]:VtValue()),providerOverride?false:block,providerOverride))changed->push_back(sample.providerRaw);
        }
        if(store->PublishSource(sample.rawDefault,sample.defaultAvailable?sample.defaultValue:VtValue(),sample.defaultBlocked,false))changed->push_back(sample.rawDefault);
        if(store->PublishSource(sample.overlay,overlay==overlays.end()?VtValue():overlay->second,false,overlay!=overlays.end()))changed->push_back(sample.overlay);
    }
    return true;
}
}
