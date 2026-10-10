#include "propertyGraphBinding.h"
#include <algorithm>
namespace rigExec {
bool RigExecBindPropertyGraph(const RigExecScenePropertyDescriptor &descriptor,
    const RigExecSceneGraphBindingContext &context,RigExecValueId incoming,RigExecValueId output,
    RigExecValueId envelope,RigExecPropertyGraphBinding *out,std::string *error) {
    if(!out || incoming==UINT64_MAX || output==UINT64_MAX){if(error)*error="missing property revision slots";return false;}
    RigExecPropertyGraphBinding binding;binding.record=descriptor.record;
    binding.incoming=incoming;binding.output=output;binding.envelope=envelope;binding.reads.push_back(incoming);
    if(envelope!=UINT64_MAX)binding.reads.push_back(envelope);
    for(const auto &[name,source]:descriptor.typedReads) {
        RigExecGraphTypedRead read;
        if(!RigExecBindGraphTypedRead(source,context,&read,error))return false;
        auto append=[&](const auto &hops){for(const auto &hop:hops)for(const auto id:{hop.raw,hop.overlay})if(id!=UINT64_MAX)binding.reads.push_back(id);};
        append(read.hops);append(read.doubleHops);binding.inputs.emplace(name,std::move(read));
    }
    std::sort(binding.reads.begin(),binding.reads.end());binding.reads.erase(std::unique(binding.reads.begin(),binding.reads.end()),binding.reads.end());
    *out=std::move(binding);return true;
}
bool RigExecRunBoundProperty(const RigExecPropertyGraphBinding &binding,RigExecTypedValueStore *store,std::string *error) {
    if(!store)return false;
    auto input=[&](const char *name,auto fallback){
        auto value=fallback;const auto found=binding.inputs.find(name);VtValue boxed;
        if(found!=binding.inputs.end() && RigExecReadGraphTypedRead(found->second,*store,&boxed,nullptr) && boxed.IsHolding<decltype(value)>())value=boxed.UncheckedGet<decltype(value)>();
        return value;
    };
    VtValue base;if(!RigExecReadSceneGraphValue(*store,binding.incoming,&base)) {
        store->PublishSource(binding.output,VtValue(),true,true);return true;
    }
    if(!input("inputs:enabled",true)){store->Copy(binding.output,binding.incoming,true);return true;}
    float envelope=input("inputs:defaultWeight",1.0f);
    if(binding.envelope!=UINT64_MAX) {
        VtValue boxed;
        if(!RigExecReadSceneGraphValue(*store,binding.envelope,&boxed) ||
            (!boxed.IsHolding<float>() && (!boxed.IsHolding<VtFloatArray>() || boxed.UncheckedGet<VtFloatArray>().size()!=1))) {
            store->Copy(binding.output,binding.incoming,true);return true;
        }
        envelope=boxed.IsHolding<float>()?boxed.UncheckedGet<float>():boxed.UncheckedGet<VtFloatArray>()[0];
    }
    VtValue result=base;
    if(const auto *original=std::get_if<RigExecFloatPropertyRecord>(&binding.record)) {
        auto record=*original;record.value=input("inputs:value",0.0f);record.minimum=input("inputs:min",0.0f);record.maximum=input("inputs:max",0.0f);
        const auto keys=input("inputs:keys",VtVec2fArray{}),tangents=input("inputs:tangents",VtVec2fArray{});
        record.keyData=keys.cdata();record.keyCount=keys.size();record.tangentData=tangents.cdata();record.tangentCount=tangents.size();
        if(base.IsHolding<float>()){auto value=base.UncheckedGet<float>();RigExecRunProperty(record,value,envelope,&value);result=VtValue(value);}
        else if(base.IsHolding<double>()){auto value=base.UncheckedGet<double>();RigExecRunProperty(record,value,envelope,&value);result=VtValue(value);}
    } else if(const auto *original=std::get_if<RigExecVec3PropertyRecord>(&binding.record)) {
        auto record=*original;record.value=input("inputs:value",GfVec3f(0));record.minimum=input("inputs:min",GfVec3f(0));record.maximum=input("inputs:max",GfVec3f(0));
        if(base.IsHolding<GfVec3f>()){auto value=base.UncheckedGet<GfVec3f>();RigExecRunProperty(record,value,envelope,&value);result=VtValue(value);}
    } else {
        auto record=std::get<RigExecMatrixPropertyRecord>(binding.record);record.value=input("inputs:value",GfMatrix4d(1.0));
        if(base.IsHolding<GfMatrix4d>()){auto value=base.UncheckedGet<GfMatrix4d>();RigExecRunProperty(record,value,envelope,&value);result=VtValue(value);}
    }
    store->PublishSource(binding.output,result,false,true);return true;
}
}
