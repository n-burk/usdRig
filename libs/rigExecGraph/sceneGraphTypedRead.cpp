#include "sceneGraphTypedRead.h"
#include <algorithm>
namespace rigExec {
namespace {
bool Fail(std::string *error,const std::string &message){if(error)*error=message;return false;}
}
bool RigExecBindGraphTypedRead(const RigExecSceneTypedRead &source,
    const RigExecSceneGraphBindingContext &context,RigExecGraphTypedRead *out,std::string *error) {
    if(!out || !context.resolve)return Fail(error,"missing typed graph linker");
    RigExecGraphTypedRead read;read.type=source.cppType;read.element=source.inputElement;
    read.base=source.readPhase=="base";read.unavailable="typed input unavailable: "+source.consumer.GetString();
    read.elementUnavailable="diag "+source.consumer.GetString()+": selected array element is unavailable; using the typed fallback";
    auto bind=[&](const auto &inputs,auto *hops) {
        for(const auto &input:inputs) {
            RigExecGraphTypedHop hop;
            hop.element=std::find(source.elementSources.begin(),source.elementSources.end(),input.consumer)!=source.elementSources.end();
            hop.computed=std::find(source.computedSources.begin(),source.computedSources.end(),input.consumer)!=source.computedSources.end();
            RigExecSceneGraphReadRequest request;
            request.consumer=source.consumer;request.source=input.consumer;request.reader=source.consumer;
            request.domain=input.consumer.GetNameToken()=="points"?RigExecSceneValueDomain::Points:RigExecSceneValueDomain::Property;
            request.phase=source.readPhase;request.raw=true;
            if(!context.resolve(request,&hop.raw,error))return false;
            request.raw=false;
            // An absent overlay is optional for ordinary authored raw leaves.
            // Computed inputs must have an actual producer in the shared graph.
            std::string diagnostic;
            if(!context.resolve(request,&hop.overlay,&diagnostic) && hop.computed)
                return Fail(error,diagnostic);
            hops->push_back(hop);
        }
        return true;
    };
    if(!bind(source.hops,&read.hops) || !bind(source.doubleHops,&read.doubleHops))return false;
    if(context.effectiveRead) {
        if(!context.effectiveRead(read,&read.effective,error))return false;
        read.hops={{read.effective,read.effective,false,true}};read.doubleHops.clear();
    }
    *out=std::move(read);return true;
}
bool RigExecReadGraphTypedRead(const RigExecGraphTypedRead &read,
    const RigExecTypedValueStore &store,VtValue *out,std::string *error) {
    if(!out)return Fail(error,"null typed graph output");*out=VtValue();if(error)error->clear();
    if(read.effective!=UINT64_MAX) {
        if(read.effective>=store.values.size())return Fail(error,read.unavailable);
        const auto &state=store.values[size_t(read.effective)];
        if(error)*error=state.error;
        return RigExecReadSceneGraphValue(store,read.effective,out);
    }
    bool shortElement=false;
    auto match=[&](const VtValue &value,std::type_index type,VtValue *result,bool element) {
        if(!value.IsEmpty() && std::type_index(value.GetTypeid())==type){*result=value;return true;}
        if(element && read.element>=0 && value.IsHolding<VtVec3fArray>()) {
            const auto &array=value.UncheckedGet<VtVec3fArray>();
            if(size_t(read.element)>=array.size()){shortElement=true;return false;}
            *result=VtValue(array[size_t(read.element)]);return true;
        }
        return false;
    };
    auto overlays=[&](const auto &hops,std::type_index type,VtValue *result) {
        for(const auto &hop:hops) {
            VtValue value;
            if(hop.overlay>=store.values.size() || (!hop.computed && !store.values[size_t(hop.overlay)].authoritative))continue;
            if(RigExecReadSceneGraphValue(store,hop.overlay,&value,true) &&
                match(value,type,result,type==read.type && hop.element))return true;
        }
        return false;
    };
    if(overlays(read.hops,read.type,out))return true;
    if(read.element>=0 && !read.base)shortElement=true;
    for(const auto &hop:read.hops)if(hop.computed) {
        VtValue value;
        if(!RigExecReadSceneGraphValue(store,hop.overlay,&value,true))
            return Fail(error,read.unavailable);
        const bool selectedArray=read.doubleHops.empty() && hop.element && read.element>=0 &&
            value.IsHolding<VtVec3fArray>();
        if(!selectedArray && std::type_index(value.GetTypeid())!=
            (read.doubleHops.empty()?read.type:std::type_index(typeid(double))))
            return Fail(error,read.unavailable);
    }
    bool narrow=!read.doubleHops.empty();const auto &hops=narrow?read.doubleHops:read.hops;
    const auto type=narrow?std::type_index(typeid(double)):read.type;VtValue value;
    bool have=narrow && overlays(hops,type,&value);
    if(!have)for(auto it=hops.rbegin();it!=hops.rend();++it) {
        VtValue raw;
        if(RigExecReadSceneGraphValue(store,it->raw,&raw) &&
            match(raw,type,&value,!narrow && read.base && !shortElement && it->element)){have=true;break;}
    }
    if(!have)return Fail(error,read.unavailable);
    *out=narrow?VtValue(float(value.UncheckedGet<double>())):std::move(value);
    if(shortElement && error)*error=read.elementUnavailable;
    return true;
}
}
