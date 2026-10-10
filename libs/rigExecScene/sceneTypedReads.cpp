#include "sceneTypedReads.h"
#include "pxr/base/tf/type.h"
#include <algorithm>
#include <set>
namespace rigExec {
namespace {
bool Fail(std::string *error,const std::string &text){if(error)*error=text;return false;}
bool Match(const VtValue &value,const std::type_index &type) {
    return !value.IsEmpty() && std::type_index(value.GetTypeid())==type;
}
}
bool RigExecBindSceneTypedRead(const RigExecSceneDescriptors &scene,const SdfPath &path,
    const SdfValueTypeName &requested,RigExecSceneTypedRead *output,std::string *error) {
    if(!output)return Fail(error,"null typed input destination");
    const RigExecSceneCompileInputs source(scene);const auto *head=source.Attribute(path);
    RigExecSceneTypedRead read;read.consumer=path;read.requested=requested;
    read.cppType=std::type_index(requested.GetType().GetTypeid());
    read.readPhase=head?head->readPhase:TfToken("base");
    if(head) {
        const auto element=head->fact.metadata.find("rigExecInputElement");
        if(element!=head->fact.metadata.end()) {
            if(read.cppType!=std::type_index(typeid(GfVec3f)) || !element->second.IsHolding<int>() ||
                element->second.UncheckedGet<int>()<0)
                return Fail(error,path.GetString()+": rigExecInputElement requires a nonnegative int on a Vec3f input");
            read.inputElement=element->second.UncheckedGet<int>();
        }
    }
    std::set<SdfPath> visited;SdfPath current=path;
    bool asDouble=requested==SdfValueTypeNames->Float && head && head->fact.type==SdfValueTypeNames->Double;
    while(!current.IsEmpty() && visited.insert(current).second) {
        const auto *attribute=source.Attribute(current);if(!attribute)break;
        const auto *prim=source.Node(current.GetPrimPath());if(!prim || !prim->fact.active)break;
        if(read.inputElement>=0 && attribute->fact.type.GetType().GetTypeid()==typeid(VtVec3fArray))
            read.elementSources.push_back(current);
        for(const auto &input:attribute->inputs)
            if(input.state==RigExecSceneInputState::Computed && input.source==current) {
                read.computedSources.push_back(current);break;
            }
        RigExecSceneBoundInput raw;
        if(!source.Bind(current,RigExecSceneReadRoute::Raw,&raw,error))return false;
        if(!asDouble)read.hops.push_back(raw);
        if(requested==SdfValueTypeNames->Float && attribute->fact.type==SdfValueTypeNames->Double)asDouble=true;
        if(asDouble)read.doubleHops.push_back(std::move(raw));
        if(attribute->fact.connections.size()!=1)break;
        current=attribute->fact.connections.front();
    }
    *output=std::move(read);return true;
}
bool RigExecResolveSceneTypedRead(const RigExecSceneDescriptors &scene,const RigExecSceneTypedRead &read,
    UsdTimeCode identity,const std::map<SdfPath,VtValue> &delivered,VtValue *output,std::string *error) {
    if(!output)return Fail(error,"null typed input read destination");*output=VtValue();
    if(error)error->clear();
    const RigExecSceneCompileInputs source(scene);
    bool unavailableElement=false;
    auto select=[&](const VtValue &value,VtValue *selected) {
        if(read.inputElement<0 || !value.IsHolding<VtVec3fArray>())return false;
        const auto &points=value.UncheckedGet<VtVec3fArray>();
        if(size_t(read.inputElement)>=points.size()){unavailableElement=true;return false;}
        *selected=VtValue(points[size_t(read.inputElement)]);return true;
    };
    auto overlays=[&](const auto &hops,const std::type_index &type,VtValue *value) {
        for(const auto &hop:hops) {
            const auto found=delivered.find(hop.consumer);
            if(found!=delivered.end() && Match(found->second,type)){*value=found->second;return true;}
            if(found!=delivered.end() && type==read.cppType &&
                std::find(read.elementSources.begin(),read.elementSources.end(),hop.consumer)!=read.elementSources.end() &&
                select(found->second,value))return true;
        }
        return false;
    };
    if(overlays(read.hops,read.cppType,output))return true;
    if(read.inputElement>=0 && !read.elementSources.empty() && read.readPhase!="base")unavailableElement=true;
    for(const auto &path:read.computedSources) {
        const auto value=delivered.find(path);
        const auto required=read.doubleHops.empty()?read.cppType:std::type_index(typeid(double));
        const bool selectedArray=value!=delivered.end() && read.doubleHops.empty() && read.inputElement>=0 &&
            std::find(read.elementSources.begin(),read.elementSources.end(),path)!=read.elementSources.end() &&
            value->second.IsHolding<VtVec3fArray>();
        if(value==delivered.end() || (!selectedArray && !Match(value->second,required)))
            return Fail(error,"computed input requires graph producer: "+path.GetString());
    }
    const bool doubleTail=!read.doubleHops.empty();
    const auto &hops=doubleTail?read.doubleHops:read.hops;
    const auto type=doubleTail?std::type_index(typeid(double)):read.cppType;VtValue value;
    bool have=doubleTail && overlays(hops,type,&value);
    if(!have)for(auto it=hops.rbegin();it!=hops.rend();++it) {
        if(!source.Read(*it,identity,&value,nullptr,nullptr))continue;
        if(Match(value,type)){have=true;break;}
        if(!doubleTail && read.readPhase=="base" && !unavailableElement &&
            std::find(read.elementSources.begin(),read.elementSources.end(),it->consumer)!=read.elementSources.end() &&
            select(value,&value)){have=true;break;}
    }
    if(!have)return Fail(error,"typed input unavailable: "+read.consumer.GetString());
    *output=doubleTail?VtValue(float(value.UncheckedGet<double>())):std::move(value);
    if(unavailableElement && error)*error="diag "+read.consumer.GetString()+
        ": cross-domain input "+(read.elementSources.empty()?std::string():read.elementSources.front().GetString())+
        " element "+std::to_string(read.inputElement)+" is unavailable; using the typed fallback";
    return true;
}
}
