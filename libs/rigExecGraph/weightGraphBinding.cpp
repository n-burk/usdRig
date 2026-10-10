#include "weightGraphBinding.h"
#include "sceneGraphTypedRead.h"
#include "rigExec/frameExtraction.h"
#include <algorithm>
#include <cstring>
#include <type_traits>
namespace rigExec {
namespace {
bool Fail(std::string *error,const std::string &text){if(error)*error=text;return false;}
template<class T> const T *Read(const RigExecTypedValueStore &store,RigExecValueId id) {
    if(id>=store.values.size())return nullptr;const auto &state=store.values[size_t(id)];
    if(!state.initialized || state.blocked)return nullptr;
    if constexpr(std::is_same_v<T,float> || std::is_same_v<T,double> || std::is_same_v<T,GfMatrix4d> ||
        std::is_same_v<T,TfToken> || std::is_same_v<T,RigExecPointFrame>)
        if(const auto *value=std::get_if<T>(&state.value))return value;
    const auto *boxed=std::get_if<VtValue>(&state.value);
    return boxed && boxed->IsHolding<T>()?&boxed->UncheckedGet<T>():nullptr;
}
template<class T> const T *Resolve(const RigExecTypedValueStore &store,const RigExecWeightGraphRead &read) {
    for(auto id:read.overlays)if(const auto *value=Read<T>(store,id))return value;
    for(auto id:read.required)if(!Read<T>(store,id))return nullptr;
    for(auto it=read.raw.rbegin();it!=read.raw.rend();++it)if(const auto *value=Read<T>(store,*it))return value;
    return nullptr;
}
float Scalar(const RigExecTypedValueStore &store,const RigExecWeightGraphRead &read,float fallback) {
    for(auto id:read.overlays)if(const auto *value=Read<float>(store,id))return *value;
    if(read.doubleTail) {
        for(auto id:read.doubleOverlays)if(const auto *value=Read<double>(store,id))return float(*value);
        for(auto id:read.required)if(!Read<float>(store,id))return fallback;
        for(auto id:read.doubleRequired)if(!Read<double>(store,id))return fallback;
        for(auto it=read.doubleRaw.rbegin();it!=read.doubleRaw.rend();++it)
            if(const auto *value=Read<double>(store,*it))return float(*value);
        return fallback;
    }
    const auto *value=Resolve<float>(store,read);return value?*value:fallback;
}
RigExecWeightPointInput Points(const RigExecTypedValueStore &store,RigExecValueId id,bool declared=false) {
    const auto *points=Read<VtVec3fArray>(store,id);
    return {declared,points!=nullptr,points?points->data():nullptr,points?points->size():0};
}
void Unique(std::vector<RigExecValueId> *ids) {
    ids->erase(std::remove(ids->begin(),ids->end(),UINT64_MAX),ids->end());
    std::sort(ids->begin(),ids->end());ids->erase(std::unique(ids->begin(),ids->end()),ids->end());
}
void Add(const RigExecWeightGraphRead &read,std::vector<RigExecValueId> *ids) {
    for(const auto *refs:{&read.overlays,&read.raw,&read.doubleOverlays,&read.doubleRaw,&read.required,&read.doubleRequired})
        ids->insert(ids->end(),refs->begin(),refs->end());
}
bool SameFloats(const std::vector<float> &a,const std::vector<float> &b) {
    return a.size()==b.size() && (a.empty() || !std::memcmp(a.data(),b.data(),a.size()*sizeof(float)));
}
bool SamePacket(const RigExecWeightPacket &a,const RigExecWeightPacket &b) {
    return a.valid==b.valid && a.representation==b.representation && a.rangePolicy==b.rangePolicy &&
        a.indices==b.indices && SameFloats(a.values,b.values) &&
        !std::memcmp(&a.defaultWeight,&b.defaultWeight,sizeof(float));
}
}
static bool BindWeightGraph(const RigExecSceneWeightProgram &program,const RigExecWeightValueResolver &resolve,
    const RigExecSceneGraphBindingContext *typedContext,RigExecWeightGraphBinding *output,std::string *error) {
    if(!output || !resolve || program.objects.size()!=program.records.size())return Fail(error,"invalid weight graph binding");
    RigExecWeightGraphBinding result;result.records=program.records;result.root=program.root;result.objects.resize(program.objects.size());
    const TfToken base("base");
    auto bindRead=[&](const RigExecSceneTypedRead &read,RigExecWeightGraphRead *out) {
        if(typedContext && typedContext->effectiveRead) {
            RigExecGraphTypedRead bound;
            if(!RigExecBindGraphTypedRead(read,*typedContext,&bound,error))return false;
            out->overlays={bound.effective};return true;
        }
        out->doubleTail=!read.doubleHops.empty();
        auto bind=[&](const auto &hops,auto *overlay,auto *raw,auto *required) {
            for(const auto &hop:hops) {
                RigExecValueId id=UINT64_MAX;
                if(!resolve(read.consumer,hop.consumer,RigExecWeightValueRole::Overlay,read.readPhase,&id,error))return false;
                overlay->push_back(id);
                if(std::find(read.computedSources.begin(),read.computedSources.end(),hop.consumer)!=read.computedSources.end())
                    required->push_back(id);
                id=UINT64_MAX;
                if(!resolve(read.consumer,hop.consumer,RigExecWeightValueRole::Raw,base,&id,error))return false;
                raw->push_back(id);
            }
            return true;
        };
        return bind(read.hops,&out->overlays,&out->raw,&out->required) &&
            bind(read.doubleHops,&out->doubleOverlays,&out->doubleRaw,&out->doubleRequired);
    };
    for(size_t id=0;id<program.objects.size();++id) {
        const auto &object=program.objects[id];auto &binding=result.objects[id];
        binding.paintedArraysDeclared=!object.paintedValues.IsEmpty() || !object.paintedIndices.IsEmpty();
        if(!object.paintedValues.IsEmpty() && !resolve(object.paintedValues,object.paintedValues,
            RigExecWeightValueRole::RawDefault,base,&binding.paintedValues,error))return false;
        if(!object.paintedIndices.IsEmpty() && !resolve(object.paintedIndices,object.paintedIndices,
            RigExecWeightValueRole::RawDefault,base,&binding.paintedIndices,error))return false;
        for(auto value:{binding.paintedValues,binding.paintedIndices}) {
            binding.fieldReads.push_back(value);binding.packetReads.push_back(value);
        }
        for(const auto &[member,read]:object.scalarReads) {
            RigExecWeightGraphRead bound;if(!bindRead(read,&bound))return false;
            Add(bound,&binding.fieldReads);Add(bound,&binding.packetReads);binding.scalars.emplace(member,std::move(bound));
        }
        for(size_t key=0;key<3;++key) {
            const auto &point=object.points[key];binding.phaseDeclared[key]=point.readPhase!="base";
            if(point.canonicalTargets.size()==1 && point.pointArrays.size()==1 && point.pointArrays[0]) {
                if(!resolve(point.reader,point.canonicalTargets[0],RigExecWeightValueRole::FieldPoints,base,&binding.rawPoints[key],error))return false;
                if(binding.phaseDeclared[key] && !resolve(point.reader,point.canonicalTargets[0],
                    RigExecWeightValueRole::PhasedPoints,point.readPhase,&binding.phasedPoints[key],error))return false;
            }
            binding.fieldReads.push_back(binding.rawPoints[key]);binding.fieldReads.push_back(binding.phasedPoints[key]);
            for(const auto &read:object.packetReads[key]) {
                RigExecWeightGraphRead bound;if(!bindRead(read,&bound))return false;
                Add(bound,&binding.packetReads);binding.packetPoints[key].push_back(std::move(bound));
            }
        }
        binding.axisAttribute=object.axisAttribute;binding.boundsAttribute=object.boundsAttribute;
        if(object.axisAttribute && !resolve(object.axis.consumer,object.axis.consumer,RigExecWeightValueRole::Raw,base,&binding.axis,error))return false;
        if(object.boundsAttribute && !resolve(object.bounds.consumer,object.bounds.consumer,RigExecWeightValueRole::Raw,base,&binding.bounds,error))return false;
        binding.fieldReads.push_back(binding.axis);binding.fieldReads.push_back(binding.bounds);
        if(result.records[id].kind>=3) {
            if(!resolve(object.path,object.path,RigExecWeightValueRole::Placement,TfToken("final"),&binding.placement,error) ||
               !resolve(object.path,object.path,RigExecWeightValueRole::BaseFrame,base,&binding.baseFrame,error))return false;
            binding.fieldReads.push_back(binding.placement);binding.packetReads.push_back(binding.baseFrame);
        }
        if(!resolve(object.path,object.path,RigExecWeightValueRole::Packet,base,&binding.packet,error))return false;
        const auto &record=program.records[id];
        if(record.base>=0) {
            const auto &path=program.objects[size_t(record.base)].path;
            if(!resolve(object.path,path,RigExecWeightValueRole::Packet,base,&binding.basePacket,error))return false;
            binding.packetReads.push_back(binding.basePacket);
        }
        for(int input:record.inputs) {
            RigExecValueId value=UINT64_MAX;
            if(input<0 || size_t(input)>=program.objects.size() ||
                !resolve(object.path,program.objects[size_t(input)].path,RigExecWeightValueRole::Packet,base,&value,error))return false;
            binding.inputPackets.push_back(value);binding.packetReads.push_back(value);
        }
        Unique(&binding.fieldReads);Unique(&binding.packetReads);
        result.fieldReads.insert(result.fieldReads.end(),binding.fieldReads.begin(),binding.fieldReads.end());
    }
    Unique(&result.fieldReads);
    *output=std::move(result);return true;
}
bool RigExecBindWeightGraph(const RigExecSceneWeightProgram &program,const RigExecWeightValueResolver &resolve,
    RigExecWeightGraphBinding *output,std::string *error) {
    return BindWeightGraph(program,resolve,nullptr,output,error);
}
bool RigExecBindWeightGraph(const RigExecSceneWeightProgram &program,const RigExecSceneGraphBindingContext &context,
    const SdfPath &consumer,const TfToken &placementPhase,RigExecWeightGraphBinding *output,std::string *error) {
    return BindWeightGraph(program,[&](const SdfPath &reader,const SdfPath &source,RigExecWeightValueRole role,
        const TfToken &phase,RigExecValueId *value,std::string *why) {
        if(role==RigExecWeightValueRole::Packet)return context.weightPacket && context.weightPacket(source,value,why);
        if(!context.resolve)return Fail(why,"weight graph has no typed value resolver");
        RigExecSceneGraphReadRequest request;request.consumer=reader;request.source=source;
        request.reader=consumer.IsEmpty()?reader.GetPrimPath():consumer;request.phase=phase;
        request.atDefault=role==RigExecWeightValueRole::RawDefault;
        request.raw=request.atDefault || role==RigExecWeightValueRole::Raw || role==RigExecWeightValueRole::FieldPoints;
        if(role==RigExecWeightValueRole::FieldPoints || role==RigExecWeightValueRole::PhasedPoints)
            request.domain=RigExecSceneValueDomain::Points;
        else if(role==RigExecWeightValueRole::Placement || role==RigExecWeightValueRole::BaseFrame) {
            request.domain=RigExecSceneValueDomain::Pose;
            if(role==RigExecWeightValueRole::Placement)request.phase=placementPhase;
        }
        return context.resolve(request,value,why);
    },&context,output,error);
}
void RigExecBindWeightGraphCycles(const std::vector<RigExecOpDescriptor> &ops,const RigExecCompiledGraph &graph,
    RigExecWeightGraphBinding *binding) {
    for(auto &object:binding->objects)object.cycleBlocked=false;
    for(size_t op=0;op<ops.size() && op<graph.canonicalIndex.size();++op)if(graph.canonicalIndex[op]<0)
        for(auto value:ops[op].writes)for(auto &object:binding->objects)
            if(value==object.packet)object.cycleBlocked=true;
}
void RigExecPrepareWeightGraphWorkspace(const RigExecWeightGraphBinding &binding,RigExecWeightGraphWorkspace *workspace) {
    workspace->fields.resize(binding.objects.size());workspace->packet.inputs.reserve(binding.objects.size());
    workspace->field.children.resize(binding.records.size()+1);
}
bool RigExecRunBoundWeightField(const RigExecWeightGraphBinding &binding,const RigExecTypedValueStore &store,
    size_t count,RigExecValueId entering,RigExecWeightGraphWorkspace *workspace,std::string *error) {
    if(!workspace || workspace->fields.size()!=binding.objects.size())return Fail(error,"unprepared weight field workspace");
    for(size_t id=0;id<binding.objects.size();++id) {
        const auto &bound=binding.objects[id];auto &input=workspace->fields[id];input=RigExecWeightFieldInputs{};
        input.blocked=bound.cycleBlocked;
        input.paintedArraysDeclared=bound.paintedArraysDeclared;
        if(const auto *values=Read<VtFloatArray>(store,bound.paintedValues)) {
            input.paintedValues=values->data();input.paintedValueCount=values->size();
        }
        if(const auto *indices=Read<VtIntArray>(store,bound.paintedIndices)) {
            input.paintedIndices=indices->data();input.paintedIndexCount=indices->size();
        }
        for(const auto &[member,read]:bound.scalars)input.scalars[size_t(member)]=Scalar(store,read,input.scalars[size_t(member)]);
        for(size_t k=0;k<3;++k){input.rawPoints[k]=Points(store,bound.rawPoints[k]);input.phasedPoints[k]=Points(store,bound.phasedPoints[k],bound.phaseDeclared[k]);}
        const auto *axis=Read<TfToken>(store,bound.axis),*bounds=Read<TfToken>(store,bound.bounds);
        input.axis=axis?axis->GetString():bound.axisAttribute?std::string():"x";
        input.bounds=bounds?bounds->GetString():bound.boundsAttribute?std::string():"unbounded";
        if(const auto *matrix=Read<GfMatrix4d>(store,bound.placement)){input.hasPlacement=true;input.placement=*matrix;}
        else if(const auto *frame=Read<RigExecPointFrame>(store,bound.placement))
            if(frame->IsValid() && !frame->IsDegenerate()){input.hasPlacement=true;input.placement=RigExecVolumePlacement(*frame);}
    }
    RigExecWeightPointView current;const RigExecWeightPointView *points=nullptr;
    if(entering!=UINT64_MAX) {
        const auto *array=Read<VtVec3fArray>(store,entering);
        if(!array){workspace->result.clear();return Fail(error,"weight input unavailable");}
        current={array->data(),array->size()};points=&current;
    }
    return RigExecRunWeightField(binding.records,binding.root,workspace->fields,count,points,&workspace->field,&workspace->result,error);
}
bool RigExecRunBoundWeightPacket(const RigExecWeightGraphBinding &binding,int id,const RigExecTypedValueStore &store,
    RigExecWeightGraphWorkspace *workspace,RigExecWeightPacket *output,std::string *error) {
    if(!workspace || !output || id<0 || size_t(id)>=binding.objects.size())return Fail(error,"weight packet input unavailable");
    const auto &bound=binding.objects[size_t(id)];const auto &record=binding.records[size_t(id)];
    RigExecWeightFieldInputs current;
    for(const auto &[member,read]:bound.scalars)current.scalars[size_t(member)]=Scalar(store,read,current.scalars[size_t(member)]);
    auto &input=workspace->packet;input.inputs.clear();input.base=nullptr;
    input.workspace=&workspace->packetScratch;input.borrowedInputs=nullptr;
    input.painted={record.representationToken,record.rangePolicy,record.values.data(),record.values.size(),record.indices.data(),record.indices.size(),current.scalars[0]};
    if(bound.paintedArraysDeclared) {
        const auto *values=Read<VtFloatArray>(store,bound.paintedValues);
        const auto *indices=Read<VtIntArray>(store,bound.paintedIndices);
        input.painted.values=values?values->data():nullptr;input.painted.valuesSize=values?values->size():0;
        input.painted.indices=indices?indices->data():nullptr;input.painted.indicesSize=indices?indices->size():0;
    }
    input.dynamic={record.representationToken,record.rangePolicy,current.scalars[1],current.scalars[2],current.scalars[3]};
    if(record.base>=0){input.base=Read<RigExecWeightPacket>(store,bound.basePacket);if(!input.base)input.base=&workspace->unavailablePacket;}
    for(auto value:bound.inputPackets){const auto *packet=Read<RigExecWeightPacket>(store,value);input.inputs.push_back(packet?packet:&workspace->unavailablePacket);}
    input.strength=current.scalars[4];input.invert=current.scalars[5];auto &volume=input.volume;
    volume.representation=record.representationToken;volume.rangePolicy=record.rangePolicy;volume.hasPlacement=false;
    if(const auto *frame=Read<RigExecPointFrame>(store,bound.baseFrame)){volume.placement=*frame;volume.hasPlacement=true;}
    else if(const auto *matrix=Read<GfMatrix4d>(store,bound.baseFrame)){volume.placement=RigExecFrameFromMatrix(*matrix);volume.hasPlacement=true;}
    volume.params.falloffMin=current.scalars[6];volume.params.falloffMax=current.scalars[7];volume.params.invert=input.invert;
    volume.params.strength=input.strength;volume.params.curveData=record.falloffCurve.data();volume.params.curveCount=record.falloffCurve.size();
    volume.scales=GfVec3f(current.scalars[8],current.scalars[9],current.scalars[10]);
    volume.positiveScales=GfVec3f(current.scalars[11],current.scalars[12],current.scalars[13]);
    volume.negativeScales=GfVec3f(current.scalars[14],current.scalars[15],current.scalars[16]);
    volume.extentU=current.scalars[17];volume.extentV=current.scalars[18];volume.planeAxis=record.planeAxis;volume.planeBounds=record.planeBounds;
    std::vector<GfVec3f> *arrays[]={&volume.samplePoints,&volume.targetPoints,&volume.curvePoints};
    RigExecWeightPointView *views[]={&volume.sampleView,&volume.targetView,&volume.curveView};
    volume.usePointViews=true;volume.localCurveScratch=&workspace->localCurve;
    for(size_t k=0;k<3;++k) {
        arrays[k]->clear();*views[k]={};
        if(bound.packetPoints[k].size()==1) {
            if(const auto *array=Resolve<VtVec3fArray>(store,bound.packetPoints[k][0]))
                *views[k]={array->data(),array->size()};
        } else {
            for(const auto &read:bound.packetPoints[k])
                if(const auto *array=Resolve<VtVec3fArray>(store,read))
                    arrays[k]->insert(arrays[k]->end(),array->begin(),array->end());
            *views[k]={arrays[k]->data(),arrays[k]->size()};
        }
    }
    volume.targetPointCount=0;
    input.targetCount=volume.targetView.count;*output=RigExecRunWeightPacket(record,input);return true;
}
bool RigExecPublishWeightPacket(RigExecTypedValueStore *store,RigExecValueId id,const RigExecWeightPacket &packet) {
    auto &state=store->values.at(size_t(id));const auto *boxed=std::get_if<VtValue>(&state.value);
    const bool same=boxed && boxed->IsHolding<RigExecWeightPacket>() && SamePacket(boxed->UncheckedGet<RigExecWeightPacket>(),packet);
    state.changed=!state.initialized || !same || state.blocked || state.authoritative || state.count!=packet.values.size() || !state.error.empty();
    if(state.changed){state.value=VtValue(packet);++state.revision;}state.initialized=true;state.blocked=false;state.authoritative=false;
    state.raw=std::get<VtValue>(state.value);
    state.count=packet.values.size();state.error.clear();return state.changed;
}
bool RigExecPublishWeightField(RigExecTypedValueStore *store,RigExecValueId id,const std::vector<float> &values,
    size_t count,bool valid,const std::string &error) {
    const std::vector<float> empty;const auto &data=valid?values:empty;
    auto &state=store->values.at(size_t(id));const auto *boxed=std::get_if<VtValue>(&state.value);
    bool same=false;
    if(boxed && boxed->IsHolding<VtFloatArray>()) {
        const auto &old=boxed->UncheckedGet<VtFloatArray>();same=old.size()==data.size() &&
            (old.empty() || !std::memcmp(old.data(),data.data(),old.size()*sizeof(float)));
    }
    state.changed=!state.initialized || !same || state.blocked!=!valid || state.authoritative || state.count!=count || state.error!=error;
    if(state.changed){state.value=VtValue(VtFloatArray(data.begin(),data.end()));++state.revision;}
    state.initialized=true;state.blocked=!valid;state.authoritative=false;state.count=count;state.error=error;
    state.raw=std::get<VtValue>(state.value);return state.changed;
}
}
