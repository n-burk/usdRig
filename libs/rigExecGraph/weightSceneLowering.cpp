#include "weightSceneLowering.h"
#include <algorithm>
#include <set>
namespace rigExec {
namespace {
constexpr const char *ScalarNames[]={"rigExec:defaultWeight","inputs:driver","inputs:scale","inputs:bias",
    "inputs:strength","inputs:invert","inputs:falloffMin","inputs:falloffMax","inputs:scaleX",
    "inputs:scaleY","inputs:scaleZ","inputs:scaleXPos","inputs:scaleYPos","inputs:scaleZPos",
    "inputs:scaleXNeg","inputs:scaleYNeg","inputs:scaleZNeg","inputs:extentU","inputs:extentV"};
constexpr const char *PointNames[]={"rigExec:sampleSource","rigExec:weightTarget","rigExec:curve"};
bool Fail(std::string *error,const std::string &text){if(error)*error=text;return false;}
bool Used(int kind,int member) {
    return member==0 || (kind==1 && member>=1 && member<=3) ||
        (kind==2 && (member==4 || member==5)) || (kind==3 && member>=4 && member<=16) ||
        (kind==4 && ((member>=4 && member<=7) || member>=17)) || (kind==5 && member>=4 && member<=10);
}
SdfPath Canonical(const RigExecSceneCompileInputs &source,const SdfPath &target) {
    const auto *node=target.IsPrimPath()?source.Node(target):nullptr;
    return node && node->pointBased?target.AppendProperty(TfToken("points")):target;
}
}
bool RigExecLowerSceneWeight(const RigExecSceneDescriptors &scene,const SdfPath &path,
    RigExecSceneWeightProgram *output,std::string *error) {
    if(!output)return Fail(error,"null weight descriptor destination");
    if(error)error->clear();
    const RigExecSceneCompileInputs source(scene);
    RigExecSceneWeightProgram result;
    auto lower=[&](const auto &self,const SdfPath &who)->int {
        const auto seen=result.indices.find(who);
        if(seen!=result.indices.end())return seen->second;
        const auto *node=source.Node(who);
        if(!node || !node->fact.active){Fail(error,"missing weight object "+who.GetString());return -1;}
        RigExecWeightRecord record;RigExecSceneWeightObject object;
        object.path=who;object.assetAnchor=scene.rigRoot.GetParentPath();
        record.name=who.GetString();record.type=node->fact.type;
        record.kind=record.type=="RigExecStaticWeight"?0:record.type=="RigExecDynamicWeight"?1:
            record.type=="RigExecCombineWeight"?2:record.type=="RigExecSphereWeight"?3:
            record.type=="RigExecPlaneWeight"?4:record.type=="RigExecCurveWeight"?5:-1;
        if(record.kind<0){Fail(error,"unknown weight object type "+record.type.GetString()+" on "+record.name);return -1;}
        const int index=int(result.records.size());result.indices[who]=index;
        result.records.emplace_back();result.objects.emplace_back();
        bool captured=true;
        auto raw=[&](const char *name,auto fallback) {
            using T=decltype(fallback);const auto property=who.AppendProperty(TfToken(name));
            if(!source.Attribute(property))return fallback;
            RigExecSceneBoundInput binding;VtValue value;
            if(!source.Bind(property,RigExecSceneReadRoute::Raw,&binding,error) ||
               !source.Read(binding,UsdTimeCode::Default(),&value,nullptr,error)){captured=false;return fallback;}
            return value.IsHolding<T>()?value.UncheckedGet<T>():fallback;
        };
        record.representationToken=raw("rigExec:representation",TfToken("constant"));
        record.rangePolicy=raw("rigExec:rangePolicy",TfToken("strict"));
        record.combineToken=raw("rigExec:combineMode",TfToken("multiply"));
        record.representation=record.representationToken=="constant"?0:record.representationToken=="dense"?1:
            record.representationToken=="sparse"?2:-1;
        record.clamp=record.rangePolicy=="clamp";
        record.planeAxis=raw("rigExec:planeAxis",TfToken("y"));
        record.planeBounds=raw("rigExec:planeBounds",TfToken("unbounded"));
        const char *modes[]={"multiply","add","subtract","max","min","average","overlay"};
        for(int k=0;k<7;++k)if(record.combineToken==modes[k])record.combineMode=k;
        const auto values=raw("rigExec:values",VtFloatArray{});
        const auto indices=raw("rigExec:indices",VtIntArray{});
        record.values.assign(values.begin(),values.end());record.indices.assign(indices.begin(),indices.end());
        if(record.kind==0) {
            const auto valuePath=who.AppendProperty(TfToken("rigExec:values"));
            const auto indexPath=who.AppendProperty(TfToken("rigExec:indices"));
            if(source.Attribute(valuePath))object.paintedValues=valuePath;
            if(source.Attribute(indexPath))object.paintedIndices=indexPath;
        }
        const auto relation=[&](const char *name,SdfPathVector *targets,std::vector<char> *exists) {
            const auto *r=source.Relationship(who.AppendProperty(TfToken(name)));
            if(r){*targets=r->fact.targets;for(const auto &target:*targets)exists->push_back(source.Node(target)!=nullptr);}
        };
        relation("rigExec:baseWeight",&object.baseTargets,&object.baseExists);
        relation("rigExec:inputWeights",&object.inputTargets,&object.inputExists);
        if(record.kind==1 && !object.baseTargets.empty()) {
            record.base=self(self,object.baseTargets.front());if(record.base<0)return -1;
        }
        if(record.kind==2)for(const auto &target:object.inputTargets) {
            const int input=self(self,target);if(input<0)return -1;record.inputs.push_back(input);
        }
        for(int member=0;member<19;++member)if(Used(record.kind,member)) {
            const auto attribute=who.AppendProperty(TfToken(ScalarNames[member]));
            if(!source.Attribute(attribute))continue;
            RigExecSceneBoundInput binding;
            if(!source.Bind(attribute,RigExecSceneReadRoute::ConnectionResolved,&binding,error))return -1;
            object.scalars.emplace(member,std::move(binding));
            RigExecSceneTypedRead typed;
            if(!RigExecBindSceneTypedRead(scene,attribute,SdfValueTypeNames->Float,&typed,error))return -1;
            object.scalarReads.emplace(member,std::move(typed));
        }
        for(int key=0;key<3;++key) {
            auto &point=object.points[size_t(key)];point.reader=who.AppendProperty(TfToken(PointNames[key]));
            const auto *r=source.Relationship(point.reader);
            point.readPhase=r?r->readPhase:TfToken("base");
            if(!r)continue;
            point.targets=r->fact.targets;
            for(const auto &target:point.targets) {
                const auto canonical=Canonical(source,target);point.canonicalTargets.push_back(canonical);
                const auto *attribute=source.Attribute(canonical);
                const bool exists=attribute!=nullptr;point.targetExists.push_back(exists);
                point.pointArrays.push_back(attribute && attribute->fact.type.GetType().GetTypeid()==typeid(VtVec3fArray));
                RigExecSceneBoundInput binding;binding.consumer=point.reader;binding.source=canonical;
                binding.readPhase=point.readPhase;binding.route=RigExecSceneReadRoute::Raw;
                if(exists && !source.Bind(canonical,RigExecSceneReadRoute::Raw,&binding,error))return -1;
                point.raw.push_back(std::move(binding));
                if(target.IsPropertyPath() && exists) {
                    RigExecSceneTypedRead typed;
                    if(!RigExecBindSceneTypedRead(scene,target,SdfValueTypeNames->Point3fArray,&typed,error))return -1;
                    object.packetReads[size_t(key)].push_back(std::move(typed));
                }
            }
        }
        record.samplesInFlight=object.points[1].readPhase=="preceding";
        auto bindToken=[&](const char *name,RigExecSceneBoundInput *binding,bool *exists) {
            const auto property=who.AppendProperty(TfToken(name));*exists=source.Attribute(property)!=nullptr;
            return !*exists || source.Bind(property,RigExecSceneReadRoute::Raw,binding,error);
        };
        if(!bindToken("rigExec:planeAxis",&object.axis,&object.axisAttribute) ||
           !bindToken("rigExec:planeBounds",&object.bounds,&object.boundsAttribute))return -1;
        if(record.kind>=3) {
            const auto profile=raw("rigExec:falloffProfile",TfToken("smooth"));
            const auto *curve=source.Attribute(who.AppendProperty(TfToken("rigExec:falloffCurve")));
            const TsSpline *spline=curve && curve->fact.spline.IsHolding<TsSpline>()?
                &curve->fact.spline.UncheckedGet<TsSpline>():nullptr;
            record.falloffCurve=RigExecBakeWeightFalloff(profile,spline);
        }
        if(record.kind==2 && record.combineMode<0)
            record.staticError=record.name+": unknown rigExec:combineMode "+record.combineToken.GetString();
        if(record.kind<=1 && record.rangePolicy!="strict" && record.rangePolicy!="clamp")
            record.staticError="unknown rangePolicy on "+record.name;
        if(record.kind==0)for(const char *name:{"rigExec:values","rigExec:indices","rigExec:defaultWeight",
            "rigExec:representation","rigExec:rangePolicy"}) {
            const auto *a=source.Attribute(who.AppendProperty(TfToken(name)));
            if(a && (!a->fact.sampleTimes.empty() || a->fact.hasAuthoredConnections)) {
                record.staticError=std::string("static weight field ")+name+" has time samples or connections on "+record.name;break;
            }
        }
        if(record.kind==1 && record.staticError.empty()) {
            if(raw("rigExec:operation",TfToken("multiply"))!="multiply")
                record.staticError="unknown dynamic-weight operation on "+record.name;
            else if(object.baseTargets.size()>1)
                record.staticError="rigExec:baseWeight must have at most one target on "+record.name;
            else if(object.baseTargets.empty() && record.representation!=0)
                record.staticError="no-base dynamic weight must be constant on "+record.name;
        }
        if(!captured)return -1;

        result.records[size_t(index)]=std::move(record);result.objects[size_t(index)]=std::move(object);return index;
    };
    result.root=lower(lower,path);if(result.root<0)return false;
    for(size_t id=0;id<result.records.size();++id) {
        auto &record=result.records[id];const auto &object=result.objects[id];
        if(record.kind!=1 || record.base<0 || !record.staticError.empty())continue;
        const auto &base=result.objects[size_t(record.base)];const auto &baseRecord=result.records[size_t(record.base)];
        const auto &a=object.points[1].canonicalTargets;const auto &b=base.points[1].canonicalTargets;
        if(a.size()!=1 || b.size()!=1 || a.front()!=b.front())
            record.staticError="dynamic/base weight target mismatch on "+record.name;
        else if(record.representationToken!=baseRecord.representationToken)
            record.staticError="dynamic/base representation mismatch on "+record.name;
        else if(record.representation==2 && !record.indices.empty() &&
            std::set<int>(record.indices.begin(),record.indices.end())!=
            std::set<int>(baseRecord.indices.begin(),baseRecord.indices.end()))
            record.staticError="dynamic/base sparse support mismatch on "+record.name;
    }
    *output=std::move(result);return true;
}
bool RigExecResolveSceneWeight(const RigExecSceneDescriptors &scene,const RigExecSceneWeightProgram &program,
    UsdTimeCode identity,const std::map<SdfPath,VtValue> &delivered,
    const std::map<SdfPath,GfMatrix4d> &placements,RigExecSceneWeightInputs *output,std::string *error) {
    if(!output || program.objects.size()!=program.records.size())return Fail(error,"invalid weight input destination");
    if(error)error->clear();
    const RigExecSceneCompileInputs source(scene);RigExecSceneWeightInputs result;
    const size_t count=program.objects.size();result.fields.resize(count);result.rawStorage.resize(count);
    result.phasedStorage.resize(count);result.packetPoints.resize(count);
    result.paintedValues.resize(count);result.paintedIndices.resize(count);
    for(size_t id=0;id<count;++id) {
        const auto &object=program.objects[id];auto &input=result.fields[id];
        if(!object.paintedValues.IsEmpty() || !object.paintedIndices.IsEmpty()) {
            input.paintedArraysDeclared=true;
            auto readDefault=[&](const SdfPath &path,VtValue *value) {
                if(path.IsEmpty())return false;
                RigExecSceneBoundInput binding;
                return source.Bind(path,RigExecSceneReadRoute::Raw,&binding,nullptr) &&
                    source.Read(binding,UsdTimeCode::Default(),value,nullptr,nullptr);
            };
            VtValue value;
            if(readDefault(object.paintedValues,&value) && value.IsHolding<VtFloatArray>())
                result.paintedValues[id]=value.UncheckedGet<VtFloatArray>();
            value=VtValue{};
            if(readDefault(object.paintedIndices,&value) && value.IsHolding<VtIntArray>())
                result.paintedIndices[id]=value.UncheckedGet<VtIntArray>();
            const auto &values=result.paintedValues[id];const auto &indices=result.paintedIndices[id];
            input.paintedValues=values.data();input.paintedValueCount=values.size();
            input.paintedIndices=indices.data();input.paintedIndexCount=indices.size();
        }
        for(const auto &[member,binding]:object.scalarReads) {
            VtValue value;
            if(!RigExecResolveSceneTypedRead(scene,binding,identity,delivered,&value,nullptr))continue;
            if(value.IsHolding<float>())input.scalars[size_t(member)]=value.UncheckedGet<float>();
        }
        for(int key=0;key<3;++key) {
            const auto &binding=object.points[size_t(key)];const auto selected=delivered.find(binding.reader);
            auto &phase=input.phasedPoints[size_t(key)];
            phase.declared=binding.readPhase!="base";
            if(phase.declared && selected!=delivered.end() && selected->second.IsHolding<VtVec3fArray>()) {
                auto &points=result.phasedStorage[id][size_t(key)];points=selected->second.UncheckedGet<VtVec3fArray>();
                const auto &stored=points;
                phase.available=true;phase.data=stored.data();phase.count=stored.size();
            }
            for(size_t t=0;t<binding.raw.size();++t) {
                if(t>=binding.targetExists.size() || !binding.targetExists[t])continue;
                VtValue value;
                const auto current=delivered.find(binding.canonicalTargets[t]);
                if(current!=delivered.end())value=current->second;
                else if(!source.Read(binding.raw[t],identity,&value,nullptr,nullptr))continue;
                if(!value.IsHolding<VtVec3fArray>())continue;
                const auto &points=value.UncheckedGet<VtVec3fArray>();
                if(binding.targets.size()==1) {
                    result.rawStorage[id][size_t(key)]=points;
                    auto &raw=input.rawPoints[size_t(key)];const auto &stored=result.rawStorage[id][size_t(key)];
                    raw={false,true,stored.data(),stored.size()};
                }
            }
            for(const auto &read:object.packetReads[size_t(key)]) {
                VtValue value;
                if(!RigExecResolveSceneTypedRead(scene,read,identity,delivered,&value,nullptr) || !value.IsHolding<VtVec3fArray>())continue;
                const auto &points=value.UncheckedGet<VtVec3fArray>();auto &gathered=result.packetPoints[id][size_t(key)];
                gathered.insert(gathered.end(),points.begin(),points.end());
            }
        }
        auto token=[&](const RigExecSceneBoundInput &binding,bool exists,const char *fallback) {
            VtValue value;return exists && source.Read(binding,identity,&value,nullptr,nullptr) && value.IsHolding<TfToken>()?
                value.UncheckedGet<TfToken>().GetString():exists?std::string():std::string(fallback);
        };
        input.axis=token(object.axis,object.axisAttribute,"x");input.bounds=token(object.bounds,object.boundsAttribute,"unbounded");
        const auto placement=placements.find(object.path);input.hasPlacement=placement!=placements.end();
        if(input.hasPlacement)input.placement=placement->second;
    }
    *output=std::move(result);return true;
}
bool RigExecMakeSceneWeightPacketInputs(const RigExecSceneWeightProgram &program,
    const RigExecSceneWeightInputs &current,int object,const std::vector<RigExecWeightPacket> &packets,
    const std::map<SdfPath,RigExecPointFrame> &baseFrames,RigExecWeightPacketInputs *output,std::string *error) {
    if(!output || object<0 || size_t(object)>=program.records.size() || size_t(object)>=current.fields.size())
        return Fail(error,"weight packet input unavailable");
    const auto &record=program.records[size_t(object)];const auto &input=current.fields[size_t(object)];
    RigExecWeightPacketInputs result;
    result.painted={record.representationToken,record.rangePolicy,record.values.data(),record.values.size(),
        record.indices.data(),record.indices.size(),input.scalars[0]};
    if(input.paintedArraysDeclared) {
        result.painted.values=input.paintedValues;result.painted.valuesSize=input.paintedValueCount;
        result.painted.indices=input.paintedIndices;result.painted.indicesSize=input.paintedIndexCount;
    }
    result.dynamic={record.representationToken,record.rangePolicy,input.scalars[1],input.scalars[2],input.scalars[3]};
    if(record.base>=0) {
        if(size_t(record.base)>=packets.size())return Fail(error,"base weight packet unavailable");
        result.base=&packets[size_t(record.base)];
    }
    for(int id:record.inputs) {
        if(id<0 || size_t(id)>=packets.size())return Fail(error,"input weight packet unavailable");
        result.inputs.push_back(&packets[size_t(id)]);
    }
    result.strength=input.scalars[4];result.invert=input.scalars[5];
    auto &volume=result.volume;volume.representation=record.representationToken;volume.rangePolicy=record.rangePolicy;
    const auto frame=baseFrames.find(program.objects[size_t(object)].path);
    if(frame!=baseFrames.end()){volume.hasPlacement=true;volume.placement=frame->second;}
    volume.params.falloffMin=input.scalars[6];volume.params.falloffMax=input.scalars[7];
    volume.params.invert=result.invert;volume.params.strength=result.strength;volume.params.curveData=record.falloffCurve.data();volume.params.curveCount=record.falloffCurve.size();
    volume.scales=GfVec3f(input.scalars[8],input.scalars[9],input.scalars[10]);
    volume.positiveScales=GfVec3f(input.scalars[11],input.scalars[12],input.scalars[13]);
    volume.negativeScales=GfVec3f(input.scalars[14],input.scalars[15],input.scalars[16]);
    volume.extentU=input.scalars[17];volume.extentV=input.scalars[18];
    volume.planeAxis=record.planeAxis;volume.planeBounds=record.planeBounds;
    volume.usePointViews=true;
    const auto &samples=current.packetPoints[size_t(object)][0],&targets=current.packetPoints[size_t(object)][1],&curve=current.packetPoints[size_t(object)][2];
    volume.sampleView={samples.data(),samples.size()};volume.targetView={targets.data(),targets.size()};
    volume.curveView={curve.data(),curve.size()};result.targetCount=targets.size();
    *output=std::move(result);return true;
}
}
