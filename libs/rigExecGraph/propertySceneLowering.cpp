#include "propertySceneLowering.h"
#include "pxr/usd/sdf/types.h"
#include "pxr/base/vt/types.h"
#include "pxr/base/tf/type.h"
#include <type_traits>
namespace rigExec {
namespace {
bool Fail(std::string *error,const std::string &message) { if(error)*error=message;return false; }
}
bool RigExecLowerSceneProperty(const RigExecSceneDescriptors &scene,const SdfPath &mover,
    const SdfPath &target,RigExecScenePropertyDescriptor *output,std::string *error) {
    if(!output)return Fail(error,"null property descriptor destination");
    const RigExecSceneCompileInputs source(scene);
    const auto *node=source.Node(mover);const auto *attribute=source.Attribute(target);
    if(!node || !node->fact.active || node->domain!=RigExecSceneDomain::PropertyMover || !attribute)
        return Fail(error,"missing property mover/target: "+mover.GetString()+" -> "+target.GetString());
    RigExecScenePropertyDescriptor result;
    result.mover=mover;result.target=target;result.targetType=attribute->fact.type;
    const auto type=node->fact.type;
    const bool scalar=result.targetType==SdfValueTypeNames->Float || result.targetType==SdfValueTypeNames->Double;
    if(type=="RigExecFloatMathMover" && scalar)result.record=RigExecFloatPropertyRecord{};
    else if(type=="RigExecVec3fMathMover" && result.targetType.GetType()==TfType::Find<GfVec3f>())
        result.record=RigExecVec3PropertyRecord{};
    else if(type=="RigExecMatrixMathMover" && result.targetType==SdfValueTypeNames->Matrix4d)
        result.record=RigExecMatrixPropertyRecord{};
    else return Fail(error,"property mover/target type mismatch: "+target.GetString());
    TfToken operation;
    RigExecSceneBoundInput operationRead;
    const auto operationPath=mover.AppendProperty(TfToken("rigExec:operation"));
    if(source.Attribute(operationPath)) {
        VtValue raw;
        if(!source.Bind(operationPath,RigExecSceneReadRoute::Raw,&operationRead,error) ||
           !source.Read(operationRead,UsdTimeCode::Default(),&raw,nullptr,error))return false;
        if(raw.IsHolding<TfToken>())operation=raw.UncheckedGet<TfToken>();
    }
    RigExecPropertyOp op=RigExecPropertyOp::Add;
    const bool valid=RigExecParsePropertyOp(operation,&op);
    std::visit([&](auto &record) { record.op=op;record.opValid=valid; },result.record);
    // Existing property movers read the first direct authored weight target,
    // rather than forwarding a relationship-property target.
    if(const auto *weight=source.Relationship(mover.AppendProperty(TfToken("rigExec:weightObject"))))
        if(!weight->fact.targets.empty())result.weightObject=weight->fact.targets.front();
    auto bind=[&](const char *name) {
        const auto path=mover.AppendProperty(TfToken(name));
        if(!source.Attribute(path))return true;
        RigExecSceneBoundInput input;
        if(!source.Bind(path,RigExecSceneReadRoute::ConnectionResolved,&input,error))return false;
        SdfValueTypeName requested;
        const std::string field(name);
        if(field=="inputs:enabled")requested=SdfValueTypeNames->Bool;
        else if(field=="inputs:defaultWeight")requested=SdfValueTypeNames->Float;
        else if(field=="inputs:keys" || field=="inputs:tangents")requested=SdfValueTypeNames->Float2Array;
        else requested=scalar?SdfValueTypeNames->Float:
            std::holds_alternative<RigExecMatrixPropertyRecord>(result.record)?SdfValueTypeNames->Matrix4d:SdfValueTypeNames->Float3;
        RigExecSceneTypedRead typed;
        if(!RigExecBindSceneTypedRead(scene,path,requested,&typed,error))return false;
        result.typedReads.emplace(name,std::move(typed));
        result.inputs.emplace(name,std::move(input));return true;
    };
    if(!bind("inputs:enabled") || !bind("inputs:defaultWeight") || !bind("inputs:value"))return false;
    if(!std::holds_alternative<RigExecMatrixPropertyRecord>(result.record))
        if(!bind("inputs:min") || !bind("inputs:max"))return false;
    if(scalar && valid && op==RigExecPropertyOp::Curve) {
        if(!bind("inputs:keys") || !bind("inputs:tangents"))return false;
        std::get<RigExecFloatPropertyRecord>(result.record).hasTangents=
            source.Attribute(mover.AppendProperty(TfToken("inputs:tangents")))!=nullptr;
    }
    *output=std::move(result);return true;
}
bool RigExecResolveSceneProperty(const RigExecSceneDescriptors &scene,
    RigExecScenePropertyDescriptor *descriptor,UsdTimeCode identity,
    const std::map<SdfPath,VtValue> &delivered,bool *enabled,float *weight,std::string *error) {
    if(!descriptor || !enabled || !weight)return Fail(error,"null property input destination");
    const RigExecSceneCompileInputs source(scene);
    auto read=[&](const char *name,auto fallback,auto *value) {
        *value=fallback;const auto found=descriptor->inputs.find(name);
        if(found==descriptor->inputs.end())return true;
        const auto &binding=found->second;
        VtValue boxed;
        const auto typed=descriptor->typedReads.find(name);
        if(typed!=descriptor->typedReads.end()) {
            if(binding.readPhase!="base" && binding.source!=binding.consumer && !delivered.count(binding.consumer))
                return Fail(error,"phased property input producer unavailable: "+binding.consumer.GetString());
            if(!RigExecResolveSceneTypedRead(scene,typed->second,identity,delivered,&boxed,nullptr))return true;
        } else if(!source.Read(binding,identity,&boxed,nullptr,error))return false;
        if(boxed.IsEmpty())return true;
        using T=std::decay_t<decltype(*value)>;
        if(!boxed.IsHolding<T>())return Fail(error,"property input type mismatch: "+binding.consumer.GetString());
        *value=boxed.UncheckedGet<T>();return true;
    };
    if(!read("inputs:enabled",true,enabled) || !read("inputs:defaultWeight",1.0f,weight))return false;
    if(auto *record=std::get_if<RigExecFloatPropertyRecord>(&descriptor->record)) {
        if(!read("inputs:value",0.0f,&record->value) || !read("inputs:min",0.0f,&record->minimum) ||
           !read("inputs:max",0.0f,&record->maximum))return false;
        if(record->opValid && record->op==RigExecPropertyOp::Curve) {
            VtVec2fArray keys,tangents;
            if(!read("inputs:keys",VtVec2fArray{},&keys) || !read("inputs:tangents",VtVec2fArray{},&tangents))return false;
            record->keys.assign(keys.begin(),keys.end());record->tangents.assign(tangents.begin(),tangents.end());
        }
        return true;
    }
    if(auto *record=std::get_if<RigExecVec3PropertyRecord>(&descriptor->record))
        return read("inputs:value",GfVec3f(0),&record->value) && read("inputs:min",GfVec3f(0),&record->minimum) &&
            read("inputs:max",GfVec3f(0),&record->maximum);
    auto &record=std::get<RigExecMatrixPropertyRecord>(descriptor->record);
    return read("inputs:value",GfMatrix4d(1.0),&record.value);
}
}
