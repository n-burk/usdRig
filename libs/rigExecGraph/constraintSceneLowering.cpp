#include "constraintSceneLowering.h"
#include "pxr/base/vt/types.h"
#include "pxr/base/gf/rotation.h"
#include "pxr/base/gf/rotation.h"
#include <type_traits>
#include <algorithm>
#include <cmath>
#include "rigExec/frameExtraction.h"
namespace rigExec {
namespace {
bool Fail(std::string *error,const std::string &message) { if(error)*error=message;return false; }
RigExecEulerOrder Order(const TfToken &token) {
    if(token=="XZY")return RigExecEulerOrder::XZY;
    if(token=="YXZ")return RigExecEulerOrder::YXZ;
    if(token=="YZX")return RigExecEulerOrder::YZX;
    if(token=="ZXY")return RigExecEulerOrder::ZXY;
    if(token=="ZYX")return RigExecEulerOrder::ZYX;
    return RigExecEulerOrder::XYZ;
}
}
bool RigExecLowerSceneConstraint(const RigExecSceneDescriptors &scene,const SdfPath &path,
    const SdfPathVector &targets,RigExecSceneConstraintDescriptor *output,std::string *error) {
    if(!output)return Fail(error,"null constraint descriptor destination");
    const RigExecSceneCompileInputs source(scene);const auto *node=source.Node(path);
    if(!node || !node->fact.active || node->domain!=RigExecSceneDomain::Constraint)
        return Fail(error,"missing active constraint: "+path.GetString());
    RigExecSceneConstraintDescriptor result;result.path=path;result.targets=targets;
    const auto type=node->fact.type;
    if(type=="RigExecPositionConstraint")result.record.kind=RigExecConstraintKind::Position;
    else if(type=="RigExecRotationConstraint")result.record.kind=RigExecConstraintKind::Rotation;
    else if(type=="RigExecScaleConstraint")result.record.kind=RigExecConstraintKind::Scale;
    else if(type=="RigExecParentConstraint")result.record.kind=RigExecConstraintKind::Parent;
    else if(type=="RigExecAimConstraint")result.record.kind=RigExecConstraintKind::Aim;
    else if(type=="RigExecSingleChainIkConstraint")result.record.kind=RigExecConstraintKind::SingleChainIk;
    else return Fail(error,"unsupported constraint: "+type.GetString());
    auto relation=[&](const char *name) { return source.Targets(path.AppendProperty(TfToken(name))); };
    auto first=[&](const char *name) { const auto rows=relation(name);return rows.empty()?SdfPath{}:rows.front(); };
    auto raw=[&](const char *name,auto fallback,auto *value) {
        *value=fallback;const auto property=path.AppendProperty(TfToken(name));
        if(!source.Attribute(property))return true;
        RigExecSceneBoundInput bound;VtValue boxed;
        if(!source.Bind(property,RigExecSceneReadRoute::Raw,&bound,error) ||
           !source.Read(bound,UsdTimeCode::Default(),&boxed,nullptr,error))return false;
        using T=std::decay_t<decltype(*value)>;
        if(boxed.IsHolding<T>())*value=boxed.UncheckedGet<T>();return true;
    };
    auto bind=[&](const char *name,RigExecSceneReadRoute route=RigExecSceneReadRoute::ConnectionResolved) {
        const auto property=path.AppendProperty(TfToken(name));if(!source.Attribute(property))return true;
        RigExecSceneBoundInput input;if(!source.Bind(property,route,&input,error))return false;
        if(route!=RigExecSceneReadRoute::Raw) {
            const std::string field(name);
            const auto requested=field=="inputs:enabled" || field.rfind("inputs:affect",0)==0?SdfValueTypeNames->Bool:
                field=="inputs:defaultWeight"?SdfValueTypeNames->Float:
                field=="inputs:twistDegrees"?SdfValueTypeNames->Double:SdfValueTypeNames->Double3;
            RigExecSceneTypedRead typed;
            if(!RigExecBindSceneTypedRead(scene,property,requested,&typed,error))return false;
            result.typedReads.emplace(name,std::move(typed));
        }
        result.inputs.emplace(name,std::move(input));return true;
    };
    if(!bind("inputs:enabled") || !bind("inputs:defaultWeight") ||
       !bind("inputs:sourceWeights",RigExecSceneReadRoute::Raw))return false;
    result.sources=relation("rigExec:sources");result.space=first("rigExec:space");
    TfToken token;if(!raw("rigExec:rotationOrder",TfToken("XYZ"),&token))return false;
    result.order=Order(token);
    if(!raw("rigExec:blendShear",false,&result.blendShear) ||
       !raw("rigExec:worldUpRotationOnly",false,&result.worldUpRotationOnly))return false;
    if(result.record.kind==RigExecConstraintKind::SingleChainIk) {
        result.effector=first("rigExec:effector");result.poleObjects=relation("rigExec:poleVectorObjects");
        if(!raw("rigExec:solverMode",TfToken("rotatePlane"),&token))return false;
        result.record.singleChain.mode=token=="singleChain"?RigExecSingleChainIkMode::SingleChain:RigExecSingleChainIkMode::RotatePlane;
        if(!raw("rigExec:orientationMode",TfToken("aimX"),&token))return false;
        result.record.singleChain.preserveJointOrientation=token=="preserve";
        if(!raw("rigExec:poleVectorMode",TfToken("vector"),&token))return false;
        result.poleModeObject=token=="object";
        const auto firstJoint=first("rigExec:firstJoint"),lastJoint=first("rigExec:endJoint");
        for(SdfPath current=lastJoint;!current.IsEmpty() && current!=SdfPath::AbsoluteRootPath();current=current.GetParentPath()) {
            const auto *joint=source.Node(current);
            if(joint && joint->fact.type=="RigExecJoint")result.ikChain.push_back(current);
            if(current==firstJoint)break;
        }
        if(result.ikChain.empty() || result.ikChain.back()!=firstJoint)
            return Fail(error,"single-chain IK endpoints do not form an inclusive joint chain: "+path.GetString());
        std::reverse(result.ikChain.begin(),result.ikChain.end());
        if(result.targets.empty())result.targets=result.ikChain;
        if(!raw("rigExec:evaluationMode",TfToken("neverTS"),&token))return false;
        bool animated=false;
        if(token=="autoDetect")for(const auto &joint:result.ikChain) {
            for(const auto &binding:scene.jointBindings)if(binding.joint==joint)animated=true;
            for(const char *name:{"posed:space","avars:tx","avars:ty","avars:tz","avars:sx","avars:sy","avars:sz"})
                if(const auto *attribute=source.Attribute(joint.AppendProperty(TfToken(name))))
                    animated=animated || !attribute->fact.sampleTimes.empty() ||
                        (attribute->fact.hasAuthoredConnections && !attribute->fact.connections.empty());
        }
        result.useAnimatedTs=token=="alwaysTS" || (token=="autoDetect" && animated);
        if(result.record.singleChain.mode==RigExecSingleChainIkMode::RotatePlane) {
            if(!bind("inputs:poleVector") || !bind("inputs:twistDegrees"))return false;
            if(result.poleModeObject && !result.poleObjects.empty() &&
               !bind("inputs:poleVectorWeights",RigExecSceneReadRoute::Raw))return false;
        }
    } else {
        const bool parent=result.record.kind==RigExecConstraintKind::Parent;
        const char *channel=result.record.kind==RigExecConstraintKind::Position?"Translation":
            result.record.kind==RigExecConstraintKind::Scale?"Scale":"Rotation";
        for(const char *name:parent?std::initializer_list<const char*>{"Translation","Rotation","Scale"}:
                std::initializer_list<const char*>{channel})
            for(const char *axis:{"X","Y","Z"}) {
                const std::string property=std::string("inputs:affect")+name+axis;
                if(!bind(property.c_str()))return false;
            }
        if(parent) {
            if(!bind("inputs:translationOffsets",RigExecSceneReadRoute::Raw) ||
               !bind("inputs:rotationOffsets",RigExecSceneReadRoute::Raw))return false;
        } else {
            const char *offset=result.record.kind==RigExecConstraintKind::Position?"inputs:translationOffset":
                result.record.kind==RigExecConstraintKind::Scale?"inputs:scaleOffset":"inputs:rotationOffset";
            if(!bind(offset))return false;
        }
        if(result.record.kind==RigExecConstraintKind::Aim) {
            if(!bind("inputs:aimVector") || !bind("inputs:upVector") || !bind("inputs:worldUpVector"))return false;
            const auto *aim=source.Attribute(path.AppendProperty(TfToken("inputs:aimVector")));
            result.aimVectorAuthored=aim && aim->fact.hasAuthoredValue;
            if(!raw("rigExec:aimAxis",TfToken("x"),&token))return false;
            result.aimAxisFallback=token=="y"?GfVec3d(0,1,0):token=="z"?GfVec3d(0,0,1):GfVec3d(1,0,0);
            if(!raw("rigExec:worldUpType",TfToken("none"),&result.worldUpType))return false;
            result.worldUpObject=first("rigExec:worldUpObject");result.preserveInputUp=result.sources.empty();
            if(result.sources.empty()) { const auto legacy=first("rigExec:aimTarget");if(!legacy.IsEmpty())result.sources.push_back(legacy); }
            result.sceneUp=scene.upAxis=="Z"?GfVec3d(0,0,1):GfVec3d(0,1,0);

        }
    }
    *output=std::move(result);return true;
}

bool RigExecResolveSceneConstraint(const RigExecSceneDescriptors &scene,
    RigExecSceneConstraintDescriptor *descriptor,UsdTimeCode identity,
    const std::map<SdfPath,VtValue> &delivered,RigExecConstraintInputs *input,
    bool *enabled,float *defaultWeight,std::string *error) {
    if(!descriptor || !input || !enabled || !defaultWeight)return Fail(error,"null constraint input destination");
    const RigExecSceneCompileInputs source(scene);
    const auto readValue=[&](const char *name,VtValue *value) {
        *value=VtValue();const auto found=descriptor->inputs.find(name);
        if(found==descriptor->inputs.end())return true;
        const auto &binding=found->second;VtValue boxed;
        const auto typed=descriptor->typedReads.find(name);
        if(typed!=descriptor->typedReads.end()) {
            if(binding.readPhase!="base" && binding.source!=binding.consumer && !delivered.count(binding.consumer))
                return Fail(error,"phased constraint input producer unavailable: "+binding.consumer.GetString());
            if(!RigExecResolveSceneTypedRead(scene,typed->second,identity,delivered,&boxed,nullptr))return true;
        } else if(!source.Read(binding,identity,&boxed,nullptr,error))return false;
        *value=std::move(boxed);return true;
    };
    return RigExecRefreshConstraintParameters(descriptor,readValue,input,enabled,defaultWeight,error);
}
bool RigExecRefreshConstraintParameters(RigExecSceneConstraintDescriptor *descriptor,
    const std::function<bool(const char *,VtValue *)> &readValue,
    RigExecConstraintInputs *input,bool *enabled,float *defaultWeight,std::string *error) {
    if(!descriptor || !input || !enabled || !defaultWeight)return Fail(error,"null constraint parameters destination");
    const auto read=[&](const char *name,auto fallback,auto *value) {
        *value=fallback;VtValue boxed;
        if(!readValue(name,&boxed))return false;
        using T=std::decay_t<decltype(*value)>;
        if(boxed.IsHolding<T>())*value=boxed.UncheckedGet<T>();
        return true;
    };
    if(!read("inputs:enabled",true,enabled) || !read("inputs:defaultWeight",1.0f,defaultWeight))return false;
    if(!*enabled)return true;
    auto &record=descriptor->record;
    if(record.kind==RigExecConstraintKind::SingleChainIk) {
        if(record.singleChain.mode==RigExecSingleChainIkMode::RotatePlane)
            if(!read("inputs:poleVector",GfVec3d(0,1,0),&record.singleChain.pole) ||
               !read("inputs:twistDegrees",0.0,&record.singleChain.twistDegrees))return false;
        if(record.singleChain.mode==RigExecSingleChainIkMode::RotatePlane && descriptor->poleModeObject &&
            !descriptor->poleObjects.empty()) {
            VtFloatArray weights;
            if(!read("inputs:poleVectorWeights",VtFloatArray{},&weights))return false;
            if(input->poleObjects.size()!=descriptor->poleObjects.size() ||
                (!weights.empty() && weights.size()!=input->poleObjects.size()))
                return Fail(error,"single-chain IK pole cardinality mismatch");
            GfVec3d pole(0);double total=0;
            for(size_t k=0;k<input->poleObjects.size();++k) {
                const double weight=weights.empty()?1.0:double(weights[k]);
                if(!RigExecConstraintFrameUsable(input->poleObjects[k]) || !std::isfinite(weight) || weight<0)
                    return Fail(error,"invalid single-chain IK pole object/weight");
                pole+=input->poleObjects[k].Origin()*weight;total+=weight;
            }
            if(!(total>0))return Fail(error,"single-chain IK pole weights have no positive total");
            record.singleChain.pole=pole/total;
        }
        record.singleChain.weight=double(*defaultWeight);
        return true;
    }
    VtFloatArray weights;
    if(!read("inputs:sourceWeights",VtFloatArray{},&weights))return false;
    if(!weights.empty() && weights.size()!=input->sources.size())
        return Fail(error,"constraint source weight cardinality mismatch: "+descriptor->path.GetString());
    for(size_t k=0;k<input->sources.size();++k)input->sources[k].normalizedWeight=weights.empty()?1.0:double(weights[k]);
    auto mask=[&](const char *channel,bool fallback,RigExecConstraintAxisMask *value) {
        const std::string prefix=std::string("inputs:affect")+channel;
        return read((prefix+"X").c_str(),fallback,&value->x) &&
            read((prefix+"Y").c_str(),fallback,&value->y) && read((prefix+"Z").c_str(),fallback,&value->z);
    };
    const double weight=double(*defaultWeight);
    switch(record.kind) {
    case RigExecConstraintKind::Position:
        record.position.weight=weight;
        return mask("Translation",true,&record.position.affect) &&
            read("inputs:translationOffset",GfVec3d(0),&record.position.offset);
    case RigExecConstraintKind::Rotation:
        record.rotation.weight=weight;record.rotation.rotationOrder=descriptor->order;
        return mask("Rotation",true,&record.rotation.affect) &&
            read("inputs:rotationOffset",GfVec3d(0),&record.rotation.offsetDegrees);
    case RigExecConstraintKind::Scale:
        record.scale.weight=weight;record.scale.blendShear=descriptor->blendShear;
        return mask("Scale",true,&record.scale.affect) && read("inputs:scaleOffset",GfVec3d(0),&record.scale.offset);
    case RigExecConstraintKind::Parent: {
        record.parent.weight=weight;record.parent.rotationOrder=descriptor->order;
        record.parent.blendShear=descriptor->blendShear;
        if(!mask("Translation",true,&record.parent.translationAxes) ||
           !mask("Rotation",true,&record.parent.rotationAxes) || !mask("Scale",false,&record.parent.scaleAxes))return false;
        VtVec3dArray translations,rotations;
        if(!read("inputs:translationOffsets",VtVec3dArray{},&translations) ||
           !read("inputs:rotationOffsets",VtVec3dArray{},&rotations))return false;
        if((!translations.empty() && translations.size()!=input->sources.size()) ||
           (!rotations.empty() && rotations.size()!=input->sources.size()))
            return Fail(error,"parent constraint offset cardinality mismatch: "+descriptor->path.GetString());
        for(size_t k=0;k<input->sources.size();++k) {
            input->sources[k].translationOffset=translations.empty()?GfVec3d(0):translations[k];
            input->sources[k].rotationOffsetDegrees=rotations.empty()?GfVec3d(0):rotations[k];
        }
        return true;
    }
    case RigExecConstraintKind::Aim: {
        auto &params=record.aim;params.weight=weight;params.rotationOrder=descriptor->order;
        params.preserveInputUp=descriptor->preserveInputUp;params.worldUpDirection.reset();
        if(!mask("Rotation",true,&params.affectRotation) ||
           !read("inputs:upVector",GfVec3d(0,1,0),&params.localUpVector) ||
           !read("inputs:rotationOffset",GfVec3d(0),&params.rotationOffsetDegrees))return false;
        if(descriptor->aimVectorAuthored) {
            if(!read("inputs:aimVector",GfVec3d(1,0,0),&params.localAimVector))return false;
        } else params.localAimVector=descriptor->aimAxisFallback;
        GfVec3d up(0,1,0);
        if(!read("inputs:worldUpVector",up,&up))return false;
        GfVec3d target(0);double total=0;
        for(const auto &source:input->sources) {
            if(!RigExecConstraintFrameUsable(source.frame) || !std::isfinite(source.normalizedWeight) ||
               source.normalizedWeight<0)return Fail(error,"invalid aim source frame/weight");
            target+=source.frame.Origin()*source.normalizedWeight;total+=source.normalizedWeight;
        }
        if(!(total>0))return Fail(error,"aim sources have no positive total weight");
        input->aimTarget=target/total;
        if(descriptor->worldUpType=="sceneUp")params.worldUpDirection=descriptor->sceneUp;
        else if(descriptor->worldUpType=="vector")params.worldUpDirection=up;
        else if(descriptor->worldUpType=="objectUp") {
            if(descriptor->worldUpObject.IsEmpty())params.worldUpDirection=GfVec3d(-input->incoming.Origin());
            else {
                if(!input->worldUp)return Fail(error,"aim world-up producer unavailable");
                params.worldUpDirection=input->worldUp->Origin()-input->incoming.Origin();
            }
        } else if(descriptor->worldUpType=="objectRotationUp") {
            if(descriptor->worldUpObject.IsEmpty())params.worldUpDirection=up;
            else {
                GfMatrix4d matrix(1.0);
                if(!input->worldUp || !RigExecPointsToMatrix(RigExecIdentityLandmarks(),input->worldUp->points,&matrix))
                    return Fail(error,"aim world-up producer is unavailable or degenerate");
                params.worldUpDirection=(descriptor->worldUpRotationOnly?matrix.GetOrthonormalized(false):matrix)
                    .ExtractRotation().TransformDir(up);
            }
        }
        return true;
    }
    case RigExecConstraintKind::SingleChainIk:return true;
    }
    return Fail(error,"unknown constraint input kind");
}

}
