#include "poseSceneLowering.h"
#include "pxr/base/gf/quatf.h"
#include "pxr/base/vt/types.h"
#include <algorithm>
#include <cmath>
#include <type_traits>
namespace rigExec {
namespace {
bool Fail(std::string *error,const std::string &message) {if(error)*error=message;return false;}
bool Provider(const RigExecSceneCompileInputs &source,const SdfPath &path) {
    const auto *node=source.Node(path);if(!node || !node->fact.active)return false;
    const auto type=node->fact.type;
    return type=="RigExecJoint" || type=="RigExecControl" || type=="RigExecSphereWeight" ||
        type=="RigExecPlaneWeight" || type=="RigExecCurveWeight";
}
bool Driver(const RigExecSceneCompileInputs &source,const SdfPath &path) {
    const auto *node=source.Node(path);return node && node->fact.active &&
        (node->fact.type=="RigExecJoint" || node->fact.type=="RigExecControl");
}
template<class T> bool Raw(const RigExecSceneCompileInputs &source,const SdfPath &path,
    const T &fallback,T *value,std::string *error) {
    *value=fallback;if(!source.Attribute(path))return true;
    RigExecSceneBoundInput binding;VtValue boxed;
    if(!source.Bind(path,RigExecSceneReadRoute::Raw,&binding,error) ||
       !source.Read(binding,UsdTimeCode::Default(),&boxed,nullptr,error))return false;
    if(boxed.IsHolding<T>())*value=boxed.UncheckedGet<T>();return true;
}
}
bool RigExecLowerSceneSpaceSwitch(const RigExecSceneDescriptors &scene,const SdfPath &path,
    RigExecSceneSpaceSwitchDescriptor *output,std::string *error) {
    if(!output)return Fail(error,"null space switch descriptor");
    const RigExecSceneCompileInputs source(scene);const auto *node=source.Node(path);
    if(!node || !node->fact.active || node->fact.type!="RigExecSpaceSwitch")
        return Fail(error,"missing active space switch: "+path.GetString());
    RigExecSceneSpaceSwitchDescriptor result;result.path=path;
    const auto property=[&](const char *name){return path.AppendProperty(TfToken(name));};
    const auto targets=source.Targets(property("rigExec:target"));
    if(targets.size()!=1 || !Provider(source,targets[0]))
        return Fail(error,"space switch requires exactly one frame provider target: "+path.GetString());
    result.target=targets[0];
    VtFloatArray weights;if(!Raw(source,property("inputs:sourceWeights"),VtFloatArray{},&weights,error))return false;
    if(!weights.empty())return Fail(error,"space switch does not read inputs:sourceWeights: "+path.GetString());
    const auto sources=source.Targets(property("rigExec:sources"));
    if(sources.empty())return Fail(error,"space switch needs at least one source: "+path.GetString());
    VtTokenArray labels,filters;
    if(!Raw(source,property("rigExec:spaceLabels"),VtTokenArray{},&labels,error) ||
       !Raw(source,property("rigExec:rotationFilters"),VtTokenArray{},&filters,error) ||
       !Raw(source,property("rigExec:twistAxis"),GfVec3d(1,0,0),&result.record.twistAxis,error))return false;
    if((!labels.empty() && labels.size()!=sources.size()) ||
       (!filters.empty() && filters.size()!=sources.size()))
        return Fail(error,"space switch labels/filters must parallel its sources: "+path.GetString());
    for(size_t k=0;k<sources.size();++k) {
        result.sources.push_back(Provider(source,sources[k])?sources[k]:SdfPath{});
        result.labels.push_back(labels.empty()?TfToken(sources[k].GetName()):labels[k]);
        const TfToken filter=filters.empty()?TfToken("all"):filters[k];
        if(!filter.IsEmpty() && filter!="all" && filter!="twist" && filter!="swing")
            return Fail(error,"unknown space switch rotation filter: "+filter.GetString());
        result.record.filters.push_back(filter=="twist"?1:filter=="swing"?2:0);
    }
    const auto spaces=source.Targets(property("rigExec:space"));
    if(spaces.size()>1)return Fail(error,"space switch has more than one rigExec:space: "+path.GetString());
    if(!spaces.empty() && Provider(source,spaces[0]))result.space=spaces[0];
    const auto active=source.Targets(property("rigExec:activeSpaceAttribute"));
    if(active.size()>1)return Fail(error,"space switch has more than one active attribute: "+path.GetString());
    result.activeAttribute=active.empty()?property("inputs:activeSpace"):active[0];
    if(!result.activeAttribute.IsPropertyPath())return Fail(error,"space switch active target is not a property");
    if(!Raw(source,property("inputs:activeSpace"),0.0,&result.activeFallback,error))return false;
    const auto *attribute=source.Attribute(result.activeAttribute);
    result.tokenIndex=attribute && attribute->fact.type==SdfValueTypeNames->Token;
    if(!RigExecBindSceneTypedRead(scene,result.activeAttribute,
        result.tokenIndex?SdfValueTypeNames->Token:SdfValueTypeNames->Double,&result.active,error))return false;
    const char *channels[]={"Translation","Rotation","Scale"};
    for(size_t channel=0;channel<3;++channel)for(size_t axis=0;axis<3;++axis) {
        const std::string name=std::string("inputs:affect")+channels[channel]+"XYZ"[axis];
        auto &mask=channel==0?result.record.affectTranslation:
            channel==1?result.record.affectRotation:result.record.affectScale;
        if(!Raw(source,path.AppendProperty(TfToken(name)),true,&mask[axis],error))return false;
    }
    *output=std::move(result);return true;
}
bool RigExecResolveSceneSpaceSwitch(const RigExecSceneDescriptors &scene,
    const RigExecSceneSpaceSwitchDescriptor &record,UsdTimeCode time,
    const std::map<SdfPath,VtValue> &delivered,double *active,std::string *error) {
    if(!active)return Fail(error,"null space switch selector");
    *active=record.tokenIndex?0.0:record.activeFallback;VtValue value;
    if(!RigExecResolveSceneTypedRead(scene,record.active,time,delivered,&value,error))return true;
    if(record.tokenIndex && value.IsHolding<TfToken>()) {
        const auto found=std::find(record.labels.begin(),record.labels.end(),value.UncheckedGet<TfToken>());
        if(found!=record.labels.end())*active=double(found-record.labels.begin());
    } else if(value.IsHolding<double>())*active=value.UncheckedGet<double>();
    return true;
}
bool RigExecLowerScenePoseInterpolator(const RigExecSceneDescriptors &scene,const SdfPath &path,
    RigExecScenePoseInterpolatorDescriptor *output,std::string *error) {
    if(!output)return Fail(error,"null pose interpolator descriptor");
    const RigExecSceneCompileInputs source(scene);const auto *node=source.Node(path);
    if(!node || !node->fact.active || node->fact.type!="RigExecPoseInterpolator")
        return Fail(error,"missing active pose interpolator: "+path.GetString());
    RigExecScenePoseInterpolatorDescriptor result;result.path=path;
    const auto property=[&](const char *name){return path.AppendProperty(TfToken(name));};
    result.driverAttributes=source.Targets(property("rigExec:driverAttributes"));
    if(result.driverAttributes.size()>3)return Fail(error,path.GetString()+": rigExec:driverAttributes has more than three numeric drivers");
    for(const auto &driver:result.driverAttributes) {
        if(!driver.IsPropertyPath() || !source.Attribute(driver))return Fail(error,"pose numeric driver is not an attribute");
        RigExecSceneTypedRead read;
        if(!RigExecBindSceneTypedRead(scene,driver,SdfValueTypeNames->Double,&read,error))return false;
        result.numericReads.push_back(std::move(read));
    }
    const auto drivers=source.Targets(property("rigExec:driver"));
    if(drivers.size()!=1 && (result.driverAttributes.empty() || !drivers.empty()))
        return Fail(error,"pose interpolator requires exactly one frame driver");
    if(!drivers.empty()) {
        result.driver=drivers[0].GetPrimPath();
        if(!Driver(source,result.driver))return Fail(error,"pose driver is not a joint or control");
        for(auto parent=result.driver.GetParentPath();!parent.IsEmpty() && !parent.IsAbsoluteRootPath() &&
            parent!=scene.rigRoot.GetParentPath();parent=parent.GetParentPath())
            if(Driver(source,parent)) {result.parent=parent;break;}
        if(!result.parent.IsEmpty() && result.parent!=result.driver.GetParentPath())
            result.notes.push_back("warning: pose interpolator "+path.GetString()+" measures its driver against "+
                result.parent.GetString()+", which is not the driver's immediate namespace parent");
    }
    if(!RigExecBindSceneTypedRead(scene,property("inputs:enabled"),SdfValueTypeNames->Bool,&result.enabled,error))return false;
    RigExecRbfSolverDesc desc;TfToken token;
    if(!Raw(source,property("rigExec:allowNegativeWeights"),true,&result.record.allowNegativeWeights,error) ||
       !Raw(source,property("rigExec:kernel"),TfToken("gaussian"),&token,error))return false;
    desc.kernel=token=="linear"?RigExecRbfKernel::Linear:RigExecRbfKernel::Gaussian;
    float regularization=0;
    if(!Raw(source,property("rigExec:regularization"),0.0f,&regularization,error) ||
       !Raw(source,property("rigExec:normalize"),true,&desc.normalize,error) ||
       !Raw(source,property("rigExec:enableRotation"),true,&desc.enableRotation,error) ||
       !Raw(source,property("rigExec:enableTranslation"),false,&desc.enableTranslation,error) ||
       !Raw(source,property("rigExec:twistAxis"),TfToken("X"),&token,error))return false;
    desc.regularization=regularization;
    desc.twistAxis=token=="Y"?GfVec3d(0,1,0):token=="Z"?GfVec3d(0,0,1):GfVec3d(1,0,0);
    if(!result.driverAttributes.empty()) {desc.enableRotation=false;desc.enableTranslation=true;}
    result.record.enableTranslation=desc.enableTranslation;
    std::vector<double> radii,translationRadii;size_t children=0;
    for(const auto &childPath:node->fact.children) {
        const auto *child=source.Node(childPath);if(!child || !child->fact.active)continue;
        if(child->fact.type!="RigExecPose")return Fail(error,"pose interpolator child is not a RigExecPose: "+childPath.GetString());
        ++children;const auto weight=childPath.AppendProperty(TfToken("outputs:weight"));
        if(const auto *attribute=source.Attribute(weight))if(attribute->fact.hasAuthoredConnections)
            return Fail(error,"pose weight output carries authored connections: "+weight.GetString());
        bool enabled=true;if(!Raw(source,childPath.AppendProperty(TfToken("inputs:enabled")),true,&enabled,error))return false;
        if(!enabled) {result.disabledPoseWeights.push_back(weight);continue;}
        GfQuatf rotation(1.0f);GfVec3f translation(0);TfToken kind;
        float radius=0,translationRadius=0;
        if(!Raw(source,childPath.AppendProperty(TfToken("rigExec:rotation")),GfQuatf(1.0f),&rotation,error) ||
           !Raw(source,childPath.AppendProperty(TfToken("rigExec:translation")),GfVec3f(0),&translation,error) ||
           !Raw(source,childPath.AppendProperty(TfToken("rigExec:poseType")),TfToken("swing"),&kind,error) ||
           !Raw(source,childPath.AppendProperty(TfToken("rigExec:rotationRadius")),0.0f,&radius,error) ||
           !Raw(source,childPath.AppendProperty(TfToken("rigExec:translationRadius")),0.0f,&translationRadius,error))return false;
        if(!std::isfinite(radius) || radius<0 || !std::isfinite(translationRadius) || translationRadius<0)
            return Fail(error,"pose radius is negative or not finite: "+childPath.GetString());
        desc.poses.push_back(RigExecRbfEulerFromQuaternion(GfQuatd(rotation.GetReal(),GfVec3d(rotation.GetImaginary()))));
        desc.translations.push_back(GfVec3d(translation[0]/100.0,translation[1]/100.0,translation[2]/100.0));
        desc.poseTypes.push_back(kind=="twist"?RigExecRbfPoseType::Twist:kind=="whole"?RigExecRbfPoseType::Whole:RigExecRbfPoseType::Swing);
        radii.push_back(double(radius));translationRadii.push_back(double(translationRadius)/100.0);
        result.poseWeights.push_back(weight);
    }
    if(!children)return Fail(error,"pose interpolator has no pose children: "+path.GetString());
    result.record.poseCount=result.poseWeights.size();
    if(!result.poseWeights.empty()) {
        if(!desc.enableTranslation) {desc.translations.clear();translationRadii.clear();}
        RigExecRbfSolver solver(desc);solver.SetSolvedTable(radii,translationRadii,{});
        if(!solver.Solve())return Fail(error,"pose interpolator could not be solved: "+path.GetString());
        if(solver.Degenerate())result.notes.push_back("warning: pose interpolator "+path.GetString()+
            " has poses that are coincident under the channels it has enabled; its weights will sit at 1/n");
        result.record.solver=std::move(solver);
    }
    *output=std::move(result);return true;
}
bool RigExecResolveScenePoseInterpolator(const RigExecSceneDescriptors &scene,
    const RigExecScenePoseInterpolatorDescriptor &record,UsdTimeCode time,
    const std::map<SdfPath,VtValue> &delivered,RigExecPoseInterpolatorInputs *input,
    std::string *error) {
    if(!input)return Fail(error,"null pose interpolator inputs");
    input->enabled=true;VtValue value;
    if(RigExecResolveSceneTypedRead(scene,record.enabled,time,delivered,&value,error) && value.IsHolding<bool>())
        input->enabled=value.UncheckedGet<bool>();
    input->numeric.assign(record.numericReads.size(),0.0);
    for(size_t k=0;k<record.numericReads.size();++k)
        if(RigExecResolveSceneTypedRead(scene,record.numericReads[k],time,delivered,&value,error) && value.IsHolding<double>())
            input->numeric[k]=value.UncheckedGet<double>();
    return true;
}
}
