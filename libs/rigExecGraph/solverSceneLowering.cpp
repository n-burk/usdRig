#include "solverSceneLowering.h"
#include "sceneInputTypes.h"
#include "rigExec/solverKernels.h"
#include "rigExec/types.h"
#include "pxr/base/vt/types.h"
#include "pxr/base/gf/math.h"
#include <type_traits>
namespace rigExec {
namespace {
bool Fail(std::string *error,const std::string &text) { if(error)*error=text;return false; }
}
bool RigExecLowerSceneSolver(const RigExecSceneDescriptors &scene,const SdfPath &path,
    const std::map<SdfPath,RigExecPointFrame> &rests,RigExecSceneSolverDescriptor *output,
    std::string *error) {
    if(!output)return Fail(error,"null solver descriptor destination");
    const RigExecSceneCompileInputs source(scene);
    const auto *node=source.Node(path);
    if(!node || !node->fact.active || node->domain!=RigExecSceneDomain::Solver)
        return Fail(error,"missing active solver: "+path.GetString());
    RigExecSceneSolverDescriptor result;result.path=path;
    auto &s=result.record;
    const auto type=node->fact.type;
    if(type=="RigExecFkChain")s.kind=RigExecSolverKind::FkChain;
    else if(type=="RigExecTwoBoneIk")s.kind=RigExecSolverKind::TwoBoneIk;
    else if(type=="RigExecBlendPointFrames")s.kind=RigExecSolverKind::BlendPointFrames;
    else if(type=="RigExecTwistDistribution")s.kind=RigExecSolverKind::TwistDistribution;
    else if(type=="RigExecRibbon")s.kind=RigExecSolverKind::Ribbon;
    else if(type=="RigExecSplineIk")s.kind=RigExecSolverKind::SplineIk;
    else return Fail(error,"unsupported solver: "+type.GetString());
    auto targets=[&](const char *name) { return source.Targets(path.AppendProperty(TfToken(name))); };
    auto providerExists=[&](const SdfPath &provider) {
        const auto *fact=source.Node(provider);
        return fact && fact->domain==RigExecSceneDomain::Provider;
    };
    // Numerical seed availability cannot change structural solver cardinality.
    auto rest=[&](const SdfPath &provider) {
        const auto found=rests.find(provider);
        if(found!=rests.end())return found->second;
        RigExecPointFrame unavailable;unavailable.flags=0;return unavailable;
    };
    auto first=[&](const char *name,bool provider) {
        const auto list=targets(name);
        return list.empty() || (provider && !providerExists(list.front()))?SdfPath{}:list.front();
    };
    auto read=[&](const SdfPath &property,UsdTimeCode time,auto *value,bool fixed) {
        const auto *attribute=source.Attribute(property);
        if(!attribute)return true;
        if(fixed && (attribute->fact.mightBeTimeVarying || !attribute->fact.sampleTimes.empty()))
            return Fail(error,"time-varying solver structure: "+property.GetString());
        RigExecSceneBoundInput binding;
        if(!source.Bind(property,RigExecSceneReadRoute::Raw,&binding,error))return false;
        VtValue raw;
        if(!source.Read(binding,time,&raw,nullptr,error))return false;
        using T=std::decay_t<decltype(*value)>;
        if(raw.IsEmpty())return true;
        if(!raw.IsHolding<T>())return true;
        *value=raw.UncheckedGet<T>();return true;
    };
    auto token=[&](const char *name,const char *fallback,TfToken *value) {
        *value=TfToken(fallback);
        return read(path.AppendProperty(TfToken(name)),UsdTimeCode::Default(),value,true);
    };
    auto bind=[&](const char *name,auto fallback) {
        const auto property=path.AppendProperty(TfToken(name));
        if(!source.Attribute(property))return true;
        RigExecSceneBoundInput binding;
        if(!source.Bind(property,RigExecSceneReadRoute::ConnectionResolved,&binding,error))return false;
        // Empty schema values retain the typed computation fallback.
        for(auto &value:binding.values)if(value.IsEmpty())value=VtValue(fallback);
        RigExecSceneTypedRead typed;
        if(!RigExecBindSceneTypedRead(scene,property,RigExecSceneInputType<decltype(fallback)>(),&typed,error))return false;
        result.typedReads.emplace(name,std::move(typed));
        result.inputs.emplace(name,std::move(binding));return true;
    };
    auto initial=[&](const char *name,auto fallback) {
        const auto found=result.typedReads.find(name);
        if(found==result.typedReads.end() || scene.identities.empty())return fallback;
        VtValue value;
        if(!RigExecResolveSceneTypedRead(scene,found->second,scene.identities.front(),{},&value,nullptr))return fallback;
        using T=decltype(fallback);
        return value.IsHolding<T>()?value.UncheckedGet<T>():fallback;
    };
    VtIntArray elements;
    const bool consumesElements=s.kind==RigExecSolverKind::TwoBoneIk || s.kind==RigExecSolverKind::SplineIk;
    if(consumesElements && !read(path.AppendProperty(TfToken("rigExec:jointElements")),UsdTimeCode::Default(),&elements,true))return false;
    for(const auto &joint:targets("rigExec:joints"))if(providerExists(joint))result.joints.push_back(joint);
    const bool remap=consumesElements && !elements.empty();
    if(remap && elements.size()!=result.joints.size())s.degenerate=true;
    for(size_t k=0;k<result.joints.size();++k) {
        result.jointElements.push_back(remap && !s.degenerate?elements[k]:int(k));
        s.jointRests.push_back(rest(result.joints[k]).points);
    }
    s.restIsLive.assign(result.joints.size(),false);
    if(s.kind==RigExecSolverKind::FkChain) {
        TfToken mode;if(!token("rigExec:controlSpace","",&mode))return false;
        s.parentRelative=mode=="parentRelative";
        if(!token("rigExec:segmentScale","none",&mode))return false;
        s.scaleSegments=mode=="toChild";
        for(const auto &control:targets("rigExec:controls"))if(providerExists(control)) {
            result.controls.push_back(control);s.controlRests.push_back(rest(control).points);
        }
        result.start=first("rigExec:startFrame",true);s.hasStart=!result.start.IsEmpty();
        if(s.hasStart)s.startRest=rest(result.start).points;
    } else if(s.kind==RigExecSolverKind::TwoBoneIk) {
        result.root=first("rigExec:rootControl",true);result.end=first("rigExec:effectorControl",true);
        result.pole=first("rigExec:poleControl",true);
        s.degenerate=s.degenerate || result.root.IsEmpty() || result.end.IsEmpty() || result.pole.IsEmpty();
        std::array<bool,3> seen{};
        for(size_t k=0;k<result.joints.size();++k) {
            const int element=result.jointElements[k];
            if(element<0 || element>=3)s.degenerate=true;
            else { s.ikRests[size_t(element)]=s.jointRests[k];seen[size_t(element)]=true; }
        }
        s.degenerate=s.degenerate || !(seen[0] && seen[1] && seen[2]);
        result.space=first("rigExec:space",true);
        if(!bind("rigExec:spaceMatrix",GfMatrix4d(1.0)) || !bind("rigExec:preferredBendRadians",0.0) ||
           !bind("rigExec:upperLengthOffset",0.0) || !bind("rigExec:lowerLengthOffset",0.0) ||
           !bind("inputs:stretch",1.0f) || !bind("inputs:softness",0.0f) ||
           !bind("inputs:pin",0.0f) || !bind("inputs:upperScale",1.0) || !bind("inputs:lowerScale",1.0) ||
           !bind("inputs:softDistance",0.0f) || !bind("inputs:twist",0.0f))return false;
        TfToken policy,scaling;
        if(!token("rigExec:stretchPolicy","uniformSegments",&policy) ||
           !token("rigExec:segmentScale","none",&scaling) ||
           !read(path.AppendProperty(TfToken("rigExec:scaleCalibration")),UsdTimeCode::Default(),&s.ikParams.limb.scaleCalibration,true))return false;
        s.ikParams.softDistancePolicy=policy=="softDistance";s.ikParams.scaleSegments=scaling=="toChild";
        s.ikParams.preferredBendRadians=initial("rigExec:preferredBendRadians",0.0);
        s.ikParams.stretch=initial("inputs:stretch",1.0f);
        s.ikParams.softness=initial("inputs:softness",0.0f);
        if(!s.degenerate)RigExecTwoBoneIkLengths(s.ikRests,initial("rigExec:spaceMatrix",GfMatrix4d(1.0)),
            initial("rigExec:upperLengthOffset",0.0),initial("rigExec:lowerLengthOffset",0.0),
            &s.ikParams.upperLength,&s.ikParams.lowerLength);

    } else if(s.kind==RigExecSolverKind::BlendPointFrames) {
        result.blendA=first("rigExec:inputA",false);result.blendB=first("rigExec:inputB",false);
        TfToken mode;if(!token("rigExec:scaleBlend","",&mode))return false;
        s.scaleMode=mode=="linear"?RigExecScaleBlend::Linear:RigExecScaleBlend::Log;
        if(!token("rigExec:rotationBlend","shortestArc",&mode))return false;
        s.blendRotationRejected=mode!="shortestArc";
        if(!bind("inputs:weight",0.0f))return false;
    } else if(s.kind==RigExecSolverKind::TwistDistribution) {
        result.root=first("rigExec:start",true);result.end=first("rigExec:end",true);
        s.degenerate=s.degenerate || result.root.IsEmpty() || result.end.IsEmpty();
        if(!result.root.IsEmpty())s.twistStartRest=rest(result.root).points;
        if(!result.end.IsEmpty())s.twistEndRest=rest(result.end).points;
        VtFloatArray weights;int count=1;
        if(!read(path.AppendProperty(TfToken("rigExec:weights")),UsdTimeCode::Default(),&weights,true) ||
           !read(path.AppendProperty(TfToken("rigExec:count")),UsdTimeCode::Default(),&count,true))return false;
        for(float weight:weights)s.twistWeights.push_back(weight);
        RigExecResolveTwistWeights(count,&s.twistWeights);
        if(!bind("inputs:twistTurns",0.0))return false;
    } else if(s.kind==RigExecSolverKind::Ribbon) {
        result.ribbonPoints=first("rigExec:driverCurve",false);
        if(result.ribbonPoints.IsPrimPath())result.ribbonPoints=result.ribbonPoints.AppendProperty(TfToken("points"));
        const auto *relation=source.Relationship(path.AppendProperty(TfToken("rigExec:driverCurve")));
        result.ribbonReadPhase=relation?relation->readPhase:TfToken("base");
        VtVec3fArray points;
        if(!result.ribbonPoints.IsEmpty() && !read(result.ribbonPoints,UsdTimeCode::Default(),&points,false))return false;
        s.ribbonRestPoints.assign(points.begin(),points.end());
        if(!result.ribbonPoints.IsEmpty() && source.Attribute(result.ribbonPoints)) {
            RigExecSceneBoundInput binding;
            if(!source.Bind(result.ribbonPoints,RigExecSceneReadRoute::Raw,&binding,error))return false;
            result.inputs.emplace("driverPoints",std::move(binding));
        }
        if(!bind("rigExec:sampleCount",5))return false;
    } else if(s.kind==RigExecSolverKind::SplineIk) {
        result.root=first("rigExec:rootControl",true);result.mid=first("rigExec:midControl",true);
        result.end=first("rigExec:endControl",true);result.space=first("rigExec:space",true);
        s.degenerate=s.degenerate || result.root.IsEmpty() || result.mid.IsEmpty() || result.end.IsEmpty();
        const size_t count=result.joints.size();s.splineCount=count;s.degenerate=s.degenerate || count==0;
        s.splineRestFrames.resize(count);s.splineJointRests.resize(count);std::vector<bool> filled(count,false);
        for(size_t k=0;k<count;++k) {
            const int element=result.jointElements[k];
            if(element<0 || size_t(element)>=count || filled[size_t(element)]) { s.degenerate=true;continue; }
            filled[size_t(element)]=true;s.splineRestFrames[size_t(element)]=rest(result.joints[k]);
            s.splineJointRests[size_t(element)]=s.jointRests[k];
        }
        VtFloatArray weights;TfToken mode;
        if(!read(path.AppendProperty(TfToken("rigExec:volumeWeights")),UsdTimeCode::Default(),&weights,true) ||
           !token("rigExec:restLength","",&mode))return false;
        for(float weight:weights)s.splineRestWeights.push_back(weight);
        if(!weights.empty() && weights.size()!=count)s.degenerate=true;
        if(!mode.IsEmpty() && mode!="curve" && mode!="chain")s.degenerate=true;
        s.splineRestMode=mode=="chain"?RigExecSplineIkRestLength::Chain:RigExecSplineIkRestLength::Curve;
        if(!result.root.IsEmpty())s.splineRootRest=rest(result.root);
        if(!result.mid.IsEmpty())s.splineMidRest=rest(result.mid);
        if(!result.end.IsEmpty())s.splineEndRest=rest(result.end);
        s.splineRest=RigExecSplineIkMakeRest(s.splineRestFrames,s.splineRootRest,s.splineMidRest,s.splineEndRest,
            s.splineRestWeights,s.splineRestMode);
        if(!token("rigExec:rootTangent","",&mode))return false;
        s.splineParams.aimRootTangent=mode=="aim";
        if(!mode.IsEmpty() && mode!="aim" && mode!="rigid")s.degenerate=true;
        if(!bind("inputs:preserveVolume",1.0) || !bind("inputs:midFollowWeight",0.5) || !bind("inputs:roll",0.0) ||
           !bind("inputs:twist",0.0) || !bind("inputs:minLengthRatio",0.0))return false;
        s.splineParams.preserveVolume=initial("inputs:preserveVolume",1.0);
        s.splineParams.midFollowWeight=initial("inputs:midFollowWeight",0.5);
        s.splineParams.roll=GfDegreesToRadians(initial("inputs:roll",0.0));
        s.splineParams.twist=GfDegreesToRadians(initial("inputs:twist",0.0));
        s.splineParams.minLengthRatio=initial("inputs:minLengthRatio",0.0);

    }
    for(const char *name:{"rigExec:controls","rigExec:startFrame","rigExec:rootControl","rigExec:midControl",
        "rigExec:effectorControl","rigExec:endControl","rigExec:poleControl","rigExec:start","rigExec:end",
        "rigExec:space","rigExec:inputA","rigExec:inputB"}) {
        const auto relationPath=path.AppendProperty(TfToken(name));
        RigExecSceneSolverFrameBinding binding;
        const auto *relation=source.Relationship(relationPath);
        binding.phase=relation?relation->readPhase:TfToken("base");
        if(relation)for(const auto &target:relation->forwardedTargets)
            binding.keys.emplace(target,relationPath.AppendTarget(target));
        result.frameBindings.emplace(name,std::move(binding));
    }
    if(!result.space.IsEmpty())result.spaceRest=rest(result.space).points;
    *output=std::move(result);return true;
}
bool RigExecResolveSceneSolverInputs(const RigExecSceneDescriptors &scene,
    const RigExecSceneSolverDescriptor &descriptor,UsdTimeCode identity,
    const std::map<SdfPath,RigExecPointFrame> &frames,
    const std::map<SdfPath,RigExecPointFrameArray> &aggregates,
    const std::map<SdfPath,VtValue> &deliveredInputs,RigExecSolverInputs *output,std::string *error) {
    if(!output)return Fail(error,"null solver inputs destination");
    const RigExecSceneCompileInputs source(scene);
    auto &input=*output;
    auto frame=[&](const SdfPath &path,const char *relationName,RigExecPointFrame *value) {
        if(path.IsEmpty())return true;
        const auto relation=descriptor.frameBindings.find(relationName);
        if(relation!=descriptor.frameBindings.end()) {
            const auto key=relation->second.keys.find(path);
            if(key!=relation->second.keys.end()) {
                const auto bound=frames.find(key->second);
                if(bound!=frames.end()) { *value=bound->second;return true; }
            }
            if(relation->second.phase!="base")
                return Fail(error,"phased solver frame producer unavailable: "+path.GetString());
        }
        const auto found=frames.find(path);
        if(found==frames.end())return Fail(error,"solver frame producer unavailable: "+path.GetString());
        *value=found->second;return true;
    };
    const auto readValue=[&](const char *name,VtValue *result) {
        const auto found=descriptor.inputs.find(name);
        *result=VtValue();
        if(found==descriptor.inputs.end())return true;
        const auto &binding=found->second;
        VtValue boxed;
        const auto typed=descriptor.typedReads.find(name);
        if(typed!=descriptor.typedReads.end()) {
            if(binding.readPhase!="base" && binding.source!=binding.consumer && !deliveredInputs.count(binding.consumer))
                return Fail(error,"phased solver input producer unavailable: "+binding.consumer.GetString());
            if(!RigExecResolveSceneTypedRead(scene,typed->second,identity,deliveredInputs,&boxed,nullptr))return true;
        } else {
            const auto delivered=deliveredInputs.find(binding.consumer);
            if(binding.route!=RigExecSceneReadRoute::Raw && delivered!=deliveredInputs.end())boxed=delivered->second;
            else if(!source.Read(binding,identity,&boxed,nullptr,error))return false;
        }
        *result=std::move(boxed);return true;
    };
    if(descriptor.record.degenerate)return true;
    input.controls.resize(descriptor.controls.size());
    for(size_t k=0;k<descriptor.controls.size();++k)
        if(!frame(descriptor.controls[k],"rigExec:controls",&input.controls[k]))return false;
    const bool twist=descriptor.record.kind==RigExecSolverKind::TwistDistribution;
    const bool twoBone=descriptor.record.kind==RigExecSolverKind::TwoBoneIk;
    if(!frame(descriptor.start,"rigExec:startFrame",&input.start) ||
       !frame(descriptor.root,twist?"rigExec:start":"rigExec:rootControl",&input.root) ||
       !frame(descriptor.mid,"rigExec:midControl",&input.mid) ||
       !frame(descriptor.end,twist?"rigExec:end":twoBone?"rigExec:effectorControl":"rigExec:endControl",&input.end) ||
       !frame(descriptor.pole,"rigExec:poleControl",&input.pole) ||
       !frame(descriptor.space,"rigExec:space",&input.space))return false;
    input.hasSpace=!descriptor.space.IsEmpty();
    input.spaceRest=descriptor.spaceRest;
    input.blendA=nullptr;input.blendB=nullptr;
    auto aggregate=[&](const SdfPath &path,const char *name,const RigExecPointFrameArray **value) {
        if(path.IsEmpty())return true;
        const auto relation=descriptor.frameBindings.find(name);
        if(relation!=descriptor.frameBindings.end()) {
            const auto key=relation->second.keys.find(path);
            if(key!=relation->second.keys.end()) {
                const auto phased=aggregates.find(key->second);
                if(phased!=aggregates.end()) { *value=&phased->second;return true; }
            }
            if(relation->second.phase!="base")
                return Fail(error,"phased solver aggregate producer unavailable: "+path.GetString());
        }
        const auto found=aggregates.find(path);
        if(found!=aggregates.end())*value=&found->second;
        return true;
    };
    if(!aggregate(descriptor.blendA,"rigExec:inputA",&input.blendA) ||
       !aggregate(descriptor.blendB,"rigExec:inputB",&input.blendB))return false;
    return RigExecRefreshSolverParameters(descriptor.record.kind,readValue,&input,error);
}

bool RigExecRefreshSolverParameters(RigExecSolverKind kind,
    const std::function<bool(const char *,VtValue *)> &read,
    RigExecSolverInputs *output,std::string *error) {
    if(!output)return Fail(error,"null solver parameters destination");
    auto &input=*output;
    const auto value=[&](const char *name,auto fallback,auto *result) {
        *result=fallback;VtValue boxed;
        if(!read(name,&boxed))return false;
        using T=std::decay_t<decltype(*result)>;
        if(boxed.IsHolding<T>())*result=boxed.UncheckedGet<T>();
        return true;
    };
    switch(kind) {
    case RigExecSolverKind::TwoBoneIk:
        input.refreshIkParams=true;
        return value("rigExec:spaceMatrix",GfMatrix4d(1.0),&input.ikSpace) &&
            value("rigExec:preferredBendRadians",0.0,&input.bend) &&
            value("rigExec:upperLengthOffset",0.0,&input.upperOffset) &&
            value("rigExec:lowerLengthOffset",0.0,&input.lowerOffset) &&
            value("inputs:stretch",1.0f,&input.stretch) && value("inputs:softness",0.0f,&input.softness) &&
            value("inputs:pin",0.0f,&input.pin) && value("inputs:upperScale",1.0,&input.upperScale) &&
            value("inputs:lowerScale",1.0,&input.lowerScale) && value("inputs:softDistance",0.0f,&input.softDistance) &&
            value("inputs:twist",0.0f,&input.limbTwist);
    case RigExecSolverKind::BlendPointFrames: {
        float weight=0;
        if(!value("inputs:weight",0.0f,&weight))return false;
        input.blendWeight=double(weight);return true;
    }
    case RigExecSolverKind::TwistDistribution:
        return value("inputs:twistTurns",0.0,&input.twistTurns);
    case RigExecSolverKind::Ribbon: {
        VtVec3fArray points;
        if(!value("driverPoints",VtVec3fArray{},&points) ||
            !value("rigExec:sampleCount",5,&input.ribbonSampleCount))return false;
        input.ribbonPoints.assign(points.begin(),points.end());return true;
    }
    case RigExecSolverKind::SplineIk:
        input.refreshSplineParams=true;
        return value("inputs:preserveVolume",1.0,&input.preserveVolume) &&
            value("inputs:midFollowWeight",0.5,&input.midFollowWeight) &&
            value("inputs:roll",0.0,&input.rollDegrees) && value("inputs:twist",0.0,&input.twistDegrees) &&
            value("inputs:minLengthRatio",0.0,&input.minLengthRatio);
    case RigExecSolverKind::FkChain:return true;
    }
    return Fail(error,"unknown normalized solver input kind");
}

}
