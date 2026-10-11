#include "autoClavicleGraph.h"
#include "rigExec/frameExtraction.h"
#include "rigExecMath/rbf.h"
#include "rigExecMath/pointFrame.h"
#include "pxr/base/gf/quatf.h"
#include "pxr/base/vt/types.h"
#include <algorithm>
#include <cmath>

namespace rigExec {
namespace {
bool Fail(std::string *error,const std::string &message) {
    if(error)*error=message;
    return false;
}
template<class T> bool Constant(const RigExecSceneCompileInputs &source,
    const SdfPath &path,T *value,std::string *error) {
    const auto *attribute=source.Attribute(path);
    if(!attribute)return true;
    if(!attribute->fact.sampleTimes.empty())return Fail(error,"auto clavicle constant is animated: "+path.GetString());
    RigExecSceneBoundInput binding;VtValue boxed;
    if(!source.Bind(path,RigExecSceneReadRoute::Raw,&binding,error) ||
       !source.Read(binding,UsdTimeCode::Default(),&boxed,nullptr,error))return false;
    if(boxed.IsHolding<T>())*value=boxed.UncheckedGet<T>();
    return true;
}
}
bool RigExecBindAutoClavicle(const RigExecSceneDescriptors &scene,const SdfPath &path,
    const RigExecSceneGraphBindingContext &context,RigExecValueId incoming,
    RigExecValueId output,RigExecBoundAutoClavicle *destination,std::string *error) {
    if(!destination || !context.resolve)return Fail(error,"missing auto clavicle graph linker");
    RigExecSceneCompileInputs source(scene);RigExecBoundAutoClavicle result;
    result.output=output;result.frames[0]=incoming;result.reads.push_back(incoming);
    result.unavailable=path.GetString()+" has no usable frame for its limb; the clavicle is not carried";
    const auto property=[&](const char *name){return path.AppendProperty(TfToken(name));};
    const auto one=[&](const char *name,bool required,SdfPath *out) {
        const auto targets=source.Targets(property(name));
        if(targets.size()>1 || (required && targets.empty()))return Fail(error,path.GetString()+" requires "+name);
        if(!targets.empty())*out=targets.front();return true;
    };
    SdfPath target,pivot,anchor,ik,pole,blend,amount;
    if(!one("rigExec:target",true,&target) || !one("rigExec:pivot",true,&pivot) ||
       !one("rigExec:anchor",true,&anchor) || !one("rigExec:ikTarget",false,&ik) ||
       !one("rigExec:poleControl",false,&pole) || !one("rigExec:ikBlendAttribute",false,&blend) ||
       !one("rigExec:amountAttribute",false,&amount))return false;
    const auto fk=source.Targets(property("rigExec:fkControls"));
    if(fk.size()!=3)return Fail(error,path.GetString()+" requires three FK controls");
    const auto frame=[&](const SdfPath &provider,const char *computation,size_t index) {
        if(provider.IsEmpty())return true;
        const auto *node=source.Node(provider);
        if(!node || !node->fact.active || !node->transformProvider)
            return Fail(error,"auto clavicle input is not an active frame provider: "+provider.GetString());
        RigExecSceneGraphReadRequest request;request.consumer=path;request.reader=path;
        request.source=provider;request.domain=RigExecSceneValueDomain::Pose;
        // Own descendants are measured on the entering revision, avoiding
        // self-feedback. Independent providers keep their final producers.
        request.phase=TfToken(provider.HasPrefix(target)?"preceding":"final");
        if(computation)request.computation=computation;
        if(!context.resolve(request,&result.frames[index],error))return false;
        result.reads.push_back(result.frames[index]);return true;
    };
    if(!frame(pivot,nullptr,1) || !frame(anchor,nullptr,2) ||
       !frame(anchor,"computeDefaultFrame",3) || !frame(fk[0],nullptr,4) ||
       !frame(fk[0],"computeDefaultFrame",5) || !frame(fk[1],"computeDefaultFrame",6) ||
       !frame(fk[2],"computeDefaultFrame",7) || !frame(ik,nullptr,8) || !frame(pole,nullptr,9))return false;
    auto &c=result.constants;GfMatrix4d basis(1.0);VtQuatfArray rotations;
    VtFloatArray falloffs;VtDoubleArray gains;TfToken kernel("gaussian");float regularization=0;
    if(!Constant(source,property("rigExec:ikValue"),&c.ikValue,error) ||
       !Constant(source,property("inputs:gain"),&c.gain,error) ||
       !Constant(source,property("rigExec:basis"),&basis,error) ||
       !Constant(source,property("rigExec:poseRotations"),&rotations,error) ||
       !Constant(source,property("rigExec:poseFalloffs"),&falloffs,error) ||
       !Constant(source,property("rigExec:poseGains"),&gains,error) ||
       !Constant(source,property("rigExec:kernel"),&kernel,error) ||
       !Constant(source,property("rigExec:regularization"),&regularization,error))return false;
    for(int r=0;r<3;++r)for(int k=0;k<3;++k)c.basis[r*3+k]=basis[r][k];
    if(falloffs.size()!=rotations.size() || gains.size()!=rotations.size())
        return Fail(error,path.GetString()+" pose rotations, falloffs and gains must have equal counts");
    if(!rotations.empty()) {
        RigExecRbfSolverDesc desc;desc.kernel=kernel=="linear"?RigExecRbfKernel::Linear:RigExecRbfKernel::Gaussian;
        desc.regularization=regularization;desc.twistAxis=GfVec3d(1,0,0);
        for(size_t i=0;i<rotations.size();++i) {
            desc.poses.push_back(RigExecRbfEulerFromQuaternion(GfQuatd(rotations[i].GetReal(),GfVec3d(rotations[i].GetImaginary()))));
            desc.falloffs.push_back(double(falloffs[i]));desc.poseTypes.push_back(RigExecRbfPoseType::Swing);
        }
        RigExecRbfSolver solver(desc);
        if(!solver.Solve() || solver.GetWeights().size()!=rotations.size())return Fail(error,path.GetString()+" has unsolvable swing poses");
        c.kernel=desc.kernel==RigExecRbfKernel::Linear?1:0;c.normalize=desc.normalize;
        const auto &radii=solver.GetRadii();
        for(size_t i=0;i<rotations.size();++i) {
            GfQuatd swing,twist;RigExecRbfSwingTwist(RigExecRbfQuaternionFromEuler(solver.GetPoses()[i]),desc.twistAxis,&swing,&twist);
            c.swings.push_back(swing.GetReal());for(int k=0;k<3;++k)c.swings.push_back(swing.GetImaginary()[k]);
            c.widths.push_back(i<radii.size()?radii[i]:solver.GetRadius());c.gains.push_back(gains[i]);
            for(double w:solver.GetWeights()[i])c.weights.push_back(w);
        }
    }
    const auto scalar=[&](const SdfPath &attribute,size_t index) {
        if(attribute.IsEmpty())return true;
        const auto *fact=source.Attribute(attribute);
        if(!fact || (fact->fact.type!=SdfValueTypeNames->Float && fact->fact.type!=SdfValueTypeNames->Double))
            return Fail(error,"auto clavicle input requires a float or double: "+attribute.GetString());
        RigExecSceneTypedRead read;
        if(!RigExecBindSceneTypedRead(scene,attribute,fact->fact.type,&read,error) ||
           !RigExecBindGraphTypedRead(read,context,&result.scalars[index],error))return false;
        const auto &bound=result.scalars[index];
        if(bound.effective!=UINT64_MAX)result.reads.push_back(bound.effective);
        for(const auto *hops:{&bound.hops,&bound.doubleHops})for(const auto &hop:*hops)
            for(auto id:{hop.raw,hop.overlay})if(id!=UINT64_MAX)result.reads.push_back(id);
        return true;
    };
    if(!scalar(blend,0) || !scalar(amount,1))return false;
    if(!ik.IsEmpty() && !pole.IsEmpty())for(const auto &[owner,node]:scene.nodes) {
        if(!node.fact.active || node.fact.type!="RigExecTwoBoneIk")continue;
        const auto prop=[&](const char *name){return owner.AppendProperty(TfToken(name));};
        if(source.Targets(prop("rigExec:effectorControl"))!=SdfPathVector{ik} ||
           source.Targets(prop("rigExec:poleControl"))!=SdfPathVector{pole})continue;
        TfToken policy;if(!Constant(source,prop("rigExec:stretchPolicy"),&policy,error))return false;
        const auto joints=source.Targets(prop("rigExec:joints"));
        if(policy!="softDistance" || joints.size()!=3)continue;
        result.hasLimb=true;
        for(size_t i=0;i<3;++i)if(!frame(joints[i],"computeRestFrame",10+i))return false;
        const char *names[]={"inputs:stretch","inputs:pin","inputs:upperScale","inputs:lowerScale","inputs:softDistance",
            "rigExec:scaleCalibration","inputs:twist","rigExec:upperLengthOffset","rigExec:lowerLengthOffset"};
        for(size_t i=0;i<9;++i)if(!scalar(prop(names[i]),i+2))return false;
        break;
    }
    std::sort(result.reads.begin(),result.reads.end());result.reads.erase(std::unique(result.reads.begin(),result.reads.end()),result.reads.end());
    *destination=std::move(result);return true;
}
bool RigExecRunAutoClavicle(const RigExecBoundAutoClavicle &op,RigExecTypedValueStore *values,std::string *error) {
    if(!values)return Fail(error,"missing auto clavicle values");
    std::array<GfMatrix4d,13> matrices;bool usable=true;
    for(size_t i=0;i<op.frames.size();++i) {
        if(op.frames[i]==UINT64_MAX)continue;
        const auto *frame=values->Read<RigExecPointFrame>(op.frames[i]);
        usable=usable && frame && frame->IsValid() && !frame->IsDegenerate() &&
            RigExecPointsToMatrix(RigExecIdentityLandmarks(),frame->points,&matrices[i]);
    }
    if(!usable) {
        const auto &source=values->values[size_t(op.frames[0])];
        const auto *frame=std::get_if<RigExecPointFrame>(&source.value);
        if(frame)values->Publish(op.output,*frame,source.blocked,false,source.count,op.unavailable);
        else values->Publish(op.output,std::monostate(),true,false,0,op.unavailable);
        return true;
    }
    const auto scalar=[&](size_t index,double fallback) {
        VtValue value;if(!RigExecReadGraphTypedRead(op.scalars[index],*values,&value,nullptr))return fallback;
        if(value.IsHolding<double>())return value.UncheckedGet<double>();
        if(value.IsHolding<float>())return double(value.UncheckedGet<float>());return fallback;
    };
    RigExecAutoClavicleFrames frames;
    frames.targetPosed=matrices[0].data();frames.pivotPosed=matrices[1].data();
    frames.anchorPosed=matrices[2].data();frames.anchorDefault=matrices[3].data();frames.fkPosed=matrices[4].data();
    for(size_t i=0;i<3;++i)frames.fkDefault[i]=matrices[5+i].data();
    if(op.frames[8]!=UINT64_MAX)frames.ikTargetPosed=matrices[8].data();
    if(op.frames[9]!=UINT64_MAX)frames.polePosed=matrices[9].data();
    frames.ikBlend=scalar(0,1-op.constants.ikValue);frames.amount=scalar(1,1);
    frames.hasLimb=op.hasLimb;
    if(op.hasLimb) {
        frames.limb.stretch=scalar(2,1);frames.limb.pin=scalar(3,0);
        frames.limb.upperScale=scalar(4,1);frames.limb.lowerScale=scalar(5,1);
        frames.limb.softDistance=scalar(6,0);frames.limb.scaleCalibration=scalar(7,0);
        frames.twistRadians=scalar(8,0)*std::acos(-1.0)/180;
        frames.limbRestUpper=(matrices[11].ExtractTranslation()-matrices[10].ExtractTranslation()).GetLength()+scalar(9,0);
        frames.limbRestLower=(matrices[12].ExtractTranslation()-matrices[11].ExtractTranslation()).GetLength()+scalar(10,0);
    }
    double delta[3];RigExecAutoClavicleShift(op.constants,frames,delta);
    if(delta[0]==0 && delta[1]==0 && delta[2]==0)values->Copy(op.output,op.frames[0],false);
    else {
        auto shifted=matrices[0];shifted.SetTranslateOnly(shifted.ExtractTranslation()+GfVec3d(delta[0],delta[1],delta[2]));
        values->Publish(op.output,RigExecFrameFromMatrix(shifted));
    }
    return true;
}
}
