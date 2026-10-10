// Numerical contracts and primary sources: docs/references.md.
#include "affineFrameKernels.h"
#include "dualQuat.h"
#include "pxr/base/gf/rotation.h"
#include "pxr/base/gf/quatd.h"
#include "pxr/base/gf/matrix3d.h"
#include <algorithm>
#include <cmath>
#include <stdexcept>
namespace rigExec {
namespace {
GfMatrix4d Channels(const RigExecAffineFrameInputs &in) {
    GfMatrix4d local(1);
    local.SetScale(GfVec3d(in.sx,
                         in.sy,in.sz));
    const double rotations[]={in.rx,in.ry,in.rz};
    for(int i=0;i<3;++i) {
        GfVec3d axis(0);axis[i]=1;
        local*=GfMatrix4d(GfRotation(axis,rotations[i]),GfVec3d(0));
    }
    local.SetTranslateOnly(GfVec3d(in.tx,
                                 in.ty,in.tz));
    return local;
}
GfMatrix4d ArmatureParent(const RigExecAffineFrameInputs &in) {
    GfMatrix4d incoming = in.useIncoming
        ? in.incoming
        : Channels(in)*in.local*
          in.parent;
    // The armature constraint applies the target's rest-to-pose map
    // over the incoming owner frame, preserving the owner's local channels.
    auto result=incoming*
        in.sourceObject.GetInverse()*
        in.inverseBind*
        in.source;
    if(in.preserveLocation) result.SetTranslateOnly(incoming.ExtractTranslation());
    return result;
}
GfVec3d Row(const GfMatrix4d &m,int i) {return GfVec3d(m[i][0],m[i][1],m[i][2]);}
void SetRow(GfMatrix4d &m,int i,const GfVec3d &v) {for(int j=0;j<3;++j)m[i][j]=v[j];}
GfVec3d Sizes(const GfMatrix4d &m) {return GfVec3d(Row(m,0).GetLength(),Row(m,1).GetLength(),Row(m,2).GetLength());}
GfVec3d VolumeSizes(const GfMatrix4d &m) {
    auto sizes=Sizes(m);double product=sizes[0]*sizes[1]*sizes[2];
    return product>0 ? sizes*std::cbrt(std::abs(m.GetDeterminant())/product) : sizes;
}
void Rescale(GfMatrix4d &m,const GfVec3d &s) {for(int i=0;i<3;++i)SetRow(m,i,Row(m,i)*s[i]);}
void NormalizeRows(GfMatrix4d &m) {for(int i=0;i<3;++i){auto v=Row(m,i);v.Normalize();SetRow(m,i,v);}}
// Symmetric orthogonalization around the bone's Y axis: project X/Z, then
// split their angular error equally. Preserving area retains volume under
// shear. This is deliberately different from a generic polar matrix split.
void Orthogonalize(GfMatrix4d &m,bool normalize) {
    auto y=Row(m,1),x=Row(m,0),z=Row(m,2);
    double y2=y.GetLengthSq();
    if(y2>0) {x-=y*(GfDot(x,y)/y2);z-=y*(GfDot(z,y)/y2);if(normalize)y/=std::sqrt(y2);}
    double lx=x.Normalize(),lz=z.Normalize();
    double cosine=std::clamp(GfDot(x,z),-1.0,1.0);
    if(std::abs(cosine)>1e-4 && std::abs(cosine)<1-1.1920928955078125e-7) {
        auto sum=x+z,difference=x-z;sum.Normalize();difference.Normalize();
        x=(sum+difference)/std::sqrt(2.0);z=(sum-difference)/std::sqrt(2.0);
        double area=std::sqrt(std::sqrt(std::max(0.0,1-cosine*cosine)));
        lx*=area;lz*=area;
    }
    SetRow(m,0,x*(normalize?1:lx));SetRow(m,1,y);SetRow(m,2,z*(normalize?1:lz));
}
struct ParentSpaces {
    GfMatrix4d rotationScale,location;
    GfVec3d post;
    GfMatrix4d Apply(const GfMatrix4d &input,bool inverse=false) const {
        auto rotation=inverse?rotationScale.GetInverse():rotationScale;
        auto position=inverse?location.GetInverse():location;
        auto output=input*rotation;
        output.SetTranslateOnly(position.Transform(input.ExtractTranslation()));
        Rescale(output,inverse?GfVec3d(1/post[0],1/post[1],1/post[2]):post);
        return output;
    }
};
ParentSpaces BoneParentSpaces(const GfMatrix4d &rest,const GfMatrix4d &parentRest,
    const GfMatrix4d &parent,bool hasParent,const std::string &mode,bool inheritRotation,bool localLocation) {
    GfMatrix4d rotationScale=rest,location=rest;GfVec3d post(1);
    if(hasParent) {
        GfMatrix4d adjusted=parent;
        if(inheritRotation) {
            if(mode=="NONE" || mode=="AVERAGE") Orthogonalize(adjusted,true);
            else if(mode=="ALIGNED") {Orthogonalize(adjusted,false);post=Sizes(adjusted);NormalizeRows(adjusted);}
            else if(mode=="NONE_LEGACY") NormalizeRows(adjusted);
        } else {
            adjusted=parentRest;
            if(mode=="FULL") Rescale(adjusted,Sizes(parent));
            else if(mode=="FIX_SHEAR" || mode=="ALIGNED") {
                if(mode=="ALIGNED") post=VolumeSizes(parent);else Rescale(adjusted,VolumeSizes(parent));
            }
        }
        if(mode=="AVERAGE") Rescale(adjusted,GfVec3d(std::cbrt(std::abs(parent.GetDeterminant()))));
        rotationScale=rest*adjusted;
        if(mode=="FIX_SHEAR") Orthogonalize(rotationScale,false);
        location=rest*parent;
        if(!localLocation) {location=parent;location.SetTranslateOnly(parent.Transform(rest.ExtractTranslation()));}
    } else if(!localLocation) location.SetTranslate(rest.ExtractTranslation());
    return {rotationScale,location,post};
}
GfMatrix4d BoneFrame(const RigExecAffineFrameInputs &in) {
    auto object=in.object;
    auto channels=Channels(in);
    if(in.connected) channels.SetTranslateOnly(GfVec3d(0));
    auto spaces=BoneParentSpaces(in.local,
        in.parentRest,
        in.parent*object.GetInverse(),
        in.hasParent,in.inheritScale,
        in.inheritRotation,in.localLocation);
    const auto kind=in.spaceKind;
    if(kind=="rotation") return spaces.rotationScale*object;
    if(kind=="translation") return spaces.location*object;
    return spaces.Apply(channels)*object;
}
ParentSpaces ConstraintParentSpaces(const RigExecAffineFrameInputs &in,bool source,const GfMatrix4d &object) {
    return BoneParentSpaces((source?in.sourceLocal:in.ownerLocal),
        (source?in.sourceParentRest:in.ownerParentRest),
        (source?in.sourceParent:in.ownerParent)*object.GetInverse(),
        (source?in.sourceHasParent:in.ownerHasParent),
        (source?in.sourceInheritScale:in.ownerInheritScale),
        (source?in.sourceInheritRotation:in.ownerInheritRotation),
        (source?in.sourceLocalLocation:in.ownerLocalLocation));
}
GfMatrix4d SkinInfluence(const RigExecAffineFrameInputs &in) {
    GfMatrix4d prefix(1);
    if(in.fromBind)
        prefix=in.inverseMesh*
            in.owner;
    if(in.followOnly) return prefix;
    return prefix*in.sourceObject.GetInverse()*
        in.inverseBind*
        in.source;
}
GfMatrix4d MappedFrame(const RigExecAffineFrameInputs &in) {
    return in.targetRest*
        in.sourceRest.GetInverse()*
        in.source;
}
GfMatrix4d CopyTransforms(const RigExecAffineFrameInputs &in) {
    auto result=in.source;
    if(in.preserveLocation)
        result.SetTranslateOnly(in.incoming.ExtractTranslation());
    return result;
}
void Track(GfMatrix4d &m,GfVec3d direction,const std::string &axisToken) {
    // Tracking direction and axis-angle use float precision. Retain
    // that boundary: near a half-turn, rounding after the cross product
    // instead of before it changes the chosen rotation axis appreciably.
    GfVec3f targetDirection(direction);
    if(targetDirection.Normalize()==0) return;
    int axis=axisToken.back()-'X';bool negative=axisToken.find("NEGATIVE")!=std::string::npos;
    GfVec3f original(Row(m,axis)*(negative?-1:1));
    if(original.Normalize()==0) {original=GfVec3f(0);original[axis]=negative?-1:1;}
    GfVec3f perpendicular(GfCross(GfVec3d(original),GfVec3d(targetDirection)));
    float sine=perpendicular.Normalize();
    float cosine=std::clamp(GfDot(original,targetDirection),-1.0f,1.0f);
    float angle=std::acos(cosine);
    if(sine<1.1920928955078125e-7) {
        if(angle<3.14159265358979323846-0.01) return;
        // At a half-turn the next local track direction resolves the ambiguity.
        auto next=Row(m,(axis+1)%3)*(axis==2?(negative?1:-1):(negative?-1:1));
        perpendicular=GfVec3f(GfCross(GfVec3d(original),next));if(perpendicular.Normalize()==0)return;
        angle=3.14159265358979323846;
    } else if(sine<0.1f) angle=cosine<0 ? float(3.14159265358979323846)-std::asin(sine) : std::asin(sine);
    auto rotation=GfMatrix4d(GfRotation(GfVec3d(perpendicular),angle*180/3.14159265358979323846),GfVec3d(0));
    auto origin=m.ExtractTranslation();m*=rotation;m.SetTranslateOnly(origin);
}
GfMatrix4d ArmatureBlend(const RigExecAffineFrameInputs &in,const GfMatrix4d &incoming) {
    const auto &binds=in.targetBinds;
    const auto &weights=in.targetWeights;
    const auto &targetIndices=in.targetIndices;
    const auto &objectIndices=in.objectIndices;
    const auto &targets=in.targets, &objects=in.targetObjects;
    if(binds.size()!=weights.size() || targetIndices.size()!=weights.size() || objectIndices.size()!=weights.size())
        throw std::runtime_error("Armature constraint target array mismatch");
    auto pivot=in.currentPivot?incoming.ExtractTranslation():
        in.ownerObject.Transform(in.pivot);
    bool dq=in.dualQuaternion;
    GfMatrix4d matrix(0.0),stretchSum(0.0);
    rigExec::RigExecDualQuat sum(GfQuatd(0),GfQuatd(0));
    double total=0;
    for(size_t i=0;i<weights.size();++i) {
        double weight=weights[i];if(weight==0)continue;total+=weight;
        int ti=targetIndices[i],oi=objectIndices[i];
        if(ti<0 || oi<0 || size_t(ti)>=targets.size() || size_t(oi)>=objects.size())throw std::runtime_error("invalid Armature target index");
        const auto delta=objects[oi].GetInverse()*binds[i]*targets[ti];
        if(!dq) {matrix+=delta*weight;continue;}
        // The scale gauge follows the rest bone's Y axis. A generic
        // polar split differs for stretched bones. Retain the residual as
        // an affine matrix, and move its pivot displacement into the rigid
        // part before blending. See docs/references.md.
        auto base=binds[i].GetInverse()*objects[oi];Orthogonalize(base,true);
        auto posed=base*delta;
        // The posed gauge uses sequential Y/X orthogonalization,
        // whereas the rest gauge above uses its symmetric stable rule.
        auto y=Row(posed,1);y.Normalize();
        auto z=GfCross(Row(posed,0),y);z.Normalize();
        auto x=GfCross(y,z);x.Normalize();
        SetRow(posed,0,x);SetRow(posed,1,y);SetRow(posed,2,z);
        auto rigid=base.GetInverse()*posed;
        auto stretch=delta*rigid.GetInverse();
        auto shift=stretch.Transform(pivot)-pivot;
        rigid.SetTranslateOnly(rigid.ExtractTranslation()+rigid.TransformDir(shift));
        stretch.SetTranslateOnly(pivot-stretch.TransformDir(pivot));
        auto value=rigExec::RigExecDualQuatFromMatrix(rigid);
        // Source order matters: correct against the accumulated
        // rotation, rather than choosing a fixed reference target.
        double signedWeight=GfDot(sum.real,value.real)<0?-weight:weight;
        sum.real+=value.real*signedWeight;sum.dual+=value.dual*signedWeight;
        stretchSum+=stretch*weight;
    }
    if(total<=0)return incoming;
    if(dq) {
        auto normalized=sum;
        if(!rigExec::RigExecDualQuatNormalize(&normalized))return incoming;
        matrix=(stretchSum*(1.0/total))*rigExec::RigExecDualQuatToMatrix(normalized);
    } else matrix*=1.0/total;
    return incoming*matrix;
}
GfVec3d EulerXYZ(GfMatrix4d matrix) {
    NormalizeRows(matrix);
    const double cy=std::hypot(matrix[0][0],matrix[0][1]);
    GfVec3d first(std::atan2(matrix[1][2],matrix[2][2]),std::atan2(-matrix[0][2],cy),std::atan2(matrix[0][1],matrix[0][0]));
    if(cy<=0.0000375)return GfVec3d(std::atan2(-matrix[2][1],matrix[1][1]),first[1],0);
    const GfVec3d second(std::atan2(-matrix[1][2],-matrix[2][2]),std::atan2(-matrix[0][2],-cy),std::atan2(-matrix[0][1],-matrix[0][0]));
    auto magnitude=[](const GfVec3d &v){return std::abs(v[0])+std::abs(v[1])+std::abs(v[2]);};
    return magnitude(first)<=magnitude(second)?first:second;
}
GfMatrix4d ConstraintValue(const RigExecAffineFrameInputs &in) {
    auto result=in.incoming;
    auto operation=in.operation;
    if(operation=="PRESERVE_ORIGIN") {
        result.SetTranslateOnly(in.origin.ExtractTranslation());return result;
    }
    if(operation=="LIMIT_ROTATION") {Orthogonalize(result,false);return result;}
    if(operation=="ARMATURE_BLEND")return ArmatureBlend(in,result);
    auto source=in.source;
    auto ownerSpace=in.ownerSpace;
    auto targetSpace=in.targetSpace;
    auto ownerObject=in.ownerObject;
    auto sourceObject=in.sourceObject;
    auto customSpace=in.customSpace;
    bool localTarget=targetSpace=="LOCAL" || targetSpace=="LOCAL_OWNER_ORIENT";
    if(localTarget)source.SetTranslateOnly(source.Transform(in.targetOffset));
    ParentSpaces ownerLocal;
    if(ownerSpace=="CUSTOM")result*=customSpace.GetInverse();
    else if(ownerSpace!="WORLD") {
        result*=ownerObject.GetInverse();
        if(ownerSpace=="LOCAL") {
            ownerLocal=ConstraintParentSpaces(in,false,ownerObject);
            result=ownerLocal.Apply(result,true);
        }
    }
    if(targetSpace=="CUSTOM")source*=customSpace.GetInverse();
    else if(targetSpace!="WORLD") {
        source*=sourceObject.GetInverse();
        if(targetSpace=="LOCAL" || targetSpace=="LOCAL_OWNER_ORIENT") {
            source=ConstraintParentSpaces(in,true,sourceObject).Apply(source,true);
            if(targetSpace=="LOCAL_OWNER_ORIENT") {
                auto difference=in.sourceRest*
                    in.ownerRest.GetInverse();
                difference.SetTranslateOnly(GfVec3d(0));
                source=difference.GetInverse()*source*difference;
            }
        }
    }
    auto world=[&](const GfMatrix4d &matrix){
        if(ownerSpace=="WORLD")return matrix;
        if(ownerSpace=="CUSTOM")return matrix*customSpace;
        return (ownerSpace=="LOCAL"?ownerLocal.Apply(matrix):matrix)*ownerObject;
    };
    auto target=localTarget?source.ExtractTranslation():source.Transform(in.targetOffset);
    if(operation=="TRANSFORM_LOCATION") {
        const auto from=in.mapFrom;
        GfVec3d channels=source.ExtractTranslation();
        if(from=="ROTATION")channels=EulerXYZ(source);
        else if(from=="SCALE")channels=Sizes(source)*(source.GetDeterminant()<0?-1.0:1.0);
        const auto minimum=in.mapFromMin,maximum=in.mapFromMax;
        const auto outputMin=in.mapToMin,outputMax=in.mapToMax;
        const auto axes=in.mapAxes;
        GfVec3d normalized(0),location(0);
        for(int i=0;i<3;++i) {
            if(minimum[i]>maximum[i] || axes[i]<0 || axes[i]>2)return in.incoming;
            const double channel=in.mapExtrapolate?channels[i]:std::clamp(channels[i],minimum[i],maximum[i]);
            if(maximum[i]!=minimum[i])normalized[i]=(channel-minimum[i])/(maximum[i]-minimum[i]);
        }
        for(int i=0;i<3;++i)location[i]=outputMin[i]+normalized[axes[i]]*(outputMax[i]-outputMin[i]);
        if(in.mapMix=="ADD")location+=result.ExtractTranslation();
        result.SetTranslateOnly(location);return world(result);
    }
    if(operation=="COPY_TRANSFORMS") {
        if(in.removeTargetShear)Orthogonalize(source,false);
        auto mix=in.rotationMix;
        if(mix=="REPLACE")return world(source);
        bool before=mix.find("BEFORE")==0;
        if(mix=="BEFORE_FULL" || mix=="AFTER_FULL")return world(before?result*source:source*result);
        auto location=mix.find("SPLIT")!=std::string::npos?result.ExtractTranslation()+source.ExtractTranslation():
            before?source.Transform(result.ExtractTranslation()):result.Transform(source.ExtractTranslation());
        auto size=GfCompMult(Sizes(result),Sizes(source));
        auto a=result,b=source;NormalizeRows(a);NormalizeRows(b);
        if(a.GetDeterminant()<0) {Rescale(a,GfVec3d(-1));size=-size;}
        if(b.GetDeterminant()<0) {Rescale(b,GfVec3d(-1));size=-size;}
        a.SetTranslateOnly(GfVec3d(0));b.SetTranslateOnly(GfVec3d(0));
        auto combined=before?a*b:b*a;Rescale(combined,size);combined.SetTranslateOnly(location);
        return world(combined);
    }
    if(operation=="ARMATURE")
        return result*in.sourceObject.GetInverse()*
            in.inverseBind*source;
    if(operation=="COPY_LOCATION") {
        auto origin=result.ExtractTranslation();int axes=in.axisMask,invert=in.invertMask;
        for(int i=0;i<3;++i)if(axes&(1<<i))origin[i]=(invert&(1<<i)?-target[i]:target[i])+(in.offset?origin[i]:0);
        result.SetTranslateOnly(origin);return world(result);
    }
    if(operation=="COPY_SCALE") {
        auto size=Sizes(source),original=Sizes(result);
        int axes=in.axisMask;
        bool uniform=in.uniformScale;
        if(uniform) {
            double product=axes==7?std::abs(source.GetDeterminant()):1;
            if(axes!=7)for(int i=0;i<3;++i)if(axes&(1<<i))product*=size[i];
            size=GfVec3d(std::cbrt(product));
        }
        for(int i=0;i<3;++i) {
            size[i]=std::pow(size[i],in.power);
            if(in.offset)
                size[i]=in.scaleAdd?size[i]+original[i]-1:size[i]*original[i];
            if((uniform || (axes&(1<<i))) && original[i]!=0) SetRow(result,i,Row(result,i)*(size[i]/original[i]));
        }
        return world(result);
    }
    if(operation=="COPY_ROTATION") {
        auto size=Sizes(result),origin=result.ExtractTranslation();
        if(in.axisMask==0)return world(result);
        auto mix=in.rotationMix;
        Orthogonalize(source,true);
        source.SetTranslateOnly(GfVec3d(0));
        if(mix!="REPLACE") {
            auto old=result;NormalizeRows(old);old.SetTranslateOnly(GfVec3d(0));
            if(old.GetDeterminant()<0) {Rescale(old,GfVec3d(-1));size=-size;}
            source=mix=="BEFORE"?old*source:source*old;
        }
        Rescale(source,size);source.SetTranslateOnly(origin);
        return world(source);
    }
    auto direction=target-result.ExtractTranslation();
    if(operation=="DAMPED_TRACK") {
        direction=GfVec3d(GfVec3f(target)-GfVec3f(result.ExtractTranslation()));
        Track(result,direction,in.trackAxis);return result;
    }
    // RigExec point-frame providers require invertible transforms. A zero
    // length stretch collapses the frame; retain the incoming frame
    // in this unsupported case rather than poisoning all downstream skinning.
    double length=in.restLength;
    if(direction.GetLength()<1e-6*std::max(1.0,length) || Row(result,1).GetLength()<1e-12) return result;
    auto keep=in.keepAxis;
    if(keep=="SWING_Y") Orthogonalize(result,false);
    auto size=Sizes(result);NormalizeRows(result);
    double distance=direction.Normalize();distance=size[1]!=0?distance/size[1]:0;
    double stretch=distance/length;
    auto volume=in.volume;
    double bulge=volume=="NO_VOLUME"?1:std::pow(length/std::max(distance,1e-30),in.bulge);
    double smooth=in.bulgeSmooth;
    if(bulge>1 && in.useBulgeMax) {
        double bound=std::max(1.0,in.bulgeMax),range=bound-1;
        double soft=range>0?1+range*std::atan((bulge-1)/range)*2/3.14159265358979323846:1;
        bulge=(1-smooth)*std::min(bulge,bound)+smooth*soft;
    }
    if(bulge<1 && in.useBulgeMin) {
        double bound=std::clamp(in.bulgeMin,0.0,1.0),range=1-bound;
        double soft=range>0?1-range*std::atan((1-bulge)/range)*2/3.14159265358979323846:1;
        bulge=(1-smooth)*std::max(bulge,bound)+smooth*soft;
    }
    GfVec3d factor(1,stretch,1);
    if(volume=="VOLUME_XZX") factor[0]=factor[2]=std::sqrt(bulge);
    else if(volume=="VOLUME_X")factor[0]=bulge;
    else if(volume=="VOLUME_Z")factor[2]=bulge;
    if(keep=="SWING_Y") Track(result,direction,"TRACK_Y");
    else {
        auto reference=Row(result,keep=="PLANE_X"?0:2);
        auto perpendicular=GfCross(reference,direction);perpendicular.Normalize();
        SetRow(result,1,direction);
        if(keep=="PLANE_X") {SetRow(result,2,perpendicular);auto x=GfCross(direction,perpendicular);x.Normalize();SetRow(result,0,x);}
        else {SetRow(result,0,-perpendicular);auto z=GfCross(direction,perpendicular);z.Normalize();SetRow(result,2,z);}
    }
    Rescale(result,GfVec3d(size[0]*factor[0],size[1]*factor[1],size[2]*factor[2]));
    return result;
}
void PolarFrame(const GfMatrix4d &matrix,GfMatrix3d *rotation,GfMatrix3d *stretch) {
    GfMatrix3d linear(matrix[0][0],matrix[0][1],matrix[0][2],
                      matrix[1][0],matrix[1][1],matrix[1][2],
                      matrix[2][0],matrix[2][1],matrix[2][2]);
    auto orthogonal=linear;
    // Newton's polar iteration for invertible point-frame inputs.
    // In USD's row convention the
    // symmetric stretch is on the left: linear = stretch * rotation.
    for(int iteration=0;iteration<64;++iteration) {
        auto next=(orthogonal+orthogonal.GetInverse().GetTranspose())*0.5;
        double difference=0;
        for(int i=0;i<3;++i)for(int j=0;j<3;++j)
            difference=std::max(difference,std::abs(next[i][j]-orthogonal[i][j]));
        orthogonal=next;if(difference<1e-12)break;
    }
    *rotation=orthogonal;
    *stretch=linear*orthogonal.GetTranspose();
    // Quaternions require a proper rotation; retain a reflection in stretch.
    if(rotation->GetDeterminant()<0) {*rotation*=-1;*stretch*=-1;}
}
GfMatrix4d BlendFrame(GfMatrix4d before,GfMatrix4d after,double weight) {
    auto location=(1-weight)*before.ExtractTranslation()+weight*after.ExtractTranslation();
    GfMatrix3d rotation0,rotation1,stretch0,stretch1;
    PolarFrame(before,&rotation0,&stretch0);PolarFrame(after,&rotation1,&stretch1);
    auto q0=rotation0.ExtractRotation().GetQuat(),q1=rotation1.ExtractRotation().GetQuat();
    double cosine=GfDot(q0,q1);if(cosine<0){q0=-q0;cosine=-cosine;}
    double w0=1-weight,w1=weight;
    if(cosine<1-1e-4) {
        double angle=std::acos(std::clamp(cosine,-1.0,1.0)),denominator=std::sin(angle);
        w0=std::sin((1-weight)*angle)/denominator;w1=std::sin(weight*angle)/denominator;
    }
    auto linear=((1-weight)*stretch0+weight*stretch1)*GfMatrix3d(w0*q0+w1*q1);
    GfMatrix4d result(1);
    for(int i=0;i<3;++i)for(int j=0;j<3;++j)result[i][j]=linear[i][j];
    result.SetTranslateOnly(location);
    return result;
}
GfMatrix4d ConstraintFrame(const RigExecAffineFrameInputs &in) {
    auto value=ConstraintValue(in);
    double influence=in.influence;
    if(influence==1)return value;
    auto before=in.incoming;
    // Return the solution to world space before blending influence.
    return BlendFrame(before,value,influence);
}
}
GfMatrix4d RigExecComputeArmatureParent(const RigExecAffineFrameInputs &inputs) {return ArmatureParent(inputs);}
GfMatrix4d RigExecComputeBoneFrame(const RigExecAffineFrameInputs &inputs) {return BoneFrame(inputs);}
GfMatrix4d RigExecComputeSkinInfluence(const RigExecAffineFrameInputs &inputs) {return SkinInfluence(inputs);}
GfMatrix4d RigExecComputeMappedFrame(const RigExecAffineFrameInputs &inputs) {return MappedFrame(inputs);}
GfMatrix4d RigExecComputeCopyTransforms(const RigExecAffineFrameInputs &inputs) {return CopyTransforms(inputs);}
GfMatrix4d RigExecComputeConstraintFrame(const RigExecAffineFrameInputs &inputs) {return ConstraintFrame(inputs);}
}
