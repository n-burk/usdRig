#ifndef RIGEXEC_GRAPH_PROVIDER_ARITHMETIC_H
#define RIGEXEC_GRAPH_PROVIDER_ARITHMETIC_H
#include "providerRecords.h"
#include "poseArithmetic.h"
#include "rigExecMath/avarScale.h"
#include <cmath>
#include <limits>
#include <string_view>
namespace rigExec {
/// Math supplies Matrix/Vector/Rotation/Frame and PointsToMatrix. Gf and the
/// zero-USD mirror use this exact arithmetic and matrix multiplication order.
template<class Math> typename Math::Matrix RigExecProviderComposeAvars(
    double tx,double ty,double tz,double sx,double sy,double sz,
    double rx,double ry,double rz,double spin,std::string_view order) {
    using M=typename Math::Matrix;using V=typename Math::Vector;using R=typename Math::Rotation;
    const V axes[3]={V(1,0,0),V(0,1,0),V(0,0,1)};
    const double angles[3]={rx,ry,rz};
    const std::string_view sequence=order.size()==3?order:std::string_view("XYZ");
    M matrix(1.0);
    matrix.SetScale(V(RigExecNormalizeAvarScale(sx),RigExecNormalizeAvarScale(sy),RigExecNormalizeAvarScale(sz)));
    for(char axis:sequence) {
        const int i=axis=='X'?0:axis=='Y'?1:2;
        if(angles[i]!=0.0)matrix=matrix*M(R(axes[i],angles[i]),V(0));
    }
    if(spin!=0.0)matrix=matrix*M(R(axes[0],spin),V(0));
    M translation(1.0);translation.SetTranslate(V(tx,ty,tz));
    return matrix*translation;
}
template<class Math> typename Math::Frame RigExecProviderFrameFromMatrix(const typename Math::Matrix &matrix) {
    using V=typename Math::Vector;
    const std::array<V,4> identity={V(0),V(1,0,0),V(0,1,0),V(0,0,1)};
    typename Math::Frame frame;
    for(size_t i=0;i<4;++i) {
        frame.points[i]=matrix.TransformAffine(identity[i]);
        for(int a=0;a<3;++a)if(!std::isfinite(frame.points[i][a])) {frame.flags=2u;return frame;}
    }
    frame.flags=1u;return frame;
}
template<class Math> typename Math::Matrix RigExecProviderSpaceFromFrame(const typename Math::Frame *frame) {
    using V=typename Math::Vector;using M=typename Math::Matrix;
    const std::array<V,4> identity={V(0),V(1,0,0),V(0,1,0),V(0,0,1)};
    M result(1.0);
    if(frame && (!frame->IsValid() || frame->IsDegenerate() ||
        !Math::PointsToMatrix(identity,frame->points,&result)))result[3][0]=std::numeric_limits<double>::quiet_NaN();
    return result;
}
/// Inputs are unit scale, translation3, scale3, rotation3, spin, order, sign.
template<class Math,class Op,class Store>
typename Math::Matrix RigExecProviderReadAvarMatrix(const Op &op,Store &store,size_t first) {
    using V=typename Math::Vector;
    const auto id=[&](size_t offset){return first+offset<op.inputs.size()?uint64_t(op.inputs[first+offset]):UINT64_MAX;};
    const auto scalar=[&](size_t offset,double fallback){const auto *v=store.ReadScalar(id(offset));return v?*v:fallback;};
    const auto *signValue=store.ReadVector(id(12));
    const V sign=signValue?V(RigExecNormalizeRotationSign((*signValue)[0]),
        RigExecNormalizeRotationSign((*signValue)[1]),RigExecNormalizeRotationSign((*signValue)[2])):V(1,1,1);
    const double units=scalar(0,1);
    return RigExecProviderComposeAvars<Math>(scalar(1,0)*units,scalar(2,0)*units,scalar(3,0)*units,
        op.scaleAvars?scalar(4,1):1,op.scaleAvars?scalar(5,1):1,op.scaleAvars?scalar(6,1):1,
        scalar(7,0)*sign[0],scalar(8,0)*sign[1],scalar(9,0)*sign[2],scalar(10,0)*sign[0],store.ReadToken(id(11)));
}
/// Store supplies ReadMatrix/ReadFrame/ReadScalar/ReadVector/ReadToken,
/// Present/Authoritative, Copy, Invalidate(output,source), PublishMatrix and PublishFrame. It retains
/// validity/change/count/error state independently from this pure arithmetic.
template<class Math,class Op,class Store> bool RigExecRunProviderArithmetic(const Op &op,Store &store) {
    using M=typename Math::Matrix;using V=typename Math::Vector;
    const auto id=[&](size_t input){return input<op.inputs.size()?uint64_t(op.inputs[input]):UINT64_MAX;};
    const auto matrix=[&](size_t input){const auto *m=store.ReadMatrix(id(input));return m?*m:M(1.0);};
    const auto scalar=[&](size_t input,double fallback){const auto *v=store.ReadScalar(id(input));return v?*v:fallback;};
    const auto frame=[&](size_t input){return store.ReadFrame(id(input));};
    switch(op.kind) {
    case RigExecProviderOpKind::Attribute: {
        const uint64_t selected=store.Authoritative(id(0)) || id(1)==UINT64_MAX?id(0):id(1);
        if(!store.Present(selected)) { store.Invalidate(op.output,selected);break; }
        store.Copy(op.output,selected,!store.Authoritative(id(0)) && id(1)!=UINT64_MAX);break;
    }
    case RigExecProviderOpKind::SpaceExpression: {
        const auto *authored=store.ReadMatrix(id(0)),*connected=store.ReadMatrix(id(1));
        if(store.Authoritative(id(0)) && authored)store.PublishMatrix(op.output,*authored,true);
        else if(connected)store.PublishMatrix(op.output,*connected,true);
        else if(authored && *authored!=M(1.0))store.PublishMatrix(op.output,*authored,false);
        else {
            if(id(2)!=UINT64_MAX && !store.ReadMatrix(id(2))) { store.Invalidate(op.output,id(2));break; }
            store.PublishMatrix(op.output,matrix(2),false);
        }break;
    }
    case RigExecProviderOpKind::RestFrame: {
        if(id(7)!=UINT64_MAX && !frame(7)) { store.Invalidate(op.output,id(7));break; }
        if(id(8)!=UINT64_MAX && !store.ReadMatrix(id(8))) { store.Invalidate(op.output,id(8));break; }
        M rest=RigExecProviderComposeAvars<Math>(scalar(1,0),scalar(2,0),scalar(3,0),1,1,1,
            scalar(4,0),scalar(5,0),scalar(6,0),0,{})*matrix(0);
        rest.Orthonormalize(false);
        const auto intervening=matrix(8);
        if(intervening!=M(1.0))rest=rest*intervening;
        store.PublishFrame(op.output,RigExecProviderFrameFromMatrix<Math>(rest*RigExecProviderSpaceFromFrame<Math>(frame(7))));break;
    }
    case RigExecProviderOpKind::DefaultSpace: {
        if(id(0)!=UINT64_MAX && !frame(0)) { store.Invalidate(op.output,id(0));break; }
        if(id(1)!=UINT64_MAX && !frame(1)) { store.Invalidate(op.output,id(1));break; }
        if(id(2)!=UINT64_MAX && !store.ReadMatrix(id(2))) { store.Invalidate(op.output,id(2));break; }
        const auto offset=RigExecProviderComposeAvars<Math>(scalar(3,0),scalar(4,0),scalar(5,0),1,1,1,
            scalar(6,0),scalar(7,0),scalar(8,0),0,{});
        store.PublishMatrix(op.output,offset*RigExecProviderSpaceFromFrame<Math>(frame(0))*
            RigExecProviderSpaceFromFrame<Math>(frame(1)).GetInverse()*matrix(2),false);break;
    }
    case RigExecProviderOpKind::FrameToSpace:
        if(id(0)!=UINT64_MAX && !frame(0)) { store.Invalidate(op.output,id(0));break; }
        store.PublishMatrix(op.output,RigExecProviderSpaceFromFrame<Math>(frame(0)),false);break;
    case RigExecProviderOpKind::MatrixToFrame:
        if(id(0)!=UINT64_MAX && !store.ReadMatrix(id(0))) { store.Invalidate(op.output,id(0));break; }
        store.PublishFrame(op.output,RigExecProviderFrameFromMatrix<Math>(matrix(0)));break;
    case RigExecProviderOpKind::PosedFrame: {
        const auto *posed=store.ReadMatrix(id(0));
        if(posed && (store.Authoritative(id(0)) || *posed!=M(1.0))) {
            store.PublishFrame(op.output,RigExecProviderFrameFromMatrix<Math>(*posed));break;
        }
        bool unavailable=false;
        for(size_t i=1;i<=3;++i)if(id(i)!=UINT64_MAX && !store.ReadMatrix(id(i))) {
            store.Invalidate(op.output,id(i));unavailable=true;break;
        }
        if(unavailable)break;
        const auto avars=RigExecProviderReadAvarMatrix<Math>(op,store,4);
        store.PublishFrame(op.output,RigExecProviderFrameFromMatrix<Math>(RigExecComposeUnswitchedPoseMatrix(avars,matrix(1),matrix(2).GetInverse(),matrix(3))));break;
    }
    case RigExecProviderOpKind::AvarMatrix:
        store.PublishMatrix(op.output,RigExecProviderReadAvarMatrix<Math>(op,store,0),false);break;
    case RigExecProviderOpKind::RelativeXform: {
        M result(1.0);
        for(size_t i=0;i<op.inputs.size();i+=2) {
            const auto *local=store.ReadMatrix(op.inputs[i]);
            if(!local){store.Invalidate(op.output,op.inputs[i]);return true;}
            result=result*(*local);
            if(i+1<op.inputs.size() && store.ResetXformStack(op.inputs[i+1]))break;
        }
        store.PublishMatrix(op.output,result,false);break;
    }
    case RigExecProviderOpKind::JointMatrix: {
        M result(1.0);
        const auto *rest=frame(0),*posed=frame(1);
        if(id(0)!=UINT64_MAX && !rest) { store.Invalidate(op.output,id(0));break; }
        if(id(1)!=UINT64_MAX && !posed) { store.Invalidate(op.output,id(1));break; }
        if(rest && rest->IsValid() && posed && posed->IsValid())Math::PointsToMatrix(rest->points,posed->points,&result);
        store.PublishMatrix(op.output,result,false);break;
    }
    default:return false;
    }
    return true;
}
}
#endif
