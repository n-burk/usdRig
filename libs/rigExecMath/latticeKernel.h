// Tensor-product cubic lattice displacement interpolation. Basis/edge behavior
// references: docs/references.md. Header-only stage-free native/playback kernel.
#ifndef RIGEXEC_MATH_LATTICE_KERNEL_H
#define RIGEXEC_MATH_LATTICE_KERNEL_H
#include "surfaceSnapKernel.h"
#include <array>
#include <algorithm>
#include <cmath>
#include <limits>
#include <vector>
#include <string>
namespace rigExec {
enum class RigExecLatticeInterpolation { Linear, Cardinal, BSpline, CatmullRom };
inline bool RigExecLatticeInterpolationFromString(const std::string &name,RigExecLatticeInterpolation *out) {
    if(name=="linear")*out=RigExecLatticeInterpolation::Linear;
    else if(name=="cardinal")*out=RigExecLatticeInterpolation::Cardinal;
    else if(name=="bspline")*out=RigExecLatticeInterpolation::BSpline;
    else if(name=="catmullRom")*out=RigExecLatticeInterpolation::CatmullRom;
    else return false;
    return true;
}
template<class M> bool RigExecLatticeCoordinateMaps(const std::string &space,const M &cage,const M &target,
    M *targetToLattice,M *latticeToTarget,M *cageToLattice) {
    if(!RigExecSurfaceSnapValidMatrix(cage) || !RigExecSurfaceSnapValidMatrix(target))return false;
    *cageToLattice=M(1.0);
    if(space=="local") {*targetToLattice=target*RigExecSurfaceSnapAffineInverse(cage);*latticeToTarget=cage*RigExecSurfaceSnapAffineInverse(target);}
    else if(space=="common") {*targetToLattice=RigExecSurfaceSnapAffineInverse(cage);*cageToLattice=RigExecSurfaceSnapAffineInverse(cage);*latticeToTarget=cage;}
    else return false;
    return true;
}
struct RigExecLatticeSettings {
    bool regularGrid=false;
    std::array<RigExecLatticeInterpolation,3> interpolation={RigExecLatticeInterpolation::BSpline,RigExecLatticeInterpolation::BSpline,RigExecLatticeInterpolation::BSpline};
    std::array<float,3> origin={-.5f,-.5f,-.5f},spacing={1,1,1};
    float strength=1;
    std::vector<float> mask;
    bool operator==(const RigExecLatticeSettings &o) const {
        return regularGrid==o.regularGrid && interpolation==o.interpolation && origin==o.origin &&
            spacing==o.spacing && strength==o.strength && mask==o.mask;
    }
};
namespace latticeDetail {
inline std::array<float,4> Basis(float t,RigExecLatticeInterpolation mode) {
    const float t2=t*t,t3=t2*t;
    if(mode==RigExecLatticeInterpolation::Linear)return {0,1-t,t,0};
    if(mode==RigExecLatticeInterpolation::BSpline)
        return {(1-3*t+3*t2-t3)/6,(4-6*t2+3*t3)/6,(1+3*t+3*t2-3*t3)/6,t3/6};
    const float c=mode==RigExecLatticeInterpolation::Cardinal?.71f:.5f;
    return {-c*t3+2*c*t2-c*t,(2-c)*t3+(c-3)*t2+1,(c-2)*t3+(3-2*c)*t2+c*t,c*t3-c*t2};
}
}
template<class V,class D,class M,class Div>
bool RigExecApplyLatticeGridKernel(std::vector<V> *points,const std::vector<V> &posed,
    const Div &divisions,const RigExecLatticeSettings &s,const M &targetToLattice,
    const M &latticeToTarget,const M &cageToLattice) {
    if(!points || !s.regularGrid || !std::isfinite(s.strength) ||
       (!s.mask.empty() && s.mask.size()!=points->size()) ||
       !RigExecSurfaceSnapValidMatrix(targetToLattice) || !RigExecSurfaceSnapValidMatrix(latticeToTarget) ||
       !RigExecSurfaceSnapValidMatrix(cageToLattice))return false;
    for(float w:s.mask)if(!std::isfinite(w) || w<0 || w>1)return false;
    size_t count=1;
    for(int axis=0;axis<3;++axis) {
        if(divisions[axis]<1 || size_t(divisions[axis])>std::numeric_limits<size_t>::max()/count ||
            !std::isfinite(s.origin[axis]) || !std::isfinite(s.spacing[axis]) ||
            (divisions[axis]>1 && s.spacing[axis]==0))return false;
        count*=size_t(divisions[axis]);
    }
    if(posed.size()!=count)return false;
    std::vector<V> deltas;deltas.reserve(count);
    for(int k=0;k<divisions[2];++k)for(int j=0;j<divisions[1];++j)for(int i=0;i<divisions[0];++i) {
        const auto &p=posed[(size_t(k)*divisions[1]+j)*divisions[0]+i];
        if(!surfaceSnapDetail::Finite(p))return false;
        const D local=cageToLattice.TransformAffine(D(p[0],p[1],p[2]));
        const D delta=local-D(s.origin[0]+i*s.spacing[0],s.origin[1]+j*s.spacing[1],s.origin[2]+k*s.spacing[2]);
        const D mapped=latticeToTarget.TransformDir(delta);
        deltas.emplace_back(float(mapped[0]),float(mapped[1]),float(mapped[2]));
        if(!surfaceSnapDetail::Finite(deltas.back()))return false;
    }
    auto candidate=*points;
    for(size_t i=0;i<candidate.size();++i) {
        const float weight=s.strength*(s.mask.empty()?1:s.mask[i]);if(weight==0)continue;
        const auto &p=candidate[i];if(!surfaceSnapDetail::Finite(p))return false;
        const D local=targetToLattice.TransformAffine(D(p[0],p[1],p[2]));
        std::array<std::array<float,4>,3> weights;std::array<int,3> lower;
        for(int axis=0;axis<3;++axis) {
            if(divisions[axis]==1) {weights[axis]={0,1,0,0};lower[axis]=0;continue;}
            const float coordinate=(float(local[axis])-s.origin[axis])/s.spacing[axis];
            if(!std::isfinite(coordinate))return false;
            // Beyond the supported range all four clamped indices coincide;
            // reducing the integer prevents overflow without clamping the query.
            if(coordinate<-2) {lower[axis]=-2;weights[axis]={0,1,0,0};}
            else if(coordinate>float(divisions[axis]+1)) {lower[axis]=divisions[axis];weights[axis]={0,1,0,0};}
            else {lower[axis]=int(std::floor(coordinate));weights[axis]=latticeDetail::Basis(coordinate-lower[axis],s.interpolation[axis]);}
        }
        V delta(0);
        for(int k=0;k<4;++k)for(int j=0;j<4;++j)for(int a=0;a<4;++a) {
            const int x=std::clamp(lower[0]+a-1,0,divisions[0]-1),y=std::clamp(lower[1]+j-1,0,divisions[1]-1),z=std::clamp(lower[2]+k-1,0,divisions[2]-1);
            const float w=weight*weights[2][k]*weights[1][j]*weights[0][a];
            delta+=deltas[(size_t(z)*divisions[1]+y)*divisions[0]+x]*w;
        }
        candidate[i]+=delta;if(!surfaceSnapDetail::Finite(candidate[i]))return false;
    }
    *points=std::move(candidate);return true;
}
}
#endif
