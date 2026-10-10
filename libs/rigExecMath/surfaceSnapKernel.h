// Closest-triangle queries: Ericson, Real-Time Collision Detection, section 5.1.5.
// Side clearance follows the on-surface, inside and outside snap policies;
// algorithm references and source provenance are recorded in docs/references.md.
// Header-only, stage-free arithmetic shared by native evaluation and playback.
#ifndef RIGEXEC_MATH_SURFACE_SNAP_KERNEL_H
#define RIGEXEC_MATH_SURFACE_SNAP_KERNEL_H
#include <algorithm>
#include <cmath>
#include <limits>
#include <vector>
namespace rigExec {
enum class RigExecSurfaceSnapMode { OnSurface, Inside, Outside, OutsideSurface };
template<class M> bool RigExecSurfaceSnapValidMatrix(const M &m) {
    for(int i=0;i<4;++i)for(int j=0;j<4;++j)if(!std::isfinite(m[i][j]))return false;
    const double determinant=m.GetDeterminant();
    return m[0][3]==0 && m[1][3]==0 && m[2][3]==0 && m[3][3]==1 &&
        std::isfinite(determinant) && std::abs(determinant)>=1e-14;
}
// A validated affine matrix can acquire homogeneous-column roundoff in a
// general 4x4 inverse. Restore that known affine column before composition.
template<class M> M RigExecSurfaceSnapAffineInverse(const M &m) {
    M inverse=m.GetInverse();
    inverse[0][3]=inverse[1][3]=inverse[2][3]=0;inverse[3][3]=1;
    return inverse;
}
template<class M> bool RigExecSurfaceSnapCanonicalComputedMatrix(M *m) {
    for(int i=0;i<4;++i)for(int j=0;j<4;++j)if(!std::isfinite((*m)[i][j]))return false;
    // Computed rest-to-pose products can accumulate double roundoff in a
    // column that is analytically affine. Authored inputs use the exact check.
    for(int i=0;i<3;++i)if(std::abs((*m)[i][3])>1e-12)return false;
    if(std::abs((*m)[3][3]-1)>1e-12)return false;
    (*m)[0][3]=(*m)[1][3]=(*m)[2][3]=0;(*m)[3][3]=1;
    return RigExecSurfaceSnapValidMatrix(*m);
}
struct RigExecSurfaceSnapSettings {
    RigExecSurfaceSnapMode mode = RigExecSurfaceSnapMode::OnSurface;
    float offset = 0;
    std::vector<float> mask;
    // Explicit triangle vertex indices; empty retains legacy face fans.
    std::vector<int> triangles;
    bool operator==(const RigExecSurfaceSnapSettings &o) const {
        return mode==o.mode && offset==o.offset && mask==o.mask && triangles==o.triangles;
    }
};
template<class M>
bool RigExecSurfaceSnapIsLegacy(const RigExecSurfaceSnapSettings &s,const M &a,const M &b,const M &c) {
    return s.mode==RigExecSurfaceSnapMode::OnSurface && s.offset==0 && s.mask.empty() &&
        s.triangles.empty() && a==M(1.0) && b==M(1.0) && c==M(1.0);
}
namespace surfaceSnapDetail {
template<class V> auto Dot(const V &a,const V &b) {return a[0]*b[0]+a[1]*b[1]+a[2]*b[2];}
template<class V> V Cross(const V &a,const V &b) {
    return V(a[1]*b[2]-a[2]*b[1],a[2]*b[0]-a[0]*b[2],a[0]*b[1]-a[1]*b[0]);
}
template<class V> bool Finite(const V &v) {
    return std::isfinite(v[0]) && std::isfinite(v[1]) && std::isfinite(v[2]);
}
template<class V> V Closest(const V &p,const V &a,const V &b,const V &c) {
    const V ab=b-a,ac=c-a,ap=p-a;
    const float d1=Dot(ab,ap),d2=Dot(ac,ap);
    if(d1<=0 && d2<=0)return a;
    const V bp=p-b;const float d3=Dot(ab,bp),d4=Dot(ac,bp);
    if(d3>=0 && d4<=d3)return b;
    const float vc=d1*d4-d3*d2;
    if(vc<=0 && d1>=0 && d3<=0)return a+ab*(d1/(d1-d3));
    const V cp=p-c;const float d5=Dot(ab,cp),d6=Dot(ac,cp);
    if(d6>=0 && d5<=d6)return c;
    const float vb=d5*d2-d1*d6;
    if(vb<=0 && d2>=0 && d6<=0)return a+ac*(d2/(d2-d6));
    const float va=d3*d6-d5*d4;
    if(va<=0 && d4-d3>=0 && d5-d6>=0)return b+(c-b)*((d4-d3)/((d4-d3)+(d5-d6)));
    const float inv=1/(va+vb+vc);
    return a+ab*(vb*inv)+ac*(vc*inv);
}
// The nearest displacement determines clearance, with the triangle normal
// deciding the side. Near the surface blend toward the normal for stability.
template<class V> V Snap(const V &p,const V &hit,const V &normal,
                         const RigExecSurfaceSnapSettings &s) {
    if(s.mode==RigExecSurfaceSnapMode::OnSurface && s.offset==0)return hit;
    V delta=p-hit;const float distance=delta.GetLength();
    const bool force=s.mode==RigExecSurfaceSnapMode::OnSurface || s.mode==RigExecSurfaceSnapMode::OutsideSurface;
    float side=s.mode==RigExecSurfaceSnapMode::Inside?-1.f:1.f;
    if(s.mode==RigExecSurfaceSnapMode::OnSurface)side=0;
    if(distance<std::numeric_limits<float>::epsilon())
        return force || s.offset>0 ? hit+normal*(s.offset*side) : hit;
    const float dot=Dot(delta,normal);
    const float sign=dot<0?-1.f:1.f;
    if(side==0)side=sign;
    if(!force && sign*distance*side>=s.offset)return p;
    delta*=sign/distance;
    const float epsilon=(std::abs(s.offset)+std::abs(hit[0])+std::abs(hit[1])+std::abs(hit[2]))*1e-4f;
    if(distance<epsilon)delta=normal+(delta-normal)*(distance/epsilon);
    return hit+delta*(s.offset*side);
}
}
// Both maps are row-vector affine maps. Distance and clearance are measured
// in surface coordinates, then the candidate returns to incoming point space.
template<class V,class D,class M>
bool RigExecApplySurfaceSnapKernel(std::vector<V> *points,const std::vector<V> &surface,
    const std::vector<int> &counts,const std::vector<int> &indices,
    const RigExecSurfaceSnapSettings &settings,const M &toSurface,const M &toTarget,
    const M &surfaceToMetric) {
    using namespace surfaceSnapDetail;
    if(!points || surface.empty() || !std::isfinite(settings.offset) ||
       (!settings.mask.empty() && settings.mask.size()!=points->size()))return false;
    for(float w:settings.mask)if(!std::isfinite(w) || w<0 || w>1)return false;
    std::vector<V> metricSurface;metricSurface.reserve(surface.size());
    for(const V &v:surface) {
        if(!Finite(v))return false;
        const D p=surfaceToMetric.TransformAffine(D(v[0],v[1],v[2]));
        metricSurface.emplace_back(float(p[0]),float(p[1]),float(p[2]));
        if(!Finite(metricSurface.back()))return false;
    }
    for(int i=0;i<4;++i)for(int j=0;j<4;++j)
        if(!std::isfinite(toSurface[i][j]) || !std::isfinite(toTarget[i][j]) ||
           !std::isfinite(surfaceToMetric[i][j]))return false;
    for(const auto &m:{toSurface,toTarget,surfaceToMetric})
        if(m[0][3]!=0 || m[1][3]!=0 || m[2][3]!=0 || m[3][3]!=1)return false;
    std::vector<int> triangles=settings.triangles;
    if(triangles.empty()) {
        size_t cursor=0;
        for(int n:counts) {
            if(n<3 || size_t(n)>indices.size()-cursor)return false;
            for(int j=1;j+1<n;++j) {
                triangles.push_back(indices[cursor]);triangles.push_back(indices[cursor+j]);triangles.push_back(indices[cursor+j+1]);
            }
            cursor+=n;
        }
        if(cursor!=indices.size())return false;
    }
    if(triangles.empty() || triangles.size()%3)return false;
    for(int i:triangles)if(i<0 || size_t(i)>=surface.size())return false;
    auto candidate=*points;
    for(size_t i=0;i<candidate.size();++i) {
        const float w=settings.mask.empty()?1:settings.mask[i];if(w==0)continue;
        const D query=toSurface.TransformAffine(D(candidate[i][0],candidate[i][1],candidate[i][2]));
        const V p{float(query[0]),float(query[1]),float(query[2])};if(!Finite(p))return false;
        float best=std::numeric_limits<float>::max();V hit=p,normal(0);bool found=false;
        for(size_t t=0;t<triangles.size();t+=3) {
            const V &a=metricSurface[triangles[t]],&b=metricSurface[triangles[t+1]],&c=metricSurface[triangles[t+2]];
            V n=Cross(b-a,c-a);const float length=n.GetLength();if(length<=0)continue;
            const V q=Closest(p,a,b,c);const float distance=(q-p).GetLengthSq();
            if(distance<best) {best=distance;hit=q;normal=n/length;found=true;}
        }
        if(!found)return false;
        const V snapped=Snap(p,hit,normal,settings);
        const D result=toTarget.TransformAffine(D(snapped[0],snapped[1],snapped[2]));
        const V local{float(result[0]),float(result[1]),float(result[2])};
        candidate[i]+=(local-candidate[i])*w;if(!Finite(candidate[i]))return false;
    }
    *points=std::move(candidate);return true;
}
}
#endif
