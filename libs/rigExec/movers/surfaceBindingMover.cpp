// Persistent surface bindings; coordinate/algorithm references: docs/references.md.
#include "moverRegistry.h"
#include "pxr/base/vt/dictionary.h"
#include "pxr/usd/usdGeom/pointBased.h"
#include <cmath>
#include <set>

PXR_NAMESPACE_USING_DIRECTIVE
using namespace rigExec;
namespace {
template<class T> bool Read(const UsdPrim &p, const RigExecProviderValues &v,
                            const char *name, UsdTimeCode t, T *out) {
    auto a=p.GetAttribute(TfToken(name));
    return v.resolved ? v.resolved->GetAttribute(a,t,out) : a.Get(out,t);
}
template<class T> const T &Get(const VtDictionary &d,const char *key) {
    return d.find(key)->second.Get<T>();
}
bool Finite(const GfVec3f &v) {
    return std::isfinite(v[0]) && std::isfinite(v[1]) && std::isfinite(v[2]);
}
bool Offsets(const VtIntArray &a,size_t rows,size_t elements,int minimum) {
    if(a.size()!=rows+1 || a[0]!=0 || a.back()<0 || size_t(a.back())!=elements) return false;
    for(size_t i=1;i<a.size();++i) if(a[i]<a[i-1] || a[i]-a[i-1]<minimum) return false;
    return true;
}
void Bind(const RigExecMoverBindContext &ctx) {
    auto &b=*ctx.binding;
    const auto surfaces=RigExecRelationshipTargets(ctx.moverPrim,"rigExec:surface");
    if(surfaces.size()==1) {
        b.surfacePoints=surfaces[0].GetPrimPath().AppendProperty(TfToken("points"));
        b.phases[b.surfacePoints]=RigExecPhaseForInput(ctx.moverPrim,"rigExec:surface");
    }
    b.influences=RigExecRelationshipTargets(ctx.moverPrim,"rigExec:frames");
    b.transformPhase=RigExecPhaseForInput(ctx.moverPrim,"rigExec:frames");
    if(b.transformPhase.kind==RigExecReadPhaseKind::Final)
        for(auto &p:b.influences) {
            auto it=ctx.frameChainHeads.find(p);if(it!=ctx.frameChainHeads.end())p=it->second;
        }
}
bool Assemble(const UsdPrim &p,const RigExecRevisionBinding &,
              const RigExecProviderValues &v,UsdTimeCode t,VtValue *out) {
    VtDictionary d;
    const char *ints[]={"vertices","vertexOffsets","polygonOffsets","pointIndices",
                        "limitOffsets","limitIndices"};
    const char *floats[]={"barycentricWeights","bindingWeights","mask","deltaMultipliers","limitWeights",
                          "limitDuWeights","limitDvWeights"};
    for(auto name:ints) {VtIntArray a;if(!Read(p,v,(std::string("rigExec:")+name).c_str(),t,&a))return false;d[name]=VtValue(a);}
    for(auto name:floats) {VtFloatArray a;if(!Read(p,v,(std::string("rigExec:")+name).c_str(),t,&a))return false;
        for(float x:a)if(!std::isfinite(x))return false;d[name]=VtValue(a);}
    VtVec3fArray offsets;if(!Read(p,v,"rigExec:offsets",t,&offsets))return false;
    for(const auto &x:offsets)if(!Finite(x))return false;d["offsets"]=VtValue(offsets);
    TfToken mode,normal;float strength,deltaMultiplier;
    if(!Read(p,v,"rigExec:surfaceMode",t,&mode) || !Read(p,v,"rigExec:normalMode",t,&normal) ||
       !Read(p,v,"rigExec:strength",t,&strength) || !std::isfinite(strength) ||
       !Read(p,v,"rigExec:deltaMultiplier",t,&deltaMultiplier) || !std::isfinite(deltaMultiplier) ||
       (mode!=TfToken("polygon") && mode!=TfToken("limit")) ||
       (normal!=TfToken("geometric") && normal!=TfToken("smooth")))return false;
    d["limit"]=VtValue(mode==TfToken("limit"));d["smooth"]=VtValue(normal==TfToken("smooth"));d["strength"]=VtValue(strength);
    d["deltaMultiplier"]=VtValue(deltaMultiplier);
    auto surfaces=RigExecRelationshipTargets(p,"rigExec:surface");if(surfaces.size()!=1)return false;
    auto surface=p.GetStage()->GetPrimAtPath(surfaces[0].GetPrimPath());
    VtVec3fArray points;if(!Read(surface,v,"points",t,&points))return false;
    GfMatrix4d sourceRest,targetRest,bind;
    if(!Read(p,v,"rigExec:surfaceRestMatrix",t,&sourceRest) || !Read(p,v,"rigExec:targetRestMatrix",t,&targetRest) ||
       !Read(p,v,"rigExec:surfaceToBinding",t,&bind))return false;
    if(v.influenceTransforms) {
        if(v.influenceTransforms->size()!=2)return false;
        sourceRest*=(*v.influenceTransforms)[0];targetRest*=(*v.influenceTransforms)[1];
    }
    if(std::abs(sourceRest.GetDeterminant())<1e-14)return false;
    const GfMatrix4d toBinding=sourceRest.GetInverse()*bind;
    for(auto &point:points) {point=GfVec3f(toBinding.Transform(GfVec3d(point)));if(!Finite(point))return false;}
    d["points"]=VtValue(points);d["toTarget"]=VtValue(targetRest);
    VtIntArray counts,indices;
    if(normal==TfToken("smooth")) {
        if(!Read(surface,v,"faceVertexCounts",t,&counts) || !Read(surface,v,"faceVertexIndices",t,&indices))return false;
        size_t count=0;for(int n:counts){if(n<3)return false;count+=n;}
        if(count!=indices.size())return false;
        for(int i:indices)if(i<0 || size_t(i)>=points.size())return false;
    }
    d["counts"]=VtValue(counts);d["topology"]=VtValue(indices);
    const auto &vertices=Get<VtIntArray>(d,"vertices");
    const auto &pi=Get<VtIntArray>(d,"pointIndices");
    const auto &mask=Get<VtFloatArray>(d,"mask");
    const auto &multipliers=Get<VtFloatArray>(d,"deltaMultipliers");
    std::set<int> seen;
    for(int i:vertices)if(i<0 || size_t(i)>=v.basePoints.size() || !seen.insert(i).second)return false;
    for(int i:pi)if(i<0 || size_t(i)>=points.size())return false;
    if(!Offsets(Get<VtIntArray>(d,"vertexOffsets"),vertices.size(),offsets.size(),1) ||
       !Offsets(Get<VtIntArray>(d,"polygonOffsets"),offsets.size(),pi.size(),3) ||
       Get<VtFloatArray>(d,"barycentricWeights").size()!=pi.size() ||
       Get<VtFloatArray>(d,"bindingWeights").size()!=offsets.size() ||
       (!mask.empty() && mask.size()!=v.basePoints.size()) ||
       (!multipliers.empty() && multipliers.size()!=v.basePoints.size()))return false;
    for(float w:mask)if(w<0 || w>1)return false;
    if(mode==TfToken("limit")) {
        const auto &li=Get<VtIntArray>(d,"limitIndices");
        for(int i:li)if(i<0 || size_t(i)>=points.size())return false;
        if(!Offsets(Get<VtIntArray>(d,"limitOffsets"),offsets.size(),li.size(),1) ||
           Get<VtFloatArray>(d,"limitWeights").size()!=li.size() ||
           Get<VtFloatArray>(d,"limitDuWeights").size()!=li.size() ||
           Get<VtFloatArray>(d,"limitDvWeights").size()!=li.size())return false;
    }
    *out=VtValue(d);return true;
}
GfVec3f PolygonNormal(const VtVec3fArray &p,const int *indices,size_t n) {
    GfVec3f normal(0);
    for(size_t i=0;i<n;++i) {
        const auto &a=p[indices[i]],&b=p[indices[(i+1)%n]];
        normal[0]+=(a[1]-b[1])*(a[2]+b[2]);
        normal[1]+=(a[2]-b[2])*(a[0]+b[0]);
        normal[2]+=(a[0]-b[0])*(a[1]+b[1]);
    }
    return normal;
}
bool Apply(const VtValue &data,std::vector<GfVec3f> *output) {
    if(!data.IsHolding<VtDictionary>())return false;
    const auto &d=data.UncheckedGet<VtDictionary>();
    const auto &p=Get<VtVec3fArray>(d,"points"),&offsets=Get<VtVec3fArray>(d,"offsets");
    const auto &vertices=Get<VtIntArray>(d,"vertices"),&vo=Get<VtIntArray>(d,"vertexOffsets");
    const auto &po=Get<VtIntArray>(d,"polygonOffsets"),&pi=Get<VtIntArray>(d,"pointIndices");
    const auto &weights=Get<VtFloatArray>(d,"barycentricWeights"),&bw=Get<VtFloatArray>(d,"bindingWeights");
    const auto &mask=Get<VtFloatArray>(d,"mask");
    const auto &multipliers=Get<VtFloatArray>(d,"deltaMultipliers");
    const bool limit=Get<bool>(d,"limit"),smooth=Get<bool>(d,"smooth");
    VtVec3fArray normals(p.size(),GfVec3f(0));
    if(smooth) {
        const auto &counts=Get<VtIntArray>(d,"counts"),&indices=Get<VtIntArray>(d,"topology");
        size_t start=0;for(int n:counts) {
            auto normal=PolygonNormal(p,indices.data()+start,n);
            for(int j=0;j<n;++j)normals[indices[start+j]]+=normal;start+=n;
        }
        for(auto &n:normals)n.Normalize();
    }
    auto candidate=*output;
    for(size_t v=0;v<vertices.size();++v) {
        const int vertex=vertices[v];if(vertex<0 || size_t(vertex)>=candidate.size())return false;
        const float deltaMultiplier=Get<float>(d,"deltaMultiplier")*(multipliers.empty()?1:multipliers[vertex]);
        GfVec3f position(0);
        for(int b=vo[v];b<vo[v+1];++b) {
            GfVec3f base(0),normal(0),tangent(0);
            if(limit) {
                const auto &lo=Get<VtIntArray>(d,"limitOffsets"),&li=Get<VtIntArray>(d,"limitIndices");
                const auto &lw=Get<VtFloatArray>(d,"limitWeights"),&du=Get<VtFloatArray>(d,"limitDuWeights"),&dv=Get<VtFloatArray>(d,"limitDvWeights");
                GfVec3f bitangent(0);
                for(int k=lo[b];k<lo[b+1];++k) {base+=p[li[k]]*lw[k];tangent+=p[li[k]]*du[k];bitangent+=p[li[k]]*dv[k];}
                normal=GfCross(tangent,bitangent);
            } else {
                for(int k=po[b];k<po[b+1];++k)base+=p[pi[k]]*weights[k];
                normal=PolygonNormal(p,pi.data()+po[b],po[b+1]-po[b]);
                for(int k=po[b]+1;k<po[b+1];++k) {
                    tangent=p[pi[k]]-p[pi[po[b]]];if(tangent.GetLengthSq()>1e-20f)break;
                }
            }
            if(smooth) {normal=GfVec3f(0);for(int k=po[b];k<po[b+1];++k)normal+=normals[pi[k]]*weights[k];}
            normal.Normalize();tangent-=normal*GfDot(normal,tangent);tangent.Normalize();
            const auto bitangent=GfCross(normal,tangent);
            const auto transported=tangent*offsets[b][0]+bitangent*offsets[b][1]+normal*offsets[b][2];
            position+=(base+deltaMultiplier*transported)*bw[b];
        }
        const float weight=Get<float>(d,"strength")*(mask.empty()?1:mask[vertex]);
        const auto bound=GfVec3f(Get<GfMatrix4d>(d,"toTarget").Transform(GfVec3d(position)));
        candidate[vertex]+=weight*(bound-candidate[vertex]);if(!Finite(candidate[vertex]))return false;
    }
    *output=std::move(candidate);return true;
}
bool Validate(const RigExecMoverValidateContext &ctx,std::string *error) {
    RigExecProviderValues v;VtVec3fArray p;
    if(ctx.targets.size()!=1 || !ctx.stage->GetAttributeAtPath(ctx.targets[0]).Get(&p))return false;
    v.basePoints.assign(p.begin(),p.end());
    const auto frames=RigExecRelationshipTargets(ctx.prim,"rigExec:frames");
    if(!frames.empty() && frames.size()!=2) {*error="surface binding requires zero or two frame providers";return false;}
    VtValue data;if(Assemble(ctx.prim,{},v,UsdTimeCode::Default(),&data))return true;
    *error=ctx.prim.GetPath().GetString()+": invalid surface binding arrays, stencil, mask or coordinate frame";return false;
}
RigExecMoverHandler Handler() {
    RigExecMoverHandler h("RigExecSurfaceBindingMover",&RigExecFixedMoverOp<RigExecRevisionOp::External>,RigExecMoverDomain::Points);
    h.singleTarget=true;h.bind=&Bind;h.validate=&Validate;h.assembleExternal=&Assemble;h.applyExternal=&Apply;
    h.frameRelationships={"rigExec:frames"};h.transformRelationship="rigExec:frames";h.hasScalarOracle=false;
    return h;
}
}
RIGEXEC_REGISTER_MOVER(Handler());
