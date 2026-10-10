// Persistent surface bindings; coordinate/algorithm references: docs/references.md.
#include "moverRegistry.h"
#include "externalPlayback.h"
#include "rigExecGraph/sceneDescriptors.h"
#include "pxr/base/vt/dictionary.h"
#include "pxr/usd/usdGeom/pointBased.h"
#include <cmath>
#include <cstring>
#include <set>

PXR_NAMESPACE_USING_DIRECTIVE
using namespace rigExec;
namespace {
// The binding's own arrays and settings, then the bound surface's points and
// topology, in this order: assembly reads them by position.
const char *const _intArrays[]={"vertices","vertexOffsets","polygonOffsets","pointIndices",
                                "limitOffsets","limitIndices"};
const char *const _floatArrays[]={"barycentricWeights","bindingWeights","mask","deltaMultipliers","limitWeights",
                                  "limitDuWeights","limitDvWeights"};
constexpr size_t _intCount = sizeof(_intArrays)/sizeof(_intArrays[0]);
constexpr size_t _floatCount = sizeof(_floatArrays)/sizeof(_floatArrays[0]);
enum Input : size_t {
    FirstInt = 0,
    FirstFloat = FirstInt + _intCount,
    OffsetVectors = FirstFloat + _floatCount,
    SurfaceMode, NormalMode, Strength, DeltaMultiplier,
    SurfaceRestMatrix, TargetRestMatrix, SurfaceToBinding,
    SurfacePoints, SurfaceCounts, SurfaceIndices, InputCount
};

// Every read is through the resolved inputs at the evaluated time, over the
// schema fallback; the surface's points carry the rigExec:surface phase.
std::vector<RigExecRevisionLeafKey>
DeclaredInputs(const SdfPath &mover, const SdfPath &surface)
{
    using Type = RigExecRevisionLeafType;
    const auto key = [](const SdfPath &path, Type type, VtValue fallback) {
        return RigExecRevisionLeafKey{path, type, RigExecRevisionLeafTime::AtTime,
            RigExecRevisionLeafFlavour::Resolved, std::move(fallback)};
    };
    const auto own = [&](const char *name) {
        return mover.AppendProperty(TfToken(std::string("rigExec:")+name));
    };
    std::vector<RigExecRevisionLeafKey> inputs;
    for (const char *name : _intArrays)
        inputs.push_back(key(own(name), Type::IntArray, VtValue(VtIntArray())));
    for (const char *name : _floatArrays)
        inputs.push_back(key(own(name), Type::FloatArray, VtValue(VtFloatArray())));
    inputs.push_back(key(own("offsets"), Type::Vec3fArray, VtValue(VtVec3fArray())));
    inputs.push_back(key(own("surfaceMode"), Type::Token, VtValue(TfToken("polygon"))));
    inputs.push_back(key(own("normalMode"), Type::Token, VtValue(TfToken("geometric"))));
    inputs.push_back(key(own("strength"), Type::Float, VtValue(1.0f)));
    inputs.push_back(key(own("deltaMultiplier"), Type::Float, VtValue(1.0f)));
    inputs.push_back(key(own("surfaceRestMatrix"), Type::Matrix4d, VtValue(GfMatrix4d(1.0))));
    inputs.push_back(key(own("targetRestMatrix"), Type::Matrix4d, VtValue(GfMatrix4d(1.0))));
    inputs.push_back(key(own("surfaceToBinding"), Type::Matrix4d, VtValue(GfMatrix4d(1.0))));
    if (surface.IsEmpty()) {
        // No single surface: a raw read of the relationship, where no
        // attribute stands, answers empty and the assembly refuses it.
        const SdfPath unbound = mover.AppendProperty(TfToken("rigExec:surface"));
        for (Type type : {Type::Vec3fArray, Type::IntArray, Type::IntArray}) {
            inputs.push_back({unbound, type, RigExecRevisionLeafTime::AtTime,
                              RigExecRevisionLeafFlavour::Raw, VtValue()});
        }
        return inputs;
    }
    const SdfPath prim = surface.GetPrimPath();
    inputs.push_back(key(prim.AppendProperty(TfToken("points")),
                         Type::Vec3fArray, VtValue(VtVec3fArray())));
    inputs.push_back(key(prim.AppendProperty(TfToken("faceVertexCounts")),
                         Type::IntArray, VtValue(VtIntArray())));
    inputs.push_back(key(prim.AppendProperty(TfToken("faceVertexIndices")),
                         Type::IntArray, VtValue(VtIntArray())));
    return inputs;
}

template<class T> bool As(const VtValue &value, T *out) {
    if (!value.IsHolding<T>()) return false;
    *out = value.UncheckedGet<T>();
    return true;
}
template<class T> const T *Get(const VtDictionary &d,const char *key) {
    const auto found = d.find(key);
    return found != d.end() && found->second.IsHolding<T>() ? &found->second.UncheckedGet<T>() : nullptr;
}
// A token input's text: a TfToken from the stage-side declared inputs, a
// std::string from playback, which builds no token (TfToken(text) takes the
// registry).
bool Text(const VtValue &value, const char **out) {
    if (value.IsHolding<TfToken>()) { *out = value.UncheckedGet<TfToken>().GetText(); return true; }
    if (value.IsHolding<std::string>()) { *out = value.UncheckedGet<std::string>().c_str(); return true; }
    return false;
}
bool Is(const char *token, const char *text) {
    return std::strcmp(token, text) == 0;
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
void DeclareInputs(const RigExecMoverBindContext &ctx,
                   std::vector<RigExecRevisionLeafKey> *inputs) {
    const auto surfaces=RigExecRelationshipTargets(ctx.moverPrim,"rigExec:surface");
    *inputs = DeclaredInputs(ctx.moverPrim.GetPath(), surfaces.size()==1 ? surfaces[0] : SdfPath());
}
bool CompileScene(const RigExecSceneDescriptors &scene, const SdfPath &mover,
                  const SdfPath &, RigExecRevisionBinding *binding, std::string *error) {
    if (!binding) return false;
    const auto relationship = [&](const char *name) {
        const auto found = scene.relationships.find(mover.AppendProperty(TfToken(name)));
        return found == scene.relationships.end() ? nullptr : &found->second;
    };
    SdfPath surface;
    if (const auto *rel = relationship("rigExec:surface"); rel && rel->fact.targets.size() == 1) {
        surface = rel->fact.targets[0];
        binding->surfacePoints = surface.GetPrimPath().AppendProperty(TfToken("points"));
        RigExecReadPhase phase;
        if (!RigExecParseReadPhase(rel->readPhase.GetString(), &phase, error)) return false;
        binding->phases[binding->surfacePoints] = phase;
    }
    if (const auto *rel = relationship("rigExec:frames")) {
        binding->influences = rel->fact.targets;
        if (!RigExecParseReadPhase(rel->readPhase.GetString(), &binding->transformPhase, error)) return false;
    }
    binding->externalInputs = DeclaredInputs(mover, surface);
    return true;
}
// The payload from the declared inputs and the provider values. Pure: no
// stage, registry or shared state.
bool Payload(const std::vector<VtValue> &in, const RigExecExternalProviderValues &v, VtValue *out) {
    if(in.size()!=InputCount)return false;
    VtDictionary d;
    for(size_t i=0;i<_intCount;++i) {VtIntArray a;if(!As(in[FirstInt+i],&a))return false;d[_intArrays[i]]=VtValue(a);}
    for(size_t i=0;i<_floatCount;++i) {VtFloatArray a;if(!As(in[FirstFloat+i],&a))return false;
        for(float x:a)if(!std::isfinite(x))return false;d[_floatArrays[i]]=VtValue(a);}
    VtVec3fArray offsets;if(!As(in[OffsetVectors],&offsets))return false;
    for(const auto &x:offsets)if(!Finite(x))return false;d["offsets"]=VtValue(offsets);
    const char *mode=nullptr,*normal=nullptr;float strength=0,deltaMultiplier=0;
    if(!Text(in[SurfaceMode],&mode) || !Text(in[NormalMode],&normal) ||
       !As(in[Strength],&strength) || !std::isfinite(strength) ||
       !As(in[DeltaMultiplier],&deltaMultiplier) || !std::isfinite(deltaMultiplier) ||
       (!Is(mode,"polygon") && !Is(mode,"limit")) ||
       (!Is(normal,"geometric") && !Is(normal,"smooth")))return false;
    const bool limit=Is(mode,"limit"),smooth=Is(normal,"smooth");
    d["limit"]=VtValue(limit);d["smooth"]=VtValue(smooth);d["strength"]=VtValue(strength);
    d["deltaMultiplier"]=VtValue(deltaMultiplier);
    VtVec3fArray points;if(!As(in[SurfacePoints],&points))return false;
    GfMatrix4d sourceRest,targetRest,bind;
    if(!As(in[SurfaceRestMatrix],&sourceRest) || !As(in[TargetRestMatrix],&targetRest) ||
       !As(in[SurfaceToBinding],&bind))return false;
    if(v.influenceTransforms) {
        if(v.influenceTransforms->size()!=2)return false;
        sourceRest*=(*v.influenceTransforms)[0];targetRest*=(*v.influenceTransforms)[1];
    }
    if(std::abs(sourceRest.GetDeterminant())<1e-14)return false;
    const GfMatrix4d toBinding=sourceRest.GetInverse()*bind;
    for(auto &point:points) {point=GfVec3f(toBinding.Transform(GfVec3d(point)));if(!Finite(point))return false;}
    d["points"]=VtValue(points);d["toTarget"]=VtValue(targetRest);
    VtIntArray counts,indices;
    if(smooth) {
        if(!As(in[SurfaceCounts],&counts) || !As(in[SurfaceIndices],&indices))return false;
        size_t count=0;for(int n:counts){if(n<3)return false;count+=n;}
        if(count!=indices.size())return false;
        for(int i:indices)if(i<0 || size_t(i)>=points.size())return false;
    }
    d["counts"]=VtValue(counts);d["topology"]=VtValue(indices);
    const auto &vertices=*Get<VtIntArray>(d,"vertices");
    const auto &pi=*Get<VtIntArray>(d,"pointIndices");
    const auto &mask=*Get<VtFloatArray>(d,"mask");
    const auto &multipliers=*Get<VtFloatArray>(d,"deltaMultipliers");
    std::set<int> seen;
    for(int i:vertices)if(i<0 || size_t(i)>=v.basePoints.size() || !seen.insert(i).second)return false;
    for(int i:pi)if(i<0 || size_t(i)>=points.size())return false;
    if(!Offsets(*Get<VtIntArray>(d,"vertexOffsets"),vertices.size(),offsets.size(),1) ||
       !Offsets(*Get<VtIntArray>(d,"polygonOffsets"),offsets.size(),pi.size(),3) ||
       Get<VtFloatArray>(d,"barycentricWeights")->size()!=pi.size() ||
       Get<VtFloatArray>(d,"bindingWeights")->size()!=offsets.size() ||
       (!mask.empty() && mask.size()!=v.basePoints.size()) ||
       (!multipliers.empty() && multipliers.size()!=v.basePoints.size()))return false;
    for(float w:mask)if(w<0 || w>1)return false;
    if(limit) {
        const auto &li=*Get<VtIntArray>(d,"limitIndices");
        for(int i:li)if(i<0 || size_t(i)>=points.size())return false;
        if(!Offsets(*Get<VtIntArray>(d,"limitOffsets"),offsets.size(),li.size(),1) ||
           Get<VtFloatArray>(d,"limitWeights")->size()!=li.size() ||
           Get<VtFloatArray>(d,"limitDuWeights")->size()!=li.size() ||
           Get<VtFloatArray>(d,"limitDvWeights")->size()!=li.size())return false;
    }
    *out=VtValue(d);return true;
}
bool Assemble(const RigExecExternalInputContext &ctx, VtValue *out) {
    return Payload(ctx.inputs, ctx.values, out);
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
    const auto *points=Get<VtVec3fArray>(d,"points"),*offsetsEntry=Get<VtVec3fArray>(d,"offsets");
    const auto *verticesEntry=Get<VtIntArray>(d,"vertices"),*voEntry=Get<VtIntArray>(d,"vertexOffsets");
    const auto *poEntry=Get<VtIntArray>(d,"polygonOffsets"),*piEntry=Get<VtIntArray>(d,"pointIndices");
    const auto *weightsEntry=Get<VtFloatArray>(d,"barycentricWeights"),*bwEntry=Get<VtFloatArray>(d,"bindingWeights");
    const auto *maskEntry=Get<VtFloatArray>(d,"mask"),*multipliersEntry=Get<VtFloatArray>(d,"deltaMultipliers");
    const auto *limitEntry=Get<bool>(d,"limit"),*smoothEntry=Get<bool>(d,"smooth");
    const auto *strength=Get<float>(d,"strength"),*deltaMultiplierEntry=Get<float>(d,"deltaMultiplier");
    const auto *toTarget=Get<GfMatrix4d>(d,"toTarget");
    const auto *countsEntry=Get<VtIntArray>(d,"counts"),*topologyEntry=Get<VtIntArray>(d,"topology");
    const auto *lo=Get<VtIntArray>(d,"limitOffsets"),*li=Get<VtIntArray>(d,"limitIndices");
    const auto *lw=Get<VtFloatArray>(d,"limitWeights"),*du=Get<VtFloatArray>(d,"limitDuWeights"),*dv=Get<VtFloatArray>(d,"limitDvWeights");
    if(!points || !offsetsEntry || !verticesEntry || !voEntry || !poEntry || !piEntry || !weightsEntry ||
       !bwEntry || !maskEntry || !multipliersEntry || !limitEntry || !smoothEntry || !strength ||
       !deltaMultiplierEntry || !toTarget || !countsEntry || !topologyEntry || !lo || !li || !lw || !du || !dv)
        return false;
    const auto &p=*points,&offsets=*offsetsEntry;
    const auto &vertices=*verticesEntry,&vo=*voEntry,&po=*poEntry,&pi=*piEntry;
    const auto &weights=*weightsEntry,&bw=*bwEntry,&mask=*maskEntry,&multipliers=*multipliersEntry;
    const bool limit=*limitEntry,smooth=*smoothEntry;
    VtVec3fArray normals(p.size(),GfVec3f(0));
    if(smooth) {
        const auto &counts=*countsEntry,&indices=*topologyEntry;
        size_t start=0;for(int n:counts) {
            auto normal=PolygonNormal(p,indices.data()+start,n);
            for(int j=0;j<n;++j)normals[indices[start+j]]+=normal;start+=n;
        }
        for(auto &n:normals)n.Normalize();
    }
    auto candidate=*output;
    for(size_t v=0;v<vertices.size();++v) {
        const int vertex=vertices[v];if(vertex<0 || size_t(vertex)>=candidate.size())return false;
        const float deltaMultiplier=*deltaMultiplierEntry*(multipliers.empty()?1:multipliers[vertex]);
        GfVec3f position(0);
        for(int b=vo[v];b<vo[v+1];++b) {
            GfVec3f base(0),normal(0),tangent(0);
            if(limit) {
                GfVec3f bitangent(0);
                for(int k=(*lo)[b];k<(*lo)[b+1];++k) {base+=p[(*li)[k]]*(*lw)[k];tangent+=p[(*li)[k]]*(*du)[k];bitangent+=p[(*li)[k]]*(*dv)[k];}
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
        const float weight=*strength*(mask.empty()?1:mask[vertex]);
        const auto bound=GfVec3f(toTarget->Transform(GfVec3d(position)));
        candidate[vertex]+=weight*(bound-candidate[vertex]);if(!Finite(candidate[vertex]))return false;
    }
    *output=std::move(candidate);return true;
}
// .rigexec playback: the epoch is a format tag and the bound surface's
// points path, and the frame bytes are empty. The kernel assembles and
// deforms through Payload and Apply from the declared inputs, the surface
// points playback evaluated at the binding's phase, and its provider
// values, so a posed playback follows the surface and the frames.
constexpr char kEpochTag[]={'R','S','B','1'};
bool EncodeEpoch(const RigExecRevisionBinding &binding,std::vector<uint8_t> *epoch) {
    epoch->assign(kEpochTag,kEpochTag+sizeof(kEpochTag));
    const std::string surface=binding.surfacePoints.IsEmpty()?std::string():binding.surfacePoints.GetString();
    epoch->insert(epoch->end(),surface.begin(),surface.end());
    return true;
}
bool Encode(const VtValue &,const RigExecRevisionBinding &binding,
            std::vector<uint8_t> *epoch,std::vector<uint8_t> *frame) {
    frame->clear();return EncodeEpoch(binding,epoch);
}
// The prepared state: the bound surface's points path.
std::shared_ptr<const void> Prepare(const uint8_t *epoch,size_t size,std::string *error) {
    if(size<sizeof(kEpochTag) || std::memcmp(epoch,kEpochTag,sizeof(kEpochTag))!=0) {
        if(error)*error="not a surface binding epoch";
        return nullptr;
    }
    return std::make_shared<const std::string>(reinterpret_cast<const char *>(epoch)+sizeof(kEpochTag),
                                               size-sizeof(kEpochTag));
}
bool Play(const void *state,const uint8_t *,size_t,const RigExecExternalPhasedPoints *phased,size_t phasedCount,
          const RigExecExternalInputValue *inputs,size_t inputCount,
          const RigExecExternalProviders &providers,float *xyz,size_t count) {
    auto values=RigExecExternalPlaybackInputs(inputs,inputCount);
    // The surface's points at the binding's phase, as playback evaluated
    // them, in place of the declared read -- the assembly's phase overlay.
    const auto &surface=*static_cast<const std::string *>(state);
    for(size_t k=0;k<phasedCount && SurfacePoints<values.size();++k) {
        if(!phased[k].path || !phased[k].xyz || surface!=phased[k].path)continue;
        VtVec3fArray points(phased[k].count);
        for(size_t i=0;i<phased[k].count;++i)
            points[i]=GfVec3f(phased[k].xyz[i*3],phased[k].xyz[i*3+1],phased[k].xyz[i*3+2]);
        values[SurfacePoints]=VtValue(points);
    }
    const RigExecExternalPlaybackProviders provided(providers);
    VtValue data;
    return Payload(values,RigExecExternalProviderValues(provided.values),&data) &&
           RigExecExternalPlaybackApply(data,&Apply,xyz,count);
}
bool Validate(const RigExecMoverValidateContext &ctx,std::string *error) {
    VtVec3fArray p;
    if(ctx.targets.size()!=1 || !ctx.stage->GetAttributeAtPath(ctx.targets[0]).Get(&p))return false;
    const auto frames=RigExecRelationshipTargets(ctx.prim,"rigExec:frames");
    if(!frames.empty() && frames.size()!=2) {*error="surface binding requires zero or two frame providers";return false;}
    const auto surfaces=RigExecRelationshipTargets(ctx.prim,"rigExec:surface");
    // The authored arrays against identity frames, the authored points and
    // the surface's Default shape.
    std::vector<VtValue> inputs;
    for(const auto &key:DeclaredInputs(ctx.prim.GetPath(),surfaces.size()==1?surfaces[0]:SdfPath())) {
        VtValue value=key.fallback;
        if(const UsdAttribute a=ctx.stage->GetAttributeAtPath(key.path))a.Get(&value);
        inputs.push_back(value);
    }
    RigExecProviderValues v;v.basePoints.assign(p.begin(),p.end());
    VtValue data;if(Payload(inputs,RigExecExternalProviderValues(v),&data))return true;
    *error=ctx.prim.GetPath().GetString()+": invalid surface binding arrays, stencil, mask or coordinate frame";return false;
}
RigExecMoverHandler Handler() {
    RigExecMoverHandler h("RigExecSurfaceBindingMover",&RigExecFixedMoverOp<RigExecRevisionOp::External>,RigExecMoverDomain::Points);
    h.singleTarget=true;h.bind=&Bind;h.validate=&Validate;
    h.declareExternalInputs=&DeclareInputs;h.compileScene=&CompileScene;
    h.assembleExternal=&Assemble;h.applyExternal=&Apply;
    h.encodeExternalEpoch=&EncodeEpoch;h.encodeExternal=&Encode;
    h.runtimeKernel.prepare=&Prepare;h.runtimeKernel.applyWithProviders=&Play;
    h.frameRelationships={"rigExec:frames"};h.transformRelationship="rigExec:frames";h.hasScalarOracle=false;
    return h;
}
}
RIGEXEC_REGISTER_MOVER(Handler());
