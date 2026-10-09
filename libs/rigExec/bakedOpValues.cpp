#include "bakedOpValues.h"
#include "bakedProgramImpl.h"
#include "pathText.h"
#include "weightField.h"
#include "movers/moverRegistry.h"
#include "pxr/base/tf/diagnostic.h"
#include <algorithm>
#include <cstring>
#include <type_traits>
#include <typeinfo>

namespace rigExec {
namespace {
template<class T> bool SourceSame(const T &a, const T &b) { return a == b; }
bool SourceSame(float a,float b) { return std::memcmp(&a,&b,sizeof(a))==0; }
bool SourceSame(double a,double b) { return std::memcmp(&a,&b,sizeof(a))==0; }
template<class T,int N> bool SourceVectorSame(const T &a,const T &b) {
    for(int i=0;i<N;++i) if(!SourceSame(a[i],b[i])) return false;
    return true;
}
bool SourceSame(const GfVec2f&a,const GfVec2f&b) {return SourceVectorSame<GfVec2f,2>(a,b);}
bool SourceSame(const GfVec3f&a,const GfVec3f&b) {return SourceVectorSame<GfVec3f,3>(a,b);}
bool SourceSame(const GfVec3d&a,const GfVec3d&b) {return SourceVectorSame<GfVec3d,3>(a,b);}
bool SourceSame(const GfMatrix4d&a,const GfMatrix4d&b) {
    for(int r=0;r<4;++r) for(int c=0;c<4;++c) if(!SourceSame(a[r][c],b[r][c])) return false;
    return true;
}
// Padding-free element types whose SourceSame is their bytes: one shared
// buffer, or one memcmp, decides the whole array.
template<class T> constexpr bool kBitwiseElement=
    std::is_same_v<T,double> || std::is_same_v<T,float> || std::is_same_v<T,int> ||
    std::is_same_v<T,GfVec2f> || std::is_same_v<T,GfVec3f> || std::is_same_v<T,GfVec3d> ||
    std::is_same_v<T,GfVec3i> || std::is_same_v<T,GfMatrix4d>;
static_assert(sizeof(GfVec2f)==2*sizeof(float) && sizeof(GfVec3f)==3*sizeof(float) &&
              sizeof(GfVec3d)==3*sizeof(double) && sizeof(GfVec3i)==3*sizeof(int) &&
              sizeof(GfMatrix4d)==16*sizeof(double),"array memcmp needs padding-free elements");
template<class T> bool SourceArraySame(const VtArray<T> &x,const VtArray<T> &y) {
    if constexpr(kBitwiseElement<T>) {
        return x.IsIdentical(y) || (x.size()==y.size() &&
            (x.empty() || std::memcmp(x.cdata(),y.cdata(),x.size()*sizeof(T))==0));
    } else {
        if(x.size()!=y.size()) return false;
        for(size_t i=0;i<x.size();++i) if(!SourceSame(x[i],y[i])) return false;
        return true;
    }
}
}
bool RigExecExactSourceValueEqual(const VtValue &a,const VtValue &b) {
    if(a.IsEmpty() || b.IsEmpty()) return a.IsEmpty() && b.IsEmpty();
#define SAME(T) if(a.IsHolding<T>()) return b.IsHolding<T>() && SourceSame(a.UncheckedGet<T>(),b.UncheckedGet<T>()); \
    if(a.IsHolding<VtArray<T>>()) return b.IsHolding<VtArray<T>>() && SourceArraySame(a.UncheckedGet<VtArray<T>>(),b.UncheckedGet<VtArray<T>>());
    SAME(double) SAME(float) SAME(int) SAME(bool) SAME(TfToken) SAME(std::string)
    SAME(GfVec2f) SAME(GfVec3f) SAME(GfVec3d) SAME(GfVec3i) SAME(GfMatrix4d)
#undef SAME
    if(a.IsHolding<SdfPath>()) return b.IsHolding<SdfPath>() && a.UncheckedGet<SdfPath>()==b.UncheckedGet<SdfPath>();
    return false;
}
namespace {
template<class T, typename std::enable_if<std::is_arithmetic<T>::value ||
    std::is_enum<T>::value, int>::type = 0>
void Put(std::string *out, const T &value)
{
    // Append individual fields rather than object representations containing
    // padding. Floating values keep signed zero and every NaN payload bit.
    out->append(reinterpret_cast<const char *>(&value), sizeof(value));
}
void Put(std::string *out, const std::string &value)
{
    Put(out, uint64_t(value.size())); out->append(value);
}
// The same bytes as its string; GetString() on an empty token reads a
// function-local static.
void Put(std::string *out, const TfToken &value)
{
    Put(out, uint64_t(value.size())); out->append(value.GetText(), value.size());
}
void Put(std::string *out, const GfVec2f &v) { for (int i=0;i<2;++i) Put(out,v[i]); }
void Put(std::string *out, const GfVec3f &v) { for (int i=0;i<3;++i) Put(out,v[i]); }
void Put(std::string *out, const GfVec3d &v) { for (int i=0;i<3;++i) Put(out,v[i]); }
void Put(std::string *out, const GfVec3i &v) { for (int i=0;i<3;++i) Put(out,v[i]); }
void Put(std::string *out, const GfMatrix4d &v)
{
    for (int r=0;r<4;++r) for (int c=0;c<4;++c) Put(out,v[r][c]);
}
// Put writes these element types as their object bytes in memory order:
// integers, float, double and the padding-free vectors above. A contiguous
// run of them is one append.
template<class T> constexpr bool kRunElement=RigExecOpKeyBulkElement<T>::value || kBitwiseElement<T>;
static_assert(std::is_trivially_copyable_v<GfVec2f> && std::is_trivially_copyable_v<GfVec3f> &&
              std::is_trivially_copyable_v<GfVec3d> && std::is_trivially_copyable_v<GfVec3i> &&
              std::is_trivially_copyable_v<GfMatrix4d>,"key runs copy object bytes");
template<class T> void PutRun(std::string *out, const T *data, size_t count)
{
    static_assert(kRunElement<T>,"Put writes this element type field by field");
    RigExecOpKeyAppendRun(out,data,count);
}
template<class ArrayT> void Array(std::string *out, const ArrayT &values);
// Boxed samples retain a type tag plus exact payload bytes. Unknown plugin
// values are deliberately non-exact; callers must propagate conservatively.
bool BoxExact(const VtValue &v)
{
    if(v.IsEmpty()) return true;
#define KNOWN(T) if(v.IsHolding<T>() || v.IsHolding<VtArray<T>>()) return true;
    KNOWN(double) KNOWN(float) KNOWN(int) KNOWN(bool) KNOWN(TfToken)
    KNOWN(std::string) KNOWN(GfVec2f) KNOWN(GfVec3f) KNOWN(GfVec3d)
    KNOWN(GfVec3i) KNOWN(GfMatrix4d)
#undef KNOWN
    return v.IsHolding<SdfPath>();
}
// Closed tags for the types Box encodes, in place of VtValue::GetTypeName(),
// which demangles through shared statics. Keys never leave the process and
// meet only keys this code made, so a tag need only be injective.
enum class BoxTag : uint8_t {
    Double=1, Float, Int, Bool, Token, String, Path, Vec2f, Vec3f, Vec3d,
    Vec3i, Matrix4d, DoubleArray, FloatArray, IntArray, BoolArray,
    TokenArray, StringArray, Vec2fArray, Vec3fArray, Vec3dArray, Vec3iArray,
    Matrix4dArray, PathElements, Opaque=255
};
// A path whose text RigExecSpellPathText cannot spell (relative, under a
// variant selection, or a target, mapper or expression path) keys by its
// elements: one self-delimiting record per prefix, read from nodes \p path
// keeps alive. Equal paths give equal bytes and distinct paths distinct
// bytes, as their texts would, without Sdf's text tables.
void PutPathElements(std::string *out, const SdfPath &path)
{
    Put(out,uint8_t(path.IsEmpty() ? 0 : path.IsAbsolutePath() ? 1 : 2));
    if(path.IsEmpty()) return;
    std::vector<SdfPath> prefixes; path.GetPrefixes(&prefixes);
    Put(out,uint64_t(prefixes.size()));
    for(const SdfPath &prefix:prefixes) {
        if(prefix.IsPrimVariantSelectionPath()) {
            const auto selection=prefix.GetVariantSelection();
            Put(out,uint8_t(1)); Put(out,selection.first); Put(out,selection.second);
        } else if(!prefix.ContainsPropertyElements()) {
            Put(out,uint8_t(2)); Put(out,prefix.GetNameToken());
        } else if(prefix.IsPrimPropertyPath()) {
            Put(out,uint8_t(3)); Put(out,prefix.GetNameToken());
        } else if(prefix.IsRelationalAttributePath()) {
            Put(out,uint8_t(4)); Put(out,prefix.GetNameToken());
        } else if(prefix.IsMapperArgPath()) {
            Put(out,uint8_t(5)); Put(out,prefix.GetNameToken());
        } else if(prefix.IsTargetPath()) {
            Put(out,uint8_t(6)); PutPathElements(out,prefix.GetTargetPath());
        } else if(prefix.IsMapperPath()) {
            Put(out,uint8_t(7)); PutPathElements(out,prefix.GetTargetPath());
        } else {
            // Sdf's one remaining property node kind.
            Put(out,uint8_t(8));
        }
    }
}
// An opaque value keeps its type's identity, so a non-exact output key still
// separates two types. The ABI name is a field of type_info; MSVC's name()
// undecorates into a shared cache.
void PutTypeIdentity(std::string *out, const std::type_info &type)
{
#if defined(_MSC_VER)
    const char *name=type.raw_name();
#else
    const char *name=type.name();
#endif
    const size_t size=std::strlen(name);
    Put(out,uint64_t(size)); out->append(name,size);
}
bool Box(std::string *out, const VtValue &v)
{
    Put(out,v.IsEmpty()); if(v.IsEmpty()) return true;
#define SCALAR(T,TAG) if(v.IsHolding<T>()) { Put(out,BoxTag::TAG); Put(out,v.UncheckedGet<T>()); return true; }
    SCALAR(double,Double) SCALAR(float,Float) SCALAR(int,Int) SCALAR(bool,Bool)
    SCALAR(TfToken,Token) SCALAR(std::string,String)
    SCALAR(GfVec2f,Vec2f) SCALAR(GfVec3f,Vec3f) SCALAR(GfVec3d,Vec3d) SCALAR(GfVec3i,Vec3i)
    SCALAR(GfMatrix4d,Matrix4d)
#undef SCALAR
    if(v.IsHolding<SdfPath>()) {
        // The text GetString() would give, spelled without its token table.
        const SdfPath &path=v.UncheckedGet<SdfPath>();
        std::string text;
        if(RigExecSpellPathText(path,&text)) { Put(out,BoxTag::Path); Put(out,text); }
        else { Put(out,BoxTag::PathElements); PutPathElements(out,path); }
        return true;
    }
#define VECTOR(T,TAG) if(v.IsHolding<VtArray<T>>()) { Put(out,BoxTag::TAG); Array(out,v.UncheckedGet<VtArray<T>>()); return true; }
    VECTOR(double,DoubleArray) VECTOR(float,FloatArray) VECTOR(int,IntArray) VECTOR(bool,BoolArray)
    VECTOR(TfToken,TokenArray) VECTOR(std::string,StringArray) VECTOR(GfVec2f,Vec2fArray)
    VECTOR(GfVec3f,Vec3fArray) VECTOR(GfVec3d,Vec3dArray) VECTOR(GfVec3i,Vec3iArray)
    VECTOR(GfMatrix4d,Matrix4dArray)
#undef VECTOR
    Put(out,BoxTag::Opaque); PutTypeIdentity(out,v.GetTypeid());
    return false;
}
void Put(std::string *out, const RigExecPointFrame &v)
{
    Put(out, v.flags); for (const auto &point : v.points) Put(out, point);
}
template<class ArrayT> void Array(std::string *out, const ArrayT &values)
{
    using T=typename ArrayT::value_type;
    Put(out, uint64_t(values.size()));
    if constexpr(kRunElement<T> && RigExecOpKeyContiguous<ArrayT>::value)
        PutRun(out,values.data(),values.size());
    else for (const auto &value : values) Put(out,value);
}
void Put(std::string *out, const RigExecPointFrameArray &v)
{
    Array(out, v.frames); Put(out,uint64_t(v.rests.size()));
    for (const auto &rest : v.rests) for (const auto &point : rest) Put(out,point);
}
void Put(std::string *out, const RigExecWeightPacket &v)
{
    Put(out,v.valid); Put(out,v.representation); Put(out,v.rangePolicy);
    Put(out,v.defaultWeight); Array(out,v.values); Array(out,v.indices);
}
void Put(std::string *out, const RigExecMoverStatus &v)
{
    Put(out,v.state); Put(out,v.firstBadAddress);
}
void Put(std::string *out, const RigExecWrinkleSettings &v)
{
    Put(out,v.iterations); Put(out,v.topology); Put(out,v.neighborDistance);
    Put(out,v.restLengthScale); Put(out,v.stretchStiffness);
    Put(out,v.compressionStiffness); Put(out,v.bendStiffness);
    Put(out,v.maxDisplacement); Put(out,v.pinBorders);
    Put(out,v.tangentPlaneCollisions); Put(out,v.tangentPlaneInset);
    Put(out,v.wrinkleScale); Put(out,v.smoothingIterations); Array(out,v.pinPoints);
}
void Put(std::string *out, const RigExecSkinTopology &v)
{
    Put(out,v.validated); Put(out,v.elementSize); Put(out,uint64_t(v.pointCount));
    Put(out,uint64_t(v.influenceCount)); Array(out,v.indices); Array(out,v.weights);
}
void Put(std::string *out, const RigExecMoverParameters &v)
{
    Put(out,v.kind); Put(out,v.enabled); Put(out,v.valid); Put(out,v.transform);
    Put(out,v.radialWeight); Put(out,v.weights); Array(out,v.blendDeltas);
    Put(out,v.blendSurfaceFrame); Put(out,v.referenceVolume); Put(out,v.strength);
    Put(out,v.mushIterations); Put(out,v.mushStep); Put(out,v.mushPinBorders);
    Put(out,v.mushDistanceWeight); Put(out,v.mushDisplacement); Put(out,v.wrinkleSettings);
    Array(out,v.topologyCounts); Array(out,v.topologyIndices); Array(out,v.auxPoints);
    Array(out,v.auxPointsB); Array(out,v.restPoints); Put(out,v.divisions);
    Array(out,v.bindCoords); Put(out,v.frames); Array(out,v.wireBindCoords);
    Put(out,v.curveOrder); Array(out,v.curveKnots); Put(out,v.dropoffDistance);
    Array(out,v.widths); Array(out,v.skinTransforms); Array(out,v.skinIndices);
    Array(out,v.skinWeights); Put(out,v.skinElementSize); Put(out,v.skinningMethod);
    Put(out,bool(v.skinTopology)); if(v.skinTopology) Put(out,*v.skinTopology);
    // mushRest and externalHandler are derived immutable handles. Opaque
    // externalData is deliberately not approximated by printable text/hash.
    Put(out,v.externalSchema); Put(out,v.externalData.IsEmpty());
}
const RigExecBakedProgramImpl::GeomRevision *Revision(
    const RigExecBakedProgramImpl &B, uint32_t slot)
{
    if (slot >= B.revisionIndex.size()) return nullptr;
    const auto index = B.revisionIndex[slot];
    if (index.first < 0 || size_t(index.first) >= B.chains.size()) return nullptr;
    const auto &chain = B.chains[size_t(index.first)];
    if (index.second < 0 || size_t(index.second) >= chain.revisions.size()) return nullptr;
    return &chain.revisions[size_t(index.second)];
}
const RigExecBakedProgramImpl::GeomRevision *LayoutRevision(
    const RigExecBakedProgramImpl &B, uint32_t slot)
{
    if(slot<B.revisionIndex.size()) return Revision(B,slot);
    const size_t d=size_t(slot)-B.revisionIndex.size();
    if(d>=B.derivedIndex.size()) return nullptr;
    const auto index=B.derivedIndex[d];
    if(index.first<0 || size_t(index.first)>=B.chains.size()) return nullptr;
    const auto &chain=B.chains[size_t(index.first)];
    if(index.second<0 || size_t(index.second)>=chain.derived.size()) return nullptr;
    return &chain.derived[size_t(index.second)].revision;
}
void Invalid(std::string *out, RigExecBakedSlotDomain domain, uint32_t slot)
{
    out->clear(); Put(out,uint8_t(0)); Put(out,domain); Put(out,slot);
    TF_VERIFY(false, "Invalid baked operation value domain %u slot %u",
              unsigned(domain), slot);
}
}

bool RigExecBakedSpaceLeafKey(const RigExecBakedProgramImpl &B,uint32_t slot,std::string *out)
{
    out->clear(); Put(out,uint8_t(1)); Put(out,RigExecBakedSlotDomain::SpaceLeaf);
    if(slot>=B.providerLeaves.values.size() || slot>=B.providerProgram.sampled.size() ||
       slot>=B.providerLeafBlocked.size()) {
        Invalid(out,RigExecBakedSlotDomain::SpaceLeaf,slot); return false;
    }
    const auto *overlay=RigExecBakedSpaceLeafOverlay(B,size_t(slot));
    const bool useOverlay=overlay!=nullptr;
    Put(out,useOverlay); Put(out,bool(!useOverlay && B.providerLeafBlocked[slot]));
    // Box answers exactly when BoxExact does.
    return Box(out,useOverlay ? *overlay : B.providerLeaves.values[slot]);
}

bool RigExecBakedOpValueKeyIsExact(const RigExecBakedProgramImpl &B,
    RigExecBakedSlotDomain domain, uint32_t slot)
{
    if (domain == RigExecBakedSlotDomain::RevisionPacket) {
        const auto *revision = Revision(B,slot);
        return revision && revision->parameters.externalData.IsEmpty();
    }
    if(domain == RigExecBakedSlotDomain::RequiredStageFramesAdmission) return slot == 0;
    if(domain == RigExecBakedSlotDomain::SwitchFrame) return slot<B.switchFrames.size();
    if(domain == RigExecBakedSlotDomain::SpaceValue) {
        if(slot>=B.providerValues.values.size()) return false;
        const auto *boxed=std::get_if<VtValue>(&B.providerValues.values[slot].value);
        if(boxed) return BoxExact(*boxed);
    }
    if(domain==RigExecBakedSlotDomain::SpaceLeaf) {
        if(slot>=B.providerLeaves.values.size() || slot>=B.providerProgram.sampled.size() || slot>=B.providerLeafBlocked.size()) return false;
        const auto *overlay=RigExecBakedSpaceLeafOverlay(B,size_t(slot));
        const bool useOverlay=overlay!=nullptr;
        return BoxExact(useOverlay ? *overlay : B.providerLeaves.values[slot]);
    }
    if(domain==RigExecBakedSlotDomain::PropertyResult &&
       slot<B.propertyRecordById.size() && B.propertyRecordById[slot]>=0) {
        const size_t r=size_t(B.propertyRecordById[slot]);
        return r<B.recordValues.size() && BoxExact(B.recordValues[r]);
    }
    if(domain==RigExecBakedSlotDomain::ConstraintInputs) {
        if(slot>=B.constraintArrays.size()) return false;
        for(const auto &value:B.constraintArrays[slot].raw) if(!BoxExact(value)) return false;
    }
    return domain != RigExecBakedSlotDomain::Snapshots;
}

void RigExecBakedOpValueKey(const RigExecBakedProgramImpl &B,
    RigExecBakedSlotDomain domain, uint32_t slot, std::string *out)
{
    if (!TF_VERIFY(out)) return;
    out->clear(); Put(out,uint8_t(1)); Put(out,domain);
    const auto bad = [&] { Invalid(out,domain,slot); };
    using D = RigExecBakedSlotDomain;
    switch(domain) {
    case D::Avars:
        if (uint64_t(slot)*11+11 > B.avars.size()) { bad(); return; }
        for(size_t i=size_t(slot)*11;i<size_t(slot)*11+11;++i) Put(out,B.avars[i]);
        return;
    case D::PoseBase: if(slot<B.base.size()) { Put(out,B.base[slot]); return; } break;
    case D::PoseFin: if(slot<B.fin.size()) { Put(out,B.fin[slot]); return; } break;
    case D::PosedM: if(slot<B.posedM.size()) { Put(out,B.posedM[slot]); return; } break;
    case D::FinalMatrix: if(slot<B.finalMatrix.size()) { Put(out,B.finalMatrix[slot]); return; } break;
    case D::BaseMatrix: if(slot<B.baseMatrix.size()) { Put(out,B.baseMatrix[slot]); return; } break;
    case D::Aggregate: if(slot<B.aggregates.size()) { Put(out,B.aggregates[slot]); return; } break;
    case D::SolverPoints:
        if(slot<B.solvers.size()) { const auto &v=B.solvers[slot];
            Array(out,v.ribbonPointsVarying ? v.ribbonPoints : v.ribbonConstantPoints); return; } break;
    case D::Candidates:
        if(slot<B.solvers.size()) { const auto &v=B.solvers[slot];
            Array(out,v.outFrames); Array(out,v.outPresent);
            // Each joint's text as Build spelled it, never SdfPath's tables.
            const size_t spelled=B.pathTexts ? B.pathTexts->size() : 0;
            Put(out,uint64_t(v.fallbackSlots.size()));
            for(const int joint:v.fallbackSlots) {
                if(joint<0 || size_t(joint)>=spelled) { bad(); return; }
                Put(out,(*B.pathTexts)[size_t(joint)]);
            }
            return; } break;
    case D::CommitTable:
        if(slot<B.commits.size()) { const auto &v=B.commits[slot];
            Put(out,v.abandoned); Array(out,v.present); Array(out,v.frames);
            Put(out,v.recordAfter); Put(out,v.recordEveryTarget); return; } break;
    case D::CommitDelta:
        if(slot<B.commits.size()) { const auto &v=B.commits[slot];
            Array(out,v.deltas); Array(out,v.deltaOk); return; } break;
    case D::CommitStaging:
        for(const auto &v:B.commits) if(v.split && v.stagingBase>=0 && slot>=uint32_t(v.stagingBase) &&
            uint64_t(slot)-uint32_t(v.stagingBase)<v.staged.size()) {
            const size_t i=slot-uint32_t(v.stagingBase);
            if(i>=v.outcome.size()) break; Put(out,v.staged[i]); Put(out,v.outcome[i]); return;
        } break;
    case D::ConstraintDelta:
        if(slot<B.deltaValues.size() && slot<B.deltaPresent.size()) {
            Put(out,B.deltaPresent[slot]); Put(out,B.deltaValues[slot]); return; } break;
    case D::FrameMatrix:
        if(slot<B.frameMatrix.size() && slot<B.frameMatrixValid.size()) {
            Put(out,B.frameMatrixValid[slot]); Put(out,B.frameMatrix[slot]); return; } break;
    case D::ChainBase:
        if(slot<B.chains.size()) { const auto &v=B.chains[slot];
            Put(out,v.haveBase); Array(out,v.lastBase); return; } break;
    case D::ChainPoints:
        if(slot<B.chains.size()) { const auto &v=B.chains[slot];
            Put(out,v.haveResult); Array(out,v.result); return; } break;
    case D::RevisionPacket:
        if(const auto *v=Revision(B,slot)) {
            Put(out,v->parameters); Put(out,v->status); Put(out,v->layoutUsable);
            Put(out,v->envelopeOk); Array(out,v->envelope); Put(out,v->fullStrength);
            Put(out,uint64_t(v->precedingCount)); Put(out,v->partitionStale);
            Put(out,v->weightFieldPublished); Array(out,v->publishedWeightValues); return;
        } break;
    case D::RevisionTransforms:
        if(const auto *v=Revision(B,slot)) { Array(out,v->influences);
            Put(out,v->transform); Put(out,v->haveTransform); Put(out,v->carry);
            Put(out,v->haveCarry); Put(out,v->influencesValid); return; } break;
    case D::RevisionOut:
        // A chunk owns only its half-open range. Reading sibling output
        // here would race the parallel writer and cause false change waves.
        for(size_t r=0;r<B.revisionChunkBase.size() && r<B.revisionChunkCount.size();++r) {
            const int base=B.revisionChunkBase[r], count=B.revisionChunkCount[r];
            if(base<0 || count<0 || slot<uint32_t(base) || uint64_t(slot)-uint32_t(base)>=uint32_t(count)) continue;
            const auto *v=Revision(B,uint32_t(r));
            const size_t chunk=slot-uint32_t(base);
            if(!v || chunk>=v->chunks.size()) break;
            const auto &part=v->chunks[chunk];
            if(part.begin<0 || part.end<part.begin) break;
            const size_t begin=std::min(size_t(part.begin),v->stagingOutput.size());
            const size_t end=std::min(size_t(part.end),v->stagingOutput.size());
            Put(out,part.ok); Put(out,uint64_t(v->stagingOutput.size())); Put(out,uint64_t(end-begin));
            PutRun(out,v->stagingOutput.data()+begin,end-begin);
            return;
        } break;
    case D::RevisionDone:
        if(const auto *v=Revision(B,slot)) {
            Put(out,v->resultStatus); Put(out,v->status);
            const auto index=B.revisionIndex[slot]; const auto &chain=B.chains[size_t(index.first)];
            if(v->currentSource<0) Array(out,chain.lastBase);
            else if(size_t(v->currentSource)<chain.revisions.size())
                Array(out,chain.revisions[size_t(v->currentSource)].output);
            else break;
            return;
        } break;
    case D::ChainDirty:
        // The compatibility control edge carries the completed semantic value.
        // Skip counters and dirty flags are never published values.
        RigExecBakedOpValueKey(B,D::RevisionDone,slot,out); return;
    case D::DerivedOut:
        if(slot<B.derivedIndex.size()) {
            const auto index=B.derivedIndex[slot];
            if(index.first<0 || size_t(index.first)>=B.chains.size()) break;
            const auto &chain=B.chains[size_t(index.first)];
            if(index.second<0 || size_t(index.second)>=chain.derived.size()) break;
            const auto &v=chain.derived[size_t(index.second)];
            Put(out,v.haveResult); Put(out,v.haveMatrix); Put(out,v.matrixTarget);
            if(v.matrixTarget) Put(out,v.matrix); else Array(out,v.result);
            Put(out,v.revision.resultStatus); Put(out,v.revision.status); return;
        } break;
    case D::WeightPacket:
        if(slot<B.weightPackets.size()) { Put(out,B.weightPackets[slot]); return; } break;
    case D::WeightFrames:
        if(slot<B.volumePlacement.size()) { Put(out,B.volumePlacement[slot]); return; } break;
    case D::WeightFramesBase:
        if(slot<B.volumePlacementBase.size()) { Put(out,B.volumePlacementBase[slot]); return; } break;
    case D::PoseWeight:
        if(slot<B.poseWeights.size()) { Put(out,B.poseWeights[slot]); return; } break;
    case D::WeightField:
        if(slot<B.weightFields.size()) { const auto &v=B.weightFields[slot];
            Put(out,uint64_t(v.count)); Put(out,v.ok); Put(out,v.error);
            Array(out,v.values); return; } break;
    case D::PropertyResult:
        // The immutable compiled map separates alias IDs from direct versions.
        // They are aliases, not entries maintained in propertyValues.
        if(slot<B.propertyRecordById.size() && B.propertyRecordById[slot]>=0) {
            const size_t r=size_t(B.propertyRecordById[slot]);
            if(r>=B.recordValues.size() || r>=B.recordStoodAside.size()) break;
            const auto &record=B.propertyRecords[r];
            if(record.chain>=B.propertyChains.size()) break;
            const auto &chain=B.propertyChains[record.chain];
            const uint32_t source=chain.versionBase+uint32_t(std::min(record.applied,chain.revisions.size()));
            if(source>=B.propertyVersionValid.size()) break;
            Put(out,uint8_t(1)); Put(out,source); Put(out,B.propertyVersionValid[source]);
            Put(out,B.recordStoodAside[r]); Box(out,B.recordValues[r]); return;
        }
        if(slot<B.propertyValues.size() && slot<B.propertyVersionValid.size()) {
            const auto found=std::upper_bound(B.propertyChains.begin(),B.propertyChains.end(),slot,
                [](uint32_t id,const auto &chain) { return id<chain.versionBase; });
            if(found==B.propertyChains.begin()) break;
            const auto &chain=*(found-1);
            if(slot>chain.versionBase+chain.revisions.size()) break;
            const auto &v=B.propertyValues[slot]; Put(out,B.propertyVersionValid[slot]);
            Put(out,chain.arm);
            switch(chain.arm) {
            case RigExecBakedPropertyChain::Arm::Float: Put(out,v.f); break;
            case RigExecBakedPropertyChain::Arm::Double: Put(out,v.d); break;
            case RigExecBakedPropertyChain::Arm::Vec3f: Put(out,v.v); break;
            case RigExecBakedPropertyChain::Arm::Matrix4d: Put(out,v.m); break;
            }
            return; } break;
    case D::Rest:
        if(slot<B.restM.size() && slot<B.restFrames.size() && slot<B.restPts.size()) {
            Put(out,B.restM[slot]); Put(out,B.restFrames[slot]);
            PutRun(out,B.restPts[slot].data(),B.restPts[slot].size()); return; } break;
    case D::Ladder:
        if(slot<B.restRoundTrip.size() && slot<B.defaultRoundTrip.size() &&
            slot<B.selfD.size() && slot<B.parentDinv.size() &&
            slot<B.posedAuthored.size() && slot<B.posedAuthoredM.size() &&
            slot<B.rotOrder.size() && slot<B.posedD.size() &&
            slot<B.parentSpaceM.size() && slot<B.parentSpaceAuthored.size() &&
            slot<B.rotationSign.size()) {
            Put(out,B.restRoundTrip[slot]); Put(out,B.defaultRoundTrip[slot]);
            Put(out,B.selfD[slot]); Put(out,B.parentDinv[slot]);
            Put(out,B.posedAuthored[slot]); Put(out,B.posedAuthoredM[slot]);
            Put(out,B.rotOrder[slot]); Put(out,B.posedD[slot]);
            Put(out,B.parentSpaceM[slot]); Put(out,B.parentSpaceAuthored[slot]);
            Put(out,B.rotationSign[slot]); return; } break;
    case D::SkinTopology:
        if(slot<B.revisionIndex.size()+B.derivedIndex.size()) {
            const auto *v=LayoutRevision(B,slot);
            if(!v) break; Put(out,v->layoutFixed); Put(out,bool(v->layoutHandle));
            if(v->layoutHandle) Put(out,*v->layoutHandle); return;
        } break;
    case D::RequiredStageFramesAdmission:
        if(slot == 0) { Put(out,B.requiredStageFramesAdmission.admitted);
            Put(out,B.requiredStageFramesAdmission.firstBadTarget); return; }
        break;
    case D::SwitchFrame:
        if(slot<B.switchFrames.size()){Put(out,B.switchFrames[slot]);return;}
        break;
    case D::SpaceValue:
        if(slot<B.providerValues.values.size()) {
            const auto &v=B.providerValues.values[slot];
            Put(out,v.initialized); Put(out,v.authoritative); Put(out,v.blocked);
            Put(out,uint64_t(v.count)); Put(out,v.error); Put(out,uint64_t(v.value.index()));
            std::visit([&](const auto &value) {
                using T=std::decay_t<decltype(value)>;
                if constexpr(std::is_same_v<T,VtValue>) Box(out,value);
                else if constexpr(!std::is_same_v<T,std::monostate>) Put(out,value);
            },v.value); return;
        } break;
    case D::ChainInput:
        if(slot<B.chains.size()) { const auto &v=B.chains[slot];
            Put(out,v.sampledHaveBase); Array(out,v.sampledBase); return; } break;
    case D::DerivedBase:
        if(slot<B.derivedIndex.size()) {
            const auto index=B.derivedIndex[slot];
            if(index.first>=0 && size_t(index.first)<B.chains.size() && index.second>=0 &&
                size_t(index.second)<B.chains[size_t(index.first)].derived.size()) {
                const auto &v=B.chains[size_t(index.first)].derived[size_t(index.second)];
                Put(out,v.sampledHaveBase); Array(out,v.sampledBase); return;
            }
        } break;
    case D::SpaceLeaf: RigExecBakedSpaceLeafKey(B,slot,out); return;
    case D::ConstraintInputs:
        if(slot<B.constraintArrays.size()) {
            for(const auto &value:B.constraintArrays[slot].raw) Box(out,value);
            return;
        } break;
    case D::Snapshots: break;
    }
    bad();
}
static bool InputKey(const RigExecBakedProgramImpl &B,
    const RigExecBakedStep &step,std::string *out,bool effective,std::vector<uint32_t> *covered,std::vector<std::pair<uint32_t,uint32_t>> *typed=nullptr,
    const RigExecBakedOpIdentityRemap *remap=nullptr,bool contentLeaves=false)
{
    if(!TF_VERIFY(out)) return false;
    out->clear(); bool exact=true;
    // Versions are program-local: a key read against another program keys
    // the leaves' contents.
    contentLeaves=contentLeaves || remap;
    if (remap && step.kind != RigExecBakedStepKind::RevisionStatic &&
        step.kind != RigExecBakedStepKind::RevisionChunk &&
        step.kind != RigExecBakedStepKind::RevisionFuse) return false;
    const auto identity = [&](const std::vector<int> &map, uint32_t id) {
        if (id >= map.size() || map[id] < 0) { exact=false; return id; }
        return uint32_t(map[id]);
    };
    if(covered)covered->clear();
    if(typed)typed->clear();
    auto coveredHead=[&](uint32_t id,bool slots) {
        if(!effective)return false;
        for(int index:step.readerWalks)if(index>=0 && size_t(index)<B.readerWalks.size()) {
            const auto &ids=slots?B.readerWalks[size_t(index)].slots:B.readerWalks[size_t(index)].leaves;
            if(std::find(ids.begin(),ids.end(),id)!=ids.end())return true;
        }
        return false;
    };

    auto invalid=[&] { Put(out,uint8_t(0)); exact=false; };
    auto coverCross=[&](int index) {
        if(!typed || index<0 || size_t(index)>=B.crossDomainReads.size())return;
        const auto &read=B.crossDomainReads[size_t(index)];using D=RigExecBakedSlotDomain;using K=RigExecCrossDomainRead::Kind;
        auto add=[&](D domain,uint32_t slot){typed->emplace_back(uint32_t(domain),slot);};
        if(read.kind==K::PointElement || read.kind==K::Points)for(const auto &p:read.points) {
            if(p.chain<0 || size_t(p.chain)>=B.chains.size())continue;
            if(read.finalPoints)add(D::ChainPoints,uint32_t(p.chain));
            else if(!p.version)add(D::ChainBase,uint32_t(p.chain));
            else {const auto id=B.chainRevisionBegin[size_t(p.chain)]+p.version-1;add(D::RevisionDone,uint32_t(id));add(D::ChainDirty,uint32_t(id));}
        } else if(read.kind==K::PoseFrame)for(auto frame:read.frames)add(read.baseFrame?D::PoseBase:D::PoseFin,frame);
        else if(read.kind==K::PropertyResult)add(D::PropertyResult,read.propertyVersion);
        else if(read.spaceValue>=0)add(D::SpaceValue,uint32_t(read.spaceValue));
    };
    auto coverWalk=[&](const RigExecBakedWalk &walk) {
        for(const auto *hops:{&walk.hops,&walk.doubleHops})for(const auto &hop:*hops) {
            coverCross(hop.crossDomain);
            if(covered && hop.chain>=0 && size_t(hop.chain)<B.propertyChains.size()) {
                const auto &chain=B.propertyChains[size_t(hop.chain)];covered->push_back(chain.versionBase+uint32_t(chain.revisions.size()));
            }
            if(covered && hop.record>=0 && size_t(hop.record)<B.propertyRecords.size())covered->push_back(B.propertyRecords[size_t(hop.record)].id);
        }
    };
    if(effective && step.kind==RigExecBakedStepKind::ComposeSubtree) {
        // Composition consumes pose/default tables, not the rest round trip
        // retained in the broader Ladder value for its other consumers.
        for(const auto &range:step.reads) {
            if(range.domain!=RigExecBakedSlotDomain::Ladder) continue;
            if(range.begin>range.end || range.end>B.paths.size() ||
               range.end>B.defaultRoundTrip.size() || range.end>B.selfD.size() ||
               range.end>B.parentDinv.size() || range.end>B.posedAuthored.size() ||
               range.end>B.posedAuthoredM.size() || range.end>B.rotOrder.size() ||
               range.end>B.posedD.size() || range.end>B.parentSpaceM.size() ||
               range.end>B.parentSpaceAuthored.size() || range.end>B.rotationSign.size()) {
                invalid(); continue;
            }
            for(uint32_t slot=range.begin;slot<range.end;++slot) {
                Put(out,uint32_t(RigExecBakedSlotDomain::Ladder)); Put(out,slot);
                Put(out,B.defaultRoundTrip[slot]); Put(out,B.selfD[slot]);
                Put(out,B.parentDinv[slot]); Put(out,B.posedAuthored[slot]);
                Put(out,B.posedAuthoredM[slot]); Put(out,B.rotOrder[slot]);
                Put(out,B.posedD[slot]); Put(out,B.parentSpaceM[slot]);
                Put(out,B.parentSpaceAuthored[slot]); Put(out,B.rotationSign[slot]);
                if(typed) typed->emplace_back(uint32_t(RigExecBakedSlotDomain::Ladder),slot);
            }
        }
    }
    if(effective && step.kind==RigExecBakedStepKind::PropertyRevision) {
        if(step.object<0 || size_t(step.object)>=B.propertyChains.size())return false;
        const auto &chain=B.propertyChains[size_t(step.object)];Put(out,chain.targetExists);
        if(step.part==0) {
            VtValue selected;
            if(chain.targetSlot>=0 && size_t(chain.targetSlot)<B.headOverrides.size() && !B.headOverrides[size_t(chain.targetSlot)].IsEmpty())
                selected=RigExecPhasedConsumerValue(B.headOverrides[size_t(chain.targetSlot)],chain.valueType);
            const bool usable=chain.arm==RigExecBakedPropertyChain::Arm::Float?selected.IsHolding<float>():chain.arm==RigExecBakedPropertyChain::Arm::Double?selected.IsHolding<double>():chain.arm==RigExecBakedPropertyChain::Arm::Vec3f?selected.IsHolding<GfVec3f>():selected.IsHolding<GfMatrix4d>();
            if(!usable)selected=chain.ownLeaf>=0 && size_t(chain.ownLeaf)<B.headLeaves.size()?B.headLeaves[size_t(chain.ownLeaf)].value:VtValue();
            return Box(out,selected);
        }
        if(step.part<0 || size_t(step.part)>chain.revisions.size())return false;
        const auto &revision=chain.revisions[size_t(step.part)-1];
        auto selected=[&](const RigExecBakedWalk &walk,auto fallback) {
            coverWalk(walk);auto value=fallback;VtValue selectedValue;
            if(RigExecBakedResolveWalkValue(B,walk,&selectedValue) && selectedValue.IsHolding<decltype(value)>())value=selectedValue.UncheckedGet<decltype(value)>();
            exact=Box(out,VtValue(value))&&exact;return value;
        };
        // Diagnostics read all cross-domain declarations before numerical admission.
        for(const auto *walk:{&revision.enabled,&revision.defaultWeight,&revision.value,&revision.minimum,&revision.maximum,&revision.keys,&revision.tangents}) {
            coverWalk(*walk);
            for(const auto *hops:{&walk->hops,&walk->doubleHops})for(const auto &hop:*hops)if(hop.crossDomain>=0){VtValue value;std::string diagnostic;const bool available=RigExecBakedReadCrossDomain(B,hop.crossDomain,&value,&diagnostic);Put(out,available);Put(out,diagnostic);}
        }
        Put(out,revision.moverExists);
        if(!revision.moverExists)return exact;
        const bool enabled=selected(revision.enabled,true);if(!enabled)return exact;
        if(revision.weightObject.IsEmpty())selected(revision.defaultWeight,1.0f);
        if(chain.arm==RigExecBakedPropertyChain::Arm::Matrix4d)selected(revision.value,GfMatrix4d(1));
        else if(chain.arm==RigExecBakedPropertyChain::Arm::Vec3f){selected(revision.value,GfVec3f(0));selected(revision.minimum,GfVec3f(0));selected(revision.maximum,GfVec3f(0));}
        else {selected(revision.value,0.0f);selected(revision.minimum,0.0f);selected(revision.maximum,0.0f);if(revision.op==RigExecPropertyOp::Curve){selected(revision.keys,VtArray<GfVec2f>());if(revision.hasTangents)selected(revision.tangents,VtArray<GfVec2f>());}}
        return exact;
    }
    if(effective)for(int index:step.readerWalks)if(index>=0 && size_t(index)<B.readerWalks.size())coverWalk(B.readerWalks[size_t(index)].walk);
    if(effective && step.kind==RigExecBakedStepKind::WeightField) {
        if(step.object<0 || size_t(step.object)>=B.weightFields.size())return false;
        const auto &field=B.weightFields[size_t(step.object)];
        RigExecWeightPointView entering;size_t count=0;
        const bool context=RigExecBakedCaptureWeightFieldInputs(B,step.object,&field.effectiveInputs,&entering,&count);
        Put(out,context);Put(out,count);
        auto points=[&](const RigExecWeightPointInput &p) {
            Put(out,p.declared);Put(out,p.available);Put(out,p.count);
            if(p.available && p.count && !p.data){exact=false;return;}
            if(p.available)PutRun(out,p.data,p.count);
        };
        for(int id:field.objects) {
            if(id<0 || size_t(id)>=B.weightProgram.size() || size_t(id)>=field.effectiveInputs.size())return false;
            const auto &record=B.weightProgram[size_t(id)];const auto &input=field.effectiveInputs[size_t(id)];
            Put(out,id);Put(out,input.blocked);
            auto scalar=[&](int i){Put(out,input.scalars[size_t(i)]);};
            if(record.kind==0)scalar(0);
            else if(record.kind==1){scalar(1);scalar(2);scalar(3);}
            else if(record.kind==2){scalar(4);scalar(5);}
            else {
                Put(out,input.hasPlacement);if(input.hasPlacement)Put(out,input.placement);
                const auto &target=input.phasedPoints[1];const auto &primary=input.phasedPoints[0];
                if(target.declared){Put(out,uint8_t(1));points(target);}
                else if(record.samplesInFlight){Put(out,uint8_t(3));Put(out,entering.count);if(entering.count&&!entering.data)return false;PutRun(out,entering.data,entering.count);}
                else if(primary.declared && primary.available){Put(out,uint8_t(2));points(primary);}
                else {const auto &raw=(!primary.declared && input.rawPoints[0].available)?input.rawPoints[0]:input.rawPoints[1];Put(out,uint8_t(4));points(raw);}
                for(int i=4;i<=10;++i)scalar(i);
                if(record.kind==3)for(int i=11;i<=16;++i)scalar(i);
                if(record.kind==4){Put(out,input.axis);Put(out,input.bounds);if(input.bounds=="bounded"){scalar(17);scalar(18);}}
                if(record.kind==5)points(input.phasedPoints[2].declared?input.phasedPoints[2]:input.rawPoints[2]);
            }
        }
        if(covered) {
            RigExecBakedWeightFieldCoveredVersions(B,step.object,covered);
            std::sort(covered->begin(),covered->end());covered->erase(std::unique(covered->begin(),covered->end()),covered->end());
        }
        return exact;
    }
    if(effective && step.kind==RigExecBakedStepKind::ProviderRefresh) {
        if(step.object<0 || size_t(step.object)>=B.providerRefreshes.size()) {invalid();return false;}
        const auto &refresh=B.providerRefreshes[size_t(step.object)];
        for(const auto &carry:refresh.carries) for(int slot:carry.blockingSlots) {
            if(slot<0 || size_t(slot)>=B.ladders.size()) {invalid();continue;}
            const auto &input=B.ladders[size_t(slot)].parentSpace;
            Put(out,slot); Put(out,RigExecBakedProviderParentRaw(B,slot));
            if(covered && input.walk>=0 && size_t(input.walk)<B.readerWalks.size()) {
                const auto &versions=B.readerWalks[size_t(input.walk)].versions;
                covered->insert(covered->end(),versions.begin(),versions.end());
            }
        }
        return exact;
    }
    if(effective && step.kind==RigExecBakedStepKind::SpaceExpression && step.part==4)
        return true; // Exact typed frame dependencies are appended by the adapter.
    if(effective && step.kind==RigExecBakedStepKind::SpaceExpression) {
        if(step.object<0)invalid();
        else if(step.part==0) {
            if(size_t(step.object)>=B.providerProgram.ops.size())invalid();
            else {
                const auto &op=B.providerProgram.ops[size_t(step.object)];Put(out,op.kind);
                auto input=[&](size_t i) {return i<op.inputs.size()?op.inputs[i]:RigExecNoProviderValue;};
                auto state=[&](RigExecValueId id) {
                    Put(out,id);const bool exists=id<B.providerValues.values.size();Put(out,exists);
                    if(!exists)return;
                    const auto &v=B.providerValues.values[size_t(id)];
                    Put(out,v.initialized);Put(out,v.blocked);Put(out,v.authoritative);Put(out,uint64_t(v.count));Put(out,v.error);
                    Put(out,uint64_t(v.value.index()));
                    std::visit([&](const auto &value) {
                        using T=std::decay_t<decltype(value)>;
                        if constexpr(std::is_same_v<T,VtValue>)exact=Box(out,value)&&exact;
                        else if constexpr(!std::is_same_v<T,std::monostate>)Put(out,value);
                    },v.value);
                    exact=Box(out,v.raw)&&exact;
                };
                auto matrix=[&](RigExecValueId id) {return B.providerValues.Read<GfMatrix4d>(id);};
                auto authority=[&](RigExecValueId id) {return id<B.providerValues.values.size() && B.providerValues.values[size_t(id)].authoritative;};
                if(op.kind==RigExecProviderOpKind::Attribute) {
                    const bool authored=authority(input(0));Put(out,authored);
                    state(authored || input(1)==RigExecNoProviderValue?input(0):input(1));
                } else if(op.kind==RigExecProviderOpKind::SpaceExpression) {
                    const auto *raw=matrix(input(0)),*connected=matrix(input(1));
                    const uint8_t selected=authority(input(0)) && raw?0:connected?1:raw && *raw!=GfMatrix4d(1)?2:3;
                    Put(out,selected);state(selected==0 || selected==2?input(0):selected==1?input(1):input(2));
                } else if(op.kind==RigExecProviderOpKind::PosedFrame) {
                    const auto *posed=matrix(input(0));const bool selected=posed && (authority(input(0)) || *posed!=GfMatrix4d(1));
                    Put(out,selected);state(input(0));
                    if(!selected) {
                        bool unavailable=false;
                        for(size_t i=1;i<=3;++i){state(input(i));if(input(i)!=RigExecNoProviderValue&&!matrix(input(i))){unavailable=true;break;}}
                        if(!unavailable)for(size_t i=4;i<op.inputs.size();++i)if(op.scaleAvars || i<8 || i>10)state(op.inputs[i]);
                    }
                } else if(op.kind==RigExecProviderOpKind::AvarMatrix) {
                    for(size_t i=0;i<op.inputs.size();++i)if(op.scaleAvars || i<4 || i>6)state(op.inputs[i]);
                } else if(op.kind==RigExecProviderOpKind::RestFrame || op.kind==RigExecProviderOpKind::DefaultSpace || op.kind==RigExecProviderOpKind::JointMatrix) {
                    const bool rest=op.kind==RigExecProviderOpKind::RestFrame;
                    const size_t first=rest?7:0, frames=op.kind==RigExecProviderOpKind::JointMatrix?2:rest?1:2;
                    bool unavailable=false;
                    for(size_t i=first;i<first+frames;++i){state(input(i));if(input(i)!=RigExecNoProviderValue&&!B.providerValues.Read<RigExecPointFrame>(input(i))){unavailable=true;break;}}
                    const size_t m=rest?8:2;
                    if(!unavailable && op.kind!=RigExecProviderOpKind::JointMatrix){state(input(m));unavailable=input(m)!=RigExecNoProviderValue&&!matrix(input(m));}
                    if(!unavailable && op.kind!=RigExecProviderOpKind::JointMatrix)for(size_t i=rest?0:3;i<(rest?7:op.inputs.size());++i)state(input(i));
                } else if(op.kind==RigExecProviderOpKind::RelativeXform) {
                    for(size_t i=0;i<op.inputs.size();i+=2) {
                        state(op.inputs[i]);if(!matrix(op.inputs[i]))break;
                        if(i+1<op.inputs.size()) {
                            const auto id=op.inputs[i+1];state(id);
                            if(id<B.providerValues.values.size()) {
                                const auto &order=B.providerValues.values[size_t(id)];
                                const auto &raw=order.raw;
                                if(order.initialized && !order.blocked && raw.IsHolding<VtTokenArray>()) {
                                    bool reset=false;for(const auto &token:raw.UncheckedGet<VtTokenArray>())if(token=="!resetXformStack!"){reset=true;break;}
                                    if(reset)break;
                                }
                            }
                        }
                    }
                } else {
                    for(const auto id:op.inputs)state(id);
                    for(const auto &xform:op.xforms)state(xform.raw);
                }
            }
        } else if(step.part==1) {
            const size_t id=size_t(step.object);
            if(id>=B.providerProgram.externalInputs.size() || id>=B.providerExternalSlots.size())invalid();
            else {
                const auto &input=B.providerProgram.externalInputs[id];
                const int slot=B.providerExternalSlots[id];
                Put(out,input.computation);Put(out,slot);
                if(input.computation=="computeRestFrame") {
                    if(slot<0)Put(out,RigExecFrameFromMatrix(GfMatrix4d(1.0)));
                    else if(size_t(slot)>=B.restFrames.size())invalid();
                    else Put(out,B.restFrames[size_t(slot)]);
                } else if(input.computation=="computePointFrame" || input.computation=="computeBasePointFrame") {
                    // Same bound SSA seed as RunSpaceOp; flags and all landmarks
                    // are semantic inputs. The adapter retains every typed edge.
                    if(slot<0)Put(out,RigExecFrameFromMatrix(GfMatrix4d(1.0)));
                    else if(size_t(slot)>=B.base.size())invalid();
                    else Put(out,B.base[size_t(slot)]);
                } else if(input.computation=="interveningSpace") {
                    // This bridge consumes its separately sampled matrix leaf.
                    // Keep the generic binding-leaf key and typed reasons below.
                    if(slot<0)Put(out,GfMatrix4d(1.0));
                    else if(size_t(slot)>=B.ladders.size())invalid();
                    else Put(out,RigExecBakedLeafRead(B,B.ladders[size_t(slot)].interveningSpace));
                } else invalid();
            }
        } else if(step.part==2) {
            const size_t id=size_t(step.object);
            if(id>=B.providerProgram.sampled.size() || id>=B.providerLeaves.values.size() || id>=B.providerLeafBlocked.size())invalid();
            else {
                const auto *overlay=RigExecBakedSpaceLeafOverlay(B,id);Put(out,overlay!=nullptr);
                Put(out,bool(!overlay && B.providerLeafBlocked[id]));exact=Box(out,overlay?*overlay:B.providerLeaves.values[id])&&exact;
            }
        } else invalid();
    }
    auto head=[&](uint32_t id) {
        Put(out,remap?identity(remap->headLeaves,id):id); if(coveredHead(id,false)) {Put(out,uint8_t(2));return;} if(id>=B.headLeaves.size()) { invalid(); return; }
        Put(out,uint8_t(1)); const auto &v=B.headLeaves[id];
        Put(out,v.typeMatches); exact=Box(out,v.value)&&exact;
    };
    auto overrideValue=[&](uint32_t id) {
        Put(out,remap?identity(remap->overrides,id):id); if(coveredHead(id,true)) {Put(out,uint8_t(2));return;} if(id>=B.headOverrides.size()) { invalid(); return; }
        Put(out,uint8_t(1)); exact=Box(out,B.headOverrides[id])&&exact;
    };
    if(step.kind==RigExecBakedStepKind::AvarInputs) {
        Put(out,uint8_t(26)); Put(out,step.object);
        auto bindings=[&](const auto &values,const std::vector<uint32_t> &begin) {
            const auto range=RigExecBakedAvarBindingRange(begin,step.object);
            Put(out,uint64_t(range.second-range.first));
            for(size_t b=range.first;b<range.second;++b) {
                const auto &binding=values[b];
                Put(out,uint64_t(binding.slot)); const auto &input=binding.input;
                // Include the direct sampled pool even when NoteInput omitted it.
                Put(out,input.leaf);
                if(!effective) Put(out,input.constant);
                if(effective && input.walk>=0) {
                    if(size_t(input.walk)>=B.readerWalks.size()) { invalid(); continue; }
                    // Same pure selection as the AvarInputs body's LeafRead.
                    // A successful walk shadows the stored fallback entirely.
                    double value=input.constant;
                    Put(out,RigExecBakedResolveReaderWalk(B,input.walk,&value));
                    Put(out,value);
                    if(covered && size_t(input.walk)<B.readerWalks.size()) {
                        const auto &versions=B.readerWalks[size_t(input.walk)].versions;
                        covered->insert(covered->end(),versions.begin(),versions.end());
                    }
                    continue;
                }

                if(input.leaf>=0) {
                    const auto &pool=B.leaves.Of<double>().value;
                    if(size_t(input.leaf)>=pool.size()) invalid();
                    else Put(out,pool[size_t(input.leaf)]);
                } else if(effective) Put(out,input.constant);
            }
        };
        bindings(B.avarBindings,B.avarBindingBegin);
        bindings(B.avarConstantBindings,B.avarConstantBindingBegin);
    }
    if(step.kind==RigExecBakedStepKind::Constraint) {
        Put(out,uint8_t(36)); Put(out,step.object);
        const int index = step.object >= 0 && size_t(step.object) < B.walkSteps.size()
            && !B.walkSteps[size_t(step.object)].solverBatch
            ? B.walkSteps[size_t(step.object)].index : -1;
        if(index < 0 || size_t(index) >= B.constraints.size()) invalid();
        else {
            const auto &constraint=B.constraints[size_t(index)];
            auto native=[&](int id) {
                Put(out,id); if(id<0) return;
                if(size_t(id)>=B.nativeFrames.size() || size_t(id)>=B.nativeFrameOk.size()) { invalid(); return; }
                Put(out,B.nativeFrameOk[size_t(id)]); Put(out,B.nativeFrames[size_t(id)]);
            };
            Put(out,uint64_t(constraint.sourceNatives.size()));
            for(int id:constraint.sourceNatives) native(id);
            native(constraint.worldUpNative); native(constraint.effectorNative);
            Put(out,uint64_t(constraint.poleObjectNatives.size()));
            for(int id:constraint.poleObjectNatives) native(id);
            Put(out,constraint.deltaBase);
            if(constraint.deltaBase>=0) {
                const size_t id=size_t(constraint.deltaBase);
                if(id>=B.deltaBaseOk.size() || id>=B.deltaBaseMatrix.size()) invalid();
                else { Put(out,B.deltaBaseOk[id]); Put(out,B.deltaBaseMatrix[id]); }
            }
        }
    }
    // Outside the effective branches, the source key reads state only in the
    // AvarInputs and Constraint branches, these four lists and the weight
    // overlay toggle. RigExecBakedOpInputKeyIsConstant tests the same gates.
    Put(out,uint64_t(step.bindingLeaves.size()));
    for(uint32_t id:step.bindingLeaves) {
        Put(out,remap?identity(remap->bindingLeaves,id):id);
        if(id<B.leafRefs.size()) {
            const auto &ref=B.leafRefs[id]; Put(out,ref.type);
            Put(out,remap?identity(remap->bindingRefIndices,id):ref.index);
            if(effective && std::find(step.walkedBindingLeaves.begin(),step.walkedBindingLeaves.end(),
                std::make_pair(ref.type,ref.index))!=step.walkedBindingLeaves.end()) {Put(out,uint8_t(2));continue;}

            auto pool=[&](const auto &p) {
                if(ref.index>=p.value.size()) { invalid(); return; }
                Put(out,uint8_t(1)); Put(out,p.value[ref.index]);
            };
            switch(ref.type) {
            case RigExecBakedLeafType::Double: pool(B.leaves.Of<double>()); break;
            case RigExecBakedLeafType::Float: pool(B.leaves.Of<float>()); break;
            case RigExecBakedLeafType::Int: pool(B.leaves.Of<int>()); break;
            case RigExecBakedLeafType::Bool: pool(B.leaves.Of<bool>()); break;
            case RigExecBakedLeafType::Token: pool(B.leaves.Of<TfToken>()); break;
            case RigExecBakedLeafType::Matrix4d: pool(B.leaves.Of<GfMatrix4d>()); break;
            case RigExecBakedLeafType::Vec3d: pool(B.leaves.Of<GfVec3d>()); break;
            case RigExecBakedLeafType::Vec3f: pool(B.leaves.Of<GfVec3f>()); break;
            default: invalid(); break;
            }
        } else {
            size_t offset=size_t(id)-B.leafRefs.size();
            if(offset<B.pathLeafRefs.size()) {
                const auto &ref=B.pathLeafRefs[offset];
                const auto *leaves=RigExecBakedPathLeavesOf(B,ref);
                if(!leaves || ref.key>=leaves->values.size()) { invalid(); continue; }
                // The body first adopts the current layoutHandle, which makes
                // the assembler return before raw
                // layout reads. The typed SkinTopology edge carries that
                // selected value; the frozen source table may omit these rows.
                if(effective && step.kind==RigExecBakedStepKind::RevisionStatic &&
                    step.object>=0 && size_t(step.object)<B.revisionIndex.size()) {
                    const auto *revision=Revision(B,uint32_t(step.object));
                    if(revision && revision->op==RigExecRevisionOp::Skin &&
                       revision->layoutHandle &&
                       leaves==&revision->leaves &&
                       (int(ref.key)==leaves->decl.Role(RigExecRevisionLeafRole::JointIndices) ||
                        int(ref.key)==leaves->decl.Role(RigExecRevisionLeafRole::JointWeights) ||
                        int(ref.key)==leaves->decl.Role(RigExecRevisionLeafRole::ElementSize))) {
                        Put(out,uint8_t(2)); continue;
                    }
                }
                // A geometry owner's raw value keys as its content version,
                // which moves exactly when the bytes Box writes would. The
                // effective tier resolves a key through no walk, record or
                // produced version to that same raw value.
                const auto at=[](const std::vector<int> &v,uint32_t k) { return k<v.size()?v[k]:-1; };
                const bool versioned=!contentLeaves && RigExecBakedPathLeafVersioned(ref) &&
                    (!effective || (ref.key<leaves->decl.keys.size() && at(leaves->walks,ref.key)<0 &&
                     at(leaves->exactVersions,ref.key)<0 && at(leaves->exactRecordIndices,ref.key)<0));
                if(versioned) {
                    if(ref.key>=leaves->versions.size()) { invalid(); continue; }
                    Put(out,uint8_t(3)); Put(out,leaves->versions[ref.key]);
                    exact=BoxExact(leaves->values[ref.key])&&exact;
                    continue;
                }
                Put(out,uint8_t(1));
                if(effective) {
                    exact=Box(out,RigExecBakedResolvePathLeaf(B,*leaves,ref.key))&&exact;
                    if(covered && ref.key<leaves->exactVersions.size() && leaves->exactVersions[ref.key]>=0)
                        covered->push_back(uint32_t(leaves->exactVersions[ref.key]));
                    if(covered && ref.key<leaves->exactRecordIndices.size()) {
                        const int r=leaves->exactRecordIndices[ref.key];
                        if(r>=0 && size_t(r)<B.propertyRecords.size()) {
                            const auto &record=B.propertyRecords[size_t(r)];
                            if(record.chain<B.propertyChains.size()) {
                                const auto &chain=B.propertyChains[record.chain];
                                covered->push_back(record.id);
                                covered->push_back(chain.versionBase+uint32_t(std::min(record.applied,chain.revisions.size())));
                            }
                        }
                    }
                    if(covered && ref.key<leaves->walks.size()) {
                        const int walk=leaves->walks[ref.key];
                        if(walk>=0 && size_t(walk)<B.readerWalks.size()) {
                            const auto &versions=B.readerWalks[size_t(walk)].versions;
                            covered->insert(covered->end(),versions.begin(),versions.end());
                        }
                    }
                } else exact=Box(out,leaves->values[ref.key])&&exact;
            } else head(uint32_t(offset-B.pathLeafRefs.size()));
        }
    }
    Put(out,uint64_t(step.leaves.size())); for(auto id:step.leaves) head(id);
    Put(out,uint64_t(step.overrideSlots.size())); for(auto id:step.overrideSlots) overrideValue(id);
    Put(out,uint64_t(step.readerWalks.size()));
    for(int index:step.readerWalks) {
        Put(out,remap && index>=0?int(identity(remap->readerWalks,uint32_t(index))):index);
        if(index<0 || size_t(index)>=B.readerWalks.size()) { invalid(); continue; }
        const auto &walk=B.readerWalks[size_t(index)];
        if(effective) {
            if(covered)covered->insert(covered->end(),walk.versions.begin(),walk.versions.end());
            Put(out,walk.walk.type);
            auto selected=[&](auto value) {
                const bool present=RigExecBakedResolveReaderWalk(B,index,&value);
                Put(out,present);exact=Box(out,VtValue(value))&&exact;
            };
            switch(walk.walk.type) {
            case RigExecBakedHeadValueType::Bool:selected(false);break;
            case RigExecBakedHeadValueType::Float:selected(float(0));break;
            case RigExecBakedHeadValueType::Double:selected(double(0));break;
            case RigExecBakedHeadValueType::Int:selected(int(0));break;
            case RigExecBakedHeadValueType::Token:selected(TfToken());break;
            case RigExecBakedHeadValueType::Vec3f:selected(GfVec3f(0));break;
            case RigExecBakedHeadValueType::Vec3d:selected(GfVec3d(0));break;
            case RigExecBakedHeadValueType::Matrix4d:selected(GfMatrix4d(1));break;
            case RigExecBakedHeadValueType::Vec3fArray:selected(VtVec3fArray());break;
            case RigExecBakedHeadValueType::Vec2fArray:selected(VtArray<GfVec2f>());break;
            }
            continue;
        }
        Put(out,uint64_t(walk.leaves.size())); for(auto id:walk.leaves) head(id);
        Put(out,uint64_t(walk.slots.size())); for(auto id:walk.slots) overrideValue(id);
    }
    if(covered) {std::sort(covered->begin(),covered->end());covered->erase(std::unique(covered->begin(),covered->end()),covered->end());}
    return exact;
}
// A held-time overlay toggle changes assembly publication without
// changing numerical leaves. Both key tiers must observe that input.
static bool KeysWeightOverlay(const RigExecBakedProgramImpl &B,const RigExecBakedStep &step)
{
    if(step.kind!=RigExecBakedStepKind::RevisionStatic || step.object<0 || size_t(step.object)>=B.revisionIndex.size())
        return false;
    const auto &id=B.revisionIndex[size_t(step.object)];
    return B.chains[size_t(id.first)].revisions[size_t(id.second)].weightObject>=0;
}
bool RigExecBakedOpInputKey(const RigExecBakedProgramImpl &B,const RigExecBakedStep &step,std::string *out,const RigExecBakedOpIdentityRemap *remap,bool contentLeaves) {
    const bool exact=InputKey(B,step,out,false,nullptr,nullptr,remap,contentLeaves);
    if(KeysWeightOverlay(B,step)) RigExecOpKeyAppend(out,B.publishWeightFields);
    return exact;
}
// The gates of every state read in the non-effective InputKey plus the
// overlay toggle above. Without one, the key is the four zero list counts.
bool RigExecBakedOpInputKeyIsConstant(const RigExecBakedProgramImpl &B,const RigExecBakedStep &step) {
    return step.kind!=RigExecBakedStepKind::AvarInputs && step.kind!=RigExecBakedStepKind::Constraint &&
        step.bindingLeaves.empty() && step.leaves.empty() && step.overrideSlots.empty() &&
        step.readerWalks.empty() && !KeysWeightOverlay(B,step);
}
bool RigExecBakedOpEffectiveInputKey(const RigExecBakedProgramImpl &B,const RigExecBakedStep &step,
    std::string *out,std::vector<uint32_t> *covered,std::vector<std::pair<uint32_t,uint32_t>> *typed,const RigExecBakedOpIdentityRemap *remap,
    bool contentLeaves) {
    const bool exact=InputKey(B,step,out,true,covered,typed,remap,contentLeaves);
    if(KeysWeightOverlay(B,step)) RigExecOpKeyAppend(out,B.publishWeightFields);
    return exact;
}
} // namespace rigExec
