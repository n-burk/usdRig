#include "inputReplayValues.h"
#include "pxr/base/vt/array.h"
#include "pxr/base/vt/dictionary.h"
#include "pxr/base/tf/token.h"
#include "pxr/base/gf/half.h"
#include "pxr/base/gf/timeCode.h"
#include "pxr/usd/sdf/path.h"
#include "pxr/usd/sdf/assetPath.h"
#include "pxr/usd/sdf/types.h"
#include "pxr/usd/sdf/listOp.h"
#include "pxr/usd/sdf/reference.h"
#include "pxr/usd/sdf/payload.h"
#include "pxr/usd/sdf/layerOffset.h"
#include "pxr/base/gf/vec2i.h"
#include "pxr/base/gf/vec2h.h"
#include "pxr/base/gf/vec2f.h"
#include "pxr/base/gf/vec2d.h"
#include "pxr/base/gf/vec3i.h"
#include "pxr/base/gf/vec3h.h"
#include "pxr/base/gf/vec3f.h"
#include "pxr/base/gf/vec3d.h"
#include "pxr/base/gf/vec4i.h"
#include "pxr/base/gf/vec4h.h"
#include "pxr/base/gf/vec4f.h"
#include "pxr/base/gf/vec4d.h"
#include "pxr/base/gf/matrix2f.h"
#include "pxr/base/gf/matrix2d.h"
#include "pxr/base/gf/matrix3f.h"
#include "pxr/base/gf/matrix3d.h"
#include "pxr/base/gf/matrix4f.h"
#include "pxr/base/gf/matrix4d.h"
#include "pxr/base/gf/quath.h"
#include "pxr/base/gf/quatf.h"
#include "pxr/base/gf/quatd.h"
#include <cstdint>
#include <cmath>
#include <algorithm>
#include <cstring>
#include <limits>
#include <stdexcept>
#include <type_traits>
#include <vector>
#include "pxr/base/ts/spline.h"
#include "pxr/base/ts/knot.h"
#include "pxr/base/ts/knotMap.h"
#include "pxr/base/ts/raii.h"
PXR_NAMESPACE_USING_DIRECTIVE
namespace rigExec { namespace {
static_assert(sizeof(char)==1 && sizeof(short)==2 && sizeof(int)==4,
              "input codec requires explicit 8/16/32-bit builtin integers");
static_assert(sizeof(float)==4 && sizeof(double)==8 &&
              std::numeric_limits<float>::is_iec559 &&
              std::numeric_limits<double>::is_iec559,
              "input codec requires IEEE binary32 and binary64");
struct Codec {
    std::string bytes;
    size_t pos=0;
    unsigned depth=0;
    bool reading=false;
    void Fail(const std::string& why) { throw std::runtime_error(why); }
    void Raw(void* data,size_t n) {
        if(reading) {
            if(n>bytes.size()-pos) Fail("truncated input value");
            std::memcpy(data,bytes.data()+pos,n); pos+=n;
        } else bytes.append(static_cast<const char*>(data),n);
    }
    template<class T> void Number(T& v) {
        // Explicit little-endian scalar bytes; never serialize object padding.
        unsigned char b[sizeof(T)];
        const uint16_t one=1;
        const bool little=*reinterpret_cast<const unsigned char*>(&one)==1;
        if(!reading) { std::memcpy(b,&v,sizeof(T)); if(!little) for(size_t i=0;i<sizeof(T)/2;++i) std::swap(b[i],b[sizeof(T)-1-i]); }
        Raw(b,sizeof(T));
        if(reading) { if(!little) for(size_t i=0;i<sizeof(T)/2;++i) std::swap(b[i],b[sizeof(T)-1-i]); std::memcpy(&v,b,sizeof(T)); }
    }
    void Text(std::string& v) {
        uint64_t n=v.size(); Number(n);
        if(reading) { if(n>bytes.size()-pos) Fail("truncated input string"); v.assign(bytes.data()+pos,size_t(n));pos+=size_t(n); }
        else if(n) Raw(&v[0],size_t(n));
    }
    template<class T> std::enable_if_t<std::is_arithmetic_v<T> && !std::is_same_v<T,bool>> Item(T& v) { Number(v); }
    void Item(bool& v) { uint8_t x=v?1:0;Number(x);if(x>1)Fail("invalid input bool");v=x!=0; }
    void Item(GfHalf& v) { uint16_t x=v.bits();Number(x);if(reading)v.setBits(x); }
    void Item(std::string& v) {Text(v);}
    void Item(TfToken& v) {std::string x=v.GetString();Text(x);if(reading)v=TfToken(x);}
    void Item(SdfPath& v) {std::string x=v.GetString();Text(x);if(reading)v=SdfPath(x);}
    void Item(SdfAssetPath& v) {std::string a=v.GetAuthoredPath(),e=v.GetEvaluatedPath(),r=v.GetResolvedPath();Text(a);Text(e);Text(r);if(reading)v=SdfAssetPath(SdfAssetPathParams().Authored(a).Evaluated(e).Resolved(r));}
    void Item(GfTimeCode& v) {double x=v.GetValue();Item(x);if(reading)v=GfTimeCode(x);}
    void Item(SdfLayerOffset& v) {double o=v.GetOffset(),s=v.GetScale();Item(o);Item(s);if(reading)v=SdfLayerOffset(o,s);}
    void Item(SdfValueBlock&) {}
    template<class T> void Sequence(T& v) {
        uint64_t n=v.size();Number(n);
        // Every supported element has at least one encoded byte.
        if(reading) {if(n>bytes.size()-pos)Fail("invalid input sequence count");v.resize(size_t(n));}
        for(size_t i=0;i<size_t(n);++i) {typename T::value_type x=v[i];Item(x);if(reading)v[i]=x;}
    }
    template<class T> void Item(VtArray<T>& v) {Sequence(v);}
    template<class T> void Item(std::vector<T>& v) {Sequence(v);}
    void Item(VtValue& v);
    void Item(VtDictionary& v) {
        uint64_t n=v.size();Number(n);
        if(reading) {if(n>bytes.size()-pos)Fail("invalid dictionary count");v.clear();for(uint64_t i=0;i<n;++i){std::string k;VtValue x;Text(k);Item(x);if(!v.insert(std::make_pair(k,x)).second)Fail("duplicate dictionary key");}}
        else for(const auto& e:v){std::string k=e.first;VtValue x=e.second;Text(k);Item(x);}
    }
    void Item(SdfTimeSampleMap& v) {
        uint64_t n=v.size();Number(n);
        if(reading) {if(n>bytes.size()-pos)Fail("invalid time sample count");v.clear();for(uint64_t i=0;i<n;++i){double k=0;VtValue x;Item(k);Item(x);if(!std::isfinite(k))Fail("nonfinite time sample key");if(!v.emplace(k,x).second)Fail("duplicate time sample key");}}
        else for(const auto& e:v){double k=e.first;VtValue x=e.second;Item(k);Item(x);}
    }
    void Item(SdfVariantSelectionMap& v) {
        uint64_t n=v.size();Number(n);
        if(reading){if(n>bytes.size()-pos)Fail("invalid variant map count");v.clear();for(uint64_t i=0;i<n;++i){std::string k,x;Text(k);Text(x);if(!v.emplace(k,x).second)Fail("duplicate variant key");}}
        else for(const auto& e:v){std::string k=e.first,x=e.second;Text(k);Text(x);}
    }
    void Item(TsExtrapolation& v) {
        int32_t mode=int32_t(v.mode);Item(mode);if(mode<0||mode>6)Fail("invalid spline extrapolation");
        Item(v.slope);bool has=v.loopBoundaryTime.has_value();Item(has);
        double t=has?*v.loopBoundaryTime:0; if(has)Item(t);
        if(reading){v.mode=TsExtrapMode(mode);v.loopBoundaryTime=has?std::optional<double>(t):std::nullopt;}
    }
    void Item(TsSpline& v) {
        TsEditBehaviorBlock noAutomaticEdits;
        TsAntiRegressionAuthoringSelector noRegression(TsAntiRegressionNone);
        std::string type=v.GetValueType()?v.GetValueType().GetTypeName():std::string();Text(type);
        if(reading){const TfType t=TfType::FindByName(type);if(!type.empty()&&!t)Fail("unknown spline value type: "+type);v=t?TsSpline(t):TsSpline();}
        int32_t curve=int32_t(v.GetCurveType());Item(curve);if(curve<0||curve>1)Fail("invalid spline curve type");
        TsExtrapolation pre=v.GetPreExtrapolation(),post=v.GetPostExtrapolation();Item(pre);Item(post);
        TsLoopParams loop=v.GetInnerLoopParams();Item(loop.protoStart);Item(loop.protoEnd);Item(loop.numPreLoops);Item(loop.numPostLoops);Item(loop.valueOffset);
        auto knots=v.GetKnots();uint64_t n=knots.size();Number(n);
        if(reading){if(n>bytes.size()-pos)Fail("invalid spline knot count");knots=TsKnotMap();}
        auto it=knots.begin();
        for(uint64_t i=0;i<n;++i) {
            TsKnot k=reading?TsKnot(v.GetValueType()):*it++;
            double time=k.GetTime(),pw=k.GetPreTanWidth(),qw=k.GetPostTanWidth();
            int32_t interpolation=int32_t(k.GetNextInterpolation()),kc=int32_t(k.GetCurveType()),pa=int32_t(k.GetPreTanAlgorithm()),qa=int32_t(k.GetPostTanAlgorithm());
            bool dual=k.IsDualValued();VtValue val,pval,ps,qs;VtDictionary custom=k.GetCustomData();
            if(!reading){if(!k.GetValue(&val)||!k.GetPreTanSlope(&ps)||!k.GetPostTanSlope(&qs)||(dual&&!k.GetPreValue(&pval)))Fail("unreadable spline knot component");}
            Item(time);Item(interpolation);Item(kc);Item(val);Item(dual);if(dual)Item(pval);Item(pw);Item(qw);Item(ps);Item(qs);Item(pa);Item(qa);Item(custom);
            if(reading){
                if(interpolation<0||interpolation>3||kc<0||kc>1)Fail("invalid spline knot mode");
                if(!k.SetTime(time)||!k.SetNextInterpolation(TsInterpMode(interpolation))||!k.SetCurveType(TsCurveType(kc))||!k.SetValue(val)||!k.SetPreTanWidth(pw)||!k.SetPostTanWidth(qw)||!k.SetPreTanSlope(ps)||!k.SetPostTanSlope(qs)||!k.SetPreTanAlgorithm(TsTangentAlgorithm(pa))||!k.SetPostTanAlgorithm(TsTangentAlgorithm(qa))||!k.SetCustomData(custom)||(dual&&!k.SetPreValue(pval)))Fail("spline component reconstruction refused");
                if(!knots.insert(k).second)Fail("duplicate spline knot time");
            }
        }
        if(reading){v.SetCurveType(TsCurveType(curve));v.SetKnots(knots);v.SetPreExtrapolation(pre);v.SetPostExtrapolation(post);v.SetInnerLoopParams(loop);}
    }
    void Item(SdfReference& v) {std::string a=v.GetAssetPath();SdfPath p=v.GetPrimPath();SdfLayerOffset o=v.GetLayerOffset();VtDictionary d=v.GetCustomData();Item(a);Item(p);Item(o);Item(d);if(reading)v=SdfReference(a,p,o,d);}
    void Item(SdfPayload& v) {std::string a=v.GetAssetPath();SdfPath p=v.GetPrimPath();SdfLayerOffset o=v.GetLayerOffset();Item(a);Item(p);Item(o);if(reading)v=SdfPayload(a,p,o);}
    template<class T> void Item(SdfListOp<T>& v) {
        bool explicitMode=v.IsExplicit();Item(explicitMode);
        if(reading)v=SdfListOp<T>();
        const SdfListOpType kinds[]={SdfListOpTypeExplicit,SdfListOpTypeAdded,SdfListOpTypeDeleted,SdfListOpTypeOrdered,SdfListOpTypePrepended,SdfListOpTypeAppended};
        for(auto kind:kinds) {
            std::vector<T> items=reading?std::vector<T>():v.GetItems(kind);Item(items);
            if(reading) {
                if(explicitMode && kind!=SdfListOpTypeExplicit && !items.empty())Fail("nonexplicit items in explicit list op");
                if(!explicitMode && kind==SdfListOpTypeExplicit && !items.empty())Fail("explicit items in nonexplicit list op");
                if(!explicitMode && kind==SdfListOpTypeAdded)v.SetAddedItems(items);
                else if(!explicitMode && kind==SdfListOpTypeOrdered)v.SetOrderedItems(items);
                else if((explicitMode && kind==SdfListOpTypeExplicit)||(!explicitMode && kind!=SdfListOpTypeExplicit)) {std::string why;if(!v.SetItems(items,kind,&why))Fail("invalid input list op: "+why);}
            }
        }
    }
    void Item(GfVec2i& v) {for(size_t i=0;i<2;++i){auto x=v[i];Item(x);if(reading)v[i]=x;}}
    void Item(GfVec2h& v) {for(size_t i=0;i<2;++i){auto x=v[i];Item(x);if(reading)v[i]=x;}}
    void Item(GfVec2f& v) {for(size_t i=0;i<2;++i){auto x=v[i];Item(x);if(reading)v[i]=x;}}
    void Item(GfVec2d& v) {for(size_t i=0;i<2;++i){auto x=v[i];Item(x);if(reading)v[i]=x;}}
    void Item(GfVec3i& v) {for(size_t i=0;i<3;++i){auto x=v[i];Item(x);if(reading)v[i]=x;}}
    void Item(GfVec3h& v) {for(size_t i=0;i<3;++i){auto x=v[i];Item(x);if(reading)v[i]=x;}}
    void Item(GfVec3f& v) {for(size_t i=0;i<3;++i){auto x=v[i];Item(x);if(reading)v[i]=x;}}
    void Item(GfVec3d& v) {for(size_t i=0;i<3;++i){auto x=v[i];Item(x);if(reading)v[i]=x;}}
    void Item(GfVec4i& v) {for(size_t i=0;i<4;++i){auto x=v[i];Item(x);if(reading)v[i]=x;}}
    void Item(GfVec4h& v) {for(size_t i=0;i<4;++i){auto x=v[i];Item(x);if(reading)v[i]=x;}}
    void Item(GfVec4f& v) {for(size_t i=0;i<4;++i){auto x=v[i];Item(x);if(reading)v[i]=x;}}
    void Item(GfVec4d& v) {for(size_t i=0;i<4;++i){auto x=v[i];Item(x);if(reading)v[i]=x;}}
    void Item(GfMatrix2f& v) {for(size_t r=0;r<2;++r)for(size_t c=0;c<2;++c){auto x=v[r][c];Item(x);if(reading)v[r][c]=x;}}
    void Item(GfMatrix2d& v) {for(size_t r=0;r<2;++r)for(size_t c=0;c<2;++c){auto x=v[r][c];Item(x);if(reading)v[r][c]=x;}}
    void Item(GfMatrix3f& v) {for(size_t r=0;r<3;++r)for(size_t c=0;c<3;++c){auto x=v[r][c];Item(x);if(reading)v[r][c]=x;}}
    void Item(GfMatrix3d& v) {for(size_t r=0;r<3;++r)for(size_t c=0;c<3;++c){auto x=v[r][c];Item(x);if(reading)v[r][c]=x;}}
    void Item(GfMatrix4f& v) {for(size_t r=0;r<4;++r)for(size_t c=0;c<4;++c){auto x=v[r][c];Item(x);if(reading)v[r][c]=x;}}
    void Item(GfMatrix4d& v) {for(size_t r=0;r<4;++r)for(size_t c=0;c<4;++c){auto x=v[r][c];Item(x);if(reading)v[r][c]=x;}}
    void Item(GfQuath& v) {auto r=v.GetReal();auto i=v.GetImaginary();Item(r);Item(i);if(reading)v=GfQuath(r,i);}
    void Item(GfQuatf& v) {auto r=v.GetReal();auto i=v.GetImaginary();Item(r);Item(i);if(reading)v=GfQuatf(r,i);}
    void Item(GfQuatd& v) {auto r=v.GetReal();auto i=v.GetImaginary();Item(r);Item(i);if(reading)v=GfQuatd(r,i);}
    void Item(SdfSpecifier& v) {int32_t x=int32_t(v);Item(x);if(reading)v=static_cast<SdfSpecifier>(x);}
    void Item(SdfPermission& v) {int32_t x=int32_t(v);Item(x);if(reading)v=static_cast<SdfPermission>(x);}
    void Item(SdfVariability& v) {int32_t x=int32_t(v);Item(x);if(reading)v=static_cast<SdfVariability>(x);}
};
void Codec::Item(VtValue& value) {
    if(++depth>256)Fail("input value nesting limit");
    std::string tag=reading?std::string():"empty";
    if(!reading) {
        if(value.IsHolding<char>())tag="char";
        if(value.IsHolding<short>())tag="short";
        if(value.IsHolding<unsigned short>())tag="unsigned short";
        if(value.IsHolding<VtArray<char>>())tag="VtArray<char>";
        if(value.IsHolding<VtArray<short>>())tag="VtArray<short>";
        if(value.IsHolding<VtArray<unsigned short>>())tag="VtArray<unsigned short>";
        if(value.IsHolding<std::vector<TfToken>>())tag="std::vector<TfToken>";
        if(value.IsHolding<std::vector<double>>())tag="std::vector<double>";
        if(value.IsHolding<std::vector<SdfLayerOffset>>())tag="std::vector<SdfLayerOffset>";
        if(value.IsHolding<TsSpline>())tag="TsSpline";
        if(value.IsHolding<SdfVariantSelectionMap>())tag="SdfVariantSelectionMap";
        if(value.IsHolding<bool>())tag="bool";
        if(value.IsHolding<unsigned char>())tag="unsigned char";
        if(value.IsHolding<int>())tag="int";
        if(value.IsHolding<unsigned int>())tag="unsigned int";
        if(value.IsHolding<int64_t>())tag="int64_t";
        if(value.IsHolding<uint64_t>())tag="uint64_t";
        if(value.IsHolding<GfHalf>())tag="GfHalf";
        if(value.IsHolding<float>())tag="float";
        if(value.IsHolding<double>())tag="double";
        if(value.IsHolding<std::string>())tag="std::string";
        if(value.IsHolding<TfToken>())tag="TfToken";
        if(value.IsHolding<SdfPath>())tag="SdfPath";
        if(value.IsHolding<SdfAssetPath>())tag="SdfAssetPath";
        if(value.IsHolding<GfTimeCode>())tag="GfTimeCode";
        if(value.IsHolding<SdfLayerOffset>())tag="SdfLayerOffset";
        if(value.IsHolding<SdfValueBlock>())tag="SdfValueBlock";
        if(value.IsHolding<VtDictionary>())tag="VtDictionary";
        if(value.IsHolding<SdfTimeSampleMap>())tag="SdfTimeSampleMap";
        if(value.IsHolding<SdfReference>())tag="SdfReference";
        if(value.IsHolding<SdfPayload>())tag="SdfPayload";
        if(value.IsHolding<SdfSpecifier>())tag="SdfSpecifier";
        if(value.IsHolding<SdfPermission>())tag="SdfPermission";
        if(value.IsHolding<SdfVariability>())tag="SdfVariability";
        if(value.IsHolding<GfVec2i>())tag="GfVec2i";
        if(value.IsHolding<GfVec2h>())tag="GfVec2h";
        if(value.IsHolding<GfVec2f>())tag="GfVec2f";
        if(value.IsHolding<GfVec2d>())tag="GfVec2d";
        if(value.IsHolding<GfVec3i>())tag="GfVec3i";
        if(value.IsHolding<GfVec3h>())tag="GfVec3h";
        if(value.IsHolding<GfVec3f>())tag="GfVec3f";
        if(value.IsHolding<GfVec3d>())tag="GfVec3d";
        if(value.IsHolding<GfVec4i>())tag="GfVec4i";
        if(value.IsHolding<GfVec4h>())tag="GfVec4h";
        if(value.IsHolding<GfVec4f>())tag="GfVec4f";
        if(value.IsHolding<GfVec4d>())tag="GfVec4d";
        if(value.IsHolding<GfMatrix2f>())tag="GfMatrix2f";
        if(value.IsHolding<GfMatrix2d>())tag="GfMatrix2d";
        if(value.IsHolding<GfMatrix3f>())tag="GfMatrix3f";
        if(value.IsHolding<GfMatrix3d>())tag="GfMatrix3d";
        if(value.IsHolding<GfMatrix4f>())tag="GfMatrix4f";
        if(value.IsHolding<GfMatrix4d>())tag="GfMatrix4d";
        if(value.IsHolding<GfQuath>())tag="GfQuath";
        if(value.IsHolding<GfQuatf>())tag="GfQuatf";
        if(value.IsHolding<GfQuatd>())tag="GfQuatd";
        if(value.IsHolding<VtArray<bool>>())tag="VtArray<bool>";
        if(value.IsHolding<VtArray<unsigned char>>())tag="VtArray<unsigned char>";
        if(value.IsHolding<VtArray<int>>())tag="VtArray<int>";
        if(value.IsHolding<VtArray<unsigned int>>())tag="VtArray<unsigned int>";
        if(value.IsHolding<VtArray<int64_t>>())tag="VtArray<int64_t>";
        if(value.IsHolding<VtArray<uint64_t>>())tag="VtArray<uint64_t>";
        if(value.IsHolding<VtArray<GfHalf>>())tag="VtArray<GfHalf>";
        if(value.IsHolding<VtArray<float>>())tag="VtArray<float>";
        if(value.IsHolding<VtArray<double>>())tag="VtArray<double>";
        if(value.IsHolding<VtArray<std::string>>())tag="VtArray<std::string>";
        if(value.IsHolding<VtArray<TfToken>>())tag="VtArray<TfToken>";
        if(value.IsHolding<VtArray<SdfPath>>())tag="VtArray<SdfPath>";
        if(value.IsHolding<VtArray<SdfAssetPath>>())tag="VtArray<SdfAssetPath>";
        if(value.IsHolding<VtArray<GfTimeCode>>())tag="VtArray<GfTimeCode>";
        if(value.IsHolding<VtArray<GfVec2i>>())tag="VtArray<GfVec2i>";
        if(value.IsHolding<VtArray<GfVec2h>>())tag="VtArray<GfVec2h>";
        if(value.IsHolding<VtArray<GfVec2f>>())tag="VtArray<GfVec2f>";
        if(value.IsHolding<VtArray<GfVec2d>>())tag="VtArray<GfVec2d>";
        if(value.IsHolding<VtArray<GfVec3i>>())tag="VtArray<GfVec3i>";
        if(value.IsHolding<VtArray<GfVec3h>>())tag="VtArray<GfVec3h>";
        if(value.IsHolding<VtArray<GfVec3f>>())tag="VtArray<GfVec3f>";
        if(value.IsHolding<VtArray<GfVec3d>>())tag="VtArray<GfVec3d>";
        if(value.IsHolding<VtArray<GfVec4i>>())tag="VtArray<GfVec4i>";
        if(value.IsHolding<VtArray<GfVec4h>>())tag="VtArray<GfVec4h>";
        if(value.IsHolding<VtArray<GfVec4f>>())tag="VtArray<GfVec4f>";
        if(value.IsHolding<VtArray<GfVec4d>>())tag="VtArray<GfVec4d>";
        if(value.IsHolding<VtArray<GfMatrix2f>>())tag="VtArray<GfMatrix2f>";
        if(value.IsHolding<VtArray<GfMatrix2d>>())tag="VtArray<GfMatrix2d>";
        if(value.IsHolding<VtArray<GfMatrix3f>>())tag="VtArray<GfMatrix3f>";
        if(value.IsHolding<VtArray<GfMatrix3d>>())tag="VtArray<GfMatrix3d>";
        if(value.IsHolding<VtArray<GfMatrix4f>>())tag="VtArray<GfMatrix4f>";
        if(value.IsHolding<VtArray<GfMatrix4d>>())tag="VtArray<GfMatrix4d>";
        if(value.IsHolding<VtArray<GfQuath>>())tag="VtArray<GfQuath>";
        if(value.IsHolding<VtArray<GfQuatf>>())tag="VtArray<GfQuatf>";
        if(value.IsHolding<VtArray<GfQuatd>>())tag="VtArray<GfQuatd>";
        if(value.IsHolding<std::vector<VtValue>>())tag="std::vector<VtValue>";
        if(value.IsHolding<std::vector<std::string>>())tag="std::vector<std::string>";
        if(value.IsHolding<std::vector<SdfPath>>())tag="std::vector<SdfPath>";
        if(value.IsHolding<SdfListOp<int>>())tag="SdfListOp<int>";
        if(value.IsHolding<SdfListOp<unsigned int>>())tag="SdfListOp<unsigned int>";
        if(value.IsHolding<SdfListOp<int64_t>>())tag="SdfListOp<int64_t>";
        if(value.IsHolding<SdfListOp<uint64_t>>())tag="SdfListOp<uint64_t>";
        if(value.IsHolding<SdfListOp<std::string>>())tag="SdfListOp<std::string>";
        if(value.IsHolding<SdfListOp<TfToken>>())tag="SdfListOp<TfToken>";
        if(value.IsHolding<SdfListOp<SdfPath>>())tag="SdfListOp<SdfPath>";
        if(value.IsHolding<SdfListOp<SdfReference>>())tag="SdfListOp<SdfReference>";
        if(value.IsHolding<SdfListOp<SdfPayload>>())tag="SdfListOp<SdfPayload>";
        if(!value.IsEmpty() && tag=="empty")Fail("unsupported input value type: "+value.GetTypeName());
    }
    Text(tag);
    if(tag=="empty") {if(reading)value=VtValue();--depth;return;}
    if(tag=="char") {char x{};if(!reading)x=value.UncheckedGet<char>();Item(x);if(reading)value=VtValue(x);--depth;return;}
    if(tag=="short") {short x{};if(!reading)x=value.UncheckedGet<short>();Item(x);if(reading)value=VtValue(x);--depth;return;}
    if(tag=="unsigned short") {unsigned short x{};if(!reading)x=value.UncheckedGet<unsigned short>();Item(x);if(reading)value=VtValue(x);--depth;return;}
    if(tag=="VtArray<char>") {VtArray<char> x{};if(!reading)x=value.UncheckedGet<VtArray<char>>();Item(x);if(reading)value=VtValue(x);--depth;return;}
    if(tag=="VtArray<short>") {VtArray<short> x{};if(!reading)x=value.UncheckedGet<VtArray<short>>();Item(x);if(reading)value=VtValue(x);--depth;return;}
    if(tag=="VtArray<unsigned short>") {VtArray<unsigned short> x{};if(!reading)x=value.UncheckedGet<VtArray<unsigned short>>();Item(x);if(reading)value=VtValue(x);--depth;return;}
    if(tag=="std::vector<TfToken>") {std::vector<TfToken> x{};if(!reading)x=value.UncheckedGet<std::vector<TfToken>>();Item(x);if(reading)value=VtValue(x);--depth;return;}
    if(tag=="std::vector<double>") {std::vector<double> x{};if(!reading)x=value.UncheckedGet<std::vector<double>>();Item(x);if(reading)value=VtValue(x);--depth;return;}
    if(tag=="std::vector<SdfLayerOffset>") {std::vector<SdfLayerOffset> x{};if(!reading)x=value.UncheckedGet<std::vector<SdfLayerOffset>>();Item(x);if(reading)value=VtValue(x);--depth;return;}
    if(tag=="TsSpline"){TsSpline x;if(!reading)x=value.UncheckedGet<TsSpline>();Item(x);if(reading)value=VtValue(x);--depth;return;}
    if(tag=="SdfVariantSelectionMap"){SdfVariantSelectionMap x;if(!reading)x=value.UncheckedGet<SdfVariantSelectionMap>();Item(x);if(reading)value=VtValue(x);--depth;return;}
    if(tag=="bool"){bool x{};if(!reading)x=value.UncheckedGet<bool>();Item(x);if(reading)value=VtValue(x);--depth;return;}
    if(tag=="unsigned char"){unsigned char x{};if(!reading)x=value.UncheckedGet<unsigned char>();Item(x);if(reading)value=VtValue(x);--depth;return;}
    if(tag=="int"){int x{};if(!reading)x=value.UncheckedGet<int>();Item(x);if(reading)value=VtValue(x);--depth;return;}
    if(tag=="unsigned int"){unsigned int x{};if(!reading)x=value.UncheckedGet<unsigned int>();Item(x);if(reading)value=VtValue(x);--depth;return;}
    if(tag=="int64_t"){int64_t x{};if(!reading)x=value.UncheckedGet<int64_t>();Item(x);if(reading)value=VtValue(x);--depth;return;}
    if(tag=="uint64_t"){uint64_t x{};if(!reading)x=value.UncheckedGet<uint64_t>();Item(x);if(reading)value=VtValue(x);--depth;return;}
    if(tag=="GfHalf"){GfHalf x{};if(!reading)x=value.UncheckedGet<GfHalf>();Item(x);if(reading)value=VtValue(x);--depth;return;}
    if(tag=="float"){float x{};if(!reading)x=value.UncheckedGet<float>();Item(x);if(reading)value=VtValue(x);--depth;return;}
    if(tag=="double"){double x{};if(!reading)x=value.UncheckedGet<double>();Item(x);if(reading)value=VtValue(x);--depth;return;}
    if(tag=="std::string"){std::string x{};if(!reading)x=value.UncheckedGet<std::string>();Item(x);if(reading)value=VtValue(x);--depth;return;}
    if(tag=="TfToken"){TfToken x{};if(!reading)x=value.UncheckedGet<TfToken>();Item(x);if(reading)value=VtValue(x);--depth;return;}
    if(tag=="SdfPath"){SdfPath x{};if(!reading)x=value.UncheckedGet<SdfPath>();Item(x);if(reading)value=VtValue(x);--depth;return;}
    if(tag=="SdfAssetPath"){SdfAssetPath x{};if(!reading)x=value.UncheckedGet<SdfAssetPath>();Item(x);if(reading)value=VtValue(x);--depth;return;}
    if(tag=="GfTimeCode"){GfTimeCode x{};if(!reading)x=value.UncheckedGet<GfTimeCode>();Item(x);if(reading)value=VtValue(x);--depth;return;}
    if(tag=="SdfLayerOffset"){SdfLayerOffset x{};if(!reading)x=value.UncheckedGet<SdfLayerOffset>();Item(x);if(reading)value=VtValue(x);--depth;return;}
    if(tag=="SdfValueBlock"){SdfValueBlock x{};if(!reading)x=value.UncheckedGet<SdfValueBlock>();Item(x);if(reading)value=VtValue(x);--depth;return;}
    if(tag=="VtDictionary"){VtDictionary x{};if(!reading)x=value.UncheckedGet<VtDictionary>();Item(x);if(reading)value=VtValue(x);--depth;return;}
    if(tag=="SdfTimeSampleMap"){SdfTimeSampleMap x{};if(!reading)x=value.UncheckedGet<SdfTimeSampleMap>();Item(x);if(reading)value=VtValue(x);--depth;return;}
    if(tag=="SdfReference"){SdfReference x{};if(!reading)x=value.UncheckedGet<SdfReference>();Item(x);if(reading)value=VtValue(x);--depth;return;}
    if(tag=="SdfPayload"){SdfPayload x{};if(!reading)x=value.UncheckedGet<SdfPayload>();Item(x);if(reading)value=VtValue(x);--depth;return;}
    if(tag=="SdfSpecifier"){SdfSpecifier x{};if(!reading)x=value.UncheckedGet<SdfSpecifier>();Item(x);if(reading)value=VtValue(x);--depth;return;}
    if(tag=="SdfPermission"){SdfPermission x{};if(!reading)x=value.UncheckedGet<SdfPermission>();Item(x);if(reading)value=VtValue(x);--depth;return;}
    if(tag=="SdfVariability"){SdfVariability x{};if(!reading)x=value.UncheckedGet<SdfVariability>();Item(x);if(reading)value=VtValue(x);--depth;return;}
    if(tag=="GfVec2i"){GfVec2i x{};if(!reading)x=value.UncheckedGet<GfVec2i>();Item(x);if(reading)value=VtValue(x);--depth;return;}
    if(tag=="GfVec2h"){GfVec2h x{};if(!reading)x=value.UncheckedGet<GfVec2h>();Item(x);if(reading)value=VtValue(x);--depth;return;}
    if(tag=="GfVec2f"){GfVec2f x{};if(!reading)x=value.UncheckedGet<GfVec2f>();Item(x);if(reading)value=VtValue(x);--depth;return;}
    if(tag=="GfVec2d"){GfVec2d x{};if(!reading)x=value.UncheckedGet<GfVec2d>();Item(x);if(reading)value=VtValue(x);--depth;return;}
    if(tag=="GfVec3i"){GfVec3i x{};if(!reading)x=value.UncheckedGet<GfVec3i>();Item(x);if(reading)value=VtValue(x);--depth;return;}
    if(tag=="GfVec3h"){GfVec3h x{};if(!reading)x=value.UncheckedGet<GfVec3h>();Item(x);if(reading)value=VtValue(x);--depth;return;}
    if(tag=="GfVec3f"){GfVec3f x{};if(!reading)x=value.UncheckedGet<GfVec3f>();Item(x);if(reading)value=VtValue(x);--depth;return;}
    if(tag=="GfVec3d"){GfVec3d x{};if(!reading)x=value.UncheckedGet<GfVec3d>();Item(x);if(reading)value=VtValue(x);--depth;return;}
    if(tag=="GfVec4i"){GfVec4i x{};if(!reading)x=value.UncheckedGet<GfVec4i>();Item(x);if(reading)value=VtValue(x);--depth;return;}
    if(tag=="GfVec4h"){GfVec4h x{};if(!reading)x=value.UncheckedGet<GfVec4h>();Item(x);if(reading)value=VtValue(x);--depth;return;}
    if(tag=="GfVec4f"){GfVec4f x{};if(!reading)x=value.UncheckedGet<GfVec4f>();Item(x);if(reading)value=VtValue(x);--depth;return;}
    if(tag=="GfVec4d"){GfVec4d x{};if(!reading)x=value.UncheckedGet<GfVec4d>();Item(x);if(reading)value=VtValue(x);--depth;return;}
    if(tag=="GfMatrix2f"){GfMatrix2f x{};if(!reading)x=value.UncheckedGet<GfMatrix2f>();Item(x);if(reading)value=VtValue(x);--depth;return;}
    if(tag=="GfMatrix2d"){GfMatrix2d x{};if(!reading)x=value.UncheckedGet<GfMatrix2d>();Item(x);if(reading)value=VtValue(x);--depth;return;}
    if(tag=="GfMatrix3f"){GfMatrix3f x{};if(!reading)x=value.UncheckedGet<GfMatrix3f>();Item(x);if(reading)value=VtValue(x);--depth;return;}
    if(tag=="GfMatrix3d"){GfMatrix3d x{};if(!reading)x=value.UncheckedGet<GfMatrix3d>();Item(x);if(reading)value=VtValue(x);--depth;return;}
    if(tag=="GfMatrix4f"){GfMatrix4f x{};if(!reading)x=value.UncheckedGet<GfMatrix4f>();Item(x);if(reading)value=VtValue(x);--depth;return;}
    if(tag=="GfMatrix4d"){GfMatrix4d x{};if(!reading)x=value.UncheckedGet<GfMatrix4d>();Item(x);if(reading)value=VtValue(x);--depth;return;}
    if(tag=="GfQuath"){GfQuath x{};if(!reading)x=value.UncheckedGet<GfQuath>();Item(x);if(reading)value=VtValue(x);--depth;return;}
    if(tag=="GfQuatf"){GfQuatf x{};if(!reading)x=value.UncheckedGet<GfQuatf>();Item(x);if(reading)value=VtValue(x);--depth;return;}
    if(tag=="GfQuatd"){GfQuatd x{};if(!reading)x=value.UncheckedGet<GfQuatd>();Item(x);if(reading)value=VtValue(x);--depth;return;}
    if(tag=="VtArray<bool>"){VtArray<bool> x{};if(!reading)x=value.UncheckedGet<VtArray<bool>>();Item(x);if(reading)value=VtValue(x);--depth;return;}
    if(tag=="VtArray<unsigned char>"){VtArray<unsigned char> x{};if(!reading)x=value.UncheckedGet<VtArray<unsigned char>>();Item(x);if(reading)value=VtValue(x);--depth;return;}
    if(tag=="VtArray<int>"){VtArray<int> x{};if(!reading)x=value.UncheckedGet<VtArray<int>>();Item(x);if(reading)value=VtValue(x);--depth;return;}
    if(tag=="VtArray<unsigned int>"){VtArray<unsigned int> x{};if(!reading)x=value.UncheckedGet<VtArray<unsigned int>>();Item(x);if(reading)value=VtValue(x);--depth;return;}
    if(tag=="VtArray<int64_t>"){VtArray<int64_t> x{};if(!reading)x=value.UncheckedGet<VtArray<int64_t>>();Item(x);if(reading)value=VtValue(x);--depth;return;}
    if(tag=="VtArray<uint64_t>"){VtArray<uint64_t> x{};if(!reading)x=value.UncheckedGet<VtArray<uint64_t>>();Item(x);if(reading)value=VtValue(x);--depth;return;}
    if(tag=="VtArray<GfHalf>"){VtArray<GfHalf> x{};if(!reading)x=value.UncheckedGet<VtArray<GfHalf>>();Item(x);if(reading)value=VtValue(x);--depth;return;}
    if(tag=="VtArray<float>"){VtArray<float> x{};if(!reading)x=value.UncheckedGet<VtArray<float>>();Item(x);if(reading)value=VtValue(x);--depth;return;}
    if(tag=="VtArray<double>"){VtArray<double> x{};if(!reading)x=value.UncheckedGet<VtArray<double>>();Item(x);if(reading)value=VtValue(x);--depth;return;}
    if(tag=="VtArray<std::string>"){VtArray<std::string> x{};if(!reading)x=value.UncheckedGet<VtArray<std::string>>();Item(x);if(reading)value=VtValue(x);--depth;return;}
    if(tag=="VtArray<TfToken>"){VtArray<TfToken> x{};if(!reading)x=value.UncheckedGet<VtArray<TfToken>>();Item(x);if(reading)value=VtValue(x);--depth;return;}
    if(tag=="VtArray<SdfPath>"){VtArray<SdfPath> x{};if(!reading)x=value.UncheckedGet<VtArray<SdfPath>>();Item(x);if(reading)value=VtValue(x);--depth;return;}
    if(tag=="VtArray<SdfAssetPath>"){VtArray<SdfAssetPath> x{};if(!reading)x=value.UncheckedGet<VtArray<SdfAssetPath>>();Item(x);if(reading)value=VtValue(x);--depth;return;}
    if(tag=="VtArray<GfTimeCode>"){VtArray<GfTimeCode> x{};if(!reading)x=value.UncheckedGet<VtArray<GfTimeCode>>();Item(x);if(reading)value=VtValue(x);--depth;return;}
    if(tag=="VtArray<GfVec2i>"){VtArray<GfVec2i> x{};if(!reading)x=value.UncheckedGet<VtArray<GfVec2i>>();Item(x);if(reading)value=VtValue(x);--depth;return;}
    if(tag=="VtArray<GfVec2h>"){VtArray<GfVec2h> x{};if(!reading)x=value.UncheckedGet<VtArray<GfVec2h>>();Item(x);if(reading)value=VtValue(x);--depth;return;}
    if(tag=="VtArray<GfVec2f>"){VtArray<GfVec2f> x{};if(!reading)x=value.UncheckedGet<VtArray<GfVec2f>>();Item(x);if(reading)value=VtValue(x);--depth;return;}
    if(tag=="VtArray<GfVec2d>"){VtArray<GfVec2d> x{};if(!reading)x=value.UncheckedGet<VtArray<GfVec2d>>();Item(x);if(reading)value=VtValue(x);--depth;return;}
    if(tag=="VtArray<GfVec3i>"){VtArray<GfVec3i> x{};if(!reading)x=value.UncheckedGet<VtArray<GfVec3i>>();Item(x);if(reading)value=VtValue(x);--depth;return;}
    if(tag=="VtArray<GfVec3h>"){VtArray<GfVec3h> x{};if(!reading)x=value.UncheckedGet<VtArray<GfVec3h>>();Item(x);if(reading)value=VtValue(x);--depth;return;}
    if(tag=="VtArray<GfVec3f>"){VtArray<GfVec3f> x{};if(!reading)x=value.UncheckedGet<VtArray<GfVec3f>>();Item(x);if(reading)value=VtValue(x);--depth;return;}
    if(tag=="VtArray<GfVec3d>"){VtArray<GfVec3d> x{};if(!reading)x=value.UncheckedGet<VtArray<GfVec3d>>();Item(x);if(reading)value=VtValue(x);--depth;return;}
    if(tag=="VtArray<GfVec4i>"){VtArray<GfVec4i> x{};if(!reading)x=value.UncheckedGet<VtArray<GfVec4i>>();Item(x);if(reading)value=VtValue(x);--depth;return;}
    if(tag=="VtArray<GfVec4h>"){VtArray<GfVec4h> x{};if(!reading)x=value.UncheckedGet<VtArray<GfVec4h>>();Item(x);if(reading)value=VtValue(x);--depth;return;}
    if(tag=="VtArray<GfVec4f>"){VtArray<GfVec4f> x{};if(!reading)x=value.UncheckedGet<VtArray<GfVec4f>>();Item(x);if(reading)value=VtValue(x);--depth;return;}
    if(tag=="VtArray<GfVec4d>"){VtArray<GfVec4d> x{};if(!reading)x=value.UncheckedGet<VtArray<GfVec4d>>();Item(x);if(reading)value=VtValue(x);--depth;return;}
    if(tag=="VtArray<GfMatrix2f>"){VtArray<GfMatrix2f> x{};if(!reading)x=value.UncheckedGet<VtArray<GfMatrix2f>>();Item(x);if(reading)value=VtValue(x);--depth;return;}
    if(tag=="VtArray<GfMatrix2d>"){VtArray<GfMatrix2d> x{};if(!reading)x=value.UncheckedGet<VtArray<GfMatrix2d>>();Item(x);if(reading)value=VtValue(x);--depth;return;}
    if(tag=="VtArray<GfMatrix3f>"){VtArray<GfMatrix3f> x{};if(!reading)x=value.UncheckedGet<VtArray<GfMatrix3f>>();Item(x);if(reading)value=VtValue(x);--depth;return;}
    if(tag=="VtArray<GfMatrix3d>"){VtArray<GfMatrix3d> x{};if(!reading)x=value.UncheckedGet<VtArray<GfMatrix3d>>();Item(x);if(reading)value=VtValue(x);--depth;return;}
    if(tag=="VtArray<GfMatrix4f>"){VtArray<GfMatrix4f> x{};if(!reading)x=value.UncheckedGet<VtArray<GfMatrix4f>>();Item(x);if(reading)value=VtValue(x);--depth;return;}
    if(tag=="VtArray<GfMatrix4d>"){VtArray<GfMatrix4d> x{};if(!reading)x=value.UncheckedGet<VtArray<GfMatrix4d>>();Item(x);if(reading)value=VtValue(x);--depth;return;}
    if(tag=="VtArray<GfQuath>"){VtArray<GfQuath> x{};if(!reading)x=value.UncheckedGet<VtArray<GfQuath>>();Item(x);if(reading)value=VtValue(x);--depth;return;}
    if(tag=="VtArray<GfQuatf>"){VtArray<GfQuatf> x{};if(!reading)x=value.UncheckedGet<VtArray<GfQuatf>>();Item(x);if(reading)value=VtValue(x);--depth;return;}
    if(tag=="VtArray<GfQuatd>"){VtArray<GfQuatd> x{};if(!reading)x=value.UncheckedGet<VtArray<GfQuatd>>();Item(x);if(reading)value=VtValue(x);--depth;return;}
    if(tag=="std::vector<VtValue>"){std::vector<VtValue> x{};if(!reading)x=value.UncheckedGet<std::vector<VtValue>>();Item(x);if(reading)value=VtValue(x);--depth;return;}
    if(tag=="std::vector<std::string>"){std::vector<std::string> x{};if(!reading)x=value.UncheckedGet<std::vector<std::string>>();Item(x);if(reading)value=VtValue(x);--depth;return;}
    if(tag=="std::vector<SdfPath>"){std::vector<SdfPath> x{};if(!reading)x=value.UncheckedGet<std::vector<SdfPath>>();Item(x);if(reading)value=VtValue(x);--depth;return;}
    if(tag=="SdfListOp<int>"){SdfListOp<int> x{};if(!reading)x=value.UncheckedGet<SdfListOp<int>>();Item(x);if(reading)value=VtValue(x);--depth;return;}
    if(tag=="SdfListOp<unsigned int>"){SdfListOp<unsigned int> x{};if(!reading)x=value.UncheckedGet<SdfListOp<unsigned int>>();Item(x);if(reading)value=VtValue(x);--depth;return;}
    if(tag=="SdfListOp<int64_t>"){SdfListOp<int64_t> x{};if(!reading)x=value.UncheckedGet<SdfListOp<int64_t>>();Item(x);if(reading)value=VtValue(x);--depth;return;}
    if(tag=="SdfListOp<uint64_t>"){SdfListOp<uint64_t> x{};if(!reading)x=value.UncheckedGet<SdfListOp<uint64_t>>();Item(x);if(reading)value=VtValue(x);--depth;return;}
    if(tag=="SdfListOp<std::string>"){SdfListOp<std::string> x{};if(!reading)x=value.UncheckedGet<SdfListOp<std::string>>();Item(x);if(reading)value=VtValue(x);--depth;return;}
    if(tag=="SdfListOp<TfToken>"){SdfListOp<TfToken> x{};if(!reading)x=value.UncheckedGet<SdfListOp<TfToken>>();Item(x);if(reading)value=VtValue(x);--depth;return;}
    if(tag=="SdfListOp<SdfPath>"){SdfListOp<SdfPath> x{};if(!reading)x=value.UncheckedGet<SdfListOp<SdfPath>>();Item(x);if(reading)value=VtValue(x);--depth;return;}
    if(tag=="SdfListOp<SdfReference>"){SdfListOp<SdfReference> x{};if(!reading)x=value.UncheckedGet<SdfListOp<SdfReference>>();Item(x);if(reading)value=VtValue(x);--depth;return;}
    if(tag=="SdfListOp<SdfPayload>"){SdfListOp<SdfPayload> x{};if(!reading)x=value.UncheckedGet<SdfListOp<SdfPayload>>();Item(x);if(reading)value=VtValue(x);--depth;return;}
    Fail("unknown input value tag: "+tag);
}
} // namespace
bool RigExecEncodeInputValue(const VtValue& value,std::string* out,std::string* error) {
    if(!out){if(error)*error="null input encoding output";return false;}
    try {Codec c; c.bytes="RIV1";VtValue copy=value;c.Item(copy);*out=std::move(c.bytes);if(error)error->clear();return true;}
    catch(const std::exception& e){if(error)*error=e.what();return false;}
}
bool RigExecDecodeInputValue(const std::string& bytes,VtValue* out,std::string* error) {
    if(!out){if(error)*error="null input decoding output";return false;}
    try {if(bytes.size()<4||bytes.compare(0,4,"RIV1")!=0)throw std::runtime_error("unsupported input value codec version");Codec c;c.bytes=bytes;c.reading=true;c.pos=4;VtValue v;c.Item(v);if(c.pos!=bytes.size())throw std::runtime_error("trailing input value bytes");std::string roundtrip,why;if(!RigExecEncodeInputValue(v,&roundtrip,&why)||roundtrip!=bytes)throw std::runtime_error("input reconstruction is not bit-exact: "+why);*out=std::move(v);if(error)error->clear();return true;}
    catch(const std::exception& e){if(error)*error=e.what();return false;}
}
} // namespace rigExec
