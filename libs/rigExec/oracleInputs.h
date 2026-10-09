// Captured source facts for independent scalar reference handlers.
#ifndef RIGEXEC_ORACLE_INPUTS_H
#define RIGEXEC_ORACLE_INPUTS_H
#include "moverGraphTypes.h"
#include "pxr/usd/sdf/types.h"
#include "pxr/usd/sdf/valueTypeName.h"
#include "pxr/usd/usd/common.h"
#include "pxr/usd/usd/timeCode.h"
#include <map>
#include <memory>
#include <utility>
#include <set>
#include <type_traits>
#include <typeindex>
#include <typeinfo>
#include <vector>
namespace rigExec {
struct RigExecBlendSampleLayout;
struct RigExecOracleAttribute {
    bool exists = false;
    SdfPath path;
    SdfValueTypeName type;
    bool connected = false;
    SdfPathVector connections;
    VtValue atTime, atDefault;
    std::map<std::type_index,VtValue> resolvedTime, resolvedDefault;
    explicit operator bool() const { return exists; }
    const SdfPath &GetPath() const { return path; }
    const SdfValueTypeName &GetTypeName() const { return type; }
    bool HasAuthoredConnections() const { return connected; }
    void GetConnections(SdfPathVector *out) const { *out=connections; }
    template<class T> bool Get(T *out, UsdTimeCode time = UsdTimeCode::Default()) const {
        const auto &v = time.IsDefault() ? atDefault : atTime;
        if (!v.IsHolding<T>()) return false;
        *out = v.UncheckedGet<T>(); return true;
    }
};
struct RigExecOracleRelationship {
    bool exists = false;
    SdfPathVector targets;
    RigExecReadPhase phase;
    explicit operator bool() const { return exists; }
    void GetTargets(SdfPathVector *out) const { *out = targets; }
};
struct RigExecOraclePrim {
    bool exists = false;
    SdfPath path;
    TfToken type;
    std::map<TfToken,RigExecOracleAttribute> attributes;
    std::map<TfToken,RigExecOracleRelationship> relationships;
    explicit operator bool() const { return exists; }
    const SdfPath &GetPath() const { return path; }
    const TfToken &GetTypeName() const { return type; }
    RigExecOracleAttribute GetAttribute(const TfToken &name) const {
        const auto it=attributes.find(name); return it==attributes.end()?RigExecOracleAttribute():it->second;
    }
    RigExecOracleAttribute GetAttribute(const char *name) const {
        for(const auto &entry:attributes)if(entry.first.GetString()==name)return entry.second;
        return {};
    }
    RigExecOracleAttribute GetAttribute(const std::string &name) const { return GetAttribute(name.c_str()); }
    RigExecOracleRelationship GetRelationship(const TfToken &name) const {
        const auto it=relationships.find(name);return it==relationships.end()?RigExecOracleRelationship():it->second;
    }
};
struct RigExecOracleScene {
    // Capture builders own these fields. Publication readers use the const
    // accessors after Begin seals the facts; only their overlay may change.
    std::map<SdfPath,RigExecOraclePrim> prims;
    std::vector<SdfPath> primOrder;
    std::map<SdfPath,VtValue> overlay;
    std::set<SdfPath> blendShapes;
    bool resolveFromFacts = false;
    const RigExecOracleScene *operator->() const { return this; }
    const std::map<SdfPath,RigExecOraclePrim> &Prims() const {
        return _capturedFacts ? _capturedFacts->prims : prims;
    }
    const std::vector<SdfPath> &PrimOrder() const {
        return _capturedFacts ? _capturedFacts->primOrder : primOrder;
    }
    const std::set<SdfPath> &BlendShapes() const {
        return _capturedFacts ? _capturedFacts->blendShapes : blendShapes;
    }
    RigExecOraclePrim GetPrimAtPath(const SdfPath &path) const {
        const auto &facts=Prims();
        const auto it=facts.find(path);return it==facts.end()?RigExecOraclePrim():it->second;
    }
    RigExecOracleAttribute GetAttributeAtPath(const SdfPath &path) const {
        const auto &facts=Prims();
        const auto it=facts.find(path.GetPrimPath());
        return it==facts.end()?RigExecOracleAttribute():it->second.GetAttribute(path.GetNameToken());
    }
    template<class T> bool GetAttribute(const RigExecOracleAttribute &a,UsdTimeCode time,T *out) const {
        if (resolveFromFacts) return ResolveAttribute(a,time,out);
        const auto &values=time.IsDefault()?a.resolvedDefault:a.resolvedTime;
        const auto it=values.find(std::type_index(typeid(T)));
        if(it==values.end()||!it->second.IsHolding<T>())return false;
        *out=it->second.UncheckedGet<T>();return true;
    }
    template<class T> bool ResolveAttribute(const RigExecOracleAttribute &attribute,UsdTimeCode time,T *out) const {
        if (!out) return false;
        if constexpr (std::is_same_v<T,float>) {
            if (attribute && attribute.type==SdfValueTypeNames->Double) {
                double wide=0.0;if(!ResolveAttribute(attribute,time,&wide))return false;
                *out=static_cast<float>(wide);return true;
            }
        }
        if (attribute && !Find(attribute.path) && !attribute.connected) return attribute.Get(out,time);
        std::set<SdfPath> visiting;
        std::vector<RigExecOracleAttribute> fallback;
        auto a=attribute;
        while(a && visiting.insert(a.path).second) {
            if(Get(a.path,out))return true;
            if constexpr(std::is_same_v<T,float>) {
                if(a.type==SdfValueTypeNames->Double) {
                    double wide=0.0;if(!ResolveAttribute(a,time,&wide))return false;
                    *out=static_cast<float>(wide);return true;
                }
            }
            fallback.push_back(a);
            if(!a.connected || a.connections.size()!=1)break;
            a=GetAttributeAtPath(a.connections[0]);
        }
        for(auto it=fallback.rbegin();it!=fallback.rend();++it)if(it->Get(out,time))return true;
        return false;
    }
    const VtValue *Find(const SdfPath &path) const { const auto it=overlay.find(path);return it==overlay.end()?nullptr:&it->second; }
    template<class T> bool Get(const SdfPath &path,T *out) const {
        const auto *v=Find(path);if(!v||!v->IsHolding<T>())return false;*out=v->UncheckedGet<T>();return true;
    }
private:
    friend class RigExecOraclePublicationContext;
    struct CapturedFacts {
        std::map<SdfPath,RigExecOraclePrim> prims;
        std::vector<SdfPath> primOrder;
        std::set<SdfPath> blendShapes;
    };
    void SealCapturedFacts() {
        if (_capturedFacts) return;
        auto facts=std::make_shared<CapturedFacts>();
        facts->prims=std::move(prims);
        facts->primOrder=std::move(primOrder);
        facts->blendShapes=std::move(blendShapes);
        _capturedFacts=std::move(facts);
    }
    std::shared_ptr<const CapturedFacts> _capturedFacts;
};
// Publication readers independently resolve raw facts; generic capture keeps
// the typed resolved-value maps for its existing reader API.
enum class RigExecOracleCaptureMode { ResolvedValues, PublicationFacts };
RigExecOracleScene RigExecCaptureOracleInputs(const UsdStageRefPtr &, const RigExecResolvedInputs &,
    UsdTimeCode,size_t pointCount,const std::vector<SdfPath> &roots,
    const std::map<SdfPath,VtValue> &upstream,
    RigExecOracleCaptureMode mode = RigExecOracleCaptureMode::ResolvedValues);
SdfPathVector RigExecRelationshipTargets(const RigExecOraclePrim &, const char *);
RigExecReadPhase RigExecPhaseForInput(const RigExecOraclePrim &, const char *);
bool RigExecResolveBlendSampleLayout(const RigExecOracleScene &,const SdfPath &,size_t,RigExecBlendSampleLayout *);
namespace evaluatorDetail {
template<class T,class Name> T _ResolvedRead(const RigExecOracleScene &scene,
    const RigExecOraclePrim &prim,const Name &name,T fallback,UsdTimeCode time) {
    scene.GetAttribute(prim.GetAttribute(name),time,&fallback);return fallback;
}
}
} // namespace rigExec
#endif
