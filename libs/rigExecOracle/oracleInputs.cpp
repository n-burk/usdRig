#include "rigExec/oracleInputs.h"
#include "rigExec/movers/moverRegistry.h"
#include "rigExec/moverGraph.h"
#include <set>
#include "pxr/usd/usd/primRange.h"
#include "pxr/usd/usdSkel/blendShape.h"
namespace rigExec {
namespace {
template<class T> void Capture(const RigExecResolvedInputs &resolved,const UsdAttribute &a,
    UsdTimeCode time,RigExecOracleAttribute *out) {
    T value;
    if(resolved.GetAttribute(a,time,&value))out->resolvedTime[std::type_index(typeid(T))]=VtValue(value);
    if(resolved.GetAttribute(a,UsdTimeCode::Default(),&value))out->resolvedDefault[std::type_index(typeid(T))]=VtValue(value);
}
}
RigExecOracleScene RigExecCaptureOracleInputs(const UsdStageRefPtr &stage,
    const RigExecResolvedInputs &resolved,UsdTimeCode time,size_t,const std::vector<SdfPath> &roots,const std::map<SdfPath,VtValue> &upstream,RigExecOracleCaptureMode mode) {
    RigExecOracleScene scene;
    scene.resolveFromFacts=mode==RigExecOracleCaptureMode::PublicationFacts;
    std::set<SdfPath> visited;
    std::vector<SdfPath> pending = roots;
    while (!pending.empty()) {
        const auto path = pending.back().GetPrimPath();pending.pop_back();
        if (!visited.insert(path).second) continue;
        const auto prim = stage->GetPrimAtPath(path);
        if (!prim) continue;
        RigExecOraclePrim row;row.exists=true;row.path=prim.GetPath();row.type=prim.GetTypeName();
        for(const auto &a:prim.GetAttributes()) {
            RigExecOracleAttribute v;v.exists=true;v.path=a.GetPath();v.type=a.GetTypeName();
            a.Get(&v.atTime,time);a.Get(&v.atDefault,UsdTimeCode::Default());
            const auto authored=upstream.find(a.GetPath());
            if (authored!=upstream.end() && authored->second.IsHolding<VtVec3fArray>())
                v.atTime=v.atDefault=authored->second;
            v.connected=a.HasAuthoredConnections();a.GetConnections(&v.connections);
            pending.insert(pending.end(),v.connections.begin(),v.connections.end());
            if(const auto *overlay=resolved.Find(a.GetPath()))scene.overlay[a.GetPath()]=*overlay;
            if(mode==RigExecOracleCaptureMode::ResolvedValues) {
                if(v.type==SdfValueTypeNames->Float||v.type==SdfValueTypeNames->Double) { Capture<float>(resolved,a,time,&v);Capture<double>(resolved,a,time,&v); }
                else if(v.type==SdfValueTypeNames->Int)Capture<int>(resolved,a,time,&v);
                else if(v.type==SdfValueTypeNames->Bool)Capture<bool>(resolved,a,time,&v);
                else if(v.type==SdfValueTypeNames->Token)Capture<TfToken>(resolved,a,time,&v);
                else if(v.type.GetType()==TfType::Find<GfVec3f>())Capture<GfVec3f>(resolved,a,time,&v);
                else if(v.type.GetType()==TfType::Find<VtVec3fArray>())Capture<VtVec3fArray>(resolved,a,time,&v);
                else if(v.type==SdfValueTypeNames->FloatArray)Capture<VtFloatArray>(resolved,a,time,&v);
                else if(v.type==SdfValueTypeNames->IntArray)Capture<VtIntArray>(resolved,a,time,&v);
                else if(v.type==SdfValueTypeNames->Matrix4d)Capture<GfMatrix4d>(resolved,a,time,&v);
                else if(v.type.GetType()==TfType::Find<GfVec3d>())Capture<GfVec3d>(resolved,a,time,&v);
                else if(v.type.GetType()==TfType::Find<GfVec3i>())Capture<GfVec3i>(resolved,a,time,&v);
            }
            row.attributes[a.GetName()]=std::move(v);
        }
        for(const auto &r:prim.GetRelationships()) {
            RigExecOracleRelationship v;v.exists=true;r.GetTargets(&v.targets);RigExecResolveReadPhase(r,&v.phase,nullptr);
            pending.insert(pending.end(),v.targets.begin(),v.targets.end());
            for (const auto &target:v.targets) {
                if (!target.IsPrimPath()) continue;
                const auto root=stage->GetPrimAtPath(target);
                if (!root) continue;
                for (const auto &child:UsdPrimRange(root)) pending.push_back(child.GetPath());
            }
            row.relationships[r.GetName()]=std::move(v);
        }
        scene.prims[prim.GetPath()]=std::move(row);
        if(prim.IsA<UsdSkelBlendShape>()) scene.blendShapes.insert(prim.GetPath());
    }
    for (const auto &prim:stage->TraverseAll())
        if (scene.prims.count(prim.GetPath())) scene.primOrder.push_back(prim.GetPath());
    return scene;
}
} // namespace rigExec
