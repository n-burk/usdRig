#include "sceneVersionRoutes.h"
#include "rigExec/moverGraph.h"
#include <algorithm>
namespace rigExec {
namespace {
bool Fail(std::string *error,const std::string &message) { if(error)*error=message;return false; }
}
bool RigExecSceneVersionRoutes::RegisterBase(const RigExecSceneVersionKey &key,
    RigExecValueId value,std::string *error) {
    if(value==UINT64_MAX)return Fail(error,"invalid scene base value");
    RigExecSceneValueChain chain;chain.base=value;chain.poseBase=value;
    if(!chains.emplace(key,std::move(chain)).second)
        return Fail(error,"duplicate scene value base: "+key.path.GetString());
    return true;
}
bool RigExecSceneVersionRoutes::Append(const RigExecSceneVersionKey &key,
    const RigExecSceneValueVersion &version,std::string *error) {
    const auto found=chains.find(key);
    if(found==chains.end())return Fail(error,"scene value version has no base: "+key.path.GetString());
    if(version.value==UINT64_MAX || version.writer.IsEmpty() || version.ordinal<0)
        return Fail(error,"invalid scene value version: "+key.path.GetString());
    auto &versions=found->second.versions;
    for(const auto &existing:versions)if(existing.value==version.value)
        return Fail(error,"duplicate scene value version ID: "+key.path.GetString());
    const auto position=std::upper_bound(versions.begin(),versions.end(),version.ordinal,[](int ordinal,const auto &existing){return ordinal<existing.ordinal;});
    versions.insert(position,version);return true;
}
bool RigExecSceneVersionRoutes::Resolve(const RigExecSceneDescriptors &scene,
    const RigExecSceneVersionKey &key,const SdfPath &reader,const TfToken &phase,
    RigExecValueId *value,std::string *error) const {
    if(!value)return Fail(error,"null scene value route destination");
    const auto found=chains.find(key);
    if(found==chains.end())return Fail(error,"scene value source has no typed route: "+key.path.GetString());
    const auto &chain=found->second;
    RigExecReadPhase parsed;
    if(!RigExecParseReadPhase(phase.GetString(),&parsed,error))return false;
    if(parsed.kind==RigExecReadPhaseKind::Base) {
        *value=key.domain==RigExecSceneValueDomain::Pose?chain.poseBase:chain.base;
        return *value!=UINT64_MAX || Fail(error,"missing solver base frame: "+key.path.GetString());
    }
    if(parsed.kind==RigExecReadPhaseKind::Final) {
        *value=chain.versions.empty()?(key.domain==RigExecSceneValueDomain::Pose?chain.poseBase:chain.base):chain.versions.back().value;return true;
    }
    const SdfPath marker=parsed.kind==RigExecReadPhaseKind::AtPrim?parsed.prim:reader.GetPrimPath();
    const auto node=scene.nodes.find(marker);
    if(node==scene.nodes.end() || node->second.stackOrdinal<0)
        return Fail(error,"scene read phase has no composed ordinal: "+marker.GetString());
    const int ordinal=node->second.stackOrdinal;
    *value=chain.base;
    for(const auto &version:chain.versions) {
        const bool included=parsed.kind==RigExecReadPhaseKind::AtPrim?
            version.ordinal<=ordinal:version.ordinal<ordinal;
        if(!included)break;
        *value=version.value;
    }
    return true;
}
}
