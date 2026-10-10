#include "sceneCompileInputs.h"
#include <algorithm>
namespace rigExec {
namespace {
bool Fail(std::string *error,const std::string &text) {
    if (error) *error=text;
    return false;
}
}
const RigExecSceneNodeDescriptor *RigExecSceneCompileInputs::Node(const SdfPath &path) const {
    const auto found=_scene.nodes.find(path);
    return found==_scene.nodes.end()?nullptr:&found->second;
}
const RigExecSceneAttributeDescriptor *RigExecSceneCompileInputs::Attribute(const SdfPath &path) const {
    const auto found=_scene.attributes.find(path);
    return found==_scene.attributes.end()?nullptr:&found->second;
}
const RigExecSceneRelationshipDescriptor *RigExecSceneCompileInputs::Relationship(const SdfPath &path) const {
    const auto found=_scene.relationships.find(path);
    return found==_scene.relationships.end()?nullptr:&found->second;
}
bool RigExecSceneCompileInputs::Bind(const SdfPath &path,RigExecSceneReadRoute route,
    RigExecSceneBoundInput *output,std::string *error) const {
    if (!output) return Fail(error,"null compiled input destination");
    const auto *attribute=Attribute(path);
    if (!attribute) return Fail(error,"missing compiled input: "+path.GetString());
    if (attribute->inputs.size()!=_scene.identities.size())
        return Fail(error,"incomplete compiled input identities: "+path.GetString());
    RigExecSceneBoundInput result;
    result.consumer=path; result.type=attribute->fact.type;
    result.readPhase=attribute->readPhase; result.route=route;
    result.varies=attribute->fact.mightBeTimeVarying || !attribute->fact.sampleTimes.empty();
    for (const auto &input:attribute->inputs) {
        const bool resolved=route==RigExecSceneReadRoute::ConnectionResolved;
        const bool computed=resolved && input.state==RigExecSceneInputState::Computed;
        const bool failed=resolved && (input.state==RigExecSceneInputState::Unavailable ||
            input.state==RigExecSceneInputState::Cycle);
        if (resolved) {
            if (result.walk.empty()) { result.walk=input.hops; result.source=input.source; }
            else if (result.walk!=input.hops || result.source!=input.source)
                return Fail(error,"identity-dependent input connection topology: "+path.GetString());
            for (const auto &hop:input.hops) {
                const auto *source=Attribute(hop);
                if (source) result.varies=result.varies || source->fact.mightBeTimeVarying ||
                    !source->fact.sampleTimes.empty();
            }
        }
        result.values.push_back(resolved?input.resolved:input.raw);
        result.blocked.push_back(resolved?input.resolvedBlocked:input.rawBlocked);
        result.computed.push_back(computed);
        // Empty raw facts are legitimate no-value states. A failed connection
        // resolution remains unavailable even if a downstream raw value exists.
        result.available.push_back(!failed && !computed);
    }
    if (route==RigExecSceneReadRoute::Raw) { result.source=path; result.walk.push_back(path); }
    *output=std::move(result);
    return true;
}
bool RigExecSceneCompileInputs::Read(const RigExecSceneBoundInput &input,
    UsdTimeCode identity,VtValue *value,bool *blocked,std::string *error) const {
    if (!value) return Fail(error,"null compiled input read destination");
    const auto found=std::find(_scene.identities.begin(),_scene.identities.end(),identity);
    if (found==_scene.identities.end())
        return Fail(error,"uncaptured compiled input identity: "+input.consumer.GetString());
    const size_t index=size_t(found-_scene.identities.begin());
    if (index>=input.values.size() || index>=input.available.size() ||
        index>=input.blocked.size() || index>=input.computed.size())
        return Fail(error,"invalid compiled input binding: "+input.consumer.GetString());
    if (input.computed[index])
        return Fail(error,"computed input requires graph producer: "+input.consumer.GetString());
    if (!input.available[index])
        return Fail(error,"unavailable compiled input: "+input.consumer.GetString());
    *value=input.values[index];
    if (blocked) *blocked=input.blocked[index]!=0;
    return true;
}
SdfPathVector RigExecSceneCompileInputs::Targets(const SdfPath &path) const {
    const auto *relation=Relationship(path);
    return relation?relation->forwardedTargets:SdfPathVector{};
}
bool RigExecSceneCompileInputs::TargetExists(const SdfPath &path,size_t target) const {
    const auto *relation=Relationship(path);
    return relation && target<relation->targetExists.size() && relation->targetExists[target];
}
}
