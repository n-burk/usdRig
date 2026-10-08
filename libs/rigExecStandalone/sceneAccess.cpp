#include "sceneAccess.h"

namespace rigExec {
SdfPathVector RigExecSceneDbAccess::Prims(const SdfPath &root) const {
    SdfPathVector result;
    for (const auto &[path, prim] : _db.prims)
        if (path.HasPrefix(root)) result.push_back(path);
    return result;
}
bool RigExecSceneDbAccess::Prim(const SdfPath &path, RigExecScenePrim *result) const {
    const auto found = _db.prims.find(path);
    if (!result || found == _db.prims.end()) return false;
    const auto &prim = found->second;
    *result = {path, prim.type, prim.appliedSchemas, prim.metadata, _db.IsActive(path)};
    result->children = prim.children;
    return true;
}
SdfPathVector RigExecSceneDbAccess::Attributes(const SdfPath &prim) const {
    SdfPathVector result;
    for (const auto &[path, attribute] : _db.attributes)
        if (path.GetPrimPath() == prim) result.push_back(path);
    return result;
}
SdfPathVector RigExecSceneDbAccess::Relationships(const SdfPath &prim) const {
    SdfPathVector result;
    for (const auto &[path, relationship] : _db.relationships)
        if (path.GetPrimPath() == prim) result.push_back(path);
    return result;
}
bool RigExecSceneDbAccess::Attribute(const SdfPath &path, RigExecSceneAttribute *result) const {
    const auto found = _db.attributes.find(path);
    if (!result || found == _db.attributes.end()) return false;
    const auto &attribute = found->second;
    *result = {path, attribute.type, attribute.metadata, attribute.connections};
    result->hasValue = attribute.hasValue;
    result->hasAuthoredValue = attribute.hasAuthoredValue;
    result->hasAuthoredReadableValue = attribute.hasAuthoredReadableValue;
    result->hasAuthoredConnections = attribute.hasAuthoredConnections;
    result->mightBeTimeVarying = attribute.mightBeTimeVarying;
    result->hasAuthoredDefault = attribute.hasAuthoredDefault;
    result->defaultBlocked = attribute.defaultBlocked;
    result->sampleTimes = attribute.sampleTimes;
    result->variability = attribute.variability;
    result->spline = attribute.spline;
    return true;
}
bool RigExecSceneDbAccess::Relationship(const SdfPath &path, RigExecSceneRelationship *result) const {
    const auto found = _db.relationships.find(path);
    if (!result || found == _db.relationships.end()) return false;
    const auto &relationship = found->second;
    *result = {path, relationship.metadata, relationship.targets};
    return true;
}
bool RigExecSceneDbAccess::Resolve(const SdfPath &path, UsdTimeCode time, VtValue *result) const {
    if (!result) return false;
    *result = VtValue();
    const auto found = _db.attributes.find(path);
    if (found == _db.attributes.end() || !_db.IsActive(path)) return false;
    const auto state = found->second.resolved.find(RigExecStandaloneTimeKey(time));
    if (state == found->second.resolved.end()) return false;
    *result = state->second;
    return true;
}
bool RigExecSceneDbAccess::HasIdentity(UsdTimeCode time) const {
    return _db.identities.count(RigExecStandaloneTimeKey(time)) != 0;
}
bool RigExecSceneDbAccess::ValueBlocked(const SdfPath &path, UsdTimeCode time) const {
    const auto found = _db.attributes.find(path);
    return found != _db.attributes.end() && found->second.blockedIdentities.count(RigExecStandaloneTimeKey(time)) != 0;
}
} // namespace rigExec
