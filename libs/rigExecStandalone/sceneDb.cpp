#include "sceneDb.h"
#include "pxr/base/tf/type.h"
#include "pxr/usd/usd/schemaRegistry.h"
#include <cmath>
#include <cstring>
#include <iomanip>
#include <sstream>
#include <algorithm>
#include <locale>
#include <mutex>

namespace rigExec {
namespace {
const void *InternSchemaKey(const std::pair<TfToken, TfTokenVector> &key)
{
    static std::mutex mutex;
    static std::map<std::pair<TfToken, TfTokenVector>, int> keys;
    std::lock_guard<std::mutex> lock(mutex);
    return &keys.try_emplace(key, 0).first->second;
}
}
bool RigExecStandaloneSupportsPrimType(const TfToken &type)
{
    if (type.GetString().rfind("RigExec", 0) != 0) return true;
    static const std::set<TfToken> supported{
        TfToken("RigExecRoot"), TfToken("RigExecControl"), TfToken("RigExecJoint"),
        TfToken("RigExecFkChain"), TfToken("RigExecTwoBoneIk"),
        TfToken("RigExecBlendPointFrames"), TfToken("RigExecTwistDistribution"),
        TfToken("RigExecStaticWeight"), TfToken("RigExecDynamicWeight"),
        TfToken("RigExecCombineWeight"), TfToken("RigExecCurvenetWeight"),
        TfToken("RigExecBlendInput"), TfToken("RigExecBlendSample"),
        TfToken("RigExecCurvenet")};
    return supported.count(type) != 0;
}
std::string RigExecStandaloneTimeKey(UsdTimeCode time)
{
    if (time.IsDefault()) return "default";
    double value = time.GetValue();
    if (!std::isfinite(value)) return {};
    if (value == 0) value = 0; // canonicalize negative zero
    uint64_t bits = 0;
    std::memcpy(&bits, &value, sizeof(bits));
    std::ostringstream result;
    result.imbue(std::locale::classic());
    result << (time.IsPreTime() ? "pre:" : "exact:") << std::hex
           << std::setw(16) << std::setfill('0') << bits;
    return result.str();
}
bool RigExecSceneDb::IsActive(const SdfPath &path) const
{
    for (SdfPath prim = path.GetPrimPath(); !prim.IsEmpty(); prim = prim.GetParentPath()) {
        const auto it = prims.find(prim);
        if (it == prims.end() || !it->second.active) return false;
    }
    return true;
}
bool RigExecSceneDb::HasObject(const SdfPath &path) const
{
    return IsActive(path) && (prims.count(path) || attributes.count(path) || relationships.count(path));
}
bool RigExecSceneDb::Get(const SdfPath &path, UsdTimeCode time, VtValue *value) const
{
    if (!value) return false;
    *value = VtValue();
    const auto attribute = attributes.find(path);
    if (attribute == attributes.end() || !IsActive(path)) return false;
    const auto state = attribute->second.resolved.find(RigExecStandaloneTimeKey(time));
    if (state == attribute->second.resolved.end() || state->second.IsEmpty()) return false;
    *value = state->second;
    return true;
}
const void *RigExecSceneDb::SchemaKey(const SdfPath &path) const
{
    const auto prim = prims.find(path.GetPrimPath());
    if (prim == prims.end()) return nullptr;
    const auto key = _schemaKeys.find(std::make_pair(prim->second.type,
        prim->second.appliedSchemas));
    return key == _schemaKeys.end() ? nullptr : key->second;
}
SdfPathVector RigExecSceneDb::IncomingConnections(const SdfPath &path) const
{
    const auto it = _incomingConnections.find(path);
    return it == _incomingConnections.end() ? SdfPathVector() : it->second;
}
bool RigExecSceneDb::ValidateCapabilities(std::string *error) const
{
    const auto fail = [&](const std::string &message) {
        if (error) *error = "standalone provider runtime " + message;
        return false;
    };
    for (const auto &[path, prim] : prims) {
        if (!RigExecStandaloneSupportsPrimType(prim.type))
            return fail("does not lower schema " + prim.type.GetString() + ": " + path.GetString());
        if (std::find(prim.appliedSchemas.begin(), prim.appliedSchemas.end(),
                      TfToken("RigExecMoverAPI")) != prim.appliedSchemas.end())
            return fail("does not lower mover revisions: " + path.GetString());
        // These stock APIs only describe data in the qualified OpenUSD build.
        // Unknown APIs may register expressions that this provider slice cannot
        // classify, so do not silently treat their raw attributes as computed.
        static const std::set<TfToken> dataApis{
            TfToken("CollectionAPI"), TfToken("GeomModelAPI"), TfToken("MotionAPI"),
            TfToken("VisibilityAPI"), TfToken("MaterialBindingAPI"), TfToken("SkelBindingAPI")};
        for (const auto &applied : prim.appliedSchemas) {
            const auto name = UsdSchemaRegistry::GetTypeNameAndInstance(applied).first;
            if (!dataApis.count(name))
                return fail("does not support applied schema " + applied.GetString() + ": " + path.GetString());
        }
    }
    const auto basePhase = [](const VtValue &value) {
        return value.IsEmpty() || value == VtValue(TfToken("base")) ||
            value == VtValue(std::string("base"));
    };
    const auto baseMetadata = [&](const VtDictionary &metadata) {
        const auto phase = metadata.find("rigExecReadPhase");
        return phase == metadata.end() || basePhase(phase->second);
    };
    for (const auto &[path, attr] : attributes) {
        if (!baseMetadata(attr.metadata))
            return fail("does not lower input read phases: " + path.GetString());
        if (path.GetNameToken() == "rigExec:pointsReadPhase") {
            for (const auto &[identity, value] : attr.resolved)
                if (!basePhase(value))
                    return fail("does not lower blend target read phases: " + path.GetString());
        }
    }
    for (const auto &[path, rel] : relationships) {
        if (path.GetNameToken() == "rigExec:joints" && !rel.targets.empty())
            return fail("requires explicit solver outputs; reverse joint binding is not lowered: " + path.GetString());
        if (path.GetNameToken() == "rigExec:moves" && !rel.targets.empty())
            return fail("does not lower mover revisions: " + path.GetString());
        if (!baseMetadata(rel.metadata))
            return fail("does not lower input read phases: " + path.GetString());
    }
    return true;
}
bool RigExecSceneDb::Validate(std::string *error) const
{
    const auto fail = [&](const std::string &message) {
        if (error) *error = message;
        return false;
    };
    if (!prims.count(SdfPath::AbsoluteRootPath()) || identities.empty())
        return fail("scene database needs a pseudo-root and exported identities");
    if (interpolation != "linear" && interpolation != "held")
        return fail("unknown stage interpolation policy");
    for (const std::string &identity : identities) {
        if (identity == "default") continue;
        const size_t prefix = identity.rfind("exact:", 0) == 0 ? 6 :
            identity.rfind("pre:", 0) == 0 ? 4 : 0;
        if (!prefix || identity.size() != prefix + 16 ||
            identity.find_first_not_of("0123456789abcdef", prefix) != std::string::npos)
            return fail("malformed exported time identity " + identity);
        uint64_t bits = 0;
        std::istringstream input(identity.substr(prefix));
        input.imbue(std::locale::classic());
        input >> std::hex >> bits;
        double value = 0;
        std::memcpy(&value, &bits, sizeof(value));
        if (!std::isfinite(value) || (value == 0 && bits != 0))
            return fail("noncanonical exported time identity " + identity);
    }
    if (!(timeCodesPerSecond > 0) || !std::isfinite(timeCodesPerSecond) ||
        !(framesPerSecond > 0) || !std::isfinite(framesPerSecond))
        return fail("invalid scene time units");
    for (const auto &[path, prim] : prims) {
        if (!path.IsAbsolutePath() || (!path.IsPrimPath() && path != SdfPath::AbsoluteRootPath()))
            return fail("invalid prim path " + path.GetString());
        if (path != SdfPath::AbsoluteRootPath() && !prims.count(path.GetParentPath()))
            return fail("missing prim parent " + path.GetString());
        const auto key = std::make_pair(prim.type, prim.appliedSchemas);
        _schemaKeys.try_emplace(key, InternSchemaKey(key));
    }
    _incomingConnections.clear();
    for (const auto &[path, attribute] : attributes) {
        if (!path.IsPropertyPath() || !prims.count(path.GetPrimPath()) || !attribute.type)
            return fail("invalid attribute descriptor " + path.GetString());
        if (relationships.count(path)) return fail("attribute/relationship path collision " + path.GetString());
        if (prims.at(path.GetPrimPath()).type.GetString().rfind("RigExec", 0) == 0 &&
            !attribute.type.IsArray() && attribute.connections.size() > 1)
            return fail("RigExec scalar attribute requires at most one connection: " + path.GetString());
        for (const auto &[identity, value] : attribute.resolved) {
            if (!identities.count(identity)) return fail("unexpected resolved-state identity " + identity);
        }
        for (const std::string &identity : identities) {
            const auto value = attribute.resolved.find(identity);
            if (value == attribute.resolved.end()) return fail("missing resolved state " + path.GetString() + " " + identity);
            if (!value->second.IsEmpty() && value->second.GetType() != attribute.type.GetType())
                return fail("wrong native value type at " + path.GetString());
        }
        for (const SdfPath &source : attribute.connections) {
            if (!source.IsPropertyPath() || !attributes.count(source))
                return fail("connection source is not retained: " + source.GetString());
            if (attributes.at(source).type != attribute.type)
                return fail("connection does not match exact native Sdf type: " + path.GetString());
            _incomingConnections[source].push_back(path);
        }
    }
    for (const auto &[path, relationship] : relationships) {
        if (!path.IsPropertyPath() || !prims.count(path.GetPrimPath()))
            return fail("invalid relationship descriptor " + path.GetString());
        for (const SdfPath &target : relationship.targets) {
            if (!prims.count(target) && !attributes.count(target) && !relationships.count(target))
                return fail("relationship target is not retained: " + target.GetString());
        }
    }
    return true;
}
} // namespace rigExec
