#include "system.h"
#include "adapter.h"
#include "rigExec/types.h"
#include "pxr/exec/exec/system.h"
#include "pxr/exec/exec/systemChangeProcessor.h"
#include "pxr/exec/exec/requestImpl.h"
#include "pxr/exec/exec/cacheView.h"
#include "pxr/exec/exec/valueKey.h"
#include "pxr/exec/exec/builtinComputations.h"
#include "pxr/exec/ef/time.h"
#include "pxr/base/tf/errorMark.h"
#include "pxr/base/tf/span.h"
#include <atomic>
#include <algorithm>

namespace rigExec {
namespace {
bool Fail(std::string *error, const std::string &message) {
    if (error) *error = message;
    return false;
}
bool SupportedPhaseValue(const SdfPath &path, const VtValue &value) {
    return path.GetNameToken() != "rigExec:pointsReadPhase" || value.IsEmpty() ||
        value == VtValue(TfToken("base"));
}
class Core : public ExecSystem {
public:
    explicit Core(const RigExecSceneDb *db) : ExecSystem(RigExecAdaptStandaloneStage(db)) {}
    ~Core() = default;
    void Time(UsdTimeCode time) { _ChangeTime(EfTime(time)); }
    void Values(const SdfPathVector &paths) {
        _ChangeProcessor change(this);
        for (const SdfPath &path : paths) change.DidChangeInfoOnly(path, {TfToken("timeSamples"), TfToken("default")});
    }
    void Resync(const SdfPath &path, const SdfPathVector &incoming = {}) {
        _ChangeProcessor change(this);
        change.DidResync(path);
        for (const SdfPath &source : incoming) change.DidChangeIncomingConnections(source);
    }
};
class Request : public Exec_RequestImpl {
public:
    Request(Core *system, std::atomic<size_t> *invalidations)
        : Exec_RequestImpl(system,
            [invalidations](const ExecRequestIndexSet &indices, const EfTimeInterval &) {
                invalidations->fetch_add(indices.size());
            }, [](const ExecRequestIndexSet &) {}) {}
    ~Request() = default;
    void Prepare(const std::vector<ExecValueKey> &keys) {
        if (_RequiresCompilation()) _Compile(keys);
        _Schedule();
    }
    Exec_CacheView Compute() { return _Compute(); }
};
}

struct RigExecStandaloneSystem::_Impl {
    // Destruction order is request, core, then database. Every Esf holder and
    // retained query therefore has a live backing database until its disposal.
    RigExecSceneDb database;
    Core core;
    std::vector<RigExecValueAddress> addresses;
    std::vector<ExecValueKey> keys;
    std::atomic<size_t> invalidations{0};
    std::unique_ptr<Request> request;
    uint64_t generation = 0;
    bool validated = false;
    explicit _Impl(const RigExecSceneDb &db) : database(db), core(&database) {}

    bool MissingDirectSource(const RigExecValueAddress &address, UsdTimeCode time) const {
        if (!address.target.IsPropertyPath()) return false;
        VtValue value;
        if (address.publicComputation == ExecBuiltinComputations->computeResolvedValue)
            return !database.Get(address.target, time, &value);
        if (!address.publicComputation.IsEmpty() &&
            address.publicComputation != ExecBuiltinComputations->computeValue) return false;
        SdfPath path = address.target;
        std::set<SdfPath> visited;
        while (visited.insert(path).second) {
            const auto attr = database.attributes.find(path);
            if (attr == database.attributes.end()) return true;
            const auto &prim = database.prims.at(path.GetPrimPath());
            // Exact attribute expressions in the supported registry, registered
            // by RIGEXEC_REGISTER_XFORMABLE in computations.cpp. Other RigExec
            // attributes (including avars and custom values) remain raw sources.
            // Unknown applied APIs are rejected by ValidateCapabilities.
            static const std::set<TfToken> spaceExpressions{
                TfToken("default:space"), TfToken("avars:defaultSpace"),
                TfToken("posed:defaultSpace"), TfToken("parent:space"),
                TfToken("parent:defaultSpace")};
            if ((prim.type == "RigExecControl" || prim.type == "RigExecJoint") &&
                spaceExpressions.count(path.GetNameToken())) return false;
            // Stock computeValue uses a connection only when exactly one valid
            // attribute has the same native type. Otherwise it uses this
            // provider's raw resolved value, including an inactive-target case.
            if (attr->second.connections.size() != 1)
                return !database.Get(path, time, &value);
            const auto &target = attr->second.connections.front();
            const auto source = database.attributes.find(target);
            if (source == database.attributes.end() || !database.IsActive(target) ||
                source->second.type.GetType() != attr->second.type.GetType())
                return !database.Get(path, time, &value);
            path = target;
        }
        return false; // Exec diagnoses a connection cycle.
    }

    bool Prepare(std::string *error) {
        if (!validated) {
            if (!database.Validate(error)) return false;
            if (!database.ValidateCapabilities(error)) return false;
            validated = true;
        }
        for (const auto &address : addresses) {
            if (!database.HasObject(address.target)) return Fail(error, "tap provider missing or inactive: " + address.target.GetString());
            if (address.phase != "base" && address.phase != "final") return Fail(error, "standalone provider runtime has no after-mover phase");
            if (address.target.IsPropertyPath() && !database.attributes.count(address.target))
                return Fail(error, "tap property is not an attribute: " + address.target.GetString());
        }
        if (addresses.empty()) return true;
        if (!request) {
            keys.clear();
            for (const auto &address : addresses) {
                keys.emplace_back(RigExecAdaptStandaloneObject(&database, address.target),
                    address.publicComputation.IsEmpty() ? ExecBuiltinComputations->computeValue : address.publicComputation);
            }
            request = std::make_unique<Request>(&core, &invalidations);
        }
        TfErrorMark errors;
        request->Prepare(keys);
        if (!errors.IsClean()) {
            std::string message = "standalone request could not compile";
            for (const auto &entry : errors) message += "; " + entry.GetCommentary();
            errors.Clear();
            return Fail(error, message);
        }
        return true;
    }
    RigExecStandaloneResult Evaluate(UsdTimeCode time) {
        RigExecStandaloneResult result;
        result.time = time;
        const std::string identity = RigExecStandaloneTimeKey(time);
        if (identity.empty() || !database.identities.count(identity)) {
            result.diagnostics.push_back("unexported standalone evaluation identity");
            return result;
        }
        std::string error;
        if (!Prepare(&error)) { result.diagnostics.push_back(error); return result; }
        if (addresses.empty()) {
            result.valid = true;
            result.generation = ++generation;
            return result;
        }
        for (const auto &address : addresses) {
            if (MissingDirectSource(address, time)) {
                result.diagnostics.push_back("standalone source tap has an explicit no-value state: " +
                    address.target.GetString());
                return result;
            }
        }
        TfErrorMark errors;
        core.Time(time);
        request->Prepare(keys);
        {
            const auto view = request->Compute();
            result.values.reserve(keys.size());
            for (size_t i = 0; i < keys.size(); ++i) result.values.push_back(view.Get(int(i)));
        }
        for (const auto &entry : errors) result.diagnostics.push_back(entry.GetCommentary());
        result.valid = errors.IsClean();
        for (size_t i = 0; i < result.values.size(); ++i) {
            if (result.values[i].IsEmpty()) {
                result.valid = false;
                result.diagnostics.push_back("standalone tap produced no value: " +
                    addresses[i].target.GetString() + " " + addresses[i].publicComputation.GetString());
            }
        }
        errors.Clear();
        if (result.valid) result.generation = ++generation;
        return result;
    }
};

RigExecStandaloneSystem::RigExecStandaloneSystem(const RigExecSceneDb &db) {
    RigExecLoadComputations();
    _impl = std::make_unique<_Impl>(db);
}
RigExecStandaloneSystem::~RigExecStandaloneSystem() = default;
int RigExecStandaloneSystem::AddTap(const RigExecValueAddress &address) {
    _impl->request.reset();
    _impl->addresses.push_back(address);
    return int(_impl->addresses.size() - 1);
}
bool RigExecStandaloneSystem::Prepare(std::string *error) { return _impl->Prepare(error); }
RigExecStandaloneResult RigExecStandaloneSystem::Evaluate(UsdTimeCode time) { return _impl->Evaluate(time); }
RigExecStandaloneResult RigExecStandaloneSystem::EvaluateResolved(
    UsdTimeCode time, const std::map<SdfPath, VtValue> &states)
{
    const std::string identity = RigExecStandaloneTimeKey(time);
    RigExecStandaloneResult failure;
    failure.time = time;
    if (identity.empty() || states.size() != _impl->database.attributes.size()) {
        failure.diagnostics.push_back("transient evaluation requires a complete resolved-state set and finite time");
        return failure;
    }
    for (const auto &[path, attr] : _impl->database.attributes) {
        const auto state = states.find(path);
        if (state == states.end() || !state->second.IsEmpty() && state->second.GetType() != attr.type.GetType()) {
            failure.diagnostics.push_back("invalid transient resolved state for " + path.GetString());
            return failure;
        }
        if (!SupportedPhaseValue(path, state->second)) {
            failure.diagnostics.push_back("transient blend target read phase requires evaluator lowering: " + path.GetString());
            return failure;
        }
    }
    const bool retainedIdentity = _impl->database.identities.count(identity) != 0;
    std::map<SdfPath, VtValue> previous;
    SdfPathVector changed;
    for (auto &[path, attr] : _impl->database.attributes) {
        if (retainedIdentity) previous[path] = attr.resolved.at(identity);
        attr.resolved[identity] = states.at(path);
        changed.push_back(path);
    }
    _impl->database.identities.insert(identity);
    _impl->core.Values(changed);
    const RigExecStandaloneResult result = _impl->Evaluate(time);
    for (auto &[path, attr] : _impl->database.attributes) {
        if (retainedIdentity) attr.resolved[identity] = previous.at(path);
        else attr.resolved.erase(identity);
    }
    if (!retainedIdentity) _impl->database.identities.erase(identity);
    _impl->core.Values(changed);
    return result;
}
bool RigExecStandaloneSystem::SetValue(const SdfPath &path, UsdTimeCode time,
                                      const VtValue &value, std::string *error) {
    auto attr = _impl->database.attributes.find(path);
    const std::string identity = RigExecStandaloneTimeKey(time);
    if (attr == _impl->database.attributes.end() || !_impl->database.identities.count(identity))
        return Fail(error, "attribute or exported identity not found");
    if (!value.IsEmpty() && value.GetType() != attr->second.type.GetType())
        return Fail(error, "value does not match exact native attribute type");
    if (!SupportedPhaseValue(path, value))
        return Fail(error, "blend target read phase requires evaluator lowering");
    if (attr->second.resolved[identity] == value) return true;
    attr->second.resolved[identity] = value;
    _impl->core.Values({path});
    return true;
}
bool RigExecStandaloneSystem::SetConnections(const SdfPath &path, const SdfPathVector &sources, std::string *error) {
    auto attr = _impl->database.attributes.find(path);
    if (attr == _impl->database.attributes.end()) return Fail(error, "attribute not found");
    if (_impl->database.prims.at(path.GetPrimPath()).type.GetString().rfind("RigExec", 0) == 0 &&
        !attr->second.type.IsArray() && sources.size() > 1)
        return Fail(error, "RigExec scalar attribute requires at most one connection");
    for (const auto &source : sources) {
        const auto input = _impl->database.attributes.find(source);
        if (input == _impl->database.attributes.end() || input->second.type != attr->second.type)
            return Fail(error, "connection needs a retained attribute of the same native Sdf type");
    }
    if (attr->second.connections == sources) return true;
    SdfPathVector changed = attr->second.connections;
    changed.insert(changed.end(), sources.begin(), sources.end());
    attr->second.connections = sources;
    _impl->validated = false;
    _impl->database.Validate(); // publish the new incoming index before resync
    _impl->core.Resync(path, changed);
    return true;
}
bool RigExecStandaloneSystem::SetTargets(const SdfPath &path, const SdfPathVector &targets, std::string *error) {
    auto rel = _impl->database.relationships.find(path);
    if (rel == _impl->database.relationships.end()) return Fail(error, "relationship not found");
    if (path.GetNameToken() == "rigExec:joints" && !targets.empty()) return Fail(error, "reverse solver-joint bindings require evaluator lowering");
    if (path.GetNameToken() == "rigExec:moves" && !targets.empty()) return Fail(error, "mover revisions require evaluator lowering");
    for (const auto &target : targets) if (!_impl->database.HasObject(target)) return Fail(error, "relationship target not found");
    if (rel->second.targets == targets) return true;
    rel->second.targets = targets;
    _impl->core.Resync(path);
    return true;
}
bool RigExecStandaloneSystem::SetPrimActive(const SdfPath &path, bool active, std::string *error) {
    auto prim = _impl->database.prims.find(path);
    if (prim == _impl->database.prims.end() || path == SdfPath::AbsoluteRootPath()) return Fail(error, "editable prim not found");
    if (prim->second.active == active) return true;
    _impl->request.reset();
    prim->second.active = active;
    _impl->core.Resync(path);
    return true;
}
const void *RigExecStandaloneSystem::GetCompilerIdentity() const { return &_impl->core; }
size_t RigExecStandaloneSystem::GetValueInvalidationCount() const { return _impl->invalidations.load(); }
} // namespace rigExec
