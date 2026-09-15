//
// RigExec tap set: USD backend implementation over ExecUsdSystem.
//
#include "tapSet.h"
#include "debugCodes.h"

#include "pxr/exec/exec/request.h"
#include "pxr/exec/execUsd/cacheView.h"
#include "pxr/exec/execUsd/request.h"
#include "pxr/exec/execUsd/system.h"
#include "pxr/exec/execUsd/valueKey.h"
#include "pxr/exec/execUsd/valueOverride.h"
#include "pxr/exec/ef/timeInterval.h"
#include "pxr/usd/usd/attribute.h"
#include "pxr/usd/usd/prim.h"
#include "pxr/base/tf/notice.h"
#include "pxr/base/tf/registryManager.h"
#include "pxr/base/tf/stopwatch.h"
#include "pxr/base/tf/weakBase.h"

#include <map>
#include <mutex>
#include <set>

namespace rigExec {

TF_REGISTRY_FUNCTION(TfDebug)
{
    TF_DEBUG_ENVIRONMENT_SYMBOL(
        RIGEXEC_TAP_TIMING,
        "RigExecTapSet: time spent in ExecUsdSystem BuildRequest, "
        "PrepareRequest and Compute, and the tap count per request.");
}

// A scoped stopwatch that reports only when the debug code is on. The
// TF_DEBUG check is hoisted to the constructor so a disabled build pays
// one array lookup per scope, not a stopwatch.
namespace {
class _DebugTimer {
public:
    explicit _DebugTimer(const char *label)
        : _label(label), _on(TfDebug::IsEnabled(RIGEXEC_TAP_TIMING)) {
        if (_on) _watch.Start();
    }
    ~_DebugTimer() {
        if (!_on) return;
        _watch.Stop();
        // GetSeconds, not GetMilliseconds: the latter returns an int64 count
        // of WHOLE milliseconds, so every sub-millisecond scope reported
        // 0.000 ms.
        TF_DEBUG(RIGEXEC_TAP_TIMING).Msg(
            "[rigExec] %-22s %9.3f ms\n",
            _label, _watch.GetSeconds() * 1000.0);
    }
private:
    const char *_label;
    bool _on;
    TfStopwatch _watch;
};
}  // namespace

// Requests share the stock compiler and value cache for a stage. Replacing a
// tap list therefore changes requests, not the underlying execution network.
// The weak table does not extend stage or evaluator lifetimes.
class RigExecTapContext : public TfWeakBase {
public:
    explicit RigExecTapContext(const UsdStageRefPtr &stage) : _stage(stage) {}
    ~RigExecTapContext() { TfNotice::Revoke(_notice); }

    static std::shared_ptr<RigExecTapContext> Find(
        const UsdStageRefPtr &stage, bool create) {
        static std::mutex mutex;
        static std::map<UsdStageWeakPtr, std::weak_ptr<RigExecTapContext>> contexts;
        std::lock_guard<std::mutex> lock(mutex);
        for (auto it = contexts.begin(); it != contexts.end();) {
            if (it->second.expired()) it = contexts.erase(it);
            else ++it;
        }
        const auto found = contexts.find(UsdStageWeakPtr(stage));
        if (found != contexts.end()) {
            if (auto context = found->second.lock()) return context;
        }
        if (!create) return {};
        auto context = std::make_shared<RigExecTapContext>(stage);
        contexts[UsdStageWeakPtr(stage)] = context;
        return context;
    }

    ExecUsdSystem *GetSystem() {
        if (!_system) {
            _system = std::make_unique<ExecUsdSystem>(UsdStageConstRefPtr(_stage));
            // TfNotice dispatches newest registrations first. This listener
            // must precede Esf's listener so deletion can revoke it safely.
            TfNotice::Revoke(_notice);
            _notice = TfNotice::Register(TfCreateWeakPtr(this),
                &RigExecTapContext::_OnObjectsChanged, UsdStageWeakPtr(_stage));
        }
        return _system.get();
    }

    void PrepareStageChange(const UsdNotice::ObjectsChanged &notice) {
        if (!_system) return;
        bool removed = false;
        for (const SdfPath &path : notice.GetResyncedPaths()) {
            if (path.IsPrimPath() && !_stage->GetPrimAtPath(path)) {
                removed = true;
                break;
            }
        }
        if (!removed) return;
        // Requests must die before their system. Their immutable extracted
        // snapshots are unaffected. Revocation cancels pending Tf callbacks.
        for (RigExecTapSet *taps : clients) {
            taps->_request.reset();
            taps->_prepared = false;
            taps->_dirty = true;
        }
        _system.reset();
    }

    std::set<RigExecTapSet *> clients;
private:
    void _OnObjectsChanged(const UsdNotice::ObjectsChanged &notice,
                           const UsdStageWeakPtr &) {
        PrepareStageChange(notice);
    }
    UsdStageRefPtr _stage;
    std::unique_ptr<ExecUsdSystem> _system;
    TfNotice::Key _notice;
};

RigExecTapSet::RigExecTapSet(const UsdStageRefPtr &stage)
    : _stage(stage)
    , _context(RigExecTapContext::Find(stage, true))
{
    _context->clients.insert(this);
}

RigExecTapSet::~RigExecTapSet()
{
    _request.reset();
    _context->clients.erase(this);
}

ExecUsdSystem *RigExecTapSet::GetSystem() { return _context->GetSystem(); }

void RigExecTapSet::PrepareStageChange(
    const UsdStageRefPtr &stage, const UsdNotice::ObjectsChanged &notice)
{
    if (auto context = RigExecTapContext::Find(stage, false)) {
        context->PrepareStageChange(notice);
    }
}

RigExecTapId
RigExecTapSet::Add(const RigExecValueAddress &address)
{
    _addresses.push_back(address);
    _resolutions.emplace_back();
    _prepared = false;
    return static_cast<RigExecTapId>(_addresses.size() - 1);
}

RigExecTapId
RigExecTapSet::AddResolved(
    const RigExecValueAddress &publicAddress, const SdfPath &privateProvider)
{
    _addresses.push_back(publicAddress);
    _resolutions.push_back(privateProvider);
    _prepared = false;
    return static_cast<RigExecTapId>(_addresses.size() - 1);
}

bool
RigExecTapSet::Prepare()
{
    // A tap set with nothing in it is a legitimate epoch, not a failure.
    //
    // It arrives whenever a rig's whole output set is derived without exec:
    // a constraint aiming a plain UsdGeomXformable at another plain
    // UsdGeomXformable reads both frames from their USD transforms, and a
    // rig with no joints and no exec-backed provider has no value keys at
    // all. Building an empty ExecUsdRequest and asking whether it is valid
    // conflates "nothing to compute" with "could not be compiled", and the
    // caller treats the second as a compile failure.
    if (_addresses.empty()) {
        _request.reset();
        _prepared = true;
        return true;
    }

    TF_DEBUG(RIGEXEC_TAP_TIMING).Msg(
        "[rigExec] Prepare() call #%zu, taps=%zu\n",
        ++_prepareCount, _addresses.size());
    _DebugTimer prepareTimer("Prepare total");

    std::vector<ExecUsdValueKey> keys;
    keys.reserve(_addresses.size());
    for (size_t i = 0; i < _addresses.size(); ++i) {
        const RigExecValueAddress &address = _addresses[i];
        // The compiler-private resolution supplies the actual provider;
        // the public address stays canonical (spec §9.1).
        const SdfPath &provider =
            _resolutions[i].IsEmpty() ? address.target : _resolutions[i];
        if (provider.IsPropertyPath()) {
            const UsdAttribute attr = _stage->GetAttributeAtPath(provider);
            keys.emplace_back(attr);
        } else {
            const UsdPrim prim = _stage->GetPrimAtPath(provider);
            keys.emplace_back(prim, address.publicComputation);
        }
    }

    ExecUsdSystem *const system = GetSystem();
    {
    _DebugTimer buildTimer("BuildRequest");
    _request = std::make_unique<ExecUsdRequest>(system->BuildRequest(
        std::move(keys),
        [this](const ExecRequestIndexSet &, const EfTimeInterval &) {
            // Invalidation callbacks only record dirtiness; evaluation
            // happens on the caller's next pull (spec §6.3, §9.3).
            _dirty = true;
        },
        [this](const ExecRequestIndexSet &) {
            // A time change invalidates values, not the request: the compiled
            // network and its schedule are time-independent. Clearing
            // _prepared here would rebuild and re-schedule every request on
            // every animated frame, which is the whole cost of scrubbing a
            // rig that has time samples.
            _dirty = true;
        }));
    }
    if (!_request->IsValid()) {
        return false;
    }
    {
        _DebugTimer prepareRequestTimer("PrepareRequest");
        system->PrepareRequest(*_request);
    }
    _prepared = true;
    return true;
}

void
RigExecTapSet::Warm(UsdTimeCode time)
{
    if (!_prepared || (_request && !_request->IsValid())) {
        Prepare();
    }
    if (_addresses.empty() || !_request || !_request->IsValid()) {
        return;
    }
    ExecUsdSystem *const system = GetSystem();
    system->ChangeTime(time);
    // The cache view is deliberately dropped: the point is the computation it
    // leaves behind in the shared executor, not the values.
    system->Compute(*_request);
}

RigExecSnapshot
RigExecTapSet::Evaluate(UsdTimeCode time)
{
    return Evaluate(time, {});
}

RigExecSnapshot
RigExecTapSet::Evaluate(
    UsdTimeCode time, const std::vector<RigExecValueOverride> &overrides)
{
    // Rebuild on expiry as well as on an explicit tap-list change. A request
    // can be invalidated under us by a structural edit -- a prim deactivated
    // and reactivated, a variant switched away and back, a payload unloaded
    // and reloaded, a delete undone -- and nothing else would ever rebuild it,
    // leaving the tap set returning nothing for the rest of the session.
    if (!_prepared || (_request && !_request->IsValid())) {
        TF_DEBUG(RIGEXEC_TAP_TIMING).Msg(
            "[rigExec] Evaluate: request was NOT prepared -- re-preparing\n");
        Prepare();
    }

    RigExecSnapshot snapshot;
    snapshot._time = time;
    // The empty set evaluates trivially: valid, complete, and holding no
    // values. Every caller checks IsComplete() before reading, and "no taps
    // were requested" satisfies "every requested tap produced a value".
    if (_addresses.empty()) {
        snapshot._valid = true;
        snapshot._complete = true;
        return snapshot;
    }
    if (!_request || !_request->IsValid()) {
        return snapshot;
    }

    ExecUsdSystem *const system = GetSystem();
    {
        _DebugTimer changeTimeTimer("ChangeTime");
        system->ChangeTime(time);
    }

    ExecUsdValueOverrideVector execOverrides;
    execOverrides.reserve(overrides.size());
    for (const RigExecValueOverride &o : overrides) {
        const UsdPrim prim = _stage->GetPrimAtPath(o.prim);
        if (!prim) {
            continue;
        }
        if (!o.attribute.IsEmpty()) {
            const UsdAttribute attr = prim.GetAttribute(o.attribute);
            if (!attr) {
                continue;
            }
            execOverrides.push_back(
                ExecUsdValueOverride{ExecUsdValueKey(attr), o.value});
            continue;
        }
        execOverrides.push_back(
            ExecUsdValueOverride{ExecUsdValueKey(prim, o.computation),
                                 o.value});
    }

    _DebugTimer computeTimer(execOverrides.empty()
                             ? "Compute" : "ComputeWithOverrides");
    ExecUsdCacheView view = execOverrides.empty()
        ? system->Compute(*_request)
        : system->ComputeWithOverrides(*_request, std::move(execOverrides));

    // Snapshot copies values immediately: the cache view must not outlive
    // its system or request (spec §6.1).
    snapshot._values.reserve(_addresses.size());
    bool complete = true;
    for (size_t i = 0; i < _addresses.size(); ++i) {
        snapshot._values.push_back(view.Get(static_cast<int>(i)));
        if (snapshot._values.back().IsEmpty()) {
            complete = false;
        }
    }
    snapshot._valid = true;
    snapshot._complete = complete;
    return snapshot;
}

}  // namespace rigExec
