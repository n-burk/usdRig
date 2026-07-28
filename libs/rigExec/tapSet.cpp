//
// RigExec tap set: USD backend implementation over ExecUsdSystem.
//
#include "tapSet.h"

#include "pxr/exec/exec/request.h"
#include "pxr/exec/execUsd/cacheView.h"
#include "pxr/exec/execUsd/request.h"
#include "pxr/exec/execUsd/system.h"
#include "pxr/exec/execUsd/valueKey.h"
#include "pxr/exec/execUsd/valueOverride.h"
#include "pxr/exec/ef/timeInterval.h"
#include "pxr/usd/usd/attribute.h"
#include "pxr/usd/usd/prim.h"

namespace rigExec {

RigExecTapSet::RigExecTapSet(const UsdStageRefPtr &stage)
    : _stage(stage)
    , _system(std::make_unique<ExecUsdSystem>(UsdStageConstRefPtr(stage)))
{
}

RigExecTapSet::~RigExecTapSet() = default;

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

    _request = std::make_unique<ExecUsdRequest>(_system->BuildRequest(
        std::move(keys),
        [this](const ExecRequestIndexSet &, const EfTimeInterval &) {
            // Invalidation callbacks only record dirtiness; evaluation
            // happens on the caller's next pull (spec §6.3, §9.3).
            _dirty = true;
        },
        [this](const ExecRequestIndexSet &) {
            _dirty = true;
        }));
    if (!_request->IsValid()) {
        return false;
    }
    _system->PrepareRequest(*_request);
    _prepared = true;
    return true;
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
    if (!_prepared) {
        Prepare();
    }

    RigExecSnapshot snapshot;
    snapshot._time = time;
    if (!_request || !_request->IsValid()) {
        return snapshot;
    }

    _system->ChangeTime(time);

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

    ExecUsdCacheView view = execOverrides.empty()
        ? _system->Compute(*_request)
        : _system->ComputeWithOverrides(*_request, std::move(execOverrides));

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
