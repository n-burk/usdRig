// Compile/link/load qualification of the stock Esf/Exec request seam (spec 11.2).
// Uses the shipped USD adapter as fixture storage; this is deliberately not a
// standalone scene database or backend parity claim. No private implementation
// is copied and no OpenUSD exports or sources are changed.
#include "pxr/exec/exec/system.h"
#include "pxr/exec/exec/systemChangeProcessor.h"
#include "pxr/exec/exec/requestImpl.h"
#include "pxr/exec/exec/cacheView.h"
#include "pxr/exec/exec/builtinComputations.h"
#include "pxr/exec/exec/valueKey.h"
#include "pxr/exec/exec/valueOverride.h"
#include "pxr/exec/esfUsd/sceneAdapter.h"
#include "pxr/exec/esfUsd/stageData.h"
#include "pxr/exec/ef/time.h"
#include "pxr/base/tf/functionRef.h"
#include "pxr/base/tf/errorMark.h"
#include "pxr/base/tf/span.h"
#include "pxr/base/vt/value.h"
#include "pxr/usd/sdf/types.h"
#include "pxr/usd/usd/stage.h"

#include <atomic>
#include <cstdio>

PXR_NAMESPACE_USING_DIRECTIVE

namespace {
// Fixture storage must retain the shipped adapter's connection-table state.
// Change delivery to ProbeSystem remains explicit so its generic hooks are
// actually exercised; no ExecUsdSystem participates in this probe.
class FixtureStageData final : public EsfUsdStageData::ListenerBase {
public:
    explicit FixtureStageData(const UsdStageRefPtr &stage)
        : _data(EsfUsdStageData::RegisterStage(stage, this)) {}
    ~FixtureStageData() override { _data->Unregister(this); }
private:
    void _DidObjectsChanged(const UsdNotice::ObjectsChanged &,
        const EsfUsdStageData::ChangedPathSet &) const override {}
    std::shared_ptr<EsfUsdStageData> _data;
};

class ProbeSystem final : public ExecSystem {
public:
    explicit ProbeSystem(const UsdStageRefPtr &stage)
        : ExecSystem(EsfUsdSceneAdapter::AdaptStage(stage)) {}
    ~ProbeSystem() = default;

    void Time(UsdTimeCode time) { _ChangeTime(EfTime(time)); }
    void ValueChanged(const SdfPath &path) {
        _ChangeProcessor changes(this);
        changes.DidChangeInfoOnly(path, {TfToken("timeSamples")});
    }
    void Resync(const SdfPath &path) {
        _ChangeProcessor changes(this);
        changes.DidResync(path);
    }
    void ConnectionsChanged(const SdfPath &path) {
        _ChangeProcessor changes(this);
        changes.DidChangeIncomingConnections(path);
    }
    size_t RequestCount() const {
        std::atomic<size_t> count{0};
        auto countRequest = [&](Exec_RequestImpl &) { ++count; };
        _ParallelForEachRequest(countRequest);
        return count;
    }
};

class ProbeRequest final : public Exec_RequestImpl {
public:
    ProbeRequest(ProbeSystem *system, int *values, int *times)
        : Exec_RequestImpl(system,
            [values](const ExecRequestIndexSet &, const EfTimeInterval &) { ++*values; },
            [times](const ExecRequestIndexSet &) { ++*times; }) {}
    ~ProbeRequest() = default;
    void Prepare(const std::vector<ExecValueKey> &keys) {
        if (_RequiresCompilation()) _Compile(keys);
        _Schedule();
    }
    Exec_CacheView Compute() { return _Compute(); }
    Exec_CacheView Override(ExecValueOverrideVector values) {
        return _ComputeWithOverrides(std::move(values));
    }
    void ExpireAndDiscard() {
        _ExpireIndices(ExecRequestIndexSet{0});
        _Discard();
    }
};
}

int main()
{
    TfErrorMark errors;
    int failures = 0;
    const auto check = [&](bool condition, const char *message) {
        if (!condition) { ++failures; std::printf("FAIL: %s\n", message); }
    };
    const auto stage = UsdStage::CreateInMemory();
    const auto prim = stage->DefinePrim(SdfPath("/Probe"));
    const auto attribute = prim.CreateAttribute(TfToken("value"), SdfValueTypeNames->Double);
    attribute.Set(1.0, UsdTimeCode(1));
    attribute.Set(2.0, UsdTimeCode(2));
    FixtureStageData fixture(stage);
    ProbeSystem system(stage);
    system.Time(UsdTimeCode(1));
    int valueInvalidations = 0, timeInvalidations = 0;
    ProbeRequest request(&system, &valueInvalidations, &timeInvalidations);
    const auto key = [&]() {
        return ExecValueKey(EsfUsdSceneAdapter::AdaptObject(attribute),
                            ExecBuiltinComputations->computeValue);
    };
    std::vector<ExecValueKey> keys;
    keys.push_back(key());
    check(system.RequestCount() == 1, "request registration/tracker iteration");
    request.Prepare(keys);
    const VtValue retained = request.Compute().Get(0);
    check(retained.IsHolding<double>() && retained.Get<double>() == 1, "prepare/compute/extract");
    for (double value : {3.0, 4.0}) {
        const int before = valueInvalidations;
        attribute.Set(value, UsdTimeCode(1));
        system.ValueChanged(attribute.GetPath());
        request.Prepare(keys);
        check(request.Compute().Get(0) == VtValue(value), "authored input invalidation");
        check(valueInvalidations > before, "ordinary compute renews invalidation interest");
    }
    check(retained == VtValue(1.0), "copied result outlives later request computation");
    {
        ExecValueOverrideVector overrides;
        overrides.push_back({key(), VtValue(9.0)});
        const auto view = request.Override(std::move(overrides));
        check(view.Get(0) == VtValue(9.0), "override cache-view lifetime/extraction");
    }
    check(request.Compute().Get(0) == VtValue(4.0), "override does not author or persist");
    system.Time(UsdTimeCode(2));
    request.Prepare(keys);
    check(request.Compute().Get(0) == VtValue(2.0), "time change");
    check(timeInvalidations > 0, "time invalidation callback");
    system.Time(UsdTimeCode::PreTime(2));
    request.Prepare(keys);
    check(request.Compute().Get(0) == VtValue(2.0), "pre-time identity accepted");
    system.ConnectionsChanged(attribute.GetPath());
    system.Resync(attribute.GetPath());
    request.Prepare(keys);
    check(request.Compute().Get(0) == VtValue(2.0), "resync/reprepare");
    request.ExpireAndDiscard();
    check(system.RequestCount() == 0, "expire providers and discard outside tracker lock");
    check(errors.IsClean(), "no USD coding/runtime errors");
    for (const auto &error : errors) {
        std::printf("USD error: %s\n", error.GetCommentary().c_str());
    }
    errors.Clear();
    std::printf("probeEsfCompatibility: %s\n", failures ? "FAILED" : "PASS");
    return failures ? 1 : 0;
}
