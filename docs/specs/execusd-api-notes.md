# OpenExec (ExecUsd) Client Evaluation API — Condensed Reference

Source studied: the OpenUSD v26.08 `pxr/exec/` tree (headers:
`execUsd/system.h`, `execUsd/request.h`, `execUsd/valueKey.h`,
`execUsd/cacheView.h`, `execUsd/valueOverride.h`, `exec/request.h`,
`exec/builtinComputations.h`, `ef/time.h`, `ef/timeInterval.h`; tests:
`execUsd/testenv/*`; docs: `execUsd/docs/tutorial1ComputingValues.md`).

Audience: the RigExecTapSet facade — stage → system → batched request →
prepare → per-time compute → typed extraction → invalidation subscription.

---

## 1. Core headers a client includes

```cpp
#include "pxr/exec/execUsd/system.h"      // ExecUsdSystem
#include "pxr/exec/execUsd/request.h"     // ExecUsdRequest
#include "pxr/exec/execUsd/valueKey.h"    // ExecUsdValueKey
#include "pxr/exec/execUsd/cacheView.h"   // ExecUsdCacheView
#include "pxr/exec/execUsd/valueOverride.h" // ExecUsdValueOverride (optional)
#include "pxr/exec/exec/request.h"        // ExecRequestIndexSet + callback typedefs
#include "pxr/exec/exec/builtinComputations.h" // ExecBuiltinComputations tokens
#include "pxr/exec/ef/timeInterval.h"     // EfTimeInterval (value-invalidation cb)
#include "pxr/exec/ef/time.h"             // EfTime (only if consuming computeTime)
```

For `computeLocalToWorldTransform` on Xformables: `pxr/exec/execGeom/tokens.h`
(`ExecGeomXformableTokens->computeLocalToWorldTransform`, links `execGeom`).

## 2. ExecUsdSystem (`execUsd/system.h`)

Non-copyable, non-movable. Extends the lifetime of the stage it wraps
(holds a ref); normally lives right alongside the UsdStage.

```cpp
class ExecUsdSystem : public ExecSystem {
public:
    explicit ExecUsdSystem(const UsdStageConstRefPtr &stage);
    ~ExecUsdSystem();

    void ChangeTime(UsdTimeCode time);

    ExecUsdRequest BuildRequest(
        std::vector<ExecUsdValueKey> &&valueKeys,
        ExecRequestComputedValueInvalidationCallback &&valueCallback = {},
        ExecRequestTimeChangeInvalidationCallback &&timeCallback = {});

    void PrepareRequest(const ExecUsdRequest &request);        // compile + schedule
    ExecUsdCacheView Compute(const ExecUsdRequest &request);   // implicit prepare
    ExecUsdCacheView ComputeWithOverrides(
        const ExecUsdRequest &request,
        ExecUsdValueOverrideVector &&valueOverrides);           // one-shot overrides
};
```

Notes:
- The system internally subscribes to UsdStage change notices (a private
  `_NoticeListener`); **any authoring on the stage is immediately reflected**
  (invalidates cached values, may trigger request value-invalidation callbacks
  synchronously during `UsdAttribute::Set()` / `SdfChangeBlock` close).
- Constructor takes `UsdStageConstRefPtr` — a `UsdStageRefPtr` converts
  implicitly.
- Multiple requests can share one system; compilation/schedule state is
  amortized across them.

### Time model
- `ChangeTime(UsdTimeCode)` sets the single "current evaluation time" of the
  whole system. There is **no per-Compute time argument** — the pattern is:
  `ChangeTime(t); Compute(reqA); Compute(reqB); ChangeTime(t2); ...`
  (compute *all* requests at one time before moving to the next time; this
  keeps time-dependent intermediate results cached — per system.h note).
- A fresh system starts at `UsdTimeCode::Default()`.
- `ChangeTime` re-resolves time-dependent inputs and invalidates only values
  whose inputs *actually differ* between old and new time. Revisiting a time
  whose values are still cached recomputes nothing
  (verified by `testExecUsdRequest.cpp` `TestTimeVaryingCache`).
- `EfTime` (ef/time.h) is the internal time type: wraps a `UsdTimeCode` plus
  app-specific `SplineEvaluationFlags` (uint8_t). Client-facing ExecUsd API
  only speaks `UsdTimeCode`. **There is no "PreTime"/evaluation-location API
  in this version** (left-side evaluation is not exposed; EfTime has only
  timeCode + spline flags).
- The builtin stage computation `ExecBuiltinComputations->computeTime`
  produces an `EfTime` (used as an input by time-dependent plugin
  computations; can also be requested directly with the stage pseudo-root
  as provider — rarely needed by clients).

## 3. ExecUsdValueKey (`execUsd/valueKey.h`)

```cpp
explicit ExecUsdValueKey(const UsdAttribute &provider);
// == { provider, ExecBuiltinComputations->computeValue }  (see valueKey.cpp)

ExecUsdValueKey(const UsdAttribute &provider, const TfToken &computation);
ExecUsdValueKey(const UsdPrim &provider, const TfToken &computation);
```

- Brace-init works in the vector: `{prim, TfToken("computeXf")}`,
  `{attr}` (attribute computeValue), `{attr, someAttrComputation}`.
- Keys are copyable value types holding a `std::variant` of
  {expired, attribute-computation, prim-computation}. Keys for objects that
  get removed from the stage are internally "expired" (request stays safe;
  see `testExecUsdRequestExpiration.cpp`).

### Builtin computation tokens (`exec/builtinComputations.h`)
Accessed via `ExecBuiltinComputations->...`:
- `computeValue` (attribute provider) — attribute's computed value:
  1) registered attribute expression, else 2) single valid connection's
  computed value, else 3) resolved (authored) value. Result type = the
  attribute's scalar value type.
- `computeResolvedValue` (attribute provider) — always the resolved authored
  value at the current system time (respects default/timeSamples/splines;
  see `testExecUsdInvalidation.cpp`).
- `computeTime` (stage provider) — current time as `EfTime`.
- `computePath` — provider's `SdfPath`.

## 4. ExecUsdRequest (`execUsd/request.h`)

- **Move-only** handle (`unique_ptr` impl). Keep it alive as long as you
  compute with it / want invalidation callbacks.
- `bool IsValid() const` — true means Prepare/Compute may be called (not that
  anything is cached/compiled).
- Value-key **index = position in the vector** passed to `BuildRequest`; the
  same indices are used by `CacheView::Get(i)` and both callbacks.
- Destroying the request unregisters it (callbacks stop).

## 5. ExecUsdCacheView (`execUsd/cacheView.h`)

```cpp
ExecUsdCacheView view = system.Compute(request);
VtValue v = view.Get(0);          // by request index
double d = v.Get<double>();       // typed extraction; v.IsHolding<T>() to test
```

- `Get(int index) const` returns `VtValue`; emits a TF error and returns an
  empty `VtValue` if the index was not evaluated.
- Default-constructed view is invalid.
- **Lifetime: must not outlive the system OR the request** it came from. It is
  a cheap view; re-obtain a fresh one from every `Compute()` call (values
  extracted after later scene edits/time changes would otherwise be stale —
  extract promptly after Compute).
- Concurrent `Get()` from multiple threads on one view is supported
  (`testExecUsdRequest.cpp` extracts under `WorkParallelForN`).
- Any `VtValue`-holdable type comes through: plugin-registered custom types,
  `GfMatrix4d`, `double`, `std::string`, `VtArray<T>` (extract with
  `v.Get<VtArray<T>>()` / `v.IsHolding<VtDoubleArray>()`), `UsdTimeCode`, ...

## 6. Invalidation (`exec/request.h`)

Callbacks are registered **only at BuildRequest time** (3rd/2nd args):

```cpp
using ExecRequestIndexSet = pxr_tsl::robin_set<int>;   // set of request indices

using ExecRequestComputedValueInvalidationCallback =
    std::function<void(const ExecRequestIndexSet &, const EfTimeInterval &)>;

using ExecRequestTimeChangeInvalidationCallback =
    std::function<void(const ExecRequestIndexSet &)>;
```

Semantics (system.h docs + `testExecUsdRequestInvalidation.cpp`):
- **valueCallback**: fired on authored-value / structural scene changes that
  invalidate previously computed values. Delivers the invalid request indices
  plus the `EfTimeInterval` over which they are invalid (authored default
  value changes deliver the *full* interval; `interval.IsFullInterval()`,
  `IsEmpty()`, `Contains(EfTime)`, `GetTimeMultiInterval()`, and `|=` to
  accumulate). Multiple simultaneous invalidations may be batched into one
  invocation (e.g. one `SdfChangeBlock` → one callback).
- **One-shot interest**: each callback is only guaranteed once per
  (index, interval) combination *after a Compute*. Re-calling `Compute()`
  renews interest; repeated edits between Computes may not re-notify.
  Pattern: on callback → mark dirty; next pull → Compute → extract.
- **timeCallback**: fired from within `ChangeTime()` with the indices of
  time-dependent keys whose inputs actually differ between old and new time.
  No interval argument.
- **Threading/reentrancy caveats**:
  - Callbacks run **synchronously on the thread doing the authoring**
    (`attr.Set(...)`, change-block close) or the thread calling
    `ChangeTime()`. There is no queue/deferral.
  - You must NOT call back into execution (Compute, PrepareRequest, value
    extraction, ...) from inside either callback. Record dirty state and
    return.
- No callback fires during the initial `Compute()` itself, and edits to
  irrelevant fields (e.g. documentation metadata) do not notify.

## 7. ComputeWithOverrides (`execUsd/valueOverride.h`)

```cpp
struct ExecUsdValueOverride {
    ExecUsdValueKey valueKey;   // which computed value to replace
    VtValue overrideValue;      // must match the computation's value type
};
using ExecUsdValueOverrideVector = std::vector<ExecUsdValueOverride>;

ExecUsdCacheView v = system.ComputeWithOverrides(request,
    { {ExecUsdValueKey{prim, token}, VtValue(3.0)} });
```

Overrides apply to that single invocation only; downstream dependents consume
the overridden value. Type mismatch → coding error.

## 8. Minimal end-to-end example

```cpp
#include "pxr/exec/execUsd/system.h"
#include "pxr/exec/execUsd/request.h"
#include "pxr/exec/execUsd/valueKey.h"
#include "pxr/exec/execUsd/cacheView.h"
#include "pxr/exec/exec/request.h"
#include "pxr/exec/ef/timeInterval.h"
#include "pxr/exec/execGeom/tokens.h"
#include "pxr/base/gf/matrix4d.h"
#include "pxr/usd/usd/stage.h"

PXR_NAMESPACE_USING_DIRECTIVE

void Run()
{
    UsdStageRefPtr stage = UsdStage::Open("rig.usda");

    // 1. System (one per stage; holds compiled network, listens to changes).
    ExecUsdSystem system(stage);

    // 2. Batched value keys. Index in this vector == extraction index.
    UsdPrim xform   = stage->GetPrimAtPath(SdfPath("/Root/A1"));
    UsdAttribute in = stage->GetAttributeAtPath(SdfPath("/Root/A1.myAttr"));

    std::vector<ExecUsdValueKey> keys {
        {xform, ExecGeomXformableTokens->computeLocalToWorldTransform}, // [0] prim comp
        {xform, TfToken("computeMyCustomThing")},                       // [1] custom plugin comp
        {in},                                       // [2] attribute computeValue (builtin)
    };

    // 3. Request with invalidation subscriptions (registered here, only here).
    std::atomic<bool> dirty{false};
    ExecUsdRequest request = system.BuildRequest(
        std::move(keys),
        /*valueCallback*/ [&](const ExecRequestIndexSet &indices,
                              const EfTimeInterval &interval) {
            // Runs synchronously during stage edits. Do NOT compute here.
            for (int i : indices) { /* mark tap i dirty over 'interval' */ }
            dirty = true;
        },
        /*timeCallback*/ [&](const ExecRequestIndexSet &indices) {
            // Runs synchronously inside ChangeTime for actually-varying keys.
            dirty = true;
        });
    TF_AXIOM(request.IsValid());

    // 4. Front-load compile + schedule.
    system.PrepareRequest(request);

    // 5. Evaluate over times: set time, compute, extract immediately.
    for (double t : {1.0, 2.0, 3.0}) {
        system.ChangeTime(UsdTimeCode(t));
        ExecUsdCacheView view = system.Compute(request);   // renews invalidation interest

        VtValue v0 = view.Get(0);
        GfMatrix4d xf = v0.Get<GfMatrix4d>();

        VtValue v1 = view.Get(1);                // custom-typed plugin value
        // if (v1.IsHolding<MyRigType>()) { ... }

        VtValue v2 = view.Get(2);                // attribute value (scalar or VtArray)
        // if (v2.IsHolding<VtDoubleArray>()) { auto arr = v2.Get<VtDoubleArray>(); }
    }
    // Default (non-animated) evaluation:
    system.ChangeTime(UsdTimeCode::Default());
    ExecUsdCacheView dv = system.Compute(request);
}
```

Custom computations are published from a plugin via
`EXEC_REGISTER_COMPUTATIONS_FOR_SCHEMA(SchemaTypeName) { self.PrimComputation(token).Callback<T>(...).Inputs(...); }`
(`pxr/exec/exec/registerSchema.h`; expands to a
`TF_REGISTRY_FUNCTION(ExecDefinitionRegistryTag)`), living in a library whose
schema types are declared in `plugInfo.json`. Exec loads the plugin defining
computations for a schema on demand.

## 9. Linking and environment

`execUsd`'s own CMake `LIBRARIES` (all public link deps of the library):
`esf esfUsd exec tf trace sdf usd`.

A realistic client link line (mirrors the testenv executables), superset:

```
execUsd exec esf ef vdf     # exec core (ef for EfTimeInterval, vdf if writing computations)
usd sdf plug tf gf vt work trace
execGeom usdGeom            # only if using computeLocalToWorldTransform etc.
ts                          # only if authoring/inspecting splines
```

Minimum for pure evaluation + invalidation: `execUsd exec ef usd sdf tf gf vt`
(transitives via execUsd cover esf/esfUsd/trace). If you register your own
computations you also need `vdf` (VdfContext) and `plug`.

Build/runtime requirements:
- OpenUSD must be built with exec enabled (`PXR_BUILD_EXEC=ON` /
  `build_usd.py --exec`); the `pxr/exec` tree is v26.08-era and marked
  experimental/in-development.
- Plugin discovery: `execUsd`, `execGeom`, and any custom computation plugin
  must be discoverable via the usual `PXR_PLUGINPATH_NAME` / installed
  `plugInfo.json` mechanism.
- Env settings (TfEnvSetting):
  - `VDF_ENABLE_PARALLEL_EVALUATION_ENGINE` — default **true**
    (`pxr/exec/vdf/types.cpp`); enables parallel evaluation within a single
    round of exec evaluation. Set to `0` to force serial engine when
    debugging.
  - No other exec-specific env settings are defined under `pxr/exec`.
- Useful debug flags (TF_DEBUG): `EXEC_REQUEST_INVALIDATION` (request
  invalidation traffic); `ExecSystem::Diagnostics(&system).InvalidateAll()`
  and `.GraphNetwork("out.dot")` exist on `exec/systemDiagnostics.h` for
  debugging.

## 10. Facade design implications (RigExecTapSet)

- Hold: `UsdStageRefPtr` (or rely on system's ref), one `ExecUsdSystem`, one
  move-only `ExecUsdRequest`, and the key vector's index→tap mapping.
- Rebuilding the tap set means building a *new* request (keys are fixed at
  BuildRequest; there is no append API).
- Invalidations arrive synchronously on authoring/ChangeTime threads: your
  callback should only flip dirty bits / forward notifications; evaluation
  must happen later on the caller's pull.
- Re-Compute after invalidation to keep callbacks flowing (interest renewal).
- Cache views are transient: compute → extract → drop; never store a view
  across stage edits, ChangeTime, or request/system destruction.
