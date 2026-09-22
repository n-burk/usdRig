//
// RigExec typed extraction API (spec §9), v0.1 subset.
//
// Backend-neutral addresses resolve through the USD implementation to
// ExecUsdValueKey batches; snapshots copy immutable values before crossing
// threads (spec §6.1).
//
#ifndef RIGEXEC_TAP_SET_H
#define RIGEXEC_TAP_SET_H

#include "pxr/pxr.h"
#include "pxr/base/tf/token.h"
#include "pxr/base/vt/value.h"
#include "pxr/usd/sdf/path.h"
#include "pxr/usd/usd/stage.h"
#include "pxr/usd/usd/timeCode.h"
#include "pxr/usd/usd/notice.h"

#include <atomic>
#include <memory>
#include <vector>

PXR_NAMESPACE_OPEN_SCOPE
class ExecUsdSystem;
class ExecUsdRequest;
PXR_NAMESPACE_CLOSE_SCOPE

PXR_NAMESPACE_USING_DIRECTIVE

namespace rigExec {

class RigExecTapContext;

/// Backend-neutral value address (spec §9.1, v0.1 subset).
///
/// A prim target with publicComputation names a rig value provider
/// (e.g. computePointFrame). A property target with an empty
/// publicComputation resolves to the stock computeValue bridge. The
/// canonical public address never names a generated provider; the
/// compiler-private resolution (RigExecTapSet::AddResolved) maps a public
/// address to its hidden chain-head provider (spec §9.1: generated
/// provider paths never enter a public or durable tap address).
struct RigExecValueAddress {
    SdfPath target;
    TfToken publicComputation;
    /// base, afterMover, or final (informational public identity).
    TfToken phase = TfToken("final");

    static RigExecValueAddress Prim(
        const SdfPath &path, const TfToken &computation,
        const TfToken &phase = TfToken("final")) {
        return {path, computation, phase};
    }
    static RigExecValueAddress Property(
        const SdfPath &propertyPath,
        const TfToken &phase = TfToken("final")) {
        return {propertyPath, TfToken(), phase};
    }
};

using RigExecTapId = int;

/// A computed value the engine supplies rather than lets exec derive.
///
/// The one v0.1 use is the solver->joint pose. OpenExec can only reach a
/// provider through a relationship authored on the consuming prim, and the
/// authored direction here is the wrong way round: a solver names its joints,
/// not the reverse (there is no reverse-relationship accessor -- see the
/// accessor list in exec/computationBuilders.h). Authoring the reverse link
/// per joint is what the derived evaluation stage existed to hold. Supplying
/// the frame as an override instead keeps the binding in the compiled graph
/// where it belongs, and exec's own consumers -- computeMatrix, frame-chain
/// applications, and the NamespaceAncestor fallback that unbound descendants
/// follow -- all read the overridden value with nothing authored anywhere.
struct RigExecValueOverride {
    SdfPath prim;
    /// Prim computation to override. Ignored when \p attribute is set.
    TfToken computation;
    /// When non-empty, overrides this attribute's value on \p prim instead
    /// of a prim computation. Used for inputs an exec accessor cannot reach
    /// from the consuming prim -- a ribbon's driver-curve points live on the
    /// curve, and a relationship accessor can only request computations on
    /// the targets, not a named attribute of them.
    TfToken attribute;
    VtValue value;

    /// Exact equality: the burst cache validates its prepared overrides
    /// against the caller's per frame, and a differing override list means
    /// differing placement, differing samples, and a differing digest.
    bool operator==(const RigExecValueOverride &other) const
    {
        return prim == other.prim && computation == other.computation &&
               attribute == other.attribute && value == other.value;
    }

    bool operator!=(const RigExecValueOverride &other) const
    {
        return !(*this == other);
    }
};

/// Immutable extracted generation: one value per tap.
class RigExecSnapshot {
public:
    const VtValue &Get(RigExecTapId tap) const {
        static const VtValue empty;
        return tap >= 0 && static_cast<size_t>(tap) < _values.size()
            ? _values[tap] : empty;
    }

    template <class T>
    T Get(RigExecTapId tap) const {
        const VtValue &v = Get(tap);
        return v.IsHolding<T>() ? v.UncheckedGet<T>() : T();
    }

    UsdTimeCode GetTime() const { return _time; }

    /// True when evaluation ran and produced this snapshot.
    bool IsValid() const { return _valid; }

    /// True when every requested tap produced a value. A valid snapshot
    /// with missing values means some computations failed to compile or
    /// evaluate; consumers must not treat defaulted values as results.
    bool IsComplete() const { return _valid && _complete; }

private:
    friend class RigExecTapSet;
    std::vector<VtValue> _values;
    UsdTimeCode _time = UsdTimeCode::Default();
    bool _valid = false;
    bool _complete = false;
};

/// Typed batched extraction over one ExecUsdSystem (spec §9.1).
///
/// Usage: Add() every address, Prepare() once, then Evaluate() per time.
/// Adding taps after Prepare() rebuilds the request on the next Prepare().
class RigExecTapSet {
public:
    explicit RigExecTapSet(const UsdStageRefPtr &stage);
    ~RigExecTapSet();

    RigExecTapSet(const RigExecTapSet &) = delete;
    RigExecTapSet &operator=(const RigExecTapSet &) = delete;

    /// Registers an address; returns its stable tap id.
    RigExecTapId Add(const RigExecValueAddress &address);

    /// Registers a public canonical address whose value is produced by a
    /// compiler-private provider (a generated chain head). The private
    /// path is used only to build the internal value key; it is never
    /// part of the public address (spec §9.1).
    RigExecTapId AddResolved(
        const RigExecValueAddress &publicAddress,
        const SdfPath &privateProvider);

    /// The public canonical address of a tap.
    const RigExecValueAddress &GetAddress(RigExecTapId tap) const {
        static const RigExecValueAddress empty;
        return tap >= 0 && static_cast<size_t>(tap) < _addresses.size()
            ? _addresses[tap] : empty;
    }

    /// Builds and front-loads the batched request schedule. Returns
    /// false when the request could not be built valid.
    bool Prepare();

    /// Populates the shared executor at \p time without copying values out.
    ///
    /// Every pull in this evaluator carries overrides, and an override-bearing
    /// compute runs in a throwaway sub-executor seeded from the main one: with
    /// nothing ever computed into the main executor it starts empty every
    /// time, so the whole network is recomputed on every call. One plain
    /// Compute() first leaves the shared cache warm and the overridden pull
    /// only has to recompute what the overrides actually reach.
    void Warm(UsdTimeCode time);

    /// Serialized ChangeTime + Compute + immutable value copy.
    RigExecSnapshot Evaluate(UsdTimeCode time);

    /// Evaluates with \p overrides standing in for the named computations.
    ///
    /// Overrides apply to this call only; they do not persist into later
    /// Evaluate calls or affect cached values (ExecUsdSystem contract).
    RigExecSnapshot Evaluate(
        UsdTimeCode time, const std::vector<RigExecValueOverride> &overrides);

    /// Returns and clears the dirty flag raised by invalidation callbacks.
    bool ConsumeDirty() { return _dirty.exchange(false); }

    ExecUsdSystem *GetSystem();

    /// Retire requests before stock Esf processes a removed prim. Consumers
    /// that evaluate synchronously inside a USD notice call this first.
    static void PrepareStageChange(const UsdStageRefPtr &stage,
                                  const UsdNotice::ObjectsChanged &notice);

private:
    friend class RigExecTapContext;
    UsdStageRefPtr _stage;
    std::shared_ptr<RigExecTapContext> _context;
    std::unique_ptr<ExecUsdRequest> _request;
    std::vector<RigExecValueAddress> _addresses;
    /// Compiler-private resolutions parallel to _addresses; an empty path
    /// means the public address resolves directly.
    std::vector<SdfPath> _resolutions;
    std::atomic<bool> _dirty{false};
    bool _prepared = false;
    /// Diagnostic only (TF_DEBUG=RIGEXEC_TAP_TIMING): how many times this
    /// tap set has rebuilt its request.
    size_t _prepareCount = 0;
};

}  // namespace rigExec

#endif  // RIGEXEC_TAP_SET_H
