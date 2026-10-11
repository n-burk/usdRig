// Stage-side frame sampling and burst sample caches.

#include "frozenContextInternal.h"
#include "oracleDispatch.h"
#include "frameCache.h"
#include "movers/moverRegistry.h"
#include "rigExecMath/geometryKernels.h"
#include "pxr/base/tf/diagnostic.h"
#include <algorithm>
#include <array>
#include <cmath>
#include <memory>
#include <set>
#include <tuple>
#include <type_traits>

namespace rigExec {

// Cold owning-thread prewarm of retained source-sampler keys only.
// The deleted frozen body/scheduler owns no tokens or execution here.
void RigExecFrozenGeometryTouchTokens()
{
    (void)frozenDetail::_frozenWeightTokens.Get();
}

/// The frozen samplers' memo on one program, one per Default-ness of the
/// time (RigExecBakedProgramImpl::frozenSamplerMemo), owning thread only.
/// `stamp`, `serial` and `atDefault` name the state its reads were taken
/// under: the program stamp, the evaluator's stage edit serial and the
/// Default-ness of the time. Every stage notice advances the serial, so
/// while the three stand a read that cannot move with the time (a leaf or
/// a resolved walk by RigExecRevisionLeafHops, a query by its own
/// ValueMightBeTimeVarying, asked under that state) reads what it read
/// then. The recorded digest orders depend on the sample path sequence
/// alone and outlive the state.
struct RigExecFrozenSamplerMemo {
    /// A held read taken at every sample, or sampling nothing.
    static constexpr int32_t kPerFrame = -1;
    static constexpr int32_t kNoSample = -2;
    /// A transport leaf's classification, made on first use under the state.
    enum : uint8_t { kUnknown = 0, kVarying, kStatic, kStaticRead };

    bool built = false;
    uint64_t stamp = 0;
    uint64_t serial = 0;
    bool atDefault = false;
    /// RIGEXEC_VERIFY_FROZEN_STATIC, copied from the program.
    bool verify = false;
    /// Every held read's sample, read when the state changed: the provider
    /// and weight-oracle leaves, the source-backed bindings, the chain and
    /// derived bases and the blend channel reads.
    std::shared_ptr<const RigExecFrozenStaticSamples> samples;
    /// Per provider leaf, its `samples` entry, kPerFrame or kNoSample.
    std::vector<int32_t> providerEntries;
    /// Per weight object and oracle key, its entry or kPerFrame, and the
    /// paths its read can reach (served only while no override or upstream
    /// value stands on one).
    std::vector<std::vector<int32_t>> oracleEntries;
    std::vector<std::vector<std::vector<SdfPath>>> oracleHops;
    /// Per revision, derived target and layout row: each key's
    /// classification, its value once read, and the paths its read can
    /// reach.
    struct Row {
        std::vector<uint8_t> state;
        std::vector<VtValue> values;
        std::vector<std::vector<SdfPath>> hops;
    };
    std::vector<Row> revisionRows;
    std::vector<Row> derivedRows;
    std::vector<Row> layoutRows;
    /// Per leaf of the double and GfVec3d pools: the `samples` entry of the
    /// source-backed binding numbered there (its query read), or kPerFrame.
    std::vector<int32_t> sourceDouble, sourceVec3d;
    /// Per chain, and per chain and derived target: the base query's
    /// entry, or kPerFrame.
    std::vector<int32_t> chainBase;
    std::vector<std::vector<int32_t>> derivedBase;
    /// One read through the resolved inputs: its entry or kPerFrame, and
    /// the paths its walk can reach (served only while no override or
    /// upstream value stands on one).
    struct HeldRead {
        int32_t entry = kPerFrame;
        std::vector<SdfPath> hops;
    };
    struct HeldChannel {
        HeldRead weight;
        std::vector<HeldRead> activation, points;
    };
    /// Per chain, revision and blend channel: its weight and per sample
    /// its activation and dense points. The pinned route serves them; the
    /// burst route keeps its own maps.
    std::vector<std::vector<std::vector<HeldChannel>>> blend;
    /// Recorded digest orders, most recently used first.
    std::array<std::shared_ptr<const RigExecFrameDigestOrder>, 4> orders;
};

using namespace frozenDetail;

namespace {

// The sampler below mirrors the binding walk the Stream 0 bench measures
// (tests/benchFrameCache.cpp MeasureSampling): the same tables, in the same
// order, through the same read route the baked frame path takes. The bench
// is the timing; this is the values.

// Reads a chain-resolved attribute exactly as RigExecResolvedInputs::
// GetAttribute does -- an in-memory property value wins, otherwise a single
// authored connection is followed, otherwise the attribute's own value is
// read -- but type-erased into a VtValue, because the sampler names no
// binding's type.
// GetAttribute itself cannot serve here: its in-memory lookup is typed
// (Find + IsHolding<T>), and T=VtValue never matches, so the chain outputs
// the sample exists to capture would be skipped and the stage read instead.
// The walk below is that function's connection logic restated without the
// type parameter; the float-from-double coercion it also performs is a
// CONSUMPTION rule (a typed read of a double source) and stays at the typed
// reads, which is where the frame path applies it too.
//
// \p layer, when given, is the job's upstream layer: in the fallback each
// hop answers from it before the stage, as GetAttributeOverStageLayer does.
bool
_SampleResolvedAttribute(const RigExecResolvedInputs &resolved,
                         const UsdAttribute &attribute, UsdTimeCode time,
                         VtValue *out, bool *fromMemory = nullptr,
                         const std::map<SdfPath, VtValue> *layer = nullptr)
{
    std::set<SdfPath> visiting;
    std::vector<UsdAttribute> fallback;
    UsdAttribute a = attribute;
    while (a && visiting.insert(a.GetPath()).second) {
        if (const VtValue *held = resolved.Find(a.GetPath())) {
            *out = *held;
            if (fromMemory) {
                // In-memory state -- a chain output computed for the
                // evaluator's last run, or a standing override. The caller
                // decides which staleness that means for its binding.
                *fromMemory = true;
            }
            return true;
        }
        fallback.push_back(a);
        SdfPathVector connections;
        if (a.HasAuthoredConnections()) {
            a.GetConnections(&connections);
        }
        if (connections.size() != 1) {
            break;
        }
        a = a.GetPrim().GetStage()->GetAttributeAtPath(connections[0]);
    }
    for (auto it = fallback.rbegin(); it != fallback.rend(); ++it) {
        const auto upstream =
            layer ? layer->find(it->GetPath())
                  : std::map<SdfPath, VtValue>::const_iterator();
        const bool fromLayer = layer && upstream != layer->end();
        if (fromLayer) {
            *out = upstream->second;
        }
        if (fromLayer || it->Get(out, time)) {
            // A stage read at the sampled time: fresh whatever the binding.
            if (fromMemory) {
                *fromMemory = false;
            }
            return true;
        }
    }
    if (fromMemory) {
        *fromMemory = false;
    }
    return false;
}

bool _HopsTouched(const std::vector<SdfPath> &hops,
                  const std::vector<SdfPath> &touched);
void _VerifyServedSample(const RigExecSampledInput &served,
                         const RigExecSampledInput &fresh);
void _SampleQuery(const SdfPath &key, const UsdAttributeQuery &query,
                  UsdTimeCode time, RigExecFrameInputs *out);
void _SampleBlendPoints(const SdfPath &key, const UsdAttribute &attribute,
                        UsdTimeCode time,
                        const RigExecResolvedInputs *refreshed,
                        RigExecFrameInputs *out);
template <class T>
void _SampleMoverScalar(const SdfPath &key, const UsdAttribute &attribute,
                        T fallback, UsdTimeCode time,
                        const RigExecResolvedInputs *refreshed,
                        RigExecFrameInputs *out,
                        const std::map<SdfPath, VtValue> *layer = nullptr);

// Appends \p memo's sample \p entry for the read keyed \p key, unless it is
// no entry, keys another path, or a \p touched path stands on one of
// \p hops; then appends what \p read appends, which is the read itself.
// Under RIGEXEC_VERIFY_FROZEN_STATIC a served sample is checked against
// \p read's. Owning thread.
template <class Read>
void
_SampleThroughMemo(const RigExecFrozenSamplerMemo &memo, int32_t entry,
                   const SdfPath &key, const std::vector<SdfPath> *hops,
                   const std::vector<SdfPath> &touched,
                   RigExecFrameInputs *out, Read &&read)
{
    const RigExecFrozenStaticSamples *table = memo.samples.get();
    if (entry < 0 || !table || size_t(entry) >= table->samples.size() ||
        table->samples[size_t(entry)].path != key ||
        (hops && _HopsTouched(*hops, touched))) {
        read(out);
        return;
    }
    out->values.push_back(table->samples[size_t(entry)]);
    if (!memo.verify) {
        return;
    }
    RigExecFrameInputs fresh;
    read(&fresh);
    if (TF_VERIFY(fresh.values.size() == 1,
                  "frozen static read <%s> sampled nothing",
                  key.GetText())) {
        _VerifyServedSample(out->values.back(), fresh.values.back());
    }
}

// _SampleThroughMemo for a read \p memo may hold as \p held.
template <class Read>
void
_SampleHeldRead(const RigExecFrozenSamplerMemo &memo,
                const RigExecFrozenSamplerMemo::HeldRead *held,
                const SdfPath &key, const std::vector<SdfPath> &touched,
                RigExecFrameInputs *out, Read &&read)
{
    if (!held) {
        read(out);
        return;
    }
    _SampleThroughMemo(memo, held->entry, key, &held->hops, touched, out,
                       read);
}

// The memo entry of source-backed binding \p input (`sourceDouble`,
// `sourceVec3d`), or kPerFrame.
template <class T>
int32_t
_SourceEntry(const RigExecBakedInput<T> &input,
             const RigExecFrozenSamplerMemo &memo)
{
    const std::vector<int32_t> *entries = nullptr;
    if constexpr (std::is_same_v<T, double>) {
        entries = &memo.sourceDouble;
    } else if constexpr (std::is_same_v<T, GfVec3d>) {
        entries = &memo.sourceVec3d;
    }
    if (!entries || !input.sourceBacked || input.varying || input.leaf < 0 ||
        size_t(input.leaf) >= entries->size()) {
        return RigExecFrozenSamplerMemo::kPerFrame;
    }
    return (*entries)[size_t(input.leaf)];
}

// The paths a read through RigExecResolvedInputs::GetAttribute can reach
// from \p attribute, and whether one of them can move with the time: what
// RigExecRevisionLeafHops answers for a Resolved, AtTime key at its path.
void
_ResolvedWalk(const UsdAttribute &attribute, std::vector<SdfPath> *hops,
              bool *varying)
{
    RigExecRevisionLeafKey key;
    key.path = attribute.GetPath();
    key.time = RigExecRevisionLeafTime::AtTime;
    key.flavour = RigExecRevisionLeafFlavour::Resolved;
    RigExecRevisionLeafHops(key, attribute, hops, varying);
}

// Samples one bound input. Epoch constants are not per-frame inputs and are
// skipped; everything else is read through the route the frame path reads
// (RigExecBakedRead): the retained query, else the resolved walk through
// \p refreshed (the job's overrides, which are all a walk that meets no
// chain target or record consumer can read), else -- varying with neither
// handle, which the frame path answers from the typed constant -- that
// constant. \p head carries the binding's walk-start path, which keys the
// sample. A binding with a reader walk samples nothing: the worker resolves
// it from the head-leaf samples and the head tier it runs, as live does.
// A source-backed read \p memo holds is served from it.
template <class T>
void
_SampleBinding(const RigExecBakedInput<T> &input,
               const RigExecResolvedInputs *refreshed, UsdTimeCode time,
               RigExecFrameInputs *out,
               const std::map<SdfPath, VtValue> *layer = nullptr,
               const RigExecFrozenSamplerMemo *memo = nullptr)
{
    if ((!input.varying && !input.sourceBacked) || input.walk >= 0) {
        return;
    }
    const int32_t entry =
        memo ? _SourceEntry(input, *memo) : RigExecFrozenSamplerMemo::kPerFrame;
    if (entry >= 0 && input.head.IsValid()) {
        _SampleThroughMemo(*memo, entry, input.head.GetPath(), nullptr, {},
                           out, [&](RigExecFrameInputs *into) {
                               _SampleBinding(input, refreshed, time, into,
                                              layer);
                           });
        return;
    }
    VtValue value;
    bool hasValue = false;
    if (input.query.IsValid()) {
        hasValue = input.query.Get(&value, time);
    } else if (input.resolvedAttr.IsValid() && refreshed) {
        hasValue =
            _SampleResolvedAttribute(*refreshed, input.resolvedAttr, time,
                                     &value, nullptr, layer);
    } else if (input.head.IsValid()) {
        // The frame path answers input.constant here, so the sample is that
        // constant rather than a valueless marker: valueless would tell the
        // worker "no value", and the frame path never sees no value on this
        // binding.
        value = VtValue(input.constant);
        hasValue = true;
    } else {
        return;
    }
    if (!input.head.IsValid()) {
        return;
    }
    out->Add(input.head.GetPath(), value, hasValue);
}

// The constant head leaves of \p B, read fresh off the stage: the program's
// table while the state it was read under stands, else a re-read, which
// keeps the standing table when every entry is bitwise the same and
// otherwise becomes the program's. Every stage notice advances the evaluator's serial
// and a stamp bump reruns live whole, so a standing table holds what a read
// would answer now, whatever the live leaves were last sampled at -- which
// is why a snapshot holding an older constant still warms the current one.
// A constant leaf reads one value at every numeric time; Default-ness is
// part of the state, as the live sampler re-reads every leaf when the time
// moves to or from Default.
std::shared_ptr<const RigExecHeadLeafConstants>
_HeadLeafConstants(const RigExecRigEvaluator &evaluator,
                   const RigExecBakedProgramImpl &B, UsdTimeCode time)
{
    const uint64_t serial = evaluator.GetStageEditSerial();
    const std::shared_ptr<const RigExecHeadLeafConstants> memo =
        B.headLeafConstants;
    if (memo && B.headLeafConstantsStamp == B.programStamp &&
        B.headLeafConstantsSerial == serial &&
        B.headLeafConstantsDefault == time.IsDefault()) {
        return memo;
    }
    B.headLeafConstantsStamp = B.programStamp;
    B.headLeafConstantsSerial = serial;
    B.headLeafConstantsDefault = time.IsDefault();
    auto table = std::make_shared<RigExecHeadLeafConstants>();
    RigExecFrameInputs asSamples;
    uint64_t digest = 1469598103934665603ull;
    RigExecForEachHeadLeaf(B, [&](const RigExecBakedHeadLeaf &leaf) {
        const bool varies = RigExecBakedHeadLeafVaries(leaf);
        VtValue value;
        if (!varies) {
            value = RigExecBakedReadHeadLeaf(leaf, time);
        }
        table->keys.push_back(leaf.frozenKey);
        table->varying.push_back(varies ? 1 : 0);
        digest = _MixWord(digest, varies ? 1u : 0u);
        if (!varies) {
            asSamples.Add(leaf.frozenKey, value, !value.IsEmpty());
            digest = _MixWord(digest,
                              RigExecSampleDigest(asSamples.values.back()));
        }
        table->values.push_back(std::move(value));
    });
    table->digest = _MixWord(digest, uint64_t(table->keys.size()));
    table->digestible = RigExecControlStateDigestible(asSamples);
    if (memo && memo->keys == table->keys &&
        memo->varying == table->varying &&
        memo->values.size() == table->values.size()) {
        bool same = true;
        for (size_t j = 0; same && j < table->values.size(); ++j) {
            same = RigExecBakedHeadValueSame(memo->values[j],
                                             table->values[j]);
        }
        if (same) {
            return memo;
        }
    }
    B.headLeafConstants = table;
    return table;
}

// Every head leaf the property chains read, under its frozen key: the raw
// typed value RigExecBakedSampleHeadLeaves reads on the live path,
// valueless where the attribute holds none of the leaf's type. A leaf that
// varies is read at \p time into the vector; the rest ride the shared table
// (_HeadLeafConstants). The worker's head tier compares each against the
// snapshot's last sample to decide what re-runs.
//
// A leaf at a path the job's upstream \p layer holds a value of its type at
// is sampled as that value whether it varies or not, as the live sampler
// reads the layer in front of the stage; the worker takes that sample over
// the table.
void
_SampleHeadLeaves(const RigExecRigEvaluator &evaluator,
                  const RigExecBakedProgramImpl &B, UsdTimeCode time,
                  RigExecFrameInputs *out,
                  const std::map<SdfPath, VtValue> *layer = nullptr)
{
    out->headLeafConstants = _HeadLeafConstants(evaluator, B, time);
    if (out->headLeafConstants->keys.empty()) {
        out->headLeafConstants.reset();
        return;
    }
    const RigExecHeadLeafConstants &constants = *out->headLeafConstants;
    size_t j = 0;
    RigExecForEachHeadLeaf(B, [&](const RigExecBakedHeadLeaf &leaf) {
        // A leaf past the table's end is sampled; the worker then declines
        // on the key mismatch.
        const bool varies =
            j >= constants.varying.size() || constants.varying[j] != 0;
        ++j;
        if (layer && !layer->empty()) {
            const auto upstream = layer->find(leaf.path);
            if (upstream != layer->end() && leaf.typeMatches &&
                RigExecBakedHeadLeafHolds(leaf, upstream->second)) {
                out->Add(leaf.frozenKey, upstream->second, true);
                return;
            }
        }
        if (!varies) {
            return;
        }
        const VtValue value = RigExecBakedReadHeadLeaf(leaf, time);
        out->Add(leaf.frozenKey, value, !value.IsEmpty());
    });
}

template <class T>
void
_SamplePromotedAvar(const RigExecBakedInput<T> &input,
                    const RigExecResolvedInputs *resolved, UsdTimeCode time,
                    RigExecFrameInputs *out,
                    const std::map<SdfPath, VtValue> *layer = nullptr)
{
    // A constant binding an edit has animated since: read per frame the long
    // way, like a varying binding, until an edit makes it constant again.
    if (!input.head.IsValid()) {
        return;
    }
    VtValue value;
    bool hasValue = false;
    if (resolved) {
        // Never viaChain: a promoted avar classified constant at Build, so
        // no chain stands on its walk and an in-memory hit is a standing
        // override -- timeless and digest-covered, like any other.
        hasValue = _SampleResolvedAttribute(*resolved, input.head, time,
                                            &value, nullptr, layer);
    }
    if (!hasValue) {
        value = VtValue(input.constant);
        hasValue = true;
    }
    out->Add(input.head.GetPath(), value, hasValue);
}

bool
_IsOverridden(const std::vector<char> &flags, int overrideIndex)
{
    return overrideIndex >= 0 &&
           size_t(overrideIndex) < flags.size() &&
           flags[size_t(overrideIndex)];
}

// Whether one bound input needs a per-frame visit under a burst's placed
// override flags: varying inputs are re-read every frame and overridden
// ones are read the long way, while anything else early-outs without
// adding a sample -- so a struct whose every field answers false can be
// skipped without changing the vector.
template <class T>
bool
_BurstInputNeedsVisit(const RigExecBakedInput<T> &input,
                      const std::vector<char> &flags)
{
    return input.varying || input.sourceBacked || _IsOverridden(flags, input.overrideIndex);
}

bool
_BurstLadderNeedsVisit(const RigExecBakedProgramImpl::Ladder &ladder,
                       const std::vector<char> &flags)
{
    bool needed = false;
    _VisitLadderInputs(ladder, [&](const auto &input) {
        needed = needed || _BurstInputNeedsVisit(input, flags);
    });
    return needed;
}

bool
_BurstSpaceSwitchNeedsVisit(
    const RigExecBakedProgramImpl::SpaceSwitch &spaceSwitch,
    const std::vector<char> &flags)
{
    bool needed = false;
    _VisitSpaceSwitchInputs(spaceSwitch, [&](const auto &input) {
        needed = needed || _BurstInputNeedsVisit(input, flags);
    });
    return needed;
}

bool
_BurstSolverNeedsVisit(const RigExecBakedProgramImpl::Solver &solver,
                       const std::vector<char> &flags)
{
    bool needed = false;
    _VisitSolverInputs(solver, [&](const auto &input) {
        needed = needed || _BurstInputNeedsVisit(input, flags);
    });
    return needed;
}

bool
_BurstConstraintNeedsVisit(
    const RigExecBakedProgramImpl::Constraint &constraint,
    const std::vector<char> &flags)
{
    bool needed = false;
    _VisitConstraintInputs(constraint, [&](const auto &input) {
        needed = needed || _BurstInputNeedsVisit(input, flags);
    });
    return needed;
}

bool
_BurstWeightNeedsVisit(const RigExecBakedProgramImpl::WeightObject &object,
                       const std::vector<char> &flags)
{
    bool needed = false;
    _VisitWeightInputs(object, [&](const auto &input) {
        needed = needed || _BurstInputNeedsVisit(input, flags);
    });
    return needed;
}

bool
_BurstInterpolatorNeedsVisit(
    const RigExecBakedProgramImpl::PoseInterpolator &interp,
    const std::vector<char> &flags)
{
    bool needed = false;
    _VisitInterpolatorInputs(interp, [&](const auto &input) {
        needed = needed || _BurstInputNeedsVisit(input, flags);
    });
    return needed;
}

// Samples one binding under the job's overrides. Unoverridden bindings
// sample exactly as before; an overridden binding is read the long way --
// the resolved walk from its head, which is what RigExecBakedRead's
// overridden arm answers -- through the refreshed inputs (job overrides
// placed, stage reads at the job's time). The sample then carries the value
// the frame consumes, and the worker patches it as a constant; no separate
// override value travels. A binding with a reader walk samples nothing
// either way (see _SampleBinding).
//
// \p flags (and so `overridden`) also carry the override numbers an
// upstream value stands on a hop of, and the long way reads through the
// job's upstream \p layer, as RigExecBakedRead's first arm does live: the
// sample at the head is then the upstream-valued answer, constant or not.
template <class T>
void
_SampleBindingWithOverrides(const RigExecBakedInput<T> &input,
                            const RigExecResolvedInputs *refreshed,
                            bool overridden, UsdTimeCode time,
                            RigExecFrameInputs *out,
                            const std::map<SdfPath, VtValue> *layer = nullptr,
                            const RigExecFrozenSamplerMemo *memo = nullptr)
{
    if (!overridden) {
        _SampleBinding(input, refreshed, time, out, layer, memo);
        return;
    }
    if (!input.head.IsValid() || input.walk >= 0) {
        return;
    }
    T value = input.sourceBacked ? input.sourceFallback : input.constant;
    if (refreshed) {
        refreshed->GetAttributeOverStageLayer(input.head, time, layer,
                                              &value);
    }
    out->Add(input.head.GetPath(), VtValue(value), /*hasValue=*/true);
}

template <class T>
void
_SampleFlaggedBinding(const RigExecBakedInput<T> &input,
                      const RigExecResolvedInputs *refreshed,
                      const std::vector<char> &flags, UsdTimeCode time,
                      RigExecFrameInputs *out,
                      const std::map<SdfPath, VtValue> *layer = nullptr,
                      const RigExecFrozenSamplerMemo *memo = nullptr)
{
    _SampleBindingWithOverrides(input, refreshed,
                                _IsOverridden(flags, input.overrideIndex),
                                time, out, layer, memo);
}

void
_SampleSolverBindings(const RigExecBakedProgramImpl::Solver &solver,
                      const RigExecResolvedInputs *refreshed,
                      const std::vector<char> &flags, UsdTimeCode time,
                      RigExecFrameInputs *out,
                      const std::map<SdfPath, VtValue> *layer = nullptr,
                      const RigExecFrozenSamplerMemo *memo = nullptr)
{
    _VisitSolverInputs(solver, [&](const auto &input) {
        _SampleFlaggedBinding(input, refreshed, flags, time, out, layer,
                              memo);
    });
}

void
_SampleConstraintBindings(
    const RigExecBakedProgramImpl::Constraint &constraint,
    const RigExecResolvedInputs *refreshed, const std::vector<char> &flags,
    UsdTimeCode time, RigExecFrameInputs *out,
    const std::map<SdfPath, VtValue> *layer = nullptr,
    const RigExecFrozenSamplerMemo *memo = nullptr)
{
    _VisitConstraintInputs(constraint, [&](const auto &input) {
        _SampleFlaggedBinding(input, refreshed, flags, time, out, layer,
                              memo);
    });
}

// _ApplyInteractiveOverridesToResolved over the job's overrides: every
// attribute override stands in the given inputs. The `overrides` out-param
// of the live routine feeds exec only; the baked path never reads it.
void
_PlaceOverridesIntoResolved(
    const std::vector<RigExecValueOverride> &overrides,
    RigExecResolvedInputs *resolved)
{
    for (const RigExecValueOverride &o : overrides) {
        if (o.attribute.IsEmpty()) {
            continue;
        }
        resolved->SetProperty(o.prim.AppendProperty(o.attribute), o.value);
    }
}

// The paths the job's overrides and upstream values stand on, sorted: a
// memoized leaf whose read reaches one is read fresh instead.
std::vector<SdfPath>
_TouchedPaths(const std::vector<SdfPath> &overridePaths,
              const std::map<SdfPath, VtValue> *layer)
{
    std::vector<SdfPath> touched;
    for (const SdfPath &path : overridePaths) {
        if (!path.IsEmpty()) {
            touched.push_back(path);
        }
    }
    if (layer) {
        for (const auto &entry : *layer) {
            touched.push_back(entry.first);
        }
    }
    std::sort(touched.begin(), touched.end());
    touched.erase(std::unique(touched.begin(), touched.end()), touched.end());
    return touched;
}

bool
_HopsTouched(const std::vector<SdfPath> &hops,
             const std::vector<SdfPath> &touched)
{
    if (touched.empty()) {
        return false;
    }
    for (const SdfPath &hop : hops) {
        if (std::binary_search(touched.begin(), touched.end(), hop)) {
            return true;
        }
    }
    return false;
}

// RIGEXEC_VERIFY_FROZEN_STATIC: a served sample against a fresh read.
void
_VerifyServedSample(const RigExecSampledInput &served,
                    const RigExecSampledInput &fresh)
{
    TF_VERIFY(served.path == fresh.path &&
                  served.hasValue == fresh.hasValue &&
                  served.valueBlocked == fresh.valueBlocked &&
                  RigExecBakedHeadValueSame(served.value, fresh.value),
              "frozen static sample <%s> differs from a fresh read",
              fresh.path.GetText());
}

// One transport leaf of \p leaves under the sampler memo's \p row. A key
// whose read can move with the time, or reaches a \p touched path, is read
// fresh by \p read; any other is read once under the memo's state and
// served after, a refcounted copy of that read.
template <class Read>
VtValue
_MemoLeafValue(RigExecFrozenSamplerMemo::Row *row,
               const RigExecBakedPathLeaves &leaves, size_t k,
               const std::vector<SdfPath> &touched, bool verify, Read &&read)
{
    using Memo = RigExecFrozenSamplerMemo;
    const size_t n = leaves.decl.keys.size();
    if (row->state.size() != n) {
        row->state.assign(n, Memo::kUnknown);
        row->values.assign(n, VtValue());
        row->hops.assign(n, std::vector<SdfPath>());
    }
    uint8_t &state = row->state[k];
    if (state == Memo::kUnknown) {
        bool varying = false;
        RigExecRevisionLeafHops(leaves.decl.keys[k], leaves.attributes[k],
                                &row->hops[k], &varying);
        state = varying ? Memo::kVarying : Memo::kStatic;
    }
    if (state == Memo::kVarying || _HopsTouched(row->hops[k], touched)) {
        return read();
    }
    if (state == Memo::kStatic) {
        row->values[k] = read();
        state = Memo::kStaticRead;
    } else if (verify) {
        TF_VERIFY(RigExecBakedHeadValueSame(read(), row->values[k]),
                  "frozen static leaf <%s> differs from a fresh read",
                  leaves.decl.keys[k].path.GetText());
    }
    return row->values[k];
}

void
_SampleWeightBindings(const RigExecBakedProgramImpl::WeightObject &object,
                      const RigExecResolvedInputs *refreshed,
                      const std::vector<char> &flags, UsdTimeCode time,
                      RigExecFrameInputs *out,
                      const std::map<SdfPath, VtValue> *layer = nullptr,
                      const RigExecFrozenSamplerMemo *memo = nullptr)
{
    _VisitWeightInputs(object, [&](const auto &input) {
        _SampleFlaggedBinding(input, refreshed, flags, time, out, layer,
                              memo);
    });
}

// Reads one array attribute through the resolved inputs first, then the
// stage -- the route RigExecBakedWeightPacket's gather arms take
// (resolved->GetAttribute, which itself reads composed opinions on a miss,
// plus the stage fallback _SampleMoverScalar applies).
template <class T>
bool
_FrozenReadWeightArray(const RigExecResolvedInputs *refreshed,
                       const UsdAttribute &attribute, UsdTimeCode time,
                       VtArray<T> *out,
                       const std::map<SdfPath, VtValue> *layer = nullptr)
{
    if (attribute.IsValid() && refreshed &&
        refreshed->GetAttributeOverStageLayer(attribute, time, layer, out)) {
        return true;
    }
    if (attribute.IsValid() && attribute.Get(out, time)) {
        return true;
    }
    return false;
}

// Gathers one weight object's point arrays exactly as the packet build's
// gather arms do (bakedWeights.cpp), under the synthetic keys the frozen
// packet build reads. Roles with no attributes sample nothing; the worker
// answers those as empty, which is what the live gather yields. Failed
// reads contribute nothing to a concatenation, matching the live arms.
// The oracle leaves of weight object \p objectIndex that \p memo holds
// time-invariant are served from it while no \p touched path stands on
// their reads.
void
_SampleWeightArrays(const RigExecBakedProgramImpl::WeightObject &object,
                    const RigExecResolvedInputs *refreshed, UsdTimeCode time,
                    RigExecFrameInputs *out,
                    const std::map<SdfPath, VtValue> *layer,
                    const RigExecFrozenSamplerMemo &memo, size_t objectIndex,
                    const std::vector<SdfPath> &touched, bool verify)
{
    const auto gatherPoints =
        [&](const std::vector<UsdAttribute> &attributes,
            const TfToken &role) {
            if (attributes.empty()) {
                return;
            }
            VtVec3fArray gathered;
            for (const UsdAttribute &a : attributes) {
                VtVec3fArray value;
                if (_FrozenReadWeightArray(refreshed, a, time, &value,
                                           layer)) {
                    for (const GfVec3f &p : value) {
                        gathered.push_back(p);
                    }
                }
            }
            out->Add(_FrozenWeightArrayKey(object.path, role),
                     VtValue(gathered), /*hasValue=*/true);
        };
    for (size_t k=0;k<object.oracleLeaves.decl.keys.size();++k) {
        const auto &key = object.oracleLeaves.decl.keys[k];
        if (key.path.IsEmpty()) continue;
        const int32_t entry =
            objectIndex < memo.oracleEntries.size() &&
                    k < memo.oracleEntries[objectIndex].size()
                ? memo.oracleEntries[objectIndex][k]
                : RigExecFrozenSamplerMemo::kPerFrame;
        const bool served =
            entry >= 0 &&
            !_HopsTouched(memo.oracleHops[objectIndex][k], touched);
        if (served) {
            out->values.push_back(memo.samples->samples[size_t(entry)]);
            if (!verify) continue;
        }
        const VtValue value = RigExecSampleRevisionLeaf(key,
            object.oracleLeaves.attributes[k],nullptr,time,layer);
        if (!served) {
            out->Add(object.oracleFrozenKeys[k],value,!value.IsEmpty());
            continue;
        }
        RigExecSampledInput fresh;
        fresh.path = object.oracleFrozenKeys[k];
        fresh.value = value;
        fresh.hasValue = !value.IsEmpty();
        _VerifyServedSample(out->values.back(), fresh);
    }
    gatherPoints(object.targetPoints, _frozenWeightTokens->targetPointsKey);
    gatherPoints(object.samplePoints, _frozenWeightTokens->samplePointsKey);
    gatherPoints(object.curvePoints, _frozenWeightTokens->curvePointsKey);
    if (!object.combineTargetPoints.empty()) {
        size_t count = 0;
        for (const UsdAttribute &a : object.combineTargetPoints) {
            VtVec3fArray value;
            if (_FrozenReadWeightArray(refreshed, a, time, &value, layer)) {
                count += value.size();
            }
        }
        // The worker casts back to size_t; mesh point counts never approach
        // int range, and int folds exactly in every digest (uint64_t would
        // not -- _HashVtValue has no uint64 arm).
        out->Add(_FrozenWeightArrayKey(
                     object.path, _frozenWeightTokens->combineTargetCountKey),
                 VtValue(static_cast<int>(count)), /*hasValue=*/true);
    }
}

// Samples one wire revision's side inputs exactly as the wire assembly arm
// reads them (moverGraph.cpp): rest points / order / knots at Default,
// dropoff at time, bind coordinates resolved-first, driver weight arrays
// resolved-first, and the posed driver curve (only when no driver
// transforms bind the table path). All under synthetic mover keys. The
// worker assembles from the revision's leaves (revisionLeaves); these
// samples are the digest's material for the same reads.
void
_SampleMoverToken(const UsdPrim &moverPrim, const SdfPath &moverPath,
                  const char *name, UsdTimeCode time, RigExecFrameInputs *out);

void
_SampleWireInputs(const RigExecBakedProgramImpl::GeomRevision &revision,
                  const RigExecResolvedInputs *refreshed,
                  const UsdStageRefPtr &stage, UsdTimeCode time,
                  RigExecFrameInputs *out)
{
    const UsdPrim &moverPrim = revision.moverPrim;
    const RigExecRevisionBinding &binding = revision.binding;
    const SdfPath &moverPath = revision.moverPath;
    if (moverPrim && !binding.driverCurvePoints.IsEmpty() && stage) {
        VtVec3fArray rest;
        bool has = false;
        if (const UsdAttribute a =
                stage->GetAttributeAtPath(binding.driverCurvePoints)) {
            has = a.Get(&rest, UsdTimeCode::Default());
        }
        out->Add(_FrozenWireInputKey(moverPath, "restPoints"),
                 VtValue(has ? rest : VtVec3fArray()), has);
    }
    if (moverPrim && !binding.driverCurveOrder.IsEmpty() && stage) {
        int order = 0;
        bool has = false;
        if (const UsdAttribute a =
                stage->GetAttributeAtPath(binding.driverCurveOrder)) {
            VtIntArray value;
            if (a.Get(&value, UsdTimeCode::Default()) && !value.empty()) {
                order = value[0];
                has = true;
            }
        }
        out->Add(_FrozenWireInputKey(moverPath, "curveOrder"),
                 VtValue(order), has);
    }
    if (moverPrim && !binding.driverCurveKnots.IsEmpty() && stage) {
        VtDoubleArray knots;
        bool has = false;
        if (const UsdAttribute a =
                stage->GetAttributeAtPath(binding.driverCurveKnots)) {
            has = a.Get(&knots, UsdTimeCode::Default());
        }
        out->Add(_FrozenWireInputKey(moverPath, "curveKnots"),
                 VtValue(has ? knots : VtDoubleArray()), has);
    }
    if (const UsdAttribute a =
            moverPrim.GetAttribute(TfToken("inputs:dropoffDistance"))) {
        float dropoff = 0.0f;
        const bool has = a.Get(&dropoff, time);
        // Live assigns even on a failed read (0.0f init), so hasValue is
        // always true here; the value carries the outcome.
        out->Add(_FrozenWireInputKey(moverPath, "dropoffDistance"),
                 VtValue(dropoff), /*hasValue=*/true);
        (void)has;
    }
    if (!binding.bindCoords.IsEmpty()) {
        VtArray<GfVec2f> coords;
        bool has = refreshed && refreshed->Get(binding.bindCoords, &coords);
        if (!has && stage) {
            if (const UsdAttribute a =
                    stage->GetAttributeAtPath(binding.bindCoords)) {
                has = a.Get(&coords, time);
            }
        }
        out->Add(_FrozenWireInputKey(moverPath, "bindCoords"),
                 VtValue(has ? coords : VtArray<GfVec2f>()), has);
    }
    const auto floats = [&](const char *name, const char *role) {
        const UsdAttribute a = moverPrim.GetAttribute(TfToken(name));
        if (!a.IsValid()) {
            return;
        }
        VtFloatArray value;
        bool has = refreshed && refreshed->GetAttribute(a, time, &value);
        if (!has) {
            has = a.Get(&value, time);
        }
        out->Add(_FrozenWireInputKey(moverPath, role),
                 VtValue(has ? value : VtFloatArray()), has);
    };
    floats("inputs:driverWeights", "driverWeights");
    floats("inputs:driverBaseWeights", "driverBaseWeights");
    _SampleMoverToken(moverPrim, moverPath, "rigExec:pointFrame", time, out);
    _SampleMoverToken(moverPrim, moverPath, "rigExec:driverDeltaFrame", time,
                      out);
    if (binding.driverTransformCount == 0 &&
        !binding.driverCurvePoints.IsEmpty()) {
        VtVec3fArray posed;
        bool has = refreshed &&
                   refreshed->Get(binding.driverCurvePoints, &posed);
        if (!has && stage) {
            if (const UsdAttribute a =
                    stage->GetAttributeAtPath(binding.driverCurvePoints)) {
                has = a.Get(&posed, time);
            }
        }
        out->Add(_FrozenWireInputKey(moverPath, "driverCurvePoints"),
                 VtValue(has ? posed : VtVec3fArray()), has);
    }
}

// A structural token the live assembler reads raw and records
// (_RecordedToken in moverGraph.cpp), sampled under its own property path:
// unsampled, the replay would take the fallback where live took the
// authored value. Only an authored token is sampled. An unauthored one
// reads its schema fallback live, which is the replay's fallback too, and
// every matrix or wire mover would otherwise add a sample to every cached
// frame.
void
_SampleMoverToken(const UsdPrim &moverPrim, const SdfPath &moverPath,
                  const char *name, UsdTimeCode time, RigExecFrameInputs *out)
{
    if (!moverPrim) {
        return;
    }
    const UsdAttribute a = moverPrim.GetAttribute(TfToken(name));
    if (!a || !a.HasAuthoredValue()) {
        return;
    }
    TfToken value;
    const bool has = a.Get(&value, time);
    out->Add(moverPath.AppendProperty(TfToken(name)), VtValue(value), has);
}

void
_SampleAttribute(const SdfPath &key, const UsdAttribute &attribute,
                 UsdTimeCode time, RigExecFrameInputs *out)
{
    if (!attribute.IsValid() || key.IsEmpty()) {
        return;
    }
    VtValue value;
    const bool hasValue = attribute.Get(&value, time);
    const bool blocked = attribute.GetResolveInfo(time).ValueIsBlocked();
    out->Add(key, value, hasValue, false, blocked);
}

// \p B's sampler memo for the state \p time samples under (see
// RigExecFrozenSamplerMemo). Past a change of that state the transport
// rows are forgotten and the held reads are taken again: each provider and
// weight-oracle leaf, source-backed binding, chain or derived base and
// blend channel read whose read cannot move with the time, asked as the
// live samplers ask it (RigExecRevisionLeafHops, or the query's own
// variance), is read once here, at \p time and with no override or
// upstream value, and becomes a table entry with its level-1 digest.
// Owning thread only.
RigExecFrozenSamplerMemo &
_FrozenSamplerMemo(const RigExecRigEvaluator &evaluator,
                   const RigExecBakedProgramImpl &B, UsdTimeCode time)
{
    using Memo = RigExecFrozenSamplerMemo;
    // One memo per Default-ness: a switch between them keeps the other's.
    std::shared_ptr<Memo> &slot =
        B.frozenSamplerMemo[time.IsDefault() ? 1 : 0];
    if (!slot) {
        slot = std::make_shared<Memo>();
    }
    Memo &memo = *slot;
    memo.verify = B.verifyFrozenStatic;
    const uint64_t serial = evaluator.GetStageEditSerial();
    // Sized as the program's tables, so a reshaped program is read again
    // rather than indexed past the memo's end.
    const bool shaped =
        memo.providerEntries.size() == B.providerFrozenKeys.size() &&
        memo.oracleEntries.size() == B.weightObjects.size() &&
        memo.revisionRows.size() == B.revisionIndex.size() &&
        memo.derivedRows.size() == B.derivedIndex.size() &&
        memo.sourceDouble.size() == B.leaves.Of<double>().value.size() &&
        memo.sourceVec3d.size() == B.leaves.Of<GfVec3d>().value.size() &&
        memo.chainBase.size() == B.chains.size();
    if (memo.built && shaped && memo.stamp == B.programStamp &&
        memo.serial == serial && memo.atDefault == time.IsDefault()) {
        return memo;
    }
    memo.built = true;
    memo.stamp = B.programStamp;
    memo.serial = serial;
    memo.atDefault = time.IsDefault();
    memo.revisionRows.assign(B.revisionIndex.size(), Memo::Row());
    memo.derivedRows.assign(B.derivedIndex.size(), Memo::Row());
    memo.layoutRows.assign(B.revisionIndex.size() + B.derivedIndex.size(),
                           Memo::Row());
    auto table = std::make_shared<RigExecFrozenStaticSamples>();
    const auto add = [&table](RigExecSampledInput sample) {
        const int32_t entry = int32_t(table->samples.size());
        sample.staticSample = entry;
        table->level1.push_back(RigExecSampleDigest(sample));
        table->digestible.push_back(RigExecSampleDigestible(sample) ? 1 : 0);
        table->samples.push_back(std::move(sample));
        return entry;
    };
    RigExecFrameInputs read;
    std::vector<SdfPath> hops;
    const RigExecBakedPathLeaves &providers = B.providerLeaves;
    memo.providerEntries.assign(B.providerFrozenKeys.size(), Memo::kPerFrame);
    for (size_t k = 0; k < B.providerFrozenKeys.size(); ++k) {
        const UsdAttribute &attribute = providers.attributes[k];
        if (!attribute.IsValid() || B.providerFrozenKeys[k].IsEmpty()) {
            memo.providerEntries[k] = Memo::kNoSample;
            continue;
        }
        bool varying = false;
        RigExecRevisionLeafHops(providers.decl.keys[k], attribute, &hops,
                                &varying);
        if (varying) {
            continue;
        }
        read.values.clear();
        _SampleAttribute(B.providerFrozenKeys[k], attribute, time, &read);
        memo.providerEntries[k] = add(std::move(read.values.back()));
    }
    memo.oracleEntries.assign(B.weightObjects.size(), {});
    memo.oracleHops.assign(B.weightObjects.size(), {});
    for (size_t o = 0; o < B.weightObjects.size(); ++o) {
        const RigExecBakedProgramImpl::WeightObject &object =
            B.weightObjects[o];
        const RigExecBakedPathLeaves &leaves = object.oracleLeaves;
        const size_t n = leaves.decl.keys.size();
        memo.oracleEntries[o].assign(n, Memo::kPerFrame);
        memo.oracleHops[o].assign(n, std::vector<SdfPath>());
        for (size_t k = 0; k < n; ++k) {
            const RigExecRevisionLeafKey &key = leaves.decl.keys[k];
            if (key.path.IsEmpty()) {
                continue;
            }
            bool varying = false;
            RigExecRevisionLeafHops(key, leaves.attributes[k],
                                    &memo.oracleHops[o][k], &varying);
            if (varying) {
                continue;
            }
            const VtValue value = RigExecSampleRevisionLeaf(
                key, leaves.attributes[k], nullptr, time, nullptr);
            read.values.clear();
            read.Add(object.oracleFrozenKeys[k], value, !value.IsEmpty());
            memo.oracleEntries[o][k] = add(std::move(read.values.back()));
        }
    }
    // Source-backed bindings: _SampleBinding's query read, a single-hop
    // raw read held while the query cannot vary with the time.
    memo.sourceDouble.assign(B.leaves.Of<double>().value.size(),
                             Memo::kPerFrame);
    memo.sourceVec3d.assign(B.leaves.Of<GfVec3d>().value.size(),
                            Memo::kPerFrame);
    _ForEachPatchableInput(B, [&](const auto &input) {
        using T = std::decay_t<decltype(input.constant)>;
        std::vector<int32_t> *entries = nullptr;
        if constexpr (std::is_same_v<T, double>) {
            entries = &memo.sourceDouble;
        } else if constexpr (std::is_same_v<T, GfVec3d>) {
            entries = &memo.sourceVec3d;
        }
        if (!entries || !input.sourceBacked || input.varying ||
            input.walk >= 0 || input.leaf < 0 ||
            size_t(input.leaf) >= entries->size() ||
            !input.query.IsValid() || !input.head.IsValid() ||
            input.query.ValueMightBeTimeVarying()) {
            return;
        }
        read.values.clear();
        _SampleBinding(input, nullptr, time, &read);
        if (read.values.size() == 1) {
            (*entries)[size_t(input.leaf)] =
                add(std::move(read.values.back()));
        }
    });
    // Chain and derived bases: the query alone, held while it cannot vary.
    const auto holdQuery = [&](const SdfPath &key,
                               const UsdAttributeQuery &query) {
        if (!query.IsValid() || query.ValueMightBeTimeVarying()) {
            return Memo::kPerFrame;
        }
        read.values.clear();
        _SampleQuery(key, query, time, &read);
        return read.values.size() == 1 ? add(std::move(read.values.back()))
                                       : Memo::kPerFrame;
    };
    // Blend channel reads: each read's walk, and where nothing on it can
    // move with the time, the read with nothing standing on the walk.
    RigExecResolvedInputs none;
    const auto holdResolved = [&](const UsdAttribute &attribute,
                                  Memo::HeldRead *held,
                                  const auto &sample) {
        if (!attribute.IsValid()) {
            return;
        }
        bool varying = false;
        _ResolvedWalk(attribute, &held->hops, &varying);
        if (varying) {
            return;
        }
        read.values.clear();
        sample(&read);
        if (read.values.size() == 1) {
            held->entry = add(std::move(read.values.back()));
        }
    };
    memo.chainBase.assign(B.chains.size(), Memo::kPerFrame);
    memo.derivedBase.assign(B.chains.size(), {});
    memo.blend.assign(B.chains.size(), {});
    for (size_t c = 0; c < B.chains.size(); ++c) {
        const RigExecBakedProgramImpl::GeomChain &chain = B.chains[c];
        memo.chainBase[c] = holdQuery(chain.target, chain.baseQuery);
        memo.derivedBase[c].assign(chain.derived.size(), Memo::kPerFrame);
        for (size_t d = 0; d < chain.derived.size(); ++d) {
            memo.derivedBase[c][d] = holdQuery(chain.derived[d].target,
                                               chain.derived[d].baseQuery);
        }
        memo.blend[c].assign(chain.revisions.size(), {});
        for (size_t r = 0; r < chain.revisions.size(); ++r) {
            const auto &channels = chain.revisions[r].blendChannels;
            std::vector<Memo::HeldChannel> &held = memo.blend[c][r];
            held.assign(channels.size(), Memo::HeldChannel());
            for (size_t h = 0; h < channels.size(); ++h) {
                const RigExecBakedProgramImpl::GeomBlendChannel &channel =
                    channels[h];
                // A pose-driven weight samples only under an override at
                // its head, which no held read serves.
                if (channel.poseWeight < 0) {
                    holdResolved(
                        channel.weight, &held[h].weight,
                        [&](RigExecFrameInputs *into) {
                            _SampleMoverScalar(channel.weight.GetPath(),
                                               channel.weight, 0.0f, time,
                                               &none, into);
                        });
                }
                held[h].activation.resize(channel.samples.size());
                held[h].points.resize(channel.samples.size());
                for (size_t s = 0; s < channel.samples.size(); ++s) {
                    const RigExecBakedProgramImpl::GeomBlendChannel::Sample
                        &sample = channel.samples[s];
                    holdResolved(
                        sample.activation, &held[h].activation[s],
                        [&](RigExecFrameInputs *into) {
                            _SampleMoverScalar(sample.activation.GetPath(),
                                               sample.activation, 1.0f, time,
                                               &none, into);
                        });
                    holdResolved(
                        sample.points, &held[h].points[s],
                        [&](RigExecFrameInputs *into) {
                            _SampleBlendPoints(
                                _FrozenBlendInputKey(sample.samplePath,
                                                     "points"),
                                sample.points, time, &none, into);
                        });
                }
            }
        }
    }
    memo.samples = std::move(table);
    return memo;
}

// \p memo's held blend channel \p h of revision \p r of chain \p c, or null.
const RigExecFrozenSamplerMemo::HeldChannel *
_HeldBlendChannel(const RigExecFrozenSamplerMemo &memo, size_t c, size_t r,
                  size_t h)
{
    if (c >= memo.blend.size() || r >= memo.blend[c].size() ||
        h >= memo.blend[c][r].size()) {
        return nullptr;
    }
    return &memo.blend[c][r][h];
}

// \p reads[\p s] when \p reads holds it, else null.
const RigExecFrozenSamplerMemo::HeldRead *
_HeldAt(const std::vector<RigExecFrozenSamplerMemo::HeldRead> *reads,
        size_t s)
{
    return reads && s < reads->size() ? &(*reads)[s] : nullptr;
}

// \p memo's entry for chain \p c's base, or with \p derived >= 0 for that
// derived target's base; kPerFrame where it holds none.
int32_t
_HeldBase(const RigExecFrozenSamplerMemo &memo, size_t c, int derived)
{
    if (derived < 0) {
        return c < memo.chainBase.size()
                   ? memo.chainBase[c]
                   : RigExecFrozenSamplerMemo::kPerFrame;
    }
    return c < memo.derivedBase.size() &&
                   size_t(derived) < memo.derivedBase[c].size()
               ? memo.derivedBase[c][size_t(derived)]
               : RigExecFrozenSamplerMemo::kPerFrame;
}

// The provider leaves, in table order: the memo's sample for each whose
// read cannot move with the time, a fresh read for the rest.
void
_SampleProviderLeaves(const RigExecBakedProgramImpl &B,
                      const RigExecFrozenSamplerMemo &memo, UsdTimeCode time,
                      RigExecFrameInputs *sampled)
{
    sampled->values.reserve(sampled->values.size() +
                            B.providerFrozenKeys.size());
    for (size_t k = 0; k < B.providerFrozenKeys.size(); ++k) {
        const int32_t entry = k < memo.providerEntries.size()
                                  ? memo.providerEntries[k]
                                  : RigExecFrozenSamplerMemo::kPerFrame;
        if (entry == RigExecFrozenSamplerMemo::kNoSample) {
            continue;
        }
        if (entry < 0) {
            _SampleAttribute(B.providerFrozenKeys[k],
                             B.providerLeaves.attributes[k], time, sampled);
            continue;
        }
        sampled->values.push_back(memo.samples->samples[size_t(entry)]);
        if (B.verifyFrozenStatic) {
            RigExecFrameInputs fresh;
            _SampleAttribute(B.providerFrozenKeys[k],
                             B.providerLeaves.attributes[k], time, &fresh);
            if (TF_VERIFY(fresh.values.size() == 1,
                          "frozen static provider leaf read nothing")) {
                _VerifyServedSample(sampled->values.back(),
                                    fresh.values.back());
            }
        }
    }
}

// Attaches the digest order recorded for \p sampled's path sequence, from
// the memo's recent orders or recorded now. Under
// RIGEXEC_VERIFY_FROZEN_STATIC, checks the vector digests and classifies as
// it does with neither the order nor the static table's memos.
void
_AttachDigestOrder(const RigExecBakedProgramImpl &B,
                   RigExecFrozenSamplerMemo *memo,
                   RigExecFrameInputs *sampled)
{
    auto &orders = memo->orders;
    size_t found = orders.size();
    for (size_t i = 0; i < orders.size(); ++i) {
        if (RigExecFrameDigestOrderMatches(orders[i].get(),
                                           sampled->values)) {
            found = i;
            break;
        }
    }
    if (found < orders.size()) {
        std::rotate(orders.begin(), orders.begin() + found,
                    orders.begin() + found + 1);
    } else {
        std::rotate(orders.begin(), orders.end() - 1, orders.end());
        orders.front() = RigExecRecordFrameDigestOrder(sampled->values);
    }
    sampled->digestOrder = orders.front();
    if (B.verifyFrozenStatic) {
        RigExecFrameInputs plain = *sampled;
        plain.staticSamples.reset();
        plain.digestOrder.reset();
        TF_VERIFY(RigExecControlStateDigest(*sampled, sampled->overrides) ==
                      RigExecControlStateDigest(plain, plain.overrides),
                  "frozen static digest differs from the plain fold");
        TF_VERIFY(RigExecControlStateDigestible(*sampled) ==
                      RigExecControlStateDigestible(plain),
                  "frozen static digestibility differs from the plain one");
    }
}

// Samples one dense blend sample's target points: the refreshed inputs
// first -- R.GetAttribute follows single authored connections, so an
// override standing on the target is what live reads -- else the stage at
// the job's time.
// Mirrors the dense arm's R.GetAttribute exactly (miss reads as empty);
// a dangling handle samples nothing, which the worker answers as empty.
void
_SampleBlendPoints(const SdfPath &key, const UsdAttribute &attribute,
                   UsdTimeCode time,
                   const RigExecResolvedInputs *refreshed,
                   RigExecFrameInputs *out)
{
    if (!attribute.IsValid() || key.IsEmpty()) {
        return;
    }
    VtVec3fArray value;
    bool hasValue =
        refreshed && refreshed->GetAttribute(attribute, time, &value);
    if (!hasValue) {
        hasValue = attribute.Get(&value, time);
    }
    out->Add(key, VtValue(hasValue ? value : VtVec3fArray()), hasValue);
}

// Samples the stage-frame seeds at the job's time through the program's
// hook (bakedProgram.cpp: the stageFrames of Run, read-only) and carries
// them on the vector. Both sampler routes call this identically: the seeds
// are per-frame stage reads, so no burst memo serves them. Unresolved
// targets carry explicit typed refusal; false denotes malformed transport.
bool
_SampleStageFrameSeeds(const RigExecBakedProgram &program, UsdTimeCode time,
                       RigExecFrameInputs *sampled, std::string *error)
{
    RigExecStageFrameSeeds seeds;
    if (!program.SampleStageFrameSeeds(time, &seeds, error)) {
        return false;
    }
    sampled->stageSeeds = std::move(seeds);
    return true;
}

// Resolve authored connections against explicit source overrides only.
// Produced property versions and walks resolve in their consuming graph body.
VtValue
_SampleRevisionLeaf(const RigExecBakedPathLeaves &leaves, size_t k,
                    const RigExecResolvedInputs *refreshed, UsdTimeCode time,
                    const std::map<SdfPath, VtValue> *layer)
{
    return RigExecSampleRevisionLeaf(leaves.decl.keys[k], leaves.attributes[k],
                                     refreshed, time, layer);
}

// The path leaves of every chain revision the worker assembles from leaves,
// and of every derived target, read by the live prologue's own reads through
// the refreshed inputs at the job's time: the worker assembles from these as
// the live RevisionStatic and Derived steps do from their prologue's.
// Parallel to revisionIndex and derivedIndex; empty for the rest. A skin
// whose layout live holds as fixed leaves its three layout reads empty, as
// the live prologue skips them: the packet carries the handle instead.
// A leaf whose read cannot move with the time is served from \p memo while
// no \p touched path stands on it.
void
_SampleRevisionLeaves(const RigExecBakedProgramImpl &B,
                      const RigExecResolvedInputs *refreshed, UsdTimeCode time,
                      RigExecFrameInputs *sampled,
                      const std::map<SdfPath, VtValue> *layer,
                      RigExecFrozenSamplerMemo *memo,
                      const std::vector<SdfPath> &touched)
{
    using Role = RigExecRevisionLeafRole;
    // Identical raw reads share one answer only within this sample call.
    // Fallback and resolved routes remain independently evaluated.
    using RawKey = std::tuple<SdfPath, RigExecRevisionLeafType,
                              RigExecRevisionLeafTime, bool>;
    std::map<RawKey, VtValue> rawSamples;
    const auto sampleLeaf = [&](const RigExecBakedPathLeaves &leaves, size_t k) {
        const auto &key = leaves.decl.keys[k];
        if (key.flavour != RigExecRevisionLeafFlavour::Raw ||
            key.type == RigExecRevisionLeafType::Dial ||
            !key.fallback.IsEmpty()) {
            return _SampleRevisionLeaf(leaves, k, refreshed, time, layer);
        }
        const RawKey identity(key.path, key.type, key.time,
                              leaves.attributes[k].IsValid());
        const auto found = rawSamples.find(identity);
        if (found != rawSamples.end()) return found->second;
        VtValue value = _SampleRevisionLeaf(leaves, k, refreshed, time, layer);
        rawSamples.emplace(identity, value);
        return value;
    };
    sampled->revisionLeaves.assign(B.revisionIndex.size(),
                                   std::vector<VtValue>());
    sampled->varyingRevisionLeaves.clear();
    for (size_t r = 0; r < B.revisionIndex.size(); ++r) {
        const auto &[chainIndex, revisionIndex] = B.revisionIndex[r];
        const RigExecBakedProgramImpl::GeomRevision &revision =
            B.chains[size_t(chainIndex)].revisions[size_t(revisionIndex)];
        const RigExecBakedPathLeaves &leaves = revision.leaves;
        if (!leaves.decl.assembles) {
            continue;
        }
        if (revision.op == RigExecRevisionOp::External &&
            leaves.decl.externalBegin >= 0) {
            // An external mover's declared inputs reach the worker only as
            // these leaves, so no sample in `values` reads them: the
            // control digests fold each one whose read can vary with the
            // time, asked again where an edit since the live prologue's
            // last sample may have moved the answer.
            const size_t begin = size_t(leaves.decl.externalBegin);
            const size_t end =
                std::min(begin + revision.binding.externalInputs.size(),
                         leaves.decl.keys.size());
            for (size_t k = begin; k < end; ++k) {
                if (RigExecBakedLeafVaryingNow(B, leaves, k)) {
                    sampled->varyingRevisionLeaves.emplace_back(
                        uint32_t(r), uint32_t(k));
                }
            }
        }
        const bool heldLayout = revision.op == RigExecRevisionOp::Skin &&
                                revision.skinTopologyFixed &&
                                revision.layoutFixed;
        std::vector<VtValue> &values = sampled->revisionLeaves[r];
        values.reserve(leaves.decl.keys.size());
        for (size_t k = 0; k < leaves.decl.keys.size(); ++k) {
            const int key = int(k);
            if (heldLayout &&
                (key == leaves.decl.Role(Role::JointIndices) ||
                 key == leaves.decl.Role(Role::JointWeights) ||
                 key == leaves.decl.Role(Role::ElementSize))) {
                values.push_back(VtValue());
                continue;
            }
            values.push_back(_MemoLeafValue(
                &memo->revisionRows[r], leaves, k, touched,
                B.verifyFrozenStatic, [&] { return sampleLeaf(leaves, k); }));
        }
    }
    // Every derived target's, parallel to derivedIndex.
    sampled->derivedLeaves.assign(B.derivedIndex.size(),
                                  std::vector<VtValue>());
    for (size_t d = 0; d < B.derivedIndex.size(); ++d) {
        const auto &[chainIndex, derivedIndex] = B.derivedIndex[d];
        const RigExecBakedPathLeaves &leaves =
            B.chains[size_t(chainIndex)]
                .derived[size_t(derivedIndex)]
                .revision.leaves;
        if (!leaves.decl.assembles) {
            continue;
        }
        std::vector<VtValue> &values = sampled->derivedLeaves[d];
        values.reserve(leaves.decl.keys.size());
        for (size_t k = 0; k < leaves.decl.keys.size(); ++k) {
            values.push_back(_MemoLeafValue(
                &memo->derivedRows[d], leaves, k, touched,
                B.verifyFrozenStatic, [&] { return sampleLeaf(leaves, k); }));
        }
    }
}

// Every SkinTopology operation receives source leaves at the job's time.
// Layout preparation remains in the graph body. Served from \p memo as the
// revision leaves are.
void
_SampleLayoutLeaves(const RigExecBakedProgramImpl &B,
                    const RigExecResolvedInputs *refreshed, UsdTimeCode time,
                    RigExecFrameInputs *sampled,
                    const std::map<SdfPath, VtValue> *layer,
                    RigExecFrozenSamplerMemo *memo,
                    const std::vector<SdfPath> &touched)
{
    const size_t count = B.revisionIndex.size() + B.derivedIndex.size();
    sampled->layoutLeaves.assign(count, std::vector<VtValue>());
    sampled->layoutSourcePaths.assign(count, std::vector<SdfPath>());
    sampled->varyingLayoutRows.clear();
    for (size_t r = 0; r < count; ++r) {
        const RigExecBakedProgramImpl::GeomRevision *revision =
            RigExecBakedLayoutRevision(B, r);
        if (revision->op != RigExecRevisionOp::Skin) {
            continue;
        }
        const RigExecBakedPathLeaves &leaves = revision->layoutLeaves;
        const size_t n = leaves.decl.keys.size();
        std::vector<VtValue> &values = sampled->layoutLeaves[r];
        auto &paths = sampled->layoutSourcePaths[r];
        values.reserve(n);
        paths.reserve(n);
        for (size_t k = 0; k < n; ++k) {
            paths.push_back(leaves.decl.keys[k].path);
            values.push_back(_MemoLeafValue(
                &memo->layoutRows[r], leaves, k, touched,
                B.verifyFrozenStatic, [&] {
                    return RigExecSampleRevisionLeaf(
                        leaves.decl.keys[k], leaves.attributes[k], refreshed,
                        time, layer);
                }));
        }
        // A layout that is not fixed can read another value at another
        // time, and no sample in `values` reads it: the control digests
        // fold the row. Fixedness is asked again where an edit since the
        // live prologue's last sample, routed or not, may have moved it.
        if (!RigExecBakedLayoutFixedNow(B, *revision)) {
            sampled->varyingLayoutRows.push_back(uint32_t(r));
        }
    }
}

// Named auxiliary rows cover sparse Raw Default arrays in the whole-pose
// digest without issuing another source read or replacing ordinary routes.
bool
_AppendSparseLayoutSources(const RigExecBakedProgramImpl &B,
                          RigExecFrameInputs *sampled)
{
    const size_t prefix = B.revisionIndex.size() + B.derivedIndex.size();
    if (sampled->layoutLeaves.size() != prefix ||
        sampled->layoutSourcePaths.size() != prefix) return false;
    for (size_t r = 0; r < prefix; ++r) {
        const auto &revision = *RigExecBakedLayoutRevision(B, r);
        std::vector<VtValue> values;
        std::vector<SdfPath> paths;
        const auto &owners = r < B.revisionIndex.size() ?
            sampled->revisionLeaves : sampled->derivedLeaves;
        const size_t owner = r < B.revisionIndex.size() ? r :
            r - B.revisionIndex.size();
        for (const auto &channel : revision.blendChannels)
            for (const auto &sample : channel.samples) {
                if (sample.blendShape.IsEmpty()) continue;
                const int ids[] = {sample.offsetsLeaf, sample.indicesLeaf};
                const RigExecRevisionLeafType types[] = {
                    RigExecRevisionLeafType::Vec3fArray,
                    RigExecRevisionLeafType::IntArray};
                for (size_t k = 0; k < 2; ++k) {
                    if (ids[k] < 0 || size_t(ids[k]) >= revision.leaves.decl.keys.size() ||
                        owner >= owners.size() || size_t(ids[k]) >= owners[owner].size())
                        return false;
                    const auto &key = revision.leaves.decl.keys[size_t(ids[k])];
                    if (key.flavour != RigExecRevisionLeafFlavour::Raw ||
                        key.time != RigExecRevisionLeafTime::AtDefault ||
                        key.type != types[k] || key.path.IsEmpty()) return false;
                    paths.push_back(key.path);
                    values.push_back(owners[owner][size_t(ids[k])]);
                }
            }
        if (!paths.empty()) {
            sampled->layoutLeaves.push_back(std::move(values));
            sampled->layoutSourcePaths.push_back(std::move(paths));
        }
    }
    return true;
}

void
_SampleQuery(const SdfPath &key, const UsdAttributeQuery &query,
             UsdTimeCode time, RigExecFrameInputs *out)
{
    if (!query.IsValid() || key.IsEmpty()) {
        return;
    }
    VtValue value;
    const bool hasValue = query.Get(&value, time);
    out->Add(key, value, hasValue);
}

// A chain or derived base: \p memo's \p entry while it stands, else the
// query read. The read is raw, so no override or upstream value reaches it.
void
_SampleBase(const RigExecFrozenSamplerMemo &memo, int32_t entry,
            const SdfPath &key, const UsdAttributeQuery &query,
            UsdTimeCode time, RigExecFrameInputs *out)
{
    _SampleThroughMemo(memo, entry, key, nullptr, {}, out,
                       [&](RigExecFrameInputs *into) {
                           _SampleQuery(key, query, time, into);
                       });
}

// A chain base an upstream value stands on, sampled from \p layer as the
// live prologue reads it. False when none stands there.
bool
_SampleUpstreamBase(const SdfPath &target,
                    const std::map<SdfPath, VtValue> *layer,
                    RigExecFrameInputs *out)
{
    if (!layer || target.IsEmpty()) {
        return false;
    }
    const auto it = layer->find(target);
    if (it == layer->end() || !it->second.IsHolding<VtVec3fArray>()) {
        return false;
    }
    out->Add(target, it->second, /*hasValue=*/true);
    return true;
}

// Samples one mover scalar input the packet assembly reads: the resolved
// walk first (an override standing on it), else the stage
// at the job's time, else the fallback. Mirrors moverGraph.cpp's _Float /
// _Enabled / skinningMethod arms exactly (no sample when the attribute does
// not exist; the worker falls back the same way). \p layer defaults to null
// (declared above).
template <class T>
void
_SampleMoverScalar(const SdfPath &key, const UsdAttribute &attribute,
                   T fallback, UsdTimeCode time,
                   const RigExecResolvedInputs *refreshed,
                   RigExecFrameInputs *out,
                   const std::map<SdfPath, VtValue> *layer)
{
    if (!attribute.IsValid() || key.IsEmpty()) {
        return;
    }
    T value = fallback;
    // The job's upstream layer stands in front of the stage, as for the
    // live leaf read of the same attribute (RigExecSampleRevisionLeaf).
    if (!(refreshed && refreshed->GetAttributeOverStageLayer(
                           attribute, time, layer, &value))) {
        attribute.Get(&value, time);
    }
    out->Add(key, VtValue(value), /*hasValue=*/true);
}

// RigExecReadProjectorTarget's reads, sampled where it reads them: the
// settings off the projector, the dials through the refreshed inputs. The
// worker reads the target's leaves (derivedLeaves); these samples are the
// digest's material for the same reads.
void
_SampleProjectorInputs(const RigExecBakedProgramImpl::GeomRevision &revision,
                       const RigExecResolvedInputs *refreshed,
                       const UsdStageRefPtr &stage, UsdTimeCode time,
                       RigExecFrameInputs *out,
                       const std::map<SdfPath, VtValue> *layer = nullptr)
{
    const UsdPrim &prim = revision.moverPrim;
    if (!prim || !stage) {
        return;
    }
    if (revision.op == RigExecRevisionOp::ShaderDials) {
        for (const SdfPath &dial : revision.binding.shaderDials) {
            const UsdAttribute a = stage->GetAttributeAtPath(dial);
            double value = 0.0;
            if (a && a.GetTypeName() == SdfValueTypeNames->Float) {
                float asFloat = 0.0f;
                if (!(refreshed && refreshed->GetAttributeOverStageLayer(
                                       a, time, layer, &asFloat))) {
                    a.Get(&asFloat, time);
                }
                value = double(asFloat);
            } else if (a) {
                if (!(refreshed && refreshed->GetAttributeOverStageLayer(
                                       a, time, layer, &value))) {
                    a.Get(&value, time);
                }
            }
            out->Add(dial, VtValue(value), /*hasValue=*/true);
        }
        return;
    }
    const SdfPath &path = revision.moverPath;
    _SampleMoverScalar(path.AppendProperty(TfToken("rigExec:rayOrigin")),
                       prim.GetAttribute(TfToken("rigExec:rayOrigin")),
                       GfVec3d(0.0, 0.0, 0.0), time, refreshed, out, layer);
    _SampleMoverScalar(path.AppendProperty(TfToken("rigExec:rayDirection")),
                       prim.GetAttribute(TfToken("rigExec:rayDirection")),
                       GfVec3d(0.0, 0.0, 1.0), time, refreshed, out, layer);
    _SampleMoverScalar(path.AppendProperty(TfToken("rigExec:rayUp")),
                       prim.GetAttribute(TfToken("rigExec:rayUp")),
                       GfVec3d(0.0, 1.0, 0.0), time, refreshed, out, layer);
    _SampleMoverScalar(path.AppendProperty(TfToken("rigExec:shaderOffset")),
                       prim.GetAttribute(TfToken("rigExec:shaderOffset")),
                       GfMatrix4d(1.0), time, refreshed, out, layer);
    _SampleMoverToken(prim, path, "rigExec:projectionMode", time, out);
}

// Samples one derived-topology array: the resolved memory first (an
// override standing on the mesh attributes), else the stage at the job's
// time. Mirrors _Array's two arms; a dangling path or a failed read samples
// valueless, which the worker answers as the empty array.
template <class T>
void
_SampleTopologyArray(const SdfPath &path, UsdTimeCode time,
                     const RigExecResolvedInputs *refreshed,
                     const UsdStageRefPtr &stage, RigExecFrameInputs *out)
{
    if (path.IsEmpty()) {
        return;
    }
    VtArray<T> value;
    bool hasValue = refreshed && refreshed->Get(path, &value);
    if (!hasValue && stage) {
        if (const UsdAttribute a = stage->GetAttributeAtPath(path)) {
            hasValue = a.Get(&value, time);
        }
    }
    out->Add(path, VtValue(value), hasValue);
}

void
_SampleIterativeMoverInputs(
    const RigExecBakedProgramImpl::GeomRevision &revision,
    const RigExecResolvedInputs *refreshed, const UsdStageRefPtr &stage,
    UsdTimeCode time, RigExecFrameInputs *out,
    const std::map<SdfPath, VtValue> *layer = nullptr)
{
    if (revision.op != RigExecRevisionOp::DeltaMush &&
        revision.op != RigExecRevisionOp::Wrinkle) {
        return;
    }
    _SampleTopologyArray<GfVec3f>(
        revision.moverPath.AppendProperty(TfToken("inputs:restPoints")),
        UsdTimeCode::Default(), refreshed, stage, out);
    _SampleTopologyArray<int>(revision.binding.topologyCounts, time,
                              refreshed, stage, out);
    _SampleTopologyArray<int>(revision.binding.topologyIndices, time,
                              refreshed, stage, out);
    RigExecMoverParameters parameters;
    _VisitIterativeMoverScalars(revision.op, parameters,
        [&](const char *name, auto fallback, auto &) {
            const TfToken token(name);
            _SampleMoverScalar(revision.moverPath.AppendProperty(token),
                revision.moverPrim.GetAttribute(token), fallback,
                time, refreshed, out, layer);
        });
    if (revision.op == RigExecRevisionOp::DeltaMush) {
        // The extended settings, where their leaves read them: the modes
        // and the edges at Default, the rest at the job's time.
        const auto scalar = [&](const char *name, auto fallback,
                                UsdTimeCode at) {
            const TfToken token(name);
            _SampleMoverScalar(revision.moverPath.AppendProperty(token),
                revision.moverPrim.GetAttribute(token), fallback, at,
                refreshed, out, layer);
        };
        scalar("inputs:smoothing", TfToken("rest"), UsdTimeCode::Default());
        scalar("inputs:frameTransport", TfToken("vertex"),
               UsdTimeCode::Default());
        scalar("inputs:onlySmooth", false, time);
        scalar("inputs:computationToTarget", GfMatrix4d(1.0), time);
        _SampleTopologyArray<float>(revision.moverPath.AppendProperty(TfToken("inputs:smoothWeights")),
                                    time, refreshed, stage, out);
        _SampleTopologyArray<int>(revision.moverPath.AppendProperty(TfToken("inputs:edges")),
                                  UsdTimeCode::Default(), refreshed, stage, out);
    }
    if (revision.op == RigExecRevisionOp::Wrinkle) {
        const TfToken topology("inputs:topology");
        _SampleMoverScalar(revision.moverPath.AppendProperty(topology),
            revision.moverPrim.GetAttribute(topology), TfToken("cloth"),
            UsdTimeCode::Default(), refreshed, out);
        _SampleTopologyArray<int>(
            revision.moverPath.AppendProperty(TfToken("inputs:pinPoints")),
            UsdTimeCode::Default(), refreshed, stage, out);
    }
}

// Whether one attribute reads identically at every time code. USD answers
// conservatively (it may claim variance for a value that happens to be
// constant), so a false here means provably time-invariant -- the same
// classification the currency check uses for folded constants.
bool
_BurstAttributeIsStatic(const UsdAttribute &attribute)
{
    return attribute.IsValid() && !attribute.ValueMightBeTimeVarying();
}

// _BurstAttributeIsStatic over every hop a read through the resolved inputs
// can follow from \p attribute (_ResolvedWalk): the head alone does not
// bound a read that follows its connections.
bool
_BurstWalkIsStatic(const UsdAttribute &attribute)
{
    if (!_BurstAttributeIsStatic(attribute)) {
        return false;
    }
    if (!attribute.HasAuthoredConnections()) {
        return true;
    }
    std::vector<SdfPath> hops;
    bool varying = false;
    _ResolvedWalk(attribute, &hops, &varying);
    return !varying;
}

// Serves one memoized static sample: the stored value, re-marked with the
// route whose map served it. VtValue copies share array payloads, so this
// is refcounts, not reads.
void
_ServeBurstStaticSample(RigExecFrameInputs *out,
                        const RigExecBurstStaticSample &entry, int route)
{
    RigExecSampledInput served = entry.sample;
    served.burstSampleRoute = route;
    out->values.push_back(served);
}

// Records the sample the plain reader just appended as a burst-static of
// \p route. Called only for attributes _BurstAttributeIsStatic accepts
// (_BurstWalkIsStatic for a reader that follows connections), whose value
// AND valuelessness are time-invariant -- and, for the resolved route,
// only with no chains bound, where the refreshed inputs are burst-fixed.
void
_MemoizeBurstStaticSample(
    RigExecFrameInputs *out,
    std::unordered_map<SdfPath, RigExecBurstStaticSample, SdfPath::Hash>
        *memo,
    int route)
{
    RigExecSampledInput &fresh = out->values.back();
    RigExecBurstStaticSample &entry = (*memo)[fresh.path];
    entry.sample = fresh;
    entry.sample.burstSampleRoute = route;
    entry.digestValid = false;
    fresh.burstSampleRoute = route;
}

// The cached read routes: consult the burst map for the route first, and
// on a miss read exactly as the plain reader does, then memoize when the
// attribute is time-invariant. Every early-out and every Add matches the
// plain reader's, so a cached frame's vector is elementwise identical to
// a plain sample's plus the route marks.
void
_SampleAttributeCached(const SdfPath &key, const UsdAttribute &attribute,
                       UsdTimeCode time, RigExecFrameInputs *out,
                       RigExecBurstSampleCache *cache)
{
    if (key.IsEmpty() || !attribute.IsValid()) {
        return;
    }
    const auto found = cache->staticStage.find(key);
    if (found != cache->staticStage.end()) {
        _ServeBurstStaticSample(out, found->second, RigExecBurstRouteStage);
        return;
    }
    _SampleAttribute(key, attribute, time, out);
    if (_BurstAttributeIsStatic(attribute)) {
        _MemoizeBurstStaticSample(out, &cache->staticStage,
                                  RigExecBurstRouteStage);
    }
}

void
_SampleQueryCached(const SdfPath &key, const UsdAttributeQuery &query,
                   UsdTimeCode time, RigExecFrameInputs *out,
                   RigExecBurstSampleCache *cache)
{
    if (key.IsEmpty() || !query.IsValid()) {
        return;
    }
    const auto found = cache->staticStage.find(key);
    if (found != cache->staticStage.end()) {
        _ServeBurstStaticSample(out, found->second, RigExecBurstRouteStage);
        return;
    }
    _SampleQuery(key, query, time, out);
    if (_BurstAttributeIsStatic(query.GetAttribute())) {
        _MemoizeBurstStaticSample(out, &cache->staticStage,
                                  RigExecBurstRouteStage);
    }
}

template <class T>
void
_SampleMoverScalarCached(const SdfPath &key, const UsdAttribute &attribute,
                         T fallback, UsdTimeCode time,
                         const RigExecResolvedInputs *refreshed,
                         RigExecFrameInputs *out,
                         RigExecBurstSampleCache *cache)
{
    if (!attribute.IsValid() || key.IsEmpty()) {
        return;
    }
    // The burst's upstream layer is pinned with it, so a memoized read
    // through it stays the burst's answer.
    const std::map<SdfPath, VtValue> *layer =
        cache->upstreamLayer.empty() ? nullptr : &cache->upstreamLayer;
    // With chains bound, read plain every frame and never memoized.
    if (!cache->bindings.chains.empty()) {
        _SampleMoverScalar(key, attribute, fallback, time, refreshed, out,
                           layer);
        return;
    }
    const auto found = cache->staticResolved.find(key);
    if (found != cache->staticResolved.end()) {
        _ServeBurstStaticSample(out, found->second,
                                RigExecBurstRouteResolved);
        return;
    }
    _SampleMoverScalar(key, attribute, fallback, time, refreshed, out,
                       layer);
    if (_BurstWalkIsStatic(attribute)) {
        _MemoizeBurstStaticSample(out, &cache->staticResolved,
                                  RigExecBurstRouteResolved);
    }
}

void
_SampleBlendPointsCached(const SdfPath &key, const UsdAttribute &attribute,
                         UsdTimeCode time,
                         const RigExecResolvedInputs *refreshed,
                         RigExecFrameInputs *out,
                         RigExecBurstSampleCache *cache)
{
    if (!attribute.IsValid() || key.IsEmpty()) {
        return;
    }
    // With chains bound, read plain every frame and never memoized.
    if (!cache->bindings.chains.empty()) {
        _SampleBlendPoints(key, attribute, time, refreshed, out);
        return;
    }
    const auto found = cache->staticResolved.find(key);
    if (found != cache->staticResolved.end()) {
        _ServeBurstStaticSample(out, found->second,
                                RigExecBurstRouteResolved);
        return;
    }
    _SampleBlendPoints(key, attribute, time, refreshed, out);
    if (_BurstWalkIsStatic(attribute)) {
        _MemoizeBurstStaticSample(out, &cache->staticResolved,
                                  RigExecBurstRouteResolved);
    }
}

// Samples one _Array arm (moverGraph.cpp) under an explicit key: the
// resolved memory first, else the stage at the job's time. A dangling path
// or a failed read samples valueless, which the worker answers as the
// empty array.
template <class T>
void
_SampleMoverPathArray(const SdfPath &key, const SdfPath &path,
                      UsdTimeCode time,
                      const RigExecResolvedInputs *refreshed,
                      const UsdStageRefPtr &stage, RigExecFrameInputs *out)
{
    if (key.IsEmpty() || path.IsEmpty()) {
        return;
    }
    VtArray<T> value;
    bool hasValue = refreshed && refreshed->Get(path, &value);
    if (!hasValue && stage) {
        if (const UsdAttribute a = stage->GetAttributeAtPath(path)) {
            hasValue = a.Get(&value, time);
        }
    }
    out->Add(key, VtValue(value), hasValue);
}

// Samples one _Array arm pinned at Default with no resolved arm (the
// lattice rest cage): a raw stage read at Default.
template <class T>
void
_SampleMoverPathArrayAtDefault(const SdfPath &key, const SdfPath &path,
                               const UsdStageRefPtr &stage,
                               RigExecFrameInputs *out)
{
    if (key.IsEmpty() || path.IsEmpty()) {
        return;
    }
    VtArray<T> value;
    bool hasValue = false;
    if (stage) {
        if (const UsdAttribute a = stage->GetAttributeAtPath(path)) {
            hasValue = a.Get(&value, UsdTimeCode::Default());
        }
    }
    out->Add(key, VtValue(value), hasValue);
}

// Samples the lattice divisions exactly as the arm reads them: raw off the
// mover prim at the evaluated time, no resolved arm. No sample when the
// attribute does not exist; the worker falls back to {0, 0, 0} the same
// way the packet's default init does.
void
_SampleLatticeDivisions(const SdfPath &key, const UsdAttribute &attribute,
                        UsdTimeCode time, RigExecFrameInputs *out)
{
    if (!attribute.IsValid() || key.IsEmpty()) {
        return;
    }
    GfVec3i value(0, 0, 0);
    const bool hasValue = attribute.Get(&value, time);
    out->Add(key, VtValue(value), hasValue);
}

template <class T>
void
_SampleTopologyArrayCached(const SdfPath &path, UsdTimeCode time,
                           const RigExecResolvedInputs *refreshed,
                           const UsdStageRefPtr &stage,
                           RigExecFrameInputs *out,
                           RigExecBurstSampleCache *cache)
{
    if (path.IsEmpty()) {
        return;
    }
    if (!cache->bindings.chains.empty()) {
        _SampleTopologyArray<T>(path, time, refreshed, stage, out);
        return;
    }
    const auto found = cache->staticResolved.find(path);
    if (found != cache->staticResolved.end()) {
        _ServeBurstStaticSample(out, found->second,
                                RigExecBurstRouteResolved);
        return;
    }
    _SampleTopologyArray<T>(path, time, refreshed, stage, out);
    // The plain read already resolved the attribute when it needed the
    // stage; re-resolving it here costs one lookup per path per burst, on
    // the miss only, and classifies the memory-hit case too.
    const UsdAttribute attribute =
        stage ? stage->GetAttributeAtPath(path) : UsdAttribute();
    if (_BurstAttributeIsStatic(attribute)) {
        _MemoizeBurstStaticSample(out, &cache->staticResolved,
                                  RigExecBurstRouteResolved);
    }
}

template <class T>
void
_SampleMoverPathArrayCached(const SdfPath &key, const SdfPath &path,
                            UsdTimeCode time,
                            const RigExecResolvedInputs *refreshed,
                            const UsdStageRefPtr &stage,
                            RigExecFrameInputs *out,
                            RigExecBurstSampleCache *cache)
{
    if (key.IsEmpty() || path.IsEmpty()) {
        return;
    }
    if (!cache->bindings.chains.empty()) {
        _SampleMoverPathArray<T>(key, path, time, refreshed, stage, out);
        return;
    }
    const auto found = cache->staticResolved.find(key);
    if (found != cache->staticResolved.end()) {
        _ServeBurstStaticSample(out, found->second,
                                RigExecBurstRouteResolved);
        return;
    }
    _SampleMoverPathArray<T>(key, path, time, refreshed, stage, out);
    const UsdAttribute attribute =
        stage ? stage->GetAttributeAtPath(path) : UsdAttribute();
    if (_BurstAttributeIsStatic(attribute)) {
        _MemoizeBurstStaticSample(out, &cache->staticResolved,
                                  RigExecBurstRouteResolved);
    }
}

template <class T>
void
_SampleMoverPathArrayAtDefaultCached(const SdfPath &key, const SdfPath &path,
                                     const UsdStageRefPtr &stage,
                                     RigExecFrameInputs *out,
                                     RigExecBurstSampleCache *cache)
{
    if (key.IsEmpty() || path.IsEmpty()) {
        return;
    }
    // Timeless by construction -- a Default read cannot vary per frame --
    // so the first read memoizes unconditionally, whatever USD reports
    // about the attribute's variance.
    const auto found = cache->staticStage.find(key);
    if (found != cache->staticStage.end()) {
        _ServeBurstStaticSample(out, found->second,
                                RigExecBurstRouteStage);
        return;
    }
    _SampleMoverPathArrayAtDefault<T>(key, path, stage, out);
    _MemoizeBurstStaticSample(out, &cache->staticStage,
                              RigExecBurstRouteStage);
}

void
_SampleLatticeDivisionsCached(const SdfPath &key,
                              const UsdAttribute &attribute,
                              UsdTimeCode time, RigExecFrameInputs *out,
                              RigExecBurstSampleCache *cache)
{
    if (key.IsEmpty() || !attribute.IsValid()) {
        return;
    }
    const auto found = cache->staticStage.find(key);
    if (found != cache->staticStage.end()) {
        _ServeBurstStaticSample(out, found->second,
                                RigExecBurstRouteStage);
        return;
    }
    _SampleLatticeDivisions(key, attribute, time, out);
    if (_BurstAttributeIsStatic(attribute)) {
        _MemoizeBurstStaticSample(out, &cache->staticStage,
                                  RigExecBurstRouteStage);
    }
}

} // namespace

namespace frozenDetail {

// Sample keys for the point arrays the packet build gathers. These are
// SYNTHETIC properties under the weight object or mover path -- never the
// attributes' own paths. The same mesh points may be sampled @time by one
// reader and @Default by another (wire rest vs posed curves), and sample
// lookup is first-wins, so sharing the attribute path would serve one
// reader's value to the other. The digest folds these keys like any other
// sample, which is what keeps distinct array states on distinct keys.
SdfPath
_FrozenWeightArrayKey(const SdfPath &objectPath, const TfToken &name)
{
    return objectPath.AppendProperty(name);
}

SdfPath
_FrozenWireInputKey(const SdfPath &moverPath, const char *role)
{
    return moverPath.AppendProperty(
        TfToken(std::string("frozenWire:") + role));
}

// Synthetic key for one blend sample's dense target points, under the
// SAMPLE prim: the target path itself cannot serve, because a sample may
// target a chain's own base points and the two readers disagree -- the
// base loop samples the raw stage, the blend gather reads refreshed-first
// (an override standing on the mesh), and first-wins would serve one
// reader's value to the other. Same reason as the weight and wire keys.
SdfPath
_FrozenBlendInputKey(const SdfPath &samplePath, const char *role)
{
    return samplePath.AppendProperty(
        TfToken(std::string("frozenBlend:") + role));
}

std::vector<SdfPath>
_FrozenOverridePaths(const std::vector<RigExecValueOverride> &overrides)
{
    std::vector<SdfPath> paths;
    paths.reserve(overrides.size());
    for (const RigExecValueOverride &o : overrides) {
        paths.push_back(o.attribute.IsEmpty()
                            ? SdfPath()
                            : o.prim.AppendProperty(o.attribute));
    }
    return paths;
}

// Replicates RigExecBakedProgram::SetOverrides (bakedProgram.cpp:1319)
// against the given epoch tables, writing the job's placement flags. The
// sampler uses it to read overridden bindings the long way; the worker uses
// it for the flags cone dirtiness reads. Returns false when an override is
// unplaceable -- live then runs dynamically, so a frozen job at these
// overrides must not exist.
bool
_FrozenPlaceOverrides(const RigExecBakedProgramImpl &B,
                      const std::vector<RigExecValueOverride> &overrides,
                      const std::vector<SdfPath> &paths,
                      std::vector<char> *flags)
{
    flags->assign(B.overridden.size(), 0);
    if (paths.size() != overrides.size()) {
        return false;
    }
    bool placeable = true;
    for (size_t i = 0; i < overrides.size(); ++i) {
        const RigExecValueOverride &o = overrides[i];
        if (o.attribute.IsEmpty()) {
            placeable = false;
            continue;
        }
        const SdfPath &path = paths[i];
        if (B.folded.count(path)) {
            placeable = false;
            continue;
        }
        const auto found = B.overridableInputs.find(path);
        if (found != B.overridableInputs.end()) {
            for (int index : found->second) {
                (*flags)[size_t(index)] = 1;
            }
            continue;
        }
        if (B.resolvedRoutedPrims.count(o.prim) ||
            RigExecBakedHasExternalInputRoute(B,path)) {
            continue;
        }
        placeable = false;
    }
    return placeable;
}

namespace {
// The extended surface-snap and regular-grid lattice settings, sampled where
// their leaves read them: each through the resolved inputs at the job's
// time, the arrays included. The worker assembles from the leaves; these
// samples are the digest's material for the same reads. \p cache, when
// given, is the burst route's (its upstream layer is its own).
template <class T>
void
_SampleSettingRead(const RigExecBakedProgramImpl::GeomRevision &revision,
                   const char *name, T fallback, UsdTimeCode time,
                   const RigExecResolvedInputs *resolved,
                   RigExecFrameInputs *out,
                   const std::map<SdfPath, VtValue> *layer,
                   RigExecBurstSampleCache *cache)
{
    const TfToken token(name);
    const SdfPath path = revision.moverPath.AppendProperty(token);
    const UsdAttribute attr = revision.moverPrim.GetAttribute(token);
    if (cache) {
        _SampleMoverScalarCached(path, attr, fallback, time, resolved, out,
                                 cache);
    } else {
        _SampleMoverScalar(path, attr, fallback, time, resolved, out, layer);
    }
}

void
_SampleSurfaceSettings(const RigExecBakedProgramImpl::GeomRevision &revision,
                       UsdTimeCode time, const RigExecResolvedInputs *resolved,
                       RigExecFrameInputs *out,
                       const std::map<SdfPath, VtValue> *layer,
                       RigExecBurstSampleCache *cache = nullptr)
{
    const auto read = [&](const char *name, auto fallback) {
        _SampleSettingRead(revision, name, fallback, time, resolved, out,
                           layer, cache);
    };
    read("rigExec:snapMode", TfToken("onSurface"));
    read("rigExec:pointSpace", TfToken("local"));
    read("rigExec:offset", 0.0f);
    read("rigExec:surfaceMatrix", GfMatrix4d(1.0));
    read("rigExec:targetMatrix", GfMatrix4d(1.0));
    read("rigExec:mask", VtFloatArray());
    read("rigExec:triangles", VtIntArray());
}

void
_SampleLatticeSettings(const RigExecBakedProgramImpl::GeomRevision &revision,
                       UsdTimeCode time, const RigExecResolvedInputs *resolved,
                       RigExecFrameInputs *out,
                       const std::map<SdfPath, VtValue> *layer,
                       RigExecBurstSampleCache *cache = nullptr)
{
    const auto read = [&](const char *name, auto fallback) {
        _SampleSettingRead(revision, name, fallback, time, resolved, out,
                           layer, cache);
    };
    read("rigExec:evaluation", TfToken("legacy"));
    read("rigExec:interpolationU", TfToken("bspline"));
    read("rigExec:interpolationV", TfToken("bspline"));
    read("rigExec:interpolationW", TfToken("bspline"));
    read("rigExec:origin", GfVec3f(-0.5f));
    read("rigExec:spacing", GfVec3f(1.0f));
    read("rigExec:strength", 1.0f);
    read("rigExec:pointSpace", TfToken("local"));
    read("rigExec:cageMatrix", GfMatrix4d(1.0));
    read("rigExec:targetMatrix", GfMatrix4d(1.0));
    read("rigExec:mask", VtFloatArray());
}

} // namespace

// Synthetic keys for mover side-input arrays whose read route differs from
// another reader of the same path: the surface driver's points (the base
// loop samples the raw stage where the arm reads refreshed-first) and the
// lattice cage's rest/live pair (one path, two times). Same reason as the
// weight, wire and blend keys.
SdfPath
_FrozenSurfaceInputKey(const SdfPath &moverPath, const char *role)
{
    return moverPath.AppendProperty(
        TfToken(std::string("frozenSurface:") + role));
}

SdfPath
_FrozenLatticeInputKey(const SdfPath &moverPath, const char *role)
{
    return moverPath.AppendProperty(
        TfToken(std::string("frozenLattice:") + role));
}

// Synthetic key for the ribbon bind coordinates: the same binding field
// the wire arm reads under its own synthetic key, and a revision is one
// op, so one key per read keeps the routes from ever aliasing.
SdfPath
_FrozenRibbonInputKey(const SdfPath &moverPath, const char *role)
{
    return moverPath.AppendProperty(
        TfToken(std::string("frozenRibbon:") + role));
}

} // namespace frozenDetail

namespace {

// The part of \p upstream live admits against \p program (the drop rule of
// RigExecRigEvaluator::SetUpstreamInputs), one per path, the last admitted
// entry winning, sorted by path; and the same values as the layer by path.
// As live, each entry is judged before the last-wins rule, so a dropped
// entry never displaces an admitted one given earlier for its path.
void
_AdmitFrameUpstream(const RigExecBakedProgram &program,
                    const std::vector<RigExecUpstreamValue> &upstream,
                    UsdTimeCode time,
                    std::vector<RigExecUpstreamValue> *admitted,
                    std::map<SdfPath, VtValue> *layer,
                    bool *arrayOverVaryingStage = nullptr)
{
    admitted->clear();
    layer->clear();
    if (arrayOverVaryingStage) {
        *arrayOverVaryingStage = false;
    }
    if (upstream.empty()) {
        return;
    }
    const RigExecBakedProgramImpl &B = program.GetStepGraph();
    const std::map<SdfPath, TfType> &listed = program.GetUpstreamAdmissible();
    // The array part, as live judges it (RigExecBakedUpstreamAdmissibleArrays
    // over the same compiled epoch).
    std::map<SdfPath, TfType> arrays;
    if (RigExecUpstreamArrayAdmission() && B.evaluator &&
        std::any_of(upstream.begin(), upstream.end(),
                    [](const RigExecUpstreamValue &v) {
                        return v.value.IsArrayValued();
                    })) {
        for (const RigExecUpstreamArrayRow &row :
             RigExecBakedUpstreamAdmissibleArrays(*B.evaluator)) {
            arrays.emplace_hint(arrays.end(), row.path, row.type);
        }
    }
    std::map<SdfPath, const RigExecUpstreamValue *> byPath;
    for (const RigExecUpstreamValue &value : upstream) {
        if (!value.path.IsPrimPropertyPath()) {
            continue;
        }
        // A listed array over a stage array that can vary is judged per
        // time (condition 4), whatever the answer at \p time.
        if (arrayOverVaryingStage && value.value.IsArrayValued() &&
            arrays.count(value.path)) {
            const UsdAttribute a = B.stage->GetAttributeAtPath(value.path);
            *arrayOverVaryingStage = *arrayOverVaryingStage ||
                                     (a && a.ValueMightBeTimeVarying());
        }
        if (RigExecUpstreamDropReason(B.stage, &listed, value.path,
                                      value.value, time, nullptr, &arrays)
                .empty()) {
            byPath[value.path] = &value;
        }
    }
    for (const auto &[path, value] : byPath) {
        admitted->push_back(*value);
        // An array keys by its fold hash; one handed over without it gets
        // it here, once.
        if (value->value.IsArrayValued() && admitted->back().foldHash == 0) {
            admitted->back().foldHash = RigExecUpstreamFoldHash(value->value);
        }
        layer->emplace_hint(layer->end(), path, value->value);
    }
}

// Per override number, whether a value of \p layer stands on a hop of its
// walk (`overridableInputs` files every hop): live's `upstreamOn`.
std::vector<char>
_UpstreamFlags(const RigExecBakedProgramImpl &B,
               const std::map<SdfPath, VtValue> &layer)
{
    std::vector<char> flags(B.overridden.size(), 0);
    for (const auto &[path, value] : layer) {
        const auto found = B.overridableInputs.find(path);
        if (found == B.overridableInputs.end()) {
            continue;
        }
        for (const int index : found->second) {
            if (index >= 0 && size_t(index) < flags.size()) {
                flags[size_t(index)] = 1;
            }
        }
    }
    return flags;
}

// \p a or \p b, elementwise, sized as \p a.
std::vector<char>
_EitherFlag(const std::vector<char> &a, const std::vector<char> &b)
{
    std::vector<char> flags = a;
    for (size_t i = 0; i < flags.size() && i < b.size(); ++i) {
        flags[i] = flags[i] || b[i];
    }
    return flags;
}

// The constant avar bindings an upstream value stands on and no override
// does (\p readFlags without \p overrideFlags), sampled at their heads: the
// worker patches their constants from these, as it does a dragged one's
// from the override sample keyed by the same path.
void
_SampleUpstreamConstantAvars(const RigExecBakedProgramImpl &B,
                             const RigExecResolvedInputs *refreshed,
                             const std::vector<char> &overrideFlags,
                             const std::vector<char> &readFlags,
                             UsdTimeCode time, RigExecFrameInputs *out,
                             const std::map<SdfPath, VtValue> *layer,
                             const RigExecFrozenSamplerMemo *memo = nullptr)
{
    if (!layer) {
        return;
    }
    for (const RigExecBakedProgramImpl::AvarBinding &binding :
         B.avarConstantBindings) {
        const int o = binding.input.overrideIndex;
        if (_IsOverridden(readFlags, o) && !_IsOverridden(overrideFlags, o)) {
            _SampleFlaggedBinding(binding.input, refreshed, readFlags, time,
                                  out, layer, memo);
        }
    }
}

// The pinned sampler's body. `verifyCurrency` is the per-call
// RigExecChainSampleBindingsStillCurrent check: on for every caller that
// cannot otherwise prove its bindings fresh, off for one that tracks their
// currency from stage notices (see the trusted entry point below).
void _SampleOracleReference(const RigExecRigEvaluator &evaluator,
    const RigExecBakedProgramImpl &B,UsdTimeCode time,
    const std::vector<RigExecValueOverride> &overrides,
    const std::map<SdfPath,VtValue> &upstream,const std::vector<char> &overrideFlags,
    RigExecFrameInputs *out) {
    out->oraclePublications.reset();
    out->oracleWeightInputs.clear();
    if(!evaluator.cpuReference)return;
    RigExecResolvedInputs original;
    for(const auto &value:upstream)original.SetProperty(value.first,value.second);
    _PlaceOverridesIntoResolved(overrides,&original);
    std::vector<SdfPath> roots;std::set<SdfPath> produced,protectedPaths;
    for(const auto &chain:B.propertyChains) {
        roots.push_back(chain.target.GetPrimPath());produced.insert(chain.target);
        for(const auto &revision:chain.revisions)roots.push_back(revision.mover);
        for(uint32_t r:chain.records) {
            const auto &record=B.propertyRecords[r];
            roots.push_back(record.consumer.GetPrimPath());produced.insert(record.consumer);
            for(int slot:record.hopSlots)if(slot>=0 && size_t(slot)<overrideFlags.size() && overrideFlags[size_t(slot)]) {
                protectedPaths.insert(record.consumer);break;
            }
        }
    }
    for(const auto &chain:B.chains) {
        roots.push_back(chain.target.GetPrimPath());
        for(const auto &revision:chain.revisions)roots.push_back(revision.moverPath);
    }
    for(const auto &weight:B.weightObjects)roots.push_back(weight.path);
    auto scene=RigExecDispatchCaptureOracleInputs(B.stage,original,time,0,roots,upstream,
        RigExecOracleCaptureMode::PublicationFacts);
    out->oraclePublications.emplace();
    out->oraclePublications->Begin(0,std::move(scene),B.propertyChains.size(),produced,protectedPaths);
    const auto noPlacement=[](const SdfPath &) -> const GfMatrix4d * {return nullptr;};
    for(const auto &weight:B.weightObjects)out->oracleWeightInputs.emplace(weight.path,
        RigExecDispatchCaptureWeightReference(B.stage,weight.path,original,upstream,time,noPlacement));
}

// The constraint operator arrays, read raw off the prim as the prologue
// reads them (bakedProgram.cpp, Run's constraintArrays), through the
// Build-time handles and keys. A channel whose read cannot move with the
// time (ConstraintArrays::Varies) is read once under the program stamp,
// the evaluator's stage edit serial and the Default-ness of the time, and
// that read's sample is added again while they stand: every stage notice
// advances the serial, so it is what a read would add now. Owning thread
// only (RigExecBakedProgramImpl::frozenArraySamples).
void
_SampleConstraintArrays(const RigExecRigEvaluator &evaluator,
                        const RigExecBakedProgramImpl &B, UsdTimeCode time,
                        RigExecFrameInputs *out)
{
    using Arrays = RigExecBakedProgramImpl::ConstraintArrays;
    using Memo = RigExecBakedProgramImpl::FrozenArraySample;
    const uint64_t serial = evaluator.GetStageEditSerial();
    if (!B.frozenArrayBuilt || B.frozenArrayStamp != B.programStamp ||
        B.frozenArraySerial != serial ||
        B.frozenArrayDefault != time.IsDefault() ||
        B.frozenArraySamples.size() != B.constraintArrays.size()) {
        B.frozenArraySamples.assign(B.constraintArrays.size(),
                                    std::array<Memo, 4>());
        B.frozenArrayBuilt = true;
        B.frozenArrayStamp = B.programStamp;
        B.frozenArraySerial = serial;
        B.frozenArrayDefault = time.IsDefault();
    }
    for (size_t row = 0; row < B.constraintArrays.size(); ++row) {
        const Arrays &arrays = B.constraintArrays[row];
        if (!arrays.prim.IsValid()) {
            continue;
        }
        for (size_t channel = 0; channel < 4; ++channel) {
            if (!arrays.Sampled(channel)) {
                continue;
            }
            Memo &memo = B.frozenArraySamples[row][channel];
            if (memo.variance == Arrays::kFixed) {
                if (memo.present) {
                    out->Add(arrays.keys[channel], memo.value, memo.hasValue,
                             /*viaChain=*/false, memo.blocked);
                }
                continue;
            }
            const size_t before = out->values.size();
            _SampleAttribute(arrays.keys[channel], arrays.attributes[channel],
                             time, out);
            if (memo.variance != Arrays::kVarianceUnknown) {
                continue;
            }
            memo.variance = arrays.Varies(channel) ? Arrays::kVaries
                                                   : Arrays::kFixed;
            memo.present = out->values.size() > before;
            if (memo.variance == Arrays::kFixed && memo.present) {
                const RigExecSampledInput &read = out->values.back();
                memo.value = read.value;
                memo.hasValue = read.hasValue;
                memo.blocked = read.valueBlocked;
            }
        }
    }
}

bool
_SampleWithPinnedChainBindings(
    const RigExecRigEvaluator &evaluator, UsdTimeCode time,
    const std::vector<RigExecValueOverride> &overrides,
    const std::vector<RigExecUpstreamValue> &upstream,
    const RigExecChainSampleBindings &bindings, RigExecFrameInputs *out,
    std::string *error, bool verifyCurrency)
{
    const auto fail = [&error](const std::string &why) {
        if (error) {
            *error = why;
        }
        return false;
    };
    if (!out) {
        return fail("no input vector to sample into");
    }
    // The pinned route trusts nothing: stale bindings fail the sample and
    // the caller rebinds. An empty pin on a chained rig fails here too, so
    // the check cannot be skipped around -- only the bind, which names the
    // rig's actual chains, feeds this route.
    if (verifyCurrency &&
        !RigExecChainSampleBindingsStillCurrent(bindings, evaluator)) {
        return fail(
            "chain bindings no longer name the evaluator's chains; rebind "
            "and retry");
    }
    const RigExecBakedProgram *program = evaluator.GetBakedProgram();
    if (!program) {
        return fail("rig has no valid compiled program for frozen sampling");
    }
    const RigExecBakedProgramImpl &B = program->GetStepGraph();
    // No shape gate on xform slots, native sources, or delta bases: the
    // seeds sample per frame below, through the program's hook.

    // Override placement decides which bindings read the long way. When an
    // override is unplaceable live runs dynamically, so no frozen job at
    // these overrides may exist.
    RigExecFrameInputs sampled;
    sampled.time = time;
    sampled.SetOverrides(overrides);
    std::vector<char> overrideFlags;
    if (!_FrozenPlaceOverrides(B, overrides, sampled.overridePaths,
                               &overrideFlags)) {
        return fail("an override is unplaceable: live runs dynamically at "
                    "these overrides, which no frozen job can reproduce");
    }
    // The refreshed inputs: the job's overrides placed over an empty map,
    // stage reads at the job's time. Bindings read the long way and mover
    // scalars sample through these. A read a chain result or a record can
    // answer is resolved by the worker from head-leaf samples instead, so
    // nothing here holds a chain result.
    RigExecResolvedInputs refreshed;
    _PlaceOverridesIntoResolved(overrides, &refreshed);
    // The job's upstream layer: the admitted values, read in front of the
    // stage wherever live reads its own (RigExecBakedPlaceUpstream). A
    // binding with a value on a hop of its walk is read the long way, as an
    // overridden one is, so `readFlags` stands for both.
    std::map<SdfPath, VtValue> upstreamLayer;
    _AdmitFrameUpstream(*program, upstream, time, &sampled.upstream,
                        &upstreamLayer);
    const std::map<SdfPath, VtValue> *layer =
        upstreamLayer.empty() ? nullptr : &upstreamLayer;
    const std::vector<char> readFlags =
        layer ? _EitherFlag(overrideFlags, _UpstreamFlags(B, upstreamLayer))
              : overrideFlags;

    // Every head leaf the property ops and the reader walks read: the
    // worker runs the head tier and resolves the walks from these.
    _SampleHeadLeaves(evaluator, B, time, &sampled, layer);
    _SampleOracleReference(evaluator,B,time,overrides,upstreamLayer,overrideFlags,&sampled);
    // The reads that cannot move with the time, held while the stage and
    // the program stand; the job's overrides and upstream values reach
    // `touched`, whose leaf reads are taken fresh.
    RigExecFrozenSamplerMemo &memo = _FrozenSamplerMemo(evaluator, B, time);
    sampled.staticSamples = memo.samples;
    const std::vector<SdfPath> touched =
        _TouchedPaths(sampled.overridePaths, layer);

    for (const RigExecBakedProgramImpl::AvarBinding &binding :
         B.avarBindings) {
        _SampleFlaggedBinding(binding.input, &refreshed, readFlags, time,
                              &sampled, layer, &memo);
    }
    for (size_t promoted : B.promotedAvars) {
        if (promoted < B.avarConstantBindings.size()) {
            // Refreshed, not standing: a promoted avar's walk is chainless,
            // so the refreshed walk (job overrides, stage at the job's
            // time) is exactly what live reads, while the standing inputs
            // may hold another frame's overrides.
            _SamplePromotedAvar(B.avarConstantBindings[promoted].input,
                                &refreshed, time, &sampled, layer);
        }
    }
    _SampleUpstreamConstantAvars(B, &refreshed, overrideFlags, readFlags,
                                 time, &sampled, layer, &memo);
    for (const RigExecBakedProgramImpl::Ladder &ladder : B.ladders) {
        _VisitLadderInputs(ladder, [&](const auto &input) {
            _SampleFlaggedBinding(input, &refreshed, readFlags, time,
                                  &sampled, layer, &memo);
        });
    }
    for(const auto &operation:B.autoClavicles) {
        _VisitAutoClavicleInputs(operation,[&](const auto &input) {
            _SampleFlaggedBinding(input,&refreshed,readFlags,time,&sampled,layer,&memo);
        });
    }
    for (const RigExecBakedProgramImpl::SpaceSwitch &spaceSwitch :
         B.spaceSwitches) {
        _VisitSpaceSwitchInputs(spaceSwitch, [&](const auto &input) {
            _SampleFlaggedBinding(input, &refreshed, readFlags, time,
                                  &sampled, layer, &memo);
        });
    }
    for (const RigExecBakedProgramImpl::PoseInterpolator &interp :
         B.poseInterpolators) {
        _VisitInterpolatorInputs(interp, [&](const auto &input) {
            _SampleFlaggedBinding(input, &refreshed, readFlags, time,
                                  &sampled, layer, &memo);
        });
    }
    for (const RigExecBakedProgramImpl::Solver &solver : B.solvers) {
        _SampleSolverBindings(solver, &refreshed, readFlags, time, &sampled,
                              layer, &memo);
    }
    for (const RigExecBakedProgramImpl::Constraint &constraint :
         B.constraints) {
        _SampleConstraintBindings(constraint, &refreshed, readFlags, time,
                                  &sampled, layer, &memo);
    }
    for (size_t o = 0; o < B.weightObjects.size(); ++o) {
        const RigExecBakedProgramImpl::WeightObject &object =
            B.weightObjects[o];
        _SampleWeightBindings(object, &refreshed, readFlags, time, &sampled,
                              layer, &memo);
        _SampleWeightArrays(object, &refreshed, time, &sampled, layer, memo,
                            o, touched, B.verifyFrozenStatic);
    }
    // Ribbon driver points: read straight off the stage by the prologue,
    // honouring neither connections nor the resolved inputs, so sampled the
    // same way.
    for (const RigExecBakedProgramImpl::Solver &solver : B.solvers) {
        if (solver.ribbonPointsVarying) {
            _SampleQuery(solver.ribbonPointsPath, solver.ribbonPointsQuery,
                         time, &sampled);
        }
    }
    // Blend channels outside the binding table, refreshed-first like the
    // mover scalars: an override standing on a weight, activation or
    // target array is what R.GetAttribute answers live. The
    // fallbacks are the gather's inits (bakedGeometry.cpp AssembleRevision):
    // a weight the read misses is 0, an activation 1, points empty. Dense
    // points ride a synthetic key under the sample prim, never the target
    // path (see _FrozenBlendInputKey).
    // A pose-driven weight samples only when the refreshed inputs hold its
    // path -- an override standing on it. Otherwise the worker takes the
    // slot from its own pose run, or a chain's result from its own head
    // tier, and no sample shadows it.
    // A read the memo holds is served while no override or upstream value
    // stands on its walk.
    for (size_t c = 0; c < B.chains.size(); ++c) {
        const RigExecBakedProgramImpl::GeomChain &chain = B.chains[c];
        for (size_t r = 0; r < chain.revisions.size(); ++r) {
            const RigExecBakedProgramImpl::GeomRevision &revision =
                chain.revisions[r];
            for (size_t h = 0; h < revision.blendChannels.size(); ++h) {
                const RigExecBakedProgramImpl::GeomBlendChannel &channel =
                    revision.blendChannels[h];
                const RigExecFrozenSamplerMemo::HeldChannel *held =
                    _HeldBlendChannel(memo, c, r, h);
                if (channel.weight.IsValid() &&
                    (channel.poseWeight < 0 ||
                     refreshed.Find(channel.weightPath))) {
                    const SdfPath key = channel.weight.GetPath();
                    _SampleHeldRead(
                        memo, held ? &held->weight : nullptr, key, touched,
                        &sampled, [&](RigExecFrameInputs *into) {
                            _SampleMoverScalar(key, channel.weight, 0.0f,
                                               time, &refreshed, into, layer);
                        });
                }
                for (size_t s = 0; s < channel.samples.size(); ++s) {
                    const RigExecBakedProgramImpl::GeomBlendChannel::Sample
                        &sample = channel.samples[s];
                    const SdfPath activationKey = sample.activation.GetPath();
                    _SampleHeldRead(
                        memo, _HeldAt(held ? &held->activation : nullptr, s),
                        activationKey, touched, &sampled,
                        [&](RigExecFrameInputs *into) {
                            _SampleMoverScalar(activationKey,
                                               sample.activation, 1.0f, time,
                                               &refreshed, into, layer);
                        });
                    const SdfPath pointsKey =
                        _FrozenBlendInputKey(sample.samplePath, "points");
                    _SampleHeldRead(
                        memo, _HeldAt(held ? &held->points : nullptr, s),
                        pointsKey, touched, &sampled,
                        [&](RigExecFrameInputs *into) {
                            _SampleBlendPoints(pointsKey, sample.points, time,
                                               &refreshed, into);
                        });
                }
            }
        }
    }
    // Constraint operator arrays: read raw off the prim.
    _SampleConstraintArrays(evaluator, B, time, &sampled);
    // Chain base points: the prologue reads every chain base off the stage
    // (or the upstream layer) per frame, and a worker cannot -- so the UI
    // thread samples these too, a base the memo holds from it.
    for (size_t c = 0; c < B.chains.size(); ++c) {
        const RigExecBakedProgramImpl::GeomChain &chain = B.chains[c];
        if (!_SampleUpstreamBase(chain.target, layer, &sampled)) {
            // A lost source remains a declared row. The worker distinguishes
            // unavailable data from a required row missing in transport.
            if (!chain.baseQuery.IsValid())
                sampled.Add(chain.target, VtValue(), /*hasValue=*/false);
            else
                _SampleBase(memo, _HeldBase(memo, c, -1), chain.target,
                            chain.baseQuery, time, &sampled);
        }
        for (size_t d = 0; d < chain.derived.size(); ++d) {
            const RigExecBakedProgramImpl::GeomChain::Derived &derived =
                chain.derived[d];
            if (!derived.matrixTarget && !derived.baseQuery.IsValid())
                sampled.Add(derived.target, VtValue(), /*hasValue=*/false);
            else
                _SampleBase(memo, _HeldBase(memo, c, int(d)), derived.target,
                            derived.baseQuery, time, &sampled);
        }
    }
    // Mover scalars the packet assembly reads per frame (enabled,
    // defaultWeight, skinningMethod), keyed by mover path, plus the derived
    // topology arrays (counts, indices, extent widths), keyed by binding
    // path. The worker assembles every revision and every derived target
    // from leaves (revisionLeaves, derivedLeaves, and a skin's layout from
    // layoutLeaves); the samples of the same reads are
    // what the digest folds for them. RevisionStatic still reads each
    // revision's defaultWeight sample.
    for (const auto &[chainIndex, revisionIndex] : B.revisionIndex) {
        const RigExecBakedProgramImpl::GeomRevision &revision =
            B.chains[size_t(chainIndex)].revisions[size_t(revisionIndex)];
        const UsdPrim &moverPrim = revision.moverPrim;
        _SampleMoverScalar(
            revision.moverPath.AppendProperty(TfToken("inputs:enabled")),
            moverPrim.GetAttribute(TfToken("inputs:enabled")), true, time,
            &refreshed, &sampled, layer);
        _SampleMoverScalar(
            revision.moverPath.AppendProperty(TfToken("inputs:defaultWeight")),
            moverPrim.GetAttribute(TfToken("inputs:defaultWeight")), 1.0f,
            time, &refreshed, &sampled, layer);
        _SampleMoverScalar(
            revision.moverPath.AppendProperty(TfToken("rigExec:skinningMethod")),
            moverPrim.GetAttribute(TfToken("rigExec:skinningMethod")),
            TfToken("classicLinear"), time, &refreshed, &sampled, layer);
        if (revision.op == RigExecRevisionOp::Wire) {
            _SampleWireInputs(revision, &refreshed, B.stage, time, &sampled);
        }
        if (revision.op == RigExecRevisionOp::Matrix) {
            _SampleMoverToken(moverPrim, revision.moverPath,
                              "rigExec:weightBlend", time, &sampled);
        }
        _SampleIterativeMoverInputs(revision, &refreshed, B.stage, time,
                                    &sampled, layer);
        if (revision.op == RigExecRevisionOp::BlendShape && moverPrim) {
            // rigExec:deltaSpace, exactly as the arm's _Token reads it: a
            // raw Default read with no resolved arm, "target" when absent.
            // Read once here; the value both samples and decides whether
            // the surface-frame topology below is read at all, as live.
            TfToken deltaSpace("target");
            const SdfPath deltaSpacePath = revision.moverPath.AppendProperty(
                TfToken("rigExec:deltaSpace"));
            if (const UsdAttribute deltaSpaceAttr =
                    moverPrim.GetAttribute(TfToken("rigExec:deltaSpace"))) {
                TfToken read;
                if (deltaSpaceAttr.Get(&read, UsdTimeCode::Default())) {
                    deltaSpace = read;
                }
                sampled.Add(deltaSpacePath, VtValue(deltaSpace),
                            /*hasValue=*/true);
            }
            if (deltaSpace == "surfaceFrame") {
                _SampleTopologyArray<int>(revision.binding.topologyCounts,
                                          time, &refreshed, B.stage, &sampled);
                _SampleTopologyArray<int>(revision.binding.topologyIndices,
                                          time, &refreshed, B.stage, &sampled);
            }
        }
        if (moverPrim &&
            (revision.op == RigExecRevisionOp::Smooth ||
             revision.op == RigExecRevisionOp::SurfaceProject)) {
            // The arms' _Array reads at the evaluated time, resolved-first;
            // binding paths are safe keys because every reader of a
            // topology path samples this same route.
            _SampleTopologyArray<int>(revision.binding.topologyCounts, time,
                                      &refreshed, B.stage, &sampled);
            _SampleTopologyArray<int>(revision.binding.topologyIndices, time,
                                      &refreshed, B.stage, &sampled);
        }
        if (revision.op == RigExecRevisionOp::SurfaceProject && moverPrim) {
            _SampleSurfaceSettings(revision, time, &refreshed, &sampled, layer);
            _SampleMoverPathArray<GfVec3f>(
                _FrozenSurfaceInputKey(revision.moverPath, "surfacePoints"),
                revision.binding.surfacePoints, time, &refreshed, B.stage,
                &sampled);
        }
        if (revision.op == RigExecRevisionOp::Lattice && moverPrim) {
            _SampleLatticeSettings(revision, time, &refreshed, &sampled, layer);
            // The rest/live cage pair, under distinct synthetic keys: one
            // path, two times (Default raw, evaluated resolved-first).
            _SampleMoverPathArrayAtDefault<GfVec3f>(
                _FrozenLatticeInputKey(revision.moverPath, "cageRest"),
                revision.binding.cagePoints, B.stage, &sampled);
            _SampleMoverPathArray<GfVec3f>(
                _FrozenLatticeInputKey(revision.moverPath, "cageLive"),
                revision.binding.cagePoints, time, &refreshed, B.stage,
                &sampled);
            _SampleLatticeDivisions(
                revision.moverPath.AppendProperty(
                    TfToken("rigExec:divisions")),
                moverPrim.GetAttribute(TfToken("rigExec:divisions")), time,
                &sampled);
        }
        if (revision.op == RigExecRevisionOp::Ribbon && moverPrim) {
            // The bind coordinates at the evaluated time, resolved-first.
            // The driver's frames need no sampling: the solver's Solve
            // step runs on the worker from already-sampled inputs, and
            // the assembly reads the worker's own aggregate.
            _SampleMoverPathArray<GfVec2f>(
                _FrozenRibbonInputKey(revision.moverPath, "bindCoords"),
                revision.binding.bindCoords, time, &refreshed, B.stage,
                &sampled);
        }
    }
    for (const auto &[chainIndex, derivedIndex] : B.derivedIndex) {
        const RigExecBakedProgramImpl::GeomChain::Derived &derived =
            B.chains[size_t(chainIndex)].derived[size_t(derivedIndex)];
        const RigExecBakedProgramImpl::GeomRevision &revision =
            derived.revision;
        _SampleTopologyArray<int>(revision.binding.topologyCounts, time,
                                  &refreshed, B.stage, &sampled);
        _SampleTopologyArray<int>(revision.binding.topologyIndices, time,
                                  &refreshed, B.stage, &sampled);
        if (revision.op == RigExecRevisionOp::RecomputeExtent) {
            _SampleTopologyArray<float>(revision.binding.widths, time,
                                        &refreshed, B.stage, &sampled);
        }
        if (RigExecIsDerivedMatrixOp(revision.op)) {
            _SampleProjectorInputs(revision, &refreshed, B.stage, time,
                                   &sampled, layer);
        }
    }
    // The skin layouts the worker's SkinTopology ops build from, then every
    // revision's assembly reads.
    _SampleLayoutLeaves(B, &refreshed, time, &sampled, layer, &memo,
                        touched);
    _SampleProviderLeaves(B, memo, time, &sampled);
    _SampleRevisionLeaves(B, &refreshed, time, &sampled, layer, &memo,
                          touched);
    if (!_AppendSparseLayoutSources(B, &sampled))
        return fail("sparse layout source census differs from compiled leaves");
    // Stage-frame seeds at the job's time, through the program's hook. A
    // decline names an unresolvable target, at which live gives the
    // generation back -- so the frame has no frozen job.
    {
        std::string seedsError;
        if (!_SampleStageFrameSeeds(*program, time, &sampled, &seedsError)) {
            return fail(seedsError);
        }
    }
    // Interactive overrides never reach the stage, so they are sampled from
    // the caller's list rather than read. Keyed by the property they stand
    // on; a computation override -- which names no attribute -- is keyed by
    // a synthetic property under its prim, so two computations on one prim
    // still hash apart.
    for (const RigExecValueOverride &o : overrides) {
        SdfPath key = o.prim;
        if (!o.attribute.IsEmpty()) {
            key = key.AppendProperty(o.attribute);
        } else if (!o.computation.IsEmpty()) {
            key = key.AppendProperty(TfToken(
                "rigExec:override:" + o.computation.GetString()));
        }
        if (!key.IsEmpty()) {
            sampled.Add(key, o.value, /*hasValue=*/true);
        }
    }
    _AttachDigestOrder(B, &memo, &sampled);

    *out = std::move(sampled);
    return true;
}
} // namespace

bool
RigExecSampleFrameInputsWithChainBindings(
    const RigExecRigEvaluator &evaluator, UsdTimeCode time,
    const std::vector<RigExecValueOverride> &overrides,
    const RigExecChainSampleBindings &bindings, RigExecFrameInputs *out,
    std::string *error)
{
    return _SampleWithPinnedChainBindings(evaluator, time, overrides, {},
                                          bindings, out, error,
                                          /* verifyCurrency = */ true);
}

bool
RigExecSampleFrameInputsWithChainBindings(
    const RigExecRigEvaluator &evaluator, UsdTimeCode time,
    const std::vector<RigExecValueOverride> &overrides,
    const std::vector<RigExecUpstreamValue> &upstream,
    const RigExecChainSampleBindings &bindings, RigExecFrameInputs *out,
    std::string *error)
{
    return _SampleWithPinnedChainBindings(evaluator, time, overrides,
                                          upstream, bindings, out, error,
                                          /* verifyCurrency = */ true);
}

bool
RigExecSampleFrameInputsWithTrustedChainBindings(
    const RigExecRigEvaluator &evaluator, UsdTimeCode time,
    const std::vector<RigExecValueOverride> &overrides,
    const RigExecChainSampleBindings &bindings, RigExecFrameInputs *out,
    std::string *error)
{
    return _SampleWithPinnedChainBindings(evaluator, time, overrides, {},
                                          bindings, out, error,
                                          /* verifyCurrency = */ false);
}

bool
RigExecSampleFrameInputsWithTrustedChainBindings(
    const RigExecRigEvaluator &evaluator, UsdTimeCode time,
    const std::vector<RigExecValueOverride> &overrides,
    const std::vector<RigExecUpstreamValue> &upstream,
    const RigExecChainSampleBindings &bindings, RigExecFrameInputs *out,
    std::string *error)
{
    return _SampleWithPinnedChainBindings(evaluator, time, overrides,
                                          upstream, bindings, out, error,
                                          /* verifyCurrency = */ false);
}

void
RigExecBurstSampleCache::Clear()
{
    program = nullptr;
    epochDigest = 0;
    bindings = RigExecChainSampleBindings();
    overrides.clear();
    overrideFlags.clear();
    upstream.clear();
    upstreamAdmitted.clear();
    upstreamLayer.clear();
    upstreamFlags.clear();
    readFlags.clear();
    placeable = false;
    usable = false;
    ladderSites.clear();
    spaceSwitchSites.clear();
    solverSites.clear();
    constraintSites.clear();
    weightSites.clear();
    interpolatorSites.clear();
    staticStage.clear();
    staticResolved.clear();
    sortedOrder.clear();
    orderValuesSize = 0;
    orderFirstPath = SdfPath();
    orderLastPath = SdfPath();
}

bool
RigExecBuildBurstSampleCache(
    const RigExecBakedProgram &program,
    const RigExecChainSampleBindings &bindings,
    const std::vector<RigExecValueOverride> &overrides, uint64_t epochDigest,
    RigExecBurstSampleCache *cache, std::string *error)
{
    return RigExecBuildBurstSampleCache(program, bindings, overrides, {},
                                        epochDigest, cache, error);
}

bool
RigExecBuildBurstSampleCache(
    const RigExecBakedProgram &program,
    const RigExecChainSampleBindings &bindings,
    const std::vector<RigExecValueOverride> &overrides,
    const std::vector<RigExecUpstreamValue> &upstream, uint64_t epochDigest,
    RigExecBurstSampleCache *cache, std::string *error)
{
    const auto fail = [&error, cache](const std::string &why) {
        if (cache) {
            cache->usable = false;
        }
        if (error) {
            *error = why;
        }
        return false;
    };
    if (!cache) {
        return fail("no burst cache to build into");
    }
    cache->Clear();
    // The decline checks mirror the sampler's below, with its messages: a
    // rig the sampler would decline per frame fails the build once, and
    // the burst skips every frame up front instead of declining each.
    const RigExecBakedProgramImpl &B = program.GetStepGraph();
    // No shape gate on xform slots, native sources, or delta bases: the
    // seeds sample per frame through the program's hook, as in the plain
    // sampler, so no frame of the burst is unshaped for them.
    if (!_FrozenPlaceOverrides(B, overrides, _FrozenOverridePaths(overrides),
                               &cache->overrideFlags)) {
        return fail("an override is unplaceable: live runs dynamically at "
                    "these overrides, which no frozen job can reproduce");
    }
    cache->program = &program;
    cache->epochDigest = epochDigest;
    cache->bindings = bindings;
    cache->overrides = overrides;
    cache->upstream = upstream;
    // A burst judges admission once, so an array over a stage array that
    // can vary (condition 4 is per time) samples on the plain route.
    bool arrayOverVaryingStage = false;
    _AdmitFrameUpstream(program, upstream, UsdTimeCode::EarliestTime(),
                        &cache->upstreamAdmitted, &cache->upstreamLayer,
                        &arrayOverVaryingStage);
    if (arrayOverVaryingStage) {
        return fail("an upstream array stands over a time-varying stage "
                    "array, whose admission is judged per frame");
    }
    cache->upstreamFlags = _UpstreamFlags(B, cache->upstreamLayer);
    cache->readFlags =
        _EitherFlag(cache->overrideFlags, cache->upstreamFlags);
    cache->placeable = true;
    // In table order, so the cached sampler visits in emission order.
    const std::vector<char> &flags = cache->readFlags;
    for (size_t i = 0; i < B.ladders.size(); ++i) {
        if (_BurstLadderNeedsVisit(B.ladders[i], flags)) {
            cache->ladderSites.push_back(i);
        }
    }
    for (size_t i = 0; i < B.spaceSwitches.size(); ++i) {
        if (_BurstSpaceSwitchNeedsVisit(B.spaceSwitches[i], flags)) {
            cache->spaceSwitchSites.push_back(i);
        }
    }
    for (size_t i = 0; i < B.solvers.size(); ++i) {
        if (_BurstSolverNeedsVisit(B.solvers[i], flags)) {
            cache->solverSites.push_back(i);
        }
    }
    for (size_t i = 0; i < B.constraints.size(); ++i) {
        if (_BurstConstraintNeedsVisit(B.constraints[i], flags)) {
            cache->constraintSites.push_back(i);
        }
    }
    for (size_t i = 0; i < B.weightObjects.size(); ++i) {
        if (_BurstWeightNeedsVisit(B.weightObjects[i], flags)) {
            cache->weightSites.push_back(i);
        }
    }
    for (size_t i = 0; i < B.poseInterpolators.size(); ++i) {
        if (_BurstInterpolatorNeedsVisit(B.poseInterpolators[i], flags)) {
            cache->interpolatorSites.push_back(i);
        }
    }
    cache->usable = true;
    return true;
}

bool
RigExecSampleFrameInputsWithBurstCache(
    const RigExecRigEvaluator &evaluator, UsdTimeCode time,
    const std::vector<RigExecValueOverride> &overrides,
    RigExecBurstSampleCache *cache, RigExecFrameInputs *out,
    std::string *error)
{
    return RigExecSampleFrameInputsWithBurstCache(evaluator, time, overrides,
                                                  {}, cache, out, error);
}

bool
RigExecSampleFrameInputsWithBurstCache(
    const RigExecRigEvaluator &evaluator, UsdTimeCode time,
    const std::vector<RigExecValueOverride> &overrides,
    const std::vector<RigExecUpstreamValue> &upstream,
    RigExecBurstSampleCache *cache, RigExecFrameInputs *out,
    std::string *error)
{
    const auto fail = [&error](const std::string &why) {
        if (error) {
            *error = why;
        }
        return false;
    };
    if (!out) {
        return fail("no input vector to sample into");
    }
    // The cache is burst-scoped: a foreign program or a differing override
    // list fails loud rather than sampling through prepared state that no
    // longer describes the call. A burst holds both fixed (the registry
    // captures the same vector for every frame), so production never lands
    // here; the plain sampler stays the answer for anything else.
    if (!cache || !cache->usable) {
        return fail("burst cache is not usable; re-prepare");
    }
    if (evaluator.GetBakedProgram() != cache->program) {
        return fail("burst cache names another program; re-prepare");
    }
    if (overrides != cache->overrides) {
        return fail("burst cache was prepared for other overrides; "
                    "re-prepare");
    }
    if (upstream != cache->upstream) {
        return fail("burst cache was prepared for other upstream values; "
                    "re-prepare");
    }
    const RigExecBakedProgramImpl &B = cache->program->GetStepGraph();
    const RigExecProfiler &sampleProfiler = evaluator.GetProfiler();
    const bool profileSample = sampleProfiler.IsEnabled();
    uint64_t sampleStart = profileSample ? RigExecProfiler::NowUs() : 0;
    const auto samplePhase = [&](const char *name) {
        if (profileSample) {
            const uint64_t end = RigExecProfiler::NowUs();
            sampleProfiler.Record(name, "frozenSample", sampleStart, end);
            sampleStart = end;
        }
    };
    // No currency re-verification (verified at prepare), no decline checks
    // (the build failed on them), no override placement (cached flags).
    // Everything below mirrors the plain sampler above: the same tables,
    // in the same order, through the same routes -- struct tables through
    // the prepared site lists, stage-direct and resolved reads through the
    // static maps -- so the vector is elementwise identical plus the route
    // marks.
    RigExecResolvedInputs refreshed;
    _PlaceOverridesIntoResolved(overrides, &refreshed);

    RigExecFrameInputs sampled;
    sampled.time = time;
    sampled.SetOverrides(overrides);
    sampled.upstream = cache->upstreamAdmitted;
    const std::map<SdfPath, VtValue> *layer =
        cache->upstreamLayer.empty() ? nullptr : &cache->upstreamLayer;
    const std::vector<char> &readFlags = cache->readFlags;

    // As in the plain sampler: the varying leaves every frame, the
    // constant ones by the shared table.
    _SampleHeadLeaves(evaluator, B, time, &sampled, layer);
    _SampleOracleReference(evaluator,B,time,overrides,cache->upstreamLayer,cache->overrideFlags,&sampled);
    // The time-invariant leaf reads, as in the plain sampler.
    RigExecFrozenSamplerMemo &memo = _FrozenSamplerMemo(evaluator, B, time);
    sampled.staticSamples = memo.samples;
    const std::vector<SdfPath> touched =
        _TouchedPaths(sampled.overridePaths, layer);

    samplePhase("Sample.HeadOracle");
    for (const RigExecBakedProgramImpl::AvarBinding &binding :
         B.avarBindings) {
        _SampleFlaggedBinding(binding.input, &refreshed, readFlags, time,
                              &sampled, layer, &memo);
    }
    for (size_t promoted : B.promotedAvars) {
        if (promoted < B.avarConstantBindings.size()) {
            _SamplePromotedAvar(B.avarConstantBindings[promoted].input,
                                &refreshed, time, &sampled, layer);
        }
    }
    _SampleUpstreamConstantAvars(B, &refreshed, cache->overrideFlags,
                                 readFlags, time, &sampled, layer, &memo);
    for (size_t i : cache->ladderSites) {
        _VisitLadderInputs(B.ladders[i], [&](const auto &input) {
            _SampleFlaggedBinding(input, &refreshed, readFlags, time,
                                  &sampled, layer, &memo);
        });
    }
    for(const auto &operation:B.autoClavicles) {
        _VisitAutoClavicleInputs(operation,[&](const auto &input) {
            _SampleFlaggedBinding(input,&refreshed,readFlags,time,&sampled,layer,&memo);
        });
    }
    for (size_t i : cache->spaceSwitchSites) {
        _VisitSpaceSwitchInputs(B.spaceSwitches[i], [&](const auto &input) {
            _SampleFlaggedBinding(input, &refreshed, readFlags, time,
                                  &sampled, layer, &memo);
        });
    }
    for (size_t i : cache->interpolatorSites) {
        _VisitInterpolatorInputs(B.poseInterpolators[i],
                                 [&](const auto &input) {
            _SampleFlaggedBinding(input, &refreshed, readFlags, time,
                                  &sampled, layer, &memo);
        });
    }
    for (size_t i : cache->solverSites) {
        _SampleSolverBindings(B.solvers[i], &refreshed, readFlags, time,
                              &sampled, layer, &memo);
    }
    for (size_t i : cache->constraintSites) {
        _SampleConstraintBindings(B.constraints[i], &refreshed, readFlags,
                                  time, &sampled, layer, &memo);
    }
    samplePhase("Sample.ScalarBindings");
    // Per object, bindings then arrays, exactly as the plain sampler
    // emits them: the sites list is table-ordered, so a merge walk visits
    // the same objects in the same positions. (A bindings loop followed by
    // an arrays loop emits the same SET in a different ORDER whenever the
    // table holds more than one visiting object, which the elementwise
    // contract forbids even though the digest and the worker's first-wins
    // reads are order-insensitive.)
    size_t weightSite = 0;
    for (size_t i = 0; i < B.weightObjects.size(); ++i) {
        if (weightSite < cache->weightSites.size() &&
            cache->weightSites[weightSite] == i) {
            _SampleWeightBindings(B.weightObjects[i], &refreshed, readFlags,
                                  time, &sampled, layer, &memo);
            ++weightSite;
        }
        // Point arrays ride outside the burst memo (always fresh, like the
        // ribbon points below): correctness first, memoization later.
        _SampleWeightArrays(B.weightObjects[i], &refreshed, time, &sampled,
                            layer, memo, i, touched, B.verifyFrozenStatic);
    }
    for (const RigExecBakedProgramImpl::Solver &solver : B.solvers) {
        if (solver.ribbonPointsVarying) {
            _SampleQuery(solver.ribbonPointsPath, solver.ribbonPointsQuery,
                         time, &sampled);
        }
    }
    // Blend channels, as the slow sampler reads them: refreshed-first
    // scalars with the gather's fallback inits, dense points under the
    // sample-prim synthetic key, and a pose-driven weight only when the
    // refreshed inputs hold its path.
    samplePhase("Sample.WeightRibbon");
    for (const RigExecBakedProgramImpl::GeomChain &chain : B.chains) {
        for (const RigExecBakedProgramImpl::GeomRevision &revision :
             chain.revisions) {
            for (const RigExecBakedProgramImpl::GeomBlendChannel &channel :
                 revision.blendChannels) {
                if (channel.weight.IsValid() &&
                    (channel.poseWeight < 0 ||
                     refreshed.Find(channel.weightPath))) {
                    _SampleMoverScalarCached(
                        channel.weight.GetPath(), channel.weight, 0.0f, time,
                        &refreshed, &sampled, cache);
                }
                for (const RigExecBakedProgramImpl::GeomBlendChannel::Sample
                         &sample : channel.samples) {
                    _SampleMoverScalarCached(
                        sample.activation.GetPath(), sample.activation, 1.0f,
                        time, &refreshed, &sampled, cache);
                    _SampleBlendPointsCached(
                        _FrozenBlendInputKey(sample.samplePath, "points"),
                        sample.points, time, &refreshed, &sampled, cache);
                }
            }
        }
    }
    // Through the Build-time handles and keys; the burst memo serves the
    // static channels.
    for (const RigExecBakedProgramImpl::ConstraintArrays &arrays :
         B.constraintArrays) {
        if (!arrays.prim.IsValid()) {
            continue;
        }
        for (size_t channel = 0; channel < 4; ++channel) {
            if (arrays.Sampled(channel)) {
                _SampleAttributeCached(arrays.keys[channel],
                                       arrays.attributes[channel], time,
                                       &sampled, cache);
            }
        }
    }
    samplePhase("Sample.BlendConstraintArrays");
    for (const RigExecBakedProgramImpl::GeomChain &chain : B.chains) {
        if (!_SampleUpstreamBase(chain.target, layer, &sampled)) {
            // Check current availability before serving a memoized value.
            if (!chain.baseQuery.IsValid())
                sampled.Add(chain.target, VtValue(), /*hasValue=*/false);
            else
                _SampleQueryCached(chain.target, chain.baseQuery, time, &sampled,
                                   cache);
        }
        for (const RigExecBakedProgramImpl::GeomChain::Derived &derived :
             chain.derived) {
            if (!derived.matrixTarget && !derived.baseQuery.IsValid())
                sampled.Add(derived.target, VtValue(), /*hasValue=*/false);
            else
                _SampleQueryCached(derived.target, derived.baseQuery, time,
                                   &sampled, cache);
        }
    }
    for (const auto &[chainIndex, revisionIndex] : B.revisionIndex) {
        const RigExecBakedProgramImpl::GeomRevision &revision =
            B.chains[size_t(chainIndex)].revisions[size_t(revisionIndex)];
        const UsdPrim &moverPrim = revision.moverPrim;
        _SampleMoverScalarCached(
            revision.moverPath.AppendProperty(TfToken("inputs:enabled")),
            moverPrim.GetAttribute(TfToken("inputs:enabled")), true, time,
            &refreshed, &sampled, cache);
        _SampleMoverScalarCached(
            revision.moverPath.AppendProperty(TfToken("inputs:defaultWeight")),
            moverPrim.GetAttribute(TfToken("inputs:defaultWeight")), 1.0f,
            time, &refreshed, &sampled, cache);
        _SampleMoverScalarCached(
            revision.moverPath.AppendProperty(TfToken("rigExec:skinningMethod")),
            moverPrim.GetAttribute(TfToken("rigExec:skinningMethod")),
            TfToken("classicLinear"), time, &refreshed, &sampled, cache);
        if (revision.op == RigExecRevisionOp::Wire) {
            _SampleWireInputs(revision, &refreshed, B.stage, time, &sampled);
        }
        if (revision.op == RigExecRevisionOp::Matrix) {
            _SampleMoverToken(moverPrim, revision.moverPath,
                              "rigExec:weightBlend", time, &sampled);
        }
        _SampleIterativeMoverInputs(revision, &refreshed, B.stage, time,
                                    &sampled, layer);
        if (revision.op == RigExecRevisionOp::BlendShape && moverPrim) {
            // rigExec:deltaSpace, as the slow sampler reads it: raw Default,
            // no resolved arm. Unmemoized -- one token read per frame is
            // nothing, and the value doubles as the topology condition.
            TfToken deltaSpace("target");
            const SdfPath deltaSpacePath = revision.moverPath.AppendProperty(
                TfToken("rigExec:deltaSpace"));
            if (const UsdAttribute deltaSpaceAttr =
                    moverPrim.GetAttribute(TfToken("rigExec:deltaSpace"))) {
                TfToken read;
                if (deltaSpaceAttr.Get(&read, UsdTimeCode::Default())) {
                    deltaSpace = read;
                }
                sampled.Add(deltaSpacePath, VtValue(deltaSpace),
                            /*hasValue=*/true);
            }
            if (deltaSpace == "surfaceFrame") {
                _SampleTopologyArrayCached<int>(
                    revision.binding.topologyCounts, time, &refreshed, B.stage,
                    &sampled, cache);
                _SampleTopologyArrayCached<int>(
                    revision.binding.topologyIndices, time, &refreshed, B.stage,
                    &sampled, cache);
            }
        }
        if (moverPrim &&
            (revision.op == RigExecRevisionOp::Smooth ||
             revision.op == RigExecRevisionOp::SurfaceProject)) {
            _SampleTopologyArrayCached<int>(
                revision.binding.topologyCounts, time, &refreshed, B.stage,
                &sampled, cache);
            _SampleTopologyArrayCached<int>(
                revision.binding.topologyIndices, time, &refreshed, B.stage,
                &sampled, cache);
        }
        if (revision.op == RigExecRevisionOp::SurfaceProject && moverPrim) {
            _SampleSurfaceSettings(revision, time, &refreshed, &sampled, nullptr, cache);
            _SampleMoverPathArrayCached<GfVec3f>(
                _FrozenSurfaceInputKey(revision.moverPath, "surfacePoints"),
                revision.binding.surfacePoints, time, &refreshed, B.stage,
                &sampled, cache);
        }
        if (revision.op == RigExecRevisionOp::Lattice && moverPrim) {
            _SampleLatticeSettings(revision, time, &refreshed, &sampled, nullptr, cache);
            _SampleMoverPathArrayAtDefaultCached<GfVec3f>(
                _FrozenLatticeInputKey(revision.moverPath, "cageRest"),
                revision.binding.cagePoints, B.stage, &sampled, cache);
            _SampleMoverPathArrayCached<GfVec3f>(
                _FrozenLatticeInputKey(revision.moverPath, "cageLive"),
                revision.binding.cagePoints, time, &refreshed, B.stage,
                &sampled, cache);
            _SampleLatticeDivisionsCached(
                revision.moverPath.AppendProperty(
                    TfToken("rigExec:divisions")),
                moverPrim.GetAttribute(TfToken("rigExec:divisions")), time,
                &sampled, cache);
        }
        if (revision.op == RigExecRevisionOp::Ribbon && moverPrim) {
            _SampleMoverPathArrayCached<GfVec2f>(
                _FrozenRibbonInputKey(revision.moverPath, "bindCoords"),
                revision.binding.bindCoords, time, &refreshed, B.stage,
                &sampled, cache);
        }
    }
    for (const auto &[chainIndex, derivedIndex] : B.derivedIndex) {
        const RigExecBakedProgramImpl::GeomChain::Derived &derived =
            B.chains[size_t(chainIndex)].derived[size_t(derivedIndex)];
        const RigExecBakedProgramImpl::GeomRevision &revision =
            derived.revision;
        _SampleTopologyArrayCached<int>(revision.binding.topologyCounts, time,
                                        &refreshed, B.stage, &sampled, cache);
        _SampleTopologyArrayCached<int>(revision.binding.topologyIndices, time,
                                        &refreshed, B.stage, &sampled, cache);
        if (revision.op == RigExecRevisionOp::RecomputeExtent) {
            _SampleTopologyArrayCached<float>(revision.binding.widths, time,
                                              &refreshed, B.stage, &sampled,
                                              cache);
        }
        if (RigExecIsDerivedMatrixOp(revision.op)) {
            _SampleProjectorInputs(revision, &refreshed, B.stage, time,
                                   &sampled, layer);
        }
    }
    samplePhase("Sample.LegacyGeometry");
    _SampleLayoutLeaves(B, &refreshed, time, &sampled, layer, &memo,
                        touched);
    samplePhase("Sample.LayoutLeaves");
    std::set<SdfPath> providerPaths;
    size_t providerGets = 0;
    for(size_t k=0;profileSample && k<B.providerFrozenKeys.size();++k) {
        if (B.providerLeaves.attributes[k].IsValid() &&
            !B.providerFrozenKeys[k].IsEmpty()) {
            providerPaths.insert(B.providerLeaves.attributes[k].GetPath());
            providerGets += memo.providerEntries[k] ==
                            RigExecFrozenSamplerMemo::kPerFrame;
        }
    }
    _SampleProviderLeaves(B, memo, time, &sampled);
    samplePhase("Sample.ProviderRawGetBlocked");
    if (profileSample) sampleProfiler.RecordInstant("Sample.ProviderInventory",
        "frozenSample", RigExecProfiler::NowUs(),
        {{"total",std::to_string(B.providerFrozenKeys.size())},
         {"uniqueAttributes",std::to_string(providerPaths.size())},
         {"getCalls",std::to_string(providerGets)},
         {"blockedFactCalls",std::to_string(providerGets)}});
    _SampleRevisionLeaves(B, &refreshed, time, &sampled, layer, &memo,
                          touched);
    if (!_AppendSparseLayoutSources(B, &sampled))
        return fail("sparse layout source census differs from compiled leaves");
    samplePhase("Sample.RevisionLeaves");
    {
        std::string seedsError;
        if (!_SampleStageFrameSeeds(*cache->program, time, &sampled,
                                    &seedsError)) {
            return fail(seedsError);
        }
    }
    samplePhase("Sample.StageFrameSeeds");
    for (const RigExecValueOverride &o : overrides) {
        SdfPath key = o.prim;
        if (!o.attribute.IsEmpty()) {
            key = key.AppendProperty(o.attribute);
        } else if (!o.computation.IsEmpty()) {
            key = key.AppendProperty(TfToken(
                "rigExec:override:" + o.computation.GetString()));
        }
        if (!key.IsEmpty()) {
            sampled.Add(key, o.value, /*hasValue=*/true);
        }
    }

    _AttachDigestOrder(B, &memo, &sampled);
    samplePhase("Sample.OverrideFinish");
    *out = std::move(sampled);
    return true;
}

uint64_t
RigExecFrozenControlDigest(const RigExecFrameInputs &inputs, bool *exact)
{
    uint64_t hash = 1469598103934665603ull;
    const size_t count = inputs.values.size();
    hash = _HashBytes(hash, &count, sizeof(count));
    bool named = true;
    for (const RigExecSampledInput &sampled : inputs.values) {
        hash = _HashString(hash, sampled.path.GetString().c_str());
        hash = _HashBytes(hash, &sampled.hasValue, sizeof(sampled.hasValue));
        hash = _HashBytes(hash, &sampled.valueBlocked, sizeof(sampled.valueBlocked));
        if (sampled.hasValue) {
            named = _HashVtValue(&hash, sampled.value) && named;
        } else {
            hash = _HashString(hash, sampled.value.GetTypeName().c_str());
        }
    }
    if (const RigExecHeadLeafConstants *constants =
            inputs.headLeafConstants.get()) {
        // The constant head leaves, folded as the samples above are.
        hash = _HashString(hash, "head");
        for (size_t j = 0; j < constants->keys.size(); ++j) {
            if (constants->varying[j]) {
                continue;
            }
            const VtValue &value = constants->values[j];
            const bool hasValue = !value.IsEmpty();
            hash = _HashString(hash, constants->keys[j].GetString().c_str());
            hash = _HashBytes(hash, &hasValue, sizeof(hasValue));
            if (hasValue) {
                named = _HashVtValue(&hash, value) && named;
            }
        }
    }
    hash = _HashString(hash,"layout-sources");
    const size_t layoutCount=inputs.layoutLeaves.size();
    hash = _HashBytes(hash,&layoutCount,sizeof(layoutCount));
    named = inputs.layoutSourcePaths.size()==layoutCount && named;
    for(size_t row=0;row<layoutCount;++row) {
        const auto &values=inputs.layoutLeaves[row];
        const size_t size=values.size();
        hash = _HashBytes(hash,&size,sizeof(size));
        if(row>=inputs.layoutSourcePaths.size() || inputs.layoutSourcePaths[row].size()!=size) {
            named=false;
            continue;
        }
        for(size_t k=0;k<size;++k) {
            hash = _HashString(hash,inputs.layoutSourcePaths[row][k].GetString().c_str());
            const bool present=!values[k].IsEmpty();
            hash = _HashBytes(hash,&present,sizeof(present));
            if(present) named = _HashVtValue(&hash,values[k]) && named;
        }
    }
    {
        // The stage seeds, folded like any other control state: a moved
        // constraint target must move this digest too, and a plain vector
        // and its burst-cached twin carry identical seeds.
        const RigExecStageFrameSeeds &seeds = inputs.stageSeeds;
        hash = _HashString(hash, "seeds");
        const auto foldMatrices = [&hash](const std::vector<GfMatrix4d> &ms) {
            const size_t count = ms.size();
            hash = _HashBytes(hash, &count, sizeof(count));
            for (const GfMatrix4d &m : ms) {
                for (int r = 0; r < 4; ++r) {
                    const GfVec4d row = m.GetRow(r);
                    hash = _HashBytes(hash, row.GetArray(),
                                      4 * sizeof(double));
                }
            }
        };
        const auto foldFrames =
            [&hash](const std::vector<RigExecPointFrame> &frames) {
                const size_t count = frames.size();
                hash = _HashBytes(hash, &count, sizeof(count));
                for (const RigExecPointFrame &frame : frames) {
                    for (int i = 0; i < 4; ++i) {
                        hash = _HashBytes(hash, frame.points[i].GetArray(),
                                          3 * sizeof(double));
                    }
                    hash = _HashBytes(hash, &frame.flags,
                                      sizeof(frame.flags));
                }
            };
        const auto foldOk = [&hash](const std::vector<char> &oks) {
            const size_t count = oks.size();
            hash = _HashBytes(hash, &count, sizeof(count));
            if (!oks.empty()) {
                hash = _HashBytes(hash, oks.data(), oks.size());
            }
        };
        hash = _HashBytes(hash, &seeds.requiredStageFramesAdmission.admitted,
                          sizeof(seeds.requiredStageFramesAdmission.admitted));
        hash = _HashBytes(hash, &seeds.requiredStageFramesAdmission.firstBadTarget,
                          sizeof(seeds.requiredStageFramesAdmission.firstBadTarget));
        foldMatrices(seeds.xformBase);
        foldFrames(seeds.xformFrames);
        foldOk(seeds.deltaOk);
        foldMatrices(seeds.deltaBase);
        foldOk(seeds.nativeOk);
        foldFrames(seeds.nativeFrames);
    }
    if (!inputs.upstream.empty()) {
        // The upstream values, by path: a job under another table hashes
        // apart even where no sample shows the difference.
        hash = _HashString(hash, "ups");
        for (const RigExecUpstreamValue &value : inputs.upstream) {
            hash = _HashString(hash, value.path.GetString().c_str());
            hash = _HashBytes(hash, &value.foldHash, sizeof(value.foldHash));
            named = _HashVtValue(&hash, value.value) && named;
        }
    }
    if (exact) {
        *exact = named;
    }
    return hash;
}

} // namespace rigExec
