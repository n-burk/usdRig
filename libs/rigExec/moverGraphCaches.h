// Per-evaluator input and epoch caches the evaluator and the baked program
// hold by value: static inputs, resolved inputs, skin layouts, blend sample
// shapes and wire bases.
#ifndef RIGEXEC_MOVER_GRAPH_CACHES_H
#define RIGEXEC_MOVER_GRAPH_CACHES_H

#include "bodyPurity.h"

#include "rigExecMath/wireKernelCache.h"

#include "pxr/base/gf/vec2f.h"
#include "pxr/base/gf/vec3f.h"
#include "pxr/base/vt/array.h"
#include "pxr/base/vt/value.h"
#include "pxr/usd/sdf/path.h"
#include "pxr/usd/sdf/types.h"
#include "pxr/usd/usd/object.h"
#include "pxr/usd/usd/attribute.h"
#include "pxr/usd/usd/prim.h"
#include "pxr/usd/usd/timeCode.h"

#include <algorithm>
#include <atomic>
#include <cstdint>
#include <functional>
#include <iterator>
#include <map>
#include <memory>
#include <mutex>
#include <set>
#include <thread>
#include <type_traits>
#include <typeinfo>
#include <unordered_map>
#include <vector>

PXR_NAMESPACE_USING_DIRECTIVE

namespace rigExec {

// Named only: the first three are defined in types.h, RigExecWireBasis in
// rigExecMath/geometryKernels.h; the caches hold them by pointer or reference.
struct RigExecSkinTopology;
struct RigExecBlendSampleLayout;
struct RigExecMoverParameters;
struct RigExecWireBasis;

/// Authored input values that cannot change between two reads in a generation.
///
/// The evaluator and the packet assemblers re-read the same handful of
/// authored inputs on every frame -- a constraint's inputs:enabled, its
/// offsets, a weight's defaultWeight -- and each read resolves the attribute
/// through USD again to be handed the same number back. An attribute that is
/// neither connected nor time-varying cannot answer differently until the
/// stage changes, so its first answer is kept and served until it does.
///
/// Correctness rests on three rules, all enforced here or by the owner:
///   * admission: only attributes with no authored connections, no possible
///     time variation AND no authored time samples at all are held. The
///     third test is not implied by the second: USD reports
///     ValueMightBeTimeVarying() == false for an attribute whose strongest
///     opinion is exactly ONE time sample of a non-composable type, while
///     Get(Default) does not see time samples at all -- so for such an
///     attribute a Default read and a numeric read legitimately disagree,
///     and an entry keyed by path alone would serve whichever came first to
///     both. A single-keyed inputs:enabled made the pose at a frame depend
///     on which time codes the same evaluator had evaluated before it;
///   * invalidation: the owner drops, on every stage notice, every entry the
///     notice could have moved -- the changed property itself, everything
///     at or under a resynced or changed prim, and the whole cache for a
///     change at the root, to a resolved asset or inside a prototype -- so
///     an edit reaches the very next read. A notice reports a composed
///     change at every stage path that depends on the edited spec, which is
///     what makes the entry's own path the right key. It also clears it when
///     interactive overrides are set AND when they are cleared, which is
///     defence in depth and not a correctness requirement: an attribute
///     override is written into the generation's resolved inputs before
///     anything reads one, and GetAttribute consults this cache only where
///     the resolved map has no entry, so an overridden attribute is never
///     answered from here in the first place. Kept because a drag is cheap
///     to re-fill from and the alternative is an argument;
///   * precedence: a value this generation already resolved in memory (a
///     property chain result, an interactive override) is consulted before
///     the cache is, by RigExecResolvedInputs::GetAttribute.
class RigExecStaticInputCache
{
public:
    RigExecStaticInputCache() { Clear(); }

    /// Forgets everything. Called on every notice: the cache holds authored
    /// values, and a notice is the only way an authored value moves.
    ///
    /// The counters below are cumulative and deliberately survive this: they
    /// describe what the cache DID, which a test reads across the notices it
    /// provokes.
    void Clear() {
        _entries.clear();
        // The cache is single-threaded state. It is stamped with the thread
        // that owns it here so a read from a worker (a parallel chain walk)
        // bypasses it instead of racing on it.
        _owner = std::this_thread::get_id();
    }

    /// Forgets the entry for \p path alone: a changed-info notice on one
    /// property. Takes ownership for the calling thread exactly as Clear()
    /// does, because the thread that handles notices is the one that reads.
    void Erase(const SdfPath &path) {
        _entries.erase(path);
        _owner = std::this_thread::get_id();
    }

    /// Forgets every entry at or under any of \p prefixes: a resync, or a
    /// change on a prim, reaches every property beneath it. One pass over
    /// the entries for all of them. Re-stamps the owner as Clear() does.
    void ErasePrefixes(const std::vector<SdfPath> &prefixes) {
        _owner = std::this_thread::get_id();
        if (prefixes.empty()) {
            return;
        }
        for (auto it = _entries.begin(); it != _entries.end();) {
            const bool under = std::any_of(
                prefixes.begin(), prefixes.end(),
                [&it](const SdfPath &p) { return it->first.HasPrefix(p); });
            it = under ? _entries.erase(it) : std::next(it);
        }
    }

    /// Reads \p attribute at \p time through the cache.
    ///
    /// Sets \p handled when the answer is authoritative: false means the
    /// attribute is not cacheable and the caller must read it the long way.
    template <class T>
    bool Read(const UsdAttribute &attribute, const SdfPath &path,
              UsdTimeCode time, T *out, bool *handled) {
        *handled = false;
        if (_owner != std::this_thread::get_id()) {
            // Not this cache's thread: neither the map nor the counters may
            // be touched from here. Counted so the bypass is observable --
            // it is the whole of the cache's thread safety.
            ++_bypasses;
            return false;
        }
        auto entry = _entries.find(path);
        if (entry == _entries.end()) {
            _Entry fresh;
            fresh.cacheable = !attribute.HasAuthoredConnections() &&
                              !attribute.ValueMightBeTimeVarying() &&
                              attribute.GetNumTimeSamples() == 0;
            entry = _entries.emplace(path, fresh).first;
        }
        if (!entry->second.cacheable) {
            ++_refusals;
            return false;
        }
        *handled = true;
        if (!entry->second.filled) {
            T value;
            entry->second.hasValue = attribute.Get(&value, time);
            if (entry->second.hasValue) {
                entry->second.value = VtValue(value);
            }
            entry->second.valueType = &typeid(T);
            entry->second.filled = true;
            if (entry->second.hasValue) {
                *out = value;
            }
            return entry->second.hasValue;
        }
        // The same attribute read as another type: the entry answers only
        // the type its first read typed it as -- including the "no value"
        // answer, which a different type may well not share -- so this read
        // goes to the stage.
        if (!entry->second.valueType || *entry->second.valueType != typeid(T)) {
            return attribute.Get(out, time);
        }
        ++_hits;
        if (!entry->second.hasValue) {
            return false;
        }
        *out = entry->second.value.UncheckedGet<T>();
        return true;
    }

    /// How many entries are held, cacheable or refused.
    size_t GetSize() const { return _entries.size(); }
    /// Reads answered from a held value.
    size_t GetHitCount() const { return _hits; }
    /// Reads that arrived from a thread that does not own the cache.
    size_t GetBypassCount() const { return _bypasses; }
    /// Reads on an attribute that admission refused, which the caller then
    /// resolved the long way.
    size_t GetRefusalCount() const { return _refusals; }

private:
    struct _Entry {
        /// The value as the first read of this attribute typed it.
        VtValue value;
        /// That type, so a later read of another one is not answered with
        /// this one's result.
        const std::type_info *valueType = nullptr;
        bool cacheable = false;
        bool filled = false;
        bool hasValue = false;
    };
    std::unordered_map<SdfPath, _Entry, SdfPath::Hash> _entries;
    std::thread::id _owner;
    /// Written from the owning thread except for _bypasses, which is
    /// written from whatever thread bounced off the guard.
    size_t _hits = 0;
    size_t _refusals = 0;
    std::atomic<size_t> _bypasses{0};
};

/// Values evaluation has already computed that a static read must prefer over
/// the authored stage value.
///
/// Packet assembly reads most of its inputs straight off the stage -- a
/// strength, a lattice cage, a topology array. Those reads do not go through
/// exec, so nothing the engine computes reaches them by default, and a
/// property mover that revised one of them would be silently ignored while
/// the same revision reached every exec consumer through a value override.
/// This is the other half of that path: one lookup, consulted first, holding
/// whatever the current generation has already resolved.
class RigExecResolvedInputs
{
public:
    /// Records a property chain's result for \p path.
    void SetProperty(const SdfPath &path, const VtValue &value) {
        _erased.erase(path);
        _values[path] = value;
    }

    /// Forgets \p path, so a read of it goes back to the stage.
    void ClearProperty(const SdfPath &path) {
        _values.erase(path);
        if (_base && _base->Find(path)) _erased.insert(path);
    }

    /// Consumer-local overlay over an immutable source layer. The borrowed
    /// base must outlive this overlay; clones/shadows detach or reset it.
    void SetChainedBase(const RigExecResolvedInputs *base) {
        _values.clear(); _erased.clear();
        _base = base == this ? nullptr : base;
        if (_base) _cache = _base->_cache;
    }

    /// Cold snapshot operations own their effective values independently.
    RigExecResolvedInputs DetachedCopy() const {
        RigExecResolvedInputs copy;
        if (_base) copy = _base->DetachedCopy();
        for (const auto &path : _erased) copy._values.erase(path);
        for (const auto &entry : _values) copy._values[entry.first] = entry.second;
        copy._cache = _cache;
        return copy;
    }

    /// The resolved value for \p path, or null to read the stage.
    const VtValue *Find(const SdfPath &path) const {
        const auto it = _values.find(path);
        if (it != _values.end()) return &it->second;
        if (_erased.count(path)) return nullptr;
        return _base ? _base->Find(path) : nullptr;
    }

    /// Typed convenience: true when \p path resolved to a \p T.
    template <class T>
    bool Get(const SdfPath &path, T *out) const {
        const VtValue *v = Find(path);
        if (!v || !v->IsHolding<T>()) {
            return false;
        }
        *out = v->UncheckedGet<T>();
        return true;
    }

    /// Resolves one scalar/static input exactly as Exec's AttributeValue
    /// accessor does: an in-memory property override wins, otherwise a single
    /// authored attribute connection is followed, otherwise the
    /// attribute's own value is read. Compile validates connection
    /// cardinality/type/cycles for schema inputs that require one scalar.
    /// The float cast GetAttribute applies to a double source; false for
    /// every other type, which never reaches it.
    template <class T>
    static bool _CoerceFromDouble(double, T *) { return false; }
    static bool _CoerceFromDouble(double value, float *out) {
        *out = static_cast<float>(value);
        return true;
    }

    template <class T>
    bool GetAttribute(
        const UsdAttribute &attribute, UsdTimeCode time, T *out) const {
        RIGEXEC_PURITY_CHECK();
        if (!out) {
            return false;
        }
        // A FLOAT read of a DOUBLE attribute is coerced. Every avar is a
        // double while the math movers compute in float, so a float input
        // connected to (or standing on) a control's avar reads its value
        // cast, instead of failing and falling back to a default.
        // Compile-time gated: a non-float instantiation asks no type
        // question of the attribute at all.
        if constexpr (std::is_same<T, float>::value) {
            if (attribute &&
                attribute.GetTypeName() == SdfValueTypeNames->Double) {
                double wide = 0.0;
                if (!GetAttribute<double>(attribute, time, &wide)) {
                    return false;
                }
                return _CoerceFromDouble(wide, out);
            }
        }
        // An input that cannot change until the stage does answers from the
        // cache -- but only after the in-memory value for this exact property
        // has been ruled out, because a property chain result outranks the
        // authored value the cache holds.
        // While no overlay is published there is nothing to rule out, so the
        // read skips the path hash; the path itself is spelled once, because
        // the cache lookup below used to spell it a second time.
        if (attribute) {
            const bool checkOverlay = !_values.empty() || (_base && !_base->IsEmpty());
            const SdfPath attrPath =
                (checkOverlay || _cache) ? attribute.GetPath() : SdfPath();
            if (!checkOverlay || !Find(attrPath)) {
                if (_cache) {
                    bool handled = false;
                    const bool got = _cache->Read(
                        attribute, attrPath, time, out, &handled);
                    if (handled) {
                        return got;
                    }
                }
                // The cache refused it: a connection to follow, or a value
                // that varies with time -- which is every avar, so the
                // refused reads are exactly the ones that recur every frame.
                // MEASURED 2026-09-13, biped: ~11 600 scalar reads per frame,
                // and an unconnected one still reaches the walk below, which
                // allocates a std::set and a std::vector to discover there is
                // no single connection to follow. Same answer, no allocation.
                if (!attribute.HasAuthoredConnections()) {
                    return attribute.Get(out, time);
                }
            }
        }
        std::set<SdfPath> visiting;
        std::vector<UsdAttribute> fallback;
        UsdAttribute a = attribute;
        while (a && visiting.insert(a.GetPath()).second) {
            if (Get(a.GetPath(), out)) {
                return true;
            }
            // A float connection chain may end on a double (an avar).
            if constexpr (std::is_same<T, float>::value) {
                if (a.GetTypeName() == SdfValueTypeNames->Double) {
                    double wide = 0.0;
                    if (!GetAttribute<double>(a, time, &wide)) {
                        return false;
                    }
                    return _CoerceFromDouble(wide, out);
                }
            }
            fallback.push_back(a);
            SdfPathVector connections;
            // Connections are derived from authored opinions only, so an
            // attribute with none can skip building the target index that
            // GetConnections would build to come back empty.
            if (a.HasAuthoredConnections()) {
                a.GetConnections(&connections);
            }
            if (connections.size() != 1) {
                break;
            }
            a = a.GetPrim().GetStage()->GetAttributeAtPath(connections[0]);
        }
        // The nearest readable upstream authored value is the same fallback
        // that recursive connection traversal selected, without consuming a
        // native stack frame for every operation in a long connection chain.
        for (auto it = fallback.rbegin(); it != fallback.rend(); ++it) {
            if (it->Get(out, time)) {
                return true;
            }
        }
        return false;
    }

    /// GetAttribute with \p layer in front of every hop's stage read: each
    /// hop answers from this overlay first (exact type, as GetAttribute),
    /// and where none does, the deepest hop \p layer holds as T or the
    /// stage can read answers, the layer first. \p layer holds
    /// authored-level values (the upstream inputs), so the answer is the
    /// one a stage authoring them gives. The static-input cache is not
    /// consulted while \p layer holds anything; a null or empty layer is
    /// GetAttribute exactly.
    template <class T>
    bool GetAttributeOverStageLayer(
        const UsdAttribute &attribute, UsdTimeCode time,
        const std::map<SdfPath, VtValue> *layer, T *out) const {
        if (!layer || layer->empty()) {
            return GetAttribute(attribute, time, out);
        }
        RIGEXEC_PURITY_CHECK();
        if (!out) {
            return false;
        }
        const auto fromLayer = [layer](const SdfPath &path, T *value) {
            const auto it = layer->find(path);
            if (it == layer->end() || !it->second.IsHolding<T>()) {
                return false;
            }
            *value = it->second.UncheckedGet<T>();
            return true;
        };
        if constexpr (std::is_same<T, float>::value) {
            if (attribute &&
                attribute.GetTypeName() == SdfValueTypeNames->Double) {
                double wide = 0.0;
                if (!GetAttributeOverStageLayer<double>(attribute, time,
                                                        layer, &wide)) {
                    return false;
                }
                return _CoerceFromDouble(wide, out);
            }
        }
        std::set<SdfPath> visiting;
        std::vector<UsdAttribute> fallback;
        UsdAttribute a = attribute;
        while (a && visiting.insert(a.GetPath()).second) {
            if (Get(a.GetPath(), out)) {
                return true;
            }
            if constexpr (std::is_same<T, float>::value) {
                if (a.GetTypeName() == SdfValueTypeNames->Double) {
                    double wide = 0.0;
                    if (!GetAttributeOverStageLayer<double>(a, time, layer,
                                                            &wide)) {
                        return false;
                    }
                    return _CoerceFromDouble(wide, out);
                }
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
            if (fromLayer(it->GetPath(), out) || it->Get(out, time)) {
                return true;
            }
        }
        return false;
    }

    bool IsEmpty() const { return GetSize() == 0; }
    size_t GetSize() const {
        if (!_base) return _values.size();
        // This cold inspection is not used by consumer typed lookup.
        return DetachedCopy()._values.size();
    }
    /// Whether \p other holds the same values at the same paths.
    bool HasSameValues(const RigExecResolvedInputs &other) const {
        return DetachedCopy()._values == other.DetachedCopy()._values;
    }
    void Clear() { _values.clear(); _erased.clear(); _base = nullptr; }

    /// Attaches the owner's static-input cache. Not owned, and not cleared
    /// by Clear(): this object is emptied every generation, while the cache
    /// spans generations and is invalidated by stage notices.
    void SetStaticCache(RigExecStaticInputCache *cache) { _cache = cache; }

private:
    std::unordered_map<SdfPath, VtValue, SdfPath::Hash> _values;
    std::set<SdfPath> _erased;
    const RigExecResolvedInputs *_base = nullptr;
    RigExecStaticInputCache *_cache = nullptr;
};

/// Per-epoch skin layouts, keyed by the mover that owns them.
///
/// The expensive half of the operation depends only on the layout, and the
/// layout is what an epoch IS. Here that half is reading two megabyte-scale
/// arrays off the stage and range-checking every element of them.
///
/// Owned by the evaluator, never a static: a cache that outlives the
/// evaluator outlives the stage it read, and a weight-paint edit has to be
/// able to throw it away. Clear() is what a change notice calls.
///
/// Resolve() is safe to call from several chain tasks at once, because the
/// chain walk runs independent chains concurrently and every skinned chain
/// asks here. The lock decides nothing: a layout is a pure function of the
/// mover's authored arrays, so whichever task happens to build it builds the
/// same one, and after the first frame of an epoch every call is a hit.
class RigExecSkinTopologyCache
{
public:
    /// The layout for \p mover, calling \p build on a miss. \p build fills
    /// a fresh topology; whatever it produces -- validated or not -- is what
    /// this epoch uses, so a rejected layout is not re-read every frame
    /// either.
    ///
    /// \p build returns false to REFUSE the cache for this mover: the layout
    /// can move within the epoch after all, and the caller must read it per
    /// frame instead. The refusal is remembered exactly as a layout is, so
    /// the question costs one answer per notice and not one per frame; a
    /// refused mover answers null until the next Clear().
    std::shared_ptr<const RigExecSkinTopology> Resolve(
        const SdfPath &mover,
        const std::function<bool(RigExecSkinTopology *)> &build);

    void Clear() {
        std::lock_guard<std::mutex> lock(_mutex);
        // Dropped as ANSWERS, kept as candidates. Every notice clears this
        // cache, including the overwhelming majority that touched no
        // layout; handing back a fresh pointer for arrays that compare equal
        // would make the mover's packet compare unequal and re-run the
        // whole per-point kernel for a binding that did not move. Resolve
        // pays one array compare per notice to avoid that, instead of the
        // kernel once per notice.
        for (auto &[mover, topology] : _entries) {
            if (topology) {
                _candidates[mover] = std::move(topology);
            }
        }
        _entries.clear();
    }
    size_t GetSize() const {
        std::lock_guard<std::mutex> lock(_mutex);
        return _entries.size();
    }

private:
    mutable std::mutex _mutex;
    /// A present entry holding null is a remembered refusal.
    std::map<SdfPath, std::shared_ptr<const RigExecSkinTopology>> _entries;
    /// The last layout each mover had, from before the most recent Clear(),
    /// so an unchanged binding can keep its pointer. Never an answer: only
    /// something a freshly read layout is compared against.
    std::map<SdfPath, std::shared_ptr<const RigExecSkinTopology>> _candidates;
};

/// Per-epoch blend sample shapes, keyed by the sample prim that owns them.
///
/// Peer of RigExecSkinTopologyCache, and there for the same reason, but the
/// arithmetic is starker here. A dense blend sample's full points array is
/// read and copied off the stage once per sample per frame whether its
/// channel sits at 0 or at 1 -- measured at 0.37-0.38 ms per sample per frame
/// on a 26,276-point body, so 169 correctives cost ~65 ms/frame with the rig
/// standing at REST. Resolving the shape
/// once per epoch and sharing it by pointer is what removes that, and the
/// sparse layout is what makes the resolved shape small: the real correctives
/// move 1,279 points on average, 4.87% of the mesh.
///
/// Owned by the evaluator, never a static: a cache that outlives the
/// evaluator outlives the stage it read, and a sculpt edit has to be able to
/// throw it away. Clear() is what a change notice calls.
///
/// Resolve() is safe to call from several chain tasks at once. The lock
/// decides nothing -- a layout is a pure function of the sample's authored
/// arrays, so whichever task builds it builds the same one.
class RigExecBlendSampleCache
{
public:
    /// The shape for \p sample, calling \p build on a miss.
    ///
    /// \p build returns false to REFUSE the cache for this sample: the shape
    /// can move within the epoch after all, and the caller must read it per
    /// frame instead. The refusal is remembered exactly as a shape is, so the
    /// question costs one answer per notice and not one per frame; a refused
    /// sample answers null until the next Clear().
    std::shared_ptr<const RigExecBlendSampleLayout> Resolve(
        const SdfPath &sample,
        const std::function<bool(RigExecBlendSampleLayout *)> &build);

    /// Adopt after the evaluator's Run joins, with stage edits and evaluator
    /// calls serialized on its owner thread. Equal layouts retain identity.
    std::shared_ptr<const RigExecBlendSampleLayout> AdoptExclusive(
        const SdfPath &sample,
        std::shared_ptr<const RigExecBlendSampleLayout> layout, bool refused);

    void Clear() {
        std::lock_guard<std::mutex> lock(_mutex);
        // Dropped as ANSWERS, kept as candidates -- see
        // RigExecSkinTopologyCache::Clear for why handing back a fresh
        // pointer for arrays that compare equal is worse than one array
        // compare per notice.
        for (auto &[sample, layout] : _entries) {
            if (layout) {
                _candidates[sample] = std::move(layout);
            }
        }
        _entries.clear();
    }
    /// Clear() for the one sample \p sample: its shape is dropped as an
    /// answer and kept as a candidate, so a re-read that finds the same
    /// arrays hands back the same pointer.
    void Erase(const SdfPath &sample) {
        std::lock_guard<std::mutex> lock(_mutex);
        const auto found = _entries.find(sample);
        if (found == _entries.end()) {
            return;
        }
        if (found->second) {
            _candidates[sample] = std::move(found->second);
        }
        _entries.erase(found);
    }
    size_t GetSize() const {
        std::lock_guard<std::mutex> lock(_mutex);
        return _entries.size();
    }

private:
    mutable std::mutex _mutex;
    /// A present entry holding null is a remembered refusal.
    std::map<SdfPath, std::shared_ptr<const RigExecBlendSampleLayout>> _entries;
    /// The last shape each sample had, from before the most recent Clear().
    std::map<SdfPath, std::shared_ptr<const RigExecBlendSampleLayout>>
        _candidates;
};

/// The bases of a sparse-envelope wire revision, memoized by content.
///
/// A basis depends on the bind table, the weighted point indices, the knots,
/// the order, the control and mesh point counts and the dropoff, never on
/// where the control points are, so a frame whose inputs repeat is a lookup.
/// The key is a hash of those inputs' contents, not their addresses (an
/// edited table can reuse a freed buffer); a hit compares the full inputs, so
/// a collision rebuilds. A rebuild is pure, so a hit, a miss and an eviction
/// all answer the same basis values.
///
/// OWNED, NOT SHARED: one per mover graph (one dynamic chain, run by one task
/// at a time) and one per baked revision (run by one step at a time), so no
/// lock guards it. A copy shares the immutable entries, which is how a frozen
/// clone starts from the program's bases.
class RigExecWireBasisCache
{
public:
    /// Entries held before the map is cleared; bind tables are fixed in
    /// practice, so one owner rarely holds more than one.
    static constexpr size_t kCapacity = 64;

    /// The basis for \p p's bind table over \p indices on a mesh of
    /// \p meshPoints points, built on a miss; null for an invalid curve
    /// layout.
    std::shared_ptr<const RigExecWireBasis> Get(
        const RigExecMoverParameters &p, const std::vector<int> &indices,
        size_t meshPoints);

    size_t Size() const { return _entries.size(); }
    /// Test observable: bases built since construction (copies included).
    size_t BuildCount() const { return _builds; }

private:
    struct _Entry;
    std::unordered_map<uint64_t, std::shared_ptr<const _Entry>> _entries;
    size_t _builds = 0;
    std::shared_ptr<const _Entry> _last;
public:
    RigExecWireRestCache<GfVec3f,GfVec2f> restEvaluations;
};

}  // namespace rigExec

#endif  // RIGEXEC_MOVER_GRAPH_CACHES_H
