// RigExec compiled mover graph (spec §7.2).
// The per-(mover, target) revision chain is a VdfNetwork built in memory from
// the relationships already authored on the user's stage. Nothing is authored
// anywhere to express it: no generated prims, no compiler, no derived stage, and
// no schema types for the revisions themselves. Compiled nodes live only on the
// graph side.
// This is what the write-set authoring model always meant. A mover says "I
// write this target" and its namespace position says "in this order"; the chain
// of revisions that implies is dataflow, and dataflow is what a VdfNetwork is
// for. Materializing it as USD prims required a second stage to hold them, cost
// a recomposition, and put engine machinery on the authoring surface -- while
// still not being able to express the optimizations the graph form makes
// natural (splitting one revision across face sets, cloning legs, per-element
// masks driving sparse recomputation).
#ifndef RIGEXEC_MOVER_GRAPH_H
#define RIGEXEC_MOVER_GRAPH_H

#include "types.h"

#include "rigExecMath/simdKernels.h"
#include "rigExecMath/solvers.h"

#include "pxr/base/vt/array.h"
#include "pxr/base/vt/value.h"
#include "pxr/base/gf/vec3f.h"
#include "pxr/exec/vdf/maskedOutput.h"
#include "pxr/exec/vdf/network.h"
#include "pxr/usd/sdf/path.h"
#include "pxr/usd/usd/object.h"
#include "pxr/usd/usd/attribute.h"
#include "pxr/usd/usd/prim.h"
#include "pxr/usd/usd/timeCode.h"

#include <algorithm>
#include <array>
#include <atomic>
#include <cstdint>
#include <functional>
#include <iterator>
#include <map>
#include <memory>
#include <mutex>
#include <optional>
#include <set>
#include <string>
#include <thread>
#include <typeinfo>
#include <unordered_map>
#include <vector>

PXR_NAMESPACE_USING_DIRECTIVE

namespace rigExec {

/// The operation a revision performs. One node type per operation and value
/// type, mirroring the frozen application signatures (spec §4.1): no runtime
/// operation dispatch inside a node.
enum class RigExecRevisionOp {
    Matrix,
    Skin,
    BlendShape,
    VolumeCorrect,
    Smooth,
    Lattice,
    SurfaceProject,
    Ribbon,
    /// RigExecCurveMover "wire": points follow a NURBS driver curve's
    /// displacement at their bind parameter (RigExecApplyWire).
    Wire,
    EmitGuidePoints,
    // Values 10 and 11 are reserved by the binary wire format.
    RecomputeNormals = 12,
    RecomputeExtent,
    // Append new ops: existing values are pinned by the binary wire format.
    DeltaMush,
    Wrinkle,
    /// A registered plugin supplies parameter assembly and point deformation.
    External,
    /// RigExecSurfaceProjector: a derived matrix primvar measured from the
    /// chain's final points (rigExec:shaderPrimvar).
    SurfaceProjector,
    /// RigExecSurfaceProjector: its shader dials packed into a derived
    /// matrix primvar (rigExec:shaderDialPrimvar).
    ShaderDials,
};

/// Whether \p op publishes a derived MATRIX primvar rather than revising
/// points or a vec3f array.
inline bool
RigExecIsDerivedMatrixOp(RigExecRevisionOp op)
{
    return op == RigExecRevisionOp::SurfaceProjector ||
           op == RigExecRevisionOp::ShaderDials;
}

/// When in the walk a side input takes its value from.
///
/// A mover or solver reads things other operators write. Which REVISION of
/// those things it gets is a separate authored choice from which path it
/// reads, declared as rigExecReadPhase metadata ON the relationship that
/// names the input: the phase sits where the binding is, so any input can
/// carry one and no schema attribute is needed to introduce another.
enum class RigExecReadPhaseKind {
    Base,       ///< the authored value: what the stage resolves at this time
    Preceding,  ///< the value immediately before the reading mover
    Final,      ///< the value after every mover that writes it has run
    AtPrim,     ///< the value as of when the walk finished with a named prim
};

/// A resolved read phase. \p prim is meaningful only for AtPrim.
///
/// AtPrim is the general form the other three are shorthands for: the walk is
/// reverse-sibling post-order over the whole composed rig, so "as of
/// this prim" means the moment that prim was finished with -- for a mover,
/// immediately after it applied; for a grouping Scope, after everything
/// beneath it applied, because post-order visits a parent last. Naming a Scope
/// is therefore how an author says "after that whole rigging stage", without
/// listing its contents.
struct RigExecReadPhase {
    RigExecReadPhaseKind kind = RigExecReadPhaseKind::Base;
    SdfPath prim;

    bool IsBase() const { return kind == RigExecReadPhaseKind::Base; }
    bool operator==(const RigExecReadPhase &o) const {
        return kind == o.kind && prim == o.prim;
    }
    bool operator!=(const RigExecReadPhase &o) const { return !(*this == o); }
    /// Stable text for digests and diagnostics.
    std::string GetAsString() const;
};

/// The metadata field an input's read phase is authored in.
///
/// Not namespaced: USD metadata field names are plain identifiers, and
/// `rigExec:readPhase` does not parse in a metadata position.
inline constexpr const char *RigExecReadPhaseMetadataName = "rigExecReadPhase";

/// Parses an authored phase string.
///
/// Accepts `base`, `preceding`, `final`, or an absolute prim path. Returns
/// false for anything else -- including a relative path, which has no
/// unambiguous meaning here -- and fills \p error.
bool RigExecParseReadPhase(
    const std::string &authored, RigExecReadPhase *phase, std::string *error);

/// The read phase declared for \p property: its rigExecReadPhase metadata,
/// or Base when none is authored.
///
/// Metadata is the only way to declare one; no schema attribute stands in
/// for it. Returns false and fills \p error on an unparseable authored
/// value; an absent declaration is Base and true.
bool RigExecResolveReadPhase(
    const UsdObject &property,
    RigExecReadPhase *phase,
    std::string *error);

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
        _values[path] = value;
    }

    /// Forgets \p path, so a read of it goes back to the stage.
    void ClearProperty(const SdfPath &path) { _values.erase(path); }

    /// The resolved value for \p path, or null to read the stage.
    const VtValue *Find(const SdfPath &path) const {
        const auto it = _values.find(path);
        return it == _values.end() ? nullptr : &it->second;
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
            const bool checkOverlay = !_values.empty();
            const SdfPath attrPath =
                (checkOverlay || _cache) ? attribute.GetPath() : SdfPath();
            if (!checkOverlay || !_values.count(attrPath)) {
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

    bool IsEmpty() const { return _values.empty(); }
    size_t GetSize() const { return _values.size(); }
    void Clear() { _values.clear(); }

    /// Attaches the owner's static-input cache. Not owned, and not cleared
    /// by Clear(): this object is emptied every generation, while the cache
    /// spans generations and is invalidated by stage notices.
    void SetStaticCache(RigExecStaticInputCache *cache) { _cache = cache; }

private:
    std::unordered_map<SdfPath, VtValue, SdfPath::Hash> _values;
    RigExecStaticInputCache *_cache = nullptr;
};

/// What each chain held at each point in the walk.
///
/// A chain is a sequence of revisions, and until now only its two ends were
/// nameable -- the authored base going in and the final value coming out.
/// Everything between them existed for a moment inside the evaluation loop
/// and was dropped. A read phase that names a prim needs exactly one of those
/// intermediate values, so they are kept: one entry per (target, mover) as
/// the chain is built, plus the final.
///
/// Cheap by construction. Chains are short, the values are already
/// materialized to publish the result anyway, and VtValue's array backing is
/// copy-on-write -- so recording a revision costs a refcount, not a copy of
/// the geometry.
class RigExecChainSnapshots
{
public:
    /// Records \p value as \p target stood immediately after \p afterMover.
    void Record(const SdfPath &target, const SdfPath &afterMover,
                const VtValue &value);

    /// Records \p target's value after its whole chain.
    void RecordFinal(const SdfPath &target, const VtValue &value);

    /// The value of \p target at \p phase, or null when nothing was recorded.
    ///
    /// \p readerMover is the mover doing the reading, needed by Preceding.
    /// An AtPrim phase naming a grouping Scope resolves to the LAST recorded
    /// revision at or beneath it, which is what post-order makes that Scope
    /// mean.
    const VtValue *Lookup(
        const SdfPath &target, const RigExecReadPhase &phase,
        const SdfPath &readerMover) const;

    /// Takes over everything recorded in \p other, appending its revisions
    /// after any this already holds for the same target.
    ///
    /// One chain's records are written by whoever walked that chain and are
    /// folded in here afterwards, in chain order. That is what lets the walk
    /// hand a task its own store instead of this one: a task records into a
    /// store nobody else can see, and the walk order -- not the order the
    /// tasks happened to finish in -- decides what this ends up holding.
    void Merge(RigExecChainSnapshots &&other);

    void Clear() { _chains.clear(); }
    bool IsEmpty() const { return _chains.empty(); }

private:
    struct _Chain {
        /// (mover, value) in walk order. A vector, not a map: Preceding and
        /// the Scope rule are both positional questions.
        std::vector<std::pair<SdfPath, VtValue>> revisions;
        VtValue final;
        bool hasFinal = false;
    };
    std::map<SdfPath, _Chain> _chains;
};

/// Where one revision reads its side inputs from.
///
/// These are the bindings the compiler used to author onto the mover prim as
/// rigExec:resolved* relationships so a registered computation could name them.
/// Every one is a build-time path choice -- which provider supplies the matrix,
/// which prim carries the topology, which attribute holds the cage -- so in the
/// graph they are just the paths an edge will be built from, and nothing needs
/// to be authored anywhere to express them.
struct RigExecBlendSampleBinding {
    SdfPath sample;
    SdfPath points;
    RigExecReadPhase phase;
    /// The UsdSkelBlendShape prim named by rigExec:blendShape, when the
    /// sample carries its shape sparsely instead of as a full points array.
    /// Empty and `points` set is the dense form; set and `points` empty is
    /// the sparse one. Never both: the compiler rejects a sample that
    /// authors both relationships rather than picking a winner.
    SdfPath blendShape;
    bool operator==(const RigExecBlendSampleBinding &o) const {
        return sample == o.sample && points == o.points &&
               phase == o.phase && blendShape == o.blendShape;
    }
};

struct RigExecRevisionBinding {
    SdfPath moverPath;        ///< the authored mover
    SdfPath target;           ///< canonical exact write target
    SdfPath transform;        ///< computeMatrix provider (matrix)
    /// Optional provider the transform is measured against (matrix):
    /// T = M(transform) * inverse(M(transformSpace)).
    SdfPath transformSpace;
    /// rigExec:space -- the provider whose own rest->pose map carries the
    /// WHOLE RIG, normally a TRS master, and which is a different thing
    /// from transformSpace above: that one is what the offset is MEASURED
    /// against, this one is what the points the offset is applied to have
    /// already been carried by. Empty when none is named, and then a
    /// post-skin cluster keeps the scale-only correction it had before.
    /// See RigExecClusterInPointFrame.
    SdfPath carrySpace;
    /// Ordered computeMatrix providers (skin): rigExec:influences, which
    /// rigExec:jointIndices index. Every entry shares transformPhase.
    /// Matrix movers use [referenceTransform, referenceTransformSpace]
    /// here when an explicit neutral solve supplies the deformation bind.
    std::vector<SdfPath> influences;
    SdfPath weightObject;     ///< computeWeightPacket provider
    SdfPath base;             ///< authored-base points (blend/volume/lattice)
    SdfPath topologyCounts;   ///< faceVertexCounts (smooth/surface)
    SdfPath topologyIndices;  ///< faceVertexIndices (smooth/surface)
    SdfPath cagePoints;       ///< lattice cage points
    SdfPath surfacePoints;    ///< driver surface points
    SdfPath bindCoords;       ///< ribbon / wire bind coordinates
    SdfPath driverCurvePoints; ///< wire: driver NURBS curve points
    SdfPath driverCurveOrder;  ///< wire: that curve's order
    SdfPath driverCurveKnots;  ///< wire: that curve's knots
    /// wire: how many of `influences` are rigExec:driverTransforms; the
    /// rest are rigExec:driverTransformSpaces. Zero when the curve's own
    /// points drive the wire.
    int driverTransformCount = 0;
    /// wire: how many of `influences` after the transforms are their
    /// spaces, and how many after those are rigExec:driverBaseTransforms;
    /// the rest are rigExec:driverBaseTransformSpaces.
    int driverSpaceCount = 0;
    int driverBaseTransformCount = 0;
    SdfPath driverFrames;     ///< aggregate frame provider
    SdfPath widths;           ///< authored widths (extent maintenance)
    /// Surface projector: rigExec:shaderDialSources, in order, at most
    /// sixteen, and the static asset-to-mesh map. The historical field name
    /// is retained on the C++/binary binding; provider frames are asset-space.
    std::vector<SdfPath> shaderDials;
    GfMatrix4d meshWorldInverse{1.0};
    std::vector<SdfPath> blendInputs;  ///< sorted blend channels
    std::map<SdfPath, std::vector<RigExecBlendSampleBinding>> blendSamples;

    /// Declared read phase per side input, keyed by the exact property path
    /// the phase governs. Absent means Base, which is what an unannotated
    /// input has always meant.
    std::map<SdfPath, RigExecReadPhase> phases;

    /// The phase declared for the transform provider. Kept apart from
    /// `phases` because it is answered from the FRAME chains rather than the
    /// point chains -- a different store with a different value type.
    RigExecReadPhase transformPhase;

    std::vector<std::pair<SdfPath, RigExecReadPhase>> GetPhasedInputs() const {
        std::vector<std::pair<SdfPath, RigExecReadPhase>> result(
            phases.begin(), phases.end());
        for (const auto &[input, samples] : blendSamples)
            for (const auto &sample : samples)
                if (!sample.phase.IsBase()) result.emplace_back(sample.points, sample.phase);
        return result;
    }

    bool operator==(const RigExecRevisionBinding &o) const;
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

/// The epoch-fixed skin layout of \p moverPrim, through \p cache.
///
/// The one call a frame makes that takes a lock (RigExecSkinTopologyCache's,
/// held across the build). Exposed so a caller that must not take a lock
/// where it assembles -- the baked program, whose step bodies may not
/// synchronise at all -- can resolve every layout up front and hand the
/// answer to the assembler through RigExecProviderValues::skinTopology. A
/// null return is the cache's remembered REFUSAL: the layout can move within
/// the epoch, so the packet must read the arrays per frame.
std::shared_ptr<const RigExecSkinTopology> RigExecResolveSkinTopology(
    const UsdPrim &moverPrim,
    size_t influenceCount,
    UsdTimeCode time,
    const RigExecResolvedInputs *resolved,
    RigExecSkinTopologyCache *cache);
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

/// Resolves a mover's side-input bindings from the authored stage.
///
/// \p frameChainHeads maps a transform provider to its final frame-chain head,
/// which is what a "final" read phase selects; an empty map resolves every
/// phase to the authored provider. Pure resolution: reads the stage, authors
/// nothing.
RigExecRevisionBinding RigExecResolveRevisionBinding(
    const UsdPrim &moverPrim,
    const SdfPath &target,
    const std::map<SdfPath, SdfPath> &frameChainHeads);

/// The revision op a mover's schema type performs, or nullopt when the type is
/// not a point-chain mover.
std::optional<RigExecRevisionOp> RigExecRevisionOpForSchema(
    const TfToken &schemaType, const TfToken &curveMode);

/// Assembles a matrix mover's parameter packet.
///
/// Peer of _BuildMatrixMoverParameters in movers/matrixMover.cpp, but built from
/// values rather than from a VdfContext: \p transform and \p weights are the
/// already-evaluated results of the providers named by the revision binding,
/// pulled through a tap set on the authored stage. A null transform fails the
/// application. Null weights mean no object is bound, so the assembler reads
/// inputs:defaultWeight and synthesizes the common constant envelope.
/// The kind token an assembled packet carries for op (the table the
/// revision guard and kernel entry read).
const TfToken &RigExecRevisionKindToken(RigExecRevisionOp op);

RigExecMoverParameters RigExecAssembleMatrixParameters(
    const UsdPrim &moverPrim,
    const GfMatrix4d *transform,
    const RigExecWeightPacket *weights,
    UsdTimeCode time = UsdTimeCode::Default(),
    const RigExecResolvedInputs *resolved = nullptr);

/// Assembles a skin mover's parameter packet.
///
/// \p influenceTransforms are the already-evaluated computeMatrix results of
/// the providers named by rigExec:influences, in that order; null fails the
/// application. The per-point layout (rigExec:jointIndices, jointWeights,
/// elementSize) and the method token are static reads off the mover prim at
/// \p time -- unless \p topologyCache is given, in which case the layout is
/// resolved through it once per epoch and carried in the packet by handle.
/// The caller passes a cache only when Compile established that the layout
/// cannot change within the epoch. The packet is valid only when the layout
/// indexes the influence table in range with finite non-negative weights, so
/// the kernel never has to guard an element; the point-count half of the
/// check happens in the kernel, which is the first place the count is known.
RigExecMoverParameters RigExecAssembleSkinParameters(
    const UsdPrim &moverPrim,
    const std::vector<GfMatrix4d> *influenceTransforms,
    const RigExecWeightPacket *weights,
    UsdTimeCode time = UsdTimeCode::Default(),
    const RigExecResolvedInputs *resolved = nullptr,
    RigExecSkinTopologyCache *topologyCache = nullptr,
    const std::shared_ptr<const RigExecSkinTopology> *resolvedTopology =
        nullptr);

/// Derives a mover's status from its packet (spec §6.6): disabled and failed
/// movers both pass their preceding revision through, and a failure records the
/// first bad canonical address.
RigExecMoverStatus RigExecStatusForParameters(
    const RigExecMoverParameters &parameters, const SdfPath &moverPath);

/// Applies the matrix operation of \p p to \p pts in place, returning false
/// when the packet fails atomically (the envelope does not resolve to the
/// point count).
///
/// The envelope is resolved and applied INSIDE the kernel, unlike the
/// point3f[] ops: the weighted-matrix rule folds the weight into the movement
/// (p' = q + w (T q - q)) rather than blending a finished result.
///
/// Shared by the mover-graph revision node and by the baked program, which
/// runs the same operation with no VdfNetwork around it.
/// The scale a posed frame carries: the length of each column of its linear
/// part.
///
/// USD's row-vector convention puts a master's own transform on the RIGHT of
/// its descendants (world = local * parent), so a frame under a scaled
/// master factors as rigid * scale -- and for an orthonormal L,
/// (L S)^T (L S) = S L^T L S = S^2, whose diagonal is exactly those column
/// lengths. A frame nothing has scaled returns (1, 1, 1), so dividing by it
/// is the identity: a rig with no scaled master measures what it always did,
/// to the last bit.
///
/// A zero-length column is a degenerate frame that cannot be divided by, so
/// it reports 1 and leaves that axis alone rather than producing infinities.
inline GfVec3d
RigExecFrameScale(const GfMatrix4d &frame)
{
    GfVec3d scale(1.0, 1.0, 1.0);
    for (size_t c = 0; c < 3; ++c) {
        const double length =
            GfVec3d(frame[0][c], frame[1][c], frame[2][c]).GetLength();
        if (length > 0.0) {
            scale[c] = length;
        }
    }
    return scale;
}

/// M(transform) * inverse(M(space)) for rigExec:transformSpace, with the
/// projective column set exactly: the product of an affine matrix and an
/// affine inverse is affine, but not to the last bit, and the matrix mover
/// refuses a transform whose last column is not exactly (0, 0, 0, 1).
inline GfMatrix4d
RigExecMeasureInSpace(const GfMatrix4d &transform, const GfMatrix4d &space)
{
    GfMatrix4d m = transform * space.GetInverse();
    m[0][3] = 0.0;
    m[1][3] = 0.0;
    m[2][3] = 0.0;
    m[3][3] = 1.0;
    return m;
}

/// Remove an explicit neutral solve before applying the animated map.
/// As above, restore the exact affine column after inverse multiplication.
inline GfMatrix4d
RigExecMeasureFromReference(const GfMatrix4d &transform, const GfMatrix4d &reference)
{
    GfMatrix4d m = reference.GetInverse() * transform;
    m[0][3] = m[1][3] = m[2][3] = 0.0;
    m[3][3] = 1.0;
    return m;
}

/// inverse(M(space)) * M(transform): the same offset RigExecMeasureInSpace
/// measures, conjugated into the space's CURRENT frame rather than left in
/// the space's own coordinates.
///
/// The pair exists because world = local * parent in USD's row-vector
/// convention, so transform * space^-1 cancels the space and hands back the
/// driver's plain LOCAL matrix. Applied to world-space control points that
/// is a local offset used as a world one, and it points wherever it pointed
/// at bind however far the space has since been carried. Reversing the
/// product gives space^-1 * local * space, which rides the space.
///
/// Both return identity when the transform sits exactly at its space, so
/// swapping one for the other never makes a still rig move.
inline GfMatrix4d
RigExecMeasureInPosedSpace(const GfMatrix4d &transform, const GfMatrix4d &space)
{
    GfMatrix4d m = space.GetInverse() * transform;
    m[0][3] = 0.0;
    m[1][3] = 0.0;
    m[2][3] = 0.0;
    m[3][3] = 1.0;
    return m;
}


/// The offset a cluster applies, in the frame its POINTS are already in.
///
/// RigExecMeasureInSpace hands back `transform * space^-1`, which IS
/// invariant under anything applied above the rig: `(X*T) * (S*T)^-1` is
/// `X * S^-1` again. What is NOT invariant is applying that offset to
/// WORLD points, because those points have been carried:
///
///     gets    (p + T) * M  =  p*M + T*R
///     wants    p*M + T
///
/// which agree only when M's rotation R is the identity.
///
/// \p carry is the rig's own rest->pose map -- rigExec:space's
/// computeMatrix, normally a TRS master's -- and conjugating by it is the
/// whole correction: `(p+T) * T^-1*M*T` is `p*M + T`, which is what
/// rigidity wants, and the same product handles a rotated or scaled master
/// because nothing about it assumed a translation. Measured 2026-09-24 on
/// the biped with the face posed and Main moved (40, 0, 25):
/// head_top_aim_cluster alone put 5.70 units of shear into body_geo, and
/// with the carry named it is 0.0157 -- the residual every OTHER mover
/// contributes, which this function cannot see.
///
/// WITHOUT a carry the correction falls back to conjugating by the SPACE's
/// own scale, which is what this function did before a carry could be
/// named: it fixes a scaled master (the squetch clusters went 25.02 ->
/// 3.1e-5 on M_HeadwireTop tx=2.0) and does nothing whatever for a
/// translated or rotated one. Kept, and kept bit-identical, because a rig
/// that names no space must not lose the half-fix it already had.
///
/// The two never compose: a named carry already contains the master's
/// scale, so applying the scale conjugation as well would apply it twice.
///
/// A rig naming no space and standing under an unscaled master measures
/// (1,1,1) and gets \p m back untouched, so it is bit-identical.
inline GfMatrix4d
RigExecClusterInPointFrame(const GfMatrix4d &m, const GfMatrix4d &space,
                           bool posedPoints,
                           const GfMatrix4d *carry = nullptr)
{
    if (!posedPoints) {
        return m;
    }
    GfMatrix4d out;
    if (carry) {
        // The guard is "a carry was NAMED", never "the carry is identity":
        // `carry^-1 * m * carry` for a master standing at its rest is m to
        // the last few ulps and not bit for bit, and a rig that named a
        // space has asked for that product. What must stay untouched is the
        // rig that named NOTHING, which is the branch below.
        out = carry->GetInverse() * m * *carry;
    } else {
        const GfVec3d k = RigExecFrameScale(space);
        if (k == GfVec3d(1.0)) {
            return m;
        }
        GfMatrix4d scale(1.0), unscale(1.0);
        scale.SetScale(k);
        unscale.SetScale(GfVec3d(1.0 / k[0], 1.0 / k[1], 1.0 / k[2]));
        out = unscale * m * scale;
    }
    out[0][3] = 0.0;
    out[1][3] = 0.0;
    out[2][3] = 0.0;
    out[3][3] = 1.0;
    return out;
}

/// Carries a transform-driven wire's two control polygons into the frame
/// its POINTS are already in. The wire's counterpart of
/// RigExecClusterInPointFrame: the same correction, applied to a
/// difference vector instead of an offset matrix.
///
/// The applier adds `posed(u) - rest(u)`, and for a driver measured as
/// `T * S^-1` that difference is master-invariant by construction: under a
/// master motion M both T and S pick up M and it cancels. The control
/// points the driver is applied to are the curve's AUTHORED ones, so the
/// delta is computed wholly in the uncarried frame -- and then added to
/// points a skin has already carried by M:
///
///     gets    p*M + d
///     wants   (p + d)*M  =  p*M + d*M_linear
///
/// A translation leaves a difference vector alone, which is why this is
/// invisible under a moved master and shows only under a rotated or
/// scaled one. Transforming BOTH polygons by the carry makes the kernel's
/// difference `d*M_linear`: the translation cancels in the subtraction,
/// and NURBS evaluation commutes with an affine map because the basis is
/// a partition of unity, so Evaluate(carried CVs) == carry(Evaluate(CVs)).
/// The bind table (u, d) was measured against the authored rest curve and
/// stays valid, because the carry moves both curves equally. Measured
/// 2026-09-24 on the biped with the face posed and Main turned ry=35: the
/// nine carried head wires put 1.0257 units of non-rigid residual into
/// hair_geo, and with the carry applied the rig is rigid to 2e-5 -- the
/// floor every other mover leaves.
///
/// Same guard rule as the cluster: called when a carry was NAMED, never
/// because it is identity, so a rig naming nothing is bit for bit what it
/// was. A named carry already holds the master's scale, so the scale-only
/// correction an uncarried posed wire gets (its displacement multiplied by
/// the measuring space's scale) is NOT applied beside it; the two never
/// compose.
inline void
RigExecCarryWireCurves(std::vector<GfVec3f> *rest,
                       std::vector<GfVec3f> *posed,
                       const GfMatrix4d &carry)
{
    for (GfVec3f &p : *rest) {
        p = GfVec3f(carry.TransformAffine(GfVec3d(p)));
    }
    for (GfVec3f &p : *posed) {
        p = GfVec3f(carry.TransformAffine(GfVec3d(p)));
    }
}

/// How a transform-driven wire measures its drivers.
///
/// \p posedPoints is rigExec:pointFrame == "posed" (the wire runs after a
/// skin on its target), \p posedDelta is rigExec:driverDeltaFrame ==
/// "posed" (the offset is applied in the space's current frame), and
/// \p carry is rigExec:space's computeMatrix, or null when none is named.
struct RigExecWireDriverFrame {
    bool posedPoints = false;
    bool posedDelta = false;
    const GfMatrix4d *carry = nullptr;
};

/// One driver's offset from its space, as a wire applies it.
///
/// "local" is RigExecMeasureInSpace and "posed" RigExecMeasureInPosedSpace.
/// When either frame option is posed and the space carries a scale, both
/// matrices are unscaled first so the offset is measured in the asset's own
/// units; \p spaceScale receives that scale for the posed-point correction.
/// A wire asking for neither keeps the plain measurement bit for bit. The
/// runtime twin is the `measured` lambda in RrGeoAssembleWire.
inline GfMatrix4d
RigExecMeasureWireDriver(GfMatrix4d transform, GfMatrix4d space,
                         const RigExecWireDriverFrame &frame,
                         GfVec3d *spaceScale)
{
    const GfVec3d k = RigExecFrameScale(space);
    if (spaceScale) {
        *spaceScale = k;
    }
    if ((frame.posedPoints || frame.posedDelta) &&
        k != GfVec3d(1.0, 1.0, 1.0)) {
        GfMatrix4d unscale(1.0);
        unscale.SetScale(GfVec3d(1.0 / k[0], 1.0 / k[1], 1.0 / k[2]));
        transform = transform * unscale;
        space = space * unscale;
    }
    return frame.posedDelta ? RigExecMeasureInPosedSpace(transform, space)
                            : RigExecMeasureInSpace(transform, space);
}

/// Poses a transform-driven wire's control polygon from its provider table.
///
/// \p table holds the driver transforms, then their spaces, then the base
/// transforms, then their spaces (counts \p t, \p s, \p bt, and the rest).
/// \p restPoints is the authored polygon on input and the base-moved rest
/// polygon on output; \p auxPoints receives the posed polygon. A posed wire
/// with no carry has its displacement multiplied by the space's scale; one
/// with a carry has both polygons carried instead (RigExecCarryWireCurves).
/// Shared by the live assembler, the frozen replay and nothing else, so the
/// two USD-side paths cannot drift; the runtime restates it.
inline void
RigExecPoseWireDrivers(const std::vector<GfMatrix4d> &table, size_t t,
                       size_t s, size_t bt, const VtFloatArray &weights,
                       const VtFloatArray &baseWeights,
                       const RigExecWireDriverFrame &frame,
                       std::vector<GfVec3f> *restPoints,
                       std::vector<GfVec3f> *auxPoints)
{
    const size_t bs = table.size() - t - s - bt;
    const auto pick = [](size_t count, size_t j) {
        return count <= 1 ? size_t(0) : j % count;
    };
    const auto measured = [&](size_t first, size_t count, size_t spaceFirst,
                              size_t spaceCount, size_t j,
                              GfVec3d *spaceScale) {
        GfMatrix4d m = table[first + pick(count, j)];
        if (spaceCount > 0) {
            m = RigExecMeasureWireDriver(
                m, table[spaceFirst + pick(spaceCount, j)], frame,
                spaceScale);
        }
        return m;
    };
    const bool carried = frame.posedPoints && frame.carry;
    auxPoints->resize(restPoints->size());
    for (size_t j = 0; j < restPoints->size(); ++j) {
        GfVec3f &rest = (*restPoints)[j];
        // A base motion moves the curve AND its rest: the wire then deforms
        // by the driver's motion on top of it.
        if (bt > 0) {
            const GfMatrix4d b = measured(t + s, bt, t + s + bt, bs, j,
                                          nullptr);
            const float wb = baseWeights.empty()
                ? 1.0f : baseWeights[pick(baseWeights.size(), j)];
            const GfVec3f moved(b.TransformAffine(GfVec3d(rest)));
            rest = rest + (moved - rest) * wb;
        }
        GfVec3d scale(1.0, 1.0, 1.0);
        const GfMatrix4d m = measured(0, t, t, s, j, &scale);
        const float w =
            weights.empty() ? 1.0f : weights[pick(weights.size(), j)];
        const GfVec3f moved(m.TransformAffine(GfVec3d(rest)));
        GfVec3f displacement = (moved - rest) * w;
        if (frame.posedPoints && !carried) {
            displacement = GfVec3f(displacement[0] * float(scale[0]),
                                   displacement[1] * float(scale[1]),
                                   displacement[2] * float(scale[2]));
        }
        (*auxPoints)[j] = rest + displacement;
    }
    if (carried) {
        RigExecCarryWireCurves(restPoints, auxPoints, *frame.carry);
    }
}

/// Whether a wire applies its envelope itself: a valid sparse field with a
/// zero default, where only the named points are worth evaluating.
inline bool
RigExecWireTakesSparseEnvelope(const RigExecWeightPacket &w)
{
    return w.valid && w.representation == "sparse" &&
           w.defaultWeight == 0.0f && w.indices.size() == w.values.size() &&
           (w.rangePolicy.IsEmpty() || w.rangePolicy == "strict" ||
            w.rangePolicy == "clamp");
}

bool RigExecApplyMatrixKernel(const RigExecMoverParameters &p,
                              std::vector<GfVec3f> *pts);

/// Applies the skin operation of \p p to \p pts in place, returning false
/// when the packet fails atomically (cardinality mismatch, unknown method).
///
/// The envelope is NOT applied here: the caller blends the result against the
/// preceding revision, because that is where the "apply once" rule lives.
///
/// Shared by the mover-graph revision node and by the baked program, which
/// runs the same operation with no VdfNetwork around it.
bool RigExecApplySkinKernel(const RigExecMoverParameters &p,
                            std::vector<GfVec3f> *pts);
/// One vertex range of the matrix operation, against an envelope the caller
/// already resolved at the FULL point count.
///
/// Peer of the skin range form below and there for the same reason: the
/// envelope resolves atomically over the whole array (a cardinality mismatch
/// fails the application before any point is written), so it cannot be
/// resolved per range -- a chunked caller resolves it once and hands the same
/// array to every range, indexed absolutely.
void RigExecApplyMatrixKernelRange(const RigExecMoverParameters &p,
                                   const float *envelope,
                                   size_t begin, size_t end, GfVec3f *pts);

/// Blends \p blended over \p preceding for one vertex range, with
/// \p envelope the FULL resolved envelope and every array indexed
/// absolutely.
///
/// The "apply once" blend of RigExecRunRevisionKernel, as a range: per point
/// it reads two arrays and writes a third at the same index, so a range is an
/// independent sub-problem and splitting it changes nothing about the
/// arithmetic. The envelope is passed in already resolved because resolving
/// it is the whole-array decision the range form may not repeat.
void RigExecBlendEnvelopeRange(const GfVec3f *preceding, const float *envelope,
                               size_t begin, size_t end, GfVec3f *blended);

/// The same blend over the WHOLE array, split across threads the way
/// RigExecRunRevisionKernel splits it.
///
/// One definition of "which threshold and which grain the blend uses", so a
/// caller that owns the blend itself -- the baked program's revision step,
/// which resolved the envelope in an earlier step -- cannot end up threading
/// it differently from the mover-graph node beside it. Values are unaffected
/// either way: the blend reads and writes index i and nothing else.
void RigExecBlendEnvelopeAll(const GfVec3f *preceding, const float *envelope,
                             size_t count, GfVec3f *blended);

/// One influence split into a stretch and a unit dual quaternion; the
/// dual-quaternion skinning path blends these rather than the matrices.
/// Named here only as a pointer, so dualQuat.h stays out of every
/// translation unit that assembles a packet.
struct RigExecScaledDualQuat;

/// The influence tables one skin range reads.
///
/// The matrices themselves, plus the two forms the kernels want them in: the
/// float rows the SIMD linear-blend path loads and the split the
/// dual-quaternion path blends. Both are pure per-matrix functions of
/// `transforms`, so a caller that skins several ranges against one table
/// builds them once and hands them to every range; null means "derive it
/// here", which is what the full-range kernel passes.
struct RigExecSkinTransformsView {
    const GfMatrix4d *transforms = nullptr;
    size_t transformCount = 0;
    /// transformCount * RigExecSkinRowStride floats, or null.
    const float *rows = nullptr;
    /// transformCount + 1 entries (the last one the weight complement's
    /// identity), or null.
    const RigExecScaledDualQuat *palette = nullptr;
    size_t paletteSize = 0;
};

/// The view of \p p's own influence table, which is what an unchunked caller
/// skins against.
RigExecSkinTransformsView RigExecSkinTransformsOf(
    const RigExecMoverParameters &p);

/// The layout \p p and \p transforms describe over \p pointCount points.
///
/// One definition of "where are the indices, the weights and the element
/// size", because the answer depends on whether the packet carries an
/// epoch-fixed layout by handle, and a second copy of that rule is a second
/// chance to read the wrong array.
RigExecSkinLayout RigExecSkinLayoutForPacket(
    const RigExecMoverParameters &p,
    const RigExecSkinTransformsView &transforms,
    size_t pointCount);

/// The half of RigExecApplySkinKernel's whole-array validation that does NOT
/// depend on the influence matrices: the element shape, the index range and
/// the weights, against \p pointCount points.
///
/// Split out because the two halves are decided in different places once a
/// revision is chunked -- the layout is static for the frame and the matrices
/// are the last thing the pose walk produces -- and because ANDing the two
/// gives exactly the boolean the unsplit check gives.
bool RigExecSkinLayoutIsUsable(const RigExecMoverParameters &p,
                               size_t pointCount);

/// The other half: every influence matrix finite and affine.
bool RigExecSkinTransformsAreUsable(const GfMatrix4d *transforms,
                                    size_t count);

/// One vertex range of the skin operation of \p p, against \p transforms,
/// written in place over [\p begin, \p end) of \p pts.
///
/// The per-vertex body, and nothing else: NO validation -- the caller has
/// done it, whole-array, because every check the skin kernel makes is a
/// statement about the whole array -- and NO WorkParallelForN, so a range is
/// unconditionally serial and a caller that already split the work does not
/// split it again. Returns false only for a method neither kernel owns.
///
/// This is the ONE definition of the per-vertex skin body: the full-range
/// RigExecApplySkinKernel validates and then calls this inside its own
/// parallel loop, so a chunked caller and an unchunked one cannot deform a
/// vertex differently.
bool RigExecApplySkinKernelRange(const RigExecMoverParameters &p,
                                 const RigExecSkinTransformsView &transforms,
                                 size_t begin, size_t end,
                                 std::vector<GfVec3f> *pts);

/// RigExecApplySkinKernel against an influence table other than the packet's
/// own, for a caller that folds the matrices outside the packet.
bool RigExecApplySkinKernelWithTransforms(
    const RigExecMoverParameters &p,
    const RigExecSkinTransformsView &transforms,
    std::vector<GfVec3f> *pts);


/// Applies the blend-shape operation of \p p to \p pts in place, returning
/// false when the packet fails atomically (the deltas or the envelope do not
/// resolve to the point count, or the surface-frame transport fails).
///
/// The envelope is resolved and applied INSIDE the kernel, like the matrix
/// kernel and unlike the point3f[] ops: the deltas are added to the preceding
/// revision and blended back against it in one pass.
///
/// ONE definition, called by the mover-graph revision node and by the baked
/// program, which runs the same operation with no VdfNetwork around it: a
/// second copy of the blend would have to agree about where the envelope is
/// folded in, and the rigs that would show a disagreement are the ones no
/// fixture happened to have.
bool RigExecApplyBlendShapeKernel(const RigExecMoverParameters &p,
                                  std::vector<GfVec3f> *pts);

/// Recomputes the derived property \p op maintains -- vertex normals or an
/// extent -- from \p p and blends it over \p pts in place, returning false
/// when the packet fails atomically (nothing computed, or a cardinality the
/// authored property cannot hold).
///
/// The envelope is resolved and applied INSIDE the kernel, as for the matrix
/// and blend-shape kernels.
///
/// ONE definition, called by the mover-graph revision node and by the baked
/// program, which maintains the same property with no VdfNetwork around it:
/// two copies would have to keep agreeing about the cardinality rules that
/// decide when a recomputation fails instead of truncating.
bool RigExecApplyDerivedKernel(RigExecRevisionOp op,
                               const RigExecMoverParameters &p,
                               std::vector<GfVec3f> *pts);

/// Applies \p op to \p pts in place, returning false when the packet fails
/// atomically.
///
/// The envelope is NOT applied here for the operations that take a separate
/// blend: RigExecRunRevisionKernel below is where the "apply once" rule
/// lives. Matrix, blendShape and the two derived recomputations fold the
/// envelope into their own arithmetic and are routed to the kernels above.
///
/// ONE definition, called by the mover-graph revision node and by the baked
/// program: a second copy of a deformation agrees on the fixtures that exist
/// and drifts on the ones that do not.
bool RigExecApplyRevisionKernel(RigExecRevisionOp op,
                                const RigExecMoverParameters &p,
                                std::vector<GfVec3f> *pts);

/// Runs one revision of \p op over \p pts in place, envelope included: the
/// packet check, the full-strength fast path, RigExecApplyRevisionKernel and
/// the "apply once" blend against the preceding revision. Returns false when
/// the revision must pass its preceding value through unchanged.
///
/// ONE definition of "apply once", called by the mover-graph revision node --
/// which is then only scratch-collect / call / write-back -- and by the baked
/// geometry loop. Two hand-written wrappers would have to agree about which
/// operations blend and which fold the envelope into their own arithmetic.
bool RigExecRunRevisionKernel(RigExecRevisionOp op,
                              const RigExecMoverParameters &p,
                              std::vector<GfVec3f> *pts);

/// Whether \p envelope makes the "apply once" blend the identity, so the
/// copy of the preceding revision, the resolved envelope array and the blend
/// loop are all dead work.
///
/// RigExecBlendEnvelope's `weight >= 1` branch returns the candidate itself,
/// with no arithmetic, and RigExecWeightPacket::ResolveAll can only answer 1
/// for every element of a constant packet carrying no values, no indices and
/// a range policy it accepts. This is the packet EVERY unweighted mover gets,
/// because it is what inputs:defaultWeight synthesizes.
///
/// ONE definition, called by the mover-graph revision node and by the baked
/// geometry loop: two hand-copied predicates could disagree about when the
/// skip is safe, and the only rigs that would show it are the ones no
/// fixture happened to have (a partial constant envelope on a skin mover).
inline bool
RigExecEnvelopeIsFullStrength(const RigExecWeightPacket &envelope)
{
    return envelope.valid && envelope.representation == "constant" &&
           envelope.values.empty() && envelope.indices.empty() &&
           envelope.defaultWeight == 1.0f &&
           (envelope.rangePolicy.IsEmpty() ||
            envelope.rangePolicy == "strict" ||
            envelope.rangePolicy == "clamp");
}

/// Provider results a revision needs that only evaluation can supply.
///
/// Everything else an assembler needs is a static read off the authored stage
/// through the revision binding. These are the dynamic ones: results of
/// computations on authored prims, pulled through a tap set on that same stage.
/// A null/empty member means the provider produced nothing and normally fails
/// the application rather than substituting a default (spec §6.6). The one
/// deliberate exception is weights: null means no weight object was bound,
/// so inputs:defaultWeight supplies the common envelope.
struct RigExecProviderValues {
    const GfMatrix4d *transform = nullptr;          ///< computeMatrix
    /// computeMatrix per binding.influences entry, in that order (skin).
    const std::vector<GfMatrix4d> *influenceTransforms = nullptr;
    /// computeMatrix of binding.carrySpace (rigExec:space), read at the
    /// revision's declared phase, or null when the mover named none. A
    /// POINTER because "named" is the guard, never "identity": a
    /// transform-driven wire whose points are posed carries both control
    /// polygons by it (RigExecCarryWireCurves), and a rig naming nothing
    /// must take the untouched branch.
    const GfMatrix4d *carry = nullptr;
    /// Bound computeWeightPacket, or null to use inputs:defaultWeight.
    const RigExecWeightPacket *weights = nullptr;
    const RigExecPointFrameArray *driverFrames = nullptr;
    std::vector<GfVec3f> basePoints;   ///< authored base of the target
    std::vector<GfVec3f> blendDeltas;  ///< summed channel deltas
    /// Cache for a skin mover's epoch-fixed per-point layout. Null re-reads
    /// and re-validates the arrays every call, which is what a layout that
    /// is animated, connected, or written by a property chain requires.
    RigExecSkinTopologyCache *skinTopologyCache = nullptr;
    /// A layout the CALLER already resolved, for a caller that may not take
    /// the cache's lock where it assembles. Set -- even to a shared_ptr
    /// holding null, which is a remembered refusal -- it is used and
    /// `skinTopologyCache` is not consulted at all.
    const std::shared_ptr<const RigExecSkinTopology> *skinTopology = nullptr;
    /// Values already resolved this generation, preferred by every static
    /// read the assembler makes. Null reads the stage throughout, which is
    /// what a rig with no property chains wants and what a test may pass.
    const RigExecResolvedInputs *resolved = nullptr;
};

/// The provider frames a surface projector reads, as ASSET frames: each
/// provider's rest frame times its base or final computeMatrix. Index 0 is
/// binding.transform (the source), 1 binding.transformSpace (the source's
/// sibling space), 2 binding.carrySpace (the rig's space). `named` says the
/// binding names the provider; `resolved` says its frames were read.
struct RigExecSurfaceProjectorFrames {
    bool named[3] = {false, false, false};
    bool resolved[3] = {false, false, false};
    GfMatrix4d base[3] = {GfMatrix4d(1.0), GfMatrix4d(1.0), GfMatrix4d(1.0)};
    GfMatrix4d final[3] = {GfMatrix4d(1.0), GfMatrix4d(1.0), GfMatrix4d(1.0)};
};

/// A provider's asset frame from its rest landmarks and a rest->pose map:
/// row-vector frame = rest * M. Identity rest landmarks give M itself.
GfMatrix4d RigExecWorldFromRest(const std::array<GfVec3d, 4> &restPoints,
                                const GfMatrix4d &restToPose);

/// What a surface projector target reads besides frames and points: its
/// settings, its dials and its surface's topology. Gathered from the stage
/// by RigExecReadProjectorTarget and from samples by the frozen replay, so
/// both hand RigExecRunProjectorTarget the same values.
struct RigExecProjectorReads {
    GfVec3d rayOrigin{0.0, 0.0, 0.0};
    GfVec3d rayDirection{0.0, 0.0, 1.0};
    GfVec3d rayUp{0.0, 1.0, 0.0};
    GfMatrix4d shaderOffset{1.0};
    bool reproject = false;
    std::vector<double> dials;
    std::vector<int> faceVertexCounts;
    std::vector<int> faceVertexIndices;
};

/// Reads (and records, for the bake) what \p op needs off the stage,
/// through the generation's resolved inputs where the live assemblers do.
void RigExecReadProjectorTarget(
    const UsdPrim &projectorPrim, RigExecRevisionOp op,
    const RigExecRevisionBinding &binding,
    const RigExecResolvedInputs *resolved, UsdTimeCode time,
    RigExecProjectorReads *reads);

/// Runs a surface projector target (SurfaceProjector or ShaderDials) on its
/// gathered reads: the shared kernel on \p finalPoints against the authored
/// \p basePoints. False, with diagnostics, when no matrix is published.
bool RigExecRunProjectorTarget(
    RigExecRevisionOp op, const RigExecRevisionBinding &binding,
    const RigExecSurfaceProjectorFrames &frames,
    const RigExecProjectorReads &reads,
    const std::vector<GfVec3f> &basePoints,
    const std::vector<GfVec3f> &finalPoints, GfMatrix4d *matrix,
    std::vector<std::string> *diagnostics);

/// RigExecReadProjectorTarget then RigExecRunProjectorTarget: what the
/// dynamic walk and the baked program both call.
bool RigExecEvaluateProjectorTarget(
    const UsdPrim &projectorPrim, RigExecRevisionOp op,
    const RigExecRevisionBinding &binding,
    const RigExecSurfaceProjectorFrames &frames,
    const std::vector<GfVec3f> &basePoints,
    const std::vector<GfVec3f> &finalPoints,
    const RigExecResolvedInputs *resolved, UsdTimeCode time,
    GfMatrix4d *matrix, std::vector<std::string> *diagnostics);

/// Sums blend channels into dense per-point deltas against \p base
/// (spec §7.3): deltas derive against the authored base, never the preceding
/// revision, and each channel's weight is clamped to [0, lastActivation] then
/// interpolated between the bracketing samples.
///
/// Shared by the mover-owned computeMoverParameters kernel and by
/// RigExecRigEvaluator, which assembles the same packet from tapped channels
/// with no derived stage. One definition, so the two cannot drift.
///
/// Returns false on a structural error -- a channel with no samples, a
/// non-finite weight, a non-positive or repeated activation, or a sample whose
/// point count disagrees with \p base -- which fails the mover atomically
/// rather than applying a partial blend.
bool RigExecSumBlendChannels(
    const std::vector<RigExecBlendChannel> &channels,
    const std::vector<GfVec3f> &base,
    std::vector<GfVec3f> *deltas);

/// Assembles any revision's parameter packet without a derived stage.
///
/// Peer of the _Build*MoverParameters family in movers/. Static inputs
/// (strength, divisions, mode, topology, cage and bind arrays) are read from the
/// authored stage through \p binding; dynamic ones arrive in \p values.
/// \p time is the evaluation time for every static scene read the packet
/// needs (cage, surface, topology, bind coords, strength, enable). The kernels
/// read the same inputs through exec at the current time, so passing anything
/// else silently diverges on animated input.
RigExecMoverParameters RigExecAssembleParameters(
    const UsdPrim &moverPrim,
    RigExecRevisionOp op,
    const RigExecRevisionBinding &binding,
    const RigExecProviderValues &values,
    UsdTimeCode time = UsdTimeCode::Default());

/// What an external mover's assembleExternal answered: the plugin's opaque
/// payload, compared by its own operator==.
struct RigExecExternalPayload {
    /// The mover's schema type, which names its handler.
    TfToken schema;
    /// A handler with an assembleExternal was found and it returned true.
    bool valid = false;
    VtValue data;

    bool operator==(const RigExecExternalPayload &o) const {
        return schema == o.schema && valid == o.valid && data == o.data;
    }
};

/// The External arm's plugin call, shared by RigExecAssembleParameters and
/// the baked prologue: finds the handler for the prim's schema and asks it
/// for the payload at \p time. Reads the stage: owning thread only.
void RigExecAssembleExternalPayload(const UsdPrim &moverPrim,
                                    const RigExecRevisionBinding &binding,
                                    const RigExecProviderValues &values,
                                    UsdTimeCode time,
                                    RigExecExternalPayload *payload);

/// What a revision leaf holds. Arrays are VtArrays, shared, not copied.
enum class RigExecRevisionLeafType : uint8_t {
    Bool,
    Int,
    Float,
    Token,
    IntArray,
    FloatArray,
    Vec2fArray,
    Vec3fArray,
    Vec3i,
    Vec3d,
    Matrix4d,
    DoubleArray,
    /// A shader dial as a double: a float attribute read as a float and
    /// widened, any other read as a double (RigExecReadProjectorTarget).
    Dial,
};

/// When a revision leaf is read: at the evaluated time, or at Default (an
/// authored rest value).
enum class RigExecRevisionLeafTime : uint8_t { AtTime, AtDefault };

/// How a revision leaf is read; each is one read site of the stage
/// assemblers, restated exactly (RigExecSampleRevisionLeaf).
enum class RigExecRevisionLeafFlavour : uint8_t {
    /// UsdAttribute::Get over the fallback (_Token, _RecordedToken).
    Raw,
    /// RigExecResolvedInputs::GetAttribute, then Raw (_Read, _Enabled).
    Resolved,
    /// RigExecResolvedInputs::GetAttribute alone, over the fallback (the
    /// blend gather, RevisionStatic's defaultWeight, the weight gathers).
    ResolvedOnly,
    /// RigExecResolvedInputs::Get at the exact path, then Raw (_Array).
    OverlayThenRaw,
    /// Whether the resolved inputs hold the exact path, as a Bool.
    Present,
};

/// The reads RigExecAssembleFromLeaves takes, one per read site of
/// RigExecAssembleParameters for the operations it covers.
enum class RigExecRevisionLeafRole : uint8_t {
    Enabled,
    DefaultWeight,
    SkinningMethod,
    ElementSize,
    JointIndices,
    JointWeights,
    WeightBlend,
    PointFrame,
    DeltaSpace,
    TopologyCounts,
    TopologyIndices,
    SurfacePoints,
    BindCoords,
    Widths,
    /// deltaMush and wrinkle: inputs:restPoints at Default; wrinkle's
    /// inputs:topology and inputs:pinPoints, both at Default.
    RestPoints,
    WrinkleTopology,
    PinPoints,
    /// Lattice: the cage at Default past the overlay, the cage at the time
    /// through it, and rigExec:divisions.
    RestCage,
    LiveCage,
    Divisions,
    /// Wire: the driver curve at Default and at the time, its order and
    /// knots, the driver weight arrays, rigExec:driverDeltaFrame (the wire's
    /// rigExec:pointFrame is PointFrame) and inputs:dropoffDistance, which
    /// answers empty where no attribute stands.
    CurveRest,
    CurveLive,
    CurveOrder,
    CurveKnots,
    DriverWeights,
    DriverBaseWeights,
    DeltaFrame,
    Dropoff,
    /// A surface projector's settings.
    RayOrigin,
    RayDirection,
    RayUp,
    ShaderOffset,
    ProjectionMode,
    Count,
};

/// One read a revision's assembly makes: the path, the value type, the time
/// policy and the read site's flavour, plus what the site answers when the
/// read finds nothing. A Raw key with an empty fallback answers empty where
/// no attribute stands (a site that reads only an attribute that exists).
struct RigExecRevisionLeafKey {
    SdfPath path;
    RigExecRevisionLeafType type = RigExecRevisionLeafType::Float;
    RigExecRevisionLeafTime time = RigExecRevisionLeafTime::AtTime;
    RigExecRevisionLeafFlavour flavour = RigExecRevisionLeafFlavour::Raw;
    VtValue fallback;
};

/// The reads of one revision (or weight object), in declaration order, and
/// which of them answers each role.
struct RigExecRevisionLeafDecl {
    std::vector<RigExecRevisionLeafKey> keys;
    /// Key index per RigExecRevisionLeafRole, or -1 when the operation does
    /// not read it (an empty binding path reads an empty array).
    std::array<int, size_t(RigExecRevisionLeafRole::Count)> roles;
    /// The first of a deltaMush's or wrinkle's scalar inputs, declared in
    /// the stage assembler's order, or -1.
    int scalarBegin = -1;
    /// The first of a ShaderDials target's dials, one key per
    /// binding.shaderDials entry in its order, or -1.
    int dialBegin = -1;
    /// RigExecAssembleFromLeaves covers the operation.
    bool assembles = false;

    RigExecRevisionLeafDecl() { roles.fill(-1); }
    int Add(RigExecRevisionLeafKey key) {
        keys.push_back(std::move(key));
        return int(keys.size()) - 1;
    }
    int Role(RigExecRevisionLeafRole role) const {
        return roles[size_t(role)];
    }
};

/// Whether RigExecAssembleFromLeaves covers \p op: every operation. A
/// projector's matrix targets read through
/// RigExecReadProjectorTargetFromLeaves, and an external mover's payload is
/// a leaf the caller samples (RigExecRevisionLeafView::external).
bool RigExecRevisionOpAssemblesFromLeaves(RigExecRevisionOp op);

/// Declares every read RigExecAssembleParameters can make for \p op on the
/// mover at \p moverPath -- a superset of what one call reads, whichever
/// branch its values take -- with the read site's flavour, time policy and
/// fallback. Declares nothing when \p op is not covered. Stage-free.
void RigExecDeclareRevisionLeaves(RigExecRevisionOp op,
                                  const SdfPath &moverPath,
                                  const RigExecRevisionBinding &binding,
                                  RigExecRevisionLeafDecl *decl);

/// The value \p key's read site answers at \p time (Default for an
/// AtDefault key) through \p resolved, or the key's fallback when it finds
/// nothing. \p attribute is the attribute at the key's path, invalid when
/// none stands there. Reads the stage: owning thread only.
VtValue RigExecSampleRevisionLeaf(const RigExecRevisionLeafKey &key,
                                  const UsdAttribute &attribute,
                                  const RigExecResolvedInputs *resolved,
                                  UsdTimeCode time);

/// The attributes \p key's read can reach from \p attribute: the attribute
/// itself, and for a connection-following flavour every hop of the
/// single-connection walk RigExecResolvedInputs::GetAttribute takes. Sets
/// \p varying when the read is at the time and some hop's value might vary
/// with it. Reads the stage: owning thread only.
void RigExecRevisionLeafHops(const RigExecRevisionLeafKey &key,
                             const UsdAttribute &attribute,
                             std::vector<SdfPath> *hops, bool *varying);

/// Sampled revision leaves as RigExecAssembleFromLeaves reads them.
struct RigExecRevisionLeafView {
    const RigExecRevisionLeafDecl *decl = nullptr;
    /// One value per key of \p decl.
    const std::vector<VtValue> *values = nullptr;
    /// The revision's phase overlay (the generation's resolved inputs plus
    /// the points its phased reads resolved to this run), or null. A points
    /// read takes the overlay's array at its path before the leaf, as the
    /// stage assembler reads through that overlay. The overlay adds only
    /// point arrays, so no other read can see it.
    const RigExecResolvedInputs *phased = nullptr;
    /// When set, every scalar role the operation reads that \p decl does not
    /// declare is appended by attribute name (a test's report); the read
    /// answers the site's fallback.
    std::vector<std::string> *missing = nullptr;
    /// An External revision's payload leaf, sampled where the plugin may
    /// read the stage. Null answers an invalid packet.
    const RigExecExternalPayload *external = nullptr;
};

/// RigExecAssembleParameters for a covered operation, from sampled leaves:
/// the same arms, gates and order, with every stage read replaced by its
/// leaf. Reads no stage, takes no lock and builds no token from text.
/// \p values carries the provider values as for the stage assembler; its
/// `resolved` and `skinTopologyCache` are not read.
RigExecMoverParameters RigExecAssembleFromLeaves(
    RigExecRevisionOp op, const RigExecRevisionBinding &binding,
    const RigExecRevisionLeafView &leaves,
    const RigExecProviderValues &values);

/// Whether RigExecAssembleFromLeaves reaches an External revision's payload
/// over \p leaves and \p values: the enable and the envelope, which the
/// stage assembler checks before it calls the plugin.
bool RigExecExternalPayloadIsRead(const RigExecRevisionLeafView &leaves,
                                  const RigExecProviderValues &values);

/// RigExecReadProjectorTarget over sampled leaves: the same reads, each
/// answered by its leaf. The caller holds a valid projector prim (the
/// program refuses a missing one at Build). Reads no stage.
void RigExecReadProjectorTargetFromLeaves(
    RigExecRevisionOp op, const RigExecRevisionBinding &binding,
    const RigExecRevisionLeafView &leaves, RigExecProjectorReads *reads);

/// A compiled mover graph.
///
/// Build order mirrors the composed mover-stack walk: descendants before their
/// mover parent, sibling branches bottom-to-top (reverse composed child order).
/// Seed a target's chain with its authored base points, then append one revision
/// per mover that writes it. Each append returns the new chain head, which is
/// the input to the next revision and, at the end, the published result.
class RigExecMoverGraph
{
public:
    RigExecMoverGraph();
    ~RigExecMoverGraph();

    RigExecMoverGraph(const RigExecMoverGraph &) = delete;
    RigExecMoverGraph &operator=(const RigExecMoverGraph &) = delete;

    /// Seeds a chain with the authored base points of \p target.
    VdfMaskedOutput AddPointSource(
        const SdfPath &target, const VtVec3fArray &points);

    /// Appends one revision reading \p previous.
    ///
    /// \p parameters and \p status are the mover's own resolved packet and
    /// status. They arrive as values rather than as scene lookups because the
    /// resolution that used to be authored as rigExec:resolved* relationships
    /// is just a build-time choice of which output to read.
    VdfMaskedOutput AddRevision(
        RigExecRevisionOp op,
        const VdfMaskedOutput &previous,
        const RigExecMoverParameters &parameters,
        const RigExecMoverStatus &status);

    /// Updates an existing source without changing graph topology. Returns
    /// false for an unknown source or a changed point count; cardinality
    /// changes require rebuilding that target's chain. Equal values stay clean.
    bool UpdatePointSource(
        const VdfMaskedOutput &source, const VtVec3fArray &points);

    /// Updates a revision's packet and status in place. Returns false for an
    /// unknown revision. Only changed inputs and their dependents are dirtied.
    bool UpdateRevision(
        const VdfMaskedOutput &revision,
        const RigExecMoverParameters &parameters,
        const RigExecMoverStatus &status);

    /// Splices a retained revision into a new chain. Its operation and point
    /// cardinality stay fixed; only that revision and its dependents are dirty.
    bool ReconnectRevision(const VdfMaskedOutput &revision,
                           const VdfMaskedOutput &previous);

    /// Removes an obsolete revision after surviving consumers are reconnected.
    bool RemoveRevision(const VdfMaskedOutput &revision);

    /// Evaluates \p output and returns its points. The schedule and executor
    /// persist between calls, including intermediate revision values, so an
    /// edit only executes the affected suffix and an unchanged call is cached.
    VtVec3fArray Evaluate(const VdfMaskedOutput &output) const;

    /// Actual cached execution status, including kernel-time rejection.
    /// Evaluate the revision or a downstream output before querying it.
    RigExecMoverStatus GetRevisionStatus(const VdfMaskedOutput &revision) const;

    /// Number of revision nodes currently in the graph (excludes sources).
    size_t GetRevisionCount() const { return _revisionCount; }

    /// Cumulative counters for inspecting incremental execution behavior.
    size_t GetRevisionExecutionCount() const;
    size_t GetScheduleBuildCount() const;

    const VdfNetwork &GetNetwork() const { return _network; }

private:
    struct _Runtime;
    VdfNetwork _network;
    size_t _revisionCount = 0;
    std::unique_ptr<_Runtime> _runtime;
};

}  // namespace rigExec

#endif  // RIGEXEC_MOVER_GRAPH_H
