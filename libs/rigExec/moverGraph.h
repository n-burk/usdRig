//
// RigExec compiled mover graph (spec §7.2).
//
// The per-(mover, target) revision chain is a VdfNetwork built in memory from
// the relationships already authored on the user's stage. Nothing is authored
// anywhere to express it: no generated prims, no compiler, no derived stage, and
// no schema types for the revisions themselves. Compiled nodes live only on the
// graph side.
//
// This is what the write-set authoring model always meant. A mover says "I
// write this target" and its namespace position says "in this order"; the chain
// of revisions that implies is dataflow, and dataflow is what a VdfNetwork is
// for. Materializing it as USD prims required a second stage to hold them, cost
// a recomposition, and put engine machinery on the authoring surface -- while
// still not being able to express the optimizations the graph form makes
// natural (splitting one revision across face sets, cloning legs, per-element
// masks driving sparse recomputation).
//
#ifndef RIGEXEC_MOVER_GRAPH_H
#define RIGEXEC_MOVER_GRAPH_H

#include "types.h"

#include "rigExecMath/profileMover.h"
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

#include <atomic>
#include <functional>
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
    EmitGuidePoints,
    Curvenet,
    CurvenetAdjuster,
    RecomputeNormals,
    RecomputeExtent,
};

/// When in the walk a side input takes its value from.
///
/// A mover reads things other movers write. Which REVISION of those things it
/// gets is a separate authored choice from which path it reads, and the two
/// were never expressible together: a read phase had to be a schema attribute
/// named for one specific role (rigExec:cageReadPhase, rigExec:surfaceRead-
/// Phase, ...), so every new input needed a new attribute and an input with no
/// attribute of its own had no way to say anything at all.
///
/// Declaring it as metadata ON the relationship or attribute that names the
/// input puts the phase where the binding is, so any input can carry one and
/// nothing has to be added to a schema to introduce another.
enum class RigExecReadPhaseKind {
    Base,       ///< the authored value: what the stage resolves at this time
    Preceding,  ///< the value immediately before the reading mover
    Final,      ///< the value after every mover that writes it has run
    AtPrim,     ///< the value as of when the walk finished with a named prim
};

/// A resolved read phase. \p prim is meaningful only for AtPrim.
///
/// AtPrim is the general form the other three are shorthands for: the walk is
/// reverse-sibling post-order over the composed Movers namespace, so "as of
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

/// The read phase declared for \p property, if any.
///
/// Metadata first, then \p legacyAttribute on the property's own prim (the
/// rigExec:<role>ReadPhase attributes, still honored so existing assets keep
/// working), then Base. Returns false and fills \p error on an unparseable
/// authored value; an absent declaration is Base and true.
bool RigExecResolveReadPhase(
    const UsdObject &property,
    const char *legacyAttribute,
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
///   * invalidation: the owner clears the whole cache on every stage notice,
///     so an edit reaches the very next read. It also clears it when
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
    template <class T>
    bool GetAttribute(
        const UsdAttribute &attribute, UsdTimeCode time, T *out) const {
        if (!out) {
            return false;
        }
        // An input that cannot change until the stage does answers from the
        // cache -- but only after the in-memory value for this exact property
        // has been ruled out, because a property chain result outranks the
        // authored value the cache holds.
        if (_cache && attribute && !_values.count(attribute.GetPath())) {
            bool handled = false;
            const bool got = _cache->Read(
                attribute, attribute.GetPath(), time, out, &handled);
            if (handled) {
                return got;
            }
        }
        std::set<SdfPath> visiting;
        std::vector<UsdAttribute> fallback;
        UsdAttribute a = attribute;
        while (a && visiting.insert(a.GetPath()).second) {
            if (Get(a.GetPath(), out)) {
                return true;
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
    std::map<SdfPath, VtValue> _values;
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
    bool operator==(const RigExecBlendSampleBinding &o) const {
        return sample == o.sample && points == o.points && phase == o.phase;
    }
};

struct RigExecRevisionBinding {
    SdfPath moverPath;        ///< the authored mover
    SdfPath target;           ///< canonical exact write target
    SdfPath transform;        ///< computeMatrix provider (matrix)
    /// Ordered computeMatrix providers (skin): rigExec:influences, which
    /// rigExec:jointIndices index. Every entry shares transformPhase.
    std::vector<SdfPath> influences;
    SdfPath weightObject;     ///< computeWeightPacket provider
    SdfPath base;             ///< authored-base points (blend/volume/lattice)
    SdfPath topologyCounts;   ///< faceVertexCounts (smooth/surface)
    SdfPath topologyIndices;  ///< faceVertexIndices (smooth/surface)
    SdfPath cagePoints;       ///< lattice cage points
    SdfPath surfacePoints;    ///< driver surface points
    SdfPath bindCoords;       ///< ribbon bind coordinates
    SdfPath driverFrames;     ///< aggregate frame provider
    SdfPath widths;           ///< authored widths (extent maintenance)
    SdfPath curvenet;         ///< RigExecCurvenet prim (Profile Mover)
    SdfPath curvenetPoints;   ///< that curvenet's points property
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

/// Profile Mover bindings, held across frames and keyed by (mover, target).
///
/// Cutting a mesh and factorizing its Laplacian is the expensive half of the
/// technique and depends only on the layout -- the curvenet's rest points,
/// its spline indices, and the target's base geometry. Rebuilding it per
/// frame would make the mover unusable, and rebuilding it never would make an
/// edited curvenet silently stale, so the cache is keyed by a digest of
/// exactly those inputs.
///
/// A failed bind is remembered too: a rig whose curvenet cannot be bound
/// should report that once, not re-attempt the cut on every frame.
class RigExecCurvenetBindCache
{
public:
    /// Returns the binding for \p key, calling \p build when the cached
    /// digest differs. Returns null on a failed bind and fills \p error.
    std::shared_ptr<const RigExecProfileMoverBinding> Resolve(
        const SdfPath &mover, const SdfPath &target, size_t digest,
        const std::function<bool(RigExecProfileMoverBinding *, std::string *)>
            &build,
        std::string *error);

    /// Messages accumulated by binds since the last call, and clears them.
    ///
    /// Cutting a mesh is where a curvenet's real problems surface -- faces
    /// the cut lost, segments it could not route, a net floating so far off
    /// the surface that its profile means nothing. None of that is fatal, so
    /// none of it can be an error return, and a silently bad deformation is
    /// exactly the failure mode worth spending a diagnostic on.
    std::vector<std::string> TakeDiagnostics();

    void Clear() { _entries.clear(); }
    size_t GetSize() const { return _entries.size(); }

private:
    struct _Entry {
        size_t digest = 0;
        std::shared_ptr<const RigExecProfileMoverBinding> binding;
        std::string error;
    };
    std::map<std::pair<SdfPath, SdfPath>, _Entry> _entries;
    std::vector<std::string> _pending;
};

/// Per-epoch skin layouts, keyed by the mover that owns them.
///
/// Peer of RigExecCurvenetBindCache, and there for the same reason: the
/// expensive half of the operation depends only on the layout, and the
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
/// Peer of _BuildMatrixMoverParameters in moverKernels.cpp, but built from
/// values rather than from a VdfContext: \p transform and \p weights are the
/// already-evaluated results of the providers named by the revision binding,
/// pulled through a tap set on the authored stage. A null transform fails the
/// application. Null weights mean no object is bound, so the assembler reads
/// inputs:defaultWeight and synthesizes the common constant envelope.
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
/// atomically. \p controlFrames receives the curvenet adjuster's fully
/// adjusted control frames and is unread by every other operation; it may be
/// null only when \p op is not RigExecRevisionOp::CurvenetAdjuster.
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
                                std::vector<GfVec3f> *pts,
                                std::vector<GfMatrix4d> *controlFrames);

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
                              std::vector<GfVec3f> *pts,
                              std::vector<GfMatrix4d> *controlFrames);

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
    /// Bound computeWeightPacket, or null to use inputs:defaultWeight.
    const RigExecWeightPacket *weights = nullptr;
    const RigExecPointFrameArray *driverFrames = nullptr;
    std::vector<GfVec3f> basePoints;   ///< authored base of the target
    std::vector<GfVec3f> blendDeltas;  ///< summed channel deltas
    /// Posed control points of the mover's curvenet. Supplied by the
    /// evaluator, which runs curvenet chains first so a curvenet posed by
    /// ordinary movers reaches the Profile Mover already articulated; empty
    /// falls back to reading the curvenet's authored points at the time.
    std::vector<GfVec3f> curvenetPoints;
    /// Cache for the expensive half of the Profile Mover. Null binds fresh
    /// every call, which is correct but only sane in a test.
    RigExecCurvenetBindCache *curvenetCache = nullptr;
    /// A bind the CALLER already resolved, for a caller that may not touch
    /// the cache where it assembles -- RigExecCurvenetBindCache has no
    /// locking at all, so the baked program resolves it in its serial
    /// prologue and hands the answer in here. Set -- even to a shared_ptr
    /// holding null, which is a remembered failed bind -- it is used and
    /// `curvenetCache` is not consulted. Peer of `skinTopology` below, and
    /// there for the same reason.
    const std::shared_ptr<const RigExecProfileMoverBinding> *curvenetBinding =
        nullptr;
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
/// Peer of the _Build*MoverParameters family in moverKernels.cpp. Static inputs
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
    /// Cached deformation-relative control frames produced by an adjuster.
    std::vector<GfMatrix4d> GetRevisionControlFrames(const VdfMaskedOutput &revision) const;

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
