// RigExec frozen evaluation contexts: everything a background frame job may
// read, sampled up front on the UI thread.
// A worker must never touch live evaluator state, the registry mutex, or the
// USD stage. So the UI thread samples every per-frame input a job needs --
// varying attribute queries, chain-resolved inputs, interactive overrides --
// for each enqueued time at enqueue time (RigExecSampleFrameInputs), packs
// the values into a RigExecFrameInputs, and hands the worker that plus a
// RigExecFrozenEvalContext pinning the epoch. The worker evaluates from the
// vector only, into a private slot arena it owns (RigExecFrozenArena),
// running the baked serial executor end to end (never Work/TBB: a
// kernel-level parallel call would leak the work back onto the shared arena
// at normal priority, past the priority boundary).
// The context carries digests and counts, never handles: no UsdStage, no
// UsdAttribute, no UsdAttributeQuery, no evaluator pointer. Anything that
// cannot be named without one of those is sampled into the input vector
// instead. The worker-side entry point (RigExecEvaluateFrozen with a runner)
// checks the epoch pin and the generation fence, forces the serial kernel
// variants for the calling thread, and runs the supplied serial step runner
// against the private arena; anything it cannot prove bit-identical --
// unimplemented, inconsistent, stale, or cancelled -- answers with an invalid
// pose, which is the fail-closed stub the caller falls back from into live
// evaluation.
// THREADING. The context is trivially copyable plain data: safe to build on
// the UI thread, hand across threads, and hold past the stage edit that
// cancels the job it was sampled for. The input vector is likewise plain
// values once sampled, but it is built by one thread (the sampler) and read
// by one job -- concurrent mutation during a run is a caller bug, not a
// checked condition. The arena is strictly thread-confined: one arena per
// job, never shared, never moved while a run reads it. The serial scope is
// thread-local: it constrains the thread that entered it and no other.
#ifndef RIGEXEC_FROZEN_CONTEXT_H
#define RIGEXEC_FROZEN_CONTEXT_H

#include "bakedTrace.h"
#include "rigEvaluator.h"
#include "scalarReferenceAdapter.h"

#include "pxr/usd/sdf/path.h"
#include "pxr/usd/sdf/types.h"
#include "pxr/usd/usd/attribute.h"
#include "pxr/usd/usd/attributeQuery.h"
#include "pxr/usd/usd/timeCode.h"
#include "pxr/base/vt/value.h"

#include <cstddef>
#include <cstdint>
#include <functional>
#include <map>
#include <memory>
#include <optional>
#include <string>
#include <type_traits>
#include <unordered_map>
#include <vector>

PXR_NAMESPACE_USING_DIRECTIVE

namespace rigExec {

class RigExecBackgroundScheduler;
using RigExecFrameGeneration = uint64_t;

/// An epoch-pinned, worker-safe snapshot of a baked program, built on the UI
/// thread by RigExecFreezeProgram. See that function for the contract;
/// frozenProgram.h defines it.
struct RigExecFrozenProgram;

/// Burst-cache route of one sampled input. Fresh samples fold fresh;
/// static samples fold the memoized level-1 the sampler recorded when it
/// first served the value.
enum RigExecBurstSampleRoute {
    RigExecBurstRouteFresh = 0,
    RigExecBurstRouteStage = 1,
    RigExecBurstRouteResolved = 2
};

/// One sampled source value: the attribute (or resolved input) at FrameInputs
/// time, read through the same route the baked frame path reads it.
struct RigExecSampledInput {
    SdfPath path;
    VtValue value;
    /// False when the source had no value at the sampled time, which the
    /// worker must reproduce exactly rather than fill with a default.
    bool hasValue = false;
    /// True when the value came from in-memory state computed for another
    /// time rather than a fresh read at the sampled time. The samplers mark
    /// none: a read a property chain or record can answer is resolved by
    /// the worker from head-leaf samples. A vector carrying a marked sample
    /// must never warm a frame it was not proven for, because the digest
    /// would name stale values as the frame's; the runner declines it.
    bool viaChain = false;
    /// Which burst-cache memo map served this sample, when one did: the
    /// digest folds the memoized level-1 from the same route, so a path
    /// sampled both stage-direct and through the resolved inputs can never
    /// alias across the two. The plain sampler leaves every sample fresh.
    int burstSampleRoute = RigExecBurstRouteFresh;
    /// True for an explicit authored source block, distinct from missing.
    bool valueBlocked = false;
    /// The RigExecFrameInputs::staticSamples entry this sample copies
    /// verbatim, or -1. Set only by the samplers, when they serve the entry;
    /// the digests then fold the entry's memoized level-1, so a caller that
    /// rewrites a marked sample must reset this.
    int32_t staticSample = -1;
};

/// The time-invariant reads of a frozen job's provider and weight-oracle
/// leaves: each sample a fresh read appends for a leaf whose read cannot
/// move with the time (RigExecRevisionLeafHops, asked when the table is
/// built), with its level-1 digest (RigExecSampleDigest) and whether the
/// control digest folds it exactly (RigExecSampleDigestible). Read on the UI
/// thread and shared, immutable, by every vector sampled while the program's
/// stamp, the evaluator's stage edit serial and the Default-ness of the time
/// stand: every stage notice advances the serial, so a standing table holds
/// what a read would answer now. Each entry's `staticSample` is its index.
struct RigExecFrozenStaticSamples {
    std::vector<RigExecSampledInput> samples;
    std::vector<uint64_t> level1;
    std::vector<char> digestible;
};

struct RigExecFrameDigestOrder;

/// One admitted upstream value at a frame's time: authored-level, so it
/// stands where the stage value of \p path stood (RigExecRigEvaluator::
/// SetUpstreamInputs). \p foldHash is the pulled table's fold of an array
/// value and 0 for a scalar, which the frame-cache key folds by value.
struct RigExecUpstreamValue {
    SdfPath path;
    VtValue value;
    uint64_t foldHash = 0;

    bool operator==(const RigExecUpstreamValue &other) const
    {
        return path == other.path && foldHash == other.foldHash &&
               value == other.value;
    }
    bool operator!=(const RigExecUpstreamValue &other) const
    {
        return !(*this == other);
    }
};

/// \p inputs (attribute entries only) as upstream values: one per path, the
/// last given winning, sorted by path, each with a zero fold hash.
std::vector<RigExecUpstreamValue> RigExecUpstreamValuesOf(
    const std::vector<RigExecValueOverride> &inputs);

/// The time-invariant half of a frozen job's head-leaf samples: every keyed
/// head leaf (RigExecForEachHeadLeaf order) whose attribute holds no time
/// samples and might not vary (RigExecBakedHeadLeafVaries), read on the UI
/// thread and shared, immutable, by every vector sampled while the
/// program's stamp, the evaluator's stage edit serial and the Default-ness
/// of the time stand, and past a change of those that re-reads every entry
/// bitwise the same. A leaf that varies is sampled per frame into `values`
/// instead, and holds an empty value and `varying` here. Retained frames
/// share it, so the frame cache counts none of it per frame.
struct RigExecHeadLeafConstants {
    /// Per keyed head leaf: its frozen key, whether it varies, and (when it
    /// does not) its value, empty where the attribute holds none.
    std::vector<SdfPath> keys;
    std::vector<char> varying;
    std::vector<VtValue> values;
    /// The constant leaves folded as samples (RigExecSampleDigest per leaf,
    /// in key order), and whether each value is hashable; every control
    /// digest folds `digest` in place of the leaves.
    uint64_t digest = 0;
    bool digestible = true;
};

/// One frame's sampled input vector, built on the UI thread at enqueue time.
/// Plain values: safe to hand across threads and to hold past the stage edit
/// that cancels the job it was sampled for.
struct RigExecFrameInputs {
    /// Independently sampled job-time oracle facts, value-owned and stage-free.
    std::optional<RigExecOraclePublicationContext> oraclePublications;
    std::map<SdfPath,RigExecWeightReferenceContext> oracleWeightInputs;
    UsdTimeCode time = UsdTimeCode::Default();
    std::vector<RigExecSampledInput> values;
    /// Per SkinTopology index (RigExecBakedLayoutRevision: revisionIndex,
    /// then derivedIndex), source values for every skin layout operation.
    /// Empty for other indices. Sparse Raw Default offset/index source rows
    /// follow this fixed prefix, one per sparse owner in the same order.
    /// The graph body prepares the current layout.
    std::vector<std::vector<VtValue>> layoutLeaves;
    /// Actual source paths, parallel to layoutLeaves. Separate route rows
    /// preserve distinct reads of a path already present in values.
    std::vector<std::vector<SdfPath>> layoutSourcePaths;
    /// The transport values no sample in `values` covers that can move with
    /// the time, which the control digests fold: each layoutLeaves row
    /// whose skin layout is not fixed (RigExecBakedLayoutFixedNow), and
    /// each (revisionLeaves row, key) of an external revision's declared
    /// input whose read can vary (RigExecBakedLeafVaryingNow). Ascending.
    /// Both take the program's answers while they are current and ask the
    /// stage again in the window between an edit and the next live run's
    /// re-ask (a routed edit marks the reads it reached; a notice the
    /// program could not route moves the program stamp), so a key sampled
    /// in that window counts what the edit made time-varying. A fixed
    /// layout reads one value at every time, and the sparse rows read at
    /// Default, so neither is listed; both lists empty fold nothing,
    /// leaving such a rig's digests as they were.
    std::vector<uint32_t> varyingLayoutRows;
    std::vector<std::pair<uint32_t, uint32_t>> varyingRevisionLeaves;
    /// Per-chain-revision path leaves (GeomRevision::leaves), parallel to
    /// the baked program's revisionIndex: one value per key, read on the UI
    /// thread at sample time by the live sampler's own reads
    /// (RigExecSampleRevisionLeaf) through the refreshed inputs, for each
    /// revision whose packet the leaves assemble on the worker; empty for
    /// the rest. Authored fallback values travel even for reader walks;
    /// the consuming graph body resolves produced versions. A fixed skin's
    /// layout keys travel separately in layoutLeaves.
    /// Transport-only: every value is a pure function of digest-covered
    /// samples (the same attributes, sampled by path in `values`, and the
    /// head-leaf samples), repeats a skin's layoutLeaves row, or reads one
    /// value at every time, so the leaves are EXCLUDED from the digest --
    /// except the external inputs `varyingRevisionLeaves` names.
    std::vector<std::vector<VtValue>> revisionLeaves;
    /// The same for every derived target (normals, extent and a projector's
    /// matrix targets), parallel to the baked program's derivedIndex.
    std::vector<std::vector<VtValue>> derivedLeaves;
    /// One frame's stage-derived constraint seeds, sampled on the UI thread
    /// through RigExecBakedProgram::SampleStageFrameSeeds: the xform-slot
    /// bases and frames, the native-source ok/frame pairs, and the
    /// delta-base ok/matrix pairs, each parallel to its program table. The
    /// frozen prologue patches these where the live prologue publishes its
    /// stage reads. Fresh stage data -- NOT a pure function of the sampled
    /// attribute values -- so unlike every other transport this one IS
    /// folded by the control-state digest; a moved constraint target must
    /// miss the cache.
    RigExecStageFrameSeeds stageSeeds;
    /// The standing overrides the vector was sampled under, verbatim from
    /// the caller's list. The worker replicates override placement from
    /// these (SetOverrides' flags drive cone dirtiness exactly as live).
    /// The values ALSO ride as samples in `values` (keyed by override
    /// path), which is what the digest hashes; this list is the worker's
    /// placement input, excluded from the digest as redundant.
    std::vector<RigExecValueOverride> overrides;
    /// Parallel to `overrides`: each attribute override's property path
    /// (prim.attribute), empty for a computation override. Built on the
    /// UI thread by SetOverrides, so the worker places overrides without
    /// building a path; a vector whose two lists disagree in length
    /// declines.
    std::vector<SdfPath> overridePaths;
    /// The head leaves that hold no time samples, read once per program
    /// state and shared by every vector sampled under it; `values` carries
    /// only the head leaves that vary (RigExecHeadLeafConstants). Null for a
    /// program without property chains. Folded by every control digest
    /// through its own precomputed digest.
    std::shared_ptr<const RigExecHeadLeafConstants> headLeafConstants;
    /// The static samples table the samples marked `staticSample` copy, and
    /// the fold order recorded for this vector's path sequence: the
    /// digests take the memoized level-1s and, while the recorded paths
    /// still equal `values`' elementwise, the recorded order. Null for a
    /// vector no sampler built; the digests then fold every sample.
    std::shared_ptr<const RigExecFrozenStaticSamples> staticSamples;
    std::shared_ptr<const RigExecFrameDigestOrder> digestOrder;
    /// The admitted upstream values the vector was sampled under, sorted by
    /// path: the worker's upstream layer, which it diffs against the
    /// snapshot's (rule 8). Every read they reach rides `values` and the
    /// leaves as sampled through that layer; the control-state digest folds
    /// the list itself (`ups` block), and the retained state counts it.
    std::vector<RigExecUpstreamValue> upstream;

    /// Sets `overrides` and builds `overridePaths` from them.
    void SetOverrides(const std::vector<RigExecValueOverride> &list);

    /// Appends \p value sampled at \p path. \p hasValue false records an
    /// explicitly valueless source. \p viaChain marks a stale sample (see
    /// RigExecSampledInput::viaChain).
    void Add(const SdfPath &path, const VtValue &value, bool hasValue = true,
             bool viaChain = false, bool valueBlocked = false);

    /// Whether any sample is marked stale (RigExecSampledInput::viaChain).
    /// No warming job may be built from a vector answering true.
    bool HasChainResolvedInputs() const;

    /// The first value sampled at \p path, or null when none was; a
    /// constant head leaf answers from `headLeafConstants`. Linear: the
    /// vector is built once and read by one job.
    const VtValue *Find(const SdfPath &path) const;

    /// Whether any value was sampled at \p path, valueless or not,
    /// `headLeafConstants` included.
    bool Contains(const SdfPath &path) const;

    void Clear();
};

/// Flags on RigExecFrozenEvalContext::flags. Plain bits, so the context
/// stays trivially copyable.
constexpr uint32_t kRigExecFrozenPublishWeightFields = 1u << 0;
constexpr uint32_t kRigExecFrozenSolverGuidesEnabled = 1u << 1;
/// No admitted compiled program is available. A worker declines a context
/// carrying this flag: it has no admitted compiled program to execute.
constexpr uint32_t kRigExecFrozenBakeRefused = 1u << 2;

/// Freezes \p evaluator's current baked program into an epoch-pinned
/// snapshot for background warming. UI thread only: it reads the live
/// program and the stage.
///
/// Answers false, leaving \p frozen untouched, when the program is absent
/// or has a descriptor the detached execution contract cannot reproduce.
/// \p error names the refusal. Property chains, providers, weight fields,
/// layouts, and geometry use their admitted operations in the same graph.
/// Optional scalar reference checks carry fresh source facts with each job.
///
/// A snapshot pins the program OBJECT it was cloned from plus the epoch it
/// was cloned in. Re-freeze after any change that rebuilds the program; a
/// change that only patches its avar constants
/// (RigExecBakedProgram::ApplyAvarValueEdits) needs only
/// RigExecPatchFrozenAvarConstants, which carries the patched region onto a
/// copy without re-cloning the epoch. The registry's session cache owns
/// that discipline in production (see RigExecImagingRegistry); direct
/// callers re-freeze around their own edits.
///
/// Upstream inputs freeze too: the snapshot keeps the table live's last run
/// placed, and each job carries its own (RigExecFrameInputs::upstream),
/// which the worker diffs against it, so a value placed, moved or lifted
/// since the freeze re-reads what it reaches.
bool RigExecFreezeProgram(const RigExecRigEvaluator &evaluator,
                          std::shared_ptr<const RigExecFrozenProgram> *frozen,
                          std::string *error = nullptr);

/// Checks the same sampling/worker support contract as FreezeProgram without
/// cloning its slot state. UI thread only. Unsupported rigs must use a cache
/// key fenced by stage edits and frame time, not an incomplete sampled digest.
bool RigExecCanFreezeProgram(const RigExecRigEvaluator &evaluator,
                             std::string *error = nullptr);

/// The digest of a baked program's patchable avar region: every constant
/// binding's value and varying flag, the promoted set, and the constant
/// table. RigExecBakedProgram::ApplyAvarValueEdits moves exactly this
/// region, so two digests that agree mean no patch landed between them --
/// which is what lets a session cache keep its snapshot without re-cloning
/// the epoch every frame. The frozen-snapshot refresh compares this to
/// decide patch-vs-keep; the epoch digest no longer folds it (plan D3).
uint64_t RigExecFrozenAvarRegionDigest(const RigExecBakedProgram &program);

/// The epoch-constant digest (plan D3): the value-sensitive constant region
/// the epoch digest no longer covers, captured at epoch build and at every
/// value patch. Folds into control-digest derivation (see
/// RigExecControlStateDigestWithConstants), so a constant patch opens a
/// new key namespace while the epoch half stays standing. Same value as
/// RigExecFrozenAvarRegionDigest: one sampler, two consumers.
uint64_t RigExecEpochConstantDigest(const RigExecBakedProgram &program);

/// Bindings per constant region (plan 2.0): the avar-constant bindings are
/// partitioned into regions of this many consecutive bindings, and each
/// entry's provenance records the regions its computation read.
constexpr size_t kRigExecConstantRegionStride = 64;

/// The constant region holding constant binding \p binding.
inline uint64_t
RigExecConstantRegionForBinding(size_t binding)
{
    return uint64_t(binding / kRigExecConstantRegionStride);
}

/// The constant regions a value patch touched: the regions of the
/// patchable bindings \p paths name, sorted and unique. Unmapped paths
/// contribute nothing. Carry-over across a constant-namespace change is
/// sound only for entries that read none of these regions.
std::vector<uint64_t> RigExecConstantRegionsForPaths(
    const RigExecBakedProgram &program,
    const std::vector<SdfPath> &paths);

/// The epoch half of a frame-cache key for \p evaluator's current program
/// state: structure only (plan D3) -- the binding epoch digest, the
/// program's build count, and the avar binding-table shapes. Avar constant
/// VALUES are not folded here: a value patch must not evict the world, so
/// they ride the control digest instead (see RigExecEpochConstantDigest).
/// A rig with no program keys on the epoch alone (its refusal digest
/// carries the stage-edit serial instead).
uint64_t RigExecFrameCacheEpochDigest(const RigExecRigEvaluator &evaluator);

/// True when \p live has routed a stage value edit to an input, or bumped
/// its program stamp, since \p base was frozen from it or last patched.
/// Neither moves the avar region, so a session that keeps its snapshot on
/// an unchanged RigExecFrozenAvarRegionDigest asks this too: the snapshot's
/// history predates the edit, and a job cloned from it would skip the steps
/// the edit reached. RigExecPatchFrozenAvarConstants carries both.
bool RigExecFrozenSnapshotOwesLiveEdits(const RigExecFrozenProgram &base,
                                        const RigExecBakedProgram &live);

/// Carries \p live's patched avar region onto a copy of \p base, for a
/// session whose snapshot still pins the program it was cloned from.
/// Copy-on-write: \p base is never mutated, so jobs already holding it run
/// on, and \p out pins the same epoch and history with the new constants.
/// The copy also owes every value edit and stamp bump \p live has taken
/// since \p base (RigExecFrozenSnapshotOwesLiveEdits): its first run
/// re-runs the steps those edits reached, or everything after a bump.
/// Answers false, having left \p out untouched, when the two programs are
/// not the same shape (different binding counts -- a rebuild, not a patch),
/// or when live's last run placed another upstream table than the
/// snapshot's (live's avar slots hold that table's values, which the
/// copy's history does not). UI thread only: it reads the live program.
bool RigExecPatchFrozenAvarConstants(
    const RigExecFrozenProgram &base, const RigExecBakedProgram &live,
    std::shared_ptr<const RigExecFrozenProgram> *out,
    std::string *error = nullptr);

/// Retained execution state for one frozen snapshot and one execution lane.
/// Reuse it for sequential jobs; overlapping jobs require separate workspaces.
class RigExecFrozenWorkspace {
public:
    ~RigExecFrozenWorkspace();
    RigExecFrozenWorkspace(const RigExecFrozenWorkspace &) = delete;
    RigExecFrozenWorkspace &operator=(const RigExecFrozenWorkspace &) = delete;
private:
    struct Impl;
    explicit RigExecFrozenWorkspace(std::shared_ptr<const RigExecFrozenProgram>);
    std::unique_ptr<Impl> _impl;
    friend struct RigExecFrozenWorkspaceAccess;
    friend std::unique_ptr<RigExecFrozenWorkspace> RigExecCreateFrozenWorkspace(
        std::shared_ptr<const RigExecFrozenProgram>);
};

/// Retains the snapshot and clones its private working state once.
std::unique_ptr<RigExecFrozenWorkspace> RigExecCreateFrozenWorkspace(
    std::shared_ptr<const RigExecFrozenProgram> snapshot);

/// The epoch a background job is pinned to: digests and counts only, plus
/// the frozen program reference. No member can name the stage, the
/// evaluator, or any USD object, which is what makes it safe to share with
/// a worker.
struct RigExecFrozenEvalContext {
    /// RigExecRigEvaluator::GetBindingEpochDigest at enqueue time: the epoch
    /// the job was sampled for. Enforcement is the generation fence (any
    /// edit bumps it) plus the epoch-keyed cache. A retained workspace also
    /// requires every job to carry its initially bound epoch.
    uint64_t epochDigest = 0;
    /// The scheduler generation the job was enqueued under. Rechecked before
    /// publish; a mismatch drops the result.
    uint64_t generation = 0;
    /// Provider slots the worker sizes its private arena for.
    size_t slotCount = 0;
    /// Identity of the baked program the job was sampled for: the epoch
    /// digest mixed with the program's bound/varying input counts. Retained
    /// workspaces reject jobs carrying a different program digest.
    uint64_t programDigest = 0;
    /// Samples RigExecSampleFrameInputs recorded. The worker compares by
    /// value against the vector it was handed; a count that disagrees means
    /// the vector and the context were sampled for different frames, and the
    /// job is dropped rather than run against a truncated input set.
    size_t varyingInputCount = 0;
    /// kRigExecFrozen* bits: the per-run toggles the live path reads off the
    /// evaluator (weight-field publication, solver guides) plus the D7
    /// refusal flag. Sampled at enqueue because the live toggles can move
    /// without the epoch moving.
    uint32_t flags = 0;
    /// The epoch-pinned program snapshot the job runs, or null when the rig
    /// was not frozen. A raw pointer, so the context stays trivially
    /// copyable: the JOB OWNS the snapshot (its closure holds the
    /// shared_ptr) and this names it for the worker, which only ever reads
    /// it. Null declines a production job -- the runner cannot evaluate
    /// without a program -- while an injected test kernel ignores it.
    const RigExecFrozenProgram *frozen = nullptr;
    /// Optional caller-owned execution lane. Snapshot identity, epoch, and
    /// program digest must match every job on this workspace. Null gives a
    /// fresh independent job without retained value or readiness state.
    RigExecFrozenWorkspace *workspace = nullptr;
};

static_assert(std::is_trivially_copyable<RigExecFrozenEvalContext>::value,
              "a frozen context must be plain digests and counts");

/// A job's private slot arena: the per-frame working state the baked serial
/// executor would otherwise keep on the live program. Owned by one job,
/// sized from the context, and never shared -- which is what lets the worker
/// run without touching live evaluator state.
///
/// The slots are doubles, the program's dense working unit for the scalar
/// state a frozen runner carries (avar tables, scratch); wider per-frame
/// state (points, matrices, packets) is carried by the runner itself, out of
/// the sampled inputs, and likewise never aliases the live program.
class RigExecFrozenArena {
public:
    RigExecFrozenArena() = default;
    explicit RigExecFrozenArena(size_t slots) { Resize(slots); }

    RigExecFrozenArena(const RigExecFrozenArena &) = delete;
    RigExecFrozenArena &operator=(const RigExecFrozenArena &) = delete;
    RigExecFrozenArena(RigExecFrozenArena &&) = default;
    RigExecFrozenArena &operator=(RigExecFrozenArena &&) = default;

    /// Sizes the arena for \p context's slot count. False, having changed
    /// nothing, when the count is absurd (fail-closed: a corrupt context
    /// must not allocate the machine).
    bool ResizeFor(const RigExecFrozenEvalContext &context);

    /// Sizes the arena to exactly \p slots, zeroed. False on an absurd
    /// count, having changed nothing.
    bool Resize(size_t slots);

    double *Data() { return _slots.data(); }
    const double *Data() const { return _slots.data(); }
    size_t Size() const { return _slots.size(); }
    size_t Bytes() const { return _slots.size() * sizeof(double); }

    /// Zeroes every slot without freeing. A reused arena cannot leak one
    /// frame's working state into another's.
    void Clear();

private:
    std::vector<double> _slots;
};

/// Whether the calling thread is inside a frozen serial scope. Kernel
/// launch sites consult this (parallel.h declares it for the kernels; the
/// declaration is repeated here so this header stands alone) and take
/// their serial variant while it is set, whatever the process-wide switch
/// says.
///
/// A process-wide switch cannot answer this question: the UI thread and a
/// below-normal worker need different answers at the same moment, and a
/// per-job toggle of shared state would be a race. Thread-local depth is
/// the whole mechanism, which is also why a frozen run must never hop
/// threads mid-frame.
bool RigExecFrozenSerialActive();

/// Marks the calling thread as running a frozen frame until the scope
/// exits. Scopes nest; only the outermost exit restores the prior state.
/// Non-copyable: a scope that could be copied could also be held past the
/// run it constrains.
class RigExecFrozenSerialScope {
public:
    RigExecFrozenSerialScope();
    ~RigExecFrozenSerialScope();

    RigExecFrozenSerialScope(const RigExecFrozenSerialScope &) = delete;
    RigExecFrozenSerialScope &operator=(
        const RigExecFrozenSerialScope &) = delete;

private:
    bool _active = false;
};

/// What one frozen job executed, for a caller that asks RigExecEvaluateFrozen
/// for it: the ordinary trace, heads and sources included, in execution
/// order, with
/// steps indexed as in the snapshot's program. Empty when the job declined.
struct RigExecFrozenRunReport {
    std::vector<RigExecOpTraceEntry> region;
    bool ran = false;
    /// The worker program's source keys: how many this job built, and how
    /// many kept keys RIGEXEC_VERIFY_SOURCE_KEYS found moved over the
    /// lane's life (a workspace's jobs accumulate; a fresh clone starts at 0).
    size_t sourceKeysBuilt = 0;
    size_t sourceKeyMismatches = 0;

    void Clear()
    {
        region.clear();
        ran = false;
        sourceKeysBuilt = 0;
        sourceKeyMismatches = 0;
    }
};

/// The serial step runner a frozen job executes: the baked program's serial
/// executor over the private arena, reading nothing but \p context and \p
/// inputs. Returns false to hand the generation back, exactly as
/// RigExecBakedProgram::Run does; a runner that runs must set
/// \p pose->valid, and the entry point treats a valid flag left down as a
/// refusal.
///
/// The parameter list IS the worker's input surface, which is the point of
/// the audit below: a runner that needs anything else -- a stage, a query,
/// the evaluator -- cannot be written against this signature without
/// capturing it, and a capture is visible at the call site rather than
/// buried three frames down.
using RigExecFrozenStepRunner = std::function<bool(
    const RigExecFrozenEvalContext &context,
    const RigExecFrameInputs &inputs, RigExecFrozenArena &arena,
    RigExecRigPose *pose)>;

/// Evaluates \p inputs under \p context with \p runner, reading nothing
/// else. The returned pose is bit-identical to a live evaluation of the same
/// inputs; when it cannot be -- unimplemented, inconsistent, stale, or
/// cancelled -- the pose is invalid and the caller evaluates live instead.
///
/// The checks, in order: the context must carry an admitted program; the
/// input vector's count must equal the context's sampled count; when \p
/// scheduler is given, the generation must be current (start check); the
/// runner executes under a serial scope against a private arena sized from
/// the context, with \p pose->time preset to the inputs' time; when \p
/// scheduler is given, the generation must STILL be current (publish check).
/// Any failure -- including a null runner, a runner that returns false, or
/// one that leaves valid down -- answers invalid at the requested time.
///
/// \p rig names the rig the generation belongs to. It is a plain value, not
/// a handle: naming the fence is not touching the stage.
///
/// \p report, when given, is cleared and then filled by a production runner
/// that ran the job (RigExecFrozenRunReport).
RigExecRigPose RigExecEvaluateFrozen(
    const RigExecFrozenEvalContext &context,
    const RigExecFrameInputs &inputs, RigExecFrozenStepRunner runner,
    const RigExecBackgroundScheduler *scheduler = nullptr,
    const SdfPath &rig = SdfPath(), RigExecFrozenRunReport *report = nullptr);

/// Evaluates \p inputs under \p context, reading nothing else. The returned
/// pose is bit-identical to a live evaluation of the same inputs; when it
/// cannot be -- unimplemented, inconsistent, or cancelled -- the pose is
/// invalid and the caller evaluates live instead.
///
/// Without a step runner this overload cannot prove bit-identity, so it
/// always answers invalid at the requested time: the fail-closed stub. It
/// is retained so callers written against the Stream 0 skeleton keep their
/// meaning -- every request declines, the caller evaluates live -- while
/// production call sites move to the runner overload above.
RigExecRigPose RigExecEvaluateFrozen(const RigExecFrozenEvalContext &context,
                                     const RigExecFrameInputs &inputs);

/// One chain input as the epoch currency check pins it, the way the live
/// path pins it on its first run: the attribute and a query over it,
/// whether it carries authored connections (read as a walk then, never as
/// a value), and, for an input that cannot change until the stage does,
/// the value read once. UI thread only: it holds live stage handles.
struct RigExecChainSampleInput {
    UsdAttribute attribute;
    UsdAttributeQuery query;
    SdfPath path;
    bool connected = false;
    bool constant = false;
    VtValue constantValue;
    explicit operator bool() const { return bool(attribute); }
};

/// One revision of one sampled chain, in the chain's own order.
struct RigExecChainSampleRevision {
    SdfPath moverPath;
    UsdPrim moverPrim;
    /// The mover's bound weight objects, if any. Their compiled field
    /// producers run from the frozen job's captured source facts.
    SdfPathVector weightObjects;
    RigExecChainSampleInput enabled;
    RigExecChainSampleInput defaultWeight;
    RigExecChainSampleInput operation;
    RigExecChainSampleInput value;
    RigExecChainSampleInput minimum;
    RigExecChainSampleInput maximum;
    RigExecChainSampleInput keys;
    RigExecChainSampleInput tangents;
};

/// One sampled property chain: its target, its revisions, and the operator
/// inputs that read it at a phase (RigExecPhasedConnection).
struct RigExecChainSampleChain {
    SdfPath targetPath;
    UsdAttribute target;
    UsdAttributeQuery targetQuery;
    SdfValueTypeName valueType;
    std::vector<RigExecChainSampleRevision> revisions;
    std::vector<RigExecPhasedConnection> phased;
};

/// The epoch-pinned chain bindings one sampling call evaluates through.
/// Discovered from the evaluator's public mover order plus the stage, in
/// the chains' dependency order, so a chain that revises an input of
/// another runs first. UI thread only: every entry holds live stage
/// handles, which is why these never travel with a job -- only the values
/// evaluated through them do.
///
/// Pinned per epoch, not per frame: constant inputs are folded at bind
/// time exactly as the live path folds them on its first run, and a
/// constant is the same value at every time code, so the pins read
/// identically whatever frame the live path ran first. A stage edit moves
/// a pin the way it moves the live path's (which rebinds on every
/// notice); RigExecChainSampleBindingsStillCurrent tells the two apart.
struct RigExecChainSampleBindings {
    std::vector<RigExecChainSampleChain> chains;
};

/// Discovers \p evaluator's property chains from its public mover order
/// and binds every input each revision reads, in dependency order. UI
/// thread only: it reads the live stage.
///
/// Answers false, having left \p out untouched, when the chains cannot be
/// named faithfully: a math mover with anything but exactly one exact
/// property target, or a dependency cycle. A bound chain carrying a weight
/// object binds and names its compiled field, so the caller can tell "no
/// chains" (empty, success) from "unchainable" (failure). \p error, when
/// given, says which.
bool RigExecBindChainSampleInputs(
    const RigExecRigEvaluator &evaluator, RigExecChainSampleBindings *out,
    std::string *error = nullptr);

/// Whether \p bindings still name \p evaluator's epoch: the same math
/// movers over the same targets of the same types, the same phased reads
/// over the same hops, and every folded constant still reading the value
/// it was pinned with. A stage edit that moves any of those answers false,
/// and the caller rebinds. UI thread only: it reads the live stage.
bool RigExecChainSampleBindingsStillCurrent(
    const RigExecChainSampleBindings &bindings,
    const RigExecRigEvaluator &evaluator);

/// Samples one frame's input vector on the UI thread, at \p time, for the
/// program \p evaluator currently holds. Every varying binding is read
/// through the same route the baked frame path reads it -- the retained
/// query when USD alone answers, the job's overrides and the stage when the
/// walk is read the long way -- plus the prologue reads the bench measures
/// (blend channels, constraint operator arrays, ribbon drivers, chain base
/// points) and the standing \p overrides, which never reach the stage and
/// therefore must be sampled from the list the caller passes.
///
/// Returns false, having left \p out untouched, when no faithful vector can
/// be sampled: no compiled program, a source read the sampler cannot
/// reproduce without the program's own sampling hook (xform-derived seeds,
/// native constraint sources, geometry-delta bases), or a null resolved
/// inputs pointer. \p error, when given, says which.
///
/// PROPERTY CHAINS. A read a property chain result or a phased record can
/// answer is not sampled as a value: every head leaf the property ops and
/// the reader walks read travels under its synthetic key -- sampled at the
/// job's time where it varies, from RigExecFrameInputs::headLeafConstants
/// where it does not. The shared graph evaluates property operations and
/// resolves produced values in consuming operations through declared reader
/// walks, exactly as live execution does.
bool RigExecSampleFrameInputs(
    const RigExecRigEvaluator &evaluator, UsdTimeCode time,
    const std::vector<RigExecValueOverride> &overrides,
    RigExecFrameInputs *out, std::string *error = nullptr);

/// UPSTREAM INPUTS. Each sampler also takes the frame's upstream values
/// (none in the overloads without them). A value is kept only where live
/// admits it (RigExecUpstreamDropReason against the program standing) and
/// travels as RigExecFrameInputs::upstream. Every read live takes through
/// the upstream layer is sampled through the same layer: a binding with a
/// value on a hop of its walk is read the long way and sampled at its head,
/// constant or not; a head leaf at a valued path is sampled under its key,
/// constant or not; revision, layout and mover-scalar reads see the value
/// where they would see the stage's. A path no value stands on reads the
/// stage, so a job whose snapshot held a value since lifted needs nothing
/// more: the worker diffs the job's table against the snapshot's.
bool RigExecSampleFrameInputs(
    const RigExecRigEvaluator &evaluator, UsdTimeCode time,
    const std::vector<RigExecValueOverride> &overrides,
    const std::vector<RigExecUpstreamValue> &upstream,
    RigExecFrameInputs *out, std::string *error = nullptr);

/// Samples exactly as RigExecSampleFrameInputs, but checks the caller's
/// epoch-pinned \p bindings instead of binding fresh.
/// Answers false, having left \p out untouched, when \p bindings no longer
/// name the evaluator's epoch (see
/// RigExecChainSampleBindingsStillCurrent) -- the caller rebinds and
/// retries -- or for any reason the plain sampler answers false. UI
/// thread only.
bool RigExecSampleFrameInputsWithChainBindings(
    const RigExecRigEvaluator &evaluator, UsdTimeCode time,
    const std::vector<RigExecValueOverride> &overrides,
    const RigExecChainSampleBindings &bindings, RigExecFrameInputs *out,
    std::string *error = nullptr);
bool RigExecSampleFrameInputsWithChainBindings(
    const RigExecRigEvaluator &evaluator, UsdTimeCode time,
    const std::vector<RigExecValueOverride> &overrides,
    const std::vector<RigExecUpstreamValue> &upstream,
    const RigExecChainSampleBindings &bindings, RigExecFrameInputs *out,
    std::string *error = nullptr);

/// Samples exactly as RigExecSampleFrameInputsWithChainBindings WITHOUT the
/// per-call currency check (RigExecChainSampleBindingsStillCurrent).
///
/// For a caller that proves its bindings current another way: bound for
/// the evaluator's present epoch digest, and dropped on every stage notice
/// that could move a chain without moving the epoch -- an edit under a
/// chain mover, its weight objects or its target, or any resync. The check
/// is the price of trusting nothing, and on the full biped stack it
/// re-probes every chain mover's inputs: MEASURED at 60.6 ms per call,
/// paid twice on every viewport release. A caller that cannot make that
/// promise uses the verifying entry point. UI thread only.
bool RigExecSampleFrameInputsWithTrustedChainBindings(
    const RigExecRigEvaluator &evaluator, UsdTimeCode time,
    const std::vector<RigExecValueOverride> &overrides,
    const RigExecChainSampleBindings &bindings, RigExecFrameInputs *out,
    std::string *error = nullptr);
bool RigExecSampleFrameInputsWithTrustedChainBindings(
    const RigExecRigEvaluator &evaluator, UsdTimeCode time,
    const std::vector<RigExecValueOverride> &overrides,
    const std::vector<RigExecUpstreamValue> &upstream,
    const RigExecChainSampleBindings &bindings, RigExecFrameInputs *out,
    std::string *error = nullptr);

/// One warming burst's prepared sample state: everything the per-frame
/// sampler re-derives that a burst holds fixed, computed once.
///
/// A burst (one OnEditCommitted/OnIdle trigger's neighbor-plus-sweep run)
/// is a synchronous UI-thread span: no notice, evaluation, or rebind can
/// interleave its frames, so the verified bindings, the override
/// placement, the per-table visit sets, the epoch digest, and every
/// time-invariant read are identical for all of them. Preparing them once
/// turns the per-frame currency re-verification, the binding-table walks
/// over epoch constants, and the static array re-reads into map lookups.
///
/// LIFETIME. Standing, UI thread only: the registry keeps one cache
/// per rig and re-serves it across triggers while its pins (program,
/// cache-epoch digest, overrides) are current, rebuilding only when they
/// move; pins-determined unusable outcomes (prep overrun, shape decline)
/// latch the same way. The cached sampler validates the program pointer
/// and the override list per frame and fails loud on any mismatch, so a
/// reused or foreign cache fails instead of serving; anything subtler is
/// contained by the epoch half of the key (a drifted cache computes
/// under a stale epoch, whose entries no lookup can reach).
struct RigExecBurstStaticSample {
    RigExecSampledInput sample;
    uint64_t digest = 0;
    bool digestValid = false;
};

struct RigExecBurstSampleCache {
    const RigExecBakedProgram *program = nullptr;
    uint64_t epochDigest = 0;
    RigExecChainSampleBindings bindings;
    std::vector<RigExecValueOverride> overrides;
    std::vector<char> overrideFlags;
    /// The upstream values the burst was prepared under, as given (a pin
    /// like `overrides`), the admitted ones by path (the layer every frame
    /// reads through), and per override number whether one stands on a hop
    /// of its walk. `readFlags` is `overrideFlags` or `upstreamFlags`: the
    /// bindings read the long way, which the site lists cover.
    std::vector<RigExecUpstreamValue> upstream;
    std::vector<RigExecUpstreamValue> upstreamAdmitted;
    std::map<SdfPath, VtValue> upstreamLayer;
    std::vector<char> upstreamFlags;
    std::vector<char> readFlags;
    bool placeable = false;
    bool usable = false;
    /// Per-table indices of structs with any varying-or-overridden field,
    /// in table order, so the cached sampler visits in emission order.
    /// avarBindings needs none: it holds the varying ones only.
    std::vector<size_t> ladderSites;
    std::vector<size_t> spaceSwitchSites;
    std::vector<size_t> solverSites;
    std::vector<size_t> constraintSites;
    std::vector<size_t> weightSites;
    std::vector<size_t> interpolatorSites;
    /// Static samples by route, filled lazily as frames serve them.
    /// Partitioned because one path can be sampled both stage-direct and
    /// through the resolved inputs at different values (an override
    /// standing on a statically-read attribute).
    std::unordered_map<SdfPath, RigExecBurstStaticSample, SdfPath::Hash>
        staticStage;
    std::unordered_map<SdfPath, RigExecBurstStaticSample, SdfPath::Hash>
        staticResolved;
    /// Emission indices in sorted-path order, recorded from the first
    /// frame's vector: the emission path sequence is timeless (every Add
    /// site's guard is validity-only), so it is identical for every frame
    /// of the burst and the digest combines through it without re-sorting.
    std::vector<size_t> sortedOrder;
    size_t orderValuesSize = 0;
    SdfPath orderFirstPath;
    SdfPath orderLastPath;

    void Clear();
};

/// Prepares \p cache for one burst over \p program: the prologue-decline
/// checks, the override placement, and the per-table visit sets. Pure in
/// \p program and \p overrides: verification (pins, epoch) is the
/// caller's, passed in ready. Answers false, having left \p cache
/// unusable, when no frame of the burst could sample: a prologue decline
/// or unplaceable overrides. \p error, when given, says which.
bool RigExecBuildBurstSampleCache(
    const RigExecBakedProgram &program,
    const RigExecChainSampleBindings &bindings,
    const std::vector<RigExecValueOverride> &overrides, uint64_t epochDigest,
    RigExecBurstSampleCache *cache, std::string *error = nullptr);
/// The same, pinned to the burst's \p upstream values as well, which every
/// frame of the burst is then sampled under (RigExecSampleFrameInputs).
bool RigExecBuildBurstSampleCache(
    const RigExecBakedProgram &program,
    const RigExecChainSampleBindings &bindings,
    const std::vector<RigExecValueOverride> &overrides,
    const std::vector<RigExecUpstreamValue> &upstream, uint64_t epochDigest,
    RigExecBurstSampleCache *cache, std::string *error = nullptr);

/// Samples exactly as RigExecSampleFrameInputsWithChainBindings, but
/// through \p cache: no currency re-verification, binding tables visited
/// through the prepared site lists, and time-invariant reads served from
/// the static maps. The vectors are elementwise identical to the plain
/// pinned sampler's (same paths in the same order, same values, same
/// flags) plus the burst-route marks the cached digest reads.
///
/// Answers false, having left \p out untouched, when \p cache is
/// unusable for this sample (a foreign program, differing overrides, or a
/// declined build) or for any reason the plain sampler answers false.
/// UI thread only.
bool RigExecSampleFrameInputsWithBurstCache(
    const RigExecRigEvaluator &evaluator, UsdTimeCode time,
    const std::vector<RigExecValueOverride> &overrides,
    RigExecBurstSampleCache *cache, RigExecFrameInputs *out,
    std::string *error = nullptr);
/// The same under \p upstream, which must equal the values \p cache was
/// prepared under (a differing list fails loud, like differing overrides).
bool RigExecSampleFrameInputsWithBurstCache(
    const RigExecRigEvaluator &evaluator, UsdTimeCode time,
    const std::vector<RigExecValueOverride> &overrides,
    const std::vector<RigExecUpstreamValue> &upstream,
    RigExecBurstSampleCache *cache, RigExecFrameInputs *out,
    std::string *error = nullptr);

/// Builds the frozen runner over the same compiled graph as live execution.
/// It places the job's sampled sources, executes on context.workspace when
/// supplied, and publishes after joining. A null workspace creates a fresh
/// private job. No stage or evaluator access occurs on the worker.
/// Snapshot, sampled counts, and retained workspace identity must agree;
/// mismatches decline the job without publishing. Numerical kernels use
/// the calling thread's frozen serial scope.
RigExecFrozenStepRunner RigExecMakeProductionStepRunner();

/// Hashes \p inputs to the control-state digest the frame cache keys on
/// (Stream A feeds this into RigExecFrameCacheKey::controlDigest). Time is
/// deliberately NOT hashed: a scrub between identical control states hits
/// across times, and a control edit changes the digest at every affected
/// frame by construction. Overrides are hashed with the authored values,
/// because a drag must never be served a pre-drag pose, and so are the
/// vector's upstream values (`upstream`), when it carries any.
///
/// \p exact, when given, reports whether every sampled value was hashed at
/// full precision. A value of a type the hasher does not name contributes
/// its type name only, and exact comes back false: the caller must then
/// decline to cache under this digest rather than risk two different frames
/// sharing one key. Known-named types are the scalar, Gf, token, path, and
/// array types the rig inputs actually hold (frozenContext.cpp).
uint64_t RigExecFrozenControlDigest(const RigExecFrameInputs &inputs,
                                   bool *exact = nullptr);

/// What the purity audit concluded about one unit of evaluation code: every
/// kernel and solver Stream B examined, and the verdict each one earned.
enum class RigExecFrozenPurity {
    /// A pure function of its inputs: no static, thread-local, or member
    /// state survives a call, so a worker may run it freely.
    Pure,
    /// Immutable within the epoch: safe for a worker only as a copy taken
    /// at enqueue, never as a live read.
    EpochPinned,
    /// Must never be reached from a worker: live evaluator state, the stage,
    /// a lock, or shared mutable state. The frozen path bypasses it by
    /// construction (sampled inputs, private arenas, program-held bindings),
    /// and a rig that cannot bypass one falls back to live eval.
    LiveOnly,
};

/// One row of the Stream B purity audit (Constraints: a pose must be a pure
/// function of its sampled inputs).
struct RigExecPurityFinding {
    /// The unit examined, e.g. "rigExecMath/solvers.h".
    const char *unit = nullptr;
    /// The verdict it earned.
    RigExecFrozenPurity verdict = RigExecFrozenPurity::LiveOnly;
    /// Why, in one line: what was examined and what the worker does instead.
    const char *note = nullptr;
};

/// The audit itself, as data: every kernel, solver, cache, and live-state
/// holder the frozen path was checked against. The worker's input surface
/// (RigExecFrozenStepRunner's parameter list) plus this table is the proof
/// no path from a worker reaches the stage: every row is either safe to run
/// (Pure), safe as an enqueue-time copy (EpochPinned), or bypassed by
/// construction with the bypass named (LiveOnly).
const std::vector<RigExecPurityFinding> &RigExecFrozenPurityAudit();

}  // namespace rigExec

#endif  // RIGEXEC_FROZEN_CONTEXT_H
