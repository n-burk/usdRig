//
// RigExec frozen evaluation contexts: everything a background frame job may
// read, sampled up front on the UI thread.
//
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
//
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
//
// THREADING. The context is trivially copyable plain data: safe to build on
// the UI thread, hand across threads, and hold past the stage edit that
// cancels the job it was sampled for. The input vector is likewise plain
// values once sampled, but it is built by one thread (the sampler) and read
// by one job -- concurrent mutation during a run is a caller bug, not a
// checked condition. The arena is strictly thread-confined: one arena per
// job, never shared, never moved while a run reads it. The serial scope is
// thread-local: it constrains the thread that entered it and no other.
//

#ifndef RIGEXEC_FROZEN_CONTEXT_H
#define RIGEXEC_FROZEN_CONTEXT_H

#include "bakedProgramImpl.h"
#include "moverGraph.h"
#include "rigEvaluator.h"

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
#include <string>
#include <type_traits>
#include <unordered_map>
#include <vector>

PXR_NAMESPACE_USING_DIRECTIVE

namespace rigExec {

class RigExecBackgroundScheduler;
using RigExecFrameGeneration = uint64_t;

/// An epoch-pinned, worker-safe snapshot of a baked program, built on the UI
/// thread by RigExecFreezeProgram. See that function for the contract.
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
    /// True when the value came from the generation's in-memory resolved
    /// inputs on a binding with a property chain on its walk -- a chain
    /// OUTPUT computed for whatever time the evaluator last ran, not a
    /// fresh read at the sampled time. The chain-sampling hook (see
    /// RigExecSampleFrameInputs) refreshes such values, so a sample is
    /// marked only when the hook declined the rig; a vector carrying one
    /// must never warm a frame it was not proven for, because the digest
    /// would name stale values as the frame's.
    bool viaChain = false;
    /// Which burst-cache memo map served this sample, when one did: the
    /// digest folds the memoized level-1 from the same route, so a path
    /// sampled both stage-direct and through the resolved inputs can never
    /// alias across the two. The plain sampler leaves every sample fresh.
    int burstSampleRoute = RigExecBurstRouteFresh;
};

/// One frame's sampled input vector, built on the UI thread at enqueue time.
/// Plain values: safe to hand across threads and to hold past the stage edit
/// that cancels the job it was sampled for.
struct RigExecFrameInputs {
    UsdTimeCode time = UsdTimeCode::Default();
    std::vector<RigExecSampledInput> values;
    /// Per-chain-revision assembled skin packets, parallel to the baked
    /// program's revisionIndex, assembled on the UI thread at sample time by
    /// the real RigExecAssembleSkinParameters (which reads the mover prim and
    /// the live topology cache -- both UI-thread-only). A worker cannot
    /// assemble them, so they travel with the job instead of being sampled
    /// as inputs. Transport-only: every number in a packet is a pure
    /// function of digest-covered values (the revision's sampled
    /// enabled/defaultWeight/method plus the sampled overrides), so packets
    /// are deliberately EXCLUDED from the control-state digest. Empty for a
    /// rig with no chain revisions, and for vectors sampled before the
    /// frozen executor landed (a job needing packets it was not given
    /// declines).
    std::vector<RigExecMoverParameters> revisionPackets;
    /// Per blend sample's sparse layout, resolved at sample time through the
    /// live blend-shape cache: [revisionIndex position][channel][sample], by
    /// shared pointer. A null entry means the revision is not a blendshape or
    /// the sample is dense (points ride the sampled values instead). Layouts
    /// resolved from the shared cache are pointer-identical to the live
    /// prologue's, which is what keeps packet == cheap; refused (connected)
    /// shapes resolve per frame and additionally sample their
    /// offsets/indices arrays into values so the key moves with them.
    std::vector<std::vector<std::vector<
        std::shared_ptr<const RigExecBlendSampleLayout>>>>
        blendLayouts;
    /// One curvenet profile bind per revisionIndex position, resolved at
    /// sample time by the same build the live prologue runs: the bind
    /// cache has no locking, so the worker cannot resolve it, and the
    /// bind's every input is Default-time epoch data. Null for a
    /// non-curvenet revision, for a revision whose net is missing (the
    /// worker breaks before reading it, as live), and for a remembered
    /// failed bind. Excluded from the digest like the other transports.
    std::vector<std::shared_ptr<const RigExecProfileMoverBinding>>
        curvenetBinds;
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
    /// Property-chain diagnostics for the sampled time, in chain order,
    /// produced by the chain-sampling hook on the UI thread. The chains run
    /// first on the live path and their lines are the first of the
    /// generation, so the frozen epilogue prepends these verbatim. Outputs,
    /// not inputs: excluded from the digest like every other pose content.
    std::vector<std::string> chainDiagnostics;
    /// Property-chain outputs for the sampled time, per target, produced by
    /// the chain-sampling hook on the UI thread. The frozen prologue
    /// publishes these into its program-owned property results exactly where
    /// the live prologue publishes the chains it ran, so a chain-driven
    /// binding -- whose patched constant was sampled through the same values
    /// -- and a chain target read as pose content agree. Outputs, not
    /// inputs: excluded from the digest (a pure function of digest-covered
    /// values, like the revision packets). Empty for a rig with no chains,
    /// and empty for a chain that skipped (no authored base), exactly as on
    /// the live path.
    std::map<SdfPath, VtValue> chainResults;
    /// The standing overrides the vector was sampled under, verbatim from
    /// the caller's list. The worker replicates override placement from
    /// these (SetOverrides' flags drive cone dirtiness exactly as live).
    /// The values ALSO ride as samples in `values` (keyed by override
    /// path), which is what the digest hashes; this list is the worker's
    /// placement input, excluded from the digest as redundant.
    std::vector<RigExecValueOverride> overrides;

    /// Appends \p value sampled at \p path. \p hasValue false records an
    /// explicitly valueless source. \p viaChain marks a chain-resolved
    /// sample (see RigExecSampledInput::viaChain).
    void Add(const SdfPath &path, const VtValue &value, bool hasValue = true,
             bool viaChain = false);

    /// Whether any sample came from in-memory chain outputs rather than a
    /// fresh read. A vector answering true is stale for every time but the
    /// one the evaluator last ran, so no warming job may be built from it
    /// (see RigExecSampleFrameInputs).
    bool HasChainResolvedInputs() const;

    /// The first value sampled at \p path, or null when none was. Linear:
    /// the vector is built once and read by one job.
    const VtValue *Find(const SdfPath &path) const;

    /// Whether any value was sampled at \p path, valueless or not.
    bool Contains(const SdfPath &path) const;

    void Clear();
};

/// Flags on RigExecFrozenEvalContext::flags. Plain bits, so the context
/// stays trivially copyable.
constexpr uint32_t kRigExecFrozenPublishWeightFields = 1u << 0;
constexpr uint32_t kRigExecFrozenSolverGuidesEnabled = 1u << 1;
/// The rig refused the bake (plan D7). A worker must decline a context
/// carrying this flag: the dynamic path drives OpenExec against the live
/// stage and cannot run off the UI thread. Refusal rigs memoize UI-thread
/// results only; no background job is ever created for them.
constexpr uint32_t kRigExecFrozenBakeRefused = 1u << 2;

/// An epoch-pinned, worker-safe snapshot of a baked program.
///
/// Built on the UI thread by RigExecFreezeProgram, which deep-copies the
/// live program's epoch tables and per-frame state into `program`, nulls
/// every pointer to live evaluator state (evaluator, stage, caches,
/// evaluator-bound std::functions), and captures the small epoch values the
/// frame path reads through those pointers (the joint/solver binding map,
/// whether guide taps stand). Immutable after the freeze: shared across the
/// jobs of its epoch, read-only on every thread. A job's worker clones the
/// snapshot into private working state at job start (on the worker, off the
/// UI thread), patches the clone's varying-input constants from its sampled
/// vector, and runs the serial executor against the clone -- so the snapshot
/// itself is never mutated, and concurrent jobs never share mutable state.
///
/// The clone keeps the live program's USD handles COPIED but DEAD: no frozen
/// code path dereferences them (see the audit on RigExecFreezeProgram in
/// frozenContext.cpp), and the worker nulls every input head/query it patches
/// so a misrouted read fails closed onto the patched constant instead of
/// reaching the stage. A null or mismatched snapshot (or none at all, see
/// RigExecFrozenEvalContext::frozen) declines the job; the frame evaluates
/// live when asked.
struct RigExecFrozenProgram {
    /// The program clone: epoch tables plus the per-frame state as it stood
    /// at freeze time (the history the frozen run branches from). Live
    /// pointers nulled; see RigExecFreezeProgram.
    RigExecBakedProgramImpl program;
    /// Copy of the evaluator's joint/solver binding map (epoch data the
    /// pose epilogue reads through a live pointer).
    std::map<SdfPath, std::vector<std::pair<SdfPath, int>>> jointSolverBinding;
    /// Whether the live program had guide taps standing at freeze time.
    /// The frozen epilogue replays the guide publication from the aggregates
    /// when this and the context's guides flag are both set.
    bool guideTapsPresent = false;
    // Freeze-captured handle identities, plain data for the worker. A
    // worker must not dereference a USD handle -- not even IsValid or
    // GetPath, which reach composed specs and prim data -- so every handle
    // identity the frozen run needs is captured here on the UI thread.
    /// Every patchable input's head path, in _ForEachPatchableInput order
    /// (empty when the head is invalid; the worker declines a varying input
    /// it cannot key).
    std::vector<SdfPath> inputHeadPaths;
    /// Per constraintArrays entry, the four operator-array keys (source
    /// weights, translation offsets, rotation offsets, pole weights); empty
    /// where the entry reads no such array.
    std::vector<SdfPath> arrayKeys;
    /// Per chain, whether its base query is valid (haveBase needs it).
    std::vector<char> chainBaseQueryValid;
    /// Per derived target (derivedIndex order), same.
    std::vector<char> derivedBaseQueryValid;
    /// Per solver, whether its ribbon query is valid.
    std::vector<char> ribbonQueryValid;
    /// Per chain revision (revisionIndex order), whether the mover carries
    /// the three per-frame scalar inputs the packet assembly reads.
    std::vector<char> moverHasEnabled;
    std::vector<char> moverHasDefaultWeight;
    std::vector<char> moverHasMethod;
};

/// Freezes \p evaluator's current baked program into an epoch-pinned
/// snapshot for background warming. UI thread only: it reads the live
/// program and the stage.
///
/// Answers false, having left \p frozen untouched, when the rig cannot warm:
/// no baked program (D7), the evaluator's CPU-parity mode is on (live runs
/// dynamically, which no snapshot can reproduce), a prologue read the frozen
/// executor cannot reproduce (xform-derived seeds, native constraint
/// sources, geometry-delta bases), any constraint or property chain binding
/// a weight object (whose oracle resolves from the live stage), any
/// current-phase weight read, any revision op outside
/// skin/normals/extent/matrix/wire, any blend channel, any read phase, any
/// curvenet or driver-frames binding, a time-varying provider ladder, or an
/// unfixed skin layout. \p error, when given, says which. Anything refused
/// here evaluates live when asked; a refusal is never served wrong. Weight
/// objects and their steps DO freeze: their scalars patch from samples and
/// their point arrays sample per frame into the shared packet kernels.
///
/// Property chains are supported: the sampler refreshes their outputs for
/// the job's time through the chain-sampling hook, and the frozen prologue
/// publishes the transported results. Only a chain binding a weight object
/// refuses, because its envelope resolves through the evaluator's live
/// oracle, which no hook can reproduce.
///
/// A snapshot pins the program OBJECT it was cloned from plus the epoch it
/// was cloned in. Re-freeze after any change that rebuilds the program; a
/// change that only patches its avar constants
/// (RigExecBakedProgram::ApplyAvarValueEdits) needs only
/// RigExecPatchFrozenAvarConstants, which carries the patched region onto a
/// copy without re-cloning the epoch. The registry's session cache owns
/// that discipline in production (see RigExecImagingRegistry); direct
/// callers re-freeze around their own edits.
bool RigExecFreezeProgram(const RigExecRigEvaluator &evaluator,
                          std::shared_ptr<const RigExecFrozenProgram> *frozen,
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

/// Carries \p live's patched avar region onto a copy of \p base, for a
/// session whose snapshot still pins the program it was cloned from.
/// Copy-on-write: \p base is never mutated, so jobs already holding it run
/// on, and \p out pins the same epoch and history with the new constants.
/// Answers false, having left \p out untouched, when the two programs are
/// not the same shape (different binding counts -- a rebuild, not a patch).
/// UI thread only: it reads the live program.
bool RigExecPatchFrozenAvarConstants(
    const RigExecFrozenProgram &base, const RigExecBakedProgram &live,
    std::shared_ptr<const RigExecFrozenProgram> *out,
    std::string *error = nullptr);

/// The epoch a background job is pinned to: digests and counts only, plus
/// the frozen program reference. No member can name the stage, the
/// evaluator, or any USD object, which is what makes it safe to share with
/// a worker.
struct RigExecFrozenEvalContext {
    /// RigExecRigEvaluator::GetBindingEpochDigest at enqueue time: the epoch
    /// the job was sampled for. Enforcement is the generation fence (any
    /// edit bumps it) plus the epoch-keyed cache (a completion lands only
    /// where the UI thread looks) -- no worker compares this field.
    uint64_t epochDigest = 0;
    /// The scheduler generation the job was enqueued under. Rechecked before
    /// publish; a mismatch drops the result.
    uint64_t generation = 0;
    /// Provider slots the worker sizes its private arena for.
    size_t slotCount = 0;
    /// Identity of the baked program the job was sampled for: the epoch
    /// digest mixed with the program's bound/varying input counts. Reserved
    /// for a pre-run shape check; today the generation fence retires a job
    /// whose program was rebuilt (a rebuild follows an edit, which bumps),
    /// so no worker compares this field either.
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
/// The checks, in order: the context must not carry the D7 refusal flag; the
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
RigExecRigPose RigExecEvaluateFrozen(
    const RigExecFrozenEvalContext &context,
    const RigExecFrameInputs &inputs, RigExecFrozenStepRunner runner,
    const RigExecBackgroundScheduler *scheduler = nullptr,
    const SdfPath &rig = SdfPath());

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

/// One chain input the sampling hook reads by value, pinned the way the
/// live path pins it on its first run: the attribute and a query over it,
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
    /// The mover's bound weight objects, if any. The hook declines a chain
    /// carrying one: its envelope resolves through the evaluator's live
    /// oracle, which no caller-owned evaluation can reproduce.
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

/// One sampled property chain: its target and its revisions.
struct RigExecChainSampleChain {
    SdfPath targetPath;
    UsdAttribute target;
    UsdAttributeQuery targetQuery;
    SdfValueTypeName valueType;
    std::vector<RigExecChainSampleRevision> revisions;
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
/// object binds fine and declines at evaluation instead, so the caller
/// can tell "no chains" (empty, success) from "unchainable" (failure).
/// \p error, when given, says which.
bool RigExecBindChainSampleInputs(
    const RigExecRigEvaluator &evaluator, RigExecChainSampleBindings *out,
    std::string *error = nullptr);

/// Whether \p bindings still name \p evaluator's epoch: the same math
/// movers over the same targets of the same types, and every folded
/// constant still reading the value it was pinned with. A stage edit that
/// moves any of those answers false, and the caller rebinds. UI thread
/// only: it reads the live stage.
bool RigExecChainSampleBindingsStillCurrent(
    const RigExecChainSampleBindings &bindings,
    const RigExecRigEvaluator &evaluator);

/// Evaluates every bound chain at \p time into caller-owned state: the
/// property-chain prologue for one sampling call, run on the UI thread.
///
/// \p resolved carries the job's pre-chain overrides in and every chain
/// output out, published per target as each chain runs so a later chain
/// reads the revised value; \p results, when given, receives the final
/// value per target; \p diagnostics, when given, receives the chains'
/// lines in chain order (the first lines of the generation, as on the
/// live path). Values and lines are exactly the live prologue's for the
/// same time and overrides: the same revision loop over the same pinned
/// reads through the same property-math kernels, recomputed every call
/// (the live path's memoization republishes identical values, so skipping
/// it changes no answer).
///
/// Answers false, having left all three out-params undefined, when a
/// chain binds a weight object: the envelope resolves through the
/// evaluator's live oracle, which no caller-owned evaluation can
/// reproduce. The caller then samples through the standing resolved
/// state, marks viaChain, and declines the job -- stale chain values are
/// never warmed. \p error, when given, says which mover. UI thread only.
bool RigExecEvaluateChainsForTime(
    const RigExecChainSampleBindings &bindings, UsdTimeCode time,
    RigExecResolvedInputs *resolved,
    std::map<SdfPath, VtValue> *results,
    std::vector<std::string> *diagnostics, std::string *error = nullptr);

/// Samples one frame's input vector on the UI thread, at \p time, for the
/// program \p evaluator currently holds. Every varying binding is read
/// through the same route the baked frame path reads it -- the retained
/// query when USD alone answers, the generation's resolved inputs when a
/// property chain stands on the walk -- plus the prologue reads the bench
/// measures (blend channels, constraint operator arrays, ribbon drivers,
/// chain base points) and the standing \p overrides, which never reach the
/// stage and therefore must be sampled from the list the caller passes.
///
/// Returns false, having left \p out untouched, when no faithful vector can
/// be sampled: no baked program (a dynamic/refusal rig takes the D7 memo
/// path instead of a background job), a prologue read the sampler cannot
/// reproduce without the program's own sampling hook (xform-derived seeds,
/// native constraint sources, geometry-delta bases), or a null resolved
/// inputs pointer. \p error, when given, says which.
///
/// CHAIN-RESOLVED FRESHNESS. A chain-resolved input's value is the property
/// chain's OUTPUT at the sampled time, which only the chain prologue
/// computes. The sampler runs that prologue for the job's time on this
/// thread, into caller-owned resolved inputs seeded with the job's
/// overrides (RigExecEvaluateChainsForTime), and reads chain-resolved
/// bindings through the refreshed values -- fresh at the sampled time by
/// construction, whatever the evaluator last ran. The chain outputs and
/// their diagnostics travel with the vector (chainResults,
/// chainDiagnostics) for the frozen prologue and epilogue. Only when the
/// hook declines (a chain binding a weight object) does the sampler fall
/// back to the standing resolved state and mark the samples viaChain (see
/// RigExecSampledInput) -- and BuildWarmWork declines a vector carrying
/// one, so no background job is ever built from stale chain values.
bool RigExecSampleFrameInputs(
    const RigExecRigEvaluator &evaluator, UsdTimeCode time,
    const std::vector<RigExecValueOverride> &overrides,
    RigExecFrameInputs *out, std::string *error = nullptr);

/// Samples exactly as RigExecSampleFrameInputs, but evaluates the chains
/// through the caller's epoch-pinned \p bindings instead of binding fresh.
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
    bool placeable = false;
    bool usable = false;
    /// Per-table indices of structs with any varying-or-overridden field,
    /// in table order, so the cached sampler visits in emission order.
    /// avarBindings needs none: it holds the varying ones only.
    std::vector<size_t> ladderSites;
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
    /// Curvenet profile binds by mover path, built once per burst: the
    /// bind's every input is Default-time, so one build serves the whole
    /// range. A null holding is a remembered failed bind, served as-is.
    std::unordered_map<SdfPath, std::shared_ptr<
        const RigExecProfileMoverBinding>, SdfPath::Hash>
        curvenetBinds;
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

/// Builds the production frozen step runner: the worker-side executor the
/// C-API warming triggers run jobs under when no runner is injected.
///
/// The runner reads its program from the job's context (frozen, pinned for
/// the job's epoch) and its per-frame values from the job's sampled vector,
/// clones the snapshot into worker-owned working state, patches the clone's
/// varying-input constants from the vector, and runs the baked serial
/// executor end to end -- prologue, closure, region, epilogue -- producing
/// a pose bit-identical to live evaluation of the same inputs. One instance
/// serves every rig and every job; per-epoch and per-frame state travels in
/// the job, never in the runner.
///
/// FAIL-CLOSED WHERE IT CANNOT PROVE BIT-IDENTITY. The runner declines
/// (hands the generation back, exactly as RigExecBakedProgram::Run does
/// when it cannot complete) when the context carries no frozen program, a
/// D7 refusal, or a mismatched count; when the vector carries stale
/// chain-resolved samples or a held type the worker cannot consume; and when
/// the snapshot or the vector fails any structural cross-check. A declined
/// job completes without publishing and its frame evaluates live when
/// asked. See RigExecFreezeProgram for what a snapshot covers, and the audit
/// in frozenContext.cpp for the frozen code paths' isolation argument.
///
/// SERIAL DISCIPLINE (D4): the shared skin kernels consult
/// RigExecFrozenSerialActive at their four launch sites (moverGraph.cpp)
/// and take their serial variant inside a frozen run, so a background job
/// never dispatches TBB work past the host's own scheduling. The serial
/// and parallel variants compute byte-identical numbers by the kernels'
/// range independence, which testRigExecParallelKernels holds to account.
/// Everything else in the frozen run is serial by construction (its own
/// region loop).
RigExecFrozenStepRunner RigExecMakeProductionStepRunner();

/// Hashes \p inputs to the control-state digest the frame cache keys on
/// (Stream A feeds this into RigExecFrameCacheKey::controlDigest). Time is
/// deliberately NOT hashed: a scrub between identical control states hits
/// across times, and a control edit changes the digest at every affected
/// frame by construction. Overrides are hashed with the authored values,
/// because a drag must never be served a pre-drag pose.
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

/// Partial cone re-run slots (plan 2.0/2.2): pose-domain step state kept
/// per cached frame (geometry chains excluded -- skipped geometry serves
/// from the retained pose, run geometry overwrites; prologue writes
/// excluded -- the re-run recomputes them from the fresh vector).
struct RigExecPartialSlots {
    std::vector<float> poseWeights;
    std::vector<RigExecWeightPacket> weightPackets;
    std::vector<GfMatrix4d> posedM;
    std::vector<GfMatrix4d> finalMatrix;
    std::vector<GfMatrix4d> baseMatrix;
    std::vector<RigExecPointFrame> base;
    std::vector<RigExecPointFrame> fin;
    std::vector<RigExecPointFrameArray> aggregates;
    std::vector<GfMatrix4d> deltaValues;
    std::vector<char> deltaPresent;
    struct SolverSlots {
        std::vector<RigExecPointFrame> outFrames;
        std::vector<char> outPresent;
        std::vector<SdfPath> fallbackJoints;
    };
    std::vector<SolverSlots> solvers;
    struct CommitSlots {
        std::vector<char> present;
        std::vector<char> deltaOk;
        std::vector<RigExecPointFrame> frames;
        std::vector<RigExecPointFrame> staged;
        std::vector<GfMatrix4d> deltas;
        std::vector<uint8_t> outcome;
        std::vector<RigExecConstraintSource> sources;
        bool abandoned = true;
    };
    std::vector<CommitSlots> commits;
    struct StepSlots {
        std::vector<std::string> diagnostics;
        RigExecBakedStepCounters counters;
        bool bail = false;
    };
    std::vector<StepSlots> steps;
    std::map<SdfPath, GfMatrix4d> volumeWeightMatrices;
    void Capture(const RigExecBakedProgramImpl &program);
    bool Restore(RigExecBakedProgramImpl *program) const;
    size_t Bytes() const;
};

bool RigExecCapturePartialSlots(
    const RigExecBakedProgramImpl &program,
    std::shared_ptr<const void> *slotsOut, size_t *bytesOut);
bool RigExecTakeLastFrozenSlots(std::shared_ptr<const void> *slotsOut,
                              size_t *bytesOut);
struct RigExecPartialRunResult {
    RigExecRigPose pose;
    std::shared_ptr<const void> slots;
    size_t slotBytes = 0;
    std::vector<int> executedClusters;
    bool completed = false;
};
RigExecPartialRunResult RigExecRunPartialCone(
    const RigExecFrozenProgram &snapshot,
    const RigExecFrameInputs &freshInputs,
    const std::shared_ptr<const void> &baseSlots,
    const RigExecRigPose &basePose,
    const RigExecBakedClusterSet &planClusters, uint32_t contextFlags);

}  // namespace rigExec

#endif  // RIGEXEC_FROZEN_CONTEXT_H
