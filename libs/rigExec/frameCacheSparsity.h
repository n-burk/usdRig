//
// RigExec sparse cross-frame reuse: a cached frame as the base of the next.
//
// Within one frame the baked step graph already runs only the closure of
// value-changed sources over clusters (§7). This module extends that closure
// ACROSS frames: a cached frame retains its source values (plus the slot
// state a re-run executes against), a lookup compares the request's sources
// by value, a frame whose cone is empty is a hit, and a partial cone re-runs
// only affected clusters against otherwise-retained slots.
//
// Whether that retention fits the byte cap is Stream 0's go/no-go (plan D2),
// answered below from the measured numbers: GO on both measured rigs, so
// this module ships the reuse, not whole-pose memo only. The no-go endpoint
// stays available -- RigExecDecideSparsity answers it for any rig -- and the
// output-affected index ships either way, since warming selection needs it.
//
// How the pieces compose (Stream E wires this into the imaging chain):
//
//   exact key hit          serve the pose, zero work (Stream A path)
//   retained base + empty cone   serve the retained pose, zero work
//   retained base + cone         re-run the plan's clusters, publish afresh
//   no retained base             live eval (the pool never serves)
//
// The re-run itself is the caller's: the plan names the clusters, a
// RigExecClusterRunner executes them, and RigExecRunSparsePlan counts what
// ran. The slot snapshot the runner executes against is captured by whoever
// owns the program (Stream E) and carried here as an opaque handle with
// accounted bytes -- this module never names a program slot.
//

#ifndef RIGEXEC_FRAME_CACHE_SPARSITY_H
#define RIGEXEC_FRAME_CACHE_SPARSITY_H

#include "bakedProgram.h"
#include "frameCache.h"
#include "frozenContext.h"
#include "outputAffectedIndex.h"
#include "taskListCache.h"

#include "pxr/usd/usd/notice.h"

#include <cstddef>
#include <cstdint>
#include <functional>
#include <map>
#include <memory>
#include <mutex>
#include <string>
#include <utility>
#include <vector>

PXR_NAMESPACE_USING_DIRECTIVE

namespace rigExec {

// ---------------------------------------------------------------------------
// D2 go/no-go, from reports/frame-cache-measurements.md.
// ---------------------------------------------------------------------------

/// Per-frame stored bytes, pose maps and slot arenas separately: biped
/// 430,258 + 2,767,505 (3.05 MiB full), 9mesh 447,038 + 1,796,703.
constexpr size_t kRigExecSparsityStream0BipedPoseBytes = 430258;
constexpr size_t kRigExecSparsityStream0BipedArenaBytes = 2767505;
constexpr size_t kRigExecSparsityStream0Mesh9PoseBytes = 447038;
constexpr size_t kRigExecSparsityStream0Mesh9ArenaBytes = 1796703;

/// A "useful frame count" is the scrub neighborhood: +-8 around the playhead.
/// The sweep beyond it is headroom, not the bar retention has to clear.
constexpr size_t kRigExecSparsityMinUsefulFrames = 16;

/// What the D2 decision says, and the numbers that said it.
struct RigExecSparsityDecision {
    bool go = false;
    /// Full (pose + arena) frames the cap holds at these bytes.
    size_t fullFramesAtCap = 0;
    /// One paragraph: the bytes, the cap, the count, the verdict.
    std::string memo;
};

/// GO when \p byteCap holds at least \p minFrames full frames at
/// (\p poseBytes + \p arenaBytes) each; whole-pose memo otherwise. Zero
/// stored bytes is never GO: there is no measurement to decide from.
RigExecSparsityDecision RigExecDecideSparsity(
    size_t arenaBytes, size_t poseBytes, size_t byteCap,
    size_t minFrames = kRigExecSparsityMinUsefulFrames);

/// The decision on the Stream 0 biped numbers at the default cap: GO, ~80
/// full frames. A test holds this to GO so a cap change revisits D2 aloud
/// rather than silently pricing reuse out.
RigExecSparsityDecision RigExecStream0SparsityDecision();

// ---------------------------------------------------------------------------
// Retained cross-frame state.
// ---------------------------------------------------------------------------

/// What one cached frame keeps so a later request can reuse it: the source
/// values the request is compared against by VALUE (§7's rule -- never "the
/// time changed"), the epoch and cluster count the plan checks for reuse,
/// and the slot snapshot a partial cone executes against.
///
/// The slots are opaque here on purpose: only the program's owner can capture
/// them, and only it can run a cluster against them. This module accounts
/// their bytes and carries the handle; Stream E captures and executes.
struct RigExecRetainedFrameState {
    RigExecFrameInputs inputs;
    std::vector<RigExecValueOverride> overrides;
    uint64_t epochDigest = 0;
    /// The epoch-constant digest (RigExecEpochConstantDigest) the frame was
    /// retained under. Sampled values exclude avar constants by design, so
    /// a value comparison alone cannot see a constant patch: reuse plans
    /// against this base only when the live program's constant digest
    /// still agrees, and live-evaluates (or carries, plan 2.2) otherwise.
    /// Zero when the capturer did not record one: plans only against a
    /// live program whose digest is also zero, i.e. never in production.
    uint64_t constantDigest = 0;
    size_t clusterCount = 0;
    /// Caller-measured bytes of the slot snapshot. Zero means pose-plus-
    /// sources only: plannable, but a Partial verdict has nothing to
    /// execute against and the caller must live-eval instead.
    size_t slotBytes = 0;
    std::shared_ptr<const void> slots;

    /// slotBytes plus the source snapshot's accounted bytes: the retained
    /// half of the entry's cap accounting, beside the pose's.
    size_t RetainedBytes() const;
};

/// Accounted bytes of the source snapshot: path strings plus value payloads
/// by the bench's counting. An estimate the cap holds monotonically, not a
/// heap measure.
size_t RigExecRetainedSourcesBytes(const RigExecRetainedFrameState &state);

/// Whether two source samples are the same value: presence and hasValue
/// first, then VtValue equality. A NaN compares unequal to everything,
/// itself included -- conservative (a re-run, never a skip).
bool RigExecSameSourceValue(const VtValue &a, bool aHas, const VtValue &b,
                            bool bHas);

/// The controls whose value moved between the retained frame and the
/// request: sampled sources by first-wins-per-path (matching what the
/// worker reads), overrides by (prim, computation, attribute) last-wins
/// (matching the evaluator's replace rule). Sorted, deduplicated.
std::vector<RigExecControlId> RigExecChangedControls(
    const RigExecRetainedFrameState &cached,
    const RigExecFrameInputs &requested,
    const std::vector<RigExecValueOverride> &requestedOverrides);

// ---------------------------------------------------------------------------
// Planning and execution.
// ---------------------------------------------------------------------------

/// What reuse decided.
enum class RigExecSparseVerdict {
    /// Nothing moved, or nothing it reaches was requested: serve the
    /// retained pose with zero cluster work.
    Hit,
    /// The caller's choice: re-run `clusters` against retained slots
    /// (or live-eval when there are no retained slots to run against).
    Partial,
    /// No reuse: no retained base, an epoch or topology mismatch, or an
    /// empty index. Live-eval.
    Miss,
};

/// The reuse plan for one request against one retained frame.
struct RigExecSparsePlan {
    RigExecSparseVerdict verdict = RigExecSparseVerdict::Miss;
    /// Clusters to run. Set only on Partial; empty on Hit by construction.
    RigExecBakedClusterSet clusters;
    /// What moved, as RigExecChangedControls reported it.
    std::vector<RigExecControlId> changedControls;
    /// A memoized selection served at least one changed control.
    bool memoUsed = false;
    /// The retained frame's topology is not the request's (cluster count
    /// moved under a standing epoch). The caller should invalidate the
    /// epoch; the verdict is already Miss.
    bool topologyChanged = false;

    size_t ClustersToRun() const { return clusters.Count(); }
    size_t ClustersSkipped(size_t clusterCount) const
    {
        const size_t run = ClustersToRun();
        return run < clusterCount ? clusterCount - run : 0;
    }
};

/// Plans the request against the retained frame.
///
/// The checks, in order: the index must be built for \p requestEpoch and
/// the retained frame must belong to it, with the index's cluster count --
/// anything else is Miss (a count mismatch under a standing epoch also sets
/// topologyChanged). The changed controls are diffed by value; none changed
/// at a standing time is a Hit, none changed at a moved time still re-runs
/// the always-dirty set plus the varying closure (§7: external reads are
/// functions of time the source vector does not name, and the executor
/// dirties varying-step clusters on a moved time). Standing overrides
/// dirty the override closure on top. Changed controls map to seeds
/// through the
/// index -- one memoized selection per control when \p memo is given, so a
/// repeated edit rewalks nothing -- union to a dirty closure, and intersect
/// `dirty ∩ affecting(requested)`: empty is a Hit, otherwise Partial.
///
/// An unknown control contributes every cluster (the index's conservative
/// answer), so a Partial plan always covers what moved; re-running it is
/// bit-identical to live eval, and running FEWER clusters than it names is
/// never correct.
RigExecSparsePlan RigExecPlanSparseReuse(
    const RigExecOutputAffectedIndex &index, RigExecTaskListCache *memo,
    const RigExecRetainedFrameState &cached, uint64_t requestEpoch,
    const RigExecFrameInputs &requested,
    const std::vector<RigExecValueOverride> &requestedOverrides,
    const RigExecBakedClusterSet *affectingRequested = nullptr);

/// Runs one planned cluster, returning false to hand the generation back
/// (the runner's caller live-evals instead).
using RigExecClusterRunner = std::function<bool(int cluster)>;

/// What a plan's execution ran, in the order it ran.
struct RigExecSparseExecution {
    size_t executed = 0;
    std::vector<int> executedClusters;
    /// False when the runner handed the generation back partway.
    bool completed = true;
};

/// Executes \p plan's clusters through \p runner, one at a time, in
/// \p clusterOrder -- the program's `clustering.topologicalOrder`, so that
/// every planned cluster runs after its planned predecessors. A Miss
/// executes nothing and reports incomplete: there is no retained base to
/// run against, so the caller live-evals. A null runner, or an order that
/// leaves out a planned cluster, also executes nothing and reports
/// incomplete: running fewer clusters than the plan names is never correct.
RigExecSparseExecution RigExecRunSparsePlan(
    const RigExecSparsePlan &plan, const std::vector<int> &clusterOrder,
    RigExecClusterRunner runner);

// ---------------------------------------------------------------------------
// Entry provenance and retained-state rebind (plan 2.0).
// ---------------------------------------------------------------------------

/// The provenance of a full evaluation over \p program: every cluster, every
/// weight object the program reaches, and every constant region, with \p
/// time as the first alias. Sorted and unique throughout. A partial cone
/// re-run records the subset it ran instead (plan 2.2).
RigExecEntryProvenance RigExecFullEvalProvenance(
    const RigExecBakedProgramImpl &program, UsdTimeCode time);

/// Captures the retained frame state for a just-evaluated frame: the sampled
/// inputs and standing overrides by value, plus the epoch, the
/// epoch-constant digest, and the cluster count the plan checks.
/// Sources-only (no slot snapshot): plannable, but a Partial verdict has
/// nothing to execute against and the caller must live-eval instead -- see
/// RigExecRetainedFrameState::slotBytes.
RigExecRetainedFrameState RigExecCaptureRetainedState(
    const RigExecFrameInputs &inputs,
    const std::vector<RigExecValueOverride> &overrides,
    uint64_t epochDigest, size_t clusterCount,
    uint64_t constantDigest = 0);

/// The context a partial cone re-run executes under (plan 2.0): a per-job
/// clone of the frozen program with the retained handle rebound into it,
/// plus the production cluster runner the owner supplies. The clone keeps
/// the frozen program's shape; the retained handle carries the source
/// snapshot (and, once slot capture lands, the slot arena) the re-run
/// reads; the runner executes one cluster against the rebound clone.
struct RigExecClusterRebindContext {
    std::shared_ptr<RigExecBakedProgramImpl> program;
    std::shared_ptr<const void> retained;
    std::function<bool(RigExecBakedProgramImpl &, int)> runCluster;
};

/// Adapts a rebind context to the cluster runner RigExecRunSparsePlan
/// executes: a planned cluster runs through the context's production
/// runner against its rebound program. Answers false (hands the
/// generation back) when the context holds no program or no runner.
RigExecClusterRunner RigExecMakeClusterRunner(
    RigExecClusterRebindContext context);

// ---------------------------------------------------------------------------
// Candidate lookup: which retained frame a request reuses.
// ---------------------------------------------------------------------------

/// The (epoch, time) -> key sidecar. A re-warm targets a TIME; the stale
/// entry at that time is the ideal reuse base (same time-varying inputs,
/// only the edited control moved). Entries evicted from the cache leave
/// stale keys here, which read as Miss -- never as a wrong base -- when the
/// retained lookup finds nothing under them.
class RigExecSparseCandidateIndex {
public:
    RigExecSparseCandidateIndex() = default;

    RigExecSparseCandidateIndex(const RigExecSparseCandidateIndex &) = delete;
    RigExecSparseCandidateIndex &operator=(
        const RigExecSparseCandidateIndex &) = delete;

    /// Records that \p key was published for (\p epoch, \p time).
    void NotePublished(uint64_t epoch, UsdTimeCode time,
                       const RigExecFrameCacheKey &key);

    /// Answers the key published for (\p epoch, \p time), or false. A null
    /// out-param misses rather than faults.
    bool Find(uint64_t epoch, UsdTimeCode time,
              RigExecFrameCacheKey *key) const;

    /// Drops the key for (\p epoch, \p time), if any.
    bool Drop(uint64_t epoch, UsdTimeCode time);

    void Clear();
    size_t Size() const;

private:
    mutable std::mutex _mutex;
    std::map<std::pair<uint64_t, UsdTimeCode>, RigExecFrameCacheKey> _map;
};

// ---------------------------------------------------------------------------
// Epoch invalidation through the baked capture index.
// ---------------------------------------------------------------------------

/// Drops every cached frame and memoized selection of \p epochDigest,
/// returning cached frames dropped (memo drops are counted on the memo).
size_t RigExecInvalidateEpoch(RigExecFrameCache &cache,
                             RigExecTaskListCache &memo,
                             uint64_t epochDigest);

/// Routes \p notice through the baked capture index: a hit drops the
/// epoch's frames and memo and answers true; a miss leaves every cached
/// frame standing (their digests decide reachability, plan D1) and answers
/// false. The program's IsInvalidatedBy is the whole oracle -- no second
/// derivation of what a notice touches.
bool RigExecNoteCaptureIndex(RigExecFrameCache &cache,
                             RigExecTaskListCache &memo,
                             uint64_t epochDigest,
                             const RigExecBakedProgram &program,
                             const UsdNotice::ObjectsChanged &notice);

// ---------------------------------------------------------------------------
// RIGEXEC_FRAME_CACHE_VERIFY shadow mode.
// ---------------------------------------------------------------------------

/// Whether RIGEXEC_FRAME_CACHE_VERIFY asks every cache hit to also
/// live-evaluate and diff. Read fresh on each call -- unlike
/// RIGEXEC_BAKED_VERIFY_CONES' cached read -- so tests toggle it in-process
/// and production pays the lookup only on the shadow path.
bool RigExecFrameCacheVerifyRequested();

/// A live runner for the shadow: evaluates the request live, the way a
/// cache miss would have.
using RigExecLiveRunner = std::function<RigExecRigPose()>;

/// What the shadow comparison said.
struct RigExecShadowVerdict {
    bool match = false;
    size_t mismatches = 0;
    /// The pose to serve: the cached one on a match, the live one on a
    /// mismatch (a plausible wrong pose is never served), the cached one
    /// with match false when the live runner failed or was null (the
    /// shadow is advisory -- it substitutes only a proven live pose).
    RigExecRigPose poseToServe;
    /// One line per mismatch domain, in RigExecComparePoses' words.
    std::string report;
};

/// Compares \p cached against a live evaluation through RigExecComparePoses
/// -- the same judge as BakedWithParityCheck -- with the live pose as the
/// reference. A null or invalid live pose is unverified, not a match.
RigExecShadowVerdict RigExecVerifyHitWithLive(
    const RigExecRigPose &cached, RigExecLiveRunner live);

}  // namespace rigExec

#endif  // RIGEXEC_FRAME_CACHE_SPARSITY_H
