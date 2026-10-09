// Retained source snapshots support whole-pose cache reuse proofs.
// A changed output is evaluated by the normal persistent graph workspace.
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

// D2 go/no-go, from reports/frame-cache-measurements.md.

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

// Retained cross-frame state.

/// Sampled sources and epoch metadata used to prove whole-pose cache reuse.
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
    /// The source snapshot's accounted bytes: the retained
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

/// Whether two upstream tables (RigExecFrameInputs::upstream, sorted by
/// path) hold the same values: path by path, the fold hash, then
/// RigExecSameSourceValue.
bool RigExecSameUpstream(const std::vector<RigExecUpstreamValue> &a,
                         const std::vector<RigExecUpstreamValue> &b);

/// The controls whose value moved between the retained frame and the
/// request: sampled sources by first-wins-per-path (matching what the
/// worker reads), overrides by (prim, computation, attribute) last-wins
/// (matching the evaluator's replace rule). Sorted, deduplicated.
std::vector<RigExecControlId> RigExecChangedControls(
    const RigExecRetainedFrameState &cached,
    const RigExecFrameInputs &requested,
    const std::vector<RigExecValueOverride> &requestedOverrides);

/// Whether the sampled request proves the retained whole pose reusable.
/// Any affected output falls through to normal graph evaluation, and so
/// does any difference in the upstream values or in the listed external
/// inputs (RigExecFrameInputs::varyingRevisionLeaves), which no control id
/// names.
bool RigExecCanReuseRetainedPose(
    const RigExecOutputAffectedIndex &index,
    const RigExecRetainedFrameState &cached, uint64_t requestEpoch,
    const RigExecFrameInputs &requested,
    const std::vector<RigExecValueOverride> &requestedOverrides);

// Entry provenance and retained-state rebind (plan 2.0).

/// The provenance of a full evaluation over \p program: every cluster, every
/// weight object the program reaches, and every constant region, with \p
/// time as the first alias. Sorted and unique throughout.
RigExecEntryProvenance RigExecFullEvalProvenance(
    const RigExecBakedProgramImpl &program, UsdTimeCode time);

/// Captures the retained frame state for a just-evaluated frame: the sampled
/// inputs and standing overrides by value, plus the epoch, the
/// epoch-constant digest, and the cluster count the plan checks.
/// Retains sampled inputs only; evaluation uses the normal frozen workspace.
RigExecRetainedFrameState RigExecCaptureRetainedState(
    const RigExecFrameInputs &inputs,
    const std::vector<RigExecValueOverride> &overrides,
    uint64_t epochDigest, size_t clusterCount,
    uint64_t constantDigest = 0);

// Candidate lookup: which retained frame a request reuses.

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

// Epoch invalidation through the baked capture index.

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

// RIGEXEC_FRAME_CACHE_VERIFY shadow mode.

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

/// Compares \p cached against a live evaluation through RigExecComparePoses.
/// A null or invalid live pose is unverified, not a match.
RigExecShadowVerdict RigExecVerifyHitWithLive(
    const RigExecRigPose &cached, RigExecLiveRunner live);

}  // namespace rigExec

#endif  // RIGEXEC_FRAME_CACHE_SPARSITY_H
