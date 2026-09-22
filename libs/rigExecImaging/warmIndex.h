//
// RigExec warm-frame index: the shared-owned completed-per-frame store
// behind the per-frame query API (plan 1.4).
//
// One index serves every rig in the registry. It records, per (rig, time):
//
//   * the publish completion (key + generation), fed by publish-confirmed
//     completions recording into shared state -- the worker closure on its
//     Published outcome, the UI thread on a memoized publish -- and retired
//     by eviction through the cache's affected-key reporting (a drained OLD
//     key for a republished time retires nothing) and by Clear;
//   * the queue visibility (queued/running), fed by the scheduler's
//     per-frame transition hooks.
//
// Dirtiness is computed, not stored: a completion under an older generation
// or epoch than the query's is dirty (global until Stream 2 scopes it).
// Proofs are NOT consulted here -- no per-frame sampling on the query path,
// and no callbacks into bridges, which sessions may destroy under live jobs.
//
// Locking: the index mutex is a leaf, taken under the registry mutex (query,
// memoize, reset), under the scheduler mutex (transition hooks), and alone
// (worker completions, eviction callbacks, which run outside every cache
// lock). It never takes another lock and never calls out.
//
#ifndef RIGEXEC_IMAGING_WARM_INDEX_H
#define RIGEXEC_IMAGING_WARM_INDEX_H

#include "rigExec/backgroundScheduler.h"
#include "rigExec/frameCache.h"

#include <cstdint>
#include <map>
#include <mutex>
#include <set>
#include <utility>
#include <vector>

namespace rigExec {

/// One frame's warming state, for the strip and the C API's ints (in order).
enum class RigExecWarmFrameState {
    /// No completion, nothing in flight.
    Uncached = 0,
    /// Queued or running under the live queue.
    Warming = 1,
    /// A completion under the query's generation and epoch.
    Cached = 2,
    /// A completion under an older generation or epoch: re-warm to serve.
    Dirty = 3,
};

/// Consecutive non-publish signals (declined-invalid finishes, reasoned
/// factory skips) after which a frame counts un-warmable: the cursor skips
/// it without sampling. Revisited on edit (generation push), on eviction
/// (retired frames re-pend), and on publish (which resets the streak).
constexpr size_t kRigExecWarmUnwarmableAfter = 3;

/// Shared-owned per-frame warming index. Thread-safe; every method takes
/// only the index mutex. See the file header for the lock discipline.
class RigExecWarmFrameIndex {
public:
    /// Records the rig's current warming generation (the registry pushes on
    /// every cancel; 0 for a rig never pushed). Also clears the rig's
    /// non-publish streaks: the edit may have fixed warmability.
    void NoteGeneration(const SdfPath &rig,
                        RigExecFrameGeneration generation);
    /// Records a reasoned factory skip for the time (a non-publish signal).
    void NoteSkipped(const SdfPath &rig, double timeValue);

    /// Whether the time's non-publish streak reached
    /// kRigExecWarmUnwarmableAfter.
    bool IsUnwarmable(const SdfPath &rig, double timeValue) const;

    /// Whether the cursor should visit the time under the query's
    /// generation and epoch: not un-warmable, and no current completion.
    /// One call (one lock) per candidate per trigger.
    bool IsVisitable(const SdfPath &rig, double timeValue,
                     RigExecFrameGeneration generation,
                     uint64_t epochDigest) const;
    RigExecFrameGeneration CurrentGeneration(const SdfPath &rig) const;

    /// Records a publish completion: \p timeValue now holds \p key under \p
    /// generation. Overwrites any earlier record for the time -- a republish
    /// under a new key retires the old by replacement -- and clears the
    /// path-scoped dirty flag.
    void NoteCompleted(const SdfPath &rig, double timeValue,
                       const RigExecFrameCacheKey &key,
                       RigExecFrameGeneration generation);

    /// Marks the time's completion path-scoped dirty (plan 2.1): a
    /// patch/stamp-bump edit retired it without bumping the generation.
    /// Records nothing when the time holds no completion (nothing cached
    /// reads uncached, never dirty). Cleared by the next completion,
    /// eviction, or reset.
    void NoteDirtied(const SdfPath &rig, double timeValue);

    /// Every time value holding a completion for the rig, in index order.
    /// The scoped-cancel path uses these (plus the scheduler's queued
    /// times) as the affected set.
    std::vector<UsdTimeCode> CompletedTimes(const SdfPath &rig) const;

    /// Every (time, key) completion for the rig, in index order. The 2.2
    /// re-resolve lanes read the recorded key per time and partition by
    /// entry provenance: clean entries carry under a new key, dirty ones
    /// retire. A completion evicted concurrently reads under its drained
    /// key and degrades to the retire lane, never to a wrong base.
    std::vector<std::pair<UsdTimeCode, RigExecFrameCacheKey>> CompletedKeys(
        const SdfPath &rig) const;

    /// Answers the recorded completion key for (\p rig, \p timeValue), or
    /// false when the time holds no completion. The sparse-serve path's
    /// same-time base lookup: one call (one lock) per lookup miss.
    bool FindKey(const SdfPath &rig, double timeValue,
                 RigExecFrameCacheKey *key) const;

    /// Answers the recorded completion key when the time's completion is
    /// servable under (\p generation, \p epochDigest): recorded, clean,
    /// and under both tags. Queued/running transitions do NOT block: an
    /// in-flight re-warm leaves the standing completion servable until
    /// its publish replaces the row. One call (one lock) per lookup; the
    /// serve path falls through to sampling on false AND on a store miss
    /// under the answered key (a concurrently evicted entry).
    bool FindCachedKey(const SdfPath &rig, double timeValue,
                       RigExecFrameGeneration generation,
                       uint64_t epochDigest,
                       RigExecFrameCacheKey *key) const;

    /// Retires the time's completion when \p key is the recorded one; a
    /// drained old key for a republished time retires nothing.
    void NoteEvicted(const SdfPath &rig, const RigExecFrameCacheKey &key,
                     double timeValue);

    /// Records one scheduler queue transition for the time.
    void NoteTransition(const RigExecWarmTransition &transition);

    /// Drops every record for the rig (Clear).
    void ResetRig(const SdfPath &rig);
    void NotePartialDeclined(const SdfPath &rig, double timeValue);
    bool TakePartialFallback(const SdfPath &rig, double timeValue);

    /// One frame's state under the caller's current (\p generation, \p
    /// epochDigest): warming while queued or running; cached when the
    /// recorded completion is under both; dirty when a completion stands
    /// under an older generation or epoch; uncached otherwise.
    RigExecWarmFrameState State(const SdfPath &rig, double timeValue,
                                RigExecFrameGeneration generation,
                                uint64_t epochDigest) const;

    /// Batched states for \p timeValues, in order -- one call per strip
    /// repaint, no sampling on the query path.
    std::vector<RigExecWarmFrameState> States(
        const SdfPath &rig, const std::vector<double> &timeValues,
        RigExecFrameGeneration generation, uint64_t epochDigest) const;

private:
    struct _Frame {
        bool hasCompletion = false;
        RigExecFrameCacheKey key{0, 0};
        RigExecFrameGeneration completedGeneration = 0;
        /// Path-scoped dirtiness (plan 2.1): set by NoteDirtied, cleared by
        /// NoteCompleted, NoteEvicted, and ResetRig. Read only beside a
        /// completion -- see _StateFor.
        bool dirty = false;
        bool queued = false;
        size_t running = 0;
        size_t streak = 0;
    };

    // The state rule over a found-or-absent record. Call with _mutex held.
    static RigExecWarmFrameState _StateFor(
        const _Frame *frame, RigExecFrameGeneration generation,
        uint64_t epochDigest);

    mutable std::mutex _mutex;
    std::map<std::pair<SdfPath, double>, _Frame> _frames;
    std::map<SdfPath, RigExecFrameGeneration> _generations;
    std::set<std::pair<SdfPath, double>> _partialFallback;
};

}  // namespace rigExec

#endif  // RIGEXEC_IMAGING_WARM_INDEX_H
