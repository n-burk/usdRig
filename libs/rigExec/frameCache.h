//
// RigExec per-frame cache: evaluated poses keyed by what they are a function
// of, not by when they were asked for.
//
// A cached pose is a pure function of its sampled inputs, so the key is the
// pair (bindingEpochDigest, controlStateDigest): the epoch the program was
// built for, and a hash over every source value the frame actually read,
// including interactive overrides (which never reach the stage). Time is
// carried on the entry as a warming hint and for stats, never as part of the
// key -- a scrub between identical control states hits across times, and a
// control edit changes the digest at every affected frame by construction,
// so stale entries become unreachable and LRU reclaims them without any
// frame-level invalidation bookkeeping.
//
// Memory is bounded by a per-rig byte cap with LRU eviction. Corrupt or
// undersized entries are evicted, never partially served: an unevaluated
// result is recoverable, a plausible wrong one is not.
//
// Threading: the store is sharded -- sixteen key partitions behind sixteen
// locks, plus lock-free counters -- so Lookup runs concurrently with
// background Publish without touching the registry mutex. Every lock is held
// only for a pointer swap or a map edit, never across evaluation, payload
// allocation, pose copying, or digest computation: Lookup copies a shared
// handle under the shard lock and the pose outside it, and Publish builds
// the entry (bytes, copy, digest) before taking the lock, so a below-normal
// worker can never stall a UI-thread lookup.
//
#ifndef RIGEXEC_FRAME_CACHE_H
#define RIGEXEC_FRAME_CACHE_H

#include "rigEvaluator.h"

#include "pxr/usd/usd/timeCode.h"

#include <array>
#include <atomic>
#include <cstddef>
#include <cstdint>
#include <functional>
#include <memory>
#include <mutex>
#include <unordered_map>
#include <vector>

PXR_NAMESPACE_USING_DIRECTIVE

namespace rigExec {

struct RigExecFrameInputs;
struct RigExecSampledInput;
struct RigExecBurstSampleCache;

/// Default per-rig byte cap: 1 GiB holds the 200-frame stack full range
/// (slot-backed retained state, ~3.1 MiB/frame) plus headroom. The 512 MiB
/// sources-only sizing thrashed once Stream 2 attached per-frame slots,
/// never converging to cached.
constexpr size_t kRigExecFrameCacheDefaultByteCap = 1024 * 1024 * 1024;

/// What a cached pose is a function of. Time is deliberately absent: see the
/// file header.
struct RigExecFrameCacheKey {
    uint64_t epochDigest = 0;
    uint64_t controlDigest = 0;

    bool operator==(const RigExecFrameCacheKey &other) const
    {
        return epochDigest == other.epochDigest &&
               controlDigest == other.controlDigest;
    }

    bool operator!=(const RigExecFrameCacheKey &other) const
    {
        return !(*this == other);
    }

    bool operator<(const RigExecFrameCacheKey &other) const
    {
        if (epochDigest != other.epochDigest) {
            return epochDigest < other.epochDigest;
        }
        return controlDigest < other.controlDigest;
    }
};

/// Hash for keyed containers. Equal keys hash equal; the value is stable
/// within a process and is not persisted.
struct RigExecFrameCacheKeyHash {
    size_t operator()(const RigExecFrameCacheKey &key) const noexcept
    {
        // 64-bit FNV-1a over the two digests.
        uint64_t hash = 1469598103934665603ull;
        hash ^= key.epochDigest;
        hash *= 1099511628211ull;
        hash ^= key.controlDigest;
        hash *= 1099511628211ull;
        return size_t(hash);
    }
};

/// One stored generation. The time is the frame the pose was evaluated at:
/// a warming hint and a stat, never key material.
struct RigExecFrameCacheEntry {
    UsdTimeCode time = UsdTimeCode::Default();
    RigExecRigPose pose;
    /// Accounted payload bytes of this entry (pose maps plus, when Stream D
    /// retains it, the slot arena). Counted at Publish; eviction compares
    /// against the byte cap.
    size_t bytes = 0;
};

/// Lifetime counters. Every counter only moves forward except across Clear.
struct RigExecFrameCacheStats {
    size_t hits = 0;
    size_t misses = 0;
    size_t published = 0;
    size_t evictions = 0;
    size_t entryCount = 0;
    /// Payload bytes currently held. Each Publish links its entry and then
    /// evicts back under the cap, so a burst of concurrent publishers can
    /// overshoot by that burst transiently; an entry larger than the cap by
    /// itself is dropped rather than stored.
    size_t bytes = 0;
};

/// The control half of the cache key: a hash over the frame's source values
/// actually read -- the sampled input vector the UI thread built at enqueue
/// time, plus the evaluator's interactive overrides, which never reach the
/// stage and so must be folded in explicitly. A scrub between identical
/// control states digests equal across times (time is not folded in) and a
/// drag over a control digests different at every affected frame.
///
/// Order-independent (sources sort by path, overrides by prim/computation/
/// attribute) and first-wins per path, matching what the worker reads. A
/// source that held no value digests distinctly from every valued source.
/// Floating point folds bitwise -- including +0.0 versus -0.0, which TfHash
/// conflates but the evaluator need not -- so two inputs digest equal only
/// when they hold equal types and equal bits. The stage seeds fold after
/// the values, in program order: they are fresh stage reads, so a moved
/// constraint target moves the digest.
///
/// Within-process only, like the key hash: never persisted.
uint64_t RigExecControlStateDigest(const RigExecFrameInputs &inputs);
uint64_t RigExecControlStateDigest(
    const RigExecFrameInputs &inputs,
    const std::vector<RigExecValueOverride> &overrides);

/// Level-1 digest of one sampled input: its path, its valuelessness, and
/// its value folded exactly as the control digest folds one sample. The
/// control digest chains one level-1 per first-win path; the burst build
/// memoizes the level-1s of time-invariant samples and folds only the
/// rest per frame. Within-process only, like the key hash.
uint64_t RigExecSampleDigest(const RigExecSampledInput &sample);

/// Folds an epoch-constant digest (plan D3: RigExecEpochConstantDigest, the
/// value-sensitive avar-constant region the epoch digest no longer covers)
/// into a control digest, under its own domain tag. The fold is
/// unconditional -- every production control digest for a baked program
/// passes through it, so a constant patch opens a new key namespace while
/// the epoch half stays standing. Within-process only, like the key hash.
uint64_t RigExecFoldConstantDigest(uint64_t controlDigest,
                                  uint64_t constantDigest);

/// The control half of the cache key with epoch-constant coverage: the
/// plain control digest folded with \p constantDigest. Lookup, memoization,
/// proofs, and warming keys all derive through this for a baked program;
/// the D7 refusal path (no program, no constants) does not. Two
/// constant-states never share a key, so a patch aliases nothing.
uint64_t RigExecControlStateDigestWithConstants(
    const RigExecFrameInputs &inputs,
    const std::vector<RigExecValueOverride> &overrides,
    uint64_t constantDigest);

/// The control-state digest of a burst-cached vector: identical to
/// RigExecControlStateDigest for the same inputs, but level-1s served
/// from the burst's static maps fold from the memo instead of the bytes,
/// and the sort order recorded from the burst's first frame replaces the
/// per-frame sort. Falls back to the plain digest (same answer, full
/// cost) whenever the cache cannot serve the call: a foreign override
/// list, or a vector whose shape differs from the recorded order's.
/// \p cache is updated (memo fills, order records) and so is non-const.
uint64_t RigExecControlStateDigestWithBurstCache(
    const RigExecFrameInputs &inputs,
    const std::vector<RigExecValueOverride> &overrides,
    RigExecBurstSampleCache *cache);

/// Whether every value in \p inputs and \p overrides can be digested exactly.
/// An unhashable held type (one VtValue cannot hash and the digest has no
/// bitwise fold for) answers false, and the frame must bypass the cache --
/// the digest still folds the type name so it stays defined, but two
/// different values of that type would digest equal, which is a plausible
/// wrong pose rather than a miss.
bool RigExecControlStateDigestible(const RigExecFrameInputs &inputs);
bool RigExecControlStateDigestible(
    const RigExecFrameInputs &inputs,
    const std::vector<RigExecValueOverride> &overrides);

/// The control half of the cache key for a rig with no baked program (plan
/// D7: a bake refusal, or an explicitly dynamic rig), which cannot sample a
/// RigExecFrameInputs: a hash over the requested time, the evaluator's
/// stage-edit serial, and the standing overrides, which never reach the
/// stage and so must be folded explicitly.
///
/// The serial is what makes a time key correct: it advances on every stage
/// notice, so an edit moves the digest at every frame by construction and
/// stale entries become unreachable (reclaimed by LRU) -- the D1 rule, with
/// the serial standing in for re-sampling. No clearing is needed on any
/// path, registry-managed or bare bridge. The domain tag keeps a refusal
/// digest from ever equaling a sampled digest under the same epoch (a mode
/// toggle crosses the two without moving the epoch). Overrides fold exactly
/// as in RigExecControlStateDigest, so a drag always digests apart from the
/// authored frame it started from.
///
/// Within-process only, like the key hash: never persisted.
uint64_t RigExecRefusalControlDigest(
    UsdTimeCode time, uint64_t stageEditSerial,
    const std::vector<RigExecValueOverride> &overrides);

/// Whether every override in \p overrides can be digested exactly (time
/// always folds). False bypasses the cache, as in
/// RigExecControlStateDigestible.
bool RigExecRefusalControlDigestible(
    const std::vector<RigExecValueOverride> &overrides);

/// One entry the cache dropped: its key and the time it was published for.
/// Reported so the per-frame index retires the time without scanning the
/// store; the key disambiguates (a time republished under a new key must
/// not retire when the old entry drains).
struct RigExecFrameCacheEviction {
    RigExecFrameCacheKey key{0, 0};
    UsdTimeCode time = UsdTimeCode::Default();
};

/// Eviction reporting for the per-frame index. Invoked outside every shard
/// lock with the entries one call dropped (Publish's cap pass, Evict,
/// EvictEpoch, SetByteCap, Clear, and Lookup's corrupt drop) -- never empty,
/// never under a cache lock, so the callback may take its own (leaf) locks.
/// Null by default. Set before warming starts; synchronized, so late sets
/// are safe but may miss drops that raced them.
using RigExecFrameCacheEvictionCallback =
    std::function<void(const std::vector<RigExecFrameCacheEviction> &)>;

/// Per-entry provenance (plan 2.0): what an entry's computation depended
/// on, recorded at publish so a later edit retires and re-runs only the
/// entries that touched what moved. A full evaluation depends on every
/// cluster, every weight object, and every constant region of its program
/// (see RigExecFullEvalProvenance); a partial cone re-run (plan 2.2)
/// records the subset it ran against retained state. Aliases name every
/// frame served under the key, starting with the publishing time.
struct RigExecEntryProvenance {
    /// Clusters whose computation produced the entry, sorted and unique.
    std::vector<int> clusters;
    /// Weight objects and packets the computation read, as object path
    /// strings, sorted and unique. WeightPacket steps resolve through
    /// step.object, outside the avar-only affected index, so this set is
    /// what maps a weight-prim edit to its entries.
    std::vector<std::string> weightReads;
    /// Constant regions the computation read (plan D3: region ids from
    /// RigExecConstantRegionForBinding), sorted and unique. Carry-over
    /// across a constant-namespace change is sound only for entries that
    /// read no patched region.
    std::vector<uint64_t> constantRegions;
    /// Time values served under the key, in record order starting with the
    /// publishing time: one digest may serve many frames. Bounded (past
    /// the cap aliases stop recording; retirement iterates the 1.4
    /// completed-time set, never this list, so the cap costs nothing).
    std::vector<double> aliasTimes;
    /// The entry's control digest BEFORE the epoch-constant fold
    /// (RigExecControlStateDigest over the sampled vector and overrides).
    /// A constant patch moves the folded key but not the sampled values,
    /// so the 2.2 carry-over lane re-keys a clean entry as
    /// FoldConstantDigest(unfoldedControlDigest, newConstants) with zero
    /// re-sampling. Zero when the publisher did not record one (pose-only
    /// entries, refusal keys): such entries never carry, they re-warm.
    uint64_t unfoldedControlDigest = 0;
};

/// Cap on RigExecEntryProvenance::aliasTimes per entry.
constexpr size_t kRigExecProvenanceAliasCap = 1024;

/// Accounted payload bytes of one pose: the struct shell plus every published
/// map, by the same counting reports/frame-cache-measurements.md measures
/// (SdfPath keys at sizeof(SdfPath), path strings interned and excluded, map
/// node overhead excluded). Publish counts this per entry; Stream D adds the
/// retained arena beside it.
size_t RigExecFrameCachePoseBytes(const RigExecRigPose &pose);

/// The (epochDigest, controlDigest) store. See the file header for the key
/// contract and the lock rule.
class RigExecFrameCache {
public:
    RigExecFrameCache();
    ~RigExecFrameCache();

    RigExecFrameCache(const RigExecFrameCache &) = delete;
    RigExecFrameCache &operator=(const RigExecFrameCache &) = delete;

    /// Answers the cached pose for \p key, or false having touched nothing
    /// but the miss counter. \p pose must be non-null; a null out-param
    /// misses rather than faults. A hit copies the stored pose -- the pose
    /// is bit-identical to the published one -- and refreshes the entry's
    /// LRU position.
    bool Lookup(const RigExecFrameCacheKey &key, RigExecRigPose *pose) const;

    /// Stores \p pose -- evaluated at \p time -- under \p key, evicting
    /// least-recently-used entries down to the byte cap as needed. Returns
    /// false when the entry was not stored and the caller falls back to live
    /// evaluation: the pose is invalid, or its bytes exceed the whole cap.
    /// Publishing under a held key replaces the pose and refreshes its LRU
    /// position; counts one publish either way.
    /// \p provenance, when given, is recorded as the entry's dependency
    /// record (plan 2.0); a null records an empty one.
    bool Publish(const RigExecFrameCacheKey &key, UsdTimeCode time,
                 const RigExecRigPose &pose,
                 const RigExecEntryProvenance *provenance = nullptr);
    /// The sparse-reuse (Stream D) Publish: stores \p pose plus the retained
    /// cross-frame state -- the source snapshot and slot arena a partial
    /// cone re-runs against -- accounted at \p retainedBytes beside the
    /// pose's own bytes. \p retained is opaque here (a
    /// RigExecRetainedFrameState to frameCacheSparsity.h); a null handle
    /// with nonzero bytes is declined, like an invalid pose. Otherwise the
    /// contract is Publish's: replace-on-held-key, LRU refresh, eviction
    /// down to the cap, drop-when-oversized.
    /// \p provenance is recorded as in Publish.
    bool Publish(const RigExecFrameCacheKey &key, UsdTimeCode time,
                 const RigExecRigPose &pose, size_t retainedBytes,
                 std::shared_ptr<const void> retained,
                 const RigExecEntryProvenance *provenance = nullptr);

    /// The sparse-reuse Lookup: answers the pose and, in \p retainedOut,
    /// the retained handle Publish stored -- null for a pose-only entry.
    /// \p retainedOut may be null to take the pose alone; \p pose follows
    /// Lookup's null-misses rule.
    bool Lookup(const RigExecFrameCacheKey &key, RigExecRigPose *pose,
                std::shared_ptr<const void> *retainedOut) const;

    /// The sparse-reuse Lookup with accounted bytes: as above, plus the
    /// entry's accounted payload bytes (pose plus retained) in \p bytesOut.
    /// The carry-over path (plan 2.2) re-publishes under a new key, which
    /// needs the bytes the plain Lookup never returned. \p bytesOut may be
    /// null to take the pose and handle alone.
    bool Lookup(const RigExecFrameCacheKey &key, RigExecRigPose *pose,
                std::shared_ptr<const void> *retainedOut,
                size_t *bytesOut) const;

    /// Answers the provenance Publish recorded under \p key, or false when
    /// no entry (or no provenance) is held. A null out-param misses rather
    /// than faults. Does not refresh the entry's LRU position: retirement
    /// scans must not pin what they inspect.
    bool LookupProvenance(const RigExecFrameCacheKey &key,
                          RigExecEntryProvenance *provenance) const;

    /// Records \p time as served under \p key: the alias list of a digest
    /// that serves many frames. No-op when no entry is held, when the time
    /// is already listed, or past the alias cap. Called on the serving
    /// path (a cache hit); under the shard lock, never across evaluation.
    void NoteProvenanceAlias(const RigExecFrameCacheKey &key,
                             UsdTimeCode time);

    /// Drops the entry under \p key, if any. Returns whether one was held.
    bool Evict(const RigExecFrameCacheKey &key);

    /// Drops every entry of \p epochDigest -- the baked capture index hit
    /// the epoch -- returning how many were held. Locks one shard at a
    /// time, like eviction; entries of every other epoch are untouched.
    size_t EvictEpoch(uint64_t epochDigest);

    /// Drops every entry and zeroes the counters. The byte cap is unchanged.
    void Clear();

    /// Installs the eviction callback (null uninstalls); see
    /// RigExecFrameCacheEvictionCallback for the delivery contract.
    void SetEvictionCallback(RigExecFrameCacheEvictionCallback callback);

    /// The per-rig byte cap. Publish evicts down to it; an entry larger than
    /// the whole cap is dropped rather than stored. Lowering the cap evicts
    /// down to it immediately.
    void SetByteCap(size_t bytes);
    size_t GetByteCap() const;

    RigExecFrameCacheStats Stats() const;

private:
    // One stored generation behind a shared handle: Lookup copies the handle
    // under the shard lock and the pose outside it, which is what keeps
    // allocation and copying out of the critical section.
    struct _Entry {
        UsdTimeCode time = UsdTimeCode::Default();
        std::shared_ptr<const RigExecRigPose> pose;
        size_t bytes = 0;
        // The sparse-reuse retained handle (a RigExecRetainedFrameState),
        // null for a pose-only entry. Accounted inside `bytes`, which is
        // the pose's bytes plus the retained bytes.
        std::shared_ptr<const void> retained;
        // The entry's dependency record (plan 2.0), empty when Publish
        // recorded none. Copied out under the shard lock by
        // LookupProvenance; appended to (aliases only) by
        // NoteProvenanceAlias.
        RigExecEntryProvenance provenance;
        // LRU clock tick of the last Lookup or Publish touching this entry.
        // Approximate across threads by design: a race only evicts a
        // recently-used entry early (a miss, never a wrong pose).
        mutable std::atomic<uint64_t> lastUse{0};
    };
    struct _Shard {
        mutable std::mutex mutex;
        // Mutable: Lookup edits through a const handle -- refreshing the
        // LRU position and dropping corrupt entries -- the same way it
        // counts hits and misses through one.
        mutable std::unordered_map<RigExecFrameCacheKey, std::shared_ptr<_Entry>,
                                   RigExecFrameCacheKeyHash> map;
    };
    static constexpr size_t _kShardCount = 16;

    _Shard &_GetShard(const RigExecFrameCacheKey &key)
    {
        return _shards[RigExecFrameCacheKeyHash{}(key) % _kShardCount];
    }
    const _Shard &_GetShard(const RigExecFrameCacheKey &key) const
    {
        return _shards[RigExecFrameCacheKeyHash{}(key) % _kShardCount];
    }

    // Evicts least-recently-used entries until the held bytes fit the cap,
    // locking one shard at a time: a scan pass finds the globally-oldest
    // entry, a second pass drops it if it is still that old. A racing touch
    // rescans rather than evicting a live entry.
    void _EvictUntilUnderCap();

    // Reports dropped entries outside every shard lock. \p evicted is
    // moved-from; an empty vector reports nothing. Const so Lookup's
    // corrupt drop (itself const) reports through the same path.
    void _ReportEvictions(
        std::vector<RigExecFrameCacheEviction> evicted) const;

    mutable std::array<_Shard, _kShardCount> _shards;
    mutable std::atomic<uint64_t> _clock{1};
    mutable std::atomic<size_t> _byteCap{kRigExecFrameCacheDefaultByteCap};
    mutable std::atomic<size_t> _hits{0};
    mutable std::atomic<size_t> _misses{0};
    mutable std::atomic<size_t> _published{0};
    mutable std::atomic<size_t> _evictions{0};
    mutable std::atomic<size_t> _entryCount{0};
    mutable std::atomic<size_t> _bytes{0};
    mutable std::mutex _callbackMutex;
    RigExecFrameCacheEvictionCallback _evictionCallback;
};

}  // namespace rigExec

#endif  // RIGEXEC_FRAME_CACHE_H
