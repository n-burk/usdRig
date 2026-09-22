//
// RigExec task-list cache: the affected-cluster set per (control, epoch).
//
// The brief's task-list cache (pp. 9, 32): computing which clusters an edit
// reaches means walking the output-affected index, and a drag re-edits ONE
// control dozens of times. The first edit walks and memoizes; every repeat
// re-runs the cached selection without rewalking. Multi-control edits union
// one memoized set per control, so each control still walks at most once per
// epoch.
//
// Keyed by (epochDigest, control): a topology change is a new epoch, which
// makes every memoized set unreachable the way a control edit makes a cached
// pose unreachable (plan D1). InvalidateEpoch drops an epoch's sets eagerly
// so a dead epoch's memo does not hold memory until LRU pressure -- there is
// no LRU here, the table is small (controls x live epochs) and explicit
// invalidation bounds it.
//

#ifndef RIGEXEC_TASK_LIST_CACHE_H
#define RIGEXEC_TASK_LIST_CACHE_H

#include "outputAffectedIndex.h"

#include <cstddef>
#include <cstdint>
#include <map>
#include <mutex>
#include <utility>

namespace rigExec {

/// What one memoized selection is: the closed affected set of one control
/// in one epoch, as a bit array over clusters.
using RigExecTaskListKey = std::pair<uint64_t, RigExecControlId>;

/// Lifetime counters. Every counter only moves forward except across Clear.
struct RigExecTaskListStats {
    size_t hits = 0;
    size_t misses = 0;
    size_t stores = 0;
    size_t entries = 0;
};

/// The memoized affected-set table. Thread-safe: the lock is held only
/// across a map edit, never across an index walk -- the caller walks outside
/// the lock and Stores the result.
class RigExecTaskListCache {
public:
    RigExecTaskListCache() = default;

    RigExecTaskListCache(const RigExecTaskListCache &) = delete;
    RigExecTaskListCache &operator=(const RigExecTaskListCache &) = delete;

    /// Answers the memoized set for (\p epoch, \p control), or false having
    /// touched nothing but the miss counter. \p out must be non-null; a null
    /// out-param misses rather than faults.
    bool Lookup(uint64_t epoch, const RigExecControlId &control,
                RigExecBakedClusterSet *out);

    /// Memoizes \p set for (\p epoch, \p control), replacing any set held.
    /// Counts one store either way.
    void Store(uint64_t epoch, const RigExecControlId &control,
               const RigExecBakedClusterSet &set);

    /// Drops every set held for \p epoch -- the capture index hit it, or its
    /// topology moved -- returning how many were held.
    size_t InvalidateEpoch(uint64_t epoch);

    /// Drops every set and zeroes the counters.
    void Clear();

    RigExecTaskListStats Stats() const;
    size_t Size() const;

private:
    mutable std::mutex _mutex;
    std::map<RigExecTaskListKey, RigExecBakedClusterSet> _map;
    size_t _hits = 0;
    size_t _misses = 0;
    size_t _stores = 0;
};

}  // namespace rigExec

#endif  // RIGEXEC_TASK_LIST_CACHE_H
