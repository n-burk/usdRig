// The baked program's cluster partition and the cluster bitset its cone
// queries union: plain values, so code that only passes cluster sets (the
// output-affected index, the task-list cache) need not see the program.
#ifndef RIGEXEC_BAKED_CLUSTERS_H
#define RIGEXEC_BAKED_CLUSTERS_H

#include <algorithm>
#include <cstddef>
#include <cstdint>
#include <thread>
#include <vector>

#if defined(_MSC_VER)
#include <intrin.h>
#endif

namespace rigExec {

/// One cluster: the steps one task runs, back to back, on one thread.
///
/// A cluster's members are in increasing PROGRAM index, which is always a
/// topological order because every edge of the step graph points forward in
/// program order (§4.1). So a cluster needs no internal schedule: the loop
/// that runs its members in order is the schedule.
struct RigExecBakedCluster {
    /// Step indices, strictly increasing.
    std::vector<int> members;
    /// Cluster indices, sorted and deduplicated.
    std::vector<int> preds, succs;
    /// The summed cost of the members, in the cost model's microseconds.
    double cost = 0;
    /// The highest step level in the cluster, which is what the packing
    /// grouped by and what the report sorts on.
    int level = 0;

    /// When the cluster's last predecessor finished, when it started and
    /// when it ended, as RigExecProfiler::NowUs() reads. Recorded only while
    /// the schedule report or the profiler asks for them, so a production
    /// frame pays no clock reads for a table nobody prints.
    uint64_t readyUs = 0, startUs = 0, endUs = 0;
    /// The thread that ran it, stamped beside startUs. A cluster is the unit
    /// one task runs back to back (see above), so this is also the thread of
    /// every step in `members` -- which is what lets the epilogue put a
    /// step's interval on the row it really ran on, having been handed the
    /// step long after that thread moved on. Default-constructed after a
    /// serial run, which RecordOn reads as "the calling thread".
    std::thread::id runner;
};

/// A partition of one program's steps into clusters.
///
/// A VALUE, not program state: RigExecBakedBuildClusters computes one from a
/// program and a grain without touching it, which is what lets a test ask
/// the same program for the schedule at three different grains and compare
/// them. The program holds one of these -- the partition Build chose -- and
/// the parallel executor runs that one.
struct RigExecBakedClustering {
    std::vector<RigExecBakedCluster> clusters;
    /// The cluster of each step, one entry per step. Every step is in
    /// exactly one cluster.
    std::vector<int> clusterOf;
    /// The grain this partition was packed to, in microseconds. Zero means
    /// "one step per cluster", which is the finest schedule the graph admits
    /// and the strongest test of it.
    double grainUs = 0;
    /// The summed cost of every step, and the longest path through the
    /// cluster graph by cost. Their ratio is the speed-up this schedule can
    /// reach with threads to spare.
    double serialCost = 0, criticalPathCost = 0;
    /// Every cluster once, each after all of its predecessors, derived from
    /// the cluster edges (RigExecBakedClusterTopologicalOrder) rather than
    /// trusted from the ids. RigExecBakedBuildCones fills it, which is where
    /// Build first needs it.
    std::vector<int> topologicalOrder;
    /// Whether the last run stamped the per-cluster times above. Only the
    /// parallel executor has clusters to time: a serial run walks the steps
    /// and never asks which cluster they are in, so its run report says so
    /// rather than printing a table of zeros that reads as "every cluster
    /// was free".
    bool lastRunTimed = false;
};

/// Population count of one 64-bit word, on every supported compiler.
///
/// MSVC has no __builtin_popcountll; its equivalent is __popcnt64 from
/// <intrin.h>. The bit-twiddling fallback is for a compiler with neither
/// (C++20's std::popcount is not available under this project's C++17).
inline size_t _RigExecPopcount64(uint64_t word)
{
#if defined(_MSC_VER)
    return size_t(__popcnt64(word));
#elif defined(__GNUC__) || defined(__clang__)
    return size_t(__builtin_popcountll(word));
#else
    word -= (word >> 1) & uint64_t(0x5555555555555555);
    word = (word & uint64_t(0x3333333333333333)) +
           ((word >> 2) & uint64_t(0x3333333333333333));
    word = (word + (word >> 4)) & uint64_t(0x0f0f0f0f0f0f0f0f);
    return size_t((word * uint64_t(0x0101010101010101)) >> 56);
#endif
}

/// A bitset over clusters, as many 64-bit words as the program needs.
///
/// Cone re-execution is a handful of set unions per frame over sets whose
/// membership was decided at Build, so the representation that matters is
/// the one a union is a word loop over. |clusters| is a few dozen on a
/// biped, so this is one or two words.
struct RigExecBakedClusterSet {
    std::vector<uint64_t> words;

    void Resize(size_t clusters) {
        words.assign((clusters + 63) / 64, 0);
    }
    void Clear() { std::fill(words.begin(), words.end(), uint64_t(0)); }
    bool Test(int cluster) const {
        return cluster >= 0 &&
               (words[size_t(cluster) >> 6] >> (size_t(cluster) & 63)) & 1;
    }
    void Set(int cluster) {
        if (cluster >= 0) {
            words[size_t(cluster) >> 6] |=
                uint64_t(1) << (size_t(cluster) & 63);
        }
    }
    void SetAll(size_t clusters) {
        Resize(clusters);
        for (size_t c = 0; c < clusters; ++c) {
            Set(int(c));
        }
    }
    /// Whether anything was added, which is the fixpoint test. Widths may
    /// differ: a narrower other reads as zero past its end (the
    /// RigExecIntersectClusterSets rule), and a wider one grows this set
    /// rather than dropping its high members.
    bool Union(const RigExecBakedClusterSet &other) {
        bool grew = false;
        if (words.size() < other.words.size()) {
            words.resize(other.words.size(), 0);
        }
        for (size_t w = 0; w < words.size(); ++w) {
            const uint64_t o =
                w < other.words.size() ? other.words[w] : 0;
            const uint64_t before = words[w];
            words[w] |= o;
            grew = grew || words[w] != before;
        }
        return grew;
    }
    bool Any() const {
        for (const uint64_t word : words) {
            if (word) return true;
        }
        return false;
    }
    size_t Count() const {
        size_t count = 0;
        for (const uint64_t word : words) {
            count += _RigExecPopcount64(word);
        }
        return count;
    }
};

}  // namespace rigExec

#endif  // RIGEXEC_BAKED_CLUSTERS_H
