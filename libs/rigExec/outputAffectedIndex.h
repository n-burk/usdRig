//
// RigExec output-affected index: which clusters a changed control reaches.
//
// Cross-frame sparse reuse (plan Stream D) needs the question the baked cone
// machinery never asks: given a control that moved, which clusters downstream
// of it have to re-run. The gap analysis flags this output-dependency list as
// missing -- RigExecBakedCones maps a changed SOURCE to its seed clusters and
// closes over them, but nothing maps a CONTROL to the clusters it reaches.
// This index is that list, at cluster granularity.
//
// It derives NOTHING. Build copies the forward closures and the seed tables
// RigExecBakedBuildCones computed; every query is a union over those copies.
// A control the index never learned (not a provider slot, no explicit
// mapping) answers conservatively with every cluster: re-running too much is
// a slower frame, skipping a cluster that moved is a wrong pose.
//

#ifndef RIGEXEC_OUTPUT_AFFECTED_INDEX_H
#define RIGEXEC_OUTPUT_AFFECTED_INDEX_H

#include "bakedProgramImpl.h"

#include <atomic>
#include <cstddef>
#include <cstdint>
#include <map>
#include <set>
#include <string>
#include <vector>

#include "pxr/usd/usd/notice.h"

PXR_NAMESPACE_USING_DIRECTIVE

namespace rigExec {

/// Names one cached-frame input for the affected-set computation: a sampled
/// source's path string, or `prim|computation|attribute` for an interactive
/// override (the same identity the control digest keys overrides by).
using RigExecControlId = std::string;

/// The identity RigExecChangedControls assigns a sampled source.
inline RigExecControlId
RigExecControlIdForPath(const SdfPath &path)
{
    return path.GetString();
}

/// The identity RigExecChangedControls assigns an interactive override.
inline RigExecControlId
RigExecControlIdForOverride(const RigExecValueOverride &o)
{
    return o.prim.GetString() + "|" + o.computation.GetString() + "|" +
           o.attribute.GetString();
}

/// `dirty ∩ affecting(requested)`: the brief's lazy dirty set (p. 10) as bit
/// arrays over clusters. Either side may be shorter than the other; missing
/// words read as zero.
RigExecBakedClusterSet RigExecIntersectClusterSets(
    const RigExecBakedClusterSet &dirty,
    const RigExecBakedClusterSet &affecting);

/// The downstream closure of every control, for one epoch.
///
/// Immutable after Build except for MapControl (build phase) and the walk
/// counter: queries are const and thread-safe, so a UI thread planning reuse
/// and background workers warming frames may share one index.
class RigExecOutputAffectedIndex {
public:
    RigExecOutputAffectedIndex() = default;

    RigExecOutputAffectedIndex(const RigExecOutputAffectedIndex &) = delete;
    RigExecOutputAffectedIndex &operator=(
        const RigExecOutputAffectedIndex &) = delete;

    /// Snapshots \p program's closures for \p epochDigest: the forward cone
    /// of every cluster, the always-dirty set, the varying/override cluster
    /// closures, and the control->seed map for every cone-source family --
    /// provider slots (path -> its compose cluster), patchable and varying
    /// avar property paths, chains, solvers, revisions, native sources,
    /// delta bases, and constraint arrays -- plus the control universe the
    /// family walk yields. Replaces any previous build, including explicit
    /// MapControl entries, and zeroes the walk counter.
    void Build(const RigExecBakedProgramImpl &program, uint64_t epochDigest);

    void Clear();

    bool Empty() const { return _clusterCount == 0; }
    size_t ClusterCount() const { return _clusterCount; }
    uint64_t EpochDigest() const { return _epochDigest; }

    /// Registers \p seeds as the clusters \p control reaches directly, before
    /// closure. Replaces any entry Build derived or an earlier call set.
    /// Out-of-range seeds are dropped; an empty index ignores the call.
    /// Admits \p control to the universe -- the dynamic-override
    /// bootstrapping path -- even when no seed survives.
    void MapControl(const RigExecControlId &control,
                    const std::vector<int> &seeds);

    /// Whether \p control is a member of the control universe: named by
    /// Build's family walk or admitted by MapControl. Members with empty
    /// seeds are known-empty (they retire nothing); non-members are
    /// unknown (they retire everything). False on an empty index.
    bool IsKnownControl(const RigExecControlId &control) const;

    /// Every control ID in the universe, in order: the sampled paths
    /// across all cone-source families plus override identities, with
    /// seedless members included. Empty on an empty index.
    std::vector<RigExecControlId> ControlUniverse() const;

    /// The seed clusters \p control reaches directly, or empty when the
    /// control was never mapped. Empty is "unknown" only for a
    /// non-member -- see IsKnownControl: a known-empty member also
    /// answers empty, and means "affects nothing". Callers that need a
    /// closed set use AffectedByControls, which branches three ways.
    std::vector<int> SeedsForControl(
        const RigExecControlId &control) const;

    /// The union of cone[c] over \p seeds: everything that has to run when
    /// any of them does. Out-of-range seeds are skipped. Counts one walk.
    RigExecBakedClusterSet AffectedClusters(
        const std::vector<int> &seeds) const;
    RigExecBakedClusterSet AffectedClusters(
        const RigExecBakedClusterSet &seeds) const;

    /// The union of AffectedClusters over every control's seeds, branching
    /// three ways: a FOREIGN control (outside the universe) contributes
    /// EVERY cluster -- the conservative answer that keeps reuse
    /// bit-identical; a KNOWN-EMPTY member (in the universe, no seeds)
    /// contributes NOTHING; a mapped member contributes the cone closure
    /// of its seeds. Counts one walk per call, however many controls were
    /// named.
    RigExecBakedClusterSet AffectedByControls(
        const std::vector<RigExecControlId> &controls) const;

    /// Every cluster, set. The conservative affected set and the default
    /// `affecting(requested)` for a whole-pose request.
    RigExecBakedClusterSet AllClusters() const;

    /// The forward closure of \p cluster (itself included), as Build copied
    /// it. Out of range answers an empty set.
    const RigExecBakedClusterSet &ConeOf(int cluster) const;

    /// Clusters holding a step that reads outside the graph: dirty on any
    /// run whose time moved (§7), whatever the sources say.
    const RigExecBakedClusterSet &Always() const { return _always; }

    /// The forward closure of every varying step's cluster: what a moved
    /// time dirties beside Always (the executor's time rule, plan 2.1).
    const RigExecBakedClusterSet &Varying() const { return _varying; }

    /// The forward closure of every override-step cluster: what dirties
    /// while a standing override holds (the executor's override rule).
    const RigExecBakedClusterSet &Override() const { return _override; }

    /// Affected-set computations performed since Build: AffectedClusters and
    /// AffectedByControls each count one. The task-list cache exists to keep
    /// this at zero across repeated edits of one control.
    size_t Walks() const { return _walks.load(std::memory_order_relaxed); }

private:
    size_t _clusterCount = 0;
    uint64_t _epochDigest = 0;
    std::vector<RigExecBakedClusterSet> _cones;
    RigExecBakedClusterSet _always;
    RigExecBakedClusterSet _varying;
    RigExecBakedClusterSet _override;
    RigExecBakedClusterSet _empty;
    std::map<RigExecControlId, std::vector<int>> _seeds;
    /// Every control ID Build's family walk yielded (sampled paths across
    /// all cone-source families plus override identities), with seedless
    /// members stored as empty entries: known-empty, distinct from unknown.
    /// MapControl admits its control here.
    std::set<RigExecControlId> _universe;
    mutable std::atomic<size_t> _walks{0};
};

/// Normalizes a USD notice's paths to control IDs (plan 2.1 notice
/// adapter): every resynced, info-only, and resolved-asset-resynced path
/// contributes its full path string plus, for a property path, its prim
/// path string -- the two granularities the index maps (patchable avar
/// properties exactly, providers and cone sources by prim). Sorted,
/// deduplicated. Empty/invalid paths normalize to nothing and contribute
/// no IDs; every valid path contributes at least its own string, so a
/// foreign edit still retires conservatively through the index's
/// foreign-ID rule -- with one exception: an info-only PRIM path shadowed
/// by a more specific path in the same notice (another entry at or under
/// it) contributes nothing. USD reports ancestor prims beside every value
/// edit; the specific path names the change and the ancestor adds no
/// information, while a LONE prim path (nothing specific under it) is
/// kept, so an unspecified change still retires conservatively.
std::vector<RigExecControlId> RigExecNoticeControlIds(
    const UsdNotice::ObjectsChanged &notice);

/// The adapter core over explicit path lists, for tests that cannot
/// fabricate a USD notice. The notice entry point forwards its three
/// path lists here unchanged.
std::vector<RigExecControlId> RigExecNoticeControlIdsFromPaths(
    const std::vector<SdfPath> &resynced,
    const std::vector<SdfPath> &infoOnly,
    const std::vector<SdfPath> &resolvedResynced);

/// The seed clusters an interactive override reaches (plan 2.1
/// MapControl production input): the clusters of the steps whose
/// overrideInputs carry the indices RigExecBakedProgram::SetOverrides
/// would flag for \p override, plus the compose clusters of the avar
/// bindings behind those indices -- the same prim-plus-attribute
/// placement, the same folded/exec-typed refusals -- sorted and unique.
/// Both halves are needed because the executor sees the two input
/// families differently: a Solve/Constraint input dirties its step
/// through the flagged index, while an avar input lands in the dense
/// table and dirties its provider's compose cluster through the
/// per-provider value comparison. Empty when the override places
/// nowhere: the caller leaves it foreign (all clusters), never admits
/// it seedless. Exact, not conservative: these are the clusters the
/// executor dirties for a standing override.
std::vector<int> RigExecOverrideSeeds(
    const RigExecBakedProgramImpl &program,
    const RigExecValueOverride &override);

}  // namespace rigExec

#endif  // RIGEXEC_OUTPUT_AFFECTED_INDEX_H
