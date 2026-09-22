//
// RigExec output-affected index. See outputAffectedIndex.h.
//

#include "outputAffectedIndex.h"

#include <algorithm>

namespace rigExec {

RigExecBakedClusterSet
RigExecIntersectClusterSets(const RigExecBakedClusterSet &dirty,
                            const RigExecBakedClusterSet &affecting)
{
    RigExecBakedClusterSet out;
    const size_t words =
        std::max(dirty.words.size(), affecting.words.size());
    out.words.assign(words, 0);
    for (size_t w = 0; w < words; ++w) {
        const uint64_t a = w < dirty.words.size() ? dirty.words[w] : 0;
        const uint64_t b =
            w < affecting.words.size() ? affecting.words[w] : 0;
        out.words[w] = a & b;
    }
    return out;
}

void
RigExecOutputAffectedIndex::Build(const RigExecBakedProgramImpl &program,
                                 uint64_t epochDigest)
{
    Clear();
    _clusterCount = program.clustering.clusters.size();
    _epochDigest = epochDigest;
    if (_clusterCount == 0) {
        return;
    }
    // The closures, copied -- not re-derived. Build refuses a program whose
    // cones do not cover its clustering (a BuildCones that never ran) by
    // treating it as empty: an index that answered empty closures would skip
    // every cluster and serve stale poses.
    if (program.cones.cone.size() != _clusterCount) {
        Clear();
        return;
    }
    _cones = program.cones.cone;
    for (RigExecBakedClusterSet &cone : _cones) {
        cone.words.resize((_clusterCount + 63) / 64, 0);
    }
    _always = program.cones.always;
    _always.words.resize((_clusterCount + 63) / 64, 0);
    _empty.Resize(_clusterCount);
    _varying.Resize(_clusterCount);
    _override.Resize(_clusterCount);
    // The executor's time/override rule as sets: varying steps close over
    // their clusters for a moved time, override steps for a standing
    // override (plan 2.1 planner parity).
    for (const int step : program.cones.varyingSteps) {
        if (step >= 0 &&
            size_t(step) < program.clustering.clusterOf.size()) {
            const int cluster = program.clustering.clusterOf[size_t(step)];
            if (cluster >= 0 && size_t(cluster) < _cones.size()) {
                _varying.Union(_cones[size_t(cluster)]);
            }
        }
    }
    for (const int step : program.cones.overrideSteps) {
        if (step >= 0 &&
            size_t(step) < program.clustering.clusterOf.size()) {
            const int cluster = program.clustering.clusterOf[size_t(step)];
            if (cluster >= 0 && size_t(cluster) < _cones.size()) {
                _override.Union(_cones[size_t(cluster)]);
            }
        }
    }
    // Admits one control with its seeds, dropping out-of-range seeds.
    // Every family admits its control even when no seed survives: a
    // seedless member is known-empty (retires nothing), distinct from a
    // foreign ID (retires everything).
    const auto admit = [&](const RigExecControlId &control,
                           const std::vector<int> &seeds) {
        if (control.empty()) {
            return;
        }
        _universe.insert(control);
        std::vector<int> &kept = _seeds[control];
        for (const int seed : seeds) {
            if (seed >= 0 && size_t(seed) < _clusterCount) {
                kept.push_back(seed);
            }
        }
    };
    const auto admitOne = [&](const RigExecControlId &control, int seed) {
        if (seed >= 0 && size_t(seed) < _clusterCount) {
            admit(control, std::vector<int>{seed});
        } else if (!control.empty()) {
            _universe.insert(control);
            _seeds.emplace(control, std::vector<int>());
        }
    };
    const std::vector<int> noSeeds;
    const auto tableAt =
        [&](const std::vector<std::vector<int>> &table,
            size_t i) -> const std::vector<int> & {
        return i < table.size() ? table[i] : noSeeds;
    };
    const auto composeCluster = [&](size_t slot) {
        return slot < program.cones.avarCluster.size()
            ? program.cones.avarCluster[slot]
            : -1;
    };
    // Every provider slot the compose pass reads: the path the sampler names
    // a source by, mapped to the cluster whose cone a moved avar dirties
    // (§7 steady state). Slots with no compose cluster (-1) join the
    // universe seedless instead of answering conservatively.
    const size_t slots =
        std::min(program.paths.size(), program.cones.avarCluster.size());
    for (size_t slot = 0; slot < slots; ++slot) {
        admitOne(RigExecControlIdForPath(program.paths[slot]),
                 program.cones.avarCluster[slot]);
    }
    // Patchable avar properties: the exact paths a value patch names, each
    // mapped to its provider's compose cluster. The binding slot is a FLAT
    // provider*11+channel index into the dense avar table (not a provider
    // ordinal), so the provider is slot/11: passing the flat slot to the
    // provider-sized compose table mis-maps low channels to the wrong
    // provider and strands the rest seedless.
    for (const auto &kv : program.patchableAvars) {
        int cluster = -1;
        if (kv.second < program.avarConstantBindings.size()) {
            cluster = composeCluster(
                program.avarConstantBindings[kv.second].slot / 11);
        }
        admitOne(RigExecControlIdForPath(kv.first), cluster);
    }
    // Varying avar bindings: the head of each binding's walk -- the avar
    // property the sampler names the sample by -- mapped to its
    // provider's compose cluster (flat slot / 11, as above).
    for (const auto &binding : program.avarBindings) {
        if (!binding.input.head) {
            continue;
        }
        admitOne(RigExecControlIdForPath(binding.input.head.GetPath()),
                 composeCluster(binding.slot / 11));
    }
    // Chains: the target path reaches the clusters reading its base points.
    for (size_t c = 0; c < program.chains.size(); ++c) {
        admit(RigExecControlIdForPath(program.chains[c].target),
              tableAt(program.cones.chainBaseClusters, c));
    }
    // Solvers: the solver path reaches the clusters reading its driver
    // points. What a ribbon's moved driver curve makes dirty, and nothing
    // else.
    for (const auto &kv : program.solverIndex) {
        admit(RigExecControlIdForPath(kv.first),
              kv.second >= 0
                  ? tableAt(program.cones.solverPointsClusters,
                            size_t(kv.second))
                  : noSeeds);
    }
    // Revisions: the mover path reaches the revision's clusters plus its
    // RevisionStatic cluster.
    for (size_t id = 0; id < program.revisionIndex.size(); ++id) {
        const int chain = program.revisionIndex[id].first;
        const int revision = program.revisionIndex[id].second;
        if (chain < 0 || size_t(chain) >= program.chains.size()) {
            continue;
        }
        const auto &revisions = program.chains[size_t(chain)].revisions;
        if (revision < 0 || size_t(revision) >= revisions.size()) {
            continue;
        }
        std::vector<int> seeds = tableAt(program.cones.revisionClusters, id);
        if (id < program.cones.revisionStaticCluster.size()) {
            seeds.push_back(program.cones.revisionStaticCluster[id]);
        }
        admit(RigExecControlIdForPath(revisions[size_t(revision)].moverPath),
              seeds);
    }
    // Native sources: the source path reaches the constraint clusters
    // reading the frame the prologue read for it.
    for (size_t i = 0; i < program.nativeSources.size(); ++i) {
        admit(RigExecControlIdForPath(program.nativeSources[i].path),
              tableAt(program.cones.nativeSourceClusters, i));
    }
    // Geometry-delta bases: the base path reaches the constraint cluster
    // measuring against it.
    for (size_t i = 0; i < program.deltaBasePaths.size(); ++i) {
        admit(RigExecControlIdForPath(program.deltaBasePaths[i]),
              tableAt(program.cones.deltaBaseClusters, i));
    }
    // Constraint arrays: the owning prim reaches the constraint cluster
    // reading its per-frame arrays.
    for (size_t i = 0; i < program.constraintArrays.size(); ++i) {
        const UsdPrim &prim = program.constraintArrays[i].prim;
        if (!prim.IsValid()) {
            continue;
        }
        admit(RigExecControlIdForPath(prim.GetPath()),
              tableAt(program.cones.constraintArrayClusters, i));
    }
}

void
RigExecOutputAffectedIndex::Clear()
{
    _clusterCount = 0;
    _epochDigest = 0;
    _cones.clear();
    _always = RigExecBakedClusterSet();
    _varying = RigExecBakedClusterSet();
    _override = RigExecBakedClusterSet();
    _empty = RigExecBakedClusterSet();
    _seeds.clear();
    _universe.clear();
    _walks.store(0, std::memory_order_relaxed);
}

void
RigExecOutputAffectedIndex::MapControl(const RigExecControlId &control,
                                      const std::vector<int> &seeds)
{
    if (_clusterCount == 0) {
        return;
    }
    std::vector<int> kept;
    for (const int seed : seeds) {
        if (seed >= 0 && size_t(seed) < _clusterCount) {
            kept.push_back(seed);
        }
    }
    _seeds[control] = kept;
    _universe.insert(control);
}

bool
RigExecOutputAffectedIndex::IsKnownControl(
    const RigExecControlId &control) const
{
    return _universe.find(control) != _universe.end();
}

std::vector<RigExecControlId>
RigExecOutputAffectedIndex::ControlUniverse() const
{
    return std::vector<RigExecControlId>(_universe.begin(), _universe.end());
}

std::vector<int>
RigExecOutputAffectedIndex::SeedsForControl(
    const RigExecControlId &control) const
{
    const auto found = _seeds.find(control);
    if (found == _seeds.end()) {
        return {};
    }
    return found->second;
}

RigExecBakedClusterSet
RigExecOutputAffectedIndex::AffectedClusters(
    const std::vector<int> &seeds) const
{
    RigExecBakedClusterSet out;
    out.Resize(_clusterCount);
    for (const int seed : seeds) {
        if (seed >= 0 && size_t(seed) < _cones.size()) {
            out.Union(_cones[size_t(seed)]);
        }
    }
    _walks.fetch_add(1, std::memory_order_relaxed);
    return out;
}

RigExecBakedClusterSet
RigExecOutputAffectedIndex::AffectedClusters(
    const RigExecBakedClusterSet &seeds) const
{
    RigExecBakedClusterSet out;
    out.Resize(_clusterCount);
    for (size_t c = 0; c < _clusterCount; ++c) {
        // Test would read past a shorter set's words; missing words are
        // zero, so only in-range words are consulted.
        const size_t word = c >> 6;
        const bool set = word < seeds.words.size() &&
                         ((seeds.words[word] >> (c & 63)) & 1) != 0;
        if (set) {
            out.Union(_cones[c]);
        }
    }
    _walks.fetch_add(1, std::memory_order_relaxed);
    return out;
}

RigExecBakedClusterSet
RigExecOutputAffectedIndex::AffectedByControls(
    const std::vector<RigExecControlId> &controls) const
{
    RigExecBakedClusterSet out;
    out.Resize(_clusterCount);
    for (const RigExecControlId &control : controls) {
        if (_universe.find(control) == _universe.end()) {
            // Foreign control: every cluster. See the file header for why
            // this is the only conservative answer.
            out.SetAll(_clusterCount);
            continue;
        }
        // A known-empty member holds no seeds and contributes nothing.
        const auto found = _seeds.find(control);
        if (found == _seeds.end()) {
            continue;
        }
        for (const int seed : found->second) {
            out.Union(_cones[size_t(seed)]);
        }
    }
    _walks.fetch_add(1, std::memory_order_relaxed);
    return out;
}

RigExecBakedClusterSet
RigExecOutputAffectedIndex::AllClusters() const
{
    RigExecBakedClusterSet out;
    out.SetAll(_clusterCount);
    return out;
}

const RigExecBakedClusterSet &
RigExecOutputAffectedIndex::ConeOf(int cluster) const
{
    if (cluster < 0 || size_t(cluster) >= _cones.size()) {
        return _empty;
    }
    return _cones[size_t(cluster)];
}

std::vector<RigExecControlId>
RigExecNoticeControlIds(const UsdNotice::ObjectsChanged &notice)
{
    const std::vector<SdfPath> resynced(notice.GetResyncedPaths().begin(),
                                        notice.GetResyncedPaths().end());
    const std::vector<SdfPath> info(
        notice.GetChangedInfoOnlyPaths().begin(),
        notice.GetChangedInfoOnlyPaths().end());
    const std::vector<SdfPath> resolved(
        notice.GetResolvedAssetPathsResyncedPaths().begin(),
        notice.GetResolvedAssetPathsResyncedPaths().end());
    return RigExecNoticeControlIdsFromPaths(resynced, info, resolved);
}

std::vector<RigExecControlId>
RigExecNoticeControlIdsFromPaths(
    const std::vector<SdfPath> &resynced,
    const std::vector<SdfPath> &infoOnly,
    const std::vector<SdfPath> &resolvedResynced)
{
    const std::vector<SdfPath> &info = infoOnly;
    const std::vector<SdfPath> &resolved = resolvedResynced;
    std::set<RigExecControlId> ids;
    const auto add = [&](const SdfPath &path) {
        if (path.IsEmpty()) {
            return;
        }
        ids.insert(path.GetString());
        if (path.IsPropertyPath()) {
            ids.insert(path.GetPrimPath().GetString());
        }
    };
    for (const SdfPath &path : resynced) {
        add(path);
    }
    for (const SdfPath &path : resolved) {
        add(path);
    }
    for (size_t i = 0; i < info.size(); ++i) {
        const SdfPath &path = info[i];
        if (path.IsEmpty()) {
            continue;
        }
        if (path.IsPrimPath()) {
            // Ancestor noise carries no information beyond the specific
            // path that shadowed it -- but a lone prim path is an
            // unspecified change under it, and stays foreign.
            bool shadowed = false;
            for (const SdfPath &other : resynced) {
                if (other.HasPrefix(path)) {
                    shadowed = true;
                    break;
                }
            }
            for (const SdfPath &other : resolved) {
                if (!shadowed && other.HasPrefix(path)) {
                    shadowed = true;
                    break;
                }
            }
            for (size_t j = 0; !shadowed && j < info.size(); ++j) {
                // Strictly under, by value: a twin entry (the notice
                // naming the same prim twice) is the same unspecified
                // change, not a more specific one, so twins never
                // shadow each other into silence.
                if (j != i && info[j] != path &&
                    info[j].HasPrefix(path)) {
                    shadowed = true;
                }
            }
            if (shadowed) {
                continue;
            }
        }
        add(path);
    }
    return std::vector<RigExecControlId>(ids.begin(), ids.end());
}

std::vector<int>
RigExecOverrideSeeds(const RigExecBakedProgramImpl &program,
                     const RigExecValueOverride &override)
{
    // Mirrors RigExecBakedProgram::SetOverrides placement: the attribute
    // override on prim-plus-attribute, refused when folded or exec-typed.
    // A computation override (empty attribute), a routed prim with no
    // binding, and an unknown path all place nothing here, exactly as
    // there -- and place nothing means no seeds, never seedless.
    if (override.attribute.IsEmpty()) {
        return {};
    }
    const SdfPath path =
        override.prim.AppendProperty(override.attribute);
    if (program.folded.count(path) ||
        program.execTypedArrayInputs.count(path)) {
        return {};
    }
    const auto found = program.overridableInputs.find(path);
    if (found == program.overridableInputs.end()) {
        return {};
    }
    std::set<int> wanted(found->second.begin(), found->second.end());
    if (wanted.empty()) {
        return {};
    }
    // The steps the executor dirties for these indices (bakedSchedule's
    // override rule): a step whose overrideInputs carry a flagged index
    // dirties its cluster, so those clusters are the override's seeds.
    std::set<int> seeds;
    const size_t clusters = program.clustering.clusters.size();
    for (const RigExecBakedStep &step : program.steps) {
        for (const int input : step.overrideInputs) {
            if (wanted.count(input)) {
                if (step.cluster >= 0 &&
                    size_t(step.cluster) < clusters) {
                    seeds.insert(step.cluster);
                }
                break;
            }
        }
    }
    // The avar bindings behind the same indices (the input block writes
    // an overridden avar into the dense table, and the per-provider
    // value comparison dirties its compose cluster). Matched by index,
    // not by head path, so an override standing on a walk hop resolves
    // to the same provider as one on the head. Flat slot / 11, as in
    // Build; a seedless provider contributes nothing (the override
    // stays foreign, never admitted seedless).
    const auto bindSeeds =
        [&](const std::vector<RigExecBakedProgramImpl::AvarBinding>
                &bindings) {
            for (const auto &binding : bindings) {
                if (!wanted.count(binding.input.overrideIndex)) {
                    continue;
                }
                const size_t provider = binding.slot / 11;
                if (provider < program.cones.avarCluster.size()) {
                    const int cluster =
                        program.cones.avarCluster[provider];
                    if (cluster >= 0 && size_t(cluster) < clusters) {
                        seeds.insert(cluster);
                    }
                }
            }
        };
    bindSeeds(program.avarBindings);
    bindSeeds(program.avarConstantBindings);
    return std::vector<int>(seeds.begin(), seeds.end());
}

}  // namespace rigExec
