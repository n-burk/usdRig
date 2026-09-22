//
// RigExec sparse cross-frame reuse. See frameCacheSparsity.h.
//

#include "frameCacheSparsity.h"

#include "pxr/base/tf/getenv.h"

#include <algorithm>
#include <cstdio>
#include <map>
#include <tuple>

namespace rigExec {

RigExecSparsityDecision
RigExecDecideSparsity(size_t arenaBytes, size_t poseBytes, size_t byteCap,
                      size_t minFrames)
{
    RigExecSparsityDecision decision;
    const size_t fullBytes = arenaBytes + poseBytes;
    if (fullBytes == 0 || byteCap == 0) {
        char memo[256];
        std::snprintf(memo, sizeof(memo),
                      "sparsity no-go: no measurement to decide from "
                      "(arena %zu, pose %zu, cap %zu).",
                      arenaBytes, poseBytes, byteCap);
        decision.memo = memo;
        return decision;
    }
    decision.fullFramesAtCap = byteCap / fullBytes;
    decision.go = decision.fullFramesAtCap >= minFrames &&
                  minFrames > 0;
    char memo[256];
    std::snprintf(memo, sizeof(memo),
                  "sparsity %s: full frame %zu B (arena %zu + pose %zu) "
                  "holds %zu at cap %zu, needs %zu.",
                  decision.go ? "GO" : "no-go", fullBytes, arenaBytes,
                  poseBytes, decision.fullFramesAtCap, byteCap, minFrames);
    decision.memo = memo;
    return decision;
}

RigExecSparsityDecision
RigExecStream0SparsityDecision()
{
    return RigExecDecideSparsity(kRigExecSparsityStream0BipedArenaBytes,
                                 kRigExecSparsityStream0BipedPoseBytes,
                                 kRigExecFrameCacheDefaultByteCap);
}

size_t
RigExecRetainedFrameState::RetainedBytes() const
{
    return slotBytes + RigExecRetainedSourcesBytes(*this);
}

namespace {

size_t
_ArrayBytes(const VtValue &value)
{
    // The bench's counting for the array payloads rig inputs actually hold;
    // anything else counts its shell. Mirrors frameCache.cpp's table for the
    // common cases without duplicating its scalar list: scalars are small
    // and uniform beside the arrays that dominate the snapshot.
    if (value.IsHolding<VtVec3fArray>()) {
        return value.UncheckedGet<VtVec3fArray>().size() * sizeof(GfVec3f);
    }
    if (value.IsHolding<VtVec3dArray>()) {
        return value.UncheckedGet<VtVec3dArray>().size() * sizeof(GfVec3d);
    }
    if (value.IsHolding<VtVec2fArray>()) {
        return value.UncheckedGet<VtVec2fArray>().size() * sizeof(GfVec2f);
    }
    if (value.IsHolding<VtVec4fArray>()) {
        return value.UncheckedGet<VtVec4fArray>().size() * sizeof(GfVec4f);
    }
    if (value.IsHolding<VtFloatArray>()) {
        return value.UncheckedGet<VtFloatArray>().size() * sizeof(float);
    }
    if (value.IsHolding<VtDoubleArray>()) {
        return value.UncheckedGet<VtDoubleArray>().size() * sizeof(double);
    }
    if (value.IsHolding<VtIntArray>()) {
        return value.UncheckedGet<VtIntArray>().size() * sizeof(int);
    }
    if (value.IsHolding<VtBoolArray>()) {
        return value.UncheckedGet<VtBoolArray>().size() * sizeof(bool);
    }
    if (value.IsHolding<VtStringArray>()) {
        size_t total = 0;
        for (const std::string &s :
             value.UncheckedGet<VtStringArray>()) {
            total += s.size();
        }
        return total;
    }
    if (value.IsHolding<std::string>()) {
        return value.UncheckedGet<std::string>().size();
    }
    return sizeof(VtValue);
}

}  // namespace

size_t
RigExecRetainedSourcesBytes(const RigExecRetainedFrameState &state)
{
    size_t total = sizeof(RigExecRetainedFrameState);
    for (const RigExecSampledInput &sampled : state.inputs.values) {
        total += sampled.path.GetString().size() + sizeof(bool);
        if (sampled.hasValue) {
            total += _ArrayBytes(sampled.value);
        }
    }
    for (const RigExecValueOverride &o : state.overrides) {
        total += o.prim.GetString().size() + o.computation.GetString().size() +
                 o.attribute.GetString().size() + _ArrayBytes(o.value);
    }
    return total;
}

bool
RigExecSameSourceValue(const VtValue &a, bool aHas, const VtValue &b,
                       bool bHas)
{
    if (aHas != bHas) {
        return false;
    }
    if (!aHas) {
        return true;
    }
    return a == b;
}

std::vector<RigExecControlId>
RigExecChangedControls(const RigExecRetainedFrameState &cached,
                       const RigExecFrameInputs &requested,
                       const std::vector<RigExecValueOverride> &requestedOverrides)
{
    // First sample per path wins on both sides, matching FrameInputs::Find.
    std::map<SdfPath, const RigExecSampledInput *> before, after;
    for (const RigExecSampledInput &sampled : cached.inputs.values) {
        before.emplace(sampled.path, &sampled);
    }
    for (const RigExecSampledInput &sampled : requested.values) {
        after.emplace(sampled.path, &sampled);
    }
    std::vector<RigExecControlId> changed;
    for (const auto &kv : before) {
        const auto found = after.find(kv.first);
        if (found == after.end()) {
            changed.push_back(RigExecControlIdForPath(kv.first));
        } else if (!RigExecSameSourceValue(kv.second->value,
                                           kv.second->hasValue,
                                           found->second->value,
                                           found->second->hasValue)) {
            changed.push_back(RigExecControlIdForPath(kv.first));
        }
    }
    for (const auto &kv : after) {
        if (!before.count(kv.first)) {
            changed.push_back(RigExecControlIdForPath(kv.first));
        }
    }

    // Overrides key by (prim, computation, attribute) last-wins, matching
    // the digest and the evaluator's replace rule.
    using _Key = std::tuple<std::string, std::string, std::string>;
    std::map<_Key, const RigExecValueOverride *> oldOvr, newOvr;
    for (const RigExecValueOverride &o : cached.overrides) {
        oldOvr[_Key(o.prim.GetString(), o.computation.GetString(),
                   o.attribute.GetString())] = &o;
    }
    for (const RigExecValueOverride &o : requestedOverrides) {
        newOvr[_Key(o.prim.GetString(), o.computation.GetString(),
                   o.attribute.GetString())] = &o;
    }
    for (const auto &kv : oldOvr) {
        const auto found = newOvr.find(kv.first);
        if (found == newOvr.end() ||
            found->second->value != kv.second->value) {
            changed.push_back(
                RigExecControlIdForOverride(*kv.second));
        }
    }
    for (const auto &kv : newOvr) {
        if (!oldOvr.count(kv.first)) {
            changed.push_back(RigExecControlIdForOverride(*kv.second));
        }
    }
    std::sort(changed.begin(), changed.end());
    changed.erase(std::unique(changed.begin(), changed.end()), changed.end());
    return changed;
}

RigExecSparsePlan
RigExecPlanSparseReuse(const RigExecOutputAffectedIndex &index,
                       RigExecTaskListCache *memo,
                       const RigExecRetainedFrameState &cached,
                       uint64_t requestEpoch,
                       const RigExecFrameInputs &requested,
                       const std::vector<RigExecValueOverride> &requestedOverrides,
                       const RigExecBakedClusterSet *affectingRequested)
{
    RigExecSparsePlan plan;
    plan.clusters.Resize(index.ClusterCount());
    if (index.Empty() || index.EpochDigest() != requestEpoch ||
        cached.epochDigest != requestEpoch) {
        return plan;
    }
    if (cached.clusterCount != index.ClusterCount()) {
        // The topology moved under a standing epoch: nothing memoized for
        // this epoch can be trusted, and no plan can be drawn.
        plan.topologyChanged = true;
        if (memo) {
            memo->InvalidateEpoch(requestEpoch);
        }
        return plan;
    }
    plan.changedControls = RigExecChangedControls(cached, requested,
                                                  requestedOverrides);

    RigExecBakedClusterSet dirty;
    dirty.Resize(index.ClusterCount());
    if (plan.changedControls.empty()) {
        // §7's time rule, cross-frame: at a standing time nothing outside
        // the compared sources can have moved, so the cone is empty and the
        // retained pose is the answer. At a moved time the always-dirty
        // steps re-run -- their external reads are functions of time the
        // source vector does not name -- and their cones carry the rest.
        if (requested.time != cached.inputs.time) {
            dirty.Union(index.Always());
            dirty.Union(index.Varying());
        }
    } else if (memo) {
        for (const RigExecControlId &control : plan.changedControls) {
            RigExecBakedClusterSet selected;
            if (memo->Lookup(requestEpoch, control, &selected)) {
                plan.memoUsed = true;
            } else {
                selected = index.AffectedByControls({control});
                memo->Store(requestEpoch, control, selected);
            }
            dirty.Union(selected);
        }
        if (requested.time != cached.inputs.time) {
            dirty.Union(index.Always());
            dirty.Union(index.Varying());
        }
    } else {
        dirty = index.AffectedByControls(plan.changedControls);
        if (requested.time != cached.inputs.time) {
            dirty.Union(index.Always());
            dirty.Union(index.Varying());
        }
    }

    // dirty ∩ affecting(requested): a control that moved but reaches
    // nothing the request reads is still a hit.
    // Standing overrides dirty their cones even when no control value
    // moved between the retained frame and the request: the executor
    // dirties override-step clusters while an override stands (plan 2.1
    // planner/executor parity), so reuse must re-run them too.
    if (!requestedOverrides.empty()) {
        dirty.Union(index.Override());
    }
    RigExecBakedClusterSet closed = dirty;
    if (affectingRequested) {
        closed = RigExecIntersectClusterSets(dirty, *affectingRequested);
    }
    if (!closed.Any()) {
        plan.verdict = RigExecSparseVerdict::Hit;
        return plan;
    }
    plan.verdict = RigExecSparseVerdict::Partial;
    plan.clusters = closed;
    return plan;
}

RigExecSparseExecution
RigExecRunSparsePlan(const RigExecSparsePlan &plan,
                     RigExecClusterRunner runner)
{
    RigExecSparseExecution execution;
    execution.completed = false;
    if (plan.verdict == RigExecSparseVerdict::Miss || !runner) {
        return execution;
    }
    // Hit executes the empty set and completes: zero work is the answer,
    // not an unevaluated frame.
    execution.completed = true;
    // Increasing cluster order. A cone is closed under the cluster edges,
    // so any order that respects them is a valid execution order (§5.1) --
    // and a linear scan over a few dozen clusters costs nothing beside a
    // cluster's own work.
    const size_t clusterCount = plan.clusters.words.size() * 64;
    for (size_t c = 0; c < clusterCount; ++c) {
        if (!plan.clusters.Test(int(c))) {
            continue;
        }
        if (!runner(int(c))) {
            execution.completed = false;
            return execution;
        }
        ++execution.executed;
        execution.executedClusters.push_back(int(c));
    }
    return execution;
}

RigExecEntryProvenance
RigExecFullEvalProvenance(const RigExecBakedProgramImpl &program,
                          UsdTimeCode time)
{
    RigExecEntryProvenance provenance;
    const size_t clusters = program.clustering.clusters.size();
    provenance.clusters.reserve(clusters);
    for (size_t c = 0; c < clusters; ++c) {
        provenance.clusters.push_back(int(c));
    }
    for (const auto &object : program.weightObjects) {
        if (!object.path.IsEmpty()) {
            provenance.weightReads.push_back(object.path.GetString());
        }
    }
    std::sort(provenance.weightReads.begin(),
              provenance.weightReads.end());
    provenance.weightReads.erase(
        std::unique(provenance.weightReads.begin(),
                    provenance.weightReads.end()),
        provenance.weightReads.end());
    for (size_t b = 0; b < program.avarConstantBindings.size(); ++b) {
        const uint64_t region = RigExecConstantRegionForBinding(b);
        if (provenance.constantRegions.empty() ||
            provenance.constantRegions.back() != region) {
            provenance.constantRegions.push_back(region);
        }
    }
    provenance.aliasTimes.push_back(
        time.IsDefault() ? 0.0 : time.GetValue());
    return provenance;
}

RigExecRetainedFrameState
RigExecCaptureRetainedState(
    const RigExecFrameInputs &inputs,
    const std::vector<RigExecValueOverride> &overrides,
    uint64_t epochDigest, size_t clusterCount, uint64_t constantDigest)
{
    RigExecRetainedFrameState state;
    state.inputs = inputs;
    state.overrides = overrides;
    state.epochDigest = epochDigest;
    state.clusterCount = clusterCount;
    state.constantDigest = constantDigest;
    return state;
}

RigExecClusterRunner
RigExecMakeClusterRunner(RigExecClusterRebindContext context)
{
    return [context](int cluster) {
        if (!context.program || !context.runCluster) {
            return false;
        }
        return context.runCluster(*context.program, cluster);
    };
}

void
RigExecSparseCandidateIndex::NotePublished(
    uint64_t epoch, UsdTimeCode time, const RigExecFrameCacheKey &key)
{
    std::lock_guard<std::mutex> lock(_mutex);
    _map[std::make_pair(epoch, time)] = key;
}

bool
RigExecSparseCandidateIndex::Find(uint64_t epoch, UsdTimeCode time,
                                 RigExecFrameCacheKey *key) const
{
    if (!key) {
        return false;
    }
    std::lock_guard<std::mutex> lock(_mutex);
    const auto found = _map.find(std::make_pair(epoch, time));
    if (found == _map.end()) {
        return false;
    }
    *key = found->second;
    return true;
}

bool
RigExecSparseCandidateIndex::Drop(uint64_t epoch, UsdTimeCode time)
{
    std::lock_guard<std::mutex> lock(_mutex);
    return _map.erase(std::make_pair(epoch, time)) > 0;
}

void
RigExecSparseCandidateIndex::Clear()
{
    std::lock_guard<std::mutex> lock(_mutex);
    _map.clear();
}

size_t
RigExecSparseCandidateIndex::Size() const
{
    std::lock_guard<std::mutex> lock(_mutex);
    return _map.size();
}

size_t
RigExecInvalidateEpoch(RigExecFrameCache &cache, RigExecTaskListCache &memo,
                       uint64_t epochDigest)
{
    memo.InvalidateEpoch(epochDigest);
    return cache.EvictEpoch(epochDigest);
}

bool
RigExecNoteCaptureIndex(RigExecFrameCache &cache, RigExecTaskListCache &memo,
                        uint64_t epochDigest,
                        const RigExecBakedProgram &program,
                        const UsdNotice::ObjectsChanged &notice)
{
    if (!program.IsInvalidatedBy(notice)) {
        return false;
    }
    RigExecInvalidateEpoch(cache, memo, epochDigest);
    return true;
}

bool
RigExecFrameCacheVerifyRequested()
{
    return TfGetenvBool("RIGEXEC_FRAME_CACHE_VERIFY", false);
}

RigExecShadowVerdict
RigExecVerifyHitWithLive(const RigExecRigPose &cached,
                         RigExecLiveRunner live)
{
    RigExecShadowVerdict verdict;
    verdict.poseToServe = cached;
    if (!live) {
        verdict.report = "frame cache verify: no live runner, unverified";
        return verdict;
    }
    const RigExecRigPose run = live();
    if (!run.valid) {
        verdict.report = "frame cache verify: live eval failed, unverified";
        return verdict;
    }
    RigExecRigPose out;
    RigExecComparePoses(run, cached, &out);
    verdict.mismatches = out.bakedParityMismatches;
    verdict.match = verdict.mismatches == 0;
    if (verdict.match) {
        return verdict;
    }
    verdict.poseToServe = run;
    for (const std::string &line : out.diagnostics) {
        verdict.report += line;
        verdict.report += "\n";
    }
    return verdict;
}

}  // namespace rigExec
