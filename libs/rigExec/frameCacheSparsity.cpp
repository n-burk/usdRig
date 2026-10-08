// RigExec sparse cross-frame reuse. See frameCacheSparsity.h.
#include "frameCacheSparsity.h"
#include "bakedOpValues.h"

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
    return RigExecRetainedSourcesBytes(*this);
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
    // The constant head leaves are a table every frame sampled under one
    // program state shares, so no frame counts them.
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
    // Upstream values as the samples: path bytes plus the payload, shared
    // buffers counted per frame (an over-count, on the cap's safe side).
    for (const RigExecUpstreamValue &value : state.inputs.upstream) {
        total += value.path.GetString().size() + _ArrayBytes(value.value);
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
    return RigExecExactSourceValueEqual(a, b);
}

bool
RigExecSameUpstream(const std::vector<RigExecUpstreamValue> &a,
                    const std::vector<RigExecUpstreamValue> &b)
{
    if (a.size() != b.size()) {
        return false;
    }
    for (size_t i = 0; i < a.size(); ++i) {
        if (a[i].path != b[i].path || a[i].foldHash != b[i].foldHash ||
            !RigExecSameSourceValue(a[i].value, true, b[i].value, true)) {
            return false;
        }
    }
    return true;
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
        } else if (kv.second->valueBlocked != found->second->valueBlocked ||
                   !RigExecSameSourceValue(kv.second->value,
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
    // Layout sources have their own row/key routes: a same-path resolved
    // ordinary sample must not shadow a raw layout read.
    const auto &oldRows=cached.inputs.layoutLeaves;
    const auto &newRows=requested.layoutLeaves;
    const auto &oldPaths=cached.inputs.layoutSourcePaths;
    const auto &newPaths=requested.layoutSourcePaths;
    const size_t rowCount=std::max(oldRows.size(),newRows.size());
    for(size_t row=0;row<rowCount;++row) {
        const size_t oldCount=row<oldRows.size()?oldRows[row].size():0;
        const size_t newCount=row<newRows.size()?newRows[row].size():0;
        const size_t oldPathCount=row<oldPaths.size()?oldPaths[row].size():0;
        const size_t newPathCount=row<newPaths.size()?newPaths[row].size():0;
        const size_t count=std::max(std::max(oldCount,newCount),std::max(oldPathCount,newPathCount));
        for(size_t key=0;key<count;++key) {
            const bool haveOld=key<oldCount && key<oldPathCount;
            const bool haveNew=key<newCount && key<newPathCount;
            const bool same=haveOld && haveNew && oldPaths[row][key]==newPaths[row][key] &&
                RigExecExactSourceValueEqual(oldRows[row][key],newRows[row][key]);
            if(!same) {
                if(key<oldPathCount) changed.push_back(RigExecControlIdForPath(oldPaths[row][key]));
                if(key<newPathCount) changed.push_back(RigExecControlIdForPath(newPaths[row][key]));
            }
        }
    }
    // The constant head leaves, by key: one shared table is no change.
    if (cached.inputs.headLeafConstants != requested.headLeafConstants) {
        const auto constantsOf = [](const RigExecFrameInputs &inputs) {
            std::map<SdfPath, const VtValue *> out;
            if (const RigExecHeadLeafConstants *table =
                    inputs.headLeafConstants.get()) {
                for (size_t j = 0; j < table->keys.size(); ++j) {
                    if (!table->varying[j]) {
                        out.emplace(table->keys[j], &table->values[j]);
                    }
                }
            }
            return out;
        };
        const std::map<SdfPath, const VtValue *> was =
            constantsOf(cached.inputs);
        const std::map<SdfPath, const VtValue *> is = constantsOf(requested);
        for (const auto &kv : was) {
            const auto found = is.find(kv.first);
            if (found == is.end() ||
                !RigExecSameSourceValue(*kv.second, !kv.second->IsEmpty(),
                                        *found->second,
                                        !found->second->IsEmpty())) {
                changed.push_back(RigExecControlIdForPath(kv.first));
            }
        }
        for (const auto &kv : is) {
            if (!was.count(kv.first)) {
                changed.push_back(RigExecControlIdForPath(kv.first));
            }
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

bool
RigExecCanReuseRetainedPose(const RigExecOutputAffectedIndex &index,
                           const RigExecRetainedFrameState &cached,
                           uint64_t requestEpoch,
                           const RigExecFrameInputs &requested,
                           const std::vector<RigExecValueOverride> &overrides)
{
    if (index.Empty() || index.EpochDigest() != requestEpoch ||
        cached.epochDigest != requestEpoch ||
        cached.clusterCount != index.ClusterCount() ||
        !RigExecSameUpstream(cached.inputs.upstream, requested.upstream)) {
        return false;
    }
    const auto completeLayouts=[](const RigExecFrameInputs &inputs) {
        if(inputs.layoutLeaves.size()!=inputs.layoutSourcePaths.size()) return false;
        for(size_t row=0;row<inputs.layoutLeaves.size();++row) {
            if(inputs.layoutLeaves[row].size()!=inputs.layoutSourcePaths[row].size()) return false;
            for(const auto &path:inputs.layoutSourcePaths[row]) if(path.IsEmpty()) return false;
        }
        return true;
    };
    if(!completeLayouts(cached.inputs) || !completeLayouts(requested)) return false;
    auto dirty = index.AffectedByControls(
        RigExecChangedControls(cached, requested, overrides));
    if (requested.time != cached.inputs.time) {
        dirty.Union(index.Always());
        dirty.Union(index.Varying());
    }
    if (!overrides.empty()) dirty.Union(index.Override());
    return !dirty.Any();
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
    verdict.mismatches = out.comparisonMismatches;
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
