//
// RigExec warm-frame index. See warmIndex.h for the contract and the lock
// discipline.
//

#include "rigExecImaging/warmIndex.h"

namespace rigExec {

void
RigExecWarmFrameIndex::NoteGeneration(const SdfPath &rig,
                                      RigExecFrameGeneration generation)
{
    std::lock_guard<std::mutex> lock(_mutex);
    _generations[rig] = generation;
    // Revisited on edit: the edit may have fixed warmability.
    for (auto &entry : _frames) {
        if (entry.first.first == rig) {
            entry.second.streak = 0;
        }
    }
}

void
RigExecWarmFrameIndex::NoteSkipped(const SdfPath &rig, double timeValue)
{
    std::lock_guard<std::mutex> lock(_mutex);
    ++_frames[std::make_pair(rig, timeValue)].streak;
}

bool
RigExecWarmFrameIndex::IsUnwarmable(const SdfPath &rig,
                                    double timeValue) const
{
    std::lock_guard<std::mutex> lock(_mutex);
    const auto found = _frames.find(std::make_pair(rig, timeValue));
    return found != _frames.end() &&
        found->second.streak >= kRigExecWarmUnwarmableAfter;
}

bool
RigExecWarmFrameIndex::IsVisitable(
    const SdfPath &rig, double timeValue, RigExecFrameGeneration generation,
    uint64_t epochDigest) const
{
    std::lock_guard<std::mutex> lock(_mutex);
    const auto found = _frames.find(std::make_pair(rig, timeValue));
    if (found != _frames.end() &&
        found->second.streak >= kRigExecWarmUnwarmableAfter) {
        return false;
    }
    return _StateFor(found == _frames.end() ? nullptr : &found->second,
                     generation, epochDigest) !=
        RigExecWarmFrameState::Cached;
}

RigExecFrameGeneration
RigExecWarmFrameIndex::CurrentGeneration(const SdfPath &rig) const
{
    std::lock_guard<std::mutex> lock(_mutex);
    const auto found = _generations.find(rig);
    return found == _generations.end() ? 0 : found->second;
}

void
RigExecWarmFrameIndex::NoteCompleted(
    const SdfPath &rig, double timeValue, const RigExecFrameCacheKey &key,
    RigExecFrameGeneration generation)
{
    std::lock_guard<std::mutex> lock(_mutex);
    _Frame &frame = _frames[std::make_pair(rig, timeValue)];
    frame.hasCompletion = true;
    frame.key = key;
    frame.completedGeneration = generation;
    frame.dirty = false;
    frame.streak = 0;
}

void
RigExecWarmFrameIndex::NoteEvicted(const SdfPath &rig,
                                   const RigExecFrameCacheKey &key,
                                   double timeValue)
{
    std::lock_guard<std::mutex> lock(_mutex);
    const auto found = _frames.find(std::make_pair(rig, timeValue));
    if (found == _frames.end() || !found->second.hasCompletion ||
        !(found->second.key == key)) {
        return;
    }
    found->second.hasCompletion = false;
    // Retired frames re-pend: the eviction clears the streak too.
    found->second.streak = 0;
    found->second.dirty = false;
}

void
RigExecWarmFrameIndex::NoteDirtied(const SdfPath &rig, double timeValue)
{
    std::lock_guard<std::mutex> lock(_mutex);
    const auto found = _frames.find(std::make_pair(rig, timeValue));
    if (found == _frames.end() || !found->second.hasCompletion) {
        return;
    }
    found->second.dirty = true;
}

std::vector<UsdTimeCode>
RigExecWarmFrameIndex::CompletedTimes(const SdfPath &rig) const
{
    std::lock_guard<std::mutex> lock(_mutex);
    std::vector<UsdTimeCode> times;
    for (const auto &entry : _frames) {
        if (entry.first.first == rig && entry.second.hasCompletion) {
            times.emplace_back(entry.first.second);
        }
    }
    return times;
}

std::vector<std::pair<UsdTimeCode, RigExecFrameCacheKey>>
RigExecWarmFrameIndex::CompletedKeys(const SdfPath &rig) const
{
    std::lock_guard<std::mutex> lock(_mutex);
    std::vector<std::pair<UsdTimeCode, RigExecFrameCacheKey>> keys;
    for (const auto &entry : _frames) {
        if (entry.first.first == rig && entry.second.hasCompletion) {
            keys.emplace_back(UsdTimeCode(entry.first.second),
                              entry.second.key);
        }
    }
    return keys;
}

bool
RigExecWarmFrameIndex::FindKey(const SdfPath &rig, double timeValue,
                              RigExecFrameCacheKey *key) const
{
    if (!key) {
        return false;
    }
    std::lock_guard<std::mutex> lock(_mutex);
    const auto found = _frames.find(std::make_pair(rig, timeValue));
    if (found == _frames.end() || !found->second.hasCompletion) {
        return false;
    }
    *key = found->second.key;
    return true;
}

bool
RigExecWarmFrameIndex::FindCachedKey(const SdfPath &rig, double timeValue,
                                     RigExecFrameGeneration generation,
                                     uint64_t epochDigest,
                                     RigExecFrameCacheKey *key) const
{
    if (!key) {
        return false;
    }
    std::lock_guard<std::mutex> lock(_mutex);
    const auto found = _frames.find(std::make_pair(rig, timeValue));
    if (found == _frames.end()) {
        return false;
    }
    const _Frame &frame = found->second;
    // The _StateFor Cached rule minus queue visibility: a queued or
    // running re-warm retires nothing until its publish replaces the
    // row, so the standing completion serves while it flies.
    if (!frame.hasCompletion || frame.dirty ||
        frame.completedGeneration != generation ||
        frame.key.epochDigest != epochDigest) {
        return false;
    }
    *key = frame.key;
    return true;
}

void
RigExecWarmFrameIndex::NoteTransition(
    const RigExecWarmTransition &transition)
{
    std::lock_guard<std::mutex> lock(_mutex);
    _Frame &frame =
        _frames[std::make_pair(transition.rig, transition.timeValue)];
    switch (transition.kind) {
    case RigExecWarmTransitionKind::Queued:
        frame.queued = true;
        break;
    case RigExecWarmTransitionKind::Running:
        frame.queued = false;
        ++frame.running;
        break;
    case RigExecWarmTransitionKind::Finished:
        // The pop already cleared queued; clear again defensively (a
        // Finished never arrives without its Running, but the index
        // answers queries, not audits).
        frame.queued = false;
        if (frame.running > 0) {
            --frame.running;
        }
        // Only a genuine decline counts toward un-warmable: a publish
        // resets the streak, and a generation fence is transient (the
        // frame requeues under the live generation).
        if (transition.outcome == RigExecWarmOutcome::Published) {
            frame.streak = 0;
        } else if (transition.outcome ==
                   RigExecWarmOutcome::DeclinedInvalid) {
            ++frame.streak;
        }
        break;
    case RigExecWarmTransitionKind::Canceled:
    case RigExecWarmTransitionKind::Shed:
    case RigExecWarmTransitionKind::DroppedAtShutdown:
        frame.queued = false;
        break;
    case RigExecWarmTransitionKind::DroppedStale:
        if (frame.running > 0) {
            --frame.running;
        }
        break;
    }
}

void
RigExecWarmFrameIndex::ResetRig(const SdfPath &rig)
{
    std::lock_guard<std::mutex> lock(_mutex);
    for (auto it = _frames.begin(); it != _frames.end();) {
        if (it->first.first == rig) {
            it = _frames.erase(it);
        } else {
            ++it;
        }
    }
}

RigExecWarmFrameState
RigExecWarmFrameIndex::_StateFor(const _Frame *frame,
                                 RigExecFrameGeneration generation,
                                 uint64_t epochDigest)
{
    if (!frame) {
        return RigExecWarmFrameState::Uncached;
    }
    if (frame->queued || frame->running > 0) {
        return RigExecWarmFrameState::Warming;
    }
    if (!frame->hasCompletion) {
        return RigExecWarmFrameState::Uncached;
    }
    if (frame->dirty) {
        return RigExecWarmFrameState::Dirty;
    }
    if (frame->completedGeneration != generation) {
        return RigExecWarmFrameState::Dirty;
    }
    if (frame->key.epochDigest != epochDigest) {
        return RigExecWarmFrameState::Dirty;
    }
    return RigExecWarmFrameState::Cached;
}

RigExecWarmFrameState
RigExecWarmFrameIndex::State(const SdfPath &rig, double timeValue,
                             RigExecFrameGeneration generation,
                             uint64_t epochDigest) const
{
    std::lock_guard<std::mutex> lock(_mutex);
    const auto found = _frames.find(std::make_pair(rig, timeValue));
    return _StateFor(found == _frames.end() ? nullptr : &found->second,
                     generation, epochDigest);
}

std::vector<RigExecWarmFrameState>
RigExecWarmFrameIndex::States(const SdfPath &rig,
                              const std::vector<double> &timeValues,
                              RigExecFrameGeneration generation,
                              uint64_t epochDigest) const
{
    std::lock_guard<std::mutex> lock(_mutex);
    std::vector<RigExecWarmFrameState> states;
    states.reserve(timeValues.size());
    for (const double timeValue : timeValues) {
        const auto found = _frames.find(std::make_pair(rig, timeValue));
        states.push_back(
            _StateFor(found == _frames.end() ? nullptr : &found->second,
                      generation, epochDigest));
    }
    return states;
}

}  // namespace rigExec
