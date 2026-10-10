// RigExec warm-frame index. See warmIndex.h for the contract and the lock
// discipline.
#include "rigExecImaging/warmIndex.h"

namespace rigExec {

void
RigExecWarmFrameIndex::NoteGeneration(const SdfPath &rig,
                                      RigExecFrameGeneration generation)
{
    std::lock_guard<std::mutex> lock(_mutex);
    _generations[rig] = generation;
    const auto request = _requests.find(rig);
    if (request != _requests.end()) {
        request->second.token = ++_nextRequest;
        for (auto &row : request->second.frames) row.second.reset();
    }
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
    const auto request = _requests.find(rig);
    if (request != _requests.end()) {
        request->second.token = ++_nextRequest;
        for (auto &row : request->second.frames) row.second.reset();
    }
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
    _requests.erase(rig);
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
    RigExecFrameGeneration generation, uint64_t epochDigest,
    std::vector<bool> *currentCompletions) const
{
    std::lock_guard<std::mutex> lock(_mutex);
    std::vector<RigExecWarmFrameState> states;
    states.reserve(timeValues.size());
    if (currentCompletions) {
        currentCompletions->clear();
        currentCompletions->reserve(timeValues.size());
    }
    for (const double timeValue : timeValues) {
        const auto found = _frames.find(std::make_pair(rig, timeValue));
        states.push_back(
            _StateFor(found == _frames.end() ? nullptr : &found->second,
                      generation, epochDigest));
        if (currentCompletions) {
            const _Frame *frame = found == _frames.end() ? nullptr : &found->second;
            currentCompletions->push_back(frame && frame->hasCompletion &&
                !frame->dirty && frame->completedGeneration == generation &&
                frame->key.epochDigest == epochDigest);
        }
    }
    return states;
}


// The request map has registry-owner serialization. Only captured cells
// cross the thread boundary; none of these methods acquires a new mutex.
void RigExecWarmFrameIndex::SetRequest(const SdfPath &rig,
    const std::vector<double> &times, RigExecFrameGeneration generation,
    uint64_t epochDigest)
{
    // Reuse the existing single batched cache-state critical section. A
    // standing completion seeds progress even while its rewarm is running.
    std::vector<bool> cached;
    States(rig, times, generation, epochDigest, &cached);
    _Request &request = _requests[rig];
    request.token = ++_nextRequest;
    request.frames.clear();
    for (size_t i = 0; i < times.size(); ++i)
        request.frames[times[i]] = cached[i]
            ? std::make_shared<_Progress>(generation, epochDigest, true)
            : std::shared_ptr<_Progress>();
}

uint64_t RigExecWarmFrameIndex::RequestToken(const SdfPath &rig) const
{
    const auto found = _requests.find(rig);
    return found == _requests.end() ? 0 : found->second.token;
}

void RigExecWarmFrameIndex::InvalidateRequest(const SdfPath &rig)
{
    const auto found = _requests.find(rig);
    if (found == _requests.end()) return;
    found->second.token = ++_nextRequest;
    for (auto &row : found->second.frames) row.second.reset();
}

RigExecWarmFrameIndex::RequestPublication
RigExecWarmFrameIndex::RequestPublicationFor(const SdfPath &rig,
    double timeValue, RigExecFrameGeneration generation, uint64_t epochDigest)
{
    RequestPublication result;
    const auto request = _requests.find(rig);
    const auto current = _generations.find(rig);
    const RigExecFrameGeneration live =
        current == _generations.end() ? 0 : current->second;
    if (request == _requests.end() || generation != live) return result;
    const auto row = request->second.frames.find(timeValue);
    if (row == request->second.frames.end()) return result;
    // A changed epoch/generation gets a new immutable cell, never an in-place
    // reset. Old jobs keep their old cell alive but cannot mark the new row.
    if (!row->second || row->second->generation != generation || row->second->epochDigest != epochDigest)
        row->second = std::make_shared<_Progress>(generation, epochDigest);
    result._progress = row->second;
    return result;
}

bool RigExecWarmFrameIndex::RequestPublication::Publish(
    const RigExecFrameCacheKey &key, RigExecFrameGeneration generation) const
{
    static_assert(std::atomic<uint8_t>::is_always_lock_free,
                  "warm publication handoff must not introduce a hidden lock");
    if (!_progress || generation != _progress->generation ||
        key.epochDigest != _progress->epochDigest) return false;
    uint8_t empty = 0;
    if (!_progress->state.compare_exchange_strong(empty, uint8_t(1),
            std::memory_order_relaxed, std::memory_order_relaxed)) return false;
    _progress->state.store(2, std::memory_order_release);
    return true;
}

void RigExecWarmFrameIndex::NoteRequestPublished(const SdfPath &rig,
    double timeValue, const RigExecFrameCacheKey &key,
    RigExecFrameGeneration generation, uint64_t requestToken)
{
    const auto request = _requests.find(rig);
    if (!requestToken || request == _requests.end() ||
        request->second.token != requestToken) return;
    RequestPublicationFor(rig, timeValue, generation, key.epochDigest).Publish(key, generation);
}

bool RigExecWarmFrameIndex::HasRequestPublished(const SdfPath &rig,
    double timeValue, RigExecFrameGeneration generation,
    uint64_t epochDigest) const
{
    const auto request = _requests.find(rig);
    if (request == _requests.end()) return false;
    const auto row = request->second.frames.find(timeValue);
    return row != request->second.frames.end() && row->second &&
        row->second->state.load(std::memory_order_acquire) == 2 &&
        row->second->generation == generation &&
        row->second->epochDigest == epochDigest;
}

bool RigExecWarmFrameIndex::IsCursorVisitable(const SdfPath &rig,
    double timeValue, RigExecFrameGeneration generation,
    uint64_t epochDigest) const
{
    return !HasRequestPublished(rig, timeValue, generation, epochDigest) &&
        IsVisitable(rig, timeValue, generation, epochDigest);
}

}  // namespace rigExec
