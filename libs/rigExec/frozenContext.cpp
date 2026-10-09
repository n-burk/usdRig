// Frozen evaluation API, input containers, serial scopes, and purity audit.

#include "frozenContextInternal.h"
#include "inputReplay.h"
#include "frameCache.h"
#include "generation.h"
#include <algorithm>
#include <cmath>
#include <set>

namespace rigExec {

using namespace frozenDetail;

namespace {

// Thread-local serial depth. A plain counter rather than a bool so scopes
// nest; the query is "inside at least one".
thread_local size_t _frozenSerialDepth = 0;

} // namespace

namespace frozenDetail {

thread_local RigExecFrozenRunReport *_frozenRunReport = nullptr;

} // namespace frozenDetail

std::vector<RigExecUpstreamValue>
RigExecUpstreamValuesOf(const std::vector<RigExecValueOverride> &inputs)
{
    std::map<SdfPath, const VtValue *> byPath;
    for (const RigExecValueOverride &o : inputs) {
        if (!o.attribute.IsEmpty() && o.computation.IsEmpty() &&
            !o.prim.IsEmpty()) {
            byPath[o.prim.AppendProperty(o.attribute)] = &o.value;
        }
    }
    std::vector<RigExecUpstreamValue> values;
    values.reserve(byPath.size());
    for (const auto &[path, value] : byPath) {
        values.push_back(
            RigExecUpstreamValue{path, *value, RigExecUpstreamFoldHash(*value)});
    }
    return values;
}

bool
RigExecFrameInputs::HasChainResolvedInputs() const
{
    for (const RigExecSampledInput &sampled : values) {
        if (sampled.viaChain) {
            return true;
        }
    }
    return false;
}

namespace {

// The constant head leaf keyed \p path, or -1.
int
_HeadConstantAt(const RigExecHeadLeafConstants *constants,
                const SdfPath &path)
{
    if (!constants) {
        return -1;
    }
    for (size_t j = 0; j < constants->keys.size(); ++j) {
        if (constants->keys[j] == path && !constants->varying[j]) {
            return int(j);
        }
    }
    return -1;
}

} // namespace

const VtValue *
RigExecFrameInputs::Find(const SdfPath &path) const
{
    for (const RigExecSampledInput &sampled : values) {
        if (sampled.path == path) {
            return &sampled.value;
        }
    }
    const int j = _HeadConstantAt(headLeafConstants.get(), path);
    return j < 0 ? nullptr : &headLeafConstants->values[size_t(j)];
}

bool
RigExecFrameInputs::Contains(const SdfPath &path) const
{
    for (const RigExecSampledInput &sampled : values) {
        if (sampled.path == path) {
            return true;
        }
    }
    return _HeadConstantAt(headLeafConstants.get(), path) >= 0;
}

void
RigExecFrameInputs::SetOverrides(const std::vector<RigExecValueOverride> &list)
{
    overrides = list;
    overridePaths = _FrozenOverridePaths(list);
}

void
RigExecFrameInputs::Clear()
{
    time = UsdTimeCode::Default();
    oraclePublications.reset();
    oracleWeightInputs.clear();
    values.clear();
    layoutLeaves.clear();
    layoutSourcePaths.clear();
    varyingLayoutRows.clear();
    varyingRevisionLeaves.clear();
    revisionLeaves.clear();
    derivedLeaves.clear();
    stageSeeds = RigExecStageFrameSeeds();
    overrides.clear();
    overridePaths.clear();
    headLeafConstants.reset();
    upstream.clear();
}

bool
RigExecFrozenArena::ResizeFor(const RigExecFrozenEvalContext &context)
{
    return Resize(context.slotCount);
}

bool
RigExecFrozenArena::Resize(size_t slots)
{
    if (slots > kMaxFrozenArenaSlots) {
        return false;
    }
    _slots.assign(slots, 0.0);
    return true;
}

void
RigExecFrozenArena::Clear()
{
    std::fill(_slots.begin(), _slots.end(), 0.0);
}

bool
RigExecFrozenSerialActive()
{
    return _frozenSerialDepth > 0;
}

RigExecFrozenSerialScope::RigExecFrozenSerialScope()
    : _active(true)
{
    ++_frozenSerialDepth;
}

RigExecFrozenSerialScope::~RigExecFrozenSerialScope()
{
    if (_active && _frozenSerialDepth > 0) {
        --_frozenSerialDepth;
    }
}

RigExecRigPose
RigExecEvaluateFrozen(const RigExecFrozenEvalContext &context,
                      const RigExecFrameInputs &inputs,
                      RigExecFrozenStepRunner runner,
                      const RigExecBackgroundScheduler *scheduler,
                      const SdfPath &rig, RigExecFrozenRunReport *report)
{
    RigExecInputReplayComparisonScope replayComparison("RigExecEvaluateFrozen");
    if (report) {
        report->Clear();
    }
    RigExecRigPose declined;
    declined.time = inputs.time;
    declined.valid = false;

    // D7: a refusal rig has no program a worker could run. The context
    // carries the flag so that even a job enqueued by mistake -- a caller
    // that never asked RigExecShouldEnqueueBackgroundJob -- declines here
    // rather than evaluating against a vector sampled for nothing.
    if (context.flags & kRigExecFrozenBakeRefused) {
        return declined;
    }
    // The vector and the context were sampled for different frames, or the
    // vector was truncated in flight. Running against a partial input set
    // would publish a pose no digest names: dropped, never served.
    if (inputs.values.size() != context.varyingInputCount) {
        return declined;
    }
    if (!runner) {
        return declined;
    }
    if (scheduler &&
        !RigExecGenerationCheckAtStart(*scheduler, rig,
                                       context.generation)) {
        return declined;
    }
    RigExecFrozenArena arena;
    if (!arena.ResizeFor(context)) {
        return declined;
    }
    // The serial scope is the D4 boundary: every kernel variant the runner
    // reaches takes its serial form on this thread, whatever the
    // process-wide switch says, so no work leaks back onto the shared arena
    // at normal priority. The runner must also never hop threads: the scope
    // constrains this thread and no other.
    RigExecRigPose pose;
    pose.time = inputs.time;
    pose.valid = false;
    bool ran = false;
    {
        RigExecFrozenSerialScope serial;
        // The production runner fills the caller's report on this thread;
        // the pointer is withdrawn however the runner leaves.
        struct ReportScope {
            explicit ReportScope(RigExecFrozenRunReport *r)
            {
                _frozenRunReport = r;
            }
            ~ReportScope() { _frozenRunReport = nullptr; }
        } reportScope(report);
        ran = runner(context, inputs, arena, &pose);
    }
    if (!ran || !pose.valid) {
        if (report) {
            report->Clear();
        }
        return declined;
    }
    pose.time = inputs.time;
    if (scheduler &&
        !RigExecGenerationCheckBeforePublish(*scheduler, rig,
                                             context.generation)) {
        return declined;
    }
    return pose;
}

RigExecRigPose
RigExecEvaluateFrozen(const RigExecFrozenEvalContext &context,
                      const RigExecFrameInputs &inputs)
{
    RigExecInputReplayComparisonScope replayComparison("RigExecEvaluateFrozen");
    // Stream 0 stub, retained: without a step runner no request can prove
    // bit-identity, so every request answers invalid -- the fail-closed
    // answer -- while still carrying the requested time for the fallback's
    // own logging.
    (void)context;
    RigExecRigPose pose;
    pose.time = inputs.time;
    pose.valid = false;
    return pose;
}

RigExecFrozenStepRunner
RigExecMakeProductionStepRunner()
{
    return [](const RigExecFrozenEvalContext &context,
              const RigExecFrameInputs &inputs, RigExecFrozenArena &arena,
              RigExecRigPose *pose) {
        (void)arena;
        if (!pose) {
            return false;
        }
        *pose = RigExecRigPose();
        if (!context.frozen) {
            // No program to run: the rig was not frozen (refused at freeze
            // time, or built before the executor landed). Decline -- hand
            // the generation back -- and the frame evaluates live when
            // asked.
            return false;
        }
        if (context.flags & kRigExecFrozenBakeRefused) {
            return false;
        }
        // The arena sizes the private working state; the runner carries its
        // own clone instead, so the check is only that the entry point sized
        // what the context promised.
        if (arena.Size() < context.slotCount) {
            return false;
        }
        return _RunFrozen(context, inputs, pose);
    };
}

bool
RigExecBindChainSampleInputs(const RigExecRigEvaluator &evaluator,
                             RigExecChainSampleBindings *out,
                             std::string *error)
{
    const auto fail = [&error](const std::string &why) {
        if (error) {
            *error = why;
        }
        return false;
    };
    if (!out) {
        return fail("no bindings to bind into");
    }
    const UsdStageRefPtr stage = evaluator.GetEvaluationStage();
    if (!stage) {
        return fail("no stage to bind the chains against");
    }
    std::vector<_ChainMoverDesc> movers;
    if (!_DiscoverChainMovers(evaluator, &movers, error)) {
        return false;
    }
    std::map<SdfPath, std::vector<_ChainMoverDesc>> grouped;
    for (const _ChainMoverDesc &mover : movers) {
        grouped[mover.target].push_back(mover);
    }
    std::vector<SdfPath> order;
    if (!_OrderDiscoveredChains(stage, grouped, &order, error)) {
        return false;
    }
    RigExecChainSampleBindings bound;
    for (const SdfPath &targetPath : order) {
        RigExecChainSampleChain chain;
        chain.targetPath = targetPath;
        chain.target = stage->GetAttributeAtPath(targetPath);
        if (chain.target) {
            chain.targetQuery = UsdAttributeQuery(chain.target);
            chain.valueType = chain.target.GetTypeName();
        }
        for (const _ChainMoverDesc &mover : grouped[targetPath]) {
            RigExecChainSampleRevision revision;
            revision.moverPath = mover.moverPath;
            revision.moverPrim = stage->GetPrimAtPath(mover.moverPath);
            if (const UsdRelationship rel = revision.moverPrim.GetRelationship(
                    TfToken("rigExec:weightObject"))) {
                rel.GetTargets(&revision.weightObjects);
            }
            const UsdPrim &prim = revision.moverPrim;
            revision.enabled = _BindChainInput(prim, "inputs:enabled");
            revision.defaultWeight =
                _BindChainInput(prim, "inputs:defaultWeight");
            revision.operation = _BindChainInput(prim, "rigExec:operation");
            revision.value = _BindChainInput(prim, "inputs:value");
            revision.minimum = _BindChainInput(prim, "inputs:min");
            revision.maximum = _BindChainInput(prim, "inputs:max");
            revision.keys = _BindChainInput(prim, "inputs:keys");
            revision.tangents = _BindChainInput(prim, "inputs:tangents");
            chain.revisions.push_back(std::move(revision));
        }
        for (const RigExecPhasedConnection &connection :
             evaluator.GetPhasedConnections()) {
            if (connection.target == targetPath) {
                chain.phased.push_back(connection);
            }
        }
        bound.chains.push_back(std::move(chain));
    }
    *out = std::move(bound);
    return true;
}

} // namespace rigExec
