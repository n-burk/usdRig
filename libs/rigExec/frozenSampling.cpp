// Stage-side frame sampling and burst sample caches.

#include "frozenContextInternal.h"
#include "movers/moverRegistry.h"
#include "rigExecMath/geometryKernels.h"
#include <algorithm>
#include <cmath>
#include <set>

namespace rigExec {

using namespace frozenDetail;

namespace {

// The sampler below mirrors the binding walk the Stream 0 bench measures
// (tests/benchFrameCache.cpp MeasureSampling): the same tables, in the same
// order, through the same read route the baked frame path takes. The bench
// is the timing; this is the values.

// Reads a chain-resolved attribute exactly as RigExecResolvedInputs::
// GetAttribute does -- an in-memory property value wins, otherwise a single
// authored connection is followed, otherwise the attribute's own value is
// read -- but type-erased into a VtValue, because the sampler names no
// binding's type.
// GetAttribute itself cannot serve here: its in-memory lookup is typed
// (Find + IsHolding<T>), and T=VtValue never matches, so the chain outputs
// the sample exists to capture would be skipped and the stage read instead.
// The walk below is that function's connection logic restated without the
// type parameter; the float-from-double coercion it also performs is a
// CONSUMPTION rule (a typed read of a double source) and stays at the typed
// reads, which is where the frame path applies it too.
bool
_SampleResolvedAttribute(const RigExecResolvedInputs &resolved,
                         const UsdAttribute &attribute, UsdTimeCode time,
                         VtValue *out, bool *fromMemory = nullptr)
{
    std::set<SdfPath> visiting;
    std::vector<UsdAttribute> fallback;
    UsdAttribute a = attribute;
    while (a && visiting.insert(a.GetPath()).second) {
        if (const VtValue *held = resolved.Find(a.GetPath())) {
            *out = *held;
            if (fromMemory) {
                // In-memory state -- a chain output computed for the
                // evaluator's last run, or a standing override. The caller
                // decides which staleness that means for its binding.
                *fromMemory = true;
            }
            return true;
        }
        fallback.push_back(a);
        SdfPathVector connections;
        if (a.HasAuthoredConnections()) {
            a.GetConnections(&connections);
        }
        if (connections.size() != 1) {
            break;
        }
        a = a.GetPrim().GetStage()->GetAttributeAtPath(connections[0]);
    }
    for (auto it = fallback.rbegin(); it != fallback.rend(); ++it) {
        if (it->Get(out, time)) {
            // A stage read at the sampled time: fresh whatever the binding.
            if (fromMemory) {
                *fromMemory = false;
            }
            return true;
        }
    }
    if (fromMemory) {
        *fromMemory = false;
    }
    return false;
}

// Samples one bound input. Epoch constants are not per-frame inputs and are
// skipped; everything else is read through the route the frame path reads
// (RigExecBakedRead): the retained query, else the resolved walk, else --
// varying with neither handle, which the frame path answers from the typed
// constant -- that constant. \p head carries the binding's walk-start path,
// which keys the sample.
template <class T>
void
_SampleBinding(const RigExecBakedInput<T> &input,
               const RigExecResolvedInputs *resolved, UsdTimeCode time,
               RigExecFrameInputs *out,
               const RigExecResolvedInputs *chainFresh = nullptr)
{
    if (!input.varying) {
        return;
    }
    // A binding with a chain on its walk reads through the hook-refreshed
    // values when the hook ran: fresh at the sampled time by construction,
    // whatever the evaluator last ran. Without the hook it reads the
    // standing state and marks stale below.
    const bool fresh = chainFresh && input.resolvedAttr.IsValid();
    const RigExecResolvedInputs *source = fresh ? chainFresh : resolved;
    VtValue value;
    bool hasValue = false;
    bool fromMemory = false;
    if (input.query.IsValid()) {
        hasValue = input.query.Get(&value, time);
    } else if (input.resolvedAttr.IsValid() && source) {
        hasValue =
            _SampleResolvedAttribute(*source, input.resolvedAttr, time,
                                     &value, &fromMemory);
    } else if (input.head.IsValid()) {
        // The frame path answers input.constant here, so the sample is that
        // constant rather than a valueless marker: valueless would tell the
        // worker "no value", and the frame path never sees no value on this
        // binding.
        value = VtValue(input.constant);
        hasValue = true;
    } else {
        return;
    }
    if (!input.head.IsValid()) {
        return;
    }
    // Chain-resolved only when BOTH hold: a chain on the walk (resolvedAttr
    // is set exactly then) AND the value came from in-memory state rather
    // than a fresh stage read -- and never when the hook refreshed the
    // values the read walked through. A stage fallback on a chained binding
    // is fresh -- and a lookup that would have read in-memory digests apart
    // from it, so it can only miss, never serve wrong. In-memory hits on
    // chainless bindings are standing overrides, which are timeless and
    // digest-covered, so they stay unmarked and preview-time warming stands.
    const bool viaChain =
        input.resolvedAttr.IsValid() && fromMemory && !fresh;
    out->Add(input.head.GetPath(), value, hasValue, viaChain);
}

template <class T>
void
_SamplePromotedAvar(const RigExecBakedInput<T> &input,
                    const RigExecResolvedInputs *resolved, UsdTimeCode time,
                    RigExecFrameInputs *out)
{
    // A constant binding an edit has animated since: read per frame the long
    // way, like a varying binding, until an edit makes it constant again.
    if (!input.head.IsValid()) {
        return;
    }
    VtValue value;
    bool hasValue = false;
    if (resolved) {
        // Never viaChain: a promoted avar classified constant at Build, so
        // no chain stands on its walk and an in-memory hit is a standing
        // override -- timeless and digest-covered, like any other.
        hasValue =
            _SampleResolvedAttribute(*resolved, input.head, time, &value);
    }
    if (!hasValue) {
        value = VtValue(input.constant);
        hasValue = true;
    }
    out->Add(input.head.GetPath(), value, hasValue);
}

bool
_IsOverridden(const std::vector<char> &flags, int overrideIndex)
{
    return overrideIndex >= 0 &&
           size_t(overrideIndex) < flags.size() &&
           flags[size_t(overrideIndex)];
}

// Whether one bound input needs a per-frame visit under a burst's placed
// override flags: varying inputs are re-read every frame and overridden
// ones are read the long way, while anything else early-outs without
// adding a sample -- so a struct whose every field answers false can be
// skipped without changing the vector.
template <class T>
bool
_BurstInputNeedsVisit(const RigExecBakedInput<T> &input,
                      const std::vector<char> &flags)
{
    return input.varying || _IsOverridden(flags, input.overrideIndex);
}

bool
_BurstLadderNeedsVisit(const RigExecBakedProgramImpl::Ladder &ladder,
                       const std::vector<char> &flags)
{
    bool needed = false;
    _VisitLadderInputs(ladder, [&](const auto &input) {
        needed = needed || _BurstInputNeedsVisit(input, flags);
    });
    return needed;
}

bool
_BurstSpaceSwitchNeedsVisit(
    const RigExecBakedProgramImpl::SpaceSwitch &spaceSwitch,
    const std::vector<char> &flags)
{
    bool needed = false;
    _VisitSpaceSwitchInputs(spaceSwitch, [&](const auto &input) {
        needed = needed || _BurstInputNeedsVisit(input, flags);
    });
    return needed;
}

bool
_BurstSolverNeedsVisit(const RigExecBakedProgramImpl::Solver &solver,
                       const std::vector<char> &flags)
{
    bool needed = false;
    _VisitSolverInputs(solver, [&](const auto &input) {
        needed = needed || _BurstInputNeedsVisit(input, flags);
    });
    return needed;
}

bool
_BurstConstraintNeedsVisit(
    const RigExecBakedProgramImpl::Constraint &constraint,
    const std::vector<char> &flags)
{
    bool needed = false;
    _VisitConstraintInputs(constraint, [&](const auto &input) {
        needed = needed || _BurstInputNeedsVisit(input, flags);
    });
    return needed;
}

bool
_BurstWeightNeedsVisit(const RigExecBakedProgramImpl::WeightObject &object,
                       const std::vector<char> &flags)
{
    bool needed = false;
    _VisitWeightInputs(object, [&](const auto &input) {
        needed = needed || _BurstInputNeedsVisit(input, flags);
    });
    return needed;
}

bool
_BurstInterpolatorNeedsVisit(
    const RigExecBakedProgramImpl::PoseInterpolator &interp,
    const std::vector<char> &flags)
{
    bool needed = false;
    _VisitInterpolatorInputs(interp, [&](const auto &input) {
        needed = needed || _BurstInputNeedsVisit(input, flags);
    });
    return needed;
}

// Samples one binding under the job's overrides. Unoverridden bindings
// sample exactly as before; an overridden binding is read the long way --
// the resolved walk from its head, which is what RigExecBakedRead's
// overridden arm answers -- through the refreshed inputs (job overrides
// placed, stage reads at the job's time). The sample then carries the value
// the frame consumes, and the worker patches it as a constant; no separate
// override value travels.
// A chain-resolved binding under an override still depends on chain outputs
// whenever the override misses mid-walk, so without the hook's refreshed
// values it is marked stale and the vector declines.
template <class T>
void
_SampleBindingWithOverrides(const RigExecBakedInput<T> &input,
                            const RigExecResolvedInputs *standing,
                            const RigExecResolvedInputs *refreshed,
                            bool overridden, UsdTimeCode time,
                            RigExecFrameInputs *out,
                            const RigExecResolvedInputs *chainFresh = nullptr)
{
    if (!overridden) {
        _SampleBinding(input, standing, time, out, chainFresh);
        return;
    }
    if (!input.head.IsValid()) {
        return;
    }
    T value = input.constant;
    if (refreshed) {
        refreshed->GetAttribute(input.head, time, &value);
    }
    out->Add(input.head.GetPath(), VtValue(value),
             /*hasValue=*/true,
             /*viaChain=*/input.resolvedAttr.IsValid() && !chainFresh);
}

template <class T>
void
_SampleFlaggedBinding(const RigExecBakedInput<T> &input,
                      const RigExecResolvedInputs *standing,
                      const RigExecResolvedInputs *refreshed,
                      const std::vector<char> &flags, UsdTimeCode time,
                      RigExecFrameInputs *out,
                      const RigExecResolvedInputs *chainFresh = nullptr)
{
    _SampleBindingWithOverrides(input, standing, refreshed,
                                _IsOverridden(flags, input.overrideIndex),
                                time, out, chainFresh);
}

void
_SampleSolverBindings(const RigExecBakedProgramImpl::Solver &solver,
                      const RigExecResolvedInputs *standing,
                      const RigExecResolvedInputs *refreshed,
                      const std::vector<char> &flags, UsdTimeCode time,
                      RigExecFrameInputs *out,
                      const RigExecResolvedInputs *chainFresh = nullptr)
{
    _VisitSolverInputs(solver, [&](const auto &input) {
        _SampleFlaggedBinding(input, standing, refreshed, flags, time, out,
                              chainFresh);
    });
}

void
_SampleConstraintBindings(
    const RigExecBakedProgramImpl::Constraint &constraint,
    const RigExecResolvedInputs *standing,
    const RigExecResolvedInputs *refreshed, const std::vector<char> &flags,
    UsdTimeCode time, RigExecFrameInputs *out,
    const RigExecResolvedInputs *chainFresh = nullptr)
{
    _VisitConstraintInputs(constraint, [&](const auto &input) {
        _SampleFlaggedBinding(input, standing, refreshed, flags, time, out,
                              chainFresh);
    });
}

// _ApplyInteractiveOverridesToResolved over the job's overrides: every
// attribute override stands in the given inputs. The `overrides` out-param
// of the live routine feeds exec only; the baked path never reads it.
void
_PlaceOverridesIntoResolved(
    const std::vector<RigExecValueOverride> &overrides,
    RigExecResolvedInputs *resolved)
{
    for (const RigExecValueOverride &o : overrides) {
        if (o.attribute.IsEmpty()) {
            continue;
        }
        resolved->SetProperty(o.prim.AppendProperty(o.attribute), o.value);
    }
}

void
_SampleWeightBindings(const RigExecBakedProgramImpl::WeightObject &object,
                      const RigExecResolvedInputs *standing,
                      const RigExecResolvedInputs *refreshed,
                      const std::vector<char> &flags, UsdTimeCode time,
                      RigExecFrameInputs *out,
                      const RigExecResolvedInputs *chainFresh)
{
    _VisitWeightInputs(object, [&](const auto &input) {
        _SampleFlaggedBinding(input, standing, refreshed, flags, time, out,
                              chainFresh);
    });
}

// Reads one array attribute through the resolved inputs first, then the
// stage -- the route RigExecBakedWeightPacket's gather arms take
// (resolved->GetAttribute, which itself reads composed opinions on a miss,
// plus the stage fallback _SampleMoverScalar applies).
template <class T>
bool
_FrozenReadWeightArray(const RigExecResolvedInputs *refreshed,
                       const UsdAttribute &attribute, UsdTimeCode time,
                       VtArray<T> *out)
{
    if (attribute.IsValid() && refreshed &&
        refreshed->GetAttribute(attribute, time, out)) {
        return true;
    }
    if (attribute.IsValid() && attribute.Get(out, time)) {
        return true;
    }
    return false;
}

// Gathers one weight object's point arrays exactly as the packet build's
// gather arms do (bakedWeights.cpp), under the synthetic keys the frozen
// packet build reads. Roles with no attributes sample nothing; the worker
// answers those as empty, which is what the live gather yields. Failed
// reads contribute nothing to a concatenation, matching the live arms.
void
_SampleWeightArrays(const RigExecBakedProgramImpl::WeightObject &object,
                    const RigExecResolvedInputs *refreshed, UsdTimeCode time,
                    RigExecFrameInputs *out)
{
    const auto gatherPoints =
        [&](const std::vector<UsdAttribute> &attributes, const char *role) {
            if (attributes.empty()) {
                return;
            }
            VtVec3fArray gathered;
            for (const UsdAttribute &a : attributes) {
                VtVec3fArray value;
                if (_FrozenReadWeightArray(refreshed, a, time, &value)) {
                    for (const GfVec3f &p : value) {
                        gathered.push_back(p);
                    }
                }
            }
            out->Add(_FrozenWeightArrayKey(object.path, role),
                     VtValue(gathered), /*hasValue=*/true);
        };
    gatherPoints(object.targetPoints, "targetPoints");
    gatherPoints(object.samplePoints, "samplePoints");
    gatherPoints(object.curvePoints, "curvePoints");
    if (!object.combineTargetPoints.empty()) {
        size_t count = 0;
        for (const UsdAttribute &a : object.combineTargetPoints) {
            VtVec3fArray value;
            if (_FrozenReadWeightArray(refreshed, a, time, &value)) {
                count += value.size();
            }
        }
        // The worker casts back to size_t; mesh point counts never approach
        // int range, and int folds exactly in every digest (uint64_t would
        // not -- _HashVtValue has no uint64 arm).
        out->Add(_FrozenWeightArrayKey(object.path, "combineTargetCount"),
                 VtValue(static_cast<int>(count)), /*hasValue=*/true);
    }
}

// Samples one wire revision's side inputs exactly as the wire assembly arm
// reads them (moverGraph.cpp): rest points / order / knots at Default,
// dropoff at time, bind coordinates resolved-first, driver weight arrays
// resolved-first, and the posed driver curve (only when no driver
// transforms bind the table path). All under synthetic mover keys; the
// worker replays them, it cannot read the mover prim or the stage.
void
_SampleMoverToken(const UsdPrim &moverPrim, const SdfPath &moverPath,
                  const char *name, UsdTimeCode time, RigExecFrameInputs *out);

void
_SampleWireInputs(const RigExecBakedProgramImpl::GeomRevision &revision,
                  const RigExecResolvedInputs *refreshed,
                  const UsdStageRefPtr &stage, UsdTimeCode time,
                  RigExecFrameInputs *out)
{
    const UsdPrim &moverPrim = revision.moverPrim;
    const RigExecRevisionBinding &binding = revision.binding;
    const SdfPath &moverPath = revision.moverPath;
    if (moverPrim && !binding.driverCurvePoints.IsEmpty() && stage) {
        VtVec3fArray rest;
        bool has = false;
        if (const UsdAttribute a =
                stage->GetAttributeAtPath(binding.driverCurvePoints)) {
            has = a.Get(&rest, UsdTimeCode::Default());
        }
        out->Add(_FrozenWireInputKey(moverPath, "restPoints"),
                 VtValue(has ? rest : VtVec3fArray()), has);
    }
    if (moverPrim && !binding.driverCurveOrder.IsEmpty() && stage) {
        int order = 0;
        bool has = false;
        if (const UsdAttribute a =
                stage->GetAttributeAtPath(binding.driverCurveOrder)) {
            VtIntArray value;
            if (a.Get(&value, UsdTimeCode::Default()) && !value.empty()) {
                order = value[0];
                has = true;
            }
        }
        out->Add(_FrozenWireInputKey(moverPath, "curveOrder"),
                 VtValue(order), has);
    }
    if (moverPrim && !binding.driverCurveKnots.IsEmpty() && stage) {
        VtDoubleArray knots;
        bool has = false;
        if (const UsdAttribute a =
                stage->GetAttributeAtPath(binding.driverCurveKnots)) {
            has = a.Get(&knots, UsdTimeCode::Default());
        }
        out->Add(_FrozenWireInputKey(moverPath, "curveKnots"),
                 VtValue(has ? knots : VtDoubleArray()), has);
    }
    if (const UsdAttribute a =
            moverPrim.GetAttribute(TfToken("inputs:dropoffDistance"))) {
        float dropoff = 0.0f;
        const bool has = a.Get(&dropoff, time);
        // Live assigns even on a failed read (0.0f init), so hasValue is
        // always true here; the value carries the outcome.
        out->Add(_FrozenWireInputKey(moverPath, "dropoffDistance"),
                 VtValue(dropoff), /*hasValue=*/true);
        (void)has;
    }
    if (!binding.bindCoords.IsEmpty()) {
        VtArray<GfVec2f> coords;
        bool has = refreshed && refreshed->Get(binding.bindCoords, &coords);
        if (!has && stage) {
            if (const UsdAttribute a =
                    stage->GetAttributeAtPath(binding.bindCoords)) {
                has = a.Get(&coords, time);
            }
        }
        out->Add(_FrozenWireInputKey(moverPath, "bindCoords"),
                 VtValue(has ? coords : VtArray<GfVec2f>()), has);
    }
    const auto floats = [&](const char *name, const char *role) {
        const UsdAttribute a = moverPrim.GetAttribute(TfToken(name));
        if (!a.IsValid()) {
            return;
        }
        VtFloatArray value;
        bool has = refreshed && refreshed->GetAttribute(a, time, &value);
        if (!has) {
            has = a.Get(&value, time);
        }
        out->Add(_FrozenWireInputKey(moverPath, role),
                 VtValue(has ? value : VtFloatArray()), has);
    };
    floats("inputs:driverWeights", "driverWeights");
    floats("inputs:driverBaseWeights", "driverBaseWeights");
    _SampleMoverToken(moverPrim, moverPath, "rigExec:pointFrame", time, out);
    _SampleMoverToken(moverPrim, moverPath, "rigExec:driverDeltaFrame", time,
                      out);
    if (binding.driverTransformCount == 0 &&
        !binding.driverCurvePoints.IsEmpty()) {
        VtVec3fArray posed;
        bool has = refreshed &&
                   refreshed->Get(binding.driverCurvePoints, &posed);
        if (!has && stage) {
            if (const UsdAttribute a =
                    stage->GetAttributeAtPath(binding.driverCurvePoints)) {
                has = a.Get(&posed, time);
            }
        }
        out->Add(_FrozenWireInputKey(moverPath, "driverCurvePoints"),
                 VtValue(has ? posed : VtVec3fArray()), has);
    }
}

// A structural token the live assembler reads raw and records
// (_RecordedToken in moverGraph.cpp), sampled under its own property path:
// unsampled, the replay would take the fallback where live took the
// authored value. Only an authored token is sampled. An unauthored one
// reads its schema fallback live, which is the replay's fallback too, and
// every matrix or wire mover would otherwise add a sample to every cached
// frame.
void
_SampleMoverToken(const UsdPrim &moverPrim, const SdfPath &moverPath,
                  const char *name, UsdTimeCode time, RigExecFrameInputs *out)
{
    if (!moverPrim) {
        return;
    }
    const UsdAttribute a = moverPrim.GetAttribute(TfToken(name));
    if (!a || !a.HasAuthoredValue()) {
        return;
    }
    TfToken value;
    const bool has = a.Get(&value, time);
    out->Add(moverPath.AppendProperty(TfToken(name)), VtValue(value), has);
}

void
_SampleAttribute(const SdfPath &key, const UsdAttribute &attribute,
                 UsdTimeCode time, RigExecFrameInputs *out)
{
    if (!attribute.IsValid() || key.IsEmpty()) {
        return;
    }
    VtValue value;
    const bool hasValue = attribute.Get(&value, time);
    out->Add(key, value, hasValue);
}

// Samples one dense blend sample's target points: the refreshed inputs
// first -- R.GetAttribute follows single authored connections into the
// program's published properties, so a chain output or override standing
// on the target is what live reads -- else the stage at the job's time.
// Mirrors the dense arm's R.GetAttribute exactly (miss reads as empty);
// a dangling handle samples nothing, which the worker answers as empty.
void
_SampleBlendPoints(const SdfPath &key, const UsdAttribute &attribute,
                   UsdTimeCode time,
                   const RigExecResolvedInputs *refreshed,
                   RigExecFrameInputs *out)
{
    if (!attribute.IsValid() || key.IsEmpty()) {
        return;
    }
    VtVec3fArray value;
    bool hasValue =
        refreshed && refreshed->GetAttribute(attribute, time, &value);
    if (!hasValue) {
        hasValue = attribute.Get(&value, time);
    }
    out->Add(key, VtValue(hasValue ? value : VtVec3fArray()), hasValue);
}

// Samples the stage-frame seeds at the job's time through the program's
// hook (bakedProgram.cpp: the stageFrames of Run, read-only) and carries
// them on the vector. Both sampler routes call this identically: the seeds
// are per-frame stage reads, so no burst memo serves them. False names an
// unresolvable target, at which live gives the generation back.
bool
_SampleStageFrameSeeds(const RigExecBakedProgram &program, UsdTimeCode time,
                       RigExecFrameInputs *sampled, std::string *error)
{
    RigExecStageFrameSeeds seeds;
    if (!program.SampleStageFrameSeeds(time, &seeds, error)) {
        return false;
    }
    sampled->stageSeeds = std::move(seeds);
    return true;
}

// Resolves every sparse blend sample's layout through the live cache,
// exactly as the geometry prologue does (bakedGeometry.cpp
// resolveBlendLayouts): the worker cannot take the cache's lock, so the
// layout travels with the job, parallel to revisionIndex. pointCount is
// the chain's base size at the job's time -- the same query the prologue
// sizes from, read again rather than looked up, because a dense sample
// may target the chain's own base path and its refreshed-first sample
// would win the lookup over the raw base. Epoch data, like the skin
// packets: deliberately excluded from the digest.
bool
_SampleBlendLayouts(const RigExecBakedProgramImpl &B, UsdTimeCode time,
                    RigExecFrameInputs *sampled, std::string *error)
{
    sampled->blendLayouts.resize(B.revisionIndex.size());
    for (size_t r = 0; r < B.revisionIndex.size(); ++r) {
        const auto &[chainIndex, revisionIndex] = B.revisionIndex[r];
        const RigExecBakedProgramImpl::GeomChain &chain =
            B.chains[size_t(chainIndex)];
        const RigExecBakedProgramImpl::GeomRevision &revision =
            chain.revisions[size_t(revisionIndex)];
        if (revision.blendChannels.empty()) {
            continue;
        }
        VtVec3fArray base;
        if (!chain.baseQuery.IsValid() ||
            !chain.baseQuery.Get(&base, time)) {
            continue;  // no base: live skips the chain before resolving
        }
        if (!B.blendSampleShapes || !B.resolveBlendSample) {
            if (error) {
                *error = "blend layouts have no live cache to resolve "
                         "through; the frozen job cannot assemble them";
            }
            return false;
        }
        sampled->blendLayouts[r].resize(revision.blendChannels.size());
        for (size_t c = 0; c < revision.blendChannels.size(); ++c) {
            const RigExecBakedProgramImpl::GeomBlendChannel &channel =
                revision.blendChannels[c];
            sampled->blendLayouts[r][c].resize(channel.samples.size());
            for (size_t s = 0; s < channel.samples.size(); ++s) {
                const RigExecBakedProgramImpl::GeomBlendChannel::Sample
                    &sample = channel.samples[s];
                if (sample.blendShape.IsEmpty()) {
                    continue;  // dense: points rode the sampled values
                }
                std::shared_ptr<const RigExecBlendSampleLayout> layout =
                    B.blendSampleShapes->Resolve(
                        sample.samplePath,
                        [&](RigExecBlendSampleLayout *built) {
                            return B.resolveBlendSample(sample.blendShape,
                                                        base.size(), built);
                        });
                if (!layout) {
                    // Refused the cache: read per frame, as the prologue.
                    auto perFrame =
                        std::make_shared<RigExecBlendSampleLayout>();
                    B.resolveBlendSample(sample.blendShape, base.size(),
                                         perFrame.get());
                    layout = perFrame;
                }
                sampled->blendLayouts[r][c][s] = std::move(layout);
            }
        }
    }
    return true;
}

void
_SampleQuery(const SdfPath &key, const UsdAttributeQuery &query,
             UsdTimeCode time, RigExecFrameInputs *out)
{
    if (!query.IsValid() || key.IsEmpty()) {
        return;
    }
    VtValue value;
    const bool hasValue = query.Get(&value, time);
    out->Add(key, value, hasValue);
}

// Samples one mover scalar input the packet assembly reads: the resolved
// walk first (an override or a chain output standing on it), else the stage
// at the job's time, else the fallback. Mirrors moverGraph.cpp's _Float /
// _Enabled / skinningMethod arms exactly (no sample when the attribute does
// not exist; the worker falls back the same way).
template <class T>
void
_SampleMoverScalar(const SdfPath &key, const UsdAttribute &attribute,
                   T fallback, UsdTimeCode time,
                   const RigExecResolvedInputs *refreshed,
                   RigExecFrameInputs *out)
{
    if (!attribute.IsValid() || key.IsEmpty()) {
        return;
    }
    T value = fallback;
    if (!(refreshed && refreshed->GetAttribute(attribute, time, &value))) {
        attribute.Get(&value, time);
    }
    out->Add(key, VtValue(value), /*hasValue=*/true);
}

// RigExecReadProjectorTarget's reads, sampled where it reads them: the
// settings off the projector, the dials through the refreshed inputs.
void
_SampleProjectorInputs(const RigExecBakedProgramImpl::GeomRevision &revision,
                       const RigExecResolvedInputs *refreshed,
                       const UsdStageRefPtr &stage, UsdTimeCode time,
                       RigExecFrameInputs *out)
{
    const UsdPrim &prim = revision.moverPrim;
    if (!prim || !stage) {
        return;
    }
    if (revision.op == RigExecRevisionOp::ShaderDials) {
        for (const SdfPath &dial : revision.binding.shaderDials) {
            const UsdAttribute a = stage->GetAttributeAtPath(dial);
            double value = 0.0;
            if (a && a.GetTypeName() == SdfValueTypeNames->Float) {
                float asFloat = 0.0f;
                if (!(refreshed && refreshed->GetAttribute(a, time, &asFloat))) {
                    a.Get(&asFloat, time);
                }
                value = double(asFloat);
            } else if (a) {
                if (!(refreshed && refreshed->GetAttribute(a, time, &value))) {
                    a.Get(&value, time);
                }
            }
            out->Add(dial, VtValue(value), /*hasValue=*/true);
        }
        return;
    }
    const SdfPath &path = revision.moverPath;
    _SampleMoverScalar(path.AppendProperty(TfToken("rigExec:rayOrigin")),
                       prim.GetAttribute(TfToken("rigExec:rayOrigin")),
                       GfVec3d(0.0, 0.0, 0.0), time, refreshed, out);
    _SampleMoverScalar(path.AppendProperty(TfToken("rigExec:rayDirection")),
                       prim.GetAttribute(TfToken("rigExec:rayDirection")),
                       GfVec3d(0.0, 0.0, 1.0), time, refreshed, out);
    _SampleMoverScalar(path.AppendProperty(TfToken("rigExec:rayUp")),
                       prim.GetAttribute(TfToken("rigExec:rayUp")),
                       GfVec3d(0.0, 1.0, 0.0), time, refreshed, out);
    _SampleMoverScalar(path.AppendProperty(TfToken("rigExec:shaderOffset")),
                       prim.GetAttribute(TfToken("rigExec:shaderOffset")),
                       GfMatrix4d(1.0), time, refreshed, out);
    _SampleMoverToken(prim, path, "rigExec:projectionMode", time, out);
}

// Samples one derived-topology array: the resolved memory first (an
// override standing on the mesh attributes), else the stage at the job's
// time. Mirrors _Array's two arms; a dangling path or a failed read samples
// valueless, which the worker answers as the empty array.
template <class T>
void
_SampleTopologyArray(const SdfPath &path, UsdTimeCode time,
                     const RigExecResolvedInputs *refreshed,
                     const UsdStageRefPtr &stage, RigExecFrameInputs *out)
{
    if (path.IsEmpty()) {
        return;
    }
    VtArray<T> value;
    bool hasValue = refreshed && refreshed->Get(path, &value);
    if (!hasValue && stage) {
        if (const UsdAttribute a = stage->GetAttributeAtPath(path)) {
            hasValue = a.Get(&value, time);
        }
    }
    out->Add(path, VtValue(value), hasValue);
}

void
_SampleIterativeMoverInputs(
    const RigExecBakedProgramImpl::GeomRevision &revision,
    const RigExecResolvedInputs *refreshed, const UsdStageRefPtr &stage,
    UsdTimeCode time, RigExecFrameInputs *out)
{
    if (revision.op != RigExecRevisionOp::DeltaMush &&
        revision.op != RigExecRevisionOp::Wrinkle) {
        return;
    }
    _SampleTopologyArray<GfVec3f>(
        revision.moverPath.AppendProperty(TfToken("inputs:restPoints")),
        UsdTimeCode::Default(), refreshed, stage, out);
    _SampleTopologyArray<int>(revision.binding.topologyCounts, time,
                              refreshed, stage, out);
    _SampleTopologyArray<int>(revision.binding.topologyIndices, time,
                              refreshed, stage, out);
    RigExecMoverParameters parameters;
    _VisitIterativeMoverScalars(revision.op, parameters,
        [&](const char *name, auto fallback, auto &) {
            const TfToken token(name);
            _SampleMoverScalar(revision.moverPath.AppendProperty(token),
                revision.moverPrim.GetAttribute(token), fallback,
                time, refreshed, out);
        });
    if (revision.op == RigExecRevisionOp::Wrinkle) {
        const TfToken topology("inputs:topology");
        _SampleMoverScalar(revision.moverPath.AppendProperty(topology),
            revision.moverPrim.GetAttribute(topology), TfToken("cloth"),
            UsdTimeCode::Default(), refreshed, out);
        _SampleTopologyArray<int>(
            revision.moverPath.AppendProperty(TfToken("inputs:pinPoints")),
            UsdTimeCode::Default(), refreshed, stage, out);
    }
}

// Whether one attribute reads identically at every time code. USD answers
// conservatively (it may claim variance for a value that happens to be
// constant), so a false here means provably time-invariant -- the same
// classification the currency check uses for folded constants.
bool
_BurstAttributeIsStatic(const UsdAttribute &attribute)
{
    return attribute.IsValid() && !attribute.ValueMightBeTimeVarying();
}

// Serves one memoized static sample: the stored value, re-marked with the
// route whose map served it. VtValue copies share array payloads, so this
// is refcounts, not reads.
void
_ServeBurstStaticSample(RigExecFrameInputs *out,
                        const RigExecBurstStaticSample &entry, int route)
{
    RigExecSampledInput served = entry.sample;
    served.burstSampleRoute = route;
    out->values.push_back(served);
}

// Records the sample the plain reader just appended as a burst-static of
// \p route. Called only for attributes _BurstAttributeIsStatic accepts,
// whose value AND valuelessness are time-invariant -- and, for the
// resolved route, only with no chains bound, where the refreshed inputs
// are overrides-only and burst-fixed.
void
_MemoizeBurstStaticSample(
    RigExecFrameInputs *out,
    std::unordered_map<SdfPath, RigExecBurstStaticSample, SdfPath::Hash>
        *memo,
    int route)
{
    RigExecSampledInput &fresh = out->values.back();
    RigExecBurstStaticSample &entry = (*memo)[fresh.path];
    entry.sample = fresh;
    entry.sample.burstSampleRoute = route;
    entry.digestValid = false;
    fresh.burstSampleRoute = route;
}

// The cached read routes: consult the burst map for the route first, and
// on a miss read exactly as the plain reader does, then memoize when the
// attribute is time-invariant. Every early-out and every Add matches the
// plain reader's, so a cached frame's vector is elementwise identical to
// a plain sample's plus the route marks.
void
_SampleAttributeCached(const SdfPath &key, const UsdAttribute &attribute,
                       UsdTimeCode time, RigExecFrameInputs *out,
                       RigExecBurstSampleCache *cache)
{
    if (key.IsEmpty() || !attribute.IsValid()) {
        return;
    }
    const auto found = cache->staticStage.find(key);
    if (found != cache->staticStage.end()) {
        _ServeBurstStaticSample(out, found->second, RigExecBurstRouteStage);
        return;
    }
    _SampleAttribute(key, attribute, time, out);
    if (_BurstAttributeIsStatic(attribute)) {
        _MemoizeBurstStaticSample(out, &cache->staticStage,
                                  RigExecBurstRouteStage);
    }
}

void
_SampleQueryCached(const SdfPath &key, const UsdAttributeQuery &query,
                   UsdTimeCode time, RigExecFrameInputs *out,
                   RigExecBurstSampleCache *cache)
{
    if (key.IsEmpty() || !query.IsValid()) {
        return;
    }
    const auto found = cache->staticStage.find(key);
    if (found != cache->staticStage.end()) {
        _ServeBurstStaticSample(out, found->second, RigExecBurstRouteStage);
        return;
    }
    _SampleQuery(key, query, time, out);
    if (_BurstAttributeIsStatic(query.GetAttribute())) {
        _MemoizeBurstStaticSample(out, &cache->staticStage,
                                  RigExecBurstRouteStage);
    }
}

template <class T>
void
_SampleMoverScalarCached(const SdfPath &key, const UsdAttribute &attribute,
                         T fallback, UsdTimeCode time,
                         const RigExecResolvedInputs *refreshed,
                         RigExecFrameInputs *out,
                         RigExecBurstSampleCache *cache)
{
    if (!attribute.IsValid() || key.IsEmpty()) {
        return;
    }
    // With chains bound the refreshed read can carry per-frame chain
    // outputs: never cached, read plain every frame.
    if (!cache->bindings.chains.empty()) {
        _SampleMoverScalar(key, attribute, fallback, time, refreshed, out);
        return;
    }
    const auto found = cache->staticResolved.find(key);
    if (found != cache->staticResolved.end()) {
        _ServeBurstStaticSample(out, found->second,
                                RigExecBurstRouteResolved);
        return;
    }
    _SampleMoverScalar(key, attribute, fallback, time, refreshed, out);
    if (_BurstAttributeIsStatic(attribute)) {
        _MemoizeBurstStaticSample(out, &cache->staticResolved,
                                  RigExecBurstRouteResolved);
    }
}

void
_SampleBlendPointsCached(const SdfPath &key, const UsdAttribute &attribute,
                         UsdTimeCode time,
                         const RigExecResolvedInputs *refreshed,
                         RigExecFrameInputs *out,
                         RigExecBurstSampleCache *cache)
{
    if (!attribute.IsValid() || key.IsEmpty()) {
        return;
    }
    // With chains bound the refreshed read can carry per-frame chain
    // outputs: never cached, read plain every frame.
    if (!cache->bindings.chains.empty()) {
        _SampleBlendPoints(key, attribute, time, refreshed, out);
        return;
    }
    const auto found = cache->staticResolved.find(key);
    if (found != cache->staticResolved.end()) {
        _ServeBurstStaticSample(out, found->second,
                                RigExecBurstRouteResolved);
        return;
    }
    _SampleBlendPoints(key, attribute, time, refreshed, out);
    if (_BurstAttributeIsStatic(attribute)) {
        _MemoizeBurstStaticSample(out, &cache->staticResolved,
                                  RigExecBurstRouteResolved);
    }
}

// Samples one _Array arm (moverGraph.cpp) under an explicit key: the
// resolved memory first, else the stage at the job's time. A dangling path
// or a failed read samples valueless, which the worker answers as the
// empty array.
template <class T>
void
_SampleMoverPathArray(const SdfPath &key, const SdfPath &path,
                      UsdTimeCode time,
                      const RigExecResolvedInputs *refreshed,
                      const UsdStageRefPtr &stage, RigExecFrameInputs *out)
{
    if (key.IsEmpty() || path.IsEmpty()) {
        return;
    }
    VtArray<T> value;
    bool hasValue = refreshed && refreshed->Get(path, &value);
    if (!hasValue && stage) {
        if (const UsdAttribute a = stage->GetAttributeAtPath(path)) {
            hasValue = a.Get(&value, time);
        }
    }
    out->Add(key, VtValue(value), hasValue);
}

// Samples one _Array arm pinned at Default with no resolved arm (the
// lattice rest cage): a raw stage read at Default.
template <class T>
void
_SampleMoverPathArrayAtDefault(const SdfPath &key, const SdfPath &path,
                               const UsdStageRefPtr &stage,
                               RigExecFrameInputs *out)
{
    if (key.IsEmpty() || path.IsEmpty()) {
        return;
    }
    VtArray<T> value;
    bool hasValue = false;
    if (stage) {
        if (const UsdAttribute a = stage->GetAttributeAtPath(path)) {
            hasValue = a.Get(&value, UsdTimeCode::Default());
        }
    }
    out->Add(key, VtValue(value), hasValue);
}

// Samples the lattice divisions exactly as the arm reads them: raw off the
// mover prim at the evaluated time, no resolved arm. No sample when the
// attribute does not exist; the worker falls back to {0, 0, 0} the same
// way the packet's default init does.
void
_SampleLatticeDivisions(const SdfPath &key, const UsdAttribute &attribute,
                        UsdTimeCode time, RigExecFrameInputs *out)
{
    if (!attribute.IsValid() || key.IsEmpty()) {
        return;
    }
    GfVec3i value(0, 0, 0);
    const bool hasValue = attribute.Get(&value, time);
    out->Add(key, VtValue(value), hasValue);
}

template <class T>
void
_SampleTopologyArrayCached(const SdfPath &path, UsdTimeCode time,
                           const RigExecResolvedInputs *refreshed,
                           const UsdStageRefPtr &stage,
                           RigExecFrameInputs *out,
                           RigExecBurstSampleCache *cache)
{
    if (path.IsEmpty()) {
        return;
    }
    if (!cache->bindings.chains.empty()) {
        _SampleTopologyArray<T>(path, time, refreshed, stage, out);
        return;
    }
    const auto found = cache->staticResolved.find(path);
    if (found != cache->staticResolved.end()) {
        _ServeBurstStaticSample(out, found->second,
                                RigExecBurstRouteResolved);
        return;
    }
    _SampleTopologyArray<T>(path, time, refreshed, stage, out);
    // The plain read already resolved the attribute when it needed the
    // stage; re-resolving it here costs one lookup per path per burst, on
    // the miss only, and classifies the memory-hit case too.
    const UsdAttribute attribute =
        stage ? stage->GetAttributeAtPath(path) : UsdAttribute();
    if (_BurstAttributeIsStatic(attribute)) {
        _MemoizeBurstStaticSample(out, &cache->staticResolved,
                                  RigExecBurstRouteResolved);
    }
}

template <class T>
void
_SampleMoverPathArrayCached(const SdfPath &key, const SdfPath &path,
                            UsdTimeCode time,
                            const RigExecResolvedInputs *refreshed,
                            const UsdStageRefPtr &stage,
                            RigExecFrameInputs *out,
                            RigExecBurstSampleCache *cache)
{
    if (key.IsEmpty() || path.IsEmpty()) {
        return;
    }
    if (!cache->bindings.chains.empty()) {
        _SampleMoverPathArray<T>(key, path, time, refreshed, stage, out);
        return;
    }
    const auto found = cache->staticResolved.find(key);
    if (found != cache->staticResolved.end()) {
        _ServeBurstStaticSample(out, found->second,
                                RigExecBurstRouteResolved);
        return;
    }
    _SampleMoverPathArray<T>(key, path, time, refreshed, stage, out);
    const UsdAttribute attribute =
        stage ? stage->GetAttributeAtPath(path) : UsdAttribute();
    if (_BurstAttributeIsStatic(attribute)) {
        _MemoizeBurstStaticSample(out, &cache->staticResolved,
                                  RigExecBurstRouteResolved);
    }
}

template <class T>
void
_SampleMoverPathArrayAtDefaultCached(const SdfPath &key, const SdfPath &path,
                                     const UsdStageRefPtr &stage,
                                     RigExecFrameInputs *out,
                                     RigExecBurstSampleCache *cache)
{
    if (key.IsEmpty() || path.IsEmpty()) {
        return;
    }
    // Timeless by construction -- a Default read cannot vary per frame --
    // so the first read memoizes unconditionally, whatever USD reports
    // about the attribute's variance.
    const auto found = cache->staticStage.find(key);
    if (found != cache->staticStage.end()) {
        _ServeBurstStaticSample(out, found->second,
                                RigExecBurstRouteStage);
        return;
    }
    _SampleMoverPathArrayAtDefault<T>(key, path, stage, out);
    _MemoizeBurstStaticSample(out, &cache->staticStage,
                              RigExecBurstRouteStage);
}

void
_SampleLatticeDivisionsCached(const SdfPath &key,
                              const UsdAttribute &attribute,
                              UsdTimeCode time, RigExecFrameInputs *out,
                              RigExecBurstSampleCache *cache)
{
    if (key.IsEmpty() || !attribute.IsValid()) {
        return;
    }
    const auto found = cache->staticStage.find(key);
    if (found != cache->staticStage.end()) {
        _ServeBurstStaticSample(out, found->second,
                                RigExecBurstRouteStage);
        return;
    }
    _SampleLatticeDivisions(key, attribute, time, out);
    if (_BurstAttributeIsStatic(attribute)) {
        _MemoizeBurstStaticSample(out, &cache->staticStage,
                                  RigExecBurstRouteStage);
    }
}

} // namespace

namespace frozenDetail {

// Sample keys for the point arrays the packet build gathers. These are
// SYNTHETIC properties under the weight object or mover path -- never the
// attributes' own paths. The same mesh points may be sampled @time by one
// reader and @Default by another (wire rest vs posed curves), and sample
// lookup is first-wins, so sharing the attribute path would serve one
// reader's value to the other. The digest folds these keys like any other
// sample, which is what keeps distinct array states on distinct keys.
SdfPath
_FrozenWeightArrayKey(const SdfPath &objectPath, const char *role)
{
    return objectPath.AppendProperty(
        TfToken(std::string("frozenWeight:") + role));
}

SdfPath
_FrozenWireInputKey(const SdfPath &moverPath, const char *role)
{
    return moverPath.AppendProperty(
        TfToken(std::string("frozenWire:") + role));
}

// Synthetic key for one blend sample's dense target points, under the
// SAMPLE prim: the target path itself cannot serve, because a sample may
// target a chain's own base points and the two readers disagree -- the
// base loop samples the raw stage, the blend gather reads refreshed-first
// (an override standing on the mesh), and first-wins would serve one
// reader's value to the other. Same reason as the weight and wire keys.
SdfPath
_FrozenBlendInputKey(const SdfPath &samplePath, const char *role)
{
    return samplePath.AppendProperty(
        TfToken(std::string("frozenBlend:") + role));
}

// Replicates RigExecBakedProgram::SetOverrides (bakedProgram.cpp:1319)
// against the given epoch tables, writing the job's placement flags. The
// sampler uses it to read overridden bindings the long way; the worker uses
// it for the flags cone dirtiness reads. Returns false when an override is
// unplaceable -- live then runs dynamically, so a frozen job at these
// overrides must not exist.
bool
_FrozenPlaceOverrides(const RigExecBakedProgramImpl &B,
                      const std::vector<RigExecValueOverride> &overrides,
                      std::vector<char> *flags)
{
    flags->assign(B.overridden.size(), 0);
    bool placeable = true;
    for (const RigExecValueOverride &o : overrides) {
        if (o.attribute.IsEmpty()) {
            placeable = false;
            continue;
        }
        const SdfPath path = o.prim.AppendProperty(o.attribute);
        if (B.folded.count(path)) {
            placeable = false;
            continue;
        }
        const auto found = B.overridableInputs.find(path);
        if (found != B.overridableInputs.end()) {
            for (int index : found->second) {
                (*flags)[size_t(index)] = 1;
            }
            continue;
        }
        if (B.resolvedRoutedPrims.count(o.prim)) {
            continue;
        }
        placeable = false;
    }
    return placeable;
}

// Synthetic keys for mover side-input arrays whose read route differs from
// another reader of the same path: the surface driver's points (the base
// loop samples the raw stage where the arm reads refreshed-first) and the
// lattice cage's rest/live pair (one path, two times). Same reason as the
// weight, wire and blend keys.
SdfPath
_FrozenSurfaceInputKey(const SdfPath &moverPath, const char *role)
{
    return moverPath.AppendProperty(
        TfToken(std::string("frozenSurface:") + role));
}

SdfPath
_FrozenLatticeInputKey(const SdfPath &moverPath, const char *role)
{
    return moverPath.AppendProperty(
        TfToken(std::string("frozenLattice:") + role));
}

// Synthetic key for the ribbon bind coordinates: the same binding field
// the wire arm reads under its own synthetic key, and a revision is one
// op, so one key per read keeps the routes from ever aliasing.
SdfPath
_FrozenRibbonInputKey(const SdfPath &moverPath, const char *role)
{
    return moverPath.AppendProperty(
        TfToken(std::string("frozenRibbon:") + role));
}

// The chain-sampling hook (Increment B).
// Replicates the live property-chain prologue through public API only: chain
// discovery from the evaluator's mover order, _BindInput pinning, and the
// _EvaluatePropertyChains revision loop over the property-math kernels. The
// sampler runs it for the job's time on the UI thread, into caller-owned
// resolved inputs seeded with the job's overrides, and reads chain-resolved
// bindings through the refreshed values.
// ORDER SOUNDNESS. The hook evaluates in dependency order computed fresh at
// bind time. That order equals the live path's compile-time order within an
// epoch: every order-relevant edge -- a chain input's connection walk, a
// weight-object relationship -- is covered by the epoch digest
// (appendAttributeBinding over the value inputs plus the rel targets), so a
// rewire that could reorder evaluation recompiles first, and the session
// cache rebinds on the new epoch. What a value edit CAN move within an
// epoch -- a folded constant, a target's type -- is what
// RigExecChainSampleBindingsStillCurrent re-verifies at use. A mid-epoch
// rewire of a non-digested attribute (curve tangents, a custom note) either
// cannot be read through the resolved inputs at a compatible type, and so
// cannot move a value, or fails the bind as a cycle; either way the job
// declines rather than warming a misordered evaluation.
// What is NOT replicated is the live path's memoization (the watch/upstream
// dirty skip): it republishes identical values and lines, so recomputing
// every call changes no answer. Weight-object envelopes are declined, not
// replicated: they resolve through the evaluator's live oracle.

bool
_ChainIsFinite(float v)
{
    return std::isfinite(v);
}

} // namespace frozenDetail

namespace {

// The pinned sampler's body. `verifyCurrency` is the per-call
// RigExecChainSampleBindingsStillCurrent check: on for every caller that
// cannot otherwise prove its bindings fresh, off for one that tracks their
// currency from stage notices (see the trusted entry point below).
bool
_SampleWithPinnedChainBindings(
    const RigExecRigEvaluator &evaluator, UsdTimeCode time,
    const std::vector<RigExecValueOverride> &overrides,
    const RigExecChainSampleBindings &bindings, RigExecFrameInputs *out,
    std::string *error, bool verifyCurrency)
{
    const auto fail = [&error](const std::string &why) {
        if (error) {
            *error = why;
        }
        return false;
    };
    if (!out) {
        return fail("no input vector to sample into");
    }
    // The pinned route trusts nothing: stale bindings fail the sample and
    // the caller rebinds. An empty pin on a chained rig fails here too, so
    // the hook cannot be skipped around -- only the bind, which names the
    // rig's actual chains, feeds this route.
    if (verifyCurrency &&
        !RigExecChainSampleBindingsStillCurrent(bindings, evaluator)) {
        return fail(
            "chain bindings no longer name the evaluator's chains; rebind "
            "and retry");
    }
    const RigExecBakedProgram *program = evaluator.GetBakedProgram();
    if (!program) {
        // D7, sampler half: a rig with no program is evaluated dynamically
        // on the UI thread, and its results memoize through the same publish
        // path -- but there is nothing to sample FOR, so no background job.
        return fail("no baked program: dynamic/refusal rigs take the D7 UI-"
                    "thread memo path, never a background job");
    }
    const RigExecBakedProgramImpl &B = program->GetStepGraph();
    // No shape gate on xform slots, native sources, or delta bases: the
    // seeds sample per frame below, through the program's hook.

    // Override placement decides which bindings read the long way. When an
    // override is unplaceable live runs dynamically, so no frozen job at
    // these overrides may exist.
    std::vector<char> overrideFlags;
    if (!_FrozenPlaceOverrides(B, overrides, &overrideFlags)) {
        return fail("an override is unplaceable: live runs dynamically at "
                    "these overrides, which no frozen job can reproduce");
    }
    // The refreshed inputs: the job's overrides placed over an empty map,
    // the hook's chain results at the job's time, stage reads at the job's
    // time. Overridden and chain-resolved bindings and mover scalars sample
    // through these (fresh at the job's time by construction); the standing
    // inputs below serve only a declined hook's fallback, which marks stale.
    RigExecResolvedInputs refreshed;
    _PlaceOverridesIntoResolved(overrides, &refreshed);

    RigExecFrameInputs sampled;
    sampled.time = time;
    sampled.overrides = overrides;
    const RigExecResolvedInputs *resolved = B.resolvedInputs;

    // The chain-sampling hook, in the live prologue's own order: the
    // chains evaluate into the refreshed inputs over the already-placed
    // overrides, a drag on a chain target being that chain's base, and the
    // per-target results travel with the vector for the frozen prologue to
    // publish as its property results. On decline (a weight object) the
    // refreshed inputs stay override-only and chain-resolved bindings mark
    // viaChain below, declining the vector downstream.
    const RigExecResolvedInputs *chainFresh = nullptr;
    if (!bindings.chains.empty()) {
        RigExecResolvedInputs hooked = refreshed;
        std::map<SdfPath, VtValue> results;
        std::vector<std::string> hookDiagnostics;
        std::string hookError;
        if (RigExecEvaluateChainsForTime(bindings, time, &hooked, &results,
                                         &hookDiagnostics, &hookError)) {
            refreshed = std::move(hooked);
            sampled.chainResults = std::move(results);
            sampled.chainDiagnostics = std::move(hookDiagnostics);
            chainFresh = &refreshed;
        }
    }

    for (const RigExecBakedProgramImpl::AvarBinding &binding :
         B.avarBindings) {
        _SampleFlaggedBinding(binding.input, resolved, &refreshed,
                              overrideFlags, time, &sampled, chainFresh);
    }
    for (size_t promoted : B.promotedAvars) {
        if (promoted < B.avarConstantBindings.size()) {
            // Refreshed, not standing: a promoted avar's walk is chainless,
            // so the refreshed walk (job overrides, stage at the job's
            // time) is exactly what live reads, while the standing inputs
            // may hold another frame's overrides.
            _SamplePromotedAvar(B.avarConstantBindings[promoted].input,
                                &refreshed, time, &sampled);
        }
    }
    for (const RigExecBakedProgramImpl::Ladder &ladder : B.ladders) {
        _VisitLadderInputs(ladder, [&](const auto &input) {
            _SampleFlaggedBinding(input, resolved, &refreshed, overrideFlags,
                                  time, &sampled, chainFresh);
        });
    }
    for (const RigExecBakedProgramImpl::SpaceSwitch &spaceSwitch :
         B.spaceSwitches) {
        _VisitSpaceSwitchInputs(spaceSwitch, [&](const auto &input) {
            _SampleFlaggedBinding(input, resolved, &refreshed, overrideFlags,
                                  time, &sampled, chainFresh);
        });
    }
    for (const RigExecBakedProgramImpl::PoseInterpolator &interp :
         B.poseInterpolators) {
        _VisitInterpolatorInputs(interp, [&](const auto &input) {
            _SampleFlaggedBinding(input, resolved, &refreshed, overrideFlags,
                                  time, &sampled, chainFresh);
        });
    }
    _VisitComposeInputs(B, [&](const auto &input) {
        _SampleFlaggedBinding(input, resolved, &refreshed, overrideFlags,
                              time, &sampled, chainFresh);
    });
    for (const RigExecBakedProgramImpl::Solver &solver : B.solvers) {
        _SampleSolverBindings(solver, resolved, &refreshed, overrideFlags,
                              time, &sampled, chainFresh);
    }
    for (const RigExecBakedProgramImpl::Constraint &constraint :
         B.constraints) {
        _SampleConstraintBindings(constraint, resolved, &refreshed,
                                  overrideFlags, time, &sampled, chainFresh);
    }
    for (const RigExecBakedProgramImpl::WeightObject &object :
         B.weightObjects) {
        _SampleWeightBindings(object, resolved, &refreshed, overrideFlags,
                              time, &sampled, chainFresh);
        _SampleWeightArrays(object, &refreshed, time, &sampled);
    }
    // Ribbon driver points: read straight off the stage by the prologue,
    // honouring neither connections nor the resolved inputs, so sampled the
    // same way.
    for (const RigExecBakedProgramImpl::Solver &solver : B.solvers) {
        if (solver.ribbonPointsVarying) {
            _SampleQuery(solver.ribbonPointsPath, solver.ribbonPointsQuery,
                         time, &sampled);
        }
    }
    // Blend channels outside the binding table, refreshed-first like the
    // mover scalars: an override or chain output standing on a weight,
    // activation or target array is what R.GetAttribute answers live. The
    // fallbacks are the gather's inits (bakedGeometry.cpp AssembleRevision):
    // a weight the read misses is 0, an activation 1, points empty. Dense
    // points ride a synthetic key under the sample prim, never the target
    // path (see _FrozenBlendInputKey).
    // A pose-driven weight samples only when the refreshed inputs hold its
    // path -- an override or chain result standing on it, which is exactly
    // when live reads R instead of the pose slot. Otherwise the worker
    // takes the slot from its own pose run, and no sample shadows it.
    for (const RigExecBakedProgramImpl::GeomChain &chain : B.chains) {
        for (const RigExecBakedProgramImpl::GeomRevision &revision :
             chain.revisions) {
            for (const RigExecBakedProgramImpl::GeomBlendChannel &channel :
                 revision.blendChannels) {
                if (channel.weight.IsValid() &&
                    (channel.poseWeight < 0 ||
                     refreshed.Find(channel.weightPath))) {
                    _SampleMoverScalar(channel.weight.GetPath(),
                                       channel.weight, 0.0f, time, &refreshed,
                                       &sampled);
                }
                for (const RigExecBakedProgramImpl::GeomBlendChannel::Sample
                         &sample : channel.samples) {
                    _SampleMoverScalar(sample.activation.GetPath(),
                                       sample.activation, 1.0f, time,
                                       &refreshed, &sampled);
                    _SampleBlendPoints(
                        _FrozenBlendInputKey(sample.samplePath, "points"),
                        sample.points, time, &refreshed, &sampled);
                }
            }
        }
    }
    // Constraint operator arrays: read raw off the prim per frame.
    for (const RigExecBakedProgramImpl::ConstraintArrays &arrays :
         B.constraintArrays) {
        if (!arrays.prim.IsValid()) {
            continue;
        }
        const SdfPath primPath = arrays.prim.GetPath();
        _SampleAttribute(primPath.AppendProperty(TfToken(
                             "inputs:sourceWeights")),
                         arrays.prim.GetAttribute(
                             TfToken("inputs:sourceWeights")),
                         time, &sampled);
        if (arrays.parentOffsets) {
            _SampleAttribute(primPath.AppendProperty(TfToken(
                                 "inputs:translationOffsets")),
                             arrays.prim.GetAttribute(TfToken(
                                 "inputs:translationOffsets")),
                             time, &sampled);
            _SampleAttribute(primPath.AppendProperty(TfToken(
                                 "inputs:rotationOffsets")),
                             arrays.prim.GetAttribute(TfToken(
                                 "inputs:rotationOffsets")),
                             time, &sampled);
        }
        if (arrays.readPole) {
            _SampleAttribute(primPath.AppendProperty(TfToken(
                                 "inputs:poleVectorWeights")),
                             arrays.prim.GetAttribute(TfToken(
                                 "inputs:poleVectorWeights")),
                             time, &sampled);
        }
    }
    // Chain base points: the prologue reads every chain base off the stage
    // per frame, and a worker cannot -- so the UI thread samples these too.
    for (const RigExecBakedProgramImpl::GeomChain &chain : B.chains) {
        _SampleQuery(chain.target, chain.baseQuery, time, &sampled);
        for (const RigExecBakedProgramImpl::GeomChain::Derived &derived :
             chain.derived) {
            _SampleQuery(derived.target, derived.baseQuery, time, &sampled);
        }
    }
    // Mover scalars the packet assembly reads per frame (enabled,
    // defaultWeight, skinningMethod), keyed by mover path, plus the derived
    // topology arrays (counts, indices, extent widths), keyed by binding
    // path. The worker replays these; it cannot read the mover prim.
    for (const auto &[chainIndex, revisionIndex] : B.revisionIndex) {
        const RigExecBakedProgramImpl::GeomRevision &revision =
            B.chains[size_t(chainIndex)].revisions[size_t(revisionIndex)];
        const UsdPrim &moverPrim = revision.moverPrim;
        _SampleMoverScalar(
            revision.moverPath.AppendProperty(TfToken("inputs:enabled")),
            moverPrim.GetAttribute(TfToken("inputs:enabled")), true, time,
            &refreshed, &sampled);
        _SampleMoverScalar(
            revision.moverPath.AppendProperty(TfToken("inputs:defaultWeight")),
            moverPrim.GetAttribute(TfToken("inputs:defaultWeight")), 1.0f,
            time, &refreshed, &sampled);
        _SampleMoverScalar(
            revision.moverPath.AppendProperty(TfToken("rigExec:skinningMethod")),
            moverPrim.GetAttribute(TfToken("rigExec:skinningMethod")),
            TfToken("classicLinear"), time, &refreshed, &sampled);
        if (revision.op == RigExecRevisionOp::Wire) {
            _SampleWireInputs(revision, &refreshed, B.stage, time, &sampled);
        }
        if (revision.op == RigExecRevisionOp::Matrix) {
            _SampleMoverToken(moverPrim, revision.moverPath,
                              "rigExec:weightBlend", time, &sampled);
        }
        _SampleIterativeMoverInputs(revision, &refreshed, B.stage, time,
                                    &sampled);
        if (revision.op == RigExecRevisionOp::BlendShape && moverPrim) {
            // rigExec:deltaSpace, exactly as the arm's _Token reads it: a
            // raw Default read with no resolved arm, "target" when absent.
            // Read once here; the value both samples and decides whether
            // the surface-frame topology below is read at all, as live.
            TfToken deltaSpace("target");
            const SdfPath deltaSpacePath = revision.moverPath.AppendProperty(
                TfToken("rigExec:deltaSpace"));
            if (const UsdAttribute deltaSpaceAttr =
                    moverPrim.GetAttribute(TfToken("rigExec:deltaSpace"))) {
                TfToken read;
                if (deltaSpaceAttr.Get(&read, UsdTimeCode::Default())) {
                    deltaSpace = read;
                }
                sampled.Add(deltaSpacePath, VtValue(deltaSpace),
                            /*hasValue=*/true);
            }
            if (deltaSpace == "surfaceFrame") {
                _SampleTopologyArray<int>(revision.binding.topologyCounts,
                                          time, &refreshed, B.stage, &sampled);
                _SampleTopologyArray<int>(revision.binding.topologyIndices,
                                          time, &refreshed, B.stage, &sampled);
            }
        }
        if (moverPrim &&
            (revision.op == RigExecRevisionOp::Smooth ||
             revision.op == RigExecRevisionOp::SurfaceProject)) {
            // The arms' _Array reads at the evaluated time, resolved-first;
            // binding paths are safe keys because every reader of a
            // topology path samples this same route.
            _SampleTopologyArray<int>(revision.binding.topologyCounts, time,
                                      &refreshed, B.stage, &sampled);
            _SampleTopologyArray<int>(revision.binding.topologyIndices, time,
                                      &refreshed, B.stage, &sampled);
        }
        if (revision.op == RigExecRevisionOp::SurfaceProject && moverPrim) {
            _SampleMoverPathArray<GfVec3f>(
                _FrozenSurfaceInputKey(revision.moverPath, "surfacePoints"),
                revision.binding.surfacePoints, time, &refreshed, B.stage,
                &sampled);
        }
        if (revision.op == RigExecRevisionOp::Lattice && moverPrim) {
            // The rest/live cage pair, under distinct synthetic keys: one
            // path, two times (Default raw, evaluated resolved-first).
            _SampleMoverPathArrayAtDefault<GfVec3f>(
                _FrozenLatticeInputKey(revision.moverPath, "cageRest"),
                revision.binding.cagePoints, B.stage, &sampled);
            _SampleMoverPathArray<GfVec3f>(
                _FrozenLatticeInputKey(revision.moverPath, "cageLive"),
                revision.binding.cagePoints, time, &refreshed, B.stage,
                &sampled);
            _SampleLatticeDivisions(
                revision.moverPath.AppendProperty(
                    TfToken("rigExec:divisions")),
                moverPrim.GetAttribute(TfToken("rigExec:divisions")), time,
                &sampled);
        }
        if (revision.op == RigExecRevisionOp::Ribbon && moverPrim) {
            // The bind coordinates at the evaluated time, resolved-first.
            // The driver's frames need no sampling: the solver's Solve
            // step runs on the worker from already-sampled inputs, and
            // the assembly reads the worker's own aggregate.
            _SampleMoverPathArray<GfVec2f>(
                _FrozenRibbonInputKey(revision.moverPath, "bindCoords"),
                revision.binding.bindCoords, time, &refreshed, B.stage,
                &sampled);
        }
    }
    for (const auto &[chainIndex, derivedIndex] : B.derivedIndex) {
        const RigExecBakedProgramImpl::GeomChain::Derived &derived =
            B.chains[size_t(chainIndex)].derived[size_t(derivedIndex)];
        const RigExecBakedProgramImpl::GeomRevision &revision =
            derived.revision;
        _SampleTopologyArray<int>(revision.binding.topologyCounts, time,
                                  &refreshed, B.stage, &sampled);
        _SampleTopologyArray<int>(revision.binding.topologyIndices, time,
                                  &refreshed, B.stage, &sampled);
        if (revision.op == RigExecRevisionOp::RecomputeExtent) {
            _SampleTopologyArray<float>(revision.binding.widths, time,
                                        &refreshed, B.stage, &sampled);
        }
        if (RigExecIsDerivedMatrixOp(revision.op)) {
            _SampleProjectorInputs(revision, &refreshed, B.stage, time,
                                   &sampled);
        }
    }
    // Skin packets, assembled here by the real assembler: the worker cannot
    // call it (mover prim and live topology cache), so the packet travels
    // with the job. Parallel to revisionIndex; non-skin revisions keep a
    // default packet the worker ignores (it assembles derived packets
    // itself from the region's points plus the sampled topology above).
    sampled.revisionPackets.resize(B.revisionIndex.size());
    for (size_t r = 0; r < B.revisionIndex.size(); ++r) {
        const auto &[chainIndex, revisionIndex] = B.revisionIndex[r];
        const RigExecBakedProgramImpl::GeomRevision &revision =
            B.chains[size_t(chainIndex)].revisions[size_t(revisionIndex)];
        if (revision.op != RigExecRevisionOp::Skin) {
            continue;
        }
        // The prologue's resolveTopology, through the refreshed inputs: the
        // same cache, the same call, the same pointer for an unmoved layout.
        // The hook's chain results are in there too, so a chain-driven
        // mover input resolves at the job's time, as on the live path.
        std::shared_ptr<const RigExecSkinTopology> topology;
        if (revision.skinTopologyFixed) {
            topology = RigExecResolveSkinTopology(
                revision.moverPrim, revision.influenceSlots.size(), time,
                &refreshed, B.skinTopologies);
        }
        sampled.revisionPackets[r] = RigExecAssembleSkinParameters(
            revision.moverPrim, &revision.packetInfluences,
            /*weights=*/nullptr, time, &refreshed, B.skinTopologies,
            topology ? &topology : nullptr);
    }
    {
        std::string layoutError;
        if (!_SampleBlendLayouts(B, time, &sampled, &layoutError)) {
            return fail(layoutError);
        }
    }
    // Stage-frame seeds at the job's time, through the program's hook. A
    // decline names an unresolvable target, at which live gives the
    // generation back -- so the frame has no frozen job.
    {
        std::string seedsError;
        if (!_SampleStageFrameSeeds(*program, time, &sampled, &seedsError)) {
            return fail(seedsError);
        }
    }
    // Interactive overrides never reach the stage, so they are sampled from
    // the caller's list rather than read. Keyed by the property they stand
    // on; a computation override -- which names no attribute -- is keyed by
    // a synthetic property under its prim, so two computations on one prim
    // still hash apart.
    for (const RigExecValueOverride &o : overrides) {
        SdfPath key = o.prim;
        if (!o.attribute.IsEmpty()) {
            key = key.AppendProperty(o.attribute);
        } else if (!o.computation.IsEmpty()) {
            key = key.AppendProperty(TfToken(
                "rigExec:override:" + o.computation.GetString()));
        }
        if (!key.IsEmpty()) {
            sampled.Add(key, o.value, /*hasValue=*/true);
        }
    }

    *out = sampled;
    return true;
}
} // namespace

bool
RigExecSampleFrameInputsWithChainBindings(
    const RigExecRigEvaluator &evaluator, UsdTimeCode time,
    const std::vector<RigExecValueOverride> &overrides,
    const RigExecChainSampleBindings &bindings, RigExecFrameInputs *out,
    std::string *error)
{
    return _SampleWithPinnedChainBindings(evaluator, time, overrides,
                                          bindings, out, error,
                                          /* verifyCurrency = */ true);
}

bool
RigExecSampleFrameInputsWithTrustedChainBindings(
    const RigExecRigEvaluator &evaluator, UsdTimeCode time,
    const std::vector<RigExecValueOverride> &overrides,
    const RigExecChainSampleBindings &bindings, RigExecFrameInputs *out,
    std::string *error)
{
    return _SampleWithPinnedChainBindings(evaluator, time, overrides,
                                          bindings, out, error,
                                          /* verifyCurrency = */ false);
}

void
RigExecBurstSampleCache::Clear()
{
    program = nullptr;
    epochDigest = 0;
    bindings = RigExecChainSampleBindings();
    overrides.clear();
    overrideFlags.clear();
    placeable = false;
    usable = false;
    ladderSites.clear();
    spaceSwitchSites.clear();
    solverSites.clear();
    constraintSites.clear();
    weightSites.clear();
    interpolatorSites.clear();
    staticStage.clear();
    staticResolved.clear();
    sortedOrder.clear();
    orderValuesSize = 0;
    orderFirstPath = SdfPath();
    orderLastPath = SdfPath();
}

bool
RigExecBuildBurstSampleCache(
    const RigExecBakedProgram &program,
    const RigExecChainSampleBindings &bindings,
    const std::vector<RigExecValueOverride> &overrides, uint64_t epochDigest,
    RigExecBurstSampleCache *cache, std::string *error)
{
    const auto fail = [&error, cache](const std::string &why) {
        if (cache) {
            cache->usable = false;
        }
        if (error) {
            *error = why;
        }
        return false;
    };
    if (!cache) {
        return fail("no burst cache to build into");
    }
    cache->Clear();
    // The decline checks mirror the sampler's below, with its messages: a
    // rig the sampler would decline per frame fails the build once, and
    // the burst skips every frame up front instead of declining each.
    const RigExecBakedProgramImpl &B = program.GetStepGraph();
    // No shape gate on xform slots, native sources, or delta bases: the
    // seeds sample per frame through the program's hook, as in the plain
    // sampler, so no frame of the burst is unshaped for them.
    if (!_FrozenPlaceOverrides(B, overrides, &cache->overrideFlags)) {
        return fail("an override is unplaceable: live runs dynamically at "
                    "these overrides, which no frozen job can reproduce");
    }
    cache->program = &program;
    cache->epochDigest = epochDigest;
    cache->bindings = bindings;
    cache->overrides = overrides;
    cache->placeable = true;
    // In table order, so the cached sampler visits in emission order.
    for (size_t i = 0; i < B.ladders.size(); ++i) {
        if (_BurstLadderNeedsVisit(B.ladders[i], cache->overrideFlags)) {
            cache->ladderSites.push_back(i);
        }
    }
    for (size_t i = 0; i < B.spaceSwitches.size(); ++i) {
        if (_BurstSpaceSwitchNeedsVisit(B.spaceSwitches[i],
                                        cache->overrideFlags)) {
            cache->spaceSwitchSites.push_back(i);
        }
    }
    for (size_t i = 0; i < B.solvers.size(); ++i) {
        if (_BurstSolverNeedsVisit(B.solvers[i], cache->overrideFlags)) {
            cache->solverSites.push_back(i);
        }
    }
    for (size_t i = 0; i < B.constraints.size(); ++i) {
        if (_BurstConstraintNeedsVisit(B.constraints[i],
                                       cache->overrideFlags)) {
            cache->constraintSites.push_back(i);
        }
    }
    for (size_t i = 0; i < B.weightObjects.size(); ++i) {
        if (_BurstWeightNeedsVisit(B.weightObjects[i],
                                   cache->overrideFlags)) {
            cache->weightSites.push_back(i);
        }
    }
    for (size_t i = 0; i < B.poseInterpolators.size(); ++i) {
        if (_BurstInterpolatorNeedsVisit(B.poseInterpolators[i],
                                         cache->overrideFlags)) {
            cache->interpolatorSites.push_back(i);
        }
    }
    cache->usable = true;
    return true;
}

bool
RigExecSampleFrameInputsWithBurstCache(
    const RigExecRigEvaluator &evaluator, UsdTimeCode time,
    const std::vector<RigExecValueOverride> &overrides,
    RigExecBurstSampleCache *cache, RigExecFrameInputs *out,
    std::string *error)
{
    const auto fail = [&error](const std::string &why) {
        if (error) {
            *error = why;
        }
        return false;
    };
    if (!out) {
        return fail("no input vector to sample into");
    }
    // The cache is burst-scoped: a foreign program or a differing override
    // list fails loud rather than sampling through prepared state that no
    // longer describes the call. A burst holds both fixed (the registry
    // captures the same vector for every frame), so production never lands
    // here; the plain sampler stays the answer for anything else.
    if (!cache || !cache->usable) {
        return fail("burst cache is not usable; re-prepare");
    }
    if (evaluator.GetBakedProgram() != cache->program) {
        return fail("burst cache names another program; re-prepare");
    }
    if (overrides != cache->overrides) {
        return fail("burst cache was prepared for other overrides; "
                    "re-prepare");
    }
    const RigExecBakedProgramImpl &B = cache->program->GetStepGraph();
    // No currency re-verification (verified at prepare), no decline checks
    // (the build failed on them), no override placement (cached flags).
    // Everything below mirrors the plain sampler above: the same tables,
    // in the same order, through the same routes -- struct tables through
    // the prepared site lists, stage-direct and resolved reads through the
    // static maps -- so the vector is elementwise identical plus the route
    // marks.
    RigExecResolvedInputs refreshed;
    _PlaceOverridesIntoResolved(overrides, &refreshed);

    RigExecFrameInputs sampled;
    sampled.time = time;
    sampled.overrides = overrides;
    const RigExecResolvedInputs *resolved = B.resolvedInputs;

    const RigExecResolvedInputs *chainFresh = nullptr;
    if (!cache->bindings.chains.empty()) {
        RigExecResolvedInputs hooked = refreshed;
        std::map<SdfPath, VtValue> results;
        std::vector<std::string> hookDiagnostics;
        std::string hookError;
        if (RigExecEvaluateChainsForTime(cache->bindings, time, &hooked,
                                         &results, &hookDiagnostics,
                                         &hookError)) {
            refreshed = std::move(hooked);
            sampled.chainResults = std::move(results);
            sampled.chainDiagnostics = std::move(hookDiagnostics);
            chainFresh = &refreshed;
        }
    }

    for (const RigExecBakedProgramImpl::AvarBinding &binding :
         B.avarBindings) {
        _SampleFlaggedBinding(binding.input, resolved, &refreshed,
                              cache->overrideFlags, time, &sampled,
                              chainFresh);
    }
    for (size_t promoted : B.promotedAvars) {
        if (promoted < B.avarConstantBindings.size()) {
            _SamplePromotedAvar(B.avarConstantBindings[promoted].input,
                                &refreshed, time, &sampled);
        }
    }
    for (size_t i : cache->ladderSites) {
        _VisitLadderInputs(B.ladders[i], [&](const auto &input) {
            _SampleFlaggedBinding(input, resolved, &refreshed,
                                  cache->overrideFlags, time, &sampled,
                                  chainFresh);
        });
    }
    for (size_t i : cache->spaceSwitchSites) {
        _VisitSpaceSwitchInputs(B.spaceSwitches[i], [&](const auto &input) {
            _SampleFlaggedBinding(input, resolved, &refreshed,
                                  cache->overrideFlags, time, &sampled,
                                  chainFresh);
        });
    }
    for (size_t i : cache->interpolatorSites) {
        _VisitInterpolatorInputs(B.poseInterpolators[i],
                                 [&](const auto &input) {
            _SampleFlaggedBinding(input, resolved, &refreshed,
                                  cache->overrideFlags, time, &sampled,
                                  chainFresh);
        });
    }
    _VisitComposeInputs(B, [&](const auto &input) {
        _SampleFlaggedBinding(input, resolved, &refreshed,
                              cache->overrideFlags, time, &sampled,
                              chainFresh);
    });
    for (size_t i : cache->solverSites) {
        _SampleSolverBindings(B.solvers[i], resolved, &refreshed,
                              cache->overrideFlags, time, &sampled,
                              chainFresh);
    }
    for (size_t i : cache->constraintSites) {
        _SampleConstraintBindings(B.constraints[i], resolved, &refreshed,
                                  cache->overrideFlags, time, &sampled,
                                  chainFresh);
    }
    // Per object, bindings then arrays, exactly as the plain sampler
    // emits them: the sites list is table-ordered, so a merge walk visits
    // the same objects in the same positions. (A bindings loop followed by
    // an arrays loop emits the same SET in a different ORDER whenever the
    // table holds more than one visiting object, which the elementwise
    // contract forbids even though the digest and the worker's first-wins
    // reads are order-insensitive.)
    size_t weightSite = 0;
    for (size_t i = 0; i < B.weightObjects.size(); ++i) {
        if (weightSite < cache->weightSites.size() &&
            cache->weightSites[weightSite] == i) {
            _SampleWeightBindings(B.weightObjects[i], resolved, &refreshed,
                                  cache->overrideFlags, time, &sampled,
                                  chainFresh);
            ++weightSite;
        }
        // Point arrays ride outside the burst memo (always fresh, like the
        // ribbon points below): correctness first, memoization later.
        _SampleWeightArrays(B.weightObjects[i], &refreshed, time, &sampled);
    }
    for (const RigExecBakedProgramImpl::Solver &solver : B.solvers) {
        if (solver.ribbonPointsVarying) {
            _SampleQuery(solver.ribbonPointsPath, solver.ribbonPointsQuery,
                         time, &sampled);
        }
    }
    // Blend channels, as the slow sampler reads them: refreshed-first
    // scalars with the gather's fallback inits, dense points under the
    // sample-prim synthetic key, and a pose-driven weight only when the
    // refreshed inputs hold its path.
    for (const RigExecBakedProgramImpl::GeomChain &chain : B.chains) {
        for (const RigExecBakedProgramImpl::GeomRevision &revision :
             chain.revisions) {
            for (const RigExecBakedProgramImpl::GeomBlendChannel &channel :
                 revision.blendChannels) {
                if (channel.weight.IsValid() &&
                    (channel.poseWeight < 0 ||
                     refreshed.Find(channel.weightPath))) {
                    _SampleMoverScalarCached(
                        channel.weight.GetPath(), channel.weight, 0.0f, time,
                        &refreshed, &sampled, cache);
                }
                for (const RigExecBakedProgramImpl::GeomBlendChannel::Sample
                         &sample : channel.samples) {
                    _SampleMoverScalarCached(
                        sample.activation.GetPath(), sample.activation, 1.0f,
                        time, &refreshed, &sampled, cache);
                    _SampleBlendPointsCached(
                        _FrozenBlendInputKey(sample.samplePath, "points"),
                        sample.points, time, &refreshed, &sampled, cache);
                }
            }
        }
    }
    for (const RigExecBakedProgramImpl::ConstraintArrays &arrays :
         B.constraintArrays) {
        if (!arrays.prim.IsValid()) {
            continue;
        }
        const SdfPath primPath = arrays.prim.GetPath();
        _SampleAttributeCached(primPath.AppendProperty(TfToken(
                                   "inputs:sourceWeights")),
                               arrays.prim.GetAttribute(
                                   TfToken("inputs:sourceWeights")),
                               time, &sampled, cache);
        if (arrays.parentOffsets) {
            _SampleAttributeCached(primPath.AppendProperty(TfToken(
                                       "inputs:translationOffsets")),
                                   arrays.prim.GetAttribute(TfToken(
                                       "inputs:translationOffsets")),
                                   time, &sampled, cache);
            _SampleAttributeCached(primPath.AppendProperty(TfToken(
                                       "inputs:rotationOffsets")),
                                   arrays.prim.GetAttribute(TfToken(
                                       "inputs:rotationOffsets")),
                                   time, &sampled, cache);
        }
        if (arrays.readPole) {
            _SampleAttributeCached(primPath.AppendProperty(TfToken(
                                       "inputs:poleVectorWeights")),
                                   arrays.prim.GetAttribute(TfToken(
                                       "inputs:poleVectorWeights")),
                                   time, &sampled, cache);
        }
    }
    for (const RigExecBakedProgramImpl::GeomChain &chain : B.chains) {
        _SampleQueryCached(chain.target, chain.baseQuery, time, &sampled,
                           cache);
        for (const RigExecBakedProgramImpl::GeomChain::Derived &derived :
             chain.derived) {
            _SampleQueryCached(derived.target, derived.baseQuery, time,
                               &sampled, cache);
        }
    }
    for (const auto &[chainIndex, revisionIndex] : B.revisionIndex) {
        const RigExecBakedProgramImpl::GeomRevision &revision =
            B.chains[size_t(chainIndex)].revisions[size_t(revisionIndex)];
        const UsdPrim &moverPrim = revision.moverPrim;
        _SampleMoverScalarCached(
            revision.moverPath.AppendProperty(TfToken("inputs:enabled")),
            moverPrim.GetAttribute(TfToken("inputs:enabled")), true, time,
            &refreshed, &sampled, cache);
        _SampleMoverScalarCached(
            revision.moverPath.AppendProperty(TfToken("inputs:defaultWeight")),
            moverPrim.GetAttribute(TfToken("inputs:defaultWeight")), 1.0f,
            time, &refreshed, &sampled, cache);
        _SampleMoverScalarCached(
            revision.moverPath.AppendProperty(TfToken("rigExec:skinningMethod")),
            moverPrim.GetAttribute(TfToken("rigExec:skinningMethod")),
            TfToken("classicLinear"), time, &refreshed, &sampled, cache);
        if (revision.op == RigExecRevisionOp::Wire) {
            _SampleWireInputs(revision, &refreshed, B.stage, time, &sampled);
        }
        if (revision.op == RigExecRevisionOp::Matrix) {
            _SampleMoverToken(moverPrim, revision.moverPath,
                              "rigExec:weightBlend", time, &sampled);
        }
        _SampleIterativeMoverInputs(revision, &refreshed, B.stage, time,
                                    &sampled);
        if (revision.op == RigExecRevisionOp::BlendShape && moverPrim) {
            // rigExec:deltaSpace, as the slow sampler reads it: raw Default,
            // no resolved arm. Unmemoized -- one token read per frame is
            // nothing, and the value doubles as the topology condition.
            TfToken deltaSpace("target");
            const SdfPath deltaSpacePath = revision.moverPath.AppendProperty(
                TfToken("rigExec:deltaSpace"));
            if (const UsdAttribute deltaSpaceAttr =
                    moverPrim.GetAttribute(TfToken("rigExec:deltaSpace"))) {
                TfToken read;
                if (deltaSpaceAttr.Get(&read, UsdTimeCode::Default())) {
                    deltaSpace = read;
                }
                sampled.Add(deltaSpacePath, VtValue(deltaSpace),
                            /*hasValue=*/true);
            }
            if (deltaSpace == "surfaceFrame") {
                _SampleTopologyArrayCached<int>(
                    revision.binding.topologyCounts, time, &refreshed, B.stage,
                    &sampled, cache);
                _SampleTopologyArrayCached<int>(
                    revision.binding.topologyIndices, time, &refreshed, B.stage,
                    &sampled, cache);
            }
        }
        if (moverPrim &&
            (revision.op == RigExecRevisionOp::Smooth ||
             revision.op == RigExecRevisionOp::SurfaceProject)) {
            _SampleTopologyArrayCached<int>(
                revision.binding.topologyCounts, time, &refreshed, B.stage,
                &sampled, cache);
            _SampleTopologyArrayCached<int>(
                revision.binding.topologyIndices, time, &refreshed, B.stage,
                &sampled, cache);
        }
        if (revision.op == RigExecRevisionOp::SurfaceProject && moverPrim) {
            _SampleMoverPathArrayCached<GfVec3f>(
                _FrozenSurfaceInputKey(revision.moverPath, "surfacePoints"),
                revision.binding.surfacePoints, time, &refreshed, B.stage,
                &sampled, cache);
        }
        if (revision.op == RigExecRevisionOp::Lattice && moverPrim) {
            _SampleMoverPathArrayAtDefaultCached<GfVec3f>(
                _FrozenLatticeInputKey(revision.moverPath, "cageRest"),
                revision.binding.cagePoints, B.stage, &sampled, cache);
            _SampleMoverPathArrayCached<GfVec3f>(
                _FrozenLatticeInputKey(revision.moverPath, "cageLive"),
                revision.binding.cagePoints, time, &refreshed, B.stage,
                &sampled, cache);
            _SampleLatticeDivisionsCached(
                revision.moverPath.AppendProperty(
                    TfToken("rigExec:divisions")),
                moverPrim.GetAttribute(TfToken("rigExec:divisions")), time,
                &sampled, cache);
        }
        if (revision.op == RigExecRevisionOp::Ribbon && moverPrim) {
            _SampleMoverPathArrayCached<GfVec2f>(
                _FrozenRibbonInputKey(revision.moverPath, "bindCoords"),
                revision.binding.bindCoords, time, &refreshed, B.stage,
                &sampled, cache);
        }
    }
    for (const auto &[chainIndex, derivedIndex] : B.derivedIndex) {
        const RigExecBakedProgramImpl::GeomChain::Derived &derived =
            B.chains[size_t(chainIndex)].derived[size_t(derivedIndex)];
        const RigExecBakedProgramImpl::GeomRevision &revision =
            derived.revision;
        _SampleTopologyArrayCached<int>(revision.binding.topologyCounts, time,
                                        &refreshed, B.stage, &sampled, cache);
        _SampleTopologyArrayCached<int>(revision.binding.topologyIndices, time,
                                        &refreshed, B.stage, &sampled, cache);
        if (revision.op == RigExecRevisionOp::RecomputeExtent) {
            _SampleTopologyArrayCached<float>(revision.binding.widths, time,
                                              &refreshed, B.stage, &sampled,
                                              cache);
        }
        if (RigExecIsDerivedMatrixOp(revision.op)) {
            _SampleProjectorInputs(revision, &refreshed, B.stage, time,
                                   &sampled);
        }
    }
    sampled.revisionPackets.resize(B.revisionIndex.size());
    for (size_t r = 0; r < B.revisionIndex.size(); ++r) {
        const auto &[chainIndex, revisionIndex] = B.revisionIndex[r];
        const RigExecBakedProgramImpl::GeomRevision &revision =
            B.chains[size_t(chainIndex)].revisions[size_t(revisionIndex)];
        if (revision.op != RigExecRevisionOp::Skin) {
            continue;
        }
        std::shared_ptr<const RigExecSkinTopology> topology;
        if (revision.skinTopologyFixed) {
            topology = RigExecResolveSkinTopology(
                revision.moverPrim, revision.influenceSlots.size(), time,
                &refreshed, B.skinTopologies);
        }
        sampled.revisionPackets[r] = RigExecAssembleSkinParameters(
            revision.moverPrim, &revision.packetInfluences,
            /*weights=*/nullptr, time, &refreshed, B.skinTopologies,
            topology ? &topology : nullptr);
    }
    {
        std::string layoutError;
        if (!_SampleBlendLayouts(B, time, &sampled, &layoutError)) {
            return fail(layoutError);
        }
    }
    {
        std::string seedsError;
        if (!_SampleStageFrameSeeds(*cache->program, time, &sampled,
                                    &seedsError)) {
            return fail(seedsError);
        }
    }
    for (const RigExecValueOverride &o : overrides) {
        SdfPath key = o.prim;
        if (!o.attribute.IsEmpty()) {
            key = key.AppendProperty(o.attribute);
        } else if (!o.computation.IsEmpty()) {
            key = key.AppendProperty(TfToken(
                "rigExec:override:" + o.computation.GetString()));
        }
        if (!key.IsEmpty()) {
            sampled.Add(key, o.value, /*hasValue=*/true);
        }
    }

    *out = std::move(sampled);
    return true;
}

uint64_t
RigExecFrozenControlDigest(const RigExecFrameInputs &inputs, bool *exact)
{
    uint64_t hash = 1469598103934665603ull;
    const size_t count = inputs.values.size();
    hash = _HashBytes(hash, &count, sizeof(count));
    bool named = true;
    for (const RigExecSampledInput &sampled : inputs.values) {
        hash = _HashString(hash, sampled.path.GetString().c_str());
        hash = _HashBytes(hash, &sampled.hasValue, sizeof(sampled.hasValue));
        if (sampled.hasValue) {
            named = _HashVtValue(&hash, sampled.value) && named;
        } else {
            hash = _HashString(hash, sampled.value.GetTypeName().c_str());
        }
    }
    {
        // The stage seeds, folded like any other control state: a moved
        // constraint target must move this digest too, and a plain vector
        // and its burst-cached twin carry identical seeds.
        const RigExecStageFrameSeeds &seeds = inputs.stageSeeds;
        hash = _HashString(hash, "seeds");
        const auto foldMatrices = [&hash](const std::vector<GfMatrix4d> &ms) {
            const size_t count = ms.size();
            hash = _HashBytes(hash, &count, sizeof(count));
            for (const GfMatrix4d &m : ms) {
                for (int r = 0; r < 4; ++r) {
                    const GfVec4d row = m.GetRow(r);
                    hash = _HashBytes(hash, row.GetArray(),
                                      4 * sizeof(double));
                }
            }
        };
        const auto foldFrames =
            [&hash](const std::vector<RigExecPointFrame> &frames) {
                const size_t count = frames.size();
                hash = _HashBytes(hash, &count, sizeof(count));
                for (const RigExecPointFrame &frame : frames) {
                    for (int i = 0; i < 4; ++i) {
                        hash = _HashBytes(hash, frame.points[i].GetArray(),
                                          3 * sizeof(double));
                    }
                    hash = _HashBytes(hash, &frame.flags,
                                      sizeof(frame.flags));
                }
            };
        const auto foldOk = [&hash](const std::vector<char> &oks) {
            const size_t count = oks.size();
            hash = _HashBytes(hash, &count, sizeof(count));
            if (!oks.empty()) {
                hash = _HashBytes(hash, oks.data(), oks.size());
            }
        };
        foldMatrices(seeds.xformBase);
        foldFrames(seeds.xformFrames);
        foldOk(seeds.deltaOk);
        foldMatrices(seeds.deltaBase);
        foldOk(seeds.nativeOk);
        foldFrames(seeds.nativeFrames);
    }
    if (exact) {
        *exact = named;
    }
    return hash;
}

} // namespace rigExec
