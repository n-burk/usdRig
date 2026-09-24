//
// RigExec frozen evaluation contexts. See frozenContext.h.
//

#include "frozenContext.h"

#include "bakedProgramImpl.h"
#include "bakedSchedule.h"
#include "generation.h"
#include "tapSet.h"
#include "weightPackets.h"
#include "frameCacheSparsity.h"
#include "movers/moverRegistry.h"

#include "rigExecMath/curvenet.h"
#include "rigExecMath/geometryKernels.h"
#include "rigExecMath/profileMover.h"
#include "rigExecMath/propertyMath.h"

#include "pxr/base/gf/matrix4d.h"
#include "pxr/base/gf/vec2f.h"
#include "pxr/base/gf/vec3d.h"
#include "pxr/base/gf/vec3f.h"
#include "pxr/base/gf/vec3i.h"
#include "pxr/base/gf/vec4f.h"
#include "pxr/base/tf/staticTokens.h"
#include "pxr/base/tf/token.h"
#include "pxr/usd/sdf/types.h"
#include "pxr/usd/usd/attribute.h"
#include "pxr/usd/usd/attributeQuery.h"

#include <algorithm>
#include <cmath>
#include <cstring>
#include <map>
#include <set>

namespace rigExec {

namespace {

// A context that asks for more slots than this is corrupt, not large: the
// largest measured arena (biped, full frame) is ~2.7 MiB, so a gigabyte of
// doubles is three orders of magnitude past anything a real job sizes.
constexpr size_t kMaxFrozenArenaSlots = size_t(1) << 27;

// The sampler below mirrors the binding walk the Stream 0 bench measures
// (tests/benchFrameCache.cpp MeasureSampling): the same tables, in the same
// order, through the same read route the baked frame path takes. The bench
// is the timing; this is the values.

// Reads a chain-resolved attribute exactly as RigExecResolvedInputs::
// GetAttribute does -- an in-memory property value wins, otherwise a single
// authored connection is followed, otherwise the attribute's own value is
// read -- but type-erased into a VtValue, because the sampler names no
// binding's type.
//
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

// Each struct table's sampled fields, in emission order, as one list both
// the sampler and the burst-site builder walk: adding a sampled field here
// adds it to both, so the visit sets cannot drift from the samples.
template <class Fn>
void
_VisitLadderInputs(const RigExecBakedProgramImpl::Ladder &ladder, Fn &&fn)
{
    fn(ladder.restSpace);
    fn(ladder.defaultSpace);
    fn(ladder.posedSpace);
    for (int i = 0; i < 6; ++i) {
        fn(ladder.restAvars[i]);
        fn(ladder.defaultAvars[i]);
    }
    fn(ladder.rotationOrder);
}

template <class Fn>
void
_VisitSolverInputs(const RigExecBakedProgramImpl::Solver &solver, Fn &&fn)
{
    fn(solver.bend);
    fn(solver.upperOffset);
    fn(solver.lowerOffset);
    fn(solver.stretch);
    fn(solver.softness);
    fn(solver.blendWeight);
    fn(solver.preserveVolume);
    fn(solver.midFollowWeight);
    fn(solver.roll);
    fn(solver.twist);
    fn(solver.minLengthRatio);
    fn(solver.twistTurns);
    fn(solver.ribbonSampleCount);
}

template <class Fn>
void
_VisitConstraintInputs(
    const RigExecBakedProgramImpl::Constraint &constraint, Fn &&fn)
{
    fn(constraint.enabled);
    fn(constraint.defaultWeight);
    fn(constraint.offset);
    fn(constraint.affectX);
    fn(constraint.affectY);
    fn(constraint.affectZ);
    fn(constraint.tX);
    fn(constraint.tY);
    fn(constraint.tZ);
    fn(constraint.rX);
    fn(constraint.rY);
    fn(constraint.rZ);
    fn(constraint.sX);
    fn(constraint.sY);
    fn(constraint.sZ);
    fn(constraint.aimVector);
    fn(constraint.upVector);
    fn(constraint.rotationOffset);
    fn(constraint.worldUpVector);
    fn(constraint.poleVector);
    fn(constraint.twistDegrees);
}

template <class Obj, class Fn>
void
_VisitWeightInputs(Obj &object, Fn &&fn)
{
    fn(object.defaultWeight);
    fn(object.driver);
    fn(object.scale);
    fn(object.bias);
    fn(object.strength);
    fn(object.invert);
    fn(object.falloffMin);
    fn(object.falloffMax);
    fn(object.scaleX);
    fn(object.scaleY);
    fn(object.scaleZ);
    fn(object.extentU);
    fn(object.extentV);
    fn(object.curvenetSamples);
    fn(object.curvenetUnreached);
}

// The weight-object schema types the frozen packet build mirrors
// (bakedWeights.cpp names the same set).
TF_DEFINE_PRIVATE_TOKENS(
    _frozenWeightTokens,
    ((staticWeight, "RigExecStaticWeight"))
    ((dynamicWeight, "RigExecDynamicWeight"))
    ((combineWeight, "RigExecCombineWeight"))
    ((sphereWeight, "RigExecSphereWeight"))
    ((planeWeight, "RigExecPlaneWeight"))
    ((curveWeight, "RigExecCurveWeight"))
    ((curvenetWeight, "RigExecCurvenetWeight"))
);

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

template <class Fn>
void
_VisitInterpolatorInputs(
    const RigExecBakedProgramImpl::PoseInterpolator &interp, Fn &&fn)
{
    fn(interp.enabled);
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
//
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
        if (B.execTypedArrayInputs.count(path)) {
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

// The resolved-placement half of _ApplyInteractiveOverridesToResolved: every
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
                      const RigExecResolvedInputs *resolved, UsdTimeCode time,
                      RigExecFrameInputs *out)
{
    _VisitWeightInputs(object, [&](const auto &input) {
        _SampleBinding(input, resolved, time, out);
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
    gatherPoints(object.curvenetMeshPoints, "curvenetMeshPoints");
    gatherPoints(object.curvenetPoints, "curvenetPoints");
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
    const auto gatherInts =
        [&](const std::vector<UsdAttribute> &attributes, const char *role) {
            if (attributes.empty()) {
                return;
            }
            VtIntArray gathered;
            for (const UsdAttribute &a : attributes) {
                VtIntArray value;
                if (_FrozenReadWeightArray(refreshed, a, time, &value)) {
                    for (const int v : value) {
                        gathered.push_back(v);
                    }
                }
            }
            out->Add(_FrozenWeightArrayKey(object.path, role),
                     VtValue(gathered), /*hasValue=*/true);
        };
    gatherInts(object.curvenetCounts, "curvenetCounts");
    gatherInts(object.curvenetIndices, "curvenetIndices");
    gatherInts(object.curvenetSplines, "curvenetSplines");
    if (object.curvenetWeights.IsValid()) {
        VtFloatArray value;
        const bool has = _FrozenReadWeightArray(refreshed,
                                                object.curvenetWeights, time,
                                                &value);
        out->Add(_FrozenWeightArrayKey(object.path, "curvenetWeights"),
                 VtValue(has ? value : VtFloatArray()), has);
    }
    if (object.curvenetAutoSmooth.IsValid()) {
        VtIntArray value;
        const bool has = _FrozenReadWeightArray(refreshed,
                                                object.curvenetAutoSmooth,
                                                time, &value);
        out->Add(_FrozenWeightArrayKey(object.path, "curvenetAutoSmooth"),
                 VtValue(has ? value : VtIntArray()), has);
    }
}

// Samples one wire revision's side inputs exactly as the wire assembly arm
// reads them (moverGraph.cpp): rest points / order / knots at Default,
// dropoff at time, bind coordinates resolved-first, driver weight arrays
// resolved-first, and the posed driver curve (only when no driver
// transforms bind the table path). All under synthetic mover keys; the
// worker replays them, it cannot read the mover prim or the stage.
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

// Builds one curvenet profile bind from Default-time reads: the same two
// calls the arm's build closure makes (RigExecAssembleParameters), minus
// the cache the sampler must not touch -- the bind cache has no locking
// at all, so resolving through it here would race the live prologue.
// Pure stage reads plus pure math, so building per sample is race-free;
// the burst memoizes per mover below. Null when the net is missing, when
// the bind inputs are empty (live breaks before resolving), or when the
// build itself fails (a remembered failed bind, as live).
std::shared_ptr<const RigExecProfileMoverBinding>
_BuildCurvenetBind(
    const RigExecBakedProgramImpl::GeomRevision &revision,
    const UsdStageRefPtr &stage)
{
    const RigExecRevisionBinding &binding = revision.binding;
    if (!revision.moverPrim || !stage) {
        return nullptr;
    }
    const UsdPrim netPrim = stage->GetPrimAtPath(binding.curvenet);
    if (!netPrim) {
        return nullptr;
    }
    const auto readPoints = [&](const SdfPath &path) {
        std::vector<GfVec3f> out;
        if (!path.IsEmpty()) {
            if (const UsdAttribute a = stage->GetAttributeAtPath(path)) {
                VtVec3fArray value;
                if (a.Get(&value, UsdTimeCode::Default())) {
                    out.assign(value.begin(), value.end());
                }
            }
        }
        return out;
    };
    const auto readInts = [&](const SdfPath &path) {
        std::vector<int> out;
        if (!path.IsEmpty()) {
            if (const UsdAttribute a = stage->GetAttributeAtPath(path)) {
                VtIntArray value;
                if (a.Get(&value, UsdTimeCode::Default())) {
                    out.assign(value.begin(), value.end());
                }
            }
        }
        return out;
    };
    const std::vector<GfVec3f> restNet = readPoints(binding.curvenetPoints);
    const std::vector<GfVec3f> meshPoints = readPoints(binding.base);
    const std::vector<int> counts = readInts(binding.topologyCounts);
    const std::vector<int> indices = readInts(binding.topologyIndices);
    if (restNet.empty() || counts.empty() || meshPoints.empty()) {
        return nullptr;
    }
    std::vector<int> splineIndices;
    if (const UsdAttribute a =
            netPrim.GetAttribute(TfToken("rigExec:splineIndices"))) {
        VtIntArray value;
        if (a.Get(&value, UsdTimeCode::Default())) {
            splineIndices.assign(value.begin(), value.end());
        }
    }
    int samplesPerSpline = 5;
    if (const UsdAttribute a =
            netPrim.GetAttribute(TfToken("rigExec:samplesPerSpline"))) {
        a.Get(&samplesPerSpline, UsdTimeCode::Default());
    }
    TfToken basisToken("bezier");
    if (const UsdAttribute a =
            netPrim.GetAttribute(TfToken("rigExec:basis"))) {
        TfToken read;
        if (a.Get(&read, UsdTimeCode::Default())) {
            basisToken = read;
        }
    }
    RigExecCurvenetTopology topology;
    std::string reason;
    const RigExecCurvenetBasis basis =
        (basisToken == "catmullRom") ? RigExecCurvenetBasis::CatmullRom
                                    : RigExecCurvenetBasis::Bezier;
    if (!RigExecBuildCurvenetTopology(splineIndices, restNet.size(), basis,
                                      restNet, nullptr, &topology,
                                      &reason)) {
        return nullptr;
    }
    auto bound = std::make_shared<RigExecProfileMoverBinding>();
    if (!RigExecBindProfileMover(topology, restNet, meshPoints, counts,
                                 indices, samplesPerSpline, bound.get(),
                                 &reason)) {
        return nullptr;
    }
    return bound;
}

// Resolves every curvenet profile bind for the job, parallel to
// revisionIndex. A null entry is a non-curvenet revision, a missing net,
// or a remembered failed bind. Epoch data: excluded from the digest.
void
_SampleCurvenetBinds(const RigExecBakedProgramImpl &B,
                     RigExecFrameInputs *sampled,
                     RigExecBurstSampleCache *burst)
{
    sampled->curvenetBinds.resize(B.revisionIndex.size());
    for (size_t r = 0; r < B.revisionIndex.size(); ++r) {
        const auto &[chainIndex, revisionIndex] = B.revisionIndex[r];
        const RigExecBakedProgramImpl::GeomRevision &revision =
            B.chains[size_t(chainIndex)].revisions[size_t(revisionIndex)];
        if (revision.op != RigExecRevisionOp::Curvenet) {
            continue;
        }
        // Prefer the live revision's own resolved bind: pointer-identical
        // across samples, which is what keeps the worker's static-dirty
        // compare honest -- a fresh build per sample would read dirty
        // every frame and execute what live skips. The bind's inputs are
        // all Default-time, so a bind live resolved at any frame of the
        // epoch is this frame's bind.
        if (revision.curvenetBindResolved) {
            sampled->curvenetBinds[r] = revision.curvenetBind;
            continue;
        }
        if (burst) {
            const auto found = burst->curvenetBinds.find(
                revision.moverPath);
            if (found != burst->curvenetBinds.end()) {
                sampled->curvenetBinds[r] = found->second;
                continue;
            }
        }
        std::shared_ptr<const RigExecProfileMoverBinding> bound =
            _BuildCurvenetBind(revision, B.stage);
        if (burst) {
            burst->curvenetBinds[revision.moverPath] = bound;
        }
        sampled->curvenetBinds[r] = std::move(bound);
    }
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

// Synthetic keys for the curvenet arm's reads, all under the mover: the
// rest surface shares its path with the chain's own base read at a
// different time, the net shares its path between rest and posed, and a
// missing net must sample nothing at all so the worker breaks with
// exactly the packet live breaks with.
SdfPath
_FrozenCurvenetInputKey(const SdfPath &moverPath, const char *role)
{
    return moverPath.AppendProperty(
        TfToken(std::string("frozenCurvenet:") + role));
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

// Samples one _Array arm that passes no resolved inputs (the curvenet
// posed net): a raw stage read at the job's time.
template <class T>
void
_SampleMoverPathArrayRawAtTime(const SdfPath &key, const SdfPath &path,
                               UsdTimeCode time,
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
            hasValue = a.Get(&value, time);
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

template <class T>
void
_SampleMoverPathArrayRawAtTimeCached(const SdfPath &key, const SdfPath &path,
                                     UsdTimeCode time,
                                     const UsdStageRefPtr &stage,
                                     RigExecFrameInputs *out,
                                     RigExecBurstSampleCache *cache)
{
    if (key.IsEmpty() || path.IsEmpty()) {
        return;
    }
    const auto found = cache->staticStage.find(key);
    if (found != cache->staticStage.end()) {
        _ServeBurstStaticSample(out, found->second,
                                RigExecBurstRouteStage);
        return;
    }
    _SampleMoverPathArrayRawAtTime<T>(key, path, time, stage, out);
    const UsdAttribute attribute =
        stage ? stage->GetAttributeAtPath(path) : UsdAttribute();
    if (_BurstAttributeIsStatic(attribute)) {
        _MemoizeBurstStaticSample(out, &cache->staticStage,
                                  RigExecBurstRouteStage);
    }
}

// ---------------------------------------------------------------------------
// The chain-sampling hook (Increment B).
// ---------------------------------------------------------------------------
//
// Replicates the live property-chain prologue through public API only: chain
// discovery from the evaluator's mover order, _BindInput pinning, and the
// _EvaluatePropertyChains revision loop over the property-math kernels. The
// sampler runs it for the job's time on the UI thread, into caller-owned
// resolved inputs seeded with the job's overrides, and reads chain-resolved
// bindings through the refreshed values.
//
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
//
// What is NOT replicated is the live path's memoization (the watch/upstream
// dirty skip): it republishes identical values and lines, so recomputing
// every call changes no answer. Weight-object envelopes are declined, not
// replicated: they resolve through the evaluator's live oracle.

bool
_ChainIsFinite(float v)
{
    return std::isfinite(v);
}

bool
_ChainIsFinite(const GfVec3f &v)
{
    return std::isfinite(v[0]) && std::isfinite(v[1]) && std::isfinite(v[2]);
}

bool
_ChainIsFinite(const GfMatrix4d &m)
{
    for (size_t r = 0; r < 4; ++r) {
        for (size_t c = 0; c < 4; ++c) {
            if (!std::isfinite(m[r][c])) {
                return false;
            }
        }
    }
    return true;
}

// NOTE: no double overload, exactly as on the live path: a double base is
// checked through the float conversion, so a finite double past float range
// skips the chain on both paths alike.

bool
_IsChainMathMover(const TfToken &schemaType)
{
    return RigExecIsPropertyMover(schemaType);
}

struct _ChainMoverDesc {
    SdfPath moverPath;
    TfToken schemaType;
    SdfPath target;
};

// Mirror of the compile pass that fills the evaluator's property chains:
// every math mover in mover order, each over its one exact property
// target. Validation guarantees the shape for a compiled rig; anything
// else fails the bind, never the evaluation.
bool
_DiscoverChainMovers(const RigExecRigEvaluator &evaluator,
                     std::vector<_ChainMoverDesc> *out, std::string *error)
{
    for (const RigExecMoverRecord &record : evaluator.GetMoverOrder()) {
        if (!_IsChainMathMover(record.schemaType)) {
            continue;
        }
        if (record.targets.size() != 1 ||
            !record.targets[0].IsPropertyPath()) {
            if (error) {
                *error = record.schemaType.GetString() + " " +
                         record.moverPath.GetString() +
                         " has no single exact property target";
            }
            return false;
        }
        _ChainMoverDesc desc;
        desc.moverPath = record.moverPath;
        desc.schemaType = record.schemaType;
        desc.target = record.targets[0];
        out->push_back(desc);
    }
    return true;
}

// Mirror of the compile pass that orders the chains: producers first, over
// the same dependency edges (every mover attribute's connection walk, plus
// the weight prims behind rigExec:weightObject). Same walk, same sets,
// same depth-first visit -- so the same order, deterministically.
bool
_OrderDiscoveredChains(
    const UsdStageRefPtr &stage,
    const std::map<SdfPath, std::vector<_ChainMoverDesc>> &chains,
    std::vector<SdfPath> *order, std::string *error)
{
    std::map<SdfPath, std::set<SdfPath>> dependsOn;
    for (const auto &[target, _] : chains) {
        dependsOn[target];
    }
    auto addAttributeDependency = [&](const SdfPath &consumer,
                                      const UsdAttribute &attribute) {
        std::set<SdfPath> visited;
        std::function<void(const UsdAttribute &)> walk =
            [&](const UsdAttribute &a) {
            if (!a || !visited.insert(a.GetPath()).second) {
                return;
            }
            if (chains.count(a.GetPath())) {
                dependsOn[consumer].insert(a.GetPath());
            }
            SdfPathVector connections;
            if (a.HasAuthoredConnections()) {
                a.GetConnections(&connections);
            }
            for (const SdfPath &sourcePath : connections) {
                walk(stage->GetAttributeAtPath(sourcePath));
            }
        };
        walk(attribute);
    };
    auto addPrimDependencies = [&](const SdfPath &consumer,
                                   const UsdPrim &prim) {
        if (!prim) {
            return;
        }
        for (const UsdAttribute &attribute : prim.GetAttributes()) {
            addAttributeDependency(consumer, attribute);
        }
    };
    std::function<void(const SdfPath &, const SdfPath &, std::set<SdfPath> *)>
        addWeightDependencies =
            [&](const SdfPath &consumer, const SdfPath &weightPath,
                std::set<SdfPath> *visited) {
            if (!visited->insert(weightPath).second) {
                return;
            }
            const UsdPrim weight = stage->GetPrimAtPath(weightPath);
            addPrimDependencies(consumer, weight);
            for (const char *relationship :
                 {"rigExec:inputWeights", "rigExec:baseWeight"}) {
                SdfPathVector inputs;
                if (const UsdRelationship rel =
                        weight.GetRelationship(TfToken(relationship))) {
                    rel.GetTargets(&inputs);
                }
                for (const SdfPath &input : inputs) {
                    addWeightDependencies(consumer, input, visited);
                }
            }
        };

    for (const auto &[target, revisions] : chains) {
        for (const _ChainMoverDesc &revision : revisions) {
            const UsdPrim mover = stage->GetPrimAtPath(revision.moverPath);
            addPrimDependencies(target, mover);
            SdfPathVector weights;
            if (const UsdRelationship rel = mover.GetRelationship(
                    TfToken("rigExec:weightObject"))) {
                rel.GetTargets(&weights);
            }
            std::set<SdfPath> visitedWeights;
            for (const SdfPath &weight : weights) {
                addWeightDependencies(target, weight, &visitedWeights);
            }
        }
    }

    std::map<SdfPath, int> colour;
    std::vector<SdfPath> stack;
    std::function<bool(const SdfPath &)> visit = [&](const SdfPath &target) {
        colour[target] = 1;
        stack.push_back(target);
        for (const SdfPath &producer : dependsOn[target]) {
            if (colour[producer] == 1) {
                if (error) {
                    std::string cycle;
                    for (const SdfPath &path : stack) {
                        cycle += path.GetString() + " -> ";
                    }
                    cycle += producer.GetString();
                    *error = "property input dependency cycle: " + cycle;
                }
                return false;
            }
            if (colour[producer] == 0 && !visit(producer)) {
                return false;
            }
        }
        stack.pop_back();
        colour[target] = 2;
        order->push_back(target);
        return true;
    };
    for (const auto &[target, _] : dependsOn) {
        if (colour[target] == 0 && !visit(target)) {
            return false;
        }
    }
    return true;
}

// Mirror of _BindInput: one chain input pinned, or empty when the mover
// authors none. Constants fold at Default rather than at the live path's
// first-run time; a constant reads identically at every time code, so the
// pins agree whatever frame ran first.
RigExecChainSampleInput
_BindChainInput(const UsdPrim &prim, const char *name)
{
    RigExecChainSampleInput input;
    if (!prim) {
        return input;
    }
    if (const UsdAttribute a = prim.GetAttribute(TfToken(name))) {
        input.attribute = a;
        input.path = a.GetPath();
        input.connected = a.HasAuthoredConnections();
        input.query = UsdAttributeQuery(a);
        input.constant = !input.connected && !a.ValueMightBeTimeVarying() &&
                         a.GetNumTimeSamples() == 0;
        if (input.constant) {
            a.Get(&input.constantValue, UsdTimeCode::Default());
        }
    }
    return input;
}

// Mirror of _PinnedRead, arm for arm: no attribute answers the fallback; a
// connected attribute walks the resolved inputs (the return ignored, so a
// failed walk keeps the fallback); an in-memory value of the read type
// wins; a folded constant answers; otherwise the pinned query resolves.
template <class T>
T
_ChainPinnedRead(const RigExecResolvedInputs &resolved,
                 const RigExecChainSampleInput &input, T fallback,
                 UsdTimeCode time)
{
    T value = fallback;
    if (!input.attribute) {
        return value;
    }
    if (input.connected) {
        resolved.GetAttribute(input.attribute, time, &value);
        return value;
    }
    if (const VtValue *const standing = resolved.Find(input.path)) {
        if (standing->IsHolding<T>()) {
            return standing->UncheckedGet<T>();
        }
    }
    if (input.constant && input.constantValue.IsHolding<T>()) {
        return input.constantValue.UncheckedGet<T>();
    }
    T resolvedValue;
    if (input.query.Get(&resolvedValue, time)) {
        value = resolvedValue;
    }
    return value;
}

// Mirror of _ReadOperation: the operation names the arithmetic, not a
// value, so it is read at Default with no resolved inputs consulted.
void
_ChainReadOperation(const RigExecChainSampleInput &input, TfToken *operation)
{
    if (input.constant) {
        if (input.constantValue.IsHolding<TfToken>()) {
            *operation = input.constantValue.UncheckedGet<TfToken>();
        }
        return;
    }
    input.query.Get(operation);
}

// Mirror of _ReadPinnedPropertyMathParams: the mover's whole authored
// state, read through the pinned inputs whether or not the operation uses
// every field.
template <class T>
bool
_ChainReadMathParams(const RigExecResolvedInputs &resolved,
                     const RigExecChainSampleRevision &mover, UsdTimeCode time,
                     RigExecPropertyMathParams<T> *params)
{
    TfToken operation;
    if (mover.operation) {
        _ChainReadOperation(mover.operation, &operation);
    }
    if (!RigExecParsePropertyOp(operation, &params->op)) {
        return false;
    }
    params->value =
        _ChainPinnedRead(resolved, mover.value, params->value, time);
    params->min = _ChainPinnedRead(resolved, mover.minimum, params->min, time);
    params->max = _ChainPinnedRead(resolved, mover.maximum, params->max, time);
    return true;
}

// Within-process mixing (never persisted, never compared across runs, never
// baked into tests): a word-at-a-time splitmix64 finalizer, the same mixer
// the frame-cache digest uses in frameCache.cpp -- identical logical
// stream, one mix per 8 bytes instead of one FNV round per byte, at roughly
// a third of the multiplies.
uint64_t
_MixWord(uint64_t hash, uint64_t word)
{
    // splitmix64 finalizer for the word, FNV-style chaining for the state.
    // The multiply matters: an xor-only fold is commutative, so duplicate
    // words cancel anywhere in the stream ([W,W] folds to the seed whatever
    // W is) and permutations collide -- both serve stale poses.
    word += 0x9E3779B97F4A7C15ull;
    word = (word ^ (word >> 30)) * 0xBF58476D1CE4E5B9ull;
    word = (word ^ (word >> 27)) * 0x94D049BB133111EBull;
    word ^= word >> 31;
    hash ^= word;
    hash *= 1099511628211ull;
    return hash;
}

uint64_t
_HashBytes(uint64_t hash, const void *data, size_t size)
{
    const unsigned char *bytes = static_cast<const unsigned char *>(data);
    while (size >= 8) {
        uint64_t word;
        std::memcpy(&word, bytes, sizeof(word));
        hash = _MixWord(hash, word);
        bytes += 8;
        size -= 8;
    }
    if (size > 0) {
        // Zero-padded, with the length mixed in so a short tail cannot
        // equal a longer stream's shared prefix. An empty hash stays a
        // no-op, as before.
        uint64_t tail = 0;
        std::memcpy(&tail, bytes, size);
        hash = _MixWord(hash, tail + size);
    }
    return hash;
}

uint64_t
_HashString(uint64_t hash, const char *text)
{
    return _HashBytes(hash, text, std::strlen(text));
}

// Mixes a VtValue's bytes. Returns false for a type this function does not
// name, having mixed only the type name: the caller reports the digest
// inexact rather than risk two different frames sharing one key.
bool
_HashVtValue(uint64_t *hash, const VtValue &value)
{
    *hash = _HashString(*hash, value.GetTypeName().c_str());
    if (value.IsEmpty()) {
        return true;
    }
#define _RIGEXEC_FROZEN_SCALAR(type)                  \
    if (value.IsHolding<type>()) {                    \
        const type &held = value.UncheckedGet<type>(); \
        *hash = _HashBytes(*hash, &held, sizeof(held)); \
        return true;                                  \
    }
    _RIGEXEC_FROZEN_SCALAR(bool)
    _RIGEXEC_FROZEN_SCALAR(int)
    _RIGEXEC_FROZEN_SCALAR(float)
    _RIGEXEC_FROZEN_SCALAR(double)
    _RIGEXEC_FROZEN_SCALAR(GfVec2f)
    _RIGEXEC_FROZEN_SCALAR(GfVec3f)
    _RIGEXEC_FROZEN_SCALAR(GfVec3d)
    _RIGEXEC_FROZEN_SCALAR(GfVec3i)
    _RIGEXEC_FROZEN_SCALAR(GfVec4f)
    _RIGEXEC_FROZEN_SCALAR(GfMatrix4d)
#undef _RIGEXEC_FROZEN_SCALAR
#define _RIGEXEC_FROZEN_ARRAY(type, element)                       \
    if (value.IsHolding<type>()) {                                 \
        const type &held = value.UncheckedGet<type>();              \
        const size_t count = held.size();                          \
        *hash = _HashBytes(*hash, &count, sizeof(count));           \
        if (!held.empty()) {                                       \
            *hash = _HashBytes(*hash, held.cdata(),                 \
                               held.size() * sizeof(element));      \
        }                                                          \
        return true;                                               \
    }
    _RIGEXEC_FROZEN_ARRAY(VtBoolArray, bool)
    _RIGEXEC_FROZEN_ARRAY(VtIntArray, int)
    _RIGEXEC_FROZEN_ARRAY(VtFloatArray, float)
    _RIGEXEC_FROZEN_ARRAY(VtDoubleArray, double)
    _RIGEXEC_FROZEN_ARRAY(VtVec2fArray, GfVec2f)
    _RIGEXEC_FROZEN_ARRAY(VtVec3fArray, GfVec3f)
    _RIGEXEC_FROZEN_ARRAY(VtVec3dArray, GfVec3d)
    _RIGEXEC_FROZEN_ARRAY(VtVec4fArray, GfVec4f)
#undef _RIGEXEC_FROZEN_ARRAY
    if (value.IsHolding<std::string>()) {
        *hash = _HashString(*hash, value.UncheckedGet<std::string>().c_str());
        return true;
    }
    if (value.IsHolding<TfToken>()) {
        *hash = _HashString(*hash, value.UncheckedGet<TfToken>().GetText());
        return true;
    }
    if (value.IsHolding<SdfPath>()) {
        *hash = _HashString(
            *hash, value.UncheckedGet<SdfPath>().GetString().c_str());
        return true;
    }
    if (value.IsHolding<VtStringArray>()) {
        const VtStringArray &held = value.UncheckedGet<VtStringArray>();
        const size_t count = held.size();
        *hash = _HashBytes(*hash, &count, sizeof(count));
        for (const std::string &entry : held) {
            *hash = _HashString(*hash, entry.c_str());
        }
        return true;
    }
    return false;
}

}  // namespace

void
RigExecFrameInputs::Add(const SdfPath &path, const VtValue &value,
                        bool hasValue, bool viaChain)
{
    RigExecSampledInput sampled;
    sampled.path = path;
    sampled.value = value;
    sampled.hasValue = hasValue;
    sampled.viaChain = viaChain;
    values.push_back(sampled);
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

const VtValue *
RigExecFrameInputs::Find(const SdfPath &path) const
{
    for (const RigExecSampledInput &sampled : values) {
        if (sampled.path == path) {
            return &sampled.value;
        }
    }
    return nullptr;
}

bool
RigExecFrameInputs::Contains(const SdfPath &path) const
{
    for (const RigExecSampledInput &sampled : values) {
        if (sampled.path == path) {
            return true;
        }
    }
    return false;
}

void
RigExecFrameInputs::Clear()
{
    time = UsdTimeCode::Default();
    values.clear();
    revisionPackets.clear();
    stageSeeds = RigExecStageFrameSeeds();
    chainDiagnostics.clear();
    chainResults.clear();
    overrides.clear();
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

namespace {

// Thread-local serial depth. A plain counter rather than a bool so scopes
// nest; the query is "inside at least one".
thread_local size_t _frozenSerialDepth = 0;
thread_local std::shared_ptr<const void> _lastFrozenSlots;
thread_local size_t _lastFrozenSlotBytes = 0;

}  // namespace

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
                      const SdfPath &rig)
{
    _lastFrozenSlots.reset();
    _lastFrozenSlotBytes = 0;
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
        ran = runner(context, inputs, arena, &pose);
    }
    if (!ran || !pose.valid) {
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

// Defined below, beside the executor.
bool _RunFrozen(const RigExecFrozenEvalContext &context,
                const RigExecFrameInputs &inputs, RigExecRigPose *pose);

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
        bound.chains.push_back(std::move(chain));
    }
    *out = std::move(bound);
    return true;
}

namespace {

// The eight inputs a revision pins, by attribute name, for the currency
// check below.
const char *const _kChainInputNames[8] = {
    "inputs:enabled", "inputs:defaultWeight", "rigExec:operation",
    "inputs:value", "inputs:min", "inputs:max", "inputs:keys",
    "inputs:tangents"};

const RigExecChainSampleInput *
_ChainBoundInput(const RigExecChainSampleRevision &revision, size_t i)
{
    switch (i) {
    case 0: return &revision.enabled;
    case 1: return &revision.defaultWeight;
    case 2: return &revision.operation;
    case 3: return &revision.value;
    case 4: return &revision.minimum;
    case 5: return &revision.maximum;
    case 6: return &revision.keys;
    default: return &revision.tangents;
    }
}

}  // namespace

bool
RigExecChainSampleBindingsStillCurrent(
    const RigExecChainSampleBindings &bindings,
    const RigExecRigEvaluator &evaluator)
{
    const UsdStageRefPtr stage = evaluator.GetEvaluationStage();
    if (!stage) {
        return false;
    }
    // The discovery signature: the same math movers over the same targets.
    // Mover membership is epoch-digest-covered, so a mismatch also means a
    // new epoch -- but the check is cheap and the failure mode of trusting
    // it is a silently misbound chain.
    std::vector<_ChainMoverDesc> movers;
    std::string ignored;
    if (!_DiscoverChainMovers(evaluator, &movers, &ignored)) {
        return false;
    }
    std::map<SdfPath, std::vector<_ChainMoverDesc>> grouped;
    for (const _ChainMoverDesc &mover : movers) {
        grouped[mover.target].push_back(mover);
    }
    if (grouped.size() != bindings.chains.size()) {
        return false;
    }
    for (const RigExecChainSampleChain &chain : bindings.chains) {
        const auto found = grouped.find(chain.targetPath);
        if (found == grouped.end() ||
            found->second.size() != chain.revisions.size()) {
            return false;
        }
        for (size_t i = 0; i < chain.revisions.size(); ++i) {
            if (found->second[i].moverPath !=
                    chain.revisions[i].moverPath ||
                found->second[i].schemaType !=
                    chain.revisions[i].moverPrim.GetTypeName()) {
                return false;
            }
        }
        // The target resolves as it did, to the same type: the evaluation
        // dispatches on the target's type, and a mid-epoch retype moves no
        // epoch digest.
        const UsdAttribute freshTarget =
            stage->GetAttributeAtPath(chain.targetPath);
        if (bool(freshTarget) != bool(chain.target)) {
            return false;
        }
        if (freshTarget && freshTarget.GetTypeName() != chain.valueType) {
            return false;
        }
        for (const RigExecChainSampleRevision &revision : chain.revisions) {
            const UsdPrim mover = stage->GetPrimAtPath(revision.moverPath);
            if (bool(mover) != bool(revision.moverPrim)) {
                return false;
            }
            SdfPathVector weights;
            if (const UsdRelationship rel = mover.GetRelationship(
                    TfToken("rigExec:weightObject"))) {
                rel.GetTargets(&weights);
            }
            if (weights != revision.weightObjects) {
                return false;
            }
            // Every pin: still present iff pinned, still classified as
            // pinned, and a folded constant still reading its pinned value.
            // Value edits move no epoch digest, so without this a constant
            // edited mid-epoch would read stale until the next recompile.
            for (size_t i = 0; i < 8; ++i) {
                const RigExecChainSampleInput *pinned =
                    _ChainBoundInput(revision, i);
                const UsdAttribute fresh = mover.GetAttribute(
                    TfToken(_kChainInputNames[i]));
                if (bool(fresh) != bool(*pinned)) {
                    return false;
                }
                if (!fresh) {
                    continue;
                }
                const bool connected = fresh.HasAuthoredConnections();
                const bool constant =
                    !connected && !fresh.ValueMightBeTimeVarying() &&
                    fresh.GetNumTimeSamples() == 0;
                if (connected != pinned->connected ||
                    constant != pinned->constant) {
                    return false;
                }
                if (constant) {
                    VtValue now;
                    fresh.Get(&now, UsdTimeCode::Default());
                    if (now != pinned->constantValue) {
                        return false;
                    }
                }
            }
        }
    }
    return true;
}

bool
RigExecEvaluateChainsForTime(
    const RigExecChainSampleBindings &bindings, UsdTimeCode time,
    RigExecResolvedInputs *resolved, std::map<SdfPath, VtValue> *results,
    std::vector<std::string> *diagnostics, std::string *error)
{
    if (!resolved) {
        if (error) {
            *error = "no resolved inputs to evaluate the chains into";
        }
        return false;
    }
    auto diag = [diagnostics](const std::string &message) {
        if (diagnostics) {
            diagnostics->push_back(message);
        }
    };

    for (const RigExecChainSampleChain &chain : bindings.chains) {
        const SdfPath &target = chain.targetPath;
        if (!chain.target) {
            diag("property chain " + target.GetString() +
                 ": target attribute disappeared; chain skipped");
            continue;
        }
        resolved->ClearProperty(target);
        const SdfValueTypeName &valueType = chain.valueType;

        // One shared revision loop over the three value domains, as on the
        // live path: each iteration reads the mover's own authored state
        // and applies it to the preceding revision, from the target's
        // AUTHORED base value.
        auto runChain = [&](auto value, auto apply) {
            using ValueT = decltype(value);
            if (!chain.targetQuery.Get(&value, time)) {
                diag("property chain " + target.GetString() +
                     ": target has no authored value; chain skipped");
                return true;
            }
            if (!_ChainIsFinite(value)) {
                diag("property chain " + target.GetString() +
                     ": authored base is not finite; chain skipped");
                return true;
            }
            for (const RigExecChainSampleRevision &revision :
                 chain.revisions) {
                const UsdPrim &moverPrim = revision.moverPrim;
                if (!moverPrim) {
                    continue;
                }
                const SdfPath moverPath = moverPrim.GetPath();
                const bool enabled = _ChainPinnedRead(
                    *resolved, revision.enabled, true, time);
                if (!enabled) {
                    diag("diag " + moverPath.GetString() +
                         ": disabled; revision passed through");
                    continue;
                }
                float envelope = 1.0f;
                if (!revision.weightObjects.empty()) {
                    if (error) {
                        *error =
                            "diag " + moverPath.GetString() +
                            " binds a weight object, whose envelope resolves "
                            "through the evaluator's live oracle";
                    }
                    return false;
                }
                envelope = _ChainPinnedRead(
                    *resolved, revision.defaultWeight, 1.0f, time);
                if (!std::isfinite(envelope) || envelope < 0.0f ||
                    envelope > 1.0f) {
                    diag("diag " + moverPath.GetString() +
                         ": inputs:defaultWeight must be finite and in "
                         "[0, 1]; revision passed through");
                    continue;
                }
                ValueT next = value;
                if (!apply(revision, value, envelope, &next)) {
                    diag("diag " + moverPath.GetString() +
                         ": inputs unusable; revision passed through");
                    continue;
                }
                if (!_ChainIsFinite(next)) {
                    diag("diag " + moverPath.GetString() +
                         ": produced a non-finite value; revision passed "
                         "through");
                    continue;
                }
                value = next;
            }
            if (results) {
                (*results)[target] = VtValue(value);
            }
            resolved->SetProperty(target, VtValue(value));
            return true;
        };

        const auto applyFloat = [&](const RigExecChainSampleRevision &mover,
                                    float in, float envelope, float *out) {
            RigExecPropertyMathParams<float> params;
            if (!_ChainReadMathParams(*resolved, mover, time, &params) ||
                !_ChainIsFinite(params.value) || !_ChainIsFinite(params.min) ||
                !_ChainIsFinite(params.max)) {
                return false;
            }
            VtArray<GfVec2f> keys;
            VtArray<GfVec2f> tangents;
            if (params.op == RigExecPropertyOp::Curve) {
                keys = _ChainPinnedRead(*resolved, mover.keys, keys, time);
                if (keys.empty() || !RigExecValidateLinearKeys(
                                        keys.cdata(), keys.size())) {
                    return false;
                }
                params.keys = keys.cdata();
                params.keyCount = keys.size();
                if (mover.tangents) {
                    tangents = _ChainPinnedRead(
                        *resolved, mover.tangents, tangents, time);
                }
                if (!tangents.empty()) {
                    if (tangents.size() != keys.size()) {
                        return false;
                    }
                    params.tangents = tangents.cdata();
                    params.tangentCount = tangents.size();
                }
            }
            params.weight = envelope;
            *out = RigExecApplyFloatMath(in, params);
            return true;
        };
        bool chainOk = true;
        if (valueType == SdfValueTypeNames->Float) {
            chainOk = runChain(float(0), applyFloat);
        } else if (valueType == SdfValueTypeNames->Double) {
            chainOk = runChain(
                double(0),
                [&](const RigExecChainSampleRevision &mover, double in,
                    float envelope, double *out) {
                    float result = 0.0f;
                    if (!applyFloat(mover, float(in), envelope, &result)) {
                        return false;
                    }
                    *out = double(result);
                    return true;
                });
        } else if (valueType == SdfValueTypeNames->Matrix4d) {
            chainOk = runChain(
                GfMatrix4d(1.0),
                [&](const RigExecChainSampleRevision &mover,
                    const GfMatrix4d &in, float envelope, GfMatrix4d *out) {
                    TfToken operation;
                    if (mover.operation) {
                        _ChainReadOperation(mover.operation, &operation);
                    }
                    RigExecPropertyOp op;
                    if (!RigExecParsePropertyOp(operation, &op)) {
                        return false;
                    }
                    const GfMatrix4d opValue = _ChainPinnedRead(
                        *resolved, mover.value, GfMatrix4d(1.0), time);
                    if (!_ChainIsFinite(opValue)) {
                        return false;
                    }
                    return RigExecApplyMatrixMath(
                        in, op, opValue, envelope, out);
                });
        } else {
            // Every remaining type the compiler admits is GfVec3f-backed.
            chainOk = runChain(
                GfVec3f(0),
                [&](const RigExecChainSampleRevision &mover, const GfVec3f &in,
                    float envelope, GfVec3f *out) {
                    RigExecPropertyMathParams<GfVec3f> params;
                    if (!_ChainReadMathParams(
                            *resolved, mover, time, &params) ||
                        !_ChainIsFinite(params.value) ||
                        !_ChainIsFinite(params.min) ||
                        !_ChainIsFinite(params.max)) {
                        return false;
                    }
                    params.weight = envelope;
                    *out = RigExecApplyVec3fMath(in, params);
                    return true;
                });
        }
        if (!chainOk) {
            return false;
        }
    }
    return true;
}

bool
RigExecSampleFrameInputs(const RigExecRigEvaluator &evaluator,
                         UsdTimeCode time,
                         const std::vector<RigExecValueOverride> &overrides,
                         RigExecFrameInputs *out, std::string *error)
{
    // Bind fresh, then sample through the pinned route: the two routes
    // share every line below the bind, and the equivalence test holds them
    // to account sample by sample. A bind failure names no chains at all,
    // so no vector is built; an evaluation decline (a weight object) still
    // builds a marked vector that declines downstream.
    RigExecChainSampleBindings fresh;
    if (!RigExecBindChainSampleInputs(evaluator, &fresh, error)) {
        return false;
    }
    return RigExecSampleFrameInputsWithChainBindings(
        evaluator, time, overrides, fresh, out, error);
}

bool
RigExecSampleFrameInputsWithChainBindings(
    const RigExecRigEvaluator &evaluator, UsdTimeCode time,
    const std::vector<RigExecValueOverride> &overrides,
    const RigExecChainSampleBindings &bindings, RigExecFrameInputs *out,
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
    // The pinned route trusts nothing: stale bindings fail the sample and
    // the caller rebinds. An empty pin on a chained rig fails here too, so
    // the hook cannot be skipped around -- only the bind, which names the
    // rig's actual chains, feeds this route.
    if (!RigExecChainSampleBindingsStillCurrent(bindings, evaluator)) {
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
    // pre-chain overrides, and the overrides are placed again after, so
    // one standing on a chain target replaces its result -- and the
    // transported per-target results, which the frozen prologue publishes
    // as its property results, are replaced alike. On decline (a weight
    // object) the refreshed inputs stay override-only and chain-resolved
    // bindings mark viaChain below, declining the vector downstream.
    const RigExecResolvedInputs *chainFresh = nullptr;
    if (!bindings.chains.empty()) {
        RigExecResolvedInputs hooked = refreshed;
        std::map<SdfPath, VtValue> results;
        std::vector<std::string> hookDiagnostics;
        std::string hookError;
        if (RigExecEvaluateChainsForTime(bindings, time, &hooked, &results,
                                         &hookDiagnostics, &hookError)) {
            _PlaceOverridesIntoResolved(overrides, &hooked);
            for (const RigExecValueOverride &o : overrides) {
                if (o.attribute.IsEmpty()) {
                    continue;
                }
                const auto found = results.find(
                    o.prim.AppendProperty(o.attribute));
                if (found != results.end()) {
                    found->second = o.value;
                }
            }
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
    for (const RigExecBakedProgramImpl::PoseInterpolator &interp :
         B.poseInterpolators) {
        _SampleFlaggedBinding(interp.enabled, resolved, &refreshed,
                              overrideFlags, time, &sampled, chainFresh);
    }
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
        _SampleWeightBindings(object, resolved, time, &sampled);
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
    //
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
        if (revision.op == RigExecRevisionOp::Curvenet && moverPrim) {
            // The bind inputs are Default-time epoch data; the posed net
            // is the only evaluated-time read, and it is raw (the arm's
            // _Array call passes no resolved -- the net chain's result,
            // when one binds, comes from the worker's own run). Nothing
            // samples when the net prim is missing: live breaks before
            // its first read.
            const UsdPrim netPrim =
                B.stage->GetPrimAtPath(revision.binding.curvenet);
            if (netPrim) {
                const SdfPath &moverPath = revision.moverPath;
                const RigExecRevisionBinding &binding = revision.binding;
                _SampleMoverPathArrayAtDefault<GfVec3f>(
                    _FrozenCurvenetInputKey(moverPath, "restNet"),
                    binding.curvenetPoints, B.stage, &sampled);
                _SampleMoverPathArrayAtDefault<int>(
                    _FrozenCurvenetInputKey(moverPath, "topologyCounts"),
                    binding.topologyCounts, B.stage, &sampled);
                _SampleMoverPathArrayAtDefault<int>(
                    _FrozenCurvenetInputKey(moverPath, "topologyIndices"),
                    binding.topologyIndices, B.stage, &sampled);
                _SampleMoverPathArrayAtDefault<GfVec3f>(
                    _FrozenCurvenetInputKey(moverPath, "restPoints"),
                    binding.base, B.stage, &sampled);
                _SampleMoverPathArrayRawAtTime<GfVec3f>(
                    _FrozenCurvenetInputKey(moverPath, "posedNet"),
                    binding.curvenetPoints, time, B.stage, &sampled);
            }
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
    _SampleCurvenetBinds(B, &sampled, /*burst=*/nullptr);
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
            _PlaceOverridesIntoResolved(overrides, &hooked);
            for (const RigExecValueOverride &o : overrides) {
                if (o.attribute.IsEmpty()) {
                    continue;
                }
                const auto found = results.find(
                    o.prim.AppendProperty(o.attribute));
                if (found != results.end()) {
                    found->second = o.value;
                }
            }
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
    for (size_t i : cache->interpolatorSites) {
        _VisitInterpolatorInputs(B.poseInterpolators[i],
                                 [&](const auto &input) {
            _SampleFlaggedBinding(input, resolved, &refreshed,
                                  cache->overrideFlags, time, &sampled,
                                  chainFresh);
        });
    }
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
            _SampleWeightBindings(B.weightObjects[i], resolved, time,
                                  &sampled);
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
        if (revision.op == RigExecRevisionOp::Curvenet && moverPrim) {
            const UsdPrim netPrim =
                B.stage->GetPrimAtPath(revision.binding.curvenet);
            if (netPrim) {
                const SdfPath &moverPath = revision.moverPath;
                const RigExecRevisionBinding &binding = revision.binding;
                _SampleMoverPathArrayAtDefaultCached<GfVec3f>(
                    _FrozenCurvenetInputKey(moverPath, "restNet"),
                    binding.curvenetPoints, B.stage, &sampled, cache);
                _SampleMoverPathArrayAtDefaultCached<int>(
                    _FrozenCurvenetInputKey(moverPath, "topologyCounts"),
                    binding.topologyCounts, B.stage, &sampled, cache);
                _SampleMoverPathArrayAtDefaultCached<int>(
                    _FrozenCurvenetInputKey(moverPath, "topologyIndices"),
                    binding.topologyIndices, B.stage, &sampled, cache);
                _SampleMoverPathArrayAtDefaultCached<GfVec3f>(
                    _FrozenCurvenetInputKey(moverPath, "restPoints"),
                    binding.base, B.stage, &sampled, cache);
                _SampleMoverPathArrayRawAtTimeCached<GfVec3f>(
                    _FrozenCurvenetInputKey(moverPath, "posedNet"),
                    binding.curvenetPoints, time, B.stage, &sampled, cache);
            }
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
    _SampleCurvenetBinds(B, &sampled, cache);
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

const std::vector<RigExecPurityFinding> &
RigExecFrozenPurityAudit()
{
    // One row per unit examined. "Examined" means read for static,
    // thread-local, and member state surviving a call, plus every lock and
    // every USD handle a worker could reach through it; the verdict is what
    // the frozen path may do with the unit.
    static const std::vector<RigExecPurityFinding> audit = {
        {"rigExecMath/* (avarScale, dualQuat, pointFrame, rbf, "
         "singleChainIk, solvers, splineIk, envelope, geometryKernels, "
         "propertyMath, weightFields)",
         RigExecFrozenPurity::Pure,
         "free functions over their arguments; no static, thread-local, or "
         "member state anywhere in the directory"},
        {"libs/rigExec/solverKernels.{h,cpp}",
         RigExecFrozenPurity::Pure,
         "ribbon/twist/rotation glue over caller buffers; no statics, no "
         "locks, no USD"},
        {"libs/rigExec/moverKernels.cpp exec callbacks and helpers",
         RigExecFrozenPurity::Pure,
         "only static const tokens and the SIMD env flag; per-evaluation "
         "state lives in the VdfContext, never in the kernel"},
        {"libs/rigExec/computations.cpp frame helpers",
         RigExecFrozenPurity::Pure,
         "static free functions over frames and params; guards and packing "
         "only, no retained state"},
        {"libs/rigExec/curvenetAdjuster.cpp, "
         "curvenetWeightComputations.cpp packet math",
         RigExecFrozenPurity::Pure,
         "bind/evaluate math over caller buffers; the file's one shared "
         "memo is the row below, never the math"},
        {"libs/rigExec/weightPackets.cpp packet math",
         RigExecFrozenPurity::Pure,
         "only static const tokens; assembly over caller buffers"},
        {"baked step bodies (bakedPose/bakedGeometry/bakedWeights/"
         "bakedVerify.cpp)",
         RigExecFrozenPurity::Pure,
         "a step reads declared slots and writes declared slots; per-step "
         "diagnostics and counters merge in step order, and BeginRun resets "
         "them, so no result survives into the next run; steps never touch "
         "USD (bakedProgram.cpp Run prologue comment)"},
        {"baked schedule serial executor",
         RigExecFrozenPurity::Pure,
         "program order on one thread; the reference every frozen run uses"},
        {"baked schedule parallel executor",
         RigExecFrozenPurity::LiveOnly,
         "WorkDispatcher + per-run atomics on the shared arena at normal "
         "priority; the frozen serial scope exists to keep workers out of "
         "it (proposed hook in RigExecBakedRunSteps)"},
        {"per-point geometry kernels (moverGraph.cpp parallel regions)",
         RigExecFrozenPurity::Pure,
         "range-independent per-point math; the per-context serial hook "
         "(D4) keeps frozen runs on the serial variants, which compute "
         "byte-identical numbers"},
        {"RigExecBakedProgramImpl structure (steps, edges, clusters, "
         "cones, walk, ladder tables)",
         RigExecFrozenPurity::EpochPinned,
         "immutable after Build; safe as the programDigest check names it, "
         "never as a live read of per-frame working state"},
        {"skin/blend/curvenet bindings (shared_ptr<const> topologies, "
         "layouts, profiles)",
         RigExecFrozenPurity::EpochPinned,
         "immutable snapshots resolved at Build/prologue; the worker runs "
         "from its own references, never from the live caches"},
        {"RigExecStaticInputCache VALUES",
         RigExecFrozenPurity::EpochPinned,
         "admission promises the same answer at every time code; safe as "
         "sampled values, while the cache OBJECT is the row below"},
        {"UsdStage, UsdAttribute, UsdAttributeQuery, UsdPrim, "
         "UsdGeomXformCache",
         RigExecFrozenPurity::LiveOnly,
         "every prologue read; sampled into the input vector on the UI "
         "thread, never named by the context, inputs, arena, or runner"},
        {"RigExecRigEvaluator (resolved inputs, live graphs, tap sets, "
         "overrides, profiler, epoch rests)",
         RigExecFrozenPurity::LiveOnly,
         "live state by definition; the program's captured pointers to it "
         "are why workers run a private arena, never the live program"},
        {"RigExecBakedProgramImpl live pointers (evaluator, stage, "
         "resolvedInputs, chainSnapshots, skinTopologies, "
         "blendSampleShapes, profiler, guideTaps)",
         RigExecFrozenPurity::LiveOnly,
         "read-what-the-evaluator-holds-now by design; a worker-owned "
         "program copy would still point at the live evaluator, so the "
         "region-only frozen entry is a program-side hook, not a copy"},
        {"ExecUsdSystem / TapSet / Snapshot (dynamic path)",
         RigExecFrozenPurity::LiveOnly,
         "OpenExec against the live stage; cannot run concurrently with "
         "stage edits, which is why refusal rigs take the D7 memo path"},
        {"curvenet binding LRU "
         "(curvenetWeightComputations.cpp, process-wide, mutex-guarded)",
         RigExecFrozenPurity::LiveOnly,
         "thread-safe and answer-preserving, but a lock held across "
         "allocation; bypassed because baked steps evaluate from "
         "program-held bindings and never call the memo"},
        {"wire-basis memo (moverGraph.cpp _CachedWireBasis, process-wide, "
         "mutex-guarded)",
         RigExecFrozenPurity::LiveOnly,
         "thread-safe and answer-preserving (full-input compare on hit), "
         "but a lock held across map insert; no baked step calls it, so "
         "frozen runs bypass it; a wire deformer that ever reaches a step "
         "makes its rig cache-ineligible until the program resolves bases "
         "in its UI-thread prologue"},
        {"RigExecStaticInputCache / RigExecSkinTopologyCache / "
         "RigExecBlendSampleCache OBJECTS",
         RigExecFrozenPurity::LiveOnly,
         "single-threaded or notice-invalidated live state (THREAD rule); "
         "workers use sampled values and held bindings, never the caches"},
        {"RigExecBakeReadRecorder",
         RigExecFrozenPurity::LiveOnly,
         "mutex plus per-run maps; a capture tool, never a worker input"},
        {"calibration/timing statics (bakedSchedule.cpp framesSeen)",
         RigExecFrozenPurity::LiveOnly,
         "unsynchronized diagnostic counters; frozen runs never enable "
         "calibration or step timing, so workers never touch them"},
        {"registry mutex / imaging bridge / snapshot store writes",
         RigExecFrozenPurity::LiveOnly,
         "Stream E's publish fence; background completions publish into "
         "the cache only, never into the snapshot, and never take the "
         "registry lock"},
    };
    return audit;
}

// ---------------------------------------------------------------------------
// The frozen executor.
// ---------------------------------------------------------------------------
//
// ISOLATION AUDIT (D3). A frozen run executes on a worker thread while the
// host may author on the UI thread, so no frozen code path may reach a
// UsdStage, UsdAttribute, UsdAttributeQuery, UsdPrim, the evaluator, its
// caches, or any live lock. The argument, per unit the run touches:
//
//  * The snapshot (RigExecFrozenProgram) is cloned on the UI thread. Its
//    program copy keeps the live program's USD handles COPIED but DEAD: the
//    worker copies them again (refcount operations, thread-safe) and never
//    dereferences one -- not even IsValid or GetPath, which reach composed
//    specs and prim data. Every handle identity the run needs (head paths,
//    query validity, attribute presence) is captured into the snapshot's
//    plain-data side-tables at freeze time.
//  * Constant patching (below) writes every varying input's sampled value
//    into its `constant` and nulls its head/query/resolvedAttr handles, so
//    RigExecBakedRead -- the only input route the reused step bodies take
//    (bakedPose.cpp's rd(), bakedProgramImpl.h:312) -- answers the patched
//    constant on every arm: the overridden arm's GetAttribute on a nulled
//    head is a safe no-op returning false with the value untouched, and the
//    query/resolved arms are skipped for invalid handles.
//  * Pose step bodies (RigExecBakedRunPoseStep, bakedPose.cpp:2524-3490)
//    touch no live state besides B.resolvedInputs (reference formation only
//    -- rd() never reaches it once patched) and B.resolveWeights, which is
//    called only for a constraint binding a weight object; the freeze gate
//    refuses any such constraint, and the snapshot nulls the function, so a
//    call would throw into the scheduler's fail-closed catch, never into
//    the evaluator.
//  * Geometry chunk/fuse/status bodies (bakedGeometry.cpp:2370-2718, minus
//    RevisionStatic/Derived) are pure functions of program slots plus the
//    shared kernels. RevisionStatic and Derived call the static,
//    stage-reading AssembleRevision, so the frozen run does NOT call them:
//    chain-revision packets are assembled on the UI thread by the REAL
//    RigExecAssembleSkinParameters and travel with the job, and the small
//    pure remainder of each body is replicated line-for-line below
//    (_FrozenRevisionStatic, _FrozenDerived, _AssembleDerivedPacket).
//  * RigExecBakedComputeClosure (bakedSchedule.cpp:1223) and
//    RigExecBakedSkipGeometryStep are pure program-state functions and run
//    unmodified. The region loop itself is a serial reimplementation of
//    RunStepsSerial (which is static and dispatches parallel by
//    environment); it never touches the parallel executor or its atomics.
//  * RigExecBakedPublishPose is stage-clean (audited: jointSolverBinding,
//    profiler, guide taps, resolved inputs only). The frozen run points the
//    first at the snapshot's copy, the profiler at a private disabled one
//    (its scopes take only that profiler's own uncontended mutex), the
//    guide taps at a null held privately (the bool test then skips), and
//    replays the guide publication itself from the aggregates.
//  * RigExecBakedPublishGeometry reads the stage UNCONDITIONALLY
//    (bakedGeometry.cpp:2733, B.stage->GetPrimAtPath for the adjuster
//    ladder), so it is replicated below (_FrozenPublishGeometry) for the
//    gated subset (no curvenet adjusters).
//  * Shared kernels (skin, derived, solvers, constraints) are pure per-point
//    math over worker-owned buffers, and their five WorkParallelForN launch
//    sites in moverGraph.cpp (the blend-channel sum among them, which the
//    frozen blend-shape assembler below calls) take the serial variant
//    inside a frozen run (the RigExecFrozenSerialActive hook, D4), so the
//    only thread a frozen frame ever runs on is its own.
//  * TfToken/SdfPath/VtValue copies on the worker touch only their own
//    atomic refcounts and the process-global immutable-after-load tables
//    under brief internal locks -- the same operations the live path
//    performs per frame, never the stage, and never held across evaluation.
//
// What the frozen run therefore reads: the immutable snapshot, the job's
// sampled vector, and its own working state. What it writes: its own
// working state and the output pose. Everything else declines.

// Visits every patchable input in one fixed order. The freeze uses it to
// capture head paths (UI thread, handles valid there) and the worker uses
// it to patch constants (side-table keys, no handle dereference); sharing
// the walker is what keeps the two in lockstep.
template <class Impl, class Fn>
void
_ForEachPatchableInput(Impl &B, Fn &&fn)
{
    for (auto &binding : B.avarBindings) {
        fn(binding.input);
    }
    for (auto &binding : B.avarConstantBindings) {
        fn(binding.input);
    }
    for (auto &ladder : B.ladders) {
        fn(ladder.restSpace);
        fn(ladder.defaultSpace);
        fn(ladder.posedSpace);
        for (int i = 0; i < 6; ++i) {
            fn(ladder.restAvars[i]);
            fn(ladder.defaultAvars[i]);
        }
        fn(ladder.rotationOrder);
    }
    for (auto &interpolator : B.poseInterpolators) {
        fn(interpolator.enabled);
    }
    for (auto &solver : B.solvers) {
        fn(solver.bend);
        fn(solver.upperOffset);
        fn(solver.lowerOffset);
        fn(solver.stretch);
        fn(solver.softness);
        fn(solver.blendWeight);
        fn(solver.preserveVolume);
        fn(solver.midFollowWeight);
        fn(solver.roll);
        fn(solver.twist);
        fn(solver.minLengthRatio);
        fn(solver.twistTurns);
        fn(solver.ribbonSampleCount);
    }
    for (auto &constraint : B.constraints) {
        fn(constraint.enabled);
        fn(constraint.defaultWeight);
        fn(constraint.offset);
        fn(constraint.affectX);
        fn(constraint.affectY);
        fn(constraint.affectZ);
        fn(constraint.tX);
        fn(constraint.tY);
        fn(constraint.tZ);
        fn(constraint.rX);
        fn(constraint.rY);
        fn(constraint.rZ);
        fn(constraint.sX);
        fn(constraint.sY);
        fn(constraint.sZ);
        fn(constraint.aimVector);
        fn(constraint.upVector);
        fn(constraint.rotationOffset);
        fn(constraint.worldUpVector);
        fn(constraint.poleVector);
        fn(constraint.twistDegrees);
    }
    for (auto &object : B.weightObjects) {
        _VisitWeightInputs(object, fn);
    }
}

// Memberwise program clone. RigExecBakedProgramImpl is movable but not
// copyable (clusterCounters' unique_ptr), so the copy is explicit, field by
// field, in struct order. Live pointers are nulled -- the worker repoints
// the ones it uses at worker-owned state -- and every other field is
// copied, including the per-frame working state (the history the frozen
// run branches from) and the USD handles (copied but dead; see the audit).
//
// When a field is added to RigExecBakedProgramImpl, it must be added here:
// a missing epoch field silently changes the frozen run's answers, and a
// missing per-frame field silently changes its history. The bit-identity
// test is the backstop, not the discipline.
void
_CloneImpl(const RigExecBakedProgramImpl &src, RigExecBakedProgramImpl *dst)
{
    RigExecBakedProgramImpl &D = *dst;
    D.evaluator = nullptr;
    D.stage = UsdStageRefPtr();
    D.assetRootPath = src.assetRootPath;
    D.resolvedInputs = nullptr;
    D.chainSnapshots = nullptr;
    D.skinTopologies = nullptr;
    D.blendSampleShapes = nullptr;
    D.resolveBlendSample = {};
    D.profiler = nullptr;
    D.interactiveOverrides = nullptr;
    D.jointSolverBinding = nullptr;
    D.guideTaps = nullptr;
    D.solverGuidesEnabled = nullptr;
    D.hasPropertyChains = src.hasPropertyChains;
    D.paths = src.paths;
    D.index = src.index;
    D.slotKind = src.slotKind;
    D.parent = src.parent;
    D.propParent = src.propParent;
    D.xformSlots = src.xformSlots;
    D.xformPrimsBySlot = src.xformPrimsBySlot;
    D.assetRoot = src.assetRoot;
    D.xformBase = src.xformBase;
    D.lastXformBase = src.lastXformBase;
    D.constraintArrays = src.constraintArrays;
    D.nativeSources = src.nativeSources;
    D.nativeFrames = src.nativeFrames;
    D.lastNativeFrames = src.lastNativeFrames;
    D.nativeFrameOk = src.nativeFrameOk;
    D.lastNativeFrameOk = src.lastNativeFrameOk;
    D.ladders = src.ladders;
    D.ladderVarying = src.ladderVarying;
    D.ladderOverrides = src.ladderOverrides;
    D.ladderDisturbed = src.ladderDisturbed;
    D.ladderMovedSlots = src.ladderMovedSlots;
    D.ladderRecomputed = src.ladderRecomputed;
    D.restChainVaries = src.restChainVaries;
    D.posedAuthored = src.posedAuthored;
    D.posedAuthoredM = src.posedAuthoredM;
    D.restM = src.restM;
    D.restPts = src.restPts;
    D.restFrames = src.restFrames;
    D.selfD = src.selfD;
    D.parentDinv = src.parentDinv;
    D.rotOrder = src.rotOrder;
    D.restRoundTrip = src.restRoundTrip;
    D.defaultRoundTrip = src.defaultRoundTrip;
    D.lastRestM = src.lastRestM;
    D.lastSelfD = src.lastSelfD;
    D.lastParentDinv = src.lastParentDinv;
    D.lastPosedAuthoredM = src.lastPosedAuthoredM;
    D.lastPosedAuthored = src.lastPosedAuthored;
    D.lastRotOrder = src.lastRotOrder;
    D.noScaleAvars = src.noScaleAvars;
    D.poseInterpolators = src.poseInterpolators;
    D.poseWeightPaths = src.poseWeightPaths;
    D.poseWeights = src.poseWeights;
    D.poseWeightIndex = src.poseWeightIndex;
    D.avarConstants = src.avarConstants;
    D.avarBindings = src.avarBindings;
    D.avarConstantBindings = src.avarConstantBindings;
    D.patchableAvars = src.patchableAvars;
    D.promotedAvars = src.promotedAvars;
    D.boundInputs = src.boundInputs;
    D.varyingInputs = src.varyingInputs;
    D.avars = src.avars;
    D.posedM = src.posedM;
    D.base = src.base;
    D.fin = src.fin;
    D.finLast = src.finLast;
    D.baseLast = src.baseLast;
    D.propertyResults = src.propertyResults;
    D.runSnapshots = src.runSnapshots;
    D.curvenetBindings = src.curvenetBindings;
    D.phasedReads = src.phasedReads;
    D.finalMatrix = src.finalMatrix;
    D.baseMatrix = src.baseMatrix;
    D.needFinal = src.needFinal;
    D.needBase = src.needBase;
    D.solvers = src.solvers;
    D.guideSolvers = src.guideSolvers;
    D.solverIndex = src.solverIndex;
    D.aggregates = src.aggregates;
    D.constraints = src.constraints;
    D.walkSteps = src.walkSteps;
    D.steps = src.steps;
    D.composeGroups = src.composeGroups;
    D.commits = src.commits;
    D.revisionIndex = src.revisionIndex;
    D.derivedIndex = src.derivedIndex;
    D.chainRevisionBegin = src.chainRevisionBegin;
    D.chainRevisionEnd = src.chainRevisionEnd;
    D.revisionChunkBase = src.revisionChunkBase;
    D.revisionChunkCount = src.revisionChunkCount;
    D.chainChunkBegin = src.chainChunkBegin;
    D.chainChunkEnd = src.chainChunkEnd;
    D.clustering = src.clustering;
    D.clusterCounters.reset();
    D.cones = src.cones;
    D.closedSteps = src.closedSteps;
    D.closed = src.closed;
    D.lastAvars = src.lastAvars;
    D.lastPropertyResults = src.lastPropertyResults;
    D.lastOverridden = src.lastOverridden;
    D.lastHaveBase = src.lastHaveBase;
    D.lastTime = src.lastTime;
    D.everRan = src.everRan;
    D.programStamp = src.programStamp;
    D.lastProgramStamp = src.lastProgramStamp;
    // Pending value edits travel with the stamps: a clone first run at the
    // source's lastTime -- a rewarm after a generation the live program did
    // not answer -- is the run that owes them, and time alone would not
    // dirty the steps they reached.
    D.edited = src.edited;
    D.anyEdited = src.anyEdited;
    D.valueEditSerial = src.valueEditSerial;
    D.editSerial = src.editSerial;
    D.lastClosedClusters = src.lastClosedClusters;
    D.lastClosedSteps = src.lastClosedSteps;
    D.solverOverrideRounds = src.solverOverrideRounds;
    D.solverEvaluations = src.solverEvaluations;
    D.jointSlots = src.jointSlots;
    D.jointPaths = src.jointPaths;
    D.controlSlots = src.controlSlots;
    D.controlPaths = src.controlPaths;
    D.solverArrays = src.solverArrays;
    D.jointPublishOrder = src.jointPublishOrder;
    D.controlPublishOrder = src.controlPublishOrder;
    D.solverPublishOrder = src.solverPublishOrder;
    D.jointPathsAscending = src.jointPathsAscending;
    D.controlPathsAscending = src.controlPathsAscending;
    D.solverArraysAscending = src.solverArraysAscending;
    D.timedPrologueUs = 0;
    D.timedRegionUs = 0;
    D.timedEpilogueUs = 0;
    D.timedFrames = 0;
    D.measurementSuspended = false;
    D.jointMatrixPublished = src.jointMatrixPublished;
    D.chains = src.chains;
    D.deltaValues = src.deltaValues;
    D.deltaPresent = src.deltaPresent;
    D.deltaBasePaths = src.deltaBasePaths;
    D.deltaBaseMatrix = src.deltaBaseMatrix;
    D.lastDeltaBaseMatrix = src.lastDeltaBaseMatrix;
    D.deltaBaseOk = src.deltaBaseOk;
    D.lastDeltaBaseOk = src.lastDeltaBaseOk;
    D.weightObjects = src.weightObjects;
    D.weightIndex = src.weightIndex;
    D.resolveWeights = {};
    D.volumeWeightMatrices = nullptr;
    D.updateVolumePlacements = {};
    D.currentPhaseWeights = src.currentPhaseWeights;
    D.falloffLuts = src.falloffLuts;
    // Sized but empty: every packet is rebuilt by its step during the run
    // (reads always follow writes in program order, as live), so copying
    // the previous run's dense fields per job would be pure waste.
    D.weightPackets.clear();
    D.weightPackets.resize(src.weightPackets.size());
    D.rebuild = src.rebuild;
    D.named = src.named;
    D.prims = src.prims;
    D.xformPrims = src.xformPrims;
    D.overridden = src.overridden;
    D.overridableInputs = src.overridableInputs;
    D.resolvedRoutedPrims = src.resolvedRoutedPrims;
    D.avarsDisturbed = src.avarsDisturbed;
    D.folded = src.folded;
    D.execTypedArrayInputs = src.execTypedArrayInputs;
    D.anyOverridden = src.anyOverridden;
    D.publishWeightFields = src.publishWeightFields;
}

bool
RigExecFreezeProgram(const RigExecRigEvaluator &evaluator,
                     std::shared_ptr<const RigExecFrozenProgram> *frozen,
                     std::string *error)
{
    const auto fail = [&error](const std::string &why) {
        if (error) {
            *error = why;
        }
        return false;
    };
    if (!frozen) {
        return fail("no snapshot to freeze into");
    }
    if (evaluator.cpuParityMode) {
        return fail("CPU parity mode runs the dynamic path, which no "
                    "snapshot can reproduce");
    }
    const RigExecBakedProgram *program = evaluator.GetBakedProgram();
    if (!program) {
        return fail("no baked program: dynamic/refusal rigs take the D7 "
                    "UI-thread memo path, never a background job");
    }
    const RigExecBakedProgramImpl &B = program->GetStepGraph();
    // Property chains are supported through the sampling hook -- except a
    // chain binding a weight object, whose envelope resolves through the
    // evaluator's live oracle. The discovery must also agree with the
    // program: a mismatch means the mover order and the epoch disagree, and
    // no snapshot is taken from a confused epoch.
    {
        RigExecChainSampleBindings bound;
        std::string bindError;
        if (!RigExecBindChainSampleInputs(evaluator, &bound, &bindError)) {
            return fail("cannot name the property chains: " + bindError);
        }
        if (bound.chains.empty() == B.hasPropertyChains) {
            return fail(
                "chain discovery disagrees with the program about whether "
                "chains exist");
        }
        for (const RigExecChainSampleChain &chain : bound.chains) {
            for (const RigExecChainSampleRevision &revision :
                 chain.revisions) {
                if (!revision.weightObjects.empty()) {
                    return fail(
                        "diag " + revision.moverPath.GetString() +
                        " binds a weight object, whose envelope the frozen "
                        "executor cannot reproduce");
                }
            }
        }
    }
    // No gate on xform slots, native sources, delta bases, or
    // geometry-domain constraints: the seeds sample per frame through the
    // program's hook, and the constraint steps are the shared bodies, so a
    // frozen run reproduces them from the transported seeds.
    // Weight objects and their steps run frozen now (sampled arrays +
    // patched scalars into the shared packet kernels); the remaining
    // oracle-dependent refusals are pose-domain constraints binding weight
    // objects, current-phase revisions, and property-chain diagnostics
    // binding weight objects, each gated where they are found. A
    // geometry-domain constraint never resolves its object -- weight stays
    // 1.0 and the revision's own weight packet scales per point
    // (bakedPose.cpp, the dynamic walk's gate alike) -- so the refusal
    // names exactly the step's oracle-call condition.
    for (const RigExecBakedProgramImpl::Constraint &constraint :
         B.constraints) {
        if (!constraint.weightObject.IsEmpty() &&
            constraint.pointsTarget.IsEmpty()) {
            return fail("constraint " + constraint.path.GetString() +
                        " binds a weight object, whose oracle resolves from "
                        "the live stage");
        }
    }
    if (B.ladderVarying) {
        return fail("a time-varying provider ladder recomposes from stage "
                    "reads the frozen executor cannot reproduce");
    }
    for (const RigExecBakedProgramImpl::GeomChain &chain : B.chains) {
        for (const RigExecBakedProgramImpl::GeomRevision &revision :
             chain.revisions) {
            if (revision.op != RigExecRevisionOp::Skin &&
                revision.op != RigExecRevisionOp::RecomputeNormals &&
                revision.op != RigExecRevisionOp::RecomputeExtent &&
                revision.op != RigExecRevisionOp::Matrix &&
                revision.op != RigExecRevisionOp::Wire &&
                revision.op != RigExecRevisionOp::BlendShape &&
                revision.op != RigExecRevisionOp::VolumeCorrect &&
                revision.op != RigExecRevisionOp::Smooth &&
                revision.op != RigExecRevisionOp::Lattice &&
                revision.op != RigExecRevisionOp::SurfaceProject &&
                revision.op != RigExecRevisionOp::Ribbon &&
                revision.op != RigExecRevisionOp::EmitGuidePoints &&
                revision.op != RigExecRevisionOp::Curvenet) {
                return fail("revision " +
                            revision.moverPath.GetString() + " runs op '" +
                            RigExecRevisionKindToken(revision.op)
                                .GetString() +
                            "', which the frozen executor does not "
                            "implement");
            }
            for (const RigExecBakedProgramImpl::GeomBlendChannel &channel :
                 revision.blendChannels) {
                for (const RigExecBakedProgramImpl::GeomBlendChannel::Sample
                         &sample : channel.samples) {
                    if (!sample.phase.IsBase()) {
                        return fail(
                            "revision " +
                            revision.moverPath.GetString() +
                            " reads a blend sample through a run snapshot, "
                            "which the frozen executor does not implement");
                    }
                }
            }
            // Read phases, snapshot-recording revisions and snapshot readers
            // all run frozen: the snapshot store fills from this run's own
            // steps in program order, and the static assembly builds the
            // same per-revision overlay live builds. (Blend-sample phases
            // keep their own refusal above.)
            if (revision.weightCurrentPhase) {
                return fail("revision " +
                            revision.moverPath.GetString() +
                            " measures its weight field against the current "
                            "phase, which resolves through the live oracle");
            }
            // Driver frames, the curvenet net chain, and the
            // geometry-delta hand-off are implemented: the worker's own
            // Solve-step aggregate and net-chain result, read in program
            // order, and the delta through the shared constraint step and
            // the shared fold -- the seeds patch the delta bases the
            // constraint step measures against, the step stashes the delta,
            // and FoldInfluences reads the stash, all shared bodies in
            // program order.
            if (revision.op == RigExecRevisionOp::Skin &&
                !revision.skinTopologyFixed) {
                return fail("revision " +
                            revision.moverPath.GetString() +
                            " has an unfixed skin layout, which the packet "
                            "assembly reads per frame off the stage");
            }
        }
        for (const RigExecBakedProgramImpl::GeomChain::Derived &derived :
             chain.derived) {
            if (derived.revision.op != RigExecRevisionOp::RecomputeNormals &&
                derived.revision.op != RigExecRevisionOp::RecomputeExtent) {
                return fail("derived target " +
                            derived.target.GetString() +
                            " runs an op the frozen executor does not "
                            "implement");
            }
            if (!derived.revision.binding.phases.empty()) {
                return fail("derived target " +
                            derived.target.GetString() +
                            " declares a read phase, which the frozen "
                            "executor does not implement");
            }
        }
    }

    auto snapshot = std::make_shared<RigExecFrozenProgram>();
    _CloneImpl(B, &snapshot->program);
    if (B.jointSolverBinding) {
        snapshot->jointSolverBinding = *B.jointSolverBinding;
    }
    snapshot->guideTapsPresent =
        B.guideTaps != nullptr && B.guideTaps->get() != nullptr;
    // Handle identities, captured here (UI thread) as plain data: the
    // worker must not even ask a handle whether it is valid.
    _ForEachPatchableInput(
        B, [&snapshot](const auto &input) {
            snapshot->inputHeadPaths.push_back(
                input.head ? input.head.GetPath() : SdfPath());
        });
    for (const RigExecBakedProgramImpl::ConstraintArrays &arrays :
         B.constraintArrays) {
        const SdfPath primPath =
            arrays.prim ? arrays.prim.GetPath() : SdfPath();
        snapshot->arrayKeys.push_back(
            primPath.IsEmpty()
                ? SdfPath()
                : primPath.AppendProperty(TfToken("inputs:sourceWeights")));
        snapshot->arrayKeys.push_back(
            (primPath.IsEmpty() || !arrays.parentOffsets)
                ? SdfPath()
                : primPath.AppendProperty(
                      TfToken("inputs:translationOffsets")));
        snapshot->arrayKeys.push_back(
            (primPath.IsEmpty() || !arrays.parentOffsets)
                ? SdfPath()
                : primPath.AppendProperty(TfToken("inputs:rotationOffsets")));
        snapshot->arrayKeys.push_back(
            (primPath.IsEmpty() || !arrays.readPole)
                ? SdfPath()
                : primPath.AppendProperty(
                      TfToken("inputs:poleVectorWeights")));
    }
    for (const RigExecBakedProgramImpl::GeomChain &chain : B.chains) {
        snapshot->chainBaseQueryValid.push_back(
            chain.baseQuery.IsValid() ? 1 : 0);
    }
    for (const auto &[chainIndex, derivedIndex] : B.derivedIndex) {
        const RigExecBakedProgramImpl::GeomChain::Derived &derived =
            B.chains[size_t(chainIndex)].derived[size_t(derivedIndex)];
        snapshot->derivedBaseQueryValid.push_back(
            derived.baseQuery.IsValid() ? 1 : 0);
    }
    for (const RigExecBakedProgramImpl::Solver &solver : B.solvers) {
        snapshot->ribbonQueryValid.push_back(
            solver.ribbonPointsQuery.IsValid() ? 1 : 0);
    }
    for (const auto &[chainIndex, revisionIndex] : B.revisionIndex) {
        const RigExecBakedProgramImpl::GeomRevision &revision =
            B.chains[size_t(chainIndex)].revisions[size_t(revisionIndex)];
        const UsdPrim &moverPrim = revision.moverPrim;
        snapshot->moverHasEnabled.push_back(
            moverPrim && moverPrim.GetAttribute(TfToken("inputs:enabled"))
                ? 1
                : 0);
        snapshot->moverHasDefaultWeight.push_back(
            moverPrim &&
                    moverPrim.GetAttribute(TfToken("inputs:defaultWeight"))
                ? 1
                : 0);
        snapshot->moverHasMethod.push_back(
            moverPrim &&
                    moverPrim.GetAttribute(TfToken("rigExec:skinningMethod"))
                ? 1
                : 0);
    }
    *frozen = std::move(snapshot);
    return true;
}

bool
RigExecFrozenSnapshotOwesLiveEdits(const RigExecFrozenProgram &base,
                                   const RigExecBakedProgram &live)
{
    const RigExecBakedProgramImpl &L = live.GetStepGraph();
    return L.valueEditSerial != base.program.valueEditSerial ||
           L.programStamp != base.program.programStamp;
}

uint64_t
RigExecFrozenAvarRegionDigest(const RigExecBakedProgram &program)
{
    // Exactly the fields RigExecProgramAvarPatch writes, plus the table it
    // keeps beside them: per-binding constants and varying flags, the
    // promoted set, the varying count, and the constant table. Working
    // state (avars, lastAvars) is history, not region, and is excluded --
    // it converges on the next run whatever the snapshot holds.
    const RigExecBakedProgramImpl &B = program.GetStepGraph();
    uint64_t h = 1469598103934665603ULL;
    h = _HashBytes(h, &B.varyingInputs, sizeof(B.varyingInputs));
    for (double constant : B.avarConstants) {
        h = _HashBytes(h, &constant, sizeof(constant));
    }
    for (const RigExecBakedProgramImpl::AvarBinding &binding :
         B.avarConstantBindings) {
        h = _HashBytes(h, &binding.input.constant,
                       sizeof(binding.input.constant));
        const unsigned char varying = binding.input.varying ? 1 : 0;
        h = _HashBytes(h, &varying, sizeof(varying));
    }
    for (size_t promoted : B.promotedAvars) {
        h = _HashBytes(h, &promoted, sizeof(promoted));
    }
    return h;
}

uint64_t
RigExecFrameCacheEpochDigest(const RigExecRigEvaluator &evaluator)
{
    uint64_t h = evaluator.GetBindingEpochDigest();
    const RigExecBakedProgram *program = evaluator.GetBakedProgram();
    if (!program) {
        return h;
    }
    const uint64_t builds = uint64_t(evaluator.GetBakedProgramBuildCount());
    h = _HashBytes(h, &builds, sizeof(builds));
    // Structure only (plan D3): the binding-table shapes, never the
    // constant values. A value patch moves the constant digest the
    // control half folds -- not this -- so it opens a new key namespace
    // instead of evicting the epoch.
    const RigExecBakedProgramImpl &B = program->GetStepGraph();
    const uint64_t constants = uint64_t(B.avarConstants.size());
    h = _HashBytes(h, &constants, sizeof(constants));
    const uint64_t bindings = uint64_t(B.avarConstantBindings.size());
    h = _HashBytes(h, &bindings, sizeof(bindings));
    for (const RigExecBakedProgramImpl::AvarBinding &binding :
         B.avarConstantBindings) {
        const uint64_t slot = uint64_t(binding.slot);
        h = _HashBytes(h, &slot, sizeof(slot));
    }
    const uint64_t varying = uint64_t(B.avarBindings.size());
    h = _HashBytes(h, &varying, sizeof(varying));
    const uint64_t table = uint64_t(B.avars.size());
    h = _HashBytes(h, &table, sizeof(table));
    return h;
}

uint64_t
RigExecEpochConstantDigest(const RigExecBakedProgram &program)
{
    return RigExecFrozenAvarRegionDigest(program);
}

std::vector<uint64_t>
RigExecConstantRegionsForPaths(const RigExecBakedProgram &program,
                               const std::vector<SdfPath> &paths)
{
    const RigExecBakedProgramImpl &B = program.GetStepGraph();
    std::vector<uint64_t> regions;
    for (const SdfPath &path : paths) {
        const auto found = B.patchableAvars.find(path);
        if (found == B.patchableAvars.end()) {
            continue;
        }
        const uint64_t region =
            RigExecConstantRegionForBinding(found->second);
        if (regions.empty() || regions.back() != region) {
            // patchableAvars iterates in path order, not binding order,
            // so contiguity is not guaranteed; the sort below fixes it.
            regions.push_back(region);
        }
    }
    std::sort(regions.begin(), regions.end());
    regions.erase(std::unique(regions.begin(), regions.end()),
                  regions.end());
    return regions;
}

bool
RigExecPatchFrozenAvarConstants(const RigExecFrozenProgram &base,
                                const RigExecBakedProgram &live,
                                std::shared_ptr<const RigExecFrozenProgram> *out,
                                std::string *error)
{
    const auto fail = [&error](const std::string &why) {
        if (error) {
            *error = why;
        }
        return false;
    };
    if (!out) {
        return fail("no snapshot to patch into");
    }
    const RigExecBakedProgramImpl &L = live.GetStepGraph();
    const RigExecBakedProgramImpl &S = base.program;
    // The same program object, or at least the same shape: a rebuild is
    // re-frozen, never patched.
    if (S.avarConstantBindings.size() != L.avarConstantBindings.size() ||
        S.avarConstants.size() != L.avarConstants.size()) {
        return fail("avar region changed shape: re-freeze, do not patch");
    }
    auto snapshot = std::make_shared<RigExecFrozenProgram>();
    _CloneImpl(S, &snapshot->program);
    RigExecBakedProgramImpl &P = snapshot->program;
    // Every field the live patch writes, carried onto the copy. The
    // consumed table for constant slots comes along too: no run recomputes
    // those slots (only a drag walks them), so the patch's write into
    // avars is the value, not history -- without it the copy would warm
    // the pre-edit constant. lastAvars is untouched on both sides, so the
    // next run's cones include the changed slots alike. Copied query and
    // resolved handles are dead by construction; the worker nulls them as
    // it patches.
    if (P.avars.size() != L.avars.size()) {
        return fail("avar table changed shape: re-freeze, do not patch");
    }
    for (size_t i = 0; i < L.avarConstantBindings.size(); ++i) {
        P.avarConstantBindings[i].input.constant =
            L.avarConstantBindings[i].input.constant;
        P.avarConstantBindings[i].input.varying =
            L.avarConstantBindings[i].input.varying;
        P.avarConstantBindings[i].input.query =
            L.avarConstantBindings[i].input.query;
        P.avarConstantBindings[i].input.resolvedAttr =
            L.avarConstantBindings[i].input.resolvedAttr;
        const size_t slot = L.avarConstantBindings[i].slot;
        if (slot >= P.avars.size()) {
            return fail("avar slot out of range: re-freeze, do not patch");
        }
        P.avars[slot] = L.avars[slot];
    }
    P.promotedAvars = L.promotedAvars;
    P.varyingInputs = L.varyingInputs;
    P.avarConstants = L.avarConstants;
    // And every value edit routed since the snapshot was taken or last
    // patched, added to the copy's own pending ones: the copy's first run
    // owes them all. Asked of the per-index edit counts, not of the live
    // program's pending flags, because a live run consumes those -- an edit
    // the live program has already answered is one the snapshot's history
    // has still never seen.
    if (L.valueEditSerial != S.valueEditSerial) {
        P.edited.resize(std::max(P.edited.size(), L.editSerial.size()), 0);
        for (size_t i = 0; i < L.editSerial.size(); ++i) {
            if (L.editSerial[i] > S.valueEditSerial) {
                P.edited[i] = 1;
                P.anyEdited = true;
            }
        }
        P.valueEditSerial = L.valueEditSerial;
        P.editSerial = L.editSerial;
    }
    // And a stamp bumped since: a notice the index could not place, which
    // the live program answered by running whole once. The copy owes the
    // same run -- its stamp moves past the one its last run recorded.
    if (L.programStamp != S.programStamp) {
        P.programStamp = L.programStamp;
    }
    // The epoch's side-tables, unchanged by a constant patch.
    snapshot->jointSolverBinding = base.jointSolverBinding;
    snapshot->guideTapsPresent = base.guideTapsPresent;
    snapshot->inputHeadPaths = base.inputHeadPaths;
    snapshot->arrayKeys = base.arrayKeys;
    snapshot->chainBaseQueryValid = base.chainBaseQueryValid;
    snapshot->derivedBaseQueryValid = base.derivedBaseQueryValid;
    snapshot->ribbonQueryValid = base.ribbonQueryValid;
    snapshot->moverHasEnabled = base.moverHasEnabled;
    snapshot->moverHasDefaultWeight = base.moverHasDefaultWeight;
    snapshot->moverHasMethod = base.moverHasMethod;
    *out = std::move(snapshot);
    return true;
}

// ---------------------------------------------------------------------------
// Worker-side state and input patching.
// ---------------------------------------------------------------------------

// One job's private working state: the snapshot cloned onto the worker plus
// the live-state stand-ins the reused code dereferences (resolved inputs,
// snapshot stores, profiler, guide taps, guides flag). Nothing in here is
// shared between jobs; nothing in here names the stage or the evaluator.
struct _FrozenWorker {
    RigExecBakedProgramImpl B;
    RigExecResolvedInputs resolved;
    RigExecChainSnapshots chainSnapshots;
    RigExecProfiler profiler;
    bool guidesEnabled = false;
    std::unique_ptr<RigExecTapSet> nullTaps;
    // The worker's own placement map: the snapshot nulls the evaluator's
    // volumeWeightMatrices pointer, so the frozen VolumePlacements body
    // writes here, and B.volumeWeightMatrices points at it for the packet
    // build and pose publish to read.
    std::map<SdfPath, GfMatrix4d> volumeWeightMatrices;
};

template <class T>
bool
_SampleHolds(const VtValue &held, T *out)
{
    if (held.IsHolding<T>()) {
        *out = held.UncheckedGet<T>();
        return true;
    }
    return false;
}

// The one sanctioned cross-type read: a float input over a double source,
// which the frame path coerces at consumption (GetAttribute's float arm and
// _CoerceFromDouble). Every other pair must hold exactly.
template <>
bool
_SampleHolds<float>(const VtValue &held, float *out)
{
    if (held.IsHolding<float>()) {
        *out = held.UncheckedGet<float>();
        return true;
    }
    if (held.IsHolding<double>()) {
        *out = float(held.UncheckedGet<double>());
        return true;
    }
    return false;
}

// Patches every patchable input's constant from its head-keyed sample and
// nulls its handles, so the reused step bodies answer the patched constant
// on every arm of RigExecBakedRead. A varying input with no sample, or a
// sample holding a type the typed read cannot consume, declines the job:
// the snapshot and the vector describe different programs.
bool
_PatchInputs(RigExecBakedProgramImpl &B,
             const RigExecFrozenProgram &snapshot,
             const std::map<SdfPath, size_t> &index,
             const RigExecFrameInputs &inputs)
{
    size_t walked = 0;
    bool ok = true;
    _ForEachPatchableInput(B, [&](auto &input) {
        if (!ok) {
            return;
        }
        if (walked >= snapshot.inputHeadPaths.size()) {
            ok = false;
            return;
        }
        const SdfPath &key = snapshot.inputHeadPaths[walked++];
        if (key.IsEmpty()) {
            // Invalid head: the sampler emits nothing, and live answers the
            // constant on every arm. A varying input the sampler cannot key
            // is a shape it cannot reproduce.
            if (input.varying) {
                ok = false;
            }
            return;
        }
        const auto found = index.find(key);
        if (found == index.end()) {
            // No sample: sound only when the input is not varying (live
            // reads the constant). A varying input with no sample means the
            // vector was sampled from a different program than the snapshot
            // (stale snapshot) -- decline.
            if (input.varying) {
                ok = false;
            }
            return;
        }
        const RigExecSampledInput &sample = inputs.values[found->second];
        if (sample.viaChain) {
            // Stale chain output: the hook declined this rig, so the runner
            // never evaluates one.
            ok = false;
            return;
        }
        if (sample.hasValue && !_SampleHolds(sample.value, &input.constant)) {
            ok = false;
            return;
        }
        // A valueless sample is a stage read that failed at sample time;
        // live's typed read fails the same way and keeps the constant.
        input.head = UsdAttribute();
        input.query = UsdAttributeQuery();
        input.resolvedAttr = UsdAttribute();
    });
    return ok && walked == snapshot.inputHeadPaths.size();
}

// The frozen prologue: RigExecBakedProgram::Run's prologue (bakedProgram.cpp)
// plus RunInputs, RunSolverSources, the constraint-array sweep, and the
// geometry prologue, with every stage or live-state read replaced by its
// sampled value. Reads the snapshot, the job's vector, and the worker's own
// program copy; writes only the copy (and the pose's prologue counters).
// Anything it cannot reproduce declines the job.
bool
_FrozenPrologue(_FrozenWorker *worker, const RigExecFrozenProgram &snapshot,
               const RigExecFrameInputs &inputs,
               const std::map<SdfPath, size_t> &index, UsdTimeCode time,
               RigExecRigPose *pose)
{
    RigExecBakedProgramImpl &B = worker->B;
    const RigExecResolvedInputs &R = *B.resolvedInputs;
    const auto findSample = [&index, &inputs](const SdfPath &key) {
        const auto found = index.find(key);
        return found == index.end() ? nullptr : &inputs.values[found->second];
    };

    // Chains, between the two override placements, as on the live path:
    // the sampler evaluated the hook for the job's time on the UI thread,
    // and the per-target results travel with the vector. The worker
    // publishes them where the live prologue publishes the chains it ran.
    // A chainless snapshot with transported results is a stale vector from
    // another epoch and declines.
    if (!B.hasPropertyChains && !inputs.chainResults.empty()) {
        return false;
    }
    B.propertyResults.clear();
    B.resolvedInputs->Clear();
    B.runSnapshots.Clear();
    B.chainSnapshots->Clear();
    // Pre- and post-chain override placement, replicated from
    // _ApplyInteractiveOverridesToResolved: every attribute override stands
    // in the resolved inputs, and one standing on a chain result replaces
    // it. The resolved inputs are unread on the worker (patched constants),
    // but the placement keeps the private state shape-identical.
    for (const RigExecValueOverride &o : inputs.overrides) {
        if (o.attribute.IsEmpty()) {
            continue;
        }
        B.resolvedInputs->SetProperty(o.prim.AppendProperty(o.attribute),
                                      o.value);
    }
    for (const auto &[target, value] : inputs.chainResults) {
        B.propertyResults[target] = value;
        B.resolvedInputs->SetProperty(target, value);
    }
    for (const RigExecValueOverride &o : inputs.overrides) {
        if (o.attribute.IsEmpty()) {
            continue;
        }
        const SdfPath path = o.prim.AppendProperty(o.attribute);
        B.resolvedInputs->SetProperty(path, o.value);
        const auto found = B.propertyResults.find(path);
        if (found != B.propertyResults.end()) {
            found->second = o.value;
        }
    }

    // RunInputs (bakedPose.cpp:2099): ladders recompose from stage reads, so
    // a recompute declines; everything else replays samples through the real
    // RigExecBakedRead, which answers the patched constants.
    bool ladderDragged = false;
    if (B.anyOverridden) {
        for (const int ladderIndex : B.ladderOverrides) {
            if (B.overridden[size_t(ladderIndex)]) {
                ladderDragged = true;
                break;
            }
        }
    }
    B.ladderRecomputed =
        B.ladderVarying || ladderDragged || B.ladderDisturbed;
    if (B.ladderRecomputed) {
        return false;
    }
    if (!B.ladderMovedSlots.empty()) {
        B.ladderMovedSlots.clear();
    }
    for (const auto &binding : B.avarBindings) {
        B.avars[binding.slot] =
            RigExecBakedRead(binding.input, R, time, &B.overridden);
    }
    for (const size_t promoted : B.promotedAvars) {
        if (promoted >= B.avarConstantBindings.size()) {
            return false;
        }
        const auto &binding = B.avarConstantBindings[promoted];
        B.avars[binding.slot] =
            RigExecBakedRead(binding.input, R, time, &B.overridden);
    }
    if (B.anyOverridden || B.avarsDisturbed) {
        for (const auto &binding : B.avarConstantBindings) {
            B.avars[binding.slot] =
                (B.overridden[size_t(binding.input.overrideIndex)] ||
                 binding.input.varying)
                    ? RigExecBakedRead(binding.input, R, time, &B.overridden)
                    : binding.input.constant;
        }
        B.avarsDisturbed = B.anyOverridden;
    }
    for (RigExecBakedProgramImpl::PoseInterpolator &interpolator :
         B.poseInterpolators) {
        interpolator.enabledValue =
            RigExecBakedRead(interpolator.enabled, R, time, &B.overridden);
    }

    // RunSolverSources (bakedPose.cpp:2156): ribbon points replay from the
    // sampled attribute, with the same swap-and-compare.
    for (size_t si = 0; si < B.solvers.size(); ++si) {
        RigExecBakedProgramImpl::Solver &solver = B.solvers[si];
        if (!solver.ribbonPointsVarying ||
            si >= snapshot.ribbonQueryValid.size() ||
            !snapshot.ribbonQueryValid[si]) {
            continue;
        }
        const RigExecSampledInput *sample =
            findSample(solver.ribbonPointsPath);
        if (!sample) {
            return false;
        }
        VtVec3fArray live;
        if (sample->hasValue) {
            if (!sample->value.IsHolding<VtVec3fArray>()) {
                return false;
            }
            live = sample->value.UncheckedGet<VtVec3fArray>();
        }
        solver.lastRibbonPoints.swap(solver.ribbonPoints);
        solver.ribbonPoints.assign(live.begin(), live.end());
        solver.ribbonPointsDirty =
            solver.ribbonPoints != solver.lastRibbonPoints;
    }

    // The stage-frame prologue (bakedProgram.cpp: Run's stageFrames): the
    // sampler read the same slots and paths at the job's time through the
    // program's hook, and the worker publishes the seeds where the live
    // prologue publishes its stage reads -- the bases and the slots' FIRST
    // versions for xform slots, the ok/matrix pairs for delta bases, the
    // ok/frame pairs for native sources. The vectors parallel the program
    // tables; a size mismatch is a vector from another epoch and declines.
    // The cone compares below are the shared ones, against the frozen
    // lasts, so a seed that moved since freeze dirties what live would
    // dirty against the run before -- possibly more, never less -- and the
    // steps are pure functions of these inputs, so freeze-time memos serve
    // wherever the seeds still match freeze.
    if (inputs.stageSeeds.xformBase.size() != B.xformSlots.size() ||
        inputs.stageSeeds.xformFrames.size() != B.xformSlots.size() ||
        inputs.stageSeeds.deltaOk.size() != B.deltaBasePaths.size() ||
        inputs.stageSeeds.deltaBase.size() != B.deltaBasePaths.size() ||
        inputs.stageSeeds.nativeOk.size() != B.nativeSources.size() ||
        inputs.stageSeeds.nativeFrames.size() != B.nativeSources.size()) {
        return false;
    }
    for (size_t k = 0; k < B.xformSlots.size(); ++k) {
        const size_t slot = size_t(B.xformSlots[k]);
        B.xformBase[k] = inputs.stageSeeds.xformBase[k];
        B.base[slot] = inputs.stageSeeds.xformFrames[k];
        B.fin[slot] = inputs.stageSeeds.xformFrames[k];
    }
    for (size_t k = 0; k < B.deltaBasePaths.size(); ++k) {
        B.deltaBaseOk[k] = inputs.stageSeeds.deltaOk[k];
        B.deltaBaseMatrix[k] = inputs.stageSeeds.deltaBase[k];
    }
    for (size_t k = 0; k < B.nativeSources.size(); ++k) {
        B.nativeFrameOk[k] = inputs.stageSeeds.nativeOk[k];
        B.nativeFrames[k] = inputs.stageSeeds.nativeFrames[k];
    }

    // Constraint operator arrays (bakedProgram.cpp:2467): replayed from the
    // sampled raw attributes with the same cardinality validation, the same
    // neutral fills, and the same diagnostic lines as
    // _ReadConstraintSourceWeights/_ReadConstraintSourceOffsets. A missing
    // or valueless sample is an absent or unreadable attribute -- neutral,
    // exactly as live -- while a mistyped holding declines.
    for (size_t k = 0; k < B.constraintArrays.size(); ++k) {
        RigExecBakedProgramImpl::ConstraintArrays &arrays =
            B.constraintArrays[k];
        if (snapshot.arrayKeys.size() < k * 4 + 4) {
            return false;
        }
        arrays.diagnostics.clear();
        const SdfPath &weightsKey = snapshot.arrayKeys[k * 4];
        VtFloatArray authoredWeights;
        if (!weightsKey.IsEmpty()) {
            if (const RigExecSampledInput *sample = findSample(weightsKey)) {
                if (sample->hasValue) {
                    if (!sample->value.IsHolding<VtFloatArray>()) {
                        return false;
                    }
                    authoredWeights =
                        sample->value.UncheckedGet<VtFloatArray>();
                }
            }
        }
        arrays.ok = true;
        if (!authoredWeights.empty() &&
            authoredWeights.size() != arrays.sourceCount) {
            arrays.diagnostics.push_back(
                weightsKey.GetPrimPath().GetString() +
                " inputs:sourceWeights has " +
                std::to_string(authoredWeights.size()) + " entries for " +
                std::to_string(arrays.sourceCount) + " sources");
            arrays.ok = false;
        } else {
            arrays.weights.assign(arrays.sourceCount, 1.0);
            for (size_t i = 0;
                 i < authoredWeights.size() && i < arrays.weights.size();
                 ++i) {
                arrays.weights[i] = double(authoredWeights[i]);
            }
        }
        bool mistyped = false;
        const auto readOffsets =
            [&](const SdfPath &key, const char *name,
                std::vector<GfVec3d> *offsets) -> bool {
            VtVec3dArray authored;
            if (!key.IsEmpty()) {
                if (const RigExecSampledInput *sample = findSample(key)) {
                    if (sample->hasValue) {
                        if (!sample->value.IsHolding<VtVec3dArray>()) {
                            mistyped = true;
                            return false;
                        }
                        authored =
                            sample->value.UncheckedGet<VtVec3dArray>();
                    }
                }
            }
            if (!authored.empty() &&
                authored.size() != arrays.sourceCount) {
                arrays.diagnostics.push_back(
                    key.GetPrimPath().GetString() + " " + name + " has " +
                    std::to_string(authored.size()) + " entries for " +
                    std::to_string(arrays.sourceCount) + " sources");
                return false;
            }
            offsets->assign(arrays.sourceCount, GfVec3d(0));
            for (size_t i = 0;
                 i < authored.size() && i < offsets->size(); ++i) {
                (*offsets)[i] = authored[i];
            }
            return true;
        };
        if (arrays.parentOffsets) {
            arrays.ok =
                arrays.ok &&
                readOffsets(snapshot.arrayKeys[k * 4 + 1],
                            "inputs:translationOffsets",
                            &arrays.translationOffsets) &&
                readOffsets(snapshot.arrayKeys[k * 4 + 2],
                            "inputs:rotationOffsets", &arrays.rotationOffsets);
            if (mistyped) {
                // A mistyped holding, not a cardinality line: decline rather
                // than serve a half-validated table.
                return false;
            }
        } else {
            arrays.translationOffsets.assign(arrays.sourceCount, GfVec3d(0));
            arrays.rotationOffsets.assign(arrays.sourceCount, GfVec3d(0));
        }
        if (arrays.readPole) {
            arrays.poleDiagnostics.clear();
            const SdfPath &poleKey = snapshot.arrayKeys[k * 4 + 3];
            VtFloatArray authoredPole;
            if (!poleKey.IsEmpty()) {
                if (const RigExecSampledInput *sample = findSample(poleKey)) {
                    if (sample->hasValue) {
                        if (!sample->value.IsHolding<VtFloatArray>()) {
                            return false;
                        }
                        authoredPole =
                            sample->value.UncheckedGet<VtFloatArray>();
                    }
                }
            }
            arrays.poleOk = true;
            if (!authoredPole.empty() &&
                authoredPole.size() != arrays.poleCount) {
                arrays.poleDiagnostics.push_back(
                    poleKey.GetPrimPath().GetString() +
                    " inputs:poleVectorWeights has " +
                    std::to_string(authoredPole.size()) + " entries for " +
                    std::to_string(arrays.poleCount) + " sources");
                arrays.poleOk = false;
            } else {
                arrays.poleWeights.assign(arrays.poleCount, 1.0);
                for (size_t i = 0;
                     i < authoredPole.size() && i < arrays.poleWeights.size();
                     ++i) {
                    arrays.poleWeights[i] = double(authoredPole[i]);
                }
            }
        }
    }

    // Geometry prologue (bakedGeometry.cpp:1759): base reads replay from the
    // sampled queries, with the same reset/count/swap/compare sequence; the
    // topology resolve replays the transported packet's layout with the same
    // re-cut rule; blend and curvenet resolves are refused at freeze.
    const auto resetRevision =
        [](RigExecBakedProgramImpl::GeomRevision *revision) {
        revision->created = true;
        revision->ran = false;
        revision->output.clear();
        revision->currentSource = -1;
        revision->lastParameters = RigExecMoverParameters();
        revision->lastAuxPoints = VtVec3fArray();
        revision->lastStatus = RigExecMoverStatus();
    };
    for (size_t ci = 0; ci < B.chains.size(); ++ci) {
        RigExecBakedProgramImpl::GeomChain &chain = B.chains[ci];
        if (ci >= snapshot.chainBaseQueryValid.size()) {
            return false;
        }
        VtVec3fArray basePoints;
        bool haveBase = false;
        if (snapshot.chainBaseQueryValid[ci]) {
            const RigExecSampledInput *sample = findSample(chain.target);
            if (!sample) {
                return false;
            }
            if (sample->hasValue) {
                if (!sample->value.IsHolding<VtVec3fArray>()) {
                    return false;
                }
                basePoints = sample->value.UncheckedGet<VtVec3fArray>();
                haveBase = true;
            }
        }
        chain.haveBase = haveBase;
        if (!chain.haveBase) {
            for (RigExecBakedProgramImpl::GeomChain::Derived &derived :
                 chain.derived) {
                derived.haveBase = false;
            }
            continue;
        }
        if (chain.haveResult && basePoints.size() != chain.lastBase.size()) {
            chain.haveResult = false;
            chain.result = VtVec3fArray();
            chain.scheduleDirty = true;
            for (RigExecBakedProgramImpl::GeomRevision &revision :
                 chain.revisions) {
                resetRevision(&revision);
            }
        }
        for (RigExecBakedProgramImpl::GeomRevision &revision :
             chain.revisions) {
            if (revision.created) {
                ++pose->moverGraphRevisionsCreated;
                revision.created = false;
            }
        }
        if (chain.scheduleDirty && !chain.revisions.empty()) {
            ++pose->moverGraphSchedulesBuilt;
        }
        chain.scheduleDirty = false;
        chain.baseDirty = !chain.haveResult || basePoints != chain.lastBase;
        chain.lastBase = basePoints;
        for (RigExecBakedProgramImpl::GeomRevision &revision :
             chain.revisions) {
            if (revision.op == RigExecRevisionOp::CurvenetAdjuster) {
                return false;
            }
        }
        for (RigExecBakedProgramImpl::GeomChain::Derived &derived :
             chain.derived) {
            // Derived bases key by target like chain bases; validity rides
            // the derivedIndex-parallel side-table.
            const RigExecSampledInput *sample = findSample(derived.target);
            VtVec3fArray derivedBase;
            bool haveDerivedBase = false;
            // The side-table index: position of (ci, di) in derivedIndex.
            size_t derivedKey = B.derivedIndex.size();
            for (size_t dk = 0; dk < B.derivedIndex.size(); ++dk) {
                if (B.derivedIndex[dk].first == int(ci) &&
                    &B.chains[size_t(B.derivedIndex[dk].first)]
                             .derived[size_t(B.derivedIndex[dk].second)] ==
                        &derived) {
                    derivedKey = dk;
                    break;
                }
            }
            if (derivedKey >= snapshot.derivedBaseQueryValid.size()) {
                return false;
            }
            if (snapshot.derivedBaseQueryValid[derivedKey]) {
                if (!sample) {
                    return false;
                }
                if (sample->hasValue) {
                    if (!sample->value.IsHolding<VtVec3fArray>()) {
                        return false;
                    }
                    derivedBase =
                        sample->value.UncheckedGet<VtVec3fArray>();
                    haveDerivedBase = true;
                }
            }
            derived.haveBase = haveDerivedBase;
            if (!derived.haveBase) {
                continue;
            }
            if (derived.haveResult &&
                derivedBase.size() != derived.lastBase.size()) {
                derived.haveResult = false;
                derived.result = VtVec3fArray();
                resetRevision(&derived.revision);
            }
            if (derived.revision.created) {
                ++pose->moverGraphRevisionsCreated;
                ++pose->moverGraphSchedulesBuilt;
                derived.revision.created = false;
            }
            derived.baseDirty =
                !derived.haveResult || derivedBase != derived.lastBase;
            derived.lastBase = derivedBase;
        }
    }
    // The topology resolve, after the bases (it reads the point count):
    // the transported packet's layout, with the prologue's re-cut rule.
    if (inputs.revisionPackets.size() != B.revisionIndex.size()) {
        return false;
    }
    for (size_t r = 0; r < B.revisionIndex.size(); ++r) {
        const auto &[chainIndex, revisionIndex] = B.revisionIndex[r];
        RigExecBakedProgramImpl::GeomRevision &revision =
            B.chains[size_t(chainIndex)].revisions[size_t(revisionIndex)];
        if (!revision.skinTopologyFixed) {
            continue;
        }
        revision.topology = inputs.revisionPackets[r].skinTopology;
        revision.topologyResolved = true;
        if (!revision.chunked ||
            revision.topology == revision.partitionTopology) {
            continue;
        }
        if (!revision.topology) {
            continue;
        }
        RigExecBakedPartitionRevision(
            &revision, revision.topology->indices.data(),
            revision.topology->indices.size(), revision.topology->elementSize,
            int(revision.chunks.size()));
        revision.partitionTopology = revision.topology;
    }
    return true;
}

// Worker-built tokens, constructed once: TfToken(const char*) takes the
// token registry's spin lock per construction, so the worker hoists them
// the way the assemblers do rather than rebuilding them per read.
const TfToken &
_FrozenKindToken(RigExecRevisionOp op)
{
    static const TfToken normals("recomputeNormals");
    static const TfToken extent("recomputeExtent");
    return op == RigExecRevisionOp::RecomputeExtent ? extent : normals;
}

// Assembles a normals/extent packet on the worker from the region's points
// plus the sampled topology. Replicates moverGraph.cpp's derived arm
// (2820-2836) line for line: synthesized enable and unit weights, topology
// and widths replayed from their samples (_Array's arms: memory, here the
// sampled override-or-stage value, else empty), and the same validity rule.
bool
_AssembleDerivedPacket(const RigExecBakedProgramImpl::GeomRevision &revision,
                       const GfVec3f *basePoints, size_t baseCount,
                       const std::map<SdfPath, size_t> &index,
                       const RigExecFrameInputs &inputs,
                       RigExecMoverParameters *parameters)
{
    RigExecMoverParameters params;
    params.kind = _FrozenKindToken(revision.op);
    params.enabled = true;
    params.weights = RigExecWeightPacket::Constant(1.0f);
    if (!params.weights.valid) {
        *parameters = params;
        return true;
    }
    params.auxPoints.assign(basePoints, basePoints + baseCount);
    const auto readInts = [&index, &inputs](const SdfPath &path,
                                            std::vector<int> *out) -> bool {
        if (path.IsEmpty()) {
            return true;
        }
        const auto found = index.find(path);
        if (found == index.end()) {
            return true;
        }
        const RigExecSampledInput &sample = inputs.values[found->second];
        if (!sample.hasValue) {
            return true;
        }
        if (!sample.value.IsHolding<VtIntArray>()) {
            return false;
        }
        const VtIntArray &held = sample.value.UncheckedGet<VtIntArray>();
        out->assign(held.begin(), held.end());
        return true;
    };
    if (!readInts(revision.binding.topologyCounts, &params.topologyCounts) ||
        !readInts(revision.binding.topologyIndices,
                  &params.topologyIndices)) {
        return false;
    }
    if (revision.op == RigExecRevisionOp::RecomputeExtent &&
        !revision.binding.widths.IsEmpty()) {
        const auto found = index.find(revision.binding.widths);
        if (found != index.end()) {
            const RigExecSampledInput &sample =
                inputs.values[found->second];
            if (sample.hasValue) {
                if (!sample.value.IsHolding<VtFloatArray>()) {
                    return false;
                }
                const VtFloatArray &held =
                    sample.value.UncheckedGet<VtFloatArray>();
                params.widths.assign(held.begin(), held.end());
            }
        }
    }
    params.valid =
        !params.auxPoints.empty() &&
        (revision.op == RigExecRevisionOp::RecomputeExtent ||
         !params.topologyCounts.empty());
    *parameters = params;
    return true;
}

// The frozen RevisionStatic: bakedGeometry.cpp:2195's body with the
// The frozen weight-object step: RigExecBakedWeightPacket restated over
// patched scalar inputs (the _ForEachPatchableInput walker covers them)
// and sampled point arrays (the synthetic keys _SampleWeightArrays
// writes). The builders are the shared pure kernels; only the reads
// differ, because the worker cannot touch the stage or the oracle.
// Volume placements mirror the evaluator's refresh over the same frames
// (rigEvaluator.cpp); the map written is the worker's own.
bool
_FrozenWeightStep(_FrozenWorker *worker, RigExecBakedStep *step,
                  const std::map<SdfPath, size_t> &index,
                  const RigExecFrameInputs &inputs, UsdTimeCode time)
{
    RigExecBakedProgramImpl &B = worker->B;
    if (step->kind == RigExecBakedStepKind::VolumePlacements) {
        // rigEvaluator.cpp _UpdateVolumePlacements over B.fin, through the
        // same provider->fin lookup the live step passes it
        // (bakedWeights.cpp): every no-scale provider writes its path,
        // placed or identity, into the worker's map. The usable gate is
        // that function's exact body (IsValid + IsDegenerate -- NOT the
        // baked pose walk's stricter RigExecBakedUsable), and the write
        // is unconditional: a failed decomposition leaves whatever it
        // wrote rather than the identity, and that partial write is the
        // contract (see the live step's comment).
        for (size_t i = 0; i < B.noScaleAvars.size(); ++i) {
            if (!B.noScaleAvars[i]) {
                continue;
            }
            GfMatrix4d placement(1.0);
            const auto slot = B.index.find(B.paths[i]);
            if (slot != B.index.end()) {
                const RigExecPointFrame &frame =
                    B.fin[size_t(B.finLast[size_t(slot->second)])];
                if (frame.IsValid() && !frame.IsDegenerate()) {
                    RigExecPointsToMatrix(RigExecIdentityLandmarks(),
                                          frame.points, &placement);
                }
            }
            worker->volumeWeightMatrices[B.paths[i]] = placement;
        }
        return true;
    }
    const int id = step->object;
    if (id < 0 || size_t(id) >= B.weightObjects.size() ||
        size_t(id) >= B.weightPackets.size()) {
        return false;
    }
    RigExecBakedProgramImpl::WeightObject &object = B.weightObjects[size_t(id)];
    const auto rd = [&](const auto &input) {
        return RigExecBakedRead(input, *B.resolvedInputs, time,
                                &B.overridden);
    };
    const auto findSample = [&](const SdfPath &path) {
        const auto found = index.find(path);
        if (found == index.end()) {
            return static_cast<const RigExecSampledInput *>(nullptr);
        }
        return &inputs.values[found->second];
    };
    const auto samplePoints = [&](const char *role,
                                  std::vector<GfVec3f> *out) {
        if (const RigExecSampledInput *sample =
                findSample(_FrozenWeightArrayKey(object.path, role))) {
            if (sample->hasValue &&
                sample->value.IsHolding<VtVec3fArray>()) {
                const VtVec3fArray &held =
                    sample->value.UncheckedGet<VtVec3fArray>();
                out->assign(held.begin(), held.end());
            }
        }
    };
    const auto sampleInts = [&](const char *role, std::vector<int> *out) {
        if (const RigExecSampledInput *sample =
                findSample(_FrozenWeightArrayKey(object.path, role))) {
            if (sample->hasValue && sample->value.IsHolding<VtIntArray>()) {
                const VtIntArray &held =
                    sample->value.UncheckedGet<VtIntArray>();
                out->assign(held.begin(), held.end());
            }
        }
    };
    if (object.type == _frozenWeightTokens->staticWeight) {
        RigExecStaticWeightInputs packetInputs;
        packetInputs.representation = object.representation;
        packetInputs.rangePolicy = object.rangePolicy;
        packetInputs.values = object.values;
        packetInputs.indices = object.indices;
        packetInputs.defaultWeight = rd(object.defaultWeight);
        B.weightPackets[size_t(id)] =
            RigExecBuildStaticWeightPacket(packetInputs);
        return true;
    }
    if (object.type == _frozenWeightTokens->dynamicWeight) {
        RigExecDynamicWeightInputs packetInputs;
        packetInputs.representation = object.representation;
        packetInputs.rangePolicy = object.rangePolicy;
        packetInputs.driver = rd(object.driver);
        packetInputs.scale = rd(object.scale);
        packetInputs.bias = rd(object.bias);
        const RigExecWeightPacket *base =
            object.base >= 0 ? &B.weightPackets[size_t(object.base)]
                             : nullptr;
        B.weightPackets[size_t(id)] =
            RigExecBuildDynamicWeightPacket(packetInputs, base);
        return true;
    }
    if (object.type == _frozenWeightTokens->combineWeight) {
        std::vector<RigExecWeightPacket> packetInputs;
        packetInputs.reserve(object.inputs.size());
        for (const int input : object.inputs) {
            packetInputs.push_back(B.weightPackets[size_t(input)]);
        }
        size_t targetCount = 0;
        if (const RigExecSampledInput *sample = findSample(
                _FrozenWeightArrayKey(object.path, "combineTargetCount"))) {
            if (sample->hasValue && sample->value.IsHolding<int>()) {
                targetCount =
                    size_t(sample->value.UncheckedGet<int>());
            }
        }
        B.weightPackets[size_t(id)] = RigExecBuildCombineWeightPacket(
            object.representation, object.rangePolicy, object.combineMode,
            packetInputs, targetCount, rd(object.strength),
            rd(object.invert));
        return true;
    }
    if (object.type == _frozenWeightTokens->sphereWeight ||
        object.type == _frozenWeightTokens->planeWeight ||
        object.type == _frozenWeightTokens->curveWeight) {
        RigExecVolumeWeightInputs packetInputs;
        packetInputs.representation = object.representation;
        packetInputs.rangePolicy = object.rangePolicy;
        if (object.providerSlot >= 0) {
            packetInputs.placement =
                B.base[size_t(B.baseLast[size_t(object.providerSlot)])];
            packetInputs.hasPlacement = true;
        }
        packetInputs.params.falloffMin = rd(object.falloffMin);
        packetInputs.params.falloffMax = rd(object.falloffMax);
        packetInputs.params.invert = rd(object.invert);
        packetInputs.params.strength = rd(object.strength);
        packetInputs.params.curve = object.falloffCurve;
        if (object.type != _frozenWeightTokens->planeWeight) {
            packetInputs.scales = GfVec3f(rd(object.scaleX), rd(object.scaleY),
                                          rd(object.scaleZ));
        } else {
            packetInputs.planeAxis = object.planeAxis;
            packetInputs.planeBounds = object.planeBounds;
            if (object.planeBounds == "bounded") {
                packetInputs.extentU = rd(object.extentU);
                packetInputs.extentV = rd(object.extentV);
            }
        }
        if (RigExecVolumeWeightCanBuild(object.type, packetInputs)) {
            samplePoints("targetPoints", &packetInputs.targetPoints);
            samplePoints("samplePoints", &packetInputs.samplePoints);
            if (object.type == _frozenWeightTokens->curveWeight) {
                samplePoints("curvePoints", &packetInputs.curvePoints);
            }
            B.weightPackets[size_t(id)] =
                RigExecBuildVolumeWeightPacket(object.type, packetInputs);
        } else {
            // Live builds nothing here either: the packet stays whatever
            // the step left, which for a fresh run is default (invalid).
            B.weightPackets[size_t(id)] = RigExecWeightPacket();
        }
        return true;
    }
    if (object.type == _frozenWeightTokens->curvenetWeight) {
        std::vector<GfVec3f> mesh, net;
        std::vector<int> counts, indices, splines, autoSmooth;
        std::vector<float> weights;
        samplePoints("curvenetMeshPoints", &mesh);
        samplePoints("curvenetPoints", &net);
        sampleInts("curvenetCounts", &counts);
        sampleInts("curvenetIndices", &indices);
        sampleInts("curvenetSplines", &splines);
        if (const RigExecSampledInput *sample = findSample(
                _FrozenWeightArrayKey(object.path, "curvenetWeights"))) {
            if (sample->hasValue &&
                sample->value.IsHolding<VtFloatArray>()) {
                const VtFloatArray &held =
                    sample->value.UncheckedGet<VtFloatArray>();
                weights.assign(held.begin(), held.end());
            }
        }
        sampleInts("curvenetAutoSmooth", &autoSmooth);
        const int samples = rd(object.curvenetSamples);
        if (!RigExecCurvenetWeightTokensAreValid(object.curvenetBasis,
                                                object.rangePolicy)) {
            RigExecWeightPacket packet;
            packet.representation = object.representation;
            packet.rangePolicy = object.rangePolicy;
            B.weightPackets[size_t(id)] = packet;
            return true;
        }
        if (!object.bound || object.boundMesh != mesh ||
            object.boundNet != net ||
            object.boundCounts != counts ||
            object.boundIndices != indices ||
            object.boundSplines != splines ||
            object.boundSmooth != autoSmooth ||
            object.boundSamples != samples) {
            object.curvenetBinding = RigExecBindCurvenetWeightPacket(
                mesh, counts, indices, net, splines, object.curvenetBasis,
                samples, autoSmooth, nullptr);
            object.boundMesh = mesh;
            object.boundNet = net;
            object.boundCounts = counts;
            object.boundIndices = indices;
            object.boundSplines = splines;
            object.boundSmooth = autoSmooth;
            object.boundSamples = samples;
            object.bound = true;
        }
        if (!object.curvenetBinding) {
            RigExecWeightPacket packet;
            packet.representation = object.representation;
            packet.rangePolicy = object.rangePolicy;
            B.weightPackets[size_t(id)] = packet;
            return true;
        }
        B.weightPackets[size_t(id)] = RigExecCurvenetWeightPacketFromBinding(
            *object.curvenetBinding, weights, object.rangePolicy,
            rd(object.curvenetUnreached), nullptr);
        return true;
    }
    B.weightPackets[size_t(id)] = RigExecWeightPacket();
    return true;
}

// The common revision envelope (moverGraph.cpp): the bound weight packet,
// or the constant packet synthesized from inputs:defaultWeight. Every
// non-derived op carries exactly this; an invalid envelope fails the
// revision before any op-specific assembly runs.
bool
_FrozenCommonEnvelope(const RigExecBakedProgramImpl &B,
                      const RigExecBakedProgramImpl::GeomRevision &revision,
                      float defaultWeight, RigExecMoverParameters *params)
{
    if (revision.weightObject >= 0 &&
        size_t(revision.weightObject) < B.weightPackets.size()) {
        params->weights = B.weightPackets[size_t(revision.weightObject)];
    } else {
        params->weights = RigExecWeightPacket::Constant(defaultWeight);
    }
    return params->weights.valid;
}

bool
_FrozenSampledEnabled(const std::map<SdfPath, size_t> &index,
                      const RigExecFrameInputs &inputs, const SdfPath &moverPath)
{
    const SdfPath key =
        moverPath.AppendProperty(TfToken("inputs:enabled"));
    const auto found = index.find(key);
    if (found == index.end()) {
        return true;
    }
    const RigExecSampledInput &sample = inputs.values[found->second];
    bool enabled = true;
    if (sample.hasValue && !_SampleHolds(sample.value, &enabled)) {
        return true;
    }
    return enabled;
}

// The phased overlay for one side-input read: when the revision phases
// the input's binding path and this run resolved it, the snapshot value
// shadows the sample, exactly as values.resolved shadows the stage read
// on live. Null otherwise, and the sample answers. Only phased paths
// consult the overlay, so a revision that declares none never touches
// its (stale) revisionInputs.
const VtValue *
_FrozenPhasedValue(
    const RigExecBakedProgramImpl::GeomRevision &revision,
    const SdfPath &bindingPath)
{
    if (revision.binding.phases.find(bindingPath) ==
        revision.binding.phases.end()) {
        return nullptr;
    }
    return revision.revisionInputs.Find(bindingPath);
}

// One phased side-input array: the overlay first, the sample when the
// input is unphased, unresolved, or holding another type (a mistyped
// overlay holding reads as a miss, as live, and the sample answers).
template <class T>
bool
_FrozenSideArray(
    const RigExecBakedProgramImpl::GeomRevision &revision,
    const SdfPath &bindingPath, const SdfPath &sampleKey,
    const std::map<SdfPath, size_t> &index,
    const RigExecFrameInputs &inputs, std::vector<T> *out)
{
    out->clear();
    if (const VtValue *phased = _FrozenPhasedValue(revision, bindingPath)) {
        if (phased->IsHolding<VtArray<T>>()) {
            const VtArray<T> &held = phased->UncheckedGet<VtArray<T>>();
            out->assign(held.begin(), held.end());
            return true;
        }
    }
    if (sampleKey.IsEmpty()) {
        return true;
    }
    const auto found = index.find(sampleKey);
    if (found == index.end()) {
        return true;
    }
    const RigExecSampledInput &sample = inputs.values[found->second];
    if (!sample.hasValue) {
        return true;
    }
    if (!sample.value.IsHolding<VtArray<T>>()) {
        return false;
    }
    const VtArray<T> &held = sample.value.UncheckedGet<VtArray<T>>();
    out->assign(held.begin(), held.end());
    return true;
}

// RigExecAssembleMatrixParameters over frozen state: kind, the enabled
// sample, the shared envelope, then the fold's transform and the
// finite/affine checks -- in live order, so an envelope failure leaves
// kind+enabled set exactly as live does.
bool
_FrozenAssembleMatrix(
    const RigExecBakedProgramImpl &B,
    const RigExecBakedProgramImpl::GeomRevision &revision, float defaultWeight,
    const std::map<SdfPath, size_t> &index, const RigExecFrameInputs &inputs,
    RigExecMoverParameters *params)
{
    params->kind = RigExecRevisionKindToken(RigExecRevisionOp::Matrix);
    params->enabled =
        _FrozenSampledEnabled(index, inputs, revision.moverPath);
    if (!params->enabled) {
        params->valid = true;  // disabled is an ordinary pass-through
        return true;
    }
    if (!revision.haveTransform) {
        return true;  // MoverFailed (valid stays false)
    }
    if (!_FrozenCommonEnvelope(B, revision, defaultWeight, params)) {
        return true;  // invalid common envelope => MoverFailed
    }
    params->transform = revision.transform;
    const GfMatrix4d &transform = params->transform;
    for (int i = 0; i < 4; ++i) {
        for (int j = 0; j < 4; ++j) {
            if (!std::isfinite(transform[i][j])) {
                return true;
            }
        }
    }
    if (transform[0][3] != 0 || transform[1][3] != 0 ||
        transform[2][3] != 0 || transform[3][3] != 1) {
        return true;
    }
    params->valid = true;
    return true;
}

// The wire assembly arm (moverGraph.cpp) over frozen state: rest/order/
// knots/dropoff/bind samples plus the fold's influence table, or the posed
// driver curve when no driver transforms bind. Table math (pick/measured,
// base motion, per-point blend) is the live arm restated.
bool
_FrozenAssembleWire(
    const RigExecBakedProgramImpl &B,
    const RigExecBakedProgramImpl::GeomRevision &revision, float defaultWeight,
    const std::map<SdfPath, size_t> &index, const RigExecFrameInputs &inputs,
    RigExecMoverParameters *params)
{
    const SdfPath &moverPath = revision.moverPath;
    const RigExecRevisionBinding &binding = revision.binding;
    params->kind = RigExecRevisionKindToken(RigExecRevisionOp::Wire);
    params->enabled =
        _FrozenSampledEnabled(index, inputs, moverPath);
    if (!params->enabled) {
        params->valid = true;  // disabled is an ordinary pass-through
        return true;
    }
    if (!_FrozenCommonEnvelope(B, revision, defaultWeight, params)) {
        return true;  // MoverFailed (kind+enabled stay set, as live)
    }
    const auto findSample = [&](const char *role) {
        const auto found =
            index.find(_FrozenWireInputKey(moverPath, role));
        if (found == index.end()) {
            return static_cast<const RigExecSampledInput *>(nullptr);
        }
        return &inputs.values[found->second];
    };
    if (const RigExecSampledInput *sample = findSample("restPoints")) {
        if (sample->hasValue &&
            sample->value.IsHolding<VtVec3fArray>()) {
            const VtVec3fArray &held =
                sample->value.UncheckedGet<VtVec3fArray>();
            params->restPoints.assign(held.begin(), held.end());
        }
    }
    if (const RigExecSampledInput *sample = findSample("curveOrder")) {
        if (sample->hasValue && sample->value.IsHolding<int>()) {
            params->curveOrder = sample->value.UncheckedGet<int>();
        }
    }
    if (const RigExecSampledInput *sample = findSample("curveKnots")) {
        if (sample->hasValue &&
            sample->value.IsHolding<VtDoubleArray>()) {
            const VtDoubleArray &held =
                sample->value.UncheckedGet<VtDoubleArray>();
            params->curveKnots.assign(held.begin(), held.end());
        }
    }
    if (const RigExecSampledInput *sample = findSample("dropoffDistance")) {
        if (sample->hasValue && sample->value.IsHolding<float>()) {
            params->dropoffDistance =
                double(sample->value.UncheckedGet<float>());
        }
    }
    if (const VtValue *phased =
            _FrozenPhasedValue(revision, binding.bindCoords)) {
        if (phased->IsHolding<VtArray<GfVec2f>>()) {
            params->wireBindCoords =
                phased->UncheckedGet<VtArray<GfVec2f>>();
        } else if (const RigExecSampledInput *sample =
                       findSample("bindCoords")) {
            if (sample->hasValue &&
                sample->value.IsHolding<VtArray<GfVec2f>>()) {
                params->wireBindCoords =
                    sample->value.UncheckedGet<VtArray<GfVec2f>>();
            }
        }
    } else if (const RigExecSampledInput *sample =
                   findSample("bindCoords")) {
        if (sample->hasValue &&
            sample->value.IsHolding<VtArray<GfVec2f>>()) {
            params->wireBindCoords =
                sample->value.UncheckedGet<VtArray<GfVec2f>>();
        }
    }
    if (binding.driverTransformCount > 0) {
        const size_t t = size_t(binding.driverTransformCount);
        const size_t s = size_t(binding.driverSpaceCount);
        const size_t bt = size_t(binding.driverBaseTransformCount);
        const std::vector<GfMatrix4d> *table = &revision.influences;
        if (table->size() < t + s + bt) {
            return true;  // MoverFailed (valid stays false)
        }
        const size_t bs = table->size() - t - s - bt;
        VtFloatArray driverWeights, driverBaseWeights;
        if (const RigExecSampledInput *sample =
                findSample("driverWeights")) {
            if (sample->hasValue &&
                sample->value.IsHolding<VtFloatArray>()) {
                driverWeights =
                    sample->value.UncheckedGet<VtFloatArray>();
            }
        }
        if (const RigExecSampledInput *sample =
                findSample("driverBaseWeights")) {
            if (sample->hasValue &&
                sample->value.IsHolding<VtFloatArray>()) {
                driverBaseWeights =
                    sample->value.UncheckedGet<VtFloatArray>();
            }
        }
        const auto pick = [](size_t count, size_t j) {
            return count <= 1 ? size_t(0) : j % count;
        };
        const auto measured = [&](size_t first, size_t count,
                                  size_t spaceFirst, size_t spaceCount,
                                  size_t j) {
            GfMatrix4d m = (*table)[first + pick(count, j)];
            if (spaceCount > 0) {
                m = RigExecMeasureInSpace(
                    m, (*table)[spaceFirst + pick(spaceCount, j)]);
            }
            return m;
        };
        params->auxPoints.resize(params->restPoints.size());
        for (size_t j = 0; j < params->restPoints.size(); ++j) {
            GfVec3f &rest = params->restPoints[j];
            if (bt > 0) {
                const GfMatrix4d b =
                    measured(t + s, bt, t + s + bt, bs, j);
                const float wb = driverBaseWeights.empty()
                    ? 1.0f
                    : driverBaseWeights[pick(driverBaseWeights.size(), j)];
                const GfVec3f moved(b.TransformAffine(GfVec3d(rest)));
                rest = rest + (moved - rest) * wb;
            }
            const GfMatrix4d m = measured(0, t, t, s, j);
            const float w = driverWeights.empty()
                ? 1.0f
                : driverWeights[pick(driverWeights.size(), j)];
            const GfVec3f moved(m.TransformAffine(GfVec3d(rest)));
            params->auxPoints[j] = rest + (moved - rest) * w;
        }
    } else if (const VtValue *phased =
                   _FrozenPhasedValue(revision,
                                       binding.driverCurvePoints)) {
        if (phased->IsHolding<VtVec3fArray>()) {
            const VtVec3fArray &held =
                phased->UncheckedGet<VtVec3fArray>();
            params->auxPoints.assign(held.begin(), held.end());
        } else if (const RigExecSampledInput *sample =
                       findSample("driverCurvePoints")) {
            if (sample->hasValue &&
                sample->value.IsHolding<VtVec3fArray>()) {
                const VtVec3fArray &held =
                    sample->value.UncheckedGet<VtVec3fArray>();
                params->auxPoints.assign(held.begin(), held.end());
            }
        }
    } else if (const RigExecSampledInput *sample =
                   findSample("driverCurvePoints")) {
        if (sample->hasValue &&
            sample->value.IsHolding<VtVec3fArray>()) {
            const VtVec3fArray &held =
                sample->value.UncheckedGet<VtVec3fArray>();
            params->auxPoints.assign(held.begin(), held.end());
        }
    }
    const RigExecNurbsCurve rest{&params->restPoints, params->curveOrder,
                                 &params->curveKnots};
    params->valid = rest.IsValid() &&
                    params->auxPoints.size() == params->restPoints.size() &&
                    !params->wireBindCoords.empty();
    return true;
}

// AssembleRevision call replaced by the transported packet (skin) or the
// The BlendShape arm (moverGraph.cpp RigExecAssembleParameters) over frozen
// state: the channel gather from AssembleRevision restated over sampled
// values and transported layouts, summed by the same
// RigExecSumBlendChannels, then kind/enabled/envelope/deltaSpace in live
// order. Only plain data from the clone's channels is touched (paths, pose
// slots, phases, blend-shape paths) -- never the dead UsdAttributes, never
// the freeze-time layouts, never the evaluator-bound resolve callback.
//
// R.Find answers the pose-slot question exactly as live: the worker's
// resolved inputs hold the job's overrides and chain results and nothing
// else, which are the only things live R can hold at a weight path that
// the pose slot does not already answer. (An interpolator's fill at the
// weight path itself is the slot's own value placed, so the slot answers
// it identically.)
bool
_FrozenAssembleBlendShape(
    const RigExecBakedProgramImpl &B,
    const RigExecBakedProgramImpl::GeomRevision &revision,
    float defaultWeight, const VtVec3fArray &lastBase,
    size_t revisionPosition, const std::map<SdfPath, size_t> &index,
    const RigExecFrameInputs &inputs, RigExecMoverParameters *params)
{
    const SdfPath &moverPath = revision.moverPath;
    const RigExecRevisionBinding &binding = revision.binding;
    const RigExecResolvedInputs &R = *B.resolvedInputs;
    params->kind = RigExecRevisionKindToken(RigExecRevisionOp::BlendShape);
    params->enabled =
        _FrozenSampledEnabled(index, inputs, moverPath);
    if (!params->enabled) {
        params->valid = true;  // disabled is an ordinary pass-through
        return true;
    }
    if (!_FrozenCommonEnvelope(B, revision, defaultWeight, params)) {
        return true;  // MoverFailed (kind+enabled stay set, as live)
    }
    const auto findSample = [&](const SdfPath &key) {
        const auto found = index.find(key);
        if (found == index.end()) {
            return static_cast<const RigExecSampledInput *>(nullptr);
        }
        return &inputs.values[found->second];
    };
    std::vector<RigExecBlendChannel> channels;
    channels.reserve(revision.blendChannels.size());
    for (size_t c = 0; c < revision.blendChannels.size(); ++c) {
        const RigExecBakedProgramImpl::GeomBlendChannel &bound =
            revision.blendChannels[c];
        RigExecBlendChannel channel;
        if (bound.poseWeight >= 0 && !R.Find(bound.weightPath)) {
            if (size_t(bound.poseWeight) >= B.poseWeights.size()) {
                return false;
            }
            channel.weight = B.poseWeights[size_t(bound.poseWeight)];
        } else {
            channel.weight = 0.0f;
            if (const RigExecSampledInput *sample =
                    findSample(bound.weightPath)) {
                // Always present when the attribute exists (the sampler
                // stores the fallback on a miss); absent means no
                // attribute, which reads as the channel's 0 init.
                if (sample->hasValue &&
                    !_SampleHolds(sample->value, &channel.weight)) {
                    return false;
                }
            }
        }
        for (size_t s = 0; s < bound.samples.size(); ++s) {
            const RigExecBakedProgramImpl::GeomBlendChannel::Sample
                &boundSample = bound.samples[s];
            if (!boundSample.phase.IsBase()) {
                return false;  // freeze refused; a stale-vector guard
            }
            RigExecBlendSampleData sample;
            sample.activation = 1.0f;
            if (const RigExecSampledInput *asample =
                    findSample(boundSample.activationPath)) {
                if (asample->hasValue &&
                    !_SampleHolds(asample->value, &sample.activation)) {
                    return false;
                }
            }
            if (!boundSample.blendShape.IsEmpty()) {
                if (revisionPosition >= inputs.blendLayouts.size()) {
                    return false;
                }
                const auto &channelLayouts =
                    inputs.blendLayouts[revisionPosition];
                if (c >= channelLayouts.size() ||
                    s >= channelLayouts[c].size() ||
                    !channelLayouts[c][s]) {
                    return false;
                }
                sample.layout = channelLayouts[c][s];
            } else if (const RigExecSampledInput *psample = findSample(
                           _FrozenBlendInputKey(boundSample.samplePath,
                                                "points"))) {
                if (psample->hasValue) {
                    if (!psample->value.IsHolding<VtVec3fArray>()) {
                        return false;
                    }
                    const VtVec3fArray &held =
                        psample->value.UncheckedGet<VtVec3fArray>();
                    sample.points.assign(held.begin(), held.end());
                }
            }
            channel.samples.push_back(std::move(sample));
        }
        std::stable_sort(channel.samples.begin(), channel.samples.end(),
                         [](const RigExecBlendSampleData &a,
                            const RigExecBlendSampleData &b) {
                             return a.activation < b.activation;
                         });
        channels.push_back(std::move(channel));
    }
    std::vector<GfVec3f> basePoints(lastBase.begin(), lastBase.end());
    std::vector<GfVec3f> blendDeltas;
    if (!RigExecSumBlendChannels(channels, basePoints, &blendDeltas)) {
        blendDeltas.clear();
    }
    params->blendDeltas = std::move(blendDeltas);
    TfToken space("target");
    if (const RigExecSampledInput *sample = findSample(
            moverPath.AppendProperty(TfToken("rigExec:deltaSpace")))) {
        if (sample->hasValue &&
            !_SampleHolds(sample->value, &space)) {
            return false;
        }
    }
    if (space != "target" && space != "surfaceFrame") {
        return true;  // valid stays false, as live
    }
    params->blendSurfaceFrame = space == "surfaceFrame";
    if (params->blendSurfaceFrame) {
        params->restPoints = basePoints;
        if (const RigExecSampledInput *sample =
                findSample(binding.topologyCounts)) {
            if (sample->hasValue) {
                if (!sample->value.IsHolding<VtIntArray>()) {
                    return false;
                }
                const VtIntArray &held =
                    sample->value.UncheckedGet<VtIntArray>();
                params->topologyCounts.assign(held.begin(), held.end());
            }
        }
        if (const RigExecSampledInput *sample =
                findSample(binding.topologyIndices)) {
            if (sample->hasValue) {
                if (!sample->value.IsHolding<VtIntArray>()) {
                    return false;
                }
                const VtIntArray &held =
                    sample->value.UncheckedGet<VtIntArray>();
                params->topologyIndices.assign(held.begin(), held.end());
            }
        }
        if (params->topologyCounts.empty()) {
            return true;
        }
    }
    params->valid = !params->blendDeltas.empty();
    return true;
}

// Reads one _Array arm's sample: missing or valueless is the empty array,
// as live; a mistyped holding fails closed (the sampler only ever stores
// the arm's own type, so anything else is a corrupt vector).
template <class T>
bool
_FrozenSampledArray(const std::map<SdfPath, size_t> &index,
                    const RigExecFrameInputs &inputs, const SdfPath &key,
                    std::vector<T> *out)
{
    out->clear();
    if (key.IsEmpty()) {
        return true;
    }
    const auto found = index.find(key);
    if (found == index.end()) {
        return true;
    }
    const RigExecSampledInput &sample = inputs.values[found->second];
    if (!sample.hasValue) {
        return true;
    }
    if (!sample.value.IsHolding<VtArray<T>>()) {
        return false;
    }
    const VtArray<T> &held = sample.value.UncheckedGet<VtArray<T>>();
    out->assign(held.begin(), held.end());
    return true;
}

// The VolumeCorrect arm over frozen state: kind, enabled, the shared
// envelope, then the bound volume of the chain's base -- measured off the
// worker's own last base, which is the array live copies into
// values.basePoints before measuring, so no copy is needed to agree.
bool
_FrozenAssembleVolumeCorrect(
    const RigExecBakedProgramImpl &B,
    const RigExecBakedProgramImpl::GeomRevision &revision,
    float defaultWeight, const VtVec3fArray &lastBase,
    const std::map<SdfPath, size_t> &index,
    const RigExecFrameInputs &inputs, RigExecMoverParameters *params)
{
    params->kind = RigExecRevisionKindToken(RigExecRevisionOp::VolumeCorrect);
    params->enabled =
        _FrozenSampledEnabled(index, inputs, revision.moverPath);
    if (!params->enabled) {
        params->valid = true;  // disabled is an ordinary pass-through
        return true;
    }
    if (!_FrozenCommonEnvelope(B, revision, defaultWeight, params)) {
        return true;  // MoverFailed (kind+enabled stay set, as live)
    }
    params->strength = 1.0f;
    if (!lastBase.empty()) {
        params->referenceVolume =
            RigExecBoundVolume(lastBase.cdata(), lastBase.size());
        params->valid = true;
    }
    return true;
}

// The Smooth arm over frozen state: kind, enabled, the shared envelope,
// then the sampled topology at the evaluated time.
bool
_FrozenAssembleSmooth(
    const RigExecBakedProgramImpl &B,
    const RigExecBakedProgramImpl::GeomRevision &revision,
    float defaultWeight, const std::map<SdfPath, size_t> &index,
    const RigExecFrameInputs &inputs, RigExecMoverParameters *params)
{
    const RigExecRevisionBinding &binding = revision.binding;
    params->kind = RigExecRevisionKindToken(RigExecRevisionOp::Smooth);
    params->enabled =
        _FrozenSampledEnabled(index, inputs, revision.moverPath);
    if (!params->enabled) {
        params->valid = true;  // disabled is an ordinary pass-through
        return true;
    }
    if (!_FrozenCommonEnvelope(B, revision, defaultWeight, params)) {
        return true;  // MoverFailed (kind+enabled stay set, as live)
    }
    params->strength = 1.0f;
    if (!_FrozenSampledArray(index, inputs, binding.topologyCounts,
                             &params->topologyCounts) ||
        !_FrozenSampledArray(index, inputs, binding.topologyIndices,
                             &params->topologyIndices)) {
        return false;
    }
    params->valid = !params->topologyCounts.empty();
    return true;
}

// The Lattice arm over frozen state: kind, enabled, the shared envelope,
// then rest points plus the rest/live cage pair and the divisions, in live
// order. auxPoints is the BIND-TIME cage and auxPointsB the live one --
// reversing them inverts the deformation as soon as the cage moves.
bool
_FrozenAssembleLattice(
    const RigExecBakedProgramImpl &B,
    const RigExecBakedProgramImpl::GeomRevision &revision,
    float defaultWeight, const VtVec3fArray &lastBase,
    const std::map<SdfPath, size_t> &index,
    const RigExecFrameInputs &inputs, RigExecMoverParameters *params)
{
    const SdfPath &moverPath = revision.moverPath;
    const RigExecRevisionBinding &latticeBinding = revision.binding;
    params->kind = RigExecRevisionKindToken(RigExecRevisionOp::Lattice);
    params->enabled =
        _FrozenSampledEnabled(index, inputs, moverPath);
    if (!params->enabled) {
        params->valid = true;  // disabled is an ordinary pass-through
        return true;
    }
    if (!_FrozenCommonEnvelope(B, revision, defaultWeight, params)) {
        return true;  // MoverFailed (kind+enabled stay set, as live)
    }
    params->restPoints.assign(lastBase.begin(), lastBase.end());
    if (!_FrozenSampledArray(
            index, inputs, _FrozenLatticeInputKey(moverPath, "cageRest"),
            &params->auxPoints) ||
        !_FrozenSideArray(
            revision, latticeBinding.cagePoints,
            _FrozenLatticeInputKey(moverPath, "cageLive"), index, inputs,
            &params->auxPointsB)) {
        return false;
    }
    params->divisions = GfVec3i(0, 0, 0);
    const auto found = index.find(
        moverPath.AppendProperty(TfToken("rigExec:divisions")));
    if (found != index.end()) {
        const RigExecSampledInput &sample = inputs.values[found->second];
        if (sample.hasValue &&
            !_SampleHolds(sample.value, &params->divisions)) {
            return false;
        }
    }
    const size_t cageCount = size_t(params->divisions[0]) *
                             size_t(params->divisions[1]) *
                             size_t(params->divisions[2]);
    params->valid = params->divisions[0] >= 2 &&
                    params->divisions[1] >= 2 &&
                    params->divisions[2] >= 2 &&
                    params->auxPoints.size() == cageCount &&
                    params->auxPointsB.size() == cageCount &&
                    !params->restPoints.empty();
    return true;
}

// The SurfaceProject arm over frozen state: kind, enabled, the shared
// envelope, then the driver surface and its topology at the evaluated
// time. Strength is fixed at full: the schema declares no strength input,
// and reading a default would project half way.
bool
_FrozenAssembleSurfaceProject(
    const RigExecBakedProgramImpl &B,
    const RigExecBakedProgramImpl::GeomRevision &revision,
    float defaultWeight, const std::map<SdfPath, size_t> &index,
    const RigExecFrameInputs &inputs, RigExecMoverParameters *params)
{
    const RigExecRevisionBinding &binding = revision.binding;
    params->kind = RigExecRevisionKindToken(RigExecRevisionOp::SurfaceProject);
    params->enabled =
        _FrozenSampledEnabled(index, inputs, revision.moverPath);
    if (!params->enabled) {
        params->valid = true;  // disabled is an ordinary pass-through
        return true;
    }
    if (!_FrozenCommonEnvelope(B, revision, defaultWeight, params)) {
        return true;  // MoverFailed (kind+enabled stay set, as live)
    }
    params->strength = 1.0f;
    if (!_FrozenSideArray(
            revision, binding.surfacePoints,
            _FrozenSurfaceInputKey(revision.moverPath, "surfacePoints"),
            index, inputs, &params->auxPoints) ||
        !_FrozenSampledArray(index, inputs, binding.topologyCounts,
                             &params->topologyCounts) ||
        !_FrozenSampledArray(index, inputs, binding.topologyIndices,
                             &params->topologyIndices)) {
        return false;
    }
    params->valid =
        !params->auxPoints.empty() && !params->topologyCounts.empty();
    return true;
}

// The driver solver's aggregate for a ribbon-family revision: the
// worker's own Solve-step output, which is this run's by program order --
// the Aggregate-slot edge runs the solver before the revision on both
// paths. Null when no driver binds, exactly as live.
const RigExecPointFrameArray *
_FrozenDriverFrames(const RigExecBakedProgramImpl &B,
                    const RigExecBakedProgramImpl::GeomRevision &revision,
                    bool *usable)
{
    *usable = true;
    if (revision.driverFramesSolver < 0) {
        return nullptr;
    }
    if (size_t(revision.driverFramesSolver) >= B.aggregates.size()) {
        *usable = false;
        return nullptr;
    }
    return &B.aggregates[size_t(revision.driverFramesSolver)];
}

// The Ribbon arm over frozen state: kind, enabled, the shared envelope,
// then the driver's frames plus the sampled bind coordinates.
bool
_FrozenAssembleRibbon(
    const RigExecBakedProgramImpl &B,
    const RigExecBakedProgramImpl::GeomRevision &revision,
    float defaultWeight, const std::map<SdfPath, size_t> &index,
    const RigExecFrameInputs &inputs, RigExecMoverParameters *params)
{
    params->kind = RigExecRevisionKindToken(RigExecRevisionOp::Ribbon);
    params->enabled =
        _FrozenSampledEnabled(index, inputs, revision.moverPath);
    if (!params->enabled) {
        params->valid = true;  // disabled is an ordinary pass-through
        return true;
    }
    if (!_FrozenCommonEnvelope(B, revision, defaultWeight, params)) {
        return true;  // MoverFailed (kind+enabled stay set, as live)
    }
    bool usable = true;
    const RigExecPointFrameArray *frames =
        _FrozenDriverFrames(B, revision, &usable);
    if (!usable) {
        return false;
    }
    if (!frames || frames->IsEmpty() ||
        frames->rests.size() != frames->GetSize()) {
        return true;  // MoverFailed
    }
    params->frames = *frames;
    if (!_FrozenSideArray(
            revision, revision.binding.bindCoords,
            _FrozenRibbonInputKey(revision.moverPath, "bindCoords"), index,
            inputs, &params->bindCoords)) {
        return false;
    }
    params->valid = !params->bindCoords.empty();
    return true;
}

// The EmitGuidePoints arm over frozen state: the driver's frames, valid
// whenever they are well-formed. No bind coordinates: the kernel writes
// one guide per frame origin.
bool
_FrozenAssembleEmitGuidePoints(
    const RigExecBakedProgramImpl &B,
    const RigExecBakedProgramImpl::GeomRevision &revision,
    float defaultWeight, const std::map<SdfPath, size_t> &index,
    const RigExecFrameInputs &inputs, RigExecMoverParameters *params)
{
    params->kind =
        RigExecRevisionKindToken(RigExecRevisionOp::EmitGuidePoints);
    params->enabled =
        _FrozenSampledEnabled(index, inputs, revision.moverPath);
    if (!params->enabled) {
        params->valid = true;  // disabled is an ordinary pass-through
        return true;
    }
    if (!_FrozenCommonEnvelope(B, revision, defaultWeight, params)) {
        return true;  // MoverFailed (kind+enabled stay set, as live)
    }
    bool usable = true;
    const RigExecPointFrameArray *frames =
        _FrozenDriverFrames(B, revision, &usable);
    if (!usable) {
        return false;
    }
    if (!frames || frames->IsEmpty() ||
        frames->rests.size() != frames->GetSize()) {
        return true;  // MoverFailed
    }
    params->frames = *frames;
    params->valid = true;
    return true;
}

// The Curvenet arm over frozen state: kind, enabled, the shared envelope,
// then the rest net, the rest surface and its topology, the posed net, and
// the transported bind, in live order. The sampler sampled nothing when
// the net prim was missing, so the empty-check below breaks with exactly
// the packet live's netPrim check breaks with. The bake-retention sink is
// skipped: a per-assembly scratch nothing downstream reads on the worker.
bool
_FrozenAssembleCurvenet(
    const RigExecBakedProgramImpl &B,
    const RigExecBakedProgramImpl::GeomRevision &revision,
    float defaultWeight, size_t revisionPosition,
    const std::map<SdfPath, size_t> &index,
    const RigExecFrameInputs &inputs, RigExecMoverParameters *params)
{
    const SdfPath &moverPath = revision.moverPath;
    params->kind = RigExecRevisionKindToken(RigExecRevisionOp::Curvenet);
    params->enabled =
        _FrozenSampledEnabled(index, inputs, moverPath);
    if (!params->enabled) {
        params->valid = true;  // disabled is an ordinary pass-through
        return true;
    }
    if (!_FrozenCommonEnvelope(B, revision, defaultWeight, params)) {
        return true;  // MoverFailed (kind+enabled stay set, as live)
    }
    params->strength = 1.0f;
    std::vector<GfVec3f> restNet;
    if (!_FrozenSampledArray(
            index, inputs, _FrozenCurvenetInputKey(moverPath, "restNet"),
            &restNet) ||
        !_FrozenSampledArray(
            index, inputs,
            _FrozenCurvenetInputKey(moverPath, "topologyCounts"),
            &params->topologyCounts) ||
        !_FrozenSampledArray(
            index, inputs,
            _FrozenCurvenetInputKey(moverPath, "topologyIndices"),
            &params->topologyIndices) ||
        !_FrozenSampledArray(
            index, inputs,
            _FrozenCurvenetInputKey(moverPath, "restPoints"),
            &params->restPoints)) {
        return false;
    }
    if (restNet.empty() || params->topologyCounts.empty() ||
        params->restPoints.empty()) {
        return true;  // MoverFailed
    }
    std::vector<GfVec3f> posed;
    if (revision.curvenetChain >= 0) {
        if (size_t(revision.curvenetChain) >= B.chains.size()) {
            return false;
        }
        const RigExecBakedProgramImpl::GeomChain &net =
            B.chains[size_t(revision.curvenetChain)];
        if (net.haveBase && net.haveResult) {
            posed.assign(net.result.begin(), net.result.end());
        }
    }
    if (posed.empty() &&
        !_FrozenSampledArray(
            index, inputs, _FrozenCurvenetInputKey(moverPath, "posedNet"),
            &posed)) {
        return false;
    }
    params->auxPoints = std::move(posed);
    if (params->auxPoints.size() != restNet.size()) {
        return true;  // MoverFailed
    }
    if (revisionPosition >= inputs.curvenetBinds.size()) {
        return false;
    }
    // A null transport is a remembered failed bind, served as-is.
    params->curvenetBinding = inputs.curvenetBinds[revisionPosition];
    params->valid = params->curvenetBinding != nullptr;
    return true;
}

// worker-side derived assembly (normals/extent), and the mover-prim
// defaultWeight read replaced by its sample. Everything else -- status,
// dirty compare, publication sizing, layout and envelope decisions -- is the
// same code shape over the same fields.
bool
_FrozenRevisionStatic(_FrozenWorker *worker, RigExecBakedStep *step,
                      const std::map<SdfPath, size_t> &index,
                      const RigExecFrameInputs &inputs)
{
    RigExecBakedProgramImpl &B = worker->B;
    const auto &[chainIndex, revisionIndex] =
        B.revisionIndex[size_t(step->object)];
    RigExecBakedProgramImpl::GeomChain &chain = B.chains[size_t(chainIndex)];
    if (!chain.haveBase) {
        return true;
    }
    RigExecBakedProgramImpl::GeomRevision &revision =
        chain.revisions[size_t(revisionIndex)];
    if (revision.weightCurrentPhase) {
        return false;
    }
    revision.defaultWeight = 1.0f;
    {
        const SdfPath key = revision.moverPath.AppendProperty(
            TfToken("inputs:defaultWeight"));
        const auto found = index.find(key);
        if (found != index.end()) {
            const RigExecSampledInput &sample =
                inputs.values[found->second];
            if (sample.hasValue) {
                if (!_SampleHolds(sample.value, &revision.defaultWeight)) {
                    return false;
                }
            }
        }
    }
    // One overlay per revision that declares phases: the worker's resolved
    // inputs plus whatever the phases resolve to out of this run's
    // snapshot store -- the same construction AssembleRevision builds,
    // over the worker's own store, which the same-ordered steps filled.
    // The side-input reads below consult it by binding path before their
    // samples, exactly where live consults values.resolved.
    if (!revision.binding.phases.empty()) {
        if (!B.resolvedInputs) {
            return false;
        }
        revision.revisionInputs = *B.resolvedInputs;
        for (const auto &[inputPath, phase] : revision.binding.phases) {
            if (const VtValue *recorded = B.runSnapshots.Lookup(
                    inputPath, phase, revision.moverPath)) {
                revision.revisionInputs.SetProperty(inputPath, *recorded);
            } else if (phase.kind != RigExecReadPhaseKind::Preceding) {
                step->diagnostics.push_back(
                    "diag " + revision.moverPath.GetString() +
                    ": read phase '" + phase.GetAsString() + "' for " +
                    inputPath.GetString() +
                    " resolved to nothing; read the authored base");
            }
        }
    }
    if (revision.op == RigExecRevisionOp::Skin) {
        if (size_t(step->object) >= inputs.revisionPackets.size()) {
            return false;
        }
        revision.parameters = inputs.revisionPackets[size_t(step->object)];
        // Transported skin packets are assembled envelopeless; a bound
        // weight object overrides with its packet, as the live assemble
        // does. Envelopeless skins keep the transported packet untouched.
        if (revision.weightObject >= 0) {
            if (!_FrozenCommonEnvelope(B, revision, revision.defaultWeight,
                                       &revision.parameters)) {
                revision.parameters.valid = false;
            }
        }
    } else if (revision.op == RigExecRevisionOp::RecomputeNormals ||
               revision.op == RigExecRevisionOp::RecomputeExtent) {
        if (!_AssembleDerivedPacket(revision, chain.lastBase.cdata(),
                                    chain.lastBase.size(), index, inputs,
                                    &revision.parameters)) {
            return false;
        }
    } else if (revision.op == RigExecRevisionOp::Matrix) {
        if (!_FrozenAssembleMatrix(B, revision, revision.defaultWeight,
                                   index, inputs, &revision.parameters)) {
            return false;
        }
    } else if (revision.op == RigExecRevisionOp::Wire) {
        if (!_FrozenAssembleWire(B, revision, revision.defaultWeight,
                                 index, inputs, &revision.parameters)) {
            return false;
        }
    } else if (revision.op == RigExecRevisionOp::BlendShape) {
        if (!_FrozenAssembleBlendShape(B, revision, revision.defaultWeight,
                                       chain.lastBase, size_t(step->object),
                                       index, inputs,
                                       &revision.parameters)) {
            return false;
        }
    } else if (revision.op == RigExecRevisionOp::VolumeCorrect) {
        if (!_FrozenAssembleVolumeCorrect(B, revision, revision.defaultWeight,
                                          chain.lastBase, index, inputs,
                                          &revision.parameters)) {
            return false;
        }
    } else if (revision.op == RigExecRevisionOp::Smooth) {
        if (!_FrozenAssembleSmooth(B, revision, revision.defaultWeight,
                                   index, inputs, &revision.parameters)) {
            return false;
        }
    } else if (revision.op == RigExecRevisionOp::Lattice) {
        if (!_FrozenAssembleLattice(B, revision, revision.defaultWeight,
                                    chain.lastBase, index, inputs,
                                    &revision.parameters)) {
            return false;
        }
    } else if (revision.op == RigExecRevisionOp::SurfaceProject) {
        if (!_FrozenAssembleSurfaceProject(B, revision, revision.defaultWeight,
                                           index, inputs,
                                           &revision.parameters)) {
            return false;
        }
    } else if (revision.op == RigExecRevisionOp::Ribbon) {
        if (!_FrozenAssembleRibbon(B, revision, revision.defaultWeight,
                                   index, inputs, &revision.parameters)) {
            return false;
        }
    } else if (revision.op == RigExecRevisionOp::EmitGuidePoints) {
        if (!_FrozenAssembleEmitGuidePoints(B, revision,
                                            revision.defaultWeight, index,
                                            inputs, &revision.parameters)) {
            return false;
        }
    } else if (revision.op == RigExecRevisionOp::Curvenet) {
        if (!_FrozenAssembleCurvenet(B, revision, revision.defaultWeight,
                                     size_t(step->object), index, inputs,
                                     &revision.parameters)) {
            return false;
        }
    } else {
        return false;
    }
    revision.status =
        RigExecStatusForParameters(revision.parameters, revision.moverPath);
    step->counters.revisionsBuilt = 1;
    revision.staticDirty =
        !revision.ran || revision.parameters != revision.lastParameters ||
        revision.status != revision.lastStatus ||
        revision.defaultWeight != revision.lastDefaultWeight;
    revision.lastDefaultWeight = revision.defaultWeight;
    revision.weightFieldPublished = false;
    if (revision.weightObject >= 0 && B.publishWeightFields) {
        const RigExecWeightPacket &packet =
            B.weightPackets[size_t(revision.weightObject)];
        if (packet.valid) {
            const size_t logicalCount = revision.weightOperationDomain
                ? size_t(1)
                : chain.lastBase.size();
            if (!packet.ResolveAll(logicalCount, &revision.weightField)) {
                revision.weightField.assign(logicalCount, 0.0f);
                for (size_t i = 0; i < logicalCount; ++i) {
                    const float w = packet.Resolve(i, logicalCount);
                    revision.weightField[i] = w < 0.0f ? 0.0f : w;
                }
            }
            revision.weightFieldPublished = true;
        }
    }
    const size_t count = chain.lastBase.size();
    if (revision.output.size() != count) {
        revision.output.resize(count);
        revision.staticDirty = true;
    }
    revision.precedingCount = count;
    revision.layoutUsable = false;
    revision.envelopeOk = true;
    revision.fullStrength = true;
    revision.partitionStale = false;
    if (revision.op != RigExecRevisionOp::Skin) {
        return true;
    }
    revision.layoutUsable =
        RigExecSkinLayoutIsUsable(revision.parameters, count);
    revision.fullStrength =
        RigExecEnvelopeIsFullStrength(revision.parameters.weights);
    if (!revision.fullStrength) {
        revision.envelopeOk = revision.parameters.weights.ResolveAll(
            count, &revision.envelope);
    }
    if (revision.chunked) {
        const RigExecSkinTopology *const topology =
            revision.parameters.skinTopology.get();
        const size_t indexCount =
            topology ? topology->indices.size()
                     : revision.parameters.skinIndices.size();
        const int elementSize =
            topology ? topology->elementSize
                     : revision.parameters.skinElementSize;
        revision.partitionStale =
            !revision.parameters.skinTopology ||
            revision.parameters.skinTopology != revision.partitionTopology ||
            indexCount != revision.partitionIndexCount ||
            elementSize != revision.partitionElementSize ||
            count != revision.partitionPointCount;
    }
    return true;
}

// The frozen Derived step: bakedGeometry.cpp:2035's body with the same
// packet substitution (the base value here is the chain's published result,
// not its last base).
bool
_FrozenDerived(_FrozenWorker *worker, RigExecBakedStep *step,
              const std::map<SdfPath, size_t> &index,
              const RigExecFrameInputs &inputs)
{
    RigExecBakedProgramImpl &B = worker->B;
    const auto &[chainIndex, derivedIndex] =
        B.derivedIndex[size_t(step->object)];
    RigExecBakedProgramImpl::GeomChain &chain = B.chains[size_t(chainIndex)];
    RigExecBakedProgramImpl::GeomChain::Derived &derived =
        chain.derived[size_t(derivedIndex)];
    RigExecBakedProgramImpl::GeomRevision &revision = derived.revision;
    if (!chain.haveResult || !derived.haveBase) {
        return true;
    }
    RigExecMoverParameters parameters;
    if (!_AssembleDerivedPacket(revision, chain.result.cdata(),
                                chain.result.size(), index, inputs,
                                &parameters)) {
        return false;
    }
    const RigExecMoverStatus status =
        RigExecStatusForParameters(parameters, revision.moverPath);
    step->counters.revisionsBuilt = 1;
    std::vector<GfVec3f> aux;
    aux.swap(parameters.auxPoints);
    const bool moved = chain.result != revision.lastAuxPoints ||
                       parameters != revision.lastParameters;
    parameters.auxPoints.swap(aux);
    if (derived.baseDirty || !revision.ran || moved ||
        status != revision.lastStatus) {
        step->counters.revisionsExecuted = 1;
        std::vector<GfVec3f> values(derived.lastBase.begin(),
                                    derived.lastBase.end());
        const bool applied =
            status.AllowsApply() &&
            RigExecRunRevisionKernel(revision.op, parameters, &values,
                                     /*controlFrames=*/nullptr);
        revision.resultStatus = status.state;
        if (!applied) {
            values.assign(derived.lastBase.begin(), derived.lastBase.end());
            if (status.AllowsApply()) {
                static const TfToken moverFailed("moverFailed");
                revision.resultStatus = moverFailed;
            }
        }
        revision.output = std::move(values);
        parameters.auxPoints.clear();
        parameters.auxPoints.shrink_to_fit();
        revision.lastParameters = std::move(parameters);
        revision.lastAuxPoints = chain.result;
        revision.lastStatus = status;
        revision.ran = true;
    }
    if (revision.resultStatus == "moverFailed") {
        step->diagnostics.push_back(
            "MoverFailed " + derived.target.GetString() +
            ": derived geometry input/cardinality validation failed");
    }
    derived.spare.resize(revision.output.size());
    std::copy(revision.output.begin(), revision.output.end(),
              derived.spare.data());
    derived.result.swap(derived.spare);
    derived.haveResult = true;
    step->counters.chainsBuilt = 1;
    return true;
}

// One step's body, with the two stage-reading bodies substituted. Otherwise
// the same dispatch RunStepBody performs (bakedSchedule.cpp:1524),
// including BeginRun's clearing promise and the weight-step refusal.
bool
_FrozenStepBody(_FrozenWorker *worker, RigExecBakedStep *step,
               const std::map<SdfPath, size_t> &index,
               const RigExecFrameInputs &inputs, UsdTimeCode time)
{
    RigExecBakedProgramImpl &B = worker->B;
    step->BeginRun();
    if (step->kind == RigExecBakedStepKind::WeightPacket ||
        step->kind == RigExecBakedStepKind::VolumePlacements) {
        return _FrozenWeightStep(worker, step, index, inputs, time);
    }
    // SnapshotFinals rides the shared pose runner below: pure slot math
    // over provider finals (bakedPose.cpp), no stage reads.
    if (step->kind == RigExecBakedStepKind::RevisionStatic) {
        return _FrozenRevisionStatic(worker, step, index, inputs);
    }
    if (step->kind == RigExecBakedStepKind::Derived) {
        return _FrozenDerived(worker, step, index, inputs);
    }
    if (RigExecBakedIsGeometryStep(step->kind)) {
        RigExecBakedRunGeometryStep(&B, step, time);
    } else {
        RigExecBakedRunPoseStep(&B, step, time);
    }
    return true;
}

// The frozen region: RigExecBakedRunSteps (bakedSchedule.cpp:1833) with the
// serial executor INLINED rather than dispatched by environment -- sources
// in program order, the real closure, skips, then the closed set serially.
// No calibration, no step timing, no trace intervals: the worker shares the
// timing statics with nothing and reports no trace. It skips by CLUSTER, not
// by the live executors' step closure: every step of a cluster that holds a
// closed step runs, which is a superset of the closure and so the same
// answer -- a clean step re-run reads what it read last run.
bool
_FrozenRunSteps(_FrozenWorker *worker,
               const std::map<SdfPath, size_t> &index,
               const RigExecFrameInputs &inputs, UsdTimeCode time)
{
    RigExecBakedProgramImpl &B = worker->B;
    for (RigExecBakedStep &step : B.steps) {
        step.startUs = step.endUs = 0;
    }
    for (RigExecBakedStep &step : B.steps) {
        if (step.isSource) {
            if (!_FrozenStepBody(worker, &step, index, inputs, time)) {
                return false;
            }
            if (!step.snapshots.IsEmpty()) {
                B.runSnapshots.Merge(std::move(step.snapshots));
            }
            if (step.bail) {
                return false;
            }
        }
    }
    RigExecBakedComputeClosure(&B, time, /*force=*/false);
    for (RigExecBakedStep &step : B.steps) {
        if (step.isSource || B.closed.Test(step.cluster)) {
            continue;
        }
        step.MarkSkipped();
        if (RigExecBakedIsGeometryStep(step.kind)) {
            RigExecBakedSkipGeometryStep(&B, &step);
        }
    }
    B.clustering.lastRunTimed = false;
    for (RigExecBakedStep &step : B.steps) {
        if (step.isSource || !B.closed.Test(step.cluster)) {
            continue;
        }
        if (!_FrozenStepBody(worker, &step, index, inputs, time)) {
            return false;
        }
        if (!step.snapshots.IsEmpty()) {
            B.runSnapshots.Merge(std::move(step.snapshots));
        }
        if (step.bail) {
            return false;
        }
    }
    return true;
}

// The production cluster runner (plan 2.0): binds the worker's rebound
// clone -- the per-job program the caller cloned and restored retained
// slots into -- plus the retained handle it was rebound from, into the
// rebind context RigExecRunSparsePlan executes. One cluster runs through
// _FrozenStepBody, the exact per-step body _FrozenRunSteps dispatches,
// with the same snapshot merge and bail handling; the program parameter
// must name the bound clone (a foreign program declines rather than run
// against state the runner does not own). File-local beside the dispatch
// it shares: _FrozenStepBody takes the worker, so no header surface can
// name this factory -- the public production surface is
// RigExecRunPartialCone, which builds its context here.
RigExecClusterRebindContext
_MakeProductionClusterRunner(
    _FrozenWorker *worker, const std::map<SdfPath, size_t> &index,
    const RigExecFrameInputs &inputs, UsdTimeCode time,
    const std::shared_ptr<const void> &retained)
{
    RigExecClusterRebindContext rebind;
    if (!worker) {
        return rebind;
    }
    rebind.program = std::shared_ptr<RigExecBakedProgramImpl>(
        &worker->B, [](RigExecBakedProgramImpl *) {});
    rebind.retained = retained;
    rebind.runCluster =
        [worker, &index, &inputs, time](RigExecBakedProgramImpl &prog,
                                       int cluster) {
            if (!worker || &prog != &worker->B) {
                return false;
            }
            RigExecBakedProgramImpl &owned = worker->B;
            for (RigExecBakedStep &step : owned.steps) {
                if (step.cluster != cluster) {
                    continue;
                }
                if (!_FrozenStepBody(worker, &step, index, inputs, time)) {
                    return false;
                }
                if (!step.snapshots.IsEmpty()) {
                    owned.runSnapshots.Merge(std::move(step.snapshots));
                }
                if (step.bail) {
                    return false;
                }
            }
            return true;
        };
    return rebind;
}

// The frozen geometry epilogue: RigExecBakedPublishGeometry
// (bakedGeometry.cpp:2719) minus the adjuster ladder, which reads the stage
// (B.stage->GetPrimAtPath, unconditionally). No adjuster revision can reach
// here (the freeze refuses the op), so the sweep -- step diagnostics in
// program order, weight fields last-writer-wins per RevisionStatic step,
// movedProperties per ChainStatus/Derived step under the haveBase gates --
// is the whole function.
void
_FrozenPublishGeometry(RigExecBakedProgramImpl &B, RigExecRigPose *pose)
{
    for (const RigExecBakedStep &step : B.steps) {
        if (!RigExecBakedIsGeometryStep(step.kind)) {
            continue;
        }
        for (const std::string &diagnostic : step.diagnostics) {
            pose->diagnostics.push_back(diagnostic);
        }
        if (step.kind == RigExecBakedStepKind::RevisionStatic) {
            const auto &[chainIndex, revisionIndex] =
                B.revisionIndex[size_t(step.object)];
            const RigExecBakedProgramImpl::GeomChain &chain =
                B.chains[size_t(chainIndex)];
            const RigExecBakedProgramImpl::GeomRevision &revision =
                chain.revisions[size_t(revisionIndex)];
            if (chain.haveBase && revision.weightFieldPublished &&
                revision.weightObject >= 0 &&
                size_t(revision.weightObject) < B.weightObjects.size()) {
                RigExecResolvedWeightField &field =
                    pose->weightFields[
                        B.weightObjects[size_t(revision.weightObject)].path];
                field.target = revision.weightFieldTarget;
                field.weights = revision.weightField;
            }
        } else if (step.kind == RigExecBakedStepKind::ChainStatus) {
            RigExecBakedProgramImpl::GeomChain &chain =
                B.chains[size_t(step.object)];
            if (chain.haveBase) {
                pose->movedProperties[chain.target] = VtValue(chain.result);
            }
        } else if (step.kind == RigExecBakedStepKind::Derived) {
            const auto &[chainIndex, derivedIndex] =
                B.derivedIndex[size_t(step.object)];
            const RigExecBakedProgramImpl::GeomChain &chain =
                B.chains[size_t(chainIndex)];
            const RigExecBakedProgramImpl::GeomChain::Derived &derived =
                chain.derived[size_t(derivedIndex)];
            if (chain.haveBase && derived.haveBase) {
                pose->movedProperties[derived.target] =
                    VtValue(derived.result);
            }
        }
    }
}

// Runs one frozen frame: clone, patch, prologue, region, epilogue. Returns
// false to decline (the caller hands the generation back); a declined run
// leaves the pose invalid and publishes nothing.
bool
_RunFrozen(const RigExecFrozenEvalContext &context,
           const RigExecFrameInputs &inputs, RigExecRigPose *pose)
{
    if (!pose || !context.frozen) {
        return false;
    }
    const RigExecFrozenProgram &snapshot = *context.frozen;
    if (inputs.HasChainResolvedInputs()) {
        return false;
    }
    _FrozenWorker worker;
    _CloneImpl(snapshot.program, &worker.B);
    RigExecBakedProgramImpl &B = worker.B;
    B.resolvedInputs = &worker.resolved;
    B.volumeWeightMatrices = &worker.volumeWeightMatrices;
    B.chainSnapshots = &worker.chainSnapshots;
    B.profiler = &worker.profiler;
    B.interactiveOverrides = &inputs.overrides;
    B.jointSolverBinding = &snapshot.jointSolverBinding;
    B.guideTaps = &worker.nullTaps;
    B.solverGuidesEnabled = &worker.guidesEnabled;
    B.publishWeightFields =
        (context.flags & kRigExecFrozenPublishWeightFields) != 0;
    worker.guidesEnabled =
        (context.flags & kRigExecFrozenSolverGuidesEnabled) != 0;

    // Override flags for the job's overrides, exactly as SetOverrides
    // computes them; unplaceable declines (live runs dynamically).
    std::vector<char> flags;
    if (!_FrozenPlaceOverrides(B, inputs.overrides, &flags)) {
        return false;
    }
    if (flags.size() != B.overridden.size()) {
        return false;
    }
    B.overridden = flags;
    B.anyOverridden = false;
    for (char flag : B.overridden) {
        if (flag) {
            B.anyOverridden = true;
            break;
        }
    }

    std::map<SdfPath, size_t> index;
    for (size_t i = 0; i < inputs.values.size(); ++i) {
        index.emplace(inputs.values[i].path, i);
    }
    if (!_PatchInputs(B, snapshot, index, inputs)) {
        return false;
    }

    const UsdTimeCode time = inputs.time;
    RigExecRigPose working;
    if (!_FrozenPrologue(&worker, snapshot, inputs, index, time, &working)) {
        return false;
    }
    if (!_FrozenRunSteps(&worker, index, inputs, time)) {
        return false;
    }

    // Epilogue (bakedProgram.cpp:2644): chain lines first (live runs the
    // chains before everything), then the pose and geometry publication, the
    // work counters, the curvenet drain, and the summary line, in live's
    // order. No timing replay, calibration, or cone verification: the worker
    // shares those statics with nothing and reports no trace.
    for (const std::string &line : inputs.chainDiagnostics) {
        working.diagnostics.push_back(line);
    }
    if (!RigExecBakedPublishPose(&B, &working)) {
        return false;
    }
    if (snapshot.guideTapsPresent && worker.guidesEnabled) {
        // The guide block PublishPose skipped for the null taps
        // (bakedPose.cpp:3673), replayed from the aggregates.
        for (const int solverIndex : B.solverPublishOrder) {
            const auto &[solverPath, si] = B.solverArrays[size_t(solverIndex)];
            RigExecBakedEmplace(&working.solverFrames, B.solverArraysAscending,
                                solverPath, B.aggregates[size_t(si)].frames);
        }
    }
    if (B.volumeWeightMatrices) {
        working.weightFrames = *B.volumeWeightMatrices;
    }
    _FrozenPublishGeometry(B, &working);
    working.solverOverrideRounds += B.solverOverrideRounds;
    working.solverEvaluations += B.solverEvaluations;
    size_t chainsBuilt = 0, revisionsBuilt = 0;
    for (const RigExecBakedStep &step : B.steps) {
        working.moverGraphRevisionsExecuted += step.counters.revisionsExecuted;
        working.moverGraphRevisionsCreated += step.counters.revisionsCreated;
        working.moverGraphSchedulesBuilt += step.counters.schedulesBuilt;
        chainsBuilt += step.counters.chainsBuilt;
        revisionsBuilt += step.counters.revisionsBuilt;
    }
    for (std::string message : B.curvenetBindings.TakeDiagnostics()) {
        working.diagnostics.push_back(std::move(message));
    }
    working.diagnostics.push_back(
        "mover graph: " + std::to_string(chainsBuilt) + " chain(s), " +
        std::to_string(revisionsBuilt) + " revision(s); " +
        std::to_string(working.moverGraphRevisionsCreated) + " created, " +
        std::to_string(working.moverGraphRevisionsExecuted) + " executed, " +
        std::to_string(working.moverGraphSchedulesBuilt) +
        " schedule(s) built");
    working.valid = true;
    working.time = time;
    *pose = working;
    RigExecCapturePartialSlots(B, &_lastFrozenSlots, &_lastFrozenSlotBytes);
    return true;
}

void
RigExecPartialSlots::Capture(const RigExecBakedProgramImpl &program)
{
    poseWeights = program.poseWeights;
    weightPackets = program.weightPackets;
    posedM = program.posedM;
    finalMatrix = program.finalMatrix;
    baseMatrix = program.baseMatrix;
    base = program.base;
    fin = program.fin;
    aggregates = program.aggregates;
    deltaValues = program.deltaValues;
    deltaPresent = program.deltaPresent;
    solvers.resize(program.solvers.size());
    for (size_t s = 0; s < program.solvers.size(); ++s) {
        solvers[s].outFrames = program.solvers[s].outFrames;
        solvers[s].outPresent = program.solvers[s].outPresent;
        solvers[s].fallbackJoints = program.solvers[s].fallbackJoints;
    }
    commits.resize(program.commits.size());
    for (size_t c = 0; c < program.commits.size(); ++c) {
        commits[c].present = program.commits[c].present;
        commits[c].deltaOk = program.commits[c].deltaOk;
        commits[c].frames = program.commits[c].frames;
        commits[c].staged = program.commits[c].staged;
        commits[c].deltas = program.commits[c].deltas;
        commits[c].outcome = program.commits[c].outcome;
        commits[c].sources = program.commits[c].sources;
        commits[c].abandoned = program.commits[c].abandoned;
    }
    steps.resize(program.steps.size());
    for (size_t k = 0; k < program.steps.size(); ++k) {
        steps[k].diagnostics = program.steps[k].diagnostics;
        steps[k].counters = program.steps[k].counters;
        steps[k].bail = program.steps[k].bail;
    }
    volumeWeightMatrices.clear();
    if (program.volumeWeightMatrices) {
        volumeWeightMatrices = *program.volumeWeightMatrices;
    }
}

bool
RigExecPartialSlots::Restore(RigExecBakedProgramImpl *program) const
{
    if (!program) {
        return false;
    }
    RigExecBakedProgramImpl &B = *program;
    if (solvers.size() != B.solvers.size() ||
        commits.size() != B.commits.size() ||
        steps.size() != B.steps.size()) {
        return false;
    }
    B.poseWeights = poseWeights;
    B.weightPackets = weightPackets;
    B.posedM = posedM;
    B.finalMatrix = finalMatrix;
    B.baseMatrix = baseMatrix;
    B.base = base;
    B.fin = fin;
    B.aggregates = aggregates;
    B.deltaValues = deltaValues;
    B.deltaPresent = deltaPresent;
    for (size_t s = 0; s < B.solvers.size(); ++s) {
        B.solvers[s].outFrames = solvers[s].outFrames;
        B.solvers[s].outPresent = solvers[s].outPresent;
        B.solvers[s].fallbackJoints = solvers[s].fallbackJoints;
    }
    for (size_t c = 0; c < B.commits.size(); ++c) {
        B.commits[c].present = commits[c].present;
        B.commits[c].deltaOk = commits[c].deltaOk;
        B.commits[c].frames = commits[c].frames;
        B.commits[c].staged = commits[c].staged;
        B.commits[c].deltas = commits[c].deltas;
        B.commits[c].outcome = commits[c].outcome;
        B.commits[c].sources = commits[c].sources;
        B.commits[c].abandoned = commits[c].abandoned;
    }
    for (size_t k = 0; k < B.steps.size(); ++k) {
        B.steps[k].diagnostics = steps[k].diagnostics;
        B.steps[k].counters = steps[k].counters;
        B.steps[k].bail = steps[k].bail;
    }
    B.runSnapshots.Clear();
    if (B.volumeWeightMatrices) {
        *B.volumeWeightMatrices = volumeWeightMatrices;
    } else if (!volumeWeightMatrices.empty()) {
        return false;
    }
    return true;
}

size_t
RigExecPartialSlots::Bytes() const
{
    size_t total = sizeof(RigExecPartialSlots);
    total += poseWeights.size() * sizeof(float);
    for (const RigExecWeightPacket &packet : weightPackets) {
        total += sizeof(RigExecWeightPacket);
        total += packet.values.size() * sizeof(float);
        total += packet.indices.size() * sizeof(int);
    }
    total += posedM.size() * sizeof(GfMatrix4d);
    total += finalMatrix.size() * sizeof(GfMatrix4d);
    total += baseMatrix.size() * sizeof(GfMatrix4d);
    total += base.size() * sizeof(RigExecPointFrame);
    total += fin.size() * sizeof(RigExecPointFrame);
    total += aggregates.size() * sizeof(RigExecPointFrameArray);
    total += deltaValues.size() * sizeof(GfMatrix4d);
    total += deltaPresent.size() * sizeof(char);
    for (const SolverSlots &solver : solvers) {
        total += solver.outFrames.size() * sizeof(RigExecPointFrame);
        total += solver.outPresent.size() * sizeof(char);
        total += solver.fallbackJoints.size() * sizeof(SdfPath);
    }
    for (const CommitSlots &commit : commits) {
        total += commit.present.size() * sizeof(char);
        total += commit.deltaOk.size() * sizeof(char);
        total += commit.frames.size() * sizeof(RigExecPointFrame);
        total += commit.staged.size() * sizeof(RigExecPointFrame);
        total += commit.deltas.size() * sizeof(GfMatrix4d);
        total += commit.outcome.size() * sizeof(uint8_t);
        total += commit.sources.size() * sizeof(RigExecConstraintSource);
    }
    for (const StepSlots &step : steps) {
        for (const std::string &line : step.diagnostics) {
            total += line.size();
        }
        total += sizeof(RigExecBakedStepCounters) + sizeof(bool);
    }
    total += volumeWeightMatrices.size() *
             (sizeof(SdfPath) + sizeof(GfMatrix4d));
    return total;
}

bool
RigExecCapturePartialSlots(
    const RigExecBakedProgramImpl &program,
    std::shared_ptr<const void> *slotsOut, size_t *bytesOut)
{
    if (!slotsOut || !bytesOut) {
        return false;
    }
    auto slots = std::make_shared<RigExecPartialSlots>();
    slots->Capture(program);
    *bytesOut = slots->Bytes();
    *slotsOut = std::move(slots);
    return true;
}

bool
RigExecTakeLastFrozenSlots(std::shared_ptr<const void> *slotsOut,
                         size_t *bytesOut)
{
    if (!slotsOut || !bytesOut) {
        return false;
    }
    if (!_lastFrozenSlots) {
        return false;
    }
    *slotsOut = std::move(_lastFrozenSlots);
    *bytesOut = _lastFrozenSlotBytes;
    _lastFrozenSlots.reset();
    _lastFrozenSlotBytes = 0;
    return true;
}

RigExecPartialRunResult
RigExecRunPartialCone(
    const RigExecFrozenProgram &snapshot,
    const RigExecFrameInputs &freshInputs,
    const std::shared_ptr<const void> &baseSlots,
    const RigExecRigPose &basePose,
    const RigExecBakedClusterSet &planClusters, uint32_t contextFlags)
{
    RigExecPartialRunResult result;
    result.pose.time = freshInputs.time;
    result.pose.valid = false;
    if (!baseSlots) {
        return result;
    }
    const RigExecPartialSlots *slots =
        static_cast<const RigExecPartialSlots *>(baseSlots.get());
    if (!slots) {
        return result;
    }
    if (freshInputs.HasChainResolvedInputs()) {
        return result;
    }
    const size_t clusterCount =
        snapshot.program.clustering.clusters.size();
    const size_t planBits = planClusters.words.size() * 64;
    for (size_t c = clusterCount; c < planBits; ++c) {
        if ((planClusters.words[c / 64] >> (c % 64)) & 1ull) {
            return result;
        }
    }
    _FrozenWorker worker;
    _CloneImpl(snapshot.program, &worker.B);
    RigExecBakedProgramImpl &B = worker.B;
    B.resolvedInputs = &worker.resolved;
    B.volumeWeightMatrices = &worker.volumeWeightMatrices;
    B.chainSnapshots = &worker.chainSnapshots;
    B.profiler = &worker.profiler;
    B.interactiveOverrides = &freshInputs.overrides;
    B.jointSolverBinding = &snapshot.jointSolverBinding;
    B.guideTaps = &worker.nullTaps;
    B.solverGuidesEnabled = &worker.guidesEnabled;
    B.publishWeightFields =
        (contextFlags & kRigExecFrozenPublishWeightFields) != 0;
    worker.guidesEnabled =
        (contextFlags & kRigExecFrozenSolverGuidesEnabled) != 0;
    if (!slots->Restore(&B)) {
        return result;
    }
    std::vector<char> flags;
    if (!_FrozenPlaceOverrides(B, freshInputs.overrides, &flags)) {
        return result;
    }
    if (flags.size() != B.overridden.size()) {
        return result;
    }
    B.overridden = flags;
    B.anyOverridden = false;
    for (char flag : B.overridden) {
        if (flag) {
            B.anyOverridden = true;
            break;
        }
    }
    std::map<SdfPath, size_t> index;
    for (size_t i = 0; i < freshInputs.values.size(); ++i) {
        index.emplace(freshInputs.values[i].path, i);
    }
    if (!_PatchInputs(B, snapshot, index, freshInputs)) {
        return result;
    }
    const UsdTimeCode time = freshInputs.time;
    RigExecRigPose working;
    if (!_FrozenPrologue(&worker, snapshot, freshInputs, index, time,
                         &working)) {
        return result;
    }
    for (RigExecBakedStep &step : B.steps) {
        step.startUs = step.endUs = 0;
    }
    for (RigExecBakedStep &step : B.steps) {
        if (step.cluster < 0 || planClusters.Test(step.cluster)) {
            continue;
        }
        step.MarkSkipped();
        if (RigExecBakedIsGeometryStep(step.kind)) {
            RigExecBakedSkipGeometryStep(&B, &step);
        }
    }
    B.clustering.lastRunTimed = false;
    RigExecClusterRebindContext rebind = _MakeProductionClusterRunner(
        &worker, index, freshInputs, time, baseSlots);
    RigExecSparsePlan sparsePlan;
    sparsePlan.verdict = RigExecSparseVerdict::Partial;
    sparsePlan.clusters = planClusters;
    const RigExecSparseExecution execution = RigExecRunSparsePlan(
        sparsePlan, B.clustering.topologicalOrder,
        RigExecMakeClusterRunner(rebind));
    if (!execution.completed) {
        return result;
    }
    result.executedClusters = execution.executedClusters;
    for (const std::string &line : freshInputs.chainDiagnostics) {
        working.diagnostics.push_back(line);
    }
    if (!RigExecBakedPublishPose(&B, &working)) {
        return result;
    }
    if (snapshot.guideTapsPresent && worker.guidesEnabled) {
        for (const int solverIndex : B.solverPublishOrder) {
            const auto &[solverPath, si] = B.solverArrays[size_t(solverIndex)];
            RigExecBakedEmplace(&working.solverFrames, B.solverArraysAscending,
                                solverPath, B.aggregates[size_t(si)].frames);
        }
    }
    if (B.volumeWeightMatrices) {
        working.weightFrames = *B.volumeWeightMatrices;
    }
    _FrozenPublishGeometry(B, &working);
    for (const RigExecBakedStep &step : B.steps) {
        if (!RigExecBakedIsGeometryStep(step.kind)) {
            continue;
        }
        if (step.cluster >= 0 && planClusters.Test(step.cluster)) {
            continue;
        }
        if (step.kind == RigExecBakedStepKind::ChainStatus) {
            const RigExecBakedProgramImpl::GeomChain &chain =
                B.chains[size_t(step.object)];
            const auto found = basePose.movedProperties.find(chain.target);
            if (found != basePose.movedProperties.end()) {
                working.movedProperties[chain.target] = found->second;
            } else {
                working.movedProperties.erase(chain.target);
            }
        } else if (step.kind == RigExecBakedStepKind::Derived) {
            const auto &[chainIndex, derivedIndex] =
                B.derivedIndex[size_t(step.object)];
            const RigExecBakedProgramImpl::GeomChain::Derived &derived =
                B.chains[size_t(chainIndex)].derived[size_t(derivedIndex)];
            const auto found =
                basePose.movedProperties.find(derived.target);
            if (found != basePose.movedProperties.end()) {
                working.movedProperties[derived.target] = found->second;
            } else {
                working.movedProperties.erase(derived.target);
            }
        } else if (step.kind == RigExecBakedStepKind::RevisionStatic) {
            const auto &[chainIndex, revisionIndex] =
                B.revisionIndex[size_t(step.object)];
            (void)chainIndex;
            (void)revisionIndex;
            const RigExecBakedProgramImpl::GeomChain &chain =
                B.chains[size_t(chainIndex)];
            const RigExecBakedProgramImpl::GeomRevision &revision =
                chain.revisions[size_t(revisionIndex)];
            if (revision.weightObject >= 0 &&
                size_t(revision.weightObject) < B.weightObjects.size()) {
                const SdfPath &path =
                    B.weightObjects[size_t(revision.weightObject)].path;
                const auto found = basePose.weightFields.find(path);
                if (found != basePose.weightFields.end()) {
                    working.weightFields[path] = found->second;
                } else {
                    working.weightFields.erase(path);
                }
            }
        }
    }
    working.solverOverrideRounds += B.solverOverrideRounds;
    working.solverEvaluations += B.solverEvaluations;
    size_t chainsBuilt = 0, revisionsBuilt = 0;
    for (const RigExecBakedStep &step : B.steps) {
        working.moverGraphRevisionsExecuted += step.counters.revisionsExecuted;
        working.moverGraphRevisionsCreated += step.counters.revisionsCreated;
        working.moverGraphSchedulesBuilt += step.counters.schedulesBuilt;
        chainsBuilt += step.counters.chainsBuilt;
        revisionsBuilt += step.counters.revisionsBuilt;
    }
    for (std::string message : B.curvenetBindings.TakeDiagnostics()) {
        working.diagnostics.push_back(std::move(message));
    }
    working.diagnostics.push_back(
        "mover graph: " + std::to_string(chainsBuilt) + " chain(s), " +
        std::to_string(revisionsBuilt) + " revision(s); " +
        std::to_string(working.moverGraphRevisionsCreated) + " created, " +
        std::to_string(working.moverGraphRevisionsExecuted) + " executed, " +
        std::to_string(working.moverGraphSchedulesBuilt) +
        " schedule(s) built");
    working.valid = true;
    working.time = time;
    result.pose = working;
    if (!RigExecCapturePartialSlots(B, &result.slots, &result.slotBytes)) {
        result.pose.valid = false;
        result.slots.reset();
        result.slotBytes = 0;
        return result;
    }
    result.completed = true;
    return result;
}

}  // namespace rigExec
