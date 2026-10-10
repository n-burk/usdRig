// Geometry chains, weight fields, and skin-layout inputs.

#include "rigEvaluatorInternal.h"
#include "bakedProgramImpl.h"
#include "frameExtraction.h"
#include "rigExecMath/geometryKernels.h"
#include "rigEvaluatorConstraints.h"
#include "movers/moverRegistry.h"
#include "weightReference.h"
#include "scalarReference.h"
#include "rigExecMath/envelope.h"
#include "rigExecMath/weightFields.h"
#include "rigExecScene/weightProgram.h"

#include "pxr/base/ts/spline.h"
#include "pxr/usd/usd/attribute.h"
#include "pxr/usd/usd/prim.h"
#include "pxr/usd/usd/relationship.h"
#include "pxr/usd/usdGeom/pointBased.h"

#include <algorithm>
#include <cmath>
#include <set>
#include <unordered_map>

namespace rigExec {

namespace evaluatorDetail {

bool
_VolumeWeightSamplesInFlight(const UsdPrim &weight, bool *inFlight,
                             std::string *error)
{
    *inFlight = false;
    for (const char *name : {"rigExec:weightTarget", "rigExec:curve"}) {
        RigExecReadPhase phase;std::string why;
        if (!RigExecResolveReadPhase(weight.GetRelationship(TfToken(name)),&phase,&why)) {
            *error=why;return false;
        }
        if (!phase.IsBase())*inFlight=true;
    }
    return true;
}

}  // namespace evaluatorDetail

using namespace evaluatorDetail;

namespace {

const TfToken _kGeoPoints("points");
const TfToken _kGeoSmoothProfile("smooth");

} // namespace

namespace evaluatorDetail {

/// Bakes a volumetric weight's distance-to-weight remap into the lookup
/// table its exec kernel consumes.
///
/// The named profiles bake analytically; `curve` resamples the Ts spline
/// authored on rigExec:falloffCurve. Both land in the same table, so the
/// kernel has one remap path and an author switching between a preset and
/// a hand-drawn curve changes only the numbers.
std::vector<float>
_BakeFalloffLut(const UsdPrim &prim)
{
    TfToken profile = _kGeoSmoothProfile;
    if (UsdAttribute a = prim.GetAttribute(_falloffProfileAttr)) {
        a.Get(&profile);
    }
    const UsdAttribute curve = prim.GetAttribute(_falloffCurveAttr);
    if (profile != "curve" || !curve || !curve.HasSpline())
        return RigExecBakeWeightFalloff(profile);
    const TsSpline spline=curve.GetSpline();
    return RigExecBakeWeightFalloff(profile,&spline);
}

// Resolves a READ-side geometry input: naming a PointBased prim means its
// .points property, because a geometry input has exactly one thing to read.
// Property paths stay exact (spec §4.2).
// This rule is deliberately NOT applied to write targets. On the write side a
// bare prim path names the transform domain and <prim>.points names the
// geometry domain -- two different write sets on the same prim -- so inferring
// between them is what made a constraint unable to target a Mesh at all.
SdfPath
_ResolveGeometryInput(const UsdStageRefPtr &stage, const SdfPath &target)
{
    if (target.IsPrimPath()) {
        const UsdPrim prim = stage->GetPrimAtPath(target);
        if (prim && prim.IsA<UsdGeomPointBased>()) {
            return target.AppendProperty(_kGeoPoints);
        }
    }
    return target;
}

} // namespace evaluatorDetail

bool
RigExecRigEvaluator::_ReadTargetPoints(
    const UsdPrim &prim, const TfToken &relationshipName, UsdTimeCode time,
    std::vector<GfVec3f> *points) const
{
    points->clear();
    SdfPathVector targets;
    if (UsdRelationship rel = prim.GetRelationship(relationshipName)) {
        rel.GetTargets(&targets);
    }
    if (targets.size() != 1) {
        return false;
    }
    const SdfPath canonical = _ResolveGeometryInput(_stage, targets[0]);
    // An admitted upstream value is the attribute's authored value, for the
    // oracle as for every other reader (read-only while a generation runs).
    VtVec3fArray value;
    const auto upstream = _upstreamValues.find(canonical);
    if (upstream != _upstreamValues.end() &&
        upstream->second.IsHolding<VtVec3fArray>()) {
        value = upstream->second.UncheckedGet<VtVec3fArray>();
    } else {
        const UsdAttribute attr = _stage->GetAttributeAtPath(canonical);
        if (!attr || !attr.Get(&value, time)) {
            return false;
        }
    }
    points->assign(value.begin(), value.end());
    return true;
}

void
RigExecRigEvaluator::_ResolveSkinLayoutInputs() const
{
    _skinLayoutInputs.clear();
    _skinLayoutInputsValid = true;
    for (const RigExecMoverRecord &record : _movers) {
        const RigExecMoverHandler *layoutHandler =
            record.handler;
        if (!layoutHandler || layoutHandler->layoutAttributes.empty()) {
            continue;
        }
        const UsdPrim prim = _stage->GetPrimAtPath(record.moverPath);
        if (!prim) {
            continue;
        }
        for (const TfToken &name : layoutHandler->layoutAttributes) {
            // The SAME walk the value is read through
            // (RigExecResolvedInputs::GetAttribute): a single authored
            // connection per hop, the resolved map consulted at every hop,
            // cycles refused. Every path along it is a path an override can
            // stand on and be seen by the read, so every path along it
            // belongs in this set -- an override one hop upstream of a
            // connected rigExec:jointIndices is the case that makes the
            // difference between a re-read and a silently stale deformation.
            UsdAttribute attribute = prim.GetAttribute(name);
            if (!attribute) {
                // Not authored and not in the schema: an override could
                // still create the opinion the read would find, so the
                // property itself is named even where the attribute is not.
                _skinLayoutInputs.insert(
                    record.moverPath.AppendProperty(name));
                continue;
            }
            while (attribute &&
                   _skinLayoutInputs.insert(attribute.GetPath()).second) {
                SdfPathVector connections;
                if (attribute.HasAuthoredConnections()) {
                    attribute.GetConnections(&connections);
                }
                if (connections.size() != 1) {
                    break;
                }
                attribute = _stage->GetAttributeAtPath(connections[0]);
            }
        }
    }
}

bool
RigExecRigEvaluator::_OverridesReachSkinLayout(
    const std::vector<RigExecValueOverride> &overrides) const
{
    if (overrides.empty()) {
        return false;
    }
    if (!_skinLayoutInputsValid) {
        _ResolveSkinLayoutInputs();
    }
    for (const RigExecValueOverride &o : overrides) {
        // A computation override names no property, so there is nothing to
        // compare it against: it is taken to reach everything.
        if (o.attribute.IsEmpty()) {
            return true;
        }
        if (_skinLayoutInputs.count(o.prim.AppendProperty(o.attribute))) {
            return true;
        }
    }
    return false;
}

size_t
RigExecRigEvaluator::GetSkinTopologyCacheSize() const
{
    if (!_bakedProgram || _skinTopologyObservationPending) return 0;
    const auto &program = _bakedProgram->GetStepGraph();
    // Cache size counts retained answers, including invalid/dynamic layouts,
    // once per mover, as the original cache did for remembered refusals.
    std::set<SdfPath> answered;
    const auto count = [&](const auto &revision) {
        if ((revision.layoutFixed || revision.layoutRan) && revision.layoutHandle)
            answered.insert(revision.moverPath);
    };
    for (const auto &chain : program.chains) {
        for (const auto &revision : chain.revisions) count(revision);
        for (const auto &derived : chain.derived) count(derived.revision);
    }
    return answered.size();
}

size_t
RigExecRigEvaluator::GetEpochRestFrameCount() const
{
    if (!_bakedProgram) return 0;
    const auto &B = _bakedProgram->GetStepGraph();
    const auto &P = B.providerProgram;
    std::map<RigExecValueId, const RigExecProviderOp *> producers;
    for (const auto &op : P.ops) producers.emplace(op.output, &op);
    std::map<RigExecValueId, size_t> raw;
    for (size_t k = 0; k < P.sampled.size(); ++k) raw.emplace(P.sampled[k].value, k);
    std::map<RigExecValueId, const RigExecProviderExternalInput *> external;
    for (const auto &input : P.externalInputs) external.emplace(input.value, &input);
    std::map<RigExecValueId, uint8_t> valueState;
    std::vector<uint8_t> restState(B.paths.size(), 0);
    std::map<int, SdfPath> interveningAnchors;
    for (size_t k = 0; k < B.interveningSlots.size() && k < B.interveningAnchors.size(); ++k)
        interveningAnchors.emplace(B.interveningSlots[k], B.interveningAnchors[k]);
    std::map<SdfPath, bool> xformState;
    const auto staticXform = [&](const SdfPath &path) {
        const auto previous = xformState.find(path);
        if (previous != xformState.end()) return previous->second;
        bool fixed = bool(B.sceneDescriptors);
        if (fixed) {
            const auto &scene = *B.sceneDescriptors;
            fixed = scene.nodes.count(path) != 0;
            const auto orderPath = path.AppendProperty(TfToken("xformOpOrder"));
            const auto order = scene.attributes.find(orderPath);
            if (fixed && order != scene.attributes.end()) {
                const auto constant = [](const auto &attribute) {
                    return !attribute.fact.mightBeTimeVarying && attribute.fact.sampleTimes.empty() &&
                        attribute.fact.spline.IsEmpty() && !attribute.fact.hasAuthoredConnections;
                };
                fixed = constant(order->second);
                if (fixed && !order->second.inputs.empty() &&
                    order->second.inputs[0].raw.IsHolding<VtTokenArray>()) {
                    for (const auto &token : order->second.inputs[0].raw.UncheckedGet<VtTokenArray>()) {
                        auto name = token.GetString();
                        if (name == "!resetXformStack!") break;
                        if (name.compare(0, 8, "!invert!") == 0) name.erase(0, 8);
                        const auto op = scene.attributes.find(path.AppendProperty(TfToken(name)));
                        if (op == scene.attributes.end() || !constant(op->second)) { fixed = false; break; }
                    }
                }
            }
        }
        xformState.emplace(path, fixed);
        return fixed;
    };
    const auto staticIntervening = [&](int slot) {
        const auto anchor = interveningAnchors.find(slot);
        if (anchor == interveningAnchors.end())
            return !B.ladders[size_t(slot)].interveningSpace.varying;
        for (auto path = B.paths[size_t(slot)].GetParentPath();
             !path.IsEmpty() && path != anchor->second; path = path.GetParentPath())
            if (!staticXform(path)) return false;
        return true;
    };
    std::function<bool(int)> staticRest;
    std::function<bool(RigExecValueId)> staticValue;
    staticValue = [&](RigExecValueId value) {
        if (value == RigExecNoProviderValue) return true;
        auto &state = valueState[value];
        if (state) return state == 2;
        state = 1; // An unresolved recurrence is not a static proof.
        bool fixed = false;
        const auto source = raw.find(value);
        const auto input = external.find(value);
        const auto producer = producers.find(value);
        if (source != raw.end()) {
            const size_t k = source->second;
            fixed = k < B.providerLeaves.varying.size() && !B.providerLeaves.varying[k] &&
                k < B.providerLeafBlocked.size() && !B.providerLeafBlocked[k];
        } else if (input != external.end()) {
            const auto slot = B.index.find(input->second->owner);
            if (slot != B.index.end() && input->second->computation == "computeRestFrame")
                fixed = staticRest(slot->second);
            else if (slot != B.index.end() && input->second->computation == "interveningSpace")
                fixed = staticIntervening(slot->second);
            // Current pose/default-frame bridges are produced state, not constants.
        } else if (producer != producers.end()) {
            const auto &op = *producer->second;
            const auto at = [&](size_t k) {
                return k < op.inputs.size() ? op.inputs[k] : RigExecNoProviderValue;
            };
            if (op.kind == RigExecProviderOpKind::Attribute) {
                // A constant authored connection remains a constant graph input.
                fixed = staticValue(at(1) != RigExecNoProviderValue ? at(1) : at(0));
            } else if (op.kind == RigExecProviderOpKind::SpaceExpression) {
                if (at(1) != RigExecNoProviderValue) fixed = staticValue(at(1));
                else {
                    const auto *authored = B.providerValues.Read<GfMatrix4d>(at(0));
                    fixed = staticValue(at(0)) &&
                        (authored && *authored != GfMatrix4d(1) ? true : staticValue(at(2)));
                }
            } else {
                fixed = true;
                for (const auto id : op.inputs) fixed = staticValue(id) && fixed;
                for (const auto &xform : op.xforms) fixed = staticValue(xform.raw) && fixed;
            }
        }
        // Unproduced central/property/routed values have no constant-source proof.
        state = fixed ? 2 : 3;
        return fixed;
    };
    staticRest = [&](int slot) {
        if (slot < 0) return true;
        if (size_t(slot) >= restState.size()) return false;
        auto &state = restState[size_t(slot)];
        if (state) return state == 2;
        state = 1;
        if (B.slotKind[size_t(slot)] != RigExecBakedSlotKind::FirstFramePose) {
            state = 2; // Namespace-only slots contribute the captured identity rest.
            return true;
        }
        const auto &ladder = B.ladders[size_t(slot)];
        bool fixed = !ladder.restSpace.varying && staticIntervening(slot);
        for (const auto &input : ladder.restAvars) fixed = !input.varying && fixed;
        if (ladder.spaceValues[0] >= 0) fixed = staticValue(uint64_t(ladder.spaceValues[0])) && fixed;
        fixed = staticRest(B.parent[size_t(slot)]) && fixed;
        state = fixed ? 2 : 3;
        return fixed;
    };
    size_t count = 0;
    for (size_t slot = 0; slot < B.paths.size(); ++slot)
        if (B.slotKind[slot] == RigExecBakedSlotKind::FirstFramePose &&
            slot < B.restFrames.size() && B.restFrames[slot].IsValid() &&
            !B.restFrames[slot].IsDegenerate() && staticRest(int(slot))) ++count;
    return count;
}
size_t
RigExecRigEvaluator::GetBlendSampleCacheSize() const
{
    return _blendSampleShapes.GetSize();
}

} // namespace rigExec
