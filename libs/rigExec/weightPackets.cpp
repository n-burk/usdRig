#include "weightPackets.h"

#include "frameExtraction.h"

#include "rigExecMath/pointFrame.h"

#include "pxr/base/tf/staticTokens.h"

#include <algorithm>
#include <cmath>
#include <utility>
#include <numeric>

// The shape types and plane bounds the packet builders dispatch on. A table
// rather than function-local statics: these run inside step bodies and the
// weight oracle on workers, and Build touches the table on the owning thread
// (RigExecWeightPacketsTouchTokens).
TF_DEFINE_PRIVATE_TOKENS(
    _weightTokens,
    ((sphere, "RigExecSphereWeight"))
    ((plane, "RigExecPlaneWeight"))
    ((curve, "RigExecCurveWeight"))
    ((unbounded, "unbounded"))
    ((bounded, "bounded"))
);

namespace rigExec {

void
RigExecWeightPacketsTouchTokens()
{
    (void)_weightTokens.Get();
}

bool
RigExecApplyWeightRangePolicy(const TfToken &policy, float *w)
{
    if (!std::isfinite(*w)) {
        return false;
    }
    if (*w < 0.0f || *w > 1.0f) {
        if (policy == "clamp") {
            *w = std::min(std::max(*w, 0.0f), 1.0f);
            return true;
        }
        return false;
    }
    return true;
}

// Core behind both RigExecBuildStaticWeightPacket overloads: identical
// validation, canonicalization, and range policy over borrowed arrays.
// Null with a zero size stands for an empty array and is never
// dereferenced.
static RigExecWeightPacket
_BuildStaticWeightPacket(
    const TfToken &representation, const TfToken &rangePolicy,
    const float *values, size_t valuesSize,
    const int *indices, size_t indicesSize,
    float defaultWeight,RigExecWeightPacketWorkspace &workspace)
{
    RigExecWeightPacket packet;
    packet.representation = representation;
    packet.rangePolicy = rangePolicy;
    // Unknown structural tokens are rejected, never coerced (spec §4.1);
    // allowedTokens metadata alone is not validation.
    if (packet.representation != "constant" &&
        packet.representation != "dense" &&
        packet.representation != "sparse") {
        return packet;
    }
    if (packet.rangePolicy != "strict" && packet.rangePolicy != "clamp") {
        return packet;
    }
    packet.defaultWeight = defaultWeight;
    if ((valuesSize && !values) || (indicesSize && !indices)) {
        return packet;
    }

    auto &pairs=workspace.pairs;pairs.clear();
    {
        if (packet.representation == "sparse") {
            const size_t paired = std::min(valuesSize, indicesSize);
            for (size_t i = 0; i < paired; ++i) {
                pairs.emplace_back(indices[i], values[i]);
            }
            if (valuesSize != indicesSize) {
                return packet;  // size mismatch: invalid
            }
        } else if (valuesSize > 0) {
            // One sized copy, not a push_back loop: the caller already
            // holds the values in an array, and a dense paint is one
            // float per mesh element.
            packet.values.assign(values, values + valuesSize);
        }
    }
    // Canonical sorted sparse support; authored pair order is
    // non-semantic and duplicates are rejected (spec §4.1).
    if (packet.representation == "sparse") {
        const bool sameSupport=workspace.support.size()==indicesSize &&
            (indicesSize==0 || std::equal(workspace.support.begin(),workspace.support.end(),indices));
        if(!sameSupport) {
            if(indicesSize)workspace.support.assign(indices,indices+indicesSize);else workspace.support.clear();
            workspace.supportOrder.resize(indicesSize);
            std::iota(workspace.supportOrder.begin(),workspace.supportOrder.end(),size_t(0));
            std::sort(workspace.supportOrder.begin(),workspace.supportOrder.end(),
                [&](size_t a,size_t b){return indices[a]<indices[b];});
            workspace.uniqueSupport=true;
            for(size_t i=1;i<indicesSize;++i)
                if(indices[workspace.supportOrder[i-1]]==indices[workspace.supportOrder[i]]) {
                    workspace.uniqueSupport=false;break;
                }
        }
        if(workspace.uniqueSupport) {
            for(size_t i=0;i<indicesSize;++i) {
                const size_t source=workspace.supportOrder[i];pairs[i]={indices[source],values[source]};
            }
        } else std::sort(pairs.begin(), pairs.end());
        packet.indices.reserve(pairs.size());packet.values.reserve(pairs.size());
        for (size_t i = 0; i < pairs.size(); ++i) {
            if (i > 0 && pairs[i].first == pairs[i - 1].first) {
                return packet;  // duplicate index: invalid
            }
            packet.indices.push_back(pairs[i].first);
            packet.values.push_back(pairs[i].second);
        }
    }
    if (packet.representation == "dense" && packet.defaultWeight != 0.0f) {
        return packet;  // canonical dense default is zero
    }

    for (float &w : packet.values) {
        if (!RigExecApplyWeightRangePolicy(packet.rangePolicy, &w)) {
            return packet;
        }
    }
    if (!RigExecApplyWeightRangePolicy(
            packet.rangePolicy, &packet.defaultWeight)) {
        return packet;
    }
    packet.valid = true;
    return packet;
}

RigExecWeightPacket
RigExecBuildStaticWeightPacket(const RigExecStaticWeightInputs &inputs)
{
    const RigExecStaticWeightInputsView view{inputs.representation,inputs.rangePolicy,
        inputs.values.data(),inputs.values.size(),inputs.indices.data(),inputs.indices.size(),inputs.defaultWeight};
    return RigExecBuildStaticWeightPacket(view);
}

RigExecWeightPacket
RigExecBuildStaticWeightPacket(const RigExecStaticWeightInputsView &inputs)
{
    RigExecWeightPacketWorkspace workspace;
    return RigExecBuildStaticWeightPacket(inputs,&workspace);
}
RigExecWeightPacket
RigExecBuildStaticWeightPacket(const RigExecStaticWeightInputsView &inputs,
    RigExecWeightPacketWorkspace *workspace)
{
    if(!workspace)return RigExecBuildStaticWeightPacket(inputs);
    return _BuildStaticWeightPacket(inputs.representation,inputs.rangePolicy,
        inputs.values,inputs.valuesSize,inputs.indices,inputs.indicesSize,
        inputs.defaultWeight,*workspace);
}

RigExecWeightPacket
RigExecBuildDynamicWeightPacket(
    const RigExecDynamicWeightInputs &inputs, const RigExecWeightPacket *base)
{
    RigExecWeightPacket packet;
    packet.representation = inputs.representation;
    packet.rangePolicy = inputs.rangePolicy;
    if (packet.representation != "constant" &&
        packet.representation != "dense" &&
        packet.representation != "sparse") {
        return packet;
    }
    if (packet.rangePolicy != "strict" && packet.rangePolicy != "clamp") {
        return packet;
    }

    const float d = inputs.driver;
    const float s = inputs.scale;
    const float a = inputs.bias;

    if (!base) {
        // Without a base only constant is legal and b_i = 1 (spec §4.1).
        if (packet.representation != "constant") {
            return packet;
        }
        packet.defaultWeight = (1.0f * d) * s + a;
        if (!RigExecApplyWeightRangePolicy(
                packet.rangePolicy, &packet.defaultWeight)) {
            return packet;
        }
        packet.valid = true;
        return packet;
    }
    if (!base->valid || base->representation != packet.representation) {
        return packet;
    }

    // r_i = (b_i d) s + a over the base field; dense keeps canonical
    // packet default zero (spec §4.1).
    packet.indices = base->indices;
    packet.values.reserve(base->values.size());
    for (float b : base->values) {
        float r = (b * d) * s + a;
        if (!RigExecApplyWeightRangePolicy(packet.rangePolicy, &r)) {
            return packet;
        }
        packet.values.push_back(r);
    }
    if (packet.representation == "dense") {
        packet.defaultWeight = 0.0f;
    } else {
        packet.defaultWeight = (base->defaultWeight * d) * s + a;
        if (!RigExecApplyWeightRangePolicy(
                packet.rangePolicy, &packet.defaultWeight)) {
            return packet;
        }
    }
    packet.valid = true;
    return packet;
}

// Volumetric weight objects (spec §4.1 volumetric extension).
// One shared body: read the placement and the band, pick a distance
// function, remap, and publish a dense packet. Only the distance function
// and the extra inputs it needs differ between sphere, plane, and curve.

bool
RigExecRigidWorldToLocal(const RigExecPointFrame &posed, GfMatrix4d *result)
{
    if (!posed.IsValid() || posed.IsDegenerate()) {
        return false;
    }
    GfMatrix4d placement(1.0);
    if (!RigExecPointsToMatrix(
            RigExecIdentityLandmarks(), posed.points, &placement)) {
        return false;
    }
    for (int i = 0; i < 4; ++i) {
        for (int j = 0; j < 4; ++j) {
            if (!std::isfinite(placement[i][j])) {
                return false;
            }
        }
    }
    GfMatrix4d rigid = placement.RemoveScaleShear();
    const double det = rigid.GetDeterminant();
    if (!std::isfinite(det) || std::abs(det) < 1e-12) {
        return false;
    }
    *result = rigid.GetInverse();
    return true;
}

GfMatrix4d
RigExecVolumePlacement(const RigExecPointFrame &final)
{
    GfMatrix4d placement(1.0);
    if (!final.IsValid() || final.IsDegenerate()) {
        return placement;
    }
    for (const GfVec3d &point : final.points) {
        if (!std::isfinite(point[0]) || !std::isfinite(point[1]) ||
            !std::isfinite(point[2])) {
            return placement;
        }
    }
    // A failed decomposition writes the identity (pointFrame.cpp).
    RigExecPointsToMatrix(RigExecIdentityLandmarks(), final.points,
                          &placement);
    return placement;
}

namespace {

// Per-axis divisors have to describe a volume: see _CheckVolumePrologue,
// which is the only caller.
bool
_ValidateScales(const GfVec3f &scales)
{
    for (int a = 0; a < 3; ++a) {
        if (!std::isfinite(scales[a]) || scales[a] <= 0.0f) {
            return false;
        }
    }
    return true;
}

// True for the shapes whose distance function divides local coordinates
// by inputs.scales. The plane ignores them, so a plane whose scaleX is
// zero is still a perfectly good plane.
bool
_UsesAxisScales(const TfToken &typeName)
{
    return typeName == _weightTokens->sphere ||
           typeName == _weightTokens->curve;
}

// Everything a volumetric weight can settle before it looks at a single
// point: the structural tokens, the placement, and the divisors the shape
// is about to divide by. Kept separate from the rest of the prologue so a
// caller that has to gather a whole mesh to fill in targetPoints can ask
// first -- see RigExecVolumeWeightCanBuild.
bool
_CheckVolumePrologue(
    bool usesScales,
    const RigExecVolumeWeightInputs &inputs,
    GfMatrix4d *worldToLocal)
{
    // A generated field has a value at every element, so dense is the only
    // representation it can honestly publish. Rejected, never coerced.
    if (inputs.representation != "dense") {
        return false;
    }
    if (inputs.rangePolicy != "strict" && inputs.rangePolicy != "clamp") {
        return false;
    }
    if (!inputs.hasPlacement ||
        !RigExecRigidWorldToLocal(inputs.placement, worldToLocal)) {
        return false;
    }
    // Non-positive or non-finite collapses the volume, so it invalidates
    // the packet rather than dividing by zero.
    if (usesScales && !_ValidateScales(inputs.scales)) {
        return false;
    }
    return true;
}

// The shared prologue: structural tokens, placement, band, and the points
// the field is measured over. Returns false when the packet is invalid.
bool
_BeginVolumeWeight(
    bool usesScales,
    const RigExecVolumeWeightInputs &inputs,
    RigExecWeightPacket *packet,
    GfMatrix4d *worldToLocal,
    RigExecWeightPointView *points)
{
    packet->representation = inputs.representation;
    packet->rangePolicy = inputs.rangePolicy;
    if (!_CheckVolumePrologue(usesScales, inputs, worldToLocal)) {
        return false;
    }

    // rigExec:sampleSource, when authored, replaces the weightTarget for
    // SAMPLING only -- the weighted domain stays the target, so the two
    // must still agree in cardinality.
    const RigExecWeightPointView target=inputs.usePointViews?inputs.targetView:
        RigExecWeightPointView{inputs.targetPoints.data(),inputs.targetPoints.size()};
    const RigExecWeightPointView samples=inputs.usePointViews?inputs.sampleView:
        RigExecWeightPointView{inputs.samplePoints.data(),inputs.samplePoints.size()};
    const size_t targetCount = inputs.targetPointCount != 0
        ? inputs.targetPointCount : target.count;
    if (targetCount == 0) return false;
    if (samples.count != 0 && samples.count != targetCount) return false;
    if (samples.count == 0 && target.count != targetCount) return false;
    *points = samples.count == 0 ? target : samples;
    return true;
}

// The shared epilogue: range policy over the generated field.
bool
_FinishVolumeWeight(RigExecWeightPacket *packet, std::vector<float> *weights)
{
    for (float &w : *weights) {
        if (!RigExecApplyWeightRangePolicy(packet->rangePolicy, &w)) {
            return false;
        }
    }
    packet->values = std::move(*weights);
    packet->defaultWeight = 0.0f;  // canonical dense default
    packet->valid = true;
    return true;
}

// Divides local coordinates by the per-axis scales, turning the sphere's
// iso-surfaces into ellipsoids and the curve's tube into an elliptical
// one. Folded into the matrix so the hot loop stays a single transform.
GfMatrix4d
_ApplyAxisScales(const GfMatrix4d &worldToLocal, const GfVec3f &scales)
{
    GfMatrix4d divide(1.0);
    divide.SetScale(GfVec3d(1.0 / double(scales[0]), 1.0 / double(scales[1]),
                            1.0 / double(scales[2])));
    return worldToLocal * divide;
}

RigExecWeightPacket
_BuildSphereWeightPacket(const RigExecVolumeWeightInputs &inputs)
{
    RigExecWeightPacket packet;
    GfMatrix4d worldToLocal;
    RigExecWeightPointView points;
    if (!_BeginVolumeWeight(
            /* usesScales = */ true, inputs, &packet, &worldToLocal,
            &points)) {
        return packet;
    }
    if (!_ValidateScales(inputs.positiveScales) ||
        !_ValidateScales(inputs.negativeScales)) return packet;
    std::vector<float> weights;
    RigExecSphereWeightField(
        points, _ApplyAxisScales(worldToLocal, inputs.scales), inputs.params,
        &weights, inputs.positiveScales, inputs.negativeScales);
    if (!_FinishVolumeWeight(&packet, &weights)) {
        return RigExecWeightPacket();
    }
    return packet;
}

RigExecWeightPacket
_BuildPlaneWeightPacket(const RigExecVolumeWeightInputs &inputs)
{
    RigExecWeightPacket packet;
    GfMatrix4d worldToLocal;
    RigExecWeightPointView points;
    if (!_BeginVolumeWeight(
            /* usesScales = */ false, inputs, &packet, &worldToLocal,
            &points)) {
        return packet;
    }
    int axisIndex;
    if (inputs.planeAxis == "x") {
        axisIndex = 0;
    } else if (inputs.planeAxis == "y") {
        axisIndex = 1;
    } else if (inputs.planeAxis == "z") {
        axisIndex = 2;
    } else {
        return packet;  // unknown structural token: rejected, not coerced
    }

    // Bounded clips the field to the in-plane rectangle; unbounded is
    // the infinite half-space gradient. The extents are consulted ONLY in
    // the bounded arm: unbounded does not use them for the field (they
    // still size the drawn guide), so a bad extent there is a legibility
    // problem, not a reason to invalidate the whole rig.
    RigExecPlaneBounds extent;
    const RigExecPlaneBounds *extentPtr = nullptr;
    if (inputs.planeBounds == _weightTokens->bounded) {
        extent.extentU = inputs.extentU;
        extent.extentV = inputs.extentV;
        for (const float e : {extent.extentU, extent.extentV}) {
            if (!std::isfinite(e) || e <= 0.0f) {
                return packet;  // no such rectangle; same rule as scales
            }
        }
        extentPtr = &extent;
    } else if (inputs.planeBounds != _weightTokens->unbounded) {
        return packet;  // unknown structural token: rejected, not coerced
    }

    std::vector<float> weights;
    RigExecPlaneWeightField(
        points, worldToLocal, axisIndex, inputs.params, &weights, extentPtr);
    if (!_FinishVolumeWeight(&packet, &weights)) {
        return RigExecWeightPacket();
    }
    return packet;
}

RigExecWeightPacket
_BuildCurveWeightPacket(const RigExecVolumeWeightInputs &inputs)
{
    RigExecWeightPacket packet;
    GfMatrix4d worldToLocal;
    RigExecWeightPointView points;
    if (!_BeginVolumeWeight(
            /* usesScales = */ true, inputs, &packet, &worldToLocal,
            &points)) {
        return packet;
    }
    const RigExecWeightPointView curve=inputs.usePointViews?inputs.curveView:
        RigExecWeightPointView{inputs.curvePoints.data(),inputs.curvePoints.size()};
    if (curve.count == 0) {
        return packet;  // a curve weight with no curve is not a field
    }
    std::vector<float> weights;
    std::vector<GfVec3f> localCurve;
    RigExecCurveWeightField(
        points, curve,
        _ApplyAxisScales(worldToLocal, inputs.scales), inputs.params,
        &weights, inputs.localCurveScratch?inputs.localCurveScratch:&localCurve);
    if (!_FinishVolumeWeight(&packet, &weights)) {
        return RigExecWeightPacket();
    }
    return packet;
}

}  // namespace

bool
RigExecVolumeWeightCanBuild(
    const TfToken &typeName, const RigExecVolumeWeightInputs &inputs)
{
    GfMatrix4d worldToLocal(1.0);
    if (typeName == "RigExecSphereWeight" &&
        (!_ValidateScales(inputs.positiveScales) ||
         !_ValidateScales(inputs.negativeScales))) return false;
    return _CheckVolumePrologue(
        _UsesAxisScales(typeName), inputs, &worldToLocal);
}

RigExecWeightPacket
RigExecBuildVolumeWeightPacket(
    const TfToken &typeName, const RigExecVolumeWeightInputs &inputs)
{
    if (typeName == _weightTokens->sphere) {
        return _BuildSphereWeightPacket(inputs);
    }
    if (typeName == _weightTokens->plane) {
        return _BuildPlaneWeightPacket(inputs);
    }
    if (typeName == _weightTokens->curve) {
        return _BuildCurveWeightPacket(inputs);
    }
    // Shaped like every other rejection rather than a bare packet, so a
    // caller that mistypes the token sees the same failure object a
    // rejected representation produces instead of a subtly different one.
    RigExecWeightPacket packet;
    packet.representation = inputs.representation;
    packet.rangePolicy = inputs.rangePolicy;
    return packet;
}

// Composition. Every input is resolved to a dense field over the same
// element count before folding, so a static sparse paint and a generated
// volume field compose without either knowing about the other.
// Publishes one combine input as a dense read-only view, resolving
// exactly as the per-point loop did. Dense inputs borrow the packet's
// own values after the cardinality and negativity checks -- no copy;
// sparse scatters through a linear merge and anything else (constant
// or unknown) broadcasts the default, both into *scratch, rejecting
// negative resolved values in every arm as the loop did. The view
// stays alive until the next input resolves: packet storage outlives
// the synchronous fold, and the caller owns the scratch. ResolveAll
// is deliberately NOT used here: it rejects unsorted, duplicate, and
// out-of-range sparse support that Resolve tolerates, and it
// validates constant packets Resolve ignores.
static bool
_ResolveCombineInput(
    const RigExecWeightPacket &in, size_t elementCount,
    std::vector<float> *scratch, const float **data, size_t *size)
{
    if (in.representation == "dense") {
        if (in.values.size() != elementCount) {
            return false;
        }
        for (const float v : in.values) {
            if (v < 0.0f) {
                return false;
            }
        }
        *data = in.values.data();
        *size = in.values.size();
        return true;
    }
    if (in.representation != "sparse") {
        // Constant and unknown broadcast the default, which the
        // per-point loop rejected when negative.
        if (in.defaultWeight < 0.0f) {
            return false;
        }
        scratch->assign(elementCount, in.defaultWeight);
        *data = scratch->data();
        *size = scratch->size();
        return true;
    }
    if (in.indices.size() != in.values.size()) {
        return false;
    }
    scratch->assign(elementCount, in.defaultWeight);
    if (!std::is_sorted(in.indices.begin(), in.indices.end())) {
        // Unsorted support: no producer writes it (the static builder
        // canonicalizes), but a hand-built packet still resolves exactly
        // as before rather than through a merge that assumes order.
        for (size_t i = 0; i < elementCount; ++i) {
            (*scratch)[i] = in.Resolve(i, elementCount);
            if ((*scratch)[i] < 0.0f) {
                return false;
            }
        }
        *data = scratch->data();
        *size = scratch->size();
        return true;
    }
    // Linear merge: the support is sorted, so one pointer advanced past
    // each point finds the same first match a per-point lower_bound
    // would -- first-wins for duplicates, no match for out-of-range
    // entries -- in O(points + support) instead of O(points log support).
    size_t p = 0;
    const size_t n = in.indices.size();
    for (size_t i = 0; i < elementCount; ++i) {
        const int ii = static_cast<int>(i);
        while (p < n && in.indices[p] < ii) {
            ++p;
        }
        if (p < n && in.indices[p] == ii) {
            (*scratch)[i] = in.values[p];
            ++p;
        }
    }
    // The per-point loop rejected negative resolved values: one scan
    // over the merged field, so a negative default under full coverage
    // (which no point reads) still passes, exactly as before.
    for (size_t i = 0; i < elementCount; ++i) {
        if ((*scratch)[i] < 0.0f) {
            return false;
        }
    }
    *data = scratch->data();
    *size = scratch->size();
    return true;
}

RigExecWeightPacket
RigExecBuildCombineWeightPacket(
    const TfToken &representation, const TfToken &rangePolicy,
    const TfToken &combineMode,
    const std::vector<const RigExecWeightPacket *> &inputs,
    size_t weightTargetCount,
    float strength, float invert)
{
    RigExecWeightPacketWorkspace workspace;
    return RigExecBuildCombineWeightPacket(representation,rangePolicy,combineMode,
        inputs,weightTargetCount,strength,invert,&workspace);
}

RigExecWeightPacket
RigExecBuildCombineWeightPacket(
    const TfToken &representation, const TfToken &rangePolicy,
    const TfToken &combineMode,
    const std::vector<const RigExecWeightPacket *> &inputs,
    size_t weightTargetCount,
    float strength, float invert,RigExecWeightPacketWorkspace *workspace)
{
    if(!workspace)return RigExecBuildCombineWeightPacket(representation,rangePolicy,combineMode,
        inputs,weightTargetCount,strength,invert);
    RigExecWeightPacket packet;
    packet.representation = representation;
    packet.rangePolicy = rangePolicy;
    if (packet.representation != "dense") {
        return packet;
    }
    if (packet.rangePolicy != "strict" && packet.rangePolicy != "clamp") {
        return packet;
    }

    RigExecWeightCombine mode;
    if (combineMode == "multiply") {
        mode = RigExecWeightCombine::Multiply;
    } else if (combineMode == "add") {
        mode = RigExecWeightCombine::Add;
    } else if (combineMode == "subtract") {
        mode = RigExecWeightCombine::Subtract;
    } else if (combineMode == "max") {
        mode = RigExecWeightCombine::Max;
    } else if (combineMode == "min") {
        mode = RigExecWeightCombine::Min;
    } else if (combineMode == "average") {
        mode = RigExecWeightCombine::Average;
    } else if (combineMode == "overlay") {
        mode = RigExecWeightCombine::Overlay;
    } else {
        return packet;
    }

    // Element count. A dense input carries it; when every input is
    // CONSTANT -- which is the schema default for an authored weight, so
    // it is not an exotic case -- nothing among the inputs knows the
    // cardinality and the combine has to get it from its own
    // rigExec:weightTarget.
    // Reading the target here is not belt-and-braces: without it a
    // constant-only combine publishes an invalid packet and the mover
    // passes through, while the CPU oracle resolves every constant to
    // `count` values and moves the points. That divergence is a parity
    // mismatch, and it is reachable the first time someone multiplies
    // two freshly created static weights together.
    size_t elementCount = 0;
    for (const RigExecWeightPacket *in : inputs) {
        if (!in || !in->valid) {
            return packet;  // one bad input fails the fold atomically
        }
        if (in->representation == "dense") {
            if (elementCount && in->values.size() != elementCount) {
                return packet;
            }
            elementCount = in->values.size();
        }
    }
    if (!elementCount) {
        elementCount = weightTargetCount;
    }
    if (!elementCount) {
        return packet;
    }

    // Streamed fold: each input publishes a dense view and folds
    // straight into the accumulator, so a many-input combine holds
    // O(points) of temporary storage instead of O(inputs x points).
    // Dense inputs borrow the packet's values with no copy; the
    // scratch serves only sparse/constant expansion.
    std::vector<float> folded;
    auto &field=workspace->combineField;
    if (!RigExecCombineWeightFieldsStreamed(
            mode, inputs.size(), elementCount,
            [&](size_t k, const float **data, size_t *size) {
                return _ResolveCombineInput(
                    *inputs[k], elementCount, &field, data, size);
            },
            &folded)) {
        return packet;
    }
    for (float &w : folded) {
        w = (w + (1.0f - 2.0f * w) * invert) * strength;
        if (!RigExecApplyWeightRangePolicy(packet.rangePolicy, &w)) {
            return RigExecWeightPacket();
        }
    }
    packet.values = std::move(folded);
    packet.defaultWeight = 0.0f;
    packet.valid = true;
    return packet;
}

RigExecWeightPacket
RigExecBuildCombineWeightPacket(
    const TfToken &representation, const TfToken &rangePolicy,
    const TfToken &combineMode,
    const std::vector<RigExecWeightPacket> &inputs, size_t weightTargetCount,
    float strength, float invert)
{
    std::vector<const RigExecWeightPacket *> borrowed;
    borrowed.reserve(inputs.size());
    for (const RigExecWeightPacket &in : inputs) {
        borrowed.push_back(&in);
    }
    return RigExecBuildCombineWeightPacket(
        representation, rangePolicy, combineMode, borrowed,
        weightTargetCount, strength, invert);
}

}  // namespace rigExec
