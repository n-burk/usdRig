#include "weightPackets.h"

#include "frameExtraction.h"

#include "rigExecMath/pointFrame.h"

#include <algorithm>
#include <cmath>
#include <utility>

namespace rigExec {

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

RigExecWeightPacket
RigExecBuildStaticWeightPacket(const RigExecStaticWeightInputs &inputs)
{
    RigExecWeightPacket packet;
    packet.representation = inputs.representation;
    packet.rangePolicy = inputs.rangePolicy;
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
    packet.defaultWeight = inputs.defaultWeight;

    std::vector<std::pair<int, float>> pairs;
    {
        if (packet.representation == "sparse") {
            const size_t paired =
                std::min(inputs.values.size(), inputs.indices.size());
            for (size_t i = 0; i < paired; ++i) {
                pairs.emplace_back(inputs.indices[i], inputs.values[i]);
            }
            if (inputs.values.size() != inputs.indices.size()) {
                return packet;  // size mismatch: invalid
            }
        } else {
            // One sized copy, not a push_back loop: the caller already
            // holds the values in a vector, and a dense paint is one
            // float per mesh element.
            packet.values = inputs.values;
        }
    }
    // Canonical sorted sparse support; authored pair order is
    // non-semantic and duplicates are rejected (spec §4.1).
    if (packet.representation == "sparse") {
        std::sort(pairs.begin(), pairs.end());
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

// ---------------------------------------------------------------------------
// Volumetric weight objects (spec §4.1 volumetric extension).
//
// One shared body: read the placement and the band, pick a distance
// function, remap, and publish a dense packet. Only the distance function
// and the extra inputs it needs differ between sphere, plane, and curve.
// ---------------------------------------------------------------------------

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
    static const TfToken sphere("RigExecSphereWeight");
    static const TfToken curve("RigExecCurveWeight");
    return typeName == sphere || typeName == curve;
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
    const std::vector<GfVec3f> **points)
{
    packet->representation = inputs.representation;
    packet->rangePolicy = inputs.rangePolicy;
    if (!_CheckVolumePrologue(usesScales, inputs, worldToLocal)) {
        return false;
    }

    // rigExec:sampleSource, when authored, replaces the weightTarget for
    // SAMPLING only -- the weighted domain stays the target, so the two
    // must still agree in cardinality.
    if (inputs.targetPoints.empty()) {
        return false;
    }
    if (!inputs.samplePoints.empty() &&
        inputs.samplePoints.size() != inputs.targetPoints.size()) {
        return false;  // a reference shape of another cardinality is a lie
    }
    // A pointer, not a copy: the sampled set is one of the two arrays the
    // caller already owns, and a volume weight is measured over a whole
    // mesh, so copying it per frame would be the single largest cost in
    // this file.
    *points = inputs.samplePoints.empty() ? &inputs.targetPoints
                                          : &inputs.samplePoints;
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
    const std::vector<GfVec3f> *points = nullptr;
    if (!_BeginVolumeWeight(
            /* usesScales = */ true, inputs, &packet, &worldToLocal,
            &points)) {
        return packet;
    }
    std::vector<float> weights;
    RigExecSphereWeightField(
        *points, _ApplyAxisScales(worldToLocal, inputs.scales), inputs.params,
        &weights);
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
    const std::vector<GfVec3f> *points = nullptr;
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
    static const TfToken unbounded("unbounded");
    static const TfToken bounded("bounded");
    RigExecPlaneBounds extent;
    const RigExecPlaneBounds *extentPtr = nullptr;
    if (inputs.planeBounds == bounded) {
        extent.extentU = inputs.extentU;
        extent.extentV = inputs.extentV;
        for (const float e : {extent.extentU, extent.extentV}) {
            if (!std::isfinite(e) || e <= 0.0f) {
                return packet;  // no such rectangle; same rule as scales
            }
        }
        extentPtr = &extent;
    } else if (inputs.planeBounds != unbounded) {
        return packet;  // unknown structural token: rejected, not coerced
    }

    std::vector<float> weights;
    RigExecPlaneWeightField(
        *points, worldToLocal, axisIndex, inputs.params, &weights, extentPtr);
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
    const std::vector<GfVec3f> *points = nullptr;
    if (!_BeginVolumeWeight(
            /* usesScales = */ true, inputs, &packet, &worldToLocal,
            &points)) {
        return packet;
    }
    if (inputs.curvePoints.empty()) {
        return packet;  // a curve weight with no curve is not a field
    }
    std::vector<float> weights;
    RigExecCurveWeightField(
        *points, inputs.curvePoints,
        _ApplyAxisScales(worldToLocal, inputs.scales), inputs.params,
        &weights);
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
    return _CheckVolumePrologue(
        _UsesAxisScales(typeName), inputs, &worldToLocal);
}

RigExecWeightPacket
RigExecBuildVolumeWeightPacket(
    const TfToken &typeName, const RigExecVolumeWeightInputs &inputs)
{
    static const TfToken sphere("RigExecSphereWeight");
    static const TfToken plane("RigExecPlaneWeight");
    static const TfToken curve("RigExecCurveWeight");
    if (typeName == sphere) {
        return _BuildSphereWeightPacket(inputs);
    }
    if (typeName == plane) {
        return _BuildPlaneWeightPacket(inputs);
    }
    if (typeName == curve) {
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
RigExecWeightPacket
RigExecBuildCombineWeightPacket(
    const TfToken &representation, const TfToken &rangePolicy,
    const TfToken &combineMode,
    const std::vector<RigExecWeightPacket> &inputs, size_t weightTargetCount,
    float strength, float invert)
{
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
    //
    // Reading the target here is not belt-and-braces: without it a
    // constant-only combine publishes an invalid packet and the mover
    // passes through, while the CPU oracle resolves every constant to
    // `count` values and moves the points. That divergence is a parity
    // mismatch, and it is reachable the first time someone multiplies
    // two freshly created static weights together.
    size_t elementCount = 0;
    for (const RigExecWeightPacket &in : inputs) {
        if (!in.valid) {
            return packet;  // one bad input fails the fold atomically
        }
        if (in.representation == "dense") {
            if (elementCount && in.values.size() != elementCount) {
                return packet;
            }
            elementCount = in.values.size();
        }
    }
    if (!elementCount) {
        elementCount = weightTargetCount;
    }
    if (!elementCount) {
        return packet;
    }

    std::vector<std::vector<float>> fields;
    fields.reserve(inputs.size());
    for (const RigExecWeightPacket &in : inputs) {
        std::vector<float> field(elementCount);
        for (size_t i = 0; i < elementCount; ++i) {
            field[i] = in.Resolve(i, elementCount);
            if (field[i] < 0.0f) {
                return packet;  // cardinality mismatch
            }
        }
        fields.push_back(std::move(field));
    }

    std::vector<float> folded;
    if (!RigExecCombineWeightFields(mode, fields, elementCount, &folded)) {
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

}  // namespace rigExec
