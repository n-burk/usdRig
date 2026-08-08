//
// RigExec volumetric weight-field kernels implementation.
//
#include "weightFields.h"

#include <algorithm>
#include <cmath>
#include <limits>

namespace rigExec {

namespace {

inline float
_Clamp01(float x)
{
    return std::min(1.0f, std::max(0.0f, x));
}

}  // namespace

std::vector<float>
RigExecBuildFalloffLut(RigExecFalloffProfile profile, size_t count)
{
    if (count < 2) {
        return {};
    }
    std::vector<float> lut(count);
    for (size_t i = 0; i < count; ++i) {
        const float r = float(i) / float(count - 1);
        switch (profile) {
        case RigExecFalloffProfile::Linear:
            lut[i] = r;
            break;
        case RigExecFalloffProfile::Smooth:
            lut[i] = r * r * (3.0f - 2.0f * r);
            break;
        case RigExecFalloffProfile::EaseIn:
            lut[i] = r * r;
            break;
        case RigExecFalloffProfile::EaseOut:
            lut[i] = 1.0f - (1.0f - r) * (1.0f - r);
            break;
        case RigExecFalloffProfile::Constant:
            lut[i] = r > 0.0f ? 1.0f : 0.0f;
            break;
        }
    }
    return lut;
}

float
RigExecSampleFalloffLut(const std::vector<float> &curve, float r)
{
    if (curve.size() < 2) {
        return r;  // empty or degenerate table is the identity
    }
    const float x = _Clamp01(r) * float(curve.size() - 1);
    // floor, not truncation: x is already non-negative here, but keeping
    // the two spellings distinct is what stops a later signed input from
    // silently indexing backwards.
    const size_t i = std::min(curve.size() - 2,
                              size_t(std::floor(x)));
    const float t = x - float(i);
    return curve[i] + (curve[i + 1] - curve[i]) * t;
}

float
RigExecEvaluateFalloff(float distance, const RigExecFalloffParams &p)
{
    const float span = p.falloffMax - p.falloffMin;
    float u;
    if (std::abs(span) <= std::numeric_limits<float>::min()) {
        // Degenerate band: a hard step at the shared distance. Anything
        // strictly inside is fully on, anything at or beyond fully off.
        u = distance < p.falloffMin ? 0.0f : 1.0f;
    } else {
        u = _Clamp01((distance - p.falloffMin) / span);
    }
    // invert exchanges the ends continuously rather than by a branch, so
    // an animated invert sweeps rather than popping.
    u = u + (1.0f - 2.0f * u) * p.invert;
    const float w = RigExecSampleFalloffLut(p.curve, 1.0f - u);
    // Deliberately unclamped after the strength multiply: rangePolicy is
    // the authority on out-of-range weights (see the header).
    return w * p.strength;
}

float
RigExecSphereDistance(const GfVec3f &p)
{
    return p.GetLength();
}

float
RigExecPlaneDistance(const GfVec3f &p, int axis)
{
    if (axis < 0 || axis > 2) {
        return 0.0f;
    }
    return p[axis];
}

bool
RigExecPlaneWithinBounds(
    const GfVec3f &p, int axis, const RigExecPlaneBounds &bounds)
{
    if (axis < 0 || axis > 2) {
        return false;
    }
    // The same (axis+1, axis+2) pair the guide draws its rectangle over,
    // so "inside the field" and "inside the drawn square" are the same
    // test written twice rather than two conventions that can drift.
    const float u = p[(axis + 1) % 3];
    const float v = p[(axis + 2) % 3];
    return std::abs(u) <= bounds.extentU && std::abs(v) <= bounds.extentV;
}

float
RigExecSegmentDistance(const GfVec3f &p, const GfVec3f &a, const GfVec3f &b)
{
    const GfVec3f ab = b - a;
    const float lengthSq = ab.GetLengthSq();
    if (lengthSq <= 1e-20f) {
        return (p - a).GetLength();  // degenerate segment is its endpoint
    }
    const float t = _Clamp01(GfDot(p - a, ab) / lengthSq);
    return (p - (a + ab * t)).GetLength();
}

float
RigExecCurveDistance(
    const GfVec3f &p, const GfVec3f *curvePoints, size_t count)
{
    if (count == 0 || !curvePoints) {
        return std::numeric_limits<float>::infinity();
    }
    if (count == 1) {
        return (p - curvePoints[0]).GetLength();
    }
    float best = std::numeric_limits<float>::infinity();
    for (size_t i = 0; i + 1 < count; ++i) {
        best = std::min(
            best, RigExecSegmentDistance(p, curvePoints[i], curvePoints[i + 1]));
    }
    return best;
}

namespace {

// One transform of a point into the volume's local space. GfMatrix4d is
// row-vector here (see the ribbon transport kernel), so TransformAffine
// is the correct spelling rather than a hand-rolled multiply.
inline GfVec3f
_ToLocal(const GfMatrix4d &worldToLocal, const GfVec3f &p)
{
    return GfVec3f(worldToLocal.TransformAffine(GfVec3d(p)));
}

}  // namespace

void
RigExecSphereWeightField(
    const std::vector<GfVec3f> &points,
    const GfMatrix4d &worldToLocal,
    const RigExecFalloffParams &params,
    std::vector<float> *weights)
{
    weights->resize(points.size());
    for (size_t i = 0; i < points.size(); ++i) {
        const float d =
            RigExecSphereDistance(_ToLocal(worldToLocal, points[i]));
        (*weights)[i] = RigExecEvaluateFalloff(d, params);
    }
}

void
RigExecPlaneWeightField(
    const std::vector<GfVec3f> &points,
    const GfMatrix4d &worldToLocal,
    int axis,
    const RigExecFalloffParams &params,
    std::vector<float> *weights,
    const RigExecPlaneBounds *bounds)
{
    weights->resize(points.size());
    for (size_t i = 0; i < points.size(); ++i) {
        const GfVec3f local = _ToLocal(worldToLocal, points[i]);
        if (bounds && !RigExecPlaneWithinBounds(local, axis, *bounds)) {
            // Outside the rectangle the field is ZERO, not the ramp's
            // value at that distance: a bounded plane is a patch, and a
            // patch grabs nothing beyond its own edge. Hard by design --
            // see RigExecPlaneWithinBounds for why, and for what to
            // compose with when a soft border is wanted.
            (*weights)[i] = 0.0f;
            continue;
        }
        // Inside, the signed axis distance is exactly what an unbounded
        // plane would measure: the bound clips the field, it does not
        // reshape it, so switching bounds on cannot move an iso-surface.
        const float d = RigExecPlaneDistance(local, axis);
        (*weights)[i] = RigExecEvaluateFalloff(d, params);
    }
}

void
RigExecCurveWeightField(
    const std::vector<GfVec3f> &points,
    const std::vector<GfVec3f> &curvePoints,
    const GfMatrix4d &worldToLocal,
    const RigExecFalloffParams &params,
    std::vector<float> *weights)
{
    weights->resize(points.size());
    if (curvePoints.empty()) {
        // No curve is an empty field, not an identity one: an author who
        // loses the curve target should see the influence vanish rather
        // than silently get full weight everywhere.
        std::fill(weights->begin(), weights->end(), 0.0f);
        return;
    }
    // The curve is transformed once, not per point.
    std::vector<GfVec3f> localCurve(curvePoints.size());
    for (size_t k = 0; k < curvePoints.size(); ++k) {
        localCurve[k] = _ToLocal(worldToLocal, curvePoints[k]);
    }
    for (size_t i = 0; i < points.size(); ++i) {
        const float d = RigExecCurveDistance(
            _ToLocal(worldToLocal, points[i]), localCurve.data(),
            localCurve.size());
        (*weights)[i] = RigExecEvaluateFalloff(d, params);
    }
}

float
RigExecWeightCombineIdentity(RigExecWeightCombine mode)
{
    switch (mode) {
    case RigExecWeightCombine::Multiply:
    case RigExecWeightCombine::Min:
        return 1.0f;
    case RigExecWeightCombine::Add:
    case RigExecWeightCombine::Subtract:
    case RigExecWeightCombine::Max:
    case RigExecWeightCombine::Average:
    case RigExecWeightCombine::Overlay:
        return 0.0f;
    }
    return 0.0f;
}

float
RigExecFoldWeight(RigExecWeightCombine mode, float acc, float value)
{
    switch (mode) {
    case RigExecWeightCombine::Multiply:
        return acc * value;
    case RigExecWeightCombine::Add:
        return acc + value;
    case RigExecWeightCombine::Subtract:
        return acc - value;
    case RigExecWeightCombine::Max:
        return std::max(acc, value);
    case RigExecWeightCombine::Min:
        return std::min(acc, value);
    case RigExecWeightCombine::Average:
        return acc + value;  // divided by the count by the caller
    case RigExecWeightCombine::Overlay:
        return acc < 0.5f ? 2.0f * acc * value
                          : 1.0f - 2.0f * (1.0f - acc) * (1.0f - value);
    }
    return acc;
}

bool
RigExecCombineWeightFields(
    RigExecWeightCombine mode,
    const std::vector<std::vector<float>> &inputs,
    size_t elementCount,
    std::vector<float> *out)
{
    for (const std::vector<float> &field : inputs) {
        if (field.size() != elementCount) {
            out->clear();
            return false;
        }
    }
    if (inputs.empty()) {
        out->assign(elementCount, RigExecWeightCombineIdentity(mode));
        return true;
    }

    // Subtract and Overlay are seeded from the FIRST input rather than
    // from an identity: "a minus the rest" is what an author means by a
    // subtract list, and overlaying onto a zero base would erase
    // everything. Both then consume the authored order, as documented.
    const bool seedFromFirst = mode == RigExecWeightCombine::Subtract ||
                               mode == RigExecWeightCombine::Overlay;
    size_t first = 0;
    if (seedFromFirst) {
        *out = inputs[0];
        first = 1;
    } else {
        out->assign(elementCount, RigExecWeightCombineIdentity(mode));
    }
    for (size_t k = first; k < inputs.size(); ++k) {
        for (size_t i = 0; i < elementCount; ++i) {
            (*out)[i] = RigExecFoldWeight(mode, (*out)[i], inputs[k][i]);
        }
    }
    if (mode == RigExecWeightCombine::Average) {
        const float inv = 1.0f / float(inputs.size());
        for (float &w : *out) {
            w *= inv;
        }
    }
    return true;
}

}  // namespace rigExec
