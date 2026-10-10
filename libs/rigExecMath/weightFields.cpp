// RigExec volumetric weight-field kernels implementation.
#include "weightFields.h"
#include "spatialAccel.h"

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
    return RigExecSampleFalloffLut(curve.data(), curve.size(), r);
}

float
RigExecSampleFalloffLut(const float *curve, size_t count, float r)
{
    if (count < 2) {
        return r;  // empty or degenerate table is the identity
    }
    const float x = _Clamp01(r) * float(count - 1);
    // floor, not truncation: x is already non-negative here, but keeping
    // the two spellings distinct is what stops a later signed input from
    // silently indexing backwards.
    const size_t i = std::min(count - 2,
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
    const float w = (p.curveData ? RigExecSampleFalloffLut(p.curveData, p.curveCount, 1.0f - u)
                                    : RigExecSampleFalloffLut(p.curve, 1.0f - u));
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
RigExecSphereWeightField(const std::vector<GfVec3f> &points,
    const GfMatrix4d &matrix, const RigExecFalloffParams &params,
    std::vector<float> *weights, const GfVec3f &positive, const GfVec3f &negative)
{
    RigExecSphereWeightField({points.data(),points.size()},matrix,params,weights,positive,negative);
}
void
RigExecPlaneWeightField(const std::vector<GfVec3f> &points,
    const GfMatrix4d &matrix, int axis, const RigExecFalloffParams &params,
    std::vector<float> *weights, const RigExecPlaneBounds *bounds)
{
    RigExecPlaneWeightField({points.data(),points.size()},matrix,axis,params,weights,bounds);
}
void
RigExecCurveWeightField(const std::vector<GfVec3f> &points,
    const std::vector<GfVec3f> &curve, const GfMatrix4d &matrix,
    const RigExecFalloffParams &params, std::vector<float> *weights)
{
    std::vector<GfVec3f> localCurve;
    RigExecCurveWeightField({points.data(),points.size()},{curve.data(),curve.size()},
        matrix,params,weights,&localCurve);
}

void
RigExecSphereWeightField(
    RigExecWeightPointView points,
    const GfMatrix4d &worldToLocal,
    const RigExecFalloffParams &params,
    std::vector<float> *weights,
    const GfVec3f &positiveScales, const GfVec3f &negativeScales)
{
    weights->resize(points.count);
    for (size_t i = 0; i < points.count; ++i) {
        GfVec3f local = _ToLocal(worldToLocal, points.data[i]);
        for (int axis = 0; axis < 3; ++axis) {
            local[axis] /= local[axis] < 0.0f
                ? negativeScales[axis] : positiveScales[axis];
        }
        const float d = RigExecSphereDistance(local);
        (*weights)[i] = RigExecEvaluateFalloff(d, params);
    }
}

void
RigExecPlaneWeightField(
    RigExecWeightPointView points,
    const GfMatrix4d &worldToLocal,
    int axis,
    const RigExecFalloffParams &params,
    std::vector<float> *weights,
    const RigExecPlaneBounds *bounds)
{
    weights->resize(points.count);
    for (size_t i = 0; i < points.count; ++i) {
        const GfVec3f local = _ToLocal(worldToLocal, points.data[i]);
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
    RigExecWeightPointView points,
    RigExecWeightPointView curvePoints,
    const GfMatrix4d &worldToLocal,
    const RigExecFalloffParams &params,
    std::vector<float> *weights, std::vector<GfVec3f> *curveScratch)
{
    weights->resize(points.count);
    if (curvePoints.count == 0) {
        // No curve is an empty field, not an identity one: an author who
        // loses the curve target should see the influence vanish rather
        // than silently get full weight everywhere.
        std::fill(weights->begin(), weights->end(), 0.0f);
        return;
    }
    // The curve is transformed once, not per point.
    auto &localCurve = *curveScratch;
    localCurve.resize(curvePoints.count);
    for (size_t k = 0; k < curvePoints.count; ++k) {
        localCurve[k] = _ToLocal(worldToLocal, curvePoints.data[k]);
    }
    if (localCurve.size() - 1 >= kRigExecBvhMinSegments &&
        points.count >= kRigExecBvhMinQueries) {
        RigExecSegmentBvh<GfVec3f> bvh;
        if (bvh.Build(localCurve.data(), localCurve.size())) {
            for (size_t i = 0; i < points.count; ++i) {
                const float d = bvh.QueryNearest(
                    _ToLocal(worldToLocal, points.data[i]), localCurve.data(),
                    RigExecSegmentDistance);
                (*weights)[i] = RigExecEvaluateFalloff(d, params);
            }
            return;
        }
        // Build failed (non-finite curve points): fall through to the
        // verbatim loop below.
    }
    for (size_t i = 0; i < points.count; ++i) {
        const float d = RigExecCurveDistance(
            _ToLocal(worldToLocal, points.data[i]), localCurve.data(),
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
    // Single definition: the streaming core owns the seeding, order, and
    // average rules, so this entry point cannot drift from it. The
    // inputs are already dense, so the core folds straight from them
    // with no per-input copy.
    return RigExecCombineWeightFieldsStreamed(
        mode, inputs.size(), elementCount,
        [&](size_t k, const float **data, size_t *size) {
            *data = inputs[k].data();
            *size = inputs[k].size();
            return true;
        },
        out);
}

}  // namespace rigExec
