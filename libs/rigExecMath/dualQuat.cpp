//
// RigExec dual-quaternion skinning primitives.
//
#include "dualQuat.h"

#include "pointFrame.h"

#include <algorithm>
#include <array>
#include <cmath>

namespace rigExec {
namespace {

bool
_IsFinite(const GfQuatd &q)
{
    const GfVec3d &v = q.GetImaginary();
    return std::isfinite(q.GetReal()) && std::isfinite(v[0]) &&
           std::isfinite(v[1]) && std::isfinite(v[2]);
}

// Row-vector convention: rows of the 3x3 part are the images of the basis
// vectors. Orthonormal with positive determinant means a proper rotation.
bool
_IsProperRotation(const GfMatrix4d &m, double tolerance)
{
    const GfVec3d r0 = m.GetRow3(0);
    const GfVec3d r1 = m.GetRow3(1);
    const GfVec3d r2 = m.GetRow3(2);
    const double gram[6] = {
        GfDot(r0, r0) - 1.0, GfDot(r1, r1) - 1.0, GfDot(r2, r2) - 1.0,
        GfDot(r0, r1), GfDot(r0, r2), GfDot(r1, r2),
    };
    for (const double g : gram) {
        if (!(std::abs(g) <= tolerance)) {
            return false;
        }
    }
    return GfDot(GfCross(r0, r1), r2) > 0.0;
}

// Accumulates sum(w_i * dq_i) with shortest-arc sign correction against the
// first non-zero-weight input, then normalises once. Shared by both public
// blend overloads through the Get(i) / Weight(i) accessors.
template <class GetDq, class GetWeight>
bool
_Blend(
    size_t count, GetDq getDq, GetWeight getWeight, RigExecDualQuat *result)
{
    *result = RigExecDualQuat();

    GfQuatd realSum(0.0);
    GfQuatd dualSum(0.0);
    GfQuatd reference(0.0);
    bool haveReference = false;
    double absWeightSum = 0.0;

    for (size_t i = 0; i < count; ++i) {
        const double w = getWeight(i);
        if (!std::isfinite(w)) {
            return false;
        }
        if (w == 0.0) {
            continue;
        }
        const RigExecDualQuat *dq = getDq(i);
        if (!dq) {
            return false;
        }
        double sign = 1.0;
        if (!haveReference) {
            reference = dq->real;
            haveReference = true;
        } else if (GfDot(dq->real, reference) < 0.0) {
            sign = -1.0;
        }
        const double sw = sign * w;
        realSum += dq->real * sw;
        dualSum += dq->dual * sw;
        absWeightSum += std::abs(w);
    }

    if (!haveReference) {
        return false;
    }
    result->real = realSum;
    result->dual = dualSum;
    const double norm = realSum.GetLength();
    if (!(norm > RigExecDualQuatEpsilon * std::max(1.0, absWeightSum))) {
        *result = RigExecDualQuat();
        return false;
    }
    return RigExecDualQuatNormalize(result);
}

}  // namespace

RigExecDualQuat
RigExecDualQuatFromRotationTranslation(
    const GfQuatd &rotation, const GfVec3d &translation)
{
    RigExecDualQuat dq;
    dq.real = rotation.GetNormalized();
    // qd = 1/2 * t * qr, with t a pure quaternion.
    dq.dual = (GfQuatd(0.0, translation) * dq.real) * 0.5;
    return dq;
}

namespace {

// Shared by the rigid-only and scale-aware conversions so that a rigid
// input yields the same rotation quaternion, bit for bit, on both paths.
// Returns the rigid flag; for non-rigid input, when \p stretch is
// supplied, it receives S = L * R^T (row-vector, symmetrised), or the raw
// linear part when the input is singular (rotation identity).
bool
_DecomposeMatrix(
    const GfMatrix4d &matrix, GfQuatd *rotation, GfMatrix3d *stretch)
{
    if (_IsProperRotation(matrix, RigExecDualQuatEpsilon)) {
        *rotation = matrix.ExtractRotationQuat();
        if (stretch) {
            stretch->SetIdentity();
        }
        return true;
    }

    // Scale / shear / reflection: the proper polar rotation from the
    // library's standard decomposition. A singular linear part has no
    // rotation to keep; identity it is.
    static const std::array<GfVec3d, 4> unitRest = {
        GfVec3d(0, 0, 0), GfVec3d(1, 0, 0),
        GfVec3d(0, 1, 0), GfVec3d(0, 0, 1)};
    RigExecTransformParams params;
    const bool nonSingular = RigExecPointsToParams(
        unitRest, RigExecMatrixToPoints(unitRest, matrix).points,
        RigExecAxis::Z, &params);
    *rotation = nonSingular ? params.rotation : GfQuatd(1.0);

    if (stretch) {
        const GfMatrix3d linear = matrix.ExtractRotationMatrix();
        if (!nonSingular) {
            *stretch = linear;
        } else {
            // L = S * R  =>  S = L * R^T; symmetrise away rounding.
            GfMatrix3d rowR;
            rowR.SetRotate(*rotation);
            const GfMatrix3d s = linear * rowR.GetTranspose();
            for (int i = 0; i < 3; ++i) {
                for (int j = 0; j < 3; ++j) {
                    (*stretch)[i][j] = 0.5 * (s[i][j] + s[j][i]);
                }
            }
        }
    }
    return false;
}

// Row-vector product p * S for the 3x3 stretch.
GfVec3d
_MulRow(const GfVec3d &p, const GfMatrix3d &s)
{
    return GfVec3d(
        p[0] * s[0][0] + p[1] * s[1][0] + p[2] * s[2][0],
        p[0] * s[0][1] + p[1] * s[1][1] + p[2] * s[2][1],
        p[0] * s[0][2] + p[1] * s[1][2] + p[2] * s[2][2]);
}

}  // namespace

RigExecDualQuat
RigExecDualQuatFromMatrix(const GfMatrix4d &matrix, bool *isRigid)
{
    GfQuatd rotation(1.0);
    const bool rigid = _DecomposeMatrix(matrix, &rotation, nullptr);
    if (isRigid) {
        *isRigid = rigid;
    }
    return RigExecDualQuatFromRotationTranslation(
        rotation, matrix.ExtractTranslation());
}

RigExecScaledDualQuat
RigExecScaledDualQuatFromMatrix(const GfMatrix4d &matrix)
{
    RigExecScaledDualQuat sdq;
    GfQuatd rotation(1.0);
    sdq.isRigid = _DecomposeMatrix(matrix, &rotation, &sdq.stretch);
    sdq.rigid = RigExecDualQuatFromRotationTranslation(
        rotation, matrix.ExtractTranslation());
    return sdq;
}

GfMatrix4d
RigExecScaledDualQuatToMatrix(const RigExecScaledDualQuat &sdq)
{
    const GfMatrix4d rigid = RigExecDualQuatToMatrix(sdq.rigid);
    if (sdq.isRigid) {
        return rigid;
    }
    // [S | 0] * [R | t] = [S * R | t] in row-vector form.
    return GfMatrix4d(sdq.stretch, GfVec3d(0)) * rigid;
}

namespace {

// The stretch half of the scale-aware blend, run after the rigid blend has
// validated every weight (finite) and every index, and found at least one
// non-zero weight. Shared by both public overloads through the same
// Get(i) / Weight(i) accessor pattern as _Blend.
template <class GetSdq, class GetWeight>
bool
_BlendStretch(
    size_t count, GetSdq getSdq, GetWeight getWeight,
    RigExecScaledDualQuat *result)
{
    // Two passes: the first is scalar work only, so a blend of rigid
    // inputs (the common skinning case) never touches the 3x3 stretches.
    bool allRigid = true;
    double weightSum = 0.0;
    double absWeightSum = 0.0;
    for (size_t i = 0; i < count; ++i) {
        const double w = getWeight(i);
        if (w == 0.0) {
            continue;
        }
        weightSum += w;
        absWeightSum += std::abs(w);
        if (!getSdq(i)->isRigid) {
            allRigid = false;
        }
    }
    if (allRigid) {
        // Exact identity stretch: nothing to normalise, nothing dropped.
        return true;
    }
    if (!(std::abs(weightSum) >
          RigExecDualQuatEpsilon * std::max(1.0, absWeightSum))) {
        *result = RigExecScaledDualQuat();
        return false;
    }
    GfMatrix3d stretchSum(0.0);
    for (size_t i = 0; i < count; ++i) {
        const double w = getWeight(i);
        if (w == 0.0) {
            continue;
        }
        stretchSum += getSdq(i)->stretch * w;
    }
    result->stretch = stretchSum * (1.0 / weightSum);
    result->isRigid = false;
    return true;
}

}  // namespace

bool
RigExecBlendScaledDualQuats(
    const RigExecScaledDualQuat *sdqs, const double *weights, size_t count,
    RigExecScaledDualQuat *result)
{
    *result = RigExecScaledDualQuat();
    if (count > 0 && (!sdqs || !weights)) {
        return false;
    }
    auto getSdq = [sdqs](size_t i) { return &sdqs[i]; };
    auto getWeight = [weights](size_t i) { return weights[i]; };
    if (!_Blend(
            count,
            [&getSdq](size_t i) { return &getSdq(i)->rigid; },
            getWeight, &result->rigid)) {
        return false;
    }
    return _BlendStretch(count, getSdq, getWeight, result);
}

bool
RigExecBlendScaledDualQuats(
    const RigExecScaledDualQuat *palette, size_t paletteSize,
    const int *indices, const double *weights, size_t count,
    RigExecScaledDualQuat *result)
{
    *result = RigExecScaledDualQuat();
    if (count > 0 && (!palette || !indices || !weights)) {
        return false;
    }
    auto getSdq = [palette, paletteSize, indices](size_t i)
        -> const RigExecScaledDualQuat * {
        const int index = indices[i];
        if (index < 0 || static_cast<size_t>(index) >= paletteSize) {
            return nullptr;
        }
        return &palette[index];
    };
    auto getWeight = [weights](size_t i) { return weights[i]; };
    if (!_Blend(
            count,
            [&getSdq](size_t i) -> const RigExecDualQuat * {
                const RigExecScaledDualQuat *sdq = getSdq(i);
                return sdq ? &sdq->rigid : nullptr;
            },
            getWeight, &result->rigid)) {
        return false;
    }
    // Every non-zero-weight index was validated by the rigid blend.
    return _BlendStretch(count, getSdq, getWeight, result);
}

GfVec3d
RigExecScaledDualQuatTransformPoint(
    const RigExecScaledDualQuat &sdq, const GfVec3d &point)
{
    if (sdq.isRigid) {
        return RigExecDualQuatTransformPoint(sdq.rigid, point);
    }
    return RigExecDualQuatTransformPoint(sdq.rigid, _MulRow(point, sdq.stretch));
}

GfMatrix4d
RigExecDualQuatToMatrix(const RigExecDualQuat &dq)
{
    GfMatrix4d m;
    m.SetRotate(dq.real);
    m.SetTranslateOnly(RigExecDualQuatTranslation(dq));
    return m;
}

GfVec3d
RigExecDualQuatTranslation(const RigExecDualQuat &dq)
{
    // t = 2 * qd * conj(qr). With qr = (w, v) and qd = (dw, dv) the vector
    // part of qd * conj(qr) is w*dv - dw*v + v x dv; the scalar part is
    // dropped, which is also what makes a dual part with a stray component
    // along the real part harmless here.
    const double w = dq.real.GetReal();
    const GfVec3d &v = dq.real.GetImaginary();
    const double dw = dq.dual.GetReal();
    const GfVec3d &dv = dq.dual.GetImaginary();
    return (dv * w - v * dw + GfCross(v, dv)) * 2.0;
}

bool
RigExecDualQuatIsUnit(const RigExecDualQuat &dq, double tolerance)
{
    if (!_IsFinite(dq.real) || !_IsFinite(dq.dual)) {
        return false;
    }
    return std::abs(dq.real.GetLength() - 1.0) <= tolerance &&
           std::abs(GfDot(dq.real, dq.dual)) <= tolerance;
}

bool
RigExecDualQuatNormalize(RigExecDualQuat *dq)
{
    const double norm = dq->real.GetLength();
    if (!(norm > RigExecDualQuatEpsilon) || !std::isfinite(norm) ||
        !_IsFinite(dq->dual)) {
        *dq = RigExecDualQuat();
        return false;
    }
    const double inv = 1.0 / norm;
    dq->real *= inv;
    dq->dual *= inv;
    // Project out the component of the dual part along the (now unit) real
    // part so that dot(real, dual) == 0. This changes only the scalar part
    // of 2 * dual * conj(real), i.e. it does not move the translation.
    dq->dual -= dq->real * GfDot(dq->real, dq->dual);
    return true;
}

bool
RigExecBlendDualQuats(
    const RigExecDualQuat *dqs, const double *weights, size_t count,
    RigExecDualQuat *result)
{
    if (count > 0 && (!dqs || !weights)) {
        *result = RigExecDualQuat();
        return false;
    }
    return _Blend(
        count,
        [dqs](size_t i) { return &dqs[i]; },
        [weights](size_t i) { return weights[i]; },
        result);
}

bool
RigExecBlendDualQuats(
    const RigExecDualQuat *palette, size_t paletteSize,
    const int *jointIndices, const float *jointWeights, size_t count,
    RigExecDualQuat *result)
{
    if (count > 0 && (!palette || !jointIndices || !jointWeights)) {
        *result = RigExecDualQuat();
        return false;
    }
    return _Blend(
        count,
        [palette, paletteSize, jointIndices](size_t i)
            -> const RigExecDualQuat * {
            const int index = jointIndices[i];
            if (index < 0 || static_cast<size_t>(index) >= paletteSize) {
                return nullptr;
            }
            return &palette[index];
        },
        [jointWeights](size_t i) {
            return static_cast<double>(jointWeights[i]);
        },
        result);
}

GfVec3d
RigExecDualQuatRotateVector(
    const RigExecDualQuat &dq, const GfVec3d &vector)
{
    // Rodrigues form of q v q*: v + 2 (w (u x v) + u x (u x v)).
    const double w = dq.real.GetReal();
    const GfVec3d &u = dq.real.GetImaginary();
    const GfVec3d c = GfCross(u, vector);
    return vector + (c * w + GfCross(u, c)) * 2.0;
}

GfVec3d
RigExecDualQuatTransformPoint(
    const RigExecDualQuat &dq, const GfVec3d &point)
{
    return RigExecDualQuatRotateVector(dq, point) +
           RigExecDualQuatTranslation(dq);
}

}  // namespace rigExec
