// Runtime pose decomposition and frame blending; no USD dependencies.

#include "poseInternal.h"
#include <algorithm>
#include <limits>
#include <utility>

namespace rigExec {

using namespace runtimePoseDetail;

namespace {

// Point-frame SRT substrate (pointFrame.cpp, matrix3d.cpp): the SVD
// decomposition Blend, Twist and the FBX constraints solve through.

// GfMatrix3d::Orthonormalize (matrix3d.cpp): orthogonalize and
// normalize the row vectors. The frame path never reads the result.
void
_RrMat3Orthonormalize(RrMat3d *m)
{
    RrVec3d r0((*m)[0][0], (*m)[0][1], (*m)[0][2]);
    RrVec3d r1((*m)[1][0], (*m)[1][1], (*m)[1][2]);
    RrVec3d r2((*m)[2][0], (*m)[2][1], (*m)[2][2]);
    RrOrthogonalizeBasis(&r0, &r1, &r2, true);
    (*m)[0][0] = r0[0];
    (*m)[0][1] = r0[1];
    (*m)[0][2] = r0[2];
    (*m)[1][0] = r1[0];
    (*m)[1][1] = r1[1];
    (*m)[1][2] = r1[2];
    (*m)[2][0] = r2[0];
    (*m)[2][1] = r2[1];
    (*m)[2][2] = r2[2];
}

// GfMatrix3d::_SetRotateFromQuat (matrix3d.cpp), verbatim.
void
_RrMat3SetRotate(RrMat3d *m, const RrQuatd &rot)
{
    const double r = rot.GetReal();
    const RrVec3d i = rot.GetImaginary();
    (*m)[0][0] = 1.0 - 2.0 * (i[1] * i[1] + i[2] * i[2]);
    (*m)[0][1] = 2.0 * (i[0] * i[1] + i[2] * r);
    (*m)[0][2] = 2.0 * (i[2] * i[0] - i[1] * r);

    (*m)[1][0] = 2.0 * (i[0] * i[1] - i[2] * r);
    (*m)[1][1] = 1.0 - 2.0 * (i[2] * i[2] + i[0] * i[0]);
    (*m)[1][2] = 2.0 * (i[1] * i[2] + i[0] * r);

    (*m)[2][0] = 2.0 * (i[2] * i[0] + i[1] * r);
    (*m)[2][1] = 2.0 * (i[1] * i[2] - i[0] * r);
    (*m)[2][2] = 1.0 - 2.0 * (i[1] * i[1] + i[0] * i[0]);
}

// GfMatrix3d::ExtractRotationQuaternion (matrix3d.cpp, including the
// int-typed 4 in the else arm), followed by the GfRotation round trip
// ExtractRotation().GetQuat() performs.
RrQuatd
_RrMat3ExtractRotationQuat(const RrMat3d &m)
{
    int i;
    if (m[0][0] > m[1][1]) {
        i = (m[0][0] > m[2][2] ? 0 : 2);
    } else {
        i = (m[1][1] > m[2][2] ? 1 : 2);
    }

    RrVec3d im(0.0, 0.0, 0.0);
    double r = 0.0;

    if (m[0][0] + m[1][1] + m[2][2] > m[i][i]) {
        r = 0.5 * std::sqrt(m[0][0] + m[1][1] + m[2][2] + 1);
        im[0] = (m[1][2] - m[2][1]) / (4.0 * r);
        im[1] = (m[2][0] - m[0][2]) / (4.0 * r);
        im[2] = (m[0][1] - m[1][0]) / (4.0 * r);
    } else {
        const int j = (i + 1) % 3;
        const int k = (i + 2) % 3;
        const double q =
            0.5 * std::sqrt(m[i][i] - m[j][j] - m[k][k] + 1);

        im[i] = q;
        im[j] = (m[i][j] + m[j][i]) / (4 * q);
        im[k] = (m[k][i] + m[i][k]) / (4 * q);
        r = (m[j][k] - m[k][j]) / (4 * q);
    }

    r = RrClamp(r, -1.0, 1.0);
    RrRotation rotation;
    rotation.SetQuat(RrQuatd(r, im));
    return rotation.GetQuat();
}

// Jacobi eigen decomposition of a symmetric 3x3 matrix (pointFrame.cpp):
// eigenvalues descending, eigenvectors as the columns of V.
void
_RrSymmetricEigen3(const double m[3][3], double eval[3],
                   double evec[3][3])
{
    double a[3][3];
    double v[3][3] = {{1, 0, 0}, {0, 1, 0}, {0, 0, 1}};
    for (int i = 0; i < 3; ++i) {
        for (int j = 0; j < 3; ++j) {
            a[i][j] = m[i][j];
        }
    }

    for (int sweep = 0; sweep < 64; ++sweep) {
        double off = std::abs(a[0][1]) + std::abs(a[0][2]) +
                     std::abs(a[1][2]);
        if (off < 1e-15) {
            break;
        }
        for (int p = 0; p < 2; ++p) {
            for (int q = p + 1; q < 3; ++q) {
                if (std::abs(a[p][q]) < 1e-18) {
                    continue;
                }
                const double theta = (a[q][q] - a[p][p]) / (2 * a[p][q]);
                const double t = (theta >= 0 ? 1.0 : -1.0) /
                    (std::abs(theta) + std::sqrt(theta * theta + 1));
                const double cth = 1.0 / std::sqrt(t * t + 1);
                const double sth = t * cth;
                for (int k = 0; k < 3; ++k) {
                    const double akp = a[k][p], akq = a[k][q];
                    a[k][p] = cth * akp - sth * akq;
                    a[k][q] = sth * akp + cth * akq;
                }
                for (int k = 0; k < 3; ++k) {
                    const double apk = a[p][k], aqk = a[q][k];
                    a[p][k] = cth * apk - sth * aqk;
                    a[q][k] = sth * apk + cth * aqk;
                }
                for (int k = 0; k < 3; ++k) {
                    const double vkp = v[k][p], vkq = v[k][q];
                    v[k][p] = cth * vkp - sth * vkq;
                    v[k][q] = sth * vkp + cth * vkq;
                }
            }
        }
    }

    int order[3] = {0, 1, 2};
    double d[3] = {a[0][0], a[1][1], a[2][2]};
    std::sort(order, order + 3,
              [&](int x, int y) { return d[x] > d[y]; });
    for (int i = 0; i < 3; ++i) {
        eval[i] = d[order[i]];
        for (int k = 0; k < 3; ++k) {
            evec[k][i] = v[k][order[i]];
        }
        int maxK = 0;
        for (int k = 1; k < 3; ++k) {
            if (std::abs(evec[k][i]) > std::abs(evec[maxK][i])) {
                maxK = k;
            }
        }
        if (evec[maxK][i] < 0) {
            for (int k = 0; k < 3; ++k) {
                evec[k][i] = -evec[k][i];
            }
        }
    }

    const double tol =
        1e-8 * std::max({std::abs(eval[0]), std::abs(eval[2]), 1.0});
    const bool eq01 = std::abs(eval[0] - eval[1]) <= tol;
    const bool eq12 = std::abs(eval[1] - eval[2]) <= tol;
    if (eq01 && eq12) {
        for (int i = 0; i < 3; ++i) {
            for (int k = 0; k < 3; ++k) {
                evec[k][i] = (k == i) ? 1.0 : 0.0;
            }
        }
    } else if (eq01 || eq12) {
        const int a = eq01 ? 0 : 1;
        const int other = eq01 ? 2 : 0;
        const double n[3] = {evec[0][other], evec[1][other],
                             evec[2][other]};
        int bestAxis = 0;
        double bestLen = -1.0;
        double b1[3] = {0, 0, 0};
        for (int axis = 0; axis < 3; ++axis) {
            double p[3];
            for (int k = 0; k < 3; ++k) {
                p[k] = ((k == axis) ? 1.0 : 0.0) - n[k] * n[axis];
            }
            const double len =
                std::sqrt(p[0] * p[0] + p[1] * p[1] + p[2] * p[2]);
            if (len > bestLen) {
                bestLen = len;
                bestAxis = axis;
                for (int k = 0; k < 3; ++k) {
                    b1[k] = p[k] / len;
                }
            }
        }
        (void)bestAxis;
        double b2[3] = {
            n[1] * b1[2] - n[2] * b1[1],
            n[2] * b1[0] - n[0] * b1[2],
            n[0] * b1[1] - n[1] * b1[0]};
        int maxK = 0;
        for (int k = 1; k < 3; ++k) {
            if (std::abs(b2[k]) > std::abs(b2[maxK])) {
                maxK = k;
            }
        }
        if (b2[maxK] < 0) {
            for (int k = 0; k < 3; ++k) {
                b2[k] = -b2[k];
            }
        }
        for (int k = 0; k < 3; ++k) {
            evec[k][a] = b1[k];
            evec[k][a + 1] = b2[k];
        }
    }
}

} // namespace

namespace runtimePoseDetail {

RrPointFrame _RrBlendFrames(
    const RrPointFrame &a, const RrPointFrame &b,
    const std::array<RrVec3d, 4> &restPoints, double weight,
    bool logScale, const std::array<RrVec3d, 4> *outRestPoints);

// RigExecPointsToParams: the affine map rest->pose decomposed into SRT
// via SVD. False for a singular reference or posed frame.
bool
_RrPointsToParams(const std::array<RrVec3d, 4> &restPoints,
                  const std::array<RrVec3d, 4> &posePoints,
                  int reflectionAxis, _RrTransformParams *params)
{
    RrMat4d m = _RrIdentity();
    if (!RrPointsToMatrix(restPoints, posePoints, &m)) {
        return false;
    }

    double L[3][3];
    for (int i = 0; i < 3; ++i) {
        for (int j = 0; j < 3; ++j) {
            L[i][j] = m[j][i];
        }
    }

    double ltl[3][3];
    for (int i = 0; i < 3; ++i) {
        for (int j = 0; j < 3; ++j) {
            double sum = 0;
            for (int k = 0; k < 3; ++k) {
                sum += L[k][i] * L[k][j];
            }
            ltl[i][j] = sum;
        }
    }
    double s2[3], V[3][3];
    _RrSymmetricEigen3(ltl, s2, V);

    double sigma[3];
    for (int i = 0; i < 3; ++i) {
        sigma[i] = std::sqrt(std::max(s2[i], 0.0));
    }
    const double detL =
        L[0][0] * (L[1][1] * L[2][2] - L[1][2] * L[2][1]) -
        L[0][1] * (L[1][0] * L[2][2] - L[1][2] * L[2][0]) +
        L[0][2] * (L[1][0] * L[2][1] - L[1][1] * L[2][0]);
    if (sigma[2] < 1e-14 * std::max(1.0, sigma[0])) {
        return false;
    }

    double U[3][3];
    for (int i = 0; i < 3; ++i) {
        for (int j = 0; j < 3; ++j) {
            double sum = 0;
            for (int k = 0; k < 3; ++k) {
                sum += L[i][k] * V[k][j];
            }
            U[i][j] = sum / sigma[j];
        }
    }

    const double delta = (detL >= 0) ? 1.0 : -1.0;

    int k = 2;
    if (delta < 0) {
        const int axis = reflectionAxis;
        double best = -1.0;
        for (int i = 0; i < 3; ++i) {
            const double align = std::abs(V[axis][i]);
            if (align > best) {
                best = align;
                k = i;
            }
        }
    }

    double R[3][3];
    for (int i = 0; i < 3; ++i) {
        for (int j = 0; j < 3; ++j) {
            double sum = 0;
            for (int kk = 0; kk < 3; ++kk) {
                const double dkk = (kk == k) ? delta : 1.0;
                sum += U[i][kk] * dkk * V[j][kk];
            }
            R[i][j] = sum;
        }
    }

    double H[3][3];
    for (int i = 0; i < 3; ++i) {
        for (int j = 0; j < 3; ++j) {
            double sum = 0;
            for (int kk = 0; kk < 3; ++kk) {
                sum += R[kk][i] * L[kk][j];
            }
            H[i][j] = sum;
        }
    }

    RrMat3d rowR(
        R[0][0], R[1][0], R[2][0],
        R[0][1], R[1][1], R[2][1],
        R[0][2], R[1][2], R[2][2]);
    _RrMat3Orthonormalize(&rowR);
    const RrQuatd q = _RrMat3ExtractRotationQuat(rowR);

    params->translation = RrVec3d(m[3][0], m[3][1], m[3][2]);
    params->rotation = q.GetNormalized();
    params->scale = RrVec3d(H[0][0], H[1][1], H[2][2]);
    params->shear = RrVec3d(
        H[0][1] / H[0][0],
        H[0][2] / H[0][0],
        H[1][2] / H[1][1]);
    params->reflectionAxis = reflectionAxis;
    return true;
}

// RigExecParamsToMatrix: L = R(q) H(s,h), t.
RrMat4d
_RrParamsToMatrix(const _RrTransformParams &params)
{
    RrMat3d rowR;
    _RrMat3SetRotate(&rowR, params.rotation);
    double R[3][3];
    for (int i = 0; i < 3; ++i) {
        for (int j = 0; j < 3; ++j) {
            R[i][j] = rowR[j][i];
        }
    }

    const RrVec3d &sc = params.scale;
    const RrVec3d &sh = params.shear;
    double H[3][3] = {
        {sc[0], sh[0] * sc[0], sh[1] * sc[0]},
        {sh[0] * sc[0], sc[1], sh[2] * sc[1]},
        {sh[1] * sc[0], sh[2] * sc[1], sc[2]},
    };

    double L[3][3];
    for (int i = 0; i < 3; ++i) {
        for (int j = 0; j < 3; ++j) {
            double sum = 0;
            for (int k = 0; k < 3; ++k) {
                sum += R[i][k] * H[k][j];
            }
            L[i][j] = sum;
        }
    }

    return RrMat4d(
        L[0][0], L[1][0], L[2][0], 0.0,
        L[0][1], L[1][1], L[2][1], 0.0,
        L[0][2], L[1][2], L[2][2], 0.0,
        params.translation[0], params.translation[1],
        params.translation[2], 1.0);
}

// RigExecBlendFrames (solvers.cpp).
RrPointFrame
_RrBlendFrames(const RrPointFrame &a, const RrPointFrame &b,
               const std::array<RrVec3d, 4> &restPoints,
               double weight, bool logScale,
               const std::array<RrVec3d, 4> *outRestPoints)
{
    const double w = std::min(std::max(weight, 0.0), 1.0);
    if (!outRestPoints) {
        if (w <= 0.0) {
            return a;
        }
        if (w >= 1.0) {
            return b;
        }
    } else if (w <= 0.0 || w >= 1.0) {
        const RrPointFrame &pick = (w <= 0.0) ? a : b;
        RrMat4d end = _RrIdentity();
        if (!RrPointsToMatrix(restPoints, pick.points, &end)) {
            RrPointFrame held = pick;
            held.flags |= RrPointFrameDegenerate;
            return held;
        }
        return RrMatrixToPoints(*outRestPoints, end);
    }

    _RrTransformParams pa, pb;
    if (!_RrPointsToParams(restPoints, a.points, 2, &pa) ||
        !_RrPointsToParams(restPoints, b.points, 2, &pb)) {
        RrPointFrame held = (w < 0.5) ? a : b;
        held.flags |= RrPointFrameDegenerate;
        return held;
    }

    _RrTransformParams pr;
    pr.translation = pa.translation * (1 - w) + pb.translation * w;

    RrQuatd qa = pa.rotation, qb = pb.rotation;
    if (RrDot(qa.GetImaginary(), qb.GetImaginary()) +
            qa.GetReal() * qb.GetReal() <
        0) {
        qb = -qb;
    }
    pr.rotation = RrSlerp(w, qa, qb).GetNormalized();

    for (int i = 0; i < 3; ++i) {
        if (logScale && pa.scale[i] > 0 && pb.scale[i] > 0) {
            pr.scale[i] = std::exp(
                std::log(pa.scale[i]) * (1 - w) +
                std::log(pb.scale[i]) * w);
        } else {
            pr.scale[i] = pa.scale[i] * (1 - w) + pb.scale[i] * w;
        }
        pr.shear[i] = pa.shear[i] * (1 - w) + pb.shear[i] * w;
    }

    const RrMat4d m = _RrParamsToMatrix(pr);
    return RrMatrixToPoints(
        outRestPoints ? *outRestPoints : restPoints, m);
}

// Swing/twist decomposition of q about unit axis (solvers.cpp).
void
_RrSwingTwist(const RrQuatd &q, const RrVec3d &axis,
              RrQuatd *swing, RrQuatd *twist)
{
    const RrVec3d im = q.GetImaginary();
    const double proj = RrDot(im, axis);
    RrQuatd t(q.GetReal(), axis * proj);
    const double len = std::sqrt(
        t.GetReal() * t.GetReal() +
        RrDot(t.GetImaginary(), t.GetImaginary()));
    if (len < 1e-15) {
        *twist = RrQuatd::GetIdentity();
    } else {
        *twist = t * (1.0 / len);
    }
    *swing = q * twist->GetInverse();
}

} // namespace runtimePoseDetail

} // namespace rigExec
