//
// RigExec point-frame math (spec §5).
//
#include "pointFrame.h"

#include "pxr/base/gf/matrix3d.h"
#include "pxr/base/gf/rotation.h"

#include <algorithm>
#include <cmath>

namespace rigExec {

RigExecFramePolicy
RigExecParseFramePolicy(const TfToken &token)
{
    if (token == "affine") return RigExecFramePolicy::Affine;
    if (token == "axial") return RigExecFramePolicy::Axial;
    if (token == "rigid") return RigExecFramePolicy::Rigid;
    return RigExecFramePolicy::Orthogonal;
}

RigExecAxis
RigExecParseAxis(const TfToken &token, RigExecAxis fallback)
{
    if (token == "x") return RigExecAxis::X;
    if (token == "y") return RigExecAxis::Y;
    if (token == "z") return RigExecAxis::Z;
    return fallback;
}

double
RigExecFrameEpsilon(const std::array<GfVec3d, 4> &restPoints)
{
    const double lx = (restPoints[1] - restPoints[0]).GetLength();
    const double ly = (restPoints[2] - restPoints[0]).GetLength();
    const double lz = (restPoints[3] - restPoints[0]).GetLength();
    return 1e-10 * std::max({1.0, lx, ly, lz});
}

namespace {

// Row-basis matrix [q1-q0; q2-q0; q3-q0]. With Gf's row-vector convention
// this is B^T of the spec's column-basis matrix B.
GfMatrix3d
_RowBasis(const std::array<GfVec3d, 4> &pts)
{
    return GfMatrix3d(
        pts[1][0] - pts[0][0], pts[1][1] - pts[0][1], pts[1][2] - pts[0][2],
        pts[2][0] - pts[0][0], pts[2][1] - pts[0][1], pts[2][2] - pts[0][2],
        pts[3][0] - pts[0][0], pts[3][1] - pts[0][1], pts[3][2] - pts[0][2]);
}

GfVec3d
_TransformDir(const GfVec3d &v, const GfMatrix3d &m)
{
    // Row-vector convention: v * m.
    return v * m;
}

}  // namespace

bool
RigExecPointsToMatrix(
    const std::array<GfVec3d, 4> &restPoints,
    const std::array<GfVec3d, 4> &posePoints,
    GfMatrix4d *matrix)
{
    // Spec §5.1 (column convention): L = B_P B_Q^-1, t = po - L qo.
    // Row-vector storage: M3 = L^T = (B_Q^T)^-1 B_P^T = MQ^-1 * MP,
    // where MQ/MP are the row-basis matrices.
    const GfMatrix3d mq = _RowBasis(restPoints);
    const GfMatrix3d mp = _RowBasis(posePoints);

    const double det = mq.GetDeterminant();
    const double eps = RigExecFrameEpsilon(restPoints);
    if (std::abs(det) < eps * eps * eps) {
        if (matrix) {
            matrix->SetIdentity();
        }
        return false;
    }

    const GfMatrix3d m3 = mq.GetInverse() * mp;
    const GfVec3d t = posePoints[0] - _TransformDir(restPoints[0], m3);

    if (matrix) {
        *matrix = GfMatrix4d(
            m3[0][0], m3[0][1], m3[0][2], 0.0,
            m3[1][0], m3[1][1], m3[1][2], 0.0,
            m3[2][0], m3[2][1], m3[2][2], 0.0,
            t[0],     t[1],     t[2],     1.0);
    }
    return true;
}

bool
RigExecPointsToMatrix(
    const std::array<GfVec3d, 4> &restPoints,
    const RigExecPointFrame &frame,
    GfMatrix4d *matrix)
{
    return RigExecPointsToMatrix(restPoints, frame.points, matrix);
}

RigExecPointFrame
RigExecMatrixToPoints(
    const std::array<GfVec3d, 4> &restPoints,
    const GfMatrix4d &matrix)
{
    RigExecPointFrame frame;
    for (size_t i = 0; i < 4; ++i) {
        frame.points[i] = matrix.TransformAffine(restPoints[i]);
    }
    frame.flags = RigExecPointFrameValid;

    // Classify: reflection and shear/nonuniform-scale.
    const GfMatrix3d posed = _RowBasis(frame.points);
    if (posed.GetDeterminant() < 0) {
        frame.flags |= RigExecPointFrameReflected;
    }
    const double eps = RigExecFrameEpsilon(restPoints);
    if (std::abs(posed.GetDeterminant()) <= eps * eps * eps) {
        frame.flags |= RigExecPointFrameDegenerate;
    }
    return frame;
}

namespace {

GfVec3d
_WorldAxisLeastParallel(const GfVec3d &ex)
{
    const GfVec3d axes[3] = {
        GfVec3d(1, 0, 0), GfVec3d(0, 1, 0), GfVec3d(0, 0, 1)};
    double best = 2.0;
    GfVec3d bestAxis(0, 1, 0);
    for (const GfVec3d &axis : axes) {
        const double a = std::abs(GfDot(axis, ex));
        if (a < best) {
            best = a;
            bestAxis = axis;
        }
    }
    return bestAxis;
}

// Projects candidate up onto the plane orthogonal to ex; returns
// (projected, length).
std::pair<GfVec3d, double>
_ProjectUp(const GfVec3d &up, const GfVec3d &ex)
{
    const GfVec3d u = up - ex * GfDot(ex, up);
    return {u, u.GetLength()};
}

}  // namespace

RigExecPointFrame
RigExecReconstructFrame(
    const std::array<GfVec3d, 4> &restPoints,
    const std::array<GfVec3d, 4> &posePoints,
    const RigExecFrameReconstructionArgs &args)
{
    RigExecPointFrame frame;
    frame.points = posePoints;
    frame.flags = RigExecPointFrameValid;

    if (args.policy == RigExecFramePolicy::Affine) {
        // Use all four points directly; retain scale/reflection/shear.
        const GfMatrix3d posed = _RowBasis(posePoints);
        const double det = posed.GetDeterminant();
        const double eps = RigExecFrameEpsilon(restPoints);
        frame.flags |= RigExecPointFrameAffine;
        if (det < 0) {
            frame.flags |= RigExecPointFrameReflected;
        }
        if (std::abs(det) <= eps * eps * eps) {
            frame.flags |= RigExecPointFrameDegenerate;
        }
        return frame;
    }

    // Orthogonal family (spec §5.2). The aim/up landmark roles come from
    // the authored aimAxis/upAxis tokens; the remaining landmark carries
    // side scale and handedness.
    const int aimIdx = static_cast<int>(args.aimAxis) + 1;
    const int upIdx = static_cast<int>(args.upAxis) + 1;
    if (upIdx == aimIdx) {
        // aim == up is an invalid authoring configuration: fail explicitly
        // with the raw points rather than silently substituting roles.
        frame.flags |= RigExecPointFrameDegenerate;
        return frame;
    }
    const int sideIdx = 6 - aimIdx - upIdx;
    // Permutation parity of (aim, up, side) relative to (x, y, z): an even
    // permutation completes the frame with side = aim x up; odd negates.
    const bool evenPermutation =
        (aimIdx == 1 && upIdx == 2) || (aimIdx == 2 && upIdx == 3) ||
        (aimIdx == 3 && upIdx == 1);
    const double sideSign = evenPermutation ? 1.0 : -1.0;

    const GfVec3d po = posePoints[0];
    const double eps = RigExecFrameEpsilon(restPoints);

    const double lAim = std::max(
        (restPoints[aimIdx] - restPoints[0]).GetLength(), eps);
    const double lUp = std::max(
        (restPoints[upIdx] - restPoints[0]).GetLength(), eps);
    const double lSide = std::max(
        (restPoints[sideIdx] - restPoints[0]).GetLength(), eps);

    const GfVec3d a = posePoints[aimIdx] - po;
    const double aLen = a.GetLength();
    if (aLen < eps) {
        // Aim invalid: orientation cannot be derived. Keep raw points,
        // flag degenerate (spec §5.3: no silent identity).
        frame.flags |= RigExecPointFrameDegenerate;
        return frame;
    }
    const GfVec3d eAim = a / aLen;

    // Up projection with the deterministic fallback ladder (spec §5.3):
    // authored up landmark, parent computed up, authored rest up,
    // world axis least parallel to the aim direction.
    auto [u, uLen] = _ProjectUp(posePoints[upIdx] - po, eAim);
    bool twistUnderdetermined = false;
    if (uLen < eps) {
        twistUnderdetermined = true;
        if (args.hasParentUp) {
            std::tie(u, uLen) = _ProjectUp(args.parentUp, eAim);
        }
        if (uLen < eps) {
            std::tie(u, uLen) =
                _ProjectUp(restPoints[upIdx] - restPoints[0], eAim);
        }
        if (uLen < eps) {
            std::tie(u, uLen) = _ProjectUp(_WorldAxisLeastParallel(eAim), eAim);
        }
    }
    GfVec3d eUp = u / uLen;
    GfVec3d eSide = sideSign * GfCross(eAim, eUp);
    eUp = sideSign * GfCross(eSide, eAim);  // re-cross removes drift
    eUp.Normalize();
    eSide.Normalize();

    // Scale magnitudes and handedness are measured on the UNTWISTED
    // orthogonal basis (spec §5.2 defines dz against e_z, then applies
    // twist to the reconstructed transverse axes).
    double sAim = aLen / lAim;
    // When the up landmark was degenerate the transverse scale is
    // underdetermined; unit scale is the deterministic choice.
    double sUp = twistUnderdetermined ? 1.0 : uLen / lUp;
    const double dSide = GfDot(posePoints[sideIdx] - po, eSide);
    double sSide = std::abs(dSide) / lSide;
    // Zero side determinant follows the authored reflection policy.
    double sigma = (dSide > 0) ? 1.0 : (dSide < 0 ? -1.0 : args.zeroSideSign);

    switch (args.policy) {
    case RigExecFramePolicy::Rigid:
        // Scale magnitudes only; handedness is separate authored policy.
        sAim = sUp = sSide = 1.0;
        break;
    case RigExecFramePolicy::Axial:
        sUp = sSide = 1.0;
        break;
    case RigExecFramePolicy::Orthogonal:
        if (sSide < eps) {
            // Side landmark in the aim/up plane: no usable side scale.
            sSide = 1.0;
        }
        break;
    default:
        break;
    }

    // Unwrapped twist rotates the transverse axes about the aim axis.
    const double c = std::cos(args.twist);
    const double s = std::sin(args.twist);
    const GfVec3d crossUp = GfCross(eAim, eUp);
    const GfVec3d crossSide = GfCross(eAim, eSide);
    const GfVec3d eUpT = c * eUp + s * crossUp;
    const GfVec3d eSideT = c * eSide + s * crossSide;

    frame.points[0] = po;
    frame.points[aimIdx] = po + lAim * sAim * eAim;
    frame.points[upIdx] = po + lUp * sUp * eUpT;
    frame.points[sideIdx] = po + lSide * sigma * sSide * eSideT;
    if (sigma < 0) {
        frame.flags |= RigExecPointFrameReflected;
    }
    if (twistUnderdetermined) {
        frame.flags |= RigExecPointFrameDegenerate;
    }
    return frame;
}

namespace {

// Jacobi eigen decomposition of a symmetric 3x3 matrix.
// Produces eigenvalues (descending) and orthonormal eigenvectors as the
// *columns* of V (column-convention).
void
_SymmetricEigen3(const double m[3][3], double eval[3], double evec[3][3])
{
    double a[3][3];
    double v[3][3] = {{1, 0, 0}, {0, 1, 0}, {0, 0, 1}};
    for (int i = 0; i < 3; ++i)
        for (int j = 0; j < 3; ++j)
            a[i][j] = m[i][j];

    for (int sweep = 0; sweep < 64; ++sweep) {
        double off = std::abs(a[0][1]) + std::abs(a[0][2]) +
                     std::abs(a[1][2]);
        if (off < 1e-15) break;
        for (int p = 0; p < 2; ++p) {
            for (int q = p + 1; q < 3; ++q) {
                if (std::abs(a[p][q]) < 1e-18) continue;
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
    std::sort(order, order + 3, [&](int x, int y) { return d[x] > d[y]; });
    for (int i = 0; i < 3; ++i) {
        eval[i] = d[order[i]];
        for (int k = 0; k < 3; ++k) {
            evec[k][i] = v[k][order[i]];
        }
        // Deterministic hemisphere canonicalization: flip each eigenvector
        // so its largest-magnitude component is positive. This removes
        // per-vector sign ambiguity only; it cannot stabilize eigenvector
        // rotation within a (nearly) repeated eigenspace, which remains an
        // open gap for the spec §5.4 pinned-axis policy.
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

    // Repeated-eigenspace canonicalization: within a (nearly) repeated
    // eigenvalue group the eigenvectors are only determined up to a
    // rotation of the subspace, so Jacobi output can rotate under tiny
    // perturbations. Rebuild each group's basis deterministically from
    // world-axis projections (spec §5.4 pinned-axis stability).
    const double tol =
        1e-8 * std::max({std::abs(eval[0]), std::abs(eval[2]), 1.0});
    const bool eq01 = std::abs(eval[0] - eval[1]) <= tol;
    const bool eq12 = std::abs(eval[1] - eval[2]) <= tol;
    if (eq01 && eq12) {
        // Full isotropy: the canonical basis is the world basis.
        for (int i = 0; i < 3; ++i)
            for (int k = 0; k < 3; ++k)
                evec[k][i] = (k == i) ? 1.0 : 0.0;
    } else if (eq01 || eq12) {
        const int a = eq01 ? 0 : 1;   // repeated pair indices (a, a+1)
        const int other = eq01 ? 2 : 0;
        // Unit normal of the repeated 2D subspace = the distinct vector.
        const double n[3] = {evec[0][other], evec[1][other], evec[2][other]};
        // First basis vector: the world axis with the largest projection
        // into the subspace, projected and normalized.
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
                for (int k = 0; k < 3; ++k) b1[k] = p[k] / len;
            }
        }
        (void)bestAxis;
        // Second basis vector completes the subspace: b2 = n x b1.
        double b2[3] = {
            n[1] * b1[2] - n[2] * b1[1],
            n[2] * b1[0] - n[0] * b1[2],
            n[0] * b1[1] - n[1] * b1[0]};
        // Sign-canonicalize b2 like the per-vector rule above.
        int maxK = 0;
        for (int k = 1; k < 3; ++k) {
            if (std::abs(b2[k]) > std::abs(b2[maxK])) maxK = k;
        }
        if (b2[maxK] < 0) {
            for (int k = 0; k < 3; ++k) b2[k] = -b2[k];
        }
        for (int k = 0; k < 3; ++k) {
            evec[k][a] = b1[k];
            evec[k][a + 1] = b2[k];
        }
    }
}

}  // namespace

bool
RigExecPointsToParams(
    const std::array<GfVec3d, 4> &restPoints,
    const std::array<GfVec3d, 4> &posePoints,
    RigExecAxis reflectionAxis,
    RigExecTransformParams *params)
{
    GfMatrix4d m;
    if (!RigExecPointsToMatrix(restPoints, posePoints, &m)) {
        return false;
    }

    // Column-convention linear part L: L[i][j] = m[j][i] for the row-vector
    // stored matrix.
    double L[3][3];
    for (int i = 0; i < 3; ++i)
        for (int j = 0; j < 3; ++j)
            L[i][j] = m[j][i];

    // SVD via eigen decomposition of L^T L = V S^2 V^T (spec §5.4).
    double ltl[3][3];
    for (int i = 0; i < 3; ++i) {
        for (int j = 0; j < 3; ++j) {
            double sum = 0;
            for (int k = 0; k < 3; ++k) sum += L[k][i] * L[k][j];
            ltl[i][j] = sum;
        }
    }
    double s2[3], V[3][3];
    _SymmetricEigen3(ltl, s2, V);

    double sigma[3];
    for (int i = 0; i < 3; ++i) {
        sigma[i] = std::sqrt(std::max(s2[i], 0.0));
    }
    const double detL =
        L[0][0] * (L[1][1] * L[2][2] - L[1][2] * L[2][1]) -
        L[0][1] * (L[1][0] * L[2][2] - L[1][2] * L[2][0]) +
        L[0][2] * (L[1][0] * L[2][1] - L[1][1] * L[2][0]);
    if (sigma[2] < 1e-14 * std::max(1.0, sigma[0])) {
        return false;  // singular posed frame: no SRT decomposition
    }

    // U = L V S^-1.
    double U[3][3];
    for (int i = 0; i < 3; ++i) {
        for (int j = 0; j < 3; ++j) {
            double sum = 0;
            for (int k = 0; k < 3; ++k) sum += L[i][k] * V[k][j];
            U[i][j] = sum / sigma[j];
        }
    }

    // delta = det(U V^T) = sign(det L) for nonsingular L.
    const double delta = (detL >= 0) ? 1.0 : -1.0;

    // Select singular index k: right-singular direction most aligned with
    // the pinned reflection axis (spec §5.4). Immaterial when delta = +1.
    int k = 2;
    if (delta < 0) {
        const int axis = static_cast<int>(reflectionAxis);
        double best = -1.0;
        for (int i = 0; i < 3; ++i) {
            const double align = std::abs(V[axis][i]);
            if (align > best) {
                best = align;
                k = i;
            }
        }
    }

    // R = U D V^T with D = diag(1,1,delta at index k).
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

    // H = R^T L: upper-triangular-ish stretch with scale on the diagonal.
    double H[3][3];
    for (int i = 0; i < 3; ++i) {
        for (int j = 0; j < 3; ++j) {
            double sum = 0;
            for (int kk = 0; kk < 3; ++kk) sum += R[kk][i] * L[kk][j];
            H[i][j] = sum;
        }
    }

    // Quaternion from the proper rotation R (column convention). Build a
    // row-vector GfMatrix3d = R^T and use Gf's extraction.
    GfMatrix3d rowR(
        R[0][0], R[1][0], R[2][0],
        R[0][1], R[1][1], R[2][1],
        R[0][2], R[1][2], R[2][2]);
    rowR.Orthonormalize();
    const GfQuatd q = rowR.ExtractRotation().GetQuat();

    params->translation = GfVec3d(m[3][0], m[3][1], m[3][2]);
    params->rotation = q.GetNormalized();
    params->scale = GfVec3d(H[0][0], H[1][1], H[2][2]);
    params->shear = GfVec3d(
        H[0][1] / H[0][0],
        H[0][2] / H[0][0],
        H[1][2] / H[1][1]);
    params->reflectionAxis = reflectionAxis;
    return true;
}

GfMatrix4d
RigExecParamsToMatrix(const RigExecTransformParams &params)
{
    // L = R(q) H(s,h) in column convention; produce the row-vector
    // GfMatrix4d with translation in row 3.
    GfMatrix3d rowR;
    rowR.SetRotate(params.rotation);
    // rowR is R^T (row-vector). Column R: R[i][j] = rowR[j][i].
    double R[3][3];
    for (int i = 0; i < 3; ++i)
        for (int j = 0; j < 3; ++j)
            R[i][j] = rowR[j][i];

    const GfVec3d &sc = params.scale;
    const GfVec3d &sh = params.shear;
    // H is the SYMMETRIC polar stretch R^T L: diag(scale) with symmetric
    // off-diagonals h_xy*sx, h_xz*sx, h_yz*sy (inverse of the
    // normalization applied at decomposition). Rebuilding it upper
    // triangular would discard half the stretch and break the round trip.
    double H[3][3] = {
        {sc[0],         sh[0] * sc[0], sh[1] * sc[0]},
        {sh[0] * sc[0], sc[1],         sh[2] * sc[1]},
        {sh[1] * sc[0], sh[2] * sc[1], sc[2]},
    };

    double L[3][3];
    for (int i = 0; i < 3; ++i) {
        for (int j = 0; j < 3; ++j) {
            double sum = 0;
            for (int k = 0; k < 3; ++k) sum += R[i][k] * H[k][j];
            L[i][j] = sum;
        }
    }

    return GfMatrix4d(
        L[0][0], L[1][0], L[2][0], 0.0,
        L[0][1], L[1][1], L[2][1], 0.0,
        L[0][2], L[1][2], L[2][2], 0.0,
        params.translation[0], params.translation[1],
        params.translation[2], 1.0);
}

}  // namespace rigExec
