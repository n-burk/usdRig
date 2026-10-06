// Shared, USD-free arithmetic of RigExecAutoClavicle. The dynamic evaluator,
// the baked program and the binary runtime all call RigExecAutoClavicleShift,
// so the three agree by construction.
//
// Conventions: frames are 16 doubles, row-major, row vectors (a point maps as
// p * M; the three handle vectors are rows 0-2 and the origin is row 3), as
// RigExecPointFrame and the runtime's frames publish them. Quaternions are
// (w, x, y, z) and rotate as q v q*, the same rotation Gf builds from them.
#ifndef RIGEXEC_MATH_AUTO_CLAVICLE_KERNEL_H
#define RIGEXEC_MATH_AUTO_CLAVICLE_KERNEL_H

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <limits>
#include <vector>

namespace rigExec {

/// What RigExecAutoClavicle carries from compile to every frame. The pose
/// interpolation constants are the RigExecRbfSolver's own, solved once at
/// compile; the frame-dependent quantities (rest direction, rest root, bone
/// lengths) are re-read from the default frames each frame instead, so a
/// rest edit needs no recompile of these numbers.
struct RigExecAutoClavicleConstants {
    /// rigExec:basis, rotation rows in the anchor's frame (row-major 3x3).
    double basis[9] = {1, 0, 0, 0, 1, 0, 0, 0, 1};
    double ikValue = 1.0;
    double gain = 0.4;
    /// 0 gaussian, 1 linear.
    int kernel = 0;
    bool normalize = true;
    /// Per pose: the swing part about X as (w, x, y, z), its width in
    /// radians, and its gain. `weights` is the solved n x n matrix,
    /// row-major, as RigExecRbfSolver::GetWeights() returns it.
    std::vector<double> swings;
    std::vector<double> widths;
    std::vector<double> gains;
    std::vector<double> weights;

    size_t PoseCount() const { return widths.size(); }
};

/// One frame's inputs. Null pointers are absent optional inputs.
struct RigExecAutoClavicleFrames {
    const double *anchorPosed = nullptr;
    const double *anchorDefault = nullptr;
    const double *pivotPosed = nullptr;
    const double *targetPosed = nullptr;
    /// The first FK control's posed frame, and all three default frames.
    const double *fkPosed = nullptr;
    const double *fkDefault[3] = {nullptr, nullptr, nullptr};
    const double *ikTargetPosed = nullptr;
    const double *polePosed = nullptr;
    double ikBlend = 0.0;
    double amount = 1.0;
};

namespace autoClavicleKernel {

inline double
Dot(const double *a, const double *b)
{
    return a[0] * b[0] + a[1] * b[1] + a[2] * b[2];
}

inline void
Cross(const double *a, const double *b, double *out)
{
    const double x = a[1] * b[2] - a[2] * b[1];
    const double y = a[2] * b[0] - a[0] * b[2];
    const double z = a[0] * b[1] - a[1] * b[0];
    out[0] = x;
    out[1] = y;
    out[2] = z;
}

inline double
Normalize(double *v)
{
    const double length = std::sqrt(Dot(v, v));
    if (length > 0.0) {
        v[0] /= length;
        v[1] /= length;
        v[2] /= length;
    }
    return length;
}

/// v * M over the frame's 3x3 rows, with each row normalized first: the
/// direction a frame carries, whatever scale it holds.
inline void
RotateByFrame(const double *v, const double *frame, double *out)
{
    double rows[3][3];
    for (int r = 0; r < 3; ++r) {
        for (int k = 0; k < 3; ++k) rows[r][k] = frame[r * 4 + k];
        Normalize(rows[r]);
    }
    for (int k = 0; k < 3; ++k) {
        out[k] = v[0] * rows[0][k] + v[1] * rows[1][k] + v[2] * rows[2][k];
    }
}

/// v * M^T over the normalized rows: a world direction into the frame's
/// axes.
inline void
UnrotateByFrame(const double *v, const double *frame, double *out)
{
    double rows[3][3];
    for (int r = 0; r < 3; ++r) {
        for (int k = 0; k < 3; ++k) rows[r][k] = frame[r * 4 + k];
        Normalize(rows[r]);
    }
    for (int r = 0; r < 3; ++r) out[r] = Dot(v, rows[r]);
}

/// The 4x4 inverse of a frame, by cofactors. False when singular.
inline bool
InvertFrame(const double *m, double *out)
{
    double inv[16];
    inv[0] = m[5] * m[10] * m[15] - m[5] * m[11] * m[14] - m[9] * m[6] * m[15] +
             m[9] * m[7] * m[14] + m[13] * m[6] * m[11] - m[13] * m[7] * m[10];
    inv[4] = -m[4] * m[10] * m[15] + m[4] * m[11] * m[14] + m[8] * m[6] * m[15] -
             m[8] * m[7] * m[14] - m[12] * m[6] * m[11] + m[12] * m[7] * m[10];
    inv[8] = m[4] * m[9] * m[15] - m[4] * m[11] * m[13] - m[8] * m[5] * m[15] +
             m[8] * m[7] * m[13] + m[12] * m[5] * m[11] - m[12] * m[7] * m[9];
    inv[12] = -m[4] * m[9] * m[14] + m[4] * m[10] * m[13] + m[8] * m[5] * m[14] -
              m[8] * m[6] * m[13] - m[12] * m[5] * m[10] + m[12] * m[6] * m[9];
    inv[1] = -m[1] * m[10] * m[15] + m[1] * m[11] * m[14] + m[9] * m[2] * m[15] -
             m[9] * m[3] * m[14] - m[13] * m[2] * m[11] + m[13] * m[3] * m[10];
    inv[5] = m[0] * m[10] * m[15] - m[0] * m[11] * m[14] - m[8] * m[2] * m[15] +
             m[8] * m[3] * m[14] + m[12] * m[2] * m[11] - m[12] * m[3] * m[10];
    inv[9] = -m[0] * m[9] * m[15] + m[0] * m[11] * m[13] + m[8] * m[1] * m[15] -
             m[8] * m[3] * m[13] - m[12] * m[1] * m[11] + m[12] * m[3] * m[9];
    inv[13] = m[0] * m[9] * m[14] - m[0] * m[10] * m[13] - m[8] * m[1] * m[14] +
              m[8] * m[2] * m[13] + m[12] * m[1] * m[10] - m[12] * m[2] * m[9];
    inv[2] = m[1] * m[6] * m[15] - m[1] * m[7] * m[14] - m[5] * m[2] * m[15] +
             m[5] * m[3] * m[14] + m[13] * m[2] * m[7] - m[13] * m[3] * m[6];
    inv[6] = -m[0] * m[6] * m[15] + m[0] * m[7] * m[14] + m[4] * m[2] * m[15] -
             m[4] * m[3] * m[14] - m[12] * m[2] * m[7] + m[12] * m[3] * m[6];
    inv[10] = m[0] * m[5] * m[15] - m[0] * m[7] * m[13] - m[4] * m[1] * m[15] +
              m[4] * m[3] * m[13] + m[12] * m[1] * m[7] - m[12] * m[3] * m[5];
    inv[14] = -m[0] * m[5] * m[14] + m[0] * m[6] * m[13] + m[4] * m[1] * m[14] -
              m[4] * m[2] * m[13] - m[12] * m[1] * m[6] + m[12] * m[2] * m[5];
    inv[3] = -m[1] * m[6] * m[11] + m[1] * m[7] * m[10] + m[5] * m[2] * m[11] -
             m[5] * m[3] * m[10] - m[9] * m[2] * m[7] + m[9] * m[3] * m[6];
    inv[7] = m[0] * m[6] * m[11] - m[0] * m[7] * m[10] - m[4] * m[2] * m[11] +
             m[4] * m[3] * m[10] + m[8] * m[2] * m[7] - m[8] * m[3] * m[6];
    inv[11] = -m[0] * m[5] * m[11] + m[0] * m[7] * m[9] + m[4] * m[1] * m[11] -
              m[4] * m[3] * m[9] - m[8] * m[1] * m[7] + m[8] * m[3] * m[5];
    inv[15] = m[0] * m[5] * m[10] - m[0] * m[6] * m[9] - m[4] * m[1] * m[10] +
              m[4] * m[2] * m[9] + m[8] * m[1] * m[6] - m[8] * m[2] * m[5];
    const double det = m[0] * inv[0] + m[1] * inv[4] + m[2] * inv[8] + m[3] * inv[12];
    if (!std::isfinite(det) || std::abs(det) < 1e-300) {
        return false;
    }
    for (int i = 0; i < 16; ++i) out[i] = inv[i] / det;
    return true;
}

/// p * M for a point (w = 1).
inline void
TransformPoint(const double *p, const double *m, double *out)
{
    double r[3];
    for (int k = 0; k < 3; ++k) {
        r[k] = p[0] * m[0 * 4 + k] + p[1] * m[1 * 4 + k] + p[2] * m[2 * 4 + k] +
               m[3 * 4 + k];
    }
    out[0] = r[0];
    out[1] = r[1];
    out[2] = r[2];
}

/// Rotates v about the unit `axis` by `angle` radians (Rodrigues).
inline void
RotateAbout(const double *v, const double *axis, double angle, double *out)
{
    const double c = std::cos(angle), s = std::sin(angle);
    double kxv[3];
    Cross(axis, v, kxv);
    const double kv = Dot(axis, v);
    for (int k = 0; k < 3; ++k) {
        out[k] = v[k] * c + kxv[k] * s + axis[k] * kv * (1.0 - c);
    }
}

/// Spherical interpolation between unit vectors.
inline void
SlerpUnit(const double *a, const double *b, double t, double *out)
{
    if (t <= 0.0) {
        out[0] = a[0], out[1] = a[1], out[2] = a[2];
        return;
    }
    if (t >= 1.0) {
        out[0] = b[0], out[1] = b[1], out[2] = b[2];
        return;
    }
    double axis[3];
    Cross(a, b, axis);
    const double angle = std::atan2(Normalize(axis), Dot(a, b));
    if (angle < 1e-12) {
        out[0] = a[0], out[1] = a[1], out[2] = a[2];
        return;
    }
    RotateAbout(a, axis, angle * t, out);
}

/// The swing about X of q, and the angle between two quaternions, as
/// RigExecRbfSwingTwist and RigExecRbfAngleBetween compute them.
inline void
SwingAboutX(const double *q, double *swing)
{
    const double w = q[0], x = q[1], y = q[2], z = q[3];
    double tw[4] = {w, x, 0.0, 0.0};
    const double size = std::sqrt(tw[0] * tw[0] + tw[1] * tw[1]);
    if (size < 1.0e-9) {
        tw[0] = 1.0;
        tw[1] = 0.0;
    } else {
        tw[0] /= size;
        tw[1] /= size;
    }
    const double tw_w = tw[0], tw_x = -tw[1], tw_y = 0.0, tw_z = 0.0;
    swing[0] = w * tw_w - x * tw_x - y * tw_y - z * tw_z;
    swing[1] = w * tw_x + x * tw_w + y * tw_z - z * tw_y;
    swing[2] = w * tw_y - x * tw_z + y * tw_w + z * tw_x;
    swing[3] = w * tw_z + x * tw_y - y * tw_x + z * tw_w;
}

inline double
AngleBetween(const double *a, const double *b)
{
    const double dot = a[0] * b[0] + a[1] * b[1] + a[2] * b[2] + a[3] * b[3];
    return 2.0 * std::acos(std::min(1.0, std::abs(dot)));
}

/// Pose weights of the swing taking the basis X axis onto unit `d`
/// (basis axes): RigExecRbfSolver::Evaluate's kernels, solve and
/// normalisation, for swing poses about X with negative weights kept.
inline void
PoseWeights(const RigExecAutoClavicleConstants &c, const double *d,
            std::vector<double> *out)
{
    const size_t n = c.PoseCount();
    out->assign(n, 0.0);
    if (n == 0 || c.weights.size() != n * n || c.swings.size() != n * 4) {
        out->clear();
        return;
    }
    // The shortest arc from X to d.
    double q[4];
    const double dot = d[0];
    if (dot < -1.0 + 1e-12) {
        q[0] = 0.0, q[1] = 0.0, q[2] = 1.0, q[3] = 0.0;
    } else {
        q[0] = 1.0 + dot, q[1] = 0.0, q[2] = -d[2], q[3] = d[1];
        const double size =
            std::sqrt(q[0] * q[0] + q[1] * q[1] + q[2] * q[2] + q[3] * q[3]);
        for (double &v : q) v /= size;
    }
    double swing[4];
    SwingAboutX(q, swing);
    std::vector<double> kernels(n);
    for (size_t i = 0; i < n; ++i) {
        const double angle = AngleBetween(swing, &c.swings[i * 4]);
        const double width = c.widths[i];
        const double ratio = width <= 0.0
            ? (angle <= 0.0 ? 0.0 : std::numeric_limits<double>::infinity())
            : angle / width;
        kernels[i] = c.kernel == 1 ? std::max(0.0, 1.0 - ratio)
                                   : std::exp(-(ratio * ratio));
    }
    for (size_t row = 0; row < n; ++row) {
        double total = 0.0;
        for (size_t i = 0; i < n; ++i) {
            total += c.weights[row * n + i] * kernels[i];
        }
        (*out)[row] = total;
    }
    if (c.normalize) {
        double total = 0.0;
        for (double v : *out) total += v;
        // RigExecRbfNormalizeFloor.
        if (std::abs(total) >= 1.0e-6) {
            for (double &v : *out) v /= total;
        }
    }
}

}  // namespace autoClavicleKernel

/// The world-space translation RigExecAutoClavicle adds to its target's
/// origin this frame, in `delta`, and the blend weight in `weight`. False
/// (and a zero delta) when a required frame is missing or degenerate.
inline bool
RigExecAutoClavicleShift(const RigExecAutoClavicleConstants &c,
                         const RigExecAutoClavicleFrames &f, double delta[3],
                         double *weight = nullptr)
{
    using namespace autoClavicleKernel;
    delta[0] = delta[1] = delta[2] = 0.0;
    if (weight) *weight = 0.0;
    if (!f.anchorPosed || !f.anchorDefault || !f.pivotPosed ||
        !f.targetPosed || !f.fkPosed || !f.fkDefault[0] || !f.fkDefault[1] ||
        !f.fkDefault[2]) {
        return false;
    }
    // The rest bone axis, in the first FK control's own axes and in world.
    const double *root0 = f.fkDefault[0] + 12;
    const double *mid0 = f.fkDefault[1] + 12;
    double restAxis[3] = {mid0[0] - root0[0], mid0[1] - root0[1],
                          mid0[2] - root0[2]};
    if (Normalize(restAxis) <= 0.0) return false;
    double axisLocal[3];
    UnrotateByFrame(restAxis, f.fkDefault[0], axisLocal);
    double dFk[3];
    RotateByFrame(axisLocal, f.fkPosed, dFk);
    Normalize(dFk);

    // The rest direction, carried by the anchor.
    double restInAnchor[3], d0[3];
    UnrotateByFrame(restAxis, f.anchorDefault, restInAnchor);
    RotateByFrame(restInAnchor, f.anchorPosed, d0);
    Normalize(d0);

    // The shift for a direction: a fraction w of the shortest arc from the
    // rest direction to d, turning the target's origin about the pivot's.
    const double *origin = f.targetPosed + 12;
    const double *pivot = f.pivotPosed + 12;
    std::vector<double> poses;
    const auto shiftFor = [&](const double *d, double out[3], double *wOut) {
        out[0] = out[1] = out[2] = 0.0;
        double inAnchor[3];
        UnrotateByFrame(d, f.anchorPosed, inAnchor);
        double inBasis[3];
        for (int r = 0; r < 3; ++r) {
            double row[3] = {c.basis[r * 3 + 0], c.basis[r * 3 + 1],
                             c.basis[r * 3 + 2]};
            Normalize(row);
            inBasis[r] = Dot(inAnchor, row);
        }
        Normalize(inBasis);
        PoseWeights(c, inBasis, &poses);
        double sum = 0.0;
        for (size_t i = 0; i < poses.size() && i < c.gains.size(); ++i) {
            sum += c.gains[i] * std::clamp(poses[i], 0.0, 1.0);
        }
        double w = c.gain * f.amount * sum;
        w = std::isfinite(w) ? std::clamp(w, 0.0, 1.0) : 0.0;
        *wOut = w;
        double axis[3];
        Cross(d0, d, axis);
        const double angle = std::atan2(Normalize(axis), Dot(d0, d)) * w;
        if (!(angle > 1e-12)) return;
        double arm[3] = {origin[0] - pivot[0], origin[1] - pivot[1],
                         origin[2] - pivot[2]};
        double turned[3];
        RotateAbout(arm, axis, angle, turned);
        for (int k = 0; k < 3; ++k) out[k] = turned[k] - arm[k];
    };

    const double span = 2.0 * c.ikValue - 1.0;
    double t = span != 0.0 ? (f.ikBlend - (1.0 - c.ikValue)) / span : 0.0;
    t = std::isfinite(t) ? std::clamp(t, 0.0, 1.0) : 0.0;
    double d[3] = {dFk[0], dFk[1], dFk[2]};
    double w = 0.0;
    if (f.ikTargetPosed && t > 0.0) {
        // The IK estimate: the upper bone of a two-bone solve from the limb's
        // root -- the target's origin carried by its own shift -- toward the
        // IK target, bending toward the pole. The root depends on the shift
        // and the shift on the direction, so the pair is iterated to its
        // fixed point; there an IK limb matched to an FK one gives the FK
        // direction back, so a matched switch moves nothing.
        double toAnchor[16], carry[16];
        if (!InvertFrame(f.anchorDefault, toAnchor)) return false;
        for (int r = 0; r < 4; ++r) {
            for (int k = 0; k < 4; ++k) {
                carry[r * 4 + k] = toAnchor[r * 4 + 0] * f.anchorPosed[0 * 4 + k] +
                                   toAnchor[r * 4 + 1] * f.anchorPosed[1 * 4 + k] +
                                   toAnchor[r * 4 + 2] * f.anchorPosed[2 * 4 + k] +
                                   toAnchor[r * 4 + 3] * f.anchorPosed[3 * 4 + k];
            }
        }
        double root0w[3], mid0w[3], end0w[3];
        TransformPoint(root0, carry, root0w);
        TransformPoint(mid0, carry, mid0w);
        TransformPoint(f.fkDefault[2] + 12, carry, end0w);
        double ab[3] = {mid0w[0] - root0w[0], mid0w[1] - root0w[1],
                        mid0w[2] - root0w[2]};
        double bc[3] = {end0w[0] - mid0w[0], end0w[1] - mid0w[1],
                        end0w[2] - mid0w[2]};
        const double a = std::sqrt(Dot(ab, ab)), b = std::sqrt(Dot(bc, bc));
        const double *target = f.ikTargetPosed + 12;
        const auto solve = [&](const double *shift, double *out) {
            const double root[3] = {origin[0] + shift[0],
                                    origin[1] + shift[1],
                                    origin[2] + shift[2]};
            double v[3] = {target[0] - root[0], target[1] - root[1],
                           target[2] - root[2]};
            const double dist = Normalize(v);
            if (!(dist > 1e-9 && a > 1e-9 && b > 1e-9)) return false;
            const double reach = std::clamp(dist, std::abs(a - b), a + b);
            const double cosA = std::clamp(
                (a * a + reach * reach - b * b) / (2.0 * a * reach), -1.0, 1.0);
            const double sinA = std::sqrt(std::max(0.0, 1.0 - cosA * cosA));
            double bend[3] = {dFk[0], dFk[1], dFk[2]};
            if (f.polePosed) {
                const double *pole = f.polePosed + 12;
                bend[0] = pole[0] - root[0];
                bend[1] = pole[1] - root[1];
                bend[2] = pole[2] - root[2];
            }
            double along = Dot(bend, v);
            for (int k = 0; k < 3; ++k) bend[k] -= v[k] * along;
            if (Normalize(bend) < 1e-9) {
                for (int k = 0; k < 3; ++k) bend[k] = dFk[k];
                along = Dot(bend, v);
                for (int k = 0; k < 3; ++k) bend[k] -= v[k] * along;
                Normalize(bend);
            }
            double dIk[3];
            for (int k = 0; k < 3; ++k) dIk[k] = v[k] * cosA + bend[k] * sinA;
            Normalize(dIk);
            SlerpUnit(dFk, dIk, t, out);
            Normalize(out);
            return true;
        };
        double shift[3] = {0.0, 0.0, 0.0};
        if (solve(shift, d)) {
            // A fixed number of passes, so every path takes the same ones.
            for (int pass = 0; pass < 12; ++pass) {
                shiftFor(d, shift, &w);
                double next[3];
                if (!solve(shift, next)) break;
                const double moved = std::abs(next[0] - d[0]) +
                                     std::abs(next[1] - d[1]) +
                                     std::abs(next[2] - d[2]);
                d[0] = next[0], d[1] = next[1], d[2] = next[2];
                if (moved == 0.0) break;
            }
        }
    }
    shiftFor(d, delta, &w);
    if (weight) *weight = w;
    return true;
}

}  // namespace rigExec

#endif
