//
// RigExec solver kernels.
//
#include "solvers.h"

#include "pxr/base/gf/matrix3d.h"
#include "pxr/base/gf/rotation.h"

#include <cmath>

namespace rigExec {

std::vector<RigExecPointFrame>
RigExecSolveFkChain(const std::vector<RigExecFkChainElement> &elements)
{
    std::vector<RigExecPointFrame> result;
    result.reserve(elements.size());

    // Accumulated rest->pose maps, row-vector convention. W_i (column
    // convention) = W_parent ∘ A_i maps to row storage W_row = A_row * Wp_row.
    std::vector<GfMatrix4d> accumulated;
    accumulated.reserve(elements.size());

    for (size_t i = 0; i < elements.size(); ++i) {
        const RigExecFkChainElement &e = elements[i];
        GfMatrix4d own;
        RigExecPointsToMatrix(e.restPoints, e.posePoints, &own);

        GfMatrix4d w = own;
        if (e.parentIndex >= 0 &&
            static_cast<size_t>(e.parentIndex) < accumulated.size()) {
            w = own * accumulated[e.parentIndex];
        }
        accumulated.push_back(w);
        result.push_back(RigExecMatrixToPoints(e.restPoints, w));
    }
    return result;
}

namespace {

// Rotates v about unit axis by angle (Rodrigues).
GfVec3d
_Rotate(const GfVec3d &v, const GfVec3d &axis, double angle)
{
    const double c = std::cos(angle);
    const double s = std::sin(angle);
    return v * c + GfCross(axis, v) * s + axis * GfDot(axis, v) * (1 - c);
}

// Builds a posed frame from an origin plus orthonormal aim/up directions,
// preserving the rest landmark handle lengths (rigid frame).
RigExecPointFrame
_FrameFromAxes(
    const std::array<GfVec3d, 4> &restPoints,
    const GfVec3d &origin,
    const GfVec3d &ex,
    const GfVec3d &eyCandidate)
{
    GfVec3d ey = eyCandidate - ex * GfDot(ex, eyCandidate);
    const double eyLen = ey.GetLength();
    if (eyLen < 1e-12) {
        // Deterministic world-axis completion (spec §5.3 ladder tail).
        const GfVec3d axes[3] = {
            GfVec3d(0, 1, 0), GfVec3d(0, 0, 1), GfVec3d(1, 0, 0)};
        for (const GfVec3d &axis : axes) {
            ey = axis - ex * GfDot(ex, axis);
            if (ey.GetLength() >= 1e-12) break;
        }
    }
    ey.Normalize();
    GfVec3d ez = GfCross(ex, ey);
    ez.Normalize();

    const double lx = (restPoints[1] - restPoints[0]).GetLength();
    const double ly = (restPoints[2] - restPoints[0]).GetLength();
    const double lz = (restPoints[3] - restPoints[0]).GetLength();

    RigExecPointFrame f;
    f.points[0] = origin;
    f.points[1] = origin + ex * lx;
    f.points[2] = origin + ey * ly;
    f.points[3] = origin + ez * lz;
    f.flags = RigExecPointFrameValid;
    return f;
}

}  // namespace

std::array<RigExecPointFrame, 3>
RigExecSolveTwoBoneIk(
    const RigExecPointFrame &rootFrame,
    const RigExecPointFrame &effectorFrame,
    const RigExecPointFrame &poleFrame,
    const std::array<std::array<GfVec3d, 4>, 3> &restPoints,
    const RigExecTwoBoneIkParams &params)
{
    const GfVec3d root = rootFrame.Origin();
    const GfVec3d goal = effectorFrame.Origin();
    const GfVec3d pole = poleFrame.Origin();

    const double l1 = std::max(params.upperLength, 1e-9);
    const double l2 = std::max(params.lowerLength, 1e-9);
    const double chain = l1 + l2;

    GfVec3d toGoal = goal - root;
    double dist = toGoal.GetLength();
    GfVec3d aim;
    if (dist > 1e-12) {
        aim = toGoal / dist;
    } else {
        // Coincident goal: fall back to the root aim handle; if that is
        // also degenerate, fail atomically — return the input frames
        // flagged degenerate rather than normalizing a zero vector
        // (spec §6.6: no silent identity, no NaNs).
        const GfVec3d rootAim = rootFrame.X() - root;
        if (rootAim.GetLength() < 1e-12) {
            std::array<RigExecPointFrame, 3> failed = {
                rootFrame, rootFrame, effectorFrame};
            for (auto &f : failed) {
                f.flags |= RigExecPointFrameDegenerate;
            }
            return failed;
        }
        aim = rootAim.GetNormalized();
    }

    // Soft clamp near full reach ("clampWithSoftness"): beyond
    // (chain - softness) the effective distance eases exponentially toward
    // the chain length instead of hard-clamping.
    const double soft = std::max(params.softness, 0.0) * chain;
    double reach = dist;
    if (soft > 1e-12 && dist > chain - soft) {
        reach = chain - soft * std::exp(-(dist - (chain - soft)) / soft);
    } else if (dist > chain) {
        reach = chain;
    }

    // Uniform-segment stretch: blend segment scaling toward the amount
    // needed to reach the raw goal distance.
    double s1 = l1, s2 = l2;
    if (dist > reach && params.stretch > 0.0) {
        const double factor = 1.0 + (dist / chain - 1.0) *
            std::min(std::max(params.stretch, 0.0), 1.0);
        if (factor > 1.0) {
            s1 = l1 * factor;
            s2 = l2 * factor;
            reach = std::min(dist, s1 + s2);
        }
    }
    reach = std::min(reach, s1 + s2);
    reach = std::max(reach, std::abs(s1 - s2) + 1e-12);

    // Bend plane normal from the pole; deterministic preferred-bend
    // completion when the pole is on the aim line (spec §5.3 ladder shape:
    // pole, rest up, then the world axis least parallel to the aim).
    GfVec3d poleDir = pole - root;
    poleDir -= aim * GfDot(aim, poleDir);
    GfVec3d bendUp;
    if (poleDir.GetLength() > 1e-12) {
        bendUp = poleDir.GetNormalized();
    } else {
        GfVec3d candidate =
            restPoints[0][2] - restPoints[0][0];
        candidate -= aim * GfDot(aim, candidate);
        if (candidate.GetLength() < 1e-12) {
            // World axis least parallel to the aim always has a nonzero
            // projection: |dot| < 1 for a unit axis, so this terminates.
            const GfVec3d axes[3] = {
                GfVec3d(1, 0, 0), GfVec3d(0, 1, 0), GfVec3d(0, 0, 1)};
            double best = 2.0;
            for (const GfVec3d &axis : axes) {
                const double align = std::abs(GfDot(axis, aim));
                if (align < best) {
                    best = align;
                    candidate = axis - aim * GfDot(aim, axis);
                }
            }
        }
        bendUp = _Rotate(candidate.GetNormalized(),
                         aim, params.preferredBendRadians);
    }
    GfVec3d bendAxis = GfCross(aim, bendUp);
    if (bendAxis.GetLength() < 1e-12) {
        // bendUp is guaranteed off-aim by construction; guard anyway.
        bendAxis = GfCross(aim, _Rotate(bendUp, aim, 0.5 * M_PI));
    }
    bendAxis.Normalize();

    // Law of cosines for the shoulder interior angle.
    const double cosAlpha =
        (s1 * s1 + reach * reach - s2 * s2) / (2 * s1 * reach);
    const double alpha = std::acos(std::min(std::max(cosAlpha, -1.0), 1.0));

    const GfVec3d upperDir = _Rotate(aim, bendAxis, alpha);
    const GfVec3d mid = root + upperDir * s1;
    const GfVec3d endPos = root + aim * reach;
    const GfVec3d lowerDir = (endPos - mid).GetNormalized();

    std::array<RigExecPointFrame, 3> out;
    out[0] = _FrameFromAxes(restPoints[0], root, upperDir, bendUp);
    out[1] = _FrameFromAxes(restPoints[1], mid, lowerDir, bendUp);

    // End frame copies the effector orientation at the solved end position.
    RigExecPointFrame end = effectorFrame;
    const GfVec3d offset = endPos - effectorFrame.Origin();
    for (auto &p : end.points) {
        p += offset;
    }
    out[2] = end;
    return out;
}

RigExecPointFrame
RigExecBlendFrames(
    const RigExecPointFrame &a,
    const RigExecPointFrame &b,
    const std::array<GfVec3d, 4> &restPoints,
    double weight,
    RigExecRotationBlend rotationBlend,
    RigExecScaleBlend scaleBlend)
{
    const double w = std::min(std::max(weight, 0.0), 1.0);
    if (w <= 0.0) return a;
    if (w >= 1.0) return b;

    RigExecTransformParams pa, pb;
    if (!RigExecPointsToParams(restPoints, a.points, RigExecAxis::Z, &pa) ||
        !RigExecPointsToParams(restPoints, b.points, RigExecAxis::Z, &pb)) {
        // A singular input frame cannot blend through SRT; hold the
        // stronger input and flag it (no silent identity, spec §6.6).
        RigExecPointFrame held = (w < 0.5) ? a : b;
        held.flags |= RigExecPointFrameDegenerate;
        return held;
    }

    RigExecTransformParams pr;
    pr.translation = pa.translation * (1 - w) + pb.translation * w;

    GfQuatd qa = pa.rotation, qb = pb.rotation;
    if (rotationBlend == RigExecRotationBlend::ShortestArc &&
        GfDot(qa.GetImaginary(), qb.GetImaginary()) +
            qa.GetReal() * qb.GetReal() < 0) {
        qb = -qb;
    }
    pr.rotation = GfSlerp(w, qa, qb).GetNormalized();

    for (int i = 0; i < 3; ++i) {
        if (scaleBlend == RigExecScaleBlend::Log &&
            pa.scale[i] > 0 && pb.scale[i] > 0) {
            pr.scale[i] = std::exp(
                std::log(pa.scale[i]) * (1 - w) + std::log(pb.scale[i]) * w);
        } else {
            pr.scale[i] = pa.scale[i] * (1 - w) + pb.scale[i] * w;
        }
        pr.shear[i] = pa.shear[i] * (1 - w) + pb.shear[i] * w;
    }

    const GfMatrix4d m = RigExecParamsToMatrix(pr);
    return RigExecMatrixToPoints(restPoints, m);
}

namespace {

// Swing/twist decomposition of q about unit axis: q = swing * twist,
// twist a rotation about the axis.
void
_SwingTwist(const GfQuatd &q, const GfVec3d &axis,
            GfQuatd *swing, GfQuatd *twist)
{
    const GfVec3d im = q.GetImaginary();
    const double proj = GfDot(im, axis);
    GfQuatd t(q.GetReal(), axis * proj);
    const double len = std::sqrt(
        t.GetReal() * t.GetReal() +
        GfDot(t.GetImaginary(), t.GetImaginary()));
    if (len < 1e-15) {
        *twist = GfQuatd::GetIdentity();
    } else {
        *twist = t * (1.0 / len);
    }
    *swing = q * twist->GetInverse();
}

}  // namespace

std::vector<RigExecPointFrame>
RigExecDistributeTwist(
    const RigExecPointFrame &start,
    const RigExecPointFrame &end,
    const std::array<GfVec3d, 4> &startRest,
    const std::array<GfVec3d, 4> &endRest,
    const std::vector<double> &weights)
{
    std::vector<RigExecPointFrame> result;
    result.reserve(weights.size());

    RigExecTransformParams ps, pe;
    const bool okS =
        RigExecPointsToParams(startRest, start.points, RigExecAxis::Z, &ps);
    const bool okE =
        RigExecPointsToParams(endRest, end.points, RigExecAxis::Z, &pe);
    if (!okS || !okE) {
        for (double w : weights) {
            RigExecPointFrame f = (w < 0.5) ? start : end;
            f.flags |= RigExecPointFrameDegenerate;
            result.push_back(f);
        }
        return result;
    }

    // Relative rotation decomposed into swing and unwrapped aim-axis twist.
    const GfVec3d aim =
        (start.X() - start.Origin()).GetNormalized();
    GfQuatd qs = ps.rotation, qe = pe.rotation;
    if (GfDot(qs.GetImaginary(), qe.GetImaginary()) +
            qs.GetReal() * qe.GetReal() < 0) {
        qe = -qe;
    }
    const GfQuatd rel = qs.GetInverse() * qe;
    // Axis in start-local space (column convention: rotate world aim by
    // qs^-1).
    const GfQuatd qsInv = qs.GetInverse();
    const GfVec3d localAim = qsInv.Transform(aim);
    GfQuatd swing, twist;
    _SwingTwist(rel, localAim, &swing, &twist);

    // Unwrapped twist angle about localAim.
    double twistAngle = 2.0 * std::atan2(
        GfDot(twist.GetImaginary(), localAim), twist.GetReal());

    for (size_t k = 0; k < weights.size(); ++k) {
        const double w = std::min(std::max(weights[k], 0.0), 1.0);

        const GfQuatd swingK =
            GfSlerp(w, GfQuatd::GetIdentity(), swing).GetNormalized();
        const double angK = twistAngle * w;
        const GfQuatd twistK(
            std::cos(angK / 2), localAim * std::sin(angK / 2));
        const GfQuatd qk = (qs * swingK * twistK).GetNormalized();

        RigExecTransformParams pk;
        pk.translation =
            ps.translation * (1 - w) + pe.translation * w;
        pk.rotation = qk;
        pk.scale = ps.scale * (1 - w) + pe.scale * w;
        pk.shear = ps.shear * (1 - w) + pe.shear * w;

        // Sample origin interpolates between the frame origins; the
        // decomposed translations already carry that motion for identical
        // rest sets, but samples use start's rest landmarks.
        const GfMatrix4d m = RigExecParamsToMatrix(pk);
        RigExecPointFrame f = RigExecMatrixToPoints(startRest, m);
        const GfVec3d origin =
            start.Origin() * (1 - w) + end.Origin() * w;
        const GfVec3d shift = origin - f.Origin();
        for (auto &p : f.points) {
            p += shift;
        }
        result.push_back(f);
    }
    return result;
}

GfVec3d
RigExecApplyWeightedMatrix(
    const GfVec3d &point, const GfMatrix4d &transform, double weight)
{
    if (weight <= 0.0) {
        return point;
    }
    const GfVec3d moved = transform.TransformAffine(point);
    if (weight >= 1.0) {
        return moved;
    }
    return point + (moved - point) * weight;
}

RigExecPointFrame
RigExecApplyAimConstraint(
    const RigExecPointFrame &input, const GfVec3d &targetOrigin,
    double weight, int aimLandmarkIndex)
{
    if (!std::isfinite(weight)) {
        RigExecPointFrame flagged = input;
        flagged.flags |= RigExecPointFrameDegenerate;
        return flagged;
    }
    weight = std::min(std::max(weight, 0.0), 1.0);

    const int aimIdx =
        std::min(std::max(aimLandmarkIndex, 1), 3);
    // Even-permutation role assignment keeps side = aim x up for a
    // right-handed input.
    const int upIdx = (aimIdx % 3) + 1;
    const int sideIdx = (upIdx % 3) + 1;

    const GfVec3d origin = input.Origin();
    const GfVec3d currentAim = input.points[aimIdx] - origin;
    const double aimLen = currentAim.GetLength();
    const GfVec3d toTarget = targetOrigin - origin;
    if (aimLen < 1e-12 || toTarget.GetLength() < 1e-12 || weight <= 0.0) {
        return input;
    }
    const GfVec3d a0 = currentAim / aimLen;
    const GfVec3d a1 = toTarget.GetNormalized();

    // Input handedness is preserved through the rebuild so a reflected
    // frame stays reflected.
    const GfVec3d inUp = input.points[upIdx] - origin;
    const GfVec3d inSide = input.points[sideIdx] - origin;
    const double handedness =
        GfDot(GfCross(currentAim, inUp), inSide) < 0 ? -1.0 : 1.0;

    const GfRotation full(a0, a1);
    const GfRotation partial(full.GetAxis(), full.GetAngle() * weight);
    const GfVec3d newAim = partial.TransformDir(a0).GetNormalized();

    const double upLen = inUp.GetLength();
    const double sideLen = inSide.GetLength();
    GfVec3d up = inUp;
    up -= newAim * GfDot(newAim, up);
    if (up.GetLength() < 1e-12) {
        up = partial.TransformDir(inUp);
        up -= newAim * GfDot(newAim, up);
    }
    if (up.GetLength() < 1e-12) {
        RigExecPointFrame flagged = input;
        flagged.flags |= RigExecPointFrameDegenerate;
        return flagged;
    }
    up.Normalize();
    const GfVec3d side = handedness * GfCross(newAim, up).GetNormalized();

    RigExecPointFrame out = input;
    out.points[aimIdx] = origin + newAim * aimLen;
    out.points[upIdx] = origin + up * upLen;
    out.points[sideIdx] = origin + side * sideLen;
    return out;
}

}  // namespace rigExec
