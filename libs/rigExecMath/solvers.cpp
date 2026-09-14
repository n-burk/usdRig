//
// RigExec solver kernels.
//
#include "solvers.h"
#include "envelope.h"

#include "pxr/base/gf/matrix3d.h"
#include "pxr/base/gf/rotation.h"

#include <algorithm>
#include <cmath>
#include <string>

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
    const std::vector<double> &weights,
    double twistTurns)
{
    std::vector<RigExecPointFrame> result;
    if (!std::isfinite(twistTurns)) return result;
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
    twistAngle += 2.0 * std::acos(-1.0) * twistTurns;
    if (!std::isfinite(twistAngle)) return result;

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

bool
RigExecSkinLayout::Validate(std::string *error) const
{
    auto fail = [error](const std::string &why) {
        if (error) {
            *error = why;
        }
        return false;
    };
    if (elementSize < 1) {
        return fail("elementSize must be at least 1");
    }
    if (!transforms || transformCount == 0) {
        return fail("no influences");
    }
    if (indexCount != pointCount * elementSize) {
        return fail("jointIndices/jointWeights length " +
                    std::to_string(indexCount) + " must equal " +
                    std::to_string(pointCount) + " points * elementSize " +
                    std::to_string(elementSize));
    }
    if (indexCount && (!indices || !weights)) {
        return fail("jointIndices and jointWeights are both required");
    }
    for (size_t t = 0; t < transformCount; ++t) {
        const GfMatrix4d &m = transforms[t];
        for (int r = 0; r < 4; ++r) {
            for (int c = 0; c < 4; ++c) {
                if (!std::isfinite(m[r][c])) {
                    return fail("influence " + std::to_string(t) +
                                " has a non-finite matrix");
                }
            }
        }
        if (m[0][3] != 0 || m[1][3] != 0 || m[2][3] != 0 || m[3][3] != 1) {
            return fail("influence " + std::to_string(t) +
                        " has a non-affine matrix");
        }
    }
    for (size_t i = 0; i < indexCount; ++i) {
        if (indices[i] < 0 || size_t(indices[i]) >= transformCount) {
            return fail("jointIndices[" + std::to_string(i) + "] = " +
                        std::to_string(indices[i]) + " is outside the " +
                        std::to_string(transformCount) + " influences");
        }
        if (!std::isfinite(weights[i]) || weights[i] < 0.0f) {
            return fail("jointWeights[" + std::to_string(i) +
                        "] must be finite and non-negative");
        }
    }
    return true;
}

GfVec3d
RigExecApplyLinearBlendSkin(
    const GfVec3d &point, const RigExecSkinLayout &layout, size_t i)
{
    GfVec3d sum(0.0);
    double total = 0.0;
    for (size_t k = 0; k < layout.elementSize; ++k) {
        const double w = layout.Weight(i, k);
        if (w == 0.0) {
            continue;
        }
        sum += layout.Transform(i, k).TransformAffine(point) * w;
        total += w;
    }
    // The complement stays with the rest point; exact when total == 1.
    return point * (1.0 - total) + sum;
}

void
RigExecApplyLinearBlendSkin(
    const GfVec3f *in, GfVec3f *out, const RigExecSkinLayout &layout)
{
    for (size_t i = 0; i < layout.pointCount; ++i) {
        out[i] = GfVec3f(RigExecApplyLinearBlendSkin(
            GfVec3d(in[i]), layout, i));
    }
}

namespace {

// The per-point gather of the dual-quaternion kernel (see the header):
// the largest-weight slot first so it becomes the blend's sign-correction
// reference, the other non-zero slots in authored order, then the identity
// entry palette[transformCount] with the weight complement when that is
// not exactly zero. The weight total is accumulated in double in slot
// order, exactly as RigExecApplyLinearBlendSkin accumulates it, so the two
// kernels see the same complement bit for bit.
void
_GatherDualQuatInfluences(
    const RigExecSkinLayout &layout, size_t i,
    std::vector<int> *indices, std::vector<double> *weights)
{
    indices->clear();
    weights->clear();
    size_t pivot = 0;
    float pivotWeight = -1.0f;
    double total = 0.0;
    for (size_t k = 0; k < layout.elementSize; ++k) {
        const float w = layout.Weight(i, k);
        if (pivotWeight < w) {
            pivotWeight = w;
            pivot = k;
        }
        total += w;
    }
    auto push = [&](size_t k) {
        const double w = layout.Weight(i, k);
        if (w != 0.0) {
            indices->push_back(layout.indices[i * layout.elementSize + k]);
            weights->push_back(w);
        }
    };
    push(pivot);
    for (size_t k = 0; k < layout.elementSize; ++k) {
        if (k != pivot) {
            push(k);
        }
    }
    const double complement = 1.0 - total;
    if (complement != 0.0) {
        indices->push_back(static_cast<int>(layout.transformCount));
        weights->push_back(complement);
    }
}

}  // namespace

std::vector<RigExecScaledDualQuat>
RigExecSkinDualQuatPalette(const RigExecSkinLayout &layout)
{
    std::vector<RigExecScaledDualQuat> palette;
    palette.reserve(layout.transformCount + 1);
    for (size_t t = 0; t < layout.transformCount; ++t) {
        palette.push_back(RigExecScaledDualQuatFromMatrix(layout.transforms[t]));
    }
    palette.emplace_back();  // identity: the weight complement's influence
    return palette;
}

bool
RigExecApplyDualQuatSkin(
    const GfVec3d &point, const RigExecScaledDualQuat *palette,
    size_t paletteSize, const RigExecSkinLayout &layout, size_t i,
    GfVec3d *out)
{
    std::vector<int> indices;
    std::vector<double> weights;
    _GatherDualQuatInfluences(layout, i, &indices, &weights);
    RigExecScaledDualQuat blend;
    if (!RigExecBlendScaledDualQuats(
            palette, paletteSize, indices.data(), weights.data(),
            indices.size(), &blend)) {
        return false;
    }
    *out = RigExecScaledDualQuatTransformPoint(blend, point);
    return true;
}

bool
RigExecApplyDualQuatSkin(
    const GfVec3f *in, GfVec3f *out, const RigExecSkinLayout &layout)
{
    const std::vector<RigExecScaledDualQuat> palette =
        RigExecSkinDualQuatPalette(layout);
    std::vector<int> indices;
    std::vector<double> weights;
    indices.reserve(layout.elementSize + 1);
    weights.reserve(layout.elementSize + 1);
    for (size_t i = 0; i < layout.pointCount; ++i) {
        _GatherDualQuatInfluences(layout, i, &indices, &weights);
        RigExecScaledDualQuat blend;
        if (!RigExecBlendScaledDualQuats(
                palette.data(), palette.size(), indices.data(),
                weights.data(), indices.size(), &blend)) {
            return false;
        }
        out[i] = GfVec3f(
            RigExecScaledDualQuatTransformPoint(blend, GfVec3d(in[i])));
    }
    return true;
}

namespace {

const std::array<GfVec3d, 4> &
_IdentityLandmarks()
{
    static const std::array<GfVec3d, 4> points = {
        GfVec3d(0, 0, 0), GfVec3d(1, 0, 0),
        GfVec3d(0, 1, 0), GfVec3d(0, 0, 1)};
    return points;
}

bool
_IsFinite(const GfVec3d &v)
{
    return std::isfinite(v[0]) && std::isfinite(v[1]) &&
           std::isfinite(v[2]);
}

bool
_HasFinitePoints(const RigExecPointFrame &frame)
{
    for (const GfVec3d &point : frame.points) {
        if (!_IsFinite(point)) {
            return false;
        }
    }
    return true;
}

bool
_IsUsableFrame(const RigExecPointFrame &frame)
{
    return frame.IsValid() && !frame.IsDegenerate() &&
           _HasFinitePoints(frame);
}

RigExecPointFrame
_ConstraintFailure(const RigExecPointFrame &input)
{
    RigExecPointFrame failed = input;
    failed.flags |= RigExecPointFrameDegenerate;
    return failed;
}

bool
_IsValidOrder(RigExecEulerOrder order)
{
    switch (order) {
    case RigExecEulerOrder::XYZ:
    case RigExecEulerOrder::XZY:
    case RigExecEulerOrder::YXZ:
    case RigExecEulerOrder::YZX:
    case RigExecEulerOrder::ZXY:
    case RigExecEulerOrder::ZYX:
        return true;
    }
    return false;
}

bool
_NormalizeDirection(GfVec3d *direction, double eps = 1e-12)
{
    if (!_IsFinite(*direction)) {
        return false;
    }
    const double largest = std::max(
        {std::abs((*direction)[0]), std::abs((*direction)[1]),
         std::abs((*direction)[2])});
    if (largest < eps) {
        return false;
    }
    *direction /= largest;
    const double length = direction->GetLength();
    if (!std::isfinite(length) || length < eps) {
        return false;
    }
    *direction /= length;
    return _IsFinite(*direction);
}

std::array<int, 3>
_OrderIndices(RigExecEulerOrder order)
{
    switch (order) {
    case RigExecEulerOrder::XYZ: return {0, 1, 2};
    case RigExecEulerOrder::XZY: return {0, 2, 1};
    case RigExecEulerOrder::YXZ: return {1, 0, 2};
    case RigExecEulerOrder::YZX: return {1, 2, 0};
    case RigExecEulerOrder::ZXY: return {2, 0, 1};
    case RigExecEulerOrder::ZYX: return {2, 1, 0};
    }
    return {0, 1, 2};
}

GfQuatd
_QuatFromEulerDegrees(const GfVec3d &degrees, RigExecEulerOrder order)
{
    static const GfVec3d axes[3] = {
        GfVec3d(1, 0, 0), GfVec3d(0, 1, 0), GfVec3d(0, 0, 1)};
    GfMatrix4d matrix(1.0);
    const std::array<int, 3> indices = _OrderIndices(order);
    for (const int axis : indices) {
        matrix = matrix * GfMatrix4d(
            GfRotation(axes[axis], degrees[axis]), GfVec3d(0));
    }
    return matrix.ExtractRotation().GetQuat().GetNormalized();
}

GfVec3d
_EulerDegreesFromQuat(const GfQuatd &rotation, RigExecEulerOrder order)
{
    static const GfVec3d axes[3] = {
        GfVec3d(1, 0, 0), GfVec3d(0, 1, 0), GfVec3d(0, 0, 1)};
    const std::array<int, 3> indices = _OrderIndices(order);
    // GfRotation::Decompose(a,b,c) describes row-matrix factors in the
    // reverse order Rc * Rb * Ra.  Pass the authored application sequence
    // reversed so returned components invert _QuatFromEulerDegrees exactly.
    const GfVec3d ordered = GfRotation(rotation).Decompose(
        axes[indices[2]], axes[indices[1]], axes[indices[0]]);
    GfVec3d result(0);
    result[indices[2]] = ordered[0];
    result[indices[1]] = ordered[1];
    result[indices[0]] = ordered[2];
    return result;
}

double
_ShortestDegrees(double degrees)
{
    double wrapped = std::fmod(degrees + 180.0, 360.0);
    if (wrapped < 0.0) {
        wrapped += 360.0;
    }
    wrapped -= 180.0;
    // Resolve the exact half-turn tie without depending on fmod's sign.
    return (wrapped == -180.0 && degrees > 0.0) ? 180.0 : wrapped;
}

bool
_Affects(const RigExecConstraintAxisMask &mask, int axis)
{
    return axis == 0 ? mask.x : axis == 1 ? mask.y : mask.z;
}

bool
_DecomposeConstraintFrame(
    const RigExecPointFrame &frame, RigExecTransformParams *params)
{
    if (!_IsUsableFrame(frame) ||
        !RigExecPointsToParams(
            _IdentityLandmarks(), frame.points, RigExecAxis::Z, params)) {
        return false;
    }
    return _IsFinite(params->translation) && _IsFinite(params->scale) &&
           _IsFinite(params->shear) &&
           std::isfinite(params->rotation.GetReal()) &&
           _IsFinite(params->rotation.GetImaginary());
}

RigExecPointFrame
_FrameFromConstraintParams(
    const RigExecPointFrame &input, const RigExecTransformParams &params)
{
    RigExecPointFrame result = RigExecMatrixToPoints(
        _IdentityLandmarks(), RigExecParamsToMatrix(params));
    if (!_HasFinitePoints(result)) {
        return _ConstraintFailure(input);
    }
    // Affine is descriptive rather than required for reconstruction, but an
    // input explicitly classified affine remains classified affine when its
    // shear is carried through the constraint.
    result.flags |= input.flags & RigExecPointFrameAffine;
    return result;
}

bool
_ValidateGlobalWeight(double weight, double *clamped)
{
    if (!std::isfinite(weight)) {
        return false;
    }
    *clamped = std::min(std::max(weight, 0.0), 1.0);
    return true;
}

bool
_ValidateSourceWeight(const RigExecConstraintSource &source)
{
    return std::isfinite(source.normalizedWeight) &&
           source.normalizedWeight >= 0.0;
}

GfVec3d
_ApplyEulerDelta(
    const GfVec3d &inputEuler, const GfVec3d &targetEuler,
    const RigExecConstraintAxisMask &affect, double weight)
{
    GfVec3d output = inputEuler;
    for (int axis = 0; axis < 3; ++axis) {
        if (_Affects(affect, axis)) {
            // The target Euler representation is the full-strength
            // candidate.  Select it directly at the endpoint: reconstructing
            // the equivalent input + shortestDelta representation can differ
            // bit-for-bit (for example, 190 versus -170 degrees).
            if (weight >= 1.0) {
                output[axis] = targetEuler[axis];
            } else {
                output[axis] += weight * _ShortestDegrees(
                    targetEuler[axis] - inputEuler[axis]);
            }
        }
    }
    return output;
}

}  // namespace

RigExecPointFrame
RigExecApplyPositionConstraint(
    const RigExecPointFrame &input,
    const std::vector<RigExecConstraintSource> &sources,
    const RigExecPositionConstraintParams &params)
{
    double globalWeight = 0.0;
    if (!_ValidateGlobalWeight(params.weight, &globalWeight)) {
        return _ConstraintFailure(input);
    }
    if (globalWeight <= 0.0) {
        return input;
    }
    if (!_IsUsableFrame(input) || !_IsFinite(params.offset)) {
        return _ConstraintFailure(input);
    }

    GfVec3d target(0);
    double totalWeight = 0.0;
    for (const RigExecConstraintSource &source : sources) {
        if (!_ValidateSourceWeight(source)) {
            return _ConstraintFailure(input);
        }
        if (source.normalizedWeight == 0.0) {
            continue;
        }
        if (!_IsUsableFrame(source.frame)) {
            return _ConstraintFailure(input);
        }
        target += source.frame.Origin() * source.normalizedWeight;
        totalWeight += source.normalizedWeight;
    }
    if (totalWeight <= 0.0 || !std::isfinite(totalWeight)) {
        return totalWeight == 0.0 ? input : _ConstraintFailure(input);
    }
    target = target / totalWeight + params.offset;
    if (!_IsFinite(target)) {
        return _ConstraintFailure(input);
    }

    GfVec3d constrainedOrigin = input.Origin();
    for (int axis = 0; axis < 3; ++axis) {
        if (_Affects(params.affect, axis)) {
            constrainedOrigin[axis] = RigExecBlendEnvelope(
                input.Origin()[axis], target[axis], globalWeight);
        }
    }
    if (!_IsFinite(constrainedOrigin)) {
        return _ConstraintFailure(input);
    }
    RigExecPointFrame output = input;
    if (globalWeight >= 1.0) {
        // At full strength the candidate pose owns its origin exactly.  Build
        // its other landmarks from the preceding linear part instead of
        // translating through another cancellation-prone a + (b - a).
        for (size_t point = 1; point < output.points.size(); ++point) {
            output.points[point] = constrainedOrigin +
                (input.points[point] - input.Origin());
        }
        output.points[0] = constrainedOrigin;
    } else {
        const GfVec3d translation = constrainedOrigin - input.Origin();
        for (GfVec3d &point : output.points) {
            point += translation;
        }
    }
    return output;
}

RigExecPointFrame
RigExecApplyRotationConstraint(
    const RigExecPointFrame &input,
    const std::vector<RigExecConstraintSource> &sources,
    const RigExecRotationConstraintParams &params)
{
    double globalWeight = 0.0;
    if (!_ValidateGlobalWeight(params.weight, &globalWeight)) {
        return _ConstraintFailure(input);
    }
    if (globalWeight <= 0.0) {
        return input;
    }
    if (!_IsValidOrder(params.rotationOrder) ||
        !_IsFinite(params.offsetDegrees)) {
        return _ConstraintFailure(input);
    }

    RigExecTransformParams inputParams;
    if (!_DecomposeConstraintFrame(input, &inputParams)) {
        return _ConstraintFailure(input);
    }
    const GfVec3d inputEuler =
        _EulerDegreesFromQuat(inputParams.rotation, params.rotationOrder);
    if (!_IsFinite(inputEuler)) {
        return _ConstraintFailure(input);
    }
    GfVec3d sourceAnchor(0), weightedDelta(0);
    bool hasSourceAnchor = false;
    double totalWeight = 0.0;
    for (const RigExecConstraintSource &source : sources) {
        if (!_ValidateSourceWeight(source)) {
            return _ConstraintFailure(input);
        }
        if (source.normalizedWeight == 0.0) {
            continue;
        }
        RigExecTransformParams sourceParams;
        if (!_DecomposeConstraintFrame(source.frame, &sourceParams)) {
            return _ConstraintFailure(input);
        }
        const GfVec3d sourceEuler =
            _EulerDegreesFromQuat(sourceParams.rotation, params.rotationOrder);
        if (!_IsFinite(sourceEuler)) {
            return _ConstraintFailure(input);
        }
        if (!hasSourceAnchor) {
            sourceAnchor = sourceEuler;
            hasSourceAnchor = true;
        }
        for (int axis = 0; axis < 3; ++axis) {
            weightedDelta[axis] += source.normalizedWeight *
                _ShortestDegrees(sourceEuler[axis] - sourceAnchor[axis]);
        }
        totalWeight += source.normalizedWeight;
    }
    if (totalWeight <= 0.0 || !std::isfinite(totalWeight)) {
        return totalWeight == 0.0 ? input : _ConstraintFailure(input);
    }
    const GfVec3d targetEuler =
        sourceAnchor + weightedDelta / totalWeight + params.offsetDegrees;
    if (!_IsFinite(weightedDelta) || !_IsFinite(targetEuler)) {
        return _ConstraintFailure(input);
    }
    const GfVec3d outputEuler = _ApplyEulerDelta(
        inputEuler, targetEuler, params.affect, globalWeight);
    inputParams.rotation =
        _QuatFromEulerDegrees(outputEuler, params.rotationOrder);
    return _FrameFromConstraintParams(input, inputParams);
}

RigExecPointFrame
RigExecApplyScaleConstraint(
    const RigExecPointFrame &input,
    const std::vector<RigExecConstraintSource> &sources,
    const RigExecScaleConstraintParams &params)
{
    double globalWeight = 0.0;
    if (!_ValidateGlobalWeight(params.weight, &globalWeight)) {
        return _ConstraintFailure(input);
    }
    if (globalWeight <= 0.0) {
        return input;
    }
    if (!_IsFinite(params.offset)) {
        return _ConstraintFailure(input);
    }

    RigExecTransformParams inputParams;
    if (!_DecomposeConstraintFrame(input, &inputParams)) {
        return _ConstraintFailure(input);
    }
    GfVec3d targetScale(0);
    double totalWeight = 0.0;
    for (const RigExecConstraintSource &source : sources) {
        if (!_ValidateSourceWeight(source)) {
            return _ConstraintFailure(input);
        }
        if (source.normalizedWeight == 0.0) {
            continue;
        }
        RigExecTransformParams sourceParams;
        if (!_DecomposeConstraintFrame(source.frame, &sourceParams)) {
            return _ConstraintFailure(input);
        }
        targetScale += sourceParams.scale * source.normalizedWeight;
        totalWeight += source.normalizedWeight;
    }
    if (totalWeight <= 0.0 || !std::isfinite(totalWeight)) {
        return totalWeight == 0.0 ? input : _ConstraintFailure(input);
    }
    targetScale = targetScale / totalWeight + params.offset;
    if (!_IsFinite(targetScale)) {
        return _ConstraintFailure(input);
    }
    for (int axis = 0; axis < 3; ++axis) {
        if (_Affects(params.affect, axis)) {
            inputParams.scale[axis] = RigExecBlendEnvelope(
                inputParams.scale[axis], targetScale[axis], globalWeight);
        }
    }
    return _FrameFromConstraintParams(input, inputParams);
}

RigExecPointFrame
RigExecApplyParentConstraint(
    const RigExecPointFrame &input,
    const std::vector<RigExecConstraintSource> &sources,
    const RigExecParentConstraintParams &params)
{
    double globalWeight = 0.0;
    if (!_ValidateGlobalWeight(params.weight, &globalWeight)) {
        return _ConstraintFailure(input);
    }
    if (globalWeight <= 0.0) {
        return input;
    }
    if (!_IsValidOrder(params.rotationOrder)) {
        return _ConstraintFailure(input);
    }

    RigExecTransformParams inputParams;
    if (!_DecomposeConstraintFrame(input, &inputParams)) {
        return _ConstraintFailure(input);
    }
    const GfVec3d inputEuler =
        _EulerDegreesFromQuat(inputParams.rotation, params.rotationOrder);
    if (!_IsFinite(inputEuler)) {
        return _ConstraintFailure(input);
    }
    GfVec3d targetTranslation(0), targetScale(0);
    GfVec3d sourceAnchor(0), weightedRotationDelta(0);
    bool hasSourceAnchor = false;
    double totalWeight = 0.0;
    for (const RigExecConstraintSource &source : sources) {
        if (!_ValidateSourceWeight(source)) {
            return _ConstraintFailure(input);
        }
        if (source.normalizedWeight == 0.0) {
            continue;
        }
        if (!_IsFinite(source.translationOffset) ||
            !_IsFinite(source.rotationOffsetDegrees)) {
            return _ConstraintFailure(input);
        }
        if (!_IsUsableFrame(source.frame)) {
            return _ConstraintFailure(input);
        }
        GfMatrix4d sourceMatrix(1.0);
        if (!RigExecPointsToMatrix(
                _IdentityLandmarks(), source.frame, &sourceMatrix)) {
            return _ConstraintFailure(input);
        }

        // FBX Parent offsets describe the constrained object's local
        // transform relative to each source. In row-vector convention the
        // local offset is therefore applied before the source transform.
        RigExecTransformParams offsetParams;
        offsetParams.translation = source.translationOffset;
        offsetParams.rotation = _QuatFromEulerDegrees(
            source.rotationOffsetDegrees, params.rotationOrder);
        const GfMatrix4d targetMatrix =
            RigExecParamsToMatrix(offsetParams) * sourceMatrix;
        const RigExecPointFrame targetFrame = RigExecMatrixToPoints(
            _IdentityLandmarks(), targetMatrix);
        RigExecTransformParams targetParams;
        if (!_DecomposeConstraintFrame(targetFrame, &targetParams)) {
            return _ConstraintFailure(input);
        }
        targetTranslation +=
            targetParams.translation * source.normalizedWeight;
        targetScale += targetParams.scale * source.normalizedWeight;

        const GfVec3d sourceEuler =
            _EulerDegreesFromQuat(targetParams.rotation, params.rotationOrder);
        if (!_IsFinite(sourceEuler)) {
            return _ConstraintFailure(input);
        }
        if (!hasSourceAnchor) {
            sourceAnchor = sourceEuler;
            hasSourceAnchor = true;
        }
        for (int axis = 0; axis < 3; ++axis) {
            weightedRotationDelta[axis] += source.normalizedWeight *
                _ShortestDegrees(sourceEuler[axis] - sourceAnchor[axis]);
        }
        totalWeight += source.normalizedWeight;
    }
    if (totalWeight <= 0.0 || !std::isfinite(totalWeight)) {
        return totalWeight == 0.0 ? input : _ConstraintFailure(input);
    }
    targetTranslation /= totalWeight;
    targetScale /= totalWeight;
    const GfVec3d targetEuler =
        sourceAnchor + weightedRotationDelta / totalWeight;
    if (!_IsFinite(targetTranslation) || !_IsFinite(targetScale) ||
        !_IsFinite(weightedRotationDelta) || !_IsFinite(targetEuler)) {
        return _ConstraintFailure(input);
    }

    for (int axis = 0; axis < 3; ++axis) {
        if (_Affects(params.translationAxes, axis)) {
            inputParams.translation[axis] = RigExecBlendEnvelope(
                inputParams.translation[axis], targetTranslation[axis],
                globalWeight);
        }
        if (_Affects(params.scaleAxes, axis)) {
            inputParams.scale[axis] = RigExecBlendEnvelope(
                inputParams.scale[axis], targetScale[axis], globalWeight);
        }
    }
    const GfVec3d outputEuler = _ApplyEulerDelta(
        inputEuler, targetEuler, params.rotationAxes, globalWeight);
    inputParams.rotation =
        _QuatFromEulerDegrees(outputEuler, params.rotationOrder);
    return _FrameFromConstraintParams(input, inputParams);
}

RigExecPointFrame
RigExecApplyAimConstraint(
    const RigExecPointFrame &input, const GfVec3d &targetPoint,
    const RigExecAimConstraintParams &params)
{
    double globalWeight = 0.0;
    if (!_ValidateGlobalWeight(params.weight, &globalWeight)) {
        return _ConstraintFailure(input);
    }
    if (globalWeight <= 0.0) {
        return input;
    }
    const bool needsRollCorrection =
        params.worldUpDirection.has_value() || params.preserveInputUp;
    if (!_IsValidOrder(params.rotationOrder) ||
        !_IsFinite(targetPoint) || !_IsFinite(params.localAimVector) ||
        (needsRollCorrection && !_IsFinite(params.localUpVector)) ||
        !_IsFinite(params.rotationOffsetDegrees) ||
        (params.worldUpDirection &&
         !_IsFinite(*params.worldUpDirection))) {
        return _ConstraintFailure(input);
    }

    RigExecTransformParams inputParams;
    if (!_DecomposeConstraintFrame(input, &inputParams)) {
        return _ConstraintFailure(input);
    }
    GfVec3d localAim = params.localAimVector;
    if (!_NormalizeDirection(&localAim)) {
        return _ConstraintFailure(input);
    }
    GfVec3d targetAim = targetPoint - inputParams.translation;
    if (!_NormalizeDirection(&targetAim)) {
        return _ConstraintFailure(input);
    }

    const GfQuatd inputRotation = inputParams.rotation.GetNormalized();
    GfVec3d currentAim = inputRotation.Transform(localAim);
    if (!_NormalizeDirection(&currentAim)) {
        return _ConstraintFailure(input);
    }
    const GfRotation swing(currentAim, targetAim);
    GfRotation twist(GfVec3d(1, 0, 0), 0.0);
    if (needsRollCorrection) {
        GfVec3d localUp = params.localUpVector;
        if (!_NormalizeDirection(&localUp)) {
            return _ConstraintFailure(input);
        }
        localUp -= localAim * GfDot(localAim, localUp);
        if (!_NormalizeDirection(&localUp)) {
            return _ConstraintFailure(input);
        }

        GfVec3d currentUp = inputRotation.Transform(localUp);
        if (!_NormalizeDirection(&currentUp)) {
            return _ConstraintFailure(input);
        }
        GfVec3d swungUp = swing.TransformDir(currentUp);
        swungUp -= targetAim * GfDot(targetAim, swungUp);
        if (!_NormalizeDirection(&swungUp)) {
            return _ConstraintFailure(input);
        }

        GfVec3d desiredUp;
        if (params.worldUpDirection) {
            desiredUp = *params.worldUpDirection;
            if (!_NormalizeDirection(&desiredUp)) {
                return _ConstraintFailure(input);
            }
            desiredUp -= targetAim * GfDot(targetAim, desiredUp);
            if (!_NormalizeDirection(&desiredUp)) {
                return _ConstraintFailure(input);
            }
        } else {
            // Legacy preserveInputUp: preserve the previous world up
            // direction rather than FBX worldUpType=None's minimum swing.
            desiredUp = currentUp - targetAim * GfDot(targetAim, currentUp);
            if (!_NormalizeDirection(&desiredUp)) {
                desiredUp = swungUp;
            }
        }
        twist = GfRotation::RotateOntoProjected(
            swungUp, desiredUp, targetAim);
    }

    GfMatrix4d aimedMatrix(1.0);
    aimedMatrix.SetRotate(inputRotation);
    GfMatrix4d swingMatrix(1.0), twistMatrix(1.0);
    swingMatrix.SetRotate(swing);
    twistMatrix.SetRotate(twist);
    aimedMatrix = aimedMatrix * swingMatrix * twistMatrix;
    const GfQuatd aimedRotation =
        aimedMatrix.ExtractRotation().GetQuat().GetNormalized();

    const GfVec3d inputEuler =
        _EulerDegreesFromQuat(inputRotation, params.rotationOrder);
    const GfVec3d targetEuler =
        _EulerDegreesFromQuat(aimedRotation, params.rotationOrder) +
        params.rotationOffsetDegrees;
    if (!_IsFinite(inputEuler) || !_IsFinite(targetEuler)) {
        return _ConstraintFailure(input);
    }
    const GfVec3d outputEuler = _ApplyEulerDelta(
        inputEuler, targetEuler, params.affectRotation, globalWeight);
    inputParams.rotation =
        _QuatFromEulerDegrees(outputEuler, params.rotationOrder);
    return _FrameFromConstraintParams(input, inputParams);
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
    const GfVec3d newAim = weight >= 1.0
        ? a1
        : partial.TransformDir(a0).GetNormalized();

    const double upLen = inUp.GetLength();
    const double sideLen = inSide.GetLength();
    GfVec3d up = inUp;
    up -= newAim * GfDot(newAim, up);
    if (up.GetLength() < 1e-12) {
        up = weight >= 1.0
            ? full.TransformDir(inUp)
            : partial.TransformDir(inUp);
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
