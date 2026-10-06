// RigExec solver glue shared by exec and the bake (spec §7.5).
// See solverKernels.h.
#include "solverKernels.h"
#include "frameExtraction.h"

#include "rigExecMath/geometryKernels.h"
#include "rigExecMath/solvers.h"

#include "pxr/base/gf/math.h"
#include "pxr/base/gf/quatd.h"
#include "pxr/base/gf/rotation.h"
#include "pxr/base/gf/transform.h"

#include <algorithm>
#include <cmath>
#include <vector>

PXR_NAMESPACE_USING_DIRECTIVE

namespace rigExec {

// The ribbon sampler, shared by the RigExecRibbon exec callback and by the
// baked program, which samples the same two curves with no VdfNetwork around
// it. One definition, so a second caller cannot drift into publishing frames
// where this one publishes nothing.

namespace {

// Re-bases one aggregate element onto a joint's rest reference.
// The element's own rest->pose map is measured from \p ownRest and applied to
// \p jointRest instead, so the frame a pose step BELOW this solver left is
// carried through the solve rather than replaced (spec 4.2). Used only where
// such a step really wrote the joint, which is what keeps every other rig
// bit-identical.
bool
_RebaseElement(const std::array<GfVec3d, 4> &ownRest,
               const std::array<GfVec3d, 4> &jointRest,
               RigExecPointFrame *frame)
{
    GfMatrix4d map;
    if (!RigExecPointsToMatrix(ownRest, frame->points, &map)) {
        return false;
    }
    const uint32_t flags = frame->flags;
    *frame = RigExecMatrixToPoints(jointRest, map);
    frame->flags = flags;
    return true;
}

}  // namespace

RigExecPointFrameArray
RigExecSampleRibbonFrames(const std::vector<GfVec3f> &posed,
                          const std::vector<GfVec3f> &rest,
                          int sampleCount,
                          const std::vector<std::array<GfVec3d, 4>>
                              &jointRests,
                          const std::vector<bool> &jointRestLive)
{
    RigExecPointFrameArray result;
    if (posed.empty() || rest.empty() || sampleCount < 2) {
        return result;
    }
    const RigExecCurveFrameSamples posedSamples =
        RigExecSampleCurveRMF(posed, sampleCount);
    const RigExecCurveFrameSamples restSamples =
        RigExecSampleCurveRMF(rest, sampleCount);
    if (posedSamples.GetSize() != size_t(sampleCount) ||
        restSamples.GetSize() != size_t(sampleCount)) {
        return result;
    }
    result.frames.reserve(sampleCount);
    result.rests.reserve(sampleCount);
    for (int k = 0; k < sampleCount; ++k) {
        RigExecPointFrame frame;
        frame.points = {
            GfVec3d(posedSamples.positions[k]),
            GfVec3d(posedSamples.positions[k] + posedSamples.tangents[k]),
            GfVec3d(posedSamples.positions[k] + posedSamples.normals[k]),
            GfVec3d(posedSamples.positions[k] + posedSamples.binormals[k])};
        frame.flags = RigExecPointFrameValid;
        std::array<GfVec3d, 4> restPoints = {
            GfVec3d(restSamples.positions[k]),
            GfVec3d(restSamples.positions[k] + restSamples.tangents[k]),
            GfVec3d(restSamples.positions[k] + restSamples.normals[k]),
            GfVec3d(restSamples.positions[k] + restSamples.binormals[k])};
        // A sample a pose step below the ribbon already wrote is re-based
        // onto what that step left; every other sample keeps the rest
        // curve's own frame as its basis.
        if (size_t(k) < jointRests.size() && size_t(k) < jointRestLive.size()
            && jointRestLive[size_t(k)] &&
            _RebaseElement(restPoints, jointRests[size_t(k)], &frame)) {
            restPoints = jointRests[size_t(k)];
        }
        result.frames.push_back(frame);
        result.rests.push_back(restPoints);
    }
    return result;
}

// The twist weight defaulting, shared by the RigExecTwistDistribution exec
// callback and by the baked program. The caller drains the authored floats
// into the vector first -- the attribute's type is float, the distribution
// solves in double -- and this fills it only when nothing was authored.
void
RigExecResolveTwistWeights(int count, std::vector<double> *weights)
{
    if (weights->empty()) {
        const int n = std::max(count, 1);
        for (int k = 0; k < n; ++k) {
            weights->push_back(n == 1 ? 0.0 : double(k) / (n - 1));
        }
    }
}

// The twist distribution, shared by the RigExecTwistDistribution exec
// callback and by the baked program. One definition, so the rests half --
// every frame paired with the START landmarks, which is what the aggregate's
// consumers re-solve against -- cannot be re-derived differently.
RigExecPointFrameArray
RigExecSolveTwistDistribution(const RigExecPointFrame &start,
                              const RigExecPointFrame &end,
                              const std::array<GfVec3d, 4> &startRest,
                              const std::array<GfVec3d, 4> &endRest,
                              const std::vector<double> &weights,
                              double twistTurns,
                              const std::vector<std::array<GfVec3d, 4>>
                                  &jointRests,
                              const std::vector<bool> &jointRestLive)
{
    RigExecPointFrameArray result;
    result.frames = RigExecDistributeTwist(start, end, startRest, endRest,
        weights, twistTurns);
    result.rests.assign(result.frames.size(), startRest);
    // Same rule, same helper: an element a step below this solver wrote is
    // re-based onto what that step left, measured from the START landmarks
    // every element of this aggregate is paired with.
    for (size_t k = 0; k < result.frames.size(); ++k) {
        if (k < jointRests.size() && k < jointRestLive.size() &&
            jointRestLive[k] &&
            _RebaseElement(startRest, jointRests[k], &result.frames[k])) {
            result.rests[k] = jointRests[k];
        }
    }
    return result;
}

bool
RigExecFrameRotation(const RigExecPointFrame &frame, GfQuatd *out)
{
    GfMatrix4d matrix(1.0);
    if (!frame.IsValid() || frame.IsDegenerate() ||
        !RigExecPointsToMatrix(RigExecIdentityLandmarks(), frame.points,
                               &matrix)) {
        return false;
    }
    matrix.SetTranslateOnly(GfVec3d(0.0));
    *out = matrix.GetOrthonormalized(/* issueWarning = */ false)
               .ExtractRotationQuat();
    return true;
}

namespace {

bool
_FrameMatrix(const RigExecPointFrame &frame, GfMatrix4d *out)
{
    return frame.IsValid() && !frame.IsDegenerate() &&
           RigExecPointsToMatrix(RigExecIdentityLandmarks(), frame.points,
                                 out);
}

}  // namespace

bool
RigExecFrameTranslation(const RigExecPointFrame &driverFinal,
                        const RigExecPointFrame &driverRest,
                        const RigExecPointFrame *parentFinal,
                        const RigExecPointFrame *parentRest,
                        GfVec3d *out)
{
    GfMatrix4d world(1.0), restWorld(1.0);
    if (!_FrameMatrix(driverFinal, &world) ||
        !_FrameMatrix(driverRest, &restWorld)) {
        return false;
    }
    GfMatrix4d local = world, restLocal = restWorld;
    double det = 0.0;
    if (parentFinal && parentRest) {
        GfMatrix4d parent(1.0), restParent(1.0);
        if (!_FrameMatrix(*parentFinal, &parent) ||
            !_FrameMatrix(*parentRest, &restParent)) {
            return false;
        }
        const GfMatrix4d parentInverse = parent.GetInverse(&det);
        if (det == 0.0) {
            return false;
        }
        const GfMatrix4d restParentInverse = restParent.GetInverse(&det);
        if (det == 0.0) {
            return false;
        }
        local = world * parentInverse;
        restLocal = restWorld * restParentInverse;
    }
    const GfMatrix4d restLocalInverse = restLocal.GetInverse(&det);
    if (det == 0.0) {
        return false;
    }
    *out = restLocalInverse.Transform(local.ExtractTranslation());
    return true;
}

namespace {

// Translation, rotation (as a quaternion) and scale of a transform, taken
// with GfTransform so that the decomposition and the recomposition below are
// each other's inverse by construction.
struct _Decomposed {
    GfVec3d translation = GfVec3d(0);
    GfQuatd rotation = GfQuatd::GetIdentity();
    GfVec3d scale = GfVec3d(1);
};

_Decomposed
_Decompose(const GfMatrix4d &m)
{
    const GfTransform transform(m);
    _Decomposed out;
    out.translation = transform.GetTranslation();
    out.rotation = transform.GetRotation().GetQuat();
    out.scale = transform.GetScale();
    return out;
}

GfMatrix4d
_Recompose(const _Decomposed &d)
{
    GfTransform transform;
    transform.SetScale(d.scale);
    transform.SetRotation(GfRotation(d.rotation));
    transform.SetTranslation(d.translation);
    return transform.GetMatrix();
}

}  // namespace

GfMatrix4d
RigExecBlendTransforms(const GfMatrix4d &a, const GfMatrix4d &b, double weight)
{
    // The endpoints return their operand untouched rather than a
    // decomposition of it: a space switch sitting on a whole number has to be
    // bit-identical to selecting that space, and a round trip through
    // GfTransform is not the identity on a sheared or near-degenerate matrix.
    if (!(weight > 0.0)) {
        return a;
    }
    if (weight >= 1.0) {
        return b;
    }
    const _Decomposed da = _Decompose(a), db = _Decompose(b);
    _Decomposed out;
    out.translation = GfLerp(weight, da.translation, db.translation);
    out.scale = GfLerp(weight, da.scale, db.scale);
    // Shortest arc: the two spaces are usually close, and a switch easing the
    // long way round is never what an animator asked for.
    GfQuatd target = db.rotation;
    if (GfDot(da.rotation, target) < 0.0) {
        target = GfQuatd(-target.GetReal(), -target.GetImaginary());
    }
    out.rotation = GfSlerp(weight, da.rotation, target).GetNormalized();
    return _Recompose(out);
}

GfMatrix4d
RigExecOrientSpaceDelta(const GfMatrix4d &delta, const GfMatrix4d &local,
                        const GfMatrix4d &localInverse,
                        const GfMatrix4d &unswitched)
{
    GfMatrix4d frame = delta * local;
    frame[3][0] = unswitched[3][0];
    frame[3][1] = unswitched[3][1];
    frame[3][2] = unswitched[3][2];
    return frame * localInverse;
}

GfMatrix4d
RigExecFilterSpaceRotation(const GfMatrix4d &m, const GfVec3d &axis,
                           RigExecRotationFilter filter)
{
    if (filter == RigExecRotationFilter::All ||
        filter == RigExecRotationFilter::Orient) {
        return m;
    }
    const double length = axis.GetLength();
    if (length < 1e-12) {
        return m;
    }
    const GfVec3d a = axis / length;
    _Decomposed d = _Decompose(m);
    const GfQuatd q = d.rotation.GetNormalized();
    // Swing-twist (Dobrowolski 2015, see docs/references.md): the twist is
    // the part of the rotation whose axis IS the limb axis, which is the
    // quaternion's imaginary component along it, renormalized; the swing is
    // whatever is left over.
    const double along = GfDot(q.GetImaginary(), a);
    GfQuatd twist(q.GetReal(), along * a);
    const double norm = std::sqrt(twist.GetReal() * twist.GetReal() +
                                  twist.GetImaginary().GetLengthSq());
    if (norm < 1e-12) {
        // A half turn square to the axis: the twist is genuinely undefined
        // there, and no twist is the only answer that does not jump.
        twist = GfQuatd::GetIdentity();
    } else {
        twist = GfQuatd(twist.GetReal() / norm, twist.GetImaginary() / norm);
    }
    d.rotation = filter == RigExecRotationFilter::Twist
                     ? twist
                     : (q * twist.GetInverse()).GetNormalized();
    return _Recompose(d);
}

GfMatrix4d
RigExecMaskTransform(const GfMatrix4d &m, const bool translation[3],
                     const bool rotation[3], const bool scale[3])
{
    const bool all = translation[0] && translation[1] && translation[2] &&
                     rotation[0] && rotation[1] && rotation[2] &&
                     scale[0] && scale[1] && scale[2];
    if (all) {
        return m;
    }
    _Decomposed d = _Decompose(m);
    for (int axis = 0; axis < 3; ++axis) {
        if (!translation[axis]) d.translation[axis] = 0.0;
        if (!scale[axis]) d.scale[axis] = 1.0;
    }
    if (!rotation[0] || !rotation[1] || !rotation[2]) {
        // Per-axis rotation masking is an EULER statement -- "keep the X
        // rotation, drop the Y" -- so the rotation is taken apart in the SAME
        // order the avar compose puts one together (_ComposeAvars /
        // RigExecBakedComposeAvars with an XYZ order): row-vector
        // R = Rx * Ry * Rz. Decomposing with GfRotation::Decompose instead
        // would be one convention removed from that, and a mask that agrees
        // with the compose only for small angles is worse than none.
        const GfMatrix4d r(GfRotation(d.rotation), GfVec3d(0));
        double x = 0.0, y = 0.0, z = 0.0;
        const double sinY = GfClamp(r[0][2], -1.0, 1.0);
        y = std::asin(sinY);
        if (std::abs(sinY) < 1.0 - 1e-9) {
            x = std::atan2(r[1][2], r[2][2]);
            z = std::atan2(r[0][1], r[0][0]);
        } else {
            // Gimbal lock: X and Z are the same rotation, so all of it is
            // given to X and Z is zero, which is what any Euler extraction
            // has to choose.
            x = std::atan2(-r[2][1], r[1][1]);
            z = 0.0;
        }
        static const GfVec3d axes[3] = {
            GfVec3d(1, 0, 0), GfVec3d(0, 1, 0), GfVec3d(0, 0, 1)};
        const double angles[3] = {GfRadiansToDegrees(x),
                                  GfRadiansToDegrees(y),
                                  GfRadiansToDegrees(z)};
        GfMatrix4d composed(1.0);
        for (int axis = 0; axis < 3; ++axis) {
            if (!rotation[axis] || angles[axis] == 0.0) continue;
            composed = composed *
                GfMatrix4d(GfRotation(axes[axis], angles[axis]), GfVec3d(0));
        }
        d.rotation = composed.ExtractRotationQuat();
    }
    return _Recompose(d);
}

}  // namespace rigExec
