//
// RigExec solver glue shared by exec and the bake (spec §7.5).
// See solverKernels.h.
//
#include "solverKernels.h"
#include "frameExtraction.h"

#include "rigExecMath/geometryKernels.h"
#include "rigExecMath/solvers.h"

#include <algorithm>
#include <vector>

PXR_NAMESPACE_USING_DIRECTIVE

namespace rigExec {

// The ribbon sampler, shared by the RigExecRibbon exec callback and by the
// baked program, which samples the same two curves with no VdfNetwork around
// it. One definition, so a second caller cannot drift into publishing frames
// where this one publishes nothing.

namespace {

// Re-bases one aggregate element onto a joint's rest reference.
//
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

}  // namespace rigExec
