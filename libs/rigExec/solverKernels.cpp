//
// RigExec solver glue shared by exec and the bake (spec §7.5).
// See solverKernels.h.
//
#include "solverKernels.h"

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
RigExecPointFrameArray
RigExecSampleRibbonFrames(const std::vector<GfVec3f> &posed,
                          const std::vector<GfVec3f> &rest,
                          int sampleCount)
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
        result.frames.push_back(frame);
        result.rests.push_back({
            GfVec3d(restSamples.positions[k]),
            GfVec3d(restSamples.positions[k] + restSamples.tangents[k]),
            GfVec3d(restSamples.positions[k] + restSamples.normals[k]),
            GfVec3d(restSamples.positions[k] + restSamples.binormals[k])});
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
                              double twistTurns)
{
    RigExecPointFrameArray result;
    result.frames = RigExecDistributeTwist(start, end, startRest, endRest,
        weights, twistTurns);
    result.rests.assign(result.frames.size(), startRest);
    return result;
}

}  // namespace rigExec
