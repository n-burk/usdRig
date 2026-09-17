//
// RigExec solver glue shared by the exec callbacks and the baked program
// (spec §7.5).
//
// The solver MATH lives in rigExecMath (geometryKernels.h, solvers.h) and is
// already shared. What is not is the glue around it: the guards that decide
// when a solver publishes nothing, the defaulting of an unauthored input, and
// the packing of sampled math results into a RigExecPointFrameArray. That glue
// used to sit inside the VdfContext callbacks in computations.cpp and
// moverKernels.cpp, where a baked program could only re-express it -- and a
// second expression of a guard is a second chance to get it wrong.
//
// These functions are the guard and the packing, with the ctx reads left
// behind in the callback. They live here rather than in rigExecMath because
// RigExecPointFrameArray is a rigExec type (types.h) and rigExecMath must not
// depend on rigExec.
//
#ifndef RIGEXEC_SOLVER_KERNELS_H
#define RIGEXEC_SOLVER_KERNELS_H

#include "types.h"

#include "rigExecMath/pointFrame.h"

#include "pxr/base/gf/vec3d.h"
#include "pxr/base/gf/vec3f.h"

#include <array>
#include <vector>

PXR_NAMESPACE_USING_DIRECTIVE

namespace rigExec {

/// Samples \p posed and \p rest driver curves at \p sampleCount arc-length
/// parameters and packs the two rotation-minimizing frame sets into one
/// aggregate: the posed samples become the published frames, the rest samples
/// their paired landmark sets (spec §7.5).
///
/// Answers an EMPTY aggregate for an unresolvable ribbon -- an empty curve on
/// either side, fewer than two samples, or a sampler that returned a different
/// count than was asked of it -- which is how an authoring error reaches the
/// consumer here: no frames rather than stale ones.
///
/// ONE definition, called by the RigExecRibbon exec callback and by the baked
/// program: the empty-aggregate guard IS the ribbon's error behavior, so two
/// hand-copied versions of it could disagree about which rigs publish, and the
/// only rigs that would show it are the malformed ones no fixture has.
/// \p jointRests, when non-empty and one per sample, re-bases each sample:
/// the rest->pose map the curve gives is applied to the joint's rest
/// reference instead of to the rest curve's own sample, so a pose step below
/// the ribbon is carried through it (spec 4.2). \p jointRestLive selects the
/// samples that were really written; the others keep the rest curve.
RigExecPointFrameArray RigExecSampleRibbonFrames(
    const std::vector<GfVec3f> &posed,
    const std::vector<GfVec3f> &rest,
    int sampleCount,
    const std::vector<std::array<GfVec3d, 4>> &jointRests = {},
    const std::vector<bool> &jointRestLive = {});

/// Resolves the sample positions of a RigExecTwistDistribution: leaves an
/// authored \p weights alone, and fills an EMPTY one with \p count positions
/// spread evenly over [0, 1] (a single sample sits at the start).
///
/// Defaults in place rather than answering a fresh vector so that a caller
/// that has already drained the authored array into \p weights -- which both
/// callers do, from a VdfReadIterator and from a VtArray -- pays for one
/// vector per solver per frame and not two. Pass \p count = 1 for an
/// unauthored rigExec:count; the clamp to >= 1 is the kernel's.
///
/// ONE definition, called by the RigExecTwistDistribution exec callback and by
/// the baked program, because "no weights authored" is a defaulting rule and
/// not arithmetic the two paths may each invent.
void RigExecResolveTwistWeights(int count, std::vector<double> *weights);

/// Distributes twist from \p start to \p end over \p weights and pairs every
/// resulting frame with the START rest landmarks, which is what makes the
/// aggregate a rest-relative value its consumers can re-solve against.
///
/// \p startRest / \p endRest are the controls' rest frames; the callback
/// substitutes identity landmarks for an unwired one.
///
/// ONE definition, called by the RigExecTwistDistribution exec callback and by
/// the baked program: the rests half is the part a second expression would
/// most easily get wrong, since it is not what RigExecDistributeTwist returns.
RigExecPointFrameArray RigExecSolveTwistDistribution(
    const RigExecPointFrame &start,
    const RigExecPointFrame &end,
    const std::array<GfVec3d, 4> &startRest,
    const std::array<GfVec3d, 4> &endRest,
    const std::vector<double> &weights,
    double twistTurns,
    const std::vector<std::array<GfVec3d, 4>> &jointRests = {},
    const std::vector<bool> &jointRestLive = {});

/// The rotation part of a published point frame, as a quaternion; false for
/// a frame that is invalid, degenerate or singular.
///
/// Translation dropped and the axes orthonormalized first, which is what
/// makes the result a rotation and not merely the rotation-ish upper 3x3 of
/// a frame carried through a dozen multiplies. The same three steps
/// tools/biped/build_psd.py:_rotation takes, in the same order, so the
/// engine's driver delta and the gate's are the same arithmetic.
///
/// ROW-VECTOR, like the rest of this codebase: p' = p * M, GfMatrix4d
/// SetRotate(q) builds M with p*M == q.Transform(p), and therefore the
/// quaternion product (q1 * q2) corresponds to M(q2) * M(q1). That is why a
/// pose interpolator's `parent^-1 * world` IS the local rotation and not its
/// reverse.
///
/// ONE definition, called by the dynamic pose-interpolator phase and by the
/// baked PoseInterpolator step: the delta both measure is compared against
/// the same authored poses, so two spellings of it would be two rigs.
bool RigExecFrameRotation(const RigExecPointFrame &frame, GfQuatd *out);

}  // namespace rigExec

#endif  // RIGEXEC_SOLVER_KERNELS_H
