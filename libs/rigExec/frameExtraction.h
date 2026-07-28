//
// Per-element frame extraction from a packed solver boundary (spec §5.7).
//
// A solver publishes one aggregate RigExecPointFrameArray; each joint it
// poses takes exactly one element of it. Two places need that extraction and
// they must agree bit for bit: the joint's computePointFrame registration
// (when a joint is reached through exec) and RigExecRigEvaluator, which
// indexes the solver's array directly using its in-memory solver->joint
// binding. Duplicating the math would let the two drift silently, so it lives
// here once.
//
// This is the former RigExecPointFrameView out:space math, kept verbatim
// through the view-free rework (user-directed 2026-07-25).
//
#ifndef RIGEXEC_FRAME_EXTRACTION_H
#define RIGEXEC_FRAME_EXTRACTION_H

#include "types.h"

#include "pxr/base/gf/matrix4d.h"
#include "pxr/base/gf/vec4d.h"

#include <array>
#include <cmath>
#include <cstddef>

PXR_NAMESPACE_USING_DIRECTIVE

namespace rigExec {

/// The four identity landmarks a frame is measured against.
inline const std::array<GfVec3d, 4> &
RigExecIdentityLandmarks()
{
    static const std::array<GfVec3d, 4> identity = {
        GfVec3d(0), GfVec3d(1, 0, 0), GfVec3d(0, 1, 0), GfVec3d(0, 0, 1)};
    return identity;
}

/// Samples an affine map at the identity landmarks.
///
/// A non-finite landmark makes the frame degenerate rather than propagating
/// NaN into consumers.
inline RigExecPointFrame
RigExecFrameFromMatrix(const GfMatrix4d &m)
{
    RigExecPointFrame frame;
    const std::array<GfVec3d, 4> &identity = RigExecIdentityLandmarks();
    for (size_t i = 0; i < 4; ++i) {
        frame.points[i] = m.TransformAffine(identity[i]);
        for (int a = 0; a < 3; ++a) {
            if (!std::isfinite(frame.points[i][a])) {
                frame.flags = RigExecPointFrameDegenerate;
                return frame;
            }
        }
    }
    frame.flags = RigExecPointFrameValid;
    return frame;
}

/// Out-space of one element of an aggregate: normalized posed axes scaled by
/// the posed/rest axis-length ratios (rest spaces are orthonormalized per the
/// Ir contract, so the ratio is what carries stretch). Identity when the
/// element is absent or degenerate.
inline GfMatrix4d
RigExecElementOutSpace(const RigExecPointFrameArray *source, size_t index)
{
    GfMatrix4d m(1.0);
    if (!source || index >= source->GetSize() ||
        !source->frames[index].IsValid()) {
        return m;
    }
    const std::array<GfVec3d, 4> &posed = source->frames[index].points;
    const GfVec3d origin = posed[0];
    const bool hasRest = index < source->rests.size();
    GfMatrix4d rm(1.0);
    for (int a = 0; a < 3; ++a) {
        GfVec3d direction = posed[a + 1] - origin;
        const double posedLength = direction.GetLength();
        if (posedLength < 1e-12) {
            return m;  // degenerate axis: identity
        }
        direction /= posedLength;
        double ratio = 1.0;
        if (hasRest) {
            const double restLength =
                (source->rests[index][a + 1] - source->rests[index][0])
                    .GetLength();
            if (restLength > 1e-12) {
                ratio = posedLength / restLength;
            }
        }
        rm.SetRow(a, GfVec4d(direction[0] * ratio, direction[1] * ratio,
                             direction[2] * ratio, 0));
    }
    rm.SetRow(3, GfVec4d(origin[0], origin[1], origin[2], 1));
    return rm;
}

/// Extracts element \p index of \p source as a scalar joint frame.
///
/// A solver frame may carry Valid|Degenerate simultaneously; a degenerate
/// element must not be laundered into an identity frame by
/// RigExecElementOutSpace (which returns identity on a collapsed axis). Only a
/// valid, non-degenerate element extracts; otherwise degeneracy propagates so
/// downstream computeMatrix/consumers see the failure. An out-of-range element
/// is likewise degenerate rather than silently following the parent.
inline RigExecPointFrame
RigExecExtractElementFrame(
    const RigExecPointFrameArray *source, size_t index)
{
    if (source && index < source->GetSize()) {
        const RigExecPointFrame &f = source->frames[index];
        if (f.IsValid() && !f.IsDegenerate()) {
            return RigExecFrameFromMatrix(
                RigExecElementOutSpace(source, index));
        }
    }
    RigExecPointFrame degenerate;
    degenerate.flags = RigExecPointFrameDegenerate;
    return degenerate;
}

}  // namespace rigExec

#endif  // RIGEXEC_FRAME_EXTRACTION_H
