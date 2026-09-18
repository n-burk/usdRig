//
// RigExec CPU SIMD kernels (spec §6.5).
//
// Direct SIMD over the stock contiguous element storage; scalar reference
// kernels remain the correctness baseline and SIMD is accepted only under
// output parity (spec §13.4: bulk float points within 1e-6 x character
// scale).
//
#ifndef RIGEXEC_MATH_SIMD_KERNELS_H
#define RIGEXEC_MATH_SIMD_KERNELS_H

#include "pxr/pxr.h"
#include "pxr/base/gf/matrix4d.h"
#include "pxr/base/gf/vec3f.h"

#include <cstddef>

PXR_NAMESPACE_USING_DIRECTIVE

namespace rigExec {

struct RigExecSkinLayout;

/// SSE weighted-matrix movement kernel: out_i = in_i + w_i (T in_i - in_i)
/// in float math (spec §7.4). in/out may alias.
void RigExecApplyWeightedMatrixSimd(
    const GfVec3f *in, GfVec3f *out, const float *weights, size_t count,
    const GfMatrix4d &transform);

/// SSE linear blend skinning: out_i = (1 - sum_k w_ik) in_i +
/// sum_k w_ik (T_ik in_i) in float math, the same rule as the scalar
/// RigExecApplyLinearBlendSkin and parity-gated against it. The caller has
/// validated \p layout. in/out may alias.
void RigExecApplyLinearBlendSkinSimd(
    const GfVec3f *in, GfVec3f *out, const RigExecSkinLayout &layout);

/// The same kernel against rows the CALLER narrowed, for one that skins
/// several ranges of an array against one influence table and would
/// otherwise narrow the whole table once per range.
///
/// \p rows is layout.transformCount * 16 floats, entry t filled by
/// RigExecNarrowSkinRows from layout.transforms[t]; null narrows them here,
/// which is what the three-argument form above does. Only the entries the
/// range's own points index are ever read, so a caller that knows which
/// influences a range uses may leave the rest identity.
void RigExecApplyLinearBlendSkinSimd(
    const GfVec3f *in, GfVec3f *out, const RigExecSkinLayout &layout,
    const float *rows);

/// Narrows one influence matrix into the 16 floats the kernel above reads:
/// four rows of three components each, padded to four.
///
/// Exposed so a caller keeping its own rows table fills an entry of it with
/// the same narrowing the kernel performs -- a per-matrix operation, so the
/// values are identical however many times it is done.
void RigExecNarrowSkinRows(const GfMatrix4d &transform, float *rows);

/// The number of floats RigExecNarrowSkinRows fills per influence.
inline constexpr size_t RigExecSkinRowStride = 16;

}  // namespace rigExec

#endif  // RIGEXEC_MATH_SIMD_KERNELS_H
