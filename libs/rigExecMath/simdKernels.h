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

/// SSE weighted-matrix movement kernel: out_i = in_i + w_i (T in_i - in_i)
/// in float math (spec §7.4). in/out may alias.
void RigExecApplyWeightedMatrixSimd(
    const GfVec3f *in, GfVec3f *out, const float *weights, size_t count,
    const GfMatrix4d &transform);

}  // namespace rigExec

#endif  // RIGEXEC_MATH_SIMD_KERNELS_H
