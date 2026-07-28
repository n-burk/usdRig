//
// RigExec CPU SIMD kernels implementation (SSE, baseline x64).
//
#include "simdKernels.h"

#include <emmintrin.h>
#include <xmmintrin.h>

namespace rigExec {

void
RigExecApplyWeightedMatrixSimd(
    const GfVec3f *in, GfVec3f *out, const float *weights, size_t count,
    const GfMatrix4d &transform)
{
    // Row-vector convention: p' = x*row0 + y*row1 + z*row2 + row3.
    const __m128 row0 = _mm_setr_ps(
        float(transform[0][0]), float(transform[0][1]),
        float(transform[0][2]), 0.0f);
    const __m128 row1 = _mm_setr_ps(
        float(transform[1][0]), float(transform[1][1]),
        float(transform[1][2]), 0.0f);
    const __m128 row2 = _mm_setr_ps(
        float(transform[2][0]), float(transform[2][1]),
        float(transform[2][2]), 0.0f);
    const __m128 row3 = _mm_setr_ps(
        float(transform[3][0]), float(transform[3][1]),
        float(transform[3][2]), 0.0f);

    for (size_t i = 0; i < count; ++i) {
        const __m128 q = _mm_setr_ps(in[i][0], in[i][1], in[i][2], 0.0f);
        __m128 moved = _mm_add_ps(
            _mm_add_ps(
                _mm_mul_ps(_mm_shuffle_ps(q, q, _MM_SHUFFLE(0, 0, 0, 0)),
                           row0),
                _mm_mul_ps(_mm_shuffle_ps(q, q, _MM_SHUFFLE(1, 1, 1, 1)),
                           row1)),
            _mm_add_ps(
                _mm_mul_ps(_mm_shuffle_ps(q, q, _MM_SHUFFLE(2, 2, 2, 2)),
                           row2),
                row3));
        const __m128 w = _mm_set1_ps(weights[i]);
        const __m128 blended =
            _mm_add_ps(q, _mm_mul_ps(w, _mm_sub_ps(moved, q)));
        alignas(16) float result[4];
        _mm_store_ps(result, blended);
        out[i] = GfVec3f(result[0], result[1], result[2]);
    }
}

}  // namespace rigExec
