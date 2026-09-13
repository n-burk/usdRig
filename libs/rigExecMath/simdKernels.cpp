//
// RigExec CPU SIMD kernels implementation.
//
// SSE2 where the target has it -- every x86-64 build, and 32-bit x86 built
// with /arch:SSE2 or -msse2 -- and a scalar fallback everywhere else, so this
// file compiles on arm64 (Apple Silicon, iOS) instead of failing on the x86
// intrinsic headers. ARM64EC is deliberately excluded even though MSVC
// defines _M_X64 there.
//
// The fallback delegates to RigExecApplyWeightedMatrix, the same scalar
// reference kernel the parity mode (spec §13.4) compares SIMD output against,
// so on a non-SSE target the two paths agree exactly rather than to tolerance.
//
#include "simdKernels.h"

#include "solvers.h"

// RIGEXEC_DISABLE_SSE2 forces the fallback on a machine that has SSE2, so the
// non-x86 path can be compiled and run through the test suite here rather than
// only ever being exercised on hardware nobody building this owns.
#if !defined(RIGEXEC_DISABLE_SSE2)                                       \
    && (defined(__SSE2__) || (defined(_M_X64) && !defined(_M_ARM64EC))   \
        || (defined(_M_IX86_FP) && _M_IX86_FP >= 2))
#  define RIGEXEC_HAS_SSE2 1
#endif

#if defined(RIGEXEC_HAS_SSE2)
#  include <emmintrin.h>
#  include <xmmintrin.h>
#endif

#include <vector>

namespace rigExec {

void
RigExecNarrowSkinRows(const GfMatrix4d &transform, float *rows)
{
    for (int r = 0; r < 4; ++r) {
        for (int c = 0; c < 3; ++c) {
            rows[r * 4 + c] = float(transform[r][c]);
        }
        rows[r * 4 + 3] = 0.0f;
    }
}

#if !defined(RIGEXEC_HAS_SSE2)

void
RigExecApplyWeightedMatrixSimd(
    const GfVec3f *in, GfVec3f *out, const float *weights, size_t count,
    const GfMatrix4d &transform)
{
    for (size_t i = 0; i < count; ++i) {
        out[i] = GfVec3f(RigExecApplyWeightedMatrix(
            GfVec3d(in[i]), transform, weights[i]));
    }
}

void
RigExecApplyLinearBlendSkinSimd(
    const GfVec3f *in, GfVec3f *out, const RigExecSkinLayout &layout,
    const float *rows)
{
    // No SSE2, so no narrowed rows either: the scalar kernel reads the
    // matrices themselves, which the caller's table and this one agree on.
    (void)rows;
    RigExecApplyLinearBlendSkin(in, out, layout);
}

void
RigExecApplyLinearBlendSkinSimd(
    const GfVec3f *in, GfVec3f *out, const RigExecSkinLayout &layout)
{
    RigExecApplyLinearBlendSkin(in, out, layout);
}

#else

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
        __m128 blended;
        if (weights[i] <= 0.0f) {
            blended = q;
        } else if (weights[i] >= 1.0f) {
            blended = moved;
        } else {
            const __m128 w = _mm_set1_ps(weights[i]);
            blended =
                _mm_add_ps(q, _mm_mul_ps(w, _mm_sub_ps(moved, q)));
        }
        alignas(16) float result[4];
        _mm_store_ps(result, blended);
        out[i] = GfVec3f(result[0], result[1], result[2]);
    }
}

void
RigExecApplyLinearBlendSkinSimd(
    const GfVec3f *in, GfVec3f *out, const RigExecSkinLayout &layout,
    const float *rows)
{
    // Every influence's rows, narrowed to float once rather than once per
    // (point, slot). Unaligned loads from a plain float table sidestep any
    // question of __m128 storage alignment.
    //
    // A caller that skins several ranges against one influence table hands
    // its own table in and this narrows nothing; the narrowing is per matrix,
    // so the two tables hold the same floats either way.
    std::vector<float> narrowed;
    if (!rows) {
        narrowed.resize(layout.transformCount * RigExecSkinRowStride);
        for (size_t t = 0; t < layout.transformCount; ++t) {
            RigExecNarrowSkinRows(layout.transforms[t],
                                  &narrowed[t * RigExecSkinRowStride]);
        }
        rows = narrowed.data();
    }

    for (size_t i = 0; i < layout.pointCount; ++i) {
        const __m128 q = _mm_setr_ps(in[i][0], in[i][1], in[i][2], 0.0f);
        const __m128 qx = _mm_shuffle_ps(q, q, _MM_SHUFFLE(0, 0, 0, 0));
        const __m128 qy = _mm_shuffle_ps(q, q, _MM_SHUFFLE(1, 1, 1, 1));
        const __m128 qz = _mm_shuffle_ps(q, q, _MM_SHUFFLE(2, 2, 2, 2));
        __m128 sum = _mm_setzero_ps();
        float total = 0.0f;
        for (size_t k = 0; k < layout.elementSize; ++k) {
            const float w = layout.Weight(i, k);
            if (w == 0.0f) {
                continue;
            }
            const float *t =
                rows + size_t(layout.indices[i * layout.elementSize + k]) *
                           RigExecSkinRowStride;
            // Row-vector convention: p' = x*row0 + y*row1 + z*row2 + row3.
            const __m128 moved = _mm_add_ps(
                _mm_add_ps(_mm_mul_ps(qx, _mm_loadu_ps(t)),
                           _mm_mul_ps(qy, _mm_loadu_ps(t + 4))),
                _mm_add_ps(_mm_mul_ps(qz, _mm_loadu_ps(t + 8)),
                           _mm_loadu_ps(t + 12)));
            sum = _mm_add_ps(sum, _mm_mul_ps(_mm_set1_ps(w), moved));
            total += w;
        }
        // The complement stays with the rest point; exact when total == 1.
        const __m128 blended =
            _mm_add_ps(_mm_mul_ps(_mm_set1_ps(1.0f - total), q), sum);
        alignas(16) float result[4];
        _mm_store_ps(result, blended);
        out[i] = GfVec3f(result[0], result[1], result[2]);
    }
}

void
RigExecApplyLinearBlendSkinSimd(
    const GfVec3f *in, GfVec3f *out, const RigExecSkinLayout &layout)
{
    RigExecApplyLinearBlendSkinSimd(in, out, layout, /*rows=*/nullptr);
}

#endif  // RIGEXEC_HAS_SSE2

}  // namespace rigExec
