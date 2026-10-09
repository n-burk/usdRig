// Linear-blend skinning shaped like the runtime SSE2 kernel
// (libs/rigExecRuntime/geometry.cpp, RrGeoApplyLinearBlendSkinSimd).
// Row-vector convention: p' = x*row0 + y*row1 + z*row2 + row3.
// A zero weight is skipped, matching that kernel, so a 0*NaN influence
// does not poison the sum. Investigation only.
#ifndef RIGEXEC_PROBE_SKIN_KERNEL_H
#define RIGEXEC_PROBE_SKIN_KERNEL_H

#include <cstddef>
#include <cstdint>
#include <emmintrin.h>
#include <xmmintrin.h>

inline void
ProbeSkinDynamic(const float *in, float *out, const int *indices,
                 const float *weights, const float *rows, size_t count,
                 size_t elementSize)
{
    constexpr size_t kStride = 16;
    for (size_t i = 0; i < count; ++i) {
        const __m128 q = _mm_setr_ps(in[i * 3 + 0], in[i * 3 + 1],
                                     in[i * 3 + 2], 0.0f);
        const __m128 qx = _mm_shuffle_ps(q, q, _MM_SHUFFLE(0, 0, 0, 0));
        const __m128 qy = _mm_shuffle_ps(q, q, _MM_SHUFFLE(1, 1, 1, 1));
        const __m128 qz = _mm_shuffle_ps(q, q, _MM_SHUFFLE(2, 2, 2, 2));
        __m128 sum = _mm_setzero_ps();
        float total = 0.0f;
        for (size_t k = 0; k < elementSize; ++k) {
            const float w = weights[i * elementSize + k];
            if (w == 0.0f) {
                continue;
            }
            const float *t =
                rows + size_t(indices[i * elementSize + k]) * kStride;
            const __m128 moved = _mm_add_ps(
                _mm_add_ps(_mm_mul_ps(qx, _mm_loadu_ps(t)),
                           _mm_mul_ps(qy, _mm_loadu_ps(t + 4))),
                _mm_add_ps(_mm_mul_ps(qz, _mm_loadu_ps(t + 8)),
                           _mm_loadu_ps(t + 12)));
            sum = _mm_add_ps(sum, _mm_mul_ps(_mm_set1_ps(w), moved));
            total += w;
        }
        const __m128 blended =
            _mm_add_ps(_mm_mul_ps(_mm_set1_ps(1.0f - total), q), sum);
        alignas(16) float result[4];
        _mm_store_ps(result, blended);
        out[i * 3 + 0] = result[0];
        out[i * 3 + 1] = result[1];
        out[i * 3 + 2] = result[2];
    }
}

// elementSize == 4, still skipping a zero weight.
inline void
ProbeSkin4(const float *in, float *out, const int *indices,
           const float *weights, const float *rows, size_t count)
{
    constexpr size_t kStride = 16;
    for (size_t i = 0; i < count; ++i) {
        const __m128 q = _mm_setr_ps(in[i * 3 + 0], in[i * 3 + 1],
                                     in[i * 3 + 2], 0.0f);
        const __m128 qx = _mm_shuffle_ps(q, q, _MM_SHUFFLE(0, 0, 0, 0));
        const __m128 qy = _mm_shuffle_ps(q, q, _MM_SHUFFLE(1, 1, 1, 1));
        const __m128 qz = _mm_shuffle_ps(q, q, _MM_SHUFFLE(2, 2, 2, 2));
        __m128 sum = _mm_setzero_ps();
        float total = 0.0f;
        const int *idx = indices + i * 4;
        const float *w = weights + i * 4;
        for (int k = 0; k < 4; ++k) {
            if (w[k] == 0.0f) {
                continue;
            }
            const float *t = rows + size_t(idx[k]) * kStride;
            const __m128 moved = _mm_add_ps(
                _mm_add_ps(_mm_mul_ps(qx, _mm_loadu_ps(t)),
                           _mm_mul_ps(qy, _mm_loadu_ps(t + 4))),
                _mm_add_ps(_mm_mul_ps(qz, _mm_loadu_ps(t + 8)),
                           _mm_loadu_ps(t + 12)));
            sum = _mm_add_ps(sum, _mm_mul_ps(_mm_set1_ps(w[k]), moved));
            total += w[k];
        }
        const __m128 blended =
            _mm_add_ps(_mm_mul_ps(_mm_set1_ps(1.0f - total), q), sum);
        alignas(16) float result[4];
        _mm_store_ps(result, blended);
        out[i * 3 + 0] = result[0];
        out[i * 3 + 1] = result[1];
        out[i * 3 + 2] = result[2];
    }
}

// One weighted rigid move, the matrix-mover SSE2 shape.
inline void
ProbeMatrixBlend(const float *in, float *out, const float *rows,
                 const float *weights, size_t count)
{
    const __m128 row0 = _mm_loadu_ps(rows);
    const __m128 row1 = _mm_loadu_ps(rows + 4);
    const __m128 row2 = _mm_loadu_ps(rows + 8);
    const __m128 row3 = _mm_loadu_ps(rows + 12);
    for (size_t i = 0; i < count; ++i) {
        const __m128 q = _mm_setr_ps(in[i * 3 + 0], in[i * 3 + 1],
                                     in[i * 3 + 2], 0.0f);
        const __m128 moved = _mm_add_ps(
            _mm_add_ps(_mm_mul_ps(_mm_shuffle_ps(q, q, _MM_SHUFFLE(0, 0, 0, 0)),
                                  row0),
                       _mm_mul_ps(_mm_shuffle_ps(q, q, _MM_SHUFFLE(1, 1, 1, 1)),
                                  row1)),
            _mm_add_ps(_mm_mul_ps(_mm_shuffle_ps(q, q, _MM_SHUFFLE(2, 2, 2, 2)),
                                  row2),
                       row3));
        const float w = weights[i];
        __m128 blended;
        if (w <= 0.0f) {
            blended = q;
        } else if (w >= 1.0f) {
            blended = moved;
        } else {
            blended = _mm_add_ps(q, _mm_mul_ps(_mm_set1_ps(w),
                                               _mm_sub_ps(moved, q)));
        }
        alignas(16) float result[4];
        _mm_store_ps(result, blended);
        out[i * 3 + 0] = result[0];
        out[i * 3 + 1] = result[1];
        out[i * 3 + 2] = result[2];
    }
}

// Four matrix movers fused into one SSE2 pass. The point stays in
// registers across the four transforms.
inline void
ProbeMatrixBlendFused4Simd(const float *in, float *out, const float *rows4,
                           const float *weights4, size_t count)
{
    __m128 rows[4][4];
    for (int m = 0; m < 4; ++m) {
        rows[m][0] = _mm_loadu_ps(rows4 + m * 16);
        rows[m][1] = _mm_loadu_ps(rows4 + m * 16 + 4);
        rows[m][2] = _mm_loadu_ps(rows4 + m * 16 + 8);
        rows[m][3] = _mm_loadu_ps(rows4 + m * 16 + 12);
    }
    for (size_t i = 0; i < count; ++i) {
        __m128 q = _mm_setr_ps(in[i * 3 + 0], in[i * 3 + 1], in[i * 3 + 2],
                               0.0f);
        for (int m = 0; m < 4; ++m) {
            const __m128 moved = _mm_add_ps(
                _mm_add_ps(_mm_mul_ps(
                               _mm_shuffle_ps(q, q, _MM_SHUFFLE(0, 0, 0, 0)),
                               rows[m][0]),
                           _mm_mul_ps(
                               _mm_shuffle_ps(q, q, _MM_SHUFFLE(1, 1, 1, 1)),
                               rows[m][1])),
                _mm_add_ps(_mm_mul_ps(
                               _mm_shuffle_ps(q, q, _MM_SHUFFLE(2, 2, 2, 2)),
                               rows[m][2]),
                           rows[m][3]));
            const float w = weights4[size_t(m) * count + i];
            if (w <= 0.0f) {
                continue;
            }
            if (w >= 1.0f) {
                q = moved;
            } else {
                q = _mm_add_ps(q, _mm_mul_ps(_mm_set1_ps(w), _mm_sub_ps(moved, q)));
            }
        }
        alignas(16) float result[4];
        _mm_store_ps(result, q);
        out[i * 3 + 0] = result[0];
        out[i * 3 + 1] = result[1];
        out[i * 3 + 2] = result[2];
    }
}

// Four matrix movers fused into one scalar pass over the points.
inline void
ProbeMatrixBlendFused4(const float *in, float *out, const float *rows4,
                       const float *weights4, size_t count)
{
    for (size_t i = 0; i < count; ++i) {
        float x = in[i * 3 + 0];
        float y = in[i * 3 + 1];
        float z = in[i * 3 + 2];
        for (int m = 0; m < 4; ++m) {
            const float *rows = rows4 + m * 16;
            const float w = weights4[m * count + i];
            const float mx = x * rows[0] + y * rows[4] + z * rows[8] + rows[12];
            const float my = x * rows[1] + y * rows[5] + z * rows[9] + rows[13];
            const float mz = x * rows[2] + y * rows[6] + z * rows[10] + rows[14];
            if (w <= 0.0f) {
                continue;
            }
            if (w >= 1.0f) {
                x = mx;
                y = my;
                z = mz;
            } else {
                x = x + w * (mx - x);
                y = y + w * (my - y);
                z = z + w * (mz - z);
            }
        }
        out[i * 3 + 0] = x;
        out[i * 3 + 1] = y;
        out[i * 3 + 2] = z;
    }
}

inline uint64_t
ProbeChecksum(const float *data, size_t count)
{
    uint64_t sum = 0;
    for (size_t i = 0; i < count; ++i) {
        uint32_t bits = 0;
        __builtin_memcpy(&bits, &data[i], sizeof(bits));
        sum ^= uint64_t(bits) * (i + 1);
    }
    return sum;
}

#endif
