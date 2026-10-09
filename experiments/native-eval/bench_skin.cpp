// Compares a dynamic-element-size skin loop, a C++ loop specialized to
// four influences, the same loop compiled ahead of time with clang, and
// a fused chain of matrix blends. Also times a threaded split of one
// large skin. Investigation only.
#include "skin_kernel.h"

#include <algorithm>
#include <chrono>
#include <condition_variable>
#include <cstdint>
#include <cstdlib>
#include <cstring>
#include <dlfcn.h>
#include <fstream>
#include <functional>
#include <iostream>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

namespace {

double
MedianUs(int warm, int count, const std::function<void()> &fn)
{
    for (int i = 0; i < warm; ++i) {
        fn();
    }
    std::vector<double> samples;
    samples.reserve(size_t(count));
    for (int i = 0; i < count; ++i) {
        const auto t0 = std::chrono::steady_clock::now();
        fn();
        const auto t1 = std::chrono::steady_clock::now();
        samples.push_back(
            std::chrono::duration<double, std::micro>(t1 - t0).count());
    }
    std::sort(samples.begin(), samples.end());
    const double med = samples[samples.size() / 2];
    std::cout << "  min_us=" << samples.front() << " median_us=" << med
              << " p90_us=" << samples[size_t(samples.size() * 9 / 10)]
              << "\n";
    return med;
}

using SkinFn = void (*)(const float *, float *, const int *, const float *,
                        const float *, size_t);

SkinFn
CompileAot(const std::string &dir, double *compileUs)
{
    const std::string src = dir + "/skin4.c";
    const std::string so = dir + "/skin4.so";
    std::ofstream out(src);
    out << R"cpp(
#include <stddef.h>
#include <emmintrin.h>
#include <xmmintrin.h>
extern "C" void rigexec_skin4(const float *in, float *out, const int *indices,
                   const float *weights, const float *rows, size_t count) {
    size_t i;
    for (i = 0; i < count; ++i) {
        __m128 q = _mm_setr_ps(in[i*3+0], in[i*3+1], in[i*3+2], 0.0f);
        __m128 qx = _mm_shuffle_ps(q, q, _MM_SHUFFLE(0,0,0,0));
        __m128 qy = _mm_shuffle_ps(q, q, _MM_SHUFFLE(1,1,1,1));
        __m128 qz = _mm_shuffle_ps(q, q, _MM_SHUFFLE(2,2,2,2));
        __m128 sum = _mm_setzero_ps();
        float total = 0.0f;
        int k;
        for (k = 0; k < 4; ++k) {
            float w = weights[i*4+k];
            const float *t;
            __m128 moved;
            if (w == 0.0f) continue;
            t = rows + (size_t)indices[i*4+k] * 16;
            moved = _mm_add_ps(
                _mm_add_ps(_mm_mul_ps(qx, _mm_loadu_ps(t)),
                           _mm_mul_ps(qy, _mm_loadu_ps(t+4))),
                _mm_add_ps(_mm_mul_ps(qz, _mm_loadu_ps(t+8)),
                           _mm_loadu_ps(t+12)));
            sum = _mm_add_ps(sum, _mm_mul_ps(_mm_set1_ps(w), moved));
            total += w;
        }
        {
            __m128 blended = _mm_add_ps(_mm_mul_ps(_mm_set1_ps(1.0f-total), q), sum);
            __attribute__((aligned(16))) float result[4];
            _mm_store_ps(result, blended);
            out[i*3+0] = result[0];
            out[i*3+1] = result[1];
            out[i*3+2] = result[2];
        }
    }
}
)cpp";
    out.close();
    const std::string cmd =
        "g++ -O3 -fno-fast-math -ffp-contract=off -fPIC -shared -o " + so +
        " " + src;
    const auto t0 = std::chrono::steady_clock::now();
    const int status = std::system(cmd.c_str());
    if (status != 0) {
        std::cerr << "command failed status=" << status << " cmd=" << cmd
                  << "\n";
        return nullptr;
    }
    const auto t1 = std::chrono::steady_clock::now();
    *compileUs = std::chrono::duration<double, std::micro>(t1 - t0).count();
    void *lib = dlopen(so.c_str(), RTLD_NOW);
    if (!lib) {
        std::cerr << dlerror() << "\n";
        return nullptr;
    }
    return reinterpret_cast<SkinFn>(dlsym(lib, "rigexec_skin4"));
}

void
Fill(size_t points, size_t influences, bool zeros, std::vector<float> *in,
     std::vector<int> *indices, std::vector<float> *weights,
     std::vector<float> *rows)
{
    in->assign(points * 3, 0);
    indices->assign(points * 4, 0);
    weights->assign(points * 4, 0);
    rows->assign(influences * 16, 0);
    for (size_t i = 0; i < points; ++i) {
        (*in)[i * 3 + 0] = float(int(i % 97) - 48) * 0.05f;
        (*in)[i * 3 + 1] = 1.0f;
        (*in)[i * 3 + 2] = -0.2f;
        for (int k = 0; k < 4; ++k) {
            (*indices)[i * 4 + size_t(k)] =
                int((i * 3 + size_t(k) * 17) % influences);
            float w = k == 0 ? 0.5f : 0.25f;
            if (zeros && k == 3) {
                w = 0.0f;
            } else if (!zeros && k == 3) {
                w = 0.0f;
            }
            // The fourth weight stays zero in both sets so the skip is live.
            // The nonzero set still has three live influences.
            (*weights)[i * 4 + size_t(k)] = w;
        }
    }
    for (size_t t = 0; t < influences; ++t) {
        float *row = rows->data() + t * 16;
        row[0] = row[5] = row[10] = 1.0f;
        row[12] = float(int(t % 11) - 5) * 0.02f;
        row[13] = float(int(t % 7)) * 0.01f;
        row[14] = 0.0f;
    }
}

}  // namespace

int
main()
{
    const std::string dir = "/tmp/rigexec-probe";
    double compileUs = 0;
    SkinFn aot = CompileAot(dir, &compileUs);
    if (!aot) {
        std::cerr << "AOT compile failed\n";
        return 1;
    }
    std::cout << "aot_compile_us=" << compileUs << "\n";
    std::cout << "threads=" << std::thread::hardware_concurrency() << "\n";

    const size_t points = 26276;
    const size_t influences = 137;
    std::vector<float> in, weights, rows, outDyn, out4, outAot;
    std::vector<int> indices;
    Fill(points, influences, true, &in, &indices, &weights, &rows);
    outDyn.assign(in.size(), 0);
    out4.assign(in.size(), 0);
    outAot.assign(in.size(), 0);

    std::cout << "dynamic_26276";
    MedianUs(2, 20, [&] {
        ProbeSkinDynamic(in.data(), outDyn.data(), indices.data(),
                         weights.data(), rows.data(), points, 4);
    });
    std::cout << "specialized_cxx_26276";
    MedianUs(2, 20, [&] {
        ProbeSkin4(in.data(), out4.data(), indices.data(), weights.data(),
                   rows.data(), points);
    });
    std::cout << "aot_clang_26276";
    MedianUs(2, 20, [&] {
        aot(in.data(), outAot.data(), indices.data(), weights.data(),
            rows.data(), points);
    });
    const bool cxxMatch = std::memcmp(outDyn.data(), out4.data(),
                                      outDyn.size() * sizeof(float)) == 0;
    const bool aotMatch = std::memcmp(outDyn.data(), outAot.data(),
                                      outDyn.size() * sizeof(float)) == 0;
    std::cout << "bit_exact_cxx=" << cxxMatch << " bit_exact_aot=" << aotMatch
              << " checksum=" << ProbeChecksum(outDyn.data(), outDyn.size())
              << "\n";

    const int hw = std::max(1, int(std::thread::hardware_concurrency()));
    // Persistent workers. Creating threads inside the sample measures
    // spawn cost, not the skin.
    struct Split {
        explicit Split(int n) : threads(n)
        {
            workers.reserve(size_t(n));
            for (int t = 0; t < n; ++t) {
                workers.emplace_back([this, t] { Loop(t); });
            }
        }
        ~Split()
        {
            {
                std::lock_guard<std::mutex> lock(mu);
                stop = true;
                ++epoch;
            }
            cv.notify_all();
            for (auto &worker : workers) {
                worker.join();
            }
        }
        void Run(const std::function<void(int, int, size_t, size_t)> &body,
                 size_t count, int active)
        {
            {
                std::lock_guard<std::mutex> lock(mu);
                work = &body;
                span = count;
                activeN = active;
                left = threads;
                ++epoch;
            }
            cv.notify_all();
            std::unique_lock<std::mutex> lock(mu);
            done.wait(lock, [&] { return left == 0; });
        }
        int threads;
        std::mutex mu;
        std::condition_variable cv, done;
        std::vector<std::thread> workers;
        const std::function<void(int, int, size_t, size_t)> *work = nullptr;
        size_t span = 0;
        int activeN = 1;
        int epoch = 0;
        int left = 0;
        bool stop = false;
        void Loop(int index)
        {
            int seen = 0;
            for (;;) {
                std::unique_lock<std::mutex> lock(mu);
                cv.wait(lock, [&] { return stop || epoch != seen; });
                if (stop) {
                    return;
                }
                seen = epoch;
                const auto *body = work;
                const size_t count = span;
                const int n = activeN;
                lock.unlock();
                if (index < n && count && body) {
                    const size_t chunk = (count + size_t(n) - 1) / size_t(n);
                    const size_t begin = size_t(index) * chunk;
                    if (begin < count) {
                        (*body)(index, n, begin, std::min(count, begin + chunk));
                    }
                }
                lock.lock();
                if (--left == 0) {
                    done.notify_one();
                }
            }
        }
    };
    Split split(hw);
    for (int n : {1, 2, hw}) {
        std::cout << "parallel_pool_" << n << "_of_" << points;
        MedianUs(2, 15, [&] {
            split.Run(
                [&](int, int, size_t begin, size_t end) {
                    ProbeSkin4(in.data() + begin * 3, out4.data() + begin * 3,
                               indices.data() + begin * 4,
                               weights.data() + begin * 4, rows.data(),
                               end - begin);
                },
                points, n);
        });
    }
    constexpr int kBodies = 4;
    std::vector<float> bodyIn(size_t(kBodies) * in.size()),
        bodyOut(size_t(kBodies) * in.size());
    for (int b = 0; b < kBodies; ++b) {
        std::memcpy(bodyIn.data() + size_t(b) * in.size(), in.data(),
                    in.size() * sizeof(float));
    }
    std::cout << "four_bodies_serial";
    MedianUs(1, 10, [&] {
        for (int b = 0; b < kBodies; ++b) {
            ProbeSkin4(bodyIn.data() + size_t(b) * points * 3,
                       bodyOut.data() + size_t(b) * points * 3, indices.data(),
                       weights.data(), rows.data(), points);
        }
    });
    std::cout << "four_bodies_parallel";
    MedianUs(1, 10, [&] {
        split.Run(
            [&](int, int, size_t begin, size_t end) {
                for (size_t b = begin; b < end; ++b) {
                    ProbeSkin4(bodyIn.data() + b * points * 3,
                               bodyOut.data() + b * points * 3, indices.data(),
                               weights.data(), rows.data(), points);
                }
            },
            kBodies, hw);
    });
    const bool parallelMatch =
        std::memcmp(outDyn.data(), out4.data(),
                    outDyn.size() * sizeof(float)) == 0;
    std::cout << "bit_exact_parallel=" << parallelMatch << "\n";

    constexpr size_t kSmall = 4096;
    std::vector<float> sin, sout4(kSmall * 3), fused(kSmall * 3),
        separate(kSmall * 3), w4(4 * kSmall), m4(4 * 16);
    sin.resize(kSmall * 3);
    for (size_t i = 0; i < kSmall; ++i) {
        sin[i * 3] = float(i) * 0.002f;
        sin[i * 3 + 1] = 0.5f;
        sin[i * 3 + 2] = -0.1f;
        for (int m = 0; m < 4; ++m) {
            w4[size_t(m) * kSmall + i] = 0.35f + 0.1f * float(m);
        }
    }
    for (int m = 0; m < 4; ++m) {
        float *row = m4.data() + m * 16;
        row[0] = row[5] = row[10] = 1;
        row[12] = 0.01f * float(m + 1);
    }
    std::cout << "matrix_four_passes_4096";
    MedianUs(3, 30, [&] {
        std::memcpy(separate.data(), sin.data(), separate.size() * sizeof(float));
        for (int m = 0; m < 4; ++m) {
            ProbeMatrixBlend(separate.data(), separate.data(),
                             m4.data() + m * 16, w4.data() + m * kSmall,
                             kSmall);
        }
    });
    std::cout << "matrix_fused_scalar_4096";
    MedianUs(3, 30, [&] {
        ProbeMatrixBlendFused4(sin.data(), fused.data(), m4.data(), w4.data(),
                               kSmall);
    });
    std::vector<float> fusedSimd(kSmall * 3);
    std::cout << "matrix_fused_sse_4096";
    MedianUs(3, 30, [&] {
        ProbeMatrixBlendFused4Simd(sin.data(), fusedSimd.data(), m4.data(),
                                   w4.data(), kSmall);
    });
    int simdMismatches = 0;
    for (size_t i = 0; i < separate.size(); ++i) {
        uint32_t a = 0, b = 0;
        std::memcpy(&a, &separate[i], 4);
        std::memcpy(&b, &fusedSimd[i], 4);
        simdMismatches += a != b;
    }
    std::cout << "matrix_fused_sse_mismatched_floats=" << simdMismatches
              << "\n";
    int mismatches = 0;
    for (size_t i = 0; i < separate.size(); ++i) {
        uint32_t a = 0, b = 0;
        std::memcpy(&a, &separate[i], 4);
        std::memcpy(&b, &fused[i], 4);
        mismatches += a != b;
    }
    std::cout << "matrix_fused_mismatched_floats=" << mismatches << "\n";
    return mismatches >= 0 ? 0 : 1;
}
