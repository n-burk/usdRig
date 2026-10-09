// Times the shared operation-graph executor (libs/rigExecGraph/opGraph.cpp)
// against a straight loop, and against a small thread pool used as the
// optional cluster dispatcher. Investigation only.
#include "rigExecGraph/opGraph.h"
#include "skin_kernel.h"

#include <algorithm>
#include <chrono>
#include <cstdio>
#include <stdexcept>
#include <condition_variable>
#include <cstdint>
#include <functional>
#include <iostream>
#include <mutex>
#include <queue>
#include <string>
#include <thread>
#include <vector>

namespace {

struct Pool {
    explicit Pool(int threads)
    {
        workers.reserve(size_t(threads));
        for (int i = 0; i < threads; ++i) {
            workers.emplace_back([this] { Loop(); });
        }
    }
    ~Pool()
    {
        {
            std::lock_guard<std::mutex> lock(mu);
            stop = true;
        }
        cv.notify_all();
        for (auto &worker : workers) {
            worker.join();
        }
    }
    void Dispatch(std::function<void()> task)
    {
        {
            std::lock_guard<std::mutex> lock(mu);
            queue.push(std::move(task));
            ++outstanding;
        }
        cv.notify_one();
    }
    void Wait()
    {
        std::unique_lock<std::mutex> lock(mu);
        done.wait(lock, [&] { return outstanding == 0 && queue.empty(); });
    }

    std::mutex mu;
    std::condition_variable cv, done;
    std::queue<std::function<void()>> queue;
    std::vector<std::thread> workers;
    int outstanding = 0;
    bool stop = false;

    void Loop()
    {
        for (;;) {
            std::function<void()> task;
            {
                std::unique_lock<std::mutex> lock(mu);
                cv.wait(lock, [&] { return stop || !queue.empty(); });
                if (stop && queue.empty()) {
                    return;
                }
                task = std::move(queue.front());
                queue.pop();
            }
            task();
            {
                std::lock_guard<std::mutex> lock(mu);
                if (--outstanding == 0 && queue.empty()) {
                    done.notify_all();
                }
            }
        }
    }
};

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
    return samples[samples.size() / 2];
}

rigExec::RigExecCompiledGraph
Build(int chains, int depth, double grain)
{
    std::vector<rigExec::RigExecOpDescriptor> desc;
    desc.reserve(size_t(chains * depth));
    for (int c = 0; c < chains; ++c) {
        for (int d = 0; d < depth; ++d) {
            rigExec::RigExecOpDescriptor op;
            char key[32];
            std::snprintf(key, sizeof(key), "c%04d-o%04d", c, d);
            op.key = key;
            op.kind = 1;
            const rigExec::RigExecValueId value =
                rigExec::RigExecValueId(c * depth + d + 1);
            op.writes.push_back(value);
            if (d) {
                op.reads.push_back(value - 1);
            }
            desc.push_back(std::move(op));
        }
    }
    rigExec::RigExecCompiledGraph graph;
    std::string error;
    if (!rigExec::RigExecCompileOpGraph(desc, {}, rigExec::RigExecCyclePolicy::Reject,
                                        &graph, &error)) {
        throw std::runtime_error(error);
    }
    std::vector<double> costs(graph.ops.size(), 1.0);
    if (!rigExec::RigExecLowerOpClusters(&graph, costs, grain, &error)) {
        throw std::runtime_error(error);
    }
    return graph;
}

void
Run(const rigExec::RigExecCompiledGraph &graph, bool heavy,
    const float *in, float *out, const int *indices, const float *weights,
    const float *rows, size_t points, Pool *pool)
{
    std::vector<double> sink(graph.ops.size(), 1.0);
    rigExec::RigExecOpCallbacks callbacks;
    callbacks.run = [&](uint32_t i) {
        if (!heavy) {
            double x = sink[i];
            for (int k = 0; k < 8; ++k) {
                x = x * 1.0000001 + 0.25;
            }
            sink[i] = x;
            return true;
        }
        const size_t begin = (size_t(i) % 8) * (points / 8);
        ProbeSkin4(in + begin * 3, out + begin * 3, indices + begin * 4,
                   weights + begin * 4, rows, points / 8);
        return true;
    };
    callbacks.skip = [](uint32_t) {};
    callbacks.changed = [](rigExec::RigExecValueId) { return true; };
    if (pool) {
        callbacks.dispatch = [&](std::function<void()> task) {
            pool->Dispatch(std::move(task));
        };
        callbacks.wait = [&] { pool->Wait(); };
    }
    rigExec::RigExecOpExecution execution;
    std::string error;
    if (!rigExec::RigExecExecuteOpGraph(graph, {}, {}, true, callbacks,
                                        &execution, &error)) {
        throw std::runtime_error(error);
    }
    if (execution.executed != graph.ops.size()) {
        throw std::runtime_error("not every op ran");
    }
    double total = 0;
    for (double value : sink) {
        total += value;
    }
    volatile double guard = total;
    if (heavy) {
        guard += out ? out[0] : 0;
    }
}

void
Report(const char *name, const rigExec::RigExecCompiledGraph &graph,
       double us)
{
    std::cout << name << " ops=" << graph.ops.size()
              << " clusters=" << graph.clusters.size()
              << " longestPath=" << graph.longestPath
              << " median_us=" << us << "\n";
}

}  // namespace

int
main()
{
    const int threads = int(std::thread::hardware_concurrency());
    std::cout << "threads=" << threads << "\n";
    Pool pool(std::max(1, threads));

    {
        auto chain = Build(1, 2000, 0);
        Report("chain2000_serial", chain,
               MedianUs(2, 20, [&] { Run(chain, false, nullptr, nullptr,
                                         nullptr, nullptr, nullptr, 0,
                                         nullptr); }));
        Report("chain2000_parallel", chain,
               MedianUs(2, 20, [&] { Run(chain, false, nullptr, nullptr,
                                         nullptr, nullptr, nullptr, 0,
                                         &pool); }));
        double direct = MedianUs(2, 20, [&] {
            double x = 1;
            for (int i = 0; i < 2000; ++i) {
                for (int k = 0; k < 8; ++k) {
                    x = x * 1.0000001 + 0.25;
                }
            }
            volatile double guard = x;
            (void)guard;
        });
        std::cout << "chain2000_straight median_us=" << direct << "\n";
    }

    {
        auto wide = Build(64, 32, 0);
        auto packed = Build(64, 32, 32);
        Report("wide_grain0_serial", wide,
               MedianUs(2, 15, [&] { Run(wide, false, nullptr, nullptr,
                                         nullptr, nullptr, nullptr, 0,
                                         nullptr); }));
        Report("wide_grain0_parallel", wide,
               MedianUs(2, 15, [&] { Run(wide, false, nullptr, nullptr,
                                         nullptr, nullptr, nullptr, 0,
                                         &pool); }));
        Report("wide_packed_serial", packed,
               MedianUs(2, 15, [&] { Run(packed, false, nullptr, nullptr,
                                         nullptr, nullptr, nullptr, 0,
                                         nullptr); }));
        Report("wide_packed_parallel", packed,
               MedianUs(2, 15, [&] { Run(packed, false, nullptr, nullptr,
                                         nullptr, nullptr, nullptr, 0,
                                         &pool); }));
    }

    constexpr size_t kPoints = 8192;
    constexpr size_t kInfluences = 64;
    std::vector<float> in(kPoints * 3), out(kPoints * 3), weights(kPoints * 4),
        rows(kInfluences * 16);
    std::vector<int> indices(kPoints * 4);
    for (size_t i = 0; i < kPoints; ++i) {
        in[i * 3 + 0] = float(i) * 0.01f;
        in[i * 3 + 1] = 1.0f;
        in[i * 3 + 2] = -0.25f;
        for (int k = 0; k < 4; ++k) {
            indices[i * 4 + k] = int((i + size_t(k) * 3) % kInfluences);
            weights[i * 4 + k] = k == 3 ? 0.0f : (k == 0 ? 0.5f : 0.25f);
        }
    }
    for (size_t t = 0; t < kInfluences; ++t) {
        float *row = rows.data() + t * 16;
        row[0] = row[5] = row[10] = row[15] = 1.0f;
        row[12] = float(t) * 0.001f;
    }
    auto skins = Build(8, 1, 0);
    Report("eight_skins_serial", skins,
           MedianUs(1, 10, [&] {
               Run(skins, true, in.data(), out.data(), indices.data(),
                   weights.data(), rows.data(), kPoints, nullptr);
           }));
    Report("eight_skins_parallel", skins,
           MedianUs(1, 10, [&] {
               Run(skins, true, in.data(), out.data(), indices.data(),
                   weights.data(), rows.data(), kPoints, &pool);
           }));
    std::cout << "eight_skins checksum=" << ProbeChecksum(out.data(), out.size())
              << "\n";
    return 0;
}
