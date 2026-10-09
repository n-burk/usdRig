// Opens one .rigexec file, prints its static schedule, and times Execute.
// Investigation probe. Does not change the runtime.
#include "rigExecBinary/format.h"
#include "rigExecRuntime/runtime.h"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <fstream>
#include <iostream>
#include <map>
#include <string>
#include <vector>

namespace {

struct LevelStats {
    int levels = 0;
    int maxWidth = 0;
    int singleton = 0;
    int maxMembers = 0;
    double meanMembers = 0;
};

template <class Pred>
LevelStats
Levels(size_t count, Pred &&preds)
{
    LevelStats stats;
    if (!count) {
        return stats;
    }
    std::vector<int> indeg(count, 0);
    std::vector<std::vector<int>> succ(count);
    for (size_t i = 0; i < count; ++i) {
        for (int p : preds(i)) {
            if (p < 0 || size_t(p) >= count) {
                continue;
            }
            succ[size_t(p)].push_back(int(i));
            ++indeg[i];
        }
    }
    std::vector<int> ready;
    for (size_t i = 0; i < count; ++i) {
        if (!indeg[i]) {
            ready.push_back(int(i));
        }
    }
    int seen = 0;
    while (!ready.empty()) {
        stats.maxWidth = std::max(stats.maxWidth, int(ready.size()));
        ++stats.levels;
        std::vector<int> next;
        for (int node : ready) {
            ++seen;
            for (int s : succ[size_t(node)]) {
                if (--indeg[size_t(s)] == 0) {
                    next.push_back(s);
                }
            }
        }
        ready.swap(next);
    }
    if (seen != int(count)) {
        stats.levels = -1;
    }
    return stats;
}

template <class F>
void
Time(const char *name, int warm, int count, F &&fn)
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
    const double min = samples.front();
    const double med = samples[samples.size() / 2];
    const double p90 = samples[size_t(samples.size() * 9 / 10)];
    std::cout << "time " << name << " n=" << count << " min_us=" << min
              << " median_us=" << med << " p90_us=" << p90 << "\n";
}

uint64_t
PointChecksum(const rigExec::RigExecRuntimeReader &reader)
{
    uint64_t sum = 1469598103934665603ull;
    for (const auto &moved : reader.GetPoints()) {
        for (const auto &p : moved.points) {
            for (int k = 0; k < 3; ++k) {
                const float component = p[k];
                uint32_t bits = 0;
                std::memcpy(&bits, &component, sizeof(bits));
                sum ^= bits;
                sum *= 1099511628211ull;
            }
        }
    }
    return sum;
}

const char *
RevisionName(uint8_t op)
{
    switch (op) {
    case 0: return "Matrix";
    case 1: return "Skin";
    case 2: return "BlendShape";
    case 3: return "VolumeCorrect";
    case 4: return "Smooth";
    case 5: return "Lattice";
    case 6: return "SurfaceProject";
    case 7: return "Ribbon";
    case 8: return "Wire";
    case 9: return "EmitGuidePoints";
    case 13: return "RecomputeNormals";
    case 14: return "RecomputeExtent";
    case 15: return "DeltaMush";
    case 16: return "Wrinkle";
    case 17: return "External";
    case 18: return "SurfaceProjector";
    case 19: return "ShaderDials";
    default: return "Other";
    }
}

}  // namespace

int
main(int argc, char **argv)
{
    if (argc < 2) {
        std::cerr << "usage: probe_program file.rigexec\n";
        return 2;
    }
    std::ifstream in(argv[1], std::ios::binary);
    std::vector<uint8_t> bytes((std::istreambuf_iterator<char>(in)),
                               std::istreambuf_iterator<char>());
    if (!in && bytes.empty()) {
        std::cerr << "could not read " << argv[1] << "\n";
        return 1;
    }
    std::string error;
    std::unique_ptr<rigExec::fb::RigExecWireFile> file;
    if (!rigExec::RigExecFormatOpen(bytes.data(), bytes.size(), &file,
                                    &error)) {
        std::cout << "reader_open_refused " << error << "\n";
        std::cout << "execute skipped\n";
        return 0;
    }
    std::cout << "file bytes=" << bytes.size()
              << " format=" << file->formatVersion
              << " bakeTime=" << file->bakeTime << "\n";
    std::cout << "inputs listed=" << file->listedInputs
              << " slots=" << file->inputs.size()
              << " steps=" << file->steps.size()
              << " externalMovers=" << file->externalMovers.size() << "\n";

    std::map<std::string, int> kinds;
    int heads = 0, always = 0, varying = 0, sources = 0, volatileOps = 0;
    double stepCost = 0;
    int maxPred = 0;
    for (const auto &step : file->steps) {
        const char *name = rigExec::fb::EnumNameStepKind(step.kind);
        ++kinds[name && name[0] ? name : "Unknown"];
        heads += step.isHead ? 1 : 0;
        always += step.headAlwaysRuns ? 1 : 0;
        varying += step.varyingInputs ? 1 : 0;
        sources += step.isSource ? 1 : 0;
        maxPred = std::max(maxPred, int(step.preds.size()));
        stepCost += step.cost;
    }
    std::cout << "steps sources=" << sources << " heads=" << heads
              << " headAlwaysRuns=" << always
              << " varyingInputs=" << varying
              << " maxPreds=" << maxPred
              << " summedStepCost_us=" << stepCost << "\n";
    for (const auto &entry : kinds) {
        std::cout << "kind " << entry.first << " " << entry.second << "\n";
    }

    if (file->clustering) {
        const auto &c = *file->clustering;
        int members = 0, maxMembers = 0, singletons = 0;
        for (const auto &cluster : c.clusters) {
            members += int(cluster.members.size());
            maxMembers = std::max(maxMembers, int(cluster.members.size()));
            singletons += cluster.members.size() == 1 ? 1 : 0;
        }
        const auto levels = Levels(c.clusters.size(), [&](size_t i) {
            return c.clusters[i].preds;
        });
        std::cout << "stepClusters count=" << c.clusters.size()
                  << " members=" << members
                  << " singletons=" << singletons
                  << " maxMembers=" << maxMembers
                  << " grain_us=" << c.grainUs
                  << " serialCost_us=" << c.serialCost
                  << " criticalPath_us=" << c.criticalPathCost
                  << " modelSpeedup="
                  << (c.criticalPathCost > 0
                          ? c.serialCost / c.criticalPathCost
                          : 0)
                  << " levels=" << levels.levels
                  << " maxWidth=" << levels.maxWidth << "\n";
    }
    if (file->commonGraph) {
        const auto &g = *file->commonGraph;
        int singletons = 0, maxMembers = 0;
        int volatileCount = 0;
        for (const auto &op : g.ops) {
            volatileCount += op.volatileInput ? 1 : 0;
        }
        volatileOps = volatileCount;
        for (const auto &cluster : g.clusters) {
            maxMembers = std::max(maxMembers, int(cluster.members.size()));
            singletons += cluster.members.size() == 1 ? 1 : 0;
        }
        const auto levels = Levels(g.clusters.size(), [&](size_t i) {
            return g.clusters[i].predecessors;
        });
        std::cout << "commonGraph ops=" << g.ops.size()
                  << " values=" << g.valueSpecs.size()
                  << " leaves=" << g.leaves.size()
                  << " cycles=" << g.cycles.size()
                  << " longestPath=" << g.longestPath
                  << " clusters=" << g.clusters.size()
                  << " singletons=" << singletons
                  << " maxMembers=" << maxMembers
                  << " volatileOps=" << volatileOps
                  << " levels=" << levels.levels
                  << " maxWidth=" << levels.maxWidth << "\n";
    }

    int animated = 0;
    if (file->geometry) {
        std::map<std::string, int> ops;
        size_t points = 0;
        int chunked = 0, chunks = 0;
        for (const auto &chain : file->geometry->chains) {
            if (chain.haveBase && chain.base < file->vec3fArrays.size()) {
                points += file->vec3fArrays[chain.base].v.size();
            }
            for (const auto &rev : chain.revisions) {
                ++ops[RevisionName(rev.op)];
                chunked += rev.chunked ? 1 : 0;
                chunks += int(rev.chunks.size());
                std::cout << "revision op=" << RevisionName(rev.op)
                          << " influences=" << rev.influenceSlots.size()
                          << " chunked=" << rev.chunked
                          << " chunks=" << rev.chunks.size()
                          << " points=" << rev.partitionPointCount
                          << " elementSize=" << rev.partitionElementSize
                          << "\n";
            }
        }
        std::cout << "geometry chains=" << file->geometry->chains.size()
                  << " basePoints=" << points
                  << " chunkedRevisions=" << chunked
                  << " chunkSteps=" << chunks << "\n";
        for (const auto &entry : ops) {
            std::cout << "revisionKind " << entry.first << " "
                      << entry.second << "\n";
        }
    }
    for (const auto &input : file->inputs) {
        if (input.flags() & uint8_t(rigExec::fb::InputSlotFlags::Animated)) {
            ++animated;
        }
    }
    std::cout << "animatedInputSlots=" << animated << "\n";

    auto reader = rigExec::RigExecRuntimeReader::Open(bytes.data(),
                                                      bytes.size(), &error);
    if (!reader) {
        std::cerr << "runtime open failed: " << error << "\n";
        return 1;
    }
    std::cout << "runtime inputs=" << reader->GetInputCount()
              << " simd=" << reader->GetSimdEnabledForTesting() << "\n";
    int animatedListed = 0;
    size_t firstAnimated = size_t(-1);
    for (size_t i = 0; i < reader->GetInputCount(); ++i) {
        const auto &info = reader->GetInputInfo(i);
        if (info.animated) {
            ++animatedListed;
            if (firstAnimated == size_t(-1) &&
                (info.type == rigExec::RrInputTag::Double ||
                 info.type == rigExec::RrInputTag::Float)) {
                firstAnimated = i;
            }
        }
    }
    std::cout << "animatedListedInputs=" << animatedListed << "\n";

    auto run = [&] { return reader->Execute(&error); };
    if (!run()) {
        std::cerr << "execute failed: " << error << "\n";
        for (const auto &line : reader->GetDiagnostics()) {
            std::cerr << "diag " << line << "\n";
        }
        return 1;
    }
    std::cout << "cold ops=" << reader->GetCounters().executedOpCount
              << " trace=" << reader->GetLastRunTraceForTesting().size()
              << " closedClusters="
              << reader->GetClosedClusterCountForTesting()
              << " pointsOut=" << reader->GetPoints().size()
              << " joints=" << reader->GetJointMatrices().size()
              << " checksum=" << PointChecksum(*reader) << "\n";
    size_t pointCount = 0;
    for (const auto &moved : reader->GetPoints()) {
        pointCount += moved.points.size();
        std::cout << "moved " << moved.path << " points=" << moved.points.size()
                  << "\n";
    }
    std::cout << "movedPoints=" << pointCount << "\n";

    Time("warm_hold", 5, 40, [&] { run(); });
    std::cout << "warm_hold ops=" << reader->GetCounters().executedOpCount
              << " trace=" << reader->GetLastRunTraceForTesting().size()
              << " closedClusters="
              << reader->GetClosedClusterCountForTesting()
              << " checksum=" << PointChecksum(*reader) << "\n";

    Time("touch_animated", 5, 40, [&] {
        reader->TouchAnimatedInputs();
        run();
    });
    std::cout << "touch_animated ops=" << reader->GetCounters().executedOpCount
              << " trace=" << reader->GetLastRunTraceForTesting().size()
              << " closedClusters="
              << reader->GetClosedClusterCountForTesting() << "\n";

    if (firstAnimated != size_t(-1)) {
        const auto &info = reader->GetInputInfo(firstAnimated);
        rigExec::RrInputValue value = info.defaultValue;
        if (info.type == rigExec::RrInputTag::Double) {
            value.f64 += 0.25;
        } else {
            value.f32 += 0.25f;
        }
        if (!reader->SetInputAt(firstAnimated, value, &error)) {
            std::cerr << "set input failed: " << error << "\n";
            return 1;
        }
        Time("one_input", 5, 40, [&] { run(); });
        std::cout << "one_input name=" << info.name
                  << " ops=" << reader->GetCounters().executedOpCount
                  << " trace=" << reader->GetLastRunTraceForTesting().size()
                  << " closedClusters="
                  << reader->GetClosedClusterCountForTesting()
                  << " checksum=" << PointChecksum(*reader) << "\n";
    } else {
        std::cout << "one_input skipped\n";
    }
    return 0;
}
