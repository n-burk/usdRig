//
// SCRATCH BENCHMARK -- what the compile's bookkeeping actually costs.
//
// Three questions the compile trace raises but cannot answer:
//   1. How dear is UsdStage::GetPrimAtPath, really? 92 call sites in
//      rigEvaluator.cpp resolve paths in inner loops; is caching the prim
//      worth a map, or is the stage's own lookup already a hash hit?
//   2. How much does std::map<SdfPath, ...> cost against a hash map? 132
//      path-keyed node containers are declared in that one file.
//
// Not a test: it asserts nothing and is not registered with ctest.
//
#include "pxr/base/tf/hashmap.h"
#include "pxr/usd/sdf/path.h"
#include "pxr/usd/usd/prim.h"
#include "pxr/usd/usd/primRange.h"
#include "pxr/usd/usd/stage.h"

#include <chrono>
#include <cstdio>
#include <map>
#include <set>
#include <string>
#include <unordered_map>
#include <vector>

PXR_NAMESPACE_USING_DIRECTIVE

namespace {

using Clock = std::chrono::steady_clock;

double
MsSince(const Clock::time_point &start)
{
    return std::chrono::duration<double, std::milli>(Clock::now() - start)
        .count();
}

void
Report(const char *what, double ms, size_t n)
{
    std::printf("  %-46s %8.2f ms   %7.3f us/op\n", what, ms,
                n ? ms * 1000.0 / double(n) : 0.0);
}

}  // namespace

int
main(int argc, char **argv)
{
    const std::string stagePath =
        argc > 1 ? argv[1]
                 : "examples/biped/Biped_stack_anim.usda";
    UsdStageRefPtr stage = UsdStage::Open(stagePath);
    if (!stage) {
        std::printf("cannot open %s\n", stagePath.c_str());
        return 1;
    }

    std::vector<SdfPath> paths;
    for (const UsdPrim &prim : stage->Traverse()) {
        paths.push_back(prim.GetPath());
    }
    std::printf("%s: %zu prims\n\n", stagePath.c_str(), paths.size());

    // The compile resolves the same paths over and over, so every pass here
    // walks the whole set REPEATS times rather than once.
    const int repeats = 20;
    const size_t ops = paths.size() * size_t(repeats);

    // 1. The call the compile actually makes.
    {
        auto start = Clock::now();
        size_t live = 0;
        for (int r = 0; r < repeats; ++r) {
            for (const SdfPath &path : paths) {
                live += stage->GetPrimAtPath(path) ? 1 : 0;
            }
        }
        Report("UsdStage::GetPrimAtPath", MsSince(start), ops);
        if (live == 0) std::printf("   (no prims?)\n");
    }

    // 2. The same answer out of a hash map filled once.
    {
        TfHashMap<SdfPath, UsdPrim, SdfPath::Hash> cache;
        auto build = Clock::now();
        for (const UsdPrim &prim : stage->Traverse()) {
            cache[prim.GetPath()] = prim;
        }
        const double buildMs = MsSince(build);
        auto start = Clock::now();
        size_t live = 0;
        for (int r = 0; r < repeats; ++r) {
            for (const SdfPath &path : paths) {
                const auto it = cache.find(path);
                live += it != cache.end() ? 1 : 0;
            }
        }
        Report("TfHashMap<SdfPath, UsdPrim> lookup", MsSince(start), ops);
        std::printf("  %-46s %8.2f ms   (one traversal)\n",
                    "  ...its build cost", buildMs);
        if (live == 0) std::printf("   (empty?)\n");
    }

    // 3. Node-based vs hash containers, the compile's other habit.
    {
        auto start = Clock::now();
        std::map<SdfPath, int> ordered;
        for (int r = 0; r < repeats; ++r) {
            for (size_t i = 0; i < paths.size(); ++i) {
                ordered[paths[i]] = int(i);
            }
        }
        Report("std::map<SdfPath,int> insert/assign", MsSince(start), ops);
    }
    {
        auto start = Clock::now();
        std::unordered_map<SdfPath, int, SdfPath::Hash> hashed;
        for (int r = 0; r < repeats; ++r) {
            for (size_t i = 0; i < paths.size(); ++i) {
                hashed[paths[i]] = int(i);
            }
        }
        Report("unordered_map<SdfPath,int> insert/assign", MsSince(start),
               ops);
    }
    {
        std::map<SdfPath, int> ordered;
        for (size_t i = 0; i < paths.size(); ++i) ordered[paths[i]] = int(i);
        auto start = Clock::now();
        size_t live = 0;
        for (int r = 0; r < repeats; ++r) {
            for (const SdfPath &path : paths) {
                live += ordered.count(path);
            }
        }
        Report("std::map<SdfPath,int> lookup", MsSince(start), ops);
        if (live == 0) std::printf("   (empty?)\n");
    }
    {
        std::unordered_map<SdfPath, int, SdfPath::Hash> hashed;
        for (size_t i = 0; i < paths.size(); ++i) hashed[paths[i]] = int(i);
        auto start = Clock::now();
        size_t live = 0;
        for (int r = 0; r < repeats; ++r) {
            for (const SdfPath &path : paths) {
                live += hashed.count(path);
            }
        }
        Report("unordered_map<SdfPath,int> lookup", MsSince(start), ops);
        if (live == 0) std::printf("   (empty?)\n");
    }

    // 5. The ancestor walk the closure passes do per call.
    {
        auto start = Clock::now();
        size_t steps = 0;
        for (int r = 0; r < repeats; ++r) {
            for (const SdfPath &path : paths) {
                for (SdfPath p = path;
                     !p.IsEmpty() && p != SdfPath::AbsoluteRootPath();
                     p = p.GetParentPath()) {
                    ++steps;
                }
            }
        }
        Report("SdfPath ancestor walk to root", MsSince(start), ops);
        std::printf("  %-46s %zu\n", "  ...average depth",
                    steps / ops);
    }

    return 0;
}
