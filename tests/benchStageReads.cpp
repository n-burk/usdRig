//
// SCRATCH BENCHMARK -- which USD reads stop scaling across threads.
//
// Compile runs several readers of one stage at once (the structure digest's
// prefetch lanes, the exec lane, the compiling thread), and they slow each
// other down: the Solvers prefetch on the biped takes ~12 ms alone and ~24 ms
// on one lane beside compile, and stops improving past four lanes. This
// times each kind of read the digest and PoseInfoPrefetch make, over the same
// prims, on 1..N threads, against a pure-CPU control that has no shared
// state, so a read whose speedup falls behind the control's is contending on
// something inside USD.
//
// Not a test: it asserts nothing and is not registered with ctest.
//
#include "pxr/base/tf/token.h"
#include "pxr/usd/sdf/attributeSpec.h"
#include "pxr/usd/sdf/path.h"
#include "pxr/usd/sdf/primSpec.h"
#include "pxr/usd/usd/attribute.h"
#include "pxr/usd/usd/prim.h"
#include "pxr/usd/usd/primDefinition.h"
#include "pxr/usd/usd/primRange.h"
#include "pxr/usd/usd/stage.h"

#include <algorithm>
#include <atomic>
#include <chrono>
#include <cstdio>
#include <functional>
#include <map>
#include <set>
#include <tuple>
#include <unordered_map>
#include <string>
#include <thread>
#include <vector>

PXR_NAMESPACE_USING_DIRECTIVE

namespace {

using Clock = std::chrono::steady_clock;

// Runs work(i) for every i in [0, n) on `threads` threads, strided, and
// returns the best wall time of `reps` runs in ms.
double
TimeParallel(size_t n, int threads, int reps,
             const std::function<size_t(size_t)> &work)
{
    double best = 1e30;
    std::atomic<size_t> sink{0};
    for (int r = 0; r < reps; ++r) {
        std::atomic<int> ready{0};
        std::atomic<bool> go{false};
        std::vector<std::thread> pool;
        std::vector<double> ends(size_t(threads));
        for (int t = 0; t < threads; ++t) {
            pool.emplace_back([&, t]() {
                ++ready;
                while (!go.load(std::memory_order_acquire)) {
                }
                size_t local = 0;
                for (size_t i = size_t(t); i < n; i += size_t(threads)) {
                    local += work(i);
                }
                sink += local;
            });
        }
        while (ready.load() != threads) {
        }
        const auto start = Clock::now();
        go.store(true, std::memory_order_release);
        for (std::thread &thread : pool) {
            thread.join();
        }
        best = std::min(
            best, std::chrono::duration<double, std::milli>(Clock::now() -
                                                            start)
                      .count());
    }
    if (sink.load() == 42) {
        std::printf(" ");  // keep the work observable
    }
    return best;
}

}  // namespace

int
main(int argc, char **argv)
{
    const std::string stagePath =
        argc > 1 ? argv[1] : "examples/biped/Biped_anim.usda";
    const std::string rigPath = argc > 2 ? argv[2] : "/Biped/Rig";
    UsdStageRefPtr stage = UsdStage::Open(stagePath);
    if (!stage) {
        std::printf("cannot open %s\n", stagePath.c_str());
        return 1;
    }
    const UsdPrim rig = stage->GetPrimAtPath(SdfPath(rigPath));
    std::vector<UsdPrim> prims;
    for (const UsdPrim &prim : UsdPrimRange(rig)) {
        prims.push_back(prim);
    }
    // Every attribute, for the per-attribute reads.
    std::vector<UsdAttribute> attrs;
    for (const UsdPrim &prim : prims) {
        for (const UsdAttribute &a : prim.GetAttributes()) {
            attrs.push_back(a);
        }
    }
    std::vector<SdfPath> attrPaths;
    for (const UsdAttribute &a : attrs) {
        attrPaths.push_back(a.GetPath());
    }
    std::printf("%s %s: %zu prims, %zu attributes\n", stagePath.c_str(),
                rigPath.c_str(), prims.size(), attrs.size());

    // ---- the digest's hop read, two ways ----------------------------------
    // One attribute as the digest records it.
    using Fact = std::tuple<SdfPath, TfToken, SdfPathVector>;
    const auto connectionsOf = [](const UsdAttribute &a) {
        SdfPathVector s;
        if (a.HasAuthoredConnections()) {
            a.GetConnections(&s);
        }
        return s;
    };
    // As today: every attribute USD lists, each read for its facts.
    const auto hopNow = [&](const UsdPrim &prim, std::vector<Fact> *out) {
        for (const UsdAttribute &a : prim.GetAttributes()) {
            out->emplace_back(a.GetPath(), a.GetTypeName().GetAsToken(),
                              connectionsOf(a));
        }
    };
    // The schema's attributes once per prim definition -- their type names
    // come from the definition, which is where USD takes them from
    // (_GetAttrTypeImpl), and an unauthored one has no connections -- and
    // per prim only what is authored.
    struct DefAttrs {
        std::vector<std::pair<TfToken, TfToken>> attrs;  // name, typeName
        std::set<TfToken> names;
    };
    std::unordered_map<const UsdPrimDefinition *, DefAttrs> defCache;
    for (const UsdPrim &prim : prims) {
        const UsdPrimDefinition *def = &prim.GetPrimDefinition();
        if (defCache.count(def)) continue;
        DefAttrs &d = defCache[def];
        for (const TfToken &name : def->GetPropertyNames()) {
            if (def->GetSpecType(name) == SdfSpecTypeAttribute) {
                d.attrs.emplace_back(
                    name,
                    def->GetAttributeDefinition(name).GetTypeName().GetAsToken());
                d.names.insert(name);
            }
        }
    }
    std::printf("%zu distinct prim definitions\n", defCache.size());
    const auto hopDef = [&](const UsdPrim &prim, std::vector<Fact> *out) {
        const DefAttrs &d = defCache.at(&prim.GetPrimDefinition());
        const SdfPath &primPath = prim.GetPath();
        const TfTokenVector authored = prim.GetAuthoredPropertyNames();
        std::set<TfToken> authoredSet(authored.begin(), authored.end());
        for (const auto &[name, typeName] : d.attrs) {
            const SdfPath path = primPath.AppendProperty(name);
            if (authoredSet.count(name)) {
                out->emplace_back(path, typeName,
                                  connectionsOf(prim.GetAttribute(name)));
            } else {
                out->emplace_back(path, typeName, SdfPathVector());
            }
        }
        for (const TfToken &name : authored) {
            if (d.names.count(name)) continue;
            const UsdAttribute a = prim.GetAttribute(name);
            if (!a) continue;  // a relationship
            out->emplace_back(a.GetPath(), a.GetTypeName().GetAsToken(),
                              connectionsOf(a));
        }
    };
    {
        size_t mismatched = 0, facts = 0;
        for (const UsdPrim &prim : prims) {
            std::vector<Fact> a, b;
            hopNow(prim, &a);
            hopDef(prim, &b);
            std::sort(a.begin(), a.end());
            std::sort(b.begin(), b.end());
            facts += a.size();
            if (a != b) {
                if (++mismatched <= 5) {
                    std::printf("  MISMATCH %s: %zu vs %zu facts\n",
                                prim.GetPath().GetText(), a.size(), b.size());
                }
            }
        }
        std::printf("hop equivalence: %zu facts, %zu prims differ\n", facts,
                    mismatched);
    }

    struct Case {
        const char *name;
        size_t n;
        std::function<size_t(size_t)> work;
    };
    const std::vector<Case> cases = {
        {"control: 20 us of hashing per prim", prims.size(),
         [](size_t i) {
             size_t h = i;
             for (int k = 0; k < 20000; ++k) {
                 h = h * 1099511628211ull ^ size_t(k);
             }
             return h;
         }},
        {"prim.GetAttributes()", prims.size(),
         [&](size_t i) { return prims[i].GetAttributes().size(); }},
        {"prim.GetPropertyNames()", prims.size(),
         [&](size_t i) { return prims[i].GetPropertyNames().size(); }},
        {"prim.GetAuthoredPropertyNames()", prims.size(),
         [&](size_t i) {
             return prims[i].GetAuthoredPropertyNames().size();
         }},
        {"attr.GetPath()", attrs.size(),
         [&](size_t i) { return size_t(attrs[i].GetPath().GetHash()); }},
        {"attr.GetTypeName()", attrs.size(),
         [&](size_t i) {
             return size_t(attrs[i].GetTypeName().GetAsToken().Hash());
         }},
        {"attr.HasAuthoredConnections()", attrs.size(),
         [&](size_t i) { return size_t(attrs[i].HasAuthoredConnections()); }},
        {"attr.GetConnections()", attrs.size(),
         [&](size_t i) {
             SdfPathVector s;
             attrs[i].GetConnections(&s);
             return s.size();
         }},
        {"stage->GetAttributeAtPath()", attrPaths.size(),
         [&](size_t i) {
             return size_t(bool(stage->GetAttributeAtPath(attrPaths[i])));
         }},
        {"stage->GetPrimAtPath()", prims.size(),
         [&](size_t i) {
             return size_t(bool(stage->GetPrimAtPath(prims[i].GetPath())));
         }},
        {"prim.GetTypeName()", prims.size(),
         [&](size_t i) { return size_t(prims[i].GetTypeName().Hash()); }},
        {"prim.GetPrimStack() + attr specs", prims.size(),
         [&](size_t i) {
             size_t k = 0;
             for (const SdfPrimSpecHandle &spec : prims[i].GetPrimStack()) {
                 for (const SdfAttributeSpecHandle &a : spec->GetAttributes()) {
                     k += a->HasConnectionPaths() ? 2 : 1;
                 }
             }
             return k;
         }},
        {"path.GetString() (interned)", attrPaths.size(),
         [&](size_t i) { return attrPaths[i].GetString().size(); }},
        {"path.GetAsString()", attrPaths.size(),
         [&](size_t i) { return attrPaths[i].GetAsString().size(); }},
        {"HOP now: GetAttributes + facts", prims.size(),
         [&](size_t i) {
             std::vector<Fact> out;
             hopNow(prims[i], &out);
             return out.size();
         }},
        {"HOP def cache + authored facts", prims.size(),
         [&](size_t i) {
             std::vector<Fact> out;
             hopDef(prims[i], &out);
             return out.size();
         }},
        {"TfToken(\"rigExec:joints\")", attrs.size(),
         [](size_t) { return TfToken("rigExec:joints").Hash(); }},
    };

    const std::vector<int> threadCounts = {1, 2, 4, 8, 16};
    std::printf("\n%-36s %8s", "ms (best of 7); speedup vs 1 thread", "n");
    for (int t : threadCounts) {
        std::printf("   %2dT ms  (x)  ", t);
    }
    std::printf("\n");
    for (const Case &c : cases) {
        // One untimed pass: every lazily built cache is warm, which is the
        // state compile's second and later readers find.
        TimeParallel(c.n, 1, 1, c.work);
        std::printf("%-36s %8zu", c.name, c.n);
        double one = 0;
        for (int t : threadCounts) {
            const double ms = TimeParallel(c.n, t, 7, c.work);
            if (t == 1) {
                one = ms;
            }
            std::printf("   %7.2f (%4.1f)", ms, one / ms);
        }
        std::printf("\n");
    }
    return 0;
}
