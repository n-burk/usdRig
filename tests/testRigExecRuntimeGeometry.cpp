//
// rigExecRuntime geometry-family parity (M2): baked program vs runtime
// over every baking fixture, comparing published chain points bit for
// bit. Owned by the geometry-port worker.
//

#include "rigExecBake/bake.h"
#include "rigExec/rigEvaluator.h"
#include "rigExecRuntime/runtime.h"
#include "rigExecExampleFixtures.h"

#include "pxr/base/plug/registry.h"
#include "pxr/usd/usd/primRange.h"
#include "pxr/usd/usd/stage.h"

#include <algorithm>
#include <cstdio>
#include <cstring>
#include <string>
#include <utility>
#include <vector>

using namespace rigExec;
PXR_NAMESPACE_USING_DIRECTIVE

static int failures = 0;
static int blockedFixtures = 0;
static int comparedFixtures = 0;

#define CHECK(cond)                                                     \
    do {                                                                \
        if (!(cond)) {                                                  \
            ++failures;                                                 \
            std::printf("FAIL %s:%d: %s\n", __FILE__, __LINE__, #cond);  \
        }                                                               \
    } while (0)

static SdfPath
_FindRig(const UsdStageRefPtr &stage)
{
    for (const UsdPrim &prim : stage->TraverseAll()) {
        if (prim.GetTypeName() == TfToken("RigExecRoot")) {
            return prim.GetPath();
        }
    }
    return SdfPath();
}

static std::vector<double>
_ParseFrames(const std::string &text)
{
    std::vector<double> frames;
    size_t begin = 0;
    while (begin <= text.size()) {
        const size_t end = text.find(",", begin);
        const std::string piece =
            text.substr(begin, end == std::string::npos
                        ? std::string::npos : end - begin);
        if (!piece.empty()) {
            frames.push_back(std::stod(piece));
        }
        if (end == std::string::npos) {
            break;
        }
        begin = end + 1;
    }
    return frames;
}

// The baked epilogue's summary line is the only place chainsBuilt and
// revisionsBuilt are published; the runtime reader emits the same line.
// Parse its five numbers so the counters are checked against it too.
static bool
_ParseSummary(const std::string &line, size_t *chains, size_t *revisions,
              size_t *created, size_t *executed, size_t *schedules)
{
    size_t values[5] = {0, 0, 0, 0, 0};
    int matched = 0;
#ifdef _MSC_VER
#pragma warning(push)
#pragma warning(disable : 4996)
#endif
    matched = std::sscanf(line.c_str(),
                           "mover graph: %zu chain(s), %zu revision(s); "
                           "%zu created, %zu executed, %zu schedule(s) built",
                           &values[0], &values[1], &values[2], &values[3],
                           &values[4]);
#ifdef _MSC_VER
#pragma warning(pop)
#endif
    *chains = values[0];
    *revisions = values[1];
    *created = values[2];
    *executed = values[3];
    *schedules = values[4];
    return matched == 5;
}

static void
_TestFixture(const std::string &name, const std::string &stagePath,
             const std::vector<double> &bakeFrames)
{
    const UsdStageRefPtr stage = UsdStage::Open(stagePath);
    CHECK(stage);
    if (!stage) {
        std::printf("%s: FAILED (no stage)\n", name.c_str());
        return;
    }
    const SdfPath rigPath = _FindRig(stage);
    CHECK(!rigPath.IsEmpty());
    if (rigPath.IsEmpty()) {
        std::printf("%s: FAILED (no rig)\n", name.c_str());
        return;
    }
    RigExecRigEvaluator evaluator(stage, rigPath);
    evaluator.SetEvaluationMode(RigExecEvaluationMode::Baked);
    RigExecBakeOpts opts;
    opts.frames = bakeFrames;
    RigExecBakeResult result;
    std::string error;
    CHECK(RigExecBakeToBinary(evaluator, opts, &result, &error));
    if (!error.empty()) {
        std::printf("bake diagnostic: %s\n", error.c_str());
    }
    CHECK(!result.bytes.empty());
    if (result.bytes.empty()) {
        std::printf("%s: FAILED (no bytes)\n", name.c_str());
        return;
    }

    std::unique_ptr<RigExecRuntimeReader> reader =
        RigExecRuntimeReader::Open(result.bytes.data(), result.bytes.size(),
                                   &error);
    CHECK(reader);
    if (!reader) {
        std::printf("open diagnostic: %s\n", error.c_str());
        std::printf("%s: FAILED (open)\n", name.c_str());
        return;
    }
    reader->SetRunMaskForTesting(0x7u);

    bool blocked = false;
    bool failed = false;
    int comparedFrames = 0;
    // The bake evaluated every frame before this comparison runs, so the
    // baked program's one-shot state (created flags, sticky statuses) is
    // past its first frame. Run the reader through the same history
    // first, uncompared, so both sides evaluate each compared frame
    // from the same prior state.
    for (double frame : bakeFrames) {
        if (!reader->SetFrame(frame, &error) ||
            !reader->Execute(&error)) {
            if (error.find("not implemented yet") != std::string::npos) {
                blocked = true;
                break;
            }
            std::printf("warmup diagnostic at %s frame %.17g: %s\n",
                        name.c_str(), frame, error.c_str());
            CHECK(false);
            failed = true;
            break;
        }
    }
    if (blocked || failed) {
        if (blocked) {
            ++blockedFixtures;
            std::printf("%s: blocked-on-family\n", name.c_str());
        } else {
            std::printf("%s: FAILED\n", name.c_str());
        }
        return;
    }
    for (double frame : bakeFrames) {
        const RigExecRigPose pose = evaluator.Evaluate(frame);
        if (!pose.valid) {
            CHECK(false);
            failed = true;
            break;
        }
        if (!reader->SetFrame(frame, &error)) {
            std::printf("setframe diagnostic: %s\n", error.c_str());
            CHECK(false);
            failed = true;
            break;
        }
        if (!reader->Execute(&error)) {
            // Another family still landing: the integrator re-runs this
            // fixture when all families are in.
            if (error.find("not implemented yet") != std::string::npos) {
                blocked = true;
                break;
            }
            std::printf("execute diagnostic at %s frame %.17g: %s\n",
                        name.c_str(), frame, error.c_str());
            CHECK(false);
            failed = true;
            break;
        }

        // Points: path plus bitwise points per moved property. Both
        // sides sorted by path string: the reader publishes in string
        // id order, not path order, so position only lines up after
        // the sort. (Framework need: publish GetPoints in path order
        // as runtime.h documents.)
        const std::vector<RigExecRuntimePoints> &readerPoints =
            reader->GetPoints();
        std::vector<const RigExecRuntimePoints *> points;
        points.reserve(readerPoints.size());
        for (const RigExecRuntimePoints &moved : readerPoints) {
            points.push_back(&moved);
        }
        std::sort(points.begin(), points.end(),
                  [](const RigExecRuntimePoints *a,
                     const RigExecRuntimePoints *b) {
                      return a->path < b->path;
                  });
        // Geometry owns the point-array entries; scalar property
        // results (inputs:weight, rigExec:gain, ...) are the pose
        // family's publication, compared by its own test.
        std::vector<std::pair<std::string, VtValue>> wantPoints;
        wantPoints.reserve(pose.movedProperties.size());
        for (const auto &entry : pose.movedProperties) {
            if (!entry.second.IsHolding<VtVec3fArray>()) {
                continue;
            }
            wantPoints.push_back(
                {entry.first.GetString(), entry.second});
        }
        std::sort(wantPoints.begin(), wantPoints.end(),
                  [](const std::pair<std::string, VtValue> &a,
                     const std::pair<std::string, VtValue> &b) {
                      return a.first < b.first;
                  });
        if (points.size() != wantPoints.size()) {
            std::printf("%s frame %.17g: %zu moved properties vs %zu\n",
                        name.c_str(), frame, points.size(),
                        wantPoints.size());
            for (const auto *moved : points) {
                std::printf("  got:  %s (%zu points)\n",
                            moved->path.c_str(), moved->points.size());
            }
            for (const auto &entry : wantPoints) {
                std::printf("  want: %s\n", entry.first.c_str());
            }
            CHECK(false);
            failed = true;
            continue;
        }
        for (size_t at = 0; at < wantPoints.size(); ++at) {
            const std::string &path = wantPoints[at].first;
            if (points[at]->path != path) {
                std::printf("%s frame %.17g: moved property %zu is %s, "
                            "expected %s\n", name.c_str(), frame, at,
                            points[at]->path.c_str(), path.c_str());
                CHECK(false);
                failed = true;
                break;
            }
            if (!wantPoints[at].second.IsHolding<VtVec3fArray>()) {
                std::printf("%s frame %.17g: %s holds %s, not points\n",
                            name.c_str(), frame, path.c_str(),
                            wantPoints[at]
                                .second.GetTypeName()
                                .c_str());
                CHECK(false);
                failed = true;
                break;
            }
            const VtVec3fArray &want =
                wantPoints[at].second.UncheckedGet<VtVec3fArray>();
            const std::vector<RrVec3f> &got = points[at]->points;
            if (got.size() != want.size() ||
                (want.size() > 0 &&
                 std::memcmp(got.data(), want.cdata(),
                             want.size() * sizeof(GfVec3f)) != 0)) {
                size_t firstDiff = 0;
                while (firstDiff < got.size() &&
                       firstDiff < want.size() &&
                       std::memcmp(&got[firstDiff], &want[firstDiff],
                                   sizeof(GfVec3f)) == 0) {
                    ++firstDiff;
                }
                std::printf("%s frame %.17g: %s points differ "
                            "(%zu vs %zu, first at %zu)\n", name.c_str(),
                            frame, path.c_str(), got.size(), want.size(),
                            firstDiff);
                if (firstDiff < got.size() && firstDiff < want.size()) {
                    std::printf("  got:  %.9g %.9g %.9g\n  want: %.9g "
                                "%.9g %.9g\n", got[firstDiff][0],
                                got[firstDiff][1], got[firstDiff][2],
                                want[firstDiff][0], want[firstDiff][1],
                                want[firstDiff][2]);
                }
                CHECK(false);
                failed = true;
                break;
            }
        }

        // Diagnostics: verbatim, summary line included. The summary's
        // five numbers are also checked against the counters below.
        const std::vector<std::string> &diagnostics =
            reader->GetDiagnostics();
        const std::vector<std::string> &wantDiags = pose.diagnostics;
        size_t wantChains = 0, wantRevisions = 0, wantCreated = 0;
        size_t wantExecuted = 0, wantSchedules = 0;
        bool haveSummary = false;
        if (!wantDiags.empty() &&
            wantDiags.back().compare(0, 12, "mover graph:") == 0) {
            haveSummary = _ParseSummary(
                wantDiags.back(), &wantChains, &wantRevisions,
                &wantCreated, &wantExecuted, &wantSchedules);
            if (!haveSummary) {
                std::printf("%s frame %.17g: unparsable summary: %s\n",
                            name.c_str(), frame,
                            wantDiags.back().c_str());
                CHECK(false);
                failed = true;
            }
        } else {
            std::printf("%s frame %.17g: no summary line\n", name.c_str(),
                        frame);
            CHECK(false);
            failed = true;
        }
        if (diagnostics != wantDiags) {
            std::printf("%s frame %.17g: diagnostics differ "
                        "(%zu vs %zu)\n", name.c_str(), frame,
                        diagnostics.size(), wantDiags.size());
            const size_t common =
                std::min(diagnostics.size(), wantDiags.size());
            for (size_t i = 0; i < common; ++i) {
                if (diagnostics[i] != wantDiags[i]) {
                    std::printf("  got:  %s\n  want: %s\n",
                                diagnostics[i].c_str(),
                                wantDiags[i].c_str());
                    break;
                }
            }
            if (diagnostics.size() > common) {
                std::printf("  extra got:  %s\n",
                            diagnostics[common].c_str());
            }
            if (wantDiags.size() > common) {
                std::printf("  extra want: %s\n",
                            wantDiags[common].c_str());
            }
            CHECK(false);
            failed = true;
        }

        // Counters: the pose's three plus the summary's five.
        const RigExecRuntimeCounters counters = reader->GetCounters();
        if (counters.revisionsExecuted !=
                pose.moverGraphRevisionsExecuted ||
            counters.revisionsCreated !=
                pose.moverGraphRevisionsCreated ||
            counters.schedulesBuilt != pose.moverGraphSchedulesBuilt) {
            std::printf("%s frame %.17g: counters %u/%u/%u vs %zu/%zu/%zu\n",
                        name.c_str(), frame, counters.revisionsCreated,
                        counters.revisionsExecuted, counters.schedulesBuilt,
                        pose.moverGraphRevisionsCreated,
                        pose.moverGraphRevisionsExecuted,
                        pose.moverGraphSchedulesBuilt);
            CHECK(false);
            failed = true;
        }
        if (haveSummary &&
            (counters.chainsBuilt != wantChains ||
             counters.revisionsBuilt != wantRevisions ||
             counters.revisionsCreated != wantCreated ||
             counters.revisionsExecuted != wantExecuted ||
             counters.schedulesBuilt != wantSchedules)) {
            std::printf("%s frame %.17g: counters vs summary differ\n",
                        name.c_str(), frame);
            CHECK(false);
            failed = true;
        }
        ++comparedFrames;
    }

    if (failed) {
        std::printf("%s: FAILED\n", name.c_str());
    } else if (blocked) {
        ++blockedFixtures;
        std::printf("%s: blocked-on-family\n", name.c_str());
    } else {
        ++comparedFixtures;
        std::printf("%s: compared %d frames\n", name.c_str(),
                    comparedFrames);
    }
}

int
main(int argc, char **argv)
{
    std::setvbuf(stdout, nullptr, _IONBF, 0);
    PlugRegistry::GetInstance().RegisterPlugins(
        RIGEXEC_SCHEMA_RESOURCE_DIR);

    std::string examplesDir = RIGEXEC_EXAMPLES_DIR;
    if (argc > 1) {
        examplesDir = argv[1];
    }
    const std::string only = argc > 2 ? argv[2] : "";
    bool sawBaking = false;
    for (const RigExecExampleFixture &fixture : kRigExecExampleFixtures) {
        if (!fixture.bakesToday) {
            continue;
        }
        if (!only.empty() &&
            std::string(fixture.stage).find(only) == std::string::npos) {
            continue;
        }
        sawBaking = true;
        const std::string stagePath =
            examplesDir + "/" + fixture.stage;
        const std::vector<double> frames = _ParseFrames(fixture.frames);
        CHECK(!frames.empty());
        if (frames.empty()) {
            continue;
        }
        _TestFixture(fixture.stage, stagePath, frames);
    }
    CHECK(sawBaking);

    if (failures == 0) {
        std::printf("testRigExecRuntimeGeometry: all tests passed "
                    "(%d compared, %d blocked-on-family)\n",
                    comparedFixtures, blockedFixtures);
        return 0;
    }
    std::printf("testRigExecRuntimeGeometry: %d failures "
                "(%d compared, %d blocked-on-family)\n", failures,
                comparedFixtures, blockedFixtures);
    return 1;
}
