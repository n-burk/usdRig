//
// The baked program's step graph must describe the straight line it replaced.
//
// Every other suite asserts on what a frame PUBLISHES, which the serial
// executor gets right by running the steps in program order whatever the
// edges say. That is exactly why the edges need a test of their own: they are
// the part of this design nothing else exercises yet, and the first thing the
// parallel executor will trust. So this suite asks the four questions whose
// wrong answer makes a parallel run wrong while leaving a serial run
// perfect:
//
//   * every edge points FORWARD in program order, so running the steps in
//     index order is always a topological order;
//   * every declared read either has a writer earlier in the program or names
//     a domain the prologue fills, so nothing reads a slot the graph never
//     produced;
//   * two steps that declare overlapping WRITES are ordered with respect to
//     each other, so no schedule can run them at once;
//   * the report is deterministic, so a schedule can be diffed between two
//     builds of the same stage and a change in it is a change someone made.
//
// argv[1] = path to the examples directory (containing biped/Biped.usda).
//
#include "rigExec/bakedProgram.h"
#include "rigExec/bakedProgramImpl.h"
#include "rigExec/bakedSchedule.h"
#include "rigExec/parallel.h"
#include "rigExec/rigEvaluator.h"

#include "pxr/base/gf/vec3f.h"
#include "pxr/base/plug/registry.h"
#include "pxr/base/tf/getenv.h"
#include "pxr/base/tf/pathUtils.h"
#include "pxr/base/vt/array.h"
#include "pxr/usd/usd/prim.h"
#include "pxr/usd/usd/primRange.h"
#include "pxr/usd/usd/relationship.h"
#include "pxr/usd/usd/stage.h"

#include <algorithm>
#include <cstdint>
#include <cstdio>
#include <memory>
#include <string>
#include <vector>

using namespace rigExec;

PXR_NAMESPACE_USING_DIRECTIVE

static int failures = 0;

#define CHECK(cond)                                                        \
    do {                                                                   \
        if (!(cond)) {                                                     \
            ++failures;                                                    \
            std::printf("FAIL %s:%d: %s\n", __FILE__, __LINE__, #cond);    \
        }                                                                  \
    } while (0)

static SdfPath
FindRig(const UsdStageRefPtr &stage)
{
    for (const UsdPrim &prim : stage->Traverse()) {
        if (prim.GetTypeName() == "RigExecRoot") {
            return prim.GetPath();
        }
    }
    return SdfPath();
}

namespace {

/// One compiled, baked rig, kept alive together with the stage and evaluator
/// the program points into.
struct BuiltProgram {
    UsdStageRefPtr stage;
    std::unique_ptr<RigExecRigEvaluator> evaluator;
    std::unique_ptr<RigExecBakedProgram> program;
};

BuiltProgram
BuildStage(const UsdStageRefPtr &stage)
{
    BuiltProgram built;
    built.stage = stage;
    if (!built.stage) {
        return built;
    }
    const SdfPath rigPath = FindRig(built.stage);
    if (rigPath.IsEmpty()) {
        return built;
    }
    built.evaluator =
        std::make_unique<RigExecRigEvaluator>(built.stage, rigPath);
    std::vector<std::string> errors;
    if (!built.evaluator->Compile(&errors)) {
        return built;
    }
    std::vector<std::string> reasons;
    built.program = RigExecBakedProgram::Build(built.evaluator.get(),
                                               &reasons);
    if (!built.program) {
        for (const std::string &reason : reasons) {
            std::printf("    not bakeable: %s\n", reason.c_str());
        }
    }
    return built;
}

BuiltProgram
Build(const std::string &stagePath)
{
    return BuildStage(UsdStage::Open(stagePath));
}

/// A rig whose chain carries THREE revisions, which none of the example
/// stages does: the biped's two chains hold one revision each, so the rules
/// about reading a chain's running value are vacuous on them.
///
/// Three matrix movers on one points attribute is the shape
/// tests/testRigExecInteractive drives, and it is the shape that found the
/// missing RevisionDone declaration under the parallel executor. Built here
/// so the declaration is checked by a test rather than by remembering to run
/// one environment.
UsdStageRefPtr
MakeStackedChainStage()
{
    const UsdStageRefPtr stage = UsdStage::CreateInMemory();
    stage->DefinePrim(SdfPath("/Asset"), TfToken("Scope"));
    stage->DefinePrim(SdfPath("/Asset/Rig"), TfToken("RigExecRoot"));
    const UsdPrim driver = stage->DefinePrim(
        SdfPath("/Asset/Rig/Driver"), TfToken("RigExecControl"));
    driver.GetAttribute(TfToken("avars:tx")).Set(1.0);
    const SdfPath target("/Asset/Shape.points");
    const UsdPrim shape =
        stage->DefinePrim(target.GetPrimPath(), TfToken("Points"));
    shape.GetAttribute(TfToken("points"))
        .Set(VtVec3fArray{GfVec3f(0), GfVec3f(1, 0, 0)});
    shape.GetAttribute(TfToken("extent"))
        .Set(VtVec3fArray{GfVec3f(0), GfVec3f(1, 0, 0)});
    stage->DefinePrim(SdfPath("/Asset/Rig/Movers"), TfToken("Scope"));
    for (int i = 0; i < 3; ++i) {
        const UsdPrim mover = stage->DefinePrim(
            SdfPath("/Asset/Rig/Movers/M" + std::to_string(i)),
            TfToken("RigExecMatrixMover"));
        mover.ApplyAPI(TfToken("RigExecMoverAPI"));
        mover.GetRelationship(TfToken("rigExec:moves")).SetTargets({target});
        mover.GetRelationship(TfToken("rigExec:transform"))
            .SetTargets({driver.GetPath()});
        mover.GetAttribute(TfToken("inputs:defaultWeight")).Set(1.0f);
    }
    return stage;
}

/// A dense bitset over steps, for the reachability the write-conflict check
/// needs: two steps that write the same slot must be ordered, and the edge
/// that orders them is often two hops away because a third step wrote
/// between them.
class Reachability {
public:
    explicit Reachability(const std::vector<RigExecBakedStep> &steps)
        : _words((steps.size() + 63) / 64), _bits(steps.size() * _words, 0)
    {
        // Program order is a topological order, so one forward pass closes
        // the relation: a step reaches whatever its predecessors reached,
        // plus the predecessors themselves.
        for (size_t index = 0; index < steps.size(); ++index) {
            uint64_t *row = &_bits[index * _words];
            for (const int pred : steps[index].preds) {
                row[size_t(pred) / 64] |= uint64_t(1) << (size_t(pred) % 64);
                const uint64_t *predecessor = &_bits[size_t(pred) * _words];
                for (size_t w = 0; w < _words; ++w) {
                    row[w] |= predecessor[w];
                }
            }
        }
    }

    bool Ordered(size_t earlier, size_t later) const
    {
        return (_bits[later * _words + earlier / 64] >>
                (earlier % 64)) & uint64_t(1);
    }

private:
    size_t _words;
    std::vector<uint64_t> _bits;
};

void
TestTheGraphDescribesTheProgram(const BuiltProgram &built, const char *name)
{
    CHECK(built.program != nullptr);
    if (!built.program) {
        return;
    }
    const RigExecBakedProgramImpl &B = built.program->GetStepGraph();
    CHECK(!B.steps.empty());

    // (1) Every edge points forward, and the identity permutation is
    // therefore a topological order -- which is what "the serial executor
    // runs the steps in program order" is allowed to mean.
    size_t edges = 0;
    for (size_t index = 0; index < B.steps.size(); ++index) {
        for (const int pred : B.steps[index].preds) {
            ++edges;
            if (pred < 0 || size_t(pred) >= index) {
                ++failures;
                std::printf("FAIL %s: step %zu (%s) depends on step %d, "
                            "which is not earlier in program order\n",
                            name, index, B.steps[index].label.c_str(), pred);
            }
        }
        // preds and succs describe the same relation.
        for (const int succ : B.steps[index].succs) {
            if (succ < 0 || size_t(succ) <= index) {
                ++failures;
                std::printf("FAIL %s: step %zu names successor %d\n", name,
                            index, succ);
            }
        }
    }
    CHECK(edges > 0);

    // (2) Every declared read is produced by something: an earlier step, or
    // the prologue.
    for (size_t index = 0; index < B.steps.size(); ++index) {
        for (const RigExecBakedSlotRange &read : B.steps[index].reads) {
            if (RigExecBakedIsSourceDomain(read.domain) || read.IsEmpty()) {
                continue;
            }
            bool written = false;
            for (size_t earlier = 0; earlier < index && !written; ++earlier) {
                for (const RigExecBakedSlotRange &write :
                         B.steps[earlier].writes) {
                    if (write.Overlaps(read)) {
                        written = true;
                        break;
                    }
                }
            }
            if (!written) {
                ++failures;
                std::printf("FAIL %s: step %zu (%s) reads %s[%u,%u) which no "
                            "earlier step writes\n", name, index,
                            B.steps[index].label.c_str(),
                            RigExecBakedSlotDomainName(read.domain),
                            read.begin, read.end);
            }
        }
    }

    // (3) Two steps whose declared writes overlap are ordered. A declared
    // write is an upper bound, so this is stricter than what any run does --
    // which is the point: a scheduler may only look at the declarations.
    const Reachability reachable(B.steps);
    size_t conflicts = 0;
    for (size_t earlier = 0; earlier < B.steps.size(); ++earlier) {
        for (size_t later = earlier + 1; later < B.steps.size(); ++later) {
            if (reachable.Ordered(earlier, later)) {
                continue;
            }
            for (const RigExecBakedSlotRange &a : B.steps[earlier].writes) {
                for (const RigExecBakedSlotRange &b : B.steps[later].writes) {
                    if (!a.Overlaps(b)) {
                        continue;
                    }
                    if (++conflicts <= 4) {
                        std::printf("FAIL %s: steps %zu (%s) and %zu (%s) "
                                    "both write %s[%u,%u) with no edge "
                                    "between them\n", name, earlier,
                                    B.steps[earlier].label.c_str(), later,
                                    B.steps[later].label.c_str(),
                                    RigExecBakedSlotDomainName(a.domain),
                                    b.begin, b.end);
                    }
                }
            }
        }
    }
    failures += int(conflicts);

    // (4) And a step that reads a slot an unordered step writes is the same
    // race seen from the other side.
    size_t races = 0;
    for (size_t earlier = 0; earlier < B.steps.size(); ++earlier) {
        for (size_t later = earlier + 1; later < B.steps.size(); ++later) {
            if (reachable.Ordered(earlier, later)) {
                continue;
            }
            for (const RigExecBakedSlotRange &write :
                     B.steps[earlier].writes) {
                for (const RigExecBakedSlotRange &read :
                         B.steps[later].reads) {
                    if (write.Overlaps(read)) {
                        ++races;
                    }
                }
            }
            for (const RigExecBakedSlotRange &read : B.steps[earlier].reads) {
                for (const RigExecBakedSlotRange &write :
                         B.steps[later].writes) {
                    if (write.Overlaps(read)) {
                        ++races;
                    }
                }
            }
        }
    }
    if (races) {
        ++failures;
        std::printf("FAIL %s: %zu unordered read/write slot overlap(s)\n",
                    name, races);
    }
    // (5) A step that reads a revision's OUTPUT buffer must also read that
    // revision's RevisionDone slot. A chain's running points live in
    // whichever buffer the last applied revision filled, and the fuse
    // publishes that choice -- the `currentSource` indirection -- into
    // RevisionDone. A step declaring the buffers alone is free to run before
    // the fuse that decides which of them to read, so it reads the wrong
    // one: deterministically wrong points, invisible in every serial order
    // and immediate at one cluster per step. The rule is asserted here
    // rather than left to the builder because the builder is the file the
    // vertex partition rewrites, and a declaration dropped in that rewrite
    // must fail a test rather than wait for someone to re-run the dumps at
    // grain 0. A revision this step writes RevisionDone for is its own fuse
    // deciding the indirection, and is excluded.
    const size_t revisions = B.revisionIndex.size();
    auto mark = [revisions](std::vector<bool> *flags,
                            const RigExecBakedSlotRange &range) {
        for (size_t slot = range.begin;
             slot < range.end && slot < revisions; ++slot) {
            (*flags)[slot] = true;
        }
    };
    std::vector<bool> readsOut(revisions), readsDone(revisions),
        writesDone(revisions);
    for (size_t index = 0; index < B.steps.size(); ++index) {
        const RigExecBakedStep &step = B.steps[index];
        readsOut.assign(revisions, false);
        readsDone.assign(revisions, false);
        writesDone.assign(revisions, false);
        for (const RigExecBakedSlotRange &read : step.reads) {
            if (read.domain == RigExecBakedSlotDomain::RevisionOut) {
                mark(&readsOut, read);
            } else if (read.domain == RigExecBakedSlotDomain::RevisionDone) {
                mark(&readsDone, read);
            }
        }
        for (const RigExecBakedSlotRange &write : step.writes) {
            if (write.domain == RigExecBakedSlotDomain::RevisionDone) {
                mark(&writesDone, write);
            }
        }
        for (size_t revision = 0; revision < revisions; ++revision) {
            if (!readsOut[revision] || readsDone[revision] ||
                writesDone[revision]) {
                continue;
            }
            ++failures;
            std::printf("FAIL %s: step %zu (%s) reads RevisionOut[%zu] "
                        "without RevisionDone[%zu], so it may read the "
                        "buffer before the fuse says which one holds the "
                        "points\n", name, index, step.label.c_str(),
                        revision, revision);
        }
    }

    std::printf("  %s: %zu step(s), %zu edge(s)\n", name, B.steps.size(),
                edges);
}

/// The clustering is a partition of the steps, and the graph it induces is
/// acyclic -- at every grain, because a grain is a performance knob and a
/// knob that could change an answer is not one.
///
/// Three grains: 0, which puts every step in its own cluster and exposes
/// every edge the graph has; the one Build chose; and 200 microseconds,
/// which on any of these rigs packs most of the program into a handful of
/// clusters. The acceptance matrix runs the rigs under all three, so what
/// this suite adds is the structural claim the dumps cannot see.
void
TestTheClusteringIsSound(const BuiltProgram &built, const char *name)
{
    CHECK(built.program != nullptr);
    if (!built.program) {
        return;
    }
    const RigExecBakedProgramImpl &B = built.program->GetStepGraph();
    const double grains[3] = {0.0, B.clustering.grainUs, 200.0};
    for (const double grain : grains) {
        const RigExecBakedClustering schedule =
            RigExecBakedBuildClusters(B, grain);
        if (schedule.clusterOf.size() != B.steps.size()) {
            ++failures;
            std::printf("FAIL %s grain %g: %zu step assignments for %zu "
                        "steps\n", name, grain, schedule.clusterOf.size(),
                        B.steps.size());
            continue;
        }

        // (1) Every step is in exactly one cluster, and it is the cluster
        // clusterOf names.
        std::vector<int> seen(B.steps.size(), 0);
        for (size_t c = 0; c < schedule.clusters.size(); ++c) {
            for (const int member : schedule.clusters[c].members) {
                if (member < 0 || size_t(member) >= B.steps.size()) {
                    ++failures;
                    std::printf("FAIL %s grain %g: cluster %zu names step "
                                "%d\n", name, grain, c, member);
                    continue;
                }
                ++seen[size_t(member)];
                if (schedule.clusterOf[size_t(member)] != int(c)) {
                    ++failures;
                    std::printf("FAIL %s grain %g: step %d is in cluster %zu "
                                "but assigned to %d\n", name, grain, member,
                                c, schedule.clusterOf[size_t(member)]);
                }
            }
        }
        for (size_t index = 0; index < seen.size(); ++index) {
            if (seen[index] != 1) {
                ++failures;
                std::printf("FAIL %s grain %g: step %zu is in %d cluster(s)\n",
                            name, grain, index, seen[index]);
            }
        }

        // (2) Members are in increasing program order, which is what lets a
        // cluster run them with no schedule of its own: program order is a
        // topological order of the step graph.
        for (size_t c = 0; c < schedule.clusters.size(); ++c) {
            const std::vector<int> &members = schedule.clusters[c].members;
            for (size_t i = 1; i < members.size(); ++i) {
                if (members[i - 1] >= members[i]) {
                    ++failures;
                    std::printf("FAIL %s grain %g: cluster %zu lists step %d "
                                "before %d\n", name, grain, c,
                                members[i - 1], members[i]);
                }
            }
        }

        // (3) The quotient graph is acyclic. A cycle would deadlock the
        // parallel executor outright -- two clusters each waiting on the
        // other's counter -- so this is the one property the executor cannot
        // check for itself.
        std::vector<int> remaining(schedule.clusters.size(), 0);
        std::vector<int> ready;
        for (size_t c = 0; c < schedule.clusters.size(); ++c) {
            remaining[c] = int(schedule.clusters[c].preds.size());
            if (!remaining[c]) {
                ready.push_back(int(c));
            }
        }
        size_t drained = 0;
        for (size_t head = 0; head < ready.size(); ++head) {
            ++drained;
            const RigExecBakedCluster &cluster =
                schedule.clusters[size_t(ready[head])];
            for (const int succ : cluster.succs) {
                if (--remaining[size_t(succ)] == 0) {
                    ready.push_back(succ);
                }
            }
        }
        if (drained != schedule.clusters.size()) {
            ++failures;
            std::printf("FAIL %s grain %g: the cluster graph has a cycle "
                        "(%zu of %zu clusters reachable)\n", name, grain,
                        drained, schedule.clusters.size());
        }

        // (4) preds and succs describe one relation, which the counters the
        // executor resets from depend on.
        for (size_t c = 0; c < schedule.clusters.size(); ++c) {
            for (const int pred : schedule.clusters[c].preds) {
                const std::vector<int> &succs =
                    schedule.clusters[size_t(pred)].succs;
                if (std::find(succs.begin(), succs.end(), int(c)) ==
                    succs.end()) {
                    ++failures;
                    std::printf("FAIL %s grain %g: cluster %zu names "
                                "predecessor %d, which does not name it "
                                "back\n", name, grain, c, pred);
                }
            }
        }
        std::printf("  %s grain %g: %zu cluster(s), serial %.1fus, critical "
                    "path %.1fus\n", name, grain, schedule.clusters.size(),
                    schedule.serialCost, schedule.criticalPathCost);
    }
}

/// Two builds of one stage produce the same schedule, character for
/// character. Without this the report is a debugging aid nobody can diff;
/// with it, a schedule change shows up in a review.
void
TestTheReportIsDeterministic(const std::string &stagePath)
{
    const BuiltProgram first = Build(stagePath);
    const BuiltProgram second = Build(stagePath);
    CHECK(first.program != nullptr);
    CHECK(second.program != nullptr);
    if (!first.program || !second.program) {
        return;
    }
    const std::string a =
        RigExecBakedScheduleReport(first.program->GetStepGraph());
    const std::string b =
        RigExecBakedScheduleReport(second.program->GetStepGraph());
    CHECK(!a.empty());
    if (a == b) {
        return;
    }
    ++failures;
    std::printf("FAIL: the schedule report is not deterministic\n");
    size_t line = 0, offset = 0;
    while (offset < a.size() && offset < b.size() && a[offset] == b[offset]) {
        if (a[offset] == '\n') ++line;
        ++offset;
    }
    std::printf("    first difference at line %zu\n", line);
}

/// The mode the whole parity argument rests on: anything but an explicit
/// RIGEXEC_BAKED_SCHEDULE=parallel is the reference order, and asking for
/// parallel with rigExec's own threading switched off is too. The suite runs
/// under all three of those environments (verify_sched.sh), so the assertion
/// is on the MAPPING rather than on one answer.
void
TestTheModeIsTheOneTheEnvironmentAsked()
{
    const bool asked = TfGetenv("RIGEXEC_BAKED_SCHEDULE", "serial") ==
                       "parallel";
    const RigExecBakedScheduleMode expected =
        asked && RigExecParallelEvaluationEnabled()
            ? RigExecBakedScheduleMode::Parallel
            : RigExecBakedScheduleMode::Serial;
    CHECK(RigExecBakedScheduleModeFromEnvironment() == expected);
}

std::string
SchemaResourceDir(const std::string &examplesDir)
{
#ifdef RIGEXEC_SCHEMA_RESOURCE_DIR
    (void)examplesDir;
    return TfAbsPath(RIGEXEC_SCHEMA_RESOURCE_DIR);
#else
    return TfAbsPath(examplesDir + "/../plugin/rigExecSchema/resources");
#endif
}

}  // namespace

int
main(int argc, char **argv)
{
    if (argc < 2) {
        std::printf("usage: testRigExecBakedSchedule <examplesDir>\n");
        return 2;
    }
    const std::string examplesDir = argv[1];
    const std::string resources = SchemaResourceDir(examplesDir);
    if (PlugRegistry::GetInstance().RegisterPlugins(resources).empty()) {
        std::printf("FATAL: no schema plugin found at %s\n",
                    resources.c_str());
        return 2;
    }
    TestTheModeIsTheOneTheEnvironmentAsked();
    const BuiltProgram biped = Build(examplesDir + "/biped/Biped.usda");
    const BuiltProgram animated =
        Build(examplesDir + "/biped/Biped_anim.usda");
    const BuiltProgram spider =
        Build(examplesDir + "/spider_legs_assembly_ref.usda");
    // Three revisions on one chain, which no example stage has and which
    // every rule about reading a chain's running value needs.
    const BuiltProgram stacked = BuildStage(MakeStackedChainStage());
    TestTheGraphDescribesTheProgram(biped, "Biped");
    TestTheGraphDescribesTheProgram(animated, "Biped_anim");
    TestTheGraphDescribesTheProgram(spider, "spider_legs");
    TestTheGraphDescribesTheProgram(stacked, "stacked_revisions");
    TestTheClusteringIsSound(biped, "Biped");
    TestTheClusteringIsSound(spider, "spider_legs");
    TestTheClusteringIsSound(stacked, "stacked_revisions");
    TestTheReportIsDeterministic(examplesDir + "/biped/Biped.usda");
    if (failures) {
        std::printf("%d FAILURE(S)\n", failures);
        return 1;
    }
    std::printf("testRigExecBakedSchedule: all tests passed\n");
    return 0;
}
