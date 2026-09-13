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

#include "pxr/base/plug/registry.h"
#include "pxr/base/tf/getenv.h"
#include "pxr/base/tf/pathUtils.h"
#include "pxr/usd/usd/prim.h"
#include "pxr/usd/usd/primRange.h"
#include "pxr/usd/usd/stage.h"

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
Build(const std::string &stagePath)
{
    BuiltProgram built;
    built.stage = UsdStage::Open(stagePath);
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
TestTheGraphDescribesTheProgram(const std::string &stagePath,
                                const char *name)
{
    const BuiltProgram built = Build(stagePath);
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
    std::printf("  %s: %zu step(s), %zu edge(s)\n", name, B.steps.size(),
                edges);
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
    TestTheGraphDescribesTheProgram(examplesDir + "/biped/Biped.usda",
                                    "Biped");
    TestTheGraphDescribesTheProgram(examplesDir + "/biped/Biped_anim.usda",
                                    "Biped_anim");
    TestTheGraphDescribesTheProgram(
        examplesDir + "/spider_legs_assembly_ref.usda", "spider_legs");
    TestTheReportIsDeterministic(examplesDir + "/biped/Biped.usda");
    if (failures) {
        std::printf("%d FAILURE(S)\n", failures);
        return 1;
    }
    std::printf("testRigExecBakedSchedule: all tests passed\n");
    return 0;
}
