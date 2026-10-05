// The baked program's step graph must describe the straight line it replaced.
// Every other suite asserts on what a frame PUBLISHES, which the serial
// executor gets right by running the steps in program order whatever the
// edges say. That is exactly why the edges need a test of their own: they are
// the part of this design nothing else exercises yet, and the first thing the
// parallel executor will trust. So this suite asks the questions whose
// wrong answer makes a parallel run wrong while leaving a serial run
// perfect (and the one whose wrong answer makes a CONE wrong while leaving
// both executors perfect):
//   * every edge points FORWARD in program order, so running the steps in
//     index order is always a topological order;
//   * every declared read either has a writer earlier in the program or names
//     a domain the prologue fills, so nothing reads a slot the graph never
//     produced;
//   * two steps that declare overlapping WRITES are ordered with respect to
//     each other, so no schedule can run them at once;
//   * a step that read-modify-writes a slot declares the READ as well, so no
//     cone can skip it in a generation that moved the slot;
//   * the report is deterministic, so a schedule can be diffed between two
//     builds of the same stage and a change in it is a change someone made;
//   * the validator Build refuses programs with accepts every rig here and
//     rejects, by name, each malformed graph the edge sweep cannot see.
// The geometry section below asks the two questions the vertex partition
// adds: that the chunks of a skin revision cover every vertex exactly once
// and that no chunk is missing an influence one of its own vertices names --
// a chunk that skins a vertex against an identity it never noticed is a
// silently wrong deformation, not a crash -- and that a revision whose packet
// the frame rejects passes its preceding points through exactly as the
// dynamic path does, which is the decision the fuse took over from the
// kernel.
// argv[1] = path to the examples directory (containing biped/Biped.usda).
#include "rigExec/bakedProgram.h"
#include "rigExec/bakedProgramImpl.h"
#include "rigExec/bakedSchedule.h"
#include "rigExec/parallel.h"
#include "rigExec/moverGraph.h"
#include "rigExecMath/dualQuat.h"
#include "rigExecMath/simdKernels.h"
#include "rigExecMath/solvers.h"
#include "rigExec/rigEvaluator.h"
#include "rigExec/tapSet.h"

#include "pxr/base/gf/vec3f.h"
#include "pxr/base/plug/registry.h"
#include "pxr/base/gf/matrix4d.h"
#include "pxr/base/tf/getenv.h"
#include "pxr/base/tf/setenv.h"
#include "pxr/base/tf/pathUtils.h"
#include "pxr/base/vt/array.h"
#include "pxr/base/vt/value.h"
#include "pxr/usd/usd/attribute.h"
#include "pxr/usd/usd/prim.h"
#include "pxr/usd/usd/primRange.h"
#include "pxr/usd/usd/relationship.h"
#include "pxr/usd/usd/stage.h"

#include "rigExecPoseCompare.h"

#include <algorithm>
#include <cmath>
#include <set>
#include <cstdint>
#include <cstdio>
#include <map>
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

/// A rig whose JOINT slots carry two solver commits each, which no example
/// stage has: every shipped multi-solver rig resolves to one writer per joint
/// through the rest-reference relaxation, so "two writes of one pose slot" is
/// only ever a solver followed by a constraint there.
///
/// rigExec:joints is an ordered write, so an FK chain and a two-bone IK may
/// both name the same three joints. That is two SolverCommit steps declaring
/// one slot, which is the shape the SSA ladder was built for and the shape
/// the storage rules below would be vacuous about otherwise.
UsdStageRefPtr
MakeStackedSolversStage()
{
    const auto rest = [](double x, double y, double z) {
        GfMatrix4d m(1.0);
        m.SetTranslateOnly(GfVec3d(x, y, z));
        return m;
    };
    const auto define = [](const UsdStageRefPtr &stage, const char *path,
                           const char *type) {
        return stage->DefinePrim(SdfPath(path), TfToken(type));
    };
    const auto targets = [](const UsdPrim &prim, const char *name,
                            const SdfPathVector &paths) {
        UsdRelationship rel = prim.GetRelationship(TfToken(name));
        if (!rel) rel = prim.CreateRelationship(TfToken(name));
        rel.SetTargets(paths);
    };

    const UsdStageRefPtr stage = UsdStage::CreateInMemory();
    define(stage, "/Asset", "Xform");
    define(stage, "/Asset/Rig", "RigExecRoot");

    const UsdPrim hipRoot =
        define(stage, "/Asset/Rig/Controls/HipRoot", "RigExecControl");
    hipRoot.GetAttribute(TfToken("rest:space")).Set(rest(0, 8, 0));
    const UsdPrim footIk =
        define(stage, "/Asset/Rig/Controls/FootIK", "RigExecControl");
    footIk.GetAttribute(TfToken("avars:ty")).Set(0.8);
    const UsdPrim kneePole =
        define(stage, "/Asset/Rig/Controls/KneePole", "RigExecControl");
    kneePole.GetAttribute(TfToken("rest:space")).Set(rest(0, 4, 3));

    const UsdPrim fkHip =
        define(stage, "/Asset/Rig/Controls/FkHip", "RigExecControl");
    fkHip.GetAttribute(TfToken("rest:space")).Set(rest(0, 8, 0));
    fkHip.GetAttribute(TfToken("avars:rz")).Set(20.0);
    const UsdPrim fkKnee =
        define(stage, "/Asset/Rig/Controls/FkHip/FkKnee", "RigExecControl");
    fkKnee.GetAttribute(TfToken("rest:space")).Set(rest(4, 0, 0));
    const UsdPrim fkAnkle = define(
        stage, "/Asset/Rig/Controls/FkHip/FkKnee/FkAnkle", "RigExecControl");
    fkAnkle.GetAttribute(TfToken("rest:space")).Set(rest(4, 0, 0));

    const UsdPrim hip = define(stage, "/Asset/Rig/Joints/Hip",
                               "RigExecJoint");
    hip.GetAttribute(TfToken("rest:space")).Set(rest(0, 8, 0));
    const UsdPrim knee =
        define(stage, "/Asset/Rig/Joints/Hip/Knee", "RigExecJoint");
    knee.GetAttribute(TfToken("rest:space")).Set(rest(4, 0, 0));
    const UsdPrim ankle =
        define(stage, "/Asset/Rig/Joints/Hip/Knee/Ankle", "RigExecJoint");
    ankle.GetAttribute(TfToken("rest:space")).Set(rest(4, 0, 0));
    const SdfPathVector chain{hip.GetPath(), knee.GetPath(),
                              ankle.GetPath()};

    const UsdPrim ik =
        define(stage, "/Asset/Rig/Solvers/LegIK", "RigExecTwoBoneIk");
    targets(ik, "rigExec:rootControl", {hipRoot.GetPath()});
    targets(ik, "rigExec:effectorControl", {footIk.GetPath()});
    targets(ik, "rigExec:poleControl", {kneePole.GetPath()});
    targets(ik, "rigExec:joints", chain);

    const UsdPrim fk =
        define(stage, "/Asset/Rig/Solvers/LegFK", "RigExecFkChain");
    targets(fk, "rigExec:controls",
            {fkHip.GetPath(), fkKnee.GetPath(), fkAnkle.GetPath()});
    targets(fk, "rigExec:joints", chain);
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
                        if (++races <= 4) {
                            std::printf("FAIL %s: step %zu (%s) writes %s"
                                        "[%u,%u) that unordered step %zu (%s) "
                                        "reads\n", name, earlier,
                                        B.steps[earlier].label.c_str(),
                                        RigExecBakedSlotDomainName(write.domain),
                                        write.begin, write.end, later,
                                        B.steps[later].label.c_str());
                        }
                    }
                }
            }
            for (const RigExecBakedSlotRange &read : B.steps[earlier].reads) {
                for (const RigExecBakedSlotRange &write :
                         B.steps[later].writes) {
                    // WRITE AFTER READ in a VERSIONED pose domain is not a
                    // race and is deliberately left unordered by
                    // RigExecBakedBuildStepEdges: a writer there writes
                    // storage of its OWN version, so a reader of an earlier
                    // version and a later writer touch different memory. The
                    // unified pose stack makes this shape ordinary rather
                    // than theoretical -- a solver reads a joint the
                    // constraint ABOVE it revises, which is exactly a read of
                    // the earlier version -- so the check follows the edge
                    // builder instead of being stricter than the program.
                    if (RigExecBakedIsVersionedDomain(write.domain)) {
                        continue;
                    }
                    if (write.Overlaps(read)) {
                        if (++races <= 4) {
                            std::printf("FAIL %s: step %zu (%s) reads %s"
                                        "[%u,%u) that unordered step %zu (%s) "
                                        "writes\n", name, earlier,
                                        B.steps[earlier].label.c_str(),
                                        RigExecBakedSlotDomainName(write.domain),
                                        write.begin, write.end, later,
                                        B.steps[later].label.c_str());
                        }
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
    // deciding the indirection, and is excluded. Chain readers declare a
    // point version, not buffers; this rule catches a buffer read that
    // appears without its version read.
    // RevisionOut is indexed by CHUNK and RevisionDone by REVISION, so the
    // rule is stated over revisions and the buffer slots are mapped back to
    // the revision that owns them: a chunk of revision r reading any earlier
    // chunk's buffer is a step reading revision r' output, and it is r' whose
    // fuse chose it.
    const size_t revisions = B.revisionIndex.size();
    std::vector<int> revisionOfChunk;
    for (size_t revision = 0; revision < revisions; ++revision) {
        const int base = B.revisionChunkBase[revision];
        const int count = B.revisionChunkCount[revision];
        if (size_t(base + count) > revisionOfChunk.size()) {
            revisionOfChunk.resize(size_t(base + count), -1);
        }
        for (int k = 0; k < count; ++k) {
            revisionOfChunk[size_t(base + k)] = int(revision);
        }
    }
    auto mark = [revisions](std::vector<bool> *flags,
                            const RigExecBakedSlotRange &range) {
        for (size_t slot = range.begin;
             slot < range.end && slot < revisions; ++slot) {
            (*flags)[slot] = true;
        }
    };
    auto markOut = [&revisionOfChunk](std::vector<bool> *flags,
                                      const RigExecBakedSlotRange &range) {
        for (size_t slot = range.begin;
             slot < range.end && slot < revisionOfChunk.size(); ++slot) {
            const int revision = revisionOfChunk[slot];
            if (revision >= 0) {
                (*flags)[size_t(revision)] = true;
            }
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
                markOut(&readsOut, read);
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

    // (6) An unsplit commit declares a READ of every candidate slot it
    // measures its delta against, and every commit declares the versions its
    // write-back carries. ComputeCommitDeltas takes B.fin[slot] as
    // it stood BEFORE the commit and the write-back then overwrites that
    // slot, so the read is not implied by the write: a cone that sees only
    // the write is free to skip the commit in a generation that moved the
    // slot, and the delta -- and so every descendant the commit propagates
    // to -- is then measured against the commit's own last answer. The
    // split arrangement states this on its CommitDelta step. It is asserted
    // here because the symptom is invisible in a serial run, and invisible
    // in every published value of a commit that has no descendants: what
    // caught it was the cone verifier at one cluster per step.
    // The same rule covers what the write-back CARRIES, on whichever step
    // performs it: FinishCommit copies the version it found into its own
    // storage wherever it declines to write, so a solver commit reads the
    // PoseBase version of every candidate and every descendant, and the
    // split arrangement's APPLY step reads the PoseFin version of every
    // candidate -- none of which the write implies either. Neither can race
    // (the same step writes those slots, so check (3) has already ordered it
    // against every other writer), which is exactly why nothing caught the
    // omission: what is at stake is a graph that says what its steps touch.
    const auto declaresRead = [](const RigExecBakedStep &step,
                                 RigExecBakedSlotDomain domain, int slot) {
        for (const RigExecBakedSlotRange &read : step.reads) {
            if (read.domain == domain && read.begin <= uint32_t(slot) &&
                uint32_t(slot) < read.end) {
                return true;
            }
        }
        return false;
    };
    for (size_t index = 0; index < B.steps.size(); ++index) {
        const RigExecBakedStep &step = B.steps[index];
        const bool head = step.kind == RigExecBakedStepKind::SolverCommit ||
                          step.kind == RigExecBakedStepKind::Constraint;
        if (!head && step.kind != RigExecBakedStepKind::CommitApply) {
            continue;
        }
        if (step.object < 0 || size_t(step.object) >= B.commits.size()) {
            continue;
        }
        const RigExecBakedCommit &commit = B.commits[size_t(step.object)];
        if (head == commit.split) {
            continue;  // the other arrangement's step of this commit
        }
        const auto require = [&](RigExecBakedSlotDomain domain, int slot,
                                 const char *what) {
            if (declaresRead(step, domain, slot)) {
                return;
            }
            ++failures;
            std::printf("FAIL %s: step %zu (%s) %s %s[%d] without "
                        "declaring the read\n", name, index,
                        step.label.c_str(), what,
                        RigExecBakedSlotDomainName(domain), slot);
        };
        for (const int slot : commit.slots) {
            require(RigExecBakedSlotDomain::PoseFin, slot,
                    head ? "measures a delta against" : "carries");
            if (commit.solverOutput) {
                require(RigExecBakedSlotDomain::PoseBase, slot, "carries");
            }
        }
        if (!commit.solverOutput) {
            continue;
        }
        for (const auto &pair : commit.propagate) {
            require(RigExecBakedSlotDomain::PoseBase, pair.first, "carries");
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

/// A synthetic skin packet: \p influences joints, \p points vertices,
/// elementSize slots per vertex, weights and indices that vary per vertex.
RigExecMoverParameters
SyntheticSkinPacket(size_t influences, size_t points, size_t elementSize,
                    const TfToken &method)
{
    RigExecMoverParameters packet;
    packet.kind = TfToken("skin");
    packet.enabled = true;
    packet.valid = true;
    packet.skinningMethod = method;
    packet.weights = RigExecWeightPacket::Constant(1.0f);
    packet.skinElementSize = int(elementSize);
    for (size_t t = 0; t < influences; ++t) {
        GfMatrix4d matrix(1.0);
        matrix.SetTranslate(GfVec3d(double(t) * 0.25, double(t) * -0.125,
                                    double(t) * 0.5));
        matrix[0][0] = 1.0 + 0.01 * double(t);
        matrix[1][2] = 0.02 * double(t);
        packet.skinTransforms.push_back(matrix);
    }
    for (size_t i = 0; i < points; ++i) {
        float total = 0.0f;
        for (size_t k = 0; k < elementSize; ++k) {
            packet.skinIndices.push_back(int((i * 7 + k * 3) % influences));
            const float weight = float((i + k + 1) % 5) / 10.0f;
            packet.skinWeights.push_back(weight);
            total += weight;
        }
        // A shortfall on some vertices and a full set on others, so the
        // complement rule is exercised on both sides.
        if (total > 0.9f) {
            packet.skinWeights[i * elementSize] +=
                std::max(0.0f, 1.0f - total);
        }
    }
    return packet;
}

/// The range form deforms a vertex exactly as the whole-array form does.
///
/// This is the shared-kernel rule as an assertion: the chunked path and the
/// unchunked one run the same per-vertex body, so a mesh cut into ranges is
/// bit-identical to the same mesh deformed whole -- not equal to tolerance,
/// bit-identical -- for both skinning methods and whether the caller narrows
/// and splits the influence table itself or lets the kernel do it. The rigs
/// that bake today are all classicLinear, so without this the
/// dual-quaternion range form has no fixture at all.
void
TestTheRangeFormDeformsLikeTheWholeArray(const char *method)
{
    const size_t influences = 11, points = 9000, elementSize = 4;
    const RigExecMoverParameters packet = SyntheticSkinPacket(
        influences, points, elementSize, TfToken(method));
    std::vector<GfVec3f> rest(points);
    for (size_t i = 0; i < points; ++i) {
        rest[i] = GfVec3f(float(i % 37) * 0.5f, float(i % 11) * 1.25f,
                          float(i % 5) * -2.0f);
    }

    std::vector<GfVec3f> whole = rest;
    CHECK(RigExecApplySkinKernel(packet, &whole));

    // The same influence table, narrowed and split once by the caller -- the
    // forms a chunk keeps beside its own matrices.
    std::vector<float> rows(influences * RigExecSkinRowStride);
    for (size_t t = 0; t < influences; ++t) {
        RigExecNarrowSkinRows(packet.skinTransforms[t],
                              &rows[t * RigExecSkinRowStride]);
    }
    RigExecSkinLayout layout;
    layout.transforms = packet.skinTransforms.data();
    layout.transformCount = influences;
    const std::vector<RigExecScaledDualQuat> palette =
        RigExecSkinDualQuatPalette(layout);

    for (const bool supplied : {false, true}) {
        for (const size_t chunk : {size_t(1), size_t(512), size_t(4096)}) {
            RigExecSkinTransformsView view;
            view.transforms = packet.skinTransforms.data();
            view.transformCount = influences;
            if (supplied) {
                view.rows = rows.data();
                view.palette = palette.data();
                view.paletteSize = palette.size();
            }
            std::vector<GfVec3f> ranged = rest;
            bool ok = true;
            for (size_t begin = 0; begin < points; begin += chunk) {
                ok = ok && RigExecApplySkinKernelRange(
                               packet, view, begin,
                               std::min(points, begin + chunk), &ranged);
            }
            CHECK(ok);
            if (ranged != whole) {
                ++failures;
                std::printf("FAIL %s: ranges of %zu%s differ from the whole "
                            "array\n", method, chunk,
                            supplied ? " (caller's tables)" : "");
            }
        }
    }
    std::printf("  %s: range forms are bit-identical to the whole array\n",
                method);
}

/// A chunk deforms its vertices against a table that is IDENTITY outside its
/// own key, and gets the same points as the whole array does.
///
/// This is the invariant the speculation rests on, asserted directly rather
/// than through a rig: a chunk copies only |key| matrices into its table and
/// leaves every other entry the identity the partition filled it with, so if
/// a kernel ever reached an entry outside the key -- a normalisation over the
/// table, a complement taken from the wrong slot -- the chunked result would
/// move and the whole-array one would not. The dual-quaternion case is the
/// one that needs saying out loud, because its palette is built from the
/// padded table and the weight complement is an extra entry past its end.
void
TestAChunkSeesOnlyItsOwnInfluences(const char *method)
{
    const size_t influences = 11, points = 9000, elementSize = 4;
    RigExecMoverParameters packet = SyntheticSkinPacket(
        influences, points, elementSize, TfToken(method));
    // Region-local indices, which is what makes a key a PART of the table:
    // the synthetic packet's indices stride the whole table, so every range
    // of it would name every influence and the padding under test would
    // never exist. A mesh's vertices are numbered by region, which is the
    // property the partition trades on.
    for (size_t i = 0; i < points; ++i) {
        const size_t region = i / 900;
        for (size_t k = 0; k < elementSize; ++k) {
            packet.skinIndices[i * elementSize + k] =
                int((region + k) % influences);
        }
    }
    std::vector<GfVec3f> rest(points);
    for (size_t i = 0; i < points; ++i) {
        rest[i] = GfVec3f(float(i % 37) * 0.5f, float(i % 11) * 1.25f,
                          float(i % 5) * -2.0f);
    }
    std::vector<GfVec3f> whole = rest;
    CHECK(RigExecApplySkinKernel(packet, &whole));

    const size_t chunk = 512;
    size_t padded = 0;
    std::vector<GfVec3f> ranged = rest;
    bool ok = true;
    for (size_t begin = 0; begin < points; begin += chunk) {
        const size_t end = std::min(points, begin + chunk);
        // The key, off the same indices the kernel reads.
        std::set<int> key;
        for (size_t point = begin; point < end; ++point) {
            for (size_t slot = 0; slot < elementSize; ++slot) {
                key.insert(packet.skinIndices[point * elementSize + slot]);
            }
        }
        if (key.size() < influences) {
            ++padded;
        }
        // The chunk's own tables: identity everywhere, the key copied in,
        // and the two derived forms maintained entry by entry beside it --
        // which is exactly what GatherChunkTransforms does per run.
        std::vector<GfMatrix4d> transforms(influences, GfMatrix4d(1.0));
        std::vector<float> rows(influences * RigExecSkinRowStride);
        std::vector<RigExecScaledDualQuat> palette(influences + 1);
        for (const int index : key) {
            transforms[size_t(index)] = packet.skinTransforms[size_t(index)];
        }
        for (size_t t = 0; t < influences; ++t) {
            RigExecNarrowSkinRows(transforms[t],
                                  &rows[t * RigExecSkinRowStride]);
            palette[t] = RigExecScaledDualQuatFromMatrix(transforms[t]);
        }
        palette[influences] = RigExecScaledDualQuat();
        RigExecSkinTransformsView view;
        view.transforms = transforms.data();
        view.transformCount = transforms.size();
        view.rows = rows.data();
        view.palette = palette.data();
        view.paletteSize = palette.size();
        ok = ok && RigExecApplySkinKernelRange(packet, view, begin, end,
                                               &ranged);
    }
    CHECK(ok);
    // Otherwise every table above held every matrix and the assertion below
    // would be the previous test's.
    CHECK(padded > 0);
    if (ranged != whole) {
        ++failures;
        std::printf("FAIL %s: identity-padded chunk tables differ from the "
                    "whole array\n", method);
        return;
    }
    std::printf("  %s: a chunk reads no entry outside its key (%zu padded "
                "range(s))\n", method, padded);
}

/// The skin mover of \p stage, or an empty path.
SdfPath
FindSkinMover(const UsdStageRefPtr &stage)
{
    for (const UsdPrim &prim : stage->Traverse()) {
        if (prim.GetTypeName() == "RigExecSkinMover") {
            return prim.GetPath();
        }
    }
    return SdfPath();
}

/// Every skin revision's partition covers its vertices exactly once, stays
/// under the cap, and gives each chunk a key that contains every influence
/// its own vertices name.
///
/// The last one is the invariant the whole design rests on: a chunk fills the
/// entries of its key and leaves the rest of the table identity, so an
/// influence missing from a key is a vertex quietly skinned against the
/// identity. Checked against the authored arrays rather than against the
/// partition's own bookkeeping, so the check cannot agree with a bug by
/// reading it back.
void
TestTheVertexPartitionCoversEveryVertexOnce(const std::string &stagePath,
                                            const char *name, bool report)
{
    const BuiltProgram built = Build(stagePath);
    CHECK(built.program != nullptr);
    if (!built.program) {
        return;
    }
    // One frame, so the partition can be asked whether it SURVIVED contact
    // with the layout the packet carries: a stale one is correct -- the fuse
    // runs the revision whole -- and would make every assertion below true
    // of chunks nothing ran.
    RigExecRigPose pose;
    CHECK(built.program->Run(UsdTimeCode::Default(), &pose));
    const RigExecBakedProgramImpl &B = built.program->GetStepGraph();
    const size_t cap =
        size_t(std::max(1, TfGetenvInt("RIGEXEC_BAKED_MAX_CHUNKS", 32)));
    size_t skinRevisions = 0;
    for (const RigExecBakedProgramImpl::GeomChain &chain : B.chains) {
        for (const RigExecBakedProgramImpl::GeomRevision &revision :
                 chain.revisions) {
            if (revision.op != RigExecRevisionOp::Skin) {
                continue;
            }
            ++skinRevisions;
            CHECK(!revision.chunks.empty());
            CHECK(revision.chunks.size() <= cap);
            // The packet's table is identity and sized to the influences:
            // that is what lets the assemble run before the fold.
            CHECK(revision.packetInfluences.size() ==
                  revision.influenceSlots.size());
            for (const GfMatrix4d &matrix : revision.packetInfluences) {
                CHECK(matrix == GfMatrix4d(1.0));
            }
            if (!revision.chunked) {
                continue;
            }
            // The frame agreed that the keys still describe the vertices,
            // and every chunk produced its range.
            CHECK(!revision.partitionStale);
            for (const RigExecBakedProgramImpl::GeomChunk &chunk :
                     revision.chunks) {
                CHECK(chunk.ok);
            }
            // (1) The ranges are contiguous, ascending and cover exactly
            // [0, pointCount).
            int expected = 0;
            for (const RigExecBakedProgramImpl::GeomChunk &chunk :
                     revision.chunks) {
                CHECK(chunk.begin == expected);
                CHECK(chunk.end >= chunk.begin);
                expected = chunk.end;
            }
            CHECK(size_t(expected) == revision.partitionPointCount);

            // (2) Every vertex's influences are in its chunk's key, read
            // back out of the authored layout.
            const UsdPrim mover =
                built.stage->GetPrimAtPath(revision.moverPath);
            VtIntArray indices;
            int elementSize = 0;
            if (const UsdAttribute a =
                    mover.GetAttribute(TfToken("rigExec:jointIndices"))) {
                a.Get(&indices, UsdTimeCode::Default());
            }
            if (const UsdAttribute a =
                    mover.GetAttribute(TfToken("rigExec:elementSize"))) {
                a.Get(&elementSize, UsdTimeCode::Default());
            }
            CHECK(elementSize >= 1);
            CHECK(indices.size() ==
                  revision.partitionPointCount * size_t(elementSize));
            if (elementSize < 1) {
                continue;
            }
            size_t missing = 0;
            for (const RigExecBakedProgramImpl::GeomChunk &chunk :
                     revision.chunks) {
                const std::set<int> key(chunk.key.begin(), chunk.key.end());
                CHECK(key.size() == chunk.key.size());  // sorted, unique
                for (size_t point = size_t(chunk.begin);
                     point < size_t(chunk.end); ++point) {
                    for (size_t slot = 0; slot < size_t(elementSize); ++slot) {
                        const int index =
                            indices[point * size_t(elementSize) + slot];
                        if (index < 0 ||
                            size_t(index) >= revision.influenceSlots.size()) {
                            continue;  // the packet will reject the layout
                        }
                        if (!key.count(index)) {
                            ++missing;
                        }
                    }
                }
            }
            if (missing) {
                ++failures;
                std::printf("FAIL %s: %s has %zu vertex influence(s) outside "
                            "their chunk's key\n", name,
                            revision.moverPath.GetString().c_str(), missing);
            }
        }
    }
    std::printf("  %s: %zu skin revision(s)\n", name, skinRevisions);
    if (report) {
        std::printf("%s", RigExecBakedGeometryReport(B).c_str());
    }
}

/// A revision is cut into chunks only where the cut buys a head start.
///
/// The rule (§6): a chunk body is a serial loop, while an uncut revision is
/// one RigExecApplySkinKernel call that spreads itself over the arena -- so
/// cutting is a LOSS unless some range becomes runnable before the whole
/// revision could. That is a question about levels, which is why Build sweeps
/// the pose half's edges before the geometry half is built.
///
/// Asserted on the decision the program recorded rather than on a rig that
/// happens to answer one way: `chunked` must be exactly "more than one
/// candidate range, and their ready levels differ". The environment override
/// is checked in the same breath, because it is what keeps every other chunk
/// assertion in this file from passing vacuously.
void
TestThePartitionIsCutOnlyWhereItPays(const BuiltProgram &built,
                                     const char *name, bool always)
{
    CHECK(built.program != nullptr);
    if (!built.program) {
        return;
    }
    const RigExecBakedProgramImpl &B = built.program->GetStepGraph();
    size_t skins = 0, cut = 0;
    for (const RigExecBakedProgramImpl::GeomChain &chain : B.chains) {
        for (const RigExecBakedProgramImpl::GeomRevision &revision :
                 chain.revisions) {
            if (revision.op != RigExecRevisionOp::Skin) {
                continue;
            }
            ++skins;
            const bool differ =
                revision.partitionReadyMin < revision.partitionReadyMax;
            const bool expected = revision.partitionCandidates > 1 &&
                                  (always || differ);
            if (revision.chunked != expected) {
                ++failures;
                std::printf("FAIL %s: %s is %scut with %zu candidate(s) and "
                            "ready levels %d..%d\n", name,
                            revision.moverPath.GetString().c_str(),
                            revision.chunked ? "" : "not ",
                            revision.partitionCandidates,
                            revision.partitionReadyMin,
                            revision.partitionReadyMax);
            }
            cut += revision.chunked ? 1 : 0;
            // An uncut revision is the degenerate partition, not a partition
            // with its keys quietly dropped: one range, and the fuse's
            // whole-array path is what runs it.
            if (!revision.chunked) {
                CHECK(revision.chunks.size() == 1);
                CHECK(revision.chunks[0].key.empty());
            }
        }
    }
    std::printf("  %s: %zu of %zu skin revision(s) cut%s\n", name, cut, skins,
                always ? " (cut forced)" : "");
}

/// A derived revision keeps its 26k-point input by HANDLE, and that
/// comparison is the elementwise one with the identity case taken first.
///
/// The derived maintenance step is the frame's serial tail -- one bounding
/// box over the whole mesh -- and it used to make three passes over 315KB
/// before it computed anything: the assemble copied the chain's points into
/// the packet, `parameters != lastParameters` walked them, and
/// `lastParameters = parameters` copied them again. The last two are gone
/// because auxPoints IS the chain's final buffer and VtArray is
/// copy-on-write, so identity decides the value.
///
/// That substitution is only sound if two things hold, and both are asserted
/// here rather than assumed: the run really does remember the buffer rather
/// than a copy of it, and VtArray's own equality falls THROUGH a
/// non-identical pair to the values -- otherwise a chain that recomputed the
/// same points (a forced pass, a drag returned to its value) would count as
/// having moved, and `revisionsExecuted` is counter-parity-bearing.
void
TestTheDerivedCompareAgreesWithTheElementwiseOne(const std::string &stagePath)
{
    const BuiltProgram built = Build(stagePath);
    CHECK(built.program != nullptr);
    if (!built.program) {
        return;
    }
    RigExecRigPose first, repeated;
    CHECK(built.program->Run(UsdTimeCode::Default(), &first));
    CHECK(built.program->Run(UsdTimeCode::Default(), &repeated));
    // The identity arm, end to end: nothing moved, so nothing re-executed.
    CHECK(repeated.moverGraphRevisionsExecuted == 0);
    const RigExecBakedProgramImpl &B = built.program->GetStepGraph();
    size_t derivedRevisions = 0;
    for (const RigExecBakedProgramImpl::GeomChain &chain : B.chains) {
        for (const RigExecBakedProgramImpl::GeomChain::Derived &derived :
                 chain.derived) {
            const RigExecBakedProgramImpl::GeomRevision &revision =
                derived.revision;
            if (!revision.ran) {
                continue;
            }
            ++derivedRevisions;
            // The remembered packet carries no points at all ...
            CHECK(revision.lastParameters.auxPoints.empty());
            // ... and what stands in for them is the chain's published
            // points. Equal by VALUE and not necessarily by buffer: the
            // chain publishes into a double buffer, so a generation in which
            // the status sweep ran has swapped the array since.
            CHECK(revision.lastAuxPoints == chain.result);

            // Remembering them costs a refcount and not 315KB: assigning the
            // handle shares the buffer, which is the whole reason the packet
            // may keep them at all.
            const VtVec3fArray shared = chain.result;
            CHECK(shared.IsIdentical(chain.result));

            // A DEEP copy is a different buffer that still compares equal:
            // the fall-through the handle test relies on, and the arm that
            // runs on every generation where the status sweep republished.
            VtVec3fArray copy;
            copy.assign(chain.result.begin(), chain.result.end());
            CHECK(!copy.IsIdentical(chain.result));
            CHECK(copy == chain.result);
            CHECK(!(copy != chain.result));
            // And values that moved are still not equal, however the buffer
            // got there.
            if (!copy.empty()) {
                copy[0] += GfVec3f(1.0f, 0.0f, 0.0f);
                CHECK(copy != chain.result);
            }
        }
    }
    std::printf("  derived compare: %zu derived revision(s) keep their "
                "points by handle\n", derivedRevisions);
}

/// A rebuilt program inherits everything a revision's `ran` promises a
/// comparison against, and not just the packet.
///
/// AdoptGeometryStateFrom is what stops a rebuild from re-running every
/// per-point kernel the edit did not touch: it hands one revision's run
/// state across to the node that replaced it. A derived revision's run state
/// now lives in TWO fields rather than one -- the packet with its points
/// emptied, and the chain's buffer held by handle beside it -- and a carry
/// that takes only the packet leaves the node saying "compare against what I
/// last saw" with an empty array to compare against. Every derived revision
/// of the rig then re-executes on the first generation after any edit, which
/// is a `revisionsExecuted` and a diagnostic the dynamic path does not
/// report, because its own graphs stood through the same edit.
///
/// Two arms, because neither alone is the contract. The first asks the
/// adopted node what it carries, which is the invariant and is rig
/// independent. The second recompiles a parity-checked rig for real and
/// evaluates it, which is the shape no other fixture has: every suite here
/// runs generations of ONE program, so nothing else evaluates after an
/// adopt at all.
void
TestARebuiltProgramKeepsItsRunState(const std::string &stagePath,
                                    const char *name)
{
    const BuiltProgram built = Build(stagePath);
    CHECK(built.program != nullptr);
    if (!built.program) {
        return;
    }
    RigExecRigPose first, repeated;
    CHECK(built.program->Run(UsdTimeCode::Default(), &first));
    CHECK(built.program->Run(UsdTimeCode::Default(), &repeated));

    // What the outgoing program remembers, read BEFORE the adopt moves it
    // out from under us.
    std::map<SdfPath, VtVec3fArray> remembered;
    std::map<SdfPath, std::vector<GfMatrix4d>> folded;
    for (const RigExecBakedProgramImpl::GeomChain &chain :
             built.program->GetStepGraph().chains) {
        for (const RigExecBakedProgramImpl::GeomRevision &revision :
                 chain.revisions) {
            if (revision.ran && !revision.influences.empty()) {
                folded.emplace(revision.moverPath, revision.influences);
            }
        }
        for (const RigExecBakedProgramImpl::GeomChain::Derived &derived :
                 chain.derived) {
            if (derived.revision.ran) {
                remembered.emplace(derived.target,
                                   derived.revision.lastAuxPoints);
            }
        }
    }

    std::vector<std::string> reasons;
    std::unique_ptr<RigExecBakedProgram> rebuilt =
        RigExecBakedProgram::Build(built.evaluator.get(), &reasons);
    CHECK(rebuilt != nullptr);
    if (!rebuilt) {
        return;
    }
    rebuilt->AdoptGeometryStateFrom(*built.program);

    size_t carried = 0, tables = 0;
    for (const RigExecBakedProgramImpl::GeomChain &chain :
             rebuilt->GetStepGraph().chains) {
        for (const RigExecBakedProgramImpl::GeomRevision &revision :
                 chain.revisions) {
            const auto found = folded.find(revision.moverPath);
            if (found == folded.end()) {
                continue;
            }
            // The fold decides `influencesChanged` against this table, so an
            // adopted node that lost it reports every matrix moved and runs.
            // The binding did not change here, so the shapes agree and the
            // whole table comes across.
            CHECK(revision.influences == found->second);
            ++tables;
        }
        for (const RigExecBakedProgramImpl::GeomChain::Derived &derived :
                 chain.derived) {
            const auto found = remembered.find(derived.target);
            if (found == remembered.end()) {
                continue;
            }
            // A node that kept its `ran` kept everything that `ran` promises
            // a comparison against -- the packet AND the points, which are
            // one remembered input split across two fields.
            CHECK(derived.revision.ran);
            CHECK(derived.revision.lastAuxPoints == found->second);
            // And the points are really there: the failure this guards is an
            // empty array that compares unequal to every non-empty mesh.
            CHECK(derived.revision.lastAuxPoints.empty() ==
                  found->second.empty());
            ++carried;
        }
    }
    std::printf("  %s: %zu derived revision(s) carried their points and %zu "
                "influence table(s) came across a rebuild\n", name, carried,
                tables);
}

/// The same question asked of a real recompile, through the parity check.
///
/// Compile() retires the program and rebuilds it, and the replacement adopts
/// the geometry state of the one it replaced -- so the generation after a
/// recompile is the one generation in which the two paths can disagree about
/// how much work there was to do. `revisionsExecuted` is a compared counter,
/// so RigExecComparePoses is the judge here as everywhere else.
void
TestARecompiledRigStillAgreesWithTheDynamicPath(const std::string &stagePath,
                                                const char *name)
{
    const UsdStageRefPtr stage = UsdStage::Open(stagePath);
    CHECK(stage != nullptr);
    if (!stage) {
        return;
    }
    const SdfPath rigPath = FindRig(stage);
    CHECK(!rigPath.IsEmpty());
    if (rigPath.IsEmpty()) {
        return;
    }
    RigExecRigEvaluator evaluator(stage, rigPath);
    std::vector<std::string> errors;
    CHECK(evaluator.Compile(&errors));
    CHECK(evaluator.IsBakeable());
    evaluator.SetEvaluationMode(RigExecEvaluationMode::BakedWithParityCheck);
    const RigExecRigPose warm = evaluator.Evaluate(UsdTimeCode::Default());
    CHECK(warm.valid);
    CHECK(warm.bakedParityMismatches == 0);

    // A recompile of the same stage: a new epoch, the same shape, and an
    // outgoing program for the replacement to adopt.
    CHECK(evaluator.Compile(&errors));
    const RigExecRigPose after = evaluator.Evaluate(UsdTimeCode::Default());
    CHECK(after.valid);
    if (after.bakedParityMismatches != 0) {
        ++failures;
        std::printf("FAIL %s: %zu mismatch(es) after a recompile\n", name,
                    after.bakedParityMismatches);
        for (const std::string &diagnostic : after.diagnostics) {
            std::printf("    %s\n", diagnostic.c_str());
        }
        return;
    }
    std::printf("  %s: a recompiled rig agrees, %zu revision(s) executed\n",
                name, after.moverGraphRevisionsExecuted);
}

/// A skin revision the frame rejects publishes what the dynamic path
/// publishes: the preceding points, and the MoverFailed line.
///
/// The decision moved: the kernel used to make it, and now the fuse does,
/// out of a packet that no longer carries the influence matrices and a fold
/// that checks them separately. So the fixture drives a rejection through an
/// interactive override -- the route a manipulator uses, and the one that
/// reaches the packet without rebuilding the program -- and asks the parity
/// mode, which runs both paths in one generation and compares every
/// published map, whether they agreed.
void
TestARejectedSkinPacketPassesThroughLikeTheDynamicPath(
    const std::string &stagePath, const char *what, const TfToken &attribute,
    const VtValue &value)
{
    const UsdStageRefPtr stage = UsdStage::Open(stagePath);
    CHECK(stage != nullptr);
    if (!stage) {
        return;
    }
    const SdfPath rigPath = FindRig(stage);
    const SdfPath moverPath = FindSkinMover(stage);
    CHECK(!rigPath.IsEmpty());
    CHECK(!moverPath.IsEmpty());
    if (rigPath.IsEmpty() || moverPath.IsEmpty()) {
        return;
    }
    RigExecRigEvaluator evaluator(stage, rigPath);
    std::vector<std::string> errors;
    CHECK(evaluator.Compile(&errors));
    CHECK(evaluator.IsBakeable());
    evaluator.SetEvaluationMode(RigExecEvaluationMode::BakedWithParityCheck);

    const RigExecRigPose clean = evaluator.Evaluate(UsdTimeCode::Default());
    CHECK(clean.valid);
    CHECK(clean.bakedParityMismatches == 0);

    RigExecValueOverride override;
    override.prim = moverPath;
    override.attribute = attribute;
    override.value = value;
    evaluator.SetInteractiveOverrides({override});
    const RigExecRigPose rejected = evaluator.Evaluate(UsdTimeCode::Default());
    evaluator.ClearInteractiveOverrides();

    CHECK(rejected.valid);
    if (rejected.bakedParityMismatches != 0) {
        ++failures;
        std::printf("FAIL %s: %zu baked/dynamic mismatch(es)\n", what,
                    rejected.bakedParityMismatches);
        for (const std::string &diagnostic : rejected.diagnostics) {
            std::printf("    %s\n", diagnostic.c_str());
        }
        return;
    }
    // And the fixture really did reject the revision, rather than agree
    // about a generation in which nothing happened.
    bool failed = false;
    for (const std::string &diagnostic : rejected.diagnostics) {
        failed = failed ||
                 diagnostic.find("MoverFailed " + moverPath.GetString()) !=
                     std::string::npos;
    }
    if (!failed) {
        ++failures;
        std::printf("FAIL %s: the revision was not rejected at all\n", what);
        return;
    }
    // Rejected means the preceding points pass through, so the mesh is no
    // longer where the skin put it.
    size_t moved = 0;
    for (const auto &[target, points] : clean.movedProperties) {
        const auto found = rejected.movedProperties.find(target);
        if (found != rejected.movedProperties.end() &&
            found->second != points) {
            ++moved;
        }
    }
    if (!moved) {
        ++failures;
        std::printf("FAIL %s: the rejected revision published the same "
                    "points as the applied one\n", what);
    }
    std::printf("  %s: passed through, %zu published target(s) moved\n", what,
                moved);
}

/// A chunked revision skinned with dual quaternions publishes what the
/// dynamic path publishes.
///
/// The chunked dual-quaternion path -- where each chunk splits its OWN
/// palette out of a table that is identity outside its key, rather than
/// reading the revision's -- and the chunked linear path each need a fixture
/// of the other's shape. An interactive override on rigExec:skinningMethod
/// gives them one: the same rig, the same partition, the OTHER method from
/// whichever the rig authored, and the parity mode running both paths in one
/// generation to compare every published map. The biped has shipped
/// dual-quaternion since the port, so on it the override is the linear side.
void
TestADualQuaternionSkinChunksLikeTheDynamicPath(const std::string &stagePath)
{
    const UsdStageRefPtr stage = UsdStage::Open(stagePath);
    CHECK(stage != nullptr);
    if (!stage) {
        return;
    }
    const SdfPath rigPath = FindRig(stage);
    const SdfPath moverPath = FindSkinMover(stage);
    CHECK(!rigPath.IsEmpty());
    CHECK(!moverPath.IsEmpty());
    if (rigPath.IsEmpty() || moverPath.IsEmpty()) {
        return;
    }
    RigExecRigEvaluator evaluator(stage, rigPath);
    std::vector<std::string> errors;
    CHECK(evaluator.Compile(&errors));
    CHECK(evaluator.IsBakeable());
    evaluator.SetEvaluationMode(RigExecEvaluationMode::BakedWithParityCheck);
    const RigExecRigPose linear = evaluator.Evaluate(UsdTimeCode::Default());
    CHECK(linear.valid);
    CHECK(linear.bakedParityMismatches == 0);

    // Whichever method the rig did NOT author, so the override changes the
    // kernel rather than restating it.
    TfToken authored("classicLinear");
    if (const UsdAttribute method = stage->GetPrimAtPath(moverPath)
            .GetAttribute(TfToken("rigExec:skinningMethod"))) {
        method.Get(&authored);
    }
    RigExecValueOverride override;
    override.prim = moverPath;
    override.attribute = TfToken("rigExec:skinningMethod");
    override.value = VtValue(TfToken(authored == TfToken("dualQuaternion")
                                         ? "classicLinear"
                                         : "dualQuaternion"));
    evaluator.SetInteractiveOverrides({override});
    const RigExecRigPose dual = evaluator.Evaluate(UsdTimeCode::Default());
    evaluator.ClearInteractiveOverrides();
    CHECK(dual.valid);
    if (dual.bakedParityMismatches != 0) {
        ++failures;
        std::printf("FAIL dual-quaternion chunks: %zu baked/dynamic "
                    "mismatch(es)\n", dual.bakedParityMismatches);
        return;
    }
    for (const std::string &diagnostic : dual.diagnostics) {
        if (diagnostic.find("MoverFailed " + moverPath.GetString()) !=
            std::string::npos) {
            ++failures;
            std::printf("FAIL dual-quaternion chunks: the revision was "
                        "rejected: %s\n", diagnostic.c_str());
            return;
        }
    }
    // And it really did skin the other way, rather than agree about points
    // the override never reached.
    size_t moved = 0;
    for (const auto &[target, pointsLinear] : linear.movedProperties) {
        const auto found = dual.movedProperties.find(target);
        if (found != dual.movedProperties.end() &&
            found->second != pointsLinear) {
            ++moved;
        }
    }
    if (!moved) {
        ++failures;
        std::printf("FAIL dual-quaternion chunks: the override changed no "
                    "published points\n");
        return;
    }
    std::printf("  dual-quaternion chunks: parity held, %zu published "
                "target(s) moved\n", moved);
}

/// A revision whose partition no longer describes its layout is run WHOLE,
/// and lands where the chunks would have landed.
///
/// The fallback is the safety net under the whole design -- the keys are
/// trusted only while they are provably current -- and no stage can drive it
/// today: the layout cache preserves its handle for a binding that did not
/// move, and one that did move re-cuts in the prologue. So the fixture makes
/// the partition disagree with the packet by hand, which is the one thing a
/// frame cannot do to itself, and then asks the question the fallback exists
/// to answer: are the points the same ones?
void
TestAStalePartitionRunsTheRevisionWhole(const std::string &stagePath)
{
    const BuiltProgram built = Build(stagePath);
    CHECK(built.program != nullptr);
    if (!built.program) {
        return;
    }
    RigExecRigPose chunkedPose;
    CHECK(built.program->Run(UsdTimeCode::Default(), &chunkedPose));
    // The program's own state, which is what a stale partition is a property
    // of. Nothing else can reach it, and a fallback nothing exercises is a
    // fallback nobody knows the value of.
    RigExecBakedProgramImpl &B =
        const_cast<RigExecBakedProgramImpl &>(built.program->GetStepGraph());
    RigExecBakedProgramImpl::GeomRevision *stale = nullptr;
    for (RigExecBakedProgramImpl::GeomChain &chain : B.chains) {
        for (RigExecBakedProgramImpl::GeomRevision &revision :
                 chain.revisions) {
            if (revision.chunked && !stale) {
                stale = &revision;
            }
        }
    }
    if (!stale) {
        // RIGEXEC_BAKED_MAX_CHUNKS=1 cuts nothing, and then there is no
        // partition to invalidate.
        std::printf("  stale partition: no chunked revision to test\n");
        return;
    }
    // One element more than the layout the keys were cut from, which is what
    // a weight-paint edit the epoch let through would look like from here.
    ++stale->partitionIndexCount;
    // And something for the revision to execute for, since a revision that
    // does not execute never reaches the fuse's decision at all.
    stale->ran = false;

    RigExecRigPose wholePose;
    CHECK(built.program->Run(UsdTimeCode::Default(), &wholePose));
    CHECK(stale->partitionStale);
    CHECK(stale->executed);
    CHECK(stale->resultStatus != TfToken("moverFailed"));
    // The chunks stood down; the fuse did the work.
    for (const RigExecBakedProgramImpl::GeomChunk &chunk : stale->chunks) {
        CHECK(!chunk.ok);
    }
    size_t differed = 0;
    for (const auto &[target, points] : chunkedPose.movedProperties) {
        const auto found = wholePose.movedProperties.find(target);
        if (found == wholePose.movedProperties.end() ||
            found->second != points) {
            ++differed;
        }
    }
    CHECK(chunkedPose.movedProperties.size() ==
          wholePose.movedProperties.size());
    if (differed) {
        ++failures;
        std::printf("FAIL stale partition: %zu published target(s) differ "
                    "from the chunked run\n", differed);
        return;
    }
    std::printf("  stale partition: %s ran whole and published the chunked "
                "points\n", stale->moverPath.GetString().c_str());
}

/// The predicate the fold hands the fuse, on the one input the pose walk
/// cannot produce.
///
/// A provider matrix is always finite -- RigExecPointsToMatrix leaves the
/// identity where it cannot solve -- so no rig fixture can put a non-finite
/// matrix in front of the fold. The check is still the one the dynamic path's
/// assembler makes over the same table, and a fold that stopped making it
/// would hand a NaN to the kernel, so it is asserted directly.
void
TestTheInfluenceValidityCheckRejectsWhatTheAssemblerRejects()
{
    std::vector<GfMatrix4d> table(3, GfMatrix4d(1.0));
    CHECK(RigExecSkinTransformsAreUsable(table.data(), table.size()));
    table[1][2][0] = std::nan("");
    CHECK(!RigExecSkinTransformsAreUsable(table.data(), table.size()));
    table[1][2][0] = 0.0;
    table[1][1][3] = 0.5;  // not affine
    CHECK(!RigExecSkinTransformsAreUsable(table.data(), table.size()));
    // An empty table is what a skin mover with no influences assembles, and
    // the assembler rejects that too.
    CHECK(!RigExecSkinTransformsAreUsable(table.data(), 0));
}

// Cone re-execution (§7).
// The one part of the program whose correctness a published pose cannot
// show: a frame that re-ran everything publishes exactly what a frame that
// skipped the right half publishes, so every assertion below is in two
// halves -- the values are the values the dynamic path produces, AND the run
// actually skipped something. Either alone passes over the defect the other
// one catches.

/// The two closures Build computes are closures (§7).
///
/// Neither can be checked against a published pose -- a cone that is too big
/// is slow and right, and one that is too small is fast and wrong in a way
/// only some frame of some rig will ever show. So they are checked as what
/// they claim to be: cone[c] holds c and every successor's cone, restore[c]
/// holds every restore of everything in it, and a step's restore
/// predecessors are predecessors.
void
TestTheConeClosuresAreSound(const BuiltProgram &built, const char *name)
{
    if (!built.program) {
        ++failures;
        std::printf("FAIL %s: no program to check cones of\n", name);
        return;
    }
    const RigExecBakedProgramImpl &B = built.program->GetStepGraph();
    const size_t clusters = B.clustering.clusters.size();
    CHECK(B.cones.cone.size() == clusters);
    size_t broken = 0;
    for (size_t c = 0; c < clusters; ++c) {
        if (!B.cones.cone[c].Test(int(c))) {
            ++broken;
        }
        for (const int succ : B.clustering.clusters[c].succs) {
            if (!B.cones.cone[c].Test(succ)) {
                ++broken;
            }
        }
        for (size_t d = 0; d < clusters; ++d) {
            if (B.cones.cone[c].Test(int(d))) {
                for (size_t e = 0; e < clusters; ++e) {
                    if (B.cones.cone[d].Test(int(e)) &&
                        !B.cones.cone[c].Test(int(e))) {
                        ++broken;
                    }
                }
            }
        }
    }
    if (broken) {
        ++failures;
        std::printf("FAIL %s: %zu cone closure violation(s)\n", name, broken);
        return;
    }
    size_t reach = 0;
    for (size_t c = 0; c < clusters; ++c) {
        reach += B.cones.cone[c].Count();
    }
    std::printf("  %s: %zu cluster(s), %.1f cluster(s) in the average cone\n",
                name, clusters,
                clusters ? double(reach) / double(clusters) : 0.0);
}

/// Every pose write has storage of its own, and every read names a version
/// some writer produced (§3.1).
///
/// The property the restore closure used to buy at runtime and the versioned
/// storage now has by construction. Three things make it true, and all three
/// are checked here against the program's own tables rather than against a
/// number: no two write sites share an entry (that is what SSA IS), every
/// read and every carry names an entry that exists and is not a later
/// version than the reader's own point, and the last-version table really
/// does name the last write of each slot.
void
TestEveryPoseWriteHasItsOwnStorage(const BuiltProgram &built,
                                   const char *name)
{
    if (!built.program) {
        ++failures;
        std::printf("FAIL %s: no program to check versions of\n", name);
        return;
    }
    const RigExecBakedProgramImpl &B = built.program->GetStepGraph();
    const size_t slots = B.paths.size();
    CHECK(B.finLast.size() == slots);
    CHECK(B.baseLast.size() == slots);
    // The layout: entry i is slot i's first version (the compose writes it),
    // entry n + i is its LAST -- dense, because that is what the matrices
    // and the publication read -- and everything past 2n is a version in
    // between. So a write site's entry is never below n, an entry in
    // [n, 2n) belongs to the slot it is named after, and every entry past
    // 2n belongs to exactly one site.
    std::vector<int> finWriter(B.fin.size(), -1), baseWriter(B.base.size(), -1);
    std::vector<uint32_t> liveFin(slots), liveBase(slots);
    for (size_t i = 0; i < slots; ++i) {
        liveFin[i] = uint32_t(i);
        liveBase[i] = uint32_t(i);
    }
    size_t broken = 0, writes = 0;
    const auto claim = [&broken, &writes](std::vector<int> *writer,
                                          uint32_t entry, size_t slots,
                                          size_t slot, int commit) {
        ++writes;
        if (entry >= writer->size() || entry < slots ||
            (entry < 2 * slots && entry != slots + slot)) {
            ++broken;
            return;
        }
        if ((*writer)[entry] >= 0) {
            ++broken;  // two commits writing one entry: not SSA
        }
        (*writer)[entry] = commit;
    };
    const auto names = [&broken](const std::vector<RigExecPointFrame> &table,
                                 uint32_t entry) {
        if (entry >= table.size()) {
            ++broken;
        }
    };
    for (size_t w = 0; w < B.commits.size(); ++w) {
        const RigExecBakedCommit &commit = B.commits[w];
        CHECK(commit.slotReads.size() == commit.slots.size());
        CHECK(commit.descendantReads.size() == commit.propagate.size());
        for (size_t pos = 0; pos < commit.slots.size(); ++pos) {
            const size_t slot = size_t(commit.slots[pos]);
            // The read is the version live where the commit runs, and the
            // carry is the version live where the write-back reaches this
            // site -- the same thing for a candidate, which is written
            // before any descendant is.
            if (commit.slotReads[pos] != liveFin[slot] ||
                commit.slotCarry[pos] != liveFin[slot]) {
                ++broken;
            }
            claim(&finWriter, commit.slotWrites[pos], slots, slot,
                  int(w));
            liveFin[slot] = commit.slotWrites[pos];
            if (commit.solverOutput) {
                if (commit.slotBaseCarry[pos] != liveBase[slot]) ++broken;
                claim(&baseWriter, commit.slotBaseWrites[pos], slots,
                      slot, int(w));
                liveBase[slot] = commit.slotBaseWrites[pos];
            }
        }
        for (size_t k = 0; k < commit.propagate.size(); ++k) {
            const size_t slot = size_t(commit.propagate[k].first);
            names(B.fin, commit.descendantReads[k]);
            names(B.fin, commit.closestReads[k]);
            if (commit.descendantCarry[k] != liveFin[slot]) {
                ++broken;
            }
            claim(&finWriter, commit.descendantWrites[k], slots, slot,
                  int(w));
            liveFin[slot] = commit.descendantWrites[k];
            if (commit.solverOutput) {
                if (commit.descendantBaseCarry[k] != liveBase[slot]) ++broken;
                claim(&baseWriter, commit.descendantBaseWrites[k], slots,
                      slot, int(w));
                liveBase[slot] = commit.descendantBaseWrites[k];
            }
        }
    }
    for (size_t i = 0; i < slots; ++i) {
        if (B.finLast[i] != liveFin[i] || B.baseLast[i] != liveBase[i]) {
            ++broken;
        }
    }
    // Every arena entry belongs to exactly one write site: the table holds
    // no storage nothing writes. (A dense last-version entry of a slot no
    // commit ever writes is the one exception, and it is the table's shape
    // rather than a hole in it.)
    for (size_t e = 2 * slots; e < B.fin.size(); ++e) {
        if (finWriter[e] < 0) ++broken;
    }
    for (size_t e = 2 * slots; e < B.base.size(); ++e) {
        if (baseWriter[e] < 0) ++broken;
    }
    if (broken) {
        ++failures;
        std::printf("FAIL %s: %zu version table violation(s)\n", name,
                    broken);
        return;
    }
    std::printf("  %s: %zu slot(s), %zu pose write(s) with storage of their "
                "own (%zu fin + %zu base entries, %zu KB)\n",
                name, slots, writes, B.fin.size(), B.base.size(),
                (B.fin.size() + B.base.size()) * sizeof(RigExecPointFrame) /
                    1024);
}

/// One rig, compiled and evaluating through the program.
struct LiveRig {
    UsdStageRefPtr stage;
    std::unique_ptr<RigExecRigEvaluator> evaluator;
};

LiveRig
OpenRig(const std::string &stagePath, RigExecEvaluationMode mode)
{
    LiveRig rig;
    rig.stage = UsdStage::Open(stagePath);
    if (!rig.stage) {
        return rig;
    }
    const SdfPath rigPath = FindRig(rig.stage);
    if (rigPath.IsEmpty()) {
        return rig;
    }
    rig.evaluator =
        std::make_unique<RigExecRigEvaluator>(rig.stage, rigPath);
    rig.evaluator->SetSolverGuidesEnabled(true);
    rig.evaluator->SetEvaluationMode(mode);
    std::vector<std::string> errors;
    if (!rig.evaluator->Compile(&errors)) {
        rig.evaluator.reset();
    }
    return rig;
}

/// The double an attribute holds at \p time, as a VtValue of its own type.
bool
AvarValue(const UsdAttribute &attribute, UsdTimeCode time, double bump,
          VtValue *out)
{
    if (!attribute) {
        return false;
    }
    if (attribute.GetTypeName() == SdfValueTypeNames->Double) {
        double value = 0;
        if (!attribute.Get(&value, time)) return false;
        *out = VtValue(value + bump);
        return true;
    }
    if (attribute.GetTypeName() == SdfValueTypeNames->Float) {
        float value = 0;
        if (!attribute.Get(&value, time)) return false;
        *out = VtValue(float(value + bump));
        return true;
    }
    return false;
}

/// A rig evaluated twice at one time re-executes nothing the second time.
///
/// The baked meaning of "nothing changed": the sources -- the avar table,
/// the chain's base points, the static packet -- compare equal, so the
/// closure is empty but for the steps that read outside the graph, and the
/// generation publishes last frame's values without recomputing one of them.
void
TestARepeatedTimeReExecutesNothing(const std::string &stagePath,
                                   const char *name)
{
    const LiveRig rig = OpenRig(stagePath, RigExecEvaluationMode::Baked);
    if (!rig.evaluator) {
        ++failures;
        std::printf("FAIL %s: does not compile\n", name);
        return;
    }
    RigExecRigEvaluator &E = *rig.evaluator;
    const RigExecRigPose first = E.Evaluate(UsdTimeCode(1));
    const RigExecRigPose second = E.Evaluate(UsdTimeCode(1));
    CHECK(first.valid && second.valid);
    if (E.GetBakedGenerationCount() != 2) {
        ++failures;
        std::printf("FAIL %s: %zu of 2 generation(s) came from the program\n",
                    name, E.GetBakedGenerationCount());
        return;
    }
    // Nothing moved, so nothing was deformed again.
    CHECK(second.moverGraphRevisionsExecuted == 0);
    CHECK(second.moverGraphRevisionsCreated == 0);
    // ... and the run knew it: a frame that re-ran every cluster would
    // satisfy the counter above by accident, because the fuse's executed
    // decision is a value comparison of its own.
    const size_t clusters = E.GetBakedClusterCount();
    const size_t ran = E.GetBakedClustersRunLastGeneration();
    CHECK(clusters > 0 && ran < clusters);
    rigExecTest::CompareEveryMap(&failures, std::string(name) +
                                     " repeated time", first, second);
    std::printf("  %s: a repeated time ran %zu of %zu cluster(s)\n", name,
                ran, clusters);
}

/// A control dragged and returned to its exact original value executes
/// nothing [S25].
///
/// The predicate a scheduler is tempted to use is "an override stands", and
/// it is wrong: an animator who drags a control and puts it back has changed
/// nothing, and the VdfNetwork the program replaced would not re-execute a
/// node for it. What decides here is the avar table compared by value, so
/// the generation under a standing override is bit-identical to the one
/// without it.
void
TestADragReturnedToItsValueExecutesNothing(const std::string &stagePath,
                                           const SdfPath &control,
                                           const TfToken &avar)
{
    const LiveRig rig = OpenRig(stagePath, RigExecEvaluationMode::Baked);
    if (!rig.evaluator) {
        ++failures;
        std::printf("FAIL drag-return: does not compile\n");
        return;
    }
    RigExecRigEvaluator &E = *rig.evaluator;
    const UsdPrim prim = rig.stage->GetPrimAtPath(control);
    VtValue original, dragged;
    if (!prim || !AvarValue(prim.GetAttribute(avar), UsdTimeCode(1), 0.0,
                            &original) ||
        !AvarValue(prim.GetAttribute(avar), UsdTimeCode(1), 1.5, &dragged)) {
        ++failures;
        std::printf("FAIL drag-return: no double or float %s on %s\n",
                    avar.GetText(), control.GetText());
        return;
    }
    E.Evaluate(UsdTimeCode(1));
    const RigExecRigPose settled = E.Evaluate(UsdTimeCode(1));

    E.SetInteractiveOverrides(
        {RigExecValueOverride{control, TfToken(), avar, dragged}});
    const RigExecRigPose moved = E.Evaluate(UsdTimeCode(1));
    CHECK(moved.moverGraphRevisionsExecuted > 0);

    // Back to where it started, and HELD there. The first generation after
    // the value moves back executes -- it moved -- and the one after it must
    // not, although the override is still standing.
    E.SetInteractiveOverrides(
        {RigExecValueOverride{control, TfToken(), avar, original}});
    const RigExecRigPose back = E.Evaluate(UsdTimeCode(1));
    CHECK(back.moverGraphRevisionsExecuted > 0);
    const RigExecRigPose held = E.Evaluate(UsdTimeCode(1));
    CHECK(held.moverGraphRevisionsExecuted == 0);
    CHECK(E.GetBakedClustersRunLastGeneration() < E.GetBakedClusterCount());
    if (E.GetBakedGenerationCount() != 5) {
        ++failures;
        std::printf("FAIL drag-return: %zu of 5 generation(s) came from the "
                    "program; the drag was not placed\n",
                    E.GetBakedGenerationCount());
        return;
    }
    // The pose under the standing override is the pose without it, to the
    // bit: the override put the control back where the stage has it.
    rigExecTest::CompareEveryMap(&failures, "drag returned to its value",
                                 settled, held);
    std::printf("  drag returned to its value: held ran %zu of %zu "
                "cluster(s), 0 revision(s) executed\n",
                E.GetBakedClustersRunLastGeneration(),
                E.GetBakedClusterCount());
}

/// The clusters a run may touch when \p dirty is what moved.
///
/// The bound a drag is asserted against, computed from the graph the program
/// actually holds rather than from a number: the forward closure of the
/// clusters a change can start, plus the clusters that read outside the
/// graph and are therefore dirty every run. A run that stays inside it
/// touched nothing the change could not reach.
size_t
ConeBound(const RigExecBakedProgramImpl &B,
          const std::vector<int> &dirty)
{
    const size_t clusters = B.clustering.clusters.size();
    RigExecBakedClusterSet closed;
    closed.Resize(clusters);
    RigExecBakedClusterSet seeds = B.cones.always;
    seeds.Resize(clusters);
    for (const int cluster : dirty) {
        if (cluster >= 0) {
            seeds.Set(cluster);
        }
    }
    // A step whose own baked input the standing override names is dirty
    // too, and on a rig where a drag lands on one -- a constraint weight, an
    // IK offset -- that widens what the drag may reach. A control avar is
    // not one of those, so on this fixture the test is asked after the run
    // and the set comes back empty; it is here so that the bound stays a
    // bound on a rig where it does not.
    for (const int index : B.cones.overrideSteps) {
        const RigExecBakedStep &step = B.steps[size_t(index)];
        for (const int input : step.overrideInputs) {
            if (size_t(input) < B.overridden.size() &&
                B.overridden[size_t(input)]) {
                seeds.Set(step.cluster);
                break;
            }
        }
    }
    for (size_t c = 0; c < clusters; ++c) {
        if (seeds.Test(int(c))) {
            closed.Union(B.cones.cone[c]);
        }
    }
    return closed.Count();
}

/// The steps a run may touch when \p dirty (step indices) is what moved.
///
/// ConeBound's twin at the grain a live run closes at: the forward step
/// closure of the seeds, plus the steps that read outside the graph and the
/// steps a standing override names.
RigExecBakedClusterSet
StepConeBound(const RigExecBakedProgramImpl &B,
              const std::vector<int> &dirty)
{
    const size_t steps = B.steps.size();
    const size_t words = B.cones.stepWords;
    RigExecBakedClusterSet seeds;
    seeds.Resize(steps);
    seeds.Union(B.cones.alwaysSteps);
    for (const int index : dirty) {
        seeds.Set(index);
    }
    for (const int index : B.cones.overrideSteps) {
        const RigExecBakedStep &step = B.steps[size_t(index)];
        for (const int input : step.overrideInputs) {
            if (size_t(input) < B.overridden.size() &&
                B.overridden[size_t(input)]) {
                seeds.Set(index);
                break;
            }
        }
    }
    RigExecBakedClusterSet closed;
    closed.Resize(steps);
    for (size_t s = 0; s < steps; ++s) {
        if (!seeds.Test(int(s))) {
            continue;
        }
        for (size_t w = 0; w < words && w < closed.words.size(); ++w) {
            closed.words[w] |= B.cones.stepCone[s * words + w];
        }
    }
    return closed;
}

/// A constraint dirtied while its target's compose is clean re-runs its own
/// cone and nothing else (§3.1).
///
/// An override on a constraint's own input moves nothing upstream of it: the
/// controls are where they were, so the compose that produced its target's
/// frame is clean. That frame used to be READ-MODIFY-WRITTEN in place, so the
/// one storage at the end of a run held the constrained value and not the
/// composed one -- re-running the constraint over it would have constrained a
/// constrained frame, and the restore closure put the whole compose (and
/// everything downstream of it, which on a biped is the program) back into
/// the run to prevent exactly that. Versioned storage ends it: the constraint
/// reads the version its program point names, that version is still in the
/// entry the compose wrote it to, and the run is the constraint's own cone.
///
/// Both halves are asserted. The pose must equal a path that never skipped
/// anything -- which is what a missing version would break -- and the run
/// must be strictly smaller than the program, which is what the restore
/// closure made impossible.
void
TestAConstraintDragRunsOnlyItsCone(const std::string &stagePath,
                                   const SdfPath &mover,
                                   const TfToken &input)
{
    const UsdStageRefPtr probe = UsdStage::Open(stagePath);
    CHECK(probe);
    if (!probe) {
        return;
    }
    const UsdPrim prim = probe->GetPrimAtPath(mover);
    VtValue dragged;
    if (!prim || !AvarValue(prim.GetAttribute(input), UsdTimeCode(1), -0.25,
                            &dragged)) {
        ++failures;
        std::printf("FAIL constraint-drag: no double or float %s on %s\n",
                    input.GetText(), mover.GetText());
        return;
    }
    const std::vector<RigExecValueOverride> overrides = {
        RigExecValueOverride{mover, TfToken(), input, dragged}};

    // The path that skips: warmed up first, so its next generation has last
    // frame's values to keep.
    const LiveRig baked = OpenRig(stagePath, RigExecEvaluationMode::Baked);
    // The path that never skips, and never baked: one evaluator, one
    // generation, the whole exec walk.
    const LiveRig reference =
        OpenRig(stagePath, RigExecEvaluationMode::ExecReference);
    if (!baked.evaluator || !reference.evaluator) {
        ++failures;
        std::printf("FAIL constraint-drag: does not compile\n");
        return;
    }
    baked.evaluator->Evaluate(UsdTimeCode(1));
    baked.evaluator->Evaluate(UsdTimeCode(1));
    baked.evaluator->SetInteractiveOverrides(overrides);
    const RigExecRigPose constrained =
        baked.evaluator->Evaluate(UsdTimeCode(1));
    if (baked.evaluator->GetBakedGenerationCount() != 3) {
        ++failures;
        std::printf("FAIL constraint-drag: %zu of 3 generation(s) came from "
                    "the program\n",
                    baked.evaluator->GetBakedGenerationCount());
        return;
    }
    const size_t ran = baked.evaluator->GetBakedClustersRunLastGeneration();
    const size_t clusters = baked.evaluator->GetBakedClusterCount();
    CHECK(ran > 0);
    CHECK(clusters > 0 && ran < clusters);

    reference.evaluator->SetInteractiveOverrides(overrides);
    const RigExecRigPose expected =
        reference.evaluator->Evaluate(UsdTimeCode(1));
    CHECK(expected.valid && constrained.valid);
    rigExecTest::CompareEveryMap(&failures, "constraint input drag", expected,
                                 constrained);
    std::printf("  constraint input drag: ran %zu of %zu cluster(s) and "
                "published the dynamic path's pose\n", ran, clusters);
}

/// A leaf control drags its own cone: its compose subtree, the pose steps
/// that read it, their matrices, and the skin chunks whose key holds one of
/// their joints.
///
/// The interactive case the whole graph exists for, and the one a number
/// cannot state: which clusters those are is a property of the rig, so the
/// bound is computed from the program's own cones -- the forward closure of
/// the compose cluster the control's avars feed, plus what is dirty every
/// run whatever happened. A run inside that bound touched nothing the drag
/// could not reach; a run outside it is a cone that leaks.
void
TestALeafControlDragRunsOnlyItsCone(const std::string &stagePath,
                                    const SdfPath &control,
                                    const TfToken &avar)
{
    const UsdStageRefPtr probe = UsdStage::Open(stagePath);
    CHECK(probe);
    if (!probe) {
        return;
    }
    const UsdPrim prim = probe->GetPrimAtPath(control);
    VtValue dragged;
    if (!prim || !AvarValue(prim.GetAttribute(avar), UsdTimeCode(1), 3.0,
                            &dragged)) {
        ++failures;
        std::printf("FAIL leaf-drag: no double or float %s on %s\n",
                    avar.GetText(), control.GetText());
        return;
    }
    const std::vector<RigExecValueOverride> overrides = {
        RigExecValueOverride{control, TfToken(), avar, dragged}};

    const LiveRig baked = OpenRig(stagePath, RigExecEvaluationMode::Baked);
    const LiveRig reference =
        OpenRig(stagePath, RigExecEvaluationMode::ExecReference);
    if (!baked.evaluator || !reference.evaluator) {
        ++failures;
        std::printf("FAIL leaf-drag: does not compile\n");
        return;
    }
    baked.evaluator->Evaluate(UsdTimeCode(1));
    baked.evaluator->Evaluate(UsdTimeCode(1));
    const RigExecBakedProgram *program = baked.evaluator->GetBakedProgram();
    if (!program) {
        ++failures;
        std::printf("FAIL leaf-drag: the rig did not bake\n");
        return;
    }
    const RigExecBakedProgramImpl &B = program->GetStepGraph();
    const auto slot = B.index.find(control);
    if (slot == B.index.end()) {
        ++failures;
        std::printf("FAIL leaf-drag: %s is not a provider slot\n",
                    control.GetText());
        return;
    }

    baked.evaluator->SetInteractiveOverrides(overrides);
    const RigExecRigPose moved = baked.evaluator->Evaluate(UsdTimeCode(1));
    if (baked.evaluator->GetBakedGenerationCount() != 3) {
        ++failures;
        std::printf("FAIL leaf-drag: %zu of 3 generation(s) came from the "
                    "program\n", baked.evaluator->GetBakedGenerationCount());
        return;
    }
    const size_t ran = baked.evaluator->GetBakedClustersRunLastGeneration();
    const size_t clusters = baked.evaluator->GetBakedClusterCount();
    // Exactly what the drag moves: the eleven avars of one provider, which
    // reach the graph through the compose step that reads them.
    const size_t bound =
        ConeBound(B, {B.cones.avarCluster[size_t(slot->second)]});
    CHECK(ran > 0);
    // The bound is a property of the rig. On the skinned biped a wrist
    // avar's compose cone is the whole clustered graph (Linux CI: 38 of 38),
    // so requiring a proper subset is asking for a skip the cone does not
    // have. A leak is ran > bound; ran == clusters is fine when bound is
    // also the whole graph.
    if (bound < clusters) {
        CHECK(ran < clusters);
    }
    if (ran > bound) {
        ++failures;
        std::printf("FAIL leaf-drag: ran %zu cluster(s), and the drag's cone "
                    "is %zu\n", ran, bound);
    }
    // The drag really moved the rig, rather than the cone being small
    // because nothing happened.
    CHECK(moved.moverGraphRevisionsExecuted > 0);

    // And per STEP, which is the grain the run closed at: the steps it ran
    // are the drag's step cone and no more, and a clean step packed into a
    // cluster the run dispatched was skipped rather than run beside its
    // dirty cluster-mate. The count survives the cone verifier's forced
    // second pass (RigExecBakedRunStatistics puts it back); the sets do not,
    // so they are read only when that pass did not run.
    const RigExecBakedClusterSet stepBound =
        StepConeBound(B, {B.cones.avarStep[size_t(slot->second)]});
    const size_t stepsRan = B.lastClosedSteps;
    CHECK(stepsRan > 0);
    if (stepsRan > stepBound.Count()) {
        ++failures;
        std::printf("FAIL leaf-drag: closed %zu step(s), and the drag's "
                    "step cone is %zu\n", stepsRan, stepBound.Count());
    }
    if (!RigExecBakedVerifyConesRequested()) {
        size_t packed = 0;
        for (size_t s = 0; s < B.steps.size(); ++s) {
            const bool stepClosed = B.closedSteps.Test(int(s));
            CHECK(!stepClosed || stepBound.Test(int(s)));
            CHECK(!stepClosed || B.closed.Test(B.steps[s].cluster));
            packed += B.closed.Test(B.steps[s].cluster) ? 1 : 0;
        }
        // Every dispatched cluster holds a closed step.
        for (size_t c = 0; c < B.clustering.clusters.size(); ++c) {
            if (!B.closed.Test(int(c))) {
                continue;
            }
            bool holds = false;
            for (const int member : B.clustering.clusters[c].members) {
                holds = holds || B.closedSteps.Test(member);
            }
            CHECK(holds);
        }
        std::printf("  leaf control drag: closed %zu of %zu step(s) "
                    "(step cone %zu); the clusters holding them pack %zu\n",
                    stepsRan, B.steps.size(), stepBound.Count(), packed);
    }

    reference.evaluator->SetInteractiveOverrides(overrides);
    const RigExecRigPose expected =
        reference.evaluator->Evaluate(UsdTimeCode(1));
    CHECK(expected.valid && moved.valid);
    rigExecTest::CompareEveryMap(&failures, "leaf control drag", expected,
                                 moved);
    std::printf("  leaf control drag: ran %zu of %zu cluster(s), cone bound "
                "%zu, and published the dynamic path's pose\n",
                ran, clusters, bound);
}

/// A program that legitimately holds a NaN is still a program whose cone the
/// verifier can prove (§8.3).
///
/// A mover whose inputs the kernel rejects publishes the packet it rejected,
/// NaN and all -- that is how the pass-through diagnostic can name the value
/// it refused -- so a non-finite number is ordinary state for a CORRECT
/// program here, not a symptom of anything. The verifier compares two runs
/// of that program field by field, and `==` says a NaN differs from itself:
/// comparing with it reported three "baked cone mismatch" lines on a
/// generation whose cone had skipped nothing, turned a correct parity run
/// red, and would have sent whoever debugged the next real cone defect after
/// the wrong thing entirely. So this asks for both halves at once -- no cone
/// mismatch, and the same pose an evaluator that never baked publishes --
/// with the NaN placed the way a manipulator would place it.
///
/// Only the cone entries prove the first half: with the verifier off there
/// is no second run to disagree with the first. That is what
/// testRigExecBakedScheduleCones_serial and _parallel are registered for,
/// and the line this prints says which of the two it ran.
void
TestANonFiniteValueIsNotAConeMismatch(const std::string &stagePath,
                                      const TfToken &input,
                                      const VtValue &value)
{
    const LiveRig baked = OpenRig(stagePath, RigExecEvaluationMode::Baked);
    const LiveRig reference =
        OpenRig(stagePath, RigExecEvaluationMode::ExecReference);
    if (!baked.evaluator || !reference.evaluator) {
        ++failures;
        std::printf("FAIL non-finite cone: does not compile\n");
        return;
    }
    const SdfPath mover = FindSkinMover(baked.stage);
    if (mover.IsEmpty()) {
        ++failures;
        std::printf("FAIL non-finite cone: no skin mover\n");
        return;
    }
    const std::vector<RigExecValueOverride> overrides = {
        RigExecValueOverride{mover, TfToken(), input, value}};

    // Warmed up, then dragged, then HELD: the generation that matters is the
    // third, where the NaN has already settled and the packet therefore
    // compares equal to the one the last run assembled. That comparison --
    // a NaN against the identical NaN -- is the whole subject.
    baked.evaluator->Evaluate(UsdTimeCode::Default());
    baked.evaluator->SetInteractiveOverrides(overrides);
    baked.evaluator->Evaluate(UsdTimeCode::Default());
    const RigExecRigPose held =
        baked.evaluator->Evaluate(UsdTimeCode::Default());
    reference.evaluator->SetInteractiveOverrides(overrides);
    const RigExecRigPose expected =
        reference.evaluator->Evaluate(UsdTimeCode::Default());
    CHECK(held.valid && expected.valid);
    if (baked.evaluator->GetBakedGenerationCount() != 3) {
        ++failures;
        std::printf("FAIL non-finite cone: %zu of 3 generation(s) came from "
                    "the program\n",
                    baked.evaluator->GetBakedGenerationCount());
        return;
    }
    // The value really did reach the program, rather than the fixture
    // agreeing about a generation in which nothing happened.
    bool rejected = false;
    for (const std::string &diagnostic : held.diagnostics) {
        rejected = rejected ||
                   diagnostic.find("MoverFailed " + mover.GetString()) !=
                       std::string::npos;
    }
    if (!rejected) {
        ++failures;
        std::printf("FAIL non-finite cone: the revision was not rejected\n");
        return;
    }
    size_t coneMismatches = 0;
    for (const std::string &diagnostic : held.diagnostics) {
        if (diagnostic.find("baked cone mismatch") != std::string::npos) {
            ++coneMismatches;
            if (coneMismatches <= 8) {
                std::printf("    %s\n", diagnostic.c_str());
            }
        }
    }
    CHECK(coneMismatches == 0);
    CHECK(held.bakedParityMismatches == 0);
    rigExecTest::CompareEveryMap(&failures, "a non-finite value held", expected,
                                 held);
    std::printf("  a non-finite %s: %zu cone mismatch(es), verifier %s\n",
                input.GetText(), coneMismatches,
                RigExecBakedVerifyConesRequested() ? "on" : "off");
}

/// An acceptance check that repeats Build's own: Build already refuses a
/// program the validator rejects, so a rig that baked passes this by
/// construction, and a rejected one arrives here as "did not bake" with the
/// reason in BuildStage's "not bakeable" line.
void
TestTheValidatorAcceptsTheProgram(const BuiltProgram &built, const char *name)
{
    CHECK(built.program != nullptr);
    if (!built.program) {
        std::printf("FAIL %s: did not bake\n", name);
        return;
    }
    std::string error;
    const bool valid =
        RigExecBakedValidateStepGraph(built.program->GetStepGraph(), &error);
    CHECK(valid);
    if (!valid) {
        std::printf("FAIL %s: %s\n", name, error.c_str());
    }
}

/// A step that reads or writes \p ranges, with a kind whose label names no
/// table, so a program of them needs nothing but its steps.
RigExecBakedStep
HandStep(std::vector<RigExecBakedSlotRange> reads,
         std::vector<RigExecBakedSlotRange> writes)
{
    RigExecBakedStep step;
    step.kind = RigExecBakedStepKind::VolumePlacements;
    step.object = 0;
    step.reads = std::move(reads);
    step.writes = std::move(writes);
    return step;
}

/// Edges, levels and a one-step-per-cluster schedule for a hand-built
/// program, the way Build derives them.
void
ScheduleByHand(RigExecBakedProgramImpl *B)
{
    RigExecBakedEdgeSweep sweep;
    RigExecBakedBuildStepEdges(B, &sweep);
    RigExecBakedAssignStepCosts(B);
    B->clustering = RigExecBakedBuildClusters(*B, 0.0);
    for (size_t index = 0; index < B->steps.size(); ++index) {
        B->steps[index].cluster = B->clustering.clusterOf[index];
    }
    B->clustering.topologicalOrder =
        RigExecBakedClusterTopologicalOrder(B->clustering);
}

/// Whether the validator rejects \p B with an error holding every one of
/// \p expected.
void
ExpectRejected(const RigExecBakedProgramImpl &B, const char *what,
               const std::vector<std::string> &expected)
{
    std::string error;
    const bool valid = RigExecBakedValidateStepGraph(B, &error);
    if (valid) {
        ++failures;
        std::printf("FAIL %s: the validator accepted it\n", what);
        return;
    }
    for (const std::string &text : expected) {
        if (error.find(text) == std::string::npos) {
            ++failures;
            std::printf("FAIL %s: \"%s\" is not in \"%s\"\n", what,
                        text.c_str(), error.c_str());
        }
    }
    std::printf("  %s: %s\n", what, error.c_str());
}

/// The shapes the edge sweep cannot see, each of which the validator must
/// refuse by name: a read whose producer is later, missing or the reader
/// itself; a phased-read prefix that reaches past its reader; edges that
/// point backward or disagree; and a cluster order that is incomplete or
/// out of order.
void
TestTheValidatorRejectsAMalformedGraph()
{
    using D = RigExecBakedSlotDomain;
    {
        // The well-formed control: step 1 reads what step 0 wrote.
        RigExecBakedProgramImpl B;
        B.steps.push_back(HandStep({}, {RigExecBakedOne(D::WeightPacket, 0)}));
        B.steps.push_back(HandStep({RigExecBakedOne(D::WeightPacket, 0)},
                                   {RigExecBakedOne(D::WeightPacket, 1)}));
        ScheduleByHand(&B);
        std::string error;
        CHECK(RigExecBakedValidateStepGraph(B, &error));
        CHECK(error.empty());
        CHECK(B.steps[1].preds == std::vector<int>{0});
    }
    {
        // The sweep raises no edge for it, so step 0 would read last run's
        // packet.
        RigExecBakedProgramImpl B;
        B.steps.push_back(HandStep({RigExecBakedOne(D::WeightPacket, 1)},
                                   {RigExecBakedOne(D::WeightPacket, 0)}));
        B.steps.push_back(HandStep({}, {RigExecBakedOne(D::WeightPacket, 1)}));
        ScheduleByHand(&B);
        CHECK(B.steps[0].preds.empty());
        ExpectRejected(B, "a later producer",
                       {"step 0 (VolumePlacements every volume weight) reads "
                        "WeightPacket[1]",
                        "no step before it writes",
                        "the first writer is step 1"});
    }
    {
        // A blend whose input solver is solved after it would read last
        // run's aggregate; in the other order the same two steps are valid.
        RigExecBakedProgramImpl B;
        B.steps.push_back(HandStep({RigExecBakedOne(D::Aggregate, 1)},
                                   {RigExecBakedOne(D::Aggregate, 0)}));
        B.steps.push_back(HandStep({}, {RigExecBakedOne(D::Aggregate, 1)}));
        ScheduleByHand(&B);
        CHECK(B.steps[0].preds.empty());
        ExpectRejected(B, "a blend input solved after the blend",
                       {"step 0", "reads Aggregate[1]",
                        "no step before it writes",
                        "the first writer is step 1"});
        RigExecBakedProgramImpl ordered;
        ordered.steps.push_back(
            HandStep({}, {RigExecBakedOne(D::Aggregate, 1)}));
        ordered.steps.push_back(HandStep({RigExecBakedOne(D::Aggregate, 1)},
                                         {RigExecBakedOne(D::Aggregate, 0)}));
        ScheduleByHand(&ordered);
        std::string error;
        CHECK(RigExecBakedValidateStepGraph(ordered, &error));
        CHECK(error.empty());
    }
    {
        RigExecBakedProgramImpl B;
        B.steps.push_back(HandStep({}, {RigExecBakedOne(D::CommitTable, 0)}));
        B.steps.push_back(HandStep(
            {RigExecBakedRange(D::CommitTable, 0, 4)}, {}));
        ScheduleByHand(&B);
        ExpectRejected(B, "a range only partly produced",
                       {"step 1", "reads CommitTable[1], which no step writes",
                        "3 slots of CommitTable[0,4) are unproduced"});
    }
    {
        // Read-modify-write with no earlier writer reads its own last run.
        RigExecBakedProgramImpl B;
        B.steps.push_back(HandStep({RigExecBakedOne(D::Candidates, 2)},
                                   {RigExecBakedOne(D::Candidates, 2)}));
        ScheduleByHand(&B);
        ExpectRejected(B, "a step that is its own only producer",
                       {"step 0", "reads Candidates[2]",
                        "the first writer is step 0"});
    }
    {
        // A source domain needs no producer; Snapshots needs only to end at
        // the reader.
        RigExecBakedProgramImpl B;
        B.steps.push_back(HandStep({RigExecBakedOne(D::SolverPoints, 0),
                                    RigExecBakedOne(D::ChainBase, 0)},
                                   {RigExecBakedOne(D::Snapshots, 0)}));
        B.steps.push_back(HandStep({RigExecBakedRange(D::Snapshots, 0, 1)},
                                   {}));
        ScheduleByHand(&B);
        std::string error;
        CHECK(RigExecBakedValidateStepGraph(B, &error));
        B.steps[1].reads = {RigExecBakedRange(D::Snapshots, 0, 2)};
        ExpectRejected(B, "a phased read past its reader",
                       {"step 1", "Snapshots[0,2)",
                        "the records of steps at or after it"});
    }
    // A three-step chain, valid as built; each case below breaks one thing.
    const auto chain = [](RigExecBakedProgramImpl *B) {
        B->steps.push_back(
            HandStep({}, {RigExecBakedOne(D::WeightPacket, 0)}));
        B->steps.push_back(HandStep({RigExecBakedOne(D::WeightPacket, 0)},
                                    {RigExecBakedOne(D::WeightPacket, 1)}));
        B->steps.push_back(
            HandStep({RigExecBakedOne(D::WeightPacket, 1)}, {}));
        ScheduleByHand(B);
    };
    {
        RigExecBakedProgramImpl B;
        chain(&B);
        B.steps[0].preds = {1};
        B.steps[1].succs = {0, 2};
        ExpectRejected(B, "a backward predecessor",
                       {"step 0 (VolumePlacements every volume weight) "
                        "depends on step 1",
                        "not earlier in program order"});
    }
    {
        RigExecBakedProgramImpl B;
        chain(&B);
        B.steps[0].succs.clear();
        ExpectRejected(B, "a predecessor that does not know its successor",
                       {"step 1", "depends on step 0",
                        "does not list it as a successor"});
    }
    {
        RigExecBakedProgramImpl B;
        chain(&B);
        CHECK(B.clustering.clusters.size() == 3);
        std::reverse(B.clustering.topologicalOrder.begin(),
                     B.clustering.topologicalOrder.end());
        ExpectRejected(B, "a reversed cluster order",
                       {"the cluster order puts cluster",
                        "before its predecessor"});
    }
    {
        RigExecBakedProgramImpl B;
        chain(&B);
        B.clustering.topologicalOrder.pop_back();
        ExpectRejected(B, "an incomplete cluster order",
                       {"the cluster order holds 2 of 3 clusters"});
    }
    {
        RigExecBakedProgramImpl B;
        chain(&B);
        const int last = B.clustering.clusterOf[2];
        const int middle = B.clustering.clusterOf[1];
        B.clustering.clusters[size_t(last)].preds.clear();
        B.clustering.clusters[size_t(middle)].succs.clear();
        ExpectRejected(B, "a cluster edge missing under a step edge",
                       {"step 2", "depends on step 1",
                        "does not depend on cluster"});
    }
    {
        RigExecBakedProgramImpl B;
        chain(&B);
        std::swap(B.clustering.clusters[0].members,
                  B.clustering.clusters[1].members);
        ExpectRejected(B, "members filed under the wrong cluster",
                       {"holds step", "which the clustering puts elsewhere"});
    }
}

/// Pose bindings on stacked_solvers (FK solve, its commit, IK solve, its
/// commit, then the matrices), each edited to name a version that is written
/// later, twice or never. Every pose slot's first writer is its compose, so
/// the slot check passes all of these; only the version binding catches
/// them. Each case restores what it broke, and the restored program must
/// pass again.
void
TestTheValidatorRejectsALaterPoseVersion()
{
    BuiltProgram built = BuildStage(MakeStackedSolversStage());
    CHECK(built.program != nullptr);
    if (!built.program) {
        return;
    }
    // The program is this test's own, so it may be edited in place.
    RigExecBakedProgramImpl &B =
        const_cast<RigExecBakedProgramImpl &>(built.program->GetStepGraph());
    CHECK(B.commits.size() >= 2);
    if (B.commits.size() < 2 || B.commits.front().slotReads.empty() ||
        B.commits.front().slotCarry.empty() ||
        B.commits.front().slotWrites.empty() ||
        B.commits.back().slotWrites.empty()) {
        ++failures;
        std::printf("FAIL stacked_solvers: no two commits to bind across\n");
        return;
    }
    std::string error;
    CHECK(RigExecBakedValidateStepGraph(B, &error));
    RigExecBakedCommit &first = B.commits.front();
    const uint32_t later = B.commits.back().slotWrites.front();
    const auto passes = [&](const char *what) {
        std::string restored;
        if (!RigExecBakedValidateStepGraph(B, &restored)) {
            ++failures;
            std::printf("FAIL %s: rejected after the restore: %s\n", what,
                        restored.c_str());
        }
    };

    {
        const uint32_t bound = first.slotReads.front();
        first.slotReads.front() = later;
        ExpectRejected(B, "a commit read of a later commit's version",
                       {"(SolverCommit batch 0) is bound to PoseFin version",
                        "/Asset/Rig/Joints/", "writes at or after it"});
        first.slotReads.front() = bound;
        passes("a commit read of a later commit's version");
    }
    {
        // Both solves precede the last commit, so either one's control read
        // bound to its write-back is a read of the future.
        RigExecBakedProgramImpl::Solver *solver = nullptr;
        for (RigExecBakedProgramImpl::Solver &candidate : B.solvers) {
            if (!candidate.controlReads.empty()) {
                solver = &candidate;
                break;
            }
        }
        CHECK(solver != nullptr);
        if (solver) {
            const uint32_t bound = solver->controlReads.front();
            solver->controlReads.front() = later;
            ExpectRejected(B, "a solver control read of a later version",
                           {"(Solve /Asset/Rig/Solvers/",
                            "is bound to PoseFin version",
                            "writes at or after it"});
            solver->controlReads.front() = bound;
            passes("a solver control read of a later version");
        }
    }
    {
        // A carry may name its own commit's write-back -- a slot that is a
        // candidate and a descendant carries the first write into the
        // second -- but not another commit's later one.
        const uint32_t carried = first.slotCarry.front();
        first.slotCarry.front() = later;
        ExpectRejected(B, "a carry of another commit's later version",
                       {"(SolverCommit batch 0) is bound to PoseFin version",
                        "writes at or after it"});
        first.slotCarry.front() = first.slotWrites.front();
        std::string own;
        CHECK(RigExecBakedValidateStepGraph(B, &own));
        if (!own.empty()) {
            std::printf("FAIL a carry of its own commit's version: %s\n",
                        own.c_str());
        }
        first.slotCarry.front() = carried;
        passes("a carry of another commit's later version");
    }
    {
        // Every matrix runs after the whole walk, so no version here is
        // later than one; a last-version entry nothing writes is the same
        // stale read.
        int matrix = -1;
        uint32_t slot = 0;
        for (size_t index = 0; index < B.steps.size() && matrix < 0;
             ++index) {
            if (B.steps[index].kind != RigExecBakedStepKind::ProviderMatrix) {
                continue;
            }
            for (const RigExecBakedSlotRange &read : B.steps[index].reads) {
                if (read.domain == RigExecBakedSlotDomain::PoseFin &&
                    !read.IsEmpty()) {
                    matrix = int(index);
                    slot = read.begin;
                    break;
                }
            }
        }
        CHECK(matrix >= 0 && slot < B.finLast.size());
        if (matrix >= 0 && slot < B.finLast.size()) {
            const uint32_t last = B.finLast[slot];
            const uint32_t unwritten = uint32_t(B.fin.size());
            B.finLast[slot] = unwritten;
            ExpectRejected(B, "a last-version read of an unwritten version",
                           {"step " + std::to_string(matrix) +
                                " (ProviderMatrix ",
                            "is bound to PoseFin version " +
                                std::to_string(unwritten),
                            "which no step writes"});
            B.finLast[slot] = last;
            passes("a last-version read of an unwritten version");
        }
    }
    {
        // BindPoseVersions hands each entry to one writer; a second hides
        // the first's write from every reader bound to it.
        const uint32_t own = first.slotWrites.front();
        first.slotWrites.front() = later;
        ExpectRejected(B, "two commits writing one version",
                       {"(SolverCommit batch 1) writes PoseFin version " +
                            std::to_string(later),
                        "(SolverCommit batch 0) writes too"});
        first.slotWrites.front() = own;
        passes("two commits writing one version");
    }
    {
        const uint32_t last = B.finLast.back();
        B.finLast.pop_back();
        ExpectRejected(B, "a last-version table one slot short",
                       {"the last-version table holds " +
                        std::to_string(B.finLast.size()) + " PoseFin and " +
                        std::to_string(B.baseLast.size()) +
                        " PoseBase entries for " +
                        std::to_string(B.paths.size()) + " pose slots"});
        B.finLast.push_back(last);
        passes("a last-version table one slot short");
    }
}

/// Two root controls switched into each other's space: each compose group
/// reads the other's last version, so no emission order is valid and Build
/// must refuse the program. The compile rejects every such rig before a
/// bake, so no stage reaches Build's refusal and this does not exercise it.
/// It pins the composeCycle flag and the validator's verdict on the same
/// steps: the group emitted first reads a PosedM slot only the second
/// writes. One switch alone is the control: its source's group is moved
/// ahead of it instead.
void
TestASwitchCycleIsDetectedNotEmitted()
{
    using Switch = RigExecBakedProgramImpl::SpaceSwitch;
    const auto program = [](RigExecBakedProgramImpl *B, bool both) {
        B->paths = {SdfPath("/Rig/A"), SdfPath("/Rig/B")};
        B->parent = {-1, -1};
        B->propParent = {-1, -1};
        B->slotKind.assign(2, RigExecBakedSlotKind::FirstFramePose);
        B->spaceSwitchBySlot = {0, both ? 1 : -1};
        for (int slot = 0; slot < (both ? 2 : 1); ++slot) {
            Switch sw;
            sw.slot = slot;
            sw.sourceSlots = {1 - slot};
            Switch::FrameVersion read;
            read.anchor = 1 - slot;
            sw.sourceReads = {read};
            B->spaceSwitches.push_back(sw);
        }
        RigExecBakedBuildPoseSteps(B);
    };
    {
        RigExecBakedProgramImpl B;
        program(&B, /*both=*/false);
        CHECK(!B.composeCycle);
        // B's group is emitted first, because A's switch reads it.
        std::vector<int> composed;
        for (const RigExecBakedStep &step : B.steps) {
            if (step.kind == RigExecBakedStepKind::ComposeSubtree) {
                composed.push_back(B.composeGroups[size_t(step.object)].begin);
            }
        }
        CHECK(composed == std::vector<int>({1, 0}));
    }
    {
        RigExecBakedProgramImpl B;
        program(&B, /*both=*/true);
        CHECK(B.composeCycle);
        ScheduleByHand(&B);
        ExpectRejected(B, "a switch cycle left in slot order",
                       {"step 0", "reads PosedM[1]",
                        "the first writer is step 1"});
    }
}

/// Every reader of a chain's running points names exactly the version it
/// reads (RigExecBakedPointVersion): the RevisionDone and ChainDirty slots of
/// the revision below it, or the base for the chain's first revision, and no
/// other revision's buffer or flag. The fuse reads its own chunks' buffers
/// besides; ChainStatus reads every revision's status and no buffer.
/// Version 0 is the source ChainBase, which every chain step reads anyway,
/// so at the first revision only the absence of other versions is checked.
/// Returns how many readers past the first revision belong to a revision cut
/// into more than one chunk: the speculative chunks and the fuse whose
/// RevisionOut read spans several buffers.
size_t
TestEachChainReaderBindsOneVersion(const BuiltProgram &built,
                                   const char *name)
{
    CHECK(built.program != nullptr);
    if (!built.program) {
        return 0;
    }
    const RigExecBakedProgramImpl &B = built.program->GetStepGraph();
    const size_t revisions = B.revisionIndex.size();
    CHECK(B.revisionFuseStep.size() == revisions);
    if (B.revisionFuseStep.size() != revisions) {
        return 0;
    }
    for (size_t id = 0; id < revisions; ++id) {
        const int fuse = B.revisionFuseStep[id];
        CHECK(fuse >= 0 && size_t(fuse) < B.steps.size() &&
              B.steps[size_t(fuse)].kind ==
                  RigExecBakedStepKind::RevisionFuse &&
              B.steps[size_t(fuse)].object == int(id));
    }
    // The slots of \p domain that \p ranges name, in order.
    const auto slots = [](const std::vector<RigExecBakedSlotRange> &ranges,
                          RigExecBakedSlotDomain domain) {
        std::vector<int> out;
        for (const RigExecBakedSlotRange &range : ranges) {
            if (range.domain == domain) {
                for (uint32_t slot = range.begin; slot < range.end; ++slot) {
                    out.push_back(int(slot));
                }
            }
        }
        return out;
    };
    size_t checked = 0, stacked = 0, stackedChunked = 0;
    for (size_t index = 0; index < B.steps.size(); ++index) {
        const RigExecBakedStep &step = B.steps[index];
        const std::vector<int> done =
            slots(step.reads, RigExecBakedSlotDomain::RevisionDone);
        const std::vector<int> dirty =
            slots(step.reads, RigExecBakedSlotDomain::ChainDirty);
        const std::vector<int> out =
            slots(step.reads, RigExecBakedSlotDomain::RevisionOut);
        if (step.kind == RigExecBakedStepKind::ChainStatus) {
            const int c = step.object;
            std::vector<int> all;
            for (int id = B.chainRevisionBegin[size_t(c)];
                 id < B.chainRevisionEnd[size_t(c)]; ++id) {
                all.push_back(id);
            }
            CHECK(done == all);
            CHECK(out.empty());
            continue;
        }
        if (step.kind != RigExecBakedStepKind::RevisionChunk &&
            step.kind != RigExecBakedStepKind::RevisionFuse &&
            step.kind != RigExecBakedStepKind::RevisionStatic) {
            continue;
        }
        const int id = step.object;
        const auto &[c, r] = B.revisionIndex[size_t(id)];
        const bool reader =
            step.kind != RigExecBakedStepKind::RevisionStatic ||
            B.chains[size_t(c)].revisions[size_t(r)].weightCurrentPhase;
        std::vector<int> ownChunks;
        if (step.kind == RigExecBakedStepKind::RevisionFuse) {
            for (int k = 0; k < B.revisionChunkCount[size_t(id)]; ++k) {
                ownChunks.push_back(B.revisionChunkBase[size_t(id)] + k);
            }
        }
        CHECK(out == ownChunks);
        if (!reader) {
            CHECK(done.empty() && dirty.empty());
            continue;
        }
        ++checked;
        if (r == 0) {
            CHECK(done.empty() && dirty.empty());
            continue;
        }
        ++stacked;
        if (B.revisionChunkCount[size_t(id)] > 1) {
            ++stackedChunked;
        }
        const bool bound = done == std::vector<int>{id - 1} &&
                           dirty == std::vector<int>{id - 1};
        CHECK(bound);
        const int producer = B.revisionFuseStep[size_t(id - 1)];
        const bool ordered =
            std::binary_search(step.preds.begin(), step.preds.end(),
                               producer);
        CHECK(ordered);
        if (!bound || !ordered) {
            std::printf("FAIL %s: step %zu (%s) does not read version %d of "
                        "chain %d from step %d alone\n", name, index,
                        step.label.c_str(), r, c, producer);
        }
    }
    std::printf("  %s: %zu chain readers, %zu past the first revision "
                "(%zu of a multi-chunk revision)\n",
                name, checked, stacked, stackedChunked);
    return stackedChunked;
}

/// The upper-bound chain reads, rebuilt from the revision tables as the
/// reference the version reads must order identically to: each chunk, fuse
/// and current-phase assemble of revision id > first reads RevisionOut of
/// every earlier chunk of its chain, RevisionDone[first, id) and
/// ChainDirty(id - 1), the fuse reads every chunk of its chain up to its
/// own, and ChainStatus reads every chunk of its chain.
std::vector<RigExecBakedSlotRange>
OverApproximateChainReads(const RigExecBakedProgramImpl &B,
                          const RigExecBakedStep &step)
{
    std::vector<RigExecBakedSlotRange> reads;
    if (step.kind == RigExecBakedStepKind::ChainStatus) {
        const size_t c = size_t(step.object);
        if (B.chainRevisionEnd[c] > B.chainRevisionBegin[c]) {
            reads.push_back(RigExecBakedRange(
                RigExecBakedSlotDomain::RevisionDone, B.chainRevisionBegin[c],
                B.chainRevisionEnd[c]));
            reads.push_back(RigExecBakedRange(
                RigExecBakedSlotDomain::RevisionOut, B.chainChunkBegin[c],
                B.chainChunkEnd[c]));
        }
        return reads;
    }
    if (step.kind != RigExecBakedStepKind::RevisionChunk &&
        step.kind != RigExecBakedStepKind::RevisionFuse &&
        step.kind != RigExecBakedStepKind::RevisionStatic) {
        return reads;
    }
    const int id = step.object;
    const auto &[c, r] = B.revisionIndex[size_t(id)];
    if (step.kind == RigExecBakedStepKind::RevisionStatic &&
        !B.chains[size_t(c)].revisions[size_t(r)].weightCurrentPhase) {
        return reads;
    }
    const int first = B.chainRevisionBegin[size_t(c)];
    const int chunkFirst = B.chainChunkBegin[size_t(c)];
    const int chunkBase = B.revisionChunkBase[size_t(id)];
    if (step.kind == RigExecBakedStepKind::RevisionFuse) {
        reads.push_back(RigExecBakedRange(
            RigExecBakedSlotDomain::RevisionOut, chunkFirst,
            chunkBase + B.revisionChunkCount[size_t(id)]));
    } else if (id > first) {
        reads.push_back(RigExecBakedRange(RigExecBakedSlotDomain::RevisionOut,
                                          chunkFirst, chunkBase));
    }
    if (id > first) {
        reads.push_back(RigExecBakedRange(RigExecBakedSlotDomain::RevisionDone,
                                          first, id));
        reads.push_back(
            RigExecBakedOne(RigExecBakedSlotDomain::ChainDirty, id - 1));
    }
    return reads;
}

/// The transitive closure of the slot-conflict relation over \p reads and
/// \p writes, one bit row per step: step j depends on an earlier step i when
/// j reads what i writes, when both write one slot, or when j writes what i
/// read in a domain that is not versioned. That is the relation the edge
/// sweep raises, before it drops the edges a later writer implies, so its
/// closure is the closure of the sweep's `preds`.
std::vector<uint64_t>
ConflictClosure(const std::vector<std::vector<RigExecBakedSlotRange>> &reads,
                const std::vector<std::vector<RigExecBakedSlotRange>> &writes,
                size_t *words)
{
    const size_t count = reads.size();
    *words = (count + 63) / 64;
    std::vector<uint64_t> bits(count * *words, 0);
    struct Use {
        size_t step;
        uint32_t begin, end;
        bool write;
    };
    std::vector<std::vector<Use>> byDomain(RigExecBakedSlotDomainCount);
    for (size_t step = 0; step < count; ++step) {
        for (const RigExecBakedSlotRange &range : reads[step]) {
            if (!range.IsEmpty()) {
                byDomain[size_t(range.domain)].push_back(
                    {step, range.begin, range.end, false});
            }
        }
        for (const RigExecBakedSlotRange &range : writes[step]) {
            if (!range.IsEmpty()) {
                byDomain[size_t(range.domain)].push_back(
                    {step, range.begin, range.end, true});
            }
        }
    }
    for (size_t domain = 0; domain < byDomain.size(); ++domain) {
        const bool versioned = RigExecBakedIsVersionedDomain(
            RigExecBakedSlotDomain(domain));
        const std::vector<Use> &uses = byDomain[domain];
        for (const Use &later : uses) {
            uint64_t *row = &bits[later.step * *words];
            for (const Use &earlier : uses) {
                if (earlier.step >= later.step ||
                    earlier.begin >= later.end ||
                    later.begin >= earlier.end) {
                    continue;
                }
                if (earlier.write || (later.write && !versioned)) {
                    row[earlier.step / 64] |= uint64_t(1)
                                              << (earlier.step % 64);
                }
            }
        }
    }
    // Program order is a topological order of the relation.
    for (size_t step = 0; step < count; ++step) {
        uint64_t *row = &bits[step * *words];
        for (size_t pred = 0; pred < step; ++pred) {
            if ((row[pred / 64] >> (pred % 64)) & uint64_t(1)) {
                const uint64_t *closed = &bits[pred * *words];
                for (size_t w = 0; w < *words; ++w) {
                    row[w] |= closed[w];
                }
            }
        }
    }
    return bits;
}

/// One version read per chain reader orders the steps exactly as the
/// upper-bound chain reads would: the transitive closure of the program's
/// `preds` equals the closure of the conflict relation with
/// OverApproximateChainReads added back. The same closure over the current
/// declarations is checked first, which is what makes the conflict relation
/// a fair stand-in for the sweep.
void
TestThePointVersionsKeepTheOrder(const BuiltProgram &built, const char *name)
{
    CHECK(built.program != nullptr);
    if (!built.program) {
        return;
    }
    const RigExecBakedProgramImpl &B = built.program->GetStepGraph();
    const size_t count = B.steps.size();
    const Reachability actual(B.steps);
    std::vector<std::vector<RigExecBakedSlotRange>> reads(count), writes(count);
    for (size_t index = 0; index < count; ++index) {
        reads[index] = B.steps[index].reads;
        writes[index] = B.steps[index].writes;
    }
    const auto compare = [&](const char *what) {
        size_t words = 0;
        const std::vector<uint64_t> bits =
            ConflictClosure(reads, writes, &words);
        size_t differ = 0;
        for (size_t later = 0; later < count; ++later) {
            for (size_t earlier = 0; earlier < later; ++earlier) {
                const bool expected =
                    (bits[later * words + earlier / 64] >> (earlier % 64)) &
                    uint64_t(1);
                if (expected == actual.Ordered(earlier, later)) {
                    continue;
                }
                if (differ++ < 5) {
                    std::printf("FAIL %s (%s): step %zu (%s) -> step %zu "
                                "(%s) is %s\n", name, what, earlier,
                                B.steps[earlier].label.c_str(), later,
                                B.steps[later].label.c_str(),
                                expected ? "missing" : "new");
                }
            }
        }
        CHECK(differ == 0);
    };
    compare("current declarations");
    size_t added = 0;
    for (size_t index = 0; index < count; ++index) {
        for (const RigExecBakedSlotRange &range :
                 OverApproximateChainReads(B, B.steps[index])) {
            reads[index].push_back(range);
            ++added;
        }
    }
    compare("over-approximate declarations");
    size_t edges = 0;
    for (const RigExecBakedStep &step : B.steps) {
        edges += step.preds.size();
    }
    std::printf("  %s: %zu steps, %zu edges, closure unchanged by %zu "
                "over-approximate chain reads\n", name, count, edges, added);
}

/// A chain reader whose version read is missing, or whose producer is not
/// the fuse, is refused by name even though the sweep raised every edge the
/// old reads did. Each case restores what it broke.
void
TestTheValidatorRejectsAnUnboundPointVersion()
{
    BuiltProgram built = BuildStage(MakeStackedChainStage());
    CHECK(built.program != nullptr);
    if (!built.program) {
        return;
    }
    // The program is this test's own, so it may be edited in place.
    RigExecBakedProgramImpl &B =
        const_cast<RigExecBakedProgramImpl &>(built.program->GetStepGraph());
    CHECK(B.revisionIndex.size() == 3);
    int chunk = -1;
    for (size_t index = 0; index < B.steps.size(); ++index) {
        if (B.steps[index].kind == RigExecBakedStepKind::RevisionChunk &&
            B.steps[index].object == 2) {
            chunk = int(index);
        }
    }
    CHECK(chunk >= 0);
    if (chunk < 0 || B.revisionIndex.size() != 3) {
        return;
    }
    RigExecBakedStep &reader = B.steps[size_t(chunk)];
    const int producer = B.revisionFuseStep[1];
    const auto passes = [&](const char *what) {
        std::string restored;
        if (!RigExecBakedValidateStepGraph(B, &restored)) {
            ++failures;
            std::printf("FAIL %s: rejected after the restore: %s\n", what,
                        restored.c_str());
        }
    };
    std::string error;
    CHECK(RigExecBakedValidateStepGraph(B, &error));
    {
        const std::vector<RigExecBakedSlotRange> reads = reader.reads;
        reader.reads.erase(
            std::remove(reader.reads.begin(), reader.reads.end(),
                        RigExecBakedOne(RigExecBakedSlotDomain::RevisionDone,
                                        1)),
            reader.reads.end());
        CHECK(reader.reads.size() + 1 == reads.size());
        ExpectRejected(B, "a chunk without its version read",
                       {"(RevisionChunk", "reads point version 2 of chain 0",
                        "without declaring it"});
        reader.reads = reads;
        passes("a chunk without its version read");
    }
    {
        B.revisionFuseStep[1] = chunk;
        ExpectRejected(B, "a fuse table naming a chunk",
                       {"revision 1's fuse is recorded as",
                        "(RevisionChunk", "which is not its fuse"});
        B.revisionFuseStep[1] = producer;
        passes("a fuse table naming a chunk");
    }
    {
        reader.writes.push_back(
            RigExecBakedOne(RigExecBakedSlotDomain::ChainDirty, 1));
        ExpectRejected(B, "a second writer of a version",
                       {"(RevisionChunk", "writes ChainDirty[1]",
                        "which only", "(RevisionFuse", "may write"});
        reader.writes.pop_back();
        passes("a second writer of a version");
    }
    {
        reader.writes.push_back(
            RigExecBakedOne(RigExecBakedSlotDomain::RevisionDone, 3));
        ExpectRejected(B, "a version written past the revision table",
                       {"(RevisionChunk", "writes RevisionDone[3]",
                        "past the table's 3 revisions"});
        reader.writes.pop_back();
        passes("a version written past the revision table");
    }
    {
        // The edge alone, both directions, so the edge check stays quiet.
        std::vector<int> &preds = reader.preds;
        std::vector<int> &succs = B.steps[size_t(producer)].succs;
        preds.erase(std::find(preds.begin(), preds.end(), producer));
        succs.erase(std::find(succs.begin(), succs.end(), chunk));
        ExpectRejected(B, "a version read without its producer's edge",
                       {"(RevisionChunk", "reads RevisionDone[1] without an "
                        "edge from its producer", "(RevisionFuse"});
        preds.insert(std::lower_bound(preds.begin(), preds.end(), producer),
                     producer);
        succs.insert(std::lower_bound(succs.begin(), succs.end(), chunk),
                     chunk);
        passes("a version read without its producer's edge");
    }
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
    // Two SOLVER commits on one pose slot, which no example stage has and
    // which the storage rules below are otherwise vacuous about.
    const BuiltProgram stackedSolvers = BuildStage(MakeStackedSolversStage());
    TestTheGraphDescribesTheProgram(biped, "Biped");
    TestTheGraphDescribesTheProgram(animated, "Biped_anim");
    TestTheGraphDescribesTheProgram(spider, "spider_legs");
    TestTheGraphDescribesTheProgram(stacked, "stacked_revisions");
    TestTheGraphDescribesTheProgram(stackedSolvers, "stacked_solvers");
    TestTheValidatorAcceptsTheProgram(biped, "Biped");
    TestTheValidatorAcceptsTheProgram(animated, "Biped_anim");
    TestTheValidatorAcceptsTheProgram(spider, "spider_legs");
    TestTheValidatorAcceptsTheProgram(stacked, "stacked_revisions");
    TestTheValidatorAcceptsTheProgram(stackedSolvers, "stacked_solvers");
    TestTheValidatorAcceptsTheProgram(
        Build(examplesDir + "/04_BlendShapeFace.usda"), "04_BlendShapeFace");
    TestTheValidatorAcceptsTheProgram(
        Build(examplesDir + "/../tests/fixtures/oneloop_cross_domain.usda"),
        "oneloop_cross_domain");
    {
        // Rigs with blends, whose input aggregates are produced in the graph.
        const std::pair<std::string, const char *> blends[] = {
            {examplesDir + "/03_IkFkBlendClamp.usda", "03_IkFkBlendClamp"},
            {examplesDir + "/ArmRig.usda", "ArmRig"},
            {examplesDir + "/../tests/fixtures/oneloop_cross_domain.usda",
             "oneloop_cross_domain"},
        };
        for (const auto &[path, name] : blends) {
            const BuiltProgram built = Build(path);
            TestTheGraphDescribesTheProgram(built, name);
            TestTheValidatorAcceptsTheProgram(built, name);
        }
    }
    TestTheValidatorRejectsAMalformedGraph();
    TestTheValidatorRejectsALaterPoseVersion();
    TestTheValidatorRejectsAnUnboundPointVersion();
    {
        // The chain fixtures: one revision per chain (Biped, spider_legs),
        // three on one chain (stacked_revisions), many stacked chains
        // (Biped_stack), current-phase weights over a skinned chain
        // (oneloop_cross_domain, computed_weights), and blend shapes
        // (04_BlendShapeFace).
        const std::string fixtures = examplesDir + "/../tests/fixtures";
        const BuiltProgram bipedStack =
            Build(examplesDir + "/biped/Biped_stack.usda");
        const BuiltProgram face = Build(examplesDir + "/04_BlendShapeFace.usda");
        const BuiltProgram crossDomain =
            Build(fixtures + "/oneloop_cross_domain.usda");
        const BuiltProgram computedWeights =
            Build(fixtures + "/computed_weights.usda");
        const std::pair<const BuiltProgram *, const char *> chains[] = {
            {&biped, "Biped"},
            {&spider, "spider_legs"},
            {&stacked, "stacked_revisions"},
            {&bipedStack, "Biped_stack"},
            {&face, "04_BlendShapeFace"},
            {&crossDomain, "oneloop_cross_domain"},
            {&computedWeights, "computed_weights"},
        };
        for (const auto &[built, name] : chains) {
            TestEachChainReaderBindsOneVersion(*built, name);
            TestThePointVersionsKeepTheOrder(*built, name);
        }
    }
    TestASwitchCycleIsDetectedNotEmitted();
    TestTheClusteringIsSound(biped, "Biped");
    TestTheClusteringIsSound(spider, "spider_legs");
    TestTheClusteringIsSound(stacked, "stacked_revisions");
    TestTheClusteringIsSound(stackedSolvers, "stacked_solvers");
    TestTheConeClosuresAreSound(biped, "Biped");
    TestTheConeClosuresAreSound(spider, "spider_legs");
    TestTheConeClosuresAreSound(stacked, "stacked_revisions");
    TestTheConeClosuresAreSound(stackedSolvers, "stacked_solvers");
    TestEveryPoseWriteHasItsOwnStorage(biped, "Biped");
    TestEveryPoseWriteHasItsOwnStorage(spider, "spider_legs");
    TestEveryPoseWriteHasItsOwnStorage(stacked, "stacked_revisions");
    TestEveryPoseWriteHasItsOwnStorage(stackedSolvers, "stacked_solvers");
    TestTheReportIsDeterministic(examplesDir + "/biped/Biped.usda");
    TestTheInfluenceValidityCheckRejectsWhatTheAssemblerRejects();
    TestTheRangeFormDeformsLikeTheWholeArray("classicLinear");
    TestTheRangeFormDeformsLikeTheWholeArray("dualQuaternion");
    TestAChunkSeesOnlyItsOwnInfluences("classicLinear");
    TestAChunkSeesOnlyItsOwnInfluences("dualQuaternion");
    // The cut decision, both ways round. The rigs that bake today pose every
    // joint of a mesh at the same level, so with the rule alone nothing is
    // cut -- which is the right answer and would leave every assertion below
    // about a chunk true of no chunk at all. So the chunk fixtures run with
    // RIGEXEC_BAKED_CHUNK_ALWAYS=1, and the rule itself is asserted in both
    // environments.
    TestThePartitionIsCutOnlyWhereItPays(biped, "Biped", /*always=*/false);
    TestThePartitionIsCutOnlyWhereItPays(spider, "spider_legs",
                                         /*always=*/false);
    TfSetenv("RIGEXEC_BAKED_CHUNK_ALWAYS", "1");
    TestThePartitionIsCutOnlyWhereItPays(
        Build(examplesDir + "/biped/Biped.usda"), "Biped", /*always=*/true);
    TestTheVertexPartitionCoversEveryVertexOnce(
        examplesDir + "/biped/Biped.usda", "Biped", /*report=*/true);
    TestTheVertexPartitionCoversEveryVertexOnce(
        examplesDir + "/spider_legs_assembly_ref.usda", "spider_legs",
        /*report=*/false);
    TestARejectedSkinPacketPassesThroughLikeTheDynamicPath(
        examplesDir + "/biped/Biped.usda", "a non-finite defaultWeight",
        TfToken("inputs:defaultWeight"), VtValue(std::nanf("")));
    TestARejectedSkinPacketPassesThroughLikeTheDynamicPath(
        examplesDir + "/biped/Biped.usda", "a non-finite jointWeight",
        TfToken("rigExec:jointWeights"), VtValue(VtFloatArray()));
    TestADualQuaternionSkinChunksLikeTheDynamicPath(
        examplesDir + "/biped/Biped.usda");
    TestAStalePartitionRunsTheRevisionWhole(
        examplesDir + "/biped/Biped.usda");
    {
        // Chunked revisions: a fuse reads several chunks of its own, and in
        // oneloop_cross_domain the smooth reads the version a chunked skin's
        // fuse left.
        const BuiltProgram chunked = Build(examplesDir + "/biped/Biped.usda");
        TestEachChainReaderBindsOneVersion(chunked, "Biped (chunked)");
        TestThePointVersionsKeepTheOrder(chunked, "Biped (chunked)");
        const BuiltProgram crossDomain = Build(
            examplesDir + "/../tests/fixtures/oneloop_cross_domain.usda");
        TestEachChainReaderBindsOneVersion(crossDomain,
                                           "oneloop_cross_domain (chunked)");
        TestThePointVersionsKeepTheOrder(crossDomain,
                                         "oneloop_cross_domain (chunked)");
        // The one shape where speculative chunks read version r > 0 and a
        // fuse's own RevisionOut read spans several buffers.
        const BuiltProgram bipedStack =
            Build(examplesDir + "/biped/Biped_stack.usda");
        const size_t stackedChunked = TestEachChainReaderBindsOneVersion(
            bipedStack, "Biped_stack (chunked)");
        CHECK(stackedChunked > 0);
        TestThePointVersionsKeepTheOrder(bipedStack, "Biped_stack (chunked)");
    }
    TfSetenv("RIGEXEC_BAKED_CHUNK_ALWAYS", "0");
    TestTheDerivedCompareAgreesWithTheElementwiseOne(
        examplesDir + "/biped/Biped.usda");
    TestARebuiltProgramKeepsItsRunState(
        examplesDir + "/biped/Biped.usda", "Biped");
    TestARebuiltProgramKeepsItsRunState(
        examplesDir + "/04_BlendShapeFace.usda", "04_BlendShapeFace");
    TestARecompiledRigStillAgreesWithTheDynamicPath(
        examplesDir + "/biped/Biped.usda", "Biped");
    TestARecompiledRigStillAgreesWithTheDynamicPath(
        examplesDir + "/04_BlendShapeFace.usda", "04_BlendShapeFace");
    TestARepeatedTimeReExecutesNothing(examplesDir + "/biped/Biped.usda",
                                       "Biped");
    TestARepeatedTimeReExecutesNothing(
        examplesDir + "/spider_legs_assembly_ref.usda", "spider_legs");
    TestADragReturnedToItsValueExecutesNothing(
        examplesDir + "/biped/Biped.usda",
        SdfPath("/Biped/Rig/Main/Shot/Aux/Controls/M_Body"),
        TfToken("avars:ty"));
    TestAConstraintDragRunsOnlyItsCone(
        examplesDir + "/biped/Biped.usda",
        SdfPath("/Biped/Rig/Movers/twist_aims/elbowTwist_l_def_aim"),
        TfToken("inputs:defaultWeight"));
    TestALeafControlDragRunsOnlyItsCone(
        examplesDir + "/biped/Biped.usda",
        SdfPath("/Biped/Rig/Main/Shot/Aux/Controls/M_Body/M_Torso/"
                "M_Chest/M_ChestTop/L_Shldr/L_UpArmSwing/L_UpArm/"
                "L_LoArm/L_Hand"),
        TfToken("avars:rz"));
    TestANonFiniteValueIsNotAConeMismatch(
        examplesDir + "/biped/Biped.usda", TfToken("inputs:defaultWeight"),
        VtValue(std::nanf("")));
    if (failures) {
        std::printf("%d FAILURE(S)\n", failures);
        return 1;
    }
    std::printf("testRigExecBakedSchedule: all tests passed\n");
    return 0;
}
