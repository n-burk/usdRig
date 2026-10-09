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
#include "rigExec/bakedOpGraph.h"
#include "rigExec/bakedOpValues.h"
#include "rigExec/frozenContext.h"
#include "rigExec/frozenProgram.h"
#include "rigExec/frameCacheSparsity.h"
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
#include "pxr/base/tf/errorMark.h"
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
#include <array>
#include <cmath>
#include <set>
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <map>
#include <limits>
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

/// MakeStackedChainStage's three matrix movers, with M0 on a driver of its
/// own that moves at frames 1, 2 and 3, and M1 and M2 on one that stands
/// still. \p firstWeight is M0's inputs:defaultWeight; zero makes its points
/// the ones it entered with however its driver moves. \p failMiddle gives M1
/// an out-of-range weight at frame 2 only, so M1 passes through there and
/// applies again at frame 3.
UsdStageRefPtr
MakeAnimatedChainStage(float firstWeight, bool failMiddle)
{
    const UsdStageRefPtr stage = UsdStage::CreateInMemory();
    stage->DefinePrim(SdfPath("/Asset"), TfToken("Scope"));
    stage->DefinePrim(SdfPath("/Asset/Rig"), TfToken("RigExecRoot"));
    const UsdPrim moving = stage->DefinePrim(
        SdfPath("/Asset/Rig/Moving"), TfToken("RigExecControl"));
    for (int frame = 1; frame <= 3; ++frame) {
        moving.GetAttribute(TfToken("avars:tx"))
            .Set(double(frame), UsdTimeCode(frame));
    }
    const UsdPrim still = stage->DefinePrim(
        SdfPath("/Asset/Rig/Still"), TfToken("RigExecControl"));
    still.GetAttribute(TfToken("avars:ty")).Set(1.0);
    const SdfPath target("/Asset/Shape.points");
    const UsdPrim shape =
        stage->DefinePrim(target.GetPrimPath(), TfToken("Points"));
    shape.GetAttribute(TfToken("points"))
        .Set(VtVec3fArray{GfVec3f(0), GfVec3f(1, 0, 0), GfVec3f(0, 2, 0)});
    stage->DefinePrim(SdfPath("/Asset/Rig/Movers"), TfToken("Scope"));
    for (int i = 0; i < 3; ++i) {
        const UsdPrim mover = stage->DefinePrim(
            SdfPath("/Asset/Rig/Movers/M" + std::to_string(i)),
            TfToken("RigExecMatrixMover"));
        mover.ApplyAPI(TfToken("RigExecMoverAPI"));
        mover.GetRelationship(TfToken("rigExec:moves")).SetTargets({target});
        mover.GetRelationship(TfToken("rigExec:transform"))
            .SetTargets({i == 0 ? moving.GetPath() : still.GetPath()});
        UsdAttribute weight =
            mover.GetAttribute(TfToken("inputs:defaultWeight"));
        if (i == 0) {
            weight.Set(firstWeight);
        } else if (i == 1 && failMiddle) {
            weight.Set(1.0f, UsdTimeCode(1.0));
            weight.Set(2.0f, UsdTimeCode(2.0));
            weight.Set(1.0f, UsdTimeCode(3.0));
        } else {
            weight.Set(1.0f);
        }
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

    size_t WordCount() const { return _words; }
    const uint64_t *Row(size_t later) const { return &_bits[later * _words]; }

private:
    size_t _words;
    std::vector<uint64_t> _bits;
};

// Index exact typed slots, retaining each range site once per query. Overlap
// of several slots is still one range-pair conflict, as in the exhaustive test.
class WriteSiteIndex {
public:
    using Site=std::pair<size_t,size_t>;
    explicit WriteSiteIndex(const std::vector<RigExecBakedStep> &steps) {
        for(size_t step=0;step<steps.size();++step)
            for(size_t range=0;range<steps[step].writes.size();++range) {
                const auto &write=steps[step].writes[range];
                _all[write.domain].push_back({{step,range},write});
                if(!CanIndex(write))_unindexed[write.domain].push_back({{step,range},write});
                else for(uint32_t slot=write.begin;slot<write.end;++slot)
                    _sites[{write.domain,slot}].push_back({step,range});
            }
    }
    std::vector<Site> Overlapping(const RigExecBakedSlotRange &range) const {
        std::vector<Site> result;
        if(CanIndex(range))for(uint32_t slot=range.begin;slot<range.end;++slot) {
            const auto found=_sites.find({range.domain,slot});
            if(found!=_sites.end())result.insert(result.end(),found->second.begin(),found->second.end());
        }
        // Large and malformed ranges use bounded interval queries. Empty
        // intervals may still satisfy Overlaps' original inequalities.
        const auto &fallback=CanIndex(range)?_unindexed:_all;
        const auto found=fallback.find(range.domain);
        if(found!=fallback.end())for(const auto &site:found->second)
            if(site.second.Overlaps(range))result.push_back(site.first);
        std::sort(result.begin(),result.end());
        result.erase(std::unique(result.begin(),result.end()),result.end());
        return result;
    }
private:
    static bool CanIndex(const RigExecBakedSlotRange &range) {
        return !range.IsEmpty() && uint64_t(range.end)-range.begin<=4096;
    }
    std::map<std::pair<RigExecBakedSlotDomain,uint32_t>,std::vector<Site>> _sites;
    std::map<RigExecBakedSlotDomain,std::vector<std::pair<Site,RigExecBakedSlotRange>>> _all,_unindexed;
};

// The four exhaustive quantified conditions, with candidates restricted only
// by exact domain/slot overlap. Witnesses retain range multiplicity and order.
using AccessWitness=std::array<size_t,5>;
template<class Report>
void IndexedAccessViolations(const std::vector<RigExecBakedStep> &steps,
                             const Reachability &reachable,Report report)
{
    const WriteSiteIndex writes(steps);
    for(size_t step=0;step<steps.size();++step) {
        for(size_t r=0;r<steps[step].reads.size();++r) {
            const auto &read=steps[step].reads[r];
            const auto candidates=writes.Overlapping(read);
            if(!RigExecBakedIsSourceDomain(read.domain) && !read.IsEmpty()) {
                bool earlier=false;
                for(const auto &site:candidates)earlier=earlier || site.first<step;
                if(!earlier)report(AccessWitness{0,step,r,0,0});
            }
            for(const auto &site:candidates) {
                if(site.first<step && !reachable.Ordered(site.first,step))
                    report(AccessWitness{2,site.first,site.second,step,r});
                else if(site.first>step && !RigExecBakedIsVersionedDomain(read.domain) &&
                        !reachable.Ordered(step,site.first))
                    report(AccessWitness{3,step,r,site.first,site.second});
            }
        }
        for(size_t r=0;r<steps[step].writes.size();++r)
            for(const auto &site:writes.Overlapping(steps[step].writes[r]))
                if(site.first<step && !reachable.Ordered(site.first,step))
                    report(AccessWitness{1,site.first,site.second,step,r});
    }
}

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

    // (2--4) Producer existence and unordered write/write or read/write
    // conflicts, queried by the exact typed values each range names.
    const Reachability reachable(B.steps);
    size_t conflicts=0,races=0;
    IndexedAccessViolations(B.steps,reachable,[&](const AccessWitness &w) {
        const auto &earlier=B.steps[w[1]];
        if(w[0]==0) {
            const auto &read=earlier.reads[w[2]];
            ++failures;
            std::printf("FAIL %s: step %zu (%s) reads %s[%u,%u) which no earlier step writes\n",
                name,w[1],earlier.label.c_str(),RigExecBakedSlotDomainName(read.domain),read.begin,read.end);
            return;
        }
        const auto &later=B.steps[w[3]];
        if(w[0]==1) {
            const auto &write=later.writes[w[4]];
            if(++conflicts<=4)
                std::printf("FAIL %s: steps %zu (%s) and %zu (%s) both write %s[%u,%u) with no edge between them\n",
                    name,w[1],earlier.label.c_str(),w[3],later.label.c_str(),
                    RigExecBakedSlotDomainName(write.domain),write.begin,write.end);
        } else {
            const auto &range=w[0]==2?earlier.writes[w[2]]:earlier.reads[w[2]];
            if(++races<=4)
                std::printf("FAIL %s: step %zu (%s) %s %s[%u,%u) that unordered step %zu (%s) %s\n",
                    name,w[1],earlier.label.c_str(),w[0]==2?"writes":"reads",
                    RigExecBakedSlotDomainName(range.domain),range.begin,range.end,
                    w[3],later.label.c_str(),w[0]==2?"reads":"writes");
        }
    });
    failures+=int(conflicts);
    if(races) {
        ++failures;
        std::printf("FAIL %s: %zu unordered read/write slot overlap(s)\n",name,races);
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
        // A range of a range-pipelined revision reads range `part` of its
        // range-pipelined predecessor's own buffer, which never moves, and
        // so needs no RevisionDone: that one slot is exempt.
        int rangeSlot = -1;
        if (step.kind == RigExecBakedStepKind::RevisionChunk &&
            step.object > 0 && size_t(step.object) < revisions) {
            const auto &[c, r] = B.revisionIndex[size_t(step.object)];
            const auto &chain = B.chains[size_t(c)];
            if (r > 0 && chain.revisions[size_t(r)].rangeRole &&
                chain.revisions[size_t(r) - 1].rangeRole) {
                rangeSlot =
                    B.revisionChunkBase[size_t(step.object) - 1] + step.part;
            }
        }
        for (const RigExecBakedSlotRange &read : step.reads) {
            if (read.domain == RigExecBakedSlotDomain::RevisionOut) {
                if (rangeSlot >= 0 && read.begin == uint32_t(rangeSlot) &&
                    read.end == uint32_t(rangeSlot) + 1) {
                    continue;
                }
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
        for (size_t pos = 0; pos < commit.slots.size(); ++pos) {
            if (head) require(RigExecBakedSlotDomain::PoseFin, commit.slotReads[pos],
                              "measures a delta against");
            require(RigExecBakedSlotDomain::PoseFin, commit.slotCarry[pos], "carries");
            if (commit.solverOutput)
                require(RigExecBakedSlotDomain::PoseBase, commit.slotBaseCarry[pos], "carries");
        }
        for (size_t k = 0; k < commit.propagate.size(); ++k) {
            const auto carry = [&](RigExecBakedSlotDomain domain, uint32_t value,
                                   const auto &candidateWrites) {
                bool earlierCandidate = false;
                for (size_t pos = 0; pos < commit.slots.size() && pos < candidateWrites.size(); ++pos)
                    if (commit.slots[pos] == commit.propagate[k].first &&
                        candidateWrites[pos] == value) earlierCandidate = true;
                if (!earlierCandidate) require(domain, value, "carries");
            };
            carry(RigExecBakedSlotDomain::PoseFin, commit.descendantCarry[k], commit.slotWrites);
            if (commit.solverOutput)
                carry(RigExecBakedSlotDomain::PoseBase, commit.descendantBaseCarry[k], commit.slotBaseWrites);
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
    if (!built.program) return;
    const auto &B = built.program->GetStepGraph();
    std::string error;
    CHECK(RigExecValidateOpClusters(B.opGraph, &error));
    std::vector<double> costs;
    for (const auto &op : B.opGraph.ops) costs.push_back(B.steps[op.originalIndex].cost);
    for (double grain : {0.0, B.clustering.grainUs, 200.0}) {
        auto graph = B.opGraph;
        CHECK(RigExecLowerOpClusters(&graph, costs, grain, &error));
        CHECK(RigExecValidateOpClusters(graph, &error));
        CHECK(graph.opClusters.size() == graph.ops.size());
        std::vector<size_t> seen(graph.ops.size());
        for (size_t c = 0; c < graph.clusters.size(); ++c) {
            const auto &cluster = graph.clusters[c];
            CHECK(!cluster.members.empty() && cluster.members.size() <= 64);
            if (grain == 0) CHECK(cluster.members.size() == 1);
            for (uint32_t member : cluster.members) {
                CHECK(member < seen.size());
                if (member >= seen.size()) continue;
                ++seen[member];
                CHECK(graph.opClusters[member] == c);
            }
            for (size_t i = 1; i < cluster.members.size(); ++i) {
                const auto before = cluster.members[i - 1], after = cluster.members[i];
                CHECK(graph.ops[before].successors == std::vector<uint32_t>{after});
                CHECK(graph.ops[after].predecessors == std::vector<uint32_t>{before});
            }
            for (uint32_t predecessor : cluster.predecessors) CHECK(predecessor < c);
        }
        for (size_t count : seen) CHECK(count == 1);
        std::printf("  %s grain %g: %zu common cluster(s)\n",
                    name, grain, graph.clusters.size());
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
    const bool asked = TfGetenv("RIGEXEC_BAKED_SCHEDULE", "parallel") ==
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
    CHECK(RigExecApplySkinKernel(packet, &whole, RigExecSimdEnabled()));

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
                               std::min(points, begin + chunk), &ranged,
                               RigExecSimdEnabled());
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
    CHECK(RigExecApplySkinKernel(packet, &whole, RigExecSimdEnabled()));

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
                                               &ranged, RigExecSimdEnabled());
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

/// Chunk selection depends on distinct declared typed producer read sets.
/// Grain and vertex caps affect grouping without introducing scheduling bands.
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
            auto sets = revision.partitionProducerSets;
            for (const auto &reads : sets) {
                CHECK(std::is_sorted(reads.begin(), reads.end()));
                CHECK(std::adjacent_find(reads.begin(), reads.end()) == reads.end());
            }
            std::sort(sets.begin(), sets.end());
            sets.erase(std::unique(sets.begin(), sets.end()), sets.end());
            CHECK(revision.partitionDistinctReads == sets.size());
            size_t minCount = 0, maxCount = 0;
            if (!sets.empty()) {
                minCount = sets.front().size();
                for (const auto &reads : sets) {
                    minCount = std::min(minCount, reads.size());
                    maxCount = std::max(maxCount, reads.size());
                }
            }
            CHECK(revision.partitionProducerMin == minCount);
            CHECK(revision.partitionProducerMax == maxCount);
            const bool expected = revision.partitionCandidates > 1 &&
                                  (always || sets.size() > 1);
            CHECK(revision.chunked == expected);
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
    CHECK(repeated.executedOpCount == 0);
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

/// A rebuilt program's retained skin ops compare the outgoing program's key
/// over leaf CONTENTS. Path-leaf content versions belong to one program: an
/// outgoing program that saw a skin input move and come back holds another
/// version for the same bytes than its fresh replacement does, and that must
/// cost the replacement nothing the plain outgoing program does not.
/// Returns how many ops the replacement retained.
size_t
TestARetainedSkinOpOutlivesItsLeafVersions(const std::string &stagePath,
                                           const char *name)
{
    size_t retained[2] = {0, 0}, executed[2] = {0, 0}, moved = 0;
    for (int detour = 0; detour < 2; ++detour) {
        const UsdStageRefPtr stage = UsdStage::Open(stagePath);
        CHECK(stage != nullptr);
        if (!stage) {
            return 0;
        }
        // Every skin mover's weight moves at frame 2 and is back at frame
        // 1, authored in this stage's own session layer.
        stage->SetEditTarget(UsdEditTarget(stage->GetSessionLayer()));
        for (const UsdPrim &prim : stage->Traverse()) {
            if (prim.GetTypeName() != TfToken("RigExecSkinMover")) {
                continue;
            }
            const TfToken weightName("inputs:defaultWeight");
            UsdAttribute weight = prim.GetAttribute(weightName);
            float authored = 1.0f;
            if (weight) {
                weight.Get(&authored);
            } else {
                weight = prim.CreateAttribute(weightName,
                                              SdfValueTypeNames->Float);
            }
            weight.Set(authored, UsdTimeCode(1.0));
            weight.Set(authored * 0.5f, UsdTimeCode(2.0));
        }
        const BuiltProgram built = BuildStage(stage);
        CHECK(built.program != nullptr);
        if (!built.program) {
            return 0;
        }
        RigExecRigPose pose;
        CHECK(built.program->Run(UsdTimeCode(1.0), &pose));
        if (detour) {
            CHECK(built.program->Run(UsdTimeCode(2.0), &pose));
            CHECK(built.program->Run(UsdTimeCode(1.0), &pose));
        }
        // Each skin weight leaf's version where the outgoing program left it.
        std::map<SdfPath, uint64_t> outgoing;
        const auto weightVersions = [](const RigExecBakedProgramImpl &B,
                                       std::map<SdfPath, uint64_t> *out) {
            for (const auto &chain : B.chains) {
                for (const auto &revision : chain.revisions) {
                    const int k = revision.leaves.decl.Role(
                        RigExecRevisionLeafRole::DefaultWeight);
                    if (revision.op == RigExecRevisionOp::Skin && k >= 0 &&
                        size_t(k) < revision.leaves.versions.size()) {
                        (*out)[revision.moverPath] =
                            revision.leaves.versions[size_t(k)];
                    }
                }
            }
        };
        weightVersions(built.program->GetStepGraph(), &outgoing);
        std::vector<std::string> reasons;
        std::unique_ptr<RigExecBakedProgram> rebuilt =
            RigExecBakedProgram::Build(built.evaluator.get(), &reasons);
        CHECK(rebuilt != nullptr);
        if (!rebuilt) {
            return 0;
        }
        rebuilt->AdoptGeometryStateFrom(*built.program);
        const auto &first = rebuilt->GetStepGraph().opAdapter.retainedFirst;
        retained[detour] = size_t(std::count(first.begin(), first.end(), char(1)));
        RigExecRigPose after;
        CHECK(rebuilt->Run(UsdTimeCode(1.0), &after));
        executed[detour] = after.executedOpCount;
        std::map<SdfPath, uint64_t> replacement;
        weightVersions(rebuilt->GetStepGraph(), &replacement);
        if (detour) {
            for (const auto &[mover, version] : replacement) {
                const auto found = outgoing.find(mover);
                moved += found != outgoing.end() && found->second != version;
            }
        }
    }
    CHECK(retained[1] == retained[0]);
    CHECK(executed[1] == executed[0]);
    // The detour must leave versions apart, or the case shows nothing.
    CHECK(retained[0] == 0 || moved > 0);
    std::printf("  %s: %zu op(s) retained, %zu executed after either history; "
                "%zu skin weight version(s) differ across the rebuild\n",
                name, retained[0], executed[0], moved);
    return retained[0];
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
    evaluator.cpuReference = true;
    const RigExecRigPose warm = evaluator.Evaluate(UsdTimeCode::Default());
    CHECK(warm.valid);
    CHECK(warm.comparisonMismatches == 0);

    // A recompile of the same stage: a new epoch, the same shape, and an
    // outgoing program for the replacement to adopt.
    CHECK(evaluator.Compile(&errors));
    const RigExecRigPose after = evaluator.Evaluate(UsdTimeCode::Default());
    CHECK(after.valid);
    if (after.comparisonMismatches != 0) {
        ++failures;
        std::printf("FAIL %s: %zu mismatch(es) after a recompile\n", name,
                    after.comparisonMismatches);
        for (const std::string &diagnostic : after.diagnostics) {
            std::printf("    %s\n", diagnostic.c_str());
        }
        return;
    }
    std::printf("  %s: a recompiled rig agrees, %zu revision(s) executed\n",
                name, after.executedOpCount);
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
    evaluator.cpuReference = true;

    const RigExecRigPose clean = evaluator.Evaluate(UsdTimeCode::Default());
    CHECK(clean.valid);
    CHECK(clean.comparisonMismatches == 0);

    RigExecValueOverride override;
    override.prim = moverPath;
    override.attribute = attribute;
    override.value = value;
    evaluator.SetInteractiveOverrides({override});
    const RigExecRigPose rejected = evaluator.Evaluate(UsdTimeCode::Default());
    evaluator.ClearInteractiveOverrides();

    CHECK(rejected.valid);
    if (rejected.comparisonMismatches != 0) {
        ++failures;
        std::printf("FAIL %s: %zu baked/dynamic mismatch(es)\n", what,
                    rejected.comparisonMismatches);
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
    evaluator.cpuReference = true;
    const RigExecRigPose linear = evaluator.Evaluate(UsdTimeCode::Default());
    CHECK(linear.valid);
    CHECK(linear.comparisonMismatches == 0);

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
    if (dual.comparisonMismatches != 0) {
        ++failures;
        std::printf("FAIL dual-quaternion chunks: %zu baked/dynamic "
                    "mismatch(es)\n", dual.comparisonMismatches);
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
/// trusted only while they are provably current. A repainted layout drives
/// it from a stage (TestARecutLayoutCannotReadAnUndeclaredJoint); here the
/// biped's partition is made to disagree with the packet by hand, and the
/// question is the one the fallback exists to answer: are the points the
/// same ones?
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
    built.program->RequestFullRun();
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

/// A skin with two Build keys, {j0} and {j1}: four points of one influence
/// each, cut two to a chunk under RIGEXEC_BAKED_CHUNK_VERTS=2. J0 moves its
/// points along x and J1 along y.
UsdStageRefPtr
MakeTwoKeySkinStage()
{
    const UsdStageRefPtr stage = UsdStage::CreateInMemory();
    stage->DefinePrim(SdfPath("/Asset"), TfToken("Scope"));
    stage->DefinePrim(SdfPath("/Asset/Rig"), TfToken("RigExecRoot"));
    const UsdPrim j0 = stage->DefinePrim(SdfPath("/Asset/Rig/J0"),
                                         TfToken("RigExecControl"));
    j0.GetAttribute(TfToken("avars:tx")).Set(10.0);
    const UsdPrim j1 = stage->DefinePrim(SdfPath("/Asset/Rig/J1"),
                                         TfToken("RigExecControl"));
    j1.GetAttribute(TfToken("avars:ty")).Set(20.0);
    const SdfPath target("/Asset/Geom/Mesh.points");
    const UsdPrim mesh =
        stage->DefinePrim(target.GetPrimPath(), TfToken("Mesh"));
    VtVec3fArray points(4);
    for (size_t i = 0; i < points.size(); ++i) {
        points[i] = GfVec3f(float(i), float(i) * 2.0f, float(i) * 3.0f);
    }
    mesh.GetAttribute(TfToken("points")).Set(points);
    stage->DefinePrim(SdfPath("/Asset/Rig/Movers"), TfToken("Scope"));
    const UsdPrim skin = stage->DefinePrim(
        SdfPath("/Asset/Rig/Movers/Skin"), TfToken("RigExecSkinMover"));
    skin.ApplyAPI(TfToken("RigExecMoverAPI"));
    skin.GetRelationship(TfToken("rigExec:moves")).SetTargets({target});
    skin.GetAttribute(TfToken("inputs:defaultWeight")).Set(1.0f);
    skin.CreateRelationship(TfToken("rigExec:influences"))
        .SetTargets({j0.GetPath(), j1.GetPath()});
    skin.CreateAttribute(TfToken("rigExec:elementSize"),
                         SdfValueTypeNames->Int).Set(1);
    skin.CreateAttribute(TfToken("rigExec:jointIndices"),
                         SdfValueTypeNames->IntArray)
        .Set(VtIntArray{0, 0, 1, 1});
    skin.CreateAttribute(TfToken("rigExec:jointWeights"),
                         SdfValueTypeNames->FloatArray)
        .Set(VtFloatArray{1.0f, 1.0f, 1.0f, 1.0f});
    return stage;
}

/// A repainted layout on a chunked revision runs the revision whole, so no
/// chunk reads a joint its Build key never declared (B21).
///
/// The chunk steps declare their reads from the Build keys, and a frame may
/// not change the program. The indices are topology, so only a drag on them
/// repaints a standing program. Repainting chunk 0's points onto j1 and
/// then moving only j1 is the frame the cone answers with chunk 1 and the
/// fuse; a chunk 0 that had been re-cut onto j1 would keep its range from
/// the frame before. Instead the keys stay Build's, the partition is stale
/// and the fuse skins every point, equal to the exec reference to the bit.
/// Lifting the drag on the indices adopts a handle with Build's arrays, and
/// the chunks run again. A frozen job at each stage agrees with the
/// reference too. An authored repaint rebuilds the program, whose keys are
/// cut from the indices it reads.
void
TestARecutLayoutCannotReadAnUndeclaredJoint()
{
    const int failuresBefore = failures;
    const std::string always = TfGetenv("RIGEXEC_BAKED_CHUNK_ALWAYS");
    const std::string verts = TfGetenv("RIGEXEC_BAKED_CHUNK_VERTS");
    TfSetenv("RIGEXEC_BAKED_CHUNK_ALWAYS", "1");
    TfSetenv("RIGEXEC_BAKED_CHUNK_VERTS", "2");
    const auto restore = [&]() {
        if (always.empty()) {
            TfUnsetenv("RIGEXEC_BAKED_CHUNK_ALWAYS");
        } else {
            TfSetenv("RIGEXEC_BAKED_CHUNK_ALWAYS", always);
        }
        if (verts.empty()) {
            TfUnsetenv("RIGEXEC_BAKED_CHUNK_VERTS");
        } else {
            TfSetenv("RIGEXEC_BAKED_CHUNK_VERTS", verts);
        }
    };

    const UsdStageRefPtr stage = MakeTwoKeySkinStage();
    const SdfPath rigPath("/Asset/Rig");
    const SdfPath skinPath("/Asset/Rig/Movers/Skin");
    const SdfPath target("/Asset/Geom/Mesh.points");
    const UsdAttribute indices = stage->GetPrimAtPath(skinPath)
        .GetAttribute(TfToken("rigExec:jointIndices"));
    RigExecRigEvaluator evaluator(stage, rigPath);
    std::vector<std::string> errors;
    CHECK(evaluator.Compile(&errors));
    CHECK(evaluator.IsBakeable());
    const UsdTimeCode time(1.0);

    const auto skin = [&]() -> const RigExecBakedProgramImpl::GeomRevision * {
        const RigExecBakedProgram *program = evaluator.GetBakedProgram();
        if (!program) {
            return nullptr;
        }
        for (const auto &chain : program->GetStepGraph().chains) {
            for (const auto &revision : chain.revisions) {
                if (revision.moverPath == skinPath) {
                    return &revision;
                }
            }
        }
        return nullptr;
    };
    // The exec reference on the stage as it stands, under the same
    // overrides, point for point; a differing point names its chunk.
    const auto agrees =
        [&](const char *what, const RigExecRigPose &pose,
            const std::vector<RigExecValueOverride> &overrides) {
        RigExecRigEvaluator reference(stage, rigPath);
        std::vector<std::string> referenceErrors;
        CHECK(reference.Compile(&referenceErrors));
        reference.cpuReference = true;
        reference.SetInteractiveOverrides(overrides);
        const RigExecRigPose expected = reference.Evaluate(time);
        CHECK(pose.valid && expected.valid);
        rigExecTest::CompareEveryMap(&failures,
                                     std::string("recut layout, ") + what,
                                     expected, pose);
        const auto found = pose.movedProperties.find(target);
        const auto want = expected.movedProperties.find(target);
        if (found == pose.movedProperties.end() ||
            want == expected.movedProperties.end() ||
            !found->second.IsHolding<VtVec3fArray>() ||
            !want->second.IsHolding<VtVec3fArray>()) {
            return;
        }
        const VtVec3fArray &got = found->second.UncheckedGet<VtVec3fArray>();
        const VtVec3fArray &ref = want->second.UncheckedGet<VtVec3fArray>();
        for (size_t i = 0; i < got.size() && i < ref.size(); ++i) {
            if (got[i] != ref[i]) {
                std::printf("    %s: point %zu (chunk %zu) is (%g %g %g), "
                            "the reference (%g %g %g)\n", what, i, i / 2,
                            got[i][0], got[i][1], got[i][2], ref[i][0],
                            ref[i][1], ref[i][2]);
            }
        }
    };
    const std::vector<RigExecValueOverride> none;
    const auto dragJ1 = [](double value) {
        return std::vector<RigExecValueOverride>{RigExecValueOverride{
            SdfPath("/Asset/Rig/J1"), TfToken(), TfToken("avars:ty"),
            VtValue(value)}};
    };
    // A frozen job at the held frame under its own drag: the worker adopts
    // with the live rule from the clone's partition, so its chunks stand
    // down and rejoin where live's do.
    const auto frozenAgrees =
        [&](const char *what,
            const std::vector<RigExecValueOverride> &overrides) {
        std::shared_ptr<const RigExecFrozenProgram> frozen;
        std::string error;
        CHECK(RigExecFreezeProgram(evaluator, &frozen, &error));
        RigExecFrameInputs inputs;
        CHECK(RigExecSampleFrameInputs(evaluator, time, overrides, &inputs,
                                       &error));
        if (!frozen) {
            std::printf("FAIL %s: the freeze refused: %s\n", what,
                        error.c_str());
            return;
        }
        RigExecFrozenEvalContext context;
        context.epochDigest = evaluator.GetBindingEpochDigest();
        context.slotCount = evaluator.GetBakedProgram()->GetProviderCount();
        context.varyingInputCount = inputs.values.size();
        context.flags = 0;
        if (evaluator.GetPublishWeightFields()) {
            context.flags |= kRigExecFrozenPublishWeightFields;
        }
        if (evaluator.GetSolverGuidesEnabled()) {
            context.flags |= kRigExecFrozenSolverGuidesEnabled;
        }
        context.frozen = frozen.get();
        const RigExecRigPose warmed = RigExecEvaluateFrozen(
            context, inputs, RigExecMakeProductionStepRunner());
        if (!warmed.valid) {
            std::printf("FAIL %s: the job was declined\n", what);
        }
        agrees(what, warmed, overrides);
    };

    // Build: keys {j0} and {j1}, adopted on the first run.
    RigExecRigPose pose = evaluator.Evaluate(time);
    agrees("authored layout", pose, none);
    const RigExecBakedProgramImpl::GeomRevision *revision = skin();
    CHECK(revision != nullptr);
    if (!revision) {
        restore();
        return;
    }
    const std::vector<int> key0{0}, key1{1};
    CHECK(revision->chunked && revision->chunks.size() == 2);
    if (!revision->chunked || revision->chunks.size() != 2) {
        restore();
        return;
    }
    CHECK(revision->chunks[0].key == key0 && revision->chunks[1].key == key1);
    CHECK(!revision->partitionStale);
    CHECK(revision->topology &&
          revision->partitionTopology == revision->topology);
    const size_t builds = evaluator.GetBakedProgramBuildCount();

    // Chunk 0's points repainted onto j1 by a drag on the indices: an
    // override stands on topology, and nothing rebuilds.
    const RigExecValueOverride onJ1{skinPath.GetPrimPath(), TfToken(),
                                    TfToken("rigExec:jointIndices"),
                                    VtValue(VtIntArray{1, 1, 1, 1})};
    const std::vector<RigExecValueOverride> paint = {onJ1};
    evaluator.SetInteractiveOverrides(paint);
    pose = evaluator.Evaluate(time);
    agrees("repainted", pose, paint);
    revision = skin();
    CHECK(revision != nullptr);
    CHECK(evaluator.GetBakedProgramBuildCount() == builds);

    // Only j1 moves.
    const auto paintedDrag = [&](double value) {
        std::vector<RigExecValueOverride> overrides = dragJ1(value);
        overrides.push_back(onJ1);
        return overrides;
    };
    const std::vector<RigExecValueOverride> drag = paintedDrag(25.0);
    evaluator.SetInteractiveOverrides(drag);
    pose = evaluator.Evaluate(time);
    agrees("repainted, j1 moved", pose, drag);
    revision = skin();
    CHECK(revision != nullptr);
    if (!revision) {
        restore();
        return;
    }
    // The keys are still Build's, the partition does not describe the new
    // layout, and the fuse did the work.
    CHECK(revision->chunks[0].key == key0 && revision->chunks[1].key == key1);
    CHECK(revision->partitionStale);
    CHECK(revision->topology &&
          revision->partitionTopology != revision->topology);
    for (const RigExecBakedProgramImpl::GeomChunk &chunk : revision->chunks) {
        CHECK(!chunk.ok);
    }
    CHECK(evaluator.GetBakedGenerationCount() == 3);
    frozenAgrees("frozen, repainted, j1 moved again", paintedDrag(30.0));

    // The drag on the indices lifted: the Build indices read again, so the
    // new handle has the partition's arrays, is adopted, and the chunks run
    // again.
    const std::vector<RigExecValueOverride> unpainted = dragJ1(25.0);
    evaluator.SetInteractiveOverrides(unpainted);
    pose = evaluator.Evaluate(time);
    agrees("painted back", pose, unpainted);
    revision = skin();
    CHECK(revision != nullptr);
    if (!revision) {
        restore();
        return;
    }
    CHECK(!revision->partitionStale);
    CHECK(revision->topology &&
          revision->partitionTopology == revision->topology);
    for (const RigExecBakedProgramImpl::GeomChunk &chunk : revision->chunks) {
        CHECK(chunk.ok);
    }
    frozenAgrees("frozen, painted back, j1 moved again", dragJ1(30.0));
    evaluator.ClearInteractiveOverrides();
    pose = evaluator.Evaluate(time);
    agrees("painted back, drag lifted", pose, none);
    CHECK(evaluator.GetBakedProgramBuildCount() == builds);
    CHECK(evaluator.GetBakedGenerationCount() == 5);

    // An authored repaint is topology: the program rebuilds and cuts its
    // keys from the indices it reads, so no partition stands stale.
    indices.Set(VtIntArray{1, 1, 1, 1});
    CHECK(evaluator.GetLastNoticeDisposition() ==
          RigExecNoticeDisposition::Stale);
    pose = evaluator.Evaluate(time);
    agrees("authored repaint", pose, none);
    CHECK(evaluator.GetBakedProgramBuildCount() == builds + 1);
    revision = skin();
    CHECK(revision != nullptr && !revision->partitionStale);
    if (failures == failuresBefore) {
        std::printf("  recut layout: %s ran whole while repainted, and its "
                    "chunks resumed\n", skinPath.GetText());
    }
    restore();
}

/// A whole-revision fuse leaves staging as the chunks' keys describe it.
///
/// Under a stale partition a chunk whose joint moved still runs, stands
/// down, and publishes its RevisionOut key over its range of staging; the
/// fuse then skins every point. Had the fuse written staging, those keys
/// would no longer describe the state: a snapshot would refuse to retain
/// it, and its first job would run every op cold where live runs the drag's
/// cone.
void
TestAStaleFuseKeepsTheChunkKeys()
{
    const int failuresBefore = failures;
    const std::string always = TfGetenv("RIGEXEC_BAKED_CHUNK_ALWAYS");
    const std::string verts = TfGetenv("RIGEXEC_BAKED_CHUNK_VERTS");
    TfSetenv("RIGEXEC_BAKED_CHUNK_ALWAYS", "1");
    TfSetenv("RIGEXEC_BAKED_CHUNK_VERTS", "2");
    const auto restore = [&]() {
        if (always.empty()) {
            TfUnsetenv("RIGEXEC_BAKED_CHUNK_ALWAYS");
        } else {
            TfSetenv("RIGEXEC_BAKED_CHUNK_ALWAYS", always);
        }
        if (verts.empty()) {
            TfUnsetenv("RIGEXEC_BAKED_CHUNK_VERTS");
        } else {
            TfSetenv("RIGEXEC_BAKED_CHUNK_VERTS", verts);
        }
    };

    const UsdStageRefPtr stage = MakeTwoKeySkinStage();
    const SdfPath rigPath("/Asset/Rig");
    const SdfPath skinPath("/Asset/Rig/Movers/Skin");
    RigExecRigEvaluator evaluator(stage, rigPath);
    std::vector<std::string> errors;
    CHECK(evaluator.Compile(&errors));
    CHECK(evaluator.IsBakeable());
    const UsdTimeCode time(1.0);
    CHECK(evaluator.Evaluate(time).valid);

    // Chunk 0's points repainted onto j1 by a drag on the indices, then j1
    // moved: chunk 1, keyed {j1}, runs and stands down; the fuse skins.
    const RigExecValueOverride onJ1{skinPath, TfToken(),
                                    TfToken("rigExec:jointIndices"),
                                    VtValue(VtIntArray{1, 1, 1, 1})};
    const auto painted = [&](double ty) {
        return std::vector<RigExecValueOverride>{
            onJ1, RigExecValueOverride{SdfPath("/Asset/Rig/J1"), TfToken(),
                                       TfToken("avars:ty"), VtValue(ty)}};
    };
    evaluator.SetInteractiveOverrides({onJ1});
    CHECK(evaluator.Evaluate(time).valid);
    evaluator.SetInteractiveOverrides(painted(25.0));
    CHECK(evaluator.Evaluate(time).valid);
    const RigExecBakedProgram *program = evaluator.GetBakedProgram();
    CHECK(program != nullptr);
    if (!program) {
        restore();
        return;
    }
    const RigExecBakedProgramImpl &B = program->GetStepGraph();
    const RigExecBakedProgramImpl::GeomRevision *revision = nullptr;
    for (const auto &chain : B.chains) {
        for (const auto &candidate : chain.revisions) {
            if (candidate.moverPath == skinPath) {
                revision = &candidate;
            }
        }
    }
    CHECK(revision != nullptr && revision->chunked &&
          revision->partitionStale && revision->executed);

    // Every published RevisionOut key still describes the state.
    using D = RigExecBakedSlotDomain;
    size_t outs = 0, standing = 0;
    for (const auto &value : B.opAdapter.values) {
        if (!value.initialized || D(value.domain) != D::RevisionOut) {
            continue;
        }
        ++outs;
        if (RigExecBakedOpValueKeyStands(B, D::RevisionOut, value.slot,
                                         value.key)) {
            ++standing;
        }
    }
    CHECK(outs > 0 && standing == outs);

    // So a snapshot retains the completed state ...
    std::shared_ptr<const RigExecFrozenProgram> frozen;
    std::string error;
    CHECK(RigExecFreezeProgram(evaluator, &frozen, &error));
    if (!frozen) {
        std::printf("FAIL stale fuse: the freeze refused: %s\n",
                    error.c_str());
        restore();
        return;
    }
    CHECK(B.opAdapter.everRan && frozen->program.opAdapter.everRan);
    CHECK(frozen->program.opAdapter.retainedFirst ==
          B.opAdapter.retainedFirst);

    // ... and its first job runs what live runs for the next drag.
    const std::vector<RigExecValueOverride> next = painted(30.0);
    RigExecFrameInputs inputs;
    CHECK(RigExecSampleFrameInputs(evaluator, time, next, &inputs, &error));
    RigExecFrozenEvalContext context;
    context.epochDigest = evaluator.GetBindingEpochDigest();
    context.slotCount = program->GetProviderCount();
    context.varyingInputCount = inputs.values.size();
    context.flags = 0;
    if (evaluator.GetPublishWeightFields()) {
        context.flags |= kRigExecFrozenPublishWeightFields;
    }
    if (evaluator.GetSolverGuidesEnabled()) {
        context.flags |= kRigExecFrozenSolverGuidesEnabled;
    }
    context.frozen = frozen.get();
    const RigExecRigPose job = RigExecEvaluateFrozen(
        context, inputs, RigExecMakeProductionStepRunner());
    evaluator.SetInteractiveOverrides(next);
    const RigExecRigPose live = evaluator.Evaluate(time);
    CHECK(job.valid && live.valid);
    rigExecTest::CompareEveryMap(&failures, "stale fuse, first frozen job",
                                 live, job);
    if (job.executedOpCount != live.executedOpCount) {
        std::printf("FAIL stale fuse: the first frozen job ran %zu op(s), "
                    "live %zu\n", size_t(job.executedOpCount),
                    size_t(live.executedOpCount));
        ++failures;
    }
    CHECK(job.executedOpCount < B.opGraph.ops.size());
    if (failures == failuresBefore) {
        std::printf("  stale fuse: %zu RevisionOut key(s) stand, and the "
                    "first frozen job ran %zu of %zu op(s), as live did\n",
                    outs, size_t(job.executedOpCount), B.opGraph.ops.size());
    }
    restore();
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

/// Exact forward reachability: on a finite DAG, each cone equals its own
/// cluster plus the union of its direct successors' cones. This equation
/// detects both missing and extraneous members without all-pairs scans.
size_t
ConeRecurrenceViolations(const RigExecBakedClustering &graph,
                         const std::vector<RigExecBakedClusterSet> &cones,
                         size_t *edges,size_t *wordChecks)
{
    const size_t count=graph.clusters.size(),words=(count+63)/64;
    size_t broken=0;
    std::vector<uint64_t> expected(words,0);
    for(size_t c=0;c<count;++c) {
        std::fill(expected.begin(),expected.end(),uint64_t(0));
        expected[c/64]|=uint64_t(1)<<(c%64);
        for(int next:graph.clusters[c].succs) {
            ++*edges;
            for(size_t w=0;w<words;++w) {
                expected[w]|=cones[size_t(next)].words[w]; ++*wordChecks;
            }
        }
        for(size_t w=0;w<words;++w) {
            ++*wordChecks;
            if(expected[w]!=cones[c].words[w])++broken;
        }
    }
    return broken;
}

void
TestTheConeClosuresAreSound(const BuiltProgram &built, const char *name)
{
    if (!built.program) {
        ++failures;
        std::printf("FAIL %s: no program to check cones of\n", name);
        return;
    }
    const auto &B=built.program->GetStepGraph();
    const size_t clusters=B.clustering.clusters.size();
    const size_t words=(clusters+63)/64;
    CHECK(B.cones.cone.size()==clusters);
    if(B.cones.cone.size()!=clusters)return;
    for(const auto &cluster:B.clustering.clusters) {
        for(int next:cluster.succs) {
            CHECK(next>=0 && size_t(next)<clusters);
            if(next<0 || size_t(next)>=clusters)return;
        }
        for(int previous:cluster.preds) {
            CHECK(previous>=0 && size_t(previous)<clusters);
            if(previous<0 || size_t(previous)>=clusters)return;
        }
    }
    const auto order=RigExecBakedClusterTopologicalOrder(B.clustering);
    CHECK(order.size()==clusters);
    if(order.size()!=clusters)return;
    std::vector<int> rank(clusters,-1);
    for(size_t i=0;i<order.size();++i) {
        const int c=order[i];
        CHECK(c>=0 && size_t(c)<clusters);
        if(c<0 || size_t(c)>=clusters)return;
        CHECK(rank[size_t(c)]<0);
        if(rank[size_t(c)]>=0)return;
        rank[size_t(c)]=int(i);
    }
    for(size_t c=0;c<clusters;++c) {
        CHECK(B.cones.cone[c].words.size()==words);
        if(B.cones.cone[c].words.size()!=words)return;
        for(int next:B.clustering.clusters[c].succs) {
            CHECK(next>=0 && size_t(next)<clusters);
            if(next<0 || size_t(next)>=clusters)return;
            CHECK(rank[c]<rank[size_t(next)]);
            if(rank[c]>=rank[size_t(next)])return;
        }
        if(clusters%64) {
            const uint64_t mask=(uint64_t(1)<<(clusters%64))-1;
            CHECK((B.cones.cone[c].words.back()&~mask)==0);
        }
    }
    size_t edges=0,wordChecks=0;
    const size_t broken=ConeRecurrenceViolations(B.clustering,B.cones.cone,&edges,&wordChecks);
    if(broken) {
        ++failures;
        std::printf("FAIL %s: %zu cone recurrence word violation(s)\n",name,broken);
    }
    size_t reach=0;
    for(const auto &cone:B.cones.cone)reach+=cone.Count();
    std::printf("  %s: %zu cluster(s), %.1f cluster(s) in the average cone; %zu edges, %zu word checks\n",
        name,clusters,clusters?double(reach)/double(clusters):0.0,edges,wordChecks);
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
OpenRig(const std::string &stagePath, bool referenceChecks = false)
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
    rig.evaluator->cpuReference = referenceChecks;
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
    const LiveRig rig = OpenRig(stagePath);
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
    CHECK(second.executedOpCount == 0);
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
    const LiveRig rig = OpenRig(stagePath);
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
    CHECK(moved.executedOpCount > 0);

    // Back to where it started, and HELD there. The first generation after
    // the value moves back executes -- it moved -- and the one after it must
    // not, although the override is still standing.
    E.SetInteractiveOverrides(
        {RigExecValueOverride{control, TfToken(), avar, original}});
    const RigExecRigPose back = E.Evaluate(UsdTimeCode(1));
    CHECK(back.executedOpCount > 0);
    const RigExecRigPose held = E.Evaluate(UsdTimeCode(1));
    CHECK(held.executedOpCount == 0);
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
    // Traverse the execution artifact; result bits retain physical step IDs.
    std::vector<uint32_t> pending;
    std::vector<char> seen(B.opGraph.ops.size(), 0);
    for (uint32_t c = 0; c < B.opGraph.ops.size(); ++c) {
        const uint32_t physical = B.opGraph.ops[c].originalIndex;
        if (physical < steps && seeds.Test(int(physical))) {
            seen[c] = 1; pending.push_back(c);
        }
    }
    for (size_t at = 0; at < pending.size(); ++at) {
        const auto &op = B.opGraph.ops[pending[at]];
        closed.Set(int(op.originalIndex));
        for (uint32_t next : op.successors) if (!seen[next]) {
            seen[next] = 1; pending.push_back(next);
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
    const LiveRig baked = OpenRig(stagePath);
    // The path that never skips, and never baked: one evaluator, one
    // generation, the whole exec walk.
    const LiveRig reference =
        OpenRig(stagePath, true);
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

    const LiveRig baked = OpenRig(stagePath);
    const LiveRig reference =
        OpenRig(stagePath, true);
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
    CHECK(moved.executedOpCount > 0);

    // And per STEP, which is the grain the run closed at: the steps it ran
    // are the drag's step cone and no more, and a clean step packed into a
    // cluster the run dispatched was skipped rather than run beside its
    // dirty cluster-mate. The count survives the cone verifier's forced
    // second pass (RigExecBakedRunStatistics puts it back); the sets do not,
    // so they are read only when that pass did not run.
    std::vector<int> sourceSeeds{B.cones.avarStep[size_t(slot->second)]};
    const SdfPath source = control.AppendProperty(avar);
    for (size_t k=0;k<B.providerProgram.sampled.size();++k) {
        if (B.providerProgram.sampled[k].attribute != source) continue;
        for (size_t s=0;s<B.steps.size();++s)
            if (B.steps[s].kind == RigExecBakedStepKind::SpaceExpression &&
                B.steps[s].part == 2 && B.steps[s].object == int(k))
                sourceSeeds.push_back(int(s));
    }
    const RigExecBakedClusterSet stepBound = StepConeBound(B,sourceSeeds);
    const size_t stepsRan = B.opExecution.executed;
    CHECK(stepsRan > 0);
    if (stepsRan > stepBound.Count()) {
        ++failures;
        std::printf("FAIL leaf-drag: closed %zu step(s), and the drag's "
                    "step cone is %zu\n", stepsRan, stepBound.Count());
    }
    if (!RigExecBakedVerifyConesRequested()) {
        size_t packed = 0;
        for (size_t s = 0; s < B.steps.size(); ++s) {
            const bool stepClosed = B.opExecution.ran[s] != 0;
            if (stepClosed && !stepBound.Test(int(s)))
                std::printf("FAIL leaf-drag: selected step %zu (%s), kind %u part %d, outside exact input cone\n",
                            s,B.steps[s].label.c_str(),unsigned(B.steps[s].kind),B.steps[s].part);
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
                holds = holds || B.opExecution.ran[size_t(member)] != 0;
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
    const LiveRig baked = OpenRig(stagePath);
    const LiveRig reference =
        OpenRig(stagePath, true);
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
    CHECK(held.comparisonMismatches == 0);
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
bool
ScheduleByHand(RigExecBakedProgramImpl *B,std::string *compileError=nullptr)
{
    for (size_t i=0;i<B->steps.size();++i) {
        auto &step=B->steps[i];
        if (step.label.empty()) {
            std::string subject="every volume weight";
            if (step.kind==RigExecBakedStepKind::VolumePlacements && step.part!=0 &&
                step.object>=0 && size_t(step.object)<B->paths.size())
                subject=B->paths[size_t(step.object)].GetString();
            else if ((step.kind==RigExecBakedStepKind::RevisionStatic ||
                      step.kind==RigExecBakedStepKind::RevisionFuse) &&
                     step.object>=0 && size_t(step.object)<B->revisionIndex.size()) {
                const auto [chain,revision]=B->revisionIndex[size_t(step.object)];
                subject=B->chains[size_t(chain)].revisions[size_t(revision)].moverPath.GetString();
            }
            step.label=std::string(RigExecBakedStepKindName(step.kind))+" "+subject;
        }
        step.descriptorKey=step.label+"/hand:"+std::to_string(i);
    }
    std::string error;
    if (!RigExecBakedCompileOpGraph(B,&error)) {
        if(compileError)*compileError=error;
        return false;
    }
    std::vector<double> costs(B->steps.size(),1.0);
    CHECK(RigExecLowerOpClusters(&B->opGraph,costs,0.0,&error));
    CHECK(RigExecValidateOpClusters(B->opGraph,&error));
    auto &view=B->clustering;
    view={};
    view.clusterOf.assign(B->opGraph.opClusters.begin(),B->opGraph.opClusters.end());
    view.clusters.resize(B->opGraph.clusters.size());
    for(size_t c=0;c<view.clusters.size();++c) {
        const auto &compiled=B->opGraph.clusters[c]; auto &cluster=view.clusters[c];
        cluster.members.assign(compiled.members.begin(),compiled.members.end());
        cluster.preds.assign(compiled.predecessors.begin(),compiled.predecessors.end());
        cluster.succs.assign(compiled.successors.begin(),compiled.successors.end());
        for(uint32_t member:compiled.members) {
            B->steps[member].cluster=int(c);
        }
        view.topologicalOrder.push_back(int(c));
    }
    B->closed.Resize(B->opGraph.clusters.size());
    B->closedSteps.Resize(B->opGraph.ops.size());
    return true;
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

// A hand-built head tier: two chains of two revisions, the second revision
// of chain 1 reading chain 0's target through its inputs:value.
void
HandBuiltHeadTier(RigExecBakedProgramImpl *B)
{
    for (int c = 0; c < 2; ++c) {
        RigExecBakedPropertyChain chain;
        chain.target = SdfPath("/Rig/C" + std::to_string(c) + ".rigExec:x");
        chain.targetExists = true;
        for (int r = 0; r < 2; ++r) {
            RigExecBakedPropertyChain::Revision revision;
            revision.mover = SdfPath("/Rig/M" + std::to_string(c) + "_" +
                                     std::to_string(r));
            revision.moverExists = true;
            chain.revisions.push_back(revision);
        }
        B->propertyChains.push_back(chain);
    }
    RigExecBakedWalkHop hop;
    hop.path = B->propertyChains[0].target;
    hop.chain = 0;
    RigExecBakedWalk &walk = B->propertyChains[1].revisions[1].value;
    walk.flavour = RigExecBakedWalk::Flavour::Pinned;
    walk.hops.push_back(hop);
    RigExecBakedBuildPropertySteps(B);
    std::string error;
    CHECK(ScheduleByHand(B));
}

// The head step of chain \p chain, part \p part.
uint32_t
HeadStepOf(const RigExecBakedProgramImpl &B, int chain, int part)
{
    for (uint32_t i : RigExecBakedHeadIndices(B)) {
        if (B.steps[i].object == chain && B.steps[i].part == part) {
            return i;
        }
    }
    return 0;
}

void
ExpectHeadRejected(const RigExecBakedProgramImpl &B, const char *what,
                   const std::string &expected)
{
    std::string error;
    if (RigExecBakedValidateHeadTier(B, &error)) {
        ++failures;
        std::printf("FAIL %s: the head validator accepted it\n", what);
        return;
    }
    if (error.find(expected) == std::string::npos) {
        ++failures;
        std::printf("FAIL %s: \"%s\" is not in \"%s\"\n", what,
                    expected.c_str(), error.c_str());
    }
    std::printf("  %s: %s\n", what, error.c_str());
}

/// The head tier's validator: it accepts a built program's and a
/// hand-built tier, and refuses a part ordered before the part it reads, a
/// property step that reads a region domain, and a walk through another
/// chain's target that does not declare that chain's final version.
void
TestTheHeadValidatorRejectsAMalformedTier(const BuiltProgram &built)
{
    std::string error;
    CHECK(built.program &&
          RigExecBakedValidateHeadTier(built.program->GetStepGraph(),
                                       &error));
    {
        RigExecBakedProgramImpl B;
        HandBuiltHeadTier(&B);
        CHECK(RigExecBakedValidateHeadTier(B, &error));
        // The sort keeps the chain order where the reads allow it.
        std::vector<uint32_t> identity(RigExecBakedHeadIndices(B).size());
        for (uint32_t i = 0; i < identity.size(); ++i) {
            identity[i] = i;
        }
        CHECK(RigExecBakedHeadIndices(B) == identity);
    }
    {
        RigExecBakedProgramImpl B;
        HandBuiltHeadTier(&B);
        const uint32_t one = HeadStepOf(B, 0, 1);
        const uint32_t two = HeadStepOf(B, 0, 2);
        std::swap(B.steps[one],B.steps[two]);
        CHECK(ScheduleByHand(&B));
        CHECK(HeadStepOf(B,0,1)<HeadStepOf(B,0,2));
        CHECK(RigExecBakedValidateHeadTier(B,&error));
    }
    {
        RigExecBakedProgramImpl B;
        HandBuiltHeadTier(&B);
        B.steps[HeadStepOf(B, 0, 1)].writes.push_back(
            RigExecBakedOne(RigExecBakedSlotDomain::PoseFin, 0));
        ExpectHeadRejected(B, "a property step writing PoseFin",
                           "writes a non-property domain");
    }
    {
        RigExecBakedProgramImpl B;
        HandBuiltHeadTier(&B);
        RigExecBakedStep &step = B.steps[HeadStepOf(B, 1, 2)];
        const uint32_t final0 = B.propertyChains[0].versionBase + 2;
        step.reads.erase(
            std::remove_if(step.reads.begin(), step.reads.end(),
                           [final0](const RigExecBakedSlotRange &range) {
                               return range.begin <= final0 &&
                                      final0 < range.end;
                           }),
            step.reads.end());
        ExpectHeadRejected(B, "an undeclared walk through a target",
                           "walk /Rig/C0.rigExec:x meets chain target "
                           "without declaring it");
    }
    {
        RigExecBakedProgramImpl B;
        HandBuiltHeadTier(&B);
        B.steps[HeadStepOf(B,0,1)].isHead=false;
        ExpectHeadRejected(B,"a property descriptor with a wrong kind flag","inconsistent head kind");
    }
    {
        RigExecBakedProgramImpl B;
        HandBuiltHeadTier(&B);
        auto &step=B.steps[HeadStepOf(B,0,1)];
        step.reads.clear();
        ExpectHeadRejected(B,"a missing predecessor version","omits its predecessor property version");
    }
    {
        RigExecBakedProgramImpl B;
        HandBuiltHeadTier(&B);
        auto &step=B.steps[HeadStepOf(B,0,1)];
        step.writes.clear();
        ExpectHeadRejected(B,"a missing revision writer","does not write exactly its version and owned records");
    }
    {
        RigExecBakedProgramImpl B;
        HandBuiltHeadTier(&B);
        auto &step=B.steps[HeadStepOf(B,0,1)];
        step.writes.push_back(B.steps[HeadStepOf(B,0,0)].writes.front());
        ExpectHeadRejected(B,"duplicate version producer","has more than one head producer");
    }

}

static void
TestVerifierComparesExactSkinTopologyContent()
{
    RigExecBakedProgramImpl B;
    B.chains.resize(1);
    B.chains[0].target = SdfPath("/Verifier/Mesh.points");
    B.chains[0].revisions.resize(1);
    auto &revision = B.chains[0].revisions[0];
    revision.moverPath = SdfPath("/Verifier/Skin");
    auto topology = std::make_shared<RigExecSkinTopology>();
    topology->indices = {0, 1};
    topology->weights = {1.0f, -0.0f};
    topology->elementSize = 2;
    topology->pointCount = 1;
    topology->influenceCount = 2;
    topology->validated = true;
    const auto bind = [&](const std::shared_ptr<RigExecSkinTopology> &value) {
        revision.layoutHandle = value;
        revision.layoutCandidate = value;
        revision.parameters.skinTopology = value;
        revision.lastParameters.skinTopology = value;
    };
    bind(topology);
    RigExecBakedRunShadow expected;
    expected.Capture(B);
    auto copy = std::make_shared<RigExecSkinTopology>(*topology);
    CHECK(copy != topology);
    bind(copy);
    std::vector<std::string> differences;
    CHECK(expected.Compare(B, &differences) == 0);
    const auto reject = [&](const auto &edit) {
        auto changed = std::make_shared<RigExecSkinTopology>(*topology);
        edit(changed.get());
        bind(changed);
        differences.clear();
        CHECK(expected.Compare(B, &differences) > 0);
        CHECK(!differences.empty());
    };
    reject([](auto *v) { v->weights[1] = 0.0f; });
    reject([](auto *v) { v->weights.push_back(0.0f); });
    reject([](auto *v) { v->indices[1] = 0; });
    reject([](auto *v) { ++v->elementSize; });
    reject([](auto *v) { ++v->pointCount; });
    reject([](auto *v) { ++v->influenceCount; });
    reject([](auto *v) { v->validated = false; });
    bind(nullptr);
    differences.clear();
    CHECK(expected.Compare(B, &differences) > 0);

    // Invalid raw layouts still retain exact non-finite payload bits.
    topology->validated = false;
    const uint32_t firstNaN = 0x7fc00011u;
    std::memcpy(&topology->weights[0], &firstNaN, sizeof(firstNaN));
    bind(topology);
    expected.Capture(B);
    bind(std::make_shared<RigExecSkinTopology>(*topology));
    differences.clear();
    CHECK(expected.Compare(B, &differences) == 0);
    reject([](auto *v) {
        const uint32_t nextNaN = 0x7fc00012u;
        std::memcpy(&v->weights[0], &nextNaN, sizeof(nextNaN));
    });
}

void
TestFrozenSparseRawLayoutDigest()
{
    RigExecFrameInputs inputs;
    // The fixed SkinTopology prefix remains present even for non-Skin owners.
    inputs.layoutLeaves.resize(1);
    inputs.layoutSourcePaths.resize(1);
    inputs.layoutSourcePaths.push_back({SdfPath("/Shape.offsets"),
                                       SdfPath("/Shape.pointIndices")});
    inputs.layoutLeaves.push_back({VtValue(VtVec3fArray{GfVec3f(1, -0.0f, 0)}),
                                   VtValue(VtIntArray{0})});
    bool exact = false;
    const auto initial = RigExecFrozenControlDigest(inputs, &exact);
    CHECK(exact);
    const auto held = inputs;
    RigExecRetainedFrameState retained;
    retained.inputs = held;
    CHECK(RigExecFrozenControlDigest(held, &exact) == initial && exact);
    CHECK(RigExecChangedControls(retained, held, {}).empty());
    inputs.layoutLeaves[1][0] = VtValue(VtVec3fArray{GfVec3f(1, 0.0f, 0)});
    CHECK(RigExecFrozenControlDigest(inputs, &exact) != initial && exact);
    CHECK(RigExecChangedControls(retained, inputs, {}) ==
          std::vector<RigExecControlId>{RigExecControlIdForPath(SdfPath("/Shape.offsets"))});
    inputs = held;
    inputs.layoutLeaves[1][1] = VtValue(VtIntArray{1});
    CHECK(RigExecFrozenControlDigest(inputs, &exact) != initial && exact);
    CHECK(RigExecChangedControls(retained, inputs, {}) ==
          std::vector<RigExecControlId>{RigExecControlIdForPath(SdfPath("/Shape.pointIndices"))});
    inputs = held;
    inputs.layoutLeaves[1][0] = VtValue(VtVec3fArray());
    CHECK(RigExecFrozenControlDigest(inputs, &exact) != initial && exact);
    inputs = held;
    inputs.layoutSourcePaths[1].pop_back();
    RigExecFrozenControlDigest(inputs, &exact);
    CHECK(!exact);
}

void
TestVerifierRestoresAdoptedLayoutsAndBlendCaches()
{
    RigExecBakedProgramImpl B;
    B.chains.resize(1);
    B.chains[0].revisions.resize(1);
    auto &revision = B.chains[0].revisions[0];
    auto topology = std::make_shared<RigExecSkinTopology>();
    topology->indices = {0};
    topology->weights = {1.0f};
    topology->elementSize = 1;
    topology->pointCount = 1;
    topology->influenceCount = 1;
    topology->validated = true;
    revision.layoutHandle = revision.layoutCandidate = topology;
    revision.topology = revision.partitionTopology = topology;
    revision.topologyResolved = true;
    revision.blendChannels.resize(1);
    revision.blendChannels[0].samples.resize(1);
    auto &sample = revision.blendChannels[0].samples[0];
    auto layout = std::make_shared<RigExecBlendSampleLayout>();
    layout->offsets = {GfVec3f(1, -0.0f, 0)};
    layout->indices = {0};
    layout->pointCount = 1;
    layout->valid = true;
    sample.layout = layout;
    sample.lastPoints = {GfVec3f(2, 0, 0)};
    sample.layoutRefused = true;
    RigExecBakedRunShadow shadow;
    shadow.Capture(B);

    // Independent allocations retain semantic equality; replay restoration
    // must restore all owned aliases and the sample cache, not only packets.
    revision.topology = std::make_shared<RigExecSkinTopology>(*topology);
    revision.partitionTopology = revision.topology;
    sample.layout = std::make_shared<RigExecBlendSampleLayout>(*layout);
    std::vector<std::string> differences;
    CHECK(shadow.Compare(B, &differences) == 0);
    revision.topologyResolved = false;
    sample.lastPoints[0] = GfVec3f(9, 0, 0);
    sample.layoutRefused = false;
    CHECK(shadow.Compare(B, &differences) > 0);
    shadow.Restore(&B);
    CHECK(revision.topology == topology);
    CHECK(revision.partitionTopology == topology);
    CHECK(revision.topology == revision.layoutHandle);
    CHECK(revision.topologyResolved);
    CHECK(sample.layout == layout);
    CHECK(sample.lastPoints == std::vector<GfVec3f>{GfVec3f(2, 0, 0)});
    CHECK(sample.layoutRefused);
    differences.clear();
    CHECK(shadow.Compare(B, &differences) == 0);
    auto changed = std::make_shared<RigExecBlendSampleLayout>(*layout);
    changed->offsets[0][1] = 0.0f;
    sample.layout = changed;
    CHECK(shadow.Compare(B, &differences) > 0);
}

void
TestVerifierRestoresInputsAndChecksSelectedWork()
{
    RigExecBakedProgramImpl B;
    RigExecResolvedInputs inputs;
    B.resolvedInputs = &inputs;
    const SdfPath input("/Rig/Channels.value");
    inputs.SetProperty(input,VtValue(.25f));
    B.propertyResults[input] = VtValue(.25f);
    B.posedD = {GfMatrix4d(2)};
    B.parentSpaceM = {GfMatrix4d(3)};
    B.parentSpaceAuthored = {1};
    B.rotationSign = {5};
    B.lastPosedD = {GfMatrix4d(4)};
    B.lastParentSpaceM = {GfMatrix4d(5)};
    B.lastParentSpaceAuthored = {1};
    B.lastRotationSign = {3};
    B.headOverrides = {VtValue(.4f)};
    B.lastHeadOverrides = {VtValue(.2f)};
    B.headOverrideMoved = {1};
    B.readerWalkMoved = {1};
    B.readerWalkChanged = {1};
    B.chains.resize(1);
    B.chains[0].revisions.resize(1);
    B.chains[0].scheduleDirty = true;
    B.chains[0].revisions[0].created = true;
    RigExecBakedRunShadow before;
    before.Capture(B);
    B.posedD[0] = GfMatrix4d(9);
    B.parentSpaceM[0] = GfMatrix4d(9);
    B.parentSpaceAuthored[0] = 0;
    B.rotationSign[0] = 0;
    B.lastPosedD[0] = GfMatrix4d(9);
    B.lastParentSpaceM[0] = GfMatrix4d(9);
    B.lastParentSpaceAuthored[0] = 0;
    B.lastRotationSign[0] = 0;
    B.headOverrides.clear(); B.lastHeadOverrides.clear();
    B.headOverrideMoved[0] = B.readerWalkMoved[0] = B.readerWalkChanged[0] = 0;
    inputs.SetProperty(input,VtValue(.75f));
    B.propertyResults.clear();
    B.chains[0].scheduleDirty = B.chains[0].revisions[0].created = false;
    before.Restore(&B);
    CHECK(B.posedD[0] == GfMatrix4d(2));
    CHECK(B.parentSpaceM[0] == GfMatrix4d(3));
    CHECK(B.parentSpaceAuthored[0] == 1 && B.rotationSign[0] == 5);
    CHECK(B.lastPosedD[0] == GfMatrix4d(4));
    CHECK(B.lastParentSpaceM[0] == GfMatrix4d(5));
    CHECK(B.lastParentSpaceAuthored[0] == 1 && B.lastRotationSign[0] == 3);
    CHECK(B.headOverrides[0].UncheckedGet<float>() == .4f);
    CHECK(B.lastHeadOverrides[0].UncheckedGet<float>() == .2f);
    CHECK(B.headOverrideMoved[0] && B.readerWalkMoved[0] && B.readerWalkChanged[0]);
    CHECK(inputs.Find(input) && inputs.Find(input)->UncheckedGet<float>() == .25f);
    CHECK(B.propertyResults.at(input).UncheckedGet<float>() == .25f);
    CHECK(B.chains[0].scheduleDirty && B.chains[0].revisions[0].created);

    B.steps.resize(2);
    B.steps[0].kind = RigExecBakedStepKind::RevisionStatic;
    B.steps[1].kind = RigExecBakedStepKind::RevisionFuse;
    for (auto &step : B.steps) { step.object = 0; step.label = "verifier fixture"; }
    B.revisionIndex.emplace_back(0,0);
    B.opGraph.ops.resize(2);
    B.opGraph.ops[0].originalIndex = 0;
    B.opGraph.ops[1].originalIndex = 1;
    B.opExecution.ran = {0,0};
    auto &revision = B.chains[0].revisions[0];
    revision.staticDirty = revision.executed = false;
    revision.output = {GfVec3f(1,2,3)};
    revision.envelope = {.25f};
    B.providerProgram.valueKeys = {"raw:/Rig/Channels.value"};
    B.providerValues.values.resize(1);
    B.providerValues.PublishSource(0,VtValue(.25f),false,true);
    B.steps[0].counters.revisionsBuilt = 1;
    B.steps[1].counters.chainsBuilt = 1;
    RigExecBakedRunShadow cone;
    cone.Capture(B);
    // Identical values from a forced run with different, truthful work facts.
    B.opExecution.ran = {1,1};
    revision.staticDirty = revision.executed = true;
    B.steps[1].counters.revisionsExecuted = 1;
    std::vector<std::string> differences;
    CHECK(cone.Compare(B,&differences) == 0);
    revision.envelope[0] = .5f;
    CHECK(cone.Compare(B,&differences) != 0);
    revision.envelope[0] = .25f;
    revision.output[0][0] = 9;
    CHECK(cone.Compare(B,&differences) != 0);
    revision.output[0][0] = 1;
    B.providerValues.PublishSource(0,VtValue(.5f),false,true);
    CHECK(cone.Compare(B,&differences) != 0);
    B.providerValues.PublishSource(0,VtValue(.25f),false,false);
    CHECK(cone.Compare(B,&differences) != 0);
    B.providerValues.PublishSource(0,VtValue(.25f),false,true);
    CHECK(cone.Compare(B,&differences) == 0);
    // A skipped body cannot claim executed work, even with identical values.
    cone.Restore(&B);
    B.steps[1].counters.revisionsExecuted = 1;
    RigExecBakedRunShadow incorrectCone;
    incorrectCone.Capture(B);
    revision.staticDirty = revision.executed = true;
    CHECK(incorrectCone.Compare(B,&differences) != 0);
    cone.Restore(&B);
    CHECK(!revision.executed && !revision.staticDirty);
    CHECK(B.steps[1].counters.revisionsExecuted == 0);
    CHECK(B.opExecution.ran == std::vector<char>({0,0}));
    B.closed.Resize(2); B.closed.Set(0);
    B.closedSteps.Resize(2); B.closedSteps.Set(0);
    B.closureFull = false;
    const RigExecBakedRunStatistics statistics(B);
    B.closed.Set(1); B.closedSteps.Set(1); B.closureFull = true;
    statistics.Restore(&B);
    CHECK(B.closed.Test(0) && !B.closed.Test(1));
    CHECK(B.closedSteps.Test(0) && !B.closedSteps.Test(1));
    CHECK(!B.closureFull);
}

static void
TestProviderRefreshFailureBoundaries()
{
    const auto translated = [](double x) {
        RigExecPointFrame frame;
        for (auto &point : frame.points) point[0] += x;
        return frame;
    };
    const auto make = [&](size_t count) {
        auto program = std::make_unique<RigExecBakedProgramImpl>();
        auto &B = *program;
        B.paths = {SdfPath("/Rig/P"), SdfPath("/Rig/P/A"), SdfPath("/Rig/P/B")};
        B.pathTexts = RigExecBakedSpellPathTexts(B.paths);
        B.base.resize(6); B.fin.resize(6);
        for (size_t slot = 0; slot < 3; ++slot) {
            B.base[slot] = translated(double(slot + 1));
            B.fin[slot] = translated(double(slot + 4));
            B.base[slot + 3] = translated(100);
            B.fin[slot + 3] = translated(200);
        }
        B.providerValues = RigExecTypedValueStore(2);
        B.providerValues.Publish(0, translated(7));
        B.providerValues.Publish(1, translated(11));
        RigExecBakedProgramImpl::ProviderRefresh refresh;
        refresh.slot = 0;
        refresh.baseValue = 0; refresh.currentValue = 1;
        refresh.baseRead = 0; refresh.finRead = 0;
        refresh.baseWrite = 3; refresh.finWrite = 3;
        for (size_t k = 0; k < count; ++k) {
            RigExecBakedProgramImpl::ProviderRefresh::Carry carry;
            carry.slot = int(k + 1);
            carry.baseRead = carry.finRead = uint32_t(k + 1);
            carry.baseWrite = carry.finWrite = uint32_t(k + 4);
            refresh.carries.push_back(std::move(carry));
        }
        refresh.baseInputs.resize(count); refresh.finInputs.resize(count);
        refresh.baseOutputs.resize(count); refresh.finOutputs.resize(count);
        refresh.blocked.resize(count);
        B.providerRefreshes.push_back(std::move(refresh));
        return program;
    };
    const auto run = [](RigExecBakedProgramImpl &B) {
        RigExecBakedStep step;
        step.kind = RigExecBakedStepKind::ProviderRefresh; step.object = 0;
        RigExecBakedRunProviderRefresh(&B, &step);
        return step.diagnostics;
    };
    {
        auto program = make(2); auto &B = *program;
        B.providerValues.values[1] = RigExecTypedValueState();
        const auto diagnostics = run(B);
        CHECK(B.base[3] == translated(1) && B.fin[3] == translated(4));
        CHECK(B.base[4] == translated(2) && B.fin[4] == translated(5));
        CHECK(B.base[5] == translated(3) && B.fin[5] == translated(6));
        CHECK(diagnostics == std::vector<std::string>{"connected final pose input incomplete: /Rig/P"});
    }
    {
        auto program = make(2); auto &B = *program;
        auto invalid = translated(11); invalid.flags = 0;
        B.providerValues.Publish(1, invalid);
        const auto diagnostics = run(B);
        CHECK(B.base[3] == translated(7) && B.fin[3] == translated(4));
        CHECK(B.base[4] == translated(8) && B.fin[4] == translated(5));
        CHECK(B.base[5] == translated(9) && B.fin[5] == translated(6));
        CHECK(diagnostics == std::vector<std::string>{"/Rig/P produced an invalid or degenerate frame for /Rig/P; constraint passed through"});
    }
    for (const size_t count : {size_t(0), size_t(1)}) {
        auto program = make(count); auto &B = *program;
        B.fin[0].points.fill(GfVec3d(0));
        if (count) {
            B.providerParentRawLeaves = {-1, 0, -1};
            B.providerLeafBlocked = {0};
            GfMatrix4d parent(1.0); parent.SetTranslate(GfVec3d(1, 0, 0));
            B.providerLeaves.values = {VtValue(parent)};
            B.providerRefreshes[0].carries[0].blockingSlots = {1};
        }
        CHECK(run(B).empty());
        CHECK(B.base[3] == translated(7) && B.fin[3] == translated(11));
        if (count) CHECK(B.base[4] == translated(2) && B.fin[4] == translated(5));
    }
    {
        auto program = make(2); auto &B = *program;
        B.fin[2].flags = 0;
        const auto invalid = B.fin[2];
        const auto diagnostics = run(B);
        CHECK(B.base[3] == translated(7) && B.fin[3] == translated(4));
        CHECK(B.base[4] == translated(8) && B.fin[4] == translated(5));
        CHECK(B.base[5] == translated(9) && B.fin[5] == invalid);
        CHECK(diagnostics == std::vector<std::string>{"/Rig/P could not propagate its pose revision through /Rig/P/B; constraint passed through"});
    }
    {
        auto program = make(1); auto &B = *program;
        B.commits.resize(1); B.commits[0].present = {1}; B.commits[0].abandoned = false;
        B.providerRefreshes[0].priorConstraints = {{0, 0}};
        CHECK(run(B).empty());
        CHECK(B.base[3] == translated(1) && B.fin[3] == translated(4));
        CHECK(B.base[4] == translated(2) && B.fin[4] == translated(5));
        B.commits[0].abandoned = true;
        CHECK(run(B).empty());
        CHECK(B.base[3] == translated(7) && B.fin[3] == translated(11));
        CHECK(B.base[4] == translated(8) && B.fin[4] == translated(12));
    }
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
        CHECK(ScheduleByHand(&B));
        std::string error;
        CHECK(RigExecBakedValidateStepGraph(B, &error));
        CHECK(error.empty());
        CHECK(B.steps[1].preds == std::vector<int>{0});
    }
    {
        // Declaration order does not constrain execution: the exact producer
        // emitted second must precede its consumer in the canonical graph.
        RigExecBakedProgramImpl B;
        B.steps.push_back(HandStep({RigExecBakedOne(D::WeightPacket,1)},
                                  {RigExecBakedOne(D::WeightPacket,0)}));
        B.steps.push_back(HandStep({}, {RigExecBakedOne(D::WeightPacket,1)}));
        CHECK(ScheduleByHand(&B));
        CHECK(B.steps[0].writes.front()==RigExecBakedOne(D::WeightPacket,1));
        CHECK(B.steps[1].preds==std::vector<int>{0});
        std::string error;
        CHECK(RigExecBakedValidateStepGraph(B,&error));
    }
    {
        // Aggregate inputs bind their exact producer in either declaration
        // order; execution never depends on a cached prior-generation value.
        RigExecBakedProgramImpl B;
        B.steps.push_back(HandStep({RigExecBakedOne(D::Aggregate, 1)},
                                   {RigExecBakedOne(D::Aggregate, 0)}));
        B.steps.push_back(HandStep({}, {RigExecBakedOne(D::Aggregate, 1)}));
        CHECK(ScheduleByHand(&B));
        CHECK(B.steps[0].writes.front()==RigExecBakedOne(D::Aggregate,1));
        CHECK(B.steps[1].preds==std::vector<int>{0});
        std::string canonicalError;
        CHECK(RigExecBakedValidateStepGraph(B,&canonicalError));
        RigExecBakedProgramImpl ordered;
        ordered.steps.push_back(
            HandStep({}, {RigExecBakedOne(D::Aggregate, 1)}));
        ordered.steps.push_back(HandStep({RigExecBakedOne(D::Aggregate, 1)},
                                         {RigExecBakedOne(D::Aggregate, 0)}));
        CHECK(ScheduleByHand(&ordered));
        std::string error;
        CHECK(RigExecBakedValidateStepGraph(ordered, &error));
        CHECK(error.empty());
    }
    {
        RigExecBakedProgramImpl B;
        B.steps.push_back(HandStep({}, {RigExecBakedOne(D::CommitTable, 0)}));
        B.steps.push_back(HandStep(
            {RigExecBakedRange(D::CommitTable, 0, 4)}, {}));
        std::string error;
        CHECK(!ScheduleByHand(&B,&error));
        CHECK(error.find("read has no producer or declared sampled source")!=std::string::npos);
        CHECK(error.find("domain "+std::to_string(uint32_t(D::CommitTable))+" slot 1")!=std::string::npos);
    }
    {
        // A self-read is a real SCC. SetAside excludes its body and resets
        // its retained output every generation, including held visits.
        RigExecBakedProgramImpl B;
        B.weightPackets.resize(1);
        B.steps.push_back(HandStep({RigExecBakedOne(D::WeightPacket,0)},
                                  {RigExecBakedOne(D::WeightPacket,0)}));
        CHECK(ScheduleByHand(&B));
        CHECK(B.opGraph.cycles.size()==1 && B.excludedSteps.size()==1);
        CHECK(B.steps.empty());
        for(int visit=0;visit<2;++visit) {
            B.weightPackets[0].valid=true;
            B.weightPackets[0].values={0.75f};
            CHECK(RigExecBakedExecuteOpGraph(&B,UsdTimeCode(1),false));
            CHECK(!B.weightPackets[0].valid && B.weightPackets[0].values.empty());
        }
    }
    {
        // SCC placement outputs must lose stale validity on every visit.
        RigExecBakedProgramImpl B;
        B.volumePlacement.resize(1,GfMatrix4d(1));
        B.volumePlacementBase.resize(1,GfMatrix4d(1));
        B.placedVolumes={1};
        for(auto domain:{D::WeightFrames,D::WeightFramesBase})
            B.steps.push_back(HandStep({RigExecBakedOne(domain,0)},
                                      {RigExecBakedOne(domain,0)}));
        CHECK(ScheduleByHand(&B));
        CHECK(B.steps.empty() && B.opAdapter.excludedValues.size()==2);
        for(int visit=0;visit<2;++visit) {
            B.volumePlacement[0]=B.volumePlacementBase[0]=GfMatrix4d(1);
            CHECK(RigExecBakedExecuteOpGraph(&B,UsdTimeCode(1),false));
            CHECK(std::isnan(B.volumePlacement[0][3][0]));
            CHECK(std::isnan(B.volumePlacementBase[0][3][0]));
            CHECK(B.placedVolumes[0]==1);
        }
    }
    {
        // A source domain needs no producer; the retired Snapshots domain
        // may be neither written nor read.
        RigExecBakedProgramImpl B;
        B.solvers.resize(1);B.chains.resize(1);
        B.steps.push_back(HandStep({RigExecBakedOne(D::SolverPoints, 0),
                                    RigExecBakedOne(D::ChainInput, 0)},
                                   {RigExecBakedOne(D::WeightPacket, 0)}));
        B.steps.push_back(HandStep({RigExecBakedOne(D::WeightPacket, 0)},
                                   {}));
        CHECK(ScheduleByHand(&B));
        std::string error;
        CHECK(RigExecBakedValidateStepGraph(B, &error));
        CHECK(error.empty());
        B.steps[0].writes.push_back(RigExecBakedOne(D::Snapshots, 0));
        ExpectRejected(B, "a write of the retired store",
                       {"step 0", "declares the retired Snapshots domain"});
        B.steps[0].writes.pop_back();
        B.steps[1].reads.push_back(RigExecBakedOne(D::Snapshots, 0));
        ExpectRejected(B, "a read of the retired store",
                       {"step 1", "declares the retired Snapshots domain"});
    }
    {
        RigExecBakedProgramImpl B;
        B.steps.push_back(HandStep({}, {RigExecBakedOne(D::WeightPacket,0)}));
        B.steps.push_back(HandStep({}, {RigExecBakedOne(D::WeightPacket,0)}));
        std::string error;
        B.steps[0].descriptorKey="first"; B.steps[1].descriptorKey="duplicate";
        CHECK(!RigExecBakedCompileOpGraph(&B,&error));
        CHECK(error.find("multiple typed producers")!=std::string::npos);
        CHECK(error.find("first")!=std::string::npos);
        CHECK(error.find("duplicate")!=std::string::npos);
    }
    // A three-step chain, valid as built; each case below breaks one thing.
    const auto chain = [](RigExecBakedProgramImpl *B) {
        B->steps.push_back(
            HandStep({}, {RigExecBakedOne(D::WeightPacket, 0)}));
        B->steps.push_back(HandStep({RigExecBakedOne(D::WeightPacket, 0)},
                                    {RigExecBakedOne(D::WeightPacket, 1)}));
        B->steps.push_back(
            HandStep({RigExecBakedOne(D::WeightPacket, 1)}, {}));
        CHECK(ScheduleByHand(B));
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
        auto graph=B.opGraph;
        CHECK(graph.clusters.size()==3);
        std::reverse(graph.clusters.begin(),graph.clusters.end());
        std::string error;
        CHECK(!RigExecValidateOpClusters(graph,&error));
        CHECK(error.find("invalid operation cluster member")!=std::string::npos);
    }
    {
        RigExecBakedProgramImpl B; chain(&B);
        auto graph=B.opGraph;
        graph.clusters.pop_back();
        std::string error;
        CHECK(!RigExecValidateOpClusters(graph,&error));
        CHECK(error.find("operation missing from cluster lowering")!=std::string::npos);
    }
    {
        RigExecBakedProgramImpl B; chain(&B);
        auto graph=B.opGraph;
        graph.clusters[2].predecessors.clear();
        graph.clusters[1].successors.clear();
        std::string error;
        CHECK(!RigExecValidateOpClusters(graph,&error));
        CHECK(error.find("cluster edges differ from operation dependencies")!=std::string::npos);
    }
    {
        RigExecBakedProgramImpl B; chain(&B);
        auto graph=B.opGraph;
        std::swap(graph.clusters[0].members,graph.clusters[1].members);
        std::string error;
        CHECK(!RigExecValidateOpClusters(graph,&error));
        CHECK(error.find("invalid operation cluster member")!=std::string::npos);
    }
}

/// Placement values feed a current-phase field, whose assembly consumer
/// reads the produced field rather than measuring the points a second time.
void
TestAPlacementReadNeedsItsVolumesStep()
{
    using D = RigExecBakedSlotDomain;
    const auto build = [](RigExecBakedProgramImpl *B,
                         std::vector<RigExecBakedSlotRange> placementReads,
                         std::string *compileError=nullptr) {
        B->paths = {SdfPath("/Rig/Joint"),SdfPath("/Rig/Joint/SphereA"),SdfPath("/Rig/SphereB")};
        B->pathTexts = RigExecBakedSpellPathTexts(B->paths);
        B->xformSlots = {1,2};
        B->slotKind.assign(3,RigExecBakedSlotKind::XformDerived);
        B->base.resize(3); B->fin.resize(3);
        B->baseLast = B->finLast = {0,1,2};
        B->chains.resize(1); B->chains[0].revisions.resize(1);
        auto &revision = B->chains[0].revisions[0];
        revision.moverPath = SdfPath("/Rig/Movers/Smooth");
        revision.op = RigExecRevisionOp::Smooth;
        revision.weightCurrentPhase = true;
        revision.weightObject = revision.weightField = 0;
        B->revisionIndex = {{0,0}}; B->revisionFuseStep = {-1};
        B->chainRevisionBegin = {0}; B->chainRevisionEnd = {1};
        B->weightObjects.resize(1);
        B->weightObjects[0].path = SdfPath("/Rig/Weights/Field");
        B->weightFields.resize(1);
        auto &field = B->weightFields[0];
        field.object = field.consumer = 0;
        field.form = RigExecBakedProgramImpl::WeightField::Form::Revision;
        field.placementPhase = RigExecBakedProgramImpl::WeightField::PlacementPhase::Final;
        for (const auto &range : placementReads)
            for (uint32_t slot=range.begin;slot<range.end;++slot) field.volumes.push_back(int(slot));
        RigExecBakedStep base;
        base.kind = RigExecBakedStepKind::ChainInputs; base.object = 0;
        base.reads = {RigExecBakedOne(D::ChainInput,0)};
        base.writes = {RigExecBakedOne(D::ChainBase,0)};
        B->steps.push_back(base);
        RigExecBakedStep packet;
        packet.kind = RigExecBakedStepKind::WeightPacket; packet.object = 0;
        packet.writes = {RigExecBakedOne(D::WeightPacket,0)};
        B->steps.push_back(packet);
        for (int slot : {1,2}) {
            RigExecBakedStep step;
            step.kind = RigExecBakedStepKind::VolumePlacements; step.object = slot; step.part = 1;
            step.reads = {RigExecBakedOne(D::PoseFin,slot)};
            step.writes = {RigExecBakedOne(D::WeightFrames,slot)};
            B->steps.push_back(step);
        }
        RigExecBakedStep measure;
        measure.kind = RigExecBakedStepKind::WeightField; measure.object = 0;
        measure.reads = std::move(placementReads);
        measure.reads.push_back(RigExecBakedOne(D::ChainBase,0));
        measure.writes = {RigExecBakedOne(D::WeightField,0)};
        B->steps.push_back(measure);
        RigExecBakedStep assemble;
        assemble.kind = RigExecBakedStepKind::RevisionStatic; assemble.object = 0;
        assemble.reads = {RigExecBakedOne(D::ChainBase,0),RigExecBakedOne(D::WeightPacket,0),RigExecBakedOne(D::WeightField,0)};
        assemble.writes = {RigExecBakedOne(D::RevisionPacket,0)};
        B->steps.push_back(assemble);
        RigExecBakedStep fuse;
        fuse.kind = RigExecBakedStepKind::RevisionFuse; fuse.object = 0;
        fuse.reads = {RigExecBakedOne(D::ChainBase,0),RigExecBakedOne(D::RevisionPacket,0)};
        fuse.writes = {RigExecBakedOne(D::RevisionDone,0),RigExecBakedOne(D::ChainDirty,0)};
        B->steps.push_back(fuse);
        return ScheduleByHand(B,compileError);
    };
    const auto index = [](const RigExecBakedProgramImpl &B,RigExecBakedStepKind kind,int object) {
        for (size_t s=0;s<B.steps.size();++s)
            if (B.steps[s].kind==kind && B.steps[s].object==object) return int(s);
        return -1;
    };
    for (const std::vector<int> volumes : {std::vector<int>{2},std::vector<int>{1,2}}) {
        RigExecBakedProgramImpl B;
        std::vector<RigExecBakedSlotRange> reads;
        for (int slot : volumes) reads.push_back(RigExecBakedOne(D::WeightFrames,slot));
        std::string error;
        CHECK(build(&B,reads,&error));
        CHECK(RigExecBakedValidateStepGraph(B,&error));
        if (!error.empty()) std::printf("FAIL placement field: %s\n",error.c_str());
        CHECK(error.empty());
        const int a=index(B,RigExecBakedStepKind::VolumePlacements,1);
        const int b=index(B,RigExecBakedStepKind::VolumePlacements,2);
        const int measured=index(B,RigExecBakedStepKind::WeightField,0);
        const int assembled=index(B,RigExecBakedStepKind::RevisionStatic,0);
        CHECK(a>=0 && b>=0 && measured>=0 && assembled>=0);
        if (a<0 || b<0 || measured<0 || assembled<0) continue;
        CHECK(B.steps[size_t(a)].label == "VolumePlacements /Rig/Joint/SphereA");
        CHECK(B.steps[size_t(b)].label == "VolumePlacements /Rig/SphereB");
        std::vector<int> expected{index(B,RigExecBakedStepKind::ChainInputs,0)};
        for (int slot : volumes) expected.push_back(index(B,RigExecBakedStepKind::VolumePlacements,slot));
        std::sort(expected.begin(),expected.end());
        CHECK(B.steps[size_t(measured)].preds == expected);
        CHECK(std::binary_search(B.steps[size_t(assembled)].preds.begin(),B.steps[size_t(assembled)].preds.end(),measured));
        auto &consumer=B.steps[size_t(assembled)];
        const auto savedReads=consumer.reads;
        consumer.reads.erase(std::remove_if(consumer.reads.begin(),consumer.reads.end(),[](const auto &read) {
            return read.domain==D::WeightField;
        }),consumer.reads.end());
        CHECK(!RigExecBakedValidateStepGraph(B,&error));
        CHECK(error.find("adopts its WeightField without declaring it")!=std::string::npos);
        consumer.reads=savedReads;
        auto &producer=B.steps[size_t(measured)];
        const auto savedFieldReads=producer.reads;
        producer.reads.erase(std::remove_if(producer.reads.begin(),producer.reads.end(),[](const auto &read) {
            return read.domain==D::ChainBase;
        }),producer.reads.end());
        CHECK(!RigExecBakedValidateStepGraph(B,&error));
        CHECK(error.find("omits its entering base")!=std::string::npos);
        producer.reads=savedFieldReads;
    }
    RigExecBakedProgramImpl B;
    std::string error;
    CHECK(!build(&B,{RigExecBakedOne(D::WeightFrames,1),RigExecBakedOne(D::WeightFrames,3)},&error));
    CHECK(!B.opAdapter.compiled);
    CHECK(error.find("read has no producer or declared sampled source")!=std::string::npos);
    CHECK(error.find("domain "+std::to_string(uint32_t(D::WeightFrames))+" slot 3")!=std::string::npos);
}

std::string
CanonicalLabel(const RigExecBakedProgramImpl &B, RigExecBakedStepKind kind,
               int object)
{
    for (const auto &step : B.steps)
        if (step.kind == kind && step.object == object)
            return "(" + step.label + ")";
    CHECK(false);
    return "(missing operation)";
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
                       {CanonicalLabel(B, RigExecBakedStepKind::SolverCommit, 0) + " is bound to PoseFin version",
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
                           {"(" + solver->path.GetString() + ")",
                            "is bound to PoseFin version",
                            "writes at or after it"});
            solver->controlReads.front() = bound;
            passes("a solver control read of a later version");
            const size_t solverIndex = size_t(solver - B.solvers.data());
            auto solve = std::find_if(B.steps.begin(), B.steps.end(), [&](const auto &step) {
                return step.kind == RigExecBakedStepKind::Solve &&
                       step.object == int(solverIndex);
            });
            CHECK(solve != B.steps.end());
            if (solve != B.steps.end()) {
                const auto reads = solve->reads;
                solve->reads.erase(std::remove_if(solve->reads.begin(), solve->reads.end(),
                    [&](const auto &read) { return read.domain == RigExecBakedSlotDomain::PoseFin &&
                        read.begin <= bound && bound < read.end; }), solve->reads.end());
                CHECK(solve->reads.size() < reads.size());
                ExpectRejected(B, "a solver body binding without its declared input",
                    {"(" + solver->path.GetString() + ") is bound to PoseFin version " +
                        std::to_string(bound), "without declaring it"});
                solve->reads = reads;
                passes("a solver body binding without its declared input");
            }
        }
    }
    {
        // A candidate carry reads the entering version. Neither a later
        // writer nor its own still-unwritten output is a valid fallback.
        const uint32_t carried = first.slotCarry.front();
        first.slotCarry.front() = later;
        ExpectRejected(B, "a carry of another commit's later version",
                       {CanonicalLabel(B, RigExecBakedStepKind::SolverCommit, 0) + " is bound to PoseFin version",
                        "writes at or after it"});
        first.slotCarry.front() = first.slotWrites.front();
        // A missing candidate executes carry before its own output exists.
        // Naming that output would retain the prior run's frame.
        ExpectRejected(B, "a candidate carrying its own unreadable output",
            {CanonicalLabel(B, RigExecBakedStepKind::SolverCommit, 0) +
                " is bound to PoseFin version " + std::to_string(first.slotWrites.front()),
             "writes at or after it"});
        first.slotCarry.front() = carried;
        passes("a carry of another commit's later version");
    }
    {
        // Candidate outcomes (produced or carried) are written before the
        // descendant loop. A descendant of the same provider may carry that
        // earlier output, with the same slot identity and a distinct SSA ID.
        const auto original = first;
        const size_t finSize = B.fin.size(), baseSize = B.base.size();
        auto apply = std::find_if(B.steps.begin(), B.steps.end(), [](const auto &step) {
            return step.kind == RigExecBakedStepKind::SolverCommit && step.object == 0;
        });
        CHECK(apply != B.steps.end() && !first.split && first.solverOutput);
        if (apply != B.steps.end() && !first.split && first.solverOutput) {
            const auto writes = apply->writes;
            first.propagate.emplace_back(first.slots.front(), first.slots.front());
            first.descendantReads.push_back(first.slotReads.front());
            first.closestReads.push_back(first.slotReads.front());
            first.descendantCarry.push_back(first.slotWrites.front());
            first.descendantBaseCarry.push_back(first.slotBaseWrites.front());
            first.descendantWrites.push_back(uint32_t(B.fin.size()));
            first.descendantBaseWrites.push_back(uint32_t(B.base.size()));
            B.fin.emplace_back(); B.base.emplace_back();
            apply->writes.push_back(RigExecBakedOne(RigExecBakedSlotDomain::PoseFin,
                                                   first.descendantWrites.back()));
            apply->writes.push_back(RigExecBakedOne(RigExecBakedSlotDomain::PoseBase,
                                                   first.descendantBaseWrites.back()));
            std::string internal;
            CHECK(RigExecBakedValidateStepGraph(B, &internal));
            if (!internal.empty()) std::printf("FAIL valid earlier-candidate carry: %s\n", internal.c_str());
            first = original; B.fin.resize(finSize); B.base.resize(baseSize);
            apply->writes = writes;
            passes("an earlier candidate carried by its descendant");
        }
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
                    slot = uint32_t(B.steps[index].object);
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
                                " " + CanonicalLabel(B, RigExecBakedStepKind::ProviderMatrix, int(slot)),
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
                       {CanonicalLabel(B, RigExecBakedStepKind::SolverCommit, int(B.commits.size()-1)) + " writes PoseFin version " +
                            std::to_string(later),
                        CanonicalLabel(B, RigExecBakedStepKind::SolverCommit, 0) + " writes too"});
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

/// Frame records on frame_record_fallbacks (three constraints on one joint,
/// AtPrim matrix-mover readers), each edited to break one rule the validator
/// owns: a record bound to a later commit's version of its provider, a fold
/// reading a record no step evaluates, a record with no FrameMatrix
/// step, and a FrameMatrix step naming a negative record. Each case restores what it broke, and the restored program must
/// pass again.
void
TestTheValidatorRejectsABadFrameRecord(const std::string &fixtures)
{
    BuiltProgram built = Build(fixtures + "/frame_record_fallbacks.usda");
    CHECK(built.program != nullptr);
    if (!built.program) {
        return;
    }
    RigExecBakedProgramImpl &B =
        const_cast<RigExecBakedProgramImpl &>(built.program->GetStepGraph());
    TestTheValidatorAcceptsTheProgram(built, "frame_record_fallbacks");
    const auto passes = [&](const char *what) {
        std::string restored;
        if (!RigExecBakedValidateStepGraph(B, &restored)) {
            ++failures;
            std::printf("FAIL %s: rejected after the restore: %s\n", what,
                        restored.c_str());
        }
    };
    // The records in walk order: C1, C2, C3.
    CHECK(B.frameRecords.size() == 3);
    if (B.frameRecords.size() != 3) {
        return;
    }
    {
        // C1's record bound to the version C3's commit writes.
        RigExecBakedFrameRecord &first = B.frameRecords.front();
        const RigExecBakedFrameRecord &last = B.frameRecords.back();
        const uint32_t bound = first.version;
        first.version = last.version;
        ExpectRejected(
            B, "a frame record bound to a later commit's version",
            {CanonicalLabel(B, RigExecBakedStepKind::FrameMatrix, 0) +
                 " is bound to PoseFin version " + std::to_string(last.version) +
                 " of /RecordAsset/Rig/Joints/X",
             "(/RecordAsset/Rig/Movers/Constrain/C3) writes at or after it"});
        first.version = bound;
        passes("a frame record bound to a later commit's version");
    }
    {
        // A fold reading one record past the table.
        RigExecBakedStep *fold = nullptr;
        for (RigExecBakedStep &step : B.steps) {
            if (step.kind == RigExecBakedStepKind::InfluenceFold) {
                for (const RigExecBakedSlotRange &read : step.reads) {
                    if (read.domain == RigExecBakedSlotDomain::FrameMatrix) {
                        fold = &step;
                    }
                }
            }
        }
        CHECK(fold != nullptr);
        if (fold) {
            const int unwritten = int(B.frameRecords.size());
            fold->reads.push_back(RigExecBakedOne(
                RigExecBakedSlotDomain::FrameMatrix, unwritten));
            ExpectRejected(B, "a fold reading a record no step evaluates",
                           {"(" + fold->label + ")",
                            "reads FrameMatrix[" + std::to_string(unwritten),
                            "which no step writes"});
            fold->reads.pop_back();
            passes("a fold reading a record no step evaluates");
        }
    }
    {
        // A record no FrameMatrix step evaluates.
        B.frameRecords.push_back(B.frameRecords.front());
        ExpectRejected(B, "a frame record with no step",
                       {"frame record 3 of /RecordAsset/Rig/Joints/X after "
                        "/RecordAsset/Rig/Movers/Constrain/Group/C1 has no "
                        "FrameMatrix step"});
        B.frameRecords.pop_back();
        passes("a frame record with no step");
    }
    {
        // A FrameMatrix step naming a negative record.
        RigExecBakedStep *first = nullptr;
        for (RigExecBakedStep &step : B.steps) {
            if (step.kind == RigExecBakedStepKind::FrameMatrix &&
                step.object == 0) {
                first = &step;
            }
        }
        CHECK(first != nullptr);
        if (first) {
            first->object = -1;
            ExpectRejected(B, "a FrameMatrix step naming record -1",
                           {"(" + first->label + ") names frame record -1 of 3"});
            first->object = 0;
            passes("a FrameMatrix step naming record -1");
        }
    }
}

/// A solver's frame record on solver_checkpoint (LegFK's knee) edited to
/// read the wrong entry of its commit's `present` table: a negative
/// position, then the position of another slot LegFK writes. The validator
/// must refuse both, and pass again once the position is restored.
void
TestTheValidatorRejectsABadSolverRecord(const std::string &fixtures)
{
    BuiltProgram built = Build(fixtures + "/solver_checkpoint.usda");
    CHECK(built.program != nullptr);
    if (!built.program) {
        return;
    }
    RigExecBakedProgramImpl &B =
        const_cast<RigExecBakedProgramImpl &>(built.program->GetStepGraph());
    TestTheValidatorAcceptsTheProgram(built, "solver_checkpoint");
    const SdfPath legFk("/CheckpointAsset/Rig/Stack/LegFK");
    RigExecBakedFrameRecord *record = nullptr;
    for (RigExecBakedFrameRecord &candidate : B.frameRecords) {
        if (candidate.mover == legFk) {
            record = &candidate;
        }
    }
    CHECK(record != nullptr);
    if (!record) {
        return;
    }
    const RigExecBakedCommit &commit = B.commits[size_t(record->commit)];
    CHECK(commit.solverOutput);
    int other = -1;
    for (size_t k = 0; k < commit.slots.size(); ++k) {
        if (commit.slots[k] != record->slot) {
            other = int(k);
        }
    }
    CHECK(other >= 0);
    const std::string step = "(" + record->mover.GetString() + ")";
    const auto passes = [&](const char *what) {
        std::string restored;
        if (!RigExecBakedValidateStepGraph(B, &restored)) {
            ++failures;
            std::printf("FAIL %s: rejected after the restore: %s\n", what,
                        restored.c_str());
        }
    };
    const int position = record->position;
    for (const int wrong : {-1, other}) {
        if (wrong == -1 || other >= 0) {
            record->position = wrong;
            ExpectRejected(B, "a solver record reading the wrong position",
                           {step + " reads position " + std::to_string(wrong) +
                                " of commit " +
                                std::to_string(record->commit) +
                                ", which is not the provider it records"});
            record->position = position;
            passes("a solver record reading the wrong position");
        }
    }
}

/// Provider emission retains authored slot order. The shared compiler binds
/// switched sources to exact producers, orders one-way reads, and sets aside
/// both members of a switch SCC rather than retaining last-run matrices.
void
TestASwitchCycleIsDetectedNotEmitted()
{
    using D=RigExecBakedSlotDomain;
    const auto program=[](RigExecBakedProgramImpl *B,bool both) {
        B->posedM.assign(2,GfMatrix4d(3));
        B->steps.push_back(HandStep({RigExecBakedOne(D::PosedM,1)},
                                   {RigExecBakedOne(D::PosedM,0)}));
        B->steps.push_back(HandStep(both ? std::vector<RigExecBakedSlotRange>{RigExecBakedOne(D::PosedM,0)}
                                        : std::vector<RigExecBakedSlotRange>{},
                                   {RigExecBakedOne(D::PosedM,1)}));
        CHECK(ScheduleByHand(B));
    };
    {
        RigExecBakedProgramImpl B; program(&B,false);
        CHECK(B.opGraph.cycles.empty());
        CHECK(B.steps.size()==2);
        CHECK(B.steps[0].writes.front()==RigExecBakedOne(D::PosedM,1));
        CHECK(B.steps[1].preds==std::vector<int>{0});
    }
    {
        RigExecBakedProgramImpl B; program(&B,true);
        CHECK(B.opGraph.cycles.size()==1);
        CHECK(B.excludedSteps.size()==2 && B.steps.empty());
        CHECK(B.opAdapter.excludedValues.size()==2);
        for(int visit=0;visit<2;++visit) {
            B.posedM.assign(2,GfMatrix4d(3));
            CHECK(RigExecBakedExecuteOpGraph(&B,UsdTimeCode(1),false));
            CHECK(std::isnan(B.posedM[0][3][0]) && std::isnan(B.posedM[1][3][0]));
        }
    }
    {
        // A real compose descendant must inherit an excluded parent's
        // unavailable matrix on both first and held generations.
        RigExecBakedProgramImpl B;
        B.posedM.assign(3,GfMatrix4d(1));B.base.resize(3);B.fin.resize(3);
        B.slotKind.assign(3,RigExecBakedSlotKind::FirstFramePose);
        B.parent={-1,-1,0};B.noScaleAvars.assign(3,0);
        B.posedAuthored.assign(3,0);B.posedAuthoredM.assign(3,GfMatrix4d(1));
        B.parentSpaceAuthored.assign(3,0);B.parentSpaceM.assign(3,GfMatrix4d(1));
        B.posedD.assign(3,GfMatrix4d(1));B.parentDinv.assign(3,GfMatrix4d(1));
        B.rotOrder.assign(3,TfToken("XYZ"));B.avars.assign(33,0);
        for(size_t i=0;i<3;++i) {for(size_t k=3;k<6;++k)B.avars[i*11+k]=1;B.avars[i*11+10]=1;}
        B.composeGroups.push_back({2,3,{0}});
        B.steps.push_back(HandStep({RigExecBakedOne(D::PosedM,1)},{RigExecBakedOne(D::PosedM,0)}));
        B.steps.push_back(HandStep({RigExecBakedOne(D::PosedM,0)},{RigExecBakedOne(D::PosedM,1)}));
        auto child=HandStep({RigExecBakedOne(D::PosedM,0)},
            {RigExecBakedOne(D::PoseBase,2),RigExecBakedOne(D::PoseFin,2),RigExecBakedOne(D::PosedM,2)});
        child.kind=RigExecBakedStepKind::ComposeSubtree;child.object=0;B.steps.push_back(child);
        CHECK(ScheduleByHand(&B));CHECK(B.excludedSteps.size()==2 && B.steps.size()==1);
        for(int visit=0;visit<2;++visit) {
            B.posedM[0]=GfMatrix4d(1);B.posedM[1]=GfMatrix4d(1);
            CHECK(RigExecBakedExecuteOpGraph(&B,UsdTimeCode(1),false));
            CHECK(std::isnan(B.posedM[0][3][0]));
            CHECK(!B.base[2].IsValid() || B.base[2].IsDegenerate());
            CHECK(!B.fin[2].IsValid() || B.fin[2].IsDegenerate());
            CHECK(std::isnan(B.posedM[2][3][0]));
        }
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
    // Typed read identities: distinct consumer routes may name the same slot.
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
        std::sort(out.begin(),out.end());
        out.erase(std::unique(out.begin(),out.end()),out.end());
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
        const bool fieldReader = step.kind == RigExecBakedStepKind::WeightField &&
            step.object >= 0 && size_t(step.object) < B.weightFields.size() &&
            B.weightFields[size_t(step.object)].form == RigExecBakedProgramImpl::WeightField::Form::Revision;
        if (step.kind != RigExecBakedStepKind::RevisionChunk &&
            step.kind != RigExecBakedStepKind::RevisionFuse &&
            step.kind != RigExecBakedStepKind::RevisionStatic && !fieldReader) continue;
        const int id = fieldReader ? B.weightFields[size_t(step.object)].consumer : step.object;
        const auto &[c, r] = B.revisionIndex[size_t(id)];
        const auto &revision = B.chains[size_t(c)].revisions[size_t(r)];
        const bool reader = step.kind != RigExecBakedStepKind::RevisionStatic;
        if (step.kind == RigExecBakedStepKind::RevisionStatic && revision.weightField >= 0) {
            CHECK(slots(step.reads,RigExecBakedSlotDomain::WeightField) == std::vector<int>{revision.weightField});
            const auto producer = std::find_if(B.steps.begin(),B.steps.end(),[&](const auto &candidate) {
                return candidate.kind == RigExecBakedStepKind::WeightField && candidate.object == revision.weightField;
            });
            CHECK(producer != B.steps.end());
            if (producer != B.steps.end())
                CHECK(std::binary_search(step.preds.begin(),step.preds.end(),int(producer-B.steps.begin())));
        }
        // A range-pipelined revision's join reads its own ranges and, as a
        // fuse does, the entering version; its range step reads range
        // `part` of a range-pipelined predecessor's buffer and no version,
        // or the version after any other revision.
        const bool rangeEntering =
            !fieldReader && revision.rangeRole &&
            step.kind == RigExecBakedStepKind::RevisionChunk && r > 0 &&
            B.chains[size_t(c)].revisions[size_t(r) - 1].rangeRole;
        std::vector<int> ownChunks;
        if (step.kind == RigExecBakedStepKind::RevisionFuse) {
            for (int k = 0; k < B.revisionChunkCount[size_t(id)]; ++k) {
                ownChunks.push_back(B.revisionChunkBase[size_t(id)] + k);
            }
        }
        if (rangeEntering) {
            ownChunks.push_back(B.revisionChunkBase[size_t(id) - 1] +
                                step.part);
        }
        CHECK(out == ownChunks);
        if (!reader) {
            CHECK(done.empty() && dirty.empty());
            continue;
        }
        ++checked;
        if (r > 0) {
            ++stacked;
            if (B.revisionChunkCount[size_t(id)] > 1) ++stackedChunked;
        }
        if ((r == 0 && !fieldReader) || rangeEntering) {
            CHECK(done.empty() && dirty.empty());
            if (rangeEntering) {
                // Ordered after the predecessor's range step of that part.
                bool ordered = false;
                for (size_t p = 0; p < B.steps.size(); ++p) {
                    const RigExecBakedStep &producer = B.steps[p];
                    if (producer.kind == RigExecBakedStepKind::RevisionChunk &&
                        producer.object == id - 1 &&
                        producer.part == step.part) {
                        ordered = std::binary_search(step.preds.begin(),
                                                     step.preds.end(),
                                                     int(p));
                    }
                }
                CHECK(ordered);
            }
            continue;
        }
        std::vector<int> expectedDone,expectedDirty;
        if (r > 0) {
            if (fieldReader) for (int earlier=B.chainRevisionBegin[size_t(c)];earlier<id;++earlier)
                expectedDone.push_back(earlier);
            else expectedDone.push_back(id-1);
            expectedDirty.push_back(id-1);
        }
        if (fieldReader) {
            const auto &field=B.weightFields[size_t(step.object)];
            const auto bases=slots(step.reads,RigExecBakedSlotDomain::ChainBase);
            const auto finals=slots(step.reads,RigExecBakedSlotDomain::ChainPoints);
            CHECK(std::binary_search(bases.begin(),bases.end(),c));
            // The entering point view and explicitly phased oracle inputs are
            // separate body reads, even when both select the same revision.
            for (const auto &input:field.pointReads) for (const auto &candidate:input.binding.candidates) {
                CHECK(candidate.chain>=0 && size_t(candidate.chain)<B.chains.size());
                if(candidate.chain<0 || size_t(candidate.chain)>=B.chains.size()) continue;
                if(input.binding.finalRead) {
                    CHECK(std::binary_search(finals.begin(),finals.end(),candidate.chain));
                } else if(candidate.version==0) {
                    CHECK(std::binary_search(bases.begin(),bases.end(),candidate.chain));
                } else {
                    const int selected=B.chainRevisionBegin[size_t(candidate.chain)]+candidate.version-1;
                    CHECK(selected>=B.chainRevisionBegin[size_t(candidate.chain)] &&
                          selected<B.chainRevisionEnd[size_t(candidate.chain)]);
                    if(selected<B.chainRevisionBegin[size_t(candidate.chain)] ||
                       selected>=B.chainRevisionEnd[size_t(candidate.chain)]) continue;
                    expectedDone.push_back(selected);
                    expectedDirty.push_back(selected);
                }
            }
        }
        const auto unique=[](std::vector<int> *values) {
            std::sort(values->begin(),values->end());
            values->erase(std::unique(values->begin(),values->end()),values->end());
        };
        unique(&expectedDone); unique(&expectedDirty);
        const bool bound=done==expectedDone && dirty==expectedDirty;
        CHECK(bound);
        bool ordered=true;
        for (int selected:expectedDone) {
            const int producer=B.revisionFuseStep[size_t(selected)];
            ordered=ordered && std::binary_search(step.preds.begin(),step.preds.end(),producer);
        }
        CHECK(ordered);
        if (!bound || !ordered) {
            std::printf("FAIL %s: step %zu (%s) has incorrect entering or phased point reads for version %d of chain %d; done",name,index,step.label.c_str(),r,c);
            for(int selected:done)std::printf(" %d",selected);
            std::printf("; dirty");
            for(int selected:dirty)std::printf(" %d",selected);
            std::printf("; expected done");
            for(int selected:expectedDone)std::printf(" %d",selected);
            std::printf("; expected dirty");
            for(int selected:expectedDirty)std::printf(" %d",selected);
            std::printf("\n");
        }
    }
    std::printf("  %s: %zu chain readers, %zu past the first revision "
                "(%zu of a multi-chunk revision)\n",
                name, checked, stacked, stackedChunked);
    return stackedChunked;
}

/// Appends the chain slots a reader of point version \p r of chain \p c
/// depends on, transitively, in a chain holding range-pipelined revisions:
/// \p mode -2 reads the version whole, k >= 0 range k only. Range k of a
/// range-pipelined revision reads range k of a range-pipelined predecessor;
/// every other reader reads the predecessor's RevisionDone, whose producer
/// (a fuse or a join) reads its own chunks and the version before, whole.
void
AddVersionAncestry(const RigExecBakedProgramImpl &B, int c, int r, int mode,
                   std::vector<RigExecBakedSlotRange> *reads)
{
    const auto &revisions = B.chains[size_t(c)].revisions;
    const int first = B.chainRevisionBegin[size_t(c)];
    for (; r > 0; --r) {
        const int q = first + r - 1;
        const int base = B.revisionChunkBase[size_t(q)];
        const int count = B.revisionChunkCount[size_t(q)];
        const bool range = revisions[size_t(r) - 1].rangeRole;
        if (range && mode >= 0) {
            reads->push_back(
                RigExecBakedOne(RigExecBakedSlotDomain::RevisionOut, base + mode));
            continue;
        }
        reads->push_back(
            RigExecBakedOne(RigExecBakedSlotDomain::RevisionDone, q));
        reads->push_back(
            RigExecBakedOne(RigExecBakedSlotDomain::ChainDirty, q));
        reads->push_back(RigExecBakedRange(RigExecBakedSlotDomain::RevisionOut,
                                           base, base + count));
        // A join, like a fuse, reads its own ranges and the version before
        // its own, whole.
        mode = -2;
    }
}

/// The upper-bound chain reads, rebuilt from the revision tables as the
/// reference the version reads must order identically to: each chunk, fuse
/// and current-phase assemble of revision id > first reads RevisionOut of
/// every earlier chunk of its chain, RevisionDone[first, id) and
/// ChainDirty(id - 1), the fuse reads every chunk of its chain up to its
/// own, and ChainStatus reads every chunk of its chain. On a chain with
/// range-pipelined revisions, whose range steps wait for no join, the reads
/// are AddVersionAncestry's instead.
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
    const auto &revisions = B.chains[size_t(c)].revisions;
    bool pipelined = false;
    for (const auto &revision : revisions) {
        pipelined = pipelined || revision.rangeRole;
    }
    if (pipelined) {
        // A chain with range-pipelined revisions: everything the version
        // read transitively depends on, by the pipelined rules (range k of
        // a range revision reads range k of its predecessor and no join; a
        // join reads its own ranges and the version whole).
        const bool range = revisions[size_t(r)].rangeRole &&
                           step.kind == RigExecBakedStepKind::RevisionChunk;
        if (step.kind == RigExecBakedStepKind::RevisionFuse) {
            reads.push_back(RigExecBakedRange(
                RigExecBakedSlotDomain::RevisionOut,
                B.revisionChunkBase[size_t(id)],
                B.revisionChunkBase[size_t(id)] +
                    B.revisionChunkCount[size_t(id)]));
        }
        const int mode = range ? step.part : -2;
        AddVersionAncestry(B, c, r, mode, &reads);
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

/// Independently rebuild dense typed producer dependencies from declarations.
/// Reads bind their unique producer regardless of descriptor emission order;
/// two reads alone and writes to distinct SSA values add no causal edge.
std::vector<uint64_t>
ConflictClosure(const std::vector<std::vector<RigExecBakedSlotRange>> &reads,
                const std::vector<std::vector<RigExecBakedSlotRange>> &writes,
                size_t *words)
{
    const size_t count=reads.size();
    *words=(count+63)/64;
    std::vector<uint64_t> bits(count * *words,0);
    std::vector<RigExecBakedStep> sites(count);
    for(size_t step=0;step<count;++step)sites[step].writes=writes[step];
    const WriteSiteIndex producers(sites);
    for(size_t step=0;step<count;++step)for(const auto &range:writes[step]) {
        if(range.IsEmpty())continue;
        for(const auto &site:producers.Overlapping(range))
            if(!writes[site.first][site.second].IsEmpty())CHECK(site.first==step);
    }
    std::vector<std::vector<size_t>> direct(count);
    for(size_t step=0;step<count;++step)for(const auto &range:reads[step]) {
        if(range.IsEmpty())continue;
        for(const auto &site:producers.Overlapping(range)) {
            if(site.first==step || writes[site.first][site.second].IsEmpty())continue;
            CHECK(site.first<step);
            if(site.first<step)direct[step].push_back(site.first);
        }
    }
    // Each predecessor's already closed row includes all its ancestors.
    for(size_t step=0;step<count;++step) {
        auto &preds=direct[step];
        std::sort(preds.begin(),preds.end());
        preds.erase(std::unique(preds.begin(),preds.end()),preds.end());
        uint64_t *row=&bits[step * *words];
        for(size_t pred:preds) {
            row[pred/64]|=uint64_t(1)<<(pred%64);
            const uint64_t *closed=&bits[pred * *words];
            for(size_t w=0;w<*words;++w)row[w]|=closed[w];
        }
    }
    return bits;
}

void
TestIndexedQueriesMatchExhaustiveWitnesses()
{
    using D=RigExecBakedSlotDomain;
    const auto exhaustive=[](const std::vector<RigExecBakedStep> &steps,const Reachability &reach) {
        std::vector<AccessWitness> witnesses;
        for(size_t i=0;i<steps.size();++i)for(size_t r=0;r<steps[i].reads.size();++r) {
            const auto &read=steps[i].reads[r];
            if(RigExecBakedIsSourceDomain(read.domain) || read.IsEmpty())continue;
            bool found=false;
            for(size_t e=0;e<i;++e)for(const auto &write:steps[e].writes)
                found=found || write.Overlaps(read);
            if(!found)witnesses.push_back({0,i,r,0,0});
        }
        for(size_t e=0;e<steps.size();++e)for(size_t l=e+1;l<steps.size();++l) {
            if(reach.Ordered(e,l))continue;
            for(size_t a=0;a<steps[e].writes.size();++a) {
                for(size_t b=0;b<steps[l].writes.size();++b)
                    if(steps[e].writes[a].Overlaps(steps[l].writes[b]))
                        witnesses.push_back({1,e,a,l,b});
                for(size_t b=0;b<steps[l].reads.size();++b)
                    if(steps[e].writes[a].Overlaps(steps[l].reads[b]))
                        witnesses.push_back({2,e,a,l,b});
            }
            for(size_t a=0;a<steps[e].reads.size();++a)
                for(size_t b=0;b<steps[l].writes.size();++b)
                    if(!RigExecBakedIsVersionedDomain(steps[l].writes[b].domain) &&
                       steps[e].reads[a].Overlaps(steps[l].writes[b]))
                        witnesses.push_back({3,e,a,l,b});
        }
        std::sort(witnesses.begin(),witnesses.end());
        return witnesses;
    };
    std::array<bool,4> witnessed{};
    for(unsigned variant=0;variant<9;++variant) {
        std::vector<RigExecBakedStep> steps(5);
        steps[0].writes={RigExecBakedRange(D::ChainDirty,0,3),RigExecBakedOne(D::ChainDirty,1)};
        steps[1].reads={RigExecBakedRange(D::ChainDirty,0,2),RigExecBakedOne(D::ChainDirty,7)};
        steps[1].writes={RigExecBakedOne(D::PoseFin,0)};
        steps[2].reads={RigExecBakedOne(D::PoseFin,0)};
        steps[2].writes={RigExecBakedRange(D::ChainDirty,2,4)};
        steps[3].reads={RigExecBakedRange(D::ChainDirty,2,4),RigExecBakedRange(D::ChainDirty,1,1)};
        steps[3].writes={RigExecBakedOne(D::PoseFin,0)};
        steps[4].reads={RigExecBakedOne(D::ChainDirty,9),RigExecBakedOne(D::SpaceLeaf,7),RigExecBakedRange(D::WeightPacket,0,0)};
        steps[4].writes={RigExecBakedOne(D::ChainDirty,7),RigExecBakedRange(D::ChainDirty,1,1)};
        if(variant==8) {
            const uint32_t limit=std::numeric_limits<uint32_t>::max();
            steps[0].writes.push_back(RigExecBakedRange(D::WeightPacket,0,limit));
            steps[1].reads.push_back(RigExecBakedRange(D::WeightPacket,limit-2,limit));
            steps[2].reads.push_back(RigExecBakedRange(D::WeightPacket,0,limit));
            steps[3].writes.push_back(RigExecBakedRange(D::WeightPacket,limit-1,limit));
            steps[4].reads.push_back(RigExecBakedRange(D::WeightPacket,limit,1));
        }
        for(size_t i=1;i<4;++i)if(!(variant&(1u<<(i-1))))steps[i].preds={int(i-1)};
        const Reachability reach(steps);
        const auto expected=exhaustive(steps,reach);
        std::vector<AccessWitness> actual;
        IndexedAccessViolations(steps,reach,[&](const AccessWitness &w){actual.push_back(w);});
        std::sort(actual.begin(),actual.end());
        CHECK(actual==expected);
        for(const auto &w:actual)witnessed[w[0]]=true;
    }
    for(bool seen:witnessed)CHECK(seen);

    // Independently reproduce the former closure scan on a small producer
    // graph, including removed and unrelated added reads and a duplicate route.
    for(unsigned variant=0;variant<4;++variant) {
        const size_t count=70,words=(count+63)/64;
        std::vector<std::vector<RigExecBakedSlotRange>> reads(count),writes(count);
        for(size_t i=0;i<count;++i) {
            writes[i]={RigExecBakedOne(D::PoseFin,int(i))};
            if(i>0)reads[i]={RigExecBakedOne(D::PoseFin,int(i-1))};
        }
        if(variant==1)reads[35].clear();
        if(variant==2)reads[35].push_back(RigExecBakedOne(D::PoseFin,7));
        if(variant==3)reads[35].push_back(reads[35].front());
        std::vector<uint64_t> old(count*words,0);
        for(size_t l=0;l<count;++l)for(size_t e=0;e<l;++e)
            for(const auto &read:reads[l])for(const auto &write:writes[e])
                if(read.Overlaps(write))old[l*words+e/64]|=uint64_t(1)<<(e%64);
        for(size_t l=0;l<count;++l)for(size_t e=0;e<l;++e)
            if((old[l*words+e/64]>>(e%64))&uint64_t(1))
                for(size_t w=0;w<words;++w)old[l*words+w]|=old[e*words+w];
        size_t actualWords=0;
        const auto actual=ConflictClosure(reads,writes,&actualWords);
        CHECK(actualWords==words && actual==old);
        std::vector<uint64_t> changed=actual;
        changed[69*words]^=uint64_t(1)<<7; // missing or spurious reachability
        changed[69*words+1]^=uint64_t(1)<<1;
        size_t scalar=0,packed=0;
        for(size_t l=0;l<count;++l) {
            for(size_t e=0;e<l;++e)
                scalar+=((actual[l*words+e/64]^changed[l*words+e/64])>>(e%64))&uint64_t(1);
            for(size_t w=0;w<(l+63)/64;++w) {
                uint64_t difference=actual[l*words+w]^changed[l*words+w];
                if(w==l/64 && l%64)difference&=(uint64_t(1)<<(l%64))-1;
                packed+=_RigExecPopcount64(difference);
            }
        }
        CHECK(scalar==2 && packed==scalar);
    }

    {
        const uint32_t limit=std::numeric_limits<uint32_t>::max();
        std::vector<std::vector<RigExecBakedSlotRange>> reads(3),writes(3);
        writes[0]={RigExecBakedRange(D::ChainDirty,0,limit)};
        writes[1]={RigExecBakedOne(D::PoseFin,0)};
        reads[1]={RigExecBakedRange(D::ChainDirty,0,limit)};
        reads[2]={RigExecBakedRange(D::ChainDirty,limit-2,limit),RigExecBakedOne(D::PoseFin,0)};
        size_t words=0;
        const auto closure=ConflictClosure(reads,writes,&words);
        CHECK(words==1 && closure==std::vector<uint64_t>({0,1,3}));
    }

    RigExecBakedClustering graph;
    graph.clusters.resize(4);
    graph.clusters[0].succs={1}; graph.clusters[1].preds={0};
    graph.clusters[1].succs={2}; graph.clusters[2].preds={1};
    std::vector<RigExecBakedClusterSet> cones(4);
    for(auto &cone:cones)cone.Resize(4);
    for(int c=0;c<4;++c)cones[size_t(c)].Set(c);
    cones[0].Set(1);cones[0].Set(2);cones[1].Set(2);
    const auto oldClosed=[&](const auto &candidate) {
        bool ok=true;
        for(int c=0;c<4;++c) {
            ok=ok && candidate[size_t(c)].Test(c);
            for(int next:graph.clusters[size_t(c)].succs)ok=ok && candidate[size_t(c)].Test(next);
            for(int d=0;d<4;++d)if(candidate[size_t(c)].Test(d))
                for(int e=0;e<4;++e)if(candidate[size_t(d)].Test(e))
                    ok=ok && candidate[size_t(c)].Test(e);
        }
        return ok;
    };
    const auto exact=[&](const auto &candidate) {
        size_t edges=0,checks=0;
        return ConeRecurrenceViolations(graph,candidate,&edges,&checks)==0;
    };
    CHECK(oldClosed(cones) && exact(cones));
    auto missing=cones; missing[0].words[0]&=~(uint64_t(1)<<2);
    CHECK(!oldClosed(missing) && !exact(missing));
    auto extra=cones; extra[0].Set(3);
    CHECK(oldClosed(extra) && !exact(extra)); // exactness strengthens the old superset check
    auto tail=cones; tail[0].words[0]|=uint64_t(1)<<63;
    CHECK(!exact(tail));
    auto unrelatedMissing=extra; unrelatedMissing[3].words[0]=0;
    CHECK(!oldClosed(unrelatedMissing) && !exact(unrelatedMissing));
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
        size_t differ = 0,reported=0;
        CHECK(words==actual.WordCount());
        for(size_t later=0;later<count;++later) {
            const uint64_t *row=actual.Row(later);
            for(size_t w=0;w<(later+63)/64;++w) {
                uint64_t difference=bits[later*words+w]^row[w];
                if(w==later/64 && later%64)
                    difference&=(uint64_t(1)<<(later%64))-1;
                if(!difference)continue;
                if(reported<5)for(unsigned bit=0;bit<64 && reported<5;++bit) {
                    if(!(difference&(uint64_t(1)<<bit)))continue;
                    const size_t earlier=w*64+bit;
                    const bool expected=(bits[later*words+w]>>bit)&uint64_t(1);
                    std::printf("FAIL %s (%s): step %zu (%s) -> step %zu (%s) is %s\n",
                        name,what,earlier,B.steps[earlier].label.c_str(),later,
                        B.steps[later].label.c_str(),expected?"missing":"new");
                    ++reported;
                }
                differ+=_RigExecPopcount64(difference);
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
                       {"(" + reader.label + ")", "reads point version 2 of chain 0",
                        "without declaring it"});
        reader.reads = reads;
        passes("a chunk without its version read");
    }
    {
        B.revisionFuseStep[1] = chunk;
        ExpectRejected(B, "a fuse table naming a chunk",
                       {"revision 1's fuse is recorded as",
                        "(" + reader.label + ")", "which is not its fuse"});
        B.revisionFuseStep[1] = producer;
        passes("a fuse table naming a chunk");
    }
    {
        reader.writes.push_back(
            RigExecBakedOne(RigExecBakedSlotDomain::ChainDirty, 1));
        ExpectRejected(B, "a second writer of a version",
                       {"(" + reader.label + ")", "writes ChainDirty[1]",
                        "which only", "(" + B.steps[size_t(producer)].label + ")", "may write"});
        reader.writes.pop_back();
        passes("a second writer of a version");
    }
    {
        reader.writes.push_back(
            RigExecBakedOne(RigExecBakedSlotDomain::RevisionDone, 3));
        ExpectRejected(B, "a version written past the revision table",
                       {"(" + reader.label + ")", "writes RevisionDone[3]",
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
                       {"(" + reader.label + ")", "reads RevisionDone[1] without an "
                        "edge from its producer", "(" + B.steps[size_t(producer)].label + ")"});
        preds.insert(std::lower_bound(preds.begin(), preds.end(), producer),
                     producer);
        succs.insert(std::lower_bound(succs.begin(), succs.end(), chunk),
                     chunk);
        passes("a version read without its producer's edge");
    }
}

/// Whether two point arrays hold the same bytes.
template <class A, class B>
bool
SameBits(const A &a, const B &b)
{
    return a.size() == b.size() &&
           (a.size() == 0 ||
            std::memcmp(a.data(), b.data(), a.size() * sizeof(GfVec3f)) == 0);
}

/// Whether the step of \p kind for \p object ran in the program's last run.
bool
RanLast(const RigExecBakedProgramImpl &B, RigExecBakedStepKind kind,
        int object)
{
    for (size_t c = 0; c < B.opGraph.ops.size(); ++c) {
        const RigExecBakedStep &step = B.steps[B.opGraph.ops[c].originalIndex];
        if (step.kind == kind && step.object == object) {
            return c < B.opExecution.ran.size() && B.opExecution.ran[c];
        }
    }
    return false;
}

/// The fuse's buffer selection, stepped by hand so that each run's chunk and
/// fuse are the test's to choose. An applying fuse swaps the chunk's buffer
/// in; one that re-applies without its chunk keeps the buffer it published;
/// one that passes through publishes the entering points, keeps a copy of
/// them and leaves the revision's last applied points where a later apply
/// finds them. The value keys carry a content version, and at every
/// publication they must tell the change the keys over the bytes tell.
void
TestTheFuseSelectsItsBuffersByHand()
{
    using D = RigExecBakedSlotDomain;
    const VtVec3fArray base{GfVec3f(0.0f), GfVec3f(1.0f, 0.0f, 0.0f),
                            GfVec3f(0.0f, 2.0f, 0.0f)};
    RigExecBakedProgramImpl B;
    RigExecResolvedInputs resolved;
    B.resolvedInputs = &resolved;
    B.chains.resize(1);
    RigExecBakedProgramImpl::GeomChain &chain = B.chains[0];
    chain.haveBase = true;
    chain.lastBase = base;
    chain.revisions.resize(1);
    B.revisionIndex = {{0, 0}};
    RigExecBakedProgramImpl::GeomRevision &revision = chain.revisions[0];
    revision.op = RigExecRevisionOp::Matrix;
    revision.moverPath = SdfPath("/M");
    revision.moverPathText = "/M";
    revision.chunks.resize(1);
    revision.precedingCount = base.size();
    revision.stagingOutput.resize(base.size());
    const auto setPacket = [&](double tx, bool valid, float strength) {
        RigExecMoverParameters p;
        p.kind = RigExecRevisionKindToken(RigExecRevisionOp::Matrix);
        p.enabled = true;
        p.valid = valid;
        p.strength = strength;
        p.weights = RigExecWeightPacket::Constant(1.0f);
        GfMatrix4d m(1.0);
        m.SetTranslateOnly(GfVec3d(tx, 0.0, 0.0));
        p.transform = m;
        revision.parameters = p;
        revision.status =
            RigExecStatusForParameters(p, revision.moverPathText);
        // RevisionStatic's decision, which the fuse selects by.
        revision.acceptance = RigExecRevisionKernelAcceptance(
            revision.op, p, base.size());
    };
    RigExecBakedStep chunk, fuse, status;
    chunk.kind = RigExecBakedStepKind::RevisionChunk;
    chunk.object = 0;
    chunk.part = 0;
    fuse.kind = RigExecBakedStepKind::RevisionFuse;
    fuse.object = 0;
    status.kind = RigExecBakedStepKind::ChainStatus;
    status.object = 0;
    const auto run = [&](RigExecBakedStep *step) {
        RigExecBakedRunGeometryStep(&B, step, UsdTimeCode::Default());
    };
    struct Watched {
        D domain;
        std::string key, content;
    };
    std::vector<Watched> watched{{D::RevisionDone, {}, {}},
                                 {D::ChainDirty, {}, {}},
                                 {D::ChainPoints, {}, {}}};
    const auto publish = [&](const char *what) {
        for (Watched &w : watched) {
            std::string key, content;
            RigExecBakedOpValueKey(B, w.domain, 0, &key);
            CHECK(RigExecBakedChainContentKey(B, w.domain, 0, &content));
            if (!w.key.empty() &&
                (key != w.key) != (content != w.content)) {
                ++failures;
                std::printf("FAIL %s: a content version and its points' "
                            "bytes disagree on a change\n", what);
            }
            w.key.swap(key);
            w.content.swap(content);
        }
    };
    const auto translated = [&](float tx) {
        VtVec3fArray moved = base;
        for (GfVec3f &point : moved) {
            point[0] += tx;
        }
        return moved;
    };

    setPacket(1.0, true, 0.0f);
    run(&chunk); run(&fuse); run(&status); publish("applies");
    CHECK(revision.currentSource == 0 && !revision.stagingFresh);
    CHECK(SameBits(chain.result, translated(1.0f)));
    const uint64_t first = revision.doneVersion;

    // Re-applies without its chunk: the published buffer stays.
    run(&fuse); run(&status); publish("re-applies");
    CHECK(revision.doneVersion == first);
    CHECK(SameBits(revision.output, translated(1.0f)));

    // Passes through: the base is published and a copy of it kept, and the
    // last applied points stay the revision's own.
    setPacket(1.0, false, 0.0f);
    run(&chunk); run(&fuse); run(&status); publish("passes through");
    CHECK(revision.currentSource == -1 && revision.doneVersion == first + 1);
    CHECK(SameBits(chain.result, base));
    CHECK(SameBits(revision.passedPoints, base));
    CHECK(SameBits(revision.output, translated(1.0f)));

    // Applies again: compared with the copy, not with its old points.
    setPacket(1.0, true, 0.0f);
    run(&chunk); run(&fuse); run(&status); publish("recovers");
    CHECK(revision.currentSource == 0 && revision.doneVersion == first + 2);
    CHECK(SameBits(chain.result, translated(1.0f)));

    // A packet that moves without moving the points: the buffers swap and
    // the version stays.
    setPacket(1.0, true, 0.5f);
    run(&chunk); run(&fuse); run(&status); publish("same points");
    CHECK(revision.doneVersion == first + 2 && !revision.stagingFresh);
    CHECK(SameBits(chain.result, translated(1.0f)));

    setPacket(2.0, true, 0.5f);
    run(&chunk); run(&fuse); run(&status); publish("moves");
    CHECK(revision.doneVersion == first + 3);
    CHECK(SameBits(chain.result, translated(2.0f)));

    // The entering points move under a pass-through: the copy follows them.
    setPacket(2.0, false, 0.5f);
    run(&chunk); run(&fuse); run(&status); publish("passes through again");
    CHECK(revision.doneVersion == first + 4);
    VtVec3fArray entering = base;
    entering[2][1] = 5.0f;
    chain.lastBase = entering;
    run(&fuse); run(&status); publish("entering points moved");
    CHECK(revision.doneVersion == first + 5);
    CHECK(SameBits(revision.passedPoints, entering));
    CHECK(SameBits(chain.result, entering));
    CHECK(SameBits(revision.output, translated(2.0f)));
}

/// The verifier compares staging only while it holds a value: the chunk's
/// result before a fuse applied it, or a chunked skin's ranges. Once an
/// applying fuse swapped it out it holds whatever the buffer last did, which
/// a forced run and a cone run legitimately leave different -- but its size
/// is the RevisionOut key's, and the versions and the pass-through copy are
/// compared like every other value.
void
TestTheVerifierComparesOnlyLiveStaging()
{
    RigExecBakedProgramImpl B;
    B.chains.resize(1);
    B.chains[0].revisions.resize(1);
    RigExecBakedProgramImpl::GeomRevision &revision = B.chains[0].revisions[0];
    revision.output = {GfVec3f(1, 2, 3)};
    revision.stagingOutput = {GfVec3f(4, 5, 6)};
    RigExecBakedRunShadow cone;
    cone.Capture(B);
    std::vector<std::string> differences;
    CHECK(cone.Compare(B, &differences) == 0);
    revision.stagingOutput[0][0] = 9.0f;
    CHECK(cone.Compare(B, &differences) == 0);
    revision.stagingOutput.push_back(GfVec3f(0.0f));
    CHECK(cone.Compare(B, &differences) != 0);
    revision.stagingOutput.pop_back();
    revision.stagingFresh = true;
    CHECK(cone.Compare(B, &differences) != 0);
    cone.Capture(B);
    revision.stagingOutput[0][0] = 4.0f;
    CHECK(cone.Compare(B, &differences) != 0);
    revision.stagingOutput[0][0] = 9.0f;
    CHECK(cone.Compare(B, &differences) == 0);
    ++revision.doneVersion;
    CHECK(cone.Compare(B, &differences) != 0);
    --revision.doneVersion;
    revision.passedPoints = {GfVec3f(1.0f)};
    CHECK(cone.Compare(B, &differences) != 0);
    revision.passedPoints.clear();
    ++B.chains[0].resultVersion;
    CHECK(cone.Compare(B, &differences) != 0);
    --B.chains[0].resultVersion;
    ++B.chains[0].baseVersion;
    CHECK(cone.Compare(B, &differences) != 0);
    --B.chains[0].baseVersion;
    CHECK(cone.Compare(B, &differences) == 0);
}

/// After an adoption, a chunk the rebuild did not retain runs and leaves its
/// result fresh in staging while its retained fuse is skipped; the forced
/// pass runs that fuse, which swaps staging out. Staging's freshness and
/// contents are compared only where the cone ran the fuse too; its size
/// always.
void
TestTheVerifierLeavesStagingToASkippedFuse()
{
    RigExecBakedProgramImpl B;
    B.chains.resize(1);
    B.chains[0].revisions.resize(1);
    B.steps.resize(2);
    B.steps[0].kind = RigExecBakedStepKind::RevisionChunk;
    B.steps[1].kind = RigExecBakedStepKind::RevisionFuse;
    for (auto &step : B.steps) {
        step.object = 0;
        step.label = "staging fixture";
    }
    B.revisionIndex.emplace_back(0, 0);
    B.opGraph.ops.resize(2);
    B.opGraph.ops[0].originalIndex = 0;
    B.opGraph.ops[1].originalIndex = 1;
    RigExecBakedProgramImpl::GeomRevision &revision = B.chains[0].revisions[0];
    // The cone: the chunk ran, its result fresh; the fuse was skipped.
    B.opExecution.ran = {1, 0};
    revision.output = {GfVec3f(1, 2, 3)};
    revision.stagingOutput = {GfVec3f(1, 2, 3)};
    revision.stagingFresh = true;
    RigExecBakedRunShadow cone;
    cone.Capture(B);
    // The forced pass: the fuse applied by a swap, leaving staging the
    // buffer `output` was, stale, and no longer fresh.
    B.opExecution.ran = {1, 1};
    revision.output.swap(revision.stagingOutput);
    revision.stagingOutput[0] = GfVec3f(7, 8, 9);
    revision.stagingFresh = false;
    std::vector<std::string> differences;
    CHECK(cone.Compare(B, &differences) == 0);
    revision.stagingOutput.push_back(GfVec3f(0.0f));
    CHECK(cone.Compare(B, &differences) != 0);
    revision.stagingOutput.pop_back();
    // A cone that ran the fuse too is held to both.
    B.opExecution.ran = {1, 1};
    revision.stagingFresh = true;
    cone.Capture(B);
    revision.stagingFresh = false;
    CHECK(cone.Compare(B, &differences) != 0);
    revision.stagingFresh = true;
    revision.stagingOutput[0] = GfVec3f(4, 5, 6);
    CHECK(cone.Compare(B, &differences) != 0);
    revision.stagingOutput[0] = GfVec3f(7, 8, 9);
    differences.clear();
    CHECK(cone.Compare(B, &differences) == 0);
}

/// The shadow puts back the chain input's content version and the base it
/// last published, as it does the other versions, so the forced pass
/// decides that version from where the cone run started.
void
TestTheVerifierRestoresTheChainInputVersion()
{
    RigExecBakedProgramImpl B;
    B.chains.resize(1);
    RigExecBakedProgramImpl::GeomChain &chain = B.chains[0];
    const VtVec3fArray published{GfVec3f(1, 2, 3)};
    chain.sampledBase = published;
    chain.publishedInput = published;
    chain.inputVersion = 4;
    RigExecBakedRunShadow before;
    before.Capture(B);
    // What a publication of a moved base does.
    chain.sampledBase = VtVec3fArray{GfVec3f(4, 5, 6)};
    chain.publishedInput = chain.sampledBase;
    ++chain.inputVersion;
    before.Restore(&B);
    CHECK(chain.inputVersion == 4);
    CHECK(chain.publishedInput == published);
}

/// A chain revision that passes through and then applies again publishes
/// what a program that never saw the failure publishes, and a rebuild
/// carries the buffers, content versions and baselines it holds, so that
/// republishing the same frame moves none of them. Under
/// RIGEXEC_VERIFY_CHAIN_VERSIONS every published point version is checked
/// against its points' bytes.
void
TestAChainRevisionRecoversAcrossAFailure()
{
    TfSetenv("RIGEXEC_VERIFY_CHAIN_VERSIONS", "1");
    const auto finish = [] { TfSetenv("RIGEXEC_VERIFY_CHAIN_VERSIONS", "0"); };
    const BuiltProgram built = BuildStage(MakeAnimatedChainStage(1.0f, true));
    const BuiltProgram fresh = BuildStage(MakeAnimatedChainStage(1.0f, true));
    CHECK(built.program != nullptr && fresh.program != nullptr);
    if (!built.program || !fresh.program) {
        finish();
        return;
    }
    const RigExecBakedProgramImpl &B = built.program->GetStepGraph();
    CHECK(B.verifyChainVersions);
    CHECK(B.chains.size() == 1 && B.chains[0].revisions.size() == 3);
    if (B.chains.size() != 1 || B.chains[0].revisions.size() != 3) {
        finish();
        return;
    }
    const RigExecBakedProgramImpl::GeomChain &chain = B.chains[0];
    const RigExecBakedProgramImpl::GeomRevision &middle = chain.revisions[1];
    RigExecRigPose pose;
    CHECK(built.program->Run(UsdTimeCode(1.0), &pose));
    CHECK(pose.comparisonMismatches == 0);
    CHECK(middle.currentSource == 1);
    const uint64_t applied = middle.doneVersion;
    CHECK(built.program->Run(UsdTimeCode(2.0), &pose));
    CHECK(pose.comparisonMismatches == 0);
    CHECK(middle.resultStatus == TfToken("moverFailed"));
    CHECK(middle.currentSource == 0);
    CHECK(middle.doneVersion == applied + 1);
    CHECK(SameBits(middle.passedPoints, chain.revisions[0].output));
    CHECK(!middle.output.empty());
    CHECK(built.program->Run(UsdTimeCode(3.0), &pose));
    CHECK(pose.comparisonMismatches == 0);
    CHECK(middle.currentSource == 1 && middle.doneVersion == applied + 2);

    // The same frame, never having failed.
    RigExecRigPose freshPose;
    CHECK(fresh.program->Run(UsdTimeCode(3.0), &freshPose));
    const RigExecBakedProgramImpl &F = fresh.program->GetStepGraph();
    CHECK(SameBits(chain.result, F.chains[0].result));
    for (size_t r = 0; r < 3; ++r) {
        CHECK(chain.revisions[r].currentSource ==
              F.chains[0].revisions[r].currentSource);
        CHECK(SameBits(chain.revisions[r].output,
                       F.chains[0].revisions[r].output));
    }
    CHECK(B.chainVersionMismatches == 0 && F.chainVersionMismatches == 0);

    // A rebuild after the failure carries what the next publication is
    // decided against: the outgoing program's numbers, read before the
    // adoption moves them.
    const BuiltProgram outgoing = BuildStage(MakeAnimatedChainStage(1.0f, true));
    CHECK(outgoing.program != nullptr);
    if (!outgoing.program) {
        finish();
        return;
    }
    CHECK(outgoing.program->Run(UsdTimeCode(1.0), &pose));
    CHECK(outgoing.program->Run(UsdTimeCode(2.0), &pose));
    const RigExecBakedProgramImpl &O = outgoing.program->GetStepGraph();
    std::vector<uint64_t> versions;
    std::vector<std::vector<GfVec3f>> outputs, passed;
    for (const auto &revision : O.chains[0].revisions) {
        versions.push_back(revision.doneVersion);
        outputs.push_back(revision.output);
        passed.push_back(revision.passedPoints);
    }
    const uint64_t baseVersion = O.chains[0].baseVersion;
    const uint64_t resultVersion = O.chains[0].resultVersion;
    std::vector<std::string> reasons;
    std::unique_ptr<RigExecBakedProgram> rebuilt =
        RigExecBakedProgram::Build(outgoing.evaluator.get(), &reasons);
    CHECK(rebuilt != nullptr);
    if (!rebuilt) {
        finish();
        return;
    }
    rebuilt->AdoptGeometryStateFrom(*outgoing.program);
    const RigExecBakedProgramImpl &R = rebuilt->GetStepGraph();
    CHECK(R.chains.size() == 1 && R.chains[0].revisions.size() == 3);
    if (R.chains.size() != 1 || R.chains[0].revisions.size() != 3) {
        finish();
        return;
    }
    const auto carried = [&](const char *when) {
        for (size_t r = 0; r < 3; ++r) {
            const auto &revision = R.chains[0].revisions[r];
            if (revision.doneVersion != versions[r] ||
                !SameBits(revision.output, outputs[r]) ||
                !SameBits(revision.passedPoints, passed[r])) {
                ++failures;
                std::printf("FAIL %s: revision %zu lost its published "
                            "points' version or baseline\n", when, r);
            }
        }
        CHECK(R.chains[0].baseVersion == baseVersion);
        CHECK(R.chains[0].resultVersion == resultVersion);
    };
    carried("adopted");
    CHECK(rebuilt->Run(UsdTimeCode(2.0), &pose));
    CHECK(pose.comparisonMismatches == 0);
    carried("republished");
    CHECK(rebuilt->Run(UsdTimeCode(3.0), &pose));
    CHECK(SameBits(R.chains[0].result, F.chains[0].result));
    CHECK(R.chainVersionMismatches == 0 && O.chainVersionMismatches == 0);
    finish();
}

/// An edit that leaves a revision's points byte for byte where they were
/// keeps their content version, so nothing that reads them runs again: M0
/// has a zero weight, so its driver moves its packet and never its points.
void
TestUnmovedPointsKeepTheirVersion()
{
    TfSetenv("RIGEXEC_VERIFY_CHAIN_VERSIONS", "1");
    const BuiltProgram built = BuildStage(MakeAnimatedChainStage(0.0f, false));
    TfSetenv("RIGEXEC_VERIFY_CHAIN_VERSIONS", "0");
    CHECK(built.program != nullptr);
    if (!built.program) {
        return;
    }
    const RigExecBakedProgramImpl &B = built.program->GetStepGraph();
    CHECK(B.chains.size() == 1 && B.chains[0].revisions.size() == 3);
    if (B.chains.size() != 1 || B.chains[0].revisions.size() != 3) {
        return;
    }
    const RigExecBakedProgramImpl::GeomChain &chain = B.chains[0];
    // The chain places the three movers itself, so M0 is found by its path.
    int m0 = -1;
    for (size_t o = 0; o < B.revisionIndex.size(); ++o) {
        const auto &[c, r] = B.revisionIndex[o];
        if (c == 0 && chain.revisions[size_t(r)].moverPath ==
                          SdfPath("/Asset/Rig/Movers/M0")) {
            m0 = int(o);
        }
    }
    CHECK(m0 >= 0);
    if (m0 < 0) {
        return;
    }
    const RigExecBakedProgramImpl::GeomRevision &zero =
        chain.revisions[size_t(B.revisionIndex[size_t(m0)].second)];
    RigExecRigPose pose;
    CHECK(built.program->Run(UsdTimeCode(1.0), &pose));
    const uint64_t first = zero.doneVersion;
    const uint64_t points = chain.resultVersion;
    const VtVec3fArray result = chain.result;
    CHECK(built.program->Run(UsdTimeCode(2.0), &pose));
    CHECK(pose.comparisonMismatches == 0);
    using K = RigExecBakedStepKind;
    // The packet moved, so M0 ran, and its points did not.
    CHECK(RanLast(B, K::RevisionChunk, m0) && RanLast(B, K::RevisionFuse, m0));
    CHECK(zero.doneVersion == first);
    for (int r = 0; r < 3; ++r) {
        CHECK(r == m0 ||
              (!RanLast(B, K::RevisionChunk, r) && !RanLast(B, K::RevisionFuse, r)));
    }
    CHECK(!RanLast(B, K::ChainStatus, 0));
    CHECK(chain.resultVersion == points && SameBits(chain.result, result));
    CHECK(B.chainVersionMismatches == 0);
}

/// Two chains of matrix movers, A0..A2 on /Asset/ShapeA and B0, B1 on
/// /Asset/ShapeB. A1, A2 and B1 carry an out-of-range inputs:defaultWeight
/// at frame 2 only, so three revisions in two chains pass through there and
/// apply again at frames 1 and 3.
UsdStageRefPtr
MakeFailingChainsStage()
{
    const UsdStageRefPtr stage = UsdStage::CreateInMemory();
    stage->DefinePrim(SdfPath("/Asset"), TfToken("Scope"));
    stage->DefinePrim(SdfPath("/Asset/Rig"), TfToken("RigExecRoot"));
    const UsdPrim moving = stage->DefinePrim(
        SdfPath("/Asset/Rig/Moving"), TfToken("RigExecControl"));
    for (int frame = 1; frame <= 3; ++frame) {
        moving.GetAttribute(TfToken("avars:tx"))
            .Set(double(frame), UsdTimeCode(frame));
    }
    const UsdPrim still = stage->DefinePrim(
        SdfPath("/Asset/Rig/Still"), TfToken("RigExecControl"));
    still.GetAttribute(TfToken("avars:ty")).Set(1.0);
    stage->DefinePrim(SdfPath("/Asset/Rig/Movers"), TfToken("Scope"));
    const auto chain = [&](const char *shape, const char *prefix, int count,
                           const std::set<int> &failing) {
        const SdfPath target(std::string("/Asset/") + shape + ".points");
        const UsdPrim points =
            stage->DefinePrim(target.GetPrimPath(), TfToken("Points"));
        points.GetAttribute(TfToken("points"))
            .Set(VtVec3fArray{GfVec3f(0), GfVec3f(1, 0, 0), GfVec3f(0, 2, 0)});
        for (int i = 0; i < count; ++i) {
            const UsdPrim mover = stage->DefinePrim(
                SdfPath(std::string("/Asset/Rig/Movers/") + prefix +
                        std::to_string(i)),
                TfToken("RigExecMatrixMover"));
            mover.ApplyAPI(TfToken("RigExecMoverAPI"));
            mover.GetRelationship(TfToken("rigExec:moves"))
                .SetTargets({target});
            mover.GetRelationship(TfToken("rigExec:transform"))
                .SetTargets({i == 0 ? moving.GetPath() : still.GetPath()});
            UsdAttribute weight =
                mover.GetAttribute(TfToken("inputs:defaultWeight"));
            if (failing.count(i)) {
                weight.Set(1.0f, UsdTimeCode(1.0));
                weight.Set(2.0f, UsdTimeCode(2.0));
                weight.Set(1.0f, UsdTimeCode(3.0));
            } else {
                weight.Set(1.0f);
            }
        }
    };
    chain("ShapeA", "A", 3, {1, 2});
    chain("ShapeB", "B", 2, {1});
    return stage;
}

/// The failing movers of MakeFailingChainsStage.
const char *const kFailingMovers[] = {"/Asset/Rig/Movers/A1:",
                                      "/Asset/Rig/Movers/A2:",
                                      "/Asset/Rig/Movers/B1:"};

/// The lines of \p diagnostics that report a failed mover, in order.
std::vector<std::string>
MoverFailedLines(const std::vector<std::string> &diagnostics)
{
    std::vector<std::string> lines;
    for (const std::string &line : diagnostics) {
        if (line.rfind("MoverFailed ", 0) == 0) {
            lines.push_back(line);
        }
    }
    return lines;
}

/// Whether some line of \p lines names the mover \p prefix ("path:").
bool
NamesMover(const std::vector<std::string> &lines, const char *prefix)
{
    for (const std::string &line : lines) {
        if (line.find(prefix) != std::string::npos) {
            return true;
        }
    }
    return false;
}

/// The epilogue's held set names exactly the steps holding diagnostics or
/// lines, once each and in step order (a publication has tidied it).
bool
HeldStepsAreExact(const RigExecBakedProgramImpl &B)
{
    const RigExecHeldSteps &held = B.epilogue.held;
    if (held.holding.size() != B.steps.size()) {
        return false;
    }
    std::vector<uint32_t> holding;
    for (uint32_t i = 0; i < B.steps.size(); ++i) {
        const bool holds = !B.steps[i].diagnostics.empty() ||
                           !B.steps[i].lines.empty();
        if ((held.holding[i] != 0) != holds) {
            return false;
        }
        if (holds) {
            holding.push_back(i);
        }
    }
    return held.ascending && held.held == holding;
}

/// What the geometry epilogue published while it swept every step: each
/// geometry step's diagnostics, in step order.
std::vector<std::string>
SweptGeometryLines(const RigExecBakedProgramImpl &B)
{
    std::vector<std::string> lines;
    for (const RigExecBakedStep &step : B.steps) {
        if (RigExecBakedIsGeometryStep(step.kind)) {
            lines.insert(lines.end(), step.diagnostics.begin(),
                         step.diagnostics.end());
        }
    }
    return lines;
}

/// One run's epilogue against the steps it read: the held set is exact,
/// the geometry lines stand in the pose in step order, and the failing
/// movers are named exactly when \p failing. Returns the failure lines.
std::vector<std::string>
CheckEpilogueRun(const char *what, const RigExecBakedProgramImpl &B,
                 const RigExecRigPose &pose, bool failing)
{
    const int before = failures;
    CHECK(HeldStepsAreExact(B));
    CHECK(B.epilogue.verify && B.epilogue.mismatches == 0);
    const std::vector<std::string> swept = SweptGeometryLines(B);
    CHECK(swept.empty() ||
          std::search(pose.diagnostics.begin(), pose.diagnostics.end(),
                      swept.begin(), swept.end()) !=
              pose.diagnostics.end());
    const std::vector<std::string> lines = MoverFailedLines(pose.diagnostics);
    for (const char *mover : kFailingMovers) {
        CHECK(NamesMover(lines, mover) == failing);
        CHECK(NamesMover(swept, mover) == failing);
    }
    CHECK(!NamesMover(lines, "/Asset/Rig/Movers/A0:") &&
          !NamesMover(lines, "/Asset/Rig/Movers/B0:"));
    if (failures != before) {
        std::printf("    (%s)\n", what);
    }
    return lines;
}

/// The epilogue publishes from the steps holding output, kept by the owner
/// after each run, and RIGEXEC_VERIFY_EPILOGUE_LISTS sweeps every step
/// beside it. Three revisions in two chains fail at frame 2: their lines
/// appear, survive a run that skips their steps, clear when the revisions
/// apply again and come back, in step order. A rebuild that adopts the
/// outgoing program's state publishes them the same way, and so does a
/// frozen job against a snapshot taken on either side of the failure.
void
TestTheEpilogueVisitsTheStepsHoldingLines()
{
    TfSetenv("RIGEXEC_VERIFY_EPILOGUE_LISTS", "1");
    const auto finish = [] { TfSetenv("RIGEXEC_VERIFY_EPILOGUE_LISTS", "0"); };
    const BuiltProgram built = BuildStage(MakeFailingChainsStage());
    CHECK(built.program != nullptr);
    if (!built.program) {
        finish();
        return;
    }
    const RigExecBakedProgramImpl &B = built.program->GetStepGraph();
    CHECK(B.chains.size() == 2);
    std::vector<std::string> failedAtTwo;
    // Frame 2 twice: the second run skips the failed steps, which keep
    // their lines.
    for (const double frame : {1.0, 2.0, 2.0, 3.0, 2.0, 1.0}) {
        RigExecRigPose pose;
        CHECK(built.program->Run(UsdTimeCode(frame), &pose));
        const std::string what =
            "native frame " + std::to_string(int(frame));
        const std::vector<std::string> lines =
            CheckEpilogueRun(what.c_str(), B, pose, frame == 2.0);
        if (frame == 2.0) {
            if (failedAtTwo.empty()) {
                failedAtTwo = lines;
            }
            CHECK(lines == failedAtTwo);
        }
    }
    CHECK(!failedAtTwo.empty());

    // A rebuild that adopts the failed frame's state.
    {
        RigExecRigPose pose;
        CHECK(built.program->Run(UsdTimeCode(2.0), &pose));
        std::vector<std::string> reasons;
        std::unique_ptr<RigExecBakedProgram> rebuilt =
            RigExecBakedProgram::Build(built.evaluator.get(), &reasons);
        CHECK(rebuilt != nullptr);
        if (rebuilt) {
            rebuilt->AdoptGeometryStateFrom(*built.program);
            const RigExecBakedProgramImpl &R = rebuilt->GetStepGraph();
            CHECK(HeldStepsAreExact(R));
            for (const double frame : {2.0, 3.0, 2.0}) {
                RigExecRigPose next;
                CHECK(rebuilt->Run(UsdTimeCode(frame), &next));
                const std::string what =
                    "rebuilt frame " + std::to_string(int(frame));
                const std::vector<std::string> lines =
                    CheckEpilogueRun(what.c_str(), R, next, frame == 2.0);
                if (frame == 2.0) {
                    CHECK(lines == failedAtTwo);
                }
            }
        }
    }

    // Frozen jobs, against a snapshot of the failed frame and of the
    // recovered one: each publishes live's failure lines at its time.
    {
        const UsdStageRefPtr stage = MakeFailingChainsStage();
        RigExecRigEvaluator evaluator(stage, SdfPath("/Asset/Rig"));
        std::vector<std::string> errors;
        CHECK(evaluator.Compile(&errors));
        CHECK(evaluator.IsBakeable());
        const std::vector<RigExecValueOverride> none;
        const auto job = [&](double time) {
            std::shared_ptr<const RigExecFrozenProgram> frozen;
            std::string error;
            CHECK(RigExecFreezeProgram(evaluator, &frozen, &error));
            RigExecFrameInputs inputs;
            CHECK(RigExecSampleFrameInputs(evaluator, UsdTimeCode(time), none,
                                           &inputs, &error));
            RigExecRigPose pose;
            const RigExecBakedProgram *program = evaluator.GetBakedProgram();
            if (!frozen || !program) {
                std::printf("FAIL epilogue lists: the freeze refused: %s\n",
                            error.c_str());
                ++failures;
                return pose;
            }
            RigExecFrozenEvalContext context;
            context.epochDigest = evaluator.GetBindingEpochDigest();
            context.slotCount = program->GetProviderCount();
            context.varyingInputCount = inputs.values.size();
            context.flags = 0;
            if (evaluator.GetPublishWeightFields()) {
                context.flags |= kRigExecFrozenPublishWeightFields;
            }
            if (evaluator.GetSolverGuidesEnabled()) {
                context.flags |= kRigExecFrozenSolverGuidesEnabled;
            }
            context.frozen = frozen.get();
            pose = RigExecEvaluateFrozen(context, inputs,
                                         RigExecMakeProductionStepRunner());
            CHECK(pose.valid);
            return pose;
        };
        std::vector<std::string> liveAtTwo;
        for (const double held : {2.0, 3.0}) {
            const RigExecRigPose live = evaluator.Evaluate(UsdTimeCode(held));
            CHECK(live.valid);
            const std::vector<std::string> liveLines =
                MoverFailedLines(live.diagnostics);
            CHECK(liveLines.empty() == (held != 2.0));
            if (held == 2.0) {
                liveAtTwo = liveLines;
            }
            // The snapshot is the live program at `held`; a job at the
            // same time skips the failed steps, one at the other time
            // re-runs them.
            for (const double time : {held, held == 2.0 ? 3.0 : 2.0}) {
                const std::vector<std::string> lines =
                    MoverFailedLines(job(time).diagnostics);
                if (time == held) {
                    CHECK(lines == liveLines);
                } else {
                    CHECK(lines.empty() == (time != 2.0));
                }
                for (const char *mover : kFailingMovers) {
                    CHECK(NamesMover(lines, mover) == (time == 2.0));
                }
            }
        }
        // Live at frame 2 again, after a recovered frame.
        CHECK(MoverFailedLines(
                  evaluator.Evaluate(UsdTimeCode(2.0)).diagnostics) ==
              liveAtTwo);
        CHECK(evaluator.GetBakedProgram() != nullptr &&
              evaluator.GetBakedProgram()->GetStepGraph().epilogue.mismatches ==
                  0);
    }
    finish();
}

/// MakeAnimatedChainStage's three matrix movers over \p points points, which
/// the default vertex target (4096) cuts into ceil(points / 4096) ranges, so
/// every revision is range-pipelined. M0 rides a driver that moves at frames
/// 1-3, M1 and M2 one that stands still; \p failMiddle gives M1 an
/// out-of-range weight at frame 2 only. With \p moveOnlyRangeZero M0 is
/// weighted by a sparse static weight naming points [0, 100) only, so frames
/// 1-3 differ only there: a base move would rerun every range, which reads
/// ChainBase. The points are time samples at frames 1-3 holding one array,
/// and a default value too unless \p samplesOnly, which leaves Build's count
/// to the earliest sample.
UsdStageRefPtr
MakeRangeChainStage(size_t points, bool moveOnlyRangeZero, bool failMiddle,
                    bool samplesOnly = false)
{
    const UsdStageRefPtr stage = UsdStage::CreateInMemory();
    stage->DefinePrim(SdfPath("/Asset"), TfToken("Scope"));
    stage->DefinePrim(SdfPath("/Asset/Rig"), TfToken("RigExecRoot"));
    const UsdPrim moving = stage->DefinePrim(
        SdfPath("/Asset/Rig/Moving"), TfToken("RigExecControl"));
    for (int frame = 1; frame <= 3; ++frame) {
        moving.GetAttribute(TfToken("avars:tx"))
            .Set(double(frame), UsdTimeCode(frame));
    }
    const UsdPrim still = stage->DefinePrim(
        SdfPath("/Asset/Rig/Still"), TfToken("RigExecControl"));
    still.GetAttribute(TfToken("avars:ty")).Set(1.0);
    const SdfPath target("/Asset/Shape.points");
    const UsdPrim shape =
        stage->DefinePrim(target.GetPrimPath(), TfToken("Points"));
    VtVec3fArray base(points);
    for (size_t i = 0; i < points; ++i) {
        base[i] = GfVec3f(float(i % 101) * 0.25f, float(i / 101) * 0.125f,
                          1.0f + float(i % 7));
    }
    UsdAttribute authored = shape.GetAttribute(TfToken("points"));
    if (!samplesOnly) {
        authored.Set(base);
    }
    for (int frame = 1; frame <= 3; ++frame) {
        authored.Set(base, UsdTimeCode(frame));
    }
    UsdPrim weight;
    if (moveOnlyRangeZero) {
        VtIntArray named(100);
        for (int i = 0; i < 100; ++i) {
            named[size_t(i)] = i;
        }
        weight = stage->DefinePrim(SdfPath("/Asset/Rig/Weights/RangeZero"),
                                   TfToken("RigExecStaticWeight"));
        weight.CreateRelationship(TfToken("rigExec:weightTarget"), false)
            .SetTargets({target});
        weight.CreateAttribute(TfToken("rigExec:representation"),
                               SdfValueTypeNames->Token, false)
            .Set(TfToken("sparse"));
        weight.CreateAttribute(TfToken("rigExec:indices"),
                               SdfValueTypeNames->IntArray, false)
            .Set(named);
        weight.CreateAttribute(TfToken("rigExec:values"),
                               SdfValueTypeNames->FloatArray, false)
            .Set(VtFloatArray(named.size(), 1.0f));
        weight.CreateAttribute(TfToken("rigExec:defaultWeight"),
                               SdfValueTypeNames->Float, false)
            .Set(0.0f);
    }
    stage->DefinePrim(SdfPath("/Asset/Rig/Movers"), TfToken("Scope"));
    for (int i = 0; i < 3; ++i) {
        const UsdPrim mover = stage->DefinePrim(
            SdfPath("/Asset/Rig/Movers/M" + std::to_string(i)),
            TfToken("RigExecMatrixMover"));
        mover.ApplyAPI(TfToken("RigExecMoverAPI"));
        mover.GetRelationship(TfToken("rigExec:moves")).SetTargets({target});
        mover.GetRelationship(TfToken("rigExec:transform"))
            .SetTargets({i == 0 ? moving.GetPath() : still.GetPath()});
        UsdAttribute scalar =
            mover.GetAttribute(TfToken("inputs:defaultWeight"));
        if (i == 0 && weight) {
            mover.CreateRelationship(TfToken("rigExec:weightObject"), false)
                .SetTargets({weight.GetPath()});
        } else if (i == 1 && failMiddle) {
            scalar.Set(1.0f, UsdTimeCode(1.0));
            scalar.Set(2.0f, UsdTimeCode(2.0));
            scalar.Set(1.0f, UsdTimeCode(3.0));
        } else {
            scalar.Set(1.0f);
        }
    }
    return stage;
}

/// Whether \p ranges declare slot \p slot of \p domain.
bool
DeclaresSlot(const std::vector<RigExecBakedSlotRange> &ranges,
             RigExecBakedSlotDomain domain, int slot)
{
    for (const RigExecBakedSlotRange &range : ranges) {
        if (range.domain == domain && slot >= 0 &&
            range.begin <= uint32_t(slot) && uint32_t(slot) < range.end) {
            return true;
        }
    }
    return false;
}

/// How many slots of \p domain \p ranges declare.
size_t
DeclaredSlots(const std::vector<RigExecBakedSlotRange> &ranges,
              RigExecBakedSlotDomain domain)
{
    size_t count = 0;
    for (const RigExecBakedSlotRange &range : ranges) {
        if (range.domain == domain && range.end > range.begin) {
            count += size_t(range.end - range.begin);
        }
    }
    return count;
}

/// Whether the step of \p kind for \p object and \p part ran last run.
bool
RanLastPart(const RigExecBakedProgramImpl &B, RigExecBakedStepKind kind,
            int object, int part)
{
    for (size_t c = 0; c < B.opGraph.ops.size(); ++c) {
        const RigExecBakedStep &step = B.steps[B.opGraph.ops[c].originalIndex];
        if (step.kind == kind && step.object == object && step.part == part) {
            return c < B.opExecution.ran.size() && B.opExecution.ran[c];
        }
    }
    return false;
}

/// The op value of \p domain slot \p slot, or null.
const RigExecOpValueState *
OpValueAt(const RigExecBakedProgramImpl &B, RigExecBakedSlotDomain domain,
          int slot)
{
    for (const RigExecOpValueState &value : B.opAdapter.values) {
        if (value.domain == uint32_t(domain) && int(value.slot) == slot) {
            return &value;
        }
    }
    return nullptr;
}

/// The chain position of the revision of mover \p path in \p B's only
/// chain, or -1: the chain orders its movers itself, so they are found by
/// path.
int
RangeChainPosition(const RigExecBakedProgramImpl &B, const char *path)
{
    if (B.chains.size() != 1) {
        return -1;
    }
    for (size_t r = 0; r < B.chains[0].revisions.size(); ++r) {
        if (B.chains[0].revisions[r].moverPath == SdfPath(path)) {
            return int(r);
        }
    }
    return -1;
}

/// The range chain's layout: three revisions, each range-pipelined with the
/// three Build bounds of 10,000 points, its own buffer as its version, three
/// range steps each reading its predecessor's range of the same part and no
/// version, and a join reading its own three ranges and, as a fuse does, the
/// version entering the revision.
void
TestARangeChainIsCutIntoRanges(const BuiltProgram &built, const char *name)
{
    CHECK(built.program != nullptr);
    if (!built.program) {
        return;
    }
    using D = RigExecBakedSlotDomain;
    using K = RigExecBakedStepKind;
    const RigExecBakedProgramImpl &B = built.program->GetStepGraph();
    CHECK(B.rangeChains);
    CHECK(B.chains.size() == 1 && B.chains[0].revisions.size() == 3);
    if (B.chains.size() != 1 || B.chains[0].revisions.size() != 3) {
        return;
    }
    const RigExecBakedProgramImpl::GeomChain &chain = B.chains[0];
    const int first = B.chainRevisionBegin[0];
    CHECK(B.chunkRevision.size() == size_t(B.chainChunkEnd[0]));
    const int bounds[4] = {0, 3334, 6668, 10000};
    for (size_t r = 0; r < chain.revisions.size(); ++r) {
        const RigExecBakedProgramImpl::GeomRevision &revision =
            chain.revisions[r];
        const int id = first + int(r);
        CHECK(revision.op == RigExecRevisionOp::Matrix);
        CHECK(revision.rangeRole && !revision.chunked);
        CHECK(revision.currentSource == int(r));
        CHECK(revision.chunks.size() == 3 && revision.joinSeen.size() == 3);
        CHECK(B.revisionChunkCount[size_t(id)] == 3);
        if (revision.chunks.size() != 3 ||
            B.revisionChunkCount[size_t(id)] != 3) {
            continue;
        }
        const int base = B.revisionChunkBase[size_t(id)];
        for (int k = 0; k < 3; ++k) {
            const auto &chunk = revision.chunks[size_t(k)];
            CHECK(chunk.begin == bounds[k] && chunk.end == bounds[k + 1]);
            CHECK(chunk.key.empty());
            CHECK(size_t(base + k) < B.chunkRevision.size() &&
                  B.chunkRevision[size_t(base + k)] == id);
        }
        const int previous = r > 0 ? B.revisionChunkBase[size_t(id) - 1] : -1;
        size_t ranges = 0, joins = 0;
        for (size_t index = 0; index < B.steps.size(); ++index) {
            const RigExecBakedStep &step = B.steps[index];
            if (step.object != id) {
                continue;
            }
            if (step.kind == K::RevisionChunk) {
                ++ranges;
                CHECK(DeclaresSlot(step.writes, D::RevisionOut,
                                   base + step.part));
                CHECK(DeclaresSlot(step.reads, D::RevisionPacket, id));
                CHECK(DeclaresSlot(step.reads, D::ChainBase, 0));
                CHECK(DeclaredSlots(step.reads, D::RevisionOut) ==
                      (r > 0 ? 1u : 0u));
                CHECK(DeclaredSlots(step.reads, D::RevisionDone) == 0 &&
                      DeclaredSlots(step.reads, D::ChainDirty) == 0);
                if (r > 0) {
                    CHECK(DeclaresSlot(step.reads, D::RevisionOut,
                                       previous + step.part));
                }
            } else if (step.kind == K::RevisionFuse) {
                ++joins;
                CHECK(int(index) == B.revisionFuseStep[size_t(id)]);
                CHECK(DeclaredSlots(step.reads, D::RevisionOut) == 3);
                for (int k = 0; k < 3; ++k) {
                    CHECK(DeclaresSlot(step.reads, D::RevisionOut, base + k));
                }
                CHECK(DeclaredSlots(step.reads, D::RevisionDone) ==
                          (r > 0 ? 1u : 0u) &&
                      DeclaredSlots(step.reads, D::ChainDirty) ==
                          (r > 0 ? 1u : 0u));
                if (r > 0) {
                    CHECK(DeclaresSlot(step.reads, D::RevisionDone, id - 1) &&
                          DeclaresSlot(step.reads, D::ChainDirty, id - 1));
                    // Ordered after the predecessor's join.
                    CHECK(std::binary_search(
                        step.preds.begin(), step.preds.end(),
                        B.revisionFuseStep[size_t(id) - 1]));
                }
                CHECK(DeclaresSlot(step.writes, D::RevisionDone, id) &&
                      DeclaresSlot(step.writes, D::ChainDirty, id));
            }
        }
        CHECK(ranges == 3 && joins == 1);
    }
    std::printf("  %s: 3 revisions of 3 ranges each\n", name);
}

/// The validator holds a range-pipelined chain to its reads by name: a
/// range without its predecessor's range, a range waiting for a point
/// version, a join without the version entering it, and a join missing one
/// of its own ranges. Each case restores what it broke.
void
TestTheValidatorRejectsABrokenRangeChain()
{
    BuiltProgram built = BuildStage(MakeRangeChainStage(10000, false, false));
    CHECK(built.program != nullptr);
    if (!built.program) {
        return;
    }
    using D = RigExecBakedSlotDomain;
    using K = RigExecBakedStepKind;
    // The program is this test's own, so it may be edited in place.
    RigExecBakedProgramImpl &B =
        const_cast<RigExecBakedProgramImpl &>(built.program->GetStepGraph());
    CHECK(B.chains.size() == 1 && B.chains[0].revisions.size() == 3);
    if (B.chains.size() != 1 || B.chains[0].revisions.size() != 3) {
        return;
    }
    // The chain's second revision, its range 1 and its join.
    const int id = B.chainRevisionBegin[0] + 1;
    int range = -1;
    for (size_t index = 0; index < B.steps.size(); ++index) {
        if (B.steps[index].kind == K::RevisionChunk &&
            B.steps[index].object == id && B.steps[index].part == 1) {
            range = int(index);
        }
    }
    const int join = B.revisionFuseStep[size_t(id)];
    CHECK(range >= 0 && join >= 0);
    if (range < 0 || join < 0) {
        return;
    }
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
        RigExecBakedStep &reader = B.steps[size_t(range)];
        const std::vector<RigExecBakedSlotRange> reads = reader.reads;
        const RigExecBakedSlotRange entering = RigExecBakedOne(
            D::RevisionOut, B.revisionChunkBase[size_t(id) - 1] + 1);
        reader.reads.erase(
            std::remove(reader.reads.begin(), reader.reads.end(), entering),
            reader.reads.end());
        CHECK(reader.reads.size() + 1 == reads.size());
        ExpectRejected(B, "a range without its predecessor's range",
                       {"(" + reader.label + ")",
                        "reads range 1 of point version 1 of chain 0",
                        "without declaring its predecessor's RevisionOut"});
        reader.reads = reads;
        passes("a range without its predecessor's range");
    }
    {
        // The version read with its producer's edge, so only the
        // pipelining rule can object. The predecessor's join need not
        // precede the range in program order; the case runs where it does.
        RigExecBakedStep &reader = B.steps[size_t(range)];
        const int producer = B.revisionFuseStep[size_t(id) - 1];
        if (producer < 0 || producer > range) {
            std::printf("  a range waiting for a point version: skipped, the "
                        "predecessor's join is ordered after it\n");
        } else {
            std::vector<int> &succs = B.steps[size_t(producer)].succs;
            const std::vector<RigExecBakedSlotRange> reads = reader.reads;
            const std::vector<int> preds = reader.preds;
            const std::vector<int> producerSuccs = succs;
            reader.reads.push_back(RigExecBakedOne(D::RevisionDone, id - 1));
            reader.reads.push_back(RigExecBakedOne(D::ChainDirty, id - 1));
            if (!std::binary_search(reader.preds.begin(), reader.preds.end(),
                                    producer)) {
                reader.preds.insert(
                    std::lower_bound(reader.preds.begin(), reader.preds.end(),
                                     producer),
                    producer);
                succs.insert(
                    std::lower_bound(succs.begin(), succs.end(), range),
                    range);
            }
            ExpectRejected(B, "a range waiting for a point version",
                           {"(" + reader.label + ")",
                            "reads point version 1 of chain 0",
                            "which a range-pipelined range must not wait for"});
            reader.reads = reads;
            reader.preds = preds;
            succs = producerSuccs;
            passes("a range waiting for a point version");
        }
    }
    {
        // The join keeps the joins in chain order through the version it
        // declares; without it, the version rule objects.
        RigExecBakedStep &reader = B.steps[size_t(join)];
        const std::vector<RigExecBakedSlotRange> reads = reader.reads;
        std::vector<RigExecBakedSlotRange> kept;
        for (const RigExecBakedSlotRange &read : reader.reads) {
            if (read.domain != D::RevisionDone) {
                kept.push_back(read);
            }
        }
        CHECK(kept.size() + 1 == reads.size());
        reader.reads = kept;
        ExpectRejected(B, "a join without the version entering it",
                       {"(" + reader.label + ")",
                        "reads point version 1 of chain 0 without declaring it"});
        reader.reads = reads;
        passes("a join without the version entering it");
    }
    {
        RigExecBakedStep &reader = B.steps[size_t(join)];
        const std::vector<RigExecBakedSlotRange> reads = reader.reads;
        const int base = B.revisionChunkBase[size_t(id)];
        for (RigExecBakedSlotRange &read : reader.reads) {
            if (read.domain == D::RevisionOut) {
                read.end = uint32_t(base + 2);
            }
        }
        ExpectRejected(B, "a join missing one of its ranges",
                       {"(" + reader.label + ")",
                        "joins revision " + std::to_string(id) +
                            " without declaring RevisionOut[" +
                            std::to_string(base + 2) + "]"});
        reader.reads = reads;
        passes("a join missing one of its ranges");
    }
}

/// The range chain against itself built with RIGEXEC_BAKED_RANGE_CHAINS=0
/// (one chunk and one fuse per revision) over frames {1,2,3,2,1,3}: every
/// revision's version, the chain's points and every diagnostic agree bit
/// for bit, M1's MoverFailed line stands at frame 2 alone, and both
/// RIGEXEC_VERIFY_CHAIN_VERSIONS and the RIGEXEC_VERIFY_RANGE_CHAINS judge
/// find nothing. The judge itself is shown to catch one wrong point.
void
TestARangeChainMatchesTheWholeChain()
{
    TfSetenv("RIGEXEC_VERIFY_CHAIN_VERSIONS", "1");
    TfSetenv("RIGEXEC_VERIFY_RANGE_CHAINS", "1");
    const BuiltProgram ranged =
        BuildStage(MakeRangeChainStage(10000, false, true));
    TfSetenv("RIGEXEC_BAKED_RANGE_CHAINS", "0");
    const BuiltProgram whole =
        BuildStage(MakeRangeChainStage(10000, false, true));
    TfUnsetenv("RIGEXEC_BAKED_RANGE_CHAINS");
    TfSetenv("RIGEXEC_VERIFY_RANGE_CHAINS", "0");
    TfSetenv("RIGEXEC_VERIFY_CHAIN_VERSIONS", "0");
    CHECK(ranged.program != nullptr && whole.program != nullptr);
    if (!ranged.program || !whole.program) {
        return;
    }
    const RigExecBakedProgramImpl &R = ranged.program->GetStepGraph();
    const RigExecBakedProgramImpl &W = whole.program->GetStepGraph();
    CHECK(R.verifyRangeChains && R.verifyChainVersions);
    CHECK(R.rangeChains && !W.rangeChains);
    CHECK(R.chains.size() == 1 && W.chains.size() == 1 &&
          R.chains[0].revisions.size() == 3 &&
          W.chains[0].revisions.size() == 3);
    if (R.chains.size() != 1 || W.chains.size() != 1 ||
        R.chains[0].revisions.size() != 3 ||
        W.chains[0].revisions.size() != 3) {
        return;
    }
    for (size_t r = 0; r < 3; ++r) {
        CHECK(R.chains[0].revisions[r].rangeRole);
        CHECK(!W.chains[0].revisions[r].rangeRole &&
              W.chains[0].revisions[r].chunks.size() == 1);
        CHECK(R.chains[0].revisions[r].moverPath ==
              W.chains[0].revisions[r].moverPath);
    }
    const int middle = RangeChainPosition(R, "/Asset/Rig/Movers/M1");
    CHECK(middle >= 0);
    if (middle < 0) {
        return;
    }
    const SdfPath target("/Asset/Shape.points");
    // Version r + 1 of the whole chain: where its fuse left it.
    const auto version = [](const RigExecBakedProgramImpl::GeomChain &chain,
                            size_t r) {
        const int source = chain.revisions[r].currentSource;
        return source < 0
                   ? std::vector<GfVec3f>(chain.lastBase.cbegin(),
                                          chain.lastBase.cend())
                   : chain.revisions[size_t(source)].output;
    };
    for (const double frame : {1.0, 2.0, 3.0, 2.0, 1.0, 3.0}) {
        RigExecRigPose rangedPose, wholePose;
        CHECK(ranged.program->Run(UsdTimeCode(frame), &rangedPose));
        CHECK(whole.program->Run(UsdTimeCode(frame), &wholePose));
        CHECK(rangedPose.comparisonMismatches == 0 &&
              wholePose.comparisonMismatches == 0);
        const auto &rangedChain = R.chains[0];
        const auto &wholeChain = W.chains[0];
        CHECK(rangedChain.result.size() == 10000);
        CHECK(SameBits(rangedChain.result, wholeChain.result));
        for (size_t r = 0; r < 3; ++r) {
            if (!SameBits(rangedChain.revisions[r].output,
                          version(wholeChain, r))) {
                ++failures;
                std::printf("FAIL range chain frame %g: revision %zu's "
                            "version differs from the whole chain's\n",
                            frame, r);
            }
            CHECK(rangedChain.revisions[r].resultStatus ==
                  wholeChain.revisions[r].resultStatus);
            CHECK(rangedChain.revisions[r].rangeRefusals == 0);
        }
        const auto rangedPoints = rangedPose.movedProperties.find(target);
        const auto wholePoints = wholePose.movedProperties.find(target);
        CHECK(rangedPoints != rangedPose.movedProperties.end() &&
              wholePoints != wholePose.movedProperties.end() &&
              rangedPoints->second.IsHolding<VtVec3fArray>() &&
              wholePoints->second.IsHolding<VtVec3fArray>() &&
              SameBits(rangedPoints->second.UncheckedGet<VtVec3fArray>(),
                       wholePoints->second.UncheckedGet<VtVec3fArray>()));
        CHECK(rangedPose.diagnostics == wholePose.diagnostics);
        const std::vector<std::string> lines =
            MoverFailedLines(rangedPose.diagnostics);
        CHECK(NamesMover(lines, "/Asset/Rig/Movers/M1:") == (frame == 2.0));
        CHECK((rangedChain.revisions[size_t(middle)].resultStatus ==
               TfToken("moverFailed")) == (frame == 2.0));
    }
    CHECK(R.chainVersionMismatches == 0 && W.chainVersionMismatches == 0);
    CHECK(R.rangeVerifyMismatches == 0);

    // The judge reports a range that disagrees with the revision run whole.
    RigExecBakedProgramImpl &edited = const_cast<RigExecBakedProgramImpl &>(R);
    std::vector<GfVec3f> &output = edited.chains[0].revisions[2].output;
    CHECK(output.size() == 10000);
    if (output.size() == 10000) {
        const GfVec3f kept = output[5000];
        output[5000][0] += 1.0f;
        {
            TfErrorMark mark;
            CHECK(RigExecBakedVerifyRangeChains(&edited) == 1);
            mark.Clear();
        }
        output[5000] = kept;
        CHECK(RigExecBakedVerifyRangeChains(&edited) == 0);
    }
}

/// Range cutoff: M0 moves points [0, 100) only, so between frames 1 and 2
/// every range of M0 runs (its packet moved) and only range 0 of every
/// revision after it does; its ranges 1 and 2 publish the bytes they held,
/// so their RevisionOut values keep their revision, and nothing before M0
/// runs. The points match the chain built whole.
void
TestARangeChainCutsOffUnmovedRanges()
{
    TfSetenv("RIGEXEC_VERIFY_CHAIN_VERSIONS", "1");
    const BuiltProgram built = BuildStage(MakeRangeChainStage(10000, true, false));
    TfSetenv("RIGEXEC_BAKED_RANGE_CHAINS", "0");
    const BuiltProgram whole = BuildStage(MakeRangeChainStage(10000, true, false));
    TfUnsetenv("RIGEXEC_BAKED_RANGE_CHAINS");
    TfSetenv("RIGEXEC_VERIFY_CHAIN_VERSIONS", "0");
    CHECK(built.program != nullptr && whole.program != nullptr);
    if (!built.program || !whole.program) {
        return;
    }
    using D = RigExecBakedSlotDomain;
    using K = RigExecBakedStepKind;
    const RigExecBakedProgramImpl &B = built.program->GetStepGraph();
    const RigExecBakedProgramImpl &W = whole.program->GetStepGraph();
    CHECK(B.chains.size() == 1 && B.chains[0].revisions.size() == 3 &&
          W.chains.size() == 1);
    if (B.chains.size() != 1 || B.chains[0].revisions.size() != 3 ||
        W.chains.size() != 1) {
        return;
    }
    const int moved = RangeChainPosition(B, "/Asset/Rig/Movers/M0");
    CHECK(moved >= 0);
    if (moved < 0) {
        return;
    }
    const int first = B.chainRevisionBegin[0];
    for (const auto &revision : B.chains[0].revisions) {
        CHECK(revision.rangeRole && revision.chunks.size() == 3);
    }
    RigExecRigPose pose, wholePose;
    CHECK(built.program->Run(UsdTimeCode(1.0), &pose));
    CHECK(whole.program->Run(UsdTimeCode(1.0), &wholePose));
    CHECK(SameBits(B.chains[0].result, W.chains[0].result));
    std::vector<uint64_t> before(9, 0);
    for (int r = 0; r < 3; ++r) {
        for (int k = 0; k < 3; ++k) {
            const RigExecOpValueState *value = OpValueAt(
                B, D::RevisionOut, B.revisionChunkBase[size_t(first + r)] + k);
            CHECK(value != nullptr);
            before[size_t(r * 3 + k)] = value ? value->revision : 0;
        }
    }
    const VtVec3fArray frameOne = B.chains[0].result;
    CHECK(built.program->Run(UsdTimeCode(2.0), &pose));
    CHECK(whole.program->Run(UsdTimeCode(2.0), &wholePose));
    CHECK(pose.comparisonMismatches == 0);
    CHECK(SameBits(B.chains[0].result, W.chains[0].result));
    // Only points [0, 100) moved.
    const VtVec3fArray &frameTwo = B.chains[0].result;
    CHECK(frameTwo.size() == frameOne.size() && frameTwo.size() == 10000);
    if (frameTwo.size() == 10000 && frameOne.size() == 10000) {
        CHECK(!SameBits(std::vector<GfVec3f>(frameOne.cbegin(),
                                             frameOne.cbegin() + 100),
                        std::vector<GfVec3f>(frameTwo.cbegin(),
                                             frameTwo.cbegin() + 100)));
        CHECK(std::memcmp(frameOne.cdata() + 100, frameTwo.cdata() + 100,
                          9900 * sizeof(GfVec3f)) == 0);
    }
    for (int r = 0; r < 3; ++r) {
        const int id = first + r;
        for (int k = 0; k < 3; ++k) {
            const bool ran = RanLastPart(B, K::RevisionChunk, id, k);
            const bool expected = r == moved || (r > moved && k == 0);
            if (ran != expected) {
                ++failures;
                std::printf("FAIL range cutoff: revision %d range %d %s\n", r,
                            k, ran ? "ran" : "did not run");
            }
            const RigExecOpValueState *value = OpValueAt(
                B, D::RevisionOut, B.revisionChunkBase[size_t(id)] + k);
            const bool republished =
                value && value->revision != before[size_t(r * 3 + k)];
            CHECK(republished == (r >= moved && k == 0));
        }
        CHECK(RanLast(B, K::RevisionFuse, id) == (r >= moved));
    }
    CHECK(B.chainVersionMismatches == 0);
}

/// Three matrix movers over 10,000 points authored Alpha, Middle, Zeta;
/// mover discovery runs siblings bottom to top, so the chain runs Zeta,
/// Middle, Alpha while their paths sort the other way.
/// Zeta's and Alpha's inputs:defaultWeight leave [0, 1] at frame 2, so both
/// joins emit a line there.
UsdStageRefPtr
MakeReversedLinesStage()
{
    const UsdStageRefPtr stage = UsdStage::CreateInMemory();
    stage->DefinePrim(SdfPath("/Asset"), TfToken("Scope"));
    stage->DefinePrim(SdfPath("/Asset/Rig"), TfToken("RigExecRoot"));
    const UsdPrim moving = stage->DefinePrim(
        SdfPath("/Asset/Rig/Moving"), TfToken("RigExecControl"));
    for (int frame = 1; frame <= 3; ++frame) {
        moving.GetAttribute(TfToken("avars:tx"))
            .Set(double(frame), UsdTimeCode(frame));
    }
    const SdfPath target("/Asset/Shape.points");
    VtVec3fArray base(10000);
    for (size_t i = 0; i < base.size(); ++i) {
        base[i] = GfVec3f(float(i % 101) * 0.25f, float(i / 101) * 0.125f,
                          1.0f + float(i % 7));
    }
    stage->DefinePrim(target.GetPrimPath(), TfToken("Points"))
        .GetAttribute(TfToken("points"))
        .Set(base);
    stage->DefinePrim(SdfPath("/Asset/Rig/Movers"), TfToken("Scope"));
    for (const char *name : {"Alpha", "Middle", "Zeta"}) {
        const UsdPrim mover = stage->DefinePrim(
            SdfPath("/Asset/Rig/Movers").AppendChild(TfToken(name)),
            TfToken("RigExecMatrixMover"));
        mover.ApplyAPI(TfToken("RigExecMoverAPI"));
        mover.GetRelationship(TfToken("rigExec:moves")).SetTargets({target});
        mover.GetRelationship(TfToken("rigExec:transform"))
            .SetTargets({moving.GetPath()});
        UsdAttribute scalar =
            mover.GetAttribute(TfToken("inputs:defaultWeight"));
        if (std::string(name) == "Middle") {
            scalar.Set(1.0f);
        } else {
            scalar.Set(1.0f, UsdTimeCode(1.0));
            scalar.Set(2.0f, UsdTimeCode(2.0));
            scalar.Set(1.0f, UsdTimeCode(3.0));
        }
    }
    return stage;
}

/// The joins publish their lines in the unsplit chain's order: two failing
/// movers whose chain order is the reverse of their paths' order report, at
/// frame 2, exactly the lines the chain built whole reports, Zeta's first.
/// The canonical order breaks ties among ready steps by path, so a join
/// that waited only for its own ranges would let Alpha's line overtake.
void
TestRangeJoinsKeepTheLineOrder()
{
    const BuiltProgram ranged = BuildStage(MakeReversedLinesStage());
    TfSetenv("RIGEXEC_BAKED_RANGE_CHAINS", "0");
    const BuiltProgram whole = BuildStage(MakeReversedLinesStage());
    TfUnsetenv("RIGEXEC_BAKED_RANGE_CHAINS");
    CHECK(ranged.program != nullptr && whole.program != nullptr);
    if (!ranged.program || !whole.program) {
        return;
    }
    const RigExecBakedProgramImpl &R = ranged.program->GetStepGraph();
    const int zeta = RangeChainPosition(R, "/Asset/Rig/Movers/Zeta");
    const int alpha = RangeChainPosition(R, "/Asset/Rig/Movers/Alpha");
    CHECK(zeta >= 0 && alpha > zeta);
    if (zeta < 0 || alpha <= zeta) {
        return;
    }
    for (const auto &revision : R.chains[0].revisions) {
        CHECK(revision.rangeRole);
    }
    const std::string zetaLine = "MoverFailed /Asset/Rig/Movers/Zeta: "
                                 "inputs:defaultWeight must be finite";
    const std::string alphaLine = "MoverFailed /Asset/Rig/Movers/Alpha: "
                                  "inputs:defaultWeight must be finite";
    const auto at = [](const std::vector<std::string> &lines,
                       const std::string &prefix) {
        for (size_t i = 0; i < lines.size(); ++i) {
            if (lines[i].rfind(prefix, 0) == 0) {
                return int(i);
            }
        }
        return -1;
    };
    for (const double frame : {1.0, 2.0, 3.0, 2.0}) {
        RigExecRigPose rangedPose, wholePose;
        CHECK(ranged.program->Run(UsdTimeCode(frame), &rangedPose));
        CHECK(whole.program->Run(UsdTimeCode(frame), &wholePose));
        CHECK(SameBits(R.chains[0].result,
                       whole.program->GetStepGraph().chains[0].result));
        const bool same = rangedPose.diagnostics == wholePose.diagnostics;
        CHECK(same);
        if (!same) {
            std::printf("FAIL range join lines at frame %g:\n", frame);
            for (const std::string &line : rangedPose.diagnostics) {
                std::printf("  ranged: %s\n", line.c_str());
            }
            for (const std::string &line : wholePose.diagnostics) {
                std::printf("  whole:  %s\n", line.c_str());
            }
        }
        const int first = at(rangedPose.diagnostics, zetaLine);
        const int second = at(rangedPose.diagnostics, alphaLine);
        if (frame == 2.0) {
            CHECK(first >= 0 && second > first);
        } else {
            CHECK(first < 0 && second < 0);
        }
    }
}

/// One matrix mover on a control that moves every frame, weighted by a
/// sparse RigExecDynamicWeight (clamp) over a sparse static base that names
/// all three points with weight 1 and defaults to 0.25. inputs:driver is 2
/// at frames 1 and 2, 3 at frame 3 and 0.5 at frame 4: the packet stands at
/// frame 2, moves only its default at frame 3 (every value clamps to 1, and
/// no point reads the default), and moves its values at frame 4.
///
/// With \p unnamedLast the base and the weight name only points 0 and 1, so
/// point 2 reads the default: at frame 3 the values and indices stay
/// byte-equal ({1, 1}) while the field's last entry goes 0.5 -> 0.75.
UsdStageRefPtr
MakeWeightOverlayStage(bool unnamedLast = false)
{
    const VtIntArray named =
        unnamedLast ? VtIntArray{0, 1} : VtIntArray{0, 1, 2};
    const UsdStageRefPtr stage = UsdStage::CreateInMemory();
    stage->DefinePrim(SdfPath("/Asset"), TfToken("Scope"));
    stage->DefinePrim(SdfPath("/Asset/Rig"), TfToken("RigExecRoot"));
    const UsdPrim moving = stage->DefinePrim(
        SdfPath("/Asset/Rig/Moving"), TfToken("RigExecControl"));
    for (int frame = 1; frame <= 4; ++frame) {
        moving.GetAttribute(TfToken("avars:tx"))
            .Set(double(frame), UsdTimeCode(frame));
    }
    const SdfPath target("/Asset/Shape.points");
    const UsdPrim shape =
        stage->DefinePrim(target.GetPrimPath(), TfToken("Points"));
    shape.GetAttribute(TfToken("points"))
        .Set(VtVec3fArray{GfVec3f(0), GfVec3f(1, 0, 0), GfVec3f(0, 2, 0)});
    const UsdPrim base = stage->DefinePrim(
        SdfPath("/Asset/Rig/Weights/Base"), TfToken("RigExecStaticWeight"));
    base.CreateRelationship(TfToken("rigExec:weightTarget"), false)
        .SetTargets({target});
    base.CreateAttribute(TfToken("rigExec:representation"),
                         SdfValueTypeNames->Token, false)
        .Set(TfToken("sparse"));
    base.CreateAttribute(TfToken("rigExec:indices"),
                         SdfValueTypeNames->IntArray, false)
        .Set(named);
    base.CreateAttribute(TfToken("rigExec:values"),
                         SdfValueTypeNames->FloatArray, false)
        .Set(VtFloatArray(named.size(), 1.0f));
    base.CreateAttribute(TfToken("rigExec:defaultWeight"),
                         SdfValueTypeNames->Float, false)
        .Set(0.25f);
    const UsdPrim weight = stage->DefinePrim(
        SdfPath("/Asset/Rig/Weights/W"), TfToken("RigExecDynamicWeight"));
    weight.CreateRelationship(TfToken("rigExec:weightTarget"), false)
        .SetTargets({target});
    weight.CreateRelationship(TfToken("rigExec:baseWeight"), false)
        .SetTargets({base.GetPath()});
    weight.CreateAttribute(TfToken("rigExec:representation"),
                           SdfValueTypeNames->Token, false)
        .Set(TfToken("sparse"));
    weight.CreateAttribute(TfToken("rigExec:rangePolicy"),
                           SdfValueTypeNames->Token, false)
        .Set(TfToken("clamp"));
    weight.CreateAttribute(TfToken("rigExec:indices"),
                           SdfValueTypeNames->IntArray, false)
        .Set(named);
    UsdAttribute driver = weight.CreateAttribute(
        TfToken("inputs:driver"), SdfValueTypeNames->Float, false);
    driver.Set(2.0f, UsdTimeCode(1.0));
    driver.Set(2.0f, UsdTimeCode(2.0));
    driver.Set(3.0f, UsdTimeCode(3.0));
    driver.Set(0.5f, UsdTimeCode(4.0));
    stage->DefinePrim(SdfPath("/Asset/Rig/Movers"), TfToken("Scope"));
    const UsdPrim mover = stage->DefinePrim(
        SdfPath("/Asset/Rig/Movers/M0"), TfToken("RigExecMatrixMover"));
    mover.ApplyAPI(TfToken("RigExecMoverAPI"));
    mover.GetRelationship(TfToken("rigExec:moves")).SetTargets({target});
    mover.GetRelationship(TfToken("rigExec:transform"))
        .SetTargets({moving.GetPath()});
    mover.CreateRelationship(TfToken("rigExec:weightObject"), false)
        .SetTargets({weight.GetPath()});
    return stage;
}

/// The weight overlay stage's mover revision, found by path, never by
/// position; \p object receives its revisionIndex entry.
const RigExecBakedProgramImpl::GeomRevision *
WeightOverlayRevision(const RigExecBakedProgramImpl &B, int *object)
{
    const RigExecBakedProgramImpl::GeomRevision *found = nullptr;
    for (size_t o = 0; o < B.revisionIndex.size(); ++o) {
        const auto &[c, r] = B.revisionIndex[o];
        const auto &candidate = B.chains[size_t(c)].revisions[size_t(r)];
        if (candidate.moverPath == SdfPath("/Asset/Rig/Movers/M0")) {
            *object = int(o);
            found = &candidate;
        }
    }
    return found;
}

/// RevisionStatic reuses its published weight field while the WeightPacket
/// op value it was resolved from keeps its revision and the count stands,
/// and keys the field by a content version that moves exactly when the
/// bytes do. Under RIGEXEC_VERIFY_PACKET_VERSIONS every published packet is
/// also keyed over the bytes and both must tell the same change.
void
TestTheWeightOverlayIsReusedByVersion()
{
    TfSetenv("RIGEXEC_VERIFY_PACKET_VERSIONS", "1");
    const BuiltProgram built = BuildStage(MakeWeightOverlayStage());
    TfSetenv("RIGEXEC_VERIFY_PACKET_VERSIONS", "0");
    CHECK(built.program != nullptr);
    if (!built.program) {
        return;
    }
    const RigExecBakedProgramImpl &B = built.program->GetStepGraph();
    CHECK(B.verifyPacketVersions);
    int object = -1;
    const RigExecBakedProgramImpl::GeomRevision *found =
        WeightOverlayRevision(B, &object);
    CHECK(found != nullptr);
    if (!found) {
        return;
    }
    const RigExecBakedProgramImpl::GeomRevision &revision = *found;
    CHECK(revision.weightObject >= 0 && !revision.weightCurrentPhase);
    CHECK(revision.weightPacketValue >= 0 &&
          size_t(revision.weightPacketValue) < B.opAdapter.values.size());
    if (revision.weightPacketValue < 0 ||
        size_t(revision.weightPacketValue) >= B.opAdapter.values.size()) {
        return;
    }
    const RigExecOpValueState &packet =
        B.opAdapter.values[size_t(revision.weightPacketValue)];
    CHECK(packet.domain == uint32_t(RigExecBakedSlotDomain::WeightPacket) &&
          int(packet.slot) == revision.weightObject);
    const auto field = [&](float w) {
        return revision.weightFieldPublished &&
               revision.publishedWeightValues == VtFloatArray(3, w);
    };
    using K = RigExecBakedStepKind;
    RigExecRigPose pose;
    CHECK(built.program->Run(UsdTimeCode(1.0), &pose));
    CHECK(pose.comparisonMismatches == 0);
    CHECK(field(1.0f) && revision.weightValuesHeld);
    CHECK(revision.weightValuesPacketRevision == packet.revision);
    const uint64_t version = revision.weightValuesVersion;
    const uint64_t standing = packet.revision;

    // The control moved, so the packet was reassembled; the weight packet
    // did not, so the field is the held one and keeps its version.
    CHECK(built.program->Run(UsdTimeCode(2.0), &pose));
    CHECK(pose.comparisonMismatches == 0);
    CHECK(RanLast(B, K::RevisionStatic, object));
    CHECK(packet.revision == standing);
    CHECK(field(1.0f) && revision.weightValuesVersion == version);

    // A new packet whose field resolves to the same bytes: resolved again
    // (the claim follows the new revision), and the version stands.
    CHECK(built.program->Run(UsdTimeCode(3.0), &pose));
    CHECK(pose.comparisonMismatches == 0);
    CHECK(RanLast(B, K::RevisionStatic, object));
    CHECK(packet.revision != standing);
    CHECK(revision.weightValuesPacketRevision == packet.revision);
    CHECK(field(1.0f) && revision.weightValuesVersion == version);

    // Real changes bump it, back to bytes it held before included.
    CHECK(built.program->Run(UsdTimeCode(4.0), &pose));
    CHECK(pose.comparisonMismatches == 0);
    CHECK(field(0.5f) && revision.weightValuesVersion == version + 1);
    CHECK(built.program->Run(UsdTimeCode(1.0), &pose));
    CHECK(pose.comparisonMismatches == 0);
    CHECK(field(1.0f) && revision.weightValuesVersion == version + 2);
    const auto published = pose.weightFields.find(SdfPath("/Asset/Rig/Weights/W"));
    CHECK(published != pose.weightFields.end() &&
          published->second.weights.size() == 3);
    CHECK(B.packetVersionMismatches == 0);
}

/// Only the packet's default moves at frame 3, and point 2 reads it: the
/// values and indices the field resolves from stay byte-equal while the
/// field moves, so the reuse must follow the whole packet (its revision),
/// never its arrays alone.
void
TestADefaultOnlyMoveResolvesTheWeightField()
{
    TfSetenv("RIGEXEC_VERIFY_PACKET_VERSIONS", "1");
    const BuiltProgram built = BuildStage(MakeWeightOverlayStage(true));
    TfSetenv("RIGEXEC_VERIFY_PACKET_VERSIONS", "0");
    CHECK(built.program != nullptr);
    if (!built.program) {
        return;
    }
    const RigExecBakedProgramImpl &B = built.program->GetStepGraph();
    int object = -1;
    const RigExecBakedProgramImpl::GeomRevision *found =
        WeightOverlayRevision(B, &object);
    CHECK(found != nullptr);
    if (!found) {
        return;
    }
    const RigExecBakedProgramImpl::GeomRevision &revision = *found;
    CHECK(revision.weightObject >= 0 && !revision.weightCurrentPhase);
    const auto field = [&](float last) {
        return revision.weightFieldPublished &&
               revision.publishedWeightValues ==
                   VtFloatArray{1.0f, 1.0f, last};
    };
    RigExecRigPose pose;
    CHECK(built.program->Run(UsdTimeCode(1.0), &pose));
    CHECK(pose.comparisonMismatches == 0);
    CHECK(field(0.5f));
    const uint64_t version = revision.weightValuesVersion;
    CHECK(built.program->Run(UsdTimeCode(2.0), &pose));
    CHECK(pose.comparisonMismatches == 0);
    CHECK(field(0.5f) && revision.weightValuesVersion == version);
    // The default alone moved: the field follows it and its version moves.
    CHECK(built.program->Run(UsdTimeCode(3.0), &pose));
    CHECK(pose.comparisonMismatches == 0);
    CHECK(RanLast(B, RigExecBakedStepKind::RevisionStatic, object));
    if (!field(0.75f)) {
        std::printf("FAIL default-only move: the field did not follow the "
                    "packet's default\n");
    }
    CHECK(field(0.75f) && revision.weightValuesVersion == version + 1);
    const auto published =
        pose.weightFields.find(SdfPath("/Asset/Rig/Weights/W"));
    CHECK(published != pose.weightFields.end() &&
          std::vector<float>(published->second.weights.cbegin(),
                             published->second.weights.cend()) ==
              std::vector<float>({1.0f, 1.0f, 0.75f}));
    // And back: the held bytes of frame 1, under a new version.
    CHECK(built.program->Run(UsdTimeCode(1.0), &pose));
    CHECK(pose.comparisonMismatches == 0);
    CHECK(field(0.5f) && revision.weightValuesVersion == version + 2);
    CHECK(B.packetVersionMismatches == 0);
}

/// A rebuilt program's retained skin ops compare the keys the outgoing
/// program published with this one's: every point-carrying value the
/// adoption copied is republished unchanged at the same time, which holds
/// only if each content version crossed with the bytes it describes.
/// Returns how many such values were checked.
size_t
TestARebuildKeepsItsPointVersions(const std::string &stagePath,
                                  const char *name)
{
    TfSetenv("RIGEXEC_VERIFY_CHAIN_VERSIONS", "1");
    const BuiltProgram built = Build(stagePath);
    CHECK(built.program != nullptr);
    if (!built.program) {
        TfSetenv("RIGEXEC_VERIFY_CHAIN_VERSIONS", "0");
        return 0;
    }
    RigExecRigPose pose;
    CHECK(built.program->Run(UsdTimeCode(1.0), &pose));
    std::vector<std::string> reasons;
    std::unique_ptr<RigExecBakedProgram> rebuilt =
        RigExecBakedProgram::Build(built.evaluator.get(), &reasons);
    TfSetenv("RIGEXEC_VERIFY_CHAIN_VERSIONS", "0");
    CHECK(rebuilt != nullptr);
    if (!rebuilt) {
        return 0;
    }
    rebuilt->AdoptGeometryStateFrom(*built.program);
    const RigExecBakedProgramImpl &B = rebuilt->GetStepGraph();
    using D = RigExecBakedSlotDomain;
    std::vector<std::pair<size_t, uint64_t>> copied;
    for (size_t i = 0; i < B.opAdapter.values.size(); ++i) {
        const auto &value = B.opAdapter.values[i];
        const D domain = D(value.domain);
        if (value.initialized &&
            (domain == D::RevisionDone || domain == D::ChainDirty ||
             domain == D::ChainBase || domain == D::ChainPoints ||
             domain == D::ChainInput || domain == D::DerivedOut)) {
            copied.emplace_back(i, value.revision);
        }
    }
    CHECK(rebuilt->Run(UsdTimeCode(1.0), &pose));
    // Retained ops skipped this run: the held set still names exactly
    // the steps holding lines.
    CHECK(HeldStepsAreExact(B));
    for (const auto &[id, revision] : copied) {
        const auto &value = B.opAdapter.values[id];
        if (value.changed || value.revision != revision) {
            ++failures;
            std::printf("FAIL %s: copied value %zu (domain %u slot %u) moved "
                        "across the rebuild\n", name, id, value.domain,
                        value.slot);
        }
    }
    CHECK(B.chainVersionMismatches == 0);
    std::printf("  %s: %zu point value(s) kept their keys across a "
                "rebuild\n", name, copied.size());
    return copied.size();
}

/// Every point version of \p ranged's only chain, read as one array through
/// RigExecBakedVersionPoints, holds the bytes \p whole's does.
bool
EveryVersionAgrees(const RigExecBakedProgramImpl &ranged,
                   const RigExecBakedProgramImpl &whole)
{
    if (ranged.chains.size() != 1 || whole.chains.size() != 1 ||
        ranged.chains[0].revisions.size() !=
            whole.chains[0].revisions.size()) {
        return false;
    }
    std::vector<GfVec3f> rangedScratch, wholeScratch;
    for (size_t v = 0; v <= ranged.chains[0].revisions.size(); ++v) {
        const GfVec3f *a = nullptr;
        const GfVec3f *b = nullptr;
        size_t na = 0, nb = 0;
        RigExecBakedVersionPoints(ranged.chains[0], v, &rangedScratch, &a, &na);
        RigExecBakedVersionPoints(whole.chains[0], v, &wholeScratch, &b, &nb);
        if (na == 0 || !RigExecBakedSamePoints(a, na, b, nb)) {
            std::printf("FAIL version %zu of the range chain differs from "
                        "the chain built whole\n", v);
            return false;
        }
    }
    return true;
}

/// ChainStatus of a chain cut into groups gathers only when a group id of
/// its final version moved. Run again by hand with nothing moved, it keeps
/// `result`: the same array and the same version. A frame that moves only
/// M0's gated group 0 gathers once, into the other buffer, and bumps the
/// version once. Every version, read whole, matches the chain built whole
/// over the visits {1,2,3,2,1,3}.
void
TestTheChainStatusKeepsAnUnmovedResult()
{
    TfSetenv("RIGEXEC_VERIFY_CHAIN_VERSIONS", "1");
    const BuiltProgram built =
        BuildStage(MakeRangeChainStage(10000, true, false));
    TfSetenv("RIGEXEC_BAKED_RANGE_CHAINS", "0");
    const BuiltProgram whole =
        BuildStage(MakeRangeChainStage(10000, true, false));
    TfUnsetenv("RIGEXEC_BAKED_RANGE_CHAINS");
    TfSetenv("RIGEXEC_VERIFY_CHAIN_VERSIONS", "0");
    CHECK(built.program != nullptr && whole.program != nullptr);
    if (!built.program || !whole.program) {
        return;
    }
    RigExecBakedProgramImpl &B =
        const_cast<RigExecBakedProgramImpl &>(built.program->GetStepGraph());
    const RigExecBakedProgramImpl &W = whole.program->GetStepGraph();
    CHECK(B.chains.size() == 1 && W.chains.size() == 1);
    if (B.chains.size() != 1 || W.chains.size() != 1 ||
        B.chains[0].revisions.empty()) {
        return;
    }
    RigExecBakedProgramImpl::GeomChain &chain = B.chains[0];
    CHECK(chain.groupBounds.size() >= 3);
    CHECK(W.chains[0].groupBounds.empty());
    RigExecBakedStep *status = nullptr;
    for (RigExecBakedStep &step : B.steps) {
        if (step.kind == RigExecBakedStepKind::ChainStatus &&
            step.object == 0) {
            status = &step;
        }
    }
    CHECK(status != nullptr);
    if (!status || chain.groupBounds.size() < 3) {
        return;
    }
    RigExecRigPose pose, wholePose;
    CHECK(built.program->Run(UsdTimeCode(1.0), &pose));
    CHECK(whole.program->Run(UsdTimeCode(1.0), &wholePose));
    CHECK(pose.comparisonMismatches == 0);
    CHECK(SameBits(chain.result, W.chains[0].result));
    CHECK(EveryVersionAgrees(B, W));
    CHECK(chain.resultIds == chain.revisions.back().groupIds);

    // Nothing moved since: the status step keeps the array it published.
    const VtVec3fArray held = chain.result;
    const uint64_t version = chain.resultVersion;
    RigExecBakedRunGeometryStep(&B, status, UsdTimeCode(1.0));
    CHECK(chain.haveResult);
    CHECK(chain.result.IsIdentical(held));
    CHECK(chain.resultVersion == version);

    // Only group 0 moved: one gather, one bump, the held array untouched.
    const VtVec3fArray before(held.cbegin(), held.cend());
    CHECK(built.program->Run(UsdTimeCode(2.0), &pose));
    CHECK(whole.program->Run(UsdTimeCode(2.0), &wholePose));
    CHECK(pose.comparisonMismatches == 0);
    CHECK(RanLast(B, RigExecBakedStepKind::ChainStatus, 0));
    CHECK(chain.resultVersion == version + 1);
    CHECK(!chain.result.IsIdentical(held));
    CHECK(SameBits(held, before));
    CHECK(SameBits(chain.result, W.chains[0].result));
    CHECK(EveryVersionAgrees(B, W));
    CHECK(chain.resultIds == chain.revisions.back().groupIds);
    for (const double frame : {3.0, 2.0, 1.0, 3.0}) {
        CHECK(built.program->Run(UsdTimeCode(frame), &pose));
        CHECK(whole.program->Run(UsdTimeCode(frame), &wholePose));
        CHECK(pose.comparisonMismatches == 0);
        CHECK(SameBits(chain.result, W.chains[0].result));
        CHECK(EveryVersionAgrees(B, W));
        CHECK(pose.diagnostics == wholePose.diagnostics);
    }
    CHECK(B.chainVersionMismatches == 0);
}

/// The epilogue publishes a revision's weight field by sharing the array
/// RevisionStatic holds, never a copy: a frame whose packet stood, and a
/// frame where nothing moved, publish the very array frame 1 did; a frame
/// that moves the field publishes its new array, and the one published
/// before keeps its values.
void
TestTheWeightFieldIsSharedNotCopied()
{
    const BuiltProgram built = BuildStage(MakeWeightOverlayStage());
    CHECK(built.program != nullptr);
    if (!built.program) {
        return;
    }
    const RigExecBakedProgramImpl &B = built.program->GetStepGraph();
    int object = -1;
    const RigExecBakedProgramImpl::GeomRevision *revision =
        WeightOverlayRevision(B, &object);
    CHECK(revision != nullptr);
    if (!revision) {
        return;
    }
    const SdfPath weight("/Asset/Rig/Weights/W");
    const auto published = [&](const RigExecRigPose &pose) {
        const auto found = pose.weightFields.find(weight);
        return found == pose.weightFields.end() ? VtFloatArray()
                                                : found->second.weights;
    };
    RigExecRigPose one, two, again, four;
    CHECK(built.program->Run(UsdTimeCode(1.0), &one));
    CHECK(built.program->Run(UsdTimeCode(2.0), &two));
    CHECK(built.program->Run(UsdTimeCode(2.0), &again));
    CHECK(one.comparisonMismatches == 0 && two.comparisonMismatches == 0 &&
          again.comparisonMismatches == 0);
    const VtFloatArray first = published(one);
    CHECK(first == VtFloatArray(3, 1.0f));
    CHECK(first.IsIdentical(revision->publishedWeightValues));
    CHECK(published(two).IsIdentical(first));
    CHECK(published(again).IsIdentical(first));
    CHECK(built.program->Run(UsdTimeCode(4.0), &four));
    CHECK(four.comparisonMismatches == 0);
    const VtFloatArray moved = published(four);
    CHECK(moved == VtFloatArray(3, 0.5f));
    CHECK(moved.IsIdentical(revision->publishedWeightValues));
    CHECK(!moved.IsIdentical(first));
    CHECK(first == VtFloatArray(3, 1.0f));
}

/// MakeRangeChainStage(10000, true, false) beside a second chain on a
/// four-point target that sorts first ("/Asset/Earlier.points"), moved by
/// E0 and, with \p inserted, also by E1: inserting E1 shifts every revision
/// id and group slot of the range chain without touching its revisions.
UsdStageRefPtr
MakeTwoChainStage(bool inserted)
{
    const UsdStageRefPtr stage = MakeRangeChainStage(10000, true, false);
    const SdfPath earlier("/Asset/Earlier.points");
    stage->DefinePrim(earlier.GetPrimPath(), TfToken("Points"))
        .GetAttribute(TfToken("points"))
        .Set(VtVec3fArray{GfVec3f(0), GfVec3f(1, 0, 0), GfVec3f(0, 1, 0),
                          GfVec3f(0, 0, 1)});
    for (const char *name : {"E0", "E1"}) {
        if (!inserted && std::string(name) == "E1") {
            continue;
        }
        const UsdPrim mover = stage->DefinePrim(
            SdfPath("/Asset/Rig/Movers").AppendChild(TfToken(name)),
            TfToken("RigExecMatrixMover"));
        mover.ApplyAPI(TfToken("RigExecMoverAPI"));
        mover.GetRelationship(TfToken("rigExec:moves")).SetTargets({earlier});
        mover.GetRelationship(TfToken("rigExec:transform"))
            .SetTargets({SdfPath("/Asset/Rig/Moving")});
        mover.GetAttribute(TfToken("inputs:defaultWeight")).Set(1.0f);
    }
    return stage;
}

/// The chain of \p B whose target is \p target, or -1.
int
ChainOf(const RigExecBakedProgramImpl &B, const SdfPath &target)
{
    for (size_t c = 0; c < B.chains.size(); ++c) {
        if (B.chains[c].target == target) {
            return int(c);
        }
    }
    return -1;
}

/// Adoption across a rebuild that renumbers a range chain (a revision
/// inserted in an earlier chain): its groups, versions and group ids are
/// carried, each id remapped to this program's slot of the same writer, so
/// right after the adoption every revision's ids are exactly the content
/// ids its version resolves to here. Run on, it matches a program built
/// fresh bit for bit, and RIGEXEC_VERIFY_CHAIN_VERSIONS finds nothing.
void
TestAnAdoptedRangeChainRemapsItsGroupIds()
{
    const SdfPath shape("/Asset/Shape.points");
    const SdfPath earlier("/Asset/Earlier.points");
    TfSetenv("RIGEXEC_VERIFY_CHAIN_VERSIONS", "1");
    const BuiltProgram outgoing = BuildStage(MakeTwoChainStage(false));
    const BuiltProgram rebuilt = BuildStage(MakeTwoChainStage(true));
    const BuiltProgram fresh = BuildStage(MakeTwoChainStage(true));
    TfSetenv("RIGEXEC_VERIFY_CHAIN_VERSIONS", "0");
    CHECK(outgoing.program && rebuilt.program && fresh.program);
    if (!outgoing.program || !rebuilt.program || !fresh.program) {
        return;
    }
    const RigExecBakedProgramImpl &O = outgoing.program->GetStepGraph();
    const RigExecBakedProgramImpl &R = rebuilt.program->GetStepGraph();
    const RigExecBakedProgramImpl &F = fresh.program->GetStepGraph();
    const int oc = ChainOf(O, shape), rc = ChainOf(R, shape);
    const int fc = ChainOf(F, shape), re = ChainOf(R, earlier);
    CHECK(oc >= 0 && rc >= 0 && fc >= 0 && re >= 0);
    if (oc < 0 || rc < 0 || fc < 0 || re < 0) {
        return;
    }
    // The inserted revision comes first, so the range chain is renumbered.
    CHECK(R.chainRevisionBegin[size_t(re)] < R.chainRevisionBegin[size_t(rc)]);
    CHECK(R.chainRevisionBegin[size_t(rc)] ==
          O.chainRevisionBegin[size_t(oc)] + 1);
    const auto &chain = R.chains[size_t(rc)];
    CHECK(chain.groupBounds.size() >= 3 &&
          chain.groupBounds == O.chains[size_t(oc)].groupBounds);
    RigExecRigPose pose;
    for (const double frame : {1.0, 2.0}) {
        CHECK(outgoing.program->Run(UsdTimeCode(frame), &pose));
        CHECK(pose.comparisonMismatches == 0);
    }
    rebuilt.program->AdoptGeometryStateFrom(*outgoing.program);
    // Every carried id names this program's writer of the group.
    size_t ids = 0;
    for (size_t r = 0; r < chain.revisions.size(); ++r) {
        const auto &revision = chain.revisions[r];
        CHECK(revision.groupIds.size() + 1 == chain.groupBounds.size());
        for (size_t g = 0; g < revision.groupIds.size(); ++g) {
            ++ids;
            if (revision.groupIds[g] !=
                RigExecBakedGroupSourceAt(R, rc, r + 1, g)) {
                ++failures;
                std::printf("FAIL adopted range chain: revision %zu group %zu "
                            "names another writer's slot\n", r, g);
            }
        }
    }
    CHECK(ids > 0);
    RigExecRigPose freshPose;
    for (const double frame : {2.0, 3.0, 1.0, 2.0}) {
        CHECK(rebuilt.program->Run(UsdTimeCode(frame), &pose));
        CHECK(fresh.program->Run(UsdTimeCode(frame), &freshPose));
        CHECK(pose.comparisonMismatches == 0);
        CHECK(SameBits(chain.result, F.chains[size_t(fc)].result));
        CHECK(SameBits(R.chains[size_t(re)].result,
                       F.chains[size_t(ChainOf(F, earlier))].result));
    }
    CHECK(R.chainVersionMismatches == 0);
}

/// Eight points under one classicLinear skin of two joints, J0 (moving at
/// frames 1-3) owning points 0-3 and J1 points 4-7, at half strength: with
/// RIGEXEC_BAKED_CHUNK_VERTS=2 and RIGEXEC_BAKED_GROUP_VERTS=2 the chain is
/// cut into four groups and the skin takes the Range role.
UsdStageRefPtr
MakeRangeSkinStage()
{
    const UsdStageRefPtr stage = UsdStage::CreateInMemory();
    stage->DefinePrim(SdfPath("/Asset"), TfToken("Scope"));
    stage->DefinePrim(SdfPath("/Asset/Rig"), TfToken("RigExecRoot"));
    const UsdPrim j0 = stage->DefinePrim(SdfPath("/Asset/Rig/J0"),
                                         TfToken("RigExecControl"));
    for (int frame = 1; frame <= 3; ++frame) {
        j0.GetAttribute(TfToken("avars:tx"))
            .Set(double(frame) * 10.0, UsdTimeCode(frame));
    }
    const UsdPrim j1 = stage->DefinePrim(SdfPath("/Asset/Rig/J1"),
                                         TfToken("RigExecControl"));
    j1.GetAttribute(TfToken("avars:ty")).Set(20.0);
    const SdfPath target("/Asset/Geom/Mesh.points");
    VtVec3fArray points(8);
    for (size_t i = 0; i < points.size(); ++i) {
        points[i] = GfVec3f(float(i), float(i) * 2.0f, float(i) * 3.0f);
    }
    stage->DefinePrim(target.GetPrimPath(), TfToken("Mesh"))
        .GetAttribute(TfToken("points"))
        .Set(points);
    stage->DefinePrim(SdfPath("/Asset/Rig/Movers"), TfToken("Scope"));
    const UsdPrim skin = stage->DefinePrim(
        SdfPath("/Asset/Rig/Movers/Skin"), TfToken("RigExecSkinMover"));
    skin.ApplyAPI(TfToken("RigExecMoverAPI"));
    skin.GetRelationship(TfToken("rigExec:moves")).SetTargets({target});
    skin.GetAttribute(TfToken("inputs:defaultWeight")).Set(0.5f);
    skin.GetAttribute(TfToken("rigExec:skinningMethod"))
        .Set(TfToken("classicLinear"));
    skin.CreateRelationship(TfToken("rigExec:influences"))
        .SetTargets({j0.GetPath(), j1.GetPath()});
    skin.CreateAttribute(TfToken("rigExec:elementSize"),
                         SdfValueTypeNames->Int).Set(1);
    skin.CreateAttribute(TfToken("rigExec:jointIndices"),
                         SdfValueTypeNames->IntArray)
        .Set(VtIntArray{0, 0, 0, 0, 1, 1, 1, 1});
    skin.CreateAttribute(TfToken("rigExec:jointWeights"),
                         SdfValueTypeNames->FloatArray)
        .Set(VtFloatArray(8, 1.0f));
    return stage;
}

/// A Range skin under the RIGEXEC_VERIFY_RANGE_CHAINS judge, which skins
/// against the folded table and blends the envelope: zero mismatches over
/// the visits, bits equal to the skin built whole, and the judge itself
/// catches one wrong point in a published group.
void
TestARangeSkinAgreesWithTheJudge()
{
    const std::string chunkVerts = TfGetenv("RIGEXEC_BAKED_CHUNK_VERTS");
    const std::string groupVerts = TfGetenv("RIGEXEC_BAKED_GROUP_VERTS");
    TfSetenv("RIGEXEC_BAKED_CHUNK_VERTS", "2");
    TfSetenv("RIGEXEC_BAKED_GROUP_VERTS", "2");
    TfSetenv("RIGEXEC_VERIFY_RANGE_CHAINS", "1");
    const BuiltProgram ranged = BuildStage(MakeRangeSkinStage());
    TfSetenv("RIGEXEC_BAKED_RANGE_CHAINS", "0");
    const BuiltProgram whole = BuildStage(MakeRangeSkinStage());
    TfUnsetenv("RIGEXEC_BAKED_RANGE_CHAINS");
    TfSetenv("RIGEXEC_VERIFY_RANGE_CHAINS", "0");
    const auto restore = [](const char *knob, const std::string &value) {
        if (value.empty()) {
            TfUnsetenv(knob);
        } else {
            TfSetenv(knob, value);
        }
    };
    restore("RIGEXEC_BAKED_CHUNK_VERTS", chunkVerts);
    restore("RIGEXEC_BAKED_GROUP_VERTS", groupVerts);
    CHECK(ranged.program != nullptr && whole.program != nullptr);
    if (!ranged.program || !whole.program) {
        return;
    }
    RigExecBakedProgramImpl &R =
        const_cast<RigExecBakedProgramImpl &>(ranged.program->GetStepGraph());
    const RigExecBakedProgramImpl &W = whole.program->GetStepGraph();
    CHECK(R.verifyRangeChains);
    CHECK(R.chains.size() == 1 && W.chains.size() == 1);
    if (R.chains.size() != 1 || W.chains.size() != 1 ||
        R.chains[0].revisions.size() != 1) {
        return;
    }
    auto &skin = R.chains[0].revisions[0];
    CHECK(skin.op == RigExecRevisionOp::Skin && skin.rangeRole);
    CHECK(R.chains[0].groupBounds.size() == 5);
    for (const double frame : {1.0, 2.0, 3.0, 2.0, 1.0, 3.0}) {
        RigExecRigPose pose, wholePose;
        CHECK(ranged.program->Run(UsdTimeCode(frame), &pose));
        CHECK(whole.program->Run(UsdTimeCode(frame), &wholePose));
        CHECK(pose.comparisonMismatches == 0);
        CHECK(R.chains[0].result.size() == 8);
        CHECK(SameBits(R.chains[0].result, W.chains[0].result));
        CHECK(EveryVersionAgrees(R, W));
        CHECK(pose.diagnostics == wholePose.diagnostics);
    }
    CHECK(R.rangeVerifyMismatches == 0);
    CHECK(R.gateViolations.load() == 0);

    // One wrong point in a published group is a mismatch; put back, none.
    CHECK(skin.groups.size() == 4);
    if (skin.groups.size() != 4 || skin.groups[1].published.count == 0) {
        return;
    }
    RigExecGroupState<GfVec3f> &group = skin.groups[1];
    const RigExecPointsRef<GfVec3f> kept = group.published;
    auto poisoned = std::make_shared<std::vector<GfVec3f>>(
        kept.data, kept.data + kept.count);
    (*poisoned)[0][0] += 1.0f;
    group.published.owner = poisoned;
    group.published.data = poisoned->data();
    {
        TfErrorMark mark;
        CHECK(RigExecBakedVerifyRangeChains(&R) == 1);
        mark.Clear();
    }
    group.published = kept;
    CHECK(RigExecBakedVerifyRangeChains(&R) == 0);
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
    TestIndexedQueriesMatchExhaustiveWitnesses();
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
    TestVerifierComparesExactSkinTopologyContent();
    TestFrozenSparseRawLayoutDigest();
    TestVerifierRestoresAdoptedLayoutsAndBlendCaches();
    TestVerifierRestoresInputsAndChecksSelectedWork();
    TestProviderRefreshFailureBoundaries();
    TestTheValidatorRejectsAMalformedGraph();
    TestTheHeadValidatorRejectsAMalformedTier(biped);
    TestAPlacementReadNeedsItsVolumesStep();
    TestTheValidatorRejectsALaterPoseVersion();
    TestTheValidatorRejectsAnUnboundPointVersion();
    TestTheValidatorRejectsABadFrameRecord(examplesDir + "/../tests/fixtures");
    TestTheValidatorRejectsABadSolverRecord(examplesDir + "/../tests/fixtures");
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
    TestARecutLayoutCannotReadAnUndeclaredJoint();
    TestAStaleFuseKeepsTheChunkKeys();
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
    {
        // Range-pipelined point chains: a chain above the vertex target cut
        // into ranges, its Matrix revisions run as one step per range and a
        // join. The count comes from the default value, or from the earliest
        // sample when there is none.
        const BuiltProgram ranged =
            BuildStage(MakeRangeChainStage(10000, false, false));
        const BuiltProgram sampled =
            BuildStage(MakeRangeChainStage(10000, false, false, true));
        TestARangeChainIsCutIntoRanges(ranged, "range_chain");
        TestARangeChainIsCutIntoRanges(sampled, "range_chain (samples only)");
        TestTheGraphDescribesTheProgram(ranged, "range_chain");
        TestTheValidatorAcceptsTheProgram(ranged, "range_chain");
        TestEachChainReaderBindsOneVersion(ranged, "range_chain");
        TestThePointVersionsKeepTheOrder(ranged, "range_chain");
        TestTheClusteringIsSound(ranged, "range_chain");
        TestTheConeClosuresAreSound(ranged, "range_chain");
        TestTheValidatorRejectsABrokenRangeChain();
        TestARangeChainMatchesTheWholeChain();
        TestARangeChainCutsOffUnmovedRanges();
        TestRangeJoinsKeepTheLineOrder();
    }
    // Chain buffers flip rather than copy, and point values key by content
    // version: buffer selection across a failure, a version an unmoved edit
    // keeps, and versions carried across a rebuild.
    TestTheFuseSelectsItsBuffersByHand();
    TestTheVerifierComparesOnlyLiveStaging();
    TestTheVerifierLeavesStagingToASkippedFuse();
    TestTheVerifierRestoresTheChainInputVersion();
    TestAChainRevisionRecoversAcrossAFailure();
    TestUnmovedPointsKeepTheirVersion();
    TestTheEpilogueVisitsTheStepsHoldingLines();
    // RevisionStatic's weight field reused by its packet's revision and
    // keyed, with the envelope, by content version.
    TestTheWeightOverlayIsReusedByVersion();
    TestADefaultOnlyMoveResolvesTheWeightField();
    {
        const std::string fixtures = examplesDir + "/../tests/fixtures";
        size_t kept = 0;
        for (const char *stage :
             {"raw_skin_layouts.usda", "oneloop_two_limbs.usda"}) {
            kept += TestARebuildKeepsItsPointVersions(fixtures + "/" + stage,
                                                      stage);
        }
        CHECK(kept > 0);
    }
    TestTheDerivedCompareAgreesWithTheElementwiseOne(
        examplesDir + "/biped/Biped.usda");
    TestARebuiltProgramKeepsItsRunState(
        examplesDir + "/biped/Biped.usda", "Biped");
    TestARebuiltProgramKeepsItsRunState(
        examplesDir + "/04_BlendShapeFace.usda", "04_BlendShapeFace");
    {
        const std::string fixtures = examplesDir + "/../tests/fixtures";
        size_t retained = 0;
        for (const char *stage : {"raw_skin_layouts.usda", "oneloop_two_limbs.usda",
                                  "upstream_inputs.usda"}) {
            retained += TestARetainedSkinOpOutlivesItsLeafVersions(
                fixtures + "/" + stage, stage);
        }
        retained += TestARetainedSkinOpOutlivesItsLeafVersions(
            examplesDir + "/biped/Biped.usda", "Biped");
        CHECK(retained > 0);
    }
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
    {
        // Chains cut into vertex groups: the status step's gather, whole
        // readers, weight-field sharing, adoption across a renumbering, and
        // the Range-skin judge.
        TestTheChainStatusKeepsAnUnmovedResult();
        TestTheWeightFieldIsSharedNotCopied();
        TestAnAdoptedRangeChainRemapsItsGroupIds();
        TestARangeSkinAgreesWithTheJudge();
    }
    if (failures) {
        std::printf("%d FAILURE(S)\n", failures);
        return 1;
    }
    std::printf("testRigExecBakedSchedule: all tests passed\n");
    return 0;
}
