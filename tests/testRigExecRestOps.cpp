// The rest chain and the default-space ladder as head-tier ops.
// The baked prologue composes them per compose group (RestCompose writes
// the rests, LadderCompose the default-space ladder), each op re-run only
// when a ladder channel it reads, or a rest or ladder it reads, moved; the
// region steps that index a rest or a ladder declare it and are re-run by
// its move. These cases hold the ten exported tables to the single
// slot-order loop the ops were split from, bit for bit, on every example
// and fixture rig, the biped and the epoch-rest stages, under no override,
// a rest drag and a default drag; check that every rest reader declares
// what it indexes; pin that a rest that moves alone, under an authored
// default:space, re-runs its matrices and not its compose; refuse a tier
// that reads a rest before it is written and a step that reads a rest no op
// writes, and a rest op ordered before a property revision; and seed a
// rest edit through the head tier's closure, so the output-affected index
// reaches a child that only the closure reaches.
// Registered plain and under the parity entries; under
// RIGEXEC_BAKED_VERIFY_CONES the rest tier is also checked against a forced
// run of itself.
// argv[1] = path to the examples directory.
#include "rigExec/bakedProgram.h"
#include "rigExec/bakedProgramImpl.h"
#include "rigExec/bakedSchedule.h"
#include "rigExec/bakedTrace.h"
#include "rigExec/outputAffectedIndex.h"
#include "rigExec/rigEvaluator.h"

#include "pxr/base/gf/matrix4d.h"
#include "pxr/base/plug/registry.h"
#include "pxr/base/tf/fileUtils.h"
#include "pxr/base/tf/pathUtils.h"
#include "pxr/base/tf/stringUtils.h"
#include "pxr/usd/sdf/types.h"
#include "pxr/usd/usd/attribute.h"
#include "pxr/usd/usd/editTarget.h"
#include "pxr/usd/usd/prim.h"
#include "pxr/usd/usd/primRange.h"
#include "pxr/usd/usd/relationship.h"
#include "pxr/usd/usd/stage.h"

#include <algorithm>
#include <cstdio>
#include <cstring>
#include <functional>
#include <map>
#include <memory>
#include <set>
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

namespace {

// Head entries are part of the ordinary execution trace.
static std::vector<RigExecOpTraceEntry>
ExecutedHeads(const RigExecBakedProgramImpl &B)
{
    auto trace = RigExecBakedLastRunTrace(B);
    trace.erase(std::remove_if(trace.begin(),trace.end(),
        [&B](const auto &entry) { return !B.steps[entry.step].isHead; }),trace.end());
    return trace;
}


using Tables = RigExecBakedProgramTesting::LadderTables;

template <class T>
bool
Bits(const T &a, const T &b)
{
    return std::memcmp(&a, &b, sizeof(T)) == 0;
}

// The first slot at which \p got differs from \p want in any of the ten
// tables, bit for bit, or -1.
int
FirstDifference(const Tables &want, const Tables &got, std::string *table)
{
    const size_t n = want.restM.size();
    if (got.restM.size() != n) {
        *table = "size";
        return 0;
    }
    for (size_t s = 0; s < n; ++s) {
        const char *name = nullptr;
        if (!Bits(want.restM[s], got.restM[s])) {
            name = "restM";
        } else if (!Bits(want.restPts[s], got.restPts[s])) {
            name = "restPts";
        } else if (want.restFrames[s].flags != got.restFrames[s].flags ||
                   !Bits(want.restFrames[s].points,
                         got.restFrames[s].points)) {
            name = "restFrames";
        } else if (!Bits(want.restRoundTrip[s], got.restRoundTrip[s])) {
            name = "restRoundTrip";
        } else if (!Bits(want.selfD[s], got.selfD[s])) {
            name = "selfD";
        } else if (!Bits(want.parentDinv[s], got.parentDinv[s])) {
            name = "parentDinv";
        } else if (!Bits(want.defaultRoundTrip[s], got.defaultRoundTrip[s])) {
            name = "defaultRoundTrip";
        } else if (want.rotOrder[s] != got.rotOrder[s]) {
            name = "rotOrder";
        } else if (want.posedAuthored[s] != got.posedAuthored[s]) {
            name = "posedAuthored";
        } else if (!Bits(want.posedAuthoredM[s], got.posedAuthoredM[s])) {
            name = "posedAuthoredM";
        }
        if (name) {
            *table = name;
            return int(s);
        }
    }
    return -1;
}

SdfPath
FindRig(const UsdStageRefPtr &stage)
{
    for (const UsdPrim &prim : stage->Traverse()) {
        if (prim.GetTypeName() == TfToken("RigExecRoot")) {
            return prim.GetPath();
        }
    }
    return SdfPath();
}

double
StartOf(const UsdStageRefPtr &stage)
{
    return stage->HasAuthoredTimeCodeRange() ? stage->GetStartTimeCode()
                                              : 1.0;
}

void
EditInSession(const UsdStageRefPtr &stage)
{
    stage->SetEditTarget(UsdEditTarget(stage->GetSessionLayer()));
}

const RigExecBakedProgramImpl *
Program(const RigExecRigEvaluator &evaluator)
{
    const RigExecBakedProgram *program = evaluator.GetBakedProgram();
    return program ? &program->GetStepGraph() : nullptr;
}

// An override of ladder channel \p channel ("rest:tx", "default:tx") on
// the first provider with a parent whose channel is authored, moved by
// \p delta from its leaf. False when no provider authors it.
bool
LadderDrag(const RigExecBakedProgramImpl &B, bool rest, double delta,
           RigExecValueOverride *out)
{
    // A provider with a parent first, so the drag reaches a child's rest;
    // any authored one otherwise.
    int chosen = -1;
    for (const bool needParent : {true, false}) {
        for (size_t slot = 0; slot < B.paths.size() && chosen < 0; ++slot) {
            if (B.slotKind[slot] != RigExecBakedSlotKind::FirstFramePose ||
                (needParent && B.parent[slot] < 0)) {
                continue;
            }
            const RigExecBakedProgramImpl::Ladder &L = B.ladders[slot];
            const RigExecBakedInput<double> &input =
                rest ? L.restAvars[0] : L.defaultAvars[0];
            if (input.head && input.overrideIndex >= 0) {
                chosen = int(slot);
            }
        }
    }
    if (chosen < 0) {
        return false;
    }
    const RigExecBakedProgramImpl::Ladder &L = B.ladders[size_t(chosen)];
    const double value = RigExecBakedLeafRead(
        B, rest ? L.restAvars[0] : L.defaultAvars[0]);
    *out = RigExecValueOverride{B.paths[size_t(chosen)], TfToken(),
                                TfToken(rest ? "rest:tx" : "default:tx"),
                                VtValue(value + delta)};
    return true;
}

// One rig's memo tables against an independently compiled fresh original epoch.
size_t
CheckTablesOfRig(const std::string &name, const UsdStageRefPtr &stage,
                 const SdfPath &rig)
{
    RigExecRigEvaluator evaluator(stage, rig);
    std::vector<std::string> errors;
    if (!evaluator.Compile(&errors) || !evaluator.IsBakeable()) {
        return 0;
    }
    evaluator.SetEvaluationMode(RigExecEvaluationMode::Baked);
    const double start = StartOf(stage);
    const double frames[] = {start, start + 2.0, start + 6.0};
    size_t compared = 0;
    double currentTime = start;
    std::vector<RigExecValueOverride> currentDrags;
    const auto check = [&](const std::string &when) {
        const RigExecBakedProgram *program = evaluator.GetBakedProgram();
        CHECK(program);
        if (!program) {
            return;
        }
        RigExecRigEvaluator fresh(stage, rig);
        CHECK(fresh.Compile());
        fresh.SetEvaluationMode(RigExecEvaluationMode::Baked);
        fresh.SetInteractiveOverrides(currentDrags);
        CHECK(fresh.Evaluate(UsdTimeCode(currentTime)).valid);
        const auto *freshProgram = fresh.GetBakedProgram();
        CHECK(freshProgram);
        if (!freshProgram) return;
        const Tables want = RigExecBakedProgramTesting::LadderTablesOf(*freshProgram);
        const Tables got = RigExecBakedProgramTesting::LadderTablesOf(*program);
        std::string table;
        const int slot = FirstDifference(want, got, &table);
        if (slot >= 0) {
            ++failures;
            std::printf("FAIL %s %s: %s differs from the fresh epoch at "
                        "%s\n",
                        name.c_str(), when.c_str(), table.c_str(),
                        program->GetStepGraph().paths[size_t(slot)].GetText());
        }
        ++compared;
    };
    std::vector<RigExecValueOverride> drags[3];
    for (int state = 0; state < 3; ++state) {
        if (state > 0) {
            const RigExecBakedProgramImpl *B = Program(evaluator);
            RigExecValueOverride drag;
            if (!B || !LadderDrag(*B, state == 1, state == 1 ? 0.5 : 0.25,
                                  &drag)) {
                continue;
            }
            currentDrags={drag};
            evaluator.SetInteractiveOverrides(currentDrags);
        }
        for (const double frame : frames) {
            currentTime=frame;
            CHECK(evaluator.Evaluate(UsdTimeCode(frame)).valid);
            check((state == 0 ? "no override" : state == 1 ? "rest drag"
                                                           : "default drag") +
                  std::string(" at ") + TfStringify(frame));
        }
    }
    evaluator.ClearInteractiveOverrides();
    currentDrags.clear();
    currentTime=frames[0];
    CHECK(evaluator.Evaluate(UsdTimeCode(frames[0])).valid);
    check("released");
    return compared;
}

// The ten memoized tables equal the independently compiled fresh epoch
// bit for bit, on every example and fixture rig
// that bakes, the biped, and the tail with an animated rest, a connected
// rest and a property chain on its rest space.
void
TestTheRestTablesMatchFreshEpochs(const std::string &examples)
{
    std::vector<std::string> files;
    for (const std::string &dir :
         {examples, examples + "/../tests/fixtures"}) {
        for (const std::string &name : TfListDir(dir)) {
            if (TfStringEndsWith(name, ".usda")) {
                files.push_back(name);
            }
        }
    }
    files.push_back(examples + "/biped/Biped_anim.usda");
    std::sort(files.begin(), files.end());
    size_t rigs = 0, compared = 0;
    for (const std::string &file : files) {
        UsdStageRefPtr stage = UsdStage::Open(file);
        if (!stage) {
            continue;
        }
        const SdfPath rig = FindRig(stage);
        if (rig.IsEmpty()) {
            continue;
        }
        const size_t n = CheckTablesOfRig(TfGetBaseName(file), stage, rig);
        rigs += n > 0 ? 1 : 0;
        compared += n;
    }
    // The epoch-rest stages: a ladder that recomposes every frame.
    const SdfPath tailRig("/TailAsset/Rig");
    const SdfPath joint("/TailAsset/Rig/Joints/Seg1/Seg2");
    const std::vector<std::pair<std::string,
                                std::function<void(const UsdStageRefPtr &)>>>
        edits = {
            {"tail with a time-sampled rest:tx",
             [&joint](const UsdStageRefPtr &stage) {
                 const UsdAttribute restTx =
                     stage->GetPrimAtPath(joint).CreateAttribute(
                         TfToken("rest:tx"), SdfValueTypeNames->Double);
                 restTx.Set(0.0, UsdTimeCode(1001.0));
                 restTx.Set(3.0, UsdTimeCode(1003.0));
                 restTx.Set(6.0, UsdTimeCode(1007.0));
             }},
            {"tail with a connected rest:tx",
             [&joint](const UsdStageRefPtr &stage) {
                 const UsdPrim prim = stage->GetPrimAtPath(joint);
                 const UsdAttribute driver = prim.CreateAttribute(
                     TfToken("inputs:restDriver"), SdfValueTypeNames->Double);
                 driver.Set(1.0, UsdTimeCode(1001.0));
                 driver.Set(4.0, UsdTimeCode(1007.0));
                 prim.CreateAttribute(TfToken("rest:tx"),
                                      SdfValueTypeNames->Double)
                     .SetConnections({driver.GetPath()});
             }},
            {"tail with a property chain on rest:space",
             [&tailRig, &joint](const UsdStageRefPtr &stage) {
                 const SdfPath mover = tailRig.AppendChild(TfToken("Movers"))
                                           .AppendChild(TfToken("RestOffset"));
                 const UsdPrim prim = stage->DefinePrim(
                     mover, TfToken("RigExecMatrixMathMover"));
                 prim.ApplyAPI(TfToken("RigExecMoverAPI"));
                 prim.CreateAttribute(TfToken("rigExec:operation"),
                                      SdfValueTypeNames->Token)
                     .Set(TfToken("multiply"));
                 GfMatrix4d offset(1.0);
                 offset[3][0] = 3.0;
                 prim.CreateAttribute(TfToken("inputs:value"),
                                      SdfValueTypeNames->Matrix4d)
                     .Set(offset);
                 prim.CreateAttribute(TfToken("inputs:defaultWeight"),
                                      SdfValueTypeNames->Float)
                     .Set(1.0f);
                 prim.CreateRelationship(TfToken("rigExec:moves"))
                     .SetTargets(
                         {joint.AppendProperty(TfToken("rest:space"))});
             }},
        };
    for (const auto &[name, edit] : edits) {
        UsdStageRefPtr stage =
            UsdStage::Open(examples + "/01_FkChainTail.usda");
        CHECK(stage);
        if (!stage) {
            continue;
        }
        EditInSession(stage);
        edit(stage);
        const size_t n = CheckTablesOfRig(name, stage, tailRig);
        CHECK(n > 0);
        rigs += n > 0 ? 1 : 0;
        compared += n;
    }
    std::printf("ladder loops: %zu rig(s), %zu run(s) compared\n", rigs,
                compared);
    CHECK(rigs > 20);
}

// Whether \p step declares head slot \p slot of \p domain.
bool
Declares(const RigExecBakedStep &step, RigExecBakedSlotDomain domain,
         int slot)
{
    for (const RigExecBakedSlotRange &range : step.reads) {
        if (range.domain == domain && range.begin <= uint32_t(slot) &&
            uint32_t(slot) < range.end) {
            return true;
        }
    }
    return false;
}

// Every ProviderMatrix, FrameMatrix, PoseInterpolator, Constraint (IK and
// matrix mover) and Solve step lists the rests it indexes, every compose its
// group's ladders and every constraint its space's ladder; and every head
// output a step declares has a producer.
void
TestEveryRestReaderDeclaresIt(const std::string &examples)
{
    const std::vector<std::string> files = {
        examples + "/biped/Biped_anim.usda",
        examples + "/02_TwoBoneIkLeg.usda",
        examples + "/03_IkFkBlendClamp.usda",
        examples + "/05_TwistRibbonSpine.usda",
        examples + "/08_AimEyes.usda",
        examples + "/15_TransformMatrixMover.usda",
        examples + "/ArmRig.usda",
        examples + "/../tests/fixtures/solver_checkpoint.usda",
        examples + "/../tests/fixtures/computed_ik_space.usda",
        examples + "/../tests/fixtures/frame_record_fallbacks.usda",
        examples + "/../tests/fixtures/projector_spaces.usda",
        examples + "/../tests/fixtures/space_switch_carry.usda",
    };
    std::map<std::string, size_t> counts;
    for (const std::string &file : files) {
        UsdStageRefPtr stage = UsdStage::Open(file);
        if (!stage) {
            continue;
        }
        const SdfPath rig = FindRig(stage);
        RigExecRigEvaluator evaluator(stage, rig);
        std::vector<std::string> errors;
        if (rig.IsEmpty() || !evaluator.Compile(&errors) ||
            !evaluator.IsBakeable()) {
            continue;
        }
        evaluator.SetEvaluationMode(RigExecEvaluationMode::Baked);
        CHECK(evaluator.Evaluate(UsdTimeCode(StartOf(stage))).valid);
        const RigExecBakedProgramImpl *program = Program(evaluator);
        CHECK(program);
        if (!program) {
            continue;
        }
        const RigExecBakedProgramImpl &B = *program;
        std::string invalid;
        CHECK(RigExecBakedValidateHeadReads(B, &invalid));
        const std::string name = TfGetBaseName(file);
        const auto rest = [&](const RigExecBakedStep &step, int slot,
                              const char *kind) {
            if (slot < 0) {
                return;
            }
            ++counts[kind];
            if (!Declares(step, RigExecBakedSlotDomain::Rest, slot)) {
                ++failures;
                std::printf("FAIL %s: %s does not declare Rest[%s]\n",
                            name.c_str(), step.label.c_str(),
                            B.paths[size_t(slot)].GetText());
            }
        };
        const auto ladder = [&](const RigExecBakedStep &step, int slot,
                                const char *kind) {
            if (slot < 0) {
                return;
            }
            ++counts[kind];
            if (!Declares(step, RigExecBakedSlotDomain::Ladder, slot)) {
                ++failures;
                std::printf("FAIL %s: %s does not declare Ladder[%s]\n",
                            name.c_str(), step.label.c_str(),
                            B.paths[size_t(slot)].GetText());
            }
        };
        for (const RigExecBakedStep &step : B.steps) {
            switch (step.kind) {
            case RigExecBakedStepKind::ProviderMatrix:
                rest(step, step.object, "ProviderMatrix");
                break;
            case RigExecBakedStepKind::FrameMatrix:
                rest(step, B.frameRecords[size_t(step.object)].slot,
                     "FrameMatrix");
                break;
            case RigExecBakedStepKind::PoseInterpolator: {
                const auto &interpolator =
                    B.poseInterpolators[size_t(step.object)];
                if (interpolator.valueInputs.empty()) {
                    rest(step, interpolator.driverSlot, "PoseInterpolator");
                    rest(step, interpolator.parentSlot, "PoseInterpolator");
                }
                break;
            }
            case RigExecBakedStepKind::Solve:
                for (const int slot :
                     B.solvers[size_t(step.object)].restSlots) {
                    rest(step, slot, "Solve");
                }
                break;
            case RigExecBakedStepKind::Constraint: {
                const auto &walk = B.walkSteps[size_t(step.object)];
                if (walk.solverBatch || walk.index < 0) {
                    break;
                }
                const auto &c = B.constraints[size_t(walk.index)];
                if (!c.useAnimatedTs) {
                    for (const int slot : c.targetSlots) {
                        rest(step, slot, "Constraint");
                    }
                }
                if (c.type == "RigExecMatrixMover") {
                    for (const int slot : c.sources) {
                        rest(step, slot, "MatrixMover");
                    }
                }
                ladder(step, c.spaceSlot, "ConstraintSpace");
                break;
            }
            case RigExecBakedStepKind::ComposeSubtree: {
                const RigExecBakedComposeGroup &group =
                    B.composeGroups[size_t(step.object)];
                for (int slot = group.begin; slot < group.end; ++slot) {
                    ladder(step, slot, "ComposeSubtree");
                }
                break;
            }
            default:
                break;
            }
        }
    }
    for (const auto &[kind, count] : counts) {
        std::printf("rest readers: %s %zu read(s)\n", kind.c_str(), count);
    }
    CHECK(counts["ProviderMatrix"] > 0);
    CHECK(counts["Solve"] > 0);
    CHECK(counts["Constraint"] > 0);
    CHECK(counts["ComposeSubtree"] > 0);
    CHECK(counts["FrameMatrix"] > 0);
}

// /Asset/Rig with a control and the joint chain Root/Child/Leaf; \p shape
// authors what each case needs on the joints.
UsdStageRefPtr
JointRig(const std::function<void(const UsdStageRefPtr &)> &shape)
{
    UsdStageRefPtr stage = UsdStage::CreateInMemory();
    stage->SetStartTimeCode(1.0);
    stage->SetEndTimeCode(3.0);
    stage->DefinePrim(SdfPath("/Asset"), TfToken("Xform"));
    stage->DefinePrim(SdfPath("/Asset/Rig"), TfToken("RigExecRoot"));
    stage->DefinePrim(SdfPath("/Asset/Rig/Controls"), TfToken("Scope"));
    stage->DefinePrim(SdfPath("/Asset/Rig/Controls/Ctl"),
                      TfToken("RigExecControl"))
        .CreateAttribute(TfToken("avars:tx"), SdfValueTypeNames->Double)
        .Set(0.25);
    stage->DefinePrim(SdfPath("/Asset/Rig/Joints"), TfToken("Scope"));
    for (const char *path :
         {"/Asset/Rig/Joints/Root", "/Asset/Rig/Joints/Root/Child",
          "/Asset/Rig/Joints/Root/Child/Leaf"}) {
        stage->DefinePrim(SdfPath(path), TfToken("RigExecJoint"))
            .CreateAttribute(TfToken("rest:tx"), SdfValueTypeNames->Double)
            .Set(1.0);
    }
    shape(stage);
    return stage;
}

GfMatrix4d
Offset(double x, double y, double z)
{
    GfMatrix4d m(1.0);
    m.SetTranslateOnly(GfVec3d(x, y, z));
    return m;
}

void
SetDefaultSpace(const UsdStageRefPtr &stage, const char *joint,
                const GfMatrix4d &space)
{
    stage->GetPrimAtPath(SdfPath(joint))
        .CreateAttribute(TfToken("default:space"),
                         SdfValueTypeNames->Matrix4d)
        .Set(space);
}

// The baked pose against the reference walk's, bit for bit, on the
// published joint frames and matrices.
void
CompareWithReference(const std::string &where, const RigExecRigPose &baked,
                     const RigExecRigPose &reference)
{
    CHECK(baked.valid && reference.valid);
    const auto frames = [&](const auto &want, const auto &got,
                            const char *what) {
        CHECK(want.size() == got.size());
        for (const auto &[path, value] : want) {
            const auto found = got.find(path);
            if (found == got.end() ||
                !Bits(found->second.points, value.points)) {
                ++failures;
                std::printf("FAIL %s: %s of %s differs from the reference\n",
                            where.c_str(), what, path.GetText());
            }
        }
    };
    frames(reference.jointFramesFinal, baked.jointFramesFinal, "final frame");
    frames(reference.jointFramesBase, baked.jointFramesBase, "base frame");
    CHECK(reference.jointMatricesFinal.size() ==
          baked.jointMatricesFinal.size());
    for (const auto &[path, matrix] : reference.jointMatricesFinal) {
        const auto found = baked.jointMatricesFinal.find(path);
        if (found == baked.jointMatricesFinal.end() ||
            !Bits(found->second, matrix)) {
            ++failures;
            std::printf("FAIL %s: final matrix of %s differs from the "
                        "reference\n",
                        where.c_str(), path.GetText());
        }
    }
}

int
SlotOf(const RigExecBakedProgramImpl &B, const SdfPath &path)
{
    for (size_t slot = 0; slot < B.paths.size(); ++slot) {
        if (B.paths[slot] == path) {
            return int(slot);
        }
    }
    return -1;
}

// The head step of kind \p kind whose compose group holds \p slot.
int
HeadOpOf(const RigExecBakedProgramImpl &B, RigExecBakedStepKind kind,
         int slot)
{
    for (size_t i = 0; i < RigExecBakedHeadIndices(B).size(); ++i) {
        const RigExecBakedStep &step = B.steps[i];
        if (step.kind == kind) {
            const RigExecBakedComposeGroup &group =
                B.composeGroups[size_t(step.object)];
            if (group.begin <= slot && slot < group.end) {
                return int(i);
            }
        }
    }
    return -1;
}

bool
HeadRan(const RigExecBakedProgramImpl &B, int op)
{
    for (const RigExecOpTraceEntry &entry : ExecutedHeads(B)) {
        if (int(entry.step) == op) {
            return true;
        }
    }
    return false;
}

bool
RegionRan(const RigExecBakedProgramImpl &B, size_t step)
{
    for (const RigExecOpTraceEntry &entry : RigExecBakedLastRunTrace(B)) {
        if (entry.step == step) {
            return true;
        }
    }
    return false;
}

// A leaf joint with an animated rest:tx and an authored non-identity
// default:space: between frames its RestCompose runs and its ProviderMatrix
// steps run, and the published matrix moves; its LadderCompose answers the
// same ladder, so its compose group does not run. Equal to the reference
// walk at every frame.
void
TestARestOnlyMoveSkipsTheCompose()
{
    const SdfPath leafPath("/Asset/Rig/Joints/Root/Child/Leaf");
    UsdStageRefPtr stage = JointRig([](const UsdStageRefPtr &stage) {
        const UsdAttribute restTx =
            stage->GetPrimAtPath(SdfPath("/Asset/Rig/Joints/Root/Child/Leaf"))
                .GetAttribute(TfToken("rest:tx"));
        restTx.Set(0.0, UsdTimeCode(1.0));
        restTx.Set(2.0, UsdTimeCode(2.0));
        restTx.Set(5.0, UsdTimeCode(3.0));
        SetDefaultSpace(stage, "/Asset/Rig/Joints/Root/Child/Leaf",
                        Offset(0.0, 0.0, 1.5));
    });
    RigExecRigEvaluator baked(stage, SdfPath("/Asset/Rig"));
    RigExecRigEvaluator reference(stage, SdfPath("/Asset/Rig"));
    std::vector<std::string> errors;
    CHECK(baked.Compile(&errors));
    CHECK(reference.Compile(&errors));
    std::vector<std::string> reasons;
    CHECK(baked.IsBakeable(&reasons));
    for (const std::string &reason : reasons) {
        std::printf("    unexpected refusal: %s\n", reason.c_str());
    }
    baked.SetEvaluationMode(RigExecEvaluationMode::Baked);
    reference.SetEvaluationMode(RigExecEvaluationMode::ExecReference);
    RigExecRigPose last = baked.Evaluate(UsdTimeCode(1.0));
    CompareWithReference("rest-only frame 1", last,
                         reference.Evaluate(UsdTimeCode(1.0)));
    const RigExecBakedProgramImpl *program = Program(baked);
    CHECK(program);
    if (!program) {
        return;
    }
    const RigExecBakedProgramImpl &B = *program;
    const int leaf = SlotOf(B, leafPath);
    CHECK(leaf >= 0);
    if (leaf < 0) {
        return;
    }
    const int restOp = HeadOpOf(B, RigExecBakedStepKind::RestCompose, leaf);
    const int ladderOp =
        HeadOpOf(B, RigExecBakedStepKind::LadderCompose, leaf);
    CHECK(restOp >= 0 && ladderOp >= 0);
    std::vector<size_t> composes, matrices;
    for (size_t s = 0; s < B.steps.size(); ++s) {
        const RigExecBakedStep &step = B.steps[s];
        if (step.kind == RigExecBakedStepKind::ComposeSubtree) {
            const RigExecBakedComposeGroup &group =
                B.composeGroups[size_t(step.object)];
            if (group.begin <= leaf && leaf < group.end) {
                composes.push_back(s);
            }
        } else if (step.kind == RigExecBakedStepKind::ProviderMatrix &&
                   step.object == leaf) {
            matrices.push_back(s);
        }
    }
    CHECK(composes.size() == 1);
    CHECK(!matrices.empty());
    for (const double frame : {2.0, 3.0, 1.0}) {
        const RigExecRigPose pose = baked.Evaluate(UsdTimeCode(frame));
        const std::string where = "rest-only frame " + TfStringify(frame);
        CompareWithReference(where, pose,
                             reference.Evaluate(UsdTimeCode(frame)));
        CHECK(HeadRan(B, restOp));
        CHECK(B.restChanged[size_t(leaf)] == 1);
        CHECK(B.ladderChanged[size_t(leaf)] == 0);
        for (const size_t s : composes) {
            if (RegionRan(B, s)) {
                ++failures;
                std::printf("FAIL %s: %s ran for a rest-only move\n",
                            where.c_str(), B.steps[s].label.c_str());
            }
        }
        for (const size_t s : matrices) {
            CHECK(RegionRan(B, s));
        }
        const auto before = last.jointMatricesFinal.find(leafPath);
        const auto after = pose.jointMatricesFinal.find(leafPath);
        CHECK(before != last.jointMatricesFinal.end() &&
              after != pose.jointMatricesFinal.end());
        if (before != last.jointMatricesFinal.end() &&
            after != pose.jointMatricesFinal.end()) {
            CHECK(before->second != after->second);
        }
        const auto frameBefore = last.jointFramesFinal.find(leafPath);
        const auto frameAfter = pose.jointFramesFinal.find(leafPath);
        if (frameBefore != last.jointFramesFinal.end() &&
            frameAfter != pose.jointFramesFinal.end()) {
            CHECK(Bits(frameBefore->second.points, frameAfter->second.points));
        }
        last = pose;
    }
    // The same frame again: nothing moved, so no rest op runs.
    baked.Evaluate(UsdTimeCode(1.0));
    CHECK(!HeadRan(B, restOp));
    CHECK(!HeadRan(B, ladderOp));
}

// The validator refuses a LadderCompose that reads a rest a later head step
// writes, and a Solve whose rest slot no RestCompose writes.
void
TestTheValidatorRefusesAMisorderedRest(const std::string &examples)
{
    UsdStageRefPtr stage = UsdStage::Open(examples + "/02_TwoBoneIkLeg.usda");
    CHECK(stage);
    if (!stage) {
        return;
    }
    RigExecRigEvaluator evaluator(stage, FindRig(stage));
    std::vector<std::string> errors;
    CHECK(evaluator.Compile(&errors));
    evaluator.SetEvaluationMode(RigExecEvaluationMode::Baked);
    CHECK(evaluator.Evaluate(UsdTimeCode(StartOf(stage))).valid);
    const RigExecBakedProgramImpl *program = Program(evaluator);
    CHECK(program);
    if (!program) {
        return;
    }
    // Edited in place and put back: the program is the evaluator's.
    RigExecBakedProgramImpl &B =
        const_cast<RigExecBakedProgramImpl &>(*program);
    std::string error;
    CHECK(RigExecBakedValidateHeadTier(B, &error));
    CHECK(RigExecBakedValidateHeadReads(B, &error));
    {
        // The first LadderCompose in head order reads the rest of a slot
        // whose RestCompose comes after it.
        int ladder = -1, laterRest = -1;
        for (const uint32_t index : RigExecBakedHeadIndices(B)) {
            const RigExecBakedStep &step = B.steps[index];
            if (ladder < 0 &&
                step.kind == RigExecBakedStepKind::LadderCompose) {
                ladder = int(index);
            } else if (ladder >= 0 &&
                       step.kind == RigExecBakedStepKind::RestCompose) {
                laterRest = int(index);
            }
        }
        CHECK(ladder >= 0 && laterRest >= 0);
        if (ladder >= 0 && laterRest >= 0) {
            RigExecBakedStep &step = B.steps[size_t(ladder)];
            const uint32_t slot =
                B.steps[size_t(laterRest)].writes.front().begin;
            step.reads.push_back(
                RigExecBakedOne(RigExecBakedSlotDomain::Rest, slot));
            error.clear();
            CHECK(!RigExecBakedValidateHeadTier(B, &error));
            const std::string expected =
                "head step " + step.label + " reads Rest slot " +
                std::to_string(slot) + ", which no earlier head step writes";
            if (error.find(expected) == std::string::npos) {
                ++failures;
                std::printf("FAIL \"%s\" is not in \"%s\"\n",
                            expected.c_str(), error.c_str());
            }
            std::printf("  a ladder reading a later rest: %s\n",
                        error.c_str());
            step.reads.pop_back();
        }
    }
    {
        // Isolate each actual compose-body dependency by removing exactly
        // one slot, leaving every other read and all writers intact.
        const auto omit = [&](RigExecBakedStep &step, RigExecBakedSlotDomain domain,
                              uint32_t slot) {
            const auto saved = step.reads;
            std::vector<RigExecBakedSlotRange> kept;
            bool removed = false;
            for (const auto &range : saved) {
                if (range.domain != domain || slot < range.begin || slot >= range.end) {
                    kept.push_back(range); continue;
                }
                removed = true;
                if (range.begin < slot) kept.push_back({domain, range.begin, slot});
                if (slot + 1 < range.end) kept.push_back({domain, slot + 1, range.end});
            }
            CHECK(removed);
            step.reads = std::move(kept);
            error.clear();
            CHECK(!RigExecBakedValidateHeadTier(B, &error));
            CHECK(error.find("head step " + step.label + " omits required " +
                RigExecBakedSlotDomainName(domain) + " slot " + std::to_string(slot))
                != std::string::npos);
            step.reads = saved;
            CHECK(RigExecBakedValidateHeadTier(B, &error));
        };
        bool restParent = false, ladderParentRest = false,
             ladderOwnRest = false, ladderParent = false;
        for (uint32_t index : RigExecBakedHeadIndices(B)) {
            auto &step = B.steps[index];
            const bool rest = step.kind == RigExecBakedStepKind::RestCompose;
            const bool ladder = step.kind == RigExecBakedStepKind::LadderCompose;
            if (!rest && !ladder) continue;
            const auto &group = B.composeGroups[size_t(step.object)];
            if (ladder && !ladderOwnRest) {
                omit(step, RigExecBakedSlotDomain::Rest, uint32_t(group.begin));
                ladderOwnRest = true;
            }
            for (int slot = group.begin; slot < group.end; ++slot) {
                if (B.slotKind[size_t(slot)] != RigExecBakedSlotKind::FirstFramePose) continue;
                const int parent = B.parent[size_t(slot)];
                if (parent < 0 || (parent >= group.begin && parent < group.end)) continue;
                if (rest && !restParent) {
                    omit(step, RigExecBakedSlotDomain::Rest, uint32_t(parent));
                    restParent = true;
                }
                if (ladder && !ladderParentRest) {
                    omit(step, RigExecBakedSlotDomain::Rest, uint32_t(parent));
                    ladderParentRest = true;
                }
                if (ladder && !ladderParent) {
                    omit(step, RigExecBakedSlotDomain::Ladder, uint32_t(parent));
                    ladderParent = true;
                }
            }
        }
        CHECK(restParent && ladderParentRest && ladderOwnRest && ladderParent);
    }
    {
        // A Solve reading a rest slot no RestCompose writes.
        RigExecBakedStep *solve = nullptr;
        for (RigExecBakedStep &step : B.steps) {
            if (step.kind == RigExecBakedStepKind::Solve) {
                solve = &step;
                break;
            }
        }
        CHECK(solve);
        if (solve) {
            const uint32_t slot = uint32_t(B.paths.size()) + 3;
            solve->reads.push_back(
                RigExecBakedOne(RigExecBakedSlotDomain::Rest, slot));
            error.clear();
            CHECK(!RigExecBakedValidateHeadReads(B, &error));
            const std::string expected =
                "step " + solve->label + " reads Rest slot " +
                std::to_string(slot) + ", which no head step writes";
            if (error.find(expected) == std::string::npos) {
                ++failures;
                std::printf("FAIL \"%s\" is not in \"%s\"\n",
                            expected.c_str(), error.c_str());
            }
            std::printf("  a solve reading an unwritten rest: %s\n",
                        error.c_str());
            solve->reads.pop_back();
            const auto saved=solve->reads;
            const auto required=RigExecBakedRequiredRestReads(B,*solve);
            CHECK(!required.empty());
            if (!required.empty()) {
                const auto missing=required.front();
                solve->reads.erase(std::remove_if(solve->reads.begin(),solve->reads.end(),
                    [&missing](const auto &r) { return r.domain==missing.domain &&
                        r.begin<=missing.begin && missing.begin<r.end; }),solve->reads.end());
                error.clear();
                CHECK(!RigExecBakedValidateHeadReads(B,&error));
                CHECK(error.find("step "+solve->label+" omits required "+
                    RigExecBakedSlotDomainName(missing.domain)+" slot "+std::to_string(missing.begin))!=std::string::npos);
                solve->reads=saved;
            }

        }
    }
    error.clear();
    CHECK(RigExecBakedValidateHeadTier(B, &error));
    CHECK(RigExecBakedValidateHeadReads(B, &error));
}

// The property revisions run in a pass before the rest and ladder ops, so
// the validator refuses a head order that puts a rest op before one.
void
TestTheValidatorRefusesARestBeforeAProperty(const std::string &examples)
{
    UsdStageRefPtr stage =
        UsdStage::Open(examples + "/09_PropertyMathMovers.usda");
    CHECK(stage);
    if (!stage) {
        return;
    }
    RigExecRigEvaluator evaluator(stage, FindRig(stage));
    std::vector<std::string> errors;
    CHECK(evaluator.Compile(&errors));
    evaluator.SetEvaluationMode(RigExecEvaluationMode::Baked);
    CHECK(evaluator.Evaluate(UsdTimeCode(StartOf(stage))).valid);
    const RigExecBakedProgramImpl *program = Program(evaluator);
    CHECK(program);
    if (!program) {
        return;
    }
    // Edited in place and put back: the program is the evaluator's.
    RigExecBakedProgramImpl &B =
        const_cast<RigExecBakedProgramImpl &>(*program);
    // A rest op that reads nothing, moved to the front of the order, so the
    // first violation is the property revision after it.
    size_t from = RigExecBakedHeadIndices(B).size();
    bool property = false;
    for (size_t p = 0; p < RigExecBakedHeadIndices(B).size(); ++p) {
        const RigExecBakedStep &step = B.steps[RigExecBakedHeadIndices(B)[p]];
        property = property ||
                   step.kind == RigExecBakedStepKind::PropertyRevision;
        if (from == RigExecBakedHeadIndices(B).size() &&
            step.kind == RigExecBakedStepKind::RestCompose &&
            step.reads.empty()) {
            from = p;
        }
    }
    CHECK(property && from < RigExecBakedHeadIndices(B).size());
    if (!property || from == RigExecBakedHeadIndices(B).size()) {
        return;
    }
    const std::vector<RigExecBakedStep> savedSteps = B.steps;
    std::rotate(B.steps.begin(),B.steps.begin()+from,B.steps.begin()+from+1);
    std::string error;
    CHECK(!RigExecBakedValidateHeadTier(B, &error));
    const std::string expected =
        "is ordered after " + B.steps[RigExecBakedHeadIndices(B)[0]].label +
        ", which runs after every property revision";
    if (error.find(expected) == std::string::npos) {
        ++failures;
        std::printf("FAIL \"%s\" is not in \"%s\"\n", expected.c_str(),
                    error.c_str());
    }
    std::printf("  a rest op before a property revision: %s\n",
                error.c_str());
    B.steps = savedSteps;
    error.clear();
    CHECK(RigExecBakedValidateHeadTier(B, &error));
}

// A parent joint's animated rest:tx, with an authored non-identity
// default:space on it and on its child: an edit of the rest moves both
// rests and neither ladder. The output-affected index admits the rest:tx
// path, its seeds are RigExecBakedHeadSeeds' closure, and they include the
// child's ProviderMatrix clusters, which the head tier's closure reaches
// through Rest[parent]. The edit is routed without a rebuild, and the
// served pose equals the reference walk's and a fresh program's.
void
TestARestEditIsSeedable()
{
    const SdfPath root("/Asset/Rig/Joints/Root");
    const SdfPath child("/Asset/Rig/Joints/Root/Child");
    const SdfPath restTxPath = root.AppendProperty(TfToken("rest:tx"));
    UsdStageRefPtr stage = JointRig([](const UsdStageRefPtr &stage) {
        const UsdAttribute restTx =
            stage->GetPrimAtPath(SdfPath("/Asset/Rig/Joints/Root"))
                .GetAttribute(TfToken("rest:tx"));
        restTx.Set(1.0, UsdTimeCode(1.0));
        restTx.Set(2.0, UsdTimeCode(2.0));
        restTx.Set(4.0, UsdTimeCode(3.0));
        SetDefaultSpace(stage, "/Asset/Rig/Joints/Root",
                        Offset(0.0, 1.0, 0.0));
        SetDefaultSpace(stage, "/Asset/Rig/Joints/Root/Child",
                        Offset(0.0, 0.0, 1.0));
    });
    RigExecRigEvaluator baked(stage, SdfPath("/Asset/Rig"));
    std::vector<std::string> errors;
    CHECK(baked.Compile(&errors));
    CHECK(baked.IsBakeable());
    baked.SetEvaluationMode(RigExecEvaluationMode::Baked);
    CHECK(baked.Evaluate(UsdTimeCode(1.0)).valid);
    CHECK(baked.Evaluate(UsdTimeCode(2.0)).valid);
    const RigExecBakedProgram *standing = baked.GetBakedProgram();
    CHECK(standing);
    if (!standing) {
        return;
    }
    const RigExecBakedProgramImpl &B = standing->GetStepGraph();
    const auto found = B.overridableInputs.find(restTxPath);
    CHECK(found != B.overridableInputs.end() && found->second.size() == 1);
    if (found == B.overridableInputs.end() || found->second.empty()) {
        return;
    }
    const int index = found->second.front();
    CHECK(B.cones.editRoute[size_t(index)] & kEditRouteHead);
    const std::vector<int> seeds = RigExecBakedHeadSeeds(B, index);
    RigExecOutputAffectedIndex affected;
    affected.Build(B, 1);
    const RigExecControlId control = RigExecControlIdForPath(restTxPath);
    CHECK(affected.IsKnownControl(control));
    std::vector<int> admitted = affected.SeedsForControl(control);
    std::sort(admitted.begin(), admitted.end());
    admitted.erase(std::unique(admitted.begin(), admitted.end()),
                   admitted.end());
    CHECK(admitted == seeds);
    // The child's ProviderMatrix clusters, and whether the ops that read
    // the leaf reach them without the closure.
    const int childSlot = SlotOf(B, child);
    CHECK(childSlot >= 0);
    std::set<int> direct;
    for (const uint32_t op : RigExecBakedHeadOpsReading(B, index)) {
        for (const RigExecBakedSlotRange &range : B.steps[op].writes) {
            const auto &table = range.domain == RigExecBakedSlotDomain::Rest
                                    ? B.cones.restReaders
                                    : B.cones.ladderReaders;
            for (uint32_t id = range.begin; id < range.end; ++id) {
                for (const int step : table[id]) {
                    direct.insert(B.steps[size_t(step)].cluster);
                }
            }
        }
    }
    size_t childMatrices = 0, closureOnly = 0;
    for (const RigExecBakedStep &step : B.steps) {
        if (step.kind == RigExecBakedStepKind::ProviderMatrix &&
            step.object == childSlot) {
            ++childMatrices;
            CHECK(std::binary_search(seeds.begin(), seeds.end(),
                                     step.cluster));
            closureOnly += direct.count(step.cluster) ? 0 : 1;
        }
    }
    CHECK(childMatrices > 0);
    std::printf("rest edit seeds: %zu cluster(s); %zu of the child's %zu "
                "matrix cluster(s) reached only through the closure\n",
                seeds.size(), closureOnly, childMatrices);

    // The edit: one of the parent's samples, at the frame shown next.
    const size_t builds = baked.GetBakedProgramBuildCount();
    stage->GetPrimAtPath(root).GetAttribute(TfToken("rest:tx"))
        .Set(1.75, UsdTimeCode(1.0));
    const RigExecRigPose edited = baked.Evaluate(UsdTimeCode(1.0));
    CHECK(baked.GetBakedProgramBuildCount() == builds);
    CHECK(baked.GetBakedProgram() == standing);
    RigExecRigEvaluator reference(stage, SdfPath("/Asset/Rig"));
    CHECK(reference.Compile(&errors));
    reference.SetEvaluationMode(RigExecEvaluationMode::ExecReference);
    CompareWithReference("rest edit, reference", edited,
                         reference.Evaluate(UsdTimeCode(1.0)));
    RigExecRigEvaluator fresh(stage, SdfPath("/Asset/Rig"));
    CHECK(fresh.Compile(&errors));
    fresh.SetEvaluationMode(RigExecEvaluationMode::Baked);
    CompareWithReference("rest edit, fresh program", edited,
                         fresh.Evaluate(UsdTimeCode(1.0)));
}

}  // namespace

int
main(int argc, char **argv)
{
    if (argc < 2) {
        std::printf("usage: testRigExecRestOps <examples dir>\n");
        return 2;
    }
    PlugRegistry::GetInstance().RegisterPlugins(
        TfAbsPath(RIGEXEC_SCHEMA_RESOURCE_DIR));
    const std::string examples = argv[1];
    TestARestOnlyMoveSkipsTheCompose();
    TestARestEditIsSeedable();
    TestTheValidatorRefusesAMisorderedRest(examples);
    TestTheValidatorRefusesARestBeforeAProperty(examples);
    TestEveryRestReaderDeclaresIt(examples);
    TestTheRestTablesMatchFreshEpochs(examples);
    std::printf("testRigExecRestOps: %d failure(s)\n", failures);
    return failures ? 1 : 0;
}
