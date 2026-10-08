// The epoch's rest frames: when they may be pulled once, and what has to put
// them back on the per-frame path.
// A rest frame is a function of rest:space and the six rest avars of a
// provider and of its RigExec ancestors, and of nothing else. When none of
// those can move within the epoch, every frame's answer is the same answer
// and the whole set is pulled once at Compile. The cases below are the ones
// where "once" is wrong, and each of them is invisible to the structure
// digest -- which hashes wiring, not values and not time codes -- so none of
// them recompiles by itself:
//   * a rest channel carrying time samples, or connected, or written by a
//     property chain: the frames are not epoch constants at all;
//   * the same three, authored AFTER the epoch was compiled;
//   * an interactive override standing on a rest channel: a drag is a value
//     that stands in for an authored one, so it has to move the rest exactly
//     as authoring it would, or the drag and the commit of that same drag
//     disagree -- and jointMatricesFinal is the rest->pose map, so a pose
//     that moved against a rest that did not is not a pose of this rig.
// Every case is checked against a REFERENCE stage carrying the same value
// authored plainly before it was ever compiled, because a rest edit moves a
// joint's rest frame and its posed frame together and comparing the two
// halves against each other proves nothing.
// argv[1] = path to the examples directory.
#include "rigExec/inputReplay.h"
#include "rigExec/rigEvaluator.h"
#include "rigExec/bakedProgramImpl.h"

#include "pxr/base/plug/registry.h"
#include "pxr/base/tf/pathUtils.h"
#include "pxr/usd/sdf/types.h"
#include "pxr/usd/usd/attribute.h"
#include "pxr/usd/usd/editTarget.h"
#include "pxr/usd/usd/prim.h"
#include "pxr/usd/usd/relationship.h"
#include "pxr/usd/usd/stage.h"

#include <algorithm>
#include <cstdio>
#include <functional>
#include <map>
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

const SdfPath kRig("/TailAsset/Rig");
const SdfPath kJoint("/TailAsset/Rig/Joints/Seg1/Seg2");
const std::vector<double> kFrames{1001.0, 1024.0, 1048.0};

using _Edit = std::function<void(const UsdStageRefPtr &)>;

std::string
StagePath(const std::string &examplesDir)
{
    return examplesDir + "/01_FkChainTail.usda";
}

void
EditInSession(const UsdStageRefPtr &stage)
{
    stage->SetEditTarget(UsdEditTarget(stage->GetSessionLayer()));
}

UsdAttribute
RestTx(const UsdStageRefPtr &stage)
{
    const UsdPrim joint = stage->GetPrimAtPath(kJoint);
    if (const UsdAttribute existing = joint.GetAttribute(TfToken("rest:tx"))) {
        return existing;
    }
    return joint.CreateAttribute(TfToken("rest:tx"),
                                 SdfValueTypeNames->Double);
}

UsdAttribute
RestSpace(const UsdStageRefPtr &stage)
{
    const UsdPrim joint = stage->GetPrimAtPath(kJoint);
    if (const UsdAttribute existing =
            joint.GetAttribute(TfToken("rest:space"))) {
        return existing;
    }
    return joint.CreateAttribute(TfToken("rest:space"),
                                 SdfValueTypeNames->Matrix4d);
}

GfMatrix4d
Translation(double x)
{
    GfMatrix4d m(1.0);
    m[3][0] = x;
    return m;
}

// Every published domain of one generation, so a case compares the whole
// answer rather than the part it happened to think of.
void
ComparePoses(const std::string &where, const RigExecRigPose &reference,
             const RigExecRigPose &subject)
{
    const auto sameFrame = [](const RigExecPointFrame &a,
                              const RigExecPointFrame &b) {
        return a.flags == b.flags && a.points == b.points;
    };
    const auto compare = [&where](const auto &referenceMap,
                                  const auto &subjectMap, const char *what,
                                  auto equal) {
        for (const auto &[path, value] : referenceMap) {
            const auto found = subjectMap.find(path);
            if (found == subjectMap.end()) {
                ++failures;
                std::printf("FAIL %s: no %s for %s\n", where.c_str(), what,
                            path.GetText());
            } else if (!equal(value, found->second)) {
                ++failures;
                std::printf("FAIL %s: %s differs at %s\n", where.c_str(),
                            what, path.GetText());
            }
        }
        if (referenceMap.size() != subjectMap.size()) {
            ++failures;
            std::printf("FAIL %s: %s count %zu, expected %zu\n", where.c_str(),
                        what, subjectMap.size(), referenceMap.size());
        }
    };
    compare(reference.jointFramesBase, subject.jointFramesBase,
            "base joint frame", sameFrame);
    compare(reference.jointFramesFinal, subject.jointFramesFinal,
            "final joint frame", sameFrame);
    compare(reference.jointMatricesFinal, subject.jointMatricesFinal,
            "joint matrix",
            [](const GfMatrix4d &a, const GfMatrix4d &b) { return a == b; });
    compare(reference.movedProperties, subject.movedProperties,
            "moved property",
            [](const VtValue &a, const VtValue &b) { return a == b; });
}

// Opens the tail, applies \p edit, compiles, and evaluates \p kFrames.
std::vector<RigExecRigPose>
Run(const std::string &examplesDir, const _Edit &edit, size_t *epochRestCount)
{
    std::vector<RigExecRigPose> poses;
    UsdStageRefPtr stage = UsdStage::Open(StagePath(examplesDir));
    CHECK(stage);
    if (!stage) return poses;
    if (edit) edit(stage);
    RigExecRigEvaluator rig(stage, kRig);
    std::vector<std::string> errors;
    if (!rig.Compile(&errors)) {
        ++failures;
        for (const std::string &error : errors) {
            std::printf("  compile error: %s\n", error.c_str());
        }
        return poses;
    }
    for (double frame : kFrames) {
        poses.push_back(rig.Evaluate(UsdTimeCode(frame)));
        CHECK(poses.back().valid);
    }
    if (epochRestCount) *epochRestCount = rig.GetEpochRestFrameCount();
    return poses;
}

// The reference for "rest:tx is v at this frame": v authored plainly, before
// this rig was ever compiled.
std::vector<RigExecRigPose>
RunWithConstantRestTx(const std::string &examplesDir, double value,
                      size_t *epochRestCount)
{
    return Run(examplesDir,
               [value](const UsdStageRefPtr &stage) {
                   EditInSession(stage);
                   RestTx(stage).Set(value);
               },
               epochRestCount);
}

// The same reference for rest:space, which the tail already authors on this
// joint -- so the value below REPLACES a rest space rather than introducing
// one, and a case that failed to apply would be visible as a pose that did
// not move at all.
std::vector<RigExecRigPose>
RunWithConstantRestSpace(const std::string &examplesDir,
                         const GfMatrix4d &value, size_t *epochRestCount)
{
    return Run(examplesDir,
               [&value](const UsdStageRefPtr &stage) {
                   EditInSession(stage);
                   RestSpace(stage).Set(value);
               },
               epochRestCount);
}


// The ordinary rig: no rest channel can move, so the epoch holds them all.
// Everything below is a departure from this, and without it they could all
// pass by the epoch never taking the constant path at all.
void
TestAnOrdinaryRigHoldsItsRests(const std::string &examplesDir)
{
    size_t epochRests = 0;
    const std::vector<RigExecRigPose> poses =
        Run(examplesDir, nullptr, &epochRests);
    CHECK(!poses.empty());
    if (poses.empty()) return;
    CHECK(epochRests > 0);
    // Providers, not joints: controls seed the same request.
    CHECK(epochRests >= poses.front().jointFramesBase.size());
}

// A time-sampled rest channel. The epoch must decline -- asserted on the
// classification and not only on the arithmetic, which could coincide -- and
// every frame must equal the value that frame's sample holds.
void
TestATimeSampledRestIsPulledPerFrame(const std::string &examplesDir)
{
    size_t epochRests = 1;
    const std::vector<RigExecRigPose> subject = Run(
        examplesDir,
        [](const UsdStageRefPtr &stage) {
            EditInSession(stage);
            const UsdAttribute restTx = RestTx(stage);
            restTx.Set(0.0, UsdTimeCode(kFrames[0]));
            restTx.Set(3.0, UsdTimeCode(kFrames[1]));
            restTx.Set(6.0, UsdTimeCode(kFrames[2]));
            CHECK(restTx.ValueMightBeTimeVarying());
        },
        &epochRests);
    // Seg2 and its two joint descendants vary; four controls and Seg1 stay static.
    CHECK(epochRests == 5);
    if (subject.size() != kFrames.size()) return;

    const double sampled[] = {0.0, 3.0, 6.0};
    for (size_t i = 0; i < kFrames.size(); ++i) {
        size_t referenceRests = 0;
        const std::vector<RigExecRigPose> reference =
            RunWithConstantRestTx(examplesDir, sampled[i], &referenceRests);
        // The reference itself has nothing animated, so it must be on the
        // epoch path -- otherwise the comparison is between two copies of
        // the same fallback.
        CHECK(referenceRests > 0);
        if (reference.size() != kFrames.size()) return;
        ComparePoses("a time-sampled rest at frame " +
                         std::to_string(int(kFrames[i])),
                     reference[i], subject[i]);
    }
}

// A rest channel that gains its samples AFTER the epoch was compiled: the
// digest does not move, so nothing recompiles by itself, and the decision has
// to be re-taken when the notice arrives.
void
TestARestThatBecomesTimeSampledAfterCompile(const std::string &examplesDir)
{
    UsdStageRefPtr stage = UsdStage::Open(StagePath(examplesDir));
    CHECK(stage);
    if (!stage) return;
    RigExecRigEvaluator rig(stage, kRig);
    std::vector<std::string> errors;
    CHECK(rig.Compile(&errors));
    const size_t digest = rig.GetBindingEpochDigest();
    CHECK(rig.GetEpochRestFrameCount() > 0);
    CHECK(rig.Evaluate(UsdTimeCode(kFrames[0])).valid);

    EditInSession(stage);
    const UsdAttribute restTx = RestTx(stage);
    const double sampled[] = {0.0, 3.0, 6.0};
    for (size_t i = 0; i < kFrames.size(); ++i) {
        restTx.Set(sampled[i], UsdTimeCode(kFrames[i]));
    }
    CHECK(restTx.ValueMightBeTimeVarying());

    for (size_t i = 0; i < kFrames.size(); ++i) {
        const RigExecRigPose subject = rig.Evaluate(UsdTimeCode(kFrames[i]));
        CHECK(subject.valid);
        size_t referenceRests = 0;
        const std::vector<RigExecRigPose> reference =
            RunWithConstantRestTx(examplesDir, sampled[i], &referenceRests);
        CHECK(referenceRests > 0);
        if (reference.size() != kFrames.size()) return;
        ComparePoses("a rest sampled after Compile, frame " +
                         std::to_string(int(kFrames[i])),
                     reference[i], subject);
    }
    // It came off the epoch path, and the wiring never changed, so the
    // rebuild that took it off is not a new binding epoch.
    CHECK(rig.GetEpochRestFrameCount() == 5);
    CHECK(rig.GetBindingEpochDigest() == digest);
}

// A connected rest channel with a constant driver remains a captured
// constant in the shared graph. All eight provider rest closures stay static;
// the numeric answer must still agree with the plainly authored reference.
void
TestAConnectedRestIsPulledPerFrame(const std::string &examplesDir)
{
    size_t epochRests = 1;
    const std::vector<RigExecRigPose> subject = Run(
        examplesDir,
        [](const UsdStageRefPtr &stage) {
            EditInSession(stage);
            const UsdPrim joint = stage->GetPrimAtPath(kJoint);
            const UsdAttribute driver = joint.CreateAttribute(
                TfToken("inputs:restDriver"), SdfValueTypeNames->Double);
            driver.Set(3.0);
            const UsdAttribute restTx = RestTx(stage);
            restTx.SetConnections({driver.GetPath()});
            CHECK(restTx.HasAuthoredConnections());
        },
        &epochRests);
    CHECK(epochRests == 8);
    if (subject.size() != kFrames.size()) return;

    size_t referenceRests = 0;
    const std::vector<RigExecRigPose> reference =
        RunWithConstantRestTx(examplesDir, 3.0, &referenceRests);
    CHECK(referenceRests > 0);
    if (reference.size() != kFrames.size()) return;
    for (size_t i = 0; i < kFrames.size(); ++i) {
        ComparePoses("a connected rest at frame " +
                         std::to_string(int(kFrames[i])),
                     reference[i], subject[i]);
    }
}

// rest:space carrying a connection.
// It is the one MATRIX channel of the ladder exec reads with a plain
// AttributeValue -- computations.cpp declares it in computeRestFrame beside
// AttributeValue<double>(rest:tx), and registers its five space expressions
// on OTHER attributes -- so a connection on it resolves by the same
// single-connection walk the scalar rests resolve by, and the program may
// read it per frame instead of refusing. The reference is the identical
// matrix authored plainly, exactly as the rest:tx case above.
void
TestAConnectedRestSpaceIsPulledPerFrame(const std::string &examplesDir)
{
    size_t epochRests = 1;
    const std::vector<RigExecRigPose> subject = Run(
        examplesDir,
        [](const UsdStageRefPtr &stage) {
            EditInSession(stage);
            const UsdPrim joint = stage->GetPrimAtPath(kJoint);
            const UsdAttribute driver = joint.CreateAttribute(
                TfToken("inputs:restSpaceDriver"),
                SdfValueTypeNames->Matrix4d);
            driver.Set(Translation(5.0));
            const UsdAttribute restSpace = RestSpace(stage);
            restSpace.SetConnections({driver.GetPath()});
            CHECK(restSpace.HasAuthoredConnections());
        },
        &epochRests);
    CHECK(epochRests == 8);
    if (subject.size() != kFrames.size()) return;

    size_t referenceRests = 0;
    const std::vector<RigExecRigPose> reference =
        RunWithConstantRestSpace(examplesDir, Translation(5.0),
                                 &referenceRests);
    CHECK(referenceRests > 0);
    if (reference.size() != kFrames.size()) return;
    for (size_t i = 0; i < kFrames.size(); ++i) {
        ComparePoses("a connected rest space at frame " +
                         std::to_string(int(kFrames[i])),
                     reference[i], subject[i]);
    }
    // And it is not the plain rig wearing a connection: the reference has to
    // have moved the joint, or both halves could agree on the unedited pose.
    size_t untouchedRests = 0;
    const std::vector<RigExecRigPose> untouched =
        Run(examplesDir, _Edit(), &untouchedRests);
    CHECK(untouched.size() == kFrames.size());
    if (untouched.size() == kFrames.size()) {
        const SdfPath child = kJoint.AppendChild(TfToken("Seg3"));
        const auto moved = reference[0].jointMatricesFinal.find(child);
        const auto still = untouched[0].jointMatricesFinal.find(child);
        CHECK(moved != reference[0].jointMatricesFinal.end());
        CHECK(still != untouched[0].jointMatricesFinal.end());
        if (moved != reference[0].jointMatricesFinal.end() &&
            still != untouched[0].jointMatricesFinal.end()) {
            CHECK(moved->second != still->second);
        }
    }
}

// A computed parent:space is a real producer of the connected rest value.
// ORIGINAL's reference evaluator agrees with an independently authored matrix
// for this identity -> translation -> held translation -> identity history.
void
TestARestSpaceConnectedToAComputedSpaceHasAProducer(const std::string &examplesDir)
{
    const auto stage=UsdStage::Open(StagePath(examplesDir));
    const auto referenceStage=UsdStage::Open(StagePath(examplesDir));
    CHECK(stage && referenceStage);
    if(!stage || !referenceStage)return;
    EditInSession(stage);EditInSession(referenceStage);
    const SdfPath parent=kJoint.GetParentPath();
    const auto source=stage->GetPrimAtPath(parent).CreateAttribute(
        TfToken("parent:space"),SdfValueTypeNames->Matrix4d);
    const auto referenceSource=referenceStage->GetPrimAtPath(parent).CreateAttribute(
        TfToken("parent:space"),SdfValueTypeNames->Matrix4d);
    CHECK(source.Set(GfMatrix4d(1.0)));
    CHECK(referenceSource.Set(GfMatrix4d(1.0)));
    CHECK(RestSpace(stage).SetConnections({source.GetPath()}));
    CHECK(RestSpace(referenceStage).Set(GfMatrix4d(1.0)));
    RigExecRigEvaluator rig(stage,kRig),reference(referenceStage,kRig);
    std::vector<std::string> errors,reasons;
    CHECK(rig.Compile(&errors));CHECK(reference.Compile(&errors));
    CHECK(rig.IsBakeable(&reasons));
    const size_t digest=rig.GetBindingEpochDigest();
    std::vector<RigExecRigPose> initial;
    for(double x:{0.0,7.0,7.0,0.0}) {
        const auto matrix=Translation(x);
        CHECK(source.Set(matrix));CHECK(referenceSource.Set(matrix));
        CHECK(RestSpace(referenceStage).Set(matrix));
        std::string before;CHECK(stage->GetSessionLayer()->ExportToString(&before));
        for(size_t frame=0;frame<kFrames.size();++frame) {
            const auto expected=reference.Evaluate(UsdTimeCode(kFrames[frame]));
            const auto actual=rig.Evaluate(UsdTimeCode(kFrames[frame]));
            CHECK(expected.valid && actual.valid);
            ComparePoses("computed rest versus independently authored matrix",expected,actual);
            CHECK(rig.GetBindingEpochDigest()==digest);
            if(initial.size()<kFrames.size())initial.push_back(actual);
            if(x==7.0) {
                const auto changed=actual.jointMatricesFinal.find(kJoint);
                const auto unchanged=initial[frame].jointMatricesFinal.find(kJoint);
                CHECK(changed!=actual.jointMatricesFinal.end() &&
                      unchanged!=initial[frame].jointMatricesFinal.end());
                if(changed!=actual.jointMatricesFinal.end() && unchanged!=initial[frame].jointMatricesFinal.end())
                    CHECK(changed->second!=unchanged->second);
            } else if(initial.size()==kFrames.size())
                ComparePoses("computed rest recovery",initial[frame],actual);
        }
        std::string after;CHECK(stage->GetSessionLayer()->ExportToString(&after));
        CHECK(before==after);
    }
    const auto *program=rig.GetBakedProgram();CHECK(program);
    if(!program)return;
    const auto &B=program->GetStepGraph();const auto slot=B.index.find(kJoint);
    CHECK(slot!=B.index.end());if(slot==B.index.end())return;
    const auto contains=[](const auto &ranges,RigExecBakedSlotDomain domain,uint32_t value) {
        for(const auto &range:ranges)if(range.domain==domain && range.begin<=value && value<range.end)return true;
        return false;
    };
    bool produced=false;
    for(size_t rest=0;rest<B.steps.size();++rest)if(B.steps[rest].kind==RigExecBakedStepKind::RestCompose &&
        contains(B.steps[rest].writes,RigExecBakedSlotDomain::Rest,uint32_t(slot->second)))
        for(size_t sourceOp=0;sourceOp<B.steps.size();++sourceOp)if(B.steps[sourceOp].kind==RigExecBakedStepKind::SpaceExpression)
            for(const auto &write:B.steps[sourceOp].writes)if(write.domain==RigExecBakedSlotDomain::SpaceValue)
                for(uint32_t value=write.begin;value<write.end;++value)
                    if(contains(B.steps[rest].reads,RigExecBakedSlotDomain::SpaceValue,value) &&
                       std::find(B.opGraph.ops[rest].predecessors.begin(),B.opGraph.ops[rest].predecessors.end(),uint32_t(sourceOp))!=B.opGraph.ops[rest].predecessors.end())produced=true;
    CHECK(produced);
}

// A rest that moves while nothing else in the rig does.
// The pose prologue recomposing a ladder is a per-run delta like any other,
// and it owes the schedule a dirty hook: without one, a frame whose avars
// and whose time-sampled inputs all stood still would skip the compose that
// the moved rest was the only reason to run. No SHIPPED rig can show that --
// on the tail every compose shares its cluster with the skin sources, which
// run on every frame -- so the case is built here, with static avars, one
// animated rest and nothing else that moves.
void
TestARestThatMovesAloneStillRecomposes(const std::string &examplesDir)
{
    (void)examplesDir;
    UsdStageRefPtr stage = UsdStage::CreateInMemory();
    stage->SetStartTimeCode(1.0);
    stage->SetEndTimeCode(3.0);
    stage->DefinePrim(SdfPath("/Asset"), TfToken("Xform"));
    const SdfPath rigPath("/Asset/Rig");
    stage->DefinePrim(rigPath, TfToken("RigExecRoot"));
    stage->DefinePrim(SdfPath("/Asset/Rig/Controls"), TfToken("Scope"));
    stage->DefinePrim(SdfPath("/Asset/Rig/Controls/Ctl"),
                      TfToken("RigExecControl"))
        .CreateAttribute(TfToken("avars:tx"), SdfValueTypeNames->Double)
        .Set(0.25);
    stage->DefinePrim(SdfPath("/Asset/Rig/Joints"), TfToken("Scope"));
    const UsdPrim root = stage->DefinePrim(
        SdfPath("/Asset/Rig/Joints/Root"), TfToken("RigExecJoint"));
    root.CreateAttribute(TfToken("rest:tx"), SdfValueTypeNames->Double)
        .Set(1.0);
    const SdfPath childPath("/Asset/Rig/Joints/Root/Child");
    const UsdPrim child =
        stage->DefinePrim(childPath, TfToken("RigExecJoint"));
    const UsdAttribute restTx =
        child.CreateAttribute(TfToken("rest:tx"), SdfValueTypeNames->Double);
    restTx.Set(0.0, UsdTimeCode(1.0));
    restTx.Set(2.0, UsdTimeCode(2.0));
    restTx.Set(5.0, UsdTimeCode(3.0));
    const SdfPath leafPath("/Asset/Rig/Joints/Root/Child/Leaf");
    stage->DefinePrim(leafPath, TfToken("RigExecJoint"))
        .CreateAttribute(TfToken("rest:tx"), SdfValueTypeNames->Double)
        .Set(1.0);

    RigExecRigEvaluator rig(stage, rigPath);
    std::vector<std::string> errors;
    CHECK(rig.Compile(&errors));
    std::vector<std::string> reasons;
    CHECK(rig.IsBakeable(&reasons));
    for (const std::string &reason : reasons) {
        std::printf("    unexpected refusal: %s\n", reason.c_str());
    }
    // Frame 1 first, so the frames that follow are STEADY-STATE frames --
    // the state in which the dirty set, and not the first-run "everything
    // runs" path, decides what is recomposed.
    for (const auto &[frame, x] : {std::make_pair(1.0, 1.0),
                                   std::make_pair(2.0, 3.0),
                                   std::make_pair(3.0, 6.0),
                                   std::make_pair(1.0, 1.0)}) {
        const RigExecRigPose pose = rig.Evaluate(UsdTimeCode(frame));
        CHECK(pose.valid);
        const auto found = pose.jointFramesFinal.find(childPath);
        CHECK(found != pose.jointFramesFinal.end());
        if (found != pose.jointFramesFinal.end()) {
            const double origin = found->second.Origin()[0];
            CHECK(GfIsClose(origin, x, 1e-9));
            if (!GfIsClose(origin, x, 1e-9)) {
                std::printf("    frame %g: child at %g, expected %g\n",
                            frame, origin, x);
            }
        }
    }
}

// A property chain writing a rest attribute. The chain recomputes it every
// generation, so the epoch cannot hold the answer -- the same guard the skin
// layout applies to its own three attributes.
void
TestAPropertyChainOnARestRefusesTheEpochPath(const std::string &examplesDir)
{
    size_t epochRests = 1;
    const auto addRestOffset = [](const UsdStageRefPtr &stage) {
            EditInSession(stage);
            // Under the rig's Movers scope, which is where the mover walk
            // composes a rig's mover stack from.
            const SdfPath mover =
                kRig.AppendChild(TfToken("Movers"))
                    .AppendChild(TfToken("RestOffset"));
            const UsdPrim prim =
                stage->DefinePrim(mover, TfToken("RigExecMatrixMathMover"));
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
                .SetTargets({kJoint.AppendProperty(TfToken("rest:space"))});
        };
    const std::vector<RigExecRigPose> subject =
        Run(examplesDir, addRestOffset, &epochRests);
    // Seg2 and its two joint descendants vary; four controls and Seg1 stay static.
    CHECK(epochRests == 5);
    CHECK(subject.size() == kFrames.size());
    // Independent authored rest: the property head must reach the ladder,
    // rather than being bypassed by its sampled raw provider value.
    size_t referenceRests = 0;
    const auto reference = Run(
        examplesDir, [](const UsdStageRefPtr &stage) {
            EditInSession(stage);
            GfMatrix4d authored(1.0);
            CHECK(RestSpace(stage).Get(&authored));
            CHECK(RestSpace(stage).Set(authored * Translation(3.0)));
        }, &referenceRests);
    CHECK(referenceRests > 0);
    CHECK(reference.size() == subject.size());
    if (reference.size() == subject.size()) {
        for (size_t i = 0; i < subject.size(); ++i) {
            CHECK(reference[i].valid && subject[i].valid);
            CHECK(reference[i].jointMatricesFinal ==
                  subject[i].jointMatricesFinal);
        }
    }
    // The same own revision must win on a connected head, without changing
    // the authored source. Reuse one compiled owner through edit/hold/recovery.
    const auto stage = UsdStage::Open(StagePath(examplesDir));
    const auto referenceStage = UsdStage::Open(StagePath(examplesDir));
    CHECK(stage && referenceStage);
    if (!stage || !referenceStage) return;
    EditInSession(stage);
    EditInSession(referenceStage);
    GfMatrix4d rawRest(1.0);
    CHECK(RestSpace(stage).Get(&rawRest));
    const auto driver = stage->GetPrimAtPath(kJoint).CreateAttribute(
        TfToken("inputs:ownRestDriver"), SdfValueTypeNames->Matrix4d);
    CHECK(driver.Set(rawRest));
    CHECK(RestSpace(stage).SetConnections({driver.GetPath()}));
    addRestOffset(stage);
    CHECK(RestSpace(referenceStage).Set(rawRest * Translation(3.0)));
    RigExecRigEvaluator rig(stage, kRig), authored(referenceStage, kRig);
    std::vector<std::string> errors;
    CHECK(rig.Compile(&errors));
    CHECK(authored.Compile(&errors));
    const auto *program = rig.GetBakedProgram();
    CHECK(program);
    if (!program) return;
    const auto &B = program->GetStepGraph();
    const auto slot = B.index.find(kJoint);
    CHECK(slot != B.index.end());
    if (slot == B.index.end()) return;
    const SdfPath target = RestSpace(stage).GetPath();
    const auto shared = B.providerProgram.attributeValues.find(target);
    CHECK(shared != B.providerProgram.attributeValues.end());
    if (shared == B.providerProgram.attributeValues.end()) return;
    const int ownValue = B.ladders[slot->second].spaceValues[0];
    CHECK(ownValue >= 0);
    CHECK(uint32_t(ownValue) != shared->second);
    bool ownFinal = false;
    for (const auto &route : B.providerProgram.routedInputs)
        if (route.value == uint32_t(ownValue))
            ownFinal = route.consumer == target && route.source == target &&
                       route.readPhase == TfToken("final");
    CHECK(ownFinal);
    const size_t digest = rig.GetBindingEpochDigest();
    const auto offsetInput = stage->GetPrimAtPath(
        kRig.AppendChild(TfToken("Movers")).AppendChild(TfToken("RestOffset")))
        .GetAttribute(TfToken("inputs:value"));
    std::vector<RigExecRigPose> initial;
    for (double x : {3.0, 7.0, 7.0, 0.0, 3.0}) {
        CHECK(offsetInput.Set(Translation(x)));
        CHECK(RestSpace(referenceStage).Set(rawRest * Translation(x)));
        for (size_t frame = 0; frame < kFrames.size(); ++frame) {
            const auto expected = authored.Evaluate(UsdTimeCode(kFrames[frame]));
            const auto actual = rig.Evaluate(UsdTimeCode(kFrames[frame]));
            CHECK(expected.valid && actual.valid);
            CHECK(expected.jointMatricesFinal == actual.jointMatricesFinal);
            CHECK(rig.GetBindingEpochDigest() == digest);
            GfMatrix4d unchanged(1.0);
            CHECK(driver.Get(&unchanged));
            CHECK(unchanged == rawRest);
            if (initial.size() < kFrames.size()) initial.push_back(actual);
            else if (x == 7.0)
                CHECK(actual.jointMatricesFinal != initial[frame].jointMatricesFinal);
            else if (x == 3.0)
                CHECK(actual.jointMatricesFinal == initial[frame].jointMatricesFinal);
        }
    }
}

// An interactive override standing on a rest channel.
// The drag and the commit of that same drag have to agree, so the override is
// compared against the identical value AUTHORED on a stage that was compiled
// with it. The epoch must still be on the constant-rest path here: that is
// the case the override has to reach, and a rig that declined would test
// nothing.
void
TestAnInteractiveOverrideOnARestIsFollowed(const std::string &examplesDir)
{
    UsdStageRefPtr stage = UsdStage::Open(StagePath(examplesDir));
    CHECK(stage);
    if (!stage) return;
    RigExecRigEvaluator rig(stage, kRig);
    std::vector<std::string> errors;
    CHECK(rig.Compile(&errors));
    CHECK(rig.GetEpochRestFrameCount() > 0);

    std::vector<RigExecRigPose> before;
    for (double frame : kFrames) {
        before.push_back(rig.Evaluate(UsdTimeCode(frame)));
    }

    // rest:tx, a plain avar.
    rig.SetInteractiveOverrides({RigExecValueOverride{
        kJoint, TfToken(), TfToken("rest:tx"), VtValue(3.0)}});
    size_t referenceRests = 0;
    const std::vector<RigExecRigPose> reference =
        RunWithConstantRestTx(examplesDir, 3.0, &referenceRests);
    CHECK(referenceRests > 0);
    if (reference.size() != kFrames.size()) return;
    for (size_t i = 0; i < kFrames.size(); ++i) {
        const RigExecRigPose dragged = rig.Evaluate(UsdTimeCode(kFrames[i]));
        CHECK(dragged.valid);
        ComparePoses("a rest:tx override at frame " +
                         std::to_string(int(kFrames[i])),
                     reference[i], dragged);
    }

    // rest:space, the matrix the ladder composes against, which is the
    // channel the manipulation path previews when it drags a joint's rest.
    GfMatrix4d space(1.0);
    stage->GetPrimAtPath(kJoint).GetAttribute(TfToken("rest:space"))
        .Get(&space);
    GfMatrix4d moved = space;
    moved[3][1] += 3.0;
    rig.SetInteractiveOverrides({RigExecValueOverride{
        kJoint, TfToken(), TfToken("rest:space"), VtValue(moved)}});
    size_t spaceReferenceRests = 0;
    const std::vector<RigExecRigPose> spaceReference = Run(
        examplesDir,
        [&moved](const UsdStageRefPtr &referenceStage) {
            EditInSession(referenceStage);
            referenceStage->GetPrimAtPath(kJoint)
                .GetAttribute(TfToken("rest:space")).Set(moved);
        },
        &spaceReferenceRests);
    CHECK(spaceReferenceRests > 0);
    if (spaceReference.size() != kFrames.size()) return;
    for (size_t i = 0; i < kFrames.size(); ++i) {
        const RigExecRigPose dragged = rig.Evaluate(UsdTimeCode(kFrames[i]));
        CHECK(dragged.valid);
        ComparePoses("a rest:space override at frame " +
                         std::to_string(int(kFrames[i])),
                     spaceReference[i], dragged);
    }

    // Released, the rig is the authored rig again -- which is what makes the
    // preview a preview.
    rig.ClearInteractiveOverrides();
    for (size_t i = 0; i < kFrames.size(); ++i) {
        ComparePoses("a released rest override at frame " +
                         std::to_string(int(kFrames[i])),
                     before[i], rig.Evaluate(UsdTimeCode(kFrames[i])));
    }
}

// An override on something that is NOT a rest channel must not drag the rests
// off the epoch path: the whole point of holding them is that almost nothing
// can move them.
void
TestAnOverrideElsewhereLeavesTheRestsAlone(const std::string &examplesDir)
{
    UsdStageRefPtr stage = UsdStage::Open(StagePath(examplesDir));
    CHECK(stage);
    if (!stage) return;
    RigExecRigEvaluator rig(stage, kRig);
    std::vector<std::string> errors;
    CHECK(rig.Compile(&errors));
    CHECK(rig.GetEpochRestFrameCount() > 0);
    const RigExecRigPose before = rig.Evaluate(UsdTimeCode(kFrames[1]));

    rig.SetInteractiveOverrides({RigExecValueOverride{
        SdfPath("/TailAsset/Rig/Controls/Tail1"), TfToken(),
        TfToken("avars:rz"), VtValue(12.0)}});
    const RigExecRigPose dragged = rig.Evaluate(UsdTimeCode(kFrames[1]));
    CHECK(dragged.valid);
    // The rests are unmoved, so the base frames are unmoved where no pose
    // reached them, and the drag still did something.
    CHECK(dragged.jointFramesFinal.size() == before.jointFramesFinal.size());
    bool moved = false;
    for (const auto &[path, frame] : dragged.jointFramesFinal) {
        const auto found = before.jointFramesFinal.find(path);
        if (found == before.jointFramesFinal.end() ||
            found->second.points != frame.points) {
            moved = true;
        }
    }
    CHECK(moved);
    rig.ClearInteractiveOverrides();
    ComparePoses("a released avar override", before,
                 rig.Evaluate(UsdTimeCode(kFrames[1])));
}

// Failure to prepare a required first-frame/rest provider refuses Compile.
// Both a cold epoch and a warm replacement withdraw the compiled program;
// clearing the unavailable-provider edit restores the original numeric pose.
void
TestAnUnavailableRequiredProviderRefusesCompile(const std::string &examplesDir)
{
    const std::string path=examplesDir+"/08_AimEyes.usda";
    const SdfPath rigPath("/EyesAsset/Rig"),aim("/EyesAsset/Rig/Movers/Pose/AimL");
    const SdfPath ghost("/EyesAsset/Rig/Controls/Ghost");
    const UsdTimeCode frame(1016.0);
    const auto breakSource=[&](const UsdStageRefPtr &stage) {
        EditInSession(stage);auto control=stage->DefinePrim(ghost,TfToken("RigExecControl"));
        CHECK(control);CHECK(control.SetActive(false));
        const auto target=stage->GetPrimAtPath(aim).GetRelationship(TfToken("rigExec:aimTarget"));
        CHECK(target);CHECK(target.SetTargets({ghost}));
    };
    const auto referenceStage=UsdStage::Open(path);CHECK(referenceStage);if(!referenceStage)return;
    RigExecRigEvaluator reference(referenceStage,rigPath);std::vector<std::string> errors;
    CHECK(reference.Compile(&errors));const auto expected=reference.Evaluate(frame);
    CHECK(expected.valid);CHECK(reference.GetBakedProgram());
    const std::vector<std::string> refusal{"failed to prepare pose provider inputs"};
    for(bool warm:{false,true}) {
        const auto stage=UsdStage::Open(path);CHECK(stage);if(!stage)continue;
        RigExecRigEvaluator rig(stage,rigPath);
        if(warm) {
            errors.clear();CHECK(rig.Compile(&errors));CHECK(rig.GetEpochRestFrameCount()>0);
            CHECK(rig.GetBakedProgram());
            ComparePoses("before unavailable required provider edit",expected,rig.Evaluate(frame));
        }
        breakSource(stage);
        std::string before;CHECK(stage->GetSessionLayer()->ExportToString(&before));
        errors.clear();CHECK(!rig.Compile(&errors));CHECK(errors==refusal);
        CHECK(rig.GetBakedProgram()==nullptr);
        for(int held=0;held<2;++held) {
            const auto actual=rig.Evaluate(frame);CHECK(!actual.valid);
            CHECK(std::find(actual.diagnostics.begin(),actual.diagnostics.end(),refusal[0])!=actual.diagnostics.end());
            CHECK(rig.GetBakedProgram()==nullptr);
        }
        std::string after;CHECK(stage->GetSessionLayer()->ExportToString(&after));CHECK(before==after);
        RigExecInputReplayClearLayer(stage->GetSessionLayer());errors.clear();CHECK(rig.Compile(&errors));
        CHECK(rig.GetEpochRestFrameCount()>0);CHECK(rig.GetBakedProgram());
        ComparePoses("unavailable required provider recovery",expected,rig.Evaluate(frame));
    }
}

// Opens the tail, applies \p edit, and evaluates \p kFrames in \p mode.
std::vector<RigExecRigPose>
RunWithReferenceChecks(const std::string &examplesDir, const _Edit &edit,
          bool referenceChecks)
{
    std::vector<RigExecRigPose> poses;
    UsdStageRefPtr stage = UsdStage::Open(StagePath(examplesDir));
    CHECK(stage);
    if (!stage) return poses;
    if (edit) edit(stage);
    RigExecRigEvaluator rig(stage, kRig);
    rig.cpuReference = referenceChecks;
    std::vector<std::string> errors;
    CHECK(rig.Compile(&errors));
    for (double frame : kFrames) {
        poses.push_back(rig.Evaluate(UsdTimeCode(frame)));
        CHECK(poses.back().valid);
    }
    return poses;
}

// A rest VALUE edited after the compile, in the session layer, on \p prim.
// The digest reads no rest value, so nothing recompiles, and the epoch keeps
// its rest frames as constants: only their values are wrong. The program
// answers the frames right after the edit from its own rebuilt rests, which
// is why the epoch's frames are left stale for as long as it does; the exec
// oracle is their one reader, so when the rig is next asked for the oracle,
// the oracle has to re-pull and read the edited rest -- not the one the
// epoch pulled at the compile. Both halves are compared with the same value
// authored before a fresh compile, in the same mode.
// \p prim is the joint itself, and a RigExec ancestor of it, whose rest
// reaches the joint through the namespace-ancestor input of its rest frame.
void
CheckARestValueEditAfterCompile(const std::string &examplesDir,
                                const SdfPath &prim, const std::string &what)
{
    const TfToken restRx("rest:rx");
    const double value = 15.0;
    const _Edit author = [&](const UsdStageRefPtr &stage) {
        EditInSession(stage);
        const UsdPrim target = stage->GetPrimAtPath(prim);
        CHECK(target);
        if (!target) return;
        UsdAttribute attribute = target.GetAttribute(restRx);
        if (!attribute) {
            attribute =
                target.CreateAttribute(restRx, SdfValueTypeNames->Double);
        }
        CHECK(attribute.Set(value));
    };
    const std::vector<RigExecRigPose> bakedReference =
        RunWithReferenceChecks(examplesDir, author, false);
    const std::vector<RigExecRigPose> oracleReference =
        RunWithReferenceChecks(examplesDir, author, true);
    if (bakedReference.size() != kFrames.size() ||
        oracleReference.size() != kFrames.size()) {
        return;
    }

    UsdStageRefPtr stage = UsdStage::Open(StagePath(examplesDir));
    CHECK(stage);
    if (!stage) return;
    RigExecRigEvaluator rig(stage, kRig);
    std::vector<std::string> errors;
    CHECK(rig.Compile(&errors));
    const size_t digest = rig.GetBindingEpochDigest();
    const size_t epochRests = rig.GetEpochRestFrameCount();
    CHECK(epochRests > 0);
    std::vector<RigExecRigPose> before;
    for (double frame : kFrames) {
        before.push_back(rig.Evaluate(UsdTimeCode(frame)));
        CHECK(before.back().valid);
    }
    CHECK(rig.GetBakedProgram() != nullptr);

    author(stage);
    for (size_t i = 0; i < kFrames.size(); ++i) {
        const RigExecRigPose pose = rig.Evaluate(UsdTimeCode(kFrames[i]));
        CHECK(pose.valid);
        for (const std::string &diagnostic : pose.diagnostics) {
            CHECK(diagnostic.find("epoch rebuilt") == std::string::npos);
        }
        ComparePoses(what + ", baked, frame " +
                         std::to_string(int(kFrames[i])),
                     bakedReference[i], pose);
    }
    // A value edit: the same epoch, still holding its rests as constants.
    CHECK(rig.GetBindingEpochDigest() == digest);
    CHECK(rig.GetEpochRestFrameCount() == epochRests);

    rig.cpuReference = true;
    for (size_t i = 0; i < kFrames.size(); ++i) {
        const RigExecRigPose pose = rig.Evaluate(UsdTimeCode(kFrames[i]));
        CHECK(pose.valid);
        ComparePoses(what + ", oracle, frame " +
                         std::to_string(int(kFrames[i])),
                     oracleReference[i], pose);
    }
    CHECK(rig.GetBindingEpochDigest() == digest);

    // And the edit moved the rig, or both halves could agree on the pose
    // the epoch had before it.
    bool moved = false;
    for (const auto &[path, matrix] : oracleReference[0].jointMatricesFinal) {
        const auto found = before[0].jointMatricesFinal.find(path);
        if (found == before[0].jointMatricesFinal.end() ||
            found->second != matrix) {
            moved = true;
        }
    }
    CHECK(moved);
}

// The same rest VALUE edit, with the program kept for the whole epoch and
// the oracle run beside it on every frame: cpuReference.
// The program stands, so the settle does not re-pull the epoch's rests; the
// scalar reference the mode runs after the program is their one reader and has
// to re-pull them itself, before it reads them. A walk that read the rests
// the epoch pulled at the compile would publish the pre-edit pose and count
// the program's (correct) answer as a parity mismatch. Compared with the
// same value authored before a fresh compile, in the same mode.
void
CheckARestValueEditUnderTheParityCheck(const std::string &examplesDir,
                                       const SdfPath &prim,
                                       const std::string &what)
{
    const TfToken restRx("rest:rx");
    const double value = 15.0;
    const _Edit author = [&](const UsdStageRefPtr &stage) {
        EditInSession(stage);
        const UsdPrim target = stage->GetPrimAtPath(prim);
        CHECK(target);
        if (!target) return;
        UsdAttribute attribute = target.GetAttribute(restRx);
        if (!attribute) {
            attribute =
                target.CreateAttribute(restRx, SdfValueTypeNames->Double);
        }
        CHECK(attribute.Set(value));
    };
    const bool mode =
        true;
    const std::vector<RigExecRigPose> reference =
        RunWithReferenceChecks(examplesDir, author, mode);
    if (reference.size() != kFrames.size()) {
        return;
    }

    UsdStageRefPtr stage = UsdStage::Open(StagePath(examplesDir));
    CHECK(stage);
    if (!stage) return;
    RigExecRigEvaluator rig(stage, kRig);
    rig.cpuReference = true;
    std::vector<std::string> errors;
    CHECK(rig.Compile(&errors));
    const size_t digest = rig.GetBindingEpochDigest();
    const size_t epochRests = rig.GetEpochRestFrameCount();
    CHECK(epochRests > 0);
    std::vector<RigExecRigPose> before;
    for (double frame : kFrames) {
        before.push_back(rig.Evaluate(UsdTimeCode(frame)));
        CHECK(before.back().valid);
        CHECK(before.back().referenceMismatches == 0);
    }
    CHECK(rig.GetBakedProgram() != nullptr);

    author(stage);
    for (size_t i = 0; i < kFrames.size(); ++i) {
        const RigExecRigPose pose = rig.Evaluate(UsdTimeCode(kFrames[i]));
        CHECK(pose.valid);
        CHECK(pose.referenceMismatches == 0);
        for (const std::string &diagnostic : pose.diagnostics) {
            CHECK(diagnostic.find("epoch rebuilt") == std::string::npos);
        }
        // Still the program's epoch: the oracle ran beside it, not instead.
        CHECK(rig.GetBakedProgram() != nullptr);
        ComparePoses(what + ", parity check, frame " +
                         std::to_string(int(kFrames[i])),
                     reference[i], pose);
    }
    CHECK(rig.GetBindingEpochDigest() == digest);
    CHECK(rig.GetEpochRestFrameCount() == epochRests);

    bool moved = false;
    for (const auto &[path, matrix] : reference[0].jointMatricesFinal) {
        const auto found = before[0].jointMatricesFinal.find(path);
        if (found == before[0].jointMatricesFinal.end() ||
            found->second != matrix) {
            moved = true;
        }
    }
    CHECK(moved);
}

void
TestARestValueEditAfterCompileReachesTheOracle(const std::string &examplesDir)
{
    CheckARestValueEditAfterCompile(examplesDir, kJoint,
                                    "a rest:rx edit after Compile");
}

void
TestARestValueEditAfterCompileReachesTheParityOracle(
    const std::string &examplesDir)
{
    CheckARestValueEditUnderTheParityCheck(examplesDir, kJoint,
                                           "a rest:rx edit after Compile");
}

void
TestARestValueEditOnAnAncestorReachesTheOracle(const std::string &examplesDir)
{
    CheckARestValueEditAfterCompile(examplesDir, kJoint.GetParentPath(),
                                    "a rest:rx edit on an ancestor");
}

void
TestARestValueEditOnAnAncestorReachesTheParityOracle(
    const std::string &examplesDir)
{
    CheckARestValueEditUnderTheParityCheck(examplesDir, kJoint.GetParentPath(),
                                           "a rest:rx edit on an ancestor");
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
        std::printf("usage: testRigExecEpochRests <examplesDir>\n");
        return 2;
    }
    const std::string examplesDir = argv[1];
    const std::string resources = SchemaResourceDir(examplesDir);
    if (PlugRegistry::GetInstance().RegisterPlugins(resources).empty()) {
        std::printf("FATAL: no schema plugin found at %s\n",
                    resources.c_str());
        return 2;
    }

    TestAnOrdinaryRigHoldsItsRests(examplesDir);
    TestATimeSampledRestIsPulledPerFrame(examplesDir);
    TestARestThatBecomesTimeSampledAfterCompile(examplesDir);
    TestAConnectedRestIsPulledPerFrame(examplesDir);
    TestAConnectedRestSpaceIsPulledPerFrame(examplesDir);
    TestARestSpaceConnectedToAComputedSpaceHasAProducer(examplesDir);
    TestARestThatMovesAloneStillRecomposes(examplesDir);
    TestAPropertyChainOnARestRefusesTheEpochPath(examplesDir);
    TestAnInteractiveOverrideOnARestIsFollowed(examplesDir);
    TestAnOverrideElsewhereLeavesTheRestsAlone(examplesDir);
    TestAnUnavailableRequiredProviderRefusesCompile(examplesDir);
    TestARestValueEditAfterCompileReachesTheOracle(examplesDir);
    TestARestValueEditOnAnAncestorReachesTheOracle(examplesDir);
    TestARestValueEditAfterCompileReachesTheParityOracle(examplesDir);
    TestARestValueEditOnAnAncestorReachesTheParityOracle(examplesDir);

    if (failures) {
        std::printf("%d FAILURE(S)\n", failures);
        return 1;
    }
    std::printf("testRigExecEpochRests: all tests passed\n");
    return 0;
}
