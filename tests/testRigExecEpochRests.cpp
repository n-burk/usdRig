//
// The epoch's rest frames: when they may be pulled once, and what has to put
// them back on the per-frame path.
//
// A rest frame is a function of rest:space and the six rest avars of a
// provider and of its RigExec ancestors, and of nothing else. When none of
// those can move within the epoch, every frame's answer is the same answer
// and the whole set is pulled once at Compile. The cases below are the ones
// where "once" is wrong, and each of them is invisible to the structure
// digest -- which hashes wiring, not values and not time codes -- so none of
// them recompiles by itself:
//
//   * a rest channel carrying time samples, or connected, or written by a
//     property chain: the frames are not epoch constants at all;
//   * the same three, authored AFTER the epoch was compiled;
//   * an interactive override standing on a rest channel: a drag is a value
//     that stands in for an authored one, so it has to move the rest exactly
//     as authoring it would, or the drag and the commit of that same drag
//     disagree -- and jointMatricesFinal is the rest->pose map, so a pose
//     that moved against a rest that did not is not a pose of this rig.
//
// Every case is checked against a REFERENCE stage carrying the same value
// authored plainly before it was ever compiled, because a rest edit moves a
// joint's rest frame and its posed frame together and comparing the two
// halves against each other proves nothing.
//
// argv[1] = path to the examples directory.
//
#include "rigExec/rigEvaluator.h"

#include "pxr/base/plug/registry.h"
#include "pxr/base/tf/pathUtils.h"
#include "pxr/usd/sdf/types.h"
#include "pxr/usd/usd/attribute.h"
#include "pxr/usd/usd/editTarget.h"
#include "pxr/usd/usd/prim.h"
#include "pxr/usd/usd/stage.h"

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

// ---------------------------------------------------------------------------

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
    CHECK(epochRests == 0);
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
    CHECK(rig.GetEpochRestFrameCount() == 0);
    CHECK(rig.GetBindingEpochDigest() == digest);
}

// A connected rest channel. A connection can reach anything, so it is refused
// without being followed -- but exec still follows it, and the rig has to
// agree with the value at the other end.
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
    CHECK(epochRests == 0);
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
//
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
    CHECK(epochRests == 0);
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

// The boundary of that walk, and the reason the case above is a rule and not
// a blanket permission: a connection that ENDS at one of the six computed
// spaces reaches a COMPUTATION -- here the space expression that follows the
// namespace parent -- and the walk would read the target's raw authored
// value instead. The program must hand that rig back rather than answer it.
//
// Bakeability only: this suite's ctest entries require the bake, so a case
// that EVALUATED an unbakeable rig would fail them by design.
void
TestARestSpaceConnectedToAComputedSpaceRefusesTheBake(
    const std::string &examplesDir)
{
    UsdStageRefPtr stage = UsdStage::Open(StagePath(examplesDir));
    CHECK(stage);
    if (!stage) return;
    EditInSession(stage);
    const UsdPrim parent = stage->GetPrimAtPath(kJoint.GetParentPath());
    CHECK(parent);
    if (!parent) return;
    RestSpace(stage).SetConnections(
        {parent.GetPath().AppendProperty(TfToken("parent:space"))});
    RigExecRigEvaluator rig(stage, kRig);
    std::vector<std::string> errors;
    CHECK(rig.Compile(&errors));
    std::vector<std::string> reasons;
    CHECK(!rig.IsBakeable(&reasons));
    bool named = false;
    for (const std::string &reason : reasons) {
        named = named ||
                reason.find("connected rest:space") != std::string::npos;
    }
    CHECK(named);
}

// A rest that moves while nothing else in the rig does.
//
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
    const std::vector<RigExecRigPose> subject = Run(
        examplesDir,
        [](const UsdStageRefPtr &stage) {
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
        },
        &epochRests);
    CHECK(epochRests == 0);
    CHECK(subject.size() == kFrames.size());
}

// An interactive override standing on a rest channel.
//
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
    TestARestSpaceConnectedToAComputedSpaceRefusesTheBake(examplesDir);
    TestARestThatMovesAloneStillRecomposes(examplesDir);
    TestAPropertyChainOnARestRefusesTheEpochPath(examplesDir);
    TestAnInteractiveOverrideOnARestIsFollowed(examplesDir);
    TestAnOverrideElsewhereLeavesTheRestsAlone(examplesDir);

    if (failures) {
        std::printf("%d FAILURE(S)\n", failures);
        return 1;
    }
    std::printf("testRigExecEpochRests: all tests passed\n");
    return 0;
}
