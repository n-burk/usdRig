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
#include "pxr/usd/usdGeom/xform.h"

#include <cmath>
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

// ---------------------------------------------------------------------------
// A plain Xform standing BETWEEN a provider and its anchor.
//
// Exec resolves such a prim as the identity and drops it, so the rig would
// evaluate as if the grouping transform were not there at all. The dynamic
// walk composes it back in at evaluation (_ComposeInterveningXforms): X(P)
// lands BETWEEN a provider and its anchor, which is a uniform right-multiply
// only while every such Xform sits above every chain root, and is not one
// otherwise.
//
// The program refuses both shapes of it -- a non-identity transform, and one
// that is identity today but animates -- and refused them BLIND: no rig and
// no fixture in the tree tripped either, so there was no parity evidence
// either way and no way to tell a correct bake from a plausible one.
//
// These fixtures are that evidence. They assert the refusal by NAME and then
// assert the fallback generation is a plain dynamic evaluator's, every
// published domain of it, so the refusal cannot quietly become a wrong
// answer -- and the day the correction is baked, the one line each case
// carries flips from false to true.
//
// WHY IT IS STILL REFUSED, measured rather than assumed. The correction is
// not confined to the walk. It rewrites the REST frames as well as the base
// ones, and the two halves reach different consumers:
//
//   * pose.jointMatricesFinal and the walk's own finalMatrices are built
//     from the CORRECTED rest and the CORRECTED final, so they follow it;
//   * a geometry mover's base-phase matrix is exec's computeMatrix tap
//     (rigEvaluator.cpp, RigExecValueAddress::Prim(transform,
//     "computeMatrix")), which knows nothing about the grouping transform,
//     so it does NOT follow it;
//   * a solver's element rests come from exec's computeRestFrame through
//     computePointFrameArray, so they do not follow it either.
//
// The program holds ONE rest per slot and derives both matrices from it, so
// expressing that means holding an exec rest and a walk rest side by side
// and routing every consumer to the right one -- which is the same rework
// an animated rest:tx needs, and is why the animated case below is a second
// refusal rather than a second arm of the first. A bake that corrected the
// one rest the program has would agree with these fixtures' joint frames and
// silently move a skinned mesh and a solver's rests; that is exactly the
// shape of wrong the refusal is in front of.
// ---------------------------------------------------------------------------

// \p animated keys the grouping transform instead of authoring it plainly;
// \p identityToday additionally makes every sample the identity at the frames
// the sweep reads, which is the case the "is it identity" test cannot see.
UsdStageRefPtr
MakeAnInterveningXformRig(bool animated, bool identityToday = false)
{
    UsdStageRefPtr stage = UsdStage::CreateInMemory();
    stage->SetStartTimeCode(1.0);
    stage->SetEndTimeCode(3.0);
    stage->DefinePrim(SdfPath("/Asset"), TfToken("Scope"));
    stage->DefinePrim(SdfPath("/Asset/Rig"), TfToken("RigExecRoot"));

    // The grouping Xform. Every rig has these -- a Scope named Joints, a
    // Scope named Controls -- and they compose to the identity, which is
    // why the program's candidate list is not by itself a refusal. This one
    // carries a transform.
    const UsdGeomXform group =
        UsdGeomXform::Define(stage, SdfPath("/Asset/Rig/Group"));
    const UsdGeomXformOp op = group.AddTranslateOp();
    if (animated) {
        op.Set(GfVec3d(0, identityToday ? 0 : 3, 0), UsdTimeCode(1.0));
        op.Set(GfVec3d(0, identityToday ? 0 : 6, 0), UsdTimeCode(3.0));
    } else {
        op.Set(GfVec3d(0, 3, 0));
    }

    const UsdPrim joint = stage->DefinePrim(
        SdfPath("/Asset/Rig/Group/Arm"), TfToken("RigExecJoint"));
    joint.CreateAttribute(TfToken("avars:tx"), SdfValueTypeNames->Double)
        .Set(2.0);
    // A child of the joint, so the correction is exercised where it is NOT a
    // uniform right-multiply: the child's own anchor is the joint, which the
    // pass has already corrected.
    const UsdPrim tip = stage->DefinePrim(
        SdfPath("/Asset/Rig/Group/Arm/Tip"), TfToken("RigExecJoint"));
    tip.CreateAttribute(TfToken("avars:tz"), SdfValueTypeNames->Double)
        .Set(1.0);

    stage->DefinePrim(SdfPath("/Asset/Geom"), TfToken("Scope"));
    const UsdPrim mesh =
        stage->DefinePrim(SdfPath("/Asset/Geom/M"), TfToken("Points"));
    mesh.CreateAttribute(TfToken("points"), SdfValueTypeNames->Point3fArray)
        .Set(VtVec3fArray{GfVec3f(0, 0, 0), GfVec3f(1, 0, 0)});
    const UsdPrim mover = stage->DefinePrim(
        SdfPath("/Asset/Rig/Movers/M"), TfToken("RigExecMatrixMover"));
    mover.ApplyAPI(TfToken("RigExecMoverAPI"));
    mover.CreateRelationship(TfToken("rigExec:moves"))
        .SetTargets({SdfPath("/Asset/Geom/M.points")});
    mover.CreateRelationship(TfToken("rigExec:transform"))
        .SetTargets({tip.GetPath()});
    return stage;
}

// Compiles \p stage in baked mode, reports whether it bakes and why not, and
// compares every published domain against a dynamic evaluator over the
// sweep. Returns the baked generation count.
size_t
BakedAgreesWithDynamic(const char *what, const UsdStageRefPtr &stage,
                       const UsdStageRefPtr &referenceStage,
                       bool expectBakeable,
                       std::vector<std::string> *refusals = nullptr)
{
    const SdfPath rig("/Asset/Rig");
    RigExecRigEvaluator baked(stage, rig);
    baked.SetEvaluationMode(RigExecEvaluationMode::Baked);
    std::vector<std::string> errors;
    if (!baked.Compile(&errors)) {
        ++failures;
        std::printf("FAIL %s: the fixture does not compile\n", what);
        for (const std::string &error : errors) {
            std::printf("    %s\n", error.c_str());
        }
        return 0;
    }
    std::vector<std::string> reasons;
    const bool bakeable = baked.IsBakeable(&reasons);
    if (bakeable != expectBakeable) {
        ++failures;
        std::printf("FAIL %s: bakeable=%d, expected %d\n", what, int(bakeable),
                    int(expectBakeable));
    }
    if (refusals) {
        *refusals = reasons;
    }

    RigExecRigEvaluator reference(referenceStage, rig);
    errors.clear();
    CHECK(reference.Compile(&errors));
    for (double frame : {1.0, 2.0, 3.0}) {
        const RigExecRigPose expected = reference.Evaluate(UsdTimeCode(frame));
        const RigExecRigPose actual = baked.Evaluate(UsdTimeCode(frame));
        CHECK(expected.valid && actual.valid);
        CHECK(!actual.movedProperties.empty());
        ComparePoses(std::string(what) + " frame " +
                         std::to_string(int(frame)),
                     expected, actual);
    }
    return baked.GetBakedGenerationCount();
}

// Asserts that \p reasons names \p expected, so a refusal that changed its
// mind about WHY is a failure rather than a pass.
void
RefusalNames(const char *what, const std::vector<std::string> &reasons,
             const char *expected)
{
    for (const std::string &reason : reasons) {
        if (reason.find(expected) != std::string::npos) {
            return;
        }
    }
    ++failures;
    std::printf("FAIL %s: no refusal mentions \"%s\"\n", what, expected);
    for (const std::string &reason : reasons) {
        std::printf("    %s\n", reason.c_str());
    }
}

void
TestAnInterveningXformAboveAProvider()
{
    // The grouping transform reaches the pose: without it the tip would sit
    // at (2, 0, 1) and the mesh with it. Measured, not assumed, because a
    // program that dropped X(P) would agree with a reference that also
    // dropped it -- and the reference here is the DYNAMIC path, which does
    // not.
    const UsdStageRefPtr probeStage = MakeAnInterveningXformRig(false);
    RigExecRigEvaluator probe(probeStage, SdfPath("/Asset/Rig"));
    CHECK(probe.Compile());
    const RigExecRigPose pose = probe.Evaluate(UsdTimeCode(1.0));
    CHECK(pose.valid);
    const auto tip = pose.jointFramesFinal.find(
        SdfPath("/Asset/Rig/Group/Arm/Tip"));
    CHECK(tip != pose.jointFramesFinal.end());
    if (tip != pose.jointFramesFinal.end()) {
        CHECK(std::abs(tip->second.points[0][1] - 3.0) < 1e-9);
    }

    // The refusal, by name, and the fallback, in full.
    std::vector<std::string> refusals;
    const char *const what = "an intervening Xform above a provider";
    const size_t generations = BakedAgreesWithDynamic(
        what, MakeAnInterveningXformRig(false),
        MakeAnInterveningXformRig(false), /* expectBakeable = */ false,
        &refusals);
    RefusalNames(what, refusals, "intervening Xform above provider");
    // Nothing ran baked, and the comparison above still held: the fallback
    // is the dynamic path, not a program that answered anyway.
    CHECK(generations == 0);
}

void
TestAnAnimatedXformAboveAProvider()
{
    std::vector<std::string> refusals;
    const char *const what = "an animated Xform above a provider";
    const size_t generations = BakedAgreesWithDynamic(
        what, MakeAnInterveningXformRig(true), MakeAnInterveningXformRig(true),
        /* expectBakeable = */ false, &refusals);
    RefusalNames(what, refusals, "animated Xform above provider");
    CHECK(generations == 0);

    // And the shape an "is it identity today" test cannot see: every sample
    // the sweep reads IS the identity, so the transform composes to nothing
    // at every frame anyone looks at -- and it is still refused, because the
    // epoch is not a frame. A bake that judged the transform once would pass
    // this one and be wrong about the case above.
    refusals.clear();
    const char *const quietWhat =
        "an identity-valued animated Xform above a provider";
    const size_t quiet = BakedAgreesWithDynamic(
        quietWhat, MakeAnInterveningXformRig(true, /* identityToday = */ true),
        MakeAnInterveningXformRig(true, /* identityToday = */ true),
        /* expectBakeable = */ false, &refusals);
    RefusalNames(quietWhat, refusals, "animated Xform above provider");
    CHECK(quiet == 0);
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
    TestAPropertyChainOnARestRefusesTheEpochPath(examplesDir);
    TestAnInteractiveOverrideOnARestIsFollowed(examplesDir);
    TestAnOverrideElsewhereLeavesTheRestsAlone(examplesDir);
    TestAnInterveningXformAboveAProvider();
    TestAnAnimatedXformAboveAProvider();

    if (failures) {
        std::printf("%d FAILURE(S)\n", failures);
        return 1;
    }
    std::printf("testRigExecEpochRests: all tests passed\n");
    return 0;
}
