// Native evaluator cache/history, independent scalar references, literal
// provider arithmetic and exact pose comparator regression coverage.
#include "rigExecFrameRecordCheck.h"
#include "rigExecPoseCompare.h"

#include "rigExec/bakedProgram.h"
#include "rigExec/moverGraph.h"
#include "rigExec/rigEvaluator.h"
#include "serialPoseCompare.h"

#include "pxr/base/plug/registry.h"
#include "pxr/base/tf/getenv.h"
#include "pxr/base/tf/pathUtils.h"
#include "pxr/base/tf/fileUtils.h"
#include "pxr/base/tf/stringUtils.h"
#include "pxr/usd/sdf/types.h"
#include "pxr/usd/usd/editTarget.h"
#include "pxr/usd/usd/prim.h"
#include "pxr/usd/usd/primRange.h"
#include "pxr/usd/usd/relationship.h"
#include "pxr/usd/usd/stage.h"
#include "pxr/usd/usdGeom/xform.h"

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <functional>
#include <iterator>
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

// Published-generation equality is defined once in the shared header.
// Everything below reports into `failures`.
using rigExecTest::SameFrame;

template <class Map, class Equal>
static void
CompareMaps(const char *what, const std::string &where, const Map &reference,
            const Map &baked, Equal equal)
{
    rigExecTest::CompareMaps(&failures, what, where, reference, baked, equal);
}


// A consumer that disabled the observational guides gets none from either
// path: the dynamic walk skips the guide request, and the program must skip
// its publication too, or the parity check reports one mismatch per solver.

// One joint's final frame, so a test can say "the answer moved".
static RigExecPointFrame
JointFrame(const RigExecRigPose &pose, const SdfPath &joint)
{
    const auto found = pose.jointFramesFinal.find(joint);
    return found == pose.jointFramesFinal.end() ? RigExecPointFrame()
                                                : found->second;
}

static SdfPath
FirstControlWithAvar(const UsdStageRefPtr &stage, const SdfPath &rigPath,
                     const TfToken &avar)
{
    for (const UsdPrim &prim : UsdPrimRange(stage->GetPrimAtPath(rigPath))) {
        if (prim.GetTypeName() == "RigExecControl" &&
            prim.GetAttribute(avar)) {
            return prim.GetPath();
        }
    }
    return SdfPath();
}

static void
TestAnEditAfterTheBakeIsFollowed(const std::string &examplesDir)
{
    UsdStageRefPtr stage = UsdStage::Open(examplesDir + "/biped/Biped.usda");
    CHECK(stage);
    if (!stage) return;
    const SdfPath rigPath = FindRig(stage);
    if (rigPath.IsEmpty()) { ++failures; return; }
    const TfToken avar("avars:tx");
    const SdfPath control = FirstControlWithAvar(stage, rigPath, avar);
    CHECK(!control.IsEmpty());
    if (control.IsEmpty()) return;

    RigExecRigEvaluator rig(stage, rigPath);
    std::vector<std::string> errors;
    CHECK(rig.Compile(&errors));
    CHECK(rig.IsBakeable(nullptr));
    const RigExecRigPose before = rig.Evaluate(UsdTimeCode(1.0));
    CHECK(before.valid);

    // Author into the session layer, which is a value edit: the epoch digest
    // hashes it, but a program that captured the old value would not notice
    // on its own.
    stage->SetEditTarget(UsdEditTarget(stage->GetSessionLayer()));
    UsdAttribute attribute =
        stage->GetPrimAtPath(control).GetAttribute(avar);
    double authored = 0;
    attribute.Get(&authored);
    CHECK(attribute.Set(authored + 3.0));

    const RigExecRigPose after = rig.Evaluate(UsdTimeCode(1.0));
    CHECK(after.valid);

    // A freshly compiled evaluator checks that retained caches followed the edit.
    UsdStageRefPtr referenceStage =
        UsdStage::Open(examplesDir + "/biped/Biped.usda");
    referenceStage->SetEditTarget(
        UsdEditTarget(referenceStage->GetSessionLayer()));
    CHECK(referenceStage->GetPrimAtPath(control)
              .GetAttribute(avar)
              .Set(authored + 3.0));
    RigExecRigEvaluator referenceRig(referenceStage, rigPath);
    CHECK(referenceRig.Compile(&errors));
    const RigExecRigPose reference = referenceRig.Evaluate(UsdTimeCode(1.0));

    CompareMaps("final joint frame after an edit", "Biped.usda",
                reference.jointFramesFinal, after.jointFramesFinal,
                SameFrame);
    // And the edit actually moved something, so the comparison above is not
    // passing because nothing happened.
    bool moved = false;
    for (const auto &[path, frame] : after.jointFramesFinal) {
        if (!SameFrame(frame, JointFrame(before, path))) { moved = true; break; }
    }
    CHECK(moved);
}

// An epoch compiled for the program commits its solver batches without the
// solver-input index, and the first dynamic generation builds it -- so the
// index a dynamic session routes edits through is, after a mode switch, one
// built long after the compile, from what the compile handed over. The two
// edits below cover both sides of that build. The first lands while the
// index is still absent, when the handler takes every batch as reached and
// no batch has an answer to lose yet, so it pins only that the switch itself
// comes out right. The second lands after the build and has to reach its
// batch through the index, and that is the one a missing index entry fails.
// A rest edit on a joint the leg IK names, because it is the kind of edit
// only the index can deliver: it does not recompile the epoch (the digest
// hashes no rest channel), and what the batch reads of it comes off the
// stage rather than through the overrides its cache is keyed on -- so a
// batch the handler failed to dirty would hand back its cached, pre-edit
// answer, and the comparison with an evaluator that only ever ran
// dynamically would say so.
static void
TestASolverInputEditIsRoutedAfterADeferredCompile(
    const std::string &examplesDir)
{
    const SdfPath knee(
        "/Biped/Rig/Main/Shot/Aux/Joints/hips_def/pelvis_l_def/"
        "thigh_l_def/knee_l_def");
    const SdfPath ankle = knee.AppendChild(TfToken("ankle_l_def"));
    const TfToken restTx("rest:tx");
    UsdStageRefPtr stage = UsdStage::Open(examplesDir + "/biped/Biped.usda");
    CHECK(stage);
    if (!stage) return;
    const SdfPath rigPath = FindRig(stage);
    if (rigPath.IsEmpty()) { ++failures; return; }
    CHECK(stage->GetPrimAtPath(knee) && stage->GetPrimAtPath(ankle));
    if (!stage->GetPrimAtPath(knee) || !stage->GetPrimAtPath(ankle)) return;
    double kneeRest = 0, ankleRest = 0;
    stage->GetPrimAtPath(knee).GetAttribute(restTx).Get(&kneeRest);
    stage->GetPrimAtPath(ankle).GetAttribute(restTx).Get(&ankleRest);

    // What each edit should produce, from an evaluator that never deferred.
    const auto reference = [&](double kneeDelta, double ankleDelta) {
        UsdStageRefPtr referenceStage =
            UsdStage::Open(examplesDir + "/biped/Biped.usda");
        referenceStage->SetEditTarget(
            UsdEditTarget(referenceStage->GetSessionLayer()));
        CHECK(referenceStage->GetPrimAtPath(knee).GetAttribute(restTx)
                  .Set(kneeRest + kneeDelta));
        CHECK(referenceStage->GetPrimAtPath(ankle).GetAttribute(restTx)
                  .Set(ankleRest + ankleDelta));
        RigExecRigEvaluator referenceRig(referenceStage, rigPath);
        std::vector<std::string> errors;
        CHECK(referenceRig.Compile(&errors));
        return referenceRig.Evaluate(UsdTimeCode(1.0));
    };

    RigExecRigEvaluator rig(stage, rigPath);
    std::vector<std::string> errors;
    CHECK(rig.Compile(&errors));
    const RigExecRigPose baked = rig.Evaluate(UsdTimeCode(1.0));
    CHECK(baked.valid);

    stage->SetEditTarget(UsdEditTarget(stage->GetSessionLayer()));
    CHECK(stage->GetPrimAtPath(knee).GetAttribute(restTx)
              .Set(kneeRest + 1.5));
    const RigExecRigPose first = rig.Evaluate(UsdTimeCode(1.0));
    CHECK(first.valid);
    CompareMaps("final joint frame, edit before the index", "Biped.usda",
                reference(1.5, 0.0).jointFramesFinal, first.jointFramesFinal,
                SameFrame);

    CHECK(stage->GetPrimAtPath(ankle).GetAttribute(restTx)
              .Set(ankleRest + 1.0));
    const RigExecRigPose second = rig.Evaluate(UsdTimeCode(1.0));
    CHECK(second.valid);
    CompareMaps("final joint frame, edit after the index", "Biped.usda",
                reference(1.5, 1.0).jointFramesFinal, second.jointFramesFinal,
                SameFrame);
    // Both edits moved something, so neither comparison passes because
    // nothing happened.
    const auto movedFrom = [](const RigExecRigPose &a,
                              const RigExecRigPose &b) {
        for (const auto &[path, frame] : b.jointFramesFinal) {
            if (!SameFrame(frame, JointFrame(a, path))) return true;
        }
        return false;
    };
    CHECK(movedFrom(baked, first));
    CHECK(movedFrom(first, second));
}

static void
TestAnInteractiveOverrideAfterTheBakeIsFollowed(const std::string &examplesDir)
{
    UsdStageRefPtr stage = UsdStage::Open(examplesDir + "/biped/Biped.usda");
    CHECK(stage);
    if (!stage) return;
    const SdfPath rigPath = FindRig(stage);
    if (rigPath.IsEmpty()) { ++failures; return; }
    const TfToken avar("avars:tx");
    const SdfPath control = FirstControlWithAvar(stage, rigPath, avar);
    CHECK(!control.IsEmpty());
    if (control.IsEmpty()) return;

    RigExecRigEvaluator rig(stage, rigPath);
    std::vector<std::string> errors;
    CHECK(rig.Compile(&errors));
    const RigExecRigPose before = rig.Evaluate(UsdTimeCode(1.0));
    CHECK(before.valid);

    double authored = 0;
    stage->GetPrimAtPath(control).GetAttribute(avar).Get(&authored);
    rig.SetInteractiveOverrides(
        {RigExecValueOverride{control, TfToken(), avar,
                              VtValue(authored + 3.0)}});
    const RigExecRigPose dragged = rig.Evaluate(UsdTimeCode(1.0));
    CHECK(dragged.valid);

    UsdStageRefPtr referenceStage =
        UsdStage::Open(examplesDir + "/biped/Biped.usda");
    RigExecRigEvaluator referenceRig(referenceStage, rigPath);
    CHECK(referenceRig.Compile(&errors));
    referenceRig.SetInteractiveOverrides(
        {RigExecValueOverride{control, TfToken(), avar,
                              VtValue(authored + 3.0)}});
    const RigExecRigPose reference = referenceRig.Evaluate(UsdTimeCode(1.0));

    CompareMaps("final joint frame under an override", "Biped.usda",
                reference.jointFramesFinal, dragged.jointFramesFinal,
                SameFrame);
    bool moved = false;
    for (const auto &[path, frame] : dragged.jointFramesFinal) {
        if (!SameFrame(frame, JointFrame(before, path))) { moved = true; break; }
    }
    CHECK(moved);

    // Releasing the drag must not leave the rig stuck on either path.
    rig.ClearInteractiveOverrides();
    const RigExecRigPose released = rig.Evaluate(UsdTimeCode(1.0));
    CompareMaps("final joint frame after release", "Biped.usda",
                before.jointFramesFinal, released.jointFramesFinal,
                SameFrame);
}

// Invalidation: the index, and the two answers it has to give.

using _Edit = std::function<void(const UsdStageRefPtr &, const SdfPath &)>;

// Every published MAP domain of one generation, so a scenario compares the
// whole answer rather than the part it happened to think of. The scalars are
// separate (CompareGenerationScalars) because the mover-graph counters
// describe how warm an evaluator is, and several callers here hold a fresh
// evaluator against a running one on purpose.
// Each of these three carries the coverage its NAME carries in the shared
// header -- maps, scalars, both -- so that a test written later gets the
// comparison it asked for rather than the one this file happened to bind to
// the shortest name.
static void
CompareEveryMap(const std::string &where, const RigExecRigPose &reference,
                const RigExecRigPose &baked)
{
    rigExecTest::CompareEveryMap(&failures, where, reference, baked);
}

// The work counters, the override rounds, the convergence flag and the
// diagnostics in order -- for a test that is ABOUT the counters and says so.
static void
CompareGenerationScalars(const std::string &where,
                         const RigExecRigPose &reference,
                         const RigExecRigPose &baked)
{
    rigExecTest::CompareGenerationScalars(&failures, where, reference, baked);
}

// Compare every published map and ordered semantic diagnostic.
static void
ComparePose(const std::string &where, const RigExecRigPose &reference,
            const RigExecRigPose &baked)
{
    rigExecTest::ComparePose(&failures, where, reference, baked);
}

// Edit an existing native epoch, then compare eight generations with a
// fresh compile that saw the edited source before its first evaluation.
// \p expectRebuild is the other half of the contract and the reason this is
// not just another equality test: an edit that moved something the bake
// captured has to rebuild the program, and an edit that did not has to leave
// it standing. Both directions are asserted, because a program that rebuilds
// on every notice is also "never wrong" and is what this work replaced.
static void
TestEditAfterTheBake(const std::string &examplesDir, const char *stageName,
                     const char *what, const _Edit &edit, bool expectRebuild,
                     bool expectMoved = true)
{
    const std::string stagePath = examplesDir + "/" + std::string(stageName);
    const std::string where = std::string(what) + " on " + stageName;
    UsdStageRefPtr stage = UsdStage::Open(stagePath);
    CHECK(stage);
    if (!stage) return;
    const SdfPath rigPath = FindRig(stage);
    if (rigPath.IsEmpty()) { ++failures; return; }

    RigExecRigEvaluator rig(stage, rigPath);
    std::vector<std::string> errors;
    CHECK(rig.Compile(&errors));
    CHECK(rig.IsBakeable(nullptr));
    const RigExecRigPose before = rig.Evaluate(UsdTimeCode(1.0));
    CHECK(before.valid);
    // The edit starts with an existing native generation.
    CHECK(rig.GetBakedGenerationCount() == 1);
    const size_t builds = rig.GetBakedProgramBuildCount();
    CHECK(builds == 1);
    const size_t digestBefore = rig.GetBindingEpochDigest();

    edit(stage, rigPath);

    UsdStageRefPtr referenceStage = UsdStage::Open(stagePath);
    edit(referenceStage, rigPath);
    RigExecRigEvaluator referenceRig(referenceStage, rigPath);
    CHECK(referenceRig.Compile(&errors));

    bool moved = false;
    for (double frame = 1; frame <= 8; ++frame) {
        const RigExecRigPose reference =
            referenceRig.Evaluate(UsdTimeCode(frame));
        const RigExecRigPose baked = rig.Evaluate(UsdTimeCode(frame));
        CHECK(reference.valid);
        CHECK(baked.valid);
        if (!reference.valid || !baked.valid) continue;
        CompareEveryMap(where + " frame " + std::to_string(int(frame)),
                        reference, baked);
        for (const auto &[path, frames] : baked.jointFramesFinal) {
            const auto found = before.jointFramesFinal.find(path);
            if (found == before.jointFramesFinal.end() ||
                !SameFrame(frames, found->second)) {
                moved = true;
            }
        }
    }
    // An edit that changed nothing would let every comparison above pass
    // without the program having had to follow anything.
    if (moved != expectMoved) {
        ++failures;
        std::printf("FAIL %s: the edit %s the answer\n", where.c_str(),
                    expectMoved ? "did not move" : "unexpectedly moved");
    }
    // Every one of the eight was the program's: the edit settled into the
    // epoch before the first of them chose a path.
    CHECK(rig.GetBakedGenerationCount() == 9);
    if (expectRebuild) {
        if (rig.GetBakedProgramBuildCount() <= builds) {
            ++failures;
            std::printf("FAIL %s: the program was not rebuilt\n",
                        where.c_str());
        }
    } else if (rig.GetBakedProgramBuildCount() != builds) {
        ++failures;
        std::printf("FAIL %s: the program was rebuilt for an edit that "
                    "touched nothing it captured (digest %zu -> %zu)\n",
                    where.c_str(), digestBefore,
                    rig.GetBindingEpochDigest());
    }
}

// The first prim of \p type under \p rigPath, so a test names a FEATURE and
// not a path that the asset is free to move.
static UsdPrim
FirstPrimOfType(const UsdStageRefPtr &stage, const SdfPath &rigPath,
                const char *type)
{
    for (const UsdPrim &prim : UsdPrimRange(stage->GetPrimAtPath(rigPath))) {
        if (prim.GetTypeName() == type) {
            return prim;
        }
    }
    return UsdPrim();
}

static UsdPrim
FirstConstraint(const UsdStageRefPtr &stage, const SdfPath &rigPath)
{
    for (const UsdPrim &prim : UsdPrimRange(stage->GetPrimAtPath(rigPath))) {
        const std::string type = prim.GetTypeName().GetString();
        if (type.size() > 10 &&
            type.compare(type.size() - 10, 10, "Constraint") == 0) {
            return prim;
        }
    }
    return UsdPrim();
}

static void
EditInSession(const UsdStageRefPtr &stage)
{
    stage->SetEditTarget(UsdEditTarget(stage->GetSessionLayer()));
}


// The same comparison with the edit in place BEFORE the bake, which is the
// other thing a keyed overlay has to work under: the bake classifies an input
// that is ALREADY animated, rather than reclassifying one that became so.
static void
TestEditBeforeTheBake(const std::string &examplesDir, const char *stageName,
                      const char *what, const _Edit &edit)
{
    const std::string stagePath = examplesDir + "/" + std::string(stageName);
    const std::string where = std::string(what) + " on " + stageName;
    UsdStageRefPtr bakedStage = UsdStage::Open(stagePath);
    UsdStageRefPtr dynamicStage = UsdStage::Open(stagePath);
    CHECK(bakedStage && dynamicStage);
    if (!bakedStage || !dynamicStage) return;
    const SdfPath rigPath = FindRig(bakedStage);
    if (rigPath.IsEmpty()) { ++failures; return; }
    edit(bakedStage, rigPath);
    edit(dynamicStage, rigPath);

    RigExecRigEvaluator bakedRig(bakedStage, rigPath);
    RigExecRigEvaluator dynamicRig(dynamicStage, rigPath);
    std::vector<std::string> errors;
    CHECK(bakedRig.Compile(&errors));
    CHECK(dynamicRig.Compile(&errors));
    std::vector<std::string> reasons;
    if (!bakedRig.IsBakeable(&reasons)) {
        ++failures;
        std::printf("FAIL %s: not bakeable\n", where.c_str());
        for (const std::string &reason : reasons) {
            std::printf("    %s\n", reason.c_str());
        }
        return;
    }
    for (double frame = 1; frame <= 8; ++frame) {
        const RigExecRigPose reference = dynamicRig.Evaluate(UsdTimeCode(frame));
        const RigExecRigPose baked = bakedRig.Evaluate(UsdTimeCode(frame));
        CHECK(reference.valid);
        CHECK(baked.valid);
        if (!reference.valid || !baked.valid) continue;
        CompareEveryMap(where + " frame " + std::to_string(int(frame)),
                        reference, baked);
    }
    CHECK(bakedRig.GetBakedGenerationCount() == 8);
}

// The edits the scenarios above run. Each is written against a FEATURE of the
// biped rather than a path, so the asset can move without the test rotting.

// The IK/FK blend weight keyed, a constraint switched off, and a rest moved:
// three binding changes in one edit: a value becomes animated, a property
// is created, and a rest matrix is replaced in place.
static void
EditKeyedBlendDisabledConstraintAndRest(const UsdStageRefPtr &stage,
                                        const SdfPath &rigPath)
{
    EditInSession(stage);
    // The weight the blend reads through a connection, so the edit lands on
    // the far end of an input's walk rather than on the input itself.
    const UsdPrim blend =
        FirstPrimOfType(stage, rigPath, "RigExecBlendPointFrames");
    CHECK(blend);
    if (blend) {
        SdfPathVector connections;
        const UsdAttribute weight =
            blend.GetAttribute(TfToken("inputs:weight"));
        CHECK(weight);
        if (weight && weight.GetConnections(&connections) &&
            !connections.empty()) {
            const UsdAttribute source =
                stage->GetAttributeAtPath(connections[0]);
            CHECK(source);
            for (int frame = 1; frame <= 8; ++frame) {
                source.Set(float(frame) / 8.0f, UsdTimeCode(frame));
            }
        } else {
            ++failures;
        }
    }
    const UsdPrim constraint = FirstConstraint(stage, rigPath);
    CHECK(constraint);
    if (constraint) {
        constraint
            .CreateAttribute(TfToken("inputs:enabled"),
                             SdfValueTypeNames->Bool)
            .Set(false);
    }
    const UsdPrim joint = FirstPrimOfType(stage, rigPath, "RigExecJoint");
    CHECK(joint);
    if (joint) {
        const UsdAttribute rest = joint.GetAttribute(TfToken("rest:space"));
        CHECK(rest);
        GfMatrix4d space(1.0);
        if (rest && rest.Get(&space)) {
            space[3][1] += 1.5;
            rest.Set(space);
        }
    }
}

// The foot roll, which reaches the rig ONLY through property chains: it
// drives the envelope of the roll constraints through five float-math movers.
// The program runs those chains itself, off the authored stage, so nothing
// about the roll is captured.
// Two edits, because they are two different questions. Keying it makes the
// attribute animated, which is a STRUCTURAL change the epoch digest hashes --
// the rig recompiles and rebakes, and what is being tested is that eight
// keyed frames come out right. Moving its value is not: the chains re-read it
// every generation, so the answer has to follow with nothing rebuilt.
static void
EditKeyedFootRoll(const UsdStageRefPtr &stage, const SdfPath &rigPath)
{
    EditInSession(stage);
    size_t keyed = 0;
    for (const UsdPrim &prim : UsdPrimRange(stage->GetPrimAtPath(rigPath))) {
        // `avars:footRoll` since the foot dials moved onto the per-limb
        // param node: the dial
        // was `bank_?.foot:roll`, and params.py now MIGRATES it -- the
        // connections are re-pointed and the original deleted, so the
        // old name resolves to nothing and this test silently found no
        // attribute to key.
        const UsdAttribute roll =
            prim.GetAttribute(TfToken("avars:footRoll"));
        if (!roll) {
            continue;
        }
        for (int frame = 1; frame <= 8; ++frame) {
            roll.Set(float(frame) * 6.0f, UsdTimeCode(frame));
        }
        ++keyed;
    }
    CHECK(keyed > 0);
}

static void
EditFootRollValue(const UsdStageRefPtr &stage, const SdfPath &rigPath)
{
    EditInSession(stage);
    size_t edited = 0;
    for (const UsdPrim &prim : UsdPrimRange(stage->GetPrimAtPath(rigPath))) {
        // `avars:footRoll` since the foot dials moved onto the per-limb
        // param node: the dial
        // was `bank_?.foot:roll`, and params.py now MIGRATES it -- the
        // connections are re-pointed and the original deleted, so the
        // old name resolves to nothing and this test silently found no
        // attribute to key.
        const UsdAttribute roll =
            prim.GetAttribute(TfToken("avars:footRoll"));
        if (!roll) {
            continue;
        }
        float value = 0;
        roll.Get(&value);
        roll.Set(value + 24.0f);
        ++edited;
    }
    CHECK(edited > 0);
}

// A keyed avar's VALUE moved. The program re-reads it every frame through a
// retained query, so this must change the answer and rebuild nothing.
static void
EditAKeyedAvarValue(const UsdStageRefPtr &stage, const SdfPath &rigPath)
{
    // Into the layer that already holds the key, so this is a value moving
    // and not a property spec appearing -- the distinction the index draws.
    size_t edited = 0;
    for (const UsdPrim &prim : UsdPrimRange(stage->GetPrimAtPath(rigPath))) {
        const UsdAttribute avar = prim.GetAttribute(TfToken("avars:rz"));
        if (!avar || avar.GetNumTimeSamples() == 0) {
            continue;
        }
        for (int frame = 1; frame <= 8; ++frame) {
            double value = 0;
            avar.Get(&value, UsdTimeCode(frame));
            avar.Set(value * 2.0 + 1.0, UsdTimeCode(frame));
        }
        ++edited;
        break;
    }
    CHECK(edited == 1);
}

// A prim the rig has never heard of. The whole point of the index is that
// this costs nothing: the epoch is unchanged, no captured value moved, and
// the program keeps answering.
static void
TestAnUnrelatedEditLeavesTheProgramStanding(const std::string &examplesDir)
{
    UsdStageRefPtr stage = UsdStage::Open(examplesDir + "/biped/Biped.usda");
    CHECK(stage);
    if (!stage) return;
    const SdfPath rigPath = FindRig(stage);
    if (rigPath.IsEmpty()) { ++failures; return; }
    RigExecRigEvaluator rig(stage, rigPath);
    std::vector<std::string> errors;
    CHECK(rig.Compile(&errors));
    const RigExecRigPose before = rig.Evaluate(UsdTimeCode(1.0));
    CHECK(before.valid);
    const size_t builds = rig.GetBakedProgramBuildCount();

    stage->SetEditTarget(UsdEditTarget(stage->GetSessionLayer()));
    const UsdPrim scratch =
        stage->DefinePrim(SdfPath("/Scratch"), TfToken("Xform"));
    CHECK(scratch);
    const UsdAttribute note = scratch.CreateAttribute(
        TfToken("custom:note"), SdfValueTypeNames->Double);
    CHECK(note.Set(1.0));
    CHECK(note.Set(2.0));

    const RigExecRigPose after = rig.Evaluate(UsdTimeCode(1.0));
    CHECK(after.valid);
    CHECK(rig.GetBakedGenerationCount() == 2);
    if (rig.GetBakedProgramBuildCount() != builds) {
        ++failures;
        std::printf("FAIL unrelated edit: the program was rebuilt (%zu -> "
                    "%zu)\n", builds, rig.GetBakedProgramBuildCount());
    }
    CompareEveryMap("an unrelated edit on Biped.usda", before, after);
}

// An interactive override on a constraint's envelope. The avar case above is
// the animator dragging a control; this is the one that lands on a mover's
// own input, and it is placed by the same rule -- read the long way, through
// the resolved inputs the override was written into.
static void
TestAnOverrideOnAConstraintWeightIsFollowed(const std::string &examplesDir)
{
    const std::string stagePath = examplesDir + "/biped/Biped.usda";
    UsdStageRefPtr stage = UsdStage::Open(stagePath);
    CHECK(stage);
    if (!stage) return;
    const SdfPath rigPath = FindRig(stage);
    if (rigPath.IsEmpty()) { ++failures; return; }
    const UsdPrim constraint = FirstConstraint(stage, rigPath);
    CHECK(constraint);
    if (!constraint) return;
    const std::vector<RigExecValueOverride> overrides{RigExecValueOverride{
        constraint.GetPath(), TfToken(), TfToken("inputs:defaultWeight"),
        VtValue(0.25f)}};

    RigExecRigEvaluator rig(stage, rigPath);
    std::vector<std::string> errors;
    CHECK(rig.Compile(&errors));
    const RigExecRigPose before = rig.Evaluate(UsdTimeCode(1.0));
    CHECK(before.valid);
    rig.SetInteractiveOverrides(overrides);
    const RigExecRigPose dragged = rig.Evaluate(UsdTimeCode(1.0));
    CHECK(dragged.valid);
    // Placed, not fallen back to: an override the program declines is also a
    // correct answer, and would make this test prove nothing.
    CHECK(rig.GetBakedGenerationCount() == 2);

    UsdStageRefPtr referenceStage = UsdStage::Open(stagePath);
    RigExecRigEvaluator referenceRig(referenceStage, rigPath);
    CHECK(referenceRig.Compile(&errors));
    referenceRig.SetInteractiveOverrides(overrides);
    const RigExecRigPose reference = referenceRig.Evaluate(UsdTimeCode(1.0));
    CompareEveryMap("a constraint weight override on Biped.usda", reference,
                    dragged);

    // The constraint may drive a control rather than a joint, so both
    // domains count: what matters is that the override reached the walk.
    bool moved = false;
    for (const auto &[path, frame] : dragged.jointFramesFinal) {
        if (!SameFrame(frame, JointFrame(before, path))) { moved = true; break; }
    }
    for (const auto &[path, frame] : dragged.controlFrames) {
        const auto found = before.controlFrames.find(path);
        if (found == before.controlFrames.end() ||
            !SameFrame(frame, found->second)) {
            moved = true;
            break;
        }
    }
    CHECK(moved);
    rig.ClearInteractiveOverrides();
    const RigExecRigPose released = rig.Evaluate(UsdTimeCode(1.0));
    CompareEveryMap("a released constraint weight override on Biped.usda",
                    before, released);
}

// An unphased posed:space connection reads the raw authored attribute.
// The root's composed avar translation does not become a connected source.
static UsdStageRefPtr
MakeAConnectedSpaceRig()
{
    UsdStageRefPtr stage = UsdStage::CreateInMemory();
    stage->DefinePrim(SdfPath("/Asset"), TfToken("Scope"));
    stage->DefinePrim(SdfPath("/Asset/Rig"), TfToken("RigExecRoot"));
    const UsdPrim root = stage->DefinePrim(SdfPath("/Asset/Rig/Root"),
                                           TfToken("RigExecControl"));
    root.GetAttribute(TfToken("avars:tx")).Set(3.0);
    const UsdPrim joint = stage->DefinePrim(SdfPath("/Asset/Rig/Bone"),
                                            TfToken("RigExecJoint"));
    joint.GetAttribute(TfToken("avars:ty")).Set(2.0);

    // Explicit phases are required to select a produced pose frame.
    joint.CreateAttribute(TfToken("posed:space"), SdfValueTypeNames->Matrix4d)
        .AddConnection(root.GetPath().AppendProperty(TfToken("posed:space")));

    stage->DefinePrim(SdfPath("/Asset/Geom"), TfToken("Scope"));
    const UsdPrim mesh = stage->DefinePrim(SdfPath("/Asset/Geom/Slab"),
                                           TfToken("Mesh"));
    VtVec3fArray points{GfVec3f(0, 0, 0), GfVec3f(1, 0, 0), GfVec3f(0, 1, 0)};
    mesh.GetAttribute(TfToken("points")).Set(points);
    stage->DefinePrim(SdfPath("/Asset/Rig/Movers"), TfToken("Scope"));
    const UsdPrim skin = stage->DefinePrim(
        SdfPath("/Asset/Rig/Movers/Skin"), TfToken("RigExecSkinMover"));
    skin.ApplyAPI(TfToken("RigExecMoverAPI"));
    skin.GetRelationship(TfToken("rigExec:moves"))
        .SetTargets({mesh.GetPath().AppendProperty(TfToken("points"))});
    skin.CreateRelationship(TfToken("rigExec:influences"))
        .SetTargets({joint.GetPath()});
    skin.CreateAttribute(TfToken("rigExec:elementSize"),
                         SdfValueTypeNames->Int).Set(1);
    skin.CreateAttribute(TfToken("rigExec:jointIndices"),
                         SdfValueTypeNames->IntArray).Set(VtIntArray{0, 0, 0});
    skin.CreateAttribute(TfToken("rigExec:jointWeights"),
                         SdfValueTypeNames->FloatArray)
        .Set(VtFloatArray{1.0f, 1.0f, 1.0f});
    return stage;
}

// A value that is keyed ONCE.
// UsdStage reports ValueMightBeTimeVarying() == false for an attribute whose
// strongest opinion is exactly one time sample of a non-composable type, while
// a Default read -- which is what the bake captures at -- never sees a time
// sample at all. So a single-keyed input is the one shape that can be
// classified as an epoch constant and then captured as the value it does NOT
// have. An animator's first pose key is exactly that shape, which is why
// these two edits are the ones worth spending fixtures on.

static void
EditOneKeyOnAControlAvar(const UsdStageRefPtr &stage, const SdfPath &rigPath)
{
    EditInSession(stage);
    const SdfPath control =
        FirstControlWithAvar(stage, rigPath, TfToken("avars:tx"));
    if (control.IsEmpty()) { ++failures; return; }
    const UsdAttribute avar =
        stage->GetPrimAtPath(control).GetAttribute(TfToken("avars:tx"));
    // One sample and no default opinion underneath it in the session layer.
    avar.Set(25.0, UsdTimeCode(1.0));
}

static void
EditOneKeyOnEveryConstraintEnable(const UsdStageRefPtr &stage,
                                  const SdfPath &rigPath)
{
    EditInSession(stage);
    size_t keyed = 0;
    for (const UsdPrim &prim : UsdPrimRange(stage->GetPrimAtPath(rigPath))) {
        const std::string type = prim.GetTypeName().GetString();
        if (type.size() > 10 &&
            type.compare(type.size() - 10, 10, "Constraint") == 0) {
            if (const UsdAttribute enabled =
                    prim.GetAttribute(TfToken("inputs:enabled"))) {
                enabled.Set(false, UsdTimeCode(1.0));
                ++keyed;
            }
        }
    }
    CHECK(keyed > 0);
}

// A skin mover at partial constant strength.
// The full-strength constant envelope is the one every unweighted mover gets,
// and both geometry loops skip the blend for it. Nothing in examples/ or
// tests/ had a PARTIAL constant envelope on a bakeable skin mover, so the
// two loops' agreement about when the skip is safe was never compared on a
// case where they could differ.
static void
EditHalfStrengthSkinEnvelope(const UsdStageRefPtr &stage,
                             const SdfPath &rigPath)
{
    EditInSession(stage);
    const UsdPrim skin = FirstPrimOfType(stage, rigPath, "RigExecSkinMover");
    CHECK(skin);
    if (!skin) return;
    UsdAttribute weight = skin.GetAttribute(TfToken("inputs:defaultWeight"));
    if (!weight) {
        weight = skin.CreateAttribute(TfToken("inputs:defaultWeight"),
                                      SdfValueTypeNames->Float);
    }
    weight.Set(0.5f);
}

// An interactive override on an attribute that is a connection SOURCE.
// A shared-envelope idiom: many constraints' inputs:defaultWeight connected to
// one upstream attribute, and the drag that switches them all off stands on
// the upstream one. The program classified each input by walking that
// connection chain, so the override belongs to every input on the walk -- not
// only to the head attribute the walk started at.

static SdfPath
ShareOneEnvelopeAcrossConstraints(const UsdStageRefPtr &stage,
                                  const SdfPath &rigPath)
{
    EditInSession(stage);
    const UsdPrim skin = FirstPrimOfType(stage, rigPath, "RigExecSkinMover");
    if (!skin) { ++failures; return SdfPath(); }
    UsdAttribute shared = skin.GetAttribute(TfToken("inputs:defaultWeight"));
    if (!shared) {
        shared = skin.CreateAttribute(TfToken("inputs:defaultWeight"),
                                      SdfValueTypeNames->Float);
    }
    shared.Set(1.0f);
    size_t connected = 0;
    for (const UsdPrim &prim : UsdPrimRange(stage->GetPrimAtPath(rigPath))) {
        const std::string type = prim.GetTypeName().GetString();
        if (type.size() <= 10 ||
            type.compare(type.size() - 10, 10, "Constraint") != 0) {
            continue;
        }
        UsdAttribute weight =
            prim.GetAttribute(TfToken("inputs:defaultWeight"));
        if (!weight) {
            weight = prim.CreateAttribute(TfToken("inputs:defaultWeight"),
                                          SdfValueTypeNames->Float);
        }
        // SetConnections, not AddConnection: an explicit list replaces the
        // weaker opinion rather than joining it, and a scalar input may
        // carry at most one connection.
        weight.SetConnections({shared.GetPath()});
        ++connected;
    }
    CHECK(connected > 0);
    return shared.GetPath();
}

static void
TestAnOverrideOnAConnectionSourceIsFollowed(const std::string &examplesDir)
{
    const std::string stagePath = examplesDir + "/biped/Biped_anim.usda";
    UsdStageRefPtr stage = UsdStage::Open(stagePath);
    UsdStageRefPtr referenceStage = UsdStage::Open(stagePath);
    CHECK(stage && referenceStage);
    if (!stage || !referenceStage) return;
    const SdfPath rigPath = FindRig(stage);
    if (rigPath.IsEmpty()) { ++failures; return; }
    const SdfPath shared = ShareOneEnvelopeAcrossConstraints(stage, rigPath);
    CHECK(ShareOneEnvelopeAcrossConstraints(referenceStage, rigPath) == shared);
    if (shared.IsEmpty()) return;

    const std::vector<RigExecValueOverride> overrides{RigExecValueOverride{
        shared.GetPrimPath(), TfToken(), shared.GetNameToken(),
        VtValue(0.0f)}};

    RigExecRigEvaluator rig(stage, rigPath);
    std::vector<std::string> errors;
    if (!rig.Compile(&errors)) {
        ++failures;
        for (const std::string &e : errors) std::printf("  ERR %s\n", e.c_str());
    }
    std::vector<std::string> reasons;
    if (!rig.IsBakeable(&reasons)) {
        ++failures;
        std::printf("FAIL shared envelope: not bakeable\n");
        for (const std::string &reason : reasons) {
            std::printf("    %s\n", reason.c_str());
        }
        return;
    }
    const RigExecRigPose before = rig.Evaluate(UsdTimeCode(1.0));
    CHECK(before.valid);
    rig.SetInteractiveOverrides(overrides);
    const RigExecRigPose dragged = rig.Evaluate(UsdTimeCode(1.0));
    CHECK(dragged.valid);
    CHECK(rig.GetBakedGenerationCount() == 2);

    // Compare the held value against the same value authored before compile.
    // This independently exercises interactive binding and authored sampling.
    CHECK(referenceStage->GetAttributeAtPath(shared).Set(0.0f));
    RigExecRigEvaluator referenceRig(referenceStage, rigPath);
    CHECK(referenceRig.Compile(&errors));
    const RigExecRigPose reference = referenceRig.Evaluate(UsdTimeCode(1.0));
    CHECK(reference.valid);
    CompareEveryMap("an override on a shared envelope, on Biped_anim.usda",
                    reference, dragged);

    // And it is a drag that DOES something, so an override silently dropped
    // on both paths could not pass this.
    size_t moved = 0;
    for (const auto &[path, frame] : dragged.jointFramesFinal) {
        if (!SameFrame(frame, JointFrame(before, path))) ++moved;
    }
    CHECK(moved > 0);
    rig.ClearInteractiveOverrides();
    CompareEveryMap("a released shared-envelope override, on Biped_anim.usda",
                    before, rig.Evaluate(UsdTimeCode(1.0)));
}

// Positive exact-comparator cases perturb each published domain and require
// the exact mismatch count and a diagnostic naming the changed value.

static RigExecPointFrame
MovedFrame(double dy)
{
    RigExecPointFrame frame;
    frame.points[0] += GfVec3d(0, dy, 0);
    return frame;
}

static void
CheckOneMismatch(const char *what, const RigExecRigPose &reference,
                 const RigExecRigPose &baked, const char *expectedSubstring)
{
    RigExecRigPose out;
    RigExecComparePoses(reference, baked, &out);
    if (out.comparisonMismatches != 1) {
        ++failures;
        std::printf("FAIL exact comparator: %s produced %zu mismatch(es), "
                    "expected 1\n", what, out.comparisonMismatches);
        return;
    }
    bool found = false;
    for (const std::string &diagnostic : out.diagnostics) {
        if (diagnostic.rfind("baked parity: ", 0) == 0 &&
            diagnostic.find(expectedSubstring) != std::string::npos) {
            found = true;
        }
    }
    if (!found) {
        ++failures;
        std::printf("FAIL exact comparator: %s reported no diagnostic "
                    "naming \"%s\"\n", what, expectedSubstring);
        for (const std::string &diagnostic : out.diagnostics) {
            std::printf("    %s\n", diagnostic.c_str());
        }
    }
}

static void
TestPoseComparatorFindsWhatIsThere()
{
    const SdfPath a("/Rig/Joints/a");
    const SdfPath b("/Rig/Joints/b");
    const SdfPath solver("/Rig/Solvers/s");
    const SdfPath weight("/Rig/Weights/w");
    const SdfPath volume("/Rig/Weights/v");

    // A pose the two paths agree on, so every case below differs in one
    // domain and one domain only.
    RigExecRigPose agreed;
    agreed.jointFramesBase[a] = RigExecPointFrame();
    agreed.jointFramesFinal[a] = RigExecPointFrame();
    agreed.jointMatricesFinal[a] = GfMatrix4d(1.0);
    agreed.controlFrames[a] = RigExecPointFrame();
    agreed.providerXforms[a] = GfMatrix4d(1.0);
    agreed.providerBaseXforms[a] = GfMatrix4d(1.0);
    agreed.movedProperties[a.AppendProperty(TfToken("points"))] =
        VtValue(1.0f);
    agreed.solverFrames[solver] =
        std::vector<RigExecPointFrame>{RigExecPointFrame()};
    // The two weight domains and the convergence flag. Nothing that bakes
    // today publishes any of them, which is exactly why they are here: a
    // domain nobody fills is a domain nobody notices the comparator is blind
    // to, until the generation that fills it.
    agreed.weightFields[weight] =
        RigExecResolvedWeightField{a.AppendProperty(TfToken("points")),
                                   {1.0f, 0.5f, 0.0f}};
    agreed.weightFrames[volume] = GfMatrix4d(1.0);
    {
        RigExecRigPose out;
        RigExecComparePoses(agreed, agreed, &out);
        CHECK(out.comparisonMismatches == 0);
        CHECK(out.diagnostics.empty());
    }

    // Each of the ten compared domains, one at a time. Not a loop: the
    // point is that every domain is named, and a loop over a list would be
    // the same omission the comparison could make.
    {
        RigExecRigPose d = agreed;
        d.jointFramesBase[a] = MovedFrame(1.0);
        CheckOneMismatch("base joint frame", agreed, d, "base joint frame");
    }
    {
        RigExecRigPose d = agreed;
        d.jointFramesFinal[a] = MovedFrame(1.0);
        CheckOneMismatch("final joint frame", agreed, d, "final joint frame");
    }
    {
        RigExecRigPose d = agreed;
        d.jointMatricesFinal[a][3][1] = 1.0;
        CheckOneMismatch("joint matrix", agreed, d, "joint matrix");
    }
    {
        RigExecRigPose d = agreed;
        d.controlFrames[a] = MovedFrame(1.0);
        CheckOneMismatch("control frame", agreed, d, "control frame");
    }
    {
        RigExecRigPose d = agreed;
        d.providerXforms[a][3][1] = 1.0;
        CheckOneMismatch("provider transform", agreed, d,
                         "provider transform");
    }
    {
        RigExecRigPose d = agreed;
        d.providerBaseXforms[a][3][1] = 1.0;
        CheckOneMismatch("provider base transform", agreed, d,
                         "provider base transform");
    }
    {
        RigExecRigPose d = agreed;
        d.movedProperties[a.AppendProperty(TfToken("points"))] =
            VtValue(2.0f);
        CheckOneMismatch("moved property", agreed, d, "moved property");
    }
    {
        RigExecRigPose d = agreed;
        d.solverFrames[solver].push_back(RigExecPointFrame());
        CheckOneMismatch("solver frames", agreed, d, "solver frames");
    }
    // A resolved weight field is the pair (what it weights, the floats): a
    // field that weights the right property with the wrong numbers and one
    // that weights the wrong property with the right numbers are each a
    // different influence, and both have to be one mismatch.
    {
        RigExecRigPose d = agreed;
        d.weightFields[weight].weights[1] = 0.25f;
        CheckOneMismatch("weight field values", agreed, d, "weight field");
    }
    {
        RigExecRigPose d = agreed;
        d.weightFields[weight].target = b.AppendProperty(TfToken("points"));
        CheckOneMismatch("weight field target", agreed, d, "weight field");
    }
    // A field that ran one element short: the cardinality flips with what
    // the weight object names as its target, so a truncated field is a real
    // shape of disagreement rather than a hypothetical one.
    {
        RigExecRigPose d = agreed;
        auto &weights = d.weightFields[weight].weights;
        weights.resize(weights.size() - 1);
        CheckOneMismatch("a short weight field", agreed, d, "weight field");
    }
    {
        RigExecRigPose d = agreed;
        d.weightFrames[volume][3][1] = 1.0;
        CheckOneMismatch("weight frame", agreed, d, "weight frame");
    }

    // Order-sensitive: the diagnostics are a sequence, and the same lines in
    // a different order describe a different walk.
    {
        RigExecRigPose reference = agreed;
        reference.diagnostics = {"MoverFailed A", "constraint B"};
        RigExecRigPose d = agreed;
        d.diagnostics = {"constraint B", "MoverFailed A"};
        CheckOneMismatch("reordered diagnostics", reference, d,
                         "diagnostics differ");
    }
    {
        RigExecRigPose reference = agreed;
        reference.diagnostics = {"MoverFailed A"};
        CheckOneMismatch("a diagnostic only the reference has", reference,
                         agreed, "diagnostics differ");
    }

    // The two asymmetric cases, which are the ones a one-directional
    // comparison would miss: a key only the reference has, and a key only
    // the baked pose has.
    {
        RigExecRigPose reference = agreed;
        reference.jointFramesFinal[b] = RigExecPointFrame();
        CheckOneMismatch("a joint the program never published", reference,
                         agreed, "no final joint frame");
    }
    {
        RigExecRigPose d = agreed;
        d.jointFramesFinal[b] = RigExecPointFrame();
        CheckOneMismatch("a joint only the program published", agreed, d,
                         "unexpected final joint frame");
    }
    // And the same two directions on the domains that have no baked
    // counterpart yet: a program that publishes NO weight field for a mover
    // that consumed one is the exact shape this work has to catch.
    {
        RigExecRigPose reference = agreed;
        reference.weightFields[SdfPath("/Rig/Weights/x")] =
            RigExecResolvedWeightField{a.AppendProperty(TfToken("points")),
                                       {1.0f}};
        CheckOneMismatch("a weight field the program never published",
                         reference, agreed, "no weight field");
    }
    {
        RigExecRigPose d = agreed;
        d.weightFrames[SdfPath("/Rig/Weights/y")] = GfMatrix4d(1.0);
        CheckOneMismatch("a weight frame only the program published", agreed,
                         d, "unexpected weight frame");
    }

    // Every domain at once, so the count is a count and not a flag.
    {
        RigExecRigPose d = agreed;
        d.jointFramesBase[a] = MovedFrame(1.0);
        d.jointFramesFinal[a] = MovedFrame(1.0);
        d.jointMatricesFinal[a][3][1] = 1.0;
        d.controlFrames[a] = MovedFrame(1.0);
        d.providerXforms[a][3][1] = 1.0;
        d.providerBaseXforms[a][3][1] = 1.0;
        d.movedProperties[a.AppendProperty(TfToken("points"))] =
            VtValue(2.0f);
        d.solverFrames[solver].clear();
        d.weightFields[weight].weights[0] = 0.25f;
        d.weightFrames[volume][3][1] = 1.0;
        d.diagnostics = {"MoverFailed A"};
        RigExecRigPose out;
        RigExecComparePoses(agreed, d, &out);
        CHECK(out.comparisonMismatches == 11);
        CHECK(out.diagnostics.size() == 11);
    }
}

// Exact equality, not a tolerance -- on a real generation of the shipped rig,
// where "close" is the failure mode a tolerance would hide.
static void
TestPoseComparatorIsExact(const std::string &examplesDir)
{
    UsdStageRefPtr stage = UsdStage::Open(examplesDir + "/biped/Biped.usda");
    CHECK(stage);
    if (!stage) return;
    const SdfPath rigPath = FindRig(stage);
    if (rigPath.IsEmpty()) { ++failures; return; }
    RigExecRigEvaluator rig(stage, rigPath);
    std::vector<std::string> errors;
    CHECK(rig.Compile(&errors));
    const RigExecRigPose reference = rig.Evaluate(UsdTimeCode(1.0));
    CHECK(reference.valid);
    CHECK(!reference.jointMatricesFinal.empty());
    if (reference.jointMatricesFinal.empty()) return;

    // A copy with ONE number moved: everything else -- the diagnostics and
    // the work counters the comparator also compares -- has to stay equal,
    // or the count below stops being about the perturbation.
    RigExecRigPose perturbed = reference;
    perturbed.comparisonMismatches = 0;
    perturbed.jointMatricesFinal.begin()->second[3][1] += 1e-9;
    RigExecRigPose out;
    RigExecComparePoses(reference, perturbed, &out);
    CHECK(out.comparisonMismatches == 1);

    // And the same pose against itself is silent, so the case above is the
    // perturbation and not the comparison being noisy.
    RigExecRigPose quiet;
    RigExecComparePoses(reference, reference, &quiet);
    CHECK(quiet.comparisonMismatches == 0);
}

// An unrelated structural notice preserves a valid native epoch.
static void
TestDirtyEpochStillEvaluates(const std::string &examplesDir)
{
    const auto stage = UsdStage::Open(examplesDir + "/biped/Biped.usda");
    CHECK(stage);
    if (!stage) return;
    const SdfPath rigPath = FindRig(stage);
    if (rigPath.IsEmpty()) { ++failures; return; }
    RigExecRigEvaluator rig(stage, rigPath);
    std::vector<std::string> errors;
    CHECK(rig.Compile(&errors));
    std::vector<RigExecRigPose> before;
    for (double frame = 1; frame <= 5; ++frame) {
        before.push_back(rig.Evaluate(UsdTimeCode(frame)));
        CHECK(before.back().valid);
    }
    const size_t builds = rig.GetBakedProgramBuildCount();
    const size_t digest = rig.GetBindingEpochDigest();
    stage->DefinePrim(SdfPath("/Scratch"), TfToken("Scope"));
    CHECK(rig.IsBakeable());
    for (double frame = 1; frame <= 5; ++frame) {
        const auto after = rig.Evaluate(UsdTimeCode(frame));
        CHECK(after.valid);
        CompareEveryMap("an unrelated structural notice", before[size_t(frame)-1], after);
        CHECK(after.executedOpCount == rig.GetLastOpTrace().size());
    }
    CHECK(rig.GetBakedProgramBuildCount() == builds);
    CHECK(rig.GetBindingEpochDigest() == digest);
}

// A rest-channel edit is evaluated after the binding epoch has been built.
static void
MoveARestChannel(const UsdStageRefPtr &stage)
{
    EditInSession(stage);
    const UsdPrim hips =
        stage->GetPrimAtPath(SdfPath("/Biped/Rig/Main/Shot/Aux/Joints/hips_def"));
    CHECK(hips);
    if (!hips) return;
    UsdAttribute rest = hips.GetAttribute(TfToken("rest:tx"));
    if (!rest) {
        rest = hips.CreateAttribute(TfToken("rest:tx"),
                                    SdfValueTypeNames->Double);
    }
    CHECK(rest);
    if (rest) CHECK(rest.Set(3.0));
}


// The frames to sweep a stage at, from the stage's OWN authored range.
// The sweep used to run every example at frames 1, 2 and 3. The numbered
// examples are authored over 1001-1048, so all three reads held the first
// key and the sweep compared one static pose three times -- it had never
// compared an interpolated frame of any of them. Start, middle and end of
// the authored range instead, and 1-3 for a stage with no authored range,
// so a newly added example is swept at frames that differ the day it lands
// rather than the day someone adds it to a table. The biped overlay authors
// 1-200, so it sweeps start, middle and end like the numbered examples.
static std::vector<double>
_SweepFrames(const UsdStageRefPtr &stage)
{
    if (!stage->HasAuthoredTimeCodeRange()) {
        return {1, 2, 3};
    }
    const double start = stage->GetStartTimeCode();
    const double end = stage->GetEndTimeCode();
    if (!(end > start)) {
        return {start};
    }
    // Truncated, not rounded: a whole-numbered frame reads an authored key
    // on a stage keyed on whole frames and interpolates on one that is not,
    // and both are worth sweeping.
    return {start, start + double(long((end - start) / 2)), end};
}

static UsdStageRefPtr
MakeAConstrainedVolumeRig()
{
    UsdStageRefPtr stage = UsdStage::CreateInMemory();
    stage->DefinePrim(SdfPath("/Asset"), TfToken("Scope"));
    stage->DefinePrim(SdfPath("/Asset/Rig"), TfToken("RigExecRoot"));

    stage->DefinePrim(SdfPath("/Asset/Geom"), TfToken("Scope"));
    const UsdPrim mesh = stage->DefinePrim(SdfPath("/Asset/Geom/Slab"),
                                           TfToken("Points"));
    mesh.CreateAttribute(TfToken("points"), SdfValueTypeNames->Point3fArray)
        .Set(VtVec3fArray{GfVec3f(0, 0, 0), GfVec3f(0, 4, 0),
                          GfVec3f(0, 8, 0)});

    const UsdPrim lift = stage->DefinePrim(
        SdfPath("/Asset/Rig/Controls/Lift"), TfToken("RigExecControl"));
    lift.CreateAttribute(TfToken("avars:ty"), SdfValueTypeNames->Double)
        .Set(8.0);

    const UsdPrim sphere = stage->DefinePrim(
        SdfPath("/Asset/Rig/Weights/Sphere"), TfToken("RigExecSphereWeight"));
    sphere.CreateRelationship(TfToken("rigExec:weightTarget"))
        .SetTargets({SdfPath("/Asset/Geom/Slab.points")});
    sphere.CreateAttribute(TfToken("inputs:falloffMin"),
                           SdfValueTypeNames->Float).Set(0.0f);
    sphere.CreateAttribute(TfToken("inputs:falloffMax"),
                           SdfValueTypeNames->Float).Set(8.0f);
    sphere.CreateAttribute(TfToken("rigExec:falloffProfile"),
                           SdfValueTypeNames->Token).Set(TfToken("linear"));
    // Resolve the entering geometry version independently of the Base/Final
    // volume placement selected on the consuming mover.
    sphere.GetRelationship(TfToken("rigExec:weightTarget"))
        .SetMetadata(TfToken("rigExecReadPhase"), std::string("preceding"));

    // The constraint that moves the volume. This is the whole fixture.
    const UsdPrim move = stage->DefinePrim(
        SdfPath("/Asset/Rig/Movers/Pose/MoveVolume"),
        TfToken("RigExecPositionConstraint"));
    move.ApplyAPI(TfToken("RigExecMoverAPI"));
    move.CreateRelationship(TfToken("rigExec:moves"))
        .SetTargets({sphere.GetPath()});
    move.CreateRelationship(TfToken("rigExec:sources"))
        .SetTargets({lift.GetPath()});

    const UsdGeomXform pull =
        UsdGeomXform::Define(stage, SdfPath("/Asset/PullTo"));
    pull.MakeMatrixXform().Set(
        GfMatrix4d(1.0).SetTranslate(GfVec3d(6, 0, 0)));
    const UsdPrim constraint = stage->DefinePrim(
        SdfPath("/Asset/Rig/Movers/Sweep/Pull"),
        TfToken("RigExecPositionConstraint"));
    constraint.ApplyAPI(TfToken("RigExecMoverAPI"));
    constraint.CreateRelationship(TfToken("rigExec:moves"))
        .SetTargets({SdfPath("/Asset/Geom/Slab.points")});
    constraint.CreateRelationship(TfToken("rigExec:sources"))
        .SetTargets({pull.GetPath()});
    constraint.CreateRelationship(TfToken("rigExec:weightObject"))
        .SetTargets({sphere.GetPath()});
    return stage;
}


// Two constraints revise one joint. A named read sees the first revision,
// while Base and Final see the literal identity and second revision.

static UsdStageRefPtr
MakeAPoseWalkReadPhaseRig(const char *phase)
{
    UsdStageRefPtr stage = UsdStage::CreateInMemory();
    stage->DefinePrim(SdfPath("/Asset"), TfToken("Scope"));
    stage->DefinePrim(SdfPath("/Asset/Rig"), TfToken("RigExecRoot"));

    // Two sources, each moved by an avar, so the two revisions of the joint
    // land in different places and the phase has something to choose between.
    const auto control = [&](const char *name, double tx, double ty) {
        const UsdPrim prim = stage->DefinePrim(
            SdfPath(std::string("/Asset/Rig/Controls/") + name),
            TfToken("RigExecControl"));
        prim.CreateAttribute(TfToken("avars:tx"), SdfValueTypeNames->Double)
            .Set(tx);
        prim.CreateAttribute(TfToken("avars:ty"), SdfValueTypeNames->Double)
            .Set(ty);
        return prim;
    };
    const UsdPrim first = control("First", 3.0, 0.0);
    const UsdPrim second = control("Second", 0.0, 7.0);

    const SdfPath jointPath("/Asset/Rig/Joints/Arm");
    stage->DefinePrim(jointPath, TfToken("RigExecJoint"));

    // Both constraints revise the same joint, in namespace order: A leaves it
    // where First is, B then leaves it where Second is.
    const auto constrain = [&](const char *name, const UsdPrim &source) {
        const UsdPrim prim = stage->DefinePrim(
            SdfPath(std::string("/Asset/Rig/Movers/Constrain/") + name),
            TfToken("RigExecPositionConstraint"));
        prim.ApplyAPI(TfToken("RigExecMoverAPI"));
        prim.CreateRelationship(TfToken("rigExec:moves"))
            .SetTargets({jointPath});
        prim.CreateRelationship(TfToken("rigExec:sources"))
            .SetTargets({source.GetPath()});
        return prim;
    };
    const UsdPrim constraintA = constrain("A", first);
    constrain("B", second);

    stage->DefinePrim(SdfPath("/Asset/Geom"), TfToken("Scope"));
    const UsdPrim mesh = stage->DefinePrim(SdfPath("/Asset/Geom/Slab"),
                                           TfToken("Points"));
    mesh.CreateAttribute(TfToken("points"), SdfValueTypeNames->Point3fArray)
        .Set(VtVec3fArray{GfVec3f(0, 0, 0), GfVec3f(1, 0, 0),
                          GfVec3f(0, 1, 0)});

    const UsdPrim mover = stage->DefinePrim(
        SdfPath("/Asset/Rig/Movers/Geometry/Slide"),
        TfToken("RigExecMatrixMover"));
    mover.ApplyAPI(TfToken("RigExecMoverAPI"));
    mover.CreateRelationship(TfToken("rigExec:moves"))
        .SetTargets({mesh.GetPath().AppendProperty(TfToken("points"))});
    const UsdRelationship transform =
        mover.CreateRelationship(TfToken("rigExec:transform"));
    transform.SetTargets({jointPath});
    // The phase itself, as rigExecReadPhase metadata on the relationship.
    // "A" is spelled as the constraint's own path because an AtPrim phase is
    // an ABSOLUTE prim path and nothing else.
    const std::string authored = std::string(phase) == "atPrim"
                                     ? constraintA.GetPath().GetString()
                                     : std::string(phase);
    transform.SetMetadata(TfToken(RigExecReadPhaseMetadataName), authored);
    // Preserve the authored revision order: A writes first, B writes last.
    // The declared phase determines the geometry consumer dependency.
    stage->GetPrimAtPath(SdfPath("/Asset/Rig/Movers"))
        .SetChildrenReorder({TfToken("Geometry"), TfToken("Constrain")});
    stage->GetPrimAtPath(SdfPath("/Asset/Rig/Movers/Constrain"))
        .SetChildrenReorder({TfToken("B"), TfToken("A")});

    return stage;
}

// Check the phase against a literal translated triangle at every frame.
static VtVec3fArray
ReadPhaseMatchesLiteral(const char *what, const UsdStageRefPtr &stage,
                        const GfVec3f &translation)
{
    VtVec3fArray moved;
    const SdfPath rigPath = FindRig(stage);
    if (rigPath.IsEmpty()) { ++failures; return moved; }
    RigExecRigEvaluator rig(stage, rigPath);
    std::vector<std::string> errors;
    if (!rig.Compile(&errors)) {
        ++failures;
        std::printf("FAIL %s: the fixture does not compile\n", what);
        for (const std::string &error : errors) {
            std::printf("    %s\n", error.c_str());
        }
        return moved;
    }
    std::vector<std::string> reasons;
    if (!rig.IsBakeable(&reasons)) {
        ++failures;
        std::printf("FAIL %s: the fixture is not bakeable\n", what);
        for (const std::string &reason : reasons) {
            std::printf("    %s\n", reason.c_str());
        }
        return moved;
    }

    const VtVec3fArray expected{translation, translation + GfVec3f(1, 0, 0),
                                translation + GfVec3f(0, 1, 0)};
    for (double frame = 1; frame <= 4; ++frame) {
        const RigExecRigPose baked = rig.Evaluate(UsdTimeCode(frame));
        CHECK(baked.valid);
        CHECK(!baked.movedProperties.empty());
        const auto it =
            baked.movedProperties.find(SdfPath("/Asset/Geom/Slab.points"));
        if (it != baked.movedProperties.end() &&
            it->second.IsHolding<VtVec3fArray>()) {
            moved = it->second.UncheckedGet<VtVec3fArray>();
            CHECK(moved == expected);
        }
        CHECK(moved.size() == expected.size());
    }
    CHECK(rig.GetBakedGenerationCount() == 4);
    return moved;
}

static void
TestAReadPhaseOnTheTransformIsExact()
{
    const VtVec3fArray atPrim = ReadPhaseMatchesLiteral(
        "a pose-walk read phase on rigExec:transform",
        MakeAPoseWalkReadPhaseRig("atPrim"), GfVec3f(3, 0, 0));
    const VtVec3fArray final = ReadPhaseMatchesLiteral(
        "a final read phase on rigExec:transform",
        MakeAPoseWalkReadPhaseRig("final"), GfVec3f(0, 7, 0));
    const VtVec3fArray base = ReadPhaseMatchesLiteral(
        "a base read phase on rigExec:transform",
        MakeAPoseWalkReadPhaseRig("base"), GfVec3f(0, 0, 0));
    // The fixture discriminates, measured rather than assumed: if the phase
    // named a point the two shorthands already reach, a program that ignored
    // it entirely would pass every comparison above.
    CHECK(!atPrim.empty());
    CHECK(atPrim != final);
    CHECK(atPrim != base);
}

// The same AtPrim read held to the compiled phased-read store: Slide's
// list is A's record alone, and it answers exactly what that store does at
// the end of every run.
static void
TestAPoseWalkReadPhaseMatchesTheStore()
{
    const SdfPath rigPath("/Asset/Rig");
    RigExecRigEvaluator rig(MakeAPoseWalkReadPhaseRig("atPrim"), rigPath);
    std::vector<std::string> errors;
    CHECK(rig.Compile(&errors));
    for (double frame = 1; frame <= 4; ++frame) {
        const std::string where =
            "pose-walk records frame " + std::to_string(int(frame));
        const RigExecRigPose pose = rig.Evaluate(UsdTimeCode(frame));
        CHECK(pose.valid);
        CHECK(pose.comparisonMismatches == 0);
        CHECK(rigExecTest::CheckFrameRecords(&failures, where, rig) == 1);
    }
    CHECK(rig.GetBakedGenerationCount() == 4);
    const RigExecBakedProgram *program = rig.GetBakedProgram();
    CHECK(program != nullptr);
    if (!program) {
        return;
    }
    const RigExecBakedProgramImpl &B = program->GetStepGraph();
    const RigExecBakedProgramImpl::GeomRevision *slide =
        rigExecTest::FindRevision(B,
                                  SdfPath("/Asset/Rig/Movers/Geometry/Slide"));
    CHECK(slide != nullptr);
    if (slide) {
        CHECK(rigExecTest::RecordMovers(B, slide->transformRecords) ==
              std::vector<SdfPath>{SdfPath("/Asset/Rig/Movers/Constrain/A")});
    }
}

static void
TestEveryExampleStage(const std::string &examplesDir,
                      const std::vector<std::string> &only = {})
{
    auto stages=TfGlob({examplesDir+"/*.usd*",examplesDir+"/biped/*.usda"});
    std::sort(stages.begin(),stages.end());
    size_t evaluated=0,referenceChecked=0;
    for(const auto &path:stages) {
        if (!only.empty()) {
            bool wanted = false;
            for (const std::string &part : only) {
                if (path.find(part) != std::string::npos) {
                    wanted = true;
                    break;
                }
            }
            if (!wanted) {
                continue;
            }
        }
        const auto stage=UsdStage::Open(path); if(!stage)continue;
        const auto rigPath=FindRig(stage); if(rigPath.IsEmpty())continue;
        RigExecRigEvaluator rig(stage,rigPath);
        rig.cpuReference=true;
        std::vector<std::string> errors; CHECK(rig.Compile(&errors));
        CHECK(rig.IsBakeable());
        for(const auto frame:_SweepFrames(stage)) {
            const auto pose=rig.Evaluate(UsdTimeCode(frame)); CHECK(pose.valid);
            CHECK(pose.referenceMismatches==0);
            if(pose.referenceAgreements)++referenceChecked;
            CHECK(pose.executedOpCount==rig.GetLastOpTrace().size());
            ++evaluated;
        }
    }
    CHECK(evaluated>0 && referenceChecked>0);
}

// Intervening Xforms contribute to the one corrected rest and posed frame.
// D5 intentionally uses that same rest for Base and Final geometry matrices.

// \p animated keys the grouping transform instead of authoring it plainly;
// \p identityToday additionally makes every sample the identity at the frames
// the sweep reads, which is the case the "is it identity" test cannot see.
static UsdStageRefPtr
MakeAnInterveningXformRig(bool animated, bool identityToday = false)
{
    UsdStageRefPtr stage = UsdStage::CreateInMemory();
    stage->SetStartTimeCode(1.0);
    stage->SetEndTimeCode(3.0);
    stage->DefinePrim(SdfPath("/Asset"), TfToken("Scope"));
    stage->DefinePrim(SdfPath("/Asset/Rig"), TfToken("RigExecRoot"));

    // The grouping Xform contributes a translation above the provider.
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

// avars:rotationSign, the per-axis sign a mirrored limb declares: it
// multiplies rx/ry/rz (and rspin, which is an X rotation) before they are
// composed, so it is the identity at rest and a mirrored control answers the
// same typed value by turning the other way.
static UsdStageRefPtr
MakeARotationSignRig(const GfVec3d &sign, const GfVec3d &rotate)
{
    UsdStageRefPtr stage = UsdStage::CreateInMemory();
    stage->SetStartTimeCode(1.0);
    stage->SetEndTimeCode(3.0);
    stage->DefinePrim(SdfPath("/Asset"), TfToken("Scope"));
    stage->DefinePrim(SdfPath("/Asset/Rig"), TfToken("RigExecRoot"));
    const UsdPrim arm = stage->DefinePrim(SdfPath("/Asset/Rig/Arm"),
                                          TfToken("RigExecControl"));
    arm.GetAttribute(TfToken("avars:rotationSign")).Set(sign);
    static const char *const kChannels[3] = {"avars:rx", "avars:ry",
                                             "avars:rz"};
    for (int axis = 0; axis < 3; ++axis) {
        arm.GetAttribute(TfToken(kChannels[axis])).Set(rotate[axis]);
    }
    // A child two units out, so a sign that reaches the compose moves it and
    // a sign that does not leaves it where it was.
    const UsdPrim tip = stage->DefinePrim(SdfPath("/Asset/Rig/Arm/Tip"),
                                          TfToken("RigExecJoint"));
    tip.GetAttribute(TfToken("rest:space"))
        .Set(GfMatrix4d(1.0).SetTranslate(GfVec3d(0, 2, 0)));
    // Something the tip moves, so the comparison has a geometry domain to
    // compare and not only a pose: the sign reaches a mesh through the
    // same frame it reaches a joint matrix through.
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

static GfVec3d
RotationSignTip(const UsdStageRefPtr &stage)
{
    RigExecRigEvaluator rig(stage, SdfPath("/Asset/Rig"));
    CHECK(rig.Compile());
    const RigExecRigPose pose = rig.Evaluate(UsdTimeCode(1.0));
    CHECK(pose.valid);
    const auto tip = pose.jointFramesFinal.find(SdfPath("/Asset/Rig/Arm/Tip"));
    CHECK(tip != pose.jointFramesFinal.end());
    return tip == pose.jointFramesFinal.end() ? GfVec3d(0)
                                              : tip->second.points[0];
}

static void
TestRotationSignNegatesTheAvar()
{
    const GfVec3d plain(1, 1, 1);
    const GfVec3d mirrored(-1, -1, 1);
    const GfVec3d pose(30, 20, 40);
    const GfVec3d negated(-pose[0], -pose[1], pose[2]);

    // The whole claim: a signed control at +pose is the unsigned control at
    // the negated pose, and nothing else about it changes.
    const GfVec3d signedTip = RotationSignTip(
        MakeARotationSignRig(mirrored, pose));
    const GfVec3d byHand = RotationSignTip(
        MakeARotationSignRig(plain, negated));
    CHECK((signedTip - byHand).GetLength() < 1e-12);
    // And it is a real difference, not two ways of writing the same pose.
    const GfVec3d unsignedTip = RotationSignTip(
        MakeARotationSignRig(plain, pose));
    CHECK((signedTip - unsignedTip).GetLength() > 1e-3);

    // Identity at rest: declaring the sign cannot move a control nobody
    // posed, which is what lets a shipped rig declare it without re-fitting
    // the skin or anything constrained to the control.
    const GfVec3d rest(0, 0, 0);
    CHECK((RotationSignTip(MakeARotationSignRig(mirrored, rest)) -
           RotationSignTip(MakeARotationSignRig(plain, rest)))
              .GetLength() < 1e-12);

    // A malformed value is the unmirrored axis rather than a collapsed one:
    // the channel carries a sign, so the magnitude is discarded and zero
    // selects +1 (RigExecNormalizeRotationSign).
    CHECK((RotationSignTip(MakeARotationSignRig(GfVec3d(0, 7, -0.25), pose)) -
           RotationSignTip(MakeARotationSignRig(GfVec3d(1, 1, -1), pose)))
              .GetLength() < 1e-12);
}

// Rotation-sign edits and animated inputs are read in the current generation.
static void
TestRotationSignEditsAndAnimation()
{
    const GfVec3d plain(1,1,1),mirrored(-1,-1,1),rotation(30,20,40);
    const SdfPath path("/Asset/Rig");
    const auto stage=MakeARotationSignRig(plain,rotation);
    RigExecRigEvaluator rig(stage,path);CHECK(rig.Compile());
    CHECK(rig.Evaluate(UsdTimeCode(1)).valid);
    const auto builds=rig.GetBakedProgramBuildCount();
    const auto sign=stage->GetPrimAtPath(SdfPath("/Asset/Rig/Arm")).GetAttribute(TfToken("avars:rotationSign"));
    CHECK(sign.Set(mirrored));
    const auto changed=rig.Evaluate(UsdTimeCode(1));CHECK(changed.valid);
    CHECK(rig.GetBakedProgramBuildCount()==builds);
    const auto expected=RotationSignTip(MakeARotationSignRig(plain,GfVec3d(-30,-20,40)));
    const auto tip=changed.jointFramesFinal.find(SdfPath("/Asset/Rig/Arm/Tip"));
    CHECK(tip!=changed.jointFramesFinal.end());
    if(tip!=changed.jointFramesFinal.end())CHECK((tip->second.points[0]-expected).GetLength()<1e-12);
    // An unavailable direct static sign selects the original caller fallback,
    // never the previously sampled mirrored value.
    const auto unsignedExpected=RotationSignTip(MakeARotationSignRig(plain,rotation));
    CHECK(sign.Clear());
    const auto cleared=rig.Evaluate(UsdTimeCode(1));CHECK(cleared.valid);
    const auto clearedTip=cleared.jointFramesFinal.find(SdfPath("/Asset/Rig/Arm/Tip"));
    CHECK(clearedTip!=cleared.jointFramesFinal.end());
    if(clearedTip!=cleared.jointFramesFinal.end())CHECK((clearedTip->second.points[0]-unsignedExpected).GetLength()<1e-12);
    CHECK(rig.GetBakedProgramBuildCount()==builds);
    sign.Block();
    const auto blocked=rig.Evaluate(UsdTimeCode(1));CHECK(blocked.valid);
    const auto blockedTip=blocked.jointFramesFinal.find(SdfPath("/Asset/Rig/Arm/Tip"));
    CHECK(blockedTip!=blocked.jointFramesFinal.end());
    if(blockedTip!=blocked.jointFramesFinal.end())CHECK((blockedTip->second.points[0]-unsignedExpected).GetLength()<1e-12);
    CHECK(rig.GetBakedProgramBuildCount()==builds);
    CHECK(sign.Set(mirrored));
    const auto restored=rig.Evaluate(UsdTimeCode(1));CHECK(restored.valid);
    const auto restoredTip=restored.jointFramesFinal.find(SdfPath("/Asset/Rig/Arm/Tip"));
    CHECK(restoredTip!=restored.jointFramesFinal.end());
    if(restoredTip!=restored.jointFramesFinal.end())CHECK((restoredTip->second.points[0]-expected).GetLength()<1e-12);
    CHECK(rig.GetBakedProgramBuildCount()==builds);
    CHECK(sign.Clear());CHECK(sign.Set(plain,UsdTimeCode(1)));CHECK(sign.Set(mirrored,UsdTimeCode(3)));
    for(const auto frame:{1.0,3.0}) {
        const auto pose=rig.Evaluate(UsdTimeCode(frame));CHECK(pose.valid);
        const auto wanted=RotationSignTip(MakeARotationSignRig(plain,frame==1?rotation:GfVec3d(-30,-20,40)));
        const auto found=pose.jointFramesFinal.find(SdfPath("/Asset/Rig/Arm/Tip"));
        CHECK(found!=pose.jointFramesFinal.end());
        if(found!=pose.jointFramesFinal.end())CHECK((found->second.points[0]-wanted).GetLength()<1e-12);
    }
}

// mind about WHY is a failure rather than a pass.

static void
TestAnInterveningXformAboveAProvider()
{
    RigExecRigEvaluator rig(MakeAnInterveningXformRig(false),SdfPath("/Asset/Rig"));
    CHECK(rig.Compile());CHECK(rig.IsBakeable());
    const auto pose=rig.Evaluate(UsdTimeCode(1));CHECK(pose.valid);
    const auto found=pose.jointFramesFinal.find(SdfPath("/Asset/Rig/Group/Arm/Tip"));
    CHECK(found!=pose.jointFramesFinal.end());
    if(found!=pose.jointFramesFinal.end())CHECK(GfIsClose(found->second.points[0],GfVec3d(2,3,1),1e-9));
}

static void
TestAnAnimatedXformAboveAProvider()
{
    for(const bool identity:{false,true}) {
        RigExecRigEvaluator rig(MakeAnInterveningXformRig(true,identity),SdfPath("/Asset/Rig"));
        CHECK(rig.Compile());CHECK(rig.IsBakeable());
        for(const double frame:{1.0,2.0,3.0}) {
            const auto pose=rig.Evaluate(UsdTimeCode(frame));CHECK(pose.valid);
            const auto found=pose.jointFramesFinal.find(SdfPath("/Asset/Rig/Group/Arm/Tip"));
            CHECK(found!=pose.jointFramesFinal.end());
            const double y=identity?0:3+1.5*(frame-1);
            if(found!=pose.jointFramesFinal.end())CHECK(GfIsClose(found->second.points[0],GfVec3d(2,y,1),1e-9));
        }
    }
}

static void
TestInterveningXformUsesOneRest()
{
    // The same rig twice, with and without the grouping transform. Nothing
    // else differs, so every difference below is X(P) and only X(P).
    const UsdStageRefPtr withXform = MakeAnInterveningXformRig(false);
    const UsdStageRefPtr withoutXform = MakeAnInterveningXformRig(false);
    {
        bool resets = false;
        const std::vector<UsdGeomXformOp> ops =
            UsdGeomXform(
                withoutXform->GetPrimAtPath(SdfPath("/Asset/Rig/Group")))
                .GetOrderedXformOps(&resets);
        CHECK(ops.size() == 1);
        if (ops.size() != 1) return;
        ops.front().Set(GfVec3d(0, 0, 0));
    }

    const SdfPath rigPath("/Asset/Rig");
    RigExecRigEvaluator moved(withXform, rigPath);
    RigExecRigEvaluator plain(withoutXform, rigPath);
    CHECK(moved.Compile());
    CHECK(plain.Compile());
    const RigExecRigPose a = moved.Evaluate(UsdTimeCode(1.0));
    const RigExecRigPose b = plain.Evaluate(UsdTimeCode(1.0));
    CHECK(a.valid && b.valid);
    if (!a.valid || !b.valid) return;

    // 1. The published FRAMES follow the grouping Xform: the correction is
    //    what put them in the group's space.
    const SdfPath tipPath("/Asset/Rig/Group/Arm/Tip");
    const auto tipA = a.jointFramesFinal.find(tipPath);
    const auto tipB = b.jointFramesFinal.find(tipPath);
    CHECK(tipA != a.jointFramesFinal.end());
    CHECK(tipB != b.jointFramesFinal.end());
    if (tipA == a.jointFramesFinal.end() ||
        tipB == b.jointFramesFinal.end()) {
        return;
    }
    CHECK(std::abs((tipA->second.points[0][1] - tipB->second.points[0][1]) -
                   3.0) < 1e-9);

    // 2. The published MATRIX does not. Rest and pose are corrected
    //    together, so X cancels out of the rest-to-pose map -- translate
    //    (2, 0, 1) either way.
    const auto matrixA = a.jointMatricesFinal.find(tipPath);
    const auto matrixB = b.jointMatricesFinal.find(tipPath);
    CHECK(matrixA != a.jointMatricesFinal.end());
    CHECK(matrixB != b.jointMatricesFinal.end());
    if (matrixA != a.jointMatricesFinal.end() &&
        matrixB != b.jointMatricesFinal.end()) {
        CHECK(matrixA->second == matrixB->second);
        CHECK(GfIsClose(matrixA->second.ExtractTranslation(),
                        GfVec3d(2, 0, 1), 1e-9));
    }

    // D5 uses the same corrected rest for BASE geometry and joint matrices.
    // The grouping translation cancels in both rest-to-pose maps.
    const SdfPath target("/Asset/Geom/M.points");
    const auto pointsA = a.movedProperties.find(target);
    const auto pointsB = b.movedProperties.find(target);
    CHECK(pointsA != a.movedProperties.end());
    CHECK(pointsB != b.movedProperties.end());
    if (pointsA == a.movedProperties.end() ||
        pointsB == b.movedProperties.end()) {
        return;
    }
    const VtVec3fArray movedWith = pointsA->second.Get<VtVec3fArray>();
    const VtVec3fArray movedWithout = pointsB->second.Get<VtVec3fArray>();
    CHECK(movedWith.size() == movedWithout.size());
    if (movedWith.size() != movedWithout.size()) return;
    for (size_t i = 0; i < movedWith.size(); ++i) {
        if (!GfIsClose(movedWith[i],movedWithout[i],1e-5)) {
            ++failures;
            std::printf("FAIL D5 one rest: BASE moved point %zu differs "
                        "(%g %g %g against %g %g %g)\n",
                        i, movedWith[i][0], movedWith[i][1], movedWith[i][2],
                        movedWithout[i][0], movedWithout[i][1],
                        movedWithout[i][2]);
            break;
        }
    }
}

static void
TestConnectedSpaceReadsTheRawAttribute()
{
    const auto stage=MakeAConnectedSpaceRig();
    GfMatrix4d raw(1);raw.SetTranslateOnly(GfVec3d(5,0,0));
    CHECK(stage->GetPrimAtPath(SdfPath("/Asset/Rig/Root")).GetAttribute(TfToken("posed:space")).Set(raw));
    RigExecRigEvaluator rig(stage,SdfPath("/Asset/Rig"));
    CHECK(rig.Compile());CHECK(rig.IsBakeable());
    // A connected posed frame is the complete current frame: ORIGINAL
    // publishes it directly before reading this provider's own avars.
    std::string before;CHECK(stage->GetRootLayer()->ExportToString(&before));
    const auto pose=rig.Evaluate(UsdTimeCode(1));CHECK(pose.valid);
    const auto bone=pose.jointFramesFinal.find(SdfPath("/Asset/Rig/Bone"));
    CHECK(bone!=pose.jointFramesFinal.end());
    if(bone!=pose.jointFramesFinal.end())CHECK(GfIsClose(bone->second.points[0],GfVec3d(5,0,0),1e-12));
    const auto found=pose.movedProperties.find(SdfPath("/Asset/Geom/Slab.points"));
    CHECK(found!=pose.movedProperties.end());
    if(found!=pose.movedProperties.end()) {
        CHECK(found->second.IsHolding<VtVec3fArray>());
        if(found->second.IsHolding<VtVec3fArray>())CHECK((found->second.UncheckedGet<VtVec3fArray>()==VtVec3fArray{
            GfVec3f(5,0,0),GfVec3f(6,0,0),GfVec3f(5,1,0)}));
    }
    const auto held=rig.Evaluate(UsdTimeCode(1));CHECK(held.valid);
    ComparePose("connected posed attribute held",pose,held);
    CHECK(held.executedOpCount==0);
    std::string after;CHECK(stage->GetRootLayer()->ExportToString(&after));
    CHECK(after==before);
}

static void
TestConstrainedVolumePlacementsAreDistinct()
{
    for(const bool final:{false,true}) {
        const auto stage=MakeAConstrainedVolumeRig();
        stage->GetPrimAtPath(SdfPath("/Asset/Rig/Movers/Sweep/Pull")).GetRelationship(TfToken("rigExec:weightObject"))
            .SetMetadata(TfToken("rigExecReadPhase"),std::string(final?"final":"base"));
        RigExecRigEvaluator rig(stage,SdfPath("/Asset/Rig"));
        CHECK(rig.Compile());CHECK(rig.IsBakeable());
        const auto pose=rig.Evaluate(UsdTimeCode(1));CHECK(pose.valid);
        const auto found=pose.movedProperties.find(SdfPath("/Asset/Geom/Slab.points"));
        CHECK(found!=pose.movedProperties.end());
        if(found==pose.movedProperties.end() || !found->second.IsHolding<VtVec3fArray>())continue;
        const VtVec3fArray expected=final?VtVec3fArray{GfVec3f(0,0,0),GfVec3f(3,4,0),GfVec3f(6,8,0)}
            :VtVec3fArray{GfVec3f(6,0,0),GfVec3f(3,4,0),GfVec3f(0,8,0)};
        CHECK(found->second.UncheckedGet<VtVec3fArray>()==expected);
        CHECK(rig.Evaluate(UsdTimeCode(1)).executedOpCount==0);
    }
}

static std::string
_SchemaResourceDir(const std::string &examplesDir)
{
#ifdef RIGEXEC_SCHEMA_RESOURCE_DIR
    (void)examplesDir;
    return TfAbsPath(RIGEXEC_SCHEMA_RESOURCE_DIR);
#else
    return TfAbsPath(examplesDir + "/../plugin/rigExecSchema/resources");
#endif
}

int
main(int argc, char **argv)
{
    if (argc < 2) {
        std::printf("usage: testRigExecBakedMode <examplesDir>\n");
        return 2;
    }
    const std::string examplesDir = argv[1];
    const std::string resources = _SchemaResourceDir(examplesDir);
    if (PlugRegistry::GetInstance().RegisterPlugins(resources).empty()) {
        std::printf("FATAL: no schema plugin found at %s\n",
                    resources.c_str());
        return 2;
    }

    // One pass with the shadow on covers the plain assertions and the
    // wholesale-clear comparison. The separate scoped-clear process repeated
    // this suite.
    TfSetenv("RIGEXEC_VERIFY_SCOPED_CLEARS", "1");

    TestAnEditAfterTheBakeIsFollowed(examplesDir);
    TestASolverInputEditIsRoutedAfterADeferredCompile(examplesDir);
    TestAnInteractiveOverrideAfterTheBakeIsFollowed(examplesDir);
    TestAnOverrideOnAConstraintWeightIsFollowed(examplesDir);

    // Invalidation, both directions.
    TestEditAfterTheBake(examplesDir, "biped/Biped.usda",
                         "a keyed blend weight, a disabled constraint and a "
                         "moved rest",
                         EditKeyedBlendDisabledConstraintAndRest,
                         /* expectRebuild = */ true);
    TestEditAfterTheBake(examplesDir, "biped/Biped.usda",
                         "a moved foot roll", EditFootRollValue,
                         /* expectRebuild = */ false);
    TestEditBeforeTheBake(examplesDir, "biped/Biped.usda",
                          "a keyed foot roll", EditKeyedFootRoll);
    TestEditAfterTheBake(examplesDir, "biped/Biped_anim.usda",
                         "a keyed avar value", EditAKeyedAvarValue,
                         /* expectRebuild = */ false);
    TestAnUnrelatedEditLeavesTheProgramStanding(examplesDir);

    // A value keyed exactly once -- the shape USD reports as not
    // time-varying while a Default read still cannot see it.
    TestEditBeforeTheBake(examplesDir, "biped/Biped.usda",
                          "one key on a control avar",
                          EditOneKeyOnAControlAvar);
    TestEditBeforeTheBake(examplesDir, "biped/Biped.usda",
                          "one key on every constraint's enable",
                          EditOneKeyOnEveryConstraintEnable);
    // A partial constant envelope on a skin mover exercises blend assembly.
    TestEditBeforeTheBake(examplesDir, "biped/Biped_anim.usda",
                          "a half-strength skin envelope",
                          EditHalfStrengthSkinEnvelope);
    // An override standing on an attribute several hops up an input's
    // connection chain.
    TestAnOverrideOnAConnectionSourceIsFollowed(examplesDir);

    // Synthetic literal mismatches prove the exact comparator executes.
    TestPoseComparatorFindsWhatIsThere();
    TestPoseComparatorIsExact(examplesDir);

    // An unrelated structural notice preserves a valid native program.
    TestDirtyEpochStillEvaluates(examplesDir);


    // A read phase naming a point in the pose walk, which no shipped rig
    // authors and no other suite builds.
    TestAReadPhaseOnTheTransformIsExact();
    TestAPoseWalkReadPhaseMatchesTheStore();
    // The mirrored-limb rotation sign, in the compose and in the program.
    TestRotationSignNegatesTheAvar();

    TestRotationSignEditsAndAnimation();
    TestConnectedSpaceReadsTheRawAttribute();
    TestConstrainedVolumePlacementsAreDistinct();
    // Grouping transforms carry provider points through their own graph inputs.
    TestAnInterveningXformAboveAProvider();
    TestAnAnimatedXformAboveAProvider();
    TestInterveningXformUsesOneRest();

    // Every compiled example checks original independent scalar arithmetic.
    TestEveryExampleStage(examplesDir);

    // Serial executor on the large rigs, one small chain, and both edit
    // directions (rebuild, and leave the program standing). The shadow stays
    // off so this pass is the serial evaluator alone.
    {
        rigExecTest::EnvOverride shadowOff("RIGEXEC_VERIFY_SCOPED_CLEARS",
                                           "0");
        rigExecTest::EnvOverride serialEval("RIGEXEC_ENABLE_PARALLEL_EVAL",
                                            "0");
        TestEveryExampleStage(examplesDir,
                              {"/biped/", "01_FkChainTail.usda"});
        TestEditAfterTheBake(examplesDir, "biped/Biped.usda",
                             "a keyed blend weight, a disabled constraint and "
                             "a moved rest",
                             EditKeyedBlendDisabledConstraintAndRest,
                             /* expectRebuild = */ true);
        TestEditAfterTheBake(examplesDir, "biped/Biped.usda",
                             "a moved foot roll", EditFootRollValue,
                             /* expectRebuild = */ false);
    }

    if (failures) {
        std::printf("%d FAILURE(S)\n", failures);
        return 1;
    }
    std::printf("testRigExecBakedMode: all tests passed\n");
    return 0;
}
