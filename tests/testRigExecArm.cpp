//
// RigExec end-to-end evaluation tests over the spec §4.5/§4.6 arm assets:
// OpenExec-evaluated transforms (controls, FK, IK, blend, joints), native
// USD animation resolution parity, and the staged geometry mover pipeline.
//
// argv[1] = path to the examples directory (containing ArmRig.usda and
// ArmShotAnim.usda). The codeless schema plugin is expected at
// <examples>/../plugin/rigExecSchema/resources.
//
#include "rigExec/rigEvaluator.h"
#include "rigExec/tapSet.h"
#include "rigExec/types.h"
#include "rigExecMath/pointFrame.h"
#include "rigExecMath/geometryKernels.h"
#include "rigExecMath/solvers.h"

#include "pxr/base/gf/rotation.h"
#include "pxr/base/plug/registry.h"
#include "pxr/base/tf/pathUtils.h"
#include "pxr/usd/sdf/types.h"
#include "pxr/usd/usd/attribute.h"
#include "pxr/usd/usd/prim.h"
#include "pxr/usd/usd/primRange.h"
#include "pxr/usd/usd/relationship.h"
#include "pxr/usd/usd/stage.h"

#include <cstdio>
#include <string>

using namespace rigExec;

static int failures = 0;

#define CHECK(cond)                                                        \
    do {                                                                   \
        if (!(cond)) {                                                     \
            ++failures;                                                    \
            std::printf("FAIL %s:%d: %s\n", __FILE__, __LINE__, #cond);    \
        }                                                                  \
    } while (0)

static bool
Near(const GfVec3d &a, const GfVec3d &b, double tol = 1e-6)
{
    return (a - b).GetLength() <= tol;
}

static std::array<GfVec3d, 4>
MatrixLandmarks(const GfMatrix4d &m)
{
    static const std::array<GfVec3d, 4> identity = {
        GfVec3d(0), GfVec3d(1, 0, 0), GfVec3d(0, 1, 0), GfVec3d(0, 0, 1)};
    std::array<GfVec3d, 4> out{};
    for (size_t i = 0; i < 4; ++i) {
        out[i] = m.TransformAffine(identity[i]);
    }
    return out;
}

static double
GetAvar(const UsdPrim &prim, const char *name, UsdTimeCode time)
{
    double value = 0;
    if (UsdAttribute a = prim.GetAttribute(TfToken(name))) {
        a.Get(&value, time);
    }
    return value;
}

// The Ir-contract local avar transform: XYZ-ordered rotations (degrees),
// then rspin about +X, then translation (row-vector convention).
static GfMatrix4d
ComposeAvars(const UsdPrim &prim, UsdTimeCode time)
{
    static const GfVec3d axes[3] = {
        GfVec3d(1, 0, 0), GfVec3d(0, 1, 0), GfVec3d(0, 0, 1)};
    const double angles[3] = {
        GetAvar(prim, "avars:rx", time), GetAvar(prim, "avars:ry", time),
        GetAvar(prim, "avars:rz", time)};
    GfMatrix4d m(1.0);
    for (int i = 0; i < 3; ++i) {
        if (angles[i] != 0.0) {
            m = m * GfMatrix4d(GfRotation(axes[i], angles[i]), GfVec3d(0));
        }
    }
    const double rspin = GetAvar(prim, "avars:rspin", time);
    if (rspin != 0.0) {
        m = m * GfMatrix4d(GfRotation(axes[0], rspin), GfVec3d(0));
    }
    GfMatrix4d t(1.0);
    t.SetTranslate(GfVec3d(GetAvar(prim, "avars:tx", time),
                           GetAvar(prim, "avars:ty", time),
                           GetAvar(prim, "avars:tz", time)));
    return m * t;
}

static GfMatrix4d
GetRestSpace(const UsdPrim &prim, UsdTimeCode time)
{
    GfMatrix4d rest(1.0);
    if (UsdAttribute a = prim.GetAttribute(TfToken("rest:space"))) {
        a.Get(&rest, time);
    }
    rest.Orthonormalize(false);
    return rest;
}

static std::array<GfVec3d, 4>
GetRestLandmarks(const UsdPrim &prim, UsdTimeCode time)
{
    return MatrixLandmarks(GetRestSpace(prim, time));
}

// Reconstructs an Ir-contract control's frame directly, for parity with
// the exec-computed result: posed = avars * rest (unconnected fallback).
static RigExecPointFrame
DirectControlFrame(const UsdPrim &control, UsdTimeCode time)
{
    RigExecPointFrame frame;
    frame.points = MatrixLandmarks(
        ComposeAvars(control, time) * GetRestSpace(control, time));
    frame.flags = RigExecPointFrameValid;
    return frame;
}

static const TfToken _computePointFrame("computePointFrame");
static const TfToken _computeMatrix("computeMatrix");
static const TfToken _computePointFrameArray("computePointFrameArray");

// View-free joints are posed by their solver through an in-memory binding
// that RigExecRigEvaluator resolves in Compile() and supplies to exec as a
// per-joint value override in Evaluate(). A joint's frame is therefore only
// complete through the evaluator, not from a raw source-stage tap: query it
// through RigExecRigPose (base phase == the old raw computePointFrame).
static RigExecPointFrame
CompiledJointFrame(const UsdStageRefPtr &stage, const SdfPath &rigPath,
                   const SdfPath &jointPath, UsdTimeCode time)
{
    RigExecRigEvaluator eval(stage, rigPath);
    std::vector<std::string> errors;
    CHECK(eval.Compile(&errors));
    for (const std::string &e : errors) {
        std::printf("  compile error: %s\n", e.c_str());
    }
    const RigExecRigPose pose = eval.Evaluate(time);
    const auto it = pose.jointFramesBase.find(jointPath);
    CHECK(it != pose.jointFramesBase.end());
    return it != pose.jointFramesBase.end() ? it->second
                                            : RigExecPointFrame();
}

static void
TestRestPose(const std::string &examplesDir)
{
    UsdStageRefPtr stage = UsdStage::Open(examplesDir + "/ArmRig.usda");
    CHECK(stage);
    if (!stage) return;

    RigExecTapSet taps(stage);
    const SdfPath shoulderPath("/ArmAsset/Rig/Joints/Shoulder");
    const SdfPath wristPath("/ArmAsset/Rig/Joints/Shoulder/Elbow/Wrist");
    const RigExecTapId shoulderTap = taps.Add(RigExecValueAddress::Prim(
        shoulderPath, _computePointFrame));
    const RigExecTapId wristTap = taps.Add(RigExecValueAddress::Prim(
        wristPath, _computePointFrame));
    const RigExecTapId wristMatrixTap = taps.Add(RigExecValueAddress::Prim(
        wristPath, _computeMatrix));
    const RigExecTapId fkTap = taps.Add(RigExecValueAddress::Prim(
        SdfPath("/ArmAsset/Rig/Solvers/FK"), _computePointFrameArray));
    taps.Prepare();

    const RigExecSnapshot snap = taps.Evaluate(UsdTimeCode::Default());
    CHECK(snap.IsValid());
    // Every requested tap produced a value (a defaulted value must never
    // masquerade as a result).
    CHECK(snap.IsComplete());

    // Rest pose in, rest pose out (weight 0 selects FK; FK inputs equal
    // their rests).
    const auto shoulder = snap.Get<RigExecPointFrame>(shoulderTap);
    const auto wrist = snap.Get<RigExecPointFrame>(wristTap);
    CHECK(shoulder.IsValid());
    CHECK(Near(shoulder.Origin(), GfVec3d(0, 10, 0)));
    CHECK(Near(wrist.Origin(), GfVec3d(8, 10, 0)));

    // Paired matrix view: rest pose gives identity.
    const auto wristMatrix = snap.Get<GfMatrix4d>(wristMatrixTap);
    CHECK(Near(wristMatrix.TransformAffine(GfVec3d(1, 2, 3)),
               GfVec3d(1, 2, 3), 1e-9));

    // Aggregate solver result is extractable and complete.
    const auto fk = snap.Get<RigExecPointFrameArray>(fkTap);
    CHECK(fk.GetSize() == 3);
    if (fk.GetSize() == 3) {
        CHECK(Near(fk.frames[2].Origin(), GfVec3d(8, 10, 0)));
    }
}

static void
TestIkAndBlend(const std::string &examplesDir)
{
    UsdStageRefPtr stage = UsdStage::Open(examplesDir + "/ArmRig.usda");
    CHECK(stage);
    if (!stage) return;

    // Author an IK goal (avars deltas from the rest at (8, 10, 0) to the
    // goal at (6, 13, 2)) and switch fully to IK before building.
    const SdfPath handIkPath("/ArmAsset/Rig/Controls/HandIK");
    const UsdPrim handIk = stage->GetPrimAtPath(handIkPath);
    handIk.GetAttribute(TfToken("avars:tx")).Set(-2.0);
    handIk.GetAttribute(TfToken("avars:ty")).Set(3.0);
    handIk.GetAttribute(TfToken("avars:tz")).Set(2.0);
    stage->GetAttributeAtPath(
        SdfPath("/ArmAsset/Rig/Solvers/IKFKBlend.inputs:weight"))
        .Set(1.0f);

    RigExecTapSet taps(stage);
    const RigExecTapId ikTap = taps.Add(RigExecValueAddress::Prim(
        SdfPath("/ArmAsset/Rig/Solvers/IK"), _computePointFrameArray));
    taps.Prepare();
    const RigExecSnapshot snap = taps.Evaluate(UsdTimeCode::Default());
    CHECK(snap.IsValid());

    // Parity: the exec-evaluated IK result must match the direct math
    // solve fed with directly reconstructed control frames.
    const UsdPrim root =
        stage->GetPrimAtPath(SdfPath("/ArmAsset/Rig/Controls/ShoulderFK"));
    const UsdPrim pole =
        stage->GetPrimAtPath(SdfPath("/ArmAsset/Rig/Controls/ElbowPole"));
    RigExecTwoBoneIkParams params;
    params.upperLength = 4;
    params.lowerLength = 4;
    // Authored as float attributes: read them back for exact parity with
    // the exec-evaluated inputs.
    params.stretch = double(1.0f);
    params.softness = double(0.15f);
    params.preferredBendRadians = -0.35;

    std::array<std::array<GfVec3d, 4>, 3> rests;
    rests[0] = GetRestLandmarks(root, UsdTimeCode::Default());
    const GfVec3d restAim = (rests[0][1] - rests[0][0]).GetNormalized();
    rests[1] = rests[0];
    for (auto &p : rests[1]) p += restAim * params.upperLength;
    rests[2] =
        GetRestLandmarks(handIk, UsdTimeCode::Default());

    const auto expected = RigExecSolveTwoBoneIk(
        DirectControlFrame(root, UsdTimeCode::Default()),
        DirectControlFrame(handIk, UsdTimeCode::Default()),
        DirectControlFrame(pole, UsdTimeCode::Default()),
        rests, params);

    const auto ik = snap.Get<RigExecPointFrameArray>(ikTap);
    CHECK(ik.GetSize() == 3);
    if (ik.GetSize() == 3) {
        for (int i = 0; i < 3; ++i) {
            for (int p = 0; p < 4; ++p) {
                const bool ok =
                    Near(ik.frames[i].points[p], expected[i].points[p], 1e-9);
                CHECK(ok);
                if (!ok) {
                    std::printf(
                        "  frame %d point %d exec (%g %g %g) "
                        "expected (%g %g %g)\n",
                        i, p,
                        ik.frames[i].points[p][0], ik.frames[i].points[p][1],
                        ik.frames[i].points[p][2],
                        expected[i].points[p][0], expected[i].points[p][1],
                        expected[i].points[p][2]);
                }
            }
        }
        // Aggregate rest sets travel with the frames.
        CHECK(ik.rests.size() == 3);
        if (ik.rests.size() == 3) {
            CHECK(Near(ik.rests[2][0], rests[2][0], 1e-9));
        }
    }

    // Full IK blend: the wrist joint publishes the IK end frame. Joints
    // are posed by the solver through the evaluator's in-memory binding
    // (view-free), so the joint frame comes from the compiled rig.
    const auto wrist = CompiledJointFrame(
        stage, SdfPath("/ArmAsset/Rig"),
        SdfPath("/ArmAsset/Rig/Joints/Shoulder/Elbow/Wrist"),
        UsdTimeCode::Default());
    CHECK(Near(wrist.Origin(), expected[2].Origin(), 1e-9));

    // The wrist reaches (or soft-clamps toward) the authored goal.
    CHECK((wrist.Origin() - GfVec3d(6, 13, 2)).GetLength() < 0.5);

    // Semantic lock (view-free): the solver binding exists only inside the
    // evaluator, so a RAW source-stage joint tap at this animated IK frame
    // returns the fallback pose, NOT the solved one.
    // Downstream consumers must read the compiled rig; this asserts the
    // documented break so it cannot silently regress (codex round-2 #8).
    RigExecTapSet rawTaps(stage);
    const RigExecTapId rawWristTap = rawTaps.Add(RigExecValueAddress::Prim(
        SdfPath("/ArmAsset/Rig/Joints/Shoulder/Elbow/Wrist"),
        _computePointFrame));
    rawTaps.Prepare();
    const auto rawWrist = rawTaps.Evaluate(UsdTimeCode::Default())
                              .Get<RigExecPointFrame>(rawWristTap);
    CHECK(!Near(rawWrist.Origin(), wrist.Origin(), 1e-6));
}

// View-free solver->joint binding validation is enforced in Phase A,
// BEFORE any epoch teardown, so an invalid binding rejects the compile
// without destroying a working epoch (codex round-2 findings 1, 4).
static void
TestViewFreeValidation(const std::string &examplesDir)
{
    const SdfPath rigPath("/ArmAsset/Rig");
    const SdfPath blendPath("/ArmAsset/Rig/Solvers/IKFKBlend");
    const TfToken jointsTok("rigExec:joints");

    // Baseline: the unmodified rig compiles.
    {
        UsdStageRefPtr stage = UsdStage::Open(examplesDir + "/ArmRig.usda");
        CHECK(stage);
        if (stage) {
            RigExecRigEvaluator eval(stage, rigPath);
            CHECK(eval.Compile());
        }
    }

    // A control listed as a solver output is rejected (not a RigExecJoint).
    {
        UsdStageRefPtr stage = UsdStage::Open(examplesDir + "/ArmRig.usda");
        CHECK(stage);
        if (stage) {
            UsdPrim blend = stage->GetPrimAtPath(blendPath);
            UsdRelationship jr = blend.GetRelationship(jointsTok);
            SdfPathVector t;
            jr.GetTargets(&t);
            t.push_back(SdfPath("/ArmAsset/Rig/Controls/HandIK"));
            jr.SetTargets(t);
            RigExecRigEvaluator eval(stage, rigPath);
            std::vector<std::string> errors;
            CHECK(!eval.Compile(&errors));
            CHECK(!errors.empty());
        }
    }

    // A joint posed by two solvers is rejected (the BLOCKER scenario: this
    // must fail in Phase A, not inside compilation after teardown).
    {
        UsdStageRefPtr stage = UsdStage::Open(examplesDir + "/ArmRig.usda");
        CHECK(stage);
        if (stage) {
            // FK already feeds the blend; also list a joint the blend owns.
            UsdPrim fk =
                stage->GetPrimAtPath(SdfPath("/ArmAsset/Rig/Solvers/FK"));
            CHECK(fk);
            fk.CreateRelationship(jointsTok).SetTargets(
                {SdfPath("/ArmAsset/Rig/Joints/Shoulder")});
            RigExecRigEvaluator eval(stage, rigPath);
            std::vector<std::string> errors;
            CHECK(!eval.Compile(&errors));
            CHECK(!errors.empty());
        }
    }

    // A non-solver prim under /Solvers carrying rigExec:joints is rejected:
    // it cannot publish computePointFrameArray, so it must never become a
    // joint's frame source (codex round-2 blocker).
    {
        UsdStageRefPtr stage = UsdStage::Open(examplesDir + "/ArmRig.usda");
        CHECK(stage);
        if (stage) {
            UsdPrim group = stage->DefinePrim(
                SdfPath("/ArmAsset/Rig/Solvers/Group"), TfToken("Scope"));
            group.CreateRelationship(jointsTok).SetTargets(
                {SdfPath("/ArmAsset/Rig/Joints/Shoulder")});
            RigExecRigEvaluator eval(stage, rigPath);
            std::vector<std::string> errors;
            CHECK(!eval.Compile(&errors));
            CHECK(!errors.empty());
        }
    }

    // A cardinality-shrinking edit (fewer FK controls than bound elements)
    // must change the structure digest so Evaluate() recompiles and Phase A
    // rejects the now-out-of-range binding atomically (codex round-3): the
    // digest, not just rigExec:joints, must cover cardinality inputs.
    {
        UsdStageRefPtr stage =
            UsdStage::Open(examplesDir + "/01_FkChainTail.usda");
        CHECK(stage);
        if (stage) {
            const SdfPath tailRig("/TailAsset/Rig");
            RigExecRigEvaluator eval(stage, tailRig);
            CHECK(eval.Compile());
            UsdPrim fk = stage->GetPrimAtPath(
                SdfPath("/TailAsset/Rig/Solvers/TailFK"));
            CHECK(fk);
            if (fk) {
                UsdRelationship controls =
                    fk.GetRelationship(TfToken("rigExec:controls"));
                SdfPathVector ct;
                controls.GetTargets(&ct);
                CHECK(ct.size() > 1);  // 4 joints bind elements 0..3
                if (ct.size() > 1) {
                    ct.resize(1);
                    controls.SetTargets(ct);  // now only element 0 is valid
                    const RigExecRigPose pose =
                        eval.Evaluate(UsdTimeCode::Default());
                    CHECK(!pose.valid);  // recompiled and rejected
                }
            }
        }
    }

    // A time-sampled cardinality is rejected even on a NON-joint-bearing
    // aggregate solver: 05's SpineRibbon drives geometry (no rigExec:joints)
    // but its sampleCount can still feed a Blend, so it must be static
    // (codex round-4 — enforcement must cover indirect aggregate solvers).
    {
        UsdStageRefPtr stage =
            UsdStage::Open(examplesDir + "/05_TwistRibbonSpine.usda");
        CHECK(stage);
        if (stage) {
            const SdfPath spineRig("/SpineAsset/Rig");
            RigExecRigEvaluator eval(stage, spineRig);
            std::vector<std::string> errors;
            CHECK(eval.Compile(&errors));  // valid baseline
            for (const std::string &e : errors) {
                std::printf("  05 compile error: %s\n", e.c_str());
            }
            UsdPrim ribbon = stage->GetPrimAtPath(
                SdfPath("/SpineAsset/Rig/Solvers/SpineRibbon"));
            CHECK(ribbon);
            if (ribbon) {
                // Add a time sample AFTER compile, EQUAL to the default so
                // the default value is unchanged. The digest must still
                // change (it records sample presence) so Evaluate()
                // recompiles and the pre-pass rejects it -- this is the
                // post-compilation bypass codex round-5 flagged.
                UsdAttribute sc =
                    ribbon.GetAttribute(TfToken("rigExec:sampleCount"));
                int deflt = 5;
                sc.Get(&deflt);
                sc.Set(deflt, UsdTimeCode(2.0));
                const RigExecRigPose pose =
                    eval.Evaluate(UsdTimeCode::Default());
                CHECK(!pose.valid);  // recompiled and rejected
            }
        }
    }
}

static void
TestShotAnimation(const std::string &examplesDir)
{
    UsdStageRefPtr stage = UsdStage::Open(examplesDir + "/ArmShotAnim.usda");
    CHECK(stage);
    if (!stage) return;

    const SdfPath rigPath("/Shot/HeroArm/Rig");
    const SdfPath weightPath =
        rigPath.AppendPath(SdfPath("Solvers/IKFKBlend"))
            .AppendProperty(TfToken("inputs:weight"));

    RigExecTapSet taps(stage);
    const RigExecTapId weightTap =
        taps.Add(RigExecValueAddress::Property(weightPath));
    taps.Prepare();
    const SdfPath wristPath =
        rigPath.AppendPath(SdfPath("Joints/Shoulder/Elbow/Wrist"));

    // Native USD animation value resolution parity (spec §5.6): the exec
    // computeValue of the spline-driven weight matches timed Get().
    const UsdAttribute weightAttr = stage->GetAttributeAtPath(weightPath);
    for (double t : {1001.0, 1006.5, 1012.0, 1013.0, 1024.0, 1048.0}) {
        const RigExecSnapshot snap = taps.Evaluate(UsdTimeCode(t));
        CHECK(snap.IsValid());
        float authored = 0;
        CHECK(weightAttr.Get(&authored, UsdTimeCode(t)));
        const float computed = snap.Get<float>(weightTap);
        CHECK(std::abs(computed - authored) < 1e-6);
    }

    // Frame 1001: weight 0 -> FK rest pose.
    {
        const auto wrist =
            CompiledJointFrame(stage, rigPath, wristPath, UsdTimeCode(1001));
        CHECK(Near(wrist.Origin(), GfVec3d(8, 10, 0), 1e-6));
    }

    // Frame 1024: weight 1 -> IK chases the animated HandIK goal; parity
    // against the direct math solve at the resolved time.
    {
        const UsdTimeCode t(1024);
        const auto wrist = CompiledJointFrame(stage, rigPath, wristPath, t);

        const UsdPrim root = stage->GetPrimAtPath(
            rigPath.AppendPath(SdfPath("Controls/ShoulderFK")));
        const UsdPrim handIk = stage->GetPrimAtPath(
            rigPath.AppendPath(SdfPath("Controls/HandIK")));
        const UsdPrim pole = stage->GetPrimAtPath(
            rigPath.AppendPath(SdfPath("Controls/ElbowPole")));

        RigExecTwoBoneIkParams params;
        params.upperLength = 4;
        params.lowerLength = 4;
        params.stretch = double(1.0f);
        params.softness = double(0.15f);
        params.preferredBendRadians = -0.35;
        std::array<std::array<GfVec3d, 4>, 3> rests;
        rests[0] = GetRestLandmarks(root, t);
        const GfVec3d restAim = (rests[0][1] - rests[0][0]).GetNormalized();
        rests[1] = rests[0];
        for (auto &p : rests[1]) p += restAim * params.upperLength;
        rests[2] = GetRestLandmarks(handIk, t);

        const auto expected = RigExecSolveTwoBoneIk(
            DirectControlFrame(root, t), DirectControlFrame(handIk, t),
            DirectControlFrame(pole, t), rests, params);
        CHECK(Near(wrist.Origin(), expected[2].Origin(), 1e-9));
    }
}

static void
TestGeometryMovers(const std::string &examplesDir)
{
    UsdStageRefPtr stage = UsdStage::Open(examplesDir + "/ArmRig.usda");
    CHECK(stage);
    if (!stage) return;

    // Shape-preserving enable edit (value-only per spec §4.2): disable the
    // wrist aim so joint matrices are exactly identity at rest and the
    // geometry expectations below are closed-form.
    stage->GetAttributeAtPath(
        SdfPath("/ArmAsset/Rig/Movers/Pose/WristAim.inputs:enabled"))
        .Set(false);

    RigExecRigEvaluator evaluator(stage, SdfPath("/ArmAsset/Rig"));
    evaluator.cpuParityMode = true;
    std::vector<std::string> errors;
    const bool compiled = evaluator.Compile(&errors);
    for (const std::string &e : errors) {
        std::printf("compile error: %s\n", e.c_str());
    }
    CHECK(compiled);
    if (!compiled) return;

    // Composed post-order: the parent-authored reorder makes Pose precede
    // Geometry, and descendants precede ancestors (spec §4.5 narrative).
    const auto &movers = evaluator.GetMoverOrder();
    CHECK(movers.size() >= 9);
    auto indexOf = [&](const char *name) -> int {
        for (size_t i = 0; i < movers.size(); ++i) {
            if (movers[i].moverPath.GetNameToken() == name) {
                return static_cast<int>(i);
            }
        }
        return -1;
    };
    const int clampIdx = indexOf("ClampIKFKWeight");
    const int aimIdx = indexOf("WristAim");
    const int blendIdx = indexOf("BicepFlex");
    const int wristMatrixIdx = indexOf("WristMatrix");
    const int shoulderMatrixIdx = indexOf("ShoulderMatrix");
    const int volumeIdx = indexOf("VolumeCorrect");
    CHECK(clampIdx >= 0 && aimIdx > clampIdx);        // child before parent
    CHECK(aimIdx < blendIdx);                          // Pose before Geometry
    CHECK(blendIdx < wristMatrixIdx);                  // deepest child first
    CHECK(wristMatrixIdx < shoulderMatrixIdx);
    CHECK(shoulderMatrixIdx < volumeIdx);

    const RigExecRigPose pose = evaluator.Evaluate(UsdTimeCode::Default());
    CHECK(pose.valid);

    // NOTHING IS AUTHORED, anywhere (spec §7.2 revised).
    //
    // This used to assert the opposite of the last two checks: that a
    // private derived stage existed and carried a __RigExecGenerated scope,
    // because compilation authored lowered property applications there. The
    // engine has no compiler now -- solver->joint bindings, frame revisions,
    // point chains and the ribbon's driver points all reach exec as value
    // overrides or through the in-memory mover graph -- so there is nothing
    // to author and no stage to derive. The evaluation stage IS the source
    // stage, and the generated scope must not exist on it.
    CHECK(!stage->GetPrimAtPath(
        SdfPath("/ArmAsset/Rig/__RigExecGenerated")));
    CHECK(stage->GetSessionLayer() &&
          stage->GetSessionLayer()->GetNumSubLayerPaths() == 0);
    CHECK(evaluator.GetEvaluationStage() == stage);
    CHECK(!evaluator.GetEvaluationStage()->GetPrimAtPath(
        SdfPath("/ArmAsset/Rig/__RigExecGenerated")));

    // At rest, matrix movers are identity; the blend shape at channel
    // weight 0.35 moves masked points:
    //   p1' = p1 + 0.35 * (target1 - base1) * mask1, mask = [0,1,1,0].
    const SdfPath bodyPoints("/ArmAsset/Geom/ArmBody.points");
    const auto it = pose.movedProperties.find(bodyPoints);
    CHECK(it != pose.movedProperties.end());
    if (it != pose.movedProperties.end()) {
        const auto points = it->second.Get<VtVec3fArray>();
        CHECK(points.size() == 4);
        if (points.size() == 4) {
            CHECK(Near(GfVec3d(points[0]), GfVec3d(0, 9.5, 0), 1e-6));
            CHECK(Near(GfVec3d(points[1]),
                       GfVec3d(8, 9.5 + 0.35 * 0.25, 0.35 * -0.15), 1e-5));
            CHECK(Near(GfVec3d(points[2]),
                       GfVec3d(8, 10.5 + 0.35 * 0.25, 0.35 * 0.15), 1e-5));
            CHECK(Near(GfVec3d(points[3]), GfVec3d(0, 10.5, 0), 1e-6));
        }
    }

    // Mover-graph parity: the compiled graph runs alongside the generated-prim
    // chains and must never disagree with them (see
    // docs/mover-graph-cutover.md). The summary line is asserted present, not
    // just the absence of a mismatch: a parity pass that silently checked
    // nothing would otherwise read exactly like one that passed.
    //
    // ArmBody's chain is the deepest in the examples --
    // volumeCorrect -> curve -> 3x matrix -> blend -- and every one of those
    // ops now has its provider values, so it must AGREE, not defer.
    // "0 chain(s) agreed" fails explicitly: a parity pass that silently
    // checked nothing is indistinguishable from one that passed.
    {
        if (pose.moverGraphParityMismatches != 0 ||
            pose.moverGraphParityAgreements == 0) {
            for (const std::string &d : pose.diagnostics) {
                if (d.find("mover graph parity") != std::string::npos) {
                    std::printf("    %s\n", d.c_str());
                }
            }
        }
        CHECK(pose.moverGraphParityMismatches == 0);
        CHECK(pose.moverGraphParityAgreements > 0);
    }

    // Derived chains: normals unit-length and extent bounds final points.
    const auto normalsIt = pose.movedProperties.find(
        SdfPath("/ArmAsset/Geom/ArmBody.normals"));
    CHECK(normalsIt != pose.movedProperties.end());
    if (normalsIt != pose.movedProperties.end()) {
        const auto normals = normalsIt->second.Get<VtVec3fArray>();
        CHECK(normals.size() == 4);
        for (const GfVec3f &n : normals) {
            CHECK(std::abs(n.GetLength() - 1.0f) < 1e-4f);
        }
    }
    const auto extentIt = pose.movedProperties.find(
        SdfPath("/ArmAsset/Geom/ArmBody.extent"));
    CHECK(extentIt != pose.movedProperties.end());

    // Scalar-reference parity (spec §7.4): the generated OpenExec
    // application chain and the CPU reference kernels agree exactly.
    const auto cpuIt = pose.movedPropertiesCpu.find(bodyPoints);
    CHECK(cpuIt != pose.movedPropertiesCpu.end());
    if (it != pose.movedProperties.end() &&
        cpuIt != pose.movedPropertiesCpu.end()) {
        const auto execPts = it->second.Get<VtVec3fArray>();
        const auto cpuPts = cpuIt->second.Get<VtVec3fArray>();
        CHECK(execPts.size() == cpuPts.size());
        for (size_t i = 0; i < execPts.size() && i < cpuPts.size(); ++i) {
            CHECK(Near(GfVec3d(execPts[i]), GfVec3d(cpuPts[i]), 1e-6));
        }
    }

    // Structural-edit epoch rebuild (spec §4.2): retargeting a mover's
    // moves relationship changes the binding-epoch digest, and the next
    // Evaluate recompiles.
    const size_t epochBefore = evaluator.GetBindingEpochDigest();
    stage->GetPrimAtPath(
             SdfPath("/ArmAsset/Rig/Movers/Geometry/VolumeCorrect/"
                     "RibbonWrap/ShoulderMatrix/ElbowMatrix/WristMatrix"))
        .GetRelationship(TfToken("rigExec:transform"))
        .SetTargets({SdfPath("/ArmAsset/Rig/Joints/Shoulder/Elbow")});
    const RigExecRigPose pose2 = evaluator.Evaluate(UsdTimeCode::Default());
    CHECK(pose2.valid);
    CHECK(evaluator.GetBindingEpochDigest() != epochBefore);
    bool sawRebuild = false;
    for (const std::string &d : pose2.diagnostics) {
        if (d.find("epoch rebuilt") != std::string::npos) {
            sawRebuild = true;
        }
    }
    CHECK(sawRebuild);

    // Ribbon guides follow the driver curve's rotation-minimizing frame
    // sample origins (spec 7.5): parity against the reference sampler on
    // the authored driver control points.
    const auto guidesIt = pose.movedProperties.find(
        SdfPath("/ArmAsset/Geom/RibbonGuides.points"));
    CHECK(guidesIt != pose.movedProperties.end());
    if (guidesIt != pose.movedProperties.end()) {
        const auto guides = guidesIt->second.Get<VtVec3fArray>();
        CHECK(guides.size() == 5);
        VtVec3fArray driverCvs;
        stage->GetAttributeAtPath(
                 SdfPath("/ArmAsset/Geom/RibbonDriver.points"))
            .Get(&driverCvs);
        const auto samples = RigExecSampleCurveRMF(
            std::vector<GfVec3f>(driverCvs.begin(), driverCvs.end()), 5);
        CHECK(samples.GetSize() == 5);
        if (guides.size() == 5 && samples.GetSize() == 5) {
            for (int i = 0; i < 5; ++i) {
                CHECK(Near(GfVec3d(guides[i]),
                           GfVec3d(samples.positions[i]), 1e-4));
            }
        }
    }
}

// Regression for spec §7.3: blend-shape deltas derive against the BASE
// points even when a preceding mover already moved the destination. The
// mover hierarchy nests the matrix mover under the blend mover, so the
// matrix mover's revision precedes the blend application.
static void
TestBlendDeltasUseBase()
{
    UsdStageRefPtr stage = UsdStage::CreateInMemory();
    stage->DefinePrim(SdfPath("/Asset"), TfToken("Xform"));

    // Geometry: a four-point mesh and a target shape offset by +Z.
    UsdPrim mesh =
        stage->DefinePrim(SdfPath("/Asset/Geom/M"), TfToken("Mesh"));
    const VtVec3fArray base = {
        GfVec3f(0, 0, 0), GfVec3f(1, 0, 0), GfVec3f(1, 1, 0),
        GfVec3f(0, 1, 0)};
    mesh.CreateAttribute(TfToken("points"),
                         SdfValueTypeNames->Point3fArray).Set(base);
    mesh.CreateAttribute(TfToken("faceVertexCounts"),
                         SdfValueTypeNames->IntArray).Set(VtIntArray{4});
    mesh.CreateAttribute(TfToken("faceVertexIndices"),
                         SdfValueTypeNames->IntArray)
        .Set(VtIntArray{0, 1, 2, 3});
    UsdPrim shape =
        stage->DefinePrim(SdfPath("/Asset/Targets/T"), TfToken("Points"));
    VtVec3fArray shapePoints = base;
    for (GfVec3f &p : shapePoints) {
        p += GfVec3f(0, 0, 1);
    }
    shape.CreateAttribute(TfToken("points"),
                          SdfValueTypeNames->Point3fArray).Set(shapePoints);

    // Rig: one joint posed as a pure +2Y translation.
    stage->DefinePrim(SdfPath("/Asset/Rig"), TfToken("RigExecRig"));
    UsdPrim joint = stage->DefinePrim(SdfPath("/Asset/Rig/Joints/J"),
                                      TfToken("RigExecJoint"));
    // Ir contract: rest defaults to identity; an authored non-identity
    // posed:space poses the joint directly.
    GfMatrix4d posedSpace(1.0);
    posedSpace.SetTranslate(GfVec3d(0, 2, 0));
    joint.CreateAttribute(TfToken("posed:space"),
                          SdfValueTypeNames->Matrix4d)
        .Set(posedSpace);
    // No rig-level output manifest: defining the RigExecJoint under the rig
    // is what publishes it (implicit discovery, replaces rigExec:jointOutputs).

    // Weights: full-coverage dense fields for both movers.
    auto makeWeight = [&](const char *name) {
        UsdPrim w = stage->DefinePrim(
            SdfPath(std::string("/Asset/Rig/Weights/") + name),
            TfToken("RigExecStaticWeight"));
        w.CreateRelationship(TfToken("rigExec:weightTarget"))
            .SetTargets({SdfPath("/Asset/Geom/M.points")});
        w.CreateAttribute(TfToken("rigExec:representation"),
                          SdfValueTypeNames->Token).Set(TfToken("dense"));
        w.CreateAttribute(TfToken("rigExec:values"),
                          SdfValueTypeNames->FloatArray)
            .Set(VtFloatArray{1, 1, 1, 1});
        w.CreateAttribute(TfToken("rigExec:defaultWeight"),
                          SdfValueTypeNames->Float).Set(0.0f);
        return w;
    };
    UsdPrim moveWeight = makeWeight("All");
    UsdPrim maskWeight = makeWeight("Mask");

    // In-between shape: half activation with a +0.8Z delta.
    UsdPrim halfShape =
        stage->DefinePrim(SdfPath("/Asset/Targets/H"), TfToken("Points"));
    VtVec3fArray halfPoints = base;
    for (GfVec3f &p : halfPoints) {
        p += GfVec3f(0, 0, 0.8f);
    }
    halfShape.CreateAttribute(TfToken("points"),
                              SdfValueTypeNames->Point3fArray)
        .Set(halfPoints);

    // Blend channel with an in-between at 0.5 and the full sample at 1,
    // channel weight 0.75: piecewise-linear between the brackets gives
    // delta = 0.8 + (1.0 - 0.8) * 0.5 = 0.9 (spec §7.3).
    UsdPrim input = stage->DefinePrim(SdfPath("/Asset/Rig/BlendInputs/B"),
                                      TfToken("RigExecBlendInput"));
    input.CreateAttribute(TfToken("inputs:weight"),
                          SdfValueTypeNames->Float).Set(0.75f);
    UsdPrim sample = stage->DefinePrim(
        SdfPath("/Asset/Rig/BlendInputs/B/Full"),
        TfToken("RigExecBlendSample"));
    sample.CreateAttribute(TfToken("rigExec:activation"),
                           SdfValueTypeNames->Float).Set(1.0f);
    sample.CreateRelationship(TfToken("rigExec:targetPoints"))
        .SetTargets({SdfPath("/Asset/Targets/T.points")});
    UsdPrim halfSample = stage->DefinePrim(
        SdfPath("/Asset/Rig/BlendInputs/B/Half"),
        TfToken("RigExecBlendSample"));
    halfSample.CreateAttribute(TfToken("rigExec:activation"),
                               SdfValueTypeNames->Float).Set(0.5f);
    halfSample.CreateRelationship(TfToken("rigExec:targetPoints"))
        .SetTargets({SdfPath("/Asset/Targets/H.points")});
    input.CreateRelationship(TfToken("rigExec:samples"))
        .SetTargets({sample.GetPath(), halfSample.GetPath()});

    // Movers: matrix mover nested under the blend mover, so it applies
    // first in post-order.
    UsdPrim blend = stage->DefinePrim(SdfPath("/Asset/Rig/Movers/Blend"),
                                      TfToken("RigExecBlendShapeMover"));
    blend.ApplyAPI(TfToken("RigExecMoverAPI"));
    blend.CreateRelationship(TfToken("rigExec:moves"))
        .SetTargets({SdfPath("/Asset/Geom/M.points")});
    blend.CreateRelationship(TfToken("rigExec:blendInputs"))
        .SetTargets({input.GetPath()});
    blend.CreateRelationship(TfToken("rigExec:weightObject"))
        .SetTargets({maskWeight.GetPath()});

    UsdPrim mover = stage->DefinePrim(
        SdfPath("/Asset/Rig/Movers/Blend/JM"), TfToken("RigExecMatrixMover"));
    mover.ApplyAPI(TfToken("RigExecMoverAPI"));
    mover.CreateRelationship(TfToken("rigExec:moves"))
        .SetTargets({SdfPath("/Asset/Geom/M.points")});
    mover.CreateRelationship(TfToken("rigExec:transform"))
        .SetTargets({joint.GetPath()});
    mover.CreateRelationship(TfToken("rigExec:weightObject"))
        .SetTargets({moveWeight.GetPath()});

    RigExecRigEvaluator evaluator(stage, SdfPath("/Asset/Rig"));
    std::vector<std::string> errors;
    const bool compiled = evaluator.Compile(&errors);
    for (const std::string &e : errors) {
        std::printf("compile error: %s\n", e.c_str());
    }
    CHECK(compiled);
    if (!compiled) return;

    const RigExecRigPose pose = evaluator.Evaluate(UsdTimeCode::Default());
    CHECK(pose.valid);
    const auto it =
        pose.movedProperties.find(SdfPath("/Asset/Geom/M.points"));
    CHECK(it != pose.movedProperties.end());
    if (it == pose.movedProperties.end()) return;
    const auto points = it->second.Get<VtVec3fArray>();
    CHECK(points.size() == 4);
    for (size_t i = 0; i < points.size(); ++i) {
        // matrix move (+2Y) then in-between blend delta vs BASE (+0.9Z).
        const GfVec3d expected =
            GfVec3d(base[i]) + GfVec3d(0, 2, 0.9);
        CHECK(Near(GfVec3d(points[i]), expected, 1e-5));
    }
}

// Negative compile: a weight object whose target does not canonicalize to
// the consuming matrix mover's points target fails compilation
// (spec §4.2, §7.4).
static void
TestWeightTargetMismatchFailsCompile(const std::string &examplesDir)
{
    UsdStageRefPtr stage = UsdStage::Open(examplesDir + "/ArmRig.usda");
    CHECK(stage);
    if (!stage) return;
    stage->GetPrimAtPath(SdfPath("/ArmAsset/Rig/Weights/ShoulderPoints"))
        .GetRelationship(TfToken("rigExec:weightTarget"))
        .SetTargets({SdfPath("/ArmAsset/Geom/RibbonGuides.points")});

    RigExecRigEvaluator evaluator(stage, SdfPath("/ArmAsset/Rig"));
    std::vector<std::string> errors;
    CHECK(!evaluator.Compile(&errors));
    CHECK(!errors.empty());
}

// Authored movers may not write derived properties (spec §7.6 revised),
// and mover-level parameter packets forbid multi-target fan-out for
// smooth/volume/lattice movers.
static void
TestDerivedTargetAndFanoutRejected(const std::string &examplesDir)
{
    UsdStageRefPtr stage = UsdStage::Open(examplesDir + "/ArmRig.usda");
    CHECK(stage);
    if (!stage) return;
    UsdPrim mover = stage->DefinePrim(
        SdfPath("/ArmAsset/Rig/Movers/Geometry/BadSmooth"),
        TfToken("RigExecSmoothMover"));
    CHECK(mover);
    mover.CreateRelationship(TfToken("rigExec:moves"))
        .SetTargets({SdfPath("/ArmAsset/Geom/ArmBody.normals")});
    {
        RigExecRigEvaluator evaluator(stage, SdfPath("/ArmAsset/Rig"));
        std::vector<std::string> errors;
        CHECK(!evaluator.Compile(&errors));
        CHECK(!errors.empty());
    }
    mover.GetRelationship(TfToken("rigExec:moves"))
        .SetTargets({SdfPath("/ArmAsset/Geom/ArmBody.points"),
                     SdfPath("/ArmAsset/Geom/RibbonGuides.points")});
    {
        RigExecRigEvaluator evaluator(stage, SdfPath("/ArmAsset/Rig"));
        std::vector<std::string> errors;
        CHECK(!evaluator.Compile(&errors));
        CHECK(!errors.empty());
    }
    // A custom point3f[] "points" attribute on a non-PointBased prim is
    // not a legal target either: the owner must be a stock PointBased.
    UsdPrim junk = stage->DefinePrim(
        SdfPath("/ArmAsset/Geom/Junk"), TfToken("Xform"));
    junk.CreateAttribute(
            TfToken("points"), SdfValueTypeNames->Point3fArray)
        .Set(VtVec3fArray{GfVec3f(0)});
    mover.GetRelationship(TfToken("rigExec:moves"))
        .SetTargets({SdfPath("/ArmAsset/Geom/Junk.points")});
    {
        RigExecRigEvaluator evaluator(stage, SdfPath("/ArmAsset/Rig"));
        std::vector<std::string> errors;
        CHECK(!evaluator.Compile(&errors));
        CHECK(!errors.empty());
    }
}

// Authored normals on a moved non-mesh PointBased target fail compile
// (vertex-normal recomputation is mesh-only) instead of going silently
// stale under the automatic-maintenance contract.
static void
TestNonMeshAuthoredNormalsRejected(const std::string &examplesDir)
{
    UsdStageRefPtr stage = UsdStage::Open(examplesDir + "/ArmRig.usda");
    CHECK(stage);
    if (!stage) return;
    // RibbonGuides (BasisCurves) is moved by GuideFromRibbon; author
    // normals on it.
    UsdPrim guides =
        stage->GetPrimAtPath(SdfPath("/ArmAsset/Geom/RibbonGuides"));
    CHECK(guides);
    guides.CreateAttribute(
              TfToken("normals"), SdfValueTypeNames->Normal3fArray)
        .Set(VtVec3fArray(5, GfVec3f(0, 0, 1)));
    RigExecRigEvaluator evaluator(stage, SdfPath("/ArmAsset/Rig"));
    std::vector<std::string> errors;
    CHECK(!evaluator.Compile(&errors));
    CHECK(!errors.empty());
}

// The rig declares no membership lists: a RigExecJoint under the rig is what
// publishes it. Guards the discovery rule that replaced rigExec:jointOutputs.
static void
TestImplicitJointDiscovery(const std::string &examplesDir)
{
    // A rig with no joint prim fails to compile rather than compiling an
    // empty output set (which would publish nothing and look like success).
    {
        UsdStageRefPtr stage = UsdStage::CreateInMemory();
        stage->DefinePrim(SdfPath("/Asset/Rig"), TfToken("RigExecRig"));
        RigExecRigEvaluator evaluator(stage, SdfPath("/Asset/Rig"));
        std::vector<std::string> errors;
        CHECK(!evaluator.Compile(&errors));
        CHECK(!errors.empty());
    }

    // Defining the joint is sufficient -- no manifest relationship anywhere.
    {
        UsdStageRefPtr stage = UsdStage::CreateInMemory();
        stage->DefinePrim(SdfPath("/Asset/Rig"), TfToken("RigExecRig"));
        stage->DefinePrim(SdfPath("/Asset/Rig/Joints/J"),
                          TfToken("RigExecJoint"));
        RigExecRigEvaluator evaluator(stage, SdfPath("/Asset/Rig"));
        std::vector<std::string> errors;
        CHECK(evaluator.Compile(&errors));
    }

    // Every example publishes exactly the joints defined under its rig, and
    // adding one changes the binding epoch the way editing the old manifest
    // relationship did.
    {
        UsdStageRefPtr stage =
            UsdStage::Open(examplesDir + "/01_FkChainTail.usda");
        CHECK(stage);
        if (!stage) return;
        RigExecRigEvaluator evaluator(stage, SdfPath("/TailAsset/Rig"));
        std::vector<std::string> errors;
        CHECK(evaluator.Compile(&errors));
        const size_t before = evaluator.GetBindingEpochDigest();

        stage->DefinePrim(SdfPath("/TailAsset/Rig/Joints/Extra"),
                          TfToken("RigExecJoint"));
        CHECK(evaluator.Compile(&errors));
        CHECK(evaluator.GetBindingEpochDigest() != before);
    }
}

// The compiled mover graph reproduces the generated-prim chains exactly, on
// rigs whose every revision the evaluator supplies provider values for:
// 01_FkChainTail is a pure matrix chain (4 skinning movers), 04_BlendShapeFace
// a pure blend chain. Both must AGREE, not defer -- "0 chain(s) agreed" fails,
// because a parity pass that silently checked nothing is indistinguishable
// from one that passed (see docs/mover-graph-cutover.md).
static void
TestMoverGraphParity(const std::string &examplesDir)
{
    // EVERY example, not a sample: the ops differ per rig (lattice, surface,
    // ribbon, aim), and a chain whose provider values are wrong publishes
    // undeformed geometry rather than failing. The rig prim is discovered by
    // type so adding an example cannot silently skip it.
    struct Case { const char *file; double time; bool useTime; };
    const Case cases[] = {
        {"/01_FkChainTail.usda", 0, false},
        {"/02_TwoBoneIkLeg.usda", 0, false},
        {"/03_IkFkBlendClamp.usda", 0, false},
        {"/04_BlendShapeFace.usda", 0, false},
        {"/05_TwistRibbonSpine.usda", 0, false},
        {"/06_LatticeBulge.usda", 0, false},
        {"/07_SurfaceDrape.usda", 0, false},
        {"/08_AimEyes.usda", 0, false},
        {"/ArmRig.usda", 0, false},
        {"/09_PropertyMathMovers.usda", 0, false},
        // At an ANIMATED time, not just the rest pose. Default-time parity is
        // structurally blind to two whole classes of bug: operands that are
        // equal at rest (the lattice rest cage vs the live cage) and static
        // reads that ignore the evaluation time. 06's cage bulges at 1024.
        {"/06_LatticeBulge.usda", 1024, true},
        {"/ArmShotAnim.usda", 1010, true},
        // 05 at an animated time is the solver-reads-solver-posed-joint case:
        // SpineFK poses Root/Chest, SpineTwist reads them as start/end and
        // poses TwistMid. At rest the override equals the fallback so nothing
        // can diverge; at 1024 ChestCtl is rotated and it can.
        {"/05_TwistRibbonSpine.usda", 1024, true},
    };

    for (const Case &c : cases) {
        UsdStageRefPtr stage = UsdStage::Open(examplesDir + c.file);
        CHECK(stage);
        if (!stage) {
            continue;
        }
        SdfPath rigPath;
        for (const UsdPrim &p : stage->Traverse()) {
            if (p.GetTypeName() == TfToken("RigExecRig")) {
                rigPath = p.GetPath();
                break;
            }
        }
        CHECK(!rigPath.IsEmpty());
        if (rigPath.IsEmpty()) {
            continue;
        }
        RigExecRigEvaluator evaluator(stage, rigPath);
        std::vector<std::string> errors;
        if (!evaluator.Compile(&errors)) {
            std::printf("  %s: compile failed\n", c.file);
            for (const std::string &e : errors) {
                std::printf("    %s\n", e.c_str());
            }
        }
        CHECK(errors.empty());
        const RigExecRigPose pose = evaluator.Evaluate(
            c.useTime ? UsdTimeCode(c.time) : UsdTimeCode::Default());
        CHECK(pose.valid);

        // Counts, not diagnostic text: zero mismatches AND non-zero
        // agreements. Asserting only "no mismatch" would pass a rig whose
        // chains were never compared at all -- which is how the
        // SurfaceProject regression reached usdview.
        if (pose.moverGraphParityMismatches != 0 ||
            pose.moverGraphParityAgreements == 0) {
            std::printf("  (%s @%s parity diagnostics:)\n", c.file,
                        c.useTime ? std::to_string(c.time).c_str()
                                  : "default");
            for (const std::string &d : pose.diagnostics) {
                if (d.find("mover graph parity") != std::string::npos) {
                    std::printf("    %s\n", d.c_str());
                }
            }
        }
        CHECK(pose.moverGraphParityMismatches == 0);
        CHECK(pose.moverGraphParityAgreements > 0);

        // Solver->joint overrides must reach a fixed point.
        CHECK(pose.solverOverridesConverged);

        // 05 at an animated time is the one case in the examples where a
        // solver consumes a joint that another solver poses: SpineFK poses
        // Root/Chest, SpineTwist reads them and poses TwistMid. It therefore
        // MUST need more than one round -- if it ever needs only one, the
        // refinement was dropped and TwistMid is being posed from unposed
        // endpoints again. Parity cannot catch that: the graph and the
        // lowered path consume the same override and agree on the same wrong
        // answer.
        if (std::string(c.file) == "/05_TwistRibbonSpine.usda" && c.useTime) {
            if (pose.solverOverrideRounds < 2) {
                std::printf("  05 @%f: overrides settled in %zu round(s); "
                            "the solver-reads-solver-posed-joint refinement "
                            "is not running\n",
                            c.time, pose.solverOverrideRounds);
            }
            CHECK(pose.solverOverrideRounds >= 2);
        }

        // Parity alone cannot say the movers DID anything -- two identical
        // pass-throughs agree perfectly. For the rigs that visibly deform at
        // the evaluated time, assert the published points actually left the
        // authored base. 07's stickers project onto the ground plane at rest;
        // 06's cage bulges at 1024.
        //
        // This catches a total pass-through (a packet that fails validation
        // and silently preserves the previous revision). It would NOT have
        // caught the SurfaceProject strength bug, which produced a half
        // projection -- wrong, but still moved. The two checks are
        // complementary: parity catches wrong-but-nonzero, this catches
        // nothing-happened.
        const bool mustDeform =
            std::string(c.file) == "/07_SurfaceDrape.usda" ||
            (std::string(c.file) == "/06_LatticeBulge.usda" && c.useTime);
        if (mustDeform) {
            bool anyMoved = false;
            for (const auto &[path, value] : pose.movedProperties) {
                if (path.GetNameToken() != "points" ||
                    !value.IsHolding<VtVec3fArray>()) {
                    continue;
                }
                VtVec3fArray base;
                const UsdAttribute a = stage->GetAttributeAtPath(path);
                if (!a || !a.Get(&base, c.useTime ? UsdTimeCode(c.time)
                                                  : UsdTimeCode::Default())) {
                    continue;
                }
                const VtVec3fArray &out = value.UncheckedGet<VtVec3fArray>();
                for (size_t i = 0; i < out.size() && i < base.size(); ++i) {
                    if ((GfVec3d(out[i]) - GfVec3d(base[i])).GetLength() >
                        1e-4) {
                        anyMoved = true;
                        break;
                    }
                }
                if (anyMoved) {
                    break;
                }
            }
            if (!anyMoved) {
                std::printf("  %s: published points equal the authored base "
                            "-- movers did nothing\n", c.file);
            }
            CHECK(anyMoved);
        }
    }
}

// A solver cycle is rejected at compile with the offending path.
//
// Unique joint ownership does NOT make the solver graph acyclic: two solvers
// can each uniquely pose their own joints while reading each other's. 05 is
// already SpineFK -> SpineTwist (SpineTwist reads Root/Chest, which SpineFK
// poses); pointing SpineFK's controls at TwistMid, which SpineTwist poses,
// closes the loop. Evaluate resolves overrides by iterating to a fixed point
// and a cycle has none, so this must fail in Compile rather than surface as a
// non-converging generation.
static void
TestSolverCycleRejected(const std::string &examplesDir)
{
    UsdStageRefPtr stage =
        UsdStage::Open(examplesDir + "/05_TwistRibbonSpine.usda");
    CHECK(stage);
    if (!stage) {
        return;
    }
    const SdfPath rigPath("/SpineAsset/Rig");

    // Sanity: unmodified, it compiles.
    {
        RigExecRigEvaluator ok(stage, rigPath);
        std::vector<std::string> errors;
        CHECK(ok.Compile(&errors));
    }

    UsdPrim fk = stage->GetPrimAtPath(
        SdfPath("/SpineAsset/Rig/Solvers/SpineFK"));
    CHECK(fk);
    if (!fk) {
        return;
    }
    fk.GetRelationship(TfToken("rigExec:controls"))
        .AddTarget(SdfPath("/SpineAsset/Rig/Joints/TwistMid"));

    RigExecRigEvaluator evaluator(stage, rigPath);
    std::vector<std::string> errors;
    CHECK(!evaluator.Compile(&errors));
    bool namedCycle = false;
    for (const std::string &e : errors) {
        if (e.find("solver dependency cycle") != std::string::npos) {
            namedCycle = true;
        }
    }
    if (!namedCycle) {
        std::printf("  cycle test: compile errors were:\n");
        for (const std::string &e : errors) {
            std::printf("    %s\n", e.c_str());
        }
    }
    CHECK(namedCycle);
}

// The same rejection for a cycle that never touches a joint.
//
// TestSolverCycleRejected above closes the loop through a joint (SpineFK poses
// what SpineTwist reads). RigExecBlendPointFrames takes rigExec:inputA/inputB
// as SOLVER paths, so a cycle can also run purely solver -> solver: 03 already
// has IKFKBlend.inputA = ArmFK, so pointing ArmFK's controls back at IKFKBlend
// closes it with no joint in the path. Deriving only the joint-mediated edges
// would miss this entirely, which is why the direct edge rule exists -- and
// why it needs its own regression (codex round-4).
static void
TestPureSolverToSolverCycleRejected(const std::string &examplesDir)
{
    UsdStageRefPtr stage =
        UsdStage::Open(examplesDir + "/03_IkFkBlendClamp.usda");
    CHECK(stage);
    if (!stage) {
        return;
    }
    const SdfPath rigPath("/BlendArmAsset/Rig");

    {
        RigExecRigEvaluator ok(stage, rigPath);
        std::vector<std::string> errors;
        CHECK(ok.Compile(&errors));
    }

    UsdPrim fk = stage->GetPrimAtPath(
        SdfPath("/BlendArmAsset/Rig/Solvers/ArmFK"));
    const UsdPrim blend = stage->GetPrimAtPath(
        SdfPath("/BlendArmAsset/Rig/Solvers/IKFKBlend"));
    CHECK(fk);
    CHECK(blend);
    if (!fk || !blend) {
        return;
    }

    // Precondition, asserted rather than assumed: neither edge of the loop can
    // come from the indirect joint-mediated rule, so a rejection can ONLY come
    // from the direct solver->solver rule. The cycle error itself is generic
    // and cannot distinguish the two, which is what makes this check the thing
    // that gives the test its meaning (codex round-5).
    //
    // ArmFK posing no joints means nothing reads a joint of ArmFK's, and
    // IKFKBlend not being a joint means ArmFK's new controls target cannot be
    // a joint-mediated edge either.
    {
        SdfPathVector fkJoints;
        if (const UsdRelationship r =
                fk.GetRelationship(TfToken("rigExec:joints"))) {
            r.GetTargets(&fkJoints);
        }
        CHECK(fkJoints.empty());
        CHECK(blend.GetTypeName() != TfToken("RigExecJoint"));
    }

    fk.GetRelationship(TfToken("rigExec:controls"))
        .AddTarget(SdfPath("/BlendArmAsset/Rig/Solvers/IKFKBlend"));

    RigExecRigEvaluator evaluator(stage, rigPath);
    std::vector<std::string> errors;
    CHECK(!evaluator.Compile(&errors));
    bool namedCycle = false;
    for (const std::string &e : errors) {
        if (e.find("solver dependency cycle") != std::string::npos) {
            namedCycle = true;
        }
    }
    if (!namedCycle) {
        std::printf("  pure solver cycle test: compile errors were:\n");
        for (const std::string &e : errors) {
            std::printf("    %s\n", e.c_str());
        }
    }
    CHECK(namedCycle);
}

// Aim constraints revise a joint's final frame, and the revision points the
// authored aim axis at the target.
//
// Nothing asserted this before: the mover-graph parity suite only covers point
// chains, and an aim moves a transform provider's FRAME. So the whole
// pose-domain revision path could have been a no-op and every test would still
// have passed -- which matters now that it is applied in memory rather than
// through a generated RigExecPointFrameMoverApplication.
//
// 08_AimEyes: EyeL/EyeR sit at (-1,10,0)/(1,10,0), LookAt at (0,10,6), and
// both aims declare rigExec:aimAxis = "z" at weight 1. So each eye's final
// frame must (a) differ from its base and (b) have its Z axis pointing at the
// LookAt origin.
static void
TestAimConstraintRevisesJointFrame(const std::string &examplesDir)
{
    UsdStageRefPtr stage =
        UsdStage::Open(examplesDir + "/08_AimEyes.usda");
    CHECK(stage);
    if (!stage) {
        return;
    }
    RigExecRigEvaluator evaluator(stage, SdfPath("/EyesAsset/Rig"));
    std::vector<std::string> errors;
    CHECK(evaluator.Compile(&errors));
    const RigExecRigPose pose = evaluator.Evaluate(UsdTimeCode::Default());
    CHECK(pose.valid);

    const GfVec3d lookAt(0, 10, 6);
    for (const char *name : {"EyeL", "EyeR"}) {
        const SdfPath joint(std::string("/EyesAsset/Rig/Joints/") + name);
        const auto baseIt = pose.jointFramesBase.find(joint);
        const auto finalIt = pose.jointFramesFinal.find(joint);
        CHECK(baseIt != pose.jointFramesBase.end());
        CHECK(finalIt != pose.jointFramesFinal.end());
        if (baseIt == pose.jointFramesBase.end() ||
            finalIt == pose.jointFramesFinal.end()) {
            continue;
        }
        // (a) the aim actually did something
        const bool revised =
            (finalIt->second.Z() - baseIt->second.Z()).GetLength() > 1e-6;
        if (!revised) {
            std::printf("  %s: final frame equals base -- aim did nothing\n",
                        name);
        }
        CHECK(revised);

        // (b) and it aims where it was told to
        const GfVec3d origin = finalIt->second.Origin();
        const GfVec3d axis = (finalIt->second.Z() - origin).GetNormalized();
        const GfVec3d toTarget = (lookAt - origin).GetNormalized();
        const double alignment = GfDot(axis, toTarget);
        if (alignment < 0.999) {
            std::printf("  %s: aim axis . toTarget = %f (expected ~1)\n",
                        name, alignment);
        }
        CHECK(alignment > 0.999);

        // The paired matrix must be published alongside the revised frame.
        CHECK(pose.jointMatricesFinal.count(joint) == 1);
    }
}

// 09 documents three property-domain math movers that v0.1-alpha does NOT
// evaluate. Assert that, so the file's claim is enforced rather than asserted
// in prose -- and so this test flips to a failure the day someone implements
// them, which is exactly when the docstring needs rewriting.
//
// The rig must still compile, publish, and deform its witness card: an
// unevaluated mover is a documented gap, not a broken rig.
static void
TestPropertyMathMoversAreNotEvaluated(const std::string &examplesDir)
{
    UsdStageRefPtr stage =
        UsdStage::Open(examplesDir + "/09_PropertyMathMovers.usda");
    CHECK(stage);
    if (!stage) {
        return;
    }
    RigExecRigEvaluator evaluator(stage, SdfPath("/PropMathAsset/Rig"));
    std::vector<std::string> errors;
    CHECK(evaluator.Compile(&errors));
    const RigExecRigPose pose = evaluator.Evaluate(UsdTimeCode::Default());
    CHECK(pose.valid);

    // All three targets are absent from the published set.
    for (const char *prop : {"rigExec:gain", "rigExec:offset",
                             "rigExec:localOffset"}) {
        const SdfPath target(
            std::string("/PropMathAsset/Rig/Channels/Dials.") + prop);
        if (pose.movedProperties.count(target) != 0) {
            std::printf("  %s is now published -- the math movers are "
                        "implemented; update 09's docstring\n", prop);
        }
        CHECK(pose.movedProperties.count(target) == 0);
    }

    // ...while the rig itself is live: the witness card follows its joint.
    const SdfPath card("/PropMathAsset/Geom/Card.points");
    const auto it = pose.movedProperties.find(card);
    CHECK(it != pose.movedProperties.end());
    CHECK(pose.moverGraphParityMismatches == 0);
}

// A constraint driving a plain UsdGeomXform publishes a transform, and the
// geometry parented under it is left alone.
//
// This is the path that had no writer at all: RigExecSnapshotPrim::hasXform
// was plumbed to HdXformSchema and never populated, so a constraint on an
// Xform computed a correct frame that never reached Hydra. Asserting the
// published matrix here is what keeps it from silently reverting to that.
static void
TestAimConstraintDrivesXform(const std::string &examplesDir)
{
    UsdStageRefPtr stage =
        UsdStage::Open(examplesDir + "/10_AimXformTurret.usda");
    CHECK(stage);
    if (!stage) {
        return;
    }
    RigExecRigEvaluator evaluator(stage, SdfPath("/TurretAsset/Rig"));
    std::vector<std::string> errors;
    if (!evaluator.Compile(&errors)) {
        for (const std::string &e : errors) {
            std::printf("    %s\n", e.c_str());
        }
    }
    CHECK(errors.empty());
    const RigExecRigPose pose = evaluator.Evaluate(UsdTimeCode(1001));
    CHECK(pose.valid);

    const SdfPath turret("/TurretAsset/Geom/Turret");
    const auto it = pose.providerXforms.find(turret);
    CHECK(it != pose.providerXforms.end());
    if (it == pose.providerXforms.end()) {
        std::printf("  no xform published for the constraint target\n");
        return;
    }

    // At 1001 the target sits at x=-8, z=10 relative to a turret at y=2, so
    // the aimed z-axis must lean toward -x. Verifying direction, not just
    // presence: a published identity would pass a mere existence check.
    const GfMatrix4d &m = it->second;
    const GfVec3d origin = m.TransformAffine(GfVec3d(0, 0, 0));
    const GfVec3d zAxis =
        (m.TransformAffine(GfVec3d(0, 0, 1)) - origin).GetNormalized();
    if (zAxis[0] >= 0.0) {
        std::printf("  aimed z-axis = (%f, %f, %f); expected it to lean -x\n",
                    zAxis[0], zAxis[1], zAxis[2]);
    }
    CHECK(zAxis[0] < 0.0);

    // The parented geometry is carried by hierarchy: no mover names it, so it
    // must NOT appear in movedProperties.
    CHECK(pose.movedProperties.count(
              SdfPath("/TurretAsset/Geom/Turret/Barrel.points")) == 0);

    // ...and it ANIMATES. The target sweeps x=-8 -> +8 between 1001 and 1024,
    // so the published transform must differ. A constraint that resolves once
    // and then holds still would pass every check above.
    const RigExecRigPose later = evaluator.Evaluate(UsdTimeCode(1024));
    CHECK(later.valid);
    const auto laterIt = later.providerXforms.find(turret);
    CHECK(laterIt != later.providerXforms.end());
    if (laterIt != later.providerXforms.end()) {
        const GfVec3d o2 = laterIt->second.TransformAffine(GfVec3d(0, 0, 0));
        const GfVec3d z2 =
            (laterIt->second.TransformAffine(GfVec3d(0, 0, 1)) - o2)
                .GetNormalized();
        if (!(z2[0] > 0.0)) {
            std::printf("  z-axis at 1024 = (%f, %f, %f); expected +x lean\n",
                        z2[0], z2[1], z2[2]);
        }
        CHECK(z2[0] > 0.0);        // target has swung to +x
        CHECK((z2 - zAxis).GetLength() > 1e-3);  // and it actually moved
    }
}

// The codeless schema's resource directory.
//
// The GENERATED one when the build supplied it: only that copy carries the
// LibraryPath that lets Plug load the compute-extent registration on demand,
// which is what makes UsdGeomBBoxCache answer for RigExec prims. The source
// tree's copy is data-only and is the fallback for an ad hoc build.
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
        std::printf("usage: testRigExecArm <examplesDir>\n");
        return 2;
    }
    const std::string examplesDir = argv[1];

    // Register the codeless RigExec schema plugin.
    const std::string resources = _SchemaResourceDir(examplesDir);
    const auto plugins =
        PlugRegistry::GetInstance().RegisterPlugins(resources);
    if (plugins.empty()) {
        std::printf("FATAL: no schema plugin found at %s\n",
                    resources.c_str());
        return 2;
    }

    TestRestPose(examplesDir);
    TestIkAndBlend(examplesDir);
    TestShotAnimation(examplesDir);
    TestGeometryMovers(examplesDir);
    TestBlendDeltasUseBase();
    TestWeightTargetMismatchFailsCompile(examplesDir);
    TestDerivedTargetAndFanoutRejected(examplesDir);
    TestNonMeshAuthoredNormalsRejected(examplesDir);
    TestViewFreeValidation(examplesDir);
    TestImplicitJointDiscovery(examplesDir);
    TestMoverGraphParity(examplesDir);
    TestSolverCycleRejected(examplesDir);
    TestPureSolverToSolverCycleRejected(examplesDir);
    TestAimConstraintRevisesJointFrame(examplesDir);
    TestPropertyMathMoversAreNotEvaluated(examplesDir);
    TestAimConstraintDrivesXform(examplesDir);

    if (failures) {
        std::printf("%d FAILURE(S)\n", failures);
        return 1;
    }
    std::printf("testRigExecArm: all tests passed\n");
    return 0;
}
