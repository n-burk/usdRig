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
#include "pxr/usd/sdf/layer.h"
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
    // documented break so it cannot silently regress.
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
// without destroying a working epoch.
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
    // joint's frame source.
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
    // rejects the now-out-of-range binding atomically: the
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
    // even when it reaches a joint only through another aggregate solver.
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
                // recompiles and the pre-pass rejects it.
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

    // Reverse-sibling post-order: Geometry is displayed above Pose, so Pose
    // executes first; descendants still precede ancestors (spec §4.5).
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
    stage->DefinePrim(SdfPath("/Asset/Rig"), TfToken("RigExecRoot"));
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
        stage->DefinePrim(SdfPath("/Asset/Rig"), TfToken("RigExecRoot"));
        RigExecRigEvaluator evaluator(stage, SdfPath("/Asset/Rig"));
        std::vector<std::string> errors;
        CHECK(!evaluator.Compile(&errors));
        CHECK(!errors.empty());
    }

    // Defining the joint is sufficient -- no manifest relationship anywhere.
    {
        UsdStageRefPtr stage = UsdStage::CreateInMemory();
        stage->DefinePrim(SdfPath("/Asset/Rig"), TfToken("RigExecRoot"));
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
        {"/13_ReadPhases.usda", 1024, true},
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
            if (p.GetTypeName() == TfToken("RigExecRoot")) {
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
// why it needs its own regression.
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
    // that gives the test its meaning.
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

// 09 exercises the three property-domain math movers: the output domain that
// is neither a joint frame nor a points array.
//
// Each target is an ordinary authored attribute of a different type, and each
// mover revises it with the operation it authors. Asserting the VALUES, not
// just presence: a chain that published its own authored base unchanged would
// pass an existence check while doing nothing, which is precisely the state
// this file used to document.
static void
TestPropertyMathMoversAreEvaluated(const std::string &examplesDir)
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

    const std::string dials = "/PropMathAsset/Rig/Channels/Dials.";

    // clamp: authored 2.5 into [0, 1].
    {
        const auto it = pose.movedProperties.find(SdfPath(dials + "rigExec:gain"));
        CHECK(it != pose.movedProperties.end());
        if (it != pose.movedProperties.end()) {
            CHECK(it->second.IsHolding<float>());
            CHECK(std::abs(it->second.Get<float>() - 1.0f) < 1e-6f);
        }
    }
    // add: authored (0, 3, 0) plus (0, 2, 0).
    {
        const auto it =
            pose.movedProperties.find(SdfPath(dials + "rigExec:offset"));
        CHECK(it != pose.movedProperties.end());
        if (it != pose.movedProperties.end()) {
            CHECK(it->second.IsHolding<GfVec3f>());
            CHECK(Near(GfVec3d(it->second.Get<GfVec3f>()),
                       GfVec3d(0, 5, 0)));
        }
    }
    // multiply: authored identity post-multiplied by translate(0, 5, 0).
    {
        const auto it =
            pose.movedProperties.find(SdfPath(dials + "rigExec:localOffset"));
        CHECK(it != pose.movedProperties.end());
        if (it != pose.movedProperties.end()) {
            CHECK(it->second.IsHolding<GfMatrix4d>());
            CHECK(Near(it->second.Get<GfMatrix4d>().ExtractTranslation(),
                       GfVec3d(0, 5, 0)));
        }
    }

    // ...while the rig itself is live: the witness card follows its joint.
    const SdfPath card("/PropMathAsset/Geom/Card.points");
    const auto it = pose.movedProperties.find(card);
    CHECK(it != pose.movedProperties.end());
    CHECK(pose.moverGraphParityMismatches == 0);
}

// A property mover's result reaches the computation that reads the attribute.
//
// This is the half that publication alone cannot demonstrate. 03 authors an
// IK/FK blend weight overdriven to -0.25..1.3 and a ClampBlendWeight mover to
// bound it, and RigExecBlendPointFrames ALSO clamps internally -- so a clamp
// that never reached exec and one that did produce identical frames, and the
// authored mover would be decoration.
//
// So the test drives the mover somewhere the kernel's own bound cannot reach:
// overridden to `blend` toward 0, it must force pure FK at a frame whose
// clamped weight is 0.525. Different joint frames are the proof the override
// landed.
static void
TestPropertyMoverFeedsConsumingComputation(const std::string &examplesDir)
{
    const SdfPath rigPath("/BlendArmAsset/Rig");
    const SdfPath weightTarget(
        "/BlendArmAsset/Rig/Solvers/IKFKBlend.inputs:weight");
    const SdfPath wrist("/BlendArmAsset/Rig/Joints/Shoulder/Elbow/Wrist");
    const UsdTimeCode when(1024);

    // As authored: the clamp passes 0.525 through untouched.
    UsdStageRefPtr authored =
        UsdStage::Open(examplesDir + "/03_IkFkBlendClamp.usda");
    CHECK(authored);
    if (!authored) {
        return;
    }
    RigExecRigEvaluator authoredEval(authored, rigPath);
    CHECK(authoredEval.Compile(nullptr));
    const RigExecRigPose authoredPose = authoredEval.Evaluate(when);
    CHECK(authoredPose.valid);
    const auto authoredWeight = authoredPose.movedProperties.find(weightTarget);
    CHECK(authoredWeight != authoredPose.movedProperties.end());
    if (authoredWeight == authoredPose.movedProperties.end()) {
        return;
    }
    const float w = authoredWeight->second.Get<float>();
    // Strictly interior, or the test proves nothing: at 0 or 1 the blend is
    // already one of its inputs and forcing it there changes nothing.
    CHECK(w > 0.01f && w < 0.99f);

    // Session-layer edit only -- the source file is untouched.
    UsdStageRefPtr forced =
        UsdStage::Open(examplesDir + "/03_IkFkBlendClamp.usda");
    CHECK(forced);
    if (!forced) {
        return;
    }
    forced->SetEditTarget(forced->GetSessionLayer());
    const UsdPrim mover = forced->GetPrimAtPath(
        SdfPath("/BlendArmAsset/Rig/Movers/Pose/ClampBlendWeight"));
    CHECK(mover);
    if (!mover) {
        return;
    }
    mover.CreateAttribute(TfToken("rigExec:operation"),
                          SdfValueTypeNames->Token, /*custom=*/false)
        .Set(TfToken("blend"));
    mover.CreateAttribute(TfToken("inputs:value"), SdfValueTypeNames->Float,
                          /*custom=*/false)
        .Set(0.0f);

    RigExecRigEvaluator forcedEval(forced, rigPath);
    std::vector<std::string> errors;
    CHECK(forcedEval.Compile(&errors));
    const RigExecRigPose forcedPose = forcedEval.Evaluate(when);
    CHECK(forcedPose.valid);
    const auto forcedWeight = forcedPose.movedProperties.find(weightTarget);
    CHECK(forcedWeight != forcedPose.movedProperties.end());
    if (forcedWeight != forcedPose.movedProperties.end()) {
        CHECK(std::abs(forcedWeight->second.Get<float>()) < 1e-6f);
    }

    // The blend consumed it: the wrist is somewhere else.
    const auto a = authoredPose.jointFramesFinal.find(wrist);
    const auto b = forcedPose.jointFramesFinal.find(wrist);
    CHECK(a != authoredPose.jointFramesFinal.end());
    CHECK(b != forcedPose.jointFramesFinal.end());
    if (a != authoredPose.jointFramesFinal.end() &&
        b != forcedPose.jointFramesFinal.end()) {
        const double moved =
            (a->second.points[0] - b->second.points[0]).GetLength();
        if (moved <= 1e-4) {
            std::printf("  forcing the blend weight to 0 did not move the "
                        "wrist: the property mover's value is not reaching "
                        "the solver\n");
        }
        CHECK(moved > 1e-4);
    }
}

// A disabled property mover passes its revision through (spec §6.6): the
// chain still publishes, holding the value the preceding revision produced.
static void
TestDisabledPropertyMoverPassesThrough(const std::string &examplesDir)
{
    UsdStageRefPtr stage =
        UsdStage::Open(examplesDir + "/09_PropertyMathMovers.usda");
    CHECK(stage);
    if (!stage) {
        return;
    }
    stage->SetEditTarget(stage->GetSessionLayer());
    const UsdPrim mover = stage->GetPrimAtPath(
        SdfPath("/PropMathAsset/Rig/Movers/ClampGain"));
    CHECK(mover);
    if (!mover) {
        return;
    }
    mover.CreateAttribute(TfToken("inputs:enabled"), SdfValueTypeNames->Bool,
                          /*custom=*/false)
        .Set(false);

    RigExecRigEvaluator evaluator(stage, SdfPath("/PropMathAsset/Rig"));
    CHECK(evaluator.Compile(nullptr));
    const RigExecRigPose pose = evaluator.Evaluate(UsdTimeCode::Default());
    CHECK(pose.valid);
    const auto it = pose.movedProperties.find(
        SdfPath("/PropMathAsset/Rig/Channels/Dials.rigExec:gain"));
    CHECK(it != pose.movedProperties.end());
    if (it != pose.movedProperties.end()) {
        // The authored 2.5, unclamped.
        CHECK(std::abs(it->second.Get<float>() - 2.5f) < 1e-6f);
    }
}

// A property mover whose target type does not match its static type fails
// the compile rather than silently publishing nothing.
static void
TestPropertyMoverTypeMismatchRejected(const std::string &examplesDir)
{
    UsdStageRefPtr stage =
        UsdStage::Open(examplesDir + "/09_PropertyMathMovers.usda");
    CHECK(stage);
    if (!stage) {
        return;
    }
    stage->SetEditTarget(stage->GetSessionLayer());
    // Point the float mover at the float3 dial.
    const UsdPrim mover = stage->GetPrimAtPath(
        SdfPath("/PropMathAsset/Rig/Movers/ClampGain"));
    CHECK(mover);
    if (!mover) {
        return;
    }
    UsdRelationship moves = mover.CreateRelationship(TfToken("rigExec:moves"),
                                                    /*custom=*/false);
    moves.SetTargets({SdfPath(
        "/PropMathAsset/Rig/Channels/Dials.rigExec:offset")});

    RigExecRigEvaluator evaluator(stage, SdfPath("/PropMathAsset/Rig"));
    std::vector<std::string> errors;
    CHECK(!evaluator.Compile(&errors));
    CHECK(!errors.empty());
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

// The smallest rig that exists: one constraint, no joints, both ends plain
// UsdGeomXformables.
//
// Two rules used to reject this and neither was about the rig being wrong. A
// joint was required because the compile gate read "a rig publishes joints",
// when in fact a mover publishes whatever its target is; and an aim target
// was always tapped for computePointFrame, which a plain Xformable does not
// publish -- a HARD exec failure that took the whole snapshot down rather
// than leaving one value missing.
//
// rigexec_flat.usda is a flattened stage of the shape an interactive session
// produces, so it is the exact case a user hits first.
static void
TestJointFreeRigPublishesXform(const std::string &examplesDir)
{
    UsdStageRefPtr stage =
        UsdStage::Open(examplesDir + "/rigexec_flat.usda");
    CHECK(stage);
    if (!stage) {
        return;
    }
    RigExecRigEvaluator evaluator(stage, SdfPath("/World/RigRoot"));
    std::vector<std::string> errors;
    if (!evaluator.Compile(&errors)) {
        for (const std::string &e : errors) {
            std::printf("    %s\n", e.c_str());
        }
    }
    CHECK(errors.empty());

    // No joints at all, and the rig is still a valid generation.
    const RigExecRigPose pose = evaluator.Evaluate(UsdTimeCode(0));
    CHECK(pose.valid);
    CHECK(pose.jointFramesFinal.empty());

    const SdfPath sphere("/World/Geom/Sphere");
    const auto it = pose.providerXforms.find(sphere);
    CHECK(it != pose.providerXforms.end());
    if (it == pose.providerXforms.end()) {
        std::printf("  no xform published for the joint-free rig\n");
        return;
    }

    // The aim target is /World/Geom/pivot/Plane at local (0, 0, 1) under a
    // pivot that rotates 0 -> 180 about Y across the shot. At frame 0 it sits
    // at +z, at 50 at +x, at 100 at -z, and the default aim axis is x -- so
    // the published x-axis has to follow it round.
    auto aimedX = [](const GfMatrix4d &m) {
        const GfVec3d origin = m.TransformAffine(GfVec3d(0, 0, 0));
        return (m.TransformAffine(GfVec3d(1, 0, 0)) - origin).GetNormalized();
    };
    CHECK(Near(aimedX(it->second), GfVec3d(0, 0, 1), 1e-5));

    const RigExecRigPose mid = evaluator.Evaluate(UsdTimeCode(50));
    CHECK(mid.valid);
    const auto midIt = mid.providerXforms.find(sphere);
    CHECK(midIt != mid.providerXforms.end());
    if (midIt != mid.providerXforms.end()) {
        CHECK(Near(aimedX(midIt->second), GfVec3d(1, 0, 0), 1e-5));
    }

    const RigExecRigPose end = evaluator.Evaluate(UsdTimeCode(100));
    CHECK(end.valid);
    const auto endIt = end.providerXforms.find(sphere);
    CHECK(endIt != end.providerXforms.end());
    if (endIt != end.providerXforms.end()) {
        CHECK(Near(aimedX(endIt->second), GfVec3d(0, 0, -1), 1e-5));
    }
}

// A rig whose entire content is two property movers competing for one dial.
//
// Authored TimesTen-then-AddOne, but DISPLAYED AddOne above TimesTen by the
// reorder below. The stack must execute bottom-to-top, so TimesTen runs before
// AddOne. The file order and composed display order disagree on purpose:
// whichever one the engine actually walks is the one the answer reveals. (A
// rig with no joints and no geometry is legal, keeping this fixture small.)
static const char *kOrderFixture = R"USDA(#usda 1.0

def Xform "Asset"
{
    def RigExecRoot "Rig"
    {
        def Scope "Channels"
        {
            float rigExec:dial = 2
        }

        def Scope "Movers"
        {
            reorder nameChildren = ["AddOne", "TimesTen"]

            def RigExecFloatMathMover "TimesTen" (
                prepend apiSchemas = ["RigExecMoverAPI"]
            )
            {
                uniform token rigExec:operation = "multiply"
                float inputs:value = 10
                rel rigExec:moves = </Asset/Rig/Channels.rigExec:dial>
            }

            def RigExecFloatMathMover "AddOne" (
                prepend apiSchemas = ["RigExecMoverAPI"]
            )
            {
                uniform token rigExec:operation = "add"
                float inputs:value = 1
                rel rigExec:moves = </Asset/Rig/Channels.rigExec:dial>
            }
        }
    }
}
)USDA";

// Each revision reads the PRECEDING revision, and execution reverses the
// composed top-to-bottom sibling order -- it is neither authoring order nor a
// fixed order.
//
// The arithmetic is chosen so the three candidate behaviours are three
// different numbers, and no assertion can pass by accident:
//
//   21  multiply-then-add over the chain   ((2 * 10) + 1)   <- correct
//   30  add-then-multiply over the chain   ((2 + 1) * 10)   <- forward order
//    3  no chaining, last writer wins      (2 + 1)          <- no chain
//
// Then the reorder is flipped through the SESSION layer and Evaluate is
// called with no intervening Compile: the composed mover topology is part of
// the binding-epoch digest, so the evaluator has to notice and rebuild. That
// is what "the order is dynamic" means operationally -- a caller never
// recompiles by hand, and never gets a stale order.
static void
TestMoverOrderIsComposedAndDynamic()
{
    const SdfLayerRefPtr layer = SdfLayer::CreateAnonymous(".usda");
    CHECK(layer);
    if (!layer || !layer->ImportFromString(kOrderFixture)) {
        std::printf("  could not build the ordering fixture\n");
        ++failures;
        return;
    }
    UsdStageRefPtr stage = UsdStage::Open(layer);
    CHECK(stage);
    if (!stage) {
        return;
    }
    const SdfPath dial("/Asset/Rig/Channels.rigExec:dial");

    RigExecRigEvaluator evaluator(stage, SdfPath("/Asset/Rig"));
    std::vector<std::string> errors;
    if (!evaluator.Compile(&errors)) {
        for (const std::string &e : errors) {
            std::printf("    %s\n", e.c_str());
        }
    }
    CHECK(errors.empty());

    auto dialValue = [&](const RigExecRigPose &pose, float *out) {
        const auto it = pose.movedProperties.find(dial);
        if (it == pose.movedProperties.end() ||
            !it->second.IsHolding<float>()) {
            return false;
        }
        *out = it->second.Get<float>();
        return true;
    };

    const RigExecRigPose first = evaluator.Evaluate(UsdTimeCode::Default());
    CHECK(first.valid);
    float value = 0;
    CHECK(dialValue(first, &value));
    if (std::abs(value - 21.0f) > 1e-6f) {
        std::printf("  dial = %f; expected 21 ((2*10)+1). 3 means the "
                    "revisions did not chain; 30 means sibling execution "
                    "was top-to-bottom\n", double(value));
    }
    CHECK(std::abs(value - 21.0f) < 1e-6f);
    const size_t firstDigest = evaluator.GetBindingEpochDigest();

    // Flip the composed order in the session layer. Nothing else changes --
    // same movers, same operands, same targets.
    stage->SetEditTarget(stage->GetSessionLayer());
    const UsdPrim movers = stage->GetPrimAtPath(SdfPath("/Asset/Rig/Movers"));
    CHECK(movers);
    if (!movers) {
        return;
    }
    movers.SetChildrenReorder({TfToken("TimesTen"), TfToken("AddOne")});

    // No Compile() call: Evaluate must detect the structural edit itself.
    const RigExecRigPose second = evaluator.Evaluate(UsdTimeCode::Default());
    CHECK(second.valid);
    CHECK(dialValue(second, &value));
    if (std::abs(value - 30.0f) > 1e-6f) {
        std::printf("  after reordering, dial = %f; expected 30 ((2+1)*10). "
                    "21 means the compiled order went stale\n",
                    double(value));
    }
    CHECK(std::abs(value - 30.0f) < 1e-6f);

    // The epoch identity moved with it, and the rebuild was reported.
    CHECK(evaluator.GetBindingEpochDigest() != firstDigest);
    bool rebuilt = false;
    for (const std::string &d : second.diagnostics) {
        rebuilt |= d.find("structural edit: epoch rebuilt") != std::string::npos;
    }
    CHECK(rebuilt);

    // A mover ADDED mid-session joins the chain, at the position the reorder
    // gives it, still with no Compile call. Order being dynamic has to mean
    // the membership of the chain too, not only the permutation of a fixed
    // set.
    const UsdPrim added = stage->DefinePrim(
        SdfPath("/Asset/Rig/Movers/MinusFive"),
        TfToken("RigExecFloatMathMover"));
    CHECK(added);
    if (added) {
        added.CreateAttribute(TfToken("rigExec:operation"),
                              SdfValueTypeNames->Token, /*custom=*/false)
            .Set(TfToken("add"));
        added.CreateAttribute(TfToken("inputs:value"),
                              SdfValueTypeNames->Float, /*custom=*/false)
            .Set(-5.0f);
        added.CreateRelationship(TfToken("rigExec:moves"), /*custom=*/false)
            .SetTargets({dial});
        movers.SetChildrenReorder(
            {TfToken("TimesTen"), TfToken("AddOne"), TfToken("MinusFive")});

        const RigExecRigPose third = evaluator.Evaluate(UsdTimeCode::Default());
        CHECK(third.valid);
        CHECK(dialValue(third, &value));
        if (std::abs(value + 20.0f) > 1e-6f) {
            std::printf("  after adding a third mover, dial = %f; expected -20 "
                        "((2-5+1)*10)\n", double(value));
        }
        CHECK(std::abs(value + 20.0f) < 1e-6f);
    }

    // The authored dial is still 2 on the stage: 21, 30 and -20 were
    // published, never written.
    float authored = 0;
    CHECK(stage->GetAttributeAtPath(dial).Get(&authored,
                                              UsdTimeCode::Default()));
    CHECK(std::abs(authored - 2.0f) < 1e-6f);
}

// The same chaining property for a POINT chain, on a real rig.
//
// 06 nests CageDeform inside Smooth inside VolumeCorrect, so post-order makes
// the lattice first and the volume correction last. Disabling the lattice
// must change the PUBLISHED points: if each mover read the authored base
// independently and the last writer simply won, the final value would be
// VolumeCorrect(base) either way and the two runs would be identical.
static void
TestPointChainConsumesPrecedingRevision(const std::string &examplesDir)
{
    const SdfPath rigPath("/LatticeAsset/Rig");
    const SdfPath slab("/LatticeAsset/Geom/Slab.points");
    const UsdTimeCode when(1024);

    auto evaluateSlab = [&](bool latticeEnabled, VtVec3fArray *out) {
        UsdStageRefPtr stage =
            UsdStage::Open(examplesDir + "/06_LatticeBulge.usda");
        CHECK(stage);
        if (!stage) {
            return false;
        }
        if (!latticeEnabled) {
            stage->SetEditTarget(stage->GetSessionLayer());
            const UsdPrim lattice = stage->GetPrimAtPath(SdfPath(
                "/LatticeAsset/Rig/Movers/Geometry/VolumeCorrect/Smooth/"
                "CageDeform"));
            CHECK(lattice);
            if (!lattice) {
                return false;
            }
            lattice
                .CreateAttribute(TfToken("inputs:enabled"),
                                 SdfValueTypeNames->Bool, /*custom=*/false)
                .Set(false);
        }
        RigExecRigEvaluator evaluator(stage, rigPath);
        CHECK(evaluator.Compile(nullptr));
        const RigExecRigPose pose = evaluator.Evaluate(when);
        CHECK(pose.valid);
        CHECK(pose.moverGraphParityMismatches == 0);
        const auto it = pose.movedProperties.find(slab);
        CHECK(it != pose.movedProperties.end());
        if (it == pose.movedProperties.end()) {
            return false;
        }
        *out = it->second.Get<VtVec3fArray>();
        return true;
    };

    VtVec3fArray withLattice, withoutLattice;
    if (!evaluateSlab(true, &withLattice) ||
        !evaluateSlab(false, &withoutLattice)) {
        return;
    }
    CHECK(!withLattice.empty());
    CHECK(withLattice.size() == withoutLattice.size());

    double worst = 0;
    for (size_t i = 0; i < withLattice.size() &&
                       i < withoutLattice.size(); ++i) {
        worst = std::max(
            worst,
            double((withLattice[i] - withoutLattice[i]).GetLength()));
    }
    if (worst <= 1e-5) {
        std::printf("  disabling the first mover in the chain did not change "
                    "the published points: the later movers are not reading "
                    "the preceding revision\n");
    }
    CHECK(worst > 1e-5);
}

// "The next mover picks up the modified value" with NO carve-out: a property
// mover's result reaches a later mover's static parameter read too.
//
// Two delivery routes exist for one revised value, because there are two
// kinds of consumer. A COMPUTATION reads the attribute through exec and gets
// the value override (03's clamped blend weight). Packet assembly reads it
// straight off the stage and never touches exec, so it gets the same value
// through RigExecResolvedInputs instead. Both routes are filled from the one
// property-chain result, before anything reads an input.
//
// Here a float mover drives the smooth mover's inputs:strength from 0.6 to 0,
// and the smoothing has to actually stop. The CPU oracle resolves its own
// inputs independently, so parity is what proves the two routes agree rather
// than both being wrong together.
static void
TestPropertyMoverReachesStaticPacketReads(const std::string &examplesDir)
{
    const SdfPath rigPath("/LatticeAsset/Rig");
    const SdfPath slab("/LatticeAsset/Geom/Slab.points");
    const SdfPath strength(
        "/LatticeAsset/Rig/Movers/Geometry/VolumeCorrect/Smooth"
        ".inputs:strength");
    const UsdTimeCode when(1024);

    UsdStageRefPtr stage =
        UsdStage::Open(examplesDir + "/06_LatticeBulge.usda");
    CHECK(stage);
    if (!stage) {
        return;
    }
    RigExecRigEvaluator baseline(stage, rigPath);
    CHECK(baseline.Compile(nullptr));
    const RigExecRigPose basePose = baseline.Evaluate(when);
    CHECK(basePose.valid);
    const auto baseIt = basePose.movedProperties.find(slab);
    CHECK(baseIt != basePose.movedProperties.end());
    if (baseIt == basePose.movedProperties.end()) {
        return;
    }
    const VtVec3fArray before = baseIt->second.Get<VtVec3fArray>();

    // A property mover that drives the smooth mover's strength from the
    // authored 0.6 to 0. It is a sibling of VolumeCorrect with an authored
    // reorder, so the competing-writer rule is satisfied and the walk order
    // is unambiguous.
    UsdStageRefPtr edited =
        UsdStage::Open(examplesDir + "/06_LatticeBulge.usda");
    CHECK(edited);
    if (!edited) {
        return;
    }
    edited->SetEditTarget(edited->GetSessionLayer());
    const UsdPrim mover = edited->DefinePrim(
        SdfPath("/LatticeAsset/Rig/Movers/Geometry/KillSmoothing"),
        TfToken("RigExecFloatMathMover"));
    CHECK(mover);
    if (!mover) {
        return;
    }
    mover.CreateAttribute(TfToken("rigExec:operation"),
                          SdfValueTypeNames->Token, /*custom=*/false)
        .Set(TfToken("blend"));
    mover.CreateAttribute(TfToken("inputs:value"), SdfValueTypeNames->Float,
                          /*custom=*/false)
        .Set(0.0f);
    mover.CreateRelationship(TfToken("rigExec:moves"), /*custom=*/false)
        .SetTargets({strength});

    RigExecRigEvaluator evaluator(edited, rigPath);
    std::vector<std::string> errors;
    if (!evaluator.Compile(&errors)) {
        for (const std::string &e : errors) {
            std::printf("    %s\n", e.c_str());
        }
    }
    CHECK(errors.empty());
    const RigExecRigPose pose = evaluator.Evaluate(when);
    CHECK(pose.valid);

    // The property chain ran and published 0...
    const auto strengthIt = pose.movedProperties.find(strength);
    CHECK(strengthIt != pose.movedProperties.end());
    if (strengthIt != pose.movedProperties.end()) {
        CHECK(std::abs(strengthIt->second.Get<float>()) < 1e-6f);
    }

    // ...and the smoothing CHANGED, because the smooth mover's packet read
    // the revised strength rather than the authored 0.6.
    const auto afterIt = pose.movedProperties.find(slab);
    CHECK(afterIt != pose.movedProperties.end());
    if (afterIt == pose.movedProperties.end()) {
        return;
    }
    const VtVec3fArray after = afterIt->second.Get<VtVec3fArray>();
    CHECK(before.size() == after.size());
    double worst = 0;
    for (size_t i = 0; i < before.size() && i < after.size(); ++i) {
        worst = std::max(worst, double((before[i] - after[i]).GetLength()));
    }
    if (worst <= 1e-5) {
        std::printf("  driving inputs:strength to 0 did not change the "
                    "smoothing: the property mover's value is not reaching "
                    "the packet assembler\n");
    }
    CHECK(worst > 1e-5);

    // The graph and the CPU oracle resolve their inputs independently, so an
    // agreement here is what says both routes carry the SAME revised value.
    CHECK(pose.moverGraphParityMismatches == 0);
    CHECK(pose.moverGraphParityAgreements > 0);
}

// A read phase authored as property metadata selects WHICH revision of an
// input a mover consumes.
//
// 13 deforms a Slab through a cage that is itself deformed by two movers, so
// the three phases are three different slabs from one rig with no other edit:
//   base                 the authored cage -- the slab does not move at all;
//   <CageLift>           the cage as of that mover -- lifted, not twisted;
//   final                the cage after both -- lifted and twisted.
//
// Asserting the DISPLACEMENTS, not just that they differ: the controls put
// ty=2 on the lift and tx=1.5 on the twist at 1024, so the three answers have
// to be 0, 2, and sqrt(1.5^2 + 2^2) = 2.5. Anything else means the phase
// selected a revision, but not the one it names.
static void
TestReadPhaseSelectsRevision(const std::string &examplesDir)
{
    const SdfPath rigPath("/ReadPhaseAsset/Rig");
    const SdfPath slab("/ReadPhaseAsset/Geom/Slab.points");
    const SdfPath cageRel(
        "/ReadPhaseAsset/Rig/Movers/Geometry/SlabLattice.rigExec:cage");
    const UsdTimeCode when(1024);

    auto slabDisplacement = [&](const char *phase, double *out) {
        UsdStageRefPtr stage =
            UsdStage::Open(examplesDir + "/13_ReadPhases.usda");
        CHECK(stage);
        if (!stage) {
            return false;
        }
        stage->SetEditTarget(stage->GetSessionLayer());
        const UsdRelationship rel = stage->GetRelationshipAtPath(cageRel);
        CHECK(rel);
        if (!rel) {
            return false;
        }
        rel.SetMetadata(TfToken("rigExecReadPhase"), std::string(phase));

        RigExecRigEvaluator evaluator(stage, rigPath);
        std::vector<std::string> errors;
        if (!evaluator.Compile(&errors)) {
            for (const std::string &e : errors) {
                std::printf("    %s\n", e.c_str());
            }
            return false;
        }
        const RigExecRigPose pose = evaluator.Evaluate(when);
        CHECK(pose.valid);
        // The graph and the CPU oracle resolve the phase independently, so
        // an agreement is what says they selected the SAME revision.
        CHECK(pose.moverGraphParityMismatches == 0);
        CHECK(pose.moverGraphParityAgreements > 0);

        const auto it = pose.movedProperties.find(slab);
        CHECK(it != pose.movedProperties.end());
        if (it == pose.movedProperties.end()) {
            return false;
        }
        const VtVec3fArray moved = it->second.Get<VtVec3fArray>();
        VtVec3fArray authored;
        stage->GetAttributeAtPath(slab).Get(&authored, when);
        CHECK(moved.size() == authored.size());
        double worst = 0;
        for (size_t i = 0; i < moved.size() && i < authored.size(); ++i) {
            worst = std::max(worst,
                             double((moved[i] - authored[i]).GetLength()));
        }
        *out = worst;
        return true;
    };

    double atBase = -1, atLift = -1, atFinal = -1;
    CHECK(slabDisplacement("base", &atBase));
    CHECK(slabDisplacement("/ReadPhaseAsset/Rig/Movers/Cage/CageLift",
                           &atLift));
    CHECK(slabDisplacement("final", &atFinal));

    if (std::abs(atBase) > 1e-4) {
        std::printf("  base phase moved the slab by %g; the authored cage "
                    "should deform nothing\n", atBase);
    }
    CHECK(std::abs(atBase) < 1e-4);

    if (std::abs(atLift - 2.0) > 1e-3) {
        std::printf("  phase at CageLift gave %g; expected 2 (ty only)\n",
                    atLift);
    }
    CHECK(std::abs(atLift - 2.0) < 1e-3);

    if (std::abs(atFinal - 2.5) > 1e-3) {
        std::printf("  final phase gave %g; expected 2.5 "
                    "(sqrt(1.5^2 + 2^2))\n", atFinal);
    }
    CHECK(std::abs(atFinal - 2.5) < 1e-3);
}

// A phase is compiled wiring, so editing one begins a new epoch and the next
// Evaluate rebuilds -- exactly like retargeting the relationship it sits on.
static void
TestReadPhaseIsStructural(const std::string &examplesDir)
{
    UsdStageRefPtr stage =
        UsdStage::Open(examplesDir + "/13_ReadPhases.usda");
    CHECK(stage);
    if (!stage) {
        return;
    }
    RigExecRigEvaluator evaluator(stage, SdfPath("/ReadPhaseAsset/Rig"));
    CHECK(evaluator.Compile(nullptr));
    CHECK(evaluator.Evaluate(UsdTimeCode(1024)).valid);
    const size_t before = evaluator.GetBindingEpochDigest();

    stage->SetEditTarget(stage->GetSessionLayer());
    const UsdRelationship rel = stage->GetRelationshipAtPath(SdfPath(
        "/ReadPhaseAsset/Rig/Movers/Geometry/SlabLattice.rigExec:cage"));
    CHECK(rel);
    if (!rel) {
        return;
    }
    rel.SetMetadata(TfToken("rigExecReadPhase"), std::string("base"));

    const RigExecRigPose pose = evaluator.Evaluate(UsdTimeCode(1024));
    CHECK(pose.valid);
    if (evaluator.GetBindingEpochDigest() == before) {
        std::printf("  editing a read phase did not change the epoch "
                    "digest; the compiled binding will go stale\n");
    }
    CHECK(evaluator.GetBindingEpochDigest() != before);
}

// Phases that cannot be satisfied are rejected at compile rather than
// silently reading something else.
static void
TestBadReadPhasesRejected(const std::string &examplesDir)
{
    const SdfPath cageRel(
        "/ReadPhaseAsset/Rig/Movers/Geometry/SlabLattice.rigExec:cage");

    auto compileWith = [&](const char *phase, std::vector<std::string> *errors) {
        UsdStageRefPtr stage =
            UsdStage::Open(examplesDir + "/13_ReadPhases.usda");
        if (!stage) {
            return true;
        }
        stage->SetEditTarget(stage->GetSessionLayer());
        const UsdRelationship rel = stage->GetRelationshipAtPath(cageRel);
        if (!rel) {
            return true;
        }
        rel.SetMetadata(TfToken("rigExecReadPhase"), std::string(phase));
        RigExecRigEvaluator evaluator(stage, SdfPath("/ReadPhaseAsset/Rig"));
        return evaluator.Compile(errors);
    };

    // Not a phase keyword and not a path.
    std::vector<std::string> errors;
    CHECK(!compileWith("halfway", &errors));
    CHECK(!errors.empty());

    // A relative path: there are two plausible things to resolve it against,
    // so it is rejected rather than guessed.
    errors.clear();
    CHECK(!compileWith("Movers/Cage/CageLift", &errors));

    // A well-formed path that writes nothing to the cage.
    errors.clear();
    CHECK(!compileWith("/ReadPhaseAsset/Rig/Movers/Geometry", &errors));
    bool sawWritesNothing = false;
    for (const std::string &e : errors) {
        sawWritesNothing |= e.find("writes nothing") != std::string::npos;
    }
    CHECK(sawWritesNothing);

    // Naming a Scope that DOES contain writers is legal: post-order means
    // "after everything beneath it".
    errors.clear();
    CHECK(compileWith("/ReadPhaseAsset/Rig/Movers/Cage", &errors));
}

// Lifting the joint requirement is not the same as removing the gate: a rig
// with neither joints nor movers publishes nothing at all, which is an
// authoring mistake and must still be reported as one.
static void
TestRigWithNoOutputsRejected(const std::string &examplesDir)
{
    UsdStageRefPtr stage =
        UsdStage::Open(examplesDir + "/rigexec_flat.usda");
    CHECK(stage);
    if (!stage) {
        return;
    }
    stage->SetEditTarget(stage->GetSessionLayer());
    // Deactivating the only mover leaves the rig with no outputs of any kind.
    const UsdPrim mover = stage->GetPrimAtPath(
        SdfPath("/World/RigRoot/Movers/RigExecAimConstraint1"));
    CHECK(mover);
    if (!mover) {
        return;
    }
    mover.SetActive(false);

    RigExecRigEvaluator evaluator(stage, SdfPath("/World/RigRoot"));
    std::vector<std::string> errors;
    CHECK(!evaluator.Compile(&errors));
    bool sawOutputsError = false;
    for (const std::string &e : errors) {
        sawOutputsError |= e.find("publishes no outputs") != std::string::npos;
    }
    CHECK(sawOutputsError);
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
    TestPropertyMathMoversAreEvaluated(examplesDir);
    TestPropertyMoverFeedsConsumingComputation(examplesDir);
    TestDisabledPropertyMoverPassesThrough(examplesDir);
    TestPropertyMoverTypeMismatchRejected(examplesDir);
    TestAimConstraintDrivesXform(examplesDir);
    TestJointFreeRigPublishesXform(examplesDir);
    TestRigWithNoOutputsRejected(examplesDir);
    TestMoverOrderIsComposedAndDynamic();
    TestReadPhaseSelectsRevision(examplesDir);
    TestReadPhaseIsStructural(examplesDir);
    TestBadReadPhasesRejected(examplesDir);
    TestPointChainConsumesPrecedingRevision(examplesDir);
    TestPropertyMoverReachesStaticPacketReads(examplesDir);

    if (failures) {
        std::printf("%d FAILURE(S)\n", failures);
        return 1;
    }
    std::printf("testRigExecArm: all tests passed\n");
    return 0;
}
