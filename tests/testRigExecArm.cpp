// RigExec end-to-end evaluation tests over the spec §4.5/§4.6 arm assets:
// OpenExec-evaluated transforms (controls, FK, IK, blend, joints), native
// USD animation resolution parity, and the staged geometry mover pipeline.
// argv[1] = path to the examples directory (containing ArmRig.usda and
// ArmShotAnim.usda). The codeless schema plugin is expected at
// <examples>/../plugin/rigExecSchema/resources.
#include "rigExec/rigEvaluator.h"
#include "rigExec/tapSet.h"
#include "rigExec/types.h"
#include "rigExecMath/avarScale.h"
#include "rigExecMath/pointFrame.h"
#include "rigExecMath/geometryKernels.h"
#include "rigExecMath/solvers.h"

#include "pxr/base/gf/rotation.h"
#include "pxr/base/plug/registry.h"
#include "pxr/base/tf/pathUtils.h"
#include "pxr/base/ts/knot.h"
#include "pxr/base/ts/spline.h"
#include "pxr/usd/sdf/layer.h"
#include "pxr/usd/sdf/types.h"
#include "pxr/usd/usd/attribute.h"
#include "pxr/usd/usd/prim.h"
#include "pxr/usd/usd/primRange.h"
#include "pxr/usd/usd/relationship.h"
#include "pxr/usd/usd/stage.h"

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <cstring>
#include <functional>
#include <limits>
#include <map>
#include <set>
#include <string>

using namespace rigExec;

// A broken operation is set aside with a warning instead of failing the rig
// (RigExecRigEvaluator::_CompileEpoch): the rest compiles and evaluates.
// True when the compile succeeded and set aside every one of `operations`.
static bool
SkipsOperations(RigExecRigEvaluator &evaluator,
                const std::vector<SdfPath> &operations,
                std::vector<std::string> *errors)
{
    if (!evaluator.Compile(errors)) {
        return false;
    }
    for (const SdfPath &operation : operations) {
        if (!evaluator.GetSkippedOperations().count(operation)) {
            return false;
        }
    }
    return !operations.empty();
}

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
GetAvar(const UsdPrim &prim, const char *name, UsdTimeCode time,
        double fallback = 0)
{
    double value = fallback;
    if (UsdAttribute a = prim.GetAttribute(TfToken(name))) {
        a.Get(&value, time);
    }
    return value;
}

// The local avar transform: scale, XYZ-ordered rotations (degrees), then
// rspin about +X, then translation (row-vector convention).
static GfMatrix4d
ComposeAvars(const UsdPrim &prim, UsdTimeCode time)
{
    static const GfVec3d axes[3] = {
        GfVec3d(1, 0, 0), GfVec3d(0, 1, 0), GfVec3d(0, 0, 1)};
    const double angles[3] = {
        GetAvar(prim, "avars:rx", time), GetAvar(prim, "avars:ry", time),
        GetAvar(prim, "avars:rz", time)};
    GfMatrix4d m(1.0);
    m.SetScale(GfVec3d(
        RigExecNormalizeAvarScale(
            GetAvar(prim, "avars:sx", time, 1)),
        RigExecNormalizeAvarScale(
            GetAvar(prim, "avars:sy", time, 1)),
        RigExecNormalizeAvarScale(
            GetAvar(prim, "avars:sz", time, 1))));
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
    eval.cpuParityMode = true;
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
    // ArmRig's IK names Shoulder/Elbow/Wrist itself, so the kernel
    // measures its own bones (4 and 4) from their rests. IKFKBlend
    // still POSES those joints -- the IK's rigExec:joints is a rest
    // reference because its aggregate is consumed by the blend.
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
            eval.cpuParityMode = true;
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
            eval.cpuParityMode = true;
            std::vector<std::string> errors;
            CHECK(SkipsOperations(eval, {blendPath}, &errors));
            CHECK(!errors.empty());
        }
    }

    // A joint written by two solvers STACKS. rigExec:joints is an ordered
    // write, not an exclusive claim, so a second unconsumed chain over a
    // shoulder the blend already writes is a legal two-writer stack: both
    // solvers run, the last one in stack order supplies the joint's frame,
    // and the compile is SILENT about it -- stacking is ordinary authoring
    // under the unified pose stack (spec 4.2), not a shape worth a
    // diagnostic. The order is read from the compiled chains.
    // Both writers must be solvers nobody READS: a solver whose aggregate is
    // consumed does not write at all, so naming a joint its consumer writes
    // is a rest reference, exercised separately below.
    {
        UsdStageRefPtr stage = UsdStage::Open(examplesDir + "/ArmRig.usda");
        CHECK(stage);
        if (stage) {
            // A second unconsumed chain writing a joint IKFKBlend writes.
            UsdPrim rogue = stage->DefinePrim(
                SdfPath("/ArmAsset/Rig/Solvers/Rogue"),
                TfToken("RigExecFkChain"));
            CHECK(rogue);
            rogue.CreateRelationship(TfToken("rigExec:controls")).SetTargets(
                {SdfPath("/ArmAsset/Rig/Controls/ShoulderFK")});
            rogue.CreateRelationship(jointsTok).SetTargets(
                {SdfPath("/ArmAsset/Rig/Joints/Shoulder")});
            RigExecRigEvaluator eval(stage, rigPath);
            eval.cpuParityMode = true;
            std::vector<std::string> errors;
            CHECK(eval.Compile(&errors));
            CHECK(errors.empty());
            // Rogue is DEFINED last under /Solvers, and the stack is the
            // reverse of the composed order, so Rogue writes FIRST and
            // IKFKBlend -- the one nothing reads and everything downstream
            // expects -- is still the last writer of the shoulder. Asserted
            // INSIDE the shoulder's own chain: a search over every chain
            // would pass on whichever one happened to carry that order.
            const auto &chains = eval.GetFrameChains();
            const auto shoulder =
                chains.find(SdfPath("/ArmAsset/Rig/Joints/Shoulder"));
            CHECK(shoulder != chains.end());
            if (shoulder != chains.end()) {
                const std::vector<SdfPath> &chain = shoulder->second;
                CHECK(chain.size() == 2);
                if (chain.size() == 2) {
                    CHECK(chain[0] ==
                          SdfPath("/ArmAsset/Rig/Solvers/Rogue"));
                    CHECK(chain[1] ==
                          SdfPath("/ArmAsset/Rig/Solvers/IKFKBlend"));
                }
            }
            CHECK(eval.Evaluate(UsdTimeCode(1001)).valid);
        }
    }

    // A CONSUMED solver naming joints its consumer writes is a REST
    // reference, not a second entry in that joint's stack: the relaxation
    // that decides so is a disposition, not claim arbitration, and it is
    // what keeps the IK/FK idiom a single-writer picture. That is how an IK
    // feeding an IK/FK blend declares the chain whose rests give it its bone
    // lengths. ArmRig's own IK does exactly this.
    {
        UsdStageRefPtr stage = UsdStage::Open(examplesDir + "/ArmRig.usda");
        CHECK(stage);
        if (stage) {
            UsdPrim fk =
                stage->GetPrimAtPath(SdfPath("/ArmAsset/Rig/Solvers/FK"));
            CHECK(fk);
            fk.CreateRelationship(jointsTok).SetTargets(
                {SdfPath("/ArmAsset/Rig/Joints/Shoulder")});
            RigExecRigEvaluator eval(stage, rigPath);
            eval.cpuParityMode = true;
            std::vector<std::string> errors;
            CHECK(eval.Compile(&errors));
            for (const std::string &error : errors) {
                std::printf("  unexpected: %s\n", error.c_str());
            }
            CHECK(errors.empty());
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
            eval.cpuParityMode = true;
            std::vector<std::string> errors;
            CHECK(SkipsOperations(
                eval, {SdfPath("/ArmAsset/Rig/Solvers/Group")}, &errors));
            CHECK(!errors.empty());
        }
    }

    // A cardinality-shrinking edit (fewer FK controls than bound elements)
    // must change the structure digest so Evaluate() recompiles and Phase A
    // skips the solver with the now-out-of-range binding: the
    // digest, not just rigExec:joints, must cover cardinality inputs.
    {
        UsdStageRefPtr stage =
            UsdStage::Open(examplesDir + "/01_FkChainTail.usda");
        CHECK(stage);
        if (stage) {
            const SdfPath tailRig("/TailAsset/Rig");
            RigExecRigEvaluator eval(stage, tailRig);
            eval.cpuParityMode = true;
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
                    CHECK(pose.valid);
                    CHECK(eval.GetSkippedOperations().count(fk.GetPath()) == 1);
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
            eval.cpuParityMode = true;
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
    // docs/specs/mover-graph-cutover.md). The summary line is asserted present, not
    // just the absence of a mismatch: a parity pass that silently checked
    // nothing would otherwise read exactly like one that passed.
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

    evaluator.cpuParityMode = true;
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

    // A static sample has identical base/final/preceding values. Phase edits
    // remain valid and preserve that result without rebuilding point nodes.
    sample.GetRelationship(TfToken("rigExec:targetPoints"))
        .SetMetadata(TfToken(RigExecReadPhaseMetadataName), std::string("final"));
    const auto finalSample = evaluator.Evaluate(UsdTimeCode::Default());
    CHECK(finalSample.valid);
    CHECK(finalSample.moverGraphRevisionsCreated == 0);
    sample.GetRelationship(TfToken("rigExec:targetPoints"))
        .SetMetadata(TfToken(RigExecReadPhaseMetadataName), std::string("base"));
    CHECK(evaluator.Evaluate(UsdTimeCode::Default()).valid);
    sample.GetRelationship(TfToken("rigExec:targetPoints"))
        .SetMetadata(TfToken(RigExecReadPhaseMetadataName), std::string("preceding"));
    CHECK(evaluator.Evaluate(UsdTimeCode::Default()).valid);
}

static size_t
DerivedEvaluateCount(const RigExecRigEvaluator &evaluator)
{
    size_t count = 0;
    for (const RigExecProfileSummaryRow &row :
         evaluator.GetProfiler().Summarize()) {
        if (row.name.find("DerivedEvaluate") != std::string::npos) {
            count += row.count;
        }
    }
    return count;
}

static VtVec3fArray
MovedArray(const RigExecRigPose &pose, const SdfPath &target)
{
    const auto it = pose.movedProperties.find(target);
    CHECK(it != pose.movedProperties.end());
    if (it == pose.movedProperties.end()) {
        return {};
    }
    return it->second.Get<VtVec3fArray>();
}

// Derived normals/extent are a pure function of the chain's final points,
// the authored base, and the assembled topology: an unchanged tuple
// republishes the stored result without running the derived graphs, while a
// moved mesh recomputes.
static void
TestDerivedMaintenanceDeferral()
{
    UsdStageRefPtr stage = UsdStage::CreateInMemory();
    stage->DefinePrim(SdfPath("/Asset"), TfToken("Xform"));
    UsdPrim mesh = stage->DefinePrim(SdfPath("/Asset/Geom/M"), TfToken("Mesh"));
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
    mesh.CreateAttribute(TfToken("normals"),
                         SdfValueTypeNames->Normal3fArray)
        .Set(VtVec3fArray{base.size(), GfVec3f(0, 0, 1)});
    mesh.CreateAttribute(TfToken("extent"),
                         SdfValueTypeNames->Float3Array)
        .Set(VtVec3fArray{GfVec3f(0, 0, 0), GfVec3f(1, 1, 0)});
    stage->DefinePrim(SdfPath("/Asset/Rig"), TfToken("RigExecRoot"));
    UsdPrim joint = stage->DefinePrim(SdfPath("/Asset/Rig/Joints/J"),
                                      TfToken("RigExecJoint"));
    GfMatrix4d posedSpace(1.0);
    posedSpace.SetTranslate(GfVec3d(0, 2, 0));
    joint.CreateAttribute(TfToken("posed:space"),
                          SdfValueTypeNames->Matrix4d)
        .Set(posedSpace);
    UsdPrim weight = stage->DefinePrim(
        SdfPath("/Asset/Rig/Weights/All"), TfToken("RigExecStaticWeight"));
    weight.CreateRelationship(TfToken("rigExec:weightTarget"))
        .SetTargets({SdfPath("/Asset/Geom/M.points")});
    weight.CreateAttribute(TfToken("rigExec:representation"),
                           SdfValueTypeNames->Token).Set(TfToken("dense"));
    weight.CreateAttribute(TfToken("rigExec:values"),
                           SdfValueTypeNames->FloatArray)
        .Set(VtFloatArray{1, 1, 1, 1});
    weight.CreateAttribute(TfToken("rigExec:defaultWeight"),
                           SdfValueTypeNames->Float).Set(0.0f);
    UsdPrim mover = stage->DefinePrim(
        SdfPath("/Asset/Rig/Movers/MM"), TfToken("RigExecMatrixMover"));
    mover.ApplyAPI(TfToken("RigExecMoverAPI"));
    mover.CreateRelationship(TfToken("rigExec:moves"))
        .SetTargets({SdfPath("/Asset/Geom/M.points")});
    mover.CreateRelationship(TfToken("rigExec:transform"))
        .SetTargets({joint.GetPath()});
    mover.CreateRelationship(TfToken("rigExec:weightObject"))
        .SetTargets({weight.GetPath()});

    RigExecRigEvaluator evaluator(stage, SdfPath("/Asset/Rig"));
    evaluator.SetProfilingEnabled(true);
    std::vector<std::string> errors;
    CHECK(evaluator.Compile(&errors));
    const SdfPath normalsPath("/Asset/Geom/M.normals");
    const SdfPath extentPath("/Asset/Geom/M.extent");
    const auto first = evaluator.Evaluate(UsdTimeCode::Default());
    CHECK(first.valid);
    const size_t derivedRuns = DerivedEvaluateCount(evaluator);
    CHECK(derivedRuns == 2);
    const VtVec3fArray firstNormals = MovedArray(first, normalsPath);
    const VtVec3fArray firstExtent = MovedArray(first, extentPath);
    CHECK(firstNormals.size() == 4 && firstExtent.size() == 2);

    const auto second = evaluator.Evaluate(UsdTimeCode::Default());
    CHECK(second.valid);
    CHECK(DerivedEvaluateCount(evaluator) == derivedRuns);
    CHECK(second.moverGraphRevisionsExecuted == 0);
    const VtVec3fArray secondNormals = MovedArray(second, normalsPath);
    const VtVec3fArray secondExtent = MovedArray(second, extentPath);
    CHECK(secondNormals.size() == firstNormals.size());
    for (size_t i = 0; i < firstNormals.size() && i < secondNormals.size(); ++i) {
        CHECK(Near(GfVec3d(secondNormals[i]), GfVec3d(firstNormals[i]), 1e-6));
    }
    CHECK(secondExtent.size() == firstExtent.size());
    for (size_t i = 0; i < firstExtent.size() && i < secondExtent.size(); ++i) {
        CHECK(Near(GfVec3d(secondExtent[i]), GfVec3d(firstExtent[i]), 1e-6));
    }

    GfMatrix4d movedSpace(1.0);
    movedSpace.SetTranslate(GfVec3d(0, 3, 0));
    joint.GetAttribute(TfToken("posed:space")).Set(movedSpace);
    const auto third = evaluator.Evaluate(UsdTimeCode::Default());
    CHECK(third.valid);
    CHECK(DerivedEvaluateCount(evaluator) > derivedRuns);
    const VtVec3fArray thirdExtent = MovedArray(third, extentPath);
    CHECK(thirdExtent.size() == 2);
    if (thirdExtent.size() == 2) {
        CHECK(Near(GfVec3d(thirdExtent[0]), GfVec3d(0, 3, 0), 1e-5));
        CHECK(Near(GfVec3d(thirdExtent[1]), GfVec3d(1, 4, 0), 1e-5));
    }
}

// Universal mover envelope

static RigExecRigPose
EvaluateEnvelopeFixture(const UsdStageRefPtr &stage)
{
    RigExecRigEvaluator evaluator(stage, SdfPath("/Asset/Rig"));
    evaluator.cpuParityMode = true;
    std::vector<std::string> errors;
    const bool compiled = evaluator.Compile(&errors);
    if (!compiled) {
        for (const std::string &error : errors) {
            std::printf("universal-weight compile error: %s\n", error.c_str());
        }
    }
    CHECK(compiled);
    if (!compiled) {
        return RigExecRigPose();
    }
    RigExecRigPose pose = evaluator.Evaluate(UsdTimeCode::Default());
    CHECK(pose.valid);
    CHECK(pose.moverGraphParityMismatches == 0);
    return pose;
}

static VtVec3fArray
EnvelopePoints(const RigExecRigPose &pose, const SdfPath &target)
{
    const auto it = pose.movedProperties.find(target);
    CHECK(it != pose.movedProperties.end());
    if (it == pose.movedProperties.end()) {
        return {};
    }
    CHECK(it->second.IsHolding<VtVec3fArray>());
    return it->second.IsHolding<VtVec3fArray>()
        ? it->second.UncheckedGet<VtVec3fArray>()
        : VtVec3fArray();
}

// Scale avars are part of the control's local affine transform, not guide
// display metadata. Verify animated non-uniform scale survives the point-frame
// representation, combines with rotation/translation, and reaches a
// MatrixMover's native points target.
static void
TestControlAvarScaleDrivesMatrixMover()
{
    const UsdStageRefPtr stage = UsdStage::CreateInMemory();
    const SdfPath rigPath("/Asset/Rig");
    const SdfPath controlPath("/Asset/Rig/Controls/C");
    const SdfPath target("/Asset/Geom/P.points");
    const VtVec3fArray base = {
        GfVec3f(0, 0, 0), GfVec3f(1, 0, 0), GfVec3f(0, 1, 0),
        GfVec3f(0, 0, 1), GfVec3f(1, 2, 3)};

    // Volume placement is intentionally rigid and exposes inputs:scaleX/Y/Z
    // instead. Future schema regeneration must not leak the control/joint
    // avars onto this sibling RigExecXformable subtype.
    const UsdPrim volumeDefinitionProbe = stage->DefinePrim(
        SdfPath("/VolumeDefinitionProbe"), TfToken("RigExecSphereWeight"));
    CHECK(!volumeDefinitionProbe.GetAttribute(TfToken("avars:sx")));
    CHECK(!volumeDefinitionProbe.GetAttribute(TfToken("avars:sy")));
    CHECK(!volumeDefinitionProbe.GetAttribute(TfToken("avars:sz")));

    const UsdPrim points =
        stage->DefinePrim(target.GetPrimPath(), TfToken("Points"));
    points.CreateAttribute(
              TfToken("points"), SdfValueTypeNames->Point3fArray, false)
        .Set(base);
    stage->DefinePrim(rigPath, TfToken("RigExecRoot"));
    const UsdPrim control =
        stage->DefinePrim(controlPath, TfToken("RigExecControl"));

    // Two samples prove that scale is a timed computation input. T/R remain
    // authored constants so the non-commutative S * R * T order is explicit.
    const GfVec3d scaleAtZero(1, 1, 1);
    const GfVec3d scaleAtTen(2, 3, 4);
    static const char *scaleNames[3] = {
        "avars:sx", "avars:sy", "avars:sz"};
    for (int axis = 0; axis < 3; ++axis) {
        const UsdAttribute attr = control.CreateAttribute(
            TfToken(scaleNames[axis]), SdfValueTypeNames->Double, false);
        CHECK(attr.Set(scaleAtZero[axis], UsdTimeCode(0)));
        CHECK(attr.Set(scaleAtTen[axis], UsdTimeCode(10)));
    }
    control.CreateAttribute(
               TfToken("avars:rz"), SdfValueTypeNames->Double, false)
        .Set(90.0);
    control.CreateAttribute(
               TfToken("avars:tx"), SdfValueTypeNames->Double, false)
        .Set(5.0);
    control.CreateAttribute(
               TfToken("avars:ty"), SdfValueTypeNames->Double, false)
        .Set(-2.0);
    control.CreateAttribute(
               TfToken("avars:tz"), SdfValueTypeNames->Double, false)
        .Set(7.0);

    const UsdPrim mover = stage->DefinePrim(
        SdfPath("/Asset/Rig/Movers/M"), TfToken("RigExecMatrixMover"));
    CHECK(mover.ApplyAPI(TfToken("RigExecMoverAPI")));
    mover.CreateRelationship(TfToken("rigExec:moves"), false)
        .SetTargets({target});
    mover.CreateRelationship(TfToken("rigExec:transform"), false)
        .SetTargets({controlPath});
    mover.CreateAttribute(
             TfToken("inputs:defaultWeight"), SdfValueTypeNames->Float, false)
        .Set(1.0f);

    RigExecRigEvaluator evaluator(stage, rigPath);

    evaluator.cpuParityMode = true;
    std::vector<std::string> errors;
    const bool compiled = evaluator.Compile(&errors);
    if (!compiled) {
        for (const std::string &error : errors) {
            std::printf("control-scale compile error: %s\n", error.c_str());
        }
    }
    CHECK(compiled);
    if (!compiled) {
        return;
    }

    auto expectedMatrix = [](const GfVec3d &scale) {
        GfMatrix4d result(1.0);
        result.SetScale(scale);
        result = result * GfMatrix4d(
                              GfRotation(GfVec3d(0, 0, 1), 90.0),
                              GfVec3d(0));
        GfMatrix4d translation(1.0);
        translation.SetTranslate(GfVec3d(5, -2, 7));
        return result * translation;
    };

    auto checkAtTime = [&](RigExecRigEvaluator &activeEvaluator, double time,
                           const GfVec3d &scale) {
        const RigExecRigPose pose =
            activeEvaluator.Evaluate(UsdTimeCode(time));
        CHECK(pose.valid);
        CHECK(pose.moverGraphParityMismatches == 0);

        const GfMatrix4d expected = expectedMatrix(scale);
        const auto expectedFrame = MatrixLandmarks(expected);
        const auto frameIt = pose.controlFrames.find(controlPath);
        CHECK(frameIt != pose.controlFrames.end());
        if (frameIt != pose.controlFrames.end()) {
            for (size_t i = 0; i < expectedFrame.size(); ++i) {
                CHECK(Near(frameIt->second.points[i], expectedFrame[i], 1e-9));
            }
        }

        const VtVec3fArray moved = EnvelopePoints(pose, target);
        CHECK(moved.size() == base.size());
        for (size_t i = 0; i < moved.size() && i < base.size(); ++i) {
            CHECK(Near(GfVec3d(moved[i]),
                       expected.TransformAffine(GfVec3d(base[i])), 1e-5));
        }
    };

    for (const auto &[time, scale] :
         std::vector<std::pair<double, GfVec3d>>{
             {0.0, scaleAtZero}, {10.0, scaleAtTen}}) {
        checkAtTime(evaluator, time, scale);
    }

    // Match the user's authored stage exactly: each scale channel has one
    // spline knot. Ts holds that value outside the knot range, so the same
    // non-unit affine must reach the MatrixMover before, at, and after it.
    const GfVec3d singleKnotScale(2.5, 1.25, 3.75);
    const TfType doubleType = TfType::Find<double>();
    for (int axis = 0; axis < 3; ++axis) {
        // Non-const: UsdAttribute::SetSpline is const-only from 26.08; the
        // Vendored USD 26.05 still takes a mutable handle.
        UsdAttribute attr = control.GetAttribute(
            TfToken(scaleNames[axis]));
        CHECK(attr.ClearAtTime(UsdTimeCode(0)));
        CHECK(attr.ClearAtTime(UsdTimeCode(10)));
        TsSpline spline(doubleType);
        TsKnot knot(doubleType);
        knot.SetTime(12.0);
        knot.SetValue(singleKnotScale[axis]);
        knot.SetNextInterpolation(TsInterpLinear);
        spline.SetKnot(knot);
        CHECK(attr.SetSpline(spline));
        CHECK(attr.HasSpline());
    }

    RigExecRigEvaluator splineEvaluator(stage, rigPath);

    splineEvaluator.cpuParityMode = true;
    errors.clear();
    const bool splineCompiled = splineEvaluator.Compile(&errors);
    if (!splineCompiled) {
        for (const std::string &error : errors) {
            std::printf(
                "single-knot control-scale compile error: %s\n",
                error.c_str());
        }
    }
    CHECK(splineCompiled);
    if (!splineCompiled) {
        return;
    }
    for (const double time : {-20.0, 12.0, 100.0}) {
        checkAtTime(splineEvaluator, time, singleKnotScale);
    }

    // Raw USD can bypass both strict authoring surfaces, so the computation
    // itself applies the same contract. Tiny values retain reflection sign;
    // non-finite values become identity scale instead of poisoning the frame.
    auto checkRawScale = [&](const GfVec3d &raw,
                             const GfVec3d &normalized) {
        for (int axis = 0; axis < 3; ++axis) {
            const UsdAttribute attr = control.GetAttribute(
                TfToken(scaleNames[axis]));
            CHECK(attr.Clear());
            CHECK(attr.Set(raw[axis]));
        }
        RigExecRigEvaluator rawEvaluator(stage, rigPath);
        rawEvaluator.cpuParityMode = true;
        errors.clear();
        const bool rawCompiled = rawEvaluator.Compile(&errors);
        if (!rawCompiled) {
            for (const std::string &error : errors) {
                std::printf("raw control-scale compile error: %s\n",
                            error.c_str());
            }
        }
        CHECK(rawCompiled);
        if (rawCompiled) {
            checkAtTime(rawEvaluator, 0.0, normalized);
        }
    };

    checkRawScale(
        GfVec3d(0.0, -0.0, -0.5 * RigExecAvarScaleFloor),
        GfVec3d(RigExecAvarScaleFloor, -RigExecAvarScaleFloor,
                -RigExecAvarScaleFloor));
    checkRawScale(
        GfVec3d(std::numeric_limits<double>::quiet_NaN(),
                std::numeric_limits<double>::infinity(),
                -std::numeric_limits<double>::infinity()),
        GfVec3d(1.0));
}

// Authors either a dense point field or a constant one-element field.  The
// mover's inputs:defaultWeight is deliberately authored to a DIFFERENT value
// in the callers: a related object supersedes the scalar fallback rather than
// multiplying it.
static UsdPrim
MakeEnvelopeWeight(
    const UsdStageRefPtr &stage, const SdfPath &path,
    const SdfPath &target, const VtFloatArray &values,
    float constantValue = 0.0f)
{
    const UsdPrim weight =
        stage->DefinePrim(path, TfToken("RigExecStaticWeight"));
    weight.CreateRelationship(TfToken("rigExec:weightTarget"), false)
        .SetTargets({target});
    const bool constant = values.empty();
    weight.CreateAttribute(TfToken("rigExec:representation"),
                           SdfValueTypeNames->Token, false)
        .Set(TfToken(constant ? "constant" : "dense"));
    weight.CreateAttribute(TfToken("rigExec:defaultWeight"),
                           SdfValueTypeNames->Float, false)
        .Set(constant ? constantValue : 0.0f);
    if (!constant) {
        weight.CreateAttribute(TfToken("rigExec:values"),
                               SdfValueTypeNames->FloatArray, false)
            .Set(values);
    }
    return weight;
}

// MatrixMover is the critical regression: a uniform move no longer needs a
// boilerplate constant RigExecStaticWeight.  When a spatial object is bound,
// its total field replaces the uniform fallback point by point.
static void
TestMatrixMoverUniversalEnvelope()
{
    const SdfPath target("/Asset/Geom/P.points");
    const VtVec3fArray base = {
        GfVec3f(0, 0, 0), GfVec3f(1, 0, 0), GfVec3f(2, 0, 0)};

    auto evaluate = [&](float defaultWeight,
                        const VtFloatArray *spatial) -> RigExecRigPose {
        const UsdStageRefPtr stage = UsdStage::CreateInMemory();
        const UsdPrim points =
            stage->DefinePrim(SdfPath("/Asset/Geom/P"), TfToken("Points"));
        points.CreateAttribute(TfToken("points"),
                               SdfValueTypeNames->Point3fArray, false)
            .Set(base);
        stage->DefinePrim(SdfPath("/Asset/Rig"), TfToken("RigExecRoot"));
        const UsdPrim joint = stage->DefinePrim(
            SdfPath("/Asset/Rig/Joints/J"), TfToken("RigExecJoint"));
        GfMatrix4d posed(1.0);
        posed.SetTranslate(GfVec3d(0, 10, 0));
        joint.CreateAttribute(TfToken("posed:space"),
                              SdfValueTypeNames->Matrix4d, false)
            .Set(posed);

        const UsdPrim mover = stage->DefinePrim(
            SdfPath("/Asset/Rig/Movers/M"), TfToken("RigExecMatrixMover"));
        CHECK(mover.ApplyAPI(TfToken("RigExecMoverAPI")));
        mover.CreateRelationship(TfToken("rigExec:moves"), false)
            .SetTargets({target});
        mover.CreateRelationship(TfToken("rigExec:transform"), false)
            .SetTargets({joint.GetPath()});
        mover.CreateAttribute(TfToken("inputs:defaultWeight"),
                              SdfValueTypeNames->Float, false)
            .Set(defaultWeight);
        if (spatial) {
            const UsdPrim weight = MakeEnvelopeWeight(
                stage, SdfPath("/Asset/Rig/Weights/W"), target, *spatial);
            mover.CreateRelationship(TfToken("rigExec:weightObject"), false)
                .SetTargets({weight.GetPath()});
        }
        return EvaluateEnvelopeFixture(stage);
    };

    for (const float weight : {0.0f, 0.5f, 1.0f}) {
        // No rigExec:weightObject is authored in this arm of the test.
        const RigExecRigPose pose = evaluate(weight, nullptr);
        const VtVec3fArray moved = EnvelopePoints(pose, target);
        CHECK(moved.size() == base.size());
        for (size_t i = 0; i < moved.size() && i < base.size(); ++i) {
            CHECK(Near(GfVec3d(moved[i]),
                       GfVec3d(base[i]) + GfVec3d(0, 10.0 * weight, 0)));
        }
    }

    const VtFloatArray spatial{0.0f, 0.5f, 1.0f};
    const RigExecRigPose fieldPose = evaluate(0.25f, &spatial);
    const VtVec3fArray fieldMoved = EnvelopePoints(fieldPose, target);
    CHECK(fieldMoved.size() == base.size());
    for (size_t i = 0; i < fieldMoved.size() && i < base.size(); ++i) {
        CHECK(Near(GfVec3d(fieldMoved[i]),
                   GfVec3d(base[i]) + GfVec3d(0, 10.0 * spatial[i], 0)));
    }

    // The common envelope is normalized.  Bad scalar values fail this one
    // application atomically and preserve the preceding value.
    for (const float invalid : {
             -0.01f, 1.01f, std::numeric_limits<float>::quiet_NaN()}) {
        const RigExecRigPose pose = evaluate(invalid, nullptr);
        const VtVec3fArray moved = EnvelopePoints(pose, target);
        CHECK(moved == base);
        CHECK(std::any_of(
            pose.diagnostics.begin(), pose.diagnostics.end(),
            [](const std::string &diagnostic) {
                return diagnostic.find("defaultWeight") != std::string::npos ||
                       diagnostic.find("weight") != std::string::npos;
            }));
    }
}

// RigExecSkinMover end to end: compiled, tapped, assembled, executed by the
// graph and checked against the independent CPU oracle (cpuParityMode).
// Three points carry three different layouts -- one influence at weight 1,
// two at 0.5 / 0.5, and two at 0.25 / 0.25 -- so the single-influence case
// matches a sequential MatrixMover exactly, the blended case is the analytic
// midpoint, and the under-weighted case documents the rest-retaining rule.
static void
TestSkinMoverLinearBlend()
{
    const SdfPath target("/Asset/Geom/P.points");
    const VtVec3fArray base = {
        GfVec3f(0, 0, 0), GfVec3f(1, 0, 0), GfVec3f(2, 0, 0)};

    auto makeStage = [&]() {
        const UsdStageRefPtr stage = UsdStage::CreateInMemory();
        const UsdPrim points =
            stage->DefinePrim(SdfPath("/Asset/Geom/P"), TfToken("Points"));
        points.CreateAttribute(TfToken("points"),
                               SdfValueTypeNames->Point3fArray, false)
            .Set(base);
        stage->DefinePrim(SdfPath("/Asset/Rig"), TfToken("RigExecRoot"));
        GfMatrix4d posedA(1.0), posedB(1.0);
        posedA.SetTranslate(GfVec3d(0, 10, 0));
        posedB.SetTranslate(GfVec3d(4, 0, 0));
        stage->DefinePrim(SdfPath("/Asset/Rig/Joints/A"), TfToken("RigExecJoint"))
            .CreateAttribute(TfToken("posed:space"),
                             SdfValueTypeNames->Matrix4d, false)
            .Set(posedA);
        stage->DefinePrim(SdfPath("/Asset/Rig/Joints/B"), TfToken("RigExecJoint"))
            .CreateAttribute(TfToken("posed:space"),
                             SdfValueTypeNames->Matrix4d, false)
            .Set(posedB);
        return stage;
    };
    auto addSkin = [&](const UsdStageRefPtr &stage, const VtIntArray &indices,
                       const VtFloatArray &weights, int elementSize) {
        const UsdPrim mover = stage->DefinePrim(
            SdfPath("/Asset/Rig/Movers/Skin"), TfToken("RigExecSkinMover"));
        CHECK(mover.ApplyAPI(TfToken("RigExecMoverAPI")));
        mover.CreateRelationship(TfToken("rigExec:moves"), false)
            .SetTargets({target});
        mover.CreateRelationship(TfToken("rigExec:influences"), false)
            .SetTargets({SdfPath("/Asset/Rig/Joints/A"),
                         SdfPath("/Asset/Rig/Joints/B")});
        mover.CreateAttribute(TfToken("rigExec:elementSize"),
                              SdfValueTypeNames->Int, false)
            .Set(elementSize);
        mover.CreateAttribute(TfToken("rigExec:jointIndices"),
                              SdfValueTypeNames->IntArray, false)
            .Set(indices);
        mover.CreateAttribute(TfToken("rigExec:jointWeights"),
                              SdfValueTypeNames->FloatArray, false)
            .Set(weights);
        return mover;
    };

    {
        const UsdStageRefPtr stage = makeStage();
        addSkin(stage, VtIntArray({0, 1, 0, 1, 0, 1}),
                VtFloatArray({1.0f, 0.0f, 0.5f, 0.5f, 0.25f, 0.25f}), 2);
        const RigExecRigPose pose = EvaluateEnvelopeFixture(stage);
        const VtVec3fArray moved = EnvelopePoints(pose, target);
        CHECK(moved.size() == base.size());
        if (moved.size() == base.size()) {
            // Weight 1 on A alone: exactly A's translation.
            CHECK(Near(GfVec3d(moved[0]), GfVec3d(base[0]) + GfVec3d(0, 10, 0)));
            // 0.5 / 0.5: the midpoint of A p and B p.
            CHECK(Near(GfVec3d(moved[1]), GfVec3d(base[1]) + GfVec3d(2, 5, 0)));
            // 0.25 / 0.25: half the rest point is retained.
            CHECK(Near(GfVec3d(moved[2]), GfVec3d(base[2]) + GfVec3d(1, 2.5, 0)));
        }
        CHECK(pose.moverGraphParityMismatches == 0);
    }

    // The single-influence case is the sequential MatrixMover's result.
    {
        const UsdStageRefPtr stage = makeStage();
        addSkin(stage, VtIntArray({0, 0, 0}), VtFloatArray({1, 1, 1}), 1);
        const VtVec3fArray skinned =
            EnvelopePoints(EvaluateEnvelopeFixture(stage), target);

        const UsdStageRefPtr sequential = makeStage();
        const UsdPrim mover = sequential->DefinePrim(
            SdfPath("/Asset/Rig/Movers/M"), TfToken("RigExecMatrixMover"));
        CHECK(mover.ApplyAPI(TfToken("RigExecMoverAPI")));
        mover.CreateRelationship(TfToken("rigExec:moves"), false)
            .SetTargets({target});
        mover.CreateRelationship(TfToken("rigExec:transform"), false)
            .SetTargets({SdfPath("/Asset/Rig/Joints/A")});
        const VtVec3fArray moved =
            EnvelopePoints(EvaluateEnvelopeFixture(sequential), target);
        CHECK(skinned.size() == moved.size());
        for (size_t i = 0; i < skinned.size() && i < moved.size(); ++i) {
            CHECK(Near(GfVec3d(skinned[i]), GfVec3d(moved[i])));
        }
    }

    // Compile is strict about the layout and the method: a token neither
    // kernel owns is refused with a diagnostic rather than silently falling
    // back to different maths, and a mis-sized layout is caught before
    // evaluation.
    auto compileError = [&](const UsdStageRefPtr &stage) {
        RigExecRigEvaluator evaluator(stage, SdfPath("/Asset/Rig"));
        std::vector<std::string> errors;
        CHECK(SkipsOperations(evaluator, {SdfPath("/Asset/Rig/Movers/Skin")}, &errors));
        CHECK(evaluator.Evaluate(UsdTimeCode::Default()).valid);
        std::string joined;
        for (const std::string &e : errors) joined += e + "\n";
        return joined;
    };
    {
        const UsdStageRefPtr stage = makeStage();
        const UsdPrim mover = addSkin(
            stage, VtIntArray({0, 0, 0}), VtFloatArray({1, 1, 1}), 1);
        mover.CreateAttribute(TfToken("rigExec:skinningMethod"),
                              SdfValueTypeNames->Token, false)
            .Set(TfToken("bogus"));
        CHECK(compileError(stage).find("unknown rigExec:skinningMethod") !=
              std::string::npos);
    }
    {
        const UsdStageRefPtr stage = makeStage();
        addSkin(stage, VtIntArray({0, 0}), VtFloatArray({1, 1}), 1);
        CHECK(compileError(stage).find("jointIndices length") !=
              std::string::npos);
    }
    {
        const UsdStageRefPtr stage = makeStage();
        addSkin(stage, VtIntArray({0, 0, 2}), VtFloatArray({1, 1, 1}), 1);
        CHECK(compileError(stage).find("outside the 2 influences") !=
              std::string::npos);
    }
}

// RigExecSkinMover with rigExec:skinningMethod = dualQuaternion, end to end
// through compile, assembly, the graph kernel and the CPU oracle (parity
// must hold on every stage, which exercises the evaluator's independent
// GfDualQuatd reference alongside the kernel). Every case also runs the
// classicLinear method on the same stage, so the linear expectations are
// re-asserted next to the dual-quaternion ones.
static void
TestSkinMoverDualQuaternion()
{
    const SdfPath target("/Asset/Geom/P.points");
    static constexpr double kPi = 3.141592653589793238462643383279502884;
    const GfVec3d X(1, 0, 0), Y(0, 1, 0), Z(0, 0, 1);

    auto rigid = [](const GfVec3d &axis, double degrees, const GfVec3d &t) {
        GfMatrix4d m(1.0);
        m.SetRotate(GfRotation(axis, degrees));
        m.SetTranslateOnly(t);
        return m;
    };
    // Row-vector [S | 0] * [R | t]: the stretch acts in the joint's
    // pre-rotation frame, which is the transform a scaled joint produces.
    auto scaled = [&](const GfVec3d &scale, const GfVec3d &axis,
                      double degrees, const GfVec3d &t) {
        return GfMatrix4d().SetScale(scale) * rigid(axis, degrees, t);
    };

    // Joints A and B at the given posed matrices and a skin mover over
    // `base` with the given layout, evaluated with the given method.
    auto skin = [&](const VtVec3fArray &base, const GfMatrix4d &posedA,
                    const GfMatrix4d &posedB, const VtIntArray &indices,
                    const VtFloatArray &weights, int elementSize,
                    const char *method) {
        const UsdStageRefPtr stage = UsdStage::CreateInMemory();
        const UsdPrim points =
            stage->DefinePrim(SdfPath("/Asset/Geom/P"), TfToken("Points"));
        points.CreateAttribute(TfToken("points"),
                               SdfValueTypeNames->Point3fArray, false)
            .Set(base);
        stage->DefinePrim(SdfPath("/Asset/Rig"), TfToken("RigExecRoot"));
        stage->DefinePrim(SdfPath("/Asset/Rig/Joints/A"), TfToken("RigExecJoint"))
            .CreateAttribute(TfToken("posed:space"),
                             SdfValueTypeNames->Matrix4d, false)
            .Set(posedA);
        stage->DefinePrim(SdfPath("/Asset/Rig/Joints/B"), TfToken("RigExecJoint"))
            .CreateAttribute(TfToken("posed:space"),
                             SdfValueTypeNames->Matrix4d, false)
            .Set(posedB);
        const UsdPrim mover = stage->DefinePrim(
            SdfPath("/Asset/Rig/Movers/Skin"), TfToken("RigExecSkinMover"));
        CHECK(mover.ApplyAPI(TfToken("RigExecMoverAPI")));
        mover.CreateRelationship(TfToken("rigExec:moves"), false)
            .SetTargets({target});
        mover.CreateRelationship(TfToken("rigExec:influences"), false)
            .SetTargets({SdfPath("/Asset/Rig/Joints/A"),
                         SdfPath("/Asset/Rig/Joints/B")});
        mover.CreateAttribute(TfToken("rigExec:elementSize"),
                              SdfValueTypeNames->Int, false)
            .Set(elementSize);
        mover.CreateAttribute(TfToken("rigExec:jointIndices"),
                              SdfValueTypeNames->IntArray, false)
            .Set(indices);
        mover.CreateAttribute(TfToken("rigExec:jointWeights"),
                              SdfValueTypeNames->FloatArray, false)
            .Set(weights);
        mover.CreateAttribute(TfToken("rigExec:skinningMethod"),
                              SdfValueTypeNames->Token, false)
            .Set(TfToken(method));
        const RigExecRigPose pose = EvaluateEnvelopeFixture(stage);
        // The oracle ran and agreed: the graph kernel and the independent
        // GfDualQuatd reference in the evaluator match to 1e-4.
        CHECK(pose.moverGraphParityMismatches == 0);
        CHECK(pose.moverGraphParityAgreements >= 1);
        const VtVec3fArray moved = EnvelopePoints(pose, target);
        CHECK(moved.size() == base.size());
        return moved.size() == base.size() ? moved : base;
    };
    auto nearPoint = [](const GfVec3f &a, const GfVec3d &b) {
        return Near(GfVec3d(a), b);
    };
    auto length = [](const GfVec3f &a) { return GfVec3d(a).GetLength(); };

    // Single influence at weight 1: DQS == classicLinear == T p (to float
    // precision, the points being GfVec3f).
    {
        const GfMatrix4d t = rigid(
            GfVec3d(1, 2, 3).GetNormalized(), 40.0, GfVec3d(1, -1, 4));
        const VtVec3fArray base = {
            GfVec3f(0, 0, 0), GfVec3f(1, 0, 0), GfVec3f(0, 1, 0),
            GfVec3f(0.3f, -0.6f, 0.9f)};
        const VtIntArray indices({0, 0, 0, 0});
        const VtFloatArray weights({1, 1, 1, 1});
        const VtVec3fArray dq = skin(
            base, t, GfMatrix4d(1.0), indices, weights, 1, "dualQuaternion");
        const VtVec3fArray lbs = skin(
            base, t, GfMatrix4d(1.0), indices, weights, 1, "classicLinear");
        for (size_t i = 0; i < base.size(); ++i) {
            const GfVec3d expected = t.TransformAffine(GfVec3d(base[i]));
            CHECK(nearPoint(dq[i], expected));
            CHECK(nearPoint(lbs[i], expected));
            CHECK(nearPoint(dq[i], GfVec3d(lbs[i])));
        }
    }

    // Two pure translations: 0.5 / 0.5 is the analytic midpoint and
    // 0.25 / 0.25 retains half of the rest point, under BOTH methods --
    // translation-only dual-quaternion blending is exact, and the identity
    // influence carrying the complement makes the shortfall rule agree
    // with the linear kernel's too.
    {
        const GfMatrix4d a = rigid(Z, 0.0, GfVec3d(0, 10, 0));
        const GfMatrix4d b = rigid(Z, 0.0, GfVec3d(4, 0, 0));
        const VtVec3fArray base = {
            GfVec3f(0, 0, 0), GfVec3f(1, 0, 0), GfVec3f(2, 0, 0)};
        for (const char *method : {"dualQuaternion", "classicLinear"}) {
            const VtVec3fArray moved = skin(
                base, a, b, VtIntArray({0, 1, 0, 1, 0, 1}),
                VtFloatArray({1.0f, 0.0f, 0.5f, 0.5f, 0.25f, 0.25f}), 2,
                method);
            CHECK(nearPoint(moved[0], GfVec3d(base[0]) + GfVec3d(0, 10, 0)));
            CHECK(nearPoint(moved[1], GfVec3d(base[1]) + GfVec3d(2, 5, 0)));
            CHECK(nearPoint(moved[2], GfVec3d(base[2]) + GfVec3d(1, 2.5, 0)));
        }
    }

    // Candy wrapper: a bone along X with its far joint twisted 150 degrees
    // about X, a ring point (0, 1, 0) weighted 0.5 / 0.5. The
    // dual-quaternion blend rotates it 75 degrees and keeps it on the
    // ring; the linear blend lands on the chord midpoint, cos 75 = 0.26
    // of the radius -- the collapse this method exists to remove.
    {
        const GfMatrix4d a = rigid(X, 0.0, GfVec3d(0));
        const GfMatrix4d b = rigid(X, 150.0, GfVec3d(0));
        const VtVec3fArray base = {
            GfVec3f(0, 1, 0), GfVec3f(0, 0, 1), GfVec3f(1, 0, 0)};
        const VtIntArray indices({0, 1, 0, 1, 0, 1});
        const VtFloatArray weights({0.5f, 0.5f, 0.5f, 0.5f, 0.5f, 0.5f});
        const VtVec3fArray dq = skin(
            base, a, b, indices, weights, 2, "dualQuaternion");
        const VtVec3fArray lbs = skin(
            base, a, b, indices, weights, 2, "classicLinear");
        const double c75 = std::cos(75.0 * kPi / 180.0);
        const double s75 = std::sin(75.0 * kPi / 180.0);
        const double c150 = std::cos(150.0 * kPi / 180.0);
        const double s150 = std::sin(150.0 * kPi / 180.0);
        CHECK(nearPoint(dq[0], GfVec3d(0, c75, s75)));
        CHECK(nearPoint(dq[1], GfVec3d(0, -s75, c75)));
        CHECK(nearPoint(dq[2], GfVec3d(1, 0, 0)));
        CHECK(std::abs(length(dq[0]) - 1.0) < 1e-6);
        CHECK(std::abs(length(dq[1]) - 1.0) < 1e-6);
        CHECK(nearPoint(lbs[0], GfVec3d(0, 0.5 * (1.0 + c150), 0.5 * s150)));
        CHECK(length(lbs[0]) < 0.3);
        CHECK(length(lbs[1]) < 0.3);
    }

    // Non-uniform scale survives: joint A carries the spine's squash
    // pattern, s_x = 1, s_y = s_z = 2, with no rotation. Alone at weight 1
    // it stretches the point; at 0.5 / 0.5 with a joint twisted 150
    // degrees about X the stretch blends to diag(1, 1.5, 1.5) in the
    // pre-rotation frame and the rotation to 75 degrees, so the ring point
    // comes out at radius 1.5, exactly the mean scale. A rigid-only DQS
    // would have silently returned radius 1.
    {
        const GfMatrix4d a = scaled(GfVec3d(1, 2, 2), X, 0.0, GfVec3d(0));
        const GfMatrix4d b = rigid(X, 150.0, GfVec3d(0));
        const VtVec3fArray base = {
            GfVec3f(0, 1, 0), GfVec3f(0, 0, 1), GfVec3f(1, 0, 0),
            GfVec3f(0, 1, 0)};
        const VtIntArray indices({0, 0, 0, 0, 0, 0, 0, 1});
        const VtFloatArray weights({1, 0, 1, 0, 1, 0, 0.5f, 0.5f});
        const VtVec3fArray dq = skin(
            base, a, b, indices, weights, 2, "dualQuaternion");
        const VtVec3fArray lbs = skin(
            base, a, b, indices, weights, 2, "classicLinear");
        const double c75 = std::cos(75.0 * kPi / 180.0);
        const double s75 = std::sin(75.0 * kPi / 180.0);
        CHECK(nearPoint(dq[0], GfVec3d(0, 2, 0)));
        CHECK(nearPoint(dq[1], GfVec3d(0, 0, 2)));
        CHECK(nearPoint(dq[2], GfVec3d(1, 0, 0)));
        CHECK(nearPoint(dq[3], GfVec3d(0, 1.5 * c75, 1.5 * s75)));
        CHECK(std::abs(length(dq[3]) - 1.5) < 1e-6);
        // The linear kernel keeps the scale on the lone influence too, and
        // collapses the blended one as before.
        CHECK(nearPoint(lbs[0], GfVec3d(0, 2, 0)));
        CHECK(length(lbs[3]) < 0.7);
    }

    // Weight shortfall with rotation: a lone 90 degree influence at weight
    // 0.5. The complement enters the blend as the identity, so the
    // dual-quaternion result is the 45 degree point ON the arc (length 1)
    // where the linear rule lands on the chord midpoint (length 0.71).
    // All-zero weights leave the point exactly where it was under both.
    {
        const GfMatrix4d a = rigid(Z, 90.0, GfVec3d(0));
        const VtVec3fArray base = {GfVec3f(1, 0, 0), GfVec3f(1, 0, 0)};
        const VtIntArray indices({0, 0});
        const VtFloatArray weights({0.5f, 0.0f});
        const VtVec3fArray dq = skin(
            base, a, GfMatrix4d(1.0), indices, weights, 1, "dualQuaternion");
        const VtVec3fArray lbs = skin(
            base, a, GfMatrix4d(1.0), indices, weights, 1, "classicLinear");
        const double c45 = std::cos(45.0 * kPi / 180.0);
        CHECK(nearPoint(dq[0], GfVec3d(c45, c45, 0)));
        CHECK(std::abs(length(dq[0]) - 1.0) < 1e-6);
        CHECK(nearPoint(lbs[0], GfVec3d(0.5, 0.5, 0)));
        CHECK(dq[1] == base[1]);
        CHECK(lbs[1] == base[1]);
    }
}

// BlendShape already had an optional spatial mask.  This pins the migration:
// the mask remains spatial, BlendInput.inputs:weight remains the channel
// amplitude, and the mover-wide scalar is now only inputs:defaultWeight.
static void
TestBlendShapeUniversalEnvelope()
{
    const SdfPath target("/Asset/Geom/P.points");
    const VtVec3fArray base = {
        GfVec3f(0, 0, 0), GfVec3f(1, 0, 0), GfVec3f(2, 0, 0)};

    auto evaluate = [&](float defaultWeight,
                        const VtFloatArray *spatial) -> VtVec3fArray {
        const UsdStageRefPtr stage = UsdStage::CreateInMemory();
        const UsdPrim points =
            stage->DefinePrim(SdfPath("/Asset/Geom/P"), TfToken("Points"));
        points.CreateAttribute(TfToken("points"),
                               SdfValueTypeNames->Point3fArray, false)
            .Set(base);
        VtVec3fArray full = base;
        for (GfVec3f &point : full) {
            point += GfVec3f(0, 0, 4);
        }
        const UsdPrim targetShape = stage->DefinePrim(
            SdfPath("/Asset/Targets/Full"), TfToken("Points"));
        targetShape.CreateAttribute(TfToken("points"),
                                    SdfValueTypeNames->Point3fArray, false)
            .Set(full);

        stage->DefinePrim(SdfPath("/Asset/Rig"), TfToken("RigExecRoot"));
        const UsdPrim input = stage->DefinePrim(
            SdfPath("/Asset/Rig/BlendInputs/B"), TfToken("RigExecBlendInput"));
        // This weight survives the migration: it is a blend-channel
        // amplitude, not a mover envelope.
        input.CreateAttribute(TfToken("inputs:weight"),
                              SdfValueTypeNames->Float, false)
            .Set(1.0f);
        const UsdPrim sample = stage->DefinePrim(
            SdfPath("/Asset/Rig/BlendInputs/B/Full"),
            TfToken("RigExecBlendSample"));
        sample.CreateAttribute(TfToken("rigExec:activation"),
                               SdfValueTypeNames->Float, false)
            .Set(1.0f);
        sample.CreateRelationship(TfToken("rigExec:targetPoints"), false)
            .SetTargets({targetShape.GetPath().AppendProperty(
                TfToken("points"))});
        input.CreateRelationship(TfToken("rigExec:samples"), false)
            .SetTargets({sample.GetPath()});

        const UsdPrim mover = stage->DefinePrim(
            SdfPath("/Asset/Rig/Movers/B"),
            TfToken("RigExecBlendShapeMover"));
        CHECK(mover.ApplyAPI(TfToken("RigExecMoverAPI")));
        mover.CreateRelationship(TfToken("rigExec:moves"), false)
            .SetTargets({target});
        mover.CreateRelationship(TfToken("rigExec:blendInputs"), false)
            .SetTargets({input.GetPath()});
        mover.CreateAttribute(TfToken("inputs:defaultWeight"),
                              SdfValueTypeNames->Float, false)
            .Set(defaultWeight);
        if (spatial) {
            const UsdPrim weight = MakeEnvelopeWeight(
                stage, SdfPath("/Asset/Rig/Weights/W"), target, *spatial);
            mover.CreateRelationship(TfToken("rigExec:weightObject"), false)
                .SetTargets({weight.GetPath()});
        }
        return EnvelopePoints(EvaluateEnvelopeFixture(stage), target);
    };

    for (const float weight : {0.0f, 0.5f, 1.0f}) {
        const VtVec3fArray moved = evaluate(weight, nullptr);
        CHECK(moved.size() == base.size());
        for (size_t i = 0; i < moved.size() && i < base.size(); ++i) {
            CHECK(Near(GfVec3d(moved[i]),
                       GfVec3d(base[i]) + GfVec3d(0, 0, 4.0 * weight)));
        }
    }

    const VtFloatArray spatial{0.0f, 0.5f, 1.0f};
    const VtVec3fArray fieldMoved = evaluate(0.25f, &spatial);
    CHECK(fieldMoved.size() == base.size());
    for (size_t i = 0; i < fieldMoved.size() && i < base.size(); ++i) {
        CHECK(Near(GfVec3d(fieldMoved[i]),
                   GfVec3d(base[i]) + GfVec3d(0, 0, 4.0 * spatial[i])));
    }
}

// A mover that formerly called its envelope inputs:strength now uses exactly
// the same scalar/field contract as Matrix and BlendShape.
static void
TestSmoothMoverUniversalEnvelope()
{
    const SdfPath target("/Asset/Geom/M.points");
    const VtVec3fArray base = {
        GfVec3f(0, 0, 0), GfVec3f(2, 0, 0),
        GfVec3f(2, 2, 0), GfVec3f(0, 2, 0)};
    const GfVec3d center(1, 1, 0);

    auto makeStage = [&](float defaultWeight,
                         const VtFloatArray *spatial) {
        const UsdStageRefPtr stage = UsdStage::CreateInMemory();
        const UsdPrim mesh =
            stage->DefinePrim(SdfPath("/Asset/Geom/M"), TfToken("Mesh"));
        mesh.CreateAttribute(TfToken("points"),
                             SdfValueTypeNames->Point3fArray, false).Set(base);
        mesh.CreateAttribute(TfToken("faceVertexCounts"),
                             SdfValueTypeNames->IntArray, false)
            .Set(VtIntArray{4});
        mesh.CreateAttribute(TfToken("faceVertexIndices"),
                             SdfValueTypeNames->IntArray, false)
            .Set(VtIntArray{0, 1, 2, 3});
        stage->DefinePrim(SdfPath("/Asset/Rig"), TfToken("RigExecRoot"));
        const UsdPrim mover = stage->DefinePrim(
            SdfPath("/Asset/Rig/Movers/S"), TfToken("RigExecSmoothMover"));
        CHECK(mover.ApplyAPI(TfToken("RigExecMoverAPI")));
        mover.CreateRelationship(TfToken("rigExec:moves"), false)
            .SetTargets({target});
        mover.CreateAttribute(TfToken("inputs:defaultWeight"),
                              SdfValueTypeNames->Float, false)
            .Set(defaultWeight);
        if (spatial) {
            const UsdPrim weight = MakeEnvelopeWeight(
                stage, SdfPath("/Asset/Rig/Weights/W"), target, *spatial);
            mover.CreateRelationship(TfToken("rigExec:weightObject"), false)
                .SetTargets({weight.GetPath()});
        }
        return stage;
    };

    for (const float weight : {0.0f, 0.5f, 1.0f}) {
        const VtVec3fArray moved = EnvelopePoints(
            EvaluateEnvelopeFixture(makeStage(weight, nullptr)), target);
        CHECK(moved.size() == base.size());
        for (size_t i = 0; i < moved.size() && i < base.size(); ++i) {
            const GfVec3d expected =
                GfVec3d(base[i]) + (center - GfVec3d(base[i])) * weight;
            CHECK(Near(GfVec3d(moved[i]), expected));
        }
    }

    const VtFloatArray spatial{0.0f, 0.5f, 1.0f, 0.25f};
    const VtVec3fArray fieldMoved = EnvelopePoints(
        EvaluateEnvelopeFixture(makeStage(0.75f, &spatial)), target);
    CHECK(fieldMoved.size() == base.size());
    for (size_t i = 0; i < fieldMoved.size() && i < base.size(); ++i) {
        const GfVec3d expected =
            GfVec3d(base[i]) + (center - GfVec3d(base[i])) * spatial[i];
        CHECK(Near(GfVec3d(fieldMoved[i]), expected));
    }

    // The old type-specific spelling must fail closed rather than compose as
    // an ignored custom attribute.
    const UsdStageRefPtr legacy = makeStage(1.0f, nullptr);
    legacy->GetPrimAtPath(SdfPath("/Asset/Rig/Movers/S"))
        .CreateAttribute(TfToken("inputs:strength"),
                         SdfValueTypeNames->Float, true)
        .Set(0.5f);
    RigExecRigEvaluator legacyEvaluator(legacy, SdfPath("/Asset/Rig"));
    legacyEvaluator.cpuParityMode = true;
    std::vector<std::string> errors;
    CHECK(!legacyEvaluator.Compile(&errors));
    CHECK(std::any_of(errors.begin(), errors.end(), [](const std::string &e) {
        return e.find("inputs:strength") != std::string::npos &&
               e.find("inputs:defaultWeight") != std::string::npos;
    }));
}

// The scalar property domain uses the same contract.  A one-element constant
// weight object is meaningful here and supersedes the fallback just as a
// dense field does for points.
static void
TestPropertyMoverUniversalEnvelope()
{
    const SdfPath target("/Asset/Channels.value");
    auto makeStage = [&](float defaultWeight, const float *objectWeight) {
        const UsdStageRefPtr stage = UsdStage::CreateInMemory();
        const UsdPrim channels =
            stage->DefinePrim(SdfPath("/Asset/Channels"), TfToken("Scope"));
        channels.CreateAttribute(TfToken("value"), SdfValueTypeNames->Float,
                                 true).Set(2.0f);
        stage->DefinePrim(SdfPath("/Asset/Rig"), TfToken("RigExecRoot"));
        const UsdPrim mover = stage->DefinePrim(
            SdfPath("/Asset/Rig/Movers/Add"),
            TfToken("RigExecFloatMathMover"));
        CHECK(mover.ApplyAPI(TfToken("RigExecMoverAPI")));
        mover.CreateRelationship(TfToken("rigExec:moves"), false)
            .SetTargets({target});
        mover.CreateAttribute(TfToken("rigExec:operation"),
                              SdfValueTypeNames->Token, false)
            .Set(TfToken("add"));
        mover.CreateAttribute(TfToken("inputs:value"),
                              SdfValueTypeNames->Float, false)
            .Set(8.0f);
        mover.CreateAttribute(TfToken("inputs:defaultWeight"),
                              SdfValueTypeNames->Float, false)
            .Set(defaultWeight);
        if (objectWeight) {
            const UsdPrim weight = MakeEnvelopeWeight(
                stage, SdfPath("/Asset/Rig/Weights/W"), target,
                VtFloatArray(), *objectWeight);
            mover.CreateRelationship(TfToken("rigExec:weightObject"), false)
                .SetTargets({weight.GetPath()});
        }
        return stage;
    };

    auto valueAt = [&](const UsdStageRefPtr &stage) {
        const RigExecRigPose pose = EvaluateEnvelopeFixture(stage);
        const auto it = pose.movedProperties.find(target);
        CHECK(it != pose.movedProperties.end());
        if (it == pose.movedProperties.end()) {
            return 0.0f;
        }
        CHECK(it->second.IsHolding<float>());
        return it->second.IsHolding<float>()
            ? it->second.UncheckedGet<float>() : 0.0f;
    };

    for (const float weight : {0.0f, 0.5f, 1.0f}) {
        // Full operation is 2 + 8 = 10; the common envelope mixes from 2.
        CHECK(std::abs(valueAt(makeStage(weight, nullptr)) -
                       (2.0f + 8.0f * weight)) < 1e-6f);
    }
    const float field = 0.75f;
    CHECK(std::abs(valueAt(makeStage(0.25f, &field)) - 8.0f) < 1e-6f);

    // inputs:weight used to be the property-mover envelope.  It is not an
    // alias: only BlendInput keeps that spelling for channel amplitude.
    const UsdStageRefPtr legacy = makeStage(1.0f, nullptr);
    legacy->GetPrimAtPath(SdfPath("/Asset/Rig/Movers/Add"))
        .CreateAttribute(TfToken("inputs:weight"),
                         SdfValueTypeNames->Float, true)
        .Set(0.5f);
    RigExecRigEvaluator legacyEvaluator(legacy, SdfPath("/Asset/Rig"));
    legacyEvaluator.cpuParityMode = true;
    std::vector<std::string> errors;
    CHECK(!legacyEvaluator.Compile(&errors));
    CHECK(std::any_of(errors.begin(), errors.end(), [](const std::string &e) {
        return e.find("inputs:weight") != std::string::npos &&
               e.find("inputs:defaultWeight") != std::string::npos;
    }));
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

    evaluator.cpuParityMode = true;
    std::vector<std::string> errors;
    CHECK(SkipsOperations(evaluator, {SdfPath("/ArmAsset/Rig/Movers/Geometry/VolumeCorrect/RibbonWrap/ShoulderMatrix")}, &errors));
    CHECK(evaluator.Evaluate(UsdTimeCode::Default()).valid);
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
    CHECK(mover.ApplyAPI(TfToken("RigExecMoverAPI")));
    mover.CreateRelationship(TfToken("rigExec:moves"))
        .SetTargets({SdfPath("/ArmAsset/Geom/ArmBody.normals")});
    {
        RigExecRigEvaluator evaluator(stage, SdfPath("/ArmAsset/Rig"));
        evaluator.cpuParityMode = true;
        std::vector<std::string> errors;
        CHECK(SkipsOperations(evaluator, {SdfPath("/ArmAsset/Rig/Movers/Geometry/BadSmooth")}, &errors));
        CHECK(!errors.empty());
    }
    mover.GetRelationship(TfToken("rigExec:moves"))
        .SetTargets({SdfPath("/ArmAsset/Geom/ArmBody.points"),
                     SdfPath("/ArmAsset/Geom/RibbonGuides.points")});
    {
        RigExecRigEvaluator evaluator(stage, SdfPath("/ArmAsset/Rig"));
        evaluator.cpuParityMode = true;
        std::vector<std::string> errors;
        CHECK(SkipsOperations(evaluator, {SdfPath("/ArmAsset/Rig/Movers/Geometry/BadSmooth")}, &errors));
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
        evaluator.cpuParityMode = true;
        std::vector<std::string> errors;
        CHECK(SkipsOperations(evaluator, {SdfPath("/ArmAsset/Rig/Movers/Geometry/BadSmooth")}, &errors));
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
    evaluator.cpuParityMode = true;
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
        evaluator.cpuParityMode = true;
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
        evaluator.cpuParityMode = true;
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
        evaluator.cpuParityMode = true;
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
// from one that passed (see docs/specs/mover-graph-cutover.md).
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
        {"/16_ConnectionReadPhases.usda", 1024, true},
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
        evaluator.cpuParityMode = true;
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
// 05 is already SpineFK -> SpineTwist (SpineTwist reads Root/Chest, which
// SpineFK poses); pointing SpineFK's controls at TwistMid, which SpineTwist
// poses, closes the loop.
// A frame read from BELOW its writer is positional under the unified pose
// stack and orders nothing (spec 4.2) -- but only for a reader that HAS a
// position. Both solvers here feed SpineRibbon, so both are PRODUCERS: they
// carry no stack position, they are scheduled by data flow alone, and data
// flow is what contradicts itself. Evaluate resolves overrides by iterating
// to a fixed point and a cycle has none, so this must fail in Compile rather
// than surface as a non-converging generation.
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
        ok.cpuParityMode = true;
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

    evaluator.cpuParityMode = true;
    std::vector<std::string> errors;
    // A cycle is no one member's fault: every member is set aside.
    CHECK(SkipsOperations(
        evaluator, {SdfPath("/SpineAsset/Rig/Solvers/SpineFK"),
                    SdfPath("/SpineAsset/Rig/Solvers/SpineRibbon"),
                    SdfPath("/SpineAsset/Rig/Solvers/SpineTwist")},
        &errors));
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
        ok.cpuParityMode = true;
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

    evaluator.cpuParityMode = true;
    std::vector<std::string> errors;
    // A cycle is no one member's fault: every member is set aside.
    CHECK(SkipsOperations(
        evaluator, {SdfPath("/BlendArmAsset/Rig/Solvers/ArmFK"),
                    SdfPath("/BlendArmAsset/Rig/Solvers/IKFKBlend")},
        &errors));
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
// Nothing asserted this before: the mover-graph parity suite only covers point
// chains, and an aim moves a transform provider's FRAME. So the whole
// pose-domain revision path could have been a no-op and every test would still
// have passed -- which matters now that it is applied in memory rather than
// through a generated RigExecPointFrameMoverApplication.
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
    evaluator.cpuParityMode = true;
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

// A TwoBoneIk that leaves its bone lengths unauthored measures them from the
// bound joints' rest frames on EVERY Evaluate, so moving a joint's rest
// recalibrates the solve with no recompile (rigEvaluator.h:298-304).
// The shipped examples are the regression that matters. Authoring an absolute
// length -- as a measure-then-bake repair session once did to spider_leg_ik --
// opts the solver out of that measurement entirely, and the pose silently
// stops answering to the skeleton it is supposed to follow. The symptom is
// not a wrong number anywhere; it is an edit that does nothing at all.
// Every TwoBoneIk names the three joints it solves for, so every rig can
// be checked. 03_IkFkBlendClamp and ArmRig route POSING through a
// RigExecBlendPointFrames, but their IKs still name the same chain as a
// rest reference, which is where their bone lengths come from.
static void
_CheckRestEditRecalibrates(const std::string &stagePath, const char *rigPath,
                           const char *editJoint, const char *restChannel,
                           const char *watchJoint)
{
    UsdStageRefPtr stage = UsdStage::Open(stagePath);
    CHECK(stage);
    if (!stage) {
        return;
    }
    RigExecRigEvaluator evaluator(stage, SdfPath(rigPath));
    std::vector<std::string> errors;
    CHECK(evaluator.Compile(&errors));

    const SdfPath watch(watchJoint);
    const RigExecRigPose before = evaluator.Evaluate(UsdTimeCode::Default());
    CHECK(before.valid);
    const auto beforeIt = before.jointFramesFinal.find(watch);
    CHECK(beforeIt != before.jointFramesFinal.end());
    if (beforeIt == before.jointFramesFinal.end()) {
        return;
    }
    const GfVec3d beforeOrigin = beforeIt->second.Origin();

    // Lengthen the lower bone at its rest. The mid joint is the one free to
    // move: the root and the effector are pinned by their own controls.
    const UsdPrim edited = stage->GetPrimAtPath(SdfPath(editJoint));
    CHECK(edited);
    if (!edited) {
        return;
    }
    const UsdAttribute rest = edited.GetAttribute(TfToken(restChannel));
    CHECK(rest);
    if (!rest) {
        return;
    }
    double value = 0.0;
    rest.Get(&value);
    CHECK(rest.Set(value - 3.0));

    const RigExecRigPose after = evaluator.Evaluate(UsdTimeCode::Default());
    CHECK(after.valid);
    const auto afterIt = after.jointFramesFinal.find(watch);
    CHECK(afterIt != after.jointFramesFinal.end());
    if (afterIt == after.jointFramesFinal.end()) {
        return;
    }
    const double moved =
        (afterIt->second.Origin() - beforeOrigin).GetLength();
    if (moved <= 1e-6) {
        std::printf("  %s: %s did not move after a rest edit -- the solver "
                    "is not measuring its joints' rests (does its "
                    "rigExec:joints name the chain?)\n",
                    stagePath.c_str(), watchJoint);
    }
    CHECK(moved > 1e-6);
}

static void
TestShippedTwoBoneIksRecalibrateOnRestEdit(const std::string &examplesDir)
{
    _CheckRestEditRecalibrates(
        examplesDir + "/components/spider_leg_ik.usd", "/RigRoot",
        "/RigRoot/Joints/Shoulder/ankle/foot", "rest:ty",
        "/RigRoot/Joints/Shoulder/ankle");
    _CheckRestEditRecalibrates(
        examplesDir + "/02_TwoBoneIkLeg.usda", "/LegAsset/Rig",
        "/LegAsset/Rig/Joints/Hip/Knee/Ankle", "rest:ty",
        "/LegAsset/Rig/Joints/Hip/Knee");
}

// 09 exercises the three property-domain math movers: the output domain that
// is neither a joint frame nor a points array.
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
    evaluator.cpuParityMode = true;
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
// This is the half that publication alone cannot demonstrate. 03 authors an
// IK/FK blend weight overdriven to -0.25..1.3 and a ClampBlendWeight mover to
// bound it, and RigExecBlendPointFrames ALSO clamps internally -- so a clamp
// that never reached exec and one that did produce identical frames, and the
// authored mover would be decoration.
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
    authoredEval.cpuParityMode = true;
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

    forcedEval.cpuParityMode = true;
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

    evaluator.cpuParityMode = true;
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

    evaluator.cpuParityMode = true;
    std::vector<std::string> errors;
    CHECK(SkipsOperations(
        evaluator, {SdfPath("/PropMathAsset/Rig/Movers/ClampGain")}, &errors));
    CHECK(!errors.empty());
}

// A constraint driving a plain UsdGeomXform publishes a transform, and the
// geometry parented under it is left alone.
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
    evaluator.cpuParityMode = true;
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
// Two rules used to reject this and neither was about the rig being wrong. A
// joint was required because the compile gate read "a rig publishes joints",
// when in fact a mover publishes whatever its target is; and an aim target
// was always tapped for computePointFrame, which a plain Xformable does not
// publish -- a HARD exec failure that took the whole snapshot down rather
// than leaving one value missing.
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
    evaluator.cpuParityMode = true;
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
// The arithmetic is chosen so the three candidate behaviours are three
// different numbers, and no assertion can pass by accident:
//   21  multiply-then-add over the chain   ((2 * 10) + 1)   <- correct
//   30  add-then-multiply over the chain   ((2 + 1) * 10)   <- forward order
//    3  no chaining, last writer wins      (2 + 1)          <- no chain
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

    evaluator.cpuParityMode = true;
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
        CHECK(added.ApplyAPI(TfToken("RigExecMoverAPI")));
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
        evaluator.cpuParityMode = true;
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
// Two delivery routes exist for one revised value, because there are two
// kinds of consumer. A COMPUTATION reads the attribute through exec and gets
// the value override (03's clamped blend weight). Packet assembly reads it
// straight off the stage and never touches exec, so it gets the same value
// through RigExecResolvedInputs instead. Both routes are filled from the one
// property-chain result, before anything reads an input.
// Here a float mover drives the smooth mover's inputs:defaultWeight from 0.6
// to 0,
// and the smoothing has to actually stop. The CPU oracle resolves its own
// inputs independently, so parity is what proves the two routes agree rather
// than both being wrong together.
static void
TestPropertyMoverReachesStaticPacketReads(const std::string &examplesDir)
{
    const SdfPath rigPath("/LatticeAsset/Rig");
    const SdfPath slab("/LatticeAsset/Geom/Slab.points");
    const SdfPath envelope(
        "/LatticeAsset/Rig/Movers/Geometry/VolumeCorrect/Smooth"
        ".inputs:defaultWeight");
    const UsdTimeCode when(1024);

    UsdStageRefPtr stage =
        UsdStage::Open(examplesDir + "/06_LatticeBulge.usda");
    CHECK(stage);
    if (!stage) {
        return;
    }
    RigExecRigEvaluator baseline(stage, rigPath);
    baseline.cpuParityMode = true;
    CHECK(baseline.Compile(nullptr));
    const RigExecRigPose basePose = baseline.Evaluate(when);
    CHECK(basePose.valid);
    const auto baseIt = basePose.movedProperties.find(slab);
    CHECK(baseIt != basePose.movedProperties.end());
    if (baseIt == basePose.movedProperties.end()) {
        return;
    }
    const VtVec3fArray before = baseIt->second.Get<VtVec3fArray>();

    // A property mover that drives the smooth mover's envelope from the
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
    CHECK(mover.ApplyAPI(TfToken("RigExecMoverAPI")));
    mover.CreateAttribute(TfToken("rigExec:operation"),
                          SdfValueTypeNames->Token, /*custom=*/false)
        .Set(TfToken("blend"));
    mover.CreateAttribute(TfToken("inputs:value"), SdfValueTypeNames->Float,
                          /*custom=*/false)
        .Set(0.0f);
    mover.CreateRelationship(TfToken("rigExec:moves"), /*custom=*/false)
        .SetTargets({envelope});

    RigExecRigEvaluator evaluator(edited, rigPath);

    evaluator.cpuParityMode = true;
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
    const auto envelopeIt = pose.movedProperties.find(envelope);
    CHECK(envelopeIt != pose.movedProperties.end());
    if (envelopeIt != pose.movedProperties.end()) {
        CHECK(std::abs(envelopeIt->second.Get<float>()) < 1e-6f);
    }

    // ...and the smoothing CHANGED, because the smooth mover's packet read
    // the revised envelope rather than the authored 0.6.
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
        std::printf("  driving inputs:defaultWeight to 0 did not change the "
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
// 13 deforms a Slab through a cage that is itself deformed by two movers, so
// the three phases are three different slabs from one rig with no other edit:
//   base                 the authored cage -- the slab does not move at all;
//   <CageLift>           the cage as of that mover -- lifted, not twisted;
//   final                the cage after both -- lifted and twisted.
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

        evaluator.cpuParityMode = true;
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

        const size_t epoch = evaluator.GetBindingEpochDigest();
        stage->GetAttributeAtPath(
            SdfPath("/ReadPhaseAsset/Rig/Controls/LiftCtl.avars:tz")).Set(3.0);
        const auto edited = evaluator.Evaluate(when);
        CHECK(edited.valid);
        CHECK(edited.moverGraphRevisionsCreated == 0);
        CHECK(edited.moverGraphSchedulesBuilt == 0);
        CHECK(edited.moverGraphParityMismatches == 0);
        CHECK(evaluator.GetBindingEpochDigest() == epoch);
        const auto after = edited.movedProperties.at(slab).Get<VtVec3fArray>();
        CHECK(after.size() == moved.size());
        const GfVec3f delta = std::string(phase) == "base"
            ? GfVec3f(0) : GfVec3f(0, 0, 3);
        for (size_t i = 0; i < after.size() && i < moved.size(); ++i) {
            CHECK(Near(GfVec3d(after[i] - moved[i]), GfVec3d(delta), 1e-4));
        }
        CHECK(evaluator.Evaluate(when).moverGraphRevisionsExecuted == 0);
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
    evaluator.cpuParityMode = true;
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
        evaluator.cpuParityMode = true;
        // Accepted means the lattice compiled with its phase; a bad phase
        // sets the lattice aside (with a warning) and the rig compiles on.
        return evaluator.Compile(errors) &&
               !evaluator.GetSkippedOperations().count(
                   SdfPath("/ReadPhaseAsset/Rig/Movers/Geometry/SlabLattice"));
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

// Read phases on connections (examples/16_ConnectionReadPhases.usda): one
// dial revised by Gain (x2) then Limit (clamp to 0.6), read undeclared (the
// `base` default), at the Gain checkpoint and at a declared `final` -- by
// float math movers that add what they read to channels of their own, and
// by matrix movers whose envelope is the connection. Dynamic and baked
// alike, and under a drag on Gain.
static void
TestConnectionReadPhasesSelectRevision(const std::string &examplesDir)
{
    const auto readout = [](const RigExecRigPose &pose, const char *name) {
        const auto it = pose.movedProperties.find(SdfPath(
            std::string("/PhaseConnectAsset/Rig/Channels/Readouts.rigExec:") +
            name));
        return it != pose.movedProperties.end() &&
                       it->second.IsHolding<float>()
                   ? double(it->second.UncheckedGet<float>())
                   : -1.0;
    };
    // How far a card rose: the lift (3 at frame 1024) times the dial value
    // its phase selects.
    const auto rise = [](const RigExecRigPose &pose, const char *card) {
        const auto it = pose.movedProperties.find(SdfPath(
            std::string("/PhaseConnectAsset/Geom/") + card + ".points"));
        if (it == pose.movedProperties.end() ||
            !it->second.IsHolding<VtVec3fArray>() ||
            it->second.UncheckedGet<VtVec3fArray>().empty()) {
            return -1.0;
        }
        return double(it->second.UncheckedGet<VtVec3fArray>()[0][1]) + 0.5;
    };
    for (const RigExecEvaluationMode mode :
         {RigExecEvaluationMode::Dynamic, RigExecEvaluationMode::Baked}) {
        UsdStageRefPtr stage =
            UsdStage::Open(examplesDir + "/16_ConnectionReadPhases.usda");
        CHECK(stage);
        if (!stage) {
            return;
        }
        RigExecRigEvaluator evaluator(stage, SdfPath("/PhaseConnectAsset/Rig"));
        evaluator.cpuParityMode = true;
        evaluator.SetEvaluationMode(mode);
        std::vector<std::string> errors;
        CHECK(evaluator.Compile(&errors));
        CHECK(evaluator.GetSkippedOperations().empty());
        // The Base and Gain readouts and cards. The Final ones declare
        // `final` and pass no recorded hop, so the dial's published value
        // answers them and they need no record.
        CHECK(evaluator.GetPhasedConnections().size() == 4);

        const RigExecRigPose pose = evaluator.Evaluate(UsdTimeCode(1024));
        CHECK(pose.valid);
        CHECK(std::abs(readout(pose, "base") - 0.45) < 1e-6);
        CHECK(std::abs(readout(pose, "gain") - 0.9) < 1e-6);
        CHECK(std::abs(readout(pose, "final") - 0.6) < 1e-6);
        CHECK(std::abs(rise(pose, "BaseCard") - 0.45 * 3) < 1e-5);
        CHECK(std::abs(rise(pose, "GainCard") - 0.9 * 3) < 1e-5);
        CHECK(std::abs(rise(pose, "FinalCard") - 0.6 * 3) < 1e-5);

        // A drag on Gain's factor moves the checkpoint, and the clamp no
        // longer bites, so final follows it; the base reader stays put.
        evaluator.SetInteractiveOverrides({RigExecValueOverride{
            SdfPath("/PhaseConnectAsset/Rig/Movers/Dial/Gain"), TfToken(),
            TfToken("inputs:value"), VtValue(1.2f)}});
        const RigExecRigPose dragged = evaluator.Evaluate(UsdTimeCode(1024));
        CHECK(dragged.valid);
        CHECK(std::abs(readout(dragged, "base") - 0.45) < 1e-6);
        CHECK(std::abs(readout(dragged, "gain") - 0.54) < 1e-5);
        CHECK(std::abs(readout(dragged, "final") - 0.54) < 1e-5);
        CHECK(std::abs(rise(dragged, "GainCard") - 0.54 * 3) < 1e-4);
        CHECK(std::abs(rise(dragged, "BaseCard") - 0.45 * 3) < 1e-5);
    }
}

// Connection phases that cannot be satisfied set their reader aside with a
// warning that says why, and editing one is structural.
static void
TestBadConnectionReadPhasesRejected(const std::string &examplesDir)
{
    const SdfPath reader("/PhaseConnectAsset/Rig/Movers/Readouts/Base");
    const SdfPath readerInput(
        "/PhaseConnectAsset/Rig/Movers/Readouts/Base.inputs:value");
    const TfToken phaseField("rigExecReadPhase");
    // Opens the example with \p edit applied in the session layer, compiles,
    // and answers whether the reader was set aside with \p expect said.
    const auto setAside = [&](const std::function<void(const UsdStageRefPtr &)>
                                  &edit,
                              const char *expect) {
        UsdStageRefPtr stage =
            UsdStage::Open(examplesDir + "/16_ConnectionReadPhases.usda");
        if (!stage) {
            return false;
        }
        stage->SetEditTarget(stage->GetSessionLayer());
        edit(stage);
        RigExecRigEvaluator evaluator(stage, SdfPath("/PhaseConnectAsset/Rig"));
        evaluator.cpuParityMode = true;
        std::vector<std::string> errors;
        const bool compiled = evaluator.Compile(&errors);
        bool said = false;
        for (const std::string &e : errors) {
            said |= e.find(expect) != std::string::npos;
        }
        if (!said) {
            std::printf("  expected \"%s\" among %zu compile message(s)\n",
                        expect, errors.size());
            for (const std::string &e : errors) {
                std::printf("    %s\n", e.c_str());
            }
        }
        return compiled && evaluator.GetSkippedOperations().count(reader) &&
               said;
    };
    const auto withPhase = [&](const char *phase) {
        return [&, phase](const UsdStageRefPtr &stage) {
            stage->GetAttributeAtPath(readerInput)
                .SetMetadata(phaseField, std::string(phase));
        };
    };

    // `preceding` names a position in the reader's own chain, and the
    // connection reads someone else's.
    CHECK(setAside(withPhase("preceding"), "'preceding' names no position"));
    // A prim beneath which nothing revises the dial.
    CHECK(setAside(withPhase("/PhaseConnectAsset/Rig/Movers/Cards"),
                   "which revises nothing on"));
    // A checkpoint on a connection that reaches no revised property.
    CHECK(setAside(
        [&](const UsdStageRefPtr &stage) {
            const UsdPrim dial = stage->GetPrimAtPath(
                SdfPath("/PhaseConnectAsset/Rig/Channels/Dial"));
            UsdAttribute spare = dial.CreateAttribute(
                TfToken("rigExec:spare"), SdfValueTypeNames->Float);
            spare.Set(0.3f);
            UsdAttribute input = stage->GetAttributeAtPath(readerInput);
            input.SetConnections({spare.GetPath()});
            input.SetMetadata(phaseField,
                              std::string("/PhaseConnectAsset/Rig/Movers/"
                                          "Dial/Gain"));
        },
        "reaches no property a math mover writes"));
    // A phase declared on an input that math movers revise themselves:
    // which value would it choose? (Undeclared, the input reads its own
    // chain's result; TestUndeclaredSkipsCompileCleanly.)
    CHECK(setAside(
        [&](const UsdStageRefPtr &stage) {
            withPhase("base")(stage);
            const UsdPrim nudge = stage->DefinePrim(
                SdfPath("/PhaseConnectAsset/Rig/Movers/Readouts/Nudge"),
                TfToken("RigExecFloatMathMover"));
            nudge.AddAppliedSchema(TfToken("RigExecMoverAPI"));
            nudge.CreateAttribute(TfToken("rigExec:operation"),
                                  SdfValueTypeNames->Token, false,
                                  SdfVariabilityUniform)
                .Set(TfToken("add"));
            nudge.CreateAttribute(TfToken("inputs:value"),
                                  SdfValueTypeNames->Float)
                .Set(0.1f);
            nudge.CreateRelationship(TfToken("rigExec:moves"), false)
                .SetTargets({readerInput});
        },
        "ambiguous"));

    // An unconnected input reads its own value, so a phase there has
    // nothing to choose: accepted, and no record -- even a checkpoint,
    // which a connection reaching no revised property is refused for.
    {
        UsdStageRefPtr stage =
            UsdStage::Open(examplesDir + "/16_ConnectionReadPhases.usda");
        CHECK(stage);
        if (stage) {
            stage->SetEditTarget(stage->GetSessionLayer());
            stage->GetAttributeAtPath(readerInput).SetConnections({});
            withPhase("/PhaseConnectAsset/Rig/Movers/Dial/Gain")(stage);
            RigExecRigEvaluator evaluator(
                stage, SdfPath("/PhaseConnectAsset/Rig"));
            evaluator.cpuParityMode = true;
            CHECK(evaluator.Compile(nullptr));
            CHECK(evaluator.GetSkippedOperations().empty());
            CHECK(evaluator.GetPhasedConnections().size() == 3);
        }
    }

    // Changing a connection's phase is structural: the epoch digest moves,
    // and the next evaluation reads at the new phase.
    {
        UsdStageRefPtr stage =
            UsdStage::Open(examplesDir + "/16_ConnectionReadPhases.usda");
        CHECK(stage);
        if (!stage) {
            return;
        }
        RigExecRigEvaluator evaluator(stage, SdfPath("/PhaseConnectAsset/Rig"));
        evaluator.cpuParityMode = true;
        CHECK(evaluator.Compile(nullptr));
        CHECK(evaluator.Evaluate(UsdTimeCode(1024)).valid);
        const size_t before = evaluator.GetBindingEpochDigest();
        stage->SetEditTarget(stage->GetSessionLayer());
        stage->GetAttributeAtPath(readerInput)
            .SetMetadata(phaseField, std::string("final"));
        const RigExecRigPose pose = evaluator.Evaluate(UsdTimeCode(1024));
        CHECK(pose.valid);
        CHECK(evaluator.GetBindingEpochDigest() != before);
        const auto it = pose.movedProperties.find(SdfPath(
            "/PhaseConnectAsset/Rig/Channels/Readouts.rigExec:base"));
        CHECK(it != pose.movedProperties.end() &&
              it->second.IsHolding<float>() &&
              std::abs(it->second.UncheckedGet<float>() - 0.6f) < 1e-6f);
    }
}

// A phase on a SOLVER's connected input. 03's blend weight is moved onto a
// channel the solver connects to, with the clamp mover turned into a halving
// of that channel: undeclared or at `base`, the solver reads the channel as
// authored, posing exactly as an unrevised weight; at `final` it reads the
// halved weight, posing exactly as a weight authored at half.
static void
TestConnectionReadPhaseOnSolverInput(const std::string &examplesDir)
{
    const SdfPath rig("/BlendArmAsset/Rig");
    const SdfPath weight("/BlendArmAsset/Rig/Solvers/IKFKBlend.inputs:weight");
    const SdfPath wrist("/BlendArmAsset/Rig/Joints/Shoulder/Elbow/Wrist");
    enum class Wiring {
        Authored,
        AuthoredHalf,
        Connected,
        ConnectedBase,
        ConnectedFinal
    };
    const auto wristAt = [&](Wiring wiring, RigExecEvaluationMode mode,
                             GfMatrix4d *out) {
        UsdStageRefPtr stage =
            UsdStage::Open(examplesDir + "/03_IkFkBlendClamp.usda");
        if (!stage) {
            return false;
        }
        stage->SetEditTarget(stage->GetSessionLayer());
        const UsdPrim channel = stage->DefinePrim(
            SdfPath("/BlendArmAsset/Rig/Channels/Blend"), TfToken("Scope"));
        UsdAttribute w = channel.CreateAttribute(TfToken("rigExec:w"),
                                                 SdfValueTypeNames->Float);
        w.Set(0.525f);
        const UsdPrim mover = stage->GetPrimAtPath(
            SdfPath("/BlendArmAsset/Rig/Movers/Pose/ClampBlendWeight"));
        if (!mover) {
            return false;
        }
        mover.GetAttribute(TfToken("rigExec:operation"))
            .Set(TfToken("multiply"));
        mover.CreateAttribute(TfToken("inputs:value"),
                              SdfValueTypeNames->Float)
            .Set(0.5f);
        mover.GetRelationship(TfToken("rigExec:moves"))
            .SetTargets({w.GetPath()});
        UsdAttribute input = stage->GetAttributeAtPath(weight);
        if (wiring == Wiring::Authored) {
            input.Set(0.525f);
        } else if (wiring == Wiring::AuthoredHalf) {
            input.Set(0.525f * 0.5f);
        } else {
            input.SetConnections({w.GetPath()});
        }
        if (wiring == Wiring::ConnectedBase) {
            input.SetMetadata(TfToken("rigExecReadPhase"), std::string("base"));
        } else if (wiring == Wiring::ConnectedFinal) {
            input.SetMetadata(TfToken("rigExecReadPhase"),
                              std::string("final"));
        }
        RigExecRigEvaluator evaluator(stage, rig);
        evaluator.cpuParityMode = true;
        evaluator.SetEvaluationMode(mode);
        if (!evaluator.Compile(nullptr) ||
            !evaluator.GetSkippedOperations().empty()) {
            return false;
        }
        const RigExecRigPose pose = evaluator.Evaluate(UsdTimeCode(1024));
        const auto it = pose.jointMatricesFinal.find(wrist);
        if (!pose.valid || it == pose.jointMatricesFinal.end()) {
            return false;
        }
        *out = it->second;
        return true;
    };
    for (const RigExecEvaluationMode mode :
         {RigExecEvaluationMode::Dynamic, RigExecEvaluationMode::Baked}) {
        GfMatrix4d authored, half, connected, base, final;
        CHECK(wristAt(Wiring::Authored, mode, &authored));
        CHECK(wristAt(Wiring::AuthoredHalf, mode, &half));
        CHECK(wristAt(Wiring::Connected, mode, &connected));
        CHECK(wristAt(Wiring::ConnectedBase, mode, &base));
        CHECK(wristAt(Wiring::ConnectedFinal, mode, &final));
        // The halving moves the pose at all...
        CHECK(!Near(half.ExtractTranslation(), authored.ExtractTranslation(),
                    1e-3));
        // ...an undeclared connection and `base` read around it...
        CHECK(Near(connected.ExtractTranslation(),
                   authored.ExtractTranslation(), 1e-9));
        CHECK(Near(base.ExtractTranslation(), authored.ExtractTranslation(),
                   1e-9));
        // ...and `final` reads through it.
        CHECK(Near(final.ExtractTranslation(), half.ExtractTranslation(),
                   1e-9));
    }
}

// The read phase of a connection, on an in-memory rig of math movers. One
// dial at 0.45, revised by Gain (x2) and then Limit (clamp to 0.6): 0.45 at
// its base, 0.9 after Gain, 0.6 final. Readout movers add what they read to
// channels of their own:
//   Undeclared       no metadata                        0.45, a base record
//   Final            `final`                            0.6, no record
//   Hop              `base`                             0.45, a base record
//   FinalViaHop      `final`, through Hop's input       0.6, a record
//   FinalHop         `final`                            0.6, no record
//   BaseViaFinalHop  no metadata, through FinalHop's    0.45, a base record
//   Checkpoint       Gain's path                        0.9
//   LastCheckpoint   Limit's path, the last revision    0.6, a record
// A reader through a hop reads at its own phase, whatever the hop declares;
// a `final` that passes no recorded hop reads the dial's published value.
static const char *const kPhaseReadouts[] = {
    "Undeclared", "Final",   "Hop",       "FinalViaHop",
    "FinalHop",   "BaseViaFinalHop", "Checkpoint", "LastCheckpoint"};

static UsdStageRefPtr
_PhaseRig()
{
    const UsdStageRefPtr stage = UsdStage::CreateInMemory();
    stage->DefinePrim(SdfPath("/Asset"), TfToken("Xform"));
    stage->DefinePrim(SdfPath("/Asset/Rig"), TfToken("RigExecRoot"));
    stage->DefinePrim(SdfPath("/Asset/Rig/Movers"), TfToken("Scope"));
    stage->DefinePrim(SdfPath("/Asset/Rig/Movers/Readouts"),
                      TfToken("Scope"));
    const UsdAttribute dial =
        stage->DefinePrim(SdfPath("/Asset/Rig/Channels/Dial"),
                          TfToken("Scope"))
            .CreateAttribute(TfToken("rigExec:amount"),
                             SdfValueTypeNames->Float);
    dial.Set(0.45f);
    const auto mover = [&](const std::string &path, const char *operation,
                           const SdfPath &target) {
        const UsdPrim prim = stage->DefinePrim(
            SdfPath("/Asset/Rig/Movers/" + path),
            TfToken("RigExecFloatMathMover"));
        CHECK(prim.ApplyAPI(TfToken("RigExecMoverAPI")));
        prim.CreateAttribute(TfToken("rigExec:operation"),
                             SdfValueTypeNames->Token)
            .Set(TfToken(operation));
        prim.CreateRelationship(TfToken("rigExec:moves"))
            .SetTargets({target});
        return prim;
    };
    // A nested mover revises before its parent: Gain, then Limit.
    const UsdPrim limit = mover("Limit", "clamp", dial.GetPath());
    limit.CreateAttribute(TfToken("inputs:min"), SdfValueTypeNames->Float)
        .Set(0.0f);
    limit.CreateAttribute(TfToken("inputs:max"), SdfValueTypeNames->Float)
        .Set(0.6f);
    mover("Limit/Gain", "multiply", dial.GetPath())
        .CreateAttribute(TfToken("inputs:value"), SdfValueTypeNames->Float)
        .Set(2.0f);
    const UsdPrim readouts = stage->DefinePrim(
        SdfPath("/Asset/Rig/Channels/Readouts"), TfToken("Scope"));
    const auto readout = [&](const char *name, const SdfPath &source,
                             const char *phase) {
        const UsdAttribute channel = readouts.CreateAttribute(
            TfToken(std::string("rigExec:") + name),
            SdfValueTypeNames->Float);
        channel.Set(0.0f);
        const UsdAttribute value =
            mover(std::string("Readouts/") + name, "add", channel.GetPath())
                .CreateAttribute(TfToken("inputs:value"),
                                 SdfValueTypeNames->Float);
        value.SetConnections({source});
        if (phase) {
            value.SetMetadata(TfToken("rigExecReadPhase"),
                              std::string(phase));
        }
    };
    const SdfPath hop("/Asset/Rig/Movers/Readouts/Hop.inputs:value");
    const SdfPath finalHop(
        "/Asset/Rig/Movers/Readouts/FinalHop.inputs:value");
    readout("Undeclared", dial.GetPath(), nullptr);
    readout("Final", dial.GetPath(), "final");
    readout("Hop", dial.GetPath(), "base");
    readout("FinalViaHop", hop, "final");
    readout("FinalHop", dial.GetPath(), "final");
    readout("BaseViaFinalHop", finalHop, nullptr);
    readout("Checkpoint", dial.GetPath(), "/Asset/Rig/Movers/Limit/Gain");
    readout("LastCheckpoint", dial.GetPath(), "/Asset/Rig/Movers/Limit");
    return stage;
}

static float
_PhaseReadout(const RigExecRigPose &pose, const char *name)
{
    const auto it = pose.movedProperties.find(SdfPath(
        std::string("/Asset/Rig/Channels/Readouts.rigExec:") + name));
    return it != pose.movedProperties.end() && it->second.IsHolding<float>()
               ? it->second.UncheckedGet<float>()
               : -1.0f;
}

// Every readout of \p pose against \p expected, in kPhaseReadouts order.
static bool
_PhaseReadoutsAre(const char *what, const RigExecRigPose &pose,
                  const std::vector<float> &expected)
{
    bool same = pose.valid;
    for (size_t i = 0; i < expected.size(); ++i) {
        const float got = _PhaseReadout(pose, kPhaseReadouts[i]);
        if (std::abs(got - expected[i]) > 1e-6f) {
            std::printf("  %s: %s reads %.9g, expected %.9g\n", what,
                        kPhaseReadouts[i], double(got),
                        double(expected[i]));
            same = false;
        }
    }
    return same;
}

static void
TestDefaultReadPhaseAndHeadWins()
{
    const std::string readers = "/Asset/Rig/Movers/Readouts/";
    for (const RigExecEvaluationMode mode :
         {RigExecEvaluationMode::Dynamic, RigExecEvaluationMode::Baked}) {
        const UsdStageRefPtr stage = _PhaseRig();
        RigExecRigEvaluator evaluator(stage, SdfPath("/Asset/Rig"));
        evaluator.cpuParityMode = true;
        evaluator.SetEvaluationMode(mode);
        std::vector<std::string> errors;
        CHECK(evaluator.Compile(&errors));
        CHECK(evaluator.GetSkippedOperations().empty());
        std::map<std::string, size_t> applied;
        std::set<std::string> finals;
        for (const RigExecPhasedConnection &connection :
             evaluator.GetPhasedConnections()) {
            applied[connection.consumer.GetString()] = connection.applied;
            if (connection.final) {
                finals.insert(connection.consumer.GetString());
            }
            CHECK(connection.target ==
                  SdfPath("/Asset/Rig/Channels/Dial.rigExec:amount"));
        }
        const auto record = [&](const char *name) -> long {
            const auto it = applied.find(readers + name + ".inputs:value");
            return it == applied.end() ? -1 : long(it->second);
        };
        CHECK(applied.size() == 6);
        CHECK(record("Undeclared") == 0);
        CHECK(record("Hop") == 0);
        CHECK(record("BaseViaFinalHop") == 0);
        CHECK(record("Checkpoint") == 1);
        // `final` through the recorded Hop: every revision, as the
        // checkpoint at the last revision is -- only one of them is final.
        CHECK(record("FinalViaHop") == 2);
        CHECK(record("LastCheckpoint") == 2);
        CHECK(finals ==
              std::set<std::string>{readers + "FinalViaHop.inputs:value"});
        CHECK(record("Final") == -1);
        CHECK(record("FinalHop") == -1);
        // FinalViaHop's hops: its own input, then Hop's.
        for (const RigExecPhasedConnection &connection :
             evaluator.GetPhasedConnections()) {
            if (connection.consumer.GetString() ==
                readers + "FinalViaHop.inputs:value") {
                CHECK(connection.hops ==
                      (SdfPathVector{connection.consumer,
                                     SdfPath(readers + "Hop.inputs:value")}));
            }
        }
        CHECK(_PhaseReadoutsAre(
            "default phases", evaluator.Evaluate(UsdTimeCode(1.0)),
            {0.45f, 0.6f, 0.45f, 0.6f, 0.6f, 0.45f, 0.9f, 0.6f}));
    }
}

// Interactive overrides against phased readers, the same in every
// evaluator. One on a reader's own input, or on a hop its walk passes
// before the target, stands the reader aside: the overlay walk meets the
// override first. One on the target is the chain's base: base readers read
// it, and the checkpoint and final readers the chain revised from it.
static void
TestPhasedReadDragRules()
{
    const auto drag = [](const char *prim, const char *attribute,
                         float value) {
        return RigExecValueOverride{SdfPath(prim), TfToken(),
                                    TfToken(attribute), VtValue(value)};
    };
    const std::vector<float> undragged = {0.45f, 0.6f,  0.45f, 0.6f,
                                          0.6f,  0.45f, 0.9f,  0.6f};
    for (const RigExecEvaluationMode mode :
         {RigExecEvaluationMode::Dynamic, RigExecEvaluationMode::Baked}) {
        const UsdStageRefPtr stage = _PhaseRig();
        RigExecRigEvaluator evaluator(stage, SdfPath("/Asset/Rig"));
        evaluator.cpuParityMode = true;
        evaluator.SetEvaluationMode(mode);
        CHECK(evaluator.Compile(nullptr));
        const UsdTimeCode time(1.0);
        CHECK(_PhaseReadoutsAre("before the drags", evaluator.Evaluate(time),
                                undragged));

        // A recorded reader's own input, which FinalViaHop's walk passes.
        evaluator.SetInteractiveOverrides({drag(
            "/Asset/Rig/Movers/Readouts/Hop", "inputs:value", 0.125f)});
        CHECK(_PhaseReadoutsAre(
            "drag on Hop", evaluator.Evaluate(time),
            {0.45f, 0.6f, 0.125f, 0.125f, 0.6f, 0.45f, 0.9f, 0.6f}));

        // An unrecorded hop: the base reader through it stands aside.
        evaluator.SetInteractiveOverrides({drag(
            "/Asset/Rig/Movers/Readouts/FinalHop", "inputs:value", 0.375f)});
        CHECK(_PhaseReadoutsAre(
            "drag on FinalHop", evaluator.Evaluate(time),
            {0.45f, 0.6f, 0.45f, 0.6f, 0.375f, 0.375f, 0.9f, 0.6f}));

        // The target: its base is the drag, 0.25, which Gain doubles to 0.5
        // and Limit leaves there, so the checkpoints and every final reader
        // read 0.5 and the dial publishes it.
        evaluator.SetInteractiveOverrides(
            {drag("/Asset/Rig/Channels/Dial", "rigExec:amount", 0.25f)});
        const RigExecRigPose onTarget = evaluator.Evaluate(time);
        CHECK(_PhaseReadoutsAre(
            "drag on the dial", onTarget,
            {0.25f, 0.5f, 0.25f, 0.5f, 0.5f, 0.25f, 0.5f, 0.5f}));
        const auto dial = onTarget.movedProperties.find(
            SdfPath("/Asset/Rig/Channels/Dial.rigExec:amount"));
        CHECK(dial != onTarget.movedProperties.end() &&
              dial->second == VtValue(0.5f));

        // Lifted: the chain's own values again.
        evaluator.SetInteractiveOverrides({});
        CHECK(_PhaseReadoutsAre("after the drags", evaluator.Evaluate(time),
                                undragged));
    }
}

// A float or double by its bits (signed zeros and NaN payloads included),
// any other value by ==.
static bool
_SameValueBits(const VtValue &a, const VtValue &b)
{
    if (a.IsHolding<float>() && b.IsHolding<float>()) {
        const float x = a.UncheckedGet<float>(), y = b.UncheckedGet<float>();
        return std::memcmp(&x, &y, sizeof(x)) == 0;
    }
    if (a.IsHolding<double>() && b.IsHolding<double>()) {
        const double x = a.UncheckedGet<double>();
        const double y = b.UncheckedGet<double>();
        return std::memcmp(&x, &y, sizeof(x)) == 0;
    }
    return a == b;
}

// The first reading \p a and \p b disagree on, or empty when every reader
// reads the same: each moved property, control frame, joint frame and
// provider transform bit for bit, and the diagnostics in order. The work
// counters are left out.
static std::string
_ReadingsDiffer(const RigExecRigPose &a, const RigExecRigPose &b)
{
    if (!a.valid || !b.valid) {
        return "an invalid pose";
    }
    const auto sameFrame = [](const RigExecPointFrame &x,
                              const RigExecPointFrame &y) {
        return x.flags == y.flags &&
               std::memcmp(x.points.data(), y.points.data(),
                           sizeof(x.points)) == 0;
    };
    const auto sameMatrix = [](const GfMatrix4d &x, const GfMatrix4d &y) {
        return std::memcmp(x.GetArray(), y.GetArray(), 16 * sizeof(double)) ==
               0;
    };
    std::string why;
    const auto compare = [&why](const auto &x, const auto &y,
                                const char *what, auto same) {
        if (!why.empty()) {
            return;
        }
        if (x.size() != y.size()) {
            why = std::string(what) + " count";
            return;
        }
        for (auto i = x.begin(), j = y.begin(); i != x.end(); ++i, ++j) {
            if (i->first != j->first || !same(i->second, j->second)) {
                why = std::string(what) + " at " + i->first.GetString();
                return;
            }
        }
    };
    compare(a.movedProperties, b.movedProperties, "moved property",
            _SameValueBits);
    compare(a.controlFrames, b.controlFrames, "control frame", sameFrame);
    compare(a.jointFramesFinal, b.jointFramesFinal, "joint frame", sameFrame);
    compare(a.providerXforms, b.providerXforms, "provider transform",
            sameMatrix);
    // The mover graph's work line counts what this generation ran, which
    // a held drag and its release need not share.
    const auto lines = [](const RigExecRigPose &pose) {
        std::vector<std::string> out;
        for (const std::string &line : pose.diagnostics) {
            if (line.rfind("mover graph: ", 0) != 0) {
                out.push_back(line);
            }
        }
        return out;
    };
    const std::vector<std::string> x = lines(a), y = lines(b);
    if (why.empty() && x != y) {
        size_t k = 0;
        while (k < x.size() && k < y.size() && x[k] == y[k]) {
            ++k;
        }
        why = "diagnostic " + std::to_string(k) + ": '" +
              (k < x.size() ? x[k] : "") + "' against '" +
              (k < y.size() ? y[k] : "") + "'";
    }
    return why;
}

static bool
_SameReadings(const std::string &what, const RigExecRigPose &a,
              const RigExecRigPose &b)
{
    const std::string why = _ReadingsDiffer(a, b);
    if (!why.empty()) {
        std::printf("  %s: the readings differ (%s)\n", what.c_str(),
                    why.c_str());
    }
    return why.empty();
}

static const char *
_ModeName(RigExecEvaluationMode mode)
{
    return mode == RigExecEvaluationMode::Dynamic  ? "dynamic"
           : mode == RigExecEvaluationMode::Baked ? "baked"
                                                  : "parity";
}

// _PhaseRig plus readers outside the readout chains: a control's avars
// reading the dial undeclared, at `final` and at Gain's checkpoint (all
// records: a double reading a float chain), and a matrix mover's envelope
// reading it at `final` with no record, through the overlay walk to the
// dial's published value.
static UsdStageRefPtr
_TargetDragRig()
{
    const UsdStageRefPtr stage = _PhaseRig();
    const SdfPath dial("/Asset/Rig/Channels/Dial.rigExec:amount");
    const auto control = [&](const char *path) {
        const UsdPrim prim =
            stage->DefinePrim(SdfPath(path), TfToken("RigExecControl"));
        prim.CreateAttribute(TfToken("rest:space"),
                             SdfValueTypeNames->Matrix4d)
            .Set(GfMatrix4d(1.0));
        return prim;
    };
    const UsdPrim gauge = control("/Asset/Rig/Controls/Gauge");
    const auto avar = [&](const char *name, const char *phase) {
        const UsdAttribute a = gauge.CreateAttribute(
            TfToken(name), SdfValueTypeNames->Double);
        a.SetConnections({dial});
        if (phase) {
            a.SetMetadata(TfToken("rigExecReadPhase"), std::string(phase));
        }
    };
    avar("avars:tx", nullptr);
    avar("avars:ty", "final");
    avar("avars:tz", "/Asset/Rig/Movers/Limit/Gain");
    const UsdPrim lever = control("/Asset/Rig/Controls/Lever");
    lever.CreateAttribute(TfToken("avars:ty"), SdfValueTypeNames->Double)
        .Set(2.0);
    const UsdPrim card =
        stage->DefinePrim(SdfPath("/Asset/Geom/Card"), TfToken("Points"));
    card.CreateAttribute(TfToken("points"), SdfValueTypeNames->Point3fArray)
        .Set(VtVec3fArray{GfVec3f(1, 0, 0)});
    const UsdPrim lift = stage->DefinePrim(
        SdfPath("/Asset/Rig/Movers/Lift"), TfToken("RigExecMatrixMover"));
    CHECK(lift.ApplyAPI(TfToken("RigExecMoverAPI")));
    lift.CreateRelationship(TfToken("rigExec:moves"))
        .SetTargets({card.GetPath().AppendProperty(TfToken("points"))});
    lift.CreateRelationship(TfToken("rigExec:transform"))
        .SetTargets({lever.GetPath()});
    const UsdAttribute weight = lift.CreateAttribute(
        TfToken("inputs:defaultWeight"), SdfValueTypeNames->Float);
    weight.SetConnections({dial});
    weight.SetMetadata(TfToken("rigExecReadPhase"), std::string("final"));
    return stage;
}

// A drag on a property-chain target is an edit of its base, the same in
// every evaluator: every reader -- the readout chains reading the dial
// inside the chain loop undeclared, at `base`, at `final` and at both
// checkpoints, directly and through hops; the control's avars; the matrix
// mover's envelope -- reads during the drag exactly what it reads once the
// drag is authored and released, bit for bit, and what a fresh evaluator
// of the authored stage reads. The drags are 0.25, inside Limit's clamp;
// 0.4, which Gain lifts past it; and 1.4 and 1.5, past it at the base. A
// second run under a held drag republishes the same readings, and lifting
// the drag gives back the authored ones. A held drag moved from value to
// value without being lifted, as each mouse sample moves it, reads at each
// step what an evaluator dragged straight to that value reads: 0.4 -> 1.4
// and 1.4 -> 1.5 leave the clamped final where it was while the base
// readers and Gain's checkpoint move.
static void
TestTargetDragEqualsRelease()
{
    const SdfPath rigPath("/Asset/Rig");
    const SdfPath dial("/Asset/Rig/Channels/Dial.rigExec:amount");
    const SdfPath gauge("/Asset/Rig/Controls/Gauge");
    const UsdTimeCode time(1.0);
    const auto dragTo = [&](float value) {
        return std::vector<RigExecValueOverride>{RigExecValueOverride{
            dial.GetPrimPath(), TfToken(), dial.GetNameToken(),
            VtValue(value)}};
    };
    for (const RigExecEvaluationMode mode :
         {RigExecEvaluationMode::Dynamic, RigExecEvaluationMode::Baked,
          RigExecEvaluationMode::BakedWithParityCheck}) {
        // Each drag as an evaluator that ran the authored rig first reads it.
        std::map<float, RigExecRigPose> draggedTo;
        for (const float value : {0.25f, 0.4f, 1.4f, 1.5f}) {
            char label[64];
            std::snprintf(label, sizeof(label), "%s drag %.9g",
                          _ModeName(mode), double(value));
            const UsdStageRefPtr stage = _TargetDragRig();
            RigExecRigEvaluator evaluator(stage, rigPath);
            evaluator.SetEvaluationMode(mode);
            std::vector<std::string> errors;
            CHECK(evaluator.Compile(&errors));
            CHECK(evaluator.GetSkippedOperations().empty());
            const RigExecRigPose authored = evaluator.Evaluate(time);
            CHECK(authored.valid);

            const size_t bakedBefore = evaluator.GetBakedGenerationCount();
            evaluator.SetInteractiveOverrides(dragTo(value));
            const RigExecRigPose dragged = evaluator.Evaluate(time);
            CHECK(dragged.valid && dragged.bakedParityMismatches == 0);
            if (mode != RigExecEvaluationMode::Dynamic) {
                // The program answered the drag, not a dynamic fallback.
                CHECK(evaluator.GetBakedGenerationCount() > bakedBefore);
            }
            draggedTo[value] = dragged;
            CHECK(!_ReadingsDiffer(authored, dragged).empty());
            const float gain = value * 2.0f;
            const float final = std::min(gain, 0.6f);
            CHECK(_PhaseReadoutsAre(
                label, dragged,
                {value, final, value, final, final, value, gain, final}));
            const auto published = dragged.movedProperties.find(dial);
            CHECK(published != dragged.movedProperties.end() &&
                  published->second == VtValue(final));
            const auto frame = dragged.controlFrames.find(gauge);
            CHECK(frame != dragged.controlFrames.end() &&
                  frame->second.points[0] ==
                      GfVec3d(double(value), double(final), double(gain)));
            CHECK(_SameReadings(std::string(label) + ", held", dragged,
                                evaluator.Evaluate(time)));

            evaluator.ClearInteractiveOverrides();
            CHECK(_SameReadings(std::string(label) + ", lifted", authored,
                                evaluator.Evaluate(time)));

            stage->GetAttributeAtPath(dial).Set(value);
            const RigExecRigPose released = evaluator.Evaluate(time);
            CHECK(released.bakedParityMismatches == 0);
            CHECK(_SameReadings(std::string(label) + ", released", dragged,
                                released));
            RigExecRigEvaluator fresh(stage, rigPath);
            fresh.SetEvaluationMode(mode);
            CHECK(fresh.Compile(nullptr));
            const RigExecRigPose rebuilt = fresh.Evaluate(time);
            CHECK(rebuilt.bakedParityMismatches == 0);
            CHECK(_SameReadings(std::string(label) + ", fresh", dragged,
                                rebuilt));
        }

        const UsdStageRefPtr stage = _TargetDragRig();
        RigExecRigEvaluator evaluator(stage, rigPath);
        evaluator.SetEvaluationMode(mode);
        CHECK(evaluator.Compile(nullptr));
        CHECK(evaluator.Evaluate(time).valid);
        for (const float value : {0.25f, 0.4f, 1.4f, 1.5f, 0.25f}) {
            char label[64];
            std::snprintf(label, sizeof(label), "%s drag moved to %.9g",
                          _ModeName(mode), double(value));
            const size_t bakedBefore = evaluator.GetBakedGenerationCount();
            evaluator.SetInteractiveOverrides(dragTo(value));
            const RigExecRigPose moved = evaluator.Evaluate(time);
            CHECK(moved.valid && moved.bakedParityMismatches == 0);
            if (mode != RigExecEvaluationMode::Dynamic) {
                CHECK(evaluator.GetBakedGenerationCount() > bakedBefore);
            }
            CHECK(_SameReadings(label, draggedTo[value], moved));
        }
    }
}

// The blink: a float channel authored at 0.2 and clamped to [0, 1], read
// undeclared and at `final` by readout chains and, when \p lid, by a lid
// control's avars (tx at the base, ty at `final`). \p authored false
// leaves the channel without a value; \p clampEnabled false authors the
// clamp disabled, so every revision passes through.
static UsdStageRefPtr
_BlinkRig(bool lid, bool authored = true, bool clampEnabled = true)
{
    const UsdStageRefPtr stage = UsdStage::CreateInMemory();
    stage->DefinePrim(SdfPath("/Asset"), TfToken("Xform"));
    stage->DefinePrim(SdfPath("/Asset/Rig"), TfToken("RigExecRoot"));
    stage->DefinePrim(SdfPath("/Asset/Rig/Movers"), TfToken("Scope"));
    const UsdAttribute blink =
        stage->DefinePrim(SdfPath("/Asset/Rig/Channels/Face"),
                          TfToken("Scope"))
            .CreateAttribute(TfToken("rigExec:blink"),
                             SdfValueTypeNames->Float);
    if (authored) {
        blink.Set(0.2f);
    }
    const auto mover = [&](const std::string &path, const char *operation,
                           const SdfPath &target) {
        const UsdPrim prim = stage->DefinePrim(
            SdfPath("/Asset/Rig/Movers/" + path),
            TfToken("RigExecFloatMathMover"));
        CHECK(prim.ApplyAPI(TfToken("RigExecMoverAPI")));
        prim.CreateAttribute(TfToken("rigExec:operation"),
                             SdfValueTypeNames->Token)
            .Set(TfToken(operation));
        prim.CreateRelationship(TfToken("rigExec:moves"))
            .SetTargets({target});
        return prim;
    };
    const UsdPrim clamp = mover("ClampBlink", "clamp", blink.GetPath());
    clamp.CreateAttribute(TfToken("inputs:min"), SdfValueTypeNames->Float)
        .Set(0.0f);
    clamp.CreateAttribute(TfToken("inputs:max"), SdfValueTypeNames->Float)
        .Set(1.0f);
    if (!clampEnabled) {
        clamp.CreateAttribute(TfToken("inputs:enabled"),
                              SdfValueTypeNames->Bool)
            .Set(false);
    }
    const UsdPrim readouts = stage->DefinePrim(
        SdfPath("/Asset/Rig/Channels/Readouts"), TfToken("Scope"));
    for (const char *name : {"base", "final"}) {
        const UsdAttribute channel = readouts.CreateAttribute(
            TfToken(std::string("rigExec:") + name),
            SdfValueTypeNames->Float);
        channel.Set(0.0f);
        const UsdAttribute input =
            mover(std::string("Readouts/") + name, "add", channel.GetPath())
                .CreateAttribute(TfToken("inputs:value"),
                                 SdfValueTypeNames->Float);
        input.SetConnections({blink.GetPath()});
        if (std::string(name) == "final") {
            input.SetMetadata(TfToken("rigExecReadPhase"),
                              std::string("final"));
        }
    }
    if (lid) {
        const UsdPrim control = stage->DefinePrim(
            SdfPath("/Asset/Rig/Controls/Lid"), TfToken("RigExecControl"));
        control
            .CreateAttribute(TfToken("rest:space"),
                             SdfValueTypeNames->Matrix4d)
            .Set(GfMatrix4d(1.0));
        control.CreateAttribute(TfToken("avars:tx"), SdfValueTypeNames->Double)
            .SetConnections({blink.GetPath()});
        const UsdAttribute ty = control.CreateAttribute(
            TfToken("avars:ty"), SdfValueTypeNames->Double);
        ty.SetConnections({blink.GetPath()});
        ty.SetMetadata(TfToken("rigExecReadPhase"), std::string("final"));
    }
    return stage;
}

static float
_BlinkReadout(const RigExecRigPose &pose, const char *name)
{
    const auto it = pose.movedProperties.find(SdfPath(
        std::string("/Asset/Rig/Channels/Readouts.rigExec:") + name));
    return it != pose.movedProperties.end() && it->second.IsHolding<float>()
               ? it->second.UncheckedGet<float>()
               : -1.0f;
}

// The blink, dragged: one drag in every evaluator against the stage with
// the dragged value authored, bit for bit, with the readings the case
// expects while dragged. Dragged to 1.4, the base readers read 1.4 and the
// final readers 1.0; with the clamp disabled every revision passes
// through, so both read 1.4; with no authored value the drag is still a
// base, so the chain runs from it; and a drag that is not finite skips the
// chain with the line an authored one prints, leaving every reader on the
// dragged value. A held drag moved without being lifted reads at each step
// what a drag placed straight at that value reads: 1.4 -> 1.5 leaves the
// clamped final on 1.0 while the base readers move, and 1.5 -> 0.5 moves
// both. A double drag on the float blink is the float base it narrows to.
static void
TestBlinkDragEqualsRelease()
{
    const SdfPath rigPath("/Asset/Rig");
    const SdfPath blink("/Asset/Rig/Channels/Face.rigExec:blink");
    const SdfPath lid("/Asset/Rig/Controls/Lid");
    const UsdTimeCode time(1.0);
    const float nan = std::numeric_limits<float>::quiet_NaN();
    struct Case {
        const char *what;
        bool lid, authored, clampEnabled;
        float drag;
        float base, final;  // the readouts while dragged; NaN: unchecked
    };
    const Case cases[] = {
        {"blink 1.4", true, true, true, 1.4f, 1.4f, 1.0f},
        {"blink 1.4, clamp disabled", true, true, false, 1.4f, 1.4f, 1.4f},
        {"blink 0.5, nothing authored", true, false, true, 0.5f, 0.5f, 0.5f},
        // The readouts cannot add a NaN and pass through, keeping 0.
        {"blink NaN", false, true, true, nan, 0.0f, 0.0f},
    };
    for (const RigExecEvaluationMode mode :
         {RigExecEvaluationMode::Dynamic, RigExecEvaluationMode::Baked,
          RigExecEvaluationMode::BakedWithParityCheck}) {
        for (const Case &c : cases) {
            const std::string label =
                std::string(_ModeName(mode)) + " " + c.what;
            const UsdStageRefPtr stage =
                _BlinkRig(c.lid, c.authored, c.clampEnabled);
            RigExecRigEvaluator evaluator(stage, rigPath);
            evaluator.SetEvaluationMode(mode);
            CHECK(evaluator.Compile(nullptr));
            CHECK(evaluator.GetSkippedOperations().empty());
            const RigExecRigPose authored = evaluator.Evaluate(time);
            const size_t bakedBefore = evaluator.GetBakedGenerationCount();
            evaluator.SetInteractiveOverrides({RigExecValueOverride{
                blink.GetPrimPath(), TfToken(), blink.GetNameToken(),
                VtValue(c.drag)}});
            const RigExecRigPose dragged = evaluator.Evaluate(time);
            CHECK(dragged.valid && dragged.bakedParityMismatches == 0);
            if (mode != RigExecEvaluationMode::Dynamic) {
                // The program answered the drag, not a dynamic fallback.
                CHECK(evaluator.GetBakedGenerationCount() > bakedBefore);
            }
            const bool read = _BlinkReadout(dragged, "base") == c.base &&
                              _BlinkReadout(dragged, "final") == c.final;
            if (!read) {
                std::printf("  %s: base reads %.9g, final %.9g\n",
                            label.c_str(),
                            double(_BlinkReadout(dragged, "base")),
                            double(_BlinkReadout(dragged, "final")));
            }
            CHECK(read);
            const auto published = dragged.movedProperties.find(blink);
            if (std::isfinite(c.drag)) {
                CHECK(published != dragged.movedProperties.end() &&
                      published->second == VtValue(c.final));
            } else {
                CHECK(published == dragged.movedProperties.end());
                CHECK(std::count(dragged.diagnostics.begin(),
                                 dragged.diagnostics.end(),
                                 "property chain " + blink.GetString() +
                                     ": authored base is not finite; chain "
                                     "skipped") == 1);
            }
            if (c.lid) {
                const auto frame = dragged.controlFrames.find(lid);
                CHECK(frame != dragged.controlFrames.end() &&
                      frame->second.points[0][0] == double(c.base) &&
                      frame->second.points[0][1] == double(c.final));
            }
            CHECK(_SameReadings(label + ", held", dragged,
                                evaluator.Evaluate(time)));
            evaluator.ClearInteractiveOverrides();
            CHECK(_SameReadings(label + ", lifted", authored,
                                evaluator.Evaluate(time)));

            stage->GetAttributeAtPath(blink).Set(c.drag);
            const RigExecRigPose released = evaluator.Evaluate(time);
            CHECK(released.bakedParityMismatches == 0);
            CHECK(_SameReadings(label + ", released", dragged, released));
            RigExecRigEvaluator fresh(stage, rigPath);
            fresh.SetEvaluationMode(mode);
            CHECK(fresh.Compile(nullptr));
            CHECK(_SameReadings(label + ", fresh", dragged,
                                fresh.Evaluate(time)));
        }

        // The authored, clamped blink with its lid, run once undragged and
        // then dragged to \p value.
        const auto draggedStraight = [&](const VtValue &value) {
            const UsdStageRefPtr stage = _BlinkRig(true);
            RigExecRigEvaluator evaluator(stage, rigPath);
            evaluator.SetEvaluationMode(mode);
            CHECK(evaluator.Compile(nullptr));
            CHECK(evaluator.Evaluate(time).valid);
            const size_t bakedBefore = evaluator.GetBakedGenerationCount();
            evaluator.SetInteractiveOverrides({RigExecValueOverride{
                blink.GetPrimPath(), TfToken(), blink.GetNameToken(),
                value}});
            const RigExecRigPose pose = evaluator.Evaluate(time);
            CHECK(pose.valid && pose.bakedParityMismatches == 0);
            if (mode != RigExecEvaluationMode::Dynamic) {
                CHECK(evaluator.GetBakedGenerationCount() > bakedBefore);
            }
            return pose;
        };
        {
            const UsdStageRefPtr stage = _BlinkRig(true);
            RigExecRigEvaluator evaluator(stage, rigPath);
            evaluator.SetEvaluationMode(mode);
            CHECK(evaluator.Compile(nullptr));
            CHECK(evaluator.Evaluate(time).valid);
            const struct {
                float drag, base, final;
            } steps[] = {{1.4f, 1.4f, 1.0f}, {1.5f, 1.5f, 1.0f},
                         {0.5f, 0.5f, 0.5f}};
            for (const auto &step : steps) {
                char label[64];
                std::snprintf(label, sizeof(label), "%s blink moved to %.9g",
                              _ModeName(mode), double(step.drag));
                const size_t bakedBefore = evaluator.GetBakedGenerationCount();
                evaluator.SetInteractiveOverrides({RigExecValueOverride{
                    blink.GetPrimPath(), TfToken(), blink.GetNameToken(),
                    VtValue(step.drag)}});
                const RigExecRigPose moved = evaluator.Evaluate(time);
                CHECK(moved.valid && moved.bakedParityMismatches == 0);
                if (mode != RigExecEvaluationMode::Dynamic) {
                    CHECK(evaluator.GetBakedGenerationCount() > bakedBefore);
                }
                const bool read =
                    _BlinkReadout(moved, "base") == step.base &&
                    _BlinkReadout(moved, "final") == step.final;
                if (!read) {
                    std::printf("  %s: base reads %.9g, final %.9g\n", label,
                                double(_BlinkReadout(moved, "base")),
                                double(_BlinkReadout(moved, "final")));
                }
                CHECK(read);
                CHECK(_SameReadings(label, draggedStraight(VtValue(step.drag)),
                                    moved));
            }
        }
        CHECK(_SameReadings(std::string(_ModeName(mode)) +
                                " blink dragged by a double",
                            draggedStraight(VtValue(1.4f)),
                            draggedStraight(VtValue(1.4))));
    }
}

// The default phase and a declared `final` on inputs outside the movers: a
// constraint's twist offset and a control's avar, each connected to a
// channel a math mover doubles (or scales by 1.5). Undeclared, each reads
// the channel as authored, posing exactly as the value authored on the
// input; declared `final`, each reads the revised value, posing exactly as
// that value authored. Both inputs are doubles, and so is the channel --
// or a float, which only a record widens, so a `final` read of it keeps
// one.
static void
TestConnectionReadPhaseOnConstraintAndAvar(const std::string &examplesDir)
{
    enum class Wiring { AuthoredBase, AuthoredFinal, Connected, ConnectedFinal };

    // The single-chain IK's twist: 20 degrees authored on the channel,
    // doubled to 40.
    const SdfPath elbow("/ScIkAsset/Rig/Joints/Shoulder/Elbow");
    const auto twistElbow = [&](Wiring wiring, RigExecEvaluationMode mode,
                                GfMatrix4d *out, bool floatChannel = false) {
        UsdStageRefPtr stage = UsdStage::Open(
            examplesDir + "/../docs/examples/single_chain_ik_constraint.usda");
        if (!stage) {
            return false;
        }
        stage->SetEditTarget(stage->GetSessionLayer());
        const UsdAttribute degrees =
            stage->DefinePrim(SdfPath("/ScIkAsset/Rig/Channels/Twist"),
                              TfToken("Scope"))
                .CreateAttribute(TfToken("rigExec:degrees"),
                                 floatChannel ? SdfValueTypeNames->Float
                                              : SdfValueTypeNames->Double);
        if (floatChannel) {
            degrees.Set(20.0f);
        } else {
            degrees.Set(20.0);
        }
        const UsdPrim twice = stage->DefinePrim(
            SdfPath("/ScIkAsset/Rig/Movers/TwistTwice"),
            TfToken("RigExecFloatMathMover"));
        twice.ApplyAPI(TfToken("RigExecMoverAPI"));
        twice.CreateAttribute(TfToken("rigExec:operation"),
                              SdfValueTypeNames->Token)
            .Set(TfToken("multiply"));
        twice.CreateAttribute(TfToken("inputs:value"),
                              SdfValueTypeNames->Float)
            .Set(2.0f);
        twice.CreateRelationship(TfToken("rigExec:moves"))
            .SetTargets({degrees.GetPath()});
        UsdAttribute twist = stage->GetAttributeAtPath(
            SdfPath("/ScIkAsset/Rig/Movers/Pose/ArmIK.inputs:twistDegrees"));
        if (wiring == Wiring::AuthoredBase) {
            twist.Set(20.0);
        } else if (wiring == Wiring::AuthoredFinal) {
            twist.Set(40.0);
        } else {
            twist.SetConnections({degrees.GetPath()});
        }
        if (wiring == Wiring::ConnectedFinal) {
            twist.SetMetadata(TfToken("rigExecReadPhase"),
                              std::string("final"));
        }
        RigExecRigEvaluator evaluator(stage, SdfPath("/ScIkAsset/Rig"));
        evaluator.cpuParityMode = true;
        evaluator.SetEvaluationMode(mode);
        if (!evaluator.Compile(nullptr) ||
            !evaluator.GetSkippedOperations().empty()) {
            return false;
        }
        const RigExecRigPose pose = evaluator.Evaluate(UsdTimeCode(1012));
        const auto it = pose.jointMatricesFinal.find(elbow);
        if (!pose.valid || it == pose.jointMatricesFinal.end()) {
            return false;
        }
        *out = it->second;
        return true;
    };

    // A control whose avars:ty reads a height channel: 2 authored, scaled
    // by 1.5 to 3.
    const SdfPath control("/Asset/Rig/Controls/Lift");
    const auto liftedTo = [&](Wiring wiring, RigExecEvaluationMode mode,
                              double *out, bool floatChannel = false) {
        const UsdStageRefPtr stage = UsdStage::CreateInMemory();
        stage->DefinePrim(SdfPath("/Asset"), TfToken("Xform"));
        stage->DefinePrim(SdfPath("/Asset/Rig"), TfToken("RigExecRoot"));
        const UsdPrim lift =
            stage->DefinePrim(control, TfToken("RigExecControl"));
        lift.CreateAttribute(TfToken("rest:space"),
                             SdfValueTypeNames->Matrix4d)
            .Set(GfMatrix4d(1.0));
        const UsdAttribute height =
            stage->DefinePrim(SdfPath("/Asset/Rig/Channels/Lift"),
                              TfToken("Scope"))
                .CreateAttribute(TfToken("rigExec:height"),
                                 floatChannel ? SdfValueTypeNames->Float
                                              : SdfValueTypeNames->Double);
        if (floatChannel) {
            height.Set(2.0f);
        } else {
            height.Set(2.0);
        }
        const UsdPrim raise = stage->DefinePrim(
            SdfPath("/Asset/Rig/Movers/Raise"),
            TfToken("RigExecFloatMathMover"));
        raise.ApplyAPI(TfToken("RigExecMoverAPI"));
        raise.CreateAttribute(TfToken("rigExec:operation"),
                              SdfValueTypeNames->Token)
            .Set(TfToken("multiply"));
        raise.CreateAttribute(TfToken("inputs:value"),
                              SdfValueTypeNames->Float)
            .Set(1.5f);
        raise.CreateRelationship(TfToken("rigExec:moves"))
            .SetTargets({height.GetPath()});
        const UsdAttribute ty = lift.CreateAttribute(
            TfToken("avars:ty"), SdfValueTypeNames->Double);
        if (wiring == Wiring::AuthoredBase) {
            ty.Set(2.0);
        } else if (wiring == Wiring::AuthoredFinal) {
            ty.Set(3.0);
        } else {
            ty.SetConnections({height.GetPath()});
        }
        if (wiring == Wiring::ConnectedFinal) {
            ty.SetMetadata(TfToken("rigExecReadPhase"), std::string("final"));
        }
        RigExecRigEvaluator evaluator(stage, SdfPath("/Asset/Rig"));
        evaluator.cpuParityMode = true;
        evaluator.SetEvaluationMode(mode);
        if (!evaluator.Compile(nullptr) ||
            !evaluator.GetSkippedOperations().empty()) {
            return false;
        }
        const RigExecRigPose pose = evaluator.Evaluate(UsdTimeCode(1.0));
        const auto it = pose.controlFrames.find(control);
        if (!pose.valid || it == pose.controlFrames.end() ||
            it->second.points.empty()) {
            return false;
        }
        *out = it->second.points[0][1];
        return true;
    };

    for (const RigExecEvaluationMode mode :
         {RigExecEvaluationMode::Dynamic, RigExecEvaluationMode::Baked}) {
        GfMatrix4d base, final, connected, connectedFinal;
        CHECK(twistElbow(Wiring::AuthoredBase, mode, &base));
        CHECK(twistElbow(Wiring::AuthoredFinal, mode, &final));
        CHECK(twistElbow(Wiring::Connected, mode, &connected));
        CHECK(twistElbow(Wiring::ConnectedFinal, mode, &connectedFinal));
        // The doubling moves the elbow at all...
        CHECK(!Near(base.ExtractTranslation(), final.ExtractTranslation(),
                    1e-3));
        // ...an undeclared twist reads around it, `final` through it.
        CHECK(Near(connected.ExtractTranslation(), base.ExtractTranslation(),
                   1e-9));
        CHECK(Near(connectedFinal.ExtractTranslation(),
                   final.ExtractTranslation(), 1e-9));
        GfMatrix4d fromFloat, fromFloatFinal;
        CHECK(twistElbow(Wiring::Connected, mode, &fromFloat, true));
        CHECK(twistElbow(Wiring::ConnectedFinal, mode, &fromFloatFinal,
                         true));
        CHECK(Near(fromFloat.ExtractTranslation(), base.ExtractTranslation(),
                   1e-9));
        CHECK(Near(fromFloatFinal.ExtractTranslation(),
                   final.ExtractTranslation(), 1e-9));

        double liftBase = -1, liftFinal = -1, liftConnected = -1,
               liftConnectedFinal = -1;
        CHECK(liftedTo(Wiring::AuthoredBase, mode, &liftBase));
        CHECK(liftedTo(Wiring::AuthoredFinal, mode, &liftFinal));
        CHECK(liftedTo(Wiring::Connected, mode, &liftConnected));
        CHECK(liftedTo(Wiring::ConnectedFinal, mode, &liftConnectedFinal));
        CHECK(liftBase == 2.0 && liftFinal == 3.0);
        CHECK(liftConnected == liftBase);
        CHECK(liftConnectedFinal == liftFinal);
        double liftFromFloat = -1, liftFromFloatFinal = -1;
        CHECK(liftedTo(Wiring::Connected, mode, &liftFromFloat, true));
        CHECK(liftedTo(Wiring::ConnectedFinal, mode, &liftFromFloatFinal,
                       true));
        CHECK(liftFromFloat == liftBase);
        CHECK(liftFromFloatFinal == liftFinal);
    }
}

// Undeclared inputs never fail the compile: one that math movers revise
// themselves reads its own chain's result, and one of another value type
// never read the chain -- neither gets a record. The type test is by value
// type: a float3 input reads a color3f chain at its base. Declared, the
// mistyped read is refused and its mover set aside. (The mistyped rigs are
// compiled, not evaluated: their read was never the chain's, and is not
// what this checks.)
static void
TestUndeclaredSkipsCompileCleanly()
{
    enum class Mistyped { None, Undeclared, Declared };
    const auto build = [](Mistyped mistyped) {
        const UsdStageRefPtr stage = _PhaseRig();
        const auto mover = [&](const char *path, const char *type,
                               const char *operation,
                               const SdfPath &target) {
            const UsdPrim prim = stage->DefinePrim(
                SdfPath(std::string("/Asset/Rig/Movers/") + path),
                TfToken(type));
            CHECK(prim.ApplyAPI(TfToken("RigExecMoverAPI")));
            prim.CreateAttribute(TfToken("rigExec:operation"),
                                 SdfValueTypeNames->Token)
                .Set(TfToken(operation));
            prim.CreateRelationship(TfToken("rigExec:moves"))
                .SetTargets({target});
            return prim;
        };
        const SdfPath dial("/Asset/Rig/Channels/Dial.rigExec:amount");
        const UsdPrim channels =
            stage->GetPrimAtPath(SdfPath("/Asset/Rig/Channels/Dial"));
        // A connected chain target: its own value 1, plus 0.5.
        const UsdAttribute out = channels.CreateAttribute(
            TfToken("rigExec:out"), SdfValueTypeNames->Float);
        out.Set(1.0f);
        out.SetConnections({dial});
        mover("Bump", "RigExecFloatMathMover", "add", out.GetPath())
            .CreateAttribute(TfToken("inputs:value"),
                             SdfValueTypeNames->Float)
            .Set(0.5f);
        // A color3f chain read by a float3 input: the same value type.
        const UsdAttribute paint = channels.CreateAttribute(
            TfToken("rigExec:paint"), SdfValueTypeNames->Color3f);
        paint.Set(GfVec3f(0.25f, 0.5f, 0.75f));
        mover("Brighten", "RigExecVec3fMathMover", "multiply",
              paint.GetPath())
            .CreateAttribute(TfToken("inputs:value"),
                             SdfValueTypeNames->Float3)
            .Set(GfVec3f(2.0f));
        const UsdAttribute copy = channels.CreateAttribute(
            TfToken("rigExec:copy"), SdfValueTypeNames->Float3);
        copy.Set(GfVec3f(0.0f));
        mover("Copy", "RigExecVec3fMathMover", "add", copy.GetPath())
            .CreateAttribute(TfToken("inputs:value"),
                             SdfValueTypeNames->Float3)
            .SetConnections({paint.GetPath()});
        if (mistyped != Mistyped::None) {
            // A float3 input connected to the float dial.
            const UsdAttribute tint = channels.CreateAttribute(
                TfToken("rigExec:tint"), SdfValueTypeNames->Float3);
            tint.Set(GfVec3f(0.0f));
            const UsdAttribute value =
                mover("Tint", "RigExecVec3fMathMover", "add",
                      tint.GetPath())
                    .CreateAttribute(TfToken("inputs:value"),
                                     SdfValueTypeNames->Float3);
            value.SetConnections({dial});
            if (mistyped == Mistyped::Declared) {
                value.SetMetadata(TfToken("rigExecReadPhase"),
                                  std::string("base"));
            }
        }
        return stage;
    };
    const auto recorded = [](const RigExecRigEvaluator &evaluator,
                             const char *consumer) -> long {
        for (const RigExecPhasedConnection &connection :
             evaluator.GetPhasedConnections()) {
            if (connection.consumer == SdfPath(consumer)) {
                return long(connection.applied);
            }
        }
        return -1;
    };

    {
        RigExecRigEvaluator evaluator(build(Mistyped::None),
                                      SdfPath("/Asset/Rig"));
        evaluator.cpuParityMode = true;
        std::vector<std::string> errors;
        CHECK(evaluator.Compile(&errors));
        CHECK(evaluator.GetSkippedOperations().empty());
        CHECK(recorded(evaluator, "/Asset/Rig/Channels/Dial.rigExec:out") ==
              -1);
        CHECK(recorded(evaluator, "/Asset/Rig/Movers/Copy.inputs:value") ==
              0);
        // The connected target is its own chain's base, revised; the copy
        // reads the color chain's base.
        const RigExecRigPose pose = evaluator.Evaluate(UsdTimeCode(1.0));
        CHECK(pose.valid);
        const auto out = pose.movedProperties.find(
            SdfPath("/Asset/Rig/Channels/Dial.rigExec:out"));
        CHECK(out != pose.movedProperties.end() &&
              out->second == VtValue(1.5f));
        const auto copy = pose.movedProperties.find(
            SdfPath("/Asset/Rig/Channels/Dial.rigExec:copy"));
        CHECK(copy != pose.movedProperties.end() &&
              copy->second == VtValue(GfVec3f(0.25f, 0.5f, 0.75f)));
    }
    {
        RigExecRigEvaluator evaluator(build(Mistyped::Undeclared),
                                      SdfPath("/Asset/Rig"));
        evaluator.cpuParityMode = true;
        std::vector<std::string> errors;
        CHECK(evaluator.Compile(&errors));
        CHECK(evaluator.GetSkippedOperations().empty());
        CHECK(recorded(evaluator, "/Asset/Rig/Movers/Tint.inputs:value") ==
              -1);
    }
    {
        RigExecRigEvaluator evaluator(build(Mistyped::Declared),
                                      SdfPath("/Asset/Rig"));
        evaluator.cpuParityMode = true;
        std::vector<std::string> errors;
        CHECK(evaluator.Compile(&errors));
        CHECK(evaluator.GetSkippedOperations().count(
            SdfPath("/Asset/Rig/Movers/Tint")));
        bool said = false;
        for (const std::string &e : errors) {
            said |= e.find("a phased read needs the same value type") !=
                    std::string::npos;
        }
        CHECK(said);
    }
}

// Rewiring a constraint field into a chain target, and declaring a phase on
// it, are structural: the connection walk and the phase are in the
// structure digest, and the next evaluation reads at the new phase. The
// plain channel and the revised one are both authored at 20, so the rewire
// reads 20 either way at its base; declared `final`, the twist reads 40,
// posing exactly as 40 authored on the input.
static void
TestConstraintRewireIsStructural(const std::string &examplesDir)
{
    UsdStageRefPtr stage = UsdStage::Open(
        examplesDir + "/../docs/examples/single_chain_ik_constraint.usda");
    CHECK(stage);
    if (!stage) {
        return;
    }
    stage->SetEditTarget(stage->GetSessionLayer());
    const UsdPrim twistScope = stage->DefinePrim(
        SdfPath("/ScIkAsset/Rig/Channels/Twist"), TfToken("Scope"));
    const UsdAttribute plain = twistScope.CreateAttribute(
        TfToken("rigExec:plain"), SdfValueTypeNames->Double);
    plain.Set(20.0);
    const UsdAttribute degrees = twistScope.CreateAttribute(
        TfToken("rigExec:degrees"), SdfValueTypeNames->Double);
    degrees.Set(20.0);
    const UsdPrim twice = stage->DefinePrim(
        SdfPath("/ScIkAsset/Rig/Movers/TwistTwice"),
        TfToken("RigExecFloatMathMover"));
    twice.ApplyAPI(TfToken("RigExecMoverAPI"));
    twice.CreateAttribute(TfToken("rigExec:operation"),
                          SdfValueTypeNames->Token)
        .Set(TfToken("multiply"));
    twice.CreateAttribute(TfToken("inputs:value"), SdfValueTypeNames->Float)
        .Set(2.0f);
    twice.CreateRelationship(TfToken("rigExec:moves"))
        .SetTargets({degrees.GetPath()});
    UsdAttribute twist = stage->GetAttributeAtPath(
        SdfPath("/ScIkAsset/Rig/Movers/Pose/ArmIK.inputs:twistDegrees"));
    twist.SetConnections({plain.GetPath()});

    RigExecRigEvaluator evaluator(stage, SdfPath("/ScIkAsset/Rig"));
    evaluator.cpuParityMode = true;
    CHECK(evaluator.Compile(nullptr));
    const UsdTimeCode time(1012);
    const SdfPath elbow("/ScIkAsset/Rig/Joints/Shoulder/Elbow");
    const auto elbowAt = [&]() {
        const RigExecRigPose pose = evaluator.Evaluate(time);
        CHECK(pose.valid);
        const auto it = pose.jointMatricesFinal.find(elbow);
        CHECK(it != pose.jointMatricesFinal.end());
        return it != pose.jointMatricesFinal.end()
                   ? it->second.ExtractTranslation()
                   : GfVec3d(std::numeric_limits<double>::quiet_NaN());
    };
    const GfVec3d at20 = elbowAt();
    const size_t unrevised = evaluator.GetBindingEpochDigest();
    const auto recorded = [&]() {
        for (const RigExecPhasedConnection &connection :
             evaluator.GetPhasedConnections()) {
            if (connection.consumer == twist.GetPath()) {
                return long(connection.applied);
            }
        }
        return -1L;
    };
    CHECK(recorded() == -1);

    twist.SetConnections({degrees.GetPath()});
    CHECK(Near(elbowAt(), at20, 1e-9));
    const size_t rewired = evaluator.GetBindingEpochDigest();
    CHECK(rewired != unrevised);
    CHECK(recorded() == 0);

    twist.SetMetadata(TfToken("rigExecReadPhase"), std::string("final"));
    const GfVec3d atFinal = elbowAt();
    CHECK(evaluator.GetBindingEpochDigest() != rewired);
    CHECK(recorded() == -1);
    CHECK(!Near(atFinal, at20, 1e-3));

    // 40 authored on the input poses exactly as the `final` read did.
    twist.ClearConnections();
    twist.ClearMetadata(TfToken("rigExecReadPhase"));
    twist.Set(40.0);
    CHECK(Near(elbowAt(), atFinal, 1e-9));
}

// Who answers for a bad declaration: the operation that reads the input.
// A weight object's input answers through the mover that names it, which
// is set aside -- and on the retry nothing compiled reads the weight, so
// the rest of the rig compiles. A mover with no rigExec:moves targets is
// inert, and so are its inputs: a declaration there is not resolved, as
// the pulled wire it usually is would make it fail the rig. A space switch
// is no operation the retry sets aside, so a bad declaration on its input
// fails the compile.
static void
TestBadPhaseReaders(const std::string &examplesDir)
{
    const TfToken phaseField("rigExecReadPhase");
    const char *const refusal = "'preceding' names no position";
    // Compiles \p rig on \p stage: whether it compiled, what it set aside,
    // and whether \p expect was among the messages.
    const auto compile = [](const UsdStageRefPtr &stage, const char *rig,
                            const char *expect,
                            std::map<SdfPath, std::string> *skipped) {
        RigExecRigEvaluator evaluator(stage, SdfPath(rig));
        evaluator.cpuParityMode = true;
        std::vector<std::string> errors;
        const bool compiled = evaluator.Compile(&errors);
        bool said = false;
        for (const std::string &e : errors) {
            said |= e.find(expect) != std::string::npos;
        }
        if (!said) {
            std::printf("  expected \"%s\" among %zu compile message(s)\n",
                        expect, errors.size());
            for (const std::string &e : errors) {
                std::printf("    %s\n", e.c_str());
            }
        }
        CHECK(said);
        *skipped = evaluator.GetSkippedOperations();
        return compiled;
    };
    const std::string weights =
        examplesDir + "/../tests/fixtures/computed_weights.usda";
    std::map<SdfPath, std::string> skipped;

    // Ramp is a weight only the Blend constraint reads, through its
    // combine weight.
    {
        const UsdStageRefPtr stage = UsdStage::Open(weights);
        CHECK(stage);
        if (stage) {
            stage->SetEditTarget(stage->GetSessionLayer());
            stage->GetAttributeAtPath(
                     SdfPath("/Asset/Rig/Weights/Ramp.inputs:driver"))
                .SetMetadata(phaseField, std::string("preceding"));
            CHECK(compile(stage, "/Asset/Rig", refusal, &skipped));
            CHECK(skipped.size() == 1 &&
                  skipped.count(SdfPath("/Asset/Rig/Movers/Blend")));
        }
    }

    // The same bad declaration on a mover's own input: set aside wired,
    // not resolved inert.
    for (const bool wired : {true, false}) {
        const UsdStageRefPtr stage = UsdStage::Open(weights);
        CHECK(stage);
        if (!stage) {
            continue;
        }
        stage->SetEditTarget(stage->GetSessionLayer());
        const UsdAttribute other =
            stage->DefinePrim(SdfPath("/Asset/Rig/Channels/Other"),
                              TfToken("Scope"))
                .CreateAttribute(TfToken("rigExec:amount"),
                                 SdfValueTypeNames->Float);
        other.Set(1.0f);
        const SdfPath pulled("/Asset/Rig/Movers/Pulled");
        const UsdPrim mover =
            stage->DefinePrim(pulled, TfToken("RigExecFloatMathMover"));
        CHECK(mover.ApplyAPI(TfToken("RigExecMoverAPI")));
        mover.CreateAttribute(TfToken("rigExec:operation"),
                              SdfValueTypeNames->Token)
            .Set(TfToken("add"));
        const UsdAttribute value = mover.CreateAttribute(
            TfToken("inputs:value"), SdfValueTypeNames->Float);
        value.SetConnections(
            {SdfPath("/Asset/Rig/Channels/Amount.rigExec:amount")});
        value.SetMetadata(phaseField, std::string("preceding"));
        mover.CreateRelationship(TfToken("rigExec:moves"))
            .SetTargets(wired ? SdfPathVector{other.GetPath()}
                              : SdfPathVector{});
        CHECK(compile(stage, "/Asset/Rig",
                      wired ? refusal : "Mover has no moves targets",
                      &skipped));
        if (wired) {
            CHECK(skipped.size() == 1 && skipped.count(pulled));
        } else {
            CHECK(skipped.empty());
        }
    }

    // A space switch's own input.
    {
        const UsdStageRefPtr stage = UsdStage::Open(
            examplesDir + "/../docs/examples/space_switch.usda");
        CHECK(stage);
        if (stage) {
            stage->SetEditTarget(stage->GetSessionLayer());
            const UsdAttribute pick =
                stage->DefinePrim(SdfPath("/SpaceSwitchAsset/Rig/Channels"),
                                  TfToken("Scope"))
                    .CreateAttribute(TfToken("rigExec:pick"),
                                     SdfValueTypeNames->Double);
            pick.Set(1.0);
            const UsdAttribute active =
                stage
                    ->GetPrimAtPath(
                        SdfPath("/SpaceSwitchAsset/Rig/Spaces/HandSpaces"))
                    .CreateAttribute(TfToken("inputs:activeSpace"),
                                     SdfValueTypeNames->Double);
            active.SetConnections({pick.GetPath()});
            active.SetMetadata(phaseField, std::string("preceding"));
            CHECK(!compile(stage, "/SpaceSwitchAsset/Rig", refusal,
                           &skipped));
        }
    }
}

// An attribute an operator names as one it reads is a connected input like
// any under the rig root, wherever it lives: a space switch's active-space
// channel on a prim outside the rig, connected to a pick channel a math
// mover moves from space 0 to space 1, reads the pick's base by default and
// its final value when declared, posing the switched hand exactly as the
// index authored there.
static void
TestOutsideNamedAttributeReadPhase(const std::string &examplesDir)
{
    enum class Wiring { AuthoredBase, AuthoredFinal, Connected, ConnectedFinal };
    const SdfPath hand("/SpaceSwitchAsset/Rig/Controls/Hand");
    const SdfPath dial("/SpaceSwitchAsset/Dials.rigExec:space");
    const auto handAt = [&](Wiring wiring, RigExecEvaluationMode mode,
                            GfVec3d *out, long *applied) {
        const UsdStageRefPtr stage = UsdStage::Open(
            examplesDir + "/../docs/examples/space_switch.usda");
        if (!stage) {
            return false;
        }
        stage->SetEditTarget(stage->GetSessionLayer());
        const UsdAttribute pick =
            stage->DefinePrim(SdfPath("/SpaceSwitchAsset/Rig/Channels"),
                              TfToken("Scope"))
                .CreateAttribute(TfToken("rigExec:pick"),
                                 SdfValueTypeNames->Double);
        pick.Set(0.0);
        const UsdPrim next = stage->DefinePrim(
            SdfPath("/SpaceSwitchAsset/Rig/Movers/NextSpace"),
            TfToken("RigExecFloatMathMover"));
        next.ApplyAPI(TfToken("RigExecMoverAPI"));
        next.CreateAttribute(TfToken("rigExec:operation"),
                             SdfValueTypeNames->Token)
            .Set(TfToken("add"));
        next.CreateAttribute(TfToken("inputs:value"),
                             SdfValueTypeNames->Float)
            .Set(1.0f);
        next.CreateRelationship(TfToken("rigExec:moves"))
            .SetTargets({pick.GetPath()});
        const UsdAttribute space =
            stage->DefinePrim(dial.GetPrimPath(), TfToken("Scope"))
                .CreateAttribute(dial.GetNameToken(),
                                 SdfValueTypeNames->Double);
        if (wiring == Wiring::AuthoredBase) {
            space.Set(0.0);
        } else if (wiring == Wiring::AuthoredFinal) {
            space.Set(1.0);
        } else {
            space.SetConnections({pick.GetPath()});
        }
        if (wiring == Wiring::ConnectedFinal) {
            space.SetMetadata(TfToken("rigExecReadPhase"),
                              std::string("final"));
        }
        stage
            ->GetPrimAtPath(
                SdfPath("/SpaceSwitchAsset/Rig/Spaces/HandSpaces"))
            .GetRelationship(TfToken("rigExec:activeSpaceAttribute"))
            .SetTargets({dial});
        RigExecRigEvaluator evaluator(stage,
                                      SdfPath("/SpaceSwitchAsset/Rig"));
        evaluator.cpuParityMode = true;
        evaluator.SetEvaluationMode(mode);
        if (!evaluator.Compile(nullptr) ||
            !evaluator.GetSkippedOperations().empty()) {
            return false;
        }
        *applied = -1;
        for (const RigExecPhasedConnection &connection :
             evaluator.GetPhasedConnections()) {
            if (connection.consumer == dial) {
                *applied = long(connection.applied);
            }
        }
        const RigExecRigPose pose = evaluator.Evaluate(UsdTimeCode(1012));
        const auto it = pose.controlFrames.find(hand);
        if (!pose.valid || it == pose.controlFrames.end() ||
            it->second.points.empty()) {
            return false;
        }
        *out = GfVec3d(it->second.points[0]);
        return true;
    };
    for (const RigExecEvaluationMode mode :
         {RigExecEvaluationMode::Dynamic, RigExecEvaluationMode::Baked}) {
        GfVec3d base, final, connected, connectedFinal;
        long applied = -1;
        CHECK(handAt(Wiring::AuthoredBase, mode, &base, &applied));
        CHECK(handAt(Wiring::AuthoredFinal, mode, &final, &applied));
        // The cart has moved by frame 1012, so the space shows.
        CHECK(!Near(base, final, 1e-3));
        CHECK(handAt(Wiring::Connected, mode, &connected, &applied));
        CHECK(applied == 0);
        CHECK(Near(connected, base, 1e-12));
        CHECK(handAt(Wiring::ConnectedFinal, mode, &connectedFinal,
                     &applied));
        CHECK(applied == -1);
        CHECK(Near(connectedFinal, final, 1e-12));
    }
}

// A property result that drives a DynamicWeight input must reach both Exec's
// tapped weight packet and the evaluator's independent CPU resolver. Before
// this regression the tap saw driver=1 while _ResolveWeights reread authored
// driver=0, so the otherwise correct deformation was rejected by parity.
static void
TestPropertyMoverDrivesDynamicWeight()
{
    const UsdStageRefPtr stage = UsdStage::CreateInMemory();
    const SdfPath target("/Asset/Geom/P.points");
    const UsdPrim points =
        stage->DefinePrim(SdfPath("/Asset/Geom/P"), TfToken("Points"));
    points.CreateAttribute(TfToken("points"), SdfValueTypeNames->Point3fArray)
        .Set(VtVec3fArray{GfVec3f(0)});
    stage->DefinePrim(SdfPath("/Asset/Rig"), TfToken("RigExecRoot"));
    const UsdPrim joint = stage->DefinePrim(
        SdfPath("/Asset/Rig/Joints/J"), TfToken("RigExecJoint"));
    GfMatrix4d posed(1.0);
    posed.SetTranslate(GfVec3d(10, 0, 0));
    joint.CreateAttribute(TfToken("posed:space"), SdfValueTypeNames->Matrix4d)
        .Set(posed);

    const UsdPrim weight = stage->DefinePrim(
        SdfPath("/Asset/Rig/Weights/W"), TfToken("RigExecDynamicWeight"));
    weight.CreateRelationship(TfToken("rigExec:weightTarget"), false)
        .SetTargets({target});
    weight.CreateAttribute(TfToken("rigExec:representation"),
                           SdfValueTypeNames->Token, false)
        .Set(TfToken("constant"));
    weight.CreateAttribute(TfToken("inputs:driver"),
                           SdfValueTypeNames->Float, false)
        .Set(0.0f);

    const UsdPrim mover = stage->DefinePrim(
        SdfPath("/Asset/Rig/Movers/M"), TfToken("RigExecMatrixMover"));
    CHECK(mover.ApplyAPI(TfToken("RigExecMoverAPI")));
    mover.CreateRelationship(TfToken("rigExec:moves"), false)
        .SetTargets({target});
    mover.CreateRelationship(TfToken("rigExec:transform"), false)
        .SetTargets({joint.GetPath()});
    mover.CreateRelationship(TfToken("rigExec:weightObject"), false)
        .SetTargets({weight.GetPath()});

    const UsdPrim driver = stage->DefinePrim(
        SdfPath("/Asset/Rig/Movers/Driver"),
        TfToken("RigExecFloatMathMover"));
    CHECK(driver.ApplyAPI(TfToken("RigExecMoverAPI")));
    driver.CreateAttribute(TfToken("rigExec:operation"),
                           SdfValueTypeNames->Token, false)
        .Set(TfToken("add"));
    driver.CreateAttribute(TfToken("inputs:value"),
                           SdfValueTypeNames->Float, false)
        .Set(1.0f);
    driver.CreateRelationship(TfToken("rigExec:moves"), false)
        .SetTargets({weight.GetPath().AppendProperty(
            TfToken("inputs:driver"))});

    RigExecRigEvaluator evaluator(stage, SdfPath("/Asset/Rig"));

    evaluator.cpuParityMode = true;
    CHECK(evaluator.Compile(nullptr));
    const RigExecRigPose pose = evaluator.Evaluate(UsdTimeCode::Default());
    CHECK(pose.valid);
    CHECK(pose.moverGraphParityMismatches == 0);
    const auto driven = pose.movedProperties.find(
        weight.GetPath().AppendProperty(TfToken("inputs:driver")));
    CHECK(driven != pose.movedProperties.end());
    if (driven != pose.movedProperties.end()) {
        CHECK(driven->second.Get<float>() == 1.0f);
    }
    const VtVec3fArray moved = EnvelopePoints(pose, target);
    CHECK(moved.size() == 1);
    if (moved.size() == 1) {
        CHECK(moved[0] == GfVec3f(10, 0, 0));
    }
}

// Property-chain dependencies, not path order, decide the prepass order. The
// driver target sorts after /Asset/Channels.value, which is the adversarial
// order that formerly left Consumer at its authored zero envelope.
static void
TestPropertyChainDependencyOrderAndExactEndpoint()
{
    const UsdStageRefPtr stage = UsdStage::CreateInMemory();
    const UsdPrim channels =
        stage->DefinePrim(SdfPath("/Asset/Channels"), TfToken("Scope"));
    const SdfPath valuePath("/Asset/Channels.value");
    channels.CreateAttribute(TfToken("value"), SdfValueTypeNames->Float, true)
        .Set(2.0f);
    stage->DefinePrim(SdfPath("/Asset/Rig"), TfToken("RigExecRoot"));

    const UsdPrim consumer = stage->DefinePrim(
        SdfPath("/Asset/Rig/Movers/Consumer"),
        TfToken("RigExecFloatMathMover"));
    CHECK(consumer.ApplyAPI(TfToken("RigExecMoverAPI")));
    consumer.CreateAttribute(TfToken("rigExec:operation"),
                             SdfValueTypeNames->Token, false)
        .Set(TfToken("add"));
    consumer.CreateAttribute(TfToken("inputs:value"),
                             SdfValueTypeNames->Float, false)
        .Set(8.0f);
    consumer.CreateAttribute(TfToken("inputs:defaultWeight"),
                             SdfValueTypeNames->Float, false)
        .Set(0.0f);
    consumer.CreateRelationship(TfToken("rigExec:moves"), false)
        .SetTargets({valuePath});

    const UsdPrim driver = stage->DefinePrim(
        SdfPath("/Asset/Rig/Movers/Driver"),
        TfToken("RigExecFloatMathMover"));
    CHECK(driver.ApplyAPI(TfToken("RigExecMoverAPI")));
    driver.CreateAttribute(TfToken("rigExec:operation"),
                           SdfValueTypeNames->Token, false)
        .Set(TfToken("add"));
    driver.CreateAttribute(TfToken("inputs:value"),
                           SdfValueTypeNames->Float, false)
        .Set(1.0f);
    driver.CreateRelationship(TfToken("rigExec:moves"), false)
        .SetTargets({consumer.GetPath().AppendProperty(
            TfToken("inputs:defaultWeight"))});

    RigExecRigEvaluator evaluator(stage, SdfPath("/Asset/Rig"));

    evaluator.cpuParityMode = true;
    CHECK(evaluator.Compile(nullptr));
    const RigExecRigPose pose = evaluator.Evaluate(UsdTimeCode::Default());
    CHECK(pose.valid);
    const auto result = pose.movedProperties.find(valuePath);
    CHECK(result != pose.movedProperties.end());
    if (result != pose.movedProperties.end()) {
        CHECK(result->second.Get<float>() == 10.0f);
    }

    // Full envelope selects the candidate directly. Arithmetic lerp used to
    // cancel 1e20 + (1 - 1e20) to zero at w=1.
    channels.GetAttribute(TfToken("value")).Set(1e20f);
    consumer.GetAttribute(TfToken("rigExec:operation")).Set(TfToken("blend"));
    consumer.GetAttribute(TfToken("inputs:value")).Set(1.0f);
    const RigExecRigPose exact = evaluator.Evaluate(UsdTimeCode::Default());
    CHECK(exact.valid);
    const auto exactResult = exact.movedProperties.find(valuePath);
    CHECK(exactResult != exact.movedProperties.end());
    if (exactResult != exact.movedProperties.end()) {
        CHECK(exactResult->second.Get<float>() == 1.0f);
    }
}

static UsdStageRefPtr
MakeConnectedMatrixWeightStage(bool dynamicWeight)
{
    const UsdStageRefPtr stage = UsdStage::CreateInMemory();
    const SdfPath target("/Asset/Geom/P.points");
    const UsdPrim points =
        stage->DefinePrim(SdfPath("/Asset/Geom/P"), TfToken("Points"));
    points.CreateAttribute(TfToken("points"), SdfValueTypeNames->Point3fArray)
        .Set(VtVec3fArray{GfVec3f(0)});
    const UsdPrim inputs =
        stage->DefinePrim(SdfPath("/Asset/Inputs"), TfToken("Scope"));
    inputs.CreateAttribute(TfToken("a"), SdfValueTypeNames->Float, true)
        .Set(0.25f);
    inputs.CreateAttribute(TfToken("b"), SdfValueTypeNames->Float, true)
        .Set(0.75f);
    inputs.CreateAttribute(TfToken("wrong"), SdfValueTypeNames->Int, true)
        .Set(1);
    stage->DefinePrim(SdfPath("/Asset/Rig"), TfToken("RigExecRoot"));
    const UsdPrim joint = stage->DefinePrim(
        SdfPath("/Asset/Rig/Joints/J"), TfToken("RigExecJoint"));
    GfMatrix4d posed(1.0);
    posed.SetTranslate(GfVec3d(10, 0, 0));
    joint.CreateAttribute(TfToken("posed:space"), SdfValueTypeNames->Matrix4d)
        .Set(posed);
    const UsdPrim mover = stage->DefinePrim(
        SdfPath("/Asset/Rig/Movers/M"), TfToken("RigExecMatrixMover"));
    CHECK(mover.ApplyAPI(TfToken("RigExecMoverAPI")));
    mover.CreateRelationship(TfToken("rigExec:moves"), false)
        .SetTargets({target});
    mover.CreateRelationship(TfToken("rigExec:transform"), false)
        .SetTargets({joint.GetPath()});
    if (dynamicWeight) {
        const UsdPrim weight = stage->DefinePrim(
            SdfPath("/Asset/Rig/Weights/W"), TfToken("RigExecDynamicWeight"));
        weight.CreateRelationship(TfToken("rigExec:weightTarget"), false)
            .SetTargets({target});
        weight.CreateAttribute(TfToken("rigExec:representation"),
                               SdfValueTypeNames->Token, false)
            .Set(TfToken("constant"));
        weight.CreateAttribute(TfToken("inputs:driver"),
                               SdfValueTypeNames->Float, false)
            .SetConnections({SdfPath("/Asset/Inputs.a")});
        mover.CreateRelationship(TfToken("rigExec:weightObject"), false)
            .SetTargets({weight.GetPath()});
    } else {
        mover.CreateAttribute(TfToken("inputs:defaultWeight"),
                              SdfValueTypeNames->Float, false)
            .SetConnections({SdfPath("/Asset/Inputs.a")});
    }
    return stage;
}

// Ordinary USD scalar connections are the same input authority as a property
// mover override, and their provider identity is binding-epoch state.
static void
TestConnectedWeightsRewireAndValidate()
{
    for (const bool dynamicWeight : {false, true}) {
        const UsdStageRefPtr stage =
            MakeConnectedMatrixWeightStage(dynamicWeight);
        const SdfPath inputPath = dynamicWeight
            ? SdfPath("/Asset/Rig/Weights/W.inputs:driver")
            : SdfPath("/Asset/Rig/Movers/M.inputs:defaultWeight");
        RigExecRigEvaluator evaluator(stage, SdfPath("/Asset/Rig"));
        evaluator.cpuParityMode = true;
        CHECK(evaluator.Compile(nullptr));
        const RigExecRigPose first =
            evaluator.Evaluate(UsdTimeCode::Default());
        CHECK(first.valid);
        CHECK(first.moverGraphParityMismatches == 0);
        VtVec3fArray moved =
            EnvelopePoints(first, SdfPath("/Asset/Geom/P.points"));
        CHECK(moved.size() == 1);
        if (moved.size() == 1) {
            CHECK(Near(GfVec3d(moved[0]), GfVec3d(2.5, 0, 0)));
        }
        const size_t before = evaluator.GetBindingEpochDigest();
        stage->GetAttributeAtPath(inputPath).SetConnections(
            {SdfPath("/Asset/Inputs.b")});
        const RigExecRigPose rewired =
            evaluator.Evaluate(UsdTimeCode::Default());
        CHECK(rewired.valid);
        CHECK(rewired.moverGraphParityMismatches == 0);
        CHECK(evaluator.GetBindingEpochDigest() != before);
        moved = EnvelopePoints(rewired, SdfPath("/Asset/Geom/P.points"));
        CHECK(moved.size() == 1);
        if (moved.size() == 1) {
            CHECK(Near(GfVec3d(moved[0]), GfVec3d(7.5, 0, 0)));
        }

        auto rejects = [&](const SdfPathVector &sources) {
            const UsdStageRefPtr bad =
                MakeConnectedMatrixWeightStage(dynamicWeight);
            bad->GetAttributeAtPath(inputPath).SetConnections(sources);
            RigExecRigEvaluator badEvaluator(
                bad, SdfPath("/Asset/Rig"));
            std::vector<std::string> errors;
            // Refused: set aside with a warning while the rig compiles on,
            // or -- when the mover is the rig's only output, so nothing is
            // left once it goes -- the rig fails, still saying why.
            const bool compiled = badEvaluator.Compile(&errors);
            const bool refused =
                !compiled || badEvaluator.GetSkippedOperations().count(
                                 SdfPath("/Asset/Rig/Movers/M"));
            return refused && !errors.empty();
        };
        CHECK(rejects(
            {SdfPath("/Asset/Inputs.a"), SdfPath("/Asset/Inputs.b")}));
        CHECK(rejects({SdfPath("/Asset/Inputs.wrong")}));
        CHECK(rejects({SdfPath("/Asset/Inputs.missing")}));
    }
}

// Frozen weight descriptor mistakes skip their mover during compile, never a later mover
// application. Dense array length is epoch identity so a valid-to-invalid edit
// is caught without an explicit Compile call.
static void
TestWeightDescriptorValidationAndEpoch()
{
    auto makeStage = [](const VtFloatArray &values, bool sampledPoints = false) {
        const UsdStageRefPtr stage = UsdStage::CreateInMemory();
        const SdfPath target("/Asset/Geom/P.points");
        const UsdPrim points =
            stage->DefinePrim(SdfPath("/Asset/Geom/P"), TfToken("Points"));
        const UsdAttribute pointsAttr = points.CreateAttribute(
            TfToken("points"), SdfValueTypeNames->Point3fArray);
        const VtVec3fArray base = {
            GfVec3f(0), GfVec3f(1, 0, 0), GfVec3f(2, 0, 0)};
        pointsAttr.Set(
            base, sampledPoints ? UsdTimeCode(1) : UsdTimeCode::Default());
        stage->DefinePrim(SdfPath("/Asset/Rig"), TfToken("RigExecRoot"));
        const UsdPrim joint = stage->DefinePrim(
            SdfPath("/Asset/Rig/Joints/J"), TfToken("RigExecJoint"));
        GfMatrix4d posed(1.0);
        posed.SetTranslate(GfVec3d(1, 0, 0));
        joint.CreateAttribute(
                 TfToken("posed:space"), SdfValueTypeNames->Matrix4d)
            .Set(posed);
        const UsdPrim weight = stage->DefinePrim(
            SdfPath("/Asset/Rig/Weights/W"), TfToken("RigExecStaticWeight"));
        weight.CreateRelationship(TfToken("rigExec:weightTarget"), false)
            .SetTargets({target});
        weight.CreateAttribute(TfToken("rigExec:representation"),
                               SdfValueTypeNames->Token, false)
            .Set(TfToken("dense"));
        weight.CreateAttribute(TfToken("rigExec:values"),
                               SdfValueTypeNames->FloatArray, false)
            .Set(values);
        weight.CreateAttribute(TfToken("rigExec:defaultWeight"),
                               SdfValueTypeNames->Float, false)
            .Set(0.0f);
        const UsdPrim mover = stage->DefinePrim(
            SdfPath("/Asset/Rig/Movers/M"), TfToken("RigExecMatrixMover"));
        CHECK(mover.ApplyAPI(TfToken("RigExecMoverAPI")));
        mover.CreateRelationship(TfToken("rigExec:moves"), false)
            .SetTargets({target});
        mover.CreateRelationship(TfToken("rigExec:transform"), false)
            .SetTargets({joint.GetPath()});
        mover.CreateRelationship(TfToken("rigExec:weightObject"), false)
            .SetTargets({weight.GetPath()});
        return stage;
    };

    {
        const UsdStageRefPtr bad = makeStage(VtFloatArray{1, 1});
        RigExecRigEvaluator evaluator(bad, SdfPath("/Asset/Rig"));
        evaluator.cpuParityMode = true;
        CHECK(SkipsOperations(evaluator, {SdfPath("/Asset/Rig/Movers/M")}, nullptr));
        CHECK(evaluator.Evaluate(UsdTimeCode::Default()).valid);
    }
    {
        // Numeric packet contents are value state, unlike the dense array's
        // descriptor cardinality.  A strict violation therefore compiles and
        // fails just this mover revision back to the preceding points.
        const UsdStageRefPtr badValue =
            makeStage(VtFloatArray{1, 2, 1});
        RigExecRigEvaluator evaluator(badValue, SdfPath("/Asset/Rig"));
        evaluator.cpuParityMode = true;
        CHECK(evaluator.Compile(nullptr));
        const RigExecRigPose pose =
            evaluator.Evaluate(UsdTimeCode::Default());
        CHECK(pose.valid);
        CHECK(EnvelopePoints(pose, SdfPath("/Asset/Geom/P.points")) ==
              VtVec3fArray({
                  GfVec3f(0), GfVec3f(1, 0, 0), GfVec3f(2, 0, 0)}));
    }
    {
        // A points property authored only at a time sample is a valid domain;
        // the epoch freezes its sampled cardinality instead of requiring a
        // default opinion.
        const UsdStageRefPtr sampled =
            makeStage(VtFloatArray{1, 1, 1}, true);
        RigExecRigEvaluator evaluator(sampled, SdfPath("/Asset/Rig"));
        evaluator.cpuParityMode = true;
        CHECK(evaluator.Compile(nullptr));
        CHECK(evaluator.Evaluate(UsdTimeCode(1)).valid);
    }
    {
        const UsdStageRefPtr stage = makeStage(VtFloatArray{1, 1, 1});
        RigExecRigEvaluator evaluator(stage, SdfPath("/Asset/Rig"));
        evaluator.cpuParityMode = true;
        CHECK(evaluator.Compile(nullptr));
        CHECK(evaluator.Evaluate(UsdTimeCode::Default()).valid);
        stage->GetAttributeAtPath(
                 SdfPath("/Asset/Rig/Weights/W.rigExec:values"))
            .Set(VtFloatArray{1, 1});
        CHECK(evaluator.Evaluate(UsdTimeCode::Default()).valid);
        CHECK(evaluator.GetSkippedOperations().count(SdfPath("/Asset/Rig/Movers/M")) == 1);
        stage->GetAttributeAtPath(
                 SdfPath("/Asset/Rig/Weights/W.rigExec:values"))
            .Set(VtFloatArray{1, 1, 1});
        CHECK(evaluator.Evaluate(UsdTimeCode::Default()).valid);
        CHECK(evaluator.GetSkippedOperations().empty());

        const UsdAttribute points = stage->GetAttributeAtPath(
            SdfPath("/Asset/Geom/P.points"));
        points.Set(VtVec3fArray{GfVec3f(0), GfVec3f(1, 0, 0)});
        CHECK(evaluator.Evaluate(UsdTimeCode::Default()).valid);
        CHECK(evaluator.GetSkippedOperations().count(SdfPath("/Asset/Rig/Movers/M")) == 1);
        points.Set(VtVec3fArray{
            GfVec3f(0), GfVec3f(1, 0, 0), GfVec3f(2, 0, 0)});
        CHECK(evaluator.Evaluate(UsdTimeCode::Default()).valid);
        CHECK(evaluator.GetSkippedOperations().empty());
    }
    {
        const UsdStageRefPtr stage = makeStage(VtFloatArray{1, 1, 1});
        const UsdPrim inputs =
            stage->DefinePrim(SdfPath("/Asset/Inputs"), TfToken("Scope"));
        inputs.CreateAttribute(TfToken("w"), SdfValueTypeNames->Float, true)
            .Set(1.0f);
        stage->GetAttributeAtPath(
                 SdfPath("/Asset/Rig/Weights/W.rigExec:defaultWeight"))
            .SetConnections({SdfPath("/Asset/Inputs.w")});
        RigExecRigEvaluator evaluator(stage, SdfPath("/Asset/Rig"));
        evaluator.cpuParityMode = true;
        CHECK(SkipsOperations(evaluator, {SdfPath("/Asset/Rig/Movers/M")}, nullptr));
        CHECK(evaluator.Evaluate(UsdTimeCode::Default()).valid);
    }
}

// Synthesized normals/extent maintenance has no authored mover. Custom
// lookalike attributes on the owner mesh must not disable or envelope it.
static void
TestDerivedMaintenanceIgnoresOwnerLookalikes(
    const std::string &examplesDir)
{
    auto evaluate = [&](bool authorLookalikes) {
        const UsdStageRefPtr stage =
            UsdStage::Open(examplesDir + "/ArmRig.usda");
        if (!stage) {
            return RigExecRigPose();
        }
        if (authorLookalikes) {
            stage->SetEditTarget(stage->GetSessionLayer());
            const UsdPrim body =
                stage->GetPrimAtPath(SdfPath("/ArmAsset/Geom/ArmBody"));
            body.CreateAttribute(TfToken("inputs:defaultWeight"),
                                 SdfValueTypeNames->Float, true)
                .Set(0.0f);
            body.CreateAttribute(TfToken("inputs:enabled"),
                                 SdfValueTypeNames->Bool, true)
                .Set(false);
            const UsdPrim trapWeight = stage->DefinePrim(
                SdfPath("/ArmAsset/Rig/Weights/DerivedTrap"),
                TfToken("RigExecStaticWeight"));
            trapWeight.CreateRelationship(
                          TfToken("rigExec:weightTarget"), false)
                .SetTargets({SdfPath("/ArmAsset/Geom/ArmBody.points")});
            trapWeight.CreateAttribute(TfToken("rigExec:representation"),
                                       SdfValueTypeNames->Token, false)
                .Set(TfToken("constant"));
            trapWeight.CreateAttribute(TfToken("rigExec:defaultWeight"),
                                       SdfValueTypeNames->Float, false)
                .Set(0.0f);
            body.CreateRelationship(TfToken("rigExec:weightObject"), false)
                .SetTargets({trapWeight.GetPath()});
        }
        RigExecRigEvaluator evaluator(stage, SdfPath("/ArmAsset/Rig"));
        evaluator.cpuParityMode = true;
        if (!evaluator.Compile(nullptr)) {
            return RigExecRigPose();
        }
        return evaluator.Evaluate(UsdTimeCode::Default());
    };
    const RigExecRigPose baseline = evaluate(false);
    const RigExecRigPose lookalikes = evaluate(true);
    CHECK(baseline.valid);
    CHECK(lookalikes.valid);
    for (const char *property : {"normals", "extent"}) {
        const SdfPath path = SdfPath("/ArmAsset/Geom/ArmBody")
                                 .AppendProperty(TfToken(property));
        const auto expected = baseline.movedProperties.find(path);
        const auto actual = lookalikes.movedProperties.find(path);
        CHECK(expected != baseline.movedProperties.end());
        CHECK(actual != lookalikes.movedProperties.end());
        if (expected != baseline.movedProperties.end() &&
            actual != lookalikes.movedProperties.end()) {
            CHECK(expected->second == actual->second);
        }
    }
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

    evaluator.cpuParityMode = true;
    std::vector<std::string> errors;
    CHECK(!evaluator.Compile(&errors));
    bool sawOutputsError = false;
    for (const std::string &e : errors) {
        sawOutputsError |= e.find("publishes no outputs") != std::string::npos;
    }
    CHECK(sawOutputsError);
}

// The codeless schema's resource directory.
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
    TestControlAvarScaleDrivesMatrixMover();
    TestMatrixMoverUniversalEnvelope();
    TestSkinMoverLinearBlend();
    TestSkinMoverDualQuaternion();
    TestBlendShapeUniversalEnvelope();
    TestSmoothMoverUniversalEnvelope();
    TestPropertyMoverUniversalEnvelope();
    TestWeightTargetMismatchFailsCompile(examplesDir);
    TestDerivedTargetAndFanoutRejected(examplesDir);
    TestNonMeshAuthoredNormalsRejected(examplesDir);
    TestViewFreeValidation(examplesDir);
    TestImplicitJointDiscovery(examplesDir);
    TestMoverGraphParity(examplesDir);
    TestSolverCycleRejected(examplesDir);
    TestPureSolverToSolverCycleRejected(examplesDir);
    TestAimConstraintRevisesJointFrame(examplesDir);
    TestShippedTwoBoneIksRecalibrateOnRestEdit(examplesDir);
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
    TestConnectionReadPhasesSelectRevision(examplesDir);
    TestBadConnectionReadPhasesRejected(examplesDir);
    TestConnectionReadPhaseOnSolverInput(examplesDir);
    TestDefaultReadPhaseAndHeadWins();
    TestPhasedReadDragRules();
    TestTargetDragEqualsRelease();
    TestBlinkDragEqualsRelease();
    TestConnectionReadPhaseOnConstraintAndAvar(examplesDir);
    TestUndeclaredSkipsCompileCleanly();
    TestConstraintRewireIsStructural(examplesDir);
    TestBadPhaseReaders(examplesDir);
    TestOutsideNamedAttributeReadPhase(examplesDir);
    TestPointChainConsumesPrecedingRevision(examplesDir);
    TestPropertyMoverReachesStaticPacketReads(examplesDir);
    TestPropertyMoverDrivesDynamicWeight();
    TestPropertyChainDependencyOrderAndExactEndpoint();
    TestConnectedWeightsRewireAndValidate();
    TestWeightDescriptorValidationAndEpoch();
    TestDerivedMaintenanceIgnoresOwnerLookalikes(examplesDir);
    TestDerivedMaintenanceDeferral();

    if (failures) {
        std::printf("%d FAILURE(S)\n", failures);
        return 1;
    }
    std::printf("testRigExecArm: all tests passed\n");
    return 0;
}
