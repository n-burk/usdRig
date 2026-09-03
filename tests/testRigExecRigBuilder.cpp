// Native contract tests for the coarse, schema-backed RigExec builder.
#include "rigExecRigging/rigBuilder.h"

#include "rigExecMath/avarScale.h"

#include "pxr/base/gf/vec3d.h"
#include "pxr/base/plug/registry.h"
#include "pxr/base/tf/pathUtils.h"
#include "pxr/usd/usd/attribute.h"
#include "pxr/usd/usd/relationship.h"
#include "pxr/usd/usd/stage.h"

#include <cstdio>
#include <exception>
#include <limits>
#include <string>
#include <utility>
#include <vector>

PXR_NAMESPACE_USING_DIRECTIVE

using namespace rigExec;

static int failures = 0;

#define CHECK(cond)                                                        \
    do {                                                                   \
        if (!(cond)) {                                                     \
            ++failures;                                                    \
            std::printf("FAIL %s:%d: %s\n", __FILE__, __LINE__, #cond);    \
        }                                                                  \
    } while (0)

template <class Fn>
static bool
Throws(Fn &&fn)
{
    try {
        std::forward<Fn>(fn)();
    } catch (const std::exception &) {
        return true;
    }
    return false;
}

static SdfPathVector
Targets(const UsdPrim &prim, const char *name)
{
    SdfPathVector result;
    const UsdRelationship relationship = prim.GetRelationship(TfToken(name));
    if (relationship) {
        relationship.GetTargets(&result);
    }
    return result;
}

int
main()
{
#ifdef RIGEXEC_SCHEMA_RESOURCE_DIR
    const std::string resources = TfAbsPath(RIGEXEC_SCHEMA_RESOURCE_DIR);
#else
    std::printf("FATAL: RIGEXEC_SCHEMA_RESOURCE_DIR is not configured\n");
    return 2;
#endif
    if (PlugRegistry::GetInstance().RegisterPlugins(resources).empty()) {
        std::printf("FATAL: no schema plugin found at %s\n", resources.c_str());
        return 2;
    }

    const UsdStageRefPtr stage = UsdStage::CreateInMemory();
    UsdPrim data = stage->DefinePrim(SdfPath("/Data"), TfToken("Xform"));
    data.CreateAttribute(TfToken("value"), SdfValueTypeNames->Float).Set(1.f);

    RigExecRigBuilder builder = RigExecRigBuilder::Create(
        stage, SdfPath("/Rig"), TfToken("Character"));
    RigExecControlHandle rootControl = builder.AddControl("RootControl");
    RigExecControlHandle endControl = builder.AddControl("EndControl");
    RigExecControlHandle poleControl = builder.AddControl("PoleControl");
    CHECK(rootControl.GetPrim().HasAPI(TfToken("RigExecControlAPI")));
    CHECK(rootControl.GetPrim().HasAPI(TfToken("NodeGraphNodeAPI")));
    CHECK(Throws([&] { builder.AddControl("RootControl"); }));
    CHECK(rootControl.IsValid());

    RigExecJointHandle rootJoint = builder.AddJoint("RootJoint");
    RigExecJointHandle middleJoint =
        builder.AddJoint("MiddleJoint", GfMatrix4d(), &rootJoint);
    RigExecJointHandle endJoint =
        builder.AddJoint("EndJoint", GfMatrix4d(), &middleJoint);

    // Scale is part of the same schema-backed avar surface on controls and
    // joints. The typed setters must author the declared double properties,
    // never custom lookalikes.
    rootControl.SetAvarScale(2.0, 3.0, 4.0);
    rootJoint.SetAvarScale(0.5, 1.5, 2.5);
    auto checkScale = [](const UsdPrim &prim, const GfVec3d &expected) {
        for (int axis = 0; axis < 3; ++axis) {
            const char *names[] = {"avars:sx", "avars:sy", "avars:sz"};
            const UsdAttribute attr = prim.GetAttribute(TfToken(names[axis]));
            double value = 0.0;
            CHECK(attr && !attr.IsCustom());
            CHECK(attr.GetTypeName() == SdfValueTypeNames->Double);
            CHECK(attr.Get(&value));
            CHECK(value == expected[axis]);
        }
    };
    checkScale(rootControl.GetPrim(), GfVec3d(2.0, 3.0, 4.0));
    checkScale(rootJoint.GetPrim(), GfVec3d(0.5, 1.5, 2.5));

    rootControl.SetAvarScale(
        0.0, -0.0, -0.5 * RigExecAvarScaleFloor);
    checkScale(
        rootControl.GetPrim(),
        GfVec3d(RigExecAvarScaleFloor, -RigExecAvarScaleFloor,
                -RigExecAvarScaleFloor));

    // Validate the complete triplet before writing any component. A
    // non-finite middle value must not leave sx partially updated.
    CHECK(Throws([&] {
        rootJoint.SetAvarScale(
            9.0, std::numeric_limits<double>::infinity(), 8.0);
    }));
    checkScale(rootJoint.GetPrim(), GfVec3d(0.5, 1.5, 2.5));

    RigExecControlHandle removedScaleControl =
        builder.AddControl("RemovedScaleControl");
    stage->RemovePrim(removedScaleControl.GetPath());
    CHECK(Throws([&] {
        removedScaleControl.SetAvarScale(7.0, 8.0, 9.0);
    }));
    CHECK(!stage->GetPrimAtPath(removedScaleControl.GetPath()));

    RigExecFkChainHandle fk = builder.AddFkChain("Fk");
    fk.SetControls(std::vector<RigExecControlHandle>{
        rootControl, endControl});
    fk.SetJoints(std::vector<RigExecJointHandle>{rootJoint, endJoint});
    CHECK(Targets(fk.GetPrim(), "rigExec:controls") == SdfPathVector({
        rootControl.GetPath(), endControl.GetPath()}));
    CHECK(Targets(fk.GetPrim(), "rigExec:joints") == SdfPathVector({
        rootJoint.GetPath(), endJoint.GetPath()}));
    CHECK(Throws([&] {
        fk.SetControls(SdfPathVector{rootJoint.GetPath()});
    }));

    const UsdStageRefPtr otherStage = UsdStage::CreateInMemory();
    RigExecControlHandle otherControl = RigExecRigBuilder::Create(
        otherStage, SdfPath("/OtherRig")).AddControl("Other");
    CHECK(Throws([&] {
        fk.SetControls(std::vector<RigExecControlHandle>{otherControl});
    }));

    RigExecTwoBoneIkHandle twoBone = builder.AddTwoBoneIk(
        "TwoBone", rootControl.GetPath(), endControl.GetPath(),
        poleControl.GetPath());
    CHECK(Targets(twoBone.GetPrim(), "rigExec:rootControl") ==
          SdfPathVector({rootControl.GetPath()}));
    twoBone.SetUpperLengthOffset(0.5);
    twoBone.SetLowerLengthOffset(-0.25);
    double upperOffset = 0.0, lowerOffset = 0.0;
    CHECK(twoBone.GetPrim()
              .GetAttribute(TfToken("rigExec:upperLengthOffset"))
              .Get(&upperOffset));
    CHECK(upperOffset == 0.5);
    CHECK(twoBone.GetPrim()
              .GetAttribute(TfToken("rigExec:lowerLengthOffset"))
              .Get(&lowerOffset));
    CHECK(lowerOffset == -0.25);
    CHECK(Throws([&] {
        builder.AddTwoBoneIk(
            "BadTwoBone", rootJoint.GetPath(), endControl.GetPath(), {});
    }));
    CHECK(!stage->GetPrimAtPath(SdfPath("/Rig/Solvers/BadTwoBone")));

    RigExecTwistDistributionHandle twist = builder.AddTwistDistribution(
        "Twist", rootControl.GetPath(), endControl.GetPath(), 2);
    twist.SetJoints(SdfPathVector{rootJoint.GetPath(), endJoint.GetPath()});
    twist.SetJointElements({0, 1});
    CHECK(twist.GetPrim()
              .GetAttribute(TfToken("rigExec:jointElements"))
              .HasAuthoredValueOpinion());
    twist.SetJoints(SdfPathVector{endJoint.GetPath()});
    CHECK(!twist.GetPrim()
               .GetAttribute(TfToken("rigExec:jointElements"))
               .HasAuthoredValueOpinion());

    RigExecStaticWeightHandle sparse = builder.AddStaticWeight(
        "Sparse", SdfPath("/Data.value"), {0.25f, 0.75f}, {0, 2});
    CHECK(Throws([&] { sparse.SetIndices({0}); }));
    sparse.SetSparseValues({0.5f}, {1});
    VtIntArray indices;
    CHECK(sparse.GetPrim()
              .GetAttribute(TfToken("rigExec:indices"))
              .Get(&indices));
    CHECK(indices == VtIntArray({1}));
    sparse.SetValues({0.1f, 0.2f});
    CHECK(!sparse.GetPrim()
               .GetAttribute(TfToken("rigExec:indices"))
               .HasAuthoredValueOpinion());

    RigExecMoverChain chain =
        builder.NewMoverChain("Stack", SdfPath("/Data.value"));
    RigExecFloatMathMoverHandle parent =
        chain.AddFloatMathMover("Parent", TfToken("add"), 10.f);
    RigExecFloatMathMoverHandle below =
        chain.AddFloatMathMover("Below", TfToken("add"), 1.f);
    RigExecFloatMathMoverHandle nested = chain.Under(parent)
        .AddFloatMathMover("Nested", TfToken("multiply"), 2.f);
    CHECK(nested.GetPath() == parent.GetPath().AppendChild(TfToken("Nested")));
    CHECK(below.GetPath().GetParentPath() == chain.GetScopePath());
    CHECK(parent.GetPrim().HasAPI(TfToken("RigExecMoverAPI")));
    CHECK(parent.GetPrim().HasAPI(TfToken("NodeGraphNodeAPI")));

    // Movers that formerly required a per-type strength argument now share
    // MoverAPI's full-strength default and can be constructed without one.
    const UsdPrim curvenet =
        stage->DefinePrim(SdfPath("/Net"), TfToken("RigExecCurvenet"));
    RigExecMoverChain defaultEnvelopeChain =
        builder.NewMoverChain("DefaultEnvelopes", SdfPath("/Data.value"));
    const RigExecSmoothMoverHandle smooth =
        defaultEnvelopeChain.AddSmoothMover("Smooth");
    const RigExecVolumeCorrectMoverHandle volume =
        defaultEnvelopeChain.AddVolumeCorrectMover("Volume");
    const RigExecCurvenetMoverHandle profile =
        defaultEnvelopeChain.AddCurvenetMover("Profile", curvenet.GetPath());
    for (const UsdPrim &mover :
         {smooth.GetPrim(), volume.GetPrim(), profile.GetPrim()}) {
        float defaultWeight = 0.0f;
        CHECK(mover.GetAttribute(TfToken("inputs:defaultWeight"))
                  .Get(&defaultWeight));
        CHECK(defaultWeight == 1.0f);
    }

    // Invalid common envelopes are rejected before authoring starts.  Every
    // affected convenience path must preserve the builder's atomic-add
    // contract rather than leaving a partially configured mover behind.
    const float nan = std::numeric_limits<float>::quiet_NaN();
    CHECK(Throws([&] {
        defaultEnvelopeChain.AddSmoothMover("BadSmoothWeight", nan);
    }));
    CHECK(Throws([&] {
        defaultEnvelopeChain.AddVolumeCorrectMover("BadVolumeWeight", 2.0f);
    }));
    CHECK(Throws([&] {
        defaultEnvelopeChain.AddCurvenetMover(
            "BadCurvenetWeight", curvenet.GetPath(), nan);
    }));
    CHECK(Throws([&] {
        defaultEnvelopeChain.AddFloatMathMover(
            "BadFloatWeight", TfToken("add"), 1.0f, {}, nan);
    }));
    CHECK(Throws([&] {
        defaultEnvelopeChain.AddVec3fMathMover(
            "BadVecWeight", TfToken("add"), GfVec3f(1.0f), {}, nan);
    }));
    CHECK(Throws([&] {
        defaultEnvelopeChain.AddMatrixMathMover(
            "BadMatrixWeight", TfToken("multiply"), GfMatrix4d(1.0), {}, nan);
    }));
    for (const char *name : {
             "BadSmoothWeight", "BadVolumeWeight", "BadCurvenetWeight",
             "BadFloatWeight", "BadVecWeight", "BadMatrixWeight"}) {
        CHECK(!stage->GetPrimAtPath(
            defaultEnvelopeChain.GetScopePath().AppendChild(TfToken(name))));
    }

    RigExecParentConstraintHandle parentConstraint =
        chain.AddParentConstraint("ParentConstraint", rootJoint.GetPath());
    parentConstraint.SetSources(
        {rootControl.GetPath(), endControl.GetPath()});
    parentConstraint.SetTranslationOffsets(
        {GfVec3d(1, 2, 3), GfVec3d(4, 5, 6)});
    parentConstraint.SetRotationOffsets(
        {GfVec3d(7, 8, 9), GfVec3d(10, 11, 12)});
    parentConstraint.SetSources({poleControl.GetPath()});
    CHECK(!parentConstraint.GetPrim()
               .GetAttribute(TfToken("inputs:translationOffsets"))
               .HasAuthoredValueOpinion());
    CHECK(!parentConstraint.GetPrim()
               .GetAttribute(TfToken("inputs:rotationOffsets"))
               .HasAuthoredValueOpinion());

    RigExecSingleChainIkConstraintHandle ik =
        builder.NewMoverChain("Ik").AddSingleChainIkConstraint(
            "Solve", rootJoint.GetPath(), endJoint.GetPath(),
            endControl.GetPath(), {poleControl.GetPath()});
    CHECK(Targets(ik.GetPrim(), "rigExec:moves") == SdfPathVector({
        rootJoint.GetPath(), middleJoint.GetPath(), endJoint.GetPath()}));

    if (failures == 0) {
        std::printf("OK: native RigExec builder contract\n");
    }
    return failures == 0 ? 0 : 1;
}
