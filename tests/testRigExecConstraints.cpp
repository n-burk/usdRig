//
// FBX-equivalent constraint schema and evaluator conformance.
//
#include "rigExec/rigEvaluator.h"
#include "rigExec/frameExtraction.h"
#include "rigExecMath/pointFrame.h"

#include "pxr/base/gf/rotation.h"
#include "pxr/base/plug/registry.h"
#include "pxr/usd/sdf/types.h"
#include "pxr/usd/usd/primDefinition.h"
#include "pxr/usd/usd/schemaRegistry.h"
#include "pxr/usd/usd/stage.h"
#include "pxr/usd/usdGeom/xform.h"

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <limits>
#include <string>
#include <vector>

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
Near(const GfVec3d &a, const GfVec3d &b, double tolerance = 1e-5)
{
    return (a - b).GetLength() <= tolerance;
}

static GfMatrix4d
Matrix(const GfVec3d &translation = GfVec3d(0),
       const GfRotation &rotation = GfRotation(),
       const GfVec3d &scale = GfVec3d(1))
{
    GfMatrix4d scaling(1.0);
    scaling.SetScale(scale);
    return scaling * GfMatrix4d(rotation, translation);
}

static UsdPrim
MakeXform(const UsdStageRefPtr &stage, const SdfPath &path,
          const GfMatrix4d &matrix)
{
    const UsdGeomXform xform = UsdGeomXform::Define(stage, path);
    xform.MakeMatrixXform().Set(matrix);
    return xform.GetPrim();
}

static UsdPrim
MakeConstraint(const UsdStageRefPtr &stage, const char *name,
               const char *type, const SdfPathVector &targets)
{
    const UsdPrim prim = stage->DefinePrim(
        SdfPath(std::string("/Asset/Rig/Movers/") + name), TfToken(type));
    CHECK(prim);
    CHECK(prim.ApplyAPI(TfToken("RigExecMoverAPI")));
    prim.CreateRelationship(TfToken("rigExec:moves")).SetTargets(targets);
    return prim;
}

static bool
HasProperty(const UsdPrimDefinition *definition, const char *name)
{
    if (!definition) {
        return false;
    }
    const TfToken token(name);
    const TfTokenVector &properties = definition->GetPropertyNames();
    return std::find(properties.begin(), properties.end(), token) !=
           properties.end();
}

static void
TestSchemaSurface()
{
    const UsdSchemaRegistry &registry = UsdSchemaRegistry::GetInstance();
    const char *types[] = {
        "RigExecAimConstraint", "RigExecPositionConstraint",
        "RigExecRotationConstraint", "RigExecScaleConstraint",
        "RigExecParentConstraint", "RigExecSingleChainIkConstraint",
        "RigExecCustomConstraint"};
    for (const char *type : types) {
        const UsdPrimDefinition *definition =
            registry.FindConcretePrimDefinition(TfToken(type));
        CHECK(definition);
        CHECK(HasProperty(definition, "inputs:weight"));
        CHECK(HasProperty(definition, "rigExec:locked"));
        if (std::string(type) != "RigExecSingleChainIkConstraint" &&
            std::string(type) != "RigExecCustomConstraint") {
            CHECK(HasProperty(definition, "rigExec:sources"));
            CHECK(HasProperty(definition, "inputs:sourceWeights"));
        }
        float weight = 0;
        CHECK(definition && definition->GetAttributeFallbackValue(
                                TfToken("inputs:weight"), &weight));
        CHECK(std::abs(weight - 1.0f) < 1e-7f);
    }
    const UsdPrimDefinition *custom = registry.FindConcretePrimDefinition(
        TfToken("RigExecCustomConstraint"));
    CHECK(custom && !HasProperty(custom, "rigExec:sources"));

    struct PropertyCase {
        const char *type;
        const char *property;
    };
    const PropertyCase properties[] = {
        {"RigExecAimConstraint", "inputs:aimVector"},
        {"RigExecAimConstraint", "inputs:worldUpVector"},
        {"RigExecAimConstraint", "rigExec:worldUpType"},
        {"RigExecPositionConstraint", "inputs:translationOffset"},
        {"RigExecRotationConstraint", "inputs:rotationOffset"},
        {"RigExecScaleConstraint", "inputs:scaleOffset"},
        {"RigExecParentConstraint", "inputs:translationOffsets"},
        {"RigExecParentConstraint", "inputs:rotationOffsets"},
        {"RigExecSingleChainIkConstraint", "rigExec:firstJoint"},
        {"RigExecSingleChainIkConstraint", "rigExec:endJoint"},
        {"RigExecSingleChainIkConstraint", "rigExec:effector"},
        {"RigExecSingleChainIkConstraint", "rigExec:solverMode"},
        {"RigExecSingleChainIkConstraint", "rigExec:poleVectorMode"},
        {"RigExecSingleChainIkConstraint", "rigExec:evaluationMode"},
    };
    for (const PropertyCase &entry : properties) {
        CHECK(HasProperty(registry.FindConcretePrimDefinition(
                              TfToken(entry.type)),
                          entry.property));
    }

    const auto definition = [&](const char *type) {
        return registry.FindConcretePrimDefinition(TfToken(type));
    };
    TfToken token;
    CHECK(definition("RigExecAimConstraint")
              ->GetAttributeFallbackValue(TfToken("rigExec:worldUpType"),
                                          &token));
    CHECK(token == "none");
    CHECK(definition("RigExecRotationConstraint")
              ->GetAttributeFallbackValue(TfToken("rigExec:rotationOrder"),
                                          &token));
    CHECK(token == "XYZ");
    CHECK(definition("RigExecSingleChainIkConstraint")
              ->GetAttributeFallbackValue(TfToken("rigExec:solverMode"),
                                          &token));
    CHECK(token == "rotatePlane");
    CHECK(definition("RigExecSingleChainIkConstraint")
              ->GetAttributeFallbackValue(TfToken("rigExec:poleVectorMode"),
                                          &token));
    CHECK(token == "vector");
    CHECK(definition("RigExecSingleChainIkConstraint")
              ->GetAttributeFallbackValue(TfToken("rigExec:evaluationMode"),
                                          &token));
    CHECK(token == "neverTS");

    GfVec3d vector;
    CHECK(definition("RigExecAimConstraint")
              ->GetAttributeFallbackValue(TfToken("inputs:aimVector"),
                                          &vector));
    CHECK(vector == GfVec3d(1, 0, 0));
    CHECK(definition("RigExecSingleChainIkConstraint")
              ->GetAttributeFallbackValue(TfToken("inputs:poleVector"),
                                          &vector));
    CHECK(vector == GfVec3d(0, 1, 0));

    bool flag = true;
    CHECK(definition("RigExecParentConstraint")
              ->GetAttributeFallbackValue(TfToken("inputs:affectScaleX"),
                                          &flag));
    CHECK(!flag);
    CHECK(definition("RigExecParentConstraint")
              ->GetAttributeFallbackValue(
                  TfToken("inputs:affectTranslationX"), &flag));
    CHECK(flag);
}

static UsdStageRefPtr
BuildConstraintStage(UsdPrim *positionOut)
{
    const UsdStageRefPtr stage = UsdStage::CreateInMemory();
    MakeXform(stage, SdfPath("/Asset"), Matrix());

    MakeXform(stage, SdfPath("/Asset/Sources/A"),
              Matrix(GfVec3d(10, 0, 0)));
    MakeXform(stage, SdfPath("/Asset/Sources/B"),
              Matrix(GfVec3d(0, 10, 0)));
    MakeXform(stage, SdfPath("/Asset/Sources/Rot"),
              Matrix(GfVec3d(0),
                     GfRotation(GfVec3d(0, 0, 1), 90)));
    MakeXform(stage, SdfPath("/Asset/Sources/Scale"),
              Matrix(GfVec3d(0), GfRotation(), GfVec3d(2, 3, 4)));
    MakeXform(stage, SdfPath("/Asset/Sources/Parent"),
              Matrix(GfVec3d(4, 5, 6),
                     GfRotation(GfVec3d(0, 0, 1), 90),
                     GfVec3d(2, 3, 4)));
    MakeXform(stage, SdfPath("/Asset/Sources/Aim"),
              Matrix(GfVec3d(0, 0, 10)));
    MakeXform(stage, SdfPath("/Asset/Sources/Effector"),
              Matrix(GfVec3d(3, 2, 0)));

    MakeXform(stage, SdfPath("/Asset/Targets/Position"),
              Matrix(GfVec3d(1, 1, 5)));
    MakeXform(stage, SdfPath("/Asset/Targets/Rotation"), Matrix());
    MakeXform(stage, SdfPath("/Asset/Targets/Scale"),
              Matrix(GfVec3d(0), GfRotation(), GfVec3d(1, 2, 3)));
    MakeXform(stage, SdfPath("/Asset/Targets/Parent"),
              Matrix(GfVec3d(1, 0, 0), GfRotation(),
                     GfVec3d(1, 1.5, 0.5)));
    MakeXform(stage, SdfPath("/Asset/Targets/Aim"), Matrix());

    stage->DefinePrim(SdfPath("/Asset/Rig"), TfToken("RigExecRoot"));
    stage->DefinePrim(SdfPath("/Asset/Rig/Movers"), TfToken("Scope"));

    // Directly posed nested joints form the namespace chain inferred by the
    // SingleChainIK constraint.
    const SdfPath rootPath("/Asset/Rig/Joints/Root");
    const SdfPath midPath("/Asset/Rig/Joints/Root/Mid");
    const SdfPath endPath("/Asset/Rig/Joints/Root/Mid/End");
    const auto makeJoint = [&](const SdfPath &path, const GfVec3d &origin) {
        const UsdPrim joint = stage->DefinePrim(path, TfToken("RigExecJoint"));
        joint.CreateAttribute(TfToken("posed:space"),
                              SdfValueTypeNames->Matrix4d)
            .Set(Matrix(origin));
        return joint;
    };
    makeJoint(rootPath, GfVec3d(0, 0, 0));
    makeJoint(midPath, GfVec3d(2, 0, 0));
    makeJoint(endPath, GfVec3d(4, 0, 0));

    const UsdPrim position = MakeConstraint(
        stage, "Position", "RigExecPositionConstraint",
        {SdfPath("/Asset/Targets/Position")});
    position.CreateRelationship(TfToken("rigExec:sources"))
        .SetTargets({SdfPath("/Asset/Sources/A"),
                     SdfPath("/Asset/Sources/B")});
    position.CreateAttribute(TfToken("inputs:sourceWeights"),
                             SdfValueTypeNames->FloatArray)
        .Set(VtFloatArray{1, 3});
    position.CreateAttribute(TfToken("inputs:translationOffset"),
                             SdfValueTypeNames->Double3)
        .Set(GfVec3d(1, 2, 3));
    position.CreateAttribute(TfToken("inputs:affectZ"),
                             SdfValueTypeNames->Bool)
        .Set(false);

    const UsdPrim rotation = MakeConstraint(
        stage, "Rotation", "RigExecRotationConstraint",
        {SdfPath("/Asset/Targets/Rotation")});
    rotation.CreateRelationship(TfToken("rigExec:sources"))
        .SetTargets({SdfPath("/Asset/Sources/Rot")});

    const UsdPrim scale = MakeConstraint(
        stage, "Scale", "RigExecScaleConstraint",
        {SdfPath("/Asset/Targets/Scale")});
    scale.CreateRelationship(TfToken("rigExec:sources"))
        .SetTargets({SdfPath("/Asset/Sources/Scale")});
    scale.CreateAttribute(TfToken("inputs:scaleOffset"),
                          SdfValueTypeNames->Double3)
        .Set(GfVec3d(1, 0, -1));
    scale.CreateAttribute(TfToken("inputs:affectY"),
                          SdfValueTypeNames->Bool)
        .Set(false);

    const UsdPrim parent = MakeConstraint(
        stage, "Parent", "RigExecParentConstraint",
        {SdfPath("/Asset/Targets/Parent")});
    parent.CreateRelationship(TfToken("rigExec:sources"))
        .SetTargets({SdfPath("/Asset/Sources/Parent")});
    parent.CreateAttribute(TfToken("inputs:translationOffsets"),
                           SdfValueTypeNames->Double3Array)
        .Set(VtVec3dArray{GfVec3d(1, 0, 0)});

    const UsdPrim aim = MakeConstraint(
        stage, "Aim", "RigExecAimConstraint",
        {SdfPath("/Asset/Targets/Aim")});
    aim.CreateRelationship(TfToken("rigExec:sources"))
        .SetTargets({SdfPath("/Asset/Sources/Aim")});

    const UsdPrim ik = MakeConstraint(
        stage, "IK", "RigExecSingleChainIkConstraint",
        {rootPath, midPath, endPath});
    ik.CreateRelationship(TfToken("rigExec:firstJoint"))
        .SetTargets({rootPath});
    ik.CreateRelationship(TfToken("rigExec:endJoint"))
        .SetTargets({endPath});
    ik.CreateRelationship(TfToken("rigExec:effector"))
        .SetTargets({SdfPath("/Asset/Sources/Effector")});
    ik.CreateAttribute(TfToken("inputs:poleVector"),
                       SdfValueTypeNames->Double3)
        .Set(GfVec3d(0, 0, 1));

    if (positionOut) {
        *positionOut = position;
    }
    return stage;
}

static void
TestEvaluatorSemantics()
{
    UsdPrim position;
    const UsdStageRefPtr stage = BuildConstraintStage(&position);
    std::string before;
    CHECK(stage->GetRootLayer()->ExportToString(&before));

    RigExecRigEvaluator evaluator(stage, SdfPath("/Asset/Rig"));
    std::vector<std::string> errors;
    CHECK(evaluator.Compile(&errors));
    for (const std::string &error : errors) {
        if (error.rfind("warning:", 0) != 0) {
            std::printf("compile diagnostic: %s\n", error.c_str());
        }
    }
    const RigExecRigPose pose = evaluator.Evaluate(UsdTimeCode::Default());
    CHECK(pose.valid);

    const auto matrixFor = [&](const char *path) {
        const auto it = pose.providerXforms.find(SdfPath(path));
        CHECK(it != pose.providerXforms.end());
        return it != pose.providerXforms.end() ? it->second : GfMatrix4d(1.0);
    };

    const GfMatrix4d positioned = matrixFor("/Asset/Targets/Position");
    CHECK(Near(positioned.ExtractTranslation(), GfVec3d(3.5, 9.5, 5)));

    const GfMatrix4d rotated = matrixFor("/Asset/Targets/Rotation");
    CHECK(Near(rotated.TransformDir(GfVec3d(1, 0, 0)),
               GfVec3d(0, 1, 0)));

    const GfMatrix4d scaled = matrixFor("/Asset/Targets/Scale");
    CHECK(std::abs(scaled.TransformDir(GfVec3d(1, 0, 0)).GetLength() - 3) <
          1e-5);
    CHECK(std::abs(scaled.TransformDir(GfVec3d(0, 1, 0)).GetLength() - 2) <
          1e-5);
    CHECK(std::abs(scaled.TransformDir(GfVec3d(0, 0, 1)).GetLength() - 3) <
          1e-5);

    const GfMatrix4d parented = matrixFor("/Asset/Targets/Parent");
    // Parent translation offsets are evaluated in the source's local space.
    CHECK(Near(parented.ExtractTranslation(), GfVec3d(4, 7, 6)));
    CHECK(Near(parented.TransformDir(GfVec3d(1, 0, 0)).GetNormalized(),
               GfVec3d(0, 1, 0)));
    // FBX runtime defaults Parent scale axes off.
    CHECK(std::abs(parented.TransformDir(GfVec3d(0, 1, 0)).GetLength() - 1.5) <
          1e-5);

    const GfMatrix4d aimed = matrixFor("/Asset/Targets/Aim");
    CHECK(Near(aimed.TransformDir(GfVec3d(1, 0, 0)).GetNormalized(),
               GfVec3d(0, 0, 1)));

    const SdfPath endPath("/Asset/Rig/Joints/Root/Mid/End");
    const auto end = pose.jointFramesFinal.find(endPath);
    CHECK(end != pose.jointFramesFinal.end());
    if (end != pose.jointFramesFinal.end()) {
        CHECK(Near(end->second.Origin(), GfVec3d(3, 2, 0), 2e-4));
    }

    std::string after;
    CHECK(stage->GetRootLayer()->ExportToString(&after));
    CHECK(before == after);  // evaluation authors nothing

    // Global weight is value-only and blends after the source aggregate.
    position.GetAttribute(TfToken("inputs:weight")).Set(0.5f);
    const RigExecRigPose half = evaluator.Evaluate(UsdTimeCode::Default());
    CHECK(half.valid);
    const auto halfPosition =
        half.providerXforms.find(SdfPath("/Asset/Targets/Position"));
    CHECK(halfPosition != half.providerXforms.end());
    if (halfPosition != half.providerXforms.end()) {
        CHECK(Near(halfPosition->second.ExtractTranslation(),
                   GfVec3d(2.25, 5.25, 5)));
    }

    // A dormant zero-weight constraint does not inspect malformed value-only
    // source data; it is an exact pass-through just like inputs:enabled=false.
    position.GetAttribute(TfToken("inputs:weight")).Set(0.0f);
    position.GetAttribute(TfToken("inputs:sourceWeights"))
        .Set(VtFloatArray{1, 2, 3});
    const RigExecRigPose dormant =
        evaluator.Evaluate(UsdTimeCode::Default());
    CHECK(dormant.valid);
    CHECK(std::none_of(
        dormant.diagnostics.begin(), dormant.diagnostics.end(),
        [](const std::string &diagnostic) {
            return diagnostic.find("sourceWeights") != std::string::npos;
        }));
    const auto dormantPosition =
        dormant.providerXforms.find(SdfPath("/Asset/Targets/Position"));
    CHECK(dormantPosition != dormant.providerXforms.end());
    if (dormantPosition != dormant.providerXforms.end()) {
        CHECK(Near(dormantPosition->second.ExtractTranslation(),
                   GfVec3d(1, 1, 5)));
    }

    // MoverAPI enable is a shape-preserving pass-through.
    position.GetAttribute(TfToken("inputs:enabled")).Set(false);
    const RigExecRigPose disabled =
        evaluator.Evaluate(UsdTimeCode::Default());
    CHECK(disabled.valid);
    const auto disabledPosition =
        disabled.providerXforms.find(SdfPath("/Asset/Targets/Position"));
    CHECK(disabledPosition != disabled.providerXforms.end());
    if (disabledPosition != disabled.providerXforms.end()) {
        CHECK(Near(disabledPosition->second.ExtractTranslation(),
                   GfVec3d(1, 1, 5)));
    }
}

static bool
HasDiagnostic(const RigExecRigPose &pose, const char *needle)
{
    return std::any_of(
        pose.diagnostics.begin(), pose.diagnostics.end(),
        [needle](const std::string &diagnostic) {
            return diagnostic.find(needle) != std::string::npos;
        });
}

static void
TestConstraintCompositionAndHierarchy()
{
    const UsdStageRefPtr stage = UsdStage::CreateInMemory();
    MakeXform(stage, SdfPath("/Asset"), Matrix());
    MakeXform(stage, SdfPath("/Asset/A"), Matrix(GfVec3d(10, 0, 0)));
    MakeXform(stage, SdfPath("/Asset/Target"), Matrix());
    stage->DefinePrim(SdfPath("/Asset/Rig"), TfToken("RigExecRoot"));
    stage->DefinePrim(SdfPath("/Asset/Rig/Movers"), TfToken("Scope"));

    const UsdPrim first = MakeConstraint(
        stage, "First", "RigExecPositionConstraint",
        {SdfPath("/Asset/Target")});
    first.CreateRelationship(TfToken("rigExec:sources"))
        .SetTargets({SdfPath("/Asset/Target")});
    first.CreateAttribute(TfToken("inputs:translationOffset"),
                          SdfValueTypeNames->Double3)
        .Set(GfVec3d(5, 0, 0));
    const UsdPrim second = stage->DefinePrim(
        SdfPath("/Asset/Rig/Movers/First/Second"),
        TfToken("RigExecPositionConstraint"));
    CHECK(second.ApplyAPI(TfToken("RigExecMoverAPI")));
    second.CreateRelationship(TfToken("rigExec:moves"))
        .SetTargets({SdfPath("/Asset/Target")});
    second.CreateRelationship(TfToken("rigExec:sources"))
        .SetTargets({SdfPath("/Asset/A")});

    RigExecRigEvaluator evaluator(stage, SdfPath("/Asset/Rig"));
    std::vector<std::string> errors;
    CHECK(evaluator.Compile(&errors));
    const RigExecRigPose pose = evaluator.Evaluate(UsdTimeCode::Default());
    CHECK(pose.valid);
    const auto target = pose.providerXforms.find(SdfPath("/Asset/Target"));
    CHECK(target != pose.providerXforms.end());
    if (target != pose.providerXforms.end()) {
        CHECK(Near(target->second.ExtractTranslation(), GfVec3d(15, 0, 0)));
    }

    // A parent constraint must carry its unchanged joint descendants, even
    // though OpenExec has already produced the base snapshot this generation.
    const SdfPath root("/Asset/Rig/Joints/Root");
    const SdfPath child("/Asset/Rig/Joints/Root/Child");
    stage->DefinePrim(root, TfToken("RigExecJoint"))
        .CreateAttribute(TfToken("posed:space"), SdfValueTypeNames->Matrix4d)
        .Set(Matrix());
    stage->DefinePrim(child, TfToken("RigExecJoint"))
        .CreateAttribute(TfToken("posed:space"), SdfValueTypeNames->Matrix4d)
        .Set(Matrix(GfVec3d(2, 0, 0)));
    MakeXform(stage, SdfPath("/Asset/ParentSource"),
              Matrix(GfVec3d(5, 0, 0)));
    const UsdPrim parent = MakeConstraint(
        stage, "Third", "RigExecPositionConstraint", {root});
    parent.CreateRelationship(TfToken("rigExec:sources"))
        .SetTargets({SdfPath("/Asset/ParentSource")});

    CHECK(evaluator.Compile(&errors));
    const RigExecRigPose hierarchy = evaluator.Evaluate(UsdTimeCode::Default());
    const auto childFrame = hierarchy.jointFramesFinal.find(child);
    CHECK(childFrame != hierarchy.jointFramesFinal.end());
    if (childFrame != hierarchy.jointFramesFinal.end()) {
        CHECK(Near(childFrame->second.Origin(), GfVec3d(7, 0, 0)));
    }
}

static void
TestModeAndFailureContracts()
{
    // Every Aim world-up spelling is evaluator-side. In particular, FBX's
    // two object modes have defined no-object fallbacks rather than a missing
    // binding failure.
    for (const char *worldUpType : {"sceneUp", "vector", "objectUp",
                                    "objectRotationUp", "none"}) {
        const UsdStageRefPtr stage = BuildConstraintStage(nullptr);
        stage->GetAttributeAtPath(
                 SdfPath("/Asset/Targets/Aim.xformOp:transform"))
            .Set(Matrix(GfVec3d(2, 0, 0)));
        const UsdPrim aim =
            stage->GetPrimAtPath(SdfPath("/Asset/Rig/Movers/Aim"));
        aim.GetAttribute(TfToken("rigExec:worldUpType"))
            .Set(TfToken(worldUpType));
        RigExecRigEvaluator evaluator(stage, SdfPath("/Asset/Rig"));
        std::vector<std::string> errors;
        CHECK(evaluator.Compile(&errors));
        const RigExecRigPose pose = evaluator.Evaluate(UsdTimeCode::Default());
        const auto aimed = pose.providerXforms.find(SdfPath("/Asset/Targets/Aim"));
        CHECK(aimed != pose.providerXforms.end());
        if (aimed != pose.providerXforms.end()) {
            CHECK(Near(aimed->second.TransformDir(GfVec3d(1, 0, 0))
                           .GetNormalized(),
                       GfVec3d(-2, 0, 10).GetNormalized()));
        }
        CHECK(!HasDiagnostic(pose, "world-up object"));
    }

    // SingleChain mode ignores dormant pole wiring, malformed pole weights,
    // non-finite pole values, and twist end-to-end.
    {
        const UsdStageRefPtr stage = BuildConstraintStage(nullptr);
        const UsdPrim ik = stage->GetPrimAtPath(SdfPath("/Asset/Rig/Movers/IK"));
        ik.GetAttribute(TfToken("rigExec:solverMode")).Set(TfToken("singleChain"));
        ik.GetAttribute(TfToken("rigExec:poleVectorMode")).Set(TfToken("object"));
        ik.GetRelationship(TfToken("rigExec:poleVectorObjects"))
            .SetTargets({SdfPath("/Missing/Pole")});
        ik.GetAttribute(TfToken("inputs:poleVectorWeights"))
            .Set(VtFloatArray{std::numeric_limits<float>::quiet_NaN(), 1});
        ik.GetAttribute(TfToken("inputs:poleVector"))
            .Set(GfVec3d(std::numeric_limits<double>::quiet_NaN(), 0, 0));
        ik.GetAttribute(TfToken("inputs:twistDegrees"))
            .Set(std::numeric_limits<double>::quiet_NaN());
        RigExecRigEvaluator evaluator(stage, SdfPath("/Asset/Rig"));
        std::vector<std::string> errors;
        CHECK(evaluator.Compile(&errors));
        const RigExecRigPose pose = evaluator.Evaluate(UsdTimeCode::Default());
        const auto end = pose.jointFramesFinal.find(
            SdfPath("/Asset/Rig/Joints/Root/Mid/End"));
        CHECK(end != pose.jointFramesFinal.end());
        if (end != pose.jointFramesFinal.end()) {
            CHECK(Near(end->second.Origin(), GfVec3d(3, 2, 0), 2e-4));
        }
    }

    // EvaluateTSAnim modes select which authored chain lengths reach the IK
    // solve: Never uses rest T/S, Always uses the current posed T/S, and Auto
    // detects the time-sampled posed transforms and follows Always.
    for (const char *evaluationMode :
         {"neverTS", "autoDetect", "alwaysTS"}) {
        const UsdStageRefPtr stage = BuildConstraintStage(nullptr);
        const SdfPath root("/Asset/Rig/Joints/Root");
        const SdfPath mid("/Asset/Rig/Joints/Root/Mid");
        const SdfPath end("/Asset/Rig/Joints/Root/Mid/End");
        const SdfPath joints[] = {root, mid, end};
        const double restX[] = {0.0, 1.0, 2.0};
        const double posedX[] = {0.0, 2.0, 4.0};
        for (size_t i = 0; i < 3; ++i) {
            const UsdPrim joint = stage->GetPrimAtPath(joints[i]);
            joint.CreateAttribute(TfToken("rest:space"),
                                  SdfValueTypeNames->Matrix4d)
                .Set(Matrix(GfVec3d(restX[i], 0, 0)));
            joint.GetAttribute(TfToken("posed:space"))
                .Set(Matrix(GfVec3d(posedX[i], 0, 0)), UsdTimeCode(1));
        }
        stage->GetAttributeAtPath(
                 SdfPath("/Asset/Sources/Effector.xformOp:transform"))
            .Set(Matrix(GfVec3d(3, 0, 0)), UsdTimeCode(1));
        const UsdPrim ik =
            stage->GetPrimAtPath(SdfPath("/Asset/Rig/Movers/IK"));
        ik.GetAttribute(TfToken("rigExec:solverMode"))
            .Set(TfToken("singleChain"));
        ik.GetAttribute(TfToken("rigExec:evaluationMode"))
            .Set(TfToken(evaluationMode));

        RigExecRigEvaluator evaluator(stage, SdfPath("/Asset/Rig"));
        std::vector<std::string> errors;
        CHECK(evaluator.Compile(&errors));
        const RigExecRigPose pose = evaluator.Evaluate(UsdTimeCode(1));
        const auto solvedEnd = pose.jointFramesFinal.find(end);
        CHECK(solvedEnd != pose.jointFramesFinal.end());
        if (solvedEnd != pose.jointFramesFinal.end()) {
            const double expectedX =
                std::string(evaluationMode) == "neverTS" ? 2.0 : 3.0;
            CHECK(Near(solvedEnd->second.Origin(),
                       GfVec3d(expectedX, 0, 0), 2e-4));
        }
    }

    // Parent rotation offsets are parallel source-local Euler values and are
    // applied alongside translation offsets before source aggregation.
    {
        const UsdStageRefPtr stage = BuildConstraintStage(nullptr);
        const UsdPrim parent =
            stage->GetPrimAtPath(SdfPath("/Asset/Rig/Movers/Parent"));
        parent.CreateAttribute(TfToken("inputs:rotationOffsets"),
                               SdfValueTypeNames->Double3Array)
            .Set(VtVec3dArray{GfVec3d(0, 0, 90)});
        RigExecRigEvaluator evaluator(stage, SdfPath("/Asset/Rig"));
        std::vector<std::string> errors;
        CHECK(evaluator.Compile(&errors));
        const RigExecRigPose pose = evaluator.Evaluate(UsdTimeCode::Default());
        const auto parentXform = pose.providerXforms.find(
            SdfPath("/Asset/Targets/Parent"));
        CHECK(parentXform != pose.providerXforms.end());
        if (parentXform != pose.providerXforms.end()) {
            CHECK(Near(parentXform->second.TransformDir(GfVec3d(1, 0, 0))
                           .GetNormalized(),
                       GfVec3d(-1, 0, 0)));
        }
    }

    // Parallel source arrays fail closed at evaluation and leave the target
    // untouched rather than accepting a partially specified Parent offset.
    {
        const UsdStageRefPtr stage = BuildConstraintStage(nullptr);
        const UsdPrim parent =
            stage->GetPrimAtPath(SdfPath("/Asset/Rig/Movers/Parent"));
        parent.CreateAttribute(TfToken("inputs:rotationOffsets"),
                               SdfValueTypeNames->Double3Array)
            .Set(VtVec3dArray{GfVec3d(0), GfVec3d(1, 0, 0)});
        RigExecRigEvaluator evaluator(stage, SdfPath("/Asset/Rig"));
        std::vector<std::string> errors;
        CHECK(evaluator.Compile(&errors));
        const RigExecRigPose pose = evaluator.Evaluate(UsdTimeCode::Default());
        CHECK(HasDiagnostic(pose, "rotationOffsets has 2 entries"));
        const auto parentXform = pose.providerXforms.find(
            SdfPath("/Asset/Targets/Parent"));
        CHECK(parentXform != pose.providerXforms.end());
        if (parentXform != pose.providerXforms.end()) {
            CHECK(Near(parentXform->second.ExtractTranslation(),
                       GfVec3d(1, 0, 0)));
        }
    }

    // Structural tokens cannot be animated or assigned an unknown value.
    for (const bool animated : {false, true}) {
        const UsdStageRefPtr stage = BuildConstraintStage(nullptr);
        const UsdPrim ik = stage->GetPrimAtPath(SdfPath("/Asset/Rig/Movers/IK"));
        const UsdAttribute mode = ik.GetAttribute(TfToken("rigExec:evaluationMode"));
        if (animated) {
            mode.Set(TfToken("alwaysTS"), UsdTimeCode(1));
        } else {
            mode.Set(TfToken("notAMode"));
        }
        RigExecRigEvaluator evaluator(stage, SdfPath("/Asset/Rig"));
        std::vector<std::string> errors;
        CHECK(!evaluator.Compile(&errors));
    }
}

// A constraint revises a TRANSFORM. Any UsdGeomXformable can carry one --
// Mesh and BasisCurves included -- which is the same predicate
// bindFrameSource already applies to constraint sources. The write path must
// therefore not rewrite a PointBased prim target into its .points property.
static void
TestXformableTargetsCompile()
{
    for (const char *targetType : {"Mesh", "BasisCurves", "Points",
                                   "Xform", "Sphere"}) {
        const UsdStageRefPtr stage = UsdStage::CreateInMemory();
        MakeXform(stage, SdfPath("/Asset"), Matrix());
        stage->DefinePrim(SdfPath("/Asset/Geom"), TfToken("Scope"));
        const UsdPrim target = stage->DefinePrim(
            SdfPath("/Asset/Geom/Target"), TfToken(targetType));
        CHECK(target);
        MakeXform(stage, SdfPath("/Asset/Source"),
                  Matrix(GfVec3d(0, 0, 0),
                         GfRotation(GfVec3d(0, 1, 0), 90.0)));
        stage->DefinePrim(SdfPath("/Asset/Rig"), TfToken("RigExecRoot"));
        stage->DefinePrim(SdfPath("/Asset/Rig/Movers"), TfToken("Scope"));
        const UsdPrim rotation = MakeConstraint(
            stage, "Rot", "RigExecRotationConstraint",
            {SdfPath("/Asset/Geom/Target")});
        rotation.CreateRelationship(TfToken("rigExec:sources"))
            .SetTargets({SdfPath("/Asset/Source")});

        RigExecRigEvaluator evaluator(stage, SdfPath("/Asset/Rig"));
        std::vector<std::string> errors;
        CHECK(evaluator.Compile(&errors));
        const RigExecRigPose pose = evaluator.Evaluate(UsdTimeCode::Default());
        CHECK(pose.valid);
        const auto revised =
            pose.providerXforms.find(SdfPath("/Asset/Geom/Target"));
        CHECK(revised != pose.providerXforms.end());
        if (revised != pose.providerXforms.end()) {
            // The source is a 90-degree Y rotation, so the revised x axis
            // must point down -z.
            CHECK(Near(revised->second.TransformDir(GfVec3d(1, 0, 0)),
                       GfVec3d(0, 0, -1)));
        }
        // A transform-domain constraint writes no points.
        CHECK(pose.movedProperties.empty());
    }
}

// The gate must test what it means. A non-Xformable prim cannot carry a
// revised transform, and a .points target names the geometry domain, which
// phase 1 does not implement -- both are hard errors with distinct wording.
static void
TestTransformProviderPredicate()
{
    // A non-Xformable target is rejected as such.
    {
        const UsdStageRefPtr stage = UsdStage::CreateInMemory();
        MakeXform(stage, SdfPath("/Asset"), Matrix());
        stage->DefinePrim(SdfPath("/Asset/Geom"), TfToken("Scope"));
        stage->DefinePrim(SdfPath("/Asset/Geom/NotXformable"),
                          TfToken("Scope"));
        MakeXform(stage, SdfPath("/Asset/Source"), Matrix());
        stage->DefinePrim(SdfPath("/Asset/Rig"), TfToken("RigExecRoot"));
        stage->DefinePrim(SdfPath("/Asset/Rig/Movers"), TfToken("Scope"));
        const UsdPrim rotation = MakeConstraint(
            stage, "Rot", "RigExecRotationConstraint",
            {SdfPath("/Asset/Geom/NotXformable")});
        rotation.CreateRelationship(TfToken("rigExec:sources"))
            .SetTargets({SdfPath("/Asset/Source")});
        RigExecRigEvaluator evaluator(stage, SdfPath("/Asset/Rig"));
        std::vector<std::string> errors;
        CHECK(!evaluator.Compile(&errors));
        CHECK(std::any_of(errors.begin(), errors.end(),
                          [](const std::string &error) {
                              return error.find("is not a transform provider") !=
                                     std::string::npos;
                          }));
    }

    // A .points target is the geometry domain: recognised, and explicitly
    // deferred rather than misreported as a bad transform provider.
    {
        const UsdStageRefPtr stage = UsdStage::CreateInMemory();
        MakeXform(stage, SdfPath("/Asset"), Matrix());
        stage->DefinePrim(SdfPath("/Asset/Geom"), TfToken("Scope"));
        stage->DefinePrim(SdfPath("/Asset/Geom/M"), TfToken("Mesh"));
        MakeXform(stage, SdfPath("/Asset/Source"), Matrix());
        stage->DefinePrim(SdfPath("/Asset/Rig"), TfToken("RigExecRoot"));
        stage->DefinePrim(SdfPath("/Asset/Rig/Movers"), TfToken("Scope"));
        const UsdPrim rotation = MakeConstraint(
            stage, "Rot", "RigExecRotationConstraint",
            {SdfPath("/Asset/Geom/M").AppendProperty(TfToken("points"))});
        rotation.CreateRelationship(TfToken("rigExec:sources"))
            .SetTargets({SdfPath("/Asset/Source")});
        RigExecRigEvaluator evaluator(stage, SdfPath("/Asset/Rig"));
        std::vector<std::string> errors;
        CHECK(!evaluator.Compile(&errors));
        CHECK(std::any_of(
            errors.begin(), errors.end(), [](const std::string &error) {
                return error.find(
                           "geometry-domain constraint targets are not "
                           "supported yet") != std::string::npos;
            }));
    }
}

// With write-path inference gone, a point-domain mover handed a bare mesh
// prim is an error -- and it must carry the fix, because the old behaviour
// silently rewrote it and that silence is what this change removes. Both
// validator families must say so: the typed geometry movers and MatrixMover,
// which validates separately.
static void
TestPointDomainMoverNamesTheFix()
{
    for (const char *moverType : {"RigExecSmoothMover", "RigExecMatrixMover"}) {
        const UsdStageRefPtr stage = UsdStage::CreateInMemory();
        MakeXform(stage, SdfPath("/Asset"), Matrix());
        stage->DefinePrim(SdfPath("/Asset/Geom"), TfToken("Scope"));
        stage->DefinePrim(SdfPath("/Asset/Geom/M"), TfToken("Mesh"));
        stage->DefinePrim(SdfPath("/Asset/Rig"), TfToken("RigExecRoot"));
        stage->DefinePrim(SdfPath("/Asset/Rig/Movers"), TfToken("Scope"));
        const UsdPrim mover = stage->DefinePrim(
            SdfPath("/Asset/Rig/Movers/M"), TfToken(moverType));
        CHECK(mover.ApplyAPI(TfToken("RigExecMoverAPI")));
        mover.CreateRelationship(TfToken("rigExec:moves"))
            .SetTargets({SdfPath("/Asset/Geom/M")});

        RigExecRigEvaluator evaluator(stage, SdfPath("/Asset/Rig"));
        std::vector<std::string> errors;
        CHECK(!evaluator.Compile(&errors));
        CHECK(std::any_of(errors.begin(), errors.end(),
                          [](const std::string &error) {
                              return error.find("Did you mean") !=
                                     std::string::npos &&
                                     error.find("/Asset/Geom/M.points") !=
                                     std::string::npos;
                          }));
    }
}

// The transform a Mesh target receives must equal the one an Xform target
// receives from the same constraint. This is the property that made the
// original bug invisible to every existing test: they all used Xform targets.
static void
TestMeshAndXformTargetsAgree()
{
    GfMatrix4d meshResult(1.0);
    GfMatrix4d xformResult(1.0);
    for (const char *targetType : {"Mesh", "Xform"}) {
        const UsdStageRefPtr stage = UsdStage::CreateInMemory();
        MakeXform(stage, SdfPath("/Asset"), Matrix());
        stage->DefinePrim(SdfPath("/Asset/Geom"), TfToken("Scope"));
        stage->DefinePrim(SdfPath("/Asset/Geom/Target"), TfToken(targetType));
        MakeXform(stage, SdfPath("/Asset/Source"),
                  Matrix(GfVec3d(0, 0, 0),
                         GfRotation(GfVec3d(0, 1, 0), 37.5)));
        stage->DefinePrim(SdfPath("/Asset/Rig"), TfToken("RigExecRoot"));
        stage->DefinePrim(SdfPath("/Asset/Rig/Movers"), TfToken("Scope"));
        const UsdPrim rotation = MakeConstraint(
            stage, "Rot", "RigExecRotationConstraint",
            {SdfPath("/Asset/Geom/Target")});
        rotation.CreateRelationship(TfToken("rigExec:sources"))
            .SetTargets({SdfPath("/Asset/Source")});
        RigExecRigEvaluator evaluator(stage, SdfPath("/Asset/Rig"));
        std::vector<std::string> errors;
        CHECK(evaluator.Compile(&errors));
        const RigExecRigPose pose = evaluator.Evaluate(UsdTimeCode::Default());
        const auto revised =
            pose.providerXforms.find(SdfPath("/Asset/Geom/Target"));
        CHECK(revised != pose.providerXforms.end());
        if (revised != pose.providerXforms.end()) {
            (std::string(targetType) == "Mesh" ? meshResult : xformResult) =
                revised->second;
        }
    }
    // Guard against a vacuous pass: if the constraint drove nothing, both
    // results would be the identity and would trivially agree.
    CHECK(!GfIsClose(meshResult, GfMatrix4d(1.0), 1e-9));
    CHECK(Near(meshResult.TransformDir(GfVec3d(1, 0, 0)),
               GfVec3d(std::cos(GfDegreesToRadians(37.5)), 0,
                       -std::sin(GfDegreesToRadians(37.5)))));
    CHECK(GfIsClose(meshResult, xformResult, 1e-12));
}

// The registry is the single source of truth about which operators exist.
// Every concrete constraint in the schema must have exactly one row, and the
// membership predicates must agree with it -- otherwise a seventh operator
// can be added to one and forgotten in the other, which is the failure mode
// the table exists to remove.
static void
TestConstraintRegistryCoversTheSchema()
{
    const UsdSchemaRegistry &registry = UsdSchemaRegistry::GetInstance();
    for (const char *typeName :
         {"RigExecAimConstraint", "RigExecPositionConstraint",
          "RigExecRotationConstraint", "RigExecScaleConstraint",
          "RigExecParentConstraint", "RigExecSingleChainIkConstraint",
          "RigExecCustomConstraint"}) {
        CHECK(registry.FindConcretePrimDefinition(TfToken(typeName)));
        CHECK(RigExecConstraintHandlerCount(TfToken(typeName)) == 1);
    }
    // Seven rows, no more: an unregistered type must not resolve.
    CHECK(RigExecConstraintHandlerCount(TfToken("RigExecSmoothMover")) == 0);
    CHECK(RigExecConstraintHandlerTotal() == 7);
}

static void
TestInvalidContractsFailClosed()
{
    // Every built-in evaluator constraint is a writer. A typed constraint
    // without the write relationship cannot silently become a grouping prim.
    {
        const UsdStageRefPtr stage = UsdStage::CreateInMemory();
        MakeXform(stage, SdfPath("/Asset"), Matrix());
        MakeXform(stage, SdfPath("/Asset/Target"), Matrix());
        MakeXform(stage, SdfPath("/Asset/Source"), Matrix(GfVec3d(1, 0, 0)));
        stage->DefinePrim(SdfPath("/Asset/Rig"), TfToken("RigExecRoot"));
        stage->DefinePrim(SdfPath("/Asset/Rig/Movers"), TfToken("Scope"));
        const UsdPrim aim = stage->DefinePrim(
            SdfPath("/Asset/Rig/Movers/Aim"), TfToken("RigExecAimConstraint"));
        CHECK(aim.ApplyAPI(TfToken("RigExecMoverAPI")));
        aim.CreateRelationship(TfToken("rigExec:sources"))
            .SetTargets({SdfPath("/Asset/Source")});
        RigExecRigEvaluator evaluator(stage, SdfPath("/Asset/Rig"));
        std::vector<std::string> errors;
        CHECK(!evaluator.Compile(&errors));
        CHECK(std::any_of(errors.begin(), errors.end(),
                          [](const std::string &error) {
                              return error.find("no moves targets") !=
                                     std::string::npos;
                          }));
    }

    // Custom is a lossless carrier, not a silent mover implementation.
    {
        const UsdStageRefPtr stage = UsdStage::CreateInMemory();
        MakeXform(stage, SdfPath("/Asset"), Matrix());
        MakeXform(stage, SdfPath("/Asset/Target"), Matrix());
        stage->DefinePrim(SdfPath("/Asset/Rig"), TfToken("RigExecRoot"));
        stage->DefinePrim(SdfPath("/Asset/Rig/Movers"), TfToken("Scope"));
        MakeConstraint(stage, "Custom", "RigExecCustomConstraint",
                       {SdfPath("/Asset/Target")});
        RigExecRigEvaluator evaluator(stage, SdfPath("/Asset/Rig"));
        std::vector<std::string> errors;
        CHECK(!evaluator.Compile(&errors));
        CHECK(std::any_of(errors.begin(), errors.end(),
                          [](const std::string &error) {
                              return error.find("no registered evaluator") !=
                                     std::string::npos;
                          }));
    }

    // A source-based constraint without a source cannot become a no-op that
    // still claims a write in the mover graph.
    {
        const UsdStageRefPtr stage = UsdStage::CreateInMemory();
        MakeXform(stage, SdfPath("/Asset"), Matrix());
        MakeXform(stage, SdfPath("/Asset/Target"), Matrix());
        stage->DefinePrim(SdfPath("/Asset/Rig"), TfToken("RigExecRoot"));
        stage->DefinePrim(SdfPath("/Asset/Rig/Movers"), TfToken("Scope"));
        MakeConstraint(stage, "Position", "RigExecPositionConstraint",
                       {SdfPath("/Asset/Target")});
        RigExecRigEvaluator evaluator(stage, SdfPath("/Asset/Rig"));
        std::vector<std::string> errors;
        CHECK(!evaluator.Compile(&errors));
        CHECK(std::any_of(errors.begin(), errors.end(),
                          [](const std::string &error) {
                              return error.find("no constraint sources") !=
                                     std::string::npos;
                          }));
    }
}

int
main()
{
    const auto plugins = PlugRegistry::GetInstance().RegisterPlugins(
        RIGEXEC_SCHEMA_RESOURCE_DIR);
    if (plugins.empty()) {
        std::printf("FATAL: no schema plugin found at %s\n",
                    RIGEXEC_SCHEMA_RESOURCE_DIR);
        return 2;
    }

    TestSchemaSurface();
    TestEvaluatorSemantics();
    TestConstraintCompositionAndHierarchy();
    TestModeAndFailureContracts();
    TestXformableTargetsCompile();
    TestTransformProviderPredicate();
    TestPointDomainMoverNamesTheFix();
    TestMeshAndXformTargetsAgree();
    TestConstraintRegistryCoversTheSchema();
    TestInvalidContractsFailClosed();

    if (failures) {
        std::printf("%d FAILURE(S)\n", failures);
        return 1;
    }
    std::printf("testRigExecConstraints: all tests passed\n");
    return 0;
}
