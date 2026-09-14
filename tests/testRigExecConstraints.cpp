//
// FBX-equivalent constraint schema and evaluator conformance.
//
#include "rigExec/rigEvaluator.h"
#include "rigExec/frameExtraction.h"
#include "rigExecMath/pointFrame.h"
#include "rigExecMath/splineIk.h"

#include "pxr/base/gf/rotation.h"
#include "pxr/base/plug/registry.h"
#include "pxr/usd/sdf/types.h"
#include "pxr/usd/usd/primDefinition.h"
#include "pxr/usd/usd/schemaRegistry.h"
#include "pxr/usd/usd/stage.h"
#include "pxr/usd/usdGeom/xform.h"

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdio>
#include <limits>
#include <string>
#include <utility>
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

// GfRotation's default constructor has an EMPTY body and its members are
// uninitialized (gf/rotation.h:42, GfVec3d() = default), so `GfRotation()` is
// whatever was on the stack. Defaulting the parameter to it made every
// Matrix() call that omits a rotation inherit the previous call's leftovers --
// silently, and only when a caller happened to look at the rotation.
// Identity has to be spelled out.
static GfMatrix4d
Matrix(const GfVec3d &translation = GfVec3d(0),
       const GfRotation &rotation = GfRotation(GfVec3d(0, 0, 1), 0.0),
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
        "RigExecParentConstraint", "RigExecSingleChainIkConstraint"};
    for (const char *type : types) {
        const UsdPrimDefinition *definition =
            registry.FindConcretePrimDefinition(TfToken(type));
        CHECK(definition);
        CHECK(HasProperty(definition, "rigExec:locked"));
        if (std::string(type) != "RigExecSingleChainIkConstraint") {
            CHECK(HasProperty(definition, "rigExec:sources"));
            CHECK(HasProperty(definition, "inputs:sourceWeights"));
        }
    }
    const UsdPrimDefinition *moverApi =
        registry.FindAppliedAPIPrimDefinition(TfToken("RigExecMoverAPI"));
    CHECK(moverApi);
    CHECK(HasProperty(moverApi, "inputs:defaultWeight"));
    float weight = 0;
    CHECK(moverApi && moverApi->GetAttributeFallbackValue(
                          TfToken("inputs:defaultWeight"), &weight));
    CHECK(std::abs(weight - 1.0f) < 1e-7f);
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
    position.CreateAttribute(TfToken("inputs:affectTranslationZ"),
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
    scale.CreateAttribute(TfToken("inputs:affectScaleY"),
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
    position.GetAttribute(TfToken("inputs:defaultWeight")).Set(0.5f);
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
    position.GetAttribute(TfToken("inputs:defaultWeight")).Set(0.0f);
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

// A DynamicWeight with a base must transform the complete base packet, not
// silently fall back to the no-base constant case. Exercise both the Exec
// packet and the evaluator's independent CPU resolver through a real matrix
// consumer so parity covers the composed field.
static void
TestDynamicWeightDenseBaseFormulaAndParity()
{
    const UsdStageRefPtr stage = UsdStage::CreateInMemory();
    const SdfPath target("/Asset/Geom/P.points");
    const VtVec3fArray basePoints = {
        GfVec3f(0, 0, 0), GfVec3f(1, 0, 0), GfVec3f(2, 0, 0)};
    const UsdPrim points =
        stage->DefinePrim(SdfPath("/Asset/Geom/P"), TfToken("Points"));
    points.CreateAttribute(TfToken("points"),
                           SdfValueTypeNames->Point3fArray)
        .Set(basePoints);
    stage->DefinePrim(SdfPath("/Asset/Rig"), TfToken("RigExecRoot"));
    stage->DefinePrim(SdfPath("/Asset/Rig/Movers"), TfToken("Scope"));
    stage->DefinePrim(SdfPath("/Asset/Rig/Weights"), TfToken("Scope"));

    const UsdPrim joint = stage->DefinePrim(
        SdfPath("/Asset/Rig/Joints/J"), TfToken("RigExecJoint"));
    joint.CreateAttribute(TfToken("posed:space"),
                          SdfValueTypeNames->Matrix4d)
        .Set(Matrix(GfVec3d(0, 10, 0)));

    const UsdPrim base = stage->DefinePrim(
        SdfPath("/Asset/Rig/Weights/Base"),
        TfToken("RigExecStaticWeight"));
    base.CreateRelationship(TfToken("rigExec:weightTarget"))
        .SetTargets({target});
    base.CreateAttribute(TfToken("rigExec:representation"),
                         SdfValueTypeNames->Token)
        .Set(TfToken("dense"));
    base.CreateAttribute(TfToken("rigExec:values"),
                         SdfValueTypeNames->FloatArray)
        .Set(VtFloatArray{0.0f, 0.25f, 0.5f});
    base.CreateAttribute(TfToken("rigExec:defaultWeight"),
                         SdfValueTypeNames->Float)
        .Set(0.0f);

    const UsdPrim dynamic = stage->DefinePrim(
        SdfPath("/Asset/Rig/Weights/Driven"),
        TfToken("RigExecDynamicWeight"));
    dynamic.CreateRelationship(TfToken("rigExec:weightTarget"))
        .SetTargets({target});
    dynamic.CreateRelationship(TfToken("rigExec:baseWeight"))
        .SetTargets({base.GetPath()});
    dynamic.CreateAttribute(TfToken("rigExec:representation"),
                            SdfValueTypeNames->Token)
        .Set(TfToken("dense"));
    dynamic.CreateAttribute(TfToken("inputs:driver"),
                            SdfValueTypeNames->Float)
        .Set(0.5f);
    dynamic.CreateAttribute(TfToken("inputs:scale"),
                            SdfValueTypeNames->Float)
        .Set(1.5f);
    dynamic.CreateAttribute(TfToken("inputs:bias"),
                            SdfValueTypeNames->Float)
        .Set(0.125f);

    const UsdPrim mover = stage->DefinePrim(
        SdfPath("/Asset/Rig/Movers/M"), TfToken("RigExecMatrixMover"));
    CHECK(mover.ApplyAPI(TfToken("RigExecMoverAPI")));
    mover.CreateRelationship(TfToken("rigExec:moves")).SetTargets({target});
    mover.CreateRelationship(TfToken("rigExec:transform"))
        .SetTargets({joint.GetPath()});
    mover.GetAttribute(TfToken("inputs:defaultWeight")).Set(0.0f);
    mover.CreateRelationship(TfToken("rigExec:weightObject"))
        .SetTargets({dynamic.GetPath()});

    RigExecRigEvaluator evaluator(stage, SdfPath("/Asset/Rig"));
    evaluator.cpuParityMode = true;
    std::vector<std::string> errors;
    const bool compiled = evaluator.Compile(&errors);
    CHECK(compiled);
    for (const std::string &error : errors) {
        if (error.rfind("warning:", 0) != 0) {
            std::printf("dynamic weight compile diagnostic: %s\n",
                        error.c_str());
        }
    }
    if (!compiled) {
        return;
    }

    const RigExecRigPose pose = evaluator.Evaluate(UsdTimeCode::Default());
    CHECK(pose.valid);
    CHECK(pose.moverGraphParityMismatches == 0);
    CHECK(pose.moverGraphParityAgreements > 0);

    // (base * 0.5) * 1.5 + 0.125 gives {0.125, 0.3125, 0.5}.
    const VtVec3fArray expected = {
        GfVec3f(0, 1.25f, 0), GfVec3f(1, 3.125f, 0),
        GfVec3f(2, 5.0f, 0)};
    const auto checkResult = [&](const auto &results) {
        const auto it = results.find(target);
        CHECK(it != results.end());
        if (it == results.end()) {
            return;
        }
        CHECK(it->second.template IsHolding<VtVec3fArray>());
        if (!it->second.template IsHolding<VtVec3fArray>()) {
            return;
        }
        const VtVec3fArray &actual =
            it->second.template UncheckedGet<VtVec3fArray>();
        CHECK(actual.size() == expected.size());
        for (size_t i = 0;
             i < actual.size() && i < expected.size(); ++i) {
            CHECK(Near(GfVec3d(actual[i]), GfVec3d(expected[i])));
        }
    };
    checkResult(pose.movedProperties);
}

// SingleChainIK writes a complete joint chain as one atomic operation. Its
// common object envelope is therefore a one-element constant targeted at the
// mover prim, not a spatial field targeted at one of the joints. The value
// broadcasts to the whole solve and supersedes inputs:defaultWeight.
static void
TestSingleChainIkUniversalWeightObject()
{
    const SdfPath ikPath("/Asset/Rig/Movers/IK");
    const SdfPath weightPath("/Asset/Rig/Weights/IkEnvelope");
    const SdfPath jointPaths[] = {
        SdfPath("/Asset/Rig/Joints/Root"),
        SdfPath("/Asset/Rig/Joints/Root/Mid"),
        SdfPath("/Asset/Rig/Joints/Root/Mid/End")};

    const auto authorWeight = [&](const UsdStageRefPtr &stage,
                                  const TfToken &representation,
                                  const SdfPath &target, float value) {
        const UsdPrim weight = stage->DefinePrim(
            weightPath, TfToken("RigExecStaticWeight"));
        weight.CreateRelationship(TfToken("rigExec:weightTarget"))
            .SetTargets({target});
        weight.CreateAttribute(TfToken("rigExec:representation"),
                               SdfValueTypeNames->Token)
            .Set(representation);
        weight.CreateAttribute(TfToken("rigExec:defaultWeight"),
                               SdfValueTypeNames->Float)
            .Set(representation == "constant" ? value : 0.0f);
        if (representation == "dense" || representation == "sparse") {
            weight.CreateAttribute(TfToken("rigExec:values"),
                                   SdfValueTypeNames->FloatArray)
                .Set(VtFloatArray{value});
        }
        if (representation == "sparse") {
            weight.CreateAttribute(TfToken("rigExec:indices"),
                                   SdfValueTypeNames->IntArray)
                .Set(VtIntArray{0});
        }
        stage->GetPrimAtPath(ikPath)
            .GetRelationship(TfToken("rigExec:weightObject"))
            .SetTargets({weightPath});
    };

    const auto evaluate = [&](float scalar, bool bindObject) {
        const UsdStageRefPtr stage = BuildConstraintStage(nullptr);
        const UsdPrim ik = stage->GetPrimAtPath(ikPath);
        ik.GetAttribute(TfToken("inputs:defaultWeight")).Set(scalar);
        if (bindObject) {
            authorWeight(stage, TfToken("constant"), ikPath, 0.5f);
        }
        RigExecRigEvaluator evaluator(stage, SdfPath("/Asset/Rig"));
        std::vector<std::string> errors;
        const bool compiled = evaluator.Compile(&errors);
        if (!compiled) {
            for (const std::string &error : errors) {
                std::printf("single-chain weight compile error: %s\n",
                            error.c_str());
            }
        }
        CHECK(compiled);
        const RigExecRigPose pose =
            compiled ? evaluator.Evaluate(UsdTimeCode::Default())
                     : RigExecRigPose();
        CHECK(pose.valid);
        return pose;
    };

    // Scalar 0 would make the IK dormant. A bound 0.5 object must instead
    // produce exactly the same complete chain as scalar 0.5 with no object.
    const RigExecRigPose scalarZero = evaluate(0.0f, false);
    const RigExecRigPose scalarHalf = evaluate(0.5f, false);
    const RigExecRigPose objectHalf = evaluate(0.0f, true);
    size_t changedJoints = 0;
    for (const SdfPath &jointPath : jointPaths) {
        const auto zero = scalarZero.jointFramesFinal.find(jointPath);
        const auto expected = scalarHalf.jointFramesFinal.find(jointPath);
        const auto actual = objectHalf.jointFramesFinal.find(jointPath);
        CHECK(zero != scalarZero.jointFramesFinal.end());
        CHECK(expected != scalarHalf.jointFramesFinal.end());
        CHECK(actual != objectHalf.jointFramesFinal.end());
        if (zero == scalarZero.jointFramesFinal.end() ||
            expected == scalarHalf.jointFramesFinal.end() ||
            actual == objectHalf.jointFramesFinal.end()) {
            continue;
        }
        bool changed = false;
        for (size_t point = 0; point < actual->second.points.size(); ++point) {
            CHECK(Near(actual->second.points[point],
                       expected->second.points[point], 2e-4));
            changed = changed ||
                !Near(actual->second.points[point],
                      zero->second.points[point], 2e-4);
        }
        changedJoints += changed ? 1 : 0;
    }
    CHECK(changedJoints >= 2);

    // A multi-target atomic operation has no per-joint spatial cardinality.
    // Dense/sparse encodings are rejected even when they contain one value.
    for (const TfToken &representation :
         {TfToken("dense"), TfToken("sparse")}) {
        const UsdStageRefPtr stage = BuildConstraintStage(nullptr);
        authorWeight(stage, representation, ikPath, 0.5f);
        RigExecRigEvaluator evaluator(stage, SdfPath("/Asset/Rig"));
        std::vector<std::string> errors;
        CHECK(!evaluator.Compile(&errors));
        CHECK(std::any_of(
            errors.begin(), errors.end(), [](const std::string &error) {
                return error.find("atomic multi-target mover") !=
                           std::string::npos &&
                       error.find("constant") != std::string::npos;
            }));
    }

    // The operation-domain target is the IK mover itself. Targeting one joint
    // would make the result ordering-dependent and must fail at compile time.
    {
        const UsdStageRefPtr stage = BuildConstraintStage(nullptr);
        authorWeight(stage, TfToken("constant"), jointPaths[1], 0.5f);
        RigExecRigEvaluator evaluator(stage, SdfPath("/Asset/Rig"));
        std::vector<std::string> errors;
        CHECK(!evaluator.Compile(&errors));
        CHECK(std::any_of(
            errors.begin(), errors.end(), [](const std::string &error) {
                return error.find("weightTarget does not match mover target") !=
                       std::string::npos;
            }));
    }
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

// The usdview hierarchy is also the mover stack UI. Sibling rows therefore
// execute bottom-to-top, while a mover that contains other movers executes
// after every descendant in its own subtree. At every depth this is reverse
// composed sibling order plus post-order parent emission.
static void
TestReverseSiblingPostOrder()
{
    const UsdStageRefPtr stage = UsdStage::CreateInMemory();
    MakeXform(stage, SdfPath("/Asset"), Matrix());
    MakeXform(stage, SdfPath("/Asset/Target"), Matrix());
    stage->DefinePrim(SdfPath("/Asset/Sources"), TfToken("Scope"));
    stage->DefinePrim(SdfPath("/Asset/Rig"), TfToken("RigExecRoot"));
    const UsdPrim movers =
        stage->DefinePrim(SdfPath("/Asset/Rig/Movers"), TfToken("Scope"));

    auto make = [&](const char *relativePath, double sourceX) {
        const SdfPath moverPath =
            SdfPath("/Asset/Rig/Movers").AppendPath(
                SdfPath(relativePath));
        const SdfPath sourcePath =
            SdfPath("/Asset/Sources").AppendChild(
                moverPath.GetNameToken());
        MakeXform(stage, sourcePath, Matrix(GfVec3d(sourceX, 0, 0)));
        const UsdPrim mover = stage->DefinePrim(
            moverPath, TfToken("RigExecPositionConstraint"));
        CHECK(mover.ApplyAPI(TfToken("RigExecMoverAPI")));
        mover.CreateRelationship(TfToken("rigExec:moves"))
            .SetTargets({SdfPath("/Asset/Target")});
        mover.CreateRelationship(TfToken("rigExec:sources"))
            .SetTargets({sourcePath});
        return mover;
    };

    const UsdPrim siblingAbove = make("SiblingAbove", 50);
    const UsdPrim moverParent = make("MoverParent", 40);
    const UsdPrim childAbove = make("MoverParent/ChildAbove", 30);
    const UsdPrim childBelow = make("MoverParent/ChildBelow", 20);
    const UsdPrim siblingBelow = make("SiblingBelow", 10);
    CHECK(siblingAbove && moverParent && childAbove && childBelow &&
          siblingBelow);

    auto checkOrder = [](const RigExecRigEvaluator &evaluator,
                         const std::vector<SdfPath> &expected) {
        const auto &actual = evaluator.GetMoverOrder();
        CHECK(actual.size() == expected.size());
        for (size_t i = 0; i < std::min(actual.size(), expected.size()); ++i) {
            CHECK(actual[i].moverPath == expected[i]);
            CHECK(actual[i].ordinal == static_cast<int>(i));
        }
    };

    const SdfPath root("/Asset/Rig/Movers");
    RigExecRigEvaluator evaluator(stage, SdfPath("/Asset/Rig"));
    std::vector<std::string> errors;
    CHECK(evaluator.Compile(&errors));
    checkOrder(
        evaluator,
        {root.AppendChild(TfToken("SiblingBelow")),
         root.AppendPath(SdfPath("MoverParent/ChildBelow")),
         root.AppendPath(SdfPath("MoverParent/ChildAbove")),
         root.AppendChild(TfToken("MoverParent")),
         root.AppendChild(TfToken("SiblingAbove"))});
    const size_t naturalDigest = evaluator.GetBindingEpochDigest();
    const RigExecRigPose natural = evaluator.Evaluate(UsdTimeCode::Default());
    CHECK(natural.valid);
    CHECK(Near(natural.providerXforms.at(SdfPath("/Asset/Target"))
                   .ExtractTranslation(),
               GfVec3d(50, 0, 0)));

    // Reorder changes the displayed top-to-bottom order. Execution consumes
    // that final composed order in reverse at BOTH levels.
    movers.SetChildrenReorder(
        {TfToken("SiblingBelow"), TfToken("MoverParent"),
         TfToken("SiblingAbove")});
    moverParent.SetChildrenReorder(
        {TfToken("ChildBelow"), TfToken("ChildAbove")});
    CHECK(movers.GetChildrenNames() ==
          TfTokenVector({TfToken("SiblingBelow"), TfToken("MoverParent"),
                         TfToken("SiblingAbove")}));
    CHECK(moverParent.GetChildrenNames() ==
          TfTokenVector({TfToken("ChildBelow"), TfToken("ChildAbove")}));
    errors.clear();
    CHECK(evaluator.Compile(&errors));
    checkOrder(
        evaluator,
        {root.AppendChild(TfToken("SiblingAbove")),
         root.AppendPath(SdfPath("MoverParent/ChildAbove")),
         root.AppendPath(SdfPath("MoverParent/ChildBelow")),
         root.AppendChild(TfToken("MoverParent")),
         root.AppendChild(TfToken("SiblingBelow"))});
    CHECK(evaluator.GetBindingEpochDigest() != naturalDigest);
    const RigExecRigPose reordered =
        evaluator.Evaluate(UsdTimeCode::Default());
    CHECK(reordered.valid);
    CHECK(Near(reordered.providerXforms.at(SdfPath("/Asset/Target"))
                   .ExtractTranslation(),
               GfVec3d(10, 0, 0)));
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
        // Parent-relative: 0 then two 1-unit steps, world 0/1/2.
        const double restX[] = {0.0, 1.0, 1.0};
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

    // AutoDetect means animated translation/scale, not merely any animated
    // avar. A time-sampled scale must preserve the current scaled chain
    // lengths. Conversely, a static scale plus a time-sampled rotation must
    // still use rest-derived lengths: rotation is deliberately outside T/S
    // detection. Author a zero rotation sample so the two cases differ only
    // in which property carries the time sample.
    for (const bool animatedScale : {true, false}) {
        const UsdStageRefPtr stage = BuildConstraintStage(nullptr);
        const SdfPath root("/Asset/Rig/Joints/Root");
        const SdfPath mid("/Asset/Rig/Joints/Root/Mid");
        const SdfPath end("/Asset/Rig/Joints/Root/Mid/End");
        const SdfPath joints[] = {root, mid, end};
        // Parent-relative: 0 then two 1-unit steps, world 0/1/2.
        const double restX[] = {0.0, 1.0, 1.0};
        for (size_t i = 0; i < 3; ++i) {
            const UsdPrim joint = stage->GetPrimAtPath(joints[i]);
            joint.GetAttribute(TfToken("posed:space")).Clear();
            joint.CreateAttribute(TfToken("rest:space"),
                                  SdfValueTypeNames->Matrix4d)
                .Set(Matrix(GfVec3d(restX[i], 0, 0)));
        }

        const UsdPrim rootJoint = stage->GetPrimAtPath(root);
        const UsdAttribute sx =
            rootJoint.GetAttribute(TfToken("avars:sx"));
        CHECK(sx);
        if (animatedScale) {
            sx.Set(2.0, UsdTimeCode(1));
        } else {
            sx.Set(2.0);
            rootJoint.GetAttribute(TfToken("avars:rz"))
                .Set(0.0, UsdTimeCode(1));
        }
        stage->GetAttributeAtPath(
                 SdfPath("/Asset/Sources/Effector.xformOp:transform"))
            .Set(Matrix(GfVec3d(3, 0, 0)), UsdTimeCode(1));
        const UsdPrim ik =
            stage->GetPrimAtPath(SdfPath("/Asset/Rig/Movers/IK"));
        ik.GetAttribute(TfToken("rigExec:solverMode"))
            .Set(TfToken("singleChain"));
        ik.GetAttribute(TfToken("rigExec:evaluationMode"))
            .Set(TfToken("autoDetect"));

        RigExecRigEvaluator evaluator(stage, SdfPath("/Asset/Rig"));
        std::vector<std::string> errors;
        CHECK(evaluator.Compile(&errors));
        const RigExecRigPose pose = evaluator.Evaluate(UsdTimeCode(1));
        const auto solvedEnd = pose.jointFramesFinal.find(end);
        CHECK(solvedEnd != pose.jointFramesFinal.end());
        if (solvedEnd != pose.jointFramesFinal.end()) {
            const double expectedX = animatedScale ? 3.0 : 2.0;
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

    // A .points target on a prim that has no point set is not the geometry
    // domain, it is a mistake -- and must be reported as that rather than as
    // a bad transform provider. (A .points target on a real PointBased prim
    // is legal; see TestGeometryDomainTargetCompiles.)
    {
        const UsdStageRefPtr stage = UsdStage::CreateInMemory();
        MakeXform(stage, SdfPath("/Asset"), Matrix());
        stage->DefinePrim(SdfPath("/Asset/Geom"), TfToken("Scope"));
        MakeXform(stage, SdfPath("/Asset/Geom/NotPointBased"), Matrix());
        MakeXform(stage, SdfPath("/Asset/Source"), Matrix());
        stage->DefinePrim(SdfPath("/Asset/Rig"), TfToken("RigExecRoot"));
        stage->DefinePrim(SdfPath("/Asset/Rig/Movers"), TfToken("Scope"));
        const UsdPrim rotation = MakeConstraint(
            stage, "Rot", "RigExecRotationConstraint",
            {SdfPath("/Asset/Geom/NotPointBased")
                 .AppendProperty(TfToken("points"))});
        rotation.CreateRelationship(TfToken("rigExec:sources"))
            .SetTargets({SdfPath("/Asset/Source")});
        RigExecRigEvaluator evaluator(stage, SdfPath("/Asset/Rig"));
        std::vector<std::string> errors;
        CHECK(!evaluator.Compile(&errors));
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
// membership predicates must agree with it -- otherwise another operator
// can be added to one and forgotten in the other, which is the failure mode
// the table exists to remove.
static void
TestConstraintRegistryCoversTheSchema()
{
    const UsdSchemaRegistry &registry = UsdSchemaRegistry::GetInstance();
    for (const char *typeName :
         {"RigExecAimConstraint", "RigExecPositionConstraint",
          "RigExecRotationConstraint", "RigExecScaleConstraint",
          "RigExecParentConstraint", "RigExecSingleChainIkConstraint"}) {
        CHECK(registry.FindConcretePrimDefinition(TfToken(typeName)));
        CHECK(RigExecConstraintHandlerCount(TfToken(typeName)) == 1);
    }
    // Six rows, no more: an unregistered type must not resolve.
    CHECK(RigExecConstraintHandlerCount(TfToken("RigExecSmoothMover")) == 0);
    CHECK(RigExecConstraintHandlerTotal() == 6);
}

// No authored constraint property is silently ignored. rigExec:rotationOrder
// on a Position constraint used to compile and do nothing; it is now
// rejected, and so is any mask or offset naming a channel group the operator
// does not write.
static void
TestRotationOrderCapabilityIsRecorded()
{
    CHECK(RigExecConstraintUsesRotationOrder(
        TfToken("RigExecRotationConstraint")));
    CHECK(RigExecConstraintUsesRotationOrder(TfToken("RigExecAimConstraint")));
    CHECK(RigExecConstraintUsesRotationOrder(
        TfToken("RigExecParentConstraint")));
    CHECK(!RigExecConstraintUsesRotationOrder(
        TfToken("RigExecPositionConstraint")));
    CHECK(!RigExecConstraintUsesRotationOrder(
        TfToken("RigExecScaleConstraint")));
    CHECK(!RigExecConstraintUsesRotationOrder(TfToken("RigExecSmoothMover")));

    // Every authored channel property is checked against what the operator
    // actually honors. Each case names the property, the constraint type it
    // is authored on, and the substring the diagnostic must carry.
    enum class Kind { Token, Vector, Flag };
    struct RejectionCase {
        const char *type;
        const char *property;
        Kind kind;
        const char *expect;
    };
    const RejectionCase cases[] = {
        // Position writes translation: a rotation order and a rotation or
        // scale mask are all meaningless on it.
        {"RigExecPositionConstraint", "rigExec:rotationOrder", Kind::Token,
         "does not honor"},
        {"RigExecPositionConstraint", "inputs:affectRotationX", Kind::Flag,
         "different channel group"},
        {"RigExecPositionConstraint", "inputs:scaleOffset", Kind::Vector,
         "different channel group"},
        // Scale writes scale.
        {"RigExecScaleConstraint", "rigExec:rotationOrder", Kind::Token,
         "does not honor"},
        {"RigExecScaleConstraint", "inputs:translationOffset", Kind::Vector,
         "different channel group"},
        // Parent composes PER-SOURCE offset arrays, so the inherited scalar
        // offsets would be silently dropped on it.
        {"RigExecParentConstraint", "inputs:translationOffset", Kind::Vector,
         "per-source offset arrays"},
        // The legacy spelling that meant a different channel per operator.
        {"RigExecRotationConstraint", "inputs:affectX", Kind::Flag,
         "different channel on every operator"},
    };
    for (const RejectionCase &entry : cases) {
        const UsdStageRefPtr stage = UsdStage::CreateInMemory();
        MakeXform(stage, SdfPath("/Asset"), Matrix());
        MakeXform(stage, SdfPath("/Asset/Target"), Matrix());
        MakeXform(stage, SdfPath("/Asset/Source"), Matrix(GfVec3d(1, 0, 0)));
        stage->DefinePrim(SdfPath("/Asset/Rig"), TfToken("RigExecRoot"));
        stage->DefinePrim(SdfPath("/Asset/Rig/Movers"), TfToken("Scope"));
        const UsdPrim constraint = MakeConstraint(
            stage, "C", entry.type, {SdfPath("/Asset/Target")});
        constraint.CreateRelationship(TfToken("rigExec:sources"))
            .SetTargets({SdfPath("/Asset/Source")});
        switch (entry.kind) {
        case Kind::Token:
            constraint
                .CreateAttribute(TfToken(entry.property),
                                 SdfValueTypeNames->Token, /*custom=*/true)
                .Set(TfToken("ZYX"));
            break;
        case Kind::Vector:
            constraint
                .CreateAttribute(TfToken(entry.property),
                                 SdfValueTypeNames->Double3, /*custom=*/true)
                .Set(GfVec3d(0, 0, 0));
            break;
        case Kind::Flag:
            constraint
                .CreateAttribute(TfToken(entry.property),
                                 SdfValueTypeNames->Bool, /*custom=*/true)
                .Set(false);
            break;
        }
        RigExecRigEvaluator evaluator(stage, SdfPath("/Asset/Rig"));
        std::vector<std::string> errors;
        CHECK(!evaluator.Compile(&errors));
        const std::string expect = entry.expect;
        CHECK(std::any_of(errors.begin(), errors.end(),
                          [&expect](const std::string &error) {
                              return error.find(expect) != std::string::npos;
                          }));
    }

    // And the honored ones still compile: Rotation reads a rotation order and
    // a rotation mask, Parent reads all three mask groups.
    for (const char *type :
         {"RigExecRotationConstraint", "RigExecParentConstraint"}) {
        const UsdStageRefPtr stage = UsdStage::CreateInMemory();
        MakeXform(stage, SdfPath("/Asset"), Matrix());
        MakeXform(stage, SdfPath("/Asset/Target"), Matrix());
        MakeXform(stage, SdfPath("/Asset/Source"), Matrix(GfVec3d(1, 0, 0)));
        stage->DefinePrim(SdfPath("/Asset/Rig"), TfToken("RigExecRoot"));
        stage->DefinePrim(SdfPath("/Asset/Rig/Movers"), TfToken("Scope"));
        const UsdPrim constraint =
            MakeConstraint(stage, "C", type, {SdfPath("/Asset/Target")});
        constraint.CreateRelationship(TfToken("rigExec:sources"))
            .SetTargets({SdfPath("/Asset/Source")});
        constraint.GetAttribute(TfToken("rigExec:rotationOrder"))
            .Set(TfToken("ZYX"));
        constraint.GetAttribute(TfToken("inputs:affectRotationX")).Set(false);
        RigExecRigEvaluator evaluator(stage, SdfPath("/Asset/Rig"));
        std::vector<std::string> errors;
        CHECK(evaluator.Compile(&errors));
    }
}

// inputs:weight became inputs:defaultWeight. The old name is no longer part
// of the constraint schema, so an authored opinion would compose as a custom
// property and be silently ignored -- which is exactly the class of defect
// this family is being cleaned of. It must fail closed and name the new
// spelling.
static void
TestLegacyWeightSpellingIsRejected()
{
    const UsdStageRefPtr stage = UsdStage::CreateInMemory();
    MakeXform(stage, SdfPath("/Asset"), Matrix());
    MakeXform(stage, SdfPath("/Asset/Target"), Matrix());
    MakeXform(stage, SdfPath("/Asset/Source"), Matrix(GfVec3d(1, 0, 0)));
    stage->DefinePrim(SdfPath("/Asset/Rig"), TfToken("RigExecRoot"));
    stage->DefinePrim(SdfPath("/Asset/Rig/Movers"), TfToken("Scope"));
    const UsdPrim position = MakeConstraint(
        stage, "Pos", "RigExecPositionConstraint", {SdfPath("/Asset/Target")});
    position.CreateRelationship(TfToken("rigExec:sources"))
        .SetTargets({SdfPath("/Asset/Source")});
    position.CreateAttribute(TfToken("inputs:weight"),
                             SdfValueTypeNames->Float, /*custom=*/true)
        .Set(0.5f);

    RigExecRigEvaluator evaluator(stage, SdfPath("/Asset/Rig"));
    std::vector<std::string> errors;
    CHECK(!evaluator.Compile(&errors));
    CHECK(std::any_of(errors.begin(), errors.end(),
                      [](const std::string &error) {
                          return error.find("inputs:defaultWeight") !=
                                 std::string::npos;
                      }));

    // The envelope itself still drives: zero is a dormant pass-through and
    // one applies in full, under the new spelling.
    const UsdStageRefPtr clean = UsdStage::CreateInMemory();
    MakeXform(clean, SdfPath("/Asset"), Matrix());
    MakeXform(clean, SdfPath("/Asset/Target"), Matrix());
    MakeXform(clean, SdfPath("/Asset/Source"), Matrix(GfVec3d(10, 0, 0)));
    clean->DefinePrim(SdfPath("/Asset/Rig"), TfToken("RigExecRoot"));
    clean->DefinePrim(SdfPath("/Asset/Rig/Movers"), TfToken("Scope"));
    const UsdPrim pos2 = MakeConstraint(
        clean, "Pos", "RigExecPositionConstraint", {SdfPath("/Asset/Target")});
    pos2.CreateRelationship(TfToken("rigExec:sources"))
        .SetTargets({SdfPath("/Asset/Source")});
    for (const float envelope : {0.0f, 0.5f, 1.0f}) {
        pos2.GetAttribute(TfToken("inputs:defaultWeight")).Set(envelope);
        RigExecRigEvaluator e(clean, SdfPath("/Asset/Rig"));
        std::vector<std::string> errs;
        CHECK(e.Compile(&errs));
        const RigExecRigPose pose = e.Evaluate(UsdTimeCode::Default());
        const auto it = pose.providerXforms.find(SdfPath("/Asset/Target"));
        CHECK(it != pose.providerXforms.end());
        if (it != pose.providerXforms.end()) {
            CHECK(Near(it->second.ExtractTranslation(),
                       GfVec3d(10.0 * envelope, 0, 0)));
        }
    }
}

// A .points target names the geometry domain: the constraint deforms the
// point set instead of revising the prim's transform.
static void
TestGeometryDomainTargetCompiles()
{
    const UsdStageRefPtr stage = UsdStage::CreateInMemory();
    MakeXform(stage, SdfPath("/Asset"), Matrix());
    stage->DefinePrim(SdfPath("/Asset/Geom"), TfToken("Scope"));
    const UsdPrim mesh =
        stage->DefinePrim(SdfPath("/Asset/Geom/M"), TfToken("Mesh"));
    mesh.CreateAttribute(TfToken("points"), SdfValueTypeNames->Point3fArray)
        .Set(VtVec3fArray{{1, 0, 0}, {0, 2, 0}, {0, 0, 3}});
    MakeXform(stage, SdfPath("/Asset/Source"),
              Matrix(GfVec3d(0, 0, 0), GfRotation(GfVec3d(0, 1, 0), 90.0)));
    stage->DefinePrim(SdfPath("/Asset/Rig"), TfToken("RigExecRoot"));
    stage->DefinePrim(SdfPath("/Asset/Rig/Movers"), TfToken("Scope"));
    const SdfPath pointsTarget =
        SdfPath("/Asset/Geom/M").AppendProperty(TfToken("points"));
    const UsdPrim rotation = MakeConstraint(
        stage, "Rot", "RigExecRotationConstraint", {pointsTarget});
    rotation.CreateRelationship(TfToken("rigExec:sources"))
        .SetTargets({SdfPath("/Asset/Source")});

    RigExecRigEvaluator evaluator(stage, SdfPath("/Asset/Rig"));
    std::vector<std::string> errors;
    CHECK(evaluator.Compile(&errors));
    const RigExecRigPose pose = evaluator.Evaluate(UsdTimeCode::Default());
    CHECK(pose.valid);

    // A constant one-element field is target-compatible with the transform
    // domain. It supersedes (rather than multiplies) the scalar fallback.
    const UsdStageRefPtr xformStage = UsdStage::CreateInMemory();
    MakeXform(xformStage, SdfPath("/Asset"), Matrix());
    MakeXform(xformStage, SdfPath("/Asset/Target"), Matrix());
    MakeXform(xformStage, SdfPath("/Asset/Source"), Matrix(GfVec3d(10, 0, 0)));
    xformStage->DefinePrim(SdfPath("/Asset/Rig"), TfToken("RigExecRoot"));
    xformStage->DefinePrim(SdfPath("/Asset/Rig/Movers"), TfToken("Scope"));
    const UsdPrim pos2 = MakeConstraint(
        xformStage, "Pos", "RigExecPositionConstraint",
        {SdfPath("/Asset/Target")});
    pos2.CreateRelationship(TfToken("rigExec:sources"))
        .SetTargets({SdfPath("/Asset/Source")});
    pos2.GetAttribute(TfToken("inputs:defaultWeight")).Set(0.25f);
    const UsdPrim weight = xformStage->DefinePrim(
        SdfPath("/Asset/Rig/Weights/W"), TfToken("RigExecStaticWeight"));
    weight.CreateRelationship(TfToken("rigExec:weightTarget"))
        .SetTargets({SdfPath("/Asset/Target")});
    weight.CreateAttribute(TfToken("rigExec:representation"),
                           SdfValueTypeNames->Token)
        .Set(TfToken("constant"));
    weight.CreateAttribute(TfToken("rigExec:defaultWeight"),
                           SdfValueTypeNames->Float)
        .Set(0.5f);
    pos2.CreateRelationship(TfToken("rigExec:weightObject"))
        .SetTargets({weight.GetPath()});
    RigExecRigEvaluator xformEvaluator(xformStage, SdfPath("/Asset/Rig"));
    std::vector<std::string> xformErrors;
    CHECK(xformEvaluator.Compile(&xformErrors));
    const RigExecRigPose xformPose =
        xformEvaluator.Evaluate(UsdTimeCode::Default());
    CHECK(xformPose.valid);
    const auto xformResult =
        xformPose.providerXforms.find(SdfPath("/Asset/Target"));
    CHECK(xformResult != xformPose.providerXforms.end());
    if (xformResult != xformPose.providerXforms.end()) {
        CHECK(Near(xformResult->second.ExtractTranslation(),
                   GfVec3d(5, 0, 0)));
    }
}

// Geometry-domain constraints apply solved deltas through point revisions.
// A bound dense object is the total per-point envelope on this
// path and must supersede, not multiply, a conflicting scalar fallback.
static void
TestGeometryConstraintDenseEnvelopeSupersedesScalar()
{
    const UsdStageRefPtr stage = UsdStage::CreateInMemory();
    MakeXform(stage, SdfPath("/Asset"), Matrix());
    const UsdPrim mesh =
        stage->DefinePrim(SdfPath("/Asset/Geom/M"), TfToken("Mesh"));
    const VtVec3fArray rest = {
        GfVec3f(0, 0, 0), GfVec3f(1, 0, 0), GfVec3f(2, 0, 0)};
    mesh.CreateAttribute(TfToken("points"), SdfValueTypeNames->Point3fArray)
        .Set(rest);
    MakeXform(stage, SdfPath("/Asset/Source"),
              Matrix(GfVec3d(0, 10, 0)));
    stage->DefinePrim(SdfPath("/Asset/Rig"), TfToken("RigExecRoot"));
    stage->DefinePrim(SdfPath("/Asset/Rig/Movers"), TfToken("Scope"));
    stage->DefinePrim(SdfPath("/Asset/Rig/Weights"), TfToken("Scope"));

    const SdfPath target("/Asset/Geom/M.points");
    const UsdPrim position = MakeConstraint(
        stage, "Pos", "RigExecPositionConstraint", {target});
    position.CreateRelationship(TfToken("rigExec:sources"))
        .SetTargets({SdfPath("/Asset/Source")});
    position.GetAttribute(TfToken("inputs:defaultWeight")).Set(0.25f);

    const UsdPrim weight = stage->DefinePrim(
        SdfPath("/Asset/Rig/Weights/PointField"),
        TfToken("RigExecStaticWeight"));
    weight.CreateRelationship(TfToken("rigExec:weightTarget"))
        .SetTargets({target});
    weight.CreateAttribute(TfToken("rigExec:representation"),
                           SdfValueTypeNames->Token)
        .Set(TfToken("dense"));
    weight.CreateAttribute(TfToken("rigExec:values"),
                           SdfValueTypeNames->FloatArray)
        .Set(VtFloatArray{0.0f, 0.5f, 1.0f});
    weight.CreateAttribute(TfToken("rigExec:defaultWeight"),
                           SdfValueTypeNames->Float)
        .Set(0.0f);
    position.CreateRelationship(TfToken("rigExec:weightObject"))
        .SetTargets({weight.GetPath()});

    RigExecRigEvaluator evaluator(stage, SdfPath("/Asset/Rig"));
    std::vector<std::string> errors;
    const bool compiled = evaluator.Compile(&errors);
    CHECK(compiled);
    for (const std::string &error : errors) {
        if (error.rfind("warning:", 0) != 0) {
            std::printf("geometry constraint compile diagnostic: %s\n",
                        error.c_str());
        }
    }
    if (!compiled) {
        return;
    }

    const RigExecRigPose pose = evaluator.Evaluate(UsdTimeCode::Default());
    CHECK(pose.valid);
    const auto it = pose.movedProperties.find(target);
    CHECK(it != pose.movedProperties.end());
    if (it == pose.movedProperties.end()) {
        return;
    }
    CHECK(it->second.IsHolding<VtVec3fArray>());
    if (!it->second.IsHolding<VtVec3fArray>()) {
        return;
    }
    const VtVec3fArray &moved = it->second.UncheckedGet<VtVec3fArray>();
    const VtVec3fArray expected = {
        GfVec3f(0, 0, 0), GfVec3f(1, 5, 0), GfVec3f(2, 10, 0)};
    CHECK(moved.size() == expected.size());
    for (size_t i = 0;
         i < moved.size() && i < expected.size(); ++i) {
        CHECK(Near(GfVec3d(moved[i]), GfVec3d(expected[i])));
    }
    CHECK(pose.movedPropertiesCpu.find(target) ==
          pose.movedPropertiesCpu.end());
}

// The design's defining property: the target spelling picks WHERE the answer
// lands, not what it is.
//
// It is EXACT at full envelope, which is the case the design is really about
// -- </Geom/M> and </Geom/M.points> are then two spellings of one result.
//
// At an intermediate envelope the two domains blend in different spaces and
// deliberately diverge: the transform domain lerps DECOMPOSED CHANNELS (half
// of a 90-degree rotation is a 45-degree rotation), while the geometry domain
// lerps POSITIONS, which is the chord rather than the arc. That is not a
// defect to fix here -- it is the same weighting RigExecApplyWeightedMatrix
// gives every matrix mover, so a constraint deforms points exactly like the
// deformers it sits beside. TestGeometryEnvelopeIsChordLerp below pins it.
static void
TestTransformAndGeometrySpellingsAgree()
{
    const VtVec3fArray rest{{1, 0, 0}, {0, 2, 0}, {0, 0, 3}, {1, 1, 1}};
    for (const GfMatrix4d &base : {Matrix(), Matrix(GfVec3d(10, 0, 0),
            GfRotation(GfVec3d(0, 0, 1), 90.0)), Matrix(GfVec3d(1,2,3),
            GfRotation(GfVec3d(1,0,0), 25.0), GfVec3d(2,3,4))})
    for (const char *type : {"RigExecRotationConstraint", "RigExecPositionConstraint",
                             "RigExecParentConstraint"}) {
        std::vector<GfVec3d> viaTransform;
        std::vector<GfVec3d> viaPoints;
        for (const bool geometry : {false, true}) {
            const UsdStageRefPtr stage = UsdStage::CreateInMemory();
            MakeXform(stage, SdfPath("/Asset"), Matrix());
            stage->DefinePrim(SdfPath("/Asset/Geom"), TfToken("Scope"));
            const UsdPrim mesh = stage->DefinePrim(
                SdfPath("/Asset/Geom/M"), TfToken("Mesh"));
            mesh.CreateAttribute(TfToken("points"),
                                 SdfValueTypeNames->Point3fArray)
                .Set(rest);
            UsdGeomXformable(mesh).MakeMatrixXform().Set(base);
            MakeXform(stage, SdfPath("/Asset/Source"),
                      Matrix(GfVec3d(2, 3, 4),
                             GfRotation(GfVec3d(0, 1, 0), 90.0)));
            stage->DefinePrim(SdfPath("/Asset/Rig"), TfToken("RigExecRoot"));
            stage->DefinePrim(SdfPath("/Asset/Rig/Movers"), TfToken("Scope"));
            const SdfPath pointsPath =
                SdfPath("/Asset/Geom/M").AppendProperty(TfToken("points"));
            const UsdPrim rot = MakeConstraint(
                stage, "Rot", type,
                {geometry ? pointsPath : SdfPath("/Asset/Geom/M")});
            rot.CreateRelationship(TfToken("rigExec:sources"))
                .SetTargets({SdfPath("/Asset/Source")});
            rot.GetAttribute(TfToken("inputs:defaultWeight")).Set(1.0f);

            RigExecRigEvaluator evaluator(stage, SdfPath("/Asset/Rig"));
            std::vector<std::string> errors;
            CHECK(evaluator.Compile(&errors));
            const RigExecRigPose pose =
                evaluator.Evaluate(UsdTimeCode::Default());
            CHECK(pose.valid);
            if (geometry) {
                const auto it = pose.movedProperties.find(pointsPath);
                CHECK(it != pose.movedProperties.end());
                if (it != pose.movedProperties.end()) {
                    const VtVec3fArray moved =
                        it->second.Get<VtVec3fArray>();
                    for (const GfVec3f &p : moved) {
                        viaPoints.push_back(base.TransformAffine(GfVec3d(p)));
                    }
                }
            } else {
                const auto it =
                    pose.providerXforms.find(SdfPath("/Asset/Geom/M"));
                CHECK(it != pose.providerXforms.end());
                if (it != pose.providerXforms.end()) {
                    for (const GfVec3f &p : rest) {
                        viaTransform.push_back(
                            it->second.TransformAffine(GfVec3d(p)));
                    }
                }
            }
        }
        CHECK(viaTransform.size() == rest.size());
        CHECK(viaPoints.size() == rest.size());
        // Not vacuous: the constraint must actually have moved the points.
        CHECK(!Near(viaPoints.empty() ? GfVec3d(0) : viaPoints[0],
                    GfVec3d(rest[0])));
        for (size_t i = 0;
             i < viaTransform.size() && i < viaPoints.size(); ++i) {
            CHECK(Near(viaTransform[i], viaPoints[i], 1e-4));
        }
    }
}

// The geometry domain's envelope is a per-point lerp toward the fully-solved
// position -- the same weighting every matrix mover applies. Pinned exactly
// so it is a specified behavior rather than an accident, and so that a change
// to channel-space blending has to edit this assertion deliberately.
static void
TestGeometryEnvelopeIsChordLerp()
{
    const VtVec3fArray rest{{1, 0, 0}, {0, 0, 3}};
    const UsdStageRefPtr stage = UsdStage::CreateInMemory();
    MakeXform(stage, SdfPath("/Asset"), Matrix());
    stage->DefinePrim(SdfPath("/Asset/Geom"), TfToken("Scope"));
    const UsdPrim mesh =
        stage->DefinePrim(SdfPath("/Asset/Geom/M"), TfToken("Mesh"));
    mesh.CreateAttribute(TfToken("points"), SdfValueTypeNames->Point3fArray)
        .Set(rest);
    MakeXform(stage, SdfPath("/Asset/Source"),
              Matrix(GfVec3d(0, 0, 0), GfRotation(GfVec3d(0, 1, 0), 90.0)));
    stage->DefinePrim(SdfPath("/Asset/Rig"), TfToken("RigExecRoot"));
    stage->DefinePrim(SdfPath("/Asset/Rig/Movers"), TfToken("Scope"));
    const SdfPath pointsPath =
        SdfPath("/Asset/Geom/M").AppendProperty(TfToken("points"));
    const UsdPrim rot = MakeConstraint(
        stage, "Rot", "RigExecRotationConstraint", {pointsPath});
    rot.CreateRelationship(TfToken("rigExec:sources"))
        .SetTargets({SdfPath("/Asset/Source")});
    rot.GetAttribute(TfToken("inputs:defaultWeight")).Set(0.5f);

    RigExecRigEvaluator evaluator(stage, SdfPath("/Asset/Rig"));
    evaluator.cpuParityMode = true;
    std::vector<std::string> errors;
    CHECK(evaluator.Compile(&errors));
    const RigExecRigPose pose = evaluator.Evaluate(UsdTimeCode::Default());
    CHECK(pose.valid);
    const auto it = pose.movedProperties.find(pointsPath);
    CHECK(it != pose.movedProperties.end());
    if (it == pose.movedProperties.end()) {
        return;
    }
    const VtVec3fArray moved = it->second.Get<VtVec3fArray>();
    CHECK(moved.size() == rest.size());
    if (moved.size() != rest.size()) {
        return;
    }
    // rotY(90) sends (1,0,0) -> (0,0,-1) and (0,0,3) -> (3,0,0). At envelope
    // 0.5 each point sits halfway along the straight line between the two.
    CHECK(Near(GfVec3d(moved[0]), GfVec3d(0.5, 0, -0.5), 1e-4));
    CHECK(Near(GfVec3d(moved[1]), GfVec3d(1.5, 0, 1.5), 1e-4));

    // The constraint now participates in the ordinary point revision graph,
    // with a scalar application oracle and the same envelope contract.
    CHECK(pose.movedPropertiesCpu.count(pointsPath) == 1);
    CHECK(pose.moverGraphParityAgreements == 1);
    CHECK(pose.moverGraphParityMismatches == 0);
}

// Competing writers are keyed by the exact target, and that is correct.
//
// Two writers of the SAME points set compose in mover order. A
// transform-domain constraint on /M and a geometry-domain one on /M.points
// are NOT competing: they write different output domains, and the prim's
// matrix composes over its points by construction. Keying them together --
// which an earlier draft of this design proposed -- would reject the very
// case of a mesh that is both moved and deformed.
static void
TestDomainsOnOnePrimDoNotCompete()
{
    const auto build = [](const char *firstTarget, const char *secondTarget) {
        const UsdStageRefPtr stage = UsdStage::CreateInMemory();
        MakeXform(stage, SdfPath("/Asset"), Matrix());
        stage->DefinePrim(SdfPath("/Asset/Geom"), TfToken("Scope"));
        const UsdPrim mesh =
            stage->DefinePrim(SdfPath("/Asset/Geom/M"), TfToken("Mesh"));
        mesh.CreateAttribute(TfToken("points"),
                             SdfValueTypeNames->Point3fArray)
            .Set(VtVec3fArray{{1, 0, 0}, {0, 2, 0}, {0, 0, 3}});
        MakeXform(stage, SdfPath("/Asset/Source"),
                  Matrix(GfVec3d(0, 0, 0),
                         GfRotation(GfVec3d(0, 1, 0), 90.0)));
        stage->DefinePrim(SdfPath("/Asset/Rig"), TfToken("RigExecRoot"));
        stage->DefinePrim(SdfPath("/Asset/Rig/Movers"), TfToken("Scope"));
        for (const auto &entry : {std::make_pair("A", firstTarget),
                                  std::make_pair("B", secondTarget)}) {
            const UsdPrim c = MakeConstraint(
                stage, entry.first, "RigExecRotationConstraint",
                {SdfPath(entry.second)});
            c.CreateRelationship(TfToken("rigExec:sources"))
                .SetTargets({SdfPath("/Asset/Source")});
        }
        return stage;
    };

    // Same points set twice: legal, and ordered by the composed hierarchy.
    // Two movers writing one target is an ordinary stack; the order is the
    // reverse-sibling post-order walk of the final composed namespace, so
    // nothing has to be declared for it to be well defined.
    {
        const UsdStageRefPtr stage =
            build("/Asset/Geom/M.points", "/Asset/Geom/M.points");
        RigExecRigEvaluator evaluator(stage, SdfPath("/Asset/Rig"));
        std::vector<std::string> errors;
        CHECK(evaluator.Compile(&errors));
        const RigExecRigPose pose =
            evaluator.Evaluate(UsdTimeCode::Default());
        CHECK(pose.valid);
    }

    // A parent child-order instruction is a convenience that redirects that
    // order, not a precondition for having one. Authoring it reverses which
    // constraint applies last, and the compiler reads the result off the
    // composed namespace exactly as before -- it never inspects HOW the
    // order came to be.
    {
        const UsdStageRefPtr stage =
            build("/Asset/Geom/M.points", "/Asset/Geom/M.points");
        const UsdPrim movers =
            stage->GetPrimAtPath(SdfPath("/Asset/Rig/Movers"));
        CHECK(movers);
        const TfTokenVector natural = movers.GetChildrenNames();
        CHECK(natural.size() == 2);
        movers.SetChildrenReorder({TfToken("B"), TfToken("A")});
        const TfTokenVector reordered = movers.GetChildrenNames();
        CHECK(reordered.size() == 2);
        if (natural.size() == 2 && reordered.size() == 2) {
            CHECK(reordered[0] == natural[1]);
            CHECK(reordered[1] == natural[0]);
        }
        RigExecRigEvaluator evaluator(stage, SdfPath("/Asset/Rig"));
        std::vector<std::string> errors;
        CHECK(evaluator.Compile(&errors));
    }

    // Different domains on one prim: legal, and both apply.
    {
        const UsdStageRefPtr stage =
            build("/Asset/Geom/M", "/Asset/Geom/M.points");
        RigExecRigEvaluator evaluator(stage, SdfPath("/Asset/Rig"));
        std::vector<std::string> errors;
        CHECK(evaluator.Compile(&errors));
        const RigExecRigPose pose =
            evaluator.Evaluate(UsdTimeCode::Default());
        CHECK(pose.valid);
        CHECK(pose.providerXforms.count(SdfPath("/Asset/Geom/M")) == 1);
        CHECK(pose.movedProperties.count(
                  SdfPath("/Asset/Geom/M").AppendProperty(
                      TfToken("points"))) == 1);
    }
}

// The stack is dynamic: unwiring one mover takes that mover out of it and
// leaves the rest running. Disconnecting rigExec:moves is the ordinary
// interactive edit -- pulling a wire in a node graph -- and it must not take
// down every other constraint in the rig with it.
static void
TestUnwiringOneMoverLeavesTheRestRunning()
{
    // Rotation applies first, Parent stacks on top; each writes a channel the
    // other can be checked by. Unwire one at a time and confirm the survivor
    // still drives.
    const auto build = [](const char *unwire) {
        const UsdStageRefPtr stage = UsdStage::CreateInMemory();
        MakeXform(stage, SdfPath("/Asset"), Matrix());
        MakeXform(stage, SdfPath("/Asset/Target"), Matrix());
        MakeXform(stage, SdfPath("/Asset/RotSource"),
                  Matrix(GfVec3d(0, 0, 0),
                         GfRotation(GfVec3d(0, 1, 0), 90.0)));
        MakeXform(stage, SdfPath("/Asset/ParSource"),
                  Matrix(GfVec3d(7, 0, 0)));
        stage->DefinePrim(SdfPath("/Asset/Rig"), TfToken("RigExecRoot"));
        stage->DefinePrim(SdfPath("/Asset/Rig/Movers"), TfToken("Scope"));
        const UsdPrim par = MakeConstraint(
            stage, "Par", "RigExecParentConstraint",
            {SdfPath("/Asset/Target")});
        par.CreateRelationship(TfToken("rigExec:sources"))
            .SetTargets({SdfPath("/Asset/ParSource")});
        // This test composes Parent translation with a later Rotation writer.
        // Parent is the top row and therefore executes last; make its intended
        // translation-only channel ownership explicit so it does not replace
        // the Rotation result with its identity source orientation.
        for (const char *axis : {"X", "Y", "Z"}) {
            par.CreateAttribute(
                   TfToken(std::string("inputs:affectRotation") + axis),
                   SdfValueTypeNames->Bool)
                .Set(false);
        }
        const UsdPrim rot = MakeConstraint(
            stage, "Rot", "RigExecRotationConstraint",
            {SdfPath("/Asset/Target")});
        rot.CreateRelationship(TfToken("rigExec:sources"))
            .SetTargets({SdfPath("/Asset/RotSource")});
        if (std::string(unwire) == "Par") {
            par.GetRelationship(TfToken("rigExec:moves")).SetTargets({});
        } else if (std::string(unwire) == "Rot") {
            rot.GetRelationship(TfToken("rigExec:moves")).SetTargets({});
        }
        return stage;
    };

    // Both wired: Parent supplies the translation, Rotation the orientation.
    {
        const UsdStageRefPtr stage = build("");
        RigExecRigEvaluator evaluator(stage, SdfPath("/Asset/Rig"));
        std::vector<std::string> errors;
        CHECK(evaluator.Compile(&errors));
        const RigExecRigPose pose =
            evaluator.Evaluate(UsdTimeCode::Default());
        const auto it = pose.providerXforms.find(SdfPath("/Asset/Target"));
        CHECK(it != pose.providerXforms.end());
        if (it != pose.providerXforms.end()) {
            CHECK(Near(it->second.ExtractTranslation(), GfVec3d(7, 0, 0)));
            CHECK(Near(it->second.TransformDir(GfVec3d(1, 0, 0)),
                       GfVec3d(0, 0, -1)));
        }
    }

    // Unwire the Rotation constraint: the rig still compiles and the Parent
    // constraint still drives the translation.
    {
        const UsdStageRefPtr stage = build("Rot");
        RigExecRigEvaluator evaluator(stage, SdfPath("/Asset/Rig"));
        std::vector<std::string> errors;
        CHECK(evaluator.Compile(&errors));
        const RigExecRigPose pose =
            evaluator.Evaluate(UsdTimeCode::Default());
        CHECK(pose.valid);
        const auto it = pose.providerXforms.find(SdfPath("/Asset/Target"));
        CHECK(it != pose.providerXforms.end());
        if (it != pose.providerXforms.end()) {
            CHECK(Near(it->second.ExtractTranslation(), GfVec3d(7, 0, 0)));
            // ...and the unwired one contributes nothing.
            CHECK(Near(it->second.TransformDir(GfVec3d(1, 0, 0)),
                       GfVec3d(1, 0, 0)));
        }
        // Inert, but never silent.
        CHECK(std::any_of(errors.begin(), errors.end(),
                          [](const std::string &m) {
                              return m.find("/Asset/Rig/Movers/Rot") !=
                                         std::string::npos &&
                                     m.find("no moves targets") !=
                                         std::string::npos;
                          }));
    }

    // Unwire the Parent constraint instead: Rotation still drives.
    {
        const UsdStageRefPtr stage = build("Par");
        RigExecRigEvaluator evaluator(stage, SdfPath("/Asset/Rig"));
        std::vector<std::string> errors;
        CHECK(evaluator.Compile(&errors));
        const RigExecRigPose pose =
            evaluator.Evaluate(UsdTimeCode::Default());
        CHECK(pose.valid);
        const auto it = pose.providerXforms.find(SdfPath("/Asset/Target"));
        CHECK(it != pose.providerXforms.end());
        if (it != pose.providerXforms.end()) {
            CHECK(Near(it->second.TransformDir(GfVec3d(1, 0, 0)),
                       GfVec3d(0, 0, -1)));
            CHECK(Near(it->second.ExtractTranslation(), GfVec3d(0, 0, 0)));
        }
    }

    // Unwire BOTH: still a legal rig, just one that publishes nothing.
    {
        const UsdStageRefPtr stage = build("Rot");
        stage->GetPrimAtPath(SdfPath("/Asset/Rig/Movers/Par"))
            .GetRelationship(TfToken("rigExec:moves"))
            .SetTargets({});
        RigExecRigEvaluator evaluator(stage, SdfPath("/Asset/Rig"));
        std::vector<std::string> errors;
        CHECK(evaluator.Compile(&errors));
    }
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
        // An unwired constraint is INERT, not fatal -- the stack is dynamic,
        // and one disconnected mover must not take the rig down. What the
        // original contract was really protecting is preserved: it does not
        // become a grouping prim SILENTLY.
        CHECK(evaluator.Compile(&errors));
        CHECK(std::any_of(errors.begin(), errors.end(),
                          [](const std::string &error) {
                              return error.find("no moves targets") !=
                                     std::string::npos &&
                                     error.find("inert") != std::string::npos;
                          }));
        // Inert means it publishes nothing.
        const RigExecRigPose pose =
            evaluator.Evaluate(UsdTimeCode::Default());
        CHECK(pose.providerXforms.empty());
        CHECK(pose.movedProperties.empty());
    }

    // A rig that found NO mover prims at all is still the misconfiguration
    // that error describes -- a rig root pointed at the wrong prim.
    {
        const UsdStageRefPtr stage = UsdStage::CreateInMemory();
        MakeXform(stage, SdfPath("/Asset"), Matrix());
        stage->DefinePrim(SdfPath("/Asset/Rig"), TfToken("RigExecRoot"));
        stage->DefinePrim(SdfPath("/Asset/Rig/Movers"), TfToken("Scope"));
        RigExecRigEvaluator evaluator(stage, SdfPath("/Asset/Rig"));
        std::vector<std::string> errors;
        CHECK(!evaluator.Compile(&errors));
        CHECK(std::any_of(errors.begin(), errors.end(),
                          [](const std::string &error) {
                              return error.find("publishes no outputs") !=
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

static void
TestSolverBindsJointsUnderAnyScope()
{
    // Solvers are discovered by TYPE anywhere beneath the rig: the scope
    // they sit under is an authoring convenience, not identity. A
    // TwoBoneIk placed under Movers (no "Solvers" scope anywhere) must
    // still bind its joints and pose them when the effector moves.
    const UsdStageRefPtr stage = UsdStage::CreateInMemory();
    MakeXform(stage, SdfPath("/Asset"), Matrix());
    MakeXform(stage, SdfPath("/Asset/Target"), Matrix());
    MakeXform(stage, SdfPath("/Asset/Source"), Matrix(GfVec3d(1, 0, 0)));
    stage->DefinePrim(SdfPath("/Asset/Rig"), TfToken("RigExecRoot"));
    stage->DefinePrim(SdfPath("/Asset/Rig/Movers"), TfToken("Scope"));
    // One wired mover so the rig publishes outputs and compiles.
    const UsdPrim mover = MakeConstraint(
        stage, "Par", "RigExecParentConstraint",
        {SdfPath("/Asset/Target")});
    mover.CreateRelationship(TfToken("rigExec:sources"))
        .SetTargets({SdfPath("/Asset/Source")});

    const auto makeControl = [&](const char *name, double tx, double tz) {
        const UsdPrim prim = stage->DefinePrim(
            SdfPath(std::string("/Asset/Rig/Controls/") + name),
            TfToken("RigExecControl"));
        CHECK(prim);
        prim.CreateAttribute(TfToken("rest:tx"), SdfValueTypeNames->Double)
            .Set(tx);
        prim.CreateAttribute(TfToken("rest:tz"), SdfValueTypeNames->Double)
            .Set(tz);
        prim.CreateAttribute(TfToken("avars:tx"), SdfValueTypeNames->Double)
            .Set(0.0);
        return prim;
    };
    const UsdPrim rootCtl = makeControl("Root", 0.0, 0.0);
    const UsdPrim effCtl = makeControl("Eff", 5.0, 0.0);
    const UsdPrim poleCtl = makeControl("Pole", 2.5, 2.0);

    stage->DefinePrim(SdfPath("/Asset/Rig/Joints/Shoulder"),
                      TfToken("RigExecJoint"));
    // Parent-local rests of 3 and 3: the bones the solve measures. They
    // used to be authored as absolute rigExec:upperLength/lowerLength,
    // which is now a compile error -- the lengths come from these rests.
    stage->DefinePrim(SdfPath("/Asset/Rig/Joints/Shoulder/Elbow"),
                      TfToken("RigExecJoint"))
        .CreateAttribute(TfToken("rest:tx"), SdfValueTypeNames->Double)
        .Set(3.0);
    stage->DefinePrim(SdfPath("/Asset/Rig/Joints/Shoulder/Elbow/Wrist"),
                      TfToken("RigExecJoint"))
        .CreateAttribute(TfToken("rest:tx"), SdfValueTypeNames->Double)
        .Set(3.0);

    // Deliberately NOT under any "Solvers" scope.
    const UsdPrim ik = stage->DefinePrim(
        SdfPath("/Asset/Rig/Movers/LegIK"), TfToken("RigExecTwoBoneIk"));
    CHECK(ik);
    ik.CreateRelationship(TfToken("rigExec:rootControl"))
        .SetTargets({rootCtl.GetPath()});
    ik.CreateRelationship(TfToken("rigExec:effectorControl"))
        .SetTargets({effCtl.GetPath()});
    ik.CreateRelationship(TfToken("rigExec:poleControl"))
        .SetTargets({poleCtl.GetPath()});
    ik.CreateRelationship(TfToken("rigExec:joints"))
        .SetTargets({SdfPath("/Asset/Rig/Joints/Shoulder"),
                     SdfPath("/Asset/Rig/Joints/Shoulder/Elbow"),
                     SdfPath("/Asset/Rig/Joints/Shoulder/Elbow/Wrist")});

    RigExecRigEvaluator evaluator(stage, SdfPath("/Asset/Rig"));
    std::vector<std::string> errors;
    CHECK(evaluator.Compile(&errors));
    for (const std::string &error : errors) {
        std::printf("  compile message: %s\n", error.c_str());
    }
    CHECK(errors.empty());

    const auto wristOrigin = [&](UsdTimeCode time) {
        const RigExecRigPose pose = evaluator.Evaluate(time);
        CHECK(pose.valid);
        const auto it = pose.jointFramesFinal.find(
            SdfPath("/Asset/Rig/Joints/Shoulder/Elbow/Wrist"));
        CHECK(it != pose.jointFramesFinal.end());
        return it != pose.jointFramesFinal.end()
                   ? it->second.Origin()
                   : GfVec3d(0);
    };
    // Rest: the wrist sits exactly on the effector rest at (5, 0, 0).
    CHECK(Near(wristOrigin(UsdTimeCode::Default()), GfVec3d(5, 0, 0)));
    // Posed: +1 on the effector reaches (6, 0, 0), full 3+3 extension.
    effCtl.GetAttribute(TfToken("avars:tx")).Set(1.0);
    CHECK(Near(wristOrigin(UsdTimeCode::Default()), GfVec3d(6, 0, 0)));
}

static void
TestIncompleteSolverLeavesJointsVisible()
{
    // Removing a solver's control relationships must not make its joints
    // disappear: with no aggregate element to extract, each bound joint
    // keeps its natural rest-chain frame (and stays drawable) while a
    // diagnostic names the gap. A present-but-degenerate solve is the
    // opposite case and still propagates; this test covers only the
    // incomplete solver, whose kernel returns an empty aggregate.
    const UsdStageRefPtr stage = UsdStage::CreateInMemory();
    MakeXform(stage, SdfPath("/Asset"), Matrix());
    stage->DefinePrim(SdfPath("/Asset/Rig"), TfToken("RigExecRoot"));
    stage->DefinePrim(SdfPath("/Asset/Rig/Controls"), TfToken("Scope"));
    stage->DefinePrim(SdfPath("/Asset/Rig/Joints"), TfToken("Scope"));

    // Controls exist but nothing wires them to the solver.
    for (const char *name : {"Root", "Eff", "Pole"}) {
        stage->DefinePrim(
            SdfPath(std::string("/Asset/Rig/Controls/") + name),
            TfToken("RigExecControl"));
    }

    const SdfPath shoulder("/Asset/Rig/Joints/Shoulder");
    const SdfPath elbow("/Asset/Rig/Joints/Shoulder/Elbow");
    const SdfPath wrist("/Asset/Rig/Joints/Shoulder/Elbow/Wrist");
    // Parent-relative: three 1-unit steps, world 1/2/3.
    const double restTx[3] = {1.0, 1.0, 1.0};
    const SdfPath joints[3] = {shoulder, elbow, wrist};
    for (int i = 0; i < 3; ++i) {
        const UsdPrim prim =
            stage->DefinePrim(joints[i], TfToken("RigExecJoint"));
        CHECK(prim);
        prim.CreateAttribute(TfToken("rest:tx"), SdfValueTypeNames->Double)
            .Set(restTx[i]);
    }

    // Solver claims the joints but wires no root/effector/pole controls.
    const UsdPrim ik = stage->DefinePrim(
        SdfPath("/Asset/Rig/Solvers/LegIK"), TfToken("RigExecTwoBoneIk"));
    CHECK(ik);
    ik.CreateRelationship(TfToken("rigExec:joints"))
        .SetTargets({shoulder, elbow, wrist});

    RigExecRigEvaluator evaluator(stage, SdfPath("/Asset/Rig"));
    std::vector<std::string> errors;
    CHECK(evaluator.Compile(&errors));
    for (const std::string &error : errors) {
        std::printf("  compile message: %s\n", error.c_str());
    }
    CHECK(errors.empty());

    const RigExecRigPose pose = evaluator.Evaluate(UsdTimeCode::Default());
    CHECK(pose.valid);
    // Every bound joint stays on its rest chain. rest:tx/ty/tz is the
    // offset from the parent frame provider, so each joint sits at the
    // accumulated chain -- three 1-unit steps reaching x = 1, 2, 3.
    const GfVec3d expected[3] = {
        GfVec3d(1, 0, 0), GfVec3d(2, 0, 0), GfVec3d(3, 0, 0)};
    for (int i = 0; i < 3; ++i) {
        const auto frame = pose.jointFramesFinal.find(joints[i]);
        CHECK(frame != pose.jointFramesFinal.end());
        if (frame == pose.jointFramesFinal.end()) {
            continue;
        }
        CHECK(frame->second.IsValid());
        CHECK(!frame->second.IsDegenerate());
        CHECK(Near(frame->second.Origin(), expected[i]));
        // The matrix is what imaging draws: omitted means invisible.
        CHECK(pose.jointMatricesFinal.count(joints[i]) == 1);
    }
    CHECK(std::any_of(
        pose.diagnostics.begin(), pose.diagnostics.end(),
        [](const std::string &diagnostic) {
            return diagnostic.find("fell back to its rest chain") !=
                   std::string::npos;
        }));
}

static void
TestTwoBoneIkImpliedLengths(bool throughBlend)
{
    // An unauthored absolute length is measured from the bound joints'
    // rest positions, plus the authored offset; an authored absolute is
    // exact and implies nothing. Joints rest on the X axis at 0/3/7, so
    // the implied bones are 3 and 4 (reach 7).
    const UsdStageRefPtr stage = UsdStage::CreateInMemory();
    MakeXform(stage, SdfPath("/Asset"), Matrix());
    stage->DefinePrim(SdfPath("/Asset/Rig"), TfToken("RigExecRoot"));
    stage->DefinePrim(SdfPath("/Asset/Rig/Controls"), TfToken("Scope"));

    const auto makeControl = [&](const char *name, double tx, double tz) {
        const UsdPrim prim = stage->DefinePrim(
            SdfPath(std::string("/Asset/Rig/Controls/") + name),
            TfToken("RigExecControl"));
        CHECK(prim);
        prim.CreateAttribute(TfToken("rest:tx"), SdfValueTypeNames->Double)
            .Set(tx);
        prim.CreateAttribute(TfToken("rest:tz"), SdfValueTypeNames->Double)
            .Set(tz);
        prim.CreateAttribute(TfToken("avars:tx"), SdfValueTypeNames->Double)
            .Set(0.0);
        return prim;
    };
    makeControl("Root", 0.0, 0.0);
    const UsdPrim effCtl = makeControl("Eff", 7.0, 0.0);
    makeControl("Pole", 3.5, 2.0);
    if (throughBlend) {
        makeControl("Mid", 3.0, 0.0);
    }

    const SdfPath shoulder("/Asset/Rig/Joints/Shoulder");
    const SdfPath elbow("/Asset/Rig/Joints/Shoulder/Elbow");
    const SdfPath wrist("/Asset/Rig/Joints/Shoulder/Elbow/Wrist");
    // Parent-relative: a 3-unit upper and a 4-unit lower, world 0/3/7.
    const double restTx[3] = {0.0, 3.0, 4.0};
    const SdfPath joints[3] = {shoulder, elbow, wrist};
    for (int i = 0; i < 3; ++i) {
        const UsdPrim prim =
            stage->DefinePrim(joints[i], TfToken("RigExecJoint"));
        CHECK(prim);
        prim.CreateAttribute(TfToken("rest:tx"), SdfValueTypeNames->Double)
            .Set(restTx[i]);
    }

    const UsdPrim ik = stage->DefinePrim(
        SdfPath("/Asset/Rig/Solvers/LegIK"), TfToken("RigExecTwoBoneIk"));
    CHECK(ik);
    ik.CreateRelationship(TfToken("rigExec:rootControl"))
        .SetTargets({SdfPath("/Asset/Rig/Controls/Root")});
    ik.CreateRelationship(TfToken("rigExec:effectorControl"))
        .SetTargets({SdfPath("/Asset/Rig/Controls/Eff")});
    ik.CreateRelationship(TfToken("rigExec:poleControl"))
        .SetTargets({SdfPath("/Asset/Rig/Controls/Pole")});
    if (throughBlend) {
        // Only the blend owns the output joints -- rigExec:joints is an
        // exclusive output claim, so the IK cannot list them too. It finds
        // their rests by following its own output edge to the consumer
        // that does own them.
        const UsdPrim fk = stage->DefinePrim(
            SdfPath("/Asset/Rig/Solvers/FK"), TfToken("RigExecFkChain"));
        fk.CreateRelationship(TfToken("rigExec:controls"))
            .SetTargets({SdfPath("/Asset/Rig/Controls/Root"),
                         SdfPath("/Asset/Rig/Controls/Mid"),
                         effCtl.GetPath()});
        const UsdPrim blend = stage->DefinePrim(
            SdfPath("/Asset/Rig/Solvers/Blend"),
            TfToken("RigExecBlendPointFrames"));
        blend.CreateRelationship(TfToken("rigExec:inputA"))
            .SetTargets({fk.GetPath()});
        blend.CreateRelationship(TfToken("rigExec:inputB"))
            .SetTargets({ik.GetPath()});
        blend.GetAttribute(TfToken("inputs:weight")).Set(1.0f);
        blend.CreateRelationship(TfToken("rigExec:joints"))
            .SetTargets({shoulder, elbow, wrist});
    }
    // The IK names its chain either way. When the blend poses those
    // joints this is a REST reference -- the claim check defers to the
    // unconsumed blend -- and it is what gives the kernel the rests it
    // measures its bone lengths from.
    ik.CreateRelationship(TfToken("rigExec:joints"))
        .SetTargets({shoulder, elbow, wrist});

    RigExecRigEvaluator evaluator(stage, SdfPath("/Asset/Rig"));
    std::vector<std::string> errors;
    CHECK(evaluator.Compile(&errors));
    CHECK(errors.empty());

    const auto origins = [&](UsdTimeCode time, GfVec3d out[3]) {
        const RigExecRigPose pose = evaluator.Evaluate(time);
        CHECK(pose.valid);
        for (int i = 0; i < 3; ++i) {
            const auto it = pose.jointFramesFinal.find(joints[i]);
            CHECK(it != pose.jointFramesFinal.end());
            out[i] = it != pose.jointFramesFinal.end()
                ? it->second.Origin()
                : GfVec3d(0);
        }
        return pose;
    };
    const auto hasDiagnostic = [](const RigExecRigPose &pose,
                                  const std::string &text) {
        return std::any_of(
            pose.diagnostics.begin(), pose.diagnostics.end(),
            [&](const std::string &diagnostic) {
                return diagnostic.find(text) != std::string::npos;
            });
    };

    // Implied 3/4, goal at full reach: elbow exactly mid-chain.
    GfVec3d atRest[3];
    const RigExecRigPose restPose =
        origins(UsdTimeCode::Default(), atRest);
    CHECK(Near(atRest[0], GfVec3d(0, 0, 0)));
    CHECK(Near(atRest[1], GfVec3d(3, 0, 0)));
    CHECK(Near(atRest[2], GfVec3d(7, 0, 0)));

    // Edit the existing rest channels on this same evaluator. The root and
    // goal controls remain fixed; only the chain's binding rest changes.
    // Both the new bend and the new segment lengths must be visible without
    // rebuilding the structural graph or requiring a time change.
    const size_t restEpoch = evaluator.GetBindingEpochDigest();
    const UsdAttribute elbowRest = stage->GetAttributeAtPath(
        elbow.AppendProperty(TfToken("rest:tx")));
    const UsdAttribute wristRest = stage->GetAttributeAtPath(
        wrist.AppendProperty(TfToken("rest:tx")));
    elbowRest.Set(4.0);
    wristRest.Set(5.0);
    GfVec3d edited[3];
    const RigExecRigPose editedPose =
        origins(UsdTimeCode::Default(), edited);
    CHECK(std::abs((edited[1] - edited[0]).GetLength() - 4.0) < 1e-4);
    CHECK(std::abs((edited[2] - edited[1]).GetLength() - 5.0) < 1e-4);
    CHECK(Near(edited[2], GfVec3d(7, 0, 0)));
    CHECK(!Near(edited[1], atRest[1]));
    CHECK(evaluator.GetBindingEpochDigest() == restEpoch);

    // Rest samples also update when scrubbing in either direction. Editing
    // an already sampled time must invalidate the cached rest request too.
    elbowRest.Set(3.0, UsdTimeCode(1));
    elbowRest.Set(5.0, UsdTimeCode(2));
    wristRest.Set(4.0, UsdTimeCode(1));
    wristRest.Set(6.0, UsdTimeCode(2));
    GfVec3d sampled[3];
    origins(UsdTimeCode(2), sampled);
    CHECK(std::abs((sampled[1] - sampled[0]).GetLength() - 5.0) < 1e-4);
    CHECK(std::abs((sampled[2] - sampled[1]).GetLength() - 6.0) < 1e-4);
    elbowRest.Set(4.0, UsdTimeCode(2));
    origins(UsdTimeCode(2), sampled);
    CHECK(std::abs((sampled[1] - sampled[0]).GetLength() - 4.0) < 1e-4);
    // The lower bone is UNCHANGED by the upper edit: rest is parent-
    // relative, so moving the elbow carries the wrist with it instead of
    // stretching the segment between them.
    CHECK(std::abs((sampled[2] - sampled[1]).GetLength() - 6.0) < 1e-4);
    origins(UsdTimeCode(1), sampled);
    CHECK(Near(sampled[1], GfVec3d(3, 0, 0)));
    CHECK(Near(sampled[2], GfVec3d(7, 0, 0)));
    CHECK(evaluator.GetBindingEpochDigest() == restEpoch);
    elbowRest.Clear();
    wristRest.Clear();
    elbowRest.Set(3.0);
    wristRest.Set(4.0);

    // Offset only touches its own bone: +1 on lower reaches (8, 0, 0)
    // with segments 3 and 5. Offsets are values, so no recompile.
    ik.CreateAttribute(TfToken("rigExec:lowerLengthOffset"),
                       SdfValueTypeNames->Double)
        .Set(1.0);
    effCtl.GetAttribute(TfToken("avars:tx")).Set(1.0);
    GfVec3d offset[3];
    const RigExecRigPose offsetPose =
        origins(UsdTimeCode::Default(), offset);
    CHECK(Near(offset[2], GfVec3d(8, 0, 0)));
    CHECK(std::abs((offset[1] - offset[0]).GetLength() - 3.0) < 1e-4);
    CHECK(std::abs((offset[2] - offset[1]).GetLength() - 5.0) < 1e-4);

    // An absolute length cannot be authored at all. It used to be an
    // opt-out that froze the bone against rest edits, which read as a
    // broken middle joint rather than as a mode switch -- and it could
    // not be discovered from the property editor either, which showed
    // the schema fallback of 1 while the solve ran on the measured
    // value. Compile now rejects it and names the knob that works.
    ik.CreateAttribute(TfToken("rigExec:upperLength"),
                       SdfValueTypeNames->Double)
        .Set(2.5);
    {
        RigExecRigEvaluator rejecting(stage, SdfPath("/Asset/Rig"));
        std::vector<std::string> rejected;
        CHECK(!rejecting.Compile(&rejected));
        CHECK(std::any_of(
            rejected.begin(), rejected.end(),
            [](const std::string &error) {
                return error.find("was removed from the schema") !=
                           std::string::npos &&
                       error.find("rigExec:upperLengthOffset") !=
                           std::string::npos;
            }));
    }
    ik.GetAttribute(TfToken("rigExec:upperLength")).Clear();

    // Clearing it hands the bone back to the rests with no other edit.
    GfVec3d cleared[3];
    const RigExecRigPose clearedPose =
        origins(UsdTimeCode::Default(), cleared);
    CHECK(std::abs((cleared[1] - cleared[0]).GetLength() - 3.0) < 1e-4);
}

static void
TestTwoBoneIkRestFrameInputs()
{
    {
        const UsdStageRefPtr stage = UsdStage::CreateInMemory();
        const auto provider = [&](const char *path, const char *type,
                                  double tx) {
            const UsdPrim prim = stage->DefinePrim(SdfPath(path), TfToken(type));
            prim.GetAttribute(TfToken("rest:tx")).Set(tx);
            prim.GetAttribute(TfToken("rest:rx")).Set(0.0);
            return prim;
        };
        const UsdPrim root = provider("/Root", "RigExecControl", 0.0);
        const UsdPrim goal = provider("/Goal", "RigExecControl", 6.0);
        const UsdPrim pole = provider("/Pole", "RigExecControl", 2.0);
        const UsdPrim jointRoot = provider("/JointRoot", "RigExecJoint", 0.0);
        const UsdPrim jointMid = provider("/JointMid", "RigExecJoint", 3.0);
        const UsdPrim jointEnd = provider("/JointEnd", "RigExecJoint", 7.0);
        const UsdPrim ik = stage->DefinePrim(
            SdfPath("/IK"), TfToken("RigExecTwoBoneIk"));
        ik.GetRelationship(TfToken("rigExec:rootControl"))
            .SetTargets({root.GetPath()});
        ik.GetRelationship(TfToken("rigExec:effectorControl"))
            .SetTargets({goal.GetPath()});
        ik.GetRelationship(TfToken("rigExec:poleControl"))
            .SetTargets({pole.GetPath()});
        // Lengths are measured from the joint rests (0/3/7 through the
        // remap below): a 3-unit upper and a 4-unit lower.
        // Custom remaps are accepted by evaluator bindings even for IK.
        ik.GetRelationship(TfToken("rigExec:joints"))
            .SetTargets({jointEnd.GetPath(), jointRoot.GetPath(),
                         jointMid.GetPath()});
        ik.CreateAttribute(TfToken("rigExec:jointElements"),
                           SdfValueTypeNames->IntArray)
            .Set(VtIntArray{2, 0, 1});
        RigExecTapSet taps(stage);
        const RigExecTapId tap = taps.Add(RigExecValueAddress::Prim(
            ik.GetPath(), TfToken("computePointFrameArray")));
        CHECK(taps.Prepare());
        const auto solve = [&]() {
            const RigExecSnapshot snapshot = taps.Evaluate(UsdTimeCode::Default());
            CHECK(snapshot.IsValid() && snapshot.IsComplete());
            return snapshot.Get<RigExecPointFrameArray>(tap);
        };
        const auto before = solve();
        CHECK(before.frames.size() == 3 && before.rests.size() == 3);
        if (before.frames.size() != 3 || before.rests.size() != 3) {
            return;
        }
        CHECK(Near(before.rests[1][0], GfVec3d(3, 0, 0)));
        CHECK(Near(before.rests[2][0], GfVec3d(7, 0, 0)));
        CHECK(before.frames[1].Origin()[1] > 1.0);
        CHECK(std::abs(before.frames[1].Origin()[2]) < 1e-6);

        // The collinear pole selects the root JOINT's rest up. The root
        // control is unchanged, so this catches a missing rest dependency.
        jointRoot.GetAttribute(TfToken("rest:rx")).Set(90.0);
        jointMid.GetAttribute(TfToken("rest:tx")).Set(4.0);
        const auto edited = solve();
        CHECK(edited.frames.size() == 3 && edited.rests.size() == 3);
        if (edited.frames.size() != 3 || edited.rests.size() != 3) {
            return;
        }
        CHECK(edited.frames[1].Origin()[2] > 1.0);
        CHECK(std::abs(edited.frames[1].Origin()[1]) < 1e-6);
        CHECK(Near(edited.rests[0][2] - edited.rests[0][0], GfVec3d(0, 0, 1)));
        CHECK(Near(edited.rests[1][0], GfVec3d(4, 0, 0)));
        // Moving the mid rest RE-PROPORTIONS the upper bone: lengths are
        // measured from the rests on every evaluation, and there is no
        // longer an authored absolute that could hold the old 3.0.
        CHECK(std::abs((edited.frames[1].Origin() -
                        edited.frames[0].Origin()).GetLength() - 4.0) < 1e-6);
    }
}

static void
TestAggregateSolverValueUpdates()
{
    const UsdStageRefPtr stage = UsdStage::CreateInMemory();
    const auto control = [&](const char *path, double tx) {
        const UsdPrim prim = stage->DefinePrim(SdfPath(path), TfToken("RigExecControl"));
        prim.GetAttribute(TfToken("rest:tx")).Set(tx);
        prim.GetAttribute(TfToken("avars:ty")).Set(0.0);
        return prim;
    };
    const UsdPrim start = control("/Start", 0.0);
    const UsdPrim mid = control("/Mid", 2.0);
    const UsdPrim end = control("/End", 4.0);
    const UsdPrim fk = stage->DefinePrim(SdfPath("/FK"), TfToken("RigExecFkChain"));
    fk.GetRelationship(TfToken("rigExec:controls"))
        .SetTargets({start.GetPath(), mid.GetPath(), end.GetPath()});
    const UsdPrim twist = stage->DefinePrim(
        SdfPath("/Twist"), TfToken("RigExecTwistDistribution"));
    twist.GetRelationship(TfToken("rigExec:start")).SetTargets({start.GetPath()});
    twist.GetRelationship(TfToken("rigExec:end")).SetTargets({end.GetPath()});
    twist.GetAttribute(TfToken("rigExec:weights")).Set(VtFloatArray{0, 0.5f, 1});
    const UsdPrim blend = stage->DefinePrim(
        SdfPath("/Blend"), TfToken("RigExecBlendPointFrames"));
    blend.GetRelationship(TfToken("rigExec:inputA")).SetTargets({fk.GetPath()});
    blend.GetRelationship(TfToken("rigExec:inputB")).SetTargets({twist.GetPath()});
    blend.GetAttribute(TfToken("inputs:weight")).Set(0.5f);

    RigExecTapSet taps(stage);
    const RigExecTapId tap = taps.Add(RigExecValueAddress::Prim(
        blend.GetPath(), TfToken("computePointFrameArray")));
    CHECK(taps.Prepare());
    const auto middle = [&]() {
        const auto snapshot = taps.Evaluate(UsdTimeCode::Default());
        CHECK(snapshot.IsValid() && snapshot.IsComplete());
        const auto frames = snapshot.Get<RigExecPointFrameArray>(tap);
        CHECK(frames.GetSize() == 3);
        return frames.GetSize() == 3 ? frames.frames[1].Origin() : GfVec3d(0);
    };
    CHECK(Near(middle(), GfVec3d(2, 0, 0)));
    // FK follows live control rest inputs, twist follows live sample weights,
    // and the dependent blend must invalidate on each value edit.
    mid.GetAttribute(TfToken("rest:tx")).Set(3.0);
    CHECK(Near(middle(), GfVec3d(2.5, 0, 0)));
    twist.GetAttribute(TfToken("rigExec:weights")).Set(VtFloatArray{0, 0.25f, 1});
    CHECK(Near(middle(), GfVec3d(2, 0, 0)));
    end.GetAttribute(TfToken("rest:tx")).Set(8.0);
    CHECK(Near(middle(), GfVec3d(2.5, 0, 0)));
    blend.GetAttribute(TfToken("inputs:weight")).Set(1.0f);
    CHECK(Near(middle(), GfVec3d(2, 0, 0)));
    end.GetAttribute(TfToken("avars:ty")).Set(4.0);
    CHECK(Near(middle(), GfVec3d(2, 1, 0)));
}

static void
TestDeepSolverDependencySchedule()
{
    const UsdStageRefPtr stage = UsdStage::CreateInMemory();
    stage->DefinePrim(SdfPath("/Asset"), TfToken("Xform"));
    stage->DefinePrim(SdfPath("/Asset/Rig"), TfToken("RigExecRoot"));
    const UsdPrim source = stage->DefinePrim(
        SdfPath("/Asset/Rig/Controls/Source"), TfToken("RigExecControl"));
    source.GetAttribute(TfToken("avars:tx")).Set(2.0);
    const int depth = 96;
    SdfPath previousSolver, previousJoint, finalSolver;
    int inheritedOffset = 0;
    for (int i = 0; i < depth; ++i) {
        const SdfPath jointPath("/Asset/Rig/Joints/J" + std::to_string(i));
        const UsdPrim joint = stage->DefinePrim(jointPath, TfToken("RigExecJoint"));
        const UsdPrim child = stage->DefinePrim(
            jointPath.AppendChild(TfToken("Input")), TfToken("RigExecControl"));
        child.GetAttribute(TfToken("rest:tx")).Set(1.0);
        child.GetAttribute(TfToken("purpose")).Set(TfToken("guide"));
        const SdfPath solverPath("/Asset/Rig/Solvers/S" + std::to_string(i));
        if (i > 0 && i % 8 == 0) {
            // Direct aggregate dependencies interleave with joint-mediated
            // dependencies; duplicate input edges must schedule only once.
            const UsdPrim blend = stage->DefinePrim(
                solverPath, TfToken("RigExecBlendPointFrames"));
            blend.GetRelationship(TfToken("rigExec:inputA"))
                .SetTargets({previousSolver});
            blend.GetRelationship(TfToken("rigExec:inputB"))
                .SetTargets({previousSolver});
            blend.GetAttribute(TfToken("inputs:weight")).Set(0.5f);
            blend.GetRelationship(TfToken("rigExec:joints"))
                .SetTargets({jointPath});
        } else {
            const UsdPrim twist = stage->DefinePrim(
                solverPath, TfToken("RigExecTwistDistribution"));
            const SdfPath input = i == 0 ? source.GetPath()
                : previousJoint.AppendChild(TfToken("Input"));
            twist.GetRelationship(TfToken("rigExec:start")).SetTargets({input});
            twist.GetRelationship(TfToken("rigExec:end")).SetTargets({input});
            twist.GetRelationship(TfToken("rigExec:joints")).SetTargets({jointPath});
            if (i > 0) {
                ++inheritedOffset;
            }
        }
        previousJoint = jointPath;
        previousSolver = solverPath;
        finalSolver = solverPath;
    }
    RigExecRigEvaluator evaluator(stage, SdfPath("/Asset/Rig"));
    std::vector<std::string> errors;
    CHECK(evaluator.Compile(&errors));
    CHECK(errors.empty());
    const size_t epoch = evaluator.GetBindingEpochDigest();
    const auto evaluate = [&](double sourceX) {
        source.GetAttribute(TfToken("avars:tx")).Set(sourceX);
        const RigExecRigPose pose = evaluator.Evaluate(UsdTimeCode::Default());
        CHECK(pose.valid && pose.solverOverridesConverged);
        CHECK(pose.solverEvaluations == size_t(depth));
        CHECK(pose.solverOverrideRounds == size_t(depth));
        CHECK(evaluator.GetBindingEpochDigest() == epoch);
        const auto joint = pose.jointFramesFinal.find(previousJoint);
        CHECK(joint != pose.jointFramesFinal.end());
        if (joint != pose.jointFramesFinal.end()) {
            CHECK(Near(joint->second.Origin(), GfVec3d(sourceX + inheritedOffset, 0, 0)));
        }
        const auto guide = pose.solverFrames.find(finalSolver);
        CHECK(guide != pose.solverFrames.end() && guide->second.size() == 1);
        if (guide != pose.solverFrames.end() && guide->second.size() == 1) {
            CHECK(Near(guide->second[0].Origin(), GfVec3d(sourceX + inheritedOffset, 0, 0)));
        }
    };
    evaluate(2.0);
    evaluate(5.0);
    evaluate(-3.0);

    // An unchanged pull reuses every solver result. Editing a rest input
    // halfway along the graph dirties only the downstream dependency levels.
    const RigExecRigPose unchanged = evaluator.Evaluate(UsdTimeCode::Default());
    CHECK(unchanged.valid && unchanged.solverEvaluations == 0);
    stage->GetAttributeAtPath(SdfPath("/Asset/Rig/Joints/J46/Input.rest:tx"))
        .Set(2.0);
    const RigExecRigPose tail = evaluator.Evaluate(UsdTimeCode::Default());
    CHECK(tail.valid);
    CHECK(tail.solverEvaluations == size_t(depth - 47));
    CHECK(evaluator.GetBindingEpochDigest() == epoch);
    const auto tailJoint = tail.jointFramesFinal.find(previousJoint);
    CHECK(tailJoint != tail.jointFramesFinal.end());
    if (tailJoint != tail.jointFramesFinal.end()) {
        CHECK(Near(tailJoint->second.Origin(), GfVec3d(-3 + inheritedOffset + 1, 0, 0)));
    }
    stage->GetAttributeAtPath(SdfPath("/Asset/Rig/Joints/J95/Input.rest:tx"))
        .Set(11.0);
    const RigExecRigPose unrelated = evaluator.Evaluate(UsdTimeCode::Default());
    CHECK(unrelated.valid && unrelated.solverEvaluations == 0);

    // Rewire an existing endpoint attribute without changing output count.
    // The same evaluator must rebuild its DAG and remove upstream levels
    // from this solver's prerequisite set.
    const UsdPrim rewired = stage->GetPrimAtPath(SdfPath("/Asset/Rig/Solvers/S47"));
    rewired.GetRelationship(TfToken("rigExec:start"))
        .SetTargets({source.GetPath()});
    rewired.GetRelationship(TfToken("rigExec:end"))
        .SetTargets({source.GetPath()});
    const RigExecRigPose afterRewire = evaluator.Evaluate(UsdTimeCode::Default());
    CHECK(afterRewire.valid);
    CHECK(evaluator.GetBindingEpochDigest() != epoch);
    CHECK(afterRewire.solverOverrideRounds < size_t(depth));
    const auto rewiredJoint = afterRewire.jointFramesFinal.find(
        SdfPath("/Asset/Rig/Joints/J47"));
    CHECK(rewiredJoint != afterRewire.jointFramesFinal.end());
    if (rewiredJoint != afterRewire.jointFramesFinal.end()) {
        CHECK(Near(rewiredJoint->second.Origin(), GfVec3d(-3, 0, 0)));
    }

    // Restore the edge using Evaluate's automatic topology refresh before
    // closing a real feedback cycle below.
    rewired.GetRelationship(TfToken("rigExec:start"))
        .SetTargets({SdfPath("/Asset/Rig/Joints/J46/Input")});
    rewired.GetRelationship(TfToken("rigExec:end"))
        .SetTargets({SdfPath("/Asset/Rig/Joints/J46/Input")});
    CHECK(evaluator.Evaluate(UsdTimeCode::Default()).valid);

    // A cycle hidden through a control's namespace parent must be rejected
    // at compilation, rather than discovered after many refinement rounds.
    const UsdPrim first = stage->GetPrimAtPath(SdfPath("/Asset/Rig/Solvers/S0"));
    first.GetRelationship(TfToken("rigExec:start"))
        .SetTargets({previousJoint.AppendChild(TfToken("Input"))});
    const RigExecRigPose cyclic = evaluator.Evaluate(UsdTimeCode::Default());
    CHECK(!cyclic.valid);
    CHECK(HasDiagnostic(cyclic, "solver dependency cycle"));
}

static void
TestSolverTransitiveConnectionInvalidation()
{
    const UsdStageRefPtr stage = UsdStage::CreateInMemory();
    stage->DefinePrim(SdfPath("/Asset"), TfToken("Xform"));
    stage->DefinePrim(SdfPath("/Asset/Rig"), TfToken("RigExecRoot"));
    const auto control = [&](const char *name, double x, double y) {
        const UsdPrim prim = stage->DefinePrim(
            SdfPath(std::string("/Asset/Rig/Controls/") + name),
            TfToken("RigExecControl"));
        prim.GetAttribute(TfToken("rest:tx")).Set(x);
        prim.GetAttribute(TfToken("rest:ty")).Set(y);
        return prim.GetPath();
    };
    const SdfPath root = control("Root", 0, 0);
    const SdfPath goal = control("Goal", 8, 0);
    const SdfPath pole = control("Pole", 0, 1);
    SdfPathVector joints;
    for (int i = 0; i < 3; ++i) {
        const UsdPrim joint = stage->DefinePrim(
            SdfPath("/Asset/Rig/Joints/J" + std::to_string(i)),
            TfToken("RigExecJoint"));
        joint.GetAttribute(TfToken("rest:tx")).Set(double(2 * i));
        joints.push_back(joint.GetPath());
    }
    const UsdPrim ik = stage->DefinePrim(
        SdfPath("/Asset/Rig/Solvers/IK"), TfToken("RigExecTwoBoneIk"));
    ik.GetRelationship(TfToken("rigExec:rootControl")).SetTargets({root});
    ik.GetRelationship(TfToken("rigExec:effectorControl")).SetTargets({goal});
    ik.GetRelationship(TfToken("rigExec:poleControl")).SetTargets({pole});
    ik.GetRelationship(TfToken("rigExec:joints")).SetTargets(joints);
    // Implied lengths ensure this request uses Exec value overrides, whose
    // repeated invalidation callbacks alone do not renew their interest.
    const auto driver = [&](const char *name) {
        return stage->DefinePrim(SdfPath(std::string("/Drivers/") + name), TfToken("Scope"))
            .CreateAttribute(TfToken("value"), SdfValueTypeNames->Float);
    };
    const UsdAttribute relay = driver("Relay");
    const UsdAttribute a = driver("A");
    const UsdAttribute b = driver("B");
    const UsdAttribute c = driver("C");
    a.Set(0.0f);
    b.Set(0.5f);
    c.Set(0.25f);
    relay.SetConnections({a.GetPath()});
    const UsdAttribute stretch = ik.GetAttribute(TfToken("inputs:stretch"));
    stretch.SetConnections({relay.GetPath()});
    RigExecRigEvaluator evaluator(stage, SdfPath("/Asset/Rig"));
    std::vector<std::string> errors;
    CHECK(evaluator.Compile(&errors));
    CHECK(errors.empty());
    const auto evaluate = [&](double expected) {
        const RigExecRigPose pose = evaluator.Evaluate(UsdTimeCode::Default());
        CHECK(pose.valid);
        const auto end = pose.jointFramesFinal.find(joints[2]);
        CHECK(end != pose.jointFramesFinal.end());
        if (end != pose.jointFramesFinal.end()) {
            CHECK(Near(end->second.Origin(), GfVec3d(expected, 0, 0)));
        }
        return pose;
    };
    evaluate(4);
    const size_t epoch = evaluator.GetBindingEpochDigest();
    a.Set(1.0f);
    evaluate(8);
    a.Set(0.5f);
    CHECK(evaluate(6).solverEvaluations == 1);
    CHECK(evaluator.GetBindingEpochDigest() == epoch);

    // Same-valued rewires must update the watch set even when the output
    // value itself stays unchanged. Exercise both an intermediate relay
    // and the solver's own input connection.
    relay.SetConnections({b.GetPath()});
    evaluate(6);
    CHECK(evaluator.GetBindingEpochDigest() != epoch);
    a.Set(0.0f);
    CHECK(evaluate(6).solverEvaluations == 0);
    b.Set(0.25f);
    CHECK(evaluate(5).solverEvaluations == 1);
    const size_t relayEpoch = evaluator.GetBindingEpochDigest();
    stretch.SetConnections({c.GetPath()});
    evaluate(5);
    CHECK(evaluator.GetBindingEpochDigest() != relayEpoch);
    b.Set(0.0f);
    CHECK(evaluate(5).solverEvaluations == 0);
    c.Set(0.75f);
    CHECK(evaluate(7).solverEvaluations == 1);
    c.Set(0.5f);
    CHECK(evaluate(6).solverEvaluations == 1);
}

static void
TestConstraintSolverDependencySchedule()
{
    const auto stage = UsdStage::CreateInMemory();
    MakeXform(stage, SdfPath("/Asset"), Matrix());
    stage->DefinePrim(SdfPath("/Asset/Rig"), TfToken("RigExecRoot"));
    const auto control = [&](const char *name, const GfVec3d &rest) {
        const UsdPrim prim = stage->DefinePrim(
            SdfPath(std::string("/Asset/Rig/Controls/") + name),
            TfToken("RigExecControl"));
        prim.GetAttribute(TfToken("rest:tx")).Set(rest[0]);
        prim.GetAttribute(TfToken("rest:ty")).Set(rest[1]);
        return prim;
    };
    // rest:tx 0/5/10 so the IK's bones MEASURE 5 and 5. These tests
    // used to author absolute lengths, which Compile now rejects.
    const auto joint = [&](const char *name, double tx = 0.0) {
        const UsdPrim prim = stage->DefinePrim(
            SdfPath(std::string("/Asset/Rig/Joints/") + name),
            TfToken("RigExecJoint"));
        prim.GetAttribute(TfToken("rest:tx")).Set(tx);
        return prim;
    };
    const auto fk = [&](const char *name, const UsdPrim &input, const UsdPrim &output) {
        const UsdPrim prim = stage->DefinePrim(
            SdfPath(std::string("/Asset/Rig/Solvers/") + name),
            TfToken("RigExecFkChain"));
        prim.GetRelationship(TfToken("rigExec:controls")).SetTargets({input.GetPath()});
        prim.GetRelationship(TfToken("rigExec:joints")).SetTargets({output.GetPath()});
        return prim;
    };
    const UsdPrim driver = control("Driver", GfVec3d(4, 0, 0));
    const UsdPrim root = control("Root", GfVec3d(0));
    const UsdPrim goal = control("Goal", GfVec3d(1, 0, 0));
    const UsdPrim pole = control("Pole", GfVec3d(0, 5, 0));
    const UsdPrim follow = control("Follow", GfVec3d(0));
    const UsdPrim other = control("Other", GfVec3d(20, 0, 0));
    const UsdPrim sourceJoint = joint("Source");
    const UsdPrim rootJoint = joint("Root", 0.0);
    const UsdPrim midJoint = joint("Mid", 5.0);
    const UsdPrim endJoint = joint("End", 10.0);
    const UsdPrim finalJoint = joint("Final");
    fk("Source", driver, sourceJoint);
    fk("Final", follow, finalJoint);
    fk("Independent", other, joint("Independent"));
    const UsdPrim ik = stage->DefinePrim(
        SdfPath("/Asset/Rig/Solvers/IK"), TfToken("RigExecTwoBoneIk"));
    ik.GetRelationship(TfToken("rigExec:rootControl")).SetTargets({root.GetPath()});
    ik.GetRelationship(TfToken("rigExec:effectorControl")).SetTargets({goal.GetPath()});
    ik.GetRelationship(TfToken("rigExec:poleControl")).SetTargets({pole.GetPath()});
    ik.GetRelationship(TfToken("rigExec:joints"))
        .SetTargets({rootJoint.GetPath(), midJoint.GetPath(), endJoint.GetPath()});
    // Reverse sibling order runs DriveGoal then FollowEnd. Solvers must be
    // interleaved: Source FK -> DriveGoal -> IK -> FollowEnd -> Final FK.
    const UsdPrim followConstraint = MakeConstraint(
        stage, "FollowEnd", "RigExecPositionConstraint", {follow.GetPath()});
    followConstraint.GetRelationship(TfToken("rigExec:sources"))
        .SetTargets({endJoint.GetPath()});
    const UsdPrim driveConstraint = MakeConstraint(
        stage, "DriveGoal", "RigExecPositionConstraint", {goal.GetPath()});
    driveConstraint.GetRelationship(TfToken("rigExec:sources"))
        .SetTargets({sourceJoint.GetPath()});
    RigExecRigEvaluator evaluator(stage, SdfPath("/Asset/Rig"));
    std::vector<std::string> errors;
    CHECK(evaluator.Compile(&errors));
    const size_t epoch = evaluator.GetBindingEpochDigest();
    auto checkPose = [&](double expected, size_t solverCount) {
        const auto pose = evaluator.Evaluate(UsdTimeCode::Default());
        if (!pose.valid) {
            for (const auto &error : pose.diagnostics) std::printf("pose: %s\n", error.c_str());
        }
        CHECK(pose.valid);
        CHECK(pose.solverEvaluations == solverCount);
        for (const SdfPath &path : {endJoint.GetPath(), finalJoint.GetPath()}) {
            const auto frame = pose.jointFramesFinal.find(path);
            CHECK(frame != pose.jointFramesFinal.end());
            if (frame != pose.jointFramesFinal.end()) {
                CHECK(Near(frame->second.Origin(), GfVec3d(expected, 0, 0)));
            }
        }
        CHECK(evaluator.GetBindingEpochDigest() == epoch);
    };
    checkPose(4, 4);
    checkPose(4, 0);
    driver.GetAttribute(TfToken("avars:tx")).Set(2.0);
    checkPose(6, 3);
    driver.GetAttribute(TfToken("avars:tx")).Set(3.0);
    checkPose(7, 3);
    driveConstraint.GetAttribute(TfToken("inputs:translationOffset"))
        .Set(GfVec3d(1, 0, 0));
    checkPose(8, 2);
    checkPose(8, 0);

    // A final-frame feedback edge is a cycle, not a previous-generation read.
    driveConstraint.GetRelationship(TfToken("rigExec:sources"))
        .SetTargets({endJoint.GetPath()});
    errors.clear();
    CHECK(!evaluator.Compile(&errors));
    CHECK(std::any_of(errors.begin(), errors.end(), [](const std::string &error) {
        return error.find("pose dependency cycle") != std::string::npos;
    }));
    driveConstraint.GetRelationship(TfToken("rigExec:sources"))
        .SetTargets({sourceJoint.GetPath()});
    CHECK(evaluator.Evaluate(UsdTimeCode::Default()).valid);
}

static void
TestConstrainedSolverInputAncestor()
{
    const auto stage = UsdStage::CreateInMemory();
    MakeXform(stage, SdfPath("/Asset"), Matrix());
    MakeXform(stage, SdfPath("/Asset/Driver"), Matrix(GfVec3d(2, 0, 0)));
    stage->DefinePrim(SdfPath("/Asset/Rig"), TfToken("RigExecRoot"));
    const auto control = [&](const char *path, double x, double y) {
        const UsdPrim prim = stage->DefinePrim(SdfPath(path), TfToken("RigExecControl"));
        prim.GetAttribute(TfToken("rest:tx")).Set(x);
        prim.GetAttribute(TfToken("rest:ty")).Set(y);
        return prim;
    };
    const UsdPrim group = control("/Asset/Rig/Controls/Group", 0, 0);
    group.GetAttribute(TfToken("purpose")).Set(TfToken("guide"));
    const UsdPrim goal = control("/Asset/Rig/Controls/Group/Goal", 4, 0);
    goal.GetAttribute(TfToken("purpose")).Set(TfToken("guide"));
    const UsdPrim root = control("/Asset/Rig/Controls/Root", 0, 0);
    const UsdPrim pole = control("/Asset/Rig/Controls/Pole", 0, 5);
    // rest:tx 0/5/10 so the IK's bones MEASURE 5 and 5. These tests
    // used to author absolute lengths, which Compile now rejects.
    SdfPathVector joints;
    {
        double tx = 0.0;
        for (const char *name : {"J0", "J1", "J2"}) {
            const UsdPrim joint = stage->DefinePrim(
                group.GetPath().AppendChild(TfToken(name)),
                TfToken("RigExecJoint"));
            joint.GetAttribute(TfToken("purpose")).Set(TfToken("guide"));
            joint.GetAttribute(TfToken("rest:tx")).Set(tx);
            tx += 5.0;
            joints.push_back(joint.GetPath());
        }
    }
    const UsdPrim ik = stage->DefinePrim(
        SdfPath("/Asset/Rig/Solvers/IK"), TfToken("RigExecTwoBoneIk"));
    ik.GetRelationship(TfToken("rigExec:rootControl")).SetTargets({root.GetPath()});
    ik.GetRelationship(TfToken("rigExec:effectorControl")).SetTargets({goal.GetPath()});
    ik.GetRelationship(TfToken("rigExec:poleControl")).SetTargets({pole.GetPath()});
    ik.GetRelationship(TfToken("rigExec:joints")).SetTargets(joints);
    const UsdPrim constraint = MakeConstraint(
        stage, "MoveGroup", "RigExecPositionConstraint", {group.GetPath()});
    constraint.GetRelationship(TfToken("rigExec:sources"))
        .SetTargets({SdfPath("/Asset/Driver")});
    RigExecRigEvaluator evaluator(stage, SdfPath("/Asset/Rig"));
    std::vector<std::string> errors;
    CHECK(evaluator.Compile(&errors));
    const auto pose = evaluator.Evaluate(UsdTimeCode::Default());
    CHECK(pose.valid);
    if (pose.valid) {
        CHECK(Near(pose.controlFrames.at(goal.GetPath()).Origin(), GfVec3d(6, 0, 0)));
        CHECK(Near(pose.jointFramesFinal.at(joints[0]).Origin(), GfVec3d(0)));
        CHECK(Near(pose.jointFramesFinal.at(joints[2]).Origin(), GfVec3d(6, 0, 0)));
        CHECK(Near(pose.jointFramesBase.at(joints[2]).Origin(), GfVec3d(6, 0, 0)));
    }
}

// Namespace propagation must STOP at a path that owns its own pose.
//
// A constraint that moves an ancestor publishes a delta its namespace
// descendants ride, but a joint a solver writes is an absolute posed
// override: neither it nor anything beneath it may take that ride. Every
// other test asserts the ride; this one asserts the stop, and its negative
// control is the same subtree with the joint unbound, which must ride.
//
// The constraint's source is itself solver-driven, so the constraint is
// scheduled after every solver batch and nothing rewrites the blocked frames
// afterwards -- with the constraint first, a later solver commit would
// re-derive the subtree from its own output and hide a lost boundary.
static void
TestSolverOwnedJointBlocksNamespacePropagation()
{
    const auto stage = UsdStage::CreateInMemory();
    MakeXform(stage, SdfPath("/Asset"), Matrix());
    stage->DefinePrim(SdfPath("/Asset/Rig"), TfToken("RigExecRoot"));
    // rest:tx is parent-relative: the asset-space origins below are the sums
    // spelled out in the expectations.
    const auto provider = [&](const char *path, const char *type, double x) {
        const UsdPrim prim = stage->DefinePrim(SdfPath(path), TfToken(type));
        prim.GetAttribute(TfToken("rest:tx")).Set(x);
        prim.GetAttribute(TfToken("purpose")).Set(TfToken("guide"));
        return prim;
    };
    const auto control = [&](const char *path, double x) {
        return provider(path, "RigExecControl", x);
    };
    const auto joint = [&](const char *path, double x) {
        return provider(path, "RigExecJoint", x);
    };
    const auto fkChain = [&](const char *name, const UsdPrim &driver,
                             const SdfPath &drivenJoint) {
        const UsdPrim solver = stage->DefinePrim(
            SdfPath(std::string("/Asset/Rig/Solvers/") + name),
            TfToken("RigExecFkChain"));
        solver.GetRelationship(TfToken("rigExec:controls"))
            .SetTargets({driver.GetPath()});
        solver.GetRelationship(TfToken("rigExec:joints")).SetTargets({drivenJoint});
        return solver;
    };

    const UsdPrim lead = control("/Asset/Rig/Controls/Lead", 0);
    lead.GetAttribute(TfToken("avars:tx")).Set(2.0, UsdTimeCode(1.0));
    lead.GetAttribute(TfToken("avars:tx")).Set(5.0, UsdTimeCode(2.0));
    const UsdPrim leadJoint = joint("/Asset/Rig/Joints/Lead", 0);
    fkChain("Lead", lead, leadJoint.GetPath());

    // The moved subtree. Bound is solver-owned and Free is not; each carries
    // a deeper provider that inherits its namespace pose.
    const UsdPrim group = control("/Asset/Rig/Controls/Group", 0);
    const UsdPrim bound = joint("/Asset/Rig/Controls/Group/Bound", 1);
    const SdfPath boundTip = control("/Asset/Rig/Controls/Group/Bound/Tip", 2).GetPath();
    const UsdPrim free = joint("/Asset/Rig/Controls/Group/Free", 10);
    const SdfPath freeTip = control("/Asset/Rig/Controls/Group/Free/Tip", 11).GetPath();
    const UsdPrim boundDriver = control("/Asset/Rig/Controls/BoundDriver", 1);
    const UsdPrim boundSolver = fkChain("Bound", boundDriver, bound.GetPath());

    const UsdPrim move = MakeConstraint(
        stage, "MoveGroup", "RigExecPositionConstraint", {group.GetPath()});
    move.GetRelationship(TfToken("rigExec:sources")).SetTargets({leadJoint.GetPath()});

    RigExecRigEvaluator evaluator(stage, SdfPath("/Asset/Rig"));
    std::vector<std::string> errors;
    CHECK(evaluator.Compile(&errors));
    CHECK(errors.empty());

    // `delta` is where the constraint takes Group, and therefore the shift
    // every unblocked descendant must show.
    const auto check = [&](double time, double delta, bool blocked) {
        const auto pose = evaluator.Evaluate(UsdTimeCode(time));
        CHECK(pose.valid);
        if (!pose.valid) return;
        const auto joints = [&](const SdfPath &path) {
            const auto frame = pose.jointFramesFinal.find(path);
            CHECK(frame != pose.jointFramesFinal.end());
            return frame == pose.jointFramesFinal.end()
                ? GfVec3d(std::numeric_limits<double>::quiet_NaN())
                : frame->second.Origin();
        };
        const auto controls = [&](const SdfPath &path) {
            const auto frame = pose.controlFrames.find(path);
            CHECK(frame != pose.controlFrames.end());
            return frame == pose.controlFrames.end()
                ? GfVec3d(std::numeric_limits<double>::quiet_NaN())
                : frame->second.Origin();
        };
        CHECK(Near(controls(group.GetPath()), GfVec3d(delta, 0, 0)));
        // Unowned: the whole subtree rides the delta.
        CHECK(Near(joints(free.GetPath()), GfVec3d(10 + delta, 0, 0)));
        CHECK(Near(controls(freeTip), GfVec3d(21 + delta, 0, 0)));
        // Owned by a solver: the joint holds its solved frame and its own
        // descendant stays with it. Unbind the solver and both ride.
        CHECK(Near(joints(bound.GetPath()),
                   GfVec3d(blocked ? 1 : 1 + delta, 0, 0)));
        CHECK(Near(controls(boundTip),
                   GfVec3d(blocked ? 3 : 3 + delta, 0, 0)));
    };
    // Two times: the ownership table is rebuilt per evaluation, and a table
    // that survived one would be wrong on the next.
    check(1.0, 2.0, true);
    check(2.0, 5.0, true);

    // Negative control: the same rig with Bound bound to nothing.
    joint("/Asset/Rig/Joints/Decoy", 0);
    boundSolver.GetRelationship(TfToken("rigExec:joints"))
        .SetTargets({SdfPath("/Asset/Rig/Joints/Decoy")});
    errors.clear();
    CHECK(evaluator.Compile(&errors));
    CHECK(errors.empty());
    check(1.0, 2.0, false);
    check(2.0, 5.0, false);
}

static void
TestConnectedParentSpaceSolverInputs()
{
    const auto stage = UsdStage::CreateInMemory();
    MakeXform(stage, SdfPath("/Asset"), Matrix());
    MakeXform(stage, SdfPath("/Asset/DriverTarget"), Matrix(GfVec3d(4, 0, 0)));
    const UsdPrim target = MakeXform(stage, SdfPath("/Asset/JointTarget"), Matrix(GfVec3d(5, 0, 0)));
    stage->DefinePrim(SdfPath("/Asset/Rig"), TfToken("RigExecRoot"));
    const auto control = [&](const char *path, double x, double y = 0) {
        const UsdPrim prim = stage->DefinePrim(SdfPath(path), TfToken("RigExecControl"));
        prim.GetAttribute(TfToken("rest:tx")).Set(x);
        prim.GetAttribute(TfToken("rest:ty")).Set(y);
        prim.GetAttribute(TfToken("purpose")).Set(TfToken("guide"));
        return prim;
    };
    const UsdPrim driver = control("/Asset/Rig/Controls/Driver", 1);
    const UsdPrim altDriver = control("/Asset/Rig/Controls/AltDriver", 2);
    const UsdPrim source = stage->DefinePrim(SdfPath("/Asset/Rig/Joints/Source"), TfToken("RigExecJoint"));
    const UsdPrim altSource = stage->DefinePrim(SdfPath("/Asset/Rig/Joints/AltSource"), TfToken("RigExecJoint"));
    for (const auto &binding : {std::make_pair(driver, source), std::make_pair(altDriver, altSource)}) {
        const UsdPrim fk = stage->DefinePrim(
            SdfPath("/Asset/Rig/Solvers").AppendChild(binding.first.GetName()), TfToken("RigExecFkChain"));
        fk.GetRelationship(TfToken("rigExec:controls")).SetTargets({binding.first.GetPath()});
        fk.GetRelationship(TfToken("rigExec:joints")).SetTargets({binding.second.GetPath()});
    }
    const UsdPrim relay = control("/Asset/Rig/Joints/Source/Relay", 0);
    const UsdPrim altRelay = control("/Asset/Rig/Joints/AltSource/Relay", 0);
    const UsdPrim bridge = control("/Asset/Rig/Controls/Bridge", 0);
    bridge.GetAttribute(TfToken("default:space"))
        .SetConnections({relay.GetPath().AppendProperty(TfToken("parent:space"))});
    // Bridge's own namespace descendants: the connected refresh has to carry
    // the BASE phase of a descendant with its connected ancestor, and has to
    // honour the same ownership boundary the constraint walk honours. Rider
    // inherits its pose and must follow; Owner is written by a solver of its
    // own and must not. Owner has to be solver-bound for the boundary to be
    // observable at all: a descendant the refresh walk visits in its own
    // right recomputes its base frame from its own tap straight afterwards,
    // while a solver-bound joint is skipped, so what the carry leaves on it
    // is what the pose publishes.
    const UsdPrim rider = stage->DefinePrim(
        SdfPath("/Asset/Rig/Controls/Bridge/Rider"), TfToken("RigExecJoint"));
    rider.GetAttribute(TfToken("rest:tx")).Set(1.0);
    rider.GetAttribute(TfToken("purpose")).Set(TfToken("guide"));
    const UsdPrim owner = stage->DefinePrim(
        SdfPath("/Asset/Rig/Controls/Bridge/Owner"), TfToken("RigExecJoint"));
    owner.GetAttribute(TfToken("rest:tx")).Set(1.0);
    owner.GetAttribute(TfToken("purpose")).Set(TfToken("guide"));
    // A descendant a constraint owns is skipped by the refresh walk too, so
    // its base phase is exactly what the loop leaves behind.
    const UsdPrim held = stage->DefinePrim(
        SdfPath("/Asset/Rig/Controls/Bridge/Held"), TfToken("RigExecJoint"));
    held.GetAttribute(TfToken("rest:tx")).Set(2.0);
    held.GetAttribute(TfToken("purpose")).Set(TfToken("guide"));
    MakeXform(stage, SdfPath("/Asset/HeldTarget"), Matrix(GfVec3d(9, 0, 0)));
    MakeConstraint(stage, "MoveHeld", "RigExecPositionConstraint", {held.GetPath()})
        .GetRelationship(TfToken("rigExec:sources"))
        .SetTargets({SdfPath("/Asset/HeldTarget")});
    const UsdPrim ownerDriver = control("/Asset/Rig/Controls/OwnerDriver", 7);
    {
        const UsdPrim fk = stage->DefinePrim(
            SdfPath("/Asset/Rig/Solvers/Owner"), TfToken("RigExecFkChain"));
        fk.GetRelationship(TfToken("rigExec:controls"))
            .SetTargets({ownerDriver.GetPath()});
        fk.GetRelationship(TfToken("rigExec:joints")).SetTargets({owner.GetPath()});
    }
    const UsdPrim goal = control("/Asset/Rig/Controls/Goal", 2);
    goal.GetAttribute(TfToken("parent:space"))
        .SetConnections({bridge.GetPath().AppendProperty(TfToken("posed:defaultSpace"))});
    const UsdPrim root = control("/Asset/Rig/Controls/Root", 0);
    const UsdPrim pole = control("/Asset/Rig/Controls/Pole", 0, 5);
    // rest:tx 0/5/10 so the IK's bones MEASURE 5 and 5. These tests
    // used to author absolute lengths, which Compile now rejects.
    SdfPathVector joints;
    {
        double tx = 0.0;
        for (const char *name : {"J0", "J1", "J2"}) {
            const SdfPath path =
                SdfPath("/Asset/Rig/Joints").AppendChild(TfToken(name));
            stage->DefinePrim(path, TfToken("RigExecJoint"))
                .GetAttribute(TfToken("rest:tx")).Set(tx);
            tx += 5.0;
            joints.push_back(path);
        }
    }
    const UsdPrim ik = stage->DefinePrim(SdfPath("/Asset/Rig/Solvers/IK"), TfToken("RigExecTwoBoneIk"));
    ik.GetRelationship(TfToken("rigExec:rootControl")).SetTargets({root.GetPath()});
    ik.GetRelationship(TfToken("rigExec:effectorControl")).SetTargets({goal.GetPath()});
    ik.GetRelationship(TfToken("rigExec:poleControl")).SetTargets({pole.GetPath()});
    ik.GetRelationship(TfToken("rigExec:joints")).SetTargets(joints);
    const UsdPrim moveJoint = MakeConstraint(stage, "MoveJoint", "RigExecPositionConstraint", {source.GetPath()});
    moveJoint.GetRelationship(TfToken("rigExec:sources")).SetTargets({target.GetPath()});
    const UsdPrim moveDriver = MakeConstraint(stage, "MoveDriver", "RigExecPositionConstraint", {driver.GetPath()});
    moveDriver.GetRelationship(TfToken("rigExec:sources")).SetTargets({SdfPath("/Asset/DriverTarget")});
    RigExecRigEvaluator evaluator(stage, SdfPath("/Asset/Rig"));
    std::vector<std::string> errors;
    CHECK(evaluator.Compile(&errors));
    const size_t epoch = evaluator.GetBindingEpochDigest();
    // `riderBase` is where Bridge's BASE phase leaves its inheriting
    // descendant, which is one unit past Bridge's own base frame and lags
    // the final phase whenever a constraint has revised Bridge's input.
    auto check = [&](double expected, size_t evaluations, double riderBase) {
        const auto pose = evaluator.Evaluate(UsdTimeCode::Default());
        CHECK(pose.valid);
        CHECK(pose.solverEvaluations == evaluations);
        if (pose.valid) {
            CHECK(Near(pose.controlFrames.at(goal.GetPath()).Origin(), GfVec3d(expected, 0, 0)));
            CHECK(Near(pose.jointFramesFinal.at(joints[2]).Origin(), GfVec3d(expected, 0, 0)));
            // Both phases of a connected provider carry its descendants.
            CHECK(Near(pose.jointFramesBase.at(rider.GetPath()).Origin(),
                       GfVec3d(riderBase, 0, 0)));
            CHECK(Near(pose.jointFramesFinal.at(rider.GetPath()).Origin(),
                       pose.controlFrames.at(bridge.GetPath()).Origin() +
                           GfVec3d(1, 0, 0)));
            // Held sits one unit further out and is carried by the same
            // write, with its own constraint owning only the final phase.
            CHECK(Near(pose.jointFramesBase.at(held.GetPath()).Origin(),
                       GfVec3d(riderBase + 1, 0, 0)));
            CHECK(Near(pose.jointFramesFinal.at(held.GetPath()).Origin(),
                       GfVec3d(9, 0, 0)));
            // ... and neither may cross an ownership boundary: Owner keeps
            // the frame its own solver wrote, in both phases.
            CHECK(Near(pose.jointFramesBase.at(owner.GetPath()).Origin(), GfVec3d(7, 0, 0)));
            CHECK(Near(pose.jointFramesFinal.at(owner.GetPath()).Origin(), GfVec3d(7, 0, 0)));
        }
        return pose;
    };
    const auto first = check(7, 4, 5);
    if (first.valid) {
        CHECK(Near(first.jointFramesBase.at(source.GetPath()).Origin(), GfVec3d(4, 0, 0)));
        CHECK(Near(first.jointFramesFinal.at(source.GetPath()).Origin(), GfVec3d(5, 0, 0)));
    }
    check(7, 0, 5);
    goal.GetAttribute(TfToken("rest:tx")).Set(3.0);
    check(8, 1, 5);
    target.GetAttribute(TfToken("xformOp:transform")).Set(Matrix(GfVec3d(6, 0, 0)));
    check(9, 1, 5);
    CHECK(evaluator.GetBindingEpochDigest() == epoch);
    bridge.GetAttribute(TfToken("default:space"))
        .SetConnections({altRelay.GetPath().AppendProperty(TfToken("parent:space"))});
    check(5, 4, 3);
    CHECK(evaluator.GetBindingEpochDigest() != epoch);
    target.GetAttribute(TfToken("xformOp:transform")).Set(Matrix(GfVec3d(7, 0, 0)));
    check(5, 0, 3);
    altDriver.GetAttribute(TfToken("avars:tx")).Set(1.0);
    check(6, 2, 4);
    altDriver.GetAttribute(TfToken("avars:tx")).Set(2.0);
    check(7, 2, 5);
    // Connected space ancestry participates in cycle validation.
    bridge.GetAttribute(TfToken("default:space"))
        .SetConnections({relay.GetPath().AppendProperty(TfToken("parent:space"))});
    moveDriver.GetRelationship(TfToken("rigExec:sources")).SetTargets({joints[2]});
    errors.clear();
    CHECK(!evaluator.Compile(&errors));
    CHECK(std::any_of(errors.begin(), errors.end(), [](const std::string &error) {
        return error.find("pose dependency cycle") != std::string::npos;
    }));
}

static void
TestSolverGuidesGate()
{
    const auto stage = UsdStage::CreateInMemory();
    stage->DefinePrim(SdfPath("/Asset"), TfToken("Xform"));
    stage->DefinePrim(SdfPath("/Asset/Rig"), TfToken("RigExecRoot"));
    const UsdPrim source = stage->DefinePrim(
        SdfPath("/Asset/Rig/Controls/Source"), TfToken("RigExecControl"));
    source.GetAttribute(TfToken("avars:tx")).Set(2.0);
    const SdfPath jointPath("/Asset/Rig/Joints/J0");
    stage->DefinePrim(jointPath, TfToken("RigExecJoint"));
    const SdfPath solverPath("/Asset/Rig/Solvers/Twist");
    const UsdPrim twist = stage->DefinePrim(solverPath, TfToken("RigExecTwistDistribution"));
    twist.GetRelationship(TfToken("rigExec:start")).SetTargets({source.GetPath()});
    twist.GetRelationship(TfToken("rigExec:end")).SetTargets({source.GetPath()});
    twist.GetRelationship(TfToken("rigExec:joints")).SetTargets({jointPath});
    RigExecRigEvaluator evaluator(stage, SdfPath("/Asset/Rig"));
    std::vector<std::string> errors;
    CHECK(evaluator.Compile(&errors));
    CHECK(evaluator.GetSolverGuidesEnabled());
    const auto guided = evaluator.Evaluate(UsdTimeCode::Default());
    CHECK(guided.valid);
    const auto guidedFrames = guided.solverFrames.find(solverPath);
    CHECK(guidedFrames != guided.solverFrames.end() && guidedFrames->second.size() == 1);

    // Headless consumers skip the whole guide request; everything else in
    // the generation must be unchanged.
    evaluator.SetSolverGuidesEnabled(false);
    CHECK(!evaluator.GetSolverGuidesEnabled());
    const auto unguided = evaluator.Evaluate(UsdTimeCode::Default());
    CHECK(unguided.valid);
    CHECK(unguided.solverFrames.empty());
    CHECK(unguided.jointFramesFinal.size() == guided.jointFramesFinal.size());
    for (const auto &[path, frame] : guided.jointFramesFinal) {
        const auto it = unguided.jointFramesFinal.find(path);
        CHECK(it != unguided.jointFramesFinal.end());
        if (it != unguided.jointFramesFinal.end()) {
            CHECK(Near(it->second.Origin(), frame.Origin()));
        }
    }

    // Re-enabling restores the exact guide frames.
    evaluator.SetSolverGuidesEnabled(true);
    const auto reguided = evaluator.Evaluate(UsdTimeCode::Default());
    CHECK(reguided.valid);
    const auto reguidedFrames = reguided.solverFrames.find(solverPath);
    CHECK(reguidedFrames != reguided.solverFrames.end() &&
          reguidedFrames->second.size() == guidedFrames->second.size());
    if (reguidedFrames != reguided.solverFrames.end() &&
        reguidedFrames->second.size() == guidedFrames->second.size()) {
        for (size_t i = 0; i < guidedFrames->second.size(); ++i) {
            CHECK(Near(reguidedFrames->second[i].Origin(),
                       guidedFrames->second[i].Origin()));
        }
    }
}

static void
TestSolverBatchLevelAudit()
{
    // Diamond aggregate dependency: A feeds B and C through their joints,
    // D blends B and C directly. Minimal longest-path layering puts A at 0,
    // B and C together at 1, and D at 2 -- dense, with no wave wasted on
    // schedule order.
    const auto stage = UsdStage::CreateInMemory();
    stage->DefinePrim(SdfPath("/Asset"), TfToken("Xform"));
    stage->DefinePrim(SdfPath("/Asset/Rig"), TfToken("RigExecRoot"));
    const UsdPrim source = stage->DefinePrim(
        SdfPath("/Asset/Rig/Controls/Source"), TfToken("RigExecControl"));
    source.GetAttribute(TfToken("avars:tx")).Set(2.0);
    const auto jointInput = [&](const SdfPath &jointPath) {
        stage->DefinePrim(jointPath, TfToken("RigExecJoint"));
        const UsdPrim child = stage->DefinePrim(
            jointPath.AppendChild(TfToken("Input")), TfToken("RigExecControl"));
        child.GetAttribute(TfToken("rest:tx")).Set(1.0);
        child.GetAttribute(TfToken("purpose")).Set(TfToken("guide"));
        return child.GetPath();
    };
    const SdfPath jointA("/Asset/Rig/Joints/JA");
    const UsdPrim twistA = stage->DefinePrim(
        SdfPath("/Asset/Rig/Solvers/A"), TfToken("RigExecTwistDistribution"));
    twistA.GetRelationship(TfToken("rigExec:start")).SetTargets({source.GetPath()});
    twistA.GetRelationship(TfToken("rigExec:end")).SetTargets({source.GetPath()});
    twistA.GetRelationship(TfToken("rigExec:joints")).SetTargets({jointA});
    const SdfPath inputA = jointInput(jointA);
    const SdfPath jointB("/Asset/Rig/Joints/JB");
    const UsdPrim twistB = stage->DefinePrim(
        SdfPath("/Asset/Rig/Solvers/B"), TfToken("RigExecTwistDistribution"));
    twistB.GetRelationship(TfToken("rigExec:start")).SetTargets({inputA});
    twistB.GetRelationship(TfToken("rigExec:end")).SetTargets({inputA});
    twistB.GetRelationship(TfToken("rigExec:joints")).SetTargets({jointB});
    jointInput(jointB);
    const SdfPath jointC("/Asset/Rig/Joints/JC");
    const UsdPrim twistC = stage->DefinePrim(
        SdfPath("/Asset/Rig/Solvers/C"), TfToken("RigExecTwistDistribution"));
    twistC.GetRelationship(TfToken("rigExec:start")).SetTargets({inputA});
    twistC.GetRelationship(TfToken("rigExec:end")).SetTargets({inputA});
    twistC.GetRelationship(TfToken("rigExec:joints")).SetTargets({jointC});
    jointInput(jointC);
    const SdfPath jointD("/Asset/Rig/Joints/JD");
    stage->DefinePrim(jointD, TfToken("RigExecJoint"));
    const SdfPath solverA("/Asset/Rig/Solvers/A");
    const SdfPath solverB("/Asset/Rig/Solvers/B");
    const SdfPath solverC("/Asset/Rig/Solvers/C");
    const SdfPath solverD("/Asset/Rig/Solvers/D");
    const UsdPrim blend = stage->DefinePrim(solverD, TfToken("RigExecBlendPointFrames"));
    blend.GetRelationship(TfToken("rigExec:inputA")).SetTargets({solverB});
    blend.GetRelationship(TfToken("rigExec:inputB")).SetTargets({solverC});
    blend.GetAttribute(TfToken("inputs:weight")).Set(0.5f);
    blend.GetRelationship(TfToken("rigExec:joints")).SetTargets({jointD});
    RigExecRigEvaluator evaluator(stage, SdfPath("/Asset/Rig"));
    std::vector<std::string> errors;
    CHECK(evaluator.Compile(&errors));
    CHECK(errors.empty());
    const std::map<SdfPath, size_t> levels = evaluator.GetSolverBatchLevels();
    CHECK(levels.size() == 4);
    CHECK(levels.at(solverA) == 0);
    CHECK(levels.at(solverB) == 1);
    CHECK(levels.at(solverC) == 1);
    CHECK(levels.at(solverD) == 2);
    std::set<size_t> dense;
    for (const auto &[solver, level] : levels) dense.insert(level);
    CHECK((dense == std::set<size_t>{0, 1, 2}));
    const auto pose = evaluator.Evaluate(UsdTimeCode::Default());
    CHECK(pose.valid && pose.solverOverridesConverged);
    CHECK(pose.solverEvaluations == 4);
    CHECK(pose.solverOverrideRounds == 3);
}

// ---------------------------------------------------------------------------
// RigExecSplineIk: the control-driven spine solver, exercised through exec
// (the aggregate tap, like TestTwoBoneIkRestFrameInputs) and through the
// evaluator (joint binding, and the non-uniform squash scale surviving into
// the bound joints' frames and matrices). The kernel itself is covered by
// testRigExecSplineIk; these tests are about the wiring: control frames in,
// per-joint frames out, and the schema knobs reaching the solve.
// ---------------------------------------------------------------------------

static constexpr double kSplineIkPi = 3.141592653589793238462643383279502884;

static GfVec3d
SplineIkUnitX(const RigExecPointFrame &f)
{
    return (f.X() - f.Origin()).GetNormalized();
}

static GfVec3d
SplineIkUnitY(const RigExecPointFrame &f)
{
    return (f.Y() - f.Origin()).GetNormalized();
}

static double
SplineIkHandle(const RigExecPointFrame &f, int axis)
{
    return (f.points[axis] - f.Origin()).GetLength();
}

// Orthonormal rest matrix (row-vector convention) with X along `aim` and Y
// the projected `up`: what a joint's rest:space must be, since rest spaces
// are orthonormalized by contract.
static GfMatrix4d
SplineIkRestSpace(
    const GfVec3d &origin, const GfVec3d &aim, const GfVec3d &upCandidate)
{
    const GfVec3d x = aim.GetNormalized();
    GfVec3d y = upCandidate - x * GfDot(x, upCandidate);
    y.Normalize();
    const GfVec3d z = GfCross(x, y);
    GfMatrix4d m(1.0);
    m.SetRow(0, GfVec4d(x[0], x[1], x[2], 0));
    m.SetRow(1, GfVec4d(y[0], y[1], y[2], 0));
    m.SetRow(2, GfVec4d(z[0], z[1], z[2], 0));
    m.SetRow(3, GfVec4d(origin[0], origin[1], origin[2], 1));
    return m;
}

struct SplineIkStage {
    UsdStageRefPtr stage;
    UsdPrim root, mid, end, solver;
    std::vector<UsdPrim> joints;
    std::vector<GfVec3d> origins;
};

// A chain whose rest origins are `origins` (X aims at the successor, Y the
// projected +Y up), root/mid/end controls at the first, middle, and last
// joint with the same orientation, and a RigExecSplineIk over the chain,
// all under a rig root so the evaluator can compile it too.
static SplineIkStage
MakeSplineIkStage(const std::vector<GfVec3d> &origins)
{
    SplineIkStage s;
    s.origins = origins;
    s.stage = UsdStage::CreateInMemory();
    s.stage->DefinePrim(SdfPath("/Asset"), TfToken("Xform"));
    s.stage->DefinePrim(SdfPath("/Asset/Rig"), TfToken("RigExecRoot"));
    const size_t n = origins.size();
    std::vector<GfMatrix4d> rests;
    for (size_t i = 0; i < n; ++i) {
        const GfVec3d aim = i + 1 < n ? origins[i + 1] - origins[i]
                                      : origins[i] - origins[i - 1];
        rests.push_back(SplineIkRestSpace(origins[i], aim, GfVec3d(0, 1, 0)));
    }
    const auto define = [&](const std::string &path, const char *type,
                            const GfMatrix4d &rest) {
        const UsdPrim prim = s.stage->DefinePrim(SdfPath(path), TfToken(type));
        CHECK(prim);
        CHECK(prim.GetAttribute(TfToken("rest:space")).Set(rest));
        return prim;
    };
    SdfPathVector jointPaths;
    for (size_t i = 0; i < n; ++i) {
        s.joints.push_back(define(
            "/Asset/Rig/Joints/J" + std::to_string(i), "RigExecJoint", rests[i]));
        jointPaths.push_back(s.joints.back().GetPath());
    }
    s.root = define("/Asset/Rig/Controls/Root", "RigExecControl", rests[0]);
    s.mid = define("/Asset/Rig/Controls/Mid", "RigExecControl", rests[n / 2]);
    s.end = define("/Asset/Rig/Controls/End", "RigExecControl", rests[n - 1]);
    s.solver = s.stage->DefinePrim(
        SdfPath("/Asset/Rig/Solvers/Spine"), TfToken("RigExecSplineIk"));
    CHECK(s.solver);
    s.solver.GetRelationship(TfToken("rigExec:rootControl"))
        .SetTargets({s.root.GetPath()});
    s.solver.GetRelationship(TfToken("rigExec:midControl"))
        .SetTargets({s.mid.GetPath()});
    s.solver.GetRelationship(TfToken("rigExec:endControl"))
        .SetTargets({s.end.GetPath()});
    s.solver.GetRelationship(TfToken("rigExec:joints")).SetTargets(jointPaths);
    return s;
}

static std::vector<GfVec3d>
SplineIkStraightOrigins()
{
    std::vector<GfVec3d> origins;
    for (int i = 0; i < 7; ++i) {
        origins.emplace_back(double(i), 0.0, 0.0);
    }
    return origins;
}

static RigExecPointFrameArray
SplineIkSolve(const SplineIkStage &s)
{
    RigExecTapSet taps(s.stage);
    const RigExecTapId tap = taps.Add(RigExecValueAddress::Prim(
        s.solver.GetPath(), TfToken("computePointFrameArray")));
    CHECK(taps.Prepare());
    const RigExecSnapshot snapshot = taps.Evaluate(UsdTimeCode::Default());
    CHECK(snapshot.IsValid() && snapshot.IsComplete());
    return snapshot.Get<RigExecPointFrameArray>(tap);
}

static void
SplineIkSet(const UsdPrim &prim, const char *attr, double value)
{
    CHECK(prim.GetAttribute(TfToken(attr)).Set(value));
}

static void
TestSplineIkRest()
{
    // Straight chain: the rest curve IS the chain, so every joint reproduces
    // its rest frame exactly and the published rests are the joint rests.
    {
        const SplineIkStage s = MakeSplineIkStage(SplineIkStraightOrigins());
        const RigExecPointFrameArray frames = SplineIkSolve(s);
        CHECK(frames.GetSize() == 7 && frames.rests.size() == 7);
        if (frames.GetSize() != 7 || frames.rests.size() != 7) {
            return;
        }
        for (size_t i = 0; i < 7; ++i) {
            const RigExecPointFrame &f = frames.frames[i];
            CHECK(f.IsValid() && !f.IsDegenerate());
            CHECK(Near(f.Origin(), s.origins[i], 1e-9));
            CHECK(Near(SplineIkUnitX(f), GfVec3d(1, 0, 0), 1e-9));
            CHECK(Near(SplineIkUnitY(f), GfVec3d(0, 1, 0), 1e-9));
            for (int axis = 1; axis < 4; ++axis) {
                CHECK(std::abs(SplineIkHandle(f, axis) - 1.0) < 1e-9);
            }
            CHECK(Near(frames.rests[i][0], s.origins[i], 1e-12));
        }
    }
    // Curved chain: the degree-2 curve through rest CVs [0], [1], [N-2],
    // [N-1] interpolates only its end CVs, so the interior joints carry a
    // rest residual. That is inherent to the model (a maintained offset
    // downstream absorbs it, as the conventional mo=1 constraints do): measure and
    // bound it, do not assert it away. With restLength = curve the ratio
    // is exactly one at rest, so the tip overshoots the curve end by the
    // chain/curve length difference along the end tangent.
    {
        std::vector<GfVec3d> origins;
        const double radius = 10.0;
        for (int i = 0; i < 7; ++i) {
            const double a = i * 8.0 * kSplineIkPi / 180.0;
            origins.emplace_back(radius * std::sin(a), radius * (1 - std::cos(a)), 0.0);
        }
        const SplineIkStage s = MakeSplineIkStage(origins);
        const RigExecPointFrameArray frames = SplineIkSolve(s);
        CHECK(frames.GetSize() == 7);
        if (frames.GetSize() != 7) {
            return;
        }
        double chainLength = 0.0;
        for (size_t i = 1; i < 7; ++i) {
            chainLength += (origins[i] - origins[i - 1]).GetLength();
        }
        const double segment = chainLength / 6.0;
        const RigExecSplineIkCurve restCurve(
            {origins[0], origins[1], origins[5], origins[6]});
        const double curveLength = restCurve.ArcLength();
        CHECK(chainLength > curveLength);

        CHECK(Near(frames.frames[0].Origin(), origins[0], 1e-9));
        double maxResidual = 0.0;
        for (size_t i = 1; i < 6; ++i) {
            const double residual =
                (frames.frames[i].Origin() - origins[i]).GetLength();
            maxResidual = std::max(maxResidual, residual);
            CHECK(residual < 0.5 * segment);
            CHECK(!frames.frames[i].IsDegenerate());
        }
        CHECK(maxResidual > 1e-6);  // measured, not zero: the model's residual
        std::printf("SplineIk curved rest: interior residual max %.4f "
                    "(segment %.4f, chain %.4f, curve %.4f)\n",
                    maxResidual, segment, chainLength, curveLength);
        const GfVec3d endTangent = (origins[6] - origins[5]).GetNormalized();
        CHECK(Near(frames.frames[6].Origin(),
                   origins[6] + endTangent * (chainLength - curveLength), 1e-9));

        // restLength = chain: the chain spans the curve, tip on the end CV.
        CHECK(s.solver.GetAttribute(TfToken("rigExec:restLength"))
                  .Set(TfToken("chain")));
        const RigExecPointFrameArray spanned = SplineIkSolve(s);
        CHECK(spanned.GetSize() == 7);
        if (spanned.GetSize() == 7) {
            CHECK(Near(spanned.frames[6].Origin(), origins[6], 1e-9));
            CHECK(Near(spanned.frames[0].Origin(), origins[0], 1e-9));
        }
    }
}

static void
TestSplineIkStretch()
{
    const SplineIkStage s = MakeSplineIkStage(SplineIkStraightOrigins());
    // End control pulled +3 along the chain; the mid control rides on its
    // follow point (half the end displacement) so it adds no offset.
    SplineIkSet(s.end, "avars:tx", 3.0);
    SplineIkSet(s.mid, "avars:tx", 1.5);
    const RigExecPointFrameArray frames = SplineIkSolve(s);
    CHECK(frames.GetSize() == 7);
    if (frames.GetSize() != 7) {
        return;
    }
    // ratio 1.5: joints at 1.5 spacing, the tip at the curve end (9), no
    // thinning without volume weights.
    for (size_t i = 0; i < 7; ++i) {
        const RigExecPointFrame &f = frames.frames[i];
        CHECK(Near(f.Origin(), GfVec3d(1.5 * i, 0, 0), 1e-9));
        CHECK(Near(SplineIkUnitX(f), GfVec3d(1, 0, 0), 1e-9));
        CHECK(Near(SplineIkUnitY(f), GfVec3d(0, 1, 0), 1e-9));
        CHECK(std::abs(SplineIkHandle(f, 1) - 1.0) < 1e-9);
        CHECK(std::abs(SplineIkHandle(f, 2) - 1.0) < 1e-9);
    }

    // Off-axis pull: the curve bends and lengthens; the ratio tracks the
    // posed curve's ARC LENGTH, so joint i sits at arc distance
    // ratio * i = (L / 6) * i along it. Verify against the curve built
    // from the CVs this pose produces: cv0/cv1 carried by the root
    // (unchanged), cv2/cv3 by the end (+4, +2), mid on its follow point.
    SplineIkSet(s.end, "avars:tx", 4.0);
    SplineIkSet(s.end, "avars:ty", 2.0);
    SplineIkSet(s.mid, "avars:tx", 2.0);
    SplineIkSet(s.mid, "avars:ty", 1.0);
    const RigExecPointFrameArray bent = SplineIkSolve(s);
    CHECK(bent.GetSize() == 7);
    if (bent.GetSize() != 7) {
        return;
    }
    const RigExecSplineIkCurve curve(
        {GfVec3d(0, 0, 0), GfVec3d(1, 0, 0), GfVec3d(9, 2, 0), GfVec3d(10, 2, 0)});
    const double ratio = curve.ArcLength() / 6.0;
    CHECK(ratio > 1.0);
    for (size_t i = 0; i < 7; ++i) {
        GfVec3d expected;
        CHECK(curve.PointAtArcLength(ratio * i, &expected, nullptr));
        CHECK(Near(bent.frames[i].Origin(), expected, 1e-9));
    }
    CHECK(Near(bent.frames[6].Origin(), GfVec3d(10, 2, 0), 1e-9));
    for (size_t i = 0; i + 1 < 7; ++i) {
        const GfVec3d chord = (bent.frames[i + 1].Origin() -
                               bent.frames[i].Origin()).GetNormalized();
        CHECK(Near(SplineIkUnitX(bent.frames[i]), chord, 1e-9));
    }
}

static void
TestSplineIkMidBend()
{
    const SplineIkStage s = MakeSplineIkStage(SplineIkStraightOrigins());
    // Mid control lifted +1.5 in Z: cv1 and cv2 take the offset, the
    // endpoints stay put, and the bend lies in the XZ plane so the +Y up
    // is untouched.
    SplineIkSet(s.mid, "avars:tz", 1.5);
    const RigExecPointFrameArray frames = SplineIkSolve(s);
    CHECK(frames.GetSize() == 7);
    if (frames.GetSize() != 7) {
        return;
    }
    CHECK(Near(frames.frames[0].Origin(), GfVec3d(0, 0, 0), 1e-9));
    CHECK(Near(frames.frames[6].Origin(), GfVec3d(6, 0, 0), 1e-9));
    for (size_t i = 1; i < 6; ++i) {
        CHECK(frames.frames[i].Origin()[2] > 0.1);
        CHECK(frames.frames[i].Origin()[2] <= 1.5);
        CHECK(std::abs(frames.frames[i].Origin()[1]) < 1e-9);
    }
    CHECK(frames.frames[3].Origin()[2] > frames.frames[1].Origin()[2]);
    CHECK(frames.frames[3].Origin()[2] > frames.frames[5].Origin()[2]);
    for (size_t i = 0; i < 7; ++i) {
        CHECK(Near(SplineIkUnitY(frames.frames[i]), GfVec3d(0, 1, 0), 1e-9));
    }
    // Exactly the curve those CVs define.
    const RigExecSplineIkCurve curve(
        {GfVec3d(0, 0, 0), GfVec3d(1, 0, 1.5), GfVec3d(5, 0, 1.5), GfVec3d(6, 0, 0)});
    const double ratio = curve.ArcLength() / 6.0;
    for (size_t i = 0; i < 7; ++i) {
        GfVec3d expected;
        CHECK(curve.PointAtArcLength(ratio * i, &expected, nullptr));
        CHECK(Near(frames.frames[i].Origin(), expected, 1e-9));
    }

    // inputs:midFollowWeight moves the follow point. With the end lifted
    // +2 in Z and the mid control left alone: weight 0 follows the root
    // only, so the mid registers no offset and only cv2/cv3 rise; weight 1
    // follows the end, so the mid registers minus the end displacement and
    // cv1/cv2 drop by it. Each is exactly the curve those CVs define.
    SplineIkSet(s.mid, "avars:tz", 0.0);
    SplineIkSet(s.end, "avars:tz", 2.0);
    const auto expectCurve = [&](const RigExecPointFrameArray &r,
                                 const std::array<GfVec3d, 4> &cvs) {
        CHECK(r.GetSize() == 7);
        if (r.GetSize() != 7) {
            return;
        }
        const RigExecSplineIkCurve c(cvs);
        const double ratio = c.ArcLength() / 6.0;
        for (size_t i = 0; i < 7; ++i) {
            GfVec3d expected;
            CHECK(c.PointAtArcLength(ratio * i, &expected, nullptr));
            CHECK(Near(r.frames[i].Origin(), expected, 1e-9));
        }
    };
    SplineIkSet(s.solver, "inputs:midFollowWeight", 0.0);
    expectCurve(SplineIkSolve(s),
                {GfVec3d(0, 0, 0), GfVec3d(1, 0, 0), GfVec3d(5, 0, 2), GfVec3d(6, 0, 2)});
    SplineIkSet(s.solver, "inputs:midFollowWeight", 1.0);
    expectCurve(SplineIkSolve(s),
                {GfVec3d(0, 0, 0), GfVec3d(1, 0, -2), GfVec3d(5, 0, 0), GfVec3d(6, 0, 2)});
    SplineIkSet(s.solver, "inputs:midFollowWeight", 0.5);
    expectCurve(SplineIkSolve(s),
                {GfVec3d(0, 0, 0), GfVec3d(1, 0, -1), GfVec3d(5, 0, 1), GfVec3d(6, 0, 2)});
}

static void
TestSplineIkTwist()
{
    const SplineIkStage s = MakeSplineIkStage(SplineIkStraightOrigins());
    const RigExecPointFrameArray plain = SplineIkSolve(s);
    CHECK(plain.GetSize() == 7);
    if (plain.GetSize() != 7) {
        return;
    }
    // Signed rotation of a joint's posed up about its aim, relative to the
    // untwisted solve.
    const auto upRotation = [&](const RigExecPointFrameArray &r, size_t i) {
        const GfVec3d x = SplineIkUnitX(r.frames[i]);
        const GfVec3d y0 = SplineIkUnitY(plain.frames[i]);
        const GfVec3d y1 = SplineIkUnitY(r.frames[i]);
        return std::atan2(GfDot(GfCross(y0, y1), x), GfDot(y0, y1));
    };
    const auto degrees = [](double d) { return d * kSplineIkPi / 180.0; };

    // inputs:roll is constant along the chain (the handle's roll).
    {
        SplineIkSet(s.solver, "inputs:roll", 35.0);
        const RigExecPointFrameArray r = SplineIkSolve(s);
        CHECK(r.GetSize() == 7);
        if (r.GetSize() == 7) {
            for (size_t i = 0; i < 7; ++i) {
                CHECK(std::abs(upRotation(r, i) - degrees(35.0)) < 1e-9);
                CHECK(Near(r.frames[i].Origin(), s.origins[i], 1e-9));
            }
        }
        SplineIkSet(s.solver, "inputs:roll", 0.0);
    }
    // Root and end controls rolled together about the chain axis: the
    // root twist is constant along the chain and every joint is its rest
    // frame rotated about that axis.
    {
        SplineIkSet(s.root, "avars:rx", 35.0);
        SplineIkSet(s.end, "avars:rx", 35.0);
        const RigExecPointFrameArray r = SplineIkSolve(s);
        CHECK(r.GetSize() == 7);
        if (r.GetSize() == 7) {
            const GfRotation rot(GfVec3d(1, 0, 0), 35.0);
            for (size_t i = 0; i < 7; ++i) {
                CHECK(std::abs(upRotation(r, i) - degrees(35.0)) < 1e-9);
                for (int k = 0; k < 4; ++k) {
                    CHECK(Near(r.frames[i].points[k],
                               rot.TransformDir(plain.frames[i].points[k]), 1e-9));
                }
            }
        }
        SplineIkSet(s.root, "avars:rx", 0.0);
        SplineIkSet(s.end, "avars:rx", 0.0);
    }
    // End control twisted alone: a gradient exactly linear in t_i = i/6,
    // asserted as equal increments between equally spaced joints, not just
    // at the endpoints.
    {
        SplineIkSet(s.end, "avars:rx", -80.0);
        const RigExecPointFrameArray r = SplineIkSolve(s);
        CHECK(r.GetSize() == 7);
        if (r.GetSize() == 7) {
            const double slope = degrees(-80.0);
            CHECK(std::abs(upRotation(r, 0)) < 1e-9);
            CHECK(std::abs(upRotation(r, 6) - slope) < 1e-9);
            for (size_t i = 0; i < 7; ++i) {
                CHECK(std::abs(upRotation(r, i) - slope * i / 6.0) < 1e-9);
                if (i > 0) {
                    CHECK(std::abs((upRotation(r, i) - upRotation(r, i - 1)) -
                                   slope / 6.0) < 1e-9);
                }
                CHECK(Near(r.frames[i].Origin(), s.origins[i], 1e-9));
            }
        }
        SplineIkSet(s.end, "avars:rx", 0.0);
    }
    // Root control twisted alone: the end holds its orientation, so the
    // roll fades linearly to zero at the tip (roll * (1 - t_i)).
    {
        SplineIkSet(s.root, "avars:rx", 30.0);
        const RigExecPointFrameArray r = SplineIkSolve(s);
        CHECK(r.GetSize() == 7);
        if (r.GetSize() == 7) {
            for (size_t i = 0; i < 7; ++i) {
                CHECK(std::abs(upRotation(r, i) - degrees(30.0) * (1.0 - i / 6.0)) < 1e-9);
            }
        }
        SplineIkSet(s.root, "avars:rx", 0.0);
    }
    // inputs:twist adds a linear gradient on top (the handle's twist).
    {
        SplineIkSet(s.solver, "inputs:twist", 60.0);
        const RigExecPointFrameArray r = SplineIkSolve(s);
        CHECK(r.GetSize() == 7);
        if (r.GetSize() == 7) {
            for (size_t i = 0; i < 7; ++i) {
                CHECK(std::abs(upRotation(r, i) - degrees(60.0) * i / 6.0) < 1e-9);
            }
        }
        SplineIkSet(s.solver, "inputs:twist", 0.0);
    }
}

static void
TestSplineIkSquash()
{
    const SplineIkStage s = MakeSplineIkStage(SplineIkStraightOrigins());
    const std::vector<float> weights = {
        0.0f, 0.1429f, 0.5f, 1.0f, 0.25f, 0.3571f, 0.0714f};
    CHECK(s.solver.GetAttribute(TfToken("rigExec:volumeWeights"))
              .Set(VtFloatArray(weights.begin(), weights.end())));
    const auto solveWithEndAt = [&](double x, double preserveVolume) {
        SplineIkSet(s.end, "avars:tx", x - 6.0);
        SplineIkSet(s.mid, "avars:tx", (x - 6.0) * 0.5);
        SplineIkSet(s.solver, "inputs:preserveVolume", preserveVolume);
        return SplineIkSolve(s);
    };
    // s_y = s_z = 1 - w_i * preserveVolume * (ratio - 1), s_x = 1.
    const auto expectScale = [&](const RigExecPointFrameArray &r, double ratio,
                                 double preserveVolume, size_t i) {
        const double expected = 1.0 - double(weights[i]) * preserveVolume * (ratio - 1.0);
        CHECK(std::abs(SplineIkHandle(r.frames[i], 2) - expected) < 1e-9);
        CHECK(std::abs(SplineIkHandle(r.frames[i], 3) - expected) < 1e-9);
        CHECK(std::abs(SplineIkHandle(r.frames[i], 1) - 1.0) < 1e-9);
    };

    // ratio 1.5, full strength: s = 1 - w / 2.
    const RigExecPointFrameArray stretched = solveWithEndAt(9.0, 1.0);
    CHECK(stretched.GetSize() == 7);
    if (stretched.GetSize() != 7) {
        return;
    }
    CHECK(std::abs(SplineIkHandle(stretched.frames[3], 2) - 0.5) < 1e-9);
    CHECK(std::abs(SplineIkHandle(stretched.frames[2], 2) - 0.75) < 1e-9);
    CHECK(std::abs(SplineIkHandle(stretched.frames[0], 2) - 1.0) < 1e-9);
    for (size_t i = 0; i < 7; ++i) {
        expectScale(stretched, 1.5, 1.0, i);
        CHECK(Near(stretched.frames[i].Origin(), GfVec3d(1.5 * i, 0, 0), 1e-9));
    }
    // The scale is what element extraction hands the bound joint: the
    // out-space measures posed/rest handle-length ratios per axis, so
    // the joint's frame carries (1, s, s) -- non-uniform, not laundered.
    const RigExecPointFrame extracted = RigExecExtractElementFrame(&stretched, 3);
    CHECK(extracted.IsValid() && !extracted.IsDegenerate());
    CHECK(std::abs(SplineIkHandle(extracted, 1) - 1.0) < 1e-9);
    CHECK(std::abs(SplineIkHandle(extracted, 2) - 0.5) < 1e-9);
    CHECK(std::abs(SplineIkHandle(extracted, 3) - 0.5) < 1e-9);
    CHECK(Near(extracted.Origin(), GfVec3d(4.5, 0, 0), 1e-9));

    // Partial strength: s = 1 - w * 0.4 * 0.5.
    const RigExecPointFrameArray partial = solveWithEndAt(9.0, 0.4);
    CHECK(partial.GetSize() == 7);
    if (partial.GetSize() == 7) {
        CHECK(std::abs(SplineIkHandle(partial.frames[3], 2) - 0.8) < 1e-9);
        CHECK(std::abs(SplineIkHandle(partial.frames[2], 3) - 0.9) < 1e-9);
        for (size_t i = 0; i < 7; ++i) {
            expectScale(partial, 1.5, 0.4, i);
        }
    }
    // preserveVolume 0: no thinning anywhere, the stretch still applies.
    const RigExecPointFrameArray off = solveWithEndAt(9.0, 0.0);
    CHECK(off.GetSize() == 7);
    if (off.GetSize() == 7) {
        for (size_t i = 0; i < 7; ++i) {
            for (int axis = 1; axis < 4; ++axis) {
                CHECK(std::abs(SplineIkHandle(off.frames[i], axis) - 1.0) < 1e-9);
            }
            CHECK(Near(off.frames[i].Origin(), GfVec3d(1.5 * i, 0, 0), 1e-9));
        }
    }
    // ratio 0.8 (squash): s = 1 + w * 0.2, thickening.
    const RigExecPointFrameArray squashed = solveWithEndAt(4.8, 1.0);
    CHECK(squashed.GetSize() == 7);
    if (squashed.GetSize() == 7) {
        CHECK(std::abs(SplineIkHandle(squashed.frames[3], 2) - 1.2) < 1e-9);
        CHECK(std::abs(SplineIkHandle(squashed.frames[2], 2) - 1.1) < 1e-9);
        CHECK(std::abs(SplineIkHandle(squashed.frames[4], 2) - 1.05) < 1e-9);
        for (size_t i = 0; i < 7; ++i) {
            expectScale(squashed, 0.8, 1.0, i);
            CHECK(Near(squashed.frames[i].Origin(), GfVec3d(0.8 * i, 0, 0), 1e-9));
        }
    }
}

static void
TestSplineIkEvaluatorBinding()
{
    // The evaluator recognizes the solver as an aggregate, accepts its
    // rigExec:joints claim, binds each joint to its chain slot, and the
    // per-joint non-uniform scale reaches the joint frames and matrices.
    SplineIkStage s = MakeSplineIkStage(SplineIkStraightOrigins());
    const std::vector<float> weights = {
        0.0f, 0.1429f, 0.5f, 1.0f, 0.25f, 0.3571f, 0.0714f};
    CHECK(s.solver.GetAttribute(TfToken("rigExec:volumeWeights"))
              .Set(VtFloatArray(weights.begin(), weights.end())));
    SplineIkSet(s.end, "avars:tx", 3.0);
    SplineIkSet(s.mid, "avars:tx", 1.5);

    RigExecRigEvaluator evaluator(s.stage, SdfPath("/Asset/Rig"));
    std::vector<std::string> errors;
    CHECK(evaluator.Compile(&errors));
    for (const std::string &e : errors) {
        std::printf("  compile: %s\n", e.c_str());
    }
    CHECK(errors.empty());
    const RigExecRigPose pose = evaluator.Evaluate(UsdTimeCode::Default());
    CHECK(pose.valid);
    for (const std::string &d : pose.diagnostics) {
        std::printf("  diagnostic: %s\n", d.c_str());
    }
    const auto solverFrames = pose.solverFrames.find(s.solver.GetPath());
    CHECK(solverFrames != pose.solverFrames.end() &&
          solverFrames->second.size() == 7);
    for (size_t i = 0; i < 7; ++i) {
        const auto it = pose.jointFramesFinal.find(s.joints[i].GetPath());
        CHECK(it != pose.jointFramesFinal.end());
        if (it == pose.jointFramesFinal.end()) {
            continue;
        }
        const RigExecPointFrame &f = it->second;
        CHECK(f.IsValid() && !f.IsDegenerate());
        CHECK(Near(f.Origin(), GfVec3d(1.5 * i, 0, 0), 1e-9));
        const double expected = 1.0 - double(weights[i]) * 0.5;
        CHECK(std::abs(SplineIkHandle(f, 1) - 1.0) < 1e-9);
        CHECK(std::abs(SplineIkHandle(f, 2) - expected) < 1e-9);
        CHECK(std::abs(SplineIkHandle(f, 3) - expected) < 1e-9);
    }
    // The joint matrix carries the same (1, s, s): its Y row is half length.
    const auto m = pose.jointMatricesFinal.find(s.joints[3].GetPath());
    CHECK(m != pose.jointMatricesFinal.end());
    if (m != pose.jointMatricesFinal.end()) {
        const GfVec3d xRow(m->second[0][0], m->second[0][1], m->second[0][2]);
        const GfVec3d yRow(m->second[1][0], m->second[1][1], m->second[1][2]);
        const GfVec3d zRow(m->second[2][0], m->second[2][1], m->second[2][2]);
        CHECK(std::abs(xRow.GetLength() - 1.0) < 1e-9);
        CHECK(std::abs(yRow.GetLength() - 0.5) < 1e-9);
        CHECK(std::abs(zRow.GetLength() - 0.5) < 1e-9);
    }

    // A value edit on a control re-solves without a recompile.
    const size_t epoch = evaluator.GetBindingEpochDigest();
    SplineIkSet(s.end, "avars:tx", 0.0);
    SplineIkSet(s.mid, "avars:tx", 0.0);
    const RigExecRigPose atRest = evaluator.Evaluate(UsdTimeCode::Default());
    CHECK(atRest.valid);
    CHECK(evaluator.GetBindingEpochDigest() == epoch);
    const auto tip = atRest.jointFramesFinal.find(s.joints[6].GetPath());
    CHECK(tip != atRest.jointFramesFinal.end());
    if (tip != atRest.jointFramesFinal.end()) {
        CHECK(Near(tip->second.Origin(), GfVec3d(6, 0, 0), 1e-9));
        CHECK(std::abs(SplineIkHandle(tip->second, 2) - 1.0) < 1e-9);
    }

    // rigExec:jointElements remaps list position to chain slot: listing
    // the tip first with a permutation binds every joint to the same slot.
    {
        SplineIkStage p = MakeSplineIkStage(SplineIkStraightOrigins());
        SplineIkSet(p.end, "avars:tx", 3.0);
        SplineIkSet(p.mid, "avars:tx", 1.5);
        SdfPathVector order = {p.joints[6].GetPath()};
        VtIntArray elements = {6};
        for (int i = 0; i < 6; ++i) {
            order.push_back(p.joints[i].GetPath());
            elements.push_back(i);
        }
        p.solver.GetRelationship(TfToken("rigExec:joints")).SetTargets(order);
        CHECK(p.solver.GetAttribute(TfToken("rigExec:jointElements")).Set(elements));
        RigExecRigEvaluator permuted(p.stage, SdfPath("/Asset/Rig"));
        std::vector<std::string> permutedErrors;
        CHECK(permuted.Compile(&permutedErrors));
        CHECK(permutedErrors.empty());
        const RigExecRigPose remapped = permuted.Evaluate(UsdTimeCode::Default());
        CHECK(remapped.valid);
        for (size_t i = 0; i < 7; ++i) {
            const auto it = remapped.jointFramesFinal.find(p.joints[i].GetPath());
            CHECK(it != remapped.jointFramesFinal.end());
            if (it != remapped.jointFramesFinal.end()) {
                CHECK(Near(it->second.Origin(), GfVec3d(1.5 * i, 0, 0), 1e-9));
            }
        }
    }
    // Compile rejects a volumeWeights array that is not parallel to the
    // chain, and a remap that fills one chain slot twice.
    {
        SplineIkStage bad = MakeSplineIkStage(SplineIkStraightOrigins());
        CHECK(bad.solver.GetAttribute(TfToken("rigExec:volumeWeights"))
                  .Set(VtFloatArray{0.1f, 0.2f, 0.3f}));
        RigExecRigEvaluator rejected(bad.stage, SdfPath("/Asset/Rig"));
        std::vector<std::string> badErrors;
        CHECK(!rejected.Compile(&badErrors));
        bool named = false;
        for (const std::string &e : badErrors) {
            named = named || e.find("rigExec:volumeWeights") != std::string::npos;
        }
        CHECK(named);
    }
    {
        SplineIkStage bad = MakeSplineIkStage(SplineIkStraightOrigins());
        CHECK(bad.solver.GetAttribute(TfToken("rigExec:jointElements"))
                  .Set(VtIntArray{0, 1, 2, 3, 4, 5, 5}));
        RigExecRigEvaluator rejected(bad.stage, SdfPath("/Asset/Rig"));
        std::vector<std::string> badErrors;
        CHECK(!rejected.Compile(&badErrors));
        bool named = false;
        for (const std::string &e : badErrors) {
            named = named || e.find("twice") != std::string::npos;
        }
        CHECK(named);
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
    TestDynamicWeightDenseBaseFormulaAndParity();
    TestSingleChainIkUniversalWeightObject();
    TestConstraintCompositionAndHierarchy();
    TestReverseSiblingPostOrder();
    TestModeAndFailureContracts();
    TestXformableTargetsCompile();
    TestTransformProviderPredicate();
    TestPointDomainMoverNamesTheFix();
    TestMeshAndXformTargetsAgree();
    TestGeometryDomainTargetCompiles();
    TestGeometryConstraintDenseEnvelopeSupersedesScalar();
    TestTransformAndGeometrySpellingsAgree();
    TestGeometryEnvelopeIsChordLerp();
    TestDomainsOnOnePrimDoNotCompete();
    TestUnwiringOneMoverLeavesTheRestRunning();
    TestConstraintRegistryCoversTheSchema();
    TestRotationOrderCapabilityIsRecorded();
    TestLegacyWeightSpellingIsRejected();
    TestInvalidContractsFailClosed();
    TestSolverBindsJointsUnderAnyScope();
    TestIncompleteSolverLeavesJointsVisible();
    TestTwoBoneIkImpliedLengths(false);
    TestTwoBoneIkImpliedLengths(true);
    TestTwoBoneIkRestFrameInputs();
    TestSplineIkRest();
    TestSplineIkStretch();
    TestSplineIkMidBend();
    TestSplineIkTwist();
    TestSplineIkSquash();
    TestSplineIkEvaluatorBinding();
    TestAggregateSolverValueUpdates();
    TestDeepSolverDependencySchedule();
    TestSolverTransitiveConnectionInvalidation();
    TestConstraintSolverDependencySchedule();
    TestConstrainedSolverInputAncestor();
    TestSolverOwnedJointBlocksNamespacePropagation();
    TestConnectedParentSpaceSolverInputs();
    TestSolverGuidesGate();
    TestSolverBatchLevelAudit();

    if (failures) {
        std::printf("%d FAILURE(S)\n", failures);
        return 1;
    }
    std::printf("testRigExecConstraints: all tests passed\n");
    return 0;
}
