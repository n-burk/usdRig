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
        const double restX[] = {0.0, 1.0, 2.0};
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

// Geometry-domain constraints publish their solved delta directly onto the
// target points. A bound dense object is the total per-point envelope on this
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
    for (const float envelope : {1.0f}) {
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
            MakeXform(stage, SdfPath("/Asset/Source"),
                      Matrix(GfVec3d(0, 0, 0),
                             GfRotation(GfVec3d(0, 1, 0), 90.0)));
            stage->DefinePrim(SdfPath("/Asset/Rig"), TfToken("RigExecRoot"));
            stage->DefinePrim(SdfPath("/Asset/Rig/Movers"), TfToken("Scope"));
            const SdfPath pointsPath =
                SdfPath("/Asset/Geom/M").AppendProperty(TfToken("points"));
            const UsdPrim rot = MakeConstraint(
                stage, "Rot", "RigExecRotationConstraint",
                {geometry ? pointsPath : SdfPath("/Asset/Geom/M")});
            rot.CreateRelationship(TfToken("rigExec:sources"))
                .SetTargets({SdfPath("/Asset/Source")});
            rot.GetAttribute(TfToken("inputs:defaultWeight")).Set(envelope);

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
                        viaPoints.push_back(GfVec3d(p));
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

    // The scalar oracle does not cover this publish, and says so rather
    // than reporting an untested agreement. Pinned so that routing the
    // geometry domain through the mover graph -- which is what would restore
    // real coverage -- has to remove this assertion deliberately.
    CHECK(pose.movedPropertiesCpu.find(pointsPath) ==
          pose.movedPropertiesCpu.end());
    CHECK(std::any_of(pose.diagnostics.begin(), pose.diagnostics.end(),
                      [](const std::string &d) {
                          return d.find("not covered") != std::string::npos;
                      }));
}

// Competing writers are keyed by the exact target, and that is correct.
//
// Two writers of the SAME points set are ambiguous and rejected. But a
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
    stage->DefinePrim(SdfPath("/Asset/Rig/Joints/Shoulder/Elbow"),
                      TfToken("RigExecJoint"));
    stage->DefinePrim(SdfPath("/Asset/Rig/Joints/Shoulder/Elbow/Wrist"),
                      TfToken("RigExecJoint"));

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
    ik.CreateAttribute(TfToken("rigExec:upperLength"),
                       SdfValueTypeNames->Double)
        .Set(3.0);
    ik.CreateAttribute(TfToken("rigExec:lowerLength"),
                       SdfValueTypeNames->Double)
        .Set(3.0);

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
    const double restTx[3] = {1.0, 2.0, 3.0};
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
    ik.CreateAttribute(TfToken("rigExec:upperLength"),
                       SdfValueTypeNames->Double)
        .Set(3.0);
    ik.CreateAttribute(TfToken("rigExec:lowerLength"),
                       SdfValueTypeNames->Double)
        .Set(3.0);

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
    // bind-world position (world = rest when the parent is at rest), so
    // each joint sits exactly at its authored rest.
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
TestTwoBoneIkImpliedLengths()
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

    const SdfPath shoulder("/Asset/Rig/Joints/Shoulder");
    const SdfPath elbow("/Asset/Rig/Joints/Shoulder/Elbow");
    const SdfPath wrist("/Asset/Rig/Joints/Shoulder/Elbow/Wrist");
    const double restTx[3] = {0.0, 3.0, 7.0};
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
    ik.CreateRelationship(TfToken("rigExec:joints"))
        .SetTargets({shoulder, elbow, wrist});
    // No lengths authored: both are implied.

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
    CHECK(hasDiagnostic(restPose, "implied rigExec:upperLength=3"));
    CHECK(hasDiagnostic(restPose, "implied rigExec:lowerLength=4"));

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
    CHECK(hasDiagnostic(offsetPose, "implied rigExec:lowerLength=5"));

    // Authored absolutes win over rests: 2.5/2.5 with no stretch clamps
    // the (7, 0, 0) goal to a reach of 5. Authoring the lengths changes
    // the epoch, which Evaluate recompiles by itself.
    ik.CreateAttribute(TfToken("rigExec:upperLength"),
                       SdfValueTypeNames->Double)
        .Set(2.5);
    ik.CreateAttribute(TfToken("rigExec:lowerLength"),
                       SdfValueTypeNames->Double)
        .Set(2.5);
    ik.CreateAttribute(TfToken("inputs:stretch"), SdfValueTypeNames->Float)
        .Set(0.0f);
    effCtl.GetAttribute(TfToken("avars:tx")).Set(0.0);
    GfVec3d authored[3];
    const RigExecRigPose authoredPose =
        origins(UsdTimeCode::Default(), authored);
    CHECK(Near(authored[1], GfVec3d(2.5, 0, 0)));
    CHECK(Near(authored[2], GfVec3d(5, 0, 0)));
    CHECK(!hasDiagnostic(authoredPose, "implied rigExec:"));
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
    TestTwoBoneIkImpliedLengths();

    if (failures) {
        std::printf("%d FAILURE(S)\n", failures);
        return 1;
    }
    std::printf("testRigExecConstraints: all tests passed\n");
    return 0;
}
