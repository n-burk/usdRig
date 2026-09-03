//
// Strict codeless-schema authoring: declarations are the only authority.
//
#include "rigExecRigging/schemaAuthoring.h"

#include "rigExecMath/avarScale.h"

#include "pxr/base/gf/matrix4d.h"
#include "pxr/base/plug/registry.h"
#include "pxr/base/tf/pathUtils.h"
#include "pxr/usd/usd/attribute.h"
#include "pxr/usd/usd/relationship.h"
#include "pxr/usd/usd/schemaRegistry.h"
#include "pxr/usd/usd/stage.h"

#include <cstdio>
#include <exception>
#include <limits>
#include <string>
#include <utility>

PXR_NAMESPACE_USING_DIRECTIVE

using rigExec::RigExecSchemaPrim;
using rigExec::RigExecAvarScaleFloor;

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

static int
TestMissingSchemaFailsClosed()
{
    CHECK(!UsdSchemaRegistry::GetInstance().FindConcretePrimDefinition(
        TfToken("RigExecRoot")));
    const UsdStageRefPtr stage = UsdStage::CreateInMemory();
    CHECK(Throws([&] {
        RigExecSchemaPrim::Define(
            stage, SdfPath("/MustNotExist"), TfToken("RigExecRoot"));
    }));
    CHECK(!stage->GetPrimAtPath(SdfPath("/MustNotExist")));
    return failures == 0 ? 0 : 1;
}

static void
TestDefineGetAndStrictAttributes(const UsdStageRefPtr &stage)
{
    RigExecSchemaPrim root = RigExecSchemaPrim::Define(
        stage, SdfPath("/Rig"), TfToken("RigExecRoot"));
    CHECK(root.IsValid());
    CHECK(root.GetPath() == SdfPath("/Rig"));
    CHECK(root.GetSchemaTypeName() == TfToken("RigExecRoot"));
    CHECK(root.GetPrim().GetTypeName() == TfToken("RigExecRoot"));

    // Idempotent Define and exact-type Get both retain the schema contract.
    CHECK(RigExecSchemaPrim::Define(
              stage, SdfPath("/Rig"), TfToken("RigExecRoot"))
              .IsValid());
    CHECK(RigExecSchemaPrim::Get(
              stage, SdfPath("/Rig"), TfToken("RigExecRoot"))
              .IsValid());
    CHECK(Throws([&] {
        RigExecSchemaPrim::Get(
            stage, SdfPath("/Rig"), TfToken("RigExecControl"));
    }));
    CHECK(Throws([&] {
        RigExecSchemaPrim::Get(
            stage, SdfPath("/Absent"), TfToken("RigExecRoot"));
    }));

    root.SetAttribute(
        TfToken("rigExec:partition"), VtValue(TfToken("Character")));
    const UsdAttribute partition =
        root.GetPrim().GetAttribute(TfToken("rigExec:partition"));
    TfToken partitionValue;
    CHECK(partition && !partition.IsCustom());
    CHECK(partition.GetVariability() == SdfVariabilityUniform);
    CHECK(partition.Get(&partitionValue));
    CHECK(partitionValue == TfToken("Character"));

    // A caller cannot supply a type, and the exact schema type is enforced
    // before Set() can author anything.
    CHECK(Throws([&] {
        root.SetAttribute(
            TfToken("rigExec:partition"), VtValue(std::string("wrong type")));
    }));
    CHECK(partition.Get(&partitionValue));
    CHECK(partitionValue == TfToken("Character"));

    CHECK(Throws([&] {
        root.SetAttribute(TfToken("rigExec:notDeclared"), VtValue(1.0f));
    }));
    CHECK(!root.GetPrim().GetProperty(TfToken("rigExec:notDeclared")));
    CHECK(Throws([&] {
        root.SetRelationship(
            TfToken("rigExec:partition"), {SdfPath("/Target")});
    }));

    CHECK(Throws([&] {
        root.SetAttribute(
            TfToken("rigExec:partition"), VtValue(TfToken("Animated")),
            UsdTimeCode(12.0));
    }));
    CHECK(partition.Get(&partitionValue));
    CHECK(partitionValue == TfToken("Character"));
    CHECK(partition.HasAuthoredValueOpinion());
    root.ClearAttribute(TfToken("rigExec:partition"));
    CHECK(!partition.HasAuthoredValueOpinion());
}

static void
TestAppliedSchemasAndRelationships(const UsdStageRefPtr &stage)
{
    RigExecSchemaPrim control = RigExecSchemaPrim::Define(
        stage, SdfPath("/Rig/Control"), TfToken("RigExecControl"));
    CHECK(!control.HasAPI(TfToken("RigExecControlAPI")));
    CHECK(!control.GetPrim().GetAttribute(TfToken("rigExec:channelRole")));
    control.ApplyAPI(TfToken("RigExecControlAPI"));
    CHECK(control.HasAPI(TfToken("RigExecControlAPI")));
    CHECK(control.GetPrim().GetAttribute(TfToken("rigExec:channelRole")));
    control.SetAttribute(
        TfToken("rigExec:channelRole"), VtValue(TfToken("tweak")));

    // The public call takes the USD schema identifier directly. This standard
    // API's identifier differs from its C++ TfType name, making it a regression
    // test for identifier-vs-TfType confusion.
    control.ApplyAPI(TfToken("NodeGraphNodeAPI"));
    CHECK(control.HasAPI(TfToken("NodeGraphNodeAPI")));

    CHECK(Throws([&] {
        control.ApplyAPI(TfToken("RigExecMissingAPI"));
    }));

    RigExecSchemaPrim mover = RigExecSchemaPrim::Define(
        stage, SdfPath("/Rig/Mover"), TfToken("RigExecMatrixMover"));
    CHECK(!mover.GetPrim().GetRelationship(TfToken("rigExec:moves")));
    mover.ApplyAPI(TfToken("RigExecMoverAPI"));
    CHECK(mover.HasAPI(TfToken("RigExecMoverAPI")));

    // Every mover gets the same normalized envelope and optional field from
    // the applied API.  These are deliberately API properties rather than
    // per-mover spellings: authoring a new mover kind must not create a new
    // name for the same blend contract.
    const UsdAttribute defaultWeight =
        mover.GetPrim().GetAttribute(TfToken("inputs:defaultWeight"));
    float defaultWeightValue = 0.0f;
    CHECK(defaultWeight && !defaultWeight.IsCustom());
    CHECK(defaultWeight.GetTypeName() == SdfValueTypeNames->Float);
    CHECK(defaultWeight.Get(&defaultWeightValue));
    CHECK(defaultWeightValue == 1.0f);
    if (defaultWeight) {
        mover.SetAttribute(TfToken("inputs:defaultWeight"), VtValue(0.5f));
        CHECK(defaultWeight.Get(&defaultWeightValue));
        CHECK(defaultWeightValue == 0.5f);
    }

    const SdfPathVector targets = {SdfPath("/Geometry.points")};
    mover.SetRelationship(TfToken("rigExec:moves"), targets);
    const UsdRelationship moves =
        mover.GetPrim().GetRelationship(TfToken("rigExec:moves"));
    SdfPathVector authoredTargets;
    CHECK(moves && !moves.IsCustom());
    CHECK(moves.GetTargets(&authoredTargets));
    CHECK(authoredTargets == targets);

    mover.SetAttribute(TfToken("inputs:enabled"), VtValue(false));
    bool enabled = true;
    CHECK(mover.GetPrim().GetAttribute(TfToken("inputs:enabled")).Get(&enabled));
    CHECK(!enabled);

    CHECK(Throws([&] {
        mover.SetRelationship(
            TfToken("rigExec:notDeclared"), {SdfPath("/Target")});
    }));
    CHECK(!mover.GetPrim().GetProperty(TfToken("rigExec:notDeclared")));
    CHECK(Throws([&] {
        mover.SetAttribute(TfToken("rigExec:moves"), VtValue(TfToken("bad")));
    }));

    // The weight-object relationship is common too, including for a scalar
    // property mover.  MatrixMover used to declare this relationship itself,
    // so using FloatMathMover here proves it really comes from MoverAPI.
    RigExecSchemaPrim scalarMover = RigExecSchemaPrim::Define(
        stage, SdfPath("/Rig/ScalarMover"), TfToken("RigExecFloatMathMover"));
    scalarMover.ApplyAPI(TfToken("RigExecMoverAPI"));
    CHECK(!Throws([&] {
        scalarMover.SetRelationship(
            TfToken("rigExec:weightObject"),
            {SdfPath("/Rig/Weights/Scalar")});
    }));
    const UsdRelationship scalarWeight =
        scalarMover.GetPrim().GetRelationship(TfToken("rigExec:weightObject"));
    CHECK(scalarWeight && !scalarWeight.IsCustom());
    authoredTargets.clear();
    CHECK(scalarWeight.GetTargets(&authoredTargets));
    CHECK(authoredTargets == SdfPathVector({SdfPath("/Rig/Weights/Scalar")}));

    // Strict replacement: the former mover-specific envelope names are not
    // aliases.  Leaving them declared would let two independent knobs fight
    // over one operation; accepting them as custom properties would silently
    // ignore authored intent.
    const std::pair<const char *, const char *> legacyEnvelopeNames[] = {
        {"RigExecFloatMathMover", "inputs:weight"},
        {"RigExecVec3fMathMover", "inputs:weight"},
        {"RigExecMatrixMathMover", "inputs:weight"},
        {"RigExecSmoothMover", "inputs:strength"},
        {"RigExecCurvenetMover", "inputs:strength"},
        {"RigExecVolumeCorrectMover", "inputs:strength"},
    };
    for (const auto &legacyEnvelopeName : legacyEnvelopeNames) {
        const char *typeName = legacyEnvelopeName.first;
        const char *propertyName = legacyEnvelopeName.second;
        const SdfPath path = SdfPath("/Rig").AppendChild(
            TfToken(std::string(typeName).substr(std::string("RigExec").size())));
        RigExecSchemaPrim operation = RigExecSchemaPrim::Define(
            stage, path, TfToken(typeName));
        operation.ApplyAPI(TfToken("RigExecMoverAPI"));
        CHECK(Throws([&] {
            operation.SetAttribute(TfToken(propertyName), VtValue(0.5f));
        }));
        CHECK(!operation.GetPrim().GetProperty(TfToken(propertyName)));
        CHECK(operation.GetPrim().GetAttribute(
            TfToken("inputs:defaultWeight")));
    }
}

static void
TestAvarScaleSchemaContract(const UsdStageRefPtr &stage)
{
    struct Case {
        const char *path;
        const char *schemaType;
    };
    const Case cases[] = {
        {"/ScaleControl", "RigExecControl"},
        {"/ScaleJoint", "RigExecJoint"},
    };
    for (const Case &testCase : cases) {
        RigExecSchemaPrim authored = RigExecSchemaPrim::Define(
            stage, SdfPath(testCase.path), TfToken(testCase.schemaType));
        const UsdPrim prim = authored.GetPrim();
        const char *names[] = {"avars:sx", "avars:sy", "avars:sz"};
        for (const char *name : names) {
            const UsdAttribute attr = prim.GetAttribute(TfToken(name));
            double value = 0.0;
            CHECK(attr && !attr.IsCustom());
            CHECK(attr.GetTypeName() == SdfValueTypeNames->Double);
            CHECK(attr.Get(&value));
            CHECK(value == 1.0);
            CHECK(!attr.HasAuthoredValueOpinion());
        }

        authored.SetAttribute(TfToken("avars:sx"), VtValue(2.0));
        authored.SetAttribute(TfToken("avars:sy"), VtValue(3.0));
        authored.SetAttribute(TfToken("avars:sz"), VtValue(4.0));
        double sy = 0.0;
        CHECK(prim.GetAttribute(TfToken("avars:sy")).Get(&sy));
        CHECK(sy == 3.0);

        // The low-level strict facade shares the runtime's signed magnitude
        // floor. Negative zero and negative sub-floor values remain mirrors.
        authored.SetAttribute(TfToken("avars:sx"), VtValue(0.0));
        authored.SetAttribute(TfToken("avars:sy"), VtValue(-0.0));
        authored.SetAttribute(
            TfToken("avars:sz"),
            VtValue(-0.5 * RigExecAvarScaleFloor));
        double sx = 0.0, sz = 0.0;
        CHECK(prim.GetAttribute(TfToken("avars:sx")).Get(&sx));
        CHECK(prim.GetAttribute(TfToken("avars:sy")).Get(&sy));
        CHECK(prim.GetAttribute(TfToken("avars:sz")).Get(&sz));
        CHECK(sx == RigExecAvarScaleFloor);
        CHECK(sy == -RigExecAvarScaleFloor);
        CHECK(sz == -RigExecAvarScaleFloor);

        // Exact schema typing fails before mutation; a rejected float payload
        // must preserve the previously authored double value.
        CHECK(Throws([&] {
            authored.SetAttribute(TfToken("avars:sy"), VtValue(9.0f));
        }));
        CHECK(prim.GetAttribute(TfToken("avars:sy")).Get(&sy));
        CHECK(sy == -RigExecAvarScaleFloor);

        // Non-finite scale is not a stage value in the strict contract. Each
        // rejection happens before mutation and preserves the signed floor.
        for (const double nonFinite : {
                 std::numeric_limits<double>::quiet_NaN(),
                 std::numeric_limits<double>::infinity(),
                 -std::numeric_limits<double>::infinity()}) {
            CHECK(Throws([&] {
                authored.SetAttribute(
                    TfToken("avars:sy"), VtValue(nonFinite));
            }));
            CHECK(prim.GetAttribute(TfToken("avars:sy")).Get(&sy));
            CHECK(sy == -RigExecAvarScaleFloor);
        }
    }
}

static void
TestReadPhaseMetadataAndRetypeGuard(const UsdStageRefPtr &stage)
{
    RigExecSchemaPrim mover = RigExecSchemaPrim::Get(
        stage, SdfPath("/Rig/Mover"), TfToken("RigExecMatrixMover"));
    mover.SetAttribute(
        TfToken("rigExec:transformReadPhase"), VtValue(TfToken("base")));
    mover.SetReadPhase(TfToken("rigExec:transform"), "final");
    mover.SetReadPhase(
        TfToken("rigExec:transformReadPhase"),
        "/Rig/Movers/Previous");

    std::string phase;
    CHECK(mover.GetPrim()
              .GetRelationship(TfToken("rigExec:transform"))
              .GetMetadata(TfToken("rigExecReadPhase"), &phase));
    CHECK(phase == "final");
    CHECK(mover.GetPrim()
              .GetAttribute(TfToken("rigExec:transformReadPhase"))
              .GetMetadata(TfToken("rigExecReadPhase"), &phase));
    CHECK(phase == "/Rig/Movers/Previous");

    CHECK(Throws([&] {
        mover.SetReadPhase(TfToken("rigExec:notDeclared"), "base");
    }));
    CHECK(!mover.GetPrim().GetProperty(TfToken("rigExec:notDeclared")));

    // Handles do not become generic authoring proxies if another tool retypes
    // the prim after Get/Define.
    CHECK(stage->DefinePrim(
        SdfPath("/Rig/Mover"), TfToken("RigExecJoint")));
    CHECK(!mover.IsValid());
    CHECK(Throws([&] {
        mover.SetAttribute(TfToken("rest:space"), VtValue(GfMatrix4d(1.0)));
    }));
}

int
main(int argc, char **argv)
{
    if (argc == 2 && std::string(argv[1]) == "--missing-schema") {
        return TestMissingSchemaFailsClosed();
    }

#ifdef RIGEXEC_SCHEMA_RESOURCE_DIR
    const std::string resources = TfAbsPath(RIGEXEC_SCHEMA_RESOURCE_DIR);
#else
    if (argc < 2) {
        std::printf("usage: testRigExecSchemaAuthoring <schemaResources>\n");
        return 2;
    }
    const std::string resources = TfAbsPath(argv[1]);
#endif
    if (PlugRegistry::GetInstance().RegisterPlugins(resources).empty()) {
        std::printf("FATAL: no schema plugin found at %s\n", resources.c_str());
        return 2;
    }

    const UsdStageRefPtr stage = UsdStage::CreateInMemory();
    TestDefineGetAndStrictAttributes(stage);
    TestAppliedSchemasAndRelationships(stage);
    TestAvarScaleSchemaContract(stage);
    TestReadPhaseMetadataAndRetypeGuard(stage);

    if (failures == 0) {
        std::printf("OK: strict RigExec schema authoring\n");
    }
    return failures == 0 ? 0 : 1;
}
