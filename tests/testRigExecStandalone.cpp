#include "rigExecStandalone/system.h"
#include "rigExec/types.h"
#include "pxr/base/plug/registry.h"
#include "pxr/base/tf/errorMark.h"
#include "pxr/usd/sdf/types.h"
#include "pxr/usd/usd/stage.h"
#include "pxr/usd/usd/primRange.h"
#include "pxr/usd/usd/attribute.h"
#include "pxr/usd/usd/relationship.h"
#include <cstdio>
#include <cmath>

using namespace rigExec;
static int failures = 0;
#define CHECK(condition) do { if (!(condition)) { ++failures; std::printf("FAIL %d: %s\n", __LINE__, #condition); } } while (0)

static VtDictionary Metadata(const UsdObject &object) {
    VtDictionary result;
    for (const auto &[key, value] : object.GetAllMetadata()) result[key.GetString()] = value;
    return result;
}

// Test-only producer. Production pack export independently traverses USD and
// serializes the same public Db contract; the runtime sees only these rows.
static RigExecSceneDb Capture(const UsdStageRefPtr &stage, const std::vector<UsdTimeCode> &times)
{
    RigExecSceneDb db;
    for (const auto time : times) db.identities.insert(RigExecStandaloneTimeKey(time));
    db.prims[SdfPath::AbsoluteRootPath()] = {};
    for (const UsdPrim &prim : stage->Traverse()) {
        db.prims[prim.GetPath()] = {prim.GetTypeName(), prim.GetAppliedSchemas(), Metadata(prim), true};
        for (const auto &attribute : prim.GetAttributes()) {
            RigExecStandaloneAttribute row;
            row.type = attribute.GetTypeName();
            row.metadata = Metadata(attribute);
            attribute.GetConnections(&row.connections);
            for (const auto time : times) {
                VtValue value;
                attribute.Get(&value, time);
                row.resolved[RigExecStandaloneTimeKey(time)] = value;
            }
            db.attributes[attribute.GetPath()] = row;
        }
        for (const auto &relationship : prim.GetRelationships()) {
            RigExecStandaloneRelationship row;
            row.metadata = Metadata(relationship);
            relationship.GetTargets(&row.targets);
            db.relationships[relationship.GetPath()] = row;
        }
    }
    std::string error;
    CHECK(db.Validate(&error));
    if (!error.empty()) std::printf("Db: %s\n", error.c_str());
    return db;
}
static bool SameFrame(const RigExecPointFrame &a, const RigExecPointFrame &b)
{
    if (a.IsValid() != b.IsValid() || a.IsDegenerate() != b.IsDegenerate()) return false;
    for (size_t i = 0; i < a.points.size(); ++i) if ((a.points[i] - b.points[i]).GetLength() > 1e-9) return false;
    return true;
}
static bool Same(const VtValue &a, const VtValue &b)
{
    if (a.IsHolding<RigExecPointFrame>() && b.IsHolding<RigExecPointFrame>())
        return SameFrame(a.UncheckedGet<RigExecPointFrame>(), b.UncheckedGet<RigExecPointFrame>());
    if (a.IsHolding<RigExecPointFrameArray>() && b.IsHolding<RigExecPointFrameArray>()) {
        const auto &x = a.UncheckedGet<RigExecPointFrameArray>(), &y = b.UncheckedGet<RigExecPointFrameArray>();
        if (x.frames.size() != y.frames.size() || x.rests != y.rests) return false;
        for (size_t i = 0; i < x.frames.size(); ++i) if (!SameFrame(x.frames[i], y.frames[i])) return false;
        return true;
    }
    return a == b;
}
static void TestRequestBoundaries(const RigExecSceneDb &db)
{
    auto wrongConnectionDb = db;
    wrongConnectionDb.attributes[SdfPath("/Blend.inputs:weight")].connections = {SdfPath("/Unused.value")};
    CHECK(!wrongConnectionDb.Validate());
    auto multipleConnectionDb = db;
    multipleConnectionDb.attributes[SdfPath("/Blend.inputs:weight")].connections = {SdfPath("/Driver.w1"), SdfPath("/Driver.w2")};
    CHECK(!multipleConnectionDb.Validate());
    RigExecStandaloneSystem empty(db);
    CHECK(empty.Prepare());
    const auto first = empty.Evaluate(UsdTimeCode::Default());
    const auto second = empty.Evaluate(UsdTimeCode::Default());
    CHECK(first.valid && first.values.empty() && first.generation == 1);
    CHECK(second.valid && second.generation == 2);
    CHECK(!empty.SetConnections(SdfPath("/Blend.inputs:weight"), {SdfPath("/Driver.w1"), SdfPath("/Driver.w2")}));
    CHECK(!empty.SetValue(SdfPath("/Sample.rigExec:pointsReadPhase"), UsdTimeCode::Default(), VtValue(TfToken("final"))));
    RigExecStandaloneSystem missing(db);
    missing.AddTap(RigExecValueAddress::Prim(SdfPath("/A"), TfToken("unregisteredComputation")));
    const auto unavailable = missing.Evaluate(UsdTimeCode::Default());
    CHECK(!unavailable.valid && !unavailable.diagnostics.empty());
    RigExecStandaloneSystem blocked(db);
    blocked.AddTap(RigExecValueAddress::Property(SdfPath("/Unused.value")));
    CHECK(blocked.Evaluate(UsdTimeCode::Default()).Get<double>(0) == 1.0);
    CHECK(blocked.SetValue(SdfPath("/Unused.value"), UsdTimeCode::Default(), VtValue()));
    CHECK(!blocked.Evaluate(UsdTimeCode::Default()).valid);
    CHECK(blocked.SetValue(SdfPath("/Unused.value"), UsdTimeCode::Default(), VtValue(9.0)));
    const auto revived = blocked.Evaluate(UsdTimeCode::Default());
    CHECK(revived.valid && revived.Get<double>(0) == 9.0);
    for (const char *path : {"/A.avars:tx", "/A.userValue", "/Api.value"}) {
        RigExecStandaloneSystem raw(db);
        raw.AddTap(RigExecValueAddress::Property(SdfPath(path)));
        CHECK(raw.Evaluate(UsdTimeCode::Default()).valid);
        CHECK(raw.SetValue(SdfPath(path), UsdTimeCode::Default(), VtValue()));
        CHECK(!raw.Evaluate(UsdTimeCode::Default()).valid);
        CHECK(raw.SetValue(SdfPath(path), UsdTimeCode::Default(), VtValue(19.0)));
        const auto restored = raw.Evaluate(UsdTimeCode::Default());
        CHECK(restored.valid && restored.Get<double>(0) == 19.0);
    }
    auto unknownApi = db;
    unknownApi.prims.at(SdfPath("/Api")).appliedSchemas = {TfToken("UnqualifiedExpressionAPI")};
    RigExecStandaloneSystem unsupportedApi(unknownApi);
    CHECK(!unsupportedApi.Prepare());

    RigExecStandaloneSystem alias(db);
    alias.AddTap(RigExecValueAddress::Property(SdfPath("/Alias.value")));
    CHECK(alias.Evaluate(UsdTimeCode::Default()).Get<double>(0) == 1.0);
    CHECK(alias.SetPrimActive(SdfPath("/Unused"), false));
    const auto fallback = alias.Evaluate(UsdTimeCode::Default());
    CHECK(fallback.valid && fallback.Get<double>(0) == 4.0);
    CHECK(alias.SetValue(SdfPath("/Alias.value"), UsdTimeCode::Default(), VtValue()));
    CHECK(!alias.Evaluate(UsdTimeCode::Default()).valid);
    CHECK(alias.SetPrimActive(SdfPath("/Unused"), true));
    const auto connected = alias.Evaluate(UsdTimeCode::Default());
    CHECK(connected.valid && connected.Get<double>(0) == 1.0);

    RigExecStandaloneSystem expression(db);
    expression.AddTap(RigExecValueAddress::Property(SdfPath("/B.default:space")));
    CHECK(expression.SetConnections(SdfPath("/B.default:space"), {SdfPath("/Driver.matrix")}));
    CHECK(expression.SetValue(SdfPath("/B.default:space"), UsdTimeCode::Default(), VtValue()));
    const auto computed = expression.Evaluate(UsdTimeCode::Default());
    CHECK(computed.valid && computed.Get<GfMatrix4d>(0).ExtractTranslation() == GfVec3d(17, 0, 0));
    for (const char *type : {"RigExecRibbon", "RigExecSphereWeight", "RigExecPlaneWeight",
                             "RigExecCurveWeight", "RigExecCurvenetAdjuster"}) {
        auto unsupportedDb = db;
        unsupportedDb.prims[SdfPath("/Unsupported")].type = TfToken(type);
        RigExecStandaloneSystem unsupported(unsupportedDb);
        std::string error;
        CHECK(!unsupported.Prepare(&error));
        CHECK(error.find(type) != std::string::npos);
    }
}
static void TestSchemaIdentityLifetime(const RigExecSceneDb &db)
{
    const void *controlKey = db.SchemaKey(SdfPath("/A"));
    for (int i = 0; i < 8; ++i) {
        auto fresh = db;
        fresh.prims.at(SdfPath("/A")).type = TfToken(i % 2 ? "RigExecControl" : "RigExecJoint");
        CHECK(fresh.Validate());
        if (i % 2) CHECK(fresh.SchemaKey(SdfPath("/A")) == controlKey);
        // Each request and database is destroyed before the next iteration.
        // Registry keys must not alias a different schema when memory is reused.
        RigExecStandaloneSystem runtime(fresh);
        runtime.AddTap(RigExecValueAddress::Prim(SdfPath("/A"), TfToken("computePointFrame")));
        runtime.AddTap(RigExecValueAddress::Prim(SdfPath("/Channel"), TfToken("computeBlendChannel")));
        const auto values = runtime.Evaluate(UsdTimeCode::Default());
        CHECK(values.valid && values.Get<RigExecPointFrame>(0).IsValid());
        const auto channel = values.Get<RigExecBlendChannel>(1);
        CHECK(channel.samples.size() == 1 && channel.samples.front().points.size() == 2);
    }
}
int main()
{
    CHECK(!PlugRegistry::GetInstance().RegisterPlugins(RIGEXEC_SCHEMA_RESOURCE_DIR).empty());
    TfErrorMark errors;
    auto stage = UsdStage::CreateInMemory();
    const auto control = [&](const char *path, double rest) {
        const auto prim = stage->DefinePrim(SdfPath(path), TfToken("RigExecControl"));
        prim.GetAttribute(TfToken("rest:tx")).Set(rest);
        return prim;
    };
    const auto a = control("/A", 0), b = control("/B", 3), goal = control("/Goal", 6), pole = control("/Pole", 0);
    a.CreateAttribute(TfToken("userValue"), SdfValueTypeNames->Double).Set(1.0);
    a.GetAttribute(TfToken("avars:ty")).Set(1.0, UsdTimeCode(1));
    a.GetAttribute(TfToken("avars:ty")).Set(2.0, UsdTimeCode(2));
    pole.GetAttribute(TfToken("rest:ty")).Set(5.0);
    const auto fk = stage->DefinePrim(SdfPath("/FK"), TfToken("RigExecFkChain"));
    fk.GetRelationship(TfToken("rigExec:controls")).SetTargets({a.GetPath(), b.GetPath()});
    const auto ik = stage->DefinePrim(SdfPath("/IK"), TfToken("RigExecTwoBoneIk"));
    ik.GetRelationship(TfToken("rigExec:rootControl")).SetTargets({a.GetPath()});
    ik.GetRelationship(TfToken("rigExec:effectorControl")).SetTargets({goal.GetPath()});
    ik.GetRelationship(TfToken("rigExec:poleControl")).SetTargets({pole.GetPath()});
    // Bone lengths are measured from the rests of the joints the solver
    // names -- there is no absolute length attribute to seed. Rests of
    // 0/3/7 give the 3-unit upper and 4-unit lower this used to author.
    const auto joint = [&](const char *path, double rest) {
        const auto prim =
            stage->DefinePrim(SdfPath(path), TfToken("RigExecJoint"));
        prim.GetAttribute(TfToken("rest:tx")).Set(rest);
        return prim;
    };
    ik.GetRelationship(TfToken("rigExec:joints")).SetTargets(
        {joint("/JRoot", 0).GetPath(), joint("/JMid", 3).GetPath(),
         joint("/JEnd", 7).GetPath()});
    const auto blend = stage->DefinePrim(SdfPath("/Blend"), TfToken("RigExecBlendPointFrames"));
    blend.GetRelationship(TfToken("rigExec:inputA")).SetTargets({fk.GetPath()});
    blend.GetRelationship(TfToken("rigExec:inputB")).SetTargets({fk.GetPath()});
    const auto driver = stage->DefinePrim(SdfPath("/Driver"));
    driver.CreateAttribute(TfToken("w1"), SdfValueTypeNames->Float).Set(0.25f);
    driver.CreateAttribute(TfToken("w2"), SdfValueTypeNames->Float).Set(0.75f);
    driver.CreateAttribute(TfToken("matrix"), SdfValueTypeNames->Matrix4d).Set(
        GfMatrix4d(1.0).SetTranslate(GfVec3d(17, 0, 0)));
    blend.GetAttribute(TfToken("inputs:weight")).SetConnections({SdfPath("/Driver.w1")});
    const auto unused = stage->DefinePrim(SdfPath("/Unused"));
    unused.CreateAttribute(TfToken("value"), SdfValueTypeNames->Double).Set(1.0);
    const auto api = stage->DefinePrim(SdfPath("/Api"));
    CHECK(api.AddAppliedSchema(TfToken("MaterialBindingAPI")));
    api.CreateAttribute(TfToken("value"), SdfValueTypeNames->Double).Set(1.0);
    const auto alias = stage->DefinePrim(SdfPath("/Alias"));
    const auto aliasValue = alias.CreateAttribute(TfToken("value"), SdfValueTypeNames->Double);
    aliasValue.Set(4.0);
    aliasValue.SetConnections({SdfPath("/Unused.value")});
    const auto geometry = stage->DefinePrim(SdfPath("/Geometry"), TfToken("Points"));
    const auto points = geometry.GetAttribute(TfToken("points"));
    points.Set(VtVec3fArray{{0, 0, 0}, {1, 0, 0}});
    points.Set(VtVec3fArray{{1, 0, 0}, {2, 0, 0}}, UsdTimeCode(1));
    points.Set(VtVec3fArray{{2, 0, 0}, {3, 0, 0}, {4, 0, 0}}, UsdTimeCode(2));
    const auto sample = stage->DefinePrim(SdfPath("/Sample"), TfToken("RigExecBlendSample"));
    sample.GetRelationship(TfToken("rigExec:targetPoints")).SetTargets({points.GetPath()});
    const auto channel = stage->DefinePrim(SdfPath("/Channel"), TfToken("RigExecBlendInput"));
    channel.GetRelationship(TfToken("rigExec:samples")).SetTargets({sample.GetPath()});
    channel.GetAttribute(TfToken("inputs:weight")).Set(0.4f);
    const std::vector<UsdTimeCode> times{UsdTimeCode::Default(), UsdTimeCode(1), UsdTimeCode(2), UsdTimeCode::PreTime(2)};
    const auto db = Capture(stage, times);
    TestSchemaIdentityLifetime(db);
    TestRequestBoundaries(db);
    const std::vector<RigExecValueAddress> addresses{
        RigExecValueAddress::Prim(a.GetPath(), TfToken("computePointFrame")),
        RigExecValueAddress::Prim(b.GetPath(), TfToken("computePointFrame")),
        RigExecValueAddress::Prim(fk.GetPath(), TfToken("computePointFrameArray")),
        RigExecValueAddress::Prim(ik.GetPath(), TfToken("computePointFrameArray")),
        RigExecValueAddress::Prim(blend.GetPath(), TfToken("computePointFrameArray")),
        RigExecValueAddress::Property(points.GetPath()),
        RigExecValueAddress::Prim(channel.GetPath(), TfToken("computeBlendChannel"))};
    RigExecStandaloneSystem portable(db);
    auto reference = std::make_unique<RigExecTapSet>(stage);
    for (const auto &address : addresses) { portable.AddTap(address); reference->Add(address); }
    std::string error;
    CHECK(portable.Prepare(&error));
    if (!error.empty()) std::printf("prepare: %s\n", error.c_str());
    CHECK(reference->Prepare());
    const void *compiler = portable.GetCompilerIdentity();
    const auto compare = [&](UsdTimeCode time) {
        const auto actual = portable.Evaluate(time);
        const auto expected = reference->Evaluate(time);
        CHECK(actual.valid && expected.IsComplete());
        if (!actual.valid) for (const auto &message : actual.diagnostics) std::printf("runtime: %s\n", message.c_str());
        for (size_t i = 0; i < addresses.size(); ++i) {
            if (!Same(actual.Get(int(i)), expected.Get(int(i)))) {
                std::printf("tap %zu mismatch actual=%s expected=%s\n", i,
                    actual.Get(int(i)).GetTypeName().c_str(), expected.Get(int(i)).GetTypeName().c_str());
                if (i == 6) {
                    const auto x = actual.Get<RigExecBlendChannel>(6), y = expected.Get<RigExecBlendChannel>(6);
                    std::printf("channel actual weight=%f samples=%zu expected weight=%f samples=%zu\n", x.weight, x.samples.size(), y.weight, y.samples.size());
                    for (const auto &sample : x.samples) std::printf("actual activation=%f points=%zu\n",sample.activation,sample.points.size());
                    for (const auto &sample : y.samples) std::printf("expected activation=%f points=%zu\n",sample.activation,sample.points.size());
                }
                CHECK(false);
            }
        }
        CHECK(portable.GetCompilerIdentity() == compiler);
        return actual;
    };
    for (const auto time : times) {
        const auto snapshot = compare(time);
        const auto blendChannel = snapshot.Get<RigExecBlendChannel>(6);
        CHECK(blendChannel.samples.size() == 1);
        if (!blendChannel.samples.empty()) {
            const auto native = snapshot.Get<VtVec3fArray>(5);
            CHECK(blendChannel.samples.front().points == std::vector<GfVec3f>(native.begin(), native.end()));
        }
    }
    const auto retained = compare(UsdTimeCode::Default());
    for (double rest : {1.0, 2.0, 4.0}) {
        CHECK(portable.SetValue(SdfPath("/B.rest:tx"), UsdTimeCode::Default(), VtValue(rest)));
        b.GetAttribute(TfToken("rest:tx")).Set(rest);
        compare(UsdTimeCode::Default());
    }
    CHECK(retained.Get<RigExecPointFrame>(1).Origin() == GfVec3d(3, 0, 0));
    const auto beforeUnused = portable.GetValueInvalidationCount();
    CHECK(portable.SetValue(SdfPath("/Unused.value"), UsdTimeCode::Default(), VtValue(2.0)));
    CHECK(portable.GetValueInvalidationCount() == beforeUnused);
    CHECK(!portable.SetValue(SdfPath("/B.rest:tx"), UsdTimeCode::Default(), VtValue(5.0f)));
    CHECK(portable.SetTargets(SdfPath("/FK.rigExec:controls"), {b.GetPath(), a.GetPath()}));
    fk.GetRelationship(TfToken("rigExec:controls")).SetTargets({b.GetPath(), a.GetPath()});
    compare(UsdTimeCode::Default());
    CHECK(portable.SetConnections(SdfPath("/Blend.inputs:weight"), {SdfPath("/Driver.w2")}));
    blend.GetAttribute(TfToken("inputs:weight")).SetConnections({SdfPath("/Driver.w2")});
    compare(UsdTimeCode::Default());
    CHECK(portable.SetPrimActive(SdfPath("/B"), false));
    CHECK(!portable.Evaluate(UsdTimeCode::Default()).valid);
    CHECK(portable.SetPrimActive(SdfPath("/B"), true));
    compare(UsdTimeCode::Default());
    CHECK(!portable.Evaluate(UsdTimeCode(3.5)).valid);
    CHECK(!portable.EvaluateResolved(UsdTimeCode(3.5), {}).valid);
    std::map<SdfPath, VtValue> complete;
    for (const auto &[path, attribute] : db.attributes) complete[path] = attribute.resolved.at("default");
    complete[SdfPath("/B.rest:tx")] = VtValue(11.0);
    const auto firstOverride = portable.EvaluateResolved(UsdTimeCode(3.5), complete);
    CHECK(firstOverride.valid && firstOverride.Get<RigExecPointFrame>(1).Origin() == GfVec3d(11, 0, 0));
    complete[SdfPath("/B.rest:tx")] = VtValue(12.0);
    const auto secondOverride = portable.EvaluateResolved(UsdTimeCode(3.5), complete);
    CHECK(secondOverride.valid && secondOverride.Get<RigExecPointFrame>(1).Origin() == GfVec3d(12, 0, 0));
    CHECK(firstOverride.Get<RigExecPointFrame>(1).Origin() == GfVec3d(11, 0, 0));
    compare(UsdTimeCode::Default());
    reference.reset();
    const UsdStageWeakPtr weakStage(stage);
    stage.Reset();
    CHECK(!weakStage);
    CHECK(portable.Evaluate(UsdTimeCode::Default()).valid);
    CHECK(errors.IsClean());
    for (const auto &entry : errors) std::printf("USD error: %s\n", entry.GetCommentary().c_str());
    errors.Clear();
    std::printf("testRigExecStandalone: %d failure(s)\n", failures);
    return failures ? 1 : 0;
}
