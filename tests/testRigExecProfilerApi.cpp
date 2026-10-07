#include "rigExecImaging/profilerApi.h"
#include "pxr/base/js/json.h"
#include "pxr/base/plug/registry.h"
#include "pxr/usd/usd/primRange.h"
#include "pxr/usd/usdUtils/stageCache.h"
#include <cstdio>
#include <fstream>
#include <iterator>
#include <string>
#include <vector>

PXR_NAMESPACE_USING_DIRECTIVE
static int failures = 0;
#define CHECK(condition) do { if (!(condition)) { ++failures; \
    std::printf("FAIL %d: %s\n", __LINE__, #condition); } } while (false)

static std::string Contents(const SdfLayerHandle &layer) {
    std::string contents; layer->ExportToString(&contents); return contents;
}

int main(int argc, char **argv) {
    if (argc != 2) return 1;
    PlugRegistry::GetInstance().RegisterPlugins(RIGEXEC_SCHEMA_RESOURCE_DIR);
    auto source = UsdStage::Open(std::string(argv[1]) + "/01_FkChainTail.usda");
    CHECK(source);
    if (!source) return 1;
    // Remove animation only in a detached test layer, leaving the sample intact.
    source = UsdStage::Open(source->Flatten());
    for (const auto &prim : source->Traverse()) {
        for (const auto &attr : prim.GetAttributes())
            if (attr.GetName().GetString().find("avars:") == 0) attr.Clear();
    }
    const char *root = "/TailAsset/Rig";
    const char *control = "/TailAsset/Rig/Controls/Tail1";
    auto channel = source->GetPrimAtPath(SdfPath(control)).GetAttribute(TfToken("avars:rz"));
    channel.Set(2.0, UsdTimeCode(0));
    const auto sourceID = UsdUtilsStageCache::Get().Insert(source).ToLongInt();
    const auto snapshotID = RigExecIntrospect_CreateSnapshot(sourceID);
    CHECK(snapshotID > 0);
    auto snapshot = UsdUtilsStageCache::Get().Find(UsdStageCache::Id::FromLongInt(snapshotID));
    CHECK(snapshot && snapshot->GetRootLayer() != source->GetRootLayer());
    channel.Set(30.0, UsdTimeCode(0));
    double captured = 0;
    snapshot->GetPrimAtPath(SdfPath(control)).GetAttribute(TfToken("avars:rz")).Get(&captured, UsdTimeCode(0));
    CHECK(captured == 2.0);
    const auto rootBefore = Contents(source->GetRootLayer());
    const auto sessionBefore = Contents(source->GetSessionLayer());
    const auto snapshotBefore = Contents(snapshot->GetRootLayer());
    const int needed = RigExecIntrospect_ScheduleReportJson(snapshotID, root, 3, control, "avars:rz", nullptr, 0);
    CHECK(needed > 0);
    char tiny[2] = {'x', 'x'};
    CHECK(RigExecIntrospect_ScheduleReportJson(snapshotID, root, 3, control, "avars:rz", tiny, 2) == needed);
    CHECK(tiny[0] == 'x');
    std::vector<char> buffer(needed + 1);
    CHECK(RigExecIntrospect_ScheduleReportJson(snapshotID, root, 3, control, "avars:rz", buffer.data(), int(buffer.size())) == needed);
    const auto report = JsParseString(buffer.data()).GetJsObject();
    CHECK(report.at("ok").GetBool());
    CHECK(report.at("geometry_changed").GetBool());
    CHECK(report.at("driven").GetJsObject().at("control").GetString() == control);
    CHECK(report.at("compile").GetJsObject().at("ms").GetReal() > 0);
    for (const char *workload : {"replay", "drag", "author"}) {
        const auto cost = report.at("cost").GetJsObject().at(workload).GetJsObject();
        CHECK(cost.at("ms").GetJsObject().at("p50").GetReal() > 0);
        CHECK(!cost.at("scopes").GetJsObject().empty());
        CHECK(cost.at("threads").GetJsObject().at("distinct_threads").GetInt() > 0);
    }
    CHECK(Contents(source->GetRootLayer()) == rootBefore);
    CHECK(Contents(source->GetSessionLayer()) == sessionBefore);
    CHECK(Contents(snapshot->GetRootLayer()) == snapshotBefore);

    for (const auto &invalid : {std::pair<const char *, const char *>{"/Missing", "avars:rz"},
                               {root, "avars:missing"}}) {
        const int size = RigExecIntrospect_ScheduleReportJson(snapshotID, invalid.first, 3, control, invalid.second, nullptr, 0);
        std::vector<char> error(size + 1);
        RigExecIntrospect_ScheduleReportJson(snapshotID, invalid.first, 3, control, invalid.second, error.data(), int(error.size()));
        CHECK(!JsParseString(error.data()).GetJsObject().at("ok").GetBool());
    }
    const char *tracePath = "profiler-test.trace.json";
    CHECK(RigExecIntrospect_WriteProfileTrace(snapshotID, root, control, "avars:rz", tracePath) == 0);
    std::ifstream trace(tracePath);
    const std::string traceJSON((std::istreambuf_iterator<char>(trace)), std::istreambuf_iterator<char>());
    CHECK(!JsParseString(traceJSON).GetJsObject().at("traceEvents").GetJsArray().empty());
    std::remove(tracePath);
    CHECK(Contents(source->GetRootLayer()) == rootBefore);
    CHECK(Contents(snapshot->GetRootLayer()) == snapshotBefore);
    RigExecIntrospect_ReleaseSnapshot(sourceID); // Never release a host-owned stage.
    CHECK(UsdUtilsStageCache::Get().Find(UsdStageCache::Id::FromLongInt(sourceID)));
    RigExecIntrospect_ReleaseSnapshot(snapshotID);
    RigExecIntrospect_ReleaseSnapshot(snapshotID);
    CHECK(RigExecIntrospect_ScheduleReportJson(snapshotID, root, 3, control, "avars:rz", nullptr, 0) == -1);
    UsdUtilsStageCache::Get().Erase(UsdStageCache::Id::FromLongInt(sourceID));
    return failures ? 1 : 0;
}
