// Live inspection reads the host's evaluator without evaluating or authoring.
#include "rigExecImaging/registry.h"
#include "rigExecImaging/profilerApi.h"
#include "pxr/base/js/json.h"
#include "pxr/base/plug/registry.h"
#include "pxr/usd/usd/primRange.h"
#include "pxr/usd/usdUtils/stageCache.h"
#include <cstdio>
#include <stdexcept>

PXR_NAMESPACE_USING_DIRECTIVE
using namespace rigExec;
#define CHECK(x) do { if (!(x)) throw std::runtime_error(#x); } while (0)

int main(int argc, char **argv) {
    try {
        CHECK(argc == 3);
        PlugRegistry::GetInstance().RegisterPlugins(argv[2]);
        auto stage = UsdStage::Open(std::string(argv[1]) + "/ArmShotAnim.usda");
        CHECK(stage);
        SdfPath root;
        for (const auto &prim : stage->Traverse())
            if (prim.GetTypeName() == TfToken("RigExecRoot")) { root = prim.GetPath(); break; }
        CHECK(!root.IsEmpty());
        std::string before; stage->GetRootLayer()->ExportToString(&before);
        const auto id = UsdUtilsStageCache::Get().Insert(stage).ToLongInt();
        CHECK(RigExecImaging_LiveDebugJsonForStage(id, root.GetText(), "", "", nullptr, 0) == -1);
        auto context = RigExecImagingRegistry::ForStage(stage);
        CHECK(context->Activate(stage, root, UsdTimeCode(1001), nullptr));
        auto profiler = context->MutableBridgeProfiler(root);
        CHECK(profiler && !profiler->IsEnabled());
        const auto eventsBefore = profiler->GetEvents().size();
        CHECK(RigExecImaging_SetLiveOpTimingForStage(id, root.GetText(), 1) == 1);
        CHECK(context->SetTime(UsdTimeCode(1002)));
        const auto evaluations = context->GetSessionEvaluationCount(root);
        const int needed = RigExecImaging_LiveDebugJsonForStage(id, root.GetText(), "", "", nullptr, 0);
        CHECK(needed > 0);
        char small[1] = {'x'};
        CHECK(RigExecImaging_LiveDebugJsonForStage(id, root.GetText(), "", "", small, 1) == needed);
        CHECK(small[0] == 'x');
        std::vector<char> buffer(size_t(needed) + 1);
        CHECK(RigExecImaging_LiveDebugJsonForStage(id, root.GetText(), "", "", buffer.data(), int(buffer.size())) == needed);
        const auto data = JsParseString(buffer.data()).GetJsObject();
        CHECK(data.at("active").GetBool());
        CHECK(!data.at("graph").GetJsArray().empty());
        CHECK(!data.at("trace").GetJsArray().empty());
        for (const auto &row : data.at("trace").GetJsArray()) {
            const auto &event = row.GetJsArray();
            CHECK(event.size() == 5);
            CHECK(event[1].GetReal() > 0);
            CHECK(event[2].GetReal() >= 0);
            CHECK(!event[3].GetString().empty());
        }
        const auto held = JsParseString(context->LiveDebugJson(root,
            data.at("graph_key").GetString(), data.at("generation").GetString())).GetJsObject();
        CHECK(!held.count("graph") && !held.count("trace"));
        CHECK(context->GetSessionEvaluationCount(root) == evaluations);
        CHECK(profiler->GetEvents().size() == eventsBefore);
        CHECK(RigExecImaging_SetLiveOpTimingForStage(id, root.GetText(), 0) == 1);
        CHECK(context->SetTime(UsdTimeCode(1003)));
        const auto disabled = JsParseString(context->LiveDebugJson(root, "", "")).GetJsObject();
        for (const auto &row : disabled.at("trace").GetJsArray())
            CHECK(row.GetJsArray()[3].GetString().empty());
        std::string after; stage->GetRootLayer()->ExportToString(&after);
        CHECK(before == after);
        auto other = UsdStage::CreateInMemory();
        const auto otherId = UsdUtilsStageCache::Get().Insert(other).ToLongInt();
        CHECK(RigExecImaging_SetLiveOpTimingForStage(otherId, root.GetText(), 1) == 0);
        context->Deactivate();
        CHECK(!JsParseString(context->LiveDebugJson(root, "", "")).GetJsObject().at("active").GetBool());
        UsdUtilsStageCache::Get().Erase(stage);
        UsdUtilsStageCache::Get().Erase(other);
        std::puts("LIVE_DEBUG_OK");
        return 0;
    } catch (const std::exception &error) {
        std::fprintf(stderr, "LIVE_DEBUG_FAILED: %s\n", error.what());
        return 1;
    }
}
