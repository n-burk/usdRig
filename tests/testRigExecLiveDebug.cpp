// Live inspection reads the host's evaluator without evaluating or authoring.
#include "rigExecImaging/registry.h"
#include "rigExecImaging/profilerApi.h"
#include "rigExec/bakedProgram.h"
#include "rigExec/bakedProgramImpl.h"
#include "pxr/base/js/json.h"
#include "pxr/base/plug/registry.h"
#include "pxr/base/tf/getenv.h"
#include "pxr/usd/usd/primRange.h"
#include "pxr/usd/usdUtils/stageCache.h"
#include <algorithm>
#include <cstdio>
#include <map>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

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
        // One [memo, publish] row per trace row, each the evaluator's own
        // phases for that op. The serial executor runs one op at a time, so
        // on each thread an op's memo starts after the previous op published.
        const auto &traceRows = data.at("trace").GetJsArray();
        const auto &phaseRows = data.at("trace_phases").GetJsArray();
        const auto *bridge = context->GetBridge(root);
        CHECK(bridge);
        const auto opTrace = bridge->GetEvaluator().GetLastOpTrace();
        CHECK(bridge->GetEvaluator().GetBakedProgram());
        const auto &steps =
            bridge->GetEvaluator().GetBakedProgram()->GetStepGraph().steps;
        CHECK(phaseRows.size() == traceRows.size());
        CHECK(opTrace.size() == traceRows.size());
        std::map<std::string, std::vector<std::pair<double, double>>> spans;
        for (size_t i = 0; i < phaseRows.size() && i < traceRows.size(); ++i) {
            const auto &phase = phaseRows[i].GetJsArray();
            const auto &event = traceRows[i].GetJsArray();
            CHECK(phase.size() == 2);
            CHECK(event[0].GetReal() == double(opTrace[i].step));
            // Live op timing alone stamps every op it times, memo start
            // through publication end.
            CHECK(opTrace[i].step < steps.size());
            CHECK(steps[opTrace[i].step].memoStartNs != 0 &&
                  steps[opTrace[i].step].publishEndNs != 0);
            CHECK(phase[0].GetReal() == double(opTrace[i].memoUs));
            CHECK(phase[1].GetReal() == double(opTrace[i].publishUs));
            spans[event[3].GetString()].emplace_back(
                event[1].GetReal() - phase[0].GetReal(),
                event[1].GetReal() + event[2].GetReal() + phase[1].GetReal());
        }
        if (TfGetenv("RIGEXEC_BAKED_SCHEDULE") == "serial") {
            for (auto &lane : spans) {
                std::sort(lane.second.begin(), lane.second.end());
                for (size_t i = 1; i < lane.second.size(); ++i)
                    CHECK(lane.second[i - 1].second <= lane.second[i].first);
            }
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
        CHECK(disabled.at("trace_phases").GetJsArray().size() ==
              disabled.at("trace").GetJsArray().size());
        for (const auto &row : disabled.at("trace_phases").GetJsArray())
            CHECK(row.GetJsArray()[0].GetReal() == 0 && row.GetJsArray()[1].GetReal() == 0);
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
