// Owner-thread snapshots of the existing imaging evaluator for live tools.
#include "profilerApi.h"
#include "rigExec/bakedProgram.h"
#include "rigExec/bakedProgramImpl.h"
#include "pxr/base/js/json.h"
#include "pxr/base/js/value.h"
#include "pxr/base/work/threadLimits.h"
#include <cstring>
#include <limits>

PXR_NAMESPACE_USING_DIRECTIVE
namespace rigExec {
namespace {
JsArray Indices(const std::vector<size_t> &values) {
    JsArray out;
    for (const auto value : values) out.emplace_back(double(value));
    return out;
}
JsArray Ranges(const std::vector<RigExecOpSlotRange> &values) {
    JsArray out;
    for (const auto &v : values)
        out.emplace_back(JsArray{JsValue(v.domain), JsValue(double(v.first)),
                                 JsValue(double(v.last))});
    return out;
}
}

bool RigExecImagingRegistry::SetLiveOpTiming(const SdfPath &rig, bool enabled)
{
    if (const Ptr routed = _Routed()) return routed->SetLiveOpTiming(rig, enabled);
    std::lock_guard<std::mutex> lock(_mutex);
    for (auto &session : _sessions) {
        if (session.rigPath == rig && session.bridge && !session.playback) {
            session.bridge->SetOpTimingEnabled(enabled);
            return true;
        }
    }
    return false;
}

std::string RigExecImagingRegistry::LiveDebugJson(
    const SdfPath &rig, const std::string &knownGraph,
    const std::string &knownGeneration)
{
    if (const Ptr routed = _Routed())
        return routed->LiveDebugJson(rig, knownGraph, knownGeneration);
    // The existing coordination boundary; no evaluator or worker runs here.
    std::lock_guard<std::mutex> lock(_mutex);
    JsObject reply{{"version", JsValue(1)}, {"active", JsValue(false)}};
    JsArray roots;
    for (const auto &session : _sessions) roots.emplace_back(session.rigPath.GetString());
    reply["rigs"] = JsValue(roots);
    for (auto &session : _sessions) {
        if (session.rigPath != rig || !session.bridge || session.playback) continue;
        const auto &evaluator = session.bridge->GetEvaluator();
        const auto *program = evaluator.GetBakedProgram();
        if (!program) continue;
        const auto epoch = evaluator.GetBindingEpochDigest();
        const auto builds = evaluator.GetBakedProgramBuildCount();
        if (session.debugGraphKey.empty() || session.debugProgram != program ||
            session.debugEpoch != epoch || session.debugBuildCount != builds) {
            session.debugProgram = program;
            session.debugEpoch = epoch;
            session.debugBuildCount = builds;
            session.debugGraphKey = std::to_string(_key) + ":" +
                                     std::to_string(++_nextDebugGraph);
        }
        const std::string generation = std::to_string(evaluator.GetBakedGenerationCount());
        const auto &body = program->GetStepGraph();
        reply["active"] = JsValue(true);
        reply["rig"] = JsValue(rig.GetString());
        reply["graph_key"] = JsValue(session.debugGraphKey);
        reply["generation"] = JsValue(generation);
        reply["recording"] = JsValue(evaluator.GetOpTimingEnabled());
        reply["time"] = body.lastTime.IsDefault() ? JsValue() : JsValue(body.lastTime.GetValue());
        reply["viewport_time"] = _lastTime.IsDefault() ? JsValue() : JsValue(_lastTime.GetValue());
        reply["evaluations"] = JsValue(double(session.evaluationCount));
        reply["concurrency_limit"] = JsValue(double(WorkGetConcurrencyLimit()));
        if (knownGraph != session.debugGraphKey) {
            JsArray graph;
            for (const auto &op : evaluator.GetOpGraph()) {
                graph.emplace_back(JsObject{{"step", JsValue(double(op.step))},
                    {"kind", JsValue(op.kind)}, {"domain", JsValue(op.domain)},
                    {"label", JsValue(op.label)}, {"cluster", JsValue(op.cluster)},
                    {"preds", JsValue(Indices(op.preds))},
                    {"succs", JsValue(Indices(op.succs))},
                    {"reads", JsValue(Ranges(op.reads))}, {"writes", JsValue(Ranges(op.writes))}});
            }
            reply["graph"] = JsValue(graph);
        }
        if (knownGraph != session.debugGraphKey || knownGeneration != generation) {
            JsArray trace;
            for (const auto &event : evaluator.GetLastOpTrace()) {
                // Labels and dependencies are held once in the graph.
                trace.emplace_back(JsArray{JsValue(double(event.step)),
                    JsValue(double(event.startUs)), JsValue(double(event.durationUs)),
                    JsValue(event.thread), JsValue(double(event.seq))});
            }
            reply["trace"] = JsValue(trace);
        }
        break;
    }
    return JsWriteToString(JsValue(reply));
}
}

extern "C" int RigExecImaging_SetLiveOpTimingForStage(
    long long id, const char *root, int enabled)
{
    try {
        const auto context = rigExec::RigExecImagingRegistry::ForStageCacheId(id, false);
        return context && root && context->SetLiveOpTiming(SdfPath(root), enabled != 0) ? 1 : 0;
    } catch (...) { return -1; }
}

extern "C" int RigExecImaging_LiveDebugJsonForStage(
    long long id, const char *root, const char *knownGraph,
    const char *knownGeneration, char *out, int cap)
{
    // Keep the size query and copy on one owner thread and one snapshot.
    struct Pending { long long id = 0; std::string root, graph, generation, json; };
    thread_local Pending pending;
    try {
        const auto context = rigExec::RigExecImagingRegistry::ForStageCacheId(id, false);
        if (!context || !root || cap < 0) { pending = {}; return -1; }
        const std::string graph = knownGraph ? knownGraph : "";
        const std::string generation = knownGeneration ? knownGeneration : "";
        if (!out || pending.json.empty() || pending.id != id || pending.root != root ||
            pending.graph != graph || pending.generation != generation) {
            pending = {id, root, graph, generation,
                       context->LiveDebugJson(*root ? SdfPath(root) : SdfPath(), graph, generation)};
        }
        if (pending.json.size() >= size_t(std::numeric_limits<int>::max())) return -1;
        const int needed = int(pending.json.size());
        if (out && cap > needed) {
            std::memcpy(out, pending.json.c_str(), size_t(needed) + 1);
            pending = {};
        }
        return needed;
    } catch (...) { pending = {}; return -1; }
}
