#include "profilerApi.h"
#include "rigExec/rigEvaluator.h"
#include "rigExec/parallel.h"
#include "pxr/base/js/json.h"
#include "pxr/base/js/value.h"
#include "pxr/usd/usdUtils/stageCache.h"
#include "pxr/usd/usd/primRange.h"
#include <algorithm>
#include <chrono>
#include <cstdlib>
#include <cstring>
#include <limits>
#include <mutex>
#include <numeric>
#include <set>
#include <stdexcept>
#include <thread>

PXR_NAMESPACE_USING_DIRECTIVE
namespace {
using namespace rigExec;
using Clock = std::chrono::steady_clock;
std::mutex snapshotMutex;
std::set<long long> snapshots;
UsdStageRefPtr Stage(long long id) {
    return UsdUtilsStageCache::Get().Find(UsdStageCache::Id::FromLongInt(static_cast<long>(id)));
}
JsArray Strings(const std::vector<std::string> &values) {
    JsArray result;for(const auto &v:values)result.emplace_back(v);return result;
}
std::string Path(const char *p) {return p?p:"";}
double Milliseconds(Clock::time_point begin) {
    return std::chrono::duration<double,std::milli>(Clock::now()-begin).count();
}
UsdStageRefPtr PrivateStage(const UsdStageRefPtr &source) {
    auto session=SdfLayer::CreateAnonymous("profiler-session.usda");
    session->GetSubLayerPaths().push_back(source->GetSessionLayer()->GetIdentifier());
    auto stage=UsdStage::Open(source->GetRootLayer(),session,source->GetPathResolverContext());
    if(!stage)throw std::runtime_error("could not open measurement stage");
    stage->SetEditTarget(session);return stage;
}
void Check(const RigExecRigPose &pose) {
    if(!pose.valid)throw std::runtime_error(pose.diagnostics.empty()?"invalid evaluated pose":pose.diagnostics.front());
}
JsObject Percentiles(std::vector<double> times) {
    if(times.empty())return {};
    std::sort(times.begin(),times.end());
    return {{"best",JsValue(times.front())},{"p50",JsValue(times[times.size()/2])},
        {"p90",JsValue(times[std::min(times.size()-1,size_t(times.size()*0.9))])},
        {"mean",JsValue(std::accumulate(times.begin(),times.end(),0.0)/times.size())},
        {"worst",JsValue(times.back())}};
}
JsObject Threads(const RigExecProfiler &profiler) {
    std::map<uint64_t,uint64_t> totals;
    std::map<std::string,std::set<uint64_t>> categories;
    for(const auto &e:profiler.GetEvents()) {totals[e.threadIndex]+=e.durationUs;categories[e.category].insert(e.threadIndex);}
    JsObject ms,byCategory;
    for(const auto &[thread,us]:totals)ms[std::to_string(thread)]=JsValue(double(us)/1000);
    for(const auto &[category,threads]:categories) {
        JsArray ids;for(auto id:threads)ids.emplace_back(int(id));byCategory[category]=JsValue(ids);
    }
    return {{"distinct_threads",JsValue(int(totals.size()))},{"ms_per_thread",JsValue(ms)},
        {"threads_per_category",JsValue(byCategory)}};
}
JsObject Scopes(const RigExecProfiler &profiler,bool compile=false) {
    JsObject result;
    for(const auto &row:profiler.Summarize()) {
        if(compile)result[row.name]=JsValue(double(row.totalUs)/1000);
        else result[row.name]=JsValue(JsObject{{"count",JsValue(int(row.count))},
            {"ms_total",JsValue(double(row.totalUs)/1000)},
            {"ms_per_call",JsValue(double(row.totalUs)/1000/std::max(size_t(1),row.count))}});
    }
    return result;
}
UsdAttribute Drive(const UsdStageRefPtr &stage,const SdfPath &root,const char *control,const char *avar) {
    SdfPath path(Path(control));
    if(path.IsEmpty()) {
        for(const auto &prim:UsdPrimRange(stage->GetPrimAtPath(root))) {
            if(prim.GetTypeName()!=TfToken("RigExecControl"))continue;
            if(path.IsEmpty() || prim.GetName()==TfToken("M_Body"))path=prim.GetPath();
        }
    }
    if(!path.IsAbsolutePath() || !path.IsPrimPath() || !path.HasPrefix(root))throw std::runtime_error("choose a control under the selected rig");
    auto prim=stage->GetPrimAtPath(path);
    if(!prim || prim.GetTypeName()!=TfToken("RigExecControl"))throw std::runtime_error("selected prim is not a rig control");
    auto attr=prim.GetAttribute(TfToken(avar?avar:"avars:ry"));
    if(!attr || (attr.GetTypeName()!=SdfValueTypeNames->Double && attr.GetTypeName()!=SdfValueTypeNames->Float))
        throw std::runtime_error("choose an existing float or double control channel");
    if(attr.HasAuthoredConnections())throw std::runtime_error("selected channel is driven; choose an editable control channel");
    return attr;
}
VtValue Value(const UsdAttribute &attr,double number) {
    return attr.GetTypeName()==SdfValueTypeNames->Float?VtValue(float(number)):VtValue(number);
}
JsObject Report(const UsdStageRefPtr &source,const char *rootPath,int samples,const char *control,const char *avar) {
    if(samples<1 || samples>1000)throw std::runtime_error("samples must be between 1 and 1000");
    auto stage=PrivateStage(source);const SdfPath root(Path(rootPath));
    if(!root.IsAbsolutePath() || !stage->GetPrimAtPath(root) || stage->GetPrimAtPath(root).GetTypeName()!=TfToken("RigExecRoot"))
        throw std::runtime_error("selected rig root does not exist");
    auto attr=Drive(stage,root,control,avar);double original=0;
    if(attr.GetTypeName()==SdfValueTypeNames->Float) {float f=0;attr.Get(&f,UsdTimeCode(0));original=f;}
    else attr.Get(&original,UsdTimeCode(0));
    RigExecRigEvaluator evaluator(stage,root);evaluator.SetPublishWeightFields(false);
    evaluator.SetProfilingEnabled(true);auto begin=Clock::now();std::vector<std::string> errors;
    if(!evaluator.Compile(&errors))throw std::runtime_error(errors.empty()?"rig compilation failed":errors.front());
    JsObject compile{{"ms",JsValue(Milliseconds(begin))},{"scopes",JsValue(Scopes(evaluator.GetProfiler(),true))},
        {"threads",JsValue(Threads(evaluator.GetProfiler()))}};
    evaluator.SetProfilingEnabled(false);
    auto first=evaluator.Evaluate(UsdTimeCode(0));Check(first);
    JsObject levels;std::map<size_t,JsArray> grouped;size_t deepest=0,widest=0;
    for(const auto &[path,level]:evaluator.GetSolverBatchLevels()) {grouped[level].emplace_back(path.GetName());deepest=std::max(deepest,level);}
    for(const auto &[level,names]:grouped) {levels[std::to_string(level)]=JsValue(names);widest=std::max(widest,names.size());}
    int constraints=0,math=0,chains=0;JsArray chainLevels;
    for(const auto &m:evaluator.GetMoverOrder()) {
        auto type=m.schemaType.GetString();if(type.size()>=10 && type.substr(type.size()-10)=="Constraint")++constraints;
        if(type=="RigExecFloatMathMover")++math;
    }
    for(size_t level=0;level<evaluator.GetChainLevelCount();++level) {
        JsArray targets;for(const auto &p:evaluator.GetChainLevelTargets(level))targets.emplace_back(p.GetString());
        chains+=int(targets.size());const bool parallel=evaluator.IsChainLevelParallel(level);
        chainLevels.emplace_back(JsObject{{"targets",JsValue(targets)},{"parallel",JsValue(parallel)},
            {"why_not",JsValue(parallel?"":targets.size()<3?"fewer than 3 chains in the level":"chains share inputs or are otherwise not independent")}});
    }
    std::vector<std::string> reasons;const bool bakeable=evaluator.IsBakeable(&reasons);
    const char *modes[]={"dynamic","baked","parity","reference"};
    const char *sources[]={"default","attribute","environment","api"};
    std::string sourceName=source->GetRootLayer()->GetIdentifier();
    auto metadata=source->GetRootLayer()->GetCustomLayerData();auto it=metadata.find("rigExecProfilerSourcePath");
    if(it!=metadata.end() && it->second.IsHolding<std::string>())sourceName=it->second.Get<std::string>();
    JsObject result{{"ok",JsValue(true)},{"samples",JsValue(samples)},
        {"measured_through",JsValue("native evaluator snapshot; viewport rendering excluded")},
        {"stage",JsValue(JsObject{{"path",JsValue(sourceName)},{"rig_root",JsValue(root.GetString())},
            {"mode",JsValue(modes[int(evaluator.GetEvaluationMode())])},
            {"mode_source",JsValue(sources[int(evaluator.GetEvaluationModeSource())])},
            {"bakeable",JsValue(bakeable)},{"bakeability_reasons",JsValue(Strings(reasons))},
            {"parallel_enabled",JsValue(RigExecParallelEvaluationEnabled())},
            {"cores",JsValue(int(std::thread::hardware_concurrency()))},{"thread_limit",JsValue(std::string(std::getenv("PXR_WORK_THREAD_LIMIT")?std::getenv("PXR_WORK_THREAD_LIMIT"):""))}})},
        {"compile",JsValue(compile)},{"chains",JsValue(chainLevels)},
        {"shape",JsValue(JsObject{{"solver_count",JsValue(int(evaluator.GetSolverBatchLevels().size()))},
            {"deepest_solver_level",JsValue(int(deepest))},{"widest_solver_level",JsValue(int(widest))},
            {"solver_levels",JsValue(levels)},{"frame_constraints",JsValue(constraints)},
            {"float_math_movers",JsValue(math)},{"mover_chains",JsValue(chains)}})},
        {"driven",JsValue(JsObject{{"control",JsValue(attr.GetPrimPath().GetString())},{"avar",JsValue(attr.GetName().GetString())}})}};
    JsObject costs;RigExecRigPose last=first;
    for(const char *workload:{"replay","drag","author"}) {
        evaluator.SetProfilingEnabled(false);evaluator.ClearInteractiveOverrides();
        auto step=[&](int index) {
            const double number=original+0.1*(index+1);
            if(std::strcmp(workload,"drag")==0)evaluator.SetInteractiveOverrides({{attr.GetPrimPath(),TfToken(),attr.GetName(),Value(attr,number)}});
            else if(std::strcmp(workload,"author")==0 && !attr.Set(Value(attr,number),UsdTimeCode(0)))throw std::runtime_error("could not author measurement channel");
            last=evaluator.Evaluate(UsdTimeCode(0));Check(last);
        };
        for(int i=0;i<5;++i)step(i);
        evaluator.ClearProfile();evaluator.SetProfilingEnabled(true);std::vector<double> times;
        for(int i=0;i<samples;++i) {begin=Clock::now();step(i);times.push_back(Milliseconds(begin));}
        costs[workload]=JsValue(JsObject{{"ms",JsValue(Percentiles(times))},{"scopes",JsValue(Scopes(evaluator.GetProfiler()))},
            {"threads",JsValue(Threads(evaluator.GetProfiler()))}});
    }
    evaluator.SetProfilingEnabled(false);evaluator.ClearInteractiveOverrides();
    bool geometryChanged=false;
    for(const auto &[path,value]:last.movedProperties)if(path.GetNameToken()==TfToken("points")) {
        const auto found=first.movedProperties.find(path);
        if(found!=first.movedProperties.end() && value!=found->second)geometryChanged=true;
    }
    result["geometry_changed"]=JsValue(geometryChanged);result["cost"]=JsValue(costs);return result;
}
JsObject Failure(const std::exception &error) {return {{"ok",JsValue(false)},{"errors",JsValue(JsArray{JsValue(error.what())})}};}
int Copy(const std::string &json,char *out,int cap) {
    if(json.size()>size_t(std::numeric_limits<int>::max()))return -1;
    if(out && cap>int(json.size()))std::memcpy(out,json.c_str(),json.size()+1);
    return int(json.size());
}
}

extern "C" long long RigExecIntrospect_CreateSnapshot(long long id) {
    try {
        auto source=Stage(id);if(!source)return -1;
        auto layer=source->Flatten();if(!layer)return -1;
        auto metadata=layer->GetCustomLayerData();metadata["rigExecProfilerSourcePath"]=VtValue(source->GetRootLayer()->GetIdentifier());layer->SetCustomLayerData(metadata);
        auto stage=UsdStage::Open(layer);if(!stage)return -1;
        const long long snapshot=UsdUtilsStageCache::Get().Insert(stage).ToLongInt();
        std::lock_guard<std::mutex> lock(snapshotMutex);snapshots.insert(snapshot);return snapshot;
    } catch(...) {return -1;}
}
extern "C" void RigExecIntrospect_ReleaseSnapshot(long long id) {
    std::lock_guard<std::mutex> lock(snapshotMutex);
    if(snapshots.erase(id))UsdUtilsStageCache::Get().Erase(UsdStageCache::Id::FromLongInt(static_cast<long>(id)));
}
extern "C" int RigExecIntrospect_ScheduleReportJson(long long id,const char *root,int samples,const char *control,const char *avar,char *out,int cap) {
    const auto source=Stage(id);if(!source)return -1;
    struct Pending {long long id=0;int samples=0;std::string root,control,avar,json;};
    thread_local Pending pending;
    if(pending.json.empty() || pending.id!=id || pending.samples!=samples || pending.root!=Path(root) || pending.control!=Path(control) || pending.avar!=Path(avar)) {
        JsObject report;try {report=Report(source,root,samples,control,avar);}catch(const std::exception &error) {report=Failure(error);}
        pending={id,samples,Path(root),Path(control),Path(avar),JsWriteToString(JsValue(report))};
    }
    int needed=Copy(pending.json,out,cap);if(out && cap>needed)pending.json.clear();return needed;
}
extern "C" int RigExecIntrospect_MoverOrderJson(long long id,const char *root,char *out,int cap) {
    auto source=Stage(id);if(!source)return -1;JsObject report;
    try {
        auto stage=PrivateStage(source);RigExecRigEvaluator evaluator(stage,SdfPath(Path(root)));std::vector<std::string> errors;
        const bool ok=evaluator.Compile(&errors);JsArray movers;
        for(const auto &m:evaluator.GetMoverOrder()) {
            JsArray targets;for(const auto &p:m.targets)targets.emplace_back(p.GetString());
            movers.emplace_back(JsObject{{"path",JsValue(m.moverPath.GetString())},{"type",JsValue(m.schemaType.GetString())},
                {"targets",JsValue(targets)},{"ordinal",JsValue(m.ordinal)}});
        }
        report={{"ok",JsValue(ok)},{"movers",JsValue(movers)},{"errors",JsValue(Strings(errors))}};
    } catch(const std::exception &error) {report=Failure(error);}
    return Copy(JsWriteToString(JsValue(report)),out,cap);
}
extern "C" int RigExecIntrospect_WriteProfileTrace(long long id,const char *root,const char *control,const char *avar,const char *path) {
    try {
        auto source=Stage(id);if(!source || !path)return -1;auto stage=PrivateStage(source);
        auto attr=Drive(stage,SdfPath(Path(root)),control,avar);RigExecRigEvaluator evaluator(stage,SdfPath(Path(root)));
        evaluator.SetPublishWeightFields(false);
        double original=0;
        if(attr.GetTypeName()==SdfValueTypeNames->Float) {float f=0;attr.Get(&f,UsdTimeCode(0));original=f;}
        else attr.Get(&original,UsdTimeCode(0));
        if(!evaluator.Compile())return -1;Check(evaluator.Evaluate(UsdTimeCode(0)));
        evaluator.SetProfilingEnabled(true);evaluator.ClearProfile();
        evaluator.SetInteractiveOverrides({{attr.GetPrimPath(),TfToken(),attr.GetName(),Value(attr,original+12.0)}});
        Check(evaluator.Evaluate(UsdTimeCode(0)));return evaluator.WriteProfileTrace(path)?0:-1;
    } catch(...) {return -1;}
}
