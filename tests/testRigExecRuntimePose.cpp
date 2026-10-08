// rigExecRuntime pose-family parity (M2): baked program vs runtime over
// every baking fixture, comparing fin/base versions, rest -> pose
// matrices and joint matrices bit for bit. Each stage is baked at one time
// and the binary is played through its inputs: the input sampler hands it
// the stage's animated inputs at each frame, and a drag is an input set to
// an authored value, held to the evaluator with the same value authored in
// the session layer.
#include "rigExecBake/bake.h"
#include "rigExec/bakedProgramImpl.h"
#include "rigExec/rigEvaluator.h"
#include "rigExec/weightReference.h"
#include "rigExecBinary/format.h"
#include "rigExecMath/propertyMath.h"
#include "rigExecRuntime/runtime.h"
#include "rigExecRuntime/poseInternal.h"
#include "rigExecExampleFixtures.h"

#include "pxr/base/gf/rotation.h"
#include "pxr/base/plug/registry.h"
#include "pxr/base/tf/getenv.h"
#include "pxr/usd/sdf/types.h"
#include "pxr/usd/usd/primRange.h"
#include "pxr/usd/usd/stage.h"
#include "pxr/usd/usdGeom/xform.h"

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdio>
#include <cstring>
#include <filesystem>
#include <functional>
#include <limits>
#include <map>
#include <memory>
#include <string>
#include <utility>
#include <vector>

using namespace rigExec;
PXR_NAMESPACE_USING_DIRECTIVE

static int failures = 0;
static int comparedFixtures = 0;
static int comparedFrames = 0;
static bool checkedRefreshRecords=false,checkedRefreshCarries=false,checkedRefreshGuards=false;

#define CHECK(cond)                                                     \
    do {                                                                \
        if (!(cond)) {                                                  \
            ++failures;                                                 \
            std::printf("FAIL %s:%d: %s\n", __FILE__, __LINE__, #cond);  \
        }                                                               \
    } while (0)

#include "rigExecFileEdit.h"
#include "rigExecRuntimeDrive.h"

static SdfPath
_FindRig(const UsdStageRefPtr &stage)
{
    for (const UsdPrim &prim : stage->TraverseAll()) {
        if (prim.GetTypeName() == TfToken("RigExecRoot")) {
            return prim.GetPath();
        }
    }
    return SdfPath();
}

static std::vector<double>
_ParseFrames(const std::string &text)
{
    std::vector<double> frames;
    size_t begin = 0;
    while (begin <= text.size()) {
        const size_t end = text.find(",", begin);
        const std::string piece =
            text.substr(begin, end == std::string::npos
                        ? std::string::npos : end - begin);
        if (!piece.empty()) {
            frames.push_back(std::stod(piece));
        }
        if (end == std::string::npos) {
            break;
        }
        begin = end + 1;
    }
    return frames;
}

static bool
_FramesEqual(const RigExecPointFrame &a, const RrPointFrame &b)
{
    if (a.flags != b.flags) {
        return false;
    }
    for (size_t i = 0; i < 4; ++i) {
        for (size_t c = 0; c < 3; ++c) {
            if (a.points[i][c] != b.points[i][c]) {
                return false;
            }
        }
    }
    return true;
}

static bool
_MatricesEqual(const GfMatrix4d &a, const RrMat4d &b)
{
    for (size_t r = 0; r < 4; ++r) {
        const GfVec4d row = a.GetRow(int(r));
        for (size_t c = 0; c < 4; ++c) {
            if (row[c] != b[r][c]) {
                return false;
            }
        }
    }
    return true;
}

static bool
_SameBits(const RrPropertyValue &a, const RrPropertyValue &b)
{
    using Tag = RrPropertyValue::Tag;
    if (a.tag != b.tag) {
        return false;
    }
    switch (a.tag) {
    case Tag::Float:
        return std::memcmp(&a.f32, &b.f32, sizeof(float)) == 0;
    case Tag::Double:
        return std::memcmp(&a.f64, &b.f64, sizeof(double)) == 0;
    case Tag::Matrix4d:
        for (size_t r = 0; r < 4; ++r) {
            if (std::memcmp(a.matrix[r], b.matrix[r], sizeof(double) * 4) !=
                0) {
                return false;
            }
        }
        return true;
    case Tag::Vec3f:
        return std::memcmp(a.vec.data(), b.vec.data(), sizeof(float) * 3) ==
               0;
    }
    return false;
}

// One played frame of a file: what a consumer reads, plus the property
// results.
struct _PlayedFrame {
    std::vector<RigExecRuntimePropertyValue> properties;
    std::vector<RrPointFrame> fin;
    std::vector<RigExecRuntimeProviderXform> xforms;
    std::vector<std::string> diagnostics;
};

static _PlayedFrame
_Snapshot(const RigExecRuntimeReader &reader)
{
    return {reader.GetPropertyValues(), reader.GetFinFrames(),
            reader.GetProviderXforms(), reader.GetDiagnostics()};
}

// Whether two played frames publish the same values bit for bit: property
// results, version pool and provider transforms, and with \p diagnostics
// the generation's lines too.
static bool
_SamePlayed(const _PlayedFrame &a, const _PlayedFrame &b,
            bool diagnostics = true)
{
    if (a.properties.size() != b.properties.size() || a.fin != b.fin ||
        (diagnostics && a.diagnostics != b.diagnostics) ||
        a.xforms.size() != b.xforms.size()) {
        return false;
    }
    for (size_t i = 0; i < a.properties.size(); ++i) {
        if (a.properties[i].path != b.properties[i].path ||
            !_SameBits(a.properties[i].value, b.properties[i].value)) {
            return false;
        }
    }
    for (size_t i = 0; i < a.xforms.size(); ++i) {
        if (a.xforms[i].path != b.xforms[i].path ||
            a.xforms[i].matrix != b.xforms[i].matrix ||
            a.xforms[i].base != b.xforms[i].base) {
            return false;
        }
    }
    return true;
}

// The file plays a value computed from its inputs, not one it holds: its
// outputs at the first and the last frame differ, while it holds the bake
// time's static data alone.
static void
_CheckFramesDiffer(const char *name, const std::vector<_PlayedFrame> &rows)
{
    const bool differ =
        rows.size() > 1 && !_SamePlayed(rows.front(), rows.back(), false);
    CHECK(differ);
    if (!differ) {
        std::printf("%s: the first and last frames play the same outputs\n",
                    name);
    }
}

// Bakes \p stage at the first of \p frames and plays the file through the
// input sampler beside a fresh baked evaluator, frame by frame: the step
// graph's version pools and rest -> pose matrices, the joint matrices and
// the diagnostics, bit for bit. A `static` stage holds an animated source
// in static data at the bake time, so it plays that time alone. \p played,
// when given, receives each frame's outputs.
static bool checkedSolverSemanticRequirements=false;
static void _TestSolverSemanticRefusals(const std::vector<uint8_t> &bytes)
{
    if(checkedSolverSemanticRequirements)return;
    auto baseline=RigExecTestUnpack(bytes);if(!baseline)return;
    size_t owner=baseline->steps.size();
    for(size_t i=0;i<baseline->steps.size();++i)
        if(baseline->steps[i].kind==fb::StepKind::Solve && !baseline->steps[i].semanticPredecessorKeys.empty()){owner=i;break;}
    if(owner==baseline->steps.size())return;
    CHECK(!baseline->pose->solvers[size_t(baseline->steps[owner].object)].relationshipRequirements.empty());
    std::vector<uint8_t> valid;std::string error;
    CHECK(RigExecFormatWrite(*baseline,&valid,&error));
    std::unique_ptr<fb::RigExecWireFile> opened;
    CHECK(RigExecFormatOpen(valid.data(),valid.size(),&opened,&error));
    const auto reject=[&](fb::RigExecWireFile bad,const char *reason){
        std::vector<uint8_t> rejected;std::string writeError,openError;
        CHECK(!RigExecFormatWrite(bad,&rejected,&writeError));
        const auto malformed=RigExecTestPackUnchecked(bad);
        CHECK(!RigExecFormatOpen(malformed.data(),malformed.size(),&opened,&openError));
        CHECK(writeError.find(reason)!=std::string::npos);CHECK(openError.find(reason)!=std::string::npos);
    };
    auto missing=*baseline;missing.steps[owner].semanticPredecessorKeys.clear();
    reject(std::move(missing),"semantic prerequisites differ from supported solver relationships");
    size_t after=baseline->commonGraph->ops.size();
    for(size_t i=0;i<baseline->commonGraph->ops.size();++i)
        if(baseline->commonGraph->ops[i].key==baseline->steps[owner].descriptorKey)after=i;
    CHECK(after<baseline->commonGraph->ops.size());
    if(after>=baseline->commonGraph->ops.size())return;
    auto omitted=*baseline;
    const auto requiredKey=omitted.steps[owner].semanticPredecessorKeys.front();
    size_t requiredOp=omitted.commonGraph->ops.size(),requiredBody=omitted.steps.size();
    for(size_t i=0;i<omitted.commonGraph->ops.size();++i)if(omitted.commonGraph->ops[i].key==requiredKey)requiredOp=i;
    for(size_t i=0;i<omitted.steps.size();++i)if(omitted.steps[i].descriptorKey==requiredKey)requiredBody=i;
    CHECK(requiredOp<omitted.commonGraph->ops.size() && requiredBody<omitted.steps.size());
    if(requiredOp>=omitted.commonGraph->ops.size() || requiredBody>=omitted.steps.size())return;
    const auto erase=[](auto &values,auto value){values.erase(std::remove(values.begin(),values.end(),value),values.end());};
    erase(omitted.steps[owner].preds,int32_t(requiredBody));erase(omitted.steps[requiredBody].succs,int32_t(owner));
    erase(omitted.commonGraph->ops[after].predecessors,uint32_t(requiredOp));erase(omitted.commonGraph->ops[requiredOp].successors,uint32_t(after));
    reject(std::move(omitted),"generated graph omitted semantic solver prerequisite");
    size_t before=baseline->commonGraph->ops.size();
    for(size_t i=0;i<after;++i) {
        size_t candidateBody=baseline->steps.size();
        for(size_t b=0;b<baseline->steps.size();++b)if(baseline->steps[b].descriptorKey==baseline->commonGraph->ops[i].key)candidateBody=b;
        if(candidateBody>=owner)continue;
        const auto from=baseline->clustering->clusterOf[candidateBody],to=baseline->clustering->clusterOf[owner];
        // The extra edge must preserve cluster order; it need not exist yet.
        // The mutation below inserts both reciprocal cluster endpoints.
        if(from>to)continue;
        if(std::find(baseline->commonGraph->ops[after].predecessors.begin(),baseline->commonGraph->ops[after].predecessors.end(),uint32_t(i))==baseline->commonGraph->ops[after].predecessors.end()){before=i;break;}
    }
    CHECK(before<after);if(before>=after)return;
    size_t beforeBody=baseline->steps.size();
    for(size_t i=0;i<baseline->steps.size();++i)
        if(baseline->steps[i].descriptorKey==baseline->commonGraph->ops[before].key)beforeBody=i;
    CHECK(beforeBody<baseline->steps.size());if(beforeBody>=baseline->steps.size())return;
    auto extra=*baseline;
    extra.steps[owner].preds.push_back(int32_t(beforeBody));std::sort(extra.steps[owner].preds.begin(),extra.steps[owner].preds.end());
    extra.steps[beforeBody].succs.push_back(int32_t(owner));std::sort(extra.steps[beforeBody].succs.begin(),extra.steps[beforeBody].succs.end());
    const auto fromCluster=extra.clustering->clusterOf[beforeBody],toCluster=extra.clustering->clusterOf[owner];
    if(fromCluster!=toCluster) {
        auto &preds=extra.clustering->clusters[size_t(toCluster)].preds;
        auto &succs=extra.clustering->clusters[size_t(fromCluster)].succs;
        if(std::find(preds.begin(),preds.end(),fromCluster)==preds.end())preds.push_back(fromCluster);
        if(std::find(succs.begin(),succs.end(),toCluster)==succs.end())succs.push_back(toCluster);
        std::sort(preds.begin(),preds.end());std::sort(succs.begin(),succs.end());
    }
    extra.commonGraph->ops[after].predecessors.push_back(uint32_t(before));
    std::sort(extra.commonGraph->ops[after].predecessors.begin(),extra.commonGraph->ops[after].predecessors.end());
    extra.commonGraph->ops[before].successors.push_back(uint32_t(after));
    std::sort(extra.commonGraph->ops[before].successors.begin(),extra.commonGraph->ops[before].successors.end());
    reject(std::move(extra),"predecessors differ from typed and semantic producers");
    auto imported=*baseline;imported.commonGraph->ops[after].descriptorPredecessors.push_back(uint32_t(before));
    reject(std::move(imported),"imported predecessor is not a typed or semantic requirement");
    checkedSolverSemanticRequirements=true;
}
static void _TestProviderRefreshRefusals(const std::vector<uint8_t> &bytes)
{
    auto baseline=RigExecTestUnpack(bytes);
    if(!baseline || !baseline->pose || baseline->pose->providerRefreshes.empty())return;
    const auto expect=[&](const char *name,const std::function<void(fb::RigExecWireFile&)> &edit,const char *reason) {
        auto file=RigExecTestUnpack(bytes);if(!file)return;
        edit(*file);std::vector<uint8_t> rejected;std::string writeError,openError;
        CHECK(!RigExecFormatWrite(*file,&rejected,&writeError));
        const auto malformed=RigExecTestPackUnchecked(*file);
        std::unique_ptr<fb::RigExecWireFile> opened;
        CHECK(!RigExecFormatOpen(malformed.data(),malformed.size(),&opened,&openError));
        if(writeError.find(reason)==std::string::npos || openError.find(reason)==std::string::npos) {
            std::printf("ProviderRefresh %s: write '%s', open '%s'; wanted '%s'\n",name,writeError.c_str(),openError.c_str(),reason);CHECK(false);
        }
    };
    if(!checkedRefreshRecords) {
        std::vector<uint8_t> valid;std::string error;CHECK(RigExecFormatWrite(*baseline,&valid,&error));
        expect("checkpoint",[](auto &f){f.pose->providerRefreshes[0].checkpoint=f.pose->walkSteps.size()+1;},"invalid or duplicate provider refresh context");
        expect("canonical key",[](auto &f){f.pose->providerRefreshes[0].key+="wrong";},"key or reader differs from canonical context");
        expect("entering bound",[](auto &f){f.pose->providerRefreshes[0].finRead=UINT32_MAX;},"entering frame belongs to another provider");
        expect("fresh write",[](auto &f){auto &r=f.pose->providerRefreshes[0];r.baseWrite=r.baseRead;},"lacks fresh owned frame versions");
        expect("body read",[](auto &f){const auto value=f.pose->providerRefreshes[0].baseRead;for(auto &s:f.steps)if(s.kind==fb::StepKind::ProviderRefresh && s.object==0)s.reads.erase(std::remove_if(s.reads.begin(),s.reads.end(),[&](const auto &r){return r.domain()==fb::SlotDomain::PoseBase && r.begin()<=value && value<r.end();}),s.reads.end());},"body SSA missing read PoseBase");
        expect("body write",[](auto &f){for(auto &s:f.steps)if(s.kind==fb::StepKind::ProviderRefresh && s.object==0)s.writes.clear();},"refresh write ownership differs");
        checkedRefreshRecords=true;
    }
    for(size_t i=0;i<baseline->pose->providerRefreshes.size();++i) {
        const auto &record=baseline->pose->providerRefreshes[i];
        if(!checkedRefreshCarries && !record.carries.empty()) {
            expect("carry target",[i](auto &f){f.pose->providerRefreshes[i].carries[0].slot=-1;},"invalid or duplicate refresh carry");
            expect("carry blocker",[i](auto &f){f.pose->providerRefreshes[i].carries[0].blockingSlots.push_back(-1);},"invalid or duplicate carry blocker");
            expect("carry ancestry",[i](auto &f){f.pose->providerRefreshes[i].carries[0].blockingSlots.clear();},"carry blocker ancestry differs");
            const auto &blockers=record.carries[0].blockingSlots;
            CHECK(!blockers.empty());
            if(!blockers.empty()) {
                const std::string path=RigExecFormatPathText(*baseline,baseline->slotMeta->paths[size_t(blockers[0])])+".parent:space";
                size_t rawRow=baseline->providerProgram->sampled.size();
                for(size_t row=0;row<baseline->providerProgram->sampled.size();++row)
                    if(baseline->providerProgram->sampled[row].path==path)rawRow=row;
                CHECK(rawRow<baseline->providerProgram->sampled.size());
                if(rawRow<baseline->providerProgram->sampled.size()) {
                    expect("blocker raw declaration",[i,rawRow](auto &f){for(auto &s:f.steps)if(s.kind==fb::StepKind::ProviderRefresh && s.object==int(i))s.reads.erase(std::remove_if(s.reads.begin(),s.reads.end(),[&](const auto &r){return r.domain()==fb::SlotDomain::SpaceLeaf && r.begin()<=rawRow && rawRow<r.end();}),s.reads.end());},"body SSA missing read SpaceLeaf");
                    expect("blocker raw identity",[rawRow](auto &f){f.providerProgram->sampled[rawRow].inputSlot=-1;},"blocker raw source type or identity differs");
                    expect("blocker produced promotion",[rawRow](auto &f){f.providerProgram->sampled[rawRow].propertyVersion=0;},"raw provider source cannot select a property version");
                }
            }
            checkedRefreshCarries=true;
        }
        if(!checkedRefreshGuards && !record.priorConstraints.empty()) {
            expect("guard commit",[i](auto &f){f.pose->providerRefreshes[i].priorConstraints[0].first=UINT32_MAX;},"invalid or duplicate prior constraint guard");
            expect("guard candidate",[i](auto &f){f.pose->providerRefreshes[i].priorConstraints[0].second=UINT32_MAX;},"invalid or duplicate prior constraint guard");
            checkedRefreshGuards=true;
        }
    }
}
static void
_TestStage(const std::string &name, const UsdStageRefPtr &stage,
           const std::vector<double> &frames,
           std::vector<_PlayedFrame> *played = nullptr,
           bool staticClass = false)
{
    const SdfPath rigPath = _FindRig(stage);
    CHECK(!rigPath.IsEmpty() && !frames.empty());
    if (rigPath.IsEmpty() || frames.empty()) {
        std::printf("%s: FAILED (no rig or no frames)\n", name.c_str());
        return;
    }
    const std::vector<double> times =
        staticClass ? std::vector<double>{frames.front()} : frames;
    std::vector<uint8_t> bytes;
    std::string error;
    {
        RigExecRigEvaluator evaluator(stage, rigPath);
        if (!RigExecTestBakeAt(evaluator, frames.front(), &bytes, &error)) {
            std::printf("%s: FAILED (bake: %s)\n", name.c_str(),
                        error.c_str());
            CHECK(false);
            return;
        }
    }
    RigExecTestPlayer player;
    _TestProviderRefreshRefusals(bytes);
    _TestSolverSemanticRefusals(bytes);
    if (!player.Open(bytes, stage, &error)) {
        std::printf("%s: FAILED (open: %s)\n", name.c_str(), error.c_str());
        CHECK(false);
        return;
    }
    // A fresh evaluator beside a fresh reader: both start at the bake
    // time's generation, the compile notices in it.
    RigExecRigEvaluator measured(stage, rigPath);

    bool failed = false;
    int compared = 0;
    for (double frame : times) {
        const RigExecRigPose pose =
            measured.Evaluate(UsdTimeCode(frame));
        if (!pose.valid) {
            std::printf("%s frame %.17g: baked pose invalid\n",
                        name.c_str(), frame);
            CHECK(false);
            failed = true;
            break;
        }
        const RigExecBakedProgram *program =
            measured.GetBakedProgram();
        if (!program) {
            std::printf("%s frame %.17g: no baked program\n",
                        name.c_str(), frame);
            CHECK(false);
            failed = true;
            break;
        }
        const RigExecBakedProgramImpl &graph = program->GetStepGraph();
        if (!player.Play(frame, &error)) {
            std::printf("execute diagnostic at %s frame %.17g: %s\n",
                        name.c_str(), frame, error.c_str());
            CHECK(false);
            failed = true;
            break;
        }
        const RigExecRuntimeReader &reader = player.Reader();

        const std::vector<RrPointFrame> &fin = reader.GetFinFrames();
        const std::vector<RrPointFrame> &base = reader.GetBaseFrames();
        const std::vector<RrMat4d> &finalM = reader.GetFinalMatrices();
        const std::vector<RrMat4d> &baseM = reader.GetBaseMatrices();
        if (fin.size() != graph.fin.size() ||
            base.size() != graph.base.size() ||
            finalM.size() != graph.finalMatrix.size() ||
            baseM.size() != graph.baseMatrix.size()) {
            std::printf("%s frame %.17g: pool sizes %zu/%zu/%zu/%zu vs "
                        "%zu/%zu/%zu/%zu\n", name.c_str(), frame,
                        fin.size(), base.size(), finalM.size(),
                        baseM.size(), graph.fin.size(),
                        graph.base.size(), graph.finalMatrix.size(),
                        graph.baseMatrix.size());
            CHECK(false);
            failed = true;
            continue;
        }
        for (size_t i = 0; i < fin.size(); ++i) {
            if (!_FramesEqual(graph.fin[i], fin[i])) {
                std::printf("%s frame %.17g: fin[%zu] differs "
                            "(flags %u vs %u)\n", name.c_str(), frame, i,
                            graph.fin[i].flags, fin[i].flags);
                CHECK(false);
                failed = true;
                break;
            }
        }
        for (size_t i = 0; i < base.size(); ++i) {
            if (!_FramesEqual(graph.base[i], base[i])) {
                std::printf("%s frame %.17g: base[%zu] differs "
                            "(flags %u vs %u)\n", name.c_str(), frame, i,
                            graph.base[i].flags, base[i].flags);
                CHECK(false);
                failed = true;
                break;
            }
        }
        for (size_t i = 0; i < finalM.size(); ++i) {
            if (!_MatricesEqual(graph.finalMatrix[i], finalM[i])) {
                std::printf("%s frame %.17g: finalMatrix[%zu] differs\n",
                            name.c_str(), frame, i);
                CHECK(false);
                failed = true;
                break;
            }
        }
        for (size_t i = 0; i < baseM.size(); ++i) {
            if (!_MatricesEqual(graph.baseMatrix[i], baseM[i])) {
                std::printf("%s frame %.17g: baseMatrix[%zu] differs\n",
                            name.c_str(), frame, i);
                CHECK(false);
                failed = true;
                break;
            }
        }

        // Joint matrices: path plus bitwise matrix.
        std::map<std::string, GfMatrix4d> wantJoints;
        for (const auto &entry : pose.jointMatricesFinal) {
            wantJoints[entry.first.GetString()] = entry.second;
        }
        const std::vector<RigExecRuntimeJointMatrix> &gotJoints =
            reader.GetJointMatrices();
        if (gotJoints.size() != wantJoints.size()) {
            std::printf("%s frame %.17g: %zu joints vs %zu\n",
                        name.c_str(), frame, gotJoints.size(),
                        wantJoints.size());
            CHECK(false);
            failed = true;
        } else {
            for (const RigExecRuntimeJointMatrix &joint : gotJoints) {
                const auto it = wantJoints.find(joint.path);
                if (it == wantJoints.end() ||
                    !_MatricesEqual(it->second, joint.matrix)) {
                    std::printf("%s frame %.17g: joint %s differs\n",
                                name.c_str(), frame, joint.path.c_str());
                    CHECK(false);
                    failed = true;
                    break;
                }
            }
        }

        // Diagnostics verbatim, the summary line included.
        const std::vector<std::string> &diagnostics = reader.GetDiagnostics();
        if (diagnostics != pose.diagnostics) {
            std::printf("%s frame %.17g: diagnostics differ "
                        "(%zu vs %zu)\n", name.c_str(), frame,
                        diagnostics.size(), pose.diagnostics.size());
            const size_t common =
                std::min(diagnostics.size(), pose.diagnostics.size());
            for (size_t i = 0; i < common; ++i) {
                if (diagnostics[i] != pose.diagnostics[i]) {
                    std::printf("  got:  %s\n  want: %s\n",
                                diagnostics[i].c_str(),
                                pose.diagnostics[i].c_str());
                    break;
                }
            }
            CHECK(false);
            failed = true;
        }
        if (played) {
            played->push_back(_Snapshot(reader));
        }
        ++compared;
    }

    if (failed) {
        std::printf("%s: FAILED\n", name.c_str());
    } else {
        ++comparedFixtures;
        comparedFrames += compared;
        std::printf("%s: compared %d frame(s) of a bake at %.17g%s (%zu "
                    "input(s), %zu sampled per frame)\n",
                    name.c_str(), compared, frames.front(),
                    staticClass ? ", static" : "",
                    player->GetInputCount(),
                    player.Sampler().GetAnimatedCount());
    }
}

static void
_TestFixture(const std::string &name, const std::string &stagePath,
             const std::vector<double> &frames, bool staticClass = false)
{
    const UsdStageRefPtr stage = UsdStage::Open(stagePath);
    CHECK(stage);
    if (!stage) {
        std::printf("%s: FAILED (no stage)\n", name.c_str());
        return;
    }
    _TestStage(name, stage, frames, nullptr, staticClass);
}

// The input slot of \p file named \p path, or -1.
static int64_t
_InputOfPath(const fb::RigExecWireFile &file, const std::string &path)
{
    for (size_t k = 0; k < file.inputs.size(); ++k) {
        if (RigExecFormatPathText(file, file.inputs[k].name()) == path) {
            return int64_t(k);
        }
    }
    return -1;
}

// Two transform constraints whose envelopes no mover binds: a StaticWeight
// at 0.5 and a DynamicWeight whose driver connects to a double keyed from
// 0.2 at frame 1 to \p lastDriver at frame 10 (testRigExecBinary's
// envelope bake). Both resolve through the oracle from the file's
// envelope-only weight objects.
static UsdStageRefPtr
_EnvelopeStage(double lastDriver = 0.8)
{
    const UsdStageRefPtr stage = UsdStage::CreateInMemory();
    const auto xform = [&](const char *path, const GfVec3d &translation) {
        const UsdGeomXform x = UsdGeomXform::Define(stage, SdfPath(path));
        GfMatrix4d m(1.0);
        m.SetTranslateOnly(translation);
        x.MakeMatrixXform().Set(m);
        return x.GetPrim();
    };
    xform("/Asset", GfVec3d(0));
    xform("/Asset/TargetA", GfVec3d(0));
    xform("/Asset/TargetB", GfVec3d(0));
    xform("/Asset/Source", GfVec3d(10, 0, 0));
    const UsdPrim dial = xform("/Asset/Dial", GfVec3d(0));
    const UsdAttribute amount = dial.CreateAttribute(
        TfToken("avars:amount"), SdfValueTypeNames->Double);
    amount.Set(0.2, UsdTimeCode(1.0));
    amount.Set(lastDriver, UsdTimeCode(10.0));
    stage->DefinePrim(SdfPath("/Asset/Rig"), TfToken("RigExecRoot"));
    stage->DefinePrim(SdfPath("/Asset/Rig/Movers"), TfToken("Scope"));
    const auto constraint = [&](const char *name, const char *target,
                                const SdfPath &weightPath) {
        const UsdPrim prim = stage->DefinePrim(
            SdfPath(std::string("/Asset/Rig/Movers/") + name),
            TfToken("RigExecPositionConstraint"));
        CHECK(prim.ApplyAPI(TfToken("RigExecMoverAPI")));
        prim.CreateRelationship(TfToken("rigExec:moves"))
            .SetTargets({SdfPath(target)});
        prim.CreateRelationship(TfToken("rigExec:sources"))
            .SetTargets({SdfPath("/Asset/Source")});
        prim.CreateRelationship(TfToken("rigExec:weightObject"))
            .SetTargets({weightPath});
    };
    const UsdPrim fixed = stage->DefinePrim(
        SdfPath("/Asset/Rig/Weights/Fixed"), TfToken("RigExecStaticWeight"));
    fixed.CreateRelationship(TfToken("rigExec:weightTarget"))
        .SetTargets({SdfPath("/Asset/TargetA")});
    fixed.CreateAttribute(TfToken("rigExec:defaultWeight"),
                          SdfValueTypeNames->Float)
        .Set(0.5f);
    const UsdPrim driven = stage->DefinePrim(
        SdfPath("/Asset/Rig/Weights/Driven"), TfToken("RigExecDynamicWeight"));
    driven.CreateRelationship(TfToken("rigExec:weightTarget"))
        .SetTargets({SdfPath("/Asset/TargetB")});
    driven.CreateAttribute(TfToken("rigExec:representation"),
                           SdfValueTypeNames->Token)
        .Set(TfToken("constant"));
    driven.CreateAttribute(TfToken("inputs:driver"), SdfValueTypeNames->Float)
        .AddConnection(amount.GetPath());
    constraint("A", "/Asset/TargetA", fixed.GetPath());
    constraint("B", "/Asset/TargetB", driven.GetPath());
    return stage;
}

// Constraint envelopes are computed, not replayed: a file baked at the
// first frame plays every frame bit for bit with the baked program, and
// the dynamic envelope's transform moves between the first and last frame
// although the file holds the first frame's static data alone.
static void
TestComputedEnvelopes()
{
    const std::vector<double> frames = {1.0, 5.0, 10.0};
    std::vector<_PlayedFrame> rows;
    _TestStage("computed envelopes", _EnvelopeStage(), frames, &rows);
    _CheckFramesDiffer("computed envelopes", rows);

    // The dynamic envelope driven to 1.6 at frame 10 breaks its strict
    // range: the oracle's error passes the constraint through with the
    // program's diagnostic (compared verbatim by the parity path).
    rows.clear();
    _TestStage("computed envelopes, strict violation", _EnvelopeStage(1.6),
               frames, &rows);
    _CheckFramesDiffer("computed envelopes, strict violation", rows);
    // The violation happens at frame 10 alone: the driver reaches 1.6 there
    // and stays inside the range at frames 1 and 5.
    const auto violates = [](const _PlayedFrame &row) {
        for (const std::string &line : row.diagnostics) {
            if (line.find("strict range violation") != std::string::npos) {
                return true;
            }
        }
        return false;
    };
    CHECK(rows.size() == frames.size());
    if (rows.size() == frames.size()) {
        CHECK(!violates(rows[0]) && !violates(rows[1]) && violates(rows[2]));
    }
}

// An Animated input the stage holds no value for at a time: a blocked
// sample from frame 5 on, and Default on an attribute keyed alone. The
// sampler leaves the input with no value there, so the binary reads its
// fallback as the evaluator reads the stage, rather than the bake time's
// value.
static void
TestSampledInputWithNoValue()
{
    const UsdStageRefPtr stage = _EnvelopeStage();
    const UsdAttribute amount =
        stage->GetAttributeAtPath(SdfPath("/Asset/Dial.avars:amount"));
    CHECK(amount && amount.Set(SdfValueBlock(), UsdTimeCode(5.0)));
    double probe = 0.0;
    CHECK(!amount.Get(&probe, UsdTimeCode(7.0)) &&
          !amount.Get(&probe, UsdTimeCode::Default()));
    const double defaultTime = UsdTimeCode::Default().GetValue();
    std::vector<_PlayedFrame> rows;
    _TestStage("sampled input with no value", stage,
               {1.0, 7.0, defaultTime, 10.0}, &rows);
    // Frames 7 and Default read the fallback, frame 1 the keyed 0.2.
    CHECK(rows.size() == 4);
    if (rows.size() == 4) {
        CHECK(!_SamePlayed(rows[0], rows[1], false));
        CHECK(_SamePlayed(rows[1], rows[2], false));
    }
}

// One constraint envelope a computed-envelope case reads back. The target
// is a plain Xform at the asset origin pulled toward a source at
// (sourceX, 0, 0), so its revised translation is sourceX * w, and the
// envelope w the runtime applied is recovered exactly: sourceX * double(w)
// needs fewer than 53 significant bits.
struct _EnvelopeProbe {
    const char *target;
    const char *weightObject;
    double sourceX;
};

static const RigExecRuntimeProviderXform *
_FindProviderXform(const RigExecRuntimeReader &reader, const char *path)
{
    for (const RigExecRuntimeProviderXform &xform :
         reader.GetProviderXforms()) {
        if (xform.path == path) {
            return &xform;
        }
    }
    return nullptr;
}

// Compare the binary to live native transforms and, independently, to the
// original weight arithmetic over captured authored scalar inputs. The
// optional clamped fixture input is computed literally from its raw avar.
static bool
_EnvelopesMatchOriginalReference(const std::string &name, const UsdStageRefPtr &stage,
                       const std::vector<double> &frames,
                       const std::vector<_EnvelopeProbe> &probes,
                       std::vector<std::vector<float>> *envelopes,
                       bool clampDial=false)
{
    const SdfPath rigPath = _FindRig(stage);
    RigExecRigEvaluator baked(stage, rigPath);
    std::vector<uint8_t> bytes;
    std::string error;
    if (!RigExecTestBakeAt(baked, frames.front(), &bytes, &error)) {
        std::printf("%s: bake failed: %s\n", name.c_str(), error.c_str());
        return false;
    }
    RigExecTestPlayer player;
    if (!player.Open(bytes, stage, &error)) {
        std::printf("%s: open failed: %s\n", name.c_str(), error.c_str());
        return false;
    }


    bool same = true;
    for (double frame : frames) {
        const RigExecRigPose want = baked.Evaluate(UsdTimeCode(frame));
        if (!want.valid) {
            std::printf("%s frame %g: no native pose\n",
                        name.c_str(), frame);
            return false;
        }
        if (!player.Play(frame, &error)) {
            std::printf("%s frame %g: %s\n", name.c_str(), frame,
                        error.c_str());
            return false;
        }
        std::vector<float> row;
        RigExecResolvedInputs original;
        if(clampDial) {
            const SdfPath path("/Asset/Rig/Controls/Dial.avars:tx");
            double raw=0.0;
            CHECK(stage->GetAttributeAtPath(path).Get(&raw,UsdTimeCode(frame)));
            original.SetProperty(path,VtValue(double(std::min(std::max(float(raw),0.0f),1.0f))));
        }
        for (const _EnvelopeProbe &probe : probes) {
            std::vector<float> w;
            std::string why;
            const auto reference=RigExecCaptureWeightReference(stage,SdfPath(probe.weightObject),original,{},UsdTimeCode(frame),
                [](const SdfPath &)->const GfMatrix4d *{return nullptr;});
            if (!RigExecResolveWeightReference(reference,SdfPath(probe.weightObject),1,&w,&why) ||
                w.size() != 1) {
                std::printf("%s frame %g: the original oracle failed on %s: "
                            "%s\n", name.c_str(), frame, probe.weightObject,
                            why.c_str());
                same = false;
                continue;
            }
            row.push_back(w[0]);
            const auto expected =
                want.providerXforms.find(SdfPath(probe.target));
            const RigExecRuntimeProviderXform *got =
                _FindProviderXform(player.Reader(), probe.target);
            if (expected == want.providerXforms.end() || !got) {
                std::printf("%s frame %g: %s has no revised transform "
                            "(dynamic %d, runtime %d)\n", name.c_str(),
                            frame, probe.target,
                            int(expected != want.providerXforms.end()),
                            int(got != nullptr));
                same = false;
                continue;
            }
            if (!_MatricesEqual(expected->second, got->matrix)) {
                std::printf("%s frame %g: %s differs from the dynamic "
                            "evaluator (x %.17g vs %.17g)\n", name.c_str(),
                            frame, probe.target, got->matrix[3][0],
                            expected->second[3][0]);
                same = false;
            }
            const float applied =
                static_cast<float>(got->matrix[3][0] / probe.sourceX);
            if (std::memcmp(&applied, &w[0], sizeof(float)) != 0) {
                std::printf("%s frame %g: %s applied envelope %.9g, the "
                            "original oracle resolves %.9g\n", name.c_str(),
                            frame, probe.target, double(applied),
                            double(w[0]));
                same = false;
            }
        }
        envelopes->push_back(std::move(row));
    }
    return same;
}

// testRigExecConstraints' StaticWeight 0.5 case
// (TestGeometryDomainTargetCompiles, transform arm): a constant
// StaticWeight at 0.5 bound as the constraint's weight object supersedes
// its own inputs:defaultWeight of 0.25, so the target lands halfway to the
// source.
static UsdStageRefPtr
_StaticWeightConstraintStage()
{
    const UsdStageRefPtr stage = UsdStage::CreateInMemory();
    const auto xform = [&](const char *path, const GfVec3d &translation) {
        const UsdGeomXform x = UsdGeomXform::Define(stage, SdfPath(path));
        x.MakeMatrixXform().Set(GfMatrix4d(
            GfRotation(GfVec3d(0, 0, 1), 0.0), translation));
    };
    xform("/Asset", GfVec3d(0));
    xform("/Asset/Target", GfVec3d(0));
    xform("/Asset/Source", GfVec3d(10, 0, 0));
    stage->DefinePrim(SdfPath("/Asset/Rig"), TfToken("RigExecRoot"));
    stage->DefinePrim(SdfPath("/Asset/Rig/Movers"), TfToken("Scope"));
    const UsdPrim pos = stage->DefinePrim(SdfPath("/Asset/Rig/Movers/Pos"),
                                          TfToken("RigExecPositionConstraint"));
    CHECK(pos.ApplyAPI(TfToken("RigExecMoverAPI")));
    pos.CreateRelationship(TfToken("rigExec:moves"))
        .SetTargets({SdfPath("/Asset/Target")});
    pos.CreateRelationship(TfToken("rigExec:sources"))
        .SetTargets({SdfPath("/Asset/Source")});
    pos.GetAttribute(TfToken("inputs:defaultWeight")).Set(0.25f);
    const UsdPrim weight = stage->DefinePrim(
        SdfPath("/Asset/Rig/Weights/W"), TfToken("RigExecStaticWeight"));
    weight.CreateRelationship(TfToken("rigExec:weightTarget"))
        .SetTargets({SdfPath("/Asset/Target")});
    weight.CreateAttribute(TfToken("rigExec:representation"),
                           SdfValueTypeNames->Token)
        .Set(TfToken("constant"));
    weight.CreateAttribute(TfToken("rigExec:defaultWeight"),
                           SdfValueTypeNames->Float)
        .Set(0.5f);
    pos.CreateRelationship(TfToken("rigExec:weightObject"))
        .SetTargets({weight.GetPath()});
    return stage;
}

static void
TestStaticWeightEnvelope()
{
    const char *const name = "static weight envelope";
    const std::vector<double> frames = {1.0, 2.0, 3.0};
    _TestStage(name, _StaticWeightConstraintStage(), frames);
    std::vector<std::vector<float>> envelopes;
    CHECK(_EnvelopesMatchOriginalReference(
        name, _StaticWeightConstraintStage(), frames,
        {{"/Asset/Target", "/Asset/Rig/Weights/W", 10.0}}, &envelopes));
    // The applied envelope is the weight object's at every frame.
    CHECK(envelopes.size() == frames.size());
    for (const std::vector<float> &row : envelopes) {
        CHECK(row == std::vector<float>{0.5f});
    }
    std::printf("%s: %zu frame(s) applied the weight object's envelope\n",
                name, envelopes.size());
}

// A DynamicWeight driven by a rig control's animated avar. The Dial
// control's avars:tx (a double) is keyed from 0 at frame 1 to 1.8 at frame
// 10; each weight's inputs:driver (a float) connects to it, so the oracle
// reads the double hop and narrows it. Driven: clamp, scale 0.625, bias
// 0.125, which leaves [0, 1] from frame 9 on. Modulated: strict, scale
// 0.625, over an envelope-only base (a constant StaticWeight at 0.7) that
// only rigExec:baseWeight reaches; neither factor is a power of two, so
// (b * d) * s and b * (d * s) differ at some frames. Blend: a
// CombineWeight (max, invert 0.25, strength 0.75) over a constant 0.25
// and the avar scaled by 0.5. With \p clampDial, a float math mover clamps
// the avar to [0, 1] and every driver declares rigExecReadPhase "final",
// so each reads the chain's result through the overlay rather than the
// authored value (undeclared, it would read the chain's base).
static UsdStageRefPtr
_AvarDrivenStage(bool clampDial = false)
{
    const UsdStageRefPtr stage = UsdStage::CreateInMemory();
    const auto xform = [&](const char *path, const GfVec3d &translation) {
        const UsdGeomXform x = UsdGeomXform::Define(stage, SdfPath(path));
        GfMatrix4d m(1.0);
        m.SetTranslateOnly(translation);
        x.MakeMatrixXform().Set(m);
    };
    xform("/Asset", GfVec3d(0));
    xform("/Asset/TargetA", GfVec3d(0));
    xform("/Asset/TargetB", GfVec3d(0));
    xform("/Asset/TargetC", GfVec3d(0));
    xform("/Asset/Source", GfVec3d(10, 0, 0));
    stage->DefinePrim(SdfPath("/Asset/Rig"), TfToken("RigExecRoot"));
    stage->DefinePrim(SdfPath("/Asset/Rig/Movers"), TfToken("Scope"));
    const UsdPrim dial = stage->DefinePrim(
        SdfPath("/Asset/Rig/Controls/Dial"), TfToken("RigExecControl"));
    const UsdAttribute tx =
        dial.CreateAttribute(TfToken("avars:tx"), SdfValueTypeNames->Double);
    tx.Set(0.0, UsdTimeCode(1.0));
    tx.Set(1.8, UsdTimeCode(10.0));

    const auto dynamicWeight = [&](const char *name, const char *target) {
        const UsdPrim w = stage->DefinePrim(
            SdfPath(std::string("/Asset/Rig/Weights/") + name),
            TfToken("RigExecDynamicWeight"));
        w.CreateRelationship(TfToken("rigExec:weightTarget"))
            .SetTargets({SdfPath(target)});
        w.CreateAttribute(TfToken("rigExec:representation"),
                          SdfValueTypeNames->Token)
            .Set(TfToken("constant"));
        const UsdAttribute driver = w.CreateAttribute(
            TfToken("inputs:driver"), SdfValueTypeNames->Float);
        driver.AddConnection(tx.GetPath());
        if (clampDial) {
            driver.SetMetadata(TfToken("rigExecReadPhase"),
                               std::string("final"));
        }
        return w;
    };
    const UsdPrim driven = dynamicWeight("Driven", "/Asset/TargetA");
    driven.CreateAttribute(TfToken("rigExec:rangePolicy"),
                           SdfValueTypeNames->Token)
        .Set(TfToken("clamp"));
    driven.CreateAttribute(TfToken("inputs:scale"), SdfValueTypeNames->Float)
        .Set(0.625f);
    driven.CreateAttribute(TfToken("inputs:bias"), SdfValueTypeNames->Float)
        .Set(0.125f);
    const UsdPrim half = stage->DefinePrim(
        SdfPath("/Asset/Rig/Weights/Half"), TfToken("RigExecStaticWeight"));
    half.CreateRelationship(TfToken("rigExec:weightTarget"))
        .SetTargets({SdfPath("/Asset/TargetB")});
    half.CreateAttribute(TfToken("rigExec:representation"),
                         SdfValueTypeNames->Token)
        .Set(TfToken("constant"));
    half.CreateAttribute(TfToken("rigExec:defaultWeight"),
                         SdfValueTypeNames->Float)
        .Set(0.7f);
    const UsdPrim modulated = dynamicWeight("Modulated", "/Asset/TargetB");
    modulated.CreateRelationship(TfToken("rigExec:baseWeight"))
        .SetTargets({half.GetPath()});
    modulated.CreateAttribute(TfToken("inputs:scale"), SdfValueTypeNames->Float)
        .Set(0.625f);
    // Blend: the max of a constant 0.25 and the avar scaled by 0.5, then
    // inverted by a quarter. Every object it composes is envelope-only.
    const UsdPrim quarter = stage->DefinePrim(
        SdfPath("/Asset/Rig/Weights/Quarter"), TfToken("RigExecStaticWeight"));
    quarter.CreateRelationship(TfToken("rigExec:weightTarget"))
        .SetTargets({SdfPath("/Asset/TargetC")});
    quarter.CreateAttribute(TfToken("rigExec:representation"),
                            SdfValueTypeNames->Token)
        .Set(TfToken("constant"));
    quarter.CreateAttribute(TfToken("rigExec:defaultWeight"),
                            SdfValueTypeNames->Float)
        .Set(0.25f);
    const UsdPrim ramp = dynamicWeight("Ramp", "/Asset/TargetC");
    ramp.CreateAttribute(TfToken("inputs:scale"), SdfValueTypeNames->Float)
        .Set(0.5f);
    const UsdPrim blend = stage->DefinePrim(
        SdfPath("/Asset/Rig/Weights/Blend"), TfToken("RigExecCombineWeight"));
    blend.CreateRelationship(TfToken("rigExec:weightTarget"))
        .SetTargets({SdfPath("/Asset/TargetC")});
    blend.CreateAttribute(TfToken("rigExec:combineMode"),
                          SdfValueTypeNames->Token)
        .Set(TfToken("max"));
    blend.CreateRelationship(TfToken("rigExec:inputWeights"))
        .SetTargets({quarter.GetPath(), ramp.GetPath()});
    blend.CreateAttribute(TfToken("inputs:invert"), SdfValueTypeNames->Float)
        .Set(0.25f);
    blend.CreateAttribute(TfToken("inputs:strength"), SdfValueTypeNames->Float)
        .Set(0.75f);
    if (clampDial) {
        const UsdPrim clamp = stage->DefinePrim(
            SdfPath("/Asset/Rig/Movers/ClampDial"),
            TfToken("RigExecFloatMathMover"));
        CHECK(clamp.ApplyAPI(TfToken("RigExecMoverAPI")));
        clamp.CreateAttribute(TfToken("rigExec:operation"),
                              SdfValueTypeNames->Token)
            .Set(TfToken("clamp"));
        clamp.CreateAttribute(TfToken("inputs:min"), SdfValueTypeNames->Float)
            .Set(0.0f);
        clamp.CreateAttribute(TfToken("inputs:max"), SdfValueTypeNames->Float)
            .Set(1.0f);
        clamp.CreateRelationship(TfToken("rigExec:moves"))
            .SetTargets({tx.GetPath()});
    }

    const auto constraint = [&](const char *name, const char *target,
                                const UsdPrim &weight) {
        const UsdPrim prim = stage->DefinePrim(
            SdfPath(std::string("/Asset/Rig/Movers/") + name),
            TfToken("RigExecPositionConstraint"));
        CHECK(prim.ApplyAPI(TfToken("RigExecMoverAPI")));
        prim.CreateRelationship(TfToken("rigExec:moves"))
            .SetTargets({SdfPath(target)});
        prim.CreateRelationship(TfToken("rigExec:sources"))
            .SetTargets({SdfPath("/Asset/Source")});
        prim.CreateRelationship(TfToken("rigExec:weightObject"))
            .SetTargets({weight.GetPath()});
    };
    constraint("A", "/Asset/TargetA", driven);
    constraint("B", "/Asset/TargetB", modulated);
    constraint("C", "/Asset/TargetC", blend);
    return stage;
}

// The avar as an input set on a file baked at frame 1: Dial.avars:tx set
// to 1.2 is what a fresh evaluator in the dynamic and the baked mode
// publishes at frame 1 with 1.2 authored on the avar in the session
// layer, every output bit for bit. The set moves Driven's envelope off the
// keyed value's to (1 * 1.2f) * 0.625 + 0.125.
static void
_TestAvarInputMatchesSessionEdit()
{
    const char *const name = "avar-driven envelope, input set";
    const UsdStageRefPtr stage = _AvarDrivenStage();
    const SdfPath rigPath("/Asset/Rig");
    const std::string tx = "/Asset/Rig/Controls/Dial.avars:tx";
    const double bakeTime = 1.0;
    const double value = 1.2;
    std::vector<uint8_t> bytes;
    std::string error;
    {
        RigExecRigEvaluator evaluator(stage, rigPath);
        CHECK(RigExecTestBakeAt(evaluator, bakeTime, &bytes, &error));
    }
    RigExecTestPlayer player;
    if (!player.Open(bytes, stage, &error)) {
        std::printf("%s: open: %s\n", name, error.c_str());
        CHECK(false);
        return;
    }
    CHECK(player.Play(bakeTime, &error));
    const RigExecRuntimeProviderXform *defaults =
        _FindProviderXform(player.Reader(), "/Asset/TargetA");
    const double defaultX = defaults ? defaults->matrix[3][0] : 0.0;
    CHECK(player->SetInput(tx, value, &error));
    CHECK(player.Play(bakeTime, &error));
    const RigExecRuntimeProviderXform *moved =
        _FindProviderXform(player.Reader(), "/Asset/TargetA");
    CHECK(defaults && moved);
    if (moved) {
        const float driven = (1.0f * static_cast<float>(value)) * 0.625f +
                             0.125f;
        CHECK(moved->matrix[3][0] == 10.0 * double(driven));
        CHECK(moved->matrix[3][0] != defaultX);
    }
    {
        std::vector<RigExecRigPose> poses;
        CHECK(RigExecTestEditedPoses(stage, rigPath,
                                     {{SdfPath(tx), VtValue(value)}},
                                     {bakeTime}, &poses, &error));
        std::vector<std::string> diffs;
        const bool same =
            poses.size() == 1 &&
            RigExecCompareRuntimeOutputs(poses[0], player.Reader(), &diffs);
        CHECK(same);
        std::printf("%s, native: %s\n", name,
                    same ? "binary == session edit" : "MISMATCH");
        for (const std::string &line : diffs) {
            std::printf("    %s\n", line.c_str());
        }
    }
}

static void
TestAvarDrivenDynamicEnvelope()
{
    const char *const name = "avar-driven dynamic envelope";
    const std::vector<double> frames = {1, 2, 3, 4, 5, 6, 7, 8, 9, 10};
    std::vector<_PlayedFrame> rows;
    _TestStage(name, _AvarDrivenStage(), frames, &rows);
    _CheckFramesDiffer(name, rows);
    std::vector<std::vector<float>> envelopes;
    CHECK(_EnvelopesMatchOriginalReference(
        name, _AvarDrivenStage(), frames,
        {{"/Asset/TargetA", "/Asset/Rig/Weights/Driven", 10.0},
         {"/Asset/TargetB", "/Asset/Rig/Weights/Modulated", 10.0},
         {"/Asset/TargetC", "/Asset/Rig/Weights/Blend", 10.0}},
        &envelopes));

    // The oracle's own arithmetic over the avar read at each frame, which
    // the envelopes above equal, with d the double avar narrowed to float:
    // (1 * d) * 0.625 + 0.125 clamped; (0.7 * d) * 0.625 + 0 strict; and
    // m = max(max(0, 0.25), (1 * d) * 0.5 + 0), then (m + (1 - 2m) * 0.25)
    // * 0.75, strict.
    const UsdStageRefPtr stage = _AvarDrivenStage();
    const UsdAttribute tx = stage->GetAttributeAtPath(
        SdfPath("/Asset/Rig/Controls/Dial.avars:tx"));
    CHECK(envelopes.size() == frames.size());
    size_t clamped = 0;
    size_t reassociated = 0;
    for (size_t f = 0; f < envelopes.size() && f < frames.size(); ++f) {
        double avar = 0.0;
        CHECK(tx.Get(&avar, UsdTimeCode(frames[f])));
        const float d = static_cast<float>(avar);
        float driven = (1.0f * d) * 0.625f + 0.125f;
        if (driven > 1.0f) {
            driven = 1.0f;
            ++clamped;
        }
        const float modulated = (0.7f * d) * 0.625f + 0.0f;
        const float other = 0.7f * (d * 0.625f) + 0.0f;
        reassociated += std::memcmp(&other, &modulated, sizeof(float)) != 0;
        const float m = std::max(std::max(0.0f, 0.25f),
                                 (1.0f * d) * 0.5f + 0.0f);
        const float blend = (m + (1.0f - 2.0f * m) * 0.25f) * 0.75f;
        CHECK(envelopes[f] ==
              (std::vector<float>{driven, modulated, blend}));
    }
    // Frames 9 and 10 exercise the clamp; the operation order shows.
    CHECK(clamped == 2);
    CHECK(reassociated > 0);
    _TestAvarInputMatchesSessionEdit();
    std::printf("%s: %zu frame(s) of envelopes equal the oracle's\n", name,
                envelopes.size());
}

// The avar-driven stage with a float math mover clamping the Dial avar
// to [0, 1]: each driver's walk crosses the chain's target, so the oracle
// reads the chain's double result through the generation's overlay, not
// the authored avar. The parity path compares the runtime against the
// baked program, the dynamic comparison against ExecReference and the
// original oracle, and the envelopes must be the oracle's arithmetic over
// the clamped value: from frame 7 on that differs from the authored
// value's.
static void
TestChainDrivenEnvelope()
{
    const char *const name = "chain-driven envelope";
    const std::vector<double> frames = {1, 2, 3, 4, 5, 6, 7, 8, 9, 10};
    std::vector<_PlayedFrame> rows;
    _TestStage(name, _AvarDrivenStage(true), frames, &rows);
    _CheckFramesDiffer(name, rows);
    std::vector<std::vector<float>> envelopes;
    CHECK(_EnvelopesMatchOriginalReference(
        name, _AvarDrivenStage(true), frames,
        {{"/Asset/TargetA", "/Asset/Rig/Weights/Driven", 10.0},
         {"/Asset/TargetB", "/Asset/Rig/Weights/Modulated", 10.0},
         {"/Asset/TargetC", "/Asset/Rig/Weights/Blend", 10.0}},
        &envelopes,true));
    const UsdStageRefPtr stage = _AvarDrivenStage(true);
    const UsdAttribute tx = stage->GetAttributeAtPath(
        SdfPath("/Asset/Rig/Controls/Dial.avars:tx"));
    CHECK(envelopes.size() == frames.size());
    size_t chained = 0;
    for (size_t f = 0; f < envelopes.size() && f < frames.size(); ++f) {
        double avar = 0.0;
        CHECK(tx.Get(&avar, UsdTimeCode(frames[f])));
        const float authored = static_cast<float>(avar);
        // The chain computes a double target in float and publishes the
        // double; the oracle narrows it back.
        const float d = std::min(std::max(authored, 0.0f), 1.0f);
        const float driven =
            std::min((1.0f * d) * 0.625f + 0.125f, 1.0f);
        const float modulated = (0.7f * d) * 0.625f + 0.0f;
        const float m = std::max(std::max(0.0f, 0.25f),
                                 (1.0f * d) * 0.5f + 0.0f);
        const float blend = (m + (1.0f - 2.0f * m) * 0.25f) * 0.75f;
        CHECK(envelopes[f] ==
              (std::vector<float>{driven, modulated, blend}));
        chained += d != authored;
    }
    CHECK(chained == 4);
    std::printf("%s: %zu frame(s) read through the chain\n", name, chained);
}

// The text Open fails \p bytes with; empty when it opens.
static std::string
_OpenError(const std::vector<uint8_t> &bytes)
{
    std::string error;
    const std::unique_ptr<RigExecRuntimeReader> reader =
        RigExecRuntimeReader::Open(bytes.data(), bytes.size(), &error);
    return reader ? std::string() : error;
}

// Open refuses a file whose envelopes do not match its constraints, each
// with the validator's text naming the field: an envelope index naming
// another object, a missing envelope index, and a weight-object read that
// is not a float.
static void
TestComputedOpenRefusals()
{
    const UsdStageRefPtr stage = _EnvelopeStage();
    RigExecRigEvaluator evaluator(stage, SdfPath("/Asset/Rig"));
    RigExecBakeOpts opts;
    opts.time = 1.0;
    RigExecBakeResult result;
    std::string error;
    CHECK(RigExecBakeToBinary(evaluator, opts, &result, &error));
    CHECK(_OpenError(result.bytes).empty());
    const std::unique_ptr<fb::RigExecWireFile> file =
        RigExecTestUnpack(result.bytes);
    if (!file) {
        return;
    }
    const auto expect = [&](const char *what,
                            const std::vector<uint8_t> &bytes,
                            const std::string &text) {
        const std::string want = "invalid .rigexec: " + text;
        const std::string got = _OpenError(bytes);
        if (got != want) {
            std::printf("%s: open said '%s', expected '%s'\n", what,
                        got.c_str(), want.c_str());
            CHECK(false);
        }
    };
    // The pose table orders constraint B before A.
    const std::vector<fb::RigExecWireConstraint> &constraints =
        file->pose->constraints;
    CHECK(constraints.size() == 2);
    if (constraints.size() == 2) {
        expect("swapped envelope indices",
               RigExecTestEdited(result.bytes,
                                 [](fb::RigExecWireFile *edited) {
                                     std::swap(edited->pose->constraints[0]
                                                   .weightObjectIndex,
                                               edited->pose->constraints[1]
                                                   .weightObjectIndex);
                                 }),
               "pose.constraints[0]: weight_object_index does not name the "
               "envelope of its weight object");
        expect("missing envelope index",
               RigExecTestEdited(result.bytes,
                                 [](fb::RigExecWireFile *edited) {
                                     edited->pose->constraints[0]
                                         .weightObjectIndex = -1;
                                 }),
               "pose.constraints[0]: weight_object_index does not name the "
               "envelope of its weight object");
    }
    {
        // A double-tagged read (its constant re-pointed at values[0], the
        // Double +0.0, so the read itself stays consistent).
        const size_t objects = file->geometry->weightObjects.size();
        CHECK(objects > 0);
        if (objects > 0) {
            expect("double read",
                   RigExecTestEdited(
                       result.bytes,
                       [](fb::RigExecWireFile *edited) {
                           fb::RigExecWireInput &driver =
                               *edited->geometry->weightObjects.back().driver;
                           driver.tag = fb::InputTag::Double;
                           driver.constant = 0;
                       }),
                   "geometry.weight_objects[" + std::to_string(objects - 1) +
                       "].driver: tag 0, expected 1");
        }
    }
    std::printf("computed open refusals: checked\n");
}

// The runtime's property results against the native evaluator's
// published values (RigExecRigPose::movedProperties): same type, same bits.
static bool
_SamePropertyValue(const VtValue &want, const RrPropertyValue &got)
{
    using Tag = RrPropertyValue::Tag;
    if (want.IsHolding<float>()) {
        const float w = want.UncheckedGet<float>();
        return got.tag == Tag::Float &&
               std::memcmp(&w, &got.f32, sizeof(float)) == 0;
    }
    if (want.IsHolding<double>()) {
        const double w = want.UncheckedGet<double>();
        return got.tag == Tag::Double &&
               std::memcmp(&w, &got.f64, sizeof(double)) == 0;
    }
    if (want.IsHolding<GfMatrix4d>()) {
        const GfMatrix4d &w = want.UncheckedGet<GfMatrix4d>();
        if (got.tag != Tag::Matrix4d) {
            return false;
        }
        for (int r = 0; r < 4; ++r) {
            for (int c = 0; c < 4; ++c) {
                const double a = w[r][c];
                const double b = got.matrix[size_t(r)][size_t(c)];
                if (std::memcmp(&a, &b, sizeof(double)) != 0) {
                    return false;
                }
            }
        }
        return true;
    }
    if (want.IsHolding<GfVec3f>()) {
        const GfVec3f &w = want.UncheckedGet<GfVec3f>();
        return got.tag == Tag::Vec3f &&
               std::memcmp(w.data(), got.vec.data(), sizeof(float) * 3) == 0;
    }
    return false;
}

// A value only a property chain publishes into movedProperties.
static bool
_IsPropertyResult(const VtValue &value)
{
    return value.IsHolding<float>() || value.IsHolding<double>() ||
           value.IsHolding<GfMatrix4d>() || value.IsHolding<GfVec3f>();
}

// The lines property chains write.
static std::vector<std::string>
_ChainLines(const std::vector<std::string> &lines)
{
    std::vector<std::string> out;
    for (const std::string &line : lines) {
        if (line.rfind("property chain ", 0) == 0 ||
            line.rfind("diag ", 0) == 0) {
            out.push_back(line);
        }
    }
    return out;
}

// One frame of a chain case: the runtime's property results by path, and
// the chains' diagnostic lines.
struct _ChainFrame {
    std::map<std::string, RrPropertyValue> values;
    std::vector<std::string> lines;
};

// Plays \p stage's bake at the first frame through its inputs beside an
// ExecReference evaluator (the exec-authoritative walk every evaluator is
// held to). At every frame each property result the runtime computed must
// equal the native evaluator's published value bit for bit, the two must
// publish the same set, and the chains' diagnostic lines must agree
// verbatim. \p rows receives the runtime's results and lines per frame.
// False on any difference.
static bool
_ChainsMatchLive(const std::string &name, const UsdStageRefPtr &stage,
                    const std::vector<double> &frames,
                    std::vector<_ChainFrame> *rows)
{
    const SdfPath rigPath = _FindRig(stage);
    RigExecRigEvaluator baked(stage, rigPath);
    std::vector<uint8_t> bytes;
    std::string error;
    if (!RigExecTestBakeAt(baked, frames.front(), &bytes, &error)) {
        std::printf("%s: bake failed: %s\n", name.c_str(), error.c_str());
        return false;
    }
    RigExecTestPlayer player;
    if (!player.Open(bytes, stage, &error)) {
        std::printf("%s: open failed: %s\n", name.c_str(), error.c_str());
        return false;
    }


    bool same = true;
    for (double frame : frames) {
        const RigExecRigPose want = baked.Evaluate(UsdTimeCode(frame));
        if (!want.valid) {
            std::printf("%s frame %g: no live pose\n", name.c_str(),
                        frame);
            return false;
        }
        if (!player.Play(frame, &error)) {
            std::printf("%s frame %g: %s\n", name.c_str(), frame,
                        error.c_str());
            return false;
        }
        _ChainFrame row;
        for (const RigExecRuntimePropertyValue &got :
             player->GetPropertyValues()) {
            row.values[got.path] = got.value;
            const auto found = want.movedProperties.find(SdfPath(got.path));
            if (found == want.movedProperties.end() ||
                !_SamePropertyValue(found->second, got.value)) {
                std::printf("%s frame %g: %s differs from the dynamic "
                            "evaluator (%s)\n", name.c_str(), frame,
                            got.path.c_str(),
                            found == want.movedProperties.end()
                                ? "not published there"
                                : "value");
                same = false;
            }
        }
        for (const auto &[path, value] : want.movedProperties) {
            if (_IsPropertyResult(value) &&
                !row.values.count(path.GetString())) {
                std::printf("%s frame %g: the runtime publishes nothing at "
                            "%s\n", name.c_str(), frame,
                            path.GetText());
                same = false;
            }
        }
        const std::vector<std::string> got =
            _ChainLines(player->GetDiagnostics());
        const std::vector<std::string> expected =
            _ChainLines(want.diagnostics);
        if (got != expected) {
            std::printf("%s frame %g: chain lines differ (%zu vs %zu)\n",
                        name.c_str(), frame, got.size(), expected.size());
            for (size_t i = 0; i < std::max(got.size(), expected.size());
                 ++i) {
                std::printf("  got:  %s\n  want: %s\n",
                            i < got.size() ? got[i].c_str() : "",
                            i < expected.size() ? expected[i].c_str() : "");
            }
            same = false;
        }
        row.lines = got;
        rows->push_back(std::move(row));
    }
    return same;
}

// A rig of property movers only: channels under /Asset/Rig/Channels,
// movers under /Asset/Rig/Movers, frames 1 to 10.
static UsdStageRefPtr
_ChainBaseStage()
{
    const UsdStageRefPtr stage = UsdStage::CreateInMemory();
    stage->SetStartTimeCode(1.0);
    stage->SetEndTimeCode(10.0);
    UsdGeomXform::Define(stage, SdfPath("/Asset"));
    stage->DefinePrim(SdfPath("/Asset/Rig"), TfToken("RigExecRoot"));
    stage->DefinePrim(SdfPath("/Asset/Rig/Channels"), TfToken("Scope"));
    stage->DefinePrim(SdfPath("/Asset/Rig/Movers"), TfToken("Scope"));
    return stage;
}

static UsdAttribute
_Channel(const UsdStageRefPtr &stage, const char *scope, const char *name,
         const SdfValueTypeName &type)
{
    const UsdPrim prim = stage->DefinePrim(
        SdfPath(std::string("/Asset/Rig/Channels/") + scope),
        TfToken("Scope"));
    return prim.CreateAttribute(TfToken(name), type);
}

// A math mover at /Asset/Rig/Movers/<path> moving \p target. A nested
// mover revises before its parent, which is how a case fixes the order.
static UsdPrim
_MathMover(const UsdStageRefPtr &stage, const std::string &path,
           const char *type, const char *operation,
           const UsdAttribute &target)
{
    const UsdPrim prim =
        stage->DefinePrim(SdfPath("/Asset/Rig/Movers/" + path), TfToken(type));
    CHECK(prim.ApplyAPI(TfToken("RigExecMoverAPI")));
    prim.CreateAttribute(TfToken("rigExec:operation"),
                         SdfValueTypeNames->Token)
        .Set(TfToken(operation));
    prim.CreateRelationship(TfToken("rigExec:moves"))
        .SetTargets({target.GetPath()});
    return prim;
}

template <class T>
static void
_SetInput(const UsdPrim &prim, const char *name, const SdfValueTypeName &type,
          const T &value)
{
    prim.CreateAttribute(TfToken(name), type).Set(value);
}

static const GfVec2f kShapeKeys[] = {
    GfVec2f(0.0f, 0.0f), GfVec2f(0.5f, 0.8f), GfVec2f(2.0f, 1.2f)};
static const GfVec2f kShapeTangents[] = {
    GfVec2f(1.5f, 1.5f), GfVec2f(0.6f, 0.6f), GfVec2f(0.1f, 0.1f)};

// A float chain: a Hermite curve with tangents, then a clamp to [0.1, 1],
// over a keyed base that leaves the curve's keys on both sides. Beside it
// a curve of twelve linear keys (no tangents, the binary-search segment
// lookup) on a second channel.
static UsdStageRefPtr
_FloatCurveClampStage()
{
    const UsdStageRefPtr stage = _ChainBaseStage();
    const UsdAttribute amount =
        _Channel(stage, "Dial", "rigExec:amount", SdfValueTypeNames->Float);
    amount.Set(-0.25f, UsdTimeCode(1.0));
    amount.Set(0.75f, UsdTimeCode(4.0));
    amount.Set(1.5f, UsdTimeCode(7.0));
    amount.Set(3.0f, UsdTimeCode(10.0));
    const UsdPrim limit = _MathMover(stage, "Limit", "RigExecFloatMathMover",
                                     "clamp", amount);
    _SetInput(limit, "inputs:min", SdfValueTypeNames->Float, 0.1f);
    _SetInput(limit, "inputs:max", SdfValueTypeNames->Float, 1.0f);
    const UsdPrim shape = _MathMover(stage, "Limit/Shape",
                                     "RigExecFloatMathMover", "curve", amount);
    _SetInput(shape, "inputs:keys", SdfValueTypeNames->Float2Array,
              VtArray<GfVec2f>(std::begin(kShapeKeys), std::end(kShapeKeys)));
    _SetInput(shape, "inputs:tangents", SdfValueTypeNames->Float2Array,
              VtArray<GfVec2f>(std::begin(kShapeTangents),
                               std::end(kShapeTangents)));

    const UsdAttribute ramp =
        _Channel(stage, "Ramp", "rigExec:amount", SdfValueTypeNames->Float);
    ramp.Set(-1.0f, UsdTimeCode(1.0));
    ramp.Set(12.5f, UsdTimeCode(10.0));
    VtArray<GfVec2f> steps;
    for (int i = 0; i < 12; ++i) {
        steps.push_back(GfVec2f(float(i), float(i * i) * 0.125f));
    }
    const UsdPrim staircase = _MathMover(
        stage, "Staircase", "RigExecFloatMathMover", "curve", ramp);
    _SetInput(staircase, "inputs:keys", SdfValueTypeNames->Float2Array,
              steps);
    return stage;
}

// A double chain multiplied by 1.25 computed in float, over a base keyed
// between values a float cannot hold exactly.
static UsdStageRefPtr
_DoubleChainStage()
{
    const UsdStageRefPtr stage = _ChainBaseStage();
    const UsdAttribute level =
        _Channel(stage, "Wide", "rigExec:level", SdfValueTypeNames->Double);
    level.Set(0.1, UsdTimeCode(1.0));
    level.Set(1.0 / 3.0, UsdTimeCode(10.0));
    const UsdPrim scale = _MathMover(stage, "Scale", "RigExecFloatMathMover",
                                     "multiply", level);
    _SetInput(scale, "inputs:value", SdfValueTypeNames->Float, 1.1f);
    return stage;
}

// A matrix chain: multiply by a rotation and translation under
// inputs:defaultWeight 0.5, then blend toward a translation under 0.25,
// then a multiply whose operand is not finite (inputs unusable).
static UsdStageRefPtr
_MatrixChainStage()
{
    const UsdStageRefPtr stage = _ChainBaseStage();
    const UsdAttribute local =
        _Channel(stage, "Space", "rigExec:local", SdfValueTypeNames->Matrix4d);
    local.Set(GfMatrix4d(GfRotation(GfVec3d(0, 1, 0), 10.0),
                         GfVec3d(1.0, 2.0, 3.0)),
              UsdTimeCode(1.0));
    local.Set(GfMatrix4d(GfRotation(GfVec3d(0, 1, 0), 70.0),
                         GfVec3d(4.0, -2.0, 0.5)),
              UsdTimeCode(10.0));
    const UsdPrim bad = _MathMover(stage, "Bad", "RigExecMatrixMathMover",
                                   "multiply", local);
    GfMatrix4d infinite(1.0);
    infinite[3][0] = std::numeric_limits<double>::infinity();
    _SetInput(bad, "inputs:value", SdfValueTypeNames->Matrix4d, infinite);
    const UsdPrim snap = _MathMover(stage, "Bad/Snap",
                                    "RigExecMatrixMathMover", "blend", local);
    _SetInput(snap, "inputs:value", SdfValueTypeNames->Matrix4d,
              GfMatrix4d(1.0).SetTranslate(GfVec3d(2.0, 2.0, 2.0)));
    _SetInput(snap, "inputs:defaultWeight", SdfValueTypeNames->Float, 0.25f);
    const UsdPrim offset = _MathMover(stage, "Bad/Snap/Offset",
                                      "RigExecMatrixMathMover", "multiply",
                                      local);
    _SetInput(offset, "inputs:value", SdfValueTypeNames->Matrix4d,
              GfMatrix4d(GfRotation(GfVec3d(0, 0, 1), 30.0),
                         GfVec3d(0.0, 5.0, 0.0)));
    _SetInput(offset, "inputs:defaultWeight", SdfValueTypeNames->Float, 0.5f);
    return stage;
}

// A vec3f chain: add, then clamp per component, then remap under 0.5.
static UsdStageRefPtr
_Vec3fChainStage()
{
    const UsdStageRefPtr stage = _ChainBaseStage();
    const UsdAttribute offset =
        _Channel(stage, "Vec", "rigExec:offset", SdfValueTypeNames->Float3);
    offset.Set(GfVec3f(0.0f, 0.0f, 0.0f), UsdTimeCode(1.0));
    offset.Set(GfVec3f(2.0f, 5.0f, -3.0f), UsdTimeCode(10.0));
    const UsdPrim spread = _MathMover(stage, "Spread",
                                      "RigExecVec3fMathMover", "remap", offset);
    _SetInput(spread, "inputs:min", SdfValueTypeNames->Float3,
              GfVec3f(-2.0f, -2.0f, -2.0f));
    _SetInput(spread, "inputs:max", SdfValueTypeNames->Float3,
              GfVec3f(2.0f, 3.0f, 2.0f));
    _SetInput(spread, "inputs:defaultWeight", SdfValueTypeNames->Float, 0.5f);
    const UsdPrim bound = _MathMover(stage, "Spread/Bound",
                                     "RigExecVec3fMathMover", "clamp", offset);
    _SetInput(bound, "inputs:min", SdfValueTypeNames->Float3,
              GfVec3f(-1.0f, -1.0f, -1.0f));
    _SetInput(bound, "inputs:max", SdfValueTypeNames->Float3,
              GfVec3f(1.0f, 4.5f, 1.0f));
    const UsdPrim lift = _MathMover(stage, "Spread/Bound/Lift",
                                    "RigExecVec3fMathMover", "add", offset);
    _SetInput(lift, "inputs:value", SdfValueTypeNames->Float3,
              GfVec3f(0.0f, 2.0f, 0.0f));
    return stage;
}

// Disabled revisions: one always (inputs:enabled false), one keyed off from
// frame 4 to frame 7, each passing the value through with its line.
static UsdStageRefPtr
_DisabledStage()
{
    const UsdStageRefPtr stage = _ChainBaseStage();
    const UsdAttribute amount =
        _Channel(stage, "Dial", "rigExec:amount", SdfValueTypeNames->Float);
    amount.Set(0.2f, UsdTimeCode(1.0));
    amount.Set(0.8f, UsdTimeCode(10.0));
    const UsdPrim off = _MathMover(stage, "Off", "RigExecFloatMathMover",
                                   "add", amount);
    _SetInput(off, "inputs:value", SdfValueTypeNames->Float, 100.0f);
    _SetInput(off, "inputs:enabled", SdfValueTypeNames->Bool, false);
    const UsdPrim toggle = _MathMover(stage, "Off/Toggle",
                                      "RigExecFloatMathMover", "multiply",
                                      amount);
    _SetInput(toggle, "inputs:value", SdfValueTypeNames->Float, 2.0f);
    const UsdAttribute enabled = toggle.CreateAttribute(
        TfToken("inputs:enabled"), SdfValueTypeNames->Bool);
    enabled.Set(true, UsdTimeCode(1.0));
    enabled.Set(false, UsdTimeCode(4.0));
    enabled.Set(true, UsdTimeCode(8.0));
    return stage;
}

// Envelopes on property revisions: a blend under a StaticWeight 0.5, an add
// under a clamped DynamicWeight an avar drives, an add under a strict one
// the avar drives out of range from frame 8 (the oracle's error passes the
// revision through), an add under inputs:defaultWeight 0.25, and three
// whose defaultWeight is out of range: 1.5, -0.25 and NaN.
static UsdStageRefPtr
_EnvelopeChainStage()
{
    const UsdStageRefPtr stage = _ChainBaseStage();
    const UsdPrim dial = stage->DefinePrim(
        SdfPath("/Asset/Rig/Controls/Dial"), TfToken("RigExecControl"));
    const UsdAttribute ty =
        dial.CreateAttribute(TfToken("avars:ty"), SdfValueTypeNames->Double);
    ty.Set(0.0, UsdTimeCode(1.0));
    ty.Set(3.0, UsdTimeCode(10.0));
    const UsdAttribute amount =
        _Channel(stage, "Dial", "rigExec:amount", SdfValueTypeNames->Float);
    amount.Set(0.1f, UsdTimeCode(1.0));
    amount.Set(0.45f, UsdTimeCode(5.0));
    amount.Set(0.2f, UsdTimeCode(10.0));

    const UsdPrim half = stage->DefinePrim(SdfPath("/Asset/Rig/Weights/Half"),
                                           TfToken("RigExecStaticWeight"));
    half.CreateRelationship(TfToken("rigExec:weightTarget"))
        .SetTargets({amount.GetPath()});
    half.CreateAttribute(TfToken("rigExec:representation"),
                         SdfValueTypeNames->Token)
        .Set(TfToken("constant"));
    half.CreateAttribute(TfToken("rigExec:defaultWeight"),
                         SdfValueTypeNames->Float)
        .Set(0.5f);
    const auto dynamicWeight = [&](const char *name, float scale,
                                   const char *rangePolicy) {
        const UsdPrim w = stage->DefinePrim(
            SdfPath(std::string("/Asset/Rig/Weights/") + name),
            TfToken("RigExecDynamicWeight"));
        w.CreateRelationship(TfToken("rigExec:weightTarget"))
            .SetTargets({amount.GetPath()});
        w.CreateAttribute(TfToken("rigExec:representation"),
                          SdfValueTypeNames->Token)
            .Set(TfToken("constant"));
        w.CreateAttribute(TfToken("rigExec:rangePolicy"),
                          SdfValueTypeNames->Token)
            .Set(TfToken(rangePolicy));
        w.CreateAttribute(TfToken("inputs:driver"), SdfValueTypeNames->Float)
            .AddConnection(ty.GetPath());
        w.CreateAttribute(TfToken("inputs:scale"), SdfValueTypeNames->Float)
            .Set(scale);
        return w;
    };
    const UsdPrim ramp = dynamicWeight("Ramp", 0.5f, "clamp");
    const UsdPrim steep = dynamicWeight("Steep", 0.45f, "strict");

    const UsdPrim over = _MathMover(stage, "Over", "RigExecFloatMathMover",
                                    "add", amount);
    _SetInput(over, "inputs:value", SdfValueTypeNames->Float, 100.0f);
    _SetInput(over, "inputs:defaultWeight", SdfValueTypeNames->Float, 1.5f);
    const UsdPrim soft = _MathMover(stage, "Over/Soft",
                                    "RigExecFloatMathMover", "add", amount);
    _SetInput(soft, "inputs:value", SdfValueTypeNames->Float, 0.5f);
    _SetInput(soft, "inputs:defaultWeight", SdfValueTypeNames->Float, 0.25f);
    const UsdPrim kick = _MathMover(stage, "Over/Soft/Kick",
                                    "RigExecFloatMathMover", "add", amount);
    _SetInput(kick, "inputs:value", SdfValueTypeNames->Float, 0.125f);
    kick.CreateRelationship(TfToken("rigExec:weightObject"))
        .SetTargets({steep.GetPath()});
    const UsdPrim nudge = _MathMover(stage, "Over/Soft/Kick/Nudge",
                                     "RigExecFloatMathMover", "add", amount);
    _SetInput(nudge, "inputs:value", SdfValueTypeNames->Float, 0.05f);
    nudge.CreateRelationship(TfToken("rigExec:weightObject"))
        .SetTargets({ramp.GetPath()});
    const UsdPrim fade = _MathMover(stage, "Over/Soft/Kick/Nudge/Fade",
                                    "RigExecFloatMathMover", "blend", amount);
    _SetInput(fade, "inputs:value", SdfValueTypeNames->Float, 0.3f);
    fade.CreateRelationship(TfToken("rigExec:weightObject"))
        .SetTargets({half.GetPath()});
    // Refused at both other edges of the defaultWeight range: below 0 (an
    // envelope that would otherwise leave the value as it is) and NaN.
    const UsdPrim negative =
        _MathMover(stage, "Over/Soft/Kick/Nudge/Fade/Negative",
                   "RigExecFloatMathMover", "add", amount);
    _SetInput(negative, "inputs:value", SdfValueTypeNames->Float, 100.0f);
    _SetInput(negative, "inputs:defaultWeight", SdfValueTypeNames->Float,
              -0.25f);
    const UsdPrim notANumber =
        _MathMover(stage, "Over/Soft/Kick/Nudge/Fade/Negative/NotANumber",
                   "RigExecFloatMathMover", "add", amount);
    _SetInput(notANumber, "inputs:value", SdfValueTypeNames->Float, 100.0f);
    _SetInput(notANumber, "inputs:defaultWeight", SdfValueTypeNames->Float,
              std::numeric_limits<float>::quiet_NaN());
    return stage;
}

// Phased consumers: a float chain (multiply by 2, then add 0.25) read at
// its base and after its first revision, and a double chain's base read
// into a float input, each by a float math mover whose result is a chain of
// its own.
static UsdStageRefPtr
_PhasedStage()
{
    const UsdStageRefPtr stage = _ChainBaseStage();
    const UsdAttribute amount =
        _Channel(stage, "Dial", "rigExec:amount", SdfValueTypeNames->Float);
    amount.Set(0.1f, UsdTimeCode(1.0));
    amount.Set(0.7f, UsdTimeCode(10.0));
    const UsdPrim bias = _MathMover(stage, "Bias", "RigExecFloatMathMover",
                                    "add", amount);
    _SetInput(bias, "inputs:value", SdfValueTypeNames->Float, 0.25f);
    const UsdPrim gain = _MathMover(stage, "Bias/Gain",
                                    "RigExecFloatMathMover", "multiply",
                                    amount);
    _SetInput(gain, "inputs:value", SdfValueTypeNames->Float, 2.0f);
    const UsdAttribute level =
        _Channel(stage, "Wide", "rigExec:level", SdfValueTypeNames->Double);
    level.Set(0.1, UsdTimeCode(1.0));
    level.Set(1.0 / 3.0, UsdTimeCode(10.0));
    const UsdPrim scale = _MathMover(stage, "Scale", "RigExecFloatMathMover",
                                     "multiply", level);
    _SetInput(scale, "inputs:value", SdfValueTypeNames->Float, 1.25f);

    const auto readout = [&](const char *name, const UsdAttribute &source,
                             const char *phase) {
        const UsdAttribute target =
            _Channel(stage, "Readouts", (std::string("rigExec:") + name).c_str(),
                     SdfValueTypeNames->Float);
        target.Set(0.0f);
        const UsdPrim mover =
            _MathMover(stage, std::string("Readouts/") + name,
                       "RigExecFloatMathMover", "add", target);
        const UsdAttribute value = mover.CreateAttribute(
            TfToken("inputs:value"), SdfValueTypeNames->Float);
        value.AddConnection(source.GetPath());
        value.SetMetadata(TfToken("rigExecReadPhase"), std::string(phase));
    };
    stage->DefinePrim(SdfPath("/Asset/Rig/Movers/Readouts"), TfToken("Scope"));
    readout("base", amount, "base");
    readout("early", amount, "/Asset/Rig/Movers/Bias/Gain");
    readout("narrow", level, "base");
    return stage;
}

// The default read phase: a dial keyed from 0.1 to 0.45, doubled by Gain and
// clamped to 0.6 by Limit, read by add movers into readout channels --
// undeclared (the base), `final` (no record: the dial's published value
// answers it), `base` on a hop, `final` through that hop (a record of every
// revision), and undeclared through a `final` hop (the base).
static UsdStageRefPtr
_DefaultPhaseStage()
{
    const UsdStageRefPtr stage = _ChainBaseStage();
    const UsdAttribute amount =
        _Channel(stage, "Dial", "rigExec:amount", SdfValueTypeNames->Float);
    amount.Set(0.1f, UsdTimeCode(1.0));
    amount.Set(0.45f, UsdTimeCode(10.0));
    const UsdPrim limit = _MathMover(stage, "Limit", "RigExecFloatMathMover",
                                     "clamp", amount);
    _SetInput(limit, "inputs:min", SdfValueTypeNames->Float, 0.0f);
    _SetInput(limit, "inputs:max", SdfValueTypeNames->Float, 0.6f);
    const UsdPrim gain = _MathMover(stage, "Limit/Gain",
                                    "RigExecFloatMathMover", "multiply",
                                    amount);
    _SetInput(gain, "inputs:value", SdfValueTypeNames->Float, 2.0f);
    stage->DefinePrim(SdfPath("/Asset/Rig/Movers/Readouts"), TfToken("Scope"));
    const auto readout = [&](const char *name, const SdfPath &source,
                             const char *phase) {
        const UsdAttribute target =
            _Channel(stage, "Readouts", (std::string("rigExec:") + name).c_str(),
                     SdfValueTypeNames->Float);
        target.Set(0.0f);
        const UsdPrim mover =
            _MathMover(stage, std::string("Readouts/") + name,
                       "RigExecFloatMathMover", "add", target);
        const UsdAttribute value = mover.CreateAttribute(
            TfToken("inputs:value"), SdfValueTypeNames->Float);
        value.AddConnection(source);
        if (phase) {
            value.SetMetadata(TfToken("rigExecReadPhase"), std::string(phase));
        }
    };
    readout("undeclared", amount.GetPath(), nullptr);
    readout("final", amount.GetPath(), "final");
    readout("hop", amount.GetPath(), "base");
    readout("finalViaHop",
            SdfPath("/Asset/Rig/Movers/Readouts/hop.inputs:value"), "final");
    readout("finalHop", amount.GetPath(), "final");
    readout("baseViaFinalHop",
            SdfPath("/Asset/Rig/Movers/Readouts/finalHop.inputs:value"),
            nullptr);
    return stage;
}

// Bases a chain cannot use and results it refuses: a float base keyed to
// +inf at frame 3 and NaN at frame 5; a double base of 1e300, finite as a
// double but not as the float it is tested through; a multiply that
// overflows a float from frame 7 on; an add whose input is +inf.
static UsdStageRefPtr
_NonFiniteStage()
{
    const UsdStageRefPtr stage = _ChainBaseStage();
    const UsdAttribute bad =
        _Channel(stage, "Bad", "rigExec:amount", SdfValueTypeNames->Float);
    bad.Set(0.5f, UsdTimeCode(1.0));
    bad.Set(std::numeric_limits<float>::infinity(), UsdTimeCode(3.0));
    bad.Set(std::numeric_limits<float>::quiet_NaN(), UsdTimeCode(5.0));
    bad.Set(0.25f, UsdTimeCode(7.0));
    const UsdPrim plus = _MathMover(stage, "Plus", "RigExecFloatMathMover",
                                    "add", bad);
    _SetInput(plus, "inputs:value", SdfValueTypeNames->Float, 1.0f);

    const UsdAttribute huge =
        _Channel(stage, "Huge", "rigExec:level", SdfValueTypeNames->Double);
    huge.Set(1e300);
    const UsdPrim keep = _MathMover(stage, "Keep", "RigExecFloatMathMover",
                                    "multiply", huge);
    _SetInput(keep, "inputs:value", SdfValueTypeNames->Float, 1.0f);

    const UsdAttribute blow =
        _Channel(stage, "Blow", "rigExec:amount", SdfValueTypeNames->Float);
    blow.Set(0.5f, UsdTimeCode(1.0));
    blow.Set(2.0f, UsdTimeCode(7.0));
    const UsdPrim inf = _MathMover(stage, "Inf", "RigExecFloatMathMover",
                                   "add", blow);
    _SetInput(inf, "inputs:value", SdfValueTypeNames->Float,
              std::numeric_limits<float>::infinity());
    const UsdPrim overflow = _MathMover(stage, "Inf/Overflow",
                                        "RigExecFloatMathMover", "multiply",
                                        blow);
    _SetInput(overflow, "inputs:value", SdfValueTypeNames->Float, 3e38f);
    return stage;
}

static float
_Float(const _ChainFrame &row, const std::string &path, bool *found)
{
    const auto it = row.values.find(path);
    *found = it != row.values.end() &&
             it->second.tag == RrPropertyValue::Tag::Float;
    return *found ? it->second.f32 : 0.0f;
}

static UsdStageRefPtr
_OraclePublicationStage(bool prior)
{
    const auto stage = _ChainBaseStage();
    stage->SetEndTimeCode(2.0);
    const auto driver = _Channel(stage, prior ? "A_Driver" : "Z_Driver",
                                 "rigExec:amount", SdfValueTypeNames->Float);
    driver.Set(0.125f);
    driver.Set(0.125f, UsdTimeCode(1));
    driver.Set(0.375f, UsdTimeCode(2));
    const auto receiver = _Channel(stage, "M_Receiver", "rigExec:amount",
                                   SdfValueTypeNames->Float);
    receiver.Set(0.25f);
    const auto shift = _MathMover(stage, prior ? "A_Driver" : "Z_Driver",
                                  "RigExecFloatMathMover", "add", driver);
    _SetInput(shift, "inputs:value", SdfValueTypeNames->Float, 0.5f);
    const auto blend = _MathMover(stage, "M_Receiver", "RigExecFloatMathMover",
                                  "add", receiver);
    _SetInput(blend, "inputs:value", SdfValueTypeNames->Float, 1.0f);
    const auto envelope = stage->DefinePrim(SdfPath("/Asset/Rig/Weights/Envelope"),
                                           TfToken("RigExecDynamicWeight"));
    envelope.CreateRelationship(TfToken("rigExec:weightTarget")).SetTargets({receiver.GetPath()});
    envelope.CreateAttribute(TfToken("rigExec:representation"), SdfValueTypeNames->Token)
        .Set(TfToken("constant"));
    envelope.CreateAttribute(TfToken("rigExec:rangePolicy"), SdfValueTypeNames->Token)
        .Set(TfToken("clamp"));
    const auto envelopeDriver = envelope.CreateAttribute(TfToken("inputs:driver"), SdfValueTypeNames->Float);
    envelopeDriver.AddConnection(driver.GetPath());
    envelopeDriver.SetMetadata(TfToken("rigExecReadPhase"), std::string("final"));
    blend.CreateRelationship(TfToken("rigExec:weightObject")).SetTargets({envelope.GetPath()});
    return stage;
}

static void
TestOraclePublicationGeneration()
{
    const std::vector<double> frames{1, 2, 2, 1};
    std::vector<std::vector<_ChainFrame>> variants;
    for (const bool prior : {true, false}) {
        const auto stage = _OraclePublicationStage(prior);
        const std::string label = prior ? "oracle prior finished chain" : "oracle later cached chain";
        std::vector<uint8_t> bytes;
        std::string error;
        RigExecRigEvaluator bake(stage, SdfPath("/Asset/Rig"));
        CHECK(RigExecTestBakeAt(bake, 1, &bytes, &error));
        const auto file = RigExecTestUnpack(bytes);
        CHECK(file);
        if (!file) continue;
        const std::string driver = std::string("/Asset/Rig/Channels/") +
            (prior ? "A_Driver" : "Z_Driver") + ".rigExec:amount";
        const std::string receiver = "/Asset/Rig/Channels/M_Receiver.rigExec:amount";
        int driverFinal = -1, receiverRevision = -1;
        for (size_t s = 0; s < file->steps.size(); ++s) {
            const auto &step = file->steps[s];
            if (!step.isHead || step.kind != fb::StepKind::PropertyRevision) continue;
            const auto &chain = file->propertyChains[size_t(step.object)];
            const auto target = RigExecFormatPathText(*file, file->inputs[chain.target].name());
            if (target == driver && size_t(step.part) == chain.revisions.size()) driverFinal = int(s);
            if (target == receiver && step.part == 1) receiverRevision = int(s);
        }
        CHECK(driverFinal >= 0 && receiverRevision >= 0);
        // Only chains available to the original receiver oracle contribute
        // producer dependencies. Later chains retain the raw-source answer.
        if (prior) CHECK(driverFinal < receiverRevision);
        _TestStage(label, stage, frames);
        std::vector<_ChainFrame> rows;
        CHECK(_ChainsMatchLive(label, stage, frames, &rows));
        CHECK(rows.size() == frames.size());
        for (size_t f = 0; f < rows.size() && f < frames.size(); ++f) {
            const float raw = frames[f] == 1 ? 0.125f : 0.375f;
            bool found = false;
            CHECK(_Float(rows[f], driver, &found) == raw + 0.5f && found);
            const float actual = _Float(rows[f], receiver, &found);
            const float expected = 0.25f + (prior ? raw + 0.5f : raw);
            if (!found || actual != expected)
                std::printf("%s frame %g: raw %.9g driver %.9g receiver %.9g expected %.9g\n",
                            label.c_str(), frames[f], raw,
                            _Float(rows[f], driver, &found), actual, expected);
            CHECK(actual == expected && found);
        }
        if (rows.size() == frames.size()) {
            CHECK(_SameBits(rows[0].values.at(receiver), rows[3].values.at(receiver)));
            CHECK(_SameBits(rows[1].values.at(receiver), rows[2].values.at(receiver)));
            CHECK(!_SameBits(rows[0].values.at(receiver), rows[1].values.at(receiver)));
        }
        variants.push_back(std::move(rows));
    }
    CHECK(variants.size() == 2);
    if (variants.size() == 2 && !variants[0].empty() && !variants[1].empty())
        CHECK(!_SameBits(variants[0][0].values.at("/Asset/Rig/Channels/M_Receiver.rigExec:amount"),
                         variants[1][0].values.at("/Asset/Rig/Channels/M_Receiver.rigExec:amount")));
}

static void
TestDoubleTailCycle()
{
    const auto stage = _ChainBaseStage();
    stage->SetEndTimeCode(4.0);
    const auto a = _Channel(stage, "CycleA", "rigExec:amount", SdfValueTypeNames->Float);
    const auto b = _Channel(stage, "CycleB", "rigExec:amount", SdfValueTypeNames->Double);
    a.Set(0.125f);
    b.Set(0.375);
    b.Set(0.375, UsdTimeCode(1));
    b.Set(0.625, UsdTimeCode(2));
    b.Set(SdfValueBlock(), UsdTimeCode(3));
    b.Set(0.375, UsdTimeCode(4));
    a.AddConnection(b.GetPath());
    b.AddConnection(a.GetPath());
    const auto target = _Channel(stage, "CycleResult", "rigExec:amount", SdfValueTypeNames->Float);
    target.Set(0.25f);
    const auto mover = _MathMover(stage, "CycleResult", "RigExecFloatMathMover", "add", target);
    const auto value = mover.CreateAttribute(TfToken("inputs:value"), SdfValueTypeNames->Float);
    value.AddConnection(a.GetPath());
    const std::vector<double> frames{1, 2, 3, 4, 2};
    _TestStage("double tail fresh cycle", stage, frames);
    std::vector<_ChainFrame> rows;
    CHECK(_ChainsMatchLive("double tail fresh cycle", stage, frames, &rows));
    CHECK(rows.size() == frames.size());
    if (rows.size() == frames.size()) {
        bool found = false;
        const std::string path = target.GetPath().GetString();
        CHECK(_Float(rows[0], path, &found) == 0.625f && found);
        CHECK(_Float(rows[1], path, &found) == 0.875f && found);
        CHECK(_SameBits(rows[0].values.at(path), rows[3].values.at(path)));
        CHECK(_SameBits(rows[1].values.at(path), rows[4].values.at(path)));
        CHECK(!_SameBits(rows[0].values.at(path), rows[1].values.at(path)));
    }
}

static void
TestScalarRawKindPreservesRecords()
{
    const auto stage = _PhasedStage();
    const std::string path = "/Asset/Rig/Movers/Readouts/early.inputs:value";
    CHECK(stage->GetAttributeAtPath(SdfPath(path)).Set(0.0f));
    RigExecRigEvaluator baked(stage, SdfPath("/Asset/Rig"));
    std::vector<uint8_t> bytes;
    std::string error;
    CHECK(RigExecTestBakeAt(baked, 1, &bytes, &error));
    auto reader = RigExecRuntimeReader::Open(bytes.data(), bytes.size(), &error);
    const auto file = RigExecTestUnpack(bytes);
    CHECK(reader && file);
    if (!reader || !file) return;
    size_t slot = 0;
    CHECK(reader->FindInput(path, &slot));
    bool recorded = false;
    for (const auto &record : file->phasedConsumers)
        recorded = recorded || RigExecFormatPathText(*file, file->inputs[record.consumer].name()) == path;
    CHECK(recorded);
    const auto raw = reader->GetInputValue(slot);
    CHECK(raw.tag == RrInputTag::Float && raw.f32 == 0.0f);
    CHECK(reader->Execute(&error));
    const auto baseline = _Snapshot(*reader);
    const auto recordValue = [&](const auto &row) {
        bool present = false;
        for (const auto &value : row.properties)
            if (value.path == path) {
                present = true;
                CHECK(value.value.tag == RrPropertyValue::Tag::Float && value.value.f32 == 0.2f);
            }
        CHECK(present);
    };
    recordValue(baseline);
    CHECK(reader->SetInputAt(slot, raw, &error));
    CHECK(reader->Execute(&error));
    recordValue(_Snapshot(*reader));
    CHECK(_SamePlayed(baseline, _Snapshot(*reader)));
    CHECK(reader->ResetInput(path, &error));
    CHECK(reader->Execute(&error));
    CHECK(_SamePlayed(baseline, _Snapshot(*reader)));
    CHECK(reader->SetSampledInputAt(slot, raw, &error));
    CHECK(reader->Execute(&error));
    CHECK(_SamePlayed(baseline, _Snapshot(*reader)));
    CHECK(reader->SetInput(path, 0.75, &error));
    CHECK(reader->Execute(&error));
    recordValue(_Snapshot(*reader));
    CHECK(_SamePlayed(baseline, _Snapshot(*reader)));
    // This authored value is below the computed phased publication, just
    // as a session-layer edit: it does not suppress or replace the record.
    CHECK(stage->GetAttributeAtPath(SdfPath(path)).Set(0.75f));
    std::vector<_ChainFrame> rows;
    CHECK(_ChainsMatchLive("raw authored scalar retains phased record", stage, {1}, &rows));
    CHECK(rows.size() == 1);
    if (!rows.empty()) {
        bool found = false;
        CHECK(_Float(rows[0], path, &found) == 0.2f && found);
    }
    CHECK(reader->ResetInput(path, &error));
    CHECK(reader->Execute(&error));
    CHECK(_SamePlayed(baseline, _Snapshot(*reader)));
}

static bool
_HasLine(const _ChainFrame &row, const std::string &line)
{
    return std::find(row.lines.begin(), row.lines.end(), line) !=
           row.lines.end();
}

// One chain case: the binary baked at the first frame and played through
// its inputs against the baked program (verbatim diagnostics), its first
// and last frames apart, then against the native evaluator, which must
// see exactly \p expected property values published over the frames.
static bool
_RunChainCase(const char *name, UsdStageRefPtr (*build)(),
              const std::vector<double> &frames, size_t expected,
              std::vector<_ChainFrame> *rows)
{
    const int failuresBefore = failures;
    std::vector<_PlayedFrame> played;
    _TestStage(name, build(), frames, &played);
    if (frames.size() > 1) {
        _CheckFramesDiffer(name, played);
    }
    CHECK(_ChainsMatchLive(name, build(), frames, rows));
    // These rigs compute nothing but property chains.
    size_t published = 0;
    for (const _ChainFrame &row : *rows) {
        published += row.values.size();
    }
    CHECK(published == expected);
    CHECK(published > 0);
    CHECK(rows->size() == frames.size());
    std::printf("%s: %zu property value(s) equal the native evaluator's\n",
                name, published);
    return failures == failuresBefore && rows->size() == frames.size();
}

static void
TestFloatChainCurveClamp()
{
    const std::vector<double> frames = {1, 2, 3, 4, 5, 6, 7, 8, 9, 10};
    std::vector<_ChainFrame> rows;
    // Two chains, each published at every frame.
    if (!_RunChainCase("float chain, curve then clamp",
                       _FloatCurveClampStage, frames, 2 * frames.size(),
                       &rows)) {
        return;
    }
    // The evaluator's own kernels over the authored base: the Hermite
    // curve, then the clamp; and the twelve linear keys.
    const UsdStageRefPtr stage = _FloatCurveClampStage();
    const UsdAttribute amount = stage->GetAttributeAtPath(
        SdfPath("/Asset/Rig/Channels/Dial.rigExec:amount"));
    const UsdAttribute ramp = stage->GetAttributeAtPath(
        SdfPath("/Asset/Rig/Channels/Ramp.rigExec:amount"));
    const UsdAttribute steps = stage->GetAttributeAtPath(
        SdfPath("/Asset/Rig/Movers/Staircase.inputs:keys"));
    VtArray<GfVec2f> staircase;
    CHECK(steps.Get(&staircase));
    size_t low = 0, high = 0, inside = 0;
    for (size_t f = 0; f < frames.size(); ++f) {
        float base = 0.0f;
        CHECK(amount.Get(&base, UsdTimeCode(frames[f])));
        RigExecPropertyMathParams<float> curve;
        curve.op = RigExecPropertyOp::Curve;
        curve.keys = kShapeKeys;
        curve.keyCount = 3;
        curve.tangents = kShapeTangents;
        curve.tangentCount = 3;
        RigExecPropertyMathParams<float> clamp;
        clamp.op = RigExecPropertyOp::Clamp;
        clamp.min = 0.1f;
        clamp.max = 1.0f;
        const float expected = RigExecApplyFloatMath(
            RigExecApplyFloatMath(base, curve), clamp);
        bool found = false;
        const float got =
            _Float(rows[f], "/Asset/Rig/Channels/Dial.rigExec:amount", &found);
        CHECK(found && std::memcmp(&got, &expected, sizeof(float)) == 0);
        low += got == 0.1f;
        high += got == 1.0f;
        inside += got > 0.1f && got < 1.0f;

        float x = 0.0f;
        CHECK(ramp.Get(&x, UsdTimeCode(frames[f])));
        const float linear = RigExecEvaluateLinearKeys(
            staircase.cdata(), staircase.size(), x);
        const float gotLinear =
            _Float(rows[f], "/Asset/Rig/Channels/Ramp.rigExec:amount", &found);
        CHECK(found && std::memcmp(&gotLinear, &linear, sizeof(float)) == 0);
    }
    // The clamp holds both bounds at some frames and neither at others.
    CHECK(low > 0 && high > 0 && inside > 0);
}

static void
TestDoubleChainInFloat()
{
    const std::vector<double> frames = {1, 2, 3, 4, 5, 6, 7, 8, 9, 10};
    std::vector<_ChainFrame> rows;
    if (!_RunChainCase("double chain computed in float", _DoubleChainStage,
                       frames, frames.size(), &rows)) {
        return;
    }
    const UsdStageRefPtr stage = _DoubleChainStage();
    const UsdAttribute level = stage->GetAttributeAtPath(
        SdfPath("/Asset/Rig/Channels/Wide.rigExec:level"));
    size_t narrowed = 0;
    for (size_t f = 0; f < frames.size(); ++f) {
        double base = 0.0;
        CHECK(level.Get(&base, UsdTimeCode(frames[f])));
        // Narrowed, multiplied in float, widened back.
        const double expected = double(float(base) * 1.1f);
        const auto it =
            rows[f].values.find("/Asset/Rig/Channels/Wide.rigExec:level");
        CHECK(it != rows[f].values.end() &&
              it->second.tag == RrPropertyValue::Tag::Double &&
              std::memcmp(&it->second.f64, &expected, sizeof(double)) == 0);
        narrowed += it != rows[f].values.end() &&
                    it->second.f64 != base * double(1.1f);
    }
    // Computing in double would have given other bits at some frames.
    CHECK(narrowed > 0);
    std::printf("double chain computed in float: %zu of %zu frame(s) differ "
                "from the double product\n",
                narrowed, frames.size());
}

static void
TestMatrixChain()
{
    const std::vector<double> frames = {1, 4, 7, 10};
    std::vector<_ChainFrame> rows;
    if (!_RunChainCase("matrix chain", _MatrixChainStage, frames,
                       frames.size(), &rows)) {
        return;
    }
    // The evaluator's kernels over the authored base: multiply under 0.5,
    // then blend under 0.25; the non-finite multiply passes through with its
    // line.
    const UsdStageRefPtr stage = _MatrixChainStage();
    const UsdAttribute local = stage->GetAttributeAtPath(
        SdfPath("/Asset/Rig/Channels/Space.rigExec:local"));
    const GfMatrix4d offset(GfRotation(GfVec3d(0, 0, 1), 30.0),
                            GfVec3d(0.0, 5.0, 0.0));
    const GfMatrix4d snap = GfMatrix4d(1.0).SetTranslate(GfVec3d(2, 2, 2));
    const std::string unusable =
        "diag /Asset/Rig/Movers/Bad: inputs unusable; revision passed through";
    for (size_t f = 0; f < frames.size(); ++f) {
        GfMatrix4d base;
        CHECK(local.Get(&base, UsdTimeCode(frames[f])));
        GfMatrix4d once, twice;
        CHECK(RigExecApplyMatrixMath(base, RigExecPropertyOp::Multiply,
                                     offset, 0.5f, &once));
        CHECK(RigExecApplyMatrixMath(once, RigExecPropertyOp::Blend, snap,
                                     0.25f, &twice));
        const auto it =
            rows[f].values.find("/Asset/Rig/Channels/Space.rigExec:local");
        CHECK(it != rows[f].values.end() &&
              _SamePropertyValue(VtValue(twice), it->second));
        CHECK(_HasLine(rows[f], unusable));
    }
}

static void
TestVec3fChain()
{
    const std::vector<double> frames = {1, 4, 7, 10};
    std::vector<_ChainFrame> rows;
    if (!_RunChainCase("vec3f chain", _Vec3fChainStage, frames,
                       frames.size(), &rows)) {
        return;
    }
    // Add, clamp, then remap under 0.5, component by component.
    const UsdStageRefPtr stage = _Vec3fChainStage();
    const UsdAttribute offset = stage->GetAttributeAtPath(
        SdfPath("/Asset/Rig/Channels/Vec.rigExec:offset"));
    size_t clamped = 0;
    for (size_t f = 0; f < frames.size(); ++f) {
        GfVec3f base;
        CHECK(offset.Get(&base, UsdTimeCode(frames[f])));
        RigExecPropertyMathParams<GfVec3f> lift, bound, spread;
        lift.op = RigExecPropertyOp::Add;
        lift.value = GfVec3f(0.0f, 2.0f, 0.0f);
        bound.op = RigExecPropertyOp::Clamp;
        bound.min = GfVec3f(-1.0f);
        bound.max = GfVec3f(1.0f, 4.5f, 1.0f);
        spread.op = RigExecPropertyOp::Remap;
        spread.min = GfVec3f(-2.0f);
        spread.max = GfVec3f(2.0f, 3.0f, 2.0f);
        spread.weight = 0.5f;
        const GfVec3f lifted = RigExecApplyVec3fMath(base, lift);
        const GfVec3f held = RigExecApplyVec3fMath(lifted, bound);
        const GfVec3f expected = RigExecApplyVec3fMath(held, spread);
        const auto it =
            rows[f].values.find("/Asset/Rig/Channels/Vec.rigExec:offset");
        CHECK(it != rows[f].values.end() &&
              _SamePropertyValue(VtValue(expected), it->second));
        clamped += held != lifted;
    }
    CHECK(clamped > 0);
}

static void
TestDisabledRevision()
{
    const std::vector<double> frames = {1, 2, 3, 4, 5, 6, 7, 8, 9, 10};
    std::vector<_ChainFrame> rows;
    if (!_RunChainCase("disabled revisions", _DisabledStage, frames,
                       frames.size(), &rows)) {
        return;
    }
    // Doubled where the toggle is on, passed through where it is off; the
    // add of 100 never applies.
    const UsdStageRefPtr stage = _DisabledStage();
    const UsdAttribute amount = stage->GetAttributeAtPath(
        SdfPath("/Asset/Rig/Channels/Dial.rigExec:amount"));
    const std::string off =
        "diag /Asset/Rig/Movers/Off: disabled; revision passed through";
    const std::string toggle =
        "diag /Asset/Rig/Movers/Off/Toggle: disabled; revision passed "
        "through";
    size_t passed = 0;
    for (size_t f = 0; f < frames.size(); ++f) {
        float base = 0.0f;
        CHECK(amount.Get(&base, UsdTimeCode(frames[f])));
        const bool on = frames[f] < 4.0 || frames[f] >= 8.0;
        bool found = false;
        const float got =
            _Float(rows[f], "/Asset/Rig/Channels/Dial.rigExec:amount", &found);
        CHECK(found && got == (on ? base * 2.0f : base));
        CHECK(_HasLine(rows[f], off));
        CHECK(_HasLine(rows[f], toggle) == !on);
        passed += !on;
    }
    CHECK(passed == 4);
}

static void
TestEnvelopeRevisions()
{
    const std::vector<double> frames = {1, 2, 3, 4, 5, 6, 7, 8, 9, 10};
    std::vector<_ChainFrame> rows;
    if (!_RunChainCase("envelope-weighted revisions", _EnvelopeChainStage,
                       frames, frames.size(), &rows)) {
        return;
    }
    // The strict weight leaves [0, 1] from frame 8 (ty 7/3 times 0.45);
    // the out-of-range defaultWeights refuse at every frame.
    const std::string strict =
        "diag /Asset/Rig/Movers/Over/Soft/Kick: strict range violation on "
        "/Asset/Rig/Weights/Steep; revision passed through";
    const auto outOfRange = [](const std::string &mover) {
        return "diag /Asset/Rig/Movers/" + mover +
               ": inputs:defaultWeight must be finite and in [0, 1]; "
               "revision passed through";
    };
    const std::string over = outOfRange("Over");
    const std::string negative =
        outOfRange("Over/Soft/Kick/Nudge/Fade/Negative");
    const std::string notANumber =
        outOfRange("Over/Soft/Kick/Nudge/Fade/Negative/NotANumber");
    size_t refused = 0;
    for (size_t f = 0; f < frames.size(); ++f) {
        CHECK(_HasLine(rows[f], over));
        CHECK(_HasLine(rows[f], negative));
        CHECK(_HasLine(rows[f], notANumber));
        CHECK(_HasLine(rows[f], strict) == (frames[f] >= 8.0));
        refused += _HasLine(rows[f], strict);
    }
    CHECK(refused == 3);
}

static void
TestPhasedConsumers()
{
    const std::vector<double> frames = {1, 4, 7, 10};
    std::vector<_ChainFrame> rows;
    // Per frame: the two chains, three readout chains, three consumers.
    if (!_RunChainCase("phased consumers", _PhasedStage, frames,
                       8 * frames.size(), &rows)) {
        return;
    }
    const UsdStageRefPtr stage = _PhasedStage();
    const UsdAttribute amount = stage->GetAttributeAtPath(
        SdfPath("/Asset/Rig/Channels/Dial.rigExec:amount"));
    const UsdAttribute level = stage->GetAttributeAtPath(
        SdfPath("/Asset/Rig/Channels/Wide.rigExec:level"));
    for (size_t f = 0; f < frames.size(); ++f) {
        float base = 0.0f;
        double wide = 0.0;
        CHECK(amount.Get(&base, UsdTimeCode(frames[f])));
        CHECK(level.Get(&wide, UsdTimeCode(frames[f])));
        bool found = false;
        CHECK(_Float(rows[f], "/Asset/Rig/Movers/Readouts/base.inputs:value",
                     &found) == base &&
              found);
        CHECK(_Float(rows[f],
                     "/Asset/Rig/Movers/Readouts/early.inputs:value",
                     &found) == base * 2.0f &&
              found);
        // The double chain's base, narrowed into the float input.
        CHECK(_Float(rows[f],
                     "/Asset/Rig/Movers/Readouts/narrow.inputs:value",
                     &found) == float(wide) &&
              found);
        CHECK(_Float(rows[f], "/Asset/Rig/Channels/Dial.rigExec:amount",
                     &found) == base * 2.0f + 0.25f &&
              found);
    }
}

// The default read phase through the .rigexec: the runtime replays each
// reader at its own phase, bit for bit with the baked program and the
// native evaluator, and the file's phased_consumers carry an entry per
// reader the overlay walk alone would answer differently -- applied 0 for
// the undeclared ones, every revision for the `final` behind a recorded
// hop.
static void
TestDefaultReadPhaseRoundTrip()
{
    const std::vector<double> frames = {1, 4, 7, 10};
    std::vector<_ChainFrame> rows;
    // Per frame: the dial's chain, six readout chains, four records.
    if (!_RunChainCase("default read phase", _DefaultPhaseStage, frames,
                       11 * frames.size(), &rows)) {
        return;
    }
    const UsdStageRefPtr stage = _DefaultPhaseStage();
    const UsdAttribute amount = stage->GetAttributeAtPath(
        SdfPath("/Asset/Rig/Channels/Dial.rigExec:amount"));
    const auto readout = [&](size_t f, const char *name) {
        bool found = false;
        const float value =
            _Float(rows[f],
                   std::string("/Asset/Rig/Channels/Readouts.rigExec:") + name,
                   &found);
        CHECK(found);
        return value;
    };
    const auto consumer = [&](size_t f, const char *name, bool *found) {
        return _Float(rows[f],
                      std::string("/Asset/Rig/Movers/Readouts/") + name +
                          ".inputs:value",
                      found);
    };
    size_t clamped = 0;
    for (size_t f = 0; f < frames.size(); ++f) {
        float base = 0.0f;
        CHECK(amount.Get(&base, UsdTimeCode(frames[f])));
        const float final = std::min(base * 2.0f, 0.6f);
        clamped += final != base * 2.0f;
        CHECK(readout(f, "undeclared") == base);
        CHECK(readout(f, "final") == final);
        CHECK(readout(f, "hop") == base);
        CHECK(readout(f, "finalViaHop") == final);
        CHECK(readout(f, "finalHop") == final);
        CHECK(readout(f, "baseViaFinalHop") == base);
        // What the records publish on the readers themselves; none on the
        // `final` readers the dial's value answers.
        bool found = false;
        CHECK(consumer(f, "undeclared", &found) == base && found);
        CHECK(consumer(f, "finalViaHop", &found) == final && found);
        CHECK(consumer(f, "baseViaFinalHop", &found) == base && found);
        consumer(f, "final", &found);
        CHECK(!found);
        consumer(f, "finalHop", &found);
        CHECK(!found);
    }
    CHECK(clamped > 0 && clamped < frames.size());

    // The round trip: the records the file carries.
    RigExecRigEvaluator evaluator(stage, SdfPath("/Asset/Rig"));
    RigExecBakeOpts opts;
    opts.time = frames.front();
    RigExecBakeResult result;
    std::string error;
    CHECK(RigExecBakeToBinary(evaluator, opts, &result, &error));
    const std::unique_ptr<fb::RigExecWireFile> file =
        RigExecTestUnpack(result.bytes);
    if (!file) {
        return;
    }
    std::map<std::string, uint32_t> applied;
    for (const fb::RigExecWirePhasedConsumer &record :
         file->phasedConsumers) {
        CHECK(record.consumer < file->inputs.size());
        if (record.consumer < file->inputs.size()) {
            applied[RigExecFormatPathText(
                *file, file->inputs[record.consumer].name())] =
                record.applied;
        }
    }
    const std::string readers = "/Asset/Rig/Movers/Readouts/";
    CHECK(applied.size() == 4);
    CHECK(applied.count(readers + "undeclared.inputs:value") &&
          applied[readers + "undeclared.inputs:value"] == 0);
    CHECK(applied.count(readers + "hop.inputs:value") &&
          applied[readers + "hop.inputs:value"] == 0);
    CHECK(applied.count(readers + "finalViaHop.inputs:value") &&
          applied[readers + "finalViaHop.inputs:value"] == 2);
    CHECK(applied.count(readers + "baseViaFinalHop.inputs:value") &&
          applied[readers + "baseViaFinalHop.inputs:value"] == 0);
    std::printf("default read phase: %zu record(s) round-tripped\n",
                applied.size());
}

static void
TestNonFiniteBase()
{
    const char *const name = "non-finite bases and results";
    // Bad's base is +inf at frame 3 and NaN at frame 5. The sampler hands
    // the reader the stage's own values, non-finite ones included, so one
    // bake at frame 1 plays all four frames. Bad publishes at frames 1 and
    // 7, Huge never, Blow at every frame.
    std::vector<_ChainFrame> rows;
    if (!_RunChainCase(name, _NonFiniteStage, {1, 3, 5, 7}, 6, &rows)) {
        return;
    }
    // A bake at frame 3 or 5 holds the non-finite base as its default.
    for (const double frame : {3.0, 5.0}) {
        std::vector<_ChainFrame> baked;
        if (!_RunChainCase(name, _NonFiniteStage, {frame}, 1, &baked)) {
            return;
        }
    }
    const std::string badPath = "/Asset/Rig/Channels/Bad.rigExec:amount";
    {
        // A host's set takes finite values only; the stage's +inf reaches
        // the reader through the sampler alone.
        const UsdStageRefPtr stage = _NonFiniteStage();
        RigExecRigEvaluator evaluator(stage, SdfPath("/Asset/Rig"));
        std::vector<uint8_t> bytes;
        std::string error;
        CHECK(RigExecTestBakeAt(evaluator, 1.0, &bytes, &error));
        RigExecTestPlayer player;
        CHECK(player.Open(bytes, stage, &error));
        error.clear();
        CHECK(!player->SetInput(badPath,
                                double(std::numeric_limits<float>::infinity()),
                                &error));
        CHECK(error == badPath + " takes finite values only");
        CHECK(player.Play(3.0, &error));
        size_t index = 0;
        CHECK(player->FindInput(badPath, &index));
        const RrInputValue &held = player->GetInputValue(index);
        CHECK(held.tag == RrInputTag::Float && std::isinf(held.f32));
    }
    const std::string hugePath = "/Asset/Rig/Channels/Huge.rigExec:level";
    const std::string blowPath = "/Asset/Rig/Channels/Blow.rigExec:amount";
    const std::string hugeLine =
        "property chain " + hugePath +
        ": authored base is not finite; chain skipped";
    const std::string infLine =
        "diag /Asset/Rig/Movers/Inf: inputs unusable; revision passed through";
    bool found = false;
    CHECK(_Float(rows[0], badPath, &found) == 1.5f && found);
    CHECK(!rows[1].values.count(badPath) && !rows[2].values.count(badPath));
    CHECK(_Float(rows[3], badPath, &found) == 1.25f && found);
    for (const _ChainFrame &row : rows) {
        CHECK(!row.values.count(hugePath));
        CHECK(_HasLine(row, hugeLine));
        CHECK(_HasLine(row, infLine));
    }
    const std::string notFinite =
        "property chain " + badPath +
        ": authored base is not finite; chain skipped";
    CHECK(!_HasLine(rows[0], notFinite) && _HasLine(rows[1], notFinite) &&
          _HasLine(rows[2], notFinite) && !_HasLine(rows[3], notFinite));
    // 0.5 and 1.0 times 3e38 stay finite; 1.5 and 2.0 overflow, and the
    // revision passes the base through.
    const std::string overflow =
        "diag /Asset/Rig/Movers/Inf/Overflow: produced a non-finite value; "
        "revision passed through";
    CHECK(_Float(rows[0], blowPath, &found) == 0.5f * 3e38f && found);
    CHECK(_Float(rows[1], blowPath, &found) == 1.0f * 3e38f && found);
    CHECK(_Float(rows[2], blowPath, &found) == 1.5f && found);
    CHECK(_Float(rows[3], blowPath, &found) == 2.0f && found);
    CHECK(!_HasLine(rows[0], overflow) && !_HasLine(rows[1], overflow) &&
          _HasLine(rows[2], overflow) && _HasLine(rows[3], overflow));
}

// tests/fixtures/computed_chains.usda: the chain cases together (a curve
// with tangents, envelopes on property revisions, double, vec3f and matrix
// chains, the chains' diagnostics, phased consumers) played through the
// inputs of a bake at the first frame, against the baked program and the
// native evaluator, 10 values per frame, the constraint weight Follow
// reads through the dial's chain at its declared `final` included.
static void
TestComputedChainsFixture()
{
    const char *const name = "computed chains fixture";
    const std::string fixture =
        (std::filesystem::path(RIGEXEC_EXAMPLES_DIR) / ".." / "tests" /
         "fixtures" / "computed_chains.usda")
            .string();
    const std::vector<double> frames = {1.0, 3.0, 5.0, 7.0, 10.0};
    const UsdStageRefPtr stage = UsdStage::Open(fixture);
    CHECK(stage);
    if (!stage) {
        return;
    }
    std::vector<_PlayedFrame> played;
    _TestStage(name, stage, frames, &played);
    _CheckFramesDiffer(name, played);
    std::vector<_ChainFrame> rows;
    CHECK(_ChainsMatchLive(name, stage, frames, &rows));
    size_t published = 0;
    for (const _ChainFrame &row : rows) {
        published += row.values.size();
    }
    CHECK(published == 10 * frames.size());
}

// tests/fixtures/computed_ik_space.usda: a TwoBoneIk measured in an
// animated rigExec:space composed with a keyed, non-uniform
// rigExec:spaceMatrix, beside one reading a constant spaceMatrix. The arm's
// spaceMatrix is a live read of the solver over an Animated input whose
// default is the keyed matrix at the bake time, and the runtime plays the
// fixture through its inputs bit for bit with the baked program.
static void
TestIkSpaceFixture()
{
    const char *const name = "ik space fixture";
    const std::string fixture =
        (std::filesystem::path(RIGEXEC_EXAMPLES_DIR) / ".." / "tests" /
         "fixtures" / "computed_ik_space.usda")
            .string();
    const std::vector<double> frames = {1.0, 3.0, 5.0, 7.0, 10.0};
    const UsdStageRefPtr stage = UsdStage::Open(fixture);
    CHECK(stage);
    if (!stage) {
        return;
    }
    std::vector<_PlayedFrame> played;
    _TestStage(name, stage, frames, &played);
    _CheckFramesDiffer(name, played);

    RigExecRigEvaluator evaluator(stage, SdfPath("/IkSpaceAsset/Rig"));
    std::vector<uint8_t> bytes;
    std::string error;
    CHECK(RigExecTestBakeAt(evaluator, frames.front(), &bytes, &error));
    const std::unique_ptr<fb::RigExecWireFile> file = RigExecTestUnpack(bytes);
    if (!file) {
        return;
    }
    const std::string spaceMatrix =
        "/IkSpaceAsset/Rig/Solvers/ArmIK.rigExec:spaceMatrix";
    const int64_t slot = _InputOfPath(*file, spaceMatrix);
    CHECK(slot >= 0);
    if (slot < 0) {
        return;
    }
    // An Animated Matrix4d input whose default is not the identity, which
    // the arm's solver reads per run.
    const fb::InputSlot &input = file->inputs[size_t(slot)];
    CHECK(input.type() == fb::InputTag::Matrix4d);
    CHECK((input.flags() & uint8_t(fb::InputSlotFlags::Animated)) != 0);
    const fb::RigExecWireValue &held = file->values[input.value()];
    CHECK(held.matrix && (*held.matrix)[0] != 1.0);
    size_t readers = 0;
    for (const fb::RigExecWireSolver &solver : file->pose->solvers) {
        const fb::RigExecWireInput &read = *solver.ikSpace;
        if (!read.walk.empty() && read.walk[0] == uint32_t(slot)) {
            ++readers;
            CHECK(read.tag == fb::InputTag::Matrix4d);
            CHECK((read.flags & uint8_t(fb::InputReadFlags::Varying)) != 0);
        }
    }
    CHECK(readers == 1);

    // A constant spaceMatrix and no space keep the leg on the solver's
    // constant arm, whose bone lengths are measured in that matrix when the
    // rests are refreshed: the knee's rest is keyed, so the ladder varies
    // and they are refreshed every frame. The baked program publishes the
    // rest-length handles there, which the dynamic computation does not, so
    // this case is compared with the baked program only.
    const UsdStageRefPtr constant = UsdStage::Open(fixture);
    CHECK(constant);
    if (!constant) {
        return;
    }
    constant->SetEditTarget(constant->GetSessionLayer());
    const UsdPrim leg =
        constant->GetPrimAtPath(SdfPath("/IkSpaceAsset/Rig/Solvers/LegIK"));
    CHECK(leg);
    GfMatrix4d scale(1.0);
    scale.SetScale(GfVec3d(0.75, 0.75, 0.75));
    CHECK(leg.CreateAttribute(TfToken("rigExec:spaceMatrix"),
                              SdfValueTypeNames->Matrix4d)
              .Set(scale));
    const UsdAttribute kneeRest =
        constant->GetPrimAtPath(SdfPath("/IkSpaceAsset/Rig/Joints/Hip/Knee"))
            .GetAttribute(TfToken("rest:space"));
    CHECK(kneeRest);
    GfMatrix4d knee(1.0);
    knee.SetTranslate(GfVec3d(0.0, -2.5, 0.2));
    CHECK(kneeRest.Set(knee, UsdTimeCode(1.0)));
    knee.SetTranslate(GfVec3d(0.0, -2.0, 0.6));
    CHECK(kneeRest.Set(knee, UsdTimeCode(10.0)));
    played.clear();
    _TestStage("ik space fixture, constant leg space", constant, frames,
               &played);
    _CheckFramesDiffer("ik space fixture, constant leg space", played);
}

// The read phase on inputs outside the movers, replayed: the docs
// single-chain IK with its twist connected to a channel a math mover
// doubles (keyed 10 to 30 degrees), a control whose avars:ty reads a
// height channel a mover scales by 1.5 (keyed 2 to 4) -- a double channel,
// or a float one that only a record widens -- and the docs space switch
// reading its index from a channel outside the rig that reads a pick a
// mover adds 1 to. Undeclared each reads the channel's base, `final` the
// revised value (testRigExecArm checks which); here the runtime must match
// the baked program bit for bit at every frame either way.
static UsdStageRefPtr
_TwistReadStage(const std::string &examplesDir, bool final)
{
    const UsdStageRefPtr stage = UsdStage::Open(
        examplesDir + "/../docs/examples/single_chain_ik_constraint.usda");
    CHECK(stage);
    if (!stage) {
        return stage;
    }
    stage->SetEditTarget(stage->GetSessionLayer());
    const UsdAttribute degrees =
        stage->DefinePrim(SdfPath("/ScIkAsset/Rig/Channels/Twist"),
                          TfToken("Scope"))
            .CreateAttribute(TfToken("rigExec:degrees"),
                             SdfValueTypeNames->Double);
    degrees.Set(10.0, UsdTimeCode(1001.0));
    degrees.Set(30.0, UsdTimeCode(1024.0));
    const UsdPrim twice = stage->DefinePrim(
        SdfPath("/ScIkAsset/Rig/Movers/TwistTwice"),
        TfToken("RigExecFloatMathMover"));
    CHECK(twice.ApplyAPI(TfToken("RigExecMoverAPI")));
    twice.CreateAttribute(TfToken("rigExec:operation"),
                          SdfValueTypeNames->Token)
        .Set(TfToken("multiply"));
    twice.CreateAttribute(TfToken("inputs:value"), SdfValueTypeNames->Float)
        .Set(2.0f);
    twice.CreateRelationship(TfToken("rigExec:moves"))
        .SetTargets({degrees.GetPath()});
    const UsdAttribute twist = stage->GetAttributeAtPath(
        SdfPath("/ScIkAsset/Rig/Movers/Pose/ArmIK.inputs:twistDegrees"));
    twist.SetConnections({degrees.GetPath()});
    if (final) {
        twist.SetMetadata(TfToken("rigExecReadPhase"), std::string("final"));
    }
    return stage;
}

static UsdStageRefPtr
_LiftReadStage(bool final, bool floatChannel = false)
{
    const UsdStageRefPtr stage = UsdStage::CreateInMemory();
    stage->SetStartTimeCode(1.0);
    stage->SetEndTimeCode(10.0);
    UsdGeomXform::Define(stage, SdfPath("/Asset"));
    stage->DefinePrim(SdfPath("/Asset/Rig"), TfToken("RigExecRoot"));
    const UsdPrim lift = stage->DefinePrim(
        SdfPath("/Asset/Rig/Controls/Lift"), TfToken("RigExecControl"));
    lift.CreateAttribute(TfToken("rest:space"), SdfValueTypeNames->Matrix4d)
        .Set(GfMatrix4d(1.0));
    const UsdAttribute height =
        stage->DefinePrim(SdfPath("/Asset/Rig/Channels/Lift"),
                          TfToken("Scope"))
            .CreateAttribute(TfToken("rigExec:height"),
                             floatChannel ? SdfValueTypeNames->Float
                                          : SdfValueTypeNames->Double);
    if (floatChannel) {
        height.Set(2.0f, UsdTimeCode(1.0));
        height.Set(4.0f, UsdTimeCode(10.0));
    } else {
        height.Set(2.0, UsdTimeCode(1.0));
        height.Set(4.0, UsdTimeCode(10.0));
    }
    const UsdPrim raise = _MathMover(stage, "Raise", "RigExecFloatMathMover",
                                     "multiply", height);
    _SetInput(raise, "inputs:value", SdfValueTypeNames->Float, 1.5f);
    const UsdAttribute ty = lift.CreateAttribute(TfToken("avars:ty"),
                                                 SdfValueTypeNames->Double);
    ty.SetConnections({height.GetPath()});
    if (final) {
        ty.SetMetadata(TfToken("rigExecReadPhase"), std::string("final"));
    }
    return stage;
}

static UsdStageRefPtr
_OutsideSpaceStage(const std::string &examplesDir, bool final)
{
    const UsdStageRefPtr stage =
        UsdStage::Open(examplesDir + "/../docs/examples/space_switch.usda");
    CHECK(stage);
    if (!stage) {
        return stage;
    }
    stage->SetEditTarget(stage->GetSessionLayer());
    const UsdAttribute pick =
        stage->DefinePrim(SdfPath("/SpaceSwitchAsset/Rig/Channels"),
                          TfToken("Scope"))
            .CreateAttribute(TfToken("rigExec:pick"),
                             SdfValueTypeNames->Double);
    pick.Set(0.0);
    const UsdPrim next = stage->DefinePrim(
        SdfPath("/SpaceSwitchAsset/Rig/Movers/NextSpace"),
        TfToken("RigExecFloatMathMover"));
    CHECK(next.ApplyAPI(TfToken("RigExecMoverAPI")));
    next.CreateAttribute(TfToken("rigExec:operation"),
                         SdfValueTypeNames->Token)
        .Set(TfToken("add"));
    next.CreateAttribute(TfToken("inputs:value"), SdfValueTypeNames->Float)
        .Set(1.0f);
    next.CreateRelationship(TfToken("rigExec:moves"))
        .SetTargets({pick.GetPath()});
    const UsdAttribute space =
        stage->DefinePrim(SdfPath("/SpaceSwitchAsset/Dials"),
                          TfToken("Scope"))
            .CreateAttribute(TfToken("rigExec:space"),
                             SdfValueTypeNames->Double);
    space.SetConnections({pick.GetPath()});
    if (final) {
        space.SetMetadata(TfToken("rigExecReadPhase"), std::string("final"));
    }
    stage->GetPrimAtPath(SdfPath("/SpaceSwitchAsset/Rig/Spaces/HandSpaces"))
        .GetRelationship(TfToken("rigExec:activeSpaceAttribute"))
        .SetTargets({space.GetPath()});
    return stage;
}

static void
TestConstraintAndAvarReadersReplay(const std::string &examplesDir)
{
    for (const bool final : {false, true}) {
        const UsdStageRefPtr twist = _TwistReadStage(examplesDir, final);
        if (twist) {
            _TestStage(final ? "twist read final" : "twist read at its base",
                       twist, {1001.0, 1006.0, 1012.0, 1024.0});
        }
        _TestStage(final ? "avar read final" : "avar read at its base",
                   _LiftReadStage(final), {1.0, 4.0, 10.0});
        _TestStage(final ? "avar read final, float channel"
                         : "avar read at its base, float channel",
                   _LiftReadStage(final, true), {1.0, 4.0, 10.0});
        const UsdStageRefPtr space = _OutsideSpaceStage(examplesDir, final);
        if (space) {
            _TestStage(final ? "outside space index read final"
                             : "outside space index read at its base",
                       space, {1001.0, 1012.0, 1024.0, 1036.0});
        }
    }
}

// Whether every played frame carries \p line among its diagnostics.
static bool
_EveryFrameSays(const std::vector<_PlayedFrame> &played,
                const std::string &line)
{
    if (played.empty()) {
        return false;
    }
    for (const _PlayedFrame &frame : played) {
        if (std::find(frame.diagnostics.begin(), frame.diagnostics.end(),
                      line) == frame.diagnostics.end()) {
            return false;
        }
    }
    return true;
}

// A constraint's source or pole weights table with the wrong cardinality:
// the bake keeps the read's line beside the arrays, and the constraint step
// replays it where the baked one does, ahead of its pass-through. The
// played diagnostics match the baked program's verbatim (_TestStage), and
// carry the line on every frame.
static void
TestConstraintArrayDiagnostics(const std::string &examplesDir)
{
    const int failuresBefore = failures;
    {
        const UsdStageRefPtr stage = UsdStage::CreateInMemory();
        const auto xform = [&](const char *path,
                               const GfVec3d &translation) {
            const UsdGeomXform x = UsdGeomXform::Define(stage, SdfPath(path));
            x.MakeMatrixXform().Set(GfMatrix4d(
                GfRotation(GfVec3d(0, 0, 1), 0.0), translation));
        };
        xform("/Asset", GfVec3d(0));
        xform("/Asset/Target", GfVec3d(0));
        xform("/Asset/Source", GfVec3d(10, 0, 0));
        stage->DefinePrim(SdfPath("/Asset/Rig"), TfToken("RigExecRoot"));
        stage->DefinePrim(SdfPath("/Asset/Rig/Movers"), TfToken("Scope"));
        const UsdPrim pos =
            stage->DefinePrim(SdfPath("/Asset/Rig/Movers/Pos"),
                              TfToken("RigExecPositionConstraint"));
        CHECK(pos.ApplyAPI(TfToken("RigExecMoverAPI")));
        pos.CreateRelationship(TfToken("rigExec:moves"))
            .SetTargets({SdfPath("/Asset/Target")});
        pos.CreateRelationship(TfToken("rigExec:sources"))
            .SetTargets({SdfPath("/Asset/Source")});
        pos.CreateAttribute(TfToken("inputs:sourceWeights"),
                            SdfValueTypeNames->FloatArray)
            .Set(VtFloatArray{1, 2});
        std::vector<_PlayedFrame> played;
        _TestStage("constraint source weights cardinality", stage,
                   {1.0, 2.0}, &played);
        const bool said = _EveryFrameSays(
            played,
            "/Asset/Rig/Movers/Pos inputs:sourceWeights has 2 entries for "
            "1 sources");
        CHECK(said);
        if (!said) {
            std::printf("constraint source weights cardinality: the line "
                        "is not played\n");
        }
    }
    {
        const UsdStageRefPtr stage = UsdStage::Open(
            examplesDir + "/../docs/examples/single_chain_ik_constraint.usda");
        CHECK(stage);
        if (!stage) {
            return;
        }
        stage->SetEditTarget(stage->GetSessionLayer());
        stage->GetPrimAtPath(SdfPath("/ScIkAsset/Rig/Movers/Pose/ArmIK"))
            .CreateAttribute(TfToken("inputs:poleVectorWeights"),
                             SdfValueTypeNames->FloatArray)
            .Set(VtFloatArray{1, 1});
        std::vector<_PlayedFrame> played;
        _TestStage("constraint pole weights cardinality", stage,
                   {1001.0, 1012.0}, &played);
        const bool said = _EveryFrameSays(
            played,
            "/ScIkAsset/Rig/Movers/Pose/ArmIK inputs:poleVectorWeights has "
            "2 entries for 1 sources");
        CHECK(said);
        if (!said) {
            std::printf("constraint pole weights cardinality: the line is "
                        "not played\n");
        }
    }
    if (failures == failuresBefore) {
        std::printf("constraint array diagnostics: source and pole lines "
                    "replayed as the baked program replays them\n");
    }
}

// One evaluation of a drag's reference: the pose and the program's
// version pools.
struct _Reference {
    RigExecRigPose pose;
    std::vector<RigExecPointFrame> fin, base;
};

// A fresh native evaluator compiled on \p stage, handed
// \p edits in the session layer and evaluated at each of \p frames
// (RigExecTestEditedPoses, compiled after the edits with
// \p compileAfterEdits), with its version pools after each. The parity
// check holds the baked program to the native evaluator.
static std::vector<_Reference>
_References(const UsdStageRefPtr &stage, const SdfPath &rigPath,
            const std::vector<RigExecTestEdit> &edits,
            const std::vector<double> &frames,
            bool compileAfterEdits = false)
{
    std::vector<RigExecRigPose> poses;
    std::vector<_Reference> out(frames.size());
    std::string error;
    const bool ok = RigExecTestEditedPoses(
        stage, rigPath, edits,
        frames, &poses, &error,
        [&](const RigExecRigEvaluator &evaluator, size_t i) {
            const RigExecBakedProgram *program = evaluator.GetBakedProgram();
            if (program) {
                out[i].fin = program->GetStepGraph().fin;
                out[i].base = program->GetStepGraph().base;
            }
        },
        compileAfterEdits);
    CHECK(ok);
    if (!ok) {
        std::printf("reference: %s\n", error.c_str());
    }
    for (size_t i = 0; i < poses.size() && i < out.size(); ++i) {
        out[i].pose = std::move(poses[i]);
    }
    return out;
}

// \p value on the attribute at \p path, typed to it.
static RigExecTestEdit
_Edit(const UsdStageRefPtr &stage, const std::string &path, double value)
{
    return {SdfPath(path),
            RigExecTestTypedValue(stage->GetAttributeAtPath(SdfPath(path)),
                                  value)};
}

// Whether the runtime's version pools equal \p fin and \p base bit for
// bit, naming the first slot that differs.
static bool
_SamePools(const char *what, double frame, const RigExecRuntimeReader &reader,
           const std::vector<RigExecPointFrame> &wantFin,
           const std::vector<RigExecPointFrame> &wantBase)
{
    const std::vector<RrPointFrame> &fin = reader.GetFinFrames();
    const std::vector<RrPointFrame> &base = reader.GetBaseFrames();
    if (fin.size() != wantFin.size() || base.size() != wantBase.size()) {
        std::printf("%s frame %.17g: the version pools differ in size\n",
                    what, frame);
        return false;
    }
    for (size_t i = 0; i < fin.size(); ++i) {
        if (!_FramesEqual(wantFin[i], fin[i])) {
            std::printf("%s frame %.17g: fin[%zu] differs\n", what, frame,
                        i);
            return false;
        }
    }
    for (size_t i = 0; i < base.size(); ++i) {
        if (!_FramesEqual(wantBase[i], base[i])) {
            std::printf("%s frame %.17g: base[%zu] differs\n", what, frame,
                        i);
            return false;
        }
    }
    return true;
}

static bool
_SamePools(const char *what, double frame, const RigExecRuntimeReader &reader,
           const RigExecBakedProgramImpl &graph)
{
    return _SamePools(what, frame, reader, graph.fin, graph.base);
}

static bool
_SamePools(const char *what, double frame, const RigExecRuntimeReader &reader,
           const _Reference &reference)
{
    return _SamePools(what, frame, reader, reference.fin, reference.base);
}

// The control's point \p axis of landmark 0 in \p pose, NaN when the pose
// has no frame for it.
static double
_ControlAt(const RigExecRigPose &pose, const SdfPath &control, size_t axis)
{
    const auto found = pose.controlFrames.find(control);
    return found == pose.controlFrames.end() ? std::nan("")
                                             : found->second.points[0][axis];
}

// An input set on an avar that reads a chain at its base through a
// connection: Lift's avars:ty, connected to the height channel the Raise
// mover revises. The set is an authored value on the head of that
// connection, and the walk reads past it to the channel, as the evaluators
// do with 5.5 authored on the avar in the session layer: the reader does
// not stand aside, and the pose is the undragged one. Played through the
// inputs of a bake at frame 1 at frames 1 and 4, every version-pool frame
// equals the baked program's under the session edit bit for bit.
static void
TestSetInputOnPhasedReader()
{
    const char *const name = "set input on a phased reader";
    const UsdStageRefPtr stage = _LiftReadStage(false);
    const UsdPrim lift =
        stage->GetPrimAtPath(SdfPath("/Asset/Rig/Controls/Lift"));
    const UsdAttribute tx =
        lift.CreateAttribute(TfToken("avars:tx"), SdfValueTypeNames->Double);
    tx.Set(1.0);
    const UsdPrim shift =
        _MathMover(stage, "Shift", "RigExecFloatMathMover", "add", tx);
    _SetInput(shift, "inputs:value", SdfValueTypeNames->Float, 0.5f);

    const SdfPath rigPath("/Asset/Rig");
    const std::vector<double> frames = {1.0, 4.0};
    RigExecRigEvaluator evaluator(stage, rigPath);
    std::vector<uint8_t> bytes;
    std::string error;
    CHECK(RigExecTestBakeAt(evaluator, frames.front(), &bytes, &error));
    RigExecTestPlayer player;
    if (!player.Open(bytes, stage, &error)) {
        std::printf("%s: open: %s\n", name, error.c_str());
        CHECK(false);
        return;
    }
    const std::string ty = "/Asset/Rig/Controls/Lift.avars:ty";
    const double drag = 5.5;
    const std::vector<_Reference> undragged =
        _References(stage, rigPath, {}, frames);
    const std::vector<_Reference> dragged =
        _References(stage, rigPath, {_Edit(stage, ty, drag)}, frames);
    CHECK(player.Hold(ty, drag, &error));
    for (size_t f = 0; f < frames.size() && f < dragged.size(); ++f) {
        CHECK(player.Play(frames[f], &error));
        CHECK(_SamePools(name, frames[f], player.Reader(), dragged[f]));
        CHECK(_SamePools(name, frames[f], player.Reader(), undragged[f]));
        CHECK(_ControlAt(dragged[f].pose, lift.GetPath(), 1) != drag);
    }
    std::printf("%s: checked\n", name);
}

// Inputs set on avars partway along other readers' connections. Hop's
// avars:tx reads the Raise chain's target at its base and Lift's avars:ty
// reads that chain through Hop's tx, so both are phased readers; Reach's
// avars:tz reads Pin's avars:tx, which no chain revises. Set on Hop's tx,
// an authored value is shadowed by the readable channel upstream of it:
// Lift and Hop keep reading the channel, and both readers still publish.
// Set on Pin's tx, it reaches Reach. Before, while and after the sets
// stand, every version-pool frame equals the baked program's under the
// same values authored in the session layer, bit for bit, and the baked
// program agrees with the native evaluator.
static void
TestSetInputOnHop()
{
    const char *const name = "set input on a hop";
    const UsdStageRefPtr stage = _LiftReadStage(false);
    const UsdAttribute height = stage->GetAttributeAtPath(
        SdfPath("/Asset/Rig/Channels/Lift.rigExec:height"));
    const auto control = [&](const char *path) {
        const UsdPrim prim =
            stage->DefinePrim(SdfPath(path), TfToken("RigExecControl"));
        prim.CreateAttribute(TfToken("rest:space"),
                             SdfValueTypeNames->Matrix4d)
            .Set(GfMatrix4d(1.0));
        return prim;
    };
    const UsdPrim hop = control("/Asset/Rig/Controls/Hop");
    const UsdAttribute hopTx =
        hop.CreateAttribute(TfToken("avars:tx"), SdfValueTypeNames->Double);
    hopTx.SetConnections({height.GetPath()});
    const UsdPrim lift =
        stage->GetPrimAtPath(SdfPath("/Asset/Rig/Controls/Lift"));
    lift.GetAttribute(TfToken("avars:ty")).SetConnections({hopTx.GetPath()});
    const UsdPrim pin = control("/Asset/Rig/Controls/Pin");
    const UsdAttribute pinTx =
        pin.CreateAttribute(TfToken("avars:tx"), SdfValueTypeNames->Double);
    pinTx.Set(0.25);
    const UsdPrim reach = control("/Asset/Rig/Controls/Reach");
    reach.CreateAttribute(TfToken("avars:tz"), SdfValueTypeNames->Double)
        .SetConnections({pinTx.GetPath()});

    const SdfPath rigPath("/Asset/Rig");
    const std::vector<double> frames = {1.0, 4.0};
    RigExecRigEvaluator evaluator(stage, rigPath);
    std::vector<uint8_t> bytes;
    std::string error;
    CHECK(RigExecTestBakeAt(evaluator, frames.front(), &bytes, &error));
    RigExecTestPlayer player;
    if (!player.Open(bytes, stage, &error)) {
        std::printf("%s: open: %s\n", name, error.c_str());
        CHECK(false);
        return;
    }
    const std::string hopPath = hopTx.GetPath().GetString();
    const std::string pinPath = pinTx.GetPath().GetString();
    const std::string liftPath = "/Asset/Rig/Controls/Lift.avars:ty";
    // Each phased reader's record is published whatever stands.
    const auto published = [&](const std::string &path) {
        for (const RigExecRuntimePropertyValue &value :
             player->GetPropertyValues()) {
            if (value.path == path) {
                return true;
            }
        }
        return false;
    };

    // One pass over the frames against \p references.
    const auto pass = [&](const char *what, bool dragging,
                          const std::vector<_Reference> &references) {
        bool same = references.size() == frames.size();
        for (size_t f = 0; f < frames.size() && f < references.size();
             ++f) {
            const RigExecRigPose &pose = references[f].pose;
            if (!player.Play(frames[f], &error)) {
                std::printf("%s, %s frame %.17g: %s\n", name, what,
                            frames[f], error.c_str());
                CHECK(false);
                return false;
            }
            same = _SamePools(what, frames[f], player.Reader(),
                              references[f]) &&
                   same;
            // The set on the hop is shadowed upstream; the set on the
            // unrevised avar reaches its reader.
            CHECK(_ControlAt(pose, lift.GetPath(), 1) != 5.5 &&
                  _ControlAt(pose, lift.GetPath(), 1) ==
                      _ControlAt(pose, hop.GetPath(), 0));
            CHECK(_ControlAt(pose, reach.GetPath(), 2) ==
                  (dragging ? -0.75 : 0.25));
            CHECK(published(liftPath) && published(hopPath));
        }
        CHECK(same);
        return same;
    };
    const std::vector<_Reference> undragged =
        _References(stage, rigPath, {}, frames);
    CHECK(pass("undragged", false, undragged));
    CHECK(player.Hold(hopPath, 5.5, &error));
    CHECK(player.Hold(pinPath, -0.75, &error));
    CHECK(pass("dragged", true,
               _References(stage, rigPath,
                           {_Edit(stage, hopPath, 5.5),
                            _Edit(stage, pinPath, -0.75)},
                           frames)));
    player.ReleaseAll();
    CHECK(pass("released", false, undragged));
    std::printf("%s: checked\n", name);
}

// An input set on an avar a provider ladder reads: Ctl's rest:tx is
// connected to Pin's constant avars:tx, so no frame recomposes the ladder
// until a value is set on Pin's tx, and the run after its reset recomposes
// it once more to write the authored rest back. Every version-pool frame
// equals the baked program's under the same value authored in the session
// layer, bit for bit: unset, set, reset at the frame the set stood on, and
// reset; and the set moves the pools.
static void
TestSetInputOnLadderInput()
{
    const char *const name = "set input on a ladder input";
    const UsdStageRefPtr stage = UsdStage::CreateInMemory();
    stage->SetStartTimeCode(1.0);
    stage->SetEndTimeCode(4.0);
    UsdGeomXform::Define(stage, SdfPath("/Asset"));
    stage->DefinePrim(SdfPath("/Asset/Rig"), TfToken("RigExecRoot"));
    const auto control = [&](const char *path) {
        const UsdPrim prim =
            stage->DefinePrim(SdfPath(path), TfToken("RigExecControl"));
        prim.CreateAttribute(TfToken("rest:space"),
                             SdfValueTypeNames->Matrix4d)
            .Set(GfMatrix4d(1.0));
        return prim;
    };
    const UsdPrim pin = control("/Asset/Rig/Controls/Pin");
    const UsdAttribute pinTx =
        pin.CreateAttribute(TfToken("avars:tx"), SdfValueTypeNames->Double);
    pinTx.Set(0.25);
    const UsdPrim ctl = control("/Asset/Rig/Controls/Ctl");
    ctl.CreateAttribute(TfToken("rest:tx"), SdfValueTypeNames->Double)
        .SetConnections({pinTx.GetPath()});

    const SdfPath rigPath("/Asset/Rig");
    const std::vector<double> frames = {1.0, 4.0};
    RigExecRigEvaluator evaluator(stage, rigPath);
    std::vector<uint8_t> bytes;
    std::string error;
    CHECK(RigExecTestBakeAt(evaluator, frames.front(), &bytes, &error));
    RigExecTestPlayer player;
    if (!player.Open(bytes, stage, &error)) {
        std::printf("%s: open: %s\n", name, error.c_str());
        CHECK(false);
        return;
    }
    // One run at \p frame against \p reference: the pools into \p fin.
    const auto run = [&](const char *what, double frame,
                         const _Reference &reference,
                         std::vector<std::vector<RrPointFrame>> *fin) {
        if (!player.Play(frame, &error)) {
            std::printf("%s, %s frame %.17g: %s\n", name, what, frame,
                        error.c_str());
            CHECK(false);
            return false;
        }
        fin->push_back(player->GetFinFrames());
        return _SamePools(what, frame, player.Reader(), reference);
    };
    const auto pass = [&](const char *what,
                          const std::vector<_Reference> &references,
                          std::vector<std::vector<RrPointFrame>> *fin) {
        bool same = references.size() == frames.size();
        for (size_t f = 0; f < frames.size() && f < references.size();
             ++f) {
            same = run(what, frames[f], references[f], fin) && same;
        }
        return same;
    };
    const std::string pinPath = pinTx.GetPath().GetString();
    const std::vector<_Reference> unset =
        _References(stage, rigPath, {}, frames);
    std::vector<std::vector<RrPointFrame>> undragged, dragged, releasedHere,
        released;
    CHECK(pass("undragged", unset, &undragged));
    CHECK(player.Hold(pinPath, -0.75, &error));
    // Compiled with the value authored: a value edit of an avar a ladder
    // reads through a connection, taken by an evaluator compiled before
    // it, breaks the baked program's parity with the native evaluator.
    CHECK(pass("dragged",
               _References(stage, rigPath, {_Edit(stage, pinPath, -0.75)},
                           frames, true),
               &dragged));
    CHECK(dragged.size() == frames.size() &&
          undragged.size() == frames.size());
    for (size_t f = 0; f < dragged.size() && f < undragged.size(); ++f) {
        CHECK(dragged[f] != undragged[f]);
    }
    player.ReleaseAll();
    if (!unset.empty()) {
        CHECK(run("released in place", frames.back(), unset.back(),
                  &releasedHere));
    }
    CHECK(!releasedHere.empty() && releasedHere.back() == undragged.back());
    CHECK(pass("released", unset, &released));
    CHECK(released == undragged);
    std::printf("%s: checked\n", name);
}

// The runtime's property results against \p pose's published values, both
// ways, bit for bit.
static bool
_SamePropertyResults(const char *what, const RigExecRigPose &pose,
                     const RigExecRuntimeReader &reader)
{
    bool same = true;
    std::map<std::string, RrPropertyValue> got;
    for (const RigExecRuntimePropertyValue &value :
         reader.GetPropertyValues()) {
        got[value.path] = value.value;
        const auto found = pose.movedProperties.find(SdfPath(value.path));
        if (found == pose.movedProperties.end() ||
            !_SamePropertyValue(found->second, value.value)) {
            std::printf("%s: %s differs from the baked program\n", what,
                        value.path.c_str());
            same = false;
        }
    }
    for (const auto &[path, value] : pose.movedProperties) {
        if (_IsPropertyResult(value) && !got.count(path.GetString())) {
            std::printf("%s: the runtime publishes nothing at %s\n", what,
                        path.GetText());
            same = false;
        }
    }
    return same;
}

// An input set at a fixed frame on \p control's avars:tx, which a property
// chain's input reads through a connection, the chain's result at
// \p chainPath being what a constraint moving /Asset/Target weighs by.
// The frame does not change, so the set reaches the constraint only
// through the chain result it moves; baked with every step in its own
// cluster (RIGEXEC_BAKED_GRAIN_US=0), only the steps the input's own
// dirtying reaches rerun. Under the same value authored in the session
// layer the runtime's property results and version pools equal the baked
// program's bit for bit, set and reset, and the set moves the chain's
// result and the constrained transform.
static void
_SetChainInput(const std::string &name, const UsdStageRefPtr &stage,
               const SdfPath &control, const std::string &chainPath)
{
    const SdfPath rigPath("/Asset/Rig");
    const std::vector<double> frames = {1.0, 5.0};
    RigExecRigEvaluator evaluator(stage, rigPath);
    std::vector<uint8_t> bytes;
    std::string error;
    CHECK(RigExecTestBakeAt(evaluator, frames.front(), &bytes, &error));
    RigExecTestPlayer player;
    if (!player.Open(bytes, stage, &error)) {
        std::printf("%s: open: %s\n", name.c_str(), error.c_str());
        CHECK(false);
        return;
    }
    // One run at \p frame against \p reference: the property results and
    // pools; the chain's result and Target's transform out.
    const auto run = [&](const char *what, double frame,
                         const _Reference &reference, float *chainValue,
                         RrMat4d *target) {
        if (!player.Play(frame, &error)) {
            std::printf("%s, %s frame %.17g: %s\n", name.c_str(), what,
                        frame, error.c_str());
            CHECK(false);
            return false;
        }
        bool chainFound = false, targetFound = false;
        for (const RigExecRuntimePropertyValue &value :
             player->GetPropertyValues()) {
            if (value.path == chainPath) {
                *chainValue = value.value.f32;
                chainFound = true;
            }
        }
        for (const RigExecRuntimeProviderXform &x :
             player->GetProviderXforms()) {
            if (x.path == "/Asset/Target") {
                *target = x.matrix;
                targetFound = true;
            }
        }
        CHECK(chainFound && targetFound);
        const std::string label = name + ", " + what;
        const bool properties = _SamePropertyResults(
            label.c_str(), reference.pose, player.Reader());
        return _SamePools(label.c_str(), frame, player.Reader(),
                          reference) &&
               properties;
    };
    const std::string tx = control.GetString() + ".avars:tx";
    const std::vector<_Reference> unset =
        _References(stage, rigPath, {}, frames);
    const std::vector<_Reference> set =
        _References(stage, rigPath, {_Edit(stage, tx, 2.0)}, {frames.back()});
    if (unset.size() != frames.size() || set.size() != 1) {
        CHECK(false);
        return;
    }
    float undraggedChain = 0.0f, draggedChain = 0.0f, releasedChain = 0.0f;
    RrMat4d undraggedTarget(1.0), draggedTarget(1.0), releasedTarget(1.0);
    float ignoredChain = 0.0f;
    RrMat4d ignoredTarget(1.0);
    CHECK(run("first frame", frames.front(), unset.front(), &ignoredChain,
              &ignoredTarget));
    CHECK(run("unset", frames.back(), unset.back(), &undraggedChain,
              &undraggedTarget));
    CHECK(player.Hold(tx, 2.0, &error));
    CHECK(run("set", frames.back(), set.front(), &draggedChain,
              &draggedTarget));
    CHECK(draggedChain != undraggedChain);
    CHECK(draggedTarget != undraggedTarget);
    player.ReleaseAll();
    CHECK(run("reset", frames.back(), unset.back(), &releasedChain,
              &releasedTarget));
    CHECK(std::memcmp(&releasedChain, &undraggedChain, sizeof(float)) == 0);
    CHECK(releasedTarget == undraggedTarget);
    std::printf("%s: checked\n", name.c_str());
}

// A position constraint whose own inputs:defaultWeight a math mover
// multiplies by Controls/Knob.avars:tx through a connection. Nothing
// declares a read phase, so the runtime runs only what a frame dirties.
static UsdStageRefPtr
_RevisedWeightStage()
{
    const UsdStageRefPtr stage = _ChainBaseStage();
    UsdGeomXform::Define(stage, SdfPath("/Asset/Target"))
        .AddTransformOp()
        .Set(GfMatrix4d(1.0));
    UsdGeomXform::Define(stage, SdfPath("/Asset/Source"))
        .AddTransformOp()
        .Set(GfMatrix4d(1.0).SetTranslate(GfVec3d(10.0, 0.0, 0.0)));
    const UsdPrim knob = stage->DefinePrim(
        SdfPath("/Asset/Rig/Controls/Knob"), TfToken("RigExecControl"));
    knob.CreateAttribute(TfToken("rest:space"), SdfValueTypeNames->Matrix4d)
        .Set(GfMatrix4d(1.0));
    const UsdAttribute tx =
        knob.CreateAttribute(TfToken("avars:tx"), SdfValueTypeNames->Double);
    tx.Set(0.25, UsdTimeCode(1.0));
    tx.Set(0.75, UsdTimeCode(10.0));
    const UsdPrim follow =
        stage->DefinePrim(SdfPath("/Asset/Rig/Movers/Follow"),
                          TfToken("RigExecPositionConstraint"));
    CHECK(follow.ApplyAPI(TfToken("RigExecMoverAPI")));
    const UsdAttribute weight = follow.CreateAttribute(
        TfToken("inputs:defaultWeight"), SdfValueTypeNames->Float);
    weight.Set(0.5f);
    follow.CreateRelationship(TfToken("rigExec:moves"))
        .SetTargets({SdfPath("/Asset/Target")});
    follow.CreateRelationship(TfToken("rigExec:sources"))
        .SetTargets({SdfPath("/Asset/Source")});
    const UsdPrim gain =
        _MathMover(stage, "Gain", "RigExecFloatMathMover", "multiply", weight);
    gain.CreateAttribute(TfToken("inputs:value"), SdfValueTypeNames->Float)
        .SetConnections({tx.GetPath()});
    return stage;
}

// computed_chains' Gain multiplies the dial by Controls/Dial.avars:tx and
// Follow's weight reads the dial's result at its declared `final`; and the
// rig above, where nothing declares a phase.
static void
TestSetInputOnChainInput()
{
    const std::string fixture =
        (std::filesystem::path(RIGEXEC_EXAMPLES_DIR) / ".." / "tests" /
         "fixtures" / "computed_chains.usda")
            .string();
    const UsdStageRefPtr stage = UsdStage::Open(fixture);
    CHECK(stage);
    if (stage) {
        _SetChainInput("set input on a chain input", stage,
                       SdfPath("/Asset/Rig/Controls/Dial"),
                       "/Asset/Rig/Channels/Dial.rigExec:amount");
    }
    _SetChainInput("set input on a revised weight's input",
                   _RevisedWeightStage(), SdfPath("/Asset/Rig/Controls/Knob"),
                   "/Asset/Rig/Movers/Follow.inputs:defaultWeight");
}

// The blink on a control avar: Dial.avars:tx authored at 0.2 (or
// \p authored), revised by Offset (add 0.1) and then Clamp ([0, 1]), and
// read by other controls' avars undeclared (its base), at Offset's
// checkpoint and at `final` (no record: the walk reads the dial's result),
// and by readout chains inside the chain loop, undeclared and `final`.
static UsdStageRefPtr
_DialTargetStage(double authored)
{
    const UsdStageRefPtr stage = _ChainBaseStage();
    const auto control = [&](const char *name) {
        const UsdPrim prim = stage->DefinePrim(
            SdfPath(std::string("/Asset/Rig/Controls/") + name),
            TfToken("RigExecControl"));
        prim.CreateAttribute(TfToken("rest:space"),
                             SdfValueTypeNames->Matrix4d)
            .Set(GfMatrix4d(1.0));
        return prim;
    };
    const UsdAttribute tx = control("Dial").CreateAttribute(
        TfToken("avars:tx"), SdfValueTypeNames->Double);
    tx.Set(authored);
    const UsdPrim clamp =
        _MathMover(stage, "Clamp", "RigExecFloatMathMover", "clamp", tx);
    _SetInput(clamp, "inputs:min", SdfValueTypeNames->Float, 0.0f);
    _SetInput(clamp, "inputs:max", SdfValueTypeNames->Float, 1.0f);
    const UsdPrim offset = _MathMover(stage, "Clamp/Offset",
                                      "RigExecFloatMathMover", "add", tx);
    _SetInput(offset, "inputs:value", SdfValueTypeNames->Float, 0.1f);
    const auto read = [&](const UsdAttribute &input, const char *phase) {
        input.SetConnections({tx.GetPath()});
        if (phase) {
            input.SetMetadata(TfToken("rigExecReadPhase"),
                              std::string(phase));
        }
    };
    read(control("BaseReader")
             .CreateAttribute(TfToken("avars:ty"), SdfValueTypeNames->Double),
         nullptr);
    read(control("CheckReader")
             .CreateAttribute(TfToken("avars:ty"), SdfValueTypeNames->Double),
         "/Asset/Rig/Movers/Clamp/Offset");
    read(control("FinalReader")
             .CreateAttribute(TfToken("avars:ty"), SdfValueTypeNames->Double),
         "final");
    for (const char *name : {"Base", "Final"}) {
        const UsdAttribute channel =
            _Channel(stage, "Readouts", name, SdfValueTypeNames->Float);
        channel.Set(0.0f);
        const UsdPrim mover =
            _MathMover(stage, std::string("Readouts/") + name,
                       "RigExecFloatMathMover", "add", channel);
        read(mover.CreateAttribute(TfToken("inputs:value"),
                                   SdfValueTypeNames->Float),
             std::string(name) == "Final" ? "final" : nullptr);
    }
    return stage;
}

// One played frame: the version pools, the property results and the
// diagnostics.
struct _PlayedPose {
    std::vector<RrPointFrame> fin, base;
    std::vector<RigExecRuntimePropertyValue> properties;
    std::vector<std::string> diagnostics;
};

static _PlayedPose
_Played(const RigExecRuntimeReader &reader)
{
    return {reader.GetFinFrames(), reader.GetBaseFrames(),
            reader.GetPropertyValues(), reader.GetDiagnostics()};
}

// The first thing \p a and \p b disagree on, or empty when they agree bit
// for bit.
static std::string
_PlayedDiffer(const _PlayedPose &a, const _PlayedPose &b)
{
    const auto samePool = [](const std::vector<RrPointFrame> &x,
                             const std::vector<RrPointFrame> &y) {
        if (x.size() != y.size()) {
            return false;
        }
        for (size_t i = 0; i < x.size(); ++i) {
            if (x[i].flags != y[i].flags ||
                std::memcmp(x[i].points.data(), y[i].points.data(),
                            sizeof(x[i].points)) != 0) {
                return false;
            }
        }
        return true;
    };
    if (!samePool(a.fin, b.fin)) {
        return "fin pool";
    }
    if (!samePool(a.base, b.base)) {
        return "base pool";
    }
    if (a.properties.size() != b.properties.size()) {
        return "property count";
    }
    for (size_t i = 0; i < a.properties.size(); ++i) {
        if (a.properties[i].path != b.properties[i].path ||
            !_SameBits(a.properties[i].value, b.properties[i].value)) {
            return "property " + a.properties[i].path;
        }
    }
    if (a.diagnostics != b.diagnostics) {
        return "diagnostics";
    }
    return std::string();
}

// An input set on an avar math movers revise is the chain's base, as the
// value authored there is in the USD evaluators. Set to 1.4, the dial's
// base readers read 1.4, Offset's checkpoint 1.5 and the final readers and
// the dial itself 1.0. At frames 1 and 4 every version-pool frame equals
// the baked program's under 1.4 authored in the session layer (which the
// parity check holds to the native evaluator), and the played pose --
// pools, property results and diagnostics -- equals bit for bit what a
// file baked with 1.4 authored on the dial plays. The same holds as the
// held value moves to 1.5 (the final holds at 1.0) and then to 0.5 without
// being reset. Reset, the input plays the original file again.
static void
TestSetInputOnChainTarget()
{
    const char *const name = "set input on a chain target";
    const SdfPath rigPath("/Asset/Rig");
    const std::string dialTx = "/Asset/Rig/Controls/Dial.avars:tx";
    const std::vector<double> frames = {1.0, 4.0};
    const double drag = 1.4;
    const auto bake = [&](const UsdStageRefPtr &stage,
                          std::vector<uint8_t> *bytes) {
        RigExecRigEvaluator evaluator(stage, rigPath);
        std::string error;
        const bool ok =
            RigExecTestBakeAt(evaluator, frames.front(), bytes, &error);
        if (!ok) {
            std::printf("%s: bake: %s\n", name, error.c_str());
        }
        return ok;
    };
    const auto play = [&](RigExecTestPlayer &player, double frame,
                          _PlayedPose *out) {
        std::string error;
        if (!player.Play(frame, &error)) {
            std::printf("%s frame %.17g: %s\n", name, frame, error.c_str());
            return false;
        }
        *out = _Played(player.Reader());
        return true;
    };

    const UsdStageRefPtr stage = _DialTargetStage(0.2);
    std::vector<uint8_t> original;
    CHECK(bake(stage, &original));
    std::string error;
    RigExecTestPlayer player;
    if (!player.Open(original, stage, &error)) {
        std::printf("%s: open: %s\n", name, error.c_str());
        CHECK(false);
        return;
    }
    std::vector<_PlayedPose> undragged(frames.size());
    for (size_t f = 0; f < frames.size(); ++f) {
        CHECK(play(player, frames[f], &undragged[f]));
    }

    // The set, then moved without a reset: 1.4 -> 1.5 leaves the clamped
    // final on 1.0 while the base and checkpoint readers move, and 1.5 ->
    // 0.5 moves every reader.
    struct HeldDrag {
        double drag, final, checkpoint;
    };
    const HeldDrag held[] = {
        {drag, 1.0, 1.5}, {1.5, 1.0, 1.6}, {0.5, 0.6, 0.6}};
    for (const HeldDrag &h : held) {
        char what[64];
        std::snprintf(what, sizeof(what), "chain target set to %.9g",
                      h.drag);
        CHECK(player.Hold(dialTx, h.drag, &error));
        const std::vector<_Reference> references = _References(
            stage, rigPath, {_Edit(stage, dialTx, h.drag)}, frames);
        if (references.size() != frames.size()) {
            CHECK(false);
            return;
        }
        std::vector<_PlayedPose> dragged(frames.size());
        for (size_t f = 0; f < frames.size(); ++f) {
            const RigExecRigPose &pose = references[f].pose;
            if (!play(player, frames[f], &dragged[f])) {
                CHECK(false);
                return;
            }
            CHECK(_SamePools(what, frames[f], player.Reader(),
                             references[f]));
            CHECK(!_PlayedDiffer(undragged[f], dragged[f]).empty());
            const auto ty = [&](const char *control) {
                return _ControlAt(
                    pose,
                    SdfPath(std::string("/Asset/Rig/Controls/") + control),
                    1);
            };
            const double dial =
                _ControlAt(pose, SdfPath("/Asset/Rig/Controls/Dial"), 0);
            CHECK(std::abs(dial - h.final) < 1e-6);
            CHECK(ty("BaseReader") == h.drag);
            CHECK(std::abs(ty("FinalReader") - h.final) < 1e-6);
            CHECK(std::abs(ty("CheckReader") - h.checkpoint) < 1e-6);
        }

        // The held value authored on the dial and baked again.
        stage->GetAttributeAtPath(SdfPath(dialTx)).Set(h.drag);
        std::vector<uint8_t> authored;
        CHECK(bake(stage, &authored));
        stage->GetAttributeAtPath(SdfPath(dialTx)).Set(0.2);
        RigExecTestPlayer released;
        if (!released.Open(authored, stage, &error)) {
            std::printf("%s: open released: %s\n", name, error.c_str());
            CHECK(false);
            return;
        }
        for (size_t f = 0; f < frames.size(); ++f) {
            _PlayedPose played;
            CHECK(play(released, frames[f], &played));
            const std::string why = _PlayedDiffer(dragged[f], played);
            if (!why.empty()) {
                std::printf("%s, frame %.17g: the set and the rebake "
                            "differ (%s)\n",
                            what, frames[f], why.c_str());
            }
            CHECK(why.empty());
        }
    }

    // Reset: the original file's poses again.
    player.ReleaseAll();
    for (size_t f = 0; f < frames.size(); ++f) {
        _PlayedPose played;
        CHECK(play(player, frames[f], &played));
        CHECK(_PlayedDiffer(undragged[f], played).empty());
    }
    std::printf("%s: checked\n", name);
}

// Open refuses a file whose registered reads do not bind the runtime's
// tables exactly, each with its own text naming the field: two avar
// bindings headed by one attribute; and, refused by the format's
// validator, a walk past the input slots, a constant of another type than
// the read's, an override number held twice, one held by no read, a read
// of a mode its table does not admit; a required read left out fails the
// FlatBuffers verifier. An input refuses a name the file does not list, a
// value of another type, a non-finite value and an index past the count;
// a chain target, which no avar override reached, takes a set like any
// input.
static void
TestRegisteredReadRefusals()
{
    const char *const name = "registered read refusals";
    const std::string fixture =
        (std::filesystem::path(RIGEXEC_EXAMPLES_DIR) / ".." / "tests" /
         "fixtures" / "computed_chains.usda")
            .string();
    const UsdStageRefPtr stage = UsdStage::Open(fixture);
    CHECK(stage);
    if (!stage) {
        return;
    }
    RigExecRigEvaluator evaluator(stage, SdfPath("/Asset/Rig"));
    RigExecBakeOpts opts;
    opts.time = 1.0;
    RigExecBakeResult result;
    std::string error;
    CHECK(RigExecBakeToBinary(evaluator, opts, &result, &error));
    CHECK(_OpenError(result.bytes).empty());
    const std::unique_ptr<fb::RigExecWireFile> file =
        RigExecTestUnpack(result.bytes);
    if (!file) {
        return;
    }
    const auto expect = [&](const char *what,
                            const std::vector<uint8_t> &bytes,
                            const std::string &text) {
        const std::string got = _OpenError(bytes);
        if (got != text) {
            std::printf("%s, %s: open said '%s', expected '%s'\n", name,
                        what, got.c_str(), text.c_str());
            CHECK(false);
        }
    };
    const std::vector<fb::RigExecWireAvarBinding> &bindings =
        file->pose->avarBindings;
    const auto varying = [](const fb::RigExecWireAvarBinding &binding) {
        return (binding.read->flags &
                uint8_t(fb::InputReadFlags::Varying)) != 0;
    };
    // A constant avar binding with an override number and a walk, and a
    // binding of another avar the runtime binds before it (a varying one,
    // or a constant one stored earlier).
    size_t constant = bindings.size(), other = bindings.size();
    for (size_t k = 0; k < bindings.size(); ++k) {
        const fb::RigExecWireInput &read = *bindings[k].read;
        if (!varying(bindings[k]) && read.overrideIndex >= 0 &&
            !read.walk.empty() && constant == bindings.size()) {
            constant = k;
        }
    }
    for (size_t k = 0; k < bindings.size() && constant < bindings.size();
         ++k) {
        if (k != constant && !bindings[k].read->walk.empty() &&
            bindings[k].flat != bindings[constant].flat &&
            (varying(bindings[k]) || k < constant) &&
            bindings[k].read->overrideIndex >= 0) {
            other = k;
            break;
        }
    }
    CHECK(constant < bindings.size() && other < bindings.size());
    if (constant >= bindings.size() || other >= bindings.size()) {
        return;
    }
    const std::string field =
        "pose.avar_bindings[" + std::to_string(constant) + "]";
    const std::string invalid = "invalid .rigexec: ";
    expect("two avars at one head",
           RigExecTestEdited(result.bytes,
                             [&](fb::RigExecWireFile *edited) {
                                 edited->pose->avarBindings[constant]
                                     .read->walk[0] =
                                     bindings[other].read->walk[0];
                             }),
           field + " heads two avars");
    expect("a walk past the slots",
           RigExecTestEdited(result.bytes,
                             [&](fb::RigExecWireFile *edited) {
                                 edited->pose->avarBindings[constant]
                                     .read->walk[0] =
                                     uint32_t(file->inputs.size());
                             }),
           invalid + field + ".read.walk[0]: slot " +
               std::to_string(file->inputs.size()) + " out of range (" +
               std::to_string(file->inputs.size()) + " slots)");
    {
        // A value of another tag than the binding's Double read.
        size_t another = file->values.size();
        for (size_t v = 0; v < file->values.size(); ++v) {
            if (file->values[v].tag != fb::InputTag::Double) {
                another = v;
                break;
            }
        }
        CHECK(another < file->values.size());
        expect("a constant of another type",
               RigExecTestEdited(result.bytes,
                                 [&](fb::RigExecWireFile *edited) {
                                     edited->pose->avarBindings[constant]
                                         .read->constant = uint32_t(another);
                                 }),
               invalid + field + ".read: constant " +
                   std::to_string(another) +
                   " is not a value of the read's tag");
    }
    {
        // The constant binding's override number given the other's: one
        // number held twice, its own held by none.
        const int32_t mine = bindings[constant].read->overrideIndex;
        const int32_t theirs = bindings[other].read->overrideIndex;
        const int32_t first = theirs < mine ? theirs + 1 : mine;
        expect("an override number held twice",
               RigExecTestEdited(result.bytes,
                                 [&](fb::RigExecWireFile *edited) {
                                     edited->pose->avarBindings[constant]
                                         .read->overrideIndex = theirs;
                                 }),
               invalid + "override number " + std::to_string(first) +
                   " is held by no read or by two");
        const uint32_t count = file->pose->overrideCount;
        expect("an override number held by no read",
               RigExecTestEdited(result.bytes,
                                 [&](fb::RigExecWireFile *edited) {
                                     edited->pose->avarBindings[constant]
                                         .read->overrideIndex = -1;
                                 }),
               invalid + "pose.override_count is " + std::to_string(count) +
                   " but " + std::to_string(count - 1) +
                   " reads carry override numbers");
    }
    expect("a read of another mode",
           RigExecTestEdited(result.bytes,
                             [&](fb::RigExecWireFile *edited) {
                                 edited->pose->avarBindings[constant]
                                     .read->mode = fb::ReadMode::Resolved;
                             }),
           invalid + field + ".read: read mode 1 is not admitted here");
    CHECK(!file->pose->ladders.empty());
    if (!file->pose->ladders.empty()) {
        expect("a required read left out",
               RigExecTestEdited(result.bytes,
                                 [](fb::RigExecWireFile *edited) {
                                     edited->pose->ladders[0].restSpace.reset();
                                 }),
               "malformed .rigexec: the FlatBuffers verifier refused it");
    }

    // An input refuses, changing nothing, a name the file does not list,
    // a value of another type, a non-finite value and an index past the
    // count.
    RigExecTestPlayer player;
    if (!player.Open(result.bytes, stage, &error)) {
        std::printf("%s: open: %s\n", name, error.c_str());
        CHECK(false);
        return;
    }
    const std::string amount = "/Asset/Rig/Channels/Dial.rigExec:amount";
    size_t index = 0;
    CHECK(player->FindInput(amount, &index));
    const RrInputValue before = player->GetInputValue(index);
    const auto unchanged = [&] {
        const RrInputValue &now = player->GetInputValue(index);
        return now.tag == before.tag &&
               std::memcmp(&now.f32, &before.f32, sizeof(float)) == 0;
    };
    error.clear();
    CHECK(!player->SetInput("/Asset/Rig/Controls/Dial.avars:none", 1.0,
                            &error));
    CHECK(error == "no input named /Asset/Rig/Controls/Dial.avars:none");
    RrInputValue wrong = before;
    wrong.tag = RrInputTag::Bool;
    error.clear();
    CHECK(!player->SetInputAt(index, wrong, &error));
    CHECK(error == amount + " is a float input, not a bool");
    error.clear();
    CHECK(!player->SetInput(amount, std::nan(""), &error));
    CHECK(error == amount + " takes finite values only");
    error.clear();
    CHECK(!player->SetInputAt(player->GetInputCount(), before, &error));
    CHECK(error == "no input at index " +
                       std::to_string(player->GetInputCount()) +
                       "; the file lists " +
                       std::to_string(player->GetInputCount()));
    CHECK(unchanged());

    // What only an avar override refused is an authored value now: the
    // chain target's base, set to 0.3, plays what the baked program plays
    // with 0.3 authored on it in the session layer.
    CHECK(player->SetInput(amount, 0.3, &error));
    CHECK(player.Play(1.0, &error));
    const std::vector<_Reference> set = _References(
        stage, SdfPath("/Asset/Rig"), {_Edit(stage, amount, 0.3)}, {1.0});
    CHECK(set.size() == 1);
    if (set.size() == 1) {
        CHECK(_SamePools(name, 1.0, player.Reader(), set[0]));
        CHECK(_SamePropertyResults(name, set[0].pose, player.Reader()));
    }
    std::printf("%s: checked\n", name);
}

// tests/fixtures/<file>.
static std::string
_FixturePath(const char *file)
{
    return (std::filesystem::path(RIGEXEC_EXAMPLES_DIR) / ".." / "tests" /
            "fixtures" / file)
        .string();
}

// The provider slot \p path names in \p file's slot inventory, or -1.
static int
_SlotOfPath(const fb::RigExecWireFile &file, const std::string &path)
{
    const std::vector<uint32_t> &paths = file.slotMeta->paths;
    for (size_t i = 0; i < paths.size(); ++i) {
        if (RigExecFormatPathText(file, paths[i]) == path) {
            return int(i);
        }
    }
    return -1;
}

static bool
_SameVersion(const fb::RigExecWireFrameVersion &read, int anchor,
             const std::vector<int32_t> &recompose)
{
    return read.context == -1 && read.anchor == anchor && read.recompose == recompose;
}

static bool
_SameCheckpoint(const fb::RigExecWireFile &file,const fb::RigExecWireFrameVersion &read,
                int anchor,const std::vector<int32_t> &recompose)
{
    if(read.context<0 || size_t(read.context)>=file.pose->spaceCheckpoints.size() ||
       read.anchor!=-1 || !read.recompose.empty())return false;
    const auto &checkpoint=file.pose->spaceCheckpoints[size_t(read.context)];
    return checkpoint.anchor==anchor && checkpoint.recompose==recompose;
}

// Space switches nested under switched controls, read at the versions the
// program bound at Build: each fixture's version pools, joints and
// diagnostics equal the baked program's bit for bit at every frame, played
// through the inputs of a bake at the first frame. In the nested fixture S
// reads P recomposed from its avars before P's switch, so its parent
// checkpoint has {-1, {P}}. In the carry fixture P hangs under G and S's space
// Q under P, so S reads {G, {P}} for its parent and {G, {P, Q}} for its
// space. The dial fixture's indices are keyed with no default: each is an
// Animated input whose default is its key at the bake time, which its
// switch reads per run.
#include "rigExecConnectedBridgeFixture.h"
static void TestConnectedBridgeRefreshExport()
{
    const auto fixture=RigExecMakeConnectedBridgeFixture(
        [](const UsdStageRefPtr &stage,const SdfPath &path,const GfMatrix4d &matrix) {
            const auto x=UsdGeomXform::Define(stage,path);x.MakeMatrixXform().Set(matrix);return x.GetPrim();
        },
        [](const UsdStageRefPtr &stage,const char *name,const char *type,const SdfPathVector &targets) {
            const auto p=stage->DefinePrim(SdfPath(std::string("/Asset/Rig/Movers/")+name),TfToken(type));
            CHECK(p.ApplyAPI(TfToken("RigExecMoverAPI")));CHECK(p.CreateRelationship(TfToken("rigExec:moves")).SetTargets(targets));return p;
        });
    const auto value=fixture.stage->DefinePrim(SdfPath("/Asset/Rig/Channels/Independent"),TfToken("Scope"))
        .CreateAttribute(TfToken("value"),SdfValueTypeNames->Float);
    CHECK(value.Set(2.0f));
    const auto add=_MathMover(fixture.stage,"IndependentAdd","RigExecFloatMathMover","add",value);
    CHECK(add.CreateAttribute(TfToken("inputs:value"),SdfValueTypeNames->Float).Set(3.0f));
    _TestStage("connected Bridge refresh",fixture.stage,{1.0,1.0});
    RigExecRigEvaluator evaluator(fixture.stage,SdfPath("/Asset/Rig"));
    RigExecTestPlayer player;std::vector<uint8_t> bytes;std::string error;
    CHECK(RigExecTestBakeAt(evaluator,1,&bytes,&error));
    const bool opened=player.Open(bytes,fixture.stage,&error);CHECK(opened);
    if(!opened)return;
    CHECK(player.Play(1,&error));
    bool sawIndependent=false;
    for(const auto &published:player.Reader().GetPropertyValues())if(published.path==value.GetPath().GetString()) {
        sawIndependent=true;CHECK(published.value.tag==RrPropertyValue::Tag::Float && published.value.f32==5.0f);
    }
    CHECK(sawIndependent);
    const auto file=RigExecTestUnpack(bytes);CHECK(file);
    if(!file)return;
    const auto literal=[&](const UsdPrim &prim,double base,double final) {
        int slot=-1;for(size_t i=0;i<file->slotMeta->paths.size();++i)if(RigExecFormatPathText(*file,file->slotMeta->paths[i])==prim.GetPath().GetString())slot=int(i);
        CHECK(slot>=0);if(slot<0)return;
        // Every checked provider has a writer in both domains; the final
        // canonical SSA version is distinct from its initial slot seed.
        const size_t version=file->slotMeta->paths.size()+size_t(slot);
        CHECK(version<player.Reader().GetBaseFrames().size() && version<player.Reader().GetFinFrames().size());
        if(version>=player.Reader().GetBaseFrames().size() || version>=player.Reader().GetFinFrames().size())return;
        const auto &b=player.Reader().GetBaseFrames()[version],&f=player.Reader().GetFinFrames()[version];
        CHECK(std::abs(b.points[0][0]-base)<1e-9 && std::abs(b.points[0][1])<1e-9 && std::abs(b.points[0][2])<1e-9);
        CHECK(std::abs(f.points[0][0]-final)<1e-9 && std::abs(f.points[0][1])<1e-9 && std::abs(f.points[0][2])<1e-9);
    };
    literal(fixture.source,4,5);literal(fixture.rider,5,6);literal(fixture.held,6,9);literal(fixture.owner,7,7);
}

static void
TestSpaceSwitchVersionFixtures()
{
    const std::vector<double> frames = {0.0, 1.0, 2.0, 3.0};
    for (const char *file :
         {"space_switch_nested.usda", "space_switch_same_round.usda",
          "space_switch_dial.usda", "space_switch_carry.usda"}) {
        _TestFixture(file, _FixturePath(file), frames);
    }

    const auto bake = [&](const char *file, RigExecBakeResult *result) {
        const UsdStageRefPtr stage = UsdStage::Open(_FixturePath(file));
        CHECK(stage);
        if (!stage) {
            return false;
        }
        RigExecRigEvaluator evaluator(stage, SdfPath("/Rig"));
        RigExecBakeOpts opts;
        opts.time = frames.front();
        std::string error;
        const bool ok = RigExecBakeToBinary(evaluator, opts, result, &error);
        CHECK(ok);
        return ok;
    };

    RigExecBakeResult nested;
    const std::unique_ptr<fb::RigExecWireFile> nestedFile =
        bake("space_switch_nested.usda", &nested)
            ? RigExecTestUnpack(nested.bytes)
            : nullptr;
    if (nestedFile) {
        const int p = _SlotOfPath(*nestedFile, "/Rig/Controls/P");
        const int s = _SlotOfPath(*nestedFile, "/Rig/Controls/P/S");
        const int c = _SlotOfPath(*nestedFile, "/Rig/Controls/P/S/C");
        const std::vector<fb::RigExecWireSpaceSwitch> &switches =
            nestedFile->pose->spaceSwitches;
        CHECK(p >= 0 && s >= 0 && c >= 0);
        CHECK(switches.size() == 2);
        for (const fb::RigExecWireSpaceSwitch &sw : switches) {
            if (sw.slot == s) {
                // P resolves after S: S reads it before its switch.
                CHECK(_SameCheckpoint(*nestedFile,*sw.parentRead,-1,{p}));
            } else {
                // C, under S, resolves before P: its last version.
                CHECK(sw.slot == p && sw.sourceReads.size() == 2 &&
                      _SameVersion(sw.sourceReads[0], c, {}));
            }
        }
    }

    RigExecBakeResult carry;
    const std::unique_ptr<fb::RigExecWireFile> carryFile =
        bake("space_switch_carry.usda", &carry)
            ? RigExecTestUnpack(carry.bytes)
            : nullptr;
    if (carryFile) {
        const auto slot = [&](const char *path) {
            return _SlotOfPath(*carryFile, path);
        };
        const int other = slot("/Rig/Controls/Other");
        const int g = slot("/Rig/Controls/G");
        const int p = slot("/Rig/Controls/G/P");
        const int q = slot("/Rig/Controls/G/P/Q");
        const int s = slot("/Rig/Controls/G/P/S");
        const int c = slot("/Rig/Controls/G/P/S/C");
        const std::vector<fb::RigExecWireSpaceSwitch> &switches =
            carryFile->pose->spaceSwitches;
        CHECK(other >= 0 && g >= 0 && p >= 0 && q >= 0 && s >= 0 &&
              c >= 0);
        CHECK(switches.size() == 2);
        for (const fb::RigExecWireSpaceSwitch &sw : switches) {
            CHECK(sw.sourceReads.size() == 2);
            if (sw.sourceReads.size() != 2) {
                continue;
            }
            if (sw.slot == s) {
                // P resolves after S: P recomposed on G, then Q on that.
                CHECK(_SameCheckpoint(*carryFile,*sw.parentRead,g,{p}));
                CHECK(sw.spaceSlot == q &&
                      _SameCheckpoint(*carryFile,*sw.spaceRead,g,{p,q}));
                CHECK(_SameVersion(sw.sourceReads[0], other, {}));
                CHECK(_SameVersion(sw.sourceReads[1], -1, {}));
            } else {
                CHECK(sw.slot == p && _SameVersion(*sw.parentRead, g, {}) &&
                      _SameVersion(sw.sourceReads[0], c, {}) &&
                      _SameVersion(*sw.spaceRead, -1, {}));
            }
        }
    }

    RigExecBakeResult dial;
    if (!bake("space_switch_dial.usda", &dial)) {
        return;
    }
    const std::unique_ptr<fb::RigExecWireFile> dialFile =
        RigExecTestUnpack(dial.bytes);
    if (!dialFile) {
        return;
    }
    struct Dial {
        const char *path;
        std::array<double, 4> keys;
    };
    const std::vector<Dial> dials = {
        {"/Rig/Controls/P.spaces:active", {0.0, 1.0, 0.5, 0.0}},
        {"/Rig/Controls/P/S.spaces:active", {0.5, 0.0, 1.0, 0.5}}};
    for (const Dial &entry : dials) {
        // Live: an Animated Double input holding its key at the bake time,
        // which one switch's index reads per run.
        const int64_t slot = _InputOfPath(*dialFile, entry.path);
        CHECK(slot >= 0);
        if (slot < 0) {
            continue;
        }
        const fb::InputSlot &input = dialFile->inputs[size_t(slot)];
        CHECK(input.type() == fb::InputTag::Double);
        CHECK((input.flags() & uint8_t(fb::InputSlotFlags::Animated)) != 0);
        double value = 0.0;
        std::memcpy(&value, &dialFile->values[input.value()].bits,
                    sizeof(value));
        CHECK(value == entry.keys[0]);
        size_t readers = 0;
        for (const fb::RigExecWireSpaceSwitch &sw :
             dialFile->pose->spaceSwitches) {
            const fb::RigExecWireInput &read = *sw.active;
            if (std::find(read.walk.begin(), read.walk.end(),
                          uint32_t(slot)) != read.walk.end()) {
                ++readers;
                CHECK((read.flags & uint8_t(fb::InputReadFlags::Varying)) !=
                      0);
            }
        }
        CHECK(readers == 1);
    }
    std::printf("space switch version fixtures: checked\n");
}

// An ordinary switch fixture keeps its natural numerical parity across all
// frames. Inserting an inline recomposition into its serialized metadata is
// malformed: only a declared checkpoint operation may perform that math.
static void
TestHandBuiltSwitchReads()
{
    const char *const name = "hand-built switch reads";
    const UsdStageRefPtr stage = UsdStage::CreateInMemory();
    stage->SetStartTimeCode(1.0);
    stage->SetEndTimeCode(3.0);
    UsdGeomXform::Define(stage, SdfPath("/Asset"));
    stage->DefinePrim(SdfPath("/Asset/Rig"), TfToken("RigExecRoot"));
    const auto control = [&](const char *path, double degrees,
                             const GfVec3d &offset) {
        const UsdPrim prim =
            stage->DefinePrim(SdfPath(path), TfToken("RigExecControl"));
        GfMatrix4d rest(1.0);
        rest.SetRotate(GfRotation(GfVec3d(0, 0, 1), degrees));
        rest.SetTranslateOnly(offset);
        prim.CreateAttribute(TfToken("rest:space"),
                             SdfValueTypeNames->Matrix4d)
            .Set(rest);
        return prim;
    };
    const auto key = [](const UsdPrim &prim, const char *avar,
                        const std::array<double, 3> &values) {
        const UsdAttribute attribute = prim.CreateAttribute(
            TfToken(avar), SdfValueTypeNames->Double);
        for (size_t f = 0; f < values.size(); ++f) {
            attribute.Set(values[f], UsdTimeCode(double(f + 1)));
        }
    };
    const UsdPrim g =
        control("/Asset/Rig/Controls/G", 0.0, GfVec3d(0, 100, 0));
    const UsdPrim arm =
        control("/Asset/Rig/Controls/G/Arm", 0.0, GfVec3d(-20, 0.5, 0));
    const UsdPrim p =
        control("/Asset/Rig/Controls/G/P", 30.0, GfVec3d(10.5, 0.25, 0));
    const UsdPrim target =
        control("/Asset/Rig/Controls/G/P/S", 0.0, GfVec3d(5.5, 0, 0));
    control("/Asset/Rig/Controls/G/P/S/C", 0.0, GfVec3d(5, 0, 0));
    key(g, "avars:rz", {10.0, 25.0, -15.0});
    key(g, "avars:tx", {1.0, -2.0, 3.0});
    key(arm, "avars:tx", {0.0, 10.0, 20.0});
    key(arm, "avars:rz", {-5.0, 12.0, 40.0});
    key(p, "avars:rz", {5.0, -20.0, 30.0});
    key(target, "avars:rz", {7.0, -13.0, 21.0});
    key(target, "avars:ty", {0.3, -0.7, 1.1});
    const UsdPrim sw = stage->DefinePrim(
        SdfPath("/Asset/Rig/Movers/sSpaces"), TfToken("RigExecSpaceSwitch"));
    sw.CreateRelationship(TfToken("rigExec:target"))
        .SetTargets({target.GetPath()});
    sw.CreateRelationship(TfToken("rigExec:sources"))
        .SetTargets({arm.GetPath(), SdfPath("/Asset/Rig")});
    key(sw, "inputs:activeSpace", {0.0, 0.5, 0.25});

    const SdfPath rigPath("/Asset/Rig");
    const std::vector<double> frames = {1.0, 2.0, 3.0};
    RigExecRigEvaluator evaluator(stage, rigPath);
    RigExecBakeOpts opts;
    opts.time = frames.front();
    RigExecBakeResult result;
    std::string error;
    CHECK(RigExecBakeToBinary(evaluator, opts, &result, &error));
    const std::unique_ptr<fb::RigExecWireFile> file =
        RigExecTestUnpack(result.bytes);
    if (!file) {
        return;
    }
    const int pSlot = _SlotOfPath(*file, "/Asset/Rig/Controls/G/P");
    const int sSlot = _SlotOfPath(*file, "/Asset/Rig/Controls/G/P/S");
    const int armSlot = _SlotOfPath(*file, "/Asset/Rig/Controls/G/Arm");
    const int gSlot = _SlotOfPath(*file, "/Asset/Rig/Controls/G");
    const std::vector<fb::RigExecWireSpaceSwitch> &switches =
        file->pose->spaceSwitches;
    const size_t slotCount = file->slotMeta->paths.size();
    CHECK(pSlot >= 0 && sSlot >= 0 && armSlot >= 0 && gSlot >= 0);
    CHECK(switches.size() == 1);
    if (pSlot < 0 || sSlot < 0 || armSlot < 0 || gSlot < 0 ||
        switches.size() != 1 ||
        switches[0].slot != sSlot || switches[0].sourceReads.size() != 2) {
        std::printf("%s: FAILED (no switch on S)\n", name);
        CHECK(false);
        return;
    }
    // Nothing above S is switched: the program reads last versions.
    CHECK(_SameVersion(*switches[0].parentRead, pSlot, {}));
    CHECK(_SameVersion(switches[0].sourceReads[0], armSlot, {}));

    _TestStage(name,stage,frames);
    enum class Rewrite { Parent, Source, AnchoredParent };
    for(const Rewrite rewrite:{Rewrite::Parent,Rewrite::Source,Rewrite::AnchoredParent}) {
        const bool source=rewrite==Rewrite::Source;
        const int anchor=rewrite==Rewrite::AnchoredParent?gSlot:-1;
        const auto bytes=RigExecTestEdited(result.bytes,[&](fb::RigExecWireFile *edited) {
            auto &sw=edited->pose->spaceSwitches[0];
            if(source) {
                sw.sourceReads[0].anchor=-1;
                sw.sourceReads[0].recompose={armSlot};
            } else {
                sw.parentRead->anchor=anchor;
                sw.parentRead->recompose={pSlot};
            }
        });
        const std::string where=source?"source_reads[0]":"parent_read";
        const std::string expected="invalid .rigexec: pose.space_switches[0]."+where+
            ": inline recomposition requires a checkpoint operation";
        const std::string actual=_OpenError(bytes);
        if(actual!=expected)std::printf("%s: %s expected %s\n",name,actual.c_str(),expected.c_str());
        CHECK(actual==expected);
    }
    std::printf("%s: checked\n", name);
}

// The nested fixture's switch indices set as inputs: pSpaces' and
// sSpaces' inputs:activeSpace (keyed) set to 0.25 and 0.75, held over
// every frame -- set again after the stage's keys at each frame, as a host
// holding a drag does -- then reset; then, at one frame, an input set on
// P's rz alone. S's switch reads P only recomposed from those avars, and
// its compose step is emitted before P's, so only the closure's dirtying
// of the steps that recompose P re-runs it. Every frame's version pools
// equal the baked program's under the same values authored in the session
// layer bit for bit, and the baked program agrees with the dynamic
// evaluator.
static void
TestSpaceSwitchIndexDrag()
{
    const char *const name = "space switch index drag";
    const UsdStageRefPtr stage =
        UsdStage::Open(_FixturePath("space_switch_nested.usda"));
    CHECK(stage);
    if (!stage) {
        return;
    }
    const SdfPath rigPath("/Rig");
    const std::vector<double> frames = {0.0, 1.0, 2.0, 3.0};
    RigExecRigEvaluator evaluator(stage, rigPath);
    std::vector<uint8_t> bytes;
    std::string error;
    CHECK(RigExecTestBakeAt(evaluator, frames.front(), &bytes, &error));
    RigExecTestPlayer player;
    if (!player.Open(bytes, stage, &error)) {
        std::printf("%s: open: %s\n", name, error.c_str());
        CHECK(false);
        return;
    }
    const auto run = [&](const char *what, double frame,
                         const _Reference &reference,
                         std::vector<std::vector<RrPointFrame>> *fin) {
        if (!player.Play(frame, &error)) {
            std::printf("%s, %s frame %.17g: %s\n", name, what, frame,
                        error.c_str());
            CHECK(false);
            return false;
        }
        fin->push_back(player->GetFinFrames());
        return _SamePools(what, frame, player.Reader(), reference);
    };
    const auto pass = [&](const char *what,
                          const std::vector<_Reference> &references,
                          std::vector<std::vector<RrPointFrame>> *fin) {
        bool same = references.size() == frames.size();
        for (size_t f = 0; f < frames.size() && f < references.size();
             ++f) {
            same = run(what, frames[f], references[f], fin) && same;
        }
        return same;
    };
    const std::string pIndex = "/Rig/Movers/pSpaces.inputs:activeSpace";
    const std::string sIndex = "/Rig/Movers/sSpaces.inputs:activeSpace";
    const std::vector<_Reference> unset =
        _References(stage, rigPath, {}, frames);
    std::vector<std::vector<RrPointFrame>> undragged, dragged, released;
    CHECK(pass("undragged", unset, &undragged));
    CHECK(player.Hold(pIndex, 0.25, &error));
    CHECK(player.Hold(sIndex, 0.75, &error));
    CHECK(pass("indices held",
               _References(stage, rigPath,
                           {_Edit(stage, pIndex, 0.25),
                            _Edit(stage, sIndex, 0.75)},
                           frames),
               &dragged));
    CHECK(dragged.size() == frames.size() &&
          undragged.size() == frames.size());
    // At frame 0 every control rests, where a switch holds its target in
    // place whatever its index; from frame 1 on the held indices move the
    // pools.
    for (size_t f = 1; f < dragged.size() && f < undragged.size(); ++f) {
        CHECK(dragged[f] != undragged[f]);
    }
    player.ReleaseAll();
    CHECK(pass("indices released", unset, &released));
    CHECK(released == undragged);

    // At one frame, only P's avars move.
    const size_t at = 2;
    const double frame = frames[at];
    const std::string pRz = "/Rig/Controls/P.avars:rz";
    std::vector<std::vector<RrPointFrame>> still, turned, back;
    CHECK(run("before the rz drag", frame, unset[at], &still));
    CHECK(run("before the rz drag, again", frame, unset[at], &still));
    CHECK(player.Hold(pRz, 40.0, &error));
    const std::vector<_Reference> rz =
        _References(stage, rigPath, {_Edit(stage, pRz, 40.0)}, {frame});
    CHECK(rz.size() == 1);
    if (rz.size() == 1) {
        CHECK(run("rz dragged", frame, rz[0], &turned));
    }
    player.ReleaseAll();
    CHECK(run("rz released", frame, unset[at], &back));
    CHECK(still.size() == 2 && turned.size() == 1 && back.size() == 1);
    if (still.size() == 2 && turned.size() == 1 && back.size() == 1) {
        CHECK(turned[0] != still[1]);
        CHECK(back[0] == still[1]);
    }
    std::printf("%s: checked\n", name);
}

// Open refuses a space switch that reads a slot the inventory does not
// hold, a recompose through a slot the compose has no avars for, and a
// version bound to a world source or a missing space; and refuses a cone
// lookup table naming a cluster that is not there. Each is edited into the
// carry fixture's bake, whose S switch reads {G, {P}} and {G, {P, Q}}, and
// refused by the format's validator naming the field.
static void
TestSpaceSwitchOpenRefusals()
{
    const char *const name = "space switch open refusals";
    const UsdStageRefPtr stage =
        UsdStage::Open(_FixturePath("space_switch_carry.usda"));
    CHECK(stage);
    if (!stage) {
        return;
    }
    // A real native Xform source supplies an otherwise-valid non-provider
    // slot for the checkpoint wrong-kind negative below.
    const auto checkpointSource=UsdGeomXform::Define(stage,SdfPath("/CheckpointSource"));
    CHECK(checkpointSource.MakeMatrixXform().Set(GfMatrix4d(1.0)));
    const auto checkpointTargetXform=UsdGeomXform::Define(stage,SdfPath("/Rig/Controls/CheckpointTarget"));
    CHECK(checkpointTargetXform.MakeMatrixXform().Set(GfMatrix4d(1.0)));
    const auto checkpointTarget=checkpointTargetXform.GetPrim();
    const auto checkpointProbe=stage->DefinePrim(SdfPath("/Rig/Movers/CheckpointProbe"),TfToken("RigExecPositionConstraint"));
    CHECK(checkpointProbe.ApplyAPI(TfToken("RigExecMoverAPI")));
    CHECK(checkpointProbe.CreateRelationship(TfToken("rigExec:moves")).SetTargets({checkpointTarget.GetPath()}));
    CHECK(checkpointProbe.GetRelationship(TfToken("rigExec:sources")).SetTargets({checkpointSource.GetPath()}));
    RigExecRigEvaluator evaluator(stage, SdfPath("/Rig"));
    RigExecBakeOpts opts;
    opts.time = 0.0;
    RigExecBakeResult result;
    std::string error;
    CHECK(RigExecBakeToBinary(evaluator, opts, &result, &error));
    CHECK(_OpenError(result.bytes).empty());
    const std::unique_ptr<fb::RigExecWireFile> file =
        RigExecTestUnpack(result.bytes);
    if (!file) {
        return;
    }
    const int g = _SlotOfPath(*file, "/Rig/Controls/G");
    const int p = _SlotOfPath(*file, "/Rig/Controls/G/P");
    const int s = _SlotOfPath(*file, "/Rig/Controls/G/P/S");
    const std::vector<fb::RigExecWireSpaceSwitch> &switches =
        file->pose->spaceSwitches;
    const fb::RigExecWireCones &cones = *file->cones;
    CHECK(g >= 0 && p >= 0 && s >= 0);
    size_t at = switches.size();
    for (size_t k = 0; k < switches.size(); ++k) {
        if (switches[k].slot == s) {
            at = k;
        }
    }
    if (g < 0 || p < 0 || s < 0 || at == switches.size() ||
        switches[at].sourceReads.size() != 2 ||
        switches[at].sourceSlots[1] != -1 || switches[at].spaceSlot < 0 ||
        size_t(s) >= cones.avarCluster.size()) {
        std::printf("%s: FAILED (no switch on S)\n", name);
        CHECK(false);
        return;
    }
    const int32_t slots = int32_t(file->slotMeta->paths.size());
    const auto expect = [&](const char *what,
                            const std::vector<uint8_t> &bytes,
                            const std::string &text) {
        const std::string want = "invalid .rigexec: " + text;
        const std::string got = _OpenError(bytes);
        if (got != want) {
            std::printf("%s, %s: open said '%s', expected '%s'\n", name,
                        what, got.c_str(), want.c_str());
            CHECK(false);
        }
    };
    const auto withSwitch =
        [&](const std::function<void(fb::RigExecWireSpaceSwitch &)> &edit) {
            return RigExecTestEdited(result.bytes,
                                     [&](fb::RigExecWireFile *edited) {
                                         edit(edited->pose->spaceSwitches[at]);
                                     });
        };
    const std::string row = "pose.space_switches[" + std::to_string(at) + "]";
    const std::string range =
        " out of range (" + std::to_string(slots) + ")";
    const std::string unread = ": reads no slot, so its version is {-1, []}";
    const int32_t contextCount=int32_t(file->pose->spaceCheckpoints.size());
    expect("parent checkpoint past the contexts",
           withSwitch([&](fb::RigExecWireSpaceSwitch &sw) {
               sw.parentRead->context=contextCount;
           }),
           row+".parent_read.context: "+std::to_string(contextCount)+
               " out of range ("+std::to_string(contextCount)+")");
    expect("source anchor past the slots",
           withSwitch([&](fb::RigExecWireSpaceSwitch &sw) {
               sw.sourceReads[0].anchor = slots;
           }),
           row + ".source_reads[0].anchor: " + std::to_string(slots) + range);
    expect("space checkpoint past the contexts",
           withSwitch([&](fb::RigExecWireSpaceSwitch &sw) {
               sw.spaceRead->context=contextCount;
           }),
           row+".space_read.context: "+std::to_string(contextCount)+
               " out of range ("+std::to_string(contextCount)+")");
    expect("space slot past the slots",
           withSwitch([&](fb::RigExecWireSpaceSwitch &sw) {
               sw.spaceSlot = slots;
           }),
           row + ".space_slot: " + std::to_string(slots) + range);
    expect("world source with a version",
           withSwitch([&](fb::RigExecWireSpaceSwitch &sw) {
               sw.sourceReads[1].anchor = g;
               sw.sourceReads[1].recompose.clear();
           }),
           row + ".source_reads[1]" + unread);
    expect("world source with a recompose",
           withSwitch([&](fb::RigExecWireSpaceSwitch &sw) {
               sw.sourceReads[1].anchor = -1;
               sw.sourceReads[1].recompose = {p};
           }),
           row+".source_reads[1]: inline recomposition requires a checkpoint operation");
    expect("missing space with a version",
           withSwitch([&](fb::RigExecWireSpaceSwitch &sw) {
               sw.spaceSlot = -1;
           }),
           row + ".space_read" + unread);
    const int32_t parentContext=switches[at].parentRead->context;
    CHECK(parentContext>=0 && parentContext<contextCount);
    if(parentContext<0 || parentContext>=contextCount)return;
    expect("checkpoint recompose past the slots",
           RigExecTestEdited(result.bytes,[&](fb::RigExecWireFile *edited) {
               edited->pose->spaceCheckpoints[size_t(parentContext)].recompose={slots};
           }),
           "pose.space_checkpoints["+std::to_string(parentContext)+"].recompose[0]: "+
               std::to_string(slots)+range);
    expect("checkpoint anchor past the slots",
           RigExecTestEdited(result.bytes,[&](fb::RigExecWireFile *edited) {
               edited->pose->spaceCheckpoints[size_t(parentContext)].anchor=slots;
           }),
           "pose.space_checkpoints["+std::to_string(parentContext)+"].anchor: "+
               std::to_string(slots)+range);
    const auto nonProvider=std::find(file->slotMeta->slotKind.begin(),file->slotMeta->slotKind.end(),fb::SlotKind::XformDerived);
    CHECK(nonProvider!=file->slotMeta->slotKind.end());
    expect("recompose of a slot that is not FirstFramePose",
           RigExecTestEdited(result.bytes,[&](fb::RigExecWireFile *edited) {
               edited->pose->spaceCheckpoints[size_t(parentContext)].recompose={int32_t(nonProvider-file->slotMeta->slotKind.begin())};
           }),
           "pose.space_checkpoints["+std::to_string(parentContext)+"]: recompose is not a provider");
    const int32_t clusters = int32_t(file->clustering->clusters.size());
    for (const int32_t cluster : {int32_t(-1), clusters}) {
        expect(cluster < 0 ? "avar cluster -1" : "avar cluster past the end",
               RigExecTestEdited(result.bytes,
                                 [&](fb::RigExecWireFile *edited) {
                                     edited->cones->avarCluster[size_t(s)] =
                                         cluster;
                                 }),
               cluster<0?"cones.avar_cluster["+std::to_string(s)+
                   "]: missing active Avars producer without an excluded value":
                   "cones.avar_cluster["+std::to_string(s)+"]: "+
                   std::to_string(cluster)+" out of range ("+std::to_string(clusters)+")");
    }
    std::printf("%s: checked\n", name);
}

// The carry fixture, at a frame where S blends Other (twist-filtered) with
// world and at one where S is at world. S's space reads Q recomposed
// through P and Q on G's frame, so an input set on G's, P's or Q's avars
// alone moves S outright. P's and Q's reach S's compose step only through
// the closure's dirtying of the steps that recompose them; G's through S's
// read of its posedM. Each set and its reset match the baked program's
// version pools under the same value authored in the session layer bit for
// bit, with parity against the dynamic walk.
static void
TestSpaceSwitchCarryDrag()
{
    const char *const name = "space switch carry drag";
    const UsdStageRefPtr stage =
        UsdStage::Open(_FixturePath("space_switch_carry.usda"));
    CHECK(stage);
    if (!stage) {
        return;
    }
    const SdfPath rigPath("/Rig");
    RigExecRigEvaluator evaluator(stage, rigPath);
    std::vector<uint8_t> bytes;
    std::string error;
    CHECK(RigExecTestBakeAt(evaluator, 0.0, &bytes, &error));
    const std::unique_ptr<fb::RigExecWireFile> file = RigExecTestUnpack(bytes);
    const int s = file ? _SlotOfPath(*file, "/Rig/Controls/G/P/S") : -1;
    RigExecTestPlayer player;
    const bool opened = player.Open(bytes, stage, &error);
    CHECK(opened && s >= 0);
    if (!opened || s < 0) {
        std::printf("%s: open: %s\n", name, error.c_str());
        return;
    }
    const auto run = [&](const std::string &what, double frame,
                         const _Reference &reference,
                         std::vector<std::vector<RrPointFrame>> *fin) {
        if (!player.Play(frame, &error)) {
            std::printf("%s, %s frame %.17g: %s\n", name, what.c_str(),
                        frame, error.c_str());
            CHECK(false);
            return false;
        }
        fin->push_back(player->GetFinFrames());
        return _SamePools(what.c_str(), frame, player.Reader(), reference);
    };
    struct Drag {
        const char *prim;
        const char *avar;
        double value;
    };
    const Drag drags[] = {{"/Rig/Controls/G/P", "avars:rz", 40.0},
                          {"/Rig/Controls/G/P/Q", "avars:tx", 6.0},
                          {"/Rig/Controls/G/P/Q", "avars:rz", -30.0},
                          {"/Rig/Controls/G", "avars:rz", 35.0}};
    for (const double frame : {0.0, 2.0}) {
        const std::vector<_Reference> unset =
            _References(stage, rigPath, {}, {frame});
        if (unset.size() != 1) {
            CHECK(false);
            continue;
        }
        for (const Drag &drag : drags) {
            const std::string what =
                std::string(drag.prim) + "." + drag.avar;
            const std::vector<_Reference> set = _References(
                stage, rigPath, {_Edit(stage, what, drag.value)}, {frame});
            std::vector<std::vector<RrPointFrame>> still, turned, back;
            CHECK(run(what + " before", frame, unset[0], &still));
            CHECK(run(what + " before, again", frame, unset[0], &still));
            CHECK(player.Hold(what, drag.value, &error));
            if (set.size() == 1) {
                CHECK(run(what + " dragged", frame, set[0], &turned));
            }
            player.ReleaseAll();
            CHECK(run(what + " released", frame, unset[0], &back));
            CHECK(still.size() == 2 && turned.size() == 1 &&
                  back.size() == 1);
            if (still.size() != 2 || turned.size() != 1 ||
                back.size() != 1 || size_t(s) >= still[1].size() ||
                size_t(s) >= turned[0].size()) {
                continue;
            }
            double moved = 0.0;
            for (size_t k = 0; k < 4; ++k) {
                for (size_t c = 0; c < 3; ++c) {
                    moved = std::max(
                        moved, std::abs(turned[0][size_t(s)].points[k][c] -
                                        still[1][size_t(s)].points[k][c]));
                }
            }
            std::printf("%s, %s at frame %g: S moves by %.17g\n", name,
                        what.c_str(), frame, moved);
            CHECK(moved > 1e-3);
            CHECK(back[0] == still[1]);
        }
    }
    std::printf("%s: checked\n", name);
}

// Rigs whose movers read phases run their cones like any other rig. At
// fine clusters (RIGEXEC_BAKED_GRAIN_US=0, one step per cluster) a repeat
// Execute at unchanged inputs closes fewer clusters than the file holds
// and still publishes what live baked publishes at the bake time, from the
// frame records and chain versions the skipped steps kept. Each drag held
// afterwards publishes what live baked publishes under the same session
// edit and moves the outputs; let go, the rig publishes the bake time's
// outputs again. At the default grain a small rig can sit in one or two
// clusters, so the count would show nothing there.
static void
TestPhasedRigsRunCones(const std::string &examplesDir)
{
    if (TfGetenv("RIGEXEC_BAKED_GRAIN_US", "") != "0") {
        std::printf("phased rigs under cones: checked at grain 0 only\n");
        return;
    }
    struct Row {
        std::string stage;
        double time;
        std::vector<std::pair<std::string, double>> drags;
    };
    const Row rows[] = {
        {examplesDir + "/13_ReadPhases.usda",
         1001.0,
         {{"/ReadPhaseAsset/Rig/Controls/LiftCtl.avars:ty", 0.25}}},
        {_FixturePath("frame_record_fallbacks.usda"),
         1.0,
         {{"/RecordAsset/Rig/Controls/Dial.avars:amount", 1.25},
          {"/RecordAsset/Rig/Controls/Late.avars:tz", 0.25}}},
        {_FixturePath("solver_checkpoint.usda"),
         1.0,
         {{"/CheckpointAsset/Rig/Controls/ZKnee.avars:rx", 15.25},
          {"/CheckpointAsset/Rig/Controls/KneeTarget.avars:tx", 0.25}}},
    };
    for (const Row &row : rows) {
        const std::string name =
            std::filesystem::path(row.stage).filename().string();
        const UsdStageRefPtr stage = UsdStage::Open(row.stage);
        CHECK(stage);
        if (!stage) {
            std::printf("%s: FAILED (no stage)\n", name.c_str());
            continue;
        }
        const SdfPath rigPath = _FindRig(stage);
        std::vector<uint8_t> bytes;
        std::string error;
        {
            RigExecRigEvaluator evaluator(stage, rigPath);
            if (!RigExecTestBakeAt(evaluator, row.time, &bytes, &error)) {
                std::printf("%s: FAILED (bake: %s)\n", name.c_str(),
                            error.c_str());
                CHECK(false);
                continue;
            }
        }
        std::unique_ptr<fb::RigExecWireFile> file;
        if (!RigExecFormatOpen(bytes.data(), bytes.size(), &file, &error)) {
            std::printf("%s: FAILED (open: %s)\n", name.c_str(),
                        error.c_str());
            CHECK(false);
            continue;
        }
        size_t bindings = 0;
        for (const fb::RigExecWireChain &chain : file->geometry->chains) {
            for (const fb::RigExecWireRevision &revision : chain.revisions) {
                bindings += revision.pointBindings.size();
            }
            for (const fb::RigExecWireDerived &derived : chain.derived) {
                if (derived.revision) {
                    bindings += derived.revision->pointBindings.size();
                }
            }
        }
        const size_t records = file->pose->frameRecords.size();
        const size_t clusters = file->clustering->clusters.size();
        CHECK(records + bindings > 0);

        std::vector<RigExecRigPose> still;
        if (!RigExecTestEditedPoses(stage, rigPath,
                                    {},
                                    {row.time}, &still, &error) ||
            still.size() != 1) {
            std::printf("%s: FAILED (reference: %s)\n", name.c_str(),
                        error.c_str());
            CHECK(false);
            continue;
        }
        RigExecTestPlayer player;
        if (!player.Open(bytes, stage, &error)) {
            std::printf("%s: FAILED (open: %s)\n", name.c_str(),
                        error.c_str());
            CHECK(false);
            continue;
        }
        const auto matches = [&](const RigExecRigPose &pose,
                                 const std::string &what) {
            std::vector<std::string> diffs;
            const bool same =
                RigExecCompareRuntimeOutputs(pose, player.Reader(), &diffs);
            if (!same) {
                std::printf("%s, %s: %s\n", name.c_str(), what.c_str(),
                            diffs.empty() ? "differs"
                                          : diffs.front().c_str());
            }
            return same;
        };
        CHECK(player.Play(row.time, &error));
        CHECK(matches(still[0], "first run"));
        const size_t first = player->GetClosedClusterCountForTesting();
        CHECK(player.Play(row.time, &error));
        const size_t repeat = player->GetClosedClusterCountForTesting();
        CHECK(repeat < clusters);
        CHECK(matches(still[0], "repeat run"));
        std::printf("%s: %zu frame record(s), %zu point binding(s); closed "
                    "%zu, then %zu at unchanged inputs, of %zu cluster(s)\n",
                    name.c_str(), records, bindings, first, repeat,
                    clusters);

        for (const auto &[path, value] : row.drags) {
            const std::string what =
                path + " = " + std::to_string(value);
            std::vector<RigExecRigPose> dragged;
            if (!RigExecTestEditedPoses(stage, rigPath,
                                        {_Edit(stage, path, value)},
                                        {row.time}, &dragged, &error) ||
                dragged.size() != 1) {
                std::printf("%s, %s: FAILED (reference: %s)\n",
                            name.c_str(), what.c_str(), error.c_str());
                CHECK(false);
                continue;
            }
            CHECK(player.Hold(path, value, &error));
            CHECK(player.Play(row.time, &error));
            const size_t held = player->GetClosedClusterCountForTesting();
            CHECK(matches(dragged[0], what));
            std::vector<std::string> moved;
            CHECK(!RigExecCompareRuntimeOutputs(still[0], player.Reader(),
                                                &moved));
            player.ReleaseAll();
            CHECK(player.Play(row.time, &error));
            const size_t released =
                player->GetClosedClusterCountForTesting();
            CHECK(matches(still[0], what + ", released"));
            std::printf("%s, %s: closed %zu, then %zu released\n",
                        name.c_str(), what.c_str(), held, released);
        }
    }
}

// The FrameMatrix step's gates, white-box over a hand-assembled program,
// against RigExecBakedEvalFrameRecord on the same values bit for bit, the
// matrix and the valid byte: a constraint commit's exit flags as its
// constraint step leaves them in the pose scratch (nothing recorded with
// recordAfter clear, the first target alone with recordEveryTarget clear),
// a solver commit's present byte, which ignores those flags, an unusable
// frame, a rest without a valid frame (the identity landmarks) and a
// singular one. Every run writes both fields over a poisoned entry. No
// authored rig reaches the two constraint exits.
static void
TestFrameMatrixGates()
{
    // One slot; commit 0 a constraint's, commit 1 a solver batch's.
    fb::RigExecWireFile file;
    file.pose = std::make_unique<fb::RigExecWireDomainPose>();
    file.geometry = std::make_unique<fb::RigExecWireDomainGeometry>();
    fb::RigExecWireSlotMeta slotMeta;
    slotMeta.paths = {0};
    fb::RigExecWireConstants constants;
    constants.restM.resize(1);
    constants.restPts.resize(1);
    constants.restFrames.resize(1);
    constants.selfD.resize(1);
    constants.posedD.resize(1);
    constants.parentSpaceM.resize(1);
    constants.parentSpaceAuthored.resize(1);
    constants.parentDinv.resize(1);
    constants.rotOrder.resize(1);
    constants.restRoundTrip.resize(1);
    constants.defaultRoundTrip.resize(1);
    constants.posedAuthored.resize(1);
    constants.posedAuthoredM.resize(1);
    constants.noScaleAvars.resize(1);
    file.pose->commits.resize(2);
    file.pose->commits[1].solverOutput = true;
    file.pose->commits[1].slots = {0};
    // Constraint targets 0 and 1, the solver's position 0, and a record of
    // an unusable version: (commit, target, position, version).
    const std::array<std::array<int, 4>, 4> records = {
        {{0, 0, -1, 0}, {0, 1, -1, 0}, {1, -1, 0, 0}, {0, 0, -1, 1}}};
    std::vector<RigExecBakedFrameRecord> baked;
    for (size_t r = 0; r < records.size(); ++r) {
        const std::array<int, 4> &row = records[r];
        file.pose->frameRecords.push_back(fb::FrameRecord(
            0, uint32_t(row[0]), row[1], row[2], uint32_t(row[3]), 0));
        RigExecBakedFrameRecord record;
        record.slot = 0;
        record.commit = row[0];
        record.target = row[1];
        record.position = row[2];
        record.version = uint32_t(row[3]);
        baked.push_back(record);
        fb::RigExecWireStep step;
        step.kind = fb::StepKind::FrameMatrix;
        step.object = int32_t(r);
        file.steps.push_back(std::move(step));
    }

    RrProgram program;
    program.steps = &file.steps;
    program.slotMeta = &slotMeta;
    program.constants = &constants;
    program.poses = file.pose.get();
    program.geometry = file.geometry.get();
    RrStore &store = program.store;
    store.commits.resize(2);
    store.stepOutputs.resize(file.steps.size());
    store.frameMatrix.assign(records.size(), RrMat4d(1.0));
    store.frameMatrixValid.assign(records.size(), 0);
    std::string error;
    CHECK(RrPoseSizeScratch(&program, &error));
    if (!program.pose) {
        std::printf("frame matrix gates: %s\n", error.c_str());
        return;
    }
    RrPoseScratch &scratch = *static_cast<RrPoseScratch *>(program.pose.get());

    // The program's twin over the same frames.
    RigExecPointFrame rest;
    rest.points = {GfVec3d(0, 1, 0), GfVec3d(1, 1, 0), GfVec3d(0, 2, 0),
                   GfVec3d(0, 1, 1)};
    RigExecPointFrame posed;
    posed.points = {GfVec3d(2, 0, 0), GfVec3d(2, 1, 0), GfVec3d(1, 0, 0),
                    GfVec3d(2, 0, 2)};
    RigExecPointFrame unusable = posed;
    unusable.flags = 0;
    RigExecBakedProgramImpl B;
    B.paths = {SdfPath("/A")};
    B.fin = {posed, unusable};
    B.commits.resize(2);
    B.commits[1].solverOutput = true;
    B.commits[1].slots = {0};
    const auto runtimeFrame = [](const RigExecPointFrame &frame) {
        RrPointFrame out;
        for (size_t i = 0; i < 4; ++i) {
            out.points[i] = RrVec3d(frame.points[i][0], frame.points[i][1],
                                    frame.points[i][2]);
        }
        out.flags = frame.flags;
        return out;
    };
    store.fin = {runtimeFrame(posed), runtimeFrame(unusable)};
    const auto setRest = [&](const RigExecPointFrame &frame) {
        B.restFrames = {frame};
        scratch.restFrames[0] = runtimeFrame(frame);
    };
    setRest(rest);

    int cases = 0;
    int recorded = 0;
    // Runs record \p r's step over a poisoned entry and compares it with
    // the program's evaluation of the same record.
    const auto agree = [&](size_t r, const char *what) {
        ++cases;
        RrMat4d poison(1.0);
        poison[3][0] = 7.0;
        store.frameMatrix[r] = poison;
        store.frameMatrixValid[r] = 2;
        error.clear();
        const bool ran = RrRunPoseStep(&program, r, &error);
        CHECK(ran);
        GfMatrix4d expected(0.0);
        const bool valid =
            RigExecBakedEvalFrameRecord(B, baked[r], &expected);
        const RrMat4d &actual = store.frameMatrix[r];
        bool same = ran && store.frameMatrixValid[r] == (valid ? 1 : 0);
        for (size_t row = 0; row < 4; ++row) {
            for (size_t col = 0; col < 4; ++col) {
                same = same && std::memcmp(&actual[row][col],
                                           &expected[int(row)][int(col)],
                                           sizeof(double)) == 0;
            }
        }
        CHECK(same);
        if (!same) {
            std::printf("frame matrix gates, %s: runtime valid %d, program "
                        "valid %d%s%s\n",
                        what, int(store.frameMatrixValid[r]), int(valid),
                        error.empty() ? "" : ": ", error.c_str());
        }
        recorded += valid ? 1 : 0;
        return valid;
    };
    const auto exitFlags = [&](bool after, bool everyTarget) {
        scratch.recordAfter[0] = after ? 1 : 0;
        scratch.recordEveryTarget[0] = everyTarget ? 1 : 0;
        B.commits[0].recordAfter = after;
        B.commits[0].recordEveryTarget = everyTarget;
    };

    // The constraint's exits: every target, the first alone, none.
    exitFlags(true, true);
    CHECK(agree(0, "every target, target 0"));
    CHECK(agree(1, "every target, target 1"));
    exitFlags(true, false);
    CHECK(agree(0, "first target alone, target 0"));
    CHECK(!agree(1, "first target alone, target 1"));
    exitFlags(false, true);
    CHECK(!agree(0, "no record, target 0"));
    CHECK(!agree(1, "no record, target 1"));
    exitFlags(false, false);
    CHECK(!agree(0, "no record nor every target, target 0"));
    CHECK(!agree(1, "no record nor every target, target 1"));

    // A solver's record reads its present byte alone; the constraint
    // flags of its own commit do not apply.
    scratch.recordAfter[1] = 0;
    scratch.recordEveryTarget[1] = 0;
    B.commits[1].recordAfter = false;
    B.commits[1].recordEveryTarget = false;
    store.commits[1].present = {1};
    B.commits[1].present = {1};
    CHECK(agree(2, "solver present"));
    store.commits[1].present = {0};
    B.commits[1].present = {0};
    CHECK(!agree(2, "solver absent"));

    // An unusable frame is not recorded; a rest without a valid frame
    // measures from the identity landmarks; a singular one does not
    // decompose.
    exitFlags(true, true);
    CHECK(!agree(3, "unusable frame"));
    RigExecPointFrame invalidRest = rest;
    invalidRest.flags = 0;
    setRest(invalidRest);
    CHECK(agree(0, "rest without a valid frame"));
    RigExecPointFrame singular = rest;
    singular.points[3] = singular.points[0];
    setRest(singular);
    CHECK(!agree(0, "singular rest"));
    setRest(rest);
    std::printf("frame matrix gates: %d case(s) agree with the program, %d "
                "recorded\n",
                cases, recorded);
}

// A non-finite composed rest keeps its typed invalid state while a usable
// final frame publishes ORIGINAL's local matrix fallback. Independent limb
// output continues, held failure retains its value, and frame 1 recovers.
static void
TestUnusableComposedRestUsesLocalMatrixFallback()
{
    const char *const name = "unusable composed rest";
    const UsdStageRefPtr stage =
        UsdStage::Open(_FixturePath("oneloop_two_limbs.usda"));
    CHECK(stage);
    if (!stage) {
        return;
    }
    const SdfPath joint("/LimbsAsset/Rig/Joints/LimbA0/LimbA1/LimbA2");
    const SdfPath restTx = joint.AppendProperty(TfToken("rest:tx"));
    {
        const UsdEditContext session(stage, stage->GetSessionLayer());
        const UsdPrim prim =
            stage->DefinePrim(joint, TfToken("RigExecJoint"));
        GfMatrix4d space(1.0);
        space.SetTranslateOnly(GfVec3d(2.0, 0.0, 0.0));
        prim.CreateAttribute(TfToken("rest:space"),
                             SdfValueTypeNames->Matrix4d)
            .Set(space);
        prim.CreateAttribute(TfToken("default:space"),
                             SdfValueTypeNames->Matrix4d)
            .Set(space);
        const UsdAttribute tx = prim.CreateAttribute(
            TfToken("rest:tx"), SdfValueTypeNames->Double);
        tx.Set(0.0, UsdTimeCode(1.0));
        tx.Set(std::numeric_limits<double>::quiet_NaN(), UsdTimeCode(2.0));
    }
    RigExecRigEvaluator evaluator(stage, SdfPath("/LimbsAsset/Rig"));
    std::vector<uint8_t> bytes;
    std::string error;
    CHECK(RigExecTestBakeAt(evaluator, 1.0, &bytes, &error));
    RigExecTestPlayer player;
    if (!player.Open(bytes, stage, &error)) {
        std::printf("FAILED: %s: %s\n", name, error.c_str());
        ++failures;
        return;
    }
    // The sampler hands the reader frame 2's NaN only through an Animated
    // slot.
    size_t index = 0;
    CHECK(player->FindInput(restTx.GetString(), &index) &&
          player->GetInputInfo(index).animated);

    // Captured independently from ORIGINAL's baked fallback and reference
    // evaluator for this exact 1 -> NaN 2 -> held 2 -> recovery 1 history.
    const std::array<std::array<uint64_t,16>,2> targetBits{{
        {{0x3ff0000000000000ULL,0x0000000000000000ULL,0x0000000000000000ULL,0x0000000000000000ULL,0x0000000000000000ULL,0x3ff0000000000000ULL,0x0000000000000000ULL,0x0000000000000000ULL,0x0000000000000000ULL,0x0000000000000000ULL,0x3ff0000000000000ULL,0x0000000000000000ULL,0xc000000000000000ULL,0x0000000000000000ULL,0x0000000000000000ULL,0x3ff0000000000000ULL}},
        {{0x3fefffffffffffffULL,0x0000000000000000ULL,0x0000000000000000ULL,0x0000000000000000ULL,0x0000000000000000ULL,0x3fefffffffffffffULL,0x0000000000000000ULL,0x0000000000000000ULL,0x0000000000000000ULL,0x0000000000000000ULL,0x3ff0000000000000ULL,0x0000000000000000ULL,0x3cb0000000000000ULL,0x3c70000000000000ULL,0x0000000000000000ULL,0x3ff0000000000000ULL}}
    }};
    const std::array<std::array<uint64_t,16>,2> unrelatedBits{{
        {{0x3fefd7583bc82e2aULL,0x3fb9791363068af7ULL,0x0000000000000000ULL,0x0000000000000000ULL,0xbfb9791363068af7ULL,0x3fefd7583bc82e2aULL,0x0000000000000000ULL,0x0000000000000000ULL,0x0000000000000000ULL,0x0000000000000000ULL,0x3ff0000000000000ULL,0x0000000000000000ULL,0x0000000000000000ULL,0x0000000000000000ULL,0x0000000000000000ULL,0x3ff0000000000000ULL}},
        {{0x3fee588e2d6fa817ULL,0x3fd44f639083e0d2ULL,0x0000000000000000ULL,0x0000000000000000ULL,0xbfd44f639083e0d3ULL,0x3fee588e2d6fa817ULL,0x0000000000000000ULL,0x0000000000000000ULL,0x0000000000000000ULL,0x0000000000000000ULL,0x3ff0000000000000ULL,0x0000000000000000ULL,0x0000000000000000ULL,0x0000000000000000ULL,0x0000000000000000ULL,0x3ff0000000000000ULL}}
    }};

    const SdfPath unrelated("/LimbsAsset/Rig/Joints/LimbB0");
    const auto checkBits = [&](const GfMatrix4d &matrix,
                               const std::array<uint64_t,16> &expected) {
        for (size_t r=0;r<4;++r) for (size_t c=0;c<4;++c) {
            uint64_t bits=0;
            const double value=matrix[r][c];
            std::memcpy(&bits,&value,sizeof(bits));
            CHECK(bits==expected[r*4+c]);
        }
    };
    const auto publishes = [&](double frame) {
        const size_t generations=evaluator.GetBakedGenerationCount();
        const RigExecRigPose live=evaluator.Evaluate(UsdTimeCode(frame));
        CHECK(live.valid);
        CHECK(evaluator.GetBakedGenerationCount()==generations+1);
        CHECK(live.jointMatricesFinal.size()==6);
        CHECK(live.diagnostics.empty());
        const auto final=live.jointFramesFinal.find(joint);
        CHECK(final!=live.jointFramesFinal.end() &&
              RigExecBakedUsable(final->second));
        const auto target=live.jointMatricesFinal.find(joint);
        const auto other=live.jointMatricesFinal.find(unrelated);
        CHECK(target!=live.jointMatricesFinal.end());
        CHECK(other!=live.jointMatricesFinal.end());
        const size_t expected=frame==2.0?1:0;
        if(target!=live.jointMatricesFinal.end())checkBits(target->second,targetBits[expected]);
        if(other!=live.jointMatricesFinal.end())checkBits(other->second,unrelatedBits[expected]);

        double raw=0.0;
        CHECK(stage->GetAttributeAtPath(restTx).Get(&raw,UsdTimeCode(frame)));
        CHECK(frame==2.0?std::isnan(raw):raw==0.0);
        const auto *program=evaluator.GetBakedProgram();
        CHECK(program);
        if(program) {
            const auto &graph=program->GetStepGraph();
            const auto slot=graph.index.find(joint);
            CHECK(slot!=graph.index.end());
            if(slot!=graph.index.end()) {
                const auto &rest=graph.restFrames[size_t(slot->second)];
                CHECK(frame==2.0?!RigExecBakedUsable(rest):RigExecBakedUsable(rest));
            }
        }
        std::string why;
        CHECK(player.Play(frame,&why));
        CHECK(why.empty());
        CHECK(frame==2.0?std::isnan(player->GetInputValue(index).f64):
                          player->GetInputValue(index).f64==0.0);
        std::vector<std::string> diffs;
        CHECK(RigExecCompareRuntimeOutputs(live,player.Reader(),&diffs));
        for(const std::string &diff:diffs)
            std::printf("  %s frame %g: %s\n",name,frame,diff.c_str());
    };
    publishes(1.0);
    publishes(2.0);
    publishes(2.0);
    publishes(1.0);
}

// computed_ik_space with Master's rest:ry and rest:tx keyed over 1..10 in
// a root layer above the fixture, so a drag authored in the session layer
// overrides them. Master is the space of both the arm's TwoBoneIk and the
// tail's SplineIk, so a moved Master rest moves the frame both solve in:
// playback refreshes it with the solvers' other rests, as baked does.
// Baked at frame 1, played at 1, 5 and 10, then dragged at a held 5.
static void
TestSpaceRestMovesWithItsRests()
{
    const char *const name = "space rest moves with its rests";
    const SdfLayerRefPtr root = SdfLayer::CreateAnonymous(".usda");
    root->SetSubLayerPaths({_FixturePath("computed_ik_space.usda")});
    const UsdStageRefPtr stage = UsdStage::Open(root);
    CHECK(stage);
    if (!stage) {
        return;
    }
    const SdfPath master("/IkSpaceAsset/Rig/Controls/Master");
    const UsdPrim prim = stage->GetPrimAtPath(master);
    const UsdAttribute ry = prim.GetAttribute(TfToken("rest:ry"));
    const UsdAttribute tx = prim.GetAttribute(TfToken("rest:tx"));
    CHECK(ry && tx);
    if (!ry || !tx) {
        return;
    }
    {
        const UsdEditContext context(stage, root);
        ry.Set(0.0, UsdTimeCode(1.0));
        ry.Set(20.0, UsdTimeCode(10.0));
        tx.Set(0.0, UsdTimeCode(1.0));
        tx.Set(3.0, UsdTimeCode(10.0));
    }
    const SdfPath rigPath("/IkSpaceAsset/Rig");
    RigExecRigEvaluator evaluator(stage, rigPath);
    std::vector<uint8_t> bytes;
    std::string error;
    CHECK(RigExecTestBakeAt(evaluator, 1.0, &bytes, &error));
    const std::unique_ptr<RigExecRuntimeReader> reader =
        RigExecRuntimeReader::Open(bytes.data(), bytes.size(), &error);
    RigExecInputSampler sampler;
    if (!reader || !sampler.Bind(stage, *reader, &error)) {
        std::printf("FAILED: %s: %s\n", name, error.c_str());
        ++failures;
        return;
    }
    // Both rests reach the binary per frame only as Animated inputs.
    for (const UsdAttribute &a : {ry, tx}) {
        size_t index = 0;
        CHECK(reader->FindInput(a.GetPath().GetString(), &index) &&
              reader->GetInputInfo(index).animated);
    }

    int compared = 0;
    const auto same = [&](const RigExecRigPose &live, const char *what,
                          double frame) {
        std::vector<std::string> diffs;
        const bool equal =
            RigExecCompareRuntimeOutputs(live, *reader, &diffs);
        CHECK(equal);
        for (const std::string &diff : diffs) {
            std::printf("  %s, %s %g: %s\n", name, what, frame,
                        diff.c_str());
        }
        ++compared;
    };
    for (const double frame : {1.0, 5.0, 10.0}) {
        CHECK(sampler.Apply(UsdTimeCode(frame), reader.get(), &error) &&
              reader->Execute(&error));
        same(evaluator.Evaluate(UsdTimeCode(frame)), "frame", frame);
    }

    // At a held 5: Master's rest:ry set to 15, then 25, against the value
    // authored in the session layer of a reference compiled before it.
    const double held = 5.0;
    CHECK(sampler.Apply(UsdTimeCode(held), reader.get(), &error));
    for (const double value : {15.0, 25.0}) {
        RigExecRigEvaluator reference(stage, rigPath);
        CHECK(reference.Compile());
        const RigExecTestSessionEdit edit(ry, value);
        CHECK(edit.IsSet());
        CHECK(reader->SetInput(ry.GetPath().GetString(), value, &error) &&
              reader->Execute(&error));
        same(reference.Evaluate(UsdTimeCode(held)), "drag", value);
    }
    // Released: the input takes the stage's value at the held frame again.
    CHECK(reader->ResetInput(ry.GetPath().GetString(), &error));
    sampler.Invalidate();
    CHECK(sampler.Apply(UsdTimeCode(held), reader.get(), &error) &&
          reader->Execute(&error));
    same(evaluator.Evaluate(UsdTimeCode(held)), "released at", held);
    std::printf("%s: %d comparison(s)\n", name, compared);
}

// Bakes the rig at \p rigPath of \p stage at \p time and opens the file's
// tables; false, with a FAILED line naming \p what, when either fails.
static bool
_BakeAndOpen(const std::string &what, const UsdStageRefPtr &stage,
             const SdfPath &rigPath, double time,
             std::vector<uint8_t> *bytes,
             std::unique_ptr<fb::RigExecWireFile> *file)
{
    std::string error;
    {
        RigExecRigEvaluator evaluator(stage, rigPath);
        if (!RigExecTestBakeAt(evaluator, time, bytes, &error)) {
            std::printf("FAILED: %s: bake: %s\n", what.c_str(),
                        error.c_str());
            ++failures;
            return false;
        }
    }
    if (!RigExecFormatOpen(bytes->data(), bytes->size(), file, &error)) {
        std::printf("FAILED: %s: open: %s\n", what.c_str(), error.c_str());
        ++failures;
        return false;
    }
    return true;
}

static void
TestRepeatedSetRunsNothing()
{
    const char *const name = "repeated set runs nothing";
    struct Row {
        const char *file;
        const char *rig;
        const char *input;
        double value;
        bool animated;
        // Stable operation kind and semantic owner among the input readers.
        fb::StepKind readerKind;
        const char *reader;
        // Matrix rest-space coverage uses the original static upstream fixture.
        bool matrix = false;
    };
    const Row rows[] = {
        {"oneloop_two_limbs.usda", "/LimbsAsset/Rig",
         "/LimbsAsset/Rig/Solvers/LimbBIK.inputs:softness", 0.5, false,
         fb::StepKind::Solve, "/LimbsAsset/Rig/Solvers/LimbBIK", false},
        {"oneloop_two_limbs.usda", "/LimbsAsset/Rig",
         "/LimbsAsset/Rig/Controls/A0.avars:rz", 15.0, true,
         fb::StepKind::ComposeSubtree, "/LimbsAsset/Rig/Controls/A0", false},
        {"computed_ik_space.usda", "/IkSpaceAsset/Rig",
         "/IkSpaceAsset/Rig/Solvers/ArmIK.inputs:softness", 0.35, false,
         fb::StepKind::Solve, "/IkSpaceAsset/Rig/Solvers/ArmIK", false},
        {"upstream_inputs.usda", "/LimbsAsset/Rig",
         "/LimbsAsset/Upstream.inputs:space", 0.0, false,
         fb::StepKind::RestCompose, "/LimbsAsset/Rig/Controls/BRoot", true},
    };
    const double frame = 1.0;
    for (const Row &row : rows) {
        const std::string what =
            std::string(name) + ", " + row.file + " " + row.input;
        const UsdStageRefPtr stage = UsdStage::Open(_FixturePath(row.file));
        CHECK(stage);
        if (!stage) {
            continue;
        }
        const SdfPath rigPath(row.rig);
        std::vector<uint8_t> bytes;
        std::unique_ptr<fb::RigExecWireFile> file;
        if (!_BakeAndOpen(what, stage, rigPath, frame, &bytes, &file)) {
            continue;
        }
        GfMatrix4d matrix(1.0); matrix.SetTranslateOnly(GfVec3d(1,0,10));
        const VtValue inputValue = row.matrix ? VtValue(matrix) :
            RigExecTestTypedValue(stage->GetAttributeAtPath(SdfPath(row.input)),row.value);
        std::string error;
        std::vector<RigExecRigPose> still;
        std::vector<RigExecRigPose> dragged;
        if (!RigExecTestEditedPoses(stage, rigPath,
                                    {},
                                    {frame}, &still, &error) ||
            !RigExecTestEditedPoses(stage, rigPath,
                                    {{SdfPath(row.input),inputValue}},
                                    {frame}, &dragged, &error) ||
            still.size() != 1 || dragged.size() != 1) {
            std::printf("FAILED: %s: reference: %s\n", what.c_str(),
                        error.c_str());
            ++failures;
            continue;
        }
        RigExecTestPlayer player;
        if (!player.Open(bytes, stage, &error)) {
            std::printf("FAILED: %s: open: %s\n", what.c_str(),
                        error.c_str());
            ++failures;
            continue;
        }
        size_t index = 0;
        CHECK(player->FindInput(row.input, &index) &&
              player->GetInputInfo(index).animated == row.animated);
        CHECK(player.Play(frame, &error));
        RrInputValue runtimeValue;
        CHECK(RigExecInputValueFrom(inputValue,player->GetInputInfo(index).type,&runtimeValue));

        // An unchanged supported fixture executes literally zero op bodies.
        const auto idle = [&](const char *leg) {
            const std::vector<int32_t> trace =
                player->GetLastRunTraceForTesting();
            const auto operations=player->GetCounters().executedOpCount;
            std::printf("%s, %s: %zu traced, %llu executed op bodies\n",what.c_str(),leg,
                trace.size(),static_cast<unsigned long long>(operations));
            CHECK(operations==trace.size());
            return trace.empty() && operations==0;
        };
        const auto ranReader = [&] {
            for (const int32_t step : player->GetLastRunTraceForTesting()) {
                if (step >= 0 && size_t(step) < file->steps.size() &&
                    file->steps[size_t(step)].kind == row.readerKind &&
                    player->GetStepLabelForTesting(size_t(step)) == row.reader) {
                    return true;
                }
            }
            return false;
        };
        const auto matches = [&](const RigExecRigPose &pose,
                                 const char *leg) {
            std::vector<std::string> diffs;
            const bool same =
                RigExecCompareRuntimeOutputs(pose, player.Reader(), &diffs);
            if (!same) {
                std::printf("%s, %s: %s\n", what.c_str(), leg,
                            diffs.empty() ? "differs"
                                          : diffs.front().c_str());
            }
            return same;
        };

        CHECK(player->SetInput(row.input, runtimeValue, &error) &&
              player->Execute(&error));
        CHECK(!idle("set") && ranReader());
        CHECK(matches(dragged[0], "set"));
        std::vector<std::string> moved;
        CHECK(!RigExecCompareRuntimeOutputs(still[0], player.Reader(),
                                            &moved));

        CHECK(player->SetInput(row.input, runtimeValue, &error) &&
              player->Execute(&error));
        CHECK(idle("set again"));
        CHECK(matches(dragged[0], "set again"));

        CHECK(player->Execute(&error));
        CHECK(idle("nothing set"));
        CHECK(matches(dragged[0], "nothing set"));

        CHECK(player->ResetInput(row.input, &error) &&
              player->Execute(&error));
        CHECK(!idle("reset") && ranReader());
        CHECK(matches(still[0], "reset"));

        CHECK(player->Execute(&error));
        CHECK(idle("after reset"));
        CHECK(matches(still[0], "after reset"));
    }
    std::printf("%s: checked\n", name);
}

// A drag held for two runs counts the work live baked counts for the same
// value authored as an edit: the edit applied, then two generations at
// the held frame, the first a whole first run. After each run the
// revisions executed and created, the schedules built and the clusters
// the closure ran equal baked's, and the outputs are equal bit for bit;
// the revision counters alone do not show a re-run that recomputes the
// same values, the cluster count does. Two drags of the two-limb rig: the
// keyed avar its verify_binary entry drags, and LimbBIK's softness, a
// Solve's declared input that reaches MeshB's skin; and the Dial drag of
// frame_record_fallbacks, which reaches R2's revision through its
// defaultWeight.
static void
TestHeldDragCountsValueEditWork()
{
    const char *const name = "held drag counts value-edit work";
    struct Row {
        const char *file;
        const char *rig;
        double time;
        const char *input;
        double value;
    };
    const Row rows[] = {
        {"oneloop_two_limbs.usda", "/LimbsAsset/Rig", 1.0,
         "/LimbsAsset/Rig/Controls/A0.avars:rz", 0.25},
        {"oneloop_two_limbs.usda", "/LimbsAsset/Rig", 1.0,
         "/LimbsAsset/Rig/Solvers/LimbBIK.inputs:softness", 0.5},
        {"frame_record_fallbacks.usda", "/RecordAsset/Rig", 1.0,
         "/RecordAsset/Rig/Controls/Dial.avars:amount", 1.25},
    };
    for (const Row &row : rows) {
        const std::string what =
            std::string(name) + ", " + row.file + " " + row.input;
        const UsdStageRefPtr stage = UsdStage::Open(_FixturePath(row.file));
        CHECK(stage);
        if (!stage) {
            continue;
        }
        const SdfPath rigPath(row.rig);
        std::vector<uint8_t> bytes;
        std::unique_ptr<fb::RigExecWireFile> file;
        if (!_BakeAndOpen(what, stage, rigPath, row.time, &bytes, &file)) {
            continue;
        }
        std::string error;
        std::vector<RigExecRigPose> poses;
        // Per generation: the clusters baked's closure ran, and how many
        // generations its program had answered.
        std::vector<std::pair<size_t, size_t>> bakedRuns;
        std::vector<size_t> liveOps;
        const auto each = [&](const RigExecRigEvaluator &evaluator,
                              size_t) {
            const RigExecBakedProgram *program =
                evaluator.GetBakedProgram();
            bakedRuns.emplace_back(
                program ? program->GetStepGraph().lastClosedClusters
                        : size_t(0),
                evaluator.GetBakedGenerationCount());
            liveOps.push_back(evaluator.GetLastOpTrace().size());
        };
        if (!RigExecTestEditedPoses(stage, rigPath,
                                    {_Edit(stage, row.input, row.value)},
                                    {row.time, row.time}, &poses, &error,
                                    each) ||
            poses.size() != 2 || bakedRuns.size() != 2) {
            std::printf("FAILED: %s: reference: %s\n", what.c_str(),
                        error.c_str());
            ++failures;
            continue;
        }
        RigExecTestPlayer player;
        if (!player.Open(bytes, stage, &error)) {
            std::printf("FAILED: %s: open: %s\n", what.c_str(),
                        error.c_str());
            ++failures;
            continue;
        }
        CHECK(player.Hold(row.input, row.value, &error));
        for (size_t g = 0; g < poses.size(); ++g) {
            const RigExecRigPose &pose = poses[g];
            CHECK(player.Play(row.time, &error));
            const auto runtimeOps=player->GetLastRunTraceForTesting().size();
            std::printf("%s, run %zu: operations %zu/%zu, clusters %zu/%zu, generations "
                        "%zu (baked/binary)\n",
                        what.c_str(), g + 1,
                        liveOps[g],runtimeOps,
                        bakedRuns[g].first,
                        player->GetClosedClusterCountForTesting(),
                        bakedRuns[g].second);
            CHECK(liveOps[g]==runtimeOps);
            CHECK(bakedRuns[g].second == g + 1 &&
                  bakedRuns[g].first ==
                      player->GetClosedClusterCountForTesting());
            std::vector<std::string> diffs;
            CHECK(RigExecCompareRuntimeOutputs(pose, player.Reader(),
                                               &diffs));
            for (const std::string &diff : diffs) {
                std::printf("  %s, run %zu: %s\n", what.c_str(), g + 1,
                            diff.c_str());
            }
        }
        // The first run executes the revisions; the second executes none.
        CHECK(liveOps[0]>0);
        CHECK(liveOps[1]==0);
    }
    std::printf("%s: checked\n", name);
}

// GetLastRunTraceForTesting on a fresh reader's first Execute: every step
// GetStepRanForTesting reports, each once, and each
// step after every predecessor the run also ran.
static void
TestRunTraceOrder()
{
    const char *const name = "run trace order";
    struct Row {
        const char *file;
        const char *rig;
    };
    const Row rows[] = {
        {"oneloop_two_limbs.usda", "/LimbsAsset/Rig"},
        {"frame_record_fallbacks.usda", "/RecordAsset/Rig"},
    };
    for (const Row &row : rows) {
        const std::string what = std::string(name) + ", " + row.file;
        const UsdStageRefPtr stage = UsdStage::Open(_FixturePath(row.file));
        CHECK(stage);
        if (!stage) {
            continue;
        }
        std::vector<uint8_t> bytes;
        std::unique_ptr<fb::RigExecWireFile> file;
        if (!_BakeAndOpen(what, stage, SdfPath(row.rig), 1.0, &bytes,
                          &file)) {
            continue;
        }
        std::string error;
        std::unique_ptr<RigExecRuntimeReader> reader =
            RigExecRuntimeReader::Open(bytes.data(), bytes.size(), &error);
        CHECK(reader);
        if (!reader) {
            continue;
        }
        CHECK(reader->GetLastRunTraceForTesting().empty());
        CHECK(reader->Execute(&error));
        const std::vector<int32_t> trace =
            reader->GetLastRunTraceForTesting();
        const size_t steps = file->steps.size();
        // Each step's place in the trace, or -1.
        std::vector<int64_t> place(steps, -1);
        bool once = true;
        for (size_t k = 0; k < trace.size(); ++k) {
            const int32_t step = trace[k];
            if (step < 0 || size_t(step) >= steps ||
                place[size_t(step)] >= 0) {
                once = false;
                continue;
            }
            place[size_t(step)] = int64_t(k);
        }
        CHECK(once);
        size_t ran = 0;
        bool listed = true;
        bool ordered = true;
        for (size_t i = 0; i < steps; ++i) {
            const bool stepRan = reader->GetStepRanForTesting(i);
            ran += stepRan ? 1 : 0;
            listed = listed && stepRan == (place[i] >= 0);
            if (place[i] < 0) {
                continue;
            }
            for (const int32_t pred : file->steps[i].preds) {
                ordered = ordered && pred >= 0 && size_t(pred) < steps &&
                          place[size_t(pred)] < place[i];
            }
        }
        CHECK(listed && ordered);
        CHECK(trace.size() == ran && ran > 0);
        std::printf("%s: %zu of %zu step(s) ran, in order\n", what.c_str(),
                    trace.size(), steps);
    }
}

int
main(int argc, char **argv)
{
    PlugRegistry::GetInstance().RegisterPlugins(
        RIGEXEC_SCHEMA_RESOURCE_DIR);
    TestComputedEnvelopes();
    TestStaticWeightEnvelope();
    TestAvarDrivenDynamicEnvelope();
    TestChainDrivenEnvelope();
    TestComputedOpenRefusals();
    TestFloatChainCurveClamp();
    TestDoubleChainInFloat();
    TestMatrixChain();
    TestVec3fChain();
    TestDisabledRevision();
    TestEnvelopeRevisions();
    TestPhasedConsumers();
    TestOraclePublicationGeneration();
    TestScalarRawKindPreservesRecords();
    TestDoubleTailCycle();
    TestDefaultReadPhaseRoundTrip();
    TestNonFiniteBase();
    TestSampledInputWithNoValue();
    TestComputedChainsFixture();
    TestIkSpaceFixture();
    TestFrameMatrixGates();
    TestUnusableComposedRestUsesLocalMatrixFallback();
    TestSpaceRestMovesWithItsRests();
    TestRepeatedSetRunsNothing();
    TestHeldDragCountsValueEditWork();
    TestRunTraceOrder();

    std::string examplesDir = RIGEXEC_EXAMPLES_DIR;
    if (argc > 1) {
        examplesDir = argv[1];
    }
    TestConstraintAndAvarReadersReplay(examplesDir);
    TestConstraintArrayDiagnostics(examplesDir);
    TestSetInputOnPhasedReader();
    TestSetInputOnHop();
    TestSetInputOnLadderInput();
    TestSetInputOnChainInput();
    TestSetInputOnChainTarget();
    TestRegisteredReadRefusals();
    TestConnectedBridgeRefreshExport();
    TestSpaceSwitchVersionFixtures();
    TestHandBuiltSwitchReads();
    TestSpaceSwitchIndexDrag();
    TestSpaceSwitchOpenRefusals();
    TestSpaceSwitchCarryDrag();
    TestPhasedRigsRunCones(examplesDir);
    bool sawBaking = false;
    for (const RigExecExampleFixture &fixture : kRigExecExampleFixtures) {
        if (!fixture.bakesToday) {
            continue;
        }
        sawBaking = true;
        const std::string stagePath =
            examplesDir + "/" + fixture.stage;
        const std::vector<double> frames = _ParseFrames(fixture.frames);
        CHECK(!frames.empty());
        if (frames.empty()) {
            continue;
        }
        // A `static` row holds an animated source in static data at the
        // bake time: it plays that time alone.
        _TestFixture(fixture.stage, stagePath, frames,
                     std::string(fixture.animation) == "static");
    }
    CHECK(sawBaking);
    CHECK(checkedSolverSemanticRequirements);
    CHECK(checkedRefreshRecords && checkedRefreshCarries && checkedRefreshGuards);

    if (failures == 0) {
        std::printf("testRigExecRuntimePose: all tests passed "
                    "(%d stage(s) compared over %d frame(s))\n",
                    comparedFixtures, comparedFrames);
        return 0;
    }
    std::printf("testRigExecRuntimePose: %d failures "
                "(%d stage(s) compared over %d frame(s))\n", failures,
                comparedFixtures, comparedFrames);
    return 1;
}
