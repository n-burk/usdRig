#include "rigExec/bakedExecCrossCheckRows.h"
#include "rigExec/bakedProgram.h"
#include "rigExec/bakedProgramImpl.h"
#include "rigExec/rigEvaluator.h"
#include "rigExec/types.h"
#include "pxr/base/plug/registry.h"
#include "pxr/usd/usd/primRange.h"
#include <cmath>
#include <iostream>
#include <limits>
#include <set>

using namespace rigExec;
PXR_NAMESPACE_USING_DIRECTIVE
#define CHECK(expression) do { if (!(expression)) { \
    std::cerr << "failed at " << __LINE__ << ": " << #expression << '\n'; return 1; } } while (false)

namespace {
bool Bump(VtValue *value)
{
    if (value->IsHolding<GfMatrix4d>()) {
        auto m = value->UncheckedGet<GfMatrix4d>();
        m[3][0] = std::nextafter(m[3][0],std::numeric_limits<double>::infinity());
        *value = VtValue(m); return true;
    }
    if (value->IsHolding<RigExecPointFrame>()) {
        auto frame = value->UncheckedGet<RigExecPointFrame>();
        frame.points[0][0] = std::nextafter(frame.points[0][0],std::numeric_limits<double>::infinity());
        *value = VtValue(frame); return true;
    }
    if (value->IsHolding<RigExecPointFrameArray>()) {
        auto array = value->UncheckedGet<RigExecPointFrameArray>();
        if (array.frames.empty()) return false;
        array.frames[0].points[0][0] = std::nextafter(array.frames[0].points[0][0],std::numeric_limits<double>::infinity());
        *value = VtValue(array); return true;
    }
    if (value->IsHolding<RigExecWeightPacket>()) {
        auto packet = value->UncheckedGet<RigExecWeightPacket>();
        if (!packet.values.empty()) packet.values[0] = std::nextafter(packet.values[0],std::numeric_limits<float>::infinity());
        else packet.defaultWeight = std::nextafter(packet.defaultWeight,std::numeric_limits<float>::infinity());
        *value = VtValue(packet); return true;
    }
    if (value->IsHolding<std::string>()) {
        *value=VtValue(value->UncheckedGet<std::string>()+" deliberate witness corruption");
        return true;
    }
    return false;
}
}

// A stage path and reporting frame make this a supported fixture regression,
// using the native program's real bound inputs and stored outputs.
int main(int argc,char **argv)
{
    CHECK(argc == 3);
    RigExecLoadComputations();
    PlugRegistry::GetInstance().RegisterPlugins(RIGEXEC_SCHEMA_RESOURCE_DIR);
    const auto stage = UsdStage::Open(argv[1]); CHECK(stage);
    SdfPath rigPath;
    for (const auto &prim : stage->Traverse()) if (prim.GetTypeName() == "RigExecRoot") { rigPath = prim.GetPath(); break; }
    CHECK(!rigPath.IsEmpty());
    RigExecRigEvaluator evaluator(stage,rigPath);
    evaluator.SetSolverGuidesEnabled(true);
    std::vector<std::string> errors;
    CHECK(evaluator.Compile(&errors));
    const auto *program = evaluator.GetBakedProgram(); CHECK(program);
    std::string error;
    const bool enabled=RigExecBakedProgramTesting::EnableExecCrossCheck(*program,&error);
    if(!enabled)std::cerr << "exec checker setup: " << error << '\n';
    CHECK(enabled);
    const auto rows = RigExecBakedProgramTesting::ExecCrossCheckRows(*program); CHECK(rows);
    const UsdTimeCode time(std::stod(argv[2]));
    CHECK(evaluator.Evaluate(time).valid);
    auto report = rows->Evaluate(time);
    for (const auto &line : report.diagnostics) std::cerr << line << '\n';
    CHECK(report.Passed()); CHECK(report.checked > 0);
    const auto clean = rows->Inputs();
    const auto &descriptors = rows->Descriptors(); CHECK(clean.size() == descriptors.size());
    // These supported fixtures must judge at least one row of every mirrored
    // kind they contain. A silently all-skipped kind cannot pass the suite.
    std::set<uint32_t> presentKinds;
    for(const auto &descriptor:descriptors) if(!descriptor.skipOnly)presentKinds.insert(descriptor.kind);
    for(const auto kind:presentKinds) CHECK(report.checkedByKind[kind]>0);
    std::vector<char> eligible(clean.size(),0);
    for (size_t i=0;i<clean.size();++i) eligible[i]=rows->WasChecked(i);
    std::set<uint32_t> eligibleKinds;
    size_t mutations = 0;
    for (size_t i = 0; i < clean.size(); ++i) {
        if (!eligible[i]) continue;
        eligibleKinds.insert(descriptors[i].kind);
        auto &input = rows->TestingInputs()[i];
        CHECK(Bump(&input.expected));
        report = rows->Evaluate(time);
        CHECK(!report.Passed() && report.failed == 1);
        CHECK(report.diagnostics.size() == 1 && report.diagnostics[0].find(descriptors[i].key) == 0);
        rows->TestingInputs()[i] = clean[i]; ++mutations;
        CHECK(!input.overrides.empty());
        input.overrides.erase(input.overrides.begin());
        report = rows->Evaluate(time);
        CHECK(!report.Passed() && report.failed == 1);
        CHECK(report.diagnostics.size() == 1 && report.diagnostics[0].find(descriptors[i].key) == 0);
        CHECK(report.diagnostics[0].find("missing bound input override") != std::string::npos);
        rows->TestingInputs()[i] = clean[i];
    }
    CHECK(mutations > 0);
    report = rows->Evaluate(time); CHECK(report.Passed());
    for (const auto kind : eligibleKinds) CHECK(report.checkedByKind[kind] > 0);
    // A clean rerun retains each skipped operation's matching input witness.
    CHECK(evaluator.Evaluate(time).valid);
    CHECK(rows->Evaluate(time).Passed());
    std::cout << "exec rows: checked=" << report.checked << " mutated=" << mutations << '\n';
    return 0;
}
