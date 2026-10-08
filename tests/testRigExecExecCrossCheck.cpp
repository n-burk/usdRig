#include "rigExec/bakedExecCrossCheck.h"
#include "rigExec/bakedExecCrossCheckRows.h"
#include "rigExec/bakedProgramImpl.h"
#include "rigExec/rigEvaluator.h"
#include "pxr/usd/sdf/layer.h"
#include "rigExec/goldenPose.h"
#include "rigExec/types.h"
#include "pxr/base/plug/registry.h"
#include "pxr/usd/usd/attribute.h"
#include <cmath>
#include <iostream>
#include <limits>

using namespace rigExec;
PXR_NAMESPACE_USING_DIRECTIVE
#define CHECK(expression) do { if (!(expression)) { \
    std::cerr << "failed at " << __LINE__ << ": " << #expression << '\n'; return 1; } } while (false)


int TestIndependentConstraintRows()
{
    const char *types[]={"RigExecAimConstraint","RigExecRotationConstraint","RigExecParentConstraint"};
    for(const auto *type:types) for(const bool geometry:{false,true}) {
        const auto stage=UsdStage::CreateInMemory();
        const std::string target=geometry?"/World/Target.points":"/World/Target";
        const std::string source=std::string("#usda 1.0\n")+
            "def Xform \"World\" {\n"
            "def Mesh \"Target\" {\n point3f[] points=[(0,0,0),(1,0,0),(0,1,0)]\n }\n"
            "def Xform \"Source\" {\n double3 xformOp:translate=(0,0,2)\n uniform token[] xformOpOrder=[\"xformOp:translate\"]\n }\n"
            "def RigExecRoot \"Rig\" {\n def Scope \"Movers\" {\n def "+type+
            " \"C\" (apiSchemas=[\"RigExecMoverAPI\"]) {\n rel rigExec:moves=<"+target+
            ">\n rel rigExec:sources=</World/Source>\n }\n }\n }\n }\n";
        CHECK(stage->GetRootLayer()->ImportFromString(source));
        RigExecRigEvaluator evaluator(stage,SdfPath("/World/Rig"));
        std::vector<std::string> errors;
        CHECK(evaluator.Compile(&errors));
        CHECK(evaluator.Evaluate(UsdTimeCode::Default()).valid);
        const auto *program=evaluator.GetBakedProgram();CHECK(program);
        std::string error;
        CHECK(RigExecBakedProgramTesting::EnableExecCrossCheck(*program,&error));
        const auto rows=RigExecBakedProgramTesting::ExecCrossCheckRows(*program);CHECK(rows);
        // Instrumentation attached after admission needs a fresh body capture.
        const_cast<RigExecBakedProgram *>(program)->BumpProgramStamp();
        CHECK(evaluator.Evaluate(UsdTimeCode::Default()).valid);
        auto report=rows->Evaluate(UsdTimeCode::Default());
        for(const auto &line:report.diagnostics) std::cerr<<line<<'\n';
        CHECK(report.Passed());
        CHECK(report.checkedByKind[uint32_t(RigExecBakedStepKind::Constraint)]==1);
        const auto clean=rows->Inputs();
        size_t observed=0;
        for(size_t i=0;i<rows->Descriptors().size();++i) {
            const auto &descriptor=rows->Descriptors()[i];
            if(!descriptor.acquireReference) continue;
            ++observed;CHECK(rows->WasChecked(i));
            CHECK(clean[i].expected.IsHolding<std::string>());
            CHECK(clean[i].expected.UncheckedGet<std::string>().find("candidate=")!=std::string::npos);
            if(geometry) CHECK(clean[i].expected.UncheckedGet<std::string>().find("deltaPresent=1")!=std::string::npos);
            auto &input=rows->TestingInputs()[i];
            input.expected=VtValue(clean[i].expected.UncheckedGet<std::string>()+" corrupted candidate witness");
            report=rows->Evaluate(UsdTimeCode::Default());
            CHECK(report.failed==1 && report.checkedByKind[uint32_t(RigExecBakedStepKind::Constraint)]==1);
            CHECK(report.diagnostics.size()==1 && report.diagnostics[0].find(descriptor.key)==0);
            input=clean[i];
            CHECK(!input.overrides.empty());input.overrides.erase(input.overrides.begin());
            report=rows->Evaluate(UsdTimeCode::Default());
            CHECK(report.failed==1 && report.diagnostics[0].find("missing bound input override")!=std::string::npos);
            input=clean[i];
        }
        CHECK(observed==1);
        CHECK(evaluator.Evaluate(UsdTimeCode::Default()).valid);
        CHECK(rows->Evaluate(UsdTimeCode::Default()).Passed());
    }
    return 0;
}

int main()
{
    RigExecLoadComputations();
    PlugRegistry::GetInstance().RegisterPlugins(RIGEXEC_SCHEMA_RESOURCE_DIR);
    const auto stage = UsdStage::CreateInMemory();
    const auto parent = stage->DefinePrim(SdfPath("/Parent"), TfToken("RigExecControl"));
    const auto child = stage->DefinePrim(SdfPath("/Parent/Child"), TfToken("RigExecControl"));
    const TfToken pointFrame("computePointFrame"), parentSpace("computedParentSpace");
    RigExecPointFrame supplied;
    for (auto &point : supplied.points) point += GfVec3d(7, 3, -2);
    const GfMatrix4d expected = GfMatrix4d(1).SetTranslate(GfVec3d(7, 3, -2));
    RigExecValueOverride frameOverride{parent.GetPath(), pointFrame, TfToken(), VtValue(supplied)};

    RigExecBakedExecCrossCheck checks(stage);
    RigExecExecCheckDescriptor row;
    row.key = "parent-space /Parent/Child";
    row.kind = 17;
    row.address = RigExecValueAddress::Prim(child.GetPath(), parentSpace);
    row.requiredOverrides = {RigExecValueAddress::Prim(parent.GetPath(), pointFrame)};
    std::string error;
    CHECK(checks.Add(row, &error));
    CHECK(!checks.Add(row, &error));
    RigExecExecCheckInput input;
    input.expected = VtValue(expected);
    input.overrides = {frameOverride};
    auto report = checks.Evaluate(UsdTimeCode::Default(), {input});
    CHECK(report.Passed());
    CHECK(report.checked == 1 && report.checkedByKind[17] == 1);

    // A prepared request must observe call-local computation overrides.
    RigExecTapSet taps(stage);
    const auto tap = taps.Add(row.address);
    CHECK(taps.Prepare());
    size_t dropped = 99;
    const auto snapshot = taps.Evaluate(UsdTimeCode::Default(), {frameOverride}, &dropped);
    CHECK(snapshot.IsComplete() && dropped == 0);
    std::string actualBits, expectedBits;
    CHECK(RigExecEncodeGoldenValue(snapshot.Get(tap), &actualBits));
    CHECK(RigExecEncodeGoldenValue(VtValue(expected), &expectedBits));
    CHECK(actualBits == expectedBits);
    const auto plain = taps.Evaluate(UsdTimeCode::Default());
    CHECK(plain.IsComplete());
    CHECK(plain.Get<GfMatrix4d>(tap) != expected);

    // One ULP in the expected output names exactly the affected row.
    auto perturbed = expected;
    perturbed[3][0] = std::nextafter(perturbed[3][0], std::numeric_limits<double>::infinity());
    input.expected = VtValue(perturbed);
    report = checks.Evaluate(UsdTimeCode::Default(), {input});
    CHECK(!report.Passed() && report.failed == 1 && report.checked == 1);
    CHECK(report.diagnostics.size() == 1 && report.diagnostics[0].find(row.key) == 0);
    input.expected = VtValue(expected);

    // Omitting a declared bound input must not quietly compare against stage data.
    input.overrides.clear();
    report = checks.Evaluate(UsdTimeCode::Default(), {input});
    CHECK(!report.Passed() && report.checked == 0 && report.failed == 1);
    CHECK(report.diagnostics[0].find("missing bound input override") != std::string::npos);
    input.overrides = {frameOverride};
    input.overrides.push_back({SdfPath("/Missing"), pointFrame, TfToken(), VtValue(supplied)});
    input.overrides.push_back({parent.GetPath(), TfToken(), TfToken("missing:attribute"), VtValue(1.0)});
    report = checks.Evaluate(UsdTimeCode::Default(), {input});
    CHECK(!report.Passed() && report.dropped == 2 && report.checked == 0);
    input.overrides = {frameOverride, frameOverride};
    report = checks.Evaluate(UsdTimeCode::Default(), {input});
    CHECK(!report.Passed() && report.checked == 0);

    // A result override would make the independent check vacuous and is refused.
    input.overrides = {frameOverride,
        {child.GetPath(), parentSpace, TfToken(), VtValue(expected)}};
    report = checks.Evaluate(UsdTimeCode::Default(), {input});
    CHECK(!report.Passed() && report.checked == 0);
    input.skip = RigExecExecCheckSkip::ArrayInput;
    report = checks.Evaluate(UsdTimeCode::Default(), {input});
    CHECK(report.Passed() && report.checked == 0 && report.skipped[input.skip] == 1);
    CHECK(!checks.Evaluate(UsdTimeCode::Default(), {}).Passed());

    // Registering an output as a required input is also refused at compile time.
    row.key = "vacuous";
    row.requiredOverrides = {row.address};
    CHECK(!checks.Add(row, &error));
    return TestIndependentConstraintRows();
}
