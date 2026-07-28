//
// Diagnostic probe: isolates which phase of evaluator compilation emits
// the nonfatal "Applying predicate to invalid prim" coding error.
//
#include "rigExec/rigEvaluator.h"
#include "rigExec/tapSet.h"

#include "pxr/base/plug/registry.h"
#include "pxr/base/tf/errorMark.h"
#include "pxr/base/tf/pathUtils.h"
#include "pxr/usd/usd/stage.h"

#include <cstdio>
#include <string>

using namespace rigExec;

static void
Report(const char *phase, TfErrorMark &mark)
{
    if (mark.IsClean()) {
        std::printf("CLEAN  %s\n", phase);
    } else {
        std::printf("DIRTY  %s:\n", phase);
        for (auto it = mark.GetBegin(); it != mark.GetEnd(); ++it) {
            std::printf("   %s\n", it->GetCommentary().c_str());
        }
    }
    mark.Clear();
}

int
main(int argc, char **argv)
{
    const std::string examplesDir = argv[1];
    PlugRegistry::GetInstance().RegisterPlugins(
        TfAbsPath(examplesDir + "/../plugin/rigExecSchema/resources"));

    // Phase A: stage open.
    TfErrorMark mark;
    UsdStageRefPtr stage = UsdStage::Open(examplesDir + "/ArmRig.usda");
    Report("stage open", mark);

    // Phase B: full evaluator compile.
    {
        UsdStageRefPtr s2 = UsdStage::Open(examplesDir + "/ArmRig.usda");
        RigExecRigEvaluator evaluator(s2, SdfPath("/ArmAsset/Rig"));
        TfErrorMark compileMark;
        evaluator.Compile(nullptr);
        Report("evaluator Compile", compileMark);

        TfErrorMark evalMark;
        evaluator.Evaluate(UsdTimeCode::Default());
        Report("evaluator Evaluate", evalMark);
    }

    // Phase C: a rig whose solvers include RigExecRibbon.
    //
    // This used to exercise chain compilation alone; there is no compiler
    // any more. The ribbon is what replaced its last remaining pass, and it
    // is the one solver whose inputs arrive as attribute value overrides
    // rather than from authored scene description, so a bad override key
    // shows up here as a coding error rather than as quietly empty geometry.
    {
        UsdStageRefPtr s3 =
            UsdStage::Open(examplesDir + "/05_TwistRibbonSpine.usda");
        TfErrorMark ribbonMark;
        RigExecRigEvaluator ribbon(s3, SdfPath("/SpineAsset/Rig"));
        ribbon.Compile();
        ribbon.Evaluate(UsdTimeCode(1024));
        Report("ribbon compile + evaluate", ribbonMark);
    }

    // Phase D: plain tap set over authored providers only (no compile).
    {
        UsdStageRefPtr s4 = UsdStage::Open(examplesDir + "/ArmRig.usda");
        TfErrorMark tapMark;
        RigExecTapSet taps(s4);
        taps.Add(RigExecValueAddress::Prim(
            SdfPath("/ArmAsset/Rig/Joints/Wrist"),
            TfToken("computePointFrame")));
        taps.Prepare();
        taps.Evaluate(UsdTimeCode::Default());
        Report("plain taps (authored only)", tapMark);
    }

    return 0;
}
