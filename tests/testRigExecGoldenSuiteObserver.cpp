#include "rigExec/goldenSuite.h"
#include "rigExec/rigEvaluator.h"
#include <string>

using namespace rigExec;
PXR_NAMESPACE_USING_DIRECTIVE

// Protocol-only subprocess fixture. These temporary synthetic captures are
// never numerical judges for a production evaluator or reference suite.
int main(int argc, char **argv)
{
    if (argc != 2) return 2;
    const std::string scenario(argv[1]);
    if (scenario == "empty") return 0;
    auto a = RigExecGoldenSuiteObserver::Create(SdfPath(scenario == "rig" ? "/Changed" : "/A"));
    auto b = scenario == "missing-evaluator" ? nullptr : RigExecGoldenSuiteObserver::Create(SdfPath("/B"));
    if (!a) return 2;
    RigExecRigPose pose;
    pose.valid = true;
    pose.time = UsdTimeCode(1);
    pose.movedProperties[SdfPath("/A.value")] = VtValue(scenario == "bits" ? 0.0 : -0.0);
    a->Record(pose);
    if (b) b->Record(pose);
    if (scenario != "missing-generation") {
        pose.time = UsdTimeCode(scenario == "time" ? 3 : 2);
        a->Record(pose);
    }
    if (scenario == "extra-generation") a->Record(pose);
    if (scenario == "extra-evaluator") {
        auto c = RigExecGoldenSuiteObserver::Create(SdfPath("/C"));
        c->Record(pose);
    }
    if (scenario == "unclosed") (void)b.release();
    // Reverse destruction is ordinary unique_ptr lifetime; the index uses
    // evaluator identity, not concurrent destructor interleaving.
    return 0;
}
