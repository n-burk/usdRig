// Installs the lowering entry points the evaluator calls. Linked into
// librigExecScene, not librigExec.
#include "rigExec/sceneDispatch.h"
#include "rigExecScene/usdSceneAccess.h"

namespace rigExec {
namespace {

bool CaptureStage(const UsdStageRefPtr &stage, const SdfPath &rigRoot,
                  const std::vector<UsdTimeCode> &identities,
                  RigExecSceneDescriptors *result, std::string *error)
{
    const RigExecUsdSceneAccess source(stage);
    return RigExecCaptureSceneDescriptors(source, rigRoot, identities, result, error);
}

struct Install {
    Install()
    {
        RigExecSceneHooks hooks;
        hooks.capture = &CaptureStage;
        hooks.select = &RigExecSelectSceneDescriptorIdentities;
        hooks.lowerPoseInterpolator = &RigExecLowerScenePoseInterpolator;
        hooks.lowerSpaceSwitch = &RigExecLowerSceneSpaceSwitch;
        hooks.lowerGeometry = &RigExecLowerSceneGeometry;
        hooks.lowerSolver = &RigExecLowerSceneSolver;
        hooks.lowerConstraint = &RigExecLowerSceneConstraint;
        hooks.bindAutoClavicle = &RigExecBindAutoClavicle;
        hooks.runAutoClavicle = &RigExecRunAutoClavicle;
        RigExecInstallSceneHooks(hooks);
    }
};

const Install installed;
}

void RigExecSceneLinkAnchor() {}

}
