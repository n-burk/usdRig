// Function pointers the scene-lowering library installs. The evaluator calls
// the dispatch functions below; those forward here. A missing hook aborts, so
// a compile cannot succeed without the library. The frame-path kernels live
// in librigExec and are not dispatched.
#ifndef RIGEXEC_SCENE_DISPATCH_H
#define RIGEXEC_SCENE_DISPATCH_H

#include "rigExecScene/autoClavicleGraph.h"
#include "rigExecScene/constraintSceneLowering.h"
#include "rigExecScene/geometrySceneLowering.h"
#include "rigExecScene/poseSceneLowering.h"
#include "rigExecScene/sceneDescriptors.h"
#include "rigExecScene/solverSceneLowering.h"

#include "pxr/usd/usd/stage.h"

#include <map>
#include <string>
#include <vector>

namespace rigExec {

struct RigExecSceneHooks {
    bool (*capture)(const UsdStageRefPtr &, const SdfPath &,
                    const std::vector<UsdTimeCode> &, RigExecSceneDescriptors *,
                    std::string *) = nullptr;
    bool (*select)(const RigExecSceneDescriptors &, const std::vector<UsdTimeCode> &,
                   RigExecSceneDescriptors *, std::string *) = nullptr;
    bool (*lowerPoseInterpolator)(const RigExecSceneDescriptors &, const SdfPath &,
                                  RigExecScenePoseInterpolatorDescriptor *,
                                  std::string *) = nullptr;
    bool (*lowerSpaceSwitch)(const RigExecSceneDescriptors &, const SdfPath &,
                             RigExecSceneSpaceSwitchDescriptor *, std::string *) = nullptr;
    bool (*lowerGeometry)(const RigExecSceneDescriptors &, const SdfPath &, const SdfPath &,
                          RigExecSceneGeometryDescriptor *, std::string *,
                          const RigExecGeometryRecord *) = nullptr;
    bool (*lowerSolver)(const RigExecSceneDescriptors &, const SdfPath &,
                        const std::map<SdfPath, RigExecPointFrame> &,
                        RigExecSceneSolverDescriptor *, std::string *) = nullptr;
    bool (*lowerConstraint)(const RigExecSceneDescriptors &, const SdfPath &,
                            const SdfPathVector &, RigExecSceneConstraintDescriptor *,
                            std::string *) = nullptr;
    bool (*bindAutoClavicle)(const RigExecSceneDescriptors &, const SdfPath &,
                             const RigExecSceneGraphBindingContext &, RigExecValueId,
                             RigExecValueId, RigExecBoundAutoClavicle *,
                             std::string *) = nullptr;
    bool (*runAutoClavicle)(const RigExecBoundAutoClavicle &, RigExecTypedValueStore *,
                            std::string *) = nullptr;
};

void RigExecInstallSceneHooks(const RigExecSceneHooks &hooks);
[[noreturn]] void RigExecSceneMissing(const char *feature);

bool RigExecDispatchCaptureSceneDescriptors(
    const UsdStageRefPtr &stage, const SdfPath &rigRoot,
    const std::vector<UsdTimeCode> &identities, RigExecSceneDescriptors *result,
    std::string *error);

bool RigExecDispatchSelectSceneDescriptorIdentities(
    const RigExecSceneDescriptors &scene, const std::vector<UsdTimeCode> &identities,
    RigExecSceneDescriptors *result, std::string *error);

bool RigExecDispatchLowerScenePoseInterpolator(
    const RigExecSceneDescriptors &scene, const SdfPath &path,
    RigExecScenePoseInterpolatorDescriptor *descriptor, std::string *error);

bool RigExecDispatchLowerSceneSpaceSwitch(
    const RigExecSceneDescriptors &scene, const SdfPath &path,
    RigExecSceneSpaceSwitchDescriptor *descriptor, std::string *error);

bool RigExecDispatchLowerSceneGeometry(
    const RigExecSceneDescriptors &scene, const SdfPath &mover, const SdfPath &target,
    RigExecSceneGeometryDescriptor *descriptor, std::string *error,
    const RigExecGeometryRecord *normalized);

bool RigExecDispatchLowerSceneSolver(
    const RigExecSceneDescriptors &scene, const SdfPath &path,
    const std::map<SdfPath, RigExecPointFrame> &providerRests,
    RigExecSceneSolverDescriptor *descriptor, std::string *error);

bool RigExecDispatchLowerSceneConstraint(
    const RigExecSceneDescriptors &scene, const SdfPath &path,
    const SdfPathVector &targets, RigExecSceneConstraintDescriptor *descriptor,
    std::string *error);

bool RigExecDispatchBindAutoClavicle(
    const RigExecSceneDescriptors &scene, const SdfPath &path,
    const RigExecSceneGraphBindingContext &context, RigExecValueId incoming,
    RigExecValueId output, RigExecBoundAutoClavicle *operation, std::string *error);

bool RigExecDispatchRunAutoClavicle(
    const RigExecBoundAutoClavicle &operation, RigExecTypedValueStore *values,
    std::string *error);

}
#endif
