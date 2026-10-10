#include "sceneDispatch.h"

#include <cstdio>
#include <cstdlib>

namespace rigExec {
namespace {
RigExecSceneHooks g_hooks;

template <class Fn>
Fn Hook(Fn fn, const char *feature)
{
    if (!fn) RigExecSceneMissing(feature);
    return fn;
}
}

void RigExecInstallSceneHooks(const RigExecSceneHooks &hooks) { g_hooks = hooks; }

void RigExecSceneMissing(const char *feature)
{
    std::fprintf(stderr,
        "RigExec scene library is not linked; %s cannot run\n", feature);
    std::fflush(stderr);
    std::_Exit(EXIT_FAILURE);
}

bool RigExecDispatchCaptureSceneDescriptors(
    const UsdStageRefPtr &stage, const SdfPath &rigRoot,
    const std::vector<UsdTimeCode> &identities, RigExecSceneDescriptors *result,
    std::string *error)
{
    return Hook(g_hooks.capture, "scene lowering")(
        stage, rigRoot, identities, result, error);
}

bool RigExecDispatchSelectSceneDescriptorIdentities(
    const RigExecSceneDescriptors &scene, const std::vector<UsdTimeCode> &identities,
    RigExecSceneDescriptors *result, std::string *error)
{
    return Hook(g_hooks.select, "scene lowering")(scene, identities, result, error);
}

bool RigExecDispatchLowerScenePoseInterpolator(
    const RigExecSceneDescriptors &scene, const SdfPath &path,
    RigExecScenePoseInterpolatorDescriptor *descriptor, std::string *error)
{
    return Hook(g_hooks.lowerPoseInterpolator, "scene lowering")(
        scene, path, descriptor, error);
}

bool RigExecDispatchLowerSceneSpaceSwitch(
    const RigExecSceneDescriptors &scene, const SdfPath &path,
    RigExecSceneSpaceSwitchDescriptor *descriptor, std::string *error)
{
    return Hook(g_hooks.lowerSpaceSwitch, "scene lowering")(
        scene, path, descriptor, error);
}

bool RigExecDispatchLowerSceneGeometry(
    const RigExecSceneDescriptors &scene, const SdfPath &mover, const SdfPath &target,
    RigExecSceneGeometryDescriptor *descriptor, std::string *error,
    const RigExecGeometryRecord *normalized)
{
    return Hook(g_hooks.lowerGeometry, "scene lowering")(
        scene, mover, target, descriptor, error, normalized);
}

bool RigExecDispatchLowerSceneSolver(
    const RigExecSceneDescriptors &scene, const SdfPath &path,
    const std::map<SdfPath, RigExecPointFrame> &providerRests,
    RigExecSceneSolverDescriptor *descriptor, std::string *error)
{
    return Hook(g_hooks.lowerSolver, "scene lowering")(
        scene, path, providerRests, descriptor, error);
}

bool RigExecDispatchLowerSceneConstraint(
    const RigExecSceneDescriptors &scene, const SdfPath &path,
    const SdfPathVector &targets, RigExecSceneConstraintDescriptor *descriptor,
    std::string *error)
{
    return Hook(g_hooks.lowerConstraint, "scene lowering")(
        scene, path, targets, descriptor, error);
}

bool RigExecDispatchBindAutoClavicle(
    const RigExecSceneDescriptors &scene, const SdfPath &path,
    const RigExecSceneGraphBindingContext &context, RigExecValueId incoming,
    RigExecValueId output, RigExecBoundAutoClavicle *operation, std::string *error)
{
    return Hook(g_hooks.bindAutoClavicle, "scene lowering")(
        scene, path, context, incoming, output, operation, error);
}

bool RigExecDispatchRunAutoClavicle(
    const RigExecBoundAutoClavicle &operation, RigExecTypedValueStore *values,
    std::string *error)
{
    return Hook(g_hooks.runAutoClavicle, "scene lowering")(operation, values, error);
}

}
