// Affine frame kernels. Matrices use USD row-vector conventions.
// Pure value inputs: no stage, Exec context, plugin or evaluator dependency.
// Numerical provenance and supported operations: docs/references.md.
#ifndef RIGEXEC_MATH_AFFINE_FRAME_KERNELS_H
#define RIGEXEC_MATH_AFFINE_FRAME_KERNELS_H
#include "pxr/base/gf/matrix4d.h"
#include "pxr/base/gf/vec3i.h"
#include <string>
#include <vector>
PXR_NAMESPACE_USING_DIRECTIVE
namespace rigExec {
struct RigExecAffineFrameInputs {
    GfMatrix4d incoming{1};
    bool preserveLocation{false};
    GfMatrix4d origin{1};
    GfMatrix4d inverseBind{1};
    std::string operation{"COPY_LOCATION"};
    double influence{1};
    std::vector<int> targetIndices{};
    std::vector<int> objectIndices{};
    std::vector<GfMatrix4d> targetBinds{};
    std::vector<double> targetWeights{};
    GfVec3d pivot{0,0,0};
    bool dualQuaternion{false};
    bool currentPivot{false};
    std::string ownerSpace{"WORLD"};
    std::string targetSpace{"WORLD"};
    int axisMask{7};
    int invertMask{0};
    bool offset{false};
    bool uniformScale{false};
    bool scaleAdd{false};
    double power{1};
    std::string rotationMix{"REPLACE"};
    bool removeTargetShear{false};
    std::string mapFrom{"LOCATION"};
    GfVec3d mapFromMin{0,0,0};
    GfVec3d mapFromMax{1,1,1};
    GfVec3d mapToMin{0,0,0};
    GfVec3d mapToMax{1,1,1};
    GfVec3i mapAxes{0,1,2};
    bool mapExtrapolate{false};
    std::string mapMix{"ADD"};
    std::string trackAxis{"TRACK_Y"};
    std::string keepAxis{"PLANE_X"};
    std::string volume{"NO_VOLUME"};
    double restLength{1};
    double bulge{1};
    double bulgeMin{0};
    double bulgeMax{1};
    double bulgeSmooth{0};
    bool useBulgeMin{false};
    bool useBulgeMax{false};
    GfVec3d targetOffset{0,0,0};
    GfMatrix4d ownerLocal{1};
    GfMatrix4d ownerRest{1};
    GfMatrix4d ownerParentRest{1};
    bool ownerHasParent{false};
    bool ownerInheritRotation{true};
    bool ownerLocalLocation{true};
    std::string ownerInheritScale{"FULL"};
    GfMatrix4d sourceLocal{1};
    GfMatrix4d sourceRest{1};
    GfMatrix4d sourceParentRest{1};
    bool sourceHasParent{false};
    bool sourceInheritRotation{true};
    bool sourceLocalLocation{true};
    std::string sourceInheritScale{"FULL"};
    GfMatrix4d inverseMesh{1};
    bool fromBind{true};
    bool followOnly{false};
    GfMatrix4d local{1};
    bool useIncoming{false};
    double tx{0};
    double ty{0};
    double tz{0};
    double rx{0};
    double ry{0};
    double rz{0};
    double sx{1};
    double sy{1};
    double sz{1};
    std::string spaceKind{"pose"};
    GfMatrix4d parentRest{1};
    bool hasParent{false};
    bool inheritRotation{true};
    bool localLocation{true};
    bool connected{false};
    std::string inheritScale{"FULL"};
    GfMatrix4d targetRest{1};
    GfMatrix4d object{1};
    GfMatrix4d source{1};
    std::vector<GfMatrix4d> targets{};
    std::vector<GfMatrix4d> targetObjects{};
    GfMatrix4d sourceObject{1};
    GfMatrix4d ownerObject{1};
    GfMatrix4d customSpace{1};
    GfMatrix4d ownerParent{1};
    GfMatrix4d sourceParent{1};
    GfMatrix4d owner{1};
    GfMatrix4d parent{1};
};
GfMatrix4d RigExecComputeArmatureParent(const RigExecAffineFrameInputs &inputs);
GfMatrix4d RigExecComputeBoneFrame(const RigExecAffineFrameInputs &inputs);
GfMatrix4d RigExecComputeSkinInfluence(const RigExecAffineFrameInputs &inputs);
GfMatrix4d RigExecComputeMappedFrame(const RigExecAffineFrameInputs &inputs);
GfMatrix4d RigExecComputeCopyTransforms(const RigExecAffineFrameInputs &inputs);
GfMatrix4d RigExecComputeConstraintFrame(const RigExecAffineFrameInputs &inputs);
}
#endif
