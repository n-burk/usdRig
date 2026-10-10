// Declarative Exec computations; all pose inputs are USD attributes/providers.
// Coordinate/numerical contracts and primary sources: docs/references.md.
#include "rigExec/types.h"
#include "rigExecMath/affineFrameKernels.h"
#include "pxr/base/tf/staticTokens.h"
#include "pxr/base/gf/rotation.h"
#include "pxr/base/gf/quatd.h"
#include "pxr/base/gf/matrix3d.h"
#include "pxr/exec/exec/builtinComputations.h"
#include "pxr/exec/exec/registerSchema.h"
#include "pxr/exec/vdf/context.h"
#include "pxr/exec/vdf/readIterator.h"
#include "pxr/base/vt/array.h"
#include <algorithm>
#include <cmath>

PXR_NAMESPACE_USING_DIRECTIVE
TF_DEFINE_PRIVATE_TOKENS(_armature,
    ((matrix,"outputs:matrix")) ((local,"inputs:local"))
    ((inverseBind,"inputs:inverseBind")) ((parent,"rigExec:parent"))
    ((source,"rigExec:source")) ((sourceObject,"rigExec:sourceObject"))
    ((incoming,"inputs:incoming")) ((useIncoming,"inputs:useIncoming"))
    ((preserveLocation,"inputs:preserveLocation"))
    ((tx,"inputs:tx")) ((ty,"inputs:ty")) ((tz,"inputs:tz"))
    ((rx,"inputs:rx")) ((ry,"inputs:ry")) ((rz,"inputs:rz"))
    ((sx,"inputs:sx")) ((sy,"inputs:sy")) ((sz,"inputs:sz"))
    (computePointFrame)
);
TF_DEFINE_PRIVATE_TOKENS(_bone,
    ((matrix,"outputs:matrix")) ((local,"inputs:local"))
    ((spaceKind,"inputs:spaceKind"))
    ((parentRest,"inputs:parentRest")) ((hasParent,"inputs:hasParent"))
    ((inheritRotation,"inputs:inheritRotation")) ((inheritScale,"inputs:inheritScale"))
    ((localLocation,"inputs:localLocation")) ((connected,"inputs:connected"))
    ((parent,"rigExec:parent")) ((object,"rigExec:sourceObject"))
    ((tx,"inputs:tx")) ((ty,"inputs:ty")) ((tz,"inputs:tz"))
    ((rx,"inputs:rx")) ((ry,"inputs:ry")) ((rz,"inputs:rz"))
    ((sx,"inputs:sx")) ((sy,"inputs:sy")) ((sz,"inputs:sz")) (computePointFrame)
);
TF_DEFINE_PRIVATE_TOKENS(_skin,
    ((matrix,"outputs:matrix")) ((inverseBind,"inputs:inverseBind"))
    ((inverseMesh,"inputs:inverseMesh")) ((fromBind,"inputs:fromBind"))
    ((followOnly,"inputs:followOnly"))
    ((owner,"rigExec:owner")) ((source,"rigExec:source"))
    ((sourceObject,"rigExec:sourceObject")) (computePointFrame)
);
TF_DEFINE_PRIVATE_TOKENS(_mapped,
    ((matrix,"outputs:matrix")) ((targetRest,"inputs:targetRest"))
    ((sourceRest,"inputs:sourceRest")) ((source,"rigExec:source"))
    (computePointFrame)
);
TF_DEFINE_PRIVATE_TOKENS(_constraint,
    ((matrix,"outputs:matrix")) ((incoming,"inputs:incoming")) ((origin,"inputs:origin"))
    ((inverseBind,"inputs:inverseBind")) ((operation,"inputs:operation"))
    ((influence,"inputs:influence"))
    ((targetIndices,"inputs:targetIndices")) ((objectIndices,"inputs:objectIndices"))
    ((targetBinds,"inputs:targetBinds")) ((targetWeights,"inputs:targetWeights"))
    ((pivot,"inputs:pivot")) ((dualQuaternion,"inputs:dualQuaternion")) ((currentPivot,"inputs:currentPivot"))
    ((targets,"rigExec:targets")) ((targetObjects,"rigExec:targetObjects"))
    ((ownerSpace,"inputs:ownerSpace")) ((targetSpace,"inputs:targetSpace"))
    ((axisMask,"inputs:axisMask")) ((invertMask,"inputs:invertMask")) ((offset,"inputs:offset"))
    ((uniformScale,"inputs:uniformScale")) ((scaleAdd,"inputs:scaleAdd")) ((power,"inputs:power"))
    ((rotationMix,"inputs:rotationMix")) ((removeTargetShear,"inputs:removeTargetShear"))
    ((mapFrom,"inputs:mapFrom")) ((mapFromMin,"inputs:mapFromMin")) ((mapFromMax,"inputs:mapFromMax"))
    ((mapToMin,"inputs:mapToMin")) ((mapToMax,"inputs:mapToMax")) ((mapAxes,"inputs:mapAxes"))
    ((mapExtrapolate,"inputs:mapExtrapolate")) ((mapMix,"inputs:mapMix"))
    ((trackAxis,"inputs:trackAxis")) ((keepAxis,"inputs:keepAxis")) ((volume,"inputs:volume"))
    ((restLength,"inputs:restLength")) ((bulge,"inputs:bulge"))
    ((bulgeMin,"inputs:bulgeMin")) ((bulgeMax,"inputs:bulgeMax")) ((bulgeSmooth,"inputs:bulgeSmooth"))
    ((useBulgeMin,"inputs:useBulgeMin")) ((useBulgeMax,"inputs:useBulgeMax"))
    ((targetOffset,"inputs:targetOffset")) ((source,"rigExec:source"))
    ((sourceObject,"rigExec:sourceObject")) ((ownerObject,"rigExec:ownerObject"))
    ((customSpace,"rigExec:customSpace")) (computePointFrame)
);
TF_DEFINE_PRIVATE_TOKENS(_localSpaces,
    ((ownerLocal,"inputs:ownerLocal")) ((ownerRest,"inputs:ownerRest")) ((ownerParentRest,"inputs:ownerParentRest"))
    ((ownerHasParent,"inputs:ownerHasParent")) ((ownerInheritRotation,"inputs:ownerInheritRotation"))
    ((ownerLocalLocation,"inputs:ownerLocalLocation")) ((ownerInheritScale,"inputs:ownerInheritScale"))
    ((sourceLocal,"inputs:sourceLocal")) ((sourceRest,"inputs:sourceRest")) ((sourceParentRest,"inputs:sourceParentRest"))
    ((sourceHasParent,"inputs:sourceHasParent")) ((sourceInheritRotation,"inputs:sourceInheritRotation"))
    ((sourceLocalLocation,"inputs:sourceLocalLocation")) ((sourceInheritScale,"inputs:sourceInheritScale"))
    ((ownerParent,"rigExec:ownerParent")) ((sourceParent,"rigExec:sourceParent"))
);
namespace {
GfMatrix4d FrameMatrix(const rigExec::RigExecPointFrame *frame) {
    GfMatrix4d result(1);
    if (!frame) return result;
    auto origin=frame->Origin();
    GfVec3d axes[3]={frame->X()-origin,frame->Y()-origin,frame->Z()-origin};
    for(int i=0;i<3;++i) for(int j=0;j<3;++j) result[i][j]=axes[i][j];
    result.SetTranslateOnly(origin);
    return result;
}
GfMatrix4d CopyTransforms(const VdfContext &ctx) {
    rigExec::RigExecAffineFrameInputs inputs;
    inputs.incoming=ctx.GetInputValue<GfMatrix4d>(_armature->incoming);
    inputs.preserveLocation=ctx.GetInputValue<bool>(_armature->preserveLocation);
    inputs.source=FrameMatrix(ctx.GetInputValuePtr<rigExec::RigExecPointFrame>(_armature->source));
    return rigExec::RigExecComputeCopyTransforms(inputs);
}
GfMatrix4d ConstraintFrame(const VdfContext &ctx) {
    rigExec::RigExecAffineFrameInputs inputs;
    inputs.incoming=ctx.GetInputValue<GfMatrix4d>(_constraint->incoming);
    inputs.origin=ctx.GetInputValue<GfMatrix4d>(_constraint->origin);
    inputs.inverseBind=ctx.GetInputValue<GfMatrix4d>(_constraint->inverseBind);
    inputs.operation=ctx.GetInputValue<TfToken>(_constraint->operation).GetString();
    inputs.influence=ctx.GetInputValue<double>(_constraint->influence);
    for(VdfReadIterator<int> it(ctx,_constraint->targetIndices);!it.IsAtEnd();++it) inputs.targetIndices.push_back(*it);
    for(VdfReadIterator<int> it(ctx,_constraint->objectIndices);!it.IsAtEnd();++it) inputs.objectIndices.push_back(*it);
    for(VdfReadIterator<GfMatrix4d> it(ctx,_constraint->targetBinds);!it.IsAtEnd();++it) inputs.targetBinds.push_back(*it);
    for(VdfReadIterator<double> it(ctx,_constraint->targetWeights);!it.IsAtEnd();++it) inputs.targetWeights.push_back(*it);
    inputs.pivot=ctx.GetInputValue<GfVec3d>(_constraint->pivot);
    inputs.dualQuaternion=ctx.GetInputValue<bool>(_constraint->dualQuaternion);
    inputs.currentPivot=ctx.GetInputValue<bool>(_constraint->currentPivot);
    inputs.ownerSpace=ctx.GetInputValue<TfToken>(_constraint->ownerSpace).GetString();
    inputs.targetSpace=ctx.GetInputValue<TfToken>(_constraint->targetSpace).GetString();
    inputs.axisMask=ctx.GetInputValue<int>(_constraint->axisMask);
    inputs.invertMask=ctx.GetInputValue<int>(_constraint->invertMask);
    inputs.offset=ctx.GetInputValue<bool>(_constraint->offset);
    inputs.uniformScale=ctx.GetInputValue<bool>(_constraint->uniformScale);
    inputs.scaleAdd=ctx.GetInputValue<bool>(_constraint->scaleAdd);
    inputs.power=ctx.GetInputValue<double>(_constraint->power);
    inputs.rotationMix=ctx.GetInputValue<TfToken>(_constraint->rotationMix).GetString();
    inputs.removeTargetShear=ctx.GetInputValue<bool>(_constraint->removeTargetShear);
    inputs.mapFrom=ctx.GetInputValue<TfToken>(_constraint->mapFrom).GetString();
    inputs.mapFromMin=ctx.GetInputValue<GfVec3d>(_constraint->mapFromMin);
    inputs.mapFromMax=ctx.GetInputValue<GfVec3d>(_constraint->mapFromMax);
    inputs.mapToMin=ctx.GetInputValue<GfVec3d>(_constraint->mapToMin);
    inputs.mapToMax=ctx.GetInputValue<GfVec3d>(_constraint->mapToMax);
    inputs.mapAxes=ctx.GetInputValue<GfVec3i>(_constraint->mapAxes);
    inputs.mapExtrapolate=ctx.GetInputValue<bool>(_constraint->mapExtrapolate);
    inputs.mapMix=ctx.GetInputValue<TfToken>(_constraint->mapMix).GetString();
    inputs.trackAxis=ctx.GetInputValue<TfToken>(_constraint->trackAxis).GetString();
    inputs.keepAxis=ctx.GetInputValue<TfToken>(_constraint->keepAxis).GetString();
    inputs.volume=ctx.GetInputValue<TfToken>(_constraint->volume).GetString();
    inputs.restLength=ctx.GetInputValue<double>(_constraint->restLength);
    inputs.bulge=ctx.GetInputValue<double>(_constraint->bulge);
    inputs.bulgeMin=ctx.GetInputValue<double>(_constraint->bulgeMin);
    inputs.bulgeMax=ctx.GetInputValue<double>(_constraint->bulgeMax);
    inputs.bulgeSmooth=ctx.GetInputValue<double>(_constraint->bulgeSmooth);
    inputs.useBulgeMin=ctx.GetInputValue<bool>(_constraint->useBulgeMin);
    inputs.useBulgeMax=ctx.GetInputValue<bool>(_constraint->useBulgeMax);
    inputs.targetOffset=ctx.GetInputValue<GfVec3d>(_constraint->targetOffset);
    inputs.ownerLocal=ctx.GetInputValue<GfMatrix4d>(_localSpaces->ownerLocal);
    inputs.ownerRest=ctx.GetInputValue<GfMatrix4d>(_localSpaces->ownerRest);
    inputs.ownerParentRest=ctx.GetInputValue<GfMatrix4d>(_localSpaces->ownerParentRest);
    inputs.ownerHasParent=ctx.GetInputValue<bool>(_localSpaces->ownerHasParent);
    inputs.ownerInheritRotation=ctx.GetInputValue<bool>(_localSpaces->ownerInheritRotation);
    inputs.ownerLocalLocation=ctx.GetInputValue<bool>(_localSpaces->ownerLocalLocation);
    inputs.ownerInheritScale=ctx.GetInputValue<TfToken>(_localSpaces->ownerInheritScale).GetString();
    inputs.sourceLocal=ctx.GetInputValue<GfMatrix4d>(_localSpaces->sourceLocal);
    inputs.sourceRest=ctx.GetInputValue<GfMatrix4d>(_localSpaces->sourceRest);
    inputs.sourceParentRest=ctx.GetInputValue<GfMatrix4d>(_localSpaces->sourceParentRest);
    inputs.sourceHasParent=ctx.GetInputValue<bool>(_localSpaces->sourceHasParent);
    inputs.sourceInheritRotation=ctx.GetInputValue<bool>(_localSpaces->sourceInheritRotation);
    inputs.sourceLocalLocation=ctx.GetInputValue<bool>(_localSpaces->sourceLocalLocation);
    inputs.sourceInheritScale=ctx.GetInputValue<TfToken>(_localSpaces->sourceInheritScale).GetString();
    for(VdfReadIterator<rigExec::RigExecPointFrame> it(ctx,_constraint->targets);!it.IsAtEnd();++it) inputs.targets.push_back(FrameMatrix(&*it));
    for(VdfReadIterator<rigExec::RigExecPointFrame> it(ctx,_constraint->targetObjects);!it.IsAtEnd();++it) inputs.targetObjects.push_back(FrameMatrix(&*it));
    inputs.source=FrameMatrix(ctx.GetInputValuePtr<rigExec::RigExecPointFrame>(_constraint->source));
    inputs.sourceObject=FrameMatrix(ctx.GetInputValuePtr<rigExec::RigExecPointFrame>(_constraint->sourceObject));
    inputs.ownerObject=FrameMatrix(ctx.GetInputValuePtr<rigExec::RigExecPointFrame>(_constraint->ownerObject));
    inputs.customSpace=FrameMatrix(ctx.GetInputValuePtr<rigExec::RigExecPointFrame>(_constraint->customSpace));
    inputs.ownerParent=FrameMatrix(ctx.GetInputValuePtr<rigExec::RigExecPointFrame>(_localSpaces->ownerParent));
    inputs.sourceParent=FrameMatrix(ctx.GetInputValuePtr<rigExec::RigExecPointFrame>(_localSpaces->sourceParent));
    return rigExec::RigExecComputeConstraintFrame(inputs);
}
GfMatrix4d SkinInfluence(const VdfContext &ctx) {
    rigExec::RigExecAffineFrameInputs inputs;
    inputs.inverseBind=ctx.GetInputValue<GfMatrix4d>(_skin->inverseBind);
    inputs.inverseMesh=ctx.GetInputValue<GfMatrix4d>(_skin->inverseMesh);
    inputs.fromBind=ctx.GetInputValue<bool>(_skin->fromBind);
    inputs.followOnly=ctx.GetInputValue<bool>(_skin->followOnly);
    inputs.owner=FrameMatrix(ctx.GetInputValuePtr<rigExec::RigExecPointFrame>(_skin->owner));
    inputs.source=FrameMatrix(ctx.GetInputValuePtr<rigExec::RigExecPointFrame>(_skin->source));
    inputs.sourceObject=FrameMatrix(ctx.GetInputValuePtr<rigExec::RigExecPointFrame>(_skin->sourceObject));
    return rigExec::RigExecComputeSkinInfluence(inputs);
}
GfMatrix4d ArmatureParent(const VdfContext &ctx) {
    rigExec::RigExecAffineFrameInputs inputs;
    inputs.local=ctx.GetInputValue<GfMatrix4d>(_armature->local);
    inputs.inverseBind=ctx.GetInputValue<GfMatrix4d>(_armature->inverseBind);
    inputs.incoming=ctx.GetInputValue<GfMatrix4d>(_armature->incoming);
    inputs.useIncoming=ctx.GetInputValue<bool>(_armature->useIncoming);
    inputs.preserveLocation=ctx.GetInputValue<bool>(_armature->preserveLocation);
    inputs.tx=ctx.GetInputValue<double>(_armature->tx);
    inputs.ty=ctx.GetInputValue<double>(_armature->ty);
    inputs.tz=ctx.GetInputValue<double>(_armature->tz);
    inputs.rx=ctx.GetInputValue<double>(_armature->rx);
    inputs.ry=ctx.GetInputValue<double>(_armature->ry);
    inputs.rz=ctx.GetInputValue<double>(_armature->rz);
    inputs.sx=ctx.GetInputValue<double>(_armature->sx);
    inputs.sy=ctx.GetInputValue<double>(_armature->sy);
    inputs.sz=ctx.GetInputValue<double>(_armature->sz);
    inputs.parent=FrameMatrix(ctx.GetInputValuePtr<rigExec::RigExecPointFrame>(_armature->parent));
    inputs.sourceObject=FrameMatrix(ctx.GetInputValuePtr<rigExec::RigExecPointFrame>(_armature->sourceObject));
    inputs.source=FrameMatrix(ctx.GetInputValuePtr<rigExec::RigExecPointFrame>(_armature->source));
    return rigExec::RigExecComputeArmatureParent(inputs);
}
GfMatrix4d BoneFrame(const VdfContext &ctx) {
    rigExec::RigExecAffineFrameInputs inputs;
    inputs.spaceKind=ctx.GetInputValue<TfToken>(_bone->spaceKind).GetString();
    inputs.local=ctx.GetInputValue<GfMatrix4d>(_bone->local);
    inputs.parentRest=ctx.GetInputValue<GfMatrix4d>(_bone->parentRest);
    inputs.hasParent=ctx.GetInputValue<bool>(_bone->hasParent);
    inputs.inheritRotation=ctx.GetInputValue<bool>(_bone->inheritRotation);
    inputs.localLocation=ctx.GetInputValue<bool>(_bone->localLocation);
    inputs.connected=ctx.GetInputValue<bool>(_bone->connected);
    inputs.inheritScale=ctx.GetInputValue<TfToken>(_bone->inheritScale).GetString();
    inputs.tx=ctx.GetInputValue<double>(_bone->tx);
    inputs.ty=ctx.GetInputValue<double>(_bone->ty);
    inputs.tz=ctx.GetInputValue<double>(_bone->tz);
    inputs.rx=ctx.GetInputValue<double>(_bone->rx);
    inputs.ry=ctx.GetInputValue<double>(_bone->ry);
    inputs.rz=ctx.GetInputValue<double>(_bone->rz);
    inputs.sx=ctx.GetInputValue<double>(_bone->sx);
    inputs.sy=ctx.GetInputValue<double>(_bone->sy);
    inputs.sz=ctx.GetInputValue<double>(_bone->sz);
    inputs.parent=FrameMatrix(ctx.GetInputValuePtr<rigExec::RigExecPointFrame>(_bone->parent));
    inputs.object=FrameMatrix(ctx.GetInputValuePtr<rigExec::RigExecPointFrame>(_bone->object));
    return rigExec::RigExecComputeBoneFrame(inputs);
}
GfMatrix4d MappedFrame(const VdfContext &ctx) {
    rigExec::RigExecAffineFrameInputs inputs;
    inputs.targetRest=ctx.GetInputValue<GfMatrix4d>(_mapped->targetRest);
    inputs.sourceRest=ctx.GetInputValue<GfMatrix4d>(_mapped->sourceRest);
    inputs.source=FrameMatrix(ctx.GetInputValuePtr<rigExec::RigExecPointFrame>(_mapped->source));
    return rigExec::RigExecComputeMappedFrame(inputs);
}
}
EXEC_REGISTER_COMPUTATIONS_FOR_SCHEMA(RigExecCopyFrame) {
    self.AttributeExpression(_armature->matrix).Callback<GfMatrix4d>(&CopyTransforms)
        .Inputs(Prim().AttributeValue<GfMatrix4d>(_armature->incoming),Prim().AttributeValue<bool>(_armature->preserveLocation),
            Prim().Relationship(_armature->source).TargetedObjects<rigExec::RigExecPointFrame>(_armature->computePointFrame).InputName(_armature->source));
}
EXEC_REGISTER_COMPUTATIONS_FOR_SCHEMA(RigExecConstraintFrame) {
    self.AttributeExpression(_constraint->matrix).Callback<GfMatrix4d>(&ConstraintFrame)
        .Inputs(Prim().AttributeValue<GfMatrix4d>(_constraint->incoming),Prim().AttributeValue<GfMatrix4d>(_constraint->origin),
            Prim().AttributeValue<GfMatrix4d>(_constraint->inverseBind),Prim().AttributeValue<TfToken>(_constraint->operation),
            Prim().AttributeValue<double>(_constraint->influence),
            Prim().AttributeValue<int>(_constraint->targetIndices),Prim().AttributeValue<int>(_constraint->objectIndices),
            Prim().AttributeValue<GfMatrix4d>(_constraint->targetBinds),Prim().AttributeValue<double>(_constraint->targetWeights),
            Prim().AttributeValue<GfVec3d>(_constraint->pivot),Prim().AttributeValue<bool>(_constraint->dualQuaternion),Prim().AttributeValue<bool>(_constraint->currentPivot),
            Prim().Relationship(_constraint->targets).TargetedObjects<rigExec::RigExecPointFrame>(_constraint->computePointFrame).InputName(_constraint->targets),
            Prim().Relationship(_constraint->targetObjects).TargetedObjects<rigExec::RigExecPointFrame>(_constraint->computePointFrame).InputName(_constraint->targetObjects),
            Prim().AttributeValue<TfToken>(_constraint->ownerSpace),Prim().AttributeValue<TfToken>(_constraint->targetSpace),
            Prim().AttributeValue<int>(_constraint->axisMask),Prim().AttributeValue<int>(_constraint->invertMask),Prim().AttributeValue<bool>(_constraint->offset),
            Prim().AttributeValue<bool>(_constraint->uniformScale),Prim().AttributeValue<bool>(_constraint->scaleAdd),Prim().AttributeValue<double>(_constraint->power),
            Prim().AttributeValue<TfToken>(_constraint->rotationMix),Prim().AttributeValue<bool>(_constraint->removeTargetShear),
            Prim().AttributeValue<TfToken>(_constraint->mapFrom),Prim().AttributeValue<GfVec3d>(_constraint->mapFromMin),Prim().AttributeValue<GfVec3d>(_constraint->mapFromMax),
            Prim().AttributeValue<GfVec3d>(_constraint->mapToMin),Prim().AttributeValue<GfVec3d>(_constraint->mapToMax),Prim().AttributeValue<GfVec3i>(_constraint->mapAxes),
            Prim().AttributeValue<bool>(_constraint->mapExtrapolate),Prim().AttributeValue<TfToken>(_constraint->mapMix),
            Prim().AttributeValue<TfToken>(_constraint->trackAxis),Prim().AttributeValue<TfToken>(_constraint->keepAxis),Prim().AttributeValue<TfToken>(_constraint->volume),
            Prim().AttributeValue<double>(_constraint->restLength),Prim().AttributeValue<double>(_constraint->bulge),
            Prim().AttributeValue<double>(_constraint->bulgeMin),Prim().AttributeValue<double>(_constraint->bulgeMax),Prim().AttributeValue<double>(_constraint->bulgeSmooth),
            Prim().AttributeValue<bool>(_constraint->useBulgeMin),Prim().AttributeValue<bool>(_constraint->useBulgeMax),Prim().AttributeValue<GfVec3d>(_constraint->targetOffset),
            Prim().Relationship(_constraint->source).TargetedObjects<rigExec::RigExecPointFrame>(_constraint->computePointFrame).InputName(_constraint->source),
            Prim().Relationship(_constraint->sourceObject).TargetedObjects<rigExec::RigExecPointFrame>(_constraint->computePointFrame).InputName(_constraint->sourceObject),
            Prim().Relationship(_constraint->ownerObject).TargetedObjects<rigExec::RigExecPointFrame>(_constraint->computePointFrame).InputName(_constraint->ownerObject),
            Prim().Relationship(_constraint->customSpace).TargetedObjects<rigExec::RigExecPointFrame>(_constraint->computePointFrame).InputName(_constraint->customSpace),
            Prim().AttributeValue<GfMatrix4d>(_localSpaces->ownerLocal),
            Prim().AttributeValue<GfMatrix4d>(_localSpaces->ownerRest),
            Prim().AttributeValue<GfMatrix4d>(_localSpaces->ownerParentRest),
            Prim().AttributeValue<bool>(_localSpaces->ownerHasParent),
            Prim().AttributeValue<bool>(_localSpaces->ownerInheritRotation),
            Prim().AttributeValue<bool>(_localSpaces->ownerLocalLocation),
            Prim().AttributeValue<TfToken>(_localSpaces->ownerInheritScale),
            Prim().AttributeValue<GfMatrix4d>(_localSpaces->sourceLocal),
            Prim().AttributeValue<GfMatrix4d>(_localSpaces->sourceRest),
            Prim().AttributeValue<GfMatrix4d>(_localSpaces->sourceParentRest),
            Prim().AttributeValue<bool>(_localSpaces->sourceHasParent),
            Prim().AttributeValue<bool>(_localSpaces->sourceInheritRotation),
            Prim().AttributeValue<bool>(_localSpaces->sourceLocalLocation),
            Prim().AttributeValue<TfToken>(_localSpaces->sourceInheritScale),
            Prim().Relationship(_localSpaces->ownerParent).TargetedObjects<rigExec::RigExecPointFrame>(_constraint->computePointFrame).InputName(_localSpaces->ownerParent),
            Prim().Relationship(_localSpaces->sourceParent).TargetedObjects<rigExec::RigExecPointFrame>(_constraint->computePointFrame).InputName(_localSpaces->sourceParent));
}
EXEC_REGISTER_COMPUTATIONS_FOR_SCHEMA(RigExecSkinInfluence) {
    self.AttributeExpression(_skin->matrix).Callback<GfMatrix4d>(&SkinInfluence)
        .Inputs(Prim().AttributeValue<GfMatrix4d>(_skin->inverseBind),Prim().AttributeValue<GfMatrix4d>(_skin->inverseMesh),
            Prim().AttributeValue<bool>(_skin->fromBind),Prim().AttributeValue<bool>(_skin->followOnly),
            Prim().Relationship(_skin->owner).TargetedObjects<rigExec::RigExecPointFrame>(_skin->computePointFrame).InputName(_skin->owner),
            Prim().Relationship(_skin->source).TargetedObjects<rigExec::RigExecPointFrame>(_skin->computePointFrame).InputName(_skin->source),
            Prim().Relationship(_skin->sourceObject).TargetedObjects<rigExec::RigExecPointFrame>(_skin->computePointFrame).InputName(_skin->sourceObject));
}
EXEC_REGISTER_COMPUTATIONS_FOR_SCHEMA(RigExecArmatureParent) {
    self.AttributeExpression(_armature->matrix).Callback<GfMatrix4d>(&ArmatureParent)
        .Inputs(Prim().AttributeValue<GfMatrix4d>(_armature->local),
            Prim().AttributeValue<GfMatrix4d>(_armature->inverseBind),
            Prim().AttributeValue<GfMatrix4d>(_armature->incoming),Prim().AttributeValue<bool>(_armature->useIncoming),
            Prim().AttributeValue<bool>(_armature->preserveLocation),
            Prim().AttributeValue<double>(_armature->tx),Prim().AttributeValue<double>(_armature->ty),Prim().AttributeValue<double>(_armature->tz),
            Prim().AttributeValue<double>(_armature->rx),Prim().AttributeValue<double>(_armature->ry),Prim().AttributeValue<double>(_armature->rz),
            Prim().AttributeValue<double>(_armature->sx),Prim().AttributeValue<double>(_armature->sy),Prim().AttributeValue<double>(_armature->sz),
            Prim().Relationship(_armature->parent).TargetedObjects<rigExec::RigExecPointFrame>(_armature->computePointFrame).InputName(_armature->parent),
            Prim().Relationship(_armature->sourceObject).TargetedObjects<rigExec::RigExecPointFrame>(_armature->computePointFrame).InputName(_armature->sourceObject),
            Prim().Relationship(_armature->source).TargetedObjects<rigExec::RigExecPointFrame>(_armature->computePointFrame).InputName(_armature->source));
}
EXEC_REGISTER_COMPUTATIONS_FOR_SCHEMA(RigExecBoneFrame) {
    self.AttributeExpression(_bone->matrix).Callback<GfMatrix4d>(&BoneFrame)
        .Inputs(Prim().AttributeValue<TfToken>(_bone->spaceKind),Prim().AttributeValue<GfMatrix4d>(_bone->local),Prim().AttributeValue<GfMatrix4d>(_bone->parentRest),
            Prim().AttributeValue<bool>(_bone->hasParent),Prim().AttributeValue<bool>(_bone->inheritRotation),
            Prim().AttributeValue<bool>(_bone->localLocation),Prim().AttributeValue<bool>(_bone->connected),Prim().AttributeValue<TfToken>(_bone->inheritScale),
            Prim().AttributeValue<double>(_bone->tx),Prim().AttributeValue<double>(_bone->ty),Prim().AttributeValue<double>(_bone->tz),
            Prim().AttributeValue<double>(_bone->rx),Prim().AttributeValue<double>(_bone->ry),Prim().AttributeValue<double>(_bone->rz),
            Prim().AttributeValue<double>(_bone->sx),Prim().AttributeValue<double>(_bone->sy),Prim().AttributeValue<double>(_bone->sz),
            Prim().Relationship(_bone->parent).TargetedObjects<rigExec::RigExecPointFrame>(_bone->computePointFrame).InputName(_bone->parent),
            Prim().Relationship(_bone->object).TargetedObjects<rigExec::RigExecPointFrame>(_bone->computePointFrame).InputName(_bone->object));
}
EXEC_REGISTER_COMPUTATIONS_FOR_SCHEMA(RigExecMappedFrame) {
    self.AttributeExpression(_mapped->matrix).Callback<GfMatrix4d>(&MappedFrame)
        .Inputs(Prim().AttributeValue<GfMatrix4d>(_mapped->targetRest),
            Prim().AttributeValue<GfMatrix4d>(_mapped->sourceRest),
            Prim().Relationship(_mapped->source).TargetedObjects<rigExec::RigExecPointFrame>(_mapped->computePointFrame).InputName(_mapped->source));
}
