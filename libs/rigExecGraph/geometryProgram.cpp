#include "geometryProgram.h"
#include "blendLayout.h"
#include <cmath>
namespace rigExec {
RigExecMoverParameters RigExecAssembleGeometry(const RigExecGeometryRecord &record,
    const RigExecGeometryInputs &inputs)
{
    RigExecRevisionLeafView leaves;
    leaves.decl=&record.leaves; leaves.values=inputs.leaves; leaves.phased=inputs.phased;
    RigExecProviderValues providers;
    providers.transform=inputs.transform; providers.carry=inputs.carry;
    providers.influenceTransforms=inputs.influenceTransforms;
    providers.weights=inputs.weights; providers.driverFrames=inputs.driverFrames;
    providers.skinTopology=inputs.skinTopology;
    providers.basePoints=inputs.basePoints; providers.blendDeltas=inputs.blendDeltas;
    return RigExecAssembleFromLeaves(record.op,record.binding,leaves,providers);
}
RigExecMoverParameters RigExecAssembleGeometry(RigExecRevisionOp op,
    const RigExecRevisionBinding &binding,const RigExecRevisionLeafView &leaves,
    const RigExecProviderValues &values)
{
    return RigExecAssembleFromLeaves(op,binding,leaves,values);
}
GfMatrix4d RigExecGeometryMatrixInPointFrame(const GfMatrix4d &transform,
    const GfMatrix4d *space,const GfMatrix4d *reference,
    const GfMatrix4d *referenceSpace,bool posedPoints,const GfMatrix4d *carry)
{
    GfMatrix4d result=reference?RigExecMeasureFromReference(transform,*reference):transform;
    if(space) {
        const auto measured=referenceSpace?RigExecMeasureFromReference(*space,*referenceSpace):*space;
        result=RigExecClusterInPointFrame(RigExecMeasureInSpace(result,measured),measured,posedPoints,carry);
    }
    return result;
}
bool RigExecBuildGeometryBlendLayout(const VtVec3fArray &offsets,
    const VtIntArray &indices,size_t pointCount,RigExecBlendSampleLayout *layout)
{
    return RigExecBuildBlendLayout(offsets,indices,pointCount,layout);
}
bool RigExecGeometryBlendDeltas(const std::vector<RigExecBlendChannel> &channels,
    const std::vector<GfVec3f> &base,std::vector<GfVec3f> *deltas)
{
    return RigExecSumBlendChannels(channels,base,deltas);
}
bool RigExecGeometryBlendDeltas(const std::vector<RigExecBlendChannel> &channels,
    const GfVec3f *base,size_t count,std::vector<GfVec3f> *deltas)
{
    return RigExecSumBlendChannels(channels,base,count,deltas);
}
bool RigExecRunGeometryDerived(RigExecRevisionOp op,
    const RigExecMoverParameters &parameters,const GfVec3f *authored,
    size_t authoredCount,std::vector<GfVec3f> *result)
{
    return RigExecApplyDerivedKernel(op,parameters,authored,authoredCount,result);
}
bool RigExecRunGeometry(RigExecRevisionOp op,const RigExecMoverParameters &parameters,
    std::vector<GfVec3f> *points,bool useSimd,RigExecWireBasisCache *wire,
    RigExecSurfaceKernelCache<GfVec3f,GfVec3d> *surface)
{
    return points && RigExecRunRevisionKernel(op,parameters,points,useSimd,wire,surface);
}
namespace geometryDetail {
bool RunDiscardableGeometry(RigExecRevisionOp op,const RigExecMoverParameters &parameters,
    std::vector<GfVec3f> *points,bool useSimd,RigExecWireBasisCache *wire,
    RigExecSurfaceKernelCache<GfVec3f,GfVec3d> *surface)
{
    return points && RunDiscardableRevisionKernel(op,parameters,points,useSimd,wire,surface);
}
}
bool RigExecRunGeometryMatrix(RigExecRevisionOp op,const RigExecRevisionBinding &binding,
    const RigExecSurfaceProjectorFrames &frames,const RigExecProjectorReads &reads,
    const std::vector<GfVec3f> &base,const std::vector<GfVec3f> &final,
    GfMatrix4d *matrix,std::vector<std::string> *diagnostics,
    RigExecSurfaceKernelCache<GfVec3f,GfVec3d> *cache)
{
    return matrix && RigExecRunProjectorTarget(op,binding,frames,reads,base,final,
                                              matrix,diagnostics,cache);
}
bool RigExecRunGeometryMatrix(const RigExecGeometryRecord &record,
    const RigExecSurfaceProjectorFrames &frames,const RigExecProjectorReads &reads,
    const std::vector<GfVec3f> &base,const std::vector<GfVec3f> &final,
    RigExecGeometryWorkspace *workspace,GfMatrix4d *matrix,
    std::vector<std::string> *diagnostics)
{
    return workspace && matrix && RigExecRunProjectorTarget(record.op,record.binding,
        frames,reads,base,final,matrix,diagnostics,&workspace->surface);
}
bool RigExecRunGeometry(const RigExecGeometryRecord &record,
    const RigExecMoverParameters &parameters,RigExecGeometryWorkspace *workspace,
    std::vector<GfVec3f> *points,bool useSimd)
{
    if(!workspace) return false;
    return RigExecRunGeometry(record.op,parameters,points,useSimd,&workspace->wire,
                              &workspace->surface);
}
}
