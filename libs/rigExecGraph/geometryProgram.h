#ifndef RIGEXEC_GRAPH_GEOMETRY_PROGRAM_H
#define RIGEXEC_GRAPH_GEOMETRY_PROGRAM_H
#include "rigExec/moverGraph.h"
namespace rigExec {
/// Detached revision facts. Source/query handles remain in compiler adapters.
struct RigExecGeometryRecord {
    RigExecRevisionOp op = RigExecRevisionOp::Matrix;
    RigExecRevisionBinding binding;
    RigExecRevisionLeafDecl leaves;
};
/// Borrowed typed producer values survive through the body only.
struct RigExecGeometryInputs {
    const std::vector<VtValue> *leaves = nullptr;
    const RigExecResolvedInputs *phased = nullptr;
    const GfMatrix4d *transform = nullptr, *carry = nullptr;
    const std::vector<GfMatrix4d> *influenceTransforms = nullptr;
    const RigExecWeightPacket *weights = nullptr;
    const RigExecPointFrameArray *driverFrames = nullptr;
    const std::shared_ptr<const RigExecSkinTopology> *skinTopology = nullptr;
    std::vector<GfVec3f> basePoints, blendDeltas;
};
struct RigExecGeometryWorkspace {
    RigExecWireBasisCache wire;
    RigExecSurfaceKernelCache<GfVec3f,GfVec3d> surface;
};
/// Same pure packet assembler used by native and detached scene consumers.
RigExecMoverParameters RigExecAssembleGeometry(const RigExecGeometryRecord &,
    const RigExecGeometryInputs &);
RigExecMoverParameters RigExecAssembleGeometry(RigExecRevisionOp,
    const RigExecRevisionBinding &,const RigExecRevisionLeafView &,
    const RigExecProviderValues &);
/// Fold measured neutral references and point-frame carry in production order.
GfMatrix4d RigExecGeometryMatrixInPointFrame(const GfMatrix4d &,
    const GfMatrix4d *space,const GfMatrix4d *reference,
    const GfMatrix4d *referenceSpace,bool posedPoints,const GfMatrix4d *carry);
bool RigExecBuildGeometryBlendLayout(const VtVec3fArray &,const VtIntArray &,
    size_t pointCount,RigExecBlendSampleLayout *);
/// Existing ordered channel interpolation and summation; no alternate arithmetic.
bool RigExecGeometryBlendDeltas(const std::vector<RigExecBlendChannel> &,
    const std::vector<GfVec3f> &base,std::vector<GfVec3f> *deltas);
bool RigExecGeometryBlendDeltas(const std::vector<RigExecBlendChannel> &,
    const GfVec3f *base,size_t count,std::vector<GfVec3f> *deltas);
/// Exact existing operation and apply-once arithmetic, including derived arrays.
/// The caller owns status guards and preserves preceding values on failure.
bool RigExecRunGeometry(RigExecRevisionOp,const RigExecMoverParameters &,
    std::vector<GfVec3f> *,bool useSimd,RigExecWireBasisCache *,
    RigExecSurfaceKernelCache<GfVec3f,GfVec3d> *);
namespace geometryDetail {
/// Owned staging only; false leaves disposable output unspecified.
bool RunDiscardableGeometry(RigExecRevisionOp,const RigExecMoverParameters &,
    std::vector<GfVec3f> *,bool,RigExecWireBasisCache *,
    RigExecSurfaceKernelCache<GfVec3f,GfVec3d> *);
}
bool RigExecRunGeometryDerived(RigExecRevisionOp,const RigExecMoverParameters &,
    const GfVec3f *authored,size_t authoredCount,std::vector<GfVec3f> *result);
bool RigExecRunGeometryMatrix(RigExecRevisionOp,const RigExecRevisionBinding &,
    const RigExecSurfaceProjectorFrames &,const RigExecProjectorReads &,
    const std::vector<GfVec3f> &base,const std::vector<GfVec3f> &final,
    GfMatrix4d *,std::vector<std::string> *diagnostics,
    RigExecSurfaceKernelCache<GfVec3f,GfVec3d> *);
bool RigExecRunGeometryMatrix(const RigExecGeometryRecord &,
    const RigExecSurfaceProjectorFrames &,const RigExecProjectorReads &,
    const std::vector<GfVec3f> &base,const std::vector<GfVec3f> &final,
    RigExecGeometryWorkspace *,GfMatrix4d *,std::vector<std::string> *diagnostics);
bool RigExecRunGeometry(const RigExecGeometryRecord &,const RigExecMoverParameters &,
    RigExecGeometryWorkspace *,std::vector<GfVec3f> *,bool useSimd);
}
#endif
