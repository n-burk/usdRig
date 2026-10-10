// Scalar geometry oracle with explicit publication and sampling adapters.
#ifndef RIGEXEC_SCALAR_REFERENCE_H
#define RIGEXEC_SCALAR_REFERENCE_H
#include "rigEvaluator.h"
#include "movers/moverRegistry.h"
namespace rigExec {
struct RigExecScalarReferenceContext {
    const RigExecOracleScene &stage;
    const RigExecOracleScene &resolved;
    const std::map<SdfPath,VtValue> &upstream;
    std::function<const VtValue *(const SdfPath &,const RigExecReadPhase &,const SdfPath &)> phasedPoints;
    std::function<const GfMatrix4d *(const SdfPath &,const RigExecReadPhase &,const SdfPath &)> phasedMatrix;
    std::function<const VtValue *(const SdfPath &,const SdfPath &,const SdfPath &)> sampleSnapshot;
    std::function<bool(const SdfPath &,const SdfPath &,size_t,std::vector<float> *,std::string *,const std::vector<GfVec3f> *)> weights;
    std::function<void(size_t,const SdfPath &,const VtVec3fArray &)> revisionObserver = {};
    std::function<const RigExecOracleFrameInput *(const SdfPath &,const SdfPath &,
                                                 UsdTimeCode)> boundFrames = {};
    // Declared consumer route; independent field resolution supplies the failure.
    std::function<bool(const SdfPath &)> currentPhaseWeight = {};
};
VtVec3fArray RigExecScalarChainReference(
    const RigExecScalarReferenceContext &context, const SdfPath &target,
    const std::vector<const RigExecMoverRecord *> &chain,
    const std::unordered_map<SdfPath,GfMatrix4d,SdfPath::Hash> &baseProviderMatrices,
    const std::unordered_map<SdfPath,GfMatrix4d,SdfPath::Hash> &finalProviderMatrices,
    UsdTimeCode time,std::vector<std::string> *diagnostics,
    const std::unordered_map<SdfPath,GfMatrix4d,SdfPath::Hash> &geometryConstraintDeltas);
} // namespace rigExec
#endif
