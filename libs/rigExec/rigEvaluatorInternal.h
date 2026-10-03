// Private evaluator implementation types and helpers.
#ifndef RIGEXEC_RIG_EVALUATOR_INTERNAL_H
#define RIGEXEC_RIG_EVALUATOR_INTERNAL_H

#include "rigEvaluator.h"
#include "pxr/usd/usd/attribute.h"
#include <atomic>

namespace rigExec {

namespace evaluatorDetail {

SdfPathVector
_AuthoredConnections(const UsdAttribute &attribute);

// Prefer the generation's resolved value, including property and interactive overrides.
template <class T>
T
_ResolvedRead(const RigExecResolvedInputs &resolved, const UsdPrim &prim,
              const char *name, T fallback, UsdTimeCode time)
{
    T value = fallback;
    if (!prim) {
        return value;
    }
    const TfToken token(name);
    if (const UsdAttribute a = prim.GetAttribute(token)) {
        resolved.GetAttribute(a, time, &value);
    }
    return value;
}

// The same read through an already-interned name, for the per-frame call
// sites that hold one. Interning takes the token registry lock, so a name
// read every frame is spelled once as a static rather than per call.
template <class T>
T
_ResolvedRead(const RigExecResolvedInputs &resolved, const UsdPrim &prim,
              const TfToken &name, T fallback, UsdTimeCode time)
{
    T value = fallback;
    if (!prim) {
        return value;
    }
    if (const UsdAttribute a = prim.GetAttribute(name)) {
        resolved.GetAttribute(a, time, &value);
    }
    return value;
}

// Read epoch structure at Default; generation overrides do not apply.
template <class T>
T
_ReadAttribute(const UsdPrim &prim, const char *name, T fallback)
{
    T value = fallback;
    if (!prim) {
        return value;
    }
    if (const UsdAttribute a = prim.GetAttribute(TfToken(name))) {
        a.Get(&value);
    }
    return value;
}

// The same epoch read through an already-interned name.
template <class T>
T
_ReadAttribute(const UsdPrim &prim, const TfToken &name, T fallback)
{
    T value = fallback;
    if (!prim) {
        return value;
    }
    if (const UsdAttribute a = prim.GetAttribute(name)) {
        a.Get(&value);
    }
    return value;
}

inline const TfToken _computePointFrame("computePointFrame");
inline const TfToken _computePointFrameArray("computePointFrameArray");

// The joint rest a solver measures from. Overridden per solver batch when an
// earlier pose step wrote the joint (spec §4.2, "the incoming frame is the
// solver's rest reference").
inline const TfToken _computeRestFrame("computeRestFrame");
inline const TfToken _movesRel("rigExec:moves");
inline const TfToken _enabledAttr("inputs:enabled");
inline const TfToken _computeFalloffLut("computeFalloffLut");
inline const TfToken _falloffProfileAttr("rigExec:falloffProfile");
inline const TfToken _falloffCurveAttr("rigExec:falloffCurve");
inline const TfToken _samplePhaseAttr("rigExec:samplePhase");

std::vector<UsdPrim>
_GetPoseStackOrder(const UsdPrim &root);

std::vector<UsdPrim>
_GetMoverExecutionOrder(const UsdPrim &rig);

bool
_IsVolumeWeightType(const TfToken &typeName);

bool
_IsFrameProviderType(const TfToken &type);

bool
_IsFrameProvider(const UsdPrim &prim);

bool
_IsWeightObjectType(const TfToken &typeName);

std::vector<float>
_BakeFalloffLut(const UsdPrim &prim);

SdfPath
_ResolveGeometryInput(const UsdStageRefPtr &stage, const SdfPath &target);

bool
_ValidateScalarConnection(
    const UsdStageRefPtr &stage, const UsdAttribute &attribute,
    const SdfValueTypeName &expectedType, std::set<SdfPath> *visiting,
    std::string *error);

bool
_ValidateWeightObjectDomain(
    const UsdStageRefPtr &stage, const SdfPath &weightPath,
    const SdfPath &moverTarget, bool pointDomain, bool operationDomain,
    size_t logicalCount, std::set<SdfPath> *visiting, std::string *error);

bool
_IsRestInputName(const TfToken &name);

bool
_ProviderRestMightVary(const UsdStageRefPtr &stage, const SdfPath &provider,
                       const std::set<SdfPath> &chainTargets);

// Counters used by RIGEXEC_VERIFY_CERTAIN_STRUCTURAL.
struct _CertainStructuralCounts {
    std::atomic<size_t> tagged{0};
    std::atomic<size_t> falsePositives{0};
    std::atomic<size_t> digestFound{0};
};

_CertainStructuralCounts *
_CertainStructuralVerifyCounts();

bool
_DigestSplitVerifyRequested();

void
_ApplyInteractiveOverrides(
    const std::vector<RigExecValueOverride> &interactive,
    std::vector<RigExecValueOverride> *overrides,
    RigExecResolvedInputs *resolved,
    std::map<SdfPath, VtValue> *publishedProperties = nullptr);

} // namespace evaluatorDetail

} // namespace rigExec

#endif
