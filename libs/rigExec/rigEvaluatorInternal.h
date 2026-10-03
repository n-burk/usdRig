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

// The attributes the structure digest hashes as bindings -- connection walk
// and declared read phase -- on every mover, on every weight object a mover
// binds, and on every blend input a mover names. With every connected
// attribute of an aggregate solver, these are where a read phase on a
// connection is honoured (RigExecPhasedConnection), and the digest hashing
// them is what makes a phase edit re-epoch the rig.
inline constexpr const char *kDigestMoverInputs[] = {
    "inputs:defaultWeight", "inputs:enabled", "inputs:value", "inputs:min",
    "inputs:max", "inputs:keys", "inputs:tangents"};
inline constexpr const char *kDigestWeightObjectFields[] = {
    "rigExec:values", "rigExec:indices", "rigExec:defaultWeight",
    "rigExec:representation", "rigExec:rangePolicy", "rigExec:operation",
    "inputs:driver", "inputs:scale", "inputs:bias", "inputs:falloffMin",
    "inputs:falloffMax", "inputs:invert", "inputs:strength", "inputs:scaleX",
    "inputs:scaleXPos", "inputs:scaleYPos", "inputs:scaleZPos",
    "inputs:scaleXNeg", "inputs:scaleYNeg", "inputs:scaleZNeg",
    "inputs:scaleY", "inputs:scaleZ", "inputs:extentU", "inputs:extentV",
    "inputs:weights", "rigExec:autoSmooth", "rigExec:basis",
    "rigExec:samplesPerSpline", "rigExec:unreachedValue"};
inline constexpr const char *kDigestBlendInputFields[] = {"inputs:weight"};

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
inline const TfToken _weightTargetRel("rigExec:weightTarget");

/// Whether a volume weight measures its distance against the points as
/// they stand at the consuming operator's position (`preceding` declared
/// on rigExec:weightTarget) rather than against its static source (`base`,
/// the default). Any other phase, or an unparseable one, is false with
/// \p error filled.
bool _VolumeWeightSamplesInFlight(const UsdPrim &weight, bool *inFlight,
                                  std::string *error);

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
