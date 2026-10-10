// Private evaluator implementation types and helpers.
#ifndef RIGEXEC_RIG_EVALUATOR_INTERNAL_H
#define RIGEXEC_RIG_EVALUATOR_INTERNAL_H

#include "rigEvaluator.h"
#include "pxr/usd/usd/attribute.h"
#include "pxr/base/tf/stringUtils.h"
#include "pxr/usd/sdf/layer.h"
#include "pxr/usd/sdf/primSpec.h"
#include "pxr/usd/sdf/schema.h"
#include "pxr/usd/usd/primRange.h"
#include "pxr/usd/usd/relationship.h"
#include "pxr/usd/usd/stage.h"
#include <algorithm>
#include <atomic>
#include <unordered_set>

namespace rigExec {

namespace evaluatorDetail {

SdfPathVector
_AuthoredConnections(const UsdAttribute &attribute);

// The attributes the movers segment of the structure digest hashes as
// bindings -- connection walk and declared read phase -- on every mover, on
// every weight object a mover binds, and on every blend input a mover
// names. The read phase of a connection is resolved on every connected
// operator input instead (_ForEachConnectedInput).
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

// The relationships through which an operator names a prim whose inputs
// it reads: a weight object (and, transitively, the weights it composes)
// and a blend input.
inline const TfToken _weightObjectRel("rigExec:weightObject");
inline const TfToken _inputWeightsRel("rigExec:inputWeights");
inline const TfToken _baseWeightRel("rigExec:baseWeight");
inline const TfToken _blendInputsRel("rigExec:blendInputs");
// The relationships through which an operator names one attribute it
// reads: a pose interpolator's numeric driver, a surface projector's
// shader dials, a space switch's active-space channel.
inline const TfToken _driverAttributesRel("rigExec:driverAttributes");
inline const TfToken _shaderDialSourcesRel("rigExec:shaderDialSources");
inline const TfToken _activeSpaceAttributeRel("rigExec:activeSpaceAttribute");

// Every connected operator input: each attribute with authored connections
// on a prim under \p rig (UsdPrimRange order, the rig prim first), then on
// each prim outside it that an operator names as a weight object or blend
// input, followed transitively in the order first named, then each
// attribute outside it that an operator names as one it reads, in the
// order first named. These are the inputs a rigExecReadPhase is resolved
// on (RigExecPhasedConnection), and the structure digest hashes each one's
// walk and authored phase, so compile and digest consider one set by
// construction.
// \p visit(attribute) per input; \p reached(primPath) per prim outside the
// rig that the relationships lead to, whether or not it exists.
template <class Visit, class Reached>
void
_ForEachConnectedInput(const UsdPrim &rig, const Visit &visit,
                       const Reached &reached)
{
    if (!rig) {
        return;
    }
    const SdfPath &root = rig.GetPath();
    std::vector<SdfPath> outside;
    std::unordered_set<SdfPath, SdfPath::Hash> named;
    std::vector<SdfPath> outsideAttributes;
    std::unordered_set<SdfPath, SdfPath::Hash> namedAttributes;
    SdfPathVector targets;
    TfTokenVector children;
    TfTokenVector connected;
    TfTokenVector relationships;
    // Name order, as UsdPrim::GetAuthoredProperties gives it.
    const auto byName = [](const TfToken &a, const TfToken &b) {
        return TfDictionaryLessThan()(a.GetString(), b.GetString());
    };
    const auto visitPrim = [&](const UsdPrim &prim) {
        // A property composes connections or targets only where a spec in
        // the prim's stack authors them, so the specs say which properties
        // to compose: a field lookup per authored property spec, instead
        // of composing every authored property of every prim.
        connected.clear();
        relationships.clear();
        for (const SdfPrimSpecHandle &spec : prim.GetPrimStack()) {
            const SdfLayerHandle layer = spec->GetLayer();
            const SdfPath &specPath = spec->GetPath();
            children.clear();
            if (!layer->HasField(specPath, SdfChildrenKeys->PropertyChildren,
                                 &children)) {
                continue;
            }
            for (const TfToken &name : children) {
                const bool relationship =
                    name == _weightObjectRel || name == _inputWeightsRel ||
                    name == _baseWeightRel || name == _blendInputsRel ||
                    name == _driverAttributesRel ||
                    name == _shaderDialSourcesRel ||
                    name == _activeSpaceAttributeRel;
                if (layer->HasField(specPath.AppendProperty(name),
                                    relationship
                                        ? SdfFieldKeys->TargetPaths
                                        : SdfFieldKeys->ConnectionPaths)) {
                    (relationship ? relationships : connected)
                        .push_back(name);
                }
            }
        }
        for (TfTokenVector *names : {&connected, &relationships}) {
            std::sort(names->begin(), names->end(), byName);
            names->erase(std::unique(names->begin(), names->end()),
                         names->end());
        }
        for (const TfToken &name : connected) {
            const UsdAttribute attribute = prim.GetAttribute(name);
            if (attribute && attribute.HasAuthoredConnections()) {
                visit(attribute);
            }
        }
        for (const TfToken &name : relationships) {
            targets.clear();
            if (const UsdRelationship rel = prim.GetRelationship(name)) {
                rel.GetTargets(&targets);
            }
            const bool namesAttributes = name == _driverAttributesRel ||
                                         name == _shaderDialSourcesRel ||
                                         name == _activeSpaceAttributeRel;
            for (const SdfPath &target : targets) {
                const SdfPath primPath = target.GetPrimPath();
                if (primPath.HasPrefix(root)) {
                    continue;
                }
                if (!namesAttributes) {
                    if (named.insert(primPath).second) {
                        reached(primPath);
                        outside.push_back(primPath);
                    }
                } else if (target.IsPropertyPath() &&
                           namedAttributes.insert(target).second) {
                    reached(primPath);
                    outsideAttributes.push_back(target);
                }
            }
        }
    };
    for (const UsdPrim &prim : UsdPrimRange(rig)) {
        visitPrim(prim);
    }
    const UsdStagePtr stage = rig.GetStage();
    for (size_t i = 0; i < outside.size(); ++i) {
        if (const UsdPrim prim = stage->GetPrimAtPath(outside[i])) {
            visitPrim(prim);
        }
    }
    // A named attribute on a prim visited whole above was visited with it.
    for (const SdfPath &path : outsideAttributes) {
        if (named.count(path.GetPrimPath())) {
            continue;
        }
        const UsdAttribute attribute = stage->GetAttributeAtPath(path);
        if (attribute && attribute.HasAuthoredConnections()) {
            visit(attribute);
        }
    }
}

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
// read every frame is a file-scope constant rather than spelled per call.
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

// Every authored input computeRestFrame reads: exactly the seven
// AttributeValue inputs of the computation (computations.cpp,
// RIGEXEC_REGISTER_XFORMABLE) -- rest:space and the six rest avars. Its
// eighth input is the NamespaceAncestor's own computeRestFrame, which reads
// these same seven on the ancestor, so the closure over a provider and its
// RigExec ancestors is the closure over this list. Nothing else can move a
// rest frame, which is what makes the epoch-constancy test and the override
// test exact rather than approximate. Built once per evaluator, in its
// constructor (RigExecRigEvaluator::_restInputNames), because the
// constancy test runs on a parallel compile and a function-local static or
// a token built from text there takes a lock.
std::vector<TfToken>
_MakeRestInputNames();

bool
_IsRestInputName(const std::vector<TfToken> &names, const TfToken &name);

bool
_ProviderRestMightVary(const UsdStageRefPtr &stage,
                       const std::vector<TfToken> &restInputNames,
                       const SdfPath &provider,
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
    RigExecResolvedInputs *resolved);

} // namespace evaluatorDetail

} // namespace rigExec

#endif
