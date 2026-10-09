// Independent weight reference over sampled oracle-origin values.
#ifndef RIGEXEC_WEIGHT_REFERENCE_H
#define RIGEXEC_WEIGHT_REFERENCE_H
#include "moverGraphTypes.h"
#include "pxr/base/gf/vec3f.h"
#include "pxr/base/vt/types.h"
#include "pxr/usd/usd/common.h"
#include "pxr/usd/usd/timeCode.h"
#include <functional>
#include <map>
#include <string>
#include <utility>
#include <vector>
namespace rigExec {
struct RigExecWeightReferenceAttribute {
    bool exists = false;
    bool connected = false;
    size_t timeSamples = 0;
    VtValue raw;
    VtValue resolvedFloat;
    bool resolvedFloatSite = false;
    explicit operator bool() const { return exists; }
    bool HasAuthoredConnections() const { return connected; }
    size_t GetNumTimeSamples() const { return timeSamples; }
    template<class T> bool Get(T *out, UsdTimeCode = UsdTimeCode::Default()) const {
        if (!raw.IsHolding<T>()) return false;
        *out = raw.UncheckedGet<T>(); return true;
    }
};
struct RigExecWeightReferenceRelationship {
    bool exists = false;
    SdfPathVector targets;
    RigExecReadPhase phase;
    std::string phaseError;
    explicit operator bool() const { return exists; }
    void GetTargets(SdfPathVector *out) const { *out = targets; }
};
struct RigExecWeightReferencePrim {
    bool exists = false;
    SdfPath path;
    TfToken type;
    std::map<TfToken, RigExecWeightReferenceAttribute> attributes;
    std::map<TfToken, RigExecWeightReferenceRelationship> relationships;
    RigExecReadPhase weightTargetPhase;
    std::string phaseError;
    std::vector<float> falloff;
    explicit operator bool() const { return exists; }
    const SdfPath &GetPath() const { return path; }
    const TfToken &GetTypeName() const { return type; }
    RigExecWeightReferenceAttribute GetAttribute(const TfToken &name) const {
        const auto it = attributes.find(name);
        return it == attributes.end() ? RigExecWeightReferenceAttribute() : it->second;
    }
    RigExecWeightReferenceRelationship GetRelationship(const TfToken &name) const {
        const auto it = relationships.find(name);
        return it == relationships.end() ? RigExecWeightReferenceRelationship() : it->second;
    }
};
struct RigExecWeightReferenceContext {
    std::map<SdfPath, RigExecWeightReferencePrim> prims;
    /// Successful raw/upstream reads only; an empty array is a successful read.
    std::map<SdfPath, VtVec3fArray> points;
    std::map<SdfPath, SdfPath> canonicalPoints;
    std::map<SdfPath, GfMatrix4d> placements;
    /// Per relationship exact phased answer; an empty box means declared but unavailable.
    std::map<std::pair<SdfPath,TfToken>,VtValue> phasedPoints;
    RigExecWeightReferencePrim GetPrimAtPath(const SdfPath &path) const {
        const auto it = prims.find(path);
        return it == prims.end() ? RigExecWeightReferencePrim() : it->second;
    }
};
/// Source sampling boundary; scalar reads use the original resolved-input walk.
/// No WeightField output, production packet or interactive setter is an input.
RigExecWeightReferenceContext RigExecCaptureWeightReference(
    const UsdStageRefPtr &stage, const SdfPath &root,
    const RigExecResolvedInputs &resolved, const std::map<SdfPath,VtValue> &upstream,
    UsdTimeCode time, const std::function<const GfMatrix4d *(const SdfPath &)> &placement);
bool RigExecResolveWeightReference(
    const RigExecWeightReferenceContext &context, const SdfPath &path, size_t count,
    std::vector<float> *weights, std::string *error,
    const std::vector<GfVec3f> *currentPoints = nullptr);
} // namespace rigExec
#endif
