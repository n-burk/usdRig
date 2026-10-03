// Geometry chains, weight fields, and skin-layout inputs.

#include "rigEvaluatorInternal.h"
#include "frameExtraction.h"
#include "rigExecMath/geometryKernels.h"
#include "rigEvaluatorConstraints.h"
#include "movers/moverRegistry.h"
#include "rigExecMath/envelope.h"
#include "rigExecMath/weightFields.h"

#include "pxr/base/ts/spline.h"
#include "pxr/usd/usd/attribute.h"
#include "pxr/usd/usd/prim.h"
#include "pxr/usd/usd/relationship.h"
#include "pxr/usd/usdGeom/pointBased.h"

#include <algorithm>
#include <cmath>
#include <set>
#include <unordered_map>

namespace rigExec {

namespace evaluatorDetail {

bool
_VolumeWeightSamplesInFlight(const UsdPrim &weight, bool *inFlight,
                             std::string *error)
{
    *inFlight = false;
    RigExecReadPhase phase;
    std::string why;
    if (!RigExecResolveReadPhase(weight.GetRelationship(_weightTargetRel),
                                 &phase, &why)) {
        *error = why;
        return false;
    }
    if (phase.kind == RigExecReadPhaseKind::Preceding) {
        *inFlight = true;
    } else if (!phase.IsBase()) {
        *error = weight.GetPath().GetString() + ": rigExecReadPhase '" +
                 phase.GetAsString() +
                 "' on rigExec:weightTarget is not supported; a volume "
                 "weight measures its source at base or the points in flight "
                 "at preceding";
        return false;
    }
    return true;
}

}  // namespace evaluatorDetail

using namespace evaluatorDetail;

namespace {

bool
_GetLandmarks(
    const UsdPrim &prim, const TfToken &attrName, UsdTimeCode time,
    std::array<GfVec3d, 4> *out)
{
    VtVec3dArray points;
    const UsdAttribute attr = prim.GetAttribute(attrName);
    if (!attr || !attr.Get(&points, time) || points.size() != 4) {
        return false;
    }
    std::copy(points.begin(), points.end(), out->begin());
    return true;
}

bool
_IsEnabled(const UsdPrim &mover, UsdTimeCode time)
{
    bool enabled = true;
    if (UsdAttribute a = mover.GetAttribute(_enabledAttr)) {
        a.Get(&enabled, time);
    }
    return enabled;
}

} // namespace

namespace evaluatorDetail {

/// Bakes a volumetric weight's distance-to-weight remap into the lookup
/// table its exec kernel consumes.
///
/// The named profiles bake analytically; `curve` resamples the Ts spline
/// authored on rigExec:falloffCurve. Both land in the same table, so the
/// kernel has one remap path and an author switching between a preset and
/// a hand-drawn curve changes only the numbers.
std::vector<float>
_BakeFalloffLut(const UsdPrim &prim)
{
    TfToken profile("smooth");
    if (UsdAttribute a = prim.GetAttribute(_falloffProfileAttr)) {
        a.Get(&profile);
    }
    if (profile == "linear") {
        return RigExecBuildFalloffLut(RigExecFalloffProfile::Linear);
    }
    if (profile == "smooth") {
        return RigExecBuildFalloffLut(RigExecFalloffProfile::Smooth);
    }
    if (profile == "easeIn") {
        return RigExecBuildFalloffLut(RigExecFalloffProfile::EaseIn);
    }
    if (profile == "easeOut") {
        return RigExecBuildFalloffLut(RigExecFalloffProfile::EaseOut);
    }
    if (profile == "constant") {
        return RigExecBuildFalloffLut(RigExecFalloffProfile::Constant);
    }
    if (profile != "curve") {
        return {};  // unknown token: linear, never coerced to a preset
    }

    const UsdAttribute curve = prim.GetAttribute(_falloffCurveAttr);
    if (!curve || !curve.HasSpline()) {
        // `curve` with nothing drawn is linear, not empty: the profile
        // token is a promise about SHAPE, and an author who selects it
        // before touching the editor should see the identity ramp.
        return RigExecBuildFalloffLut(RigExecFalloffProfile::Linear);
    }
    const TsSpline spline = curve.GetSpline();
    std::vector<float> lut(RigExecFalloffLutSize);
    for (size_t i = 0; i < RigExecFalloffLutSize; ++i) {
        const double x = double(i) / double(RigExecFalloffLutSize - 1);
        float value = 0.0f;
        // Ts extrapolates HELD outside the authored knot range, so a
        // curve drawn over a shorter span still yields a total field.
        if (!spline.Eval(x, &value) || !std::isfinite(value)) {
            value = float(x);
        }
        lut[i] = value;
    }
    return lut;
}

// Resolves a READ-side geometry input: naming a PointBased prim means its
// .points property, because a geometry input has exactly one thing to read.
// Property paths stay exact (spec §4.2).
// This rule is deliberately NOT applied to write targets. On the write side a
// bare prim path names the transform domain and <prim>.points names the
// geometry domain -- two different write sets on the same prim -- so inferring
// between them is what made a constraint unable to target a Mesh at all.
SdfPath
_ResolveGeometryInput(const UsdStageRefPtr &stage, const SdfPath &target)
{
    if (target.IsPrimPath()) {
        const UsdPrim prim = stage->GetPrimAtPath(target);
        if (prim && prim.IsA<UsdGeomPointBased>()) {
            return target.AppendProperty(TfToken("points"));
        }
    }
    return target;
}

} // namespace evaluatorDetail

bool
RigExecRigEvaluator::_ReadTargetPoints(
    const UsdPrim &prim, const char *relationshipName, UsdTimeCode time,
    std::vector<GfVec3f> *points) const
{
    points->clear();
    SdfPathVector targets;
    if (UsdRelationship rel = prim.GetRelationship(TfToken(relationshipName))) {
        rel.GetTargets(&targets);
    }
    if (targets.size() != 1) {
        return false;
    }
    const SdfPath canonical = _ResolveGeometryInput(_stage, targets[0]);
    const UsdAttribute attr = _stage->GetAttributeAtPath(canonical);
    VtVec3fArray value;
    if (!attr || !attr.Get(&value, time)) {
        return false;
    }
    points->assign(value.begin(), value.end());
    return true;
}

bool
RigExecRigEvaluator::_ResolveVolumeWeights(
    const UsdPrim &prim, size_t count, UsdTimeCode time,
    std::vector<float> *weights, std::string *error,
    const std::vector<GfVec3f> *currentPoints) const
{
    // Spelled only when an error needs it: this runs every time a volume
    // weight is resolved, and a resolve that succeeds reports nothing.
    const auto who = [&prim]() { return prim.GetPath().GetAsString(); };
    const TfToken typeName = prim.GetTypeName();

    // The composed field folds its inputs; it measures nothing itself.
    if (typeName == "RigExecCombineWeight") {
        SdfPathVector inputs;
        if (UsdRelationship rel =
                prim.GetRelationship(TfToken("rigExec:inputWeights"))) {
            rel.GetTargets(&inputs);
        }
        TfToken modeName("multiply");
        if (UsdAttribute a =
                prim.GetAttribute(TfToken("rigExec:combineMode"))) {
            a.Get(&modeName, time);
        }
        RigExecWeightCombine mode;
        if (modeName == "multiply") {
            mode = RigExecWeightCombine::Multiply;
        } else if (modeName == "add") {
            mode = RigExecWeightCombine::Add;
        } else if (modeName == "subtract") {
            mode = RigExecWeightCombine::Subtract;
        } else if (modeName == "max") {
            mode = RigExecWeightCombine::Max;
        } else if (modeName == "min") {
            mode = RigExecWeightCombine::Min;
        } else if (modeName == "average") {
            mode = RigExecWeightCombine::Average;
        } else if (modeName == "overlay") {
            mode = RigExecWeightCombine::Overlay;
        } else {
            *error = who() + ": unknown rigExec:combineMode " +
                     modeName.GetString();
            return false;
        }

        // Authored order, unsorted: subtract and overlay are order
        // dependent by design (see the schema doc).
        std::vector<std::vector<float>> fields;
        fields.reserve(inputs.size());
        for (const SdfPath &input : inputs) {
            std::vector<float> field;
            if (!_ResolveWeights(input, count, time, &field, error,
                                 currentPoints)) {
                return false;
            }
            fields.push_back(std::move(field));
        }
        if (!RigExecCombineWeightFields(mode, fields, count, weights)) {
            *error = who() + ": combine inputs disagree on element count";
            return false;
        }
        const float strength = _ResolvedRead(
            _resolvedInputs, prim, "inputs:strength", 1.0f, time);
        const float invert = _ResolvedRead(
            _resolvedInputs, prim, "inputs:invert", 0.0f, time);
        for (float &w : *weights) {
            w = (w + (1.0f - 2.0f * w) * invert) * strength;
        }
        return true;
    }

    // Placement.
    // Taken from the volume's own exec computeMatrix rather than
    // recomputed here. The oracle exists to check the WEIGHT FIELD math
    // independently, not the xformable frame chain -- that already has
    // its own parity coverage, and a second hand-rolled implementation
    // of posed:space + rest offsets + avars + rotation order is exactly
    // the drift frameExtraction.h was created to prevent.
    const auto matrixIt = _volumeWeightMatrices.find(prim.GetPath());
    if (matrixIt == _volumeWeightMatrices.end()) {
        *error = who() + ": no resolved placement for this volume weight";
        return false;
    }
    // Scale and shear are removed so the field matches the rigid guide a
    // viewer draws; inputs:scaleX/Y/Z is the sole authority on
    // anisotropy (see the RigExecVolumeWeight schema doc).
    GfMatrix4d rigid = matrixIt->second.RemoveScaleShear();
    const double det = rigid.GetDeterminant();
    if (!std::isfinite(det) || std::abs(det) < 1e-12) {
        *error = who() + ": degenerate volume placement";
        return false;
    }
    GfMatrix4d worldToLocal = rigid.GetInverse();

    // Which points the distance function measures.
    std::vector<GfVec3f> samplePoints;
    bool inFlight = false;
    std::string phaseError;
    if (!_VolumeWeightSamplesInFlight(prim, &inFlight, &phaseError)) {
        *error = who() + ": " + phaseError;
        return false;
    }
    if (inFlight) {
        if (!currentPoints) {
            *error = who() +
                     ": rigExec:weightTarget reads `preceding` but no "
                     "in-flight points were supplied";
            return false;
        }
        samplePoints = *currentPoints;
    } else {
        // An explicit sampleSource wins over the weighted domain, which
        // is how one mesh is weighted by another mesh's shape.
        if (!_ReadTargetPoints(prim, "rigExec:sampleSource", time,
                               &samplePoints) &&
            !_ReadTargetPoints(prim, "rigExec:weightTarget", time,
                               &samplePoints)) {
            *error = who() + ": could not read the points to sample";
            return false;
        }
    }
    if (samplePoints.size() != count) {
        *error = who() + ": sampled point count does not match the target";
        return false;
    }

    RigExecFalloffParams params;
    auto readFloat = [this, &prim, time](const char *name, float fallback) {
        return _ResolvedRead(
            _resolvedInputs, prim, name, fallback, time);
    };
    params.falloffMin = readFloat("inputs:falloffMin", 0.0f);
    params.falloffMax = readFloat("inputs:falloffMax", 1.0f);
    params.invert = readFloat("inputs:invert", 0.0f);
    params.strength = readFloat("inputs:strength", 1.0f);
    params.curve = _BakeFalloffLut(prim);

    if (typeName == "RigExecPlaneWeight") {
        TfToken axis("y");
        if (UsdAttribute a = prim.GetAttribute(TfToken("rigExec:planeAxis"))) {
            a.Get(&axis, time);
        }
        const int axisIndex =
            axis == "x" ? 0 : (axis == "y" ? 1 : (axis == "z" ? 2 : -1));
        if (axisIndex < 0) {
            *error = who() + ": unknown rigExec:planeAxis " + axis.GetString();
            return false;
        }
        // Bounded clips the field to the in-plane rectangle. Mirrors
        // _BuildPlaneWeightPacket exactly, including reading the extents
        // only in the bounded arm -- the two paths have to agree value
        // for value or the parity harness fires.
        TfToken boundsMode("unbounded");
        if (UsdAttribute a =
                prim.GetAttribute(TfToken("rigExec:planeBounds"))) {
            a.Get(&boundsMode, time);
        }
        RigExecPlaneBounds extent;
        const RigExecPlaneBounds *extentPtr = nullptr;
        if (boundsMode == "bounded") {
            extent.extentU = readFloat("inputs:extentU", 1.0f);
            extent.extentV = readFloat("inputs:extentV", 1.0f);
            for (const float e : {extent.extentU, extent.extentV}) {
                if (!std::isfinite(e) || e <= 0.0f) {
                    *error = who() +
                             ": inputs:extentU/V must be finite and positive "
                             "when rigExec:planeBounds is `bounded`";
                    return false;
                }
            }
            extentPtr = &extent;
        } else if (boundsMode != "unbounded") {
            *error = who() + ": unknown rigExec:planeBounds " +
                     boundsMode.GetString();
            return false;
        }
        RigExecPlaneWeightField(
            samplePoints, worldToLocal, axisIndex, params, weights, extentPtr);
        return true;
    }

    // Sphere and curve both take the per-axis divisors, folded into the
    // matrix so the hot loop stays one transform.
    const float sx = readFloat("inputs:scaleX", 1.0f);
    const float sy = readFloat("inputs:scaleY", 1.0f);
    const float sz = readFloat("inputs:scaleZ", 1.0f);
    for (float s : {sx, sy, sz}) {
        if (!std::isfinite(s) || s <= 0.0f) {
            *error = who() + ": inputs:scaleX/Y/Z must be finite and positive";
            return false;
        }
    }
    GfMatrix4d divide(1.0);
    divide.SetScale(GfVec3d(1.0 / double(sx), 1.0 / double(sy),
                            1.0 / double(sz)));
    worldToLocal = worldToLocal * divide;

    if (typeName == "RigExecSphereWeight") {
        const GfVec3f positiveScales(
            readFloat("inputs:scaleXPos", 1.0f),
            readFloat("inputs:scaleYPos", 1.0f),
            readFloat("inputs:scaleZPos", 1.0f));
        const GfVec3f negativeScales(
            readFloat("inputs:scaleXNeg", 1.0f),
            readFloat("inputs:scaleYNeg", 1.0f),
            readFloat("inputs:scaleZNeg", 1.0f));
        for (int axis = 0; axis < 3; ++axis) {
            if (!std::isfinite(positiveScales[axis]) || positiveScales[axis] <= 0 ||
                !std::isfinite(negativeScales[axis]) || negativeScales[axis] <= 0) {
                *error = who() + ": signed axis scales must be finite and positive";
                return false;
            }
        }
        RigExecSphereWeightField(samplePoints, worldToLocal, params, weights,
                                 positiveScales, negativeScales);
        return true;
    }
    if (typeName == "RigExecCurveWeight") {
        std::vector<GfVec3f> curvePoints;
        if (!_ReadTargetPoints(prim, "rigExec:curve", time, &curvePoints) ||
            curvePoints.empty()) {
            *error = who() + ": rigExec:curve must name exactly one points source";
            return false;
        }
        RigExecCurveWeightField(
            samplePoints, curvePoints, worldToLocal, params, weights);
        return true;
    }
    *error = who() + ": not a volumetric weight object";
    return false;
}

bool
RigExecRigEvaluator::_ResolveWeights(
    const SdfPath &weightPrimPath, size_t count, UsdTimeCode time,
    std::vector<float> *weights, std::string *error,
    const std::vector<GfVec3f> *currentPoints) const
{
    weights->assign(count, 1.0f);
    const UsdPrim prim = _stage->GetPrimAtPath(weightPrimPath);
    if (!prim) {
        *error = "missing weight object " + weightPrimPath.GetString();
        return false;
    }

    // A type this oracle does not understand must FAIL, never fall
    // through to the authored-table path. That path reads no
    // representation and no defaultWeight off a volumetric prim and so
    // returns an all-zero field and `true` -- a silently wrong answer,
    // and the one shape of bug the parity harness cannot catch because
    // both sides would agree on nothing.
    const TfToken typeName = prim.GetTypeName();
    if (!_IsWeightObjectType(typeName)) {
        *error = "unknown weight object type " + typeName.GetString() +
                 " on " + weightPrimPath.GetString();
        return false;
    }
    if (_IsVolumeWeightType(typeName) || typeName == "RigExecCombineWeight") {
        std::vector<float> resolved;
        if (!_ResolveVolumeWeights(prim, count, time, &resolved, error,
                                   currentPoints)) {
            return false;
        }
        TfToken volumePolicy("clamp");
        if (UsdAttribute a =
                prim.GetAttribute(TfToken("rigExec:rangePolicy"))) {
            a.Get(&volumePolicy, time);
        }
        for (float &w : resolved) {
            if (!std::isfinite(w)) {
                *error = "non-finite weight on " + weightPrimPath.GetString();
                return false;
            }
            if (w < 0.0f || w > 1.0f) {
                if (volumePolicy != "clamp") {
                    *error = "strict range violation on " +
                             weightPrimPath.GetString();
                    return false;
                }
                w = std::min(std::max(w, 0.0f), 1.0f);
            }
        }
        *weights = std::move(resolved);
        return true;
    }

    TfToken representation("constant");
    if (UsdAttribute a = prim.GetAttribute(
            TfToken("rigExec:representation"))) {
        a.Get(&representation, time);
    }
    const float defaultWeight = _ResolvedRead(
        _resolvedInputs, prim, "rigExec:defaultWeight", 0.0f, time);
    TfToken rangePolicy("strict");
    if (UsdAttribute a = prim.GetAttribute(TfToken("rigExec:rangePolicy"))) {
        a.Get(&rangePolicy, time);
    }
    if (rangePolicy != "strict" && rangePolicy != "clamp") {
        *error = "unknown rangePolicy on " + weightPrimPath.GetString();
        return false;
    }

    const bool isDynamic = prim.GetTypeName() == "RigExecDynamicWeight";
    if (isDynamic) {
        TfToken operation("multiply");
        if (UsdAttribute a =
                prim.GetAttribute(TfToken("rigExec:operation"))) {
            a.Get(&operation, time);
        }
        if (operation != "multiply") {
            *error = "unknown dynamic-weight operation on " +
                     weightPrimPath.GetString();
            return false;
        }

        // Base field first, then r_i = (b_i * d) * s + a (spec §4.1).
        std::vector<float> base(count, 1.0f);
        SdfPathVector baseTargets;
        if (UsdRelationship rel =
                prim.GetRelationship(TfToken("rigExec:baseWeight"))) {
            rel.GetTargets(&baseTargets);
        }
        if (baseTargets.size() > 1) {
            *error = "rigExec:baseWeight must have at most one target on " +
                     weightPrimPath.GetString();
            return false;
        }
        if (baseTargets.empty()) {
            // Without a base, only constant representation is legal and
            // b_i = 1 everywhere (spec §4.1).
            if (representation != "constant") {
                *error = "no-base dynamic weight must be constant on " +
                         weightPrimPath.GetString();
                return false;
            }
        } else {
            const UsdPrim basePrim = _stage->GetPrimAtPath(baseTargets[0]);
            if (!basePrim) {
                *error = "missing base weight object on " +
                         weightPrimPath.GetString();
                return false;
            }
            // The base descriptor must exactly match the dynamic
            // descriptor: canonical target, representation, and sparse
            // support (spec §4.1).
            auto canonicalWeightTarget =
                [this, &time](const UsdPrim &p) -> SdfPath {
                SdfPathVector t;
                if (UsdRelationship rel = p.GetRelationship(
                        TfToken("rigExec:weightTarget"))) {
                    rel.GetTargets(&t);
                }
                return t.size() == 1 ? _ResolveGeometryInput(_stage, t[0])
                                     : SdfPath();
            };
            if (canonicalWeightTarget(prim) !=
                    canonicalWeightTarget(basePrim) ||
                canonicalWeightTarget(prim).IsEmpty()) {
                *error = "dynamic/base weight target mismatch on " +
                         weightPrimPath.GetString();
                return false;
            }
            TfToken baseRepresentation("constant");
            if (UsdAttribute a = basePrim.GetAttribute(
                    TfToken("rigExec:representation"))) {
                a.Get(&baseRepresentation, time);
            }
            if (baseRepresentation != representation) {
                *error = "dynamic/base representation mismatch on " +
                         weightPrimPath.GetString();
                return false;
            }
            if (representation == "sparse") {
                // The dynamic descriptor's sparse support is inherited
                // from the base; a dynamic prim that authors its own
                // support must match the base exactly (spec §4.1).
                VtIntArray mine;
                if (UsdAttribute a =
                        prim.GetAttribute(TfToken("rigExec:indices"))) {
                    a.Get(&mine, time);
                }
                if (!mine.empty()) {
                    VtIntArray theirs;
                    if (UsdAttribute a = basePrim.GetAttribute(
                            TfToken("rigExec:indices"))) {
                        a.Get(&theirs, time);
                    }
                    const std::set<int> mySupport(mine.begin(), mine.end());
                    const std::set<int> baseSupport(
                        theirs.begin(), theirs.end());
                    if (mySupport != baseSupport) {
                        *error = "dynamic/base sparse support mismatch on " +
                                 weightPrimPath.GetString();
                        return false;
                    }
                }
            }
            // currentPoints is forwarded: a dynamic weight modulating a
            // current-phase volume must still measure against the
            // in-flight points, or the base silently reverts to the
            // reference field.
            if (!_ResolveWeights(baseTargets[0], count, time, &base, error,
                                 currentPoints)) {
                return false;
            }
        }
        const float driver = _ResolvedRead(
            _resolvedInputs, prim, "inputs:driver", 1.0f, time);
        const float scale = _ResolvedRead(
            _resolvedInputs, prim, "inputs:scale", 1.0f, time);
        const float bias = _ResolvedRead(
            _resolvedInputs, prim, "inputs:bias", 0.0f, time);
        for (size_t i = 0; i < count; ++i) {
            float r = (base[i] * driver) * scale + bias;
            if (!std::isfinite(r)) {
                *error = "non-finite dynamic weight on " +
                         weightPrimPath.GetString();
                return false;
            }
            if (r < 0.0f || r > 1.0f) {
                if (rangePolicy == "clamp") {
                    r = std::min(std::max(r, 0.0f), 1.0f);
                } else {
                    *error = "strict range violation on " +
                             weightPrimPath.GetString();
                    return false;
                }
            }
            (*weights)[i] = r;
        }
        return true;
    }

    // Static weights are time-invariant by contract: reject time samples
    // and value connections on every field (spec §4.1).
    static const TfToken staticFields[] = {
        TfToken("rigExec:values"), TfToken("rigExec:indices"),
        TfToken("rigExec:defaultWeight"), TfToken("rigExec:representation"),
        TfToken("rigExec:rangePolicy")};
    for (const TfToken &field : staticFields) {
        const UsdAttribute a = prim.GetAttribute(field);
        if (a && (a.GetNumTimeSamples() > 0 || a.HasAuthoredConnections())) {
            *error = "static weight field " + field.GetString() +
                     " has time samples or connections on " +
                     weightPrimPath.GetString();
            return false;
        }
    }
    VtFloatArray values;
    if (UsdAttribute a = prim.GetAttribute(TfToken("rigExec:values"))) {
        a.Get(&values, time);
    }
    if (representation == "constant") {
        if (!values.empty()) {
            *error = "constant weight must not author values on " +
                     weightPrimPath.GetString();
            return false;
        }
        weights->assign(count, defaultWeight);
    } else if (representation == "dense") {
        if (values.size() != count) {
            *error = "dense weight cardinality mismatch on " +
                     weightPrimPath.GetString();
            return false;
        }
        // Canonical encoding requires dense defaultWeight = 0 (spec §4.1).
        if (defaultWeight != 0.0f) {
            *error = "dense weight requires canonical defaultWeight 0 on " +
                     weightPrimPath.GetString();
            return false;
        }
        weights->assign(values.begin(), values.end());
    } else if (representation == "sparse") {
        VtIntArray indices;
        if (UsdAttribute a = prim.GetAttribute(TfToken("rigExec:indices"))) {
            a.Get(&indices, time);
        }
        if (indices.size() != values.size()) {
            *error = "sparse index/value size mismatch on " +
                     weightPrimPath.GetString();
            return false;
        }
        weights->assign(count, defaultWeight);
        std::set<int> seen;
        for (size_t i = 0; i < indices.size(); ++i) {
            if (indices[i] < 0 || static_cast<size_t>(indices[i]) >= count) {
                *error = "sparse index out of range on " +
                         weightPrimPath.GetString();
                return false;
            }
            // Indices are unique logical element indices; authored pair
            // order is non-semantic (spec §4.1).
            if (!seen.insert(indices[i]).second) {
                *error = "duplicate sparse index on " +
                         weightPrimPath.GetString();
                return false;
            }
            (*weights)[indices[i]] = values[i];
        }
    } else {
        *error = "unknown weight representation on " +
                 weightPrimPath.GetString();
        return false;
    }

    for (float w : *weights) {
        if (!std::isfinite(w) ||
            (rangePolicy == "strict" && (w < 0.0f || w > 1.0f))) {
            *error = "weight range violation on " +
                     weightPrimPath.GetString();
            return false;
        }
    }
    if (rangePolicy == "clamp") {
        for (float &w : *weights) {
            w = std::min(std::max(w, 0.0f), 1.0f);
        }
    }
    return true;
}

VtVec3fArray
RigExecRigEvaluator::_EvaluateChain(
    const SdfPath &target,
    const std::vector<const RigExecMoverRecord *> &chain,
    const RigExecRigPose &pose,
    const std::unordered_map<SdfPath, GfMatrix4d, SdfPath::Hash> &baseProviderMatrices,
    const std::unordered_map<SdfPath, GfMatrix4d, SdfPath::Hash> &finalProviderMatrices,
    UsdTimeCode time,
    std::vector<std::string> *diagnostics,
    const std::unordered_map<SdfPath, GfMatrix4d, SdfPath::Hash> &geometryConstraintDeltas) const
{
    // The oracle resolves a read phase ITSELF, from the authored metadata,
    // and reads the same recorded snapshots. That keeps it independent of
    // RigExecResolveRevisionBinding and RigExecAssembleParameters -- which is
    // what makes parity a real check -- while sharing the authored INTENT,
    // which it must, or the two paths are evaluating different rigs.

    // Base: the stock resolved value of the exact native property
    // (spec §7.2). The base revision is retained: blend-shape deltas
    // derive against base points, not the preceding revision (spec §7.3).
    VtVec3fArray points;
    const UsdAttribute baseAttr = _stage->GetAttributeAtPath(target);
    if (!baseAttr || !baseAttr.Get(&points, time)) {
        diagnostics->push_back("no base value for " + target.GetString());
        return points;
    }
    const VtVec3fArray basePoints = points;

    for (const RigExecMoverRecord *mover : chain) {
        const UsdPrim prim = _stage->GetPrimAtPath(mover->moverPath);
        if (!prim || !_IsEnabled(prim, time)) {
            continue;  // pass-through (spec §4.2)
        }

        // Resolve the common envelope against the PRECEDING revision. A
        // current-phase volume therefore measures exactly the points that
        // enter this mover. A bound object supersedes inputs:defaultWeight.
        const VtVec3fArray preceding = points;
        std::vector<float> envelope(points.size(), 1.0f);
        SdfPathVector weightObjects;
        if (const UsdRelationship rel =
                prim.GetRelationship(TfToken("rigExec:weightObject"))) {
            rel.GetTargets(&weightObjects);
        }
        if (!weightObjects.empty()) {
            const std::vector<GfVec3f> currentPoints(
                preceding.begin(), preceding.end());
            std::string error;
            if (!_ResolveWeights(weightObjects[0], points.size(), time,
                                 &envelope, &error, &currentPoints)) {
                diagnostics->push_back(
                    "MoverFailed " + mover->moverPath.GetString() + ": " +
                    error);
                continue;
            }
        } else {
            const float scalar = _ResolvedRead(
                _resolvedInputs, prim, "inputs:defaultWeight", 1.0f, time);
            if (!std::isfinite(scalar) || scalar < 0.0f || scalar > 1.0f) {
                diagnostics->push_back(
                    "MoverFailed " + mover->moverPath.GetString() +
                    ": inputs:defaultWeight must be finite and in [0, 1]");
                continue;
            }
            std::fill(envelope.begin(), envelope.end(), scalar);
        }

        if (_IsSourceFrameConstraintType(prim.GetTypeName())) {
            const auto delta = geometryConstraintDeltas.find(mover->moverPath);
            if (delta == geometryConstraintDeltas.end()) continue;
            for (auto &point : points)
                point = GfVec3f(delta->second.TransformAffine(GfVec3d(point)));
        } else if (const RigExecMoverHandler *oracleHandler =
                       RigExecFindMoverHandler(mover->schemaType)) {
            // The parity-oracle branch, from the mover's own row (see movers/).
            // A type with no row, or a row with no oracle, falls through to
            // the shared envelope blend over the unmodified points.
            if (oracleHandler->oracle) {
                // A dense blend sample phased read answers from the
                // recorded snapshot, resolved through this revision
                // compiled binding -- the one piece of evaluator
                // compile state the oracle needs.
                auto sampleSnapshot = [this, &target, mover](
                    const SdfPath &inputPath,
                    const SdfPath &samplePath) -> const VtValue * {
                    const auto compiledChain =
                        _graphChains.find(target);
                    if (compiledChain == _graphChains.end()) {
                        return nullptr;
                    }
                    for (const auto &revision :
                         compiledChain->second) {
                        if (revision.moverPath != mover->moverPath) {
                            continue;
                        }
                        const auto channel =
                            revision.binding.blendSamples.find(inputPath);
                        if (channel ==
                            revision.binding.blendSamples.end()) {
                            continue;
                        }
                        for (const auto &binding : channel->second) {
                            if (binding.sample != samplePath) {
                                continue;
                            }
                            return _chainSnapshots.Lookup(
                                binding.points, binding.phase,
                                mover->moverPath);
                        }
                    }
                    return nullptr;
                };
                const RigExecMoverOracleContext oracleCtx{
                    _stage,
                    prim,
                    mover->moverPath,
                    target,
                    time,
                    _resolvedInputs,
                    _chainSnapshots,
                    baseProviderMatrices,
                    finalProviderMatrices,
                    diagnostics,
                    &points,
                    basePoints,
                    sampleSnapshot};
                if (oracleHandler->oracle(oracleCtx) ==
                    RigExecOracleResult::PassThrough) {
                    continue;
                }
            }
        }

        // Every branch above computes the operation's full-strength
        // candidate. The universal envelope is the one and only blend back
        // over the preceding revision.
        if (points.size() != preceding.size() ||
            envelope.size() != points.size()) {
            diagnostics->push_back(
                "MoverFailed " + mover->moverPath.GetString() +
                ": result cardinality changed; revision passed through");
            points = preceding;
            continue;
        }
        for (size_t i = 0; i < points.size(); ++i) {
            points[i] = RigExecBlendEnvelope(
                preceding[i], points[i], envelope[i]);
        }
    }
    return points;
}

void
RigExecRigEvaluator::_ResolveSkinLayoutInputs() const
{
    _skinLayoutInputs.clear();
    _skinLayoutInputsValid = true;
    for (const RigExecMoverRecord &record : _movers) {
        const RigExecMoverHandler *layoutHandler =
            RigExecFindMoverHandler(record.schemaType);
        if (!layoutHandler || layoutHandler->layoutAttributes.empty()) {
            continue;
        }
        const UsdPrim prim = _stage->GetPrimAtPath(record.moverPath);
        if (!prim) {
            continue;
        }
        for (const TfToken &name : layoutHandler->layoutAttributes) {
            // The SAME walk the value is read through
            // (RigExecResolvedInputs::GetAttribute): a single authored
            // connection per hop, the resolved map consulted at every hop,
            // cycles refused. Every path along it is a path an override can
            // stand on and be seen by the read, so every path along it
            // belongs in this set -- an override one hop upstream of a
            // connected rigExec:jointIndices is the case that makes the
            // difference between a re-read and a silently stale deformation.
            UsdAttribute attribute = prim.GetAttribute(name);
            if (!attribute) {
                // Not authored and not in the schema: an override could
                // still create the opinion the read would find, so the
                // property itself is named even where the attribute is not.
                _skinLayoutInputs.insert(
                    record.moverPath.AppendProperty(name));
                continue;
            }
            while (attribute &&
                   _skinLayoutInputs.insert(attribute.GetPath()).second) {
                SdfPathVector connections;
                if (attribute.HasAuthoredConnections()) {
                    attribute.GetConnections(&connections);
                }
                if (connections.size() != 1) {
                    break;
                }
                attribute = _stage->GetAttributeAtPath(connections[0]);
            }
        }
    }
}

bool
RigExecRigEvaluator::_OverridesReachSkinLayout(
    const std::vector<RigExecValueOverride> &overrides) const
{
    if (overrides.empty()) {
        return false;
    }
    if (!_skinLayoutInputsValid) {
        _ResolveSkinLayoutInputs();
    }
    for (const RigExecValueOverride &o : overrides) {
        // A computation override names no property, so there is nothing to
        // compare it against: it is taken to reach everything.
        if (o.attribute.IsEmpty()) {
            return true;
        }
        if (_skinLayoutInputs.count(o.prim.AppendProperty(o.attribute))) {
            return true;
        }
    }
    return false;
}

size_t
RigExecRigEvaluator::GetSkinTopologyCacheSize() const
{
    return _skinTopologies.GetSize();
}

size_t
RigExecRigEvaluator::GetBlendSampleCacheSize() const
{
    return _blendSampleShapes.GetSize();
}

} // namespace rigExec
