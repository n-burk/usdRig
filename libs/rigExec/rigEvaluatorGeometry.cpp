// Geometry chains, weight fields, and skin-layout inputs.

#include "rigEvaluatorInternal.h"
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

// Attribute names, values, and weight types the weight oracle reads per
// weight object per frame, interned once. A per-frame TfToken construction
// takes the token registry lock.
const TfToken _kGeoInputWeights("rigExec:inputWeights");
const TfToken _kGeoCombineMode("rigExec:combineMode");
const TfToken _kGeoPlaneAxis("rigExec:planeAxis");
const TfToken _kGeoPlaneBounds("rigExec:planeBounds");
const TfToken _kGeoRangePolicy("rigExec:rangePolicy");
const TfToken _kGeoRepresentation("rigExec:representation");
const TfToken _kGeoDefaultWeight("rigExec:defaultWeight");
const TfToken _kGeoOperation("rigExec:operation");
const TfToken _kGeoBaseWeight("rigExec:baseWeight");
const TfToken _kGeoWeightTarget("rigExec:weightTarget");
const TfToken _kGeoIndices("rigExec:indices");
const TfToken _kGeoValues("rigExec:values");
const TfToken _kGeoSampleSource("rigExec:sampleSource");
const TfToken _kGeoCurve("rigExec:curve");
const TfToken _kGeoStrength("inputs:strength");
const TfToken _kGeoInvert("inputs:invert");
const TfToken _kGeoFalloffMin("inputs:falloffMin");
const TfToken _kGeoFalloffMax("inputs:falloffMax");
const TfToken _kGeoExtentU("inputs:extentU");
const TfToken _kGeoExtentV("inputs:extentV");
const TfToken _kGeoScaleX("inputs:scaleX");
const TfToken _kGeoScaleY("inputs:scaleY");
const TfToken _kGeoScaleZ("inputs:scaleZ");
const TfToken _kGeoScaleXPos("inputs:scaleXPos");
const TfToken _kGeoScaleYPos("inputs:scaleYPos");
const TfToken _kGeoScaleZPos("inputs:scaleZPos");
const TfToken _kGeoScaleXNeg("inputs:scaleXNeg");
const TfToken _kGeoScaleYNeg("inputs:scaleYNeg");
const TfToken _kGeoScaleZNeg("inputs:scaleZNeg");
const TfToken _kGeoDriver("inputs:driver");
const TfToken _kGeoScale("inputs:scale");
const TfToken _kGeoBias("inputs:bias");
const TfToken _kGeoMultiply("multiply");
const TfToken _kGeoAdd("add");
const TfToken _kGeoSubtract("subtract");
const TfToken _kGeoMax("max");
const TfToken _kGeoMin("min");
const TfToken _kGeoAverage("average");
const TfToken _kGeoOverlay("overlay");
const TfToken _kGeoReference("reference");
const TfToken _kGeoCurrent("current");
const TfToken _kGeoAxisX("x");
const TfToken _kGeoAxisY("y");
const TfToken _kGeoAxisZ("z");
const TfToken _kGeoUnbounded("unbounded");
const TfToken _kGeoBounded("bounded");
const TfToken _kGeoConstant("constant");
const TfToken _kGeoStrict("strict");
const TfToken _kGeoClamp("clamp");
const TfToken _kGeoDense("dense");
const TfToken _kGeoSparse("sparse");
const TfToken _kGeoCombineWeight("RigExecCombineWeight");
const TfToken _kGeoPlaneWeight("RigExecPlaneWeight");
const TfToken _kGeoSphereWeight("RigExecSphereWeight");
const TfToken _kGeoCurveWeight("RigExecCurveWeight");
const TfToken _kGeoDynamicWeight("RigExecDynamicWeight");
const TfToken _kGeoWeightObject("rigExec:weightObject");
const TfToken _kGeoPoints("points");
// The mover envelope default, in the inputs: namespace -- distinct from a
// weight object's rigExec:defaultWeight above.
const TfToken _kGeoInputsDefaultWeight("inputs:defaultWeight");

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
            return target.AppendProperty(_kGeoPoints);
        }
    }
    return target;
}

} // namespace evaluatorDetail

bool
RigExecRigEvaluator::_ReadTargetPoints(
    const UsdPrim &prim, const TfToken &relationshipName, UsdTimeCode time,
    std::vector<GfVec3f> *points) const
{
    points->clear();
    SdfPathVector targets;
    if (UsdRelationship rel = prim.GetRelationship(relationshipName)) {
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
    if (typeName == _kGeoCombineWeight) {
        SdfPathVector inputs;
        if (UsdRelationship rel =
                prim.GetRelationship(_kGeoInputWeights)) {
            rel.GetTargets(&inputs);
        }
        TfToken modeName = _kGeoMultiply;
        if (UsdAttribute a =
                prim.GetAttribute(_kGeoCombineMode)) {
            a.Get(&modeName, time);
        }
        RigExecWeightCombine mode;
        if (modeName == _kGeoMultiply) {
            mode = RigExecWeightCombine::Multiply;
        } else if (modeName == _kGeoAdd) {
            mode = RigExecWeightCombine::Add;
        } else if (modeName == _kGeoSubtract) {
            mode = RigExecWeightCombine::Subtract;
        } else if (modeName == _kGeoMax) {
            mode = RigExecWeightCombine::Max;
        } else if (modeName == _kGeoMin) {
            mode = RigExecWeightCombine::Min;
        } else if (modeName == _kGeoAverage) {
            mode = RigExecWeightCombine::Average;
        } else if (modeName == _kGeoOverlay) {
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
            _resolvedInputs, prim, _kGeoStrength, 1.0f, time);
        const float invert = _ResolvedRead(
            _resolvedInputs, prim, _kGeoInvert, 0.0f, time);
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
    TfToken samplePhase = _kGeoReference;
    if (UsdAttribute a = prim.GetAttribute(_samplePhaseAttr)) {
        a.Get(&samplePhase, time);
    }
    if (samplePhase == _kGeoCurrent) {
        if (!currentPoints) {
            *error = who() +
                     ": rigExec:samplePhase is `current` but no in-flight "
                     "points were supplied";
            return false;
        }
        samplePoints = *currentPoints;
    } else if (samplePhase == _kGeoReference) {
        // An explicit sampleSource wins over the weighted domain, which
        // is how one mesh is weighted by another mesh's shape.
        if (!_ReadTargetPoints(prim, _kGeoSampleSource, time,
                               &samplePoints) &&
            !_ReadTargetPoints(prim, _kGeoWeightTarget, time,
                               &samplePoints)) {
            *error = who() + ": could not read the points to sample";
            return false;
        }
    } else {
        *error = who() + ": unknown rigExec:samplePhase " +
                 samplePhase.GetString();
        return false;
    }
    if (samplePoints.size() != count) {
        *error = who() + ": sampled point count does not match the target";
        return false;
    }

    RigExecFalloffParams params;
    auto readFloat = [this, &prim, time](const TfToken &name, float fallback) {
        return _ResolvedRead(
            _resolvedInputs, prim, name, fallback, time);
    };
    params.falloffMin = readFloat(_kGeoFalloffMin, 0.0f);
    params.falloffMax = readFloat(_kGeoFalloffMax, 1.0f);
    params.invert = readFloat(_kGeoInvert, 0.0f);
    params.strength = readFloat(_kGeoStrength, 1.0f);
    params.curve = _BakeFalloffLut(prim);

    if (typeName == _kGeoPlaneWeight) {
        TfToken axis = _kGeoAxisY;
        if (UsdAttribute a = prim.GetAttribute(_kGeoPlaneAxis)) {
            a.Get(&axis, time);
        }
        const int axisIndex =
            axis == _kGeoAxisX ? 0
            : (axis == _kGeoAxisY ? 1 : (axis == _kGeoAxisZ ? 2 : -1));
        if (axisIndex < 0) {
            *error = who() + ": unknown rigExec:planeAxis " + axis.GetString();
            return false;
        }
        // Bounded clips the field to the in-plane rectangle. Mirrors
        // _BuildPlaneWeightPacket exactly, including reading the extents
        // only in the bounded arm -- the two paths have to agree value
        // for value or the parity harness fires.
        TfToken boundsMode = _kGeoUnbounded;
        if (UsdAttribute a =
                prim.GetAttribute(_kGeoPlaneBounds)) {
            a.Get(&boundsMode, time);
        }
        RigExecPlaneBounds extent;
        const RigExecPlaneBounds *extentPtr = nullptr;
        if (boundsMode == _kGeoBounded) {
            extent.extentU = readFloat(_kGeoExtentU, 1.0f);
            extent.extentV = readFloat(_kGeoExtentV, 1.0f);
            for (const float e : {extent.extentU, extent.extentV}) {
                if (!std::isfinite(e) || e <= 0.0f) {
                    *error = who() +
                             ": inputs:extentU/V must be finite and positive "
                             "when rigExec:planeBounds is `bounded`";
                    return false;
                }
            }
            extentPtr = &extent;
        } else if (boundsMode != _kGeoUnbounded) {
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
    const float sx = readFloat(_kGeoScaleX, 1.0f);
    const float sy = readFloat(_kGeoScaleY, 1.0f);
    const float sz = readFloat(_kGeoScaleZ, 1.0f);
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

    if (typeName == _kGeoSphereWeight) {
        const GfVec3f positiveScales(
            readFloat(_kGeoScaleXPos, 1.0f),
            readFloat(_kGeoScaleYPos, 1.0f),
            readFloat(_kGeoScaleZPos, 1.0f));
        const GfVec3f negativeScales(
            readFloat(_kGeoScaleXNeg, 1.0f),
            readFloat(_kGeoScaleYNeg, 1.0f),
            readFloat(_kGeoScaleZNeg, 1.0f));
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
    if (typeName == _kGeoCurveWeight) {
        std::vector<GfVec3f> curvePoints;
        if (!_ReadTargetPoints(prim, _kGeoCurve, time, &curvePoints) ||
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
    if (_IsVolumeWeightType(typeName) || typeName == _kGeoCombineWeight) {
        std::vector<float> resolved;
        if (!_ResolveVolumeWeights(prim, count, time, &resolved, error,
                                   currentPoints)) {
            return false;
        }
        TfToken volumePolicy = _kGeoClamp;
        if (UsdAttribute a =
                prim.GetAttribute(_kGeoRangePolicy)) {
            a.Get(&volumePolicy, time);
        }
        for (float &w : resolved) {
            if (!std::isfinite(w)) {
                *error = "non-finite weight on " + weightPrimPath.GetString();
                return false;
            }
            if (w < 0.0f || w > 1.0f) {
                if (volumePolicy != _kGeoClamp) {
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

    TfToken representation = _kGeoConstant;
    if (UsdAttribute a = prim.GetAttribute(
            _kGeoRepresentation)) {
        a.Get(&representation, time);
    }
    const float defaultWeight = _ResolvedRead(
        _resolvedInputs, prim, _kGeoDefaultWeight, 0.0f, time);
    TfToken rangePolicy = _kGeoStrict;
    if (UsdAttribute a = prim.GetAttribute(_kGeoRangePolicy)) {
        a.Get(&rangePolicy, time);
    }
    if (rangePolicy != _kGeoStrict && rangePolicy != _kGeoClamp) {
        *error = "unknown rangePolicy on " + weightPrimPath.GetString();
        return false;
    }

    const bool isDynamic = typeName == _kGeoDynamicWeight;
    if (isDynamic) {
        TfToken operation = _kGeoMultiply;
        if (UsdAttribute a =
                prim.GetAttribute(_kGeoOperation)) {
            a.Get(&operation, time);
        }
        if (operation != _kGeoMultiply) {
            *error = "unknown dynamic-weight operation on " +
                     weightPrimPath.GetString();
            return false;
        }

        // Base field first, then r_i = (b_i * d) * s + a (spec §4.1).
        std::vector<float> base(count, 1.0f);
        SdfPathVector baseTargets;
        if (UsdRelationship rel =
                prim.GetRelationship(_kGeoBaseWeight)) {
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
            if (representation != _kGeoConstant) {
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
                        _kGeoWeightTarget)) {
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
            TfToken baseRepresentation = _kGeoConstant;
            if (UsdAttribute a = basePrim.GetAttribute(
                    _kGeoRepresentation)) {
                a.Get(&baseRepresentation, time);
            }
            if (baseRepresentation != representation) {
                *error = "dynamic/base representation mismatch on " +
                         weightPrimPath.GetString();
                return false;
            }
            if (representation == _kGeoSparse) {
                // The dynamic descriptor's sparse support is inherited
                // from the base; a dynamic prim that authors its own
                // support must match the base exactly (spec §4.1).
                VtIntArray mine;
                if (UsdAttribute a =
                        prim.GetAttribute(_kGeoIndices)) {
                    a.Get(&mine, time);
                }
                if (!mine.empty()) {
                    VtIntArray theirs;
                    if (UsdAttribute a = basePrim.GetAttribute(
                            _kGeoIndices)) {
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
            _resolvedInputs, prim, _kGeoDriver, 1.0f, time);
        const float scale = _ResolvedRead(
            _resolvedInputs, prim, _kGeoScale, 1.0f, time);
        const float bias = _ResolvedRead(
            _resolvedInputs, prim, _kGeoBias, 0.0f, time);
        for (size_t i = 0; i < count; ++i) {
            float r = (base[i] * driver) * scale + bias;
            if (!std::isfinite(r)) {
                *error = "non-finite dynamic weight on " +
                         weightPrimPath.GetString();
                return false;
            }
            if (r < 0.0f || r > 1.0f) {
                if (rangePolicy == _kGeoClamp) {
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
    static const TfToken *staticFields[] = {
        &_kGeoValues, &_kGeoIndices,
        &_kGeoDefaultWeight, &_kGeoRepresentation,
        &_kGeoRangePolicy};
    for (const TfToken *field : staticFields) {
        const UsdAttribute a = prim.GetAttribute(*field);
        if (a && (a.GetNumTimeSamples() > 0 || a.HasAuthoredConnections())) {
            *error = "static weight field " + field->GetString() +
                     " has time samples or connections on " +
                     weightPrimPath.GetString();
            return false;
        }
    }
    VtFloatArray values;
    if (UsdAttribute a = prim.GetAttribute(_kGeoValues)) {
        a.Get(&values, time);
    }
    if (representation == _kGeoConstant) {
        if (!values.empty()) {
            *error = "constant weight must not author values on " +
                     weightPrimPath.GetString();
            return false;
        }
        weights->assign(count, defaultWeight);
    } else if (representation == _kGeoDense) {
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
    } else if (representation == _kGeoSparse) {
        VtIntArray indices;
        if (UsdAttribute a = prim.GetAttribute(_kGeoIndices)) {
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
            (rangePolicy == _kGeoStrict && (w < 0.0f || w > 1.0f))) {
            *error = "weight range violation on " +
                     weightPrimPath.GetString();
            return false;
        }
    }
    if (rangePolicy == _kGeoClamp) {
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
                prim.GetRelationship(_kGeoWeightObject)) {
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
                _resolvedInputs, prim, _kGeoInputsDefaultWeight, 1.0f, time);
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
