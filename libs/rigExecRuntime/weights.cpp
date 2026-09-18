//
// rigExecRuntime weight family (M2): WeightPacket, VolumePlacements.
//
// A bit-identical port of RigExecBakedRunWeightStep
// (libs/rigExec/bakedWeights.cpp) with its builders
// (libs/rigExec/weightPackets.cpp, libs/rigExecMath/weightFields.cpp).
// Gf -> Rr, TfToken comparisons -> string-table text comparisons, the
// arithmetic untouched: float stays float, in the same order.
//
// Where the baked step reads the stage per frame, the runtime reads the
// frame record. Scalar inputs arrive through the uid holders (the
// runtime form of RigExecBakedRead); the point arrays a volume measures
// resolve through two layers, in order: a live (wasDefault == false)
// pathReads entry for the attribute, else the chain base of the chain
// whose target IS that attribute path. Chain targets are attribute
// paths (the prologue builds the base query on GetAttributeAtPath of
// the target), so the second layer matches by exact id, not by
// heuristics. Its values equal the baked gather's when the attribute
// carries no connections and no overlay entry -- the overlay holds
// scalars only during capture, so arrays never hit it -- and the
// parity test proves it per fixture by comparing every packet bit for
// bit. A connected weight target is a known residual risk: the correct
// fix is a bake-side recorder hook in the weight gather (a capture
// hole, not a runtime approximation), which this file cannot add.
//
// Two deliberate deferrals, both loud rather than silent:
//   * RigExecCurvenetWeight's numerical bind (cutMesh + curvenet +
//     sparseSolve, ~140KB of numerics with no fixture coverage) is not
//     ported: a curvenet weight whose tokens are valid fails the step
//     naming the object. The token-invalid path needs no bind and is
//     exact. No shipped fixture carries a curvenet weight.
//   * Unknown weight object TYPES and unknown structural tokens stay
//     exactly what the baked builders return for them: invalid packets
//     (bare for an unknown type, carrying the object's tokens
//     otherwise), never a defaulted valid field.
//

#include "rigExecRuntime/store.h"

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <limits>
#include <string>
#include <unordered_map>
#include <utility>
#include <vector>

namespace rigExec {

struct RrWeightScratch {
    // The record the lookup below was built from. Keyed by frame time:
    // Execute passes the selected record's own frame into the walk, so
    // equality with a record's frame is exact.
    bool haveRecord = false;
    double recordFrame = 0.0;
    // Live stage reads this frame: attribute path id -> pathReads index,
    // for wasDefault == false entries only. A rest-time read is not what
    // a frame-time gather consumed.
    std::unordered_map<uint32_t, size_t> liveReads;
    // Chain target attribute id -> chain index, built once (epoch data).
    std::unordered_map<uint32_t, size_t> chainByTarget;
};

bool
RrWeightSizeScratch(RrProgram *program, std::string *error)
{
    if (!program || !program->geometry) {
        if (error) {
            *error = "weight scratch needs the geometry domain";
        }
        return false;
    }
    RrWeightScratch *scratch = new RrWeightScratch();
    const std::vector<RigExecWireChain> &chains =
        program->geometry->chains;
    for (size_t c = 0; c < chains.size(); ++c) {
        scratch->chainByTarget.emplace(chains[c].target, c);
    }
    program->weights = std::shared_ptr<void>(scratch);
    return true;
}

namespace {

// The step's label for fatal errors. Weight steps cannot fail in the
// baked program (their indices close by construction); every failure
// below names a wire violation the runtime refuses to crash on.
std::string
_RrStepLabel(const RrProgram *program, size_t step)
{
    return program->TextOrEmpty((*program->steps)[step].label);
}

const RigExecWireFrameInputs *
_RrFindRecord(const RrProgram *program, double time)
{
    for (const RigExecWireFrameInputs &record :
         program->inputs->frames) {
        if (record.frame == time) {
            return &record;
        }
    }
    return nullptr;
}

RrWeightScratch *
_RrScratch(const RrProgram *program)
{
    return static_cast<RrWeightScratch *>(program->weights.get());
}

void
_RrRefreshReads(RrProgram *program, const RigExecWireFrameInputs *record)
{
    RrWeightScratch *scratch = _RrScratch(program);
    if (scratch->haveRecord && scratch->recordFrame == record->frame) {
        return;
    }
    scratch->liveReads.clear();
    for (size_t i = 0; i < record->pathReads.size(); ++i) {
        const RigExecWirePathRead &read = record->pathReads[i];
        if (read.wasDefault == 0) {
            scratch->liveReads.emplace(read.path, i);
        }
    }
    scratch->haveRecord = true;
    scratch->recordFrame = record->frame;
}

// A bound weight input as the float the builders consume. The holder
// (or the wire constant when the input takes no uid) always carries
// the input's own tag; the double arm mirrors the float-from-double
// coercion GetAttribute applies to a scalar float read.
float
_RrReadWeightFloat(const RrProgram *program, size_t object, int field)
{
    const RrInputValue value = program->ReadWeight(object, field);
    if (value.tag == RigExecWireInput::Tag::Float) {
        return value.f32;
    }
    if (value.tag == RigExecWireInput::Tag::Double) {
        return static_cast<float>(value.f64);
    }
    return 0.0f;
}

int
_RrReadWeightInt(const RrProgram *program, size_t object, int field)
{
    const RrInputValue value = program->ReadWeight(object, field);
    if (value.tag == RigExecWireInput::Tag::Int) {
        return int(value.i32);
    }
    if (value.tag == RigExecWireInput::Tag::Float) {
        return int(value.f32);
    }
    if (value.tag == RigExecWireInput::Tag::Double) {
        return int(value.f64);
    }
    return 0;
}

// Appends one attribute's points to a volume gather. A failed read --
// no live entry, a known-absent mark, a mistyped holding, or a chain
// with no base this frame -- contributes nothing, exactly as a false
// GetAttribute does in the baked gather.
void
_RrGatherPoints(const RrProgram *program, RrWeightScratch *scratch,
               const RigExecWireFrameInputs *record, uint32_t path,
               std::vector<RrVec3f> *out)
{
    const auto live = scratch->liveReads.find(path);
    if (live != scratch->liveReads.end() &&
        live->second < record->pathReads.size()) {
        const RigExecWirePathValue &value =
            record->pathReads[live->second].value;
        if (value.tag == RigExecWirePathValue::Tag::Vec3fArray) {
            for (const RigExecWireVec3f &p : value.vec3s) {
                out->push_back(RrVec3f(p[0], p[1], p[2]));
            }
        }
        return;
    }
    const auto chain = scratch->chainByTarget.find(path);
    if (chain == scratch->chainByTarget.end() ||
        chain->second >= record->chainBases.size() ||
        chain->second >= record->chainHaveBase.size() ||
        !record->chainHaveBase[chain->second]) {
        return;
    }
    for (const RigExecWireVec3f &p :
         record->chainBases[chain->second]) {
        out->push_back(RrVec3f(p[0], p[1], p[2]));
    }
}

// The element count one attribute contributes to a combine's target
// fallback: the same read as above, measured rather than copied.
size_t
_RrGatherPointCount(const RrProgram *program, RrWeightScratch *scratch,
                   const RigExecWireFrameInputs *record, uint32_t path)
{
    const auto live = scratch->liveReads.find(path);
    if (live != scratch->liveReads.end() &&
        live->second < record->pathReads.size()) {
        const RigExecWirePathValue &value =
            record->pathReads[live->second].value;
        if (value.tag == RigExecWirePathValue::Tag::Vec3fArray) {
            return value.vec3s.size();
        }
        return 0;
    }
    const auto chain = scratch->chainByTarget.find(path);
    if (chain == scratch->chainByTarget.end() ||
        chain->second >= record->chainBases.size() ||
        chain->second >= record->chainHaveBase.size() ||
        !record->chainHaveBase[chain->second]) {
        return 0;
    }
    return record->chainBases[chain->second].size();
}

// Appends one attribute's ints to a curvenet gather. No chain layer:
// only the recorded stage reads carry these arrays.
void
_RrGatherInts(const RrProgram *program, RrWeightScratch *scratch,
              const RigExecWireFrameInputs *record, uint32_t path,
              std::vector<int> *out)
{
    const auto live = scratch->liveReads.find(path);
    if (live == scratch->liveReads.end() ||
        live->second >= record->pathReads.size()) {
        return;
    }
    const RigExecWirePathValue &value =
        record->pathReads[live->second].value;
    if (value.tag != RigExecWirePathValue::Tag::IntArray) {
        return;
    }
    for (int32_t v : value.ints) {
        out->push_back(int(v));
    }
}

// Reads one attribute's array wholesale, for the two arrays a curvenet
// weight reads off its own prim. A failed read leaves the baked empty
// default: the baked `array` step assigns only on success.
void
_RrReadFloatArray(const RrProgram *program, RrWeightScratch *scratch,
                  const RigExecWireFrameInputs *record, uint32_t path,
                  std::vector<float> *out)
{
    out->clear();
    const auto live = scratch->liveReads.find(path);
    if (live == scratch->liveReads.end() ||
        live->second >= record->pathReads.size()) {
        return;
    }
    const RigExecWirePathValue &value =
        record->pathReads[live->second].value;
    if (value.tag != RigExecWirePathValue::Tag::FloatArray) {
        return;
    }
    out->assign(value.floats.begin(), value.floats.end());
}

void
_RrReadIntArray(const RrProgram *program, RrWeightScratch *scratch,
                const RigExecWireFrameInputs *record, uint32_t path,
                std::vector<int> *out)
{
    out->clear();
    const auto live = scratch->liveReads.find(path);
    if (live == scratch->liveReads.end() ||
        live->second >= record->pathReads.size()) {
        return;
    }
    const RigExecWirePathValue &value =
        record->pathReads[live->second].value;
    if (value.tag != RigExecWirePathValue::Tag::IntArray) {
        return;
    }
    for (int32_t v : value.ints) {
        out->push_back(int(v));
    }
}

void
_RrGatherPointList(const RrProgram *program, RrWeightScratch *scratch,
                   const RigExecWireFrameInputs *record,
                   const std::vector<uint32_t> &paths,
                   const std::vector<uint8_t> &valid,
                   std::vector<RrVec3f> *out)
{
    for (size_t i = 0; i < paths.size(); ++i) {
        if (i < valid.size() && !valid[i]) {
            continue;
        }
        _RrGatherPoints(program, scratch, record, paths[i], out);
    }
}

void
_RrGatherIntList(const RrProgram *program, RrWeightScratch *scratch,
                 const RigExecWireFrameInputs *record,
                 const std::vector<uint32_t> &paths,
                 const std::vector<uint8_t> &valid,
                 std::vector<int> *out)
{
    for (size_t i = 0; i < paths.size(); ++i) {
        if (i < valid.size() && !valid[i]) {
            continue;
        }
        _RrGatherInts(program, scratch, record, paths[i], out);
    }
}

// ---------------------------------------------------------------------------
// Packet envelope (RigExecWeightPacket::Resolve, types.cpp).
// ---------------------------------------------------------------------------

enum _RrRepresentation {
    _RrRepConstant,
    _RrRepDense,
    _RrRepSparse,
    _RrRepOther,
};

_RrRepresentation
_RrClassifyRepresentation(const RrProgram *program, uint32_t id)
{
    if (program->TokenEquals(id, "dense")) {
        return _RrRepDense;
    }
    if (program->TokenEquals(id, "sparse")) {
        return _RrRepSparse;
    }
    if (program->TokenEquals(id, "constant")) {
        return _RrRepConstant;
    }
    return _RrRepOther;
}

// Effective weight for logical element i of a count-element target, or
// -1 for a cardinality mismatch. An unknown representation answers as
// constant, which is the baked else arm.
float
_RrResolvePacket(const RrProgram *program, const RrWeightPacket &packet,
                 size_t i, size_t count)
{
    switch (_RrClassifyRepresentation(program, packet.representation)) {
    case _RrRepDense:
        if (packet.values.size() != count || i >= count) {
            return -1.0f;
        }
        return packet.values[i];
    case _RrRepSparse: {
        if (packet.indices.size() != packet.values.size()) {
            return -1.0f;
        }
        const auto it = std::lower_bound(
            packet.indices.begin(), packet.indices.end(), int32_t(i));
        if (it != packet.indices.end() && *it == int32_t(i)) {
            return packet.values[size_t(it - packet.indices.begin())];
        }
        return packet.defaultWeight;
    }
    case _RrRepConstant:
    case _RrRepOther:
        break;
    }
    return packet.defaultWeight;
}

bool
_RrApplyRangePolicy(bool clamp, float *w)
{
    if (!std::isfinite(*w)) {
        return false;
    }
    if (*w < 0.0f || *w > 1.0f) {
        if (clamp) {
            *w = std::min(std::max(*w, 0.0f), 1.0f);
            return true;
        }
        return false;
    }
    return true;
}

// ---------------------------------------------------------------------------
// Volumetric field kernels (rigExecMath/weightFields.cpp).
// ---------------------------------------------------------------------------

float
_RrClamp01(float x)
{
    return std::min(1.0f, std::max(0.0f, x));
}

float
_RrSampleFalloffLut(const std::vector<float> &curve, float r)
{
    if (curve.size() < 2) {
        return r;
    }
    const float x = _RrClamp01(r) * float(curve.size() - 1);
    const size_t i = std::min(curve.size() - 2, size_t(std::floor(x)));
    const float t = x - float(i);
    return curve[i] + (curve[i + 1] - curve[i]) * t;
}

float
_RrEvaluateFalloff(float distance, float falloffMin, float falloffMax,
                   float invert, float strength,
                   const std::vector<float> &curve)
{
    const float span = falloffMax - falloffMin;
    float u;
    if (std::abs(span) <= std::numeric_limits<float>::min()) {
        u = distance < falloffMin ? 0.0f : 1.0f;
    } else {
        u = _RrClamp01((distance - falloffMin) / span);
    }
    u = u + (1.0f - 2.0f * u) * invert;
    const float w = _RrSampleFalloffLut(curve, 1.0f - u);
    return w * strength;
}

RrVec3f
_RrToLocal(const RrMat4d &worldToLocal, const RrVec3f &p)
{
    const RrVec3d wide{double(p[0]), double(p[1]), double(p[2])};
    const RrVec3d local = worldToLocal.TransformAffine(wide);
    return RrVec3f(float(local[0]), float(local[1]), float(local[2]));
}

float
_RrSegmentDistance(const RrVec3f &p, const RrVec3f &a, const RrVec3f &b)
{
    const RrVec3f ab = b - a;
    const float lengthSq = ab.GetLengthSq();
    if (lengthSq <= 1e-20f) {
        return (p - a).GetLength();
    }
    const float t = _RrClamp01(RrDot(p - a, ab) / lengthSq);
    return (p - (a + ab * t)).GetLength();
}

float
_RrCurveDistance(const RrVec3f &p, const RrVec3f *curvePoints,
                 size_t count)
{
    if (count == 0 || !curvePoints) {
        return std::numeric_limits<float>::infinity();
    }
    if (count == 1) {
        return (p - curvePoints[0]).GetLength();
    }
    float best = std::numeric_limits<float>::infinity();
    for (size_t i = 0; i + 1 < count; ++i) {
        best = std::min(
            best,
            _RrSegmentDistance(p, curvePoints[i], curvePoints[i + 1]));
    }
    return best;
}

bool
_RrPlaneWithinBounds(const RrVec3f &p, int axis, float extentU,
                     float extentV)
{
    if (axis < 0 || axis > 2) {
        return false;
    }
    const float u = p[size_t((axis + 1) % 3)];
    const float v = p[size_t((axis + 2) % 3)];
    return std::abs(u) <= extentU && std::abs(v) <= extentV;
}

void
_RrSphereWeightField(const std::vector<RrVec3f> &points,
                     const RrMat4d &worldToLocal, float falloffMin,
                     float falloffMax, float invert, float strength,
                     const std::vector<float> &curve,
                     std::vector<float> *weights)
{
    weights->resize(points.size());
    for (size_t i = 0; i < points.size(); ++i) {
        const float d = _RrToLocal(worldToLocal, points[i]).GetLength();
        (*weights)[i] = _RrEvaluateFalloff(d, falloffMin, falloffMax,
                                           invert, strength, curve);
    }
}

void
_RrPlaneWeightField(const std::vector<RrVec3f> &points,
                    const RrMat4d &worldToLocal, int axis,
                    float falloffMin, float falloffMax, float invert,
                    float strength, const std::vector<float> &curve,
                    std::vector<float> *weights, bool bounded,
                    float extentU, float extentV)
{
    weights->resize(points.size());
    for (size_t i = 0; i < points.size(); ++i) {
        const RrVec3f local = _RrToLocal(worldToLocal, points[i]);
        if (bounded && !_RrPlaneWithinBounds(local, axis, extentU,
                                             extentV)) {
            (*weights)[i] = 0.0f;
            continue;
        }
        float d = 0.0f;
        if (axis >= 0 && axis <= 2) {
            d = local[size_t(axis)];
        }
        (*weights)[i] = _RrEvaluateFalloff(d, falloffMin, falloffMax,
                                           invert, strength, curve);
    }
}

void
_RrCurveWeightField(const std::vector<RrVec3f> &points,
                    const std::vector<RrVec3f> &curvePoints,
                    const RrMat4d &worldToLocal, float falloffMin,
                    float falloffMax, float invert, float strength,
                    const std::vector<float> &curve,
                    std::vector<float> *weights)
{
    weights->resize(points.size());
    if (curvePoints.empty()) {
        std::fill(weights->begin(), weights->end(), 0.0f);
        return;
    }
    std::vector<RrVec3f> localCurve(curvePoints.size());
    for (size_t k = 0; k < curvePoints.size(); ++k) {
        localCurve[k] = _RrToLocal(worldToLocal, curvePoints[k]);
    }
    for (size_t i = 0; i < points.size(); ++i) {
        const float d = _RrCurveDistance(
            _RrToLocal(worldToLocal, points[i]), localCurve.data(),
            localCurve.size());
        (*weights)[i] = _RrEvaluateFalloff(d, falloffMin, falloffMax,
                                           invert, strength, curve);
    }
}

// How one weight field folds into the accumulated result, in
// RigExecWeightCombine order.
enum _RrCombineMode {
    _RrCombineMultiply,
    _RrCombineAdd,
    _RrCombineSubtract,
    _RrCombineMax,
    _RrCombineMin,
    _RrCombineAverage,
    _RrCombineOverlay,
};

float
_RrCombineIdentity(_RrCombineMode mode)
{
    switch (mode) {
    case _RrCombineMultiply:
    case _RrCombineMin:
        return 1.0f;
    case _RrCombineAdd:
    case _RrCombineSubtract:
    case _RrCombineMax:
    case _RrCombineAverage:
    case _RrCombineOverlay:
        break;
    }
    return 0.0f;
}

float
_RrFoldWeight(_RrCombineMode mode, float acc, float value)
{
    switch (mode) {
    case _RrCombineMultiply:
        return acc * value;
    case _RrCombineAdd:
        return acc + value;
    case _RrCombineSubtract:
        return acc - value;
    case _RrCombineMax:
        return std::max(acc, value);
    case _RrCombineMin:
        return std::min(acc, value);
    case _RrCombineAverage:
        return acc + value;
    case _RrCombineOverlay:
        return acc < 0.5f ? 2.0f * acc * value
                          : 1.0f - 2.0f * (1.0f - acc) * (1.0f - value);
    }
    return acc;
}

bool
_RrCombineWeightFields(
    _RrCombineMode mode,
    const std::vector<std::vector<float>> &inputs,
    size_t elementCount,
    std::vector<float> *out)
{
    for (const std::vector<float> &field : inputs) {
        if (field.size() != elementCount) {
            out->clear();
            return false;
        }
    }
    if (inputs.empty()) {
        out->assign(elementCount, _RrCombineIdentity(mode));
        return true;
    }
    const bool seedFromFirst = mode == _RrCombineSubtract ||
                               mode == _RrCombineOverlay;
    size_t first = 0;
    if (seedFromFirst) {
        *out = inputs[0];
        first = 1;
    } else {
        out->assign(elementCount, _RrCombineIdentity(mode));
    }
    for (size_t k = first; k < inputs.size(); ++k) {
        for (size_t i = 0; i < elementCount; ++i) {
            (*out)[i] = _RrFoldWeight(mode, (*out)[i], inputs[k][i]);
        }
    }
    if (mode == _RrCombineAverage) {
        const float inv = 1.0f / float(inputs.size());
        for (float &w : *out) {
            w *= inv;
        }
    }
    return true;
}

// ---------------------------------------------------------------------------
// Rigid placement (RigExecRigidWorldToLocal, weightPackets.cpp, over
// GfMatrix4d::RemoveScaleShear from pxr/base/gf/matrix4d.cpp).
// ---------------------------------------------------------------------------

// Jacobi eigen decomposition of the symmetric 3x3 in _Jacobi3's upper
// triangle, in place. A line port of GfMatrix4d::_Jacobi3.
void
_RrJacobi3(RrMat4d *self, RrVec3d *eigenvalues,
           RrVec3d eigenvectors[3])
{
    (*eigenvalues)[0] = (*self)[0][0];
    (*eigenvalues)[1] = (*self)[1][1];
    (*eigenvalues)[2] = (*self)[2][2];
    eigenvectors[0] = RrVec3d::XAxis();
    eigenvectors[1] = RrVec3d::YAxis();
    eigenvectors[2] = RrVec3d::ZAxis();

    RrMat4d a = *self;
    RrVec3d b = *eigenvalues;
    RrVec3d z(0.0);

    for (int i = 0; i < 50; i++) {
        double sm = 0.0;
        for (int p = 0; p < 2; p++) {
            for (int q = p + 1; q < 3; q++) {
                sm += std::abs(a[p][q]);
            }
        }

        if (sm == 0.0) {
            return;
        }

        const double thresh = (i < 3 ? (.2 * sm / (3 * 3)) : 0.0);

        for (int p = 0; p < 3; p++) {
            for (int q = p + 1; q < 3; q++) {
                double g = 100.0 * std::abs(a[p][q]);

                if (i > 3 &&
                    (std::abs((*eigenvalues)[p]) + g ==
                     std::abs((*eigenvalues)[p])) &&
                    (std::abs((*eigenvalues)[q]) + g ==
                     std::abs((*eigenvalues)[q]))) {
                    a[p][q] = 0.0;
                } else if (std::abs(a[p][q]) > thresh) {
                    double h = (*eigenvalues)[q] - (*eigenvalues)[p];
                    double t;

                    if (std::abs(h) + g == std::abs(h)) {
                        t = a[p][q] / h;
                    } else {
                        const double theta =
                            0.5 * h / a[p][q];
                        t = 1.0 /
                            (std::abs(theta) +
                             std::sqrt(1.0 + theta * theta));
                        if (theta < 0.0) {
                            t = -t;
                        }
                    }

                    const double c = 1.0 / std::sqrt(1.0 + t * t);
                    const double s = t * c;
                    const double tau = s / (1.0 + c);
                    h = t * a[p][q];
                    z[p] -= h;
                    z[q] += h;
                    (*eigenvalues)[p] -= h;
                    (*eigenvalues)[q] += h;
                    a[p][q] = 0.0;

                    for (int j = 0; j < p; j++) {
                        g = a[j][p];
                        h = a[j][q];
                        a[j][p] = g - s * (h + g * tau);
                        a[j][q] = h + s * (g - h * tau);
                    }

                    for (int j = p + 1; j < q; j++) {
                        g = a[p][j];
                        h = a[j][q];
                        a[p][j] = g - s * (h + g * tau);
                        a[j][q] = h + s * (g - h * tau);
                    }

                    for (int j = q + 1; j < 3; j++) {
                        g = a[p][j];
                        h = a[q][j];
                        a[p][j] = g - s * (h + g * tau);
                        a[q][j] = h + s * (g - h * tau);
                    }

                    for (int j = 0; j < 3; j++) {
                        g = eigenvectors[j][p];
                        h = eigenvectors[j][q];
                        eigenvectors[j][p] = g - s * (h + g * tau);
                        eigenvectors[j][q] = h + s * (g - h * tau);
                    }
                }
            }
        }
        for (int p = 0; p < 3; p++) {
            (*eigenvalues)[p] = b[p] += z[p];
            z[p] = 0;
        }
    }
}

// Factors the matrix into rotation, scale, rotation, translation and
// perspective (M = r * s * -r * u * t). A line port of
// GfMatrix4d::Factor with its default eps.
bool
_RrFactorMatrix(const RrMat4d &self, RrMat4d *r, RrVec3d *s,
                RrMat4d *u, RrVec3d *t, RrMat4d *p)
{
    const double eps = 1e-10;

    p->SetDiagonal(1);

    RrMat4d a;
    for (int i = 0; i < 3; i++) {
        for (int j = 0; j < 3; j++) {
            a[i][j] = self[i][j];
        }
        a[3][i] = a[i][3] = 0.0;
        (*t)[i] = self[3][i];
    }
    a[3][3] = 1.0;

    const double det = a.GetDeterminant3();
    const double detSign = (det < 0.0 ? -1.0 : 1.0);
    const bool isSingular = det * detSign < eps;

    const RrMat4d b = a * a.GetTranspose();
    RrVec3d eigenvalues;
    RrVec3d eigenvectors[3];
    RrMat4d mutableB = b;
    _RrJacobi3(&mutableB, &eigenvalues, eigenvectors);
    r->Set(eigenvectors[0][0], eigenvectors[0][1], eigenvectors[0][2],
           0.0, eigenvectors[1][0], eigenvectors[1][1],
           eigenvectors[1][2], 0.0, eigenvectors[2][0],
           eigenvectors[2][1], eigenvectors[2][2], 0.0, 0.0, 0.0, 0.0,
           1.0);

    RrMat4d sInv;
    sInv.SetIdentity();
    for (int i = 0; i < 3; i++) {
        if (eigenvalues[i] < eps) {
            (*s)[i] = detSign * eps;
        } else {
            (*s)[i] = detSign * std::sqrt(eigenvalues[i]);
        }
        sInv[i][i] = 1.0 / (*s)[i];
    }

    *u = *r * sInv * r->GetTranspose() * a;

    return !isSingular;
}

RrMat4d
_RrRemoveScaleShear(const RrMat4d &self)
{
    RrMat4d scaleOrientMat, factoredRotMat, perspMat;
    RrVec3d scale, translation;
    if (!_RrFactorMatrix(self, &scaleOrientMat, &scale, &factoredRotMat,
                         &translation, &perspMat)) {
        return self;
    }

    factoredRotMat.Orthonormalize();
    RrMat4d translate(1.0);
    translate.SetTranslateOnly(translation);
    return factoredRotMat * translate;
}

bool
_RrRigidWorldToLocal(const RrPointFrame &posed, RrMat4d *result)
{
    if (!posed.IsValid() || posed.IsDegenerate()) {
        return false;
    }
    RrMat4d placement(1.0);
    if (!RrPointsToMatrix(RrIdentityLandmarks(), posed.points,
                          &placement)) {
        return false;
    }
    for (int i = 0; i < 4; ++i) {
        for (int j = 0; j < 4; ++j) {
            if (!std::isfinite(placement[size_t(i)][size_t(j)])) {
                return false;
            }
        }
    }
    const RrMat4d rigid = _RrRemoveScaleShear(placement);
    const double det = rigid.GetDeterminant();
    if (!std::isfinite(det) || std::abs(det) < 1e-12) {
        return false;
    }
    *result = rigid.GetInverse();
    return true;
}

// The volume's structural prologue: representation, range policy,
// placement and (for sphere and curve) the per-axis divisors. Reads
// nothing it costs anything to gather, so the step asks before copying
// a whole mesh -- exactly RigExecVolumeWeightCanBuild's contract.
bool
_RrVolumeCanBuild(const RrProgram *program,
                  const RigExecWireWeightObject &wire, bool usesScales,
                  const RrPointFrame *placement, bool hasPlacement,
                  const RrVec3f &scales, RrMat4d *worldToLocal)
{
    if (!program->TokenEquals(wire.representation, "dense")) {
        return false;
    }
    if (!program->TokenEquals(wire.rangePolicy, "strict") &&
        !program->TokenEquals(wire.rangePolicy, "clamp")) {
        return false;
    }
    if (!hasPlacement ||
        !_RrRigidWorldToLocal(*placement, worldToLocal)) {
        return false;
    }
    if (usesScales) {
        for (int a = 0; a < 3; ++a) {
            if (!std::isfinite(scales[size_t(a)]) ||
                scales[size_t(a)] <= 0.0f) {
                return false;
            }
        }
    }
    return true;
}

bool
_RrBeginVolumeWeight(const RrProgram *program,
                     const RigExecWireWeightObject &wire, bool usesScales,
                     const RrPointFrame *placement, bool hasPlacement,
                     const RrVec3f &scales,
                     const std::vector<RrVec3f> &targetPoints,
                     const std::vector<RrVec3f> &samplePoints,
                     RrWeightPacket *packet, RrMat4d *worldToLocal,
                     const std::vector<RrVec3f> **points)
{
    packet->representation = wire.representation;
    packet->rangePolicy = wire.rangePolicy;
    if (!_RrVolumeCanBuild(program, wire, usesScales, placement,
                           hasPlacement, scales, worldToLocal)) {
        return false;
    }
    if (targetPoints.empty()) {
        return false;
    }
    if (!samplePoints.empty() &&
        samplePoints.size() != targetPoints.size()) {
        return false;
    }
    *points = samplePoints.empty() ? &targetPoints : &samplePoints;
    return true;
}

// The shared epilogue: range policy over the generated field. A
// violation returns a BARE packet, which is the baked shape for this
// failure rather than the token-carrying one above.
bool
_RrFinishVolumeWeight(const RrProgram *program,
                      const RrWeightPacket *packet,
                      std::vector<float> *weights, RrWeightPacket *out)
{
    const bool clamp =
        program->TokenEquals(packet->rangePolicy, "clamp");
    for (float &w : *weights) {
        if (!_RrApplyRangePolicy(clamp, &w)) {
            *out = RrWeightPacket();
            return false;
        }
    }
    out->representation = packet->representation;
    out->rangePolicy = packet->rangePolicy;
    out->values = std::move(*weights);
    out->defaultWeight = 0.0f;
    out->valid = true;
    return true;
}

RrMat4d
_RrApplyAxisScales(const RrMat4d &worldToLocal, const RrVec3f &scales)
{
    RrMat4d divide(1.0);
    divide.Set(1.0 / double(scales[0]), 0.0, 0.0, 0.0, 0.0,
               1.0 / double(scales[1]), 0.0, 0.0, 0.0, 0.0,
               1.0 / double(scales[2]), 0.0, 0.0, 0.0, 0.0, 1.0);
    return worldToLocal * divide;
}

RrWeightPacket
_RrBuildStaticPacket(const RrProgram *program,
                     const RigExecWireWeightObject &wire,
                     float defaultWeight)
{
    RrWeightPacket packet;
    packet.representation = wire.representation;
    packet.rangePolicy = wire.rangePolicy;
    if (_RrClassifyRepresentation(program, packet.representation) ==
        _RrRepOther) {
        return packet;
    }
    const bool clamp =
        program->TokenEquals(wire.rangePolicy, "clamp");
    if (!clamp && !program->TokenEquals(wire.rangePolicy, "strict")) {
        return packet;
    }
    packet.defaultWeight = defaultWeight;

    if (_RrClassifyRepresentation(program, packet.representation) ==
        _RrRepSparse) {
        const size_t paired =
            std::min(wire.values.size(), wire.indices.size());
        std::vector<std::pair<int32_t, float>> pairs;
        for (size_t i = 0; i < paired; ++i) {
            pairs.emplace_back(wire.indices[i], wire.values[i]);
        }
        if (wire.values.size() != wire.indices.size()) {
            return packet;
        }
        std::sort(pairs.begin(), pairs.end());
        for (size_t i = 0; i < pairs.size(); ++i) {
            if (i > 0 && pairs[i].first == pairs[i - 1].first) {
                return packet;
            }
            packet.indices.push_back(pairs[i].first);
            packet.values.push_back(pairs[i].second);
        }
    } else {
        packet.values = wire.values;
    }
    if (_RrClassifyRepresentation(program, packet.representation) ==
            _RrRepDense &&
        packet.defaultWeight != 0.0f) {
        return packet;
    }
    for (float &w : packet.values) {
        if (!_RrApplyRangePolicy(clamp, &w)) {
            return packet;
        }
    }
    if (!_RrApplyRangePolicy(clamp, &packet.defaultWeight)) {
        return packet;
    }
    packet.valid = true;
    return packet;
}

RrWeightPacket
_RrBuildDynamicPacket(const RrProgram *program,
                      const RigExecWireWeightObject &wire, float driver,
                      float scale, float bias,
                      const RrStore &store)
{
    RrWeightPacket packet;
    packet.representation = wire.representation;
    packet.rangePolicy = wire.rangePolicy;
    if (_RrClassifyRepresentation(program, packet.representation) ==
        _RrRepOther) {
        return packet;
    }
    const bool clamp =
        program->TokenEquals(wire.rangePolicy, "clamp");
    if (!clamp && !program->TokenEquals(wire.rangePolicy, "strict")) {
        return packet;
    }

    const float d = driver;
    const float s = scale;
    const float a = bias;

    if (wire.base < 0) {
        if (!program->TokenEquals(packet.representation, "constant")) {
            return packet;
        }
        packet.defaultWeight = (1.0f * d) * s + a;
        if (!_RrApplyRangePolicy(clamp, &packet.defaultWeight)) {
            return packet;
        }
        packet.valid = true;
        return packet;
    }
    if (size_t(wire.base) >= store.weightPackets.size()) {
        return packet;
    }
    const RrWeightPacket &base = store.weightPackets[size_t(wire.base)];
    // String-table ids intern equal strings, so id inequality is token
    // inequality -- the same comparison the baked TfToken one is.
    if (!base.valid || base.representation != packet.representation) {
        return packet;
    }
    packet.indices = base.indices;
    packet.values.reserve(base.values.size());
    for (float b : base.values) {
        float r = (b * d) * s + a;
        if (!_RrApplyRangePolicy(clamp, &r)) {
            return packet;
        }
        packet.values.push_back(r);
    }
    if (program->TokenEquals(packet.representation, "dense")) {
        packet.defaultWeight = 0.0f;
    } else {
        packet.defaultWeight = (base.defaultWeight * d) * s + a;
        if (!_RrApplyRangePolicy(clamp, &packet.defaultWeight)) {
            return packet;
        }
    }
    packet.valid = true;
    return packet;
}

RrWeightPacket
_RrBuildCombinePacket(const RrProgram *program,
                      const RigExecWireWeightObject &wire,
                      const RrStore &store, size_t targetCount,
                      float strength, float invert)
{
    RrWeightPacket packet;
    packet.representation = wire.representation;
    packet.rangePolicy = wire.rangePolicy;
    if (!program->TokenEquals(packet.representation, "dense")) {
        return packet;
    }
    const bool clamp =
        program->TokenEquals(wire.rangePolicy, "clamp");
    if (!clamp && !program->TokenEquals(wire.rangePolicy, "strict")) {
        return packet;
    }

    _RrCombineMode mode = _RrCombineMultiply;
    if (program->TokenEquals(wire.combineMode, "multiply")) {
        mode = _RrCombineMultiply;
    } else if (program->TokenEquals(wire.combineMode, "add")) {
        mode = _RrCombineAdd;
    } else if (program->TokenEquals(wire.combineMode, "subtract")) {
        mode = _RrCombineSubtract;
    } else if (program->TokenEquals(wire.combineMode, "max")) {
        mode = _RrCombineMax;
    } else if (program->TokenEquals(wire.combineMode, "min")) {
        mode = _RrCombineMin;
    } else if (program->TokenEquals(wire.combineMode, "average")) {
        mode = _RrCombineAverage;
    } else if (program->TokenEquals(wire.combineMode, "overlay")) {
        mode = _RrCombineOverlay;
    } else {
        return packet;
    }

    for (int32_t input : wire.inputs) {
        if (input < 0 ||
            size_t(input) >= store.weightPackets.size()) {
            return packet;
        }
        const RrWeightPacket &in = store.weightPackets[size_t(input)];
        if (!in.valid) {
            return packet;
        }
    }
    size_t elementCount = 0;
    for (int32_t input : wire.inputs) {
        const RrWeightPacket &in = store.weightPackets[size_t(input)];
        if (program->TokenEquals(in.representation, "dense")) {
            if (elementCount && in.values.size() != elementCount) {
                return packet;
            }
            elementCount = in.values.size();
        }
    }
    if (!elementCount) {
        elementCount = targetCount;
    }
    if (!elementCount) {
        return packet;
    }

    std::vector<std::vector<float>> fields;
    fields.reserve(wire.inputs.size());
    for (int32_t input : wire.inputs) {
        const RrWeightPacket &in = store.weightPackets[size_t(input)];
        std::vector<float> field(elementCount);
        for (size_t i = 0; i < elementCount; ++i) {
            field[i] = _RrResolvePacket(program, in, i, elementCount);
            if (field[i] < 0.0f) {
                return packet;
            }
        }
        fields.push_back(std::move(field));
    }

    std::vector<float> folded;
    if (!_RrCombineWeightFields(mode, fields, elementCount, &folded)) {
        return packet;
    }
    for (float &w : folded) {
        w = (w + (1.0f - 2.0f * w) * invert) * strength;
        if (!_RrApplyRangePolicy(clamp, &w)) {
            return RrWeightPacket();
        }
    }
    packet.values = std::move(folded);
    packet.defaultWeight = 0.0f;
    packet.valid = true;
    return packet;
}

RrWeightPacket
_RrBuildSpherePacket(const RrProgram *program,
                     const RigExecWireWeightObject &wire,
                     const RrPointFrame *placement, bool hasPlacement,
                     float falloffMin, float falloffMax, float invert,
                     float strength, const RrVec3f &scales,
                     const std::vector<RrVec3f> &targetPoints,
                     const std::vector<RrVec3f> &samplePoints)
{
    RrWeightPacket packet;
    RrMat4d worldToLocal;
    const std::vector<RrVec3f> *points = nullptr;
    if (!_RrBeginVolumeWeight(program, wire, /* usesScales = */ true,
                              placement, hasPlacement, scales,
                              targetPoints, samplePoints, &packet,
                              &worldToLocal, &points)) {
        return packet;
    }
    std::vector<float> weights;
    _RrSphereWeightField(*points,
                         _RrApplyAxisScales(worldToLocal, scales),
                         falloffMin, falloffMax, invert, strength,
                         wire.falloffCurve, &weights);
    RrWeightPacket out;
    if (!_RrFinishVolumeWeight(program, &packet, &weights, &out)) {
        return out;
    }
    return out;
}

RrWeightPacket
_RrBuildPlanePacket(const RrProgram *program,
                    const RigExecWireWeightObject &wire,
                    const RrPointFrame *placement, bool hasPlacement,
                    float falloffMin, float falloffMax, float invert,
                    float strength,
                    const std::vector<RrVec3f> &targetPoints,
                    const std::vector<RrVec3f> &samplePoints,
                    bool bounded, float extentU, float extentV)
{
    RrWeightPacket packet;
    RrMat4d worldToLocal;
    const std::vector<RrVec3f> *points = nullptr;
    if (!_RrBeginVolumeWeight(program, wire, /* usesScales = */ false,
                              placement, hasPlacement, RrVec3f(1.0f),
                              targetPoints, samplePoints, &packet,
                              &worldToLocal, &points)) {
        return packet;
    }
    int axisIndex = 1;
    if (program->TokenEquals(wire.planeAxis, "x")) {
        axisIndex = 0;
    } else if (program->TokenEquals(wire.planeAxis, "y")) {
        axisIndex = 1;
    } else if (program->TokenEquals(wire.planeAxis, "z")) {
        axisIndex = 2;
    } else {
        return packet;
    }

    if (!bounded) {
        if (!program->TokenEquals(wire.planeBounds, "unbounded")) {
            return packet;
        }
    } else if (!std::isfinite(extentU) || extentU <= 0.0f ||
               !std::isfinite(extentV) || extentV <= 0.0f) {
        return packet;
    }

    std::vector<float> weights;
    _RrPlaneWeightField(*points, worldToLocal, axisIndex, falloffMin,
                        falloffMax, invert, strength, wire.falloffCurve,
                        &weights, bounded, extentU, extentV);
    RrWeightPacket out;
    if (!_RrFinishVolumeWeight(program, &packet, &weights, &out)) {
        return out;
    }
    return out;
}

RrWeightPacket
_RrBuildCurvePacket(const RrProgram *program,
                    const RigExecWireWeightObject &wire,
                    const RrPointFrame *placement, bool hasPlacement,
                    float falloffMin, float falloffMax, float invert,
                    float strength, const RrVec3f &scales,
                    const std::vector<RrVec3f> &targetPoints,
                    const std::vector<RrVec3f> &samplePoints,
                    const std::vector<RrVec3f> &curvePoints)
{
    RrWeightPacket packet;
    RrMat4d worldToLocal;
    const std::vector<RrVec3f> *points = nullptr;
    if (!_RrBeginVolumeWeight(program, wire, /* usesScales = */ true,
                              placement, hasPlacement, scales,
                              targetPoints, samplePoints, &packet,
                              &worldToLocal, &points)) {
        return packet;
    }
    if (curvePoints.empty()) {
        return packet;
    }
    std::vector<float> weights;
    _RrCurveWeightField(*points, curvePoints,
                        _RrApplyAxisScales(worldToLocal, scales),
                        falloffMin, falloffMax, invert, strength,
                        wire.falloffCurve, &weights);
    RrWeightPacket out;
    if (!_RrFinishVolumeWeight(program, &packet, &weights, &out)) {
        return out;
    }
    return out;
}

// One WeightPacket step: rebuilds the object's packet from its bound
// inputs and its (already built) composed inputs. No packet is carried
// over: the step assigns unconditionally, exactly as the baked one
// does, whatever the cone skipped.
bool
_RrRunWeightPacket(RrProgram *program, size_t step,
                   const RigExecWireFrameInputs *record,
                   std::string *error)
{
    RrStore &store = program->store;
    const RigExecWireStep &wireStep = (*program->steps)[step];
    const int32_t object = wireStep.object;
    if (object < 0 ||
        size_t(object) >= program->geometry->weightObjects.size() ||
        size_t(object) >= store.weightPackets.size()) {
        if (error) {
            *error = "weight step " + _RrStepLabel(program, step) +
                     " names no weight object";
        }
        return false;
    }
    const size_t index = size_t(object);
    const RigExecWireWeightObject &wire =
        program->geometry->weightObjects[index];
    RrWeightScratch *scratch = _RrScratch(program);

    if (program->TokenEquals(wire.type, "RigExecStaticWeight")) {
        store.weightPackets[index] = _RrBuildStaticPacket(
            program, wire,
            _RrReadWeightFloat(program, index,
                               RrWeightDefaultWeight));
        return true;
    }
    if (program->TokenEquals(wire.type, "RigExecDynamicWeight")) {
        store.weightPackets[index] = _RrBuildDynamicPacket(
            program, wire,
            _RrReadWeightFloat(program, index, RrWeightDriver),
            _RrReadWeightFloat(program, index, RrWeightScale),
            _RrReadWeightFloat(program, index, RrWeightBias), store);
        return true;
    }
    if (program->TokenEquals(wire.type, "RigExecCombineWeight")) {
        size_t targetCount = 0;
        for (size_t i = 0; i < wire.combineTargetPoints.size(); ++i) {
            if (i < wire.combineTargetValid.size() &&
                !wire.combineTargetValid[i]) {
                continue;
            }
            targetCount += _RrGatherPointCount(
                program, scratch, record, wire.combineTargetPoints[i]);
        }
        store.weightPackets[index] = _RrBuildCombinePacket(
            program, wire, store, targetCount,
            _RrReadWeightFloat(program, index, RrWeightStrength),
            _RrReadWeightFloat(program, index, RrWeightInvert));
        return true;
    }
    const bool isSphere =
        program->TokenEquals(wire.type, "RigExecSphereWeight");
    const bool isPlane =
        program->TokenEquals(wire.type, "RigExecPlaneWeight");
    const bool isCurve =
        program->TokenEquals(wire.type, "RigExecCurveWeight");
    if (isSphere || isPlane || isCurve) {
        // The volume's BASE frame: the last base version of the
        // provider slot, which is what the authoritative snapshot
        // carries for every seeded provider.
        const RrPointFrame *placement = nullptr;
        bool hasPlacement = false;
        if (wire.providerSlot >= 0) {
            const size_t slot = size_t(wire.providerSlot);
            if (slot >= store.baseLast.size() ||
                size_t(store.baseLast[slot]) >= store.base.size()) {
                if (error) {
                    *error = "weight step " +
                             _RrStepLabel(program, step) +
                             " is placed by no provider slot";
                }
                return false;
            }
            placement = &store.base[size_t(store.baseLast[slot])];
            hasPlacement = true;
        }
        const float falloffMin = _RrReadWeightFloat(
            program, index, RrWeightFalloffMin);
        const float falloffMax = _RrReadWeightFloat(
            program, index, RrWeightFalloffMax);
        const float invert = _RrReadWeightFloat(
            program, index, RrWeightInvert);
        const float strength = _RrReadWeightFloat(
            program, index, RrWeightStrength);
        RrVec3f scales(1.0f);
        bool bounded = false;
        float extentU = 1.0f, extentV = 1.0f;
        if (!isPlane) {
            scales = RrVec3f(
                _RrReadWeightFloat(program, index, RrWeightScaleX),
                _RrReadWeightFloat(program, index, RrWeightScaleY),
                _RrReadWeightFloat(program, index, RrWeightScaleZ));
        } else if (program->TokenEquals(wire.planeBounds, "bounded")) {
            bounded = true;
            extentU = _RrReadWeightFloat(program, index,
                                         RrWeightExtentU);
            extentV = _RrReadWeightFloat(program, index,
                                         RrWeightExtentV);
        }
        // The points are a whole mesh, gathered only once the
        // structural half has said the volume can produce a field.
        std::vector<RrVec3f> targetPoints, samplePoints, curvePoints;
        RrMat4d worldToLocal;
        if (_RrVolumeCanBuild(program, wire, !isPlane, placement,
                              hasPlacement, scales, &worldToLocal)) {
            _RrGatherPointList(program, scratch, record,
                               wire.targetPoints, wire.targetValid,
                               &targetPoints);
            _RrGatherPointList(program, scratch, record,
                               wire.samplePoints, wire.sampleValid,
                               &samplePoints);
            if (isCurve) {
                _RrGatherPointList(program, scratch, record,
                                   wire.curvePoints, wire.curveValid,
                                   &curvePoints);
            }
        }
        if (isSphere) {
            store.weightPackets[index] = _RrBuildSpherePacket(
                program, wire, placement, hasPlacement, falloffMin,
                falloffMax, invert, strength, scales, targetPoints,
                samplePoints);
        } else if (isPlane) {
            store.weightPackets[index] = _RrBuildPlanePacket(
                program, wire, placement, hasPlacement, falloffMin,
                falloffMax, invert, strength, targetPoints,
                samplePoints, bounded, extentU, extentV);
        } else {
            store.weightPackets[index] = _RrBuildCurvePacket(
                program, wire, placement, hasPlacement, falloffMin,
                falloffMax, invert, strength, scales, targetPoints,
                samplePoints, curvePoints);
        }
        return true;
    }
    if (program->TokenEquals(wire.type, "RigExecCurvenetWeight")) {
        std::vector<RrVec3f> mesh, net;
        std::vector<int> counts, indices, splines, autoSmooth;
        std::vector<float> weights;
        _RrGatherPointList(program, scratch, record,
                           wire.curvenetMeshPoints,
                           wire.curvenetMeshValid, &mesh);
        _RrGatherPointList(program, scratch, record,
                           wire.curvenetPoints,
                           wire.curvenetPointsValid, &net);
        _RrGatherIntList(program, scratch, record,
                         wire.curvenetCounts,
                         wire.curvenetCountsValid, &counts);
        _RrGatherIntList(program, scratch, record,
                         wire.curvenetIndices,
                         wire.curvenetIndicesValid, &indices);
        _RrGatherIntList(program, scratch, record,
                         wire.curvenetSplines,
                         wire.curvenetSplinesValid, &splines);
        (void)mesh;
        (void)net;
        (void)counts;
        (void)indices;
        (void)splines;
        if (wire.curvenetWeightsValid) {
            _RrReadFloatArray(program, scratch, record,
                              wire.curvenetWeights, &weights);
        }
        if (wire.curvenetAutoSmoothValid) {
            _RrReadIntArray(program, scratch, record,
                            wire.curvenetAutoSmooth, &autoSmooth);
        }
        (void)weights;
        (void)autoSmooth;
        _RrReadWeightInt(program, index, RrWeightCurvenetSamples);
        const bool basisOk =
            program->TokenEquals(wire.curvenetBasis, "bezier") ||
            program->TokenEquals(wire.curvenetBasis, "catmullRom");
        const bool policyOk =
            program->TokenEquals(wire.rangePolicy, "strict") ||
            program->TokenEquals(wire.rangePolicy, "clamp");
        if (!basisOk || !policyOk) {
            RrWeightPacket packet;
            packet.representation = wire.representation;
            packet.rangePolicy = wire.rangePolicy;
            store.weightPackets[index] = packet;
            return true;
        }
        // The bind itself (cutMesh + curvenet + sparseSolve) has no
        // zero-USD port: failing names the object rather than
        // publishing an invalid packet the baked path would not.
        if (error) {
            *error = "weight object " +
                     program->TextOrEmpty(wire.path) +
                     " needs the curvenet weight binding, which the "
                     "zero-USD runtime does not port";
        }
        return false;
    }
    // Every weight object type the epoch can hold has an arm above; an
    // unknown type is an invalid packet, which is a MoverFailed
    // pass-through rather than a plausible wrong deformation.
    store.weightPackets[index] = RrWeightPacket();
    return true;
}

// The one VolumePlacements step: where every volume weight ended up,
// over the frames the walk ended with. The evaluator's own routine,
// which this mirrors rather than calls: a frame no matrix can be
// built from leaves whatever the failed decomposition wrote (the
// identity, over the identity seed) rather than skipping the entry.
bool
_RrRunVolumePlacements(RrProgram *program, size_t step,
                       std::string *error)
{
    RrStore &store = program->store;
    const size_t slots = program->slotMeta->paths.size();
    if (store.finLast.size() < slots ||
        (slots > 0 && store.fin.empty())) {
        if (error) {
            *error = "weight step " + _RrStepLabel(program, step) +
                     " runs with no pose frames";
        }
        return false;
    }
    const std::vector<uint8_t> &noScale =
        program->constants->noScaleAvars;
    store.weightFrames.clear();
    for (size_t i = 0; i < slots; ++i) {
        if (i >= noScale.size() || !noScale[i]) {
            continue;
        }
        if (size_t(store.finLast[i]) >= store.fin.size()) {
            if (error) {
                *error = "weight step " +
                         _RrStepLabel(program, step) +
                         " reads a pose frame beyond the pools";
            }
            return false;
        }
        const RrPointFrame &frame =
            store.fin[size_t(store.finLast[i])];
        RrMat4d placement(1.0);
        if (RrFrameUsable(frame)) {
            RrPointsToMatrix(RrIdentityLandmarks(), frame.points,
                             &placement);
        }
        store.weightFrames[program->slotMeta->paths[i]] = placement;
    }
    return true;
}

}  // namespace

bool
RrRunWeightStep(RrProgram *program, size_t step, double time,
                std::string *error)
{
    if (!program || !program->steps || !program->geometry ||
        !program->inputs || !program->slotMeta || !program->constants ||
        !program->weights || step >= program->steps->size()) {
        if (error) {
            *error = "weight step names no step";
        }
        return false;
    }
    const RigExecWireStepKind kind = (*program->steps)[step].kind;
    if (kind == RigExecWireStepKind::VolumePlacements) {
        return _RrRunVolumePlacements(program, step, error);
    }
    if (kind != RigExecWireStepKind::WeightPacket) {
        if (error) {
            *error = "weight step " + _RrStepLabel(program, step) +
                     " is not a weight step";
        }
        return false;
    }
    const RigExecWireFrameInputs *record = _RrFindRecord(program, time);
    if (!record) {
        if (error) {
            char buffer[64];
            std::snprintf(buffer, sizeof(buffer),
                          "weight step %s names no baked frame",
                          _RrStepLabel(program, step).c_str());
            *error = buffer;
        }
        return false;
    }
    _RrRefreshReads(program, record);
    return _RrRunWeightPacket(program, step, record, error);
}

}  // namespace rigExec
