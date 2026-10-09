// Proving a cone: RIGEXEC_BAKED_VERIFY_CONES.
// Cone re-execution is an argument -- "nothing outside the closure could
// have moved, so last run's values are this run's" -- and an argument about
// floating-point state is worth exactly what a machine can check of it. So
// this file lets one frame run twice from one starting point: the cone run's
// whole answer is shadowed, the starting point is put back, every step runs,
// and the two answers are compared slot by slot, counter by counter and
// diagnostic by diagnostic.
// Nothing here is on a production path. It is deliberately a deep copy of
// everything a step can touch, because a shadow that left a field out would
// agree with the cone run about the one thing the cone got wrong. What it
// leaves out on purpose, and why, is listed beside RigExecBakedRunShadow.
#include "bakedProgramImpl.h"

#include "pxr/base/tf/getenv.h"

#include <cmath>
#include <cstring>
#include <cstdio>
#include <map>
#include <string>
#include <vector>

namespace rigExec {

bool
RigExecBakedVerifyConesRequested()
{
    static const bool requested =
        TfGetenvBool("RIGEXEC_BAKED_VERIFY_CONES", false);
    return requested;
}

namespace {

// Equality, with a NaN counted equal to a NaN.
// The two runs compared here are the SAME program over the SAME inputs, so a
// field holding a non-finite number in one holds the identical one in the
// other -- and `==` calls every one of them a difference, because a NaN is
// equal to nothing, itself included.
// Rigs carry non-finite numbers into slots on purpose. A mover whose inputs
// the kernel rejects still publishes the packet it rejected, NaN and all,
// which is how the pass-through diagnostic can name the value; an override
// of inputs:defaultWeight to a NaN is the fixture that drives it. Comparing
// those with `==` turned a correct cone into three "baked cone mismatch"
// lines and a failed parity run -- the instrument crying wolf on the one
// generation a reader most needs to trust it.
// So every floating-point comparison below goes through Same(), which counts
// two NaNs as the same value and is `==` otherwise. ANY type with a float or
// a double anywhere in it needs its own overload: the fallback at the end of
// the list is `a == b`, which is right for a path, a token or an integer and
// silently wrong for a new struct with a coordinate in it.

/// Whether two scalars are the same value, counting a NaN as equal to a NaN.
inline bool Same(float a, float b);
inline bool Same(double a, double b);
/// Elementwise, through the scalar rule.
inline bool Same(const GfVec2f &a, const GfVec2f &b);
inline bool Same(const GfVec3f &a, const GfVec3f &b);
inline bool Same(const GfVec3d &a, const GfVec3d &b);
inline bool Same(const GfQuatd &a, const GfQuatd &b);
inline bool Same(const GfMatrix3d &a, const GfMatrix3d &b);
inline bool Same(const GfMatrix4d &a, const GfMatrix4d &b);
/// Field by field, mirroring each type's own operator==.
inline bool Same(const RigExecPointFrame &a, const RigExecPointFrame &b);
inline bool Same(const RigExecPointFrameArray &a,
                 const RigExecPointFrameArray &b);
inline bool Same(const RigExecDualQuat &a, const RigExecDualQuat &b);
inline bool Same(const RigExecScaledDualQuat &a,
                 const RigExecScaledDualQuat &b);
inline bool Same(const RigExecWeightPacket &a, const RigExecWeightPacket &b);
inline bool Same(const RigExecMoverParameters &a,
                 const RigExecMoverParameters &b);
inline bool Same(const RigExecConstraintSource &a,
                 const RigExecConstraintSource &b);
/// Containers, elementwise.
template <class T, size_t N>
bool Same(const std::array<T, N> &a, const std::array<T, N> &b);
template <class T>
bool Same(const std::vector<T> &a, const std::vector<T> &b);
template <class T>
bool Same(const VtArray<T> &a, const VtArray<T> &b);
template <class K, class V>
bool Same(const std::map<K, V> &a, const std::map<K, V> &b);
/// Everything with no floating-point field in it: a path, a token, a bool,
/// an integer, a status. A new type with a coordinate in it must NOT reach
/// this -- give it an overload above.
template <class T>
bool Same(const T &a, const T &b);

inline bool
Same(float a, float b)
{
    return a == b || (std::isnan(a) && std::isnan(b));
}

inline bool
Same(double a, double b)
{
    return a == b || (std::isnan(a) && std::isnan(b));
}

inline bool
Same(const GfVec2f &a, const GfVec2f &b)
{
    return Same(a[0], b[0]) && Same(a[1], b[1]);
}

inline bool
Same(const GfVec3f &a, const GfVec3f &b)
{
    return Same(a[0], b[0]) && Same(a[1], b[1]) && Same(a[2], b[2]);
}

inline bool
Same(const GfVec3d &a, const GfVec3d &b)
{
    return Same(a[0], b[0]) && Same(a[1], b[1]) && Same(a[2], b[2]);
}

inline bool
Same(const GfQuatd &a, const GfQuatd &b)
{
    return Same(a.GetReal(), b.GetReal()) &&
           Same(a.GetImaginary(), b.GetImaginary());
}

inline bool
Same(const GfMatrix3d &a, const GfMatrix3d &b)
{
    for (int row = 0; row < 3; ++row) {
        for (int column = 0; column < 3; ++column) {
            if (!Same(a[row][column], b[row][column])) {
                return false;
            }
        }
    }
    return true;
}

inline bool
Same(const GfMatrix4d &a, const GfMatrix4d &b)
{
    for (int row = 0; row < 4; ++row) {
        for (int column = 0; column < 4; ++column) {
            if (!Same(a[row][column], b[row][column])) {
                return false;
            }
        }
    }
    return true;
}

inline bool
Same(const RigExecPointFrame &a, const RigExecPointFrame &b)
{
    return a.flags == b.flags && Same(a.points, b.points);
}

inline bool
Same(const RigExecPointFrameArray &a, const RigExecPointFrameArray &b)
{
    return Same(a.frames, b.frames) && Same(a.rests, b.rests);
}

inline bool
Same(const RigExecDualQuat &a, const RigExecDualQuat &b)
{
    return Same(a.real, b.real) && Same(a.dual, b.dual);
}

inline bool
Same(const RigExecScaledDualQuat &a, const RigExecScaledDualQuat &b)
{
    return Same(a.rigid, b.rigid) && Same(a.stretch, b.stretch) &&
           a.isRigid == b.isRigid;
}

inline bool
Same(const RigExecWeightPacket &a, const RigExecWeightPacket &b)
{
    return a.representation == b.representation &&
           a.rangePolicy == b.rangePolicy && Same(a.values, b.values) &&
           a.indices == b.indices &&
           Same(a.defaultWeight, b.defaultWeight) && a.valid == b.valid;
}

// A replay may allocate a distinct immutable handle for the same layout.
// Compare its complete typed content, including signed zero and NaN bits.
bool
SameSkinTopology(const std::shared_ptr<const RigExecSkinTopology> &a,
                 const std::shared_ptr<const RigExecSkinTopology> &b)
{
    if (a == b) return true;
    if (!a || !b) return false;
    return a->elementSize == b->elementSize &&
           a->pointCount == b->pointCount &&
           a->influenceCount == b->influenceCount &&
           a->validated == b->validated && a->indices == b->indices &&
           a->weights.size() == b->weights.size() &&
           (a->weights.empty() ||
            std::memcmp(a->weights.data(), b->weights.data(),
                        a->weights.size() * sizeof(float)) == 0);
}

bool
SameBlendLayout(const std::shared_ptr<const RigExecBlendSampleLayout> &a,
                const std::shared_ptr<const RigExecBlendSampleLayout> &b)
{
    if (a == b) return true;
    if (!a || !b || a->valid != b->valid ||
        a->pointCount != b->pointCount || a->indices != b->indices ||
        a->offsets.size() != b->offsets.size()) return false;
    for (size_t i = 0; i < a->offsets.size(); ++i)
        if (std::memcmp(&a->offsets[i][0], &b->offsets[i][0],
                        3 * sizeof(float)) != 0) return false;
    return true;
}

inline bool
Same(const RigExecMoverParameters &a, const RigExecMoverParameters &b)
{
    // Mirror the packet fields, comparing immutable layout content rather
    // than allocations made independently by the two verifier passes.
    return a.kind == b.kind && a.enabled == b.enabled && a.valid == b.valid &&
           a.radialWeight == b.radialWeight &&
           Same(a.transform, b.transform) && Same(a.weights, b.weights) &&
           Same(a.blendDeltas, b.blendDeltas) &&
           a.blendSurfaceFrame == b.blendSurfaceFrame &&
           Same(a.referenceVolume, b.referenceVolume) &&
           Same(a.strength, b.strength) &&
           a.topologyCounts == b.topologyCounts &&
           a.topologyIndices == b.topologyIndices &&
           Same(a.auxPoints, b.auxPoints) &&
           Same(a.auxPointsB, b.auxPointsB) &&
           Same(a.restPoints, b.restPoints) && a.divisions == b.divisions &&
           Same(a.bindCoords, b.bindCoords) && Same(a.frames, b.frames) &&
           a.wireBindCoords == b.wireBindCoords &&
           a.curveOrder == b.curveOrder && a.curveKnots == b.curveKnots &&
           a.dropoffDistance == b.dropoffDistance &&
           Same(a.widths, b.widths) &&
           Same(a.skinTransforms, b.skinTransforms) &&
           a.skinIndices == b.skinIndices &&
           Same(a.skinWeights, b.skinWeights) &&
           SameSkinTopology(a.skinTopology, b.skinTopology) &&
           a.skinElementSize == b.skinElementSize &&
           a.skinningMethod == b.skinningMethod &&
           a.externalSchema == b.externalSchema &&
           a.externalData == b.externalData;
}

inline bool
Same(const RigExecConstraintSource &a, const RigExecConstraintSource &b)
{
    return Same(a.frame, b.frame) &&
           Same(a.normalizedWeight, b.normalizedWeight) &&
           Same(a.translationOffset, b.translationOffset) &&
           Same(a.rotationOffsetDegrees, b.rotationOffsetDegrees);
}

template <class T, size_t N>
bool
Same(const std::array<T, N> &a, const std::array<T, N> &b)
{
    for (size_t i = 0; i < N; ++i) {
        if (!Same(a[i], b[i])) {
            return false;
        }
    }
    return true;
}

template <class T>
bool
Same(const std::vector<T> &a, const std::vector<T> &b)
{
    if (a.size() != b.size()) {
        return false;
    }
    for (size_t i = 0; i < a.size(); ++i) {
        if (!Same(a[i], b[i])) {
            return false;
        }
    }
    return true;
}

template <class T>
bool
Same(const VtArray<T> &a, const VtArray<T> &b)
{
    if (a.size() != b.size()) {
        return false;
    }
    for (size_t i = 0; i < a.size(); ++i) {
        if (!Same(a[i], b[i])) {
            return false;
        }
    }
    return true;
}

template <class K, class V>
bool
Same(const std::map<K, V> &a, const std::map<K, V> &b)
{
    if (a.size() != b.size()) {
        return false;
    }
    auto left = a.begin();
    auto right = b.begin();
    for (; left != a.end(); ++left, ++right) {
        if (left->first != right->first || !Same(left->second, right->second)) {
            return false;
        }
    }
    return true;
}

template <class T>
bool
Same(const T &a, const T &b)
{
    return a == b;
}

/// Appends one line naming \p what, and counts it.
void
Differ(std::vector<std::string> *differences, size_t *count,
       const std::string &what)
{
    ++*count;
    if (differences && differences->size() < 64) {
        differences->push_back("baked cone mismatch: " + what);
    }
}

template <class T>
void
CompareValue(std::vector<std::string> *differences, size_t *count,
             const std::string &what, const T &shadow, const T &current)
{
    if (!Same(shadow, current)) {
        Differ(differences, count, what);
    }
}

template <class T>
void
CompareVector(std::vector<std::string> *differences, size_t *count,
              const std::string &what, const std::vector<T> &shadow,
              const std::vector<T> &current)
{
    if (shadow.size() != current.size()) {
        Differ(differences, count, what + " size");
        return;
    }
    for (size_t i = 0; i < shadow.size(); ++i) {
        if (!Same(shadow[i], current[i])) {
            Differ(differences, count, what + "[" + std::to_string(i) + "]");
            return;
        }
    }
}

void
CaptureRevision(const RigExecBakedProgramImpl::GeomRevision &revision,
                RigExecBakedRunShadow::RevisionState *state)
{
    state->output = revision.output;
    state->stagingOutput = revision.stagingOutput;
    state->packetInfluences = revision.packetInfluences;
    state->influences = revision.influences;
    state->revisionInputs = revision.revisionInputs.DetachedCopy();
    state->rows = revision.rows;
    state->envelope = revision.envelope;
    state->palette = revision.palette;
    state->parameters = revision.parameters;
    state->lastParameters = revision.lastParameters;
    state->lastAuxPoints = revision.lastAuxPoints;
    state->status = revision.status;
    state->lastStatus = revision.lastStatus;
    state->resultStatus = revision.resultStatus;
    state->transform = revision.transform;
    state->precedingCount = revision.precedingCount;
    state->currentSource = revision.currentSource;
    state->defaultWeight = revision.defaultWeight;
    state->lastDefaultWeight = revision.lastDefaultWeight;
    state->haveTransform = revision.haveTransform;
    state->ran = revision.ran;
    state->executed = revision.executed;
    state->created = revision.created;
    state->influencesValid = revision.influencesValid;
    state->influencesChanged = revision.influencesChanged;
    state->staticDirty = revision.staticDirty;
    state->partitionStale = revision.partitionStale;
    state->publishedWeightValues = revision.publishedWeightValues;
    state->currentPhasePacket = revision.currentPhasePacket;
    state->weightFieldPublished = revision.weightFieldPublished;
    state->layoutUsable = revision.layoutUsable;
    state->topology = revision.topology;
    state->partitionTopology = revision.partitionTopology;
    state->topologyResolved = revision.topologyResolved;
    state->blendSamples.resize(revision.blendChannels.size());
    for (size_t c = 0; c < revision.blendChannels.size(); ++c) {
        const auto &samples = revision.blendChannels[c].samples;
        auto &saved = state->blendSamples[c];
        saved.resize(samples.size());
        for (size_t s = 0; s < samples.size(); ++s) {
            saved[s].layout = samples[s].layout;
            saved[s].lastPoints = samples[s].lastPoints;
            saved[s].layoutRefused = samples[s].layoutRefused;
        }
    }
    state->layoutHandle = revision.layoutHandle;
    state->layoutCandidate = revision.layoutCandidate;
    state->layoutFixed = revision.layoutFixed;
    state->layoutRan = revision.layoutRan;
    state->envelopeOk = revision.envelopeOk;
    state->fullStrength = revision.fullStrength;
    state->chunks.resize(revision.chunks.size());
    for (size_t k = 0; k < revision.chunks.size(); ++k) {
        state->chunks[k].transforms = revision.chunks[k].transforms;
        state->chunks[k].rows = revision.chunks[k].rows;
        state->chunks[k].palette = revision.chunks[k].palette;
        state->chunks[k].keyChanged = revision.chunks[k].keyChanged;
        state->chunks[k].ok = revision.chunks[k].ok;
    }
}

void
RestoreRevision(const RigExecBakedRunShadow::RevisionState &state,
                RigExecBakedProgramImpl::GeomRevision *revision)
{
    revision->output = state.output;
    revision->stagingOutput = state.stagingOutput;
    revision->packetInfluences = state.packetInfluences;
    revision->influences = state.influences;
    revision->revisionInputs = state.revisionInputs;
    revision->rows = state.rows;
    revision->envelope = state.envelope;
    revision->palette = state.palette;
    revision->parameters = state.parameters;
    revision->lastParameters = state.lastParameters;
    revision->lastAuxPoints = state.lastAuxPoints;
    revision->status = state.status;
    revision->lastStatus = state.lastStatus;
    revision->resultStatus = state.resultStatus;
    revision->transform = state.transform;
    revision->precedingCount = state.precedingCount;
    revision->currentSource = state.currentSource;
    revision->defaultWeight = state.defaultWeight;
    revision->lastDefaultWeight = state.lastDefaultWeight;
    revision->haveTransform = state.haveTransform;
    revision->ran = state.ran;
    revision->executed = state.executed;
    revision->created = state.created;
    revision->influencesValid = state.influencesValid;
    revision->influencesChanged = state.influencesChanged;
    revision->staticDirty = state.staticDirty;
    revision->partitionStale = state.partitionStale;
    revision->publishedWeightValues = state.publishedWeightValues;
    revision->currentPhasePacket = state.currentPhasePacket;
    revision->weightFieldPublished = state.weightFieldPublished;
    revision->layoutUsable = state.layoutUsable;
    revision->topology = state.topology;
    revision->partitionTopology = state.partitionTopology;
    revision->topologyResolved = state.topologyResolved;
    for (size_t c = 0; c < revision->blendChannels.size() &&
                       c < state.blendSamples.size(); ++c) {
        auto &samples = revision->blendChannels[c].samples;
        const auto &saved = state.blendSamples[c];
        for (size_t s = 0; s < samples.size() && s < saved.size(); ++s) {
            samples[s].layout = saved[s].layout;
            samples[s].lastPoints = saved[s].lastPoints;
            samples[s].layoutRefused = saved[s].layoutRefused;
        }
    }
    revision->layoutHandle = state.layoutHandle;
    revision->layoutCandidate = state.layoutCandidate;
    revision->layoutFixed = state.layoutFixed;
    revision->layoutRan = state.layoutRan;
    revision->envelopeOk = state.envelopeOk;
    revision->fullStrength = state.fullStrength;
    for (size_t k = 0; k < revision->chunks.size() && k < state.chunks.size();
         ++k) {
        revision->chunks[k].transforms = state.chunks[k].transforms;
        revision->chunks[k].rows = state.chunks[k].rows;
        revision->chunks[k].palette = state.chunks[k].palette;
        revision->chunks[k].keyChanged = state.chunks[k].keyChanged;
        revision->chunks[k].ok = state.chunks[k].ok;
    }
}

void
CompareRevision(std::vector<std::string> *differences, size_t *count,
                const std::string &where,
                const RigExecBakedRunShadow::RevisionState &shadow,
                const RigExecBakedProgramImpl::GeomRevision &revision,
                bool staticSelected = true, bool fuseSelected = true)
{
    if (!SameSkinTopology(shadow.layoutHandle,revision.layoutHandle) ||
        !SameSkinTopology(shadow.layoutCandidate,revision.layoutCandidate))
        Differ(differences,count,where+" layout handle");
    if (!SameSkinTopology(shadow.topology, revision.topology) ||
        !SameSkinTopology(shadow.partitionTopology, revision.partitionTopology))
        Differ(differences, count, where + " adopted topology");
    CompareValue(differences, count, where + " topologyResolved",
                 shadow.topologyResolved, revision.topologyResolved);
    if (shadow.blendSamples.size() != revision.blendChannels.size()) {
        Differ(differences, count, where + " blend channels size");
    } else for (size_t c = 0; c < shadow.blendSamples.size(); ++c) {
        const auto &saved = shadow.blendSamples[c];
        const auto &samples = revision.blendChannels[c].samples;
        if (saved.size() != samples.size()) {
            Differ(differences, count, where + " blend samples size");
            continue;
        }
        for (size_t s = 0; s < saved.size(); ++s) {
            if (!SameBlendLayout(saved[s].layout, samples[s].layout) ||
                !Same(saved[s].lastPoints, samples[s].lastPoints) ||
                saved[s].layoutRefused != samples[s].layoutRefused)
                Differ(differences, count, where + " blend sample state");
        }
    }
    CompareValue(differences,count,where+" layoutFixed",shadow.layoutFixed,revision.layoutFixed);
    CompareValue(differences,count,where+" layoutRan",shadow.layoutRan,revision.layoutRan);
    // The values.
    CompareVector(differences, count, where + " output", shadow.output,
                  revision.output);
    CompareVector(differences, count, where + " staging output", shadow.stagingOutput, revision.stagingOutput);
    CompareVector(differences, count, where + " packetInfluences",
                  shadow.packetInfluences, revision.packetInfluences);
    CompareVector(differences, count, where + " influences", shadow.influences,
                  revision.influences);
    CompareVector(differences, count, where + " rows", shadow.rows,
                  revision.rows);
    CompareVector(differences, count, where + " palette", shadow.palette,
                  revision.palette);
    CompareVector(differences, count, where + " envelope", shadow.envelope,
                  revision.envelope);
    CompareValue(differences, count, where + " parameters", shadow.parameters,
                 revision.parameters);
    CompareValue(differences, count, where + " status", shadow.status,
                 revision.status);
    CompareValue(differences, count, where + " lastParameters",
                 shadow.lastParameters, revision.lastParameters);
    // The half of lastParameters a derived revision keeps by handle.
    if (!Same(shadow.lastAuxPoints, revision.lastAuxPoints)) {
        Differ(differences, count, where + " lastAuxPoints");
    }
    CompareValue(differences, count, where + " lastStatus", shadow.lastStatus,
                 revision.lastStatus);
    CompareValue(differences, count, where + " resultStatus",
                 shadow.resultStatus, revision.resultStatus);
    CompareValue(differences, count, where + " currentSource",
                 shadow.currentSource, revision.currentSource);
    CompareValue(differences, count, where + " defaultWeight",
                 shadow.defaultWeight, revision.defaultWeight);
    CompareValue(differences, count, where + " lastDefaultWeight",
                 shadow.lastDefaultWeight, revision.lastDefaultWeight);
    CompareValue(differences, count, where + " transform", shadow.transform,
                 revision.transform);
    CompareValue(differences, count, where + " haveTransform",
                 shadow.haveTransform, revision.haveTransform);
    CompareValue(differences, count, where + " precedingCount",
                 shadow.precedingCount, revision.precedingCount);
    CompareValue(differences, count, where + " ran", shadow.ran, revision.ran);
    // The whole-array decisions RevisionStatic and InfluenceFold make.
    CompareValue(differences, count, where + " layoutUsable",
                 shadow.layoutUsable, revision.layoutUsable);
    CompareValue(differences, count, where + " envelopeOk", shadow.envelopeOk,
                 revision.envelopeOk);
    CompareValue(differences, count, where + " fullStrength",
                 shadow.fullStrength, revision.fullStrength);
    CompareValue(differences, count, where + " partitionStale",
                 shadow.partitionStale, revision.partitionStale);
    // The published influence overlay, which no slot names and which the
    // comparator DOES compare on the pose: a cone that skipped the assemble
    // of a revision whose packet moved would publish last generation's field
    // beside this generation's points.
    CompareVector(differences, count, where + " weightField",
                  shadow.publishedWeightValues, revision.publishedWeightValues);
    CompareValue(differences, count, where + " currentPhasePacket",
                 shadow.currentPhasePacket, revision.currentPhasePacket);
    CompareValue(differences, count, where + " weightFieldPublished",
                 shadow.weightFieldPublished, revision.weightFieldPublished);
    CompareValue(differences, count, where + " influencesValid",
                 shadow.influencesValid, revision.influencesValid);
    // Work flags follow the cone's own selected bodies. Value-based fold
    // changes still agree exactly against the shared starting state.
    // An unselected static/fuse body resets its flag under MarkSkipped;
    // the forced pass deliberately executes those same bodies.
    CompareValue(differences, count, where + " executed", shadow.executed,
                 fuseSelected && revision.executed);
    CompareValue(differences, count, where + " influencesChanged",
                 shadow.influencesChanged, revision.influencesChanged);
    CompareValue(differences, count, where + " staticDirty",
                 shadow.staticDirty, staticSelected && revision.staticDirty);
    for (size_t k = 0;
         k < shadow.chunks.size() && k < revision.chunks.size(); ++k) {
        const std::string what =
            where + " chunk " + std::to_string(k);
        CompareValue(differences, count, what + " ok", shadow.chunks[k].ok,
                     revision.chunks[k].ok);
        CompareValue(differences, count, what + " keyChanged",
                     shadow.chunks[k].keyChanged,
                     revision.chunks[k].keyChanged);
        CompareVector(differences, count, what + " transforms",
                      shadow.chunks[k].transforms,
                      revision.chunks[k].transforms);
        CompareVector(differences, count, what + " rows",
                      shadow.chunks[k].rows, revision.chunks[k].rows);
        CompareVector(differences, count, what + " palette",
                      shadow.chunks[k].palette, revision.chunks[k].palette);
    }
}

}  // namespace

void
RigExecBakedRunShadow::Capture(const RigExecBakedProgramImpl &program)
{
    const auto &B=program;

    values = B.propertyValues;
    versionValid = B.propertyVersionValid;
    changed = B.propertyChanged;
    chainValid = B.chainValid;
    recordStoodAside = B.recordStoodAside;
    chainFinal = B.chainFinal;
    recordValues = B.recordValues;
    propertyResults = B.propertyResults;
    hasResolvedInputs = B.resolvedInputs != nullptr;
    if (hasResolvedInputs) resolvedInputs = B.resolvedInputs->DetachedCopy();
    headOverrides = B.headOverrides;
    lastHeadOverrides = B.lastHeadOverrides;
    headOverrideMoved = B.headOverrideMoved;
    readerWalkMoved = B.readerWalkMoved;
    readerWalkChanged = B.readerWalkChanged;



    tables.restM = B.restM;
    tables.restPts = B.restPts;
    tables.restFrames = B.restFrames;
    tables.selfD = B.selfD;
    tables.parentDinv = B.parentDinv;
    tables.posedD = B.posedD;
    tables.parentSpaceM = B.parentSpaceM;
    tables.parentSpaceAuthored = B.parentSpaceAuthored;
    tables.rotationSign = B.rotationSign;

    tables.rotOrder = B.rotOrder;
    tables.restRoundTrip = B.restRoundTrip;
    tables.defaultRoundTrip = B.defaultRoundTrip;
    tables.posedAuthored = B.posedAuthored;
    tables.posedAuthoredM = B.posedAuthoredM;
    lastRestM = B.lastRestM;
    lastSelfD = B.lastSelfD;
    lastParentDinv = B.lastParentDinv;
    lastPosedAuthoredM = B.lastPosedAuthoredM;
    lastPosedAuthored = B.lastPosedAuthored;
    lastRotOrder = B.lastRotOrder;
    lastPosedD = B.lastPosedD;
    lastParentSpaceM = B.lastParentSpaceM;
    lastParentSpaceAuthored = B.lastParentSpaceAuthored;
    lastRotationSign = B.lastRotationSign;

    restChanged = B.restChanged;
    ladderChanged = B.ladderChanged;
    restMoved = B.restMoved;
    ladderMoved = B.ladderMoved;

    opsRun = B.headOpsRun;

    avars = program.avars;
    poseWeights = program.poseWeights;
    weightPackets = program.weightPackets;
    providerValues = program.providerValues;
    switchFrames = program.switchFrames;
    opAdapter = program.opAdapter;
    opExecution = program.opExecution;
    oraclePublications = program.oraclePublications;
    oracleWeightInputs = program.oracleWeightInputs;
    posedM = program.posedM;
    finalMatrix = program.finalMatrix;
    baseMatrix = program.baseMatrix;
    base = program.base;
    fin = program.fin;
    aggregates = program.aggregates;
    deltaValues = program.deltaValues;
    deltaPresent = program.deltaPresent;
    volumePlacement = program.volumePlacement;
    volumePlacementBase = program.volumePlacementBase;
    weightFields = program.weightFields;
    frameMatrix = program.frameMatrix;
    frameMatrixValid = program.frameMatrixValid;
    avarsDisturbed = program.avarsDisturbed;

    solvers.resize(program.solvers.size());
    for (size_t s = 0; s < program.solvers.size(); ++s) {
        solvers[s].outFrames = program.solvers[s].outFrames;
        solvers[s].outPresent = program.solvers[s].outPresent;
        solvers[s].fallbackSlots = program.solvers[s].fallbackSlots;
    }
    commits.resize(program.commits.size());
    for (size_t c = 0; c < program.commits.size(); ++c) {
        commits[c].present = program.commits[c].present;
        commits[c].deltaOk = program.commits[c].deltaOk;
        commits[c].frames = program.commits[c].frames;
        commits[c].staged = program.commits[c].staged;
        commits[c].deltas = program.commits[c].deltas;
        commits[c].outcome = program.commits[c].outcome;
        commits[c].sources = program.commits[c].sources;
        commits[c].abandoned = program.commits[c].abandoned;
        commits[c].recordAfter = program.commits[c].recordAfter;
        commits[c].recordEveryTarget = program.commits[c].recordEveryTarget;
    }
    chains.resize(program.chains.size());
    for (size_t c = 0; c < program.chains.size(); ++c) {
        const RigExecBakedProgramImpl::GeomChain &chain = program.chains[c];
        chains[c].lastBase = chain.lastBase;
        chains[c].result = chain.result;
        chains[c].spare = chain.spare;
        chains[c].haveResult = chain.haveResult;
        chains[c].haveBase = chain.haveBase;
        chains[c].baseDirty = chain.baseDirty;
        chains[c].scheduleDirty = chain.scheduleDirty;
        chains[c].revisions.resize(chain.revisions.size());
        for (size_t r = 0; r < chain.revisions.size(); ++r) {
            CaptureRevision(chain.revisions[r], &chains[c].revisions[r]);
        }
        chains[c].derived.resize(chain.derived.size());
        for (size_t d = 0; d < chain.derived.size(); ++d) {
            CaptureRevision(chain.derived[d].revision,
                            &chains[c].derived[d].revision);
            chains[c].derived[d].result = chain.derived[d].result;
            chains[c].derived[d].spare = chain.derived[d].spare;
            chains[c].derived[d].lastBase = chain.derived[d].lastBase;
            chains[c].derived[d].haveResult = chain.derived[d].haveResult;
            chains[c].derived[d].haveBase = chain.derived[d].haveBase;
            chains[c].derived[d].baseDirty = chain.derived[d].baseDirty;
            chains[c].derived[d].matrix = chain.derived[d].matrix;
            chains[c].derived[d].haveMatrix = chain.derived[d].haveMatrix;
        }
    }
    steps.resize(program.steps.size());
    for (size_t k = 0; k < program.steps.size(); ++k) {
        steps[k].diagnostics = program.steps[k].diagnostics;
        steps[k].lines = program.steps[k].lines;
        steps[k].counters = program.steps[k].counters;
    }
}

void
RigExecBakedRunShadow::Restore(RigExecBakedProgramImpl *program) const
{
    RigExecBakedProgramImpl &B = *program;

    B.propertyValues = values;
    B.propertyVersionValid = versionValid;
    B.propertyChanged = changed;
    B.chainValid = chainValid;
    B.recordStoodAside = recordStoodAside;
    B.chainFinal = chainFinal;
    B.recordValues = recordValues;
    B.propertyResults = propertyResults;
    if (hasResolvedInputs && B.resolvedInputs) *B.resolvedInputs = resolvedInputs;
    B.headOverrides = headOverrides;
    B.lastHeadOverrides = lastHeadOverrides;
    B.headOverrideMoved = headOverrideMoved;
    B.readerWalkMoved = readerWalkMoved;
    B.readerWalkChanged = readerWalkChanged;



    B.restM = tables.restM;
    B.restPts = tables.restPts;
    B.restFrames = tables.restFrames;
    B.selfD = tables.selfD;
    B.parentDinv = tables.parentDinv;
    B.posedD = tables.posedD;
    B.parentSpaceM = tables.parentSpaceM;
    B.parentSpaceAuthored = tables.parentSpaceAuthored;
    B.rotationSign = tables.rotationSign;

    B.rotOrder = tables.rotOrder;
    B.restRoundTrip = tables.restRoundTrip;
    B.defaultRoundTrip = tables.defaultRoundTrip;
    B.posedAuthored = tables.posedAuthored;
    B.posedAuthoredM = tables.posedAuthoredM;
    B.lastRestM = lastRestM;
    B.lastSelfD = lastSelfD;
    B.lastParentDinv = lastParentDinv;
    B.lastPosedAuthoredM = lastPosedAuthoredM;
    B.lastPosedAuthored = lastPosedAuthored;
    B.lastRotOrder = lastRotOrder;
    B.lastPosedD = lastPosedD;
    B.lastParentSpaceM = lastParentSpaceM;
    B.lastParentSpaceAuthored = lastParentSpaceAuthored;
    B.lastRotationSign = lastRotationSign;

    B.restChanged = restChanged;
    B.ladderChanged = ladderChanged;
    B.restMoved = restMoved;
    B.ladderMoved = ladderMoved;

    B.headOpsRun = opsRun;

    B.avars = avars;
    B.poseWeights = poseWeights;
    B.weightPackets = weightPackets;
    B.providerValues = providerValues;
    B.switchFrames = switchFrames;
    B.opAdapter = opAdapter;
    B.opExecution = opExecution;
    B.oraclePublications = oraclePublications;
    B.oracleWeightInputs = oracleWeightInputs;
    B.posedM = posedM;
    B.finalMatrix = finalMatrix;
    B.baseMatrix = baseMatrix;
    B.base = base;
    B.fin = fin;
    B.aggregates = aggregates;
    B.deltaValues = deltaValues;
    B.deltaPresent = deltaPresent;
    B.volumePlacement = volumePlacement;
    B.volumePlacementBase = volumePlacementBase;
    B.weightFields = weightFields;
    B.frameMatrix = frameMatrix;
    B.frameMatrixValid = frameMatrixValid;
    B.avarsDisturbed = avarsDisturbed;

    for (size_t s = 0; s < B.solvers.size() && s < solvers.size(); ++s) {
        B.solvers[s].outFrames = solvers[s].outFrames;
        B.solvers[s].outPresent = solvers[s].outPresent;
        B.solvers[s].fallbackSlots = solvers[s].fallbackSlots;
    }
    for (size_t c = 0; c < B.commits.size() && c < commits.size(); ++c) {
        B.commits[c].present = commits[c].present;
        B.commits[c].deltaOk = commits[c].deltaOk;
        B.commits[c].frames = commits[c].frames;
        B.commits[c].staged = commits[c].staged;
        B.commits[c].deltas = commits[c].deltas;
        B.commits[c].outcome = commits[c].outcome;
        B.commits[c].sources = commits[c].sources;
        B.commits[c].abandoned = commits[c].abandoned;
        B.commits[c].recordAfter = commits[c].recordAfter;
        B.commits[c].recordEveryTarget = commits[c].recordEveryTarget;
    }
    for (size_t c = 0; c < B.chains.size() && c < chains.size(); ++c) {
        RigExecBakedProgramImpl::GeomChain &chain = B.chains[c];
        chain.lastBase = chains[c].lastBase;
        chain.result = chains[c].result;
        chain.spare = chains[c].spare;
        chain.haveResult = chains[c].haveResult;
        chain.haveBase = chains[c].haveBase;
        chain.baseDirty = chains[c].baseDirty;
        chain.scheduleDirty = chains[c].scheduleDirty;
        for (size_t r = 0;
             r < chain.revisions.size() && r < chains[c].revisions.size();
             ++r) {
            RestoreRevision(chains[c].revisions[r], &chain.revisions[r]);
        }
        for (size_t d = 0;
             d < chain.derived.size() && d < chains[c].derived.size(); ++d) {
            RestoreRevision(chains[c].derived[d].revision,
                            &chain.derived[d].revision);
            chain.derived[d].result = chains[c].derived[d].result;
            chain.derived[d].spare = chains[c].derived[d].spare;
            chain.derived[d].lastBase = chains[c].derived[d].lastBase;
            chain.derived[d].haveResult = chains[c].derived[d].haveResult;
            chain.derived[d].haveBase = chains[c].derived[d].haveBase;
            chain.derived[d].baseDirty = chains[c].derived[d].baseDirty;
            chain.derived[d].matrix = chains[c].derived[d].matrix;
            chain.derived[d].haveMatrix = chains[c].derived[d].haveMatrix;
        }
    }
    for (size_t k = 0; k < B.steps.size() && k < steps.size(); ++k) {
        B.steps[k].diagnostics = steps[k].diagnostics;
        B.steps[k].lines = steps[k].lines;
        B.steps[k].counters = steps[k].counters;
    }
}

size_t
RigExecBakedRunShadow::Compare(const RigExecBakedProgramImpl &program,
                               std::vector<std::string> *differences) const
{
    size_t count = 0;
    // Project work observations through the cone's actual body selection.
    // A forced run computes identical values while deliberately doing more work.
    std::vector<char> selected(program.steps.size(), 1);
    if (!program.opGraph.ops.empty()) {
        if (opExecution.ran.size() != program.opGraph.ops.size())
            Differ(differences, &count, "operation selection size");
        for (size_t c = 0; c < program.opGraph.ops.size(); ++c) {
            const size_t step = program.opGraph.ops[c].originalIndex;
            if (step >= selected.size() || c >= opExecution.ran.size()) {
                Differ(differences, &count, "operation selection binding");
                continue;
            }
            selected[step] = opExecution.ran[c];
        }
    }
    std::vector<std::vector<char>> staticSelected(program.chains.size()), fuseSelected(program.chains.size());
    for (size_t c = 0; c < program.chains.size(); ++c) {
        staticSelected[c].assign(program.chains[c].revisions.size(), 1);
        fuseSelected[c].assign(program.chains[c].revisions.size(), 1);
    }
    for (size_t s = 0; s < program.steps.size(); ++s) {
        const auto &step = program.steps[s];
        if (step.kind != RigExecBakedStepKind::RevisionStatic &&
            step.kind != RigExecBakedStepKind::RevisionFuse) continue;
        if (step.object < 0 || size_t(step.object) >= program.revisionIndex.size()) continue;
        const auto [c,r] = program.revisionIndex[size_t(step.object)];
        (step.kind == RigExecBakedStepKind::RevisionStatic ? staticSelected : fuseSelected)
            [size_t(c)][size_t(r)] = selected[s];
    }
    CompareVector(differences,&count,"property version validity",versionValid,program.propertyVersionValid);
    CompareVector(differences,&count,"property chain validity",chainValid,program.chainValid);
    CompareVector(differences,&count,"property changed",changed,program.propertyChanged);
    CompareVector(differences,&count,"phased record stand-aside",recordStoodAside,program.recordStoodAside);

    { const auto &B=program; const auto &memo=*this;
    size_t mismatches = 0;
    const auto differ = [&](const std::string &what) {
        ++mismatches;
        differences->push_back("baked cone mismatch: " + what);
    };
    for (size_t c = 0; c < B.propertyChains.size(); ++c) {
        const RigExecBakedPropertyChain &chain = B.propertyChains[c];
        for (size_t k = 0; k <= chain.revisions.size(); ++k) {
            const uint32_t id = chain.versionBase + uint32_t(k);
            if (memo.versionValid[id] != B.propertyVersionValid[id]) {
                differ("head version " + std::to_string(k) + " of " +
                       chain.target.GetString() + " differs in validity");
                continue;
            }
            if (!B.propertyVersionValid[id]) {
                continue;
            }
            const RigExecBakedPropertyValue &a = memo.values[id];
            const RigExecBakedPropertyValue &b = B.propertyValues[id];
            bool same = true;
            switch (chain.arm) {
            case RigExecBakedPropertyChain::Arm::Float:
                same = std::memcmp(&a.f, &b.f, sizeof(a.f)) == 0;
                break;
            case RigExecBakedPropertyChain::Arm::Double:
                same = std::memcmp(&a.d, &b.d, sizeof(a.d)) == 0;
                break;
            case RigExecBakedPropertyChain::Arm::Matrix4d:
                same = std::memcmp(&a.m, &b.m, sizeof(a.m)) == 0;
                break;
            case RigExecBakedPropertyChain::Arm::Vec3f:
                same = std::memcmp(&a.v, &b.v, sizeof(a.v)) == 0;
                break;
            }
            if (!same) {
                differ("head version " + std::to_string(k) + " of " +
                       chain.target.GetString() + " differs");
            }
        }
        if (B.chainValid[c] &&
            !RigExecBakedHeadValueSame(memo.chainFinal[c], B.chainFinal[c])) {
            differ("head final value of " + chain.target.GetString() +
                   " differs");
        }
        for (const uint32_t r : chain.records) {
            if (B.chainValid[c] && !B.recordStoodAside[r] &&
                !RigExecBakedHeadValueSame(memo.recordValues[r],
                                           B.recordValues[r])) {
                differ("head record " +
                       B.propertyRecords[r].consumer.GetString() +
                       " differs");
            }
        }
    }
    for (size_t i = 0; i < B.steps.size(); ++i) {
        if (memo.steps[i].lines != B.steps[i].lines) {
            differ("head step " + B.steps[i].label +
                   " reports different lines");
        }
    }
    count += mismatches;

    }
    { const auto &B=program; const auto &memo=*this;
    size_t mismatches = 0;
    const auto bits = [](const auto &a, const auto &b) {
        return std::memcmp(&a, &b, sizeof(a)) == 0;
    };
    const RigExecBakedLadderTables &T = memo.tables;
    for (size_t slot = 0; slot < B.paths.size(); ++slot) {
        const bool same =
            bits(T.restM[slot], B.restM[slot]) &&
            bits(T.restPts[slot], B.restPts[slot]) &&
            T.restFrames[slot].flags == B.restFrames[slot].flags &&
            bits(T.restFrames[slot].points, B.restFrames[slot].points) &&
            bits(T.restRoundTrip[slot], B.restRoundTrip[slot]) &&
            bits(T.selfD[slot], B.selfD[slot]) &&
            bits(T.parentDinv[slot], B.parentDinv[slot]) &&
            bits(T.posedD[slot], B.posedD[slot]) &&
            bits(T.parentSpaceM[slot], B.parentSpaceM[slot]) &&
            T.parentSpaceAuthored[slot] == B.parentSpaceAuthored[slot] &&
            T.rotationSign[slot] == B.rotationSign[slot] &&
            bits(T.defaultRoundTrip[slot], B.defaultRoundTrip[slot]) &&
            T.posedAuthored[slot] == B.posedAuthored[slot] &&
            bits(T.posedAuthoredM[slot], B.posedAuthoredM[slot]) &&
            T.rotOrder[slot] == B.rotOrder[slot] &&
            memo.restChanged[slot] == B.restChanged[slot] &&
            memo.ladderChanged[slot] == B.ladderChanged[slot];
        if (!same) {
            ++mismatches;
            differences->push_back("baked cone mismatch: head rest or ladder "
                                   "of " + B.paths[slot].GetString() +
                                   " differs");
        }
    }
    count += mismatches;

    }
    // These last-value tables determine whether a later generation is
    // dirty. Equal current outputs alone do not establish equal memo state.
    const auto memoBits = [&](const std::string &name, const auto &a, const auto &b) {
        if (a.size() != b.size()) {
            Differ(differences, &count, name + " size");
            return;
        }
        for (size_t slot = 0; slot < a.size(); ++slot) {
            if (std::memcmp(&a[slot], &b[slot], sizeof(a[slot])) != 0)
                Differ(differences, &count, name + " slot " + std::to_string(slot));
        }
    };
    memoBits("last rest", lastRestM, program.lastRestM);
    memoBits("last default", lastSelfD, program.lastSelfD);
    memoBits("last parent default inverse", lastParentDinv, program.lastParentDinv);
    memoBits("last authored pose", lastPosedAuthoredM, program.lastPosedAuthoredM);
    memoBits("last posed default", lastPosedD, program.lastPosedD);
    memoBits("last parent space", lastParentSpaceM, program.lastParentSpaceM);
    CompareVector(differences, &count, "last parent space presence", lastParentSpaceAuthored, program.lastParentSpaceAuthored);
    CompareVector(differences, &count, "last rotation sign", lastRotationSign, program.lastRotationSign);
    CompareVector(differences, &count, "last authored pose presence",
                  lastPosedAuthored, program.lastPosedAuthored);
    CompareVector(differences, &count, "last rotation order", lastRotOrder, program.lastRotOrder);
    CompareVector(differences, &count, "rest moved", restMoved, program.restMoved);
    CompareVector(differences, &count, "ladder moved", ladderMoved, program.ladderMoved);
    // The avar table is the prologue's, and the prologue ran once: a
    // difference here is a STEP that wrote it, which is the one thing
    // nothing else in the program is positioned to notice.
    CompareVector(differences, &count, "avars", avars, program.avars);
    // The pose-interpolator hand-off to the geometry half: a cone that
    // skipped an interpolator kept last run's weights, and they have to be
    // what a whole run computes again.
    CompareVector(differences, &count, "poseWeights", poseWeights,
                  program.poseWeights);
    CompareVector(differences, &count, "weightPackets", weightPackets,
                  program.weightPackets);
    CompareVector(differences, &count, "posedM", posedM, program.posedM);
    CompareVector(differences, &count, "base", base, program.base);
    CompareVector(differences, &count, "fin", fin, program.fin);
    CompareVector(differences, &count, "switchFrames", switchFrames, program.switchFrames);
    CompareValue(differences, &count, "provider value count", providerValues.values.size(), program.providerValues.values.size());
    for (size_t i = 0; i < std::min(providerValues.values.size(), program.providerValues.values.size()); ++i) {
        const auto &a = providerValues.values[i]; const auto &b = program.providerValues.values[i];
        const std::string where = "provider value " + std::to_string(i) +
            (i < program.providerProgram.valueKeys.size() ? " (" + program.providerProgram.valueKeys[i] + ")" : "");
        bool same = a.value.index() == b.value.index();
        if (same) std::visit([&](const auto &value) {
            using T = std::decay_t<decltype(value)>;
            same = RigExecTypedSame(value, std::get<T>(b.value));
        }, a.value);
        if (!same || !RigExecTypedSame(a.raw,b.raw) || a.initialized != b.initialized ||
            a.blocked != b.blocked || a.authoritative != b.authoritative ||
            a.count != b.count || a.error != b.error)
            Differ(differences, &count, where);
    }
    CompareVector(differences, &count, "finalMatrix", finalMatrix,
                  program.finalMatrix);
    CompareVector(differences, &count, "baseMatrix", baseMatrix,
                  program.baseMatrix);
    CompareVector(differences, &count, "aggregates", aggregates,
                  program.aggregates);
    // The geometry half's hand-off from the pose half: what each
    // geometry-domain constraint measured, and whether it measured anything
    // at all. A cone that skipped the constraint and kept last run's answer
    // has to agree with a whole run that measured it again.
    CompareVector(differences, &count, "constraint deltas", deltaValues,
                  program.deltaValues);
    CompareVector(differences, &count, "constraint delta present",
                  deltaPresent, program.deltaPresent);
    // The weight half's hand-off: what the oracle and pose.weightFrames
    // read. A cone that skipped a VolumePlacements step kept that volume's
    // placement from the last run.
    CompareVector(differences, &count, "volumePlacement", volumePlacement,
                  program.volumePlacement);
    CompareVector(differences, &count, "volumePlacementBase", volumePlacementBase,
                  program.volumePlacementBase);
    CompareValue(differences,&count,"weightField count",weightFields.size(),program.weightFields.size());
    for (size_t i = 0; i < weightFields.size() && i < program.weightFields.size(); ++i) {
        const auto &before = weightFields[i];
        const auto &after = program.weightFields[i];
        if (before.values.size() != after.values.size() ||
            (!before.values.empty() && std::memcmp(before.values.data(),after.values.data(),
                                                    before.values.size() * sizeof(float)) != 0))
            Differ(differences,&count,"weightField values " + std::to_string(i));
        CompareValue(differences, &count, "weightField changed " + std::to_string(i),
                     before.changed, after.changed);
        CompareValue(differences, &count, "weightField sample count " + std::to_string(i),
                     before.count, after.count);
        CompareValue(differences, &count, "weightField validity " + std::to_string(i),
                     before.ok, after.ok);
        CompareValue(differences, &count, "weightField error " + std::to_string(i),
                     before.error, after.error);
    }
    // What an AtPrim transform phase reads: a cone that skipped a
    // FrameMatrix step kept the record it would write again.
    CompareVector(differences, &count, "frameMatrix", frameMatrix,
                  program.frameMatrix);
    CompareVector(differences, &count, "frameMatrixValid", frameMatrixValid,
                  program.frameMatrixValid);
    for (size_t s = 0; s < program.solvers.size() && s < solvers.size(); ++s) {
        const std::string where =
            "solver " + program.solvers[s].path.GetString();
        CompareVector(differences, &count, where + " outFrames",
                      solvers[s].outFrames, program.solvers[s].outFrames);
        CompareVector(differences, &count, where + " outPresent",
                      solvers[s].outPresent, program.solvers[s].outPresent);
        CompareVector(differences, &count, where + " fallbackJoints",
                      solvers[s].fallbackSlots,
                      program.solvers[s].fallbackSlots);
    }
    for (size_t c = 0; c < program.commits.size() && c < commits.size(); ++c) {
        const std::string where = "commit " + std::to_string(c);
        CompareVector(differences, &count, where + " present",
                      commits[c].present, program.commits[c].present);
        CompareVector(differences, &count, where + " frames",
                      commits[c].frames, program.commits[c].frames);
        CompareVector(differences, &count, where + " staged",
                      commits[c].staged, program.commits[c].staged);
        CompareVector(differences, &count, where + " outcome",
                      commits[c].outcome, program.commits[c].outcome);
        // The split commit's three steps hand each other these: the delta
        // CommitDelta measured, whether it measured one at all, and the
        // source frames a constraint gathered.
        CompareVector(differences, &count, where + " deltaOk",
                      commits[c].deltaOk, program.commits[c].deltaOk);
        // Every delta a candidate is PRESENT for, with nothing excused.
        // This comparison used to be guarded, because two runs of one
        // generation disagreed about the deltas of commits with no
        // propagation pairs while agreeing about `present`, `frames`,
        // `staged`, `outcome` and `deltaOk`. A delta is a pure function of
        // its candidate's frame and of B.fin[slot] as it stands BEFORE the
        // commit, so two passes that agree about every input to it cannot
        // disagree about its output unless one of them did not compute it --
        // and that was the whole story: the head step read B.fin[slot] and
        // declared it only as a WRITE, so a cone could skip the commit in a
        // generation that moved the slot and leave the delta measuring
        // against its own last answer. The read is declared now
        // (bakedPose.cpp, beside declarePropagation) and the disagreement is
        // gone: 0 differences over the eight bakeable fixtures at
        // RIGEXEC_BAKED_GRAIN_US=0, the grain that puts every step in its own
        // cluster, against 3 per generation without the declaration.
        for (size_t pos = 0;
             pos < commits[c].deltas.size() &&
                 pos < program.commits[c].deltas.size() &&
                 pos < program.commits[c].deltaOk.size();
             ++pos) {
            if (!program.commits[c].deltaOk[pos]) {
                continue;
            }
            CompareValue(differences, &count,
                         where + " deltas[" + std::to_string(pos) + "]",
                         commits[c].deltas[pos],
                         program.commits[c].deltas[pos]);
        }
        CompareVector(differences, &count, where + " sources",
                      commits[c].sources, program.commits[c].sources);
        CompareValue(differences, &count, where + " abandoned",
                     commits[c].abandoned, program.commits[c].abandoned);
        CompareValue(differences, &count, where + " recordAfter",
                     commits[c].recordAfter, program.commits[c].recordAfter);
        CompareValue(differences, &count, where + " recordEveryTarget",
                     commits[c].recordEveryTarget,
                     program.commits[c].recordEveryTarget);
    }
    for (size_t c = 0; c < program.chains.size() && c < chains.size(); ++c) {
        const RigExecBakedProgramImpl::GeomChain &chain = program.chains[c];
        const std::string where = "chain " + chain.target.GetString();
        if (!Same(chains[c].result, chain.result)) {
            Differ(differences, &count, where + " points");
        }
        CompareValue(differences, &count, where + " haveResult",
                     chains[c].haveResult, chain.haveResult);
        for (size_t r = 0;
             r < chain.revisions.size() && r < chains[c].revisions.size();
             ++r) {
            CompareRevision(differences, &count,
                            where + " revision " +
                                chain.revisions[r].moverPath.GetString(),
                            chains[c].revisions[r], chain.revisions[r],
                            staticSelected[c][r], fuseSelected[c][r]);
        }
        for (size_t d = 0;
             d < chain.derived.size() && d < chains[c].derived.size(); ++d) {
            if (!Same(chains[c].derived[d].result, chain.derived[d].result)) {
                Differ(differences, &count,
                       where + " derived " +
                           chain.derived[d].target.GetString() + " points");
            }
            CompareValue(differences, &count,
                         where + " derived " +
                             chain.derived[d].target.GetString() +
                             " haveResult",
                         chains[c].derived[d].haveResult,
                         chain.derived[d].haveResult);
            CompareValue(differences, &count,
                         where + " derived " +
                             chain.derived[d].target.GetString() +
                             " haveMatrix",
                         chains[c].derived[d].haveMatrix,
                         chain.derived[d].haveMatrix);
            if (!Same(chains[c].derived[d].matrix, chain.derived[d].matrix)) {
                Differ(differences, &count,
                       where + " derived " +
                           chain.derived[d].target.GetString() + " matrix");
            }
            CompareRevision(differences, &count,
                            where + " derived " +
                                chain.derived[d].target.GetString(),
                            chains[c].derived[d].revision,
                            chain.derived[d].revision);
        }
    }
    for (size_t k = 0; k < program.steps.size() && k < steps.size(); ++k) {
        const RigExecBakedStep &step = program.steps[k];
        const std::string where = "step " + step.label;
        CompareVector(differences, &count, where + " diagnostics",
                      steps[k].diagnostics, step.diagnostics);
        CompareVector(differences,&count,where+" memo lines",steps[k].lines,step.lines);
        auto expected = step.counters;
        if (!selected[k]) {
            // Exactly the MarkSkipped contract; program-size facts persist.
            expected.revisionsExecuted = 0;
            expected.revisionsCreated = 0;
            expected.schedulesBuilt = 0;
        }
        if (steps[k].counters.revisionsExecuted != expected.revisionsExecuted ||
            steps[k].counters.revisionsCreated != expected.revisionsCreated ||
            steps[k].counters.schedulesBuilt != expected.schedulesBuilt ||
            steps[k].counters.chainsBuilt != expected.chainsBuilt ||
            steps[k].counters.revisionsBuilt != expected.revisionsBuilt) {
            Differ(differences, &count, where + " counters");
        }
    }
    return count;
}

RigExecBakedRunStatistics::RigExecBakedRunStatistics(
    const RigExecBakedProgramImpl &program)
{
    opExecution = program.opExecution;
    selectedClusters = program.closed;
    selectedSteps = program.closedSteps;
    closureFull = program.closureFull;
    closedClusters = program.lastClosedClusters;
    closedSteps = program.lastClosedSteps;
    timed = program.clustering.lastRunTimed;
    clusters.resize(program.clustering.clusters.size());
    for (size_t c = 0; c < clusters.size(); ++c) {
        const RigExecBakedCluster &cluster = program.clustering.clusters[c];
        clusters[c].readyUs = cluster.readyUs;
        clusters[c].startUs = cluster.startUs;
        clusters[c].endUs = cluster.endUs;
        clusters[c].runner = cluster.runner;
    }
    steps.resize(program.steps.size());
    for (size_t k = 0; k < steps.size(); ++k) {
        steps[k].startUs = program.steps[k].startUs;
        steps[k].endUs = program.steps[k].endUs;
        steps[k].memoStartNs = program.steps[k].memoStartNs;
        steps[k].publishEndNs = program.steps[k].publishEndNs;
        steps[k].runner = program.steps[k].runner;
    }
}

void
RigExecBakedRunStatistics::Restore(RigExecBakedProgramImpl *program) const
{
    RigExecBakedProgramImpl &B = *program;
    B.opExecution = opExecution;
    B.closed = selectedClusters;
    B.closedSteps = selectedSteps;
    B.closureFull = closureFull;
    B.lastClosedClusters = closedClusters;
    B.lastClosedSteps = closedSteps;
    B.clustering.lastRunTimed = timed;
    for (size_t c = 0;
         c < B.clustering.clusters.size() && c < clusters.size(); ++c) {
        B.clustering.clusters[c].readyUs = clusters[c].readyUs;
        B.clustering.clusters[c].startUs = clusters[c].startUs;
        B.clustering.clusters[c].endUs = clusters[c].endUs;
        B.clustering.clusters[c].runner = clusters[c].runner;
    }
    for (size_t k = 0; k < B.steps.size() && k < steps.size(); ++k) {
        B.steps[k].startUs = steps[k].startUs;
        B.steps[k].endUs = steps[k].endUs;
        B.steps[k].memoStartNs = steps[k].memoStartNs;
        B.steps[k].publishEndNs = steps[k].publishEndNs;
        B.steps[k].runner = steps[k].runner;
    }
}

}  // namespace rigExec
