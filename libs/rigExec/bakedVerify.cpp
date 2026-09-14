//
// Proving a cone: RIGEXEC_BAKED_VERIFY_CONES.
//
// Cone re-execution is an argument -- "nothing outside the closure could
// have moved, so last run's values are this run's" -- and an argument about
// floating-point state is worth exactly what a machine can check of it. So
// this file lets one frame run twice from one starting point: the cone run's
// whole answer is shadowed, the starting point is put back, every step runs,
// and the two answers are compared slot by slot, counter by counter and
// diagnostic by diagnostic.
//
// Nothing here is on a production path. It is deliberately a deep copy of
// everything a step can touch, because a shadow that left a field out would
// agree with the cone run about the one thing the cone got wrong. What it
// leaves out on purpose, and why, is listed beside RigExecBakedRunShadow.
//
#include "bakedProgramImpl.h"

#include "pxr/base/tf/getenv.h"

#include <cmath>
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

// ---------------------------------------------------------------------------
// Equality, with a NaN counted equal to a NaN.
// ---------------------------------------------------------------------------
//
// The two runs compared here are the SAME program over the SAME inputs, so a
// field holding a non-finite number in one holds the identical one in the
// other -- and `==` calls every one of them a difference, because a NaN is
// equal to nothing, itself included.
//
// Rigs carry non-finite numbers into slots on purpose. A mover whose inputs
// the kernel rejects still publishes the packet it rejected, NaN and all,
// which is how the pass-through diagnostic can name the value; an override
// of inputs:defaultWeight to a NaN is the fixture that drives it. Comparing
// those with `==` turned a correct cone into three "baked cone mismatch"
// lines and a failed parity run -- the instrument crying wolf on the one
// generation a reader most needs to trust it.
//
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
inline bool Same(const RigExecCurvenetAdjustmentCommand &a,
                 const RigExecCurvenetAdjustmentCommand &b);
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

inline bool
Same(const RigExecMoverParameters &a, const RigExecMoverParameters &b)
{
    // Mirrors RigExecMoverParameters::operator== field for field. It has to
    // be kept beside it: a field added there and not here is a field this
    // mode stops looking at. The two shared_ptr members are compared by
    // IDENTITY, exactly as operator== compares them -- two packets naming
    // one epoch-fixed layout or one curvenet cut name the same object.
    return a.kind == b.kind && a.enabled == b.enabled && a.valid == b.valid &&
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
           Same(a.widths, b.widths) &&
           Same(a.skinTransforms, b.skinTransforms) &&
           a.skinIndices == b.skinIndices &&
           Same(a.skinWeights, b.skinWeights) &&
           a.skinTopology == b.skinTopology &&
           a.skinElementSize == b.skinElementSize &&
           a.skinningMethod == b.skinningMethod &&
           a.curvenetBinding == b.curvenetBinding &&
           a.curvenetAdjustmentBasis == b.curvenetAdjustmentBasis &&
           Same(a.curvenetAdjustments, b.curvenetAdjustments);
}

inline bool
Same(const RigExecConstraintSource &a, const RigExecConstraintSource &b)
{
    return Same(a.frame, b.frame) &&
           Same(a.normalizedWeight, b.normalizedWeight) &&
           Same(a.translationOffset, b.translationOffset) &&
           Same(a.rotationOffsetDegrees, b.rotationOffsetDegrees);
}

inline bool
Same(const RigExecCurvenetAdjustmentCommand &a,
     const RigExecCurvenetAdjustmentCommand &b)
{
    return a.pointIndex == b.pointIndex &&
           a.parentCommand == b.parentCommand &&
           a.includeTangents == b.includeTangents &&
           Same(a.localTransform, b.localTransform);
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
    state->packetInfluences = revision.packetInfluences;
    state->influences = revision.influences;
    state->revisionInputs = revision.revisionInputs;
    state->rows = revision.rows;
    state->envelope = revision.envelope;
    state->palette = revision.palette;
    state->parameters = revision.parameters;
    state->lastParameters = revision.lastParameters;
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
    state->influencesValid = revision.influencesValid;
    state->influencesChanged = revision.influencesChanged;
    state->staticDirty = revision.staticDirty;
    state->partitionStale = revision.partitionStale;
    state->layoutUsable = revision.layoutUsable;
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
    revision->packetInfluences = state.packetInfluences;
    revision->influences = state.influences;
    revision->revisionInputs = state.revisionInputs;
    revision->rows = state.rows;
    revision->envelope = state.envelope;
    revision->palette = state.palette;
    revision->parameters = state.parameters;
    revision->lastParameters = state.lastParameters;
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
    revision->influencesValid = state.influencesValid;
    revision->influencesChanged = state.influencesChanged;
    revision->staticDirty = state.staticDirty;
    revision->partitionStale = state.partitionStale;
    revision->layoutUsable = state.layoutUsable;
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
                const RigExecBakedProgramImpl::GeomRevision &revision)
{
    // The values.
    CompareVector(differences, count, where + " output", shadow.output,
                  revision.output);
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
    CompareValue(differences, count, where + " influencesValid",
                 shadow.influencesValid, revision.influencesValid);
    // The per-run DELTAS. These are the fields a skip resets
    // (RigExecBakedSkipGeometryStep) and the ones a cone can most easily get
    // wrong: a step that skipped says "nothing moved", and this is where
    // that claim is held against a run that recomputed the comparison.
    CompareValue(differences, count, where + " executed", shadow.executed,
                 revision.executed);
    CompareValue(differences, count, where + " influencesChanged",
                 shadow.influencesChanged, revision.influencesChanged);
    CompareValue(differences, count, where + " staticDirty",
                 shadow.staticDirty, revision.staticDirty);
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
    avars = program.avars;
    posedM = program.posedM;
    finalMatrix = program.finalMatrix;
    baseMatrix = program.baseMatrix;
    base = program.base;
    fin = program.fin;
    aggregates = program.aggregates;
    constraintDeltas = program.constraintDeltas;
    avarsDisturbed = program.avarsDisturbed;

    solvers.resize(program.solvers.size());
    for (size_t s = 0; s < program.solvers.size(); ++s) {
        solvers[s].outFrames = program.solvers[s].outFrames;
        solvers[s].outPresent = program.solvers[s].outPresent;
        solvers[s].fallbackJoints = program.solvers[s].fallbackJoints;
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
        }
    }
    steps.resize(program.steps.size());
    for (size_t k = 0; k < program.steps.size(); ++k) {
        steps[k].diagnostics = program.steps[k].diagnostics;
        steps[k].counters = program.steps[k].counters;
        steps[k].bail = program.steps[k].bail;
    }
}

void
RigExecBakedRunShadow::Restore(RigExecBakedProgramImpl *program) const
{
    RigExecBakedProgramImpl &B = *program;
    B.avars = avars;
    B.posedM = posedM;
    B.finalMatrix = finalMatrix;
    B.baseMatrix = baseMatrix;
    B.base = base;
    B.fin = fin;
    B.aggregates = aggregates;
    B.constraintDeltas = constraintDeltas;
    B.avarsDisturbed = avarsDisturbed;
    // Run-local by construction: the prologue empties it, so a second run
    // over one frame has to start with it empty too or every record lands in
    // it twice.
    B.runSnapshots.Clear();

    for (size_t s = 0; s < B.solvers.size() && s < solvers.size(); ++s) {
        B.solvers[s].outFrames = solvers[s].outFrames;
        B.solvers[s].outPresent = solvers[s].outPresent;
        B.solvers[s].fallbackJoints = solvers[s].fallbackJoints;
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
    }
    for (size_t c = 0; c < B.chains.size() && c < chains.size(); ++c) {
        RigExecBakedProgramImpl::GeomChain &chain = B.chains[c];
        chain.lastBase = chains[c].lastBase;
        chain.result = chains[c].result;
        chain.spare = chains[c].spare;
        chain.haveResult = chains[c].haveResult;
        chain.haveBase = chains[c].haveBase;
        chain.baseDirty = chains[c].baseDirty;
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
        }
    }
    for (size_t k = 0; k < B.steps.size() && k < steps.size(); ++k) {
        B.steps[k].diagnostics = steps[k].diagnostics;
        B.steps[k].counters = steps[k].counters;
        B.steps[k].bail = steps[k].bail;
    }
}

size_t
RigExecBakedRunShadow::Compare(const RigExecBakedProgramImpl &program,
                               std::vector<std::string> *differences) const
{
    size_t count = 0;
    // The avar table is the prologue's, and the prologue ran once: a
    // difference here is a STEP that wrote it, which is the one thing
    // nothing else in the program is positioned to notice.
    CompareVector(differences, &count, "avars", avars, program.avars);
    CompareVector(differences, &count, "posedM", posedM, program.posedM);
    CompareVector(differences, &count, "base", base, program.base);
    CompareVector(differences, &count, "fin", fin, program.fin);
    CompareVector(differences, &count, "finalMatrix", finalMatrix,
                  program.finalMatrix);
    CompareVector(differences, &count, "baseMatrix", baseMatrix,
                  program.baseMatrix);
    CompareVector(differences, &count, "aggregates", aggregates,
                  program.aggregates);
    // The geometry half's hand-off from the pose half. Empty on every rig
    // that bakes today -- IsBakeable refuses a geometry-domain constraint --
    // and compared anyway, because the group that adds the writer will want
    // to know whether a skipped constraint left a hole in it (§10).
    CompareValue(differences, &count, "constraintDeltas", constraintDeltas,
                 program.constraintDeltas);
    for (size_t s = 0; s < program.solvers.size() && s < solvers.size(); ++s) {
        const std::string where =
            "solver " + program.solvers[s].path.GetString();
        CompareVector(differences, &count, where + " outFrames",
                      solvers[s].outFrames, program.solvers[s].outFrames);
        CompareVector(differences, &count, where + " outPresent",
                      solvers[s].outPresent, program.solvers[s].outPresent);
        CompareVector(differences, &count, where + " fallbackJoints",
                      solvers[s].fallbackJoints,
                      program.solvers[s].fallbackJoints);
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
        // A delta is an answer only where something READS it. It is the
        // commit's own scratch rather than a slot: StageCommitPairs is the
        // only reader, over the commit's propagation pairs, and
        // ComputeCommitDeltas writes an entry only for a PRESENT candidate
        // (a zero flag is what stops anything reading the rest). So a
        // commit with no pairs computes a delta for nobody, and the two runs
        // then legitimately hold different ones -- the cone run's is from
        // the generation its commit last ran, the forced run's is this
        // generation's. Measured across every example fixture at three
        // grains in both schedules: every delta the two runs disagreed about
        // belonged to a commit with no propagation pairs, and none belonged
        // to one with any. (A commit with no pairs computing a delta at all
        // is dead per-frame work, and worth removing where it is built.)
        for (size_t pos = 0;
             !program.commits[c].propagate.empty() &&
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
                            chains[c].revisions[r], chain.revisions[r]);
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
        if (steps[k].counters.revisionsExecuted !=
                step.counters.revisionsExecuted ||
            steps[k].counters.revisionsCreated !=
                step.counters.revisionsCreated ||
            steps[k].counters.schedulesBuilt !=
                step.counters.schedulesBuilt ||
            steps[k].counters.chainsBuilt != step.counters.chainsBuilt ||
            steps[k].counters.revisionsBuilt != step.counters.revisionsBuilt) {
            Differ(differences, &count, where + " counters");
        }
        CompareValue(differences, &count, where + " bail", steps[k].bail,
                     step.bail);
    }
    return count;
}

RigExecBakedRunStatistics::RigExecBakedRunStatistics(
    const RigExecBakedProgramImpl &program)
{
    closedClusters = program.lastClosedClusters;
    timed = program.clustering.lastRunTimed;
    clusters.resize(program.clustering.clusters.size());
    for (size_t c = 0; c < clusters.size(); ++c) {
        const RigExecBakedCluster &cluster = program.clustering.clusters[c];
        clusters[c].readyUs = cluster.readyUs;
        clusters[c].startUs = cluster.startUs;
        clusters[c].endUs = cluster.endUs;
    }
}

void
RigExecBakedRunStatistics::Restore(RigExecBakedProgramImpl *program) const
{
    RigExecBakedProgramImpl &B = *program;
    B.lastClosedClusters = closedClusters;
    B.clustering.lastRunTimed = timed;
    for (size_t c = 0;
         c < B.clustering.clusters.size() && c < clusters.size(); ++c) {
        B.clustering.clusters[c].readyUs = clusters[c].readyUs;
        B.clustering.clusters[c].startUs = clusters[c].startUs;
        B.clustering.clusters[c].endUs = clusters[c].endUs;
    }
}

}  // namespace rigExec
