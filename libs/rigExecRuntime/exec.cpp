// rigExecRuntime frame driver (M2 framework).
// SetFrame selects a baked frame, whose slot values the input reads see;
// Execute places the standing overrides, runs the property chains, then
// the prologues, the region and the epilogue, then assembles the outputs.
// Failures name the step and keep the previous outputs.
#include "rigExecRuntime/runtime.h"

#include <algorithm>
#include <cmath>

namespace rigExec {

namespace {

// The store's publication maps key by string id (bake insertion order);
// every reader getter promises path order, so each assembly sorts by the
// resolved path text before it ships.
template <typename T>
void
_SortByPath(std::vector<T> *out)
{
    std::sort(out->begin(), out->end(),
              [](const T &a, const T &b) { return a.path < b.path; });
}

}  // namespace

std::vector<double>
RigExecRuntimeReader::GetFrameTimes() const
{
    std::vector<double> out;
    out.reserve(_inputs.frames.size());
    for (const auto &frame : _inputs.frames) {
        out.push_back(frame.frame);
    }
    return out;
}

bool
RigExecRuntimeReader::SetFrame(double frame, std::string *error)
{
    for (size_t i = 0; i < _inputs.frames.size(); ++i) {
        if (_inputs.frames[i].frame != frame) {
            continue;
        }
        const RigExecWireFrameInputs &record = _inputs.frames[i];
        if (record.uids.size() != record.values.size()) {
            if (error) {
                *error = "frame record pairs no values with its uids";
            }
            return false;
        }
        _frameIndex = i;
        _frameSelected = true;
        return true;
    }
    if (error) {
        char buffer[64];
        std::snprintf(buffer, sizeof(buffer), "no baked frame at %.17g",
                      frame);
        *error = buffer;
    }
    return false;
}

void
RigExecRuntimeReader::ClearAvars()
{
    _avarOverrides.clear();
}

bool
RigExecRuntimeReader::SetAvar(const std::string &path, double value,
                            std::string *error)
{
    // The avar's input slot, and the pose slot and channel of the avar
    // binding it heads.
    const RrInputState &state = _program.inputState;
    const auto found = state.nameIndex.find(path);
    const int32_t avar = found != state.nameIndex.end()
                             ? state.slotAvar[found->second]
                             : -1;
    const size_t slot = avar >= 0 ? size_t(avar) / 11 : 0;
    const int channel = avar >= 0 ? avar % 11 : -1;
    if (!std::isfinite(value) || avar < 0 || channel >= 9 ||
        _slotMeta.slotKind[slot] != RigExecWireSlotKind::FirstFramePose ||
        _constants.posedAuthored[slot] ||
        std::find(_slotMeta.controlSlots.begin(), _slotMeta.controlSlots.end(),
                  int(slot)) == _slotMeta.controlSlots.end() ||
        (channel >= 3 && channel <= 5 && _constants.noScaleAvars[slot])) {
        if (error) *error = "expected a finite TRS avar on a compiled pose slot: " + path;
        return false;
    }
    // An interactive override, as in the USD evaluators: every read whose
    // walk passes the avar reads it, and a phased reader whose consumer or
    // hop it is stands aside. On an avar math movers revise it is the
    // chain's base (RrChainBase), and the chain's result answers there.
    _avarOverrides[found->second] = value;
    return true;
}

void
RigExecRuntimeReader::SetRunMaskForTesting(unsigned mask)
{
    _program.runMask = mask;
}

void
RigExecRuntimeReader::SetCrossCheckForTesting(bool enabled)
{
    _program.crossCheck = enabled;
}

std::vector<RigExecRuntimeJointMatrix>
RigExecRuntimeReader::GetJointRestMatrices() const
{
    std::vector<RigExecRuntimeJointMatrix> out;
    for (size_t i = 0; i < _slotMeta.jointSlots.size(); ++i) {
        const size_t slot = size_t(_slotMeta.jointSlots[i]);
        RrMat4d rest;
        const RrPointFrame frame = RrWireToFrame(_constants.restFrames[slot]);
        if (RrPointsToMatrix(RrIdentityLandmarks(), frame.points, &rest)) {
            out.push_back({_program.TextOrEmpty(_slotMeta.jointPaths[i]), rest});
        }
    }
    _SortByPath(&out);
    return out;
}

std::vector<RigExecRuntimeJointMatrix>
RigExecRuntimeReader::GetJointPoseMatrices() const
{
    const auto rests = GetJointRestMatrices();
    std::vector<RigExecRuntimeJointMatrix> out;
    for (const auto &delta : _jointMatrices) {
        const auto rest = std::lower_bound(rests.begin(), rests.end(), delta.path,
            [](const RigExecRuntimeJointMatrix &a, const std::string &path) {
                return a.path < path;
            });
        if (rest != rests.end() && rest->path == delta.path) {
            out.push_back({delta.path, rest->matrix * delta.matrix});
        }
    }
    return out;
}

std::vector<RigExecRuntimePropertyValue>
RigExecRuntimeReader::GetPropertyResultsForTesting() const
{
    std::vector<RigExecRuntimePropertyValue> out;
    out.reserve(_program.store.propertyResults.size());
    for (const auto &entry : _program.store.propertyResults) {
        out.push_back({_program.TextOrEmpty(entry.first), entry.second});
    }
    _SortByPath(&out);
    return out;
}

const std::vector<RrPointFrame> &
RigExecRuntimeReader::GetFinFrames() const
{
    return _program.store.fin;
}

const std::vector<RrPointFrame> &
RigExecRuntimeReader::GetBaseFrames() const
{
    return _program.store.base;
}

const std::vector<RrMat4d> &
RigExecRuntimeReader::GetFinalMatrices() const
{
    return _program.store.finalMatrix;
}

const std::vector<RrMat4d> &
RigExecRuntimeReader::GetBaseMatrices() const
{
    return _program.store.baseMatrix;
}

const std::vector<RrWeightPacket> &
RigExecRuntimeReader::GetWeightPackets() const
{
    return _program.store.weightPackets;
}

bool
RigExecRuntimeReader::Execute(std::string *error)
{
    if (!_frameSelected) {
        if (error) {
            *error = "no frame selected";
        }
        return false;
    }
    RrProgram &program = _program;
    RrStore &store = program.store;
    const RigExecWireFrameInputs &record = _inputs.frames[_frameIndex];
    const double time = record.frame;
    program.frameIndex = _frameIndex;
    // The slot values every computed read sees this run.
    if (!RrInputsSelectFrame(&program, _frameIndex, error)) {
        return false;
    }
    // The standing overrides, placed before the chains as the program
    // places them: every read sees them from here on.
    RrInputsSetOverrides(&program, _avarOverrides);
    const bool crossCheck = program.CrossCheckThisRun();
    for (RrStepOutput &output : store.stepOutputs) {
        output.crossChecked.fill(0);
    }
    RrCrossCheckCounts checked{};

    // The property chains run first, as the baked prologue runs them: their
    // lines open the generation and their results are what every later
    // read of a chain target sees.
    std::vector<std::string> poseDiagnostics;
    if (!RrRunPropertyChains(&program, &poseDiagnostics)) {
        if (error) {
            *error = "the property chains disagree with the computed section";
        }
        return false;
    }
    // The chains' results and every input read, evaluated over the slots,
    // against the frame record.
    if (crossCheck &&
        (!RrCrossCheckPropertyResults(&program, record,
                                      &checked[RrCrossCheckPropertyValue],
                                      error) ||
         !RrCrossCheckChainReads(&program, record,
                                 &checked[RrCrossCheckChainRead], error) ||
         !RrCrossCheckReads(&program, record, &checked, error))) {
        return false;
    }
    store.runSnapshots.Clear();

    if ((program.runMask & 0x1u) != 0 &&
        !RrProloguePose(&program, record, &poseDiagnostics, error)) {
        return false;
    }
    if ((program.runMask & 0x4u) != 0 &&
        !RrPrologueGeometry(&program, time, record, &poseDiagnostics,
                            error)) {
        return false;
    }
    if (!RrRunSteps(&program, time, false, error)) {
        return false;
    }
    if (!RrPublishPose(&program, &poseDiagnostics, error)) {
        return false;
    }
    RrPublishGeometry(&program, record, &poseDiagnostics);

    RigExecRuntimeCounters counters;
    for (size_t s = 0; s < store.stepOutputs.size(); ++s) {
        const RrStepOutput &output = store.stepOutputs[s];
        counters.revisionsExecuted += output.counters.revisionsExecuted;
        counters.revisionsCreated += output.counters.revisionsCreated;
        counters.schedulesBuilt += output.counters.schedulesBuilt;
        counters.chainsBuilt += output.counters.chainsBuilt;
        counters.revisionsBuilt += output.counters.revisionsBuilt;
        for (size_t kind = 0; kind < RrCrossCheckKindCount; ++kind) {
            checked[kind] += output.crossChecked[kind];
        }
    }
    poseDiagnostics.push_back(
        "mover graph: " + std::to_string(counters.chainsBuilt) +
        " chain(s), " + std::to_string(counters.revisionsBuilt) +
        " revision(s); " +
        std::to_string(counters.revisionsCreated) + " created, " +
        std::to_string(counters.revisionsExecuted) + " executed, " +
        std::to_string(counters.schedulesBuilt) +
        " schedule(s) built");

    // A plugin mover this runtime has no kernel for is a no-op, said on
    // every frame it is one, beside the lines of the step it sat in.
    for (const auto &[slot, index] : program.externalIndex) {
        const RrProgram::ExternalRevision &state = program.externals[index];
        if (state.state) {
            continue;
        }
        const RigExecWireRevision &wire =
            _geometry.chains[slot.first].revisions[slot.second];
        poseDiagnostics.push_back(
            "warning: " + program.TextOrEmpty(wire.moverPath) + " is a " +
            state.type + ", which this runtime has no kernel for; its "
            "points pass through");
    }

    // The compile notices a fresh evaluator seeds its first generation
    // with (inert movers, purpose warnings): manifest order, ahead of
    // every program line, exactly once. Drained here, on the success
    // path only, so a failed Execute keeps both the seed and the
    // previous frame.
    if (!program.compileDiagnostics.empty()) {
        poseDiagnostics.insert(poseDiagnostics.begin(),
                               program.compileDiagnostics.begin(),
                               program.compileDiagnostics.end());
        program.compileDiagnostics.clear();
    }

    // Assemble the outputs only here, at the one exit that published:
    // a failure anywhere above keeps the previous frame.
    std::vector<RigExecRuntimeJointMatrix> jointMatrices;
    jointMatrices.reserve(store.jointMatricesFinal.size());
    for (const auto &entry : store.jointMatricesFinal) {
        RigExecRuntimeJointMatrix joint;
        joint.path = program.TextOrEmpty(entry.first);
        joint.matrix = entry.second;
        jointMatrices.push_back(std::move(joint));
    }
    std::vector<RigExecRuntimePoints> points;
    points.reserve(store.movedProperties.size());
    for (const auto &entry : store.movedProperties) {
        RigExecRuntimePoints moved;
        moved.path = program.TextOrEmpty(entry.first);
        moved.points = entry.second;
        points.push_back(std::move(moved));
    }
    std::vector<RigExecRuntimeMatrixPrimvar> matrixPrimvars;
    matrixPrimvars.reserve(store.movedMatrices.size());
    for (const auto &entry : store.movedMatrices) {
        RigExecRuntimeMatrixPrimvar primvar;
        primvar.path = program.TextOrEmpty(entry.first);
        primvar.matrix = entry.second;
        matrixPrimvars.push_back(std::move(primvar));
    }
    std::vector<RigExecRuntimeWeightFrame> weightFrames;
    weightFrames.reserve(store.weightFrames.size());
    for (const auto &entry : store.weightFrames) {
        RigExecRuntimeWeightFrame placed;
        placed.path = program.TextOrEmpty(entry.first);
        placed.matrix = entry.second;
        weightFrames.push_back(std::move(placed));
    }
    std::vector<RigExecRuntimeWeightField> weightFieldsOut;
    weightFieldsOut.reserve(store.weightFields.size());
    for (const auto &entry : store.weightFields) {
        RigExecRuntimeWeightField field;
        field.path = program.TextOrEmpty(entry.first);
        field.target = program.TextOrEmpty(entry.second.target);
        field.weights = entry.second.weights;
        weightFieldsOut.push_back(std::move(field));
    }
    _SortByPath(&jointMatrices);
    _SortByPath(&points);
    _SortByPath(&weightFrames);
    _SortByPath(&weightFieldsOut);
    std::vector<RigExecRuntimeProviderXform> providerXforms;
    providerXforms.reserve(store.providerXforms.size());
    for (const auto &entry : store.providerXforms) {
        const auto base = store.providerBaseXforms.find(entry.first);
        if (base == store.providerBaseXforms.end()) {
            continue;
        }
        RigExecRuntimeProviderXform revised;
        revised.path = program.TextOrEmpty(entry.first);
        revised.matrix = entry.second;
        revised.base = base->second;
        providerXforms.push_back(std::move(revised));
    }
    _SortByPath(&providerXforms);
    _jointMatrices = std::move(jointMatrices);
    _points = std::move(points);
    _matrixPrimvars = std::move(matrixPrimvars);
    _weightFrames = std::move(weightFrames);
    _weightFields = std::move(weightFieldsOut);
    _providerXforms = std::move(providerXforms);
    _diagnostics = std::move(poseDiagnostics);
    _counters = counters;
    for (size_t kind = 0; kind < RrCrossCheckKindCount; ++kind) {
        _crossCheckCounts[kind] += checked[kind];
    }
    return true;
}

}  // namespace rigExec
