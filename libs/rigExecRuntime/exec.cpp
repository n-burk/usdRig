//
// rigExecRuntime frame driver (M2 framework).
//
// SetFrame loads a baked frame's sparse values into the holders;
// Execute runs prologues, the region and the epilogue, then assembles
// the outputs. Failures name the step and keep the previous outputs.
//

#include "rigExecRuntime/runtime.h"

#include <algorithm>

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
        for (size_t k = 0; k < record.uids.size(); ++k) {
            const uint32_t uid = record.uids[k];
            if (uid >= _program.store.inputHolders.size() ||
                record.values[k].tag != _inputs.directory[uid].tag) {
                if (error) {
                    *error = "frame record names a value no input holds";
                }
                return false;
            }
            _program.store.inputHolders[uid] =
                RrWireFrameValue(record.values[k]);
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
RigExecRuntimeReader::SetRunMaskForTesting(unsigned mask)
{
    _program.runMask = mask;
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

    // The property chains' published values, straight from the record.
    if (record.propertyPaths.size() != record.propertyValues.size()) {
        if (error) {
            *error = "frame record pairs no values with its properties";
        }
        return false;
    }
    store.propertyResults.clear();
    for (size_t i = 0; i < record.propertyPaths.size(); ++i) {
        const RigExecWirePropertyValue &wire = record.propertyValues[i];
        RrPropertyValue value;
        value.tag = RrPropertyValue::Tag(wire.tag);
        value.f32 = wire.f32;
        value.f64 = wire.f64;
        for (size_t r = 0; r < 4; ++r) {
            for (size_t c = 0; c < 4; ++c) {
                value.matrix[r][c] = wire.matrix[r * 4 + c];
            }
        }
        value.vec = RrVec3f(wire.vec[0], wire.vec[1], wire.vec[2]);
        store.propertyResults[record.propertyPaths[i]] = value;
    }
    store.runSnapshots.Clear();

    std::vector<std::string> poseDiagnostics;
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

    // Whatever the curvenet binds reported; drained so a cached bind
    // stays silent on every later frame. Then the summary line, in
    // the same words the baked epilogue uses.
    for (std::string &message : store.curvenetBindDiagnostics) {
        poseDiagnostics.push_back(std::move(message));
    }
    store.curvenetBindDiagnostics.clear();
    RigExecRuntimeCounters counters;
    for (const RrStepOutput &output : store.stepOutputs) {
        counters.revisionsExecuted += output.counters.revisionsExecuted;
        counters.revisionsCreated += output.counters.revisionsCreated;
        counters.schedulesBuilt += output.counters.schedulesBuilt;
        counters.chainsBuilt += output.counters.chainsBuilt;
        counters.revisionsBuilt += output.counters.revisionsBuilt;
    }
    poseDiagnostics.push_back(
        "mover graph: " + std::to_string(counters.chainsBuilt) +
        " chain(s), " + std::to_string(counters.revisionsBuilt) +
        " revision(s); " +
        std::to_string(counters.revisionsCreated) + " created, " +
        std::to_string(counters.revisionsExecuted) + " executed, " +
        std::to_string(counters.schedulesBuilt) +
        " schedule(s) built");

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
    _weightFrames = std::move(weightFrames);
    _weightFields = std::move(weightFieldsOut);
    _providerXforms = std::move(providerXforms);
    _diagnostics = std::move(poseDiagnostics);
    _counters = counters;
    return true;
}

}  // namespace rigExec
