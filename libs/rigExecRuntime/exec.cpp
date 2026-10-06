// rigExecRuntime driver (M2 framework).
// The input API writes slot values; Execute applies the inputs set since
// the last run, runs the property chains, then the prologues over the
// file's static data, the region and the epilogue, then assembles the
// outputs.
// Failures name the step and keep the previous outputs.
#include "rigExecRuntime/runtime.h"
#include "stageArrayInputs.h"

#include "rigExecRuntime/labels.h"
#include "poseInternal.h"

#include <algorithm>

namespace rigExec {

namespace {

// The store's publication maps key by path id (bake insertion order);
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

bool
RigExecRuntimeStageArrayInputs::CanSample(const RigExecRuntimeReader &reader,
                                         size_t slot)
{
    const auto &slots = reader._program->inputState.stageArraySlots;
    return std::binary_search(slots.begin(), slots.end(), slot);
}

std::vector<RigExecStageArrayInputInfo>
RigExecRuntimeStageArrayInputs::Enumerate(const RigExecRuntimeReader &reader)
{
    std::vector<RigExecStageArrayInputInfo> result;
    const auto &file = *reader._program->inputState.file;
    for (size_t slot : RrStageArraySlots(reader._program.get())) {
        result.push_back({slot, reader._program->TextOrEmpty(file.inputs[slot].name()),
                          RrInputTag(uint8_t(file.inputs[slot].type()))});
    }
    return result;
}

bool
RigExecRuntimeStageArrayInputs::SetSample(RigExecRuntimeReader &reader,
    size_t slot, const RigExecRuntimeArray &value, std::string *error)
{
    return RrStageArraySet(reader._program.get(), slot, value, error);
}

bool
RigExecRuntimeStageArrayInputs::ClearSample(RigExecRuntimeReader &reader,
    size_t slot, std::string *error)
{
    return RrStageArrayClear(reader._program.get(), slot, error);
}

size_t
RigExecRuntimeReader::GetInputCount() const
{
    return _program->inputState.inputInfo.size();
}

const RigExecRuntimeInputInfo &
RigExecRuntimeReader::GetInputInfo(size_t index) const
{
    const auto &info = _program->inputState.inputInfo;
    return index < info.size() ? info[index] : _noInput;
}

bool
RigExecRuntimeReader::FindInput(const std::string &name, size_t *index) const
{
    const RrInputState &state = _program->inputState;
    const auto found = state.nameIndex.find(name);
    if (found == state.nameIndex.end() ||
        found->second >= state.inputInfo.size()) {
        return false;
    }
    if (index) {
        *index = found->second;
    }
    return true;
}

const RrInputValue &
RigExecRuntimeReader::GetInputValue(size_t index) const
{
    const auto &values = _program->inputState.inputValues;
    return index < values.size() ? values[index] : _noValue;
}

bool
RigExecRuntimeReader::SetInput(const std::string &name,
                               const RrInputValue &value, std::string *error)
{
    size_t index = 0;
    if (!FindInput(name, &index)) {
        if (error) {
            *error = "no input named " + name;
        }
        return false;
    }
    return RrInputsSet(_program.get(), index, value, error);
}

bool
RigExecRuntimeReader::SetInput(const std::string &name, double value,
                               std::string *error)
{
    RrInputValue held = _noValue;
    held.tag = RrInputTag::Double;
    held.f64 = value;
    return SetInput(name, held, error);
}

bool
RigExecRuntimeReader::SetInputToken(const std::string &name,
                                    const std::string &text,
                                    std::string *error)
{
    size_t index = 0;
    if (!FindInput(name, &index)) {
        if (error) {
            *error = "no input named " + name;
        }
        return false;
    }
    return RrInputsSetToken(_program.get(), index, text, error);
}

bool
RigExecRuntimeReader::SetInputAt(size_t index, const RrInputValue &value,
                                 std::string *error)
{
    return RrInputsSet(_program.get(), index, value, error);
}

bool
RigExecRuntimeReader::SetSampledInputAt(size_t index,
                                        const RrInputValue &value,
                                        std::string *error)
{
    return RrInputsSet(_program.get(), index, value, error,
                       /*acceptNonFinite=*/true);
}

bool
RigExecRuntimeReader::SetInputArray(const std::string &name,
                                    const RigExecRuntimeArray &value,
                                    std::string *error)
{
    size_t index = 0;
    if (!FindInput(name, &index)) {
        if (error) {
            *error = "no input named " + name;
        }
        return false;
    }
    return RrInputsSetArray(_program.get(), index, value, /*authored=*/true,
                            error);
}

bool
RigExecRuntimeReader::SetInputArrayAt(size_t index,
                                      const RigExecRuntimeArray &value,
                                      std::string *error)
{
    return RrInputsSetArray(_program.get(), index, value, /*authored=*/true,
                            error);
}

bool
RigExecRuntimeReader::SetSampledInputArrayAt(size_t index,
                                             const RigExecRuntimeArray &value,
                                             std::string *error)
{
    return RrInputsSetArray(_program.get(), index, value, /*authored=*/false,
                            error);
}

bool
RigExecRuntimeReader::GetInputArrayAt(size_t index,
                                      RigExecRuntimeArray *out) const
{
    return RrInputsGetArray(_program.get(), index, out);
}

bool
RigExecRuntimeReader::ClearInput(const std::string &name, std::string *error)
{
    size_t index = 0;
    if (!FindInput(name, &index)) {
        if (error) {
            *error = "no input named " + name;
        }
        return false;
    }
    return RrInputsClear(_program.get(), index, error);
}

bool
RigExecRuntimeReader::ClearInputAt(size_t index, std::string *error)
{
    return RrInputsClear(_program.get(), index, error);
}

bool
RigExecRuntimeReader::ResetInput(const std::string &name, std::string *error)
{
    size_t index = 0;
    if (!FindInput(name, &index)) {
        if (error) {
            *error = "no input named " + name;
        }
        return false;
    }
    RrInputsReset(_program.get(), index);
    return true;
}

void
RigExecRuntimeReader::ResetInputs()
{
    for (size_t i = 0; i < GetInputCount(); ++i) {
        RrInputsReset(_program.get(), i);
    }
}

void
RigExecRuntimeReader::TouchAnimatedInputs()
{
    _program->store.animatedTouched = true;
}

double
RigExecRuntimeReader::GetBakeTime() const
{
    return _program->file ? _program->file->bakeTime : 0.0;
}

std::string
RigExecRuntimeReader::GetTokenText(uint32_t token) const
{
    return _program->TextOrEmpty(token);
}

void
RigExecRuntimeReader::SetRunMaskForTesting(unsigned mask)
{
    _program->runMask = mask;
}

std::string
RigExecRuntimeReader::GetStepLabelForTesting(size_t step) const
{
    return RrStepLabel(*_program, step);
}

size_t
RigExecRuntimeReader::GetClosedClusterCountForTesting() const
{
    return _program->store.lastClosedClusters;
}

bool
RigExecRuntimeReader::GetSimdEnabledForTesting() const
{
    return _program->geoSettings.useSimd;
}

bool
RigExecRuntimeReader::GetPartitionStaleForTesting(
    const std::string &moverPath) const
{
    return RrGeometryPartitionStaleForTesting(_program.get(), moverPath);
}

bool
RigExecRuntimeReader::GetSkinLayoutIsOpenForTesting(
    const std::string &moverPath) const
{
    return RrGeometrySkinLayoutIsOpenForTesting(_program.get(), moverPath);
}

std::vector<int32_t>
RigExecRuntimeReader::GetLastRunTraceForTesting() const
{
    return _program->store.runTrace;
}

bool
RigExecRuntimeReader::GetStepRanForTesting(size_t step) const
{
    const RrStore &store = _program->store;
    if (step >= _program->steps->size() || !store.everRan) {
        return false;
    }
    const RigExecWireStep &wire = (*_program->steps)[step];
    if (wire.isHead)
        return std::find(store.runTrace.begin(), store.runTrace.end(), int32_t(step)) != store.runTrace.end();
    if (wire.isSource) {
        return true;
    }
    const size_t cluster = size_t(wire.cluster);
    return cluster / 64 < store.closedWords.size() &&
           ((store.closedWords[cluster / 64] >> (cluster % 64)) & 1u) != 0;
}

std::vector<RigExecRuntimeJointMatrix>
RigExecRuntimeReader::GetJointRestMatrices() const
{
    std::vector<RigExecRuntimeJointMatrix> out;
    const std::vector<RrPointFrame> &restFrames =
        RrPoseRestFrames(_program.get());
    const RigExecWireSlotMeta &meta = *_program->slotMeta;
    for (size_t i = 0; i < meta.jointSlots.size(); ++i) {
        const size_t slot = size_t(meta.jointSlots[i]);
        if (slot >= restFrames.size()) {
            continue;
        }
        RrMat4d rest;
        const RrPointFrame &frame = restFrames[slot];
        if (RrPointsToMatrix(RrIdentityLandmarks(), frame.points, &rest)) {
            out.push_back({_program->TextOrEmpty(meta.jointPaths[i]), rest});
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
RigExecRuntimeReader::GetPropertyValues() const
{
    std::vector<RigExecRuntimePropertyValue> out;
    out.reserve(_program->store.propertyResults.size());
    for (const auto &entry : _program->store.propertyResults) {
        out.push_back({_program->TextOrEmpty(entry.first), entry.second});
    }
    _SortByPath(&out);
    return out;
}

const std::vector<RrPointFrame> &
RigExecRuntimeReader::GetFinFrames() const
{
    return _program->store.fin;
}

const std::vector<RrPointFrame> &
RigExecRuntimeReader::GetBaseFrames() const
{
    return _program->store.base;
}

const std::vector<RrMat4d> &
RigExecRuntimeReader::GetFinalMatrices() const
{
    return _program->store.finalMatrix;
}

const std::vector<RrMat4d> &
RigExecRuntimeReader::GetBaseMatrices() const
{
    return _program->store.baseMatrix;
}

const std::vector<RrWeightPacket> &
RigExecRuntimeReader::GetWeightPackets() const
{
    return _program->store.weightPackets;
}

bool
RigExecRuntimeReader::Execute(std::string *error)
{
    RrProgram &program = *_program;
    RrStore &store = program.store;
    store.runTrace.clear();
    // The inputs set since the last run. Only an Animated input set, or
    // TouchAnimatedInputs, dirties what a change of time dirties.
    RrInputsApplyTouched(&program);

    // The property chains run first, as the baked prologue runs them: their
    // lines open the generation and their results are what every later
    // read of a chain target sees.
    std::vector<std::string> poseDiagnostics;
    if (!RrRunHeadSteps(&program, &poseDiagnostics, error)) {
        if (error) {
            *error = "the property chains disagree with the file";
        }
        return false;
    }

    if ((program.runMask & 0x1u) != 0 &&
        !RrProloguePose(&program, &poseDiagnostics, error)) {
        return false;
    }
    if ((program.runMask & 0x4u) != 0 &&
        !RrPrologueGeometry(&program, &poseDiagnostics, error)) {
        return false;
    }
    if (!RrRunSteps(&program, false, error)) {
        return false;
    }
    if (!RrPublishPose(&program, &poseDiagnostics, error)) {
        return false;
    }
    RrPublishGeometry(&program, &poseDiagnostics);

    RigExecRuntimeCounters counters;
    for (size_t s = 0; s < store.stepOutputs.size(); ++s) {
        const RrStepOutput &output = store.stepOutputs[s];
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

    // A plugin mover this runtime has no kernel for is a no-op, said on
    // every frame it is one, beside the lines of the step it sat in.
    for (const auto &[slot, index] : program.externalIndex) {
        const RrProgram::ExternalRevision &state = program.externals[index];
        if (state.state) {
            continue;
        }
        const RigExecWireRevision &wire =
            program.geometry->chains[slot.first].revisions[slot.second];
        poseDiagnostics.push_back(
            "warning: " + program.TextOrEmpty(wire.moverPath) + " is a " +
            state.type + ", which this runtime has no kernel for; its "
            "points pass through");
    }

    // The compile notices a fresh evaluator seeds its first generation
    // with (inert movers, purpose warnings): file order, ahead of
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
    // Every volume slot whose VolumePlacements step has run, under the
    // slot's path.
    std::vector<RigExecRuntimeWeightFrame> weightFrames;
    const size_t volumeSlots =
        std::min({program.slotMeta ? program.slotMeta->paths.size()
                                   : size_t(0),
                  store.volumePlaced.size(), store.volumePlacement.size()});
    for (size_t slot = 0; slot < volumeSlots; ++slot) {
        if (!store.volumePlaced[slot]) {
            continue;
        }
        RigExecRuntimeWeightFrame placed;
        placed.path = program.TextOrEmpty(program.slotMeta->paths[slot]);
        placed.matrix = store.volumePlacement[slot];
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
    _SortByPath(&matrixPrimvars);
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
    return true;
}

}  // namespace rigExec
