// Runtime driver: sample sources, execute the common graph, then publish outputs.
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

bool
RigExecRuntimeStageArrayInputs::CanSampleProviderValue(
    const RigExecRuntimeReader &reader, size_t slot)
{
    const auto &sources=reader._program->inputState.slotProviderSource;
    return slot<sources.size() && sources[slot];
}

bool
RigExecRuntimeStageArrayInputs::SetSampleBlocked(
    RigExecRuntimeReader &reader,size_t slot,bool blocked,std::string *error)
{
    return RrStageInputBlockedSet(reader._program.get(),slot,blocked,error);
}

std::vector<RigExecStageArrayInputInfo>
RigExecRuntimeStageArrayInputs::EnumerateProviderValues(
    const RigExecRuntimeReader &reader)
{
    std::vector<RigExecStageArrayInputInfo> result;
    const auto &state=reader._program->inputState;
    for (size_t slot=0;slot<state.slotProviderSource.size();++slot) {
        if (!state.slotProviderSource[slot]) continue;
        const auto &input=state.file->inputs[slot];
        const auto tag=RrInputTag(uint8_t(input.type()));
        const bool sourceBacked = (state.slotProviderSource[slot] & 2) != 0;
        if ((!sourceBacked && !(input.flags() & uint8_t(RigExecWireInputSlotFlags::Animated))) ||
            tag==RrInputTag::Token || RrInputTagIsArray(tag)) continue;
        result.push_back({slot,reader._program->TextOrEmpty(input.name()),tag,
                          (input.flags() & uint8_t(RigExecWireInputSlotFlags::Animated)) != 0});
    }
    return result;
}

bool
RigExecRuntimeStageArrayInputs::SetScalarSample(
    RigExecRuntimeReader &reader,size_t slot,const RrInputValue &value,std::string *error)
{
    return RrStageScalarSet(reader._program.get(),slot,value,error);
}

bool
RigExecRuntimeStageArrayInputs::ClearScalarSample(
    RigExecRuntimeReader &reader,size_t slot,std::string *error)
{
    return RrStageScalarClear(reader._program.get(),slot,error);
}

bool
RigExecRuntimeStageArrayInputs::CanSampleToken(
    const RigExecRuntimeReader &reader, size_t slot)
{
    const auto &slots = reader._program->inputState.stageTokenSlots;
    return std::binary_search(slots.begin(), slots.end(), slot);
}

std::vector<RigExecStageArrayInputInfo>
RigExecRuntimeStageArrayInputs::EnumerateTokens(const RigExecRuntimeReader &reader)
{
    std::vector<RigExecStageArrayInputInfo> result;
    const auto &state = reader._program->inputState;
    for (size_t slot : state.stageTokenSlots)
        result.push_back({slot, reader._program->TextOrEmpty(state.file->inputs[slot].name()),
                          RrInputTag::Token});
    return result;
}

bool
RigExecRuntimeStageArrayInputs::SetTokenSample(
    RigExecRuntimeReader &reader, size_t slot, const std::string &text,
    std::string *error)
{
    return RrStageTokenSet(reader._program.get(), slot, text, true, error);
}

bool
RigExecRuntimeStageArrayInputs::ClearTokenSample(
    RigExecRuntimeReader &reader, size_t slot, std::string *error)
{
    return RrStageTokenSet(reader._program.get(), slot, {}, false, error);
}

bool
RigExecRuntimeStageArrayInputs::SetTokenArraySample(
    RigExecRuntimeReader &reader, size_t slot,
    const std::vector<std::string> &texts, std::string *error)
{
    return RrStageTokenArraySet(reader._program.get(), slot, texts, error);
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

std::string
RigExecRuntimeReader::GetStepLabelForTesting(size_t step) const
{
    if (!_program->file || step >= _program->file->steps.size())
        return std::to_string(step);
    const auto &key = _program->file->steps[step].descriptorKey;
    // Compile appends this suffix to the exact native display label.
    const auto suffix = key.rfind("/category:");
    return suffix == std::string::npos ? key : key.substr(0, suffix);
}

size_t
RigExecRuntimeReader::GetClosedClusterCountForTesting() const
{
    return _program->store.lastClosedClusters;
}

bool
RigExecRuntimeReader::GetParallelSafeForTesting() const
{
    return _program->parallelSafe;
}

void
RigExecRuntimeReader::SetTaskDispatch(
    std::function<void(std::function<void()>)> dispatch,
    std::function<void()> wait)
{
    RrStore &store = _program->store;
    // Both or neither: a dispatch without its join could return from
    // Execute before the bodies finished.
    if (!dispatch || !wait) {
        store.dispatch = nullptr;
        store.wait = nullptr;
        return;
    }
    store.dispatch = std::move(dispatch);
    store.wait = std::move(wait);
}

size_t
RigExecRuntimeReader::GetSlotLeafKeysForTesting() const
{
    return _program->store.slotLeafKeys;
}

size_t
RigExecRuntimeReader::GetSlotLeafCountForTesting(size_t slot) const
{
    const std::vector<uint32_t> &begin = _program->slotLeafBegin;
    return slot + 1 < begin.size() ? begin[slot + 1] - begin[slot] : 0;
}

size_t
RigExecRuntimeReader::GetSourceKeysBuiltForTesting() const
{
    return _program->store.sourceKeysBuilt;
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
RigExecRuntimeReader::GetRevisionDecisionForTesting(
    const std::string &moverPath, int *acceptance, bool *chunksOk) const
{
    return acceptance && chunksOk &&
           RrGeometryRevisionDecisionForTesting(_program.get(), moverPath,
                                                acceptance, chunksOk);
}

bool
RigExecRuntimeReader::GetRangeRoleForTesting(const std::string &moverPath,
                                             bool *rangeRole,
                                             bool *ownSource) const
{
    return RrGeometryRangeRoleForTesting(_program.get(), moverPath, rangeRole,
                                         ownSource);
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

std::vector<std::string>
RigExecRuntimeReader::GetComposeMovesForTesting(bool ladder) const
{
    const RrStore &store = _program->store;
    const std::vector<char> &changed =
        ladder ? store.ladderChanged : store.restChanged;
    const RigExecWireSlotMeta &meta = *_program->slotMeta;
    std::vector<std::string> out;
    for (size_t slot = 0; slot < changed.size() && slot < meta.paths.size();
         ++slot) {
        if (changed[slot]) {
            out.push_back(_program->TextOrEmpty(meta.paths[slot]));
        }
    }
    return out;
}

bool
RigExecRuntimeReader::GetStepRanForTesting(size_t step) const
{
    const RrStore &store = _program->store;
    if (step >= _program->steps->size() || !store.everRan) {
        return false;
    }
    return step < store.opExecution.ran.size() && store.opExecution.ran[step];
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
RigExecRuntimeReader::SampleGeometryForTesting(std::string *error)
{
    std::vector<std::string> diagnostics;
    return RrPrologueGeometry(_program.get(), &diagnostics, error);
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

    std::vector<std::string> poseDiagnostics;
    RrProloguePose(&program);
    if (!RrPrologueGeometry(&program, &poseDiagnostics, error)) return false;
    const bool ran = RrRunSteps(&program, false, error);
    // Joined: the binds this run built share now, not at the next run.
    RrShareLatticeBinds(&program);
    if (!ran) {
        return false;
    }

    if(!program.requiredStageFramesAdmission.admitted) {
        std::vector<size_t> propertySteps;
        for(size_t i=0;i<program.steps->size();++i)
            if((*program.steps)[i].kind==RigExecWireStepKind::PropertyRevision)propertySteps.push_back(i);
        std::sort(propertySteps.begin(),propertySteps.end(),[&](size_t a,size_t b) {
            const auto &left=(*program.steps)[a]; const auto &right=(*program.steps)[b];
            return std::make_pair(left.object,left.part)<std::make_pair(right.object,right.part);
        });
        for(size_t i:propertySteps) {
            poseDiagnostics.insert(poseDiagnostics.end(),store.headLines[i].begin(),store.headLines[i].end());
        }
        const size_t target=size_t(program.requiredStageFramesAdmission.firstBadTarget);
        const size_t slot=size_t(program.slotMeta->xformSlots[target]);
        const std::string refusal="could not resolve constraint target "+
            program.TextOrEmpty(program.slotMeta->paths[slot])+" relative to the asset root";
        poseDiagnostics.push_back(refusal);
        _jointMatrices.clear(); _points.clear(); _matrixPrimvars.clear();
        _weightFrames.clear(); _weightFields.clear(); _providerXforms.clear();
        _diagnostics=std::move(poseDiagnostics);
        _counters=RigExecRuntimeCounters();
        _counters.executedOpCount=store.opExecution.executed;
        if(error)*error=refusal;
        return false;
    }

    const size_t epilogueMismatches = program.epilogue.mismatches;
    if (!RrPublishPose(&program, &poseDiagnostics, error)) {
        return false;
    }
    RrPublishGeometry(&program, &poseDiagnostics);
    // RIGEXEC_VERIFY_EPILOGUE_LISTS: lines the held steps published
    // differently from a sweep of every step fail the run.
    if (program.epilogue.mismatches != epilogueMismatches) {
        if (error) {
            *error = "the epilogue's held steps and a sweep of every "
                     "step published different lines";
        }
        return false;
    }

    for (const auto &loop : program.opGraph.cycles) {
        std::string message = "operation cycle: ";
        for (size_t i = 0; i < loop.size(); ++i) {
            if (i) message += " -> ";
            message += loop[i];
        }
        if (std::find(poseDiagnostics.begin(), poseDiagnostics.end(), message) == poseDiagnostics.end())
            poseDiagnostics.push_back(std::move(message));
    }

    RigExecRuntimeCounters counters;
    counters.executedOpCount=store.opExecution.executed;

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
        moved.points = RigExecSharedArray<RrVec3f>(entry.second.Share());
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
        field.weights =
            RigExecSharedArray<float>(entry.second.weights.Share());
        weightFieldsOut.push_back(std::move(field));
    }
    // The outputs share the retained buffers. Clearing the maps drops the
    // epilogue's extra retainers. A buffer this reader still owns alone is
    // returned to the spare pool; the chain has already published the new
    // one. A share the caller kept is left alone, and the next fill of that
    // array allocates.
    store.movedProperties.clear();
    store.weightFields.clear();
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
    for (RigExecRuntimePoints &old : _points) {
        RrRetainedRecycle(old.points.Release());
    }
    for (RigExecRuntimeWeightField &old : _weightFields) {
        RrRetainedRecycle(old.weights.Release());
    }
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
