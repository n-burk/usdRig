// rigExecRuntime frame driver (M2 framework).
// SetFrame loads a baked frame's sparse values into the holders;
// Execute runs prologues, the region and the epilogue, then assembles
// the outputs. Failures name the step and keep the previous outputs.
#include "rigExecRuntime/runtime.h"

#include "rigExecMath/propertyMathKernel.h"

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
    // The record writes only what it carries; a displaced constant comes
    // back first, and Execute overrides it again.
    _RestoreInputs();
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
RigExecRuntimeReader::ClearAvars()
{
    _avarOverrides.clear();
    _pathOverrides.clear();
    _inputOverrides.clear();
    _RestoreInputs();
    std::fill(_program.store.overridden.begin(),
              _program.store.overridden.end(), 0);
}

void
RigExecRuntimeReader::_RestoreInputs()
{
    for (const auto &entry : _inputBase) {
        if (entry.first < _program.store.inputHolders.size()) {
            _program.store.inputHolders[entry.first] = entry.second;
        }
    }
    _inputBase.clear();
}

bool
RigExecRuntimeReader::SetAvar(const std::string &path, double value,
                            std::string *error)
{
    // An input the property chains read: held by path, read by the chains'
    // walks. A TRS control avar can be both, so this falls through to the
    // slot override below when it is.
    bool chainInput = false;
    if (_hasPropertyChains && std::isfinite(value)) {
        const auto it = _chainInputPaths.find(path);
        if (it != _chainInputPaths.end()) {
            _pathOverrides[it->second] = value;
            chainInput = true;
        }
    }
    const size_t dot = path.find_last_of('.');
    const auto slot = _program.pathIndex.find(path.substr(0, dot));
    int channel = -1;
    if (dot != std::string::npos) {
        for (int i = 0; i < 9; ++i) {
            if (path.substr(dot + 1) == RrAvarNames[i]) channel = i;
        }
    }
    if (!std::isfinite(value) || slot == _program.pathIndex.end() ||
        channel < 0 ||
        _slotMeta.slotKind[size_t(slot->second)] !=
            RigExecWireSlotKind::FirstFramePose ||
        _constants.posedAuthored[size_t(slot->second)] ||
        std::find(_slotMeta.controlSlots.begin(), _slotMeta.controlSlots.end(),
                  slot->second) == _slotMeta.controlSlots.end() ||
        (channel >= 3 && channel <= 5 &&
         _constants.noScaleAvars[size_t(slot->second)])) {
        if (std::isfinite(value) &&
            _overridableInputs.count(path) != 0) {
            _inputOverrides[path] = value;
            return true;
        }
        if (chainInput) {
            return true;
        }
        if (error) *error = "expected a finite TRS avar on a compiled pose slot: " + path;
        return false;
    }
    for (const auto &frame : _inputs.frames) {
        for (uint32_t id : frame.propertyPaths) {
            if (_program.TextOrEmpty(id) == path) {
                if (error) *error = "cannot override a captured property-mover output: " + path;
                return false;
            }
        }
    }
    _avarOverrides[size_t(slot->second) * 11 + size_t(channel)] = value;
    return true;
}

void
RigExecRuntimeReader::SetRunMaskForTesting(unsigned mask)
{
    _program.runMask = mask;
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

const std::vector<RrPointFrame> &
RigExecRuntimeReader::GetFinFrames() const
{
    return _program.store.fin;
}

bool
RigExecRuntimeReader::GetControlFrame(const std::string &primPath,
                                      double out[16]) const
{
    // The slot's last version: its base frame for an animator input, the
    // revised one for a control a constraint names (bakedPose.cpp).
    const RrStore &store = _program.store;
    const auto slot = _program.pathIndex.find(primPath);
    if (slot == _program.pathIndex.end() ||
        size_t(slot->second) >= store.finLast.size() ||
        size_t(store.finLast[size_t(slot->second)]) >= store.fin.size()) {
        return false;
    }
    const RrPointFrame &frame =
        store.fin[size_t(store.finLast[size_t(slot->second)])];
    const RrVec3d &origin = frame.points[0];
    for (int r = 0; r < 3; ++r) {
        for (int k = 0; k < 3; ++k) {
            out[r * 4 + k] = frame.points[size_t(r + 1)][k] - origin[k];
        }
        out[r * 4 + 3] = 0.0;
    }
    for (int k = 0; k < 3; ++k) {
        out[12 + k] = origin[k];
    }
    out[15] = 1.0;
    return true;
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
    // The chains the file carries as programs replace their recorded
    // values with ones computed from this run's inputs.
    if (_hasPropertyChains) {
        _RunPropertyChains();
    }
    store.runSnapshots.Clear();

    // Restore constant channels as well as sampled ones, so clearing an
    // override returns to the baked pose on the very next evaluation.
    store.avars = _constants.avarConstants;

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
    for (const auto &entry : _avarOverrides) {
        store.avars[entry.first] = entry.second;
    }
    // Inputs set by path: each holder takes the value in its own type and
    // its override index marks the steps that read it for this run.
    for (const auto &entry : _inputOverrides) {
        for (const auto &target : _overridableInputs[entry.first]) {
            if (target.first >= store.inputHolders.size()) {
                continue;
            }
            RrInputValue &holder = store.inputHolders[target.first];
            _inputBase.emplace(target.first, holder);
            switch (holder.tag) {
            case RigExecWireInput::Tag::Double: holder.f64 = entry.second; break;
            case RigExecWireInput::Tag::Float: holder.f32 = float(entry.second); break;
            case RigExecWireInput::Tag::Int: holder.i32 = int32_t(std::lround(entry.second)); break;
            case RigExecWireInput::Tag::Bool: holder.boolean = entry.second != 0.0; break;
            default: continue;
            }
            if (size_t(target.second) < store.overridden.size()) {
                store.overridden[size_t(target.second)] = 1;
            }
        }
    }
    // Chain results computed this run reach holders the closure does not
    // track by value, so a run whose chains moved runs every step.
    const bool chainsMoved = _hasPropertyChains &&
                             store.propertyResults != store.lastPropertyResults;
    if (!RrRunSteps(&program, time, chainsMoved, error)) {
        return false;
    }
    if (!RrPublishPose(&program, &poseDiagnostics, error)) {
        return false;
    }
    RrPublishGeometry(&program, record, &poseDiagnostics);

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
    return true;
}

namespace {

// A value a chain input's walk finds live at `hop`: one SetAvar set, or a
// chain result published earlier in this run.
bool
_LiveHop(const std::map<uint32_t, double> &overrides,
         const std::map<uint32_t, RrPropertyValue> &results, uint32_t hop,
         double *out)
{
    const auto set = overrides.find(hop);
    if (set != overrides.end()) {
        *out = set->second;
        return true;
    }
    const auto result = results.find(hop);
    if (result != results.end()) {
        if (result->second.tag == RrPropertyValue::Tag::Float) {
            *out = double(result->second.f32);
            return true;
        }
        if (result->second.tag == RrPropertyValue::Tag::Double) {
            *out = result->second.f64;
            return true;
        }
    }
    return false;
}

float
_ReadFloat(const RigExecWirePropertyChainInput &input,
           const std::map<uint32_t, double> &overrides,
           const std::map<uint32_t, RrPropertyValue> &results, size_t frame,
           float fallback)
{
    switch (input.kind) {
    case RigExecWirePropertyChainInput::Kind::Absent:
        return fallback;
    case RigExecWirePropertyChainInput::Kind::Constant:
        return float(input.constant);
    case RigExecWirePropertyChainInput::Kind::Walk:
        break;
    }
    for (uint32_t hop : input.hops) {
        double live = 0.0;
        if (_LiveHop(overrides, results, hop, &live)) {
            return float(live);
        }
    }
    if (frame < input.frameValues.size() && input.frameHave[frame]) {
        return float(input.frameValues[frame]);
    }
    return fallback;
}

bool
_ReadBool(const RigExecWirePropertyChainInput &input, size_t frame, bool fallback)
{
    switch (input.kind) {
    case RigExecWirePropertyChainInput::Kind::Absent:
        return fallback;
    case RigExecWirePropertyChainInput::Kind::Constant:
        return input.constant != 0.0;
    case RigExecWirePropertyChainInput::Kind::Walk:
        break;
    }
    // No chain publishes a bool and SetAvar holds doubles, so a bool read
    // never meets a live value of its type: the recorded fallback answers.
    if (frame < input.frameValues.size() && input.frameHave[frame]) {
        return input.frameValues[frame] != 0.0;
    }
    return fallback;
}

}  // namespace

std::map<std::string, double>
RigExecRuntimeReader::GetPropertyResults() const
{
    std::map<std::string, double> out;
    for (const auto &[path, value] : _program.store.propertyResults) {
        if (value.tag == RrPropertyValue::Tag::Float) {
            out[_program.TextOrEmpty(path)] = double(value.f32);
        } else if (value.tag == RrPropertyValue::Tag::Double) {
            out[_program.TextOrEmpty(path)] = value.f64;
        }
    }
    return out;
}

void
RigExecRuntimeReader::_RunPropertyChains()
{
    RrStore &store = _program.store;
    std::map<uint32_t, RrPropertyValue> &results = store.propertyResults;
    const size_t frame = _frameIndex;
    // RigExecRigEvaluator::_EvaluatePropertyChains, revision for revision:
    // the base is the target's authored value, each enabled revision with a
    // usable envelope and finite inputs applies, and a non-finite result
    // passes through.
    // One revision applied to `in`: false when it passes through (disabled,
    // unusable envelope or inputs), leaving the caller's value untouched.
    const auto apply = [&](const RigExecWirePropertyChainRevision &revision,
                           float in, float *out) {
        if (!_ReadBool(revision.enabled, frame, true)) {
            return false;
        }
        const float envelope = _ReadFloat(revision.defaultWeight,
                                          _pathOverrides, results, frame,
                                          1.0f);
        if (!std::isfinite(envelope) || envelope < 0.0f ||
            envelope > 1.0f) {
            return false;
        }
        const float amount = _ReadFloat(revision.value, _pathOverrides,
                                        results, frame, 0.0f);
        const float low = _ReadFloat(revision.minimum, _pathOverrides,
                                     results, frame, 0.0f);
        const float high = _ReadFloat(revision.maximum, _pathOverrides,
                                      results, frame, 0.0f);
        if (!std::isfinite(amount) || !std::isfinite(low) ||
            !std::isfinite(high)) {
            return false;
        }
        const RigExecPropertyOp op = RigExecPropertyOp(revision.op);
        const RigExecWireVec2f *keys = nullptr;
        const RigExecWireVec2f *tangents = nullptr;
        size_t keyCount = 0;
        size_t tangentCount = 0;
        if (op == RigExecPropertyOp::Curve) {
            keyCount = revision.keys.size();
            if (keyCount == 0 ||
                !propertyMathKernel::ValidateLinearKeys(revision.keys.data(),
                                                        keyCount)) {
                return false;
            }
            keys = revision.keys.data();
            if (!revision.tangents.empty()) {
                if (revision.tangents.size() != keyCount) {
                    return false;
                }
                tangents = revision.tangents.data();
                tangentCount = revision.tangents.size();
            }
        }
        *out = propertyMathKernel::ApplyFloatMath(
            in, op, amount, low, high, envelope, keys, keyCount, tangents,
            tangentCount);
        return true;
    };
    for (const RigExecWirePropertyChain &chain : _chains.chains) {
        if (frame >= chain.frameBase.size() || !chain.frameBaseHave[frame]) {
            results.erase(chain.target);
            continue;
        }
        RrPropertyValue published;
        if (chain.valueType == RigExecWirePropertyChain::ValueType::Float) {
            const float base = float(chain.frameBase[frame]);
            if (!std::isfinite(base)) {
                results.erase(chain.target);
                continue;
            }
            float value = base;
            for (const RigExecWirePropertyChainRevision &revision : chain.revisions) {
                float next = value;
                if (apply(revision, value, &next) && std::isfinite(next)) {
                    value = next;
                }
            }
            published.tag = RrPropertyValue::Tag::Float;
            published.f32 = value;
        } else {
            // The double chain runs each revision in float and widens the
            // result, as the evaluator's double instantiation does.
            double value = chain.frameBase[frame];
            if (!std::isfinite(value)) {
                results.erase(chain.target);
                continue;
            }
            for (const RigExecWirePropertyChainRevision &revision : chain.revisions) {
                float result = 0.0f;
                if (apply(revision, float(value), &result)) {
                    const double next = double(result);
                    if (std::isfinite(next)) {
                        value = next;
                    }
                }
            }
            published.tag = RrPropertyValue::Tag::Double;
            published.f64 = value;
        }
        results[chain.target] = published;
    }
    // Each result to the holders it feeds, as the evaluator's read of the
    // input would see it: a float read of a double result is cast; a
    // double read of a float result never matches the overlay and keeps
    // the recorded stage value.
    for (const RigExecWirePropertyChainConsumer &consumer : _chains.consumers) {
        if (consumer.uid >= store.inputHolders.size()) {
            continue;
        }
        const auto it = results.find(_chains.chains[consumer.chain].target);
        if (it == results.end()) {
            continue;
        }
        RrInputValue &holder = store.inputHolders[consumer.uid];
        if (holder.tag == RigExecWireInput::Tag::Double &&
            it->second.tag == RrPropertyValue::Tag::Double) {
            holder.f64 = it->second.f64;
        } else if (holder.tag == RigExecWireInput::Tag::Float) {
            if (it->second.tag == RrPropertyValue::Tag::Float) {
                holder.f32 = it->second.f32;
            } else if (it->second.tag == RrPropertyValue::Tag::Double) {
                holder.f32 = float(it->second.f64);
            }
        }
    }
}

}  // namespace rigExec
