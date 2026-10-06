// Shared helpers for the suites that drive a .rigexec through the runtime:
// bake at one time, play the binary along the stage's timeline through the
// input sampler, author the reference value of an input drag in the session
// layer, and compare a runtime reader's last run with a pose the evaluator
// published, output domain by output domain, bit for bit, the way
// rigExecPose --verify-binary compares them; and set array inputs (authored
// or sampled) beside their session-layer references.
#ifndef RIGEXEC_TEST_RUNTIME_DRIVE_H
#define RIGEXEC_TEST_RUNTIME_DRIVE_H

#include "rigExecBake/bake.h"
#include "rigExec/rigEvaluator.h"
#include "rigExecRuntime/runtime.h"
#include "rigExecSampler/inputSampler.h"

#include "pxr/base/gf/matrix4d.h"
#include "pxr/base/gf/vec3f.h"
#include "pxr/base/tf/type.h"
#include "pxr/base/vt/array.h"
#include "pxr/base/vt/types.h"
#include "pxr/base/vt/value.h"
#include "pxr/usd/sdf/layer.h"
#include "pxr/usd/usd/attribute.h"
#include "pxr/usd/usd/editContext.h"
#include "pxr/usd/usd/stage.h"
#include "pxr/usd/usd/timeCode.h"

#include <algorithm>
#include <cstdint>
#include <cstring>
#include <functional>
#include <map>
#include <memory>
#include <string>
#include <utility>
#include <vector>

/// Bakes \p evaluator's epoch at \p time (NaN: the probe time) into
/// \p bytes. False with the bake's reason.
inline bool
RigExecTestBakeAt(rigExec::RigExecRigEvaluator &evaluator, double time,
                  std::vector<uint8_t> *bytes, std::string *error)
{
    rigExec::RigExecBakeOpts opts;
    opts.time = time;
    rigExec::RigExecBakeResult result;
    if (!rigExec::RigExecBakeToBinary(evaluator, opts, &result, error)) {
        return false;
    }
    *bytes = std::move(result.bytes);
    return true;
}

/// One step of playback over a stage: \p sampler hands \p reader the
/// stage's values of its Animated inputs at \p time (no call when the time
/// is the last one), then the reader runs. False with the first reason.
inline bool
RigExecTestDrive(rigExec::RigExecRuntimeReader *reader,
                 rigExec::RigExecInputSampler *sampler, double time,
                 std::string *error)
{
    return sampler->Apply(PXR_NS::UsdTimeCode(time), reader, error) &&
           reader->Execute(error);
}

/// \p value typed to \p attribute the way SetInput(name, value) sets a
/// Double or a Float input: a double attribute takes it as is, a float one
/// static_cast<float>(value). Empty for any other attribute type.
inline PXR_NS::VtValue
RigExecTestTypedValue(const PXR_NS::UsdAttribute &attribute, double value)
{
    if (!attribute) {
        return PXR_NS::VtValue();
    }
    const PXR_NS::TfType type = attribute.GetTypeName().GetType();
    if (type == PXR_NS::TfType::Find<double>()) {
        return PXR_NS::VtValue(value);
    }
    if (type == PXR_NS::TfType::Find<float>()) {
        return PXR_NS::VtValue(static_cast<float>(value));
    }
    return PXR_NS::VtValue();
}

/// A value authored on an attribute in its stage's session layer: the
/// evaluator's reference for the same value set on the binary as an input
/// (an authored value). Restore, or the destructor, puts the session layer
/// back exactly as it was before the edit -- not merely cleared, since a
/// spec left behind changes how an evaluator takes a later edit. Edits
/// alive together restore in reverse order of construction.
class RigExecTestSessionEdit
{
public:
    /// Typed to the attribute (RigExecTestTypedValue); any other attribute
    /// type is refused (IsSet false).
    RigExecTestSessionEdit(const PXR_NS::UsdAttribute &attribute,
                           double value)
        : _attribute(attribute)
    {
        _Set(RigExecTestTypedValue(attribute, value));
    }
    /// \p value as given, which must hold the attribute's own type.
    RigExecTestSessionEdit(const PXR_NS::UsdAttribute &attribute,
                           const PXR_NS::VtValue &value)
        : _attribute(attribute)
    {
        _Set(value);
    }
    ~RigExecTestSessionEdit() { Restore(); }

    RigExecTestSessionEdit(const RigExecTestSessionEdit &) = delete;
    RigExecTestSessionEdit &operator=(const RigExecTestSessionEdit &) =
        delete;

    bool IsSet() const { return _set; }

    /// The session layer as it was before this edit.
    void Restore()
    {
        if (!_saved) {
            return;
        }
        _set = false;
        const PXR_NS::UsdStagePtr stage = _attribute.GetStage();
        if (stage) {
            stage->GetSessionLayer()->TransferContent(_saved);
        }
        _saved = PXR_NS::SdfLayerRefPtr();
    }

private:
    void _Set(const PXR_NS::VtValue &value)
    {
        const PXR_NS::UsdStagePtr stage =
            _attribute ? _attribute.GetStage() : PXR_NS::UsdStagePtr();
        if (!stage || value.IsEmpty()) {
            return;
        }
        _saved = PXR_NS::SdfLayer::CreateAnonymous();
        _saved->TransferContent(stage->GetSessionLayer());
        PXR_NS::UsdEditContext context(stage, stage->GetSessionLayer());
        _set = _attribute.Set(value);
    }

    PXR_NS::UsdAttribute _attribute;
    PXR_NS::SdfLayerRefPtr _saved;
    bool _set = false;
};

/// One value a reference evaluation authors: \p value (the attribute's
/// own type) on the attribute at \p path.
struct RigExecTestEdit {
    PXR_NS::SdfPath path;
    PXR_NS::VtValue value;
};

/// The reference for a binary driven with inputs set: a fresh evaluator in
/// \p mode, compiled on \p stage as it stands (the stage the binary was
/// baked from), then handed \p edits in the session layer, evaluates each
/// of \p times. With \p compileAfterEdits it compiles once the edits stand
/// instead, for an edit the evaluator's value-edit path does not take. \p
/// each, when set, sees the evaluator right after each Evaluate (to
/// snapshot its program). The session layer is put back exactly
/// afterwards. False with the reason when the compile fails, an edit names
/// no attribute or does not author, or a pose is invalid.
inline bool
RigExecTestEditedPoses(
    const PXR_NS::UsdStageRefPtr &stage, const PXR_NS::SdfPath &rigPath,
    rigExec::RigExecEvaluationMode mode,
    const std::vector<RigExecTestEdit> &edits,
    const std::vector<double> &times,
    std::vector<rigExec::RigExecRigPose> *poses, std::string *error,
    const std::function<void(const rigExec::RigExecRigEvaluator &, size_t)>
        &each = {},
    bool compileAfterEdits = false)
{
    rigExec::RigExecRigEvaluator evaluator(stage, rigPath);
    evaluator.SetEvaluationMode(mode);
    std::vector<std::string> notices;
    if (!compileAfterEdits && !evaluator.Compile(&notices)) {
        *error = "the reference did not compile";
        return false;
    }
    std::vector<std::unique_ptr<RigExecTestSessionEdit>> held;
    for (const RigExecTestEdit &edit : edits) {
        const PXR_NS::UsdAttribute attribute =
            stage->GetAttributeAtPath(edit.path);
        held.push_back(
            std::make_unique<RigExecTestSessionEdit>(attribute, edit.value));
        if (!held.back()->IsSet()) {
            *error = "the session layer refused " + edit.path.GetString();
            while (!held.empty()) {
                held.pop_back();
            }
            return false;
        }
    }
    if (compileAfterEdits && !evaluator.Compile(&notices)) {
        *error = "the reference did not compile";
        while (!held.empty()) {
            held.pop_back();
        }
        return false;
    }
    bool ok = true;
    for (size_t i = 0; i < times.size(); ++i) {
        rigExec::RigExecRigPose pose =
            evaluator.Evaluate(PXR_NS::UsdTimeCode(times[i]));
        if (!pose.valid || pose.bakedParityMismatches != 0 ||
            pose.moverGraphParityMismatches != 0) {
            *error = "the reference pose at " + std::to_string(times[i]) +
                     " is invalid";
            ok = false;
        }
        if (each) {
            each(evaluator, i);
        }
        poses->push_back(std::move(pose));
    }
    // Last in, first out: the session layer as it was before the first.
    while (!held.empty()) {
        held.pop_back();
    }
    return ok;
}

/// A reader played over the stage it was baked from, as a host plays it:
/// the input sampler hands it the stage's Animated inputs at each new time,
/// and the inputs a drag holds are set again after each sample, so a held
/// drag stands at every time as an authored value does.
class RigExecTestPlayer
{
public:
    /// Opens \p bytes and binds the sampler to \p stage. False with the
    /// reason, also when the sampler cannot resolve an input there.
    bool Open(const std::vector<uint8_t> &bytes,
              const PXR_NS::UsdStagePtr &stage, std::string *error)
    {
        _reader = rigExec::RigExecRuntimeReader::Open(bytes.data(),
                                                      bytes.size(), error);
        if (!_reader || !_sampler.Bind(stage, *_reader, error)) {
            _reader.reset();
            return false;
        }
        if (!_sampler.GetWarnings().empty()) {
            *error = "sampler: " + _sampler.GetWarnings().front();
            _reader.reset();
            return false;
        }
        _held.clear();
        return true;
    }

    rigExec::RigExecRuntimeReader &Reader() { return *_reader; }
    const rigExec::RigExecRuntimeReader &Reader() const { return *_reader; }
    rigExec::RigExecRuntimeReader *operator->() { return _reader.get(); }
    const rigExec::RigExecInputSampler &Sampler() const { return _sampler; }

    /// The sampler at \p time, the held inputs set again where it sampled,
    /// then Execute. False with the first reason.
    bool Play(double time, std::string *error)
    {
        const bool moved = !_played || time != _time;
        if (!_sampler.Apply(PXR_NS::UsdTimeCode(time), _reader.get(),
                            error)) {
            return false;
        }
        if (moved) {
            for (const auto &[name, value] : _held) {
                size_t index = 0;
                if (_reader->FindInput(name, &index) &&
                    _reader->GetInputInfo(index).animated &&
                    !_reader->SetInput(name, value, error)) {
                    return false;
                }
            }
        }
        _played = true;
        _time = time;
        return _reader->Execute(error);
    }

    /// Sets input \p name to \p value and holds it there over later times.
    bool Hold(const std::string &name, double value, std::string *error)
    {
        if (!_reader->SetInput(name, value, error)) {
            return false;
        }
        _held[name] = value;
        return true;
    }

    /// Lets every held input go: each returns to its default, and the next
    /// Play samples the stage again whatever its time, so an Animated one
    /// takes the stage's value at that time.
    void ReleaseAll()
    {
        for (const auto &entry : _held) {
            _reader->ResetInput(entry.first, nullptr);
        }
        _held.clear();
        _sampler.Invalidate();
    }

private:
    std::unique_ptr<rigExec::RigExecRuntimeReader> _reader;
    rigExec::RigExecInputSampler _sampler;
    std::map<std::string, double> _held;
    double _time = 0.0;
    bool _played = false;
};

namespace rigExecTestDrive {

inline void
Push(std::vector<std::string> *diffs, const std::string &line)
{
    if (diffs->size() < 12) {
        diffs->push_back(line);
    }
}

inline bool
SameMatrix(const PXR_NS::GfMatrix4d &a, const rigExec::RrMat4d &b)
{
    for (int r = 0; r < 4; ++r) {
        for (int c = 0; c < 4; ++c) {
            if (a[r][c] != b[r][c]) {
                return false;
            }
        }
    }
    return true;
}

}  // namespace rigExecTestDrive

/// Compares \p reader's last run with \p pose in the output domains alone:
/// joint matrices, moved point arrays (a moved scalar or matrix is outside
/// the runtime's output contract and is skipped, as verify-binary skips
/// it), `primvars:` matrices, weight frames, weight fields, and provider
/// transforms and their bases. Appends at most twelve lines to \p diffs;
/// true when nothing differs.
inline bool
RigExecCompareRuntimeOutputs(const rigExec::RigExecRigPose &pose,
                             const rigExec::RigExecRuntimeReader &reader,
                             std::vector<std::string> *diffs)
{
    using rigExecTestDrive::SameMatrix;
    using PXR_NS::GfMatrix4d;
    using PXR_NS::VtVec3fArray;
    bool same = true;
    const auto push = [&](const std::string &line) {
        same = false;
        rigExecTestDrive::Push(diffs, line);
    };
    {
        const auto &rt = reader.GetJointMatrices();
        if (pose.jointMatricesFinal.size() != rt.size()) {
            push("joint count baked " +
                     std::to_string(pose.jointMatricesFinal.size()) +
                     " binary " + std::to_string(rt.size()));
        }
        std::map<std::string, const rigExec::RrMat4d *> byPath;
        for (const auto &joint : rt) {
            byPath[joint.path] = &joint.matrix;
        }
        for (const auto &[path, matrix] : pose.jointMatricesFinal) {
            const auto found = byPath.find(path.GetString());
            if (found == byPath.end()) {
                push("joint " + path.GetString() +
                         " missing from the binary");
            } else if (!SameMatrix(matrix, *found->second)) {
                push("joint " + path.GetString() + " differs");
            }
        }
    }
    {
        std::map<std::string, const std::vector<rigExec::RrVec3f> *> byPath;
        for (const auto &moved : reader.GetPoints()) {
            byPath[moved.path] = &moved.points;
        }
        size_t arrays = 0;
        for (const auto &[path, value] : pose.movedProperties) {
            if (!value.IsHolding<VtVec3fArray>()) {
                continue;
            }
            ++arrays;
            const auto found = byPath.find(path.GetString());
            if (found == byPath.end()) {
                push("moved " + path.GetString() +
                         " missing from the binary");
                continue;
            }
            const VtVec3fArray &array = value.UncheckedGet<VtVec3fArray>();
            if (array.size() != found->second->size()) {
                push("moved " + path.GetString() + " count " +
                         std::to_string(array.size()) + " vs " +
                         std::to_string(found->second->size()));
                continue;
            }
            for (size_t i = 0; i < array.size(); ++i) {
                const rigExec::RrVec3f &b = (*found->second)[i];
                if (array[i][0] != b[0] || array[i][1] != b[1] ||
                    array[i][2] != b[2]) {
                    push("moved " + path.GetString() + " point " +
                             std::to_string(i) + " differs");
                    break;
                }
            }
        }
        if (arrays != byPath.size()) {
            push("moved-property count baked " +
                     std::to_string(arrays) + " binary " +
                     std::to_string(byPath.size()));
        }
    }
    {
        std::map<std::string, const rigExec::RrMat4d *> byPath;
        for (const auto &primvar : reader.GetMatrixPrimvars()) {
            byPath[primvar.path] = &primvar.matrix;
        }
        const std::string prefix = "primvars:";
        size_t matrices = 0;
        for (const auto &[path, value] : pose.movedProperties) {
            if (!value.IsHolding<GfMatrix4d>() ||
                path.GetName().compare(0, prefix.size(), prefix) != 0) {
                continue;
            }
            ++matrices;
            const auto found = byPath.find(path.GetString());
            if (found == byPath.end()) {
                push("matrix primvar " + path.GetString() +
                         " missing from the binary");
            } else if (!SameMatrix(value.UncheckedGet<GfMatrix4d>(),
                                   *found->second)) {
                push("matrix primvar " + path.GetString() +
                         " differs");
            }
        }
        if (matrices != byPath.size()) {
            push("matrix primvar count baked " +
                     std::to_string(matrices) + " binary " +
                     std::to_string(byPath.size()));
        }
    }
    {
        std::map<std::string, const rigExec::RrMat4d *> byPath;
        for (const auto &placed : reader.GetWeightFrames()) {
            byPath[placed.path] = &placed.matrix;
        }
        if (pose.weightFrames.size() != byPath.size()) {
            push("weight-frame count baked " +
                     std::to_string(pose.weightFrames.size()) +
                     " binary " + std::to_string(byPath.size()));
        }
        for (const auto &[path, matrix] : pose.weightFrames) {
            const auto found = byPath.find(path.GetString());
            if (found == byPath.end()) {
                push("weight frame " + path.GetString() +
                         " missing from the binary");
            } else if (!SameMatrix(matrix, *found->second)) {
                push("weight frame " + path.GetString() + " differs");
            }
        }
    }
    {
        std::map<std::string, const rigExec::RigExecRuntimeWeightField *>
            byPath;
        for (const auto &field : reader.GetWeightFields()) {
            byPath[field.path] = &field;
        }
        if (pose.weightFields.size() != byPath.size()) {
            push("weight-field count baked " +
                     std::to_string(pose.weightFields.size()) +
                     " binary " + std::to_string(byPath.size()));
        }
        for (const auto &[path, field] : pose.weightFields) {
            const auto found = byPath.find(path.GetString());
            if (found == byPath.end()) {
                push("weight field " + path.GetString() +
                         " missing from the binary");
                continue;
            }
            if (field.target.GetString() != found->second->target) {
                push("weight field " + path.GetString() +
                         " targets " + found->second->target);
                continue;
            }
            if (field.weights.size() != found->second->weights.size()) {
                push("weight field " + path.GetString() +
                         " count differs");
                continue;
            }
            for (size_t i = 0; i < field.weights.size(); ++i) {
                if (field.weights[i] != found->second->weights[i]) {
                    push("weight field " + path.GetString() +
                             " weight " + std::to_string(i) +
                             " differs");
                    break;
                }
            }
        }
    }
    {
        std::map<std::string, const rigExec::RigExecRuntimeProviderXform *>
            byPath;
        for (const auto &revised : reader.GetProviderXforms()) {
            byPath[revised.path] = &revised;
        }
        if (pose.providerXforms.size() != byPath.size()) {
            push("provider-xform count baked " +
                     std::to_string(pose.providerXforms.size()) +
                     " binary " + std::to_string(byPath.size()));
        }
        for (const auto &[path, matrix] : pose.providerXforms) {
            const auto found = byPath.find(path.GetString());
            if (found == byPath.end()) {
                push("provider xform " + path.GetString() +
                         " missing from the binary");
                continue;
            }
            if (!SameMatrix(matrix, found->second->matrix)) {
                push("provider xform " + path.GetString() +
                         " differs");
                continue;
            }
            const auto base = pose.providerBaseXforms.find(path);
            if (base == pose.providerBaseXforms.end() ||
                !SameMatrix(base->second, found->second->base)) {
                push("provider base xform " + path.GetString() +
                         " differs");
            }
        }
    }
    return same;
}

/// The property chains' published values: every value \p reader published
/// is the one \p pose moved at that path, of the same type and bit for
/// bit, and every float, double, vec3f or (non-`primvars:`) matrix \p pose
/// moved is one \p reader published. Appends at most twelve lines to
/// \p diffs; true when nothing differs.
inline bool
RigExecCompareRuntimeProperties(const rigExec::RigExecRigPose &pose,
                                const rigExec::RigExecRuntimeReader &reader,
                                std::vector<std::string> *diffs)
{
    using Tag = rigExec::RrPropertyValue::Tag;
    bool same = true;
    const auto push = [&](const std::string &line) {
        same = false;
        rigExecTestDrive::Push(diffs, line);
    };
    const auto sameValue = [](const PXR_NS::VtValue &want,
                              const rigExec::RrPropertyValue &got) {
        if (want.IsHolding<float>()) {
            const float w = want.UncheckedGet<float>();
            return got.tag == Tag::Float &&
                   std::memcmp(&w, &got.f32, sizeof(float)) == 0;
        }
        if (want.IsHolding<double>()) {
            const double w = want.UncheckedGet<double>();
            return got.tag == Tag::Double &&
                   std::memcmp(&w, &got.f64, sizeof(double)) == 0;
        }
        if (want.IsHolding<PXR_NS::GfMatrix4d>()) {
            const PXR_NS::GfMatrix4d &w =
                want.UncheckedGet<PXR_NS::GfMatrix4d>();
            if (got.tag != Tag::Matrix4d) {
                return false;
            }
            for (int r = 0; r < 4; ++r) {
                for (int c = 0; c < 4; ++c) {
                    const double a = w[r][c];
                    const double b = got.matrix[size_t(r)][size_t(c)];
                    if (std::memcmp(&a, &b, sizeof(double)) != 0) {
                        return false;
                    }
                }
            }
            return true;
        }
        if (want.IsHolding<PXR_NS::GfVec3f>()) {
            const PXR_NS::GfVec3f &w = want.UncheckedGet<PXR_NS::GfVec3f>();
            return got.tag == Tag::Vec3f &&
                   std::memcmp(w.data(), got.vec.data(), sizeof(float) * 3) ==
                       0;
        }
        return false;
    };
    std::map<std::string, bool> published;
    for (const rigExec::RigExecRuntimePropertyValue &value :
         reader.GetPropertyValues()) {
        published[value.path] = true;
        const auto found =
            pose.movedProperties.find(PXR_NS::SdfPath(value.path));
        if (found == pose.movedProperties.end()) {
            push("property " + value.path + " is not moved by the pose");
        } else if (!sameValue(found->second, value.value)) {
            push("property " + value.path + " differs");
        }
    }
    const std::string prefix = "primvars:";
    for (const auto &[path, value] : pose.movedProperties) {
        const bool scalar = value.IsHolding<float>() ||
                            value.IsHolding<double>() ||
                            value.IsHolding<PXR_NS::GfVec3f>();
        const bool matrix =
            value.IsHolding<PXR_NS::GfMatrix4d>() &&
            path.GetName().compare(0, prefix.size(), prefix) != 0;
        if ((scalar || matrix) && !published.count(path.GetString())) {
            push("property " + path.GetString() +
                 " is not published by the binary");
        }
    }
    return same;
}

/// Compares \p reader's last run with \p pose: the output domains
/// (RigExecCompareRuntimeOutputs), and the diagnostics, with
/// \p compileNotices ahead of the pose's (the notices an explicit Compile
/// took from the first generation, which the runtime replays on its first
/// Execute). With \p compareCounters, the revisions executed and created
/// and the schedules built too. Appends at most twelve lines to \p diffs;
/// true when nothing differs.
inline bool
RigExecCompareRuntime(const rigExec::RigExecRigPose &pose,
                      const rigExec::RigExecRuntimeReader &reader,
                      bool compareCounters,
                      const std::vector<std::string> &compileNotices,
                      std::vector<std::string> *diffs)
{
    bool same = RigExecCompareRuntimeOutputs(pose, reader, diffs);
    const auto push = [&](const std::string &line) {
        same = false;
        rigExecTestDrive::Push(diffs, line);
    };
    {
        std::vector<std::string> expected = compileNotices;
        expected.insert(expected.end(), pose.diagnostics.begin(),
                        pose.diagnostics.end());
        const std::vector<std::string> &rt = reader.GetDiagnostics();
        if (expected.size() != rt.size()) {
            push("diagnostic count baked " +
                     std::to_string(expected.size()) + " binary " +
                     std::to_string(rt.size()));
        }
        for (size_t i = 0; i < std::min(expected.size(), rt.size()); ++i) {
            if (expected[i] != rt[i]) {
                push("diagnostic " + std::to_string(i) + " baked [" +
                         expected[i].substr(0, 160) + "] binary [" +
                         rt[i].substr(0, 160) + "]");
            }
        }
    }
    if (compareCounters) {
        const rigExec::RigExecRuntimeCounters counters = reader.GetCounters();
        if (pose.moverGraphRevisionsExecuted != counters.revisionsExecuted ||
            pose.moverGraphRevisionsCreated != counters.revisionsCreated ||
            pose.moverGraphSchedulesBuilt != counters.schedulesBuilt) {
            push("work counters differ: executed " +
                     std::to_string(pose.moverGraphRevisionsExecuted) +
                     "/" + std::to_string(counters.revisionsExecuted) +
                     ", created " +
                     std::to_string(pose.moverGraphRevisionsCreated) +
                     "/" + std::to_string(counters.revisionsCreated) +
                     ", schedules " +
                     std::to_string(pose.moverGraphSchedulesBuilt) +
                     "/" + std::to_string(counters.schedulesBuilt));
        }
    }
    return same;
}

/// \p lines without the "mover graph:" summary line, whose counters a
/// freshly compiled reference moves (rigExecPose's drag comparison strips
/// it the same way).
inline std::vector<std::string>
RigExecTestWithoutSummary(std::vector<std::string> lines)
{
    lines.erase(std::remove_if(lines.begin(), lines.end(),
                               [](const std::string &line) {
                                   return line.rfind("mover graph:", 0) == 0;
                               }),
                lines.end());
    return lines;
}

/// The runtime view of array value \p value (a VtIntArray, VtFloatArray,
/// VtDoubleArray, VtVec2fArray or VtVec3fArray), pointing into it; a
/// Double-tagged empty view for any other value.
inline rigExec::RigExecRuntimeArray
RigExecTestArrayView(const PXR_NS::VtValue &value)
{
    using rigExec::RrInputTag;
    rigExec::RigExecRuntimeArray view;
    view.tag = RrInputTag::Double;
    const auto take = [&](RrInputTag tag, const void *data, size_t count) {
        view.tag = tag;
        view.data = data;
        view.count = count;
    };
    if (value.IsHolding<PXR_NS::VtIntArray>()) {
        const auto &a = value.UncheckedGet<PXR_NS::VtIntArray>();
        take(RrInputTag::IntArray, a.cdata(), a.size());
    } else if (value.IsHolding<PXR_NS::VtFloatArray>()) {
        const auto &a = value.UncheckedGet<PXR_NS::VtFloatArray>();
        take(RrInputTag::FloatArray, a.cdata(), a.size());
    } else if (value.IsHolding<PXR_NS::VtDoubleArray>()) {
        const auto &a = value.UncheckedGet<PXR_NS::VtDoubleArray>();
        take(RrInputTag::DoubleArray, a.cdata(), a.size());
    } else if (value.IsHolding<PXR_NS::VtVec2fArray>()) {
        const auto &a = value.UncheckedGet<PXR_NS::VtVec2fArray>();
        take(RrInputTag::Vec2fArray, a.cdata(), a.size());
    } else if (value.IsHolding<PXR_NS::VtVec3fArray>()) {
        const auto &a = value.UncheckedGet<PXR_NS::VtVec3fArray>();
        take(RrInputTag::Vec3fArray, a.cdata(), a.size());
    }
    return view;
}

/// One input set on a reader beside its reference: \p value (an array,
/// or an int, float or double scalar) on the input at \p name, authored in
/// the reference's session layer at Default, or, \p sampled, set with the
/// sampled array API and authored as a time sample at the bake time.
struct RigExecTestArraySet {
    std::string name;
    PXR_NS::VtValue value;
    bool sampled = false;
};

/// The reference of \p sets at \p time: a fresh evaluator in \p mode,
/// compiled on \p stage, then given each set's value in the session layer
/// (at Default, or as a time sample at \p time), evaluated at \p time. The
/// session layer is put back exactly. False with the reason.
inline bool
RigExecTestArrayReference(const PXR_NS::UsdStageRefPtr &stage,
                          const PXR_NS::SdfPath &rigPath,
                          rigExec::RigExecEvaluationMode mode,
                          const std::vector<RigExecTestArraySet> &sets,
                          double time, rigExec::RigExecRigPose *pose,
                          std::string *error)
{
    rigExec::RigExecRigEvaluator evaluator(stage, rigPath);
    evaluator.SetEvaluationMode(mode);
    std::vector<std::string> notices;
    if (!evaluator.Compile(&notices)) {
        *error = "the reference did not compile";
        return false;
    }
    PXR_NS::SdfLayerRefPtr saved = PXR_NS::SdfLayer::CreateAnonymous();
    saved->TransferContent(stage->GetSessionLayer());
    bool authored = true;
    {
        PXR_NS::UsdEditContext context(stage, stage->GetSessionLayer());
        for (const RigExecTestArraySet &set : sets) {
            const PXR_NS::UsdAttribute attribute =
                stage->GetAttributeAtPath(PXR_NS::SdfPath(set.name));
            authored = authored && attribute &&
                       (set.sampled
                            ? attribute.Set(set.value,
                                            PXR_NS::UsdTimeCode(time))
                            : attribute.Set(set.value));
        }
    }
    if (authored) {
        *pose = evaluator.Evaluate(PXR_NS::UsdTimeCode(time));
    }
    stage->GetSessionLayer()->TransferContent(saved);
    if (!authored) {
        *error = "the session layer refused a set";
        return false;
    }
    if (!pose->valid || pose->bakedParityMismatches != 0 ||
        pose->moverGraphParityMismatches != 0) {
        *error = "the reference pose is invalid";
        return false;
    }
    return true;
}

/// Sets \p sets on \p reader, which already ran at its bake time: arrays
/// through SetInputArray or SetSampledInputArrayAt, scalars through
/// SetInput. False with the reader's reason.
inline bool
RigExecTestApplyArraySets(rigExec::RigExecRuntimeReader *reader,
                          const std::vector<RigExecTestArraySet> &sets,
                          std::string *error)
{
    for (const RigExecTestArraySet &set : sets) {
        size_t index = 0;
        if (!reader->FindInput(set.name, &index)) {
            *error = "no input " + set.name;
            return false;
        }
        const rigExec::RigExecRuntimeArray view =
            RigExecTestArrayView(set.value);
        if (view.tag != rigExec::RrInputTag::Double) {
            if (!(set.sampled ? reader->SetSampledInputArrayAt(index, view,
                                                               error)
                              : reader->SetInputArrayAt(index, view,
                                                        error))) {
                return false;
            }
            continue;
        }
        rigExec::RrInputValue scalar;
        if (set.value.IsHolding<int>()) {
            scalar.tag = rigExec::RrInputTag::Int;
            scalar.i32 = set.value.UncheckedGet<int>();
        } else if (set.value.IsHolding<float>()) {
            scalar.tag = rigExec::RrInputTag::Float;
            scalar.f32 = set.value.UncheckedGet<float>();
        } else if (set.value.IsHolding<double>()) {
            scalar.tag = rigExec::RrInputTag::Double;
            scalar.f64 = set.value.UncheckedGet<double>();
        } else {
            *error = "no runtime form for the set of " + set.name;
            return false;
        }
        if (!reader->SetInputAt(index, scalar, error)) {
            return false;
        }
    }
    return true;
}

/// \p reader's last run against \p pose: the outputs, the property values
/// and, unless \p outputsOnly, the diagnostics less the summary line and
/// less the line a reference that rebuilt for a structural edit opens with
/// (the binary never rebuilds). Appends at most twelve lines to \p diffs;
/// true when nothing differs.
inline bool
RigExecCompareRuntimeRun(const rigExec::RigExecRigPose &pose,
                         const rigExec::RigExecRuntimeReader &reader,
                         std::vector<std::string> *diffs,
                         bool outputsOnly = false)
{
    bool same = RigExecCompareRuntimeOutputs(pose, reader, diffs);
    same = RigExecCompareRuntimeProperties(pose, reader, diffs) && same;
    if (outputsOnly) {
        return same;
    }
    std::vector<std::string> want =
        RigExecTestWithoutSummary(pose.diagnostics);
    if (!want.empty() && want.front() == "structural edit: epoch rebuilt") {
        want.erase(want.begin());
    }
    const std::vector<std::string> got =
        RigExecTestWithoutSummary(reader.GetDiagnostics());
    if (want != got) {
        same = false;
        rigExecTestDrive::Push(diffs, "diagnostics differ: " +
                                          std::to_string(want.size()) +
                                          " baked line(s), " +
                                          std::to_string(got.size()) +
                                          " binary");
        for (size_t i = 0; i < std::max(want.size(), got.size()); ++i) {
            const std::string a = i < want.size() ? want[i] : "(none)";
            const std::string b = i < got.size() ? got[i] : "(none)";
            if (a != b) {
                rigExecTestDrive::Push(diffs, "  baked [" + a.substr(0, 150) +
                                                  "] binary [" +
                                                  b.substr(0, 150) + "]");
            }
        }
    }
    return same;
}

#endif  // RIGEXEC_TEST_RUNTIME_DRIVE_H
