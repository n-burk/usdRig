//
// .rigexec capture.
//

#include "rigExecBake/capture.h"
#include "rigExec/bakedProgram.h"
#include "rigExec/bakedProgramImpl.h"
#include "rigExec/rigEvaluator.h"

#include "pxr/usd/usd/timeCode.h"
#include "pxr/base/gf/vec2f.h"
#include "pxr/base/gf/vec3i.h"
#include "pxr/base/vt/array.h"

#include <cstdio>

PXR_NAMESPACE_USING_DIRECTIVE

namespace rigExec {
namespace {

RigExecWireMatrix4d
_ToMatrix(const GfMatrix4d &m)
{
    RigExecWireMatrix4d out;
    for (int r = 0; r < 4; ++r) {
        for (int c = 0; c < 4; ++c) {
            out[size_t(r * 4 + c)] = m[r][c];
        }
    }
    return out;
}

RigExecWireVec3d
_ToVec3d(const GfVec3d &v)
{
    return RigExecWireVec3d{v[0], v[1], v[2]};
}

RigExecWireVec3f
_ToVec3f(const GfVec3f &v)
{
    return RigExecWireVec3f{v[0], v[1], v[2]};
}

RigExecWireVec2f
_ToVec2f(const GfVec2f &v)
{
    return RigExecWireVec2f{v[0], v[1]};
}

RigExecWireFrame
_ToFrame(const RigExecPointFrame &f)
{
    RigExecWireFrame out;
    for (size_t i = 0; i < 4; ++i) {
        out.points[i] = _ToVec3d(f.points[i]);
    }
    out.flags = f.flags;
    return out;
}

std::string
_FormatFrame(double frame)
{
    char buffer[32];
    std::snprintf(buffer, sizeof(buffer), "%.17g", frame);
    return std::string(buffer);
}

}  // namespace

RigExecBakeCapture::RigExecBakeCapture(RigExecRigEvaluator &evaluator,
                                       RigExecBinaryWriter *writer,
                                       std::string *error)
    : _evaluator(&evaluator), _writer(writer)
{
    auto Fail = [&](const std::string &what) {
        if (error) {
            *error = what;
        }
        return;
    };
    if (evaluator.HasInteractiveOverrides()) {
        Fail("cannot bake with interactive overrides standing");
        return;
    }
    _program = evaluator.GetBakedProgram();
    if (!_program) {
        Fail("no baked program standing to capture from");
        return;
    }
    const RigExecBakedProgramImpl &program = _program->GetStepGraph();
    auto Add = [&](const auto &input, RigExecWireInput::Tag tag) {
        using T = std::decay_t<decltype(input.constant)>;
        (void)sizeof(T);
        const bool bound =
            input.query.IsValid() || bool(input.resolvedAttr);
        if (!input.varying || !bound) {
            return true;
        }
        const void *key = &input;
        if (_uids.count(key)) {
            return true;
        }
        const uint32_t uid = uint32_t(_table.directory.size());
        _uids[key] = uid;
        RigExecWireInputDirectoryEntry entry;
        entry.tag = tag;
        entry.overrideIndex = int32_t(input.overrideIndex);
        entry.head = input.head
                         ? _writer->AddString(
                               input.head.GetPath().GetString())
                         : 0;
        _table.directory.push_back(entry);
        return true;
    };
    for (const RigExecBakedProgramImpl::Ladder &ladder : program.ladders) {
        Add(ladder.restSpace, RigExecWireInput::Tag::Matrix4d);
        Add(ladder.defaultSpace, RigExecWireInput::Tag::Matrix4d);
        Add(ladder.posedSpace, RigExecWireInput::Tag::Matrix4d);
        for (const auto &avar : ladder.restAvars) {
            Add(avar, RigExecWireInput::Tag::Double);
        }
        for (const auto &avar : ladder.defaultAvars) {
            Add(avar, RigExecWireInput::Tag::Double);
        }
        Add(ladder.rotationOrder, RigExecWireInput::Tag::Token);
    }
    for (const RigExecBakedProgramImpl::PoseInterpolator &interp :
         program.poseInterpolators) {
        Add(interp.enabled, RigExecWireInput::Tag::Bool);
    }
    for (const RigExecBakedProgramImpl::Solver &solver : program.solvers) {
        Add(solver.bend, RigExecWireInput::Tag::Double);
        Add(solver.upperOffset, RigExecWireInput::Tag::Double);
        Add(solver.lowerOffset, RigExecWireInput::Tag::Double);
        Add(solver.stretch, RigExecWireInput::Tag::Float);
        Add(solver.softness, RigExecWireInput::Tag::Float);
        Add(solver.blendWeight, RigExecWireInput::Tag::Float);
        Add(solver.preserveVolume, RigExecWireInput::Tag::Double);
        Add(solver.midFollowWeight, RigExecWireInput::Tag::Double);
        Add(solver.roll, RigExecWireInput::Tag::Double);
        Add(solver.twist, RigExecWireInput::Tag::Double);
        Add(solver.minLengthRatio, RigExecWireInput::Tag::Double);
        Add(solver.twistTurns, RigExecWireInput::Tag::Double);
        Add(solver.ribbonSampleCount, RigExecWireInput::Tag::Int);
    }
    for (const RigExecBakedProgramImpl::Constraint &constraint :
         program.constraints) {
        Add(constraint.enabled, RigExecWireInput::Tag::Bool);
        Add(constraint.defaultWeight, RigExecWireInput::Tag::Float);
        Add(constraint.offset, RigExecWireInput::Tag::Vec3d);
        Add(constraint.affectX, RigExecWireInput::Tag::Bool);
        Add(constraint.affectY, RigExecWireInput::Tag::Bool);
        Add(constraint.affectZ, RigExecWireInput::Tag::Bool);
        Add(constraint.tX, RigExecWireInput::Tag::Bool);
        Add(constraint.tY, RigExecWireInput::Tag::Bool);
        Add(constraint.tZ, RigExecWireInput::Tag::Bool);
        Add(constraint.rX, RigExecWireInput::Tag::Bool);
        Add(constraint.rY, RigExecWireInput::Tag::Bool);
        Add(constraint.rZ, RigExecWireInput::Tag::Bool);
        Add(constraint.sX, RigExecWireInput::Tag::Bool);
        Add(constraint.sY, RigExecWireInput::Tag::Bool);
        Add(constraint.sZ, RigExecWireInput::Tag::Bool);
        Add(constraint.aimVector, RigExecWireInput::Tag::Vec3d);
        Add(constraint.upVector, RigExecWireInput::Tag::Vec3d);
        Add(constraint.rotationOffset, RigExecWireInput::Tag::Vec3d);
        Add(constraint.worldUpVector, RigExecWireInput::Tag::Vec3d);
        Add(constraint.poleVector, RigExecWireInput::Tag::Vec3d);
        Add(constraint.twistDegrees, RigExecWireInput::Tag::Double);
    }
    for (const RigExecBakedProgramImpl::WeightObject &object :
         program.weightObjects) {
        Add(object.defaultWeight, RigExecWireInput::Tag::Float);
        Add(object.driver, RigExecWireInput::Tag::Float);
        Add(object.scale, RigExecWireInput::Tag::Float);
        Add(object.bias, RigExecWireInput::Tag::Float);
        Add(object.strength, RigExecWireInput::Tag::Float);
        Add(object.invert, RigExecWireInput::Tag::Float);
        Add(object.falloffMin, RigExecWireInput::Tag::Float);
        Add(object.falloffMax, RigExecWireInput::Tag::Float);
        Add(object.scaleX, RigExecWireInput::Tag::Float);
        Add(object.scaleY, RigExecWireInput::Tag::Float);
        Add(object.scaleZ, RigExecWireInput::Tag::Float);
        Add(object.extentU, RigExecWireInput::Tag::Float);
        Add(object.extentV, RigExecWireInput::Tag::Float);
        Add(object.curvenetSamples, RigExecWireInput::Tag::Int);
        Add(object.curvenetUnreached, RigExecWireInput::Tag::Float);
    }
    for (const RigExecBakedProgramImpl::AvarBinding &binding :
         program.avarBindings) {
        Add(binding.input, RigExecWireInput::Tag::Double);
    }
    for (const RigExecBakedProgramImpl::AvarBinding &binding :
         program.avarConstantBindings) {
        Add(binding.input, RigExecWireInput::Tag::Double);
    }
    _table.overridablePaths.reserve(program.overridableInputs.size());
    _table.overridableIndices.reserve(program.overridableInputs.size());
    for (const auto &entry : program.overridableInputs) {
        _table.overridablePaths.push_back(
            _writer->AddString(entry.first.GetString()));
        std::vector<int32_t> indices(entry.second.begin(),
                                     entry.second.end());
        _table.overridableIndices.push_back(std::move(indices));
    }
    _recorder = std::make_unique<RigExecBakeReadRecorder>();
    program.resolvedInputs->bakeRecorder = _recorder.get();
    _valid = true;
}

RigExecBakeCapture::~RigExecBakeCapture()
{
    if (_valid && _program) {
        _program->GetStepGraph().resolvedInputs->bakeRecorder = nullptr;
    }
}

bool
RigExecBakeCapture::CaptureFrame(double frame, RigExecRigPose *pose,
                                 std::string *error)
{
    auto Fail = [&](const std::string &what) {
        if (error) {
            *error = what;
        }
        return false;
    };
    if (!_valid || !pose) {
        return Fail("capture is not armed");
    }
    _recorder->Clear();
    *pose = _evaluator->Evaluate(UsdTimeCode(frame));
    if (_evaluator->GetBakedProgram() != _program) {
        return Fail("the program rebuilt during capture at frame " +
                    _FormatFrame(frame));
    }
    return _Drain(_program->GetStepGraph(), frame, error);
}

bool
RigExecBakeCapture::_Drain(const RigExecBakedProgramImpl &program,
                           double frame, std::string *error)
{
    auto Fail = [&](const std::string &what) {
        if (error) {
            *error = what;
        }
        return false;
    };
    RigExecWireFrameInputs record;
    record.frame = frame;
    // The recorder's map is address-ordered; the streams are uid-ordered,
    // so the drain sorts through the uid map. An unmapped address is a
    // read the directory walk missed -- failing names the hole rather
    // than dropping a value the runtime would need.
    std::map<uint32_t, VtValue> ordered;
    {
        std::lock_guard<std::mutex> guard(_recorder->mutex);
        for (const auto &entry : _recorder->reads) {
            const auto found = _uids.find(entry.first);
            if (found == _uids.end()) {
                return Fail("a resolved input is missing from the "
                            "directory at frame " +
                            _FormatFrame(frame));
            }
            ordered[found->second] = entry.second;
        }
    }
    record.uids.reserve(ordered.size());
    record.values.reserve(ordered.size());
    for (const auto &entry : ordered) {
        const RigExecWireInputDirectoryEntry &dir =
            _table.directory[entry.first];
        RigExecWireValue value;
        value.tag = dir.tag;
        const VtValue &held = entry.second;
        switch (dir.tag) {
        case RigExecWireInput::Tag::Double:
            if (!held.IsHolding<double>()) {
                return Fail("input type drifted at frame " +
                            _FormatFrame(frame));
            }
            value.f64 = held.UncheckedGet<double>();
            break;
        case RigExecWireInput::Tag::Float:
            if (!held.IsHolding<float>()) {
                return Fail("input type drifted at frame " +
                            _FormatFrame(frame));
            }
            value.f32 = held.UncheckedGet<float>();
            break;
        case RigExecWireInput::Tag::Bool:
            if (!held.IsHolding<bool>()) {
                return Fail("input type drifted at frame " +
                            _FormatFrame(frame));
            }
            value.boolean = held.UncheckedGet<bool>();
            break;
        case RigExecWireInput::Tag::Int:
            if (!held.IsHolding<int>()) {
                return Fail("input type drifted at frame " +
                            _FormatFrame(frame));
            }
            value.i32 = int32_t(held.UncheckedGet<int>());
            break;
        case RigExecWireInput::Tag::Matrix4d:
            if (!held.IsHolding<GfMatrix4d>()) {
                return Fail("input type drifted at frame " +
                            _FormatFrame(frame));
            }
            value.matrix = _ToMatrix(held.UncheckedGet<GfMatrix4d>());
            break;
        case RigExecWireInput::Tag::Token:
            if (!held.IsHolding<TfToken>()) {
                return Fail("input type drifted at frame " +
                            _FormatFrame(frame));
            }
            value.token = _writer->AddString(
                held.UncheckedGet<TfToken>().GetString());
            break;
        case RigExecWireInput::Tag::Vec3d:
            if (!held.IsHolding<GfVec3d>()) {
                return Fail("input type drifted at frame " +
                            _FormatFrame(frame));
            }
            value.vec = _ToVec3d(held.UncheckedGet<GfVec3d>());
            break;
        }
        record.uids.push_back(entry.first);
        record.values.push_back(value);
    }
    record.chainHaveBase.reserve(program.chains.size());
    record.chainBases.reserve(program.chains.size());
    for (const RigExecBakedProgramImpl::GeomChain &chain : program.chains) {
        record.chainHaveBase.push_back(chain.haveBase ? uint8_t(1)
                                                      : uint8_t(0));
        std::vector<RigExecWireVec3f> base;
        base.reserve(chain.lastBase.size());
        for (const GfVec3f &p : chain.lastBase) {
            base.push_back(_ToVec3f(p));
        }
        record.chainBases.push_back(std::move(base));
    }
    for (const RigExecBakedProgramImpl::GeomChain &chain : program.chains) {
        for (const RigExecBakedProgramImpl::GeomChain::Derived &derived :
             chain.derived) {
            record.derivedHaveBase.push_back(derived.haveBase ? uint8_t(1)
                                                              : uint8_t(0));
            std::vector<RigExecWireVec3f> base;
            base.reserve(derived.lastBase.size());
            for (const GfVec3f &p : derived.lastBase) {
                base.push_back(_ToVec3f(p));
            }
            record.derivedBases.push_back(std::move(base));
        }
    }
    record.xformBase.reserve(program.xformBase.size());
    for (const GfMatrix4d &m : program.xformBase) {
        record.xformBase.push_back(_ToMatrix(m));
    }
    record.nativeFrames.reserve(program.nativeFrames.size());
    for (const RigExecPointFrame &f : program.nativeFrames) {
        record.nativeFrames.push_back(_ToFrame(f));
    }
    record.propertyPaths.reserve(program.propertyResults.size());
    record.propertyValues.reserve(program.propertyResults.size());
    for (const auto &entry : program.propertyResults) {
        record.propertyPaths.push_back(
            _writer->AddString(entry.first.GetString()));
        RigExecWirePropertyValue value;
        const VtValue &held = entry.second;
        if (held.IsHolding<float>()) {
            value.tag = RigExecWirePropertyValue::Tag::Float;
            value.f32 = held.UncheckedGet<float>();
        } else if (held.IsHolding<double>()) {
            value.tag = RigExecWirePropertyValue::Tag::Double;
            value.f64 = held.UncheckedGet<double>();
        } else if (held.IsHolding<GfMatrix4d>()) {
            value.tag = RigExecWirePropertyValue::Tag::Matrix4d;
            value.matrix = _ToMatrix(held.UncheckedGet<GfMatrix4d>());
        } else if (held.IsHolding<GfVec3f>()) {
            value.tag = RigExecWirePropertyValue::Tag::Vec3f;
            value.vec = _ToVec3f(held.UncheckedGet<GfVec3f>());
        } else {
            return Fail(std::string("unencodable property result of type ") +
                        held.GetTypeName() + " at " +
                        entry.first.GetString());
        }
        record.propertyValues.push_back(value);
    }
    record.arrayWeights.reserve(program.constraintArrays.size());
    record.arrayTranslationOffsets.reserve(
        program.constraintArrays.size());
    record.arrayRotationOffsets.reserve(program.constraintArrays.size());
    record.arrayOk.reserve(program.constraintArrays.size());
    record.arrayPoleWeights.reserve(program.constraintArrays.size());
    record.arrayPoleOk.reserve(program.constraintArrays.size());
    for (const RigExecBakedProgramImpl::ConstraintArrays &arrays :
         program.constraintArrays) {
        record.arrayWeights.push_back(arrays.weights);
        std::vector<RigExecWireVec3d> translation;
        translation.reserve(arrays.translationOffsets.size());
        for (const GfVec3d &v : arrays.translationOffsets) {
            translation.push_back(_ToVec3d(v));
        }
        record.arrayTranslationOffsets.push_back(std::move(translation));
        std::vector<RigExecWireVec3d> rotation;
        rotation.reserve(arrays.rotationOffsets.size());
        for (const GfVec3d &v : arrays.rotationOffsets) {
            rotation.push_back(_ToVec3d(v));
        }
        record.arrayRotationOffsets.push_back(std::move(rotation));
        record.arrayOk.push_back(arrays.ok ? uint8_t(1) : uint8_t(0));
        record.arrayPoleWeights.push_back(arrays.poleWeights);
        record.arrayPoleOk.push_back(arrays.poleOk ? uint8_t(1)
                                                   : uint8_t(0));
    }
    record.deltaBaseMatrix.reserve(program.deltaBaseMatrix.size());
    for (const GfMatrix4d &m : program.deltaBaseMatrix) {
        record.deltaBaseMatrix.push_back(_ToMatrix(m));
    }
    record.deltaBaseOk.reserve(program.deltaBaseOk.size());
    for (char v : program.deltaBaseOk) {
        record.deltaBaseOk.push_back(v ? uint8_t(1) : uint8_t(0));
    }
    record.weightPackets.reserve(program.weightPackets.size());
    for (const RigExecWeightPacket &packet : program.weightPackets) {
        RigExecWireWeightPacket wire;
        wire.representation =
            _writer->AddString(packet.representation.GetString());
        wire.rangePolicy =
            _writer->AddString(packet.rangePolicy.GetString());
        wire.values = packet.values;
        wire.indices.assign(packet.indices.begin(), packet.indices.end());
        wire.defaultWeight = packet.defaultWeight;
        wire.valid = packet.valid;
        record.weightPackets.push_back(std::move(wire));
    }
    record.currentPhaseWeights.reserve(program.currentPhaseWeights.size());
    for (const SdfPath &path : program.currentPhaseWeights) {
        record.currentPhaseWeights.push_back(
            _writer->AddString(path.GetString()));
    }
    // The blend reads the assembly retained, per chain/revision in program
    // order, derived beside their chain's: weights per channel, activations
    // and dense points per sample. Sparse samples carry no points -- their
    // shape rides the epoch layouts or the refused stream below.
    auto DrainBlend =
        [&](const RigExecBakedProgramImpl::GeomRevision &revision,
            std::vector<float> *weights,
            std::vector<std::vector<float>> *activations,
            std::vector<std::vector<std::vector<RigExecWireVec3f>>> *points) {
            weights->reserve(revision.blendChannels.size());
            activations->reserve(revision.blendChannels.size());
            points->reserve(revision.blendChannels.size());
            for (const auto &channel : revision.blendChannels) {
                weights->push_back(channel.lastWeight);
                std::vector<float> channelActivations;
                std::vector<std::vector<RigExecWireVec3f>> channelPoints;
                channelActivations.reserve(channel.samples.size());
                channelPoints.reserve(channel.samples.size());
                for (const auto &sample : channel.samples) {
                    channelActivations.push_back(sample.lastActivation);
                    std::vector<RigExecWireVec3f> samplePoints;
                    samplePoints.reserve(sample.lastPoints.size());
                    for (const GfVec3f &p : sample.lastPoints) {
                        samplePoints.push_back(_ToVec3f(p));
                    }
                    channelPoints.push_back(std::move(samplePoints));
                }
                activations->push_back(std::move(channelActivations));
                points->push_back(std::move(channelPoints));
            }
        };
    auto DrainPacket = [&](const RigExecWeightPacket &packet) {
        RigExecWireWeightPacket wire;
        wire.representation =
            _writer->AddString(packet.representation.GetString());
        wire.rangePolicy =
            _writer->AddString(packet.rangePolicy.GetString());
        wire.values = packet.values;
        wire.indices.assign(packet.indices.begin(), packet.indices.end());
        wire.defaultWeight = packet.defaultWeight;
        wire.valid = packet.valid;
        return wire;
    };
    auto DrainAdjuster =
        [&](const RigExecBakedProgramImpl::GeomRevision &revision,
            RigExecWireMatrix4d *matrix, uint8_t *have) {
            *matrix = _ToMatrix(revision.lastAdjusterNetToAsset);
            // The publish's own condition, re-evaluated: the matrix is
            // fresh only for an adjuster that published this frame.
            const bool published =
                revision.op == RigExecRevisionOp::CurvenetAdjuster &&
                revision.resultStatus == "ok";
            *have = published ? uint8_t(1) : uint8_t(0);
        };
    record.blendWeights.reserve(program.chains.size());
    record.blendActivations.reserve(program.chains.size());
    record.blendPoints.reserve(program.chains.size());
    record.derivedBlendWeights.reserve(program.chains.size());
    record.derivedBlendActivations.reserve(program.chains.size());
    record.derivedBlendPoints.reserve(program.chains.size());
    record.revisionDefaultWeights.reserve(program.chains.size());
    record.revisionPhasePackets.reserve(program.chains.size());
    record.derivedPhasePackets.reserve(program.chains.size());
    record.revisionAdjusters.reserve(program.chains.size());
    record.revisionAdjusterHave.reserve(program.chains.size());
    for (const RigExecBakedProgramImpl::GeomChain &chain : program.chains) {
        std::vector<std::vector<float>> weights;
        std::vector<std::vector<std::vector<float>>> activations;
        std::vector<std::vector<std::vector<std::vector<RigExecWireVec3f>>>>
            points;
        std::vector<float> defaults;
        std::vector<RigExecWireWeightPacket> packets;
        std::vector<RigExecWireMatrix4d> adjusters;
        std::vector<uint8_t> adjusterHave;
        weights.reserve(chain.revisions.size());
        activations.reserve(chain.revisions.size());
        points.reserve(chain.revisions.size());
        defaults.reserve(chain.revisions.size());
        packets.reserve(chain.revisions.size());
        adjusters.reserve(chain.revisions.size());
        adjusterHave.reserve(chain.revisions.size());
        auto DrainRefused =
            [&](const RigExecBakedProgramImpl::GeomRevision &revision) {
                for (const auto &channel : revision.blendChannels) {
                    for (const auto &sample : channel.samples) {
                        if (!sample.layoutRefused) {
                            continue;
                        }
                        if (sample.blendShape.IsEmpty()) {
                            return Fail("a dense blend sample is flagged "
                                        "cache-refused at frame " +
                                        _FormatFrame(frame));
                        }
                        if (!sample.layout) {
                            return Fail("a refused blend layout has no "
                                        "per-frame shape at frame " +
                                        _FormatFrame(frame));
                        }
                        RigExecWireRefusedLayout wire;
                        wire.mover = _writer->AddString(
                            revision.moverPath.GetString());
                        wire.sample = _writer->AddString(
                            sample.samplePath.GetString());
                        wire.pointCount = uint64_t(sample.layout->pointCount);
                        wire.valid = sample.layout->valid;
                        wire.offsets.reserve(sample.layout->offsets.size());
                        for (const GfVec3f &p : sample.layout->offsets) {
                            wire.offsets.push_back(_ToVec3f(p));
                        }
                        wire.indices.assign(sample.layout->indices.begin(),
                                            sample.layout->indices.end());
                        record.refusedLayouts.push_back(std::move(wire));
                    }
                }
                return true;
            };
        auto DrainRevision =
            [&](const RigExecBakedProgramImpl::GeomRevision &revision) {
                std::vector<float> revisionWeights;
                std::vector<std::vector<float>> revisionActivations;
                std::vector<std::vector<std::vector<RigExecWireVec3f>>>
                    revisionPoints;
                DrainBlend(revision, &revisionWeights, &revisionActivations,
                           &revisionPoints);
                weights.push_back(std::move(revisionWeights));
                activations.push_back(std::move(revisionActivations));
                points.push_back(std::move(revisionPoints));
                defaults.push_back(revision.defaultWeight);
                packets.push_back(DrainPacket(revision.currentPhasePacket));
                RigExecWireMatrix4d adjuster;
                uint8_t have = 0;
                DrainAdjuster(revision, &adjuster, &have);
                adjusters.push_back(adjuster);
                adjusterHave.push_back(have);
                return DrainRefused(revision);
            };
        for (const RigExecBakedProgramImpl::GeomRevision &revision :
             chain.revisions) {
            if (!DrainRevision(revision)) {
                return false;
            }
        }
        record.blendWeights.push_back(std::move(weights));
        record.blendActivations.push_back(std::move(activations));
        record.blendPoints.push_back(std::move(points));
        record.revisionDefaultWeights.push_back(std::move(defaults));
        record.revisionPhasePackets.push_back(std::move(packets));
        record.revisionAdjusters.push_back(std::move(adjusters));
        record.revisionAdjusterHave.push_back(std::move(adjusterHave));
        std::vector<std::vector<float>> derivedWeights;
        std::vector<std::vector<std::vector<float>>> derivedActivations;
        std::vector<std::vector<std::vector<std::vector<RigExecWireVec3f>>>>
            derivedPoints;
        std::vector<RigExecWireWeightPacket> derivedPackets;
        for (const auto &derived : chain.derived) {
            std::vector<float> revisionWeights;
            std::vector<std::vector<float>> revisionActivations;
            std::vector<std::vector<std::vector<RigExecWireVec3f>>>
                revisionPoints;
            DrainBlend(derived.revision, &revisionWeights,
                       &revisionActivations, &revisionPoints);
            derivedWeights.push_back(std::move(revisionWeights));
            derivedActivations.push_back(std::move(revisionActivations));
            derivedPoints.push_back(std::move(revisionPoints));
            derivedPackets.push_back(
                DrainPacket(derived.revision.currentPhasePacket));
            if (!DrainRefused(derived.revision)) {
                return false;
            }
        }
        record.derivedBlendWeights.push_back(std::move(derivedWeights));
        record.derivedBlendActivations.push_back(std::move(derivedActivations));
        record.derivedBlendPoints.push_back(std::move(derivedPoints));
        record.derivedPhasePackets.push_back(std::move(derivedPackets));
    }
    record.solverRibbonPoints.reserve(program.solvers.size());
    for (const RigExecBakedProgramImpl::Solver &solver : program.solvers) {
        std::vector<RigExecWireVec3f> points;
        points.reserve(solver.ribbonPoints.size());
        for (const GfVec3f &p : solver.ribbonPoints) {
            points.push_back(_ToVec3f(p));
        }
        record.solverRibbonPoints.push_back(std::move(points));
    }
    record.constraintWeights.reserve(program.constraints.size());
    record.constraintHaveWeight.reserve(program.constraints.size());
    for (const RigExecBakedProgramImpl::Constraint &constraint :
         program.constraints) {
        // The resolve arm's own condition, plus the success the step
        // requires before it consumes the scratch: a failed resolve
        // passes through weightless, and the record says so.
        const bool resolved =
            !constraint.weightObject.IsEmpty() &&
            constraint.pointsTarget.IsEmpty() &&
            constraint.weightScratch.size() == 1;
        record.constraintHaveWeight.push_back(resolved ? uint8_t(1)
                                                       : uint8_t(0));
        record.constraintWeights.push_back(
            resolved ? constraint.weightScratch[0] : 0.0f);
    }
    // The assemblers stage-sourced reads: the recorder path half, in
    // (path, was-Default) order. An unencodable holding fails the frame:
    // a value the runtime cannot replay is a capture hole, not a skip.
    {
        std::lock_guard<std::mutex> guard(_recorder->mutex);
        record.pathReads.reserve(_recorder->pathReads.size());
        for (const auto &entry : _recorder->pathReads) {
            RigExecWirePathRead read;
            read.path = _writer->AddString(entry.first.first.GetString());
            read.wasDefault = entry.first.second ? uint8_t(1) : uint8_t(0);
            read.forceFrame =
                entry.second.forceFrame ? uint8_t(1) : uint8_t(0);
            const VtValue &held = entry.second.value;
            RigExecWirePathValue &value = read.value;
            if (held.IsEmpty()) {
                value.tag = RigExecWirePathValue::Tag::Absent;
            } else if (held.IsHolding<bool>()) {
                value.tag = RigExecWirePathValue::Tag::Bool;
                value.boolean = held.UncheckedGet<bool>();
            } else if (held.IsHolding<int>()) {
                value.tag = RigExecWirePathValue::Tag::Int;
                value.i32 = int32_t(held.UncheckedGet<int>());
            } else if (held.IsHolding<float>()) {
                value.tag = RigExecWirePathValue::Tag::Float;
                value.f32 = held.UncheckedGet<float>();
            } else if (held.IsHolding<double>()) {
                value.tag = RigExecWirePathValue::Tag::Double;
                value.f64 = held.UncheckedGet<double>();
            } else if (held.IsHolding<TfToken>()) {
                value.tag = RigExecWirePathValue::Tag::Token;
                value.token = _writer->AddString(
                    held.UncheckedGet<TfToken>().GetString());
            } else if (held.IsHolding<GfMatrix4d>()) {
                value.tag = RigExecWirePathValue::Tag::Matrix4d;
                value.matrix = _ToMatrix(held.UncheckedGet<GfMatrix4d>());
            } else if (held.IsHolding<GfVec3d>()) {
                value.tag = RigExecWirePathValue::Tag::Vec3d;
                value.vec = _ToVec3d(held.UncheckedGet<GfVec3d>());
            } else if (held.IsHolding<VtIntArray>()) {
                value.tag = RigExecWirePathValue::Tag::IntArray;
                const VtIntArray &array =
                    held.UncheckedGet<VtIntArray>();
                value.ints.reserve(array.size());
                for (int v : array) {
                    value.ints.push_back(int32_t(v));
                }
            } else if (held.IsHolding<VtFloatArray>()) {
                value.tag = RigExecWirePathValue::Tag::FloatArray;
                const VtFloatArray &array =
                    held.UncheckedGet<VtFloatArray>();
                value.floats.assign(array.begin(), array.end());
            } else if (held.IsHolding<VtArray<GfVec2f>>()) {
                value.tag = RigExecWirePathValue::Tag::Vec2fArray;
                const VtArray<GfVec2f> &array =
                    held.UncheckedGet<VtArray<GfVec2f>>();
                value.vec2s.reserve(array.size());
                for (const GfVec2f &v : array) {
                    value.vec2s.push_back(_ToVec2f(v));
                }
            } else if (held.IsHolding<VtVec3fArray>()) {
                value.tag = RigExecWirePathValue::Tag::Vec3fArray;
                const VtVec3fArray &array =
                    held.UncheckedGet<VtVec3fArray>();
                value.vec3s.reserve(array.size());
                for (const GfVec3f &v : array) {
                    value.vec3s.push_back(_ToVec3f(v));
                }
            } else if (held.IsHolding<GfVec3i>()) {
                value.tag = RigExecWirePathValue::Tag::Vec3i;
                const GfVec3i &v = held.UncheckedGet<GfVec3i>();
                value.vec3i = {{int32_t(v[0]), int32_t(v[1]),
                                int32_t(v[2])}};
            } else if (held.IsHolding<VtDoubleArray>()) {
                value.tag = RigExecWirePathValue::Tag::DoubleArray;
                const VtDoubleArray &array =
                    held.UncheckedGet<VtDoubleArray>();
                value.doubles.assign(array.begin(), array.end());
            } else {
                return Fail(std::string("unencodable path read of type ") +
                            held.GetTypeName() + " at " +
                            entry.first.first.GetString());
            }
            record.pathReads.push_back(std::move(read));
        }
    }
    _table.frames.push_back(std::move(record));
    return true;
}

}  // namespace rigExec
