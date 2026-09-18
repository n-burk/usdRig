//
// .rigexec DomainPose section: wire encoding.
//

#include "rigExecBinary/pose.h"

namespace rigExec {
namespace {

void
_PutI32s(std::vector<uint8_t> *out, const std::vector<int32_t> &values)
{
    RigExecWirePutU32(out, uint32_t(values.size()));
    for (int32_t value : values) {
        RigExecWirePutI32(out, value);
    }
}

bool
_ReadI32s(RigExecWireReader *reader, std::vector<int32_t> *values)
{
    uint32_t count = 0;
    if (!reader->ReadU32(&count)) {
        return false;
    }
    values->resize(count);
    for (uint32_t i = 0; i < count; ++i) {
        if (!reader->ReadI32(&(*values)[i])) {
            return false;
        }
    }
    return true;
}

void
_PutU32s(std::vector<uint8_t> *out, const std::vector<uint32_t> &values)
{
    RigExecWirePutU32(out, uint32_t(values.size()));
    for (uint32_t value : values) {
        RigExecWirePutU32(out, value);
    }
}

bool
_ReadU32s(RigExecWireReader *reader, std::vector<uint32_t> *values)
{
    uint32_t count = 0;
    if (!reader->ReadU32(&count)) {
        return false;
    }
    values->resize(count);
    for (uint32_t i = 0; i < count; ++i) {
        if (!reader->ReadU32(&(*values)[i])) {
            return false;
        }
    }
    return true;
}

void
_PutU8s(std::vector<uint8_t> *out, const std::vector<uint8_t> &values)
{
    RigExecWirePutU32(out, uint32_t(values.size()));
    for (uint8_t value : values) {
        RigExecWirePutU8(out, value);
    }
}

bool
_ReadU8s(RigExecWireReader *reader, std::vector<uint8_t> *values)
{
    uint32_t count = 0;
    if (!reader->ReadU32(&count)) {
        return false;
    }
    values->resize(count);
    for (uint32_t i = 0; i < count; ++i) {
        if (!reader->ReadU8(&(*values)[i])) {
            return false;
        }
    }
    return true;
}

// A u8 list that only holds 0/1: every other value is corruption, not a
// truthy flag.
bool
_ReadBools(RigExecWireReader *reader, std::vector<uint8_t> *values)
{
    if (!_ReadU8s(reader, values)) {
        return false;
    }
    for (uint8_t value : *values) {
        if (value > 1) {
            return false;
        }
    }
    return true;
}

// Pose-type bytes, in RigExecRbfPoseType range (Whole/Swing/Twist).
bool
_ReadPoseTypes(RigExecWireReader *reader, std::vector<uint8_t> *values)
{
    if (!_ReadU8s(reader, values)) {
        return false;
    }
    for (uint8_t value : *values) {
        if (value > 2) {
            return false;
        }
    }
    return true;
}

void
_PutF64s(std::vector<uint8_t> *out, const std::vector<double> &values)
{
    RigExecWirePutU32(out, uint32_t(values.size()));
    for (double value : values) {
        RigExecWirePutF64(out, value);
    }
}

bool
_ReadF64s(RigExecWireReader *reader, std::vector<double> *values)
{
    uint32_t count = 0;
    if (!reader->ReadU32(&count)) {
        return false;
    }
    values->resize(count);
    for (uint32_t i = 0; i < count; ++i) {
        if (!reader->ReadF64(&(*values)[i])) {
            return false;
        }
    }
    return true;
}

void
_PutPairs(std::vector<uint8_t> *out,
          const std::vector<std::pair<int32_t, int32_t>> &values)
{
    RigExecWirePutU32(out, uint32_t(values.size()));
    for (const auto &pair : values) {
        RigExecWirePutI32(out, pair.first);
        RigExecWirePutI32(out, pair.second);
    }
}

bool
_ReadPairs(RigExecWireReader *reader,
           std::vector<std::pair<int32_t, int32_t>> *values)
{
    uint32_t count = 0;
    if (!reader->ReadU32(&count)) {
        return false;
    }
    values->resize(count);
    for (uint32_t i = 0; i < count; ++i) {
        if (!reader->ReadI32(&(*values)[i].first) ||
            !reader->ReadI32(&(*values)[i].second)) {
            return false;
        }
    }
    return true;
}

void
_PutVec3ds(std::vector<uint8_t> *out,
           const std::vector<RigExecWireVec3d> &values)
{
    RigExecWirePutU32(out, uint32_t(values.size()));
    for (const RigExecWireVec3d &value : values) {
        RigExecWirePutVec3d(out, value);
    }
}

bool
_ReadVec3ds(RigExecWireReader *reader,
            std::vector<RigExecWireVec3d> *values)
{
    uint32_t count = 0;
    if (!reader->ReadU32(&count)) {
        return false;
    }
    values->resize(count);
    for (uint32_t i = 0; i < count; ++i) {
        if (!RigExecWireReadVec3d(reader, &(*values)[i])) {
            return false;
        }
    }
    return true;
}

void
_PutVec3fs(std::vector<uint8_t> *out,
           const std::vector<RigExecWireVec3f> &values)
{
    RigExecWirePutU32(out, uint32_t(values.size()));
    for (const RigExecWireVec3f &value : values) {
        RigExecWirePutVec3f(out, value);
    }
}

bool
_ReadVec3fs(RigExecWireReader *reader,
            std::vector<RigExecWireVec3f> *values)
{
    uint32_t count = 0;
    if (!reader->ReadU32(&count)) {
        return false;
    }
    values->resize(count);
    for (uint32_t i = 0; i < count; ++i) {
        if (!RigExecWireReadVec3f(reader, &(*values)[i])) {
            return false;
        }
    }
    return true;
}

void
_PutFrames(std::vector<uint8_t> *out,
           const std::vector<RigExecWireFrame> &values)
{
    RigExecWirePutU32(out, uint32_t(values.size()));
    for (const RigExecWireFrame &value : values) {
        RigExecWirePutFrame(out, value);
    }
}

bool
_ReadFrames(RigExecWireReader *reader,
            std::vector<RigExecWireFrame> *values)
{
    uint32_t count = 0;
    if (!reader->ReadU32(&count)) {
        return false;
    }
    values->resize(count);
    for (uint32_t i = 0; i < count; ++i) {
        if (!RigExecWireReadFrame(reader, &(*values)[i])) {
            return false;
        }
    }
    return true;
}

void
_PutLandmarkSets(std::vector<uint8_t> *out,
                 const std::vector<std::array<RigExecWireVec3d, 4>> &values)
{
    RigExecWirePutU32(out, uint32_t(values.size()));
    for (const std::array<RigExecWireVec3d, 4> &set : values) {
        for (const RigExecWireVec3d &point : set) {
            RigExecWirePutVec3d(out, point);
        }
    }
}

bool
_ReadLandmarkSets(RigExecWireReader *reader,
                  std::vector<std::array<RigExecWireVec3d, 4>> *values)
{
    uint32_t count = 0;
    if (!reader->ReadU32(&count)) {
        return false;
    }
    values->resize(count);
    for (uint32_t i = 0; i < count; ++i) {
        for (RigExecWireVec3d &point : (*values)[i]) {
            if (!RigExecWireReadVec3d(reader, &point)) {
                return false;
            }
        }
    }
    return true;
}

void
_PutLandmarkSet(std::vector<uint8_t> *out,
                const std::array<RigExecWireVec3d, 4> &set)
{
    for (const RigExecWireVec3d &point : set) {
        RigExecWirePutVec3d(out, point);
    }
}

bool
_ReadLandmarkSet(RigExecWireReader *reader,
                 std::array<RigExecWireVec3d, 4> *set)
{
    for (RigExecWireVec3d &point : *set) {
        if (!RigExecWireReadVec3d(reader, &point)) {
            return false;
        }
    }
    return true;
}

void
_PutInput(std::vector<uint8_t> *out, const RigExecWireInput &input)
{
    RigExecWirePutU8(out, uint8_t(input.tag));
    switch (input.tag) {
    case RigExecWireInput::Tag::Double:
        RigExecWirePutF64(out, input.f64);
        break;
    case RigExecWireInput::Tag::Float:
        RigExecWirePutF32(out, input.f32);
        break;
    case RigExecWireInput::Tag::Bool:
        RigExecWirePutU8(out, input.boolean ? uint8_t(1) : uint8_t(0));
        break;
    case RigExecWireInput::Tag::Int:
        RigExecWirePutI32(out, input.i32);
        break;
    case RigExecWireInput::Tag::Matrix4d:
        RigExecWirePutMatrix4d(out, input.matrix);
        break;
    case RigExecWireInput::Tag::Token:
        RigExecWirePutU32(out, input.token);
        break;
    case RigExecWireInput::Tag::Vec3d:
        RigExecWirePutVec3d(out, input.vec);
        break;
    }
    const uint8_t flags =
        uint8_t(input.varying ? 1 : 0) | uint8_t(input.bound ? 2 : 0) |
        uint8_t(input.viaResolved ? 4 : 0);
    RigExecWirePutU8(out, flags);
    RigExecWirePutI32(out, input.overrideIndex);
    RigExecWirePutU32(out, input.head);
}

bool
_ReadInput(RigExecWireReader *reader, RigExecWireInput *input)
{
    uint8_t tag = 0;
    if (!reader->ReadU8(&tag) ||
        tag > uint8_t(RigExecWireInput::Tag::Vec3d)) {
        return false;
    }
    input->tag = RigExecWireInput::Tag(tag);
    uint8_t boolean = 0;
    switch (input->tag) {
    case RigExecWireInput::Tag::Double:
        if (!reader->ReadF64(&input->f64)) {
            return false;
        }
        break;
    case RigExecWireInput::Tag::Float:
        if (!reader->ReadF32(&input->f32)) {
            return false;
        }
        break;
    case RigExecWireInput::Tag::Bool:
        if (!reader->ReadU8(&boolean) || boolean > 1) {
            return false;
        }
        input->boolean = boolean != 0;
        break;
    case RigExecWireInput::Tag::Int:
        if (!reader->ReadI32(&input->i32)) {
            return false;
        }
        break;
    case RigExecWireInput::Tag::Matrix4d:
        if (!RigExecWireReadMatrix4d(reader, &input->matrix)) {
            return false;
        }
        break;
    case RigExecWireInput::Tag::Token:
        if (!reader->ReadU32(&input->token)) {
            return false;
        }
        break;
    case RigExecWireInput::Tag::Vec3d:
        if (!RigExecWireReadVec3d(reader, &input->vec)) {
            return false;
        }
        break;
    }
    uint8_t flags = 0;
    if (!reader->ReadU8(&flags) || (flags & ~uint8_t(7)) != 0 ||
        !reader->ReadI32(&input->overrideIndex) ||
        !reader->ReadU32(&input->head)) {
        return false;
    }
    input->varying = (flags & 1) != 0;
    input->bound = (flags & 2) != 0;
    input->viaResolved = (flags & 4) != 0;
    return true;
}

void
_PutRbf(std::vector<uint8_t> *out, const RigExecWireRbf &rbf)
{
    _PutVec3ds(out, rbf.poses);
    _PutVec3ds(out, rbf.translations);
    _PutU8s(out, rbf.poseTypes);
    RigExecWirePutVec3d(out, rbf.twistAxis);
    RigExecWirePutU8(out, rbf.kernel);
    RigExecWirePutF64(out, rbf.radius);
    RigExecWirePutF64(out, rbf.translationRadius);
    _PutF64s(out, rbf.radii);
    _PutF64s(out, rbf.translationRadii);
    RigExecWirePutF64(out, rbf.regularization);
    const uint8_t flags =
        uint8_t(rbf.normalize ? 1 : 0) |
        uint8_t(rbf.enableRotation ? 2 : 0) |
        uint8_t(rbf.enableTranslation ? 4 : 0) |
        uint8_t(rbf.regularizedSingular ? 8 : 0);
    RigExecWirePutU8(out, flags);
    RigExecWirePutU32(out, uint32_t(rbf.weights.size()));
    for (const std::vector<double> &row : rbf.weights) {
        _PutF64s(out, row);
    }
}

bool
_ReadRbf(RigExecWireReader *reader, RigExecWireRbf *rbf)
{
    uint8_t flags = 0;
    uint32_t rows = 0;
    if (!_ReadVec3ds(reader, &rbf->poses) ||
        !_ReadVec3ds(reader, &rbf->translations) ||
        !_ReadPoseTypes(reader, &rbf->poseTypes) ||
        !RigExecWireReadVec3d(reader, &rbf->twistAxis) ||
        !reader->ReadU8(&rbf->kernel) || rbf->kernel > 1 ||
        !reader->ReadF64(&rbf->radius) ||
        !reader->ReadF64(&rbf->translationRadius) ||
        !_ReadF64s(reader, &rbf->radii) ||
        !_ReadF64s(reader, &rbf->translationRadii) ||
        !reader->ReadF64(&rbf->regularization) || !reader->ReadU8(&flags) ||
        (flags & ~uint8_t(15)) != 0 || !reader->ReadU32(&rows)) {
        return false;
    }
    rbf->normalize = (flags & 1) != 0;
    rbf->enableRotation = (flags & 2) != 0;
    rbf->enableTranslation = (flags & 4) != 0;
    rbf->regularizedSingular = (flags & 8) != 0;
    rbf->weights.resize(rows);
    for (uint32_t i = 0; i < rows; ++i) {
        if (!_ReadF64s(reader, &rbf->weights[i])) {
            return false;
        }
    }
    return true;
}

void
_PutAncestorRead(std::vector<uint8_t> *out,
                 const RigExecWireAncestorRead &read)
{
    RigExecWirePutI32(out, read.slot);
    RigExecWirePutU32(out, read.fin);
    RigExecWirePutU32(out, read.base);
}

bool
_ReadAncestorRead(RigExecWireReader *reader, RigExecWireAncestorRead *read)
{
    return reader->ReadI32(&read->slot) && reader->ReadU32(&read->fin) &&
           reader->ReadU32(&read->base);
}

void
_PutAncestorReads(std::vector<uint8_t> *out,
                  const std::vector<RigExecWireAncestorRead> &values)
{
    RigExecWirePutU32(out, uint32_t(values.size()));
    for (const RigExecWireAncestorRead &read : values) {
        _PutAncestorRead(out, read);
    }
}

bool
_ReadAncestorReads(RigExecWireReader *reader,
                   std::vector<RigExecWireAncestorRead> *values)
{
    uint32_t count = 0;
    if (!reader->ReadU32(&count)) {
        return false;
    }
    values->resize(count);
    for (uint32_t i = 0; i < count; ++i) {
        if (!_ReadAncestorRead(reader, &(*values)[i])) {
            return false;
        }
    }
    return true;
}

void
_PutAncestorReadsNested(
    std::vector<uint8_t> *out,
    const std::vector<std::vector<RigExecWireAncestorRead>> &values)
{
    RigExecWirePutU32(out, uint32_t(values.size()));
    for (const std::vector<RigExecWireAncestorRead> &row : values) {
        _PutAncestorReads(out, row);
    }
}

bool
_ReadAncestorReadsNested(
    RigExecWireReader *reader,
    std::vector<std::vector<RigExecWireAncestorRead>> *values)
{
    uint32_t count = 0;
    if (!reader->ReadU32(&count)) {
        return false;
    }
    values->resize(count);
    for (uint32_t i = 0; i < count; ++i) {
        if (!_ReadAncestorReads(reader, &(*values)[i])) {
            return false;
        }
    }
    return true;
}

bool
_Fail(std::string *error)
{
    if (error) {
        *error = "malformed pose tables";
    }
    return false;
}

}  // namespace

bool
RigExecWireEncodeDomainPose(const RigExecWireDomainPose &pose,
                            std::vector<uint8_t> *out)
{
    RigExecWirePutU32(out, uint32_t(pose.ladders.size()));
    for (const RigExecWireLadder &ladder : pose.ladders) {
        _PutInput(out, ladder.restSpace);
        _PutInput(out, ladder.defaultSpace);
        _PutInput(out, ladder.posedSpace);
        for (const RigExecWireInput &input : ladder.restAvars) {
            _PutInput(out, input);
        }
        for (const RigExecWireInput &input : ladder.defaultAvars) {
            _PutInput(out, input);
        }
        _PutInput(out, ladder.rotationOrder);
    }
    RigExecWirePutU8(out, pose.ladderVarying ? uint8_t(1) : uint8_t(0));
    _PutI32s(out, pose.ladderOverrides);
    _PutU8s(out, pose.restChainVaries);
    RigExecWirePutU32(out, uint32_t(pose.poseInterpolators.size()));
    for (const RigExecWirePoseInterpolator &interp : pose.poseInterpolators) {
        RigExecWirePutU32(out, interp.path);
        RigExecWirePutI32(out, interp.driverSlot);
        RigExecWirePutI32(out, interp.parentSlot);
        RigExecWirePutU8(out, interp.allowNegativeWeights ? uint8_t(1)
                                                          : uint8_t(0));
        _PutInput(out, interp.enabled);
        RigExecWirePutI32(out, interp.weightBegin);
        RigExecWirePutI32(out, interp.weightEnd);
        _PutI32s(out, interp.poseSlots);
        _PutI32s(out, interp.disabledSlots);
        _PutRbf(out, interp.solver);
    }
    _PutU32s(out, pose.poseWeightPaths);
    RigExecWirePutU32(out, uint32_t(pose.solvers.size()));
    for (const RigExecWireSolver &solver : pose.solvers) {
        RigExecWirePutU32(out, solver.path);
        RigExecWirePutU32(out, solver.type);
        _PutI32s(out, solver.restSlots);
        _PutPairs(out, solver.restRefs);
        _PutU8s(out, solver.restIsLive);
        _PutU32s(out, solver.restReads);
        RigExecWirePutU8(out, solver.hasLiveRest ? uint8_t(1) : uint8_t(0));
        _PutLandmarkSets(out, solver.jointRests);
        RigExecWirePutU8(out, solver.restsVary ? uint8_t(1) : uint8_t(0));
        _PutI32s(out, solver.restOverrides);
        _PutF64s(out, solver.splineRestWeights);
        RigExecWirePutU8(out, solver.splineRestMode);
        RigExecWirePutU8(out, solver.degenerate ? uint8_t(1) : uint8_t(0));
        _PutI32s(out, solver.controls);
        RigExecWirePutU8(out, solver.parentRelative ? uint8_t(1)
                                                    : uint8_t(0));
        _PutLandmarkSets(out, solver.controlRests);
        RigExecWirePutI32(out, solver.root);
        RigExecWirePutI32(out, solver.mid);
        RigExecWirePutI32(out, solver.end);
        RigExecWirePutI32(out, solver.pole);
        for (const std::array<RigExecWireVec3d, 4> &set : solver.ikRests) {
            _PutLandmarkSet(out, set);
        }
        RigExecWirePutF64(out, solver.ikParams.upperLength);
        RigExecWirePutF64(out, solver.ikParams.lowerLength);
        RigExecWirePutF64(out, solver.ikParams.stretch);
        RigExecWirePutF64(out, solver.ikParams.softness);
        RigExecWirePutF64(out, solver.ikParams.preferredBendRadians);
        _PutInput(out, solver.bend);
        _PutInput(out, solver.upperOffset);
        _PutInput(out, solver.lowerOffset);
        _PutInput(out, solver.stretch);
        _PutInput(out, solver.softness);
        RigExecWirePutF64(out, solver.upperLengthBase);
        RigExecWirePutF64(out, solver.lowerLengthBase);
        RigExecWirePutI32(out, solver.inA);
        RigExecWirePutI32(out, solver.inB);
        _PutInput(out, solver.blendWeight);
        RigExecWirePutU8(out, solver.scaleMode);
        RigExecWirePutU8(out, solver.blendRotationRejected ? uint8_t(1)
                                                           : uint8_t(0));
        _PutLandmarkSet(out, solver.splineRest.cvs);
        RigExecWirePutFrame(out, solver.splineRest.rootControl);
        RigExecWirePutFrame(out, solver.splineRest.midControl);
        RigExecWirePutFrame(out, solver.splineRest.endControl);
        _PutFrames(out, solver.splineRest.joints);
        _PutF64s(out, solver.splineRest.segmentLengths);
        RigExecWirePutF64(out, solver.splineRest.restArcLength);
        _PutF64s(out, solver.splineRest.volumeWeights);
        RigExecWirePutF64(out, solver.splineParams.preserveVolume);
        RigExecWirePutF64(out, solver.splineParams.midFollowWeight);
        RigExecWirePutF64(out, solver.splineParams.roll);
        RigExecWirePutF64(out, solver.splineParams.twist);
        RigExecWirePutF64(out, solver.splineParams.minLengthRatio);
        RigExecWirePutU8(out, solver.splineParams.aimRootTangent
                                    ? uint8_t(1)
                                    : uint8_t(0));
        _PutLandmarkSets(out, solver.splineJointRests);
        RigExecWirePutU64(out, solver.splineCount);
        _PutInput(out, solver.preserveVolume);
        _PutInput(out, solver.midFollowWeight);
        _PutInput(out, solver.roll);
        _PutInput(out, solver.twist);
        _PutInput(out, solver.minLengthRatio);
        RigExecWirePutU8(out, solver.splineParamsVary ? uint8_t(1)
                                                      : uint8_t(0));
        _PutLandmarkSet(out, solver.twistStartRest);
        _PutLandmarkSet(out, solver.twistEndRest);
        _PutF64s(out, solver.twistWeights);
        _PutInput(out, solver.twistTurns);
        RigExecWirePutU32(out, solver.ribbonPointsPath);
        _PutVec3fs(out, solver.ribbonRestPoints);
        _PutVec3fs(out, solver.ribbonConstantPoints);
        RigExecWirePutU8(out, solver.ribbonPointsVarying ? uint8_t(1)
                                                         : uint8_t(0));
        _PutInput(out, solver.ribbonSampleCount);
        _PutPairs(out, solver.outputs);
        _PutI32s(out, solver.outPosition);
        _PutU32s(out, solver.controlReads);
        RigExecWirePutU32(out, solver.rootRead);
        RigExecWirePutU32(out, solver.midRead);
        RigExecWirePutU32(out, solver.endRead);
        RigExecWirePutU32(out, solver.poleRead);
    }
    _PutI32s(out, pose.guideSolvers);
    RigExecWirePutU32(out, uint32_t(pose.constraints.size()));
    for (const RigExecWireConstraint &constraint : pose.constraints) {
        RigExecWirePutU32(out, constraint.path);
        RigExecWirePutU32(out, constraint.type);
        RigExecWirePutU32(out, constraint.weightObject);
        RigExecWirePutI32(out, constraint.target);
        _PutI32s(out, constraint.targetSlots);
        _PutU8s(out, constraint.snapshotTargets);
        _PutI32s(out, constraint.sources);
        _PutI32s(out, constraint.sourceNatives);
        _PutU32s(out, constraint.sourcePaths);
        RigExecWirePutI32(out, constraint.arrays);
        _PutInput(out, constraint.enabled);
        _PutInput(out, constraint.defaultWeight);
        _PutInput(out, constraint.offset);
        _PutInput(out, constraint.affectX);
        _PutInput(out, constraint.affectY);
        _PutInput(out, constraint.affectZ);
        _PutInput(out, constraint.tX);
        _PutInput(out, constraint.tY);
        _PutInput(out, constraint.tZ);
        _PutInput(out, constraint.rX);
        _PutInput(out, constraint.rY);
        _PutInput(out, constraint.rZ);
        _PutInput(out, constraint.sX);
        _PutInput(out, constraint.sY);
        _PutInput(out, constraint.sZ);
        RigExecWirePutU8(out, constraint.order);
        _PutInput(out, constraint.aimVector);
        _PutInput(out, constraint.upVector);
        _PutInput(out, constraint.rotationOffset);
        _PutInput(out, constraint.worldUpVector);
        RigExecWirePutVec3d(out, constraint.aimAxisFallback);
        RigExecWirePutU8(out, constraint.aimVectorAuthored ? uint8_t(1)
                                                           : uint8_t(0));
        RigExecWirePutU8(out, constraint.preserveInputUp ? uint8_t(1)
                                                         : uint8_t(0));
        RigExecWirePutU32(out, constraint.worldUpType);
        RigExecWirePutVec3d(out, constraint.sceneUp);
        RigExecWirePutU32(out, constraint.pointsTarget);
        RigExecWirePutU32(out, constraint.deltaBasePath);
        RigExecWirePutI32(out, constraint.deltaBase);
        RigExecWirePutI32(out, constraint.worldUpObject);
        RigExecWirePutI32(out, constraint.worldUpNative);
        RigExecWirePutU32(out, constraint.worldUpPath);
        RigExecWirePutU8(out, constraint.worldUpObjectNamed ? uint8_t(1)
                                                            : uint8_t(0));
        RigExecWirePutU8(out, constraint.snapshotAfter ? uint8_t(1)
                                                       : uint8_t(0));
        RigExecWirePutU8(out, constraint.singleChainIk ? uint8_t(1)
                                                       : uint8_t(0));
        RigExecWirePutU8(out, constraint.ikMode);
        RigExecWirePutU8(out, constraint.poleModeObject ? uint8_t(1)
                                                        : uint8_t(0));
        RigExecWirePutU8(out, constraint.useAnimatedTs ? uint8_t(1)
                                                       : uint8_t(0));
        _PutU8s(out, constraint.ikRestLive);
        RigExecWirePutI32(out, constraint.effector);
        RigExecWirePutI32(out, constraint.effectorNative);
        RigExecWirePutU32(out, constraint.effectorPath);
        _PutI32s(out, constraint.poleObjects);
        _PutI32s(out, constraint.poleObjectNatives);
        _PutInput(out, constraint.poleVector);
        _PutInput(out, constraint.twistDegrees);
    }
    RigExecWirePutU32(out, uint32_t(pose.constraintArrays.size()));
    for (const RigExecWireConstraintArrays &arrays : pose.constraintArrays) {
        RigExecWirePutU32(out, arrays.prim);
        RigExecWirePutU64(out, arrays.sourceCount);
        RigExecWirePutU8(out, arrays.parentOffsets ? uint8_t(1)
                                                   : uint8_t(0));
        RigExecWirePutU8(out, arrays.readPole ? uint8_t(1) : uint8_t(0));
        RigExecWirePutU64(out, arrays.poleCount);
    }
    RigExecWirePutU32(out, uint32_t(pose.nativeSources.size()));
    for (const RigExecWireNativeSource &source : pose.nativeSources) {
        RigExecWirePutU32(out, source.path);
        _PutI32s(out, source.ancestorSlots);
    }
    RigExecWirePutU32(out, uint32_t(pose.walkSteps.size()));
    for (const RigExecWireWalkStep &step : pose.walkSteps) {
        RigExecWirePutU8(out, step.solverBatch ? uint8_t(1) : uint8_t(0));
        RigExecWirePutU64(out, step.level);
        RigExecWirePutI32(out, step.index);
        _PutI32s(out, step.batchSolvers);
        _PutPairs(out, step.propagate);
    }
    RigExecWirePutU32(out, uint32_t(pose.composeGroups.size()));
    for (const RigExecWireComposeGroup &group : pose.composeGroups) {
        RigExecWirePutI32(out, group.begin);
        RigExecWirePutI32(out, group.end);
        _PutI32s(out, group.parentSlots);
    }
    RigExecWirePutU32(out, uint32_t(pose.commits.size()));
    for (const RigExecWireCommit &commit : pose.commits) {
        RigExecWirePutU32(out, commit.moverPath);
        RigExecWirePutU8(out, commit.solverOutput ? uint8_t(1)
                                                  : uint8_t(0));
        _PutI32s(out, commit.slots);
        _PutPairs(out, commit.propagate);
        _PutI32s(out, commit.closestPos);
        RigExecWirePutU8(out, commit.split ? uint8_t(1) : uint8_t(0));
        RigExecWirePutI32(out, commit.stagingBase);
        RigExecWirePutU32(out, uint32_t(commit.sources.size()));
        for (const RigExecWireConstraintSource &source : commit.sources) {
            RigExecWirePutFrame(out, source.frame);
            RigExecWirePutF64(out, source.normalizedWeight);
            RigExecWirePutVec3d(out, source.translationOffset);
            RigExecWirePutVec3d(out, source.rotationOffsetDegrees);
        }
        _PutU32s(out, commit.slotReads);
        _PutU32s(out, commit.slotWrites);
        _PutU32s(out, commit.slotBaseWrites);
        _PutU32s(out, commit.descendantReads);
        _PutU32s(out, commit.closestReads);
        _PutU32s(out, commit.descendantWrites);
        _PutU32s(out, commit.descendantBaseWrites);
        _PutU32s(out, commit.slotCarry);
        _PutU32s(out, commit.slotBaseCarry);
        _PutU32s(out, commit.descendantCarry);
        _PutU32s(out, commit.descendantBaseCarry);
        _PutU32s(out, commit.sourceReads);
        RigExecWirePutU32(out, commit.worldUpRead);
        RigExecWirePutU32(out, commit.targetRead);
        _PutU32s(out, commit.targetReads);
        RigExecWirePutU32(out, commit.effectorRead);
        _PutAncestorReads(out, commit.effectorAncestors);
        _PutU32s(out, commit.poleReads);
        _PutAncestorReadsNested(out, commit.poleAncestors);
        RigExecWirePutU8(out, commit.recordAfter ? uint8_t(1) : uint8_t(0));
        RigExecWirePutU8(out, commit.recordEveryTarget ? uint8_t(1)
                                                       : uint8_t(0));
        _PutAncestorReadsNested(out, commit.sourceAncestors);
        _PutAncestorReads(out, commit.worldUpAncestors);
    }
    _PutU32s(out, pose.jointBindingJoints);
    RigExecWirePutU32(out, uint32_t(pose.jointBindingSolvers.size()));
    for (const std::vector<uint32_t> &row : pose.jointBindingSolvers) {
        _PutU32s(out, row);
    }
    RigExecWirePutU32(out, uint32_t(pose.jointBindingElements.size()));
    for (const std::vector<int32_t> &row : pose.jointBindingElements) {
        _PutI32s(out, row);
    }
    RigExecWirePutU8(out, pose.hasPropertyChains ? uint8_t(1) : uint8_t(0));
    RigExecWirePutU8(out, pose.phasedReads ? uint8_t(1) : uint8_t(0));
    RigExecWirePutU8(out, pose.publishWeightFields ? uint8_t(1)
                                                   : uint8_t(0));
    return true;
}

bool
RigExecWireDecodeDomainPose(RigExecWireReader *reader,
                            RigExecWireDomainPose *pose, std::string *error)
{
    uint32_t count = 0;
    uint8_t flag = 0;
    if (!reader->ReadU32(&count)) {
        return _Fail(error);
    }
    pose->ladders.resize(count);
    for (uint32_t i = 0; i < count; ++i) {
        RigExecWireLadder &ladder = pose->ladders[i];
        if (!_ReadInput(reader, &ladder.restSpace) ||
            !_ReadInput(reader, &ladder.defaultSpace) ||
            !_ReadInput(reader, &ladder.posedSpace)) {
            return _Fail(error);
        }
        for (RigExecWireInput &input : ladder.restAvars) {
            if (!_ReadInput(reader, &input)) {
                return _Fail(error);
            }
        }
        for (RigExecWireInput &input : ladder.defaultAvars) {
            if (!_ReadInput(reader, &input)) {
                return _Fail(error);
            }
        }
        if (!_ReadInput(reader, &ladder.rotationOrder)) {
            return _Fail(error);
        }
    }
    if (!reader->ReadU8(&flag) || flag > 1) {
        return _Fail(error);
    }
    pose->ladderVarying = flag != 0;
    if (!_ReadI32s(reader, &pose->ladderOverrides) ||
        !_ReadBools(reader, &pose->restChainVaries) ||
        !reader->ReadU32(&count)) {
        return _Fail(error);
    }
    pose->poseInterpolators.resize(count);
    for (uint32_t i = 0; i < count; ++i) {
        RigExecWirePoseInterpolator &interp = pose->poseInterpolators[i];
        if (!reader->ReadU32(&interp.path) ||
            !reader->ReadI32(&interp.driverSlot) ||
            !reader->ReadI32(&interp.parentSlot) ||
            !reader->ReadU8(&flag) || flag > 1) {
            return _Fail(error);
        }
        interp.allowNegativeWeights = flag != 0;
        if (!_ReadInput(reader, &interp.enabled) ||
            !reader->ReadI32(&interp.weightBegin) ||
            !reader->ReadI32(&interp.weightEnd) ||
            !_ReadI32s(reader, &interp.poseSlots) ||
            !_ReadI32s(reader, &interp.disabledSlots) ||
            !_ReadRbf(reader, &interp.solver)) {
            return _Fail(error);
        }
    }
    if (!_ReadU32s(reader, &pose->poseWeightPaths) ||
        !reader->ReadU32(&count)) {
        return _Fail(error);
    }
    pose->solvers.resize(count);
    for (uint32_t i = 0; i < count; ++i) {
        RigExecWireSolver &solver = pose->solvers[i];
        if (!reader->ReadU32(&solver.path) ||
            !reader->ReadU32(&solver.type) ||
            !_ReadI32s(reader, &solver.restSlots) ||
            !_ReadPairs(reader, &solver.restRefs) ||
            !_ReadBools(reader, &solver.restIsLive) ||
            !_ReadU32s(reader, &solver.restReads) ||
            !reader->ReadU8(&flag) || flag > 1) {
            return _Fail(error);
        }
        solver.hasLiveRest = flag != 0;
        if (!_ReadLandmarkSets(reader, &solver.jointRests) ||
            !reader->ReadU8(&flag) || flag > 1) {
            return _Fail(error);
        }
        solver.restsVary = flag != 0;
        if (!_ReadI32s(reader, &solver.restOverrides) ||
            !_ReadF64s(reader, &solver.splineRestWeights) ||
            !reader->ReadU8(&solver.splineRestMode) ||
            solver.splineRestMode > 1 || !reader->ReadU8(&flag) ||
            flag > 1) {
            return _Fail(error);
        }
        solver.degenerate = flag != 0;
        if (!_ReadI32s(reader, &solver.controls) || !reader->ReadU8(&flag) ||
            flag > 1) {
            return _Fail(error);
        }
        solver.parentRelative = flag != 0;
        if (!_ReadLandmarkSets(reader, &solver.controlRests) ||
            !reader->ReadI32(&solver.root) ||
            !reader->ReadI32(&solver.mid) ||
            !reader->ReadI32(&solver.end) ||
            !reader->ReadI32(&solver.pole)) {
            return _Fail(error);
        }
        for (std::array<RigExecWireVec3d, 4> &set : solver.ikRests) {
            if (!_ReadLandmarkSet(reader, &set)) {
                return _Fail(error);
            }
        }
        if (!reader->ReadF64(&solver.ikParams.upperLength) ||
            !reader->ReadF64(&solver.ikParams.lowerLength) ||
            !reader->ReadF64(&solver.ikParams.stretch) ||
            !reader->ReadF64(&solver.ikParams.softness) ||
            !reader->ReadF64(&solver.ikParams.preferredBendRadians) ||
            !_ReadInput(reader, &solver.bend) ||
            !_ReadInput(reader, &solver.upperOffset) ||
            !_ReadInput(reader, &solver.lowerOffset) ||
            !_ReadInput(reader, &solver.stretch) ||
            !_ReadInput(reader, &solver.softness) ||
            !reader->ReadF64(&solver.upperLengthBase) ||
            !reader->ReadF64(&solver.lowerLengthBase) ||
            !reader->ReadI32(&solver.inA) ||
            !reader->ReadI32(&solver.inB) ||
            !_ReadInput(reader, &solver.blendWeight) ||
            !reader->ReadU8(&solver.scaleMode) || solver.scaleMode > 1 ||
            !reader->ReadU8(&flag) || flag > 1) {
            return _Fail(error);
        }
        solver.blendRotationRejected = flag != 0;
        if (!_ReadLandmarkSet(reader, &solver.splineRest.cvs) ||
            !RigExecWireReadFrame(reader, &solver.splineRest.rootControl) ||
            !RigExecWireReadFrame(reader, &solver.splineRest.midControl) ||
            !RigExecWireReadFrame(reader, &solver.splineRest.endControl) ||
            !_ReadFrames(reader, &solver.splineRest.joints) ||
            !_ReadF64s(reader, &solver.splineRest.segmentLengths) ||
            !reader->ReadF64(&solver.splineRest.restArcLength) ||
            !_ReadF64s(reader, &solver.splineRest.volumeWeights) ||
            !reader->ReadF64(&solver.splineParams.preserveVolume) ||
            !reader->ReadF64(&solver.splineParams.midFollowWeight) ||
            !reader->ReadF64(&solver.splineParams.roll) ||
            !reader->ReadF64(&solver.splineParams.twist) ||
            !reader->ReadF64(&solver.splineParams.minLengthRatio) ||
            !reader->ReadU8(&flag) || flag > 1) {
            return _Fail(error);
        }
        solver.splineParams.aimRootTangent = flag != 0;
        if (!_ReadLandmarkSets(reader, &solver.splineJointRests) ||
            !reader->ReadU64(&solver.splineCount) ||
            !_ReadInput(reader, &solver.preserveVolume) ||
            !_ReadInput(reader, &solver.midFollowWeight) ||
            !_ReadInput(reader, &solver.roll) ||
            !_ReadInput(reader, &solver.twist) ||
            !_ReadInput(reader, &solver.minLengthRatio) ||
            !reader->ReadU8(&flag) || flag > 1) {
            return _Fail(error);
        }
        solver.splineParamsVary = flag != 0;
        if (!_ReadLandmarkSet(reader, &solver.twistStartRest) ||
            !_ReadLandmarkSet(reader, &solver.twistEndRest) ||
            !_ReadF64s(reader, &solver.twistWeights) ||
            !_ReadInput(reader, &solver.twistTurns) ||
            !reader->ReadU32(&solver.ribbonPointsPath) ||
            !_ReadVec3fs(reader, &solver.ribbonRestPoints) ||
            !_ReadVec3fs(reader, &solver.ribbonConstantPoints) ||
            !reader->ReadU8(&flag) || flag > 1) {
            return _Fail(error);
        }
        solver.ribbonPointsVarying = flag != 0;
        if (!_ReadInput(reader, &solver.ribbonSampleCount) ||
            !_ReadPairs(reader, &solver.outputs) ||
            !_ReadI32s(reader, &solver.outPosition) ||
            !_ReadU32s(reader, &solver.controlReads) ||
            !reader->ReadU32(&solver.rootRead) ||
            !reader->ReadU32(&solver.midRead) ||
            !reader->ReadU32(&solver.endRead) ||
            !reader->ReadU32(&solver.poleRead)) {
            return _Fail(error);
        }
    }
    if (!_ReadI32s(reader, &pose->guideSolvers) ||
        !reader->ReadU32(&count)) {
        return _Fail(error);
    }
    pose->constraints.resize(count);
    for (uint32_t i = 0; i < count; ++i) {
        RigExecWireConstraint &constraint = pose->constraints[i];
        if (!reader->ReadU32(&constraint.path) ||
            !reader->ReadU32(&constraint.type) ||
            !reader->ReadU32(&constraint.weightObject) ||
            !reader->ReadI32(&constraint.target) ||
            !_ReadI32s(reader, &constraint.targetSlots) ||
            !_ReadBools(reader, &constraint.snapshotTargets) ||
            !_ReadI32s(reader, &constraint.sources) ||
            !_ReadI32s(reader, &constraint.sourceNatives) ||
            !_ReadU32s(reader, &constraint.sourcePaths) ||
            !reader->ReadI32(&constraint.arrays) ||
            !_ReadInput(reader, &constraint.enabled) ||
            !_ReadInput(reader, &constraint.defaultWeight) ||
            !_ReadInput(reader, &constraint.offset) ||
            !_ReadInput(reader, &constraint.affectX) ||
            !_ReadInput(reader, &constraint.affectY) ||
            !_ReadInput(reader, &constraint.affectZ) ||
            !_ReadInput(reader, &constraint.tX) ||
            !_ReadInput(reader, &constraint.tY) ||
            !_ReadInput(reader, &constraint.tZ) ||
            !_ReadInput(reader, &constraint.rX) ||
            !_ReadInput(reader, &constraint.rY) ||
            !_ReadInput(reader, &constraint.rZ) ||
            !_ReadInput(reader, &constraint.sX) ||
            !_ReadInput(reader, &constraint.sY) ||
            !_ReadInput(reader, &constraint.sZ) ||
            !reader->ReadU8(&constraint.order) || constraint.order > 5 ||
            !_ReadInput(reader, &constraint.aimVector) ||
            !_ReadInput(reader, &constraint.upVector) ||
            !_ReadInput(reader, &constraint.rotationOffset) ||
            !_ReadInput(reader, &constraint.worldUpVector) ||
            !RigExecWireReadVec3d(reader, &constraint.aimAxisFallback) ||
            !reader->ReadU8(&flag) || flag > 1) {
            return _Fail(error);
        }
        constraint.aimVectorAuthored = flag != 0;
        if (!reader->ReadU8(&flag) || flag > 1) {
            return _Fail(error);
        }
        constraint.preserveInputUp = flag != 0;
        if (!reader->ReadU32(&constraint.worldUpType) ||
            !RigExecWireReadVec3d(reader, &constraint.sceneUp) ||
            !reader->ReadU32(&constraint.pointsTarget) ||
            !reader->ReadU32(&constraint.deltaBasePath) ||
            !reader->ReadI32(&constraint.deltaBase) ||
            !reader->ReadI32(&constraint.worldUpObject) ||
            !reader->ReadI32(&constraint.worldUpNative) ||
            !reader->ReadU32(&constraint.worldUpPath) ||
            !reader->ReadU8(&flag) || flag > 1) {
            return _Fail(error);
        }
        constraint.worldUpObjectNamed = flag != 0;
        if (!reader->ReadU8(&flag) || flag > 1) {
            return _Fail(error);
        }
        constraint.snapshotAfter = flag != 0;
        if (!reader->ReadU8(&flag) || flag > 1) {
            return _Fail(error);
        }
        constraint.singleChainIk = flag != 0;
        if (!reader->ReadU8(&constraint.ikMode) || constraint.ikMode > 1 ||
            !reader->ReadU8(&flag) || flag > 1) {
            return _Fail(error);
        }
        constraint.poleModeObject = flag != 0;
        if (!reader->ReadU8(&flag) || flag > 1) {
            return _Fail(error);
        }
        constraint.useAnimatedTs = flag != 0;
        if (!_ReadBools(reader, &constraint.ikRestLive) ||
            !reader->ReadI32(&constraint.effector) ||
            !reader->ReadI32(&constraint.effectorNative) ||
            !reader->ReadU32(&constraint.effectorPath) ||
            !_ReadI32s(reader, &constraint.poleObjects) ||
            !_ReadI32s(reader, &constraint.poleObjectNatives) ||
            !_ReadInput(reader, &constraint.poleVector) ||
            !_ReadInput(reader, &constraint.twistDegrees)) {
            return _Fail(error);
        }
    }
    if (!reader->ReadU32(&count)) {
        return _Fail(error);
    }
    pose->constraintArrays.resize(count);
    for (uint32_t i = 0; i < count; ++i) {
        RigExecWireConstraintArrays &arrays = pose->constraintArrays[i];
        if (!reader->ReadU32(&arrays.prim) ||
            !reader->ReadU64(&arrays.sourceCount) ||
            !reader->ReadU8(&flag) || flag > 1) {
            return _Fail(error);
        }
        arrays.parentOffsets = flag != 0;
        if (!reader->ReadU8(&flag) || flag > 1 ||
            !reader->ReadU64(&arrays.poleCount)) {
            return _Fail(error);
        }
        arrays.readPole = flag != 0;
    }
    if (!reader->ReadU32(&count)) {
        return _Fail(error);
    }
    pose->nativeSources.resize(count);
    for (uint32_t i = 0; i < count; ++i) {
        RigExecWireNativeSource &source = pose->nativeSources[i];
        if (!reader->ReadU32(&source.path) ||
            !_ReadI32s(reader, &source.ancestorSlots)) {
            return _Fail(error);
        }
    }
    if (!reader->ReadU32(&count)) {
        return _Fail(error);
    }
    pose->walkSteps.resize(count);
    for (uint32_t i = 0; i < count; ++i) {
        RigExecWireWalkStep &step = pose->walkSteps[i];
        if (!reader->ReadU8(&flag) || flag > 1 ||
            !reader->ReadU64(&step.level) ||
            !reader->ReadI32(&step.index) ||
            !_ReadI32s(reader, &step.batchSolvers) ||
            !_ReadPairs(reader, &step.propagate)) {
            return _Fail(error);
        }
        step.solverBatch = flag != 0;
    }
    if (!reader->ReadU32(&count)) {
        return _Fail(error);
    }
    pose->composeGroups.resize(count);
    for (uint32_t i = 0; i < count; ++i) {
        RigExecWireComposeGroup &group = pose->composeGroups[i];
        if (!reader->ReadI32(&group.begin) ||
            !reader->ReadI32(&group.end) ||
            !_ReadI32s(reader, &group.parentSlots)) {
            return _Fail(error);
        }
    }
    if (!reader->ReadU32(&count)) {
        return _Fail(error);
    }
    pose->commits.resize(count);
    for (uint32_t i = 0; i < count; ++i) {
        RigExecWireCommit &commit = pose->commits[i];
        uint32_t sources = 0;
        if (!reader->ReadU32(&commit.moverPath) || !reader->ReadU8(&flag) ||
            flag > 1) {
            return _Fail(error);
        }
        commit.solverOutput = flag != 0;
        if (!_ReadI32s(reader, &commit.slots) ||
            !_ReadPairs(reader, &commit.propagate) ||
            !_ReadI32s(reader, &commit.closestPos) ||
            !reader->ReadU8(&flag) || flag > 1) {
            return _Fail(error);
        }
        commit.split = flag != 0;
        if (!reader->ReadI32(&commit.stagingBase) ||
            !reader->ReadU32(&sources)) {
            return _Fail(error);
        }
        commit.sources.resize(sources);
        for (uint32_t s = 0; s < sources; ++s) {
            RigExecWireConstraintSource &source = commit.sources[s];
            if (!RigExecWireReadFrame(reader, &source.frame) ||
                !reader->ReadF64(&source.normalizedWeight) ||
                !RigExecWireReadVec3d(reader, &source.translationOffset) ||
                !RigExecWireReadVec3d(reader,
                                       &source.rotationOffsetDegrees)) {
                return _Fail(error);
            }
        }
        if (!_ReadU32s(reader, &commit.slotReads) ||
            !_ReadU32s(reader, &commit.slotWrites) ||
            !_ReadU32s(reader, &commit.slotBaseWrites) ||
            !_ReadU32s(reader, &commit.descendantReads) ||
            !_ReadU32s(reader, &commit.closestReads) ||
            !_ReadU32s(reader, &commit.descendantWrites) ||
            !_ReadU32s(reader, &commit.descendantBaseWrites) ||
            !_ReadU32s(reader, &commit.slotCarry) ||
            !_ReadU32s(reader, &commit.slotBaseCarry) ||
            !_ReadU32s(reader, &commit.descendantCarry) ||
            !_ReadU32s(reader, &commit.descendantBaseCarry) ||
            !_ReadU32s(reader, &commit.sourceReads) ||
            !reader->ReadU32(&commit.worldUpRead) ||
            !reader->ReadU32(&commit.targetRead) ||
            !_ReadU32s(reader, &commit.targetReads) ||
            !reader->ReadU32(&commit.effectorRead) ||
            !_ReadAncestorReads(reader, &commit.effectorAncestors) ||
            !_ReadU32s(reader, &commit.poleReads) ||
            !_ReadAncestorReadsNested(reader, &commit.poleAncestors)) {
            return _Fail(error);
        }
        if (!reader->ReadU8(&flag) || flag > 1) {
            return _Fail(error);
        }
        commit.recordAfter = flag != 0;
        if (!reader->ReadU8(&flag) || flag > 1) {
            return _Fail(error);
        }
        commit.recordEveryTarget = flag != 0;
        if (!_ReadAncestorReadsNested(reader, &commit.sourceAncestors) ||
            !_ReadAncestorReads(reader, &commit.worldUpAncestors)) {
            return _Fail(error);
        }
    }
    if (!_ReadU32s(reader, &pose->jointBindingJoints) ||
        !reader->ReadU32(&count)) {
        return _Fail(error);
    }
    pose->jointBindingSolvers.resize(count);
    for (uint32_t i = 0; i < count; ++i) {
        if (!_ReadU32s(reader, &pose->jointBindingSolvers[i])) {
            return _Fail(error);
        }
    }
    if (!reader->ReadU32(&count)) {
        return _Fail(error);
    }
    pose->jointBindingElements.resize(count);
    for (uint32_t i = 0; i < count; ++i) {
        if (!_ReadI32s(reader, &pose->jointBindingElements[i])) {
            return _Fail(error);
        }
    }
    if (!reader->ReadU8(&flag) || flag > 1) {
        return _Fail(error);
    }
    pose->hasPropertyChains = flag != 0;
    if (!reader->ReadU8(&flag) || flag > 1) {
        return _Fail(error);
    }
    pose->phasedReads = flag != 0;
    if (!reader->ReadU8(&flag) || flag > 1) {
        return _Fail(error);
    }
    pose->publishWeightFields = flag != 0;
    if (!reader->Exhausted()) {
        if (error) {
            *error = "trailing bytes in pose tables";
        }
        return false;
    }
    return true;
}

void
RigExecWirePutInput(std::vector<uint8_t> *out,
                    const RigExecWireInput &input)
{
    _PutInput(out, input);
}

bool
RigExecWireReadInput(RigExecWireReader *reader, RigExecWireInput *input)
{
    return _ReadInput(reader, input);
}

}  // namespace rigExec
