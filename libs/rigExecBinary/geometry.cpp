//
// .rigexec DomainGeometry section: wire encoding.
//
// The scalar-list helpers here mirror pose.cpp's: a u32 list is a count plus
// items on every section, so there is no format to fork. The bound-input
// codec is NOT mirrored -- its tag order is real format -- and is shared
// through RigExecWirePutInput/RigExecWireReadInput.
//

#include "rigExecBinary/geometry.h"

namespace rigExec {
namespace {

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
_PutF32s(std::vector<uint8_t> *out, const std::vector<float> &values)
{
    RigExecWirePutU32(out, uint32_t(values.size()));
    for (float value : values) {
        RigExecWirePutF32(out, value);
    }
}

bool
_ReadF32s(RigExecWireReader *reader, std::vector<float> *values)
{
    uint32_t count = 0;
    if (!reader->ReadU32(&count)) {
        return false;
    }
    values->resize(count);
    for (uint32_t i = 0; i < count; ++i) {
        if (!reader->ReadF32(&(*values)[i])) {
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
_PutU8s(std::vector<uint8_t> *out, const std::vector<uint8_t> &values)
{
    RigExecWirePutU32(out, uint32_t(values.size()));
    for (uint8_t value : values) {
        RigExecWirePutU8(out, value);
    }
}

bool
_ReadBools(RigExecWireReader *reader, std::vector<uint8_t> *values)
{
    uint32_t count = 0;
    if (!reader->ReadU32(&count)) {
        return false;
    }
    values->resize(count);
    for (uint32_t i = 0; i < count; ++i) {
        uint8_t value = 0;
        if (!reader->ReadU8(&value) || value > 1) {
            return false;
        }
        (*values)[i] = value;
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
_PutMatrices(std::vector<uint8_t> *out,
             const std::vector<RigExecWireMatrix4d> &values)
{
    RigExecWirePutU32(out, uint32_t(values.size()));
    for (const RigExecWireMatrix4d &value : values) {
        RigExecWirePutMatrix4d(out, value);
    }
}

bool
_ReadMatrices(RigExecWireReader *reader,
              std::vector<RigExecWireMatrix4d> *values)
{
    uint32_t count = 0;
    if (!reader->ReadU32(&count)) {
        return false;
    }
    values->resize(count);
    for (uint32_t i = 0; i < count; ++i) {
        if (!RigExecWireReadMatrix4d(reader, &(*values)[i])) {
            return false;
        }
    }
    return true;
}

void
_PutPhase(std::vector<uint8_t> *out, const RigExecWireReadPhase &phase)
{
    RigExecWirePutU8(out, phase.kind);
    RigExecWirePutU32(out, phase.prim);
}

bool
_ReadPhase(RigExecWireReader *reader, RigExecWireReadPhase *phase)
{
    return reader->ReadU8(&phase->kind) && phase->kind <= 3 &&
           reader->ReadU32(&phase->prim);
}

void
_PutBinding(std::vector<uint8_t> *out,
            const RigExecWireRevisionBinding &binding)
{
    RigExecWirePutU32(out, binding.moverPath);
    RigExecWirePutU32(out, binding.target);
    RigExecWirePutU32(out, binding.transform);
    RigExecWirePutU32(out, binding.transformSpace);
    _PutU32s(out, binding.influences);
    RigExecWirePutU32(out, binding.weightObject);
    RigExecWirePutU32(out, binding.base);
    RigExecWirePutU32(out, binding.topologyCounts);
    RigExecWirePutU32(out, binding.topologyIndices);
    RigExecWirePutU32(out, binding.cagePoints);
    RigExecWirePutU32(out, binding.surfacePoints);
    RigExecWirePutU32(out, binding.bindCoords);
    RigExecWirePutU32(out, binding.driverCurvePoints);
    RigExecWirePutU32(out, binding.driverCurveOrder);
    RigExecWirePutU32(out, binding.driverCurveKnots);
    RigExecWirePutI32(out, binding.driverTransformCount);
    RigExecWirePutI32(out, binding.driverSpaceCount);
    RigExecWirePutI32(out, binding.driverBaseTransformCount);
    RigExecWirePutU32(out, binding.driverFrames);
    RigExecWirePutU32(out, binding.widths);
    RigExecWirePutU32(out, binding.curvenet);
    RigExecWirePutU32(out, binding.curvenetPoints);
    _PutU32s(out, binding.blendInputs);
    _PutU32s(out, binding.blendSampleInputs);
    RigExecWirePutU32(out, uint32_t(binding.blendSamples.size()));
    for (const std::vector<RigExecWireBlendSampleBinding> &row :
         binding.blendSamples) {
        RigExecWirePutU32(out, uint32_t(row.size()));
        for (const RigExecWireBlendSampleBinding &sample : row) {
            RigExecWirePutU32(out, sample.sample);
            RigExecWirePutU32(out, sample.points);
            _PutPhase(out, sample.phase);
            RigExecWirePutU32(out, sample.blendShape);
        }
    }
    _PutU32s(out, binding.phaseInputs);
    RigExecWirePutU32(out, uint32_t(binding.phases.size()));
    for (const RigExecWireReadPhase &phase : binding.phases) {
        _PutPhase(out, phase);
    }
    _PutPhase(out, binding.transformPhase);
}

bool
_ReadBinding(RigExecWireReader *reader, RigExecWireRevisionBinding *binding)
{
    uint32_t rows = 0;
    if (!reader->ReadU32(&binding->moverPath) ||
        !reader->ReadU32(&binding->target) ||
        !reader->ReadU32(&binding->transform) ||
        !reader->ReadU32(&binding->transformSpace) ||
        !_ReadU32s(reader, &binding->influences) ||
        !reader->ReadU32(&binding->weightObject) ||
        !reader->ReadU32(&binding->base) ||
        !reader->ReadU32(&binding->topologyCounts) ||
        !reader->ReadU32(&binding->topologyIndices) ||
        !reader->ReadU32(&binding->cagePoints) ||
        !reader->ReadU32(&binding->surfacePoints) ||
        !reader->ReadU32(&binding->bindCoords) ||
        !reader->ReadU32(&binding->driverCurvePoints) ||
        !reader->ReadU32(&binding->driverCurveOrder) ||
        !reader->ReadU32(&binding->driverCurveKnots) ||
        !reader->ReadI32(&binding->driverTransformCount) ||
        !reader->ReadI32(&binding->driverSpaceCount) ||
        !reader->ReadI32(&binding->driverBaseTransformCount) ||
        !reader->ReadU32(&binding->driverFrames) ||
        !reader->ReadU32(&binding->widths) ||
        !reader->ReadU32(&binding->curvenet) ||
        !reader->ReadU32(&binding->curvenetPoints) ||
        !_ReadU32s(reader, &binding->blendInputs) ||
        !_ReadU32s(reader, &binding->blendSampleInputs) ||
        !reader->ReadU32(&rows)) {
        return false;
    }
    binding->blendSamples.resize(rows);
    for (uint32_t i = 0; i < rows; ++i) {
        uint32_t samples = 0;
        if (!reader->ReadU32(&samples)) {
            return false;
        }
        binding->blendSamples[i].resize(samples);
        for (uint32_t s = 0; s < samples; ++s) {
            RigExecWireBlendSampleBinding &sample =
                binding->blendSamples[i][s];
            if (!reader->ReadU32(&sample.sample) ||
                !reader->ReadU32(&sample.points) ||
                !_ReadPhase(reader, &sample.phase) ||
                !reader->ReadU32(&sample.blendShape)) {
                return false;
            }
        }
    }
    uint32_t phases = 0;
    if (!_ReadU32s(reader, &binding->phaseInputs) ||
        !reader->ReadU32(&phases)) {
        return false;
    }
    binding->phases.resize(phases);
    for (uint32_t i = 0; i < phases; ++i) {
        if (!_ReadPhase(reader, &binding->phases[i])) {
            return false;
        }
    }
    return _ReadPhase(reader, &binding->transformPhase);
}

void
_PutTopology(std::vector<uint8_t> *out,
             const RigExecWireSkinTopology &topology)
{
    RigExecWirePutU8(out, topology.hasTopology ? uint8_t(1) : uint8_t(0));
    _PutI32s(out, topology.indices);
    _PutF32s(out, topology.weights);
    RigExecWirePutI32(out, topology.elementSize);
    RigExecWirePutU64(out, topology.pointCount);
    RigExecWirePutU64(out, topology.influenceCount);
    RigExecWirePutU8(out, topology.validated ? uint8_t(1) : uint8_t(0));
}

bool
_ReadTopology(RigExecWireReader *reader, RigExecWireSkinTopology *topology)
{
    uint8_t flag = 0;
    if (!reader->ReadU8(&flag) || flag > 1) {
        return false;
    }
    topology->hasTopology = flag != 0;
    if (!_ReadI32s(reader, &topology->indices) ||
        !_ReadF32s(reader, &topology->weights) ||
        !reader->ReadI32(&topology->elementSize) ||
        !reader->ReadU64(&topology->pointCount) ||
        !reader->ReadU64(&topology->influenceCount) ||
        !reader->ReadU8(&flag) || flag > 1) {
        return false;
    }
    topology->validated = flag != 0;
    return true;
}

void
_PutRevision(std::vector<uint8_t> *out, const RigExecWireRevision &revision)
{
    RigExecWirePutU32(out, revision.moverPath);
    RigExecWirePutU32(out, revision.target);
    RigExecWirePutU32(out, revision.moverPrim);
    RigExecWirePutU8(out, revision.op);
    _PutBinding(out, revision.binding);
    RigExecWirePutI32(out, revision.curvenetChain);
    RigExecWirePutU8(out, revision.curvenetBindResolved ? uint8_t(1)
                                                       : uint8_t(0));
    RigExecWirePutU8(out, revision.hasCurvenetBind ? uint8_t(1)
                                                     : uint8_t(0));
    _PutVec3fs(out, revision.curvenetRestNet);
    _PutI32s(out, revision.curvenetSplineIndices);
    RigExecWirePutI32(out, revision.curvenetSamplesPerSpline);
    RigExecWirePutU32(out, revision.curvenetBasis);
    _PutVec3fs(out, revision.curvenetMeshPoints);
    _PutI32s(out, revision.curvenetMeshCounts);
    _PutI32s(out, revision.curvenetMeshIndices);
    RigExecWirePutU8(out, revision.curvenetBindInputsHeld ? uint8_t(1)
                                                          : uint8_t(0));
    RigExecWirePutU32(out, uint32_t(revision.blendChannels.size()));
    for (const RigExecWireBlendChannel &channel : revision.blendChannels) {
        RigExecWirePutU32(out, channel.weight);
        RigExecWirePutU8(out, channel.weightValid ? uint8_t(1)
                                                  : uint8_t(0));
        RigExecWirePutU32(out, channel.weightPath);
        RigExecWirePutI32(out, channel.poseWeight);
        RigExecWirePutU32(out, uint32_t(channel.samples.size()));
        for (const RigExecWireBlendChannel::Sample &sample :
             channel.samples) {
            RigExecWirePutU32(out, sample.samplePath);
            RigExecWirePutU32(out, sample.activation);
            RigExecWirePutU8(out, sample.activationValid ? uint8_t(1)
                                                         : uint8_t(0));
            RigExecWirePutU32(out, sample.points);
            RigExecWirePutU8(out, sample.pointsValid ? uint8_t(1)
                                                     : uint8_t(0));
            RigExecWirePutU32(out, sample.pointsPath);
            _PutPhase(out, sample.phase);
            RigExecWirePutU32(out, sample.blendShape);
            RigExecWirePutU8(out, sample.hasLayout ? uint8_t(1)
                                                   : uint8_t(0));
            _PutVec3fs(out, sample.offsets);
            _PutI32s(out, sample.indices);
            RigExecWirePutU64(out, sample.pointCount);
            RigExecWirePutU8(out, sample.layoutValid ? uint8_t(1)
                                                     : uint8_t(0));
        }
    }
    _PutI32s(out, revision.influenceSlots);
    RigExecWirePutI32(out, revision.transformSlot);
    RigExecWirePutI32(out, revision.transformSpaceSlot);
    RigExecWirePutI32(out, revision.constraintDelta);
    RigExecWirePutI32(out, revision.driverFramesSolver);
    RigExecWirePutU8(out, revision.finalPhase ? uint8_t(1) : uint8_t(0));
    RigExecWirePutU8(out, revision.skinTopologyFixed ? uint8_t(1)
                                                     : uint8_t(0));
    RigExecWirePutU8(out, revision.snapshotAfter ? uint8_t(1) : uint8_t(0));
    RigExecWirePutU8(out, revision.readsSnapshots ? uint8_t(1) : uint8_t(0));
    _PutMatrices(out, revision.packetInfluences);
    RigExecWirePutU32(out, uint32_t(revision.chunks.size()));
    for (const RigExecWireChunk &chunk : revision.chunks) {
        RigExecWirePutI32(out, chunk.begin);
        RigExecWirePutI32(out, chunk.end);
        _PutI32s(out, chunk.key);
    }
    RigExecWirePutI32(out, revision.chunkBase);
    _PutTopology(out, revision.partitionTopology);
    RigExecWirePutI32(out, revision.partitionElementSize);
    RigExecWirePutU64(out, revision.partitionIndexCount);
    RigExecWirePutU64(out, revision.partitionPointCount);
    RigExecWirePutU8(out, revision.chunked ? uint8_t(1) : uint8_t(0));
    RigExecWirePutU64(out, revision.partitionCandidates);
    RigExecWirePutI32(out, revision.partitionReadyMin);
    RigExecWirePutI32(out, revision.partitionReadyMax);
    RigExecWirePutI32(out, revision.weightObject);
    RigExecWirePutU8(out, revision.weightOperationDomain ? uint8_t(1)
                                                         : uint8_t(0));
    RigExecWirePutU32(out, revision.weightFieldTarget);
    RigExecWirePutU8(out, revision.weightCurrentPhase ? uint8_t(1)
                                                      : uint8_t(0));
    _PutTopology(out, revision.topology);
    RigExecWirePutU8(out, revision.topologyResolved ? uint8_t(1)
                                                    : uint8_t(0));
}

bool
_ReadRevision(RigExecWireReader *reader, RigExecWireRevision *revision)
{
    uint8_t flag = 0;
    uint32_t count = 0;
    if (!reader->ReadU32(&revision->moverPath) ||
        !reader->ReadU32(&revision->target) ||
        !reader->ReadU32(&revision->moverPrim) ||
        !reader->ReadU8(&revision->op) || revision->op > 14 ||
        !_ReadBinding(reader, &revision->binding) ||
        !reader->ReadI32(&revision->curvenetChain) ||
        !reader->ReadU8(&flag) || flag > 1) {
        return false;
    }
    revision->curvenetBindResolved = flag != 0;
    if (!reader->ReadU8(&flag) || flag > 1) {
        return false;
    }
    revision->hasCurvenetBind = flag != 0;
    if (!_ReadVec3fs(reader, &revision->curvenetRestNet) ||
        !_ReadI32s(reader, &revision->curvenetSplineIndices) ||
        !reader->ReadI32(&revision->curvenetSamplesPerSpline) ||
        !reader->ReadU32(&revision->curvenetBasis) ||
        !_ReadVec3fs(reader, &revision->curvenetMeshPoints) ||
        !_ReadI32s(reader, &revision->curvenetMeshCounts) ||
        !_ReadI32s(reader, &revision->curvenetMeshIndices) ||
        !reader->ReadU8(&flag) || flag > 1) {
        return false;
    }
    revision->curvenetBindInputsHeld = flag != 0;
    if (!reader->ReadU32(&count)) {
        return false;
    }
    revision->blendChannels.resize(count);
    for (uint32_t i = 0; i < count; ++i) {
        RigExecWireBlendChannel &channel = revision->blendChannels[i];
        uint32_t samples = 0;
        if (!reader->ReadU32(&channel.weight) || !reader->ReadU8(&flag) ||
            flag > 1) {
            return false;
        }
        channel.weightValid = flag != 0;
        if (!reader->ReadU32(&channel.weightPath) ||
            !reader->ReadI32(&channel.poseWeight) ||
            !reader->ReadU32(&samples)) {
            return false;
        }
        channel.samples.resize(samples);
        for (uint32_t s = 0; s < samples; ++s) {
            RigExecWireBlendChannel::Sample &sample = channel.samples[s];
            if (!reader->ReadU32(&sample.samplePath) ||
                !reader->ReadU32(&sample.activation) ||
                !reader->ReadU8(&flag) || flag > 1) {
                return false;
            }
            sample.activationValid = flag != 0;
            if (!reader->ReadU32(&sample.points) || !reader->ReadU8(&flag) ||
                flag > 1) {
                return false;
            }
            sample.pointsValid = flag != 0;
            if (!reader->ReadU32(&sample.pointsPath) ||
                !_ReadPhase(reader, &sample.phase) ||
                !reader->ReadU32(&sample.blendShape) ||
                !reader->ReadU8(&flag) || flag > 1) {
                return false;
            }
            sample.hasLayout = flag != 0;
            if (!_ReadVec3fs(reader, &sample.offsets) ||
                !_ReadI32s(reader, &sample.indices) ||
                !reader->ReadU64(&sample.pointCount) ||
                !reader->ReadU8(&flag) || flag > 1) {
                return false;
            }
            sample.layoutValid = flag != 0;
        }
    }
    if (!_ReadI32s(reader, &revision->influenceSlots) ||
        !reader->ReadI32(&revision->transformSlot) ||
        !reader->ReadI32(&revision->transformSpaceSlot) ||
        !reader->ReadI32(&revision->constraintDelta) ||
        !reader->ReadI32(&revision->driverFramesSolver) ||
        !reader->ReadU8(&flag) || flag > 1) {
        return false;
    }
    revision->finalPhase = flag != 0;
    if (!reader->ReadU8(&flag) || flag > 1) {
        return false;
    }
    revision->skinTopologyFixed = flag != 0;
    if (!reader->ReadU8(&flag) || flag > 1) {
        return false;
    }
    revision->snapshotAfter = flag != 0;
    if (!reader->ReadU8(&flag) || flag > 1) {
        return false;
    }
    revision->readsSnapshots = flag != 0;
    if (!_ReadMatrices(reader, &revision->packetInfluences) ||
        !reader->ReadU32(&count)) {
        return false;
    }
    revision->chunks.resize(count);
    for (uint32_t i = 0; i < count; ++i) {
        RigExecWireChunk &chunk = revision->chunks[i];
        if (!reader->ReadI32(&chunk.begin) ||
            !reader->ReadI32(&chunk.end) ||
            !_ReadI32s(reader, &chunk.key)) {
            return false;
        }
    }
    if (!reader->ReadI32(&revision->chunkBase) ||
        !_ReadTopology(reader, &revision->partitionTopology) ||
        !reader->ReadI32(&revision->partitionElementSize) ||
        !reader->ReadU64(&revision->partitionIndexCount) ||
        !reader->ReadU64(&revision->partitionPointCount) ||
        !reader->ReadU8(&flag) || flag > 1) {
        return false;
    }
    revision->chunked = flag != 0;
    if (!reader->ReadU64(&revision->partitionCandidates) ||
        !reader->ReadI32(&revision->partitionReadyMin) ||
        !reader->ReadI32(&revision->partitionReadyMax) ||
        !reader->ReadI32(&revision->weightObject) ||
        !reader->ReadU8(&flag) || flag > 1) {
        return false;
    }
    revision->weightOperationDomain = flag != 0;
    if (!reader->ReadU32(&revision->weightFieldTarget) ||
        !reader->ReadU8(&flag) || flag > 1) {
        return false;
    }
    revision->weightCurrentPhase = flag != 0;
    if (!_ReadTopology(reader, &revision->topology) ||
        !reader->ReadU8(&flag) || flag > 1) {
        return false;
    }
    revision->topologyResolved = flag != 0;
    return true;
}

void
_PutWeightObject(std::vector<uint8_t> *out,
                 const RigExecWireWeightObject &object)
{
    RigExecWirePutU32(out, object.path);
    RigExecWirePutU32(out, object.type);
    RigExecWirePutU32(out, object.representation);
    RigExecWirePutU32(out, object.rangePolicy);
    _PutF32s(out, object.values);
    _PutI32s(out, object.indices);
    RigExecWirePutInput(out, object.defaultWeight);
    RigExecWirePutI32(out, object.base);
    _PutI32s(out, object.inputs);
    RigExecWirePutInput(out, object.driver);
    RigExecWirePutInput(out, object.scale);
    RigExecWirePutInput(out, object.bias);
    RigExecWirePutU32(out, object.combineMode);
    RigExecWirePutInput(out, object.strength);
    RigExecWirePutInput(out, object.invert);
    _PutU32s(out, object.combineTargetPoints);
    _PutU8s(out, object.combineTargetValid);
    RigExecWirePutU64(out, object.costElements);
    RigExecWirePutI32(out, object.providerSlot);
    RigExecWirePutInput(out, object.falloffMin);
    RigExecWirePutInput(out, object.falloffMax);
    RigExecWirePutInput(out, object.scaleX);
    RigExecWirePutInput(out, object.scaleY);
    RigExecWirePutInput(out, object.scaleZ);
    RigExecWirePutInput(out, object.extentU);
    RigExecWirePutInput(out, object.extentV);
    RigExecWirePutU32(out, object.planeAxis);
    RigExecWirePutU32(out, object.planeBounds);
    _PutU32s(out, object.targetPoints);
    _PutU8s(out, object.targetValid);
    _PutU32s(out, object.samplePoints);
    _PutU8s(out, object.sampleValid);
    _PutU32s(out, object.curvePoints);
    _PutU8s(out, object.curveValid);
    _PutF32s(out, object.falloffCurve);
    _PutU32s(out, object.curvenetMeshPoints);
    _PutU8s(out, object.curvenetMeshValid);
    _PutU32s(out, object.curvenetPoints);
    _PutU8s(out, object.curvenetPointsValid);
    _PutU32s(out, object.curvenetCounts);
    _PutU8s(out, object.curvenetCountsValid);
    _PutU32s(out, object.curvenetIndices);
    _PutU8s(out, object.curvenetIndicesValid);
    _PutU32s(out, object.curvenetSplines);
    _PutU8s(out, object.curvenetSplinesValid);
    RigExecWirePutU32(out, object.curvenetWeights);
    RigExecWirePutU8(out, object.curvenetWeightsValid ? uint8_t(1)
                                                     : uint8_t(0));
    RigExecWirePutU32(out, object.curvenetAutoSmooth);
    RigExecWirePutU8(out, object.curvenetAutoSmoothValid ? uint8_t(1)
                                                         : uint8_t(0));
    RigExecWirePutU32(out, object.curvenetBasis);
    RigExecWirePutInput(out, object.curvenetSamples);
    RigExecWirePutInput(out, object.curvenetUnreached);
    _PutVec3fs(out, object.boundMesh);
    _PutVec3fs(out, object.boundNet);
    _PutI32s(out, object.boundCounts);
    _PutI32s(out, object.boundIndices);
    _PutI32s(out, object.boundSplines);
    _PutI32s(out, object.boundSmooth);
    RigExecWirePutI32(out, object.boundSamples);
    RigExecWirePutU8(out, object.bound ? uint8_t(1) : uint8_t(0));
}

bool
_ReadWeightObject(RigExecWireReader *reader,
                  RigExecWireWeightObject *object)
{
    uint8_t flag = 0;
    if (!reader->ReadU32(&object->path) ||
        !reader->ReadU32(&object->type) ||
        !reader->ReadU32(&object->representation) ||
        !reader->ReadU32(&object->rangePolicy) ||
        !_ReadF32s(reader, &object->values) ||
        !_ReadI32s(reader, &object->indices) ||
        !RigExecWireReadInput(reader, &object->defaultWeight) ||
        !reader->ReadI32(&object->base) ||
        !_ReadI32s(reader, &object->inputs) ||
        !RigExecWireReadInput(reader, &object->driver) ||
        !RigExecWireReadInput(reader, &object->scale) ||
        !RigExecWireReadInput(reader, &object->bias) ||
        !reader->ReadU32(&object->combineMode) ||
        !RigExecWireReadInput(reader, &object->strength) ||
        !RigExecWireReadInput(reader, &object->invert)) {
        return false;
    }
    // Attribute-validity bytes are 0/1 like every other flag vector.
    auto readValid = [&](std::vector<uint8_t> *values) {
        uint32_t count = 0;
        if (!reader->ReadU32(&count)) {
            return false;
        }
        values->resize(count);
        for (uint32_t i = 0; i < count; ++i) {
            uint8_t value = 0;
            if (!reader->ReadU8(&value) || value > 1) {
                return false;
            }
            (*values)[i] = value;
        }
        return true;
    };
    if (!_ReadU32s(reader, &object->combineTargetPoints) ||
        !readValid(&object->combineTargetValid) ||
        !reader->ReadU64(&object->costElements) ||
        !reader->ReadI32(&object->providerSlot) ||
        !RigExecWireReadInput(reader, &object->falloffMin) ||
        !RigExecWireReadInput(reader, &object->falloffMax) ||
        !RigExecWireReadInput(reader, &object->scaleX) ||
        !RigExecWireReadInput(reader, &object->scaleY) ||
        !RigExecWireReadInput(reader, &object->scaleZ) ||
        !RigExecWireReadInput(reader, &object->extentU) ||
        !RigExecWireReadInput(reader, &object->extentV) ||
        !reader->ReadU32(&object->planeAxis) ||
        !reader->ReadU32(&object->planeBounds) ||
        !_ReadU32s(reader, &object->targetPoints) ||
        !readValid(&object->targetValid) ||
        !_ReadU32s(reader, &object->samplePoints) ||
        !readValid(&object->sampleValid) ||
        !_ReadU32s(reader, &object->curvePoints) ||
        !readValid(&object->curveValid) ||
        !_ReadF32s(reader, &object->falloffCurve) ||
        !_ReadU32s(reader, &object->curvenetMeshPoints) ||
        !readValid(&object->curvenetMeshValid) ||
        !_ReadU32s(reader, &object->curvenetPoints) ||
        !readValid(&object->curvenetPointsValid) ||
        !_ReadU32s(reader, &object->curvenetCounts) ||
        !readValid(&object->curvenetCountsValid) ||
        !_ReadU32s(reader, &object->curvenetIndices) ||
        !readValid(&object->curvenetIndicesValid) ||
        !_ReadU32s(reader, &object->curvenetSplines) ||
        !readValid(&object->curvenetSplinesValid) ||
        !reader->ReadU32(&object->curvenetWeights) ||
        !reader->ReadU8(&flag) || flag > 1) {
        return false;
    }
    object->curvenetWeightsValid = flag != 0;
    if (!reader->ReadU32(&object->curvenetAutoSmooth) ||
        !reader->ReadU8(&flag) || flag > 1) {
        return false;
    }
    object->curvenetAutoSmoothValid = flag != 0;
    if (!reader->ReadU32(&object->curvenetBasis) ||
        !RigExecWireReadInput(reader, &object->curvenetSamples) ||
        !RigExecWireReadInput(reader, &object->curvenetUnreached) ||
        !_ReadVec3fs(reader, &object->boundMesh) ||
        !_ReadVec3fs(reader, &object->boundNet) ||
        !_ReadI32s(reader, &object->boundCounts) ||
        !_ReadI32s(reader, &object->boundIndices) ||
        !_ReadI32s(reader, &object->boundSplines) ||
        !_ReadI32s(reader, &object->boundSmooth) ||
        !reader->ReadI32(&object->boundSamples) ||
        !reader->ReadU8(&flag) || flag > 1) {
        return false;
    }
    object->bound = flag != 0;
    return true;
}

bool
_Fail(std::string *error)
{
    if (error) {
        *error = "malformed geometry tables";
    }
    return false;
}

}  // namespace

bool
RigExecWireEncodeDomainGeometry(const RigExecWireDomainGeometry &geometry,
                                std::vector<uint8_t> *out)
{
    RigExecWirePutU32(out, uint32_t(geometry.chains.size()));
    for (const RigExecWireChain &chain : geometry.chains) {
        RigExecWirePutU32(out, chain.target);
        RigExecWirePutU32(out, uint32_t(chain.revisions.size()));
        for (const RigExecWireRevision &revision : chain.revisions) {
            _PutRevision(out, revision);
        }
        RigExecWirePutU32(out, uint32_t(chain.derived.size()));
        for (const RigExecWireDerived &derived : chain.derived) {
            RigExecWirePutU32(out, derived.target);
            _PutRevision(out, derived.revision);
        }
    }
    _PutPairs(out, geometry.revisionIndex);
    _PutPairs(out, geometry.derivedIndex);
    _PutI32s(out, geometry.chainRevisionBegin);
    _PutI32s(out, geometry.chainRevisionEnd);
    _PutI32s(out, geometry.revisionChunkBase);
    _PutI32s(out, geometry.revisionChunkCount);
    _PutI32s(out, geometry.chainChunkBegin);
    _PutI32s(out, geometry.chainChunkEnd);
    RigExecWirePutU32(out, uint32_t(geometry.weightObjects.size()));
    for (const RigExecWireWeightObject &object : geometry.weightObjects) {
        _PutWeightObject(out, object);
    }
    _PutU32s(out, geometry.falloffPaths);
    RigExecWirePutU32(out, uint32_t(geometry.falloffLuts.size()));
    for (const std::vector<float> &lut : geometry.falloffLuts) {
        _PutF32s(out, lut);
    }
    _PutU32s(out, geometry.currentPhaseWeights);
    _PutU32s(out, geometry.deltaBasePaths);
    return true;
}

bool
RigExecWireDecodeDomainGeometry(RigExecWireReader *reader,
                                RigExecWireDomainGeometry *geometry,
                                std::string *error)
{
    uint32_t count = 0;
    uint8_t flag = 0;
    if (!reader->ReadU32(&count)) {
        return _Fail(error);
    }
    geometry->chains.resize(count);
    for (uint32_t i = 0; i < count; ++i) {
        RigExecWireChain &chain = geometry->chains[i];
        uint32_t revisions = 0, derived = 0;
        if (!reader->ReadU32(&chain.target)) {
            return _Fail(error);
        }
        if (!reader->ReadU32(&revisions)) {
            return _Fail(error);
        }
        chain.revisions.resize(revisions);
        for (uint32_t r = 0; r < revisions; ++r) {
            if (!_ReadRevision(reader, &chain.revisions[r])) {
                return _Fail(error);
            }
        }
        if (!reader->ReadU32(&derived)) {
            return _Fail(error);
        }
        chain.derived.resize(derived);
        for (uint32_t d = 0; d < derived; ++d) {
            RigExecWireDerived &entry = chain.derived[d];
            if (!reader->ReadU32(&entry.target)) {
                return _Fail(error);
            }
            if (!_ReadRevision(reader, &entry.revision)) {
                return _Fail(error);
            }
        }
    }
    uint32_t objects = 0, luts = 0;
    if (!_ReadPairs(reader, &geometry->revisionIndex) ||
        !_ReadPairs(reader, &geometry->derivedIndex) ||
        !_ReadI32s(reader, &geometry->chainRevisionBegin) ||
        !_ReadI32s(reader, &geometry->chainRevisionEnd) ||
        !_ReadI32s(reader, &geometry->revisionChunkBase) ||
        !_ReadI32s(reader, &geometry->revisionChunkCount) ||
        !_ReadI32s(reader, &geometry->chainChunkBegin) ||
        !_ReadI32s(reader, &geometry->chainChunkEnd) ||
        !reader->ReadU32(&objects)) {
        return _Fail(error);
    }
    geometry->weightObjects.resize(objects);
    for (uint32_t i = 0; i < objects; ++i) {
        if (!_ReadWeightObject(reader, &geometry->weightObjects[i])) {
            return _Fail(error);
        }
    }
    if (!_ReadU32s(reader, &geometry->falloffPaths) ||
        !reader->ReadU32(&luts)) {
        return _Fail(error);
    }
    geometry->falloffLuts.resize(luts);
    for (uint32_t i = 0; i < luts; ++i) {
        if (!_ReadF32s(reader, &geometry->falloffLuts[i])) {
            return _Fail(error);
        }
    }
    if (!_ReadU32s(reader, &geometry->currentPhaseWeights) ||
        !_ReadU32s(reader, &geometry->deltaBasePaths)) {
        return _Fail(error);
    }
    if (!reader->Exhausted()) {
        if (error) {
            *error = "trailing bytes in geometry tables";
        }
        return false;
    }
    return true;
}

}  // namespace rigExec
