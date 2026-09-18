//
// .rigexec InputTable section: wire encoding.
//

#include "rigExecBinary/inputTable.h"

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
_PutVec2fs(std::vector<uint8_t> *out,
           const std::vector<RigExecWireVec2f> &values)
{
    RigExecWirePutU32(out, uint32_t(values.size()));
    for (const RigExecWireVec2f &value : values) {
        RigExecWirePutVec2f(out, value);
    }
}

bool
_ReadVec2fs(RigExecWireReader *reader,
            std::vector<RigExecWireVec2f> *values)
{
    uint32_t count = 0;
    if (!reader->ReadU32(&count)) {
        return false;
    }
    values->resize(count);
    for (uint32_t i = 0; i < count; ++i) {
        if (!RigExecWireReadVec2f(reader, &(*values)[i])) {
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
_PutValue(std::vector<uint8_t> *out, const RigExecWireValue &value)
{
    RigExecWirePutU8(out, uint8_t(value.tag));
    switch (value.tag) {
    case RigExecWireInput::Tag::Double:
        RigExecWirePutF64(out, value.f64);
        break;
    case RigExecWireInput::Tag::Float:
        RigExecWirePutF32(out, value.f32);
        break;
    case RigExecWireInput::Tag::Bool:
        RigExecWirePutU8(out, value.boolean ? uint8_t(1) : uint8_t(0));
        break;
    case RigExecWireInput::Tag::Int:
        RigExecWirePutI32(out, value.i32);
        break;
    case RigExecWireInput::Tag::Matrix4d:
        RigExecWirePutMatrix4d(out, value.matrix);
        break;
    case RigExecWireInput::Tag::Token:
        RigExecWirePutU32(out, value.token);
        break;
    case RigExecWireInput::Tag::Vec3d:
        RigExecWirePutVec3d(out, value.vec);
        break;
    }
}

bool
_ReadValue(RigExecWireReader *reader, RigExecWireValue *value)
{
    uint8_t tag = 0;
    if (!reader->ReadU8(&tag) ||
        tag > uint8_t(RigExecWireInput::Tag::Vec3d)) {
        return false;
    }
    value->tag = RigExecWireInput::Tag(tag);
    uint8_t boolean = 0;
    switch (value->tag) {
    case RigExecWireInput::Tag::Double:
        return reader->ReadF64(&value->f64);
    case RigExecWireInput::Tag::Float:
        return reader->ReadF32(&value->f32);
    case RigExecWireInput::Tag::Bool:
        if (!reader->ReadU8(&boolean) || boolean > 1) {
            return false;
        }
        value->boolean = boolean != 0;
        return true;
    case RigExecWireInput::Tag::Int:
        return reader->ReadI32(&value->i32);
    case RigExecWireInput::Tag::Matrix4d:
        return RigExecWireReadMatrix4d(reader, &value->matrix);
    case RigExecWireInput::Tag::Token:
        return reader->ReadU32(&value->token);
    case RigExecWireInput::Tag::Vec3d:
        return RigExecWireReadVec3d(reader, &value->vec);
    }
    return false;
}

void
_PutPropertyValue(std::vector<uint8_t> *out,
                  const RigExecWirePropertyValue &value)
{
    RigExecWirePutU8(out, uint8_t(value.tag));
    switch (value.tag) {
    case RigExecWirePropertyValue::Tag::Float:
        RigExecWirePutF32(out, value.f32);
        break;
    case RigExecWirePropertyValue::Tag::Double:
        RigExecWirePutF64(out, value.f64);
        break;
    case RigExecWirePropertyValue::Tag::Matrix4d:
        RigExecWirePutMatrix4d(out, value.matrix);
        break;
    case RigExecWirePropertyValue::Tag::Vec3f:
        RigExecWirePutVec3f(out, value.vec);
        break;
    }
}

bool
_ReadPropertyValue(RigExecWireReader *reader,
                   RigExecWirePropertyValue *value)
{
    uint8_t tag = 0;
    if (!reader->ReadU8(&tag) ||
        tag > uint8_t(RigExecWirePropertyValue::Tag::Vec3f)) {
        return false;
    }
    value->tag = RigExecWirePropertyValue::Tag(tag);
    switch (value->tag) {
    case RigExecWirePropertyValue::Tag::Float:
        return reader->ReadF32(&value->f32);
    case RigExecWirePropertyValue::Tag::Double:
        return reader->ReadF64(&value->f64);
    case RigExecWirePropertyValue::Tag::Matrix4d:
        return RigExecWireReadMatrix4d(reader, &value->matrix);
    case RigExecWirePropertyValue::Tag::Vec3f:
        return RigExecWireReadVec3f(reader, &value->vec);
    }
    return false;
}

void
_PutWeightPacket(std::vector<uint8_t> *out,
                 const RigExecWireWeightPacket &packet)
{
    RigExecWirePutU32(out, packet.representation);
    RigExecWirePutU32(out, packet.rangePolicy);
    _PutF32s(out, packet.values);
    _PutI32s(out, packet.indices);
    RigExecWirePutF32(out, packet.defaultWeight);
    RigExecWirePutU8(out, packet.valid ? uint8_t(1) : uint8_t(0));
}

bool
_ReadWeightPacket(RigExecWireReader *reader,
                  RigExecWireWeightPacket *packet)
{
    uint8_t flag = 0;
    if (!reader->ReadU32(&packet->representation) ||
        !reader->ReadU32(&packet->rangePolicy) ||
        !_ReadF32s(reader, &packet->values) ||
        !_ReadI32s(reader, &packet->indices) ||
        !reader->ReadF32(&packet->defaultWeight) ||
        !reader->ReadU8(&flag) || flag > 1) {
        return false;
    }
    packet->valid = flag != 0;
    return true;
}

void
_PutF32s2(std::vector<uint8_t> *out,
          const std::vector<std::vector<float>> &values)
{
    RigExecWirePutU32(out, uint32_t(values.size()));
    for (const std::vector<float> &row : values) {
        _PutF32s(out, row);
    }
}

bool
_ReadF32s2(RigExecWireReader *reader,
           std::vector<std::vector<float>> *values)
{
    uint32_t count = 0;
    if (!reader->ReadU32(&count)) {
        return false;
    }
    values->resize(count);
    for (uint32_t i = 0; i < count; ++i) {
        if (!_ReadF32s(reader, &(*values)[i])) {
            return false;
        }
    }
    return true;
}

void
_PutF32s3(std::vector<uint8_t> *out,
          const std::vector<std::vector<std::vector<float>>> &values)
{
    RigExecWirePutU32(out, uint32_t(values.size()));
    for (const auto &row : values) {
        _PutF32s2(out, row);
    }
}

bool
_ReadF32s3(RigExecWireReader *reader,
           std::vector<std::vector<std::vector<float>>> *values)
{
    uint32_t count = 0;
    if (!reader->ReadU32(&count)) {
        return false;
    }
    values->resize(count);
    for (uint32_t i = 0; i < count; ++i) {
        if (!_ReadF32s2(reader, &(*values)[i])) {
            return false;
        }
    }
    return true;
}

void
_PutF32s4(std::vector<uint8_t> *out,
          const std::vector<std::vector<std::vector<std::vector<float>>>>
              &values)
{
    RigExecWirePutU32(out, uint32_t(values.size()));
    for (const auto &row : values) {
        _PutF32s3(out, row);
    }
}

bool
_ReadF32s4(
    RigExecWireReader *reader,
    std::vector<std::vector<std::vector<std::vector<float>>>> *values)
{
    uint32_t count = 0;
    if (!reader->ReadU32(&count)) {
        return false;
    }
    values->resize(count);
    for (uint32_t i = 0; i < count; ++i) {
        if (!_ReadF32s3(reader, &(*values)[i])) {
            return false;
        }
    }
    return true;
}

void
_PutVec3fs2(std::vector<uint8_t> *out,
            const std::vector<std::vector<RigExecWireVec3f>> &values)
{
    RigExecWirePutU32(out, uint32_t(values.size()));
    for (const std::vector<RigExecWireVec3f> &row : values) {
        _PutVec3fs(out, row);
    }
}

bool
_ReadVec3fs2(RigExecWireReader *reader,
             std::vector<std::vector<RigExecWireVec3f>> *values)
{
    uint32_t count = 0;
    if (!reader->ReadU32(&count)) {
        return false;
    }
    values->resize(count);
    for (uint32_t i = 0; i < count; ++i) {
        if (!_ReadVec3fs(reader, &(*values)[i])) {
            return false;
        }
    }
    return true;
}

void
_PutVec3fs5(std::vector<uint8_t> *out,
            const std::vector<std::vector<std::vector<std::vector<
                std::vector<RigExecWireVec3f>>>>> &values)
{
    RigExecWirePutU32(out, uint32_t(values.size()));
    for (const auto &chain : values) {
        RigExecWirePutU32(out, uint32_t(chain.size()));
        for (const auto &revision : chain) {
            RigExecWirePutU32(out, uint32_t(revision.size()));
            for (const auto &channel : revision) {
                RigExecWirePutU32(out, uint32_t(channel.size()));
                for (const std::vector<RigExecWireVec3f> &sample : channel) {
                    _PutVec3fs(out, sample);
                }
            }
        }
    }
}

bool
_ReadVec3fs5(RigExecWireReader *reader,
             std::vector<std::vector<std::vector<std::vector<
                 std::vector<RigExecWireVec3f>>>>> *values)
{
    uint32_t chains = 0;
    if (!reader->ReadU32(&chains)) {
        return false;
    }
    values->resize(chains);
    for (uint32_t c = 0; c < chains; ++c) {
        uint32_t revisions = 0;
        if (!reader->ReadU32(&revisions)) {
            return false;
        }
        (*values)[c].resize(revisions);
        for (uint32_t r = 0; r < revisions; ++r) {
            uint32_t channels = 0;
            if (!reader->ReadU32(&channels)) {
                return false;
            }
            (*values)[c][r].resize(channels);
            for (uint32_t k = 0; k < channels; ++k) {
                uint32_t samples = 0;
                if (!reader->ReadU32(&samples)) {
                    return false;
                }
                (*values)[c][r][k].resize(samples);
                for (uint32_t s = 0; s < samples; ++s) {
                    if (!_ReadVec3fs(reader, &(*values)[c][r][k][s])) {
                        return false;
                    }
                }
            }
        }
    }
    return true;
}

void
_PutMatrices2(std::vector<uint8_t> *out,
              const std::vector<std::vector<RigExecWireMatrix4d>> &values)
{
    RigExecWirePutU32(out, uint32_t(values.size()));
    for (const std::vector<RigExecWireMatrix4d> &row : values) {
        _PutMatrices(out, row);
    }
}

bool
_ReadMatrices2(RigExecWireReader *reader,
               std::vector<std::vector<RigExecWireMatrix4d>> *values)
{
    uint32_t count = 0;
    if (!reader->ReadU32(&count)) {
        return false;
    }
    values->resize(count);
    for (uint32_t i = 0; i < count; ++i) {
        if (!_ReadMatrices(reader, &(*values)[i])) {
            return false;
        }
    }
    return true;
}

void
_PutU8s2(std::vector<uint8_t> *out,
         const std::vector<std::vector<uint8_t>> &values)
{
    RigExecWirePutU32(out, uint32_t(values.size()));
    for (const std::vector<uint8_t> &row : values) {
        RigExecWirePutU32(out, uint32_t(row.size()));
        for (uint8_t v : row) {
            RigExecWirePutU8(out, v);
        }
    }
}

void
_PutPackets2(std::vector<uint8_t> *out,
             const std::vector<std::vector<RigExecWireWeightPacket>> &values)
{
    RigExecWirePutU32(out, uint32_t(values.size()));
    for (const std::vector<RigExecWireWeightPacket> &row : values) {
        RigExecWirePutU32(out, uint32_t(row.size()));
        for (const RigExecWireWeightPacket &packet : row) {
            _PutWeightPacket(out, packet);
        }
    }
}

bool
_ReadPackets2(RigExecWireReader *reader,
              std::vector<std::vector<RigExecWireWeightPacket>> *values)
{
    uint32_t rows = 0;
    if (!reader->ReadU32(&rows)) {
        return false;
    }
    values->resize(rows);
    for (uint32_t i = 0; i < rows; ++i) {
        uint32_t count = 0;
        if (!reader->ReadU32(&count)) {
            return false;
        }
        (*values)[i].resize(count);
        for (uint32_t k = 0; k < count; ++k) {
            if (!_ReadWeightPacket(reader, &(*values)[i][k])) {
                return false;
            }
        }
    }
    return true;
}

void
_PutRefusedLayout(std::vector<uint8_t> *out,
                  const RigExecWireRefusedLayout &layout)
{
    RigExecWirePutU32(out, layout.mover);
    RigExecWirePutU32(out, layout.sample);
    RigExecWirePutU64(out, layout.pointCount);
    RigExecWirePutU8(out, layout.valid ? uint8_t(1) : uint8_t(0));
    _PutVec3fs(out, layout.offsets);
    _PutI32s(out, layout.indices);
}

bool
_ReadRefusedLayout(RigExecWireReader *reader,
                   RigExecWireRefusedLayout *layout)
{
    uint8_t flag = 0;
    if (!reader->ReadU32(&layout->mover) ||
        !reader->ReadU32(&layout->sample) ||
        !reader->ReadU64(&layout->pointCount) ||
        !reader->ReadU8(&flag) || flag > 1 ||
        !_ReadVec3fs(reader, &layout->offsets) ||
        !_ReadI32s(reader, &layout->indices)) {
        return false;
    }
    layout->valid = flag != 0;
    return true;
}

void
_PutPathValue(std::vector<uint8_t> *out,
              const RigExecWirePathValue &value)
{
    RigExecWirePutU8(out, uint8_t(value.tag));
    switch (value.tag) {
    case RigExecWirePathValue::Tag::Absent:
        break;
    case RigExecWirePathValue::Tag::Bool:
        RigExecWirePutU8(out, value.boolean ? uint8_t(1) : uint8_t(0));
        break;
    case RigExecWirePathValue::Tag::Int:
        RigExecWirePutI32(out, value.i32);
        break;
    case RigExecWirePathValue::Tag::Float:
        RigExecWirePutF32(out, value.f32);
        break;
    case RigExecWirePathValue::Tag::Double:
        RigExecWirePutF64(out, value.f64);
        break;
    case RigExecWirePathValue::Tag::Token:
        RigExecWirePutU32(out, value.token);
        break;
    case RigExecWirePathValue::Tag::Matrix4d:
        RigExecWirePutMatrix4d(out, value.matrix);
        break;
    case RigExecWirePathValue::Tag::Vec3d:
        RigExecWirePutVec3d(out, value.vec);
        break;
    case RigExecWirePathValue::Tag::IntArray:
        _PutI32s(out, value.ints);
        break;
    case RigExecWirePathValue::Tag::FloatArray:
        _PutF32s(out, value.floats);
        break;
    case RigExecWirePathValue::Tag::Vec2fArray:
        _PutVec2fs(out, value.vec2s);
        break;
    case RigExecWirePathValue::Tag::Vec3fArray:
        _PutVec3fs(out, value.vec3s);
        break;
    case RigExecWirePathValue::Tag::Vec3i:
        RigExecWirePutI32(out, value.vec3i[0]);
        RigExecWirePutI32(out, value.vec3i[1]);
        RigExecWirePutI32(out, value.vec3i[2]);
        break;
    case RigExecWirePathValue::Tag::DoubleArray:
        _PutF64s(out, value.doubles);
        break;
    }
}

bool
_ReadPathValue(RigExecWireReader *reader, RigExecWirePathValue *value)
{
    uint8_t tag = 0;
    if (!reader->ReadU8(&tag) || tag > 13) {
        return false;
    }
    value->tag = RigExecWirePathValue::Tag(tag);
    switch (value->tag) {
    case RigExecWirePathValue::Tag::Absent:
        break;
    case RigExecWirePathValue::Tag::Bool: {
        uint8_t flag = 0;
        if (!reader->ReadU8(&flag) || flag > 1) {
            return false;
        }
        value->boolean = flag != 0;
        break;
    }
    case RigExecWirePathValue::Tag::Int:
        if (!reader->ReadI32(&value->i32)) {
            return false;
        }
        break;
    case RigExecWirePathValue::Tag::Float:
        if (!reader->ReadF32(&value->f32)) {
            return false;
        }
        break;
    case RigExecWirePathValue::Tag::Double:
        if (!reader->ReadF64(&value->f64)) {
            return false;
        }
        break;
    case RigExecWirePathValue::Tag::Token:
        if (!reader->ReadU32(&value->token)) {
            return false;
        }
        break;
    case RigExecWirePathValue::Tag::Matrix4d:
        if (!RigExecWireReadMatrix4d(reader, &value->matrix)) {
            return false;
        }
        break;
    case RigExecWirePathValue::Tag::Vec3d:
        if (!RigExecWireReadVec3d(reader, &value->vec)) {
            return false;
        }
        break;
    case RigExecWirePathValue::Tag::IntArray:
        if (!_ReadI32s(reader, &value->ints)) {
            return false;
        }
        break;
    case RigExecWirePathValue::Tag::FloatArray:
        if (!_ReadF32s(reader, &value->floats)) {
            return false;
        }
        break;
    case RigExecWirePathValue::Tag::Vec2fArray:
        if (!_ReadVec2fs(reader, &value->vec2s)) {
            return false;
        }
        break;
    case RigExecWirePathValue::Tag::Vec3fArray:
        if (!_ReadVec3fs(reader, &value->vec3s)) {
            return false;
        }
        break;
    case RigExecWirePathValue::Tag::Vec3i:
        if (!reader->ReadI32(&value->vec3i[0]) ||
            !reader->ReadI32(&value->vec3i[1]) ||
            !reader->ReadI32(&value->vec3i[2])) {
            return false;
        }
        break;
    case RigExecWirePathValue::Tag::DoubleArray:
        if (!_ReadF64s(reader, &value->doubles)) {
            return false;
        }
        break;
    }
    return true;
}

void
_PutPathRead(std::vector<uint8_t> *out, const RigExecWirePathRead &read)
{
    RigExecWirePutU32(out, read.path);
    RigExecWirePutU8(out, read.wasDefault);
    RigExecWirePutU8(out, read.forceFrame);
    _PutPathValue(out, read.value);
}

bool
_ReadPathRead(RigExecWireReader *reader, RigExecWirePathRead *read)
{
    if (!reader->ReadU32(&read->path) ||
        !reader->ReadU8(&read->wasDefault) || read->wasDefault > 1 ||
        !reader->ReadU8(&read->forceFrame) || read->forceFrame > 1 ||
        !_ReadPathValue(reader, &read->value)) {
        return false;
    }
    return true;
}

void
_PutFrame(std::vector<uint8_t> *out, const RigExecWireFrameInputs &frame)
{
    RigExecWirePutF64(out, frame.frame);
    _PutU32s(out, frame.uids);
    RigExecWirePutU32(out, uint32_t(frame.values.size()));
    for (const RigExecWireValue &value : frame.values) {
        _PutValue(out, value);
    }
    RigExecWirePutU32(out, uint32_t(frame.chainHaveBase.size()));
    for (uint8_t v : frame.chainHaveBase) {
        RigExecWirePutU8(out, v);
    }
    RigExecWirePutU32(out, uint32_t(frame.chainBases.size()));
    for (const std::vector<RigExecWireVec3f> &base : frame.chainBases) {
        _PutVec3fs(out, base);
    }
    RigExecWirePutU32(out, uint32_t(frame.derivedHaveBase.size()));
    for (uint8_t v : frame.derivedHaveBase) {
        RigExecWirePutU8(out, v);
    }
    RigExecWirePutU32(out, uint32_t(frame.derivedBases.size()));
    for (const std::vector<RigExecWireVec3f> &base : frame.derivedBases) {
        _PutVec3fs(out, base);
    }
    _PutMatrices(out, frame.xformBase);
    RigExecWirePutU32(out, uint32_t(frame.nativeFrames.size()));
    for (const RigExecWireFrame &fr : frame.nativeFrames) {
        RigExecWirePutFrame(out, fr);
    }
    _PutU32s(out, frame.propertyPaths);
    RigExecWirePutU32(out, uint32_t(frame.propertyValues.size()));
    for (const RigExecWirePropertyValue &value : frame.propertyValues) {
        _PutPropertyValue(out, value);
    }
    RigExecWirePutU32(out, uint32_t(frame.arrayWeights.size()));
    for (const std::vector<double> &row : frame.arrayWeights) {
        _PutF64s(out, row);
    }
    RigExecWirePutU32(out, uint32_t(frame.arrayTranslationOffsets.size()));
    for (const std::vector<RigExecWireVec3d> &row :
         frame.arrayTranslationOffsets) {
        _PutVec3ds(out, row);
    }
    RigExecWirePutU32(out, uint32_t(frame.arrayRotationOffsets.size()));
    for (const std::vector<RigExecWireVec3d> &row :
         frame.arrayRotationOffsets) {
        _PutVec3ds(out, row);
    }
    RigExecWirePutU32(out, uint32_t(frame.arrayOk.size()));
    for (uint8_t v : frame.arrayOk) {
        RigExecWirePutU8(out, v);
    }
    RigExecWirePutU32(out, uint32_t(frame.arrayPoleWeights.size()));
    for (const std::vector<double> &row : frame.arrayPoleWeights) {
        _PutF64s(out, row);
    }
    RigExecWirePutU32(out, uint32_t(frame.arrayPoleOk.size()));
    for (uint8_t v : frame.arrayPoleOk) {
        RigExecWirePutU8(out, v);
    }
    _PutMatrices(out, frame.deltaBaseMatrix);
    RigExecWirePutU32(out, uint32_t(frame.deltaBaseOk.size()));
    for (uint8_t v : frame.deltaBaseOk) {
        RigExecWirePutU8(out, v);
    }
    RigExecWirePutU32(out, uint32_t(frame.weightPackets.size()));
    for (const RigExecWireWeightPacket &packet : frame.weightPackets) {
        _PutWeightPacket(out, packet);
    }
    _PutU32s(out, frame.currentPhaseWeights);
    _PutF32s3(out, frame.blendWeights);
    _PutF32s4(out, frame.blendActivations);
    _PutVec3fs5(out, frame.blendPoints);
    _PutF32s3(out, frame.derivedBlendWeights);
    _PutF32s4(out, frame.derivedBlendActivations);
    _PutVec3fs5(out, frame.derivedBlendPoints);
    RigExecWirePutU32(out, uint32_t(frame.refusedLayouts.size()));
    for (const RigExecWireRefusedLayout &layout : frame.refusedLayouts) {
        _PutRefusedLayout(out, layout);
    }
    _PutF32s2(out, frame.revisionDefaultWeights);
    _PutPackets2(out, frame.revisionPhasePackets);
    _PutPackets2(out, frame.derivedPhasePackets);
    _PutMatrices2(out, frame.revisionAdjusters);
    _PutU8s2(out, frame.revisionAdjusterHave);
    _PutVec3fs2(out, frame.solverRibbonPoints);
    _PutF32s(out, frame.constraintWeights);
    RigExecWirePutU32(out, uint32_t(frame.constraintHaveWeight.size()));
    for (uint8_t v : frame.constraintHaveWeight) {
        RigExecWirePutU8(out, v);
    }
    RigExecWirePutU32(out, uint32_t(frame.pathReads.size()));
    for (const RigExecWirePathRead &read : frame.pathReads) {
        _PutPathRead(out, read);
    }
}

bool
_ReadU8sChecked(RigExecWireReader *reader, std::vector<uint8_t> *values)
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

bool
_ReadU8sChecked2(RigExecWireReader *reader,
                 std::vector<std::vector<uint8_t>> *values)
{
    uint32_t count = 0;
    if (!reader->ReadU32(&count)) {
        return false;
    }
    values->resize(count);
    for (uint32_t i = 0; i < count; ++i) {
        if (!_ReadU8sChecked(reader, &(*values)[i])) {
            return false;
        }
    }
    return true;
}

bool
_ReadFrame(RigExecWireReader *reader, RigExecWireFrameInputs *frame)
{
    uint32_t count = 0;
    if (!reader->ReadF64(&frame->frame) ||
        !_ReadU32s(reader, &frame->uids) || !reader->ReadU32(&count)) {
        return false;
    }
    frame->values.resize(count);
    for (uint32_t i = 0; i < count; ++i) {
        if (!_ReadValue(reader, &frame->values[i])) {
            return false;
        }
    }
    if (frame->uids.size() != frame->values.size()) {
        return false;
    }
    for (size_t i = 1; i < frame->uids.size(); ++i) {
        if (frame->uids[i - 1] >= frame->uids[i]) {
            return false;
        }
    }
    if (!_ReadU8sChecked(reader, &frame->chainHaveBase) ||
        !reader->ReadU32(&count)) {
        return false;
    }
    frame->chainBases.resize(count);
    for (uint32_t i = 0; i < count; ++i) {
        if (!_ReadVec3fs(reader, &frame->chainBases[i])) {
            return false;
        }
    }
    if (!_ReadU8sChecked(reader, &frame->derivedHaveBase) ||
        !reader->ReadU32(&count)) {
        return false;
    }
    frame->derivedBases.resize(count);
    for (uint32_t i = 0; i < count; ++i) {
        if (!_ReadVec3fs(reader, &frame->derivedBases[i])) {
            return false;
        }
    }
    if (!_ReadMatrices(reader, &frame->xformBase) ||
        !reader->ReadU32(&count)) {
        return false;
    }
    frame->nativeFrames.resize(count);
    for (uint32_t i = 0; i < count; ++i) {
        if (!RigExecWireReadFrame(reader, &frame->nativeFrames[i])) {
            return false;
        }
    }
    if (!_ReadU32s(reader, &frame->propertyPaths) ||
        !reader->ReadU32(&count)) {
        return false;
    }
    frame->propertyValues.resize(count);
    for (uint32_t i = 0; i < count; ++i) {
        if (!_ReadPropertyValue(reader, &frame->propertyValues[i])) {
            return false;
        }
    }
    if (!reader->ReadU32(&count)) {
        return false;
    }
    frame->arrayWeights.resize(count);
    for (uint32_t i = 0; i < count; ++i) {
        if (!_ReadF64s(reader, &frame->arrayWeights[i])) {
            return false;
        }
    }
    if (!reader->ReadU32(&count)) {
        return false;
    }
    frame->arrayTranslationOffsets.resize(count);
    for (uint32_t i = 0; i < count; ++i) {
        if (!_ReadVec3ds(reader, &frame->arrayTranslationOffsets[i])) {
            return false;
        }
    }
    if (!reader->ReadU32(&count)) {
        return false;
    }
    frame->arrayRotationOffsets.resize(count);
    for (uint32_t i = 0; i < count; ++i) {
        if (!_ReadVec3ds(reader, &frame->arrayRotationOffsets[i])) {
            return false;
        }
    }
    if (!_ReadU8sChecked(reader, &frame->arrayOk) ||
        !reader->ReadU32(&count)) {
        return false;
    }
    frame->arrayPoleWeights.resize(count);
    for (uint32_t i = 0; i < count; ++i) {
        if (!_ReadF64s(reader, &frame->arrayPoleWeights[i])) {
            return false;
        }
    }
    if (!_ReadU8sChecked(reader, &frame->arrayPoleOk)) {
        return false;
    }
    if (!_ReadMatrices(reader, &frame->deltaBaseMatrix) ||
        !_ReadU8sChecked(reader, &frame->deltaBaseOk) ||
        !reader->ReadU32(&count)) {
        return false;
    }
    frame->weightPackets.resize(count);
    for (uint32_t i = 0; i < count; ++i) {
        if (!_ReadWeightPacket(reader, &frame->weightPackets[i])) {
            return false;
        }
    }
    if (!_ReadU32s(reader, &frame->currentPhaseWeights) ||
        !_ReadF32s3(reader, &frame->blendWeights) ||
        !_ReadF32s4(reader, &frame->blendActivations) ||
        !_ReadVec3fs5(reader, &frame->blendPoints) ||
        !_ReadF32s3(reader, &frame->derivedBlendWeights) ||
        !_ReadF32s4(reader, &frame->derivedBlendActivations) ||
        !_ReadVec3fs5(reader, &frame->derivedBlendPoints) ||
        !reader->ReadU32(&count)) {
        return false;
    }
    frame->refusedLayouts.resize(count);
    for (uint32_t i = 0; i < count; ++i) {
        if (!_ReadRefusedLayout(reader, &frame->refusedLayouts[i])) {
            return false;
        }
    }
    if (!_ReadF32s2(reader, &frame->revisionDefaultWeights) ||
        !_ReadPackets2(reader, &frame->revisionPhasePackets) ||
        !_ReadPackets2(reader, &frame->derivedPhasePackets) ||
        !_ReadMatrices2(reader, &frame->revisionAdjusters) ||
        !_ReadU8sChecked2(reader, &frame->revisionAdjusterHave) ||
        !_ReadVec3fs2(reader, &frame->solverRibbonPoints) ||
        !_ReadF32s(reader, &frame->constraintWeights) ||
        !_ReadU8sChecked(reader, &frame->constraintHaveWeight) ||
        !reader->ReadU32(&count)) {
        return false;
    }
    frame->pathReads.resize(count);
    for (uint32_t i = 0; i < count; ++i) {
        if (!_ReadPathRead(reader, &frame->pathReads[i])) {
            return false;
        }
    }
    return true;
}

bool
_Fail(std::string *error)
{
    if (error) {
        *error = "malformed input table";
    }
    return false;
}

}  // namespace

bool
RigExecWireEncodeInputTable(const RigExecWireInputTable &table,
                            std::vector<uint8_t> *out)
{
    RigExecWirePutU32(out, uint32_t(table.directory.size()));
    for (const RigExecWireInputDirectoryEntry &entry : table.directory) {
        RigExecWirePutU8(out, uint8_t(entry.tag));
        RigExecWirePutI32(out, entry.overrideIndex);
        RigExecWirePutU32(out, entry.head);
    }
    _PutU32s(out, table.overridablePaths);
    RigExecWirePutU32(out, uint32_t(table.overridableIndices.size()));
    for (const std::vector<int32_t> &row : table.overridableIndices) {
        _PutI32s(out, row);
    }
    RigExecWirePutU32(out, uint32_t(table.frames.size()));
    for (const RigExecWireFrameInputs &frame : table.frames) {
        _PutFrame(out, frame);
    }
    return true;
}

bool
RigExecWireDecodeInputTable(RigExecWireReader *reader,
                            RigExecWireInputTable *table, std::string *error)
{
    uint32_t count = 0;
    if (!reader->ReadU32(&count)) {
        return _Fail(error);
    }
    table->directory.resize(count);
    for (uint32_t i = 0; i < count; ++i) {
        RigExecWireInputDirectoryEntry &entry = table->directory[i];
        uint8_t tag = 0;
        if (!reader->ReadU8(&tag) ||
            tag > uint8_t(RigExecWireInput::Tag::Vec3d) ||
            !reader->ReadI32(&entry.overrideIndex) ||
            !reader->ReadU32(&entry.head)) {
            return _Fail(error);
        }
        entry.tag = RigExecWireInput::Tag(tag);
    }
    if (!_ReadU32s(reader, &table->overridablePaths) ||
        !reader->ReadU32(&count)) {
        return _Fail(error);
    }
    table->overridableIndices.resize(count);
    for (uint32_t i = 0; i < count; ++i) {
        if (!_ReadI32s(reader, &table->overridableIndices[i])) {
            return _Fail(error);
        }
    }
    if (!reader->ReadU32(&count)) {
        return _Fail(error);
    }
    table->frames.resize(count);
    for (uint32_t i = 0; i < count; ++i) {
        if (!_ReadFrame(reader, &table->frames[i])) {
            return _Fail(error);
        }
    }
    if (!reader->Exhausted()) {
        if (error) {
            *error = "trailing bytes in input table";
        }
        return false;
    }
    return true;
}

}  // namespace rigExec
