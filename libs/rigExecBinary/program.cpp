//
// .rigexec program sections: wire encoding.
//

#include "rigExecBinary/program.h"

#include <cstring>

namespace rigExec {
namespace {

bool
_PutEnum(std::vector<uint8_t> *out, uint8_t value, uint8_t last)
{
    if (value > last) {
        return false;
    }
    RigExecWirePutU8(out, value);
    return true;
}

bool
_ReadEnum(RigExecWireReader *reader, uint8_t last, uint8_t *out)
{
    uint8_t value = 0;
    if (!reader->ReadU8(&value) || value > last) {
        return false;
    }
    *out = value;
    return true;
}

void
_PutRange(std::vector<uint8_t> *out, const RigExecWireSlotRange &range)
{
    RigExecWirePutU8(out, uint8_t(range.domain));
    RigExecWirePutU32(out, range.begin);
    RigExecWirePutU32(out, range.end);
}

bool
_ReadRange(RigExecWireReader *reader, RigExecWireSlotRange *range)
{
    uint8_t domain = 0;
    if (!_ReadEnum(reader, uint8_t(RigExecWireSlotDomain::Snapshots),
                   &domain) ||
        !reader->ReadU32(&range->begin) || !reader->ReadU32(&range->end)) {
        return false;
    }
    range->domain = RigExecWireSlotDomain(domain);
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
_PutI32sNested(std::vector<uint8_t> *out,
               const std::vector<std::vector<int32_t>> &values)
{
    RigExecWirePutU32(out, uint32_t(values.size()));
    for (const std::vector<int32_t> &row : values) {
        _PutI32s(out, row);
    }
}

bool
_ReadI32sNested(RigExecWireReader *reader,
                std::vector<std::vector<int32_t>> *values)
{
    uint32_t count = 0;
    if (!reader->ReadU32(&count)) {
        return false;
    }
    values->resize(count);
    for (uint32_t i = 0; i < count; ++i) {
        if (!_ReadI32s(reader, &(*values)[i])) {
            return false;
        }
    }
    return true;
}

void
_PutClusterSet(std::vector<uint8_t> *out,
               const RigExecWireClusterSet &set)
{
    RigExecWirePutU32(out, set.clusters);
    RigExecWirePutU32(out, uint32_t(set.words.size()));
    for (uint64_t word : set.words) {
        RigExecWirePutU64(out, word);
    }
}

bool
_ReadClusterSet(RigExecWireReader *reader, RigExecWireClusterSet *set)
{
    uint32_t words = 0;
    if (!reader->ReadU32(&set->clusters) || !reader->ReadU32(&words)) {
        return false;
    }
    set->words.resize(words);
    for (uint32_t i = 0; i < words; ++i) {
        if (!reader->ReadU64(&set->words[i])) {
            return false;
        }
    }
    return true;
}

bool
_CheckExhausted(RigExecWireReader *reader, std::string *error,
                const char *what)
{
    if (!reader->Exhausted()) {
        if (error) {
            *error = std::string("trailing bytes in ") + what;
        }
        return false;
    }
    return true;
}

bool
_Fail(std::string *error, const char *what)
{
    if (error) {
        *error = std::string("malformed ") + what;
    }
    return false;
}

}  // namespace

void
RigExecWirePutU8(std::vector<uint8_t> *out, uint8_t value)
{
    out->push_back(value);
}

void
RigExecWirePutU32(std::vector<uint8_t> *out, uint32_t value)
{
    out->push_back(uint8_t(value & 0xffu));
    out->push_back(uint8_t((value >> 8) & 0xffu));
    out->push_back(uint8_t((value >> 16) & 0xffu));
    out->push_back(uint8_t((value >> 24) & 0xffu));
}

void
RigExecWirePutI32(std::vector<uint8_t> *out, int32_t value)
{
    RigExecWirePutU32(out, uint32_t(value));
}

void
RigExecWirePutU64(std::vector<uint8_t> *out, uint64_t value)
{
    for (int shift = 0; shift < 64; shift += 8) {
        out->push_back(uint8_t((value >> shift) & 0xffu));
    }
}

void
RigExecWirePutF32(std::vector<uint8_t> *out, float value)
{
    uint32_t bits = 0;
    std::memcpy(&bits, &value, sizeof(bits));
    RigExecWirePutU32(out, bits);
}

void
RigExecWirePutF64(std::vector<uint8_t> *out, double value)
{
    uint64_t bits = 0;
    std::memcpy(&bits, &value, sizeof(bits));
    RigExecWirePutU64(out, bits);
}

bool
RigExecWireReader::ReadU8(uint8_t *out)
{
    if (_at + 1 > _size) {
        return false;
    }
    *out = _data[_at++];
    return true;
}

bool
RigExecWireReader::ReadU32(uint32_t *out)
{
    if (_at + 4 > _size) {
        return false;
    }
    *out = uint32_t(_data[_at]) | (uint32_t(_data[_at + 1]) << 8) |
           (uint32_t(_data[_at + 2]) << 16) |
           (uint32_t(_data[_at + 3]) << 24);
    _at += 4;
    return true;
}

bool
RigExecWireReader::ReadI32(int32_t *out)
{
    uint32_t bits = 0;
    if (!ReadU32(&bits)) {
        return false;
    }
    *out = int32_t(bits);
    return true;
}

bool
RigExecWireReader::ReadU64(uint64_t *out)
{
    if (_at + 8 > _size) {
        return false;
    }
    uint64_t value = 0;
    for (int i = 7; i >= 0; --i) {
        value = (value << 8) | _data[_at + size_t(i)];
    }
    *out = value;
    _at += 8;
    return true;
}

bool
RigExecWireReader::ReadF32(float *out)
{
    uint32_t bits = 0;
    if (!ReadU32(&bits)) {
        return false;
    }
    std::memcpy(out, &bits, sizeof(*out));
    return true;
}

bool
RigExecWireReader::ReadF64(double *out)
{
    uint64_t bits = 0;
    if (!ReadU64(&bits)) {
        return false;
    }
    std::memcpy(out, &bits, sizeof(*out));
    return true;
}

void
RigExecWirePutVec3d(std::vector<uint8_t> *out,
                    const RigExecWireVec3d &value)
{
    for (double v : value) {
        RigExecWirePutF64(out, v);
    }
}

void
RigExecWirePutVec3f(std::vector<uint8_t> *out,
                    const RigExecWireVec3f &value)
{
    for (float v : value) {
        RigExecWirePutF32(out, v);
    }
}

void
RigExecWirePutVec2f(std::vector<uint8_t> *out,
                    const RigExecWireVec2f &value)
{
    for (float v : value) {
        RigExecWirePutF32(out, v);
    }
}

void
RigExecWirePutMatrix4d(std::vector<uint8_t> *out,
                       const RigExecWireMatrix4d &value)
{
    for (double v : value) {
        RigExecWirePutF64(out, v);
    }
}

void
RigExecWirePutFrame(std::vector<uint8_t> *out,
                    const RigExecWireFrame &value)
{
    for (const RigExecWireVec3d &point : value.points) {
        RigExecWirePutVec3d(out, point);
    }
    RigExecWirePutU32(out, value.flags);
}

bool
RigExecWireReadVec3d(RigExecWireReader *reader, RigExecWireVec3d *value)
{
    for (double &v : *value) {
        if (!reader->ReadF64(&v)) {
            return false;
        }
    }
    return true;
}

bool
RigExecWireReadVec3f(RigExecWireReader *reader, RigExecWireVec3f *value)
{
    for (float &v : *value) {
        if (!reader->ReadF32(&v)) {
            return false;
        }
    }
    return true;
}

bool
RigExecWireReadVec2f(RigExecWireReader *reader, RigExecWireVec2f *value)
{
    for (float &v : *value) {
        if (!reader->ReadF32(&v)) {
            return false;
        }
    }
    return true;
}

bool
RigExecWireReadMatrix4d(RigExecWireReader *reader,
                        RigExecWireMatrix4d *value)
{
    for (double &v : *value) {
        if (!reader->ReadF64(&v)) {
            return false;
        }
    }
    return true;
}

bool
RigExecWireReadFrame(RigExecWireReader *reader, RigExecWireFrame *value)
{
    for (RigExecWireVec3d &point : value->points) {
        if (!RigExecWireReadVec3d(reader, &point)) {
            return false;
        }
    }
    return reader->ReadU32(&value->flags);
}

bool
RigExecWireEncodeSteps(const std::vector<RigExecWireStep> &steps,
                       std::vector<uint8_t> *out)
{
    RigExecWirePutU32(out, uint32_t(steps.size()));
    for (const RigExecWireStep &step : steps) {
        if (!_PutEnum(out, uint8_t(step.kind),
                      uint8_t(RigExecWireStepKind::Derived))) {
            return false;
        }
        RigExecWirePutI32(out, step.object);
        RigExecWirePutI32(out, step.part);
        RigExecWirePutU32(out, uint32_t(step.reads.size()));
        for (const RigExecWireSlotRange &range : step.reads) {
            _PutRange(out, range);
        }
        RigExecWirePutU32(out, uint32_t(step.writes.size()));
        for (const RigExecWireSlotRange &range : step.writes) {
            _PutRange(out, range);
        }
        _PutI32s(out, step.preds);
        _PutI32s(out, step.succs);
        const uint8_t flags =
            uint8_t(step.isSource ? 1 : 0) |
            uint8_t(step.externalReads ? 2 : 0) |
            uint8_t(step.varyingInputs ? 4 : 0) |
            uint8_t(step.resolvedInputReads ? 8 : 0);
        RigExecWirePutU8(out, flags);
        _PutI32s(out, step.overrideInputs);
        RigExecWirePutI32(out, step.cluster);
        RigExecWirePutI32(out, step.level);
        RigExecWirePutF64(out, step.sizeUnits);
        RigExecWirePutF64(out, step.cost);
        RigExecWirePutU32(out, step.maxDiagnostics);
        RigExecWirePutU32(out, step.label);
    }
    return true;
}

bool
RigExecWireDecodeSteps(RigExecWireReader *reader,
                       std::vector<RigExecWireStep> *steps, std::string *error)
{
    uint32_t count = 0;
    if (!reader->ReadU32(&count)) {
        return _Fail(error, "steps");
    }
    steps->resize(count);
    for (uint32_t i = 0; i < count; ++i) {
        RigExecWireStep &step = (*steps)[i];
        uint8_t kind = 0;
        uint32_t reads = 0, writes = 0;
        uint8_t flags = 0;
        if (!_ReadEnum(reader, uint8_t(RigExecWireStepKind::Derived),
                       &kind) ||
            !reader->ReadI32(&step.object) ||
            !reader->ReadI32(&step.part) || !reader->ReadU32(&reads)) {
            return _Fail(error, "steps");
        }
        step.kind = RigExecWireStepKind(kind);
        step.reads.resize(reads);
        for (uint32_t r = 0; r < reads; ++r) {
            if (!_ReadRange(reader, &step.reads[r])) {
                return _Fail(error, "steps");
            }
        }
        if (!reader->ReadU32(&writes)) {
            return _Fail(error, "steps");
        }
        step.writes.resize(writes);
        for (uint32_t w = 0; w < writes; ++w) {
            if (!_ReadRange(reader, &step.writes[w])) {
                return _Fail(error, "steps");
            }
        }
        if (!_ReadI32s(reader, &step.preds) ||
            !_ReadI32s(reader, &step.succs) || !reader->ReadU8(&flags) ||
            (flags & ~uint8_t(15)) != 0) {
            return _Fail(error, "steps");
        }
        step.isSource = (flags & 1) != 0;
        step.externalReads = (flags & 2) != 0;
        step.varyingInputs = (flags & 4) != 0;
        step.resolvedInputReads = (flags & 8) != 0;
        if (!_ReadI32s(reader, &step.overrideInputs) ||
            !reader->ReadI32(&step.cluster) ||
            !reader->ReadI32(&step.level) ||
            !reader->ReadF64(&step.sizeUnits) ||
            !reader->ReadF64(&step.cost) ||
            !reader->ReadU32(&step.maxDiagnostics) ||
            !reader->ReadU32(&step.label)) {
            return _Fail(error, "steps");
        }
    }
    return _CheckExhausted(reader, error, "steps");
}

bool
RigExecWireEncodeClustering(const RigExecWireClustering &clustering,
                            std::vector<uint8_t> *out)
{
    RigExecWirePutU32(out, uint32_t(clustering.clusters.size()));
    for (const RigExecWireCluster &cluster : clustering.clusters) {
        _PutI32s(out, cluster.members);
        _PutI32s(out, cluster.preds);
        _PutI32s(out, cluster.succs);
        RigExecWirePutF64(out, cluster.cost);
        RigExecWirePutI32(out, cluster.level);
    }
    _PutI32s(out, clustering.clusterOf);
    RigExecWirePutF64(out, clustering.grainUs);
    RigExecWirePutF64(out, clustering.serialCost);
    RigExecWirePutF64(out, clustering.criticalPathCost);
    return true;
}

bool
RigExecWireDecodeClustering(RigExecWireReader *reader,
                            RigExecWireClustering *clustering,
                            std::string *error)
{
    uint32_t count = 0;
    if (!reader->ReadU32(&count)) {
        return _Fail(error, "clusters");
    }
    clustering->clusters.resize(count);
    for (uint32_t i = 0; i < count; ++i) {
        RigExecWireCluster &cluster = clustering->clusters[i];
        if (!_ReadI32s(reader, &cluster.members) ||
            !_ReadI32s(reader, &cluster.preds) ||
            !_ReadI32s(reader, &cluster.succs) ||
            !reader->ReadF64(&cluster.cost) ||
            !reader->ReadI32(&cluster.level)) {
            return _Fail(error, "clusters");
        }
    }
    if (!_ReadI32s(reader, &clustering->clusterOf) ||
        !reader->ReadF64(&clustering->grainUs) ||
        !reader->ReadF64(&clustering->serialCost) ||
        !reader->ReadF64(&clustering->criticalPathCost)) {
        return _Fail(error, "clusters");
    }
    return _CheckExhausted(reader, error, "clusters");
}

bool
RigExecWireEncodeCones(const RigExecWireCones &cones,
                       std::vector<uint8_t> *out)
{
    RigExecWirePutU32(out, uint32_t(cones.cone.size()));
    for (const RigExecWireClusterSet &set : cones.cone) {
        _PutClusterSet(out, set);
    }
    _PutClusterSet(out, cones.always);
    _PutClusterSet(out, cones.poseClusters);
    _PutI32s(out, cones.avarCluster);
    _PutI32sNested(out, cones.chainBaseClusters);
    _PutI32sNested(out, cones.solverPointsClusters);
    _PutI32sNested(out, cones.revisionClusters);
    _PutI32s(out, cones.revisionStaticCluster);
    _PutI32sNested(out, cones.nativeSourceClusters);
    _PutI32sNested(out, cones.deltaBaseClusters);
    _PutI32sNested(out, cones.constraintArrayClusters);
    _PutI32s(out, cones.varyingSteps);
    _PutI32s(out, cones.overrideSteps);
    return true;
}

bool
RigExecWireDecodeCones(RigExecWireReader *reader, RigExecWireCones *cones,
                       std::string *error)
{
    uint32_t count = 0;
    if (!reader->ReadU32(&count)) {
        return _Fail(error, "cones");
    }
    cones->cone.resize(count);
    for (uint32_t i = 0; i < count; ++i) {
        if (!_ReadClusterSet(reader, &cones->cone[i])) {
            return _Fail(error, "cones");
        }
    }
    if (!_ReadClusterSet(reader, &cones->always) ||
        !_ReadClusterSet(reader, &cones->poseClusters) ||
        !_ReadI32s(reader, &cones->avarCluster) ||
        !_ReadI32sNested(reader, &cones->chainBaseClusters) ||
        !_ReadI32sNested(reader, &cones->solverPointsClusters) ||
        !_ReadI32sNested(reader, &cones->revisionClusters) ||
        !_ReadI32s(reader, &cones->revisionStaticCluster) ||
        !_ReadI32sNested(reader, &cones->nativeSourceClusters) ||
        !_ReadI32sNested(reader, &cones->deltaBaseClusters) ||
        !_ReadI32sNested(reader, &cones->constraintArrayClusters) ||
        !_ReadI32s(reader, &cones->varyingSteps) ||
        !_ReadI32s(reader, &cones->overrideSteps)) {
        return _Fail(error, "cones");
    }
    return _CheckExhausted(reader, error, "cones");
}

bool
RigExecWireEncodeSlotMeta(const RigExecWireSlotMeta &meta,
                          std::vector<uint8_t> *out)
{
    _PutU32s(out, meta.paths);
    RigExecWirePutU32(out, uint32_t(meta.slotKind.size()));
    for (RigExecWireSlotKind kind : meta.slotKind) {
        if (!_PutEnum(out, uint8_t(kind),
                      uint8_t(RigExecWireSlotKind::XformDerived))) {
            return false;
        }
    }
    _PutI32s(out, meta.parent);
    _PutI32s(out, meta.propParent);
    _PutI32s(out, meta.xformSlots);
    _PutU32s(out, meta.xformPaths);
    _PutI32s(out, meta.jointSlots);
    _PutU32s(out, meta.jointPaths);
    _PutI32s(out, meta.controlSlots);
    _PutU32s(out, meta.controlPaths);
    _PutU32s(out, meta.solverArrayPaths);
    _PutI32s(out, meta.solverArrayElements);
    _PutI32s(out, meta.jointPublishOrder);
    _PutI32s(out, meta.controlPublishOrder);
    _PutI32s(out, meta.solverPublishOrder);
    const uint8_t flags =
        uint8_t(meta.jointPathsAscending ? 1 : 0) |
        uint8_t(meta.controlPathsAscending ? 2 : 0) |
        uint8_t(meta.solverArraysAscending ? 4 : 0);
    RigExecWirePutU8(out, flags);
    RigExecWirePutU32(out, uint32_t(meta.needFinal.size()));
    for (uint8_t v : meta.needFinal) {
        RigExecWirePutU8(out, v ? uint8_t(1) : uint8_t(0));
    }
    RigExecWirePutU32(out, uint32_t(meta.needBase.size()));
    for (uint8_t v : meta.needBase) {
        RigExecWirePutU8(out, v ? uint8_t(1) : uint8_t(0));
    }
    return true;
}

bool
RigExecWireDecodeSlotMeta(RigExecWireReader *reader,
                          RigExecWireSlotMeta *meta, std::string *error)
{
    uint32_t kinds = 0;
    uint8_t flags = 0;
    uint32_t finals = 0, bases = 0;
    if (!_ReadU32s(reader, &meta->paths) || !reader->ReadU32(&kinds)) {
        return _Fail(error, "slot inventory");
    }
    meta->slotKind.resize(kinds);
    for (uint32_t i = 0; i < kinds; ++i) {
        uint8_t kind = 0;
        if (!_ReadEnum(reader, uint8_t(RigExecWireSlotKind::XformDerived),
                       &kind)) {
            return _Fail(error, "slot inventory");
        }
        meta->slotKind[i] = RigExecWireSlotKind(kind);
    }
    if (!_ReadI32s(reader, &meta->parent) ||
        !_ReadI32s(reader, &meta->propParent) ||
        !_ReadI32s(reader, &meta->xformSlots) ||
        !_ReadU32s(reader, &meta->xformPaths) ||
        !_ReadI32s(reader, &meta->jointSlots) ||
        !_ReadU32s(reader, &meta->jointPaths) ||
        !_ReadI32s(reader, &meta->controlSlots) ||
        !_ReadU32s(reader, &meta->controlPaths) ||
        !_ReadU32s(reader, &meta->solverArrayPaths) ||
        !_ReadI32s(reader, &meta->solverArrayElements) ||
        !_ReadI32s(reader, &meta->jointPublishOrder) ||
        !_ReadI32s(reader, &meta->controlPublishOrder) ||
        !_ReadI32s(reader, &meta->solverPublishOrder) ||
        !reader->ReadU8(&flags) || (flags & ~uint8_t(7)) != 0 ||
        !reader->ReadU32(&finals)) {
        return _Fail(error, "slot inventory");
    }
    meta->jointPathsAscending = (flags & 1) != 0;
    meta->controlPathsAscending = (flags & 2) != 0;
    meta->solverArraysAscending = (flags & 4) != 0;
    meta->needFinal.resize(finals);
    for (uint32_t i = 0; i < finals; ++i) {
        uint8_t v = 0;
        if (!reader->ReadU8(&v) || v > 1) {
            return _Fail(error, "slot inventory");
        }
        meta->needFinal[i] = v;
    }
    if (!reader->ReadU32(&bases)) {
        return _Fail(error, "slot inventory");
    }
    meta->needBase.resize(bases);
    for (uint32_t i = 0; i < bases; ++i) {
        uint8_t v = 0;
        if (!reader->ReadU8(&v) || v > 1) {
            return _Fail(error, "slot inventory");
        }
        meta->needBase[i] = v;
    }
    return _CheckExhausted(reader, error, "slot inventory");
}

bool
RigExecWireEncodeConstants(const RigExecWireConstants &constants,
                           std::vector<uint8_t> *out)
{
    RigExecWirePutU32(out, uint32_t(constants.restM.size()));
    for (const RigExecWireMatrix4d &m : constants.restM) {
        RigExecWirePutMatrix4d(out, m);
    }
    RigExecWirePutU32(out, uint32_t(constants.restPts.size()));
    for (const std::array<RigExecWireVec3d, 4> &pts : constants.restPts) {
        for (const RigExecWireVec3d &p : pts) {
            RigExecWirePutVec3d(out, p);
        }
    }
    RigExecWirePutU32(out, uint32_t(constants.restFrames.size()));
    for (const RigExecWireFrame &f : constants.restFrames) {
        RigExecWirePutFrame(out, f);
    }
    RigExecWirePutU32(out, uint32_t(constants.selfD.size()));
    for (const RigExecWireMatrix4d &m : constants.selfD) {
        RigExecWirePutMatrix4d(out, m);
    }
    RigExecWirePutU32(out, uint32_t(constants.parentDinv.size()));
    for (const RigExecWireMatrix4d &m : constants.parentDinv) {
        RigExecWirePutMatrix4d(out, m);
    }
    _PutU32s(out, constants.rotOrder);
    RigExecWirePutU32(out, uint32_t(constants.restRoundTrip.size()));
    for (const RigExecWireMatrix4d &m : constants.restRoundTrip) {
        RigExecWirePutMatrix4d(out, m);
    }
    RigExecWirePutU32(out, uint32_t(constants.defaultRoundTrip.size()));
    for (const RigExecWireMatrix4d &m : constants.defaultRoundTrip) {
        RigExecWirePutMatrix4d(out, m);
    }
    RigExecWirePutU32(out, uint32_t(constants.posedAuthored.size()));
    for (uint8_t v : constants.posedAuthored) {
        RigExecWirePutU8(out, v ? uint8_t(1) : uint8_t(0));
    }
    RigExecWirePutU32(out, uint32_t(constants.posedAuthoredM.size()));
    for (const RigExecWireMatrix4d &m : constants.posedAuthoredM) {
        RigExecWirePutMatrix4d(out, m);
    }
    RigExecWirePutU32(out, uint32_t(constants.noScaleAvars.size()));
    for (uint8_t v : constants.noScaleAvars) {
        RigExecWirePutU8(out, v ? uint8_t(1) : uint8_t(0));
    }
    RigExecWirePutU32(out, uint32_t(constants.avarConstants.size()));
    for (double v : constants.avarConstants) {
        RigExecWirePutF64(out, v);
    }
    return true;
}

bool
RigExecWireDecodeConstants(RigExecWireReader *reader,
                           RigExecWireConstants *constants,
                           std::string *error)
{
    uint32_t count = 0;
#define _RIGEXEC_WIRE_MATRICES(field)                                          \
    if (!reader->ReadU32(&count)) {                                            \
        return _Fail(error, "constants");                                      \
    }                                                                          \
    constants->field.resize(count);                                            \
    for (uint32_t i = 0; i < count; ++i) {                                      \
        if (!RigExecWireReadMatrix4d(reader, &constants->field[i])) {           \
            return _Fail(error, "constants");                                  \
        }                                                                      \
    }
    _RIGEXEC_WIRE_MATRICES(restM)
    if (!reader->ReadU32(&count)) {
        return _Fail(error, "constants");
    }
    constants->restPts.resize(count);
    for (uint32_t i = 0; i < count; ++i) {
        for (RigExecWireVec3d &p : constants->restPts[i]) {
            if (!RigExecWireReadVec3d(reader, &p)) {
                return _Fail(error, "constants");
            }
        }
    }
    if (!reader->ReadU32(&count)) {
        return _Fail(error, "constants");
    }
    constants->restFrames.resize(count);
    for (uint32_t i = 0; i < count; ++i) {
        if (!RigExecWireReadFrame(reader, &constants->restFrames[i])) {
            return _Fail(error, "constants");
        }
    }
    _RIGEXEC_WIRE_MATRICES(selfD)
    _RIGEXEC_WIRE_MATRICES(parentDinv)
#undef _RIGEXEC_WIRE_MATRICES
    if (!_ReadU32s(reader, &constants->rotOrder)) {
        return _Fail(error, "constants");
    }
    uint32_t trips = 0;
    if (!reader->ReadU32(&trips)) {
        return _Fail(error, "constants");
    }
    constants->restRoundTrip.resize(trips);
    for (uint32_t i = 0; i < trips; ++i) {
        if (!RigExecWireReadMatrix4d(reader, &constants->restRoundTrip[i])) {
            return _Fail(error, "constants");
        }
    }
    if (!reader->ReadU32(&trips)) {
        return _Fail(error, "constants");
    }
    constants->defaultRoundTrip.resize(trips);
    for (uint32_t i = 0; i < trips; ++i) {
        if (!RigExecWireReadMatrix4d(reader,
                                     &constants->defaultRoundTrip[i])) {
            return _Fail(error, "constants");
        }
    }
    uint32_t authored = 0;
    if (!reader->ReadU32(&authored)) {
        return _Fail(error, "constants");
    }
    constants->posedAuthored.resize(authored);
    for (uint32_t i = 0; i < authored; ++i) {
        uint8_t v = 0;
        if (!reader->ReadU8(&v) || v > 1) {
            return _Fail(error, "constants");
        }
        constants->posedAuthored[i] = v;
    }
    uint32_t authoredM = 0;
    if (!reader->ReadU32(&authoredM)) {
        return _Fail(error, "constants");
    }
    constants->posedAuthoredM.resize(authoredM);
    for (uint32_t i = 0; i < authoredM; ++i) {
        if (!RigExecWireReadMatrix4d(reader, &constants->posedAuthoredM[i])) {
            return _Fail(error, "constants");
        }
    }
    uint32_t noScale = 0;
    if (!reader->ReadU32(&noScale)) {
        return _Fail(error, "constants");
    }
    constants->noScaleAvars.resize(noScale);
    for (uint32_t i = 0; i < noScale; ++i) {
        uint8_t v = 0;
        if (!reader->ReadU8(&v) || v > 1) {
            return _Fail(error, "constants");
        }
        constants->noScaleAvars[i] = v;
    }
    uint32_t avars = 0;
    if (!reader->ReadU32(&avars)) {
        return _Fail(error, "constants");
    }
    constants->avarConstants.resize(avars);
    for (uint32_t i = 0; i < avars; ++i) {
        if (!reader->ReadF64(&constants->avarConstants[i])) {
            return _Fail(error, "constants");
        }
    }
    return _CheckExhausted(reader, error, "constants");
}

}  // namespace rigExec
