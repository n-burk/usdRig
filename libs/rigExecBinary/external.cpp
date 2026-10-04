// .rigexec ExternalMovers section: wire encoding.
#include "rigExecBinary/external.h"

namespace rigExec {
namespace {

void
_PutBytes(std::vector<uint8_t> *out, const std::vector<uint8_t> &bytes)
{
    RigExecWirePutU32(out, uint32_t(bytes.size()));
    out->insert(out->end(), bytes.begin(), bytes.end());
}

bool
_ReadBytes(RigExecWireReader *reader, std::vector<uint8_t> *bytes)
{
    uint32_t count = 0;
    bytes->clear();
    return reader->ReadU32(&count) && reader->ReadBytes(count, bytes);
}

bool
_Fail(std::string *error, const char *why)
{
    if (error) {
        *error = std::string("external movers: ") + why;
    }
    return false;
}

}  // namespace

bool
RigExecWireEncodeExternalMovers(const RigExecWireExternalMovers &movers,
                                std::vector<uint8_t> *out)
{
    RigExecWirePutU32(out, uint32_t(movers.revisions.size()));
    for (const RigExecWireExternalRevision &revision : movers.revisions) {
        RigExecWirePutU32(out, revision.chain);
        RigExecWirePutU32(out, revision.revision);
        RigExecWirePutU32(out, revision.type);
        _PutBytes(out, revision.epoch);
    }
    RigExecWirePutU32(out, uint32_t(movers.blobs.size()));
    for (const std::vector<uint8_t> &blob : movers.blobs) {
        _PutBytes(out, blob);
    }
    RigExecWirePutU32(out, uint32_t(movers.frames.size()));
    for (const std::vector<uint32_t> &frame : movers.frames) {
        if (frame.size() != movers.revisions.size()) {
            return false;
        }
        for (uint32_t blob : frame) {
            if (blob != RigExecWireExternalNoFrame &&
                size_t(blob) >= movers.blobs.size()) {
                return false;
            }
            RigExecWirePutU32(out, blob);
        }
    }
    return true;
}

bool
RigExecWireDecodeExternalMovers(RigExecWireReader *reader,
                                RigExecWireExternalMovers *movers,
                                std::string *error)
{
    *movers = RigExecWireExternalMovers();
    uint32_t revisions = 0;
    if (!reader->ReadU32(&revisions)) {
        return _Fail(error, "truncated revision count");
    }
    movers->revisions.resize(revisions);
    for (RigExecWireExternalRevision &revision : movers->revisions) {
        if (!reader->ReadU32(&revision.chain) ||
            !reader->ReadU32(&revision.revision) ||
            !reader->ReadU32(&revision.type) ||
            !_ReadBytes(reader, &revision.epoch)) {
            return _Fail(error, "truncated revision");
        }
    }
    uint32_t blobs = 0;
    if (!reader->ReadU32(&blobs)) {
        return _Fail(error, "truncated blob count");
    }
    movers->blobs.resize(blobs);
    for (std::vector<uint8_t> &blob : movers->blobs) {
        if (!_ReadBytes(reader, &blob)) {
            return _Fail(error, "truncated frame bytes");
        }
    }
    uint32_t frames = 0;
    if (!reader->ReadU32(&frames)) {
        return _Fail(error, "truncated frame count");
    }
    movers->frames.resize(frames);
    for (std::vector<uint32_t> &frame : movers->frames) {
        frame.resize(revisions);
        for (uint32_t &blob : frame) {
            if (!reader->ReadU32(&blob)) {
                return _Fail(error, "truncated frame table");
            }
            if (blob != RigExecWireExternalNoFrame && blob >= blobs) {
                return _Fail(error, "a frame names no blob");
            }
        }
    }
    if (!reader->Exhausted()) {
        return _Fail(error, "trailing bytes");
    }
    return true;
}

}  // namespace rigExec
