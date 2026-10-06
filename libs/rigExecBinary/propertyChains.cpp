#include "rigExecBinary/propertyChains.h"

namespace rigExec {

namespace {

void
PutInput(std::vector<uint8_t> *out, const RigExecWirePropertyChainInput &input)
{
    RigExecWirePutU8(out, uint8_t(input.kind));
    RigExecWirePutF64(out, input.constant);
    RigExecWirePutU32(out, uint32_t(input.hops.size()));
    for (uint32_t hop : input.hops) {
        RigExecWirePutU32(out, hop);
    }
    RigExecWirePutU32(out, uint32_t(input.frameValues.size()));
    for (size_t i = 0; i < input.frameValues.size(); ++i) {
        RigExecWirePutU8(out, i < input.frameHave.size()
                                  ? input.frameHave[i]
                                  : uint8_t(0));
        RigExecWirePutF64(out, input.frameValues[i]);
    }
}

bool
ReadInput(RigExecWireReader *reader, RigExecWirePropertyChainInput *input)
{
    uint8_t kind = 0;
    uint32_t count = 0;
    if (!reader->ReadU8(&kind) || kind > 2 ||
        !reader->ReadF64(&input->constant) || !reader->ReadU32(&count)) {
        return false;
    }
    input->kind = RigExecWirePropertyChainInput::Kind(kind);
    input->hops.resize(count);
    for (uint32_t &hop : input->hops) {
        if (!reader->ReadU32(&hop)) {
            return false;
        }
    }
    if (!reader->ReadU32(&count)) {
        return false;
    }
    input->frameHave.resize(count);
    input->frameValues.resize(count);
    for (uint32_t i = 0; i < count; ++i) {
        if (!reader->ReadU8(&input->frameHave[i]) ||
            !reader->ReadF64(&input->frameValues[i])) {
            return false;
        }
    }
    return true;
}

void
PutVec2fs(std::vector<uint8_t> *out,
          const std::vector<RigExecWireVec2f> &values)
{
    RigExecWirePutU32(out, uint32_t(values.size()));
    for (const RigExecWireVec2f &v : values) {
        RigExecWirePutVec2f(out, v);
    }
}

bool
ReadVec2fs(RigExecWireReader *reader, std::vector<RigExecWireVec2f> *values)
{
    uint32_t count = 0;
    if (!reader->ReadU32(&count)) {
        return false;
    }
    values->resize(count);
    for (RigExecWireVec2f &v : *values) {
        if (!RigExecWireReadVec2f(reader, &v)) {
            return false;
        }
    }
    return true;
}

}  // namespace

bool
RigExecWireEncodePropertyChains(const RigExecWirePropertyChains &chains,
                                std::vector<uint8_t> *out)
{
    if (!out) {
        return false;
    }
    RigExecWirePutU32(out, uint32_t(chains.chains.size()));
    for (const RigExecWirePropertyChain &chain : chains.chains) {
        RigExecWirePutU32(out, chain.target);
        RigExecWirePutU8(out, uint8_t(chain.valueType));
        RigExecWirePutU32(out, uint32_t(chain.frameBase.size()));
        for (size_t i = 0; i < chain.frameBase.size(); ++i) {
            RigExecWirePutU8(out, i < chain.frameBaseHave.size()
                                      ? chain.frameBaseHave[i]
                                      : uint8_t(0));
            RigExecWirePutF64(out, chain.frameBase[i]);
        }
        RigExecWirePutU32(out, uint32_t(chain.revisions.size()));
        for (const RigExecWirePropertyChainRevision &revision : chain.revisions) {
            RigExecWirePutU32(out, revision.mover);
            RigExecWirePutU8(out, revision.op);
            PutInput(out, revision.enabled);
            PutInput(out, revision.defaultWeight);
            PutInput(out, revision.value);
            PutInput(out, revision.minimum);
            PutInput(out, revision.maximum);
            PutVec2fs(out, revision.keys);
            PutVec2fs(out, revision.tangents);
        }
    }
    RigExecWirePutU32(out, uint32_t(chains.consumers.size()));
    for (const RigExecWirePropertyChainConsumer &consumer : chains.consumers) {
        RigExecWirePutU32(out, consumer.uid);
        RigExecWirePutU32(out, consumer.chain);
    }
    return true;
}

bool
RigExecWireDecodePropertyChains(RigExecWireReader *reader,
                                RigExecWirePropertyChains *chains,
                                std::string *error)
{
    const auto fail = [error](const char *what) {
        if (error) {
            *error = std::string("malformed property chains: ") + what;
        }
        return false;
    };
    uint32_t count = 0;
    if (!reader->ReadU32(&count)) {
        return fail("chain count");
    }
    chains->chains.resize(count);
    for (RigExecWirePropertyChain &chain : chains->chains) {
        uint8_t type = 0;
        uint32_t frames = 0;
        if (!reader->ReadU32(&chain.target) || !reader->ReadU8(&type) ||
            type > 1 || !reader->ReadU32(&frames)) {
            return fail("chain header");
        }
        chain.valueType = RigExecWirePropertyChain::ValueType(type);
        chain.frameBaseHave.resize(frames);
        chain.frameBase.resize(frames);
        for (uint32_t i = 0; i < frames; ++i) {
            if (!reader->ReadU8(&chain.frameBaseHave[i]) ||
                !reader->ReadF64(&chain.frameBase[i])) {
                return fail("chain base");
            }
        }
        uint32_t revisions = 0;
        if (!reader->ReadU32(&revisions)) {
            return fail("revision count");
        }
        chain.revisions.resize(revisions);
        for (RigExecWirePropertyChainRevision &revision : chain.revisions) {
            if (!reader->ReadU32(&revision.mover) ||
                !reader->ReadU8(&revision.op) || revision.op > 5 ||
                !ReadInput(reader, &revision.enabled) ||
                !ReadInput(reader, &revision.defaultWeight) ||
                !ReadInput(reader, &revision.value) ||
                !ReadInput(reader, &revision.minimum) ||
                !ReadInput(reader, &revision.maximum) ||
                !ReadVec2fs(reader, &revision.keys) ||
                !ReadVec2fs(reader, &revision.tangents)) {
                return fail("revision");
            }
        }
    }
    if (!reader->ReadU32(&count)) {
        return fail("consumer count");
    }
    chains->consumers.resize(count);
    for (RigExecWirePropertyChainConsumer &consumer : chains->consumers) {
        if (!reader->ReadU32(&consumer.uid) ||
            !reader->ReadU32(&consumer.chain) ||
            consumer.chain >= chains->chains.size()) {
            return fail("consumer");
        }
    }
    return true;
}

}  // namespace rigExec
