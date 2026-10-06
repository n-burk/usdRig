// Runtime-suite helper: a baked .rigexec unpacked into its object form,
// edited, and packed again without validation, so a defect the format's
// validator or the runtime refuses reaches Open; the edits that add an
// input slot to a hand-built file where an export would list one (a path,
// a value, a listed slot at its place, every slot id after it renumbered);
// and the lookups of a slot and a path-read row by text.
// Included AFTER the suite's CHECK macro, which reports through it.
#ifndef RIGEXEC_TESTS_FILE_EDIT_H
#define RIGEXEC_TESTS_FILE_EDIT_H

#include "rigExecBinary/format.h"

#include <algorithm>
#include <cstddef>
#include <cstdint>
#include <cstdio>
#include <initializer_list>
#include <memory>
#include <string>
#include <vector>

// The object form of \p bytes, through RigExecFormatOpen; null, with a
// failed CHECK, when it does not open.
static std::unique_ptr<rigExec::fb::RigExecWireFile>
RigExecTestUnpack(const std::vector<uint8_t> &bytes)
{
    std::unique_ptr<rigExec::fb::RigExecWireFile> file;
    std::string error;
    const bool opened =
        rigExec::RigExecFormatOpen(bytes.data(), bytes.size(), &file, &error);
    CHECK(opened);
    if (!opened) {
        std::printf("unpack: %s\n", error.c_str());
    }
    return file;
}

// \p file packed with the file identifier and no validation: what
// RigExecFormatWrite would write if it accepted the file.
static std::vector<uint8_t>
RigExecTestPackUnchecked(const rigExec::fb::RigExecWireFile &file)
{
    flatbuffers::FlatBufferBuilder builder(1u << 16);
    rigExec::fb::FinishFileBuffer(builder,
                                  rigExec::fb::File::Pack(builder, &file));
    return std::vector<uint8_t>(builder.GetBufferPointer(),
                                builder.GetBufferPointer() +
                                    builder.GetSize());
}

// \p bytes unpacked, changed by \p edit, and packed again unchecked; empty
// when \p bytes does not open.
template <class Edit>
static std::vector<uint8_t>
RigExecTestEdited(const std::vector<uint8_t> &bytes, const Edit &edit)
{
    const std::unique_ptr<rigExec::fb::RigExecWireFile> file =
        RigExecTestUnpack(bytes);
    if (!file) {
        return {};
    }
    edit(file.get());
    return RigExecTestPackUnchecked(*file);
}

// Calls \p visit with a pointer to every input slot id \p file holds: the
// walk of every read, the property chains' targets, the phased consumers'
// consumers and hops, and the chain base, layout, painted and oracle slot
// fields that name one.
template <class Visit>
static void
RigExecTestForEachSlotId(rigExec::fb::RigExecWireFile *file,
                         const Visit &visit)
{
    namespace fb = rigExec::fb;
    const auto read = [&](fb::RigExecWireInput *input) {
        if (input) {
            for (uint32_t &slot : input->walk) {
                visit(&slot);
            }
        }
    };
    const auto reads =
        [&](std::initializer_list<std::unique_ptr<fb::RigExecWireInput> *>
                inputs) {
            for (std::unique_ptr<fb::RigExecWireInput> *input : inputs) {
                read(input->get());
            }
        };
    const auto field = [&](int32_t *slot) {
        if (*slot >= 0) {
            uint32_t id = uint32_t(*slot);
            visit(&id);
            *slot = int32_t(id);
        }
    };
    fb::RigExecWireDomainPose &pose = *file->pose;
    for (fb::RigExecWireLadder &ladder : pose.ladders) {
        reads({&ladder.restSpace, &ladder.defaultSpace, &ladder.posedSpace,
               &ladder.rotationOrder});
        for (fb::RigExecWireInput &input : ladder.restAvars) {
            read(&input);
        }
        for (fb::RigExecWireInput &input : ladder.defaultAvars) {
            read(&input);
        }
    }
    for (fb::RigExecWirePoseInterpolator &interp : pose.poseInterpolators) {
        reads({&interp.enabled});
        for (fb::RigExecWireInput &input : interp.valueInputs) {
            read(&input);
        }
    }
    for (fb::RigExecWireSolver &s : pose.solvers) {
        reads({&s.bend, &s.upperOffset, &s.lowerOffset, &s.stretch,
               &s.softness, &s.ikSpace, &s.blendWeight, &s.preserveVolume,
               &s.midFollowWeight, &s.roll, &s.twist, &s.minLengthRatio,
               &s.twistTurns, &s.ribbonSampleCount});
    }
    for (fb::RigExecWireConstraint &c : pose.constraints) {
        reads({&c.enabled, &c.defaultWeight, &c.offset, &c.affectX,
               &c.affectY, &c.affectZ, &c.tX, &c.tY, &c.tZ, &c.rX, &c.rY,
               &c.rZ, &c.sX, &c.sY, &c.sZ, &c.aimVector, &c.upVector,
               &c.rotationOffset, &c.worldUpVector, &c.poleVector,
               &c.twistDegrees});
    }
    for (fb::RigExecWireSpaceSwitch &sw : pose.spaceSwitches) {
        reads({&sw.active});
    }
    for (fb::RigExecWireAvarBinding &binding : pose.avarBindings) {
        reads({&binding.read});
    }
    const auto revision = [&](fb::RigExecWireRevision &r) {
        reads({&r.defaultWeight});
        field(&r.jointIndicesSlot);
        field(&r.jointWeightsSlot);
        for (fb::RigExecWireBlendChannel &channel : r.blendChannels) {
            reads({&channel.weightRead});
            for (fb::RigExecWireBlendSample &sample : channel.samples) {
                reads({&sample.activationRead, &sample.pointsRead});
            }
        }
    };
    fb::RigExecWireDomainGeometry &geometry = *file->geometry;
    for (fb::RigExecWireChain &chain : geometry.chains) {
        field(&chain.baseSlot);
        for (fb::RigExecWireRevision &r : chain.revisions) {
            revision(r);
        }
        for (fb::RigExecWireDerived &derived : chain.derived) {
            revision(*derived.revision);
        }
    }
    for (fb::RigExecWireWeightObject &w : geometry.weightObjects) {
        reads({&w.defaultWeight, &w.driver, &w.scale, &w.bias, &w.strength,
               &w.invert, &w.falloffMin, &w.falloffMax, &w.scaleXPos,
               &w.scaleYPos, &w.scaleZPos, &w.scaleXNeg, &w.scaleYNeg,
               &w.scaleZNeg, &w.scaleX, &w.scaleY, &w.scaleZ, &w.extentU,
               &w.extentV});
        field(&w.valuesSlot);
        field(&w.indicesSlot);
        field(&w.oracleSamplesSlot);
        field(&w.oracleCurveSlot);
    }
    for (fb::RigExecWirePathRead &row : geometry.pathReads) {
        reads({&row.read});
    }
    for (fb::RigExecWirePropertyChain &chain : file->propertyChains) {
        visit(&chain.target);
        for (fb::RigExecWirePropertyRevision &r : chain.revisions) {
            reads({&r.enabled, &r.defaultWeight, &r.value, &r.min, &r.max});
        }
    }
    for (fb::RigExecWirePhasedConsumer &phased : file->phasedConsumers) {
        visit(&phased.consumer);
        for (uint32_t &hop : phased.hops) {
            visit(&hop);
        }
    }
    for (fb::RigExecWireExternalMover &mover : file->externalMovers) {
        for (fb::RigExecWireInput &input : mover.inputs) {
            read(&input);
        }
    }
}

// The path id of \p text ("/A/B" or "/A/B.attr") in \p file, adding the
// names and nodes it lacks at the ends of their tables.
static uint32_t
RigExecTestPathId(rigExec::fb::RigExecWireFile *file, const std::string &text)
{
    namespace fb = rigExec::fb;
    const auto nameId = [&](const std::string &name) {
        for (size_t n = 0; n < file->names.size(); ++n) {
            if (file->names[n] == name) {
                return uint32_t(n);
            }
        }
        file->names.push_back(name);
        return uint32_t(file->names.size() - 1);
    };
    const auto child = [&](uint32_t parent, const std::string &name,
                           fb::PathKind kind) {
        const uint32_t id = nameId(name);
        for (size_t p = 1; p < file->paths.size(); ++p) {
            const fb::PathNode &node = file->paths[p];
            if (node.parent() == parent && node.name() == id &&
                node.kind() == kind) {
                return uint32_t(p);
            }
        }
        file->paths.push_back(fb::PathNode(parent, id, kind));
        return uint32_t(file->paths.size() - 1);
    };
    const size_t dot = text.find('.');
    const std::string prims = text.substr(0, dot);
    uint32_t at = 0;
    for (size_t begin = 1; begin < prims.size();) {
        const size_t end = std::min(prims.find('/', begin), prims.size());
        at = child(at, prims.substr(begin, end - begin), fb::PathKind::Prim);
        begin = end + 1;
    }
    if (dot != std::string::npos) {
        at = child(at, text.substr(dot + 1), fb::PathKind::Property);
    }
    return at;
}

// The input slot of attribute \p text in \p file, or -1.
static int64_t
RigExecTestSlotOf(const rigExec::fb::RigExecWireFile &file,
                  const std::string &text)
{
    for (size_t s = 0; s < file.inputs.size(); ++s) {
        if (rigExec::RigExecFormatPathText(file, file.inputs[s].name()) ==
            text) {
            return int64_t(s);
        }
    }
    return -1;
}

// Appends \p value to \p file's values; its id.
static uint32_t
RigExecTestAddValue(rigExec::fb::RigExecWireFile *file,
                    rigExec::fb::RigExecWireValue value)
{
    file->values.push_back(std::move(value));
    return uint32_t(file->values.size() - 1);
}

// An array value of \p tag: \p id in the source \p source names.
static rigExec::fb::RigExecWireValue
RigExecTestArrayValue(rigExec::fb::InputTag tag, uint32_t id,
                      rigExec::fb::ArraySource source =
                          rigExec::fb::ArraySource::Pool)
{
    rigExec::fb::RigExecWireValue value;
    value.tag = tag;
    value.arraySource = source;
    value.array = id;
    return value;
}

// The path-read row of attribute \p text at the rest time when \p rest, or
// null.
static rigExec::fb::RigExecWirePathRead *
RigExecTestPathReadRow(rigExec::fb::RigExecWireFile *file,
                       const std::string &text, bool rest)
{
    for (rigExec::fb::RigExecWirePathRead &row : file->geometry->pathReads) {
        if (row.rest == rest &&
            rigExec::RigExecFormatPathText(*file, row.path) == text) {
            return &row;
        }
    }
    return nullptr;
}

// Adds a listed input slot for attribute \p text, of tag \p tag and
// default \p value (a values[] id), with \p flags besides Listed, at its
// place in the path order; every slot id at or past it moves up one. The
// new slot's id.
static uint32_t
RigExecTestAddListedSlot(rigExec::fb::RigExecWireFile *file,
                         const std::string &text, rigExec::fb::InputTag tag,
                         uint32_t value, uint8_t flags)
{
    namespace fb = rigExec::fb;
    const uint32_t path = RigExecTestPathId(file, text);
    uint32_t at = 0;
    while (at < file->listedInputs &&
           rigExec::RigExecFormatPathText(*file, file->inputs[at].name()) <
               text) {
        ++at;
    }
    RigExecTestForEachSlotId(file, [at](uint32_t *slot) {
        if (*slot >= at) {
            ++*slot;
        }
    });
    file->inputs.insert(
        file->inputs.begin() + std::ptrdiff_t(at),
        fb::InputSlot(path, value, -1, -1, tag,
                      uint8_t(flags | uint8_t(fb::InputSlotFlags::Listed))));
    ++file->listedInputs;
    return at;
}

#endif  // RIGEXEC_TESTS_FILE_EDIT_H
