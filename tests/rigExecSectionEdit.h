// Runtime-suite helper: a baked .rigexec with one section replaced, so a
// test can cut a rewritten table into a real bake.
// Included AFTER the suite's CHECK macro, which reports through it.
#ifndef RIGEXEC_TESTS_SECTION_EDIT_H
#define RIGEXEC_TESTS_SECTION_EDIT_H

#include "rigExecBinary/container.h"

#include <cstddef>
#include <cstdint>
#include <memory>
#include <string>
#include <vector>

// \p bytes with section \p tag replaced by \p payload; the string table and
// every other section are copied unchanged.
static std::vector<uint8_t>
_ReplaceSection(const std::vector<uint8_t> &bytes,
                rigExec::RigExecBinarySection tag,
                const std::vector<uint8_t> &payload)
{
    using rigExec::RigExecBinarySection;
    std::string error;
    const std::unique_ptr<rigExec::RigExecBinaryReader> reader =
        rigExec::RigExecBinaryReader::Open(bytes.data(), bytes.size(),
                                           &error);
    CHECK(reader);
    if (!reader) {
        return {};
    }
    rigExec::RigExecBinaryWriter writer;
    std::string text;
    for (uint32_t id = 1; reader->GetString(id, &text); ++id) {
        CHECK(writer.AddString(text) == id);
    }
    for (uint32_t t = uint32_t(RigExecBinarySection::Manifest);
         t <= uint32_t(RigExecBinarySection::Computed); ++t) {
        const RigExecBinarySection section = RigExecBinarySection(t);
        const uint8_t *data = nullptr;
        size_t size = 0;
        if (!reader->FindSection(section, &data, &size)) {
            continue;
        }
        if (section == tag) {
            writer.AddSection(section, payload);
        } else {
            writer.AddSection(section, data, size);
        }
    }
    return writer.Finish();
}

#endif  // RIGEXEC_TESTS_SECTION_EDIT_H
