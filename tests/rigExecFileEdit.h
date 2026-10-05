// Runtime-suite helper: a baked .rigexec unpacked into its object form,
// edited, and packed again without validation, so a defect the format's
// validator or the runtime refuses reaches Open.
// Included AFTER the suite's CHECK macro, which reports through it.
#ifndef RIGEXEC_TESTS_FILE_EDIT_H
#define RIGEXEC_TESTS_FILE_EDIT_H

#include "rigExecBinary/format.h"

#include <cstdint>
#include <cstdio>
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

#endif  // RIGEXEC_TESTS_FILE_EDIT_H
