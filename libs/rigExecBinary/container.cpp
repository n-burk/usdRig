//
// .rigexec sectioned container, writer and reader.
//

#include "rigExecBinary/container.h"

#include <cstring>

namespace rigExec {
namespace {

constexpr size_t _HeaderSize = 16;
constexpr size_t _SectionEntrySize = 20;

void
_AppendU32(std::vector<uint8_t> *out, uint32_t value)
{
    out->push_back(uint8_t(value & 0xffu));
    out->push_back(uint8_t((value >> 8) & 0xffu));
    out->push_back(uint8_t((value >> 16) & 0xffu));
    out->push_back(uint8_t((value >> 24) & 0xffu));
}

void
_AppendU64(std::vector<uint8_t> *out, uint64_t value)
{
    for (int shift = 0; shift < 64; shift += 8) {
        out->push_back(uint8_t((value >> shift) & 0xffu));
    }
}

uint32_t
_ReadU32(const uint8_t *bytes)
{
    return uint32_t(bytes[0]) | (uint32_t(bytes[1]) << 8) |
           (uint32_t(bytes[2]) << 16) | (uint32_t(bytes[3]) << 24);
}

uint64_t
_ReadU64(const uint8_t *bytes)
{
    uint64_t value = 0;
    for (int i = 7; i >= 0; --i) {
        value = (value << 8) | bytes[i];
    }
    return value;
}

}  // namespace

uint32_t
RigExecBinaryWriter::AddString(const std::string &text)
{
    // Linear, because the table is hundreds of entries at most and this runs
    // once per bake: a map would buy nothing and cost an include.
    for (size_t i = 0; i < _strings.size(); ++i) {
        if (_strings[i] == text) {
            return uint32_t(i);
        }
    }
    _strings.push_back(text);
    return uint32_t(_strings.size() - 1);
}

void
RigExecBinaryWriter::AddSection(RigExecBinarySection tag, const uint8_t *data,
                               size_t size)
{
    _Section section;
    section.tag = tag;
    if (data && size) {
        section.bytes.assign(data, data + size);
    }
    _sections.push_back(std::move(section));
}

void
RigExecBinaryWriter::AddSection(RigExecBinarySection tag,
                               const std::vector<uint8_t> &bytes)
{
    AddSection(tag, bytes.empty() ? nullptr : bytes.data(), bytes.size());
}

std::vector<uint8_t>
RigExecBinaryWriter::Finish()
{
    // Index 0 is "" by construction (see the header): AddString hands out
    // final indices, so serializing the table shifts nothing.
    std::vector<uint8_t> table;
    _AppendU32(&table, uint32_t(_strings.size()));
    uint32_t at = 0;
    for (const std::string &text : _strings) {
        _AppendU32(&table, at);
        at += uint32_t(text.size()) + 1;
    }
    for (const std::string &text : _strings) {
        table.insert(table.end(), text.begin(), text.end());
        table.push_back(0);
    }

    const uint32_t sectionCount = uint32_t(_sections.size()) + 1;
    std::vector<uint8_t> out;
    _AppendU32(&out, RigExecBinaryMagic);
    _AppendU32(&out, RigExecBinaryVersion);
    _AppendU32(&out, sectionCount);
    _AppendU32(&out, /* flags = */ 0);

    size_t payloadAt =
        _HeaderSize + size_t(sectionCount) * _SectionEntrySize;
    _AppendU32(&out, uint32_t(RigExecBinarySection::StringTable));
    _AppendU64(&out, payloadAt);
    _AppendU64(&out, table.size());
    payloadAt += table.size();
    for (const _Section &section : _sections) {
        _AppendU32(&out, uint32_t(section.tag));
        _AppendU64(&out, payloadAt);
        _AppendU64(&out, section.bytes.size());
        payloadAt += section.bytes.size();
    }
    out.insert(out.end(), table.begin(), table.end());
    for (const _Section &section : _sections) {
        out.insert(out.end(), section.bytes.begin(), section.bytes.end());
    }
    return out;
}

std::unique_ptr<RigExecBinaryReader>
RigExecBinaryReader::Open(const uint8_t *bytes, size_t size,
                          std::string *error)
{
    auto Fail = [&](const std::string &what) {
        if (error) {
            *error = what;
        }
        return std::unique_ptr<RigExecBinaryReader>();
    };
    if (!bytes || size < _HeaderSize) {
        return Fail("not a .rigexec file: truncated header");
    }
    if (_ReadU32(bytes) != RigExecBinaryMagic) {
        return Fail("not a .rigexec file: bad magic");
    }
    const uint32_t version = _ReadU32(bytes + 4);
    if (RigExecBinaryMajor(version) !=
        RigExecBinaryMajor(RigExecBinaryVersion)) {
        return Fail("unsupported .rigexec major version");
    }
    const uint32_t sectionCount = _ReadU32(bytes + 8);
    const uint32_t flags = _ReadU32(bytes + 12);
    const size_t tableEnd =
        _HeaderSize + size_t(sectionCount) * _SectionEntrySize;
    if (tableEnd > size) {
        return Fail("not a .rigexec file: truncated section table");
    }
    std::unique_ptr<RigExecBinaryReader> reader(new RigExecBinaryReader());
    reader->_version = version;
    reader->_flags = flags;
    reader->_bytes.assign(bytes, bytes + size);
    for (uint32_t i = 0; i < sectionCount; ++i) {
        const uint8_t *entry = bytes + _HeaderSize + i * _SectionEntrySize;
        const uint64_t offset = _ReadU64(entry + 4);
        const uint64_t sectionSize = _ReadU64(entry + 12);
        if (offset > size || sectionSize > size - offset) {
            return Fail("not a .rigexec file: section runs past the end");
        }
        const auto tag = RigExecBinarySection(_ReadU32(entry));
        for (const _Span &seen : reader->_sections) {
            if (seen.tag == tag) {
                return Fail("not a .rigexec file: duplicate section tag");
            }
            // Overlaps are rejected rather than tolerated: two sections
            // sharing a byte is never a well-formed file, and silently
            // accepting one would let a corrupt offset hide behind a valid
            // neighbour.
            const size_t a0 = seen.offset;
            const size_t a1 = seen.offset + seen.size;
            const size_t b0 = size_t(offset);
            const size_t b1 = b0 + size_t(sectionSize);
            if (a0 < b1 && b0 < a1) {
                return Fail("not a .rigexec file: overlapping sections");
            }
        }
        _Span span;
        span.tag = tag;
        span.offset = size_t(offset);
        span.size = size_t(sectionSize);
        reader->_sections.push_back(span);
    }
    // The string table is mandatory: every name in every later section is an
    // index into it, so a file without one has nothing to say.
    const uint8_t *tableData = nullptr;
    size_t tableSize = 0;
    if (!reader->FindSection(RigExecBinarySection::StringTable, &tableData,
                             &tableSize) ||
        tableSize < 4) {
        return Fail("not a .rigexec file: missing string table");
    }
    const uint32_t stringCount = _ReadU32(tableData);
    if (tableSize < 4 + size_t(stringCount) * 4) {
        return Fail("not a .rigexec file: truncated string table");
    }
    reader->_stringCharsBegin =
        size_t(tableData - reader->_bytes.data()) + 4 +
        size_t(stringCount) * 4;
    reader->_stringCharsEnd =
        size_t(tableData - reader->_bytes.data()) + tableSize;
    for (uint32_t i = 0; i < stringCount; ++i) {
        const uint32_t at = _ReadU32(tableData + 4 + i * 4);
        const size_t charsSize =
            reader->_stringCharsEnd - reader->_stringCharsBegin;
        if (at > charsSize) {
            return Fail("not a .rigexec file: string offset out of range");
        }
        // Every entry is NUL-terminated inside the table: an unterminated
        // entry would read into the next payload.
        const uint8_t *chars =
            reader->_bytes.data() + reader->_stringCharsBegin;
        size_t end = at;
        while (end < charsSize && chars[end] != 0) {
            ++end;
        }
        if (end >= charsSize) {
            return Fail("not a .rigexec file: unterminated string");
        }
        reader->_stringOffsets.push_back(at);
    }
    if (!reader->_stringOffsets.empty()) {
        std::string first;
        reader->GetString(0, &first);
        if (!first.empty()) {
            return Fail("not a .rigexec file: string 0 is not empty");
        }
    }
    return reader;
}

bool
RigExecBinaryReader::FindSection(RigExecBinarySection tag,
                                const uint8_t **data, size_t *size) const
{
    for (const _Span &span : _sections) {
        if (span.tag == tag) {
            if (data) {
                *data = _bytes.data() + span.offset;
            }
            if (size) {
                *size = span.size;
            }
            return true;
        }
    }
    return false;
}

bool
RigExecBinaryReader::GetString(uint32_t index, std::string *out) const
{
    if (index >= _stringOffsets.size()) {
        return false;
    }
    const uint8_t *chars = _bytes.data() + _stringCharsBegin;
    const char *text =
        reinterpret_cast<const char *>(chars + _stringOffsets[index]);
    if (out) {
        *out = std::string(text);
    }
    return true;
}

}  // namespace rigExec
