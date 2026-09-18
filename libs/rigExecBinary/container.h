//
// .rigexec sectioned container: the reader and writer every bake producer and
// runtime consumer shares.
//
// A .rigexec file is a flat little-endian binary: a 16-byte header, a section
// table, then the section payloads. Unknown section tags are skipped, so a
// minor version can add tables without breaking old loaders. All offsets are
// u64, all counts u32.
//
// This header is pleasure-free on purpose: <cstdint>, <string>, <vector> and
// <memory> only. It is the foundation of the zero-USD runtime (M2), so it
// must never gain a USD include -- directly or through another project
// header. The bake side (libs/rigExecBake) is what knows about USD.
//
#ifndef RIGEXEC_BINARY_CONTAINER_H
#define RIGEXEC_BINARY_CONTAINER_H

#include <cstdint>
#include <memory>
#include <string>
#include <vector>

namespace rigExec {

/// Magic bytes "REXB" as a little-endian u32.
inline constexpr uint32_t RigExecBinaryMagic = 0x42584552u;

/// The container version this code writes: major 1, minor 0.
/// Encoded (minor << 16) | major; the reader requires the major and
/// tolerates the minor.
inline constexpr uint32_t RigExecBinaryVersion = 0x00000001u;
inline constexpr uint32_t RigExecBinaryMajor(uint32_t version)
{
    return version & 0xffffu;
}

/// Section tags. The list is the format's public vocabulary: tags are never
/// reused, and a reader skips the ones it does not know.
enum class RigExecBinarySection : uint32_t {
    StringTable = 1,  ///< string pool; every name in the file is an index
    Manifest = 2,     ///< JSON manifest text (see bake.h)
    SlotMeta = 3,     ///< slot-domain inventory (M1 slice 2)
    Constants = 4,    ///< epoch constant buffer (M1 slice 2)
    InputTable = 5,   ///< per-frame varying inputs (M1 slice 4)
    DomainPose = 6,   ///< pose-domain build tables (M1 slice 2)
    DomainGeometry = 7,  ///< geometry-domain build tables (M1 slice 3)
    Steps = 8,        ///< the step list (M1 slice 2)
    Clusters = 9,     ///< cluster partition (M1 slice 2)
    Cones = 10,       ///< cone closures (M1 slice 4)
    Diagnostics = 11,  ///< embedded diagnostics; the loader skips it
};

/// Builds a .rigexec file in memory.
class RigExecBinaryWriter {
public:
    RigExecBinaryWriter() = default;

    RigExecBinaryWriter(const RigExecBinaryWriter &) = delete;
    RigExecBinaryWriter &operator=(const RigExecBinaryWriter &) = delete;

    /// Interns \p text, returning its string-table index. Equal strings share
    /// one index; the empty string is always index 0.
    uint32_t AddString(const std::string &text);

    /// Queues one section payload. Tags must be unique per file.
    void AddSection(RigExecBinarySection tag, const uint8_t *data,
                    size_t size);
    void AddSection(RigExecBinarySection tag,
                    const std::vector<uint8_t> &bytes);

    /// Serializes the file: header, section table (string table first, then
    /// the queued sections in order), then the payloads.
    std::vector<uint8_t> Finish();

private:
    // Index 0 is "" from construction: AddString hands out final indices,
    // so nothing Finish does may shift them.
    std::vector<std::string> _strings = {std::string()};
    struct _Section {
        RigExecBinarySection tag;
        std::vector<uint8_t> bytes;
    };
    std::vector<_Section> _sections;
};

/// Reads a .rigexec file. Open copies the bytes, so the caller keeps no
/// lifetime obligation past the call.
class RigExecBinaryReader {
public:
    ~RigExecBinaryReader() = default;

    RigExecBinaryReader(const RigExecBinaryReader &) = delete;
    RigExecBinaryReader &operator=(const RigExecBinaryReader &) = delete;

    /// Opens \p bytes, or returns null with the reason: a bad magic, a
    /// truncated header or section table, a section running past the end,
    /// overlapping sections, a duplicate tag, or a malformed string table.
    /// A major-version mismatch is a refusal, not a truncation: the file is
    /// well-formed but unreadable by this reader.
    static std::unique_ptr<RigExecBinaryReader> Open(const uint8_t *bytes,
                                                     size_t size,
                                                     std::string *error);

    uint32_t GetVersion() const { return _version; }
    uint32_t GetFlags() const { return _flags; }

    /// Locates \p tag's payload. False when the file carries no such
    /// section; the pointers borrow from the reader.
    bool FindSection(RigExecBinarySection tag, const uint8_t **data,
                     size_t *size) const;

    /// The string-table entry at \p index. False for an out-of-range index.
    bool GetString(uint32_t index, std::string *out) const;

private:
    RigExecBinaryReader() = default;

    uint32_t _version = 0;
    uint32_t _flags = 0;
    std::vector<uint8_t> _bytes;
    struct _Span {
        RigExecBinarySection tag;
        size_t offset = 0;
        size_t size = 0;
    };
    std::vector<_Span> _sections;
    std::vector<uint32_t> _stringOffsets;
    size_t _stringCharsBegin = 0;
    size_t _stringCharsEnd = 0;
};

}  // namespace rigExec

#endif  // RIGEXEC_BINARY_CONTAINER_H
